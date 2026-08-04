#include "rendering/m2_renderer.hpp"
#include "rendering/m2_renderer_internal.h"
#include "core/thread_pool.hpp"
#include "rendering/m2_model_classifier.hpp"
#include "rendering/hiz_system.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/vk_buffer.hpp"
#include "rendering/vk_texture.hpp"
#include "rendering/vk_pipeline.hpp"
#include "rendering/vk_shader.hpp"
#include "rendering/vk_utils.hpp"
#include "rendering/vk_frame_data.hpp"
#include "rendering/camera.hpp"
#include "rendering/frustum.hpp"
#include "rendering/render_constants.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/blp_loader.hpp"
#include "core/logger.hpp"
#include "core/profiler.hpp"
#include <chrono>
#include <cctype>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>
#include <unordered_set>
#include <functional>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <random>
#include <limits>
#include <future>
#include <thread>

namespace wowee {
namespace rendering {

uint32_t M2Renderer::createInstance(uint32_t modelId, const glm::vec3& position,
                                     const glm::vec3& rotation, float scale,
                                     bool allowPositionDedup) {
    // Reject NaN inputs at the boundary — std::round of NaN is implementation-
    // defined and a NaN instance position propagates into the GPU model matrix,
    // either tripping Vulkan validation or rendering at the world origin.
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z) || !std::isfinite(rotation.x) ||
        !std::isfinite(rotation.y) || !std::isfinite(rotation.z) ||
        !std::isfinite(scale) || scale <= 0.0f) {
        return 0;
    }
    auto modelIt = models.find(modelId);
    if (modelIt == models.end()) {
        LOG_WARNING("Cannot create instance: model ", modelId, " not loaded");
        return 0;
    }
    const auto& mdlRef = modelIt->second;
    modelUnusedSince_.erase(modelId);


    // Deduplicate: skip if same model already at nearly the same position.
    // Uses hash map for O(1) lookup instead of O(N) scan.
    // Spell effects are exempt — transient visuals must always create fresh instances.
    if (allowPositionDedup && !mdlRef.isGroundDetail && !mdlRef.isSpellEffect) {
        DedupKey dk{modelId,
                    static_cast<int32_t>(std::round(position.x * 10.0f)),
                    static_cast<int32_t>(std::round(position.y * 10.0f)),
                    static_cast<int32_t>(std::round(position.z * 10.0f))};
        auto dit = instanceDedupMap_.find(dk);
        if (dit != instanceDedupMap_.end()) {
            return dit->second;
        }
    }

    M2Instance instance;
    instance.id = nextInstanceId++;
    instance.modelId = modelId;
    instance.position = position;
    if (mdlRef.isGroundDetail) {
        instance.position.z -= computeGroundDetailDownOffset(mdlRef, scale);
    }
    instance.rotation = rotation;
    instance.scale = scale;
    instance.updateModelMatrix();
    glm::vec3 localMin, localMax;
    getTightCollisionBounds(mdlRef, localMin, localMax);
    transformAABB(instance.modelMatrix, localMin, localMax, instance.worldBoundsMin, instance.worldBoundsMax);

    // Cache model flags on instance to avoid per-frame hash lookups
    instance.cachedHasAnimation = mdlRef.hasAnimation;
    instance.cachedDisableAnimation = mdlRef.disableAnimation;
    instance.cachedIsSmoke = mdlRef.isSmoke;
    instance.cachedHasParticleEmitters = !mdlRef.particleEmitters.empty();
    instance.cachedBoundRadius = mdlRef.boundRadius;
    instance.cachedIsGroundDetail = mdlRef.isGroundDetail;
    instance.cachedIsInvisibleTrap = mdlRef.isInvisibleTrap;
    instance.cachedIsInstancePortal = mdlRef.isInstancePortal;
    instance.cachedIsSkyBird = mdlRef.isSkyBird;
    instance.cachedIsLightBeam = mdlRef.isLightBeam;
    instance.cachedIsTransportDoodad = mdlRef.isTransportDoodad;
    instance.cachedIsValid = mdlRef.isValid();
    instance.cachedModel = &mdlRef;
    instance.recomputeCachedCullFactors();

    // Initialize animation: play first sequence (usually Stand/Idle)
    const auto& mdl = mdlRef;
    if (mdl.hasAnimation && !mdl.disableAnimation) {
        if (!mdl.sequences.empty()) {
            instance.currentSequenceIndex = 0;
            instance.idleSequenceIndex = 0;
            instance.animDuration = static_cast<float>(mdl.sequences[0].duration);
            instance.animTime = static_cast<float>(randRange(std::max(1u, mdl.sequences[0].duration)));
            instance.variationTimer = randFloat(rendering::M2_VARIATION_TIMER_MIN_MS, rendering::M2_VARIATION_TIMER_MAX_MS);
        }

        // Seed bone matrices from an existing instance of the same model so the
        // new instance renders immediately instead of being invisible until the
        // next update() computes bones (prevents pop-in flash).
        auto seedIt = boneSeedInstanceByModel_.find(modelId);
        if (seedIt != boneSeedInstanceByModel_.end()) {
            auto idxIt = instanceIndexById.find(seedIt->second);
            if (idxIt != instanceIndexById.end() && idxIt->second < instances.size()) {
                const auto& existing = instances[idxIt->second];
                if (existing.modelId == modelId && !existing.boneMatrices.empty()) {
                    instance.boneMatrices = existing.boneMatrices;
                    instance.bonesDirty[0] = instance.bonesDirty[1] = true;
                } else {
                    boneSeedInstanceByModel_.erase(seedIt);  // stale entry
                }
            } else {
                boneSeedInstanceByModel_.erase(seedIt);  // that instance is gone
            }
        }
        // If no sibling exists yet, compute bones immediately
        if (instance.boneMatrices.empty()) {
            computeBoneMatrices(mdlRef, instance);
        }
        if (!instance.boneMatrices.empty()) {
            boneSeedInstanceByModel_.emplace(modelId, instance.id);
        }
    }

    // Register in dedup map before pushing (uses original position, not ground-adjusted)
    // Spell effects are exempt from dedup tracking (transient, overlapping allowed).
    if (allowPositionDedup && !mdlRef.isGroundDetail && !mdlRef.isSpellEffect) {
        DedupKey dk{modelId,
                    static_cast<int32_t>(std::round(position.x * 10.0f)),
                    static_cast<int32_t>(std::round(position.y * 10.0f)),
                    static_cast<int32_t>(std::round(position.z * 10.0f))};
        instanceDedupMap_[dk] = instance.id;
    }

    instances.push_back(instance);
    size_t idx = instances.size() - 1;
    // Track special instances for fast-path iteration
    if (mdlRef.isSmoke) {
        smokeInstanceIndices_.push_back(idx);
    }
    if (mdlRef.isInstancePortal) {
        portalInstanceIndices_.push_back(idx);
    }
    if (!mdlRef.particleEmitters.empty()) {
        particleInstanceIndices_.push_back(idx);
    }
    if (mdlRef.hasAnimation && !mdlRef.disableAnimation) {
        animatedInstanceIndices_.push_back(idx);
    } else if (!mdlRef.particleEmitters.empty()) {
        particleOnlyInstanceIndices_.push_back(idx);
    }
    instanceIndexById[instance.id] = idx;
    GridCell minCell = toCell(instance.worldBoundsMin);
    GridCell maxCell = toCell(instance.worldBoundsMax);
    for (int z = minCell.z; z <= maxCell.z; z++) {
        for (int y = minCell.y; y <= maxCell.y; y++) {
            for (int x = minCell.x; x <= maxCell.x; x++) {
                spatialGrid[GridCell{x, y, z}].push_back(instance.id);
            }
        }
    }

    return instance.id;
}

uint32_t M2Renderer::createInstanceWithMatrix(uint32_t modelId, const glm::mat4& modelMatrix,
                                                const glm::vec3& position) {
    // Reject NaN inputs at the boundary. position feeds the dedup hash
    // (std::round of NaN is implementation-defined); the matrix goes
    // straight to the GPU UBO and would crash validation.
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z)) {
        return 0;
    }
    for (int c = 0; c < 4; c++)
        for (int r = 0; r < 4; r++)
            if (!std::isfinite(modelMatrix[c][r])) return 0;
    if (models.find(modelId) == models.end()) {
        LOG_WARNING("Cannot create instance: model ", modelId, " not loaded");
        return 0;
    }
    modelUnusedSince_.erase(modelId);

    // Deduplicate: O(1) hash lookup
    {
        DedupKey dk{modelId,
                    static_cast<int32_t>(std::round(position.x * 10.0f)),
                    static_cast<int32_t>(std::round(position.y * 10.0f)),
                    static_cast<int32_t>(std::round(position.z * 10.0f))};
        auto dit = instanceDedupMap_.find(dk);
        if (dit != instanceDedupMap_.end()) {
            return dit->second;
        }
    }

    M2Instance instance;
    instance.id = nextInstanceId++;
    instance.modelId = modelId;
    instance.position = position;  // Used for frustum culling
    instance.rotation = glm::vec3(0.0f);
    instance.scale = 1.0f;
    instance.modelMatrix = modelMatrix;
    instance.invModelMatrix = glm::inverse(modelMatrix);
    glm::vec3 localMin, localMax;
    getTightCollisionBounds(models[modelId], localMin, localMax);
    transformAABB(instance.modelMatrix, localMin, localMax, instance.worldBoundsMin, instance.worldBoundsMax);
    // Cache model flags on instance to avoid per-frame hash lookups
    const auto& mdl2 = models[modelId];
    instance.cachedHasAnimation = mdl2.hasAnimation;
    instance.cachedDisableAnimation = mdl2.disableAnimation;
    instance.cachedIsSmoke = mdl2.isSmoke;
    instance.cachedHasParticleEmitters = !mdl2.particleEmitters.empty();
    instance.cachedBoundRadius = mdl2.boundRadius;
    instance.cachedIsGroundDetail = mdl2.isGroundDetail;
    instance.cachedIsInvisibleTrap = mdl2.isInvisibleTrap;
    instance.cachedIsSkyBird = mdl2.isSkyBird;
    instance.cachedIsLightBeam = mdl2.isLightBeam;
    instance.cachedIsTransportDoodad = mdl2.isTransportDoodad;
    instance.cachedIsValid = mdl2.isValid();
    instance.cachedModel = &mdl2;
    instance.recomputeCachedCullFactors();

    // Initialize animation
    if (mdl2.hasAnimation && !mdl2.disableAnimation) {
        if (!mdl2.sequences.empty()) {
            instance.currentSequenceIndex = 0;
            instance.idleSequenceIndex = 0;
            instance.animDuration = static_cast<float>(mdl2.sequences[0].duration);
            instance.animTime = static_cast<float>(randRange(std::max(1u, mdl2.sequences[0].duration)));
            instance.variationTimer = randFloat(rendering::M2_VARIATION_TIMER_MIN_MS, rendering::M2_VARIATION_TIMER_MAX_MS);
        }

        // Seed bone matrices from an existing sibling so the instance renders immediately
        auto seedIt = boneSeedInstanceByModel_.find(modelId);
        if (seedIt != boneSeedInstanceByModel_.end()) {
            auto idxIt = instanceIndexById.find(seedIt->second);
            if (idxIt != instanceIndexById.end() && idxIt->second < instances.size()) {
                const auto& existing = instances[idxIt->second];
                if (existing.modelId == modelId && !existing.boneMatrices.empty()) {
                    instance.boneMatrices = existing.boneMatrices;
                    instance.bonesDirty[0] = instance.bonesDirty[1] = true;
                } else {
                    boneSeedInstanceByModel_.erase(seedIt);  // stale entry
                }
            } else {
                boneSeedInstanceByModel_.erase(seedIt);  // that instance is gone
            }
        }
        if (instance.boneMatrices.empty()) {
            computeBoneMatrices(mdl2, instance);
        }
        if (!instance.boneMatrices.empty()) {
            boneSeedInstanceByModel_.emplace(modelId, instance.id);
        }
    } else {
        instance.animTime = randFloat(0.0f, 10000.0f);
    }

    // Register in dedup map
    {
        DedupKey dk{modelId,
                    static_cast<int32_t>(std::round(position.x * 10.0f)),
                    static_cast<int32_t>(std::round(position.y * 10.0f)),
                    static_cast<int32_t>(std::round(position.z * 10.0f))};
        instanceDedupMap_[dk] = instance.id;
    }

    instances.push_back(instance);
    size_t idx = instances.size() - 1;
    if (mdl2.isSmoke) {
        smokeInstanceIndices_.push_back(idx);
    }
    if (!mdl2.particleEmitters.empty()) {
        particleInstanceIndices_.push_back(idx);
    }
    if (mdl2.hasAnimation && !mdl2.disableAnimation) {
        animatedInstanceIndices_.push_back(idx);
    } else if (!mdl2.particleEmitters.empty()) {
        particleOnlyInstanceIndices_.push_back(idx);
    }
    instanceIndexById[instance.id] = idx;
    GridCell minCell = toCell(instance.worldBoundsMin);
    GridCell maxCell = toCell(instance.worldBoundsMax);
    for (int z = minCell.z; z <= maxCell.z; z++) {
        for (int y = minCell.y; y <= maxCell.y; y++) {
            for (int x = minCell.x; x <= maxCell.x; x++) {
                spatialGrid[GridCell{x, y, z}].push_back(instance.id);
            }
        }
    }

    return instance.id;
}

void M2Renderer::update(float deltaTime, const glm::vec3& cameraPos, const glm::mat4& viewProjection) {
    ZoneScopedN("M2Renderer::update");
    if (spatialIndexDirty_) {
        rebuildSpatialIndex();
    }

    float dtMs = deltaTime * 1000.0f;

    // Cache camera state for frustum-culling bone computation
    cachedCamPos_ = cameraPos;
    const float maxRenderDistance = viewDistanceScale_ *
        ((instances.size() > rendering::M2_HIGH_DENSITY_INSTANCE_THRESHOLD)
             ? rendering::M2_MAX_RENDER_DISTANCE_HIGH_DENSITY
             : rendering::M2_MAX_RENDER_DISTANCE_LOW_DENSITY);
    cachedMaxRenderDistSq_ = maxRenderDistance * maxRenderDistance;

    // Build frustum for culling bones
    Frustum updateFrustum;
    updateFrustum.extractFromMatrix(viewProjection);

    // --- Smoke particle spawning (only iterate tracked smoke instances) ---
    std::uniform_real_distribution<float> distXY(rendering::SMOKE_OFFSET_XY_MIN, rendering::SMOKE_OFFSET_XY_MAX);
    std::uniform_real_distribution<float> distVelXY(-0.3f, 0.3f);
    std::uniform_real_distribution<float> distVelZ(rendering::SMOKE_VEL_Z_MIN, rendering::SMOKE_VEL_Z_MAX);
    std::uniform_real_distribution<float> distLife(rendering::SMOKE_LIFETIME_MIN, rendering::SMOKE_LIFETIME_MAX);
    std::uniform_real_distribution<float> distDrift(-0.2f, 0.2f);

    smokeEmitAccum += deltaTime;
    constexpr float emitInterval = kSmokeEmitInterval;  // 48 particles per second per emitter

    if (smokeEmitAccum >= emitInterval &&
        static_cast<int>(smokeParticles.size()) < MAX_SMOKE_PARTICLES) {
        for (size_t si : smokeInstanceIndices_) {
            if (si >= instances.size()) continue;
            auto& instance = instances[si];

            glm::vec3 emitWorld = glm::vec3(instance.modelMatrix * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
            bool spark = (smokeRng() % rendering::SPARK_PROBABILITY_DENOM == 0);

            SmokeParticle p;
            p.position = emitWorld + glm::vec3(distXY(smokeRng), distXY(smokeRng), 0.0f);
            if (spark) {
                p.velocity = glm::vec3(distVelXY(smokeRng) * 2.0f, distVelXY(smokeRng) * 2.0f, distVelZ(smokeRng) * 1.5f);
                p.maxLife = rendering::SPARK_LIFE_BASE + static_cast<float>(smokeRng() % 100) / 100.0f * rendering::SPARK_LIFE_RANGE;
                p.size = 0.5f;
                p.isSpark = 1.0f;
            } else {
                p.velocity = glm::vec3(distVelXY(smokeRng), distVelXY(smokeRng), distVelZ(smokeRng));
                p.maxLife = distLife(smokeRng);
                p.size = 1.0f;
                p.isSpark = 0.0f;
            }
            p.life = 0.0f;
            p.instanceId = instance.id;
            smokeParticles.push_back(p);
            if (static_cast<int>(smokeParticles.size()) >= MAX_SMOKE_PARTICLES) break;
        }
        smokeEmitAccum = 0.0f;
    }

    // --- Update existing smoke particles (swap-and-pop for O(1) removal) ---
    for (size_t i = 0; i < smokeParticles.size(); ) {
        auto& p = smokeParticles[i];
        p.life += deltaTime;
        if (p.life >= p.maxLife) {
            smokeParticles[i] = smokeParticles.back();
            smokeParticles.pop_back();
            continue;
        }
        p.position += p.velocity * deltaTime;
        p.velocity.z *= rendering::SMOKE_Z_VEL_DAMPING;  // Slight deceleration
        p.velocity.x += distDrift(smokeRng) * deltaTime;
        p.velocity.y += distDrift(smokeRng) * deltaTime;
        // Grow from 1.0 to 3.5 over lifetime
        float t = p.life / p.maxLife;
        p.size = rendering::SMOKE_SIZE_START + t * rendering::SMOKE_SIZE_GROWTH;
        ++i;
    }

    // --- Spin instance portals ---
    static constexpr float PORTAL_SPIN_SPEED = 1.2f; // radians/sec
    static constexpr float kTwoPi = 6.2831853f;
    for (size_t idx : portalInstanceIndices_) {
        if (idx >= instances.size()) continue;
        auto& inst = instances[idx];
        inst.portalSpinAngle += PORTAL_SPIN_SPEED * deltaTime;
        if (inst.portalSpinAngle > kTwoPi)
            inst.portalSpinAngle -= kTwoPi;
        inst.rotation.z = inst.portalSpinAngle;
        inst.updateModelMatrix();
    }

    // --- Normal M2 animation update ---
    // Advance animTime for ALL instances (needed for texture UV animation on static doodads).
    // This is a tight loop touching only one float per instance — no hash lookups.
    for (auto& instance : instances) {
        instance.animTime += dtMs;
        instance.globalSequenceTime += dtMs;
    }
    // Wrap animTime for particle-only instances so emission rate tracks keep looping.
    // 3333ms chosen as a safe wrap period: long enough to cover the longest known M2
    // particle emission cycle (~3s for torch/campfire effects) while preventing float
    // precision loss that accumulates over hours of runtime.
    static constexpr float kParticleWrapMs = 3333.0f;
    for (size_t idx : particleOnlyInstanceIndices_) {
        if (idx >= instances.size()) continue;
        auto& instance = instances[idx];
        // Use iterative subtraction instead of fmod() to preserve precision
        while (instance.animTime > kParticleWrapMs) {
            instance.animTime -= kParticleWrapMs;
        }
    }

    boneWorkIndices_.clear();
    boneWorkIndices_.reserve(animatedInstanceIndices_.size());

    // Update animated instances (full animation state + bone computation culling)
    // Note: animTime was already advanced by dtMs in the global loop above.
    // Here we apply the speed factor: subtract the base dtMs and add dtMs*speed.
    for (size_t idx : animatedInstanceIndices_) {
        if (idx >= instances.size()) continue;
        auto& instance = instances[idx];

        instance.animTime += dtMs * (instance.animSpeed - 1.0f);

        // For animation looping/variation, we need the actual model data.
        if (!instance.cachedModel) continue;
        const M2ModelGPU& model = *instance.cachedModel;

        // Validate sequence index
        if (instance.currentSequenceIndex < 0 ||
            instance.currentSequenceIndex >= static_cast<int>(model.sequences.size())) {
            instance.currentSequenceIndex = 0;
            if (!model.sequences.empty()) {
                instance.animDuration = static_cast<float>(model.sequences[0].duration);
            }
        }

        // Handle animation looping / variation transitions
        if (instance.animDuration <= 0.0f && instance.cachedHasParticleEmitters) {
            instance.animDuration = rendering::M2_DEFAULT_PARTICLE_ANIM_MS;
        }
        if (instance.animDuration > 0.0f && instance.animTime >= instance.animDuration) {
            if (instance.playingVariation) {
                instance.playingVariation = false;
                instance.currentSequenceIndex = instance.idleSequenceIndex;
                if (instance.idleSequenceIndex < static_cast<int>(model.sequences.size())) {
                    instance.animDuration = static_cast<float>(model.sequences[instance.idleSequenceIndex].duration);
                }
                instance.animTime = 0.0f;
                instance.variationTimer = randFloat(rendering::M2_LOOP_VARIATION_TIMER_MIN_MS, rendering::M2_LOOP_VARIATION_TIMER_MAX_MS);
            } else {
                // Use iterative subtraction instead of fmod() to preserve precision
                float duration = std::max(1.0f, instance.animDuration);
                while (instance.animTime >= duration) {
                    instance.animTime -= duration;
                }
            }
        }

        // Idle variation timer
        if (!instance.playingVariation && model.idleVariationIndices.size() > 1) {
            instance.variationTimer -= dtMs;
            if (instance.variationTimer <= 0.0f) {
                int pick = static_cast<int>(randRange(static_cast<uint32_t>(model.idleVariationIndices.size())));
                int newSeq = model.idleVariationIndices[pick];
                if (newSeq != instance.currentSequenceIndex && newSeq < static_cast<int>(model.sequences.size())) {
                    instance.playingVariation = true;
                    instance.currentSequenceIndex = newSeq;
                    instance.animDuration = static_cast<float>(model.sequences[newSeq].duration);
                    instance.animTime = 0.0f;
                } else {
                    instance.variationTimer = randFloat(rendering::M2_IDLE_VARIATION_TIMER_MIN_MS, rendering::M2_IDLE_VARIATION_TIMER_MAX_MS);
                }
            }
        }

        // Frustum + distance cull: skip expensive bone computation for off-screen instances.
        // Both effectiveMaxDistSq and paddedRadius are precomputed per instance in
        // recomputeCachedCullFactors(); we only need the per-frame distance and frustum test.
        glm::vec3 toCam = instance.position - cachedCamPos_;
        float distSq = glm::dot(toCam, toCam);
        float effectiveMaxDistSq = cachedMaxRenderDistSq_ * instance.cachedEffectiveMaxDistSqFactor;
        if (instance.cachedIsSkyBird) {
            constexpr float kBirdMaxDistSq =
                rendering::M2_SKY_BIRD_MAX_RENDER_DISTANCE *
                rendering::M2_SKY_BIRD_MAX_RENDER_DISTANCE;
            effectiveMaxDistSq = std::min(effectiveMaxDistSq, kBirdMaxDistSq);
        }
        if (distSq > effectiveMaxDistSq) continue;
        float paddedRadius = instance.cachedPaddedRadius;
        if (paddedRadius > 0.0f && !updateFrustum.intersectsSphere(instance.cachedCullCenter, paddedRadius)) continue;

        // LOD 3 skip: models beyond 150 units use the lowest LOD mesh which has
        // no visible skeletal animation.  Keep their last-computed bone matrices
        // (always valid — seeded on spawn) and avoid the expensive per-bone work.
        // Sky birds, light beams, and ship machinery are exempt: their visible
        // motion is baked entirely into bone animation.
        constexpr float kLOD3DistSq = rendering::M2_LOD3_DISTANCE * rendering::M2_LOD3_DISTANCE;
        const bool needsDistantBones = instance.cachedIsSkyBird || instance.cachedIsLightBeam ||
                                       instance.cachedIsTransportDoodad;
        if (distSq > kLOD3DistSq && !needsDistantBones) continue;

        // Distance-based frame skipping: update distant bones less frequently
        uint32_t boneInterval = 1;
        if (!needsDistantBones) {
            if (distSq > rendering::M2_BONE_SKIP_DIST_FAR * rendering::M2_BONE_SKIP_DIST_FAR) boneInterval = 4;
            else if (distSq > rendering::M2_BONE_SKIP_DIST_MID * rendering::M2_BONE_SKIP_DIST_MID) boneInterval = 2;
        }
        instance.frameSkipCounter++;
        if ((instance.frameSkipCounter % boneInterval) != 0) continue;

        boneWorkIndices_.push_back(idx);
    }

    // Compute bone matrices (expensive, parallel if enough work)
    const size_t animCount = boneWorkIndices_.size();
    if (animCount > 0) {
        static const size_t minParallelAnimInstances = std::max<size_t>(
            8, envSizeOrDefault("WOWEE_M2_ANIM_MT_MIN", 96));
        if (animCount < minParallelAnimInstances || numAnimThreads_ <= 1) {
            // Sequential — not enough work to justify thread overhead
            for (size_t i : boneWorkIndices_) {
                if (i >= instances.size()) continue;
                auto& inst = instances[i];
                if (!inst.cachedModel) continue;
                computeBoneMatrices(*inst.cachedModel, inst);
            }
        } else {
            // Parallel — dispatch across worker threads
            static const size_t minAnimWorkPerThread = std::max<size_t>(
                16, envSizeOrDefault("WOWEE_M2_ANIM_WORK_PER_THREAD", 64));
            const size_t maxUsefulThreads = std::max<size_t>(
                1, (animCount + minAnimWorkPerThread - 1) / minAnimWorkPerThread);
            const size_t numThreads = std::min(static_cast<size_t>(numAnimThreads_), maxUsefulThreads);
            if (numThreads <= 1) {
                for (size_t i : boneWorkIndices_) {
                    if (i >= instances.size()) continue;
                    auto& inst = instances[i];
                    if (!inst.cachedModel) continue;
                    computeBoneMatrices(*inst.cachedModel, inst);
                }
            } else {
                const size_t chunkSize = animCount / numThreads;
                const size_t remainder = animCount % numThreads;

                auto processRange = [this](size_t begin, size_t end) {
                    for (size_t j = begin; j < end; ++j) {
                        size_t idx = boneWorkIndices_[j];
                        if (idx >= instances.size()) continue;
                        auto& inst = instances[idx];
                        if (!inst.cachedModel) continue;
                        computeBoneMatrices(*inst.cachedModel, inst);
                    }
                };

                // Reuse persistent futures vector to avoid allocation
                animFutures_.clear();
                if (animFutures_.capacity() < numThreads) {
                    animFutures_.reserve(numThreads);
                }

                // Dispatch all but the last chunk to the shared pool; process the
                // last chunk on this thread so this call always makes progress even
                // when it itself runs on a pool worker (see ThreadPool docs).
                size_t start = 0;
                for (size_t t = 0; t + 1 < numThreads; ++t) {
                    size_t end = start + chunkSize + (t < remainder ? 1 : 0);
                    animFutures_.push_back(core::ThreadPool::frameWorkers().submit(
                        [processRange, start, end]() { processRange(start, end); }));
                    start = end;
                }
                processRange(start, animCount);

                for (auto& f : animFutures_) {
                    f.get();
                }
            }
        }
    }

    // Particle update (sequential — uses RNG, not thread-safe)
    // Only iterate instances that have particle emitters (pre-built list).
    for (size_t idx : particleInstanceIndices_) {
        if (idx >= instances.size()) continue;
        auto& instance = instances[idx];
        // Distance cull: only update particles within visible range
        glm::vec3 toCam = instance.position - cachedCamPos_;
        float distSq = glm::dot(toCam, toCam);
        if (distSq > cachedMaxRenderDistSq_) continue;
        if (!instance.cachedModel) continue;
        emitParticles(instance, *instance.cachedModel, deltaTime);
        updateParticles(instance, deltaTime);
        if (!instance.cachedModel->ribbonEmitters.empty()) {
            updateRibbons(instance, *instance.cachedModel, deltaTime);
        }
    }

}

// Diagnostic: WOWEE_M2_NO_SKINNING=1 renders every M2 in its bind pose by
// telling the shader to ignore bones, separating a skinning artifact from one
// drawn by the particle or ribbon systems.
static const bool kM2NoSkinning = envFlagEnabled("WOWEE_M2_NO_SKINNING");

void M2Renderer::prepareRender(uint32_t frameIndex, const Camera& camera) {
    if (!initialized_ || instances.empty()) return;
    (void)camera;  // reserved for future frustum-based culling

    // --- Mega bone SSBO: assign ranges and upload all animated instance bones ---
    // Offset 0 is reserved as the identity/no-bones sentinel; animated instances
    // are packed after it at their own bone count, so a 300-bone creature gets
    // all 300 matrices instead of being truncated into a fixed stride and
    // reading into its neighbour's range.
    uint32_t nextOffset = 1;
    for (size_t idx : animatedInstanceIndices_) {
        if (idx >= instances.size()) continue;
        auto& instance = instances[idx];

        if (instance.boneMatrices.empty()) {
            instance.megaBoneOffset = 0;  // Use identity slot
            continue;
        }

        const uint32_t boneCount = static_cast<uint32_t>(instance.boneMatrices.size());
        if (boneCount > MEGA_BONE_MATRIX_CAPACITY - nextOffset) {
            instance.megaBoneOffset = 0;  // Overflow — use identity
            continue;
        }

        instance.megaBoneOffset = nextOffset;

        // Upload bone matrices to mega buffer — only when they were recomputed
        // since the last upload into this frame's buffer, or the instance's
        // slot moved (animated set changed). Most animated instances are
        // distance/frustum/frame-skip culled and keep their previous bones, so
        // skipping their memcpy avoids megabytes of redundant writes per frame.
        if (megaBoneMapped_[frameIndex] &&
            (instance.bonesDirty[frameIndex] ||
             instance.megaBoneUploadedSlot[frameIndex] != instance.megaBoneOffset)) {
            auto* dst = static_cast<glm::mat4*>(megaBoneMapped_[frameIndex]) + instance.megaBoneOffset;
            memcpy(dst, instance.boneMatrices.data(), boneCount * sizeof(glm::mat4));
            instance.bonesDirty[frameIndex] = false;
            instance.megaBoneUploadedSlot[frameIndex] = instance.megaBoneOffset;
        }

        nextOffset += boneCount;
    }
}

// Dispatch GPU frustum culling compute shader into the primary frame command
// buffer. render() consumes the completed output left in this frame slot from
// its previous use; this dispatch produces results for the slot's next reuse.
void M2Renderer::dispatchCullCompute(VkCommandBuffer cmd, uint32_t frameIndex, const Camera& camera) {
    if (!cullPipeline_ || instances.empty()) return;

    const uint32_t numInstances = std::min(static_cast<uint32_t>(instances.size()), MAX_CULL_INSTANCES);

    // --- Compute per-instance adaptive distances (same formula as old CPU cull) ---
    const float targetRenderDist = viewDistanceScale_ *
        ((instances.size() > 2000) ? 300.0f
         : (instances.size() > 1000) ? 500.0f
                                     : 1000.0f);
    const float shrinkRate = 0.005f;
    const float growRate   = 0.05f;
    float blendRate = (targetRenderDist < smoothedRenderDist_) ? shrinkRate : growRate;
    smoothedRenderDist_ = glm::mix(smoothedRenderDist_, targetRenderDist, blendRate);
    const float maxRenderDistance = smoothedRenderDist_;
    const float maxRenderDistanceSq = maxRenderDistance * maxRenderDistance;
    // The shader rejects on this bound before it ever reads the per-instance
    // distance, so it has to clear the game-object floor as well — otherwise
    // that floor is silently capped at 2x the ambient doodad distance.
    const float maxPossibleDistSq = std::max(
        maxRenderDistanceSq * 4.0f,  // 2x safety margin
        rendering::M2_GAME_OBJECT_MIN_RENDER_DISTANCE *
        rendering::M2_GAME_OBJECT_MIN_RENDER_DISTANCE);

    // --- Upload frustum planes + camera (UBO, binding 0) ---
    const glm::mat4 vp = camera.getProjectionMatrix() * camera.getViewMatrix();
    Frustum frustum;
    frustum.extractFromMatrix(vp);
    const glm::vec3 camPos = camera.getPosition();

    if (cullUniformMapped_[frameIndex]) {
        auto* ubo = static_cast<CullUniformsGPU*>(cullUniformMapped_[frameIndex]);
        for (int i = 0; i < 6; i++) {
            const auto& p = frustum.getPlane(static_cast<Frustum::Side>(i));
            ubo->frustumPlanes[i] = glm::vec4(p.normal, p.distance);
        }
        ubo->cameraPos = glm::vec4(camPos, maxPossibleDistSq);
        ubo->instanceCount = numInstances;

        // HiZ occlusion culling fields
        const bool hizReady = hizSystem_ && hizSystem_->isReady();

        // Auto-disable HiZ when the camera has moved/rotated significantly.
        // Large VP changes make the depth pyramid unreliable because the
        // reprojected screen positions diverge from the actual pyramid data.
        bool hizSafe = hizReady;
        if (hizReady) {
            // Compare current VP against previous VP — Frobenius-style max diff.
            float maxDiff = 0.0f;
            const float* curM  = &vp[0][0];
            const float* prevM = &prevVP_[0][0];
            for (int k = 0; k < 16; ++k)
                maxDiff = std::max(maxDiff, std::abs(curM[k] - prevM[k]));
            // Threshold: typical tracking-camera motion (following a walking
            // character) produces diffs of 0.05–0.25.  A fast rotation or
            // zoom easily exceeds 0.5.  The previous threshold (0.15) caused
            // the HiZ pass to toggle on/off every other frame during normal
            // gameplay, which produced global M2 doodad flicker.
            if (maxDiff > rendering::HIZ_VP_DIFF_THRESHOLD) hizSafe = false;
        }

        ubo->hizEnabled = hizSafe ? 1u : 0u;
        ubo->hizMipLevels = hizReady ? hizSystem_->getMipLevels() : 0u;
        ubo->_pad2 = 0;
        if (hizReady) {
            ubo->hizParams = glm::vec4(
                static_cast<float>(hizSystem_->getPyramidWidth()),
                static_cast<float>(hizSystem_->getPyramidHeight()),
                camera.getNearPlane(),
                0.0f
            );
            ubo->viewProj = vp;
            // Use previous frame's VP for HiZ reprojection — the HiZ pyramid
            // was built from the previous frame's depth, so we must project
            // into the same screen space to sample the correct depths.
            ubo->prevViewProj = prevVP_;
        } else {
            ubo->hizParams = glm::vec4(0.0f);
            ubo->viewProj = glm::mat4(1.0f);
            ubo->prevViewProj = glm::mat4(1.0f);
        }

        // Save current VP for next frame's temporal reprojection
        prevVP_ = vp;
    }

    // Rotate this slot's ID log: the dispatch recorded below replaces the one
    // whose results render() is about to consume, so the IDs it was built from
    // become the readable set.  Done before the upload loop overwrites them, and
    // after the early-out above, so "readable" always describes the last dispatch
    // actually recorded on this slot.
    if (frameIndex < 2) {
        cullReadableIds_[frameIndex].swap(cullSubmittedIds_[frameIndex]);
        cullSubmittedIds_[frameIndex].clear();
        cullSubmittedIds_[frameIndex].reserve(numInstances);
        for (uint32_t i = 0; i < numInstances; i++)
            cullSubmittedIds_[frameIndex].push_back(instances[i].id);
    }

    // --- Upload per-instance cull data (SSBO, binding 1) ---
    // The per-instance radius math used to be recomputed here every frame; it's
    // now precomputed once by recomputeCachedCullFactors() since it depends only
    // on static instance state (scale, bound radius, animation/ground flags).
    if (cullInputMapped_[frameIndex]) {
        auto* input = static_cast<CullInstanceGPU*>(cullInputMapped_[frameIndex]);
        for (uint32_t i = 0; i < numInstances; i++) {
            const auto& inst = instances[i];
            float effectiveMaxDistSq = maxRenderDistanceSq * inst.cachedEffectiveMaxDistSqFactor;
            if (inst.isGameObject) {
                effectiveMaxDistSq = std::max(effectiveMaxDistSq,
                    rendering::M2_GAME_OBJECT_MIN_RENDER_DISTANCE *
                    rendering::M2_GAME_OBJECT_MIN_RENDER_DISTANCE);
            }
            if (inst.cachedIsSkyBird && inst.cachedHasAnimation && !inst.cachedDisableAnimation) {
                constexpr float kBirdMaxDistSq =
                    rendering::M2_SKY_BIRD_MAX_RENDER_DISTANCE *
                    rendering::M2_SKY_BIRD_MAX_RENDER_DISTANCE;
                effectiveMaxDistSq = std::min(effectiveMaxDistSq, kBirdMaxDistSq);
            }

            uint32_t flags = 0;
            if (inst.cachedIsValid)          flags |= 1u;
            if (inst.cachedIsSmoke)           flags |= 2u;
            if (inst.cachedIsInvisibleTrap)   flags |= 4u;
            // Bit 3: previouslyVisible — the shader runs the HiZ occlusion test
            // ONLY when this bit is set (an object with no depth in last frame's
            // pyramid can't be tested reliably). Hysteresis: keep it set unless
            // culled for 2+ consecutive frames, preventing single-frame false-cull
            // flicker. The counter lives on the instance, so streaming churn can
            // never pair it with a different object's history.
            //
            // Server game objects (mailboxes, chests, ...) opt out of HiZ entirely
            // by never setting this bit: they are small gameplay props that sit
            // flush against walls and doorframes, exactly where the coarse depth
            // pyramid reports false occlusions. Such a false-cull would persist
            // (the prop is then not rendered, so it never regains depth to clear
            // itself) — the "mailbox went invisible in place" report. Frustum +
            // distance culling still bound them; only the unreliable occlusion
            // test is waived.
            if (inst.hizPrevCulledFrames < 2 && !inst.isGameObject)
                flags |= 8u;

            input[i].sphere = glm::vec4(inst.cachedCullCenter, inst.cachedPaddedRadius);
            input[i].effectiveMaxDistSq = effectiveMaxDistSq;
            input[i].flags = flags;
        }
    }

    // --- Dispatch compute shader ---
    const bool useHiZ = (cullHiZPipeline_ != VK_NULL_HANDLE)
                     && hizSystem_ && hizSystem_->isReady();
    if (useHiZ) {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, cullHiZPipeline_);
        // Set 0: cull UBO + input/output SSBOs
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                cullHiZPipelineLayout_, 0, 1, &cullSet_[frameIndex], 0, nullptr);
        // Set 1: HiZ pyramid sampler
        VkDescriptorSet hizSet = hizSystem_->getDescriptorSet(frameIndex);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                cullHiZPipelineLayout_, 1, 1, &hizSet, 0, nullptr);
    } else {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, cullPipeline_);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                cullPipelineLayout_, 0, 1, &cullSet_[frameIndex], 0, nullptr);
    }

    const uint32_t groupCount = (numInstances + 63) / 64;
    vkCmdDispatch(cmd, groupCount, 1, 1);

    // Make writes available to the host after this frame's fence signals. The
    // CPU invalidates and reads them when this frame slot is reused.
    VkMemoryBarrier barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
    barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
    barrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
    vkCmdPipelineBarrier(cmd,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        VK_PIPELINE_STAGE_HOST_BIT,
        0, 1, &barrier, 0, nullptr, 0, nullptr);
}

void M2Renderer::render(VkCommandBuffer cmd, VkDescriptorSet perFrameSet, const Camera& camera) {
    if (instances.empty() || !opaquePipeline_) {
        return;
    }

    // Debug: log once when we start rendering
    static bool loggedOnce = false;
    if (!loggedOnce) {
        loggedOnce = true;
        LOG_INFO("M2 render: ", instances.size(), " instances, ", models.size(), " models");
    }

    // Periodic diagnostic: report render pipeline stats every 10 seconds
    static int diagCounter = 0;
    if (++diagCounter == 600) { // ~10s at 60fps
        diagCounter = 0;
        uint32_t totalValid = 0, totalAnimated = 0, totalBonesReady = 0, totalMegaBoneOk = 0;
        for (const auto& inst : instances) {
            if (inst.cachedIsValid) totalValid++;
            if (inst.cachedHasAnimation && !inst.cachedDisableAnimation) {
                totalAnimated++;
                if (!inst.boneMatrices.empty()) totalBonesReady++;
                if (inst.megaBoneOffset != 0) totalMegaBoneOk++;
            }
        }
        LOG_INFO("M2 diag: total=", instances.size(),
                 " valid=", totalValid,
                 " animated=", totalAnimated,
                 " bonesReady=", totalBonesReady,
                 " megaBoneOk=", totalMegaBoneOk,
                 " visible=", sortedVisible_.size(),
                 " draws=", lastDrawCallCount);
    }

    // Reuse persistent buffers (clear instead of reallocating)
    glowSprites_.clear();

    lastDrawCallCount = 0;
    const float lavaAnimSeconds = std::chrono::duration<float>(
        std::chrono::steady_clock::now() - kLavaAnimStart).count();

    // GPU cull results — dispatchCullCompute() already updated smoothedRenderDist_.
    // Use the cached value (set by dispatchCullCompute or fallback below).
    const uint32_t frameIndex = vkCtx_->getCurrentFrame();
    const uint32_t numInstances = std::min(static_cast<uint32_t>(instances.size()), MAX_CULL_INSTANCES);
    const uint32_t* visibility = static_cast<const uint32_t*>(cullOutputMapped_[frameIndex]);
    const bool gpuCullAvailable = (cullPipeline_ != VK_NULL_HANDLE && visibility != nullptr);

    // Scatter the GPU visibility results back onto the instances they were
    // computed for.  The results belong to the dispatch recorded on this slot
    // ~2 frames ago, so they are keyed by that dispatch's instance IDs, never by
    // the current array index: createInstance() appends and removeInstance()
    // swap-removes, so a respawned game object lands at the volatile tail of the
    // array and would otherwise inherit the verdict of whatever transient object
    // (spell visual, streamed doodad, creature) held that index two frames back
    // — leaving it culled for as long as the churn continued.
    //
    // Instances with no entry in the readable set were created after that
    // dispatch and keep their defaults: visible, and exempt from the HiZ test.
    //
    // hizPrevCulledFrames is a hysteresis counter rather than a binary flag: an
    // object must be culled for 2 consecutive frames before it stops counting as
    // "previously visible", which prevents the 1-frame-on / 1-frame-off
    // oscillation that showed up as doodad flicker near moving characters.
    if (gpuCullAvailable) {
        const auto& ids = cullReadableIds_[frameIndex < 2 ? frameIndex : 0];
        for (size_t k = 0; k < ids.size(); ++k) {
            // Fast path: with no churn since that dispatch the ordering still
            // matches, so skip the hash lookup.
            M2Instance* inst = nullptr;
            if (k < instances.size() && instances[k].id == ids[k]) {
                inst = &instances[k];
            } else {
                auto idxIt = instanceIndexById.find(ids[k]);
                if (idxIt == instanceIndexById.end() || idxIt->second >= instances.size())
                    continue;  // instance was removed since the dispatch
                inst = &instances[idxIt->second];
            }
            if (visibility[k]) {
                inst->lastCullVisible = 1;
                inst->hizPrevCulledFrames = 0;
            } else {
                inst->lastCullVisible = 0;
                inst->hizPrevCulledFrames =
                    std::min<uint8_t>(inst->hizPrevCulledFrames + 1, 3);
            }
        }
    } else {
        // No GPU cull data — conservatively treat everything as visible.
        for (auto& inst : instances) {
            inst.lastCullVisible = 1;
            inst.hizPrevCulledFrames = 0;
        }
    }

    // If GPU culling was not dispatched, fallback: compute distances on CPU
    float maxRenderDistanceSq;
    if (!gpuCullAvailable) {
        const float targetRenderDist = viewDistanceScale_ *
            ((instances.size() > 2000) ? 300.0f
             : (instances.size() > 1000) ? 500.0f
                                         : 1000.0f);
        const float shrinkRate = 0.005f;
        const float growRate = 0.05f;
        float blendRate = (targetRenderDist < smoothedRenderDist_) ? shrinkRate : growRate;
        smoothedRenderDist_ = glm::mix(smoothedRenderDist_, targetRenderDist, blendRate);
        maxRenderDistanceSq = smoothedRenderDist_ * smoothedRenderDist_;
    } else {
        maxRenderDistanceSq = smoothedRenderDist_ * smoothedRenderDist_;
    }

    const float fadeStartFraction = 0.75f;
    const glm::vec3 camPos = camera.getPosition();

    // Build sorted visible instance list
    sortedVisible_.clear();
    transparentVisible_.clear();
    const size_t expectedVisible = std::min(instances.size() / 3, size_t(600));
    if (sortedVisible_.capacity() < expectedVisible) {
        sortedVisible_.reserve(expectedVisible);
    }
    if (transparentVisible_.capacity() < expectedVisible / 4)
        transparentVisible_.reserve(expectedVisible / 4);

    // GPU frustum culling — build frustum for CPU fallback path and overflow instances
    Frustum frustum;
    {
        const glm::mat4 vp = camera.getProjectionMatrix() * camera.getViewMatrix();
        frustum.extractFromMatrix(vp);
    }
    // Matches the bound uploaded to the cull shader, including the headroom the
    // game-object floor needs (see dispatchCullCompute).
    const float maxPossibleDistSq = std::max(
        maxRenderDistanceSq * 4.0f,
        rendering::M2_GAME_OBJECT_MIN_RENDER_DISTANCE *
        rendering::M2_GAME_OBJECT_MIN_RENDER_DISTANCE);

    const uint32_t totalInstances = static_cast<uint32_t>(instances.size());
    struct VisibleChunk {
        std::vector<VisibleEntry> opaque;
        std::vector<VisibleEntry> transparent;
    };

    // Visibility classification is independent per instance and was the
    // remaining monolithic M2 CPU pass. Split dense scenes across a few pool
    // workers; the caller handles the final chunk so nested use from the M2
    // render worker cannot deadlock the shared pool.
    const uint32_t chunkCount = totalInstances >= 2048
        ? std::min<uint32_t>(4, (totalInstances + 1023) / 1024)
        : 1;
    std::vector<VisibleChunk> chunks(chunkCount);
    auto classifyRange = [&](uint32_t chunk, uint32_t begin, uint32_t end) {
        auto& out = chunks[chunk];
        out.opaque.reserve((end - begin) / 3);
        out.transparent.reserve((end - begin) / 12);
        for (uint32_t i = begin; i < end; ++i) {
            const auto& instance = instances[i];
            float distSq;
            float effectiveMaxDistSq;

            // Server game objects keep a distance floor instead of following the
            // ambient doodad distance down; it also feeds the fade curve below,
            // so a mailbox doesn't fade out at the doodad boundary either.
            const float instanceMaxDistSq = [&] {
                float d = maxRenderDistanceSq * instance.cachedEffectiveMaxDistSqFactor;
                if (instance.isGameObject) {
                    d = std::max(d, rendering::M2_GAME_OBJECT_MIN_RENDER_DISTANCE *
                                    rendering::M2_GAME_OBJECT_MIN_RENDER_DISTANCE);
                }
                return d;
            }();

            if (forceNoCull_) {
                if (!instance.cachedIsValid) continue;
                glm::vec3 toCam = instance.position - camPos;
                distSq = glm::dot(toCam, toCam);
                effectiveMaxDistSq = instanceMaxDistSq;
            } else if (gpuCullAvailable && i < numInstances) {
                // Per-instance verdict scattered above — indexing visibility[]
                // directly here would read the slot of whichever instance held
                // this array position when the dispatch was recorded.
                if (!instance.lastCullVisible) continue;
                glm::vec3 toCam = instance.position - camPos;
                distSq = glm::dot(toCam, toCam);
                effectiveMaxDistSq = instanceMaxDistSq;
            } else {
                if (!instance.cachedIsValid || instance.cachedIsSmoke || instance.cachedIsInvisibleTrap) continue;
                glm::vec3 toCam = instance.position - camPos;
                distSq = glm::dot(toCam, toCam);
                if (distSq > maxPossibleDistSq) continue;
                effectiveMaxDistSq = instanceMaxDistSq;
                if (distSq > effectiveMaxDistSq) continue;
                float paddedRadius = instance.cachedPaddedRadius;
                if (paddedRadius > 0.0f && !frustum.intersectsSphere(instance.cachedCullCenter, paddedRadius)) continue;
            }

            if (instance.cachedIsSkyBird && instance.cachedHasAnimation && !instance.cachedDisableAnimation) {
                constexpr float kBirdMaxDistSq =
                    rendering::M2_SKY_BIRD_MAX_RENDER_DISTANCE *
                    rendering::M2_SKY_BIRD_MAX_RENDER_DISTANCE;
                effectiveMaxDistSq = std::min(effectiveMaxDistSq, kBirdMaxDistSq);
                if (distSq > effectiveMaxDistSq) continue;
            }

            VisibleEntry visible{i, instance.modelId, distSq, effectiveMaxDistSq};
            out.opaque.push_back(visible);
            if (instance.cachedModel &&
                (instance.cachedModel->hasTransparentBatches || instance.cachedModel->isSpellEffect)) {
                out.transparent.push_back(visible);
            }
        }
    };

    std::vector<std::future<void>> visibilityFutures;
    visibilityFutures.reserve(chunkCount > 0 ? chunkCount - 1 : 0);
    const uint32_t chunkSize = (totalInstances + chunkCount - 1) / chunkCount;
    for (uint32_t chunk = 0; chunk + 1 < chunkCount; ++chunk) {
        const uint32_t begin = chunk * chunkSize;
        const uint32_t end = std::min(totalInstances, begin + chunkSize);
        visibilityFutures.push_back(core::ThreadPool::frameWorkers().submit(
            [&, chunk, begin, end]() { classifyRange(chunk, begin, end); }));
    }
    const uint32_t lastChunk = chunkCount - 1;
    classifyRange(lastChunk, lastChunk * chunkSize, totalInstances);
    for (auto& future : visibilityFutures) future.get();

    for (auto& chunk : chunks) {
        sortedVisible_.insert(sortedVisible_.end(),
                              std::make_move_iterator(chunk.opaque.begin()),
                              std::make_move_iterator(chunk.opaque.end()));
        transparentVisible_.insert(transparentVisible_.end(),
                                   std::make_move_iterator(chunk.transparent.begin()),
                                   std::make_move_iterator(chunk.transparent.end()));
    }

    // Two-pass rendering: opaque/alpha-test first (depth write ON), then transparent/additive
    // (depth write OFF, sorted back-to-front) so transparent geometry composites correctly
    // against all opaque geometry rather than only against what was rendered before it.

    // Pass 1: sort by modelId for minimum buffer rebinds (opaque batches)
    std::sort(sortedVisible_.begin(), sortedVisible_.end(),
              [](const VisibleEntry& a, const VisibleEntry& b) { return a.modelId < b.modelId; });


    uint32_t currentModelId = UINT32_MAX;
    const M2ModelGPU* currentModel = nullptr;
    bool currentModelValid = false;

    // State tracking
    VkPipeline currentPipeline = VK_NULL_HANDLE;
    VkDescriptorSet currentMaterialSet = VK_NULL_HANDLE;

    // Push constants now carry per-batch data only; per-instance data is in instance SSBO.
    struct M2PushConstants {
        int32_t texCoordSet;        // UV set index (0 or 1)
        int32_t isFoliage;          // Foliage wind animation flag
        int32_t instanceDataOffset; // Base index into instance SSBO for this draw group
    };

    auto appendInstancePortalGlow = [&](const M2Instance& instance, float distSq) {
        if (distSq >= 400.0f * 400.0f) return;
        glm::vec3 center = glm::vec3(instance.modelMatrix * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
        GlowSprite core;
        core.worldPos = center;
        core.color = glm::vec4(0.35f, 0.55f, 1.0f, 1.25f);
        core.size = instance.scale * 7.0f;
        glowSprites_.push_back(core);

        GlowSprite halo = core;
        halo.color.a *= 0.35f;
        halo.size *= 2.4f;
        glowSprites_.push_back(halo);
    };

    // Validate per-frame descriptor set before any Vulkan commands
    if (!perFrameSet) {
        LOG_ERROR("M2Renderer::render: perFrameSet is VK_NULL_HANDLE — skipping M2 render");
        return;
    }

    // Bind per-frame descriptor set (set 0) — shared across all draws
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                            pipelineLayout_, 0, 1, &perFrameSet, 0, nullptr);

    // Start with opaque pipeline
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, opaquePipeline_);
    currentPipeline = opaquePipeline_;

    // Bind dummy bone set (set 2) so non-animated draws have a valid binding.
    // Bind mega bone SSBO instead — all instances index into one buffer via boneBase.
    if (megaBoneSet_[frameIndex]) {
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                pipelineLayout_, 2, 1, &megaBoneSet_[frameIndex], 0, nullptr);
    } else if (dummyBoneSet_) {
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                pipelineLayout_, 2, 1, &dummyBoneSet_, 0, nullptr);
    }

    // Bind instance data SSBO (set 3) — per-instance transforms, fade, bones
    if (instanceSet_[frameIndex]) {
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                pipelineLayout_, 3, 1, &instanceSet_[frameIndex], 0, nullptr);
    }

    // Reset instance SSBO write cursor for this frame
    instanceDataCount_ = 0;
    auto* instSSBO = static_cast<M2InstanceGPU*>(instanceMapped_[frameIndex]);

    // =====================================================================
    // Opaque pass — instanced draws grouped by (modelId, LOD)
    // =====================================================================
    // sortedVisible_ is already sorted by modelId so consecutive entries share
    // the same vertex/index buffer.  Within each model group we sub-group by
    // targetLOD to guarantee all instances in one vkCmdDrawIndexed use the
    // same batch set.  Per-instance data (model matrix, fade, bones) is
    // written to the instance SSBO; the shader reads it via gl_InstanceIndex.
    {
        struct PendingInstance {
            uint32_t instanceIdx;
            float fadeAlpha;
            bool useBones;
            uint16_t targetLOD;
        };
        std::vector<PendingInstance> pending;
        pending.reserve(128);

        size_t visStart = 0;
        while (visStart < sortedVisible_.size()) {
            // Find group of consecutive entries with same modelId
            uint32_t groupModelId = sortedVisible_[visStart].modelId;
            size_t groupEnd = visStart;
            while (groupEnd < sortedVisible_.size() && sortedVisible_[groupEnd].modelId == groupModelId)
                groupEnd++;

            // Pull the model through the first entry's instance.cachedModel pointer
            // (set at addInstance) instead of doing models.find(groupModelId) per group.
            const auto& firstEntry = sortedVisible_[visStart];
            if (firstEntry.index >= instances.size() || !instances[firstEntry.index].cachedModel) {
                visStart = groupEnd;
                continue;
            }
            const M2ModelGPU& model = *instances[firstEntry.index].cachedModel;
            if (model.isInstancePortal) {
                for (size_t vi = visStart; vi < groupEnd; vi++) {
                    const auto& entry = sortedVisible_[vi];
                    if (entry.index >= instances.size()) continue;
                    appendInstancePortalGlow(instances[entry.index], entry.distSq);
                }
                visStart = groupEnd;
                continue;
            }
            if (!model.vertexBuffer || !model.indexBuffer) {
                visStart = groupEnd;
                continue;
            }

            bool modelNeedsAnimation = model.hasAnimation && !model.disableAnimation;
            const bool foliageLikeModel = model.isFoliageLike;
            const bool particleDominantEffect = model.isSpellEffect &&
                !model.particleEmitters.empty() && model.batches.size() <= 2;

            // Collect per-instance data for this model group
            pending.clear();
            for (size_t vi = visStart; vi < groupEnd; vi++) {
                const auto& entry = sortedVisible_[vi];
                if (entry.index >= instances.size()) continue;
                auto& instance = instances[entry.index];

                // Distance-based fade alpha
                float fadeFrac = model.disableAnimation ? 0.55f : fadeStartFraction;
                float fadeStartDistSq = entry.effectiveMaxDistSq * fadeFrac * fadeFrac;
                float fadeAlpha = 1.0f;
                if (entry.distSq > fadeStartDistSq) {
                    fadeAlpha = std::clamp((entry.effectiveMaxDistSq - entry.distSq) /
                                          (entry.effectiveMaxDistSq - fadeStartDistSq), 0.0f, 1.0f);
                }
                float instanceFadeAlpha = fadeAlpha;
                if (model.isGroundDetail) instanceFadeAlpha *= 0.82f;

                // Bone readiness check
                if (modelNeedsAnimation && instance.boneMatrices.empty()) continue;
                bool needsBones = modelNeedsAnimation && !instance.boneMatrices.empty();
                if (needsBones && instance.megaBoneOffset == 0) continue;

                // LOD selection
                uint16_t desiredLOD = 0;
                if (entry.distSq > 150.0f * 150.0f) desiredLOD = 3;
                else if (entry.distSq > 80.0f * 80.0f) desiredLOD = 2;
                else if (entry.distSq > 40.0f * 40.0f) desiredLOD = 1;
                uint16_t targetLOD = desiredLOD;
                if (desiredLOD > 0 && !(model.availableLODs & (1u << desiredLOD))) targetLOD = 0;

                pending.push_back({entry.index, instanceFadeAlpha, needsBones, targetLOD});
            }

            if (pending.empty()) { visStart = groupEnd; continue; }

            // Sort by targetLOD so each sub-group occupies a contiguous SSBO range
            std::sort(pending.begin(), pending.end(),
                      [](const PendingInstance& a, const PendingInstance& b) { return a.targetLOD < b.targetLOD; });

            // Bind vertex/index buffers once per model group
            VkDeviceSize vbOffset = 0;
            vkCmdBindVertexBuffers(cmd, 0, 1, &model.vertexBuffer, &vbOffset);
            vkCmdBindIndexBuffer(cmd, model.indexBuffer, 0, VK_INDEX_TYPE_UINT16);

            // Write base instance data to SSBO (uvOffset=0 — overridden for tex-anim batches)
            uint32_t baseSSBOOffset = instanceDataCount_;
            size_t writtenInstances = 0;
            for (const auto& p : pending) {
                if (instanceDataCount_ >= MAX_INSTANCE_DATA) break;
                auto& inst = instances[p.instanceIdx];
                auto& e = instSSBO[instanceDataCount_];
                e.model = inst.modelMatrix;
                e.uvOffset = glm::vec2(0.0f);
                e.fadeAlpha = p.fadeAlpha;
                e.useBones = (p.useBones && !kM2NoSkinning) ? 1 : 0;
                e.boneBase = p.useBones ? static_cast<int32_t>(inst.megaBoneOffset) : 0;
                e.boneCount = static_cast<int32_t>(inst.boneMatrices.size());
                std::memset(e._pad, 0, sizeof(e._pad));
                instanceDataCount_++;
                ++writtenInstances;
            }

            // Drop what did not fit. The loop above stops writing at the cap,
            // but the LOD sub-groups below are ranges over `pending` and were
            // still being drawn in full — groupSSBOOffset then runs past the
            // end of the buffer and the vertex shader reads instance data that
            // is not there. That is a real out-of-bounds read on the GPU, not a
            // missing model: it fires once per instance past the cap, hundreds
            // of times a frame, and the device is lost seconds later.
            //
            // Truncating is safe here precisely because `pending` was sorted by
            // LOD before the write: the sub-groups are contiguous and in the
            // same order, so cutting the tail cuts whole instances rather than
            // splitting a range.
            if (writtenInstances < pending.size()) {
                static bool warnedInstanceCap = false;
                if (!warnedInstanceCap) {
                    warnedInstanceCap = true;
                    LOG_WARNING("M2Renderer: instance buffer full at ", MAX_INSTANCE_DATA,
                                "; dropping ", pending.size() - writtenInstances,
                                " instances of '", model.name, "' this frame");
                }
                pending.resize(writtenInstances);
            }

            // Process LOD sub-groups within this model group
            size_t lodIdx = 0;
            while (lodIdx < pending.size()) {
                uint16_t lod = pending[lodIdx].targetLOD;
                size_t lodEnd = lodIdx + 1;
                while (lodEnd < pending.size() && pending[lodEnd].targetLOD == lod) lodEnd++;
                uint32_t groupSize = static_cast<uint32_t>(lodEnd - lodIdx);
                uint32_t groupSSBOOffset = baseSSBOOffset + static_cast<uint32_t>(lodIdx);

                for (size_t bi = 0; bi < model.batches.size(); bi++) {
                    const auto& batch = model.batches[bi];
                    if (batch.indexCount == 0) continue;
                    if (!model.isGroundDetail && batch.submeshLevel != lod) continue;
                    if (batch.batchOpacity < 0.01f) continue;

                    // Glow sprite check (per model+batch, sprites generated per instance)
                    const bool koboldFlameCard = batch.colorKeyBlack && model.isKoboldFlame;
                    const bool smallCardLikeBatch =
                        (batch.glowSize <= 1.35f) ||
                        (batch.lanternGlowHint && batch.glowSize <= 6.0f);
                    const bool batchUnlit = (batch.materialFlags & 0x01) != 0;
                    const bool shouldUseGlowSprite =
                        !koboldFlameCard &&
                        (model.isElvenLike ||
                         ((model.isLanternLike || model.isTorch || model.isBrazierOrFire) &&
                          batch.lanternGlowHint)) &&
                        !model.isSpellEffect &&
                        smallCardLikeBatch &&
                        (batch.lanternGlowHint ||
                         (batch.blendMode >= 3) ||
                         (batch.colorKeyBlack && batchUnlit && batch.blendMode >= 1));
                    if (shouldUseGlowSprite) {
                        // Generate glow sprites for each instance in the group
                        for (size_t j = lodIdx; j < lodEnd; j++) {
                            auto& inst = instances[pending[j].instanceIdx];
                            glm::vec3 worldPos;
                            if (model.isGroundFire &&
                                !model.particleEmitters.empty()) {
                                worldPos = glm::vec3(std::numeric_limits<float>::max());
                                for (const auto& emitter : model.particleEmitters) {
                                    glm::mat4 boneXform(1.0f);
                                    if (emitter.bone < inst.boneMatrices.size()) {
                                        boneXform = inst.boneMatrices[emitter.bone];
                                    }
                                    const glm::vec3 emitterWorld = glm::vec3(
                                        inst.modelMatrix * boneXform * glm::vec4(emitter.position, 1.0f));
                                    if (emitterWorld.z < worldPos.z) worldPos = emitterWorld;
                                }
                            } else {
                                worldPos = animatedBatchWorldCenter(inst, batch);
                            }
                            // Preserved emissive glass writes opaque depth before
                            // this additive point sprite. Move only the visual
                            // halo just beyond the camera-facing glass surface so
                            // depth testing does not reject it; the associated
                            // local light remains at the true batch center.
                            if (batch.preserveGlowMesh) {
                                const glm::vec3 towardCamera = camPos - worldPos;
                                const float lenSq = glm::dot(towardCamera, towardCamera);
                                if (lenSq > 0.0001f) {
                                    worldPos += towardCamera * glm::inversesqrt(lenSq) *
                                        (batch.glowSize * inst.scale * 1.25f);
                                }
                            }
                            GlowSprite gs;
                            gs.worldPos = worldPos;
                            if (batch.glowTint == 1 || model.isElvenLike)
                                gs.color = glm::vec4(0.48f, 0.72f, 1.0f, 1.05f);
                            else if (batch.glowTint == 2)
                                gs.color = glm::vec4(1.0f, 0.28f, 0.22f, 1.10f);
                            else
                                gs.color = glm::vec4(1.0f, 0.82f, 0.46f, 1.15f);
                            // Match the parent M2's distance fade instead of a separate
                            // hard 180-unit cutoff, which made tunnel lights pop on.
                            gs.color.a *= pending[j].fadeAlpha;
                            gs.size = batch.glowSize * inst.scale *
                                (batch.preserveGlowMesh ? 2.0f : 1.45f);
                            if (batch.preserveGlowMesh) gs.color.a *= 1.25f;

                            // A fixture with real particle flames should read as
                            // flames with a halo behind them. The sprite is sized
                            // from its glow card's geometric radius, which on a
                            // chandelier spans the whole fixture — an additive
                            // blob about a unit across, against candle flames of
                            // 0.15, so the glow swallowed them entirely. Cap it
                            // just above what a small glow card already produces
                            // (the 0.5 floor times 1.45), so candles, lanterns
                            // and torches are untouched and only oversized cards
                            // are clamped.
                            if (!model.particleEmitters.empty() && model.isLanternLike) {
                                constexpr float kMaxHaloRadius = 0.75f;
                                gs.size = std::min(gs.size, kMaxHaloRadius * inst.scale);
                            }

                            // Fire burning inside a hearth. The sprite is a point
                            // billboard carrying one depth value for the whole
                            // quad, so as soon as its centre shows through the
                            // fireplace opening the entire square draws — brick
                            // surround included, which reads as the fire glowing
                            // through the masonry. Sized from the glow card's
                            // geometric radius these spheres are wider than the
                            // opening, so keep them inside it.
                            const bool hearthFire = model.isBrazierOrFire ||
                                                    model.isGroundFire ||
                                                    model.isForge;
                            if (hearthFire) {
                                constexpr float kMaxFireGlowRadius = 0.5f;
                                gs.size = std::min(gs.size, kMaxFireGlowRadius * inst.scale);
                            }

                            // Flame guttering. The phase comes from the lamp's own
                            // world position, so two lanterns on the same street
                            // never pulse together — a synchronised row of lamps
                            // reads as a rendering artifact, not firelight. Two
                            // detuned sines keep any single lamp from looping
                            // visibly. Alpha and size move together, since a
                            // brighter flame also looks slightly larger.
                            {
                                // Matches the phase used for this lamp's local
                                // light, so the sprite and the pool of light it
                                // casts breathe together.
                                // Same clock and parameters as the local light in
                                // gatherLocalLights, so the sprite and the pool of
                                // light it casts rise and fall together.
                                const float flicker = lampFlicker(
                                    inst.position, lampFlickerClockSeconds(),
                                    0.82f, 0.12f, 0.06f);
                                gs.color.a *= flicker;
                                gs.size    *= 0.98f + 0.02f * flicker;
                            }
                            glowSprites_.push_back(gs);
                            GlowSprite halo = gs;
                            halo.color.a *= batch.preserveGlowMesh ? 0.34f : 0.42f;
                            halo.size *= batch.preserveGlowMesh ? 2.2f : 1.8f;
                            // The halo is nearly twice the sprite, so capping only
                            // the sprite would leave the bleed to the halo.
                            if (hearthFire) {
                                constexpr float kMaxFireHaloRadius = 0.85f;
                                halo.size = std::min(halo.size, kMaxFireHaloRadius * inst.scale);
                            }
                            glowSprites_.push_back(halo);
                        }
                        const bool cardLikeSkipMesh =
                            !batch.preserveGlowMesh &&
                            (batch.glowCardLike || (batch.blendMode >= 3) ||
                             batch.colorKeyBlack || batchUnlit);
                        const bool lanternGlowCardSkip =
                            (model.isLanternLike || model.isTorch || model.isBrazierOrFire) &&
                            batch.lanternGlowHint &&
                            smallCardLikeBatch && cardLikeSkipMesh;
                        if (lanternGlowCardSkip || (cardLikeSkipMesh && !model.isLanternLike))
                            continue;
                    }

                    // Opaque gate — transparent glow cards were handled above so their
                    // sprites are generated before the mesh moves to pass 2.
                    const bool rawTransparent = (batch.blendMode >= 2) || model.isSpellEffect;
                    if (rawTransparent) continue;

                    // Particle-dominant effects: emission geometry — skip opaque
                    if (particleDominantEffect && batch.blendMode <= 1) continue;

                    // Handle texture animation: if this batch has per-instance uvOffset,
                    // write a separate SSBO range with the correct offsets.
                    bool hasBatchTexAnim = (batch.textureAnimIndex != 0xFFFF && model.hasTextureAnimation)
                                           || model.isLavaModel;
                    uint32_t drawOffset = groupSSBOOffset;
                    if (hasBatchTexAnim && instanceDataCount_ + groupSize <= MAX_INSTANCE_DATA) {
                        drawOffset = instanceDataCount_;
                        // Hoist per-batch lookups: the transform pointer is fixed for
                        // every instance in this group; only the sampled translation
                        // varies (per-instance animTime).
                        const pipeline::M2TextureTransform* tt = nullptr;
                        if (batch.textureAnimIndex != 0xFFFF && model.hasTextureAnimation) {
                            uint16_t lookupIdx = batch.textureAnimIndex;
                            if (lookupIdx < model.textureTransformLookup.size()) {
                                uint16_t transformIdx = model.textureTransformLookup[lookupIdx];
                                if (transformIdx < model.textureTransforms.size()) {
                                    tt = &model.textureTransforms[transformIdx];
                                }
                            }
                        }
                        for (size_t j = lodIdx; j < lodEnd; j++) {
                            const auto& p = pending[j];
                            auto& inst = instances[p.instanceIdx];
                            glm::vec2 uvOffset(0.0f);
                            if (tt) {
                                glm::vec3 trans = m2_track::sampleVec3(
                                    tt->translation, inst.currentSequenceIndex,
                                    inst.animTime, inst.globalSequenceTime,
                                    model.globalSequenceDurations, glm::vec3(0.0f));
                                uvOffset = glm::vec2(trans.x, trans.y);
                            }
                            if (model.isLavaModel && uvOffset == glm::vec2(0.0f)) {
                                uvOffset = glm::vec2(lavaAnimSeconds * 0.03f,
                                                     -lavaAnimSeconds * 0.08f);
                            }
                            // Rebuild the entry from CPU-side data rather than copying it
                            // out of the base entry. instSSBO lives in write-combined
                            // upload memory: writing it is cheap, but reading it back is
                            // uncached and costs microseconds per instance.
                            auto& e = instSSBO[instanceDataCount_];
                            e.model = inst.modelMatrix;
                            e.uvOffset = uvOffset;
                            e.fadeAlpha = p.fadeAlpha;
                            e.useBones = (p.useBones && !kM2NoSkinning) ? 1 : 0;
                            e.boneBase = p.useBones ? static_cast<int32_t>(inst.megaBoneOffset) : 0;
                            e.boneCount = static_cast<int32_t>(inst.boneMatrices.size());
                            std::memset(e._pad, 0, sizeof(e._pad));
                            instanceDataCount_++;
                        }
                    }

                    // Pipeline selection (per-model/batch, not per-instance)
                    const bool foliageCutout = foliageLikeModel && !model.isSpellEffect && batch.blendMode <= 3;
                    // The fire burning in the hearth is an effect overlay on a
                    // black background, the same shape as a spell visual; drawn
                    // opaque it fills the forge opening with a black rectangle
                    // instead of flame. That is true of the flame cards only —
                    // treating the whole model this way turned the masonry and
                    // ironwork additive, which is to say translucent.
                    const bool fireEffectModel = batch.forgeFireCard;
                    const bool forceCutout =
                        !model.isSpellEffect && !fireEffectModel &&
                        (model.isGroundDetail || foliageCutout ||
                         batch.blendMode == 1 ||
                         (batch.blendMode >= 2 && !batch.hasAlpha) ||
                         batch.colorKeyBlack);

                    uint8_t effectiveBlendMode = batch.blendMode;
                    if (model.isSpellEffect || fireEffectModel) {
                        if (effectiveBlendMode <= 1) effectiveBlendMode = 3;
                        else if (effectiveBlendMode == 4 || effectiveBlendMode == 5) effectiveBlendMode = 3;
                    }
                    if (forceCutout) effectiveBlendMode = 1;

                    VkPipeline desiredPipeline;
                    if (forceCutout) {
                        desiredPipeline = opaquePipeline_;
                    } else {
                        switch (effectiveBlendMode) {
                            case 0: desiredPipeline = opaquePipeline_; break;
                            case 1: desiredPipeline = alphaTestPipeline_; break;
                            case 2: desiredPipeline = alphaPipeline_; break;
                            default: desiredPipeline = additivePipeline_; break;
                        }
                    }
                    if (desiredPipeline != currentPipeline) {
                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, desiredPipeline);
                        currentPipeline = desiredPipeline;
                    }

                    // Update material UBO
                    if (batch.materialUBOMapped) {
                        auto* mat = static_cast<M2MaterialUBO*>(batch.materialUBOMapped);
                        // interiorDarken is a camera-based flag — it darkens ALL M2s (incl.
                        // outdoor trees) when the camera is inside a WMO.  Disable it; indoor
                        // M2s already look correct from the darker ambient/lighting.
                        mat->interiorDarken = 0.0f;
                        if (batch.colorKeyBlack)
                            mat->colorKeyThreshold = (effectiveBlendMode == 4 || effectiveBlendMode == 5) ? 0.7f : 0.08f;
                        if (forceCutout) {
                            mat->alphaTest = model.isGroundDetail ? 3 : (foliageCutout ? 2 : 1);
                            if (model.isGroundDetail) mat->unlit = 0;
                        }
                    }

                    // Bind material descriptor set (set 1)
                    if (!batch.materialSet) continue;
                    if (batch.materialSet != currentMaterialSet) {
                        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                                pipelineLayout_, 1, 1, &batch.materialSet, 0, nullptr);
                        currentMaterialSet = batch.materialSet;
                    }

                    // Push constants + instanced draw
                    M2PushConstants pc;
                    pc.texCoordSet = static_cast<int32_t>(batch.textureUnit);
                    pc.isFoliage = skyMode_ ? -1 : (model.shadowWindFoliage ? 1 : 0);
                    pc.instanceDataOffset = static_cast<int32_t>(drawOffset);
                    vkCmdPushConstants(cmd, pipelineLayout_, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(pc), &pc);
                    vkCmdDrawIndexed(cmd, batch.indexCount, groupSize, batch.indexStart, 0, 0);
                    lastDrawCallCount++;
                }

                lodIdx = lodEnd;
            }

            visStart = groupEnd;
        }
    }

    // =====================================================================
    // Pass 2: Transparent/additive batches — back-to-front per instance
    // =====================================================================
    // Transparent geometry must be drawn individually per instance in back-to-
    // front order for correct alpha compositing.  Each draw writes one
    // M2InstanceGPU entry and issues a single-instance indexed draw.
    std::sort(transparentVisible_.begin(), transparentVisible_.end(),
              [](const VisibleEntry& a, const VisibleEntry& b) { return a.distSq > b.distSq; });

    currentModelId = UINT32_MAX;
    currentModel = nullptr;
    currentModelValid = false;
    currentPipeline = opaquePipeline_;
    currentMaterialSet = VK_NULL_HANDLE;

    for (const auto& entry : transparentVisible_) {
        if (entry.index >= instances.size()) continue;
        auto& instance = instances[entry.index];

        // Model boundary: read cachedModel off the instance — was doing a
        // per-boundary models.find() even though every instance already has
        // the pointer cached at addInstance time.
        if (entry.modelId != currentModelId) {
            currentModelId = entry.modelId;
            currentModelValid = false;
            currentModel = instance.cachedModel;
            if (!currentModel) continue;
            if (currentModel->isInstancePortal) continue;
            if (!currentModel->hasTransparentBatches && !currentModel->isSpellEffect) continue;
            if (!currentModel->vertexBuffer || !currentModel->indexBuffer) continue;
            currentModelValid = true;
            VkDeviceSize vbOff = 0;
            vkCmdBindVertexBuffers(cmd, 0, 1, &currentModel->vertexBuffer, &vbOff);
            vkCmdBindIndexBuffer(cmd, currentModel->indexBuffer, 0, VK_INDEX_TYPE_UINT16);
        }
        if (!currentModelValid) continue;

        const M2ModelGPU& model = *currentModel;

        // Fade alpha
        float fadeAlpha = 1.0f;
        float fadeFrac = model.disableAnimation ? 0.55f : fadeStartFraction;
        float fadeStartDistSq = entry.effectiveMaxDistSq * fadeFrac * fadeFrac;
        if (entry.distSq > fadeStartDistSq) {
            fadeAlpha = std::clamp((entry.effectiveMaxDistSq - entry.distSq) /
                                  (entry.effectiveMaxDistSq - fadeStartDistSq), 0.0f, 1.0f);
        }
        float instanceFadeAlpha = fadeAlpha;
        if (model.isGroundDetail) instanceFadeAlpha *= 0.82f;
        if (model.isInstancePortal) instanceFadeAlpha *= 0.72f;

        bool modelNeedsAnimation = model.hasAnimation && !model.disableAnimation;
        if (modelNeedsAnimation && instance.boneMatrices.empty()) continue;
        bool needsBones = modelNeedsAnimation && !instance.boneMatrices.empty();
        if (needsBones && instance.megaBoneOffset == 0) continue;

        uint16_t desiredLOD = 0;
        if (entry.distSq > 150.0f * 150.0f) desiredLOD = 3;
        else if (entry.distSq > 80.0f * 80.0f) desiredLOD = 2;
        else if (entry.distSq > 40.0f * 40.0f) desiredLOD = 1;
        uint16_t targetLOD = desiredLOD;
        if (desiredLOD > 0 && !(model.availableLODs & (1u << desiredLOD))) targetLOD = 0;

        const bool particleDominantEffect = model.isSpellEffect &&
            !model.particleEmitters.empty() && model.batches.size() <= 2;

        for (const auto& batch : model.batches) {
            if (batch.indexCount == 0) continue;
            if (!model.isGroundDetail && batch.submeshLevel != targetLOD) continue;
            if (batch.batchOpacity < 0.01f) continue;

            // Pass 2 gate: only transparent/additive batches
            {
                const bool rawTransparent = (batch.blendMode >= 2) || model.isSpellEffect;
                if (!rawTransparent) continue;
            }

            // Skip glow sprites (handled in opaque pass)
            const bool batchUnlit = (batch.materialFlags & 0x01) != 0;
            const bool koboldFlameCard = batch.colorKeyBlack && model.isKoboldFlame;
            const bool smallCardLikeBatch =
                (batch.glowSize <= 1.35f) ||
                (batch.lanternGlowHint && batch.glowSize <= 6.0f);
            const bool shouldUseGlowSprite =
                !koboldFlameCard &&
                (model.isElvenLike ||
                 ((model.isLanternLike || model.isTorch || model.isBrazierOrFire) &&
                  batch.lanternGlowHint)) &&
                !model.isSpellEffect &&
                smallCardLikeBatch &&
                (batch.lanternGlowHint || (batch.blendMode >= 3) ||
                 (batch.colorKeyBlack && batchUnlit && batch.blendMode >= 1));
            if (shouldUseGlowSprite) {
                const bool cardLikeSkipMesh = !batch.preserveGlowMesh &&
                    (batch.glowCardLike || (batch.blendMode >= 3) ||
                     batch.colorKeyBlack || batchUnlit);
                const bool lanternGlowCardSkip =
                    (model.isLanternLike || model.isTorch || model.isBrazierOrFire) &&
                    batch.lanternGlowHint &&
                    smallCardLikeBatch &&
                    cardLikeSkipMesh;
                if (lanternGlowCardSkip || (cardLikeSkipMesh && !model.isLanternLike))
                    continue;
            }

            if (particleDominantEffect) continue; // emission-only mesh

            // Compute UV offset for this instance + batch
            glm::vec2 uvOffset(0.0f);
            if (batch.textureAnimIndex != 0xFFFF && model.hasTextureAnimation) {
                uint16_t lookupIdx = batch.textureAnimIndex;
                if (lookupIdx < model.textureTransformLookup.size()) {
                    uint16_t transformIdx = model.textureTransformLookup[lookupIdx];
                    if (transformIdx < model.textureTransforms.size()) {
                        const auto& tt = model.textureTransforms[transformIdx];
                        glm::vec3 trans = m2_track::sampleVec3(
                            tt.translation, instance.currentSequenceIndex,
                            instance.animTime, instance.globalSequenceTime,
                            model.globalSequenceDurations, glm::vec3(0.0f));
                        uvOffset = glm::vec2(trans.x, trans.y);
                    }
                }
            }
            if (model.isLavaModel && uvOffset == glm::vec2(0.0f)) {
                uvOffset = glm::vec2(lavaAnimSeconds * 0.03f,
                                     -lavaAnimSeconds * 0.08f);
            }

            // Write single instance entry to SSBO
            if (instanceDataCount_ >= MAX_INSTANCE_DATA) continue;
            uint32_t drawOffset = instanceDataCount_;
            auto& e = instSSBO[instanceDataCount_];
            e.model = instance.modelMatrix;
            e.uvOffset = uvOffset;
            e.fadeAlpha = instanceFadeAlpha;
            e.useBones = (needsBones && !kM2NoSkinning) ? 1 : 0;
            e.boneBase = needsBones ? static_cast<int32_t>(instance.megaBoneOffset) : 0;
            e.boneCount = static_cast<int32_t>(instance.boneMatrices.size());
            std::memset(e._pad, 0, sizeof(e._pad));
            instanceDataCount_++;

            // Pipeline selection
            uint8_t effectiveBlendMode = batch.blendMode;
            if (model.isSpellEffect || batch.forgeFireCard) {
                // Matches the opaque pass: a forge's flame cards are additive,
                // the forge itself is not.
                if (effectiveBlendMode <= 1) effectiveBlendMode = 3;
                else if (effectiveBlendMode == 4 || effectiveBlendMode == 5) effectiveBlendMode = 3;
            }

            VkPipeline desiredPipeline;
            switch (effectiveBlendMode) {
                case 2: desiredPipeline = alphaPipeline_; break;
                default: desiredPipeline = additivePipeline_; break;
            }
            if (desiredPipeline != currentPipeline) {
                vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, desiredPipeline);
                currentPipeline = desiredPipeline;
            }

            if (batch.materialUBOMapped) {
                auto* mat = static_cast<M2MaterialUBO*>(batch.materialUBOMapped);
                mat->interiorDarken = 0.0f;
                if (batch.colorKeyBlack)
                    mat->colorKeyThreshold = (effectiveBlendMode == 4 || effectiveBlendMode == 5) ? 0.7f : 0.08f;
            }

            if (!batch.materialSet) continue;
            if (batch.materialSet != currentMaterialSet) {
                vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                        pipelineLayout_, 1, 1, &batch.materialSet, 0, nullptr);
                currentMaterialSet = batch.materialSet;
            }

            // Push constants + single-instance draw
            M2PushConstants pc;
            pc.texCoordSet = static_cast<int32_t>(batch.textureUnit);
            pc.isFoliage = skyMode_ ? -1 : (model.shadowWindFoliage ? 1 : 0);
            pc.instanceDataOffset = static_cast<int32_t>(drawOffset);
            vkCmdPushConstants(cmd, pipelineLayout_, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(pc), &pc);
            vkCmdDrawIndexed(cmd, batch.indexCount, 1, batch.indexStart, 0, 0);
            lastDrawCallCount++;
        }
    }

    // Render glow sprites as billboarded additive point lights
    if (!glowSprites_.empty() && particleAdditivePipeline_ && glowVB_ && glowTexDescSet_) {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, particleAdditivePipeline_);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                particlePipelineLayout_, 0, 1, &perFrameSet, 0, nullptr);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                particlePipelineLayout_, 1, 1, &glowTexDescSet_, 0, nullptr);

        // Push constants for particle: tileCount(vec2) + alphaKey(int)
        struct { float tileX, tileY; int alphaKey; } particlePush = {1.0f, 1.0f, 0};
        vkCmdPushConstants(cmd, particlePipelineLayout_, VK_SHADER_STAGE_FRAGMENT_BIT, 0,
                           sizeof(particlePush), &particlePush);

        // Write glow vertex data directly to mapped buffer (no temp vector)
        size_t uploadCount = std::min(glowSprites_.size(), MAX_GLOW_SPRITES);
        float* dst = static_cast<float*>(glowVBMapped_);
        for (size_t gi = 0; gi < uploadCount; gi++) {
            const auto& gs = glowSprites_[gi];
            *dst++ = gs.worldPos.x;
            *dst++ = gs.worldPos.y;
            *dst++ = gs.worldPos.z;
            *dst++ = gs.color.r;
            *dst++ = gs.color.g;
            *dst++ = gs.color.b;
            *dst++ = gs.color.a;
            *dst++ = gs.size;
            *dst++ = 0.0f;
        }

        VkDeviceSize offset = 0;
        vkCmdBindVertexBuffers(cmd, 0, 1, &glowVB_, &offset);
        vkCmdDraw(cmd, static_cast<uint32_t>(uploadCount), 1, 0, 0);
    }

}

bool M2Renderer::initializeShadow(VkRenderPass shadowRenderPass) {
    if (!vkCtx_ || shadowRenderPass == VK_NULL_HANDLE) return false;
    VkDevice device = vkCtx_->getDevice();

    // Create ShadowParams UBO
    VkBufferCreateInfo bufCI{};
    bufCI.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bufCI.size = sizeof(ShadowParamsUBO);
    bufCI.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
    VmaAllocationCreateInfo allocCI{};
    allocCI.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
    allocCI.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
    VmaAllocationInfo allocInfo{};
    if (vmaCreateBuffer(vkCtx_->getAllocator(), &bufCI, &allocCI,
            &shadowParamsUBO_, &shadowParamsAlloc_, &allocInfo) != VK_SUCCESS) {
        LOG_ERROR("M2Renderer: failed to create shadow params UBO");
        return false;
    }
    ShadowParamsUBO defaultParams{};
    std::memcpy(allocInfo.pMappedData, &defaultParams, sizeof(defaultParams));

    // Create descriptor set layout: binding 0 = sampler2D, binding 1 = ShadowParams UBO
    VkDescriptorSetLayoutBinding layoutBindings[2]{};
    layoutBindings[0].binding = 0;
    layoutBindings[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    layoutBindings[0].descriptorCount = 1;
    layoutBindings[0].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    layoutBindings[1].binding = 1;
    layoutBindings[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    layoutBindings[1].descriptorCount = 1;
    layoutBindings[1].stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    VkDescriptorSetLayoutCreateInfo layoutCI{};
    layoutCI.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutCI.bindingCount = 2;
    layoutCI.pBindings = layoutBindings;
    if (vkCreateDescriptorSetLayout(device, &layoutCI, nullptr, &shadowParamsLayout_) != VK_SUCCESS) {
        LOG_ERROR("M2Renderer: failed to create shadow params layout");
        return false;
    }

    // Create descriptor pool
    VkDescriptorPoolSize poolSizes[2]{};
    poolSizes[0].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    poolSizes[0].descriptorCount = 1;
    poolSizes[1].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    poolSizes[1].descriptorCount = 1;
    VkDescriptorPoolCreateInfo poolCI{};
    poolCI.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolCI.maxSets = 1;
    poolCI.poolSizeCount = 2;
    poolCI.pPoolSizes = poolSizes;
    if (vkCreateDescriptorPool(device, &poolCI, nullptr, &shadowParamsPool_) != VK_SUCCESS) {
        LOG_ERROR("M2Renderer: failed to create shadow params pool");
        return false;
    }

    // Allocate descriptor set
    VkDescriptorSetAllocateInfo setAlloc{};
    setAlloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    setAlloc.descriptorPool = shadowParamsPool_;
    setAlloc.descriptorSetCount = 1;
    setAlloc.pSetLayouts = &shadowParamsLayout_;
    if (vkAllocateDescriptorSets(device, &setAlloc, &shadowParamsSet_) != VK_SUCCESS) {
        LOG_ERROR("M2Renderer: failed to allocate shadow params set");
        return false;
    }

    // Write descriptors (use white fallback for binding 0)
    VkDescriptorBufferInfo bufInfo{};
    bufInfo.buffer = shadowParamsUBO_;
    bufInfo.offset = 0;
    bufInfo.range = sizeof(ShadowParamsUBO);

    VkDescriptorImageInfo imgInfo{};
    imgInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    imgInfo.imageView = whiteTexture_->getImageView();
    imgInfo.sampler = whiteTexture_->getSampler();

    VkWriteDescriptorSet writes[2]{};
    writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[0].dstSet = shadowParamsSet_;
    writes[0].dstBinding = 0;
    writes[0].descriptorCount = 1;
    writes[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writes[0].pImageInfo = &imgInfo;
    writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[1].dstSet = shadowParamsSet_;
    writes[1].dstBinding = 1;
    writes[1].descriptorCount = 1;
    writes[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[1].pBufferInfo = &bufInfo;
    vkUpdateDescriptorSets(device, 2, writes, 0, nullptr);

    // Per-frame pools for foliage shadow texture sets (one per frame-in-flight, reset each frame)
    {
        VkDescriptorPoolSize texPoolSizes[2]{};
        texPoolSizes[0].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        texPoolSizes[0].descriptorCount = 256;
        texPoolSizes[1].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        texPoolSizes[1].descriptorCount = 256;
        VkDescriptorPoolCreateInfo texPoolCI{};
        texPoolCI.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
        texPoolCI.maxSets = 256;
        texPoolCI.poolSizeCount = 2;
        texPoolCI.pPoolSizes = texPoolSizes;
        for (uint32_t f = 0; f < kShadowTexPoolFrames; ++f) {
            if (vkCreateDescriptorPool(device, &texPoolCI, nullptr, &shadowTexPool_[f]) != VK_SUCCESS) {
                LOG_ERROR("M2Renderer: failed to create shadow texture pool ", f);
                return false;
            }
        }
    }

    // Create shadow pipeline layout: set 1 = shadowParamsLayout_, push constants = 128 bytes
    VkPushConstantRange pc{};
    pc.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    pc.offset = 0;
    pc.size = 128;  // lightSpaceMatrix (64) + model (64)
    shadowPipelineLayout_ = createPipelineLayout(device, {shadowParamsLayout_}, {pc});
    if (!shadowPipelineLayout_) {
        LOG_ERROR("M2Renderer: failed to create shadow pipeline layout");
        return false;
    }

    // Load shadow shaders
    VkShaderModule vertShader, fragShader;
    if (!vertShader.loadFromFile(device, "assets/shaders/shadow.vert.spv")) {
        LOG_ERROR("M2Renderer: failed to load shadow vertex shader");
        return false;
    }
    if (!fragShader.loadFromFile(device, "assets/shaders/shadow.frag.spv")) {
        LOG_ERROR("M2Renderer: failed to load shadow fragment shader");
        return false;
    }

    // M2 vertex layout: 18 floats = 72 bytes stride
    // loc0=pos(off0), loc1=normal(off12), loc2=texCoord0(off24), loc5=texCoord1(off32),
    // loc3=boneWeights(off40), loc4=boneIndices(off56)
    // Shadow shader locations: 0=aPos, 1=aTexCoord, 2=aBoneWeights, 3=aBoneIndicesF
    // useBones=0 so locations 2,3 are never used
    VkVertexInputBindingDescription vertBind{};
    vertBind.binding = 0;
    vertBind.stride = 18 * sizeof(float);
    vertBind.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
    std::vector<VkVertexInputAttributeDescription> vertAttrs = {
        {0, 0, VK_FORMAT_R32G32B32_SFLOAT,    0},                     // aPos       -> position
        {1, 0, VK_FORMAT_R32G32_SFLOAT,       6 * sizeof(float)},     // aTexCoord  -> texCoord0
        {2, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 10 * sizeof(float)},    // aBoneWeights
        {3, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 14 * sizeof(float)},    // aBoneIndicesF
    };

    shadowPipeline_ = PipelineBuilder()
        .setShaders(vertShader.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                    fragShader.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
        .setVertexInput({vertBind}, vertAttrs)
        .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
        // Foliage/leaf cards are effectively two-sided; front-face culling can
        // drop them from the shadow map depending on light/view orientation.
        .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
        .setDepthTest(true, true, VK_COMPARE_OP_LESS_OR_EQUAL)
        .setDepthBias(0.05f, 0.20f)
        .setNoColorAttachment()
        .setLayout(shadowPipelineLayout_)
        .setRenderPass(shadowRenderPass)
        .setDynamicStates({VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR})
        .build(device, vkCtx_->getPipelineCache());

    vertShader.destroy();
    fragShader.destroy();

    if (!shadowPipeline_) {
        LOG_ERROR("M2Renderer: failed to create shadow pipeline");
        return false;
    }
    LOG_INFO("M2Renderer shadow pipeline initialized");
    return true;
}

void M2Renderer::renderShadow(VkCommandBuffer cmd, const glm::mat4& lightSpaceMatrix, float globalTime,
                              const glm::vec3& shadowCenter, float shadowRadius) {
    if (!shadowPipeline_ || !shadowParamsSet_) return;
    if (instances.empty() || models.empty()) return;

    // Reset this frame slot's texture descriptor pool (safe: fence was waited on in beginFrame)
    const uint32_t frameIdx = vkCtx_->getCurrentFrame();
    VkDescriptorPool curShadowTexPool = shadowTexPool_[frameIdx];
    if (curShadowTexPool) {
        vkResetDescriptorPool(vkCtx_->getDevice(), curShadowTexPool, 0);
    }
    // Cache: texture imageView -> allocated descriptor set (avoids duplicates within frame)
    // Reuse persistent map — pool reset already invalidated the sets.
    shadowTexSetCache_.clear();
    auto& texSetCache = shadowTexSetCache_;

    auto getTexDescSet = [&](VkTexture* tex) -> VkDescriptorSet {
        VkImageView iv = tex->getImageView();
        auto cacheIt = texSetCache.find(iv);
        if (cacheIt != texSetCache.end()) return cacheIt->second;

        VkDescriptorSet set = VK_NULL_HANDLE;
        VkDescriptorSetAllocateInfo ai{};
        ai.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        ai.descriptorPool = curShadowTexPool;
        ai.descriptorSetCount = 1;
        ai.pSetLayouts = &shadowParamsLayout_;
        if (vkAllocateDescriptorSets(vkCtx_->getDevice(), &ai, &set) != VK_SUCCESS) {
            return shadowParamsSet_; // fallback to white texture
        }
        VkDescriptorImageInfo imgInfo{};
        imgInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        imgInfo.imageView = iv;
        imgInfo.sampler = tex->getSampler();
        VkDescriptorBufferInfo bufInfo{};
        bufInfo.buffer = shadowParamsUBO_;
        bufInfo.offset = 0;
        bufInfo.range = sizeof(ShadowParamsUBO);
        VkWriteDescriptorSet writes[2]{};
        writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[0].dstSet = set;
        writes[0].dstBinding = 0;
        writes[0].descriptorCount = 1;
        writes[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        writes[0].pImageInfo = &imgInfo;
        writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[1].dstSet = set;
        writes[1].dstBinding = 1;
        writes[1].descriptorCount = 1;
        writes[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        writes[1].pBufferInfo = &bufInfo;
        vkUpdateDescriptorSets(vkCtx_->getDevice(), 2, writes, 0, nullptr);
        texSetCache[iv] = set;
        return set;
    };

    // Helper lambda to draw instances with a given foliageSway setting
    auto drawPass = [&](bool foliagePass) {
        ShadowParamsUBO params{};
        params.foliageSway = foliagePass ? 1 : 0;
        params.windTime = globalTime;
        params.foliageMotionDamp = 1.0f;
        // For foliage pass: enable texture+alphaTest in UBO (per-batch textures bound below)
        if (foliagePass) {
            params.useTexture = 1;
            params.alphaTest = 1;
        }

        VmaAllocationInfo allocInfo{};
        vmaGetAllocationInfo(vkCtx_->getAllocator(), shadowParamsAlloc_, &allocInfo);
        std::memcpy(allocInfo.pMappedData, &params, sizeof(params));

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, shadowPipeline_);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, shadowPipelineLayout_,
            0, 1, &shadowParamsSet_, 0, nullptr);

        uint32_t currentModelId = UINT32_MAX;
        const M2ModelGPU* currentModel = nullptr;

        for (const auto& instance : instances) {
            // Use cached flags to skip early without hash lookup
            if (!instance.cachedIsValid || instance.cachedIsSmoke || instance.cachedIsInvisibleTrap) continue;

            if (!instance.cachedModel) continue;
            const M2ModelGPU& model = *instance.cachedModel;

            // Cull casters against the light-space ortho footprint, not a
            // world-space sphere around the player. The shadow frustum extends
            // ~2000 units toward the sun, so a distant tree can legitimately
            // cast across the whole view while sitting far outside any player
            // sphere — sphere culling made such shadows pop on/off at the cull
            // boundary as the player moved (large-area flicker at low sun).
            {
                const glm::vec4 clip = lightSpaceMatrix * glm::vec4(instance.position, 1.0f);
                // Orthographic projection: w == 1, NDC directly comparable.
                // Inflate by the model's bounding sphere converted to NDC
                // (shadowRadius ≈ frustum half-extent; overshoot is harmless).
                const float margin = (model.boundRadius * instance.scale) / shadowRadius * 1.5f;
                if (std::abs(clip.x) > 1.0f + margin || std::abs(clip.y) > 1.0f + margin) continue;
                if (clip.z < -margin || clip.z > 1.0f + margin) continue;
            }

            // Filter: only draw foliage models in foliage pass, non-foliage in non-foliage pass
            if (model.shadowWindFoliage != foliagePass) continue;

            // Bind vertex/index buffers when model changes
            if (instance.modelId != currentModelId) {
                currentModelId = instance.modelId;
                currentModel = &model;
                VkDeviceSize offset = 0;
                vkCmdBindVertexBuffers(cmd, 0, 1, &currentModel->vertexBuffer, &offset);
                vkCmdBindIndexBuffer(cmd, currentModel->indexBuffer, 0, VK_INDEX_TYPE_UINT16);
            }

            ShadowPush push{lightSpaceMatrix, instance.modelMatrix};
            vkCmdPushConstants(cmd, shadowPipelineLayout_, VK_SHADER_STAGE_VERTEX_BIT,
                               0, 128, &push);

            for (const auto& batch : model.batches) {
                if (batch.submeshLevel > 0) continue;
                // For foliage: bind per-batch texture for alpha-tested shadows
                if (foliagePass && batch.hasAlpha && batch.texture) {
                    VkDescriptorSet texSet = getTexDescSet(batch.texture);
                    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, shadowPipelineLayout_,
                        0, 1, &texSet, 0, nullptr);
                } else if (foliagePass) {
                    // Non-alpha batch: rebind default set (white texture, alpha test passes)
                    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, shadowPipelineLayout_,
                        0, 1, &shadowParamsSet_, 0, nullptr);
                }
                vkCmdDrawIndexed(cmd, batch.indexCount, 1, batch.indexStart, 0, 0);
            }
        }
    };

    // Pass 1: non-foliage (no wind displacement)
    drawPass(false);
    // Pass 2: foliage (wind displacement enabled, per-batch alpha-tested textures)
    drawPass(true);
}

} // namespace rendering
} // namespace wowee
