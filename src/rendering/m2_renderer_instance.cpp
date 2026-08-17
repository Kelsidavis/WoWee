#include <cstdlib>
#include "rendering/m2_renderer.hpp"
#include "rendering/m2_renderer_internal.h"
#include "rendering/m2_model_classifier.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/vk_buffer.hpp"
#include "rendering/vk_texture.hpp"
#include "rendering/vk_pipeline.hpp"
#include "rendering/vk_shader.hpp"
#include "rendering/vk_utils.hpp"
#include "rendering/vk_frame_data.hpp"
#include "rendering/camera.hpp"
#include "rendering/frustum.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/blp_loader.hpp"
#include "core/logger.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <algorithm>
#include <set>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace wowee {
namespace rendering {

// Thread-local scratch buffers for collision queries (moved from header to
// avoid inline thread_local TLS init linker errors on Windows ARM64 / LLD).
namespace m2_internal {
thread_local std::vector<size_t> tl_m2_candidateScratch;
thread_local std::unordered_set<uint32_t> tl_m2_candidateIdScratch;
thread_local std::vector<uint32_t> tl_m2_collisionTriScratch;
} // namespace m2_internal

void M2Renderer::setInstancePosition(uint32_t instanceId, const glm::vec3& position) {
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z)) return;
    auto idxIt = instanceIndexById.find(instanceId);
    if (idxIt == instanceIndexById.end()) return;
    auto& inst = instances[idxIt->second];

    // The box the instance is filed under, which is what it has to be taken
    // out of once it has moved.
    const glm::vec3 oldBoundsMin = inst.worldBoundsMin;
    const glm::vec3 oldBoundsMax = inst.worldBoundsMax;

    inst.position = position;
    inst.updateModelMatrix();
    // Use cachedModel instead of a fresh models.find() - the pointer was set
    // at addInstance and stays valid as long as the instance exists.
    if (inst.cachedModel) {
        glm::vec3 localMin, localMax;
        getTightCollisionBounds(*inst.cachedModel, localMin, localMax);
        transformAABB(inst.modelMatrix, localMin, localMax, inst.worldBoundsMin, inst.worldBoundsMax);
        inst.recomputeCachedCullFactors();
    }

    // Incrementally update spatial grid
    refileBounds(spatialGrid, oldBoundsMin, oldBoundsMax,
                 inst.worldBoundsMin, inst.worldBoundsMax, instanceId);
}

void M2Renderer::setInstanceAnimationFrozen(uint32_t instanceId, bool frozen) {
    auto idxIt = instanceIndexById.find(instanceId);
    if (idxIt == instanceIndexById.end()) return;
    auto& inst = instances[idxIt->second];
    inst.animSpeed = frozen ? 0.0f : 1.0f;
    if (frozen) {
        inst.animTime = 0.0f;  // Reset to bind pose
    }
}

void M2Renderer::setInstanceAnimation(uint32_t instanceId, uint32_t animationId, bool loop) {
    auto idxIt = instanceIndexById.find(instanceId);
    if (idxIt == instanceIndexById.end()) return;
    auto& inst = instances[idxIt->second];
    if (!inst.cachedModel) return;
    const auto& seqs = inst.cachedModel->sequences;
    // Find the first sequence matching the requested animation ID
    for (int i = 0; i < static_cast<int>(seqs.size()); ++i) {
        if (seqs[i].id == animationId) {
            inst.currentSequenceIndex = i;
            inst.animDuration = static_cast<float>(seqs[i].duration);
            inst.animTime = 0.0f;
            inst.animSpeed = 1.0f;
            // Use playingVariation=true for one-shot (returns to idle when done)
            inst.playingVariation = !loop;
            return;
        }
    }
}

bool M2Renderer::hasAnimation(uint32_t instanceId, uint32_t animationId) const {
    auto idxIt = instanceIndexById.find(instanceId);
    if (idxIt == instanceIndexById.end()) return false;
    const auto& inst = instances[idxIt->second];
    if (!inst.cachedModel) return false;
    for (const auto& seq : inst.cachedModel->sequences) {
        if (seq.id == animationId) return true;
    }
    return false;
}

float M2Renderer::getInstanceAnimDuration(uint32_t instanceId) const {
    auto idxIt = instanceIndexById.find(instanceId);
    if (idxIt == instanceIndexById.end()) return 0.0f;
    const auto& inst = instances[idxIt->second];
    if (!inst.cachedModel) return 0.0f;
    const auto& seqs = inst.cachedModel->sequences;
    if (seqs.empty()) return 0.0f;
    int seqIdx = inst.currentSequenceIndex;
    if (seqIdx < 0 || seqIdx >= static_cast<int>(seqs.size())) seqIdx = 0;
    return seqs[seqIdx].duration; // in milliseconds
}

bool M2Renderer::getInstanceBounds(uint32_t instanceId, glm::vec3& outCenter, float& outRadius) const {
    auto idxIt = instanceIndexById.find(instanceId);
    if (idxIt == instanceIndexById.end()) return false;
    const auto& inst = instances[idxIt->second];
    outCenter = inst.cachedCullCenter;
    outRadius = inst.cachedVisualRadius;
    return outRadius > 0.0f;
}

void M2Renderer::setInstanceTransform(uint32_t instanceId, const glm::mat4& transform) {
    auto idxIt = instanceIndexById.find(instanceId);
    if (idxIt == instanceIndexById.end()) return;
    // Reject NaN matrix - would propagate into the model matrix uniform
    // and the spatial-grid bounds, leaving stale grid cells pointing at
    // a NaN-bounded instance.
    for (int c = 0; c < 4; c++)
        for (int r = 0; r < 4; r++)
            if (!std::isfinite(transform[c][r])) return;
    auto& inst = instances[idxIt->second];

    // Remove old grid cells before updating bounds
    const glm::vec3 oldBoundsMin = inst.worldBoundsMin;
    const glm::vec3 oldBoundsMax = inst.worldBoundsMax;
    const glm::vec3 oldPosition = inst.position;

    // Update model matrix directly
    inst.modelMatrix = transform;
    inst.invModelMatrix = glm::inverse(transform);

    // Extract position from transform for bounds
    inst.position = glm::vec3(transform[3]);

    // The dedup map is keyed on position, so it has to move with the instance.
    // It did not, and only rebuildSpatialIndex ever put it right - which this
    // path deliberately avoids. A ship's doodads are created at the origin and
    // moved here a frame later, so the origin key stayed pointing at them and
    // the next ship of the same class was handed the first ship's sails instead
    // of its own. Both hulls then wrote their transform to the one instance, so
    // it rendered at whichever ship updated last, and whichever hull unloaded
    // first destroyed it for the other.
    if (!inst.cachedIsGroundDetail) {
        auto keyFor = [&](const glm::vec3& p) {
            return DedupKey{.modelId = inst.modelId,
                            .qx = static_cast<int32_t>(std::round(p.x * 10.0f)),
                            .qy = static_cast<int32_t>(std::round(p.y * 10.0f)),
                            .qz = static_cast<int32_t>(std::round(p.z * 10.0f))};
        };
        const DedupKey oldKey = keyFor(oldPosition);
        const DedupKey newKey = keyFor(inst.position);
        if (!(oldKey == newKey)) {
            auto oldIt = instanceDedupMap_.find(oldKey);
            // Only drop the entry if it still names this instance.
            if (oldIt != instanceDedupMap_.end() && oldIt->second == instanceId) {
                instanceDedupMap_.erase(oldIt);
            }
            instanceDedupMap_.emplace(newKey, instanceId);
        }
    }

    // Update bounds via the cached model pointer
    if (inst.cachedModel) {
        glm::vec3 localMin, localMax;
        getTightCollisionBounds(*inst.cachedModel, localMin, localMax);
        transformAABB(inst.modelMatrix, localMin, localMax, inst.worldBoundsMin, inst.worldBoundsMax);
        inst.recomputeCachedCullFactors();
    }

    // Incrementally update spatial grid (remove old cells, add new cells)
    refileBounds(spatialGrid, oldBoundsMin, oldBoundsMax,
                 inst.worldBoundsMin, inst.worldBoundsMax, instanceId);
    // No spatialIndexDirty_ = true - handled incrementally
}

void M2Renderer::removeInstance(uint32_t instanceId) {
    auto idxIt = instanceIndexById.find(instanceId);
    if (idxIt == instanceIndexById.end()) return;
    size_t idx = idxIt->second;
    if (idx >= instances.size()) return;

    auto& inst = instances[idx];

    // Remove from spatial grid incrementally (same pattern as the move-update path)
    eraseBounds(spatialGrid, inst.worldBoundsMin, inst.worldBoundsMax, instanceId);

    // Remove from dedup map
    if (!inst.cachedIsGroundDetail) {
        DedupKey dk{.modelId = inst.modelId,
                    .qx = static_cast<int32_t>(std::round(inst.position.x * 10.0f)),
                    .qy = static_cast<int32_t>(std::round(inst.position.y * 10.0f)),
                    .qz = static_cast<int32_t>(std::round(inst.position.z * 10.0f))};
        instanceDedupMap_.erase(dk);
    }

    destroyInstanceBones(inst, /*defer=*/true);

    // Swap-remove: move last element to the hole and pop_back to avoid O(n) shift
    instanceIndexById.erase(instanceId);
    if (idx < instances.size() - 1) {
        uint32_t movedId = instances.back().id;
        instances[idx] = std::move(instances.back());
        instances.pop_back();
        instanceIndexById[movedId] = idx;
    } else {
        instances.pop_back();
    }

    // Rebuild the lightweight auxiliary index vectors (smoke, portal, etc.)
    // These are small vectors of indices that are rebuilt cheaply.
    smokeInstanceIndices_.clear();
    portalInstanceIndices_.clear();
    animatedInstanceIndices_.clear();
    particleOnlyInstanceIndices_.clear();
    particleInstanceIndices_.clear();
    for (size_t i = 0; i < instances.size(); i++) {
        auto& ri = instances[i];
        if (ri.cachedIsSmoke) smokeInstanceIndices_.push_back(i);
        if (ri.cachedIsInstancePortal) portalInstanceIndices_.push_back(i);
        if (ri.cachedHasParticleEmitters) particleInstanceIndices_.push_back(i);
        if (ri.cachedHasAnimation && !ri.cachedDisableAnimation)
            animatedInstanceIndices_.push_back(i);
        else if (ri.cachedHasParticleEmitters)
            particleOnlyInstanceIndices_.push_back(i);
    }
}

void M2Renderer::setInstanceIsGameObject(uint32_t instanceId, bool isGameObject) {
    auto idxIt = instanceIndexById.find(instanceId);
    if (idxIt == instanceIndexById.end() || idxIt->second >= instances.size()) return;
    instances[idxIt->second].isGameObject = isGameObject;
}

void M2Renderer::setSkipCollision(uint32_t instanceId, bool skip) {
    for (auto& inst : instances) {
        if (inst.id == instanceId) {
            inst.skipCollision = skip;
            return;
        }
    }
}

void M2Renderer::setSkipWallCollision(uint32_t instanceId, bool skip) {
    for (auto& inst : instances) {
        if (inst.id == instanceId) {
            inst.skipWallCollision = skip;
            return;
        }
    }
}

void M2Renderer::removeInstances(const std::vector<uint32_t>& instanceIds) {
    if (instanceIds.empty() || instances.empty()) {
        return;
    }

    std::unordered_set<uint32_t> toRemove(instanceIds.begin(), instanceIds.end());
    const size_t oldSize = instances.size();
    for (auto& inst : instances) {
        if (toRemove.count(inst.id)) {
            destroyInstanceBones(inst, /*defer=*/true);
        }
    }
    instances.erase(std::remove_if(instances.begin(), instances.end(),
                   [&toRemove](const M2Instance& inst) {
                       return toRemove.find(inst.id) != toRemove.end();
                   }),
                   instances.end());

    if (instances.size() != oldSize) {
        rebuildSpatialIndex();
    }
}

void M2Renderer::clear() {
    if (vkCtx_) {
        vkDeviceWaitIdle(vkCtx_->getDevice());
        for (auto& [id, model] : models) {
            destroyModelGPU(model);
        }
        for (auto& inst : instances) {
            destroyInstanceBones(inst);
        }
        // Reset descriptor pools so new allocations succeed after reload.
        // destroyModelGPU/destroyInstanceBones don't free individual sets,
        // so the pools fill up across map changes without this reset.
        VkDevice device = vkCtx_->getDevice();
        if (materialDescPool_) {
            vkResetDescriptorPool(device, materialDescPool_, 0);
            // Re-allocate the glow texture descriptor set (pre-allocated during init,
            // invalidated by pool reset).
            if (glowTexture_ && particleTexLayout_) {
                VkDescriptorSetAllocateInfo ai{.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
                ai.descriptorPool = materialDescPool_;
                ai.descriptorSetCount = 1;
                ai.pSetLayouts = &particleTexLayout_;
                glowTexDescSet_ = VK_NULL_HANDLE;
                if (vkAllocateDescriptorSets(device, &ai, &glowTexDescSet_) == VK_SUCCESS) {
                    VkDescriptorImageInfo imgInfo = glowTexture_->descriptorInfo();
                    VkWriteDescriptorSet write{.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
                    write.dstSet = glowTexDescSet_;
                    write.dstBinding = 0;
                    write.descriptorCount = 1;
                    write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                    write.pImageInfo = &imgInfo;
                    vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);
                }
            }
        }
        if (boneDescPool_) {
            if (boneDescPoolGeneration_) boneDescPoolGeneration_->fetch_add(1, std::memory_order_relaxed);
            vkResetDescriptorPool(device, boneDescPool_, 0);
            // Re-allocate the dummy bone set (invalidated by pool reset)
            dummyBoneSet_ = allocateBoneSet();
            if (dummyBoneSet_ && dummyBoneBuffer_) {
                VkDescriptorBufferInfo bufInfo{};
                bufInfo.buffer = dummyBoneBuffer_;
                bufInfo.offset = 0;
                bufInfo.range = sizeof(glm::mat4);
                VkWriteDescriptorSet write{.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
                write.dstSet = dummyBoneSet_;
                write.dstBinding = 0;
                write.descriptorCount = 1;
                write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                write.pBufferInfo = &bufInfo;
                vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);
            }
            // Re-allocate mega bone sets (invalidated by pool reset)
            for (int i = 0; i < 2; i++) {
                megaBoneSet_[i] = allocateBoneSet();
                if (megaBoneSet_[i] && megaBoneBuffer_[i]) {
                    VkDescriptorBufferInfo mbInfo{};
                    mbInfo.buffer = megaBoneBuffer_[i];
                    mbInfo.offset = 0;
                    mbInfo.range = VkDeviceSize(MEGA_BONE_MATRIX_CAPACITY) * sizeof(glm::mat4);
                    VkWriteDescriptorSet mw{.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
                    mw.dstSet = megaBoneSet_[i];
                    mw.dstBinding = 0;
                    mw.descriptorCount = 1;
                    mw.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                    mw.pBufferInfo = &mbInfo;
                    vkUpdateDescriptorSets(device, 1, &mw, 0, nullptr);
                }
            }
        }
    }
    models.clear();
    pinnedModelIds_.clear();
    instances.clear();
    spatialGrid.clear();
    instanceIndexById.clear();
    boneSeedInstanceByModel_.clear();
    instanceDedupMap_.clear();
    for (auto& ids : cullSubmittedIds_) ids.clear();
    for (auto& ids : cullReadableIds_) ids.clear();
    smokeParticles.clear();
    smokeInstanceIndices_.clear();
    portalInstanceIndices_.clear();
    animatedInstanceIndices_.clear();
    particleOnlyInstanceIndices_.clear();
    particleInstanceIndices_.clear();
    smokeEmitAccum = 0.0f;

    // Clear texture cache so stale textures don't block loads for the next
    // character/map.  Without this, the old session's textures fill the cache
    // budget and failedTextureRetryAt_ blocks legitimate reloads, causing an
    // infinite model-load loop on character switch.
    textureCache.clear();
    texturePropsByPtr_.clear();
    textureCacheBytes_ = 0;
    textureCacheCounter_ = 0;
    failedTextureCache_.clear();
    failedTextureRetryAt_.clear();
    loggedTextureLoadFails_.clear();
    textureLookupSerial_ = 0;
    textureBudgetRejectWarnings_ = 0;
}

void M2Renderer::clearInstances() {
    if (vkCtx_) vkDeviceWaitIdle(vkCtx_->getDevice());
    for (auto& inst : instances) destroyInstanceBones(inst);
    instances.clear();
    spatialGrid.clear();
    instanceIndexById.clear();
    boneSeedInstanceByModel_.clear();
    instanceDedupMap_.clear();
    for (auto& ids : cullSubmittedIds_) ids.clear();
    for (auto& ids : cullReadableIds_) ids.clear();
    smokeInstanceIndices_.clear();
    portalInstanceIndices_.clear();
    animatedInstanceIndices_.clear();
    particleOnlyInstanceIndices_.clear();
    particleInstanceIndices_.clear();
    smokeParticles.clear();
    smokeEmitAccum = 0.0f;
}

void M2Renderer::setCollisionFocus(const glm::vec3& worldPos, float radius) {
    collisionFocus.set(worldPos, radius);
}

void M2Renderer::resetQueryStats() {
    queryTimeMs = 0.0;
    queryCallCount = 0;
}

void M2Renderer::rebuildSpatialIndex() {
    spatialGrid.clear();
    instanceIndexById.clear();
    boneSeedInstanceByModel_.clear();
    instanceDedupMap_.clear();
    instanceIndexById.reserve(instances.size());
    smokeInstanceIndices_.clear();
    portalInstanceIndices_.clear();
    animatedInstanceIndices_.clear();
    particleOnlyInstanceIndices_.clear();
    particleInstanceIndices_.clear();

    for (size_t i = 0; i < instances.size(); i++) {
        auto& inst = instances[i];
        instanceIndexById[inst.id] = i;

        // Re-cache model pointer (may have changed after model map modifications)
        auto mdlIt = models.find(inst.modelId);
        inst.cachedModel = (mdlIt != models.end()) ? &mdlIt->second : nullptr;

        // Rebuild dedup map (skip ground detail)
        if (!inst.cachedIsGroundDetail) {
            DedupKey dk{.modelId = inst.modelId,
                        .qx = static_cast<int32_t>(std::round(inst.position.x * 10.0f)),
                        .qy = static_cast<int32_t>(std::round(inst.position.y * 10.0f)),
                        .qz = static_cast<int32_t>(std::round(inst.position.z * 10.0f))};
            instanceDedupMap_[dk] = inst.id;
        }

        if (inst.cachedIsSmoke) {
            smokeInstanceIndices_.push_back(i);
        }
        if (inst.cachedIsInstancePortal) {
            portalInstanceIndices_.push_back(i);
        }
        if (inst.cachedHasParticleEmitters) {
            particleInstanceIndices_.push_back(i);
        }
        if (inst.cachedHasAnimation && !inst.cachedDisableAnimation) {
            animatedInstanceIndices_.push_back(i);
        } else if (inst.cachedHasParticleEmitters) {
            particleOnlyInstanceIndices_.push_back(i);
        }

        insertBounds(spatialGrid, inst.worldBoundsMin, inst.worldBoundsMax, inst.id);
    }
    spatialIndexDirty_ = false;
}

void M2Renderer::gatherCandidates(const glm::vec3& queryMin, const glm::vec3& queryMax,
                                  std::vector<size_t>& outIndices) const {
    outIndices.clear();

    gatherIds(spatialGrid, queryMin, queryMax, tl_m2_candidateIdScratch,
              [&](uint32_t id) {
                  auto idxIt = instanceIndexById.find(id);
                  if (idxIt != instanceIndexById.end()) {
                      outIndices.push_back(idxIt->second);
                  }
              });

    // Safety fallback to preserve collision correctness if the spatial index
    // misses candidates (e.g. during streaming churn).
    if (outIndices.empty() && !instances.empty()) {
        outIndices.reserve(instances.size());
        for (size_t i = 0; i < instances.size(); i++) {
            outIndices.push_back(i);
        }
    }
}

void M2Renderer::setModelPinned(uint32_t modelId, bool pinned) {
    if (pinned) {
        pinnedModelIds_.insert(modelId);
        modelUnusedSince_.erase(modelId);
    } else {
        pinnedModelIds_.erase(modelId);
    }
}

void M2Renderer::cleanupUnusedModels() {
    // Build set of model IDs that are still referenced by instances
    std::unordered_set<uint32_t> usedModelIds;
    for (const auto& instance : instances) {
        usedModelIds.insert(instance.modelId);
    }

    const auto now = std::chrono::steady_clock::now();
    constexpr auto kGracePeriod = std::chrono::seconds(60);

    // Find models with no instances that have exceeded the grace period.
    // Models that just lost their last instance get tracked but not evicted
    // immediately - this prevents thrashing when GO models are briefly
    // instance-free between despawn and respawn cycles.
    std::vector<uint32_t> toRemove;
    for (const auto& [id, model] : models) {
        if (usedModelIds.find(id) != usedModelIds.end() ||
            pinnedModelIds_.find(id) != pinnedModelIds_.end()) {
            // Model still in use or pinned - clear any pending unused timestamp
            modelUnusedSince_.erase(id);
            continue;
        }
        auto unusedIt = modelUnusedSince_.find(id);
        if (unusedIt == modelUnusedSince_.end()) {
            // First cycle with no instances - start the grace timer
            modelUnusedSince_[id] = now;
        } else if (now - unusedIt->second >= kGracePeriod) {
            // Grace period expired - mark for removal
            toRemove.push_back(id);
            modelUnusedSince_.erase(unusedIt);
        }
    }

    // Delete GPU resources and remove from map.
    // Wait for the GPU to finish all in-flight frames before destroying any
    // buffers - the previous frame's command buffer may still be referencing
    // vertex/index buffers that are about to be freed. Without this wait,
    // the GPU reads freed memory, which can cause VK_ERROR_DEVICE_LOST.
    // Suspected (not yet confirmed) contributor to multi-second freezes seen
    // right after taxi landings - timed here so the next repro pins it down.
    if (!toRemove.empty() && vkCtx_) {
        const auto waitStart = std::chrono::steady_clock::now();
        vkDeviceWaitIdle(vkCtx_->getDevice());
        const float waitMs = std::chrono::duration<float, std::milli>(
            std::chrono::steady_clock::now() - waitStart).count();
        LOG_DEBUG("M2 cleanup: vkDeviceWaitIdle took ", waitMs, "ms (", toRemove.size(), " models to remove)");
    }
    for (uint32_t id : toRemove) {
        auto it = models.find(id);
        if (it != models.end()) {
            destroyModelGPU(it->second);
            models.erase(it);
            // Record the eviction so owners caching "uploaded" model IDs can
            // drop it; otherwise their next spawn skips the load as a stale hit
            // and pushes an empty placeholder (missing doodads on revisit).
            reapedModelIds_.push_back(id);
        }
    }

    if (!toRemove.empty()) {
        LOG_INFO("M2 cleanup: removed ", toRemove.size(), " unused models, ", models.size(), " remaining");
    }
}

std::vector<uint32_t> M2Renderer::drainReapedModelIds() {
    std::vector<uint32_t> out;
    out.swap(reapedModelIds_);
    return out;
}

void M2Renderer::unloadModel(uint32_t modelId) {
    auto it = models.find(modelId);
    if (it == models.end()) return;
    if (vkCtx_) vkDeviceWaitIdle(vkCtx_->getDevice());
    destroyModelGPU(it->second);
    models.erase(it);
    modelUnusedSince_.erase(modelId);
    pinnedModelIds_.erase(modelId);
    reapedModelIds_.push_back(modelId);
}

VkTexture* M2Renderer::loadTexture(const std::string& path, uint32_t texFlags) {
    constexpr uint64_t kFailedTextureRetryLookups = 512;
    auto normalizeKey = [](std::string key) {
        std::replace(key.begin(), key.end(), '/', '\\');
        std::transform(key.begin(), key.end(), key.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return key;
    };
    std::string key = normalizeKey(path);
    const uint64_t lookupSerial = ++textureLookupSerial_;

    // Check cache
    auto it = textureCache.find(key);
    if (it != textureCache.end()) {
        it->second.lastUse = ++textureCacheCounter_;
        return it->second.texture.get();
    }
    auto failIt = failedTextureRetryAt_.find(key);
    if (failIt != failedTextureRetryAt_.end() && lookupSerial < failIt->second) {
        return whiteTexture_.get();
    }

    // The black key discards every pixel darker than the threshold, so a
    // texture it is applied to wrongly loses its dark areas.
    //
    // Both the list and what it is matched against now live in
    // assetNameLooksLikeFlame. This asked the whole path, which is what made
    // Outland's sky flicker: HellFireSkyNebula01 has "fire" in it, so every
    // layer of the Hellfire sky was keyed as a flame and lost whichever of its
    // dark pixels fell under the threshold that frame.
    const bool colorKeyBlackHint = assetNameLooksLikeFlame(key);

    // Check pre-decoded BLP cache first (populated by background worker threads)
    pipeline::BLPImage blp;
    if (predecodedBLPCache_) {
        auto pit = predecodedBLPCache_->find(key);
        if (pit != predecodedBLPCache_->end()) {
            blp = std::move(pit->second);
            predecodedBLPCache_->erase(pit);
        }
    }
    if (!blp.isValid()) {
        // M2 skins are sampled and their transparency is read from the
        // blocks, so they need no decode.
        blp = assetManager->loadTexture(key, true);
    }
    if (!blp.isValid()) {
        // Cache misses briefly to avoid repeated expensive MPQ/disk probes.
        failedTextureCache_.insert(key);
        failedTextureRetryAt_[key] = lookupSerial + kFailedTextureRetryLookups;
        if (loggedTextureLoadFails_.insert(key).second) {
            LOG_WARNING("M2: Failed to load texture: ", path);
        }
        return whiteTexture_.get();
    }

    const size_t approxBytes = blp.approxUploadBytes();
    if (textureCacheBytes_ + approxBytes > textureCacheBudgetBytes_) {
        static constexpr size_t kMaxFailedTextureCache = 200000;
        if (failedTextureCache_.size() < kMaxFailedTextureCache) {
            // Cache budget-rejected keys too; without this we repeatedly decode/load
            // the same textures every frame once budget is saturated.
            failedTextureCache_.insert(key);
            failedTextureRetryAt_[key] = lookupSerial + kFailedTextureRetryLookups;
        }
        if (textureBudgetRejectWarnings_ < 3) {
            LOG_WARNING("M2 texture cache full (", textureCacheBytes_ / (1024 * 1024),
                        " MB / ", textureCacheBudgetBytes_ / (1024 * 1024),
                        " MB), rejecting texture: ", path);
        }
        ++textureBudgetRejectWarnings_;
        return whiteTexture_.get();
    }

    // Whether the texture actually uses alpha. Reads the DXT blocks when the
    // loader kept them, and every fourth decoded byte when it did not; the two
    // agree, which tests/test_blp_alpha_scan.cpp checks against the assets.
    // This was the only thing forcing M2 textures to be decoded.
    const bool hasAlpha = blp.hasTransparency();

    // Create Vulkan texture
    auto tex = std::make_unique<VkTexture>();
    tex->uploadBLP(*vkCtx_, blp);

    // M2Texture flags: bit 0 = WrapS (1=repeat, 0=clamp), bit 1 = WrapT
    VkSamplerAddressMode wrapS = (texFlags & 0x1) ? VK_SAMPLER_ADDRESS_MODE_REPEAT : VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    VkSamplerAddressMode wrapT = (texFlags & 0x2) ? VK_SAMPLER_ADDRESS_MODE_REPEAT : VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    // WOWEE_SKY_MIP_BIAS pushes the sky's textures toward smaller mips.
    //
    // Every per-frame quantity behind the sky has now been measured and holds
    // still while it flickers: the lighting, the clock, the culling, the frame
    // time, and every one of the thirty-four batches' material flags. Nothing
    // is alpha tested, colour keyed, or drawn as a glow card. What is left is
    // the texture sample, and the sky is the one model where that is loud:
    // eighteen of its layers are additive, so each layer's sampling noise adds
    // to the last rather than replacing it, and the dome is at its most
    // foreshortened exactly where a turn sweeps fastest.
    //
    // A knob rather than a fixed value, because if this is the cause the right
    // amount is a thing to measure and not to guess.
    static const float skyMipBias = [] {
        const char* set = std::getenv("WOWEE_SKY_MIP_BIAS");
        return set ? std::strtof(set, nullptr) : 0.0f;
    }();
    tex->createSampler(vkCtx_->getDevice(), VK_FILTER_LINEAR, wrapS, wrapT,
                       16.0f, skyMode_ ? skyMipBias : 0.0f);

    VkTexture* texPtr = tex.get();

    TextureCacheEntry e;
    e.texture = std::move(tex);
    e.approxBytes = approxBytes;
    e.hasAlpha = hasAlpha;
    e.colorKeyBlack = colorKeyBlackHint;
    e.lastUse = ++textureCacheCounter_;
    textureCacheBytes_ += e.approxBytes;
    textureCache[key] = std::move(e);
    failedTextureCache_.erase(key);
    failedTextureRetryAt_.erase(key);
    texturePropsByPtr_[texPtr] = {.hasAlpha = hasAlpha, .colorKeyBlack = colorKeyBlackHint};
    LOG_DEBUG("M2: Loaded texture: ", path, " (", blp.width, "x", blp.height, ")");

    return texPtr;
}

uint32_t M2Renderer::getTotalTriangleCount() const {
    uint32_t total = 0;
    for (const auto& instance : instances) {
        if (instance.cachedModel) {
            total += instance.cachedModel->indexCount / 3;
        }
    }
    return total;
}

std::optional<float> M2Renderer::getFloorHeight(float glX, float glY, float glZ, float* outNormalZ) const {
    QueryTimer timer(&queryTimeMs, &queryCallCount);
    std::optional<float> bestFloor;
    float bestNormalZ = 1.0f;  // Default to flat

    glm::vec3 queryMin(glX - 2.0f, glY - 2.0f, glZ - 6.0f);
    glm::vec3 queryMax(glX + 2.0f, glY + 2.0f, glZ + 8.0f);
    gatherCandidates(queryMin, queryMax, tl_m2_candidateScratch);

    for (size_t idx : tl_m2_candidateScratch) {
        const auto& instance = instances[idx];
        if (collisionFocus.excludes(instance.worldBoundsMin,
                                    instance.worldBoundsMax)) {
            continue;
        }

        if (!instance.cachedModel) continue;
        if (instance.scale <= 0.001f) continue;

        const M2ModelGPU& model = *instance.cachedModel;
        // A model that ships collision geometry blocks, whatever its name
        // suggests. collisionNoBlock is a guess from the name and the unscaled
        // bounds, and it is only there to stop collision being *invented* for
        // something soft - it has no business discarding what the artist
        // authored.
        //
        // It was discarding a great deal. hellfiretreethorns03 carries 15,376
        // collision triangles and is 2.87 wide unscaled, so it failed the
        // treeWithTrunk gate of horiz > 6, came out a softTree, and was walked
        // through. Every bush and rug beside it ships zero collision triangles,
        // so they stay soft on their own evidence and need no rule at all.
        const bool authoredCollision = model.collision.valid();
        if ((model.collisionNoBlock && !authoredCollision) ||
            model.isInvisibleTrap || model.isSpellEffect) continue;
        if (instance.skipCollision) continue;

        // --- Mesh-based floor: vertical ray vs collision triangles ---
        // A successful authored-mesh hit wins; AABB remains the fallback on a miss.
        if (model.collision.valid()) {
            // Cast world-down through the instance transform. Using local -Z
            // only works for yaw-only placements; pitched flat models (notably
            // Exodarplatform01, placed as the Azuremyst flight-master ramp)
            // otherwise see a ray that is diagonal relative to world gravity.
            const glm::vec3 localRayOrigin = glm::vec3(
                instance.invModelMatrix * glm::vec4(glX, glY, glZ + 5.0f, 1.0f));
            const glm::vec3 localRayEnd = glm::vec3(
                instance.invModelMatrix * glm::vec4(glX, glY, glZ - 10.0f, 1.0f));
            glm::vec3 localRayVector = localRayEnd - localRayOrigin;
            const float localRayLength = glm::length(localRayVector);
            if (localRayLength <= 1e-5f) continue;
            const glm::vec3 localRayDir = localRayVector / localRayLength;

            model.collision.getFloorTrisInRange(
                std::min(localRayOrigin.x, localRayEnd.x) - 1.0f,
                std::min(localRayOrigin.y, localRayEnd.y) - 1.0f,
                std::max(localRayOrigin.x, localRayEnd.x) + 1.0f,
                std::max(localRayOrigin.y, localRayEnd.y) + 1.0f,
                tl_m2_collisionTriScratch);

            float bestWorldHitZ = -std::numeric_limits<float>::max();
            float bestHitNormalZ = 1.0f;
            bool hitAny = false;

            for (uint32_t ti : tl_m2_collisionTriScratch) {
                if (ti >= model.collision.triCount) continue;

                const auto& verts = model.collision.vertices;
                const auto& idx   = model.collision.indices;
                const auto& v0 = verts[idx[ti * 3]];
                const auto& v1 = verts[idx[ti * 3 + 1]];
                const auto& v2 = verts[idx[ti * 3 + 2]];

                // The intersection is already two-sided, so the reversed
                // winding this used to retry answers the same thing.
                const float tHit = rayTriangleIntersect(localRayOrigin, localRayDir, v0, v1, v2);
                if (tHit < 0.0f || tHit > localRayLength) continue;

                const glm::vec3 localHit = localRayOrigin + localRayDir * tHit;
                const glm::vec3 worldHit = glm::vec3(
                    instance.modelMatrix * glm::vec4(localHit, 1.0f));

                // Walkable normal check (world space)
                glm::vec3 worldN(0.0f, 0.0f, 1.0f);  // Default to flat
                glm::vec3 localN = glm::cross(v1 - v0, v2 - v0);
                float nLen = glm::length(localN);
                if (nLen > 0.001f) {
                    localN /= nLen;
                    if (localN.z < 0.0f) localN = -localN;
                    glm::vec3 transformedN = glm::vec3(
                        instance.modelMatrix * glm::vec4(localN, 0.0f));
                    float wnLen = glm::length(transformedN);
                    if (wnLen > 0.001f) {
                        worldN = transformedN / wnLen;
                    } // else: keep worldN = (0,0,1) flat default
                    if (std::abs(worldN.z) < 0.35f) continue; // too steep (~70° max slope)
                }

                if (worldHit.z <= glZ + 3.0f && worldHit.z > bestWorldHitZ) {
                    bestWorldHitZ = worldHit.z;
                    hitAny = true;
                    bestHitNormalZ = std::abs(worldN.z);
                }
            }

            if (hitAny) {
                if (!bestFloor || bestWorldHitZ > *bestFloor) {
                    bestFloor = bestWorldHitZ;
                    bestNormalZ = bestHitNormalZ;
                }
                // A successful authored-mesh hit is more accurate than the
                // model AABB top, especially for pitched ramps where the AABB
                // projection is not a world-vertical intersection.
                continue;
            }
            // Fall through to AABB floor - both contribute, highest wins
        }

        float zMargin = model.collisionBridge ? 25.0f : 2.0f;
        if (glX < instance.worldBoundsMin.x || glX > instance.worldBoundsMax.x ||
            glY < instance.worldBoundsMin.y || glY > instance.worldBoundsMax.y ||
            glZ < instance.worldBoundsMin.z - zMargin || glZ > instance.worldBoundsMax.z + zMargin) {
            continue;
        }
        glm::vec3 localMin, localMax;
        getTightCollisionBounds(model, localMin, localMax);

        glm::vec3 localPos = glm::vec3(instance.invModelMatrix * glm::vec4(glX, glY, glZ, 1.0f));

        // Must be within doodad footprint in local XY.
        // Stepped low platforms get a small pad so walk-up snapping catches edges.
        float footprintPad = 0.0f;
        if (model.collisionSteppedLowPlatform) {
            footprintPad = model.collisionPlanter ? 0.22f : 0.16f;
            if (model.collisionBridge) {
                footprintPad = 0.35f;
            }
        }
        if (localPos.x < localMin.x - footprintPad || localPos.x > localMax.x + footprintPad ||
            localPos.y < localMin.y - footprintPad || localPos.y > localMax.y + footprintPad) {
            continue;
        }

        // Construct "top" point at queried XY in local space, then transform back.
        float localTopZ = getEffectiveCollisionTopLocal(model, localPos, localMin, localMax);
        glm::vec3 localTop(localPos.x, localPos.y, localTopZ);
        glm::vec3 worldTop = glm::vec3(instance.modelMatrix * glm::vec4(localTop, 1.0f));

        // Reachability filter: allow a bit more climb for stepped low platforms.
        float maxStepUp = 1.0f;
        if (model.collisionStatue) {
            maxStepUp = 2.5f;
        } else if (model.collisionSmallSolidProp) {
            maxStepUp = 2.0f;
        } else if (model.collisionSteppedFountain) {
            maxStepUp = 2.5f;
        } else if (model.collisionSteppedLowPlatform) {
            maxStepUp = model.collisionPlanter ? 3.0f : 2.4f;
            if (model.collisionBridge) {
                maxStepUp = 25.0f;
            }
        }
        if (worldTop.z > glZ + maxStepUp) continue;

        if (!bestFloor || worldTop.z > *bestFloor) {
            bestFloor = worldTop.z;
        }
    }

    // Output surface normal if requested
    if (outNormalZ) {
        *outNormalZ = bestNormalZ;
    }

    return bestFloor;
}

bool M2Renderer::checkCollision(const glm::vec3& from, const glm::vec3& to,
                                 glm::vec3& adjustedPos, float playerRadius) const {
    QueryTimer timer(&queryTimeMs, &queryCallCount);
    adjustedPos = to;
    bool collided = false;

    glm::vec3 queryMin = glm::min(from, to) - glm::vec3(7.0f, 7.0f, 5.0f);
    glm::vec3 queryMax = glm::max(from, to) + glm::vec3(7.0f, 7.0f, 5.0f);
    gatherCandidates(queryMin, queryMax, tl_m2_candidateScratch);

    // Check against all M2 instances in local space (rotation-aware).
    for (size_t idx : tl_m2_candidateScratch) {
        const auto& instance = instances[idx];
        if (collisionFocus.excludes(instance.worldBoundsMin,
                                    instance.worldBoundsMax)) {
            continue;
        }

        const float broadMargin = playerRadius + 1.0f;
        if (from.x < instance.worldBoundsMin.x - broadMargin && adjustedPos.x < instance.worldBoundsMin.x - broadMargin) continue;
        if (from.x > instance.worldBoundsMax.x + broadMargin && adjustedPos.x > instance.worldBoundsMax.x + broadMargin) continue;
        if (from.y < instance.worldBoundsMin.y - broadMargin && adjustedPos.y < instance.worldBoundsMin.y - broadMargin) continue;
        if (from.y > instance.worldBoundsMax.y + broadMargin && adjustedPos.y > instance.worldBoundsMax.y + broadMargin) continue;
        if (from.z > instance.worldBoundsMax.z + 2.5f && adjustedPos.z > instance.worldBoundsMax.z + 2.5f) continue;
        if (from.z + 2.5f < instance.worldBoundsMin.z && adjustedPos.z + 2.5f < instance.worldBoundsMin.z) continue;

        if (!instance.cachedModel) continue;

        const M2ModelGPU& model = *instance.cachedModel;
        // A model that ships collision geometry blocks, whatever its name
        // suggests. collisionNoBlock is a guess from the name and the unscaled
        // bounds, and it is only there to stop collision being *invented* for
        // something soft - it has no business discarding what the artist
        // authored.
        //
        // It was discarding a great deal. hellfiretreethorns03 carries 15,376
        // collision triangles and is 2.87 wide unscaled, so it failed the
        // treeWithTrunk gate of horiz > 6, came out a softTree, and was walked
        // through. Every bush and rug beside it ships zero collision triangles,
        // so they stay soft on their own evidence and need no rule at all.
        const bool authoredCollision = model.collision.valid();
        if ((model.collisionNoBlock && !authoredCollision) ||
            model.isInvisibleTrap || model.isSpellEffect) continue;
        if (instance.skipCollision || instance.skipWallCollision) continue;
        if (instance.scale <= 0.001f) continue;

        // --- Mesh-based wall collision: closest-point push ---
        if (model.collision.valid()) {
            glm::vec3 localFrom = glm::vec3(instance.invModelMatrix * glm::vec4(from, 1.0f));
            glm::vec3 localPos  = glm::vec3(instance.invModelMatrix * glm::vec4(adjustedPos, 1.0f));
            float localRadius = playerRadius / instance.scale;

            model.collision.getWallTrisInRange(
                std::min(localFrom.x, localPos.x) - localRadius - 1.0f,
                std::min(localFrom.y, localPos.y) - localRadius - 1.0f,
                std::max(localFrom.x, localPos.x) + localRadius + 1.0f,
                std::max(localFrom.y, localPos.y) + localRadius + 1.0f,
                tl_m2_collisionTriScratch);

            constexpr float PLAYER_HEIGHT = 2.0f;
            constexpr float MAX_TOTAL_PUSH = 0.02f; // Cap total push per instance
            bool pushed = false;
            float totalPushX = 0.0f, totalPushY = 0.0f;

            for (uint32_t ti : tl_m2_collisionTriScratch) {
                if (ti >= model.collision.triCount) continue;
                if (localPos.z + PLAYER_HEIGHT < model.collision.triBounds[ti].minZ ||
                    localPos.z > model.collision.triBounds[ti].maxZ) continue;

                // Step-up: only skip wall when player is rising (jumping over it)
                constexpr float MAX_STEP_UP = 1.2f;
                bool rising = (localPos.z > localFrom.z + 0.05f);
                if (rising && localPos.z + MAX_STEP_UP >= model.collision.triBounds[ti].maxZ) continue;

                // Early out if we already pushed enough this instance
                float totalPushSoFar = std::sqrt(totalPushX * totalPushX + totalPushY * totalPushY);
                if (totalPushSoFar >= MAX_TOTAL_PUSH) break;

                const auto& verts = model.collision.vertices;
                const auto& idx   = model.collision.indices;
                const auto& v0 = verts[idx[ti * 3]];
                const auto& v1 = verts[idx[ti * 3 + 1]];
                const auto& v2 = verts[idx[ti * 3 + 2]];

                glm::vec3 closest = closestPointOnTriangle(localPos, v0, v1, v2);
                glm::vec3 diff = localPos - closest;
                float distXY = std::sqrt(diff.x * diff.x + diff.y * diff.y);

                if (distXY < localRadius && distXY > 1e-4f) {
                    // Gentle push - very small fraction of penetration
                    float penetration = localRadius - distXY;
                    float pushDist = std::clamp(penetration * 0.08f, 0.001f, 0.015f);
                    float dx = (diff.x / distXY) * pushDist;
                    float dy = (diff.y / distXY) * pushDist;
                    localPos.x += dx;
                    localPos.y += dy;
                    totalPushX += dx;
                    totalPushY += dy;
                    pushed = true;
                } else if (distXY < 1e-4f) {
                    // On the plane - soft push along triangle normal XY
                    glm::vec3 n = glm::cross(v1 - v0, v2 - v0);
                    float nxyLen = std::sqrt(n.x * n.x + n.y * n.y);
                    if (nxyLen > 1e-4f) {
                        float pushDist = std::min(localRadius, 0.015f);
                        float dx = (n.x / nxyLen) * pushDist;
                        float dy = (n.y / nxyLen) * pushDist;
                        localPos.x += dx;
                        localPos.y += dy;
                        totalPushX += dx;
                        totalPushY += dy;
                        pushed = true;
                    }
                }
            }

            if (pushed) {
                glm::vec3 worldPos = glm::vec3(instance.modelMatrix * glm::vec4(localPos, 1.0f));
                adjustedPos.x = worldPos.x;
                adjustedPos.y = worldPos.y;
                collided = true;
                // Which doodad is in the way, once per model.
                //
                // A model that blocks and should not is reported as "I am
                // stuck on this bush", and the one thing needed to fix it -
                // the model's name - is the one thing nobody can see. The
                // classifier decides collision from that name, so without it
                // the only way forward is guessing tokens, which is how the
                // folder-token regression happened.
                //
                // Once per distinct model, not per frame: walking into
                // something touches it every frame for as long as you lean on
                // it.
                static std::set<std::string> saidBlocked;
                if (!model.name.empty() && saidBlocked.insert(model.name).second) {
                    LOG_WARNING("Collision: blocked by '", model.name,
                                "' - if this should be walked through, that is "
                                "the name the classifier needs");
                }
            }
            continue;
        }

        glm::vec3 localFrom = glm::vec3(instance.invModelMatrix * glm::vec4(from, 1.0f));
        glm::vec3 localPos = glm::vec3(instance.invModelMatrix * glm::vec4(adjustedPos, 1.0f));
        float radiusScale = model.collisionNarrowVerticalProp ? 0.45f : 1.0f;
        float localRadius = (playerRadius * radiusScale) / instance.scale;

        glm::vec3 rawMin, rawMax;
        getTightCollisionBounds(model, rawMin, rawMax);
        glm::vec3 localMin = rawMin - glm::vec3(localRadius);
        glm::vec3 localMax = rawMax + glm::vec3(localRadius);
        float effectiveTop = getEffectiveCollisionTopLocal(model, localPos, rawMin, rawMax) + localRadius;
        glm::vec2 localCenter((localMin.x + localMax.x) * 0.5f, (localMin.y + localMax.y) * 0.5f);
        float fromR = glm::length(glm::vec2(localFrom.x, localFrom.y) - localCenter);
        float toR = glm::length(glm::vec2(localPos.x, localPos.y) - localCenter);

        // Feet-based vertical overlap test: ignore objects fully above/below us.
        constexpr float PLAYER_HEIGHT = 2.0f;
        if (localPos.z + PLAYER_HEIGHT < localMin.z || localPos.z > effectiveTop) {
            continue;
        }

        bool fromInsideXY =
            (localFrom.x >= localMin.x && localFrom.x <= localMax.x &&
             localFrom.y >= localMin.y && localFrom.y <= localMax.y);
        bool fromInsideZ = (localFrom.z + PLAYER_HEIGHT >= localMin.z && localFrom.z <= effectiveTop);
        bool escapingOverlap = (fromInsideXY && fromInsideZ && (toR > fromR + 1e-4f));
        bool allowEscapeRelax = escapingOverlap && !model.collisionSmallSolidProp;

        // Swept hard clamp for taller blockers only.
        // Low/stepable objects should be climbable and not "shove" the player off.
        float maxStepUp = 1.20f;
        if (model.collisionStatue) {
            maxStepUp = 2.5f;
        } else if (model.collisionSmallSolidProp) {
            // Keep box/crate-class props hard-solid to prevent phase-through.
            maxStepUp = 0.75f;
        } else if (model.collisionSteppedFountain) {
            maxStepUp = 2.5f;
        } else if (model.collisionSteppedLowPlatform) {
            maxStepUp = model.collisionPlanter ? 2.8f : 2.4f;
            if (model.collisionBridge) {
                maxStepUp = 25.0f;
            }
        }
        bool stepableLowObject = (effectiveTop <= localFrom.z + maxStepUp);
        bool climbingAttempt = (localPos.z > localFrom.z + 0.18f);
        bool nearTop = (localFrom.z >= effectiveTop - 0.30f);
        float climbAllowance = model.collisionPlanter ? 0.95f : 0.60f;
        if (model.collisionSteppedLowPlatform && !model.collisionPlanter) {
            // Let low curb/planter blocks be stepable without sticky side shoves.
            climbAllowance = 1.00f;
        }
        if (model.collisionBridge) {
            climbAllowance = 3.0f;
        }
        if (model.collisionSmallSolidProp) {
            climbAllowance = 1.05f;
        }
        bool climbingTowardTop = climbingAttempt && (localFrom.z + climbAllowance >= effectiveTop);
        bool forceHardLateral =
            model.collisionSmallSolidProp &&
            !nearTop && !climbingTowardTop;
        if ((!stepableLowObject || forceHardLateral) && !allowEscapeRelax) {
            float tEnter = 0.0f;
            glm::vec3 sweepMax = localMax;
            sweepMax.z = std::min(sweepMax.z, effectiveTop);
            if (segmentIntersectsAABB(localFrom, localPos, localMin, sweepMax, tEnter)) {
                float tSafe = std::clamp(tEnter - 0.03f, 0.0f, 1.0f);
                glm::vec3 localSafe = localFrom + (localPos - localFrom) * tSafe;
                glm::vec3 worldSafe = glm::vec3(instance.modelMatrix * glm::vec4(localSafe, 1.0f));
                adjustedPos.x = worldSafe.x;
                adjustedPos.y = worldSafe.y;
                collided = true;
                // The other way a doodad blocks, and the one that was missing.
                //
                // The first of these lines went inside the branch for models
                // that carry a collision mesh. A model without one is stopped
                // by its bounding box here instead, so the doodads reported as
                // wrongly solid - grass among them - were exactly the ones the
                // diagnostic could not see.
                static std::set<std::string> saidBoxBlocked;
                if (!model.name.empty() && saidBoxBlocked.insert(model.name).second) {
                    LOG_WARNING("Collision: blocked by '", model.name,
                                "' (bounding box, no collision mesh) - if this "
                                "should be walked through, that is the name the "
                                "classifier needs");
                }
                continue;
            }
        }

        if (localPos.x < localMin.x || localPos.x > localMax.x ||
            localPos.y < localMin.y || localPos.y > localMax.y) {
            continue;
        }

        float pushLeft  = localPos.x - localMin.x;
        float pushRight = localMax.x - localPos.x;
        float pushBack  = localPos.y - localMin.y;
        float pushFront = localMax.y - localPos.y;

        float minPush = std::min({pushLeft, pushRight, pushBack, pushFront});
        if (allowEscapeRelax) {
            continue;
        }
        if (stepableLowObject && localFrom.z >= effectiveTop - 0.35f) {
            // Already on/near top surface: don't apply lateral push that ejects
            // the player from the object (carpets, platforms, etc).
            continue;
        }
        // Gentle fallback push for overlapping cases.
        float pushAmount;
        if (model.collisionNarrowVerticalProp) {
            pushAmount = std::clamp(minPush * 0.10f, 0.001f, 0.010f);
        } else if (model.collisionSteppedLowPlatform) {
            if (model.collisionPlanter && stepableLowObject) {
                pushAmount = std::clamp(minPush * 0.06f, 0.001f, 0.006f);
            } else {
            pushAmount = std::clamp(minPush * 0.12f, 0.003f, 0.012f);
            }
        } else if (stepableLowObject) {
            pushAmount = std::clamp(minPush * 0.12f, 0.002f, 0.015f);
        } else {
            pushAmount = std::clamp(minPush * 0.28f, 0.010f, 0.045f);
        }
        glm::vec3 localPush(0.0f);
        if (minPush == pushLeft) {
            localPush.x = -pushAmount;
        } else if (minPush == pushRight) {
            localPush.x = pushAmount;
        } else if (minPush == pushBack) {
            localPush.y = -pushAmount;
        } else {
            localPush.y = pushAmount;
        }

        glm::vec3 worldPush = glm::vec3(instance.modelMatrix * glm::vec4(localPush, 0.0f));
        adjustedPos.x += worldPush.x;
        adjustedPos.y += worldPush.y;
        collided = true;
    }

    return collided;
}

float M2Renderer::raycastBoundingBoxes(const glm::vec3& origin, const glm::vec3& direction, float maxDistance) const {
    QueryTimer timer(&queryTimeMs, &queryCallCount);
    float closestHit = maxDistance;

    glm::vec3 rayEnd = origin + direction * maxDistance;
    glm::vec3 queryMin = glm::min(origin, rayEnd) - glm::vec3(1.0f);
    glm::vec3 queryMax = glm::max(origin, rayEnd) + glm::vec3(1.0f);
    gatherCandidates(queryMin, queryMax, tl_m2_candidateScratch);

    for (size_t idx : tl_m2_candidateScratch) {
        const auto& instance = instances[idx];
        if (collisionFocus.excludes(instance.worldBoundsMin,
                                    instance.worldBoundsMax)) {
            continue;
        }

        // Cheap world-space broad-phase.
        float tEnter = 0.0f;
        glm::vec3 worldMin = instance.worldBoundsMin - glm::vec3(0.35f);
        glm::vec3 worldMax = instance.worldBoundsMax + glm::vec3(0.35f);
        if (!segmentIntersectsAABB(origin, origin + direction * maxDistance, worldMin, worldMax, tEnter)) {
            continue;
        }

        if (!instance.cachedModel) continue;

        const M2ModelGPU& model = *instance.cachedModel;
        // A model that ships collision geometry blocks, whatever its name
        // suggests. collisionNoBlock is a guess from the name and the unscaled
        // bounds, and it is only there to stop collision being *invented* for
        // something soft - it has no business discarding what the artist
        // authored.
        //
        // It was discarding a great deal. hellfiretreethorns03 carries 15,376
        // collision triangles and is 2.87 wide unscaled, so it failed the
        // treeWithTrunk gate of horiz > 6, came out a softTree, and was walked
        // through. Every bush and rug beside it ships zero collision triangles,
        // so they stay soft on their own evidence and need no rule at all.
        const bool authoredCollision = model.collision.valid();
        if ((model.collisionNoBlock && !authoredCollision) ||
            model.isInvisibleTrap || model.isSpellEffect) continue;
        glm::vec3 localMin, localMax;
        getTightCollisionBounds(model, localMin, localMax);
        // Skip tiny doodads for camera occlusion; they cause jitter and false hits.
        glm::vec3 extents = (localMax - localMin) * instance.scale;
        if (glm::dot(extents, extents) < 0.5625f) continue;

        glm::vec3 localOrigin = glm::vec3(instance.invModelMatrix * glm::vec4(origin, 1.0f));
        glm::vec3 localDir = glm::normalize(glm::vec3(instance.invModelMatrix * glm::vec4(direction, 0.0f)));
        if (!std::isfinite(localDir.x) || !std::isfinite(localDir.y) || !std::isfinite(localDir.z)) {
            continue;
        }

        // Local-space AABB slab intersection.
        glm::vec3 invDir = 1.0f / localDir;
        glm::vec3 tMin = (localMin - localOrigin) * invDir;
        glm::vec3 tMax = (localMax - localOrigin) * invDir;
        glm::vec3 t1 = glm::min(tMin, tMax);
        glm::vec3 t2 = glm::max(tMin, tMax);

        float tNear = std::max({t1.x, t1.y, t1.z});
        float tFar = std::min({t2.x, t2.y, t2.z});
        if (tNear > tFar || tFar <= 0.0f) continue;

        float tHit = tNear > 0.0f ? tNear : tFar;
        glm::vec3 localHit = localOrigin + localDir * tHit;
        glm::vec3 worldHit = glm::vec3(instance.modelMatrix * glm::vec4(localHit, 1.0f));
        float worldDist = glm::length(worldHit - origin);
        if (worldDist > 0.0f && worldDist < closestHit) {
            closestHit = worldDist;
        }
    }

    return closestHit;
}

void M2Renderer::recreatePipelines() {
    if (!vkCtx_) return;
    VkDevice device = vkCtx_->getDevice();

    // Destroy old main-pass pipelines (NOT shadow, NOT pipeline layouts)
    destroy(device, opaquePipeline_);
    destroy(device, alphaTestPipeline_);
    destroy(device, alphaPipeline_);
    destroy(device, additivePipeline_);
    destroy(device, particlePipeline_);
    destroy(device, particleAdditivePipeline_);
    destroy(device, smokePipeline_);
    destroy(device, ribbonPipeline_);
    destroy(device, ribbonAdditivePipeline_);

    // The same nine pipelines initialize() builds, built by the same
    // function. The layouts are untouched above, so it makes none.
    buildMainPassPipelines(perFrameLayout_);

    core::Logger::getInstance().info("M2Renderer: pipelines recreated");
}

} // namespace rendering
} // namespace wowee
