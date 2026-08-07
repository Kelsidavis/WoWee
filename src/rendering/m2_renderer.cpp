#include <atomic>
#include "rendering/m2_renderer.hpp"
#include "rendering/m2_renderer_internal.h"
#include "rendering/render_constants.hpp"
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

namespace {

bool envFlagEnabled(const char* key, bool defaultValue) {
    const char* raw = std::getenv(key);
    if (!raw || !*raw) return defaultValue;
    std::string v(raw);
    std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return !(v == "0" || v == "false" || v == "off" || v == "no");
}

} // namespace

void M2Instance::updateModelMatrix() {
    modelMatrix = glm::mat4(1.0f);
    modelMatrix = glm::translate(modelMatrix, position);

    // Rotation in radians, applied Z then Y then X.
    //
    // The order is not free: euler rotations do not commute, and this was
    // X then Y then Z — the exact reverse of what the WMO placements beside it
    // use, for records that carry the same rotation in the same encoding. MDDF
    // and MODF both store (rotX, rotY, rotZ) degrees and mean the same thing
    // by them, so two orders cannot both be right.
    //
    // Z-Y-X is the one that matches the file. WoW places a doodad by turning
    // it about the vertical axis, then about the two horizontal ones, which in
    // this render space — where the ADT's up axis maps to Z — is Z, then Y,
    // then X. That is what the WMO path already did.
    //
    // Only a placement with more than one non-zero component could tell the
    // difference, which is why this survived: anything standing straight up on
    // flat ground is yaw alone, and yaw alone is order-independent. A gravestone
    // set at a slight lean has two, and came out leaning the wrong way.
    modelMatrix = glm::rotate(modelMatrix, rotation.z, glm::vec3(0.0f, 0.0f, 1.0f));
    modelMatrix = glm::rotate(modelMatrix, rotation.y, glm::vec3(0.0f, 1.0f, 0.0f));
    modelMatrix = glm::rotate(modelMatrix, rotation.x, glm::vec3(1.0f, 0.0f, 0.0f));

    modelMatrix = glm::scale(modelMatrix, glm::vec3(scale));
    invModelMatrix = glm::inverse(modelMatrix);
}

void M2Instance::recomputeCachedCullFactors() {
    // Matrix instances (notably ADT tree doodads) can have an offset pivot and
    // arbitrary scale. A sphere centered at the placement origin with the M2
    // header radius can therefore exclude much of the visible canopy. Derive
    // the render-cull sphere from transformed vertex bounds instead. The
    // separate worldBounds fields intentionally remain collision bounds.
    if (cachedModel) {
        glm::vec3 visualMin(std::numeric_limits<float>::max());
        glm::vec3 visualMax(std::numeric_limits<float>::lowest());
        for (int x = 0; x < 2; ++x) {
            for (int y = 0; y < 2; ++y) {
                for (int z = 0; z < 2; ++z) {
                    const glm::vec3 local(
                        x ? cachedModel->boundMax.x : cachedModel->boundMin.x,
                        y ? cachedModel->boundMax.y : cachedModel->boundMin.y,
                        z ? cachedModel->boundMax.z : cachedModel->boundMin.z);
                    const glm::vec3 world = glm::vec3(modelMatrix * glm::vec4(local, 1.0f));
                    visualMin = glm::min(visualMin, world);
                    visualMax = glm::max(visualMax, world);
                }
            }
        }
        cachedCullCenter = (visualMin + visualMax) * 0.5f;
        cachedVisualRadius = glm::length(visualMax - visualMin) * 0.5f;
    } else {
        cachedCullCenter = position;
        cachedVisualRadius = cachedBoundRadius * scale;
    }

    float worldRadius = cachedVisualRadius;
    float cullRadius = worldRadius;
    if (cachedDisableAnimation) cullRadius = std::max(cullRadius, 3.0f);
    float factor = std::max(1.0f, cullRadius / rendering::M2_CULL_RADIUS_SCALE_DIVISOR);
    if (cachedDisableAnimation) factor *= 2.6f;
    if (cachedIsGroundDetail)   factor *= 0.9f;
    cachedEffectiveMaxDistSqFactor = factor;
    cachedPaddedRadius = std::max(cullRadius * rendering::M2_PADDED_RADIUS_SCALE,
                                  cullRadius + rendering::M2_PADDED_RADIUS_MIN_MARGIN);
}

M2Renderer::M2Renderer() {
}

M2Renderer::~M2Renderer() {
    shutdown();
}

uint32_t M2Renderer::gatherLocalLights(const glm::vec3& cameraPos,
                                       glm::vec4* outPosRadius,
                                       glm::vec4* outColorIntensity,
                                       uint32_t maxLights) const {
    if (!outPosRadius || !outColorIntensity || maxLights == 0) return 0;

    struct Candidate {
        float distSq;
        glm::vec4 posRadius;
        glm::vec4 colorIntensity;
        bool flame = false;  // lamps/torches/braziers gutter; lava burns steady
        // Fixture placement, used only to seed the flicker phase. The light's own
        // position animates with the flame, which would re-roll the phase every frame.
        glm::vec3 phaseSeed{0.0f};
    };
    std::vector<Candidate> candidates;

    for (const auto& instance : instances) {
        const M2ModelGPU* model = instance.cachedModel;
        if (!model || (!model->isLanternLike && !model->isTorch &&
                       !model->isBrazierOrFire && !model->isForge &&
                       !model->isLavaModel)) continue;

        if (model->isLavaModel) {
            const glm::vec3 worldPos = instance.cachedCullCenter;
            const glm::vec3 delta = worldPos - cameraPos;
            const float distSq = glm::dot(delta, delta);
            if (distSq <= 300.0f * 300.0f) {
                const float radius = std::clamp(instance.cachedVisualRadius * 0.8f,
                                                10.0f, 35.0f);
                candidates.push_back({distSq, glm::vec4(worldPos, radius),
                                      glm::vec4(1.0f, 0.28f, 0.035f, 1.75f),
                                      /*flame=*/false, instance.position});
            }
        }

        bool hasBatchLight = false;
        for (const auto& batch : model->batches) {
            if (!batch.glowCardLike || !batch.lanternGlowHint) continue;

            glm::vec3 worldPos;
            if (model->isGroundFire &&
                !model->particleEmitters.empty()) {
                worldPos = glm::vec3(std::numeric_limits<float>::max());
                for (const auto& emitter : model->particleEmitters) {
                    glm::mat4 boneXform(1.0f);
                    if (emitter.bone < instance.boneMatrices.size()) {
                        boneXform = instance.boneMatrices[emitter.bone];
                    }
                    const glm::vec3 emitterWorld = glm::vec3(
                        instance.modelMatrix * boneXform * glm::vec4(emitter.position, 1.0f));
                    if (emitterWorld.z < worldPos.z) worldPos = emitterWorld;
                }
            } else {
                worldPos = animatedBatchLightWorldCenter(instance, batch);
            }
            const glm::vec3 delta = worldPos - cameraPos;
            const float distSq = glm::dot(delta, delta);
            // Keep tunnel/interior fixtures resident well before their lit
            // surfaces enter view; the shader's radius still bounds the work.
            if (distSq > 300.0f * 300.0f) continue;

            const float radius = std::clamp(batch.glowSize * instance.scale * 8.0f,
                                            5.0f, 12.0f);
            glm::vec3 color(1.0f, 0.58f, 0.22f);
            if (batch.glowTint == 1) color = glm::vec3(0.42f, 0.68f, 1.0f);
            else if (batch.glowTint == 2) color = glm::vec3(1.0f, 0.24f, 0.14f);
            candidates.push_back({distSq, glm::vec4(worldPos, radius),
                                  glm::vec4(color, 1.35f), /*flame=*/true,
                                  instance.position});
            hasBatchLight = true;
        }

        // Candles, hearth fires, campfires, torches and forges express their
        // flame purely as particle emitters, with no glow-card batch for the
        // path above — so without this they lit nothing at all. One light at
        // the emitter centroid; chandeliers with an authored glow batch stay
        // represented by that batch rather than by five separate lights.
        const bool openFlame = model->isBrazierOrFire || model->isTorch ||
                               model->isGroundFire || model->isForge;
        if (!hasBatchLight && (model->isLanternLike || openFlame) &&
            !model->particleEmitters.empty()) {
            glm::vec3 worldPos(0.0f);
            uint32_t emitterCount = 0;
            for (const auto& emitter : model->particleEmitters) {
                glm::mat4 boneXform(1.0f);
                if (emitter.bone < instance.boneMatrices.size()) {
                    boneXform = instance.boneMatrices[emitter.bone];
                }
                worldPos += glm::vec3(instance.modelMatrix * boneXform *
                                      glm::vec4(emitter.position, 1.0f));
                emitterCount++;
            }
            if (emitterCount > 0) {
                worldPos /= static_cast<float>(emitterCount);
                const glm::vec3 delta = worldPos - cameraPos;
                const float distSq = glm::dot(delta, delta);
                if (distSq <= 300.0f * 300.0f) {
                    const bool chandelier =
                        model->name.find("Chandelier") != std::string::npos ||
                        model->name.find("chandelier") != std::string::npos;
                    // A hearth or campfire throws light much further than a
                    // candle, and burns oranger than a lamp wick.
                    float radius    = chandelier ? 10.0f : 5.0f;
                    float intensity = chandelier ? 1.25f : 0.85f;
                    glm::vec3 color(1.0f, 0.58f, 0.22f);
                    if (openFlame) {
                        // Local lights are not shadowed, so this radius is the
                        // distance the glow reaches straight through whatever
                        // surrounds the fire. At 11 a hearth fire lit its entire
                        // chimney from the inside out and the brickwork read as
                        // though it were glowing. Keep the reach close to the
                        // firebox and let the sprite and flames carry the rest.
                        radius    = model->isTorch ? 3.0f : 3.5f;
                        intensity = model->isTorch ? 0.95f : 1.05f;
                        color     = glm::vec3(1.0f, 0.50f, 0.18f);
                    }
                    candidates.push_back({distSq,
                        glm::vec4(worldPos, radius),
                        glm::vec4(color, intensity),
                        /*flame=*/true, instance.position});
                }
            }
        }
    }

    const uint32_t count = std::min<uint32_t>(maxLights,
        static_cast<uint32_t>(candidates.size()));
    if (count == 0) return 0;
    std::partial_sort(candidates.begin(), candidates.begin() + count, candidates.end(),
        [](const Candidate& a, const Candidate& b) { return a.distSq < b.distSq; });
    // Guttering is applied to the light itself, not just the glow sprite: the
    // pool of light a lamp throws on the ground and nearby walls is what the eye
    // actually reads as firelight, so modulating the sprite alone was too subtle
    // to notice.
    const float flickerSeconds = lampFlickerClockSeconds();
    for (uint32_t i = 0; i < count; ++i) {
        outPosRadius[i] = candidates[i].posRadius;
        outColorIntensity[i] = candidates[i].colorIntensity;
        if (candidates[i].flame) {
            outColorIntensity[i].w *= lampFlicker(candidates[i].phaseSeed,
                                                  flickerSeconds, 0.82f, 0.12f, 0.06f);
        }
    }
    return count;
}

bool M2Renderer::initialize(VkContext* ctx, VkDescriptorSetLayout perFrameLayout,
                            pipeline::AssetManager* assets) {
    if (initialized_) { assetManager = assets; return true; }
    vkCtx_ = ctx;
    assetManager = assets;

    // Announce the renderer diagnostics this build understands, and which of
    // them are active. A run that logs this line is definitely a build that has
    // them, which takes the guesswork out of "did that binary include the fix?".
    LOG_INFO("M2 render diagnostics available (NO_PARTICLES/NO_RIBBONS/NO_SKINNING): ",
             "particles=", envFlagEnabled("WOWEE_M2_NO_PARTICLES") ? "OFF" : "on",
             " ribbons=", envFlagEnabled("WOWEE_M2_NO_RIBBONS") ? "OFF" : "on",
             " skinning=", envFlagEnabled("WOWEE_M2_NO_SKINNING") ? "OFF" : "on",
             " maxBonesPerInstance=", kMaxBonesPerInstance);

    // Instance storage grows to tens of thousands as a session explores, and
    // each doubling reallocates the whole thing mid-frame: measured at 8.9ms
    // crossing 32k and 18.5ms crossing 64k, doubling again each time. Take that
    // allocation up front, where a stall is invisible.
    instances.reserve(65536);

    const unsigned hc = std::thread::hardware_concurrency();
    const size_t availableCores = (hc > 1u) ? static_cast<size_t>(hc - 1u) : 1ull;
    // Keep headroom for other frame tasks: M2 gets about half of non-main cores by default.
    const size_t defaultAnimThreads = std::max<size_t>(1, availableCores / 2);
    numAnimThreads_ = static_cast<uint32_t>(std::max<size_t>(
        1, envSizeOrDefault("WOWEE_M2_ANIM_THREADS", defaultAnimThreads)));
    LOG_INFO("Initializing M2 renderer (Vulkan, ", numAnimThreads_, " anim threads)...");

    VkDevice device = vkCtx_->getDevice();

    // --- Descriptor set layouts ---

    // Material set layout (set 1): binding 0 = sampler2D, binding 2 = M2Material UBO
    // (M2Params moved to push constants alongside model matrix)
    {
        VkDescriptorSetLayoutBinding bindings[2] = {};
        bindings[0].binding = 0;
        bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        bindings[0].descriptorCount = 1;
        bindings[0].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        bindings[1].binding = 2;
        bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        bindings[1].descriptorCount = 1;
        bindings[1].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

        VkDescriptorSetLayoutCreateInfo ci{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        ci.bindingCount = 2;
        ci.pBindings = bindings;
        vkCreateDescriptorSetLayout(device, &ci, nullptr, &materialSetLayout_);
    }

    // Bone set layout (set 2): binding 0 = STORAGE_BUFFER (bone matrices)
    {
        VkDescriptorSetLayoutBinding binding{};
        binding.binding = 0;
        binding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        binding.descriptorCount = 1;
        binding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;

        VkDescriptorSetLayoutCreateInfo ci{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        ci.bindingCount = 1;
        ci.pBindings = &binding;
        vkCreateDescriptorSetLayout(device, &ci, nullptr, &boneSetLayout_);
    }

    // Instance data set layout (set 3): binding 0 = STORAGE_BUFFER (per-instance data)
    {
        VkDescriptorSetLayoutBinding binding{};
        binding.binding = 0;
        binding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        binding.descriptorCount = 1;
        binding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;

        VkDescriptorSetLayoutCreateInfo ci{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        ci.bindingCount = 1;
        ci.pBindings = &binding;
        vkCreateDescriptorSetLayout(device, &ci, nullptr, &instanceSetLayout_);
    }

    // Particle texture set layout (set 1 for particles): binding 0 = sampler2D
    {
        VkDescriptorSetLayoutBinding binding{};
        binding.binding = 0;
        binding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        binding.descriptorCount = 1;
        binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

        VkDescriptorSetLayoutCreateInfo ci{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        ci.bindingCount = 1;
        ci.pBindings = &binding;
        vkCreateDescriptorSetLayout(device, &ci, nullptr, &particleTexLayout_);
    }

    // --- Descriptor pools ---
    {
        VkDescriptorPoolSize sizes[] = {
            {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, MAX_MATERIAL_SETS + 256},
            {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, MAX_MATERIAL_SETS + 256},
        };
        VkDescriptorPoolCreateInfo ci{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        ci.maxSets = MAX_MATERIAL_SETS + 256;
        ci.poolSizeCount = 2;
        ci.pPoolSizes = sizes;
        ci.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
        vkCreateDescriptorPool(device, &ci, nullptr, &materialDescPool_);
    }
    {
        VkDescriptorPoolSize sizes[] = {
            {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, MAX_BONE_SETS},
        };
        VkDescriptorPoolCreateInfo ci{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        ci.maxSets = MAX_BONE_SETS;
        ci.poolSizeCount = 1;
        ci.pPoolSizes = sizes;
        ci.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
        vkCreateDescriptorPool(device, &ci, nullptr, &boneDescPool_);
    }

    // Create a small identity-bone SSBO + descriptor set so that non-animated
    // draws always have a valid set 2 bound.  The Intel ANV driver segfaults
    // on vkCmdDrawIndexed when a declared descriptor set slot is unbound.
    {
        // Single identity matrix (bone 0 = identity)
        glm::mat4 identity(1.0f);
        VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        bci.size = sizeof(glm::mat4);
        bci.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
        VmaAllocationCreateInfo aci{};
        aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
        aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
        VmaAllocationInfo allocInfo{};
        vmaCreateBuffer(ctx->getAllocator(), &bci, &aci,
                        &dummyBoneBuffer_, &dummyBoneAlloc_, &allocInfo);
        if (allocInfo.pMappedData) {
            memcpy(allocInfo.pMappedData, &identity, sizeof(identity));
        }

        dummyBoneSet_ = allocateBoneSet();
        if (dummyBoneSet_) {
            VkDescriptorBufferInfo bufInfo{};
            bufInfo.buffer = dummyBoneBuffer_;
            bufInfo.offset = 0;
            bufInfo.range = sizeof(glm::mat4);
            VkWriteDescriptorSet write{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
            write.dstSet = dummyBoneSet_;
            write.dstBinding = 0;
            write.descriptorCount = 1;
            write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            write.pBufferInfo = &bufInfo;
            vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);
        }
    }

    // Mega bone SSBO — consolidates all animated instance bones into one buffer per frame.
    // Slot 0 = identity matrix (for non-animated instances), slots 1..N = animated instances.
    {
        const VkDeviceSize megaSize = VkDeviceSize(MEGA_BONE_MATRIX_CAPACITY) * sizeof(glm::mat4);
        glm::mat4 identity(1.0f);
        for (int i = 0; i < 2; i++) {
            VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
            bci.size = megaSize;
            bci.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
            VmaAllocationCreateInfo aci{};
            aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
            aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
            VmaAllocationInfo allocInfo{};
            vmaCreateBuffer(ctx->getAllocator(), &bci, &aci,
                            &megaBoneBuffer_[i], &megaBoneAlloc_[i], &allocInfo);
            megaBoneMapped_[i] = allocInfo.pMappedData;

            // Slot 0: identity matrix (for non-animated instances)
            if (megaBoneMapped_[i]) {
                memcpy(megaBoneMapped_[i], &identity, sizeof(identity));
            }

            megaBoneSet_[i] = allocateBoneSet();
            if (megaBoneSet_[i]) {
                VkDescriptorBufferInfo bufInfo{};
                bufInfo.buffer = megaBoneBuffer_[i];
                bufInfo.offset = 0;
                bufInfo.range = megaSize;
                VkWriteDescriptorSet write{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
                write.dstSet = megaBoneSet_[i];
                write.dstBinding = 0;
                write.descriptorCount = 1;
                write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                write.pBufferInfo = &bufInfo;
                vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);
            }
        }

        // Fresh buffers hold no bone data — invalidate any per-instance upload
        // tracking so prepareRender() re-uploads everything (defensive: instances
        // are normally empty when this runs, but re-init must not skip uploads).
        for (auto& inst : instances) {
            inst.megaBoneUploadedSlot[0] = inst.megaBoneUploadedSlot[1] = 0;
        }
    }

    // Instance data SSBO — per-frame buffer holding per-instance transforms, fade, bones.
    // Shader reads instanceData[push.instanceDataOffset + gl_InstanceIndex].
    {
        static_assert(sizeof(M2InstanceGPU) == 96, "M2InstanceGPU must be 96 bytes (std430)");
        const VkDeviceSize instBufSize = MAX_INSTANCE_DATA * sizeof(M2InstanceGPU);

        // Descriptor pool for 2 sets (double-buffered)
        VkDescriptorPoolSize poolSize{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 2};
        VkDescriptorPoolCreateInfo poolCi{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        poolCi.maxSets = 2;
        poolCi.poolSizeCount = 1;
        poolCi.pPoolSizes = &poolSize;
        vkCreateDescriptorPool(device, &poolCi, nullptr, &instanceDescPool_);

        for (int i = 0; i < 2; i++) {
            VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
            bci.size = instBufSize;
            bci.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
            VmaAllocationCreateInfo aci{};
            aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
            aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
            VmaAllocationInfo allocInfo{};
            vmaCreateBuffer(ctx->getAllocator(), &bci, &aci,
                            &instanceBuffer_[i], &instanceAlloc_[i], &allocInfo);
            instanceMapped_[i] = allocInfo.pMappedData;

            VkDescriptorSetAllocateInfo setAi{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
            setAi.descriptorPool = instanceDescPool_;
            setAi.descriptorSetCount = 1;
            setAi.pSetLayouts = &instanceSetLayout_;
            vkAllocateDescriptorSets(device, &setAi, &instanceSet_[i]);

            VkDescriptorBufferInfo bufInfo{};
            bufInfo.buffer = instanceBuffer_[i];
            bufInfo.offset = 0;
            bufInfo.range = instBufSize;
            VkWriteDescriptorSet write{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
            write.dstSet = instanceSet_[i];
            write.dstBinding = 0;
            write.descriptorCount = 1;
            write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            write.pBufferInfo = &bufInfo;
            vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);
        }
    }

    // GPU frustum culling — compute pipeline, buffers, descriptors.
    // Compute shader tests each instance bounding sphere against 6 frustum planes + distance.
    // Output: uint visibility[] read back by CPU to skip culled instances in sortedVisible_ build.
    {
        static_assert(sizeof(CullInstanceGPU) == 32, "CullInstanceGPU must be 32 bytes (std430)");
        static_assert(sizeof(CullUniformsGPU) == 272, "CullUniformsGPU must be 272 bytes (std140)");

        // Descriptor set layout: binding 0 = UBO (frustum+camera), 1 = SSBO (input), 2 = SSBO (output)
        VkDescriptorSetLayoutBinding bindings[3] = {};
        bindings[0].binding = 0;
        bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        bindings[0].descriptorCount = 1;
        bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        bindings[1].binding = 1;
        bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        bindings[1].descriptorCount = 1;
        bindings[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        bindings[2].binding = 2;
        bindings[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        bindings[2].descriptorCount = 1;
        bindings[2].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

        VkDescriptorSetLayoutCreateInfo layoutCi{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        layoutCi.bindingCount = 3;
        layoutCi.pBindings = bindings;
        vkCreateDescriptorSetLayout(device, &layoutCi, nullptr, &cullSetLayout_);

        // Pipeline layout (no push constants — everything via UBO)
        VkPipelineLayoutCreateInfo plCi{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        plCi.setLayoutCount = 1;
        plCi.pSetLayouts = &cullSetLayout_;
        vkCreatePipelineLayout(device, &plCi, nullptr, &cullPipelineLayout_);

        // Load compute shader
        rendering::VkShaderModule cullComp;
        if (!cullComp.loadFromFile(device, "assets/shaders/m2_cull.comp.spv")) {
            LOG_ERROR("M2Renderer: failed to load m2_cull.comp.spv — GPU culling disabled");
        } else {
            VkComputePipelineCreateInfo cpCi{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
            cpCi.stage = cullComp.stageInfo(VK_SHADER_STAGE_COMPUTE_BIT);
            cpCi.layout = cullPipelineLayout_;
            if (vkCreateComputePipelines(device, VK_NULL_HANDLE, 1, &cpCi, nullptr, &cullPipeline_) != VK_SUCCESS) {
                LOG_ERROR("M2Renderer: failed to create cull compute pipeline");
                cullPipeline_ = VK_NULL_HANDLE;
            }
            cullComp.destroy();
        }

        // HiZ-aware cull pipeline (Phase 6.3 Option B)
        // Uses set 0 (same as frustum-only) + set 1 (HiZ pyramid sampler from HiZSystem).
        // The HiZ descriptor set layout is created lazily when hizSystem_ is set, but the
        // pipeline layout and shader are created now if the shader is available.
        rendering::VkShaderModule cullHiZComp;
        if (cullHiZComp.loadFromFile(device, "assets/shaders/m2_cull_hiz.comp.spv")) {
            // HiZ cull set 1 layout: single combined image sampler (the HiZ pyramid)
            VkDescriptorSetLayoutBinding hizBinding{};
            hizBinding.binding = 0;
            hizBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            hizBinding.descriptorCount = 1;
            hizBinding.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

            VkDescriptorSetLayout hizSamplerLayout = VK_NULL_HANDLE;
            VkDescriptorSetLayoutCreateInfo hizLayoutCi{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
            hizLayoutCi.bindingCount = 1;
            hizLayoutCi.pBindings = &hizBinding;
            vkCreateDescriptorSetLayout(device, &hizLayoutCi, nullptr, &hizSamplerLayout);

            VkDescriptorSetLayout hizSetLayouts[2] = {cullSetLayout_, hizSamplerLayout};
            VkPipelineLayoutCreateInfo hizPlCi{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
            hizPlCi.setLayoutCount = 2;
            hizPlCi.pSetLayouts = hizSetLayouts;
            vkCreatePipelineLayout(device, &hizPlCi, nullptr, &cullHiZPipelineLayout_);

            VkComputePipelineCreateInfo hizCpCi{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
            hizCpCi.stage = cullHiZComp.stageInfo(VK_SHADER_STAGE_COMPUTE_BIT);
            hizCpCi.layout = cullHiZPipelineLayout_;
            if (vkCreateComputePipelines(device, VK_NULL_HANDLE, 1, &hizCpCi, nullptr, &cullHiZPipeline_) != VK_SUCCESS) {
                LOG_WARNING("M2Renderer: failed to create HiZ cull compute pipeline — HiZ disabled");
                cullHiZPipeline_ = VK_NULL_HANDLE;
                vkDestroyPipelineLayout(device, cullHiZPipelineLayout_, nullptr);
                cullHiZPipelineLayout_ = VK_NULL_HANDLE;
            } else {
                LOG_INFO("M2Renderer: HiZ occlusion cull pipeline created");
            }

            // The hizSamplerLayout is now owned by the pipeline layout; we don't track it
            // separately because the pipeline layout keeps a ref. But actually Vulkan
            // requires us to keep it alive. Store it where HiZSystem will provide it.
            // For now, we can destroy it since the pipeline layout was already created.
            vkDestroyDescriptorSetLayout(device, hizSamplerLayout, nullptr);

            cullHiZComp.destroy();
        } else {
            LOG_INFO("M2Renderer: m2_cull_hiz.comp.spv not found — HiZ occlusion culling not available");
        }

        // Descriptor pool: 2 sets × 3 descriptors each (1 UBO + 2 SSBO)
        VkDescriptorPoolSize poolSizes[2] = {};
        poolSizes[0] = {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 2};
        poolSizes[1] = {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 4};  // 2 input + 2 output
        VkDescriptorPoolCreateInfo poolCi{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        poolCi.maxSets = 2;
        poolCi.poolSizeCount = 2;
        poolCi.pPoolSizes = poolSizes;
        vkCreateDescriptorPool(device, &poolCi, nullptr, &cullDescPool_);

        const VkDeviceSize uniformSize = sizeof(CullUniformsGPU);
        const VkDeviceSize inputSize   = MAX_CULL_INSTANCES * sizeof(CullInstanceGPU);
        const VkDeviceSize outputSize  = MAX_CULL_INSTANCES * sizeof(uint32_t);

        for (int i = 0; i < 2; i++) {
            // Uniform buffer (frustum planes + camera)
            {
                VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
                bci.size = uniformSize;
                bci.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
                VmaAllocationCreateInfo aci{};
                aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
                aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
                VmaAllocationInfo ai{};
                vmaCreateBuffer(ctx->getAllocator(), &bci, &aci,
                                &cullUniformBuffer_[i], &cullUniformAlloc_[i], &ai);
                cullUniformMapped_[i] = ai.pMappedData;
            }
            // Input SSBO (per-instance cull data)
            {
                VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
                bci.size = inputSize;
                bci.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
                VmaAllocationCreateInfo aci{};
                aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
                aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
                VmaAllocationInfo ai{};
                vmaCreateBuffer(ctx->getAllocator(), &bci, &aci,
                                &cullInputBuffer_[i], &cullInputAlloc_[i], &ai);
                cullInputMapped_[i] = ai.pMappedData;
            }
            // Output SSBO (visibility flags — GPU writes, CPU reads)
            {
                VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
                bci.size = outputSize;
                bci.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
                VmaAllocationCreateInfo aci{};
                aci.usage = VMA_MEMORY_USAGE_GPU_TO_CPU;
                aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
                VmaAllocationInfo ai{};
                vmaCreateBuffer(ctx->getAllocator(), &bci, &aci,
                                &cullOutputBuffer_[i], &cullOutputAlloc_[i], &ai);
                cullOutputMapped_[i] = ai.pMappedData;
            }

            // Allocate and write descriptor set
            VkDescriptorSetAllocateInfo setAi{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
            setAi.descriptorPool = cullDescPool_;
            setAi.descriptorSetCount = 1;
            setAi.pSetLayouts = &cullSetLayout_;
            vkAllocateDescriptorSets(device, &setAi, &cullSet_[i]);

            VkDescriptorBufferInfo uboInfo{cullUniformBuffer_[i], 0, uniformSize};
            VkDescriptorBufferInfo inputInfo{cullInputBuffer_[i], 0, inputSize};
            VkDescriptorBufferInfo outputInfo{cullOutputBuffer_[i], 0, outputSize};

            VkWriteDescriptorSet writes[3] = {};
            writes[0] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
            writes[0].dstSet = cullSet_[i];
            writes[0].dstBinding = 0;
            writes[0].descriptorCount = 1;
            writes[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
            writes[0].pBufferInfo = &uboInfo;

            writes[1] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
            writes[1].dstSet = cullSet_[i];
            writes[1].dstBinding = 1;
            writes[1].descriptorCount = 1;
            writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            writes[1].pBufferInfo = &inputInfo;

            writes[2] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
            writes[2].dstSet = cullSet_[i];
            writes[2].dstBinding = 2;
            writes[2].descriptorCount = 1;
            writes[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            writes[2].pBufferInfo = &outputInfo;

            vkUpdateDescriptorSets(device, 3, writes, 0, nullptr);
        }
    }

    // --- Pipeline layouts ---

    // Main M2 pipeline layout: set 0 = perFrame, set 1 = material, set 2 = bones, set 3 = instances
    // Push constant: int texCoordSet + int isFoliage + int instanceDataOffset (12 bytes)
    {
        VkDescriptorSetLayout setLayouts[] = {perFrameLayout, materialSetLayout_, boneSetLayout_, instanceSetLayout_};
        VkPushConstantRange pushRange{};
        pushRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
        pushRange.offset = 0;
        pushRange.size = 12; // int texCoordSet + int isFoliage + int instanceDataOffset

        VkPipelineLayoutCreateInfo ci{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        ci.setLayoutCount = 4;
        ci.pSetLayouts = setLayouts;
        ci.pushConstantRangeCount = 1;
        ci.pPushConstantRanges = &pushRange;
        vkCreatePipelineLayout(device, &ci, nullptr, &pipelineLayout_);
    }

    // Particle pipeline layout: set 0 = perFrame, set 1 = particleTex
    // Push constant: vec2 tileCount + int alphaKey (12 bytes)
    {
        VkDescriptorSetLayout setLayouts[] = {perFrameLayout, particleTexLayout_};
        VkPushConstantRange pushRange{};
        pushRange.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        pushRange.offset = 0;
        pushRange.size = 12; // vec2 + int

        VkPipelineLayoutCreateInfo ci{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        ci.setLayoutCount = 2;
        ci.pSetLayouts = setLayouts;
        ci.pushConstantRangeCount = 1;
        ci.pPushConstantRanges = &pushRange;
        vkCreatePipelineLayout(device, &ci, nullptr, &particlePipelineLayout_);
    }

    // Smoke pipeline layout: set 0 = perFrame
    // Push constant: float screenHeight (4 bytes)
    {
        VkDescriptorSetLayout setLayouts[] = {perFrameLayout};
        VkPushConstantRange pushRange{};
        pushRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
        pushRange.offset = 0;
        pushRange.size = 4;

        VkPipelineLayoutCreateInfo ci{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        ci.setLayoutCount = 1;
        ci.pSetLayouts = setLayouts;
        ci.pushConstantRangeCount = 1;
        ci.pPushConstantRanges = &pushRange;
        vkCreatePipelineLayout(device, &ci, nullptr, &smokePipelineLayout_);
    }

    // --- Load shaders ---
    rendering::VkShaderModule m2Vert, m2Frag;
    rendering::VkShaderModule particleVert, particleFrag;
    rendering::VkShaderModule smokeVert, smokeFrag;

    (void)m2Vert.loadFromFile(device, "assets/shaders/m2.vert.spv");
    (void)m2Frag.loadFromFile(device, "assets/shaders/m2.frag.spv");
    (void)particleVert.loadFromFile(device, "assets/shaders/m2_particle.vert.spv");
    (void)particleFrag.loadFromFile(device, "assets/shaders/m2_particle.frag.spv");
    (void)smokeVert.loadFromFile(device, "assets/shaders/m2_smoke.vert.spv");
    (void)smokeFrag.loadFromFile(device, "assets/shaders/m2_smoke.frag.spv");

    if (!m2Vert.isValid() || !m2Frag.isValid()) {
        LOG_ERROR("M2: Missing required shaders, cannot initialize");
        return false;
    }

    VkRenderPass mainPass = vkCtx_->getImGuiRenderPass();

    // --- Build M2 model pipelines ---
    // Vertex input: 18 floats = 72 bytes stride
    // loc 0: vec3 pos (0), loc 1: vec3 normal (12), loc 2: vec2 uv0 (24),
    // loc 5: vec2 uv1 (32), loc 3: vec4 boneWeights (40), loc 4: vec4 boneIndices (56)
    VkVertexInputBindingDescription m2Binding{};
    m2Binding.binding = 0;
    m2Binding.stride = 18 * sizeof(float);
    m2Binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    std::vector<VkVertexInputAttributeDescription> m2Attrs = {
        {0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},                     // position
        {1, 0, VK_FORMAT_R32G32B32_SFLOAT, 3 * sizeof(float)},     // normal
        {2, 0, VK_FORMAT_R32G32_SFLOAT, 6 * sizeof(float)},        // texCoord0
        {5, 0, VK_FORMAT_R32G32_SFLOAT, 8 * sizeof(float)},        // texCoord1
        {3, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 10 * sizeof(float)}, // boneWeights
        {4, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 14 * sizeof(float)}, // boneIndices (float)
    };

    // Pipeline derivatives — opaque is the base, others derive from it for shared state optimization
    auto buildM2Pipeline = [&](VkPipelineColorBlendAttachmentState blendState, bool depthWrite,
                               VkPipelineCreateFlags flags = 0, VkPipeline basePipeline = VK_NULL_HANDLE,
                               bool alphaToCoverage = false) -> VkPipeline {
        auto builder = PipelineBuilder()
            .setShaders(m2Vert.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                        m2Frag.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
            .setVertexInput({m2Binding}, m2Attrs)
            .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
            .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
            .setDepthTest(!skyMode_, skyMode_ ? false : depthWrite,
                          skyMode_ ? VK_COMPARE_OP_ALWAYS : VK_COMPARE_OP_LESS_OR_EQUAL)
            .setColorBlendAttachment(blendState)
            .setMultisample(vkCtx_->getMsaaSamples());
        // MSAA alpha-to-coverage dithers the shader's sharpened cutout alpha
        // across samples for smooth foliage/leaf silhouettes.
        if (alphaToCoverage) builder.setAlphaToCoverage(true);
        return builder
            .setLayout(pipelineLayout_)
            .setRenderPass(mainPass)
            .setDynamicStates({VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR})
            .setFlags(flags)
            .setBasePipeline(basePipeline)
            .build(device, vkCtx_->getPipelineCache());
    };

    opaquePipeline_ = buildM2Pipeline(PipelineBuilder::blendDisabled(), true,
                                      VK_PIPELINE_CREATE_ALLOW_DERIVATIVES_BIT);
    alphaTestPipeline_ = buildM2Pipeline(PipelineBuilder::blendAlpha(), true,
                                         VK_PIPELINE_CREATE_DERIVATIVE_BIT, opaquePipeline_,
                                         /*alphaToCoverage=*/true);
    alphaPipeline_ = buildM2Pipeline(PipelineBuilder::blendAlpha(), false,
                                     VK_PIPELINE_CREATE_DERIVATIVE_BIT, opaquePipeline_);
    additivePipeline_ = buildM2Pipeline(PipelineBuilder::blendAdditive(), false,
                                        VK_PIPELINE_CREATE_DERIVATIVE_BIT, opaquePipeline_);

    // --- Build particle pipelines ---
    if (particleVert.isValid() && particleFrag.isValid()) {
        VkVertexInputBindingDescription pBind{};
        pBind.binding = 0;
        pBind.stride = 9 * sizeof(float); // pos3 + color4 + size1 + tile1
        pBind.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

        std::vector<VkVertexInputAttributeDescription> pAttrs = {
            {0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},                    // position
            {1, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 3 * sizeof(float)}, // color
            {2, 0, VK_FORMAT_R32_SFLOAT, 7 * sizeof(float)},          // size
            {3, 0, VK_FORMAT_R32_SFLOAT, 8 * sizeof(float)},          // tile
        };

        auto buildParticlePipeline = [&](VkPipelineColorBlendAttachmentState blend) -> VkPipeline {
            return PipelineBuilder()
                .setShaders(particleVert.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                            particleFrag.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
                .setVertexInput({pBind}, pAttrs)
                .setTopology(VK_PRIMITIVE_TOPOLOGY_POINT_LIST)
                .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
                .setDepthTest(true, false, VK_COMPARE_OP_LESS_OR_EQUAL)
                .setColorBlendAttachment(blend)
                .setMultisample(vkCtx_->getMsaaSamples())
                .setLayout(particlePipelineLayout_)
                .setRenderPass(mainPass)
                .setDynamicStates({VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR})
                .build(device, vkCtx_->getPipelineCache());
        };

        particlePipeline_ = buildParticlePipeline(PipelineBuilder::blendAlpha());
        particleAdditivePipeline_ = buildParticlePipeline(PipelineBuilder::blendAdditive());
    }

    // --- Build smoke pipeline ---
    if (smokeVert.isValid() && smokeFrag.isValid()) {
        VkVertexInputBindingDescription sBind{};
        sBind.binding = 0;
        sBind.stride = 6 * sizeof(float); // pos3 + lifeRatio1 + size1 + isSpark1
        sBind.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

        std::vector<VkVertexInputAttributeDescription> sAttrs = {
            {0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},           // position
            {1, 0, VK_FORMAT_R32_SFLOAT, 3 * sizeof(float)}, // lifeRatio
            {2, 0, VK_FORMAT_R32_SFLOAT, 4 * sizeof(float)}, // size
            {3, 0, VK_FORMAT_R32_SFLOAT, 5 * sizeof(float)}, // isSpark
        };

        smokePipeline_ = PipelineBuilder()
            .setShaders(smokeVert.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                        smokeFrag.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
            .setVertexInput({sBind}, sAttrs)
            .setTopology(VK_PRIMITIVE_TOPOLOGY_POINT_LIST)
            .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
            .setDepthTest(true, false, VK_COMPARE_OP_LESS_OR_EQUAL)
            .setColorBlendAttachment(PipelineBuilder::blendAlpha())
            .setMultisample(vkCtx_->getMsaaSamples())
            .setLayout(smokePipelineLayout_)
            .setRenderPass(mainPass)
            .setDynamicStates({VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR})
            .build(device, vkCtx_->getPipelineCache());
    }

    // --- Build ribbon pipelines ---
    // Vertex format: pos(3) + color(3) + alpha(1) + uv(2) = 9 floats = 36 bytes
    {
        rendering::VkShaderModule ribVert, ribFrag;
        (void)ribVert.loadFromFile(device, "assets/shaders/m2_ribbon.vert.spv");
        (void)ribFrag.loadFromFile(device, "assets/shaders/m2_ribbon.frag.spv");
        if (ribVert.isValid() && ribFrag.isValid()) {
            // Reuse particleTexLayout_ for set 1 (single texture sampler)
            VkDescriptorSetLayout ribLayouts[] = {perFrameLayout, particleTexLayout_};
            VkPipelineLayoutCreateInfo lci{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
            lci.setLayoutCount = 2;
            lci.pSetLayouts = ribLayouts;
            vkCreatePipelineLayout(device, &lci, nullptr, &ribbonPipelineLayout_);

            VkVertexInputBindingDescription rBind{};
            rBind.binding = 0;
            rBind.stride = 9 * sizeof(float);
            rBind.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

            std::vector<VkVertexInputAttributeDescription> rAttrs = {
                {0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},                    // pos
                {1, 0, VK_FORMAT_R32G32B32_SFLOAT, 3 * sizeof(float)},    // color
                {2, 0, VK_FORMAT_R32_SFLOAT,       6 * sizeof(float)},    // alpha
                {3, 0, VK_FORMAT_R32G32_SFLOAT,    7 * sizeof(float)},    // uv
            };

            auto buildRibbonPipeline = [&](VkPipelineColorBlendAttachmentState blend) -> VkPipeline {
                return PipelineBuilder()
                    .setShaders(ribVert.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                                ribFrag.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
                    .setVertexInput({rBind}, rAttrs)
                    .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP)
                    .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
                    .setDepthTest(true, false, VK_COMPARE_OP_LESS_OR_EQUAL)
                    .setColorBlendAttachment(blend)
                    .setMultisample(vkCtx_->getMsaaSamples())
                    .setLayout(ribbonPipelineLayout_)
                    .setRenderPass(mainPass)
                    .setDynamicStates({VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR})
                    .build(device, vkCtx_->getPipelineCache());
            };

            ribbonPipeline_         = buildRibbonPipeline(PipelineBuilder::blendAlpha());
            ribbonAdditivePipeline_ = buildRibbonPipeline(PipelineBuilder::blendAdditive());
        }
        ribVert.destroy(); ribFrag.destroy();
    }

    // Clean up shader modules
    m2Vert.destroy(); m2Frag.destroy();
    particleVert.destroy(); particleFrag.destroy();
    smokeVert.destroy(); smokeFrag.destroy();

    // --- Create dynamic particle buffers (mapped for CPU writes) ---
    {
        VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        bci.usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;

        VmaAllocationCreateInfo aci{};
        aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
        aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;

        VmaAllocationInfo allocInfo{};

        // Smoke particle buffer
        bci.size = MAX_SMOKE_PARTICLES * 6 * sizeof(float);
        vmaCreateBuffer(vkCtx_->getAllocator(), &bci, &aci, &smokeVB_, &smokeVBAlloc_, &allocInfo);
        smokeVBMapped_ = allocInfo.pMappedData;

        // M2 particle buffer
        bci.size = MAX_M2_PARTICLES * 9 * sizeof(float);
        vmaCreateBuffer(vkCtx_->getAllocator(), &bci, &aci, &m2ParticleVB_, &m2ParticleVBAlloc_, &allocInfo);
        m2ParticleVBMapped_ = allocInfo.pMappedData;

        // Dedicated glow sprite buffer (separate from particle VB to avoid data race)
        bci.size = MAX_GLOW_SPRITES * 9 * sizeof(float);
        vmaCreateBuffer(vkCtx_->getAllocator(), &bci, &aci, &glowVB_, &glowVBAlloc_, &allocInfo);
        glowVBMapped_ = allocInfo.pMappedData;

        // Ribbon vertex buffer — triangle strip: pos(3)+color(3)+alpha(1)+uv(2)=9 floats/vert
        bci.size = MAX_RIBBON_VERTS * 9 * sizeof(float);
        vmaCreateBuffer(vkCtx_->getAllocator(), &bci, &aci, &ribbonVB_, &ribbonVBAlloc_, &allocInfo);
        ribbonVBMapped_ = allocInfo.pMappedData;
    }

    // --- Create white fallback texture ---
    {
        uint8_t white[] = {255, 255, 255, 255};
        whiteTexture_ = std::make_unique<VkTexture>();
        whiteTexture_->upload(*vkCtx_, white, 1, 1, VK_FORMAT_R8G8B8A8_UNORM);
        whiteTexture_->createSampler(device, VK_FILTER_LINEAR, VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_REPEAT);
    }

    // --- Generate soft radial gradient glow texture ---
    {
        static constexpr int SZ = 64;
        std::vector<uint8_t> px(SZ * SZ * 4);
        float half = SZ / 2.0f;
        for (int y = 0; y < SZ; y++) {
            for (int x = 0; x < SZ; x++) {
                float dx = (x + 0.5f - half) / half;
                float dy = (y + 0.5f - half) / half;
                float r = std::sqrt(dx * dx + dy * dy);
                float a = std::max(0.0f, 1.0f - r);
                a = a * a; // Quadratic falloff
                int idx = (y * SZ + x) * 4;
                px[idx + 0] = 255;
                px[idx + 1] = 255;
                px[idx + 2] = 255;
                px[idx + 3] = static_cast<uint8_t>(a * 255);
            }
        }
        glowTexture_ = std::make_unique<VkTexture>();
        glowTexture_->upload(*vkCtx_, px.data(), SZ, SZ, VK_FORMAT_R8G8B8A8_UNORM);
        glowTexture_->createSampler(device, VK_FILTER_LINEAR, VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE);
        // Pre-allocate glow texture descriptor set (reused every frame)
        if (particleTexLayout_ && materialDescPool_) {
            VkDescriptorSetAllocateInfo ai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
            ai.descriptorPool = materialDescPool_;
            ai.descriptorSetCount = 1;
            ai.pSetLayouts = &particleTexLayout_;
            if (vkAllocateDescriptorSets(device, &ai, &glowTexDescSet_) == VK_SUCCESS) {
                VkDescriptorImageInfo imgInfo = glowTexture_->descriptorInfo();
                VkWriteDescriptorSet write{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
                write.dstSet = glowTexDescSet_;
                write.dstBinding = 0;
                write.descriptorCount = 1;
                write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                write.pImageInfo = &imgInfo;
                vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);
            }
        }
    }
    textureCacheBudgetBytes_ =
        envSizeMBOrDefault("WOWEE_M2_TEX_CACHE_MB", 4096) * 1024ull * 1024ull;
    modelCacheLimit_ = envSizeMBOrDefault("WOWEE_M2_MODEL_LIMIT", 6000);
    LOG_INFO("M2 texture cache budget: ", textureCacheBudgetBytes_ / (1024 * 1024), " MB");
    LOG_INFO("M2 model cache limit: ", modelCacheLimit_);

    LOG_INFO("M2 renderer initialized (Vulkan)");
    initialized_ = true;
    return true;
}

void M2Renderer::invalidateCullOutput(uint32_t frameIndex) {
    // On non-HOST_COHERENT memory, VMA-mapped GPU→CPU buffers need explicit
    // invalidation so the CPU cache sees the latest GPU writes.
    if (frameIndex < 2 && cullOutputAlloc_[frameIndex]) {
        vmaInvalidateAllocation(vkCtx_->getAllocator(), cullOutputAlloc_[frameIndex], 0, VK_WHOLE_SIZE);
    }
}

void M2Renderer::shutdown() {
    LOG_INFO("Shutting down M2 renderer...");
    if (!vkCtx_) return;

    vkDeviceWaitIdle(vkCtx_->getDevice());
    VkDevice device = vkCtx_->getDevice();
    VmaAllocator alloc = vkCtx_->getAllocator();

    // Delete model GPU resources
    for (auto& [id, model] : models) {
        destroyModelGPU(model);
    }
    models.clear();
    pinnedModelIds_.clear();

    // Destroy instance bone buffers
    for (auto& inst : instances) {
        destroyInstanceBones(inst);
    }
    instances.clear();
    spatialGrid.clear();
    instanceIndexById.clear();
    instanceDedupMap_.clear();
    // Model and bone destruction above is deferred; drain it now while the
    // descriptor pools are still alive.
    vkCtx_->flushDeferredCleanup();

    // Delete cached textures
    textureCache.clear();
    textureCacheBytes_ = 0;
    textureCacheCounter_ = 0;
    texturePropsByPtr_.clear();
    failedTextureCache_.clear();
    failedTextureRetryAt_.clear();
    loggedTextureLoadFails_.clear();
    textureLookupSerial_ = 0;
    textureBudgetRejectWarnings_ = 0;
    whiteTexture_.reset();
    glowTexture_.reset();

    // Clean up particle/ribbon buffers
    if (smokeVB_) { vmaDestroyBuffer(alloc, smokeVB_, smokeVBAlloc_); smokeVB_ = VK_NULL_HANDLE; }
    if (m2ParticleVB_) { vmaDestroyBuffer(alloc, m2ParticleVB_, m2ParticleVBAlloc_); m2ParticleVB_ = VK_NULL_HANDLE; }
    if (glowVB_) { vmaDestroyBuffer(alloc, glowVB_, glowVBAlloc_); glowVB_ = VK_NULL_HANDLE; }
    if (ribbonVB_) { vmaDestroyBuffer(alloc, ribbonVB_, ribbonVBAlloc_); ribbonVB_ = VK_NULL_HANDLE; }
    smokeParticles.clear();

    // Destroy pipelines
    auto destroyPipeline = [&](VkPipeline& p) { if (p) { vkDestroyPipeline(device, p, nullptr); p = VK_NULL_HANDLE; } };
    destroyPipeline(opaquePipeline_);
    destroyPipeline(alphaTestPipeline_);
    destroyPipeline(alphaPipeline_);
    destroyPipeline(additivePipeline_);
    destroyPipeline(particlePipeline_);
    destroyPipeline(particleAdditivePipeline_);
    destroyPipeline(smokePipeline_);
    destroyPipeline(ribbonPipeline_);
    destroyPipeline(ribbonAdditivePipeline_);

    if (pipelineLayout_) { vkDestroyPipelineLayout(device, pipelineLayout_, nullptr); pipelineLayout_ = VK_NULL_HANDLE; }
    if (particlePipelineLayout_) { vkDestroyPipelineLayout(device, particlePipelineLayout_, nullptr); particlePipelineLayout_ = VK_NULL_HANDLE; }
    if (smokePipelineLayout_) { vkDestroyPipelineLayout(device, smokePipelineLayout_, nullptr); smokePipelineLayout_ = VK_NULL_HANDLE; }
    if (ribbonPipelineLayout_) { vkDestroyPipelineLayout(device, ribbonPipelineLayout_, nullptr); ribbonPipelineLayout_ = VK_NULL_HANDLE; }

    // Destroy descriptor pools and layouts
    if (dummyBoneBuffer_) { vmaDestroyBuffer(alloc, dummyBoneBuffer_, dummyBoneAlloc_); dummyBoneBuffer_ = VK_NULL_HANDLE; }
    // dummyBoneSet_ is freed implicitly when boneDescPool_ is destroyed
    dummyBoneSet_ = VK_NULL_HANDLE;
    // Mega bone SSBO cleanup (sets freed implicitly with boneDescPool_)
    for (int i = 0; i < 2; i++) {
        if (megaBoneBuffer_[i]) { vmaDestroyBuffer(alloc, megaBoneBuffer_[i], megaBoneAlloc_[i]); megaBoneBuffer_[i] = VK_NULL_HANDLE; }
        megaBoneMapped_[i] = nullptr;
        megaBoneSet_[i] = VK_NULL_HANDLE;
    }
    if (materialDescPool_) { vkDestroyDescriptorPool(device, materialDescPool_, nullptr); materialDescPool_ = VK_NULL_HANDLE; }
    if (boneDescPool_) {
        if (boneDescPoolGeneration_) boneDescPoolGeneration_->fetch_add(1, std::memory_order_relaxed);
        vkDestroyDescriptorPool(device, boneDescPool_, nullptr);
        boneDescPool_ = VK_NULL_HANDLE;
    }
    // Instance data SSBO cleanup (sets freed with instanceDescPool_)
    for (int i = 0; i < 2; i++) {
        if (instanceBuffer_[i]) { vmaDestroyBuffer(alloc, instanceBuffer_[i], instanceAlloc_[i]); instanceBuffer_[i] = VK_NULL_HANDLE; }
        instanceMapped_[i] = nullptr;
        instanceSet_[i] = VK_NULL_HANDLE;
    }
    if (instanceDescPool_) { vkDestroyDescriptorPool(device, instanceDescPool_, nullptr); instanceDescPool_ = VK_NULL_HANDLE; }

    // GPU frustum culling compute pipeline + buffers cleanup
    if (cullHiZPipeline_) { vkDestroyPipeline(device, cullHiZPipeline_, nullptr); cullHiZPipeline_ = VK_NULL_HANDLE; }
    if (cullHiZPipelineLayout_) { vkDestroyPipelineLayout(device, cullHiZPipelineLayout_, nullptr); cullHiZPipelineLayout_ = VK_NULL_HANDLE; }
    if (cullPipeline_) { vkDestroyPipeline(device, cullPipeline_, nullptr); cullPipeline_ = VK_NULL_HANDLE; }
    if (cullPipelineLayout_) { vkDestroyPipelineLayout(device, cullPipelineLayout_, nullptr); cullPipelineLayout_ = VK_NULL_HANDLE; }
    for (int i = 0; i < 2; i++) {
        if (cullUniformBuffer_[i]) { vmaDestroyBuffer(alloc, cullUniformBuffer_[i], cullUniformAlloc_[i]); cullUniformBuffer_[i] = VK_NULL_HANDLE; }
        if (cullInputBuffer_[i])   { vmaDestroyBuffer(alloc, cullInputBuffer_[i], cullInputAlloc_[i]); cullInputBuffer_[i] = VK_NULL_HANDLE; }
        if (cullOutputBuffer_[i])  { vmaDestroyBuffer(alloc, cullOutputBuffer_[i], cullOutputAlloc_[i]); cullOutputBuffer_[i] = VK_NULL_HANDLE; }
        cullUniformMapped_[i] = cullInputMapped_[i] = cullOutputMapped_[i] = nullptr;
        cullSet_[i] = VK_NULL_HANDLE;
    }
    if (cullDescPool_) { vkDestroyDescriptorPool(device, cullDescPool_, nullptr); cullDescPool_ = VK_NULL_HANDLE; }
    if (cullSetLayout_) { vkDestroyDescriptorSetLayout(device, cullSetLayout_, nullptr); cullSetLayout_ = VK_NULL_HANDLE; }

    if (materialSetLayout_) { vkDestroyDescriptorSetLayout(device, materialSetLayout_, nullptr); materialSetLayout_ = VK_NULL_HANDLE; }
    if (boneSetLayout_) { vkDestroyDescriptorSetLayout(device, boneSetLayout_, nullptr); boneSetLayout_ = VK_NULL_HANDLE; }
    if (instanceSetLayout_) { vkDestroyDescriptorSetLayout(device, instanceSetLayout_, nullptr); instanceSetLayout_ = VK_NULL_HANDLE; }
    if (particleTexLayout_) { vkDestroyDescriptorSetLayout(device, particleTexLayout_, nullptr); particleTexLayout_ = VK_NULL_HANDLE; }

    // Destroy shadow resources
    destroyPipeline(shadowPipeline_);
    if (shadowPipelineLayout_) { vkDestroyPipelineLayout(device, shadowPipelineLayout_, nullptr); shadowPipelineLayout_ = VK_NULL_HANDLE; }
    for (auto& pool : shadowTexPool_) { if (pool) { vkDestroyDescriptorPool(device, pool, nullptr); pool = VK_NULL_HANDLE; } }
    if (shadowParamsPool_) { vkDestroyDescriptorPool(device, shadowParamsPool_, nullptr); shadowParamsPool_ = VK_NULL_HANDLE; }
    if (shadowParamsLayout_) { vkDestroyDescriptorSetLayout(device, shadowParamsLayout_, nullptr); shadowParamsLayout_ = VK_NULL_HANDLE; }
    if (shadowParamsUBO_) { vmaDestroyBuffer(alloc, shadowParamsUBO_, shadowParamsAlloc_); shadowParamsUBO_ = VK_NULL_HANDLE; }

    initialized_ = false;
}

void M2Renderer::destroyModelGPU(M2ModelGPU& model) {
    if (!vkCtx_) return;
    VmaAllocator alloc = vkCtx_->getAllocator();
    if (model.vertexBuffer) { vmaDestroyBuffer(alloc, model.vertexBuffer, model.vertexAlloc); model.vertexBuffer = VK_NULL_HANDLE; }
    if (model.indexBuffer) { vmaDestroyBuffer(alloc, model.indexBuffer, model.indexAlloc); model.indexBuffer = VK_NULL_HANDLE; }
    VkDevice device = vkCtx_->getDevice();
    for (auto& batch : model.batches) {
        if (batch.materialSet) { vkFreeDescriptorSets(device, materialDescPool_, 1, &batch.materialSet); batch.materialSet = VK_NULL_HANDLE; }
        if (batch.materialUBO) { vmaDestroyBuffer(alloc, batch.materialUBO, batch.materialUBOAlloc); batch.materialUBO = VK_NULL_HANDLE; }
    }
    // Free pre-allocated particle texture descriptor sets
    for (auto& pSet : model.particleTexSets) {
        if (pSet) { vkFreeDescriptorSets(device, materialDescPool_, 1, &pSet); pSet = VK_NULL_HANDLE; }
    }
    model.particleTexSets.clear();
    // Free ribbon texture descriptor sets
    for (auto& rSet : model.ribbonTexSets) {
        if (rSet) { vkFreeDescriptorSets(device, materialDescPool_, 1, &rSet); rSet = VK_NULL_HANDLE; }
    }
    model.ribbonTexSets.clear();
}

void M2Renderer::destroyInstanceBones(M2Instance& inst, bool defer) {
    if (!vkCtx_) return;
    VkDevice device = vkCtx_->getDevice();
    VmaAllocator alloc = vkCtx_->getAllocator();
    for (int i = 0; i < 2; i++) {
        // Snapshot handles before clearing the instance — needed for both
        // immediate and deferred paths.
        VkDescriptorSet boneSet = inst.boneSet[i];
        ::VkBuffer boneBuf = inst.boneBuffer[i];
        VmaAllocation boneAlloc = inst.boneAlloc[i];
        inst.boneSet[i] = VK_NULL_HANDLE;
        inst.boneBuffer[i] = VK_NULL_HANDLE;
        inst.boneMapped[i] = nullptr;

        if (!defer) {
            // Immediate destruction (safe after vkDeviceWaitIdle)
            if (boneSet != VK_NULL_HANDLE) {
                vkFreeDescriptorSets(device, boneDescPool_, 1, &boneSet);
            }
            if (boneBuf) {
                vmaDestroyBuffer(alloc, boneBuf, boneAlloc);
            }
        } else if (boneSet != VK_NULL_HANDLE || boneBuf) {
            // Deferred destruction — the loop destroys bone sets for ALL frame
            // slots, so the other slot's command buffer may still be in flight.
            // Must wait for all fences, not just the current frame's.
            VkDescriptorPool pool = boneDescPool_;
            auto poolGeneration = boneDescPoolGeneration_;
            uint64_t generation = poolGeneration ? poolGeneration->load(std::memory_order_relaxed) : 0;
            vkCtx_->deferAfterAllFrameFences([device, alloc, pool, poolGeneration, generation, boneSet, boneBuf, boneAlloc]() {
                const bool poolStillValid =
                    poolGeneration && poolGeneration->load(std::memory_order_relaxed) == generation;
                if (boneSet != VK_NULL_HANDLE && pool != VK_NULL_HANDLE && poolStillValid) {
                    VkDescriptorSet s = boneSet;
                    vkFreeDescriptorSets(device, pool, 1, &s);
                }
                if (boneBuf) {
                    vmaDestroyBuffer(alloc, boneBuf, boneAlloc);
                }
            });
        }
    }
}

VkDescriptorSet M2Renderer::allocateMaterialSet() {
    VkDescriptorSetAllocateInfo ai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
    ai.descriptorPool = materialDescPool_;
    ai.descriptorSetCount = 1;
    ai.pSetLayouts = &materialSetLayout_;
    VkDescriptorSet set = VK_NULL_HANDLE;
    VkResult result = vkAllocateDescriptorSets(vkCtx_->getDevice(), &ai, &set);
    if (result != VK_SUCCESS) {
        LOG_ERROR("M2Renderer: material descriptor set allocation failed (", result, ")");
        return VK_NULL_HANDLE;
    }
    return set;
}

VkDescriptorSet M2Renderer::allocateBoneSet() {
    VkDescriptorSetAllocateInfo ai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
    ai.descriptorPool = boneDescPool_;
    ai.descriptorSetCount = 1;
    ai.pSetLayouts = &boneSetLayout_;
    VkDescriptorSet set = VK_NULL_HANDLE;
    VkResult result = vkAllocateDescriptorSets(vkCtx_->getDevice(), &ai, &set);
    if (result != VK_SUCCESS) {
        LOG_ERROR("M2Renderer: bone descriptor set allocation failed (", result, ")");
        return VK_NULL_HANDLE;
    }
    return set;
}

// ---------------------------------------------------------------------------
// M2 collision mesh: build spatial grid + classify triangles
// ---------------------------------------------------------------------------
void M2ModelGPU::CollisionMesh::build() {
    if (indices.size() < 3 || vertices.empty()) return;
    triCount = static_cast<uint32_t>(indices.size() / 3);

    // Bounding box for grid
    glm::vec3 bmin(std::numeric_limits<float>::max());
    glm::vec3 bmax(-std::numeric_limits<float>::max());
    for (const auto& v : vertices) {
        bmin = glm::min(bmin, v);
        bmax = glm::max(bmax, v);
    }

    gridOrigin = glm::vec2(bmin.x, bmin.y);
    gridCellsX = std::max(1, std::min(32, static_cast<int>(std::ceil((bmax.x - bmin.x) / CELL_SIZE))));
    gridCellsY = std::max(1, std::min(32, static_cast<int>(std::ceil((bmax.y - bmin.y) / CELL_SIZE))));

    cellFloorTris.resize(static_cast<size_t>(gridCellsX) * static_cast<size_t>(gridCellsY));
    cellWallTris.resize(static_cast<size_t>(gridCellsX) * static_cast<size_t>(gridCellsY));
    triBounds.resize(triCount);

    for (uint32_t ti = 0; ti < triCount; ti++) {
        uint16_t i0 = indices[ti * 3];
        uint16_t i1 = indices[ti * 3 + 1];
        uint16_t i2 = indices[ti * 3 + 2];
        if (i0 >= vertices.size() || i1 >= vertices.size() || i2 >= vertices.size()) continue;

        const auto& v0 = vertices[i0];
        const auto& v1 = vertices[i1];
        const auto& v2 = vertices[i2];

        triBounds[ti].minZ = std::min({v0.z, v1.z, v2.z});
        triBounds[ti].maxZ = std::max({v0.z, v1.z, v2.z});

        glm::vec3 normal = glm::cross(v1 - v0, v2 - v0);
        float normalLen = glm::length(normal);
        float absNz = (normalLen > 0.001f) ? std::abs(normal.z / normalLen) : 0.0f;
        bool isFloor = (absNz >= 0.35f);  // ~70° max slope (relaxed for steep stairs)
        bool isWall  = (absNz < 0.65f);

        float triMinX = std::min({v0.x, v1.x, v2.x});
        float triMaxX = std::max({v0.x, v1.x, v2.x});
        float triMinY = std::min({v0.y, v1.y, v2.y});
        float triMaxY = std::max({v0.y, v1.y, v2.y});

        int cxMin = std::clamp(static_cast<int>((triMinX - gridOrigin.x) / CELL_SIZE), 0, gridCellsX - 1);
        int cxMax = std::clamp(static_cast<int>((triMaxX - gridOrigin.x) / CELL_SIZE), 0, gridCellsX - 1);
        int cyMin = std::clamp(static_cast<int>((triMinY - gridOrigin.y) / CELL_SIZE), 0, gridCellsY - 1);
        int cyMax = std::clamp(static_cast<int>((triMaxY - gridOrigin.y) / CELL_SIZE), 0, gridCellsY - 1);

        for (int cy = cyMin; cy <= cyMax; cy++) {
            for (int cx = cxMin; cx <= cxMax; cx++) {
                int ci = cy * gridCellsX + cx;
                if (isFloor) cellFloorTris[ci].push_back(ti);
                if (isWall)  cellWallTris[ci].push_back(ti);
            }
        }
    }
}

void M2ModelGPU::CollisionMesh::getFloorTrisInRange(
        float minX, float minY, float maxX, float maxY,
        std::vector<uint32_t>& out) const {
    out.clear();
    if (gridCellsX == 0 || gridCellsY == 0) return;
    int cxMin = std::clamp(static_cast<int>((minX - gridOrigin.x) / CELL_SIZE), 0, gridCellsX - 1);
    int cxMax = std::clamp(static_cast<int>((maxX - gridOrigin.x) / CELL_SIZE), 0, gridCellsX - 1);
    int cyMin = std::clamp(static_cast<int>((minY - gridOrigin.y) / CELL_SIZE), 0, gridCellsY - 1);
    int cyMax = std::clamp(static_cast<int>((maxY - gridOrigin.y) / CELL_SIZE), 0, gridCellsY - 1);
    const size_t cellCount = static_cast<size_t>(cxMax - cxMin + 1) *
                             static_cast<size_t>(cyMax - cyMin + 1);
    out.reserve(cellCount * 8);
    for (int cy = cyMin; cy <= cyMax; cy++) {
        for (int cx = cxMin; cx <= cxMax; cx++) {
            const auto& cell = cellFloorTris[cy * gridCellsX + cx];
            out.insert(out.end(), cell.begin(), cell.end());
        }
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
}

void M2ModelGPU::CollisionMesh::getWallTrisInRange(
        float minX, float minY, float maxX, float maxY,
        std::vector<uint32_t>& out) const {
    out.clear();
    if (gridCellsX == 0 || gridCellsY == 0) return;
    int cxMin = std::clamp(static_cast<int>((minX - gridOrigin.x) / CELL_SIZE), 0, gridCellsX - 1);
    int cxMax = std::clamp(static_cast<int>((maxX - gridOrigin.x) / CELL_SIZE), 0, gridCellsX - 1);
    int cyMin = std::clamp(static_cast<int>((minY - gridOrigin.y) / CELL_SIZE), 0, gridCellsY - 1);
    int cyMax = std::clamp(static_cast<int>((maxY - gridOrigin.y) / CELL_SIZE), 0, gridCellsY - 1);
    const size_t cellCount = static_cast<size_t>(cxMax - cxMin + 1) *
                             static_cast<size_t>(cyMax - cyMin + 1);
    out.reserve(cellCount * 8);
    for (int cy = cyMin; cy <= cyMax; cy++) {
        for (int cx = cxMin; cx <= cxMax; cx++) {
            const auto& cell = cellWallTris[cy * gridCellsX + cx];
            out.insert(out.end(), cell.begin(), cell.end());
        }
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
}

bool M2Renderer::hasModel(uint32_t modelId) const {
    return models.find(modelId) != models.end();
}

namespace {

// Which batches of a forge are the fire, as opposed to the stone and iron it is
// built from. classifyBatchTexture already recognises flame and glow cards; the
// forge adds coals, lava and the reflect textures Blizzard uses for hot metal,
// none of which carry a flame token in their name.
bool isForgeFireTexture(const std::string& texKeyLower,
                        const M2BatchTexClassification& tcls) {
    if (tcls.hasFlameToken || tcls.likelyFlame || tcls.hasGlowToken
        || tcls.hasGlowCardToken) {
        return true;
    }
    static constexpr std::string_view kEmberTokens[] = {
        "cinder", "coal", "ember", "lava", "magma", "reflect", "smoke",
    };
    for (auto tok : kEmberTokens) {
        if (texKeyLower.find(tok) != std::string::npos) return true;
    }
    return false;
}

} // namespace

void M2Renderer::markModelAsSpellEffect(uint32_t modelId) {
    auto it = models.find(modelId);
    if (it != models.end()) {
        it->second.isSpellEffect = true;
        // Spell effects MUST have bone animation for ribbons/particles to work.
        // The classifier may have set disableAnimation=true based on name tokens
        // (e.g. "chest" in HolySmite_Low_Chest.m2) — override that for spell effects.
        if (it->second.disableAnimation && it->second.hasAnimation) {
            it->second.disableAnimation = false;
            LOG_INFO("SpellEffect: re-enabled animation for '", it->second.name, "'");
        }
    }
}

bool M2Renderer::loadModel(const pipeline::M2Model& model, uint32_t modelId) {
    if (models.find(modelId) != models.end()) {
        // Already loaded
        return true;
    }
    if (models.size() >= modelCacheLimit_) {
        if (modelLimitRejectWarnings_ < 3) {
            LOG_WARNING("M2 model cache full (", models.size(), "/", modelCacheLimit_,
                        "), skipping model load: id=", modelId, " name=", model.name);
        }
        ++modelLimitRejectWarnings_;
        return false;
    }

    bool hasGeometry = !model.vertices.empty() && !model.indices.empty();
    bool hasParticles = !model.particleEmitters.empty();
    bool hasRibbons   = !model.ribbonEmitters.empty();
    if (!hasGeometry && !hasParticles && !hasRibbons) {
        LOG_WARNING("M2 model has no renderable content: id=", modelId,
                    " name=", model.name.empty() ? "<unnamed>" : model.name);
        return false;
    }

    M2ModelGPU gpuModel;
    gpuModel.name = model.name;

    // Use tight bounds from actual vertices for collision/camera occlusion.
    // Header bounds in some M2s are overly conservative.
    glm::vec3 tightMin(0.0f);
    glm::vec3 tightMax(0.0f);
    if (hasGeometry) {
        tightMin = glm::vec3(std::numeric_limits<float>::max());
        tightMax = glm::vec3(-std::numeric_limits<float>::max());
        for (const auto& v : model.vertices) {
            // Skip NaN-positioned vertices — would corrupt the bounds
            // (glm::min on NaN is implementation-defined) and feed NaN
            // into the camera-occlusion / culling AABB.
            if (!std::isfinite(v.position.x) || !std::isfinite(v.position.y) ||
                !std::isfinite(v.position.z)) continue;
            tightMin = glm::min(tightMin, v.position);
            tightMax = glm::max(tightMax, v.position);
        }
        // If all vertices were NaN (very unlikely after the loader scrub
        // but defense in depth), fall back to a unit box around origin.
        if (tightMin.x > tightMax.x) {
            tightMin = glm::vec3(-1.0f);
            tightMax = glm::vec3(1.0f);
        }
    }

    // Classify model from name and geometry — pure function, no GPU dependencies.
    auto cls = classifyM2Model(model.name, tightMin, tightMax,
                                model.vertices.size(),
                                model.particleEmitters.size());
    const bool isInvisibleTrap   = cls.isInvisibleTrap;
    const bool groundDetailModel = cls.isGroundDetail;
    if (isInvisibleTrap) {
        LOG_INFO("Loading InvisibleTrap model: ", model.name, " (will be invisible, no collision)");
    }

    gpuModel.isInvisibleTrap             = cls.isInvisibleTrap;
    gpuModel.collisionSteppedFountain    = cls.collisionSteppedFountain;
    gpuModel.collisionSteppedLowPlatform = cls.collisionSteppedLowPlatform;
    gpuModel.collisionBridge             = cls.collisionBridge;
    gpuModel.collisionPlanter            = cls.collisionPlanter;
    gpuModel.collisionStatue             = cls.collisionStatue;
    gpuModel.collisionTreeTrunk          = cls.collisionTreeTrunk;
    gpuModel.collisionNarrowVerticalProp = cls.collisionNarrowVerticalProp;
    gpuModel.collisionSmallSolidProp     = cls.collisionSmallSolidProp;
    gpuModel.collisionNoBlock            = cls.collisionNoBlock;
    gpuModel.isGroundDetail              = cls.isGroundDetail;
    gpuModel.isFoliageLike               = cls.isFoliageLike;
    gpuModel.disableAnimation            = cls.disableAnimation;
    gpuModel.shadowWindFoliage           = cls.shadowWindFoliage;
    gpuModel.isFireflyEffect             = cls.isFireflyEffect;
    gpuModel.isSmallFoliage              = cls.isSmallFoliage;
    gpuModel.isSmoke                     = cls.isSmoke;
    gpuModel.isSpellEffect               = cls.isSpellEffect;
    gpuModel.isLavaModel                 = cls.isLavaModel;
    gpuModel.isInstancePortal            = cls.isInstancePortal;
    gpuModel.isWaterVegetation           = cls.isWaterVegetation;
    gpuModel.isElvenLike                 = cls.isElvenLike;
    gpuModel.isLanternLike               = cls.isLanternLike;
    gpuModel.isKoboldFlame               = cls.isKoboldFlame;
    gpuModel.isWaterfall                 = cls.isWaterfall;
    gpuModel.isBrazierOrFire             = cls.isBrazierOrFire;
    gpuModel.isGroundFire                = cls.isGroundFire;
    gpuModel.isForge                     = cls.isForge;
    gpuModel.isTorch                     = cls.isTorch;
    // Data-driven flight-path detection: name tokens miss many flying doodads
    // (buzzards, swallows, bird swarms, ...). A small mesh whose bone animation
    // translates it tens of units is a flight-path doodad — it visibly freezes
    // mid-air whenever distance culling stops its bone updates, so give it the
    // same treatment as named sky birds.
    bool flightPathDoodad = cls.isSkyBird;
    if (!flightPathDoodad && !cls.disableAnimation) {
        glm::vec3 meshExtent = tightMax - tightMin;
        const bool smallMesh = meshExtent.x < 6.0f && meshExtent.y < 6.0f &&
                               meshExtent.z < 6.0f;
        if (smallMesh) {
            constexpr float kFlightPathRange = 15.0f;
            for (const auto& bone : model.bones) {
                for (const auto& seq : bone.translation.sequences) {
                    for (const auto& v : seq.vec3Values) {
                        if (std::abs(v.x) > kFlightPathRange ||
                            std::abs(v.y) > kFlightPathRange ||
                            std::abs(v.z) > kFlightPathRange) {
                            flightPathDoodad = true;
                            break;
                        }
                    }
                    if (flightPathDoodad) break;
                }
                if (flightPathDoodad) break;
            }
            if (flightPathDoodad) {
                LOG_DEBUG("Flight-path doodad detected (unnamed sky bird): ", model.name);
            }
        }
    }
    gpuModel.isSkyBird                   = flightPathDoodad;
    gpuModel.isLightBeam                 = cls.isLightBeam;
    gpuModel.isTransportDoodad           = cls.isTransportDoodad;
    gpuModel.ambientEmitterType          = cls.ambientEmitterType;
    gpuModel.boundMin = tightMin;
    gpuModel.boundMax = tightMax;
    gpuModel.boundRadius = model.boundRadius;
    // Fallback: compute bound radius from vertex extents when M2 header reports 0
    if (gpuModel.boundRadius < 0.01f && !model.vertices.empty()) {
        glm::vec3 extent = tightMax - tightMin;
        gpuModel.boundRadius = glm::length(extent) * 0.5f;
    }
    gpuModel.indexCount = static_cast<uint32_t>(model.indices.size());
    gpuModel.vertexCount = static_cast<uint32_t>(model.vertices.size());

    // Store bone/sequence data for animation
    gpuModel.bones = model.bones;
    gpuModel.sequences = model.sequences;
    gpuModel.globalSequenceDurations = model.globalSequenceDurations;
    gpuModel.hasAnimation = false;
    for (const auto& bone : model.bones) {
        if (bone.translation.hasData() || bone.rotation.hasData() || bone.scale.hasData()) {
            gpuModel.hasAnimation = true;
            break;
        }
    }

    // Build collision mesh + spatial grid from M2 bounding geometry
    gpuModel.collision.vertices = model.collisionVertices;
    gpuModel.collision.indices = model.collisionIndices;
    gpuModel.collision.build();
    if (gpuModel.collision.valid()) {
        core::Logger::getInstance().debug("  M2 collision mesh: ", gpuModel.collision.triCount,
            " tris, grid ", gpuModel.collision.gridCellsX, "x", gpuModel.collision.gridCellsY);
    }

    // Identify idle variation sequences (animation ID 0 = Stand)
    for (int i = 0; i < static_cast<int>(model.sequences.size()); i++) {
        if (model.sequences[i].id == 0 && model.sequences[i].duration > 0) {
            gpuModel.idleVariationIndices.push_back(i);
        }
    }

    // Batch all GPU uploads (VB, IB, textures) into a single command buffer
    // submission with one fence wait, instead of one fence wait per upload.
    vkCtx_->beginUploadBatch();

    if (hasGeometry) {
        // Create VBO with interleaved vertex data
        // Format: position (3), normal (3), texcoord0 (2), texcoord1 (2), boneWeights (4), boneIndices (4 as float)
        const size_t floatsPerVertex = 18;
        std::vector<float> vertexData;
        vertexData.reserve(model.vertices.size() * floatsPerVertex);

        for (const auto& v : model.vertices) {
            vertexData.push_back(v.position.x);
            vertexData.push_back(v.position.y);
            vertexData.push_back(v.position.z);
            vertexData.push_back(v.normal.x);
            vertexData.push_back(v.normal.y);
            vertexData.push_back(v.normal.z);
            vertexData.push_back(v.texCoords[0].x);
            vertexData.push_back(v.texCoords[0].y);
            vertexData.push_back(v.texCoords[1].x);
            vertexData.push_back(v.texCoords[1].y);
            float w0 = v.boneWeights[0] / 255.0f;
            float w1 = v.boneWeights[1] / 255.0f;
            float w2 = v.boneWeights[2] / 255.0f;
            float w3 = v.boneWeights[3] / 255.0f;
            vertexData.push_back(w0);
            vertexData.push_back(w1);
            vertexData.push_back(w2);
            vertexData.push_back(w3);
            vertexData.push_back(static_cast<float>(std::min(v.boneIndices[0], uint8_t(127))));
            vertexData.push_back(static_cast<float>(std::min(v.boneIndices[1], uint8_t(127))));
            vertexData.push_back(static_cast<float>(std::min(v.boneIndices[2], uint8_t(127))));
            vertexData.push_back(static_cast<float>(std::min(v.boneIndices[3], uint8_t(127))));
        }

        // Upload vertex buffer to GPU
        {
            auto buf = uploadBuffer(*vkCtx_,
                vertexData.data(), vertexData.size() * sizeof(float),
                VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
            gpuModel.vertexBuffer = buf.buffer;
            gpuModel.vertexAlloc = buf.allocation;
        }

        // Upload index buffer to GPU
        {
            auto buf = uploadBuffer(*vkCtx_,
                model.indices.data(), model.indices.size() * sizeof(uint16_t),
                VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
            gpuModel.indexBuffer = buf.buffer;
            gpuModel.indexAlloc = buf.allocation;
        }

        if (!gpuModel.vertexBuffer || !gpuModel.indexBuffer) {
            LOG_ERROR("M2Renderer::loadModel: GPU buffer upload failed for model ", modelId);
        }
    }

    // Load ALL textures from the model into a local vector.
    // textureLoadFailed[i] is true if texture[i] had a named path that failed to load.
    // Such batches are hidden (batchOpacity=0) rather than rendered white.
    std::vector<VkTexture*> allTextures;
    std::vector<bool> textureLoadFailed;
    std::vector<std::string> textureKeysLower;
    if (assetManager) {
        for (size_t ti = 0; ti < model.textures.size(); ti++) {
            const auto& tex = model.textures[ti];
            std::string texPath = tex.filename;
            // Some extracted M2 texture strings contain embedded NUL + garbage suffix.
            // Truncate at first NUL so valid paths like "...foo.blp\0junk" still resolve.
            size_t nul = texPath.find('\0');
            if (nul != std::string::npos) {
                texPath.resize(nul);
            }
            if (!texPath.empty()) {
                std::string texKey = texPath;
                std::replace(texKey.begin(), texKey.end(), '/', '\\');
                std::transform(texKey.begin(), texKey.end(), texKey.begin(),
                               [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                VkTexture* texPtr = loadTexture(texPath, tex.flags);
                bool failed = (texPtr == whiteTexture_.get());
                if (failed) {
                    static uint32_t loggedModelTextureFails = 0;
                    static bool loggedModelTextureFailSuppressed = false;
                    if (loggedModelTextureFails < 250) {
                        LOG_WARNING("M2 model ", model.name, " texture[", ti, "] failed to load: ", texPath);
                        ++loggedModelTextureFails;
                    } else if (!loggedModelTextureFailSuppressed) {
                        LOG_WARNING("M2 model texture-failure warnings suppressed after ",
                                    loggedModelTextureFails, " entries");
                        loggedModelTextureFailSuppressed = true;
                    }
                }
                if (isInvisibleTrap) {
                    LOG_INFO("  InvisibleTrap texture[", ti, "]: ", texPath, " -> ", (failed ? "WHITE" : "OK"));
                }
                allTextures.push_back(texPtr);
                textureLoadFailed.push_back(failed);
                textureKeysLower.push_back(std::move(texKey));
            } else {
                if (isInvisibleTrap) {
                    LOG_INFO("  InvisibleTrap texture[", ti, "]: EMPTY (using white fallback)");
                }
                allTextures.push_back(whiteTexture_.get());
                textureLoadFailed.push_back(false);  // Empty filename = intentional white (type!=0)
                textureKeysLower.emplace_back();
            }
        }
    }

    static const bool kGlowDiag = envFlagEnabled("WOWEE_M2_GLOW_DIAG", false);
    if (kGlowDiag) {
        if (gpuModel.isLanternLike) {
            for (size_t ti = 0; ti < model.textures.size(); ++ti) {
                const std::string key = (ti < textureKeysLower.size()) ? textureKeysLower[ti] : std::string();
                LOG_DEBUG("M2 GLOW TEX '", model.name, "' tex[", ti, "]='", key, "' flags=0x",
                          std::hex, model.textures[ti].flags, std::dec);
            }
        }
    }

    // Copy particle emitter data and resolve textures
    gpuModel.particleEmitters = model.particleEmitters;
    gpuModel.particleTextures.resize(model.particleEmitters.size(), whiteTexture_.get());
    for (size_t ei = 0; ei < model.particleEmitters.size(); ei++) {
        uint16_t texIdx = model.particleEmitters[ei].texture;
        if (texIdx < allTextures.size() && allTextures[texIdx] != nullptr) {
            gpuModel.particleTextures[ei] = allTextures[texIdx];
        } else {
            LOG_WARNING("M2 '", model.name, "' particle emitter[", ei,
                        "] texture index ", texIdx, " out of range (", allTextures.size(),
                        " textures) — using white fallback");
        }
    }

    // Pre-allocate one stable descriptor set per particle emitter to avoid per-frame allocation.
    // This prevents materialDescPool_ exhaustion when many emitters are active each frame.
    if (particleTexLayout_ && materialDescPool_ && !model.particleEmitters.empty()) {
        VkDevice device = vkCtx_->getDevice();
        gpuModel.particleTexSets.resize(model.particleEmitters.size(), VK_NULL_HANDLE);
        for (size_t ei = 0; ei < model.particleEmitters.size(); ei++) {
            VkDescriptorSetAllocateInfo ai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
            ai.descriptorPool = materialDescPool_;
            ai.descriptorSetCount = 1;
            ai.pSetLayouts = &particleTexLayout_;
            if (vkAllocateDescriptorSets(device, &ai, &gpuModel.particleTexSets[ei]) == VK_SUCCESS) {
                VkTexture* tex = gpuModel.particleTextures[ei];
                VkDescriptorImageInfo imgInfo = tex->descriptorInfo();
                VkWriteDescriptorSet write{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
                write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                write.dstSet = gpuModel.particleTexSets[ei];
                write.dstBinding = 0;
                write.descriptorCount = 1;
                write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                write.pImageInfo = &imgInfo;
                vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);
            }
        }
    }

    // Copy ribbon emitter data and resolve textures
    gpuModel.ribbonEmitters = model.ribbonEmitters;
    if (!model.ribbonEmitters.empty()) {
        VkDevice device = vkCtx_->getDevice();
        gpuModel.ribbonTextures.resize(model.ribbonEmitters.size(), whiteTexture_.get());
        gpuModel.ribbonTexSets.resize(model.ribbonEmitters.size(), VK_NULL_HANDLE);
        for (size_t ri = 0; ri < model.ribbonEmitters.size(); ri++) {
            // Resolve texture: ribbon textureIndex is a direct index into the
            // model's texture array (NOT through the textureLookup table).
            uint16_t texDirect = model.ribbonEmitters[ri].textureIndex;
            if (texDirect < allTextures.size() && allTextures[texDirect] != nullptr) {
                gpuModel.ribbonTextures[ri] = allTextures[texDirect];
            } else {
                // Fallback: try through textureLookup table
                uint32_t texIdx = (texDirect < model.textureLookup.size())
                                  ? model.textureLookup[texDirect] : UINT32_MAX;
                if (texIdx < allTextures.size() && allTextures[texIdx] != nullptr) {
                    gpuModel.ribbonTextures[ri] = allTextures[texIdx];
                } else {
                    LOG_WARNING("M2 '", model.name, "' ribbon emitter[", ri,
                                "] texIndex=", texDirect, " lookup failed"
                                " (direct=", (texDirect < allTextures.size() ? "yes" : "OOB"),
                                " lookup=", texIdx,
                                " textures=", allTextures.size(),
                                ") — using white fallback");
                }
            }
            // Allocate descriptor set (reuse particleTexLayout_ = single sampler)
            if (particleTexLayout_ && materialDescPool_) {
                VkDescriptorSetAllocateInfo ai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
                ai.descriptorPool = materialDescPool_;
                ai.descriptorSetCount = 1;
                ai.pSetLayouts = &particleTexLayout_;
                if (vkAllocateDescriptorSets(device, &ai, &gpuModel.ribbonTexSets[ri]) == VK_SUCCESS) {
                    VkTexture* tex = gpuModel.ribbonTextures[ri];
                    VkDescriptorImageInfo imgInfo = tex->descriptorInfo();
                    VkWriteDescriptorSet write{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
                    write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                    write.dstSet = gpuModel.ribbonTexSets[ri];
                    write.dstBinding = 0;
                    write.descriptorCount = 1;
                    write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                    write.pImageInfo = &imgInfo;
                    vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);
                }
            }
        }
        LOG_DEBUG("  Ribbon emitters loaded: ", model.ribbonEmitters.size());
    }

    // Copy texture transform data for UV animation
    gpuModel.textureTransforms = model.textureTransforms;
    gpuModel.textureTransformLookup = model.textureTransformLookup;
    gpuModel.hasTextureAnimation = false;

    // Build per-batch GPU entries
    if (!model.batches.empty()) {
        for (const auto& batch : model.batches) {
            M2ModelGPU::BatchGPU bgpu;
            bgpu.indexStart = batch.indexStart;
            bgpu.indexCount = batch.indexCount;

            // Store texture animation index from batch
            bgpu.textureAnimIndex = batch.textureAnimIndex;
            if (bgpu.textureAnimIndex != 0xFFFF) {
                gpuModel.hasTextureAnimation = true;
            }

            // Store blend mode and flags from material
            if (batch.materialIndex < model.materials.size()) {
                bgpu.blendMode = model.materials[batch.materialIndex].blendMode;
                bgpu.materialFlags = model.materials[batch.materialIndex].flags;
                if (bgpu.blendMode >= 2) gpuModel.hasTransparentBatches = true;
            }

            // Copy LOD level from batch
            bgpu.submeshLevel = batch.submeshLevel;

            // Resolve texture: batch.textureIndex → textureLookup → allTextures
            VkTexture* tex = whiteTexture_.get();
            bool texFailed = false;
            std::string batchTexKeyLower;
            if (batch.textureIndex < model.textureLookup.size()) {
                uint16_t texIdx = model.textureLookup[batch.textureIndex];
                if (texIdx < allTextures.size()) {
                    tex = allTextures[texIdx];
                    texFailed = (texIdx < textureLoadFailed.size()) && textureLoadFailed[texIdx];
                    if (texIdx < textureKeysLower.size()) {
                        batchTexKeyLower = textureKeysLower[texIdx];
                    }
                }
                if (texIdx < model.textures.size()) {
                    bgpu.texFlags = static_cast<uint8_t>(model.textures[texIdx].flags & 0x3);
                }
            } else if (!allTextures.empty()) {
                LOG_WARNING("M2 '", model.name, "' batch textureIndex ", batch.textureIndex,
                            " out of range (textureLookup size=", model.textureLookup.size(),
                            ") — falling back to texture[0]");
                tex = allTextures[0];
                texFailed = !textureLoadFailed.empty() && textureLoadFailed[0];
                if (!textureKeysLower.empty()) {
                    batchTexKeyLower = textureKeysLower[0];
                }
            }

            if (texFailed && groundDetailModel) {
                static const std::string kDetailFallbackTexture = "World\\NoDXT\\Detail\\8des_detaildoodads01.blp";
                VkTexture* fallbackTex = loadTexture(kDetailFallbackTexture, 0);
                if (fallbackTex != nullptr && fallbackTex != whiteTexture_.get()) {
                    tex = fallbackTex;
                    texFailed = false;
                }
            }
            bgpu.texture = tex;
            const auto tcls = classifyBatchTexture(batchTexKeyLower);
            const bool modelLanternFamily = gpuModel.isLanternLike;
            const bool torchGlowCard = gpuModel.isTorch &&
                tcls.hasGlowToken && tcls.hasGlowCardToken;
            // Fire pits and braziers commonly pair an authored flame mesh
            // (for example FLAMELICKSMALL) with a separate flat GLOW32 card.
            // Replace only that glow-textured card; retaining the flame mesh
            // preserves the intended animated fire shape.
            const bool fireGlowCard = gpuModel.isBrazierOrFire &&
                tcls.hasGlowToken && tcls.hasGlowCardToken;
            bgpu.lanternGlowHint =
                tcls.softGlowSurface ||
                tcls.exactLanternGlowTex ||
                torchGlowCard ||
                fireGlowCard ||
                ((tcls.hasGlowToken || (modelLanternFamily && tcls.hasFlameToken)) &&
                 (tcls.lanternFamily || modelLanternFamily) &&
                 (!tcls.likelyFlame || modelLanternFamily));
            bgpu.glowCardLike = bgpu.lanternGlowHint &&
                (tcls.hasGlowCardToken || tcls.softGlowSurface);
            // A flame texture is the fire itself, not a flat card standing in for
            // one, so swapping it for a featureless sprite deletes the visible
            // flame — chandeliers and candelabra lit their surroundings while
            // their candles sat unlit. Braziers already keep their flame mesh for
            // this reason (see fireGlowCard above); FLAMELICK counts as a
            // glow-card token, so lantern-family models were not getting the same
            // treatment. Keep the mesh and let the sprite glow behind it.
            const bool flameCard = tcls.hasFlameToken || tcls.likelyFlame;
            bgpu.preserveGlowMesh = tcls.softGlowSurface ||
                                    (bgpu.lanternGlowHint && flameCard);
            bgpu.glowTint = tcls.glowTint;
            if (tex != nullptr && tex != whiteTexture_.get()) {
                auto pit = texturePropsByPtr_.find(tex);
                if (pit != texturePropsByPtr_.end()) {
                    bgpu.hasAlpha = pit->second.hasAlpha;
                    bgpu.colorKeyBlack = pit->second.colorKeyBlack;
                    // Forge fire is drawn on cards with a black backing that has
                    // to be keyed out, and some of them use effect textures
                    // carrying none of the flame/glow tokens the hint looks for
                    // — coals, lava lumps, ARMORREFLECT/ORBREFLECT. Keying the
                    // whole model instead made the masonry and ironwork
                    // translucent, since a forge is mostly those.
                    if (gpuModel.isForge && isForgeFireTexture(batchTexKeyLower, tcls)) {
                        bgpu.colorKeyBlack = true;
                        bgpu.forgeFireCard = true;
                    }
                }
            }
            // textureCoordIndex is an index into a texture coord combo table, not directly
            // a UV set selector. Most batches have index=0 (UV set 0). We always use UV set 0
            // since we don't have the full combo table — dual-UV effects are rare edge cases.
            bgpu.textureUnit = 0;

            // Start at full opacity; hide only if texture failed to load.
            bgpu.batchOpacity = (texFailed && !groundDetailModel) ? 0.0f : 1.0f;

            // Apply at-rest transparency and color alpha from the M2 animation tracks.
            // These provide per-batch opacity for ghosts, ethereal effects, fading doodads, etc.
            // Skip zero values: some animated tracks start at 0 and animate up, and baking
            // that first keyframe would make the entire batch permanently invisible.
            if (bgpu.batchOpacity > 0.0f) {
                float animAlpha = 1.0f;
                if (batch.colorIndex < model.colorAlphas.size()) {
                    float ca = model.colorAlphas[batch.colorIndex];
                    if (ca > 0.001f) animAlpha *= ca;
                }
                if (batch.transparencyIndex < model.textureWeights.size()) {
                    float tw = model.textureWeights[batch.transparencyIndex];
                    if (tw > 0.001f) animAlpha *= tw;
                }
                bgpu.batchOpacity *= animAlpha;
            }

            // Compute batch center and radius for glow sprite positioning
            if ((bgpu.blendMode >= 3 || bgpu.colorKeyBlack || bgpu.glowCardLike) && batch.indexCount > 0) {
                glm::vec3 sum(0.0f);
                uint32_t counted = 0;
                std::unordered_map<uint16_t, glm::vec4> boneAnchorSums;
                for (uint32_t j = batch.indexStart; j < batch.indexStart + batch.indexCount; j++) {
                    if (j < model.indices.size()) {
                        uint16_t vi = model.indices[j];
                        if (vi < model.vertices.size()) {
                            const auto& vertex = model.vertices[vi];
                            sum += vertex.position;
                            for (size_t influence = 0; influence < 4; ++influence) {
                                const float weight = static_cast<float>(vertex.boneWeights[influence]) / 255.0f;
                                if (weight <= 0.0f) continue;
                                const uint16_t bone = vertex.boneIndices[influence];
                                boneAnchorSums[bone] += glm::vec4(vertex.position * weight, weight);
                            }
                            counted++;
                        }
                    }
                }
                if (counted > 0) {
                    bgpu.center = sum / static_cast<float>(counted);
                    bgpu.lightBoneAnchors.reserve(boneAnchorSums.size());
                    for (const auto& [bone, weightedPoint] : boneAnchorSums) {
                        bgpu.lightBoneAnchors.push_back({
                            bone, weightedPoint / static_cast<float>(counted)});
                    }
                    if (!boneAnchorSums.empty() && !model.bones.empty()) {
                        const auto dominant = std::max_element(
                            boneAnchorSums.begin(), boneAnchorSums.end(),
                            [](const auto& a, const auto& b) {
                                return a.second.w < b.second.w;
                            });
                        size_t suspensionBone = dominant->first;
                        while (suspensionBone < model.bones.size()) {
                            const int16_t parent = model.bones[suspensionBone].parentBone;
                            if (parent < 0 || static_cast<size_t>(parent) >= model.bones.size()) break;
                            const glm::vec3 span = model.bones[parent].pivot -
                                                   model.bones[suspensionBone].pivot;
                            // A hanging chain climbs through parents above the
                            // bulb. Stop before a generic model/root bone at the
                            // placement origin poisons the projection direction.
                            if (span.z <= 0.01f || glm::length(span) > 5.0f) break;
                            suspensionBone = static_cast<size_t>(parent);
                        }
                        if (suspensionBone < model.bones.size() &&
                            suspensionBone != dominant->first) {
                            bgpu.lightSuspensionBone = static_cast<uint16_t>(suspensionBone);
                            bgpu.lightSuspensionPoint = model.bones[suspensionBone].pivot;
                        }
                    }
                    float maxDist = 0.0f;
                    for (uint32_t j = batch.indexStart; j < batch.indexStart + batch.indexCount; j++) {
                        if (j < model.indices.size()) {
                            uint16_t vi = model.indices[j];
                            if (vi < model.vertices.size()) {
                                float d = glm::length(model.vertices[vi].position - bgpu.center);
                                maxDist = std::max(maxDist, d);
                            }
                        }
                    }
                    bgpu.glowSize = std::max(maxDist, 0.5f);
                    // Upright fire glow cards are deliberately tall, so their
                    // geometric center floats above a small ground fire. Keep
                    // the radial halo near the fuel/flame base instead.
                    if (fireGlowCard && gpuModel.isGroundFire) {
                        // Ground-fire sprites follow a particle emitter and its
                        // animated bone at render time. Origin is the fallback
                        // for unusual fire models without an emitter.
                        bgpu.center = glm::vec3(0.0f);
                    }
                }
            }

            // Optional diagnostics for glow/light batches (disabled by default).
            if (kGlowDiag && gpuModel.isLanternLike) {
                LOG_DEBUG("M2 GLOW DIAG '", model.name, "' batch ", gpuModel.batches.size(),
                          ": blend=", bgpu.blendMode, " matFlags=0x",
                          std::hex, bgpu.materialFlags, std::dec,
                          " colorKey=", bgpu.colorKeyBlack ? "Y" : "N",
                          " hasAlpha=", bgpu.hasAlpha ? "Y" : "N",
                          " unlit=", (bgpu.materialFlags & 0x01) ? "Y" : "N",
                          " lanternHint=", bgpu.lanternGlowHint ? "Y" : "N",
                          " glowSize=", bgpu.glowSize,
                          " tex=", bgpu.texture,
                          " idxCount=", bgpu.indexCount);
            }
            gpuModel.batches.push_back(bgpu);
        }
    } else {
        // Fallback: single batch covering all indices with first texture
        M2ModelGPU::BatchGPU bgpu;
        bgpu.indexStart = 0;
        bgpu.indexCount = gpuModel.indexCount;
        bgpu.texture = allTextures.empty() ? whiteTexture_.get() : allTextures[0];
        if (bgpu.texture != nullptr && bgpu.texture != whiteTexture_.get()) {
            auto pit = texturePropsByPtr_.find(bgpu.texture);
            if (pit != texturePropsByPtr_.end()) {
                bgpu.hasAlpha = pit->second.hasAlpha;
                bgpu.colorKeyBlack = pit->second.colorKeyBlack;
            }
        }
        gpuModel.batches.push_back(bgpu);
    }

    // Detect particle emitter volume models: box mesh (24 verts, 36 indices)
    // with disproportionately large bounds. These are invisible bounding volumes
    // that only exist to spawn particles — their mesh should never be rendered.
    if (!isInvisibleTrap && !groundDetailModel &&
        gpuModel.vertexCount <= 24 && gpuModel.indexCount <= 36
        && !model.particleEmitters.empty()) {
        glm::vec3 size = gpuModel.boundMax - gpuModel.boundMin;
        float maxDim = std::max({size.x, size.y, size.z});
        if (maxDim > 5.0f) {
            gpuModel.isInvisibleTrap = true;
            LOG_DEBUG("M2 emitter volume hidden: '", model.name, "' size=(",
                      size.x, " x ", size.y, " x ", size.z, ")");
        }
    }

    vkCtx_->endUploadBatch();

    // Allocate Vulkan descriptor sets and UBOs for each batch
    for (auto& bgpu : gpuModel.batches) {
        // Create combined UBO for M2Params (binding 1) + M2Material (binding 2)
        // We allocate them as separate buffers for clarity
        VmaAllocationInfo matAllocInfo{};
        {
            VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
            bci.size = sizeof(M2MaterialUBO);
            bci.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
            VmaAllocationCreateInfo aci{};
            aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
            aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
            vmaCreateBuffer(vkCtx_->getAllocator(), &bci, &aci, &bgpu.materialUBO, &bgpu.materialUBOAlloc, &matAllocInfo);

            // Write initial material data (static per-batch — fadeAlpha/interiorDarken updated at draw time)
            M2MaterialUBO mat{};
            mat.hasTexture = (bgpu.texture != nullptr && bgpu.texture != whiteTexture_.get()) ? 1 : 0;
            mat.alphaTest = (bgpu.blendMode == 1 || (bgpu.blendMode >= 2 && !bgpu.hasAlpha)) ? 1 : 0;
            mat.colorKeyBlack = bgpu.colorKeyBlack ? 1 : 0;
            mat.colorKeyThreshold = 0.08f;
            mat.unlit = (bgpu.materialFlags & 0x01) ? 1 : 0;
            mat.blendMode = bgpu.blendMode;
            mat.fadeAlpha = 1.0f;
            mat.interiorDarken = 0.0f;
            mat.specularIntensity = 0.5f;
            mat.emissiveBoost = bgpu.preserveGlowMesh ? 2.4f : 1.0f;
            memcpy(matAllocInfo.pMappedData, &mat, sizeof(mat));
            bgpu.materialUBOMapped = matAllocInfo.pMappedData;
        }

        // Allocate descriptor set and write all bindings
        bgpu.materialSet = allocateMaterialSet();
        if (bgpu.materialSet) {
            VkTexture* batchTex = bgpu.texture ? bgpu.texture : whiteTexture_.get();
            VkDescriptorImageInfo imgInfo = batchTex->descriptorInfo();

            VkDescriptorBufferInfo matBufInfo{};
            matBufInfo.buffer = bgpu.materialUBO;
            matBufInfo.offset = 0;
            matBufInfo.range = sizeof(M2MaterialUBO);

            VkWriteDescriptorSet writes[2] = {};
            // binding 0: texture
            writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[0].dstSet = bgpu.materialSet;
            writes[0].dstBinding = 0;
            writes[0].descriptorCount = 1;
            writes[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            writes[0].pImageInfo = &imgInfo;
            // binding 2: M2Material UBO
            writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[1].dstSet = bgpu.materialSet;
            writes[1].dstBinding = 2;
            writes[1].descriptorCount = 1;
            writes[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
            writes[1].pBufferInfo = &matBufInfo;

            vkUpdateDescriptorSets(vkCtx_->getDevice(), 2, writes, 0, nullptr);
        }
    }

    // Pre-compute available LOD levels to avoid per-instance batch iteration
    gpuModel.availableLODs = 0;
    for (const auto& b : gpuModel.batches) {
        if (b.submeshLevel < 8) gpuModel.availableLODs |= (1u << b.submeshLevel);
    }

    models[modelId] = std::move(gpuModel);
    spatialIndexDirty_ = true;  // Map may have rehashed — refresh cachedModel pointers

    LOG_DEBUG("Loaded M2 model: ", model.name, " (", models[modelId].vertexCount, " vertices, ",
              models[modelId].indexCount / 3, " triangles, ", models[modelId].batches.size(), " batches)");


    return true;
}

} // namespace rendering
} // namespace wowee
