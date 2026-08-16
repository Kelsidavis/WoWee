// PostProcessPipeline - FSR 1.0, FXAA, FSR 2.2/3 state and passes (§4.3)
// Extracted from Renderer to isolate post-processing concerns.

#include "rendering/post_process_pipeline.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/vk_shader.hpp"
#include "rendering/vk_pipeline.hpp"
#include "rendering/camera.hpp"
#include "rendering/amd_fsr3_runtime.hpp"
#include "core/logger.hpp"
#include "core/profiler.hpp"
#include <cstdlib>
#include <algorithm>
#include <glm/gtc/matrix_inverse.hpp>

namespace wowee {
namespace rendering {

PostProcessPipeline::PostProcessPipeline() = default;
PostProcessPipeline::~PostProcessPipeline() { shutdown(); }

void PostProcessPipeline::initialize(VkContext* ctx) {
    vkCtx_ = ctx;
}

void PostProcessPipeline::shutdown() {
    destroyFSRResources();
    destroyFSR2Resources();
    destroyFXAAResources();
    vkCtx_ = nullptr;
}

// ========================= Frame-loop integration =========================

void PostProcessPipeline::manageResources() {
    // FSR resource management (safe: between frames, no command buffer in flight)
    if (fsr_.needsRecreate && fsr_.sceneFramebuffer) {
        destroyFSRResources();
        fsr_.needsRecreate = false;
        if (!fsr_.enabled) LOG_INFO("FSR: disabled");
    }
    if (fsr_.enabled && !fsr2_.enabled && !fsr_.sceneFramebuffer) {
        if (!initFSRResources()) {
            LOG_ERROR("FSR: initialization failed, disabling");
            fsr_.enabled = false;
        }
    }

    // FSR 2.2 resource management
    if (fsr2_.needsRecreate && fsr2_.sceneFramebuffer) {
        destroyFSR2Resources();
        fsr2_.needsRecreate = false;
        if (!fsr2_.enabled) LOG_INFO("FSR2: disabled");
    }
    if (fsr2_.enabled && !fsr2_.sceneFramebuffer) {
        if (!initFSR2Resources()) {
            LOG_ERROR("FSR2: initialization failed, disabling");
            fsr2_.enabled = false;
        }
    }

    // FXAA resource management - FXAA can coexist with FSR1 and FSR3.
    // When both FXAA and FSR3 are enabled, FXAA runs as a post-FSR3 pass.
    // Do not force this pass for ghost mode; keep AA quality strictly user-controlled.
    const bool useFXAA = needsFXAAPass();
    if ((fxaa_.needsRecreate || !useFXAA) && fxaa_.sceneFramebuffer) {
        destroyFXAAResources();
        fxaa_.needsRecreate = false;
        if (!useFXAA) LOG_INFO("FXAA: disabled");
    }
    if (useFXAA && !fxaa_.sceneFramebuffer) {
        if (!initFXAAResources()) {
            LOG_ERROR("FXAA: initialization failed, disabling");
            if (fxaa_.enabled) fxaa_.enabled = false;
            intoxication_ = 0.0f;
        }
    }
}

void PostProcessPipeline::handleSwapchainResize() {
    const bool useFXAA = needsFXAAPass();
    // Recreate FSR resources for new swapchain dimensions
    if (fsr_.enabled && !fsr2_.enabled) {
        destroyFSRResources();
        initFSRResources();
    }
    if (fsr2_.enabled) {
        destroyFSR2Resources();
        initFSR2Resources();
    }
    // Recreate FXAA resources for new swapchain dimensions.
    if (useFXAA) {
        destroyFXAAResources();
        initFXAAResources();
    }
}

void PostProcessPipeline::applyJitter(Camera* camera) {
    if (!fsr2_.enabled || !fsr2_.sceneFramebuffer || !camera) return;

    if (!fsr2_.useAmdBackend) {
        camera->setJitter(0.0f, 0.0f);
    } else {
#if WOWEE_HAS_AMD_FSR2
        // AMD-recommended jitter sequence in pixel space, converted to NDC projection offset.
        int32_t phaseCount = ffxFsr2GetJitterPhaseCount(
            static_cast<int32_t>(fsr2_.internalWidth),
            static_cast<int32_t>(vkCtx_->getSwapchainExtent().width));
        float jitterX = 0.0f;
        float jitterY = 0.0f;
        if (phaseCount > 0 &&
            ffxFsr2GetJitterOffset(&jitterX, &jitterY, static_cast<int32_t>(fsr2_.frameIndex % static_cast<uint32_t>(phaseCount)), phaseCount) == FFX_OK) {
            float ndcJx = (2.0f * jitterX) / static_cast<float>(fsr2_.internalWidth);
            float ndcJy = (2.0f * jitterY) / static_cast<float>(fsr2_.internalHeight);
            // Keep projection jitter and FSR dispatch jitter in sync.
            camera->setJitter(fsr2_.jitterSign * ndcJx, fsr2_.jitterSign * ndcJy);
        } else {
            camera->setJitter(0.0f, 0.0f);
        }
#else
        const float jitterScale = 0.5f;
        float jx = (halton(fsr2_.frameIndex + 1, 2) - 0.5f) * 2.0f * jitterScale / static_cast<float>(fsr2_.internalWidth);
        float jy = (halton(fsr2_.frameIndex + 1, 3) - 0.5f) * 2.0f * jitterScale / static_cast<float>(fsr2_.internalHeight);
        camera->setJitter(fsr2_.jitterSign * jx, fsr2_.jitterSign * jy);
#endif
    }
}

VkFramebuffer PostProcessPipeline::getSceneFramebuffer() const {
    if (fsr2_.enabled && fsr2_.sceneFramebuffer)
        return fsr2_.sceneFramebuffer;
    if (needsFXAAPass() && fxaa_.sceneFramebuffer)
        return fxaa_.sceneFramebuffer;
    if (fsr_.enabled && fsr_.sceneFramebuffer)
        return fsr_.sceneFramebuffer;
    return VK_NULL_HANDLE;
}

VkExtent2D PostProcessPipeline::getSceneRenderExtent() const {
    if (fsr2_.enabled && fsr2_.sceneFramebuffer)
        return { .width = fsr2_.internalWidth, .height = fsr2_.internalHeight };
    if (needsFXAAPass() && fxaa_.sceneFramebuffer)
        return vkCtx_->getSwapchainExtent();  // native resolution - no downscaling
    if (fsr_.enabled && fsr_.sceneFramebuffer)
        return { .width = fsr_.internalWidth, .height = fsr_.internalHeight };
    return vkCtx_->getSwapchainExtent();
}
VkImage PostProcessPipeline::getSceneColorImage() const {
    if (fsr2_.enabled && fsr2_.sceneFramebuffer) return fsr2_.sceneColor.image;
    if (needsFXAAPass() && fxaa_.sceneFramebuffer) return fxaa_.sceneColor.image;
    if (fsr_.enabled && fsr_.sceneFramebuffer) return fsr_.sceneColor.image;
    return VK_NULL_HANDLE;
}

VkImage PostProcessPipeline::getSceneDepthImage() const {
    // Prefer the resolved depth where MSAA produced one: it is single-sampled and
    // can be copied directly.
    if (fsr2_.enabled && fsr2_.sceneFramebuffer) return fsr2_.sceneDepth.image;
    if (needsFXAAPass() && fxaa_.sceneFramebuffer)
        return fxaa_.sceneDepthResolve.image ? fxaa_.sceneDepthResolve.image : fxaa_.sceneDepth.image;
    if (fsr_.enabled && fsr_.sceneFramebuffer)
        return fsr_.sceneDepthResolve.image ? fsr_.sceneDepthResolve.image : fsr_.sceneDepth.image;
    return VK_NULL_HANDLE;
}

bool PostProcessPipeline::sceneDepthIsMsaa() const {
    if (!vkCtx_ || vkCtx_->getMsaaSamples() == VK_SAMPLE_COUNT_1_BIT) return false;
    if (fsr2_.enabled && fsr2_.sceneFramebuffer) return true;  // FSR2 keeps only the sampled depth
    if (needsFXAAPass() && fxaa_.sceneFramebuffer) return fxaa_.sceneDepthResolve.image == VK_NULL_HANDLE;
    if (fsr_.enabled && fsr_.sceneFramebuffer) return fsr_.sceneDepthResolve.image == VK_NULL_HANDLE;
    return true;
}
bool PostProcessPipeline::executePostProcessing(VkCommandBuffer cmd, uint32_t imageIndex,
                                                  Camera* camera, float deltaTime) {
    ZoneScopedN("PostProcess::execute");
    currentCmd_ = cmd;
    camera_ = camera;
    lastDeltaTime_ = deltaTime;
    bool inlineMode = false;

    if (fsr2_.enabled && fsr2_.sceneFramebuffer) {
        // End the off-screen scene render pass
        vkCmdEndRenderPass(currentCmd_);

        if (fsr2_.useAmdBackend) {
            // Compute passes: motion vectors -> temporal accumulation
            dispatchMotionVectors();
            if (fsr2_.amdFsr3FramegenEnabled && fsr2_.amdFsr3FramegenRuntimeReady) {
                dispatchAmdFsr3Framegen();
                if (!fsr2_.amdFsr3FramegenRuntimeActive) {
                    dispatchAmdFsr2();
                }
            } else {
                dispatchAmdFsr2();
            }

            // Transition post-FSR input for sharpen pass.
            if (fsr2_.amdFsr3FramegenRuntimeActive && fsr2_.framegenOutput.image) {
                transitionImageLayout(currentCmd_, fsr2_.framegenOutput.image,
                    VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                    VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                    VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);
                fsr2_.framegenOutputValid = true;
            } else {
                transitionImageLayout(currentCmd_, fsr2_.history[fsr2_.currentHistory].image,
                    VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                    VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                    VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);
            }
        } else {
            transitionImageLayout(currentCmd_, fsr2_.sceneColor.image,
                VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
                VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);
        }

        // FSR3+FXAA combined: re-point FXAA's descriptor to the FSR3 temporal output
        // so renderFXAAPass() applies spatial AA on the temporally-stabilized frame.
        // This must happen outside the render pass (descriptor updates are CPU-side).
        // Use per-frame descriptor set to avoid race with in-flight command buffers.
        uint32_t fxaaFrameIdx = vkCtx_->getCurrentFrame();
        if (needsFXAAPass() && fxaa_.descSet[fxaaFrameIdx] && fxaa_.sceneSampler) {
            VkImageView fsr3OutputView = VK_NULL_HANDLE;
            if (fsr2_.useAmdBackend) {
                if (fsr2_.amdFsr3FramegenRuntimeActive && fsr2_.framegenOutput.image)
                    fsr3OutputView = fsr2_.framegenOutput.imageView;
                else if (fsr2_.history[fsr2_.currentHistory].image)
                    fsr3OutputView = fsr2_.history[fsr2_.currentHistory].imageView;
            } else if (fsr2_.history[fsr2_.currentHistory].image) {
                fsr3OutputView = fsr2_.history[fsr2_.currentHistory].imageView;
            }
            if (fsr3OutputView) {
                VkDescriptorImageInfo imgInfo{};
                imgInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                imgInfo.imageView   = fsr3OutputView;
                imgInfo.sampler     = fxaa_.sceneSampler;
                VkWriteDescriptorSet write{};
                write.sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                write.dstSet           = fxaa_.descSet[fxaaFrameIdx];
                write.dstBinding       = 0;
                write.descriptorCount  = 1;
                write.descriptorType   = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                write.pImageInfo       = &imgInfo;
                vkUpdateDescriptorSets(vkCtx_->getDevice(), 1, &write, 0, nullptr);
            }
        }

        // Begin swapchain render pass at full resolution for sharpening + ImGui
        VkRenderPassBeginInfo rpInfo{};
        rpInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
        // Output goes through the single-sampled overlay pass. The scene pass
        // carries the scene's sample count, and these quads are 1x.
        rpInfo.renderPass = vkCtx_->getOverlayClearRenderPass();
        rpInfo.framebuffer = vkCtx_->getOverlayFramebuffers()[imageIndex];
        rpInfo.renderArea.offset = {.x = 0, .y = 0};
        rpInfo.renderArea.extent = vkCtx_->getSwapchainExtent();

        // The overlay pass has one attachment whatever the scene's MSAA is.
        VkClearValue clearValues[1]{};
        clearValues[0].color = {{0.0f, 0.0f, 0.0f, 1.0f}};
        rpInfo.clearValueCount = 1;
        rpInfo.pClearValues = clearValues;

        inlineMode = true; vkCmdBeginRenderPass(currentCmd_, &rpInfo, VK_SUBPASS_CONTENTS_INLINE);

        VkExtent2D ext = vkCtx_->getSwapchainExtent();
        VkViewport vp{};
        vp.width = static_cast<float>(ext.width);
        vp.height = static_cast<float>(ext.height);
        vp.maxDepth = 1.0f;
        vkCmdSetViewport(currentCmd_, 0, 1, &vp);
        VkRect2D sc{};
        sc.extent = ext;
        vkCmdSetScissor(currentCmd_, 0, 1, &sc);

        // When FXAA is also enabled: apply FXAA on the FSR3 temporal output instead
        // of RCAS sharpening. FXAA descriptor is temporarily pointed to the FSR3
        // history buffer (which is already in SHADER_READ_ONLY_OPTIMAL). This gives
        // FSR3 temporal stability + FXAA spatial edge smoothing ("ultra quality native").
        if (needsFXAAPass() && fxaa_.pipeline && fxaa_.descSet[fxaaFrameIdx]) {
            renderFXAAPass();
        } else {
            // Draw RCAS sharpening from accumulated history buffer
            renderFSR2Sharpen();
        }

        // Restore this frame's FXAA descriptor to its normal scene color source
        // so standalone FXAA frames are not affected by the FSR3 history pointer.
        if (needsFXAAPass() && fxaa_.descSet[fxaaFrameIdx] && fxaa_.sceneSampler && fxaa_.sceneColor.imageView) {
            VkDescriptorImageInfo restoreInfo{};
            restoreInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            restoreInfo.imageView   = fxaa_.sceneColor.imageView;
            restoreInfo.sampler     = fxaa_.sceneSampler;
            VkWriteDescriptorSet restoreWrite{};
            restoreWrite.sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            restoreWrite.dstSet          = fxaa_.descSet[fxaaFrameIdx];
            restoreWrite.dstBinding      = 0;
            restoreWrite.descriptorCount = 1;
            restoreWrite.descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            restoreWrite.pImageInfo      = &restoreInfo;
            vkUpdateDescriptorSets(vkCtx_->getDevice(), 1, &restoreWrite, 0, nullptr);
        }

        // Maintain frame bookkeeping
        fsr2_.prevViewProjection = camera_->getViewProjectionMatrix();
        fsr2_.prevJitter = camera_->getJitter();
        camera_->clearJitter();
        if (fsr2_.useAmdBackend) {
            fsr2_.currentHistory = 1 - fsr2_.currentHistory;
        }
        fsr2_.frameIndex = (fsr2_.frameIndex + 1) % 256;  // Wrap to keep Halton values well-distributed

    } else if (needsFXAAPass() && fxaa_.sceneFramebuffer) {
        inlineMode = true;
        // End the off-screen scene render pass
        vkCmdEndRenderPass(currentCmd_);

        // Transition resolved scene color: PRESENT_SRC_KHR → SHADER_READ_ONLY
        transitionImageLayout(currentCmd_, fxaa_.sceneColor.image,
            VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);

        // Begin swapchain render pass (1x - no MSAA on the output pass)
        VkRenderPassBeginInfo rpInfo{};
        rpInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
        // Output goes through the single-sampled overlay pass. The scene pass
        // carries the scene's sample count, and these quads are 1x.
        rpInfo.renderPass = vkCtx_->getOverlayClearRenderPass();
        rpInfo.framebuffer = vkCtx_->getOverlayFramebuffers()[imageIndex];
        rpInfo.renderArea.offset = {.x = 0, .y = 0};
        rpInfo.renderArea.extent = vkCtx_->getSwapchainExtent();
        VkClearValue fxaaClear[1]{};
        fxaaClear[0].color = {{0.0f, 0.0f, 0.0f, 1.0f}};
        rpInfo.clearValueCount = 1;
        rpInfo.pClearValues = fxaaClear;

        vkCmdBeginRenderPass(currentCmd_, &rpInfo, VK_SUBPASS_CONTENTS_INLINE);

        VkExtent2D ext = vkCtx_->getSwapchainExtent();
        VkViewport vp{};
        vp.width = static_cast<float>(ext.width);
        vp.height = static_cast<float>(ext.height);
        vp.maxDepth = 1.0f;
        vkCmdSetViewport(currentCmd_, 0, 1, &vp);
        VkRect2D sc{};
        sc.extent = ext;
        vkCmdSetScissor(currentCmd_, 0, 1, &sc);

        // Draw FXAA pass
        renderFXAAPass();

    } else if (fsr_.enabled && fsr_.sceneFramebuffer) {
        inlineMode = true;
        // FSR1 upscale path - only runs when FXAA is not active.
        // When both FSR1 and FXAA are enabled, FXAA took priority above.
        vkCmdEndRenderPass(currentCmd_);

        // Transition scene color (1x resolve/color target): PRESENT_SRC_KHR → SHADER_READ_ONLY
        transitionImageLayout(currentCmd_, fsr_.sceneColor.image,
            VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);

        // Begin swapchain render pass at full resolution
        VkRenderPassBeginInfo fsrRpInfo{};
        fsrRpInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
        // Output goes through the single-sampled overlay pass. The scene pass
        // carries the scene's sample count, and these quads are 1x.
        fsrRpInfo.renderPass = vkCtx_->getOverlayClearRenderPass();
        fsrRpInfo.framebuffer = vkCtx_->getOverlayFramebuffers()[imageIndex];
        fsrRpInfo.renderArea.offset = {.x = 0, .y = 0};
        fsrRpInfo.renderArea.extent = vkCtx_->getSwapchainExtent();

        VkClearValue fsrClearValues[1]{};
        fsrClearValues[0].color = {{0.0f, 0.0f, 0.0f, 1.0f}};
        fsrRpInfo.clearValueCount = 1;
        fsrRpInfo.pClearValues = fsrClearValues;

        vkCmdBeginRenderPass(currentCmd_, &fsrRpInfo, VK_SUBPASS_CONTENTS_INLINE);

        VkExtent2D fsrExt = vkCtx_->getSwapchainExtent();
        VkViewport fsrVp{};
        fsrVp.width = static_cast<float>(fsrExt.width);
        fsrVp.height = static_cast<float>(fsrExt.height);
        fsrVp.maxDepth = 1.0f;
        vkCmdSetViewport(currentCmd_, 0, 1, &fsrVp);
        VkRect2D fsrSc{};
        fsrSc.extent = fsrExt;
        vkCmdSetScissor(currentCmd_, 0, 1, &fsrSc);

        renderFSRUpscale();
    }

    currentCmd_ = VK_NULL_HANDLE;
    camera_ = nullptr;
    return inlineMode;
}

void PostProcessPipeline::destroyAllResources() {
    if (fsr_.sceneFramebuffer) destroyFSRResources();
    if (fsr2_.sceneFramebuffer) destroyFSR2Resources();
    if (fxaa_.sceneFramebuffer) destroyFXAAResources();
}

// ========================= Public API =========================

void PostProcessPipeline::setFXAAEnabled(bool enabled) {
    if (fxaa_.enabled == enabled) return;
    // FXAA is a post-process AA pass intended to supplement FSR temporal output.
    // It conflicts with MSAA (which resolves AA during the scene render pass), so
    // refuse to enable FXAA when hardware MSAA is active.
    if (enabled && vkCtx_ && vkCtx_->getMsaaSamples() > VK_SAMPLE_COUNT_1_BIT) {
        LOG_INFO("FXAA: blocked while MSAA is active - disable MSAA first");
        return;
    }
    fxaa_.enabled = enabled;
    if (!enabled) {
        fxaa_.needsRecreate = true;  // defer destruction to next beginFrame()
    }
}

MsaaChangeRequest PostProcessPipeline::setFSREnabled(bool enabled) {
    MsaaChangeRequest req;
    if (fsr_.enabled == enabled) return req;
    fsr_.enabled = enabled;

    if (enabled) {
        // FSR1 upscaling renders its own AA - disable MSAA to avoid redundant work
        if (vkCtx_ && vkCtx_->getMsaaSamples() > VK_SAMPLE_COUNT_1_BIT) {
            req.requested = true;
            req.samples = VK_SAMPLE_COUNT_1_BIT;
        }
    } else {
        // Defer destruction to next beginFrame() - can't destroy mid-render
        fsr_.needsRecreate = true;
    }
    // Resources created/destroyed lazily in beginFrame()
    return req;
}

void PostProcessPipeline::setFSRQuality(float scaleFactor) {
    scaleFactor = glm::clamp(scaleFactor, 0.5f, 1.0f);
    fsr_.scaleFactor = scaleFactor;
    fsr2_.scaleFactor = scaleFactor;
    // Don't destroy/recreate mid-frame - mark for lazy recreation in next beginFrame()
    if (fsr_.enabled && fsr_.sceneFramebuffer) {
        fsr_.needsRecreate = true;
    }
    if (fsr2_.enabled && fsr2_.sceneFramebuffer) {
        fsr2_.needsRecreate = true;
        fsr2_.needsHistoryReset = true;
    }
}

void PostProcessPipeline::setFSRSharpness(float sharpness) {
    fsr_.sharpness = glm::clamp(sharpness, 0.0f, 2.0f);
    fsr2_.sharpness = glm::clamp(sharpness, 0.0f, 2.0f);
}

MsaaChangeRequest PostProcessPipeline::setFSR2Enabled(bool enabled, Camera* camera) {
    MsaaChangeRequest req;
    if (fsr2_.enabled == enabled) return req;
    fsr2_.enabled = enabled;

    if (enabled) {
        static bool initFramegenToggleFromEnv = false;
        if (!initFramegenToggleFromEnv) {
            initFramegenToggleFromEnv = true;
            if (std::getenv("WOWEE_ENABLE_AMD_FSR3_FRAMEGEN_RUNTIME") != nullptr) {
                fsr2_.amdFsr3FramegenEnabled = true;
            }
        }
        // FSR2 replaces both FSR1 and MSAA
        if (fsr_.enabled) {
            fsr_.enabled = false;
            fsr_.needsRecreate = true;
        }
        // FSR2 requires non-MSAA render pass (its framebuffer has 2 attachments)
        if (vkCtx_ && vkCtx_->getMsaaSamples() > VK_SAMPLE_COUNT_1_BIT) {
            req.requested = true;
            req.samples = VK_SAMPLE_COUNT_1_BIT;
        }
        // Use FSR1's scale factor and sharpness as defaults
        fsr2_.scaleFactor = fsr_.scaleFactor;
        fsr2_.sharpness = fsr_.sharpness;
        fsr2_.needsHistoryReset = true;
    } else {
        fsr2_.needsRecreate = true;
        if (camera) camera->clearJitter();
    }
    return req;
}

void PostProcessPipeline::setFSR2DebugTuning(float jitterSign, float motionVecScaleX, float motionVecScaleY) {
    fsr2_.jitterSign = glm::clamp(jitterSign, -2.0f, 2.0f);
    fsr2_.motionVecScaleX = glm::clamp(motionVecScaleX, -2.0f, 2.0f);
    fsr2_.motionVecScaleY = glm::clamp(motionVecScaleY, -2.0f, 2.0f);
}

void PostProcessPipeline::setAmdFsr3FramegenEnabled(bool enabled) {
    if (fsr2_.amdFsr3FramegenEnabled == enabled) return;
    fsr2_.amdFsr3FramegenEnabled = enabled;
#if WOWEE_HAS_AMD_FSR3_FRAMEGEN
    if (enabled) {
        fsr2_.amdFsr3FramegenRuntimeActive = false;
        fsr2_.framegenOutputValid = false;
        fsr2_.needsRecreate = true;
        fsr2_.needsHistoryReset = true;
        fsr2_.amdFsr3FramegenRuntimeReady = false;
        fsr2_.amdFsr3RuntimePath = "Path C";
        fsr2_.amdFsr3RuntimeLastError.clear();
        LOG_INFO("FSR3 framegen requested; runtime will initialize on next FSR2 resource creation.");
    } else {
        fsr2_.amdFsr3FramegenRuntimeActive = false;
        fsr2_.amdFsr3FramegenRuntimeReady = false;
        fsr2_.framegenOutputValid = false;
        fsr2_.needsHistoryReset = true;
        fsr2_.needsRecreate = true;
        fsr2_.amdFsr3RuntimePath = "Path C";
        fsr2_.amdFsr3RuntimeLastError = "disabled by user";
        if (fsr2_.amdFsr3Runtime) {
            fsr2_.amdFsr3Runtime->shutdown();
            fsr2_.amdFsr3Runtime.reset();
        }
        LOG_INFO("FSR3 framegen disabled; forcing FSR2-only path rebuild.");
    }
#else
    fsr2_.amdFsr3FramegenRuntimeActive = false;
    fsr2_.amdFsr3FramegenRuntimeReady = false;
    fsr2_.framegenOutputValid = false;
    if (enabled) {
        LOG_WARNING("FSR3 framegen requested, but AMD FSR3 framegen SDK headers are unavailable in this build.");
    }
#endif
}

const char* PostProcessPipeline::getAmdFsr3FramegenRuntimePath() const {
    return fsr2_.amdFsr3RuntimePath.c_str();
}

// ========================= FSR 1.0 Upscaling =========================

bool PostProcessPipeline::initFSRResources() {
    if (!vkCtx_) return false;

    VkDevice device = vkCtx_->getDevice();
    VmaAllocator alloc = vkCtx_->getAllocator();
    VkExtent2D swapExtent = vkCtx_->getSwapchainExtent();
    VkSampleCountFlagBits msaa = vkCtx_->getMsaaSamples();
    bool useMsaa = (msaa > VK_SAMPLE_COUNT_1_BIT);
    bool useDepthResolve = (vkCtx_->getDepthResolveImageView() != VK_NULL_HANDLE);

    fsr_.internalWidth = static_cast<uint32_t>(swapExtent.width * fsr_.scaleFactor);
    fsr_.internalHeight = static_cast<uint32_t>(swapExtent.height * fsr_.scaleFactor);
    fsr_.internalWidth = (fsr_.internalWidth + 1) & ~1u;
    fsr_.internalHeight = (fsr_.internalHeight + 1) & ~1u;

    LOG_INFO("FSR: initializing at ", fsr_.internalWidth, "x", fsr_.internalHeight,
             " -> ", swapExtent.width, "x", swapExtent.height,
             " (scale=", fsr_.scaleFactor, ", MSAA=", static_cast<int>(msaa), "x)");

    VkFormat colorFmt = vkCtx_->getSwapchainFormat();
    VkFormat depthFmt = vkCtx_->getDepthFormat();

    // sceneColor: always 1x, always sampled - this is what FSR reads
    // Non-MSAA: direct render target. MSAA: resolve target.
    fsr_.sceneColor = createImage(device, alloc, fsr_.internalWidth, fsr_.internalHeight,
        colorFmt, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT
                | VK_IMAGE_USAGE_TRANSFER_SRC_BIT);
    if (!fsr_.sceneColor.image) {
        LOG_ERROR("FSR: failed to create scene color image");
        return false;
    }

    // sceneDepth: matches current MSAA sample count
    fsr_.sceneDepth = createImage(device, alloc, fsr_.internalWidth, fsr_.internalHeight,
        depthFmt, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT
                | VK_IMAGE_USAGE_TRANSFER_SRC_BIT, msaa);
    if (!fsr_.sceneDepth.image) {
        LOG_ERROR("FSR: failed to create scene depth image");
        destroyFSRResources();
        return false;
    }

    if (useMsaa) {
        // sceneMsaaColor: multisampled color target
        fsr_.sceneMsaaColor = createImage(device, alloc, fsr_.internalWidth, fsr_.internalHeight,
            colorFmt, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT, msaa);
        if (!fsr_.sceneMsaaColor.image) {
            LOG_ERROR("FSR: failed to create MSAA color image");
            destroyFSRResources();
            return false;
        }

        if (useDepthResolve) {
            fsr_.sceneDepthResolve = createImage(device, alloc, fsr_.internalWidth, fsr_.internalHeight,
                depthFmt, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT
                        | VK_IMAGE_USAGE_TRANSFER_SRC_BIT);
            if (!fsr_.sceneDepthResolve.image) {
                LOG_ERROR("FSR: failed to create depth resolve image");
                destroyFSRResources();
                return false;
            }
        }
    }

    // Build framebuffer matching the main render pass attachment layout:
    //   Non-MSAA:              [color, depth]
    //   MSAA (no depth res):   [msaaColor, depth, resolve]
    //   MSAA (depth res):      [msaaColor, depth, resolve, depthResolve]
    VkImageView fbAttachments[4]{};
    uint32_t fbCount;
    if (useMsaa) {
        fbAttachments[0] = fsr_.sceneMsaaColor.imageView;
        fbAttachments[1] = fsr_.sceneDepth.imageView;
        fbAttachments[2] = fsr_.sceneColor.imageView;  // resolve target
        fbCount = 3;
        if (useDepthResolve) {
            fbAttachments[3] = fsr_.sceneDepthResolve.imageView;
            fbCount = 4;
        }
    } else {
        fbAttachments[0] = fsr_.sceneColor.imageView;
        fbAttachments[1] = fsr_.sceneDepth.imageView;
        fbCount = 2;
    }

    VkFramebufferCreateInfo fbInfo{};
    fbInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
    fbInfo.renderPass = vkCtx_->getImGuiRenderPass();
    fbInfo.attachmentCount = fbCount;
    fbInfo.pAttachments = fbAttachments;
    fbInfo.width = fsr_.internalWidth;
    fbInfo.height = fsr_.internalHeight;
    fbInfo.layers = 1;

    if (vkCreateFramebuffer(device, &fbInfo, nullptr, &fsr_.sceneFramebuffer) != VK_SUCCESS) {
        LOG_ERROR("FSR: failed to create scene framebuffer");
        destroyFSRResources();
        return false;
    }

    // Sampler for the resolved scene color
    VkSamplerCreateInfo samplerInfo{};
    samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    samplerInfo.minFilter = VK_FILTER_LINEAR;
    samplerInfo.magFilter = VK_FILTER_LINEAR;
    samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    fsr_.sceneSampler = vkCtx_->getOrCreateSampler(samplerInfo);
    if (fsr_.sceneSampler == VK_NULL_HANDLE) {
        LOG_ERROR("FSR: failed to create sampler");
        destroyFSRResources();
        return false;
    }

    // Descriptor set layout: binding 0 = combined image sampler
    VkDescriptorSetLayoutBinding binding{};
    binding.binding = 0;
    binding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    binding.descriptorCount = 1;
    binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutInfo.bindingCount = 1;
    layoutInfo.pBindings = &binding;
    vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr, &fsr_.descSetLayout);

    VkDescriptorPoolSize poolSize{};
    poolSize.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    poolSize.descriptorCount = 1;
    VkDescriptorPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.maxSets = 1;
    poolInfo.poolSizeCount = 1;
    poolInfo.pPoolSizes = &poolSize;
    vkCreateDescriptorPool(device, &poolInfo, nullptr, &fsr_.descPool);

    VkDescriptorSetAllocateInfo dsAllocInfo{};
    dsAllocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    dsAllocInfo.descriptorPool = fsr_.descPool;
    dsAllocInfo.descriptorSetCount = 1;
    dsAllocInfo.pSetLayouts = &fsr_.descSetLayout;
    vkAllocateDescriptorSets(device, &dsAllocInfo, &fsr_.descSet);

    // Always bind the 1x sceneColor (FSR reads the resolved image)
    VkDescriptorImageInfo imgInfo{};
    imgInfo.sampler = fsr_.sceneSampler;
    imgInfo.imageView = fsr_.sceneColor.imageView;
    imgInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkWriteDescriptorSet write{};
    write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    write.dstSet = fsr_.descSet;
    write.dstBinding = 0;
    write.descriptorCount = 1;
    write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    write.pImageInfo = &imgInfo;
    vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);

    // Pipeline layout
    VkPushConstantRange pc{};
    pc.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    pc.offset = 0;
    pc.size = 64;
    VkPipelineLayoutCreateInfo plCI{};
    plCI.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    plCI.setLayoutCount = 1;
    plCI.pSetLayouts = &fsr_.descSetLayout;
    plCI.pushConstantRangeCount = 1;
    plCI.pPushConstantRanges = &pc;
    vkCreatePipelineLayout(device, &plCI, nullptr, &fsr_.pipelineLayout);

    // Load shaders
    VkShaderModule vertMod, fragMod;
    if (!vertMod.loadFromFile(device, "assets/shaders/postprocess.vert.spv") ||
        !fragMod.loadFromFile(device, "assets/shaders/fsr_easu.frag.spv")) {
        LOG_ERROR("FSR: failed to load shaders");
        destroyFSRResources();
        return false;
    }

    // FSR upscale pipeline renders into the swapchain pass at full resolution
    // Must match swapchain pass MSAA setting
    fsr_.pipeline = PipelineBuilder()
        .setShaders(vertMod.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                    fragMod.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
        .setVertexInput({}, {})
        .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
        .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
        .setNoDepthTest()
        .setColorBlendAttachment(PipelineBuilder::blendDisabled())
        .setMultisample(VK_SAMPLE_COUNT_1_BIT)
        .setLayout(fsr_.pipelineLayout)
        .setRenderPass(vkCtx_->getOverlayRenderPass())
        .setDynamicStates(viewportAndScissorDynamic())
        .build(device, vkCtx_->getPipelineCache());

    vertMod.destroy();
    fragMod.destroy();

    if (!fsr_.pipeline) {
        LOG_ERROR("FSR: failed to create upscale pipeline");
        destroyFSRResources();
        return false;
    }

    LOG_INFO("FSR: initialized successfully");
    return true;
}

void PostProcessPipeline::destroyFSRResources() {
    if (!vkCtx_) return;

    VkDevice device = vkCtx_->getDevice();
    VmaAllocator alloc = vkCtx_->getAllocator();
    vkDeviceWaitIdle(device);

    destroy(device, fsr_.pipeline);
    destroy(device, fsr_.pipelineLayout);
    if (fsr_.descPool) { vkDestroyDescriptorPool(device, fsr_.descPool, nullptr); fsr_.descPool = VK_NULL_HANDLE; fsr_.descSet = VK_NULL_HANDLE; }
    destroy(device, fsr_.descSetLayout);
    if (fsr_.sceneFramebuffer) { vkDestroyFramebuffer(device, fsr_.sceneFramebuffer, nullptr); fsr_.sceneFramebuffer = VK_NULL_HANDLE; }
    fsr_.sceneSampler = VK_NULL_HANDLE; // Owned by VkContext sampler cache
    destroyImage(device, alloc, fsr_.sceneDepthResolve);
    destroyImage(device, alloc, fsr_.sceneMsaaColor);
    destroyImage(device, alloc, fsr_.sceneDepth);
    destroyImage(device, alloc, fsr_.sceneColor);

    fsr_.internalWidth = 0;
    fsr_.internalHeight = 0;
}

void PostProcessPipeline::renderFSRUpscale() {
    if (!fsr_.pipeline || currentCmd_ == VK_NULL_HANDLE) return;

    VkExtent2D outExtent = vkCtx_->getSwapchainExtent();
    float inW = static_cast<float>(fsr_.internalWidth);
    float inH = static_cast<float>(fsr_.internalHeight);
    float outW = static_cast<float>(outExtent.width);
    float outH = static_cast<float>(outExtent.height);

    // FSR push constants
    struct {
        glm::vec4 con0;  // inputSize.xy, 1/inputSize.xy
        glm::vec4 con1;  // inputSize.xy / outputSize.xy, 0.5 * inputSize.xy / outputSize.xy
        glm::vec4 con2;  // outputSize.xy, 1/outputSize.xy
        glm::vec4 con3;  // sharpness, 0, 0, 0
    } fsrConst;

    fsrConst.con0 = glm::vec4(inW, inH, 1.0f / inW, 1.0f / inH);
    fsrConst.con1 = glm::vec4(inW / outW, inH / outH, 0.5f * inW / outW, 0.5f * inH / outH);
    fsrConst.con2 = glm::vec4(outW, outH, 1.0f / outW, 1.0f / outH);
    fsrConst.con3 = glm::vec4(fsr_.sharpness, 0.0f, 0.0f, 0.0f);

    vkCmdBindPipeline(currentCmd_, VK_PIPELINE_BIND_POINT_GRAPHICS, fsr_.pipeline);
    vkCmdBindDescriptorSets(currentCmd_, VK_PIPELINE_BIND_POINT_GRAPHICS,
        fsr_.pipelineLayout, 0, 1, &fsr_.descSet, 0, nullptr);
    vkCmdPushConstants(currentCmd_, fsr_.pipelineLayout,
        VK_SHADER_STAGE_FRAGMENT_BIT, 0, 64, &fsrConst);
    vkCmdDraw(currentCmd_, 3, 1, 0, 0);
}

// ========================= FSR 2.2 Temporal Upscaling =========================

float PostProcessPipeline::halton(uint32_t index, uint32_t base) {
    float f = 1.0f;
    float r = 0.0f;
    uint32_t current = index;
    while (current > 0) {
        f /= static_cast<float>(base);
        r += f * static_cast<float>(current % base);
        current /= base;
    }
    return r;
}

bool PostProcessPipeline::initFSR2Resources() {
    if (!vkCtx_) return false;

    VkDevice device = vkCtx_->getDevice();
    VmaAllocator alloc = vkCtx_->getAllocator();
    VkExtent2D swapExtent = vkCtx_->getSwapchainExtent();

    // Temporary stability fallback: keep FSR2 path at native internal resolution
    // until temporal reprojection is reworked.
    fsr2_.internalWidth = static_cast<uint32_t>(swapExtent.width * fsr2_.scaleFactor);
    fsr2_.internalHeight = static_cast<uint32_t>(swapExtent.height * fsr2_.scaleFactor);
    fsr2_.internalWidth = (fsr2_.internalWidth + 1) & ~1u;
    fsr2_.internalHeight = (fsr2_.internalHeight + 1) & ~1u;

    LOG_INFO("FSR2: initializing at ", fsr2_.internalWidth, "x", fsr2_.internalHeight,
             " -> ", swapExtent.width, "x", swapExtent.height,
             " (scale=", fsr2_.scaleFactor, ")");
    fsr2_.useAmdBackend = false;
    fsr2_.amdFsr3FramegenRuntimeActive = false;
    fsr2_.amdFsr3FramegenRuntimeReady = false;
    fsr2_.framegenOutputValid = false;
    fsr2_.amdFsr3RuntimePath = "Path C";
    fsr2_.amdFsr3RuntimeLastError.clear();
    fsr2_.amdFsr3UpscaleDispatchCount = 0;
    fsr2_.amdFsr3FramegenDispatchCount = 0;
    fsr2_.amdFsr3FallbackCount = 0;
#if WOWEE_HAS_AMD_FSR2
    LOG_INFO("FSR2: AMD FidelityFX SDK detected at build time.");
#else
    LOG_WARNING("FSR2: AMD FidelityFX SDK not detected; using internal fallback path.");
#endif

    VkFormat colorFmt = vkCtx_->getSwapchainFormat();
    VkFormat depthFmt = vkCtx_->getDepthFormat();

    // Scene color (internal resolution, 1x - FSR2 replaces MSAA)
    fsr2_.sceneColor = createImage(device, alloc, fsr2_.internalWidth, fsr2_.internalHeight,
        colorFmt, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT
                | VK_IMAGE_USAGE_TRANSFER_SRC_BIT);
    if (!fsr2_.sceneColor.image) { LOG_ERROR("FSR2: failed to create scene color"); return false; }

    // Scene depth (internal resolution, 1x, sampled for motion vectors)
    fsr2_.sceneDepth = createImage(device, alloc, fsr2_.internalWidth, fsr2_.internalHeight,
        depthFmt, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT
                | VK_IMAGE_USAGE_TRANSFER_SRC_BIT);
    if (!fsr2_.sceneDepth.image) { LOG_ERROR("FSR2: failed to create scene depth"); destroyFSR2Resources(); return false; }

    // Motion vector buffer (internal resolution)
    fsr2_.motionVectors = createImage(device, alloc, fsr2_.internalWidth, fsr2_.internalHeight,
        VK_FORMAT_R16G16_SFLOAT, VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT);
    if (!fsr2_.motionVectors.image) { LOG_ERROR("FSR2: failed to create motion vectors"); destroyFSR2Resources(); return false; }

    // History buffers (display resolution, ping-pong)
    for (int i = 0; i < 2; i++) {
        fsr2_.history[i] = createImage(device, alloc, swapExtent.width, swapExtent.height,
            VK_FORMAT_R16G16B16A16_SFLOAT,
            VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT);
        if (!fsr2_.history[i].image) { LOG_ERROR("FSR2: failed to create history buffer ", i); destroyFSR2Resources(); return false; }
    }
    fsr2_.framegenOutput = createImage(device, alloc, swapExtent.width, swapExtent.height,
        VK_FORMAT_R16G16B16A16_SFLOAT, VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT);
    if (!fsr2_.framegenOutput.image) { LOG_ERROR("FSR2: failed to create framegen output"); destroyFSR2Resources(); return false; }

    // Scene framebuffer (non-MSAA: [color, depth])
    // Must use the same render pass as the swapchain - which must be non-MSAA when FSR2 is active
    VkImageView fbAttachments[2] = { fsr2_.sceneColor.imageView, fsr2_.sceneDepth.imageView };
    VkFramebufferCreateInfo fbInfo{};
    fbInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
    fbInfo.renderPass = vkCtx_->getImGuiRenderPass();
    fbInfo.attachmentCount = 2;
    fbInfo.pAttachments = fbAttachments;
    fbInfo.width = fsr2_.internalWidth;
    fbInfo.height = fsr2_.internalHeight;
    fbInfo.layers = 1;
    if (vkCreateFramebuffer(device, &fbInfo, nullptr, &fsr2_.sceneFramebuffer) != VK_SUCCESS) {
        LOG_ERROR("FSR2: failed to create scene framebuffer");
        destroyFSR2Resources();
        return false;
    }

    // Samplers
    VkSamplerCreateInfo samplerInfo{};
    samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    samplerInfo.minFilter = VK_FILTER_LINEAR;
    samplerInfo.magFilter = VK_FILTER_LINEAR;
    samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    fsr2_.linearSampler = vkCtx_->getOrCreateSampler(samplerInfo);

    samplerInfo.minFilter = VK_FILTER_NEAREST;
    samplerInfo.magFilter = VK_FILTER_NEAREST;
    fsr2_.nearestSampler = vkCtx_->getOrCreateSampler(samplerInfo);

#if WOWEE_HAS_AMD_FSR2
    // Initialize AMD FSR2 context; fall back to internal path on any failure.
    fsr2_.amdScratchBufferSize = ffxFsr2GetScratchMemorySizeVK(vkCtx_->getPhysicalDevice());
    if (fsr2_.amdScratchBufferSize > 0) {
        fsr2_.amdScratchBuffer = std::malloc(fsr2_.amdScratchBufferSize);
    }
    if (!fsr2_.amdScratchBuffer) {
        LOG_WARNING("FSR2 AMD: failed to allocate scratch buffer, using internal fallback.");
    } else {
        FfxErrorCode ifaceErr = ffxFsr2GetInterfaceVK(
            &fsr2_.amdInterface,
            fsr2_.amdScratchBuffer,
            fsr2_.amdScratchBufferSize,
            vkCtx_->getPhysicalDevice(),
            vkGetDeviceProcAddr);
        if (ifaceErr != FFX_OK) {
            LOG_WARNING("FSR2 AMD: ffxFsr2GetInterfaceVK failed (", static_cast<int>(ifaceErr), "), using internal fallback.");
            std::free(fsr2_.amdScratchBuffer);
            fsr2_.amdScratchBuffer = nullptr;
            fsr2_.amdScratchBufferSize = 0;
        } else {
            FfxFsr2ContextDescription ctxDesc{};
            ctxDesc.flags = FFX_FSR2_ENABLE_AUTO_EXPOSURE | FFX_FSR2_ENABLE_MOTION_VECTORS_JITTER_CANCELLATION;
            ctxDesc.maxRenderSize.width = fsr2_.internalWidth;
            ctxDesc.maxRenderSize.height = fsr2_.internalHeight;
            ctxDesc.displaySize.width = swapExtent.width;
            ctxDesc.displaySize.height = swapExtent.height;
            ctxDesc.callbacks = fsr2_.amdInterface;
            ctxDesc.device = ffxGetDeviceVK(vkCtx_->getDevice());
            ctxDesc.fpMessage = nullptr;

            FfxErrorCode ctxErr = ffxFsr2ContextCreate(&fsr2_.amdContext, &ctxDesc);
            if (ctxErr == FFX_OK) {
                fsr2_.useAmdBackend = true;
                LOG_INFO("FSR2 AMD: context created successfully.");
#if WOWEE_HAS_AMD_FSR3_FRAMEGEN
                // FSR3 frame generation runtime uses AMD FidelityFX SDK which can
                // corrupt Vulkan driver state on NVIDIA GPUs when context creation
                // fails, causing subsequent vkCmdBeginRenderPass to crash.
                // Skip FSR3 frame gen entirely on non-AMD GPUs.
                if (fsr2_.amdFsr3FramegenEnabled && vkCtx_->isAmdGpu()) {
                    fsr2_.amdFsr3FramegenRuntimeActive = false;
                    if (!fsr2_.amdFsr3Runtime) fsr2_.amdFsr3Runtime = std::make_unique<AmdFsr3Runtime>();
                    AmdFsr3RuntimeInitDesc fgInit{};
                    fgInit.physicalDevice = vkCtx_->getPhysicalDevice();
                    fgInit.device = vkCtx_->getDevice();
                    fgInit.getDeviceProcAddr = vkGetDeviceProcAddr;
                    fgInit.maxRenderWidth = fsr2_.internalWidth;
                    fgInit.maxRenderHeight = fsr2_.internalHeight;
                    fgInit.displayWidth = swapExtent.width;
                    fgInit.displayHeight = swapExtent.height;
                    fgInit.colorFormat = VK_FORMAT_R16G16B16A16_SFLOAT;
                    fgInit.hdrInput = false;
                    fgInit.depthInverted = false;
                    fgInit.enableFrameGeneration = true;
                    fsr2_.amdFsr3FramegenRuntimeReady = fsr2_.amdFsr3Runtime->initialize(fgInit);
                    if (fsr2_.amdFsr3FramegenRuntimeReady) {
                        fsr2_.amdFsr3RuntimeLastError.clear();
                        fsr2_.amdFsr3RuntimePath = "Path A";
                        LOG_INFO("FSR3 framegen runtime library loaded from ", fsr2_.amdFsr3Runtime->loadedLibraryPath(),
                                 " (upscale+framegen dispatch enabled)");
                    } else {
                        fsr2_.amdFsr3RuntimePath = "Path C";
                        fsr2_.amdFsr3RuntimeLastError = fsr2_.amdFsr3Runtime->lastError();
                        LOG_WARNING("FSR3 framegen toggle is enabled, but runtime initialization failed. ",
                                    "path=", fsr2_.amdFsr3RuntimePath,
                                    " error=", fsr2_.amdFsr3RuntimeLastError.empty() ? "(none)" : fsr2_.amdFsr3RuntimeLastError,
                                    " runtimeLib=", fsr2_.amdFsr3Runtime->loadedLibraryPath().empty() ? "(not loaded)" : fsr2_.amdFsr3Runtime->loadedLibraryPath());
                    }
                }
#endif
            } else {
                LOG_WARNING("FSR2 AMD: context creation failed (", static_cast<int>(ctxErr), "), using internal fallback.");
                std::free(fsr2_.amdScratchBuffer);
                fsr2_.amdScratchBuffer = nullptr;
                fsr2_.amdScratchBufferSize = 0;
            }
        }
    }
#endif

    // --- Motion Vector Compute Pipeline ---
    {
        // Descriptor set layout: binding 0 = depth (sampler), binding 1 = motion vectors (storage image)
        VkDescriptorSetLayoutBinding bindings[2] = {};
        bindings[0].binding = 0;
        bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        bindings[0].descriptorCount = 1;
        bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        bindings[1].binding = 1;
        bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
        bindings[1].descriptorCount = 1;
        bindings[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

        VkDescriptorSetLayoutCreateInfo layoutInfo{.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        layoutInfo.bindingCount = 2;
        layoutInfo.pBindings = bindings;
        vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr, &fsr2_.motionVecDescSetLayout);

        VkPushConstantRange pc{};
        pc.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        pc.offset = 0;
        pc.size = 2 * sizeof(glm::mat4);  // 128 bytes

        VkPipelineLayoutCreateInfo plCI{.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        plCI.setLayoutCount = 1;
        plCI.pSetLayouts = &fsr2_.motionVecDescSetLayout;
        plCI.pushConstantRangeCount = 1;
        plCI.pPushConstantRanges = &pc;
        vkCreatePipelineLayout(device, &plCI, nullptr, &fsr2_.motionVecPipelineLayout);

        VkShaderModule compMod;
        if (!compMod.loadFromFile(device, "assets/shaders/fsr2_motion.comp.spv")) {
            LOG_ERROR("FSR2: failed to load motion vector compute shader");
            destroyFSR2Resources();
            return false;
        }

        VkComputePipelineCreateInfo cpCI{.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
        cpCI.stage = compMod.stageInfo(VK_SHADER_STAGE_COMPUTE_BIT);
        cpCI.layout = fsr2_.motionVecPipelineLayout;
        if (vkCreateComputePipelines(device, VK_NULL_HANDLE, 1, &cpCI, nullptr, &fsr2_.motionVecPipeline) != VK_SUCCESS) {
            LOG_ERROR("FSR2: failed to create motion vector pipeline");
            compMod.destroy();
            destroyFSR2Resources();
            return false;
        }
        compMod.destroy();

        // Descriptor pool + set
        VkDescriptorPoolSize poolSizes[2] = {};
        poolSizes[0] = {.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, .descriptorCount = 1};
        poolSizes[1] = {.type = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, .descriptorCount = 1};
        VkDescriptorPoolCreateInfo poolInfo{.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        poolInfo.maxSets = 1;
        poolInfo.poolSizeCount = 2;
        poolInfo.pPoolSizes = poolSizes;
        vkCreateDescriptorPool(device, &poolInfo, nullptr, &fsr2_.motionVecDescPool);

        VkDescriptorSetAllocateInfo dsAI{.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        dsAI.descriptorPool = fsr2_.motionVecDescPool;
        dsAI.descriptorSetCount = 1;
        dsAI.pSetLayouts = &fsr2_.motionVecDescSetLayout;
        vkAllocateDescriptorSets(device, &dsAI, &fsr2_.motionVecDescSet);

        // Write descriptors
        VkDescriptorImageInfo depthImgInfo{};
        depthImgInfo.sampler = fsr2_.nearestSampler;
        depthImgInfo.imageView = fsr2_.sceneDepth.imageView;
        depthImgInfo.imageLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;

        VkDescriptorImageInfo mvImgInfo{};
        mvImgInfo.imageView = fsr2_.motionVectors.imageView;
        mvImgInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;

        VkWriteDescriptorSet writes[2] = {};
        writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[0].dstSet = fsr2_.motionVecDescSet;
        writes[0].dstBinding = 0;
        writes[0].descriptorCount = 1;
        writes[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        writes[0].pImageInfo = &depthImgInfo;

        writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[1].dstSet = fsr2_.motionVecDescSet;
        writes[1].dstBinding = 1;
        writes[1].descriptorCount = 1;
        writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
        writes[1].pImageInfo = &mvImgInfo;

        vkUpdateDescriptorSets(device, 2, writes, 0, nullptr);
    }

    // --- Temporal Accumulation Compute Pipeline ---
    {
        // bindings: 0=sceneColor, 1=depth, 2=motionVectors, 3=historyInput, 4=historyOutput
        VkDescriptorSetLayoutBinding bindings[5] = {};
        for (int i = 0; i < 4; i++) {
            bindings[i].binding = i;
            bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            bindings[i].descriptorCount = 1;
            bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        }
        bindings[4].binding = 4;
        bindings[4].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
        bindings[4].descriptorCount = 1;
        bindings[4].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

        VkDescriptorSetLayoutCreateInfo layoutInfo{.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        layoutInfo.bindingCount = 5;
        layoutInfo.pBindings = bindings;
        vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr, &fsr2_.accumulateDescSetLayout);

        VkPushConstantRange pc{};
        pc.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        pc.offset = 0;
        pc.size = 4 * sizeof(glm::vec4);  // 64 bytes

        VkPipelineLayoutCreateInfo plCI{.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        plCI.setLayoutCount = 1;
        plCI.pSetLayouts = &fsr2_.accumulateDescSetLayout;
        plCI.pushConstantRangeCount = 1;
        plCI.pPushConstantRanges = &pc;
        vkCreatePipelineLayout(device, &plCI, nullptr, &fsr2_.accumulatePipelineLayout);

        VkShaderModule compMod;
        if (!compMod.loadFromFile(device, "assets/shaders/fsr2_accumulate.comp.spv")) {
            LOG_ERROR("FSR2: failed to load accumulation compute shader");
            destroyFSR2Resources();
            return false;
        }

        VkComputePipelineCreateInfo cpCI{.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
        cpCI.stage = compMod.stageInfo(VK_SHADER_STAGE_COMPUTE_BIT);
        cpCI.layout = fsr2_.accumulatePipelineLayout;
        if (vkCreateComputePipelines(device, VK_NULL_HANDLE, 1, &cpCI, nullptr, &fsr2_.accumulatePipeline) != VK_SUCCESS) {
            LOG_ERROR("FSR2: failed to create accumulation pipeline");
            compMod.destroy();
            destroyFSR2Resources();
            return false;
        }
        compMod.destroy();

        // Descriptor pool: 2 sets (ping-pong), each with 4 samplers + 1 storage image
        VkDescriptorPoolSize poolSizes[2] = {};
        poolSizes[0] = {.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, .descriptorCount = 8};
        poolSizes[1] = {.type = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, .descriptorCount = 2};
        VkDescriptorPoolCreateInfo poolInfo{.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        poolInfo.maxSets = 2;
        poolInfo.poolSizeCount = 2;
        poolInfo.pPoolSizes = poolSizes;
        vkCreateDescriptorPool(device, &poolInfo, nullptr, &fsr2_.accumulateDescPool);

        // Allocate 2 descriptor sets (one per ping-pong direction)
        VkDescriptorSetLayout layouts[2] = { fsr2_.accumulateDescSetLayout, fsr2_.accumulateDescSetLayout };
        VkDescriptorSetAllocateInfo dsAI{.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        dsAI.descriptorPool = fsr2_.accumulateDescPool;
        dsAI.descriptorSetCount = 2;
        dsAI.pSetLayouts = layouts;
        vkAllocateDescriptorSets(device, &dsAI, fsr2_.accumulateDescSets);

        // Write descriptors for both ping-pong sets
        for (int pp = 0; pp < 2; pp++) {
            int inputHistory = 1 - pp;   // Read from the other
            int outputHistory = pp;       // Write to this one

            // The accumulation shader already performs custom Lanczos reconstruction.
            // Use nearest here to avoid double filtering (linear + Lanczos) softening.
            VkDescriptorImageInfo colorInfo{.sampler = fsr2_.nearestSampler, .imageView = fsr2_.sceneColor.imageView, .imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
            VkDescriptorImageInfo depthInfo{.sampler = fsr2_.nearestSampler, .imageView = fsr2_.sceneDepth.imageView, .imageLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL};
            VkDescriptorImageInfo mvInfo{.sampler = fsr2_.nearestSampler, .imageView = fsr2_.motionVectors.imageView, .imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
            VkDescriptorImageInfo histInInfo{.sampler = fsr2_.linearSampler, .imageView = fsr2_.history[inputHistory].imageView, .imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
            VkDescriptorImageInfo histOutInfo{.sampler = VK_NULL_HANDLE, .imageView = fsr2_.history[outputHistory].imageView, .imageLayout = VK_IMAGE_LAYOUT_GENERAL};

            VkWriteDescriptorSet writes[5] = {};
            for (int w = 0; w < 5; w++) {
                writes[w].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                writes[w].dstSet = fsr2_.accumulateDescSets[pp];
                writes[w].dstBinding = w;
                writes[w].descriptorCount = 1;
            }
            writes[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER; writes[0].pImageInfo = &colorInfo;
            writes[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER; writes[1].pImageInfo = &depthInfo;
            writes[2].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER; writes[2].pImageInfo = &mvInfo;
            writes[3].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER; writes[3].pImageInfo = &histInInfo;
            writes[4].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;          writes[4].pImageInfo = &histOutInfo;

            vkUpdateDescriptorSets(device, 5, writes, 0, nullptr);
        }
    }

    // --- RCAS Sharpening Pipeline (fragment shader, fullscreen pass) ---
    {
        VkDescriptorSetLayoutBinding binding{};
        binding.binding = 0;
        binding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        binding.descriptorCount = 1;
        binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

        VkDescriptorSetLayoutCreateInfo layoutInfo{.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        layoutInfo.bindingCount = 1;
        layoutInfo.pBindings = &binding;
        vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr, &fsr2_.sharpenDescSetLayout);

        VkPushConstantRange pc{};
        pc.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        pc.offset = 0;
        pc.size = sizeof(glm::vec4);

        VkPipelineLayoutCreateInfo plCI{.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        plCI.setLayoutCount = 1;
        plCI.pSetLayouts = &fsr2_.sharpenDescSetLayout;
        plCI.pushConstantRangeCount = 1;
        plCI.pPushConstantRanges = &pc;
        vkCreatePipelineLayout(device, &plCI, nullptr, &fsr2_.sharpenPipelineLayout);

        VkShaderModule vertMod, fragMod;
        if (!vertMod.loadFromFile(device, "assets/shaders/postprocess.vert.spv") ||
            !fragMod.loadFromFile(device, "assets/shaders/fsr2_sharpen.frag.spv")) {
            LOG_ERROR("FSR2: failed to load sharpen shaders");
            destroyFSR2Resources();
            return false;
        }

        fsr2_.sharpenPipeline = PipelineBuilder()
            .setShaders(vertMod.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                        fragMod.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
            .setVertexInput({}, {})
            .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
            .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
            .setNoDepthTest()
            .setColorBlendAttachment(PipelineBuilder::blendDisabled())
            .setMultisample(VK_SAMPLE_COUNT_1_BIT)
            .setLayout(fsr2_.sharpenPipelineLayout)
            // The output quad is single-sampled, so it belongs to the overlay
            // pass. getImGuiRenderPass() is the scene pass, which carries the
            // scene's sample count - an 8x pass for a 1x pipeline.
            .setRenderPass(vkCtx_->getOverlayRenderPass())
            .setDynamicStates(viewportAndScissorDynamic())
            .build(device, vkCtx_->getPipelineCache());

        vertMod.destroy();
        fragMod.destroy();

        if (!fsr2_.sharpenPipeline) {
            LOG_ERROR("FSR2: failed to create sharpen pipeline");
            destroyFSR2Resources();
            return false;
        }

        // Descriptor pool + sets for sharpen pass (double-buffered to avoid race condition)
        VkDescriptorPoolSize poolSize{.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, .descriptorCount = 2};
        VkDescriptorPoolCreateInfo poolInfo{.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        poolInfo.maxSets = 2;
        poolInfo.poolSizeCount = 1;
        poolInfo.pPoolSizes = &poolSize;
        vkCreateDescriptorPool(device, &poolInfo, nullptr, &fsr2_.sharpenDescPool);

        VkDescriptorSetLayout layouts[2] = {fsr2_.sharpenDescSetLayout, fsr2_.sharpenDescSetLayout};
        VkDescriptorSetAllocateInfo dsAI{.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        dsAI.descriptorPool = fsr2_.sharpenDescPool;
        dsAI.descriptorSetCount = 2;
        dsAI.pSetLayouts = layouts;
        vkAllocateDescriptorSets(device, &dsAI, fsr2_.sharpenDescSets);
        // Descriptors updated dynamically each frame to point at the correct history buffer
    }

    fsr2_.needsHistoryReset = true;
    fsr2_.frameIndex = 0;
    LOG_INFO("FSR2: initialized successfully");
    return true;
}

void PostProcessPipeline::destroyFSR2Resources() {
    if (!vkCtx_) return;

    VkDevice device = vkCtx_->getDevice();
    VmaAllocator alloc = vkCtx_->getAllocator();
    vkDeviceWaitIdle(device);

#if WOWEE_HAS_AMD_FSR2
    if (fsr2_.useAmdBackend) {
        ffxFsr2ContextDestroy(&fsr2_.amdContext);
        fsr2_.useAmdBackend = false;
    }
    if (fsr2_.amdScratchBuffer) {
        std::free(fsr2_.amdScratchBuffer);
        fsr2_.amdScratchBuffer = nullptr;
    }
    fsr2_.amdScratchBufferSize = 0;
#endif
    fsr2_.amdFsr3FramegenRuntimeActive = false;
    fsr2_.amdFsr3FramegenRuntimeReady = false;
    fsr2_.framegenOutputValid = false;
    fsr2_.amdFsr3RuntimePath = "Path C";
    fsr2_.amdFsr3RuntimeLastError.clear();
#if WOWEE_HAS_AMD_FSR3_FRAMEGEN
    if (fsr2_.amdFsr3Runtime) {
        fsr2_.amdFsr3Runtime->shutdown();
        fsr2_.amdFsr3Runtime.reset();
    }
#endif

    destroy(device, fsr2_.sharpenPipeline);
    destroy(device, fsr2_.sharpenPipelineLayout);
    if (fsr2_.sharpenDescPool) { vkDestroyDescriptorPool(device, fsr2_.sharpenDescPool, nullptr); fsr2_.sharpenDescPool = VK_NULL_HANDLE; fsr2_.sharpenDescSets[0] = fsr2_.sharpenDescSets[1] = VK_NULL_HANDLE; }
    destroy(device, fsr2_.sharpenDescSetLayout);

    destroy(device, fsr2_.accumulatePipeline);
    destroy(device, fsr2_.accumulatePipelineLayout);
    if (fsr2_.accumulateDescPool) { vkDestroyDescriptorPool(device, fsr2_.accumulateDescPool, nullptr); fsr2_.accumulateDescPool = VK_NULL_HANDLE; fsr2_.accumulateDescSets[0] = fsr2_.accumulateDescSets[1] = VK_NULL_HANDLE; }
    destroy(device, fsr2_.accumulateDescSetLayout);

    destroy(device, fsr2_.motionVecPipeline);
    destroy(device, fsr2_.motionVecPipelineLayout);
    if (fsr2_.motionVecDescPool) { vkDestroyDescriptorPool(device, fsr2_.motionVecDescPool, nullptr); fsr2_.motionVecDescPool = VK_NULL_HANDLE; fsr2_.motionVecDescSet = VK_NULL_HANDLE; }
    destroy(device, fsr2_.motionVecDescSetLayout);

    if (fsr2_.sceneFramebuffer) { vkDestroyFramebuffer(device, fsr2_.sceneFramebuffer, nullptr); fsr2_.sceneFramebuffer = VK_NULL_HANDLE; }
    fsr2_.linearSampler = VK_NULL_HANDLE;  // Owned by VkContext sampler cache
    fsr2_.nearestSampler = VK_NULL_HANDLE; // Owned by VkContext sampler cache

    destroyImage(device, alloc, fsr2_.motionVectors);
    for (auto& img : fsr2_.history) destroyImage(device, alloc, img);
    destroyImage(device, alloc, fsr2_.framegenOutput);
    destroyImage(device, alloc, fsr2_.sceneDepth);
    destroyImage(device, alloc, fsr2_.sceneColor);

    fsr2_.internalWidth = 0;
    fsr2_.internalHeight = 0;
}

void PostProcessPipeline::dispatchMotionVectors() {
    if (!fsr2_.motionVecPipeline || currentCmd_ == VK_NULL_HANDLE) return;

    // Transition depth: DEPTH_STENCIL_ATTACHMENT → DEPTH_STENCIL_READ_ONLY
    transitionImageLayout(currentCmd_, fsr2_.sceneDepth.image,
        VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
        VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL,
        VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);

    // Transition motion vectors: UNDEFINED → GENERAL
    transitionImageLayout(currentCmd_, fsr2_.motionVectors.image,
        VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL,
        VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);

    vkCmdBindPipeline(currentCmd_, VK_PIPELINE_BIND_POINT_COMPUTE, fsr2_.motionVecPipeline);
    vkCmdBindDescriptorSets(currentCmd_, VK_PIPELINE_BIND_POINT_COMPUTE,
        fsr2_.motionVecPipelineLayout, 0, 1, &fsr2_.motionVecDescSet, 0, nullptr);

    // Reprojection with jittered matrices:
    // reconstruct world position from current depth, then project into previous clip.
    struct {
        glm::mat4 prevViewProjection;
        glm::mat4 invCurrentViewProj;
    } pc;

    glm::mat4 currentVP = camera_->getViewProjectionMatrix();
    pc.prevViewProjection = fsr2_.prevViewProjection;
    pc.invCurrentViewProj = glm::inverse(currentVP);

    vkCmdPushConstants(currentCmd_, fsr2_.motionVecPipelineLayout,
        VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(pc), &pc);

    uint32_t gx = (fsr2_.internalWidth + 7) / 8;
    uint32_t gy = (fsr2_.internalHeight + 7) / 8;
    vkCmdDispatch(currentCmd_, gx, gy, 1);

    // Transition motion vectors: GENERAL → SHADER_READ_ONLY for accumulation
    transitionImageLayout(currentCmd_, fsr2_.motionVectors.image,
        VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
}
void PostProcessPipeline::dispatchAmdFsr2() {
    if (currentCmd_ == VK_NULL_HANDLE || !camera_) return;
#if WOWEE_HAS_AMD_FSR2
    if (!fsr2_.useAmdBackend) return;

    VkExtent2D swapExtent = vkCtx_->getSwapchainExtent();
    uint32_t outputIdx = fsr2_.currentHistory;

    transitionImageLayout(currentCmd_, fsr2_.sceneColor.image,
        VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
    transitionImageLayout(currentCmd_, fsr2_.motionVectors.image,
        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
    transitionImageLayout(currentCmd_, fsr2_.sceneDepth.image,
        VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL, VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
    transitionImageLayout(currentCmd_, fsr2_.history[outputIdx].image,
        fsr2_.needsHistoryReset ? VK_IMAGE_LAYOUT_UNDEFINED : VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
        VK_IMAGE_LAYOUT_GENERAL,
        VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);

    FfxFsr2DispatchDescription desc{};
    desc.commandList = ffxGetCommandListVK(currentCmd_);
    desc.color = ffxGetTextureResourceVK(&fsr2_.amdContext,
        fsr2_.sceneColor.image, fsr2_.sceneColor.imageView,
        fsr2_.internalWidth, fsr2_.internalHeight, vkCtx_->getSwapchainFormat(),
        L"FSR2_InputColor", FFX_RESOURCE_STATE_COMPUTE_READ);
    desc.depth = ffxGetTextureResourceVK(&fsr2_.amdContext,
        fsr2_.sceneDepth.image, fsr2_.sceneDepth.imageView,
        fsr2_.internalWidth, fsr2_.internalHeight, vkCtx_->getDepthFormat(),
        L"FSR2_InputDepth", FFX_RESOURCE_STATE_COMPUTE_READ);
    desc.motionVectors = ffxGetTextureResourceVK(&fsr2_.amdContext,
        fsr2_.motionVectors.image, fsr2_.motionVectors.imageView,
        fsr2_.internalWidth, fsr2_.internalHeight, VK_FORMAT_R16G16_SFLOAT,
        L"FSR2_InputMotionVectors", FFX_RESOURCE_STATE_COMPUTE_READ);
    desc.output = ffxGetTextureResourceVK(&fsr2_.amdContext,
        fsr2_.history[outputIdx].image, fsr2_.history[outputIdx].imageView,
        swapExtent.width, swapExtent.height, VK_FORMAT_R16G16B16A16_SFLOAT,
        L"FSR2_Output", FFX_RESOURCE_STATE_UNORDERED_ACCESS);

    // Camera jitter is stored as NDC projection offsets; convert to render-pixel offsets.
    // Do not apply jitterSign again here: it is already baked into camera jitter.
    glm::vec2 jitterNdc = camera_->getJitter();
    desc.jitterOffset.x = jitterNdc.x * 0.5f * static_cast<float>(fsr2_.internalWidth);
    desc.jitterOffset.y = jitterNdc.y * 0.5f * static_cast<float>(fsr2_.internalHeight);
    desc.motionVectorScale.x = static_cast<float>(fsr2_.internalWidth) * fsr2_.motionVecScaleX;
    desc.motionVectorScale.y = static_cast<float>(fsr2_.internalHeight) * fsr2_.motionVecScaleY;
    desc.renderSize.width = fsr2_.internalWidth;
    desc.renderSize.height = fsr2_.internalHeight;
    desc.enableSharpening = false;  // Keep existing RCAS post pass.
    desc.sharpness = 0.0f;
    desc.frameTimeDelta = glm::max(0.001f, lastDeltaTime_ * 1000.0f);
    desc.preExposure = 1.0f;
    desc.reset = fsr2_.needsHistoryReset;
    desc.cameraNear = camera_->getNearPlane();
    desc.cameraFar = camera_->getFarPlane();
    desc.cameraFovAngleVertical = glm::radians(camera_->getFovDegrees());
    desc.viewSpaceToMetersFactor = 1.0f;
    desc.enableAutoReactive = false;

    FfxErrorCode dispatchErr = ffxFsr2ContextDispatch(&fsr2_.amdContext, &desc);
    if (dispatchErr != FFX_OK) {
        LOG_WARNING("FSR2 AMD: dispatch failed (", static_cast<int>(dispatchErr), "), forcing history reset.");
        fsr2_.needsHistoryReset = true;
    } else {
        fsr2_.needsHistoryReset = false;
    }
#endif
}

void PostProcessPipeline::dispatchAmdFsr3Framegen() {
#if WOWEE_HAS_AMD_FSR3_FRAMEGEN
    if (!fsr2_.amdFsr3FramegenEnabled) {
        fsr2_.amdFsr3FramegenRuntimeActive = false;
        return;
    }
    if (!fsr2_.amdFsr3Runtime || !fsr2_.amdFsr3FramegenRuntimeReady) {
        fsr2_.amdFsr3FramegenRuntimeActive = false;
        return;
    }
    uint32_t outputIdx = fsr2_.currentHistory;
    transitionImageLayout(currentCmd_, fsr2_.sceneColor.image,
        VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
    transitionImageLayout(currentCmd_, fsr2_.motionVectors.image,
        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
    transitionImageLayout(currentCmd_, fsr2_.sceneDepth.image,
        VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL, VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
    transitionImageLayout(currentCmd_, fsr2_.history[outputIdx].image,
        fsr2_.needsHistoryReset ? VK_IMAGE_LAYOUT_UNDEFINED : VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
        VK_IMAGE_LAYOUT_GENERAL,
        VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
    if (fsr2_.amdFsr3FramegenEnabled && fsr2_.framegenOutput.image) {
        transitionImageLayout(currentCmd_, fsr2_.framegenOutput.image,
            fsr2_.framegenOutputValid ? VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL : VK_IMAGE_LAYOUT_UNDEFINED,
            VK_IMAGE_LAYOUT_GENERAL,
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
    }

    AmdFsr3RuntimeDispatchDesc fgDispatch{};
    fgDispatch.commandBuffer = currentCmd_;
    fgDispatch.colorImage = fsr2_.sceneColor.image;
    fgDispatch.depthImage = fsr2_.sceneDepth.image;
    fgDispatch.motionVectorImage = fsr2_.motionVectors.image;
    fgDispatch.outputImage = fsr2_.history[fsr2_.currentHistory].image;
    fgDispatch.renderWidth = fsr2_.internalWidth;
    fgDispatch.renderHeight = fsr2_.internalHeight;
    fgDispatch.outputWidth = vkCtx_->getSwapchainExtent().width;
    fgDispatch.outputHeight = vkCtx_->getSwapchainExtent().height;
    fgDispatch.colorFormat = VK_FORMAT_R16G16B16A16_SFLOAT;
    fgDispatch.depthFormat = vkCtx_->getDepthFormat();
    fgDispatch.motionVectorFormat = VK_FORMAT_R16G16_SFLOAT;
    fgDispatch.outputFormat = VK_FORMAT_R16G16B16A16_SFLOAT;
    fgDispatch.frameGenOutputImage = fsr2_.framegenOutput.image;
    glm::vec2 jitterNdc = camera_ ? camera_->getJitter() : glm::vec2(0.0f);
    fgDispatch.jitterX = jitterNdc.x * 0.5f * static_cast<float>(fsr2_.internalWidth);
    fgDispatch.jitterY = jitterNdc.y * 0.5f * static_cast<float>(fsr2_.internalHeight);
    fgDispatch.motionScaleX = static_cast<float>(fsr2_.internalWidth) * fsr2_.motionVecScaleX;
    fgDispatch.motionScaleY = static_cast<float>(fsr2_.internalHeight) * fsr2_.motionVecScaleY;
    fgDispatch.frameTimeDeltaMs = glm::max(0.001f, lastDeltaTime_ * 1000.0f);
    fgDispatch.cameraNear = camera_ ? camera_->getNearPlane() : 0.1f;
    fgDispatch.cameraFar = camera_ ? camera_->getFarPlane() : 1000.0f;
    fgDispatch.cameraFovYRadians = camera_ ? glm::radians(camera_->getFovDegrees()) : 1.0f;
    fgDispatch.reset = fsr2_.needsHistoryReset;


    if (!fsr2_.amdFsr3Runtime->dispatchUpscale(fgDispatch)) {
        static bool warnedRuntimeDispatch = false;
        if (!warnedRuntimeDispatch) {
            warnedRuntimeDispatch = true;
            LOG_WARNING("FSR3 runtime upscale dispatch failed; falling back to FSR2 dispatch output.");
        }
        fsr2_.amdFsr3RuntimeLastError = fsr2_.amdFsr3Runtime->lastError();
        fsr2_.amdFsr3FallbackCount++;
        fsr2_.amdFsr3FramegenRuntimeActive = false;
        return;
    }
    fsr2_.amdFsr3RuntimeLastError.clear();
    fsr2_.amdFsr3UpscaleDispatchCount++;

    if (!fsr2_.amdFsr3FramegenEnabled) {
        fsr2_.amdFsr3FramegenRuntimeActive = false;
        return;
    }
    if (!fsr2_.amdFsr3Runtime->isFrameGenerationReady()) {
        fsr2_.amdFsr3FramegenRuntimeActive = false;
        return;
    }
    if (!fsr2_.amdFsr3Runtime->dispatchFrameGeneration(fgDispatch)) {
        static bool warnedFgDispatch = false;
        if (!warnedFgDispatch) {
            warnedFgDispatch = true;
            LOG_WARNING("FSR3 runtime frame generation dispatch failed; using upscaled output only.");
        }
        fsr2_.amdFsr3RuntimeLastError = fsr2_.amdFsr3Runtime->lastError();
        fsr2_.amdFsr3FallbackCount++;
        fsr2_.amdFsr3FramegenRuntimeActive = false;
        return;
    }
    fsr2_.amdFsr3RuntimeLastError.clear();
    fsr2_.amdFsr3FramegenDispatchCount++;
    fsr2_.framegenOutputValid = true;
    fsr2_.amdFsr3FramegenRuntimeActive = true;
#else
    fsr2_.amdFsr3FramegenRuntimeActive = false;
#endif
}

void PostProcessPipeline::renderFSR2Sharpen() {
    if (!fsr2_.sharpenPipeline || currentCmd_ == VK_NULL_HANDLE) return;

    VkExtent2D ext = vkCtx_->getSwapchainExtent();
    uint32_t outputIdx = fsr2_.currentHistory;

    // Use per-frame descriptor set to avoid race with in-flight command buffers
    uint32_t frameIdx = vkCtx_->getCurrentFrame();
    VkDescriptorSet descSet = fsr2_.sharpenDescSets[frameIdx];

    // Update sharpen descriptor to point at current history output
    VkDescriptorImageInfo imgInfo{};
    imgInfo.sampler = fsr2_.linearSampler;
    if (fsr2_.useAmdBackend) {
        imgInfo.imageView = (fsr2_.amdFsr3FramegenEnabled && fsr2_.amdFsr3FramegenRuntimeActive && fsr2_.framegenOutput.imageView)
            ? fsr2_.framegenOutput.imageView
            : fsr2_.history[outputIdx].imageView;
    } else {
        imgInfo.imageView = fsr2_.sceneColor.imageView;
    }
    imgInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkWriteDescriptorSet write{.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    write.dstSet = descSet;
    write.dstBinding = 0;
    write.descriptorCount = 1;
    write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    write.pImageInfo = &imgInfo;
    vkUpdateDescriptorSets(vkCtx_->getDevice(), 1, &write, 0, nullptr);

    vkCmdBindPipeline(currentCmd_, VK_PIPELINE_BIND_POINT_GRAPHICS, fsr2_.sharpenPipeline);
    vkCmdBindDescriptorSets(currentCmd_, VK_PIPELINE_BIND_POINT_GRAPHICS,
        fsr2_.sharpenPipelineLayout, 0, 1, &descSet, 0, nullptr);

    glm::vec4 params(1.0f / ext.width, 1.0f / ext.height, fsr2_.sharpness, 0.0f);
    vkCmdPushConstants(currentCmd_, fsr2_.sharpenPipelineLayout,
        VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(glm::vec4), &params);

    vkCmdDraw(currentCmd_, 3, 1, 0, 0);
}

// ========================= FXAA Post-Process =========================

bool PostProcessPipeline::initFXAAResources() {
    if (!vkCtx_) return false;

    VkDevice device = vkCtx_->getDevice();
    VmaAllocator alloc = vkCtx_->getAllocator();
    VkExtent2D ext = vkCtx_->getSwapchainExtent();
    VkSampleCountFlagBits msaa = vkCtx_->getMsaaSamples();
    bool useMsaa = (msaa > VK_SAMPLE_COUNT_1_BIT);
    bool useDepthResolve = (vkCtx_->getDepthResolveImageView() != VK_NULL_HANDLE);

    LOG_INFO("FXAA: initializing at ", ext.width, "x", ext.height,
             " (MSAA=", static_cast<int>(msaa), "x)");

    VkFormat colorFmt = vkCtx_->getSwapchainFormat();
    VkFormat depthFmt = vkCtx_->getDepthFormat();

    // sceneColor: 1x resolved color target - FXAA reads from here
    fxaa_.sceneColor = createImage(device, alloc, ext.width, ext.height,
        colorFmt, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT
                | VK_IMAGE_USAGE_TRANSFER_SRC_BIT);
    if (!fxaa_.sceneColor.image) {
        LOG_ERROR("FXAA: failed to create scene color image");
        return false;
    }

    // sceneDepth: depth buffer at current MSAA sample count
    fxaa_.sceneDepth = createImage(device, alloc, ext.width, ext.height,
        depthFmt, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT
                | VK_IMAGE_USAGE_TRANSFER_SRC_BIT, msaa);
    if (!fxaa_.sceneDepth.image) {
        LOG_ERROR("FXAA: failed to create scene depth image");
        destroyFXAAResources();
        return false;
    }

    if (useMsaa) {
        fxaa_.sceneMsaaColor = createImage(device, alloc, ext.width, ext.height,
            colorFmt, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT, msaa);
        if (!fxaa_.sceneMsaaColor.image) {
            LOG_ERROR("FXAA: failed to create MSAA color image");
            destroyFXAAResources();
            return false;
        }
        if (useDepthResolve) {
            fxaa_.sceneDepthResolve = createImage(device, alloc, ext.width, ext.height,
                depthFmt, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT
                        | VK_IMAGE_USAGE_TRANSFER_SRC_BIT);
            if (!fxaa_.sceneDepthResolve.image) {
                LOG_ERROR("FXAA: failed to create depth resolve image");
                destroyFXAAResources();
                return false;
            }
        }
    }

    // Framebuffer - same attachment layout as main render pass
    VkImageView fbAttachments[4]{};
    uint32_t fbCount;
    if (useMsaa) {
        fbAttachments[0] = fxaa_.sceneMsaaColor.imageView;
        fbAttachments[1] = fxaa_.sceneDepth.imageView;
        fbAttachments[2] = fxaa_.sceneColor.imageView;  // resolve target
        fbCount = 3;
        if (useDepthResolve) {
            fbAttachments[3] = fxaa_.sceneDepthResolve.imageView;
            fbCount = 4;
        }
    } else {
        fbAttachments[0] = fxaa_.sceneColor.imageView;
        fbAttachments[1] = fxaa_.sceneDepth.imageView;
        fbCount = 2;
    }

    VkFramebufferCreateInfo fbInfo{};
    fbInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
    fbInfo.renderPass = vkCtx_->getImGuiRenderPass();
    fbInfo.attachmentCount = fbCount;
    fbInfo.pAttachments = fbAttachments;
    fbInfo.width = ext.width;
    fbInfo.height = ext.height;
    fbInfo.layers = 1;
    if (vkCreateFramebuffer(device, &fbInfo, nullptr, &fxaa_.sceneFramebuffer) != VK_SUCCESS) {
        LOG_ERROR("FXAA: failed to create scene framebuffer");
        destroyFXAAResources();
        return false;
    }

    // Sampler
    VkSamplerCreateInfo samplerInfo{};
    samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    samplerInfo.minFilter = VK_FILTER_LINEAR;
    samplerInfo.magFilter = VK_FILTER_LINEAR;
    samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    fxaa_.sceneSampler = vkCtx_->getOrCreateSampler(samplerInfo);
    if (fxaa_.sceneSampler == VK_NULL_HANDLE) {
        LOG_ERROR("FXAA: failed to create sampler");
        destroyFXAAResources();
        return false;
    }

    // Descriptor set layout: binding 0 = combined image sampler
    VkDescriptorSetLayoutBinding binding{};
    binding.binding = 0;
    binding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    binding.descriptorCount = 1;
    binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    VkDescriptorSetLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutInfo.bindingCount = 1;
    layoutInfo.pBindings = &binding;
    vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr, &fxaa_.descSetLayout);

    constexpr uint32_t setCount = FXAAState::DESC_SET_COUNT;
    VkDescriptorPoolSize poolSize{};
    poolSize.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    poolSize.descriptorCount = setCount;
    VkDescriptorPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.maxSets = setCount;
    poolInfo.poolSizeCount = 1;
    poolInfo.pPoolSizes = &poolSize;
    vkCreateDescriptorPool(device, &poolInfo, nullptr, &fxaa_.descPool);

    VkDescriptorSetLayout layouts[setCount];
    for (auto& layout : layouts) layout = fxaa_.descSetLayout;
    VkDescriptorSetAllocateInfo dsAllocInfo{};
    dsAllocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    dsAllocInfo.descriptorPool = fxaa_.descPool;
    dsAllocInfo.descriptorSetCount = setCount;
    dsAllocInfo.pSetLayouts = layouts;
    vkAllocateDescriptorSets(device, &dsAllocInfo, fxaa_.descSet);

    // Bind the resolved 1x sceneColor to all per-frame sets
    VkDescriptorImageInfo imgInfo{};
    imgInfo.sampler = fxaa_.sceneSampler;
    imgInfo.imageView = fxaa_.sceneColor.imageView;
    imgInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    for (auto& descSet : fxaa_.descSet) {
        VkWriteDescriptorSet write{};
        write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        write.dstSet = descSet;
        write.dstBinding = 0;
        write.descriptorCount = 1;
        write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        write.pImageInfo = &imgInfo;
        vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);
    }

    // Pipeline layout - push constant holds vec4(rcpFrame.xy, sharpness, pad)
    VkPushConstantRange pc{};
    pc.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    pc.offset = 0;
    pc.size = 16;  // vec4
    VkPipelineLayoutCreateInfo plCI{};
    plCI.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    plCI.setLayoutCount = 1;
    plCI.pSetLayouts = &fxaa_.descSetLayout;
    plCI.pushConstantRangeCount = 1;
    plCI.pPushConstantRanges = &pc;
    vkCreatePipelineLayout(device, &plCI, nullptr, &fxaa_.pipelineLayout);

    // FXAA pipeline - fullscreen triangle into the swapchain render pass
    // Uses VK_SAMPLE_COUNT_1_BIT: it always runs after MSAA resolve.
    VkShaderModule vertMod, fragMod;
    if (!vertMod.loadFromFile(device, "assets/shaders/postprocess.vert.spv") ||
        !fragMod.loadFromFile(device, "assets/shaders/fxaa.frag.spv")) {
        LOG_ERROR("FXAA: failed to load shaders");
        destroyFXAAResources();
        return false;
    }

    fxaa_.pipeline = PipelineBuilder()
        .setShaders(vertMod.stageInfo(VK_SHADER_STAGE_VERTEX_BIT),
                    fragMod.stageInfo(VK_SHADER_STAGE_FRAGMENT_BIT))
        .setVertexInput({}, {})
        .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
        .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
        .setNoDepthTest()
        .setColorBlendAttachment(PipelineBuilder::blendDisabled())
        .setMultisample(VK_SAMPLE_COUNT_1_BIT)  // the overlay pass is always 1x
        .setLayout(fxaa_.pipelineLayout)
        .setRenderPass(vkCtx_->getOverlayRenderPass())
        .setDynamicStates(viewportAndScissorDynamic())
        .build(device, vkCtx_->getPipelineCache());

    vertMod.destroy();
    fragMod.destroy();

    if (!fxaa_.pipeline) {
        LOG_ERROR("FXAA: failed to create pipeline");
        destroyFXAAResources();
        return false;
    }

    LOG_INFO("FXAA: initialized successfully");
    return true;
}

void PostProcessPipeline::destroyFXAAResources() {
    if (!vkCtx_) return;
    VkDevice device = vkCtx_->getDevice();
    VmaAllocator alloc = vkCtx_->getAllocator();
    vkDeviceWaitIdle(device);

    destroy(device, fxaa_.pipeline);
    destroy(device, fxaa_.pipelineLayout);
    if (fxaa_.descPool)       { vkDestroyDescriptorPool(device, fxaa_.descPool, nullptr);       fxaa_.descPool = VK_NULL_HANDLE; for (auto& s : fxaa_.descSet) s = VK_NULL_HANDLE; }
    destroy(device, fxaa_.descSetLayout);
    if (fxaa_.sceneFramebuffer) { vkDestroyFramebuffer(device, fxaa_.sceneFramebuffer, nullptr); fxaa_.sceneFramebuffer = VK_NULL_HANDLE; }
    fxaa_.sceneSampler = VK_NULL_HANDLE; // Owned by VkContext sampler cache
    destroyImage(device, alloc, fxaa_.sceneDepthResolve);
    destroyImage(device, alloc, fxaa_.sceneMsaaColor);
    destroyImage(device, alloc, fxaa_.sceneDepth);
    destroyImage(device, alloc, fxaa_.sceneColor);
}

void PostProcessPipeline::renderFXAAPass() {
    if (!fxaa_.pipeline || currentCmd_ == VK_NULL_HANDLE) return;
    VkExtent2D ext = vkCtx_->getSwapchainExtent();

    uint32_t fi = vkCtx_->getCurrentFrame();
    vkCmdBindPipeline(currentCmd_, VK_PIPELINE_BIND_POINT_GRAPHICS, fxaa_.pipeline);
    vkCmdBindDescriptorSets(currentCmd_, VK_PIPELINE_BIND_POINT_GRAPHICS,
                            fxaa_.pipelineLayout, 0, 1, &fxaa_.descSet[fi], 0, nullptr);

    // Pass rcpFrame + sharpness + effect flag (vec4, 16 bytes).
    // When FSR2/FSR3 is active alongside FXAA, forward FSR2's sharpness so the
    // post-FXAA unsharp-mask step restores the crispness that FXAA's blur removes.
    float sharpness = fsr2_.enabled ? fsr2_.sharpness : 0.0f;
    float pc[4] = {
        1.0f / static_cast<float>(ext.width),
        1.0f / static_cast<float>(ext.height),
        sharpness,
        intoxication_
    };
    vkCmdPushConstants(currentCmd_, fxaa_.pipelineLayout,
                       VK_SHADER_STAGE_FRAGMENT_BIT, 0, 16, pc);

    vkCmdDraw(currentCmd_, 3, 1, 0, 0);  // fullscreen triangle
}

} // namespace rendering
} // namespace wowee
