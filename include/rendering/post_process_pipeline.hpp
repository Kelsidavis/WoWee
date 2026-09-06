#pragma once

#include <memory>
#include <string>
#include <cstdint>
#include <glm/glm.hpp>
#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>
#include "rendering/vk_utils.hpp"
#if WOWEE_HAS_AMD_FSR2
#include "ffx_fsr2.h"
#include "ffx_fsr2_vk.h"
#endif

namespace wowee {
namespace rendering {

class VkContext;
class Camera;
class AmdFsr3Runtime;

/// Returned by setFSREnabled/setFSR2Enabled when they need the Renderer
/// to schedule an MSAA sample-count change (§4.3).
struct MsaaChangeRequest {
    bool requested = false;
    VkSampleCountFlagBits samples = VK_SAMPLE_COUNT_1_BIT;
};

/// PostProcessPipeline owns all FSR 1.0, FXAA, and FSR 2.2/3 state and
/// orchestrates post-processing passes between the scene render pass and
/// the final swapchain presentation (§4.3 extraction from Renderer).
class PostProcessPipeline {
public:
    PostProcessPipeline();
    ~PostProcessPipeline();

    void initialize(VkContext* ctx);
    void shutdown();

    // --- Frame-loop integration (called from Renderer::beginFrame) ---

    /// Lazy-create / lazy-destroy FSR/FXAA/FSR2 resources between frames.
    void manageResources();

    /// Recreate post-process resources after swapchain resize.
    void handleSwapchainResize();

    /// Apply FSR2 temporal jitter to the camera projection.
    void applyJitter(Camera* camera);

    /// Returns the framebuffer the scene should render into.
    /// If no post-processing is active, returns VK_NULL_HANDLE (use swapchain).
    [[nodiscard]] VkFramebuffer getSceneFramebuffer() const;

    /// Returns the render extent for the active post-process pipeline.
    /// Falls back to swapchain extent if nothing is active.
    [[nodiscard]] VkExtent2D getSceneRenderExtent() const;
    // The images behind getSceneFramebuffer(), so the scene can be copied for
    // water refraction after it is drawn but before the water goes over it.
    [[nodiscard]] VkImage getSceneColorImage() const;
    [[nodiscard]] VkImage getSceneDepthImage() const;
    [[nodiscard]] bool sceneDepthIsMsaa() const;


    /// True when FXAA alone (no FSR2) needs its own off-screen pass.
    [[nodiscard]] bool useFXAAPostPass() const { return fxaa_.enabled; }

    // --- Frame-loop integration (called from Renderer::endFrame) ---

    /// Execute all post-processing passes.  Returns true if an INLINE
    /// render pass was started (affects ImGui recording mode).
    bool executePostProcessing(VkCommandBuffer cmd, uint32_t imageIndex,
                               Camera* camera, float deltaTime);

    // --- MSAA interop (called from Renderer::applyMsaaChange) ---

    /// Destroy FSR/FSR2/FXAA resources (they will be lazily recreated).
    void destroyAllResources();

    /// True when FSR2 is active and MSAA changes should be blocked.
    /// True only where the temporal upscaler cannot share the scene with
    /// multisampling: it needs a single-sampled depth to read, which with MSAA
    /// on is the render pass's depth resolve, and a device without depth
    /// resolve has no such image. Everywhere else the two run together.
    [[nodiscard]] bool isFsr2BlockingMsaa() const;

    // --- Public API (delegated from Renderer) ---

    // FXAA
    void setFXAAEnabled(bool enabled);
    [[nodiscard]] bool isFXAAEnabled() const { return fxaa_.enabled; }

    // FSR 1.0
    MsaaChangeRequest setFSREnabled(bool enabled);
    [[nodiscard]] bool isFSREnabled() const { return fsr_.enabled; }
    void setFSRQuality(float scaleFactor);
    void setFSRSharpness(float sharpness);
    [[nodiscard]] float getFSRScaleFactor() const { return fsr_.scaleFactor; }
    [[nodiscard]] float getFSRSharpness() const { return fsr_.sharpness; }

    // FSR 2.2
    MsaaChangeRequest setFSR2Enabled(bool enabled, Camera* camera);
    [[nodiscard]] bool isFSR2Enabled() const { return fsr2_.enabled; }
    void setFSR2DebugTuning(float jitterSign, float motionVecScaleX, float motionVecScaleY);

    // FSR3 Framegen
    void setAmdFsr3FramegenEnabled(bool enabled);
    [[nodiscard]] bool isAmdFsr3FramegenEnabled() const { return fsr2_.amdFsr3FramegenEnabled; }
    [[nodiscard]] float getFSR2JitterSign() const { return fsr2_.jitterSign; }
    [[nodiscard]] float getFSR2MotionVecScaleX() const { return fsr2_.motionVecScaleX; }
    [[nodiscard]] float getFSR2MotionVecScaleY() const { return fsr2_.motionVecScaleY; }
#if WOWEE_HAS_AMD_FSR2
    bool isAmdFsr2SdkAvailable() const { return true; }
#else
    [[nodiscard]] bool isAmdFsr2SdkAvailable() const { return false; }
#endif
#if WOWEE_HAS_AMD_FSR3_FRAMEGEN
    bool isAmdFsr3FramegenSdkAvailable() const { return true; }
#else
    [[nodiscard]] bool isAmdFsr3FramegenSdkAvailable() const { return false; }
#endif
    [[nodiscard]] bool isAmdFsr3FramegenRuntimeActive() const { return fsr2_.amdFsr3FramegenRuntimeActive; }
    [[nodiscard]] bool isAmdFsr3FramegenRuntimeReady() const { return fsr2_.amdFsr3FramegenRuntimeReady; }
    [[nodiscard]] const char* getAmdFsr3FramegenRuntimePath() const;
    [[nodiscard]] const std::string& getAmdFsr3FramegenRuntimeError() const { return fsr2_.amdFsr3RuntimeLastError; }
    [[nodiscard]] size_t getAmdFsr3UpscaleDispatchCount() const { return fsr2_.amdFsr3UpscaleDispatchCount; }
    [[nodiscard]] size_t getAmdFsr3FramegenDispatchCount() const { return fsr2_.amdFsr3FramegenDispatchCount; }
    [[nodiscard]] size_t getAmdFsr3FallbackCount() const { return fsr2_.amdFsr3FallbackCount; }

    // Brightness (1.0 = default, <1 darkens, >1 brightens)
    void setBrightness(float b) { brightness_ = b; }
    [[nodiscard]] float getBrightness() const { return brightness_; }
    void setIntoxication(float amount) { intoxication_ = glm::clamp(amount, 0.0f, 1.0f); }
    [[nodiscard]] float getIntoxication() const { return intoxication_; }

private:
    VkContext* vkCtx_ = nullptr;

    // Per-frame state set during executePostProcessing
    VkCommandBuffer currentCmd_ = VK_NULL_HANDLE;
    Camera* camera_ = nullptr;
    float lastDeltaTime_ = 0.0f;

    // Brightness
    float brightness_ = 1.0f;
    float intoxication_ = 0.0f;

    [[nodiscard]] bool needsFXAAPass() const { return fxaa_.enabled || intoxication_ > 0.001f; }

    // FSR 1.0 upscaling state
    struct FSRState {
        bool enabled = false;
        bool needsRecreate = false;
        float scaleFactor = 1.00f;  // Native default
        float sharpness = 1.6f;
        uint32_t internalWidth = 0;
        uint32_t internalHeight = 0;

        // Off-screen scene target (reduced resolution)
        AllocatedImage sceneColor{};        // 1x color (non-MSAA render target / MSAA resolve target)
        AllocatedImage sceneDepth{};        // Depth (matches current MSAA sample count)
        AllocatedImage sceneMsaaColor{};    // MSAA color target (only when MSAA > 1x)
        AllocatedImage sceneDepthResolve{}; // Depth resolve (only when MSAA + depth resolve)
        VkFramebuffer sceneFramebuffer = VK_NULL_HANDLE;
        VkSampler sceneSampler = VK_NULL_HANDLE;

        // Upscale pipeline
        VkPipeline pipeline = VK_NULL_HANDLE;
        VkPipelineLayout pipelineLayout = VK_NULL_HANDLE;
        VkDescriptorSetLayout descSetLayout = VK_NULL_HANDLE;
        VkDescriptorPool descPool = VK_NULL_HANDLE;
        VkDescriptorSet descSet = VK_NULL_HANDLE;
    };
    FSRState fsr_;
    bool initFSRResources();
    void destroyFSRResources();
    void renderFSRUpscale();

    // FXAA post-process state
    struct FXAAState {
        bool enabled       = false;
        bool needsRecreate = false;

        // Off-screen scene target (same resolution as swapchain - no scaling)
        AllocatedImage sceneColor{};        // 1x resolved color target
        AllocatedImage sceneDepth{};        // Depth (matches MSAA sample count)
        AllocatedImage sceneMsaaColor{};    // MSAA color target (when MSAA > 1x)
        AllocatedImage sceneDepthResolve{}; // Depth resolve (MSAA + depth resolve)
        VkFramebuffer sceneFramebuffer = VK_NULL_HANDLE;
        VkSampler sceneSampler         = VK_NULL_HANDLE;

        // FXAA fullscreen pipeline
        VkPipeline           pipeline          = VK_NULL_HANDLE;
        VkPipelineLayout     pipelineLayout    = VK_NULL_HANDLE;
        VkDescriptorSetLayout descSetLayout    = VK_NULL_HANDLE;
        VkDescriptorPool     descPool          = VK_NULL_HANDLE;
        // Per-frame descriptor sets to avoid race with in-flight command buffers
        static constexpr uint32_t DESC_SET_COUNT = 2; // matches MAX_FRAMES_IN_FLIGHT
        VkDescriptorSet      descSet[DESC_SET_COUNT] = {};
    };
    FXAAState fxaa_;
    bool initFXAAResources();
    void destroyFXAAResources();
    void renderFXAAPass();

    // FSR 2.2 temporal upscaling state
    struct FSR2State {
        bool enabled = false;
        bool needsRecreate = false;
        float scaleFactor = 0.77f;
        float sharpness = 3.0f;  // Very strong RCAS to counteract upscale softness
        uint32_t internalWidth = 0;
        uint32_t internalHeight = 0;

        // Off-screen scene targets at the internal resolution, laid out like
        // FSR 1's: with MSAA on, the scene draws into the multisampled pair
        // and the render pass resolves both into the single-sampled pair,
        // which is what the upscaler reads. It used to insist on 1x and
        // switch multisampling off behind the player's back to get it.
        AllocatedImage sceneColor{};        // 1x: the render target, or the resolve target under MSAA
        AllocatedImage sceneDepth{};        // at the scene's sample count
        AllocatedImage sceneMsaaColor{};    // MSAA colour target, only while MSAA is on
        AllocatedImage sceneDepthResolve{}; // 1x depth, only while MSAA is on
        VkFramebuffer sceneFramebuffer = VK_NULL_HANDLE;

        // Samplers
        VkSampler linearSampler = VK_NULL_HANDLE;   // For color
        VkSampler nearestSampler = VK_NULL_HANDLE;  // For depth / motion vectors

        // Motion vector buffer (internal resolution)
        AllocatedImage motionVectors{};

        // History buffers (display resolution, ping-pong)
        AllocatedImage history[2]{};
        AllocatedImage framegenOutput{};
        bool framegenOutputValid = false;
        uint32_t currentHistory = 0;  // Output index (0 or 1)

        // Compute pipelines
        VkPipeline motionVecPipeline = VK_NULL_HANDLE;
        VkPipelineLayout motionVecPipelineLayout = VK_NULL_HANDLE;
        VkDescriptorSetLayout motionVecDescSetLayout = VK_NULL_HANDLE;
        VkDescriptorPool motionVecDescPool = VK_NULL_HANDLE;
        VkDescriptorSet motionVecDescSet = VK_NULL_HANDLE;

        VkPipeline accumulatePipeline = VK_NULL_HANDLE;
        VkPipelineLayout accumulatePipelineLayout = VK_NULL_HANDLE;
        VkDescriptorSetLayout accumulateDescSetLayout = VK_NULL_HANDLE;
        VkDescriptorPool accumulateDescPool = VK_NULL_HANDLE;
        VkDescriptorSet accumulateDescSets[2] = {};  // Per ping-pong

        // RCAS sharpening pass (display resolution)
        VkPipeline sharpenPipeline = VK_NULL_HANDLE;
        VkPipelineLayout sharpenPipelineLayout = VK_NULL_HANDLE;
        VkDescriptorSetLayout sharpenDescSetLayout = VK_NULL_HANDLE;
        VkDescriptorPool sharpenDescPool = VK_NULL_HANDLE;
        VkDescriptorSet sharpenDescSets[2] = {};

        // Previous frame state for motion vector reprojection
        glm::mat4 prevViewProjection = glm::mat4(1.0f);
        glm::vec2 prevJitter = glm::vec2(0.0f);
        uint32_t frameIndex = 0;
        bool needsHistoryReset = true;
        bool useAmdBackend = false;
        bool amdFsr3FramegenEnabled = false;
        bool amdFsr3FramegenRuntimeActive = false;
        bool amdFsr3FramegenRuntimeReady = false;
        std::string amdFsr3RuntimePath = "Path C";
        std::string amdFsr3RuntimeLastError{};
        size_t amdFsr3UpscaleDispatchCount = 0;
        size_t amdFsr3FramegenDispatchCount = 0;
        size_t amdFsr3FallbackCount = 0;
        float jitterSign = 0.38f;
        float motionVecScaleX = 1.0f;
        float motionVecScaleY = 1.0f;
#if WOWEE_HAS_AMD_FSR2
        FfxFsr2Context amdContext{};
        FfxFsr2Interface amdInterface{};
        void* amdScratchBuffer = nullptr;
        size_t amdScratchBufferSize = 0;
#endif
        std::unique_ptr<AmdFsr3Runtime> amdFsr3Runtime;

        // Convergent accumulation: jitter for N frames then freeze
        int convergenceFrame = 0;
        glm::mat4 lastStableVP = glm::mat4(1.0f);
    };
    FSR2State fsr2_;
    bool initFSR2Resources();
    /// The single-sampled depth the upscaler's passes read: the resolve while
    /// MSAA is on, the scene depth itself otherwise.
    [[nodiscard]] const AllocatedImage& fsr2ReadDepth() const {
        return fsr2_.sceneDepthResolve.image != VK_NULL_HANDLE ? fsr2_.sceneDepthResolve : fsr2_.sceneDepth;
    }
    void destroyFSR2Resources();
    void dispatchMotionVectors();
    void dispatchAmdFsr2();
    void dispatchAmdFsr3Framegen();
    void renderFSR2Sharpen();
    static float halton(uint32_t index, uint32_t base);
};

} // namespace rendering
} // namespace wowee
