#pragma once

#include <algorithm>

#include "rendering/vk_utils.hpp"
#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>
#include <VkBootstrap.h>
#include <SDL2/SDL.h>
#include <vector>
#include <functional>
#include <cstdint>
#include <unordered_map>
#include <mutex>

namespace wowee {
namespace rendering {

static constexpr uint32_t MAX_FRAMES_IN_FLIGHT = 2;

struct FrameData {
    VkCommandPool commandPool = VK_NULL_HANDLE;
    VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
    /// Signalled by this slot's submit. Unused when the timeline is available.
    VkFence inFlightFence = VK_NULL_HANDLE;
    /// The timeline value this slot's last submit signals. Reaching it means
    /// the GPU is done with the slot. Zero is "never submitted", which the
    /// timeline starts at, so the first wait on each slot returns immediately
    /// -- the same reason the fences are created VK_FENCE_CREATE_SIGNALED_BIT.
    uint64_t timelineValue = 0;
};

class VkContext {
public:
    /// Put frame synchronisation back to the state it starts in.
    ///
    /// After the swapchain and every pipeline are rebuilt, the frame slots are
    /// left mid-cycle: a fence may be unsignalled with no submit coming, and
    /// the slot index points partway through the ring. The next frame then
    /// resets a fence and re-records a command buffer that the GPU has not
    /// finished with, which validation reports as VUID-vkResetFences-01123
    /// and VUID-vkBeginCommandBuffer-00049 and the driver answers by losing
    /// the device.
    ///
    /// Waits for the device, signals every fence, and starts again at slot
    /// zero. Only safe between frames, which is where the rebuild happens.
    void resetFrameSyncState();
    /// Says once when the shared immediate-submit fence is reached from more
    /// than one thread, which is unsafe and matches what validation reports.
    void noteImmediateSubmitThread(const char* who);

    /// Which incarnation of the UI textures is current. Anything holding a
    /// descriptor set from uploadImGuiTexture caches this alongside it and
    /// drops the cache when it moves, because those sets are this context's to
    /// free and are not valid across the free.
    ///
    /// Counted against the destruction rather than against an ImGui backend
    /// restart, which is what it used to key on: the restart was removed, its
    /// only caller went with it, and the check downstream quietly became dead
    /// code that never fired.
    [[nodiscard]] uint32_t uiTextureGeneration() const { return uiTextureGeneration_; }

    VkContext() = default;
    ~VkContext();

    VkContext(const VkContext&) = delete;
    VkContext& operator=(const VkContext&) = delete;

    [[nodiscard]] bool initialize(SDL_Window* window);
    void shutdown();

    // Swapchain management
    [[nodiscard]] bool recreateSwapchain(int width, int height);

    // Frame operations
    VkCommandBuffer beginFrame(uint32_t& imageIndex);
    void endFrame(VkCommandBuffer cmd, uint32_t imageIndex);

    // Single-time command buffer helpers
    VkCommandBuffer beginSingleTimeCommands();
    void endSingleTimeCommands(VkCommandBuffer cmd);

    // Immediate submit for one-off GPU work (descriptor pool creation, etc.)
    void immediateSubmit(std::function<void(VkCommandBuffer cmd)>&& function);

    // Batch upload mode: records multiple upload commands into a single
    // command buffer, then submits with ONE fence wait instead of one per upload.
    void beginUploadBatch();
    /// Opens the batch command buffer on first record, so a batch that
    /// nothing writes to never allocates one.
    void ensureBatchCmd();
    void endUploadBatch();       // Async: submits but does NOT wait for fence
    void endUploadBatchSync();   // Sync: submits and waits (for load screens)
    [[nodiscard]] bool isInUploadBatch() const { return inUploadBatch_; }
    /// Hands a plainly-allocated staging buffer to the current batch, which
    /// frees it once its copies have actually run.
    void deferRawStagingCleanup(VkBuffer buffer, VkDeviceMemory memory);
    void freeRawStaging();
    void deferStagingCleanup(AllocatedBuffer staging);
    void pollUploadBatches();    // Check completed async uploads, free staging buffers
    void waitAllUploads();       // Block until all in-flight uploads complete

    // Defer resource destruction until it is safe with multiple frames in flight.
    //
    // This queues work to run after the fence for the *current frame slot* has
    // signaled the next time we enter beginFrame() for that slot (i.e. after
    // MAX_FRAMES_IN_FLIGHT submissions). Use this for resources that may still
    // be referenced by command buffers submitted in the previous frame(s),
    // such as descriptor sets and buffers freed during streaming/unload.
    void deferAfterFrameFence(std::function<void()>&& fn);
    // Like deferAfterFrameFence, but waits until ALL in-flight frame slots have
    // been fenced - safe for shared resources bound by multiple frames' command
    // buffers (material descriptor sets, vertex/index buffers, etc.).
    void deferAfterAllFrameFences(std::function<void()>&& fn);

    // Accessors
    [[nodiscard]] VkInstance getInstance() const { return instance; }
    [[nodiscard]] VkPhysicalDevice getPhysicalDevice() const { return physicalDevice; }
    [[nodiscard]] VkDevice getDevice() const { return device; }
    [[nodiscard]] uint32_t getGpuVendorId() const { return gpuVendorId_; }
    [[nodiscard]] const char* getGpuName() const { return gpuName_; }
    [[nodiscard]] bool isAmdGpu() const { return gpuVendorId_ == 0x1002; }
    [[nodiscard]] bool isNvidiaGpu() const { return gpuVendorId_ == 0x10DE; }
    [[nodiscard]] VkQueue getGraphicsQueue() const { return graphicsQueue; }
    [[nodiscard]] uint32_t getGraphicsQueueFamily() const { return graphicsQueueFamily; }
    [[nodiscard]] bool hasDedicatedTransferQueue() const { return hasDedicatedTransfer_; }
    [[nodiscard]] VmaAllocator getAllocator() const { return allocator; }
    [[nodiscard]] VkSurfaceKHR getSurface() const { return surface; }
    [[nodiscard]] VkPipelineCache getPipelineCache() const { return pipelineCache_; }

    [[nodiscard]] VkSwapchainKHR getSwapchain() const { return swapchain; }
    [[nodiscard]] VkFormat getSwapchainFormat() const { return swapchainFormat; }
    [[nodiscard]] VkExtent2D getSwapchainExtent() const { return swapchainExtent; }
    [[nodiscard]] const std::vector<VkImageView>& getSwapchainImageViews() const { return swapchainImageViews; }
    [[nodiscard]] const std::vector<VkImage>& getSwapchainImages() const { return swapchainImages; }
    [[nodiscard]] uint32_t getSwapchainImageCount() const { return static_cast<uint32_t>(swapchainImages.size()); }

    [[nodiscard]] uint32_t getCurrentFrame() const { return currentFrame; }
    [[nodiscard]] const FrameData& getCurrentFrameData() const { return frames[currentFrame]; }

    // For ImGui
    [[nodiscard]] VkRenderPass getImGuiRenderPass() const { return imguiRenderPass; }
    // Single-sampled, colour-only pass that loads the swapchain. The UI draws
    // here, after the scene has resolved and after water refraction has copied
    // it, so the capture never contains the UI.
    [[nodiscard]] VkRenderPass getOverlayRenderPass() const { return overlayRenderPass; }
    // The same pass but clearing, for screens that draw the UI with no scene
    // behind it. Shares getOverlayFramebuffers().
    [[nodiscard]] VkRenderPass getOverlayClearRenderPass() const { return overlayClearRenderPass; }
    [[nodiscard]] const std::vector<VkFramebuffer>& getOverlayFramebuffers() const { return overlayFramebuffers; }
    // Compatible with getImGuiRenderPass(), but loads the scene instead of
    // clearing it, so drawing can continue into the same framebuffer after the
    // pass has been closed for a copy. Null under MSAA.
    [[nodiscard]] VkRenderPass getSceneContinueRenderPass() const { return sceneContinueRenderPass; }
    [[nodiscard]] VkDescriptorPool getImGuiDescriptorPool() const { return imguiDescriptorPool; }
    [[nodiscard]] const std::vector<VkFramebuffer>& getSwapchainFramebuffers() const { return swapchainFramebuffers; }

    [[nodiscard]] bool isSwapchainDirty() const { return swapchainDirty; }
    void markSwapchainDirty() { swapchainDirty = true; }

    // VSync (present mode)
    [[nodiscard]] bool isVsyncEnabled() const { return vsync_; }
    void setVsync(bool enabled) { vsync_ = enabled; }

    [[nodiscard]] bool isDeviceLost() const { return deviceLost_; }

    // MSAA
    [[nodiscard]] VkSampleCountFlagBits getMsaaSamples() const { return msaaSamples_; }
    void setMsaaSamples(VkSampleCountFlagBits samples);
    [[nodiscard]] VkSampleCountFlagBits getMaxUsableSampleCount() const;
    [[nodiscard]] VkImage getDepthImage() const { return depthImage; }
    [[nodiscard]] VkImage getDepthCopySourceImage() const {
        return (depthResolveImage != VK_NULL_HANDLE) ? depthResolveImage : depthImage;
    }
    [[nodiscard]] bool isDepthCopySourceMsaa() const {
        return (depthResolveImage == VK_NULL_HANDLE) && (msaaSamples_ > VK_SAMPLE_COUNT_1_BIT);
    }
    [[nodiscard]] VkFormat getDepthFormat() const { return depthFormat; }
    [[nodiscard]] VkImageView getDepthResolveImageView() const { return depthResolveImageView; }
    [[nodiscard]] VkImageView getDepthImageView() const { return depthImageView; }

    // Sampler cache: returns a shared VkSampler matching the given create info.
    // Callers must NOT destroy the returned sampler - it is owned by VkContext.
    // Automatically clamps anisotropy if the device doesn't support it.
    VkSampler getOrCreateSampler(const VkSamplerCreateInfo& info);

    // Whether the physical device supports sampler anisotropy.
    [[nodiscard]] bool isSamplerAnisotropySupported() const { return samplerAnisotropySupported_; }
    /// False on hardware without fillModeNonSolid, where a VK_POLYGON_MODE_LINE
    /// pipeline cannot be built and the wireframe views are unavailable.
    [[nodiscard]] bool isWireframeSupported() const { return fillModeNonSolidSupported_; }
    /// False on hardware missing shaderStorageImageWriteWithoutFormat or
    /// shaderInt16, which the FSR2 compute shaders both need.
    [[nodiscard]] bool areFsr2ComputeFeaturesSupported() const {
        return fsr2ComputeFeaturesSupported_;
    }

    /// Whether barriers can be recorded as VkDependencyInfo. False means the
    /// same barriers still record, through the legacy entry point.
    [[nodiscard]] bool isSynchronization2Supported() const { return synchronization2Supported_; }
    [[nodiscard]] PFN_vkCmdPipelineBarrier2KHR cmdPipelineBarrier2Fn() const { return cmdPipelineBarrier2_; }

    /// Whether a texture can be uploaded without a staging buffer.
    [[nodiscard]] bool isHostImageCopySupported() const { return hostImageCopySupported_; }
    [[nodiscard]] PFN_vkCopyMemoryToImageEXT copyMemoryToImageFn() const { return copyMemoryToImage_; }
    [[nodiscard]] PFN_vkTransitionImageLayoutEXT transitionImageLayoutHostFn() const {
        return transitionImageLayoutHost_;
    }

    /// A ceiling on every sampler's anisotropy - the game's Texture Filtering.
    ///
    /// Applied where samplers are made rather than by rebuilding the ones that
    /// exist, because the shipped panel marks this setting gameRestart: the
    /// original client did not apply it live either, and says so in the
    /// control's own tooltip. Textures loaded after it changes take the new
    /// value; the rest follow on the next run.
    void setAnisotropyLimit(float limit) {
        anisotropyLimit_ = std::clamp(limit, 1.0f, 16.0f);
    }
    [[nodiscard]] float anisotropyLimit() const { return anisotropyLimit_; }

    // Global sampler cache accessor (set during VkContext::initialize, cleared on shutdown).
    // Used by VkTexture and other code that only has a VkDevice handle.
    static VkContext* globalInstance() { return sInstance_; }

    // UI texture upload: creates a Vulkan texture from RGBA data and returns
    // a VkDescriptorSet suitable for use as ImTextureID.
    // The caller does NOT need to free the result - resources are tracked and
    // cleaned up when the VkContext is destroyed.
    VkDescriptorSet uploadImGuiTexture(const uint8_t* rgba, int width, int height);

private:
    bool createInstance(SDL_Window* window);
    bool createSurface(SDL_Window* window);
    bool selectPhysicalDevice();
    void reportUnsuitableDevices() const;
    bool createLogicalDevice();
    bool createAllocator();
    bool createSwapchain(int width, int height);
    void destroySwapchain();
    bool createCommandPools();
    bool createSyncObjects();
    bool createPipelineCache();
    void savePipelineCache();
    /// Depth buffer, MSAA images, the main render pass and the swapchain
    /// framebuffers: everything that depends on the swapchain's size and
    /// sample count.
    ///
    /// Built once at startup and again on every resize, and the two used to
    /// be separate copies of the same two hundred and sixty lines. `verb` is
    /// only the word in the failure messages, so a log still says which of
    /// the two was running.
    bool createSwapchainRenderTargets(const char* verb);

    bool createImGuiResources();
    void destroyImGuiResources();

    // vk-bootstrap objects (kept alive for swapchain recreation etc.)
    vkb::Instance vkbInstance_;
    vkb::PhysicalDevice vkbPhysicalDevice_;

    VkInstance instance = VK_NULL_HANDLE;
    VkDebugUtilsMessengerEXT debugMessenger = VK_NULL_HANDLE;
    VkSurfaceKHR surface = VK_NULL_HANDLE;
    VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
    VkDevice device = VK_NULL_HANDLE;
    VmaAllocator allocator = VK_NULL_HANDLE;

    // Pipeline cache (persisted to disk for faster startup)
    VkPipelineCache pipelineCache_ = VK_NULL_HANDLE;
    uint32_t gpuVendorId_ = 0;
    char gpuName_[256] = {};

    VkQueue graphicsQueue = VK_NULL_HANDLE;
    VkQueue presentQueue = VK_NULL_HANDLE;
    uint32_t graphicsQueueFamily = 0;
    uint32_t presentQueueFamily = 0;

    // Dedicated transfer queue (second queue from same graphics family)
    VkQueue transferQueue_ = VK_NULL_HANDLE;
    VkCommandPool transferCommandPool_ = VK_NULL_HANDLE;
    bool hasDedicatedTransfer_ = false;
    uint32_t graphicsQueueFamilyQueueCount_ = 1; // queried in selectPhysicalDevice

    // Swapchain
    VkSwapchainKHR swapchain = VK_NULL_HANDLE;
    VkFormat swapchainFormat = VK_FORMAT_UNDEFINED;
    VkExtent2D swapchainExtent = {.width = 0, .height = 0};
    std::vector<VkImage> swapchainImages;
    std::vector<VkImageView> swapchainImageViews;
    std::vector<VkFramebuffer> swapchainFramebuffers;
    bool swapchainDirty = false;
    bool deviceLost_ = false;
    bool vsync_ = true;

    // Per-frame resources
    FrameData frames[MAX_FRAMES_IN_FLIGHT];
    uint32_t currentFrame = 0;

    /// One timeline semaphore across the whole frame ring, replacing the
    /// per-slot fences. VK_NULL_HANDLE when the device did not offer
    /// timelineSemaphore, in which case every path below falls back to the
    /// fences, which are still created either way.
    ///
    /// This does not touch the swapchain semaphores above it and cannot:
    /// vkAcquireNextImageKHR and vkQueuePresentKHR take binary semaphores
    /// only, so the acquire/renderFinished pair stays exactly as it is. The
    /// timeline replaces the CPU-side "is this slot free yet" question.
    bool timelineSemaphoreSupported_ = false;

    /// VK_KHR_synchronization2, detected rather than required. It is core in
    /// Vulkan 1.3, but MoltenVK advertises 1.2 and offers it as an extension,
    /// so asking for 1.3 would lose the platform this is developed on while
    /// the feature itself is right there. Every barrier goes through
    /// cmdPipelineBarrier2() in vk_utils, which lowers the dependency info
    /// back to a legacy vkCmdPipelineBarrier when this is false.
    bool synchronization2Supported_ = false;
    /// Whether it came from core 1.3 rather than the extension. Decides which
    /// entry point name resolves - the promoted one is not loadable on 1.2.
    bool sync2IsCore_ = false;
    PFN_vkCmdPipelineBarrier2KHR cmdPipelineBarrier2_ = nullptr;

    /// VK_EXT_host_image_copy. When present, pixels go from host memory into
    /// the image directly - no staging buffer, no transfer submission, no
    /// barriers around the copy. Detected, so the staging path stays for
    /// devices without it.
    bool hostImageCopySupported_ = false;
    PFN_vkCopyMemoryToImageEXT copyMemoryToImage_ = nullptr;
    PFN_vkTransitionImageLayoutEXT transitionImageLayoutHost_ = nullptr;

    VkSemaphore frameTimeline_ = VK_NULL_HANDLE;
    /// Last value signalled on frameTimeline_. Monotonic for the life of the
    /// device, so it survives a swapchain rebuild without being reset.
    uint64_t frameTimelineValue_ = 0;

    // Per-swapchain-image semaphores (avoids reuse while presentation engine holds them)
    std::vector<VkSemaphore> imageAcquiredSemaphores_;   // [swapchainImageCount], per-image
    std::vector<VkSemaphore> renderFinishedSemaphores_;  // [swapchainImageCount], per-image
    VkSemaphore nextAcquireSemaphore_ = VK_NULL_HANDLE;  // free semaphore for next acquire
    VkSemaphore currentAcquireSemaphore_ = VK_NULL_HANDLE; // the one used for the current frame

    // Immediate submit resources
    VkCommandPool immCommandPool = VK_NULL_HANDLE;
    VkFence immFence = VK_NULL_HANDLE;
    // Cached, reusable cmd buffer for beginSingleTimeCommands. Pool was created
    // with RESET_COMMAND_BUFFER_BIT so we can reset and reuse instead of
    // round-tripping through vkAllocateCommandBuffers + vkFreeCommandBuffers
    // every immediate submit (the M2 frustum-cull dispatch fires this per frame).
    VkCommandBuffer immCmdBuf_ = VK_NULL_HANDLE;

    // Batch upload state (nesting-safe via depth counter)
    int uploadBatchDepth_ = 0;
    bool inUploadBatch_ = false;
    VkCommandBuffer batchCmd_ = VK_NULL_HANDLE;
    std::vector<AllocatedBuffer> batchStagingBuffers_;
    /// Staging allocated with plain Vulkan calls rather than the allocator.
    /// A batch only records the copy, so anything it reads from has to outlive
    /// the recording and be freed once the submit has completed.
    struct RawStaging { VkBuffer buffer; VkDeviceMemory memory; };
    std::vector<RawStaging> batchRawStaging_;

    // Async upload: in-flight batches awaiting GPU completion
    struct InFlightBatch {
        VkFence fence = VK_NULL_HANDLE;
        VkCommandBuffer cmd = VK_NULL_HANDLE;
        std::vector<AllocatedBuffer> stagingBuffers;
        /// Plainly-allocated staging, which has to outlive the submit just as
        /// the allocator's does.
        std::vector<RawStaging> rawStaging;
    };
    std::vector<InFlightBatch> inFlightBatches_;

    void runDeferredCleanup(uint32_t frameIndex);
public:
    // Execute all deferred destruction immediately. For shutdown paths, where no
    // further frames will run to drain the queues naturally.
    void flushDeferredCleanup();
private:
    std::vector<std::function<void()>> deferredCleanup_[MAX_FRAMES_IN_FLIGHT];

    // Depth buffer (shared across all framebuffers)
    VkImage depthImage = VK_NULL_HANDLE;
    VkImageView depthImageView = VK_NULL_HANDLE;
    VmaAllocation depthAllocation = VK_NULL_HANDLE;
    VkFormat depthFormat = VK_FORMAT_D32_SFLOAT;

    bool createDepthBuffer();
    void destroyDepthBuffer();

    // MSAA resources
    VkSampleCountFlagBits msaaSamples_ = VK_SAMPLE_COUNT_1_BIT;
    VkImage msaaColorImage_ = VK_NULL_HANDLE;
    VkImageView msaaColorView_ = VK_NULL_HANDLE;
    VmaAllocation msaaColorAllocation_ = VK_NULL_HANDLE;

    bool createMsaaColorImage();
    void destroyMsaaColorImage();
    bool createDepthResolveImage();
    void destroyDepthResolveImage();

    // Actual Vulkan API version the instance was created with (gates core 1.2 calls)
    uint32_t instanceApiVersion_ = VK_API_VERSION_1_1;
    /// What the physical device reports, which is not what the instance was
    /// created with. Used to tell a 1.3 device - where synchronization2 is
    /// core and the extension string may not be advertised at all - from a
    /// 1.2 one that offers it as an extension.
    uint32_t deviceApiVersion_ = VK_API_VERSION_1_0;

    // MSAA depth resolve support (for sampling/copying resolved depth)
    bool depthResolveSupported_ = false;
    VkResolveModeFlagBits depthResolveMode_ = VK_RESOLVE_MODE_NONE;
    VkImage depthResolveImage = VK_NULL_HANDLE;
    VkImageView depthResolveImageView = VK_NULL_HANDLE;
    VmaAllocation depthResolveAllocation = VK_NULL_HANDLE;

    // ImGui resources
    VkRenderPass imguiRenderPass = VK_NULL_HANDLE;
    VkRenderPass overlayRenderPass = VK_NULL_HANDLE;
    VkRenderPass overlayClearRenderPass = VK_NULL_HANDLE;
    std::vector<VkFramebuffer> overlayFramebuffers;
    VkRenderPass sceneContinueRenderPass = VK_NULL_HANDLE;
    bool createOverlayRenderPass();
    bool createSceneContinueRenderPass();
    void destroyOverlayRenderPass();
    VkDescriptorPool imguiDescriptorPool = VK_NULL_HANDLE;

    // Shared sampler for UI textures (created on first uploadImGuiTexture call)
    VkSampler uiTextureSampler_ = VK_NULL_HANDLE;
    /// Bumped whenever the UI textures and the pool their descriptor sets came
    /// from are destroyed.
    ///
    /// Any cache of those sets is dangling from that moment, and drawing with
    /// one is a fault the GPU reports by resetting. Callers that keep sets
    /// compare this against what they last saw and throw their cache away.
    uint32_t uiTextureGeneration_ = 0;
    /// How many asynchronous upload batches have been submitted and retired.
    /// Only used to name the first fence and to say how many are outstanding.
    uint64_t batchesSubmitted_ = 0;
    uint64_t batchesRetired_ = 0;

    /// A descriptor pool and layout this context owns, for UI textures.
    ///
    /// ImGui_ImplVulkan_AddTexture allocates from ImGui's pool, which is
    /// destroyed whenever the backend restarts - and the backend restarts on
    /// every anti-aliasing change, because that is how its render pass is
    /// rebound. Ten different caches around the interface hold sets from that
    /// pool and none of them hear about it. Allocating from a pool owned here
    /// makes the sets outlive the restart; the layout matches the one ImGui
    /// allocates with, so its pipeline binds them just the same.
    VkDescriptorPool uiTexturePool_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout uiTextureLayout_ = VK_NULL_HANDLE;
    bool ensureUiTextureDescriptorPool();

    // Tracked UI textures for cleanup
    struct UiTexture {
        VkImage image;
        VkDeviceMemory memory;
        VkImageView view;
    };
    std::vector<UiTexture> uiTextures_;

    // Sampler cache - deduplicates VkSamplers by configuration hash.
    std::mutex samplerCacheMutex_;
    std::unordered_map<uint64_t, VkSampler> samplerCache_;
    bool samplerAnisotropySupported_ = false;
    bool fillModeNonSolidSupported_ = false;
    bool fsr2ComputeFeaturesSupported_ = false;
    /// True when the swapchain was built with a transform the surface is not
    /// using, which makes VK_SUBOPTIMAL_KHR permanent rather than a signal.
    bool presentsOffNativeTransform_ = false;
    float anisotropyLimit_ = 16.0f;

    static VkContext* sInstance_;

#ifndef NDEBUG
    bool enableValidation = true;
#else
    bool enableValidation = false;
#endif
    // Whether the layers actually came up this run, including via the env var.
    bool validationActive_ = false;
};

} // namespace rendering
} // namespace wowee
