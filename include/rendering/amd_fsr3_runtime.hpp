#pragma once

#include <cstdint>
#include <string>
#include <vulkan/vulkan.h>

namespace wowee::rendering {

struct AmdFsr3RuntimeInitDesc {
    VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
    VkDevice device = VK_NULL_HANDLE;
    PFN_vkGetDeviceProcAddr getDeviceProcAddr = nullptr;
    uint32_t maxRenderWidth = 0;
    uint32_t maxRenderHeight = 0;
    uint32_t displayWidth = 0;
    uint32_t displayHeight = 0;
    VkFormat colorFormat = VK_FORMAT_UNDEFINED;
    bool hdrInput = false;
    bool depthInverted = false;
    bool enableFrameGeneration = false;
};

struct AmdFsr3RuntimeDispatchDesc {
    VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
    VkImage colorImage = VK_NULL_HANDLE;
    VkImage depthImage = VK_NULL_HANDLE;
    VkImage motionVectorImage = VK_NULL_HANDLE;
    VkImage outputImage = VK_NULL_HANDLE;
    VkImage frameGenOutputImage = VK_NULL_HANDLE;
    uint32_t renderWidth = 0;
    uint32_t renderHeight = 0;
    uint32_t outputWidth = 0;
    uint32_t outputHeight = 0;
    VkFormat colorFormat = VK_FORMAT_UNDEFINED;
    VkFormat depthFormat = VK_FORMAT_UNDEFINED;
    VkFormat motionVectorFormat = VK_FORMAT_R16G16_SFLOAT;
    VkFormat outputFormat = VK_FORMAT_UNDEFINED;
    float jitterX = 0.0f;
    float jitterY = 0.0f;
    float motionScaleX = 1.0f;
    float motionScaleY = 1.0f;
    float frameTimeDeltaMs = 16.67f;
    float cameraNear = 0.1f;
    float cameraFar = 1000.0f;
    float cameraFovYRadians = 1.0f;
    bool reset = false;
};

class AmdFsr3Runtime {
public:
    enum class LoadPathKind {
        None,
        Official
    };

    AmdFsr3Runtime();
    ~AmdFsr3Runtime();

    bool initialize(const AmdFsr3RuntimeInitDesc& desc);
    bool dispatchUpscale(const AmdFsr3RuntimeDispatchDesc& desc);
    bool dispatchFrameGeneration(const AmdFsr3RuntimeDispatchDesc& desc);
    void shutdown();

    [[nodiscard]] bool isReady() const { return ready_; }
    [[nodiscard]] bool isFrameGenerationReady() const { return frameGenerationReady_; }
    [[nodiscard]] const std::string& loadedLibraryPath() const { return loadedLibraryPath_; }
    [[nodiscard]] LoadPathKind loadPathKind() const { return loadPathKind_; }
    [[nodiscard]] const std::string& lastError() const { return lastError_; }

private:
    enum class ApiMode {
        LegacyFsr3,
        GenericApi
    };

    void* libHandle_ = nullptr;
    std::string loadedLibraryPath_;
    void* scratchBuffer_ = nullptr;
    size_t scratchBufferSize_ = 0;
    bool ready_ = false;
    bool frameGenerationReady_ = false;
    LoadPathKind loadPathKind_ = LoadPathKind::None;
    std::string lastError_;

    struct RuntimeFns;
    RuntimeFns* fns_ = nullptr;
    void* contextStorage_ = nullptr;
    ApiMode apiMode_ = ApiMode::LegacyFsr3;
    // Guarded to match their uses: every read and write of these is inside a
    // WOWEE_HAS_AMD_FSR3_FRAMEGEN block, so with the backend off they are
    // fields nothing touches, which clang rejects under -Werror.
#if WOWEE_HAS_AMD_FSR3_FRAMEGEN
    void* genericUpscaleContext_ = nullptr;
    void* genericFramegenContext_ = nullptr;
#endif
    // Not guarded: this one is read outside the block, unlike the two above.
    uint64_t genericFrameId_ = 1;
};

}  // namespace wowee::rendering
