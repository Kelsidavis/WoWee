#pragma once

#include <vulkan/vulkan.h>
#include <string>
#include <vector>

namespace wowee {
namespace rendering {

class VkShaderModule {
public:
    VkShaderModule() = default;
    ~VkShaderModule();

    VkShaderModule(const VkShaderModule&) = delete;
    VkShaderModule& operator=(const VkShaderModule&) = delete;
    VkShaderModule(VkShaderModule&& other) noexcept;
    VkShaderModule& operator=(VkShaderModule&& other) noexcept;

    // Load a SPIR-V file from disk
    [[nodiscard]] bool loadFromFile(VkDevice device, const std::string& path);

    // Load from raw SPIR-V bytes
    bool loadFromMemory(VkDevice device, const uint32_t* code, size_t sizeBytes);

    void destroy();

    ::VkShaderModule getModule() const { return module_; }
    bool isValid() const { return module_ != VK_NULL_HANDLE; }

    // Create a VkPipelineShaderStageCreateInfo for this module
    VkPipelineShaderStageCreateInfo stageInfo(VkShaderStageFlagBits stage,
        const char* entryPoint = "main") const;

private:
    VkDevice device_ = VK_NULL_HANDLE;
    ::VkShaderModule module_ = VK_NULL_HANDLE;
};

/// A pipeline's vertex and fragment shaders, loaded together.
///
/// The stage infos carry the module handles, so this has to outlive the
/// pipeline creation that reads them - keep it a local of whatever builds the
/// pipeline, which is what every call site already does.
struct ShaderPair {
    VkShaderModule vert;
    VkShaderModule frag;
    VkPipelineShaderStageCreateInfo vertStage{};
    VkPipelineShaderStageCreateInfo fragStage{};

    /// False when either failed to load, which is when the caller gives up.
    explicit operator bool() const { return vert.isValid() && frag.isValid(); }
};

/// Load a vertex and fragment shader, naming what they were for if one fails.
///
/// Twenty-six places did this by hand: two loads, two error logs, two stage
/// infos. The error message was the only thing that varied, and half of them
/// said the same words with a different noun.
ShaderPair loadShaderPair(VkDevice device, const std::string& vertPath,
                          const std::string& fragPath, const char* what);


} // namespace rendering
} // namespace wowee
