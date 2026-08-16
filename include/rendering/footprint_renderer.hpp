#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include <glm/glm.hpp>
#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

#include "rendering/vk_texture.hpp"

namespace wowee {
namespace pipeline { class AssetManager; }
namespace rendering {

class Camera;
class Renderer;
class VkContext;

enum class FootprintFallback : uint8_t {
    BIPED,
    HOOF,
    PAW,
    CLAW,
    CLOVEN
};

/**
 * Short-lived ground decals spawned by authored M2 footfall events.
 * CreatureModelData supplies the original-client mask and physical dimensions.
 */
class FootprintRenderer {
public:
    FootprintRenderer() = default;
    ~FootprintRenderer();

    [[nodiscard]] bool initialize(Renderer* owner, VkContext* ctx,
                                  VkDescriptorSetLayout perFrameLayout,
                                  pipeline::AssetManager* assetManager);
    void shutdown();
    void recreatePipelines();

    void update(float deltaTime);
    void clear();
    void spawn(const std::string& modelName, const glm::vec3& basePosition,
               float yawRadians, bool leftFoot, FootprintFallback fallback);
    void render(VkCommandBuffer cmd, VkDescriptorSet perFrameSet, const Camera& camera);

private:
    struct Profile {
        uint8_t textureIndex = 0;
        float length = 1.0f;
        float width = 0.8f;
    };

    struct Print {
        glm::vec3 position{0.0f};
        float yaw = 0.0f;
        float length = 1.0f;
        float signedWidth = 0.8f;
        float age = 0.0f;
        uint8_t textureIndex = 0;
    };

    static constexpr size_t kTextureCount = 6;

    Renderer* owner_ = nullptr;
    VkContext* vkCtx_ = nullptr;
    VkDescriptorSetLayout perFrameLayout_ = VK_NULL_HANDLE;
    VkPipeline pipeline_ = VK_NULL_HANDLE;
    VkPipelineLayout pipelineLayout_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout materialSetLayout_ = VK_NULL_HANDLE;
    VkDescriptorPool descriptorPool_ = VK_NULL_HANDLE;
    std::array<VkDescriptorSet, kTextureCount> textureSets_{};
    std::array<VkTexture, kTextureCount> textures_{};
    VkBuffer quadVB_ = VK_NULL_HANDLE;
    VmaAllocation quadVBAlloc_ = VK_NULL_HANDLE;

    std::unordered_map<std::string, Profile> profilesByPath_;
    std::unordered_map<std::string, Profile> profilesByBasename_;
    std::vector<Print> prints_;

    bool createPipeline();
    bool createDescriptorResources();
    bool createQuad();
    bool loadFootprintData(pipeline::AssetManager* assetManager);
    [[nodiscard]] Profile resolveProfile(const std::string& modelName, FootprintFallback fallback) const;
    [[nodiscard]] float resolveFloorHeight(const glm::vec3& position) const;
};

} // namespace rendering
} // namespace wowee
