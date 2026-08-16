#include "rendering/imgui_texture.hpp"

#include <backends/imgui_impl_vulkan.h>

#include "rendering/vk_context.hpp"

namespace wowee {
namespace rendering {

void ImGuiTexture::destroy(VkDevice device, VmaAllocator allocator) {
    if (descriptorSet != VK_NULL_HANDLE) {
        ImGui_ImplVulkan_RemoveTexture(descriptorSet);
        descriptorSet = VK_NULL_HANDLE;
    }
    if (texture) {
        texture->destroy(device, allocator);
        texture.reset();
    }
}

ImGuiTexture makeImGuiTexture(VkContext& ctx, const pipeline::BLPImage& image) {
    if (!image.isValid()) return {};

    VkDevice device = ctx.getDevice();
    auto texture = std::make_unique<VkTexture>();
    if (!texture->upload(ctx, image.data.data(), image.width, image.height,
                         VK_FORMAT_R8G8B8A8_UNORM, false)) {
        return {};
    }
    if (!texture->createSampler(device, VK_FILTER_LINEAR, VK_FILTER_LINEAR,
                                VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE, 1.0f)) {
        texture->destroy(device, ctx.getAllocator());
        return {};
    }

    VkDescriptorSet descriptorSet = ImGui_ImplVulkan_AddTexture(
        texture->getSampler(), texture->getImageView(),
        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    if (!descriptorSet) {
        texture->destroy(device, ctx.getAllocator());
        return {};
    }

    return ImGuiTexture{.texture = std::move(texture), .descriptorSet = descriptorSet};
}

ImGuiTexture loadImGuiTexture(pipeline::AssetManager& assets, VkContext& ctx,
                              const std::string& path) {
    return makeImGuiTexture(ctx, assets.loadTexture(path));
}

}  // namespace rendering
}  // namespace wowee
