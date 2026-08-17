#include "rendering/imgui_texture.hpp"

#include <imgui.h>
#include <backends/imgui_impl_vulkan.h>

#include "rendering/vk_context.hpp"

namespace wowee {
namespace rendering {

void removeImGuiTexture(VkDescriptorSet& descriptorSet) {
    if (descriptorSet == VK_NULL_HANDLE) return;
    // BackendRendererUserData is what ImGui_ImplVulkan_Init sets and
    // ImGui_ImplVulkan_Shutdown clears, so it answers whether there is still a
    // backend to give this back to.
    if (ImGui::GetCurrentContext() != nullptr &&
        ImGui::GetIO().BackendRendererUserData != nullptr) {
        ImGui_ImplVulkan_RemoveTexture(descriptorSet);
    }
    descriptorSet = VK_NULL_HANDLE;
}

void ImGuiTexture::destroy(VkDevice device, VmaAllocator allocator) {
    removeImGuiTexture(descriptorSet);
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
