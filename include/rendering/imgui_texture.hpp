#pragma once

#include <memory>

#include <vk_mem_alloc.h>
#include <string>

#include "pipeline/asset_manager.hpp"
#include "rendering/vk_texture.hpp"

namespace wowee {
namespace rendering {

class VkContext;

/// A texture together with the descriptor set that lets ImGui draw it.
///
/// The two are one thing: the descriptor set refers to the texture's image
/// view and sampler, so it stops being valid the moment the texture goes.
/// Holding them apart is what made three copies of the loading code all have
/// to remember to destroy the texture on each of their failure paths.
/// Hand a descriptor set back to ImGui's Vulkan backend, if it is still there.
///
/// ImGui_ImplVulkan_RemoveTexture reads the backend's data unconditionally, so
/// calling it after ImGui_ImplVulkan_Shutdown dereferences what that just
/// freed. Several things hold these sets until their own destructors run, and
/// those run in an order no single call site controls, so the check belongs
/// here rather than in an ordering that has to be got right everywhere.
void removeImGuiTexture(VkDescriptorSet& descriptorSet);

struct ImGuiTexture {
    std::unique_ptr<VkTexture> texture;
    VkDescriptorSet descriptorSet = VK_NULL_HANDLE;

    explicit operator bool() const { return texture != nullptr; }

    /// Release both, in the order that keeps them valid to the end.
    ///
    /// The descriptor set goes first: it refers to the texture's image view,
    /// so freeing the image while ImGui still holds the set leaves the set
    /// pointing at freed memory for as long as it takes to remove it. Four
    /// hand-written copies of this existed - a destructor and a clear in each
    /// of the two world map marker layers - and each had to remember both
    /// halves and the order.
    void destroy(VkDevice device, VmaAllocator allocator);
};

/// Uploads a decoded image and registers it with the ImGui backend.
///
/// Answers an empty result if any step fails, having already destroyed
/// whatever it had built, so a caller has nothing to unwind.
///
/// Clamped rather than repeated, because everything drawn this way is a
/// discrete piece of art: a map marker, a zone highlight. Repeating puts the
/// opposite edge's pixels along the seam of a rotated marker.
ImGuiTexture makeImGuiTexture(VkContext& ctx, const pipeline::BLPImage& image);

/// The same, for the common case of a BLP named by path. An image that will
/// not load answers an empty result exactly as a failed upload does; the
/// caller decides whether that is worth a log line.
ImGuiTexture loadImGuiTexture(pipeline::AssetManager& assets, VkContext& ctx,
                              const std::string& path);

}  // namespace rendering
}  // namespace wowee
