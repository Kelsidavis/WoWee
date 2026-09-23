// rt_probe: the ray traced lighting pass, run once against a scene whose
// answer is known, on whatever Vulkan device this machine has.
//
// The pass only runs in the world, and the world needs a realm. This stands
// up the real context and the real RtScene / RtLighting on a hidden window,
// puts a box between a wall and the sun, fakes the wall's depth buffer, traces
// one frame and reads the result back:
//
//   - a wall point whose ray to the sun crosses the box must be in shadow,
//   - wall points whose rays miss it must be lit,
//   - every value must be finite.
//
//     cmake -S . -B build -DWOWEE_BUILD_RT_PROBE=ON
//     cmake --build build --target rt_probe
//     cd build/bin && ./rt_probe            (exit status 0 = pass)
//
// WOWEE_RT_FORCE_SOFTWARE=1 selects the compute tracer on hardware that has
// ray queries, so both backends can be checked on one machine.

#include "core/logger.hpp"
#include "rendering/rt_bvh.hpp"
#include "rendering/rt_lighting.hpp"
#include "rendering/rt_scene.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/vk_utils.hpp"

#include <SDL3/SDL.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/packing.hpp>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

using namespace wowee::rendering;

namespace {

constexpr uint32_t kSize = 256;       // scene depth, square
constexpr float kWallDistance = 20.0f;

RtScene::MeshSource box(const glm::vec3& lo, const glm::vec3& hi) {
    RtScene::MeshSource m;
    for (int i = 0; i < 8; ++i) {
        m.positions.emplace_back((i & 1) ? hi.x : lo.x, (i & 2) ? hi.y : lo.y, (i & 4) ? hi.z : lo.z);
    }
    const uint32_t faces[6][4] = {{0, 2, 6, 4}, {1, 5, 7, 3}, {0, 4, 5, 1},
                                  {2, 3, 7, 6}, {0, 1, 3, 2}, {4, 6, 7, 5}};
    for (const auto& f : faces) {
        for (uint32_t idx : {f[0], f[1], f[2], f[0], f[2], f[3]}) m.indices.push_back(idx);
    }
    m.surfaces.push_back(packRtSurface(glm::vec3(0.6f), 1.0f));
    return m;
}

}  // namespace

int main() {
    wowee::core::Logger::getInstance().setLogLevel(wowee::core::LogLevel::INFO);
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        return 2;
    }
    SDL_Window* window = SDL_CreateWindow("rt_probe", kSize, kSize,
                                          SDL_WINDOW_VULKAN | SDL_WINDOW_HIDDEN);
    if (!window) {
        std::fprintf(stderr, "SDL_CreateWindow: %s\n", SDL_GetError());
        return 2;
    }

    int status = 1;
    {
        VkContext ctx;
        if (!ctx.initialize(window)) {
            std::fprintf(stderr, "VkContext failed\n");
            return 2;
        }
        RtScene scene;
        RtLighting lighting;
        if (!scene.initialize(&ctx) || !lighting.initialize(&ctx, &scene)) {
            std::fprintf(stderr, "RT init failed\n");
            return 2;
        }
        std::printf("backend: %s\n",
                    scene.backend() == RtScene::Backend::Hardware ? "hardware" : "compute");

        // A 2x4x4 box five yards in front of the wall. The camera looks down
        // +X with Z up, as the client's does.
        const RtScene::MeshId mesh = scene.addMesh(box({14, -2, -2}, {16, 2, 2}));
        scene.addInstance(mesh, glm::mat4(1.0f));
        // A second instance of it, moved, to exercise the top level.
        scene.addInstance(mesh, glm::translate(glm::mat4(1.0f), glm::vec3(0, 40, 0)));

        glm::mat4 proj = glm::perspective(glm::radians(60.0f), 1.0f, 0.5f, 2000.0f);
        proj[1][1] *= -1.0f;
        const glm::mat4 view = glm::lookAt(glm::vec3(0), glm::vec3(1, 0, 0), glm::vec3(0, 0, 1));
        const glm::mat4 viewProj = proj * view;
        const glm::vec4 wallClip = viewProj * glm::vec4(kWallDistance, 0, 0, 1);
        const float wallDepth = wallClip.z / wallClip.w;

        const VkExtent2D sceneExtent{kSize, kSize};
        AllocatedImage depth = createImage(ctx.getDevice(), ctx.getAllocator(), kSize, kSize,
                                           ctx.getDepthFormat(),
                                           VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT |
                                               VK_IMAGE_USAGE_SAMPLED_BIT |
                                               VK_IMAGE_USAGE_TRANSFER_DST_BIT);

        lighting.setMode(RtLighting::Mode::Full);
        lighting.prepare(sceneExtent);

        RtLighting::FrameInputs in{};
        in.viewProj = viewProj;
        in.cameraPos = glm::vec3(0);
        in.sunDir = glm::normalize(glm::vec3(-1.0f, 0.0f, 0.25f));  // toward the sun
        in.sunColor = glm::vec3(1.0f);
        in.skyColor = glm::vec3(0.3f, 0.35f, 0.4f);
        in.sunUp = true;

        auto depthLayout = [&](VkCommandBuffer cmd, VkImageLayout from, VkImageLayout to) {
            VkImageMemoryBarrier2 b{};
            b.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
            b.srcStageMask = b.dstStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
            b.srcAccessMask = b.dstAccessMask = VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT;
            b.oldLayout = from;
            b.newLayout = to;
            b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            b.image = depth.image;
            b.subresourceRange = {VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, 1};
            VkDependencyInfo dep{};
            dep.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
            dep.imageMemoryBarrierCount = 1;
            dep.pImageMemoryBarriers = &b;
            cmdPipelineBarrier2(cmd, dep);
        };
        ctx.immediateSubmit([&](VkCommandBuffer cmd) {
            depthLayout(cmd, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
            VkClearDepthStencilValue clear{wallDepth, 0};
            VkImageSubresourceRange range{VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, 1};
            vkCmdClearDepthStencilImage(cmd, depth.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                        &clear, 1, &range);
            depthLayout(cmd, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                        VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL);
            lighting.record(cmd, depth.image, sceneExtent, false, in);
        });

        const VkExtent2D te = lighting.traceExtent();
        const VkDeviceSize bytes = VkDeviceSize(te.width) * te.height * 8;
        AllocatedBuffer light = createBuffer(ctx.getAllocator(), bytes,
                                             VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                             VMA_MEMORY_USAGE_CPU_ONLY);
        AllocatedBuffer gi = createBuffer(ctx.getAllocator(), bytes,
                                          VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                          VMA_MEMORY_USAGE_CPU_ONLY);
        ctx.immediateSubmit([&](VkCommandBuffer cmd) {
            VkBufferImageCopy region{};
            region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
            region.imageExtent = {te.width, te.height, 1};
            vkCmdCopyImageToBuffer(cmd, lighting.lightImageWrittenBy(ctx.getCurrentFrame()),
                                   VK_IMAGE_LAYOUT_GENERAL, light.buffer, 1, &region);
            vkCmdCopyImageToBuffer(cmd, lighting.giImageWrittenBy(ctx.getCurrentFrame()),
                                   VK_IMAGE_LAYOUT_GENERAL, gi.buffer, 1, &region);
        });
        vkDeviceWaitIdle(ctx.getDevice());

        const auto* lightTexels = static_cast<const uint16_t*>(light.info.pMappedData);
        const auto* giTexels = static_cast<const uint16_t*>(gi.info.pMappedData);
        auto texel = [&](const uint16_t* base, uint32_t x, uint32_t y, int c) {
            return glm::unpackHalf1x16(base[(size_t(y) * te.width + x) * 4 + c]);
        };
        auto sample = [&](const glm::vec3& world, float& sun, float& dist) {
            const glm::vec4 c = viewProj * glm::vec4(world, 1.0f);
            const glm::vec2 uv = glm::vec2(c) / c.w * 0.5f + 0.5f;
            const auto x = static_cast<uint32_t>(uv.x * te.width);
            const auto y = static_cast<uint32_t>(uv.y * te.height);
            sun = texel(lightTexels, x, y, 0);
            dist = texel(lightTexels, x, y, 3);
        };

        int failures = 0;
        auto expect = [&](const char* what, bool ok) {
            std::printf("  %-52s %s\n", what, ok ? "ok" : "FAIL");
            if (!ok) ++failures;
        };

        // Rays toward the sun rise a quarter yard per yard: from the wall, the
        // box's front face (x = 16) is four yards back, so a wall point is
        // shadowed for z in about [-3, 1] and |y| < 2.
        float sun = 0, dist = 0;
        sample({kWallDistance, 0.0f, -1.0f}, sun, dist);
        std::printf("wall behind box: sun=%.3f dist=%.2f\n", sun, dist);
        expect("wall point behind the box is shadowed", sun < 0.25f);
        expect("stored distance matches the wall", std::abs(dist - kWallDistance) < 0.5f);
        sample({kWallDistance, 0.0f, 5.0f}, sun, dist);
        std::printf("wall above shadow: sun=%.3f\n", sun);
        expect("wall point above the shadow is lit", sun > 0.75f);
        sample({kWallDistance, 7.0f, 0.0f}, sun, dist);
        std::printf("wall beside shadow: sun=%.3f\n", sun);
        expect("wall point beside the shadow is lit", sun > 0.75f);

        bool finite = true;
        uint32_t shadowed = 0;
        for (uint32_t y = 0; y < te.height; ++y) {
            for (uint32_t x = 0; x < te.width; ++x) {
                for (int c = 0; c < 4; ++c) {
                    finite = finite && std::isfinite(texel(lightTexels, x, y, c)) &&
                             std::isfinite(texel(giTexels, x, y, c));
                }
                if (texel(lightTexels, x, y, 0) < 0.5f) ++shadowed;
            }
        }
        std::printf("trace %ux%u, %u shadowed texels\n", te.width, te.height, shadowed);
        expect("every value is finite", finite);
        expect("the shadow is a patch, not the frame", shadowed > 20 && shadowed < te.width * te.height / 3);

        destroyBuffer(ctx.getAllocator(), light);
        destroyBuffer(ctx.getAllocator(), gi);
        destroyImage(ctx.getDevice(), ctx.getAllocator(), depth);
        lighting.shutdown();
        scene.shutdown();
        ctx.shutdown();
        status = failures == 0 ? 0 : 1;
        std::printf("%s\n", status == 0 ? "PASS" : "FAIL");
    }
    SDL_DestroyWindow(window);
    SDL_Quit();
    return status;
}
