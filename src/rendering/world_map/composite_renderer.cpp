// composite_renderer.cpp - Vulkan off-screen composite rendering for the world map.
// Extracted from WorldMap::initialize, shutdown, compositePass, loadZoneTextures,
// loadOverlayTextures, destroyZoneTextures (Phase 7 of refactoring plan).
#include "rendering/world_map/composite_renderer.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/vk_texture.hpp"
#include "rendering/vk_render_target.hpp"
#include "rendering/vk_pipeline.hpp"
#include "rendering/vk_shader.hpp"
#include "rendering/vk_utils.hpp"
#include "pipeline/asset_manager.hpp"
#include "core/logger.hpp"

namespace wowee {
namespace rendering {
namespace world_map {

CompositeRenderer::CompositeRenderer() = default;

CompositeRenderer::~CompositeRenderer() {
    shutdown();
}

void CompositeRenderer::ensureTextureSlots(size_t zoneCount, const std::vector<Zone>& zones) {
    if (zoneTextureSlots_.size() >= zoneCount) return;
    zoneTextureSlots_.resize(zoneCount);
    for (size_t i = 0; i < zoneCount; i++) {
        auto& slots = zoneTextureSlots_[i];
        if (slots.overlays.size() != zones[i].overlays.size()) {
            slots.overlays.resize(zones[i].overlays.size());
            for (size_t oi = 0; oi < zones[i].overlays.size(); oi++) {
                const auto& ov = zones[i].overlays[oi];
                slots.overlays[oi].tiles.resize(static_cast<size_t>(ov.tileCols) * static_cast<size_t>(ov.tileRows), nullptr);
            }
        }
    }
}

bool CompositeRenderer::initialize(VkContext* ctx, pipeline::AssetManager* am) {
    if (initialized) return true;
    vkCtx = ctx;
    assetManager = am;
    VkDevice device = vkCtx->getDevice();

    // --- Composite render target (1024x768) ---
    compositeTarget = std::make_unique<VkRenderTarget>();
    if (!compositeTarget->create(*vkCtx, FBO_W, FBO_H)) {
        LOG_ERROR("CompositeRenderer: failed to create composite render target");
        return false;
    }

    // --- Quad vertex buffer (unit quad: pos2 + uv2) ---
    float quadVerts[] = {
        0.0f, 0.0f,  0.0f, 0.0f,
        1.0f, 0.0f,  1.0f, 0.0f,
        1.0f, 1.0f,  1.0f, 1.0f,
        0.0f, 0.0f,  0.0f, 0.0f,
        1.0f, 1.0f,  1.0f, 1.0f,
        0.0f, 1.0f,  0.0f, 1.0f,
    };
    auto quadBuf = uploadBuffer(*vkCtx, quadVerts, sizeof(quadVerts),
                                VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    quadVB = quadBuf.buffer;
    quadVBAlloc = quadBuf.allocation;

    // --- Descriptor set layout: 1 combined image sampler at binding 0 ---
    VkDescriptorSetLayoutBinding samplerBinding{};
    samplerBinding.binding = 0;
    samplerBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    samplerBinding.descriptorCount = 1;
    samplerBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    samplerSetLayout = createDescriptorSetLayout(device, { samplerBinding });

    // --- Descriptor pool ---
    VkDescriptorPoolSize poolSize{};
    poolSize.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    poolSize.descriptorCount = MAX_DESC_SETS;

    VkDescriptorPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.maxSets = MAX_DESC_SETS;
    poolInfo.poolSizeCount = 1;
    poolInfo.pPoolSizes = &poolSize;
    vkCreateDescriptorPool(device, &poolInfo, nullptr, &descPool);

    // --- Allocate descriptor sets ---
    constexpr uint32_t tileSetCount = 24;
    constexpr uint32_t overlaySetCount = MAX_OVERLAY_TILES * 2;
    constexpr uint32_t totalSets = tileSetCount + 1 + 1 + overlaySetCount;
    std::vector<VkDescriptorSetLayout> layouts(totalSets, samplerSetLayout);
    VkDescriptorSetAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = descPool;
    allocInfo.descriptorSetCount = totalSets;
    allocInfo.pSetLayouts = layouts.data();

    std::vector<VkDescriptorSet> allSets(totalSets);
    vkAllocateDescriptorSets(device, &allocInfo, allSets.data());

    uint32_t si = 0;
    for (auto& tileRow : tileDescSets)
        for (auto& tileSet : tileRow)
            tileSet = allSets[si++];
    imguiDisplaySet = allSets[si++];
    fogDescSet_ = allSets[si++];
    for (auto& overlayRow : overlayDescSets_)
        for (auto& overlaySet : overlayRow)
            overlaySet = allSets[si++];

    // --- Write display descriptor set → composite render target ---
    VkDescriptorImageInfo compositeImgInfo = compositeTarget->descriptorInfo();
    VkWriteDescriptorSet displayWrite{};
    displayWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    displayWrite.dstSet = imguiDisplaySet;
    displayWrite.dstBinding = 0;
    displayWrite.descriptorCount = 1;
    displayWrite.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    displayWrite.pImageInfo = &compositeImgInfo;
    vkUpdateDescriptorSets(device, 1, &displayWrite, 0, nullptr);

    // --- Pipeline layout: samplerSetLayout + push constant (16 bytes, vertex) ---
    VkPushConstantRange tilePush{};
    tilePush.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    tilePush.offset = 0;
    tilePush.size = sizeof(WorldMapTilePush);
    tilePipelineLayout = createPipelineLayout(device, { samplerSetLayout }, { tilePush });

    // --- Vertex input: pos2 (loc 0) + uv2 (loc 1), stride 16 ---
    VkVertexInputBindingDescription binding{};
    binding.binding = 0;
    binding.stride = 4 * sizeof(float);
    binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    std::vector<VkVertexInputAttributeDescription> attrs(2);
    attrs[0] = { .location = 0, .binding = 0, .format = VK_FORMAT_R32G32_SFLOAT, .offset = 0 };
    attrs[1] = { .location = 1, .binding = 0, .format = VK_FORMAT_R32G32_SFLOAT, .offset = 2 * sizeof(float) };

    // --- Load tile shaders and build pipeline ---
    {
        auto shaders = loadShaderPair(device, "assets/shaders/world_map.vert.spv", "assets/shaders/world_map.frag.spv", "world_map tile");
        if (!shaders) return false;

        tilePipeline = PipelineBuilder()
            .setShaders(shaders.vertStage, shaders.fragStage)
            .setVertexInput({ binding }, attrs)
            .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
            .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
            .setNoDepthTest()
            .setColorBlendAttachment(PipelineBuilder::blendDisabled())
            .setLayout(tilePipelineLayout)
            .setRenderPass(compositeTarget->getRenderPass())
            .setDynamicStates(viewportAndScissorDynamic())
            .build(device, vkCtx->getPipelineCache());

    }

    if (!tilePipeline) {
        LOG_ERROR("CompositeRenderer: failed to create tile pipeline");
        return false;
    }

    // --- Overlay pipeline (alpha-blended) ---
    {
        VkPushConstantRange overlayPushVert{};
        overlayPushVert.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
        overlayPushVert.offset = 0;
        overlayPushVert.size = sizeof(WorldMapTilePush);

        VkPushConstantRange overlayPushFrag{};
        overlayPushFrag.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        overlayPushFrag.offset = 16;
        overlayPushFrag.size = sizeof(glm::vec4);

        overlayPipelineLayout_ = createPipelineLayout(device, { samplerSetLayout },
                                                       { overlayPushVert, overlayPushFrag });

        auto shaders = loadShaderPair(device, "assets/shaders/world_map.vert.spv", "assets/shaders/world_map_fog.frag.spv", "world_map overlay");
        if (!shaders) return false;

        overlayPipeline_ = PipelineBuilder()
            .setShaders(shaders.vertStage, shaders.fragStage)
            .setVertexInput({ binding }, attrs)
            .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
            .setRasterization(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE)
            .setNoDepthTest()
            .setColorBlendAttachment(PipelineBuilder::blendAlpha())
            .setLayout(overlayPipelineLayout_)
            .setRenderPass(compositeTarget->getRenderPass())
            .setDynamicStates(viewportAndScissorDynamic())
            .build(device, vkCtx->getPipelineCache());

    }

    if (!overlayPipeline_) {
        LOG_ERROR("CompositeRenderer: failed to create overlay pipeline");
        return false;
    }

    // --- 1×1 white fog texture ---
    {
        uint8_t white[] = { 255, 255, 255, 255 };
        fogTexture_ = std::make_unique<VkTexture>();
        fogTexture_->upload(*vkCtx, white, 1, 1, VK_FORMAT_R8G8B8A8_UNORM, false);
        fogTexture_->createSampler(device, VK_FILTER_NEAREST, VK_FILTER_NEAREST,
                                   VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE, 1.0f);

        // upload and createSampler both answer nothing, and the image view is
        // created without being checked either, so a 1x1 texture is not
        // guaranteed sampleable just because it is tiny. Writing an
        // unsampleable one into this set costs the device rather than the fog.
        // See #123.
        if (!fogTexture_->isValid()) {
            LOG_ERROR("CompositeRenderer: the fog texture is not sampleable");
            return false;
        }
        VkDescriptorImageInfo fogImgInfo = fogTexture_->descriptorInfo();
        VkWriteDescriptorSet fogWrite{};
        fogWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        fogWrite.dstSet = fogDescSet_;
        fogWrite.dstBinding = 0;
        fogWrite.descriptorCount = 1;
        fogWrite.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        fogWrite.pImageInfo = &fogImgInfo;
        vkUpdateDescriptorSets(device, 1, &fogWrite, 0, nullptr);
    }

    initialized = true;
    LOG_INFO("CompositeRenderer initialized (", FBO_W, "x", FBO_H, " composite)");
    return true;
}

void CompositeRenderer::shutdown() {
    if (!vkCtx) return;
    VkDevice device = vkCtx->getDevice();
    VmaAllocator alloc = vkCtx->getAllocator();

    vkDeviceWaitIdle(device);

    destroy(device, tilePipeline);
    destroy(device, tilePipelineLayout);
    destroy(device, overlayPipeline_);
    destroy(device, overlayPipelineLayout_);
    destroy(device, descPool);
    destroy(device, samplerSetLayout);
    destroy(alloc, quadVB, quadVBAlloc);

    for (auto& tex : zoneTextures) {
        if (tex) tex->destroy(device, alloc);
    }
    zoneTextures.clear();
    zoneTextureSlots_.clear();

    if (fogTexture_) { fogTexture_->destroy(device, alloc); fogTexture_.reset(); }
    if (compositeTarget) { compositeTarget->destroy(device, alloc); compositeTarget.reset(); }

    initialized = false;
    vkCtx = nullptr;
}

void CompositeRenderer::loadZoneTextures(int zoneIdx, std::vector<Zone>& zones,
                                          const std::string& mapName) {
    if (zoneIdx < 0 || zoneIdx >= static_cast<int>(zones.size())) return;
    ensureTextureSlots(zones.size(), zones);
    auto& slots = zoneTextureSlots_[zoneIdx];
    if (slots.tilesLoaded) return;
    slots.tilesLoaded = true;

    const auto& zone = zones[zoneIdx];
    const std::string& folder = zone.areaName;
    if (folder.empty()) return;

    LOG_INFO("loadZoneTextures: zone[", zoneIdx, "] areaName='", zone.areaName,
             "' areaID=", zone.areaID, " mapName='", mapName, "'");

    VkDevice device = vkCtx->getDevice();
    int loaded = 0;

    for (int i = 0; i < 12; i++) {
        std::string path = "Interface\\WorldMap\\" + folder + "\\" +
                           folder + std::to_string(i + 1) + ".blp";
        auto blpImage = assetManager->loadTexture(path);
        if (!blpImage.isValid()) {
            slots.tileTextures[i] = nullptr;
            continue;
        }

        auto tex = std::make_unique<VkTexture>();
        tex->upload(*vkCtx, blpImage.data.data(), blpImage.width, blpImage.height,
                    VK_FORMAT_R8G8B8A8_UNORM, false);
        tex->createSampler(device, VK_FILTER_LINEAR, VK_FILTER_LINEAR,
                           VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE, 1.0f);

        slots.tileTextures[i] = tex.get();
        zoneTextures.push_back(std::move(tex));
        loaded++;
    }

    LOG_INFO("CompositeRenderer: loaded ", loaded, "/12 tiles for '", folder, "'");
}

void CompositeRenderer::loadOverlayTextures(int zoneIdx, std::vector<Zone>& zones) {
    if (zoneIdx < 0 || zoneIdx >= static_cast<int>(zones.size())) return;
    ensureTextureSlots(zones.size(), zones);

    const auto& zone = zones[zoneIdx];
    auto& slots = zoneTextureSlots_[zoneIdx];
    if (zone.overlays.empty()) return;

    const std::string& folder = zone.areaName;
    if (folder.empty()) return;

    VkDevice device = vkCtx->getDevice();
    int totalLoaded = 0;

    for (size_t oi = 0; oi < zone.overlays.size(); oi++) {
        const auto& ov = zone.overlays[oi];
        auto& ovSlots = slots.overlays[oi];
        if (ovSlots.tilesLoaded) continue;
        ovSlots.tilesLoaded = true;

        int tileCount = ov.tileCols * ov.tileRows;
        for (int t = 0; t < tileCount; t++) {
            std::string tileName = ov.textureName + std::to_string(t + 1);
            std::string path = "Interface\\WorldMap\\" + folder + "\\" + tileName + ".blp";
            auto blpImage = assetManager->loadTexture(path);
            if (!blpImage.isValid()) {
                ovSlots.tiles[t] = nullptr;
                continue;
            }

            auto tex = std::make_unique<VkTexture>();
            tex->upload(*vkCtx, blpImage.data.data(), blpImage.width, blpImage.height,
                        VK_FORMAT_R8G8B8A8_UNORM, false);
            tex->createSampler(device, VK_FILTER_LINEAR, VK_FILTER_LINEAR,
                               VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE, 1.0f);

            ovSlots.tiles[t] = tex.get();
            zoneTextures.push_back(std::move(tex));
            totalLoaded++;
        }
    }

    LOG_INFO("CompositeRenderer: loaded ", totalLoaded, " overlay tiles for '", folder, "'");
}

void CompositeRenderer::detachZoneTextures() {
    if (!zoneTextures.empty() && vkCtx) {
        // Defer destruction until all in-flight frames have completed.
        // This avoids calling vkDeviceWaitIdle mid-frame, which can trigger
        // driver TDR (GPU device lost) under heavy rendering load.
        VkDevice device = vkCtx->getDevice();
        VmaAllocator alloc = vkCtx->getAllocator();
        auto captured = std::make_shared<std::vector<std::unique_ptr<VkTexture>>>(
            std::move(zoneTextures));
        vkCtx->deferAfterAllFrameFences([device, alloc, captured]() {
            for (auto& tex : *captured) {
                if (tex) tex->destroy(device, alloc);
            }
        });
    }
    zoneTextures.clear();

    // Clear CPU-side tracking immediately so new zones get fresh loads
    for (auto& slots : zoneTextureSlots_) {
        for (auto& tex : slots.tileTextures) tex = nullptr;
        slots.tilesLoaded = false;
        for (auto& ov : slots.overlays) {
            for (auto& t : ov.tiles) t = nullptr;
            ov.tilesLoaded = false;
        }
    }
    zoneTextureSlots_.clear();
}

void CompositeRenderer::flushStaleTextures() {
    // No-op: texture cleanup is now handled by deferAfterAllFrameFences
    // in detachZoneTextures. Kept for API compatibility.
}

void CompositeRenderer::requestComposite(int zoneIdx) {
    pendingCompositeIdx_ = zoneIdx;
}

bool CompositeRenderer::hasAnyTile(int zoneIdx) const {
    if (zoneIdx < 0 || zoneIdx >= static_cast<int>(zoneTextureSlots_.size()))
        return false;
    const auto& slots = zoneTextureSlots_[zoneIdx];
    for (auto tileTexture : slots.tileTextures) {
        if (tileTexture != nullptr) return true;
    }
    return false;
}

void CompositeRenderer::compositePass(VkCommandBuffer cmd,
                                       const std::vector<Zone>& zones,
                                       const std::unordered_set<int>& exploredOverlays,
                                       bool hasServerMask) {
    if (!initialized || pendingCompositeIdx_ < 0 || !compositeTarget) return;
    if (pendingCompositeIdx_ >= static_cast<int>(zones.size())) {
        pendingCompositeIdx_ = -1;
        return;
    }

    int zoneIdx = pendingCompositeIdx_;
    pendingCompositeIdx_ = -1;

    if (compositedIdx_ == zoneIdx) return;
    ensureTextureSlots(zones.size(), zones);

    const auto& zone = zones[zoneIdx];
    const auto& slots = zoneTextureSlots_[zoneIdx];
    uint32_t frameIdx = vkCtx->getCurrentFrame();
    VkDevice device = vkCtx->getDevice();

    // Update tile descriptor sets for this frame
    for (int i = 0; i < 12; i++) {
        VkTexture* tileTex = slots.tileTextures[i];
        if (!tileTex || !tileTex->isValid()) continue;

        VkDescriptorImageInfo imgInfo = tileTex->descriptorInfo();
        VkWriteDescriptorSet write{};
        write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        write.dstSet = tileDescSets[frameIdx][i];
        write.dstBinding = 0;
        write.descriptorCount = 1;
        write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        write.pImageInfo = &imgInfo;
        vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);
    }

    // Begin off-screen render pass
    VkClearColorValue clearColor = {{ 0.05f, 0.08f, 0.12f, 1.0f }};
    compositeTarget->beginPass(cmd, clearColor);

    VkDeviceSize offset = 0;
    vkCmdBindVertexBuffers(cmd, 0, 1, &quadVB, &offset);

    // --- Pass 1: Draw base map tiles (opaque) ---
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, tilePipeline);

    for (int i = 0; i < 12; i++) {
        if (!slots.tileTextures[i] || !slots.tileTextures[i]->isValid()) continue;

        int col = i % GRID_COLS;
        int row = i / GRID_COLS;

        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                tilePipelineLayout, 0, 1,
                                &tileDescSets[frameIdx][i], 0, nullptr);

        WorldMapTilePush push{};
        push.gridOffset = glm::vec2(static_cast<float>(col), static_cast<float>(row));
        push.gridCols = static_cast<float>(GRID_COLS);
        push.gridRows = static_cast<float>(GRID_ROWS);
        vkCmdPushConstants(cmd, tilePipelineLayout, VK_SHADER_STAGE_VERTEX_BIT,
                           0, sizeof(push), &push);

        vkCmdDraw(cmd, 6, 1, 0, 0);
    }

    // --- Draw explored overlay textures on top of the base map ---
    bool hasOverlays = !zone.overlays.empty() && zone.areaID != 0;
    if (hasOverlays && overlayPipeline_) {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, overlayPipeline_);

        uint32_t descSlot = 0;
        for (int oi = 0; oi < static_cast<int>(zone.overlays.size()); oi++) {
            if (exploredOverlays.count(oi) == 0) continue;
            const auto& ov = zone.overlays[oi];
            const auto& ovSlots = slots.overlays[oi];

            for (int t = 0; t < static_cast<int>(ovSlots.tiles.size()); t++) {
                if (!ovSlots.tiles[t] || !ovSlots.tiles[t]->isValid()) continue;
                if (descSlot >= MAX_OVERLAY_TILES) break;

                VkDescriptorImageInfo ovImgInfo = ovSlots.tiles[t]->descriptorInfo();
                VkWriteDescriptorSet ovWrite{};
                ovWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                ovWrite.dstSet = overlayDescSets_[frameIdx][descSlot];
                ovWrite.dstBinding = 0;
                ovWrite.descriptorCount = 1;
                ovWrite.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                ovWrite.pImageInfo = &ovImgInfo;
                vkUpdateDescriptorSets(device, 1, &ovWrite, 0, nullptr);

                vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                        overlayPipelineLayout_, 0, 1,
                                        &overlayDescSets_[frameIdx][descSlot], 0, nullptr);

                int tileCol = t % ov.tileCols;
                int tileRow = t / ov.tileCols;

                float px = static_cast<float>(ov.offsetX + tileCol * TILE_PX);
                float py = static_cast<float>(ov.offsetY + tileRow * TILE_PX);

                OverlayPush ovPush{};
                ovPush.gridOffset = glm::vec2(px / static_cast<float>(TILE_PX),
                                              py / static_cast<float>(TILE_PX));
                ovPush.gridCols = static_cast<float>(GRID_COLS);
                ovPush.gridRows = static_cast<float>(GRID_ROWS);
                ovPush.tintColor = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f);
                vkCmdPushConstants(cmd, overlayPipelineLayout_, VK_SHADER_STAGE_VERTEX_BIT,
                                   0, sizeof(WorldMapTilePush), &ovPush);
                vkCmdPushConstants(cmd, overlayPipelineLayout_, VK_SHADER_STAGE_FRAGMENT_BIT,
                                   16, sizeof(glm::vec4), &ovPush.tintColor);
                vkCmdDraw(cmd, 6, 1, 0, 0);

                descSlot++;
            }
        }
    }

    // --- Draw fog of war overlay over unexplored areas ---
    if (hasServerMask && zone.areaID != 0 && overlayPipeline_ && fogDescSet_) {
        bool hasAnyExplored = false;
        for (int oi = 0; oi < static_cast<int>(zone.overlays.size()); oi++) {
            if (exploredOverlays.count(oi) > 0) { hasAnyExplored = true; break; }
        }
        if (!hasAnyExplored && !zone.overlays.empty()) {
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, overlayPipeline_);
            vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                    overlayPipelineLayout_, 0, 1,
                                    &fogDescSet_, 0, nullptr);

            OverlayPush fogPush{};
            fogPush.gridOffset = glm::vec2(0.0f, 0.0f);
            fogPush.gridCols = 1.0f;
            fogPush.gridRows = 1.0f;
            fogPush.tintColor = glm::vec4(0.15f, 0.15f, 0.2f, 0.55f);
            vkCmdPushConstants(cmd, overlayPipelineLayout_, VK_SHADER_STAGE_VERTEX_BIT,
                               0, sizeof(WorldMapTilePush), &fogPush);
            vkCmdPushConstants(cmd, overlayPipelineLayout_, VK_SHADER_STAGE_FRAGMENT_BIT,
                               16, sizeof(glm::vec4), &fogPush.tintColor);
            vkCmdDraw(cmd, 6, 1, 0, 0);
        }
    }

    compositeTarget->endPass(cmd);
    compositedIdx_ = zoneIdx;
    everComposited_ = true;
}

} // namespace world_map
} // namespace rendering
} // namespace wowee
