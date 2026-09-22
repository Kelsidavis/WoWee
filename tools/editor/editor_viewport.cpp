#include "editor_viewport.hpp"
#include "rendering/volumetric_fog.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/vk_texture.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/m2_loader.hpp"
#include "pipeline/wmo_loader.hpp"
#include "pipeline/wowee_model.hpp"
#include "pipeline/wowee_building.hpp"
#include "core/logger.hpp"
#include <cstring>
#include <cmath>
#include <unordered_map>
#include <glm/gtc/matrix_transform.hpp>

namespace wowee {
namespace editor {

EditorViewport::EditorViewport() = default;
EditorViewport::~EditorViewport() { shutdown(); }

bool EditorViewport::initialize(rendering::VkContext* ctx, pipeline::AssetManager* am,
                                 rendering::Camera* cam) {
    vkCtx_ = ctx;
    assetManager_ = am;
    camera_ = cam;

    if (!createPerFrameResources()) return false;

    terrainRenderer_ = std::make_unique<rendering::TerrainRenderer>();
    if (!terrainRenderer_->initialize(ctx, perFrameSetLayout_, am)) {
        LOG_ERROR("Failed to initialize terrain renderer");
        return false;
    }
    terrainRenderer_->setFogEnabled(false);

    m2Renderer_ = std::make_unique<rendering::M2Renderer>();
    if (!m2Renderer_->initialize(ctx, perFrameSetLayout_, am)) {
        LOG_WARNING("M2 renderer init failed - object rendering disabled");
        m2Renderer_.reset();
    } else {
        m2Renderer_->setForceNoCull(true);
    }

    wmoRenderer_ = std::make_unique<rendering::WMORenderer>();
    if (!wmoRenderer_->initialize(ctx, perFrameSetLayout_, am)) {
        LOG_WARNING("WMO renderer init failed - building rendering disabled");
        wmoRenderer_.reset();
    }

    waterRenderer_.initialize(ctx, ctx->getImGuiRenderPass(), perFrameSetLayout_);
    gizmo_.initialize(ctx, ctx->getImGuiRenderPass(), perFrameSetLayout_);

    LOG_INFO("Editor viewport initialized");
    return true;
}

void EditorViewport::shutdown() {
    if (!vkCtx_) return;
    vkDeviceWaitIdle(vkCtx_->getDevice());

    if (npcMarkerVB_) { vmaDestroyBuffer(vkCtx_->getAllocator(), npcMarkerVB_, npcMarkerVBAlloc_); npcMarkerVB_ = VK_NULL_HANDLE; }
    if (brushVB_) { vmaDestroyBuffer(vkCtx_->getAllocator(), brushVB_, brushVBAlloc_); brushVB_ = VK_NULL_HANDLE; }
    if (pathVB_) { vmaDestroyBuffer(vkCtx_->getAllocator(), pathVB_, pathVBAlloc_); pathVB_ = VK_NULL_HANDLE; }
    if (patrolVB_) { vmaDestroyBuffer(vkCtx_->getAllocator(), patrolVB_, patrolVBAlloc_); patrolVB_ = VK_NULL_HANDLE; }
    gizmo_.shutdown();
    waterRenderer_.shutdown();

    if (wmoRenderer_) { wmoRenderer_->shutdown(); wmoRenderer_.reset(); }
    if (m2Renderer_) { m2Renderer_->shutdown(); m2Renderer_.reset(); }
    if (terrainRenderer_) { terrainRenderer_->shutdown(); terrainRenderer_.reset(); }

    destroyPerFrameResources();
    vkCtx_ = nullptr;
}

bool EditorViewport::loadTerrain(const pipeline::TerrainMesh& mesh,
                                  const std::vector<std::string>& texturePaths,
                                  int tileX, int tileY) {
    return terrainRenderer_->loadTerrain(mesh, texturePaths, tileX, tileY);
}

void EditorViewport::clearTerrain() {
    if (terrainRenderer_) terrainRenderer_->clear();
    // Loading a different zone invalidates the cached models; flush them so
    // their slots can be reused without leaking GPU memory across zones.
    persistentM2ModelIds_.clear();
    persistentWMOModelIds_.clear();
    nextPersistentModelId_ = 1;
    if (m2Renderer_) m2Renderer_->clear();
    if (wmoRenderer_) wmoRenderer_->clearAll();
}

void EditorViewport::updateWater(const pipeline::ADTTerrain& terrain, int tileX, int tileY) {
    waterRenderer_.update(terrain, tileX, tileY);
}

void EditorViewport::updateMarkers(const std::vector<PlacedObject>& /*objects*/) {
}

void EditorViewport::placeM2(const std::string& path, const glm::vec3& pos,
                              const glm::vec3& rot, float scale) {
    (void)path; (void)pos; (void)rot; (void)scale;
}

void EditorViewport::placeWMO(const std::string& path, const glm::vec3& pos,
                               const glm::vec3& rot) {
    (void)path; (void)pos; (void)rot;
}

void EditorViewport::clearObjects() {
    // Clear ghost state since the M2 renderer is about to be wiped
    ghostActive_ = false;
    ghostInstanceId_ = 0;
    ghostModelId_ = 0;
    ghostModelPath_.clear();

    // Drop instances but keep models cached on the GPU. The editor's rebuild
    // path destroys-and-recreates instances every time the placement set
    // changes; preserving model GPU buffers makes that path much cheaper for
    // large NPC populations using shared models.
    if (m2Renderer_) {
        vkCtx_->waitAllUploads();
        m2Renderer_->clearInstances();
    }
    if (wmoRenderer_) {
        wmoRenderer_->clearInstances();
    }
}

void EditorViewport::rebuildObjects(const std::vector<PlacedObject>& objects,
                                     const std::vector<CreatureSpawn>& npcs) {
    clearObjects();
    if (objects.empty() && npcs.empty()) return;

    // Don't call beginUploadBatch here - loadModel starts its own batch.
    // Use the persistent model-id maps so models stay cached across rebuilds.
    auto& m2ModelIds = persistentM2ModelIds_;
    auto& wmoModelIds = persistentWMOModelIds_;
    uint32_t& nextModelId = nextPersistentModelId_;

    for (const auto& obj : objects) {
        if (obj.type == PlaceableType::M2 && m2Renderer_) {
            uint32_t modelId;
            auto it = m2ModelIds.find(obj.path);
            if (it != m2ModelIds.end()) {
                modelId = it->second;
            } else {
                pipeline::M2Model model;
                bool loaded = false;

                // Try WOM open format first (replaces proprietary M2 when available).
                // Per-zone WOM directories shadow the global custom_zones folder.
                std::vector<std::string> womExtra;
                if (!activeMapName_.empty()) {
                    womExtra.push_back("output/" + activeMapName_ + "/models/");
                    womExtra.push_back("custom_zones/" + activeMapName_ + "/models/");
                }
                if (auto wom = pipeline::WoweeModelLoader::tryLoadByGamePath(obj.path, womExtra);
                    wom.isValid()) {
                    model = pipeline::WoweeModelLoader::toM2(wom);
                    loaded = true;
                }

                // Fall back to M2 from game data
                if (!loaded) {
                    auto data = assetManager_->readFile(obj.path);
                    if (data.empty()) continue;
                    model = pipeline::M2Loader::load(data);
                    // Always load skin (WotLK M2s need it for geometry)
                    {
                        std::string skinPath = pipeline::skinPathForM2(obj.path);
                        auto skinData = assetManager_->readFile(skinPath);
                        if (!skinData.empty())
                            pipeline::M2Loader::loadSkin(skinData, model);
                    }
                }

                if (!model.isValid()) continue;

                if (model.boundRadius < 1.0f) model.boundRadius = 50.0f;

                // Validate vertex data to prevent GPU crashes
                bool vertexOk = true;
                for (const auto& vert : model.vertices) {
                    if (!std::isfinite(vert.position.x) || !std::isfinite(vert.position.y) ||
                        !std::isfinite(vert.position.z) || std::abs(vert.position.x) > 100000.0f) {
                        vertexOk = false;
                        break;
                    }
                }
                if (!vertexOk) {
                    LOG_WARNING("M2 has invalid vertex data, skipping: ", obj.path);
                    continue;
                }

                modelId = nextModelId++;
                if (!m2Renderer_->loadModel(model, modelId)) {
                    LOG_WARNING("M2 failed to upload to GPU: ", obj.path);
                    continue;
                }
                LOG_INFO("M2 loaded: ", obj.path, " (modelId=", modelId, ", ",
                         model.vertices.size(), " verts)");
                m2ModelIds[obj.path] = modelId;
            }

        } else if (obj.type == PlaceableType::WMO && wmoRenderer_) {
            uint32_t modelId;
            auto it = wmoModelIds.find(obj.path);
            if (it != wmoModelIds.end()) {
                modelId = it->second;
            } else {
                pipeline::WMOModel model;
                bool loaded = false;

                // Try WOB open format first (replaces proprietary WMO when available)
                std::vector<std::string> wobExtra;
                if (!activeMapName_.empty()) {
                    wobExtra.push_back("output/" + activeMapName_ + "/buildings/");
                    wobExtra.push_back("custom_zones/" + activeMapName_ + "/buildings/");
                }
                if (auto wob = pipeline::WoweeBuildingLoader::tryLoadByGamePath(obj.path, wobExtra);
                    wob.isValid() &&
                    pipeline::WoweeBuildingLoader::toWMOModel(wob, model)) {
                    loaded = true;
                }

                if (!loaded) {
                    auto data = assetManager_->readFile(obj.path);
                    if (data.empty()) {
                        LOG_WARNING("WMO file not found in manifest: ", obj.path);
                        continue;
                    }
                    model = pipeline::WMOLoader::load(data);

                    // Load WMO group files (_000.wmo, _001.wmo, etc.)
                    std::string basePath = obj.path;
                    auto dotPos = basePath.rfind('.');
                    if (dotPos != std::string::npos) basePath = basePath.substr(0, dotPos);
                    for (uint32_t gi = 0; gi < model.nGroups; gi++) {
                        char groupSuffix[16];
                        std::snprintf(groupSuffix, sizeof(groupSuffix), "_%03u.wmo", gi);
                        std::string groupPath = basePath + groupSuffix;
                        auto groupData = assetManager_->readFile(groupPath);
                        if (!groupData.empty()) {
                            pipeline::WMOLoader::loadGroup(groupData, model, gi);
                        }
                    }
                }

                if (!model.isValid()) {
                    LOG_WARNING("WMO failed to parse (groups expected: ",
                                model.nGroups, "): ", obj.path);
                    continue;
                }

                modelId = nextModelId++;
                if (!wmoRenderer_->loadModel(model, modelId)) {
                    LOG_WARNING("WMO failed to upload to GPU: ", obj.path);
                    continue;
                }
                LOG_INFO("WMO loaded: ", obj.path, " (modelId=", modelId, ", ",
                         model.groups.size(), " groups)");
                wmoModelIds[obj.path] = modelId;
            }
            glm::vec3 wmoRotRad = glm::radians(obj.rotation);
            // Pass through obj.scale so non-1.0 WMO instance scales (loaded
            // from MODF, edited via the gizmo, or duplicated) actually render
            // at the right size instead of always 1.0.
            wmoRenderer_->createInstance(modelId, obj.position, wmoRotRad, obj.scale);
        }
    }

    // Render NPC creatures as M2 instances
    if (m2Renderer_ && !npcs.empty()) {
        for (const auto& npc : npcs) {
            if (npc.modelPath.empty()) continue;
            uint32_t modelId;
            auto it = m2ModelIds.find(npc.modelPath);
            if (it != m2ModelIds.end()) {
                modelId = it->second;
            } else {
                // Try WOM open format first (replaces proprietary M2 when available)
                pipeline::M2Model model;
                bool loaded = false;
                std::vector<std::string> npcWomExtra;
                if (!activeMapName_.empty()) {
                    npcWomExtra.push_back("output/" + activeMapName_ + "/models/");
                    npcWomExtra.push_back("custom_zones/" + activeMapName_ + "/models/");
                }
                if (auto wom = pipeline::WoweeModelLoader::tryLoadByGamePath(
                        npc.modelPath, npcWomExtra);
                    wom.isValid()) {
                    model = pipeline::WoweeModelLoader::toM2(wom);
                    loaded = true;
                }

                // Fall back to M2 from game data
                if (!loaded) {
                    auto data = assetManager_->readFile(npc.modelPath);
                    if (data.empty()) {
                        LOG_WARNING("NPC model file not found: ", npc.modelPath);
                        continue;
                    }
                    model = pipeline::M2Loader::load(data);
                    {
                        std::string skinPath = pipeline::skinPathForM2(npc.modelPath);
                        auto skinData = assetManager_->readFile(skinPath);
                        if (!skinData.empty())
                            pipeline::M2Loader::loadSkin(skinData, model);
                    }
                }
                if (!model.isValid()) {
                    LOG_WARNING("NPC model invalid: ", npc.modelPath,
                             " (verts=", model.vertices.size(), " idx=", model.indices.size(), ")");
                    continue;
                }
                LOG_DEBUG("NPC M2 OK: ", npc.modelPath, " (",
                         model.vertices.size(), "v ", model.indices.size(), "i ",
                         model.batches.size(), "b)");
                if (model.boundRadius < 1.0f) model.boundRadius = 50.0f;
                // Validate vertex data
                bool ok = true;
                for (const auto& vert : model.vertices) {
                    if (!std::isfinite(vert.position.x) || std::abs(vert.position.x) > 100000.0f) {
                        ok = false; break;
                    }
                }
                if (!ok) { LOG_WARNING("NPC M2 bad vertices: ", npc.modelPath); continue; }
                modelId = nextModelId++;
                if (!m2Renderer_->loadModel(model, modelId)) {
                    LOG_WARNING("NPC M2 loadModel failed: ", npc.modelPath,
                               " (", model.vertices.size(), "v ", model.indices.size(), "i ",
                               model.batches.size(), "b)");
                    continue;
                }
                m2ModelIds[npc.modelPath] = modelId;
            }
        }
    }

    // Finalize all GPU uploads BEFORE creating instances
    // (vertex buffers must be valid for isValid() check in createInstance)
    vkCtx_->waitAllUploads();
    vkCtx_->pollUploadBatches();

    // Now create instances (vertex buffers are finalized)
    for (const auto& obj : objects) {
        if (obj.type == PlaceableType::M2) {
            auto it = m2ModelIds.find(obj.path);
            if (it == m2ModelIds.end()) continue;
            glm::vec3 rotRad = glm::radians(obj.rotation);
            m2Renderer_->createInstance(it->second, obj.position, rotRad, obj.scale);
        }
    }
    for (const auto& npc : npcs) {
        auto it = m2ModelIds.find(npc.modelPath);
        if (it == m2ModelIds.end()) continue;
        glm::vec3 rotRad = glm::radians(glm::vec3(0, 0, npc.orientation));
        m2Renderer_->createInstance(it->second, npc.position, rotRad, npc.scale);
    }

    // Update NPC markers via dedicated method
    updateNpcMarkers(npcs);
}

void EditorViewport::setBrushIndicator(const glm::vec3& center, float radius, bool active) {
    brushVisible_ = active;
    if (!active) return;

    // Rebuild circle vertex buffer
    if (brushVB_) {
        vmaDestroyBuffer(vkCtx_->getAllocator(), brushVB_, brushVBAlloc_);
        brushVB_ = VK_NULL_HANDLE;
    }

    constexpr int SEGMENTS = 48;
    struct BV { float pos[3]; float color[4]; };
    std::vector<BV> verts;

    for (int i = 0; i < SEGMENTS; i++) {
        float a0 = static_cast<float>(i) / SEGMENTS * 6.2831853f;
        float a1 = static_cast<float>(i + 1) / SEGMENTS * 6.2831853f;
        float x0 = center.x + std::cos(a0) * radius;
        float y0 = center.y + std::sin(a0) * radius;
        float x1 = center.x + std::cos(a1) * radius;
        float y1 = center.y + std::sin(a1) * radius;
        float z = center.z + 1.0f; // slightly above terrain

        float w = 0.6f; // line width via thin quad
        float dx0 = std::cos(a0), dy0 = std::sin(a0);
        float dx1 = std::cos(a1), dy1 = std::sin(a1);

        BV v;
        v.color[0] = 1.0f; v.color[1] = 1.0f; v.color[2] = 0.3f; v.color[3] = 0.7f;

        // Thin quad for each segment
        v.pos[0] = x0 - dy0*w; v.pos[1] = y0 + dx0*w; v.pos[2] = z; verts.push_back(v);
        v.pos[0] = x0 + dy0*w; v.pos[1] = y0 - dx0*w; v.pos[2] = z; verts.push_back(v);
        v.pos[0] = x1 - dy1*w; v.pos[1] = y1 + dx1*w; v.pos[2] = z; verts.push_back(v);

        v.pos[0] = x1 - dy1*w; v.pos[1] = y1 + dx1*w; v.pos[2] = z; verts.push_back(v);
        v.pos[0] = x0 + dy0*w; v.pos[1] = y0 - dx0*w; v.pos[2] = z; verts.push_back(v);
        v.pos[0] = x1 + dy1*w; v.pos[1] = y1 - dx1*w; v.pos[2] = z; verts.push_back(v);
    }

    brushVertCount_ = static_cast<uint32_t>(verts.size());
    VkBufferCreateInfo bufInfo{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    bufInfo.size = verts.size() * sizeof(BV);
    bufInfo.usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
    VmaAllocationCreateInfo allocInfo{};
    allocInfo.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
    allocInfo.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
    VmaAllocationInfo mapInfo{};
    if (vmaCreateBuffer(vkCtx_->getAllocator(), &bufInfo, &allocInfo,
            &brushVB_, &brushVBAlloc_, &mapInfo) == VK_SUCCESS) {
        std::memcpy(mapInfo.pMappedData, verts.data(), verts.size() * sizeof(BV));
    }
}

void EditorViewport::setPathPreview(const glm::vec3& start, const glm::vec3& end,
                                     float width, bool visible) {
    pathVisible_ = visible;
    if (pathVB_) {
        vmaDestroyBuffer(vkCtx_->getAllocator(), pathVB_, pathVBAlloc_);
        pathVB_ = VK_NULL_HANDLE;
        pathVertCount_ = 0;
    }
    if (!visible) return;

    struct BV { float pos[3]; float color[4]; };
    std::vector<BV> verts;

    glm::vec2 delta(end.x - start.x, end.y - start.y);
    float dlen = glm::length(delta);
    // start == end would produce NaN dir/perp from glm::normalize and then
    // NaN positions in the path ribbon - Vulkan would either drop the draw
    // or crash on validation. Hide the preview instead.
    if (dlen < 1e-4f) { pathVisible_ = false; return; }
    glm::vec2 dir = delta / dlen;
    glm::vec2 perp(-dir.y, dir.x);
    float z0 = start.z + 2.0f;
    float z1 = end.z + 2.0f;
    float hw = width * 0.5f;

    // Path ribbon (semi-transparent)
    BV v;
    v.color[0] = 0.3f; v.color[1] = 0.6f; v.color[2] = 1.0f; v.color[3] = 0.35f;
    v.pos[0] = start.x - perp.x*hw; v.pos[1] = start.y - perp.y*hw; v.pos[2] = z0; verts.push_back(v);
    v.pos[0] = start.x + perp.x*hw; v.pos[1] = start.y + perp.y*hw; v.pos[2] = z0; verts.push_back(v);
    v.pos[0] = end.x - perp.x*hw;   v.pos[1] = end.y - perp.y*hw;   v.pos[2] = z1; verts.push_back(v);
    v.pos[0] = end.x - perp.x*hw;   v.pos[1] = end.y - perp.y*hw;   v.pos[2] = z1; verts.push_back(v);
    v.pos[0] = start.x + perp.x*hw; v.pos[1] = start.y + perp.y*hw; v.pos[2] = z0; verts.push_back(v);
    v.pos[0] = end.x + perp.x*hw;   v.pos[1] = end.y + perp.y*hw;   v.pos[2] = z1; verts.push_back(v);

    // Edge lines (brighter)
    float lw = 0.8f;
    v.color[0] = 0.4f; v.color[1] = 0.8f; v.color[2] = 1.0f; v.color[3] = 0.8f;
    for (int side = -1; side <= 1; side += 2) {
        float s = static_cast<float>(side);
        glm::vec2 offset = perp * hw * s;
        glm::vec2 linePerp = perp * lw * s;
        v.pos[0] = start.x + offset.x - linePerp.x; v.pos[1] = start.y + offset.y - linePerp.y; v.pos[2] = z0; verts.push_back(v);
        v.pos[0] = start.x + offset.x + linePerp.x; v.pos[1] = start.y + offset.y + linePerp.y; v.pos[2] = z0; verts.push_back(v);
        v.pos[0] = end.x + offset.x - linePerp.x;   v.pos[1] = end.y + offset.y - linePerp.y;   v.pos[2] = z1; verts.push_back(v);
        v.pos[0] = end.x + offset.x - linePerp.x;   v.pos[1] = end.y + offset.y - linePerp.y;   v.pos[2] = z1; verts.push_back(v);
        v.pos[0] = start.x + offset.x + linePerp.x; v.pos[1] = start.y + offset.y + linePerp.y; v.pos[2] = z0; verts.push_back(v);
        v.pos[0] = end.x + offset.x + linePerp.x;   v.pos[1] = end.y + offset.y + linePerp.y;   v.pos[2] = z1; verts.push_back(v);
    }

    pathVertCount_ = static_cast<uint32_t>(verts.size());
    VkBufferCreateInfo bufInfo{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    bufInfo.size = verts.size() * sizeof(BV);
    bufInfo.usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
    VmaAllocationCreateInfo allocInfo{};
    allocInfo.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
    allocInfo.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
    VmaAllocationInfo mapInfo{};
    if (vmaCreateBuffer(vkCtx_->getAllocator(), &bufInfo, &allocInfo,
            &pathVB_, &pathVBAlloc_, &mapInfo) == VK_SUCCESS) {
        std::memcpy(mapInfo.pMappedData, verts.data(), verts.size() * sizeof(BV));
    }
}

void EditorViewport::setPatrolPath(const std::vector<glm::vec3>& points, float width) {
    if (patrolVB_) {
        vmaDestroyBuffer(vkCtx_->getAllocator(), patrolVB_, patrolVBAlloc_);
        patrolVB_ = VK_NULL_HANDLE;
        patrolVertCount_ = 0;
    }
    if (points.size() < 2) return;

    struct BV { float pos[3]; float color[4]; };
    std::vector<BV> verts;
    verts.reserve(points.size() * 24);

    auto addRibbon = [&](const glm::vec3& a, const glm::vec3& b, float r, float g, float bl, float al) {
        // NaN endpoints would short-circuit the len < 0.001f check (NaN
        // comparisons return false) and propagate NaN through dir.
        if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(a.z) ||
            !std::isfinite(b.x) || !std::isfinite(b.y) || !std::isfinite(b.z)) return;
        glm::vec2 dir = glm::vec2(b.x - a.x, b.y - a.y);
        float len = glm::length(dir);
        if (!std::isfinite(len) || len < 0.001f) return;
        dir /= len;
        glm::vec2 perp(-dir.y, dir.x);
        float hw = width * 0.5f;
        float z0 = a.z + 1.5f;
        float z1 = b.z + 1.5f;
        BV v;
        v.color[0] = r; v.color[1] = g; v.color[2] = bl; v.color[3] = al;
        v.pos[0] = a.x - perp.x*hw; v.pos[1] = a.y - perp.y*hw; v.pos[2] = z0; verts.push_back(v);
        v.pos[0] = a.x + perp.x*hw; v.pos[1] = a.y + perp.y*hw; v.pos[2] = z0; verts.push_back(v);
        v.pos[0] = b.x - perp.x*hw; v.pos[1] = b.y - perp.y*hw; v.pos[2] = z1; verts.push_back(v);
        v.pos[0] = b.x - perp.x*hw; v.pos[1] = b.y - perp.y*hw; v.pos[2] = z1; verts.push_back(v);
        v.pos[0] = a.x + perp.x*hw; v.pos[1] = a.y + perp.y*hw; v.pos[2] = z0; verts.push_back(v);
        v.pos[0] = b.x + perp.x*hw; v.pos[1] = b.y + perp.y*hw; v.pos[2] = z1; verts.push_back(v);
    };

    auto addWaypoint = [&](const glm::vec3& p, float r, float g, float bl) {
        float s = 1.5f;
        BV v;
        v.color[0] = r; v.color[1] = g; v.color[2] = bl; v.color[3] = 0.95f;
        glm::vec3 top(p.x, p.y, p.z + s * 2);
        glm::vec3 bot(p.x, p.y, p.z + 0.2f);
        glm::vec3 n(p.x, p.y + s, p.z + s);
        glm::vec3 s2(p.x, p.y - s, p.z + s);
        glm::vec3 e(p.x + s, p.y, p.z + s);
        glm::vec3 w(p.x - s, p.y, p.z + s);
        auto pushV = [&](const glm::vec3& vv){ v.pos[0]=vv.x; v.pos[1]=vv.y; v.pos[2]=vv.z; verts.push_back(v); };
        pushV(top); pushV(n); pushV(e);
        pushV(top); pushV(e); pushV(s2);
        pushV(top); pushV(s2); pushV(w);
        pushV(top); pushV(w); pushV(n);
        pushV(bot); pushV(e); pushV(n);
        pushV(bot); pushV(s2); pushV(e);
        pushV(bot); pushV(w); pushV(s2);
        pushV(bot); pushV(n); pushV(w);
    };

    // Ribbons fade from bright orange at the start to dim orange at the end
    // so direction of travel is visually obvious.
    for (size_t i = 0; i + 1 < points.size(); i++) {
        float t = points.size() > 1 ? static_cast<float>(i) / (points.size() - 1) : 0.0f;
        float bright = 1.0f - t * 0.5f;
        addRibbon(points[i], points[i+1], bright, 0.7f * bright, 0.2f * bright, 0.55f);
    }
    for (size_t i = 0; i < points.size(); i++) {
        // Start (NPC home) green, intermediate yellow→orange, last red.
        if (i == 0) {
            addWaypoint(points[i], 0.2f, 1.0f, 0.3f);
        } else if (i == points.size() - 1 && points.size() >= 2) {
            addWaypoint(points[i], 1.0f, 0.3f, 0.2f);
        } else {
            float t = points.size() > 1 ? static_cast<float>(i) / (points.size() - 1) : 0.0f;
            addWaypoint(points[i], 1.0f, 1.0f - t * 0.6f, 0.2f);
        }
    }

    patrolVertCount_ = static_cast<uint32_t>(verts.size());
    VkBufferCreateInfo bufInfo{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    bufInfo.size = verts.size() * sizeof(BV);
    bufInfo.usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
    VmaAllocationCreateInfo allocInfo{};
    allocInfo.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
    allocInfo.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
    VmaAllocationInfo mapInfo{};
    if (vmaCreateBuffer(vkCtx_->getAllocator(), &bufInfo, &allocInfo,
            &patrolVB_, &patrolVBAlloc_, &mapInfo) == VK_SUCCESS) {
        std::memcpy(mapInfo.pMappedData, verts.data(), verts.size() * sizeof(BV));
    }
}

void EditorViewport::updateNpcMarkers(const std::vector<CreatureSpawn>& npcs) {
    if (npcMarkerVB_) {
        vmaDestroyBuffer(vkCtx_->getAllocator(), npcMarkerVB_, npcMarkerVBAlloc_);
        npcMarkerVB_ = VK_NULL_HANDLE;
        npcMarkerVertCount_ = 0;
    }
    if (npcs.empty()) return;

    struct MV { float pos[3]; float color[4]; };
    std::vector<MV> verts;
    for (const auto& npc : npcs) {
        // Skip NPCs with non-finite position - would produce NaN vertices
        // in the marker mesh (Vulkan validation drops the whole batch).
        if (!std::isfinite(npc.position.x) || !std::isfinite(npc.position.y) ||
            !std::isfinite(npc.position.z)) continue;
        // Selected NPC: larger marker in cyan-yellow so it pops out among
        // hostile/friendly markers without losing the hostile colour signal.
        float s = npc.selected ? 2.5f : 1.5f;
        float x = npc.position.x, y = npc.position.y, z = npc.position.z;
        float r = npc.selected ? 1.0f : (npc.hostile ? 1.0f : 0.1f);
        float g = npc.selected ? 1.0f : (npc.hostile ? 0.15f : 0.9f);
        float b = npc.selected ? 0.2f : 0.1f;
        float a = npc.selected ? 1.0f : 0.7f;

        MV v; v.color[0]=r; v.color[1]=g; v.color[2]=b; v.color[3]=a;
        // Small octagonal base
        for (int seg = 0; seg < 8; seg++) {
            float a0 = seg * 0.7854f, a1 = (seg+1) * 0.7854f;
            v.pos[0]=x; v.pos[1]=y; v.pos[2]=z+0.2f; verts.push_back(v);
            v.pos[0]=x+std::cos(a0)*s; v.pos[1]=y+std::sin(a0)*s; v.pos[2]=z+0.2f; verts.push_back(v);
            v.pos[0]=x+std::cos(a1)*s; v.pos[1]=y+std::sin(a1)*s; v.pos[2]=z+0.2f; verts.push_back(v);
        }
        // Thin pole
        float pw = 0.3f, ph = 8.0f;  // was 0.8 wide, 30 tall
        v.color[3] = 0.6f;
        v.pos[0]=x-pw; v.pos[1]=y; v.pos[2]=z; verts.push_back(v);
        v.pos[0]=x+pw; v.pos[1]=y; v.pos[2]=z; verts.push_back(v);
        v.pos[0]=x; v.pos[1]=y; v.pos[2]=z+ph; verts.push_back(v);
        v.pos[0]=x; v.pos[1]=y-pw; v.pos[2]=z; verts.push_back(v);
        v.pos[0]=x; v.pos[1]=y+pw; v.pos[2]=z; verts.push_back(v);
        v.pos[0]=x; v.pos[1]=y; v.pos[2]=z+ph; verts.push_back(v);
        // Small diamond top
        float ts = 1.0f, tz = z + ph;  // was 3
        v.color[0]=1; v.color[1]=1; v.color[2]=0.3f; v.color[3]=0.8f;
        v.pos[0]=x+ts; v.pos[1]=y; v.pos[2]=tz; verts.push_back(v);
        v.pos[0]=x; v.pos[1]=y+ts; v.pos[2]=tz; verts.push_back(v);
        v.pos[0]=x-ts; v.pos[1]=y; v.pos[2]=tz; verts.push_back(v);
        v.pos[0]=x+ts; v.pos[1]=y; v.pos[2]=tz; verts.push_back(v);
        v.pos[0]=x-ts; v.pos[1]=y; v.pos[2]=tz; verts.push_back(v);
        v.pos[0]=x; v.pos[1]=y-ts; v.pos[2]=tz; verts.push_back(v);

        // Facing arrow on the ground: triangle pointing in the orientation direction.
        // Helps users see which way each NPC faces without selecting it.
        float yaw = glm::radians(npc.orientation);
        float fx = std::cos(yaw), fy = std::sin(yaw);
        float perpX = -fy, perpY = fx;
        float arrowLen = s * 2.5f, arrowHalfW = s * 0.8f;
        v.color[0]=1.0f; v.color[1]=0.9f; v.color[2]=0.2f; v.color[3]=0.85f;
        v.pos[0]=x + fx*arrowLen; v.pos[1]=y + fy*arrowLen; v.pos[2]=z+0.25f; verts.push_back(v);
        v.pos[0]=x + perpX*arrowHalfW; v.pos[1]=y + perpY*arrowHalfW; v.pos[2]=z+0.25f; verts.push_back(v);
        v.pos[0]=x - perpX*arrowHalfW; v.pos[1]=y - perpY*arrowHalfW; v.pos[2]=z+0.25f; verts.push_back(v);
    }
    npcMarkerVertCount_ = static_cast<uint32_t>(verts.size());
    VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    bi.size = verts.size() * sizeof(MV);
    bi.usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
    VmaAllocationCreateInfo ai{}; ai.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
    ai.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
    VmaAllocationInfo mi{};
    if (vmaCreateBuffer(vkCtx_->getAllocator(), &bi, &ai,
            &npcMarkerVB_, &npcMarkerVBAlloc_, &mi) == VK_SUCCESS)
        std::memcpy(mi.pMappedData, verts.data(), verts.size() * sizeof(MV));
}

void EditorViewport::update(float deltaTime) {
    if (m2Renderer_)
        m2Renderer_->update(deltaTime, camera_->getPosition(), camera_->getViewProjectionMatrix());
}

void EditorViewport::setGhostPreview(const std::string& path, const glm::vec3& pos,
                                      const glm::vec3& rotDeg, float scale) {
    if (!m2Renderer_) return;
    // Reject NaN inputs - would propagate into the M2 renderer transform
    // and either crash on the GPU or silently render at the origin.
    if (!std::isfinite(pos.x) || !std::isfinite(pos.y) || !std::isfinite(pos.z) ||
        !std::isfinite(rotDeg.x) || !std::isfinite(rotDeg.y) || !std::isfinite(rotDeg.z) ||
        !std::isfinite(scale) || scale <= 0.0f) {
        clearGhostPreview();
        return;
    }

    // Load model if path changed
    if (path != ghostModelPath_ || ghostModelId_ == 0) {
        clearGhostPreview();
        auto data = assetManager_->readFile(path);
        if (data.empty()) { LOG_WARNING("Ghost: file not found: ", path); return; }
        auto model = pipeline::M2Loader::load(data);
        if (!model.isValid()) {
            std::string skinPath = pipeline::skinPathForM2(path);
            auto skinData = assetManager_->readFile(skinPath);
            if (!skinData.empty())
                pipeline::M2Loader::loadSkin(skinData, model);
        }
        if (!model.isValid()) return;
        if (model.boundRadius < 1.0f) model.boundRadius = 50.0f;

        ghostModelId_ = 59999; // High ID to avoid collision with placed objects
        if (!m2Renderer_->loadModel(model, ghostModelId_)) {
            ghostModelId_ = 0;
            return;
        }
        vkCtx_->waitAllUploads();
        vkCtx_->pollUploadBatches();
        ghostModelPath_ = path;
    }

    // Create or update ghost instance
    glm::vec3 rotRad = glm::radians(rotDeg);
    if (!ghostActive_) {
        ghostInstanceId_ = m2Renderer_->createInstance(ghostModelId_, pos, rotRad, scale);
        ghostActive_ = (ghostInstanceId_ != 0);
    } else {
        m2Renderer_->setInstancePosition(ghostInstanceId_, pos);
        // Rebuild transform with new rotation/scale
        glm::mat4 mat = glm::mat4(1.0f);
        mat = glm::translate(mat, pos);
        mat = glm::rotate(mat, rotRad.x, glm::vec3(1, 0, 0));
        mat = glm::rotate(mat, rotRad.y, glm::vec3(0, 1, 0));
        mat = glm::rotate(mat, rotRad.z, glm::vec3(0, 0, 1));
        mat = glm::scale(mat, glm::vec3(scale));
        m2Renderer_->setInstanceTransform(ghostInstanceId_, mat);
    }
}

void EditorViewport::clearGhostPreview() {
    if (ghostActive_ && m2Renderer_) {
        m2Renderer_->removeInstance(ghostInstanceId_);
        ghostActive_ = false;
        ghostInstanceId_ = 0;
    }
    if (ghostModelId_ != 0 && m2Renderer_) {
        // Ghost ID is reserved for previews only - safe to unload so a path
        // change can re-load with the new model under the same ID.
        m2Renderer_->unloadModel(ghostModelId_);
        ghostModelId_ = 0;
        ghostModelPath_.clear();
    }
}

void EditorViewport::render(VkCommandBuffer cmd) {
    updatePerFrameUBO();

    uint32_t frame = vkCtx_->getCurrentFrame();
    VkDescriptorSet perFrameSet = perFrameDescSets_[frame];

    terrainRenderer_->render(cmd, perFrameSet, *camera_);

    if (m2Renderer_) {
        m2Renderer_->prepareRender(frame, *camera_);
        m2Renderer_->render(cmd, perFrameSet, *camera_);
    }
    if (wmoRenderer_) {
        wmoRenderer_->prepareRender();
        wmoRenderer_->render(cmd, perFrameSet, *camera_);
    }

    waterRenderer_.render(cmd, perFrameSet);

    // NPC position markers - render AFTER gizmo (no depth test = always on top)

    // Brush indicator circle
    if (brushVisible_ && brushVB_ && brushVertCount_ > 0) {
        // Reuse gizmo pipeline (same vertex format, no depth test, alpha blend)
        if (gizmo_.getMode() == TransformMode::None && !gizmo_.isActive()) {
            // Use water pipeline for brush (it has alpha blend + depth test)
            // Actually just render through the water pipeline
        }
        // Render brush circle using the water renderer's pipeline setup
        // (same pos+color vertex format)
        auto* waterPipeline = waterRenderer_.getPipeline();
        auto* waterLayout = waterRenderer_.getPipelineLayout();
        if (waterPipeline && waterLayout) {
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, waterPipeline);
            vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, waterLayout,
                                    0, 1, &perFrameSet, 0, nullptr);
            VkDeviceSize off = 0;
            vkCmdBindVertexBuffers(cmd, 0, 1, &brushVB_, &off);
            vkCmdDraw(cmd, brushVertCount_, 1, 0, 0);
        }
    }

    // Path preview line (river/road tool)
    if (pathVisible_ && pathVB_ && pathVertCount_ > 0) {
        auto* waterPipeline = waterRenderer_.getPipeline();
        auto* waterLayout = waterRenderer_.getPipelineLayout();
        if (waterPipeline && waterLayout) {
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, waterPipeline);
            vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, waterLayout,
                                    0, 1, &perFrameSet, 0, nullptr);
            VkDeviceSize off = 0;
            vkCmdBindVertexBuffers(cmd, 0, 1, &pathVB_, &off);
            vkCmdDraw(cmd, pathVertCount_, 1, 0, 0);
        }
    }

    // Patrol path ribbon for selected NPC
    if (patrolVB_ && patrolVertCount_ > 0) {
        auto* waterPipeline = waterRenderer_.getPipeline();
        auto* waterLayout = waterRenderer_.getPipelineLayout();
        if (waterPipeline && waterLayout) {
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, waterPipeline);
            vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, waterLayout,
                                    0, 1, &perFrameSet, 0, nullptr);
            VkDeviceSize off = 0;
            vkCmdBindVertexBuffers(cmd, 0, 1, &patrolVB_, &off);
            vkCmdDraw(cmd, patrolVertCount_, 1, 0, 0);
        }
    }

    gizmo_.render(cmd, perFrameSet);

    // NPC markers - render with water pipeline (pos+color, alpha blend)
    if (showNpcMarkers_ && npcMarkerVB_ && npcMarkerVertCount_ > 0) {
        auto* waterPipeline = waterRenderer_.getPipeline();
        auto* waterLayout = waterRenderer_.getPipelineLayout();
        if (waterPipeline && waterLayout) {
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, waterPipeline);
            vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, waterLayout,
                                    0, 1, &perFrameSet, 0, nullptr);
            VkDeviceSize off = 0;
            vkCmdBindVertexBuffers(cmd, 0, 1, &npcMarkerVB_, &off);
            vkCmdDraw(cmd, npcMarkerVertCount_, 1, 0, 0);
        }
    }
}

void EditorViewport::setWireframe(bool enabled) {
    wireframe_ = enabled;
    if (terrainRenderer_) terrainRenderer_->setWireframe(enabled);
}

bool EditorViewport::createPerFrameResources() {
    VkDevice device = vkCtx_->getDevice();

    // The world shaders read a fog volume at binding 2. The editor never
    // builds one, so every set binds the neutral volume and the block's
    // switch stays at zero.
    fogVolume_ = std::make_unique<rendering::VolumetricFog>();
    if (!fogVolume_->initialize(vkCtx_)) return false;
    const VkSampler fogSampler = fogVolume_->getSampler();

    VkDescriptorSetLayoutBinding bindings[3]{};
    bindings[0].binding = 0;
    bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    bindings[0].descriptorCount = 1;
    bindings[0].stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    bindings[1].binding = 1;
    bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    bindings[1].descriptorCount = 1;
    bindings[1].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    bindings[2].binding = 2;
    bindings[2].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    bindings[2].descriptorCount = 1;
    bindings[2].stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    bindings[2].pImmutableSamplers = &fogSampler;

    VkDescriptorSetLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutInfo.bindingCount = 3;
    layoutInfo.pBindings = bindings;

    if (vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr, &perFrameSetLayout_) != VK_SUCCESS)
        return false;

    VkDescriptorPoolSize poolSizes[2]{};
    poolSizes[0].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    poolSizes[0].descriptorCount = MAX_FRAMES;
    poolSizes[1].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    poolSizes[1].descriptorCount = MAX_FRAMES * 2;

    VkDescriptorPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.maxSets = MAX_FRAMES;
    poolInfo.poolSizeCount = 2;
    poolInfo.pPoolSizes = poolSizes;

    if (vkCreateDescriptorPool(device, &poolInfo, nullptr, &sceneDescPool_) != VK_SUCCESS)
        return false;

    dummyShadowTexture_ = std::make_unique<rendering::VkTexture>();
    if (!dummyShadowTexture_->createDepth(*vkCtx_, 1, 1)) return false;

    VkSamplerCreateInfo sampCI{};
    sampCI.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    sampCI.magFilter = VK_FILTER_LINEAR;
    sampCI.minFilter = VK_FILTER_LINEAR;
    sampCI.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
    sampCI.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    sampCI.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    sampCI.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    sampCI.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
    sampCI.compareEnable = VK_TRUE;
    sampCI.compareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
    shadowSampler_ = vkCtx_->getOrCreateSampler(sampCI);

    vkCtx_->immediateSubmit([this](VkCommandBuffer cmd) {
        VkImageMemoryBarrier barrier{};
        barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        barrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.image = dummyShadowTexture_->getImage();
        barrier.subresourceRange = {VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, 1};
        barrier.srcAccessMask = 0;
        barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        vkCmdPipelineBarrier(cmd,
            VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
            0, 0, nullptr, 0, nullptr, 1, &barrier);
    });

    for (uint32_t i = 0; i < MAX_FRAMES; i++) {
        VkBufferCreateInfo bufInfo{};
        bufInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufInfo.size = sizeof(rendering::GPUPerFrameData);
        bufInfo.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;

        VmaAllocationCreateInfo allocInfo{};
        allocInfo.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
        allocInfo.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;

        VmaAllocationInfo mapInfo{};
        if (vmaCreateBuffer(vkCtx_->getAllocator(), &bufInfo, &allocInfo,
                &perFrameUBOs_[i], &perFrameUBOAllocs_[i], &mapInfo) != VK_SUCCESS)
            return false;
        perFrameUBOMapped_[i] = mapInfo.pMappedData;

        VkDescriptorSetAllocateInfo setAlloc{};
        setAlloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        setAlloc.descriptorPool = sceneDescPool_;
        setAlloc.descriptorSetCount = 1;
        setAlloc.pSetLayouts = &perFrameSetLayout_;
        if (vkAllocateDescriptorSets(device, &setAlloc, &perFrameDescSets_[i]) != VK_SUCCESS)
            return false;

        VkDescriptorBufferInfo descBuf{};
        descBuf.buffer = perFrameUBOs_[i];
        descBuf.offset = 0;
        descBuf.range = sizeof(rendering::GPUPerFrameData);

        VkDescriptorImageInfo shadowImgInfo{};
        shadowImgInfo.sampler = shadowSampler_;
        shadowImgInfo.imageView = dummyShadowTexture_->getImageView();
        shadowImgInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

        VkDescriptorImageInfo fogImgInfo{};
        fogImgInfo.imageView = fogVolume_->getNeutralView();
        fogImgInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;

        VkWriteDescriptorSet writes[3]{};
        writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[0].dstSet = perFrameDescSets_[i];
        writes[0].dstBinding = 0;
        writes[0].descriptorCount = 1;
        writes[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        writes[0].pBufferInfo = &descBuf;
        writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[1].dstSet = perFrameDescSets_[i];
        writes[1].dstBinding = 1;
        writes[1].descriptorCount = 1;
        writes[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        writes[1].pImageInfo = &shadowImgInfo;
        writes[2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[2].dstSet = perFrameDescSets_[i];
        writes[2].dstBinding = 2;
        writes[2].descriptorCount = 1;
        writes[2].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        writes[2].pImageInfo = &fogImgInfo;

        vkUpdateDescriptorSets(device, 3, writes, 0, nullptr);
    }

    return true;
}

void EditorViewport::destroyPerFrameResources() {
    if (!vkCtx_) return;
    VkDevice device = vkCtx_->getDevice();

    for (uint32_t i = 0; i < MAX_FRAMES; i++) {
        if (perFrameUBOs_[i]) {
            vmaDestroyBuffer(vkCtx_->getAllocator(), perFrameUBOs_[i], perFrameUBOAllocs_[i]);
            perFrameUBOs_[i] = VK_NULL_HANDLE;
        }
    }
    if (dummyShadowTexture_) {
        dummyShadowTexture_->destroy(device, vkCtx_->getAllocator());
        dummyShadowTexture_.reset();
    }
    if (fogVolume_) {
        fogVolume_->shutdown();
        fogVolume_.reset();
    }
    if (sceneDescPool_) {
        vkDestroyDescriptorPool(device, sceneDescPool_, nullptr);
        sceneDescPool_ = VK_NULL_HANDLE;
    }
    if (perFrameSetLayout_) {
        vkDestroyDescriptorSetLayout(device, perFrameSetLayout_, nullptr);
        perFrameSetLayout_ = VK_NULL_HANDLE;
    }
}

void EditorViewport::setTimeOfDay(float t) {
    timeOfDay_ = std::clamp(t, 0.0f, 24.0f);
    float hour = timeOfDay_;

    // Sun angle: noon=overhead, 6am/6pm=horizon, night=below
    float sunAngle = (hour - 6.0f) / 12.0f * 3.14159f;
    lightDir_ = glm::normalize(glm::vec3(std::cos(sunAngle) * 0.5f, -1.0f, std::sin(sunAngle)));

    // Dawn/dusk warm tones, noon white, night blue
    if (hour >= 6.0f && hour <= 8.0f) {
        float t2 = (hour - 6.0f) / 2.0f;
        lightColor_ = glm::mix(glm::vec3(1.0f, 0.5f, 0.2f), glm::vec3(1.0f, 0.95f, 0.85f), t2);
        ambientColor_ = glm::mix(glm::vec3(0.15f, 0.1f, 0.2f), glm::vec3(0.3f, 0.3f, 0.35f), t2);
        fogColor_ = glm::mix(glm::vec3(0.5f, 0.3f, 0.3f), glm::vec3(0.6f, 0.7f, 0.8f), t2);
    } else if (hour >= 17.0f && hour <= 19.0f) {
        float t2 = (hour - 17.0f) / 2.0f;
        lightColor_ = glm::mix(glm::vec3(1.0f, 0.95f, 0.85f), glm::vec3(1.0f, 0.4f, 0.15f), t2);
        ambientColor_ = glm::mix(glm::vec3(0.3f, 0.3f, 0.35f), glm::vec3(0.1f, 0.08f, 0.15f), t2);
        fogColor_ = glm::mix(glm::vec3(0.6f, 0.7f, 0.8f), glm::vec3(0.4f, 0.25f, 0.3f), t2);
    } else if (hour < 6.0f || hour > 19.0f) {
        lightColor_ = glm::vec3(0.15f, 0.15f, 0.25f);
        ambientColor_ = glm::vec3(0.05f, 0.05f, 0.1f);
        fogColor_ = glm::vec3(0.1f, 0.1f, 0.15f);
    } else {
        lightColor_ = glm::vec3(1.0f, 0.95f, 0.85f);
        ambientColor_ = glm::vec3(0.3f, 0.3f, 0.35f);
        fogColor_ = glm::vec3(0.6f, 0.7f, 0.8f);
    }

    // Sky/clear color follows fog
    clearR_ = fogColor_.x * 0.7f;
    clearG_ = fogColor_.y * 0.7f;
    clearB_ = fogColor_.z * 0.7f;
}

void EditorViewport::updatePerFrameUBO() {
    uint32_t frame = vkCtx_->getCurrentFrame();

    rendering::GPUPerFrameData data{};
    data.view = camera_->getViewMatrix();
    data.projection = camera_->getProjectionMatrix();
    data.lightSpaceMatrix = glm::mat4(1.0f);
    data.lightDir = glm::vec4(lightDir_, 0.0f);
    data.lightColor = glm::vec4(lightColor_, 0.0f);
    data.ambientColor = glm::vec4(ambientColor_, 0.0f);
    data.viewPos = glm::vec4(camera_->getPosition(), 0.0f);
    data.fogColor = glm::vec4(fogColor_, 0.0f);
    data.fogParams = glm::vec4(fogNear_, fogFar_, 0.0f, 0.0f);
    data.shadowParams = glm::vec4(0.0f, 0.0f, 0.0f, 0.0f);

    std::memcpy(perFrameUBOMapped_[frame], &data, sizeof(data));
}

} // namespace editor
} // namespace wowee
