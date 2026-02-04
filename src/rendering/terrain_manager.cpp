#include "rendering/terrain_manager.hpp"
#include "rendering/terrain_renderer.hpp"
#include "rendering/water_renderer.hpp"
#include "rendering/m2_renderer.hpp"
#include "rendering/wmo_renderer.hpp"
#include "rendering/camera.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/adt_loader.hpp"
#include "pipeline/m2_loader.hpp"
#include "pipeline/wmo_loader.hpp"
#include "pipeline/terrain_mesh.hpp"
#include "core/logger.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <cmath>
#include <cctype>
#include <functional>
#include <unordered_set>

namespace wowee {
namespace rendering {

namespace {

bool decodeLayerAlpha(const pipeline::MapChunk& chunk, size_t layerIdx, std::vector<uint8_t>& outAlpha) {
    if (layerIdx >= chunk.layers.size()) return false;
    const auto& layer = chunk.layers[layerIdx];
    if (!layer.useAlpha() || layer.offsetMCAL >= chunk.alphaMap.size()) return false;

    size_t offset = layer.offsetMCAL;
    size_t layerSize = chunk.alphaMap.size() - offset;
    for (size_t j = layerIdx + 1; j < chunk.layers.size(); j++) {
        if (chunk.layers[j].useAlpha()) {
            layerSize = chunk.layers[j].offsetMCAL - offset;
            break;
        }
    }

    outAlpha.assign(4096, 255);

    if (layer.compressedAlpha()) {
        size_t readPos = offset;
        size_t writePos = 0;
        while (writePos < 4096 && readPos < chunk.alphaMap.size()) {
            uint8_t cmd = chunk.alphaMap[readPos++];
            bool fill = (cmd & 0x80) != 0;
            int count = (cmd & 0x7F) + 1;

            if (fill) {
                if (readPos >= chunk.alphaMap.size()) break;
                uint8_t val = chunk.alphaMap[readPos++];
                for (int i = 0; i < count && writePos < 4096; i++) {
                    outAlpha[writePos++] = val;
                }
            } else {
                for (int i = 0; i < count && writePos < 4096 && readPos < chunk.alphaMap.size(); i++) {
                    outAlpha[writePos++] = chunk.alphaMap[readPos++];
                }
            }
        }
        return true;
    }

    if (layerSize >= 4096) {
        std::copy(chunk.alphaMap.begin() + offset, chunk.alphaMap.begin() + offset + 4096, outAlpha.begin());
        return true;
    }

    if (layerSize >= 2048) {
        for (size_t i = 0; i < 2048; i++) {
            uint8_t v = chunk.alphaMap[offset + i];
            outAlpha[i * 2] = (v & 0x0F) * 17;
            outAlpha[i * 2 + 1] = (v >> 4) * 17;
        }
        return true;
    }

    return false;
}

} // namespace

TerrainManager::TerrainManager() {
}

TerrainManager::~TerrainManager() {
    // Stop worker thread before cleanup (containers clean up via destructors)
    if (workerRunning.load()) {
        workerRunning.store(false);
        queueCV.notify_all();
        for (auto& t : workerThreads) {
            if (t.joinable()) {
                t.join();
            }
        }
        workerThreads.clear();
    }
}

bool TerrainManager::initialize(pipeline::AssetManager* assets, TerrainRenderer* renderer) {
    assetManager = assets;
    terrainRenderer = renderer;

    if (!assetManager) {
        LOG_ERROR("Asset manager is null");
        return false;
    }

    if (!terrainRenderer) {
        LOG_ERROR("Terrain renderer is null");
        return false;
    }

    // Start background worker pool
    workerRunning.store(true);
    unsigned hc = std::thread::hardware_concurrency();
    workerCount = static_cast<int>(hc > 0 ? std::min(4u, std::max(2u, hc - 1)) : 2u);
    workerThreads.reserve(workerCount);
    for (int i = 0; i < workerCount; i++) {
        workerThreads.emplace_back(&TerrainManager::workerLoop, this);
    }

    LOG_INFO("Terrain manager initialized (async loading enabled)");
    LOG_INFO("  Map: ", mapName);
    LOG_INFO("  Load radius: ", loadRadius, " tiles");
    LOG_INFO("  Unload radius: ", unloadRadius, " tiles");
    LOG_INFO("  Workers: ", workerCount);

    return true;
}

void TerrainManager::update(const Camera& camera, float deltaTime) {
    if (!streamingEnabled || !assetManager || !terrainRenderer) {
        return;
    }

    // Always process ready tiles each frame (GPU uploads from background thread)
    processReadyTiles();

    timeSinceLastUpdate += deltaTime;

    // Only update streaming periodically (not every frame)
    if (timeSinceLastUpdate < updateInterval) {
        return;
    }

    timeSinceLastUpdate = 0.0f;

    // Get current tile from camera position
    // GL coordinate mapping: GL Y = -(wowX - ZEROPOINT), GL X = -(wowZ - ZEROPOINT), GL Z = height
    // worldToTile expects: worldX = -glY (maps to tileX), worldY = glX (maps to tileY)
    glm::vec3 camPos = camera.getPosition();
    TileCoord newTile = worldToTile(-camPos.y, camPos.x);

    // Check if we've moved to a different tile
    if (newTile.x != currentTile.x || newTile.y != currentTile.y) {
        LOG_DEBUG("Camera moved to tile [", newTile.x, ",", newTile.y, "]");
        currentTile = newTile;
    }

    // Stream tiles if we've moved significantly or initial load
    if (newTile.x != lastStreamTile.x || newTile.y != lastStreamTile.y) {
        LOG_DEBUG("Streaming: cam=(", camPos.x, ",", camPos.y, ",", camPos.z,
                 ") tile=[", newTile.x, ",", newTile.y,
                 "] loaded=", loadedTiles.size());
        streamTiles();
        lastStreamTile = newTile;
    }
}

// Synchronous fallback for initial tile loading (before worker thread is useful)
bool TerrainManager::loadTile(int x, int y) {
    TileCoord coord = {x, y};

    // Check if already loaded
    if (loadedTiles.find(coord) != loadedTiles.end()) {
        return true;
    }

    // Don't retry tiles that already failed
    if (failedTiles.find(coord) != failedTiles.end()) {
        return false;
    }

    LOG_INFO("Loading terrain tile [", x, ",", y, "] (synchronous)");

    auto pending = prepareTile(x, y);
    if (!pending) {
        failedTiles[coord] = true;
        return false;
    }

    finalizeTile(std::move(pending));
    return true;
}

std::unique_ptr<PendingTile> TerrainManager::prepareTile(int x, int y) {
    TileCoord coord = {x, y};

    LOG_DEBUG("Preparing tile [", x, ",", y, "] (CPU work)");

    // Load ADT file
    std::string adtPath = getADTPath(coord);
    auto adtData = assetManager->readFile(adtPath);

    if (adtData.empty()) {
        LOG_WARNING("Failed to load ADT file: ", adtPath);
        return nullptr;
    }

    // Parse ADT
    pipeline::ADTTerrain terrain = pipeline::ADTLoader::load(adtData);
    if (!terrain.isLoaded()) {
        LOG_ERROR("Failed to parse ADT terrain: ", adtPath);
        return nullptr;
    }

    // Set tile coordinates so mesh knows where to position this tile in world
    terrain.coord.x = x;
    terrain.coord.y = y;

    // Generate mesh
    pipeline::TerrainMesh mesh = pipeline::TerrainMeshGenerator::generate(terrain);
    if (mesh.validChunkCount == 0) {
        LOG_ERROR("Failed to generate terrain mesh: ", adtPath);
        return nullptr;
    }

    auto pending = std::make_unique<PendingTile>();
    pending->coord = coord;
    pending->terrain = std::move(terrain);
    pending->mesh = std::move(mesh);

    // Pre-load M2 doodads (CPU: read files, parse models)
    if (!pending->terrain.doodadPlacements.empty()) {
        std::unordered_set<uint32_t> preparedModelIds;

        int skippedNameId = 0, skippedFileNotFound = 0, skippedInvalid = 0, skippedSkinNotFound = 0;

        for (const auto& placement : pending->terrain.doodadPlacements) {
            if (placement.nameId >= pending->terrain.doodadNames.size()) {
                skippedNameId++;
                continue;
            }

            std::string m2Path = pending->terrain.doodadNames[placement.nameId];

            // Convert .mdx to .m2 if needed
            if (m2Path.size() > 4) {
                std::string ext = m2Path.substr(m2Path.size() - 4);
                for (char& c : ext) c = std::tolower(c);
                if (ext == ".mdx") {
                    m2Path = m2Path.substr(0, m2Path.size() - 4) + ".m2";
                }
            }

            // Use path hash as globally unique model ID (nameId is per-tile local)
            uint32_t modelId = static_cast<uint32_t>(std::hash<std::string>{}(m2Path));

            // Parse model if not already done for this tile
            if (preparedModelIds.find(modelId) == preparedModelIds.end()) {
                std::vector<uint8_t> m2Data = assetManager->readFile(m2Path);
                if (!m2Data.empty()) {
                    pipeline::M2Model m2Model = pipeline::M2Loader::load(m2Data);

                    // Try to load skin file
                    std::string skinPath = m2Path.substr(0, m2Path.size() - 3) + "00.skin";
                    std::vector<uint8_t> skinData = assetManager->readFile(skinPath);
                    if (!skinData.empty()) {
                        pipeline::M2Loader::loadSkin(skinData, m2Model);
                    } else {
                        skippedSkinNotFound++;
                        LOG_WARNING("M2 skin not found: ", skinPath);
                    }

                    if (m2Model.isValid()) {
                        PendingTile::M2Ready ready;
                        ready.modelId = modelId;
                        ready.model = std::move(m2Model);
                        ready.path = m2Path;
                        pending->m2Models.push_back(std::move(ready));
                        preparedModelIds.insert(modelId);
                    } else {
                        skippedInvalid++;
                        LOG_WARNING("M2 model invalid (no verts/indices): ", m2Path);
                    }
                } else {
                    skippedFileNotFound++;
                    LOG_WARNING("M2 file not found: ", m2Path);
                }
            }

            // Store placement data for instance creation on main thread
            if (preparedModelIds.count(modelId)) {
                const float ZEROPOINT = 32.0f * 533.33333f;

                float wowX = placement.position[0];
                float wowY = placement.position[1];
                float wowZ = placement.position[2];

                PendingTile::M2Placement p;
                p.modelId = modelId;
                p.uniqueId = placement.uniqueId;
                p.position = glm::vec3(
                    -(wowZ - ZEROPOINT),
                    -(wowX - ZEROPOINT),
                    wowY
                );
                p.rotation = glm::vec3(
                    -placement.rotation[2] * 3.14159f / 180.0f,
                    -placement.rotation[0] * 3.14159f / 180.0f,
                    placement.rotation[1] * 3.14159f / 180.0f
                );
                p.scale = placement.scale / 1024.0f;
                pending->m2Placements.push_back(p);
            }
        }

        if (skippedNameId > 0 || skippedFileNotFound > 0 || skippedInvalid > 0) {
            LOG_WARNING("Tile [", x, ",", y, "] doodad issues: ",
                       skippedNameId, " bad nameId, ",
                       skippedFileNotFound, " file not found, ",
                       skippedInvalid, " invalid model, ",
                       skippedSkinNotFound, " skin not found");
        }
    }

    // Pre-load WMOs (CPU: read files, parse models and groups)
    if (!pending->terrain.wmoPlacements.empty()) {
        for (const auto& placement : pending->terrain.wmoPlacements) {
            if (placement.nameId >= pending->terrain.wmoNames.size()) continue;

            const std::string& wmoPath = pending->terrain.wmoNames[placement.nameId];
            std::vector<uint8_t> wmoData = assetManager->readFile(wmoPath);
            if (wmoData.empty()) continue;

            pipeline::WMOModel wmoModel = pipeline::WMOLoader::load(wmoData);
            if (wmoModel.nGroups > 0) {
                std::string basePath = wmoPath;
                std::string extension;
                if (basePath.size() > 4) {
                    extension = basePath.substr(basePath.size() - 4);
                    std::string extLower = extension;
                    for (char& c : extLower) c = std::tolower(c);
                    if (extLower == ".wmo") {
                        basePath = basePath.substr(0, basePath.size() - 4);
                    }
                }

                for (uint32_t gi = 0; gi < wmoModel.nGroups; gi++) {
                    char groupSuffix[16];
                    snprintf(groupSuffix, sizeof(groupSuffix), "_%03u%s", gi, extension.c_str());
                    std::string groupPath = basePath + groupSuffix;
                    std::vector<uint8_t> groupData = assetManager->readFile(groupPath);
                    if (groupData.empty()) {
                        snprintf(groupSuffix, sizeof(groupSuffix), "_%03u.wmo", gi);
                        groupData = assetManager->readFile(basePath + groupSuffix);
                    }
                    if (groupData.empty()) {
                        snprintf(groupSuffix, sizeof(groupSuffix), "_%03u.WMO", gi);
                        groupData = assetManager->readFile(basePath + groupSuffix);
                    }
                    if (!groupData.empty()) {
                        pipeline::WMOLoader::loadGroup(groupData, wmoModel, gi);
                    }
                }
            }

            if (!wmoModel.groups.empty()) {
                const float ZEROPOINT = 32.0f * 533.33333f;

                glm::vec3 pos(
                    -(placement.position[2] - ZEROPOINT),
                    -(placement.position[0] - ZEROPOINT),
                    placement.position[1]
                );

                glm::vec3 rot(
                    -placement.rotation[2] * 3.14159f / 180.0f,
                    -placement.rotation[0] * 3.14159f / 180.0f,
                    (placement.rotation[1] + 180.0f) * 3.14159f / 180.0f
                );

                // Pre-load WMO doodads (M2 models inside WMO)
                if (!wmoModel.doodadSets.empty() && !wmoModel.doodads.empty()) {
                    glm::mat4 wmoMatrix(1.0f);
                    wmoMatrix = glm::translate(wmoMatrix, pos);
                    wmoMatrix = glm::rotate(wmoMatrix, rot.z, glm::vec3(0, 0, 1));
                    wmoMatrix = glm::rotate(wmoMatrix, rot.y, glm::vec3(0, 1, 0));
                    wmoMatrix = glm::rotate(wmoMatrix, rot.x, glm::vec3(1, 0, 0));

                    const auto& doodadSet = wmoModel.doodadSets[0];
                    for (uint32_t di = 0; di < doodadSet.count; di++) {
                        uint32_t doodadIdx = doodadSet.startIndex + di;
                        if (doodadIdx >= wmoModel.doodads.size()) break;

                        const auto& doodad = wmoModel.doodads[doodadIdx];
                        auto nameIt = wmoModel.doodadNames.find(doodad.nameIndex);
                        if (nameIt == wmoModel.doodadNames.end()) continue;

                        std::string m2Path = nameIt->second;
                        if (m2Path.empty()) continue;

                        if (m2Path.size() > 4) {
                            std::string ext = m2Path.substr(m2Path.size() - 4);
                            for (char& c : ext) c = std::tolower(c);
                            if (ext == ".mdx" || ext == ".mdl") {
                                m2Path = m2Path.substr(0, m2Path.size() - 4) + ".m2";
                            }
                        }

                        uint32_t doodadModelId = static_cast<uint32_t>(std::hash<std::string>{}(m2Path));
                        std::vector<uint8_t> m2Data = assetManager->readFile(m2Path);
                        if (m2Data.empty()) continue;

                        pipeline::M2Model m2Model = pipeline::M2Loader::load(m2Data);
                        std::string skinPath = m2Path.substr(0, m2Path.size() - 3) + "00.skin";
                        std::vector<uint8_t> skinData = assetManager->readFile(skinPath);
                        if (!skinData.empty()) {
                            pipeline::M2Loader::loadSkin(skinData, m2Model);
                        }
                        if (!m2Model.isValid()) continue;

                        // Build doodad's local transform (WoW coordinates)
                        // WMO doodads use quaternion rotation
                        glm::mat4 doodadLocal(1.0f);
                        doodadLocal = glm::translate(doodadLocal, doodad.position);
                        doodadLocal *= glm::mat4_cast(doodad.rotation);
                        doodadLocal = glm::scale(doodadLocal, glm::vec3(doodad.scale));

                        // Full world transform = WMO world transform * doodad local transform
                        glm::mat4 worldMatrix = wmoMatrix * doodadLocal;

                        // Extract world position for frustum culling
                        glm::vec3 worldPos = glm::vec3(worldMatrix[3]);

                        PendingTile::WMODoodadReady doodadReady;
                        doodadReady.modelId = doodadModelId;
                        doodadReady.model = std::move(m2Model);
                        doodadReady.worldPosition = worldPos;
                        doodadReady.modelMatrix = worldMatrix;
                        pending->wmoDoodads.push_back(std::move(doodadReady));
                    }
                }

                PendingTile::WMOReady ready;
                ready.modelId = placement.uniqueId;
                ready.model = std::move(wmoModel);
                ready.position = pos;
                ready.rotation = rot;
                pending->wmoModels.push_back(std::move(ready));
            }
        }
    }

    LOG_DEBUG("Prepared tile [", x, ",", y, "]: ",
             pending->m2Models.size(), " M2 models, ",
             pending->m2Placements.size(), " M2 placements, ",
             pending->wmoModels.size(), " WMOs, ",
             pending->wmoDoodads.size(), " WMO doodads");

    return pending;
}

void TerrainManager::finalizeTile(std::unique_ptr<PendingTile> pending) {
    int x = pending->coord.x;
    int y = pending->coord.y;
    TileCoord coord = pending->coord;

    LOG_DEBUG("Finalizing tile [", x, ",", y, "] (GPU upload)");

    // Check if tile was already loaded (race condition guard) or failed
    if (loadedTiles.find(coord) != loadedTiles.end()) {
        return;
    }
    if (failedTiles.find(coord) != failedTiles.end()) {
        return;
    }

    // Upload terrain to GPU
    if (!terrainRenderer->loadTerrain(pending->mesh, pending->terrain.textures, x, y)) {
        LOG_ERROR("Failed to upload terrain to GPU for tile [", x, ",", y, "]");
        failedTiles[coord] = true;
        return;
    }

    // Load water
    if (waterRenderer) {
        waterRenderer->loadFromTerrain(pending->terrain, true, x, y);
    }

    std::vector<uint32_t> m2InstanceIds;
    std::vector<uint32_t> wmoInstanceIds;
    std::vector<uint32_t> tileUniqueIds;

    // Upload M2 models to GPU and create instances
    if (m2Renderer && assetManager) {
        if (!m2Renderer->getModelCount()) {
            m2Renderer->initialize(assetManager);
        }

        // Upload unique models
        std::unordered_set<uint32_t> uploadedModelIds;
        for (auto& m2Ready : pending->m2Models) {
            if (m2Renderer->loadModel(m2Ready.model, m2Ready.modelId)) {
                uploadedModelIds.insert(m2Ready.modelId);
            }
        }

        // Create instances (deduplicate by uniqueId across tile boundaries)
        int loadedDoodads = 0;
        int skippedDedup = 0;
        for (const auto& p : pending->m2Placements) {
            // Skip if this doodad was already placed by a neighboring tile
            if (placedDoodadIds.count(p.uniqueId)) {
                skippedDedup++;
                continue;
            }
            uint32_t instId = m2Renderer->createInstance(p.modelId, p.position, p.rotation, p.scale);
            if (instId) {
                m2InstanceIds.push_back(instId);
                placedDoodadIds.insert(p.uniqueId);
                tileUniqueIds.push_back(p.uniqueId);
                loadedDoodads++;
            }
        }

        LOG_DEBUG("  Loaded doodads for tile [", x, ",", y, "]: ",
                 loadedDoodads, " instances (", uploadedModelIds.size(), " new models, ",
                 skippedDedup, " dedup skipped)");
    }

    // Upload WMO models to GPU and create instances
    if (wmoRenderer && assetManager) {
        if (!wmoRenderer->getModelCount()) {
            wmoRenderer->initialize(assetManager);
        }

        int loadedWMOs = 0;
        int loadedLiquids = 0;
        for (auto& wmoReady : pending->wmoModels) {
            if (wmoRenderer->loadModel(wmoReady.model, wmoReady.modelId)) {
                uint32_t wmoInstId = wmoRenderer->createInstance(wmoReady.modelId, wmoReady.position, wmoReady.rotation);
                if (wmoInstId) {
                    wmoInstanceIds.push_back(wmoInstId);
                    loadedWMOs++;

                    // Load WMO liquids (canals, pools, etc.)
                    if (waterRenderer) {
                        // Compute the same model matrix as WMORenderer uses
                        glm::mat4 modelMatrix = glm::mat4(1.0f);
                        modelMatrix = glm::translate(modelMatrix, wmoReady.position);
                        modelMatrix = glm::rotate(modelMatrix, wmoReady.rotation.z, glm::vec3(0.0f, 0.0f, 1.0f));
                        modelMatrix = glm::rotate(modelMatrix, wmoReady.rotation.y, glm::vec3(0.0f, 1.0f, 0.0f));
                        modelMatrix = glm::rotate(modelMatrix, wmoReady.rotation.x, glm::vec3(1.0f, 0.0f, 0.0f));

                        // Load liquids from each WMO group
                        for (const auto& group : wmoReady.model.groups) {
                            if (group.liquid.hasLiquid()) {
                                waterRenderer->loadFromWMO(group.liquid, modelMatrix, wmoInstId);
                                loadedLiquids++;
                            }
                        }
                    }
                }
            }
        }
        if (loadedLiquids > 0) {
            LOG_DEBUG("  Loaded WMO liquids for tile [", x, ",", y, "]: ", loadedLiquids);
        }

        // Upload WMO doodad M2 models
        if (m2Renderer) {
            for (auto& doodad : pending->wmoDoodads) {
                m2Renderer->loadModel(doodad.model, doodad.modelId);
                uint32_t wmoDoodadInstId = m2Renderer->createInstanceWithMatrix(
                    doodad.modelId, doodad.modelMatrix, doodad.worldPosition);
                if (wmoDoodadInstId) m2InstanceIds.push_back(wmoDoodadInstId);
            }
        }

        if (loadedWMOs > 0) {
            LOG_DEBUG("  Loaded WMOs for tile [", x, ",", y, "]: ", loadedWMOs);
        }
    }

    // Create tile entry
    auto tile = std::make_unique<TerrainTile>();
    tile->coord = coord;
    tile->terrain = std::move(pending->terrain);
    tile->mesh = std::move(pending->mesh);
    tile->loaded = true;
    tile->m2InstanceIds = std::move(m2InstanceIds);
    tile->wmoInstanceIds = std::move(wmoInstanceIds);
    tile->doodadUniqueIds = std::move(tileUniqueIds);

    // Calculate world bounds
    getTileBounds(coord, tile->minX, tile->minY, tile->maxX, tile->maxY);

    loadedTiles[coord] = std::move(tile);

    LOG_DEBUG("  Finalized tile [", x, ",", y, "]");
}

void TerrainManager::workerLoop() {
    LOG_INFO("Terrain worker thread started");

    while (workerRunning.load()) {
        TileCoord coord;
        bool hasWork = false;

        {
            std::unique_lock<std::mutex> lock(queueMutex);
            queueCV.wait(lock, [this]() {
                return !loadQueue.empty() || !workerRunning.load();
            });

            if (!workerRunning.load()) {
                break;
            }

            if (!loadQueue.empty()) {
                coord = loadQueue.front();
                loadQueue.pop();
                hasWork = true;
            }
        }

        if (hasWork) {
            auto pending = prepareTile(coord.x, coord.y);

            std::lock_guard<std::mutex> lock(queueMutex);
            if (pending) {
                readyQueue.push(std::move(pending));
            } else {
                // Mark as failed so we don't re-enqueue
                // We'll set failedTiles on the main thread in processReadyTiles
                // For now, just remove from pending tracking
                pendingTiles.erase(coord);
            }
        }
    }

    LOG_INFO("Terrain worker thread stopped");
}

void TerrainManager::processReadyTiles() {
    // Process up to 2 ready tiles per frame to spread GPU work
    int processed = 0;
    const int maxPerFrame = 2;

    while (processed < maxPerFrame) {
        std::unique_ptr<PendingTile> pending;

        {
            std::lock_guard<std::mutex> lock(queueMutex);
            if (readyQueue.empty()) {
                break;
            }
            pending = std::move(readyQueue.front());
            readyQueue.pop();
        }

        if (pending) {
            TileCoord coord = pending->coord;
            finalizeTile(std::move(pending));
            pendingTiles.erase(coord);
            processed++;
        }
    }
}

void TerrainManager::unloadTile(int x, int y) {
    TileCoord coord = {x, y};

    // Also remove from pending if it was queued but not yet loaded
    pendingTiles.erase(coord);

    auto it = loadedTiles.find(coord);
    if (it == loadedTiles.end()) {
        return;
    }

    LOG_INFO("Unloading terrain tile [", x, ",", y, "]");

    const auto& tile = it->second;

    // Remove doodad unique IDs from dedup set
    for (uint32_t uid : tile->doodadUniqueIds) {
        placedDoodadIds.erase(uid);
    }

    // Remove M2 doodad instances
    if (m2Renderer) {
        for (uint32_t id : tile->m2InstanceIds) {
            m2Renderer->removeInstance(id);
        }
        LOG_DEBUG("  Removed ", tile->m2InstanceIds.size(), " M2 instances");
    }

    // Remove WMO instances and their liquids
    if (wmoRenderer) {
        for (uint32_t id : tile->wmoInstanceIds) {
            // Remove WMO liquids associated with this instance
            if (waterRenderer) {
                waterRenderer->removeWMO(id);
            }
            wmoRenderer->removeInstance(id);
        }
        LOG_DEBUG("  Removed ", tile->wmoInstanceIds.size(), " WMO instances");
    }

    // Remove terrain chunks for this tile
    if (terrainRenderer) {
        terrainRenderer->removeTile(x, y);
    }

    // Remove water surfaces for this tile
    if (waterRenderer) {
        waterRenderer->removeTile(x, y);
    }

    loadedTiles.erase(it);

    // Clean up any models that are no longer referenced
    if (m2Renderer) {
        m2Renderer->cleanupUnusedModels();
    }
    if (wmoRenderer) {
        wmoRenderer->cleanupUnusedModels();
    }
}

void TerrainManager::unloadAll() {
    // Stop worker thread
    if (workerRunning.load()) {
        workerRunning.store(false);
        queueCV.notify_all();
        for (auto& t : workerThreads) {
            if (t.joinable()) {
                t.join();
            }
        }
        workerThreads.clear();
    }

    // Clear queues
    {
        std::lock_guard<std::mutex> lock(queueMutex);
        while (!loadQueue.empty()) loadQueue.pop();
        while (!readyQueue.empty()) readyQueue.pop();
    }
    pendingTiles.clear();
    placedDoodadIds.clear();

    LOG_INFO("Unloading all terrain tiles");
    loadedTiles.clear();
    failedTiles.clear();

    // Clear terrain renderer
    if (terrainRenderer) {
        terrainRenderer->clear();
    }

    // Clear water
    if (waterRenderer) {
        waterRenderer->clear();
    }
}

TileCoord TerrainManager::worldToTile(float worldX, float worldY) const {
    // WoW world coordinate system:
    // - Tiles are 8533.33 units wide (TILE_SIZE)
    // - Tile (32, 32) is roughly at world origin for continents
    // - Coordinates increase going east (X) and south (Y)

    int tileX = 32 + static_cast<int>(std::floor(worldX / TILE_SIZE));
    int tileY = 32 - static_cast<int>(std::floor(worldY / TILE_SIZE));

    // Clamp to valid range (0-63)
    tileX = std::max(0, std::min(63, tileX));
    tileY = std::max(0, std::min(63, tileY));

    return {tileX, tileY};
}

void TerrainManager::getTileBounds(const TileCoord& coord, float& minX, float& minY,
                                    float& maxX, float& maxY) const {
    // Calculate world bounds for this tile
    // Tile (32, 32) is at origin
    float offsetX = (32 - coord.x) * TILE_SIZE;
    float offsetY = (32 - coord.y) * TILE_SIZE;

    minX = offsetX - TILE_SIZE;
    minY = offsetY - TILE_SIZE;
    maxX = offsetX;
    maxY = offsetY;
}

std::string TerrainManager::getADTPath(const TileCoord& coord) const {
    // Format: World\Maps\{MapName}\{MapName}_{X}_{Y}.adt
    // Example: Azeroth_32_49.adt for tile at coord.x=32, coord.y=49
    return "World\\Maps\\" + mapName + "\\" + mapName + "_" +
           std::to_string(coord.x) + "_" + std::to_string(coord.y) + ".adt";
}

std::optional<float> TerrainManager::getHeightAt(float glX, float glY) const {
    // Terrain mesh vertices use chunk.position directly (WoW coordinates)
    // But camera is in GL coordinates. We query using the mesh coordinates directly
    // since terrain is rendered without model transformation.
    //
    // The terrain mesh generation puts vertices at:
    //   vertex.position[0] = chunk.position[0] - (offsetY * unitSize)
    //   vertex.position[1] = chunk.position[1] - (offsetX * unitSize)
    //   vertex.position[2] = chunk.position[2] + height
    //
    // So chunk spans:
    //   X: [chunk.position[0] - 8*unitSize, chunk.position[0]]
    //   Y: [chunk.position[1] - 8*unitSize, chunk.position[1]]

    const float unitSize = CHUNK_SIZE / 8.0f;

    auto sampleTileHeight = [&](const TerrainTile* tile) -> std::optional<float> {
        if (!tile || !tile->loaded) return std::nullopt;

        auto sampleChunk = [&](int cx, int cy) -> std::optional<float> {
            if (cx < 0 || cx >= 16 || cy < 0 || cy >= 16) return std::nullopt;
            const auto& chunk = tile->terrain.getChunk(cx, cy);
            if (!chunk.hasHeightMap()) return std::nullopt;

            float chunkMaxX = chunk.position[0];
            float chunkMinX = chunk.position[0] - 8.0f * unitSize;
            float chunkMaxY = chunk.position[1];
            float chunkMinY = chunk.position[1] - 8.0f * unitSize;

            if (glX < chunkMinX || glX > chunkMaxX ||
                glY < chunkMinY || glY > chunkMaxY) {
                return std::nullopt;
            }

            // Fractional position within chunk (0-8 range)
            float fracY = (chunk.position[0] - glX) / unitSize;  // maps to offsetY
            float fracX = (chunk.position[1] - glY) / unitSize;  // maps to offsetX

            fracX = glm::clamp(fracX, 0.0f, 8.0f);
            fracY = glm::clamp(fracY, 0.0f, 8.0f);

            // Bilinear interpolation on 9x9 outer grid
            int gx0 = static_cast<int>(std::floor(fracX));
            int gy0 = static_cast<int>(std::floor(fracY));
            int gx1 = std::min(gx0 + 1, 8);
            int gy1 = std::min(gy0 + 1, 8);

            float tx = fracX - gx0;
            float ty = fracY - gy0;

            float h00 = chunk.heightMap.heights[gy0 * 17 + gx0];
            float h10 = chunk.heightMap.heights[gy0 * 17 + gx1];
            float h01 = chunk.heightMap.heights[gy1 * 17 + gx0];
            float h11 = chunk.heightMap.heights[gy1 * 17 + gx1];

            float h = h00 * (1 - tx) * (1 - ty) +
                      h10 * tx * (1 - ty) +
                      h01 * (1 - tx) * ty +
                      h11 * tx * ty;

            return chunk.position[2] + h;
        };

        // Fast path: infer likely chunk index and probe 3x3 neighborhood.
        int guessCy = glm::clamp(static_cast<int>(std::floor((tile->maxX - glX) / CHUNK_SIZE)), 0, 15);
        int guessCx = glm::clamp(static_cast<int>(std::floor((tile->maxY - glY) / CHUNK_SIZE)), 0, 15);
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                auto h = sampleChunk(guessCx + dx, guessCy + dy);
                if (h) return h;
            }
        }

        // Fallback full scan for robustness at seams/unusual coords.
        for (int cy = 0; cy < 16; cy++) {
            for (int cx = 0; cx < 16; cx++) {
                auto h = sampleChunk(cx, cy);
                if (h) {
                    return h;
                }
            }
        }
        return std::nullopt;
    };

    // Fast path: sample the expected containing tile first.
    TileCoord tc = worldToTile(glX, glY);
    auto it = loadedTiles.find(tc);
    if (it != loadedTiles.end()) {
        auto h = sampleTileHeight(it->second.get());
        if (h) return h;
    }

    // Fallback: check all loaded tiles (handles seam/edge coordinate ambiguity).
    for (const auto& [coord, tile] : loadedTiles) {
        if (coord == tc) continue;
        auto h = sampleTileHeight(tile.get());
        if (h) return h;
    }

    return std::nullopt;
}

std::optional<std::string> TerrainManager::getDominantTextureAt(float glX, float glY) const {
    const float unitSize = CHUNK_SIZE / 8.0f;
    std::vector<uint8_t> alphaScratch;
    auto sampleTileTexture = [&](const TerrainTile* tile) -> std::optional<std::string> {
        if (!tile || !tile->loaded) return std::nullopt;

        auto sampleChunkTexture = [&](int cx, int cy) -> std::optional<std::string> {
            if (cx < 0 || cx >= 16 || cy < 0 || cy >= 16) return std::nullopt;
            const auto& chunk = tile->terrain.getChunk(cx, cy);
            if (!chunk.hasHeightMap() || chunk.layers.empty()) return std::nullopt;

            float chunkMaxX = chunk.position[0];
            float chunkMinX = chunk.position[0] - 8.0f * unitSize;
            float chunkMaxY = chunk.position[1];
            float chunkMinY = chunk.position[1] - 8.0f * unitSize;
            if (glX < chunkMinX || glX > chunkMaxX || glY < chunkMinY || glY > chunkMaxY) {
                return std::nullopt;
            }

            float fracY = (chunk.position[0] - glX) / unitSize;
            float fracX = (chunk.position[1] - glY) / unitSize;
            fracX = glm::clamp(fracX, 0.0f, 8.0f);
            fracY = glm::clamp(fracY, 0.0f, 8.0f);

            int alphaX = glm::clamp(static_cast<int>((fracX / 8.0f) * 63.0f), 0, 63);
            int alphaY = glm::clamp(static_cast<int>((fracY / 8.0f) * 63.0f), 0, 63);
            int alphaIndex = alphaY * 64 + alphaX;

            int weights[4] = {0, 0, 0, 0};
            size_t numLayers = std::min(chunk.layers.size(), static_cast<size_t>(4));
            int accum = 0;
            for (size_t layerIdx = 1; layerIdx < numLayers; layerIdx++) {
                int alpha = 0;
                if (decodeLayerAlpha(chunk, layerIdx, alphaScratch) && alphaIndex < static_cast<int>(alphaScratch.size())) {
                    alpha = alphaScratch[alphaIndex];
                }
                weights[layerIdx] = alpha;
                accum += alpha;
            }
            weights[0] = glm::clamp(255 - accum, 0, 255);

            size_t bestLayer = 0;
            int bestWeight = weights[0];
            for (size_t i = 1; i < numLayers; i++) {
                if (weights[i] > bestWeight) {
                    bestWeight = weights[i];
                    bestLayer = i;
                }
            }

            uint32_t texId = chunk.layers[bestLayer].textureId;
            if (texId < tile->terrain.textures.size()) {
                return tile->terrain.textures[texId];
            }
            return std::nullopt;
        };

        int guessCy = glm::clamp(static_cast<int>(std::floor((tile->maxX - glX) / CHUNK_SIZE)), 0, 15);
        int guessCx = glm::clamp(static_cast<int>(std::floor((tile->maxY - glY) / CHUNK_SIZE)), 0, 15);
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                auto tex = sampleChunkTexture(guessCx + dx, guessCy + dy);
                if (tex) return tex;
            }
        }

        for (int cy = 0; cy < 16; cy++) {
            for (int cx = 0; cx < 16; cx++) {
                auto tex = sampleChunkTexture(cx, cy);
                if (tex) {
                    return tex;
                }
            }
        }
        return std::nullopt;
    };

    // Fast path: check expected containing tile first.
    TileCoord tc = worldToTile(glX, glY);
    auto it = loadedTiles.find(tc);
    if (it != loadedTiles.end()) {
        auto tex = sampleTileTexture(it->second.get());
        if (tex) return tex;
    }

    // Fallback: seam/edge case.
    for (const auto& [coord, tile] : loadedTiles) {
        if (coord == tc) continue;
        auto tex = sampleTileTexture(tile.get());
        if (tex) return tex;
    }

    return std::nullopt;
}

void TerrainManager::streamTiles() {
    // Enqueue tiles in radius around current tile for async loading
    {
        std::lock_guard<std::mutex> lock(queueMutex);

        for (int dy = -loadRadius; dy <= loadRadius; dy++) {
            for (int dx = -loadRadius; dx <= loadRadius; dx++) {
                int tileX = currentTile.x + dx;
                int tileY = currentTile.y + dy;

                // Check valid range
                if (tileX < 0 || tileX > 63 || tileY < 0 || tileY > 63) {
                    continue;
                }

                TileCoord coord = {tileX, tileY};

                // Skip if already loaded, pending, or failed
                if (loadedTiles.find(coord) != loadedTiles.end()) continue;
                if (pendingTiles.find(coord) != pendingTiles.end()) continue;
                if (failedTiles.find(coord) != failedTiles.end()) continue;

                loadQueue.push(coord);
                pendingTiles[coord] = true;
            }
        }
    }

    // Notify workers that there's work
    queueCV.notify_all();

    // Unload tiles beyond unload radius (well past the camera far clip)
    std::vector<TileCoord> tilesToUnload;

    for (const auto& pair : loadedTiles) {
        const TileCoord& coord = pair.first;

        int dx = std::abs(coord.x - currentTile.x);
        int dy = std::abs(coord.y - currentTile.y);

        // Chebyshev distance
        if (dx > unloadRadius || dy > unloadRadius) {
            tilesToUnload.push_back(coord);
        }
    }

    for (const auto& coord : tilesToUnload) {
        unloadTile(coord.x, coord.y);
    }

    if (!tilesToUnload.empty()) {
        LOG_INFO("Unloaded ", tilesToUnload.size(), " distant tiles, ",
                 loadedTiles.size(), " remain");
    }
}

} // namespace rendering
} // namespace wowee
