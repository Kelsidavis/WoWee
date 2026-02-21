#include "rendering/terrain_manager.hpp"
#include "rendering/terrain_renderer.hpp"
#include "rendering/water_renderer.hpp"
#include "rendering/m2_renderer.hpp"
#include "rendering/wmo_renderer.hpp"
#include "rendering/camera.hpp"
#include "audio/ambient_sound_manager.hpp"
#include "core/coordinates.hpp"
#include "core/memory_monitor.hpp"
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

std::string toLowerCopy(std::string v) {
    std::transform(v.begin(), v.end(), v.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return v;
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

    // Set dynamic tile cache budget.
    // Keep this lower so decompressed MPQ file cache can stay very aggressive.
    auto& memMonitor = core::MemoryMonitor::getInstance();
    tileCacheBudgetBytes_ = memMonitor.getRecommendedCacheBudget() / 4;
    LOG_INFO("Terrain tile cache budget: ", tileCacheBudgetBytes_ / (1024 * 1024), " MB (dynamic)");

    // Start background worker pool (dynamic: scales with available cores)
    // Use 75% of logical cores for decompression, leaving headroom for render/OS
    workerRunning.store(true);
    unsigned hc = std::thread::hardware_concurrency();
    if (hc > 0) {
        unsigned targetWorkers = std::max(6u, (hc * 3) / 4);  // 75% of cores, minimum 6
        workerCount = static_cast<int>(targetWorkers);
    } else {
        workerCount = 6;  // Fallback
    }
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
    // Time budget prevents frame spikes from heavy tiles
    processReadyTiles();

    timeSinceLastUpdate += deltaTime;

    // Only update streaming periodically (not every frame)
    if (timeSinceLastUpdate < updateInterval) {
        return;
    }

    timeSinceLastUpdate = 0.0f;

    // Get current tile from camera position.
    glm::vec3 camPos = camera.getPosition();
    TileCoord newTile = worldToTile(camPos.x, camPos.y);

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

    finalizeTile(pending);
    return true;
}

bool TerrainManager::enqueueTile(int x, int y) {
    TileCoord coord = {x, y};
    if (loadedTiles.find(coord) != loadedTiles.end()) {
        return true;
    }
    if (pendingTiles.find(coord) != pendingTiles.end()) {
        return true;
    }
    if (failedTiles.find(coord) != failedTiles.end()) {
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(queueMutex);
        loadQueue.push_back(coord);
        pendingTiles[coord] = true;
    }
    queueCV.notify_all();
    return true;
}

std::shared_ptr<PendingTile> TerrainManager::prepareTile(int x, int y) {
    TileCoord coord = {x, y};
    if (auto cached = getCachedTile(coord)) {
        LOG_DEBUG("Using cached tile [", x, ",", y, "]");
        return cached;
    }

    LOG_DEBUG("Preparing tile [", x, ",", y, "] (CPU work)");

    // Load ADT file
    std::string adtPath = getADTPath(coord);
    auto adtData = assetManager->readFile(adtPath);

    if (adtData.empty()) {
        logMissingAdtOnce(adtPath);
        return nullptr;
    }

    // Parse ADT
    pipeline::ADTTerrain terrain = pipeline::ADTLoader::load(adtData);
    if (!terrain.isLoaded()) {
        LOG_ERROR("Failed to parse ADT terrain: ", adtPath);
        return nullptr;
    }

    // WotLK split ADTs can store placements in *_obj0.adt.
    // Merge object chunks so doodads/WMOs (including ground clutter) are available.
    std::string objPath = "World\\Maps\\" + mapName + "\\" + mapName + "_" +
                          std::to_string(coord.x) + "_" + std::to_string(coord.y) + "_obj0.adt";
    auto objData = assetManager->readFile(objPath);
    if (!objData.empty()) {
        pipeline::ADTTerrain objTerrain = pipeline::ADTLoader::load(objData);
        if (objTerrain.isLoaded()) {
            const uint32_t doodadNameBase = static_cast<uint32_t>(terrain.doodadNames.size());
            const uint32_t wmoNameBase = static_cast<uint32_t>(terrain.wmoNames.size());

            terrain.doodadNames.insert(terrain.doodadNames.end(),
                                       objTerrain.doodadNames.begin(), objTerrain.doodadNames.end());
            terrain.wmoNames.insert(terrain.wmoNames.end(),
                                    objTerrain.wmoNames.begin(), objTerrain.wmoNames.end());

            std::unordered_set<uint32_t> existingDoodadUniqueIds;
            existingDoodadUniqueIds.reserve(terrain.doodadPlacements.size());
            for (const auto& p : terrain.doodadPlacements) {
                if (p.uniqueId != 0) existingDoodadUniqueIds.insert(p.uniqueId);
            }

            size_t mergedDoodads = 0;
            for (auto placement : objTerrain.doodadPlacements) {
                if (placement.nameId >= objTerrain.doodadNames.size()) continue;
                placement.nameId += doodadNameBase;
                if (placement.uniqueId != 0 && !existingDoodadUniqueIds.insert(placement.uniqueId).second) {
                    continue;
                }
                terrain.doodadPlacements.push_back(placement);
                mergedDoodads++;
            }

            std::unordered_set<uint32_t> existingWmoUniqueIds;
            existingWmoUniqueIds.reserve(terrain.wmoPlacements.size());
            for (const auto& p : terrain.wmoPlacements) {
                if (p.uniqueId != 0) existingWmoUniqueIds.insert(p.uniqueId);
            }

            size_t mergedWmos = 0;
            for (auto placement : objTerrain.wmoPlacements) {
                if (placement.nameId >= objTerrain.wmoNames.size()) continue;
                placement.nameId += wmoNameBase;
                if (placement.uniqueId != 0 && !existingWmoUniqueIds.insert(placement.uniqueId).second) {
                    continue;
                }
                terrain.wmoPlacements.push_back(placement);
                mergedWmos++;
            }

            if (mergedDoodads > 0 || mergedWmos > 0) {
                LOG_DEBUG("Merged obj0 tile [", x, ",", y, "]: +", mergedDoodads,
                          " doodads, +", mergedWmos, " WMOs");
            }
        }
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

    auto pending = std::make_shared<PendingTile>();
    pending->coord = coord;
    pending->terrain = std::move(terrain);
    pending->mesh = std::move(mesh);

    std::unordered_set<uint32_t> preparedModelIds;
    auto ensureModelPrepared = [&](const std::string& m2Path,
                                   uint32_t modelId,
                                   int& skippedFileNotFound,
                                   int& skippedInvalid,
                                   int& skippedSkinNotFound) -> bool {
        if (preparedModelIds.find(modelId) != preparedModelIds.end()) return true;

        std::vector<uint8_t> m2Data = assetManager->readFile(m2Path);
        if (m2Data.empty()) {
            skippedFileNotFound++;
            LOG_WARNING("M2 file not found: ", m2Path);
            return false;
        }

        pipeline::M2Model m2Model = pipeline::M2Loader::load(m2Data);
        if (m2Model.name.empty()) {
            m2Model.name = m2Path;
        }
        std::string skinPath = m2Path.substr(0, m2Path.size() - 3) + "00.skin";
        std::vector<uint8_t> skinData = assetManager->readFileOptional(skinPath);
        if (!skinData.empty() && m2Model.version >= 264) {
            pipeline::M2Loader::loadSkin(skinData, m2Model);
        } else if (skinData.empty() && m2Model.version >= 264) {
            skippedSkinNotFound++;
        }

        if (!m2Model.isValid()) {
            skippedInvalid++;
            LOG_DEBUG("M2 model invalid (no verts/indices): ", m2Path);
            return false;
        }

        PendingTile::M2Ready ready;
        ready.modelId = modelId;
        ready.model = std::move(m2Model);
        ready.path = m2Path;
        pending->m2Models.push_back(std::move(ready));
        preparedModelIds.insert(modelId);
        return true;
    };

    // Pre-load M2 doodads (CPU: read files, parse models)
    int skippedNameId = 0, skippedFileNotFound = 0, skippedInvalid = 0, skippedSkinNotFound = 0;
    for (const auto& placement : pending->terrain.doodadPlacements) {
        if (placement.nameId >= pending->terrain.doodadNames.size()) {
            skippedNameId++;
            continue;
        }

        std::string m2Path = pending->terrain.doodadNames[placement.nameId];
        if (m2Path.size() > 4) {
            std::string ext = toLowerCopy(m2Path.substr(m2Path.size() - 4));
            if (ext == ".mdx") {
                m2Path = m2Path.substr(0, m2Path.size() - 4) + ".m2";
            }
        }

        uint32_t modelId = static_cast<uint32_t>(std::hash<std::string>{}(m2Path));
        if (!ensureModelPrepared(m2Path, modelId, skippedFileNotFound, skippedInvalid, skippedSkinNotFound)) {
            continue;
        }

        float wowX = placement.position[0];
        float wowY = placement.position[1];
        float wowZ = placement.position[2];
        glm::vec3 glPos = core::coords::adtToWorld(wowX, wowY, wowZ);

        PendingTile::M2Placement p;
        p.modelId = modelId;
        p.uniqueId = placement.uniqueId;
        p.position = glPos;
        p.rotation = glm::vec3(
            -placement.rotation[2] * 3.14159f / 180.0f,
            -placement.rotation[0] * 3.14159f / 180.0f,
            (placement.rotation[1] + 180.0f) * 3.14159f / 180.0f
        );
        p.scale = placement.scale / 1024.0f;
        pending->m2Placements.push_back(p);
    }

    if (skippedNameId > 0 || skippedFileNotFound > 0 || skippedInvalid > 0 || skippedSkinNotFound > 0) {
        LOG_DEBUG("Tile [", x, ",", y, "] doodad issues: ",
                  skippedNameId, " bad nameId, ",
                  skippedFileNotFound, " file not found, ",
                  skippedInvalid, " invalid model, ",
                  skippedSkinNotFound, " skin not found");
    }

    // Procedural ground clutter from terrain layer effectId -> GroundEffectTexture/Doodad DBCs.
    ensureGroundEffectTablesLoaded();
    generateGroundClutterPlacements(pending, preparedModelIds);

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
                glm::vec3 pos = core::coords::adtToWorld(placement.position[0],
                                                       placement.position[1],
                                                       placement.position[2]);

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
                        if (m2Model.name.empty()) {
                            m2Model.name = m2Path;
                        }
                        std::string skinPath = m2Path.substr(0, m2Path.size() - 3) + "00.skin";
                        std::vector<uint8_t> skinData = assetManager->readFile(skinPath);
                        if (!skinData.empty() && m2Model.version >= 264) {
                            pipeline::M2Loader::loadSkin(skinData, m2Model);
                        }
                        if (!m2Model.isValid()) continue;

                        // Build doodad's local transform (WoW coordinates)
                        // WMO doodads use quaternion rotation
                        // Fix: WoW quaternions need X/Y swap for correct orientation
                        glm::quat fixedRotation(doodad.rotation.w, doodad.rotation.y, doodad.rotation.x, doodad.rotation.z);

                        glm::mat4 doodadLocal(1.0f);
                        doodadLocal = glm::translate(doodadLocal, doodad.position);
                        doodadLocal *= glm::mat4_cast(fixedRotation);
                        doodadLocal = glm::scale(doodadLocal, glm::vec3(doodad.scale));

                        // Full world transform = WMO world transform * doodad local transform
                        glm::mat4 worldMatrix = wmoMatrix * doodadLocal;

                        // Extract world position for frustum culling
                        glm::vec3 worldPos = glm::vec3(worldMatrix[3]);

                        // Detect ambient sound emitters from doodad model path
                        std::string m2PathLower = m2Path;
                        std::transform(m2PathLower.begin(), m2PathLower.end(), m2PathLower.begin(), ::tolower);

                        // Debug: Log all doodad paths to help identify fire-related models
                        static int doodadLogCount = 0;
                        if (doodadLogCount < 50) {  // Limit logging to first 50 doodads
                            LOG_DEBUG("WMO doodad: ", m2Path);
                            doodadLogCount++;
                        }

                        if (m2PathLower.find("fire") != std::string::npos ||
                            m2PathLower.find("brazier") != std::string::npos ||
                            m2PathLower.find("campfire") != std::string::npos) {
                            // Fireplace/brazier emitter
                            PendingTile::AmbientEmitter emitter;
                            emitter.position = worldPos;
                            if (m2PathLower.find("small") != std::string::npos || m2PathLower.find("campfire") != std::string::npos) {
                                emitter.type = 0;  // FIREPLACE_SMALL
                            } else {
                                emitter.type = 1;  // FIREPLACE_LARGE
                            }
                            pending->ambientEmitters.push_back(emitter);
                        } else if (m2PathLower.find("torch") != std::string::npos) {
                            // Torch emitter
                            PendingTile::AmbientEmitter emitter;
                            emitter.position = worldPos;
                            emitter.type = 2;  // TORCH
                            pending->ambientEmitters.push_back(emitter);
                        } else if (m2PathLower.find("fountain") != std::string::npos) {
                            // Fountain emitter
                            PendingTile::AmbientEmitter emitter;
                            emitter.position = worldPos;
                            emitter.type = 3;  // FOUNTAIN
                            pending->ambientEmitters.push_back(emitter);
                        } else if (m2PathLower.find("waterfall") != std::string::npos) {
                            // Waterfall emitter
                            PendingTile::AmbientEmitter emitter;
                            emitter.position = worldPos;
                            emitter.type = 6;  // WATERFALL
                            pending->ambientEmitters.push_back(emitter);
                        }

                        PendingTile::WMODoodadReady doodadReady;
                        doodadReady.modelId = doodadModelId;
                        doodadReady.model = std::move(m2Model);
                        doodadReady.worldPosition = worldPos;
                        doodadReady.modelMatrix = worldMatrix;
                        pending->wmoDoodads.push_back(std::move(doodadReady));
                    }
                }

                PendingTile::WMOReady ready;
                // Cache WMO model uploads by path; placement dedup uses uniqueId separately.
                ready.modelId = static_cast<uint32_t>(std::hash<std::string>{}(wmoPath));
                if (ready.modelId == 0) ready.modelId = 1;
                ready.uniqueId = placement.uniqueId;
                ready.model = std::move(wmoModel);
                ready.position = pos;
                ready.rotation = rot;
                pending->wmoModels.push_back(std::move(ready));
            }
        }
    }

    // Pre-load terrain texture BLP data on background thread so finalizeTile
    // doesn't block the main thread with file I/O.
    for (const auto& texPath : pending->terrain.textures) {
        if (pending->preloadedTextures.find(texPath) != pending->preloadedTextures.end()) continue;
        pending->preloadedTextures[texPath] = assetManager->loadTexture(texPath);
    }

    LOG_DEBUG("Prepared tile [", x, ",", y, "]: ",
             pending->m2Models.size(), " M2 models, ",
             pending->m2Placements.size(), " M2 placements, ",
             pending->wmoModels.size(), " WMOs, ",
             pending->wmoDoodads.size(), " WMO doodads, ",
             pending->preloadedTextures.size(), " textures");

    return pending;
}

void TerrainManager::logMissingAdtOnce(const std::string& adtPath) {
    std::string normalized = adtPath;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    std::lock_guard<std::mutex> lock(missingAdtWarningsMutex_);
    if (missingAdtWarnings_.insert(normalized).second) {
        LOG_WARNING("Failed to load ADT file: ", adtPath);
    }
}

void TerrainManager::finalizeTile(const std::shared_ptr<PendingTile>& pending) {
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

    // Upload pre-loaded textures to the GL cache so loadTerrain avoids file I/O
    if (!pending->preloadedTextures.empty()) {
        terrainRenderer->uploadPreloadedTextures(pending->preloadedTextures);
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

    // Register water surface ambient sound emitters
    if (ambientSoundManager) {
        // Scan ADT water data for water surfaces
        int waterEmitterCount = 0;
        for (size_t chunkIdx = 0; chunkIdx < pending->terrain.waterData.size(); chunkIdx++) {
            const auto& chunkWater = pending->terrain.waterData[chunkIdx];
            if (!chunkWater.hasWater()) continue;

            // Calculate chunk position in world coordinates
            int chunkX = chunkIdx % 16;
            int chunkY = chunkIdx / 16;

            // WoW coordinates: Each ADT tile is 533.33 units, each chunk is 533.33/16 = 33.333 units
            // Tile origin in GL space
            float tileOriginX = (32.0f - x) * 533.33333f;
            float tileOriginY = (32.0f - y) * 533.33333f;

            // Chunk center position
            float chunkCenterX = tileOriginX + (chunkX + 0.5f) * 33.333333f;
            float chunkCenterY = tileOriginY + (chunkY + 0.5f) * 33.333333f;

            // Use first layer for height and type detection
            if (!chunkWater.layers.empty()) {
                const auto& layer = chunkWater.layers[0];
                float waterHeight = layer.minHeight;

                // Determine water type and register appropriate emitter
                // liquidType: 0=water/lake, 1=ocean, 2=magma, 3=slime
                if (layer.liquidType == 0) {
                    // Lake/river water - add water surface emitter every 32 chunks to avoid spam
                    if (chunkIdx % 32 == 0) {
                        PendingTile::AmbientEmitter emitter;
                        emitter.position = glm::vec3(chunkCenterX, chunkCenterY, waterHeight);
                        emitter.type = 4;  // WATER_SURFACE
                        pending->ambientEmitters.push_back(emitter);
                        waterEmitterCount++;
                    }
                } else if (layer.liquidType == 1) {
                    // Ocean - add ocean emitter every 64 chunks (oceans are very large)
                    if (chunkIdx % 64 == 0) {
                        PendingTile::AmbientEmitter emitter;
                        emitter.position = glm::vec3(chunkCenterX, chunkCenterY, waterHeight);
                        emitter.type = 4;  // WATER_SURFACE (could add separate OCEAN type later)
                        pending->ambientEmitters.push_back(emitter);
                        waterEmitterCount++;
                    }
                }
                // Skip magma and slime for now (no ambient sounds for those)
            }
        }
        if (waterEmitterCount > 0) {
        }
    }

    std::vector<uint32_t> m2InstanceIds;
    std::vector<uint32_t> wmoInstanceIds;
    std::vector<uint32_t> tileUniqueIds;
    std::vector<uint32_t> tileWmoUniqueIds;

    // Upload M2 models to GPU and create instances
    if (m2Renderer && assetManager) {
        // Always pass the latest asset manager. initialize() is idempotent and updates
        // the pointer even when the renderer was initialized earlier without assets.
        m2Renderer->initialize(assetManager);

        // Upload M2 models immediately (batching was causing hangs)
        // The 5ms time budget in processReadyTiles() limits the spike
        std::unordered_set<uint32_t> uploadedModelIds;
        for (auto& m2Ready : pending->m2Models) {
            if (m2Renderer->loadModel(m2Ready.model, m2Ready.modelId)) {
                uploadedModelIds.insert(m2Ready.modelId);
            }
        }
        if (!uploadedModelIds.empty()) {
            LOG_DEBUG("  Uploaded ", uploadedModelIds.size(), " M2 models for tile [", x, ",", y, "]");
        }

        // Create instances (deduplicate by uniqueId across tile boundaries)
        int loadedDoodads = 0;
        int skippedDedup = 0;
        for (const auto& p : pending->m2Placements) {
            // Skip if this doodad was already placed by a neighboring tile
            if (p.uniqueId != 0 && placedDoodadIds.count(p.uniqueId)) {
                skippedDedup++;
                continue;
            }
            uint32_t instId = m2Renderer->createInstance(p.modelId, p.position, p.rotation, p.scale);
            if (instId) {
                m2InstanceIds.push_back(instId);
                if (p.uniqueId != 0) {
                    placedDoodadIds.insert(p.uniqueId);
                    tileUniqueIds.push_back(p.uniqueId);
                }
                loadedDoodads++;
            }
        }

        LOG_DEBUG("  Loaded doodads for tile [", x, ",", y, "]: ",
                 loadedDoodads, " instances (", uploadedModelIds.size(), " new models, ",
                 skippedDedup, " dedup skipped)");
    }

    // Upload WMO models to GPU and create instances
    if (wmoRenderer && assetManager) {
        // WMORenderer may be initialized before assets are ready; always re-pass assets.
        wmoRenderer->initialize(assetManager);

        int loadedWMOs = 0;
        int loadedLiquids = 0;
        int skippedWmoDedup = 0;
        for (auto& wmoReady : pending->wmoModels) {
            // Deduplicate by placement uniqueId when available.
            // Some ADTs use uniqueId=0, which is not safe for dedup.
            if (wmoReady.uniqueId != 0 && placedWmoIds.count(wmoReady.uniqueId)) {
                skippedWmoDedup++;
                continue;
            }

            if (wmoRenderer->loadModel(wmoReady.model, wmoReady.modelId)) {
                uint32_t wmoInstId = wmoRenderer->createInstance(wmoReady.modelId, wmoReady.position, wmoReady.rotation);
                if (wmoInstId) {
                    wmoInstanceIds.push_back(wmoInstId);
                    if (wmoReady.uniqueId != 0) {
                        placedWmoIds.insert(wmoReady.uniqueId);
                        tileWmoUniqueIds.push_back(wmoReady.uniqueId);
                    }
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
        if (loadedWMOs > 0 || skippedWmoDedup > 0) {
            LOG_DEBUG("  Loaded WMOs for tile [", x, ",", y, "]: ",
                     loadedWMOs, " instances, ", skippedWmoDedup, " dedup skipped");
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

    // Register ambient sound emitters with ambient sound manager
    if (ambientSoundManager && !pending->ambientEmitters.empty()) {
        for (const auto& emitter : pending->ambientEmitters) {
            // Cast uint32_t type to AmbientSoundManager::AmbientType enum
            auto type = static_cast<audio::AmbientSoundManager::AmbientType>(emitter.type);
            ambientSoundManager->addEmitter(emitter.position, type);
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
    tile->wmoUniqueIds = std::move(tileWmoUniqueIds);
    tile->doodadUniqueIds = std::move(tileUniqueIds);

    // Calculate world bounds
    getTileBounds(coord, tile->minX, tile->minY, tile->maxX, tile->maxY);

    loadedTiles[coord] = std::move(tile);
    putCachedTile(pending);

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
                loadQueue.pop_front();
                hasWork = true;
            }
        }

        if (hasWork) {
            auto pending = prepareTile(coord.x, coord.y);

            std::lock_guard<std::mutex> lock(queueMutex);
            if (pending) {
                readyQueue.push(pending);
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
    // Process tiles with time budget to avoid frame spikes
    // Taxi mode gets a slightly larger budget to avoid visible late-pop terrain/models.
    const float timeBudgetMs = taxiStreamingMode_ ? 8.0f : 5.0f;
    auto startTime = std::chrono::high_resolution_clock::now();
    int processed = 0;

    while (true) {
        std::shared_ptr<PendingTile> pending;

        {
            std::lock_guard<std::mutex> lock(queueMutex);
            if (readyQueue.empty()) {
                break;
            }
            pending = readyQueue.front();
            readyQueue.pop();
        }

        if (pending) {
            TileCoord coord = pending->coord;
            auto tileStart = std::chrono::high_resolution_clock::now();

            finalizeTile(pending);

            auto tileEnd = std::chrono::high_resolution_clock::now();
            float tileTimeMs = std::chrono::duration<float, std::milli>(tileEnd - tileStart).count();

            {
                std::lock_guard<std::mutex> lock(queueMutex);
                pendingTiles.erase(coord);
            }
            processed++;

            // Check if we've exceeded time budget
            float elapsedMs = std::chrono::duration<float, std::milli>(tileEnd - startTime).count();
            if (elapsedMs >= timeBudgetMs) {
                if (processed > 1) {
                    LOG_DEBUG("Processed ", processed, " tiles in ", elapsedMs, "ms (budget: ", timeBudgetMs, "ms)");
                }
                break;
            }
        }
    }
}

void TerrainManager::processM2UploadQueue() {
    // Upload up to MAX_M2_UPLOADS_PER_FRAME models per frame
    int uploaded = 0;
    while (!m2UploadQueue_.empty() && uploaded < MAX_M2_UPLOADS_PER_FRAME) {
        auto& upload = m2UploadQueue_.front();
        if (m2Renderer) {
            m2Renderer->loadModel(upload.model, upload.modelId);
        }
        m2UploadQueue_.pop();
        uploaded++;
    }

    if (uploaded > 0) {
        LOG_DEBUG("Uploaded ", uploaded, " M2 models (", m2UploadQueue_.size(), " remaining in queue)");
    }
}

void TerrainManager::processAllReadyTiles() {
    while (true) {
        std::shared_ptr<PendingTile> pending;
        {
            std::lock_guard<std::mutex> lock(queueMutex);
            if (readyQueue.empty()) break;
            pending = readyQueue.front();
            readyQueue.pop();
        }
        if (pending) {
            TileCoord coord = pending->coord;
            finalizeTile(pending);
            {
                std::lock_guard<std::mutex> lock(queueMutex);
                pendingTiles.erase(coord);
            }
        }
    }
}

std::shared_ptr<PendingTile> TerrainManager::getCachedTile(const TileCoord& coord) {
    std::lock_guard<std::mutex> lock(tileCacheMutex_);
    auto it = tileCache_.find(coord);
    if (it == tileCache_.end()) return nullptr;
    tileCacheLru_.erase(it->second.lruIt);
    tileCacheLru_.push_front(coord);
    it->second.lruIt = tileCacheLru_.begin();
    return it->second.tile;
}

void TerrainManager::putCachedTile(const std::shared_ptr<PendingTile>& tile) {
    if (!tile) return;
    std::lock_guard<std::mutex> lock(tileCacheMutex_);
    TileCoord coord = tile->coord;

    auto it = tileCache_.find(coord);
    if (it != tileCache_.end()) {
        tileCacheLru_.erase(it->second.lruIt);
        tileCacheBytes_ -= it->second.bytes;
        tileCache_.erase(it);
    }

    size_t bytes = estimatePendingTileBytes(*tile);
    tileCacheLru_.push_front(coord);
    tileCache_[coord] = CachedTile{tile, bytes, tileCacheLru_.begin()};
    tileCacheBytes_ += bytes;

    // Evict least-recently used tiles until under budget
    while (tileCacheBytes_ > tileCacheBudgetBytes_ && !tileCacheLru_.empty()) {
        TileCoord evictCoord = tileCacheLru_.back();
        auto eit = tileCache_.find(evictCoord);
        if (eit != tileCache_.end()) {
            tileCacheBytes_ -= eit->second.bytes;
            tileCache_.erase(eit);
        }
        tileCacheLru_.pop_back();
    }
}

size_t TerrainManager::estimatePendingTileBytes(const PendingTile& tile) const {
    size_t bytes = 0;
    bytes += sizeof(PendingTile);
    bytes += tile.terrain.textures.size() * 64;
    bytes += tile.terrain.doodadNames.size() * 64;
    bytes += tile.terrain.wmoNames.size() * 64;
    bytes += tile.terrain.doodadPlacements.size() * sizeof(pipeline::ADTTerrain::DoodadPlacement);
    bytes += tile.terrain.wmoPlacements.size() * sizeof(pipeline::ADTTerrain::WMOPlacement);

    for (const auto& chunk : tile.terrain.chunks) {
        bytes += sizeof(chunk);
        bytes += chunk.layers.size() * sizeof(pipeline::TextureLayer);
        bytes += chunk.alphaMap.size();
    }

    for (const auto& cm : tile.mesh.chunks) {
        bytes += cm.vertices.size() * sizeof(pipeline::TerrainVertex);
        bytes += cm.indices.size() * sizeof(pipeline::TerrainIndex);
        for (const auto& layer : cm.layers) {
            bytes += layer.alphaData.size();
        }
    }

    for (const auto& ready : tile.m2Models) {
        bytes += ready.model.vertices.size() * sizeof(pipeline::M2Vertex);
        bytes += ready.model.indices.size() * sizeof(uint16_t);
        bytes += ready.model.textures.size() * sizeof(pipeline::M2Texture);
    }
    bytes += tile.m2Placements.size() * sizeof(PendingTile::M2Placement);

    for (const auto& ready : tile.wmoModels) {
        for (const auto& group : ready.model.groups) {
            bytes += group.vertices.size() * sizeof(pipeline::WMOVertex);
            bytes += group.indices.size() * sizeof(uint16_t);
            bytes += group.batches.size() * sizeof(pipeline::WMOBatch);
            bytes += group.portalVertices.size() * sizeof(glm::vec3);
            bytes += group.portals.size() * sizeof(pipeline::WMOPortal);
            bytes += group.bspNodes.size();
        }
    }
    bytes += tile.wmoDoodads.size() * sizeof(PendingTile::WMODoodadReady);

    for (const auto& [_, img] : tile.preloadedTextures) {
        bytes += img.data.size();
    }
    return bytes;
}

void TerrainManager::unloadTile(int x, int y) {
    TileCoord coord = {x, y};

    // Also remove from pending if it was queued but not yet loaded
    {
        std::lock_guard<std::mutex> lock(queueMutex);
        pendingTiles.erase(coord);
    }

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
    for (uint32_t uid : tile->wmoUniqueIds) {
        placedWmoIds.erase(uid);
    }

    // Remove M2 doodad instances
    if (m2Renderer) {
        m2Renderer->removeInstances(tile->m2InstanceIds);
        LOG_DEBUG("  Removed ", tile->m2InstanceIds.size(), " M2 instances");
    }

    // Remove WMO instances and their liquids
    if (wmoRenderer) {
        for (uint32_t id : tile->wmoInstanceIds) {
            // Remove WMO liquids associated with this instance
            if (waterRenderer) {
                waterRenderer->removeWMO(id);
            }
        }
        wmoRenderer->removeInstances(tile->wmoInstanceIds);
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
}

void TerrainManager::unloadAll() {
    // Stop worker threads
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
        while (!loadQueue.empty()) loadQueue.pop_front();
        while (!readyQueue.empty()) readyQueue.pop();
    }
    pendingTiles.clear();
    placedDoodadIds.clear();

    LOG_INFO("Unloading all terrain tiles");
    loadedTiles.clear();
    failedTiles.clear();

    // Reset tile tracking so streaming re-triggers at the new location
    currentTile = {-1, -1};
    lastStreamTile = {-1, -1};

    // Clear terrain renderer
    if (terrainRenderer) {
        terrainRenderer->clear();
    }

    // Clear water
    if (waterRenderer) {
        waterRenderer->clear();
    }

    // Clear WMO and M2 renderers so old-location geometry doesn't persist
    if (wmoRenderer) {
        wmoRenderer->clearInstances();
    }
    if (m2Renderer) {
        m2Renderer->clear();
    }

    // Restart worker threads so streaming can resume (dynamic: scales with available cores)
    // Use 75% of logical cores for decompression, leaving headroom for render/OS
    workerRunning.store(true);
    unsigned hc = std::thread::hardware_concurrency();
    if (hc > 0) {
        unsigned targetWorkers = std::max(6u, (hc * 3) / 4);  // 75% of cores, minimum 6
        workerCount = static_cast<int>(targetWorkers);
    } else {
        workerCount = 6;  // Fallback
    }
    workerThreads.reserve(workerCount);
    for (int i = 0; i < workerCount; i++) {
        workerThreads.emplace_back(&TerrainManager::workerLoop, this);
    }
}

TileCoord TerrainManager::worldToTile(float glX, float glY) const {
    auto [tileX, tileY] = core::coords::worldToTile(glX, glY);
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
    return "World\\Maps\\" + mapName + "\\" + mapName + "_" +
           std::to_string(coord.x) + "_" + std::to_string(coord.y) + ".adt";
}

void TerrainManager::ensureGroundEffectTablesLoaded() {
    if (groundEffectsLoaded_ || !assetManager) return;
    groundEffectsLoaded_ = true;

    auto groundEffectTex = assetManager->loadDBC("GroundEffectTexture.dbc");
    auto groundEffectDoodad = assetManager->loadDBC("GroundEffectDoodad.dbc");
    if (!groundEffectTex || !groundEffectDoodad) {
        LOG_WARNING("Ground clutter DBCs missing; skipping procedural ground effects");
        return;
    }

    // GroundEffectTexture: id + 4 doodad IDs + 4 weights + density + sound
    for (uint32_t i = 0; i < groundEffectTex->getRecordCount(); ++i) {
        uint32_t effectId = groundEffectTex->getUInt32(i, 0);
        if (effectId == 0) continue;

        GroundEffectEntry e;
        e.doodadIds[0] = groundEffectTex->getUInt32(i, 1);
        e.doodadIds[1] = groundEffectTex->getUInt32(i, 2);
        e.doodadIds[2] = groundEffectTex->getUInt32(i, 3);
        e.doodadIds[3] = groundEffectTex->getUInt32(i, 4);
        e.weights[0] = groundEffectTex->getUInt32(i, 5);
        e.weights[1] = groundEffectTex->getUInt32(i, 6);
        e.weights[2] = groundEffectTex->getUInt32(i, 7);
        e.weights[3] = groundEffectTex->getUInt32(i, 8);
        e.density = groundEffectTex->getUInt32(i, 9);
        groundEffectById_[effectId] = e;
    }

    // GroundEffectDoodad: id + modelName(offset) + flags
    for (uint32_t i = 0; i < groundEffectDoodad->getRecordCount(); ++i) {
        uint32_t doodadId = groundEffectDoodad->getUInt32(i, 0);
        std::string modelName = groundEffectDoodad->getString(i, 1);
        if (doodadId == 0 || modelName.empty()) continue;

        std::string lower = toLowerCopy(modelName);
        if (lower.size() > 4 && lower.substr(lower.size() - 4) == ".mdl") {
            lower = lower.substr(0, lower.size() - 4) + ".m2";
        }
        if (lower.find('\\') != std::string::npos || lower.find('/') != std::string::npos) {
            groundDoodadModelById_[doodadId] = lower;
        } else {
            groundDoodadModelById_[doodadId] = "World\\NoDXT\\Detail\\" + lower;
        }
    }

    LOG_INFO("Ground clutter tables loaded: ", groundEffectById_.size(),
             " effects, ", groundDoodadModelById_.size(), " doodad models");
}

void TerrainManager::generateGroundClutterPlacements(std::shared_ptr<PendingTile>& pending,
                                                     std::unordered_set<uint32_t>& preparedModelIds) {
    if (taxiStreamingMode_) return;  // Skip clutter while on taxi flights.
    if (!pending || groundEffectById_.empty() || groundDoodadModelById_.empty()) return;

    static const std::string kGroundClutterProxyModel = "World\\NoDXT\\Detail\\ElwGra01.m2";
    static bool loggedProxy = false;
    if (!loggedProxy) {
        LOG_INFO("Ground clutter: forcing proxy model ", kGroundClutterProxyModel);
        loggedProxy = true;
    }

    size_t modelMissing = 0;
    size_t modelInvalid = 0;
    auto ensureModelPrepared = [&](const std::string& m2Path, uint32_t modelId) -> bool {
        if (preparedModelIds.count(modelId)) return true;

        std::vector<uint8_t> m2Data = assetManager->readFile(m2Path);
        if (m2Data.empty()) {
            modelMissing++;
            return false;
        }

        pipeline::M2Model m2Model = pipeline::M2Loader::load(m2Data);
        if (m2Model.name.empty()) {
            m2Model.name = m2Path;
        }
        std::string skinPath = m2Path.substr(0, m2Path.size() - 3) + "00.skin";
        std::vector<uint8_t> skinData = assetManager->readFileOptional(skinPath);
        if (!skinData.empty() && m2Model.version >= 264) {
            pipeline::M2Loader::loadSkin(skinData, m2Model);
        }
        if (!m2Model.isValid()) {
            modelInvalid++;
            return false;
        }

        PendingTile::M2Ready ready;
        ready.modelId = modelId;
        ready.model = std::move(m2Model);
        ready.path = m2Path;
        pending->m2Models.push_back(std::move(ready));
        preparedModelIds.insert(modelId);
        return true;
    };

    constexpr float unitSize = CHUNK_SIZE / 8.0f;
    constexpr float pi = 3.1415926535f;
    constexpr size_t kBaseMaxGroundClutterPerTile = 220;
    constexpr uint32_t kBaseMaxAttemptsPerLayer = 4;
    const float densityScaleRaw = glm::clamp(groundClutterDensityScale_, 0.0f, 1.5f);
    // Keep runtime density bounded to avoid large streaming spikes in dense tiles.
    const float densityScale = std::min(densityScaleRaw, 1.0f);
    const size_t kMaxGroundClutterPerTile = std::max<size_t>(
        0, static_cast<size_t>(std::lround(static_cast<float>(kBaseMaxGroundClutterPerTile) * densityScale)));
    const uint32_t kMaxAttemptsPerLayer = std::max<uint32_t>(
        1u, static_cast<uint32_t>(std::lround(static_cast<float>(kBaseMaxAttemptsPerLayer) * densityScale)));
    std::vector<uint8_t> alphaScratch;
    std::vector<uint8_t> alphaScratchTex;
    size_t added = 0;
    size_t attemptsTotal = 0;
    size_t alphaRejected = 0;
    size_t roadRejected = 0;
    size_t noEffectMatch = 0;
    size_t textureIdFallbackMatch = 0;
    size_t noDoodadModel = 0;
    std::array<uint16_t, 256> perChunkAdded{};

    auto isRoadLikeTexture = [](const std::string& texPath) -> bool {
        std::string t = toLowerCopy(texPath);
        return (t.find("road") != std::string::npos) ||
               (t.find("cobble") != std::string::npos) ||
               (t.find("path") != std::string::npos) ||
               (t.find("street") != std::string::npos) ||
               (t.find("pavement") != std::string::npos) ||
               (t.find("brick") != std::string::npos);
    };

    auto layerWeightAt = [&](const pipeline::MapChunk& chunk, size_t layerIdx, int alphaIndex) -> int {
        if (layerIdx >= chunk.layers.size()) return 0;
        if (layerIdx == 0) {
            int accum = 0;
            size_t numLayers = std::min(chunk.layers.size(), static_cast<size_t>(4));
            for (size_t i = 1; i < numLayers; ++i) {
                int a = 0;
                if (decodeLayerAlpha(chunk, i, alphaScratchTex) &&
                    alphaIndex >= 0 &&
                    alphaIndex < static_cast<int>(alphaScratchTex.size())) {
                    a = alphaScratchTex[alphaIndex];
                }
                accum += a;
            }
            return glm::clamp(255 - accum, 0, 255);
        }
        if (decodeLayerAlpha(chunk, layerIdx, alphaScratchTex) &&
            alphaIndex >= 0 &&
            alphaIndex < static_cast<int>(alphaScratchTex.size())) {
            return alphaScratchTex[alphaIndex];
        }
        return 0;
    };

    auto hasRoadLikeTextureAt = [&](const pipeline::MapChunk& chunk, float fracX, float fracY) -> bool {
        if (chunk.layers.empty()) return false;
        int alphaX = glm::clamp(static_cast<int>((fracX / 8.0f) * 63.0f), 0, 63);
        int alphaY = glm::clamp(static_cast<int>((fracY / 8.0f) * 63.0f), 0, 63);
        int alphaIndex = alphaY * 64 + alphaX;

        size_t numLayers = std::min(chunk.layers.size(), static_cast<size_t>(4));
        for (size_t layerIdx = 0; layerIdx < numLayers; ++layerIdx) {
            uint32_t texId = chunk.layers[layerIdx].textureId;
            if (texId >= pending->terrain.textures.size()) continue;
            const std::string& texPath = pending->terrain.textures[texId];
            if (!isRoadLikeTexture(texPath)) continue;
            // Treat meaningful blend contribution as road occupancy.
            int w = layerWeightAt(chunk, layerIdx, alphaIndex);
            if (w >= 24) return true;
        }
        return false;
    };

    for (int cy = 0; cy < 16; ++cy) {
        if (added >= kMaxGroundClutterPerTile) break;
        for (int cx = 0; cx < 16; ++cx) {
            if (added >= kMaxGroundClutterPerTile) break;
            const auto& chunk = pending->terrain.getChunk(cx, cy);
            if (!chunk.hasHeightMap() || chunk.layers.empty()) continue;

            for (size_t layerIdx = 0; layerIdx < chunk.layers.size(); ++layerIdx) {
                if (added >= kMaxGroundClutterPerTile) break;
                const auto& layer = chunk.layers[layerIdx];
                if (layer.effectId == 0) continue;

                auto geIt = groundEffectById_.find(layer.effectId);
                if (geIt == groundEffectById_.end() && layer.textureId != 0) {
                    geIt = groundEffectById_.find(layer.textureId);
                    if (geIt != groundEffectById_.end()) {
                        textureIdFallbackMatch++;
                    }
                }
                if (geIt == groundEffectById_.end()) {
                    noEffectMatch++;
                    continue;
                }
                const GroundEffectEntry& ge = geIt->second;

                uint32_t totalWeight = ge.weights[0] + ge.weights[1] + ge.weights[2] + ge.weights[3];
                if (totalWeight == 0) totalWeight = 4;

                uint32_t density = std::min<uint32_t>(ge.density, 16u);
                density = static_cast<uint32_t>(std::lround(static_cast<float>(density) * densityScale));
                if (density == 0) continue;
                uint32_t attempts = std::max<uint32_t>(3u, density * 2u);
                attempts = std::min<uint32_t>(attempts, kMaxAttemptsPerLayer);
                attemptsTotal += attempts;

                bool hasAlpha = decodeLayerAlpha(chunk, layerIdx, alphaScratch);
                uint32_t seed = static_cast<uint32_t>(
                    ((pending->coord.x & 0xFF) << 24) ^
                    ((pending->coord.y & 0xFF) << 16) ^
                    ((cx & 0x1F) << 8) ^
                    ((cy & 0x1F) << 3) ^
                    (layerIdx & 0x7));
                auto nextRand = [&seed]() -> uint32_t {
                    seed = seed * 1664525u + 1013904223u;
                    return seed;
                };

                for (uint32_t a = 0; a < attempts; ++a) {
                    float fracX = (nextRand() & 0xFFFFu) / 65535.0f * 8.0f;
                    float fracY = (nextRand() & 0xFFFFu) / 65535.0f * 8.0f;

                    if (hasAlpha && !alphaScratch.empty()) {
                        int alphaX = glm::clamp(static_cast<int>((fracX / 8.0f) * 63.0f), 0, 63);
                        int alphaY = glm::clamp(static_cast<int>((fracY / 8.0f) * 63.0f), 0, 63);
                        int alphaIndex = alphaY * 64 + alphaX;
                        if (alphaIndex < 0 || alphaIndex >= static_cast<int>(alphaScratch.size())) continue;
                        if (alphaScratch[alphaIndex] < 64) {
                            alphaRejected++;
                            continue;
                        }
                    }

                    if (hasRoadLikeTextureAt(chunk, fracX, fracY)) {
                        roadRejected++;
                        continue;
                    }

                    uint32_t roll = nextRand() % totalWeight;
                    int pick = 0;
                    uint32_t acc = 0;
                    for (int i = 0; i < 4; ++i) {
                        uint32_t w = ge.weights[i] > 0 ? ge.weights[i] : 1;
                        acc += w;
                        if (roll < acc) { pick = i; break; }
                    }
                    uint32_t doodadId = ge.doodadIds[pick];
                    if (doodadId == 0) continue;

                    auto doodadIt = groundDoodadModelById_.find(doodadId);
                    if (doodadIt == groundDoodadModelById_.end()) {
                        noDoodadModel++;
                        continue;
                    }
                    const std::string& doodadModelPath = doodadIt->second;
                    uint32_t modelId = static_cast<uint32_t>(std::hash<std::string>{}(doodadModelPath));
                    if (!ensureModelPrepared(doodadModelPath, modelId)) {
                        modelId = static_cast<uint32_t>(std::hash<std::string>{}(kGroundClutterProxyModel));
                        if (!ensureModelPrepared(kGroundClutterProxyModel, modelId)) {
                            continue;
                        }
                    }

                    float worldX = chunk.position[0] - fracY * unitSize;
                    float worldY = chunk.position[1] - fracX * unitSize;

                    int gx0 = glm::clamp(static_cast<int>(std::floor(fracX)), 0, 8);
                    int gy0 = glm::clamp(static_cast<int>(std::floor(fracY)), 0, 8);
                    int gx1 = std::min(gx0 + 1, 8);
                    int gy1 = std::min(gy0 + 1, 8);
                    float tx = fracX - static_cast<float>(gx0);
                    float ty = fracY - static_cast<float>(gy0);
                    float h00 = chunk.heightMap.getHeight(gx0, gy0);
                    float h10 = chunk.heightMap.getHeight(gx1, gy0);
                    float h01 = chunk.heightMap.getHeight(gx0, gy1);
                    float h11 = chunk.heightMap.getHeight(gx1, gy1);
                    float worldZ = chunk.position[2] +
                                 (h00 * (1 - tx) * (1 - ty) +
                                  h10 * tx * (1 - ty) +
                                  h01 * (1 - tx) * ty +
                                  h11 * tx * ty);

                    PendingTile::M2Placement p;
                    p.modelId = modelId;
                    p.uniqueId = 0;
                    // MCNK chunk.position is already in terrain/render world space.
                    // Do not convert via ADT placement mapping (that is for MDDF/MODF records).
                    p.rotation = glm::vec3(0.0f, 0.0f, (nextRand() & 0xFFFFu) / 65535.0f * (2.0f * pi));
                    p.scale = 0.80f + ((nextRand() & 0xFFFFu) / 65535.0f) * 0.35f;
                    // Snap directly to sampled terrain height.
                    p.position = glm::vec3(worldX, worldY, worldZ + 0.01f);
                    pending->m2Placements.push_back(p);
                    added++;
                    perChunkAdded[cy * 16 + cx]++;
                    if (added >= kMaxGroundClutterPerTile) break;
                }
            }
        }
    }

    size_t fallbackAdded = 0;
    const size_t kMinGroundClutterPerTile = static_cast<size_t>(std::lround(40.0f * densityScale));
    size_t fallbackNeeded = (added < kMinGroundClutterPerTile) ? (kMinGroundClutterPerTile - added) : 0;
    if (fallbackNeeded > 0) {
        const uint32_t proxyModelId = static_cast<uint32_t>(std::hash<std::string>{}(kGroundClutterProxyModel));
        if (ensureModelPrepared(kGroundClutterProxyModel, proxyModelId)) {
            constexpr uint32_t kFallbackPerChunk = 2;
            for (int cy = 0; cy < 16; ++cy) {
                for (int cx = 0; cx < 16; ++cx) {
                    if (fallbackAdded >= fallbackNeeded || added >= kMaxGroundClutterPerTile) break;
                    const auto& chunk = pending->terrain.getChunk(cx, cy);
                    if (!chunk.hasHeightMap()) continue;

                    for (uint32_t i = 0; i < kFallbackPerChunk; ++i) {
                        if (fallbackAdded >= fallbackNeeded || added >= kMaxGroundClutterPerTile) break;
                        // Deterministic scatter so the tile stays visually stable.
                        uint32_t seed = static_cast<uint32_t>(
                            ((pending->coord.x & 0xFF) << 24) ^
                            ((pending->coord.y & 0xFF) << 16) ^
                            ((cx & 0x1F) << 8) ^
                            ((cy & 0x1F) << 3) ^
                            (i & 0x7));
                        auto nextRand = [&seed]() -> uint32_t {
                            seed = seed * 1664525u + 1013904223u;
                            return seed;
                        };

                        float fracX = (nextRand() & 0xFFFFu) / 65535.0f * 8.0f;
                        float fracY = (nextRand() & 0xFFFFu) / 65535.0f * 8.0f;
                        if (hasRoadLikeTextureAt(chunk, fracX, fracY)) {
                            roadRejected++;
                            continue;
                        }
                        float worldX = chunk.position[0] - fracY * unitSize;
                        float worldY = chunk.position[1] - fracX * unitSize;

                        int gx0 = glm::clamp(static_cast<int>(std::floor(fracX)), 0, 8);
                        int gy0 = glm::clamp(static_cast<int>(std::floor(fracY)), 0, 8);
                        int gx1 = std::min(gx0 + 1, 8);
                        int gy1 = std::min(gy0 + 1, 8);
                        float tx = fracX - static_cast<float>(gx0);
                        float ty = fracY - static_cast<float>(gy0);
                        float h00 = chunk.heightMap.getHeight(gx0, gy0);
                        float h10 = chunk.heightMap.getHeight(gx1, gy0);
                        float h01 = chunk.heightMap.getHeight(gx0, gy1);
                        float h11 = chunk.heightMap.getHeight(gx1, gy1);
                        float worldZ = chunk.position[2] +
                                     (h00 * (1 - tx) * (1 - ty) +
                                      h10 * tx * (1 - ty) +
                                      h01 * (1 - tx) * ty +
                                      h11 * tx * ty);

                        PendingTile::M2Placement p;
                        p.modelId = proxyModelId;
                        p.uniqueId = 0;
                        p.rotation = glm::vec3(0.0f, 0.0f, (nextRand() & 0xFFFFu) / 65535.0f * (2.0f * pi));
                        p.scale = 0.75f + ((nextRand() & 0xFFFFu) / 65535.0f) * 0.40f;
                        p.position = glm::vec3(worldX, worldY, worldZ + 0.01f);
                        pending->m2Placements.push_back(p);
                        fallbackAdded++;
                        added++;
                        perChunkAdded[cy * 16 + cx]++;
                    }
                }
                if (fallbackAdded >= fallbackNeeded || added >= kMaxGroundClutterPerTile) break;
            }
        }
    }

    // Baseline pass disabled: one-per-chunk fill caused large instance spikes and hitches
    // when streaming tiles around the player.
    size_t baselineAdded = 0;

    if (added > 0) {
        static int clutterLogCount = 0;
        if (clutterLogCount < 12) {
            LOG_INFO("Ground clutter tile [", pending->coord.x, ",", pending->coord.y,
                     "] added=", added, " attempts=", attemptsTotal,
                     " fallbackAdded=", fallbackAdded,
                     " baselineAdded=", baselineAdded,
                     " roadRejected=", roadRejected);
            clutterLogCount++;
        }
    } else {
        static int noClutterLogCount = 0;
        if (noClutterLogCount < 8) {
            LOG_INFO("Ground clutter tile [", pending->coord.x, ",", pending->coord.y,
                     "] added=0 attempts=", attemptsTotal,
                     " alphaRejected=", alphaRejected,
                     " roadRejected=", roadRejected,
                     " noEffect=", noEffectMatch,
                     " textureFallback=", textureIdFallbackMatch,
                     " noDoodadModel=", noDoodadModel,
                     " modelMissing=", modelMissing,
                     " modelInvalid=", modelInvalid);
            noClutterLogCount++;
        }
    }
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
    auto shouldSkipMissingAdt = [this](const TileCoord& coord) -> bool {
        if (!assetManager) return false;
        if (failedTiles.find(coord) != failedTiles.end()) return true;
        const std::string adtPath = getADTPath(coord);
        if (!assetManager->fileExists(adtPath)) {
            // Mark permanently failed so future stream/precache passes do not retry.
            failedTiles[coord] = true;
            return true;
        }
        return false;
    };

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

                // Circular pattern: skip corner tiles beyond radius (Euclidean distance)
                if (dx*dx + dy*dy > loadRadius*loadRadius) {
                    continue;
                }

                TileCoord coord = {tileX, tileY};

                // Skip if already loaded, pending, or failed
                if (loadedTiles.find(coord) != loadedTiles.end()) continue;
                if (pendingTiles.find(coord) != pendingTiles.end()) continue;
                if (failedTiles.find(coord) != failedTiles.end()) continue;
                if (shouldSkipMissingAdt(coord)) continue;

                loadQueue.push_back(coord);
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

        int dx = coord.x - currentTile.x;
        int dy = coord.y - currentTile.y;

        // Circular pattern: unload beyond radius (Euclidean distance)
        if (dx*dx + dy*dy > unloadRadius*unloadRadius) {
            tilesToUnload.push_back(coord);
        }
    }

    for (const auto& coord : tilesToUnload) {
        unloadTile(coord.x, coord.y);
    }

    if (!tilesToUnload.empty()) {
        // Don't clean up models during streaming - keep them in VRAM for performance
        // Modern GPUs have 8-16GB VRAM, models are only ~hundreds of MB
        // Cleanup can be done manually when memory pressure is detected
        // NOTE: Disabled permanent model cleanup to leverage modern VRAM capacity
        // if (m2Renderer) {
        //     m2Renderer->cleanupUnusedModels();
        // }
        // if (wmoRenderer) {
        //     wmoRenderer->cleanupUnusedModels();
        // }

        LOG_INFO("Unloaded ", tilesToUnload.size(), " distant tiles, ",
                 loadedTiles.size(), " remain (models kept in VRAM)");
    }
}

void TerrainManager::precacheTiles(const std::vector<std::pair<int, int>>& tiles) {
    std::lock_guard<std::mutex> lock(queueMutex);

    for (const auto& [x, y] : tiles) {
        if (x < 0 || x > 63 || y < 0 || y > 63) continue;

        TileCoord coord = {x, y};

        // Skip if already loaded, pending, or failed
        if (loadedTiles.find(coord) != loadedTiles.end()) continue;
        if (pendingTiles.find(coord) != pendingTiles.end()) continue;
        if (failedTiles.find(coord) != failedTiles.end()) continue;
        if (assetManager && !assetManager->fileExists(getADTPath(coord))) {
            failedTiles[coord] = true;
            continue;
        }

        // Precache work is prioritized so taxi-route tiles are prepared before
        // opportunistic radius streaming tiles.
        loadQueue.push_front(coord);
        pendingTiles[coord] = true;
    }

    // Notify workers to start loading
    queueCV.notify_all();
}

} // namespace rendering
} // namespace wowee
