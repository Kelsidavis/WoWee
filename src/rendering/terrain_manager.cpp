#include "rendering/terrain_manager.hpp"
#include "rendering/terrain_renderer.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/water_renderer.hpp"
#include "rendering/m2_renderer.hpp"
#include "rendering/m2_model_classifier.hpp"
#include "rendering/wmo_renderer.hpp"
#include "rendering/camera.hpp"
#include "audio/ambient_sound_manager.hpp"
#include "core/coordinates.hpp"
#include "pipeline/wowee_terrain_loader.hpp"
#include "pipeline/wowee_model.hpp"
#include "pipeline/wowee_building.hpp"
#include "pipeline/wowee_collision.hpp"
#include "core/memory_monitor.hpp"
#include "core/profiler.hpp"
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
#include <cstdlib>
#include <functional>
#include <unordered_set>

#ifdef __linux__
#include <sched.h>
#include <pthread.h>
#elif defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#elif defined(__APPLE__)
#include <mach/mach.h>
#include <mach/thread_policy.h>
#include <pthread.h>
#endif

namespace wowee {
namespace rendering {

namespace {

// Alpha map format constants
constexpr size_t  ALPHA_MAP_SIZE    = 4096;  // 64×64 uncompressed alpha bytes
constexpr size_t  ALPHA_MAP_PACKED  = 2048;  // 64×64 packed 4-bit alpha (half size)
static_assert(ALPHA_MAP_PACKED * 2 == ALPHA_MAP_SIZE, "packed alpha must unpack to full size");
constexpr uint8_t ALPHA_FILL_FLAG   = 0x80;  // RLE command: fill vs. copy
constexpr uint8_t ALPHA_COUNT_MASK  = 0x7F;  // RLE command: count bits

// Random float normalization: mask to 16-bit then divide by max value to get [0..1]
constexpr float kRand16Max = 65535.0f;

// Placement transform constants
constexpr float kDegToRad = 3.14159f / 180.0f;
constexpr float kInv1024  = 1.0f / 1024.0f;

int computeTerrainWorkerCount() {
    const char* raw = std::getenv("WOWEE_TERRAIN_WORKERS");
    if (raw && *raw) {
        char* end = nullptr;
        unsigned long long forced = std::strtoull(raw, &end, 10);
        if (end != raw && forced > 0) {
            return static_cast<int>(forced);
        }
    }

    unsigned hc = std::thread::hardware_concurrency();
    if (hc > 0) {
        // Keep terrain workers conservative by default. Over-subscribing loader
        // threads can starve main-thread networking/render updates on large-core CPUs.
        const unsigned reserved = (hc >= 16u) ? 4u : ((hc >= 8u) ? 2u : 1u);
        const unsigned maxDefaultWorkers = 8u;
        const unsigned targetWorkers = std::max(4u, std::min(maxDefaultWorkers, hc - reserved));
        return static_cast<int>(targetWorkers);
    }
    return 4;  // Fallback
}

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

    outAlpha.assign(ALPHA_MAP_SIZE, 255);

    if (layer.compressedAlpha()) {
        size_t readPos = offset;
        size_t writePos = 0;
        while (writePos < ALPHA_MAP_SIZE && readPos < chunk.alphaMap.size()) {
            uint8_t cmd = chunk.alphaMap[readPos++];
            bool fill = (cmd & ALPHA_FILL_FLAG) != 0;
            int count = (cmd & ALPHA_COUNT_MASK) + 1;

            if (fill) {
                if (readPos >= chunk.alphaMap.size()) break;
                uint8_t val = chunk.alphaMap[readPos++];
                for (int i = 0; i < count && writePos < ALPHA_MAP_SIZE; i++) {
                    outAlpha[writePos++] = val;
                }
            } else {
                for (int i = 0; i < count && writePos < ALPHA_MAP_SIZE && readPos < chunk.alphaMap.size(); i++) {
                    outAlpha[writePos++] = chunk.alphaMap[readPos++];
                }
            }
        }
        return true;
    }

    if (layerSize >= ALPHA_MAP_SIZE) {
        std::copy(chunk.alphaMap.begin() + offset, chunk.alphaMap.begin() + offset + ALPHA_MAP_SIZE, outAlpha.begin());
        return true;
    }

    if (layerSize >= ALPHA_MAP_PACKED) {
        for (size_t i = 0; i < ALPHA_MAP_PACKED; i++) {
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
    stopWorkers();
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
    // Keep defaults moderate; env override can increase if streaming is bottlenecked.
    workerRunning.store(true);
    workerCount = computeTerrainWorkerCount();
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
    ZoneScopedN("TerrainManager::update");
    if (!streamingEnabled || !assetManager || !terrainRenderer) {
        return;
    }

    // Phase timing: this call stalls for ~180ms when tiles arrive, and the
    // caller only sees one number. Both processReadyTiles and
    // processPendingUnloads claim to be time-budgeted internally, so knowing
    // which phase actually runs long says whether a budget is being exceeded or
    // whether the cost is in streamTiles enumerating the world.
    using clock = std::chrono::steady_clock;
    const auto tStart = clock::now();
    auto elapsedMs = [](clock::time_point a, clock::time_point b) {
        return std::chrono::duration<float, std::milli>(b - a).count();
    };
    float reconcileMs = 0.0f, readyMs = 0.0f, unloadMs = 0.0f, streamMs = 0.0f;
    // Reports the breakdown on the way out of any long call, whichever return
    // path is taken.
    struct PhaseReport {
        clock::time_point start;
        const float *reconcile, *ready, *unload, *stream;
        ~PhaseReport() {
            const float total = std::chrono::duration<float, std::milli>(
                clock::now() - start).count();
            if (total > 50.0f) {
                LOG_WARNING("SLOW terrain update ", total, "ms: reconcile=", *reconcile,
                            " readyTiles=", *ready, " unloads=", *unload,
                            " streamTiles=", *stream);
            }
        }
    } report{tStart, &reconcileMs, &readyMs, &unloadMs, &streamMs};

    // Reconcile the "already uploaded" cache against models the renderer reaped
    // for being instanceless. Without this, a model freed after leaving an area
    // stays marked uploaded, so the next tile prep skips its load and pushes an
    // empty placeholder — doodads (e.g. Stormwind tunnel torches) then fail to
    // spawn on revisit with "M2 model has no renderable content".
    if (m2Renderer) {
        std::vector<uint32_t> reaped = m2Renderer->drainReapedModelIds();
        if (!reaped.empty()) {
            std::lock_guard<std::mutex> lock(uploadedM2IdsMutex_);
            for (uint32_t id : reaped) uploadedM2Ids_.erase(id);
        }
    }

    // Always process ready tiles each frame (GPU uploads from background thread)
    // Time-budgeted internally to prevent frame spikes.
    const auto tReconcile = clock::now();
    reconcileMs = elapsedMs(tStart, tReconcile);
    processReadyTiles();
    readyMs = elapsedMs(tReconcile, clock::now());

    // Always drain a bounded batch of pending unloads each frame — same
    // frame-spike rationale as processReadyTiles() above.
    const auto tUnloadStart = clock::now();
    processPendingUnloads();
    unloadMs = elapsedMs(tUnloadStart, clock::now());

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

    // Stream tiles when player crosses a tile boundary
    if (newTile.x != lastStreamTile.x || newTile.y != lastStreamTile.y) {
        LOG_DEBUG("Streaming: cam=(", camPos.x, ",", camPos.y, ",", camPos.z,
                 ") tile=[", newTile.x, ",", newTile.y,
                 "] loaded=", loadedTiles.size());
        const auto tStream = clock::now();
        streamTiles();
        streamMs = elapsedMs(tStream, clock::now());
        lastStreamTile = newTile;
    } else {
        // Proactive loading: when workers are idle, periodically re-check for
        // unloaded tiles within range. Throttled to avoid hitching right after
        // world load when many tiles finalize simultaneously.
        proactiveStreamTimer_ += deltaTime;
        if (proactiveStreamTimer_ >= 2.0f) {
            proactiveStreamTimer_ = 0.0f;
            bool workersIdle;
            {
                std::lock_guard<std::mutex> lock(queueMutex);
                workersIdle = loadQueue.empty();
            }
            if (workersIdle) {
                const auto tStream = clock::now();
                streamTiles();
                streamMs = elapsedMs(tStream, clock::now());
            }
        }
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

    VkContext* vkCtx = terrainRenderer ? terrainRenderer->getVkContext() : nullptr;
    if (vkCtx) vkCtx->beginUploadBatch();

    FinalizingTile ft;
    ft.pending = std::move(pending);
    while (!advanceFinalization(ft)) {}

    if (vkCtx) vkCtx->endUploadBatchSync();  // Sync — caller expects tile ready
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

    // Early-exit check — worker should bail fast during shutdown
    if (!workerRunning.load()) return nullptr;

    // Try Wowee Open Terrain format first (custom zones)
    std::string wotBase = "custom_zones/" + mapName + "/" + mapName + "_" +
                          std::to_string(coord.x) + "_" + std::to_string(coord.y);
    auto terrainPtr = std::make_unique<pipeline::ADTTerrain>();
    bool loadedFromWot = false;

    if (pipeline::WoweeTerrainLoader::exists(wotBase)) {
        if (pipeline::WoweeTerrainLoader::load(wotBase, *terrainPtr)) {
            loadedFromWot = true;
            LOG_INFO("Loaded custom zone terrain: ", wotBase);
            // Load collision mesh if available
            if (pipeline::WoweeCollisionBuilder::exists(wotBase)) {
                auto woc = pipeline::WoweeCollisionBuilder::load(wotBase + ".woc");
                if (woc.isValid()) {
                    CollisionData cd;
                    cd.triangles.reserve(woc.triangles.size());
                    for (const auto& t : woc.triangles)
                        cd.triangles.push_back({t.v0, t.v1, t.v2, t.flags});
                    cd.boundsMin = woc.bounds.min;
                    cd.boundsMax = woc.bounds.max;
                    cd.loaded = true;
                    collisionTiles_[tileKey(coord.x, coord.y)] = std::move(cd);
                    LOG_INFO("Loaded WOC collision: ", woc.triangles.size(), " triangles");
                }
            }
        }
    }

    // Also check output directory (editor exports here)
    if (!loadedFromWot) {
        std::string outputBase = "output/" + mapName + "/" + mapName + "_" +
                                 std::to_string(coord.x) + "_" + std::to_string(coord.y);
        if (pipeline::WoweeTerrainLoader::exists(outputBase)) {
            if (pipeline::WoweeTerrainLoader::load(outputBase, *terrainPtr)) {
                loadedFromWot = true;
                LOG_INFO("Loaded editor output terrain: ", outputBase);
                if (pipeline::WoweeCollisionBuilder::exists(outputBase)) {
                    auto woc = pipeline::WoweeCollisionBuilder::load(outputBase + ".woc");
                    if (woc.isValid()) {
                        CollisionData cd;
                        cd.triangles.reserve(woc.triangles.size());
                        for (const auto& t : woc.triangles)
                            cd.triangles.push_back({t.v0, t.v1, t.v2, t.flags});
                        cd.boundsMin = woc.bounds.min;
                        cd.boundsMax = woc.bounds.max;
                        cd.loaded = true;
                        collisionTiles_[tileKey(coord.x, coord.y)] = std::move(cd);
                        LOG_INFO("Loaded WOC collision: ", woc.triangles.size(), " triangles");
                    }
                }
            }
        }
    }

    // Try WHM/WOT sidecar from the asset tree (asset_extract --emit-terrain
    // writes one alongside the ADT). This lets the runtime use the open
    // format without copying anything into custom_zones/.
    if (!loadedFromWot) {
        std::string adtPath = getADTPath(coord);
        std::string adtFsPath = assetManager->resolveFile(adtPath);
        if (!adtFsPath.empty() && adtFsPath.size() >= 4) {
            std::string sidecarBase = adtFsPath.substr(0, adtFsPath.size() - 4);
            if (pipeline::WoweeTerrainLoader::exists(sidecarBase) &&
                pipeline::WoweeTerrainLoader::load(sidecarBase, *terrainPtr)) {
                loadedFromWot = true;
                LOG_INFO("Loaded asset-tree WHM/WOT sidecar: ", sidecarBase);
                if (pipeline::WoweeCollisionBuilder::exists(sidecarBase)) {
                    auto woc = pipeline::WoweeCollisionBuilder::load(sidecarBase + ".woc");
                    if (woc.isValid()) {
                        CollisionData cd;
                        cd.triangles.reserve(woc.triangles.size());
                        for (const auto& t : woc.triangles)
                            cd.triangles.push_back({t.v0, t.v1, t.v2, t.flags});
                        cd.boundsMin = woc.bounds.min;
                        cd.boundsMax = woc.bounds.max;
                        cd.loaded = true;
                        collisionTiles_[tileKey(coord.x, coord.y)] = std::move(cd);
                        LOG_INFO("Loaded sidecar WOC collision: ",
                                 woc.triangles.size(), " triangles");
                    }
                }
            }
        }
    }

    // Fall back to ADT format
    if (!loadedFromWot) {
        std::string adtPath = getADTPath(coord);
        auto adtData = assetManager->readFile(adtPath);

        if (adtData.empty()) {
            logMissingAdtOnce(adtPath);
            return nullptr;
        }

        *terrainPtr = pipeline::ADTLoader::load(adtData);
        if (!terrainPtr->isLoaded()) {
            LOG_ERROR("Failed to parse ADT terrain: ", adtPath);
            return nullptr;
        }
    }

    if (!workerRunning.load()) return nullptr;

    // WotLK split ADTs can store placements in *_obj0.adt.
    // Only needed for ADT-loaded tiles, not for WOT custom zones.
    if (!loadedFromWot) {
    std::string objPath = "World\\Maps\\" + mapName + "\\" + mapName + "_" +
                          std::to_string(coord.x) + "_" + std::to_string(coord.y) + "_obj0.adt";
    auto objData = assetManager->readFile(objPath);
    if (!objData.empty()) {
        auto objTerrain = std::make_unique<pipeline::ADTTerrain>(pipeline::ADTLoader::load(objData));
        if (objTerrain->isLoaded()) {
            const uint32_t doodadNameBase = static_cast<uint32_t>(terrainPtr->doodadNames.size());
            const uint32_t wmoNameBase = static_cast<uint32_t>(terrainPtr->wmoNames.size());

            terrainPtr->doodadNames.insert(terrainPtr->doodadNames.end(),
                                       objTerrain->doodadNames.begin(), objTerrain->doodadNames.end());
            terrainPtr->wmoNames.insert(terrainPtr->wmoNames.end(),
                                    objTerrain->wmoNames.begin(), objTerrain->wmoNames.end());

            std::unordered_set<uint32_t> existingDoodadUniqueIds;
            existingDoodadUniqueIds.reserve(terrainPtr->doodadPlacements.size());
            for (const auto& p : terrainPtr->doodadPlacements) {
                if (p.uniqueId != 0) existingDoodadUniqueIds.insert(p.uniqueId);
            }

            size_t mergedDoodads = 0;
            for (auto placement : objTerrain->doodadPlacements) {
                if (placement.nameId >= objTerrain->doodadNames.size()) continue;
                placement.nameId += doodadNameBase;
                if (placement.uniqueId != 0 && !existingDoodadUniqueIds.insert(placement.uniqueId).second) {
                    continue;
                }
                terrainPtr->doodadPlacements.push_back(placement);
                mergedDoodads++;
            }

            std::unordered_set<uint32_t> existingWmoUniqueIds;
            existingWmoUniqueIds.reserve(terrainPtr->wmoPlacements.size());
            for (const auto& p : terrainPtr->wmoPlacements) {
                if (p.uniqueId != 0) existingWmoUniqueIds.insert(p.uniqueId);
            }

            size_t mergedWmos = 0;
            for (auto placement : objTerrain->wmoPlacements) {
                if (placement.nameId >= objTerrain->wmoNames.size()) continue;
                placement.nameId += wmoNameBase;
                if (placement.uniqueId != 0 && !existingWmoUniqueIds.insert(placement.uniqueId).second) {
                    continue;
                }
                terrainPtr->wmoPlacements.push_back(placement);
                mergedWmos++;
            }

            if (mergedDoodads > 0 || mergedWmos > 0) {
                LOG_DEBUG("Merged obj0 tile [", x, ",", y, "]: +", mergedDoodads,
                          " doodads, +", mergedWmos, " WMOs");
            }
        }
    }
    } // end if (!loadedFromWot) obj0 merge

    // Set tile coordinates so mesh knows where to position this tile in world
    terrainPtr->coord.x = x;
    terrainPtr->coord.y = y;

    // The stock Azeroth ADT leaves two terrain-hole cells exposed along the
    // exterior edge of the Westfall lumbermill. The original client hides the
    // mask beneath the building footprint, but our WMO floor ends just short
    // of it, producing a small textureless strip between the inn/prison camp
    // and lumber yard. Repair only this confirmed exterior seam; other ADT
    // holes remain intact for caves and below-ground WMO entrances.
    if (toLowerCopy(mapName) == "azeroth" && x == 29 && y == 51) {
        auto& lumbermillChunk = terrainPtr->chunks[15 * 16 + 14];
        if (lumbermillChunk.indexX == 14 && lumbermillChunk.indexY == 15 &&
            lumbermillChunk.holes == 0x0044) {
            lumbermillChunk.holes = 0;
            LOG_INFO("Repaired Westfall lumbermill terrain seam in tile [29,51]");
        }
    }

    // Generate mesh
    pipeline::TerrainMesh mesh = pipeline::TerrainMeshGenerator::generate(*terrainPtr);
    if (mesh.validChunkCount == 0) {
        LOG_ERROR("Failed to generate terrain mesh for tile [", x, ",", y, "]");
        return nullptr;
    }

    if (!workerRunning.load()) return nullptr;

    auto pending = std::make_shared<PendingTile>();
    pending->coord = coord;
    pending->terrain = std::move(*terrainPtr);
    pending->mesh = std::move(mesh);

    std::unordered_set<uint32_t> preparedModelIds;
    auto ensureModelPrepared = [&](const std::string& m2Path,
                                   uint32_t modelId,
                                   int& skippedFileNotFound,
                                   int& skippedInvalid,
                                   int& skippedSkinNotFound) -> bool {
        if (preparedModelIds.find(modelId) != preparedModelIds.end()) return true;

        // Skip file I/O + parsing for models already uploaded to GPU from previous tiles
        {
            std::lock_guard<std::mutex> lock(uploadedM2IdsMutex_);
            if (uploadedM2Ids_.count(modelId)) {
                preparedModelIds.insert(modelId);
                return true;
            }
        }

        // Check for WOM open format first (custom zone models)
        // Try open WOM format first via shared helper. Per-zone prefixes are
        // checked before the global fallback so a zone export overrides a
        // generic custom asset of the same name.
        {
            std::vector<std::string> extraPrefixes = {
                "output/" + mapName + "/models/",
                "custom_zones/" + mapName + "/models/",
            };
            // Asset extractor's --emit-wom writes WOM sidecars next to the
            // M2 in the asset tree (e.g. <data>/world/maps/foo/foo.wom).
            // Add the data path as a prefix so the runtime picks them up
            // without needing to copy them into custom_zones/.
            if (assetManager && !assetManager->getDataPath().empty()) {
                extraPrefixes.push_back(assetManager->getDataPath() + "/");
            }
            auto wom = pipeline::WoweeModelLoader::tryLoadByGamePath(m2Path, extraPrefixes);
            if (wom.isValid()) {
                auto m2Model = pipeline::WoweeModelLoader::toM2(wom);
                m2Model.name = m2Path;
                pending->m2Models.push_back({modelId, std::move(m2Model), {}});
                preparedModelIds.insert(modelId);
                LOG_INFO("Loaded WOM model: ", m2Path, " (v", wom.version,
                         ", ", wom.batches.size(), " batches)");
                return true;
            }
        }

        std::vector<uint8_t> m2Data = assetManager->readFile(m2Path);
        if (m2Data.empty()) {
            skippedFileNotFound++;
            return false;
        }

        pipeline::M2Model m2Model = pipeline::M2Loader::load(m2Data);
        // The asset path carries foliage semantics; embedded names frequently
        // do not. Always classify ADT doodads using the path.
        m2Model.name = m2Path;
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

        // Pre-decode M2 model textures on background thread
        for (const auto& tex : m2Model.textures) {
            if (tex.filename.empty()) continue;
            std::string texKey = tex.filename;
            std::replace(texKey.begin(), texKey.end(), '/', '\\');
            std::transform(texKey.begin(), texKey.end(), texKey.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            if (pending->preloadedM2Textures.find(texKey) != pending->preloadedM2Textures.end()) continue;
            auto blp = assetManager->loadTexture(texKey);
            if (blp.isValid()) {
                pending->preloadedM2Textures[texKey] = std::move(blp);
            }
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
        if (!workerRunning.load()) return nullptr;
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
            -placement.rotation[2] * kDegToRad,
            -placement.rotation[0] * kDegToRad,
            (placement.rotation[1] + 180.0f) * kDegToRad
        );
        p.scale = placement.scale * kInv1024;
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

    if (!workerRunning.load()) return nullptr;

    // Pre-load WMOs (CPU: read files, parse models and groups)
    if (!pending->terrain.wmoPlacements.empty()) {
        for (const auto& placement : pending->terrain.wmoPlacements) {
            if (!workerRunning.load()) return nullptr;
            if (placement.nameId >= pending->terrain.wmoNames.size()) continue;

            const std::string& wmoPath = pending->terrain.wmoNames[placement.nameId];

            // Check for WOB open format first (custom zone buildings)
            bool wobLoaded = false;
            pipeline::WMOModel wmoModel;
            {
                // Per-zone overrides win over global custom_zones/ overrides.
                std::vector<std::string> extraPrefixes = {
                    "output/" + mapName + "/buildings/",
                    "custom_zones/" + mapName + "/buildings/",
                };
                // asset_extract --emit-wob writes WOB next to the WMO in
                // the asset tree; add the data path so the runtime picks
                // them up there too.
                if (assetManager && !assetManager->getDataPath().empty()) {
                    extraPrefixes.push_back(assetManager->getDataPath() + "/");
                }
                auto wob = pipeline::WoweeBuildingLoader::tryLoadByGamePath(
                    wmoPath, extraPrefixes);
                if (wob.isValid() &&
                    pipeline::WoweeBuildingLoader::toWMOModel(wob, wmoModel)) {
                    LOG_INFO("Loaded WOB building: ", wmoPath);
                    wobLoaded = true;
                }
            }

            if (!wobLoaded) {
                std::vector<uint8_t> wmoData = assetManager->readFile(wmoPath);
                if (wmoData.empty()) continue;

                wmoModel = pipeline::WMOLoader::load(wmoData);
                if (wmoModel.nGroups > 0) {
                    std::string basePath = wmoPath;
                    std::string extension;
                    if (basePath.size() > 4) {
                        extension = basePath.substr(basePath.size() - 4);
                        std::string extLower = extension;
                        for (char& c : extLower) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
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
            }

            if (!wmoModel.groups.empty()) {
                wmoModel.sourcePath = wmoPath;
                glm::vec3 pos = core::coords::adtToWorld(placement.position[0],
                                                       placement.position[1],
                                                       placement.position[2]);

                glm::vec3 rot(
                    -placement.rotation[2] * kDegToRad,
                    -placement.rotation[0] * kDegToRad,
                    (placement.rotation[1] + 180.0f) * kDegToRad
                );

                // Pre-load WMO doodads (M2 models inside WMO)
                if (!workerRunning.load()) return nullptr;

                // Skip WMO doodads if this placement was already prepared by another tile's worker.
                // This prevents 15+ copies of Stormwind's ~6000 doodads from being parsed
                // simultaneously, which was the primary cause of OOM during world load.
                bool wmoAlreadyPrepared = false;
                if (placement.uniqueId != 0) {
                    std::lock_guard<std::mutex> lock(preparedWmoUniqueIdsMutex_);
                    wmoAlreadyPrepared = !preparedWmoUniqueIds_.insert(placement.uniqueId).second;
                }

                if (!wmoAlreadyPrepared && !wmoModel.doodadSets.empty() && !wmoModel.doodads.empty()) {
                    glm::mat4 wmoMatrix(1.0f);
                    wmoMatrix = glm::translate(wmoMatrix, pos);
                    wmoMatrix = glm::rotate(wmoMatrix, rot.z, glm::vec3(0, 0, 1));
                    wmoMatrix = glm::rotate(wmoMatrix, rot.y, glm::vec3(0, 1, 0));
                    wmoMatrix = glm::rotate(wmoMatrix, rot.x, glm::vec3(1, 0, 0));

                    // Load doodads from set 0 (global) + placement-specific set
                    std::vector<uint32_t> setsToLoad = {0};
                    if (placement.doodadSet > 0 && placement.doodadSet < wmoModel.doodadSets.size()) {
                        setsToLoad.push_back(placement.doodadSet);
                    }
                    std::unordered_set<uint32_t> loadedDoodadIndices;
                    std::unordered_set<uint32_t> wmoPreparedModelIds;  // within-WMO model dedup
                    for (uint32_t setIdx : setsToLoad) {
                        const auto& doodadSet = wmoModel.doodadSets[setIdx];
                    for (uint32_t di = 0; di < doodadSet.count; di++) {
                        uint32_t doodadIdx = doodadSet.startIndex + di;
                        if (doodadIdx >= wmoModel.doodads.size()) break;
                        if (!loadedDoodadIndices.insert(doodadIdx).second) continue;

                        const auto& doodad = wmoModel.doodads[doodadIdx];
                        auto nameIt = wmoModel.doodadNames.find(doodad.nameIndex);
                        if (nameIt == wmoModel.doodadNames.end()) continue;

                        std::string m2Path = nameIt->second;
                        if (m2Path.empty()) continue;

                        if (m2Path.size() > 4) {
                            std::string ext = m2Path.substr(m2Path.size() - 4);
                            for (char& c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                            if (ext == ".mdx" || ext == ".mdl") {
                                m2Path = m2Path.substr(0, m2Path.size() - 4) + ".m2";
                            }
                        }

                        uint32_t doodadModelId = static_cast<uint32_t>(std::hash<std::string>{}(m2Path));

                        // Skip file I/O if model already uploaded or already prepared within this WMO
                        bool modelAlreadyUploaded = false;
                        {
                            std::lock_guard<std::mutex> lock(uploadedM2IdsMutex_);
                            modelAlreadyUploaded = uploadedM2Ids_.count(doodadModelId) > 0;
                        }
                        // Membership check only — the id is claimed below, after a
                        // successful prep. Claiming up front meant a model whose first
                        // occurrence failed (missing/invalid file) poisoned every later
                        // occurrence into an empty placeholder push, which finalize then
                        // rejected with "no renderable content".
                        bool modelAlreadyPreparedInWmo = wmoPreparedModelIds.count(doodadModelId) > 0;

                        pipeline::M2Model m2Model;
                        if (!modelAlreadyUploaded && !modelAlreadyPreparedInWmo) {
                            std::vector<uint8_t> m2Data = assetManager->readFile(m2Path);
                            if (m2Data.empty()) continue;

                            m2Model = pipeline::M2Loader::load(m2Data);
                            m2Model.name = m2Path;
                            std::string skinPath = m2Path.substr(0, m2Path.size() - 3) + "00.skin";
                            std::vector<uint8_t> skinData = assetManager->readFile(skinPath);
                            if (!skinData.empty() && m2Model.version >= 264) {
                                pipeline::M2Loader::loadSkin(skinData, m2Model);
                            }
                            if (!m2Model.isValid()) continue;
                            wmoPreparedModelIds.insert(doodadModelId);

                            // Pre-decode doodad M2 textures on background thread
                            for (const auto& tex : m2Model.textures) {
                                if (tex.filename.empty()) continue;
                                std::string texKey = tex.filename;
                                std::replace(texKey.begin(), texKey.end(), '/', '\\');
                                std::transform(texKey.begin(), texKey.end(), texKey.begin(),
                                               [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                                if (pending->preloadedM2Textures.find(texKey) != pending->preloadedM2Textures.end()) continue;
                                auto blp = assetManager->loadTexture(texKey);
                                if (blp.isValid()) {
                                    pending->preloadedM2Textures[texKey] = std::move(blp);
                                }
                            }
                        }

                        // Build doodad's local transform (WoW coordinates)
                        // WMO doodads use quaternion rotation
                        glm::quat fixedRotation(doodad.rotation.w, doodad.rotation.x, doodad.rotation.y, doodad.rotation.z);

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

                        auto emitterType = rendering::classifyAmbientEmitter(m2PathLower);
                        if (emitterType != rendering::AmbientEmitterType::None) {
                            PendingTile::AmbientEmitter emitter;
                            emitter.position = worldPos;
                            // Map classifier enum to AmbientSoundManager type codes
                            switch (emitterType) {
                                case rendering::AmbientEmitterType::FireplaceSmall: emitter.type = 0; break;
                                case rendering::AmbientEmitterType::FireplaceLarge: emitter.type = 1; break;
                                case rendering::AmbientEmitterType::Torch:          emitter.type = 2; break;
                                case rendering::AmbientEmitterType::Fountain:       emitter.type = 3; break;
                                case rendering::AmbientEmitterType::Waterfall:      emitter.type = 6; break;
                                case rendering::AmbientEmitterType::Forge:          emitter.type = 1; break; // Forge → large fire
                                default: emitter.type = 0; break;
                            }
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
                }

                // Pre-decode WMO textures on background thread
                for (const auto& texPath : wmoModel.textures) {
                    if (texPath.empty()) continue;
                    std::string texKey = texPath;
                    // Truncate at NUL (WMO paths can have stray bytes)
                    size_t nul = texKey.find('\0');
                    if (nul != std::string::npos) texKey.resize(nul);
                    std::replace(texKey.begin(), texKey.end(), '/', '\\');
                    std::transform(texKey.begin(), texKey.end(), texKey.begin(),
                                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                    if (texKey.empty()) continue;
                    if (pending->preloadedWMOTextures.find(texKey) != pending->preloadedWMOTextures.end()) continue;
                    // Try .blp variant
                    std::string blpKey = texKey;
                    if (blpKey.size() >= 4) {
                        std::string ext = blpKey.substr(blpKey.size() - 4);
                        if (ext == ".tga" || ext == ".dds") {
                            blpKey = blpKey.substr(0, blpKey.size() - 4) + ".blp";
                        }
                    }
                    auto blp = assetManager->loadTexture(blpKey);
                    if (blp.isValid()) {
                        float variance = 0.0f;
                        auto normalPixels = WMORenderer::generateNormalHeightMapPixels(
                            blp.data.data(), static_cast<uint32_t>(blp.width),
                            static_cast<uint32_t>(blp.height), variance);
                        if (normalPixels.isValid()) {
                            pending->preloadedWMONormalMaps[blpKey] = std::move(normalPixels);
                            pending->preloadedWMONormalMapVariances[blpKey] = variance;
                        }
                        pending->preloadedWMOTextures[blpKey] = std::move(blp);
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
                ready.scale = placement.scale > 0
                    ? static_cast<float>(placement.scale) / 1024.0f : 1.0f;
                pending->wmoModels.push_back(std::move(ready));
            }
        }
    }

    if (!workerRunning.load()) return nullptr;

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

bool TerrainManager::advanceFinalization(FinalizingTile& ft) {
    auto& pending = ft.pending;
    int x = pending->coord.x;
    int y = pending->coord.y;
    TileCoord coord = pending->coord;

    switch (ft.phase) {

    case FinalizationPhase::TERRAIN: {
        // Check if tile was already loaded or failed
        if (loadedTiles.find(coord) != loadedTiles.end() || failedTiles.find(coord) != failedTiles.end()) {
            {
                std::lock_guard<std::mutex> lock(queueMutex);
                pendingTiles.erase(coord);
            }
            ft.phase = FinalizationPhase::DONE;
            return true;
        }

        // Upload pre-loaded textures (once)
        if (!ft.terrainPreloaded) {
            LOG_DEBUG("Finalizing tile [", x, ",", y, "] (incremental)");
            if (!pending->preloadedTextures.empty()) {
                terrainRenderer->uploadPreloadedTextures(pending->preloadedTextures);
            }
            ft.terrainPreloaded = true;
            // Yield after preload to give time budget a chance to interrupt
            return false;
        }

        // Upload terrain chunks incrementally (16 per call to spread across frames)
        if (!ft.terrainMeshDone) {
            if (pending->mesh.validChunkCount == 0) {
                LOG_ERROR("Failed to upload terrain to GPU for tile [", x, ",", y, "]");
                failedTiles[coord] = true;
                {
                    std::lock_guard<std::mutex> lock(queueMutex);
                    pendingTiles.erase(coord);
                }
                ft.phase = FinalizationPhase::DONE;
                return true;
            }
            bool allDone = terrainRenderer->loadTerrainIncremental(
                pending->mesh, pending->terrain.textures, x, y,
                ft.terrainChunkNext, 16);
            if (!allDone) {
                return false; // More chunks remain — yield to time budget
            }
            ft.terrainMeshDone = true;
        }

        // Load water after all terrain chunks are uploaded
        if (waterRenderer) {
            size_t beforeSurfaces = waterRenderer->getSurfaceCount();
            waterRenderer->loadFromTerrain(pending->terrain, true, x, y);
            size_t afterSurfaces = waterRenderer->getSurfaceCount();
            if (afterSurfaces > beforeSurfaces) {
                LOG_INFO("Water: tile [", x, ",", y, "] added ", afterSurfaces - beforeSurfaces,
                         " surfaces (total: ", afterSurfaces, ")");
            }
        } else {
            LOG_WARNING("Water: waterRenderer is null during tile [", x, ",", y, "] finalization!");
        }

        // Ensure M2 renderer has asset manager
        if (m2Renderer && assetManager) {
            if (!m2Renderer->initialize(nullptr, VK_NULL_HANDLE, assetManager))
                LOG_WARNING("M2Renderer terrain re-init failed");
        }

        ft.phase = FinalizationPhase::M2_MODELS;
        return false;
    }

    case FinalizationPhase::M2_MODELS: {
        // Upload multiple M2 models per call (batched GPU uploads).
        // When no more tiles are queued for background parsing, increase the
        // per-frame budget so idle workers don't waste time waiting for the
        // main thread to trickle-upload models.
        if (m2Renderer && ft.m2ModelIndex < pending->m2Models.size()) {
            // Set pre-decoded BLP cache so loadTexture() skips main-thread BLP decode
            m2Renderer->setPredecodedBLPCache(&pending->preloadedM2Textures);
            bool workersIdle;
            {
                std::lock_guard<std::mutex> lk(queueMutex);
                workersIdle = loadQueue.empty() && readyQueue.empty();
            }
            const size_t kModelsPerStep = workersIdle ? 6 : 4;
            size_t uploaded = 0;
            while (ft.m2ModelIndex < pending->m2Models.size() && uploaded < kModelsPerStep) {
                auto& m2Ready = pending->m2Models[ft.m2ModelIndex];
                if (m2Renderer->loadModel(m2Ready.model, m2Ready.modelId)) {
                    ft.uploadedM2ModelIds.insert(m2Ready.modelId);
                    // Track uploaded model IDs so background threads can skip re-reading
                    std::lock_guard<std::mutex> lock(uploadedM2IdsMutex_);
                    uploadedM2Ids_.insert(m2Ready.modelId);
                }
                ft.m2ModelIndex++;
                uploaded++;
            }
            m2Renderer->setPredecodedBLPCache(nullptr);
            // Stay in this phase until all models uploaded
            if (ft.m2ModelIndex < pending->m2Models.size()) {
                return false;
            }
        }
        if (!ft.uploadedM2ModelIds.empty()) {
            LOG_DEBUG("  Uploaded ", ft.uploadedM2ModelIds.size(), " M2 models for tile [", x, ",", y, "]");
        }
        ft.phase = FinalizationPhase::M2_INSTANCES;
        return false;
    }

    case FinalizationPhase::M2_INSTANCES: {
        // Create M2 instances incrementally to avoid main-thread stalls.
        // createInstance includes an O(n) bone-sibling scan that becomes expensive
        // on dense tiles with many placements and a large existing instance list.
        if (m2Renderer && ft.m2InstanceIndex < pending->m2Placements.size()) {
            // A fixed count was overshooting: 32 instances measured at 19ms
            // against this phase's 8ms budget, because createInstance carries an
            // O(n) bone-sibling scan whose cost grows with the instances already
            // present. Bound by time instead, which holds regardless of how
            // heavy each one turns out to be.
            constexpr size_t kInstancesPerStep = 32;
            constexpr float kInstanceBudgetMs = 4.0f;
            const auto instanceStart = std::chrono::steady_clock::now();
            size_t created = 0;
            while (ft.m2InstanceIndex < pending->m2Placements.size() && created < kInstancesPerStep) {
                if (created > 0 && std::chrono::duration<float, std::milli>(
                        std::chrono::steady_clock::now() - instanceStart).count() >= kInstanceBudgetMs) {
                    break;
                }
                const auto& p = pending->m2Placements[ft.m2InstanceIndex++];
                if (p.uniqueId != 0 && placedDoodadIds.count(p.uniqueId)) {
                    continue;
                }
                if (!m2Renderer->hasModel(p.modelId)) {
                    continue;
                }
                uint32_t instId = m2Renderer->createInstance(p.modelId, p.position, p.rotation, p.scale);
                if (instId) {
                    ft.m2InstanceIds.push_back(instId);
                    if (p.uniqueId != 0) {
                        placedDoodadIds.insert(p.uniqueId);
                        ft.tileUniqueIds.push_back(p.uniqueId);
                    }
                    created++;
                }
            }
            if (ft.m2InstanceIndex < pending->m2Placements.size()) {
                return false; // More instances to create — yield
            }
            LOG_DEBUG("  Loaded doodads for tile [", x, ",", y, "]: ",
                     ft.m2InstanceIds.size(), " instances (", ft.uploadedM2ModelIds.size(), " new models)");
        }
        ft.phase = FinalizationPhase::WMO_MODELS;
        return false;
    }

    case FinalizationPhase::WMO_MODELS: {
        // Upload multiple WMO models per call (batched GPU uploads)
        if (wmoRenderer && assetManager) {
            if (!wmoRenderer->initialize(nullptr, VK_NULL_HANDLE, assetManager))
                LOG_WARNING("WMORenderer terrain re-init failed");
            // Diffuse decode and normal/height generation were completed by the
            // terrain worker. The main thread only uploads those prepared pixels.
            wmoRenderer->setPredecodedBLPCache(&pending->preloadedWMOTextures);
            wmoRenderer->setPredecodedNormalMapCache(
                &pending->preloadedWMONormalMaps,
                &pending->preloadedWMONormalMapVariances);
            wmoRenderer->setDeferNormalMaps(true);

            // One model per step, and a large model spread across several
            // steps: a 286-group WMO took 131ms to upload in one go, against
            // this phase's 8ms budget. Uploading two per step when the workers
            // were idle only doubled that worst case.
            constexpr float kWmoGroupBudgetMs = 6.0f;
            while (ft.wmoModelIndex < pending->wmoModels.size()) {
                auto& wmoReady = pending->wmoModels[ft.wmoModelIndex];
                if (wmoReady.uniqueId != 0 && placedWmoIds.count(wmoReady.uniqueId)) {
                    ft.wmoModelIndex++;
                    continue;
                }
                const auto result = wmoRenderer->loadModelIncremental(
                    wmoReady.model, wmoReady.modelId, kWmoGroupBudgetMs);
                if (result == WMORenderer::ModelLoadResult::InProgress) {
                    break;  // same model resumes on the next call
                }
                ft.wmoModelIndex++;  // Complete or Failed — either way, move on
                break;               // one model per step
            }
            wmoRenderer->setDeferNormalMaps(false);
            wmoRenderer->setPredecodedBLPCache(nullptr);
            wmoRenderer->setPredecodedNormalMapCache(nullptr, nullptr);
            if (ft.wmoModelIndex < pending->wmoModels.size()) return false;
        }
        ft.phase = FinalizationPhase::WMO_INSTANCES;
        return false;
    }

    case FinalizationPhase::WMO_INSTANCES: {
        // Create WMO instances incrementally to avoid stalls on tiles with many WMOs.
        // Liquid group loading is also budgeted (max 4 per call) to prevent stalls
        // on WMOs with many liquid groups (e.g. Stormwind canals).
        if (wmoRenderer && ft.wmoInstanceIndex < pending->wmoModels.size()) {
            constexpr size_t kWmoInstancesPerStep = 4;
            constexpr size_t kLiquidGroupsPerStep = 4;
            size_t created = 0;
            size_t liquidGroupsLoaded = 0;
            while (ft.wmoInstanceIndex < pending->wmoModels.size() && created < kWmoInstancesPerStep) {
                auto& wmoReady = pending->wmoModels[ft.wmoInstanceIndex];
                // Skip duplicates and unloaded models
                if (wmoReady.uniqueId != 0 && placedWmoIds.count(wmoReady.uniqueId)) {
                    ft.wmoInstanceIndex++;
                    ft.wmoLiquidGroupIndex = 0;
                    continue;
                }
                if (!wmoRenderer->isModelLoaded(wmoReady.modelId)) {
                    ft.wmoInstanceIndex++;
                    ft.wmoLiquidGroupIndex = 0;
                    continue;
                }
                // Create the instance on first visit (liquidGroupIndex == 0)
                if (ft.wmoLiquidGroupIndex == 0) {
                    uint32_t wmoInstId = wmoRenderer->createInstance(
                        wmoReady.modelId, wmoReady.position, wmoReady.rotation, wmoReady.scale);
                    if (!wmoInstId) {
                        ft.wmoInstanceIndex++;
                        continue;
                    }
                    ft.wmoInstanceIds.push_back(wmoInstId);
                    if (wmoReady.uniqueId != 0) {
                        placedWmoIds.insert(wmoReady.uniqueId);
                        ft.tileWmoUniqueIds.push_back(wmoReady.uniqueId);
                    }
                }
                // Load WMO liquids incrementally (canals, pools, etc.)
                if (waterRenderer) {
                    uint32_t wmoInstId = ft.wmoInstanceIds.back();
                    glm::mat4 modelMatrix = glm::mat4(1.0f);
                    modelMatrix = glm::translate(modelMatrix, wmoReady.position);
                    modelMatrix = glm::rotate(modelMatrix, wmoReady.rotation.z, glm::vec3(0.0f, 0.0f, 1.0f));
                    modelMatrix = glm::rotate(modelMatrix, wmoReady.rotation.y, glm::vec3(0.0f, 1.0f, 0.0f));
                    modelMatrix = glm::rotate(modelMatrix, wmoReady.rotation.x, glm::vec3(1.0f, 0.0f, 0.0f));
                    const auto& groups = wmoReady.model.groups;
                    while (ft.wmoLiquidGroupIndex < groups.size() && liquidGroupsLoaded < kLiquidGroupsPerStep) {
                        const auto& group = groups[ft.wmoLiquidGroupIndex];
                        ft.wmoLiquidGroupIndex++;
                        if (!group.liquid.hasLiquid()) continue;
                        if (group.flags & 0x2000) {
                            uint16_t lt = group.liquid.materialId;
                            uint8_t basicType = (lt == 0) ? 0 : ((lt - 1) % 4);
                            if (basicType < 2) continue;
                        }
                        waterRenderer->loadFromWMO(group.liquid, modelMatrix, wmoInstId);
                        liquidGroupsLoaded++;
                    }
                    // More liquid groups remain on this WMO — yield
                    if (ft.wmoLiquidGroupIndex < groups.size()) {
                        return false;
                    }
                }
                ft.wmoInstanceIndex++;
                ft.wmoLiquidGroupIndex = 0;
                created++;
            }
            if (ft.wmoInstanceIndex < pending->wmoModels.size()) {
                return false; // More WMO instances to create — yield
            }
            LOG_DEBUG("  Loaded WMOs for tile [", x, ",", y, "]: ", ft.wmoInstanceIds.size(), " instances");
        }
        ft.phase = FinalizationPhase::WMO_DOODADS;
        return false;
    }

    case FinalizationPhase::WMO_DOODADS: {
        // Upload multiple WMO doodad M2s per call (batched GPU uploads)
        if (m2Renderer && ft.wmoDoodadIndex < pending->wmoDoodads.size()) {
            // Set pre-decoded BLP cache for doodad M2 textures
            m2Renderer->setPredecodedBLPCache(&pending->preloadedM2Textures);
            constexpr size_t kDoodadsPerStep = 4;
            size_t uploaded = 0;
            while (ft.wmoDoodadIndex < pending->wmoDoodads.size() && uploaded < kDoodadsPerStep) {
                auto& doodad = pending->wmoDoodads[ft.wmoDoodadIndex];
                if (!m2Renderer->loadModel(doodad.model, doodad.modelId)) {
                    ft.wmoDoodadIndex++;
                    uploaded++;
                    continue;
                }
                {
                    std::lock_guard<std::mutex> lock(uploadedM2IdsMutex_);
                    uploadedM2Ids_.insert(doodad.modelId);
                }
                uint32_t wmoDoodadInstId = m2Renderer->createInstanceWithMatrix(
                    doodad.modelId, doodad.modelMatrix, doodad.worldPosition);
                if (wmoDoodadInstId) {
                    // WMO doodads should not add duplicate/over-aggressive wall
                    // blocking, but structural doodads such as Exodarplatform01
                    // carry the only authored floor for their walkable ramps.
                    m2Renderer->setSkipWallCollision(wmoDoodadInstId, true);
                    ft.m2InstanceIds.push_back(wmoDoodadInstId);
                }
                ft.wmoDoodadIndex++;
                uploaded++;
            }
            m2Renderer->setPredecodedBLPCache(nullptr);
            if (ft.wmoDoodadIndex < pending->wmoDoodads.size()) return false;
        }
        ft.phase = FinalizationPhase::WATER;
        return false;
    }

    case FinalizationPhase::WATER: {
        // Terrain water was already loaded in TERRAIN phase.
        // Generate water ambient emitters here.
        if (ambientSoundManager) {
            for (size_t chunkIdx = 0; chunkIdx < pending->terrain.waterData.size(); chunkIdx++) {
                const auto& chunkWater = pending->terrain.waterData[chunkIdx];
                if (!chunkWater.hasWater()) continue;

                int chunkX = chunkIdx % 16;
                int chunkY = chunkIdx / 16;
                float tileOriginX = (32.0f - x) * 533.33333f;
                float tileOriginY = (32.0f - y) * 533.33333f;
                float chunkCenterX = tileOriginX + (chunkX + 0.5f) * 33.333333f;
                float chunkCenterY = tileOriginY + (chunkY + 0.5f) * 33.333333f;

                if (!chunkWater.layers.empty()) {
                    const auto& layer = chunkWater.layers[0];
                    float waterHeight = layer.minHeight;
                    if (layer.liquidType == 0 && chunkIdx % 32 == 0) {
                        PendingTile::AmbientEmitter emitter;
                        emitter.position = glm::vec3(chunkCenterX, chunkCenterY, waterHeight);
                        emitter.type = 4;
                        pending->ambientEmitters.push_back(emitter);
                    } else if (layer.liquidType == 1 && chunkIdx % 64 == 0) {
                        PendingTile::AmbientEmitter emitter;
                        emitter.position = glm::vec3(chunkCenterX, chunkCenterY, waterHeight);
                        emitter.type = 4;
                        pending->ambientEmitters.push_back(emitter);
                    }
                }
            }
        }

        ft.phase = FinalizationPhase::AMBIENT;
        return false;
    }

    case FinalizationPhase::AMBIENT: {
        // Register ambient sound emitters
        if (ambientSoundManager && !pending->ambientEmitters.empty()) {
            for (const auto& emitter : pending->ambientEmitters) {
                auto type = static_cast<audio::AmbientSoundManager::AmbientType>(emitter.type);
                ambientSoundManager->addEmitter(emitter.position, type);
            }
        }

        // Commit tile to loadedTiles
        auto tile = std::make_unique<TerrainTile>();
        tile->coord = coord;
        tile->terrain = std::move(pending->terrain);
        tile->mesh = std::move(pending->mesh);
        tile->loaded = true;
        tile->m2InstanceIds = std::move(ft.m2InstanceIds);
        tile->wmoInstanceIds = std::move(ft.wmoInstanceIds);
        tile->wmoUniqueIds = std::move(ft.tileWmoUniqueIds);
        tile->doodadUniqueIds = std::move(ft.tileUniqueIds);
        getTileBounds(coord, tile->minX, tile->minY, tile->maxX, tile->maxY);
        loadedTiles[coord] = std::move(tile);
        // NOTE: Don't cache pending here — std::move above empties terrain/mesh,
        // so the cached tile would have 0 valid chunks on reuse.  Tiles are
        // re-parsed from ADT files (file-cache hit) when they re-enter range.

        // Now safe to remove from pendingTiles (tile is in loadedTiles)
        {
            std::lock_guard<std::mutex> lock(queueMutex);
            pendingTiles.erase(coord);
        }

        LOG_DEBUG("  Finalized tile [", x, ",", y, "]");

        ft.phase = FinalizationPhase::DONE;
        return true;
    }

    case FinalizationPhase::DONE:
        return true;
    }
    return true;
}

void TerrainManager::workerLoop() {
    // Leave placement to the OS scheduler. Artificially reserving CPU 0 made
    // this worker policy depend on the old main-thread pin and reduced the
    // scheduler's ability to balance streaming with render workers.
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

            // --- Memory-aware throttling ---
            // Back-pressure: if the ready queue is deep (finalization can't
            // keep up), or the system is running low on RAM, sleep instead
            // of pulling more tiles.  Each prepared tile can hold hundreds
            // of MB of decoded textures; limiting concurrency here prevents
            // WoWee from consuming all system memory during world load.
            const auto& memMon = core::MemoryMonitor::getInstance();
            if (memMon.isSevereMemoryPressure()) {
                // Severe pressure — don't pull ANY work until main thread
                // finalizes tiles and frees decoded texture data.
                lock.unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                continue;
            }
            if (readyQueue.size() >= maxReadyQueueSize_ || memMon.isMemoryPressure()) {
                // Moderate pressure or ready queue is backing up — sleep briefly
                // to let the main thread catch up with finalization.
                lock.unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
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
    ZoneScopedN("TerrainManager::processReadyTiles");
    // Move newly ready tiles into the finalizing deque.
    // Keep them in pendingTiles so streamTiles() won't re-enqueue them.
    {
        std::lock_guard<std::mutex> lock(queueMutex);
        while (!readyQueue.empty()) {
            auto pending = readyQueue.front();
            readyQueue.pop();
            if (pending) {
                FinalizingTile ft;
                ft.pending = std::move(pending);
                finalizingTiles_.push_back(std::move(ft));
            }
        }
    }

    VkContext* vkCtx = terrainRenderer ? terrainRenderer->getVkContext() : nullptr;

    // Reclaim completed async uploads from previous frames (non-blocking)
    if (vkCtx) vkCtx->pollUploadBatches();

    // Nothing to finalize — done.
    if (finalizingTiles_.empty()) return;

    // Async upload batch: record GPU copies into a command buffer, submit with
    // a fence, but DON'T wait.  The fence is polled on subsequent frames.
    // This eliminates the main-thread stall from vkWaitForFences entirely.
    //
    // Time-budgeted: yield after 8ms to prevent main-loop stalls. Each
    // advanceFinalization step is designed to be small, but texture uploads
    // and M2 model loads can occasionally spike. The budget ensures we
    // spread heavy tiles across multiple frames instead of blocking.
    const auto budgetStart = std::chrono::steady_clock::now();
    const float budgetMs = taxiStreamingMode_ ? 16.0f : 8.0f;

    if (vkCtx) vkCtx->beginUploadBatch();

    // The budget is checked between steps, so it bounds how many run — not how
    // long one takes. Most phases handle a single model per call, but TERRAIN
    // uploads a whole tile's chunks and textures, and the INSTANCES phases build
    // every instance at once, so one step can overrun the whole budget on its
    // own. Name the phase when it does, rather than leaving one opaque number.
    auto phaseName = [](FinalizationPhase p) {
        switch (p) {
            case FinalizationPhase::TERRAIN:       return "TERRAIN";
            case FinalizationPhase::M2_MODELS:     return "M2_MODELS";
            case FinalizationPhase::M2_INSTANCES:  return "M2_INSTANCES";
            case FinalizationPhase::WMO_MODELS:    return "WMO_MODELS";
            case FinalizationPhase::WMO_INSTANCES: return "WMO_INSTANCES";
            case FinalizationPhase::WMO_DOODADS:   return "WMO_DOODADS";
            case FinalizationPhase::WATER:         return "WATER";
            case FinalizationPhase::AMBIENT:       return "AMBIENT";
            case FinalizationPhase::DONE:          return "DONE";
        }
        return "?";
    };

    while (!finalizingTiles_.empty()) {
        auto& ft = finalizingTiles_.front();
        const auto phaseBefore = ft.phase;
        const auto stepStart = std::chrono::steady_clock::now();
        bool done = advanceFinalization(ft);
        const float stepMs = std::chrono::duration<float, std::milli>(
            std::chrono::steady_clock::now() - stepStart).count();
        if (stepMs > budgetMs) {
            LOG_WARNING("Terrain finalize step overran: ", phaseName(phaseBefore),
                        " took ", stepMs, "ms (budget ", budgetMs, "ms) tile=[",
                        ft.pending ? ft.pending->coord.x : -1, ",",
                        ft.pending ? ft.pending->coord.y : -1, "]");
        }
        if (done) {
            finalizingTiles_.pop_front();
        }
        float elapsed = std::chrono::duration<float, std::milli>(
            std::chrono::steady_clock::now() - budgetStart).count();
        if (elapsed >= budgetMs) break;
    }

    if (vkCtx) vkCtx->endUploadBatch();  // Async — submits but doesn't wait
}

void TerrainManager::processPendingUnloads() {
    ZoneScopedN("TerrainManager::processPendingUnloads");
    if (pendingUnloadQueue_.empty()) return;

    // Time-budgeted rather than count-capped (see pendingUnloadQueue_'s comment) so
    // throughput scales with whatever time is actually available this frame instead
    // of a fixed tile count that can't keep pace when frame rate drops or the queue
    // is unusually large (e.g. after a long, fast taxi flight). Taxi mode gets a
    // larger budget, matching processReadyTiles()'s existing taxi-aware budget.
    const auto budgetStart = std::chrono::steady_clock::now();
    const float budgetMs = taxiStreamingMode_ ? 16.0f : 8.0f;

    size_t unloaded = 0;
    while (!pendingUnloadQueue_.empty()) {
        TileCoord coord = pendingUnloadQueue_.front();
        pendingUnloadQueue_.pop_front();

        // Skip stale entries: the player may have reversed course since this
        // tile was queued, bringing it back within range. Unloading it now
        // would pop visible terrain out from under them.
        int dx = coord.x - currentTile.x;
        int dy = coord.y - currentTile.y;
        if (dx*dx + dy*dy <= unloadRadius*unloadRadius) continue;

        unloadTile(coord.x, coord.y);
        unloaded++;

        const float elapsed = std::chrono::duration<float, std::milli>(
            std::chrono::steady_clock::now() - budgetStart).count();
        if (elapsed >= budgetMs) break;
    }

    if (unloaded > 0) {
        LOG_DEBUG("Unloaded ", unloaded, " distant tiles (", pendingUnloadQueue_.size(),
                 " queued), ", loadedTiles.size(), " remain (models kept in VRAM)");
    }
}

void TerrainManager::processAllReadyTiles() {
    // Move all ready tiles into finalizing deque
    // Keep in pendingTiles until committed (same as processReadyTiles)
    {
        std::lock_guard<std::mutex> lock(queueMutex);
        while (!readyQueue.empty()) {
            auto pending = readyQueue.front();
            readyQueue.pop();
            if (pending) {
                FinalizingTile ft;
                ft.pending = std::move(pending);
                finalizingTiles_.push_back(std::move(ft));
            }
        }
    }

    // Batch all GPU uploads across all tiles into a single submission
    VkContext* vkCtx = terrainRenderer ? terrainRenderer->getVkContext() : nullptr;
    if (vkCtx) vkCtx->beginUploadBatch();

    // Finalize all tiles completely (no time budget — used for loading screens)
    while (!finalizingTiles_.empty()) {
        auto& ft = finalizingTiles_.front();
        while (!advanceFinalization(ft)) {}
        finalizingTiles_.pop_front();
    }

    if (vkCtx) vkCtx->endUploadBatchSync();  // Sync — load screen needs data ready
}

void TerrainManager::processOneReadyTile() {
    // Move ready tiles into finalizing deque
    {
        std::lock_guard<std::mutex> lock(queueMutex);
        while (!readyQueue.empty()) {
            auto pending = readyQueue.front();
            readyQueue.pop();
            if (pending) {
                FinalizingTile ft;
                ft.pending = std::move(pending);
                finalizingTiles_.push_back(std::move(ft));
            }
        }
    }
    // Finalize ONE tile completely, then return so caller can update the screen
    if (!finalizingTiles_.empty()) {
        VkContext* vkCtx = terrainRenderer ? terrainRenderer->getVkContext() : nullptr;
        if (vkCtx) vkCtx->beginUploadBatch();

        auto& ft = finalizingTiles_.front();
        while (!advanceFinalization(ft)) {}
        finalizingTiles_.pop_front();

        if (vkCtx) vkCtx->endUploadBatchSync();  // Sync — load screen needs data ready
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

    // Remove from finalizingTiles_ if it's being incrementally finalized.
    // Water may have already been loaded in TERRAIN phase, so clean it up.
    for (auto fit = finalizingTiles_.begin(); fit != finalizingTiles_.end(); ++fit) {
        if (fit->pending && fit->pending->coord == coord) {
            // If terrain chunks were already uploaded, free their descriptor sets
            if (fit->terrainMeshDone && terrainRenderer) {
                terrainRenderer->removeTile(x, y);
            }
            // If past TERRAIN phase, water was already loaded — remove it
            if (fit->phase != FinalizationPhase::TERRAIN && waterRenderer) {
                waterRenderer->removeTile(x, y);
            }
            // Clean up any M2/WMO instances that were already created
            if (m2Renderer && !fit->m2InstanceIds.empty()) {
                m2Renderer->removeInstances(fit->m2InstanceIds);
            }
            if (wmoRenderer && !fit->wmoInstanceIds.empty()) {
                for (uint32_t id : fit->wmoInstanceIds) {
                    if (waterRenderer) waterRenderer->removeWMO(id);
                }
                wmoRenderer->removeInstances(fit->wmoInstanceIds);
            }
            for (uint32_t uid : fit->tileUniqueIds) placedDoodadIds.erase(uid);
            for (uint32_t uid : fit->tileWmoUniqueIds) {
                placedWmoIds.erase(uid);
                std::lock_guard<std::mutex> lock(preparedWmoUniqueIdsMutex_);
                preparedWmoUniqueIds_.erase(uid);
            }
            finalizingTiles_.erase(fit);
            return;
        }
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
        std::lock_guard<std::mutex> lock(preparedWmoUniqueIdsMutex_);
        preparedWmoUniqueIds_.erase(uid);
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

void TerrainManager::stopWorkers() {
    if (!workerRunning.load()) {
        LOG_DEBUG("stopWorkers: already stopped");
        return;
    }
    LOG_DEBUG("stopWorkers: signaling ", workerThreads.size(), " workers to stop...");
    workerRunning.store(false);
    queueCV.notify_all();

    // Workers check workerRunning at each I/O point in prepareTile() and bail
    // out quickly.  Use plain join() which is safe with std::thread — no
    // pthread_timedjoin_np (which silently joins the pthread but leaves the
    // std::thread object thinking it's still joinable → std::terminate on dtor).
    for (size_t i = 0; i < workerThreads.size(); i++) {
        if (workerThreads[i].joinable()) {
            LOG_DEBUG("stopWorkers: joining worker ", i, "...");
            workerThreads[i].join();
        }
    }
    workerThreads.clear();
    LOG_DEBUG("stopWorkers: done");
}

void TerrainManager::unloadAll() {
    // Signal worker threads to stop and wait briefly for them to finish.
    // Workers may be mid-prepareTile (reading MPQ / parsing ADT) which can
    // take seconds, so use a short deadline and detach any stragglers.
    if (workerRunning.load()) {
        workerRunning.store(false);
        queueCV.notify_all();

        for (auto& t : workerThreads) {
            if (t.joinable()) t.join();
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
    finalizingTiles_.clear();
    placedDoodadIds.clear();
    placedWmoIds.clear();
    {
        std::lock_guard<std::mutex> lock(uploadedM2IdsMutex_);
        uploadedM2Ids_.clear();
    }
    {
        std::lock_guard<std::mutex> lock(preparedWmoUniqueIdsMutex_);
        preparedWmoUniqueIds_.clear();
    }

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
    workerCount = computeTerrainWorkerCount();
    workerThreads.reserve(workerCount);
    for (int i = 0; i < workerCount; i++) {
        workerThreads.emplace_back(&TerrainManager::workerLoop, this);
    }
}

void TerrainManager::softReset() {
    // Clear queues (workers may still be running — they'll find empty queues)
    {
        std::lock_guard<std::mutex> lock(queueMutex);
        loadQueue.clear();
        while (!readyQueue.empty()) readyQueue.pop();
    }
    pendingTiles.clear();
    finalizingTiles_.clear();
    placedDoodadIds.clear();
    placedWmoIds.clear();
    {
        std::lock_guard<std::mutex> lock(uploadedM2IdsMutex_);
        uploadedM2Ids_.clear();
    }
    {
        std::lock_guard<std::mutex> lock(preparedWmoUniqueIdsMutex_);
        preparedWmoUniqueIds_.clear();
    }

    // Clear tile cache — keys are (x,y) without map name, so stale entries from
    // a different map with overlapping coordinates would produce wrong geometry.
    {
        std::lock_guard<std::mutex> lock(tileCacheMutex_);
        tileCache_.clear();
        tileCacheLru_.clear();
        tileCacheBytes_ = 0;
    }

    LOG_INFO("Soft-resetting terrain (clearing tiles + water + cache, workers stay alive)");
    loadedTiles.clear();
    failedTiles.clear();

    currentTile = {-1, -1};
    lastStreamTile = {-1, -1};

    if (terrainRenderer) {
        terrainRenderer->clear();
    }
    if (waterRenderer) {
        waterRenderer->clear();
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
        m2Model.name = m2Path;
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
                    float fracX = (nextRand() & 0xFFFFu) / kRand16Max * 8.0f;
                    float fracY = (nextRand() & 0xFFFFu) / kRand16Max * 8.0f;

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
                    p.rotation = glm::vec3(0.0f, 0.0f, (nextRand() & 0xFFFFu) / kRand16Max * (2.0f * pi));
                    p.scale = 0.80f + ((nextRand() & 0xFFFFu) / kRand16Max) * 0.35f;
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

                        float fracX = (nextRand() & 0xFFFFu) / kRand16Max * 8.0f;
                        float fracY = (nextRand() & 0xFFFFu) / kRand16Max * 8.0f;
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
                        p.rotation = glm::vec3(0.0f, 0.0f, (nextRand() & 0xFFFFu) / kRand16Max * (2.0f * pi));
                        p.scale = 0.75f + ((nextRand() & 0xFFFFu) / kRand16Max) * 0.40f;
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

bool TerrainManager::isHoleAt(float glX, float glY) const {
    const float unitSize = CHUNK_SIZE / 8.0f;

    auto tileHole = [&](const TerrainTile* tile) -> std::optional<bool> {
        if (!tile || !tile->loaded) return std::nullopt;

        auto chunkHole = [&](int cx, int cy) -> std::optional<bool> {
            if (cx < 0 || cx >= 16 || cy < 0 || cy >= 16) return std::nullopt;
            const auto& chunk = tile->terrain.getChunk(cx, cy);
            if (!chunk.hasHeightMap()) return std::nullopt;

            const float chunkMaxX = chunk.position[0];
            const float chunkMinX = chunk.position[0] - 8.0f * unitSize;
            const float chunkMaxY = chunk.position[1];
            const float chunkMinY = chunk.position[1] - 8.0f * unitSize;
            if (glX < chunkMinX || glX > chunkMaxX ||
                glY < chunkMinY || glY > chunkMaxY) {
                return std::nullopt;
            }
            if (chunk.holes == 0) return false;

            // The same two fractions getHeightAt derives its height from, and
            // the same order the mesh builder passes to isHole — it walks
            // `for y { for x { if (chunk.isHole(y, x)) continue; ... } }` over
            // vertices at `y * 17 + x`, which is the grid getHeightAt samples
            // as `heights[gy * 17 + gx]`. Answering the quad from the same two
            // numbers is what keeps the surface the player stands on and the
            // surface drawn from disagreeing about which quads exist.
            const float fracY = (chunk.position[0] - glX) / unitSize;
            const float fracX = (chunk.position[1] - glY) / unitSize;
            const int qy = glm::clamp(static_cast<int>(std::floor(fracY)), 0, 7);
            const int qx = glm::clamp(static_cast<int>(std::floor(fracX)), 0, 7);
            return chunk.isHole(qy, qx);
        };

        int guessCy = glm::clamp(static_cast<int>(std::floor((tile->maxX - glX) / CHUNK_SIZE)), 0, 15);
        int guessCx = glm::clamp(static_cast<int>(std::floor((tile->maxY - glY) / CHUNK_SIZE)), 0, 15);
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                if (auto r = chunkHole(guessCx + dx, guessCy + dy)) return r;
            }
        }
        return std::nullopt;
    };

    const TileCoord tc = worldToTile(glX, glY);
    auto it = loadedTiles.find(tc);
    if (it != loadedTiles.end()) {
        if (auto r = tileHole(it->second.get())) return *r;
    }
    for (const auto& [coord, tile] : loadedTiles) {
        if (coord == tc) continue;
        if (auto r = tileHole(tile.get())) return *r;
    }
    return false;
}

bool TerrainManager::chunkHasHoles(float glX, float glY) const {
    const float unitSize = CHUNK_SIZE / 8.0f;
    (void)unitSize;

    auto tileHasHoles = [&](const TerrainTile* tile) -> std::optional<bool> {
        if (!tile || !tile->loaded) return std::nullopt;
        int cy = static_cast<int>(std::floor((tile->maxX - glX) / CHUNK_SIZE));
        int cx = static_cast<int>(std::floor((tile->maxY - glY) / CHUNK_SIZE));
        if (cx < 0 || cx >= 16 || cy < 0 || cy >= 16) return std::nullopt;
        return tile->terrain.getChunk(cx, cy).holes != 0;
    };

    const TileCoord tc = worldToTile(glX, glY);
    auto it = loadedTiles.find(tc);
    if (it != loadedTiles.end()) {
        if (auto r = tileHasHoles(it->second.get())) return *r;
    }
    for (const auto& [coord, tile] : loadedTiles) {
        if (coord == tc) continue;
        if (auto r = tileHasHoles(tile.get())) return *r;
    }
    return false;
}

std::optional<uint32_t> TerrainManager::getAreaIdAt(float glX, float glY) const {
    const TileCoord tc = worldToTile(glX, glY);
    auto it = loadedTiles.find(tc);
    if (it == loadedTiles.end() || !it->second || !it->second->loaded) {
        return std::nullopt;
    }

    const TerrainTile& tile = *it->second;
    // tileX advances along renderY while tileY advances along renderX.
    const float tileMaxRenderX = (32.0f - static_cast<float>(tc.y)) * TILE_SIZE;
    const float tileMaxRenderY = (32.0f - static_cast<float>(tc.x)) * TILE_SIZE;
    const int chunkY = glm::clamp(
        static_cast<int>(std::floor((tileMaxRenderX - glX) / CHUNK_SIZE)), 0, 15);
    const int chunkX = glm::clamp(
        static_cast<int>(std::floor((tileMaxRenderY - glY) / CHUNK_SIZE)), 0, 15);
    const uint32_t areaId = tile.terrain.getChunk(chunkX, chunkY).areaId;
    return areaId != 0 ? std::optional<uint32_t>(areaId) : std::nullopt;
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

    // Enqueue tiles in radius around current tile for async loading.
    // Collect all newly-needed tiles, then sort by distance so the closest
    // (most visible) tiles get loaded first.  This is critical during taxi
    // flight where new tiles enter the radius faster than they can load.
    {
        std::lock_guard<std::mutex> lock(queueMutex);

        struct PendingEntry { TileCoord coord; int distSq; };
        std::vector<PendingEntry> newTiles;

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

                newTiles.push_back({coord, dx*dx + dy*dy});
                pendingTiles[coord] = true;
            }
        }

        // Sort nearest tiles first so workers service the most visible tiles
        std::sort(newTiles.begin(), newTiles.end(),
                  [](const PendingEntry& a, const PendingEntry& b) { return a.distSq < b.distSq; });

        // Insert at front so new close tiles preempt any distant tiles already queued
        for (auto it = newTiles.rbegin(); it != newTiles.rend(); ++it) {
            loadQueue.push_front(it->coord);
        }
    }

    // Notify workers that there's work
    queueCV.notify_all();

    // Unload tiles beyond unload radius (well past the camera far clip).
    // Queue them rather than unloading synchronously here — processPendingUnloads()
    // drains a time-budgeted batch per frame instead (see pendingUnloadQueue_'s comment).
    std::unordered_set<TileCoord, TileCoord::Hash> alreadyQueued(
        pendingUnloadQueue_.begin(), pendingUnloadQueue_.end());
    size_t queuedNow = 0;

    for (const auto& pair : loadedTiles) {
        const TileCoord& coord = pair.first;

        int dx = coord.x - currentTile.x;
        int dy = coord.y - currentTile.y;

        // Circular pattern: unload beyond radius (Euclidean distance)
        if (dx*dx + dy*dy > unloadRadius*unloadRadius && !alreadyQueued.count(coord)) {
            pendingUnloadQueue_.push_back(coord);
            queuedNow++;
        }
    }

    if (queuedNow > 0) {
        LOG_DEBUG("Queued ", queuedNow, " distant tiles for unload (",
                 pendingUnloadQueue_.size(), " total pending)");
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

uint8_t TerrainManager::getCollisionFlags(float glX, float glY) const {
    for (const auto& [key, cd] : collisionTiles_) {
        if (!cd.loaded) continue;
        if (glX < cd.boundsMin.x || glX > cd.boundsMax.x ||
            glY < cd.boundsMin.y || glY > cd.boundsMax.y) continue;

        for (const auto& tri : cd.triangles) {
            // Barycentric point-in-triangle test (XY plane)
            glm::vec2 p(glX, glY);
            glm::vec2 a(tri.v0.x, tri.v0.y), b(tri.v1.x, tri.v1.y), c(tri.v2.x, tri.v2.y);
            glm::vec2 v0 = c - a, v1 = b - a, v2 = p - a;
            float d00 = glm::dot(v0, v0), d01 = glm::dot(v0, v1), d02 = glm::dot(v0, v2);
            float d11 = glm::dot(v1, v1), d12 = glm::dot(v1, v2);
            float inv = d00 * d11 - d01 * d01;
            if (std::abs(inv) < 1e-10f) continue;
            float u = (d11 * d02 - d01 * d12) / inv;
            float v = (d00 * d12 - d01 * d02) / inv;
            if (u >= 0 && v >= 0 && u + v <= 1)
                return tri.flags;
        }
    }
    return 0x01; // default walkable if no collision data
}

bool TerrainManager::isPositionWalkable(float glX, float glY) const {
    return (getCollisionFlags(glX, glY) & 0x01) != 0;
}

} // namespace rendering
} // namespace wowee
