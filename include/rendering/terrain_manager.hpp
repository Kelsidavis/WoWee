#pragma once

#include "pipeline/adt_loader.hpp"
#include "pipeline/terrain_mesh.hpp"
#include "pipeline/m2_loader.hpp"
#include "pipeline/wmo_loader.hpp"
#include "pipeline/blp_loader.hpp"
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <optional>
#include <thread>
#include <mutex>
#include <atomic>
#include <queue>
#include <list>
#include <vector>
#include <condition_variable>
#include <deque>
#include <glm/glm.hpp>
#include <array>

namespace wowee {

namespace pipeline { class AssetManager; }
namespace audio { class AmbientSoundManager; }
namespace rendering { class TerrainRenderer; class Camera; class WaterRenderer; class M2Renderer; class WMORenderer; }

namespace rendering {

/**
 * Terrain tile coordinates
 */
struct TileCoord {
    int x;
    int y;

    bool operator==(const TileCoord& other) const {
        return x == other.x && y == other.y;
    }

    struct Hash {
        size_t operator()(const TileCoord& coord) const {
            return std::hash<int>()(coord.x) ^ (std::hash<int>()(coord.y) << 1);
        }
    };
};

/**
 * Loaded terrain tile data
 */
struct TerrainTile {
    TileCoord coord;
    pipeline::ADTTerrain terrain;
    pipeline::TerrainMesh mesh;
    bool loaded = false;

    // Tile bounds in world coordinates
    float minX, minY, maxX, maxY;

    // Instance IDs for cleanup on unload
    std::vector<uint32_t> wmoInstanceIds;
    std::vector<uint32_t> wmoUniqueIds;  // For WMO dedup cleanup on unload
    std::vector<uint32_t> m2InstanceIds;
    std::vector<uint32_t> doodadUniqueIds;  // For dedup cleanup on unload
};

/**
 * Pre-processed tile data ready for GPU upload (produced by background thread)
 */
struct PendingTile {
    TileCoord coord;
    pipeline::ADTTerrain terrain;
    pipeline::TerrainMesh mesh;

    // Pre-loaded M2 data
    struct M2Ready {
        uint32_t modelId;
        pipeline::M2Model model;
        std::string path;
    };
    std::vector<M2Ready> m2Models;

    // M2 instance placement data (references modelId from m2Models)
    struct M2Placement {
        uint32_t modelId;
        uint32_t uniqueId;
        glm::vec3 position;
        glm::vec3 rotation;
        float scale;
    };
    std::vector<M2Placement> m2Placements;

    // Pre-loaded WMO data
    struct WMOReady {
        uint32_t modelId;
        uint32_t uniqueId;
        pipeline::WMOModel model;
        glm::vec3 position;
        glm::vec3 rotation;
        float scale = 1.0f;
    };
    std::vector<WMOReady> wmoModels;

    // WMO doodad M2 models (M2s placed inside WMOs)
    struct WMODoodadReady {
        uint32_t modelId;
        pipeline::M2Model model;
        glm::vec3 worldPosition;   // For frustum culling
        glm::mat4 modelMatrix;     // Pre-computed world transform
    };
    std::vector<WMODoodadReady> wmoDoodads;

    // Ambient sound emitters (detected from doodads)
    struct AmbientEmitter {
        glm::vec3 position;
        uint32_t type;  // Maps to AmbientSoundManager::AmbientType
    };
    std::vector<AmbientEmitter> ambientEmitters;

    // Pre-loaded terrain texture BLP data (loaded on background thread to avoid
    // blocking file I/O on the main thread during finalizeTile)
    std::unordered_map<std::string, pipeline::BLPImage> preloadedTextures;

    // Pre-decoded M2 model textures (decoded on background thread)
    std::unordered_map<std::string, pipeline::BLPImage> preloadedM2Textures;

    // Pre-decoded WMO textures (decoded on background thread)
    std::unordered_map<std::string, pipeline::BLPImage> preloadedWMOTextures;
    // CPU-generated normal/height pixels for those WMO textures. Keeping this
    // work on terrain workers avoids multi-second main-thread backfill stalls.
    std::unordered_map<std::string, pipeline::BLPImage> preloadedWMONormalMaps;
    std::unordered_map<std::string, float> preloadedWMONormalMapVariances;
};

/**
 * Phases for incremental tile finalization (one bounded unit of work per call)
 */
enum class FinalizationPhase {
    TERRAIN,        // Upload terrain mesh + textures + water
    M2_MODELS,      // Upload ONE M2 model per call
    M2_INSTANCES,   // Create all M2 instances (lightweight struct allocation)
    WMO_MODELS,     // Upload ONE WMO model per call
    WMO_INSTANCES,  // Create all WMO instances + load WMO liquids
    WMO_DOODADS,    // Upload ONE WMO doodad M2 per call
    WATER,          // Generate water ambient emitters
    AMBIENT,        // Register ambient emitters + commit tile
    DONE            // Fully finalized
};

/**
 * In-progress tile finalization state — tracks progress across frames
 */
struct FinalizingTile {
    std::shared_ptr<PendingTile> pending;
    FinalizationPhase phase = FinalizationPhase::TERRAIN;

    // Progress indices within current phase
    size_t m2ModelIndex     = 0;   // Next M2 model to upload
    size_t m2InstanceIndex  = 0;   // Next M2 placement to instantiate
    size_t wmoModelIndex    = 0;   // Next WMO model to upload
    size_t wmoInstanceIndex = 0;   // Next WMO placement to instantiate
    size_t wmoDoodadIndex   = 0;   // Next WMO doodad to upload
    size_t wmoLiquidGroupIndex = 0; // Next liquid group within current WMO instance

    // Incremental terrain upload state (splits TERRAIN phase across frames)
    bool terrainPreloaded = false;  // True after preloaded textures uploaded
    int terrainChunkNext = 0;       // Next chunk index to upload (0-255, row-major)
    bool terrainMeshDone = false;   // True when all chunks uploaded

    // Accumulated results (built up across phases)
    std::vector<uint32_t> m2InstanceIds;
    std::vector<uint32_t> wmoInstanceIds;
    std::vector<uint32_t> tileUniqueIds;
    std::vector<uint32_t> tileWmoUniqueIds;
    std::unordered_set<uint32_t> uploadedM2ModelIds;
};

/**
 * Terrain manager for multi-tile terrain streaming
 *
 * Handles loading and unloading terrain tiles based on camera position
 */
class TerrainManager {
public:
    TerrainManager();
    ~TerrainManager();

    /**
     * Initialize terrain manager
     * @param assetManager Asset manager for loading files
     * @param terrainRenderer Terrain renderer for GPU upload
     */
    bool initialize(pipeline::AssetManager* assetManager, TerrainRenderer* terrainRenderer);

    /**
     * Update terrain streaming based on camera position
     * @param camera Current camera
     * @param deltaTime Time since last update
     */
    void update(const Camera& camera, float deltaTime);

    /**
     * Set map name
     * @param mapName Map name (e.g., "Azeroth", "Kalimdor")
     */
    void setMapName(const std::string& mapName) { this->mapName = mapName; }
    bool isCustomZone() const { return isCustomZone_; }
    void setCustomZone(bool custom) { isCustomZone_ = custom; }

    /**
     * Load a single tile
     * @param x Tile X coordinate (0-63)
     * @param y Tile Y coordinate (0-63)
     * @return true if loaded successfully
     */
    bool loadTile(int x, int y);

    /**
     * Enqueue a tile for async loading (returns false if previously failed).
     */
    bool enqueueTile(int x, int y);

    /**
     * Unload a tile
     * @param x Tile X coordinate
     * @param y Tile Y coordinate
     */
    void unloadTile(int x, int y);

    /**
     * Unload all tiles
     */
    void unloadAll();
    void stopWorkers();  // Stop worker threads without restarting (for shutdown)
    void softReset();  // Clear tile data without stopping worker threads (non-blocking)

    /**
     * Precache a set of tiles (for taxi routes, etc.)
     * @param tiles Vector of (x, y) tile coordinates to preload
     */
    void precacheTiles(const std::vector<std::pair<int, int>>& tiles);

    /**
     * Set streaming parameters
     */
    void setLoadRadius(int radius) { loadRadius = radius; }
    void setUnloadRadius(int radius) { unloadRadius = radius; }
    void setStreamingEnabled(bool enabled) { streamingEnabled = enabled; }
    void setUpdateInterval(float seconds) { updateInterval = seconds; }
    void setTaxiStreamingMode(bool enabled) { taxiStreamingMode_ = enabled; }
    void setGroundClutterDensityScale(float scale) { groundClutterDensityScale_ = glm::clamp(scale, 0.0f, 1.5f); }
    float getGroundClutterDensityScale() const { return groundClutterDensityScale_; }
    void setWaterRenderer(WaterRenderer* renderer) { waterRenderer = renderer; }
    void setM2Renderer(M2Renderer* renderer) { m2Renderer = renderer; }
    void setWMORenderer(WMORenderer* renderer) { wmoRenderer = renderer; }
    void setAmbientSoundManager(audio::AmbientSoundManager* manager) { ambientSoundManager = manager; }

    /**
     * Get terrain height at GL coordinates
     * @param glX GL X position
     * @param glY GL Y position
     * @return Height (GL Z) if terrain loaded at that position, empty otherwise
     */
    std::optional<float> getHeightAt(float glX, float glY) const;

    /**
     * True when the MCNK chunk containing this position is cut by terrain holes.
     * getHeightAt interpolates straight across a hole and reports a surface that
     * is not there, so anything reasoning about "below the terrain" has to know
     * the heightfield is fiction here — hole-cut chunks are how cave mouths and
     * below-ground entrances are opened up. Answered per chunk rather than per
     * quad: this only ever gates a safety net, and being coarse in the safe
     * direction beats depending on the hole bit's axis order.
     */
    bool chunkHasHoles(float glX, float glY) const;

    /**
     * True when the single terrain quad under this position is cut away.
     * chunkHasHoles answers for a whole 33x33-yard chunk, which is far too
     * coarse to decide whether the player is standing on ground: a chunk
     * containing one cave mouth would report every yard of it as empty. This
     * is the per-quad question, matched to the quads the mesh builder skips.
     */
    bool isHoleAt(float glX, float glY) const;

    /** Get the precise MCNK AreaTable ID at a world position. */
    std::optional<uint32_t> getAreaIdAt(float glX, float glY) const;

    // Collision queries using WOC data (custom zones)
    bool isPositionWalkable(float glX, float glY) const;
    uint8_t getCollisionFlags(float glX, float glY) const;

    /**
     * Get dominant terrain texture name at a GL position.
     * Returns empty if terrain is not loaded at that position.
     */
    std::optional<std::string> getDominantTextureAt(float glX, float glY) const;

    /**
     * Get statistics
     */
    int getLoadedTileCount() const { return static_cast<int>(loadedTiles.size()); }
    int getPendingTileCount() const { return static_cast<int>(pendingTiles.size()); }
    int getReadyQueueCount() const { return static_cast<int>(readyQueue.size()); }
    /** Total unfinished tiles (worker threads + ready queue + finalizing) */
    int getRemainingTileCount() const { return static_cast<int>(pendingTiles.size() + readyQueue.size() + finalizingTiles_.size()); }
    TileCoord getCurrentTile() const { return currentTile; }

    /** Process all ready tiles immediately (use during loading screens) */
    void processAllReadyTiles();

    /** Process one ready tile (for loading screens with per-tile progress updates) */
    void processOneReadyTile();

    /** Process a bounded batch of ready tiles with async GPU upload (no sync wait) */
    void processReadyTiles();

    /** Unload a bounded batch of distant tiles queued by streamTiles() (no frame-spike) */
    void processPendingUnloads();

private:
    /**
     * Get tile coordinates from GL world position
     */
    TileCoord worldToTile(float worldX, float worldY) const;

    /**
     * Get world bounds for a tile
     */
    void getTileBounds(const TileCoord& coord, float& minX, float& minY,
                       float& maxX, float& maxY) const;

    /**
     * Build ADT file path
     */
    std::string getADTPath(const TileCoord& coord) const;

    /**
     * Load tiles in radius around current tile
     */
    void streamTiles();

    /**
     * Background thread: prepare tile data (CPU work only, no OpenGL)
     */
    std::shared_ptr<PendingTile> prepareTile(int x, int y);

    /**
     * Advance incremental finalization of a tile (one bounded unit of work).
     * Returns true when the tile is fully finalized (phase == DONE).
     */
    bool advanceFinalization(FinalizingTile& ft);

    /**
     * Background worker thread loop
     */
    void workerLoop();

    void ensureGroundEffectTablesLoaded();
    void generateGroundClutterPlacements(std::shared_ptr<PendingTile>& pending,
                                         std::unordered_set<uint32_t>& preparedModelIds);

    pipeline::AssetManager* assetManager = nullptr;
    TerrainRenderer* terrainRenderer = nullptr;
    WaterRenderer* waterRenderer = nullptr;
    M2Renderer* m2Renderer = nullptr;
    WMORenderer* wmoRenderer = nullptr;
    audio::AmbientSoundManager* ambientSoundManager = nullptr;

    std::string mapName = "Azeroth";

    // Loaded tiles (keyed by coordinate)
    std::unordered_map<TileCoord, std::unique_ptr<TerrainTile>, TileCoord::Hash> loadedTiles;

    // Tiles that failed to load (don't retry)
    std::unordered_map<TileCoord, bool, TileCoord::Hash> failedTiles;

    // Current tile (where camera is)
    TileCoord currentTile = {-1, -1};
    TileCoord lastStreamTile = {-1, -1};

    // Streaming parameters
    bool streamingEnabled = true;
    int loadRadius = 6;      // Load tiles within this radius (13x13 grid = 169 tiles)
    int unloadRadius = 9;    // Unload tiles beyond this radius
    float updateInterval = 0.033f;  // Check streaming every 33ms (~30 fps)
    float timeSinceLastUpdate = 0.0f;
    float proactiveStreamTimer_ = 0.0f;
    bool taxiStreamingMode_ = false;
    bool isCustomZone_ = false;

    // Collision data for custom zones (loaded from WOC files)
    struct CollisionData {
        struct Triangle { glm::vec3 v0, v1, v2; uint8_t flags; };
        std::vector<Triangle> triangles;
        glm::vec3 boundsMin{1e30f}, boundsMax{-1e30f};
        bool loaded = false;
    };
    std::unordered_map<uint64_t, CollisionData> collisionTiles_;
    uint64_t tileKey(int x, int y) const { return (static_cast<uint64_t>(x) << 32) | static_cast<uint32_t>(y); }

    // Tile size constants (WoW ADT specifications)
    // A tile (ADT) = 16x16 chunks = 533.33 units across
    // A chunk = 8x8 vertex quads = 33.33 units across
    static constexpr float TILE_SIZE = 533.33333f;          // One tile = 533.33 units
    static constexpr float CHUNK_SIZE = 33.33333f;          // One chunk = 33.33 units

    // Background loading worker pool
    std::vector<std::thread> workerThreads;
    int workerCount = 0;
    // THREAD-SAFE: guards loadQueue, readyQueue, and pendingTiles.
    // Workers wait on queueCV; main thread signals when new tiles are enqueued
    // or when readyQueue drains below maxReadyQueueSize_.
    std::mutex queueMutex;
    std::condition_variable queueCV;
    std::deque<TileCoord> loadQueue;              // THREAD-SAFE: protected by queueMutex
    std::queue<std::shared_ptr<PendingTile>> readyQueue; // THREAD-SAFE: protected by queueMutex
    // Maximum number of prepared-but-not-finalized tiles in readyQueue.
    // Each prepared tile can hold 100–500 MB of decoded textures in RAM.
    // Workers sleep when this limit is reached, letting the main thread
    // finalize (GPU-upload + free) before more tiles are prepared.
    static constexpr size_t maxReadyQueueSize_ = 3;

    // In-RAM tile cache (LRU) to avoid re-reading from disk
    struct CachedTile {
        std::shared_ptr<PendingTile> tile;
        size_t bytes = 0;
        std::list<TileCoord>::iterator lruIt;
    };
    // THREAD-SAFE: protected by tileCacheMutex_.
    std::unordered_map<TileCoord, CachedTile, TileCoord::Hash> tileCache_;
    std::list<TileCoord> tileCacheLru_;
    size_t tileCacheBytes_ = 0;
    size_t tileCacheBudgetBytes_ = 8ull * 1024 * 1024 * 1024; // Dynamic, set at init based on RAM
    std::mutex tileCacheMutex_;

    std::shared_ptr<PendingTile> getCachedTile(const TileCoord& coord);
    void putCachedTile(const std::shared_ptr<PendingTile>& tile);
    size_t estimatePendingTileBytes(const PendingTile& tile) const;
    void logMissingAdtOnce(const std::string& adtPath);
    std::atomic<bool> workerRunning{false};

    // Track tiles currently queued or being processed to avoid duplicates
    std::unordered_map<TileCoord, bool, TileCoord::Hash> pendingTiles; // THREAD-SAFE: protected by queueMutex
    std::unordered_set<std::string> missingAdtWarnings_; // THREAD-SAFE: protected by missingAdtWarningsMutex_
    std::mutex missingAdtWarningsMutex_;

    // Thread-safe set of M2 model IDs already uploaded to GPU
    // (checked by workers to skip redundant file I/O + parsing)
    std::unordered_set<uint32_t> uploadedM2Ids_;
    std::mutex uploadedM2IdsMutex_;

    // Cross-tile dedup for WMO doodad preparation on background workers
    // (prevents re-parsing thousands of doodads when same WMO spans multiple tiles)
    std::unordered_set<uint32_t> preparedWmoUniqueIds_;
    std::mutex preparedWmoUniqueIdsMutex_;

    // MAIN-THREAD-ONLY: tiles beyond unloadRadius, queued by streamTiles() and drained a
    // time-budgeted batch at a time by processPendingUnloads() each frame. Unloading them
    // all synchronously in one call (e.g. ~100 tiles right after a taxi landing snaps the
    // radius down) caused multi-second main-thread stalls — live-confirmed via "SLOW
    // terrainManager->update: 1943.71ms" immediately after "Unloaded 103 distant tiles" in
    // a real flight-landing log.
    //
    // This was originally a fixed per-frame *count* cap (8 tiles/frame) rather than a time
    // budget. That prevented the single-frame catastrophic stall but doesn't scale with
    // actual frame cost: streamTiles() discovers newly-out-of-range tiles at a roughly
    // constant real-world rate during a long, fast (taxi) flight, but a count-based drain
    // processes fewer tiles per *second* whenever frame rate drops for any reason — and once
    // the backlog (and therefore loadedTiles_/streamTiles()'s own per-call scan cost) starts
    // growing, frame rate drops further, which throttles the count-based drain further still.
    // Live-reproduced: a long cross-continent taxi flight (the first sustained-large-radius
    // tile churn any test had done) snowballed from ~50ms terrain-update frames early on to
    // 600ms+ later in the same flight. A time budget (matching processReadyTiles()/
    // advanceFinalization()'s existing pattern) scales unload throughput with whatever time
    // is actually available each frame instead of a fixed count, so it can't fall behind
    // real-world churn the way the count cap could.
    std::deque<TileCoord> pendingUnloadQueue_;

    // MAIN-THREAD-ONLY: checked and modified in processReadyTiles() and unloadDistantTiles(),
    // both of which run exclusively on the main thread.
    std::unordered_set<uint32_t> placedDoodadIds;

    // MAIN-THREAD-ONLY: same contract as placedDoodadIds.
    std::unordered_set<uint32_t> placedWmoIds;

    // Tiles currently being incrementally finalized across frames
    std::deque<FinalizingTile> finalizingTiles_;

    struct GroundEffectEntry {
        std::array<uint32_t, 4> doodadIds{{0, 0, 0, 0}};
        std::array<uint32_t, 4> weights{{0, 0, 0, 0}};
        uint32_t density = 0;
    };
    bool groundEffectsLoaded_ = false;
    std::unordered_map<uint32_t, GroundEffectEntry> groundEffectById_; // effectId -> config
    std::unordered_map<uint32_t, std::string> groundDoodadModelById_;  // doodadId -> model path
    float groundClutterDensityScale_ = 1.0f;
};

} // namespace rendering
} // namespace wowee
