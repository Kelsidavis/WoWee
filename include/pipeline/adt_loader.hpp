#pragma once

#include <span>
#include <vector>
#include <string>
#include <cstdint>
#include <array>

namespace wowee {
namespace pipeline {

/**
 * ADT chunk coordinates
 */
struct ADTCoord {
    int32_t x;
    int32_t y;
};

/**
 * Heightmap for a map chunk (9x9 + 8x8 grid)
 */
struct HeightMap {
    std::array<float, 145> heights;  // 9x9 outer + 8x8 inner vertices
    bool loaded = false;

    [[nodiscard]] float getHeight(int x, int y) const;
    [[nodiscard]] bool isLoaded() const { return loaded; }
};

/**
 * Texture layer for a map chunk
 */
struct TextureLayer {
    uint32_t textureId;      // Index into MTEX array
    uint32_t flags;          // Layer flags
    uint32_t offsetMCAL;     // Offset to alpha map in MCAL chunk
    uint32_t effectId;       // Effect ID (optional)

    [[nodiscard]] bool useAlpha() const { return (flags & 0x100) != 0; }
    [[nodiscard]] bool compressedAlpha() const { return (flags & 0x200) != 0; }
};

/**
 * Map chunk (256x256 units, 1/16 of ADT)
 */
struct MapChunk {
    uint32_t flags = 0;
    uint32_t indexX = 0;
    uint32_t indexY = 0;
    uint32_t areaId = 0;     // AreaTable ID for this precise terrain chunk
    uint16_t holes = 0;      // 4x4 bitmask for terrain holes (cave entrances, etc.)
    uint64_t noEffectDoodad = 0;  // 8x8 quad bitmask: 1 = no ground effect doodads
    // 2 bits per quad of the 8x8 grid: which of the four texture layers this
    // quad takes its ground effect from. A quad whose dominant paint is dirt
    // points at the dirt layer, and grows what dirt grows - nothing - even
    // where a grassy layer's alpha bleeds a few faint texels across it.
    std::array<uint8_t, 16> doodadMapping{};
    float position[3] = {};  // World position (X, Y, Z)

    HeightMap heightMap;
    std::vector<TextureLayer> layers;
    std::vector<uint8_t> alphaMap;  // Alpha blend maps for layers

    // Normals (compressed)
    std::array<int8_t, 145 * 3> normals;  // X, Y, Z per vertex

    [[nodiscard]] bool hasHeightMap() const { return heightMap.isLoaded(); }
    [[nodiscard]] bool hasLayers() const { return !layers.empty(); }

    // Check if a quad has a hole (y and x are quad indices 0-7)
    [[nodiscard]] bool isHole(int y, int x) const {
        int column = y / 2;
        int row = x / 2;
        int bit = 1 << (column * 4 + row);
        return (bit & holes) != 0;
    }

    // Whether the map forbids ground effect doodads on a quad (indices 0-7).
    // This is how the data keeps clutter off tilled fields, building
    // footprints and interior floors: the ground there is often painted with
    // a texture whose effect grows, and only this mask says not to.
    // Bit order matches isHole's: y-major, x fastest.
    [[nodiscard]] bool isEffectDisabled(int y, int x) const {
        return (noEffectDoodad >> (y * 8 + x)) & 1u;
    }

    // Which layer (0-3) a quad takes its ground effect from. Same y-major
    // order as the masks above, two bits per quad.
    [[nodiscard]] uint32_t effectLayerFor(int y, int x) const {
        const int quad = y * 8 + x;
        return (doodadMapping[quad / 4] >> ((quad % 4) * 2)) & 3u;
    }
};

/**
 * Complete ADT terrain tile (16x16 map chunks)
 */
struct ADTTerrain {
    bool loaded = false;
    uint32_t version = 0;

    ADTCoord coord;          // ADT coordinates (e.g., 32, 49 for Azeroth)

    // 16x16 map chunks (256 total)
    std::array<MapChunk, 256> chunks;

    // Texture filenames
    std::vector<std::string> textures;

    // Doodad definitions (M2 models)
    std::vector<std::string> doodadNames;
    std::vector<uint32_t> doodadIds;

    // WMO definitions (buildings)
    std::vector<std::string> wmoNames;
    std::vector<uint32_t> wmoIds;

    // Doodad placement data (from MDDF chunk)
    struct DoodadPlacement {
        uint32_t nameId;      // Index into doodadNames
        uint32_t uniqueId;
        float position[3];    // X, Y, Z
        float rotation[3];    // Rotation in degrees
        uint16_t scale;       // 1024 = 1.0
        uint16_t flags;
    };
    std::vector<DoodadPlacement> doodadPlacements;

    // WMO placement data (from MODF chunk)
    struct WMOPlacement {
        uint32_t nameId;      // Index into wmoNames
        uint32_t uniqueId;
        float position[3];    // X, Y, Z
        float rotation[3];    // Rotation in degrees
        float extentLower[3]; // Bounding box
        float extentUpper[3];
        uint16_t flags;
        uint16_t doodadSet;
        uint16_t nameSet = 0;     // MODF nameSet (rare; usually 0)
        uint16_t scale = 1024;    // MODF scale * 1024 (1024 = 1.0)
    };
    std::vector<WMOPlacement> wmoPlacements;

    // Water/liquid data (from MH2O chunk)
    struct WaterLayer {
        uint16_t liquidType;      // Type of liquid (0=water, 1=ocean, 2=magma, 3=slime)
        uint16_t flags;
        float minHeight;
        float maxHeight;
        uint8_t x;                // X offset within chunk (0-7)
        uint8_t y;                // Y offset within chunk (0-7)
        uint8_t width;            // Width in vertices (1-9)
        uint8_t height;           // Height in vertices (1-9)
        std::vector<float> heights;  // Height values (width * height)
        std::vector<uint8_t> mask;   // Render mask (which tiles to render)
    };

    struct ChunkWater {
        std::vector<WaterLayer> layers;
        [[nodiscard]] bool hasWater() const { return !layers.empty(); }
    };

    std::array<ChunkWater, 256> waterData;  // Water for each chunk

    MapChunk& getChunk(int x, int y) { return chunks[y * 16 + x]; }
    [[nodiscard]] const MapChunk& getChunk(int x, int y) const { return chunks[y * 16 + x]; }

    [[nodiscard]] bool isLoaded() const { return loaded; }
    [[nodiscard]] size_t getTextureCount() const { return textures.size(); }
};

/**
 * ADT terrain loader
 *
 * Loads WoW 3.3.5a ADT (Azeroth Data Tile) terrain files
 * Format specification: https://wowdev.wiki/ADT
 */
class ADTLoader {
public:
    /**
     * Load ADT terrain from byte data
     * @param adtData Raw ADT file data
     * @return Loaded terrain (check isLoaded())
     */
    static ADTTerrain load(const std::vector<uint8_t>& adtData);

private:
    // Chunk identifiers (as they appear in file when read as little-endian uint32)
    static constexpr uint32_t MVER = 0x4D564552;  // Version (ASCII "MVER")
    static constexpr uint32_t MHDR = 0x4D484452;  // Header (ASCII "MHDR")
    static constexpr uint32_t MCIN = 0x4D43494E;  // Chunk info (ASCII "MCIN")
    static constexpr uint32_t MTEX = 0x4D544558;  // Textures (ASCII "MTEX")
    static constexpr uint32_t MMDX = 0x4D4D4458;  // Doodad names (ASCII "MMDX")
    static constexpr uint32_t MMID = 0x4D4D4944;  // Doodad IDs (ASCII "MMID")
    static constexpr uint32_t MWMO = 0x4D574D4F;  // WMO names (ASCII "MWMO")
    static constexpr uint32_t MWID = 0x4D574944;  // WMO IDs (ASCII "MWID")
    static constexpr uint32_t MDDF = 0x4D444446;  // Doodad placement (ASCII "MDDF")
    static constexpr uint32_t MODF = 0x4D4F4446;  // WMO placement (ASCII "MODF")
    static constexpr uint32_t MH2O = 0x4D48324F;  // Water/liquid (ASCII "MH2O")
    static constexpr uint32_t MCNK = 0x4D434E4B;  // Map chunk (ASCII "MCNK")

    // Sub-chunks within MCNK
    static constexpr uint32_t MCVT = 0x4D435654;  // Height values (ASCII "MCVT")
    static constexpr uint32_t MCNR = 0x4D434E52;  // Normals (ASCII "MCNR")
    static constexpr uint32_t MCLY = 0x4D434C59;  // Layers (ASCII "MCLY")
    static constexpr uint32_t MCRF = 0x4D435246;  // References (ASCII "MCRF")
    static constexpr uint32_t MCSH = 0x4D435348;  // Shadow map (ASCII "MCSH")
    static constexpr uint32_t MCAL = 0x4D43414C;  // Alpha maps (ASCII "MCAL")
    static constexpr uint32_t MCLQ = 0x4D434C51;  // Liquid (deprecated) (ASCII "MCLQ")

    struct ChunkHeader {
        uint32_t magic;
        uint32_t size;
    };

    static bool readChunkHeader(std::span<const uint8_t> data, size_t offset, ChunkHeader& header);
    static uint32_t readUInt32(std::span<const uint8_t> data, size_t offset);
    static uint16_t readUInt16(std::span<const uint8_t> data, size_t offset);
    static float readFloat(std::span<const uint8_t> data, size_t offset);

    static void parseMVER(std::span<const uint8_t> data, ADTTerrain& terrain);
    static void parseMTEX(std::span<const uint8_t> data, ADTTerrain& terrain);
    static void parseMMDX(std::span<const uint8_t> data, ADTTerrain& terrain);
    static void parseMWMO(std::span<const uint8_t> data, ADTTerrain& terrain);
    static void parseMDDF(std::span<const uint8_t> data, ADTTerrain& terrain);
    static void parseMODF(std::span<const uint8_t> data, ADTTerrain& terrain);
    static void parseMCNK(std::span<const uint8_t> data, int chunkIndex, ADTTerrain& terrain);

    static void parseMCVT(std::span<const uint8_t> data, MapChunk& chunk);
    static void parseMCNR(std::span<const uint8_t> data, MapChunk& chunk);
    static void parseMCLY(std::span<const uint8_t> data, MapChunk& chunk);
    static void parseMCAL(std::span<const uint8_t> data, MapChunk& chunk);
    static void parseMH2O(std::span<const uint8_t> data, ADTTerrain& terrain);
    static void parseMCLQ(std::span<const uint8_t> data, int chunkIndex,
                          uint32_t mcnkFlags, ADTTerrain& terrain);
};

} // namespace pipeline
} // namespace wowee
