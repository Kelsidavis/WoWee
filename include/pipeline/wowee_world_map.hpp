#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open World Map index (.womx) - novel replacement for
// Blizzard's WDT (top-level world definition table). A WOMX file
// holds the manifest of which terrain tiles exist within a world,
// plus a tiny bit of map-level metadata. The runtime consults it
// before attempting to load any individual tile (so missing tiles
// produce a clean "no data" instead of a file-not-found error).
//
// The grid is a square of size N×N where N is typically 64 (the
// historical WoW value), but the format permits any N up to 128.
// Tile presence is stored as a packed bitmap (1 bit per tile,
// row-major), so a 64×64 manifest is only 512 bytes.
//
// Binary layout (little-endian):
//   magic[4]            = "WMPX"
//   version (uint32)    = current 1
//   nameLen (uint32) + name bytes
//   worldType (uint8)   = 0=continent, 1=instance, 2=battleground, 3=arena
//   gridSize (uint8)    = N (1..128)
//   pad[2]
//   defaultLightId (uint32)   -- 0 if no atmosphere preset
//   defaultWeatherId (uint32) -- 0 if no atmosphere preset
//   reserved[3] (uint32 each)
//   bitmapBytes (uint32) = ceil(N*N/8)
//   bitmap (bitmapBytes)
struct WoweeWorldMap {
    enum WorldType : uint8_t {
        Continent    = 0,
        Instance     = 1,
        Battleground = 2,
        Arena        = 3,
    };

    std::string name;
    uint8_t worldType = Continent;
    uint8_t gridSize = 64;
    uint32_t defaultLightId = 0;
    uint32_t defaultWeatherId = 0;

    // Packed row-major bitmap: bit (y * gridSize + x) is set
    // when a tile exists at column x, row y.
    std::vector<uint8_t> tileBitmap;

    [[nodiscard]] bool isValid() const { return gridSize > 0 && gridSize <= 128; }

    [[nodiscard]] bool hasTile(uint32_t x, uint32_t y) const;
    void setTile(uint32_t x, uint32_t y, bool present);

    // Count of set bits in the bitmap (= number of present tiles).
    [[nodiscard]] uint32_t countTiles() const;

    static const char* worldTypeName(uint8_t t);
};

class WoweeWorldMapLoader {
public:
    static bool save(const WoweeWorldMap& map,
                     const std::string& basePath);
    static WoweeWorldMap load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-world-map* variants.
    //
    //   makeContinent - full 64×64 grid with all tiles present
    //                    (continent-style world map, ~4.3 km²)
    //   makeInstance  - small 4×4 grid for dungeon-scale worlds
    //   makeArena     - 1×1 single-tile arena
    static WoweeWorldMap makeContinent(const std::string& mapName);
    static WoweeWorldMap makeInstance(const std::string& mapName);
    static WoweeWorldMap makeArena(const std::string& mapName);
};

} // namespace pipeline
} // namespace wowee
