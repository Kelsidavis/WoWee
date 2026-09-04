#pragma once

#include <string>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace wowee {
namespace pipeline { class AssetManager; }
namespace game {

// Conservative tile fallback for Duskwood's interior. Northern Duskwood and
// southern Elwynn share ADTs, so their bank areas must be distinguished through
// AreaTable parentage rather than broad tile rectangles.
constexpr bool isDuskwoodAdtTile(int tileX, int tileY) {
    const bool runtime = tileX >= 33 && tileX <= 35 && tileY >= 52 && tileY <= 53;
    const bool transposed = tileX >= 52 && tileX <= 53 && tileY >= 33 && tileY <= 35;
    return runtime || transposed;
}

struct ZoneInfo {
    uint32_t id;
    std::string name;
    std::vector<std::string> musicPaths;  // MPQ paths to music files
};

class ZoneManager {
public:
    void initialize();

    // Supplement zone music paths using AreaTable → ZoneMusic → SoundEntries DBC chain.
    // Safe to call after initialize(); idempotent and additive (does not remove existing paths).
    void enrichFromDBC(pipeline::AssetManager* assets);

    [[nodiscard]] uint32_t getZoneId(int tileX, int tileY) const;
    [[nodiscard]] uint32_t resolveAreaZoneId(uint32_t areaId) const;

    /// Whether an area is a world PvP objective - one of the eleven the
    /// AREA_FLAG_OUTDOOR_PVP bit marks, or anywhere in Wintergrasp, which
    /// carries a flag of its own instead.
    ///
    /// Field 4 of AreaTable.dbc, verified against two rows rather than taken
    /// from the position: Wintergrasp's own area carries 0x01000000 there and
    /// Shattrath carries the sanctuary bit, and neither would hold if the
    /// column were something else.
    [[nodiscard]] bool isOutdoorPvpArea(uint32_t areaId) const;
    [[nodiscard]] const ZoneInfo* getZoneInfo(uint32_t zoneId) const;
    std::string getRandomMusic(uint32_t zoneId);
    [[nodiscard]] std::vector<std::string> getAllMusicPaths() const;

    /// Resolve "assets/Original Music/<filename>" to a "file:<absolute path>"
    /// track the music manager plays off disk, or "" when the file is absent.
    /// Shared with the tavern rotation, which draws from the same directory.
    [[nodiscard]] static std::string resolveOriginalMusicFile(const char* filename);

    // When false, file: (original soundtrack) tracks are excluded from the pool
    void setUseOriginalSoundtrack(bool use) { useOriginalSoundtrack_ = use; }
    [[nodiscard]] bool getUseOriginalSoundtrack() const { return useOriginalSoundtrack_; }

private:
    // tile key = tileX * 100 + tileY
    std::unordered_map<int, uint32_t> tileToZone;
    std::unordered_map<uint32_t, ZoneInfo> zones;
    std::unordered_map<uint32_t, uint32_t> areaParents_;
    /// AreaTable's Flags column, by area id. Read in the same pass as the
    /// parents; the file is walked once either way.
    std::unordered_map<uint32_t, uint32_t> areaFlags_;
    std::string lastPlayedMusic_;
    bool useOriginalSoundtrack_ = true;
};

} // namespace game
} // namespace wowee
