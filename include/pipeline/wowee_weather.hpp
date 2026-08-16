#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Weather format (.wow) - novel replacement for WoW's
// WeatherTypes.dbc / WeatherEffect logic. A WOW file holds a list
// of weather states for one zone (clear / rain / snow / fog / etc.)
// each tagged with intensity bounds, probability weight, and
// duration bounds. The renderer / game runtime samples one entry
// at a time using weighted-random selection, drives it for a
// uniform-random duration in [minDurationSec, maxDurationSec],
// then re-rolls.
//
// Binary layout (little-endian):
//   magic[4]            = "WOWA"
//   version (uint32)    = current 1
//   nameLen (uint32) + name bytes
//   entryCount (uint32)
//   entries (each):
//     weatherTypeId (uint32)
//     minIntensity (float)
//     maxIntensity (float)
//     weight (float)              -- probability share in selection
//     minDurationSec (uint32)
//     maxDurationSec (uint32)
struct WoweeWeather {
    enum Type : uint32_t {
        Clear     = 0,
        Rain      = 1,
        Snow      = 2,
        Storm     = 3,    // rain + lightning
        Sandstorm = 4,
        Fog       = 5,
        Blizzard  = 6,
    };

    struct Entry {
        uint32_t weatherTypeId = Clear;
        float minIntensity = 0.0f;       // 0..1
        float maxIntensity = 1.0f;
        float weight = 1.0f;             // selection probability share
        uint32_t minDurationSec = 60;
        uint32_t maxDurationSec = 600;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    // Total weight across all entries - handy for normalizing
    // selection probabilities at the call site.
    [[nodiscard]] float totalWeight() const;

    static const char* typeName(uint32_t typeId);
};

class WoweeWeatherLoader {
public:
    static bool save(const WoweeWeather& weather,
                     const std::string& basePath);
    static WoweeWeather load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-weather variants.
    //   makeTemperate  - clear-dominant + occasional rain + fog
    //   makeArctic     - snow-dominant + blizzard + fog
    //   makeDesert     - clear-dominant + sandstorm
    //   makeStormy     - heavy rain + storm + occasional clear
    static WoweeWeather makeTemperate(const std::string& zoneName);
    static WoweeWeather makeArctic(const std::string& zoneName);
    static WoweeWeather makeDesert(const std::string& zoneName);
    static WoweeWeather makeStormy(const std::string& zoneName);
};

} // namespace pipeline
} // namespace wowee
