#include "pipeline/wowee_weather.hpp"
#include "pipeline/wowee_binary_io.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>

namespace wowee {
namespace pipeline {

namespace {

constexpr char kMagic[4] = {'W', 'O', 'W', 'A'};
constexpr uint32_t kVersion = 1;
constexpr char kExtension[] = ".wow";

} // namespace

float WoweeWeather::totalWeight() const {
    float t = 0.0f;
    for (const auto& e : entries) t += e.weight;
    return t;
}

const char* WoweeWeather::typeName(uint32_t typeId) {
    switch (typeId) {
        case Clear:     return "clear";
        case Rain:      return "rain";
        case Snow:      return "snow";
        case Storm:     return "storm";
        case Sandstorm: return "sandstorm";
        case Fog:       return "fog";
        case Blizzard:  return "blizzard";
        default:        return "unknown";
    }
}

bool WoweeWeatherLoader::save(const WoweeWeather& w,
                     const std::string& basePath) {
    return saveCatalog(w, basePath, kMagic, kVersion, kExtension,
                       [](std::ofstream& os, const WoweeWeather::Entry& e) {
        writePOD(os, e.weatherTypeId);
        writePOD(os, e.minIntensity);
        writePOD(os, e.maxIntensity);
        writePOD(os, e.weight);
        writePOD(os, e.minDurationSec);
        writePOD(os, e.maxDurationSec);
                       });
}

WoweeWeather WoweeWeatherLoader::load(const std::string& basePath) {
    WoweeWeather out;
    std::ifstream is(normalizePath(basePath, kExtension), std::ios::binary);
    if (!is) return out;
    // The name length used to be read with no cap, and the entry count with no
    // cap either - this format wrote its own header rather than using the one
    // every other format uses, and so missed both.
    uint32_t entryCount = 0;
    if (!readCatalogHeader(is, kMagic, kVersion, out.name, entryCount)) return out;
    out.entries.resize(entryCount);
    for (auto& e : out.entries) {
        if (!readPOD(is, e.weatherTypeId) ||
            !readPOD(is, e.minIntensity) ||
            !readPOD(is, e.maxIntensity) ||
            !readPOD(is, e.weight) ||
            !readPOD(is, e.minDurationSec) ||
            !readPOD(is, e.maxDurationSec)) {
            out.entries.clear();
            return out;
        }
    }
    return out;
}

bool WoweeWeatherLoader::exists(const std::string& basePath) {
    return catalogExists(basePath, kExtension);
}

WoweeWeather WoweeWeatherLoader::makeTemperate(const std::string& zoneName) {
    WoweeWeather w;
    w.name = zoneName;
    w.entries.push_back({.weatherTypeId = WoweeWeather::Clear, .minIntensity = 0.0f, .maxIntensity = 0.0f, .weight = 6.0f, .minDurationSec = 300, .maxDurationSec = 1800});
    w.entries.push_back({.weatherTypeId = WoweeWeather::Rain,  .minIntensity = 0.3f, .maxIntensity = 0.7f, .weight = 2.0f, .minDurationSec = 120, .maxDurationSec = 900});
    w.entries.push_back({.weatherTypeId = WoweeWeather::Fog,   .minIntensity = 0.4f, .maxIntensity = 0.8f, .weight = 1.0f, .minDurationSec = 180, .maxDurationSec = 600});
    return w;
}

WoweeWeather WoweeWeatherLoader::makeArctic(const std::string& zoneName) {
    WoweeWeather w;
    w.name = zoneName;
    w.entries.push_back({.weatherTypeId = WoweeWeather::Snow,     .minIntensity = 0.3f, .maxIntensity = 0.7f, .weight = 5.0f, .minDurationSec = 300, .maxDurationSec = 1800});
    w.entries.push_back({.weatherTypeId = WoweeWeather::Blizzard, .minIntensity = 0.7f, .maxIntensity = 1.0f, .weight = 2.0f, .minDurationSec = 120, .maxDurationSec = 600});
    w.entries.push_back({.weatherTypeId = WoweeWeather::Fog,      .minIntensity = 0.5f, .maxIntensity = 0.9f, .weight = 2.0f, .minDurationSec = 180, .maxDurationSec = 900});
    w.entries.push_back({.weatherTypeId = WoweeWeather::Clear,    .minIntensity = 0.0f, .maxIntensity = 0.0f, .weight = 1.0f, .minDurationSec = 180, .maxDurationSec = 600});
    return w;
}

WoweeWeather WoweeWeatherLoader::makeDesert(const std::string& zoneName) {
    WoweeWeather w;
    w.name = zoneName;
    w.entries.push_back({.weatherTypeId = WoweeWeather::Clear,     .minIntensity = 0.0f, .maxIntensity = 0.0f, .weight = 8.0f, .minDurationSec = 600, .maxDurationSec = 2400});
    w.entries.push_back({.weatherTypeId = WoweeWeather::Sandstorm, .minIntensity = 0.5f, .maxIntensity = 0.9f, .weight = 2.0f, .minDurationSec = 120, .maxDurationSec = 600});
    return w;
}

WoweeWeather WoweeWeatherLoader::makeStormy(const std::string& zoneName) {
    WoweeWeather w;
    w.name = zoneName;
    w.entries.push_back({.weatherTypeId = WoweeWeather::Rain,  .minIntensity = 0.5f, .maxIntensity = 0.9f, .weight = 5.0f, .minDurationSec = 300, .maxDurationSec = 1200});
    w.entries.push_back({.weatherTypeId = WoweeWeather::Storm, .minIntensity = 0.6f, .maxIntensity = 1.0f, .weight = 3.0f, .minDurationSec = 180, .maxDurationSec = 600});
    w.entries.push_back({.weatherTypeId = WoweeWeather::Fog,   .minIntensity = 0.4f, .maxIntensity = 0.7f, .weight = 1.0f, .minDurationSec = 120, .maxDurationSec = 300});
    w.entries.push_back({.weatherTypeId = WoweeWeather::Clear, .minIntensity = 0.0f, .maxIntensity = 0.0f, .weight = 1.0f, .minDurationSec = 60, .maxDurationSec = 240});
    return w;
}

} // namespace pipeline
} // namespace wowee
