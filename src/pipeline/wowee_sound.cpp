#include "pipeline/wowee_sound.hpp"
#include "pipeline/wowee_binary_io.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>

namespace wowee {
namespace pipeline {

namespace {

constexpr char kMagic[4] = {'W', 'S', 'N', 'D'};
constexpr uint32_t kVersion = 1;
constexpr char kExtension[] = ".wsnd";

} // namespace

const WoweeSound::Entry* WoweeSound::findById(uint32_t soundId) const {
    for (const auto& e : entries) {
        if (e.soundId == soundId) return &e;
    }
    return nullptr;
}

const char* WoweeSound::kindName(uint8_t k) {
    switch (k) {
        case Sfx:     return "sfx";
        case Music:   return "music";
        case Ambient: return "ambient";
        case Ui:      return "ui";
        case Voice:   return "voice";
        case Spell:   return "spell";
        case Combat:  return "combat";
        default:      return "unknown";
    }
}

bool WoweeSoundLoader::save(const WoweeSound& cat,
                     const std::string& basePath) {
    return saveCatalog(cat, basePath, kMagic, kVersion, kExtension,
                       [](std::ofstream& os, const WoweeSound::Entry& e) {
        writePOD(os, e.soundId);
        writePOD(os, e.kind);
        writePadding(os, 3);
        writePOD(os, e.flags);
        writePOD(os, e.volume);
        writePOD(os, e.minDistance);
        writePOD(os, e.maxDistance);
        writeStr(os, e.filePath);
        writeStr(os, e.label);
                       });
}

WoweeSound WoweeSoundLoader::load(
    const std::string& basePath) {
    return loadCatalog<WoweeSound>(basePath, kMagic, kVersion, kExtension,
                              [](std::ifstream& is, WoweeSound::Entry& e) {
        if (!readPOD(is, e.soundId)) { return false; }
        if (!readPOD(is, e.kind))    { return false; }
        if (!skipPadding(is, 3)) { return false; }
        if (!readPOD(is, e.flags))       { return false; }
        if (!readPOD(is, e.volume))      { return false; }
        if (!readPOD(is, e.minDistance)) { return false; }
        if (!readPOD(is, e.maxDistance)) { return false; }
        if (!readStr(is, e.filePath))    { return false; }
        if (!readStr(is, e.label))       { return false; }
                                  return true;
                              });
}

bool WoweeSoundLoader::exists(const std::string& basePath) {
    return catalogExists(basePath, kExtension);
}

WoweeSound WoweeSoundLoader::makeStarter(const std::string& catalogName) {
    WoweeSound c;
    c.name = catalogName;
    c.entries.push_back({.soundId = 1, .kind = WoweeSound::Sfx, .flags = 0,                    .volume = 1.0f,  .minDistance = 5.0f,  .maxDistance = 30.0f, .filePath = "Sound/Sfx/footstep_grass.ogg",  .label = "Footstep (grass)"});
    c.entries.push_back({.soundId = 2, .kind = WoweeSound::Music, .flags = WoweeSound::Stream, .volume = 0.7f,  .minDistance = 0.0f,   .maxDistance = 0.0f, .filePath = "Sound/Music/main_theme.ogg",    .label = "Main theme"});
    c.entries.push_back({.soundId = 3, .kind = WoweeSound::Ambient,
                          .flags = WoweeSound::Loop | WoweeSound::Is3D,      .volume = 0.5f, .minDistance = 10.0f,  .maxDistance = 60.0f, .filePath = "Sound/Ambient/forest_loop.ogg", .label = "Forest ambience"});
    c.entries.push_back({.soundId = 4, .kind = WoweeSound::Ui, .flags = 0,                     .volume = 1.0f,  .minDistance = 0.0f,   .maxDistance = 0.0f, .filePath = "Sound/Ui/button_click.ogg",     .label = "UI button click"});
    c.entries.push_back({.soundId = 5, .kind = WoweeSound::Voice, .flags = WoweeSound::Is3D,   .volume = 1.0f,  .minDistance = 3.0f,  .maxDistance = 20.0f, .filePath = "Sound/Voice/vendor_greet.ogg",  .label = "Vendor greeting"});
    c.entries.push_back({.soundId = 6, .kind = WoweeSound::Spell, .flags = WoweeSound::Is3D,   .volume = 0.9f,  .minDistance = 4.0f,  .maxDistance = 40.0f, .filePath = "Sound/Spell/fireball_cast.ogg", .label = "Fireball cast"});
    c.entries.push_back({.soundId = 7, .kind = WoweeSound::Combat, .flags = WoweeSound::Is3D,  .volume = 0.9f,  .minDistance = 3.0f,  .maxDistance = 25.0f, .filePath = "Sound/Combat/sword_clang.ogg",  .label = "Sword clang"});
    return c;
}

WoweeSound WoweeSoundLoader::makeAmbient(const std::string& catalogName) {
    WoweeSound c;
    c.name = catalogName;
    c.entries.push_back({.soundId = 100, .kind = WoweeSound::Ambient,
                          .flags = WoweeSound::Loop | WoweeSound::Is3D, .volume = 0.4f, .minDistance = 12.0f, .maxDistance = 80.0f,
                          .filePath = "Sound/Ambient/birds_loop.ogg",   .label = "Birds (loop)"});
    c.entries.push_back({.soundId = 101, .kind = WoweeSound::Ambient,
                          .flags = WoweeSound::Loop | WoweeSound::Is3D, .volume = 0.3f, .minDistance = 20.0f, .maxDistance = 120.0f,
                          .filePath = "Sound/Ambient/wind_loop.ogg",    .label = "Wind (loop)"});
    c.entries.push_back({.soundId = 102, .kind = WoweeSound::Sfx, .flags = WoweeSound::Is3D, .volume = 0.8f, .minDistance = 4.0f, .maxDistance = 25.0f,
                          .filePath = "Sound/Sfx/footstep_grass.ogg",   .label = "Footstep grass"});
    c.entries.push_back({.soundId = 103, .kind = WoweeSound::Sfx, .flags = WoweeSound::Is3D, .volume = 0.8f, .minDistance = 4.0f, .maxDistance = 25.0f,
                          .filePath = "Sound/Sfx/footstep_dirt.ogg",    .label = "Footstep dirt"});
    c.entries.push_back({.soundId = 104, .kind = WoweeSound::Sfx, .flags = WoweeSound::Is3D, .volume = 0.8f, .minDistance = 4.0f, .maxDistance = 25.0f,
                          .filePath = "Sound/Sfx/footstep_leaves.ogg",  .label = "Footstep leaves"});
    return c;
}

WoweeSound WoweeSoundLoader::makeTavern(const std::string& catalogName) {
    WoweeSound c;
    c.name = catalogName;
    c.entries.push_back({.soundId = 200, .kind = WoweeSound::Ambient,
                          .flags = WoweeSound::Loop | WoweeSound::Is3D, .volume = 0.5f, .minDistance = 6.0f, .maxDistance = 40.0f,
                          .filePath = "Sound/Ambient/fire_crackle.ogg", .label = "Fire crackle (loop)"});
    c.entries.push_back({.soundId = 201, .kind = WoweeSound::Ambient,
                          .flags = WoweeSound::Loop, .volume = 0.4f, .minDistance = 0.0f, .maxDistance = 0.0f,
                          .filePath = "Sound/Ambient/crowd_murmur.ogg", .label = "Crowd murmur (loop)"});
    c.entries.push_back({.soundId = 202, .kind = WoweeSound::Sfx, .flags = WoweeSound::Is3D, .volume = 0.9f, .minDistance = 3.0f, .maxDistance = 15.0f,
                          .filePath = "Sound/Sfx/drink_clink.ogg",      .label = "Drink clink"});
    c.entries.push_back({.soundId = 203, .kind = WoweeSound::Sfx, .flags = WoweeSound::Is3D, .volume = 0.7f, .minDistance = 4.0f, .maxDistance = 20.0f,
                          .filePath = "Sound/Sfx/door_creak.ogg",       .label = "Door creak"});
    c.entries.push_back({.soundId = 204, .kind = WoweeSound::Music, .flags = WoweeSound::Stream, .volume = 0.6f, .minDistance = 0.0f, .maxDistance = 0.0f,
                          .filePath = "Sound/Music/tavern_lute.ogg",    .label = "Tavern lute"});
    return c;
}

} // namespace pipeline
} // namespace wowee
