#include "game/expansion_profile.hpp"
#include "game/inventory_slots.hpp"
#include "core/logger.hpp"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cstdlib>
#include <limits>

// Minimal JSON parsing (no external dependency) - expansion.json is tiny and flat.
// We parse the subset we need: strings, integers, arrays of integers.
namespace {

std::string trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n\"");
    size_t end = s.find_last_not_of(" \t\r\n\",");
    if (start == std::string::npos) return "";
    return s.substr(start, end - start + 1);
}

// Quick-and-dirty JSON value extractor for flat objects.
// Returns the raw value string for a given key, or empty.
std::string jsonValue(const std::string& json, const std::string& key) {
    std::string needle = "\"" + key + "\"";
    auto pos = json.find(needle);
    if (pos == std::string::npos) return "";
    pos = json.find(':', pos + needle.size());
    if (pos == std::string::npos) return "";
    ++pos;
    // Skip whitespace
    while (pos < json.size() && (json[pos] == ' ' || json[pos] == '\t' || json[pos] == '\r' || json[pos] == '\n'))
        ++pos;
    if (pos >= json.size()) return "";

    if (json[pos] == '"') {
        // String value
        size_t end = json.find('"', pos + 1);
        return (end != std::string::npos) ? json.substr(pos + 1, end - pos - 1) : "";
    }
    if (json[pos] == '{') {
        // Nested object - return content between braces
        size_t depth = 1;
        size_t start = pos + 1;
        for (size_t i = start; i < json.size() && depth > 0; ++i) {
            if (json[i] == '{') ++depth;
            else if (json[i] == '}') { --depth; if (depth == 0) return json.substr(start, i - start); }
        }
        return "";
    }
    if (json[pos] == '[') {
        // Array - return content between brackets (including brackets)
        size_t end = json.find(']', pos);
        return (end != std::string::npos) ? json.substr(pos, end - pos + 1) : "";
    }
    // Number or other literal
    size_t end = json.find_first_of(",}\n\r", pos);
    return trim(json.substr(pos, end - pos));
}

int jsonInt(const std::string& json, const std::string& key, int def = 0) {
    std::string v = jsonValue(json, key);
    if (v.empty()) return def;
    try {
        return std::stoi(v);
    } catch (...) {
        // Non-numeric value for an integer field - fall back to default rather than
        // crashing, but log it so malformed expansion.json files are diagnosable.
        wowee::core::Logger::getInstance().warning("jsonInt: failed to parse '", key, "' value '", v, "', using default ", def);
        return def;
    }
}

std::vector<uint32_t> jsonUintArray(const std::string& json, const std::string& key) {
    std::vector<uint32_t> result;
    std::string arr = jsonValue(json, key);
    if (arr.empty() || arr.front() != '[') return result;
    // Strip brackets
    arr = arr.substr(1, arr.size() - 2);
    std::istringstream ss(arr);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        std::string t = trim(tok);
        if (!t.empty()) {
            try { result.push_back(static_cast<uint32_t>(std::stoul(t))); } catch (...) {}
        }
    }
    return result;
}

} // namespace

namespace wowee {
namespace game {

std::string ExpansionProfile::versionString() const {
    std::ostringstream ss;
    ss << static_cast<int>(majorVersion) << "." << static_cast<int>(minorVersion) << "." << static_cast<int>(patchVersion);
    // Append letter suffix for known builds
    if (majorVersion == 3 && minorVersion == 3 && patchVersion == 5) ss << "a";
    else if (majorVersion == 2 && minorVersion == 4 && patchVersion == 3) ss << "";
    else if (majorVersion == 1 && minorVersion == 12 && patchVersion == 1) ss << "";
    return ss.str();
}

size_t ExpansionRegistry::initialize(const std::string& dataRoot) {
    profiles_.clear();
    activeId_.clear();

    std::string expansionsDir = dataRoot + "/expansions";
    std::error_code ec;
    if (!std::filesystem::is_directory(expansionsDir, ec)) {
        LOG_WARNING("ExpansionRegistry: no expansions/ directory at ", expansionsDir);
        return 0;
    }

    for (auto& entry : std::filesystem::directory_iterator(expansionsDir, ec)) {
        if (!entry.is_directory()) continue;
        std::string jsonPath = entry.path().string() + "/expansion.json";
        if (std::filesystem::exists(jsonPath, ec)) {
            loadProfile(jsonPath, entry.path().string());
        }
    }

    // Sort by build number (ascending: classic < tbc < wotlk < cata)
    std::sort(profiles_.begin(), profiles_.end(),
              [](const ExpansionProfile& a, const ExpansionProfile& b) { return a.build < b.build; });

    // Prefer WotLK when it has extracted assets. Otherwise choose the highest
    // extracted profile so a single-expansion installation can initialize its
    // asset manager without requiring a legacy root Data/manifest.json.
    if (!profiles_.empty()) {
        auto hasManifest = [](const ExpansionProfile& p) {
            std::error_code existsEc;
            return std::filesystem::exists(p.dataPath + "/manifest.json", existsEc);
        };
        auto it = std::find_if(profiles_.begin(), profiles_.end(),
                               [&](const ExpansionProfile& p) {
                                   return p.id == "wotlk" && hasManifest(p);
                               });
        if (it == profiles_.end()) {
            it = std::find_if(profiles_.rbegin(), profiles_.rend(), hasManifest).base();
            if (it != profiles_.begin()) --it;
            else if (!hasManifest(*it)) it = profiles_.end();
        }
        if (it == profiles_.end()) {
            it = std::find_if(profiles_.begin(), profiles_.end(),
                              [](const ExpansionProfile& p) { return p.id == "wotlk"; });
        }
        activeId_ = (it != profiles_.end()) ? it->id : profiles_.back().id;
    }
    activeChanged();

    LOG_INFO("ExpansionRegistry: discovered ", profiles_.size(), " expansion(s), active=", activeId_);
    return profiles_.size();
}

const ExpansionProfile* ExpansionRegistry::getProfile(const std::string& id) const {
    for (auto& p : profiles_) {
        if (p.id == id) return &p;
    }
    return nullptr;
}

void ExpansionRegistry::activeChanged() {
    // The bank is not the same shape in every expansion - vanilla has 24
    // general slots and 6 bags where 2.0 onward have 28 and 7, and the keyring
    // moves with them - and a slot number is what a move names.
    //
    // Both places that move activeId_ come through here. Discovery sets it
    // directly rather than through setActive, so hanging this off setActive
    // alone would have left every auto-detected install - which is every
    // ordinary one - on the layout it was not playing.
    slots::setBankLayout(
        slots::bankLayoutFor(activeId_ == "classic" || activeId_ == "turtle"));
}

bool ExpansionRegistry::setActive(const std::string& id) {
    if (!getProfile(id)) return false;
    activeId_ = id;
    activeChanged();
    return true;
}

const ExpansionProfile* ExpansionRegistry::getActive() const {
    return getProfile(activeId_);
}

bool ExpansionRegistry::loadProfile(const std::string& jsonPath, const std::string& dirPath) {
    std::ifstream f(jsonPath);
    if (!f.is_open()) return false;

    std::string json((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());

    ExpansionProfile p;
    p.id = jsonValue(json, "id");
    p.name = jsonValue(json, "name");
    p.shortName = jsonValue(json, "shortName");
    p.dataPath = dirPath;

    // Version nested object
    std::string ver = jsonValue(json, "version");
    if (!ver.empty()) {
        p.majorVersion = static_cast<uint8_t>(jsonInt(ver, "major"));
        p.minorVersion = static_cast<uint8_t>(jsonInt(ver, "minor"));
        p.patchVersion = static_cast<uint8_t>(jsonInt(ver, "patch"));
    }

    p.build = static_cast<uint16_t>(jsonInt(json, "build"));
    p.worldBuild = static_cast<uint16_t>(jsonInt(json, "worldBuild", p.build));
    // Custom clients sometimes advance their authentication build without
    // changing the underlying Vanilla world protocol. Permit a profile-scoped
    // override so archived/private servers on an older Turtle revision remain
    // usable without editing tracked expansion metadata.
    const std::string buildEnv = jsonValue(json, "buildEnv");
    if (!buildEnv.empty()) {
        if (const char* raw = std::getenv(buildEnv.c_str()); raw && *raw) {
            try {
                const unsigned long overrideBuild = std::stoul(raw);
                if (overrideBuild == 0 || overrideBuild > std::numeric_limits<uint16_t>::max()) {
                    throw std::out_of_range("client build");
                }
                p.build = static_cast<uint16_t>(overrideBuild);
                LOG_WARNING("ExpansionRegistry: overriding auth build via ", buildEnv,
                            "=", p.build);
            } catch (...) {
                LOG_WARNING("ExpansionRegistry: ignoring invalid ", buildEnv,
                            "='", raw, "'");
            }
        }
    }
    p.protocolVersion = static_cast<uint8_t>(jsonInt(json, "protocolVersion"));
    // Optional client header fields (LOGON_CHALLENGE)
    {
        std::string v;
        v = jsonValue(json, "game"); if (!v.empty()) p.game = v;
        v = jsonValue(json, "platform"); if (!v.empty()) p.platform = v;
        v = jsonValue(json, "os"); if (!v.empty()) p.os = v;
        v = jsonValue(json, "locale"); if (!v.empty()) p.locale = v;
        p.timezone = static_cast<uint32_t>(jsonInt(json, "timezone", static_cast<int>(p.timezone)));
    }
    p.maxLevel = static_cast<uint32_t>(jsonInt(json, "maxLevel", 60));

    // The realm's own Warden signing key, if it has one. 512 hex characters;
    // anything else is refused rather than half-read, because a key that is
    // wrong in one nibble fails exactly the way no key at all does and would
    // be looked for anywhere but here.
    if (const std::string hex = jsonValue(json, "wardenRsaModulus"); !hex.empty()) {
        auto nibble = [](char c) -> int {
            if (c >= '0' && c <= '9') return c - '0';
            if (c >= 'a' && c <= 'f') return c - 'a' + 10;
            if (c >= 'A' && c <= 'F') return c - 'A' + 10;
            return -1;
        };
        bool ok = (hex.size() == 512);
        std::vector<uint8_t> bytes;
        bytes.reserve(256);
        for (size_t i = 0; ok && i + 1 < hex.size(); i += 2) {
            const int hi = nibble(hex[i]);
            const int lo = nibble(hex[i + 1]);
            if (hi < 0 || lo < 0) { ok = false; break; }
            bytes.push_back(static_cast<uint8_t>((hi << 4) | lo));
        }
        if (ok) {
            p.wardenRsaModulus = std::move(bytes);
        } else {
            wowee::core::Logger::getInstance().warning(
                "Expansion '", p.id, "': wardenRsaModulus is not 512 hex characters"
                " - ignoring it and checking against the retail key");
        }
    }
    p.races = jsonUintArray(json, "races");
    p.classes = jsonUintArray(json, "classes");

    if (p.id.empty() || p.build == 0) {
        LOG_WARNING("ExpansionRegistry: skipping invalid profile at ", jsonPath);
        return false;
    }

    LOG_INFO("ExpansionRegistry: loaded '", p.name, "' (", p.shortName,
             ") v", p.versionString(), " build=", p.build);
    profiles_.push_back(std::move(p));
    return true;
}

} // namespace game
} // namespace wowee
