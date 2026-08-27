#include "pipeline/asset_manifest.hpp"
#include "core/logger.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <filesystem>
#include <chrono>

namespace wowee {
namespace pipeline {

namespace {

/// Reads the manifest as it streams, rather than building the document first.
///
/// The file is 32 MB and about two hundred thousand entries, and parsing it
/// into a DOM and copying out of that was 12.3% of a profiled interface load.
/// The shape is fixed and shallow:
///
///     { "version": 1, "basePath": ".", "entries": { key: {p, s, h}, ... } }
///
/// so a small state machine over the parser's callbacks fills the map directly
/// and never allocates a node for anything.
///
/// Measured with the loader's own timing, ten interleaved runs of each build
/// against the same file: 444ms for the document version, 284ms for this one.
/// Reading the whole file into a buffer first and parsing over that was also
/// tried and is 279ms, which is inside the run-to-run spread and costs a 32 MB
/// transient allocation, so it was not kept.
///
/// Do not measure this by perf's children percentage. That said the document
/// version was 12.28% and this one 9.97%, which is attribution moving rather
/// than work going away, and it is off by a factor of three against the clock.
struct ManifestSax {
    using string_t = nlohmann::json::string_t;

    int version = 0;
    std::string basePath;
    std::string expansion;
    std::unordered_map<std::string, AssetManifest::Entry>* entries = nullptr;

    // Where the parser is: 1 is the root object, 2 the entries map, 3 one
    // entry.
    int depth = 0;
    std::string rootKey;      ///< the key being read at the root
    bool inEntries = false;
    std::string entryKey;     ///< the entry currently being filled
    std::string valueKey;     ///< p, s or h
    AssetManifest::Entry entry;

    bool start_object(std::size_t hint) {
        ++depth;
        if (depth == 2 && rootKey == "entries") {
            inEntries = true;
            if (hint != static_cast<std::size_t>(-1) && entries) entries->reserve(hint);
        } else if (depth == 3) {
            entry = AssetManifest::Entry{};
        }
        return true;
    }

    bool end_object() {
        if (depth == 3 && inEntries && entries) {
            (*entries)[entryKey] = std::move(entry);
        } else if (depth == 2 && inEntries) {
            inEntries = false;
        }
        --depth;
        return true;
    }

    bool key(string_t& k) {
        if (depth == 1) rootKey = k;
        else if (depth == 2 && inEntries) entryKey = k;
        else if (depth == 3) valueKey = k;
        return true;
    }

    bool string(string_t& v) {
        if (depth == 1 && rootKey == "basePath") basePath = v;
        else if (depth == 1 && rootKey == "expansion") expansion = v;
        else if (depth == 3) {
            if (valueKey == "p") entry.filesystemPath = v;
            else if (valueKey == "h") {
                entry.crc32 = v.empty() ? 0u
                    : static_cast<uint32_t>(std::strtoul(v.c_str(), nullptr, 16));
            }
        }
        return true;
    }

    bool number_unsigned(nlohmann::json::number_unsigned_t v) {
        if (depth == 1 && rootKey == "version") version = static_cast<int>(v);
        else if (depth == 3 && valueKey == "s") entry.size = v;
        return true;
    }
    bool number_integer(nlohmann::json::number_integer_t v) {
        return number_unsigned(static_cast<nlohmann::json::number_unsigned_t>(v));
    }

    // The manifest has none of these; accepting them keeps the parser going
    // rather than failing on a field this reader does not use.
    bool null() { return true; }
    bool boolean(bool) { return true; }
    bool number_float(nlohmann::json::number_float_t, const string_t&) { return true; }
    bool binary(nlohmann::json::binary_t&) { return true; }
    bool start_array(std::size_t) { ++depth; return true; }
    bool end_array() { --depth; return true; }
    bool parse_error(std::size_t position, const std::string&,
                     const nlohmann::detail::exception& ex) {
        LOG_ERROR("Failed to parse manifest JSON at ", position, ": ", ex.what());
        return false;
    }
};

}  // namespace

bool AssetManifest::load(const std::string& manifestPath) {
    auto startTime = std::chrono::steady_clock::now();

    loaded_ = false;
    basePath_.clear();
    expansion_.clear();
    manifestDir_.clear();
    entries_.clear();

    std::ifstream file(manifestPath);
    if (!file.is_open()) {
        LOG_ERROR("Failed to open manifest: ", manifestPath);
        return false;
    }

    ManifestSax sax;
    sax.entries = &entries_;
    if (!nlohmann::json::sax_parse(file, &sax)) {
        return false;
    }

    if (sax.version != 1) {
        LOG_ERROR("Unsupported manifest version: ", sax.version);
        return false;
    }

    expansion_ = sax.expansion;
    basePath_ = sax.basePath.empty() ? "assets" : sax.basePath;
    manifestDir_ = std::filesystem::path(manifestPath).parent_path().string();

    // If basePath is relative, resolve against manifest directory
    if (!basePath_.empty() && basePath_[0] != '/') {
        basePath_ = manifestDir_ + "/" + basePath_;
    }

    loaded_ = true;

    auto elapsed = std::chrono::steady_clock::now() - startTime;
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
    LOG_INFO("Loaded asset manifest: ", entries_.size(), " entries in ", ms, "ms (base: ", basePath_, ")");

    return true;
}

const AssetManifest::Entry* AssetManifest::lookup(const std::string& normalizedWowPath) const {
    auto it = entries_.find(normalizedWowPath);
    if (it != entries_.end()) {
        return &it->second;
    }
    return nullptr;
}

std::string AssetManifest::resolveFilesystemPath(const std::string& normalizedWowPath) const {
    auto it = entries_.find(normalizedWowPath);
    if (it == entries_.end()) {
        return {};
    }
    return basePath_ + "/" + it->second.filesystemPath;
}

bool AssetManifest::hasEntry(const std::string& normalizedWowPath) const {
    return entries_.find(normalizedWowPath) != entries_.end();
}

} // namespace pipeline
} // namespace wowee
