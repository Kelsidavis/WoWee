#include "extractor.hpp"
#include "path_mapper.hpp"
#include "manifest_writer.hpp"

#include <StormLib.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <set>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "pipeline/dbc_loader.hpp"

#ifndef INVALID_HANDLE_VALUE
#define INVALID_HANDLE_VALUE ((HANDLE)(long long)-1)
#endif

namespace wowee {
namespace tools {

namespace fs = std::filesystem;
using wowee::pipeline::DBCFile;

// Archive descriptor for priority-based loading
struct ArchiveDesc {
    std::string path;
    int priority;
};

static std::string toLowerStr(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

static std::string normalizeWowPath(const std::string& path) {
    std::string n = path;
    std::replace(n.begin(), n.end(), '/', '\\');
    std::transform(n.begin(), n.end(), n.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return n;
}

static bool shouldSkipFile(const Extractor::Options& opts, const std::string& wowPath) {
    if (!opts.skipDbcExtraction) {
        return false;
    }
    std::string n = normalizeWowPath(wowPath);
    if (n.rfind("dbfilesclient\\", 0) == 0) {
        if (n.size() >= 4 && n.substr(n.size() - 4) == ".dbc") {
            return true;
        }
    }
    return false;
}

static std::vector<uint8_t> readFileBytes(const std::string& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) return {};
    auto size = f.tellg();
    if (size <= 0) return {};
    f.seekg(0);
    std::vector<uint8_t> buf(static_cast<size_t>(size));
    f.read(reinterpret_cast<char*>(buf.data()), size);
    return buf;
}

static bool isValidStringOffset(const std::vector<uint8_t>& stringBlock, uint32_t offset) {
    if (offset >= stringBlock.size()) return false;
    for (size_t i = offset; i < stringBlock.size(); ++i) {
        uint8_t c = stringBlock[i];
        if (c == 0) return true;
        if (c < 0x20 && c != '\t' && c != '\n' && c != '\r') return false;
    }
    return false;
}

static std::set<uint32_t> detectStringColumns(const DBCFile& dbc,
                                              const std::vector<uint8_t>& rawData) {
    const uint32_t recordCount = dbc.getRecordCount();
    const uint32_t fieldCount = dbc.getFieldCount();
    const uint32_t recordSize = dbc.getRecordSize();
    const uint32_t strBlockSize = dbc.getStringBlockSize();

    constexpr size_t kHeaderSize = 20;
    const size_t strBlockOffset = kHeaderSize + static_cast<size_t>(recordCount) * recordSize;

    std::vector<uint8_t> stringBlock;
    if (strBlockSize > 0 && strBlockOffset + strBlockSize <= rawData.size()) {
        stringBlock.assign(rawData.begin() + strBlockOffset,
                           rawData.begin() + strBlockOffset + strBlockSize);
    }

    std::set<uint32_t> cols;
    if (stringBlock.size() <= 1) return cols;

    for (uint32_t col = 0; col < fieldCount; ++col) {
        bool allZeroOrValid = true;
        bool hasNonZero = false;

        for (uint32_t row = 0; row < recordCount; ++row) {
            uint32_t val = dbc.getUInt32(row, col);
            if (val == 0) continue;
            hasNonZero = true;
            if (!isValidStringOffset(stringBlock, val)) {
                allZeroOrValid = false;
                break;
            }
        }

        if (allZeroOrValid && hasNonZero) {
            cols.insert(col);
        }
    }

    return cols;
}

static std::string csvEscape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 2);
    out.push_back('"');
    for (char c : s) {
        if (c == '"') out.push_back('"');
        out.push_back(c);
    }
    out.push_back('"');
    return out;
}

static bool convertDbcToCsv(const std::string& dbcPath, const std::string& csvPath) {
    auto rawData = readFileBytes(dbcPath);
    if (rawData.size() < 4 || std::memcmp(rawData.data(), "WDBC", 4) != 0) {
        std::cerr << "  DBC missing or not WDBC: " << dbcPath << "\n";
        return false;
    }

    DBCFile dbc;
    if (!dbc.load(rawData) || !dbc.isLoaded()) {
        std::cerr << "  Failed to parse DBC: " << dbcPath << "\n";
        return false;
    }

    const auto stringCols = detectStringColumns(dbc, rawData);

    fs::path outPath(csvPath);
    std::error_code ec;
    fs::create_directories(outPath.parent_path(), ec);
    if (ec) {
        std::cerr << "  Failed to create dir: " << outPath.parent_path().string()
                  << " (" << ec.message() << ")\n";
        return false;
    }

    std::ofstream out(csvPath, std::ios::binary);
    if (!out) {
        std::cerr << "  Failed to write: " << csvPath << "\n";
        return false;
    }

    out << "# fields=" << dbc.getFieldCount();
    if (!stringCols.empty()) {
        out << " strings=";
        bool first = true;
        for (uint32_t col : stringCols) {
            if (!first) out << ",";
            out << col;
            first = false;
        }
    }
    out << "\n";

    for (uint32_t row = 0; row < dbc.getRecordCount(); ++row) {
        for (uint32_t col = 0; col < dbc.getFieldCount(); ++col) {
            if (col > 0) out << ",";
            if (stringCols.count(col)) {
                out << csvEscape(dbc.getString(row, col));
            } else {
                out << dbc.getUInt32(row, col);
            }
        }
        out << "\n";
    }

    return true;
}

static std::vector<std::string> getUsedDbcNamesForExpansion(const std::string& expansion) {
    // Keep this list small: these are the ~30 tables wowee actually uses.
    // Other DBCs can remain extracted (ignored) as binary.
    (void)expansion;
    return {
        "AreaTable",
        "CharSections",
        "CharHairGeosets",
        "CharacterFacialHairStyles",
        "CreatureDisplayInfo",
        "CreatureDisplayInfoExtra",
        "CreatureModelData",
        "Emotes",
        "EmotesText",
        "EmotesTextData",
        "Faction",
        "FactionTemplate",
        "GameObjectDisplayInfo",
        "ItemDisplayInfo",
        "Light",
        "LightParams",
        "LightIntBand",
        "LightFloatBand",
        "Map",
        "SkillLine",
        "SkillLineAbility",
        "Spell",
        "SpellIcon",
        "Talent",
        "TalentTab",
        "TaxiNodes",
        "TaxiPath",
        "TaxiPathNode",
        "TransportAnimation",
        "WorldMapArea",
    };
}

static std::unordered_set<std::string> buildWantedDbcSet(const Extractor::Options& opts) {
    std::unordered_set<std::string> wanted;
    if (!opts.onlyUsedDbcs) {
        return wanted;
    }

    for (const auto& base : getUsedDbcNamesForExpansion(opts.expansion)) {
        // normalizeWowPath lowercases and uses backslashes.
        wanted.insert(normalizeWowPath("DBFilesClient\\" + base + ".dbc"));
    }
    return wanted;
}

// Parse a quoted JSON string starting after the opening quote at pos.
// Returns the unescaped string and advances pos past the closing quote.
static std::string parseJsonString(const std::string& line, size_t& pos) {
    std::string result;
    while (pos < line.size() && line[pos] != '"') {
        if (line[pos] == '\\' && pos + 1 < line.size()) {
            result += line[pos + 1];
            pos += 2;
        } else {
            result += line[pos];
            pos++;
        }
    }
    if (pos < line.size()) pos++; // skip closing quote
    return result;
}

// Load all entries from a manifest.json into a map keyed by normalized WoW path.
// Minimal parser that extracts keys and values without a full JSON library.
static std::unordered_map<std::string, ManifestWriter::FileEntry> loadManifestEntries(
    const std::string& manifestPath) {
    std::unordered_map<std::string, ManifestWriter::FileEntry> entries;
    std::ifstream f(manifestPath);
    if (!f.is_open()) return entries;

    bool inEntries = false;
    std::string line;
    while (std::getline(f, line)) {
        if (!inEntries) {
            if (line.find("\"entries\"") != std::string::npos) inEntries = true;
            continue;
        }

        size_t closeBrace = line.find_first_not_of(" \t");
        if (closeBrace != std::string::npos && line[closeBrace] == '}') break;

        // Extract key
        size_t q1 = line.find('"');
        if (q1 == std::string::npos) continue;
        size_t pos = q1 + 1;
        std::string key = parseJsonString(line, pos);
        if (key.empty()) continue;

        // Extract value object fields: "p", "s", "h"
        ManifestWriter::FileEntry entry;
        entry.wowPath = key;

        size_t pPos = line.find("\"p\":", pos);
        if (pPos != std::string::npos) {
            size_t pq = line.find('"', pPos + 4);
            if (pq != std::string::npos) {
                size_t pp = pq + 1;
                entry.filesystemPath = parseJsonString(line, pp);
            }
        }

        size_t sPos = line.find("\"s\":", pos);
        if (sPos != std::string::npos) {
            size_t numStart = sPos + 4;
            while (numStart < line.size() && (line[numStart] == ' ')) numStart++;
            entry.size = std::strtoull(line.c_str() + numStart, nullptr, 10);
        }

        size_t hPos = line.find("\"h\":", pos);
        if (hPos != std::string::npos) {
            size_t hq = line.find('"', hPos + 4);
            if (hq != std::string::npos) {
                size_t hp = hq + 1;
                std::string hexStr = parseJsonString(line, hp);
                entry.crc32 = static_cast<uint32_t>(std::strtoul(hexStr.c_str(), nullptr, 16));
            }
        }

        entries[key] = std::move(entry);
    }

    return entries;
}

// Load all entry keys from a manifest.json into a set of normalized WoW paths.
static std::unordered_set<std::string> loadManifestKeys(const std::string& manifestPath) {
    auto entries = loadManifestEntries(manifestPath);
    std::unordered_set<std::string> keys;
    keys.reserve(entries.size());
    for (auto& [k, v] : entries) {
        keys.insert(k);
    }
    return keys;
}

// Known WoW client locales
static const std::vector<std::string> kKnownLocales = {
    "enUS", "enGB", "deDE", "frFR", "esES", "esMX",
    "ruRU", "koKR", "zhCN", "zhTW", "ptBR"
};

std::string Extractor::detectExpansion(const std::string& mpqDir) {
    if (fs::exists(mpqDir + "/lichking.MPQ"))
        return "wotlk";
    if (fs::exists(mpqDir + "/expansion.MPQ"))
        return "tbc";
    // Turtle WoW typically uses vanilla-era base MPQs plus letter patch MPQs (patch-a.mpq ... patch-z.mpq).
    if (fs::exists(mpqDir + "/dbc.MPQ") || fs::exists(mpqDir + "/terrain.MPQ")) {
        for (char c = 'a'; c <= 'z'; ++c) {
            if (fs::exists(mpqDir + std::string("/patch-") + c + ".mpq") ||
                fs::exists(mpqDir + std::string("/Patch-") + static_cast<char>(std::toupper(c)) + ".mpq")) {
                return "turtle";
            }
        }
        return "classic";
    }
    return "";
}

std::string Extractor::detectLocale(const std::string& mpqDir) {
    for (const auto& loc : kKnownLocales) {
        if (fs::is_directory(mpqDir + "/" + loc))
            return loc;
    }
    return "";
}

// Discover archive files with expansion-specific and locale-aware loading
static std::vector<ArchiveDesc> discoverArchives(const std::string& mpqDir,
                                                  const std::string& expansion,
                                                  const std::string& locale) {
    std::vector<ArchiveDesc> result;

    auto tryAdd = [&](const std::string& name, int prio) {
        std::string fullPath = mpqDir + "/" + name;
        if (fs::exists(fullPath)) {
            result.push_back({fullPath, prio});
        }
    };

    if (expansion == "classic" || expansion == "turtle") {
        // Vanilla-era base archives (also used by Turtle WoW clients)
        tryAdd("base.MPQ", 90);
        tryAdd("base.mpq", 90);
        tryAdd("backup.MPQ", 95);
        tryAdd("backup.mpq", 95);
        tryAdd("dbc.MPQ", 100);
        tryAdd("dbc.mpq", 100);
        tryAdd("fonts.MPQ", 100);
        tryAdd("fonts.mpq", 100);
        tryAdd("interface.MPQ", 100);
        tryAdd("interface.mpq", 100);
        tryAdd("misc.MPQ", 100);
        tryAdd("misc.mpq", 100);
        tryAdd("model.MPQ", 100);
        tryAdd("model.mpq", 100);
        tryAdd("sound.MPQ", 100);
        tryAdd("sound.mpq", 100);
        tryAdd("speech.MPQ", 100);
        tryAdd("speech.mpq", 100);
        tryAdd("terrain.MPQ", 100);
        tryAdd("terrain.mpq", 100);
        tryAdd("texture.MPQ", 100);
        tryAdd("texture.mpq", 100);
        tryAdd("wmo.MPQ", 100);
        tryAdd("wmo.mpq", 100);

        // Patches
        tryAdd("patch.MPQ", 150);
        tryAdd("patch.mpq", 150);
        for (int i = 1; i <= 9; ++i) {
            tryAdd("patch-" + std::to_string(i) + ".MPQ", 160 + (i * 10));
            tryAdd("patch-" + std::to_string(i) + ".mpq", 160 + (i * 10));
        }
        // Turtle WoW uses letter patch MPQs (patch-a.mpq ... patch-z.mpq).
        for (char c = 'a'; c <= 'z'; ++c) {
            tryAdd(std::string("patch-") + c + ".mpq", 800 + (c - 'a'));
            tryAdd(std::string("Patch-") + static_cast<char>(std::toupper(c)) + ".mpq", 900 + (c - 'a'));
        }

        // Locale
        if (!locale.empty()) {
            tryAdd(locale + "/base-" + locale + ".MPQ", 230);
            tryAdd(locale + "/speech-" + locale + ".MPQ", 240);
            tryAdd(locale + "/locale-" + locale + ".MPQ", 250);
            tryAdd(locale + "/patch-" + locale + ".MPQ", 450);
        }
    } else if (expansion == "tbc") {
        // TBC 2.4.x base archives
        tryAdd("common.MPQ", 100);
        tryAdd("common-2.MPQ", 100);
        tryAdd("expansion.MPQ", 100);

        // Patches
        tryAdd("patch.MPQ", 150);
        tryAdd("patch-2.MPQ", 200);
        tryAdd("patch-3.MPQ", 300);
        tryAdd("patch-4.MPQ", 400);
        tryAdd("patch-5.MPQ", 500);

        // Letter patches
        for (char c = 'a'; c <= 'z'; ++c) {
            tryAdd(std::string("patch-") + c + ".mpq", 800 + (c - 'a'));
            tryAdd(std::string("Patch-") + static_cast<char>(std::toupper(c)) + ".mpq", 900 + (c - 'a'));
        }

        // Locale
        if (!locale.empty()) {
            tryAdd(locale + "/backup-" + locale + ".MPQ", 225);
            tryAdd(locale + "/base-" + locale + ".MPQ", 230);
            tryAdd(locale + "/speech-" + locale + ".MPQ", 240);
            tryAdd(locale + "/expansion-speech-" + locale + ".MPQ", 245);
            tryAdd(locale + "/expansion-locale-" + locale + ".MPQ", 246);
            tryAdd(locale + "/locale-" + locale + ".MPQ", 250);
            tryAdd(locale + "/patch-" + locale + ".MPQ", 450);
            tryAdd(locale + "/patch-" + locale + "-2.MPQ", 460);
            tryAdd(locale + "/patch-" + locale + "-3.MPQ", 470);
        }
    } else {
        // WotLK 3.3.5a (default)
        tryAdd("common.MPQ", 100);
        tryAdd("common-2.MPQ", 100);
        tryAdd("expansion.MPQ", 100);
        tryAdd("lichking.MPQ", 100);

        // Patches
        tryAdd("patch.MPQ", 150);
        tryAdd("patch-2.MPQ", 200);
        tryAdd("patch-3.MPQ", 300);
        tryAdd("patch-4.MPQ", 400);
        tryAdd("patch-5.MPQ", 500);

        // Letter patches
        for (char c = 'a'; c <= 'z'; ++c) {
            tryAdd(std::string("patch-") + c + ".mpq", 800 + (c - 'a'));
            tryAdd(std::string("Patch-") + static_cast<char>(std::toupper(c)) + ".mpq", 900 + (c - 'a'));
        }

        // Locale
        if (!locale.empty()) {
            tryAdd(locale + "/backup-" + locale + ".MPQ", 225);
            tryAdd(locale + "/base-" + locale + ".MPQ", 230);
            tryAdd(locale + "/speech-" + locale + ".MPQ", 240);
            tryAdd(locale + "/expansion-speech-" + locale + ".MPQ", 245);
            tryAdd(locale + "/expansion-locale-" + locale + ".MPQ", 246);
            tryAdd(locale + "/lichking-speech-" + locale + ".MPQ", 248);
            tryAdd(locale + "/lichking-locale-" + locale + ".MPQ", 249);
            tryAdd(locale + "/locale-" + locale + ".MPQ", 250);
            tryAdd(locale + "/patch-" + locale + ".MPQ", 450);
            tryAdd(locale + "/patch-" + locale + "-2.MPQ", 460);
            tryAdd(locale + "/patch-" + locale + "-3.MPQ", 470);
        }
    }

    // Sort by priority so highest-priority archives are last
    // (we'll iterate highest-prio first when extracting)
    std::sort(result.begin(), result.end(),
              [](const ArchiveDesc& a, const ArchiveDesc& b) { return a.priority < b.priority; });

    return result;
}

bool Extractor::enumerateFiles(const Options& opts,
                               std::vector<std::string>& outFiles) {
    // Open all archives, enumerate files from highest priority to lowest.
    // Use a set to deduplicate (highest-priority version wins).
    auto archives = discoverArchives(opts.mpqDir, opts.expansion, opts.locale);
    if (archives.empty()) {
        std::cerr << "No MPQ archives found in: " << opts.mpqDir << "\n";
        return false;
    }

    std::cout << "Found " << archives.size() << " MPQ archives\n";

    const auto wantedDbcs = buildWantedDbcSet(opts);

    // Enumerate from highest priority first so first-seen files win
    std::set<std::string> seenNormalized;
    std::vector<std::pair<std::string, std::string>> fileList; // (original name, archive path)

    for (auto it = archives.rbegin(); it != archives.rend(); ++it) {
        HANDLE hMpq = nullptr;
        if (!SFileOpenArchive(it->path.c_str(), 0, 0, &hMpq)) {
            std::cerr << "  Failed to open: " << it->path << "\n";
            continue;
        }

        if (opts.verbose) {
            std::cout << "  Scanning: " << it->path << " (priority " << it->priority << ")\n";
        }

        SFILE_FIND_DATA findData;
        HANDLE hFind = SFileFindFirstFile(hMpq, "*", &findData, nullptr);
        if (hFind) {
            do {
                std::string fileName = findData.cFileName;
                // Skip internal listfile/attributes
                if (fileName == "(listfile)" || fileName == "(attributes)" ||
                    fileName == "(signature)" || fileName == "(patch_metadata)") {
                    continue;
                }

                if (shouldSkipFile(opts, fileName)) {
                    continue;
                }

                // Verify file actually exists in this archive's hash table
                // (listfiles can reference files from other archives)
                if (!SFileHasFile(hMpq, fileName.c_str())) {
                    continue;
                }

                std::string norm = normalizeWowPath(fileName);
                if (opts.onlyUsedDbcs && !wantedDbcs.empty() && !wantedDbcs.contains(norm)) {
                    continue;
                }
                if (seenNormalized.insert(norm).second) {
                    // First time seeing this file — this is the highest-priority version
                    outFiles.push_back(fileName);
                }
            } while (SFileFindNextFile(hFind, &findData));
            SFileFindClose(hFind);
        }

        SFileCloseArchive(hMpq);
    }

    std::cout << "Enumerated " << outFiles.size() << " unique files\n";
    return true;
}

bool Extractor::run(const Options& opts) {
    auto startTime = std::chrono::steady_clock::now();

    const std::string effectiveOutputDir = opts.outputDir;

    // Enumerate all unique files across all archives
    std::vector<std::string> files;
    if (!enumerateFiles(opts, files)) {
        return false;
    }

    // Delta extraction: filter out files that already exist in the reference manifest
    if (!opts.referenceManifest.empty()) {
        auto refKeys = loadManifestKeys(opts.referenceManifest);
        if (refKeys.empty()) {
            std::cerr << "Warning: reference manifest is empty or failed to load\n";
        } else {
            size_t before = files.size();
            files.erase(std::remove_if(files.begin(), files.end(),
                [&refKeys](const std::string& wowPath) {
                    return refKeys.count(normalizeWowPath(wowPath)) > 0;
                }), files.end());
            std::cout << "Delta filter: " << before << " -> " << files.size()
                      << " files (" << (before - files.size()) << " already in reference)\n";
        }
    }

    if (files.empty()) {
        std::cerr << "No files to extract\n";
        return false;
    }

    // Create output directory
    std::error_code ec;
    fs::create_directories(effectiveOutputDir, ec);
    if (ec) {
        std::cerr << "Failed to create output directory: " << ec.message() << "\n";
        return false;
    }

    auto archives = discoverArchives(opts.mpqDir, opts.expansion, opts.locale);

    // Determine thread count
    int numThreads = opts.threads;
    if (numThreads <= 0) {
        numThreads = static_cast<int>(std::thread::hardware_concurrency());
        if (numThreads <= 0) numThreads = 4;
    }

    Stats stats;
    std::mutex manifestMutex;
    std::vector<ManifestWriter::FileEntry> manifestEntries;

    // Partition files across threads
    std::atomic<size_t> fileIndex{0};
    size_t totalFiles = files.size();

    // Open archives ONCE in main thread — StormLib has global state that is not
    // thread-safe even with separate handles, so we serialize all MPQ reads.
    struct SharedArchive {
        HANDLE handle;
        int priority;
        std::string path;
    };
    std::vector<SharedArchive> sharedHandles;
    for (const auto& ad : archives) {
        HANDLE h = nullptr;
        if (SFileOpenArchive(ad.path.c_str(), 0, 0, &h)) {
            sharedHandles.push_back({h, ad.priority, ad.path});
        } else {
            std::cerr << "  Failed to open archive: " << ad.path << "\n";
        }
    }
    if (sharedHandles.empty()) {
        std::cerr << "Failed to open any archives for extraction\n";
        return false;
    }
    if (sharedHandles.size() < archives.size()) {
        std::cerr << "  Opened " << sharedHandles.size()
                  << "/" << archives.size() << " archives\n";
    }

    // Mutex protecting all StormLib calls (open/read/close are not thread-safe)
    std::mutex mpqMutex;

    auto workerFn = [&]() {
        int failLogCount = 0;

        while (true) {
            size_t idx = fileIndex.fetch_add(1);
            if (idx >= totalFiles) break;

            const std::string& wowPath = files[idx];
            std::string normalized = normalizeWowPath(wowPath);

            // Map to new filesystem path
            std::string mappedPath = PathMapper::mapPath(wowPath);
            std::string fullOutputPath = effectiveOutputDir + "/" + mappedPath;

            // Read file data from MPQ under lock
            std::vector<uint8_t> data;
            {
                std::lock_guard<std::mutex> lock(mpqMutex);

                // Search archives in reverse priority order (highest priority first)
                HANDLE hFile = nullptr;
                for (auto it = sharedHandles.rbegin(); it != sharedHandles.rend(); ++it) {
                    if (SFileOpenFileEx(it->handle, wowPath.c_str(), 0, &hFile)) {
                        break;
                    }
                    hFile = nullptr;
                }
                if (!hFile) {
                    stats.filesFailed++;
                    if (failLogCount < 5) {
                        failLogCount++;
                        std::cerr << "  FAILED open: " << wowPath
                                  << " (tried " << sharedHandles.size() << " archives)\n";
                    }
                    continue;
                }

                DWORD fileSize = SFileGetFileSize(hFile, nullptr);
                if (fileSize == SFILE_INVALID_SIZE || fileSize == 0) {
                    SFileCloseFile(hFile);
                    stats.filesSkipped++;
                    continue;
                }

                data.resize(fileSize);
                DWORD bytesRead = 0;
                if (!SFileReadFile(hFile, data.data(), fileSize, &bytesRead, nullptr)) {
                    SFileCloseFile(hFile);
                    stats.filesFailed++;
                    if (failLogCount < 5) {
                        failLogCount++;
                        std::cerr << "  FAILED read: " << wowPath
                                  << " (size=" << fileSize << ")\n";
                    }
                    continue;
                }
                SFileCloseFile(hFile);
                data.resize(bytesRead);
            }
            // Lock released — CRC computation and disk write happen in parallel

            // Compute CRC32
            uint32_t crc = ManifestWriter::computeCRC32(data.data(), data.size());

            // Create output directory and write file
            fs::path outPath(fullOutputPath);
            fs::create_directories(outPath.parent_path(), ec);

            std::ofstream out(fullOutputPath, std::ios::binary);
            if (!out.is_open()) {
                stats.filesFailed++;
                if (failLogCount < 5) {
                    failLogCount++;
                    std::lock_guard<std::mutex> lock(manifestMutex);
                    std::cerr << "  FAILED write: " << fullOutputPath << "\n";
                }
                continue;
            }
            out.write(reinterpret_cast<const char*>(data.data()), data.size());
            out.close();

            // Add manifest entry
            ManifestWriter::FileEntry entry;
            entry.wowPath = normalized;
            entry.filesystemPath = mappedPath;
            entry.size = data.size();
            entry.crc32 = crc;

            {
                std::lock_guard<std::mutex> lock(manifestMutex);
                manifestEntries.push_back(std::move(entry));
            }

            stats.filesExtracted++;
            stats.bytesExtracted += data.size();

            // Progress
            uint64_t done = stats.filesExtracted.load();
            if (done % 1000 == 0) {
                std::cout << "\r  Extracted " << done << " / " << totalFiles << " files..."
                          << std::flush;
            }
        }
    };

    std::cout << "Extracting " << totalFiles << " files using " << numThreads << " threads...\n";

    std::vector<std::thread> threads;
    for (int i = 0; i < numThreads; ++i) {
        threads.emplace_back(workerFn);
    }
    for (auto& t : threads) {
        t.join();
    }

    // Close archives (opened once in main thread)
    for (auto& sh : sharedHandles) {
        SFileCloseArchive(sh.handle);
    }

    auto extracted = stats.filesExtracted.load();
    auto failed = stats.filesFailed.load();
    auto skipped = stats.filesSkipped.load();
    std::cout << "\n  Extracted " << extracted << " files ("
              << stats.bytesExtracted.load() / (1024 * 1024) << " MB), "
              << skipped << " skipped, "
              << failed << " failed\n";

    // If most files failed, print a diagnostic hint
    if (failed > 0 && failed > extracted * 10) {
        std::cerr << "\nWARNING: " << failed << " out of " << totalFiles
                  << " files failed to extract.\n"
                  << "  This usually means worker threads could not open one or more MPQ archives.\n"
                  << "  Common causes:\n"
                  << "    - MPQ files on a network/external drive with access restrictions\n"
                  << "    - Another program (WoW client, antivirus) has the MPQ files locked\n"
                  << "    - Too many threads for the OS file-handle limit (try --threads 1)\n"
                  << "  Re-run with --verbose for detailed diagnostics.\n";
    }

    // Merge with existing manifest so partial extractions don't nuke prior entries
    std::string manifestPath = effectiveOutputDir + "/manifest.json";
    if (fs::exists(manifestPath)) {
        auto existing = loadManifestEntries(manifestPath);
        if (!existing.empty()) {
            // New entries override existing ones with same key
            for (auto& entry : manifestEntries) {
                existing[entry.wowPath] = entry;
            }
            // Rebuild manifestEntries from merged map
            manifestEntries.clear();
            manifestEntries.reserve(existing.size());
            for (auto& [k, v] : existing) {
                manifestEntries.push_back(std::move(v));
            }
            std::cout << "Merged with existing manifest (" << existing.size() << " total entries)\n";
        }
    }

    // Sort manifest entries for deterministic output
    std::sort(manifestEntries.begin(), manifestEntries.end(),
              [](const ManifestWriter::FileEntry& a, const ManifestWriter::FileEntry& b) {
                  return a.wowPath < b.wowPath;
              });

    // basePath is "." since manifest sits inside the output directory
    if (!ManifestWriter::write(manifestPath, ".", manifestEntries)) {
        std::cerr << "Failed to write manifest: " << manifestPath << "\n";
        return false;
    }

    std::cout << "Wrote manifest: " << manifestPath << " (" << manifestEntries.size() << " entries)\n";

    // Verification pass
    if (opts.verify) {
        std::cout << "Verifying extracted files...\n";
        uint64_t verified = 0, verifyFailed = 0;
        for (const auto& entry : manifestEntries) {
            std::string fsPath = effectiveOutputDir + "/" + entry.filesystemPath;
            std::ifstream f(fsPath, std::ios::binary | std::ios::ate);
            if (!f.is_open()) {
                std::cerr << "  MISSING: " << fsPath << "\n";
                verifyFailed++;
                continue;
            }

            auto size = f.tellg();
            if (static_cast<uint64_t>(size) != entry.size) {
                std::cerr << "  SIZE MISMATCH: " << fsPath << " (expected "
                          << entry.size << ", got " << size << ")\n";
                verifyFailed++;
                continue;
            }

            f.seekg(0);
            std::vector<uint8_t> data(static_cast<size_t>(size));
            f.read(reinterpret_cast<char*>(data.data()), size);

            uint32_t crc = ManifestWriter::computeCRC32(data.data(), data.size());
            if (crc != entry.crc32) {
                std::cerr << "  CRC MISMATCH: " << fsPath << "\n";
                verifyFailed++;
                continue;
            }

            verified++;
        }
        std::cout << "Verified " << verified << " files";
        if (verifyFailed > 0) {
            std::cout << " (" << verifyFailed << " FAILED)";
        }
        std::cout << "\n";
    }

    auto elapsed = std::chrono::steady_clock::now() - startTime;
    auto secs = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();

    if (opts.generateDbcCsv) {
        std::cout << "Converting selected DBCs to CSV for committing...\n";
        const std::string dbcDir = effectiveOutputDir + "/db";
        const std::string csvExpansion = opts.expansion;
        const std::string csvDir = !opts.dbcCsvOutputDir.empty()
            ? opts.dbcCsvOutputDir
            : (opts.outputDir + "/expansions/" + csvExpansion + "/db");

        uint32_t ok = 0, fail = 0, missing = 0;
        for (const auto& base : getUsedDbcNamesForExpansion(opts.expansion)) {
            const std::string in = dbcDir + "/" + base + ".dbc";
            const std::string out = csvDir + "/" + base + ".csv";
            if (!fs::exists(in)) {
                std::cerr << "  Missing extracted DBC: " << in << "\n";
                missing++;
                continue;
            }
            if (!convertDbcToCsv(in, out)) {
                fail++;
            } else {
                ok++;
            }
        }

        std::cout << "DBC CSV conversion: " << ok << " ok";
        if (missing) std::cout << ", " << missing << " missing";
        if (fail) std::cout << ", " << fail << " failed";
        std::cout << "\n";

        if (fail > 0) {
            std::cerr << "DBC CSV conversion failed for some files\n";
            return false;
        }
    }

    // Cache WoW.exe for Warden MEM_CHECK responses
    {
        const char* exeNames[] = { "WoW.exe", "TurtleWoW.exe", "Wow.exe" };
        std::vector<std::string> searchDirs = {
            fs::path(opts.mpqDir).parent_path().string(),  // WoW.exe is typically next to Data/
            opts.mpqDir                                      // Some layouts put it inside Data/
        };
        for (const auto& dir : searchDirs) {
            bool found = false;
            for (const char* name : exeNames) {
                auto src = fs::path(dir) / name;
                if (fs::exists(src)) {
                    auto dstDir = fs::path(opts.outputDir) / "misc";
                    fs::create_directories(dstDir);
                    auto dst = dstDir / "WoW.exe";
                    fs::copy_file(src, dst, fs::copy_options::overwrite_existing);
                    std::cout << "Cached " << name << " -> " << dst.string() << "\n";
                    found = true;
                    break;
                }
            }
            if (found) break;
        }
    }

    std::cout << "Done in " << secs / 60 << "m " << secs % 60 << "s\n";

    return true;
}

} // namespace tools
} // namespace wowee
