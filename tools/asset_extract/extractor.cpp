#include "extractor.hpp"
#include "path_mapper.hpp"
#include "manifest_writer.hpp"
#include "open_format_emitter.hpp"

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

// Precompute the set of valid string-boundary offsets in the string block.
// An offset is a valid boundary if it is 0 or immediately follows a null byte.
// This prevents small integer values (e.g. RaceID=1, 2, 3) from being falsely
// detected as string offsets just because they land in the middle of a longer
// string that starts at a lower offset.
static std::set<uint32_t> computeStringBoundaries(const std::vector<uint8_t>& stringBlock) {
    std::set<uint32_t> boundaries;
    if (stringBlock.empty()) return boundaries;
    boundaries.insert(0);
    for (size_t i = 0; i + 1 < stringBlock.size(); ++i) {
        if (stringBlock[i] == 0) {
            boundaries.insert(static_cast<uint32_t>(i + 1));
        }
    }
    return boundaries;
}

static bool isValidStringOffset(const std::vector<uint8_t>& stringBlock,
                                const std::set<uint32_t>& boundaries,
                                uint32_t offset) {
    if (offset >= stringBlock.size()) return false;
    // Must start at a string boundary (offset 0 or right after a null byte).
    if (!boundaries.count(offset)) return false;
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

    auto boundaries = computeStringBoundaries(stringBlock);

    // Field 0 is always the numeric record ID - skip it.
    for (uint32_t col = 1; col < fieldCount; ++col) {
        bool allZeroOrValid = true;
        bool hasNonZero = false;
        std::set<std::string> distinctStrings;

        for (uint32_t row = 0; row < recordCount; ++row) {
            uint32_t val = dbc.getUInt32(row, col);
            if (val == 0) continue;
            hasNonZero = true;
            if (!isValidStringOffset(stringBlock, boundaries, val)) {
                allZeroOrValid = false;
                break;
            }
            const char* s = reinterpret_cast<const char*>(stringBlock.data() + val);
            if (*s != '\0') {
                distinctStrings.insert(std::string(s, strnlen(s, 256)));
            }
        }

        if (allZeroOrValid && hasNonZero && distinctStrings.size() >= 2) {
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
        "BarberShopStyle",
        "gtBarberShopCostBase",
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

static bool hasFileCaseInsensitive(const fs::path& directory,
                                   const std::string& expectedName) {
    std::error_code ec;
    if (!fs::is_directory(directory, ec)) return false;
    const std::string expected = toLowerStr(expectedName);
    for (const auto& entry : fs::directory_iterator(directory, ec)) {
        if (ec) break;
        if (entry.is_regular_file() &&
            toLowerStr(entry.path().filename().string()) == expected) {
            return true;
        }
    }
    return false;
}

std::string Extractor::detectExpansion(const std::string& mpqDir) {
    if (hasFileCaseInsensitive(mpqDir, "lichking.mpq"))
        return "wotlk";
    if (hasFileCaseInsensitive(mpqDir, "expansion.mpq"))
        return "tbc";
    // Turtle WoW uses vanilla-era base MPQs, so detect its executable and its
    // custom high-numbered/letter patch archives before falling back to Classic.
    if (hasFileCaseInsensitive(mpqDir, "dbc.mpq") ||
        hasFileCaseInsensitive(mpqDir, "terrain.mpq")) {
        const fs::path clientRoot = fs::path(mpqDir).parent_path();
        if (hasFileCaseInsensitive(clientRoot, "TurtleWoW.exe")) return "turtle";
        for (int patch = 8; patch <= 9; ++patch) {
            if (hasFileCaseInsensitive(mpqDir,
                                       "patch-" + std::to_string(patch) + ".mpq")) {
                return "turtle";
            }
        }
        for (char c = 'a'; c <= 'z'; ++c) {
            if (hasFileCaseInsensitive(mpqDir,
                                       std::string("patch-") + c + ".mpq")) {
                return "turtle";
            }
        }
        return "classic";
    }
    return "";
}

static std::string findCaseInsensitiveDirectory(const std::string& parentDir,
                                                   const std::string& directoryName) {
    if (!fs::exists(parentDir) || !fs::is_directory(parentDir)) return "";
    std::string lowerDirectoryName = toLowerStr(directoryName);
    for (const auto& entry : fs::directory_iterator(parentDir)) {
        if (!entry.is_directory()) continue;
        std::string name = entry.path().filename().string();
        if (toLowerStr(name) == lowerDirectoryName) {
            return name;
        }
    }
    return "";
}

std::string Extractor::detectLocale(const std::string& mpqDir) {
    if (!fs::exists(mpqDir) || !fs::is_directory(mpqDir)) return "";
    for (const auto& entry : fs::directory_iterator(mpqDir)) {
        if (!entry.is_directory()) continue;
        std::string name = entry.path().filename().string();
        std::string lower = toLowerStr(name);
        for (const auto& loc : kKnownLocales) {
            if (toLowerStr(loc) == lower) {
                return name;
            }
        }
    }
    return "";
}

static std::unordered_map<std::string, std::string> buildCaseMap(const std::string& dir) {
    std::unordered_map<std::string, std::string> map;
    if (!fs::exists(dir) || !fs::is_directory(dir)) return map;
    for (const auto& entry : fs::directory_iterator(dir)) {
        if (entry.is_regular_file()) {
            std::string filename = entry.path().filename().string();
            if (filename.rfind("._", 0) == 0) {
                continue;
            }
            std::string ext = toLowerStr(entry.path().extension().string());
            if (ext == ".mpq") {
                std::string lower = toLowerStr(filename);
                map[lower] = filename;
            }
        }
    }
    return map;
}

// Discover archive files with expansion-specific and locale-aware loading
static std::vector<std::string> discoverArchives(const std::string& mpqDir,
                                                  const std::string& expansion,
                                                  const std::string& locale) {
    std::vector<std::string> result;

    auto caseMap = buildCaseMap(mpqDir);
    std::string lowerLocale = toLowerStr(locale);
    if (!locale.empty()) {
        std::string actualLocaleDir = findCaseInsensitiveDirectory(mpqDir, locale);
        if (actualLocaleDir.empty()) {
            actualLocaleDir = locale;
        }
        fs::path localeDirPath = fs::path(mpqDir) / actualLocaleDir;
        std::string localeDir = localeDirPath.string();
        auto localeMap = buildCaseMap(localeDir);
        for (auto& [name, realName] : localeMap) {
            fs::path fullPath = fs::path(actualLocaleDir) / realName;
            caseMap[lowerLocale + "/" + name] = fullPath.string();
        }
    }

    std::vector<std::string> baseSequence;
    std::vector<std::string> localeSequence;

    if (expansion == "classic" || expansion == "turtle") {
        baseSequence = {
            "base.mpq", "backup.mpq", "dbc.mpq", "fonts.mpq",
            "interface.mpq", "misc.mpq", "model.mpq", "sound.mpq",
            "speech.mpq", "terrain.mpq", "texture.mpq", "wmo.mpq"
        };
    } else if (expansion == "tbc") {
        baseSequence = { "common.mpq", "expansion.mpq" };
        if (!locale.empty()) {
            localeSequence = {
                lowerLocale + "/backup-" + lowerLocale + ".mpq",
                lowerLocale + "/base-" + lowerLocale + ".mpq",
                lowerLocale + "/locale-" + lowerLocale + ".mpq",
                lowerLocale + "/speech-" + lowerLocale + ".mpq",
                lowerLocale + "/expansion-locale-" + lowerLocale + ".mpq",
                lowerLocale + "/expansion-speech-" + lowerLocale + ".mpq",
            };
        }
    } else {
        baseSequence = { "common.mpq", "common-2.mpq", "expansion.mpq", "lichking.mpq" };
        if (!locale.empty()) {
            localeSequence = {
                lowerLocale + "/backup-" + lowerLocale + ".mpq",
                lowerLocale + "/base-" + lowerLocale + ".mpq",
                lowerLocale + "/locale-" + lowerLocale + ".mpq",
                lowerLocale + "/speech-" + lowerLocale + ".mpq",
                lowerLocale + "/expansion-locale-" + lowerLocale + ".mpq",
                lowerLocale + "/expansion-speech-" + lowerLocale + ".mpq",
                lowerLocale + "/lichking-locale-" + lowerLocale + ".mpq",
                lowerLocale + "/lichking-speech-" + lowerLocale + ".mpq",
            };
        }
    }

    std::vector<std::string> sequence;
    for (const auto& name : baseSequence) {
        sequence.push_back(name);
    }
    for (const auto& name : localeSequence) {
        sequence.push_back(name);
    }

    // Interleave patches: base patch then locale patch for each tier
    std::vector<std::string> patchSuffixes = {""};
    for (int i = 2; i <= 9; ++i) {
        patchSuffixes.push_back(std::string("-") + std::to_string(i));
    }
    for (char c = 'a'; c <= 'z'; ++c) {
        patchSuffixes.push_back(std::string("-") + c);
    }

    for (const auto& suffix : patchSuffixes) {
        sequence.push_back("patch" + suffix + ".mpq");
        if (!locale.empty()) {
            sequence.push_back(lowerLocale + "/patch-" + lowerLocale + suffix + ".mpq");
        }
    }

    auto addIfPresent = [&](const std::string& expected) {
        auto it = caseMap.find(toLowerStr(expected));
        if (it != caseMap.end()) {
            fs::path fullPath = fs::path(mpqDir) / it->second;
            result.push_back(fullPath.string());
        }
    };

    for (const auto& entry : sequence) {
        addIfPresent(entry);
    }

    return result;
}

// Extract the (listfile) from an MPQ archive into a set of filenames
static void extractInternalListfile(HANDLE hMpq, std::set<std::string>& out) {
    HANDLE hFile = nullptr;
    if (!SFileOpenFileEx(hMpq, "(listfile)", 0, &hFile)) return;

    DWORD size = SFileGetFileSize(hFile, nullptr);
    if (size == SFILE_INVALID_SIZE || size == 0) {
        SFileCloseFile(hFile);
        return;
    }

    std::vector<char> buf(size);
    DWORD bytesRead = 0;
    if (!SFileReadFile(hFile, buf.data(), size, &bytesRead, nullptr)) {
        SFileCloseFile(hFile);
        return;
    }
    SFileCloseFile(hFile);

    // Parse newline/CR-delimited entries
    std::string entry;
    for (DWORD i = 0; i < bytesRead; ++i) {
        if (buf[i] == '\n' || buf[i] == '\r') {
            if (!entry.empty()) {
                out.insert(std::move(entry));
                entry.clear();
            }
        } else {
            entry += buf[i];
        }
    }
    if (!entry.empty()) out.insert(std::move(entry));
}

bool Extractor::enumerateFiles(const Options& opts,
                               std::vector<std::string>& outFiles) {
    auto archives = discoverArchives(opts.mpqDir, opts.expansion, opts.locale);
    if (archives.empty()) {
        std::cerr << "No MPQ archives found in: " << opts.mpqDir << "\n";
        return false;
    }

    std::cout << "Found " << archives.size() << " MPQ archives\n";

    const bool haveExternalListFile = !opts.listFile.empty();
    if (haveExternalListFile) {
        std::cout << "  Using external listfile: " << opts.listFile << "\n";
    }

    const auto wantedDbcs = buildWantedDbcSet(opts);
    std::set<std::string> seenNormalized;

    // Enumerate from highest priority first so first-seen files win
    for (auto it = archives.rbegin(); it != archives.rend(); ++it) {
        HANDLE hMpq = nullptr;
        if (!SFileOpenArchive(it->c_str(), 0, 0, &hMpq)) {
            std::cerr << "  Failed to open: " << *it << "\n";
            continue;
        }

        // Inject external listfile into archive's in-memory name table.
        // SFileAddListFile reads the file and hashes names against the
        // archive's hash table.
        if (haveExternalListFile) {
            SFileAddListFile(hMpq, opts.listFile.c_str());
        }

        if (opts.verbose) {
            std::cout << "  Scanning: " << *it << "\n";
        }

        SFILE_FIND_DATA findData;
        HANDLE hFind = SFileFindFirstFile(hMpq, "*", &findData, nullptr);
        if (hFind) {
            do {
                std::string fileName = findData.cFileName;
                if (fileName == "(listfile)" || fileName == "(attributes)" ||
                    fileName == "(signature)" || fileName == "(patch_metadata)") {
                    continue;
                }

                if (shouldSkipFile(opts, fileName)) continue;

                if (!SFileHasFile(hMpq, fileName.c_str())) continue;

                std::string norm = normalizeWowPath(fileName);
                if (opts.onlyUsedDbcs && !wantedDbcs.empty() && !wantedDbcs.contains(norm)) {
                    continue;
                }
                if (seenNormalized.insert(norm).second) {
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

    const std::string effectiveOutputDir = opts.expansionSubdir
        ? (fs::path(opts.outputDir) / "expansions" / opts.expansion).string()
        : opts.outputDir;

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

    // Open archives ONCE in main thread - StormLib has global state that is not
    // thread-safe even with separate handles, so we serialize all MPQ reads.
    struct SharedArchive {
        HANDLE handle;
        std::string path;
    };
    std::vector<SharedArchive> sharedHandles;
    for (const auto& path : archives) {
        HANDLE h = nullptr;
        if (SFileOpenArchive(path.c_str(), 0, 0, &h)) {
            if (!opts.listFile.empty()) {
                SFileAddListFile(h, opts.listFile.c_str());
            }
            sharedHandles.push_back({h, path});
        } else {
            std::cerr << "  Failed to open archive: " << path << "\n";
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
            fs::path fullOutputPath = fs::path(effectiveOutputDir) / mappedPath;

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
            // Lock released - CRC computation and disk write happen in parallel

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
    fs::path manifestPath = fs::path(effectiveOutputDir) / "manifest.json";
    if (fs::exists(manifestPath)) {
        auto existing = loadManifestEntries(manifestPath.string());
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
    if (!ManifestWriter::write(manifestPath.string(), ".", opts.expansion, manifestEntries)) {
        std::cerr << "Failed to write manifest: " << manifestPath << "\n";
        return false;
    }

    std::cout << "Wrote manifest: " << manifestPath << " (" << manifestEntries.size() << " entries)\n";

    // Verification pass
    if (opts.verify) {
        std::cout << "Verifying extracted files...\n";
        uint64_t verified = 0, verifyFailed = 0;
        for (const auto& entry : manifestEntries) {
            fs::path fsPath = fs::path(effectiveOutputDir) / entry.filesystemPath;
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
        // Where extraction actually put them. This read <out>/db, which is
        // where the CSVs are written to, not where the DBCs came out - so
        // every name reported "Missing extracted DBC" and the conversion did
        // nothing at all. PathMapper lowercases every output path, so the file
        // is dbfilesclient/spell.dbc; the mixed-case name is tried after it for
        // a tree some other tool extracted. Reported in #132.
        const std::string dbcDir = effectiveOutputDir + "/dbfilesclient";
        const std::string csvExpansion = opts.expansion;
        const std::string csvDir = !opts.dbcCsvOutputDir.empty()
            ? opts.dbcCsvOutputDir
            : (opts.outputDir + "/expansions/" + csvExpansion + "/db");

        uint32_t ok = 0, fail = 0, missing = 0;
        for (const auto& base : getUsedDbcNamesForExpansion(opts.expansion)) {
            std::string lower = base;
            std::transform(lower.begin(), lower.end(), lower.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            std::string in = dbcDir + "/" + lower + ".dbc";
            if (!fs::exists(in)) in = dbcDir + "/" + base + ".dbc";
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

    // Open-format emission: walk the extracted tree and write
    // wowee-format side-files (PNG / JSON DBC) next to each .blp/.dbc.
    // Originals are left untouched so private servers continue to work.
    if (opts.emitPng || opts.emitJsonDbc || opts.emitWom || opts.emitWob ||
        opts.emitTerrain) {
        std::cout << "Emitting wowee open-format side-files...\n";
        OpenFormatStats ofs;
        emitOpenFormats(effectiveOutputDir, opts.emitPng, opts.emitJsonDbc,
                        opts.emitWom, opts.emitWob, opts.emitTerrain, ofs);
        if (opts.emitPng) {
            std::cout << "  PNG (BLP→PNG)     : " << ofs.pngOk << " ok";
            if (ofs.pngFail) std::cout << ", " << ofs.pngFail << " failed";
            std::cout << "\n";
        }
        if (opts.emitJsonDbc) {
            std::cout << "  JSON (DBC→JSON)   : " << ofs.jsonDbcOk << " ok";
            if (ofs.jsonDbcFail) std::cout << ", " << ofs.jsonDbcFail << " failed";
            std::cout << "\n";
        }
        if (opts.emitWom) {
            std::cout << "  WOM (M2→WOM)      : " << ofs.womOk << " ok";
            if (ofs.womFail) std::cout << ", " << ofs.womFail << " failed";
            std::cout << "\n";
        }
        if (opts.emitWob) {
            std::cout << "  WOB (WMO→WOB)     : " << ofs.wobOk << " ok";
            if (ofs.wobFail) std::cout << ", " << ofs.wobFail << " failed";
            std::cout << "\n";
        }
        if (opts.emitTerrain) {
            std::cout << "  WHM/WOT/WOC (ADT) : " << ofs.whmOk << " ok";
            if (ofs.whmFail) std::cout << ", " << ofs.whmFail << " failed";
            std::cout << "\n";
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
                    // Keep each client's executable beside its isolated manifest.
                    // Warden integrity checks must never use a WoW.exe cached by a
                    // later extraction from another expansion.
                    auto dstDir = fs::path(effectiveOutputDir) / "misc";
                    fs::create_directories(dstDir);
                    auto dst = dstDir / name;
                    fs::copy_file(src, dst, fs::copy_options::overwrite_existing);
                    std::cout << "Cached " << name << " -> " << dst.string() << "\n";

                    // Legacy auth integrity hashes include companion DLLs. Keep
                    // them with the executable so Classic and Turtle servers can
                    // validate the exact distribution that was extracted.
                    for (const char* companion : {
                            "fmod.dll", "ijl15.dll", "dbghelp.dll", "unicows.dll",
                            "twloader.dll", "twdiscord.dll"}) {
                        const auto companionSrc = fs::path(dir) / companion;
                        if (!fs::exists(companionSrc)) continue;
                        const auto companionDst = dstDir / companion;
                        fs::copy_file(companionSrc, companionDst,
                                      fs::copy_options::overwrite_existing);
                        std::cout << "Cached " << companion << " -> "
                                  << companionDst.string() << "\n";
                    }
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
