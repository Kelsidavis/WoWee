#include "extractor.hpp"
#include "open_format_emitter.hpp"
#include <nlohmann/json.hpp>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>
#include <cstring>

#ifdef _WIN32
// Guarded: CMakeLists passes both of these to every translation unit under
// its Windows branch, and redefining one is a warning the build treats as an
// error. Kept rather than dropped so the include below is still narrow if
// this file is ever compiled outside that branch.
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <shellapi.h>

/// Open the asset manager window instead, when this was double-clicked.
///
/// Somebody who finds this in the install folder and runs it wants their
/// assets built, not a usage message - and Explorer's console closes with the
/// process, so they never see one anyway. The window beside it does the same
/// job and asks for the folders rather than expecting them as flags.
static bool startAssetManager() {
    wchar_t self[MAX_PATH];
    const DWORD length = GetModuleFileNameW(nullptr, self, MAX_PATH);
    if (length == 0 || length >= MAX_PATH) return false;

    const std::filesystem::path beside = std::filesystem::path(self).parent_path();
    const std::filesystem::path gui = beside / L"wowee_assets.exe";
    std::error_code ec;
    if (!std::filesystem::is_regular_file(gui, ec)) return false;

    // Over 32 is ShellExecute's success threshold; anything at or below it is
    // one of its error codes.
    const auto result = reinterpret_cast<INT_PTR>(ShellExecuteW(
        nullptr, L"open", gui.c_str(), nullptr, beside.c_str(), SW_SHOWNORMAL));
    return result > 32;
}
#endif

static void printUsage(const char* prog) {
    std::cout << "Usage: " << prog << " --mpq-dir <path> --output <path> [options]\n"
              << "\n"
              << "Extract WoW MPQ archives to organized loose files with manifest.\n"
              << "\n"
              << "Required:\n"
              << "  --mpq-dir <path>    Path to WoW Data directory containing MPQ files\n"
              << "  --output <path>     Output directory for extracted assets\n"
              << "\n"
              << "Options:\n"
              << "  --expansion <id>    Expansion: classic, turtle, tbc, wotlk, cata (default: auto-detect)\n"
              << "  --expansion-subdir Write into <output>/expansions/<id> so multiple clients\n"
              << "                      cannot overwrite one another\n"
              << "  --locale <id>       Locale: enUS, deDE, frFR, etc. (default: auto-detect)\n"
              << "  --only-used-dbcs    Extract only the DBCs wowee uses (no other assets)\n"
              << "  --skip-dbc          Do not extract DBFilesClient/*.dbc (visual assets only)\n"
              << "  --dbc-csv           Convert selected DBFilesClient/*.dbc to CSV under\n"
              << "                      <output>/expansions/<expansion>/db/*.csv (for committing)\n"
              << "  --listfile <path>   External listfile for MPQ file enumeration (auto-detected)\n"
              << "  --include <text>    Extract only paths containing this (repeatable); for\n"
              << "                      borrowing one zone's art instead of a whole client\n"
              << "  --reference-manifest <path>\n"
              << "                      Only extract files NOT in this manifest (delta extraction)\n"
              << "  --dbc-csv-out <dir> Write CSV DBCs into <dir> (overrides default output path)\n"
              << "  --emit-png          Emit foo.png next to every extracted foo.blp\n"
              << "  --emit-json-dbc     Emit foo.json next to every extracted foo.dbc\n"
              << "  --emit-wom          Emit foo.wom next to every extracted foo.m2 (+skin)\n"
              << "  --emit-wob          Emit foo.wob next to every extracted foo.wmo (+groups)\n"
              << "  --emit-terrain      Emit foo.whm + foo.wot + foo.woc next to every foo.adt\n"
              << "  --emit-open         Shortcut: enable every open-format emitter (png+json+wom+wob+terrain)\n"
              << "  --upgrade-extract <dir> [--json]\n"
              << "                      Standalone post-extract pass on an existing tree -\n"
              << "                      writes open-format sidecars without re-running MPQ extract\n"
              << "                      --json emits a structured summary instead of text\n"
              << "  --purge-proprietary <dir> [--json]\n"
              << "                      Walk tree and dry-run report which proprietary files have\n"
              << "                      an open-format sidecar; add --confirm-purge to actually delete\n"
              << "  --confirm-purge     Required to actually delete files in --purge-proprietary mode\n"
              << "  --json              Emit machine-readable summary in --upgrade-extract /\n"
              << "                      --purge-proprietary modes\n"
              << "  --verify            CRC32 verify all extracted files\n"
              << "  --threads <N>       Number of extraction threads (default: auto)\n"
              << "  --verbose           Verbose output\n"
              << "  --help              Show this help\n";
}

int main(int argc, char** argv) {
#ifdef _WIN32
    // Nothing on the command line means Explorer, not a shell. Hand the
    // person the window rather than a usage message their console closes
    // before they can read. Falls through to that message when the window is
    // not installed beside this.
    if (argc == 1 && startAssetManager()) return 0;
#endif
    wowee::tools::Extractor::Options opts;
    std::string expansion;
    std::string locale;
    // Standalone open-format emit mode: skip MPQ enumeration entirely
    // and just walk an existing extracted tree, writing sidecars in
    // place. Useful for upgrading an old extraction without re-running
    // it from MPQ. Triggered by --upgrade-extract <dir>.
    std::string upgradeDir;
    bool jsonOutput = false;  // --json after upgrade-extract

    // Purge proprietary files when their open-format sidecar is present
    // and at least as new. Dry-run by default; --confirm-purge actually
    // deletes. Lets users free disk after dual-format extraction when
    // they only need wowee runtime (no private server).
    std::string purgeDir;
    bool confirmPurge = false;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--mpq-dir") == 0 && i + 1 < argc) {
            opts.mpqDir = argv[++i];
        } else if (std::strcmp(argv[i], "--output") == 0 && i + 1 < argc) {
            opts.outputDir = argv[++i];
        } else if (std::strcmp(argv[i], "--expansion") == 0 && i + 1 < argc) {
            expansion = argv[++i];
        } else if (std::strcmp(argv[i], "--locale") == 0 && i + 1 < argc) {
            locale = argv[++i];
        } else if (std::strcmp(argv[i], "--expansion-subdir") == 0) {
            opts.expansionSubdir = true;
        } else if (std::strcmp(argv[i], "--threads") == 0 && i + 1 < argc) {
            opts.threads = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--only-used-dbcs") == 0) {
            opts.onlyUsedDbcs = true;
        } else if (std::strcmp(argv[i], "--skip-dbc") == 0) {
            opts.skipDbcExtraction = true;
        } else if (std::strcmp(argv[i], "--dbc-csv") == 0) {
            opts.generateDbcCsv = true;
        } else if (std::strcmp(argv[i], "--emit-png") == 0) {
            opts.emitPng = true;
        } else if (std::strcmp(argv[i], "--emit-json-dbc") == 0) {
            opts.emitJsonDbc = true;
        } else if (std::strcmp(argv[i], "--emit-wom") == 0) {
            opts.emitWom = true;
        } else if (std::strcmp(argv[i], "--emit-wob") == 0) {
            opts.emitWob = true;
        } else if (std::strcmp(argv[i], "--emit-terrain") == 0) {
            opts.emitTerrain = true;
        } else if (std::strcmp(argv[i], "--emit-open") == 0) {
            // Meta-flag: turn on every available open-format emitter.
            opts.emitPng = true;
            opts.emitJsonDbc = true;
            opts.emitWom = true;
            opts.emitWob = true;
            opts.emitTerrain = true;
        } else if (std::strcmp(argv[i], "--dbc-csv-out") == 0 && i + 1 < argc) {
            opts.dbcCsvOutputDir = argv[++i];
        } else if (std::strcmp(argv[i], "--listfile") == 0 && i + 1 < argc) {
            opts.listFile = argv[++i];
        } else if (std::strcmp(argv[i], "--include") == 0 && i + 1 < argc) {
            // Matched against the normalized path: lowercase, backslashes. Take
            // it however it was typed.
            std::string fragment = argv[++i];
            for (char& c : fragment) {
                c = (c == '/') ? '\\' : static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            }
            opts.includeSubstrings.push_back(fragment);
        } else if (std::strcmp(argv[i], "--reference-manifest") == 0 && i + 1 < argc) {
            opts.referenceManifest = argv[++i];
        } else if (std::strcmp(argv[i], "--verify") == 0) {
            opts.verify = true;
        } else if (std::strcmp(argv[i], "--verbose") == 0) {
            opts.verbose = true;
        } else if (std::strcmp(argv[i], "--purge-proprietary") == 0 && i + 1 < argc) {
            purgeDir = argv[++i];
        } else if (std::strcmp(argv[i], "--confirm-purge") == 0) {
            confirmPurge = true;
        } else if (std::strcmp(argv[i], "--json") == 0) {
            jsonOutput = true;
        } else if (std::strcmp(argv[i], "--upgrade-extract") == 0 && i + 1 < argc) {
            upgradeDir = argv[++i];
            // Implies --emit-open if no individual emit flag was set.
            if (!opts.emitPng && !opts.emitJsonDbc && !opts.emitWom &&
                !opts.emitWob && !opts.emitTerrain) {
                opts.emitPng = opts.emitJsonDbc = opts.emitWom =
                    opts.emitWob = opts.emitTerrain = true;
            }
        } else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            printUsage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown option: " << argv[i] << "\n";
            printUsage(argv[0]);
            return 1;
        }
    }

    // --purge-proprietary: walk a tree and (dry-run unless --confirm-purge)
    // remove every .blp/.dbc/.m2/.skin/.wmo/.adt that has a confirmed
    // open-format sidecar at least as new. Useful after a dual-format
    // extraction when the user only wants the open-format files (no
    // private-server compatibility needed).
    if (!purgeDir.empty()) {
        if (!std::filesystem::exists(purgeDir)) {
            std::cerr << "purge-proprietary: " << purgeDir << " does not exist\n";
            return 1;
        }
        if (!jsonOutput) {
            std::cout << (confirmPurge ? "Purging" : "Dry-run: would purge")
                      << " proprietary files under " << purgeDir
                      << " where open-format sidecar exists...\n";
        }
        // (proprietary ext, sidecar ext) pairs. .skin pairs with the
        // matching foo.m2's foo.wom (skin gets purged when WOM exists
        // because WOM stores merged geometry). Group .wmo (foo_NNN.wmo)
        // pair with the parent's .wob.
        struct Pair { const char* propExt; const char* sidecarExt; };
        const Pair pairs[] = {
            {".blp", ".png"},
            {".dbc", ".json"},
            {".m2",  ".wom"},
            {".wmo", ".wob"},  // root WMO sidecar; group handling below
            {".adt", ".whm"},
        };
        uint64_t toRemove = 0, removed = 0, totalBytes = 0;
        namespace fs = std::filesystem;
        for (auto& entry : fs::recursive_directory_iterator(purgeDir)) {
            if (!entry.is_regular_file()) continue;
            std::string p = entry.path().string();
            std::string ext = entry.path().extension().string();
            std::transform(ext.begin(), ext.end(), ext.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            std::string base = p.substr(0, p.size() - ext.size());

            // Skin file shares its WOM sidecar with the parent .m2.
            std::string sidecar;
            if (ext == ".skin") {
                // foo00.skin → foo.wom check
                if (base.size() >= 2 && base.substr(base.size() - 2) == "00") {
                    sidecar = base.substr(0, base.size() - 2) + ".wom";
                }
            } else if (ext == ".wmo") {
                // Group sub-files purge if the parent root WMO has WOB.
                std::string fname = entry.path().filename().string();
                auto under = fname.rfind('_');
                bool isGroup = (under != std::string::npos &&
                                fname.size() - under == 8);
                if (isGroup) {
                    auto last = base.rfind('_');
                    if (last != std::string::npos)
                        sidecar = base.substr(0, last) + ".wob";
                } else {
                    sidecar = base + ".wob";
                }
            } else {
                for (const auto& pr : pairs) {
                    if (ext == pr.propExt) { sidecar = base + pr.sidecarExt; break; }
                }
            }
            if (sidecar.empty() || !fs::exists(sidecar)) continue;

            std::error_code ec;
            auto srcMtime  = fs::last_write_time(p, ec);
            auto sideMtime = fs::last_write_time(sidecar, ec);
            if (ec || sideMtime < srcMtime) continue;

            toRemove++;
            totalBytes += entry.file_size();
            if (confirmPurge) {
                std::error_code rmEc;
                if (fs::remove(p, rmEc)) removed++;
            }
        }
        if (jsonOutput) {
            nlohmann::json j;
            j["dir"] = purgeDir;
            j["confirmed"] = confirmPurge;
            j["candidates"] = toRemove;
            j["removed"] = removed;
            j["totalBytes"] = totalBytes;
            std::cout << j.dump(2) << "\n";
            return 0;
        }
        std::cout << (confirmPurge ? "  removed: " : "  would remove: ")
                  << toRemove << " files (" << (totalBytes / (1024.0 * 1024.0))
                  << " MB)\n";
        if (!confirmPurge) {
            std::cout << "  (re-run with --confirm-purge to actually delete)\n";
        }
        return 0;
    }

    // --upgrade-extract: standalone post-extract pass on an existing tree.
    if (!upgradeDir.empty()) {
        if (!std::filesystem::exists(upgradeDir)) {
            std::cerr << "upgrade-extract: " << upgradeDir << " does not exist\n";
            return 1;
        }
        if (!jsonOutput) {
            std::cout << "Walking " << upgradeDir
                      << " for open-format upgrades...\n";
        }
        auto t0 = std::chrono::steady_clock::now();
        wowee::tools::OpenFormatStats stats;
        unsigned int t = opts.threads > 0 ? static_cast<unsigned int>(opts.threads) : 0;
        wowee::tools::emitOpenFormats(upgradeDir,
                                       opts.emitPng, opts.emitJsonDbc,
                                       opts.emitWom, opts.emitWob,
                                       opts.emitTerrain, stats, t,
                                       /*incremental=*/true);
        auto secs = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t0).count() / 1000.0;
        if (jsonOutput) {
            // Schema mirrors the wowee_editor --info-extract --json layout
            // so CI scripts can use one jq filter for both.
            nlohmann::json j;
            j["dir"] = upgradeDir;
            j["elapsedSeconds"] = secs;
            j["skipped"] = stats.skipped;
            auto fmt = [](uint32_t ok, uint32_t fail) {
                return nlohmann::json{{"ok", ok}, {"failed", fail}};
            };
            j["png"]      = fmt(stats.pngOk,     stats.pngFail);
            j["jsonDbc"]  = fmt(stats.jsonDbcOk, stats.jsonDbcFail);
            j["wom"]      = fmt(stats.womOk,     stats.womFail);
            j["wob"]      = fmt(stats.wobOk,     stats.wobFail);
            j["terrain"]  = fmt(stats.whmOk,     stats.whmFail);
            std::cout << j.dump(2) << "\n";
            return 0;
        }
        std::cout << "  elapsed           : " << secs << " s\n";
        if (stats.skipped > 0)
            std::cout << "  up-to-date (skip) : " << stats.skipped << "\n";
        std::cout << "  PNG (BLP→PNG)     : " << stats.pngOk     << " ok"
                  << (stats.pngFail     ? ", " + std::to_string(stats.pngFail)     + " failed" : "") << "\n";
        std::cout << "  JSON (DBC→JSON)   : " << stats.jsonDbcOk << " ok"
                  << (stats.jsonDbcFail ? ", " + std::to_string(stats.jsonDbcFail) + " failed" : "") << "\n";
        std::cout << "  WOM (M2→WOM)      : " << stats.womOk     << " ok"
                  << (stats.womFail     ? ", " + std::to_string(stats.womFail)     + " failed" : "") << "\n";
        std::cout << "  WOB (WMO→WOB)     : " << stats.wobOk     << " ok"
                  << (stats.wobFail     ? ", " + std::to_string(stats.wobFail)     + " failed" : "") << "\n";
        std::cout << "  WHM/WOT/WOC (ADT) : " << stats.whmOk     << " ok"
                  << (stats.whmFail     ? ", " + std::to_string(stats.whmFail)     + " failed" : "") << "\n";
        return 0;
    }

    if (opts.mpqDir.empty() || opts.outputDir.empty()) {
        std::cerr << "Error: --mpq-dir and --output are required\n\n";
        printUsage(argv[0]);
#ifdef _WIN32
        if (argc == 1) {
            // Double-clicked, and the asset manager window was not beside
            // this to hand over to. Explorer's console dies with the process,
            // so without this pause the message above is gone before anyone
            // can read it and the whole thing looks like it did nothing.
            std::cout << "\nwowee_assets.exe, the window that does this "
                         "without any of the above, was not found next to "
                         "this program.\n"
                      << "\nPress Enter to close this window...";
            std::cin.get();
        }
#endif
        return 1;
    }

    // Auto-detect expansion if not specified
    if (expansion.empty() || expansion == "auto") {
        expansion = wowee::tools::Extractor::detectExpansion(opts.mpqDir);
        if (expansion.empty()) {
            std::cerr << "Error: Could not auto-detect expansion. No known MPQ archives found in: "
                      << opts.mpqDir << "\n"
                      << "Specify manually with --expansion classic|tbc|wotlk\n";
            return 1;
        }
        std::cout << "Auto-detected expansion: " << expansion << "\n";
    }
    opts.expansion = expansion;

    // cata extracts, and only extracts. Its archives are still MPQ and its
    // models are still MD20, so the art is readable; the wire is not, and
    // docs/plan-cataclysm.md is the measure of what that would take. Extract a
    // 4.3.4 client to borrow its art - as an override over another expansion's
    // tree - rather than to play it.
    if (expansion != "classic" && expansion != "turtle" &&
        expansion != "tbc" && expansion != "wotlk" && expansion != "cata") {
        std::cerr << "Error: Unsupported expansion '" << expansion
                  << "'. Expected classic, turtle, tbc, wotlk, or cata.\n";
        return 1;
    }
    if (expansion == "cata") {
        std::cout << "Note: 4.3.4 is extracted for its art. WoWee cannot talk to a "
                     "Cataclysm server;\n      see docs/plan-cataclysm.md.\n";
    }

    // Auto-detect locale if not specified. Nothing is said when none is found
    // here: this looks in the folder that was named, which may be the game
    // folder rather than Data, and the extractor asks again against the Data
    // folder it resolves - so it is the one that knows whether a locale is
    // really absent, and it warns there.
    if (locale.empty() || locale == "auto") {
        locale = wowee::tools::Extractor::detectLocale(opts.mpqDir);
        if (!locale.empty()) {
            std::cout << "Auto-detected locale: " << locale << "\n";
        }
    }
    opts.locale = locale;

    // Auto-detect external listfile if not specified
    if (opts.listFile.empty()) {
        // Look next to the binary, then in the source tree
        namespace fs = std::filesystem;
        std::string binDir = fs::path(argv[0]).parent_path().string();
        for (const auto& candidate : {
            binDir + "/listfile.txt",
            binDir + "/../../../tools/asset_extract/listfile.txt",
            opts.mpqDir + "/listfile.txt",
        }) {
            if (fs::exists(candidate)) {
                opts.listFile = candidate;
                std::cout << "Auto-detected listfile: " << candidate << "\n";
                break;
            }
        }
    }

    std::cout << "=== Wowee Asset Extractor ===\n";
    std::cout << "MPQ directory: " << opts.mpqDir << "\n";
    std::cout << "Output:        " << opts.outputDir << "\n";
    std::cout << "Expansion:     " << expansion << "\n";
    if (opts.expansionSubdir) {
        std::cout << "Layout:        isolated (expansions/" << expansion << ")\n";
    }
    if (!locale.empty()) {
        std::cout << "Locale:        " << locale << "\n";
    }
    if (opts.onlyUsedDbcs) {
        std::cout << "Mode:          only-used-dbcs\n";
    }
    if (opts.skipDbcExtraction) {
        std::cout << "DBC extract:   skipped\n";
    }
    if (opts.generateDbcCsv) {
        std::cout << "DBC CSV:       enabled\n";
        if (!opts.dbcCsvOutputDir.empty()) {
            std::cout << "DBC CSV out:   " << opts.dbcCsvOutputDir << "\n";
        }
    }

    if (!opts.referenceManifest.empty()) {
        std::cout << "Reference:     " << opts.referenceManifest << " (delta mode)\n";
    }

    if (!wowee::tools::Extractor::run(opts)) {
        std::cerr << "Extraction failed!\n";
        return 1;
    }

    return 0;
}
