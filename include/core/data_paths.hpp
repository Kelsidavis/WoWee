#pragma once

/**
 * data_paths.hpp - where this client's assets live when nobody has said.
 *
 * Two programs need the same answer and did not have it. The client looked in
 * ~/Library/Application Support/Wowee/Data on macOS and nowhere in particular
 * anywhere else; the asset manager wrote to Data/ beside whatever directory the
 * terminal happened to be in. Run the manager from a home directory and it
 * spent eight minutes writing a complete extraction the client would never
 * look at, with nothing on screen saying where any of it had gone.
 *
 * So: one rule, in one place, that both read. Each platform's own per-user data
 * directory, which is where a user's data belongs on that platform and is
 * writable without asking for anything.
 *
 * This is only the default. WOW_DATA_PATH still wins for the client and the
 * asset manager still lets the folder be chosen; neither of those goes through
 * here.
 */

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace wowee {
namespace core {

/// The per-user data directory for this client, or empty if the platform will
/// not say where one is.
inline std::filesystem::path userDataRoot() {
    namespace fs = std::filesystem;
#ifdef _WIN32
    if (const char* local = std::getenv("LOCALAPPDATA"); local != nullptr && *local != '\0') {
        return fs::path(local) / "Wowee" / "Data";
    }
    if (const char* profile = std::getenv("USERPROFILE"); profile != nullptr && *profile != '\0') {
        return fs::path(profile) / "AppData" / "Local" / "Wowee" / "Data";
    }
    return {};
#else
    const char* home = std::getenv("HOME");
    if (home == nullptr || *home == '\0') return {};
#ifdef __APPLE__
    return fs::path(home) / "Library" / "Application Support" / "Wowee" / "Data";
#else
    // The XDG base directory spec, which is what a Linux or BSD desktop expects
    // and what a backup tool knows to pick up.
    if (const char* share = std::getenv("XDG_DATA_HOME"); share != nullptr && *share != '\0') {
        return fs::path(share) / "wowee" / "Data";
    }
    return fs::path(home) / ".local" / "share" / "wowee" / "Data";
#endif
#endif
}

/// The expansions extracted under a data root, by directory name, sorted.
///
/// Several games can be built into one data folder, each under its own name,
/// and both programs need to know which: the asset manager to say what is
/// already there and to name a pack holding more than one, the client to offer
/// the choice at its login screen. One answer, so the two cannot disagree about
/// what is installed.
///
/// The manifest is what the extractor writes last, so its presence is the
/// difference between a finished extraction and a directory somebody made -
/// or one a run that was stopped halfway left behind.
inline std::vector<std::string> installedExpansions(const std::filesystem::path& dataRoot) {
    namespace fs = std::filesystem;
    std::vector<std::string> out;
    if (dataRoot.empty()) return out;

    std::error_code ec;
    const fs::path expansions = dataRoot / "expansions";
    for (fs::directory_iterator it(expansions, ec), end; it != end && !ec; it.increment(ec)) {
        if (!fs::is_regular_file(it->path() / "manifest.json", ec)) continue;
        out.push_back(it->path().filename().string());
    }
    std::sort(out.begin(), out.end());
    return out;
}

/// Whether a directory holds an extraction: a manifest at its root, or one in
/// any expansion beneath it.
inline bool holdsExtraction(const std::filesystem::path& dataRoot) {
    namespace fs = std::filesystem;
    std::error_code ec;
    if (dataRoot.empty()) return false;
    if (fs::is_regular_file(dataRoot / "manifest.json", ec)) return true;
    return !installedExpansions(dataRoot).empty();
}

/// The data folders something the extraction wrote may be under, most likely
/// first: the one the client is reading - WOW_DATA_PATH, which startup points
/// at the per-user folder when an extraction is there - and then Data/ beside
/// the client. The same folder is not named twice.
///
/// For the lookups that name a file under the data folder by hand rather than
/// through the asset manager. Those looked in Data/ alone, which is not where
/// the asset builder writes, so an extraction it made was never found by them.
inline std::vector<std::string> extractionRoots() {
    namespace fs = std::filesystem;
    std::vector<std::string> roots;
    if (const char* d = std::getenv("WOW_DATA_PATH"); d != nullptr && *d != '\0') {
        roots.emplace_back(d);
    }
    std::error_code ec;
    if (roots.empty() || !fs::equivalent(roots.front(), "Data", ec)) roots.emplace_back("Data");
    return roots;
}

/// Copy this client's own description of each expansion into a data root
/// that holds an extraction of it. Returns how many files it wrote.
///
/// expansion.json, opcodes.json, update_fields.json and dbc_layouts.json are
/// not game data. They ship with the client, in Data/expansions/<id>/ beside
/// it, and they describe how this build speaks to a server - but the client
/// reads them from the same directory as the extraction. The extractor does
/// not write them. So an extraction made anywhere but beside the client - the
/// per-user data directory, which is where the asset builder puts one - held
/// no expansion.json at all: the client found no expansion to run, opened no
/// assets, and could log in and then go no further. And one that did have
/// them, copied in by an older build, went on answering with that build's
/// tables after every upgrade.
///
/// Only expansions the data root already holds are touched, and only the
/// files the install carries at the top of its own expansion directory. A
/// file already identical is left alone, so an up-to-date data root costs a
/// comparison and no writes. `installRoot` and `dataRoot` being the same
/// directory is the development tree and a portable install, and does
/// nothing.
inline int syncClientTables(const std::filesystem::path& installRoot,
                            const std::filesystem::path& dataRoot,
                            std::vector<std::string>* failures = nullptr) {
    namespace fs = std::filesystem;
    std::error_code ec;
    if (installRoot.empty() || dataRoot.empty()) return 0;
    if (fs::equivalent(installRoot, dataRoot, ec)) return 0;

    auto contents = [](const fs::path& p, std::string& out) {
        std::ifstream in(p, std::ios::binary);
        if (!in) return false;
        out.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
        return true;
    };

    int written = 0;
    const fs::path source = installRoot / "expansions";
    for (fs::directory_iterator exp(source, ec), end; exp != end && !ec; exp.increment(ec)) {
        std::error_code entryEc;
        if (!exp->is_directory(entryEc)) continue;
        const fs::path target = dataRoot / "expansions" / exp->path().filename();
        if (!fs::is_directory(target, entryEc)) continue;

        for (fs::directory_iterator file(exp->path(), entryEc), fileEnd;
             file != fileEnd && !entryEc; file.increment(entryEc)) {
            std::error_code fileEc;
            if (!file->is_regular_file(fileEc)) continue;
            if (file->path().extension() != ".json") continue;
            // The extraction's own record of what it wrote, if an install
            // ever carries one, is not a table and not the client's to replace.
            if (file->path().filename() == "manifest.json") continue;

            const fs::path dest = target / file->path().filename();
            std::string want, have;
            if (!contents(file->path(), want)) continue;
            if (contents(dest, have) && have == want) continue;

            std::ofstream out(dest, std::ios::binary | std::ios::trunc);
            if (out && out.write(want.data(), static_cast<std::streamsize>(want.size()))) {
                ++written;
            } else if (failures) {
                failures->push_back(dest.string());
            }
        }
    }
    return written;
}

}  // namespace core
}  // namespace wowee
