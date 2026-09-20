#include "install_scan.hpp"

#include "casc.hpp"
#include "install_probe.hpp"
#include "../asset_extract/extractor.hpp"

#include <algorithm>
#include <filesystem>

namespace wowee::assets {
namespace {

namespace fs = std::filesystem;

bool looksLikeCasc(const fs::path& dir) {
    std::error_code ec;
    // Data/data holds the .idx indices and the numbered blobs; Data/indices is
    // where some builds keep the former instead. Either is enough to know.
    return fs::is_directory(dir / "Data" / "data", ec) ||
           fs::is_directory(dir / "Data" / "indices", ec) ||
           fs::is_directory(dir / "data", ec);
}

}  // namespace

InstallScan scanInstall(const std::string& path) {
    InstallScan out;
    if (path.empty()) return out;

    std::error_code ec;
    const fs::path root(path);
    if (!fs::is_directory(root, ec)) {
        out.kind = InstallKind::Missing;
        out.note = "That folder does not exist.";
        return out;
    }

    // Somebody may hand over the game folder, the Data folder inside it, the
    // folder they unpacked everything into with the game one level down, or
    // any of those spelled with a different case. All of them are a reasonable
    // answer to "choose your World of Warcraft folder", so this looks for the
    // archives rather than insisting on one shape.
    const DataDirProbe probe = findDataDir(path);
    {
        const int archives = probe.archives;
        if (archives > 0) {
            out.kind = InstallKind::Mpq;
            out.dataDir = probe.dataDir;
            out.archiveCount = archives;

            // Which game it is, from the archives that are there. Worth saying
            // rather than making somebody pick it off a list: the answer is in
            // the folder they just chose, and a person who mis-picks it builds
            // the wrong thing for eight minutes before finding out.
            out.expansion = tools::Extractor::detectExpansion(out.dataDir);
            // Where, not just how many. When the archives turn up somewhere
            // other than the folder that was chosen - one level down, or in a
            // Data folder spelled differently - the person needs to see which
            // folder is being read, because that is the answer to "is it
            // going to use the right copy".
            const bool elsewhere = fs::path(out.dataDir) != root;
            const std::string where = elsewhere ? " in " + out.dataDir : " here";
            out.note = out.expansion.empty()
                           ? "Found " + std::to_string(archives) + " archives" + where +
                                 ", but not which game they are from."
                           : "Found a game" + where + " - " + std::to_string(archives) +
                                 " archives, readable.";
            return out;
        }
    }

    if (looksLikeCasc(root)) {
        out.kind = InstallKind::Casc;
        out.dataDir = root.string();
        out.note = "This is Warlords or later. Those can supply models, but not a whole "
                   "game: none of their data tables are in a form this client reads, and "
                   "the terrain is later geography besides.";
        return out;
    }

    out.kind = InstallKind::Unknown;
    if (probe.packed > 0) {
        // The likeliest reason for an empty-looking folder, and the one a
        // person cannot be expected to guess from "no game archives here":
        // the download is still a download.
        out.note = "No game archives in there, but " + std::to_string(probe.packed) +
                   " packed file(s) - a .zip, .rar or installer. Unpack the game first, "
                   "then choose the folder that has Data inside it.";
        return out;
    }
    out.note = "No game archives in " + std::to_string(probe.searched) +
               " folder(s) here. Choose the folder that has Data inside it, or the Data "
               "folder itself.";
    return out;
}

}  // namespace wowee::assets

namespace wowee::assets {

bool confirmCasc(InstallScan& scan, std::string* error) {
    if (scan.kind != InstallKind::Casc) {
        if (error) *error = "not a CASC installation";
        return false;
    }
    CascStorage storage;
    if (!storage.open(scan.dataDir, error)) return false;
    scan.fileCount = storage.rootCount();
    scan.note = "Read it: " + std::to_string(scan.fileCount) +
                " files. Models can be taken from here - not a whole game, since "
                "none of its data tables are in a form this client reads.";
    return true;
}

}  // namespace wowee::assets
