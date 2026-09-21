// Where this client's assets live when nobody has said, and what is in there.
//
// Two programs needed the same answer and had neither. The client looked in a
// macOS-only location and nowhere in particular elsewhere; the asset manager
// wrote to Data/ beside whatever directory the terminal happened to be in - so
// running it from a home folder spent eight minutes writing a complete
// extraction the client would never look at, with nothing saying where it went.

#include <catch2/catch_amalgamated.hpp>

#include <atomic>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include "core/data_paths.hpp"
#include "core/env.hpp"

namespace fs = std::filesystem;
using namespace wowee::core;

namespace {

struct Sandbox {
    fs::path root;
    Sandbox() {
        static std::atomic<int> counter{0};
        root = fs::temp_directory_path() /
               ("wowee_paths_" + std::to_string(counter.fetch_add(1)) + "_" +
                std::to_string(reinterpret_cast<uintptr_t>(this)));
        fs::create_directories(root);
    }
    ~Sandbox() { std::error_code ec; fs::remove_all(root, ec); }

    void extraction(const std::string& expansion) {
        const fs::path at = root / "expansions" / expansion;
        fs::create_directories(at);
        std::ofstream(at / "manifest.json") << "{\"files\":[]}";
    }
    /// A directory with no manifest: a run that was stopped, or somebody's
    /// empty folder.
    void halfDone(const std::string& expansion) {
        fs::create_directories(root / "expansions" / expansion / "world");
    }
};

}  // namespace

TEST_CASE("the default data root is a real per-user location") {
    const fs::path root = userDataRoot();

    // Empty is the documented answer where the platform will not say - a
    // container with no HOME, say - and is not a failure. Everywhere it does
    // answer, the answer has to be somewhere absolute.
    if (root.empty()) {
        WARN("This environment names no per-user directory; nothing to check.");
        return;
    }
    CHECK(root.is_absolute());
    // The folder the assets go in, not the folder above it: both programs join
    // "expansions" onto this, and a root off by one level is two programs
    // writing and reading different places.
    CHECK(root.filename() == "Data");
}

TEST_CASE("nothing is installed in an empty folder") {
    Sandbox box;
    CHECK(installedExpansions(box.root).empty());
    CHECK_FALSE(holdsExtraction(box.root));
}

TEST_CASE("a folder that does not exist answers empty rather than throwing") {
    CHECK(installedExpansions("/no/such/place/at/all").empty());
    CHECK_FALSE(holdsExtraction("/no/such/place/at/all"));
    CHECK(installedExpansions("").empty());
    CHECK_FALSE(holdsExtraction(""));
}

TEST_CASE("every game built into one folder is found") {
    Sandbox box;
    box.extraction("wotlk");
    box.extraction("tbc");
    box.extraction("classic");

    const std::vector<std::string> found = installedExpansions(box.root);
    REQUIRE(found.size() == 3);
    // Sorted, so the same folder reads the same way twice running - a
    // directory iterator's order is not promised to be anything.
    CHECK(found[0] == "classic");
    CHECK(found[1] == "tbc");
    CHECK(found[2] == "wotlk");
    CHECK(holdsExtraction(box.root));
}

TEST_CASE("an extraction that never finished does not count as installed") {
    Sandbox box;
    box.halfDone("wotlk");
    // The manifest is written last. A directory of files with no manifest is a
    // run that stopped partway, and offering it to play with is offering a
    // game with holes in it.
    CHECK(installedExpansions(box.root).empty());
    CHECK_FALSE(holdsExtraction(box.root));

    box.extraction("wotlk");
    CHECK(installedExpansions(box.root).size() == 1);
}

TEST_CASE("a manifest at the root counts, without any expansions under it") {
    Sandbox box;
    std::ofstream(box.root / "manifest.json") << "{\"files\":[]}";
    // The older layout, from before expansions had folders of their own. The
    // client still reads it, so it still counts as something being there.
    CHECK(holdsExtraction(box.root));
    CHECK(installedExpansions(box.root).empty());
}

namespace {
std::string readAll(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}
}  // namespace

TEST_CASE("the client's tables reach an extraction that has none") {
    // The report: built with the asset builder into the per-user directory,
    // which got the game data and no expansion.json, so the client found no
    // expansion and could not enter the world after logging in.
    Sandbox install;
    Sandbox data;
    const fs::path shipped = install.root / "expansions" / "wotlk";
    fs::create_directories(shipped);
    std::ofstream(shipped / "expansion.json") << "{\"id\":\"wotlk\"}";
    std::ofstream(shipped / "opcodes.json") << "{\"a\":1}";
    data.extraction("wotlk");

    CHECK(syncClientTables(install.root, data.root) == 2);
    const fs::path got = data.root / "expansions" / "wotlk";
    CHECK(readAll(got / "expansion.json") == "{\"id\":\"wotlk\"}");
    CHECK(readAll(got / "opcodes.json") == "{\"a\":1}");
    // The extraction's own manifest is not the client's to touch.
    CHECK(readAll(got / "manifest.json") == "{\"files\":[]}");

    // Up to date is no writes at all.
    CHECK(syncClientTables(install.root, data.root) == 0);
}

TEST_CASE("an older build's table is replaced by this one's") {
    Sandbox install;
    Sandbox data;
    const fs::path shipped = install.root / "expansions" / "wotlk";
    fs::create_directories(shipped);
    std::ofstream(shipped / "dbc_layouts.json") << "new";
    data.extraction("wotlk");
    std::ofstream(data.root / "expansions" / "wotlk" / "dbc_layouts.json") << "old";

    CHECK(syncClientTables(install.root, data.root) == 1);
    CHECK(readAll(data.root / "expansions" / "wotlk" / "dbc_layouts.json") == "new");
}

TEST_CASE("an expansion nobody extracted is not created") {
    Sandbox install;
    Sandbox data;
    fs::create_directories(install.root / "expansions" / "classic");
    std::ofstream(install.root / "expansions" / "classic" / "expansion.json") << "{}";
    data.extraction("wotlk");

    CHECK(syncClientTables(install.root, data.root) == 0);
    CHECK_FALSE(fs::exists(data.root / "expansions" / "classic"));
}

TEST_CASE("an install that is its own data root copies nothing") {
    Sandbox box;
    box.extraction("wotlk");
    std::ofstream(box.root / "expansions" / "wotlk" / "expansion.json") << "{}";
    CHECK(syncClientTables(box.root, box.root) == 0);
    CHECK(syncClientTables("", box.root) == 0);
    CHECK(syncClientTables(box.root, "/no/such/place") == 0);
}

TEST_CASE("the data folder being read is searched before the one beside the client") {
    // The integrity hash and Warden's copy of the executable looked in Data/
    // alone, and the asset builder writes to the per-user folder - so an
    // extraction it made was never found by either.
    const char* saved = std::getenv("WOW_DATA_PATH");
    const std::string restore = saved ? saved : "";

    Sandbox data;
    setEnvVar("WOW_DATA_PATH", data.root.string().c_str());
    auto roots = extractionRoots();
    REQUIRE(roots.size() == 2);
    CHECK(roots[0] == data.root.string());
    CHECK(roots[1] == "Data");

    unsetEnvVar("WOW_DATA_PATH");
    roots = extractionRoots();
    REQUIRE(roots.size() == 1);
    CHECK(roots[0] == "Data");

    if (saved) setEnvVar("WOW_DATA_PATH", restore.c_str());
}
