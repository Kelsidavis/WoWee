// The compressed alpha scan answers what the decoded one did.
//
// M2 textures used to be decoded to RGBA8 for one reason: something walked
// every fourth byte to learn whether the texture had any transparency, and
// fed that to m2BatchNeedsAlphaTest. hasAlpha == false is what turns an alpha
// test on, so an answer that drifts toward "has alpha" silences a test that
// is currently running, in code whose comments record having been debugged
// for exactly that.
//
// Reading the format instead would drift that way. A DXT3 or DXT5 texture
// carries an alpha channel that may be opaque in every texel, and DXT1's
// punch-through is per block. So BLPImage::hasTransparency reads the blocks,
// and this checks its answer against the decoded one over whatever assets are
// present - the only way to know the two agree on real data rather than on
// the cases someone thought to write down.
#include <catch_amalgamated.hpp>

#include "pipeline/blp_loader.hpp"

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {

std::vector<uint8_t> slurpBytes(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

#ifdef WOWEE_SOURCE_DIR
const std::filesystem::path kDataDir = std::filesystem::path(WOWEE_SOURCE_DIR) / "Data";
#else
const std::filesystem::path kDataDir = "Data";
#endif

}  // namespace

TEST_CASE("the block alpha scan agrees with the decoded one", "[blp]") {
    std::error_code ec;
    if (!std::filesystem::is_directory(kDataDir, ec)) {
        WARN("no Data directory - nothing to compare against");
        return;
    }

    int compared = 0;
    int disagreed = 0;
    std::vector<std::string> examples;

    // Enough to cross every format and both answers without walking 6944
    // files on every test run.
    constexpr int kMaxFiles = 400;
    for (std::filesystem::recursive_directory_iterator it(kDataDir, ec), end;
         it != end && compared < kMaxFiles; it.increment(ec)) {
        if (ec) break;
        if (!it->is_regular_file(ec)) continue;
        if (it->path().extension() != ".blp") continue;

        const std::vector<uint8_t> bytes = slurpBytes(it->path());
        if (bytes.empty()) continue;

        const wowee::pipeline::BLPImage decoded =
            wowee::pipeline::BLPLoader::load(bytes, false);
        const wowee::pipeline::BLPImage blocks =
            wowee::pipeline::BLPLoader::load(bytes, true);
        if (!decoded.isValid() || !blocks.isValid()) continue;
        // Palette and ARGB8888 come back decoded either way; there is nothing
        // to compare that is not the same code path twice.
        if (!blocks.isBlockCompressed()) continue;

        ++compared;
        if (decoded.hasTransparency() != blocks.hasTransparency()) {
            ++disagreed;
            if (examples.size() < 5) {
                examples.push_back(it->path().filename().string() + ": decoded=" +
                                   (decoded.hasTransparency() ? "yes" : "no") + " blocks=" +
                                   (blocks.hasTransparency() ? "yes" : "no"));
            }
        }
    }

    if (compared == 0) {
        WARN("no block-compressed BLP files found - nothing compared");
        return;
    }

    INFO("compared " << compared << " textures, " << disagreed << " disagreed");
    for (const auto& e : examples) INFO("  " << e);
    CHECK(disagreed == 0);
}
