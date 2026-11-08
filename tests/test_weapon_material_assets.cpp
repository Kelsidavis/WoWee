#include <catch_amalgamated.hpp>

#include "pipeline/blp_loader.hpp"
#include "pipeline/m2_loader.hpp"
#include "rendering/character_material_policy.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <vector>

namespace {

std::vector<uint8_t> readBytes(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(input), {}};
}

} // namespace

TEST_CASE("Bloodsurge's opaque sword geometry ignores the texture alpha", "[m2][weapon][assets]") {
    // Optional regression against Bloodsurge's extracted model, skin and texture.
    const char* root = std::getenv("WOWEE_BLOODSURGE_WEAPON_DIR");
    if (!root || !*root) SKIP("Set WOWEE_BLOODSURGE_WEAPON_DIR to item/objectcomponents/weapon");
    const auto dir = std::filesystem::path(root);
    const auto m2 = readBytes(dir / "sword_1h_icecrownraid_d_03.m2");
    const auto skin = readBytes(dir / "sword_1h_icecrownraid_d_0300.skin");
    const auto blp = readBytes(dir / "sword_1h_icecrownraid_d_03blue.blp");
    REQUIRE_FALSE(m2.empty());
    REQUIRE_FALSE(skin.empty());
    REQUIRE_FALSE(blp.empty());

    auto model = wowee::pipeline::M2Loader::load(m2);
    REQUIRE(wowee::pipeline::M2Loader::loadSkin(skin, model));
    REQUIRE(model.isValid());
    REQUIRE(model.textures.size() >= 1);
    CHECK(model.textures[0].type == 2); // ItemDisplayInfo supplies the blue atlas.
    const auto image = wowee::pipeline::BLPLoader::load(blp);
    REQUIRE(image.isValid());
    REQUIRE(image.data.size() == static_cast<size_t>(image.width * image.height * 4));
    REQUIRE(image.hasTransparency());

    const wowee::pipeline::M2Batch* solid = nullptr;
    for (const auto& batch : model.batches) {
        REQUIRE(batch.materialIndex < model.materials.size());
        if (model.materials[batch.materialIndex].blendMode == 0) {
            REQUIRE(solid == nullptr);
            solid = &batch;
        }
    }
    REQUIRE(solid != nullptr);
    REQUIRE(solid->indexCount >= 3);
    REQUIRE(solid->indexCount % 3 == 0);
    REQUIRE(static_cast<size_t>(solid->indexStart) + solid->indexCount <= model.indices.size());

    // Sample each triangle's actual atlas UVs. This is not a screenshot test,
    // but it proves that the opaque batch samples painted RGB with low alpha,
    // rather than merely finding unused transparency elsewhere in the atlas.
    size_t samples = 0;
    size_t lowAlphaPainted = 0;
    for (size_t i = solid->indexStart; i < solid->indexStart + solid->indexCount; i += 3) {
        const auto a = model.indices[i];
        const auto b = model.indices[i + 1];
        const auto c = model.indices[i + 2];
        REQUIRE(a < model.vertices.size());
        REQUIRE(b < model.vertices.size());
        REQUIRE(c < model.vertices.size());
        const auto uv = (model.vertices[a].texCoords[0] + model.vertices[b].texCoords[0] +
                         model.vertices[c].texCoords[0]) / 3.0f;
        const int x = static_cast<int>((uv.x - std::floor(uv.x)) * image.width);
        const int y = static_cast<int>((uv.y - std::floor(uv.y)) * image.height);
        REQUIRE(x >= 0);
        REQUIRE(x < image.width);
        REQUIRE(y >= 0);
        REQUIRE(y < image.height);
        const size_t pixel = (static_cast<size_t>(y) * image.width + x) * 4;
        ++samples;
        if (image.data[pixel + 3] < 128 &&
            std::max({image.data[pixel], image.data[pixel + 1], image.data[pixel + 2]}) > 20)
            ++lowAlphaPainted;
    }
    INFO("painted low-alpha triangle samples: " << lowAlphaPainted << " / " << samples);
    CHECK(lowAlphaPainted * 4 > samples * 3);

    const auto domain = wowee::rendering::characterMaterialDomain(false, false, true, false);
    CHECK(domain == wowee::rendering::CharacterMaterialDomain::Attachment);
    CHECK_FALSE(wowee::rendering::characterBatchNeedsCutout(
        domain, model.materials[solid->materialIndex].blendMode, true, false));
    // This is the old character-only decision that erased the sword surface.
    CHECK(wowee::rendering::characterBatchNeedsCutout(
        wowee::rendering::CharacterMaterialDomain::Character, 0, true, false));
}
