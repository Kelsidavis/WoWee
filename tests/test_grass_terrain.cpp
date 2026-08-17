// Whether grass grows at a point, answered from the terrain rather than from
// a classifier of our own.
//
// The rule these pin down is that the ADT already knows: a texture layer's
// ground-effect id, looked up in the ground-effect table, is Blizzard saying
// what grows there. Roads, cliffs and bare dirt carry no effect, and the
// answer for them has to be nothing - spec §44, "if the terrain says no grass,
// put no grass".
//
// Headless: no device, no tile, no world. The chunks here are built by hand.

#include <catch_amalgamated.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "pipeline/adt_alpha.hpp"
#include "pipeline/grass_terrain.hpp"

using wowee::pipeline::evaluateGrass;
using wowee::pipeline::GrassSuitability;
using wowee::pipeline::MapChunk;
using wowee::pipeline::TextureLayer;

namespace {

constexpr uint32_t kGrassEffect = 11;   // has density
constexpr uint32_t kRoadEffect = 12;    // present, but grows nothing
constexpr uint32_t kUnknownEffect = 99; // not in the table at all

// A ground-effect table with one entry that grows something.
uint32_t densityFor(uint32_t effectId) {
    if (effectId == kGrassEffect) return 32;
    if (effectId == kRoadEffect) return 0;
    return 0;
}

TextureLayer makeLayer(uint32_t effectId, uint32_t flags = 0, uint32_t offset = 0) {
    TextureLayer layer{};
    layer.textureId = 0;
    layer.flags = flags;
    layer.offsetMCAL = offset;
    layer.effectId = effectId;
    return layer;
}

/// A chunk with a flat height grid and upward normals, so slope and height
/// never interfere with what a case is actually testing.
MapChunk makeFlatChunk() {
    MapChunk chunk;
    chunk.heightMap.heights.fill(0.0f);
    chunk.heightMap.loaded = true;
    chunk.normals.fill(0);
    for (size_t i = 0; i < 145; ++i) {
        chunk.normals[i * 3 + 2] = 127;  // straight up
    }
    return chunk;
}

/// 64x64 uncompressed alpha map, 8 bit, appended to the chunk's MCAL blob.
/// Returns the offset the layer should point at.
uint32_t appendAlpha(MapChunk& chunk, const std::vector<uint8_t>& texels) {
    const auto offset = static_cast<uint32_t>(chunk.alphaMap.size());
    chunk.alphaMap.insert(chunk.alphaMap.end(), texels.begin(), texels.end());
    return offset;
}

/// An alpha map that ramps left to right, which is what a blended boundary
/// between two ground textures looks like.
std::vector<uint8_t> rampAlpha() {
    std::vector<uint8_t> texels(wowee::pipeline::ALPHA_MAP_SIZE, 0);
    for (size_t y = 0; y < wowee::pipeline::ALPHA_MAP_DIM; ++y) {
        for (size_t x = 0; x < wowee::pipeline::ALPHA_MAP_DIM; ++x) {
            texels[y * wowee::pipeline::ALPHA_MAP_DIM + x] =
                static_cast<uint8_t>(x * 255 / (wowee::pipeline::ALPHA_MAP_DIM - 1));
        }
    }
    return texels;
}

} // namespace

TEST_CASE("a chunk of nothing but grass is fully suitable", "[grass][terrain]") {
    MapChunk chunk = makeFlatChunk();
    chunk.layers.push_back(makeLayer(kGrassEffect));

    const auto result = evaluateGrass(chunk, 4.0f, 4.0f, densityFor);
    REQUIRE(result.suitability == Catch::Approx(1.0f));
    REQUIRE(result.effectId == kGrassEffect);
    REQUIRE(result.slope == Catch::Approx(0.0f).margin(0.01));
}

TEST_CASE("a road grows nothing even though its layer exists", "[grass][terrain]") {
    // The distinction that matters: the layer is present and textured, and its
    // effect is a real id. It simply has no density, which is the map saying
    // no vegetation rather than the map saying nothing.
    MapChunk chunk = makeFlatChunk();
    chunk.layers.push_back(makeLayer(kRoadEffect));

    const auto result = evaluateGrass(chunk, 4.0f, 4.0f, densityFor);
    REQUIRE(result.suitability == Catch::Approx(0.0f));
    REQUIRE(result.effectId == 0);
}

TEST_CASE("an effect id absent from the table grows nothing", "[grass][terrain]") {
    MapChunk chunk = makeFlatChunk();
    chunk.layers.push_back(makeLayer(kUnknownEffect));

    REQUIRE(evaluateGrass(chunk, 4.0f, 4.0f, densityFor).suitability == Catch::Approx(0.0f));
}

TEST_CASE("a road never grows grass through a texture-id collision",
          "[grass][terrain]") {
    // A layer with effect id zero is the map saying no vegetation - that is
    // how roads are marked. textureId is an index into the tile's own texture
    // list, a small integer that can numerically equal a real ground-effect
    // id; a fallback that looked it up in the effect table made roads sprout
    // grass chunk by chunk, depending on where the road texture sat in each
    // tile's list. Which is also why the bug looked camera-directional: the
    // chunk ahead collided and the chunk behind did not.
    MapChunk chunk = makeFlatChunk();
    TextureLayer road = makeLayer(0);
    road.textureId = kGrassEffect;  // collides with a real effect id
    chunk.layers.push_back(road);

    const auto result = evaluateGrass(chunk, 4.0f, 4.0f, densityFor);
    REQUIRE(result.suitability == Catch::Approx(0.0f));
    REQUIRE(result.effectId == 0);
}

TEST_CASE("a layer with no effect at all grows nothing", "[grass][terrain]") {
    MapChunk chunk = makeFlatChunk();
    chunk.layers.push_back(makeLayer(0));  // textureId 0 too, so no fallback

    REQUIRE(evaluateGrass(chunk, 4.0f, 4.0f, densityFor).suitability == Catch::Approx(0.0f));
}

TEST_CASE("grass blending into dirt is continuous across the boundary",
          "[grass][terrain]") {
    // Base layer is grass; a dirt layer ramps over it left to right. This is
    // the case spec §4 is about - the answer has to fall off smoothly, because
    // a step would put a straight edge of grass across the ground wherever two
    // textures meet.
    MapChunk chunk = makeFlatChunk();
    chunk.layers.push_back(makeLayer(kGrassEffect));
    const uint32_t offset = appendAlpha(chunk, rampAlpha());
    chunk.layers.push_back(makeLayer(kRoadEffect, 0x100, offset));

    float previous = evaluateGrass(chunk, 0.0f, 4.0f, densityFor).suitability;
    REQUIRE(previous == Catch::Approx(1.0f).margin(0.02));

    float largestStep = 0.0f;
    for (int i = 1; i <= 64; ++i) {
        const float u = static_cast<float>(i) / 8.0f;
        const float current = evaluateGrass(chunk, u, 4.0f, densityFor).suitability;
        largestStep = std::max(largestStep, std::fabs(current - previous));
        previous = current;
    }

    REQUIRE(previous == Catch::Approx(0.0f).margin(0.02));
    // Each step covers a 64th of the chunk. A nearest-neighbour sample would
    // move in jumps of about 1/64; anything near that means the interpolation
    // was lost.
    REQUIRE(largestStep < 0.05f);
}

TEST_CASE("suitability follows the dominant layer's effect", "[grass][terrain]") {
    MapChunk chunk = makeFlatChunk();
    chunk.layers.push_back(makeLayer(kRoadEffect));
    const uint32_t offset = appendAlpha(chunk, rampAlpha());
    chunk.layers.push_back(makeLayer(kGrassEffect, 0x100, offset));

    // Left edge: almost all road. Right edge: almost all grass.
    REQUIRE(evaluateGrass(chunk, 0.16f, 4.0f, densityFor).suitability < 0.1f);

    const auto right = evaluateGrass(chunk, 7.84f, 4.0f, densityFor);
    REQUIRE(right.suitability > 0.9f);
    REQUIRE(right.effectId == kGrassEffect);
}

TEST_CASE("a hole in the terrain grows nothing", "[grass][terrain]") {
    MapChunk chunk = makeFlatChunk();
    chunk.layers.push_back(makeLayer(kGrassEffect));
    // Quad (0,0) - isHole takes (y, x) as quad indices.
    chunk.holes = 1;

    REQUIRE(evaluateGrass(chunk, 0.0f, 0.0f, densityFor).suitability == Catch::Approx(0.0f));
    // A hole is one quad, not the chunk.
    REQUIRE(evaluateGrass(chunk, 7.2f, 7.2f, densityFor).suitability == Catch::Approx(1.0f));
}

TEST_CASE("steep ground holds less grass and a cliff holds none",
          "[grass][terrain]") {
    auto chunkWithNormalZ = [](int8_t nz) {
        MapChunk chunk = makeFlatChunk();
        for (size_t i = 0; i < 145; ++i) chunk.normals[i * 3 + 2] = nz;
        chunk.layers.push_back(makeLayer(kGrassEffect));
        return chunk;
    };

    // Gentle: below the taper, nothing taken away.
    const auto gentle = chunkWithNormalZ(120);
    REQUIRE(evaluateGrass(gentle, 4.0f, 4.0f, densityFor).suitability == Catch::Approx(1.0f));

    // Moderate: inside the taper, reduced but present.
    const auto moderate = chunkWithNormalZ(114);  // slope ~0.10
    const float moderateFit = evaluateGrass(moderate, 4.0f, 4.0f, densityFor).suitability;
    REQUIRE(moderateFit > 0.0f);

    // A cliff face: past the taper entirely.
    const auto cliff = chunkWithNormalZ(40);  // slope ~0.69
    REQUIRE(evaluateGrass(cliff, 4.0f, 4.0f, densityFor).suitability == Catch::Approx(0.0f));
}

TEST_CASE("root height is interpolated from the chunk's own grid",
          "[grass][terrain]") {
    MapChunk chunk = makeFlatChunk();
    chunk.layers.push_back(makeLayer(kGrassEffect));
    chunk.position[2] = 100.0f;
    // Ramp the outer grid along x: column 0 at 0, column 8 at 8.
    for (int y = 0; y <= 8; ++y) {
        for (int x = 0; x <= 8; ++x) {
            chunk.heightMap.heights[static_cast<size_t>(y * 17 + x)] = static_cast<float>(x);
        }
    }

    REQUIRE(evaluateGrass(chunk, 0.0f, 4.0f, densityFor).rootHeight == Catch::Approx(100.0f));
    REQUIRE(evaluateGrass(chunk, 8.0f, 4.0f, densityFor).rootHeight == Catch::Approx(108.0f));
    // Halfway between two grid columns, not snapped to one of them.
    REQUIRE(evaluateGrass(chunk, 0.5f, 4.0f, densityFor).rootHeight
                == Catch::Approx(100.5f).margin(0.01));
}

TEST_CASE("a chunk with no layers grows nothing and does not crash",
          "[grass][terrain]") {
    const MapChunk chunk = makeFlatChunk();
    const auto result = evaluateGrass(chunk, 4.0f, 4.0f, densityFor);
    REQUIRE(result.suitability == Catch::Approx(0.0f));
    REQUIRE(result.effectId == 0);
}

TEST_CASE("root height follows the drawn surface, not the corners",
          "[grass][terrain]") {
    // The mesh fans four triangles from each quad's centre vertex, and MCVT
    // puts that centre wherever the artist needed it. Interpolating the four
    // corners instead answers below the ground that is drawn - by a yard or
    // two on a hillside. A blade is a third of a yard tall, so that buries the
    // entire field, and it looks like nothing was ever uploaded.
    MapChunk chunk = makeFlatChunk();
    chunk.layers.push_back(makeLayer(kGrassEffect));
    chunk.position[2] = 0.0f;

    // Corners of quad (0,0) all at zero, its centre lifted by two yards. The
    // centre vertex of quad (qx,qy) lives at 9 + qy * 17 + qx.
    chunk.heightMap.heights.fill(0.0f);
    chunk.heightMap.heights[9] = 2.0f;

    // Bilinear over the corners would answer 0 here; the fan answers higher.
    const float middle = evaluateGrass(chunk, 0.5f, 0.5f, densityFor).rootHeight;
    REQUIRE(middle > 0.5f);

    // The corner itself is still on the corner.
    REQUIRE(evaluateGrass(chunk, 0.0f, 0.0f, densityFor).rootHeight
                == Catch::Approx(0.0f).margin(0.001));
}

TEST_CASE("the sample range is 0..8 grid units, not 0..1", "[grass][terrain]") {
    // Passing normalised coordinates clamps every sample to one chunk corner,
    // which reads as "this whole chunk is uniformly suitable" rather than as a
    // units mistake. Height is the cheapest place to catch it.
    MapChunk chunk = makeFlatChunk();
    chunk.layers.push_back(makeLayer(kGrassEffect));
    chunk.position[2] = 0.0f;
    for (int y = 0; y <= 8; ++y) {
        for (int x = 0; x <= 8; ++x) {
            chunk.heightMap.heights[static_cast<size_t>(y * 17 + x)] = static_cast<float>(x);
        }
    }
    // Centre vertices on the same ramp, so the fan agrees with the corners.
    for (int qy = 0; qy < 8; ++qy) {
        for (int qx = 0; qx < 8; ++qx) {
            chunk.heightMap.heights[static_cast<size_t>(9 + qy * 17 + qx)] =
                static_cast<float>(qx) + 0.5f;
        }
    }

    // 1.0 is one grid quad in, not the far edge.
    const float atOne = evaluateGrass(chunk, 1.0f, 4.0f, densityFor).rootHeight;
    const float atEight = evaluateGrass(chunk, 8.0f, 4.0f, densityFor).rootHeight;
    REQUIRE(atOne == Catch::Approx(1.0f).margin(0.01));
    REQUIRE(atEight == Catch::Approx(8.0f).margin(0.01));
    REQUIRE(atEight > atOne);
}

TEST_CASE("ground colour is the layers' blend, and absent when not provided",
          "[grass][terrain]") {
    // The same alpha weights that blend the layers' suitability blend their
    // colours, so the grass tints toward exactly the ground drawn under it.
    MapChunk chunk = makeFlatChunk();
    chunk.layers.push_back(makeLayer(kGrassEffect));
    const uint32_t offset = appendAlpha(chunk, rampAlpha());
    chunk.layers.push_back(makeLayer(kRoadEffect, 0x100, offset));

    wowee::pipeline::ChunkGrassContext context;
    context.build(chunk, densityFor);

    SECTION("without colours the flag stays down") {
        const auto fit = evaluateGrass(context, chunk, 4.0f, 4.0f);
        REQUIRE_FALSE(fit.hasGroundColor);
    }

    SECTION("with tones the blend follows the alpha ramp") {
        context.layerShadow[0] = {0.0f, 0.4f, 0.0f};     // base: dark green
        context.layerHighlight[0] = {0.0f, 1.0f, 0.0f};  //       bright green
        context.layerShadow[1] = {0.4f, 0.0f, 0.0f};     // overlay: dark red
        context.layerHighlight[1] = {1.0f, 0.0f, 0.0f};  //          bright red
        context.hasLayerColors = true;

        const auto west = evaluateGrass(context, chunk, 0.2f, 4.0f);
        const auto east = evaluateGrass(context, chunk, 7.8f, 4.0f);
        REQUIRE(west.hasGroundColor);
        REQUIRE(west.groundHighlight.g > west.groundHighlight.r);  // mostly base
        REQUIRE(east.groundHighlight.r > east.groundHighlight.g);  // mostly overlay
        // Shadow and highlight stay distinct through the blend, which is the
        // whole point of carrying two.
        REQUIRE(west.groundHighlight.g > west.groundShadow.g);
    }
}

TEST_CASE("a suppressed layer grows nothing whatever its effect says",
          "[grass][terrain]") {
    // What the renderer does to a road layer once it has seen the texture
    // name. The effect is real and has density; the surface is still a road.
    MapChunk chunk = makeFlatChunk();
    chunk.layers.push_back(makeLayer(kGrassEffect));

    wowee::pipeline::ChunkGrassContext context;
    REQUIRE(context.build(chunk, densityFor));
    REQUIRE(evaluateGrass(context, chunk, 4.0f, 4.0f).suitability == Catch::Approx(1.0f));

    context.suppressLayer(0);
    REQUIRE_FALSE(context.growsAnything);
    REQUIRE(evaluateGrass(context, chunk, 4.0f, 4.0f).suitability == Catch::Approx(0.0f));
}

TEST_CASE("suppressing a road layer leaves the grass under it growing",
          "[grass][terrain]") {
    // A road blended over grass: the grass keeps its own weight and thins as
    // the road takes over, which is the verge.
    MapChunk chunk = makeFlatChunk();
    chunk.layers.push_back(makeLayer(kGrassEffect));      // base: grass
    const uint32_t offset = appendAlpha(chunk, rampAlpha());
    chunk.layers.push_back(makeLayer(kGrassEffect, 0x100, offset));  // road, ramping in

    wowee::pipeline::ChunkGrassContext context;
    context.build(chunk, densityFor);
    context.suppressLayer(1);
    REQUIRE(context.growsAnything);

    // West: almost all base grass. East: almost all road, so almost nothing.
    REQUIRE(evaluateGrass(context, chunk, 0.2f, 4.0f).suitability > 0.9f);
    REQUIRE(evaluateGrass(context, chunk, 7.8f, 4.0f).suitability < 0.1f);
}

TEST_CASE("a road texture keeps grass off, whatever its effect says",
          "[grass][terrain]") {
    // The road's ground effect is as real as a meadow's - this is the only
    // thing in the whole chain that knows a road is a road. It lives inside
    // build() rather than in the caller because a caller can quietly not run
    // it, and then grass grows down the middle of every street.
    MapChunk chunk = makeFlatChunk();
    TextureLayer road = makeLayer(kGrassEffect);
    road.textureId = 3;
    chunk.layers.push_back(road);

    auto nameFor = [](uint32_t texId) -> std::string {
        return texId == 3 ? "Tileset\\Elwynn\\ElwynnCobbleStoneBase.blp" : "";
    };

    wowee::pipeline::ChunkGrassContext paved;
    REQUIRE_FALSE(paved.build(chunk, densityFor, nameFor));
    REQUIRE(evaluateGrass(paved, chunk, 4.0f, 4.0f).suitability == Catch::Approx(0.0f));

    // Without the names it cannot know, and grows - which is the failure this
    // guards, stated the way it actually happened.
    wowee::pipeline::ChunkGrassContext blind;
    REQUIRE(blind.build(chunk, densityFor));
    REQUIRE(evaluateGrass(blind, chunk, 4.0f, 4.0f).suitability == Catch::Approx(1.0f));
}

TEST_CASE("an overlay whose alpha runs out still covers the ground",
          "[grass][terrain]") {
    // The terrain renderer leaves texels an RLE run never reached as fully
    // covered, so that is what the ground looks like. Grass read them as
    // uncovered and grew a full field down every road the renderer had
    // already paved: it has to agree with what is painted, not with its own
    // reading of the data.
    MapChunk chunk = makeFlatChunk();
    chunk.layers.push_back(makeLayer(kGrassEffect));   // base: grass

    // A compressed overlay whose stream stops after a handful of texels.
    const auto offset = static_cast<uint32_t>(chunk.alphaMap.size());
    chunk.alphaMap.push_back(0x80 | 3);   // fill, 4 texels
    chunk.alphaMap.push_back(255);
    TextureLayer overlay = makeLayer(kRoadEffect, 0x100 | 0x200, offset);
    chunk.layers.push_back(overlay);

    wowee::pipeline::ChunkGrassContext context;
    context.build(chunk, densityFor);
    // Well past the four texels the stream actually wrote.
    REQUIRE(evaluateGrass(context, chunk, 4.0f, 4.0f).suitability == Catch::Approx(0.0f));
}

TEST_CASE("a road over dirt grows nothing, though the dirt would",
          "[grass][terrain]") {
    // The shape of a real Elwynn road chunk: grass base, dirt over it, then
    // cobblestone over that. Layers paint over one another, so where the
    // cobblestone is solid it is the only thing there - the dirt beneath it is
    // covered, and covered ground grows nothing whatever it would grow bare.
    //
    // Weighted the other way round the dirt takes the road's weight, and grass
    // comes up through the paving however correctly the cobblestone layer is
    // marked bare. That is what it did.
    MapChunk chunk = makeFlatChunk();
    chunk.layers.push_back(makeLayer(kGrassEffect));           // [0] base grass

    std::vector<uint8_t> solid(wowee::pipeline::ALPHA_MAP_SIZE, 255);
    const uint32_t dirtOffset = appendAlpha(chunk, solid);
    chunk.layers.push_back(makeLayer(kGrassEffect, 0x100, dirtOffset));   // [1] dirt, grows

    const uint32_t roadOffset = appendAlpha(chunk, solid);
    TextureLayer road = makeLayer(kGrassEffect, 0x100, roadOffset);       // [2] cobblestone
    road.textureId = 5;
    chunk.layers.push_back(road);

    auto nameFor = [](uint32_t texId) -> std::string {
        return texId == 5 ? "Tileset\\Elwynn\\ElwynnCobbleStoneBase.blp" : "";
    };

    wowee::pipeline::ChunkGrassContext context;
    context.build(chunk, densityFor, nameFor);
    REQUIRE(evaluateGrass(context, chunk, 4.0f, 4.0f).suitability == Catch::Approx(0.0f));
}
