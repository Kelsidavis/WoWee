// M2 struct layout and field tests (header-only, no loader source)
#include <catch_amalgamated.hpp>
#include "pipeline/m2_loader.hpp"
#include "rendering/m2_model_classifier.hpp"
#include "rendering/m2_renderer_internal.h"
#include <cstring>

using namespace wowee::pipeline;

TEST_CASE("an M2 batch without an authored material is not assumed one-sided", "[m2]") {
    const wowee::rendering::M2ModelGPU::BatchGPU batch;
    CHECK_FALSE(batch.hasAuthoredMaterial);
}

TEST_CASE("Animated glow centers follow their skinned bones", "[m2][lights]") {
    wowee::rendering::M2Instance instance;
    instance.modelMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(10.0f, 0.0f, 0.0f));
    instance.boneMatrices = {
        glm::mat4(1.0f),
        glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, 2.0f))
    };

    wowee::rendering::M2ModelGPU::BatchGPU batch;
    // Average of two rigid vertices: (0,0,0) on bone 0 and (2,0,0)
    // on bone 1. Moving bone 1 upward by two moves their center upward by one.
    batch.lightBoneAnchors.push_back({0, glm::vec4(0.0f, 0.0f, 0.0f, 0.5f)});
    batch.lightBoneAnchors.push_back({1, glm::vec4(1.0f, 0.0f, 0.0f, 0.5f)});

    const glm::vec3 center = wowee::rendering::animatedBatchWorldCenter(instance, batch);
    CHECK(center.x == Catch::Approx(11.0f));
    CHECK(center.y == Catch::Approx(0.0f));
    CHECK(center.z == Catch::Approx(1.0f));
}

TEST_CASE("Hanging lantern light pools project from suspension to animated tip", "[m2][lights]") {
    wowee::rendering::M2Instance instance;
    instance.modelMatrix = glm::mat4(1.0f);
    instance.scale = 1.0f;
    instance.boneMatrices = {
        glm::mat4(1.0f),
        glm::translate(glm::mat4(1.0f), glm::vec3(1.0f, 0.0f, 1.0f))
    };

    wowee::rendering::M2ModelGPU::BatchGPU batch;
    batch.lightBoneAnchors.push_back({1, glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)});
    batch.lightSuspensionBone = 0;
    batch.lightSuspensionPoint = glm::vec3(0.0f, 0.0f, 2.0f);

    CHECK(wowee::rendering::animatedBatchWorldCenter(instance, batch).x ==
          Catch::Approx(1.0f));
    const glm::vec3 pool = wowee::rendering::animatedBatchLightWorldCenter(instance, batch);
    CHECK(pool.x == Catch::Approx(4.0f));
    CHECK(pool.y == Catch::Approx(0.0f));
    CHECK(pool.z == Catch::Approx(1.0f));
}

TEST_CASE("Foliage asset paths never block player movement", "[m2][collision]") {
    const auto grass = wowee::rendering::classifyM2Model(
        "World\\NoDXT\\Detail\\ElwynnGrass01.m2",
        glm::vec3(-1.0f), glm::vec3(1.0f), 100, 0);
    REQUIRE(grass.collisionNoBlock);

    const auto herb = wowee::rendering::classifyM2Model(
        "World\\SkillActivated\\TradeskillNodes\\Bush_Magebloom01.m2",
        glm::vec3(-1.0f), glm::vec3(1.0f), 100, 0);
    REQUIRE(herb.collisionNoBlock);
}

TEST_CASE("Lighthouse beams are classified for distant bone updates", "[m2][animation]") {
    const auto beam = wowee::rendering::classifyM2Model(
        "World\\Generic\\Human\\Passive Doodads\\Stormwind\\Stormwind_LighthouseBeam_01.m2",
        glm::vec3(-1.0f), glm::vec3(1.0f), 100, 0);
    REQUIRE(beam.isLightBeam);
    REQUIRE_FALSE(beam.disableAnimation);
}

TEST_CASE("Ship machinery is classified for distant bone updates", "[m2][animation][transport]") {
    const auto sails = wowee::rendering::classifyM2Model(
        "World\\Generic\\PassiveDoodads\\Ships\\ShipAnimation\\TransportShip_Sails.m2",
        glm::vec3(-40.0f), glm::vec3(40.0f), 161, 0);
    const auto paddle = wowee::rendering::classifyM2Model(
        "World\\Generic\\PassiveDoodads\\Ships\\ShipAnimation\\PaddleWheel\\Icebreaker_Paddlewheel.m2",
        glm::vec3(-14.0f), glm::vec3(14.0f), 2676, 4);
    REQUIRE(sails.isTransportDoodad);
    REQUIRE(paddle.isTransportDoodad);
}

TEST_CASE("Steam Tonk vehicles are not steam VFX", "[m2][classification]") {
    // The TBC/Turtle SteamTonk overlay is a tiny proxy that slips under the
    // low-poly vertex gate; combined with a smoke emitter it was mis-classified
    // as an additive spell effect (glowing translucent). A "tonk"/"tank" name
    // must never be treated as steam VFX regardless of geometry.
    const auto tonkLow = wowee::rendering::classifyM2Model(
        "Creature\\SteamTonk\\SteamTonk.m2",
        glm::vec3(-1.0f), glm::vec3(1.0f), 23, 2);
    REQUIRE_FALSE(tonkLow.isSpellEffect);
    const auto tonkHi = wowee::rendering::classifyM2Model(
        "Creature\\SteamTonk\\SteamTonk.m2",
        glm::vec3(-1.0f), glm::vec3(1.0f), 606, 1);
    REQUIRE_FALSE(tonkHi.isSpellEffect);

    // A genuine low-poly steam emitter is still a spell effect.
    const auto steam = wowee::rendering::classifyM2Model(
        "Spells\\Steam.m2",
        glm::vec3(-1.0f), glm::vec3(1.0f), 40, 2);
    REQUIRE(steam.isSpellEffect);
}

TEST_CASE("M2Sequence fields are default-initialized", "[m2]") {
    M2Sequence seq{};
    REQUIRE(seq.id == 0);
    REQUIRE(seq.duration == 0);
    REQUIRE(seq.movingSpeed == 0.0f);
    REQUIRE(seq.flags == 0);
    REQUIRE(seq.blendTime == 0);
    REQUIRE(seq.boundRadius == 0.0f);
}

TEST_CASE("M2AnimationTrack hasData", "[m2]") {
    M2AnimationTrack track;
    REQUIRE_FALSE(track.hasData());

    track.sequences.push_back({});
    REQUIRE(track.hasData());
}

TEST_CASE("M2AnimationTrack default interpolation", "[m2]") {
    M2AnimationTrack track;
    REQUIRE(track.interpolationType == 0);
    REQUIRE(track.globalSequence == -1);
}

TEST_CASE("M2Bone parent defaults to root", "[m2]") {
    M2Bone bone{};
    bone.parentBone = -1;
    REQUIRE(bone.parentBone == -1);
    REQUIRE(bone.keyBoneId == 0);
}

TEST_CASE("M2Vertex layout", "[m2]") {
    M2Vertex vert{};
    vert.position = glm::vec3(1.0f, 2.0f, 3.0f);
    vert.boneWeights[0] = 255;
    vert.boneWeights[1] = 0;
    vert.boneWeights[2] = 0;
    vert.boneWeights[3] = 0;
    vert.boneIndices[0] = 5;
    vert.normal = glm::vec3(0.0f, 1.0f, 0.0f);
    vert.texCoords[0] = glm::vec2(0.5f, 0.5f);

    REQUIRE(vert.position.x == 1.0f);
    REQUIRE(vert.boneWeights[0] == 255);
    REQUIRE(vert.boneIndices[0] == 5);
    REQUIRE(vert.normal.y == 1.0f);
    REQUIRE(vert.texCoords[0].x == 0.5f);
}

TEST_CASE("M2Texture stores filename", "[m2]") {
    M2Texture tex{};
    tex.type = 1;
    tex.filename = "Creature\\Hogger\\Hogger.blp";
    REQUIRE(tex.filename == "Creature\\Hogger\\Hogger.blp");
}

TEST_CASE("M2Batch submesh fields", "[m2]") {
    M2Batch batch{};
    batch.skinSectionIndex = 3;
    batch.textureCount = 2;
    batch.indexStart = 100;
    batch.indexCount = 300;
    batch.vertexStart = 0;
    batch.vertexCount = 150;
    batch.submeshId = 0;
    batch.submeshLevel = 0;

    REQUIRE(batch.skinSectionIndex == 3);
    REQUIRE(batch.textureCount == 2);
    REQUIRE(batch.indexCount == 300);
    REQUIRE(batch.vertexCount == 150);
}

TEST_CASE("M2Material blend modes", "[m2]") {
    M2Material mat{};
    mat.flags = 0;
    mat.blendMode = 2;  // Alpha blend
    REQUIRE(mat.blendMode == 2);

    mat.blendMode = 0;  // Opaque
    REQUIRE(mat.blendMode == 0);
}

TEST_CASE("M2Model isValid", "[m2]") {
    M2Model model{};
    REQUIRE_FALSE(model.isValid()); // no vertices or indices

    model.vertices.push_back({});
    REQUIRE_FALSE(model.isValid()); // vertices but no indices

    model.indices.push_back(0);
    REQUIRE(model.isValid()); // both present
}

TEST_CASE("M2Model bounding box", "[m2]") {
    M2Model model{};
    model.boundMin = glm::vec3(-1.0f, -2.0f, -3.0f);
    model.boundMax = glm::vec3(1.0f, 2.0f, 3.0f);
    model.boundRadius = 5.0f;

    glm::vec3 center = (model.boundMin + model.boundMax) * 0.5f;
    REQUIRE(center.x == Catch::Approx(0.0f));
    REQUIRE(center.y == Catch::Approx(0.0f));
    REQUIRE(center.z == Catch::Approx(0.0f));
}

TEST_CASE("M2ParticleEmitter defaults", "[m2]") {
    M2ParticleEmitter emitter{};
    emitter.textureRows = 1;
    emitter.textureCols = 1;
    emitter.enabled = true;
    REQUIRE(emitter.textureRows == 1);
    REQUIRE(emitter.textureCols == 1);
    REQUIRE(emitter.enabled);
}

TEST_CASE("M2RibbonEmitter defaults", "[m2]") {
    M2RibbonEmitter ribbon{};
    REQUIRE(ribbon.edgesPerSecond == Catch::Approx(15.0f));
    REQUIRE(ribbon.edgeLifetime == Catch::Approx(0.5f));
    REQUIRE(ribbon.gravity == Catch::Approx(0.0f));
}

TEST_CASE("M2Attachment position", "[m2]") {
    M2Attachment att{};
    att.id = 1;  // Right hand
    att.bone = 42;
    att.position = glm::vec3(0.1f, 0.2f, 0.3f);

    REQUIRE(att.id == 1);
    REQUIRE(att.bone == 42);
    REQUIRE(att.position.z == Catch::Approx(0.3f));
}

TEST_CASE("M2Model collections", "[m2]") {
    M2Model model{};

    // Bones
    model.bones.push_back({});
    model.bones[0].parentBone = -1;
    model.bones[0].pivot = glm::vec3(0, 0, 0);

    // Sequences
    model.sequences.push_back({});
    model.sequences[0].id = 0; // Stand
    model.sequences[0].duration = 1000;

    // Textures
    model.textures.push_back({});
    model.textures[0].type = 0;
    model.textures[0].filename = "test.blp";

    REQUIRE(model.bones.size() == 1);
    REQUIRE(model.sequences.size() == 1);
    REQUIRE(model.textures.size() == 1);
    REQUIRE(model.sequences[0].duration == 1000);
}
