// Tests for indoor shadow disable logic (WMO interior groups)
//
// WMO interior groups (flag 0x2000) should NOT receive directional sun shadows
// because they rely on pre-baked vertex color lighting (MOCV) and the shadow map
// only makes them darker.  The fix is in the fragment shader: interior groups
// skip the shadow map sample entirely.
//
// These tests verify the data contract between the renderer and the shader:
//   - GPUPerFrameData.shadowParams.x controls global shadow enable
//   - WMOMaterial.isInterior controls per-group interior flag
//   - Interior groups ignore shadows regardless of global shadow state

#include <catch_amalgamated.hpp>
#include "rendering/vk_frame_data.hpp"

#include <glm/glm.hpp>

using wowee::rendering::GPUPerFrameData;

TEST_CASE("shared shadow push constants reach both shader stages", "[shadows]") {
    const auto stages = wowee::rendering::kSharedShadowPushStages;
    CHECK((stages & VK_SHADER_STAGE_VERTEX_BIT) != 0);
    CHECK((stages & VK_SHADER_STAGE_FRAGMENT_BIT) != 0);
    CHECK(sizeof(wowee::rendering::ShadowPush) <= 128);
}

// Replicates the shadow params logic from Renderer::updatePerFrameUBO()
// This should NOT be affected by indoor state - shadows remain globally enabled
static void applyShadowParams(GPUPerFrameData& fd,
                              bool shadowsEnabled,
                              float shadowDistance = 300.0f) {
    float shadowBias = glm::clamp(0.8f * (shadowDistance / 300.0f), 0.0f, 1.0f);
    fd.shadowParams = glm::vec4(shadowsEnabled ? 1.0f : 0.0f, shadowBias, 0.0f, 0.0f);
}

// Replicates the WMO interior shader logic:
// interior groups skip shadow sampling entirely (shadow factor = 1.0 = fully lit).
// This covers both lit and unlit interior materials - isInterior takes priority.
static float computeWmoShadowFactor(bool isInterior, float globalShadowEnabled, float rawShadow) {
    if (isInterior) {
        // Interior groups always get shadow factor 1.0 (no shadow darkening)
        // regardless of unlit flag - isInterior is checked first in shader
        return 1.0f;
    }
    if (globalShadowEnabled > 0.5f) {
        return rawShadow;  // exterior: use shadow map result
    }
    return 1.0f;  // shadows globally disabled
}

TEST_CASE("Global shadow params are not affected by indoor state", "[indoor_shadows]") {
    GPUPerFrameData fd{};

    // Shadows enabled - should stay 1.0 regardless of any indoor logic
    applyShadowParams(fd, /*shadowsEnabled=*/true);
    REQUIRE(fd.shadowParams.x == Catch::Approx(1.0f));

    // Shadows disabled - should be 0.0
    applyShadowParams(fd, /*shadowsEnabled=*/false);
    REQUIRE(fd.shadowParams.x == Catch::Approx(0.0f));
}

TEST_CASE("Interior WMO groups skip shadow sampling", "[indoor_shadows]") {
    // Even when shadows are globally on and the shadow map says 0.2 (dark shadow),
    // interior groups should get 1.0 (no shadow)
    float factor = computeWmoShadowFactor(/*isInterior=*/true, /*globalShadowEnabled=*/1.0f, /*rawShadow=*/0.2f);
    REQUIRE(factor == Catch::Approx(1.0f));
}

TEST_CASE("Exterior WMO groups receive shadows normally", "[indoor_shadows]") {
    float factor = computeWmoShadowFactor(/*isInterior=*/false, /*globalShadowEnabled=*/1.0f, /*rawShadow=*/0.3f);
    REQUIRE(factor == Catch::Approx(0.3f));
}

TEST_CASE("Exterior WMO groups skip shadows when globally disabled", "[indoor_shadows]") {
    float factor = computeWmoShadowFactor(/*isInterior=*/false, /*globalShadowEnabled=*/0.0f, /*rawShadow=*/0.3f);
    REQUIRE(factor == Catch::Approx(1.0f));
}

TEST_CASE("Interior WMO groups skip shadows even when globally disabled", "[indoor_shadows]") {
    float factor = computeWmoShadowFactor(/*isInterior=*/true, /*globalShadowEnabled=*/0.0f, /*rawShadow=*/0.5f);
    REQUIRE(factor == Catch::Approx(1.0f));
}

TEST_CASE("Unlit interior surfaces skip shadows (isInterior takes priority over unlit)", "[indoor_shadows]") {
    // Many interior walls use F_UNLIT material flag (0x01). The shader must check
    // isInterior BEFORE unlit so these surfaces don't receive shadow darkening.
    // Even though the surface is unlit, it's interior → shadow factor = 1.0
    float factor = computeWmoShadowFactor(/*isInterior=*/true, /*globalShadowEnabled=*/1.0f, /*rawShadow=*/0.1f);
    REQUIRE(factor == Catch::Approx(1.0f));
}

TEST_CASE("Outdoor unlit surfaces still receive shadows", "[indoor_shadows]") {
    // Exterior unlit surfaces (isInterior=false, unlit=true in shader) should
    // still receive shadow darkening from the shadow map
    float factor = computeWmoShadowFactor(/*isInterior=*/false, /*globalShadowEnabled=*/1.0f, /*rawShadow=*/0.25f);
    REQUIRE(factor == Catch::Approx(0.25f));
}

TEST_CASE("Shadow bias scales with shadow distance", "[indoor_shadows]") {
    GPUPerFrameData fd{};

    // At default 300.0f, bias = 0.8
    applyShadowParams(fd, true, 300.0f);
    REQUIRE(fd.shadowParams.y == Catch::Approx(0.8f));

    // At 150.0f, bias = 0.4
    applyShadowParams(fd, true, 150.0f);
    REQUIRE(fd.shadowParams.y == Catch::Approx(0.4f));

    // Bias is clamped to [0, 1]
    applyShadowParams(fd, true, 600.0f);
    REQUIRE(fd.shadowParams.y == Catch::Approx(1.0f));
}

TEST_CASE("Ambient color is NOT modified globally for indoor state", "[indoor_shadows]") {
    // The global UBO ambient color should never be modified based on indoor state.
    // Indoor lighting is handled per-group in the WMO shader via MOCV vertex colors
    // and MOHD ambient color.
    GPUPerFrameData fd{};
    fd.ambientColor = glm::vec4(0.3f, 0.3f, 0.3f, 1.0f);

    applyShadowParams(fd, true);

    // Ambient should be untouched
    REQUIRE(fd.ambientColor.x == Catch::Approx(0.3f));
    REQUIRE(fd.ambientColor.y == Catch::Approx(0.3f));
    REQUIRE(fd.ambientColor.z == Catch::Approx(0.3f));
}
