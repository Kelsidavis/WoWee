#pragma once

#include <cstddef>
#include <cstdint>

#include <glm/glm.hpp>

// The GPU-side grass structures, kept apart from the renderer that fills them.
//
// This is a data contract shared with two shaders, not part of the Vulkan
// interface, and separating it means the test that guards the contract needs
// only glm - no device, no context, no Vulkan headers. A layout test that is
// cheap to build is a layout test that keeps being run.

namespace wowee {
namespace rendering {

/// One grass blade, as both the compute cull and the vertex shader read it.
///
/// Matches `GrassBlade` in `assets/shaders/grass_cull.comp.glsl` and
/// `assets/shaders/grass.vert.glsl` (std430). Two vec4s rather than named
/// scalars because std430 rounds a struct's array stride up to its largest
/// member's alignment anyway: sixteen bytes here, so any packing finer than a
/// vec4 buys padding rather than space.
///
/// | offset | field              | meaning                                |
/// |--------|--------------------|----------------------------------------|
/// | 0      | positionHeight.xyz | root world position (render space)      |
/// | 12     | positionHeight.w   | height in yards                         |
/// | 16     | facingWidthPhase.x | facing, radians about +Z                |
/// | 20     | facingWidthPhase.y | width in yards                          |
/// | 24     | facingWidthPhase.z | profile index, as a whole number        |
/// | 28     | facingWidthPhase.w | wind phase seed                         |
/// | 32     | groundShadow.xyz   | terrain's shadow tone under the root     |
/// | 44     | groundShadow.w     | 1 when the tones are real, else 0        |
/// | 48     | groundHighlight.xyz| terrain's highlight tone                 |
/// | 60     | groundHighlight.w  | blade's own fade distance in yards;      |
/// |        |                    | negative when the blade stands in water  |
struct GrassBladeGPU {
    glm::vec4 positionHeight{};
    glm::vec4 facingWidthPhase{};
    glm::vec4 groundShadow{};
    glm::vec4 groundHighlight{};
};

static_assert(sizeof(GrassBladeGPU) == 64,
              "GrassBladeGPU must be 64 bytes to match the std430 GrassBlade");
static_assert(sizeof(GrassBladeGPU) % 16 == 0,
              "std430 rounds the array stride of a vec4-bearing struct up to 16");
static_assert(offsetof(GrassBladeGPU, positionHeight) == 0);
static_assert(offsetof(GrassBladeGPU, facingWidthPhase) == 16);
static_assert(offsetof(GrassBladeGPU, groundShadow) == 32);
static_assert(offsetof(GrassBladeGPU, groundHighlight) == 48);

/// Cull parameters, one per frame in flight.
///
/// Matches `GrassCullUniforms` in `grass_cull.comp.glsl` (std140). Laid out
/// the way `CullUniformsGPU` is in `m2_renderer.hpp`, including reusing
/// `cameraPos.w` to carry the squared distance bound rather than spending a
/// second vec4 on one float.
struct GrassCullUniformsGPU {
    glm::vec4 frustumPlanes[6]{};
    glm::vec4 cameraPos{};
    uint32_t bladeCount = 0;
    /// Bit 1 skips the frustum test, bit 2 the distance test. Driven by
    /// WOWEE_GRASS_NOCULL / WOWEE_GRASS_NODIST, the same bisecting flags the
    /// other renderers use: turn one off and see whether the artifact
    /// survives.
    uint32_t debugFlags = 0;
    uint32_t _pad1 = 0;
    uint32_t _pad2 = 0;
};

static_assert(sizeof(GrassCullUniformsGPU) == 128,
              "GrassCullUniformsGPU must be 128 bytes to match the std140 block");
static_assert(offsetof(GrassCullUniformsGPU, cameraPos) == 96);
static_assert(offsetof(GrassCullUniformsGPU, bladeCount) == 112);

/// One vegetation profile, as the vertex shader reads it.
///
/// Matches `GrassProfile` in `grass.vert.glsl` (std430). Only what the GPU
/// needs: the scales that change a blade's geometry and how many of them
/// there are were already applied when the population was generated, so they
/// never reach the device. The bloom and head anchors are per-profile so a
/// biome can put tulips in one zone and wheat gold in another.
struct GrassProfileGPU {
    glm::vec4 rootColor{};   ///< xyz colour, w unused
    glm::vec4 tipColor{};    ///< xyz colour, w unused
    glm::vec4 params{};      ///< x colourVariation, y stiffness, z bloomChance, w seedChance
    glm::vec4 bloomColorA{}; ///< xyz colour, w unused; blade blends A toward B
    glm::vec4 bloomColorB{};
    glm::vec4 headColorA{};  ///< xyz colour, w unused; seed head range
    glm::vec4 headColorB{};
};

static_assert(sizeof(GrassProfileGPU) == 112,
              "GrassProfileGPU must be 112 bytes to match the std430 GrassProfile");
static_assert(offsetof(GrassProfileGPU, tipColor) == 16);
static_assert(offsetof(GrassProfileGPU, params) == 32);
static_assert(offsetof(GrassProfileGPU, bloomColorA) == 48);
static_assert(offsetof(GrassProfileGPU, bloomColorB) == 64);
static_assert(offsetof(GrassProfileGPU, headColorA) == 80);
static_assert(offsetof(GrassProfileGPU, headColorB) == 96);

} // namespace rendering
} // namespace wowee
