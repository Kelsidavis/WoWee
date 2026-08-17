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
/// | 32     | groundColor.xyz    | terrain colour under the root           |
/// | 44     | groundColor.w      | 1 when that colour is real, else 0      |
struct GrassBladeGPU {
    glm::vec4 positionHeight{};
    glm::vec4 facingWidthPhase{};
    glm::vec4 groundColor{};
};

static_assert(sizeof(GrassBladeGPU) == 48,
              "GrassBladeGPU must be 48 bytes to match the std430 GrassBlade");
static_assert(sizeof(GrassBladeGPU) % 16 == 0,
              "std430 rounds the array stride of a vec4-bearing struct up to 16");
static_assert(offsetof(GrassBladeGPU, positionHeight) == 0);
static_assert(offsetof(GrassBladeGPU, facingWidthPhase) == 16);
static_assert(offsetof(GrassBladeGPU, groundColor) == 32);

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
    uint32_t _pad0 = 0;
    uint32_t _pad1 = 0;
    uint32_t _pad2 = 0;
};

static_assert(sizeof(GrassCullUniformsGPU) == 128,
              "GrassCullUniformsGPU must be 128 bytes to match the std140 block");
static_assert(offsetof(GrassCullUniformsGPU, cameraPos) == 96);
static_assert(offsetof(GrassCullUniformsGPU, bladeCount) == 112);

/// One vegetation profile, as the vertex and fragment shaders read it.
///
/// Matches `GrassProfile` in `grass.vert.glsl` and `grass.frag.glsl` (std430).
/// Only what the GPU needs: the scales that change a blade's geometry and how
/// many of them there are were already applied when the population was
/// generated, so they never reach the device.
struct GrassProfileGPU {
    glm::vec4 rootColor{};   ///< xyz colour, w unused
    glm::vec4 tipColor{};    ///< xyz colour, w unused
    glm::vec4 params{};      ///< x colourVariation, y stiffness, z bloomChance, w seedChance
};

static_assert(sizeof(GrassProfileGPU) == 48,
              "GrassProfileGPU must be 48 bytes to match the std430 GrassProfile");
static_assert(offsetof(GrassProfileGPU, tipColor) == 16);
static_assert(offsetof(GrassProfileGPU, params) == 32);

} // namespace rendering
} // namespace wowee
