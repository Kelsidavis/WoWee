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
/// | 24     | facingWidthPhase.z | tilt (unused until Phase 5)             |
/// | 28     | facingWidthPhase.w | wind phase seed (unused until Phase 5)  |
///
/// The last two are carried now rather than added later so the stride does not
/// change once real population data exists.
struct GrassBladeGPU {
    glm::vec4 positionHeight{};
    glm::vec4 facingWidthPhase{};
};

static_assert(sizeof(GrassBladeGPU) == 32,
              "GrassBladeGPU must be 32 bytes to match the std430 GrassBlade");
static_assert(sizeof(GrassBladeGPU) % 16 == 0,
              "std430 rounds the array stride of a vec4-bearing struct up to 16");
static_assert(offsetof(GrassBladeGPU, positionHeight) == 0);
static_assert(offsetof(GrassBladeGPU, facingWidthPhase) == 16);

/// Cull parameters, one per frame in flight.
///
/// Matches `GrassCullUniforms` in `grass_cull.comp.glsl` (std140). Laid out
/// the way `CullUniformsGPU` is in `m2_renderer.hpp`, including reusing
/// `cameraPos.w` to carry the squared distance bound rather than spending a
/// second vec4 on one float.
struct GrassCullUniformsGPU {
    glm::vec4 frustumPlanes[6]{};
    glm::vec4 cameraPos{};
    /// xyz = where the test field is planted; w unused. Blades are generated
    /// around a local origin and placed by this, so moving the field costs a
    /// uniform write rather than regenerating and re-uploading the buffer.
    /// Phase 3 generates real world positions per tile and drops it.
    glm::vec4 fieldOrigin{};
    uint32_t bladeCount = 0;
    uint32_t _pad0 = 0;
    uint32_t _pad1 = 0;
    uint32_t _pad2 = 0;
};

static_assert(sizeof(GrassCullUniformsGPU) == 144,
              "GrassCullUniformsGPU must be 144 bytes to match the std140 block");
static_assert(offsetof(GrassCullUniformsGPU, cameraPos) == 96);
static_assert(offsetof(GrassCullUniformsGPU, fieldOrigin) == 112);
static_assert(offsetof(GrassCullUniformsGPU, bladeCount) == 128);

/// Pushed to the vertex shader, which needs the same origin the cull used but
/// does not see the cull's uniform block.
struct GrassPushConstants {
    glm::vec4 fieldOrigin{};
};
static_assert(sizeof(GrassPushConstants) == 16);

} // namespace rendering
} // namespace wowee
