// The C++ and GLSL views of a grass blade have to agree byte for byte.
//
// Nothing checks this at run time: a mismatch does not fail to compile, does
// not trip validation, and does not crash. It renders the wrong thing - blades
// at the wrong place, or of zero height, or facing one direction - and the
// symptom looks like a bug in the generator rather than in the layout. So the
// agreement is asserted here, member by member, against the offsets written
// into the shader's own comment.
//
// If a member is added to GrassBladeGPU, this test must be updated in the same
// commit as the two shaders that read it.

#include <catch_amalgamated.hpp>

#include <cstddef>

#include "rendering/grass_blade.hpp"

using wowee::rendering::GrassBladeGPU;
using wowee::rendering::GrassCullUniformsGPU;

TEST_CASE("GrassBladeGPU matches the std430 GrassBlade", "[grass]") {
    SECTION("size is the std430 array stride") {
        // Four vec4s. std430 rounds a struct's stride up to its largest
        // member's alignment, which is 16 for a vec4, so 64 is both the packed
        // size and the stride the shader indexes with.
        REQUIRE(sizeof(GrassBladeGPU) == 64);
        REQUIRE(sizeof(GrassBladeGPU) % 16 == 0);
    }

    SECTION("every member sits where the shader reads it") {
        REQUIRE(offsetof(GrassBladeGPU, positionHeight) == 0);
        REQUIRE(offsetof(GrassBladeGPU, facingWidthPhase) == 16);
        REQUIRE(offsetof(GrassBladeGPU, groundShadow) == 32);
        REQUIRE(offsetof(GrassBladeGPU, groundHighlight) == 48);
    }

    SECTION("an array is tightly packed") {
        // The shader indexes blades[] by stride alone. Any padding the compiler
        // inserted between elements would shift every blade after the first.
        GrassBladeGPU blades[4]{};
        const auto* base = reinterpret_cast<const std::byte*>(&blades[0]);
        const auto* second = reinterpret_cast<const std::byte*>(&blades[1]);
        REQUIRE(static_cast<std::size_t>(second - base) == 64);
        REQUIRE(sizeof(blades) == 256);
    }
}

TEST_CASE("GrassCullUniformsGPU matches the std140 uniform block", "[grass]") {
    SECTION("size and trailing padding") {
        // std140: six vec4 planes (96) + camera (16) + a uint and the three
        // pads that round the block to a 16-byte multiple.
        REQUIRE(sizeof(GrassCullUniformsGPU) == 128);
        REQUIRE(sizeof(GrassCullUniformsGPU) % 16 == 0);
    }

    SECTION("every member sits where the shader reads it") {
        REQUIRE(offsetof(GrassCullUniformsGPU, frustumPlanes) == 0);
        REQUIRE(offsetof(GrassCullUniformsGPU, cameraPos) == 96);
        REQUIRE(offsetof(GrassCullUniformsGPU, bladeCount) == 112);
    }

    SECTION("the plane array is tightly packed") {
        GrassCullUniformsGPU u{};
        const auto* p0 = reinterpret_cast<const std::byte*>(&u.frustumPlanes[0]);
        const auto* p1 = reinterpret_cast<const std::byte*>(&u.frustumPlanes[1]);
        REQUIRE(static_cast<std::size_t>(p1 - p0) == 16);
        REQUIRE(sizeof(u.frustumPlanes) == 96);
    }
}
