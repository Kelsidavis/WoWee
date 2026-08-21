#include <catch_amalgamated.hpp>

#include "game/warden_module.hpp"

using wowee::game::wardenAbsoluteRelocTarget;
using wowee::game::wardenRelocTargetFits;

// A Warden module arrives from the server. Its relocation entries name offsets
// this client then writes four bytes at, so both of these decide where an
// attacker-supplied number is allowed to point.

TEST_CASE("An absolute relocation entry drops its form flag", "[warden]") {
    // The top bit says which of the two entry forms this is; it is not part of
    // the offset. Kept, every absolute entry came out at or above 0x80000000 -
    // past the end of any image the parser accepts - so the bounds check below
    // rejected all of them and no module carrying one could relocate at all.
    CHECK(wardenAbsoluteRelocTarget(0x80, 0x00, 0x00, 0x10) == 0x10u);
    CHECK(wardenAbsoluteRelocTarget(0xFF, 0xFF, 0xFF, 0xFF) == 0x7FFFFFFFu);
    // And the low 31 bits still carry, so a real offset survives intact.
    CHECK(wardenAbsoluteRelocTarget(0x80, 0x01, 0x23, 0x45) == 0x00012345u);
}

TEST_CASE("A relocation target is measured without wrapping", "[warden]") {
    constexpr size_t kImage = 49152;   // the size the reported module carried

    // The four values that used to get through. target + 4 in 32 bits wraps to
    // 0..3, which compares below the image size, so the check passed and the
    // write landed at image + 0xFFFFFFFF.
    CHECK_FALSE(wardenRelocTargetFits(0xFFFFFFFFu, kImage));
    CHECK_FALSE(wardenRelocTargetFits(0xFFFFFFFEu, kImage));
    CHECK_FALSE(wardenRelocTargetFits(0xFFFFFFFDu, kImage));
    CHECK_FALSE(wardenRelocTargetFits(0xFFFFFFFCu, kImage));

    // The ordinary refusals still refuse.
    CHECK_FALSE(wardenRelocTargetFits(0x80000010u, kImage));
    CHECK_FALSE(wardenRelocTargetFits(static_cast<uint32_t>(kImage), kImage));

    // The last target that leaves room for its own four bytes, and the first
    // that does not.
    CHECK(wardenRelocTargetFits(static_cast<uint32_t>(kImage) - 4, kImage));
    CHECK_FALSE(wardenRelocTargetFits(static_cast<uint32_t>(kImage) - 3, kImage));
    CHECK(wardenRelocTargetFits(0, kImage));
}
