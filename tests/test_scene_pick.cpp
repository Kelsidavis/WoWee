// Which of ScenePick's answers a caller wants.
//
// The struct carries several, and two of them are easy to confuse: closestGuid
// is the nearest *entry point* of anything the ray touched, and resolve() is
// what a click acts on. They disagree exactly where it matters. A game object's
// fallback sphere is 2.5 yards against a unit's 1.8, so a wide object beside an
// NPC is entered first even when the NPC's centre is nearer - which is why
// kMaxGameObjectPickRadius exists at all, and why the vendor cursor, asking
// closestGuid, lost every argument to the stall a merchant stands behind.
//
// Nothing raises when a caller picks the wrong one. It just quietly describes a
// different thing from the one the click will hit.
#include <catch_amalgamated.hpp>

#include <cstdint>

#include "ui/scene_pick.hpp"

using wowee::ui::ScenePick;

namespace {
constexpr uint64_t kVendor = 0xF130000001u;
constexpr uint64_t kStall  = 0xF110000002u;
constexpr uint64_t kWolf   = 0xF130000003u;
constexpr uint64_t kCorpse = 0xF130000004u;
}  // namespace

TEST_CASE("a unit behind a wider object still wins the click") {
    // The vendor's centre is nearer, but the stall's bigger sphere is entered
    // first - so closestGuid names the stall and resolve() names the vendor.
    ScenePick pick;
    pick.closestGuid = kStall;      // entered at 8.0
    pick.closestT = 8.0f;
    pick.livingUnitGuid = kVendor;
    pick.livingUnitCenterT = 10.0f;
    pick.objectGuid = kStall;
    pick.objectCenterT = 10.5f;

    CHECK(pick.resolve() == kVendor);
    CHECK(pick.closestGuid != pick.resolve());
}

TEST_CASE("an object clearly in front of the unit takes it") {
    // Clearly is two yards. A mailbox between the player and an NPC behind it
    // is what the player means to click.
    ScenePick pick;
    pick.livingUnitGuid = kVendor;
    pick.livingUnitCenterT = 14.0f;
    pick.objectGuid = kStall;
    pick.objectCenterT = 11.0f;
    CHECK(pick.resolve() == kStall);

    // Just inside the bias, and the unit keeps it.
    pick.objectCenterT = 12.5f;
    CHECK(pick.resolve() == kVendor);
}

TEST_CASE("a hostile outranks whatever else is under the cursor") {
    ScenePick pick;
    pick.livingUnitGuid = kVendor;
    pick.livingUnitCenterT = 6.0f;
    pick.hostileUnitGuid = kWolf;
    pick.hostileUnitT = 9.0f;
    CHECK(pick.resolve() == kWolf);
}

TEST_CASE("a corpse is picked only when nothing living is there") {
    ScenePick pick;
    pick.deadUnitGuid = kCorpse;
    pick.deadUnitCenterT = 5.0f;
    CHECK(pick.resolve() == kCorpse);

    // Someone standing on the body takes it back, however the distances fall.
    pick.livingUnitGuid = kVendor;
    pick.livingUnitCenterT = 7.0f;
    CHECK(pick.resolve() == kVendor);
}

TEST_CASE("an empty pick resolves to nothing") {
    ScenePick pick;
    CHECK(pick.resolve() == 0u);
    CHECK(pick.unitGuid() == 0u);
}
