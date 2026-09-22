// Which attack animation to try, and what to fall back to.
//
// Not every model carries every attack, so each weapon situation names an
// ordered chain and the first sequence the model has is the one played. The
// chains were written out twice - in the controller that plays them and in the
// probe that answers what a model can do - under a comment in the probe saying
// its copies "match resolveMeleeAnimId", which nothing enforced. They did
// match, which is the state a pair of tables is in right before one is edited.
//
// A divergence reports nothing: the probe says a model can attack one-handed,
// the controller lands somewhere else in a different chain, and a rogue swings
// a dagger like a mace.
#include <catch_amalgamated.hpp>

#include <algorithm>
#include <set>
#include <vector>

#include "rendering/animation/melee_anim_chains.hpp"

using namespace wowee::rendering::anim;

namespace {

std::vector<uint32_t> chain(MeleeChain kind) {
    const auto span = meleeAnimChain(kind);
    return std::vector<uint32_t>(span.begin(), span.end());
}

constexpr MeleeChain kAll[] = {
    MeleeChain::OneHand,   MeleeChain::TwoHand,       MeleeChain::TwoHandLoose,
    MeleeChain::Dagger,    MeleeChain::Fist,          MeleeChain::Unarmed,
    MeleeChain::OffHand,   MeleeChain::OffHandPierce, MeleeChain::OffHandFist,
    MeleeChain::OffHandUnarmed, MeleeChain::Generic,
};

}  // namespace

TEST_CASE("each chain asks first for what the weapon actually is", "[anim]") {
    // The head of the chain is the whole point; everything after it is a
    // compromise. Getting the head wrong means a model that has the right
    // animation still never plays it.
    CHECK(chain(MeleeChain::OneHand).front() == ATTACK_1H);
    CHECK(chain(MeleeChain::TwoHand).front() == ATTACK_2H);
    CHECK(chain(MeleeChain::TwoHandLoose).front() == ATTACK_2H_LOOSE_PIERCE);
    CHECK(chain(MeleeChain::Dagger).front() == ATTACK_1H_PIERCE);
    CHECK(chain(MeleeChain::Fist).front() == ATTACK_UNARMED);   // 3.3.5 has no fist animations
    CHECK(chain(MeleeChain::Unarmed).front() == ATTACK_UNARMED);
    CHECK(chain(MeleeChain::OffHand).front() == ATTACK_OFF);
    CHECK(chain(MeleeChain::OffHandPierce).front() == ATTACK_OFF_PIERCE);
    CHECK(chain(MeleeChain::OffHandFist).front() == ATTACK_UNARMED_OFF);
    CHECK(chain(MeleeChain::OffHandUnarmed).front() == ATTACK_UNARMED_OFF);
}

TEST_CASE("a dagger never falls back to a swing", "[anim]") {
    // The one distinction a player would notice immediately. A dagger thrusts;
    // ATTACK_2H in this chain would have a rogue swinging it overhead.
    const auto dagger = chain(MeleeChain::Dagger);
    CHECK(std::find(dagger.begin(), dagger.end(), ATTACK_2H) == dagger.end());
    CHECK(std::find(dagger.begin(), dagger.end(), ATTACK_2H_LOOSE) == dagger.end());
}

TEST_CASE("a polearm tries the thrust before the swing", "[anim]") {
    // Order, not membership: both are in the chain, and the pierce has to come
    // first or every polearm swings like a greatsword on a model that has both.
    const auto loose = chain(MeleeChain::TwoHandLoose);
    const auto pierceAt = std::find(loose.begin(), loose.end(), ATTACK_2H_LOOSE_PIERCE);
    const auto swingAt = std::find(loose.begin(), loose.end(), ATTACK_2H_LOOSE);
    REQUIRE(pierceAt != loose.end());
    REQUIRE(swingAt != loose.end());
    CHECK(pierceAt < swingAt);
}

TEST_CASE("every off-hand chain prefers an off-hand animation", "[anim]") {
    // Dual-wielding alternates hands, and an off-hand turn that plays the
    // main-hand animation makes both weapons swing on the same side.
    const MeleeChain offHands[] = {MeleeChain::OffHand, MeleeChain::OffHandPierce,
                                   MeleeChain::OffHandFist, MeleeChain::OffHandUnarmed};
    for (MeleeChain kind : offHands) {
        const auto ids = chain(kind);
        REQUIRE_FALSE(ids.empty());
        const bool offHandFirst = ids.front() == ATTACK_OFF ||
                                  ids.front() == ATTACK_OFF_PIERCE ||
                                  ids.front() == ATTACK_UNARMED_OFF;
        CHECK(offHandFirst);
    }
}

TEST_CASE("no chain is empty and none repeats itself", "[anim]") {
    // A repeat is a wasted probe and a sign the chain was edited by hand in
    // one of the two places it used to live.
    for (MeleeChain kind : kAll) {
        const auto ids = chain(kind);
        INFO("chain " << static_cast<int>(kind));
        REQUIRE_FALSE(ids.empty());
        const std::set<uint32_t> unique(ids.begin(), ids.end());
        CHECK(unique.size() == ids.size());
    }
}

TEST_CASE("every chain ends somewhere a model with nothing can still go",
          "[anim]") {
    // The last entry is what plays when a model carries none of the rest. It
    // has to be one of the two every rig has - an unarmed swing, or a parry,
    // which at least moves the weapon arm.
    for (MeleeChain kind : kAll) {
        const auto ids = chain(kind);
        INFO("chain " << static_cast<int>(kind));
        const uint32_t last = ids.back();
        CHECK((last == ATTACK_UNARMED || last == ATTACK_1H ||
               last == PARRY_1H || last == PARRY_UNARMED));
    }
}

TEST_CASE("the generic chain has no parry in it", "[anim]") {
    // It is what a creature swings when nothing knows its weapon, and it ends
    // at an unarmed attack rather than falling through to a parry: a parry in
    // place of an attack reads as the model flinching.
    const auto ids = chain(MeleeChain::Generic);
    CHECK(std::find(ids.begin(), ids.end(), PARRY_1H) == ids.end());
    CHECK(std::find(ids.begin(), ids.end(), PARRY_UNARMED) == ids.end());
    CHECK(ids.back() == ATTACK_UNARMED);
}
