// Choosing a geoset when the model does not have the one asked for.
//
// The rule is one line and it was written five times, as a local lambda in each
// place that needed it, and only some of them knew the exception. That produced
// three faults in one evening: a clean-shaven NPC with a beard, a character
// wearing an untextured cloak it did not own, and a player with no feet.
#include <catch_amalgamated.hpp>
#include "core/geoset_rules.hpp"

using namespace wowee::core;

TEST_CASE("a geoset id is a group and a variant", "[geoset]") {
    CHECK(geosetGroup(501) == 5);
    CHECK(geosetVariant(501) == 1);
    CHECK(geosetGroup(2001) == 20);
    CHECK(geosetVariant(2001) == 1);
    CHECK(geosetGroup(1506) == 15);
    CHECK(geosetVariant(1506) == 6);
}

TEST_CASE("none has two spellings and both are none", "[geoset]") {
    // The geoset tables say variant 1: bare feet, no cloak, no beard.
    CHECK(geosetMeansNone(501));
    CHECK(geosetMeansNone(1501));
    CHECK(geosetMeansNone(101));
    // A DBC that stores the variant directly says 0, and it arrives as x00.
    CHECK(geosetMeansNone(200));
    CHECK(geosetMeansNone(300));
    // Anything else is a thing the character has.
    CHECK_FALSE(geosetMeansNone(1502));
    CHECK_FALSE(geosetMeansNone(505));
    CHECK_FALSE(geosetMeansNone(202));
}

TEST_CASE("resolving against what a model actually carries", "[geoset]") {
    SECTION("the exact geoset wins whenever the model has it") {
        std::unordered_set<uint16_t> model{501, 502, 503};
        CHECK(resolveGeoset(502, model) == 502);
        CHECK(resolveGeoset(501, model) == 501);
    }

    SECTION("a missing variant falls back within its group") {
        // Five kinds of boot, and the one asked for is not among them.
        std::unordered_set<uint16_t> model{502, 503, 505};
        CHECK(resolveGeoset(504, model) == 502);
    }

    SECTION("none never falls back - this is the whole point") {
        // The HD models carry no 1501, the "no cloak" panel. Falling back
        // inside group 15 hands a cloak to a character wearing none, and with
        // no cloak texture bound, a white sheet.
        std::unordered_set<uint16_t> hdModel{1502, 1503, 1504, 1505, 1506};
        CHECK(resolveGeoset(1501, hdModel) == 0);

        // CharFacialHairStyles stores 0 for a character with no beard, which
        // arrives as group*100. Every other member of group 2 is facial hair.
        std::unordered_set<uint16_t> bearded{201, 202, 203};
        CHECK(resolveGeoset(200, bearded) == 0);
    }

    SECTION("a group the model has nothing in draws nothing") {
        std::unordered_set<uint16_t> model{0, 1, 2};
        CHECK(resolveGeoset(1802, model) == 0);
    }

    SECTION("an unknown model is not guessed at") {
        // No geoset list means the model has not been read yet. Answering 0
        // there would hide a part of a character for want of information.
        std::unordered_set<uint16_t> unknown;
        CHECK(resolveGeoset(1501, unknown) == 1501);
        CHECK(resolveGeoset(504, unknown) == 504);
    }

    SECTION("the feet, which is where this was found") {
        // Group 20 is split out of the body on the HD models and they do not
        // agree on which member to use: an HD human female carries 2001 and an
        // HD human male 2002. Asking for either resolves to the one present.
        std::unordered_set<uint16_t> female{2001};
        std::unordered_set<uint16_t> male{2002};
        // 2002 is variant 2, so it is a thing rather than the absence of one,
        // and substituting the 2001 the model does carry is exactly the fix:
        // ask for feet, get the feet this model spells.
        CHECK(resolveGeoset(2002, female) == 2001);
        CHECK(resolveGeoset(2001, female) == 2001);
        CHECK(resolveGeoset(2002, male) == 2002);
        // 2001 is variant 1 and so reads as none, which is why the caller names
        // both rather than relying on one to find the other.
        CHECK(resolveGeoset(2001, male) == 0);
        // A stock model has no group 20 at all - the feet are part of the body.
        std::unordered_set<uint16_t> stock{0, 1, 501, 1501};
        CHECK(resolveGeoset(2001, stock) == 0);
        CHECK(resolveGeoset(2002, stock) == 0);
    }
}

TEST_CASE("facial hair: zero is none, and none is not an id", "[geoset]") {
    SECTION("a clean-shaven character adds nothing") {
        std::unordered_set<uint16_t> set;
        addFacialHairGeosets(set, 0, 0, 0);
        CHECK(set.empty());
    }

    SECTION("each group takes its own variant") {
        std::unordered_set<uint16_t> set;
        addFacialHairGeosets(set, 2, 3, 4);
        CHECK(set == std::unordered_set<uint16_t>{102, 203, 304});
    }

    SECTION("a character with a beard and no sideburns gets only the beard") {
        // The fault this exists for: 200 + 0 was being asked for, no model
        // carries 200, and the caller substituted the first geoset in group 2 -
        // which is a beard on a character that has none.
        std::unordered_set<uint16_t> set;
        addFacialHairGeosets(set, 0, 5, 0);
        CHECK(set == std::unordered_set<uint16_t>{205});
        CHECK(set.count(200) == 0);
        CHECK(set.count(100) == 0);
        CHECK(set.count(300) == 0);
    }
}

TEST_CASE("the appearance key packs three bytes and nothing else", "[geoset]") {
    // Ten hand-written copies of this expression had to agree bit for bit. A
    // lookup that misses because one of them did not does not report a fault -
    // it quietly draws the default hair, or the default beard.
    CHECK(appearanceKey(1, 1, 5) == ((1u << 16) | (1u << 8) | 5u));
    CHECK(appearanceKey(0, 0, 0) == 0u);

    SECTION("no two characters share a key") {
        CHECK(appearanceKey(1, 0, 3) != appearanceKey(1, 1, 3));   // sex
        CHECK(appearanceKey(1, 1, 3) != appearanceKey(2, 1, 3));   // race
        CHECK(appearanceKey(1, 1, 3) != appearanceKey(1, 1, 4));   // variation
    }

    SECTION("a byte in one field cannot reach another") {
        // Race 0 with variation 255 must not collide with race 0 sex 255.
        CHECK(appearanceKey(0, 0, 255) != appearanceKey(0, 255, 0));
        CHECK(appearanceKey(255, 0, 0) != appearanceKey(0, 255, 0));
    }
}

TEST_CASE("the bare set a character shows with nothing equipped", "[geoset]") {
    const auto bare = bareCharacterGeosets(1, 1, 1, 1);

    SECTION("the body and the chosen scalp") {
        CHECK(bare.count(0) == 1);
        CHECK(bare.count(1) == 1);
    }

    SECTION("both spellings of the feet") {
        // The models the game shipped have no group 20 at all and are
        // unaffected; the replacements split the feet out and do not agree on
        // the number, so naming one loses them on half of them.
        CHECK(bare.count(kGeosetBareFeet) == 1);
        CHECK(bare.count(kGeosetBareFeetAlt) == 1);
    }

    SECTION("no cloak of any kind") {
        // Built before equipment is known. Naming the cloak mesh gives an
        // untextured cape to a character wearing none; naming the no-cloak
        // panel is wrong on a model that has no such panel.
        CHECK(bare.count(kGeosetWithCape) == 0);
        CHECK(bare.count(kGeosetNoCape) == 0);
    }

    SECTION("a clean-shaven character adds no facial geoset") {
        const auto shaven = bareCharacterGeosets(1, 0, 0, 0);
        for (uint16_t id : shaven) {
            const uint16_t group = geosetGroup(id);
            CHECK((group < 1 || group > 3));
        }
    }
}

TEST_CASE("equipment selects a variant after the bare one", "[geoset]") {
    // ItemDisplayInfo's GeosetGroup columns hold "the Gth variant after bare".
    // A chest with G=2 wants group 8 variant 3.
    CHECK(equippedGeoset(kGeosetBareSleeves, 2) == 803);
    CHECK(equippedGeoset(kGeosetBareShins, 1) == 502);
    CHECK(equippedGeoset(kGeosetBarePants, 4) == 1305);

    SECTION("zero leaves the bare variant") {
        // The caller only applies this when G is non-zero, and the identity is
        // what makes that safe to read.
        CHECK(equippedGeoset(kGeosetBareForearms, 0) == kGeosetBareForearms);
    }

    SECTION("it stays inside its group") {
        // A group holds well under a hundred variants, so the arithmetic cannot
        // carry into the next one for any value a table actually holds.
        CHECK(geosetGroup(equippedGeoset(kGeosetBareSleeves, 9)) == 8);
        CHECK(geosetGroup(equippedGeoset(kGeosetBareShins, 9)) == 5);
    }
}

TEST_CASE("a night elf's eyes glow and nobody else's do", "[geoset]") {
    // Group 17 is the eye-glow overlay. The NPC path restored it for race 4 and
    // the player path never added it at all, so a night elf player looked out of
    // pale eyes while every night elf standing beside them glowed.
    const auto nightElf = wowee::core::bareCharacterGeosets(1, 1, 1, 1, 4);
    CHECK(nightElf.count(wowee::core::kGeosetEyeGlow) == 1);

    SECTION("every other playable race has it off") {
        for (uint8_t race : {1, 2, 3, 5, 6, 7, 8, 10, 11}) {
            const auto other = wowee::core::bareCharacterGeosets(1, 1, 1, 1, race);
            CHECK(other.count(wowee::core::kGeosetEyeGlow) == 0);
        }
    }

    SECTION("a caller that does not say the race gets no glow") {
        // The default, which is what a path with no race to hand asks for.
        const auto unknown = wowee::core::bareCharacterGeosets(1, 1, 1, 1);
        CHECK(unknown.count(wowee::core::kGeosetEyeGlow) == 0);
    }
}

TEST_CASE("the ears are the variant that has ears on it", "[geoset]") {
    // Three of the four places that built this set named 702 and the fourth
    // named 701, which is the bare head. The character composed through that
    // one lost its ears.
    const auto bare = wowee::core::bareCharacterGeosets(1, 1, 1, 1, 4);
    CHECK(bare.count(wowee::core::kGeosetDefaultEars) == 1);
    CHECK(wowee::core::kGeosetDefaultEars == 702);
}

TEST_CASE("no cloak group is chosen before the equipment is known", "[geoset]") {
    // Naming the cloak mesh gives an untextured cape to someone wearing none,
    // and naming the no-cloak panel is wrong on the models that have no such
    // panel. The equipment pass decides.
    const auto bare = wowee::core::bareCharacterGeosets(1, 1, 1, 1, 4);
    CHECK(bare.count(wowee::core::kGeosetNoCape) == 0);
    CHECK(bare.count(wowee::core::kGeosetWithCape) == 0);
}

TEST_CASE("the belt's base variant is the waist, not nothing", "[geoset]") {
    // Group 18 is erased and rebuilt on every equipment change. On the older
    // human male the group holds only 1802, so erasing it and adding nothing
    // costs nothing; the Legion model carries 1801, 1802 and 1803, and 1801 is
    // the waist itself - dropped with nothing in its place, the torso floats
    // above the legs.
    CHECK(wowee::core::equipment::kBeltBase == 1801);
    CHECK(geosetGroup(wowee::core::equipment::kBeltBase) == 18);
    // A belt worn is the variant after it, which is what the model carries for
    // a buckle on both asset sets.
    CHECK(equippedGeoset(wowee::core::equipment::kBeltBase, 1) == 1802);
}
