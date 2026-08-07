#include <catch_amalgamated.hpp>

#include "rendering/m2_model_classifier.hpp"

#include <string>

using wowee::rendering::classifyM2Model;

namespace {

// Bounds and counts stand in for a mid-size doodad. None of the cases below
// turn on geometry — they are all decided by the name — but the classifier
// needs something plausible to reason about.
wowee::rendering::M2ClassificationResult classify(const std::string& name,
                                                  float horiz = 3.0f,
                                                  float vert = 3.0f) {
    return classifyM2Model(name,
                           glm::vec3(-horiz * 0.5f, -horiz * 0.5f, 0.0f),
                           glm::vec3(horiz * 0.5f, horiz * 0.5f, vert),
                           500, 0);
}

} // namespace

// Foliage tokens are substring-matched because model names concatenate words
// with no separator, so a short token can land inside an unrelated word. Every
// name here is a real asset that swayed in the wind because of it.
TEST_CASE("rigid props whose names contain a foliage token do not sway",
          "[m2][classifier][foliage]") {
    SECTION("zone name inside the filename") {
        // "thorn" inside Stranglethorn — the Stranglethorn Vale troll ruins.
        for (const char* n : {"STRANGLETHORNRUINS01", "StranglethornRuins14",
                              "stranglethornruins21", "StranglethornRuins_Pylon",
                              "StranglethornCliffRock02"}) {
            INFO(n);
            CHECK_FALSE(classify(n).isFoliageLike);
        }
    }

    SECTION("token inside a longer structural word") {
        // "corn" in Corner, "hops" in ShopSign, "crop" in Outcrop,
        // "herb" in Herbalism, "tree" in StreetSign, "melon"/"vine" likewise.
        CHECK_FALSE(classify("Azjol_Wall_Corner").isFoliageLike);
        CHECK_FALSE(classify("AquaductStone_Corner1").isFoliageLike);
        CHECK_FALSE(classify("ZulDrak_Ruin_CornerTall01").isFoliageLike);
        CHECK_FALSE(classify("WineShopSign01").isFoliageLike);
        CHECK_FALSE(classify("HumanMagicShopSign").isFoliageLike);
        CHECK_FALSE(classify("AeriePeaksRockOutcrop01").isFoliageLike);
        CHECK_FALSE(classify("BE_Signs_Herbalism").isFoliageLike);
        CHECK_FALSE(classify("Dwarfsign_Herbalist").isFoliageLike);
        CHECK_FALSE(classify("GnomeStreetSign01").isFoliageLike);
        CHECK_FALSE(classify("UldamanStreetSign").isFoliageLike);
        CHECK_FALSE(classify("DivineShield_Low_Base").isFoliageLike);
    }

    SECTION("full paths are matched on the basename") {
        CHECK_FALSE(classify("WORLD\\AZEROTH\\STRANGLETHORN\\PASSIVEDOODADS"
                             "\\RUINS\\STRANGLETHORNRUINS03.M2").isFoliageLike);
    }
}

// The fix ranks matches by where they end, so a structural word only wins when
// it comes last. These names carry one too, and are still plants.
TEST_CASE("plants keep swaying when a structural word comes first",
          "[m2][classifier][foliage]") {
    SECTION("place name first, plant last") {
        for (const char* n : {"DustwallowTree04", "DustwallowBush01",
                              "DustwallowShrub03", "StoneTree06",
                              "BurntStoneTree07", "DeadwindPassRockTree02",
                              "AO_BridgeTree01"}) {
            INFO(n);
            CHECK(classify(n, 8.0f, 9.0f).isFoliageLike);
        }
    }

    SECTION("ordinary foliage is unaffected") {
        for (const char* n : {"StranglethornFern01", "StranglethornPlant02",
                              "ElwynnMelon01", "G_Watermelon",
                              "NorthshireBush01", "DesolaceCactus02"}) {
            INFO(n);
            CHECK(classify(n).isFoliageLike);
        }
    }
}

// isFoliageLike also drives collision and animation, so a false positive did
// more than add wind: it made ruins walk-through and froze their animation.
TEST_CASE("a ruin misread as foliage would also lose its collision",
          "[m2][classifier][foliage]") {
    const auto ruin = classify("StranglethornRuins07", 9.0f, 7.0f);
    CHECK_FALSE(ruin.isFoliageLike);
    CHECK_FALSE(ruin.collisionNoBlock);
    CHECK_FALSE(ruin.disableAnimation);

    const auto tree = classify("DustwallowTree04", 9.0f, 7.0f);
    CHECK(tree.isFoliageLike);
    CHECK(tree.disableAnimation);
}

// isForge forces the batch additive, so a false positive renders a solid model
// as glowing translucent VFX — the same failure the Steam Tank had.
TEST_CASE("only an actual forge is treated as forge fire", "[m2][classifier][forge]") {
    SECTION("the city of Ironforge is not a forge") {
        for (const char* n : {"IronforgeBench_Average01", "IronforgeStatue_01",
                              "IronforgeCliff01", "IronforgeElevator",
                              "IronforgeHangingLantern01", "IronforgeBanner01",
                              "IronforgeSignpost", "ironforgepiston"}) {
            INFO(n);
            CHECK_FALSE(classify(n).isForge);
        }
    }

    SECTION("nor is a part of one, or a panel that controls one") {
        CHECK_FALSE(classify("Dalaran_ForgeArms").isForge);
        CHECK_FALSE(classify("Dalaran_ForgeSmelter").isForge);
        CHECK_FALSE(classify("BU_CrystalForgeController").isForge);
        CHECK_FALSE(classify("UL_Forge_Iron_Press").isForge);
    }

    SECTION("real forges still are, with or without a numeric suffix") {
        for (const char* n : {"DR_Forge_01", "OM_Forge_01", "BU_Forge_01",
                              "ID_Forge", "TS_Forge_01", "BE_Forge01",
                              "Dalaran_Forge", "SC_RuneForge_02",
                              "ET_CrystalForge", "BlacksmithForge",
                              "DarkIronForge", "Wolvar_Forge",
                              "WORLD\\GENERIC\\HUMAN\\FORGE\\ID_FORGE.M2"}) {
            INFO(n);
            CHECK(classify(n).isForge);
        }
    }

    SECTION("forge lava stays excluded as before") {
        CHECK_FALSE(classify("UL_ForgeLava").isForge);
    }
}

TEST_CASE("Blizzard's own misspellings are foliage too", "[m2][classifier]") {
    // WETLANDSSHURB09.M2 is a real path — "shurb", not "shrub" — and it is the
    // only spelling those models have. Reported as grass with cobwebs on it
    // that the player could not walk through.
    //
    // The correct spelling stays covered beside it, because adding the typo is
    // the kind of change that invites someone to "fix" the list later.
    for (const char* path : {
             "WORLD\\KHAZMODAN\\WETLANDS\\PASSIVEDOODADS\\BUSHES\\WETLANDSSHURB09.M2",
             "WORLD\\AZEROTH\\ELWYNN\\PASSIVEDOODADS\\BUSHES\\ELWYNNSHRUB01.M2"}) {
        const auto cls = classify(path);
        INFO(path);
        CHECK(cls.collisionNoBlock);
    }
}
