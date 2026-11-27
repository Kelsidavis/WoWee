#!/usr/bin/env python3
"""Checks the data profiles and the matcher that reads them.

The profiles decide what a phone-sized install contains. A pattern that stops
matching does not fail, it silently drops files, and the client only says so on
a device. This runs without an extraction so it can run in CI.
"""

import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from make_minimal_data import build_filter, load_profiles, matches  # noqa: E402

failures = []


def check(condition, message):
    if not condition:
        failures.append(message)


def test_matcher():
    check(matches("interface/glues/x.blp", ["interface/**"]), "/** should match below it")
    check(matches("interface/x.blp", ["interface/**"]), "/** should match directly below it")
    check(not matches("interfaces/x.blp", ["interface/**"]),
          "interface/** must not match a sibling whose name starts the same")
    check(matches("anything/at/all", ["**"]), "** should match everything")
    check(matches("db/spell.dbc", ["db/*.dbc"]), "fnmatch patterns should work")
    check(not matches("db/sub/spell.dbc", ["db/*.dbc"]),
          "a single * must not cross a separator")
    check(not matches("db/spell.dbc", []), "an empty pattern list matches nothing")


def test_exclude_beats_include():
    profile = {"include": ["interface/**"], "exclude": ["interface/worldmap/**"]}
    keep = build_filter(profile, None)
    check(keep("interface/glues/a.blp"), "an included path should be kept")
    check(not keep("interface/worldmap/a.blp"), "an excluded path should be dropped")
    check(not keep("sound/a.wav"), "a path outside the include list should be dropped")


def test_maps_narrowing():
    profile = {"include": ["world/maps/**", "dbfilesclient/**"]}
    keep = build_filter(profile, ["azeroth"])
    check(keep("world/maps/azeroth/a.adt"), "the named map should be kept")
    check(not keep("world/maps/kalimdor/a.adt"), "an unnamed map should be dropped")
    check(keep("dbfilesclient/spell.dbc"), "narrowing maps must not touch anything else")

    # A profile that carries no world stays that way when maps are named.
    keep = build_filter({"include": ["dbfilesclient/**"]}, ["azeroth"])
    check(not keep("world/maps/azeroth/a.adt"),
          "naming a map must not add terrain to a profile that excludes it")


def test_profiles_file():
    profiles = load_profiles()
    check(bool(profiles), "data_profiles.json defines no profiles")
    for name, profile in profiles.items():
        check(bool(profile.get("include")), "profile %s includes nothing" % name)
        check(bool(profile.get("summary")), "profile %s has no summary" % name)
        reasons = profile.get("exclude_reasons", {})
        for pattern in profile.get("exclude", []):
            check(pattern in reasons,
                  "profile %s excludes %s without saying why. check_profile_coverage.py "
                  "reports an unexplained exclusion as a hole." % (name, pattern))
        for pattern in reasons:
            check(pattern in profile.get("exclude", []),
                  "profile %s explains %s, which it does not exclude" % (name, pattern))

    # The one guarantee worth pinning: every profile carries the DBCs and the
    # fonts. Without either the client has no data and draws no text.
    for name, profile in profiles.items():
        keep = build_filter(profile, None)
        check(keep("dbfilesclient/spell.dbc"), "profile %s drops the DBCs" % name)
        check(keep("fonts/frizqt__.ttf"), "profile %s drops the fonts" % name)


def test_profile_paths_exist():
    """Every include pattern names a directory an extraction really has.

    A pattern for a directory that is not there matches nothing, and fails
    quietly: the profile carries less than it says, and the files it was
    written to keep are dropped without a word. It happened here - the
    patterns said db/** and misc/fonts/**, and an extraction puts the DBC
    tables in dbfilesclient/ and the fonts in fonts/, so every profile
    dropped every table the client reads and the coverage checker called
    it "dropped from directories this profile does not carry, as
    intended".

    The names below are what asset_extract writes: the WoW path each file
    had in the archive, lowercased.
    """
    present = {
        "dbfilesclient/spell.dbc",
        "fonts/frizqt__.ttf",
        "interface/icons/ability.blp",
        "character/human/female/humanfemale.m2",
        "environments/effects/effects.blp",
        "item/icon/inv_misc_note_01.blp",
        "creature/bear/bearskin.blp",
        "spells/levelup/levelup.m2",
        "world/generic/gnome/passive doodads/subway/subwaycar.m2",
        "textures/minimap/azeroth_0_0.blp",
        "dungeons/textures/doodads/stormwindlampglass.blp",
        "tileset/general/general.til",
        "xtextures/blacktip/blacktip.blp",
        "particles/lootfx.m2",
        "textures/guildemblems/background_49_tl_u.blp",
    }
    for name, profile in load_profiles().items():
        for pattern in profile.get("include", []):
            check(any(matches(path, [pattern]) for path in present),
                  "profile %s includes %s, which no extracted file is under"
                  % (name, pattern))


def main():
    for test in (test_matcher, test_exclude_beats_include, test_maps_narrowing,
                 test_profiles_file, test_profile_paths_exist):
        test()
    if failures:
        print("%d failure(s):" % len(failures))
        for message in failures:
            print("  %s" % message)
        return 1
    print("data profiles and the matcher behave.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
