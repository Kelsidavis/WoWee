#!/usr/bin/env python3
"""Settings the player cannot reach from any control.

A setting is bound to a field, applied, saved and loaded whether or not
anything can change it. Three things give a player a way to set one: a schema
row, which draws a control on this client's own panel; an entry in kClientCVars,
which maps a Blizzard control onto it; and a graphics preset, which sets several
at once. A setting with none of the three can only be changed by editing
settings.cfg by hand.

Three are in that state deliberately and are listed below with the reason. Any
other is a setting that was added without a way to use it.

Canaried by removing a schema row, which reports that key.
"""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PANEL = ROOT / "src/ui/settings_panel.cpp"
SCHEMA = ROOT / "src/ui/settings_schema.cpp"
CVARS = ROOT / "src/addons/lua_system_api.cpp"

# Reachable by nothing, on purpose. Each names the file that says why.
KNOWN = {
    "shadows": "no control: turning shadows off loses the GPU (settings_schema.cpp)",
    "waterrefraction": "not a choice: the shoreline and underwater work assume it "
                       "(settings_schema.cpp)",
    "brightness": "driven by the video panel's brightness slider through SetGamma "
                  "(lua_system_api.cpp)",
}

# Floors, so a regex that stops matching reports an empty world rather than a
# clean one.
MIN_BOUND, MIN_ROWS, MIN_CVARS = 60, 60, 5


def main():
    for path in (PANEL, SCHEMA, CVARS):
        if not path.is_file():
            print(f"{path.name} is missing - nothing checked.")
            return 1

    bound = set(re.findall(r'\{\.key = "([a-z0-9_]+)"', PANEL.read_text()))
    rows = set(re.findall(r'\{(?:\.\w+\s*=\s*)?"([a-z0-9_]+)", "[^"]*", SettingKind::', SCHEMA.read_text()))

    text = CVARS.read_text()
    at = text.find("kClientCVars")
    block = text[at:at + 6000] if at != -1 else ""
    driven = set(re.findall(r'\{(?:\.\w+\s*=\s*)?"[a-z0-9_]+",\s*(?:\.\w+\s*=\s*)?"([a-z0-9_]+)"', block))

    if len(bound) < MIN_BOUND or len(rows) < MIN_ROWS or len(driven) < MIN_CVARS:
        print(f"parsed {len(bound)} bound settings, {len(rows)} schema rows and "
              f"{len(driven)} CVar bindings, which is fewer than there are - the "
              "parse stopped matching rather than finding nothing wrong.")
        return 1

    orphans = sorted(bound - rows - driven)
    unexpected = [k for k in orphans if k not in KNOWN]
    missing = [k for k in KNOWN if k not in orphans]

    print(f"{len(bound)} settings bound to a field, {len(orphans)} reachable by "
          "no control.\n")
    for key in orphans:
        print(f"  {key}: {KNOWN.get(key, 'NO CONTROL AND NO REASON GIVEN')}")

    if unexpected:
        print(f"\n{len(unexpected)} setting(s) can only be changed by editing "
              "settings.cfg. Give each a schema row, a kClientCVars entry, or an "
              "entry in this script saying why it has neither.")
        return 0

    if missing:
        print(f"\n{len(missing)} setting(s) named here as unreachable now have a "
              "control: " + ", ".join(sorted(missing)) + ". Remove them from KNOWN.")
        return 0

    print("every setting a player can change has something to change it with.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
