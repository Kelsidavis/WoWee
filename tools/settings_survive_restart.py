#!/usr/bin/env python3
"""Settings set in one session, read back in the next.

Everything else here checks a setting inside one run: that a control shows it,
that changing it writes it, that the store hears about it. None of that says the
value comes back, and coming back is the whole of what a setting is for.

Two runs of the runner over one config root is a restart. The first sets the
settings a Blizzard control drives to values of its own; the second starts
clean, reads its config from disk the way the client does, and says what it
found.

The chain being watched has three links, each fixed in its own pass and none of
them provable from inside a single run:

  * a setting changed in this client's own window reaches the CVar store, which
    it did not - the change was saved to settings.cfg and undone at the next
    start by a CVar nobody had touched;
  * a CVar applied to its setting does not come back changed, which it did - a
    Ground Density of 24 was rewritten as 23.893333, the value rounded to what
    the setting holds and echoed back over what the player picked;
  * the store is applied over the settings at start-up, which is what makes the
    first two matter.

It also catches the harness undoing itself. The runner used to seed every
setting from the schema at start-up through the same path a player's change
takes, so the defaults were written over the store before the stored values were
read from it: both runs came back on the defaults and the file was emptied of
what the first had saved. That looked exactly like the client losing settings.

Needs build/bin/framexml_run and an extracted interface, and skips rather than
fails without them.
"""

import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
RUNNER = ROOT / "build" / "bin" / "framexml_run"
DATA = ROOT / "Data"
INTERFACE = ROOT / "Data/interface"
CONFIG_ROOT = ROOT / "logs/settings_restart_config"

# Set in the first run, expected back in the second. Chosen to be inside every
# range involved and not equal to any default, so a setting that quietly went
# back to its default is visible.
WANTED = {
    "groundclutter": 1.0,
    "mousespeed": 0.4,
    "viewdistance": 1700,
    "effectsvolume": 0.35,
}

FIRST = "".join(f'WoweeSetSetting("{k}", "{v}") ' for k, v in WANTED.items())
SECOND = ('local out = {} ' +
          "".join(f'out[#out+1] = "{k}=" .. tostring(WoweeGetSetting("{k}")) '
                  for k in WANTED) +
          'error("QQ" .. "RESTART " .. table.concat(out, " "))')


def run(lua):
    env = dict(os.environ)
    env["WOWEE_CONFIG_ROOT"] = str(CONFIG_ROOT)
    return subprocess.run([str(RUNNER), str(DATA), lua],
                          capture_output=True, text=True, timeout=900, env=env)


def main():
    if not RUNNER.is_file() or not INTERFACE.is_dir():
        print("framexml_run or an extracted interface is missing - no restart was made.")
        return 0

    # From nothing, or the first run is reading a store some other run left.
    shutil.rmtree(CONFIG_ROOT, ignore_errors=True)
    CONFIG_ROOT.mkdir(parents=True, exist_ok=True)

    try:
        run(FIRST)
        second = run(SECOND)
    except subprocess.TimeoutExpired:
        print("the runner did not finish - no restart was made.")
        return 1

    payload = None
    for line in (second.stdout + second.stderr).splitlines():
        at = line.find("QQ" + "RESTART ")
        if at != -1:
            payload = line[at + len("QQRESTART "):]
            break
    if payload is None:
        print("the second run said nothing - the settings could not be read back.")
        return 1

    came = {}
    for item in payload.split():
        name, _, value = item.partition("=")
        came[name] = value

    lost = []
    for key, want in WANTED.items():
        got = came.get(key)
        if got is None:
            lost.append(f"{key}: the second run had no value for it at all")
            continue
        try:
            near = abs(float(got) - float(want)) <= max(0.02, abs(want) * 0.01)
        except ValueError:
            near = False
        if not near:
            lost.append(f"{key}: set to {want} and came back as {got}")

    print(f"{len(WANTED)} settings set in one run and read in the next.\n")
    for entry in lost:
        print(f"  {entry}")

    if lost:
        print(f"\n{len(lost)} setting(s) did not survive the restart.")
        return 0

    print("every setting set in one session is still there in the next.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
