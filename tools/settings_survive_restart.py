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

# The six a Blizzard control drives that have no schema row of their own - they
# are driven by that control instead, which is why they have none. Set in the
# first run and expected back in the second, on values inside every range
# involved and different from the defaults, so a setting that quietly went back
# to one is visible.
#
# Only those six. invertmouse, autoloot and vsync are driven by a CVar and are
# schema rows as well, so the walk below sets them after this does and these
# expectations would be of a value that had since moved on.
WANTED = {
    "groundclutter": 1.0,
    "mousespeed": 0.4,
    "viewdistance": 1700,
    "effectsvolume": 0.35,
    # The bools too. They travel the same chain and are where its arithmetic is
    # least like the numbers': a bool cannot hold half of anything, and the
    # store has to carry the value the setting actually took rather than the one
    # it was offered. Each of these is the opposite of its default, so a setting
    # that quietly went back to it is the failure.
    "minimapclock": 1,
    "friendlyplates": 0,
}

# Every row of the schema as well, moved off whatever it holds. The nine above
# ride the CVar store; these live in settings.cfg, which is the file the client
# writes when it closes and reads when it opens.
SCHEMA_LUA = r"""
local moved = {}
for _, r in ipairs(WoweeSettingList()) do
  local now = tonumber(WoweeGetSetting(r.key))
  local want
  if r.kind == "bool" then
    want = (now == 1) and 0 or 1
  elseif r.kind == "enum" then
    want = (now == 0) and 1 or 0
  else
    -- A step off the end it is not already sitting on, so the value is inside
    -- the row and is not the one it started at.
    want = (now == r.min) and r.max or r.min
  end
  WoweeSetSetting(r.key, tostring(want))
end

-- What everything holds once the walk is done, not what each was set to as it
-- went past. Choosing a quality preset sets nine other settings, and changing
-- any of those moves the preset to Custom - so graphicspreset was recorded as
-- Low and was Custom by the end, through no fault of the saving.
for _, r in ipairs(WoweeSettingList()) do
  moved[#moved+1] = r.key .. "=" .. tostring(WoweeGetSetting(r.key))
end
"""

FIRST = ('local before = {} ' +
         "".join(f'before[#before+1] = "{k}=" .. tostring(WoweeGetSetting("{k}")) '
                 for k in WANTED) +
         "".join(f'WoweeSetSetting("{k}", "{v}") ' for k, v in WANTED.items()) +
         SCHEMA_LUA +
         'error("QQ" .. "BEFORE " .. table.concat(before, " ") .. " ~ " .. '
         'table.concat(moved, " "))')
SECOND = ('local out = {} ' +
          "".join(f'out[#out+1] = "{k}=" .. tostring(WoweeGetSetting("{k}")) '
                  for k in WANTED) +
          'for _, r in ipairs(WoweeSettingList()) do '
          'out[#out+1] = r.key .. "=" .. tostring(WoweeGetSetting(r.key)) end ' +
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
        first = run(FIRST)
        second = run(SECOND)
    except subprocess.TimeoutExpired:
        print("the runner did not finish - no restart was made.")
        return 1

    # What they were before the first run set them. A wanted value that is
    # already the default would come back whether anything persisted or not:
    # friendlyplates was written here as 1 and defaults to 1, so that one key
    # was passing on a value it had never moved off.
    started, schemaLeft = {}, {}
    for line in (first.stdout + first.stderr).splitlines():
        at = line.find("QQ" + "BEFORE ")
        if at != -1:
            head, _, tail = line[at + len("QQBEFORE "):].partition(" ~ ")
            for item in head.split():
                name, _, value = item.partition("=")
                started[name] = value
            for item in tail.split():
                name, _, value = item.partition("=")
                schemaLeft[name] = value
            break

    stuck = []
    for key, want in WANTED.items():
        was = started.get(key)
        if was is None:
            continue
        try:
            same = abs(float(was) - float(want)) <= 0.001
        except ValueError:
            same = was == str(want)
        if same:
            stuck.append(f"{key}: asked for {want}, which is what it already was - "
                         "this one would come back whether anything persisted or not")

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

    # And every schema row, against what the first run left it on.
    for key, left in sorted(schemaLeft.items()):
        got = came.get(key)
        if got is None:
            lost.append(f"{key}: the second run had no value for it")
            continue
        try:
            near = abs(float(got) - float(left)) <= max(0.02, abs(float(left)) * 0.01)
        except ValueError:
            near = got == left
        if not near:
            lost.append(f"{key}: left on {left} and came back as {got}")

    lost.extend(stuck)
    print(f"{len(WANTED)} settings a Blizzard control drives and "
          f"{len(schemaLeft)} rows of the schema, set in one run and read in the next.\n")
    for entry in lost:
        print(f"  {entry}")

    if lost:
        print(f"\n{len(lost)} setting(s) did not survive the restart.")
        return 0

    print("every setting set in one session is still there in the next.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
