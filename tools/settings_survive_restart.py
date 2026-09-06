#!/usr/bin/env python3
"""Settings set in one session, read back in the next.

Everything else here checks a setting inside one run: that a control shows it,
that changing it writes it, that the store hears about it. None of that says the
value comes back, and coming back is the whole of what a setting is for.

Two runs of the runner over one config root is a restart. The first moves
every row of the schema off the value it holds; the second starts clean, reads
its config from disk the way the client does, and says what it found.

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

The comparison is exact rather than near, because every one of them is: a
setting that came back a little different would come back a little different
again on the next restart, and a value that walks is worse than one that jumps.
Ground clutter is the one that has to travel furthest to stay put - a whole
percent here is 0.426667 of a doodad in the units Blizzard's slider counts in,
and the store is not handed the rounded echo - and it still returns exactly.
Canaried by making one setting drift half a degree per save, which is reported
by name.

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

# Every row of the schema, moved off whatever it holds. They live in
# settings.cfg, which is the file the client writes when it closes and reads
# when it opens; the ones a CVar is bound to ride the CVar store as well, and
# have to come back through both.
#
# Until 2026-09-05 six of them - ground clutter, mouse speed, view distance,
# the effects volume, the minimap clock and friendly nameplates - had no schema
# row and were set here by name, on values of their own. They are rows now, so
# the walk reaches them like any other, and a value set by name first would
# only be one the walk had since moved on from.
SCHEMA_LUA = r"""
-- Not the rows the server keeps - the cloak and the helm, on a "lua:" store -
-- which this runner has no server to keep them with.
local function rows()
  local out = {}
  for _, r in ipairs(WoweeSettingList()) do
    if not tostring(r.store or ""):find("^lua:") then out[#out+1] = r end
  end
  return out
end
local moved = {}
for _, r in ipairs(rows()) do
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
for _, r in ipairs(rows()) do
  moved[#moved+1] = r.key .. "=" .. tostring(WoweeGetSetting(r.key))
end
"""

FIRST = (SCHEMA_LUA +
         'error("QQ" .. "BEFORE " .. table.concat(moved, " "))')
SECOND = ('local out = {} '
          'for _, r in ipairs(WoweeSettingList()) do '
          'if not tostring(r.store or ""):find("^lua:") then '
          'out[#out+1] = r.key .. "=" .. tostring(WoweeGetSetting(r.key)) end end '
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

    # What the first run left every row on, which is what the second must find.
    schemaLeft = {}
    for line in (first.stdout + first.stderr).splitlines():
        at = line.find("QQ" + "BEFORE ")
        if at != -1:
            for item in line[at + len("QQBEFORE "):].split():
                name, _, value = item.partition("=")
                schemaLeft[name] = value
            break
    if not schemaLeft:
        print("the first run said nothing - no setting was moved.")
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
    # Every schema row, against what the first run left it on.
    for key, left in sorted(schemaLeft.items()):
        got = came.get(key)
        if got is None:
            lost.append(f"{key}: the second run had no value for it")
            continue
        try:
            near = abs(float(got) - float(left)) <= 0.0001
        except ValueError:
            near = got == left
        if not near:
            lost.append(f"{key}: left on {left} and came back as {got}")

    print(f"{len(schemaLeft)} rows of the schema, set in one run and read in the next.\n")
    for entry in lost:
        print(f"  {entry}")

    if lost:
        print(f"\n{len(lost)} setting(s) did not survive the restart.")
        return 0

    print("every setting set in one session is still there in the next.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
