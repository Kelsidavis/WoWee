#!/usr/bin/env python3
"""Every load-on-demand panel, loaded and opened for real.

The static sweeps read the interface and the bindings and compare them. None
of them can see a call that *is* answered and answers wrongly - a widget method
that exists and returns nil is not a missing name, not a short return and not a
type mismatch, so it passes every one of them and raises the first time a
handler reaches through it.

That is exactly what kept the calendar shut. `region:GetParent()` fell through
to the shared widget no-op, so `darkTop:GetParent()` was nil and every day of
every month raised before one could be drawn. Nothing in `tools/` could have
found it. Loading the addon and opening the panel found it in one run.

So this opens all of them:

    tools/framexml_addon_open_check.py

For each panel - LoadAddOn, then Show, then its own OnShow handler, each inside
a pcall. OnShow by hand rather than by showing alone, because visibility is
reported after the render in the real client and there is no render here; the
handler is where the work is, and a check that only called Show would have
reported the calendar clean while it was broken.

Then the interface is **ticked** with every panel open. Opening a frame and
calling one handler says nothing about the work it does every frame afterwards,
and OnUpdate is where a panel dies quietly: it is dispatched from a list, gated
on the widget's visible chain, and unhooked entirely after five consecutive
failures. A frame whose OnUpdate raises looks perfectly healthy when its
function is invoked by hand and is dead for the rest of the session in the
running client - the symptom is something on screen that has simply stopped
moving.

Canaried against the fault it was written for: with the region GetParent
binding removed, the calendar reports its raise and everything else stays
clean.

Needs `build/bin/framexml_run` and a `Data` directory, and skips rather than
fails when either is absent - the same rule sweep_guard uses for the runner.
"""

import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
RUNNER = ROOT / "build" / "bin" / "framexml_run"
DATA = ROOT / "Data"
# What this actually needs. Data/expansions and Data/opcodes are tracked, so a
# checkout with no extracted interface still has a Data directory - and testing
# for that one ran the whole sweep against an interface that is not there.
INTERFACE = ROOT / "Data/interface"

# The panel each addon exists to draw. Eight of these are the load-on-demand
# entries in framexml_takeover.cpp's check list, which is where the frame name
# for each was taken from; the rest are the addons that ship beside them.
PANELS = [
    ("Blizzard_TalentUI", "PlayerTalentFrame"),
    ("Blizzard_TradeSkillUI", "TradeSkillFrame"),
    ("Blizzard_TrainerUI", "ClassTrainerFrame"),
    ("Blizzard_AuctionUI", "AuctionFrame"),
    ("Blizzard_GuildBankUI", "GuildBankFrame"),
    ("Blizzard_InspectUI", "InspectFrame"),
    ("Blizzard_AchievementUI", "AchievementFrame"),
    ("Blizzard_BarbershopUI", "BarberShopFrame"),
    ("Blizzard_Calendar", "CalendarFrame"),
    ("Blizzard_MacroUI", "MacroFrame"),
    ("Blizzard_BindingUI", "KeyBindingFrame"),
    ("Blizzard_GlyphUI", "GlyphFrame"),
    ("Blizzard_ItemSocketingUI", "ItemSocketingFrame"),
    ("Blizzard_RaidUI", "RaidFrame"),
    ("Blizzard_TimeManager", "TimeManagerFrame"),
    ("Blizzard_TokenUI", "TokenFrame"),
    ("Blizzard_GMSurveyUI", "GMSurveyFrame"),
    ("Blizzard_BattlefieldMinimap", "BattlefieldMinimap"),
    ("Blizzard_ArenaUI", "ArenaEnemyFrames"),
    ("Blizzard_CombatText", "CombatText"),
    ("Blizzard_GMChatUI", "GMChatFrame"),
]

# One chunk for all of them rather than one process each: loading the interface
# takes most of a run, and twenty-one of those is minutes rather than seconds.
# Every step is inside a pcall so one panel raising does not hide the rest.
PIECE = (
    'do local ok, err = pcall(LoadAddOn, "{addon}") '
    'local f = _G["{frame}"] '
    'local ok2, err2 = true, nil '
    'if f and f.Show then ok2, err2 = pcall(function() '
    'f:Show() '
    'local h = f.GetScript and f:GetScript("OnShow") '
    'if h then h(f) end end) end '
    'local note = "" '
    'if not ok then note = " LOAD:" .. tostring(err) end '
    'if not ok2 then note = note .. " SHOW:" .. tostring(err2) end '
    'if not f then note = note .. " NOFRAME" end '
    'out[#out+1] = "{addon}=" .. (note == "" and "ok" or note) end'
)


def main():
    if not RUNNER.is_file() or not DATA.is_dir() or not INTERFACE.is_dir():
        print("framexml_run or Data is missing - nothing opened.")
        return 0

    lua = ["local out = {}"]
    for addon, frame in PANELS:
        lua.append(PIECE.format(addon=addon, frame=frame))
    # Reported through error() because that is the one channel whose text comes
    # back whole; the runner's own output is a report of its own.
    lua.append('error("QQPANELS " .. table.concat(out, " ~ "))')

    # Four seconds of frames with everything open. Long enough for the timers
    # FrameXML drives in seconds - fades, flashes, combat text ageing out - to
    # run to completion rather than only to start.
    # A config root of this check's own. The runner reads the CVar store at
    # start-up, so a run inherits whatever the last one wrote there - and three
    # checks drive this runner. A panel that opens differently because another
    # check left a CVar behind is a failure nobody can reproduce alone.
    env = dict(os.environ)
    env["WOWEE_CONFIG_ROOT"] = str(ROOT / "logs/addon_open_check_config")

    try:
        run = subprocess.run([str(RUNNER), str(DATA), " ".join(lua), "--tick:240"],
                             capture_output=True, text=True, timeout=900, env=env)
    except subprocess.TimeoutExpired:
        print("the runner did not finish - nothing opened.")
        return 1

    payload = None
    for line in (run.stdout + run.stderr).splitlines():
        if "QQPANELS " in line and "script error" in line:
            payload = line.split("QQPANELS ", 1)[1]
            break
    if payload is None:
        print("no result from the runner - the chunk did not reach its report.")
        return 1

    # Errors reported while ticking. The runner prints them under the tick's
    # own heading, so anything between that line and the end of its block
    # belongs to a per-frame handler rather than to opening a panel.
    # The runner prints each new error under the tick as an indented line, and
    # its own "ticked N frame(s)" beside them. Matching on the word "error"
    # instead caught the engine's end-of-session summary - a count of errors,
    # not an error - and reported a clean run as one failure.
    tickErrors = []
    inTick = False
    for line in (run.stdout + run.stderr).splitlines():
        if line.startswith("== --tick:"):
            inTick = True
            continue
        if inTick:
            if line.startswith("== "):
                inTick = False
            elif line.startswith("   ") and "ticked" not in line:
                tickErrors.append(line.strip())

    bad = []
    for entry in payload.split(" ~ "):
        name, _, state = entry.partition("=")
        if state.strip() == "ok":
            continue
        bad.append((name, state.strip()))

    print(f"{len(PANELS)} panels opened; {len(PANELS) - len(bad)} clean.\n")
    for name, state in bad:
        print(f"  {name}")
        print(f"      {state}")
    for line in tickErrors:
        print(f"  while ticking: {line}")

    if bad or tickErrors:
        print(f"\n{len(bad)} panel(s) raise on being opened, "
              f"{len(tickErrors)} error(s) while ticking.")
    else:
        print("every panel loads, shows and runs its OnShow without raising, "
              "and none raises while the interface ticks.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
