#!/usr/bin/env python3
"""Every control the settings schema generates, shown a value and read back.

The tests under `tests/` check the schema against itself and against the C++
that loads and saves it. None of them builds a control. So a row can be correct
in every one of them and still draw a dropdown whose text belongs to a different
index - which is precisely what the login screen's parallax control did, from
its own hand-written copy of the same scale: its "High" asked for the 32 samples
the panel calls Medium, and the 64 it named could not be chosen there at all.

That fault is invisible to a static check, because both halves are individually
right. The scale is right, the panel builds from the scale, and the mapping
between them is wrong.

So this drives the real thing. `WoweeSettingList()` hands over the compiled
schema; for each dropdown it sets every index the row allows, refreshes the
panel, and reads the text the control is showing. For each checkbox and slider
it sets a value the row allows and reads back what the control holds.

Then it goes the other way and moves the control, because a control that shows
the right value and writes nothing when it is used is the shape the options
audit spent itself on - it reads correctly, it saves correctly, and it changes
nothing. Sixty-six of the seventy-two can be driven that way; the dropdowns are
left out, their menu buttons being built on being opened.

Then it presses Defaults on every panel with every setting moved off its
default. That button was a function that did nothing here once, before the
schema carried a default at all, and what that looked like is exactly what a
broken one looks like now: the panel redraws and every value stays put.

Then Okay and Cancel, which are halves of one promise: Cancel leaves the panel
as it was found, and Okay re-takes the snapshot so a Cancel afterwards has
nothing to undo. Without the second, accepting a change and cancelling anything
later would put the accepted change back.

It opens a panel, changes a control and presses Cancel. Cancel snapshots
on refresh, which is what opening a panel does, so the panel has to be opened
before the change or Cancel is being asked to undo something it never saw.

Then the quality preset, the one setting that sets others. Choosing it has to
move the nine it covers, and each step up has to ask for at least as much as
the one below - a column that goes backwards is a preset that improves
something by turning it down.

Last the dropdowns, which are the one control the move above cannot reach - a
menu's buttons are built when it opens. Their menus are built by hand and what
a click calls is called, because the index conversion in there is the fault
this file exists for: a menu button is 1-based and the setting is 0-based, and
`button.value` where `button.value - 1` belongs writes every choice one along.

Canaried against all three faults it was written for: with the panel's label
pick moved one along - `choices[selected() + 1]` - all twenty-six index and
label pairs report wrong; with the slider's write taken out, all thirty-one
sliders report the setting unmoved; with the Defaults write taken out, all
seventy-two report themselves left where they were; with Cancel's restore taken
out, all thirty-five keep the change; with Okay's snapshot taken out, all
thirty-five have the accepted change undone by the next Cancel; with the menu's `- 1` taken off, all
twenty-six choices write an index one too high, which is the parallax fault
itself; with High's shadow distance dropped below Medium's, the preset step
reports it.

It is backed by a real SettingsPanel rather than a map of strings. A map answers
and stores, which is enough to draw a control and read it back and not enough to
be the client: setSettingValue clamps, finds the field through the binding
table, runs the side effects, and for the quality preset assigns nine other
settings. Asking a map whether a preset moves anything asks the harness. Rebuild after putting any of
them back. A restored header and a stale binary reported every dropdown off by
one, convincingly, for a fault that was no longer in the source.

Two things it does not do, on purpose. It does not compare the labels against a
list of its own: the point is whether the panel and the schema agree, and a
third copy here would be one more thing to drift. And it does not ask a widget
whether it is enabled - `IsEnabled` is one of the stand-ins that answers true for
everything, so the greying is read off the label colour `setEnabled` writes.

Needs `build/bin/framexml_run` and a `Data` directory with an extracted
interface, and skips rather than fails when either is absent - the same rule
sweep_guard uses for the runner.
"""

import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
RUNNER = ROOT / "build" / "bin" / "framexml_run"
DATA = ROOT / "Data"
# Data/expansions and Data/opcodes are tracked, so a checkout with no extracted
# interface still has a Data directory. Testing for that one runs the sweep
# against an interface that is not there and calls the silence a pass.
INTERFACE = ROOT / "Data/interface"

# The panel global for a category is its name with everything but letters and
# digits taken out: "Combat & HUD" builds WoweeOptionsCombatHUD. A control is
# that name with the setting key on the end, and a slider or dropdown's text is
# that again with "Text".
LUA = r'''
local function panelOf(cat) return "WoweeOptions" .. cat:gsub("[^%w]", "") end

-- cvar -> client setting, read out of kClientCVars rather than written again
-- here. Six settings have no schema row because a Blizzard control drives them
-- through one of these, so a binding that stops working is one of the game's
-- own controls going quietly dead - which is the fault the options audit was.
local CLIENT_CVARS = { --[[PAIRS]] }
local cvarsReached = 0
local bad, checked, controls, written = {}, 0, 0, 0

for _, r in ipairs(WoweeSettingList()) do
  local base = panelOf(r.category)
  local panel, ctrl = _G[base], _G[base .. r.key]
  if not panel then
    bad[#bad+1] = r.key .. ": no panel " .. base
  elseif not ctrl then
    bad[#bad+1] = r.key .. ": no control " .. base .. r.key
  else
    controls = controls + 1
    local before = WoweeGetSetting(r.key)

    if r.kind == "enum" then
      local labels = {}
      for w in (r.choices .. "|"):gmatch("(.-)|") do labels[#labels+1] = w end
      local textw = _G[base .. r.key .. "Text"]
      if #labels == 0 then
        bad[#bad+1] = r.key .. ": a dropdown with no choices"
      elseif not textw or not textw.GetText then
        bad[#bad+1] = r.key .. ": dropdown has no text to read"
      else
        for i = 1, #labels do
          WoweeSetSetting(r.key, tostring(i - 1))
          if panel.refresh then panel:refresh() end
          checked = checked + 1
          local shown = textw:GetText()
          if shown ~= labels[i] then
            bad[#bad+1] = string.format("%s index %d shows %q, the schema says %q",
                                        r.key, i - 1, tostring(shown), labels[i])
          end
        end
      end

    elseif r.kind == "bool" then
      for _, want in ipairs({"1", "0"}) do
        WoweeSetSetting(r.key, want)
        if panel.refresh then panel:refresh() end
        checked = checked + 1
        if ctrl.GetChecked then
          local got = ctrl:GetChecked() and "1" or "0"
          if got ~= want then
            bad[#bad+1] = string.format("%s set to %s, the box reads %s", r.key, want, got)
          end
        end
      end

    else
      -- A value the row allows that is not its default, so a control that
      -- ignores what it is given and shows the default reads as wrong.
      local mid = r.min + (r.max - r.min) / 2
      if r.step and r.step > 0 then mid = r.min + math.floor((mid - r.min) / r.step) * r.step end
      WoweeSetSetting(r.key, tostring(mid))
      if panel.refresh then panel:refresh() end
      checked = checked + 1
      if ctrl.GetValue then
        local got = ctrl:GetValue()
        if type(got) ~= "number" or math.abs(got - mid) > math.max(0.001, (r.step or 0) / 2) then
          bad[#bad+1] = string.format("%s set to %s, the slider holds %s",
                                      r.key, tostring(mid), tostring(got))
        end
      end
    end

    -- And the other way. A control that shows the right value and writes
    -- nothing when it is moved is the shape the options audit spent itself on:
    -- it reads correctly, it saves correctly, and it never changes anything.
    if r.kind == "bool" and ctrl.Click then
      WoweeSetSetting(r.key, "0")
      if panel.refresh then panel:refresh() end
      ctrl:Click()
      written = written + 1
      if WoweeGetSetting(r.key) ~= "1" then
        bad[#bad+1] = r.key .. ": ticking the box left the setting at " ..
                      tostring(WoweeGetSetting(r.key))
      end
    elseif r.kind ~= "enum" and r.kind ~= "bool" and ctrl.SetValue then
      local target = r.min + (r.max - r.min) / 2
      if r.step and r.step > 0 then target = r.min + math.floor((target - r.min) / r.step) * r.step end
      WoweeSetSetting(r.key, tostring(r.min))
      if panel.refresh then panel:refresh() end
      ctrl:SetValue(target)
      written = written + 1
      local got = tonumber(WoweeGetSetting(r.key))
      if not got or math.abs(got - target) > math.max(0.01, r.step or 0) then
        bad[#bad+1] = string.format("%s: moving the slider to %s left the setting at %s",
                                    r.key, tostring(target), tostring(WoweeGetSetting(r.key)))
      end
    end

    WoweeSetSetting(r.key, before)
    if panel.refresh then panel:refresh() end
  end
end

-- The sentinel is built rather than written, because the runner echoes the
-- expression it was given: a literal here matches in the echo before the
-- result, and what comes back is this script's own source.
-- And the button the game puts on every one of its panels. It was a function
-- that did nothing here once, before the schema carried a default at all, so
-- what it looks like when it is broken is exactly what it looked like then:
-- the panel redraws and every value stays where it was.
local restored = 0
do
  local away = {}
  for _, r in ipairs(WoweeSettingList()) do
    local off
    if r.kind == "bool" then off = (tostring(r.default) == "1") and "0" or "1"
    elseif r.kind == "enum" then off = tostring((tonumber(r.default) == 0) and 1 or 0)
    else off = tostring(r.default == r.min and r.max or r.min) end
    away[r.key] = off
    WoweeSetSetting(r.key, off)
  end

  local pressed = {}
  for _, r in ipairs(WoweeSettingList()) do
    local base = panelOf(r.category)
    if not pressed[base] then
      pressed[base] = true
      local p = _G[base]
      if p and p.default then p:default() end
    end
  end

  for _, r in ipairs(WoweeSettingList()) do
    local now, want = WoweeGetSetting(r.key), tostring(r.default)
    local same = (tonumber(now) and tonumber(want)
                  and math.abs(tonumber(now) - tonumber(want)) <= 0.001) or now == want
    restored = restored + 1
    if not same then
      bad[#bad+1] = string.format("%s: Defaults left it at %s, the schema says %s",
                                  r.key, tostring(now), want)
    end
  end
end

-- And Cancel, which promises the panel is left as it was found. It snapshots
-- on refresh, which is what opening a panel does, so the test has to open one
-- before changing anything or it is asking Cancel to undo a change it never
-- saw.
local cancelled = 0
for _, r in ipairs(WoweeSettingList()) do
  local base = panelOf(r.category)
  local panel, ctrl = _G[base], _G[base .. r.key]
  if panel and ctrl and r.kind == "bool" and ctrl.Click and panel.cancel then
    local opened = (tostring(r.default) == "1") and "1" or "0"
    WoweeSetSetting(r.key, opened)
    if panel.refresh then panel:refresh() end
    ctrl:Click()
    panel:cancel()
    cancelled = cancelled + 1
    if WoweeGetSetting(r.key) ~= opened then
      bad[#bad+1] = string.format("%s: Cancel left it at %s, it was %s when the panel opened",
                                  r.key, tostring(WoweeGetSetting(r.key)), opened)
    end
  end
end

-- The dropdowns, which are the one control the write check above cannot move:
-- their buttons are built when the menu opens. Building them by hand and
-- calling what a click calls is the only way to see the index conversion, and
-- the index conversion is where the fault this file exists for lived.
--
-- The infos are copied rather than held: UIDropDownMenu_CreateInfo hands back
-- one recycled table, so a list of references to it is a list of the last
-- button built, and every choice appears to write the last index.
local chosen = 0
do
  local captured
  local realAdd = UIDropDownMenu_AddButton
  UIDropDownMenu_AddButton = function(info, level)
    if captured then
      captured[#captured + 1] = {value = info.value, func = info.func, text = info.text}
    end
    return realAdd(info, level)
  end

  for _, r in ipairs(WoweeSettingList()) do
    if r.kind == "enum" then
      local dd = _G[panelOf(r.category) .. r.key]
      if dd and dd.initialize then
        captured = {}
        dd:initialize(1)
        local infos = captured
        captured = nil
        for i, info in ipairs(infos) do
          if info.func then
            info.func({value = info.value})
            chosen = chosen + 1
            local got = WoweeGetSetting(r.key)
            if tostring(got) ~= tostring(i - 1) then
              bad[#bad+1] = string.format("%s: choosing %q wrote %s, the index is %d",
                                          r.key, tostring(info.text), tostring(got), i - 1)
            end
          end
        end
      else
        bad[#bad+1] = r.key .. ": a dropdown with nothing to build its menu"
      end
    end
  end
  UIDropDownMenu_AddButton = realAdd
end

-- The quality preset, which is the one setting that sets others. Choosing it
-- has to move the nine it has an opinion about, and each step up has to ask for
-- at least as much as the one below.
local presets = 0
do
  local watched = {"viewdistance", "shadowdistance", "antialiasing", "parallaxquality",
                   "groundclutter"}
  local previous
  for index = 1, 4 do
    WoweeSetSetting("graphicspreset", tostring(index))
    presets = presets + 1
    if WoweeGetSetting("graphicspreset") ~= tostring(index) then
      bad[#bad+1] = "graphicspreset: choosing " .. index .. " left it at " ..
                    tostring(WoweeGetSetting("graphicspreset"))
    end
    local now = {}
    for _, k in ipairs(watched) do now[k] = tonumber(WoweeGetSetting(k)) end
    if previous then
      local moved = false
      for _, k in ipairs(watched) do
        if now[k] and previous[k] then
          if now[k] ~= previous[k] then moved = true end
          if now[k] < previous[k] then
            bad[#bad+1] = string.format("preset %d asks for less %s than preset %d (%s against %s)",
                                        index, k, index - 1, tostring(now[k]), tostring(previous[k]))
          end
        end
      end
      if not moved then
        bad[#bad+1] = "preset " .. index .. " changed none of the settings it covers"
      end
    end
    previous = now
  end
end

-- Every CVar a Blizzard control writes, and the setting it is meant to reach.
for _, pair in ipairs(CLIENT_CVARS) do
  local cvar, key = pair[1], pair[2]
  local before = WoweeGetSetting(key)
  for _, probe in ipairs({"1", "0"}) do
    SetCVar(cvar, probe)
    cvarsReached = cvarsReached + 1
    local got = WoweeGetSetting(key)
    local same = (tonumber(got) and math.abs(tonumber(got) - tonumber(probe)) < 0.001)
                 or tostring(got) == probe
    if not same then
      bad[#bad+1] = string.format("%s: setting the CVar to %s left %s reading %s",
                                  cvar, probe, key, tostring(got))
    end
  end
  WoweeSetSetting(key, before)
end

-- Okay, which is Cancel's other half. It re-takes the snapshot, so a Cancel
-- afterwards has nothing to undo: without that, accepting a change and then
-- cancelling anything later would put the accepted change back.
local committed = 0
for _, r in ipairs(WoweeSettingList()) do
  local base = panelOf(r.category)
  local panel, ctrl = _G[base], _G[base .. r.key]
  if panel and ctrl and r.kind == "bool" and ctrl.Click and panel.okay and panel.cancel then
    local opened = (tostring(r.default) == "1") and "1" or "0"
    WoweeSetSetting(r.key, opened)
    if panel.refresh then panel:refresh() end
    ctrl:Click()
    local accepted = WoweeGetSetting(r.key)
    panel:okay()
    panel:cancel()
    committed = committed + 1
    if WoweeGetSetting(r.key) ~= accepted then
      bad[#bad+1] = string.format("%s: accepted %s with Okay, a later Cancel put it back to %s",
                                  r.key, accepted, tostring(WoweeGetSetting(r.key)))
    end
  end
end

-- The range in the row, honoured by the one path nothing bounded. The sliders
-- are built from it and the config loader clamps to it; setSettingValue, which
-- is what these panels and any addon calling WoweeSetSetting go through, took
-- whatever it was handed.
local bounded = 0
for _, r in ipairs(WoweeSettingList()) do
  if r.kind ~= "bool" then
    local before = WoweeGetSetting(r.key)
    for _, probe in ipairs({{r.max + 1000, r.max}, {r.min - 1000, r.min}}) do
      WoweeSetSetting(r.key, tostring(probe[1]))
      bounded = bounded + 1
      local got = tonumber(WoweeGetSetting(r.key))
      if not got or math.abs(got - probe[2]) > 0.001 then
        bad[#bad+1] = string.format("%s: given %s, kept %s, the row stops at %s",
                                    r.key, tostring(probe[1]), tostring(got), tostring(probe[2]))
      end
    end
    WoweeSetSetting(r.key, before)
  end
end

error("QQ" .. string.format("SETTINGS %d ~ %d ~ %d ~ %d ~ %d ~ %d ~ %d ~ %d ~ %d ~ %d ~ %s",
                            controls, checked, written, restored, cancelled, chosen, presets,
                            bounded, committed, cvarsReached, table.concat(bad, " ~ ")))
'''


def clientCVarPairs():
    """The cvar -> setting rows of kClientCVars, so the probe is not a copy."""
    source = (ROOT / "src/addons/lua_system_api.cpp").read_text()
    at = source.find("kClientCVars[] = {")
    if at == -1:
        return []
    body = source[at:source.find("};", at)]
    return re.findall(r'\{"([a-z0-9_]+)",\s*"([a-z0-9_]+)"\}', body)


def main():
    if not RUNNER.is_file() or not DATA.is_dir() or not INTERFACE.is_dir():
        print("framexml_run or an extracted interface is missing - no control was built.")
        return 0

    # The probe is built from the table it checks, which means a row deleted
    # from kClientCVars is one fewer check rather than a failure. The floor is
    # what catches that: nine is what the table has, and six of them are the
    # settings with no schema row at all, whose only control is Blizzard's.
    pairs = clientCVarPairs()
    if len(pairs) < 9:
        print(f"kClientCVars has {len(pairs)} rows where it had nine - a binding "
              "was removed, and a Blizzard control now writes a CVar nothing reads.")
        return 1
    lua = LUA.replace("--[[PAIRS]]",
                      ", ".join('{"%s", "%s"}' % (c, k) for c, k in pairs))

    try:
        run = subprocess.run([str(RUNNER), str(DATA), lua],
                             capture_output=True, text=True, timeout=900)
    except subprocess.TimeoutExpired:
        print("the runner did not finish - no control was built.")
        return 1

    # Reported through error() because that is the one channel whose text comes
    # back whole; print() goes to the chat frame, which is not drawn here.
    payload = None
    for line in (run.stdout + run.stderr).splitlines():
        at = line.find("QQ" + "SETTINGS ")
        if at != -1:
            payload = line[at + len("QQSETTINGS "):]
            break

    if payload is None:
        print("the runner built no controls - the settings list did not answer.")
        return 1

    controls, _, rest = payload.partition(" ~ ")
    checked, _, rest = rest.partition(" ~ ")
    written, _, rest = rest.partition(" ~ ")
    restored, _, rest = rest.partition(" ~ ")
    cancelled, _, rest = rest.partition(" ~ ")
    chosen, _, rest = rest.partition(" ~ ")
    presets, _, rest = rest.partition(" ~ ")
    bounded, _, rest = rest.partition(" ~ ")
    committed, _, rest = rest.partition(" ~ ")
    cvars, _, rest = rest.partition(" ~ ")
    bad = [b.strip() for b in rest.split(" ~ ") if b.strip()]

    print(f"{controls} controls built, {checked} values shown and read back, "
          f"{written} changed at the control, {restored} restored by Defaults, "
          f"{cancelled} put back by Cancel, {chosen} chosen from a menu, "
          f"{presets} quality presets applied, {bounded} out-of-range values "
          f"held to the row, {committed} kept by Okay against a later Cancel, "
          f"{cvars} CVar writes followed to their setting.\n")
    for entry in bad:
        print(f"  {entry}")

    if bad:
        print(f"\n{len(bad)} control(s) show something other than the value they were given.")
        return 0

    # A sweep that matched nothing reports the same silence as a clean one.
    if int(checked or 0) < 20 or int(written or 0) < 20 or int(restored or 0) < 20 or int(cancelled or 0) < 20 or int(chosen or 0) < 20:
        print("\nfewer values were checked than any build has settings - "
              "the walk stopped matching rather than finding nothing wrong.")
        return 1

    # sweep_guard matches the first clause, so the rest is free to be a list.
    print("every control shows the value it is given, and:\n"
          "  a dropdown shows the label its index names, and writes that index "
          "when it is chosen\n"
          "  moving a control writes the setting behind it\n"
          "  Defaults puts every setting back\n"
          "  Cancel leaves the panel as it was found, and Okay keeps a change "
          "against a Cancel after it\n"
          "  a quality preset moves what it covers, and never asks for less "
          "than the one below\n"
          "  a value past the end of a row is held to it\n"
          "  a CVar a Blizzard control writes reaches the setting behind it")
    return 0


if __name__ == "__main__":
    sys.exit(main())
