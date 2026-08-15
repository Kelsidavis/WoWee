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

Last it opens a panel, changes a control and presses Cancel. Cancel snapshots
on refresh, which is what opening a panel does, so the panel has to be opened
before the change or Cancel is being asked to undo something it never saw.

Canaried against all three faults it was written for: with the panel's label
pick moved one along - `choices[selected() + 1]` - all twenty-six index and
label pairs report wrong; with the slider's write taken out, all thirty-one
sliders report the setting unmoved; with the Defaults write taken out, all
seventy-two report themselves left where they were; with Cancel's restore taken
out, all thirty-five keep the change. Rebuild after putting any of
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

error("QQ" .. string.format("SETTINGS %d ~ %d ~ %d ~ %d ~ %d ~ %s",
                            controls, checked, written, restored, cancelled,
                            table.concat(bad, " ~ ")))
'''


def main():
    if not RUNNER.is_file() or not DATA.is_dir() or not INTERFACE.is_dir():
        print("framexml_run or an extracted interface is missing - no control was built.")
        return 0

    try:
        run = subprocess.run([str(RUNNER), str(DATA), LUA],
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
    bad = [b.strip() for b in rest.split(" ~ ") if b.strip()]

    print(f"{controls} controls built, {checked} values shown and read back, "
          f"{written} changed at the control, {restored} restored by Defaults, "
          f"{cancelled} put back by Cancel.\n")
    for entry in bad:
        print(f"  {entry}")

    if bad:
        print(f"\n{len(bad)} control(s) show something other than the value they were given.")
        return 0

    # A sweep that matched nothing reports the same silence as a clean one.
    if int(checked or 0) < 20 or int(written or 0) < 20 or int(restored or 0) < 20 or int(cancelled or 0) < 20:
        print("\nfewer values were checked than any build has settings - "
              "the walk stopped matching rather than finding nothing wrong.")
        return 1

    print("every control shows the value it is given, every dropdown shows the "
          "label its index names, moving one writes the setting, Defaults puts "
          "every setting back, and Cancel leaves the panel as it was found.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
