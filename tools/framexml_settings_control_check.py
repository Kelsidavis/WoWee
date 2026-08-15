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

Canaried against the fault it was written for: with the panel's label pick moved
one along - `choices[selected() + 1]` - all twenty-six index and label pairs
report wrong, and putting it back reports none.

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
local bad, checked, controls = {}, 0, 0

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

    WoweeSetSetting(r.key, before)
    if panel.refresh then panel:refresh() end
  end
end

-- The sentinel is built rather than written, because the runner echoes the
-- expression it was given: a literal here matches in the echo before the
-- result, and what comes back is this script's own source.
error("QQ" .. string.format("SETTINGS %d ~ %d ~ %s", controls, checked, table.concat(bad, " ~ ")))
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
    bad = [b.strip() for b in rest.split(" ~ ") if b.strip()]

    print(f"{controls} controls built, {checked} values shown and read back.\n")
    for entry in bad:
        print(f"  {entry}")

    if bad:
        print(f"\n{len(bad)} control(s) show something other than the value they were given.")
        return 0

    # A sweep that matched nothing reports the same silence as a clean one.
    if int(checked or 0) < 20:
        print("\nfewer values were checked than any build has settings - "
              "the walk stopped matching rather than finding nothing wrong.")
        return 1

    print("every control shows the value it is given, and every dropdown shows "
          "the label its index names.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
