#!/usr/bin/env python3
"""Blizzard's sliders, against the range this client will accept from them.

The interface declares a range beside each slider it draws -
`{ minValue = 16, maxValue = 64 }` for Ground Density, `0.5` to `1.5` for Mouse
Sensitivity - and those sliders write CVars this client binds to settings of its
own. The two do not always count the same thing, and when they do not, the
slider hands over a number the setting has no room for.

Both faults that produced this looked fine from either end:

  * Ground Density counts doodads, 16 to 64. The setting is a proportion, 0 to
    1.5. Every one of that slider's seven positions came out as the most clutter
    the client draws, and the value written to the config was clamped back down
    on the next load, which made it permanent.
  * Mouse Sensitivity is a multiplier around 1.0, 0.5 to 1.5. The setting is an
    amount that sits at 0.2 and stops at 1.0. The slowest setting that slider
    offered was two and a half times the client's default.

Neither shows up as a slider that does nothing, which is what a check would
naturally look for: each position wrote a different number. What made them wrong
is where those numbers landed, so this follows the whole chain rather than the
ends of it -

    the interface's declared range
      x the scale on the kClientCVars row
      x 100 where the binding is a fraction
      against the range the config loader clamps that field to

and reports a slider whose ends do not fit. The loader's clamp is used as the
statement of what the client accepts because it is the one the client applies to
the same field, in machine-readable form, for settings that have no schema row -
which these are, the six with no row being exactly the ones a Blizzard control
drives.

The declared range is not always the offered one. `kOptionRangeFixesLua`
rewrites some of these tables at start-up, because a slider can ship on a scale
this client does not measure in - the gamma slider offers -0.5 to 0.5 where
GetGamma answers 1 for a neutral screen, so every position on it asked for a
nearly black picture, which is this same fault in a control that is not bound
through kClientCVars. The overrides are read too, or this would compare against
numbers the client never uses.

It also checks that two sliders over one value offer the same range. One
setting has two: Mouse Sensitivity and Mouse Look Speed both write this client's
mouse sensitivity. Fixing the first with a scale while the second passed its
number through left them disagreeing - one read 1.0 where the other read 0.2,
and moving either put the other somewhere it could not show. That was a fix for
this file's own first finding, and it is the reason the check is here.

Canaried two ways: taking the scale off a binding reports that slider's ends
landing outside what the setting holds, and scaling one of a shared pair without
redefining its range reports the pair as disagreeing.

Data/interface is not in the repository, so this skips rather than fails when it
is absent.
"""

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import settings_config_parse  # noqa: E402  (a sibling, not a package)
from cpp_number import number  # noqa: E402  (same)

ROOT = Path(__file__).resolve().parent.parent
FRAMEXML = ROOT / "Data/interface/framexml"
API = ROOT / "src/addons/lua_system_api.cpp"
PANEL = ROOT / "src/ui/settings_panel.cpp"


def sliderRanges():
    """cvar -> (min, max), as the client actually offers them.

    The shipped tables are not the last word. kOptionRangeFixesLua rewrites some
    of them at start-up, because a slider can ship on a scale this client does
    not measure in - the gamma slider offers -0.5 to 0.5 where GetGamma answers
    1 for a neutral screen, so every position on it asked for a nearly black
    picture. Reading the .lua files alone would compare against numbers the
    client never uses.
    """
    out = {}
    for path in sorted(FRAMEXML.glob("*.lua")):
        text = path.read_text(errors="ignore")
        for m in re.finditer(
                r'(\w+)\s*=\s*\{[^}]*minValue\s*=\s*([-\d.]+)[^}]*maxValue\s*=\s*([-\d.]+)[^}]*\}',
                text):
            try:
                out.setdefault(m.group(1).lower(), (float(m.group(2)), float(m.group(3))))
            except ValueError:
                pass

    snippets = ROOT / "include/addons/addon_lua_snippets.hpp"
    if snippets.is_file():
        text = snippets.read_text()
        for m in re.finditer(r'\w+\.(\w+)\.minValue\s*=\s*([-\d.]+)', text):
            name, lo = m.group(1).lower(), float(m.group(2))
            hi = out.get(name, (lo, lo))[1]
            out[name] = (lo, hi)
        for m in re.finditer(r'\w+\.(\w+)\.maxValue\s*=\s*([-\d.]+)', text):
            name, hi = m.group(1).lower(), float(m.group(2))
            lo = out.get(name, (hi, hi))[0]
            out[name] = (lo, hi)

    # And kCVarRanges, which is the third and the strongest: the options panels
    # ask GetCVarMin and GetCVarMax for any control that names a cvar, so a row
    # here is the range the slider is actually drawn with. It is how this client
    # answers a shipped range that does not fit - view distance stopping at the
    # number the original renderer could reach, mouse sensitivity shipped as a
    # multiplier against a setting that is an amount.
    text = API.read_text()
    at = text.find("kCVarRanges[] = {")
    if at != -1:
        body = text[at:text.find("};", at)]
        for m in re.finditer(r'\{"([a-z0-9_]+)",\s*([-\d.]+)f?\s*,\s*([-\d.]+)f?\s*\}', body):
            try:
                out[m.group(1).lower()] = (float(m.group(2)), float(m.group(3)))
            except ValueError:
                pass
    return out


def bindings():
    """cvar -> (setting key, scale), from kClientCVars."""
    text = API.read_text()
    at = text.find("kClientCVars[] = {")
    if at == -1:
        return {}
    body = text[at:text.find("};", at)]
    out = {}
    for m in re.finditer(r'\{"([a-z0-9_]+)",\s*"([a-z0-9_]+)"\s*(?:,\s*([^}]+?))?\s*\}', body):
        scale = 1.0
        if m.group(3):
            scale = number(m.group(3))
        out[m.group(1)] = (m.group(2), scale)
    return out


def fields():
    """setting key -> (member, whether the binding is a fraction)."""
    text = PANEL.read_text()
    out = {}
    for m in re.finditer(
            r'\{\.key = "([a-z0-9_]+)",\s*\.as\w+\s*=\s*&SettingsPanel::(\w+)\s*(,\s*\.fraction = true)?',
            text):
        out[m.group(1)] = (m.group(2), bool(m.group(3)))
    return out



def sharedSettings():
    """setting -> the CVars that write it, where more than one does.

    Both the binding table and the branches in applyCVarSideEffects can write a
    client setting, and one setting is written by both: Mouse Sensitivity and
    Mouse Look Speed are two sliders over one value.

    A branch is taken as passing its number through unchanged, which is what
    the one that exists does. A branch that converted would need reading here -
    weatherDensity divides by three, but it drives a renderer rather than a
    setting, so it is not in this list.
    """
    text = API.read_text()
    out = {}
    at = text.find("kClientCVars[] = {")
    if at != -1:
        body = text[at:text.find("};", at)]
        for m in re.finditer(r'\{"([a-z0-9_]+)",\s*"([a-z0-9_]+)"', body):
            out.setdefault(m.group(2), []).append(m.group(1))
    for m in re.finditer(r'key == "([a-z0-9_]+)"[\s\S]{0,400}?setClientSetting\(\s*"([a-z0-9_]+)"',
                         text):
        out.setdefault(m.group(2), []).append(m.group(1))
    return {k: sorted(set(v)) for k, v in out.items() if len(set(v)) > 1}


def main():
    if not FRAMEXML.is_dir():
        print("the extracted interface is missing - no slider was compared.")
        return 0

    ranges, binds, flds = sliderRanges(), bindings(), fields()
    clmp = settings_config_parse.rangesByMember()
    if not binds or not flds or not clmp:
        print("kClientCVars, the binding table or the loader could not be read - "
              "nothing compared.")
        return 1

    bad, compared = [], 0
    for cvar, (key, scale) in sorted(binds.items()):
        if cvar not in ranges:
            continue  # a checkbox, or a slider the interface does not declare
        if scale is None:
            bad.append(f"{cvar}: the scale on its row could not be read")
            continue
        field = flds.get(key)
        if field is None:
            bad.append(f"{cvar}: {key} has no field binding to find its range by")
            continue
        member, fraction = field
        room = clmp.get(member)
        if room is None:
            continue  # a bool, which has no range to fall outside of

        lo, hi = ranges[cvar]
        factor = scale * (100.0 if fraction else 1.0)
        landed = (lo * factor, hi * factor)
        compared += 1
        if landed[0] < room[0] - 0.001 or landed[1] > room[1] + 0.001:
            bad.append(
                f"{cvar}: the slider runs {lo} to {hi}, which reaches {member} as "
                f"{landed[0]:g} to {landed[1]:g}, and the client holds that field "
                f"to {room[0]:g} to {room[1]:g}")

    # Two sliders over one value have to offer the same range, or moving either
    # puts the other somewhere it cannot show. This is the fault that came of
    # scaling Mouse Sensitivity while Mouse Look Speed passed its number
    # through: one read 1.0 where the other read 0.2.
    shared = 0
    for key, cvars in sorted(sharedSettings().items()):
        offered = {}
        for cvar in cvars:
            if cvar not in ranges:
                continue
            scale = binds.get(cvar, (None, 1.0))[1] or 1.0
            lo, hi = ranges[cvar]
            offered[cvar] = (round(lo * scale, 4), round(hi * scale, 4))
        if len(offered) < 2:
            continue
        shared += 1
        spans = set(offered.values())
        if len(spans) > 1:
            described = ", ".join(f"{c} offers {v[0]:g} to {v[1]:g}"
                                  for c, v in sorted(offered.items()))
            bad.append(f"{key} is written by {len(offered)} sliders and they do not "
                       f"agree: {described}")

    print(f"{compared} of Blizzard's sliders drive a setting with a range of its own, "
          f"and {shared} setting(s) are written by more than one.\n")
    for entry in bad:
        print(f"  {entry}")

    if bad:
        print(f"\n{len(bad)} slider(s) hand over a number the setting has no room for.")
        return 0

    if compared < 3:
        print("\nfewer sliders were compared than the bindings have - the parse "
              "stopped matching rather than finding nothing wrong.")
        return 1

    print("every slider the interface draws lands inside the range its setting holds.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
