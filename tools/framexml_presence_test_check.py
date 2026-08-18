#!/usr/bin/env python3
"""Presence tests the shared frame metatable can never fail.

FrameXML asks whether an object has a method and means it:

    if ( slider.GetCurrentValue ) then ... elseif ( slider.cvar ) then

The first branch is for the handful of controls that define a closure of that
name themselves; everything else is meant to fall through. A method that lives
on the metatable every frame shares makes the question unanswerable - the test
passes for every frame, and the fall-through never happens.

That is not a hypothetical. GetCurrentValue and SetDisplayValue are StatusBar
methods and sat on the shared table, so BlizzardOptionsPanel_Slider_Refresh
read every options slider's value off a status bar - zero - and stored it, and
closing the panel wrote that back. Opening Video Options and closing it set the
UI scale to the smallest the range allows. Nothing raised; the settings simply
did not stick.

Both are now installed on the frame itself, for the types that own them, which
is what this sweep is here to keep true.

A hit is a question rather than an answer. Two of the three known ones are
ordinary defensive guards - a caller that does not know whether it holds a
texture or a frame - and are settled below. Anything not settled is either new
or was never judged.
"""
import collections
import pathlib
import re
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent

# Judged and left alone, with the reason.
SETTLED = {
    "SetVertexColor":
        "vehiclemenubar walks a table of frames that may be a texture or a "
        "frame and colours the ones that can be coloured. Every frame "
        "answering it means the call is made on a frame too, where it does "
        "nothing - the guard is defensive rather than load-bearing to a "
        "branch.",
    "SetScale":
        "The same walk, and every frame really does have SetScale in the real "
        "client as well. The guard reads as caution, not as a question about "
        "the type.",
    "GetValue":
        "Latent, and measured rather than assumed: of 75 option checkboxes, "
        "none reaches BlizzardOptionsPanel_CheckButton_Refresh without either "
        "a cvar or a GetValue closure of its own, and the SetupControl branch "
        "it opens acts only on checkboxes - the five controls that inherit it "
        "are dropdowns, for which that branch is empty. Left on the shared "
        "table because every caller in FrameXML is a slider, a scroll bar, a "
        "status bar or a cvar-backed control, all of which own the method.",
}


def shared_methods(src: str) -> set:
    """Methods answered by the metatable every frame shares."""
    found = set(re.findall(r'\{(?:\.\w+\s*=\s*)?"(\w+)",\s*(?:\.\w+\s*=\s*)?lua_\w+\}', src))
    found |= set(re.findall(r'set\("(\w+)"', src))
    found |= set(re.findall(r"function\s+[\w.]*[Mm][Tt]\w*\s*:\s*(\w+)", src))
    return found


def per_instance_methods(src: str) -> set:
    """Methods put on a frame's own table rather than on the shared one.

    Reported for the count only. They are deliberately NOT subtracted from the
    hits: a method can be in both places, and being on the shared table is what
    makes the presence test unanswerable no matter what else is true. Excluding
    them was this sweep's own first bug - putting GetCurrentValue back on the
    shared table raised nothing, because the per-instance install was still
    there and hid it.

    Capitalised only: the same call writes __wid and __name.
    """
    return set(re.findall(r'lua_setfield\(L, -2, "([A-Z]\w+)"\)', src))


def main() -> int:
    engine = REPO / "src" / "addons" / "lua_engine.cpp"
    if not engine.exists():
        print("lua_engine.cpp is not where this expects it; the zero below "
              "would mean the scan broke.")
        return 1
    src = engine.read_text(errors="ignore")
    shared = shared_methods(src)
    per_instance = per_instance_methods(src)
    if len(shared) < 100:
        print(f"Only {len(shared)} shared methods found, which cannot be "
              f"right - the scan broke rather than the code improving.")
        return 1

    interface = REPO / "Data" / "interface"
    lua = sorted(interface.rglob("*.lua"))
    lua += sorted((REPO / "Data" / "expansions").rglob("*/overlay/interface/**/*.lua"))
    if not lua:
        # Data/ is gitignored, so a checkout without the game's files has no
        # interface to read. That is the ordinary state on CI and is not a
        # broken scan - report nothing found and let the count stand at zero,
        # which is what every other sweep over this data does.
        #
        # Told apart from a real break by whether the directory is there at
        # all: present but empty means the glob stopped matching, and that is
        # worth failing over.
        if interface.is_dir():
            print("Data/interface is here but holds no Lua; the scan broke "
                  "rather than the interface emptying.")
            return 1
        print("0 presence test(s) the shared metatable always passes")
        print("  (no interface data in this checkout - Data/ is not tracked)")
        return 0

    # `if ( x.Method )` and `if not x.Method then` - asking whether it is
    # there, rather than calling it.
    test = re.compile(r"if\s*\(?\s*(?:not\s+)?(\w+)\.([A-Z]\w+)\s*\)?\s*(?:then|\))")
    hits = collections.defaultdict(list)
    for path in lua:
        try:
            text = path.read_text(errors="ignore")
        except OSError:
            continue
        for number, line in enumerate(text.split("\n"), 1):
            for match in test.finditer(line):
                name = match.group(2)
                if name in shared:
                    hits[name].append(f"{path.name}:{number}")

    unsettled = {k: v for k, v in hits.items() if k not in SETTLED}
    print(f"{len(lua)} interface files, {len(shared)} shared methods, "
          f"{len(per_instance)} put on the frame itself")
    print(f"{len(hits) - len(unsettled)} settled and not reported\n")
    print(f"{len(unsettled)} presence test(s) the shared metatable always passes:")
    if not unsettled:
        print("  (none)")
    for name, where in sorted(unsettled.items(), key=lambda kv: -len(kv[1])):
        print(f"  {name:26} {len(where):2} site(s)")
        for site in where[:4]:
            print(f"      {site}")

    print("\nA hit is a question. Read the branch it guards before moving the")
    print("method: some of these are a caller being careful about a type, and")
    print("some are a question the shared table has stopped it from asking.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
