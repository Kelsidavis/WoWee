#!/usr/bin/env python3
"""Names defined in more than one place, where load order decides the winner.

Three times now a feature has been dead because something defined a name twice
and the copy that won was the empty one. The bug is invisible from either site:
each looks like the only definition, and nothing errors - the call just does
nothing, which reads as an unimplemented feature rather than a shadowed one.

Three mechanisms, all found in this codebase:

  bootstrap over binding   The Lua in lua_engine.cpp runs after the C tables are
                           registered, so a stub there replaces the real
                           implementation. Twelve did, including the pet bar and
                           the stance bar.

  table field over metatable   A field on a frame's own table beats the frame
                           metatable. GameTooltip carried five of these and they
                           shadowed every tooltip binding.

  binding against binding  The same name in two C tables. GetActionBarPage was
                           registered twice against different storage, so
                           changing the page moved one number and reading it
                           returned the other.

Not every hit is a fault: two definitions that agree are clutter, and a Lua one
that calls through to the real methods is fine. The point is to see them.

    python3 tools/api_shadowing_check.py
"""

import re
import sys
from pathlib import Path

ADDONS = Path(__file__).resolve().parent.parent / "src" / "addons"


def sources():
    return sorted(ADDONS.glob("*.cpp"))


def scan():
    binding_names = {}          # name -> [files]
    bootstrap_globals = {}      # name -> [files]
    counting_stubs = set()      # names the bootstrap answers zero for
    metatable_methods = set()
    zero_bindings = set()       # bindings that answer a constant
    table_methods = {}          # object -> {method: file}

    for path in sources():
        text = path.read_text(errors="replace")

        for name in re.findall(r'\{\s*(?:\.\w+\s*=\s*)?"([A-Za-z_]\w*)"\s*,\s*(?:lua_|\[)', text):
            binding_names.setdefault(name, []).append(path.name)
        # A binding that is itself a plain stub answers the same as the
        # counting list, so one shadowing the other changes nothing.
        for name in re.findall(
                r'\{\s*"([A-Za-z_]\w*)"\s*,\s*lua_Return(?:Zero|Nil|Nothing|False)\b', text):
            zero_bindings.add(name)
        for name in re.findall(r'\{\s*(?:\.\w+\s*=\s*)?"([A-Za-z_]\w*)"\s*,\s*lua_', text):
            metatable_methods.add(name)
        for name in re.findall(r'"function\s+[\w.]*[Mm][Tt]\w*\s*:\s*([A-Za-z_]\w*)', text):
            metatable_methods.add(name)

        # Globals the bootstrap Lua defines: "function Name(" and "Name = function".
        #
        # Except where the chunk captures the binding first -
        #     local markPortrait = SetPortraitTexture
        #     function SetPortraitTexture(texture, unit) ... markPortrait(...)
        # - which composes with it rather than replacing it. Redefining a name
        # is only a fault when what was underneath stops running, and that is
        # the difference between the two shapes.
        captured = set(re.findall(r'"local\s+\w+\s*=\s*([A-Z]\w*)\\n"', text))
        for name in re.findall(r'"function\s+([A-Za-z_]\w*)\s*\(', text):
            if name in captured:
                continue
            bootstrap_globals.setdefault(name, []).append(path.name)
        for name in re.findall(r'"([A-Za-z_]\w*)\s*=\s*function', text):
            bootstrap_globals.setdefault(name, []).append(path.name)

        # The counting stubs, defined by looping over a list of names rather
        # than one at a time. They were bootstrap Lua like any other, so one
        # that shared a name with a C binding won and answered zero forever -
        # GetNumMacroIcons did exactly that, leaving the icon picker empty
        # beside a working GetMacroIconInfo.
        #
        # That loop is guarded now: `if rawget(_G, name) == nil then`, so a
        # name a binding already defines is skipped and the stub is installed
        # only where there is nothing. Reading the list without reading the
        # guard reported twenty-three bindings as shadowed when none was -
        # including four bound the same day, which read as though the day's
        # work had been overwritten. So the guard is what decides whether the
        # list counts, and it is looked for here.
        for block in re.findall(
                r"local counting = \{(.*?)\}(.*?)\bend\\n", text, re.S):
            names, body = block
            if "rawget(_G, name) == nil" in body:
                continue   # installed only where nothing is defined
            for name in re.findall(r"'(\w+)'", names):
                counting_stubs.add(name)

        # Methods hung directly on a named global's table.
        for obj, meth in re.findall(r'"function ([A-Z]\w*):([A-Za-z_]\w*)\(', text):
            table_methods.setdefault(obj, {})[meth] = path.name

    return (binding_names, bootstrap_globals, metatable_methods,
            table_methods, counting_stubs, zero_bindings)


def main():
    bindings, boot, mt, tables, counting, zeroes = scan()
    # A counting stub over a binding matters only when the binding
    # does real work; two ways of answering zero are not a fault.
    for name in counting:
        # Not filtered by metatable_methods: that set holds every binding,
        # so testing against it silenced the very case this catches.
        if name in bindings and name not in zeroes:
            boot.setdefault(name, []).append('lua_engine.cpp (counting)')
    problems = 0

    over_binding = sorted(set(bindings) & set(boot))
    if over_binding:
        problems += len(over_binding)
        print("bootstrap Lua shadows a binding "
              "(the bootstrap runs later, so it wins):")
        for n in over_binding:
            print(f"    {n:<28} binding in {', '.join(sorted(set(bindings[n])))}")

    twice = sorted(n for n, files in bindings.items() if len(files) > 1)
    if twice:
        problems += len(twice)
        print("\nregistered as a binding more than once "
              "(the later registration wins):")
        for n in twice:
            print(f"    {n:<28} {', '.join(sorted(set(bindings[n])))}")

    field_hits = {o: sorted(m for m in ms if m in mt) for o, ms in tables.items()}
    field_hits = {o: ms for o, ms in field_hits.items() if ms}
    if field_hits:
        problems += sum(len(ms) for ms in field_hits.values())
        print("\na table field shadows the frame metatable "
              "(a field always wins):")
        for o in sorted(field_hits):
            print(f"    {o}: {' '.join(field_hits[o])}")

    # The one hit here that is never ambiguous: a name bound in C into
    # frameMethods and then redefined as Lua on the same metatable. The
    # bootstrap runs afterwards, so the Lua one always wins - and these are
    # written as no-ops, which turns a working method into silence. It has
    # happened twice: EnableMouse, so no frame took the mouse, and SetBackdrop
    # with its two colour setters, so no panel drew a background.
    eng = (ADDONS / "lua_engine.cpp").read_text(errors="ignore")
    m = re.search(r"static const struct luaL_Reg frameMethods\[\] = \{(.*?)\n    \};",
                  eng, re.S)
    if m:
        c_bound = set(re.findall(r'\{(?:\.\w+\s*=\s*)?"(\w+)"', m.group(1)))
        after = eng[eng.index('"local mt = __WoweeFrameMT'):] \
            if '"local mt = __WoweeFrameMT' in eng else ""
        lua_defined = set(re.findall(r'"function\s+[\w.]*[Mm][Tt]\w*\s*:\s*(\w+)', after))
        both = sorted(c_bound & lua_defined)
        if both:
            problems += len(both)
            print("\nbound in C and then redefined as Lua on the frame metatable"
                  "\n(the Lua one runs later and wins - this is always a fault):")
            for n in both:
                print(f"    {n}")

    print("\nA caveat this cannot see past: frame methods and globals are"
          "\nregistered the same way here, so HasFocus the edit-box method and"
          "\nHasFocus the focus-target query look like one name in two places."
          "\nCheck what each is before treating a hit as a fault.")

    if not problems:
        print("no name is defined in two places that could disagree")
    else:
        print(f"\n{problems} to look at - each is a name whose winner depends on "
              "load order.\nSome are harmless duplication; the dangerous ones are "
              "a stub over an implementation,\nand a setter and getter that end up "
              "on opposite sides.")
    # Never fails the build: most hits are clutter, and a check that cries wolf
    # gets muted. This is for reading.
    return 0


# ── the same method defined twice in bootstrap Lua ──────────
#
# Every "function mt:X" lands on one metatable, so a second definition wins and
# the first is dead the moment it is written. That is invisible to the checks
# above, which compare C bindings against Lua rather than Lua against itself.
#
# It matters because the loser is not always harmless. A superseded
# GetAttribute took one argument where the real one takes three and kept its
# values under a different key - and it sat on the path every unit frame's
# click goes through, so reordering the two chunks would have stopped
# targeting and right-click menus with nothing to say why.
import collections as _collections
import pathlib as _pathlib
# Resolved from this file rather than the working directory. Both of the
# paths below were relative to it, so the report crashed anywhere but the
# repository root - which is where anything running it from a build
# directory finds out, and only then.
_src = (ADDONS / "lua_engine.cpp").read_text()
_defs = _collections.Counter(
    re.findall(r'"function\s+[\w.]*[Mm][Tt]\w*\s*:\s*(\w+)', _src))
_dupes = sorted(n for n, c in _defs.items() if c > 1)
print("\nbootstrap Lua defining the same metatable method more than once:")
if _dupes:
    for n in _dupes:
        print(f"    {n:28} {_defs[n]} definitions - the last one wins")
else:
    print("    (none)")

# ── events fired by hand into one registry ──────────────────
#
# There are two: __WoweeEvents, which frame:RegisterEvent from an addon fills,
# and __WoweeFrameEvents, which FrameXML's own frames fill. LuaEngine::fireEvent
# delivers to both. Anything that walks one of those tables itself reaches half
# the listeners.
#
# ChangeActionBarPage did exactly that, and the six FrameXML frames registered
# for ACTIONBAR_PAGE_CHANGED - including the one that redraws every action
# button - never heard it. The page number changed and the icons did not.
_here = ADDONS
_offenders = []
for _f in sorted(_here.glob("*.cpp")):
    if _f.name == "lua_engine.cpp":
        continue  # the dispatcher itself
    for _i, _line in enumerate(_f.read_text(errors="ignore").splitlines(), 1):
        if "__WoweeEvents" not in _line and "__WoweeFrameEvents" not in _line:
            continue
        if _line.lstrip().startswith("//"):
            continue  # a comment explaining the rule is not a breach of it
        _offenders.append(f"{_f.name}:{_i}: {_line.strip()[:70]}")
print("\nevent tables touched outside the dispatcher:")
if _offenders:
    for _o in _offenders:
        print("    " + _o)
    print("    → call engine->fireEvent instead; it delivers to both.")
else:
    print("    (none - every event goes through fireEvent)")


if __name__ == "__main__":
    sys.exit(main())

