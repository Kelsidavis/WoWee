#!/usr/bin/env python3
"""Bindings answering something a numeric comparison can never match.

Two shapes, one failure:

  * **a boolean**, where `x == 0` against `true`/`false` is silently false and
    the branch simply never runs. IsActionInRange is the case that named this:
    actionbutton.lua tests `valid == 0` and `valid == 1` for the out-of-range
    indicator, and a boolean would have failed both and hidden it forever.

  * **nil**, which is worse, because it fails the comparison in the *other*
    direction. `nil ~= 0` is **true** in Lua, so a binding answering nil where
    the reader asks `if ( x ~= 0 )` takes the branch meant for "there is one"
    every single time - carrying the nil into whatever is behind it.

    GetQuestWorldMapAreaID was that. WorldMap_OpenToQuest asks
    `if ( mapID ~= 0 )` and then `if ( floorNumber ~= 0 )`, and lua_ReturnNil
    ran both, on the path the quest tracker takes whenever a tracked quest is
    clicked. SetMapByID drops a zero id and SetDungeonMapLevel is a no-op, so
    nothing came of it - it was inert by luck, not by design, and the answer
    for "none" is now the pair of zeroes the reader is testing for.

The nil arm only sees bindings registered straight to a shared nil returner.
A hand-written body that pushes nil on some path is invisible here, and that
is the shape to widen to if this ever reports zero for long.

The boolean arm asks which return position was compared, because answering a
boolean first and numbers after is correct and common - GetLFGQueueStats leads
with hasData and then gives thirteen numbers. Knowing only that the body
pushes a boolean somewhere called five such bindings a fault. A boolean at a
position after the first is therefore out of reach here; the nil arm needs no
such test, since a binding registered to luaReturnNil answers nil everywhere.

THE SECOND ARM: HANDED ON RATHER THAN COMPARED

A comparison that can never match is the quiet failure - the branch does not
run and the code carries on. Passing the answer to another binding is the loud
one, except that nothing hears it: it raises inside the callee, and a raise
inside a click handler is swallowed, so the action simply does not happen.

GetChecked was that. FrameXML writes
`QueryAuctionItems(..., IsUsableCheckButton:GetChecked(), ...)` and
`SetCVar("questPOI", self:GetChecked())`, handing a widget's answer straight on
without touching it. luaL_optnumber falls back to its default for nil and none
only, so a boolean raised - and `false` is not nil, so the unticked box raised
just as the ticked one did. The auction browse never sent a search either way,
and reported no items and no error.

This arm is about the pair rather than either half: a boolean is a fine answer
until it reaches something that reads it as a number. Only the raising readers
count - lua_toboolean takes anything and luaL_optstring falls back for nil, so
matching those would bury the ones that matter.

THE TWO THE FIRST ARM REPORTS TODAY

  * GetMapDebugObjectInfo - unreachable. The loop that calls it runs to
    GetNumMapDebugObjects(), which answers zero, so `for i = 1, 0` never
    executes. It is reported because this sweep reads comparisons, not
    reachability.
  * GetWintergraspWaitTime - guarded the careful way, `if ( nextBattleTime and
    nextBattleTime > 60 )`, so nil takes the final else and the timer reads
    "in progress" forever. Wrong on screen and safe underneath, and there is
    no better answer: a number would be a more confident lie.

Verified failable by restoring GetQuestWorldMapAreaID's lua_ReturnNil, which
takes the report from two rows to three.
"""
import functools
import re
import subprocess
import sys
from pathlib import Path

# The repository this file sits in, and the interface to read.
#
# ROOT was one contributor's absolute home directory, so these eight sweeps ran
# on exactly one machine and silently read nothing anywhere else - loaded_files
# on a directory that is not there returns an empty set, and a sweep with no
# input reports a clean tree.
#
# The interface directory can be named on the command line, because the one
# under Data/ is whichever expansion was extracted last and the question these
# answer is usually about a particular one:
#
#     python3 tools/framexml_bool_vs_number.py ~/wow-1.12/interface
ROOT = Path(__file__).resolve().parent.parent
import sys as _s; _s.path.insert(0, str(Path(__file__).resolve().parent))
from framexml_source import loaded_files
import sys as _sys
import pathlib as _pathlib
_sys.path.insert(0, str(_pathlib.Path(__file__).resolve().parent))
from lua_binding_scan import resolve_body

XML = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "Data/interface"
if not XML.is_dir():
    print(f"no interface at {XML} - name one on the command line")
    raise SystemExit(2)

# Name(...) compared numerically, or a local assigned from it then compared.
numeric = {}
# Which return position was the one compared. A binding that answers a boolean
# first and numbers after is correct, and common - GetLFGQueueStats leads with
# hasData and then gives thirteen numbers - so knowing only that the body
# pushes a boolean somewhere reported five of those as faults. The position
# separates them: a boolean at position one that a caller compares to a number
# is the fault; a boolean at position one that nobody compares is the design.
numeric_pos = {}
for path in sorted(loaded_files(XML)):
    text = path.read_text(errors="ignore")
    for m in re.finditer(r"\b([A-Z][A-Za-z0-9_]*)\s*\([^()\n]*\)\s*(==|~=|>=?|<=?)\s*(-?\d+)", text):
        numeric.setdefault(m.group(1), set()).add(f"{path.name}: {m.group(0)[:60]}")
        numeric_pos.setdefault(m.group(1), set()).add(1)
    # local v = Name(...)  ... later  v == 0
    #
    # Every name on the left, not only the first. `local mapID, floorNumber =
    # GetQuestWorldMapAreaID(questID)` is the case: both are compared against
    # zero two lines down, and reading one name saw neither, because what
    # follows `local mapID` is a comma rather than an equals sign. This is the
    # same blind spot framexml_unbound_globals had, where it cost 555 names.
    for m in re.finditer(r"local\s+([a-zA-Z_]\w*(?:\s*,\s*[a-zA-Z_]\w*)*)\s*=\s*"
                         r"([A-Z][A-Za-z0-9_]*)\s*\(", text):
        fn = m.group(2)
        tail = text[m.end(): m.end() + 900]
        for pos, var in enumerate((v.strip() for v in m.group(1).split(",")), start=1):
            if re.search(rf"\b{re.escape(var)}\s*(==|~=|>=?|<=?)\s*-?\d", tail):
                numeric.setdefault(fn, set()).add(
                    f"{path.name}: local {var} = {fn}(...) then compared "
                    f"(return {pos})")
                numeric_pos.setdefault(fn, set()).add(pos)

# Which of those names are C bindings, and do they push a boolean?
src = subprocess.run(["grep", "-rn", "-A", "40", "static int lua_",
                      str(ROOT / "src/addons")], capture_output=True, text=True).stdout

# Every file, not the six that were listed by hand. The named list left out
# lua_engine.cpp, which is where every widget method lives - so this sweep
# could not see GetChecked, GetText, IsEnabled or any of their neighbours at
# all, and the widget methods are exactly the ones FrameXML hands straight to
# something else. It also left out the quest, lfg, and map domains.
_ADDON_SRC = "\n".join(p.read_text(errors="ignore")
                       for p in sorted((ROOT / "src/addons").glob("*.cpp")))

bound = {}
for m in re.finditer(r'\{(?:\.\w+\s*=\s*)?"([A-Za-z0-9_]+)",\s*(?:\.\w+\s*=\s*)?(?:lua_)?([A-Za-z0-9_]+)\}', _ADDON_SRC):
    bound[m.group(1)] = m.group(2)

# The inline form, whose body is right there rather than in a named function
# somewhere above. Matching only {"Name", lua_Name} asked this question of
# fewer than half the bindings while reporting a number that read as all of
# them - the same blind spot four other sweeps had, and a zero from half a
# search is not a zero.
inline_bodies = {}
for m in re.finditer(r'\{(?:\.\w+\s*=\s*)?"([A-Za-z0-9_]+)",\s*(?:\.\w+\s*=\s*)?\[\]\(lua_State\*\s*L?\s*\)\s*->\s*int\s*\{',
                     _ADDON_SRC):
    depth, i = 1, m.end()
    while i < len(_ADDON_SRC) and depth:
        if _ADDON_SRC[i] == "{":
            depth += 1
        elif _ADDON_SRC[i] == "}":
            depth -= 1
        i += 1
    inline_bodies[m.group(1)] = _ADDON_SRC[m.end():i - 1]
    bound.setdefault(m.group(1), m.group(1))

hits = []
for name in sorted(numeric):
    if name not in bound:
        continue
    impl = bound[name]
    # Before the body lookup, not after: a binding registered straight to a
    # shared returner has no body of its own to find, so asking for one and
    # skipping when it is missing discarded exactly the rows this arm is for.
    # Verified by restoring the fault and watching this report it.
    if impl in ("ReturnNil", "luaReturnNil"):
        hits.append((name, impl + " (answers nil)", sorted(numeric[name])[:2]))
        continue
    if name in inline_bodies:
        body = inline_bodies[name]
    else:
        body = subprocess.run(["grep", "-rn", "-A", "45", f"int {impl}(lua_State",
                               str(ROOT / "src/addons")], capture_output=True, text=True).stdout
    if not body:
        continue
    # Only when the compared return is the one the boolean sits at. The nil arm
    # above needs no such test - a binding registered straight to luaReturnNil
    # answers nil at every position, so any of them is the fault.
    if 1 not in numeric_pos.get(name, {1}):
        continue
    if "lua_pushboolean" in body and "lua_pushnumber" not in body.split("lua_pushboolean")[0][-400:]:
        hits.append((name, impl, sorted(numeric[name])[:2]))

# ---------------------------------------------------------------------------
# Arm two: answered a boolean, then handed to a binding that wants a number.
#
# The arm above reads comparisons, which is only one of the two ways the wrong
# type fails, and the quieter one - a comparison that can never match takes the
# other branch and carries on. Passing the answer on does not carry on: it
# raises inside the callee, and a raise inside a click handler is swallowed, so
# the whole action simply does not happen and nothing is logged.
#
# GetChecked was that. FrameXML writes
# `QueryAuctionItems(..., IsUsableCheckButton:GetChecked(), ...)` and
# `SetCVar("questPOI", self:GetChecked())`, handing a widget's answer straight
# to another binding without touching it. luaL_optnumber falls back to its
# default for nil and none only - a boolean is neither, so `false` raised just
# as `true` did, and the auction browse never sent a search whether the box was
# ticked or not.
#
# Note this arm is about the *pair*, not either half: a boolean is a fine
# answer until someone passes it somewhere that reads it as a number.


# Each of these three walks the whole of src/addons or a binding body, and the
# scan below asks them once per call site - tens of thousands of times, for a
# few hundred distinct answers. Without this the sweep took eighteen seconds,
# which is too slow for something wired into the build.
_cache = functools.lru_cache(maxsize=None)


def _split_args(text):
    """The top-level commas of an argument list, ignoring nested calls."""
    out, depth, start, instr = [], 0, 0, None
    for i, ch in enumerate(text):
        if instr:
            if ch == instr:
                instr = None
        elif ch in "\"'":
            instr = ch
        elif ch in "([{":
            depth += 1
        elif ch in ")]}":
            depth -= 1
        elif ch == "," and depth == 0:
            out.append(text[start:i])
            start = i + 1
    out.append(text[start:])
    return out


@_cache
def _body_of(name):
    """The binding body for a name, whether inline or a named function."""
    return resolve_body(name, _ADDON_SRC, bound, inline_bodies)


@_cache
def _answers_boolean(name):
    body = _body_of(name)
    return bool(body) and "lua_pushboolean" in body and "lua_pushnumber" not in body


@_cache
def _wants_scalar_at(name, index):
    """Does this binding read argument `index` as a number or a string?

    Only the raising readers count. lua_toboolean and lua_isstring take
    anything, and luaL_optstring falls back for nil - none of those is a
    fault, so matching them would bury the ones that are.
    """
    body = _body_of(name)
    if not body:
        return None
    pat = rf"luaL_(check|opt)(number|integer|string|int)\s*\(\s*L\s*,\s*{index}\b"
    for m in re.finditer(pat, body):
        kind, typ = m.group(1), m.group(2)
        if kind == "opt" and typ == "string":
            continue  # falls back for nil and takes a boolean without raising
        return f"luaL_{kind}{typ}(L, {index})"
    return None


def scan_handoffs(files_text):
    """(caller, arg index, callee, reader, where) for each boolean handed on."""
    found = []
    call = re.compile(r"\b([A-Z][A-Za-z0-9_]*)\s*\(([^()]*(?:\([^()]*\)[^()]*)*)\)")
    for label, text in files_text:
        for m in call.finditer(text):
            callee, args = m.group(1), m.group(2)
            if callee not in bound and callee not in inline_bodies:
                continue
            for idx, arg in enumerate(_split_args(args), start=1):
                inner = re.search(r"[:.]?\b([A-Z][A-Za-z0-9_]*)\s*\(\s*\)\s*$", arg.strip())
                if not inner:
                    continue
                producer = inner.group(1)
                if producer == callee or not _answers_boolean(producer):
                    continue
                reader = _wants_scalar_at(callee, idx)
                if reader:
                    found.append((producer, idx, callee, reader, label))
    return found


handoffs = scan_handoffs(
    [(p.name, p.read_text(errors="ignore")) for p in sorted(loaded_files(XML))])

# A zero here is only worth anything if the sweep can still see. Both stages
# are reported, and the canary is checked: IsActionInRange is the case that
# named this sweep, and it must at least reach the numeric-comparison set. If
# it stops appearing there, the Lua side has stopped parsing and the zero
# below means nothing.
print(f"{len(numeric)} names compared numerically in FrameXML, "
      f"{len(bound)} C bindings parsed")
canary = "IsActionInRange"
if canary in numeric:
    print(f"canary: {canary} seen compared numerically - the Lua side parses")
else:
    print(f"CANARY MISSING: {canary} not found compared numerically. "
          f"The sweep is not reading FrameXML; the count below is meaningless.")
print()

print(f"{len(hits)} binding(s) answer a boolean or nil and are compared numerically:\n")
for name, impl, where in hits:
    print(f"  {name}  ->  {impl}")
    for w in where:
        print(f"      {w}")

# The second arm's canary has to be built rather than found, because the case
# that named it is fixed and a canary that names a live fault stops being one
# the day it is fixed. Both halves are real - a binding that does answer a
# boolean, and a binding that does read its second argument with
# luaL_checknumber - so what is synthetic is only FrameXML calling one with the
# other, and every lookup the arm depends on is exercised.
print()
_probe = scan_handoffs([("<canary>", 'GetAuctionItemInfo("list", Foo:IsShown())')])
if _probe:
    print("canary: a boolean handed to a numeric argument is seen - arm two works")
else:
    print("CANARY MISSING: the synthetic hand-off was not reported. Arm two is "
          "not matching; the count below is meaningless.")

print(f"\n{len(handoffs)} boolean answer(s) handed to a binding that reads a number:\n")
for producer, idx, callee, reader, where in sorted(set(handoffs)):
    print(f"  {producer}() -> {callee}() argument {idx}, read as {reader}")
    print(f"      {where}")
