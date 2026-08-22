#!/usr/bin/env python3
"""Bindings answering a number where FrameXML uses the answer as a table key.

Three faults in one day were this shape, and each cost a whole panel:

  * GetGuildRosterInfo's eleventh value is classFileName, and this answered
    the numeric class id. friendsframe.lua does

        if ( classFileName ) then
            classTextColor = RAID_CLASS_COLORS[classFileName];
        else
            classTextColor = NORMAL_FONT_COLOR;
        end
        ... buttonText:SetTextColor(classTextColor.r, ...)

    A number is truthy, so it took the first branch, found no such key, and
    read .r off a nil one line later - inside GuildStatus_Update, which took
    the guild roster down with it.

  * GetWhoInfo had the same fault at its seventh value, from a private copy
    of the class-token table.

  * UnitClass answered the string "UNKNOWN" for a unit it could not see, which
    is a key in no class table either. That one took Blizzard_ArenaUI down at
    load, so the arena frames did not exist at all.

THE SHAPE, AND WHY THE GUARD MAKES IT WORSE

Every one of these sites is guarded - `if ( x ) then TABLE[x]`. The guard is
the caller saying it already knows the value can be missing and has a branch
for that. An answer of the wrong kind is therefore worse than no answer: it
passes the guard, skips the branch written for it, and raises one line later
inside a function that looks unrelated to the binding at fault.

So this reads FrameXML for a variable that comes out of a binding at a known
position and is later used as a table index, and asks whether what the binding
pushes there is the kind of key that table is built with.

Both directions, because both fail identically - a lookup that finds nothing
and a field read off the nil a line later. A number into RAID_CLASS_COLORS is
the shape that cost three panels; a name into ITEM_QUALITY_COLORS, which is
indexed 0..6, is its mirror and no more survivable.

WHAT IT CANNOT SEE

The position is counted from the last run of pushes before the final return,
which is how a binding with guard paths above it is laid out. A body that
pushes conditionally in the middle of that run will be counted wrongly, and a
binding registered straight to a shared returner has no run at all.

Verified failable by the canary below, which is built rather than found: the
faults that named it are fixed, and a canary naming a live fault stops being
one the day it is fixed.
"""
import functools
import re
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
#     python3 tools/framexml_key_returns.py ~/wow-1.12/interface
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

_ADDON_SRC = "\n".join(p.read_text(errors="ignore")
                       for p in sorted((ROOT / "src/addons").glob("*.cpp")))

# name -> implementation symbol, for both registration styles.
bound = {}
for m in re.finditer(r'\{(?:\.\w+\s*=\s*)?"([A-Za-z0-9_]+)",\s*(?:\.\w+\s*=\s*)?(?:lua_)?([A-Za-z0-9_]+)\}', _ADDON_SRC):
    bound[m.group(1)] = m.group(2)

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

_cache = functools.lru_cache(maxsize=None)


@_cache
def body_of(name):
    return resolve_body(name, _ADDON_SRC, bound, inline_bodies)


@_cache
def push_kinds(name):
    """The kind pushed at each 1-based return position, or () if unreadable.

    Counted back from the last `return N`, taking the N pushes before it. That
    is the shape of a binding with guard paths above its real answer, which is
    most of them.
    """
    body = body_of(name)
    if not body:
        return ()
    returns = list(re.finditer(r"\breturn\s+(\d+)\s*;", body))
    if not returns:
        return ()
    last = returns[-1]
    count = int(last.group(1))
    if count == 0 or count > 32:
        return ()
    pushes = list(re.finditer(r"\blua_push(number|string|boolean|nil|integer|lstring)\b",
                              body[:last.start()]))
    if len(pushes) < count:
        return ()
    return tuple(p.group(1) for p in pushes[-count:])


# Read once. Both stages want every loaded file and reading them twice was
# most of the time this took.
FILES = [(p.name, p.read_text(errors="ignore")) for p in sorted(loaded_files(XML))]


# How each global table FrameXML declares is keyed, worked out from its own
# literal. This is the whole difference between a fault and a false one: a
# number is exactly right for ITEM_QUALITY_COLORS, which is indexed 0..6, and
# exactly wrong for RAID_CLASS_COLORS, which is indexed by "WARRIOR". Without
# asking, every quality, tier, column and power type in the interface reads as
# a fault - sixteen rows of them, all correct code.
def _table_key_kinds():
    kinds = {}
    for _name, text in FILES:
        for m in re.finditer(r"^([A-Z][A-Z0-9_]{3,})\s*=\s*\{(?:\.\w+\s*=\s*)?", text, re.M):
            name = m.group(1)
            # Only what is between the braces. Reading a fixed span after the
            # opening one instead meant an empty table - `LOCAL_MAP_QUESTS = {}`
            # - was classified from whatever unrelated code followed it in the
            # file, and both of the rows this reported were that: tables
            # FrameXML fills itself, keyed by the very ids it was calling a
            # fault.
            depth, i = 1, m.end()
            while i < len(text) and depth:
                if text[i] == "{":
                    depth += 1
                elif text[i] == "}":
                    depth -= 1
                i += 1
            body = text[m.end(): i - 1]
            numeric = len(re.findall(r"\[\s*-?\d+\s*\]\s*=", body))
            stringy = len(re.findall(r'\[\s*"[^"]+"\s*\]\s*=', body))
            stringy += len(re.findall(r"^\s*([A-Za-z_]\w*)\s*=\s*[\{\"]", body, re.M))
            if numeric or stringy:
                kinds[name] = "number" if numeric > stringy else "string"
    return kinds


TABLE_KEYS = _table_key_kinds()


# Every `SOME_TABLE[var]` in a file, found in one pass and looked up after.
#
# Searching for each variable at each call site instead meant a fresh scan of a
# kilobyte of Lua per name per site - thousands of them - and that was nearly
# all of the time this took.
_INDEX_USES = re.compile(r"([A-Z][A-Z0-9_]{3,})\s*\[\s*([A-Za-z_]\w*)\s*\]")


def _indexed_in(text):
    uses = {}
    for m in _INDEX_USES.finditer(text):
        uses.setdefault(m.group(2), []).append((m.start(), m.group(1)))
    return uses


def scan(files_text):
    """(binding, position, variable, kind, where) for each key used as an index."""
    found = []
    # local a, b, c = Fn(...)   /   a, b, c = Fn(...)
    assign = re.compile(r"(?:local\s+)?([a-zA-Z_]\w*(?:\s*,\s*[a-zA-Z_]\w*)+)\s*=\s*"
                        r"([A-Z][A-Za-z0-9_]*)\s*\(")
    for label, text in files_text:
        uses = _indexed_in(text)
        for m in assign.finditer(text):
            fn = m.group(2)
            if fn not in bound and fn not in inline_bodies:
                continue
            kinds = push_kinds(fn)
            if not kinds:
                continue
            names = [v.strip() for v in m.group(1).split(",")]
            tail = text[m.end(): m.end() + 1200]
            for pos, var in enumerate(names, start=1):
                if pos > len(kinds):
                    continue
                # Used as a table index somewhere below, and only where the
                # table being indexed is one FrameXML keys by name.
                table = None
                for at, tbl in uses.get(var, ()):
                    if m.end() <= at < m.end() + 1200:
                        table = tbl
                        break
                if table is None:
                    continue
                keyed = TABLE_KEYS.get(table)
                kind = kinds[pos - 1]
                # Both directions fail the same way - a lookup that finds
                # nothing, and a field read off the nil a line later. A number
                # into a table keyed by name is the shape that cost three
                # panels; a name into one keyed by number is its mirror and is
                # no more survivable, and asking about only one of them would
                # have been asking half the question.
                if keyed == "string" and kind not in ("string", "lstring", "nil"):
                    found.append((fn, pos, var, kind, table, label))
                elif keyed == "number" and kind in ("string", "lstring"):
                    found.append((fn, pos, var, kind, table, label))
    return found


hits = scan(FILES)

print(f"{len(bound)} C bindings parsed, "
      f"{sum(1 for v in TABLE_KEYS.values() if v == 'string')} name-keyed tables")

# Built rather than found, for the reason the docstring gives. Both halves are
# real - a binding that does push a number at a known position, and the Lua
# shape that indexes with it - so only FrameXML putting them together is
# synthetic, and every lookup the scan depends on is exercised.
# Every stage at once: a real binding that really does push a number at value
# four, a real table that FrameXML really does key by name, and only the line
# putting the two together invented. If this stops reporting, the sweep has
# stopped reading one of the three and the zero below means nothing.
_probe = scan([("<canary>",
                "local a, b, c, quality = GetLootSlotInfo(1)\n"
                "local col = RAID_CLASS_COLORS[quality]\n")])
if any(h[0] == "GetLootSlotInfo" and h[1] == 4 for h in _probe):
    print("canary: a number indexing a name-keyed table is seen - "
          "pushes, tables and call sites all read")
else:
    print("CANARY MISSING: the synthetic fault was not reported. The sweep is "
          "not reading one of its three inputs; the count below is meaningless.")
print()

print(f"{len(hits)} binding return(s) of the wrong kind for the table they index:\n")
for fn, pos, var, kind, table, where in sorted(set(hits)):
    print(f"  {fn}() value {pos} -> `{var}`, pushed as {kind}, indexes {table}")
    print(f"      {where}")
