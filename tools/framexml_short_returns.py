#!/usr/bin/env python3
"""Bindings that return fewer values than FrameXML unpacks.

`local a, b, c = GetThing()` with a binding that pushes two leaves c nil, and
nil is only an error when something touches it - so this shows up as a missing
column, a nil concat much later, or nothing at all until the one path that
reads the last value.

Approximate by construction: it takes the largest `return N` in the function
body against the largest left-hand side in FrameXML. Both are upper bounds, so
a hit means "worth reading", not "wrong". Prints its own parse counts so a zero
can be told from a silent failure.

One false-positive shape, seen and worth knowing before acting:

  * **A widget method sharing a global's name.** The binding tables hold both
    globals and widget methods, and this cannot tell them apart.
    `GetCursorPosition` is a global returning x and y *and* an EditBox method
    returning one character offset; resolving the name to the method makes the
    global look short by one.

THE FOURTEEN IT REPORTS TODAY, ALL CHECKED

None is a fault. Read once so the count can be watched rather than re-triaged,
and so a fifteenth stands out.

  * A name that is both a global and a widget method (1). GetCursorPosition
    the global answers two values; GetCursorPosition the edit-box method
    answers one, and this cannot tell them apart - the same caveat
    api_shadowing_check carries. The caller in blizzard_battlefieldminimap
    reaches the global.
  * Missing values that are correctly nil (6). UnitName's second return is the
    realm, which is nil on your own; UnitPowerType's last three are the
    alternate power bar's colour; GetSpellLink's second is a trade-skill link;
    GetActionInfo's fourth is a vehicle spell id; QuestPOIGetIconInfo's fourth
    is an objective index; GetQuestWorldMapAreaID's second is a dungeon floor.
    Each caller either guards or passes the nil straight into something that
    does.
  * Features that are absent, answering one nil where several are unpacked (7).
    The knowledge base, the world map's debug objects and zone map, the
    battlefield vehicle list, the quest item drop. Answering more nils would
    change nothing: the first one already tells the caller to stop.
"""
import re
from pathlib import Path

import sys as _sys
_sys.path.insert(0, str(Path(__file__).resolve().parent))
from framexml_source import without_comments, loaded_files

ROOT = Path("/home/k/Desktop/wowee")
ADDONS = ROOT / "src/addons"
XML = ROOT / "Data/interface"

# name -> implementation symbol
bound = {}
for f in ADDONS.glob("*.cpp"):
    s = f.read_text(errors="ignore")
    for m in re.finditer(r'\{(?:\.\w+\s*=\s*)?"([A-Za-z0-9_]+)",\s*(?:\.\w+\s*=\s*)?(?:&)?\s*(lua_[A-Za-z0-9_]+)\}', s):
        bound[m.group(1)] = (m.group(2), f)
    # ...and the inline form, which is more than half of the bindings and was
    # invisible here for the same reason it was invisible to the argument
    # sweep: only a *named* implementation was matched, and a lambda has no
    # name. The name it is registered under stands in for one.
    for m in re.finditer(r'\{(?:\.\w+\s*=\s*)?"([A-Za-z0-9_]+)",\s*(?:\.\w+\s*=\s*)?\[\]\s*\(lua_State\*\s*L\)\s*->\s*int\s*\{', s):
        bound[m.group(1)] = (m.group(1), f)
    # And the third registration form: a global, set one at a time rather than
    # through a table.
    #
    # Last, so it wins a name that is also a widget method. Both spellings of
    # GetCursorPosition exist - an EditBox method answering where the caret is,
    # and a global answering where the mouse is - and reading only the tables
    # found the method, whose one return was then held against
    # `xPos, yPos = GetCursorPosition()`. That call is a global call, and the
    # global returns both. A destructured call site is never a method call, so
    # the global is the right answer wherever there is one.
    for m in re.finditer(
            r'lua_pushcfunction\(\s*L_?\s*,\s*(lua_[A-Za-z0-9_]+)\s*\)\s*;\s*'
            r'lua_setglobal\(\s*L_?\s*,\s*"([A-Za-z0-9_]+)"\s*\)', s):
        bound[m.group(2)] = (m.group(1), f)

#: `return n;` - a count worked out at run time, which no reading of the source
#: can pin down. A body with one of these is uncountable and is left out
#: entirely rather than being scored on whatever literal returns it also has.
#:
#: Those literals are almost always guards. GetGossipOptions pushes two values
#: per option and ends `return n;`, and its only literal return is the `if
#: (!gh) return 0;` above the loop - so it read as answering nothing at all,
#: against a call site unpacking two.
COMPUTED = re.compile(r"\breturn\s+(?!\d)[A-Za-z_]\w*\s*;")

# implementation -> max returned count
pushes = {}
for f in ADDONS.glob("*.cpp"):
    s = f.read_text(errors="ignore")
    for m in re.finditer(r"\bint\s+(lua_[A-Za-z0-9_]+)\s*\(lua_State\*\s*L\s*\)\s*\{(?:\.\w+\s*=\s*)?", s):
        name = m.group(1)
        # crude body slice: to the next top-level function definition
        nxt = s.find("\nint ", m.end())
        nxt2 = s.find("\nstatic int ", m.end())
        end = min(x for x in [nxt, nxt2, len(s)] if x != -1)
        body = s[m.end():end]
        # Plain `return N;` and the conditional form `return c ? N : M;`.
        # Only the first was matched, and UnitCastingInfo ends
        # `return wantChannel ? 8 : 9;` - so a binding returning nine values
        # read as returning none and never appeared here, while the cast bar
        # was raising on a shifted endTime.
        rets = [int(x) for x in re.findall(r"\breturn\s+(\d+)\s*;", body)]
        for a, b in re.findall(r"\breturn\s+[^;]*\?\s*(\d+)\s*:\s*(\d+)\s*;", body):
            rets += [int(a), int(b)]
        if rets and not COMPUTED.search(body):
            pushes[name] = max(rets)
    # The inline bodies, keyed by the name they are registered under. Braces
    # are matched rather than scanning to the next definition, because a lambda
    # sits inside a table and has no next definition to stop at.
    for m in re.finditer(r'\{(?:\.\w+\s*=\s*)?"([A-Za-z0-9_]+)",\s*(?:\.\w+\s*=\s*)?\[\]\s*\(lua_State\*\s*L\)\s*->\s*int\s*\{', s):
        depth, i = 1, m.end()
        while i < len(s) and depth:
            if s[i] == "{":
                depth += 1
            elif s[i] == "}":
                depth -= 1
            i += 1
        body = s[m.end():i - 1]
        rets = [int(x) for x in re.findall(r"\breturn\s+(\d+)\s*;", body)]
        for a, b in re.findall(r"\breturn\s+[^;]*\?\s*(\d+)\s*:\s*(\d+)\s*;", body):
            rets += [int(a), int(b)]
        if rets and not COMPUTED.search(body):
            pushes[m.group(1)] = max(rets)

def strip_comments(text: str) -> str:
    """Drop Lua and XML comments.

    The unbound sweep learned this and this one had not: mainmenubar.lua carries
    a commented-out `--exhaustionCurrXP, exhaustionMaxXP = GetXPExhaustion()`
    above the live single-value call, so GetXPExhaustion read as short by one
    against a line nothing runs.
    """
    text = re.sub(r"<!--.*?-->", "", text, flags=re.S)
    return without_comments(text)


def top_level_split(rhs):
    """The right-hand side split at commas outside brackets, strings and calls.

    A comma inside an argument list separates arguments, not expressions, and
    counting it as one made every multi-argument call look like several.
    """
    parts, depth, quote, start = [], 0, None, 0
    for i, c in enumerate(rhs):
        if quote:
            if c == quote and rhs[i - 1] != "\\":
                quote = None
        elif c in "\"'":
            quote = c
        elif c in "([{":
            depth += 1
        elif c in ")]}":
            depth -= 1
            if depth < 0:  # the statement ended inside an enclosing call
                break
        elif c == "," and depth == 0:
            parts.append(rhs[start:i])
            start = i + 1
    parts.append(rhs[start:i + 1 if depth < 0 else len(rhs)])
    return parts


# FrameXML: max destructured count per call
unpack = {}
# Only files the loader opens. GlueXML - login, character select, realm
# list - sits in the same tree and is refused by name, so a name it calls
# or a value it unpacks says nothing about this client.
for path in sorted(loaded_files(XML)):
    t = strip_comments(path.read_text(errors="ignore"))
    for m in re.finditer(r"(?:local\s+)?([A-Za-z_][\w., \t]*?)\s*=\s*([^\n]+)", t):
        lhs, rhs = m.group(1), m.group(2)
        if "." in lhs or "[" in lhs:
            continue
        names = [p for p in lhs.split(",") if p.strip()]
        if len(names) <= 1:
            continue
        exprs = top_level_split(rhs)
        # Lua adjusts every expression but the last to exactly one value, so
        # only the last one can absorb more than a single name. Two calls on a
        # line used to credit the whole left-hand side to the first of them.
        for i, expr in enumerate(exprs):
            call = re.fullmatch(r"\s*([A-Z][A-Za-z0-9_]*)\s*\((.*)", expr, re.S)
            if not call:
                continue
            fn = call.group(1)
            n = 1 if i < len(exprs) - 1 else len(names) - (len(exprs) - 1)
            if n <= 1:
                continue
            prev = unpack.get(fn)
            if not prev or n > prev[0]:
                site = f"{lhs.strip()} = {rhs.strip()}"
                unpack[fn] = (n, f"{path.name}: {site[:70]}")

print(f"parsed {len(bound)} bindings, {len(pushes)} bodies, "
      f"{len(unpack)} destructured call sites\n")

#: Bindings answering short on purpose, checked one at a time.
#:
#: A set rather than a count. A ceiling says only how many there are, so fixing
#: one and introducing another leaves the number where it was -
#: handler_announce_check was pinned that way and hid a live bug for as long as
#: it was.
EXPECTED_SHORT = {
    # Battle.net. A 3.3.5 server has none of it, so every one of these answers
    # nothing and every caller guards for that.
    "BNGetFriendInfo": "no Battle.net",
    "BNGetFriendInfoByID": "no Battle.net",
    "BNGetToonInfo": "no Battle.net",
    "BNGetFriendInviteInfo": "no Battle.net",
    "BNGetConversationMemberInfo": "no Battle.net",
    # The in-game knowledge base. There is no article store behind it.
    "KBArticle_GetData": "no knowledge base",
    "KBSetup_GetCategoryData": "no knowledge base",
    "KBSetup_GetSubCategoryData": "no knowledge base",
    # Blizzard's own debug overlay, shipped switched off.
    "GetDebugZoneMap": "debug overlay, no data",
    "GetMapDebugObjectInfo": "debug overlay, no data",
    # Battleground vehicles are not tracked.
    "GetBattlefieldVehicleInfo": "vehicles not tracked",
    # Two of five. The last three are the colour of a custom power bar, which
    # only vehicles and a few encounters have; a nil leaves unitframe on the
    # power type's own colour, which is the answer for everything here.
    "UnitPowerType": "alt power colours, absent by design",
    # One of three. Answering nil is the answer - the item objectives *are* the
    # leaderboard objectives, listed by the branch above this one, so there is
    # nothing left for this to describe.
    "GetQuestLogItemDrop": "no drops left to name",
    # One of two. The second is the realm, which is nil for a player on your
    # own realm - and there is no other kind here.
    "UnitName": "realm is nil for a same-realm player",
    # One of two. The second is a trade skill link, which a spell only has when
    # it is a recipe being viewed in a trade skill window.
    "GetSpellLink": "no trade skill link for a plain spell",
}

hits = []
for fn, (n, where) in unpack.items():
    if fn not in bound or fn in EXPECTED_SHORT:
        continue
    impl, _ = bound[fn]
    if impl not in pushes:
        continue
    if n > pushes[impl]:
        hits.append((fn, pushes[impl], n, where))

hits.sort(key=lambda h: h[2] - h[1], reverse=True)
print(f"{len(hits)} binding(s) may return short:\n")
for fn, got, want, where in hits:
    print(f"  {fn:<32} pushes {got}, unpacked {want}")
    print(f"      {where}")
