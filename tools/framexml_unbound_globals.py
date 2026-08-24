#!/usr/bin/env python3
"""Globals FrameXML calls that nothing defines.

An unanswered call *raises*, where an unfired event only goes unheard - so
this is the sharpest of the sweeps. InspectUnit and StartDuel sat here for
months behind the unit right-click menu.

File-agnostic on purpose. Every per-element sweep so far has been crippled by
its own hand-written file list, so this one maps nothing: it reports the file
each name is called from and lets the reader decide whether that file is on
screen.

THE THIRTY-SIX IT REPORTS TODAY, EVERY ONE OPENED

Not grouped and dismissed - opened. Grouping is how five wrong claims survived
in this file's neighbours: "the refund window is server state this client is
never sent" was written once and cited four times, and it was one request away
the whole time.

  * Voice chat (5). AzerothCore's handlers read the request and throw it away;
    no SMSG_VOICE_* is ever sent. Verified in VoiceChatHandler.cpp.
  * Movie recording (5). The renderer captures one frame to a PNG and has no
    encoder. MovieRecording_* is video.
  * The GM survey's submit trio (3). Its questions DO come from four DBCs this
    install carries - absent by choice, not by necessity, and the reason is in
    framexml_live_stubs.
  * Vehicle seats (6). Vehicle state is tracked and fired; seats are not, and
    the seat indicator additionally needs UNIT_ENTERED_VEHICLE's sixth
    argument, which is not sent.
  * The refund window's client-side verbs (4). Refunds themselves are
    implemented; these end a window rather than read one.
  * The calendar (3). LoadAddOn refuses Blizzard_Calendar by name, and
    ToggleCalendar tests for Calendar_Toggle before calling it.
  * Skill-point purchase (3). WotLK has none, and skillframe.lua gates every
    one of these on GetAdjustedSkillPoints, which is correctly zero.
  * Recruit-a-friend (2), Battle.net (1), trade-window enchanting (2), and
    four singletons: TeleportToDebugObject is debug, StopCinematic has nothing
    to stop because SMSG_TRIGGER_CINEMATIC is acknowledged and skipped,
    GameMovieFinished has no movie to finish, and PickupEquipmentSet would put
    something on a cursor with no kind for it and no drop target that takes it.

WHAT IT CANNOT SEE: A GLOBAL A WIDGET METHOD IS NAMED AFTER

Bindings are found by their name in the C++ source, and a widget method
registers by name the same way a global does. So a global goes on reading as
bound the moment any widget method shares its name, and no amount of reading
this report finds it.

GetText was exactly that. FrameXML's GetText(token, gender) looks a global
string up with a gendered variant, and ReputationFrame_Update builds every
standing label with it - but set("GetText", lua_FontString_GetText) registers
a FontString method of that name, so this sweep counted it bound and the
reputation list raised on open regardless. The runtime missing-API report
caught it; this could not. Bound 2026-08-05.

The shape generalises: any name that is both a global and a method - GetText,
and whatever else grows into the collision later - is invisible here. The
runtime report is the only sweep that distinguishes them, because it records
what was actually asked for and of what.

A NOTE ON THE COUNT

It fell from 368 to 36 across one session, and most of that was not binding
things. 201 went when this stopped reading files the loader never opens, and
another handful when it learned that `local A, B = f()` declares two names
rather than one. A report is only as honest as its input set.
"""
import re
import sys
from pathlib import Path as _ToolPath
sys.path.insert(0, str(_ToolPath(__file__).resolve().parent))
from pathlib import Path

import sys as _sys
_sys.path.insert(0, str(Path(__file__).resolve().parent))
from framexml_source import without_comments_or_strings, loaded_files

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
#     python3 tools/framexml_unbound_globals.py ~/wow-1.12/interface
ROOT = Path(__file__).resolve().parent.parent
# --all prints every name in the second list rather than the first 25. The
# truncated form is the one to read while working; the whole list is what a
# report from someone else's interface needs to carry, and there was no way to
# ask for it - the cap was a literal, so the other 71 names could not be sent.
SHOW_ALL = "--all" in sys.argv[1:]
_paths = [a for a in sys.argv[1:] if not a.startswith("-")]
XML = Path(_paths[0]) if _paths else ROOT / "Data/interface"
if not XML.is_dir():
    print(f"no interface at {XML} - name one on the command line")
    raise SystemExit(2)

# Only the files the loader actually reaches.
#
# Still file-agnostic in the sense that matters - nothing here names a panel or
# maps a name to an element. What it does refuse to read is a file that never
# runs, and the loader's own manifests say which those are. Fifty-six of them
# are GlueXML: Blizzard's login, character select and character create, which
# AddonManager refuses by name because this client has its own. They put sixty
# names under "these raise as their panel opens", and not one of them can
# raise, because none of those files ever load.
SOURCES = sorted(loaded_files(XML))

# Bound on the C++ side.
# One source of truth - see framexml_provides. Working this out per tool is
# how six sweeps came to disagree about what the client answers.
from framexml_provides import globals_provided, widget_methods_provided

bound = globals_provided() | widget_methods_provided()

# Defined in FrameXML itself, as a function or assigned one.
defined = set()
for path in SOURCES:
    t = path.read_text(errors="ignore")
    defined |= set(re.findall(r"\bfunction\s+([A-Za-z_][A-Za-z0-9_]*)\s*\(", t))
    defined |= set(re.findall(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*=\s*", t, re.M))
    # Every name in a local declaration, not just the first. Lua declares
    # several at once - `local A, B = CreateRestrictedEnvironment(...)` in
    # restrictedexecution.lua - and reading only the first reported the second
    # as a global nothing defines.
    for names in re.findall(r"\blocal\s+([A-Za-z_][A-Za-z0-9_,\s]*)", t):
        for name in names.split(","):
            name = name.strip()
            if re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", name):
                defined.add(name)
    # Frame names become globals.
    defined |= set(re.findall(r'name="\$?parent?([A-Za-z0-9_]+)"', t))
    defined |= set(re.findall(r'name="([A-Za-z0-9_]+)"', t))

LUA = {"assert","collectgarbage","dofile","error","getfenv","getmetatable","ipairs",
       "load","loadstring","next","pairs","pcall","print","rawequal","rawget","rawset",
       "select","setfenv","setmetatable","tonumber","tostring","type","unpack","xpcall",
       "require","string","table","math","os","io","coroutine","debug","format","gsub",
       "strsub","strlen","strupper","strlower","strfind","strjoin","strsplit","strtrim",
       "strrep","strrev","strbyte","strchar","tinsert","tremove","tsort","wipe","date",
       "time","difftime","abs","ceil","floor","max","min","mod","random","sqrt","bit"}

def strip_comments(text: str) -> str:
    """Comments and the insides of string literals, from the one place that
    has both rules.

    Not cosmetic: two of the six names this flagged on the candidate elements
    were commented-out calls - --FCFDock_ForceTabSort and
    --GuildBankItemButton_OnUpdate - which read exactly like missing bindings
    and are not called at all. Strings for a worse reason: a Lua pattern is a
    string full of parentheses, so `strmatch(name, "DropDownList(%d+)")` read
    as a call to a global named DropDownList, and put "every dropdown in the
    interface raises as it opens" at the top of this report.
    """
    return without_comments_or_strings(text)


# Every function this interface hangs off a script handler, by name. A call
# inside one of these runs on its own the moment its panel loads, shows or
# hears an event; a call inside a dialog's OnAccept - or an OnClick, which is
# why those four events and no others are counted here - waits for someone to
# press a button that may never appear.
#
# This is the whole difference between the rows worth reading and the rest. The
# list below stood at four hundred and thirty-eight and was ignored for it,
# with SortBGList sitting in the middle: called from PVPBattlegroundFrame_OnShow,
# so the battleground panel raised as it opened.
AUTORUN = set()
for path in [p for p in SOURCES if p.suffix.lower() == ".xml"]:
    t = strip_comments(path.read_text(errors="ignore"))
    AUTORUN |= set(re.findall(r'<On(?:Load|Show|Event|Update)\s+function="([A-Za-z_]\w*)"', t))
    for body in re.findall(r"<On(?:Load|Show|Event|Update)[^>]*>(.*?)</On\w+>", t, re.S):
        AUTORUN |= set(re.findall(r"(?<![\w.:])([A-Z][A-Za-z0-9_]*)\s*\(", body))
for path in [p for p in SOURCES if p.suffix.lower() == ".lua"]:
    t = strip_comments(path.read_text(errors="ignore"))
    AUTORUN |= set(re.findall(
        r'SetScript\s*\(\s*"On(?:Load|Show|Event|Update)"\s*,\s*([A-Za-z_]\w*)', t))


def enclosing(text, pos):
    """The Lua function a position sits in, or None."""
    head = text.rfind("\nfunction ", 0, pos)
    local = text.rfind("\nlocal function ", 0, pos)
    start = max(head, local)
    if start < 0:
        return None
    m = re.match(r"\n(?:local )?function\s+([\w:.]+)", text[start:])
    return m.group(1) if m else None


calls = {}
autorun_hits = {}
for path in SOURCES:
    t = strip_comments(path.read_text(errors="ignore"))
    for m in re.finditer(r"(?<![\w.:])([A-Z][A-Za-z0-9_]*)\s*\(", t):
        calls.setdefault(m.group(1), set()).add(path.name)
        fn = enclosing(t, m.start())
        # A handler named directly, or one whose own name says what runs it.
        if fn and (fn in AUTORUN or re.search(r"_On(Load|Show|Event|Update)$", fn)):
            autorun_hits.setdefault(m.group(1), set()).add(f"{path.name}:{fn}")

missing = {n: f for n, f in calls.items()
           if n not in bound and n not in defined and n not in LUA}

print(f"{len(calls)} distinct globals called, {len(bound)} bound, "
      f"{len(defined)} defined in FrameXML\n")
print(f"{len(missing)} called and nowhere defined.\n")

# Split rather than sorted, because the two halves want different reactions.
live = {n: autorun_hits[n] for n in missing if n in autorun_hits}
print(f"{len(live)} of them from a function that runs on its own - these raise "
      f"as their panel opens:\n")
for n in sorted(live):
    print(f"  {n:<36} {', '.join(sorted(live[n])[:2])}")
if not live:
    print("  (none)")

rest = {n: f for n, f in missing.items() if n not in autorun_hits}
print(f"\n{len(rest)} reached only from something a player has to do first "
      f"(a dialog's accept, a menu click), or not reached at all:\n")
shown = sorted(rest, key=lambda k: (-len(rest[k]), k))
for n in (shown if SHOW_ALL else shown[:25]):
    print(f"  {n:<36} {', '.join(sorted(rest[n])[:3])}")
if not SHOW_ALL and len(rest) > 25:
    print(f"  ... and {len(rest) - 25} more - re-run with --all for the rest")
