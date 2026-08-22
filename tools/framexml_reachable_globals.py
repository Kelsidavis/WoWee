#!/usr/bin/env python3
"""Unbound globals reachable from a live file, following FrameXML's own calls.

The plain unbound sweep reports the file a name is *written* in, and judging
reachability from that is how two live raises were missed: GetQuestGreenRange
is written in uiparent.lua and reached from targetframe.lua through
GetQuestDifficultyColor; GetBindingByKey the same way through
GetBindingFromClick, reached from staticpopup.lua.

So walk it. Every FrameXML function that calls an unbound global is a carrier;
anything calling a carrier is a carrier. Report the ones a live file reaches,
with the chain.

THE ONE IT REPORTS TODAY, AND WHY IT IS NOT A FAULT

Calendar_Toggle, reached from ToggleCalendar in uiparent.lua - which tests for
it first:

    function ToggleCalendar()
        Calendar_LoadUI();
        if ( Calendar_Toggle ) then Calendar_Toggle(); end
    end

Calendar_Toggle exists only once Blizzard_Calendar has loaded, and LoadAddOn
refuses that addon by name, so the guard is what makes the minimap's date
button do nothing instead of raising. The report cannot see a guard on the
name it is reporting.

It read two hundred and two before it stopped reading files the loader never
opens - GlueXML, and the handful in framexml/ that no manifest lists. A report
of two hundred is one nobody triages.
"""
import re
import sys
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
#     python3 tools/framexml_reachable_globals.py ~/wow-1.12/interface
ROOT = Path(__file__).resolve().parent.parent
XML = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "Data/interface"
if not XML.is_dir():
    print(f"no interface at {XML} - name one on the command line")
    raise SystemExit(2)

# The elements handed over by default plus the candidates tier, mapped to
# files through the readiness tool's own table, and the shared files every
# panel goes through. A hand-made list of what is live is the mistake that hid
# the character sheet's other four tabs from one sweep and the whole verb
# surface from another - and listing *every* element is the opposite error,
# reporting friendsframe when social is deliberately not handed over.
def _live_files():
    """The FrameXML files that are actually drawn, derived rather than listed.

    This was a frozen literal of sixty-nine names with a comment above it
    claiming it was derived. It had been correct once. By the time mail and the
    barber shop joined the candidates tier it was two elements out of date, and
    a sweep that reports which live files reach an unbound global is worth
    exactly as much as its idea of "live".

    Reads the element lists from framexml_takeover.cpp and maps them to files
    through the readiness tool's own tables, so adding an element to the
    candidates tier brings its files here with no second edit.
    """
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import framexml_element_readiness as readiness

    takeover = (ROOT / "src/ui/framexml_takeover.cpp").read_text(errors="ignore")
    names = set()
    # Both the defaults block and the candidates block are lists of bare string
    # literals; every element name that appears in either is live.
    for block in re.findall(r"for \(const char\* name : \{(.*?)\}\)", takeover, re.S):
        names |= set(re.findall(r'"([a-z]+)"', block))

    files = set(readiness.SHARED)
    for name in names:
        for f in readiness.ELEMENTS.get(name, ()):
            files.add(f)
        addon = readiness.ADDON_ELEMENTS.get(name)
        if addon:
            files.add(f"{addon}.lua")
            files.add(f"{addon}.xml")
    return files


LIVE = _live_files()


def strip(t):
    t = re.sub(r"<!--.*?-->", "", t, flags=re.S)
    return without_comments_or_strings(t)


# One source of truth. This tool used to work it out from the C++ tables
# alone, which is why it reported GetNumStationeries as unbound and reachable
# from the mail frame - it had been answered by the bootstrap counting table
# all along, and acting on the report made things worse.
from framexml_provides import globals_provided, widget_methods_provided

bound = globals_provided() | widget_methods_provided()

# Every FrameXML function body, and the file it lives in.
bodies, defined = {}, set()
# Names a file declares as its own locals, kept per file rather than pooled.
# A local is not a global, and calling one is not a missing binding -
# blizzard_auctiondressup opens with `local DressUpItemLink_orig =
# DressUpItemLink` and calls it later, which read as a call to a global
# nothing answers and was the last non-glue entry in a report headed "these
# raise". Per file because a local in one is nothing in another, and pooling
# them would hide a genuinely missing global that shares a name.
locals_by_file = {}
for p in sorted(loaded_files(XML)):
    t = strip(p.read_text(errors="ignore"))
    defined |= set(re.findall(r"\bfunction\s+([A-Za-z_][A-Za-z0-9_]*)\s*\(", t))
    defined |= set(re.findall(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*=\s*", t, re.M))
    mine = set(re.findall(r"\blocal\s+function\s+([A-Za-z_][A-Za-z0-9_]*)", t))
    for group in re.findall(r"\blocal\s+([\w,\s]+?)\s*=", t):
        mine |= {n.strip() for n in group.split(",") if n.strip()}
    locals_by_file[p.name] = mine
    for m in re.finditer(r"\bfunction\s+([A-Za-z_][A-Za-z0-9_]*)\s*\((.*?)\nend", t, re.S):
        bodies[m.group(1)] = (p.name, m.group(2))

LUA = {"assert","ipairs","pairs","next","print","select","setmetatable","tonumber",
       "tostring","type","unpack","pcall","format","gsub","string","table","math",
       "date","time","abs","ceil","floor","max","min","mod","random","sqrt","bit",
       "wipe","tinsert","tremove","strsub","strlen","strupper","strlower","strfind",
       "strtrim","strjoin","strsplit","strrep","getmetatable","rawget","rawset",
       "error","loadstring","xpcall","strmatch","gmatch","tostringall"}

# Which functions call an unbound global directly.
carriers = {}
for fn, (fname, body) in bodies.items():
    for m in re.finditer(r"(?<![\w.:])([A-Z][A-Za-z0-9_]*)\s*\(", body):
        n = m.group(1)
        if (n not in bound and n not in defined and n not in LUA
                and n not in locals_by_file.get(fname, ())):
            carriers.setdefault(fn, set()).add(n)

# Who calls whom.
callers = {}
for fn, (fname, body) in bodies.items():
    for m in re.finditer(r"(?<![\w.:])([A-Za-z_][A-Za-z0-9_]*)\s*\(", body):
        callers.setdefault(m.group(1), set()).add(fn)

print(f"{len(bodies)} FrameXML functions, {len(carriers)} call something unbound\n")

reported = []
for fn, names in carriers.items():
    # Walk outward from the carrier to any live file, up to four hops.
    seen, frontier, chain = {fn}, [fn], {fn: [fn]}
    hit = None
    for _ in range(4):
        nxt = []
        for cur in frontier:
            if bodies.get(cur, ("", ""))[0] in LIVE:
                hit = chain[cur]
                break
            for up in callers.get(cur, ()):
                if up in seen:
                    continue
                seen.add(up)
                chain[up] = chain[cur] + [up]
                nxt.append(up)
        if hit:
            break
        frontier = nxt
    if hit:
        reported.append((sorted(names), " <- ".join(hit), bodies[hit[-1]][0]))

print(f"{len(reported)} carriers a live file reaches:\n")
for names, path, where in sorted(reported)[:30]:
    print(f"  {', '.join(names)}")
    print(f"      {path}   [{where}]")
