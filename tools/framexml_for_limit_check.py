#!/usr/bin/env python3
"""Missing globals used as a numeric for-limit, where nil is an error.

Most missing globals are survivable. The fallback makes any unknown name
callable and it answers nil, and FrameXML checks nil constantly - so a gap
usually shows up as a panel that stays empty.

`for i = 1, GetNumSomething()` is the exception. A nil limit is not an empty
loop; it raises "'for' limit must be a number" and takes down the handler
around it. GetNumTrackingTypes was unbound, so the minimap's tracking menu was
not empty but dead, and the only visible trace was a button that did nothing.

That is what this looks for: a call whose result reaches the limit position of
a numeric `for`, where the name is neither defined by the interface nor bound
by the client.

**Two forms, and the first version only found one of them.** The direct
`for i = 1, Foo()` is easy. But the bug that prompted this is written the other
way - `local count = GetNumTrackingTypes()` and then `for id = 1, count` on a
later line - so a version matching only the direct form reported six real
faults and stayed silent about the one it was written for. Both are checked
now, the indirect one within the same function.

**`or 0` is a guard too, and the tersest one.** channelframe.lua writes
`local numMembers = GetNumVoiceSessionMembersBySessionID(...) or 0;` - a
missing function answers nil, `or` replaces it, and the loop runs zero times.
Reading only the call and not the rest of the line reported it as a crash.

**A guarded variable is not a fault.** worldmapframe.lua writes
`local n = GetNumQuestItemDrops(...)` and then `if (n and n > 0) then` before
looping - nil is handled and nothing raises. Reporting it anyway was the
indirect check's own false positive, so a variable tested in an `if` between
its assignment and its loop is dropped.

Two things it deliberately does not report:

  * names the bootstrap aliases rather than binds - `getn` is table.getn, and
    the first version named it four times
  * systems that genuinely do not exist here (LFD, Battle.net). Those frames
    are unreachable, so the raise is unreachable too. They are still listed,
    because "unreachable" is a judgement about the rest of the client and this
    script is not the place to make it - read the call site before acting.

Usage:  tools/framexml_for_limit_check.py [path to Interface/FrameXML]
"""
import re
import sys
import pathlib

# Paths resolve against the repository, not the working directory.
REPO = pathlib.Path(__file__).resolve().parent.parent

import collections

fx = (pathlib.Path(sys.argv[1]) if len(sys.argv) > 1
      else REPO / "Data" / "interface" / "framexml")
if not fx.is_dir():
    sys.exit(f"no such directory: {fx}")

lua = list(fx.glob("*.lua")) + list(fx.glob("addons/*/*.lua"))
allsrc = lua + list(fx.glob("*.xml")) + list(fx.glob("addons/*/*.xml"))

# What the interface defines for itself.
defined = set()
for f in allsrc:
    t = f.read_text(errors="ignore")
    defined |= set(re.findall(r'^\s*(?:local\s+)?function\s+([A-Za-z_][\w.]*)\s*\(', t, re.M))
    defined |= set(re.findall(r'^\s*([A-Za-z_]\w*)\s*=\s*', t, re.M))
    defined |= set(re.findall(r'\b([A-Za-z_]\w*)\s*=\s*function', t))

# What the client binds, however it is spelled - including the bootstrap's
# aliases, which are why `getn` is not a hit.
src = "\n".join(p.read_text(errors="ignore")
                for p in (REPO / "src" / "addons").glob("*.cpp"))
provided = set(re.findall(r'\{(?:\.\w+\s*=\s*)?"(\w+)"', src))
provided |= set(re.findall(r'"(\w+)"\s*,\s*lua_', src))
provided |= set(re.findall(r'"function\s+(\w+)\s*\(', src))
provided |= set(re.findall(r'^\s*([A-Za-z_]\w*)\s*=', src, re.M))
provided |= set(re.findall(r'\{(?:\.\w+\s*=\s*)?"\w+",\s*(?:\.\w+\s*=\s*)?"(\w+)",\s*(?:\.\w+\s*=\s*)?"\w+"\}', src))
provided |= set(re.findall(r'\{(?:\.\w+\s*=\s*)?"\w+",\s*(?:\.\w+\s*=\s*)?"\w+",\s*(?:\.\w+\s*=\s*)?"(\w+)"\}', src))
# The counting table in lua_engine.cpp bootstraps a zero for every name in it,
# precisely so a nil never reaches a `for` limit. Missing it here reported
# GetLFDLockPlayerCount, GetNumRandomDungeons and GetNumQuestLogRewardFactions
# as raises when all three answer zero - the exact bug this sweep exists to
# find, reported against its own fix.
_counting = re.search(r"local counting = \{(.*?)\}", src, re.S)
if _counting:
    provided |= set(re.findall(r"'(\w+)'", _counting.group(1)))

LIMIT_DIRECT = re.compile(r'\bfor\s+\w+\s*=\s*[^,]+,\s*([A-Za-z_]\w*)\s*\(')
ASSIGN = re.compile(r'\blocal\s+([A-Za-z_]\w*)\s*=\s*([A-Za-z_]\w*)\s*\(')
LIMIT_VAR = re.compile(r'\bfor\s+\w+\s*=\s*[^,]+,\s*([A-Za-z_]\w*)\s*(?:do\b|$)')

hits = collections.defaultdict(list)
for f in lua:
    lines = f.read_text(errors="ignore").splitlines()
    # local name -> (function it came from, line). Cleared at each function
    # boundary so a variable cannot be matched against a call in another scope.
    pending = {}
    for i, line in enumerate(lines, 1):
        if re.match(r'\s*(?:local\s+)?function\b', line):
            pending.clear()
        if line.lstrip().startswith("--"):
            continue
        for fn in LIMIT_DIRECT.findall(line):
            if re.search(r'\)\s*or\s+\S', line):
                continue
            if fn not in defined and fn not in provided:
                hits[fn].append(f"{f.name}:{i}")
        for var, fn in ASSIGN.findall(line):
            # `= Foo() or 0` has already handled the nil.
            if re.search(r'\)\s*or\s+\S', line):
                pending.pop(var, None)
                continue
            if fn not in defined and fn not in provided:
                pending[var] = (fn, i)
            else:
                pending.pop(var, None)   # reassigned from something that answers
        # An `if` mentioning the variable is the author checking it. That is
        # the whole difference between a crash and an empty list, so it clears
        # the variable rather than merely noting it.
        if re.search(r'\bif\b', line):
            for var in list(pending):
                if re.search(r'\b' + re.escape(var) + r'\b', line):
                    pending.pop(var, None)
        for var in LIMIT_VAR.findall(line):
            if var in pending:
                fn, at = pending[var]
                hits[fn].append(f"{f.name}:{at} (via {var}, looped at line {i})")

print(f"scanned {len(lua)} Lua files")
if not hits:
    print("\nno unprovided global is used as a for-limit.")
else:
    print(f"\n{len(hits)} unprovided global(s) in a for-limit - each raises when reached:\n")
    for fn, where in sorted(hits.items(), key=lambda kv: -len(kv[1])):
        print(f"  {fn:32} {len(where)} site(s)")
        for w in where[:3]:
            print(f"      {w}")
    print("\nBinding one to 0 turns the raise into an empty list. Check first")
    print("whether the client has data to count - zero should be the truth,")
    print("not a way to silence the script.")
