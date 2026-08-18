#!/usr/bin/env python3
"""How far the Lua API is from running FrameXML.

FrameXML calls a few thousand globals, but most of them are its own - it
defines them itself as it loads. Subtracting those leaves the client API it
expects to already be there, which is the real measure of what is left to do.

Counting the raw call list instead gives a number several times too large and
makes the job look hopeless, which is why this subtracts.

Usage:  tools/framexml_api_gap.py [path to Interface/FrameXML]

Reports the size of the gap and the most-used names in it, so the work can be
ordered by what FrameXML actually leans on rather than by guesswork.
"""
import sys
from pathlib import Path as _Path
sys.path.insert(0, str(_Path(__file__).resolve().parent))
from framexml_source import without_comments_or_strings

import re, os, collections
#!/usr/bin/env python3
"""How far the Lua API is from running FrameXML.

FrameXML calls a few thousand globals, but most of them are its own - it
defines them itself as it loads. Subtracting those leaves the client API it
expects to already be there, which is the real measure of what is left to do.

Counting the raw call list instead gives a number several times too large and
makes the job look hopeless, which is why this subtracts.

Usage:  tools/framexml_api_gap.py [path to Interface/FrameXML]

Reports the size of the gap and the most-used names in it, so the work can be
ordered by what FrameXML actually leans on rather than by guesswork.
"""
import sys


# Lower case: the tree on disk is Data/interface/framexml, and the mixed-case
# spelling this defaulted to only ever resolved on a case-insensitive
# filesystem. Everywhere else it raised FileNotFoundError before reading a line.
fx = sys.argv[1] if len(sys.argv) > 1 else "Data/interface/framexml"
addons = os.path.join(os.path.dirname(__file__), "..", "src", "addons")

eng = ""
for f in os.listdir(addons):
    if f.endswith(".cpp"): eng += open(addons+"/"+f).read()
have  = set(re.findall(r'lua_setglobal\(L_?,\s*"(\w+)"', eng))
have |= set(re.findall(r'\{(?:\.\w+\s*=\s*)?"(\w+)"\s*,', eng))
have |= set(re.findall(r'"(\w+)\s*=', eng))
have |= set(re.findall(r'"function (\w+)\(', eng))

# What FrameXML itself defines - these are not gaps.
#
# The addon directory counts as "itself". Blizzard ships a dozen of its own
# addons and they define plenty of what framexml calls - the achievement UI,
# the combat log and the combat text between them held five of the ten
# most-called "missing" names, every one of them defined a directory away.
# Loaded on demand is still loaded.
defined, calls = set(), collections.Counter()
_lua = [os.path.join(fx, f) for f in sorted(os.listdir(fx)) if f.endswith(".lua")]
_addons = os.path.join(os.path.dirname(fx.rstrip("/")), "addons")
if os.path.isdir(_addons):
    for root, _dirs, files in os.walk(_addons):
        _lua += [os.path.join(root, f) for f in sorted(files) if f.endswith(".lua")]
for fn in _lua:
    if not fn.endswith(".lua"): continue
    # Comments and the insides of strings blanked, from the one place that has
    # both rules. Without it the fifteen most-called "missing" names included
    # Flanagan, Stephens, Master and TOP - the credits file and a handful of
    # anchor points, read as calls because a string is full of parentheses.
    src = without_comments_or_strings(open(fn, errors="ignore").read())
    # `local function` too. Anchoring on `function` alone missed every
    # file-local helper, and FrameXML declares plenty of them - GetHandleFrame
    # and GetUIPanelWindowInfo between them accounted for the two most-called
    # "gaps" in the report, neither of which was missing at all.
    defined |= set(re.findall(r'^\s*(?:local\s+)?function\s+([A-Za-z_][\w]*)\s*\(', src, re.M))
    defined |= set(re.findall(r'^\s*(?:local\s+)?([A-Za-z_][\w]*)\s*=\s*function', src, re.M))
    # ...and assigned from something else that already is one.
    # AchievementFrameTab_OnClick is `= AchievementFrameBaseTab_OnClick`, and
    # the achievement UI swaps it between two implementations as its tabs
    # change. An assignment defines the global whatever is on the right of it,
    # which is the only question here.
    defined |= set(re.findall(
        r'^\s*(?:local\s+)?([A-Za-z_][\w]*)\s*=\s*[A-Za-z_][\w.]*\s*;?\s*$',
        src, re.M))
    for m in re.finditer(r'(?<![\w.:])([A-Z][A-Za-z0-9_]{2,})\s*\(', src):
        calls[m.group(1)] += 1

missing = [(n,c) for n,c in calls.most_common() if n not in have and n not in defined]
print(f"called by FrameXML      : {len(calls)}")
print(f"defined by FrameXML     : {len(defined & set(calls))}")
print(f"already provided        : {len(set(calls) & have)}")
print(f"genuinely missing API   : {len(missing)}")
print(f"  calls they account for: {sum(c for _,c in missing)}")
print()
for n,c in missing[:20]: print(f"  {c:5d}  {n}")
