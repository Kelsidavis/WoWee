#!/usr/bin/env python3
"""Binding return values in the wrong position, where the count is right.

    tools/framexml_return_order.py

The companion to framexml_event_order.py, asking the same question of functions
instead of events. framexml_short_returns.py counts what a binding pushes
against what the interface unpacks; this asks whether the values that are there
are the right KIND for the places they land in.

FrameXML names what it destructures - `local name, rank, icon = GetSpellInfo(id)`
- and the binding pushes an expression per value. Paired positionally, the names
say what the expressions are supposed to be. A slot the interface calls name,
text, title or link that is handed something id-shaped is the fault this looks
for; the reverse is an id slot handed a name.

TWO THINGS THE FIRST DRAFT GOT WRONG, both of which made it report faults that
were not there:

**Every push in the function, rather than the ones on the return path.**
GetLFGProposal pushes eleven nils and returns before its real eleven values, so
every position after that shifted and each one looked wrong. It takes the
longest run of pushes with no `return` between them now.

**`rank` counted as a name.** It is a string in the spellcast events and a
number in GetGuildInfo and GetRaidRosterInfo, where WoW's own contract makes the
third value a rank *index*. A discriminator that means opposite things on the
two sides is not one, so it is gone - and the spellcast fault this class was
found through is still caught, on `name`, not on `rank`.

Seven hits became zero, and all seven were the tool.

WHAT IT CANNOT SEE

Two positions of the same kind swapped. A binding whose values come from a
helper rather than from pushes in its own body. And whether a value is right,
only whether it is the right kind.

VERIFIED BOTH WAYS: pointing GetLFGProposal's name slot at its dungeon id makes
this report it; putting it back empties the report.
"""
import pathlib
import re
import sys
import pathlib as _pathlib
sys.path.insert(0, str(_pathlib.Path(__file__).resolve().parent))
from framexml_source import without_comments, loaded_files

ROOT = pathlib.Path(__file__).resolve().parent.parent
XML, ADDONS = ROOT/"Data/interface", ROOT/"src/addons"

# C bindings: name -> ordered list of pushed expressions (best-effort)
bodies = {}
for p in ADDONS.rglob("*.cpp"):
    src = p.read_text(errors="ignore")
    # map registered name -> function name
    reg = dict(re.findall(r'\{\s*(?:\.\w+\s*=\s*)?"(\w+)"\s*,\s*(lua_\w+)\}', src))
    for fn_name, fn in reg.items():
        mm = re.search(r"^(?:static\s+)?int\s+" + re.escape(fn) + r"\(lua_State\*\s*L\)\s*\{", src, re.M)
        if not mm: continue
        depth, i = 1, mm.end()
        while i < len(src) and depth:
            if src[i] == '{': depth += 1
            elif src[i] == '}': depth -= 1
            i += 1
        bodies[fn_name] = src[mm.end():i]
    # inline lambdas: {"Name", [](lua_State* L) -> int { ... }}
    for m in re.finditer(r'\{\s*(?:\.\w+\s*=\s*)?"(\w+)"\s*,\s*\[\]\s*\(lua_State\*\s*L\)\s*->\s*int\s*\{', src):
        depth, i = 1, m.end()
        while i < len(src) and depth:
            if src[i] == '{': depth += 1
            elif src[i] == '}': depth -= 1
            i += 1
        bodies.setdefault(m.group(1), src[m.end():i])

PUSH = re.compile(r"lua_push(?:string|number|integer|boolean|nil|lstring)\s*\(\s*L\s*,?\s*([^;]*)\)\s*;")
def pushes(body):
    # The longest run of pushes with no `return` between them: the main path.
    # Collecting every push in the function counts the early-return block too
    # - GetLFGProposal pushes eleven nils and returns before the real eleven -
    # which shifts every position and invents a mismatch at each one.
    runs, cur, last = [], [], 0
    for m in PUSH.finditer(body):
        if "return" in body[last:m.start()]:
            runs.append(cur); cur = []
        cur.append(m.group(1).strip()); last = m.end()
    runs.append(cur)
    return max(runs, key=len) if runs else []

NAMEISH = re.compile(r"(?i)^name$|name$|text$|title$|link$")
IDISH = re.compile(r"(?i)id$|index$|slot$|count$")
def looks_id(e): return bool(re.search(r"(?i)\b[\w.:>()\[\]-]*(id|index|slot|count)\b", e)) and not re.search(r"(?i)name", e)
def looks_name(e): return bool(re.search(r"(?i)name|\.c_str\(\)|^\"", e))

rows, seen = [], set()
DEST = re.compile(r"local\s+([\w\s,_]+?)\s*=\s*(\w+)\s*\(")
for p in sorted(q for q in loaded_files(XML) if q.suffix.lower() == ".lua"):
    t = without_comments(p.read_text(errors="ignore"))
    for m in DEST.finditer(t):
        names = [x.strip() for x in m.group(1).split(",") if x.strip()]
        fn = m.group(2)
        if len(names) < 2 or fn not in bodies: continue
        if fn in ("strsplit", "strjoin", "select", "unpack"): continue
        pl = pushes(bodies[fn])
        if len(pl) < 2: continue
        for i, nm in enumerate(names):
            if i >= len(pl): break
            e = pl[i]
            if nm == "_" : continue
            if NAMEISH.search(nm) and looks_id(e) and not looks_name(e): why = "id in a name slot"
            elif IDISH.search(nm) and looks_name(e) and not looks_id(e): why = "name in an id slot"
            else: continue
            if (fn, i) in seen: continue
            seen.add((fn, i)); rows.append((fn, i+1, nm, e, p.name, why))
print(f"{len(bodies)} bindings read\n")
print(f"{len(rows)} return value(s) in the wrong position:\n")
for fn,i,nm,e,f,why in sorted(rows):
    print(f"  {fn:30} ret{i} '{nm}' <- {e[:40]}  ({why}) [{f}]")

if not rows:
    print("  (none)")
sys.exit(0)
