#!/usr/bin/env python3
"""Values from a binding that can be nil, used where nil raises.

The nil-use sweep asks whether a *missing* function is called somewhere fatal.
This asks the harder question: a binding that exists, answers nil, and whose
answer is then added to something or concatenated into a string.

Nil is survivable almost everywhere in FrameXML - a guard fails, a branch is
skipped, a label goes blank. It is not survivable in two places:

    "prefix"..value        attempt to concatenate a nil value
    base + var * count     attempt to perform arithmetic on a nil value

Both raise and take the handler with them. Both are what the dungeon-ready
dialog did, twice, in the same function four lines apart: the texture path was
built by concatenation and the reward total by arithmetic, and each was a
binding that answered nil because nothing had filled it in yet.

WHAT IT MATCHES

A destructure from a call - `local a, b, c = Foo(...)` - followed inside the
same function by one of those two operations on one of those names, with no
`if a`, `if not a`, `a or`, or `a and` between. The guard test is deliberately
generous: FrameXML guards constantly and a sweep that ignores guards reports
hundreds of lines that are fine.

PRECISION, AND WHAT IT COST TO GET THERE

The first run reported 87. Three things were wrong, each of which turned
correct code into a finding:

  * The nil test scanned a fixed window after each table entry and ran into the
    next one, so GetMerchantNumItems - which returns zero - was flagged because
    a later binding pushes nil. Bodies are read by brace matching now.
  * Nearly every binding pushes nil on an early-out (`if (!gh) return
    luaReturnNil(L)`), and a caller that got as far as calling it has already
    made that unreachable. Only the last run of pushes before the final return
    counts, which is the success block.
  * Position matters. GetTalentTabInfo answers nil for its icon and numbers for
    both point counts; flagging every name unpacked from it reported the two
    that were fine.

With all three: 2. Both false positives, and both checkable in a minute -
LOOT_BIND_CONFIRM is never fired, and LoadAddOn's reason sits inside
`if ( not loaded )`.

WHAT IT STILL CANNOT SEE

A guard in the caller rather than in the function doing the arithmetic.
GroupLootFrame_OnShow tests `name == nil` and returns, and the concatenation is
two functions away - correct, and invisible from here.

It found nothing new. The two bugs it was written for, both in the
dungeon-ready dialog, were already fixed by reading the file. It is here so
the next value that goes nil on a success path is caught before a player finds
it.
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
XML = ROOT / "Data/interface"
ADDONS = ROOT / "src/addons"

sys.path.insert(0, str(Path(__file__).resolve().parent))
from framexml_provides import globals_provided
from framexml_source import without_comments_or_strings, loaded_files

def _balanced(src, open_at):
    """The braced block starting at open_at, so a body ends where it ends.

    A fixed window instead of this is why the first run flagged
    GetMerchantNumItems, which returns zero: the window ran past the end of its
    entry and into a neighbour that pushes nil. A sweep that over-reports is
    one nobody reads, and this one is about a class subtle enough that noise
    buries it.
    """
    depth, i = 0, open_at
    while i < len(src):
        if src[i] == "{":
            depth += 1
        elif src[i] == "}":
            depth -= 1
            if depth == 0:
                return src[open_at:i + 1]
        i += 1
    return ""


def _nil_positions(body):
    """Which return positions are nil on the success path, 1-based.

    Position matters. GetTalentTabInfo answers nil for its icon and numbers for
    both point counts, so flagging every name unpacked from it reports the two
    that are fine and buries the one that would not be. Knowing the position
    turns "this binding can answer nil" into "this value is nil".
    """
    if "return " not in body:
        return set()
    head = body[:body.rfind("return ")]
    lines = [ln.strip() for ln in head.split("\n") if ln.strip()]
    block = []
    for ln in reversed(lines):
        if "lua_push" in ln or ln.startswith("//") or ln.startswith("}"):
            block.append(ln)
            continue
        break
    block.reverse()
    positions, index = set(), 0
    for ln in block:
        for call in re.finditer(r"lua_push(\w+)", ln):
            index += 1
            if call.group(1) == "nil":
                positions.add(index)
    return positions


def _answers_nil(body):
    """Whether the binding answers nil on its SUCCESS path.

    This is the whole difficulty. Nearly every binding pushes nil somewhere -
    `if (!gh) return luaReturnNil(L)` is the house style - and a caller that
    reached the binding at all has already made that path unreachable.
    Counting those flagged seven bindings in a row that were all fine.

    What matters is a nil among the values a *successful* call returns, which
    is what GetLFGProposal did: eleven values, six of them nil, and two of the
    six went straight into a concatenation and an addition.

    So: look only at the last run of pushes before the final return, which is
    the success block, and ignore every early-out above it.
    """
    tail = body[body.rfind("return "):] if "return " in body else ""
    if not tail:
        return False
    # Walk back from the final return over the contiguous push statements.
    head = body[:body.rfind("return ")]
    lines = [ln.strip() for ln in head.split("\n") if ln.strip()]
    block = []
    for ln in reversed(lines):
        if "lua_push" in ln or ln.startswith("//") or ln.startswith("}"):
            block.append(ln)
            continue
        break
    return any("lua_pushnil" in ln for ln in block)


# Bindings whose own body can answer nil.
pushes_nil = {}
for path in sorted(ADDONS.glob("*.cpp")):
    src = path.read_text(errors="ignore")
    # An inline lambda: its body is the block after the arrow.
    for m in re.finditer(r'\{(?:\.\w+\s*=\s*)?"([A-Za-z_]\w*)",\s*(?:\.\w+\s*=\s*)?\[\]\(lua_State\*\s*L?\s*\)\s*->\s*int\s*\{', src):
        body = _balanced(src, src.index("{", m.end() - 1))
        if _answers_nil(body):
            pushes_nil[m.group(1)] = _nil_positions(body)
    # A named implementation: find the function and read its own body.
    for m in re.finditer(r'\{(?:\.\w+\s*=\s*)?"([A-Za-z_]\w*)",\s*(?:\.\w+\s*=\s*)?&?\s*(lua_[A-Za-z0-9_]+)\s*\}', src):
        fn = re.search(r"int %s\s*\(lua_State\*[^)]*\)\s*\{" % re.escape(m.group(2)), src)
        if not fn:
            continue
        fnbody = _balanced(src, fn.end() - 1)
        if _answers_nil(fnbody):
            pushes_nil[m.group(1)] = _nil_positions(fnbody)

provided = globals_provided()

FUNC = re.compile(r"\bfunction\s+[\w.:]+\s*\((.*?)\n(?:end\b|\Z)", re.S)
CALL = re.compile(r"local\s+([\w\s,]+?)\s*=\s*([A-Z]\w*)\s*\(")

rows = []
for path in sorted(loaded_files(XML)):
    text = without_comments_or_strings(path.read_text(errors="ignore"))
    for fn in FUNC.finditer(text):
        body = fn.group(1)
        for call in CALL.finditer(body):
            names = [n.strip() for n in call.group(1).split(",") if n.strip()]
            source = call.group(2)
            if source not in provided or source not in pushes_nil:
                continue
            after = body[call.end():]
            for position, name in enumerate(names, start=1):
                if name == "_":
                    continue
                if position not in pushes_nil[source]:
                    continue
                esc = re.escape(name)
                # Guarded anywhere after the call? Generous on purpose.
                # FrameXML writes `if ( texture ) then`, so the name is not
                # adjacent to the keyword. Requiring adjacency reported every
                # guarded use in the mail frame as unguarded.
                # Every guard shape FrameXML uses, because missing one turns a
                # correct line into a finding. Three passes were needed to get
                # this list: `if ( x )` with the parens, `x == nil` as a
                # comparison rather than a truth test, and `x or default`.
                if re.search(rf"\b(?:if|elseif)\s*\(?\s*(?:not\s+)?{esc}\b", after) or \
                   re.search(rf"\b{esc}\s*\)?\s+(?:or|and)\b", after) or \
                   re.search(rf"\bnot\s+{esc}\b", after) or \
                   re.search(rf"\b{esc}\s*[=~]=\s*nil\b", after) or \
                   re.search(rf"\bnil\s*[=~]=\s*{esc}\b", after):
                    continue
                hit = re.search(rf"(\.\.\s*{esc}\b|\b{esc}\s*\.\.)", after) or \
                      re.search(rf"\b{esc}\s*[-+*/]\s*\w|\w\s*[-+*/]\s*{esc}\b", after)
                if hit:
                    line = after[max(0, hit.start() - 40):hit.end() + 40].strip().split("\n")[0]
                    rows.append((source, name, path.name, line))

# Only files this client actually draws. The full list is dominated by the
# arena and battlefield panels, which no element hands over.
from framexml_reachable_globals import LIVE  # noqa: E402
rows = [r for r in rows if r[2] in LIVE]

print(f"{len(pushes_nil)} bindings can answer nil\n")
print(f"{len(rows)} unguarded use(s) of one in concatenation or arithmetic:\n")
seen = set()
for source, name, where, line in sorted(rows):
    key = (source, name, where)
    if key in seen:
        continue
    seen.add(key)
    print(f"  {source}  ->  {name}   [{where}]")
    print(f"      {line[:96]}")

sys.exit(0)
