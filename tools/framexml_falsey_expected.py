#!/usr/bin/env python3
"""Bindings that answer "" or 0 where the interface tests for nothing.

    tools/framexml_falsey_expected.py

WHY THIS FINDS WHAT THE OTHER SWEEPS CANNOT

framexml_nil_use_check asks about names that do not exist, used where nil
raises. This asks the opposite: a name that *does* exist, answering a value
that is not nil, in a place where nil was the answer meaning "none". In Lua
only nil and false are false - an empty string is true and so is zero - so the
branch meant for "there is nothing here" never runs, and the one meant for
"here it is" runs with nothing in its hands.

Two on 2026-08-06, both showing a panel that should not have been there:

  * GetAbandonQuestItems answered "". Both callers do `if ( items )` to choose
    between ABANDON_QUEST and ABANDON_QUEST_WITH_ITEMS, so abandoning any quest
    offered "you will lose:" with nothing after the colon.
  * GetGuildBankTabCost answered 0. The panel does `if ( not tabCost )` to
    decide the guild has bought all six tabs, so a guild that owned them all
    still saw the buy screen - priced at nothing, over a button that sends.

WHAT IT COMPARES

Names the interface tests directly - `if ( X() )`, `if ( not X() )`, `and X()`,
`or X()` - against C bindings whose body pushes a constant "" or 0 and never
pushes nil at all. Both halves have to hold: a binding that can answer nil is
already able to say "none", and a name that is only ever assigned and read
later is not evidence of anything on its own.

WHAT IT CANNOT SEE

A test further from its assignment than ASSIGNED_WINDOW lines, or one whose
variable is passed somewhere before being tested. The window is short because a
long one starts matching a name reused further down the file.

Nor a binding that answers a *computed* zero. Only a literal push is matched,
because a computed zero is usually a real count.
"""
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from framexml_source import loaded_files, without_comments  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
INTERFACE = ROOT / "Data/interface"
ADDONS = ROOT / "src/addons"

#: Answering a falsey-looking constant is right for these, checked once.
#:
#: GetLootMethod's "freeforall" is a real method rather than a "none"; the two
#: statistics ones answer the dash WoW itself shows. They are listed rather
#: than filtered by shape because each is a judgement about that binding.
EXPECTED = {
    "GetStatistic", "GetComparisonStatistic",
    # The test this one fails gates the *button*, not a popup:
    # questlogframe.lua enables Abandon while GetAbandonQuestName() is true, and
    # the name is only set once SetAbandonQuest has been called - which happens
    # when the button is clicked. Answering nil before that would disable the
    # button for good and there would be no way to reach the call that fills it.
    "GetAbandonQuestName",
    # Two call sites and only one of them tests: petstable.lua:101 puts the
    # result straight into format(PET_DIET_TEMPLATE, BuildListString(...)), and
    # string.format raises on the nil BuildListString hands back for a nil. The
    # empty string says "not known" at the guarded site and keeps the stable
    # window up at the unguarded one.
    "GetStablePetFoodTypes",
}


#: How far past `local v = X()` to look for `if ( v )`. Short on purpose: the
#: assignment and the test it decides are next to each other in every case that
#: matters, and a long window starts matching a variable reused further down.
ASSIGNED_WINDOW = 12


def tested_directly():
    """Names the interface puts into a truth test, written either way.

    The direct form - `if ( X() )` - and the assigned one, `local v = X()`
    followed by `if ( v )` a few lines later. The assigned form was left out at
    first and that made the sweep useless for the two faults it was written
    for: both write `local items = GetAbandonQuestItems()` and test `items` on
    the next line. Putting the fault back and watching the report stay empty is
    what caught it.
    """
    names = set()
    direct = [
        re.compile(r"if\s*\(\s*(?:not\s+)?([A-Z]\w+)\s*\([^()]*\)\s*\)"),
        re.compile(r"(?:and|or)\s+([A-Z]\w+)\s*\([^()]*\)\s*(?:\)|then)"),
    ]
    assign = re.compile(r"local\s+([a-z]\w*)\s*=\s*([A-Z]\w+)\s*\(")
    for path in loaded_files(INTERFACE):
        text = without_comments(path.read_text(errors="ignore"))
        for p in direct:
            names.update(p.findall(text))
        lines = text.splitlines()
        for i, line in enumerate(lines):
            m = assign.search(line)
            if not m:
                continue
            var, fn = m.group(1), m.group(2)
            tail = "\n".join(lines[i + 1:i + 1 + ASSIGNED_WINDOW])
            if re.search(r"if\s*\(\s*(?:not\s+)?" + re.escape(var) + r"\s*\)", tail):
                names.add(fn)
    return names


def constant_falsey_bindings():
    """name -> what it pushes, for bindings that never push nil."""
    out = {}
    src = "".join(p.read_text(errors="ignore") for p in sorted(ADDONS.glob("*.cpp")))

    def body_at(start):
        depth, i = 1, start
        while i < len(src) and depth:
            if src[i] == "{":
                depth += 1
            elif src[i] == "}":
                depth -= 1
            i += 1
        return src[start:i - 1]

    bodies = {}
    for m in re.finditer(r"static int (lua_\w+)\(lua_State\* L\)\s*\{(?:\.\w+\s*=\s*)?", src):
        bodies[m.group(1)] = body_at(m.end())
    named = dict(re.findall(r'\{(?:\.\w+\s*=\s*)?"([A-Za-z_]\w*)",\s*(?:\.\w+\s*=\s*)?(?:&)?\s*(lua_\w+)\}', src))
    for name, impl in named.items():
        if impl in bodies:
            out.setdefault(name, bodies[impl])
    for m in re.finditer(r'\{(?:\.\w+\s*=\s*)?"([A-Za-z_]\w*)",\s*(?:\.\w+\s*=\s*)?\[\]\(lua_State\* L\) -> int \{', src):
        out.setdefault(m.group(1), body_at(m.end()))

    hits = {}
    for name, body in out.items():
        if "lua_pushnil" in body or "luaReturnNil" in body:
            continue
        if re.search(r'lua_pushstring\(\s*L\s*,\s*""\s*\)', body):
            hits[name] = 'pushes ""'
        elif re.search(r"lua_pushnumber\(\s*L\s*,\s*0\s*\)", body):
            hits[name] = "pushes 0"
    return hits


def main():
    tested = tested_directly()
    falsey = constant_falsey_bindings()
    rows = sorted((n, w) for n, w in falsey.items()
                  if n in tested and n not in EXPECTED)

    print(f"{len(tested)} names tested for truth by the interface, "
          f"{len(falsey)} bindings answer a constant \"\" or 0 and never nil")
    if "GetAbandonQuestName" not in falsey and "UnitName" not in tested:
        print("  CANARY: neither half parsed - the report below means nothing.")
    print()
    print(f"{len(rows)} answer something true where nothing was meant:\n")
    for name, what in rows:
        print(f"  {name:34} {what}")
    if not rows:
        print("  (none)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
