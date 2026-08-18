#!/usr/bin/env python3
"""What the client answers for - the one place that decides.

Seven sweeps each worked this out for themselves, and they disagreed. Most read
only the C++ binding tables, which is between a third and a half of the answer:
138 names are provided from bootstrap Lua living inside C++ string literals, and
a tool that cannot see those reports them as gaps.

That is not hypothetical. `framexml_reachable_globals` reported
GetNumStationeries as unbound and reachable from the mail frame. It has been
answered by the counting table since long before, and acting on that report made
things worse - binding it explicitly removed it from the missing-API report,
which is the one thing that report exists to preserve. Meanwhile
`framexml_element_readiness` had it right the whole time, because its
`registered()` did read the bootstrap.

So: one implementation, imported. A sweep that wants to know whether a name is
answered asks here.

    from framexml_provides import globals_provided, widget_methods_provided
"""
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
ADDONS = ROOT / "src/addons"
ENGINE = ADDONS / "lua_engine.cpp"


def _sources():
    return [p.read_text(encoding="utf-8", errors="ignore")
            for p in sorted(ADDONS.glob("*.cpp"))]


def globals_provided():
    """Every global name the client answers, however it answers it.

    Four routes, and missing any one of them produces false gaps:

      * a C++ table entry, `{"Name", lua_Name}`
      * an explicit `lua_setglobal(L, "Name")`
      * a function defined in bootstrap Lua, which lives in C++ string
        literals - `function Name(` and `function Name:`
      * a name in a quoted list inside the bootstrap, which is how the counting
        table provides a zero for thirty-five names so that a nil never reaches
        a `for` limit
    """
    names = set()
    for src in _sources():
        names |= set(re.findall(r'\{\s*(?:\.\w+\s*=\s*)?"([A-Za-z_]\w*)"\s*,', src))
        names |= set(re.findall(r'lua_setglobal\(\s*\w+\s*,\s*(?:\.\w+\s*=\s*)?"([A-Za-z_]\w*)"', src))
        names |= set(re.findall(r"function\s+([A-Za-z_]\w*)\s*[:(]", src))
        names |= set(re.findall(r"'([A-Za-z_]\w*)'", src))
    return names


def widget_methods_provided():
    """Method names a frame answers: the method table, Lua shims, the allowlist.

    Separate from globals because they resolve through the widget metatable
    rather than _G, and a name can be one without being the other.
    """
    src = ENGINE.read_text(encoding="utf-8", errors="ignore")
    table = set(re.findall(r'\{(?:\.\w+\s*=\s*)?"([A-Za-z_]\w*)",\s*(?:\.\w+\s*=\s*)?lua_', src))
    # Every method the bootstrap defines on anything, not only on the frame
    # metatable. Matching `mt:` alone missed animMeta and groupMeta, so
    # IsPlaying, SetDuration and SetOffset were reported as answering nil when
    # the animation system has provided all three since it was written.
    shims = set(re.findall(r'function\s+\w+\s*:\s*(\w+)\s*\(', src))
    # The region method table is built with a `set("Name", fn)` lambda rather
    # than a braced table, and fifty-seven names go on that way. Five of them
    # were answered nowhere else and so read as unprovided - GetTextColor and
    # SetTextHeight among them, both of which had just been implemented. A
    # source of truth that cannot see one of the registration forms sends every
    # sweep that trusts it looking for gaps that are not there.
    region = set(re.findall(r'\bset\("([A-Za-z_]\w*)"', src))
    # Several names per string literal - "SetMovable=1,SetNormalTexture=1,\n" -
    # so anchoring on the opening quote finds only the first of each and
    # under-counts the allowlist by four to one.
    allowlist = set(re.findall(r"\b([A-Za-z]\w*)=1", src))
    return table | shims | region | allowlist


def counting_table():
    """Names given a zero so a nil never reaches a numeric `for` limit.

    Worth having apart: these are deliberately *not* real implementations, and
    they stay in the missing-API report under a "count:" prefix. Binding one
    for real takes it out of that report, which is a loss rather than progress.
    """
    src = ENGINE.read_text(encoding="utf-8", errors="ignore")
    m = re.search(r"local counting = \{(.*?)\}", src, re.S)
    return set(re.findall(r"'(\w+)'", m.group(1))) if m else set()


def noop_widget_methods():
    """Method names answered only by the no-op - which returns nothing.

    The distinction the other sweeps do not draw. `widget_methods_provided`
    folds the allowlist in with the real ones, because for "does this raise
    outright" they are the same. They are not the same for a caller that reads
    the return: a no-op answers nil, and nil in a comparison raises just as an
    absent method does. GetFieldSize sat in the allowlist and so counted as
    answered, while its one caller compared a byte count against it and the
    guild event log came out blank.
    """
    src = ENGINE.read_text(encoding="utf-8", errors="ignore")
    real = set(re.findall(r'\{(?:\.\w+\s*=\s*)?"([A-Za-z_]\w*)",\s*(?:\.\w+\s*=\s*)?lua_', src))
    real |= set(re.findall(r'set\("([A-Za-z_]\w*)"', src))
    real |= set(re.findall(r'function\s+\w+\s*:\s*(\w+)\s*\(', src))
    real |= _loop_built(src)
    allowlist = set(re.findall(r"\b([A-Za-z]\w*)=1", src))
    return allowlist - real


def _loop_built(src):
    """Method names the bootstrap assembles rather than writes out.

    The button art block is the reason: it loops a list of slot names and
    assigns `mt['Set' .. slot]` and `mt['Get' .. slot]`, so twelve real methods
    appear nowhere as a literal. Reading only literals put GetNormalTexture,
    GetCheckedTexture and their neighbours in the no-op set, which is the
    opposite of true - they are singled out for a real implementation *because*
    the no-op was wrong for them, and the comment above the loop says so.

    Cross-product rather than exact: every affix used with a loop variable
    against every name in every list. Wider than the truth, and wide is the
    safe direction here - a name wrongly called provided drops a row from a
    report, a name wrongly called missing sends someone to implement what
    already works.
    """
    items = set()
    for lst in re.findall(r"ipairs\(\{(.*?)\}\)", src, re.S):
        items |= set(re.findall(r"'([A-Za-z_]\w*)'", lst))
    affixes = set(re.findall(r"\w+\[\s*'([A-Za-z_]\w*)'\s*\.\.", src))
    affixes |= set(re.findall(r"\.\.\s*'([A-Za-z_]\w*)'\s*\]", src))
    return {a + i for a in affixes for i in items} | \
           {i + a for a in affixes for i in items}


if __name__ == "__main__":
    g, w, c = globals_provided(), widget_methods_provided(), counting_table()
    print(f"{len(g)} globals provided")
    print(f"{len(w)} widget methods provided")
    print(f"{len(c)} of the globals are counting-table zeros, which belong in "
          f"the missing-API report and should not be bound away")
