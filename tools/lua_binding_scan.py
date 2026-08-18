"""The C++ bodies behind the Lua names the interface calls.

Shared by the checks that need to read what a binding actually does -
framexml_lua_override_check and framexml_vararg_spread - because both had
their own copy of this, character for character, and a parser that stops
recognising a registration shape does not fail. It reports fewer bindings, and
a sweep that quietly sees less of its subject is worse than no sweep: it goes
green for a reason nobody looks at.

TWO REGISTRATION SPELLINGS, AND BOTH ARE IN USE

    {"UnitName", lua_UnitName}                     a named function
    {"UnitName", [](lua_State* L) -> int { ... }}   a lambda in the table

A parser that knows only the first misses several hundred bindings, and the
ones it misses are not a random sample: the lambda form is what the newer
bindings use, so the sweep would be blindest to the most recently written code.

WHAT IT CANNOT SEE

A binding registered through a macro, or one whose name is computed. Neither
exists here; if one appears, every caller of this goes blind at once, which is
the trade for having a single parser rather than several.
"""
import pathlib
import re

ROOT = pathlib.Path(__file__).resolve().parent.parent
ADDONS = ROOT / "src" / "addons"


def binding_bodies():
    """Bound Lua name -> the C++ body behind it, for both spellings."""
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

    out = {}
    for name, impl in re.findall(r'\{(?:\.\w+\s*=\s*)?"([A-Za-z_]\w*)",\s*(?:\.\w+\s*=\s*)?(?:&)?\s*(lua_\w+)\}', src):
        if impl in bodies:
            out.setdefault(name, bodies[impl])
    for m in re.finditer(r'\{(?:\.\w+\s*=\s*)?"([A-Za-z_]\w*)",\s*(?:\.\w+\s*=\s*)?\[\]\(lua_State\* L\) -> int \{', src):
        out.setdefault(m.group(1), body_at(m.end()))
    return out


def resolve_body(name, src, bound=None, inline_bodies=None):
    """The C++ body behind a Lua name, trying harder than binding_bodies does.

    Two checks need this rather than the registration table alone, and they
    need it for a reason worth stating: the table is read with the `lua_`
    prefix optional, so a name that never appears in one can still be answered
    by a function named after it. That fallback is what finds the widget
    methods - every method framexml_bool_vs_number's second arm is about - and
    a version without it resolved 1612 names where this resolves 1863.

    `bound` and `inline_bodies` stay with the caller: each check parses the
    registration tables slightly differently on purpose, and this is only the
    brace-walk that both of them had written out.
    """
    inline_bodies = inline_bodies or {}
    bound = bound or {}
    if name in inline_bodies:
        return inline_bodies[name]
    impl = bound.get(name, name)
    for cand in (impl, "lua_" + impl):
        m = re.search(rf"\bint\s+{re.escape(cand)}\s*\(lua_State\s*\*\s*\w*\s*\)\s*\{{",
                      src)
        if not m:
            continue
        depth, i = 1, m.end()
        while i < len(src) and depth:
            if src[i] == "{":
                depth += 1
            elif src[i] == "}":
                depth -= 1
            i += 1
        return src[m.end():i - 1]
    return ""
