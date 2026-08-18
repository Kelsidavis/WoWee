#!/usr/bin/env python3
"""Frame methods that write a Lua field where the widget holds the truth.

    tools/widget_field_check.py

A frame is two things at once: a Lua table the interface holds, and a widget in
the C++ tree that is laid out and drawn. Some state belongs to the table -
scripts, registered events - and some belongs to the widget - parent, size,
visibility, anchors. A method that writes the table where the widget is what
gets read looks like it worked and changes nothing on screen.

That is not a hypothetical shape. SetParent wrote `__parent` and nothing else,
so GetParent answered the new parent while layout went on placing, clipping and
hiding the frame under the old one - with QuestInfo reparenting every element of
a quest into whichever window is showing it on every display. GetCenter read
`__xOfs`, written only by a SetPoint registered nowhere, so it answered zero for
every frame in the interface. Both read as ordinary code.

WHAT IT LOOKS FOR

Every method in the frame metatable whose body touches a `__field` on the frame
table and never reaches the widget tree - no widgetOf, no widgetIdOf, no
getWidgetTree.

WHAT IT CANNOT SEE

Which side owns a given fact. Scripts and events genuinely live on the table and
always will, so the honest number here is not zero. Read each before acting: the
question is whether the widget also holds that state, not whether a field is
used at all.

Nor a method that reaches the tree *and* keeps a stale field beside it. Those
drift more slowly and show up as two answers to one question.
"""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
ENGINE = ROOT / "src/addons/lua_engine.cpp"

TREE_CALLS = ("widgetOf", "widgetIdOf", "getWidgetTree")


def bodies(src):
    """Function name -> its body, matched by counting braces.

    A regex to the first `}` stops inside the first `if` in most of these.
    """
    out = {}
    for m in re.finditer(r"^(?:static\s+)?int\s+(lua_\w+)\(lua_State\*\s*L\)\s*\{(?:\.\w+\s*=\s*)?",
                         src, re.M):
        depth, i = 1, m.end()
        while i < len(src) and depth:
            if src[i] == "{":
                depth += 1
            elif src[i] == "}":
                depth -= 1
            i += 1
        out[m.group(1)] = src[m.end():i]
    return out


#: Methods whose whole subject is the Lua object rather than the widget behind
#: it, checked one at a time. A set rather than a count: a ceiling says only how
#: many there are, so fixing one and introducing another leaves the number
#: unmoved.
#:
#: What these six have in common is that the widget has no opinion to diverge
#: from. A frame's name, its parent, its scripts and the events it listens to
#: are all Lua-side truth - nothing renames a widget behind __name, and
#: SetParent writes the widget as well as the field precisely because the
#: widget *does* have an opinion about parenthood.
#:
#: GetParent is the one to be careful with. It has to answer nil for a frame
#: created with an explicit nil parent, because that is how FrameXML tells a
#: top-level frame from a nested one - a template's parent= is applied only to
#: a frame that arrived without one. Reading the widget instead would answer
#: the screen, which is not nil.
EXPECTED_FIELD_ONLY = {
    "GetName": "__name, and nothing renames a widget",
    "GetParent": "__parent, and nil is load bearing",
    "GetScript": "__scripts",
    "SetScript": "__scripts",
    "RegisterEvent": "__events",
    "UnregisterEvent": "__events",
    # The reader for the pair above. Events are one of the few things that
    # genuinely live in Lua rather than on the widget - the dispatch reads the
    # same table - so answering from it is what keeps the answer and the
    # behaviour from drifting apart.
    "IsEventRegistered": "__events",
    "UnregisterAllEvents": "__events",
}


def main():
    src = ENGINE.read_text(errors="ignore")
    registered = dict(re.findall(r'\{(?:\.\w+\s*=\s*)?"(\w+)",\s*(?:\.\w+\s*=\s*)?(lua_\w+)\}', src))
    fns = bodies(src)

    rows = []
    for api, fn in sorted(registered.items()):
        body = fns.get(fn)
        if body is None:
            continue
        if any(call in body for call in TREE_CALLS):
            continue
        writes = 'lua_setfield(L, 1, "__' in body or \
                 'lua_setfield(L, -2, "__' in body
        reads = 'lua_getfield(L, 1, "__' in body
        if (writes or reads) and api not in EXPECTED_FIELD_ONLY:
            rows.append((api, fn, "writes" if writes else "reads"))

    print(f"{len(registered)} registered methods, {len(fns)} bodies read\n")
    print(f"{len(rows)} method(s) touch only a Lua field, never the widget:\n")
    for api, fn, kind in rows:
        print(f"  {api:26} {fn:34} {kind} a __field")
    if not rows:
        print("  (none)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
