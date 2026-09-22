#!/usr/bin/env python3
"""Packet handlers left behind in the class they were moved out of.

    tools/handler_twin_check.py

WHY

GameHandler was split into ChatHandler, SocialHandler, InventoryHandler and the
rest, and each move left the original method where it was. Most became one-line
forwarders and are still registered, which is fine. Two were not:

    GameHandler::handleQueryTimeResponse   a whole second implementation
    GameHandler::handleItemQueryResponse   a forwarder nothing dispatches

Neither is reachable. Both read as live code to anyone grepping the name,
because the twin that replaced them answers to it.

The dead-symbol sweep cannot see these. It matches on the name, and the name
has a caller - just not this one's. That is the whole shape: a duplicate is
invisible to a check that does not know which class it is looking at.

WHAT IT DOES

Finds every `Class::handleX` defined in src/, and asks whether anything reaches
that class's copy: a call inside another method of the same class (the dispatch
lambdas in registerOpcodes are written that way), a `&Class::handleX` member
pointer, a call through a member of that type, or a call through the free
accessor that hands out the one instance - `touchControls().handleEvent(e)`,
which is how the two classes with no owner but the application are reached.

WHAT IT CANNOT SEE

A handler reached only through a base-class virtual, and one whose name is
built at runtime. Neither exists here today; both would read as dead.
"""
import collections
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
# A definition starts at column 0 here. Allowing leading whitespace made a
# wrapped argument list - "std::to_string(maxV), ..." - read as the start of
# a method on a class called std, and every handler defined after it in the
# file was attributed to that.
DEF = re.compile(r"^[\w:<>,&\*][\w:<>,&\* ]*?\b(\w+)::(\w+)\(")
CALL = re.compile(r"(?<![\w:.>])(handle\w+)\s*\(")
# `spellHandler_->handleX(` and `owner_.handleX(` - how one handler reaches
# another. Without this every sub-handler reads as unreachable, because the
# only thing that calls it is the class that owns it.
THROUGH = re.compile(r"(\w+)\s*(?:->|\.)\s*(handle\w+)\s*\(")
# `std::unique_ptr<SpellHandler> spellHandler_;` and plain members, so a call
# through a member can be attributed to the class it points at. Any class held
# by a smart pointer, namespace and all - `std::unique_ptr<ui::MapWindow>
# mapWindow_` owns the only MapWindow, and matching Handler names alone read
# the call through it as nothing, and the handler it reaches as dead.
MEMBER = re.compile(r"\b(?:std::(?:unique_ptr|shared_ptr)<\s*(?:\w+::)*(\w+)\s*>|(\w+Handler))"
                    r"\s*[\*&]?\s*(\w+_)\s*[;=]")
# `TouchControls& touchControls();` - a free function handing out the single
# instance of a class that has no owner to be a member of. Without this the
# only thing calling that class's handler is a line the scan cannot attribute
# to anything, and every one of its handlers reads as unreachable.
ACCESSOR = re.compile(r"^\s*(\w+)\s*&\s*(\w+)\s*\(\s*\)\s*;", re.M)
# `touchControls().handleEvent(` - the call that accessor makes possible.
VIA_ACCESSOR = re.compile(r"(\w+)\s*\(\s*\)\s*\.\s*(handle\w+)\s*\(")


def main() -> int:
    src = ROOT / "src"
    if not src.is_dir():
        print("src is not here; the zero below would mean the scan broke.")
        return 1

    defined = collections.defaultdict(set)   # handler name -> {class}
    called_in = collections.defaultdict(set)  # handler name -> {calling class}
    text_all = []
    for path in sorted(src.rglob("*.cpp")):
        text = path.read_text(errors="ignore")
        text_all.append(text)
        owner = None
        for line in text.split("\n"):
            m = DEF.match(line)
            if m:
                owner = m.group(1)
                if m.group(2).startswith("handle"):
                    defined[m.group(2)].add(owner)
                continue
            for c in CALL.finditer(line):
                if owner:
                    called_in[c.group(1)].add(owner)
    joined = "\n".join(text_all)

    # A member's declared type, so `spellHandler_->handleX` counts as a call on
    # SpellHandler rather than on nothing.
    member_class = {}
    for path in sorted((ROOT / "include").rglob("*.hpp")):
        for m in MEMBER.finditer(path.read_text(errors="ignore")):
            member_class[m.group(3)] = m.group(1) or m.group(2)
    for m in THROUGH.finditer(joined):
        cls = member_class.get(m.group(1))
        if cls:
            called_in[m.group(2)].add(cls)

    # And the class a free accessor hands out, so a call through it is a call
    # on that class.
    accessor_class = {}
    for path in sorted((ROOT / "include").rglob("*.hpp")):
        for m in ACCESSOR.finditer(path.read_text(errors="ignore")):
            accessor_class[m.group(2)] = m.group(1)
    for m in VIA_ACCESSOR.finditer(joined):
        cls = accessor_class.get(m.group(1))
        if cls:
            called_in[m.group(2)].add(cls)

    twins = {n: c for n, c in defined.items() if len(c) > 1}
    if not twins:
        print("No handler name is defined in two classes, which cannot be "
              "right - the scan broke rather than the split being undone.")
        return 1

    orphans = []
    for name, classes in sorted(twins.items()):
        for cls in sorted(classes):
            if cls in called_in.get(name, set()):
                continue                       # a sibling method calls it
            if re.search(r"&%s::%s\b" % (cls, name), joined):
                continue                       # registered as a member pointer
            orphans.append((cls, name))

    print(f"{len(twins)} handler name(s) defined in more than one class")
    print(f"\n{len(orphans)} copy that nothing reaches:")
    if not orphans:
        print("  (none)")
    for cls, name in orphans:
        print(f"  {cls}::{name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
