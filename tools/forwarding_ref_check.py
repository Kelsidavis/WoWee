#!/usr/bin/env python3
"""Writable accessors that reach round the forwarding their readers go through.

    tools/forwarding_ref_check.py

WHY

GameHandler was split into per-subject handlers, and what is left of it forwards:
every getter asks the sub-handler when there is one and only falls back to its
own member when there is not. In practice the sub-handler is always there, so
the member behind the fallback is never written and never read.

Next to those getters sit accessors that hand out a writable reference, for the
few places that edit a list in place rather than replacing it. Those were left
returning the member directly. A caller then writes to a list that nothing
displays, and the write is not lost in a way anybody can see: no error, no log
line, the panel simply never changes.

Two were live when this was written. Sorting an auction house column reordered
GameHandler's empty copy while the rows on screen came from the inventory
handler's, so clicking a column header did nothing. The backfill that fills in
a mail's sender once the name packet arrives wrote to the same empty copy, so
mail from a player whose name had not yet resolved kept a blank sender.

WHAT IT DOES

Pairs the members handed out by a writable Ref() accessor against the members
that appear only as the fallback branch of a forwarding accessor. A member in
both sets has a reader and a writer looking at different objects.

WHAT IT CANNOT SEE

Forwarding written in some other shape than the two below, and the same split
in a class other than GameHandler. It also cannot tell a deliberate local
cache from a mistake, which is why it reports rather than judges.
"""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
HEADER = ROOT / "include/game/game_handler.hpp"
SOURCES = sorted((ROOT / "src/game").glob("game_handler*.cpp"))

# `auto& xRef() { return x_; }` - a writable reference to this class's member.
WRITABLE_REF = re.compile(r"(\w+Ref)\(\)\s*(?:const\s*)?\{\s*return (\w+_?);")

# The two shapes the forwarding is written in.
FORWARD_TERNARY = re.compile(r"(\w+Handler_) \? \1->\w+\([^\)]*\) : (\w+_?)\b")
FORWARD_EARLY_RETURN = re.compile(
    r"if \((\w+Handler_)\) return \1->\w+\([^\)]*\);\s*\n\s*return (\w+_?);")


def main():
    if not HEADER.exists() or not SOURCES:
        print("Found no game handler. The zero below means the scan broke.")
        return 1

    header = HEADER.read_text(errors="ignore")
    refs = {}
    for match in WRITABLE_REF.finditer(header):
        refs.setdefault(match.group(2), []).append(match.group(1))

    source = "".join(p.read_text(errors="ignore") for p in SOURCES)
    fallback = set()
    for pattern in (FORWARD_TERNARY, FORWARD_EARLY_RETURN):
        for match in pattern.finditer(source):
            fallback.add(match.group(2))

    if not refs or not fallback:
        print("Recognised no accessors at all. The zero below means the scan broke.")
        return 1

    # A second shape, and the one that got past the first. The pattern above
    # needs the forwarding getter to *fall back to the same member*, which is
    # the tidy case. getGossipPois forwarded to QuestHandler and fell back to a
    # `static const ... empty` instead - so gossipPois_ never appeared in the
    # fallback at all, and nothing here associated the two. That is worse
    # rather than better: the member is not merely a stale fallback, it is
    # never returned by anything.
    #
    # So: a member handed out by a writable Ref() whose getter forwards, and
    # which GameHandler's own sources never read. Something edits it through
    # the reference and nothing can ever see the edit. movement_handler cleared
    # the gossip points of interest that way, and the markers stayed on the map.
    forwarding = set(re.findall(r"if \(\w+Handler_\) return \w+Handler_->(\w+)\(", source))
    forwarding |= set(re.findall(r"return \w+Handler_ \? \w+Handler_->(\w+)\(", source))
    unread = []
    for member in sorted(refs):
        stem = member.rstrip("_")
        getter = "get" + stem[:1].upper() + stem[1:]
        if getter not in forwarding:
            continue
        if re.search(r"[^\w.>]" + re.escape(member) + r"\b(?!\s*=[^=])", source):
            continue
        unread.append((member, getter))

    diverging = sorted(set(refs) & fallback)
    print(f"{len(refs)} member(s) handed out by a writable Ref()")
    print(f"{len(fallback)} member(s) reached only as a forwarding fallback\n")
    print(f"{len(diverging)} member(s) written locally and read through a sub-handler:")
    for name in diverging:
        print(f"  {name}  via {', '.join(refs[name])}")
    if not diverging:
        print("  (none)")

    # A third shape: a *writer* that touches a member whose getter forwards,
    # without telling the sub-handler. The first two arms look at the member;
    # this one looks at the method, so it catches a writer whose member is read
    # elsewhere in GameHandler and therefore never appears as dead.
    #
    # resetDbcCaches was this. It is called when the active expansion changes
    # and cleared local copies of the talent and taxi caches, while the getters
    # beside them forward to SpellHandler and MovementHandler - so a switch left
    # the previous expansion's talents and flight points live, and the comment
    # above the call said the opposite.
    forwarded_getters = set(re.findall(
        r"\bGameHandler::(\w+)\([^)]*\)[^{]*\{\s*(?:if \()?\w+Handler_\s*\)?\s*(?:\?|return)\s*\w+Handler_->", source))
    stale_writers = []
    for m in re.finditer(r"\bGameHandler::(\w+)\([^)]*\)[^{;]*\{(?:\.\w+\s*=\s*)?", source):
        depth, j, started = 0, m.end() - 1, False
        while j < len(source):
            if source[j] == "{":
                depth += 1
                started = True
            elif source[j] == "}":
                depth -= 1
                if started and depth == 0:
                    j += 1
                    break
            j += 1
        name, body = m.group(1), source[m.end():j]
        if name in forwarded_getters or "Handler_->" in body:
            continue
        for w in re.finditer(r"[^\w.>](\w+_)\s*(?:=[^=]|\.clear\(|\.push_back|\.insert)", body):
            member = w.group(1)
            if member.endswith("Handler_"):
                continue
            stem = member.rstrip("_")
            getter = "get" + stem[:1].upper() + stem[1:]
            if getter in forwarded_getters:
                stale_writers.append((name, member, getter))
                break

    print(f"\n{len(stale_writers)} writer(s) that change a member whose reader forwards:")
    if not stale_writers:
        print("  (none)")
    for name, member, getter in stale_writers:
        print(f"  {name}()")
        print(f"      writes {member}, but {getter}() forwards to a sub-handler "
              f"- the change is invisible to every reader")

    print(f"\n{len(unread)} member(s) edited through a reference nothing reads:")
    if not unread:
        print("  (none)")
    for member, getter in unread:
        print(f"  {member}")
        print(f"      handed out writable, while {getter}() forwards to a "
              f"sub-handler - the edit goes nowhere anything can see")
    return 0


if __name__ == "__main__":
    sys.exit(main())
