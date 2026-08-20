#!/usr/bin/env python3
"""Every handed-over element needs both halves, and neither half is loud.

Handing a panel to FrameXML is two edits in two places:

  * a **suppression entry**, so FrameXML's frame is hidden while this client
    still draws its own - without it both are on screen the moment the
    interface loads, before anyone has chosen anything;
  * a **frameXmlOwns gate** around this client's own drawing, so it stops when
    the element is handed over - without it both are on screen again, in the
    other direction.

Neither omission raises, logs, or fails a test. Both look exactly like a panel
that works, drawn twice. Twenty-four elements were being drawn twice at one
point, each for one of these two reasons.

    tools/handover_halves_check.py

WHAT IT CHECKS

Both halves, for every element named in the defaults or the candidates, and
both of them against `clientDraws` in the element table - which is the one
place that says whether this client still has a version of its own.

  * a gate, unless this client draws nothing, in which case there is nothing
    to gate and the exemption is that flag rather than a list here;
  * a suppression entry, if this client draws its own;
  * **no** suppression entry, if it does not. That half is the late one. A row
    here outlives its element: once the drawing behind it is deleted, hiding
    FrameXML's frame leaves a blank where a working window would have been.
    Seventeen rows were in that state, silently, because suppression is
    skipped for an owned element and all seventeen are owned by default - so
    the fault could only appear in a run naming a subset, which is the only
    thing the table is for. Found by asking why the exemption list here named
    twelve elements the table still claimed this client drew.

WHAT IT CANNOT SEE

Whether the gate is in the right place. The world map had both halves and was
still broken, because the gate wrapped the only function that *fed* the map as
well as the only one that drew it - so handing it over produced no map rather
than two. A gate around a function that does more than draw is a different
fault from a missing gate, and only reading the function finds it.

Nor whether an element needs a counterpart at all: a few are handed over by
other means and are listed here as exceptions rather than reported.
"""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
TAKEOVER = ROOT / "src/ui/framexml_takeover.cpp"

# The exemption from needing a gate used to be a list here, one entry per
# element with a note on what was deleted. It is read from `clientDraws` in the
# element table now. The two said the same thing and had drifted: this list had
# twelve elements the table still recorded as drawn by this client, which is
# how the stale suppression rows stayed invisible. What each handover deleted
# is in the commit that deleted it.

def main():
    if not TAKEOVER.exists():
        print(f"no {TAKEOVER}")
        return 1
    cpp = TAKEOVER.read_text(errors="ignore")

    table = re.search(r"\{UiElement::PlayerFrame.*?\n\};", cpp, re.S)
    # The third field says whether this client still draws its own version.
    # Absent means it does; an explicit false means the drawing was deleted.
    rows = re.findall(r'\{UiElement::(\w+),\s*"([a-z]\w*)"(,\s*\w+)?\}',
                      table.group(0))
    names = {e: n for e, n, _ in rows}
    draws = {e: "false" not in (flag or "") for e, _, flag in rows}
    # Only rows inside kSuppress count.
    #
    # This used to match any row in the file whose first string was
    # UpperCamelCase, on the reasoning that a suppression row names frames and
    # the element table names lowercase elements. True, and useless: a *check*
    # row names frames too, and every handed-over element has one. So the half
    # of this sweep that looks for a missing suppression entry answered zero
    # for every element that had a check row - which is all of them. Caught by
    # deleting a real suppression row and watching the count stay at zero.
    body = re.search(r"const Suppress kSuppress\[\] = \{(.*?)\n *\};", cpp, re.S)
    if not body:
        print("could not find kSuppress - this reads it")
        return 1
    suppress = set(re.findall(r'\{UiElement::(\w+),', body.group(1)))

    gates = set()
    for path in (ROOT / "src").rglob("*.cpp"):
        gates |= set(re.findall(
            r"frameXmlOwns\(\s*(?:ui::)?UiElement::(\w+)\s*\)",
            path.read_text(errors="ignore")))

    defaults = set(re.findall(r'"(\w+)"', re.search(
        r"return std::set<std::string>\{(.*?)\};", cpp, re.S).group(1)))
    # "candidates" used to add a list on top of the defaults. It adds
    # nothing now - every element is in the defaults - so the loop it was read
    # out of is gone. Read it as empty rather than as a parse failure, but only
    # when the word is still there: if the whole mechanism is renamed, this
    # should go back to failing loudly.
    if "candidates" not in cpp:
        print("no 'candidates' tier in the takeover source - this reads it")
        return 1
    extra = re.search(r"for \(const char\* name : \{(.*?)\}\) \{", cpp, re.S)
    candidates = set(re.findall(r'"(\w+)"', extra.group(1))) if extra else set()

    live = sorted(e for e, n in names.items()
                  if n in defaults or n in candidates)
    no_gate = [e for e in live if e not in gates and draws[e]]
    no_suppress = [e for e in live if e not in suppress and draws[e]]
    # And the other way: a row hiding FrameXML's frame with nothing behind it.
    stale_suppress = [e for e in live if e in suppress and not draws[e]]

    print(f"{len(live)} elements handed over, {len(defaults)} by default and "
          f"{len(candidates)} named by 'candidates'\n")

    print(f"{len(no_gate)} with no frameXmlOwns gate - this client goes on "
          f"drawing when the element is handed over:")
    for e in no_gate:
        print(f'    {e:<20} "{names[e]}"')
    if not no_gate:
        print("    (none)")

    print(f"\n{len(no_suppress)} with no suppression entry - FrameXML's frame "
          f"is drawn while this client still owns it:")
    for e in no_suppress:
        print(f'    {e:<20} "{names[e]}"')
    if not no_suppress:
        print("    (none)")

    print(f"\n{len(stale_suppress)} suppressed while this client draws nothing "
          f"- FrameXML's frame is hidden with nothing behind it:")
    for e in stale_suppress:
        print(f'    {e:<20} "{names[e]}"')
    if not stale_suppress:
        print("    (none)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
