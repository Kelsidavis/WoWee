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

For every element named in the defaults or the candidates: that some entry in
kSuppress names it, and that some file under src/ gates on it.

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

# Elements this client does not draw, so there is nothing to gate. Either it
# never had its own version, or that version has been removed now that
# FrameXML's is the one on screen. An entry here is a deletion recorded, not an
# exemption: the element must have no drawing left in src/ui at all.
NOTHING_TO_GATE = {
    # src/ui/crafting_window.cpp, removed once the trade skill window was
    # FrameXML's. Its two difficulty helpers moved to window_manager.cpp,
    # where the trainer list still uses them.
    "TradeSkill",
    # src/ui/window_manager.cpp's renderStableWindow, removed once the stable
    # was FrameXML's. Its pet list comes from the same handler either way.
    "Stable",
    # src/ui/window_manager.cpp's renderGmTicketWindow, removed once the help
    # panel was FrameXML's. Both openers - the escape menu button and the slash
    # command - call ToggleHelpFrame now.
    "Help",
}


def main():
    if not TAKEOVER.exists():
        print(f"no {TAKEOVER}")
        return 1
    cpp = TAKEOVER.read_text(errors="ignore")

    table = re.search(r"\{UiElement::PlayerFrame.*?\n\};", cpp, re.S)
    names = dict(re.findall(r'\{UiElement::(\w+),\s*"([a-z]\w*)"\}', table.group(0)))
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
    no_gate = [e for e in live if e not in gates and e not in NOTHING_TO_GATE]
    no_suppress = [e for e in live if e not in suppress]

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
    return 0


if __name__ == "__main__":
    sys.exit(main())
