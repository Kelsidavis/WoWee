#!/usr/bin/env python3
"""What this client says an unset CVar is, against what Blizzard's own tables say.

FrameXML declares a default beside each CVar its option panels drive -
`{ default = "0", cvar = "lockActionBars" }` - and those are the values the real
client shipped with. This client answers `GetCVar` for 133 CVars from its own
list. Where both have an opinion they should agree, because a player who has
touched nothing is entitled to the game they expect.

Two do not agree, and both are decisions rather than drift, so they are named
below with the reason. The rule they follow is that **the default has to
describe what the client actually does**: this client draws buff durations on
aura icons and floating text for dodges, parries and misses, and answering 0 for
either would tick the box off while the thing it names is on screen - a control
that reads as off, does nothing when switched on, and is the exact fault the
options audit spent itself on.

Changing the answer without changing the drawing would be worse than the
divergence. Changing the drawing is a product decision about a 2004 default,
not a bug fix, and is not one to make quietly inside a sweep.

So this exists to catch the third case: a divergence nobody decided on. Add to
KNOWN only with the reason, the way these two carry one.

Data/interface is not in the repository - the extracted interface is the
player's own - so this skips rather than fails when it is absent, the same rule
the other interface sweeps use.
"""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
API = ROOT / "src/addons/lua_system_api.cpp"
FRAMEXML = ROOT / "Data/interface/framexml"

# Divergences that were decided on, with why. The value is what this client
# answers; anything else differing is reported.
KNOWN = {
    "buffdurations": (
        "1",
        "this client draws the remaining time on aura icons, so the box has to "
        "read ticked while it is doing it; Blizzard shipped that off"),
    "fctdodgeparrymiss": (
        "1",
        "this client draws floating text for dodges, parries and misses, same "
        "reasoning as buffDurations; Blizzard shipped that off"),
}

FALLBACK = re.compile(
    r'((?:n == "[a-z0-9_]+"\s*(?:\|\|\s*)?)+)\)?\s*(?:\{\s*)?lua_pushstring\(L,\s*"([^"]*)"\)',
    re.IGNORECASE | re.DOTALL)
NAME = re.compile(r'n == "([a-z0-9_]+)"', re.IGNORECASE)
# Blizzard writes the pair in both orders across its option tables.
PAIRS = (re.compile(r'default\s*=\s*"([^"]*)"\s*,\s*cvar\s*=\s*"([^"]*)"'),
         re.compile(r'cvar\s*=\s*"([^"]*)"\s*,\s*default\s*=\s*"([^"]*)"'))


def main():
    if not API.is_file() or not FRAMEXML.is_dir():
        print("the extracted interface is missing - nothing compared.")
        return 0

    blizzard = {}
    for path in sorted(FRAMEXML.glob("*.lua")):
        text = path.read_text(errors="ignore")
        for value, cvar in PAIRS[0].findall(text):
            blizzard.setdefault(cvar.lower(), value)
        for cvar, value in PAIRS[1].findall(text):
            blizzard.setdefault(cvar.lower(), value)

    ours = {}
    for match in FALLBACK.finditer(API.read_text()):
        for name in NAME.findall(match.group(1)):
            ours.setdefault(name.lower(), match.group(2))

    shared = sorted(set(blizzard) & set(ours))
    print(f"Blizzard declares {len(blizzard)} defaults, this client answers "
          f"{len(ours)}, {len(shared)} in both.\n")

    undecided, drifted = [], []
    for name in shared:
        if blizzard[name] == ours[name]:
            continue
        known = KNOWN.get(name)
        if known is None:
            undecided.append((name, blizzard[name], ours[name]))
        elif known[0] != ours[name]:
            drifted.append((name, known[0], ours[name]))

    for name, theirs, mine in undecided:
        print(f"  {name}: Blizzard ships {theirs!r} and this client answers {mine!r}, "
              "and nothing here says that was decided")
    for name, expected, mine in drifted:
        print(f"  {name}: recorded as answering {expected!r} and now answers {mine!r} - "
              "the decision changed without the note changing")

    if undecided or drifted:
        print(f"\n{len(undecided) + len(drifted)} default(s) differ from the game's "
              "own without a reason recorded.")
        return 0

    # Both sides are found by pattern, so a pattern that stops matching reports
    # the same silence as agreement.
    if len(shared) < 25:
        print(f"\nonly {len(shared)} CVars were declared by both, which is fewer "
              "than either side has - the parse stopped matching rather than "
              "finding nothing wrong.")
        return 1

    print(f"every default this client answers matches the game's own, but for "
          f"{len(KNOWN)} that are written down here with the reason.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
