#!/usr/bin/env python3
"""What an unset CVar reads as, said in two places, checked against itself.

A CVar the player has never touched has to answer the same to both sides of this
client or the options panel and the game disagree about the same setting.

The two sides are:

  * `GetCVar` in `src/addons/lua_system_api.cpp`, which is what the interface
    asks. Its fallbacks are what a checkbox ticks itself from.
  * `storedCVarValue(name, default)`, which is what the client reads where the
    setting is actually used - twenty-nine of them, in the combat text, the
    nameplates, the chat filters, the shadow quality and the rest.

Neither is wrong to exist. The interface needs an answer before anything is
stored, and the client needs one at the point of use. What they must not do is
differ: a `fctHealing` that answers 0 to the panel and 1 to the client draws
healing numbers over an unticked box, and the player's only way to fix a setting
that is already doing what they want is to tick it twice.

Nothing else here would notice. Both values are individually reasonable, both
sides work, and the disagreement only shows up on screen - which is the same
shape as the login screen's parallax scale and the preset table that had drifted
in four of ten columns.

Reads the source rather than running anything, so it costs nothing and holds for
files that start using either side after this was written.

The `||` chains matter. A branch may test several names before pushing one
value - `n == "fctdamage" || n == "fcthealing" || n == "fctdodgeparrymiss"` -
and a pattern that only takes the first name reports the other two as having no
answer at all. That is what the first version of this did, and it named two
CVars as unanswered that are answered on the same line.
"""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
API = ROOT / "src/addons/lua_system_api.cpp"

# Every name in a branch, not just the first, then the value that branch pushes.
FALLBACK = re.compile(
    r'((?:n == "[a-z0-9_]+"\s*(?:\|\|\s*)?)+)\)?\s*(?:\{\s*)?lua_pushstring\(L,\s*"([^"]*)"\)',
    re.IGNORECASE | re.DOTALL)
NAME = re.compile(r'n == "([a-z0-9_]+)"', re.IGNORECASE)
DIRECT = re.compile(r'storedCVarValue\(\s*"([A-Za-z0-9_]+)"\s*,\s*"([^"]*)"')


def main():
    if not API.is_file():
        print("lua_system_api.cpp is missing - nothing compared.")
        return 1

    source = API.read_text()
    answers = {}
    for match in FALLBACK.finditer(source):
        for name in NAME.findall(match.group(1)):
            answers.setdefault(name.lower(), match.group(2))

    reads = {}
    for path in sorted(list(ROOT.glob("src/**/*.cpp")) + list(ROOT.glob("include/**/*.hpp"))):
        for name, value in DIRECT.findall(path.read_text()):
            reads.setdefault(name, (value, path.relative_to(ROOT)))

    print(f"GetCVar answers for {len(answers)} CVars; "
          f"the client reads {len(reads)} of them where they are used.\n")

    unanswered, disagree = [], []
    for name, (value, where) in sorted(reads.items()):
        answer = answers.get(name.lower())
        if answer is None:
            unanswered.append((name, value, where))
        elif answer != value:
            disagree.append((name, answer, value, where))

    for name, value, where in unanswered:
        print(f"  {name}: the client reads {value!r} at {where}, and GetCVar has no "
              f"answer - so the panel reads it as unset while the client acts on {value!r}")
    for name, answer, value, where in disagree:
        print(f"  {name}: GetCVar answers {answer!r} and the client reads {value!r} "
              f"at {where}")

    if unanswered or disagree:
        print(f"\n{len(unanswered) + len(disagree)} CVar(s) mean different things "
              "to the panel and to the client.")
        return 0

    # Both sides are found by pattern, so a pattern that stops matching reports
    # the same silence as agreement.
    if len(answers) < 100 or len(reads) < 20:
        print(f"\nonly {len(answers)} answers and {len(reads)} reads were found, "
              "which is fewer than either side has - the parse stopped matching "
              "rather than finding nothing wrong.")
        return 1

    print("every CVar the client reads for itself answers the interface the same way.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
