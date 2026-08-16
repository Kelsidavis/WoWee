#!/usr/bin/env python3
"""The apply-once latches, and whether each is set where the work happened.

A saved setting reaches its subsystem on the first frame that subsystem exists.
`loadSettings` runs from the GameScreen constructor, before any renderer,
pipeline or audio manager has been built, so the value can only be read there -
handing it over has to wait, and a bool remembers whether it has been handed
over yet.

The bug that shape produces is silent in both directions. Set the latch as soon
as the *outer* thing exists and the setting is marked delivered to something
that was not there to take it: gamma was read from the file and never reached
anything that draws, because the post-process pipeline is built after the
renderer and the latch only waited for the renderer. Never set it and the
client re-applies the same value every frame forever.

So for each `if (!somethingApplied_)` guard this reads the block under it and
asks where `somethingApplied_ = true` sits. If the block tests anything before
doing the work - and all of them do, because the work needs a target - then the
assignment has to be *inside* that test. An assignment at the top of the block
is the fault above: it runs whether or not the guard it sits beside succeeded.

Two shapes are deliberately not faults:

- A guard whose block sets the latch nowhere. The MSAA one calls
  applySavedAntiAliasing and lets it decide; the assignment being elsewhere is
  not the same as the assignment being unguarded.
- An `else` that marks the work done because there is no work to do - no
  anti-aliasing wanted, nothing to hand anybody. Only the `if` body is read.

Canaried by hoisting one assignment out of its inner guard, which this reports
by name.
"""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SOURCES = sorted((ROOT / "src").rglob("*.cpp"))
PANEL_HEADER = ROOT / "include/ui/settings_panel.hpp"

GUARD = re.compile(r'if \(!\s*(?:settingsPanel_\.)?(\w*Applied_)\b')
# The latches this expects to find guarded. A guard that stops matching is the
# failure this floor catches: the parse finding nothing reads exactly like
# every latch being correct.
EXPECTED_GUARDS = 9


def blockAfter(text, start):
    """The braced body following position `start`, and where it begins."""
    open_at = text.find("{", start)
    if open_at == -1:
        return None, None
    depth, i = 0, open_at
    while i < len(text):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return text[open_at + 1:i], open_at + 1
        i += 1
    return None, None


def depthOf(body, at):
    """Brace nesting of a position within a block body."""
    return body.count("{", 0, at) - body.count("}", 0, at)


def main():
    declared = set()
    if PANEL_HEADER.is_file():
        declared = set(re.findall(r'bool (\w*Applied_)\s*=', PANEL_HEADER.read_text()))

    guards, bad, latchesSeen = 0, [], set()
    for path in SOURCES:
        text = path.read_text()
        if "Applied_" not in text:
            continue
        for match in GUARD.finditer(text):
            latch = match.group(1)
            body, bodyStart = blockAfter(text, match.end())
            if body is None:
                continue
            guards += 1
            latchesSeen.add(latch)

            # Every assignment of this latch inside the block it guards.
            assigns = list(re.finditer(re.escape(latch) + r'\s*=\s*true\s*;', body))
            if not assigns:
                continue
            # Does the block test anything before doing the work? A block that
            # applies unconditionally has nothing to be inside of.
            hasInnerTest = any(depthOf(body, m.start()) == 0
                               for m in re.finditer(r'\bif\s*\(', body))
            if not hasInnerTest:
                continue
            for assign in assigns:
                if depthOf(body, assign.start()) == 0:
                    line = text.count("\n", 0, bodyStart + assign.start()) + 1
                    bad.append(
                        f"{path.relative_to(ROOT)}:{line}: {latch} is set at the top of "
                        "its own guard, beside the test for the thing it applies to "
                        "rather than inside it - so the setting is marked delivered "
                        "on a frame where nothing took it, and is never offered again")

    print(f"{guards} apply-once latch guards read, {len(latchesSeen)} distinct latches.\n")
    for entry in bad:
        print(f"  {entry}")

    unguarded = sorted(declared - latchesSeen)
    for latch in unguarded:
        print(f"  {latch} is declared and never guards anything, so nothing it was "
              "meant to hold back is waiting on it")

    if bad or unguarded:
        print(f"\n{len(bad) + len(unguarded)} latch problem(s).")
        return 0

    if guards < EXPECTED_GUARDS:
        print(f"\nonly {guards} guards were found where {EXPECTED_GUARDS} were "
              "expected - the parse stopped matching rather than finding nothing "
              "wrong. Lower this in the same commit if a latch was removed.")
        return 1

    print("every apply-once latch is set inside the test for the thing it applies to.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
