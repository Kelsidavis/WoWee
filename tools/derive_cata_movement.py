#!/usr/bin/env python3
"""Derive Cataclysm's per-opcode movement wire layouts from a 4.3.4 core.

4.3.4 marshals movement as a bit stream interleaved with the byte stream, and
the order of both is chosen per opcode with no pattern to it: MSG_MOVE_START_
FORWARD writes position Y, Z, X and then guid mask bits 5, 2, 0. There are
around a hundred such orders and no way to derive one from another, so they are
read off the core that speaks the protocol rather than written out by hand.

The core keeps them as arrays of MovementStatusElements in
src/server/game/Movement/MovementStructures.cpp, one per opcode, mapped by a
switch in GetMovementStatusElementsSequence. Both are parsed here.

Usage:
    tools/derive_cata_movement.py [core-root] [-o <output>]

The core is taken from WOWEE_CATA_CORE when no path is given. With neither,
there is nothing to derive from and the script says so and stops, because
tools_run_check runs every tool here with no arguments and a tool that cannot
run counts as a sweep that cannot run.

See docs/plan-cataclysm.md. Re-run when the core moves; do not edit the output.
"""

import argparse
import json
import re
import sys
from pathlib import Path

# `const` is optional: three of the arrays in the core are declared without it,
# and requiring it silently dropped them along with their opcodes.
ARRAY_RE = re.compile(
    r'^MovementStatusElements\s+(?:const\s+)?(\w+)\s*\[\]\s*=\s*\{(.*?)\};',
    re.M | re.S)
CASE_RE = re.compile(
    r'case\s+((?:MSG|CMSG|SMSG)_\w+)\s*:\s*return\s+(\w+)\s*;', re.M)
ELEMENT_RE = re.compile(r'\bMSE\w+\b')


def parse(core_root: Path):
    source = core_root / "src/server/game/Movement/MovementStructures.cpp"
    if not source.is_file():
        sys.exit(f"not found: {source}\n"
                 "Pass the root of a 4.3.4 core, e.g. /media/k/vbox/wowee-cata/core")
    text = source.read_text(errors="replace")

    arrays = {}
    for name, body in ARRAY_RE.findall(text):
        elements = ELEMENT_RE.findall(body)
        # MSEEnd is the loop terminator, not a field. Dropping it here keeps
        # the reader from having to know that.
        if elements and elements[-1] == "MSEEnd":
            elements = elements[:-1]
        arrays[name] = elements

    sequences = {}
    unresolved = []
    for opcode, array in CASE_RE.findall(text):
        if array in arrays:
            sequences[opcode] = arrays[array]
        else:
            unresolved.append((opcode, array))
    return arrays, sequences, unresolved


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("core_root", type=Path, nargs="?",
                    help="root of a 4.3.4 core; defaults to $WOWEE_CATA_CORE")
    ap.add_argument("-o", "--output", type=Path,
                    default=Path("Data/expansions/cata/movement_sequences.json"))
    args = ap.parse_args()

    core_root = args.core_root
    if core_root is None:
        from os import environ
        env = environ.get("WOWEE_CATA_CORE")
        if not env:
            print("no 4.3.4 core given and WOWEE_CATA_CORE is unset; "
                  "nothing to derive")
            return
        core_root = Path(env)

    arrays, sequences, unresolved = parse(core_root)
    if not sequences:
        sys.exit("no sequences parsed; the core's layout has changed")
    for opcode, array in unresolved:
        print(f"warning: {opcode} names {array}, which was not parsed",
              file=sys.stderr)

    vocabulary = sorted({e for seq in sequences.values() for e in seq})
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({
        "_source": "MovementStructures.cpp, via tools/derive_cata_movement.py",
        "_note": "Derived. Re-run the script rather than editing this file.",
        "elements": vocabulary,
        "sequences": dict(sorted(sequences.items())),
    }, indent=1) + "\n")

    print(f"{len(arrays)} arrays parsed, {len(sequences)} opcodes mapped, "
          f"{len(vocabulary)} distinct elements -> {args.output}")


if __name__ == "__main__":
    main()
