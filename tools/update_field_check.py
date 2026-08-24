#!/usr/bin/env python3
"""Update-field indices that disagree with the server's own header.

    tools/update_field_check.py [--server PATH]

An update field is read by index out of Data/expansions/<x>/update_fields.json.
A wrong index does not fail: it reads whatever the server happened to put at
that slot, or nothing at all, and the value comes out zero or stale forever.
What that looks like is a corpse that never registers as lootable, a stealthed
NPC that never reveals, a title that never displays, a currency stuck at zero -
each of which reads as a feature that does not work rather than as a number in
the wrong place.

Five were wrong on the first run, all WotLK: UNIT_FIELD_BYTES_1 and
UNIT_DYNAMIC_FLAGS were being read from 137 and 147, which are inside the unit
block and so look plausible, while the server writes them at 74 and 79 - 147 is
UNIT_FIELD_PADDING. PLAYER_CHOSEN_TITLE and the two PvP currencies were past
PLAYER_END entirely and could never have arrived.

WHAT IT COMPARES

Every name in the JSON against the same name in EUnitFields and its siblings in
AzerothCore's UpdateFields.h, resolved through OBJECT_END, UNIT_END and
PLAYER_END. That header is what the server writes from, so it is the wire.

WOTLK ONLY, AND WHY

There is one server here and it is 3.3.5a, so only the WotLK layout can be
checked. The Classic, TBC and Turtle files carry different values for the same
names - plausible ones, and unverifiable from here. Checking them against this
header would report three expansions of noise, which is the mistake the DBC
check made before it learned to pick one.

WHAT IT CANNOT SEE

Names the header does not define - PLAYER_QUEST_LOG_START and the other
block-start aliases this client invents for its own convenience. They are
counted and listed, not judged.
"""
import argparse
import json
import re
import sys
from pathlib import Path
import os

ROOT = Path(__file__).resolve().parent.parent
LAYOUT = ROOT / "Data/expansions/wotlk/update_fields.json"

#: The three block bases every other field is written relative to.
BASES = {"OBJECT_END": 6, "UNIT_END": 148, "PLAYER_END": 1326}


def server_fields(header):
    """name -> absolute index, for every field defined off a known base."""
    text = Path(header).read_text(errors="ignore")
    out = {}
    for base, value in BASES.items():
        out[base] = value
    for m in re.finditer(
            r"^\s*([A-Z][A-Z0-9_]*)\s*=\s*(OBJECT_END|UNIT_END|PLAYER_END)"
            r"\s*\+\s*(0x[0-9A-Fa-f]+)", text, re.M):
        out[m.group(1)] = BASES[m.group(2)] + int(m.group(3), 16)
    return out


# Where the server source lives is the machine's business rather than this
# file's. WOWEE_SERVER_SRC names the root of an AzerothCore checkout; --server
# still overrides it outright. This carried one contributor's home directory
# until now, so the sweep ran on exactly one machine and skipped silently on
# every other.
def _serverDefault(*parts):
    root = os.environ.get("WOWEE_SERVER_SRC", "").strip()
    return str(Path(root).joinpath(*parts)) if root else ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--server", default=_serverDefault(
        "src/server/game/Entities/Object/Updates/UpdateFields.h"))
    args = ap.parse_args()
    if not Path(args.server).is_file():
        print(f"server header not found at {args.server or '<unset>'} - set WOWEE_SERVER_SRC to an "
              "AzerothCore checkout, or pass --server")
        return 0

    server = server_fields(args.server)
    ours = json.loads(LAYOUT.read_text())

    rows, agree, unnamed = [], 0, []
    for name, index in sorted(ours.items()):
        if name not in server:
            unnamed.append(name)
            continue
        if server[name] != index:
            rows.append((name, index, server[name]))
        else:
            agree += 1

    print(f"{len(ours)} WotLK update fields asserted, {agree} agree with the "
          f"server, {len(unnamed)} it does not name\n")
    print(f"{len(rows)} disagree with the server's own header:\n")
    for name, ours_i, server_i in rows:
        print(f"  {name:44} ours={ours_i:<6} server={server_i}")
    if not rows:
        print("  (none)")
    if unnamed:
        print("\nnot in the header, so not judged: " + ", ".join(unnamed))
    return 0


if __name__ == "__main__":
    sys.exit(main())
