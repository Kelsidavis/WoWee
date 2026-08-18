#!/usr/bin/env python3
"""Struct fields everything reads and nothing ever fills in.

    tools/read_never_written_check.py

WHY THIS FINDS WHAT THE OTHER SWEEPS CANNOT

parsed_never_read_check asks what the server sent that nothing collects. This
is its mirror: a field with readers all over the client and no writer anywhere,
so every one of them agrees on whatever the declaration's initialiser happened
to say. Nothing is missing, nothing raises, and the value is plausible - which
is why it survives. The other direction cannot see it, because there is no
parser line to notice the absence of.

Found by hand on 2026-08-06, which is why this exists:
PetitionInfo::signaturesRequired was read in five places and assigned in none.
It sat at its declared default of nine, so the counter under a charter's
signature list read "x / 9" whatever the realm was configured for, and
CanSignPetition compared against the same nine and left the Sign button live
past the real requirement. The value that filled it was in a packet the client
never asked for.

WHAT IT COMPARES

Fields of the structs in world_packets.hpp and handler_types.hpp against every
member access in the tree, split into reads and writes.

WHAT COUNTS AS A WRITE, AND THE THREE WAYS THAT GOES WRONG

All three were hit while writing this and all three read as findings:

  * A container is filled, not assigned. `attachments`, `auctions`, `spells`
    and twenty more are only ever push_back'd, and reading `=` alone reported
    every one of them. Mutating calls count.
  * A field passed as a reference out-parameter is written by the callee -
    `readCastResultArgs(packet, data.result, data.miscArg, data.miscArg2)`
    fills three fields with no `=` in sight. These are named in EXPECTED one at
    a time rather than detected, because every rule for spotting them also
    matches an ordinary argument: the first attempt excused any field passed to
    a call, which includes `ImGui::Text("%u / %u", a.count, a.required)` - and
    that is the very field this sweep was written for. The canary caught it.
  * A whole-struct assignment writes every field at once and names none of
    them. `result = {fields[0], fields[2], ...}` is how AuctionMailSubject is
    filled, and memory records the same shape twice over elsewhere. There is no
    good way to attribute that to a field, so those are listed in EXPECTED with
    the site that does it.

WHAT IT CANNOT SEE

A field written through a memcpy or a pointer alias, and one whose writer runs
only on a path nothing reaches - which is the *next* question and not this one.
It also says nothing about whether the value written is right.
"""
import collections
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
HDRS = [ROOT / "include/game/world_packets.hpp",
        ROOT / "include/game/handler_types.hpp"]
HDRSET = {h.resolve() for h in HDRS}

#: Fields checked once and found to be written after all, with what does it.
#: A set rather than a count, so a new one cannot hide inside an accepted total.
EXPECTED = {
    # parseAuctionMailSubject ends `result = {fields[0], fields[2], ...}`,
    # which writes all four fields and names none. The other three are read
    # through the same struct and so are covered by the container rule; this
    # one is not.
    "response": "whole-struct assignment in parseAuctionMailSubject",
    # readCastResultArgs takes both by reference and fills them from the
    # trailing ids on SMSG_CAST_FAILED, per expansion. Written by a callee, so
    # no assignment to them appears anywhere.
    "miscArg": "reference out-parameter of readCastResultArgs",
    "miscArg2": "reference out-parameter of readCastResultArgs",
}

MEMBER = re.compile(r"(?:\.|->)\s*(\w+)")
ASSIGN = re.compile(r"(?:\.|->)\s*(\w+)\s*(?:=[^=]|\+=|-=|\|=|\[)"
                    r"|^\s*(\w+)\s*=[^=]", re.M)
MUTATE = re.compile(r"(?:\.|->)\s*(\w+)\s*\.\s*"
                    r"(?:push_back|emplace_back|emplace|insert|resize|assign|clear)")


def struct_fields():
    """field name -> {structs declaring it}"""
    out = {}
    for h in HDRS:
        text = h.read_text(errors="ignore")
        for m in re.finditer(r"\bstruct\s+(\w+)\s*\{(?:\.\w+\s*=\s*)?", text):
            name, i, depth = m.group(1), m.end(), 1
            while i < len(text) and depth:
                if text[i] == "{":
                    depth += 1
                elif text[i] == "}":
                    depth -= 1
                i += 1
            for f in re.findall(r"^\s+(?:[\w:<>,\s\*&]+?)\b(\w+)\s*(?:=[^;]*)?;",
                                text[m.end():i - 1], re.M):
                # Short names collide with ordinary locals too often to count.
                if len(f) >= 5:
                    out.setdefault(f, set()).add(name)
    return out


def scan(fields, skip=None):
    read, written = collections.Counter(), collections.Counter()
    sources = list((ROOT / "src").rglob("*.cpp")) + list((ROOT / "include").rglob("*.hpp"))
    for p in sources:
        if p.resolve() in HDRSET or p.name == skip:
            continue
        s = re.sub(r"//[^\n]*|/\*.*?\*/", "", p.read_text(errors="ignore"), flags=re.S)
        for m in MEMBER.finditer(s):
            if m.group(1) in fields:
                read[m.group(1)] += 1
        for m in ASSIGN.finditer(s):
            n = m.group(1) or m.group(2)
            if n in fields:
                written[n] += 1
        for m in MUTATE.finditer(s):
            if m.group(1) in fields:
                written[m.group(1)] += 1
    return sorted(f for f in fields if read[f] and not written[f]), read


def main():
    fields = struct_fields()

    # The canary. SocialHandler::handlePetitionQueryResponse is the only writer
    # of signaturesRequired, so hiding that file must bring the field back. Two
    # of the three write-forms above were added *because* their absence made
    # this report empty while it was seeing nothing, and an empty report is
    # indistinguishable from a clean one.
    hidden, _ = scan(fields, skip="social_handler.cpp")
    if "signaturesRequired" not in hidden:
        print("  CANARY FAILED: hiding the only writer did not surface the field.")
        print("  The report below means nothing. Do not believe the zero.\n")
        return 1

    plain, read = scan(fields)
    rows = [f for f in plain if f not in EXPECTED]

    print(f"{len(fields)} struct field names, {len(EXPECTED)} settled\n")
    print(f"{len(rows)} read and never written:\n")
    for f in rows:
        print(f"  {','.join(sorted(fields[f])):38} {f:26} reads={read[f]}")
    if not rows:
        print("  (none)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
