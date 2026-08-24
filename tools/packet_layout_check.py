#!/usr/bin/env python3
"""Packets the client reads in a different shape from the one the server wrote.

The two size sweeps beside this one ask whether a packet is long enough. This
asks the harder question: whether the fields line up. A handler can guard the
right number of bytes, run every time, and still be wrong from the first field
on - SMSG_BATTLEFIELD_MGR_EJECTED read a guid where the server wrote a battle
id, and it was the *length* that gave it away, not the layout. Two packets of
the same size disagreeing about what those bytes mean is invisible to both of
the other sweeps and to the client itself: nothing raises, nothing logs, the
numbers are simply wrong.

    tools/packet_layout_check.py [--server PATH]

WHAT IT COMPARES

A sequence of widths, not a sequence of names. From the server, the run of
`data << uintN(...)` after `WorldPacket data(SMSG_X, ...)`. From this client,
the run of `packet.readUIntN()` after the handler's opcode. Each stops at the
first thing whose width is not fixed - a string, a packed guid, a loop, a
branch - because past that point the sequences cannot be lined up by position.

A mismatch in that prefix is reported with both readings, so the disagreement
can be read rather than taken on trust.

WHY WIDTHS AND NOT TYPES

Signedness does not change where a field ends, and this is about position. A
client reading int32 where the server wrote uint32 is a separate question and
not one a byte count can answer.

WHAT IT FOUND, AND WHAT SURVIVED

Seven more on 2026-08-05, once `data << guid` and GetPackGUID() stopped ending
the prefix - coverage went from 76 opcodes to 117. SMSG_THREAT_UPDATE built its
list from nothing, SMSG_LOOT_START_ROLL missed itemCount so the roll timer and
the need/greed mask were both wrong, SMSG_MODIFY_COOLDOWN took half a player
guid as the change in milliseconds, SMSG_SOCKET_GEMS_RESULT reported every
socketing as failed because it read a result field the server does not send,
and three read a guid in the wrong shape. The eighth was the sweep's own blind
spot rather than a fault: a handler that decoded its packed guid with a byte
loop written out in place, which now goes through readPackedGuid.

Six on the first run, all real: SMSG_CHAR_RENAME read a four-byte result where
the server writes one byte, SMSG_BATTLEFIELD_MGR_ENTERED read a guid the server
does not send, SMSG_GMRESPONSE_STATUS_UPDATE read a ticket id that is not
there, and the three GM ticket answers each read one byte of a uint32 - which
little-endian kept working for the delete and broke for the other two.

Two are reported and are not faults. Both have been read and neither should be
silenced, because the shapes really do differ and a future edit could make one
of them matter:

  * SMSG_AUCTION_OWNER_NOTIFICATION - the client reads the server's uint64 as
    two uint32s. Every field after it therefore lands on the same offset it
    would have anyway, and the item entry, which is what this handler is for,
    is read correctly at offset twenty.
  * SMSG_PARTY_MEMBER_STATS_FULL - was an artifact, and the artifact turned
    out to be worth fixing rather than annotating. A region ended at the next
    opcode, and the pattern that found opcodes required a letter in front of
    MSG_ - so MSG_RAID_READY_CHECK did not end one, the region ran through its
    handler, and the guid that ready check reads was credited here. Both this
    and packet_size_check.py had it; two entries came off that report as well.

WHAT IT CANNOT SEE

  * Anything after the first variable-width field. Most packets have one early,
    which is why the compared prefix is often short. The run prints how many
    opcodes had a prefix worth comparing at all, and how many of those are two
    fields or fewer - the zero means nothing without that number beside it.

    This is not theoretical. MSG_LIST_STABLED_PETS is written as
    uint32/uint32/uint32/name/uint8 and was read as
    uint32/uint32/uint32/name/uint32/uint8, so the display id it invented
    swallowed the flag and three bytes of the next pet, every pet after the
    first was read at the wrong offset, and the last was dropped. The prefix
    this compares ends at the name, three fields before any of that.

  * A misalignment between fields of the same width, which is invisible here by
    construction: the two readings line up byte for byte and only the *meaning*
    has slid. SMSG_LFG_PLAYER_REWARD was read that way - the server writes a
    constant 1, then money, then XP, and the client took the constant as the
    money, the money as the XP and the XP as an item count, then looped that
    many times over the rest. Five uint32s in a row, so this reported a match
    throughout. Deleting one of them makes it report instantly, which is how
    the coverage was confirmed; nothing about that helps when the count is
    right. Reading the server's writer is the only check for this shape.
  * A handler that reads through a helper or a parser class rather than
    directly. Those are the larger packets, and they are the ones where a
    misparse is hardest to spot by eye - a real gap in this, not a small one.
  * Two fields of the same width swapped. Identical widths line up perfectly;
    only the client's own reading of what the fields mean can catch that.
"""
import argparse
import re
import sys
from pathlib import Path
import os

ROOT = Path(__file__).resolve().parent.parent

SERVER_WIDTH = {"uint8": 1, "int8": 1, "uint16": 2, "int16": 2, "uint32": 4,
                "int32": 4, "uint64": 8, "int64": 8, "float": 4, "double": 8}
CLIENT_WIDTH = {"readUInt8": 1, "readInt8": 1, "readUInt16": 2, "readInt16": 2,
                "readUInt32": 4, "readInt32": 4, "readUInt64": 8, "readInt64": 8,
                "readFloat": 4, "readDouble": 8}

# Opcodes whose two readings differ and are still the same bytes, with the
# reason. Judged rather than silenced: each has been read against the server's
# writer, and the entry says what makes the difference harmless so the next
# person does not have to work it out again.
SETTLED = {
    # `data << uint8(0)  // some string` is how AzerothCore spells an empty
    # C string, and an empty string on the wire is exactly one NUL byte. The
    # client reading it with readString consumes that byte and answers "",
    # which is what the field means. Every field after it lands where it
    # should. The petition response does this in eleven places.
    "SMSG_PETITION_QUERY_RESPONSE":
        "the server's uint8(0) is an empty string, which is the byte the "
        "client's readString consumes",
}

# The declared type of a bare `data << name`. AzerothCore writes a great many
# fields that way rather than through a cast, and every one of them used to end
# the prefix - so a packet whose second field is a plain variable was compared
# one field deep and reported as agreeing.
DECLARED_WIDTH = {
    "uint8": 1, "int8": 1, "uint16": 2, "int16": 2, "uint32": 4, "int32": 4,
    "uint64": 8, "int64": 8, "float": 4, "double": 8, "bool": 1,
    "std::string": "S", "string": "S",
    "ObjectGuid": 8,
}

# Calls whose return type is known without resolving anything. A name is a
# NUL-terminated string on the wire and lines the two sequences up exactly as a
# packed guid does, so it is a token rather than a stop.
KNOWN_CALL_WIDTH = {
    "GetName": "S",
}


def _declared_width(src, before, name):
    """The width of a bare `data << name`, from the name's declaration.

    The scope searched is the text back to the previous top-level close brace,
    which is the enclosing function often enough to be worth having and never
    reaches into another one. An unresolved name still ends the prefix, so a
    missed declaration costs coverage rather than correctness.
    """
    scope_start = src.rfind("\n}", 0, before)
    scope = src[scope_start if scope_start != -1 else 0:before]
    decl = re.findall(
        r"(?:^|[\s(,])((?:std::)?\w+)(?:\s+const)?\s*&?\s*\b"
        + re.escape(name) + r"\b\s*(?:=|;|,|\))", scope)
    for kind in reversed(decl):
        if kind in DECLARED_WIDTH:
            return DECLARED_WIDTH[kind]
        if kind in ("return", "case", "const"):
            continue
        # A type this does not know is not a guess to make: the field could be
        # an enum of any width, a struct, or something with its own operator<<.
        return None
    return None


def server_layouts(server_root):
    """SMSG name -> (the widths it writes, whether that is the whole packet).

    The second half matters as much as the first. Most of the prefixes that
    end after one or two fields end because the packet ends - the next line
    hands it to SendPacket - and those are compared in full. Counting them as
    "could not be sized" said this sweep sees far less than it does, and a
    coverage figure that is wrong in the pessimistic direction is still wrong:
    it hides which packets are actually uncompared.
    """
    out = {}
    for path in Path(server_root).rglob("*.cpp"):
        src = path.read_text(errors="ignore")
        for m in re.finditer(r"WorldPacket\s+(\w+)\s*\(\s*(SMSG_\w+)", src):
            var, opcode = m.group(1), m.group(2)
            widths = []
            whole = False
            for line in src[m.end():].split("\n")[1:60]:
                s = line.strip()
                if not s or s.startswith("//"):
                    continue
                w = re.match(r"[*]?" + re.escape(var) + r"\s*<<\s*(\w+)\s*\(", s)
                if w and w.group(1) in SERVER_WIDTH:
                    widths.append(SERVER_WIDTH[w.group(1)])
                    continue
                # A packed guid has no fixed width, but it is unambiguous on
                # both sides - WriteAsPacked here, readPackedGuid there - so it
                # is a token the sequences can be lined up on rather than a
                # stop. Most packets carry one early, and stopping at it left
                # everything after the guid uncompared.
                if re.match(r"[*]?" + re.escape(var) + r"\s*<<\s*[\w>.\[\]()-]*"
                            r"(?:WriteAsPacked|GetPackGUID)\(\)", s):
                    widths.append("P")
                    continue
                # The third spelling of a packed guid, and the one that reads
                # least like one: `data.appendPackGUID(x)` is a method on the
                # buffer rather than anything shifted into it. It ended the
                # prefix of SMSG_CRITERIA_UPDATE at its first field.
                if re.match(re.escape(var) + r"\s*\.\s*appendPackGUID\s*\(", s):
                    widths.append("P")
                    continue
                # A plain ObjectGuid is eight bytes, not a variable field:
                # operator<<(ByteBuffer&, ObjectGuid const&) writes
                # uint64(guid.GetRawValue()). It was ending the prefix anyway,
                # and it is what ends it most often by a wide margin - fifty-odd
                # opcodes stopped at `data << guid` or `<< x->GetGUID()` with
                # everything after them uncompared. Checked after the packed
                # spelling above, which is a different shape.
                if re.match(r"[*]?" + re.escape(var) +
                            r"\s*<<\s*[\w>.\[\]()-]*(?:[Gg][Uu][Ii][Dd]|GUID)"
                            r"(?:\(\))?\s*;", s):
                    widths.append(8)
                    continue
                # A string literal and a name are both NUL-terminated on the
                # wire, so they line the two sequences up rather than ending
                # them - the same argument as the packed guid above. This is
                # the field that ends most of the remaining prefixes, and
                # MSG_LIST_STABLED_PETS was misread from the field *after* its
                # name, three past where the comparison used to stop.
                if re.match(r"[*]?" + re.escape(var) + r'\s*<<\s*"', s):
                    widths.append("S")
                    continue
                call = re.match(r"[*]?" + re.escape(var) +
                                r"\s*<<\s*[\w>.\[\]()-]*?(\w+)\(\s*\)\s*;", s)
                if call and call.group(1) in KNOWN_CALL_WIDTH:
                    widths.append(KNOWN_CALL_WIDTH[call.group(1)])
                    continue
                # `data << count;` - the width is the variable's declared type.
                bare = re.match(r"[*]?" + re.escape(var) + r"\s*<<\s*(\w+)\s*;", s)
                if bare:
                    width = _declared_width(src, m.end(), bare.group(1))
                    if width is not None:
                        widths.append(width)
                        continue
                # A guid picked by a ternary is still a guid, whichever arm
                # runs: `data << (creature ? creature->GetGUID() :
                # ObjectGuid::Empty)`. Both arms are eight bytes, so unlike
                # the width-by-expansion ternaries on the client side there is
                # nothing here to guess.
                if re.match(r"[*]?" + re.escape(var) + r"\s*<<\s*\([^;]*"
                            r"(?:GetGUID\(\)|ObjectGuid::Empty)[^;]*\)\s*;", s):
                    widths.append(8)
                    continue
                # A member spelt `name` is a string in every AzerothCore
                # structure that has one. Narrow on purpose: a member of some
                # other name could be any width, and guessing turns this sweep
                # into a generator of false reports.
                if re.match(r"[*]?" + re.escape(var) +
                            r"\s*<<\s*[\w>.\[\]-]*\b\w*[Nn]ame\s*;", s):
                    widths.append("S")
                    continue
                # Not a write at all. The packet ending here is the ordinary
                # case and means the prefix is the whole of it; anything else -
                # a branch, a loop, a helper that appends more - means the
                # comparison stops short and should be counted as such.
                if re.search(r"\bSend\w*\s*\(\s*&?\s*" + re.escape(var) + r"\b", s):
                    whole = True
                break
            # The longest reading wins: several call sites build the same
            # opcode and only the fullest one describes the whole prefix.
            if widths and len(widths) > len(out.get(opcode, ([], False))[0]):
                out[opcode] = (widths, whole)
    return out


def _balanced(src, open_at):
    """The braced block starting at open_at, so a function body ends where it does."""
    depth, i = 0, open_at
    while i < len(src):
        if src[i] == "{":
            depth += 1
        elif src[i] == "}":
            depth -= 1
            if depth == 0:
                return src[open_at:i + 1]
        i += 1
    return src[open_at:]


def _handler_bodies():
    """Named handler -> (body, the name its packet argument goes by).

    A hundred and fifty of this client's registrations are one line that calls
    a handler by name, and those are the large packets - the quest, guild and
    auction replies, where a misparse is hardest to see by eye. Reading only
    the registration measured them as reading nothing.
    """
    out = {}
    for path in (ROOT / "src/game").rglob("*.cpp"):
        src = path.read_text(errors="ignore")
        for m in re.finditer(
                r"void\s+\w+::(\w+)\s*\(\s*network::Packet\s*&\s*(\w+)"
                r"[^)]*\)\s*\{", src):
            out[m.group(1)] = (_balanced(src, m.end() - 1), m.group(2))
    return out


def _widths_in(body, var):
    """The fixed-width reads at the head of a body, in order."""
    widths = []
    # A guard or a log line between two reads is not a field and does not end
    # the run; anything that consumes bytes and is not a plain integer does.
    #
    # A read inside a conditional expression ends the run rather than being
    # counted. This client picks its width by expansion in places -
    #
    #     spellId = classicSpellId ? packet.readUInt16() : packet.readUInt32();
    #
    # - and taking the first branch reported four packets as misread when the
    # WotLK branch beside it was right. Which arm runs is not knowable here, so
    # the honest answer is to stop rather than to guess.
    for m in re.finditer(r"\b" + re.escape(var) + r"\.(\w+)\s*\(", body):
        call = m.group(1)
        line_start = body.rfind("\n", 0, m.start()) + 1
        line_end = body.find("\n", m.start())
        line = body[line_start:line_end if line_end != -1 else len(body)]
        if "?" in line and ":" in line:
            break
        if call in CLIENT_WIDTH:
            widths.append(CLIENT_WIDTH[call])
        elif call == "readPackedGuid":
            widths.append("P")
        elif call == "readString":
            widths.append("S")
        elif call.startswith("read") or call in ("skipAll",):
            break
    return widths


def client_layouts():
    """SMSG name -> the widths this client reads, up to the first variable one."""
    handlers = _handler_bodies()
    out = {}
    for path in (ROOT / "src/game").rglob("*.cpp"):
        src = path.read_text(errors="ignore")
        # An opcode with no letter before MSG_ - MSG_RAID_READY_CHECK,
        # MSG_MOVE_* and the rest - has to end a region too. It did not, so a
        # region ran straight through the next handler and credited its reads
        # here: SMSG_PARTY_MEMBER_STATS_FULL was reported reading a guid that
        # belongs to the ready check below it.
        marks = [(m.start(), m.group(1)) for m in
                 re.finditer(r"Opcode::([A-Z]*MSG_\w+)", src)]
        for i, (at, opcode) in enumerate(marks):
            if not opcode.startswith("SMSG_"):
                continue
            end = marks[i + 1][0] if i + 1 < len(marks) else len(src)
            body = src[at:end]
            widths = _widths_in(body, "packet")
            # A registration that only hands the packet to a named handler
            # reads nothing itself. Follow it, once - a handler that delegates
            # again is not chased, because the second hop is where a wrong
            # guess starts costing more than the answer is worth.
            if not widths:
                call = re.search(r"\b(\w+)\s*\(\s*\w+\s*[,)]", body)
                if call and call.group(1) in handlers:
                    inner, var = handlers[call.group(1)]
                    widths = _widths_in(inner, var)
            if widths and len(widths) > len(out.get(opcode, [])):
                out[opcode] = widths
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
    ap.add_argument("--server", default=_serverDefault("src/server/game"))
    args = ap.parse_args()
    # An empty path is not the current directory. Path("").is_dir() is
    # True, so an unset variable otherwise ran the whole sweep against the
    # tree it was launched from and reported nothing in both - which reads
    # exactly like a clean run.
    if not args.server or not Path(args.server).is_dir():
        print(f"server source not found at {args.server or '<unset>'} - set WOWEE_SERVER_SRC to an "
              "AzerothCore checkout, or pass --server")
        return 1

    server = server_layouts(args.server)
    client = client_layouts()
    shared = sorted(set(server) & set(client))

    print(f"{len(server)} server writers with a fixed prefix, "
          f"{len(client)} client readers, {len(shared)} in both")
    # What the comparison could not reach. The zero below should be read
    # against how much of each packet it actually covers - otherwise it reads
    # as "every packet lines up", which is a much larger claim than this sweep
    # can make. MSG_LIST_STABLED_PETS was misread from the field after its name
    # string, three fields past where its prefix ends.
    #
    # The distinction that makes this number honest: a prefix ending at the
    # line that sends the packet *is* the whole packet, and used to be counted
    # as a comparison that had barely started. Eighty-odd of them are that.
    # What is actually uncompared is a prefix cut short by a branch, a loop or
    # a helper that appends more.
    whole = sum(1 for op in shared if server[op][1])
    cut = [op for op in shared if not server[op][1]]
    print(f"{whole} of those are compared end to end - the server's prefix is "
          f"the whole packet")
    print(f"{len(cut)} stop early, at a branch, a loop or a field with no "
          f"width; of those, {sum(1 for op in cut if min(len(server[op][0]), len(client[op])) <= 2)} "
          f"had two fields or fewer to compare\n")

    rows = []
    for op in shared:
        s, c = server[op][0], client[op]
        n = min(len(s), len(c))
        if s[:n] != c[:n] and op not in SETTLED:
            rows.append((op, s[:n], c[:n]))

    print(f"{len(rows)} packet(s) read in a different shape from the one written:\n")
    for op, s, c in rows:
        print(f"  {op}")
        print(f"      server writes {s}")
        print(f"      client reads  {c}")
    if not rows:
        print("  (none)")
    if SETTLED:
        print(f"\n{len(SETTLED)} settled - the readings differ and the bytes "
              f"do not:")
        for op, why in sorted(SETTLED.items()):
            print(f"  {op}\n      {why}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
