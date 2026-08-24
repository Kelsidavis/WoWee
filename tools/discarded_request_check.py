#!/usr/bin/env python3
"""Requests this client sends that the server reads off the wire and throws away.

    tools/discarded_request_check.py [--server PATH]

An opcode can be perfectly real, present in both opcode tables, correctly sized
and correctly built, and still do nothing - because the server registers it with
Handle_NULL. Two hundred and thirty-eight of AzerothCore's client opcodes are
registered that way: the number exists so the packet can be recognised and
discarded, usually because the opcode was replaced in a later expansion and the
name kept.

That is the quietest failure a request can have. Nothing is malformed, nothing
is logged, no size check fires and no layout check fires - the packet leaves and
the world does not change. CMSG_CHANGEPLAYER_DIFFICULTY was the case that named
this: every difficulty change this client sent, including the ones its own
/difficulty command sends, went to Handle_NULL. The opcodes the server reads are
MSG_SET_DUNGEON_DIFFICULTY and MSG_SET_RAID_DIFFICULTY, and both were already in
the table beside it.

WHAT IT LOOKS FOR

Every opcode this client *constructs a packet with* - `network::Packet p(
wireOpcode(Opcode::X))` - checked against the handler the server registers for
that name.

Construction, not mention. The first cut matched any `wireOpcode(Opcode::X)`
and so counted the incoming-handler tables and the `wireOp == wireOpcode(...)`
comparisons in the movement code, which are the opposite of a send: ten of the
sixteen it first reported were opcodes this client only ever *receives*.

THE OTHER DIRECTION

The same table read backwards: opcodes the server *sends* that this client
names and never handles. A dropped reply is as quiet as a discarded request -
the packet arrives, the dispatch table has no entry, and nothing anywhere says
so.

Most of what that turns up is obsolete, and the table cannot tell you which.
What can is whether the server ever *builds* one: an opcode that appears only in
Opcodes.cpp and nowhere else in the server source is a name kept for
recognition, and one that appears in a WorldPacket constructor somewhere is a
message this client is actually being sent.

Three of these were live on 2026-08-06. AzerothCore answers a force-move ack by
broadcasting the mover's movement info to everyone else, and MSG_MOVE_HOVER,
MSG_MOVE_FEATHER_FALL and MSG_MOVE_WATER_WALK were missing from the relay list
that MSG_MOVE_GRAVITY_CHNG and MSG_MOVE_UPDATE_CAN_FLY were already in - same
body, same handler, nothing to tell them apart. Another player starting to
hover stayed where they were last seen until their next heartbeat.

WHAT IT CANNOT SEE

An opcode the server handles but ignores in some *condition* - a guard inside
the handler rather than at the table. This is about the table only.

Nor does it know what the client should send instead. Handle_NULL says the
request is dead, not what replaced it; that is a question for the handler list
and usually has an obvious neighbour.

On the receiving arm, it cannot tell a skip handler from a real one: both are
entries in the dispatch table. A deliberately dropped packet is a decision on
record and reads here as handled, which is the right answer for this check.
"""
import argparse
import re
import sys
from pathlib import Path
import os

ROOT = Path(__file__).resolve().parent.parent

#: Handlers that mean "recognised, then dropped".
DEAD = {"Handle_NULL", "Handle_ServerSide", "Handle_Deprecated"}


def server_handlers(path):
    """opcode name -> the handler the server registers for it."""
    text = Path(path).read_text(errors="ignore")
    out = {}
    for m in re.finditer(
            r"DEFINE_(?:SERVER_OPCODE_)?HANDLER\(\s*(\w+)\s*,[^,]+,[^,]+,"
            r"\s*&WorldSession::(\w+)", text):
        out[m.group(1)] = m.group(2)
    # The server-side form has no handler column; those are replies, not
    # requests, and are not this check's business.
    for m in re.finditer(r"DEFINE_SERVER_OPCODE_HANDLER\(\s*(\w+)", text):
        out.setdefault(m.group(1), "server")
    return out


def client_sends():
    """opcode name -> the files that build a packet for it."""
    out = {}
    for path in (ROOT / "src").rglob("*.cpp"):
        text = path.read_text(errors="ignore")
        for m in re.finditer(
                r"Packet\s+\w+\s*\(\s*wireOpcode\(\s*Opcode::(\w+)\s*\)", text):
            name = m.group(1)
            if name.startswith("SMSG_"):
                continue
            out.setdefault(name, set()).add(path.name)
    return out


#: Server messages this client names, never handles, and does not need to -
#: each checked against whether the server builds one anywhere.
EXPECTED_UNHANDLED = {
    # Sent one line before SMSG_DESTROY_OBJECT with the same guid and only
    # inside an arena. The unit is removed by that one either way; the real
    # client uses this to tell an opponent who died from one who went out of
    # range, and nothing here draws that distinction. Skipped on purpose in
    # game_handler_packets, so it no longer reaches this arm.
    "SMSG_ARENA_UNIT_DESTROYED": "redundant with SMSG_DESTROY_OBJECT",
}


def client_handles():
    """Opcode names this client has any dispatch entry for."""
    out = set()
    for path in (ROOT / "src/game").rglob("*.cpp"):
        out |= set(re.findall(r"Opcode::(SMSG_\w+|MSG_\w+)",
                              path.read_text(errors="ignore")))
    return out


def server_builds(server_path):
    """Opcode names the server constructs a packet for, anywhere but the table."""
    root = Path(server_path).parent.parent.parent   # .../src/server/game
    text = []
    for p in root.rglob("*.cpp"):
        if p.name == "Opcodes.cpp":
            continue
        text.append(p.read_text(errors="ignore"))
    return " ".join(text)


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
        "src/server/game/Server/Protocol/Opcodes.cpp"))
    args = ap.parse_args()
    if not Path(args.server).is_file():
        print(f"server opcode table not found at {args.server or '<unset>'} - set WOWEE_SERVER_SRC to an "
              "AzerothCore checkout, or pass --server")
        return 0

    handlers = server_handlers(args.server)
    sends = client_sends()

    rows, unknown = [], []
    for name, files in sorted(sends.items()):
        handler = handlers.get(name)
        if handler is None:
            unknown.append(name)
        elif handler in DEAD:
            rows.append((name, handler, " ".join(sorted(files))))

    print(f"{len(sends)} opcode(s) this client sends, {len(handlers)} in the "
          f"server's table, {len(unknown)} it does not list\n")
    print(f"{len(rows)} that the server reads and discards:\n")
    for name, handler, files in rows:
        print(f"  {name:44} {handler:16} {files}")
    if not rows:
        print("  (none)")
    if unknown:
        print(f"\nnot in the server's table, so not judged: "
              f"{', '.join(unknown[:14])}"
              f"{' ...' if len(unknown) > 14 else ''}")

    # ...and the same table read backwards.
    known = set(re.findall(r"\b(SMSG_\w+|MSG_\w+)\b",
                           (ROOT / "include/game/opcode_enum_generated.inc")
                           .read_text(errors="ignore")))
    handled = client_handles()
    from_server = {n for n, h in handlers.items() if h == "server"}
    unhandled = sorted(n for n in from_server
                       if n in known and n not in handled
                       and n not in EXPECTED_UNHANDLED)
    built = server_builds(args.server)
    live = [n for n in unhandled if n in built]

    print(f"\n{len(unhandled)} server message(s) named here and never handled")
    print(f"{len(live)} that the server actually builds:\n")
    for name in live:
        print(f"  {name}")
    if not live:
        print("  (none)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
