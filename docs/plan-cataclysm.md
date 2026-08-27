# Cataclysm Support Plan

**Status:** steps 1 to 3 in progress. A 4.3.4.15595 core runs at `/media/k/vbox/wowee-cata`
(The-Cataclysm-Preservation-Project, since upstream deleted its 4.3.4 branch). The bit primitive is
in `network::Packet`, the trap in section 5 is closed, and the 109 per-opcode movement layouts are
derived from that core and executed by a table-driven reader. Update-object is next, and the gate in
section 6 has not been reached.
**Target:** 4.3.4 build 15595, which is where the private-server cores settled.
**Constraint set by the project:** the same one every other expansion works under. The player
extracts from their own client, the release ships no data, and the wire format is derived from a
running core rather than from memory.

Measured 2026-08-27 against the tree at `eb0f0386b`. Numbers here are from this working copy.

---

## 1. What the multi-expansion layer already gives it

A fair amount, and more than a reading of the expansion list suggests. The 2026 multi-expansion
work moved the parts that differ per expansion out of code and into data, and a fourth expansion
mostly fills those files in.

| Thing | Evidence | Consequence for Cata |
|---|---|---|
| Opcode numbering | `include/game/opcode_table.hpp`: a `LogicalOpcode` enum mapped to wire values loaded from `Data/expansions/<id>/opcodes.json`, with `_extends`/`_remove` deltas | 4.3.4 renumbered every opcode. That is a data file, not a code change. wotlk has 1306 mappings, tbc 1054, classic 818 |
| Update field indices | `Data/expansions/<id>/update_fields.json`, 79 keys | Data file |
| DBC column indices | `Data/expansions/<id>/dbc_layouts.json`, 40 entries, read by `src/pipeline/dbc_layout.cpp` | Data file, though every entry has to be re-derived |
| Parser dispatch | `include/game/packet_parsers.hpp`: a 47-method virtual interface whose base class *is* WotLK; Classic and TBC override only the differences | A `CataPacketParsers` has somewhere to live from day one |
| Interface convention | `src/addons/addon_manager.cpp:600` splits on the `## Interface:` number at 30000 | Cata's 40300 takes the modern branch with no work |
| Archive format | 4.3.4 is still MPQ. CASC did not arrive until Warlords | `tools/asset_extract` and StormLib keep working |
| Texture format | BLP1 and BLP2 both handled, `src/pipeline/blp_loader.cpp:136-140` | Unchanged |
| Model format | `src/pipeline/m2_loader.cpp:821` requires `MD20`, which 4.3.4 still is. The chunked `MD21` is Legion | Unchanged |
| World model format | `src/pipeline/wmo_loader.cpp` parses root plus group files, v17 throughout | Unchanged |

The header comment at `include/game/expansion_profile.hpp:16` already lists `"cata"` as an example
id, and `src/game/expansion_profile.cpp:124` already sorts for it. Nothing behind those exists.

---

## 2. The one thing that decides it

**Cataclysm replaced packed GUIDs with bit-streamed ones, in exactly the packets that carry the
game.** A 4.x movement or update-object packet writes a GUID as a bitmask of which bytes are
non-zero followed by those bytes XOR'd and emitted in an order that **differs per opcode**. It is
not a variant of the packed GUID; it is a different marshalling with no byte alignment.

What that meets in this codebase:

| Measure | Value |
|---|---|
| Bit-level accessors on `network::Packet` | none. `include/network/packet.hpp` is byte-oriented throughout |
| `readPackedGuid` call sites | 149 |
| `writePackedGuid` call sites | 32 |
| Files touching packed GUIDs | 17 |

So this is not "write a `CataPacketParsers` that overrides some methods". It is a new primitive in
the packet layer, a per-opcode bit and byte order table beside it, and rework of every movement,
update-object, spell and combat path that reads a GUID. On its own it is larger than Classic and
TBC support put together, and every other item in section 3 is worthless until it lands.

Two adjacent things are unknown rather than large, and have to be read off a running 4.3.4 core
before they can be sized at all:

- **Header framing.** `src/network/world_socket.cpp` frames 3.3.5a headers. Whether 4.3.4's differ
  in width or direction is not something to take from memory.
- **Session crypt.** `src/auth/` implements 3.3.5a's RC4 seeding. The 4.x constants and derivation
  need reading, not recalling.

---

## 3. The rest, in order of size

None of this is interesting, and all of it is real.

| Work | Where it lands | Note |
|---|---|---|
| Split ADTs: `_tex0`, `_obj1`, `_tex1` | `src/rendering/terrain_manager.cpp:426`, `src/core/world_loader.cpp:1337` | Only `_obj0` is read today, and through the same monolithic `ADTLoader::load`. Cata splits terrain properly, so the loader has to learn that a tile is several files with different chunk sets |
| New ADT chunks | `src/pipeline/adt_loader.cpp:61-80` (top level), `:363-408` (MCNK sub-chunks) | Reads MTEX, MMDX, MWMO, MDDF, MODF, MH2O, MCNK, and MCVT/MCNR/MCLY/MCAL/MCLQ under it. MCCV, MTXP and MAMP are absent |
| Re-derive 40 DBC layouts for 4.3.4 columns | `Data/expansions/cata/dbc_layouts.json` | Tedious and unavoidable. See `dbc_layout_traps`: a named index is not a checked one, and a wrong column reads as zero forever |
| A WDB2 reader | `src/pipeline/dbc_loader.cpp:61` rejects anything whose magic is not `WDBC` | 4.3.4 moved `Item` and `Item-sparse` to `.db2`. WDB2 is a fixed-width format with a larger header, so this is small |
| `update_fields.json` for 4.3.4 | data only | 79 keys to re-derive |
| Extractor archive list | `tools/asset_extract/extractor.cpp:487` | 4.3.4's patch chain is not the `patch-N` scheme the other three use. The base and locale archives and their update chain have to be enumerated and ordered |
| Cata FrameXML against the Lua API | `src/addons/`, 41,722 lines, 1,095 distinct bound names over 1,139 registrations | The long tail. The bindings are shaped for 3.3.5a's FrameXML and 4.x both adds and changes calls |

---

## 4. What is outside this repo

Two constraints that no amount of work here removes.

**There is no oracle.** Every wire fact in this client was checked against AzerothCore, which is
3.3.5a only. `docs/server-setup.md` is written end to end around 3.3.5a, the only core checked out
locally is `/home/k/azerothcore-wotlk`, and `oracle_enum_sweep` plus the wire-side sweeps all point
at it. Cata means TrinityCore's 4.3.4 branch, which is considerably less maintained, and it means
re-pointing or duplicating every sweep that currently proves the wire. See `wire_side_sweeps`: a
zero from a canary-less sweep means nothing, and a sweep aimed at the wrong core is exactly that.

**The world is a different world.** 4.3.4's Azeroth is the sundered one. It shares no terrain with
the three expansions already supported, so the asset cost is a full additional extraction rather
than an overlay. For scale, the current expansion trees are tbc 7.2 GB, turtle 5.5 GB, wotlk 3.0 GB
and classic 51 MB.

---

## 5. A trap already in the tree, now closed

`createPacketParsers` in `include/game/packet_parsers.hpp` used to end with a bare
`return std::make_unique<WotlkPacketParsers>();`, so any expansion id it did not recognise got
WotLK's parsers. Dropping a `Data/expansions/cata/` directory in would have come up, logged in, and
misparsed in the paths that matter rather than failing where the gap is.

It now lists `wotlk` explicitly and answers null otherwise. The header stays free of a logger,
because ten standalone tests include it, so the decision is at the call site in
`src/core/application.cpp`: null is reported and then falls back to WotLK anyway, which keeps a
hand-written profile working while saying what it is running on.

---

## 6. Order of work

The point of this ordering is to reach the decision early and cheaply.

1. **Stand up a 4.3.4 core.** Nothing below can be checked without one. If this step does not
   finish, neither does the port.
2. **Bit reader and writer on `network::Packet`.** Done. `writeBit`/`readBit`, `writeBits`/
   `readBits`, and GUID mask and byte passes that take the order as a parameter, in
   `include/network/packet.hpp` and `src/network/packet.cpp`, covered by `tests/test_bit_packet.cpp`.
   The per-opcode order tables are deliberately *not* here: they are the part that has to be read
   off a core rather than recalled, and the helpers take them as arguments so that stays true.
3. **Movement and update-object only.** A `CataPacketParsers` that logs in, receives the player,
   and moves. Everything else may fail.

   Movement is read. The core keeps each opcode's layout as an array of
   `MovementStatusElements` in `MovementStructures.cpp`, and there are 109 of them with no pattern
   between: `MSG_MOVE_START_FORWARD` writes position Y, Z, X then guid mask bits 5, 2, 0, while
   `MSG_MOVE_HEARTBEAT` writes Z, X, Y then pitch, timestamp and fall bits. `tools/derive_cata_movement.py`
   parses them into `Data/expansions/cata/movement_sequences.json` and `src/game/cata_movement.cpp`
   executes that, which is how the core does it and for the same reason.

   Two things came off the core that no amount of care would have produced from memory. The presence
   bits for movement flags, flags2, timestamp, orientation, pitch and spline elevation are
   **inverted**, so a set bit means the field is absent, while the transport, fall and vehicle bits
   are not. And eleven layouts carry `MSEFlushBits`, all of them SMSG, which is why the core's own
   reader has no case for it: the server writes those and never reads one.
4. **Stop and look.** This is the gate. If steps 2 and 3 came in near the estimate the rest is
   ordinary work; if they did not, nothing further is worth starting.
5. Asset extraction: archive list, split ADTs, the new MCNK sub-chunks, WDB2.
6. `dbc_layouts.json` and `update_fields.json`, derived against the core rather than assumed.
7. The remaining parser overrides, by traffic volume.
8. FrameXML, last, because it is the only part that degrades gracefully.

---

## 7. Calibration

The commit that introduced the whole multi-expansion layer *and* TBC was 51 files and 5,258
insertions. That is the number to distrust. Since then `src/game/packet_parsers_tbc.cpp` has been
touched 106 times and `Data/expansions/tbc/` 77 times.

The landing is a fraction of the cost, and TBC is a far smaller delta from WotLK than Cata is:
TBC's parser is 1,458 lines of overrides against a WotLK base that mostly already fits, and its
wire reads with the same primitives. Cata's does not.

---

## 8. Decisions taken, so they are not re-litigated

- **Assets stay player-extracted.** This is settled for every platform already, most recently in
  `docs/plan-android.md`: assets are never bundled and never downloaded by us.
- **No CASC, and no CDN fetching.** Answered at length on issue #130. 4.3.4 is MPQ anyway, so this
  question does not arise for Cata even if it is reopened for something else.
- **4.3.4 only.** Not 4.0.6, 4.1 or 4.2. One build, the one the cores implement, in keeping with
  1.12.1, 2.4.3 and 3.3.5.
- **The Legion overlay is not a precedent.** `Data/expansions/wotlk/legion` is 3.0 GB of later
  models substituted into an earlier client through the overlay mechanism at
  `src/pipeline/asset_manager.cpp:389`. Assets and protocol are separable, and nothing about that
  overlay working says anything about a later wire format.
- **The universal-client rule still holds.** One build serves every expansion. A Cata profile does
  not get a build flag, a second binary, or a shared-wire parse gated to one expansion.
