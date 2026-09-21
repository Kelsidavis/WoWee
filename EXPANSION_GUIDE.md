# Multi-Expansion Architecture Guide

WoWee supports four World of Warcraft expansion profiles in a unified codebase using an expansion profile system. This guide explains how the multi-expansion support works.

## Supported Expansions

- **Vanilla (Classic) 1.12** - Original World of Warcraft
- **The Burning Crusade (TBC) 2.4.3** - First expansion
- **Wrath of the Lich King (WotLK) 3.3.5a** - Second expansion
- **Turtle WoW 1.18** - Custom Vanilla-based server with extended content

Each profile is a directory under `Data/expansions/` (`classic`, `tbc`, `wotlk`,
`turtle`). `Data/expansions/cata` carries no `expansion.json`, so Cataclysm is not
a server profile; a 4.3.4 client can still be read as an asset source by
`asset_extract --expansion cata` and the asset builder.

## Architecture Overview

The multi-expansion support is built on the **Expansion Profile** system:

1. **ExpansionProfile** (`include/game/expansion_profile.hpp`) - Metadata about each expansion
   - Read from `Data/expansions/<id>/expansion.json` by `ExpansionRegistry`
   - Defines version, build, protocol version, data path, races and classes,
     and optionally the realm's Warden signing key (`wardenRsaModulus`)

2. **Packet Parsers** (`include/game/packet_parsers.hpp`) - Expansion-specific message handling
   - `createPacketParsers(id)` picks `ClassicPacketParsers`, `TurtlePacketParsers`,
     `TbcPacketParsers` or `WotlkPacketParsers`
   - `packet_parsers_classic.cpp` - Vanilla 1.12 / Turtle WoW message parsing
   - `packet_parsers_tbc.cpp` - TBC 2.4.3 message parsing
   - Default (WotLK 3.3.5a) parsers in `game_handler.cpp` and domain handlers

3. **Update Fields** - Expansion-specific entity data layout
   - Loaded from `update_fields.json` in expansion data directory
   - Defines UNIT_END, OBJECT_END, field indices for stats/health/mana

4. **Opcodes and DBC layouts** - `opcodes.json` (which may `_extends` another
   profile's) and `dbc_layouts.json` in the same directory

`Application::loadExpansionTables()` (`src/core/application.cpp`) loads the
opcode table, update fields, packet parsers and DBC layouts together, at startup
and again whenever the active expansion changes.

## How to Use Different Expansions

### At Startup

`ExpansionRegistry::initialize()` scans `Data/expansions/*/expansion.json` and
picks the active profile from what is installed, not from the server:
1. WotLK, if WotLK assets are extracted
2. Otherwise the newest profile with extracted assets (a `manifest.json` in its directory)
3. Otherwise WotLK

Each saved server on the login screen remembers its expansion, so choosing
that server switches to it.

### Manual Selection

Choose the expansion under **more options** on the login screen. The selection
calls `ExpansionRegistry::setActive(id)` and then `Application::reloadExpansionData()`
(see `src/ui/auth_screen.cpp`), which loads the matching opcode table,
update-field layout, packet parsers and DBC layout for that expansion.

## Key Differences Between Expansions

### Packet Format Differences

#### SMSG_SPELL_COOLDOWN
- **Classic**: 12 bytes per entry (spellId + itemId + cooldown, no flags)
- **TBC/WotLK**: 8 bytes per entry (spellId + cooldown) + flags byte

#### SMSG_ACTION_BUTTONS
- **Classic**: 120 slots, no mode byte
- **TBC**: 132 slots, no mode byte
- **WotLK**: 144 slots + uint8 mode byte

#### SMSG_PARTY_MEMBER_STATS
- **Classic/TBC**: Full uint64 for guid, uint16 health
- **WotLK**: PackedGuid format, uint32 health

### Data Differences

- **Talent trees**: Different spell IDs and tree structure per expansion
- **Items**: Different ItemDisplayInfo entries
- **Spells**: Different base stats, cooldowns
- **Character textures**: Expansion-specific variants for races

## Adding Support for Another Expansion

1. Create `Data/expansions/<id>/expansion.json` with the profile (copy an
   existing one for the fields)
2. Add `opcodes.json`, `update_fields.json` and `dbc_layouts.json` beside it
3. Add a packet parser class (`packet_parsers_*.cpp`, declared in
   `include/game/packet_parsers.hpp`) for message variants and return it from
   `createPacketParsers()`. Without one the client logs an error and falls back
   to WotLK's parsers
4. Test realm connection and character loading

## Code Patterns

### Checking Current Expansion

```cpp
#include "game/game_utils.hpp"

// Shared helpers (defined in game_utils.hpp)
if (isActiveExpansion("tbc")) {
    // TBC-specific code
}

if (isClassicLikeExpansion()) {
    // Classic or Turtle WoW
}

if (isPreWotlk()) {
    // Classic, Turtle, or TBC (not WotLK)
}
```

### Expansion-Specific Packet Parsing

```cpp
// In packet_parsers_*.cpp, implement expansion-specific logic
bool TbcPacketParsers::parseXxx(network::Packet& packet, XxxData& data) {
    // Custom logic for this expansion's packet format
}
```

## Common Issues

### "Update fields mismatch" Error
- Ensure `update_fields.json` matches server's field layout
- Check OBJECT_END and UNIT_END values
- Verify field indices for your target expansion

### "Unknown packet" Warnings
- Expansion-specific opcodes may not be registered
- Check the opcode's entry in the profile's `opcodes.json` and its handler
  registration in `src/game/game_handler_packets.cpp`
- Verify expansion profile is active

### Packet Parsing Failures
- Each expansion has different struct layouts
- Always read data size first, then upfront validate
- Use size capping (e.g., max 100 items in list)

## References

- `include/game/expansion_profile.hpp` - Expansion metadata
- `include/game/packet_parsers.hpp` - Parser classes and `createPacketParsers()`
- `Data/expansions/<id>/` - Per-expansion `expansion.json`, `opcodes.json`, `update_fields.json`, `dbc_layouts.json`
- `include/game/game_utils.hpp` - `isActiveExpansion()`, `isClassicLikeExpansion()`, `isPreWotlk()`
- `src/game/packet_parsers_classic.cpp` / `packet_parsers_tbc.cpp` - Expansion-specific parsing
- `docs/status.md` - Current feature support
- `docs/` directory - Additional protocol documentation
