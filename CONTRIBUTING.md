# Contributing to Wowee

## Build Setup

See [BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md) for full platform-specific details.
The short version: CMake on Linux/macOS, MSYS2 on Windows. The desktop build needs
SDL3 and a Vulkan 1.3 driver.

```
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j
```

Warnings are errors in the client by default (`WOWEE_WARNINGS_AS_ERRORS=ON`).
The world editor and the headless `framexml_run` tool are off the default build;
see the build options table in BUILD_INSTRUCTIONS.md.

## Code Style

- **C++20**. Use `#pragma once` for include guards.
- Namespaces: `wowee::game`, `wowee::rendering`, `wowee::rendering::world_map`, `wowee::ui`, `wowee::ui::chat`, `wowee::addons`, `wowee::pipeline`, `wowee::audio`, `wowee::auth`, `wowee::math`, `wowee::core`, `wowee::network`.
- Commit messages are a short, lower-case subject prefixed with the area it
  touches, in imperative mood, e.g. `vulkan: record the shadow pass with dynamic rendering`
  or `ui: parse SimpleHTML item text and draw its images`.
- Prefer `constexpr` over `static const` for compile-time data.
- Mark functions whose return value should not be ignored with `[[nodiscard]]`.

## Pull Request Process

1. Branch from `master`.
2. Keep commits focused -- one logical change per commit.
3. Describe *what* changed and *why* in the PR description.
4. Ensure the project compiles cleanly (warnings are errors) and `./test.sh --test` passes before submitting.
5. Manual testing against a WoW 3.3.5a server (e.g. AzerothCore/ChromieCraft) is expected
   for gameplay-affecting changes.

## Architecture Overview

See [docs/architecture.md](docs/architecture.md) for the full picture. Key namespaces:

| Namespace | Responsibility |
|---|---|
| `wowee::game` | Game state, packet handling (`GameHandler` and its domain handlers), opcode dispatch, spline parsing |
| `wowee::rendering` | Vulkan renderer, M2/WMO/terrain, sky system |
| `wowee::rendering::world_map` | Modular world map (16 components: facade, compositor, layers, etc.) |
| `wowee::ui` | Login and settings screens, the FrameXML emitter and widget renderer, HUD (`GameScreen`) |
| `wowee::ui::chat` | Modular chat system (15+ components: commands, markup, macros, etc.) |
| `wowee::addons` | Lua engine, addon manager and the Lua API FrameXML calls |
| `wowee::pipeline` | Asset manager and the DBC, BLP, M2, WMO and ADT loaders |
| `wowee::audio` | Audio engine and sound managers |
| `wowee::auth` | SRP-6a login and the realm list |
| `wowee::math` | Reusable math modules (CatmullRomSpline) |
| `wowee::core` | Application, window, logging, config and data paths |
| `wowee::network` | Connection, `Packet` read/write API |

## Packet Handlers

The standard pattern for adding a new server packet handler:

1. Define a `struct FooData` holding the parsed fields.
2. Write `void GameHandler::handleFoo(network::Packet& packet)` to parse into `FooData`.
3. Register it in the dispatch table in `src/game/game_handler_packets.cpp`
   (`registerCoreOpcodes` or `registerRemainingOpcodes`):
   `registerHandler(Opcode::SMSG_FOO, &GameHandler::handleFoo)`.

`registerSkipHandler(Opcode::SMSG_FOO)` registers an opcode that is read and discarded.
Handlers that need more than a member call are written straight into `dispatchTable_`
as lambdas. The domain handlers (`ChatHandler`, `CombatHandler`, `EntityController`
and the rest) register their own opcodes in their `registerOpcodes(DispatchTable&)`.

## Testing

About two hundred ctest suites cover core systems, packets, assets, the interface,
animation, transport/spline, world map, and chat, alongside Python sweeps over the
interface. See [TESTING.md](TESTING.md) for the full guide. Run with `./test.sh --test`.
Manual testing against WoW 3.3.5a private servers (primarily ChromieCraft/AzerothCore)
is expected for gameplay-affecting changes.

## Expansion Config Files

`Data/expansions/<id>/` holds the per-expansion config (`expansion.json`,
`opcodes.json`, `update_fields.json`, `dbc_layouts.json`, plus the
`db/` CSV fallback and DBC overlay).

- `opcodes.json` supports `_extends` / `_remove` (see
  `src/game/opcode_table.cpp` `loadOpcodeJsonRecursive`). The turtle
  profile uses this to inherit from classic.
- `update_fields.json` does **not** currently support `_extends` - the
  classic and turtle files must be kept byte-identical by hand. Update
  both together when changing vanilla field indices.
- Authoritative source for vanilla 1.12 field indices: vmangos
  `UpdateFields_1_12_1.h`. For 2.4.3 and 3.3.5a, use vmangos
  `UpdateFields_2_4_3.h` and AzerothCore / TrinityCore `UpdateFields.h`
  respectively.
- `expansion.json` may name the realm's Warden signing key as
  `wardenRsaModulus`, 512 hex characters for the 256-byte RSA-2048 modulus.
  Omit it and the module's signature is checked against Blizzard's own key,
  which is right for a server running a genuine module. A server that builds
  its own module signs it with a key of its own, and its key can be pulled
  out of that server's client with `extract_warden_rsa.py`. The value is
  refused unless it is exactly 512 hex characters: a key wrong in one nibble
  fails verification the same way no key at all does.

## Key Files for New Contributors

| File / Directory | What it does |
|---|---|
| `include/game/game_handler.hpp` | Central game state and all packet handler declarations |
| `src/game/game_handler.cpp` | `GameHandler` construction and handler implementations |
| `src/game/game_handler_packets.cpp` | Packet dispatch registration |
| `include/network/packet.hpp` | `Packet` class -- the read/write API every handler uses |
| `include/ui/game_screen.hpp` | Main gameplay UI screen (ImGui) |
| `src/addons/` | Lua engine and the API FrameXML runs against |
| `src/ui/framexml_emitter.cpp` | Turns FrameXML's XML into the Lua that builds its frames |
| `src/ui/settings_schema.cpp` | Every client setting, from which the options pages are generated |
| `src/ui/chat/` | Modular chat system (commands, markup, macros, tab completion) |
| `src/rendering/world_map/` | Modular world map (facade, compositor, layers, coordinate projection) |
| `src/math/spline.cpp` | Reusable CatmullRomSpline math |
| `src/game/spline_packet.cpp` | Unified spline packet parsing for all expansions |
| `src/rendering/m2_renderer.cpp` | M2 model loading and rendering |
| `docs/architecture.md` | High-level system architecture reference |
