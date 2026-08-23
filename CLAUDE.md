# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

WoWee is a native C++20 World of Warcraft client with a custom Vulkan renderer. One
tree targets **Vanilla 1.12**, **TBC 2.4.3** and **WotLK 3.3.5a** through per-expansion
profiles (plus Turtle WoW 1.18), on Linux/macOS/Windows (x86-64 and ARM64) and Android
arm64. It ships no Blizzard assets: the user extracts their own client into a loose-file
`Data/` tree.

Read [`docs/architecture.md`](docs/architecture.md) before touching an unfamiliar
subsystem — it is current and detailed.

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
./build/bin/wowee
```

- `./rebuild.sh` for a full clean rebuild (~10 min); `./build.sh` for incremental. Both
  fetch the optional AMD FSR2 SDK into `extern/`. Plain CMake works without them.
- Always clone/checkout with submodules: `git submodule update --init --recursive`.
- `WOWEE_WARNINGS_AS_ERRORS` defaults to **ON** — a new warning breaks the build.
- Platform dependency lists (Homebrew / apt / pacman / MSYS2 / vcpkg) live in
  [`BUILD_INSTRUCTIONS.md`](BUILD_INSTRUCTIONS.md). On macOS the build needs
  `PKG_CONFIG_PATH` and `CMAKE_PREFIX_PATH` pointed at Homebrew.
- Notable options: `WOWEE_BUILD_TESTS` (ON), `WOWEE_ENABLE_ASAN`, `WOWEE_ENABLE_TRACY`,
  `WOWEE_BUILD_EDITOR`, `WOWEE_BUILD_FRAMEXML_RUN`, `WOWEE_ENABLE_AMD_FSR2`.
- `./clean.sh --build|--assets|--cache|--user|--all` resets generated state. User config
  lives in `~/.wowee/`.

Secondary targets: `wowee_editor`, `dbc_to_csv`, `blp_convert`, `auth_probe`,
`auth_login_probe`, `asset_extract` (needs StormLib), `framexml_run`.

## Test and lint

`./test.sh` is the single entry point; everything exits non-zero on failure.

```bash
./test.sh              # unit tests (Release) + clang-tidy
./test.sh --test       # unit tests only
./test.sh --lint       # clang-tidy only    (FIX=1 ./test.sh --lint applies fixes)
./test.sh --asan       # tests under ASAN+UBSan, needs a build_asan/ directory
```

Single suites go through ctest or the binary directly:

```bash
cd build && ctest --output-on-failure -R srp
./build/bin/test_srp "[authentication]"
```

ASAN build directory (kept separate so sanitizer flags never reach the Release binary):

```bash
cmake -B build_asan -DCMAKE_BUILD_TYPE=Debug -DWOWEE_ENABLE_ASAN=ON -DWOWEE_BUILD_TESTS=ON
cmake --build build_asan --parallel
```

Adding a test: create `tests/test_<name>.cpp` (Catch2 v3 amalgamated, in `extern/catch2/`),
register it in `tests/CMakeLists.txt`, and **call `register_test_target(test_<name>)`** —
without it the test silently misses ASAN/UBSan instrumentation. Full guide in
[`TESTING.md`](TESTING.md).

## The sweeps (`tools/*.py`)

Over 120 Python static-analysis sweeps live in `tools/`. They are not optional
tooling — they encode faults this codebase produces repeatedly and that raise nothing,
log nothing and fail no test (a panel drawn twice, a binding answering short, a name in a
manifest resolving to nothing).

- `tools/sweep_guard.py` runs the fast ones (~1 min) and is wired into ctest as the
  `sweep_guard` test. Each entry pins a **ceiling, not a snapshot**: lowering a count
  passes, and the ceiling is meant to be lowered with it. It is a ratchet.
- `tools/tools_run_check.py` runs every sweep `sweep_guard` does not, and fails on any
  that cannot execute at all.
- Sweeps that read extracted game data (`Data/interface`, `Data/expansions`) or
  `build/bin/framexml_run` skip themselves when the input is absent, so a fresh checkout
  reports fewer sweeps than a populated one. That is a skip, not a pass.
- When writing a sweep that parses FrameXML, import `tools/framexml_source.py`
  (`without_comments` for reading names out of strings, `without_comments_or_strings` for
  reading syntax) and `tools/framexml_provides.py` for "does the client answer this name" —
  hand-rolled copies of those two rules are this repo's dominant bug shape.

## Architecture

### Packet flow and expansion abstraction

`GameHandler` (`include/game/game_handler.hpp`, `src/game/game_handler.cpp`) is the
central game state. It owns domain handlers by composition — `EntityController`,
`MovementHandler`, `CombatHandler`, `SpellHandler`, `InventoryHandler`, `QuestHandler`,
`SocialHandler`, `ChatHandler`, `WardenHandler` — and receives dependencies through a
`GameServices` struct. No singletons.

Adding a server packet handler:

1. `struct FooData` for the parsed fields.
2. `void GameHandler::handleFoo(network::Packet& packet)` parses into it.
3. Register in the dispatch table (`src/game/game_handler_packets.cpp`):
   `registerHandler(LogicalOpcode::SMSG_FOO, &GameHandler::handleFoo)`.
   Variants: `registerWorldHandler` (requires `isInWorld()`), `registerSkipHandler`,
   `registerErrorHandler`.

Handlers speak `LogicalOpcode`, never wire values. `OpcodeTable` maps logical → wire per
expansion from `Data/expansions/<id>/opcodes.json`. Branch on behaviour with the helpers
in `game_utils.hpp`: `isActiveExpansion("classic"|"tbc"|"wotlk")`,
`isClassicLikeExpansion()` (classic or turtle), `isPreWotlk()`.

### Expansion config files — the traps

`Data/expansions/<id>/` holds `expansion.json`, `opcodes.json`, `update_fields.json`,
`dbc_layouts.json`.

- `opcodes.json` supports `_extends` / `_remove` (see `loadOpcodeJsonRecursive` in
  `src/game/opcode_table.cpp`); turtle inherits from classic this way.
- `update_fields.json` does **not** support `_extends`. The classic and turtle files must
  stay byte-identical by hand — change both together. Authoritative indices: vmangos
  `UpdateFields_1_12_1.h` / `_2_4_3.h`, AzerothCore/TrinityCore `UpdateFields.h` for 3.3.5a.
- `wardenRsaModulus` in `expansion.json` is refused unless it is exactly 512 hex chars;
  extract a server's own key with `extract_warden_rsa.py`.

### The interface is FrameXML's

As of v3.1.8 the UI is Blizzard's own FrameXML, loaded from extracted data and driven by
the client's Lua implementation. `src/addons/` provides the WoW Lua API — `lua_unit_api`,
`lua_spell_api`, `lua_inventory_api`, `lua_action_api`, etc. — plus bootstrap Lua that
lives **inside C++ string literals in `lua_engine.cpp`**. A name can therefore be answered
by a C++ binding table, an explicit `lua_setglobal`, a bootstrap `function Name(`, or a
quoted list in the bootstrap; a tool or a fix that only looks at the binding tables sees
between a third and a half of the truth.

What remains client-drawn (ImGui, `src/ui/`) is a handful of surfaces rendered *into*
FrameXML-owned frames: minimap, world map, zone text, taxi picker, settings panel.
`src/ui/chat/` and `src/rendering/world_map/` are modular subsystems in their own right.

`framexml_run` (`-DWOWEE_BUILD_FRAMEXML_RUN=ON`) is the client minus `main.cpp`: a
headless runner that answers Lua for real, used by the heavier sweeps.

```bash
cmake -S . -B build -DWOWEE_BUILD_FRAMEXML_RUN=ON
cmake --build build --target framexml_run
./build/bin/framexml_run Data 'ToggleGameMenu()'
```

### Assets

No MPQ is read at runtime. `extract_assets.sh|ps1 <WoW/Data> <classic|turtle|tbc|wotlk>`
produces a loose-file tree with a `manifest.json` per expansion under `Data/expansions/`;
`AssetManager` indexes it, with overlay manifests for cross-expansion dedup. Point
elsewhere with `WOW_DATA_PATH`.

### Threading

Main thread runs events, game update and rendering. Terrain chunks stream via
`std::async`, network recv is non-blocking on the main thread with a per-frame budget,
normal maps are generated on background CPU threads behind a mutexed result queue, and a
second Vulkan queue handles GPU uploads. See [`docs/threading.md`](docs/threading.md).

## Conventions

- C++20, `#pragma once`, `constexpr` over `static const`, `[[nodiscard]]` where ignoring
  the result is a bug.
- Namespaces: `wowee::game`, `wowee::rendering`, `wowee::rendering::world_map`,
  `wowee::ui`, `wowee::ui::chat`, `wowee::math`, `wowee::core`, `wowee::network`.
- Commits: imperative mood with a lowercase prefix — either conventional
  (`feat:`, `fix:`, `perf:`, `refactor:`, `docs:`, `ci:`, `build:`, `tools:`) or a
  subsystem (`render:`, `framexml:`, `warden:`, `items:`, `vendor:`). One logical change
  per commit. Branch from `master`.
- Gameplay-affecting changes are expected to be tested by hand against a real 3.3.5a
  server (AzerothCore/ChromieCraft) — the unit tests do not cover the wire.
- Useful runtime overrides: `WOWEE_REALM_HOST_OVERRIDE` (MaNGOS advertising an
  unreachable world host), `WOWEE_TURTLE_AUTH_BUILD` (defaults 7272; older servers 7234).

## CI

`.github/workflows/` — `build.yml` (Linux, macOS, Windows x64 + ARM), `release.yml`
(adds Android and macOS notarization), `security.yml` (CodeQL, Semgrep, sanitizer build),
`pages.yml`. macOS release DMGs are Developer ID signed, notarized and stapled; the
credential contract is in [`docs/macos-distribution.md`](docs/macos-distribution.md).
