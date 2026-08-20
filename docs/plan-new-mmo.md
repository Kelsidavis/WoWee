# Plan: New D&D-Inspired Mobile MMO

## Vision

A simplified, phone-first MMO inspired by Dungeons & Dragons. Built on the WoWee
engine (Vulkan renderer, SDL2 window, audio, math, core loop) with WoW-specific
code stripped and replaced by new game systems, a new asset pipeline, and a
new network protocol.

## What we keep from WoWee (the "engine")

| Layer | Path | LOC | Notes |
|-------|------|-----|-------|
| Core loop | `src/core/` | ~16k | Application, Window, Input, Config, Logger, Env |
| Math | `src/math/` | ~0.2k | glm wrapper |
| Audio | `src/audio/` | ~5k | miniaudio-based playback |
| Rendering | `src/rendering/` | ~57k | Vulkan: terrain, models, lighting, sky, water, particles |
| Pipeline | `src/pipeline/` | ~39k | Asset loading, mesh/texture/skeleton management |
| UI | `src/ui/` | ~42k | ImGui-based (will be heavily reworked for touch) |
| Network | `src/network/` | ~1.3k | Transport layer (keep TCP/UDP framing, drop WoW protocol) |

**Total engine: ~160k LOC** (before trimming WoW-specific paths within these dirs)

## What we throw away

| Layer | Path | LOC | Why |
|-------|------|-----|-----|
| Game logic | `src/game/` | ~60k | WoW-specific: combat, classes, items, spells, quests |
| Addons | `src/addons/` | ~40k | WoW Lua addon system |
| Auth | `src/auth/` | ~2k | SRP6 / WoW login protocol |
| WoW parsers | within pipeline | ~15k? | BLP, ADT, M2, WMO, DBC format readers |
| Warden | within game | ? | Blizzard anti-cheat emulation |

## What we build new

1. **Touch input** — virtual joystick, tap-to-move, pinch-to-zoom camera, touch UI
2. **Asset pipeline** — glTF 2.0 models, PNG/KTX2 textures, standard audio (OGG/WAV)
3. **Network protocol** — simple binary protocol (not WoW's), server-authoritative
4. **D&D game systems** — classes (Fighter, Wizard, Rogue, Cleric…), ability system,
   dice-based mechanics (d20 checks), party/group, dungeon instances
5. **Mobile UI** — touch-first, thumb-zone layout, simplified HUD
6. **Mobile rendering** — lower draw calls, simpler shaders, battery-aware quality tiers

## Phases

### Phase 1: Engine extraction (this session)
- Create new project directory (name TBD)
- Copy engine layers (core, math, audio, rendering, pipeline, network, ui)
- Strip WoW-specific code paths (BLP/ADT/M2/WMO/DBC parsers, WoW protocol)
- Get it compiling as a standalone "engine" with a minimal test scene
- **Deliverable:** builds and runs a blank Vulkan window with a spinning cube

### Phase 2: Mobile input
- Touch input layer (SDL2 touch events)
- Virtual joystick (left thumb)
- Tap-to-move / tap-to-interact (right thumb)
- Pinch-to-zoom / two-finger camera rotate
- **Deliverable:** on-device (or emulator) touch controls working

### Phase 3: Asset pipeline
- glTF 2.0 model loader (mesh, skeleton, animation, materials)
- Texture loader (PNG → GPU, KTX2 optional)
- Audio loader (OGG/WAV)
- Asset manifest (JSON: what to load, where it goes)
- **Deliverable:** load and render a glTF character with animation

### Phase 4: World rendering
- Terrain (heightmap-based, not ADT)
- Lighting (directional + ambient, optional point lights)
- Sky (procedural or cubemap)
- Basic water
- **Deliverable:** a small test world renders correctly

### Phase 5: Character & camera
- Skeletal animation playback
- Third-person camera (follow, orbit, zoom)
- Player movement (walk/run, grounded)
- **Deliverable:** walk a character around the test world

### Phase 6: Network
- Simple TCP protocol (length-prefixed JSON or flatbuffers)
- Client: send input, receive state
- Server: authoritative position, broadcast to nearby players
- **Deliverable:** two clients see each other move

### Phase 7: D&D game systems
- Class/ability framework
- Dice system (d4/d6/d8/d10/d12/d20)
- Combat (turn-based or real-time with cooldowns — TBD)
- Party/group
- **Deliverable:** two players fight a dummy target

### Phase 8: Dungeon instances
- Instance loading (separate world per group)
- Encounter scripting (simple event system)
- Loot/loot table
- **Deliverable:** a 3-room dungeon with a boss

### Phase 9: UI
- Touch-first HUD (health, mana, ability bar)
- Character sheet
- Inventory
- Chat
- **Deliverable:** playable UI on phone

### Phase 10: Polish & ship
- Quality tiers (low/med/high for phones)
- Loading screens
- Settings
- **Deliverable:** testable build

## Open questions (need your input)

1. **Project name?** (repo name, binary name)
2. **Where does it live?** New directory alongside WoWee? New repo?
3. **Combat model?** Real-time (WoW-like) or turn-based (D&D-like) or hybrid?
4. **Asset format for models?** glTF 2.0 is the obvious choice — confirm?
5. **Server language?** C++ (same as client) or something else (Go, Rust, Node)?
6. **How "simplified" for phones?** (e.g., no complex particle effects? lower res? simplified UI?)

## API facts confirmed

- WoWee uses C++20, CMake 3.15+, SDL2, Vulkan, ImGui, miniaudio, glm, nlohmann/json
- Build: `cmake -S . -B build && cmake --build build`
- The rendering layer is tightly coupled to the pipeline (asset manager) — extraction
  will need to stub or replace the WoW format parsers
- Audio uses miniaudio (single-header, easy to keep)
- Network is thin (~1.3k LOC) — mostly transport, easy to repurpose
