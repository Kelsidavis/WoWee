# WoWee - World Of Warcraft Engine Experiment

<p align="center">
  <img src="assets/Wowee.png" alt="Wowee Logo" width="240" />
</p>

A native C++ World of Warcraft client with a custom OpenGL renderer.

[![Sponsor](https://img.shields.io/github/sponsors/Kelsidavis?label=Sponsor&logo=GitHub)](https://github.com/sponsors/Kelsidavis)

[![Watch the video](https://img.youtube.com/vi/Pd9JuYYxu0o/maxresdefault.jpg)](https://youtu.be/Pd9JuYYxu0o)

[![Watch the video](https://img.youtube.com/vi/J4NXegzqWSQ/maxresdefault.jpg)](https://youtu.be/J4NXegzqWSQ)

Compatible with **Vanilla (Classic) 1.12 + TBC 2.4.3 + WotLK 3.3.5a**, including Turtle WoW (1.17). All three expansions are broadly functional with roughly even support.

> **Legal Disclaimer**: This is an educational/research project. It does not include any Blizzard Entertainment assets, data files, or proprietary code. World of Warcraft and all related assets are the property of Blizzard Entertainment, Inc. This project is not affiliated with or endorsed by Blizzard Entertainment. Users are responsible for supplying their own legally obtained game data files and for ensuring compliance with all applicable laws in their jurisdiction.

## Status & Direction (2026-02-18)

- **Compatibility**: **Vanilla (Classic) 1.12 + TBC 2.4.3 + WotLK 3.3.5a** are all broadly supported via expansion profiles and per-expansion packet parsers (`src/game/packet_parsers_classic.cpp`, `src/game/packet_parsers_tbc.cpp`). Turtle WoW (1.17) is also supported. All three expansions are roughly on par — no single one is significantly more complete than the others.
- **Tested against**: AzerothCore, TrinityCore, Turtle WoW, and ChromieCraft.
- **Current focus**: protocol correctness across server variants, visual accuracy (M2/WMO edge cases, equipment textures), and multi-expansion coverage.
- **Warden**: Full module execution via Unicorn Engine CPU emulation. Decrypts (RC4→RSA→zlib), parses and relocates the PE module, executes via x86 emulation with Windows API interception. Module cache at `~/.local/share/wowee/warden_cache/`.

## Features

### Rendering Engine
- **Terrain** -- Multi-tile streaming with async loading, texture splatting (4 layers), frustum culling
- **Water** -- Animated surfaces, reflections, refractions, Fresnel effect
- **Sky System** -- WoW-accurate DBC-driven lighting with skybox authority
  - **Skybox** -- Camera-locked celestial sphere (M2 model support, gradient fallback)
  - **Celestial Bodies** -- Sun (lighting-driven), White Lady + Blue Child (Azeroth's two moons)
  - **Moon Phases** -- Game time-driven deterministic phases when server time is available (fallback: local cycling for development)
  - **Stars** -- Baked into skybox assets (procedural fallback for development/debug only)
- **Atmosphere** -- Procedural clouds (FBM noise), lens flare with chromatic aberration, cloud/fog star occlusion
- **Weather** -- Rain and snow particle systems (2000 particles, camera-relative)
- **Characters** -- Skeletal animation with GPU vertex skinning (256 bones), race-aware textures
- **Buildings** -- WMO renderer with multi-material batches, frustum culling, 160-unit distance culling
- **Particles** -- M2 particle emitters with WotLK struct parsing, billboarded glow effects
- **Post-Processing** -- HDR, tonemapping, shadow mapping (2048x2048)

### Asset Pipeline
- Extracted loose-file **`Data/`** tree indexed by **`manifest.json`** (fast lookup + caching)
- Optional **overlay layers** for multi-expansion asset deduplication
- `asset_extract` + `extract_assets.sh` for MPQ extraction (StormLib tooling)
- File formats: **BLP** (DXT1/3/5), **ADT**, **M2**, **WMO**, **DBC** (Spell/Item/Faction/etc.)

### Gameplay Systems
- **Authentication** -- Full SRP6a implementation with RC4 header encryption
- **Character System** -- Creation (with nonbinary gender option), selection, 3D preview, stats panel, race/class support
- **Movement** -- WASD movement, camera orbit, spline path following
- **Combat** -- Auto-attack, spell casting with cooldowns, damage calculation, death handling
- **Targeting** -- Tab-cycling, click-to-target, faction-based hostility (using Faction.dbc)
- **Inventory** -- 23 equipment slots, 16 backpack slots, drag-drop, auto-equip
- **Spells** -- Spellbook with class specialty tabs, drag-drop to action bar, spell icons
- **Action Bar** -- 12 slots, drag-drop from spellbook/inventory, click-to-cast, keybindings
- **Trainers** -- Spell trainer UI, buy spells, known/available/unavailable states
- **Quests** -- Quest markers (! and ?) on NPCs and minimap, quest log, quest details, turn-in flow
- **Vendors** -- Buy and sell items, gold tracking, inventory sync
- **Loot** -- Loot window, gold looting, item pickup
- **Gossip** -- NPC interaction, dialogue options
- **Chat** -- Tabs/channels, emotes, chat bubbles, clickable URLs, clickable item links with tooltips
- **Party** -- Group invites, party list
- **Warden** -- Warden anti-cheat module execution via Unicorn Engine x86 emulation (cross-platform, no Wine)
- **UI** -- Loading screens with progress bar, settings window, minimap with zoom/rotation/square mode

## Building

### Prerequisites

```bash
# Ubuntu/Debian
sudo apt install libsdl2-dev libglew-dev libglm-dev \
                 libssl-dev cmake build-essential \
                 libunicorn-dev \          # for Warden module execution
                 libstorm-dev             # for asset_extract

# Fedora
sudo dnf install SDL2-devel glew-devel glm-devel \
                 openssl-devel cmake gcc-c++ \
                 unicorn-devel \          # for Warden module execution
                 StormLib-devel           # for asset_extract

# Arch
sudo pacman -S sdl2 glew glm openssl cmake base-devel \
                 unicorn \               # for Warden module execution
                 stormlib                # for asset_extract
```

### Game Data

This project requires WoW client data that you extract from your own legally obtained install.

Wowee loads assets via an extracted loose-file tree indexed by `manifest.json` (it does not read MPQs at runtime).

#### 1) Extract MPQs into `./Data/`

```bash
# WotLK 3.3.5a example
./extract_assets.sh /path/to/WoW/Data wotlk
```

```
Data/
  manifest.json
  interface/
  sound/
  world/
  expansions/
```

Notes:

- `StormLib` is required to build/run the extractor (`asset_extract`), but the main client does not require StormLib at runtime.
- `extract_assets.sh` supports `classic`, `turtle`, `tbc`, `wotlk` targets.

#### 2) Point wowee at the extracted data

By default, wowee looks for `./Data/`. You can override with:

```bash
export WOW_DATA_PATH=/path/to/extracted/Data
```

### Compile & Run

```bash
git clone --recurse-submodules https://github.com/Kelsidavis/WoWee.git
cd WoWee

mkdir build && cd build
cmake ..
make -j$(nproc)

./bin/wowee
```

## Controls

### Camera & Movement
| Key | Action |
|-----|--------|
| WASD | Move camera / character |
| Mouse | Look around / orbit camera |
| Shift | Move faster |
| Mouse Left Click | Target entity / interact |
| Tab | Cycle targets |

### UI & Windows
| Key | Action |
|-----|--------|
| I | Toggle inventory |
| P | Toggle spellbook |
| L | Toggle quest log |
| Enter | Open chat |
| Escape | Close windows / deselect |

### Action Bar
| Key | Action |
|-----|--------|
| 1-9, 0, -, = | Use action bar slots 1-12 |
| Drag & Drop | Spells from spellbook, items from inventory |
| Click | Cast spell / use item |

### Debug & Development
| Key | Action |
|-----|--------|
| F1 | Performance HUD |
| F2 | Wireframe mode |
| F9 | Toggle time progression |
| F10 | Toggle celestial bodies (sun + moons) |
| F11 | Toggle procedural stars (debug mode) |
| +/- | Change time of day |
| C | Toggle clouds |
| L | Toggle lens flare |
| W | Cycle weather (None/Rain/Snow) |
| K / J | Spawn / remove test characters |
| O / P | Spawn / clear WMOs |

## Documentation

### Getting Started
- [Project Status](docs/status.md) -- Current code state, limitations, and near-term direction
- [Quick Start](docs/quickstart.md) -- Installation and first steps
- [Build Instructions](BUILD_INSTRUCTIONS.md) -- Detailed dependency, build, and run guide

### Technical Documentation
- [Architecture](docs/architecture.md) -- System design and module overview
- [Authentication](docs/authentication.md) -- SRP6 auth protocol details
- [Server Setup](docs/server-setup.md) -- Local server configuration
- [Sky System](docs/SKY_SYSTEM.md) -- Celestial bodies, Azeroth astronomy, and WoW-accurate sky rendering
- [SRP Implementation](docs/srp-implementation.md) -- Cryptographic details
- [Packet Framing](docs/packet-framing.md) -- Network protocol framing
- [Realm List](docs/realm-list.md) -- Realm selection system
- [Warden Quick Reference](docs/WARDEN_QUICK_REFERENCE.md) -- Warden module execution overview and testing
- [Warden Implementation](docs/WARDEN_IMPLEMENTATION.md) -- Technical details of the implementation

## Technical Details

- **Graphics**: OpenGL 3.3 Core, GLSL 330, forward rendering with post-processing
- **Performance**: 60 FPS (vsync), ~50k triangles/frame, ~30 draw calls, <10% GPU
- **Platform**: Linux (primary), C++20, CMake 3.15+
- **Dependencies**: SDL2, OpenGL/GLEW, GLM, OpenSSL, ImGui, FFmpeg, Unicorn Engine (StormLib for asset extraction tooling)
- **Architecture**: Modular design with clear separation (core, rendering, networking, game logic, asset pipeline, UI, audio)
- **Networking**: Non-blocking TCP, SRP6a authentication, RC4 encryption, WoW 3.3.5a protocol
- **Asset Loading**: Extracted loose-file tree + `manifest.json` indexing, async terrain streaming, overlay layers
- **Sky System**: WoW-accurate DBC-driven architecture
  - **Skybox Authority**: Stars baked into M2 sky dome models (not procedurally generated)
  - **Lore-Accurate Moons**: White Lady (30-day cycle) + Blue Child (27-day cycle)
  - **Deterministic Phases**: Computed from server game time when available (fallback: local time/dev cycling)
  - **Camera-Locked**: Sky dome uses rotation-only transform (translation ignored)
  - **No Latitude Math**: Per-zone artistic constants, not Earth-like planetary simulation
  - **Zone Identity**: Different skyboxes per continent (Azeroth, Outland, Northrend)

## License

This project's source code is licensed under the [Wowee Source License (Entity-Restricted)](LICENSE).

This project does not include any Blizzard Entertainment proprietary data, assets, or code. World of Warcraft is (c) 2004-2024 Blizzard Entertainment, Inc. All rights reserved.

## References

- [WoWDev Wiki](https://wowdev.wiki/) -- File format documentation
- [TrinityCore](https://github.com/TrinityCore/TrinityCore) -- Server reference
- [MaNGOS](https://github.com/cmangos/mangos-wotlk) -- Server reference
- [StormLib](https://github.com/ladislav-zezula/StormLib) -- MPQ library

## Known Issues

### Water Rendering
- **Stormwind Canal Overflow**: Canal water surfaces extend spatially beyond their intended boundaries, causing water to appear in tunnels, buildings, and the park. This is due to oversized water mesh extents in the WoW data files.
  - **Current Workaround**: Water heights are lowered by 1 unit in Stormwind (tiles 28-50, 28-52) for surfaces above 94 units, with a 20-unit exclusion zone around the moonwell (-8755.9, 1108.9, 96.1). This hides most problem water while keeping canals and the moonwell functional.
  - **Limitation**: Some park water may still be visible. The workaround uses hardcoded coordinates and height thresholds rather than fixing the root cause.
  - **Proper Fix**: Would require trimming water surface meshes to actual boundaries in ADT/WMO data, or implementing spatial clipping at render time.
