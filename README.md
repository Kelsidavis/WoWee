# Wowee - World Of Warcraft Engine Experiment

<p align="center">
  <img src="assets/Wowee.png" alt="Wowee Logo" width="240" />
</p>

A native C++ client for World of Warcraft 3.3.5a (Wrath of the Lich King) with a fully functional OpenGL rendering engine.

> **Legal Disclaimer**: This is an educational/research project. It does not include any Blizzard Entertainment assets, data files, or proprietary code. World of Warcraft and all related assets are the property of Blizzard Entertainment, Inc. This project is not affiliated with or endorsed by Blizzard Entertainment. Users are responsible for supplying their own legally obtained game data files and for ensuring compliance with all applicable laws in their jurisdiction.

## Features

### Rendering Engine
- **Terrain** -- Multi-tile streaming with async loading, texture splatting (4 layers), frustum culling
- **Water** -- Animated surfaces, reflections, refractions, Fresnel effect
- **Sky System** -- WoW-accurate DBC-driven lighting with skybox authority
  - **Skybox** -- Camera-locked celestial sphere (M2 model support, gradient fallback)
  - **Celestial Bodies** -- Sun (lighting-driven), White Lady + Blue Child (Azeroth's two moons)
  - **Moon Phases** -- Server time-driven deterministic phases (30-day / 27-day cycles)
  - **Stars** -- Baked into skybox assets (procedural fallback for development/debug only)
- **Atmosphere** -- Procedural clouds (FBM noise), lens flare with chromatic aberration, cloud/fog star occlusion
- **Weather** -- Rain and snow particle systems (2000 particles, camera-relative)
- **Characters** -- Skeletal animation with GPU vertex skinning (256 bones), race-aware textures
- **Buildings** -- WMO renderer with multi-material batches, frustum culling, 160-unit distance culling
- **Particles** -- M2 particle emitters with WotLK struct parsing, billboarded glow effects
- **Post-Processing** -- HDR, tonemapping, shadow mapping (2048x2048)

### Asset Pipeline
- **MPQ** archive extraction (StormLib), **BLP** DXT1/3/5 textures, **ADT** terrain tiles, **M2** character models with animations, **WMO** buildings, **DBC** database files (Spell, Item, SkillLine, Faction, etc.)

### Gameplay Systems
- **Authentication** -- Full SRP6a implementation with RC4 header encryption
- **Character System** -- Creation, selection, 3D preview, stats panel, race/class support
- **Movement** -- WASD movement, camera orbit, spline path following
- **Combat** -- Auto-attack, spell casting with cooldowns, damage calculation, death handling
- **Targeting** -- Tab-cycling, click-to-target, faction-based hostility (using Faction.dbc)
- **Inventory** -- 23 equipment slots, 16 backpack slots, drag-drop, auto-equip
- **Spells** -- Spellbook with class specialty tabs, drag-drop to action bar, spell icons
- **Action Bar** -- 12 slots, drag-drop from spellbook/inventory, click-to-cast, keybindings
- **Quests** -- Quest markers (! and ?) on NPCs and minimap, quest log, quest details, turn-in flow
- **Vendors** -- Buy and sell items, gold tracking, inventory sync
- **Loot** -- Loot window, gold looting, item pickup
- **Gossip** -- NPC interaction, dialogue options
- **Chat** -- SAY, YELL, WHISPER, chat window with formatting
- **Party** -- Group invites, party list
- **UI** -- Loading screens with progress bar, settings window with opacity slider

## Building

### Prerequisites

```bash
# Ubuntu/Debian
sudo apt install libsdl2-dev libglew-dev libglm-dev \
                 libssl-dev libstorm-dev cmake build-essential

# Fedora
sudo dnf install SDL2-devel glew-devel glm-devel \
                 openssl-devel StormLib-devel cmake gcc-c++

# Arch
sudo pacman -S sdl2 glew glm openssl stormlib cmake base-devel
```

### Game Data

This project requires WoW 3.3.5a (patch 3.3.5, build 12340) data files. You must supply your own legally obtained copy. Place (or symlink) the MPQ files into a `Data/` directory at the project root:

```
wowee/
└── Data/
    ├── common.MPQ
    ├── common-2.MPQ
    ├── expansion.MPQ
    ├── lichking.MPQ
    ├── patch.MPQ
    ├── patch-2.MPQ
    ├── patch-3.MPQ
    └── enUS/          (or your locale)
```

Alternatively, set the `WOW_DATA_PATH` environment variable to point to your WoW data directory.

### Compile & Run

```bash
git clone https://github.com/yourname/wowee.git
cd wowee

# Get ImGui (required)
git clone https://github.com/ocornut/imgui.git extern/imgui

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

## Technical Details

- **Graphics**: OpenGL 3.3 Core, GLSL 330, forward rendering with post-processing
- **Performance**: 60 FPS (vsync), ~50k triangles/frame, ~30 draw calls, <10% GPU
- **Platform**: Linux (primary), C++20, CMake 3.15+
- **Dependencies**: SDL2, OpenGL/GLEW, GLM, OpenSSL, StormLib, ImGui, FFmpeg
- **Architecture**: Modular design with clear separation (core, rendering, networking, game logic, asset pipeline, UI, audio)
- **Networking**: Non-blocking TCP, SRP6a authentication, RC4 encryption, WoW 3.3.5a protocol
- **Asset Loading**: Async terrain streaming, lazy loading, MPQ archive support
- **Sky System**: WoW-accurate DBC-driven architecture
  - **Skybox Authority**: Stars baked into M2 sky dome models (not procedurally generated)
  - **Lore-Accurate Moons**: White Lady (30-day cycle) + Blue Child (27-day cycle)
  - **Deterministic Phases**: Computed from server game time (consistent across sessions)
  - **Camera-Locked**: Sky dome uses rotation-only transform (translation ignored)
  - **No Latitude Math**: Per-zone artistic constants, not Earth-like planetary simulation
  - **Zone Identity**: Different skyboxes per continent (Azeroth, Outland, Northrend)

## License

This project's source code is licensed under the [MIT License](LICENSE).

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
