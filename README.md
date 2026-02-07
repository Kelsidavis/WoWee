# Wowee

A native C++ client for World of Warcraft 3.3.5a (Wrath of the Lich King) with a fully functional OpenGL rendering engine.

> **Legal Disclaimer**: This is an educational/research project. It does not include any Blizzard Entertainment assets, data files, or proprietary code. World of Warcraft and all related assets are the property of Blizzard Entertainment, Inc. This project is not affiliated with or endorsed by Blizzard Entertainment. Users are responsible for supplying their own legally obtained game data files and for ensuring compliance with all applicable laws in their jurisdiction.

## Features

### Rendering Engine
- **Terrain** -- Multi-tile streaming with async loading, texture splatting (4 layers), frustum culling
- **Water** -- Animated surfaces, reflections, refractions, Fresnel effect
- **Sky** -- Dynamic day/night cycle, sun/moon with orbital movement
- **Stars** -- 1000+ procedurally placed stars (night-only)
- **Atmosphere** -- Procedural clouds (FBM noise), lens flare with chromatic aberration
- **Moon Phases** -- 8 realistic lunar phases with elliptical terminator
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
| F10 | Toggle sun/moon |
| F11 | Toggle stars |
| +/- | Change time of day |
| C | Toggle clouds |
| L | Toggle lens flare |
| W | Cycle weather (None/Rain/Snow) |
| K / J | Spawn / remove test characters |
| O / P | Spawn / clear WMOs |

## Documentation

### Getting Started
- [Quick Start](docs/quickstart.md) -- Installation and first steps
- [Features Overview](FEATURES.md) -- Complete feature list
- [Changelog](CHANGELOG.md) -- Development history and recent changes

### Technical Documentation
- [Architecture](docs/architecture.md) -- System design and module overview
- [Authentication](docs/authentication.md) -- SRP6 auth protocol details
- [Server Setup](docs/server-setup.md) -- Local server configuration
- [Single Player](docs/single-player.md) -- Offline mode
- [SRP Implementation](docs/srp-implementation.md) -- Cryptographic details
- [Packet Framing](docs/packet-framing.md) -- Network protocol framing
- [Realm List](docs/realm-list.md) -- Realm selection system

## Technical Details

- **Graphics**: OpenGL 3.3 Core, GLSL 330, forward rendering with post-processing
- **Performance**: 60 FPS (vsync), ~50k triangles/frame, ~30 draw calls, <10% GPU
- **Platform**: Linux (primary), C++17, CMake 3.15+
- **Dependencies**: SDL2, OpenGL/GLEW, GLM, OpenSSL, StormLib, ImGui, FFmpeg
- **Architecture**: Modular design with clear separation (core, rendering, networking, game logic, asset pipeline, UI, audio)
- **Networking**: Non-blocking TCP, SRP6a authentication, RC4 encryption, WoW 3.3.5a protocol
- **Asset Loading**: Async terrain streaming, lazy loading, MPQ archive support

## License

This project's source code is licensed under the [MIT License](LICENSE).

This project does not include any Blizzard Entertainment proprietary data, assets, or code. World of Warcraft is (c) 2004-2024 Blizzard Entertainment, Inc. All rights reserved.

## References

- [WoWDev Wiki](https://wowdev.wiki/) -- File format documentation
- [TrinityCore](https://github.com/TrinityCore/TrinityCore) -- Server reference
- [MaNGOS](https://github.com/cmangos/mangos-wotlk) -- Server reference
- [StormLib](https://github.com/ladislav-zezula/StormLib) -- MPQ library

## Known Issues

- Stormwind Mage Quarter: around the Moonwell area, water is currently overflowing into nearby geometry and should be constrained to the proper liquid volume.
