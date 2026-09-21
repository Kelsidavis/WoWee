# Quick Start Guide

## Current Status

Wowee is a native C++ World of Warcraft client focused on online multiplayer. It speaks the Vanilla 1.12 (including Turtle WoW 1.18), TBC 2.4.3 and WotLK 3.3.5a protocols.

Implemented today:

- SRP6a authentication + world connection
- Character creation/selection and in-world entry
- Vulkan 1.3 rendering pipeline (terrain, water, sky, M2/WMO, particles)
- Core gameplay plumbing (movement, combat/spell casting, inventory/equipment, chat)
- Blizzard's FrameXML interface, loaded from your extracted game data
- Keyboard and mouse, or a game controller
- Transport support (boats/zeppelins) with active ongoing fixes

For a more honest snapshot of gaps and current direction, see `docs/status.md`.

## Build And Run

### 1. Clone

```bash
git clone --recurse-submodules https://github.com/Kelsidavis/WoWee.git
cd WoWee
```

### 2. Build

The desktop build needs SDL3 and a Vulkan 1.3 driver (MoltenVK on macOS). Package lists are in the README's Quick start.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

### 3. Provide WoW Data (Extract + Manifest)

Wowee loads assets from an extracted loose-file tree indexed by `manifest.json`.

If nothing is extracted yet, the client opens its asset builder instead of the login screen. Choose your World of Warcraft folder, build, then reopen the client. The builder is also on the login screen under **more options** → **add or rebuild assets...**, and is built as the standalone `wowee_assets`. See `docs/asset-manager.md`.

To extract from a terminal instead:

```bash
# WotLK 3.3.5a example
./extract_assets.sh /path/to/WoW/Data wotlk
```

The client looks for data in `WOW_DATA_PATH` if set, then in the per-user data directory (`~/Library/Application Support/Wowee/Data` on macOS, `%LOCALAPPDATA%\Wowee\Data` on Windows, `~/.local/share/wowee/Data` on Linux) when it holds an extraction, then in `./Data/`. To override:

```bash
export WOW_DATA_PATH=/path/to/extracted/Data
```

### 4. Run

```bash
./build/bin/wowee
```

## Connect To A Server

1. Launch `./build/bin/wowee`
2. Pick a server from the **Server** list (ChromieCraft is listed), or choose **Somewhere else...** and enter the address and port under **more options**
3. Enter account credentials
4. Log in, pick realm, pick character, enter world

For local AzerothCore setup, see `docs/server-setup.md`.

## Useful Controls

- `W`/`S`: Move forward/back
- `A`/`D`: Turn (strafe while right mouse is held)
- `Q`/`E`: Strafe
- `Mouse`: Look/orbit camera
- `Tab`: Target nearest enemy
- `1-9,0,-,=`: Action bar slots
- `B`: Bags
- `C`: Character
- `P`: Spellbook
- `N`: Talents
- `L`: Quest log
- `M`: World map
- `O`: Social window
- `Enter`: Chat
- `/`: Chat slash command
- `Escape`: Game menu (Key Bindings, Video, Sound, Interface)
- `F1`: Performance HUD (debug builds only)

A controller also works: left stick moves, right stick looks, face buttons and D-pad are action slots, and Back turns the right stick into a pointer. Its buttons are listed in the Key Bindings panel.

## Troubleshooting

### Build fails on missing dependencies

Use `BUILD_INSTRUCTIONS.md` for distro-specific package lists.

### Client cannot connect

- Verify auth/world server is running
- Check host/port settings
- Check server logs and client logs in `logs/wowee.log`

### Missing assets (models/textures/terrain)

- Run the asset builder (it opens by itself when nothing is extracted), or re-run `./extract_assets.sh ...`
- Verify a `manifest.json` exists under `Data/expansions/<expansion>/`
- Or export `WOW_DATA_PATH=/path/to/extracted/Data`
