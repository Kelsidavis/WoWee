# WoWee — World of Warcraft Engine Experiment

<p align="center">
  <img src="assets/Wowee.png" alt="WoWee logo" width="240" />
</p>

<p align="center">
  A native C++ World of Warcraft client with a custom Vulkan renderer.
</p>

<p align="center">
  <a href="https://github.com/sponsors/Kelsidavis"><img src="https://img.shields.io/github/sponsors/Kelsidavis?label=Sponsor&logo=GitHub" alt="Sponsor" /></a>
  <a href="https://discord.gg/PSdMPS8uje"><img src="https://img.shields.io/badge/Discord-Join-5865F2?logo=discord&logoColor=white" alt="Join the WoWee Discord" /></a>
</p>

WoWee supports **Vanilla 1.12**, **TBC 2.4.3**, and **WotLK 3.3.5a** through
expansion-specific profiles and packet parsers. It has been tested with
AzerothCore/ChromieCraft, TrinityCore, MaNGOS, and Turtle WoW 1.18.

[Watch the latest demonstration](https://youtu.be/1Ax1jeNV_GU) ·
[Download the latest release](https://github.com/Kelsidavis/WoWee/releases/latest) ·
[Read the project status](docs/status.md)

<p align="center">
  <img src="assets/orgrimmar-entrance.png" alt="WoWee rendering Orgrimmar" width="100%" />
</p>

> [!NOTE]
> macOS release DMGs are Developer ID signed, notarized by Apple, and stapled
> before publication. Gatekeeper should identify them as notarized Developer ID
> software without requiring an **Open Anyway** exception.

> [!IMPORTANT]
> WoWee is an educational and research project. It contains no Blizzard
> Entertainment assets, data, or proprietary code. You must supply your own
> legally obtained game data and comply with the laws in your jurisdiction.
> WoWee is not affiliated with or endorsed by Blizzard Entertainment.

## What works

- Vulkan terrain, WMO, M2, water/lava, particles, lighting, shadows, weather,
  and asynchronous world streaming
- SRP6 authentication, RC4 header encryption, and protocol handling for all
  three supported expansions
- Character creation and selection, movement, transports, combat, spells,
  talents, inventory, banks, vendors, trainers, quests, loot, mail, auction
  house, gossip, chat, parties, pets, maps, and taxi travel
- Optional Warden module execution through Unicorn Engine x86 emulation

This is an active work in progress, not a drop-in replacement for the official
client. See [Known limitations](#known-limitations) before reporting a bug.

### Experimental components

The world editor and AMD FSR3 frame generation are early developer features.
Their interfaces, formats, runtime requirements, and behavior may change. FSR2
upscaling is available, but graphics acceleration features are still under
active development and should not be treated as release-critical functionality.

## Quick start

### 1. Install dependencies

Unicorn enables Warden execution. StormLib is needed by the asset extractor,
but is not required by the client at runtime.

<details>
<summary>Ubuntu / Debian</summary>

```bash
sudo apt install build-essential cmake pkg-config git \
  libsdl2-dev libglm-dev libssl-dev zlib1g-dev libx11-dev \
  libvulkan-dev vulkan-tools glslc \
  libavformat-dev libavcodec-dev libswscale-dev libavutil-dev

# Optional: Warden execution and MPQ extraction
sudo apt install libunicorn-dev
sudo apt install libstorm-dev
```

</details>

<details>
<summary>Fedora</summary>

```bash
sudo dnf install gcc-c++ cmake pkgconf-pkg-config git \
  SDL2-devel glm-devel openssl-devel zlib-devel libX11-devel \
  vulkan-devel vulkan-tools glslc ffmpeg-devel

# Optional: Warden execution
sudo dnf install unicorn-devel
```

For `asset_extract`, build [StormLib](https://github.com/ladislav-zezula/StormLib)
from source if it is unavailable in your enabled Fedora repositories.

</details>

<details>
<summary>Arch Linux</summary>

```bash
sudo pacman -S base-devel cmake pkgconf git \
  sdl2-compat glm openssl zlib libx11 \
  vulkan-headers vulkan-icd-loader vulkan-tools shaderc \
  ffmpeg
```

Install `unicorn` for optional Warden execution. StormLib is not in the official
repositories; install `stormlib-git` from the AUR if you need `asset_extract`.

</details>

<details>
<summary>macOS</summary>

Vulkan runs through MoltenVK.

```bash
brew install cmake pkg-config sdl2 glm openssl@3 zlib ffmpeg \
  vulkan-loader vulkan-headers molten-vk shaderc

# Optional: Warden execution and MPQ extraction
brew install unicorn stormlib
```

</details>

For Windows/MSYS2, Visual Studio, and platform-specific notes, see the
[complete build guide](BUILD_INSTRUCTIONS.md).

### 2. Clone and build

```bash
git clone --recurse-submodules https://github.com/Kelsidavis/WoWee.git
cd WoWee

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

The optional `build.sh` and `build.ps1` helpers fetch supported AMD SDK sources
when needed. A plain CMake build works without them and uses the internal
fallback when optional SDK components are unavailable.

Lua 5.1 is vendored and built in, so nothing needs installing for it. To link
an installed one instead — which is what a distribution package usually wants —
configure with `-DWOWEE_SYSTEM_LUA=ON`. It must be 5.1: the interface this
client runs is written for that version, and configuring stops with a message
rather than linking a 5.2 or later.

### 3. Extract game data

WoWee does not read MPQs at runtime. Extract a legally obtained client into a
loose-file `Data/` tree with a generated `manifest.json`:

```bash
# Linux / macOS
./extract_assets.sh /path/to/WoW/Data wotlk

# Windows PowerShell
.\extract_assets.ps1 "C:\Games\WoW-3.3.5a\Data" wotlk
```

Valid targets are `classic`, `turtle`, `tbc`, and `wotlk`. Each target is kept
separately, so multiple expansions can coexist:

```text
Data/
└── expansions/
    ├── classic/manifest.json
    ├── turtle/manifest.json
    ├── tbc/manifest.json
    └── wotlk/manifest.json
```

The login-screen **Assets** selector normally follows the selected server's
expansion. It can be overridden per saved server profile.

To store data elsewhere:

```bash
export WOW_DATA_PATH=/path/to/extracted/Data
```

The [Asset Pipeline GUI](docs/asset-pipeline-gui.md) provides extraction,
texture-pack management, ordering, and override controls.

### 4. Run

```bash
./build/bin/wowee
```

If a MaNGOS realm advertises a world address that is unreachable from your LAN,
override only the host; the advertised port is preserved:

```bash
export WOWEE_REALM_HOST_OVERRIDE=192.168.1.50
```

Turtle WoW defaults to authentication build 7272. Older compatible servers can
use:

```bash
export WOWEE_TURTLE_AUTH_BUILD=7234
```

## Container builds

Docker or Podman can build all supported targets without installing a host
toolchain:

```bash
./container/run-linux.sh    # build/linux/bin/wowee
./container/run-macos.sh    # build/macos/bin/wowee
./container/run-windows.sh  # build/windows/bin/wowee.exe
```

PowerShell equivalents are included. macOS defaults to ARM64; set
`MACOS_ARCH=x86_64` for Intel. See [Container Builds](container/README.md) for
all options.

## Experimental world editor

`wowee_editor` is an early-stage tool for experimenting with custom zones and
open, JSON-friendly formats. It is not yet a production-ready content pipeline.

```bash
cmake --build build --target wowee_editor
./build/bin/wowee_editor --data Data

# Batch conversion examples
./build/bin/wowee_editor --convert-m2 Creature/Bear/Bear.m2 --data Data
./build/bin/wowee_editor --convert-wmo World/WMO/Stormwind/Stormwind.wmo --data Data
```

Current work includes terrain editing, object placement, water, NPCs, quests,
lighting, and experimental AzerothCore export. Exported zones can load from
`custom_zones/` or `output/`, but formats and compatibility may change.

See the [editor format specification](tools/editor/FORMAT_SPEC.md) for the
custom binary and catalog formats.

## Controls

| Input | Action |
|---|---|
| `W` `A` `S` `D` | Move |
| Mouse | Look or orbit camera |
| Left click | Target or interact |
| `Tab` | Cycle targets |
| `1`–`0`, `-`, `=` | Action-bar slots 1–12 |
| `B` / `C` / `P` / `N` | Bags / character / spellbook / talents |
| `L` / `M` / `O` | Quest log / map / guild roster |
| `Enter` or `/` | Open chat |
| `/unstuck` | Recover when terrain or WMO collision traps the character |
| `Escape` | Close windows or deselect |
| `F1` / `F4` | Performance HUD / shadows |

Graphics presets and individual controls are available under **Video
Settings**. Shadows and MSAA have the largest performance cost; FSR2 can improve
frame rate on supported hardware.

## Documentation

### Getting started

- [Quick Start](docs/quickstart.md)
- [Complete Build Instructions](BUILD_INSTRUCTIONS.md)
- [Server Setup](docs/server-setup.md)
- [Project Status](docs/status.md)
- [Asset Pipeline GUI](docs/asset-pipeline-gui.md)

### Internals

- [Architecture](docs/architecture.md)
- [Authentication](docs/authentication.md) and [SRP Implementation](docs/srp-implementation.md)
- [Packet Framing](docs/packet-framing.md) and [Realm List](docs/realm-list.md)
- [Sky System](docs/SKY_SYSTEM.md)
- [Warden Quick Reference](docs/WARDEN_QUICK_REFERENCE.md) and [Implementation](docs/WARDEN_IMPLEMENTATION.md)

## Development and CI

WoWee uses C++20 and CMake 3.15+. GitHub Actions builds Linux x86-64/ARM64,
Windows x86-64/ARM64, and macOS ARM64 releases. Security checks include CodeQL,
Semgrep, AddressSanitizer, and UndefinedBehaviorSanitizer.

The codebase is split into focused rendering, networking, gameplay, asset, UI,
audio, and editor modules. Start with the [architecture guide](docs/architecture.md)
before making broad changes.

## Known limitations

- The Warden RSA modulus is a placeholder; module execution otherwise works
  through Unicorn x86 emulation.
- Terrain/WMO transitions are a longstanding regression area. Character floor
  selection can occasionally prefer the wrong surface or leave the character
  stuck. Enter `/unstuck` in chat to recover, then include the location and
  relevant log lines in a bug report.
- Long rotational Northrend transport paths can occasionally show spline-wrap
  glitches.
- World-map zone hover has edge cases near continent boundaries.
- Pet-bar protocol behavior may require adjustment for non-AzerothCore forks.

When reporting a bug, include the relevant client log lines, expansion, server
core, and reproduction steps.

## License and references

WoWee source code is available under the [MIT License](LICENSE). World of
Warcraft and its assets are property of Blizzard Entertainment, Inc.

- [WoWDev Wiki](https://wowdev.wiki/) — file-format documentation
- [TrinityCore](https://github.com/TrinityCore/TrinityCore) — server reference
- [MaNGOS](https://github.com/cmangos/mangos-wotlk) — server reference
- [StormLib](https://github.com/ladislav-zezula/StormLib) — MPQ library
