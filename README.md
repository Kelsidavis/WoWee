# WoWee

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

WoWee is a complete replacement for the World of Warcraft client: a native C++
program with its own Vulkan renderer, networking and audio, that logs into
**Vanilla 1.12**, **TBC 2.4.3**, **WotLK 3.3.5a** and **Turtle WoW 1.18** servers
and plays the game from your own game data. It runs Blizzard's own interface and
your addons, and it answers the servers' anti-cheat. It is played on
AzerothCore/ChromieCraft, TrinityCore, CMaNGOS/vmangos and Turtle WoW, on Linux,
macOS, Windows and Android.

[Watch the latest demonstration](https://youtu.be/1Ax1jeNV_GU) ·
[Download the latest release](https://github.com/Kelsidavis/WoWee/releases/latest) ·
[Read the project status](docs/status.md) ·
[Hear the soundtrack](https://kelsidavis.bandcamp.com/album/wowee-soundtrack)

<p align="center">
  <img src="assets/hellfire-peninsula.png" alt="WoWee rendering Honor Hold in Hellfire Peninsula" width="100%" />
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

**The game.** Character creation and selection, movement and swimming, flying
and mounts, combat, spells and auras, talents, pets, quests and their map
objectives, loot and roll windows, inventory, bags, the bank, vendors, trainers,
the auction house, mail, trade, gossip, chat and channels, parties and raids,
guilds, the Dungeon Finder, reputation, equipment sets, achievements, the world
and flight maps, taxis, boats, zeppelins and lifts, and instances. Each
expansion is spoken in its own wire format, from expansion profiles and packet
parsers.

**The interface.** Blizzard's FrameXML, loaded from your game data and drawn by
the client: the same frames, Lua and events the original client runs, with a
Lua API wide enough for the stock interface of every expansion. Addons in the
game's `Interface\AddOns` folder load as they would there. Key bindings are the
game's own panel, with two keys per command, and the client's settings sit in
the game's Video, Sound and Interface panels.

**The world.** A Vulkan 1.3 renderer for terrain, buildings, models and
characters, water, lava and slime, particles and spell effects, shadows,
weather, the day and night sky, volumetric fog and sun shafts, GPU grass, and
asynchronous world streaming. FSR 1 and FSR 3 upscaling are built in.

**The sound.** Zone and city music, tavern and weather ambience, footsteps,
mounts, combat, spell, interface and NPC voice audio, from your game data, and
[a soundtrack of its own](#soundtrack).

**Logging in.** SRP6 authentication, the realm list and header encryption for
every expansion, the login hashes servers check, and Warden: the module the
server sends is verified and run under x86 emulation, and its checks are
answered as a genuine client answers them. The answers are graded against
AzerothCore's own check list in the test suite.

**Your game data.** An asset builder, in the client and as the standalone
`wowee_assets` window, extracts your own installation - several games into one
folder if you like - and can borrow upgraded art from a later client you own.

**Input.** Keyboard and mouse, mouse buttons 4 and 5, or a game controller on
every platform, with buttons named as the pad in hand names them (Xbox,
PlayStation, Nintendo, Steam Deck) and its back paddles on action slots.

**Platforms.** Linux, macOS and Windows on x86-64 and ARM64, and Android on
arm64 with on-screen controls.

### Experimental components

The world editor, ray traced lighting and AMD FSR3 frame generation are early
developer features; their interfaces, formats and behavior may change. Frame
generation is only built when `WOWEE_ENABLE_AMD_FSR3_FRAMEGEN` is turned on
with AMD's SDK under `extern/`. Cataclysm 4.3.4 support is in progress: its
movement packets are done, and the client does not reach its world yet.

## Quick start

### 1. Install dependencies

The desktop client uses SDL3 and needs a Vulkan 1.3 driver (MoltenVK on
macOS). Unicorn enables Warden execution. StormLib is needed by
`asset_extract`, the `wowee_assets` window and the asset builder inside the
client; without it the client still builds, with those left out.

**Linux** needs no libraries and no compiler on your machine. The build runs in
a container and ends as an AppImage that carries its own libraries, so
[Docker](https://docs.docker.com/get-docker/) or Podman and a Vulkan 1.3 graphics
driver are all there is to install. The package lists for compiling natively on
Ubuntu, Debian, Fedora and Arch, which only development needs, are in the
[complete build guide](BUILD_INSTRUCTIONS.md).

<details>
<summary>macOS</summary>

Vulkan runs through MoltenVK.

```bash
brew install cmake pkg-config sdl3 glm openssl@3 zlib ffmpeg \
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
```

**Linux.** The [releases](https://github.com/Kelsidavis/WoWee/releases/latest)
are AppImages, and the container builds the same:

```bash
./container/run-linux.sh        # on a Windows host: .\container\run-linux.ps1
```

It leaves `build/linux/wowee-<version>-<arch>.AppImage` and nothing else. The
first run builds the container image and compiles everything, which takes a
while; later runs compile only what changed.

**macOS and Windows:**

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

With the default Makefile generator, `--parallel` without a number starts
every compile job at once, and the larger files need a lot of memory; that has run a
32 GB machine out of memory. Give it a limit, such as `cmake --build build -j2`,
or `-j4` with 16 GB or more.

The `build.sh` and `build.ps1` helpers run the same build and also clone AMD's
FidelityFX SDKs into `extern/`. Those are only used when `WOWEE_ENABLE_AMD_FSR2`
or `WOWEE_ENABLE_AMD_FSR3_FRAMEGEN` is turned on, and both are off by default;
the client's own FSR 1 and FSR 3 upscaling need neither.

### 3. Extract game data

WoWee does not read MPQs at runtime. It needs a loose-file tree with a
generated `manifest.json`, extracted from a legally obtained client.

The asset builder does this. When the client starts and finds nothing
extracted, it opens the builder instead of the login screen; later it is under
**more options** → **add or rebuild assets...** on the login screen, and it is
also the standalone `wowee_assets` window. Point it at your World of Warcraft
folder and it extracts into the per-user data directory the client reads.
Reopen the client when a build finishes. See [the asset manager](docs/asset-manager.md)
for upgrades, packs, and several games in one folder.

From a terminal, the scripts extract into `Data/` in the checkout:

```bash
# macOS
./extract_assets.sh /path/to/WoW/Data wotlk

# Windows PowerShell
.\extract_assets.ps1 "C:\Games\WoW-3.3.5a\Data" wotlk
```

The Linux AppImage carries the extractor and the asset manager. The manager
writes to the per-user data directory the client reads; point the extractor
there:

```bash
./wowee-*.AppImage --asset-manager
./wowee-*.AppImage --asset-extract --mpq-dir /path/to/WoW/Data \
  --output ~/.local/share/wowee/Data --expansion-subdir --expansion wotlk
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

The login-screen **Assets** selector appears once more than one set is
installed. It normally follows the selected server's expansion and can be
overridden per saved server profile.

The client looks for data in this order:

1. `WOW_DATA_PATH`, if set
2. The per-user data directory, if it holds an extraction:
   `~/Library/Application Support/Wowee/Data` on macOS,
   `%LOCALAPPDATA%\Wowee\Data` on Windows, and `$XDG_DATA_HOME/wowee/Data`
   (default `~/.local/share/wowee/Data`) on Linux
3. `Data/` in the working directory, which on Linux and macOS is the
   executable's own directory. CMake links `build/bin/Data` to the checkout's
   `Data/` on macOS and in a native Linux build; on Windows `build.ps1` makes a
   junction. In the AppImage that directory is inside the image and holds only
   the expansion profiles, so extract into the per-user directory, as above

To store data elsewhere:

```bash
export WOW_DATA_PATH=/path/to/extracted/Data
```

### 4. Run

On Linux, run the AppImage you built or downloaded:

```bash
chmod +x wowee-*.AppImage
./wowee-*.AppImage
```

With no arguments it starts the client, and any arguments are passed on to the
client. `--asset-manager` and `--asset-extract` start the asset tools instead
(see step 3); they work only as the first argument.

An AppImage mounts itself through FUSE. Where there is none, such as inside a
container, run it with `--appimage-extract-and-run`. On macOS and Windows:

```bash
./build/bin/wowee
```

Choose a server from the login screen's **Server** list, which carries
ChromieCraft and every server you have logged into. **Somewhere else...** opens
the address and port fields under **more options**, where the expansion is
chosen too.

Once at startup the client asks GitHub whether a newer release exists and, if
so, shows its tag beside the version on the login screen. Nothing is
downloaded; **Check for new versions** under Interface → WoWee → Interface
turns it off.

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

## Android

The Android client is the same tree built for arm64. It reaches the login
screen, character selection and the world, and is tested on a Pixel 9a.

Install `wowee-<version>-android-arm64.apk` from
[the latest release](https://github.com/Kelsidavis/WoWee/releases/latest). It
needs **Android 13 or newer** on **arm64**: Android's `libvulkan.so` only
exports the Vulkan 1.2/1.3 entry points the renderer uses from API 33. The APK
is signed with the debug key, so Android asks you to allow installation from an
unknown source.

### Game data on a phone

Extract on a desktop exactly as above. There is no need to extract on the
device. A full extraction is around 18 GB, so cut it down to a profile that
fits, then copy the result across. Open the app once before pushing, so Android
creates its folder:

```bash
tools/android/make_minimal_data.py --source ~/Data --out ~/Data-phone \
    --profile world --maps all
adb push ~/Data-phone/. /sdcard/Android/data/com.wowee.client/files/Data/
adb shell chmod -R a+rwX /sdcard/Android/data/com.wowee.client/files/Data
```

The subset also carries the extracted `WoW.exe` and its DLLs from
`expansions/*/misc`. Without them a server that runs Warden sees every memory
check from the phone fail, which some servers treat as a modified client.

The trailing `/.` matters: push the **contents**, not the directory. Nesting it
one level deeper leaves the client unable to find its `manifest.json`, and it
starts with no game data at all.

The `chmod` matters too. Files `adb push` writes into the app's folder belong to
the shell user, not the app, and their directories are closed to it; without
the `chmod` the client stops at startup with `Permission denied` on
`manifest.json`. Run it again after any later push.

| Profile | Size | Reaches |
|---|---|---|
| `login` | 787 MB | Login, character selection and creation. No world |
| `world --maps azeroth` | 7.5 GB | One continent |
| `world --maps all` | 12 GB | Every map |
| `full` | 18 GB | Everything, sound included |

A subset carries its own rewritten `manifest.json`, because that file is how the
client locates its data root. See [tools/android/README.md](tools/android/README.md)
for the profiles and how to check one.

### Touch controls

| Input | Action |
|---|---|
| Left thumb, lower left | Move and strafe |
| Right thumb, drag | Turn the view; the character faces where it looks |
| Two fingers | Zoom the camera |
| Tap | Target, interact, and everything in the interface |

A keyboard and mouse should work when attached, since SDL delivers them the same
way, but that is untested.

### Building it yourself

```bash
tools/build-android-deps.sh arm64-v8a          # OpenSSL, once
cd android && ./gradlew :app:assembleDebug     # needs ANDROID_HOME and NDK 28
```

Debug builds carry this project's own music; release builds leave it out, the
same split the desktop archives use. `adb logcat -s wowee` shows the client's
log, and `adb shell setprop debug.wowee.loglevel info` opens it up beyond
warnings.

## Container builds

Docker or Podman can build all supported targets without installing a host
toolchain:

```bash
./container/run-linux.sh    # build/linux/wowee-<version>-<arch>.AppImage
./container/run-macos.sh    # build/macos/bin/wowee
./container/run-windows.sh  # build/windows/bin/wowee.exe
```

PowerShell equivalents are included. The Linux build leaves only the AppImage;
its build tree stays in the Docker volume `wowee-linux-build`. macOS defaults to
ARM64; set `MACOS_ARCH=x86_64` for Intel. See
[Container Builds](container/README.md) for all options.

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
| `W` `S` | Move forward and back |
| `A` `D` | Turn; strafe while the right mouse button is held |
| `Q` `E` | Strafe |
| Mouse | Look or orbit camera |
| Left click | Target or interact |
| `Tab` | Cycle targets |
| `1`–`0`, `-`, `=` | Action-bar slots 1–12 |
| `B` / `C` / `P` / `N` | Bags / character / spellbook / talents |
| `L` / `M` / `O` / `H` | Quest log / map / social / player vs player |
| `Enter` or `/` | Open chat |
| `/unstuck` | Recover when terrain or WMO collision traps the character |
| `Escape` | Close windows, deselect, or open the game menu |
| `F1` | Performance HUD (debug builds only) |

Keys are listed and can be rebound in the game's **Key Bindings** panel, from
the menu `Escape` opens.

On Android the same actions are driven by
[touch controls](#touch-controls).

### Controller

A controller works on every platform the client runs on. The left stick moves,
the right stick looks and the triggers zoom. The bottom face button jumps and
the right one, like Start, acts as `Escape`. The other two face buttons and the
D-pad are action slots 1–6; hold the left bumper for 7–12. The right bumper
targets the nearest enemy, clicking the left stick toggles autorun, and clicking
the right stick sits (or dives while swimming). Back paddles - a Steam Deck's
L4, R4, L5 and R5, an Elite pad's P1 to P4, a DualSense Edge's - take slots
7–10 with nothing held.

**Back** switches the right stick to moving the pointer, for looting, gossip
and vendors: the bottom face button clicks and the left one right-clicks.

Buttons are named as the pad in hand names them - A, Cross or B for the same
button on Xbox, PlayStation and Nintendo pads - and the scheme is listed in the
**Key Bindings** panel, where it can be rebound. Look speed, inversion and
deadzone are under Interface → WoWee → Camera. A pad SDL does not recognise
can be described in a `gamecontrollerdb.txt` in the per-user data directory.

### Settings

Escape opens the game menu. This client's own settings sit under a **WoWee**
heading in the game's **Video**, **Sound** and **Interface** panels: Graphics,
Detail, Grass, Upscaling and Display under Video, with the quality preset on
Graphics. View distance, shadow distance and MSAA have the largest performance
cost; FSR 1 or FSR 3 upscaling can improve frame rate on weaker hardware.

## Soundtrack

The client carries music of its own: the login theme, the track taverns open
their rotation on, and zone rotations that play alongside whatever your game
data provides - Elwynn, the Barrens, Booty Bay, Lordaeron, Stormwind and
Ironforge among them. **WoWee soundtrack** under Sound → WoWee → Sound turns it
off and leaves the zone music to the game's own files.

<p align="center">
  <a href="https://kelsidavis.bandcamp.com/album/wowee-soundtrack">
    <img src="assets/wowee-soundtrack.png" alt="WoWee Soundtrack cover" width="320" />
  </a>
</p>

[**WoWee Soundtrack**](https://kelsidavis.bandcamp.com/album/wowee-soundtrack) -
19 tracks, 56 minutes, available in lossless FLAC. The tracks are original
compositions and contain no Blizzard audio.

The files live in `assets/Original Music/`. Build trees and the debug APK carry
them; release archives, AppImages and the release APK leave them out. The music
is not covered by the MIT license; see [LICENSE](LICENSE).

## Documentation

### Getting started

- [Quick Start](docs/quickstart.md)
- [Complete Build Instructions](BUILD_INSTRUCTIONS.md)
- [Server Setup](docs/server-setup.md)
- [Project Status](docs/status.md)
- [Asset Manager](docs/asset-manager.md) and [Upgraded Assets](docs/upgraded-assets.md)

### Internals

- [Architecture](docs/architecture.md)
- [Authentication](docs/authentication.md) and [SRP Implementation](docs/srp-implementation.md)
- [Packet Framing](docs/packet-framing.md) and [Realm List](docs/realm-list.md)
- [Sky System](docs/SKY_SYSTEM.md)
- [Warden Quick Reference](docs/WARDEN_QUICK_REFERENCE.md) and [Implementation](docs/WARDEN_IMPLEMENTATION.md)

## Development and CI

WoWee uses C++20, CMake 3.15+, SDL3 and Vulkan 1.3. GitHub Actions builds
Linux x86-64/ARM64 AppImages, Windows x86-64/ARM64, macOS ARM64/x86-64 and Android
arm64 releases. Security checks include CodeQL, Semgrep, AddressSanitizer, and
UndefinedBehaviorSanitizer.

A native Linux build, with the packages listed in the
[build guide](BUILD_INSTRUCTIONS.md), is for development: it leaves
`build/bin/wowee`, and `container/appimage/build-appimage.sh build` turns that
build into an AppImage.

The codebase is split into focused rendering, networking, gameplay, asset, UI,
audio, and editor modules. Start with the [architecture guide](docs/architecture.md)
before making broad changes.

## Warden

Servers that run Warden read memory from the client's executable. The asset
builder caches each expansion's `WoW.exe` with its game data, in
`Data/expansions/<id>/misc`, and the client reads the one for the server being
played. Extract from the same client you would otherwise play with: an
executable of another build answers every memory check wrongly, which servers
treat as a modified client, and the client says so in its log.

A realm that signs its own Warden module needs its key set as
`wardenRsaModulus` in the expansion's `expansion.json`. To log in without
answering Warden at all, start the client with `WOWEE_WARDEN_ANSWER_HASH=0`:
the server then sends no checks, which most servers tolerate. See the
[Warden reference](docs/WARDEN_QUICK_REFERENCE.md).

## Known limitations

- The restored shadows toggle still needs live GPU validation. Turning shadows
  off disables sampling while retaining the shadow pass.
- Warden checks are graded against AzerothCore's list and build 5875 memory
  fixtures from CMaNGOS and vmangos. Set `WOWEE_TEST_CLASSIC_WOW_EXE` to a
  1.12.1 executable and run `test_warden_check_grader '[classic]'`. The supplied
  RetroWoW executable passes all 74 CMaNGOS checks and all 80 vmangos checks
  with the build 5875 runtime defaults initialized. These fixtures do not cover scripted scans
  or establish live realm compatibility.
  TBC build 8606 has 61 CMaNGOS memory fixtures: set `WOWEE_TEST_TBC_WOW_EXE`
  to a 2.4.3 executable and run `test_warden_check_grader '[tbc]'`. All 61 pass
  against the supplied enGB repack. Executables are not included in the repository.
  CMaNGOS TBC and WotLK scripted scans that locate system information and
  EndScene use addresses from CMaNGOS's source and have not been tested on a live realm.
- Where terrain meets a building, the character can occasionally take the wrong
  floor or stick. Enter `/unstuck` in chat to recover, press `F8` to write the
  floor data at your position to the log, and include the location and those
  log lines in a bug report.
- On Android the session drops a few seconds after the app goes to the
  background, because the system closes the socket of an app without a
  foreground service. The client returns to the login screen as it does on any
  disconnect.
- On a phone the interface starts at the largest UI Scale its frames still fit
  at, and is still small on a small screen. The client's own panels scale with
  display density, and **Window scale** under Interface → WoWee → Interface
  reaches 3x.

When reporting a bug, include the relevant client log lines, expansion, server
core, and reproduction steps.

## License and references

WoWee source code is available under the [MIT License with an additional
restriction](LICENSE): it may not be used, in whole or in part, as the basis
for or a component of a commercial video game or other commercial game
product without written permission. Original music and audio assets are not
covered by the MIT terms at all and are reserved - see [LICENSE](LICENSE) and
[NOTICE](NOTICE). World of Warcraft and its assets are property of Blizzard
Entertainment, Inc.

- [WoWDev Wiki](https://wowdev.wiki/) - file-format documentation
- [TrinityCore](https://github.com/TrinityCore/TrinityCore) - server reference
- [MaNGOS](https://github.com/cmangos/mangos-wotlk) - server reference
- [StormLib](https://github.com/ladislav-zezula/StormLib) - MPQ library
