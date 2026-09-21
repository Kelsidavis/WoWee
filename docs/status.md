# Project Status

**Last updated**: 2026-09-21

## What This Repo Is

Wowee is a native C++ World of Warcraft client experiment focused on connecting to real emulator servers (online/multiplayer) with a custom renderer and asset pipeline.

## Current Code State

Implemented (working in normal use):

- Auth flow: SRP6a auth + realm list + world connect with header encryption
- Login screen: a server list in the login card (ChromieCraft plus every saved server), with the address, port and expansion under "more options"; a once-per-start check against GitHub releases that shows a newer tag beside the version ("Check for new versions" in Interface turns it off)
- Interface: Blizzard's FrameXML, loaded from the player's extracted game data and drawn by the client, with addons from `Interface\AddOns`. The login, realm and character screens are the client's own. SimpleHTML item text pages (headings, paragraphs, links and images) render
- Assets: an asset builder shared by the client and the standalone `wowee_assets` window. The client opens it when nothing is extracted, and "more options" on the login screen reaches it afterwards
- Rendering: Vulkan 1.3, terrain, WMO/M2, water/magma/slime (FBM noise shaders), sky system, particles, shadow mapping, minimap/world map, loading video playback
- Instances: WDT parser, WMO-only dungeon maps, area trigger portals with glow/spin effects, zone transitions
- Character system: creation (including nonbinary gender), selection, 3D preview with equipment, character screen, per-instance NPC hair/skin textures
- Core gameplay: movement (with ACK responses), targeting (hostility-filtered tab-cycle), combat, action bar, inventory/equipment, chat (tabs/channels, emotes, item links)
- Quests: quest markers (! and ?) on NPCs/minimap, quest log with detail queries/retry, objective tracking, accept/complete flow, turn-in, quest item progress
- Trainers: spell trainer UI, buy spells, known/available/unavailable states
- Vendors, loot (including chest/gameobject loot), gossip dialogs (including buyback for most recently sold item)
- Bank: full bank support for all expansions, bag slots, drag-drop, right-click deposit
- Auction house: search with filters, pagination, sell picker, bid/buyout, tooltips
- Mail: item attachment support for sending
- Spellbook with specialty/general/profession/mount/companion tabs, drag-drop to action bar, spell icons, item use
- Talent tree UI with proper visuals and functionality
- Pet tracking (SMSG_PET_SPELLS), dismiss pet button
- Party: group invites, party list, out-of-range member health (SMSG_PARTY_MEMBER_STATS)
- Nameplates: NPC subtitles, guild names, elite/boss/rare borders, quest/raid indicators, cast bars, debuff dots
- Floating combat text: world-space damage/heal numbers above entities with 3D projection
- Target/focus frames: guild name, creature type, rank badges, combo points, cast bars
- Map exploration: subzone-level fog-of-war reveal
- Warden anti-cheat: full module execution via Unicorn Engine x86 emulation; module caching
- Audio: ambient, movement, combat, spell, and UI sound systems; NPC voice lines for all playable races (greeting/farewell/vendor/pissed/aggro/flee)
- Bags: FrameXML's container frames, or the bundled `WoweeAllBags` combined window (carried bags, keyring and bank, with a Sort button); "Separate bag windows" under Interface > Bags chooses between them
- Controllers: SDL3 gamepads on every platform, with buttons named per pad family (Xbox, PlayStation, Nintendo, Steam Deck and others), back paddles on action slots 7-10, a pointer mode on Back, and the scheme seeded into the game's Key Bindings panel
- DBC auto-detection: CharSections.dbc field layout auto-detected at runtime (handles stock WotLK vs HD-textured clients)
- Multi-expansion: Classic/Vanilla, TBC, WotLK, and Turtle WoW (1.18) protocol and asset variants
- CI: GitHub Actions for Linux (x86-64, ARM64), Windows (MSYS2 x86-64 + ARM64), macOS (ARM64, x86-64), Android (arm64); container builds via Podman

Recent refactors (PRs #59-63, April 2026):

- Chat system decomposed into 15+ modules under `src/ui/chat/` with 11 command modules, GM command support, macro evaluator, and tab completion
- World map decomposed into 16 modules under `src/rendering/world_map/` with overlay layer system, view state machine, and ZMP-based hover detection
- TransportManager decomposed: spline math extracted to `src/math/`, path data to TransportPathRepository, 7 duplicated spline parsers consolidated into `spline_packet.cpp`
- Spell visual effects system with bone-tracked ribbons and particles
- Entity movement improvements: multi-segment path interpolation, terrain height clamping, walk/run animation fix
- 31 unit-test suites (up from 8), covering chat, world map, spline math, transport, and animation systems
- Code quality fix pass: 7 issues resolved across hover detection, null safety, buffer bounds, and coordinate validation

Recent fixes (July 2026):

- Login pipeline hardened: login-critical opcodes have hardcoded fallback when opcode table lookup fails; OpcodeTable::loadFromJson() is now safe against failed reloads (issue #87)
- Integrity hash is build-aware: Classic-era DLLs only required for builds <=6005 or Turtle; TBC/WotLK hash only the .exe
- Strafing reworked: torso-twist via SpineLow bone rotation instead of dedicated strafe animations
- Camera smoothing snaps 1:1 during active drag/keyboard turn to reduce input lag
- Mount strafing uses MOUNT_RUN_LEFT/RIGHT when available

Recent work (August 2026):

- FrameXML interface transition: the original interface owns the chat window, and this client's own was removed along with the tab manager and completer that served it. The command registry, macro evaluation, and chat bubbles stay - FrameXML's edit box routes unknown slash commands into `runClientChatCommand`. The rest of the interface followed in v3.1.8; see `CHANGELOG.md`
- Warnings are errors: `WOWEE_WARNINGS_AS_ERRORS` (default ON) puts `-Werror` / `/WX` on the `wowee` target. Turn it off for a bisect or an unfamiliar compiler
- AMD FidelityFX SDK backends are off by default (`WOWEE_ENABLE_AMD_FSR2`, `WOWEE_ENABLE_AMD_FSR3_FRAMEGEN`). This client's own FSR 1 and `fsr2_*` compute shaders are in-tree and unaffected
- 89 test suites registered with CTest, up from the 31 noted above
- macOS: SIGPIPE is ignored at startup, so a send to a dropped connection no longer terminates the client; crash backtraces now work there as well as on Linux

Recent work (September 2026, v3.1.20-v3.1.32):

- Moved from SDL2 to SDL3, desktop and Android both (Android fetches SDL3 `release-3.2.24` with the build)
- Vulkan 1.3 is required: synchronization2 and dynamic rendering are core. The shadow pass is the first pass recorded with dynamic rendering; `WOWEE_VK_NO_DYNAMIC_RENDERING=1` puts it back on its render pass
- Frame pacing in integer nanoseconds off a monotonic clock: the pacer sleeps to within a measured margin and spins the rest
- Asset tooling: the Python asset pipeline GUI and CASC scripts were replaced by the native `wowee_assets` window, whose panel is also built into the client. Several games can be built into one data folder, packs can be saved and installed, and models can be imported from a Cataclysm or Legion install. See `docs/asset-manager.md` and `docs/upgraded-assets.md`
- The client and `wowee_assets` share one per-user data directory: `~/Library/Application Support/Wowee/Data` (macOS), `%LOCALAPPDATA%\Wowee\Data` (Windows), `$XDG_DATA_HOME/wowee/Data` or `~/.local/share/wowee/Data` (Linux). `WOW_DATA_PATH` still wins
- Controller support, the server list on the login card, and the update check (see above)
- The game's own Video, Sound and most Interface pages were folded into this client's settings schema, which is hosted as WoWee pages inside those same panels
- `ctest -N` lists 214 tests, up from the 89 test suites noted above

In progress / known gaps:

- World map: zone hover detection has edge cases with some zone boundaries
- Transports: M2 transports (trams) working with position-delta riding; WMO transports (ships, zeppelins) working with path following; some edge cases remain
- Quest GO interaction: CMSG_GAMEOBJ_USE + CMSG_LOOT sent correctly, but some AzerothCore/ChromieCraft servers don't grant quest credit for chest-type GOs (server-side limitation)
- Visual edge cases: some M2/WMO rendering gaps (some particle effects)
- Water refraction: always on and no longer a setting; srcAccessMask barrier fix (2026-03-18) resolved prior VK_ERROR_DEVICE_LOST on AMD/Mali GPUs
- Shadows: held on, with no settings control. Turning them off loses the device, and the fault is not found yet
- Realm patch directories: per-realm directory naming and patch-manifest validation exist (`src/core/realm_patches.cpp`), but nothing downloads or calls them yet
- Fixed 2026-05-15: classic/turtle update-field tables had multiple wrong indices (`UNIT_FIELD_BYTES_1`=133 colliding with `UNIT_FIELD_MOUNTDISPLAYID`=133; STAT0..4 at 138..142; RESISTANCES at 154; missing NATIVEDISPLAYID). Corrected against vmangos `UpdateFields_1_12_1.h`: BYTES_1=138, STAT0..4=150..154, RESISTANCES=155, added NATIVEDISPLAYID=132.

## Where To Look

- Entry point: `src/main.cpp`, `src/core/application.cpp`
- Networking/auth: `src/auth/`, `src/network/`, `src/game/game_handler.cpp`
- Rendering: `src/rendering/`
- Interface: `src/ui/` (FrameXML emitter, widget tree and renderer, login screens, settings schema), `src/addons/` (Lua API)
- Assets/extraction: `tools/asset_manager/` (the builder panel and `wowee_assets`), `tools/asset_extract/`, `extract_assets.sh`, `src/pipeline/asset_manager.cpp`, `include/core/data_paths.hpp`
