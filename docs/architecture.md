# Architecture Overview

## System Design

Wowee follows a modular architecture with clear separation of concerns:

```
┌─────────────────────────────────────────────┐
│           Application (main loop)            │
│  - State management (auth/realms/game)      │
│  - Update cycle                             │
│  - Event dispatch                           │
└──────────────┬──────────────────────────────┘
               │
       ┌───────┴────────┐
       │                │
┌──────▼──────┐  ┌─────▼──────┐
│   Window    │  │   Input    │
│  (SDL3 +    │  │ (Keyboard/ │
│   Vulkan)   │  │ Mouse/Pad) │
└──────┬──────┘  └─────┬──────┘
       │                │
       └───────┬────────┘
               │
    ┌──────────┴──────────┐
    │                     │
┌───▼────────┐   ┌───────▼──────┐
│  Renderer  │   │  UI Manager  │
│  (Vulkan)  │   │  (ImGui) +   │
│            │   │   FrameXML   │
└───┬────────┘   └──────────────┘
    │
    ├─ Camera + CameraController
    ├─ TerrainRenderer (ADT streaming)
    ├─ WMORenderer (buildings, collision)
    ├─ M2Renderer (models, particles, ribbons)
    ├─ CharacterRenderer (skeletal animation)
    ├─ WaterRenderer (refraction, lava, slime)
    ├─ SkySystem (skybox, stars, clouds, lens flare) + Weather
    ├─ LightingManager (Light.dbc volumes)
    └─ SwimEffects, ChargeEffect, Lightning
```

## Core Systems

### 1. Application Layer (`src/core/`)

**Application** (`application.hpp/cpp`) - Main controller
- Owns all subsystems (renderer, game handler, asset manager, UI, addon manager, audio coordinator)
- Manages application state (`AUTHENTICATION` → `REALM_SELECTION` → `CHARACTER_SELECTION` / `CHARACTER_CREATION` → `IN_GAME`, plus `DISCONNECTED`, and `FIRST_RUN` when nothing has been extracted and the asset builder is shown instead of the login)
- Runs update/render loop, paced by `FramePacer` (`frame_pacer.hpp/cpp`): integer-nanosecond deltas off a monotonic clock, and a frame cap that sleeps most of the wait and spins the rest
- Populates `GameServices` struct and passes to `GameHandler` at construction

**Window** (`window.hpp/cpp`) - SDL3 + Vulkan wrapper
- Creates SDL3 window with Vulkan surface
- Owns `VkContext` (Vulkan device, swapchain, render passes)
- Handles resize events; carries the window size in points and the drawable size in pixels, which differ on high-density displays

**Input** (`input.hpp/cpp`) - Input management
- Keyboard state tracking (SDL scancodes)
- Mouse position, buttons (1-based SDL indices), wheel delta
- Per-frame delta calculation
- Virtual keys and mouse buttons, which the touch controls and the gamepad press on the player's behalf

**Gamepad** (`gamepad.hpp/cpp`) - SDL game controller device
- Open, hot-plug, radial stick deadzone, trigger and button state per frame
- `ui::GamepadControls` (`src/ui/gamepad_controls.cpp`) turns it into virtual keys, camera motion and a pointer. The default scheme (`src/ui/gamepad_bindings.cpp`) is seeded into the interface's key bindings, where it can be rebound

**Logger** (`logger.hpp/cpp`) - Thread-safe logging
- Multiple log levels (DEBUG, INFO, WARNING, ERROR, FATAL)
- File output to `logs/wowee.log` (`WOWEE_LOG_FILE` renames it), or a per-user log directory when the working directory is not writable
- Configurable via `WOWEE_LOG_LEVEL` env var

### 2. Rendering System (`src/rendering/`)

**Renderer** (`renderer.hpp/cpp`) - Main rendering coordinator
- Manages Vulkan pipeline state
- Coordinates frame rendering across all sub-renderers
- Owns camera, sky, weather, lighting, and all sub-renderers
- Shadow mapping with PCF filtering

**VkContext** (`vk_context.hpp/cpp`) - Vulkan infrastructure
- Vulkan 1.3 instance and device; synchronization2 barriers throughout
- Device selection, queue families, swapchain
- Render passes, framebuffers, command pools; the shadow pass records with dynamic rendering instead (`WOWEE_VK_NO_DYNAMIC_RENDERING=1` puts it back on its render pass)
- Sampler cache (FNV-1a hashed dedup)
- Pipeline cache persistence for fast startup

**Camera** (`camera.hpp/cpp`) - View/projection matrices
- Position and orientation
- FOV, aspect ratio, near/far planes
- Sub-pixel jitter for the temporal upscaler (column 2 NDC offset)
- Frustum extraction for culling

**TerrainManager / TerrainRenderer** - ADT terrain streaming and drawing
- Async tile loading on worker threads (`WOWEE_TERRAIN_WORKERS`) within configurable radius
- 4-layer texture splatting with alpha blending
- Frustum + distance culling
- Ground clutter placement from GroundEffectTexture/GroundEffectDoodad via deterministic RNG

**WMORenderer** - World Map Objects (buildings)
- Multi-material batch rendering
- Portal-based visibility culling
- Floor/wall collision (normal-based classification)
- Interior glass transparency, doodad placement

**M2Renderer** - Models (creatures, doodads, spell effects)
- Skeletal animation with GPU bone transforms
- Particle emitters (WotLK FBlock format)
- Ribbon emitters (charge trails, enchant glows)
- Portal spin effects, foliage wind displacement
- Per-instance animation state

**CharacterRenderer** - Player/NPC character models
- GPU vertex skinning (240 bones)
- Race/gender-aware textures via CharSections.dbc
- Equipment rendering (geoset visibility per slot)
- Fallback textures (white/transparent/flat-normal) for missing assets

**WaterRenderer** - Terrain and WMO water
- Refraction/reflection rendering
- Magma/slime with multi-octave FBM noise flow
- Beer-Lambert absorption

**SkySystem + Weather**
- Procedural sky gradient (fullscreen triangle) coloured from Light.dbc / LightIntBand.dbc, with the sky model Light.dbc names drawn over it where there is one (`WOWEE_NO_SKY_M2=1` turns the model off)
- Drawn after the terrain on the far plane, depth-tested, so hidden sky pixels are skipped
- Star field with day/night fade (dusk 18:00–20:00, dawn 04:00–06:00)
- Rain/snow particle systems per zone (via zone weather table)

**LightingManager** - Light.dbc volume sampling
- Time-of-day color bands (half-minutes, 0–2879)
- Distance-weighted light volume blending
- Fog color/distance parameters

**World Map System** (`src/rendering/world_map/`) - Modular map architecture:
- `WorldMapFacade` - Public API (PIMPL pattern), composes all components
- `CompositeRenderer` - Vulkan tile pipeline + off-screen FBO compositing (1024×768 FBO, 1002×668 visible)
- `DataRepository` - DBC zone loading, ZMP pixel map, POI/overlay storage
- `CoordinateProjection` - UV projection, zone/continent spatial lookups
- `ExplorationState` - Server exploration mask + local fog-of-war tracking
- `ViewStateMachine` - COSMIC → WORLD → CONTINENT → ZONE navigation with transitions
- `InputHandler` - Keyboard/mouse input → `InputAction` mapping
- `OverlayRenderer` - Layer-based ImGui overlay system (Open/Closed Principle)
- `MapResolver` - Cross-map navigation (Outland, Northrend detection)
- `ZoneMetadata` - Zone level ranges and faction data for labels
- 10 overlay layers (each implements `IOverlayLayer`): player marker, party dot, taxi node, POI marker, quest POI, rare tracker, corpse marker, zone highlight, coordinate display, subzone tooltip

### 3. Networking (`src/network/`)

**TCPSocket** (`tcp_socket.hpp/cpp`) - Platform TCP, used for the auth connection
- Non-blocking I/O, drained on each update
- 4 KB recv buffer per call
- Portable across Linux/macOS/Windows

**WorldSocket** (`world_socket.hpp/cpp`) - WoW world connection (derives from `Socket`, not `TCPSocket`)
- Header encryption chosen by build: RC4 keyed from the SRP session key (WotLK), or the XOR header cipher (`VanillaCrypt`) for Classic, keyed through HMAC-SHA1 for TBC
- Receives on a pump thread of its own (`WOWEE_NET_ASYNC_PUMP`, on by default); packets are parsed and dispatched on the main thread with configurable per-frame budgets

**Packet** (`packet.hpp/cpp`) - Binary data container
- Read/write primitives (uint8–uint64, float, string, packed GUID)
- Bounds-checked reads (return 0 past end)

### 4. Authentication (`src/auth/`)

**AuthHandler** - Auth server protocol (port 3724)
- SRP6a challenge/proof flow
- Security flags: PIN (0x01) and Authenticator (0x04); Matrix card (0x02) is detected and not supported
- Realm list retrieval

**SRP** (`srp.hpp/cpp`) - Secure Remote Password
- SRP6a with 19-byte (152-bit) ephemeral
- OpenSSL BIGNUM math
- Session key generation (40 bytes)

**Integrity** (`integrity.hpp/cpp`) - Client integrity verification
- The logon proof's `crc_hash`: `SHA1(A || HMAC-SHA1(checksumSalt, client files))`

### 5. Game Logic (`src/game/`)

**GameHandler** (`game_handler.hpp/cpp`) - Central game state
- Dispatch table routing server opcodes to domain handlers (registrations live in `game_handler_packets.cpp`)
- Owns all domain handlers via composition
- Receives dependencies via `GameServices` struct

**Domain Handlers** (SOLID decomposition from GameHandler):
- `EntityController` - UPDATE_OBJECT parsing, entity spawn/despawn
- `MovementHandler` - Movement packets (including SMSG_COMPRESSED_MOVES), speed, taxi, swimming, flying
- `CombatHandler` - Damage, healing, death, auto-attack, threat
- `SpellHandler` - Spell casting, cooldowns, auras, talents, pet spells
- `InventoryHandler` - Equipment, bags, bank, mail, auction, vendors, loot, trade
- `QuestHandler` - Quest accept/complete, objectives, progress tracking
- `SocialHandler` - Party, guild, LFG, friends, who, duel, inspect, arena and battleground
- `ChatHandler` - Chat messages, channels, emotes, system messages
- `WardenHandler` - Anti-cheat module management

**OpcodeTable** - Expansion-agnostic opcode mapping
- `LogicalOpcode` enum → wire opcode via JSON config per expansion
- Runtime remapping for Classic/TBC/WotLK/Turtle protocol differences

**Entity / EntityManager** - Entity lifecycle
- Shared entity base class with update fields (sparse index → uint32, `FlatFieldMap`)
- Player, Unit, GameObject subtypes
- GUID-based lookup, field extraction (health, level, display ID, etc.)

**TransportManager** - Transport lifecycle and server sync
- Delegates path data to `TransportPathRepository`
- Delegates spline math to `math::CatmullRomSpline`
- Clock-based motion with `TransportClockSync`
- Path data, clock sync and animation live in the extracted modules TransportPathRepository, TransportClockSync and TransportAnimator

**TransportPathRepository** - Transport path data
- DBC loading (TransportAnimation.dbc, TaxiPathNode.dbc); a taxi route (zeppelins, boats) is one journey on one clock that each map takes its own slice of
- Path inference heuristics for server spawn→DBC mapping
- Z-only elevator detection vs XY transport paths

**math::CatmullRomSpline** (`src/math/`) - Reusable spline module
- Catmull-Rom interpolation with O(log n) binary search segment lookup
- Fused position+tangent evaluation (single call per frame per transport)
- Time-closed (looping) and clamped (non-looping) path modes
- `orientationFromTangent()` for smooth transport/entity facing

**SplineBlockData** (`src/game/spline_packet.hpp/cpp`) - Unified spline parsing
- Consolidates 7 duplicated spline parsers into shared functions
- `parseMonsterMoveSplineBody()` (WotLK/TBC), `parseMonsterMoveSplineBodyVanilla()`
- `parseWotlkMoveUpdateSpline()`, `parseClassicMoveUpdateSpline()`
- Packed delta decoding (11+11+10-bit signed, ×0.25 scale)

**Expansion Helpers** (`game_utils.hpp`):
- `isActiveExpansion("classic")` / `isActiveExpansion("tbc")` / `isActiveExpansion("wotlk")`
- `isClassicLikeExpansion()` (Classic or Turtle WoW)
- `isPreWotlk()` (Classic, Turtle, or TBC)

### 6. Asset Pipeline (`src/pipeline/`)

**AssetManager** - Runtime asset access
- Extracted loose-file tree indexed by `manifest.json`. The data root is `WOW_DATA_PATH`, else the per-user data directory (`core::userDataRoot()` in `data_paths.hpp`) when it holds an extraction, else `./Data`; each installed game has its own tree under `expansions/<id>/`
- Resolution order: the `override/` directory (upgraded assets, which may add files as well as replace them), the active game's manifest, the root manifest as a base fallback (refused when it was extracted from a different client), then the loose file on disk
- File cache with configurable budget (256 MB min, 12 GB max)
- Texture sidecars: a `.dds` (block-compressed) and then a `.png` are tried before the `.blp`

**Asset builder** (`tools/asset_manager/`, `tools/asset_extract/`)
- `asset_extract` uses StormLib to extract MPQs into the data directory and generate `manifest.json`; driven by `extract_assets.sh` / `extract_assets.ps1`
- `wowee_assets` is the windowed builder: several games into one destination, upgrades from a later client (CASC or MPQ), packs saved and installed. See `docs/asset-manager.md`
- The client carries the same panel (`FirstRunScreen`) and opens it when nothing is extracted, or from "more options" on the login screen; compiled in where StormLib is found (`WOWEE_HAVE_ASSET_PANEL`)

**BLPLoader** - Texture decompression
- DXT1/3/5 block compression (RGB565 color endpoints)
- Palette mode with 1/4/8-bit alpha
- Mipmap extraction

**M2Loader** - Model binary parsing
- Version-aware header (Classic v256 vs WotLK v264)
- Skeletal animation tracks (embedded vs external .anim files, flag 0x20)
- Compressed quaternions (int16 offset mapping)
- Particle emitters, ribbon emitters, attachment points
- Geoset support (group × 100 + variant encoding)

**WMOLoader** - World object parsing
- Multi-group rendering with portal visibility
- Doodad placement (24-bit name index + 8-bit flags packing)
- Liquid data, collision geometry

**ADTLoader** - Terrain parsing
- 64×64 tiles per map, 16×16 chunks per tile (MCNK)
- MCVT height grid (145 vertices: 9 outer + 8 inner per row × 9 rows)
- Texture layers (up to 4 with alpha blending, RLE-compressed alpha maps)
- Async loading to prevent frame stalls

**DBCLoader** - Database table parsing
- Binary DBC format (fixed 4-byte uint32 fields + string block)
- CSV fallback for pre-extracted data
- Expansion-aware field layout via `dbc_layouts.json`
- 20+ DBC files: Spell, Item, Creature, Faction, Map, AreaTable, etc.

### 7. UI System (`src/ui/`)

**UIManager** - ImGui coordinator
- ImGui initialization with SDL3/Vulkan backend
- Screen state management and transitions
- Event handling and input routing

**FrameXML interface** (`src/addons/`, `src/ui/`) - Blizzard's own interface, run through this client's Lua engine and widget tree. In game it draws the unit frames, action bars, bags, chat, quest log, spellbook, talents and the rest. See `docs/widget-system.md`
- `AddonManager` - Loads FrameXML from its own manifest, then addons from their `.toc` files. `WOWEE_LOAD_FRAMEXML=0` turns FrameXML off
- `LuaEngine` - Lua 5.1 state and the WoW API bindings (`lua_engine.cpp`, `lua_*_api.cpp`)
- `WidgetTree` / `WidgetRenderer` - Retained frames, anchors and hit testing; drawn through ImGui draw lists with `Interface\` art
- `emitFrameXml` (`framexml_emitter.cpp`) - Turns FrameXML's XML into the Lua that builds it

**Screens:**
- `FirstRunScreen` - The asset builder's panel, shown when nothing has been extracted
- `AuthScreen` - Login card: server list, account, password, and the asset set when more than one is installed. "more options" holds the address and port, expansion, security code, and a way into the asset builder
- `RealmScreen` - Realm list with population and type indicators
- `CharacterScreen` - Character selection with 3D animated preview, keyboard navigation
- `CharacterCreateScreen` - Race/class/gender/appearance customization
- `GameScreen` - What the client still draws in game around FrameXML: nameplates, minimap markers, the world map, combat text and meters (`CombatUI`), dialogs, toasts, and the settings window
- `InventoryScreen` / `SpellbookScreen` - Item and spell data, icons and tooltips for the rest of the client; the windows themselves are FrameXML's
- `SettingsPanel` - This client's settings window, built from `settings_schema.cpp`: graphics presets (Low/Medium/High/Ultra), sound, controller. The same schema is registered as a category in FrameXML's Interface Options

**Chat System** (`src/ui/chat/`) - Modular chat architecture:

FrameXML owns the chat window itself - tabs, input, message display, and name
completion are the interface's, not this client's. What remains here is what
FrameXML calls into, plus what the client draws in the world.

- `ChatPanel` - Command registry host, bubbles, and the helpers FrameXML calls
  (`setWhisperTarget`, `insertChatLink`). Does not render a chat window
- `ChatCommandRegistry` - Slash command dispatch with `IChatCommand` interface.
  FrameXML's edit box routes unknown commands here via `runClientChatCommand`
- `ChatMarkupParser` / `ChatMarkupRenderer` - Item link parsing and colored rich-text rendering
- `ChatBubbleManager` - Floating chat bubbles above entities
- `ChatSettings` - Timestamps, font size, auto-joined channels, speech bubbles
- `MacroEvaluator` - WoW-style macro conditional evaluation (`[mod:shift]`, `[target=focus]`, etc.)
- `GameStateAdapter` / `InputModifierAdapter` - Testable abstractions over game state
- `ItemTooltipRenderer` - Chat-embedded item tooltip rendering
- `CastSequenceTracker` - `/castsequence` state tracking
- 11 command modules under `commands/`: channel, combat, emote, GM, group, guild, help, misc, social, system, target
- 206-command GM data table with dot-prefix interception and `/gmhelp`

### 8. Audio System (`src/audio/`)

**AudioCoordinator** - Owns the sound managers; owned by Application and fed a per-frame zone context by the Renderer

**AudioEngine** - miniaudio-based playback
- WAV decode cache (256 entries; half are dropped when it fills)
- 2D and 3D positional audio
- Sample rate preservation (explicit to avoid miniaudio pitch distortion)

**Sound Managers:**
- `AmbientSoundManager` - Wind, water, fire, birds, crickets, city ambience, bell tolls
- `ActivitySoundManager` - Swimming strokes, jumping, landing
- `FootstepManager` - Footsteps (terrain-aware), mount footsteps
- `MovementSoundManager` - Water entry and wading splashes
- `MountSoundManager` - Mount-specific movement audio
- `MusicManager` - Zone music (ZoneMusic.dbc day and night tracks pooled, plus this project's own soundtrack)
- `CombatSoundManager`, `SpellSoundManager`, `UiSoundManager`, `NpcVoiceManager`, `PlayerVoiceManager`

### 9. Warden Anti-Cheat (`src/game/`)

Components:
- `WardenHandler` - Packet handling (SMSG/CMSG_WARDEN_DATA); builds each check reply itself, from `WardenMemory` and `.cr` challenge/response files in the Warden cache directory
- `WardenCrypto` - RC4 keys for Warden traffic, derived from the SRP session key
- `WardenModule` - 8-step load pipeline: verify MD5, decrypt (RC4), verify the RSA-2048 signature, strip it and decompress (zlib), parse the module's own executable format, relocate, bind imports, initialize under the emulator. A failed step logs and continues
- `WardenEmulator` - Unicorn Engine x86 CPU emulation with Windows API interception
- `WardenMemory` - WoW.exe PE image loading with bounds-checked reads, runtime global patching

## Threading Model

See `docs/threading.md` for the full inventory.

- **Main thread**: Window events, game logic update, FrameXML Lua, rendering, frame pacing
- **Terrain workers**: `TerrainManager` worker threads load and prepare tiles
- **Network I/O**: `WorldSocket`'s pump thread receives; packets are dispatched on the main thread with per-frame budgets
- **Per-frame pool**: `ThreadPool` workers for bone animation, M2 visibility and secondary command buffer recording
- **Normal maps**: Background CPU generation with mutex-protected result queue
- **GPU uploads**: A second queue from the graphics family for parallel texture/buffer transfers, where the driver offers one (MoltenVK does not)

## Memory Management

- **Smart pointers**: `std::unique_ptr` / `std::shared_ptr` throughout
- **RAII**: All Vulkan resources wrapped with proper destructors
- **VMA**: Vulkan Memory Allocator for GPU memory
- **Object pooling**: Weather particles, combat text entries
- **DBC caching**: Lazy-loaded mutable caches in const getters

## Build System

**CMake** with modular targets:
- `wowee` - Main executable
- `wowee_assets` - Asset builder window (requires StormLib; the client carries the same panel when StormLib is found)
- `asset_extract` / `mpq_build` - MPQ extraction and packing tools (require StormLib)
- `dbc_to_csv` / `auth_probe` / `auth_login_probe` / `blp_convert` - Utility tools
- `framexml_run` (`WOWEE_BUILD_FRAMEXML_RUN`) and `wowee_editor` (`WOWEE_BUILD_EDITOR`) - Off the default build

**Dependencies:**
- SDL3, Vulkan SDK (1.3), OpenSSL, GLM, zlib (system; Android fetches SDL3 and GLM)
- ImGui, vk-bootstrap (submodules in extern/)
- VMA, stb_image, miniaudio, nlohmann/json, Lua 5.1.5 (vendored in extern/)
- StormLib (system, optional - for the asset tools and the client's asset builder)
- Unicorn Engine (system, optional - only for Warden emulation)
- FFmpeg (system, optional - for video playback)

**CI**: GitHub Actions for Linux (x86-64, ARM64), Windows (MSYS2, x86-64 and ARM64), macOS (ARM64, x86-64), Android (ARM64)
**Container builds**: Docker cross-compilation for Linux, macOS (osxcross), Windows (LLVM-MinGW)

## Code Style

- **C++20 standard**
- **Namespaces**: `wowee::core`, `wowee::rendering`, `wowee::rendering::world_map`, `wowee::game`, `wowee::ui` (the chat classes included), `wowee::addons`, `wowee::math`, `wowee::network`, `wowee::auth`, `wowee::audio`, `wowee::pipeline`
- **Naming**: PascalCase for classes, camelCase for functions/variables, kPascalCase for constants
- **Headers**: `.hpp` extension, `#pragma once`
- **Commits**: Prefixed with the area they touch (`ui:`, `m2:`, `assets:`, `vulkan:`, `changelog:`), short and factual
