# Features Overview

A comprehensive overview of all implemented features in Wowee, the native C++ World of Warcraft 3.3.5a client.

## Table of Contents
- [Rendering Features](#rendering-features)
- [Gameplay Features](#gameplay-features)
- [UI Features](#ui-features)
- [Network & Authentication](#network--authentication)
- [Asset Pipeline](#asset-pipeline)
- [Audio Features](#audio-features)
- [Developer Tools](#developer-tools)

---

## Rendering Features

### Terrain Rendering
- ✅ **Multi-tile streaming**: Load and render multiple ADT terrain tiles
- ✅ **Async loading**: Non-blocking terrain chunk loading (prevents frame stalls)
- ✅ **Height maps**: 9x9 outer + 8x8 inner vertex grids per chunk
- ✅ **Texture splatting**: Up to 4 texture layers per chunk with alpha blending
- ✅ **Frustum culling**: Only render visible terrain chunks
- ✅ **Terrain holes**: Support for gaps in terrain geometry

### Water & Liquids
- ✅ **Animated water**: Vertex animation with wave motion
- ✅ **Reflections**: Sky and environment reflections on water surface
- ✅ **Refractions**: Underwater view distortion
- ✅ **Fresnel effect**: View-angle dependent reflection/refraction mixing
- ✅ **Multiple liquid types**: Water, lava, slime support
- ✅ **Liquid height**: Variable water levels per chunk

### Sky & Atmosphere
- ✅ **Dynamic day/night cycle**: Smooth time-of-day transitions
- ✅ **Sun and moon**: Orbital movement with proper positioning
- ✅ **Moon phases**: 8 realistic lunar phases with elliptical terminator
- ✅ **Star field**: 1000+ procedurally placed stars (visible at night)
- ✅ **Gradient sky**: Time-dependent sky color gradients
- ✅ **Manual time control**: F9 to pause, +/- to adjust time

### Weather & Particles
- ✅ **Rain system**: 2000 particle rain with gravity and wind
- ✅ **Snow system**: 2000 particle snow with drift and accumulation
- ✅ **Camera-relative particles**: Particles follow camera for consistent coverage
- ✅ **Weather cycling**: W key to cycle None/Rain/Snow
- ✅ **M2 particle emitters**: WotLK-compatible particle emitters on models

### Characters & Models
- ✅ **M2 model loading**: Character, creature, and prop models
- ✅ **Skeletal animation**: Up to 256 bones per model with GPU skinning
- ✅ **Animation sequences**: Idle, walk, run, attack, death, emote, etc.
- ✅ **Multiple animations**: Smooth animation blending
- ✅ **Geosets**: Show/hide body parts (e.g., helmets, cloaks)
- ✅ **Attachment points**: Weapons, shields, helmets properly attached
- ✅ **Race-aware textures**: Correct skin, hair, and face textures for all races
- ✅ **All races supported**: Human, Dwarf, Night Elf, Gnome, Draenei, Orc, Undead, Tauren, Troll, Blood Elf

### Buildings & World Objects
- ✅ **WMO rendering**: World Map Object (building) rendering
- ✅ **Multi-material batches**: Multiple textures per building
- ✅ **Portal system**: Visibility culling using portals
- ✅ **Doodad placement**: Decorative objects within buildings
- ✅ **Distance culling**: WMO groups culled beyond 160 units
- ✅ **Frustum culling**: Only render visible building groups

### Visual Effects
- ✅ **Lens flare**: Chromatic aberration sun lens flare
- ✅ **Volumetric clouds**: FBM noise-based cloud generation
- ✅ **Glow effects**: Billboarded light sprites for lanterns, torches
- ✅ **Blend modes**: Proper alpha, additive, and modulate blending
- ✅ **Unlit rendering**: Separate shader path for emissive materials

### Post-Processing
- ✅ **HDR rendering**: High dynamic range rendering pipeline
- ✅ **Tonemapping**: Exposure adjustment and color grading
- ✅ **Shadow mapping**: 2048x2048 shadow maps
- ✅ **MSAA support**: Multi-sample anti-aliasing

### Camera
- ✅ **Free-fly camera**: WASD movement with mouse look
- ✅ **Camera orbit**: Orbit around character in online mode
- ✅ **Sprint**: Shift key for faster camera movement
- ✅ **Smooth movement**: Interpolated camera transitions
- ✅ **Intro animations**: Camera swoops on creature spawns

---

## Gameplay Features

### Authentication & Login
- ✅ **SRP6a authentication**: Full cryptographic authentication
- ✅ **Username/password**: Standard login credentials
- ✅ **Server selection**: Configure auth server address
- ✅ **Session keys**: 40-byte session key generation
- ✅ **RC4 encryption**: Header encryption after authentication
- ✅ **Auto-connect**: Reconnect on disconnect

### Realm & Character Selection
- ✅ **Realm list**: Display available realms from auth server
- ✅ **Realm info**: Population, type (PvP/PvE/RP), online status
- ✅ **Auto-select**: Auto-select when only one realm available
- ✅ **Character list**: Display all characters on selected realm
- ✅ **Character preview**: 3D animated character preview
- ✅ **Character stats**: Level, race, class, location display
- ✅ **Auto-select character**: Auto-select when only one character exists

### Character Creation
- ✅ **Race selection**: All 10 playable races (Alliance and Horde)
- ✅ **Class selection**: Classes filtered by race compatibility
- ✅ **Gender selection**: Male and female options
- ✅ **Appearance customization**: Face, skin color, hair style, hair color, facial features
- ✅ **Name validation**: Character name input and validation
- ✅ **3D preview**: Real-time 3D character preview during creation
- ✅ **Character deletion**: Delete existing characters

### Movement & Navigation
- ✅ **WASD movement**: Smooth character movement
- ✅ **Mouse steering**: Camera-relative movement direction
- ✅ **Sprint**: Shift key for faster movement
- ✅ **Server sync**: Position synchronized with world server
- ✅ **Spline movement**: Follow server-dictated movement paths
- ✅ **Fall time**: Proper gravity and fall damage calculation
- ✅ **Collision**: Terrain height-based positioning

### Combat System
- ✅ **Targeting**: Left-click or Tab to target enemies
- ✅ **Auto-attack**: Automatic melee attacks on targeted enemy
- ✅ **Spell casting**: Cast spells from spellbook or action bar
- ✅ **Spell targeting**: Click-to-target and self-cast support
- ✅ **Cooldowns**: Visual cooldown indicators on action bar
- ✅ **Resource costs**: Mana, rage, energy consumption
- ✅ **Damage calculation**: Server-side damage processing
- ✅ **Attack animations**: Proper melee and spell cast animations
- ✅ **Death handling**: Player death, corpse, resurrection
- ✅ **NPC combat**: NPCs attack and animate properly
- ✅ **Faction hostility**: Faction.dbc-based friend/foe detection
- ✅ **Level-based difficulty**: Color-coded enemy levels (gray/green/yellow/orange/red)

### Spells & Abilities
- ✅ **Spellbook**: Complete spellbook UI
- ✅ **Class specialties**: Tabs organized by SkillLine (e.g., Fire, Frost, Arcane for Mage)
- ✅ **General tab**: Universal spells available to all classes
- ✅ **Spell icons**: Loaded from SpellIcon.dbc
- ✅ **Spell tooltips**: Name, rank, cost, range, cooldown, description
- ✅ **Drag-drop**: Drag spells to action bar
- ✅ **Known spells**: Track learned spells per character
- ✅ **Spell ranks**: Multiple ranks per spell

### Action Bar
- ✅ **12 slots**: Main action bar with 12 ability slots
- ✅ **Keybindings**: 1-9, 0, -, = for quick access
- ✅ **Spell icons**: Visual spell icons with cooldown overlay
- ✅ **Item icons**: Visual item icons for usable items
- ✅ **Drag-drop**: Drag spells from spellbook or items from inventory
- ✅ **Click-to-cast**: Left-click to use ability
- ✅ **Drag-to-remove**: Drag to ground to remove from action bar
- ✅ **Cooldown display**: Visual cooldown timer
- ✅ **Hotkey labels**: Show keybinding on each slot

### Inventory & Equipment
- ✅ **23 equipment slots**: Head, shoulders, chest, legs, feet, wrist, hands, waist, back, mainhand, offhand, ranged, ammo, tabard, shirt, trinkets, rings, neck
- ✅ **16 backpack slots**: Main backpack storage
- ✅ **Item icons**: Rich item icons from ItemDisplayInfo.dbc
- ✅ **Item tooltips**: Stats, durability, item level, required level
- ✅ **Drag-drop**: Drag items to equip, unequip, or action bar
- ✅ **Auto-equip**: Double-click or drag to auto-equip to correct slot
- ✅ **Gold display**: Current gold/silver/copper
- ✅ **Item stats**: Armor, damage, stats, enchantments
- ✅ **Durability**: Equipment durability tracking

### Quest System
- ✅ **Quest markers**: ! (available) and ? (complete) above NPCs
- ✅ **Minimap markers**: Quest markers on minimap
- ✅ **Quest details**: Rich quest description and objectives
- ✅ **Quest log**: Track up to 25 active quests
- ✅ **Objective tracking**: Progress on kill/collect objectives
- ✅ **Quest rewards**: Choose reward items on completion
- ✅ **Quest turn-in**: Complete quest workflow
- ✅ **Quest levels**: Color-coded quest difficulty
- ✅ **Abandon quests**: Drop unwanted quests
- ✅ **Quest giver status**: Dynamic ! and ? based on quest state

### Vendor System
- ✅ **Buy items**: Purchase items with gold
- ✅ **Sell items**: Sell items from inventory
- ✅ **Vendor inventory**: Display vendor's available items
- ✅ **Price display**: Gold/silver/copper prices
- ✅ **Gold validation**: Prevent buying without enough gold
- ✅ **Inventory space check**: Verify space before buying
- ✅ **Multi-buy**: Purchase multiple quantities

### Loot System
- ✅ **Loot window**: Visual loot interface
- ✅ **Loot corpses**: Loot from defeated enemies
- ✅ **Gold looting**: Pick up gold from corpses
- ✅ **Item looting**: Add looted items to inventory
- ✅ **Loot all**: Take all items and gold
- ✅ **Auto-loot**: Shift-click for instant loot (TODO)

### NPC Interaction
- ✅ **Gossip system**: Talk to NPCs with dialogue options
- ✅ **Gossip options**: Multiple choice dialogues
- ✅ **Quest givers**: NPCs offer quests
- ✅ **Vendors**: NPCs sell and buy items
- ✅ **Trainers**: NPCs offer training (placeholder)
- ✅ **Flight masters**: Flight points (TODO)

### Social Features
- ✅ **Chat system**: Send and receive chat messages
- ✅ **Chat channels**: SAY, YELL, WHISPER, GUILD (TODO)
- ✅ **Chat formatting**: Color-coded messages by type
- ✅ **Chat history**: Scrollable message history
- ✅ **Party system**: Group invites and party list
- ✅ **Friend list**: (TODO)
- ✅ **Guild system**: (TODO)

### Character Progression
- ✅ **Experience points**: Gain XP from kills and quests
- ✅ **Leveling**: Level up with stat increases
- ✅ **Level-based XP**: Appropriate XP from mob kills
- ✅ **Stats tracking**: Health, mana, strength, agility, etc.
- ✅ **Talent points**: (TODO - placeholder screen exists)
- ✅ **Talent trees**: (TODO)

---

## UI Features

### Authentication Screen
- ✅ Username input field
- ✅ Password input field (masked)
- ✅ Server address input
- ✅ Login button
- ✅ Connection status display
- ✅ Error message display

### Realm Selection Screen
- ✅ Realm list with names
- ✅ Realm type indicators (PvP/PvE/RP)
- ✅ Population display (Low/Medium/High/Full)
- ✅ Online/offline status
- ✅ Select button
- ✅ Back to login button

### Character Selection Screen
- ✅ Character list with names and levels
- ✅ 3D animated character preview
- ✅ Race, class, level display
- ✅ Location display
- ✅ Create Character button
- ✅ Delete Character button
- ✅ Enter World button
- ✅ Back to realms button

### Character Creation Screen
- ✅ Race selection buttons (all 10 races)
- ✅ Class selection buttons (filtered by race)
- ✅ Gender selection (male/female)
- ✅ Face selection
- ✅ Skin color selection
- ✅ Hair style selection
- ✅ Hair color selection
- ✅ Facial features selection
- ✅ Character name input
- ✅ 3D character preview (updates in real-time)
- ✅ Create button
- ✅ Back button

### In-Game HUD
- ✅ Player health bar
- ✅ Player mana/rage/energy bar
- ✅ Player level and name
- ✅ Target frame (health, level, name)
- ✅ Target hostility coloring (red=hostile, green=friendly, yellow=neutral)
- ✅ Experience bar
- ✅ Action bar (12 slots with icons and keybindings)
- ✅ Minimap
- ✅ Chat window
- ✅ Gold display
- ✅ Coordinates display (debug)

### Inventory Window
- ✅ Paper doll (equipment visualization)
- ✅ 23 equipment slots
- ✅ Backpack grid (16 slots)
- ✅ Item icons with tooltips
- ✅ Gold display
- ✅ Character stats panel
- ✅ Close button

### Spellbook Window
- ✅ Tabbed interface (class specs + General)
- ✅ Spell list with icons
- ✅ Spell names and ranks
- ✅ Spell tooltips (cost, range, cooldown, description)
- ✅ Drag-drop to action bar
- ✅ Tab switching
- ✅ Close button

### Quest Log Window
- ✅ Quest list (up to 25 quests)
- ✅ Quest objectives and progress
- ✅ Quest description
- ✅ Quest level and recommended party size
- ✅ Abandon Quest button
- ✅ Close button

### Quest Details Dialog
- ✅ Quest title and level
- ✅ Quest description text
- ✅ Objectives list
- ✅ Reward display (gold, items, XP)
- ✅ Reward choice (select one of multiple rewards)
- ✅ Accept button
- ✅ Decline button

### Vendor Window
- ✅ Vendor item list with icons
- ✅ Item prices (gold/silver/copper)
- ✅ Buy button per item
- ✅ Sell tab (your items)
- ✅ Buyback tab (TODO)
- ✅ Close button

### Loot Window
- ✅ Loot item list with icons
- ✅ Item names and quantities
- ✅ Gold display
- ✅ Click to loot individual items
- ✅ Auto-close when empty

### Gossip Window
- ✅ NPC dialogue text
- ✅ Gossip option buttons
- ✅ Quest links
- ✅ Vendor button
- ✅ Trainer button
- ✅ Close button

### Talent Window (Placeholder)
- ✅ Talent tree visualization (TODO)
- ✅ Talent point allocation (TODO)
- ✅ Close button

### Settings Window
- ✅ UI opacity slider
- ✅ Graphics options (TODO)
- ✅ Audio volume controls (TODO)
- ✅ Keybinding customization (TODO)
- ✅ Close button

### Loading Screen
- ✅ Progress bar with percentage
- ✅ Map name display (TODO)
- ✅ Loading tips (TODO)
- ✅ Background image (TODO)

### Performance HUD (F1)
- ✅ FPS counter
- ✅ Frame time (ms)
- ✅ Draw calls
- ✅ Triangle count
- ✅ GPU usage %
- ✅ Memory usage
- ✅ Network stats (TODO)

### Minimap
- ✅ Player position indicator
- ✅ Player direction arrow
- ✅ Quest markers (! and ?)
- ✅ Zoom levels (TODO)
- ✅ Zone name (TODO)

---

## Network & Authentication

### Connection
- ✅ TCP socket abstraction
- ✅ Non-blocking I/O
- ✅ Packet framing (6-byte outgoing, 4-byte incoming headers)
- ✅ Automatic reconnection
- ✅ Connection status tracking

### Authentication (Port 3724)
- ✅ SRP6a protocol implementation
- ✅ LOGON_CHALLENGE packet
- ✅ LOGON_PROOF packet
- ✅ Session key generation (40 bytes)
- ✅ SHA1 hashing (OpenSSL)
- ✅ Big integer arithmetic
- ✅ Salt and verifier calculation
- ✅ Realm list retrieval

### World Server (Port 8085)
- ✅ SMSG_AUTH_CHALLENGE / CMSG_AUTH_SESSION
- ✅ RC4 header encryption
- ✅ 100+ packet types implemented
- ✅ Packet batching (multiple per frame)
- ✅ Opcode handling for WoW 3.3.5a (build 12340)

### Implemented Packets (Partial List)
- ✅ Character enumeration (SMSG_CHAR_ENUM)
- ✅ Character creation (CMSG_CHAR_CREATE)
- ✅ Character deletion (CMSG_CHAR_DELETE)
- ✅ Player login (CMSG_PLAYER_LOGIN)
- ✅ Movement (CMSG_MOVE_*)
- ✅ Chat messages (CMSG_MESSAGECHAT, SMSG_MESSAGECHAT)
- ✅ Quest packets (CMSG_QUESTGIVER_*, SMSG_QUESTGIVER_*)
- ✅ Vendor packets (CMSG_LIST_INVENTORY, CMSG_BUY_ITEM, CMSG_SELL_ITEM)
- ✅ Loot packets (CMSG_LOOT, CMSG_LOOT_MONEY)
- ✅ Spell packets (CMSG_CAST_SPELL, SMSG_SPELL_GO)
- ✅ Inventory packets (CMSG_SWAP_INV_ITEM, CMSG_AUTOEQUIP_ITEM)
- ✅ Gossip packets (CMSG_GOSSIP_HELLO, SMSG_GOSSIP_MESSAGE)
- ✅ Combat packets (CMSG_ATTACKSWING, SMSG_ATTACKERSTATEUPDATE)

---

## Asset Pipeline

### MPQ Archives
- ✅ StormLib integration
- ✅ Priority-based file loading (patches override base files)
- ✅ Locale support (enUS, enGB, deDE, frFR, etc.)
- ✅ File existence checking
- ✅ Data extraction with caching

### BLP Textures
- ✅ BLP format parser
- ✅ DXT1 decompression (opaque textures)
- ✅ DXT3 decompression (sharp alpha)
- ✅ DXT5 decompression (gradient alpha)
- ✅ Mipmap extraction
- ✅ OpenGL texture object creation
- ✅ Texture caching

### M2 Models
- ✅ M2 format parser (WotLK version)
- ✅ Vertex data (position, normal, texcoord, bone weights)
- ✅ Skeletal animation (256 bones max)
- ✅ Animation sequences (idle, walk, run, attack, etc.)
- ✅ Bone hierarchies and transforms
- ✅ Particle emitters (FBlock format)
- ✅ Render batches (multiple materials)
- ✅ Geosets (show/hide body parts)
- ✅ Attachment points (weapons, helmets, etc.)
- ✅ Texture animations
- ✅ Blend modes (opaque, alpha, additive, modulate)

### WMO Buildings
- ✅ WMO format parser
- ✅ Group-based rendering
- ✅ Multi-material batches
- ✅ Portal system (visibility culling)
- ✅ Doodad placement (decorative objects)
- ✅ Liquid data (indoor water)
- ✅ Vertex colors
- ✅ Distance culling (160 units)

### ADT Terrain
- ✅ ADT format parser
- ✅ 64x64 tile grid per continent
- ✅ 16x16 chunks per tile (MCNK)
- ✅ Height maps (9x9 + 8x8 vertices)
- ✅ Texture layers (up to 4 per chunk)
- ✅ Alpha blending between layers
- ✅ Liquid data (water, lava, slime)
- ✅ Terrain holes
- ✅ Object placement (M2 and WMO references)
- ✅ Async loading (prevents frame stalls)

### DBC Databases
- ✅ DBC format parser
- ✅ 20+ DBC files loaded at startup
- ✅ String block parsing
- ✅ Type-safe record access

**Loaded DBC Files:**
- ✅ Spell.dbc - Spell data
- ✅ SpellIcon.dbc - Spell icons
- ✅ Item.dbc - Item definitions
- ✅ ItemDisplayInfo.dbc - Item icons and models
- ✅ CreatureDisplayInfo.dbc - Creature appearances
- ✅ CreatureModelData.dbc - Creature model data
- ✅ ChrClasses.dbc - Class information
- ✅ ChrRaces.dbc - Race information
- ✅ SkillLine.dbc - Skill categories
- ✅ SkillLineAbility.dbc - Spell-to-skill mapping
- ✅ Faction.dbc - Faction and reputation data
- ✅ Map.dbc - Map definitions
- ✅ AreaTable.dbc - Zone and area names
- ✅ And more...

---

## Audio Features

### Music System
- ✅ Background music playback (FFmpeg)
- ✅ Music looping
- ✅ Zone-specific music (TODO)
- ✅ Combat music (TODO)

### Sound Effects
- ✅ Footstep sounds
- ✅ Surface-dependent footsteps (grass, stone, metal, etc.)
- ✅ Combat sounds (sword swing, spell cast)
- ✅ UI sounds (button clicks, window open/close)
- ✅ Environmental sounds (TODO)

### Audio Management
- ✅ FFmpeg integration
- ✅ Multiple audio channels
- ✅ Volume control (TODO - settings screen)
- ✅ 3D positional audio (TODO)

---

## Developer Tools

### Debug HUD (F1)
- ✅ FPS and frame time
- ✅ Draw call count
- ✅ Triangle count
- ✅ GPU usage
- ✅ Memory usage
- ✅ Camera position
- ✅ Player position
- ✅ Current map and zone

### Rendering Debug
- ✅ F2: Wireframe mode toggle
- ✅ F9: Pause time progression
- ✅ F10: Toggle sun/moon visibility
- ✅ F11: Toggle star field
- ✅ +/-: Manual time control
- ✅ C: Toggle clouds
- ✅ L: Toggle lens flare
- ✅ W: Cycle weather (None/Rain/Snow)

### Entity Spawning
- ✅ K: Spawn test character (M2 model)
- ✅ J: Remove test character
- ✅ O: Spawn test building (WMO)
- ✅ P: Clear all spawned WMOs

### Logging System
- ✅ Multi-level logging (DEBUG, INFO, WARNING, ERROR, FATAL)
- ✅ Timestamp formatting
- ✅ File output
- ✅ Console output
- ✅ Log rotation (truncate on startup)

### Performance Profiling
- ✅ Frame timing
- ✅ Draw call tracking
- ✅ Memory allocation tracking
- ✅ Network packet stats (TODO)

---

## Platform Support

### Currently Supported
- ✅ Linux (primary development platform)
- ✅ X11 window system
- ✅ OpenGL 3.3 Core Profile

### Planned Support
- ⏳ Windows (via Windows Subsystem)
- ⏳ macOS (via MoltenVK for Metal support)
- ⏳ Wayland (Linux)

---

## Technical Specifications

### Graphics
- **API**: OpenGL 3.3 Core Profile
- **Shaders**: GLSL 330
- **Rendering**: Forward rendering with post-processing
- **Shadow Maps**: 2048x2048 resolution
- **Particles**: 2000 max simultaneous (weather)
- **Bones**: 256 max per character (GPU skinning)

### Performance
- **Target FPS**: 60 (VSync enabled)
- **Typical Draw Calls**: 30-50 per frame
- **Typical Triangles**: 50,000-100,000 per frame
- **GPU Usage**: <10% on modern GPUs
- **Memory**: ~500MB-1GB (depends on loaded assets)

### Network
- **Protocol**: WoW 3.3.5a (build 12340)
- **Auth Port**: 3724
- **World Port**: 8085 (configurable)
- **Encryption**: RC4 (header encryption)
- **Authentication**: SRP6a

### Asset Requirements
- **WoW Version**: 3.3.5a (build 12340)
- **MPQ Files**: common.MPQ, expansion.MPQ, lichking.MPQ, patch*.MPQ
- **Locale**: enUS (or other supported locales)
- **Total Size**: ~15-20GB of game data

---

## Compatibility

### Server Compatibility
- ✅ TrinityCore (3.3.5a branch)
- ✅ MaNGOS (WotLK)
- ✅ AzerothCore
- ✅ CMaNGOS (WotLK)

### Build Systems
- ✅ CMake 3.15+
- ✅ GCC 7+ (C++17 support required)
- ✅ Clang 6+ (C++17 support required)

---

**Last Updated**: Based on commit `06fe167` - February 2026

For detailed change history, see [CHANGELOG.md](CHANGELOG.md).
For architecture details, see [docs/architecture.md](docs/architecture.md).
For quick start guide, see [docs/quickstart.md](docs/quickstart.md).
