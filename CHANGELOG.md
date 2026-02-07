# Changelog

All notable changes to the Wowee project are documented here.

## Recent Development (2024-2026)

### Architecture Changes
- **Removed single-player mode**: Removed offline/single-player functionality to focus exclusively on multiplayer. This includes removal of SQLite persistence, local combat simulation, and all single-player UI elements.

### Quest System
- **Quest markers**: Added ! (quest available) and ? (quest complete) markers above NPCs
- **Minimap integration**: Quest markers now appear on minimap for easy navigation
- **Quest log**: Full quest log UI with objectives, progress tracking, and rewards
- **Quest details dialog**: Rich quest details window with description and objectives
- **Quest turn-in flow**: Complete quest workflow from accept to turn-in with reward selection
- **Quest giver status**: Automatic CMSG_QUESTGIVER_STATUS_QUERY when NPCs spawn
- **Status re-query**: Re-query quest status after accepting or completing quests

### Spellbook & Action Bar
- **Class specialty tabs**: Spellbook organized by SkillLine specialties using SkillLine.dbc and SkillLineAbility.dbc
- **General tab**: Separate tab for universal spells
- **Spell icons**: Loaded from SpellIcon.dbc with proper rendering
- **Drag-drop system**: Drag spells from spellbook to action bar slots
- **Click-to-cast**: Click action bar slots to cast spells
- **Spell targeting**: Proper spell targeting implementation
- **Error messages**: Clear error messages for spell cast failures
- **Cooldown tracking**: Visual cooldown indicators on action bar
- **Keybindings**: 1-9, 0, -, = for quick action bar access
- **Window behavior**: Fixed spellbook window dragging and escape-from-bounds issues

### Inventory & Equipment
- **Equipment slots**: 23 slots (head, shoulders, chest, legs, feet, wrist, hands, waist, back, mainhand, offhand, ranged, etc.)
- **Backpack**: 16-slot backpack storage
- **Item icons**: Loaded from ItemDisplayInfo.dbc
- **Drag-drop**: Drag items between inventory, equipment, and action bar
- **Auto-equip**: Automatic equipment slot detection and equipping
- **Item tooltips**: Rich tooltips with item stats and information
- **Online sync**: Proper GUID resolution and inventory enrichment
- **Slot mapping**: Fixed online equipment slot mapping and backpack offsets

### Vendor System
- **Buy items**: Purchase items from vendors with gold
- **Sell items**: Sell items back to vendors (online and offline)
- **Gold tracking**: Proper coinage field (PLAYER_FIELD_COINAGE at index 1170)
- **Inventory errors**: Handle sell/inventory errors gracefully
- **UI improvements**: Clean vendor interface with item lists

### Loot System
- **Loot window**: Visual loot window with item icons
- **Gold looting**: CMSG_LOOT_MONEY packet for online gold pickup
- **Item pickup**: Automatic item transfer to inventory
- **Corpse looting**: Loot from defeated enemies

### Combat System
- **Auto-attack**: Automatic attack on targeted enemies
- **Spell casting**: Full spell casting with resource costs (mana/rage/energy)
- **Attack animations**: Proper NPC and player attack animations
- **Damage calculation**: Server-side damage processing
- **Death handling**: Player death, corpse creation, resurrection
- **Faction hostility**: Faction.dbc-based hostility using base reputation
- **Race-aware factions**: Proper faction checking for all player races
- **Neutral-flagged hostile**: Support for neutral-flagged hostile mobs (Monster faction group)
- **Level-based coloring**: WoW-canonical mob level colors (gray, green, yellow, orange, red)

### Character System
- **Character creation**: Full creation flow with race, class, gender, appearance
- **Character screen**: 3D animated character preview
- **Stats panel**: Display level, race, class, location on character screen
- **Model preview**: 3D character model on creation and selection screens
- **All races**: Support for all Alliance and Horde races
- **Texture support**: Race-aware skin, hair, and feature textures
- **Auto-select**: Auto-select single realm or single character
- **Logout cleanup**: Clear character state on logout to prevent stale models

### M2 Model Rendering
- **Particle emitters**: Enabled M2 particle emitters with WotLK struct parsing
- **FBlock format**: Correct FBlock format and struct size for particle data
- **Safety caps**: Overflow guards and safety caps for emitter parameters
- **Glow rendering**: Billboarded light sprites for M2 glow batches
- **Blend modes**: Skip additive/mod blend batches for correct rendering
- **Unlit rendering**: Unlit shader path for glow and additive batches
- **Lantern glow**: Fixed lantern and torch glow rendering

### NPCs & Gossip
- **Gossip system**: NPC dialogue with options
- **Gossip packets**: CMSG_GOSSIP_SELECT_OPTION with proper opcode
- **Duplicate prevention**: Clear gossip options before re-parsing
- **Reopen guard**: Prevent gossip window conflicts
- **Combat animations**: NPCs play combat animations during attacks
- **Creature spawning**: Camera intro animation on all creature spawns
- **Display lookups**: Pre-load CreatureDisplayInfo DBC at startup

### Movement & Navigation
- **WASD movement**: Smooth WASD character movement
- **Camera orbit**: Mouse-based camera orbit around character
- **Spline movement**: Follow server-side spline paths
- **Fall time**: Correct movement packet format (unconditional fallTime write)
- **Position updates**: Smooth position interpolation
- **Respawn handling**: Fixed respawned corpse movement

### Terrain & World
- **Async loading**: Asynchronous terrain streaming to prevent hang
- **Streaming loop**: Fixed terrain streaming loop for continuous loading
- **Multi-tile support**: Load multiple ADT tiles simultaneously
- **Auto-load**: Load terrain as player moves through world
- **Height maps**: Proper height calculation for player positioning

### UI Improvements
- **Loading screen**: Loading screen with progress bar during world entry
- **Resize handling**: Proper resize handling during loading
- **Progress tracking**: Visual progress percentage
- **UI opacity**: Slider to adjust UI opacity in settings
- **ImGui frame conflict**: Fixed ImGui frame management issues
- **Popup positioning**: Fixed popup window positioning
- **ID conflicts**: Resolved ImGui widget ID conflicts
- **Both-button clicks**: Suppress simultaneous left+right mouse clicks
- **Player name display**: Fixed player name rendering in UI
- **Target frame**: Display targeted entity name, level, health

### Chat System
- **Chat window**: Scrollable chat window with message history
- **Message formatting**: Proper chat message formatting with colors
- **Chat commands**: /say, /yell, /whisper support
- **Chat input**: Press Enter to open chat, type and send

### Minimap
- **Quest markers**: Show quest givers and turn-in NPCs
- **Player position**: Display player position and direction
- **Zoom**: Minimap zoom levels

### Rendering Improvements
- **WMO distance cull**: Increased WMO group distance cull from 80 to 160 units
- **Hair textures**: Fixed hair texture loading and rendering
- **Skin textures**: Proper skin color and texture application
- **Action bar icons**: Spell and item icons on action bar
- **Targeting visuals**: Visual targeting indicators

### Network & Protocol
- **Opcode fixes**: Corrected loot and gossip opcodes for 3.3.5a
- **Movement packets**: Fixed MOVE_* packet format
- **Quest opcodes**: Added quest-related opcodes
- **Item queries**: Proper CMSG_ITEM_QUERY_SINGLE parsing
- **Sell item packets**: CMSG_SELL_ITEM with correct uint32 count

### Performance
- **Frame stalls**: Eliminated stalls from terrain loading (async)
- **Startup optimization**: Load DBC lookups at startup
- **Log truncation**: Truncate log file on start to prevent bloat
- **Memory efficiency**: Proper cleanup and resource management

### Bug Fixes
- **Vendor bugs**: Fixed vendor gold calculation and item display
- **Loot bugs**: Fixed loot window showing incorrect items
- **Hair bugs**: Fixed hair texture selection and rendering
- **Critter hostility**: Fixed neutral critters not being attackable
- **Faction bugs**: Fixed Monster faction group bit (use 8 not 4)
- **XP calculation**: Proper level-based XP from mob kills
- **Respawn bugs**: Fixed corpse movement after respawn
- **Camera bugs**: Fixed camera orbit and deselect behavior
- **Spell targeting**: Fixed spell targeting for ranged abilities
- **Action bar**: Fixed drag-drop and right-click removal
- **Character screen**: Various character screen display bugs
- **Stale models**: Prevent stale player model across logins

### Single-Player Mode
- **Offline play**: Full offline mode without server
- **Local persistence**: SQLite-based character and settings storage
- **Simulated combat**: Local XP and damage calculation
- **Settings sync**: Save and load settings locally

## Future Roadmap

### Planned Features
- Talent system implementation
- Guild system
- Auction house
- Mail system
- Crafting and professions
- Achievements
- Dungeon finder
- Battlegrounds and PvP
- Mount system
- Pet system

### Rendering Improvements
- LOD (Level of Detail) system
- Improved shadow quality
- SSAO (Screen Space Ambient Occlusion)
- Better water caustics
- Improved particle effects

### Performance Optimizations
- Multi-threaded asset loading
- Occlusion culling improvements
- Texture compression
- Model instancing
- Shader optimizations

### Quality of Life
- Keybinding customization UI
- Graphics settings menu
- Audio volume controls
- Addon support
- Macros
- UI customization

---

See [GitHub commit history](https://github.com/yourname/wowee/commits) for detailed commit messages and technical changes.
