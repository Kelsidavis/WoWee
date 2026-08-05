# Changelog

## [v2.0.38-preview] — 2026-08-05

### Fixed
- **A rejected teleport left the server discarding every movement packet after it.** `handleTeleportAck` refused any teleport whose destination looked "near origin" on Eastern Kingdoms and returned without acknowledging it — and an unacknowledged teleport means the server drops all movement from that point on. The test was wrong twice over: canonical coordinates swap x and y, and the box it drew covered Southshore
- **A creature that failed to spawn for five seconds was lost for good.** The spawn queue retries for a five-second window and then abandons the entry, and nothing ever asks again — the server does not re-send an object already in range. Walking out of the zone and back is what made them appear, which is why they turned up on zoning and not before
- **The minimap zone name came from the server's last announcement.** `SMSG_INIT_WORLD_STATES` is sent when the server notices a zone change and at no other time, and the label read that first with the terrain under the player only as a fallback — so it stayed on the last announced zone while the player walked out of it
- **The client no longer switches talent spec on its own say-so.** Switching spec is a spell cast, not a message: AzerothCore reads `CMSG_SET_ACTIVE_TALENT_GROUP_OBSOLETE` and does nothing, and what moves a player between specs is a spell effect cast at themselves. This sent the dead opcode and then set the active spec locally anyway, so the client believed it was on the second spec while the server had never heard of it
- **Hiding your helm no longer leaves you bald wearing nothing.** The world geoset build asked whether a helm is *equipped*; the show-helm toggle answers whether one is *shown*. So the branch that drops the hair scalp and fits the bald cap went on running with no helm over it
- **The breath bar goes away when it refills.** Surfacing does not stop the timer — the server sends one update and then nothing until its own counter reaches full seconds later — so the bar sat at a hundred percent until the stop arrived
- **The action bar redraws when it changes.** `ACTIONBAR_SLOT_CHANGED` was fired with no argument from two of its three sites, and the button reads `arg1 == 0 or arg1 == tonumber(self.action)` where zero means every slot. Nil matched neither, so not one button redrew — including when the whole bar arrived from the server
- **A quest that progresses can be auto-watched again.** `QUEST_WATCH_UPDATE` was wrong at all three sites: two carried nothing and the third carried a quest id where the interface reads a quest *log index*, which it hands straight to `GetNumQuestLeaderBoards` and `AddQuestWatch`
- **`CVAR_UPDATE` carries the CVar's label, not its name.** The two are different spellings of the same setting and FrameXML uses both two lines apart, so firing the name meant every consumer compared a camelCase name against an upper-case label and took the other branch — silently. The health and mana numbers on unit frames never appeared or disappeared, the free-bag-slots count never switched on, and the target and focus cast bars never followed their setting
- **The battleground scoreboard read a row no server sends.** A battleground's per-player row and an arena's are two different shapes and the type byte at the top says which follows; this read one that was neither, taking a team byte from the arena shape and then the battleground's four counters. Everything after the guid was off by a byte and damage and healing were skipped entirely, which is why both always read zero. The end-of-match flag and the winner were read *after* the rows, where there is nothing left to read them from. A battleground row carries no team, so the scoreboard no longer groups or colours by a field nobody fills
- **Accepting a summon sent one byte where the server reads nine.** The reply carries the summoner's guid and the accept flag; the flag alone left the packet short and the server discarded it, so accepting did nothing and the offer expired
- **Every guid in the equipment-set family was read and written flat** — all twenty-one. A packed guid is a mask byte followed by only its non-zero bytes, so reading eight raw bytes put every field after the first at the wrong offset, and saving, equipping and deleting a set all sent packets the server could not parse
- **An achievement's progress counter is a packed guid too**, and reading it as a plain 64-bit value left every counter wrong and no criterion drawing a progress bar
- **The quest log and the quest-giver marks survived a character switch.** Logging out to the character list and back in on someone else kept the previous character's quest log, its pending queries, and the marks over every NPC
- **Ten chat types the client could not name.** The event name is built from the type byte, so a value missing from the enum is a line of chat that never appears — no error, nothing in the log. The whole run between LOOT and the battleground block was absent
- **Destroying a stack means the whole stack.** A count of zero was coerced to one, and zero is how the wire says "all of it"
- **A portal guard that never expired blocked the way back in.** The hold that stops a player bouncing straight back through a return portal is released when they leave the trigger, and the staleness escape hatch could leave it held
- **The game clock has one unit, and the sky reads it.** `SMSG_LOGIN_SETTIMESPEED` carries the same packed bitfield the guild date does, and it was stored raw under a comment calling it seconds since epoch
- **One reading of the packed date, and it is the server's.** The guild creation date is one `uint32` of bitfields; this read a day, a month and a year as three separate `uint32`s — twelve bytes where four were sent — so the date was nonsense and the member and account counts after it were read from the wrong place
- **An elevator keeps the yaw it was placed at.** `registerTransport` took no orientation, so every transport began at identity and had whatever the spawner placed discarded on the first tick
- **Elevators are not airships.** Entry and displayId are different numbering spaces and the transport model override mixed them, so three GameObject entries read as displayIds matched nothing
- **O opened the social window and would not close it again.** The guard read `WantCaptureKeyboard`, which is true whenever any ImGui window wants the keyboard — and opening this window is what gives it focus, so the key that opened it could never close it
- **Instances the buffer had no room for are no longer drawn.** The vertex shader read past the end of the instance SSBO hundreds of times a frame and the device was lost seconds later
- **No WMO group is dropped for any reason.** Buildings disappeared from angles that had no business hiding them: distance culling had been turned off years ago for the same complaint, and the test ran whether the flag was set or not
- **One clock, so a cooldown sweep is drawn where it belongs.** `GetTime` and the application each fixed their own origin on first call, and the two differed by whatever separated those calls
- **Three bootstrap constants had values the game does not use.** With the original interface not loaded they are the only values there are, so a wrong one stays wrong

### Added
- **Interacting with a game object dismounts.** Opening a chest or gathering a node puts a player on foot in WoW, and staying mounted left the server refusing the actions that check for it
- **The pet's name is asked for**, rather than left to whatever the creature template calls it
- **`START_LOOT_ROLL` carries the countdown** the packet already held, so the roll window's timer bar has a length
- **`CONFIRM_BINDER` carries the innkeeper's name**, which the question is asked with
- **`-DWOWEE_SYSTEM_LUA=ON` links an installed Lua 5.1** instead of the vendored copy, which is what a distribution package usually wants. Off by default, so which interpreter a build links does not depend on what happens to be installed. It must be 5.1: configuring stops with a message rather than linking a later one, which is not redundant with the version handed to `find_package` — that is a minimum, and CMake's own `FindLua` reports a 5.4 install as satisfying it

### Changed
- **The top-level `CMakeLists.txt` is 1428 lines rather than 2134.** The command-line tools and the packaging rules moved to `cmake/Tools.cmake` and `cmake/Packaging.cmake`, verbatim and included from the same scope; both trees generate the same 2190 targets
- **glm is linked once, on the target every test links.** There were thirty copies of the same per-target block, each added because one platform's CI broke — glm's include path arrives with an imported target rather than any directory the tests file lists, so a test that reaches `<glm/glm.hpp>` through a chain of headers compiles anyway on Linux and fails on macOS. Twenty-six of the thirty also checked only `glm::glm`, with no branch for the header-only target GLM 1.0 exposes

## [v2.0.37-preview] — 2026-08-02

### Fixed
- **A broken script no longer takes the client down.** Lua errors were reported by firing `UI_ERROR_MESSAGE`, which is itself a Lua event that `UIErrorsFrame` listens for — so reporting an error ran script, and when that script errored it was reported the same way, recursing through both stacks inside a single frame until the process died. Because it died with drawing in flight, what survived in the log was a Vulkan device loss, which sent the search into the renderer for a fault that was never there. Errors now go to a path that shows them without telling any script, and event dispatch refuses to nest more than eight deep
- **Flights to anywhere but the next stop work.** Two separate faults: `CMSG_ACTIVATETAXI` carries only a source and a destination, so the server had no single path to answer with and the request timed out; and the route was worked out by a second search of the flight graph that included nodes the player had never visited, which the server refuses outright on the first one it does not recognise. Multi-stop routes are sent whole, and are built from the same discovered-node search the quoted price already came from
- **Turning shadows off no longer loses the device**
- **Casting a spell on yourself no longer turns you around**
- **A long flight path is no longer rejected for its length**, and a rejected one says why
- **Arriving in the world during a flight no longer returns the player to where they logged in**
- **The interface fonts are found whatever the case the install spells them in.** Four fixed spellings were tried, so a directory written `Misc/fonts` matched none of them and the built-in face was kept — reported at a level the log does not carry, which left wrong-looking text and no reason anywhere. Every way this can fail is now a distinct warning
- **Texture uploads no longer share one fence across two queues**, and finished batches are retired every frame rather than only while terrain is streaming

### Changed
- **The client's interface is drawn in the game's own typeface.** FRIZQT is loaded at fifteen points and added first, so it is the face ImGui uses for everything that does not ask for another — close enough to the metrics the panels were built against for their layouts to survive. The five faces stay registered at eighteen points for the widget renderer, which asks for them by name. This reverses the previous release's fix, which kept the built-in face as the default

### Fixed
- **Loading the original interface no longer covers this one.** `WOWEE_LOAD_FRAMEXML=1` builds a hundred of Blizzard's frames, most of them still half-supported, and every one of them was drawn on top of the client's own. They now appear only for the elements named in `WOWEE_FRAMEXML_UI`, so loading it to exercise the parser leaves the interface you were using on screen

## [v2.0.35-preview] — 2026-08-01

### Fixed
- **The client's own interface keeps its own font.** ImGui draws with whichever face is added first, so loading the game's typefaces made FRIZQT the default for every panel this client draws, at eighteen points — larger text and different metrics than the layouts were built against, on startup, whether or not the original interface was being loaded

## [v2.0.34-preview] — 2026-08-01

### Original interface (FrameXML)
- **The interface draws its own art.** Batching the widget renderer's texture uploads left a batch holding only ImGui textures looking empty to both ends of endUploadBatch, so the command buffer carrying every copy was freed unsubmitted and every image stayed blank while rectangles and text drew normally. Staging allocated with plain Vulkan calls is now handed to the batch and freed once its copies have run
- **The interface is laid out in its own units.** FrameXML is authored against a virtual screen 768 units tall, so a frame is the same apparent size on every display; treating those numbers as pixels drew it at half size on a 1528-tall window. The tree lays out in units and the renderer converts once, with the cursor making the same trip in reverse
- **UIParent fills the screen.** It was created with no anchors, and an unanchored frame falls to the centre-on-parent default with no size — so everything hanging off it, including FrameXML's own UIParent, inherited a zero-size box in the middle of the screen
- **Type is drawn in the game's own faces** — FRIZQT, MORPHEUS, SKURRI, ARIALN and FRIENDS — at the size, colour and outline FrameXML's 42 font objects specify, rather than one built-in face at a guessed size
- **Sliders drag, cooldowns sweep and edit boxes take text.** GetTime, which all three need and nothing had implemented, answers from one clock shared with the renderer
- **All three mouse buttons reach the interface**, gated on what each frame registered for, which is how a context menu opens on a unit frame and not on a plain button
- **The whole original interface loads.** All 139 files of Blizzard's own FrameXML — 13 Lua and 126 XML — build against this client's widget tree in around 380ms, behind `WOWEE_LOAD_FRAMEXML=1`. It was 67 files failing and a client frozen hard enough to need killing
- **CreateFrame applies the template it is given.** The fourth argument was ignored outright, and that is not a missing feature so much as a trap: OptionsList_OnLoad makes one button, divides the list's height by that button's height to decide how many fit, and loops to the result. No template means no size, a height of zero, and a count of (h-8)/0 — which Lua computes happily as infinity. The loop then created frames under fresh names until memory ran out
- **A template that inherits another applies it.** The emitter returned before `inherits` was ever read for a virtual frame, so 217 of FrameXML's 296 templates arrived without the base they are built on
- **`parentKey` binds a region to a field on its owner.** 242 declarations across 31 files, every one of them nil, and FrameXML's handlers reach for them constantly — QuestHonorFrameTemplate's OnLoad opens with `self.icon:SetTexture(...)`
- **A frame's `id` exists.** 848 declared across 57 files with no `GetID`/`SetID` behind them, and FrameXML concatenates the result straight into a name: `_G["PartyMemberFrame" .. self:GetID() .. "PetFrame"]`. It is set before any template runs, because a template's children load while the template is being applied
- **Button art declared outside a Layer is created.** `<NormalTexture>`, `<HighlightTexture>`, `<ButtonText>` and their siblings were ignored; HighlightTexture alone appears in 62 files. The setters take a file path as readily as a texture, which is how LoadMicroButtonTextures uses them
- **A scroll frame's `<ScrollChild>` is built**, and handler arguments have their real names — `OnVerticalScroll`'s body opens with `scrollbar:SetValue(offset)`, which was nil on every scroll frame in the interface
- **`$parent` skips unnamed frames to the nearest named ancestor**, and a frame loads once when it is finished rather than once per template it is built from
- **The missing-API stand-in answers methods, not data.** FrameXML guards its optional frames properly — `if (prefixText) then prefixText:GetText()` — and a stand-in that answers everything makes the correct check worse than no check. Widget methods are now enumerated rather than guessed at, because measurement showed methods and data cannot be told apart by shape: of the 307 method names FrameXML calls, eighteen read as nouns
- **A runaway script costs one file rather than the session.** The load runs on the main thread during world entry, so a script that will not return freezes the client until the server drops the connection. A wall-clock deadline aborts it and names the Lua line it was on
- **Errors carry the Lua call stack.** An error says where it happened; the interesting part is nearly always how it got there


## [v2.0.33-preview] — 2026-08-01

### Transports
- **Ships sail bow-first.** Facing was pinned to whichever orientation the server last reported — a berth heading — and held there for the whole voyage while the position ran along the route underneath. That field is set by every server update, including ones for a ship the client animates itself, and it was taken as authoritative regardless. It is authoritative only while the server is also driving position; when the client owns the animation it owns the phase, and facing comes from the route
- **Every hull's bow is at model -X, so they all take the same correction.** The table said the opposite and then listed the icebreaker and the night-elf ferry as the two exceptions — exactly inverted. Measured two ways across every .wmo under World\wmo\transports: the hulls taper to a point at -X and stay blunt at +X, and the icebreaker is a paddle steamer whose paddlewheel, which belongs at the stern, sits at x=+36.3 on a hull spanning -60.7..+50.1. The table could never have been right, because it was fitted while facing came from a frozen server yaw rather than from the route
- **A docked hull lies on the chord through its berth.** A route turns as it passes its dock — the Maiden's Fancy comes into Menethil 26 degrees off the bearing it leaves on — so taking the arrival leg alone parked it half that turn out of true, enough to walk the gangway off the plank
- **Transports run on the server's route clock.** A WotLK MO_TRANSPORT publishes its period and how far through it the hull is, as a fraction of 65535. Neither field was read, so the client animated on a period it worked out from distance over speed; when that came out short, the ferry lapped its shore until the server's schedule caught up. This syncs the cycle, not yet the position within it
- **A cross-continent boat waits at the pier between crossings** rather than ferrying its shore over and over. Each map's slice is animated on its own and was sized from its own nodes alone; it now measures the whole route and spends the difference held at the dock
- **The 180-degree correction was measuring nothing.** It compared the canonical velocity against (cos s, sin s), where a server yaw points along (sin s, cos s) — the two components swapped, a reflection rather than a rotation. A transport facing exactly along its travel scored sin(2s), which is -1 near a heading of 135 degrees, so correctly-oriented ships were flipped purely on which way their route ran
- **Sails and paddlewheels are on the ships.** They were being drawn at the world origin: WMORenderer::setM2Renderer is declared and was never called, so every path that moves or destroys a WMO's child doodads sat behind a null pointer. Static world doodads were unaffected, because terrain streaming places those itself
- **Each ship gets its own doodads.** M2 instances are deduplicated on model and position, which is right for the static world and wrong for children created at the origin and moved into place by a parent — every ship of a class was handed the first one's sails, and whichever hull unloaded first destroyed them for the other. The same collapsed thirteen barrels in a hold into one
- **A docked ship's machinery stops.** The animation was set once when the doodad spawned and never revisited, so the Kraken's paddlewheel turned while the ship sat at the pier
- **A rider can get off.** Boarding somewhere the deck query never succeeds — a gangway belonging to the pier rather than the hull — left a hold engaged that discards walking motion and reapplies the boarding offset every frame. The character ran on the spot, could not walk far enough to trigger disembark, and so could not leave. Stepping ashore onto the dock also kept them attached: the disembark footprint is larger than any hull by design, and losing the deck underfoot is what actually tells ashore from aboard
- **Zeppelins stopped flying one another's routes.** The fallback returned the first usable id in a candidate list for the whole display family, and below it a last-resort branch handed out the first moving path in an unordered map

### Mounts & Pets
- **Dismounting no longer strikes a pose.** The mount display field keeps its old value for a few frames after the request, and that was taken at face value and re-mounted the player seven milliseconds after they got off. The restored value then made the server's own dismount read as transient and get discarded. The mount blinked off, back on, and off again over about two hundred milliseconds, with the character caught holding the seated rider animation
- **A rider on a moving boat sits on their mount.** The seat position is smoothed to damp bone jitter while sitting still, and the branch that snaps instead keys off movement input — but a player standing on a boat presses nothing while the world carries them. The filter trailed them by its own time constant, two yards at ferry speed, swinging to one side as the hull turned. It now asks whether the seat is moving in the world rather than whether the player asked it to
- **Pets can be dismissed.** The action field of CMSG_PET_ACTION is a pair, not an id: the high byte says what kind of action it is. Dismiss packed action 0 under the command type, which is COMMAND_STAY — the pet planted itself. Four callers each read the field differently; one had the type and action swapped, another sent a bare 1..6 with no type byte, and the bar labelled slots off a numbering that does not exist on the wire
- **Companions can be dismissed by pressing them again.** They have no aura, so nothing appears in the buff bar to right-click, and pressing the spell only ever summoned. CMSG_DISMISS_CRITTER was in the opcode tables with nothing sending it

### World & Movement
- **Ironforge stops emptying out at doorways and hallways.** Portal culling seeded its walk from one position while testing every door against another's frustum — the camera and the character are in different rooms in a doorway, and neither is reliably the right place to start. It now seeds from both, which can only add groups. Ironforge showed it worst because exterior groups are seeded unconditionally, and it has almost none
- **Stairs leading underground can be walked down.** The terrain-penetration rescue pushes a player back to the heightfield when their feet end up beneath it, and a stairwell cut into a keep passes under that surface within a step or two, so each step down was undone. A floor already resolved by grounding that sits below the heightfield and at the feet now settles it
- **Landing a taxi flight at Booty Bay no longer throws the character under the structure.** The clamp probed for a floor at forty above the player, and the player is what the clamp rewrites every frame — so the deck was found only from far below it, and snapping onto it lifted the probe out of range. It flip-flopped between the deck at 36.5 and the terrain at 4.5 twelve times, and abandoned the player wherever the last frame landed
- **The world map opens on the zone you are in.** It worked the zone out from geometry: of the WorldMapArea boxes containing you, the one you sit deepest inside. Those boxes are axis-aligned rectangles around irregular zones and overlap heavily, so it opened on zones you were merely near. The server sends the zone id and the client already tracked it for other things

### Items & Mail
- **Mail shows who it is from.** The sender was rendered from a field that was never populated for player mail
- **Sorting bags merges partial stacks** before ordering them, so two half stacks become one
- **Disenchant can pick the item it works on.** The spell's target flags mark it as needing an item, which nothing acted on

## [v2.0.32-preview] — 2026-07-31

### Water
- **The water's edge has a shoreline.** A wet-sand band that darkens what is under it, sediment that moves with the surf, a swash line that runs up the beach and back, foam that rides the water instead of sitting still in world space, and spray thrown off the advancing front. The foam is broken up by cellular octaves at rotated, non-multiple scales with a jittered threshold, because thresholding Worley cells near their centres puts a dot in every cell and makes the lattice itself the pattern — which is the grid that was visible before
- **The ocean fades into the horizon haze** rather than ending on a hard line, and the wave fronts are phase-warped by noise so the generator's pattern stops reading as bright parallel lines at distance
- **Water churns where you move through it.** Wading lays down froth underfoot; swimming leaves a V wake off the shoulders whose arms open with distance behind. Points age out and spread as they go, and carry a bounding circle so every water pixel outside the trail rejects them in one test
- **Spray is drawn on top of the water instead of underneath it.** Water moved into a pass of its own so the refraction copy could be taken before it, which left the swim effects recording into the scene pass that now runs first — shallow water hid the droplets partly, the deeper water you swim in hid them completely. The wading spray also never spawned at all: it shared an accumulator that the swimming branch zeroes on every frame it is not swimming, so a 30/s rate could only ever reach 0.5 in a frame
- **Crossing the surface sweeps a waterline across the view** instead of the whole scene flipping at once. The line is anchored to the projected horizon rather than the middle of the screen, and the tint no longer gives out past 15 units down — the depth query's default vertical reach was rejecting the surface once you were deeper than that, so the scene snapped bright at a fixed depth
- **Refraction no longer feeds itself.** The scene copy the water samples was being taken from the finished frame, so a moving object left one sharp copy per frame — a train of ghosts — and the brightness compounded through the loop. The copy is now taken before the water draws and at half resolution

### Movement & Swimming
- **You no longer sink through hills.** Floor selection rejects any surface more than 0.60 yards above the feet as unreachable, which at the steepest walkable slope covers a mounted player for about 1/40th of a second — a 20 fps frame rises 0.83 yards and the terrain being climbed stops counting as ground. From there it compounds, because falling puts the feet further below the surface. Outdoors the heightfield has one surface per column, so feet below it are pushed back out, guarded against everything legitimately built underneath: WMO and M2 floors probed from the player, and hole-cut chunks, which is how a cave mouth is opened
- **Swimming holds its depth** instead of being pulled to the surface, and holding space keeps ascending rather than rising for a moment and stopping
- Walking out of water no longer stutters: both swim checks decided from a single depth, so a character at the boundary flipped state every frame, restarting the locomotion animation and sending a START/STOP pair each time

### Character & Equipment
- **Helmets go on the head.** All three paths — your character, other players, NPCs — attached head gear at M2 attachment 0, which is the shield mount, falling back to 11 (the helm) only if that failed. It never failed. Detaching 0 on an equipment refresh was also dropping shields
- **Your own character wears a helm at all.** The local appearance path had no head slot: six attachment calls, all weapons. Head-model resolution — race and gender suffix, base fallback, suffixed texture — now lives in one place that all three paths call
- **A circlet leaves your hair showing.** Hair was hidden for any head item, so a tiara left the character bald with nothing visible. ItemDisplayInfo points at a HelmetGeosetVisData row per gender, and the row crowns and circlets use is all zeroes where a plate helm's is not. The columns holding those references move between the 23-field and 25-field builds, so they are found by asking which columns reference the visibility table
- **Show Helm works.** It flipped a bool, sent the packet and printed a message; nothing read the flag
- **Facial features exist.** CharacterFacialHairStyles' geoset columns were read at 3, 4 and 5, which hold a constant per race in every copy of that DBC here — Draenei rows read 2010429269 on every variation. Truncated and offset they name geosets no model has, so no character had a beard, tendrils or earrings. The variants are at columns 6 to 8. The clamp that forced each channel to at least 1 goes with them, since zero means the channel has no feature
- **A face overlay authored at a different resolution than the body is fitted to its region.** The only resizing was a whole-factor upscale, so an overlay larger than its region was pasted at its own size across the regions next to it. Mismatched art sets are now reported
- A character whose appearance the data cannot draw takes the nearest face rather than none, and says so — character creation offers an unverified 0..9 range whenever its DBC scan comes up empty, and those numbers are backed by no CharSections row

### Targeting & Interaction
- **A corpse no longer outranks the living player standing on it.** A dead creature is still a UNIT and still answers isHostile(), and hostiles are selected ahead of everything; failing that, a body at ground level is nearer the camera than a player's hit sphere a metre up. The living now rank first, and a corpse stays selectable only when nothing alive is under the cursor
- **Fishing schools cannot be right-clicked empty.** They are fished, not opened. Clicking one sent CMSG_GAMEOBJ_USE and, while the object's metadata was outstanding, a CMSG_LOOT — which the server answers with the hole's loot
- Left-click targeting, the right-click world picker and the hover cursor shared one ray picker instead of three copies. The hover copy had already drifted: it had no critter case, so the hand cursor appeared over a sphere three times the size a click would test
- Warrior Charge rejects game objects and corpses, and quests marked complete can be abandoned

### Combat & Spells
- **Casting at a target actually faces it.** The renderer holds the character's yaw and the game side holds canonical yaw, and the frame loop converts render to game every frame — so a facing set only in the packet is undone before anything with a cast time completes, and the server re-checks the arc against the restored heading. Smite reported the target as not in front while the character plainly faced it
- **The conversion between those two was a mirror where it should be a rotation.** Render yaw is canonical plus 90 degrees, which falls out of the swap in canonicalToRender and the atan2(-dy, dx) canonical convention; it was written as 180 minus, which agrees at exactly one heading. Every user of the pair was wrong together, so nothing looked amiss until a value crossed to the server
- **Heals and buffs fall back to you when nothing friendly is targeted.** A heal and a nuke share an effect id and can share a school; EffectImplicitTargetA is what tells them apart. Spells that take either target, like Dispel Magic, are left alone
- Pressing the mount you are riding dismounts you instead of dismounting and immediately remounting

### Items, Mail & Bank
- **A priest robe read as a cloak.** The item query layout is guessed from the bytes, and it decided on InventoryType alone. On a server without BuyCount that read lands on AllowableClass, and Priest-only is 16 — INVTYPE_CLOAK. Both readings are now scored across several fields, with BuyCount itself breaking the tie: it is how many the vendor sells at once, and a layout read one field short puts a price there
- Mail attachment slots match what the realm's packet can carry — Vanilla writes a single item GUID, and the compose window offered twelve regardless, sending the first and leaving the rest in your bags without a word. Attached stacks show their size
- Each bank bag can be sorted on its own; sorting the whole bank pools everything into the main slots, which empties a bag being kept as a category

### Rendering
- **Rigid props stopped swaying like trees.** Foliage tokens are matched as substrings because model names run words together, so "thorn" inside Stranglethorn made every troll ruin sway — along with "corn" in Corner, "hops" in ShopSign, "tree" in StreetSign, "crop" in Outcrop and "herb" in Herbalism. Names are head-final compounds, so the match ending furthest right decides: StranglethornRuins is a ruin while DustwallowTree is still a tree. 73 models stop swaying and no plant loses its wind
- **Forges are solid, and so is Ironforge.** isForge matched the city, so all 64 of its doodads had every batch forced to additive — benches, statues, cliffs, elevators. A forge is now a forge only when the name ends on it, and the additive override applies to the flame cards rather than the whole model, which is mostly masonry
- **Vertex explosions on creatures.** Bone indices were declared signed and read as such in the shader, so a bone index above 127 became negative and flung vertices across the world
- The UI draws in its own single-sampled pass rather than being multisampled and refracted through water
- NPC speech bubbles resolve $-tokens the way the chat log does

### Performance
- **Terrain streaming no longer stalls.** A single WMO took 158 ms against an 8 ms budget, 81% of it in group upload. Groups and textures now upload incrementally across frames, and one model per step
- M2 instance creation is bounded by time, its instance storage reserved so growth cannot stall a frame, and the bone seed found by lookup rather than scanning every instance
- Transport WMO uploads spread across frames; the login background decodes off the main thread

### Stability
- **Quitting no longer crashes.** The deferred-destruction drain added to WMO shutdown landed inside the `if (!vkCtx_)` early return, calling through the pointer exactly when it was null
- Renderers drain their deferred destruction while their own descriptor pools are alive, which closes a 426 MB shutdown leak across roughly 63,000 allocations
- **Incremental WMO loading dropped a group at every budget break.** It marked the current group done before deciding whether to stop, so Stormwind — 286 groups against a 6 ms budget — lost around 22 of them, interior floors past a doorway among them. Every non-empty group is now checked for before a model is published
- Water footsteps point at sounds that exist: the WATER surface was built from a naming convention that is real for Stone, Dirt, Grass, Wood and Snow but has no Water variant in any archive, and the movement sounds pointed at a folder that does not exist

## [v2.0.31-preview] — 2026-07-24

### UI
- **Auction listings show the item's rolled random property.** An auction carries the "of the …" suffix separately from the item template, so browse-tab tooltips rendered the base template only — a Bear's suffix looked identical to no suffix at all, and there was no way to tell what you were bidding on. Tooltips now fold the auction's random property and suffix factor into the same instance-aware view the bags use, so the Strength, Stamina and secondary-stat bonuses read the same in the auction house as they will in your inventory

### Platform
- **Holding a key on macOS opened the accent chooser instead of repeating it.** SDL2 leaves text input enabled for the whole session, so AppKit routed every keystroke through `NSTextInputContext` — and A, S, E and the other letters that take diacritics popped the press-and-hold menu over the game rather than moving the character. Mac builds now register `ApplePressAndHoldEnabled=NO` before `SDL_Init` brings up NSApplication. It lands in this process' registration domain, so nothing is written to your saved preferences

## [v2.0.30-preview] — 2026-07-24

### Rendering
- **Brightness is a true multiply again, and no longer blows out over water.** The multiplicative overlay (scene × brightness, instead of a lerp toward white) had to be reverted once because water refraction samples a scene-history image captured from the final swapchain, which already has display brightness baked in — re-applying it each frame fed back through that temporal capture and diverged, where the old white-lerp had merely converged. The brightness factor is now passed to the water shader and divided back out of the refraction sample, so refraction sees the un-brightened scene and the display gets a real multiply. This also fixes the latent inverse, water slowly creeping to black
- **FSR3 frame generation creates its upscale context.** "Path C upscale failed rc 3" was the AMD FFX Vulkan backend failing to build its compute pipelines: the device enabled `shaderFloat16` for fp16 math but not the 16-bit *storage* features the SDK's shaders need to pack fp16 into buffers. `storageBuffer16BitAccess`, `uniformAndStorageBuffer16BitAccess` and `shaderInt8` are now enabled where the device supports them
- `WOWEE_VULKAN_VALIDATION=1` turns on the Khronos validation layer in a release build and routes its output to the log — the tool that identified the FSR3 failure above as SDK-side invalid shaders (4KB push constants against a 256-byte limit, NV-only SPIR-V extensions, descriptor mismatches)
- **Steam tonks stopped glowing.** The "steam" substring in the VFX classifier also matches SteamTonk vehicle models. Gating on low-poly geometry wasn't enough — the TBC/Turtle tonk overlay models are small enough to slip under the vertex threshold, so a tonk with a smoke emitter was classified as an additive spell effect and rendered translucent. Any "tonk"/"tank" token is now excluded outright; real steam effects never carry one. Covered by a classifier regression test
- **Cloaks are textured in the world, not just in the paperdoll.** The in-world player model read the cloak texture from ItemDisplayInfo's LeftModelTexture only, but some cloaks — Jaina's Radiance among them — store it in the right field, leaving them blank in the world while the character preview (which already checked both) looked right

### Bank & Guild Bank
- **Depositing at the bank uses your purchased bank bags.** The old path scanned only the main bank slots and announced "Bank is full" once they filled, ignoring bag space entirely; deposits now go through `CMSG_AUTOBANK_ITEM` so the server places the item in any free bank slot
- **Right-clicking a bank item withdraws it.** The bank slot renderer had drag and shift-link but no right-click handler at all, so right-clicks were silently dropped. Withdrawal now uses `CMSG_AUTOSTORE_BANK_ITEM`, letting the server place the item in any free bag rather than only the backpack
- **Guild vaults open when you interact with them.** Nothing called `openGuildBank()` for a type-34 GameObject, and `openGuildBank()` never sent `CMSG_GUILD_BANKER_ACTIVATE` — it only queried a tab, so no bank list ever arrived
- **Guild bank item transfers were malformed on the wire and silently dropped.** `CMSG_GUILD_BANK_SWAP_ITEMS` was missing the toChar direction byte, wrote splitedAmount as a mid-packet u8 rather than a trailing u32, and set bankToBank=1 on deposits, which sent the server down its bank-to-bank path. Both builders now match the 3.3.5a layout, with withdrawals using the autoStore sub-format so the server auto-places into a free inventory slot
- Right-clicking a bag item while the guild bank is open deposits it into the first free slot of the viewed tab, and bags open automatically with the vault so items are reachable. Clicking a tab previously sent a query without updating the active tab, so withdraw and deposit always targeted tab 0 — the active tab now syncs from each `SMSG_GUILD_BANK_LIST`
- The guild bank renders a full 98-slot (14×7) grid and looks items up by slot ID. It previously drew only the slots the server sent, which is a sparse list — often just the occupied ones, and nothing at all for an empty tab
- The bank's "Combine bags" toggle persists across relaunches (`bank_combine_bags` in settings.cfg); it was a function-local static that reset to the split view every session

### Crafting
- **Crafting while mounted no longer freezes the window.** `startCraftQueue` filled the queue and then called `castSpell`, which bails early when mounted — leaving the queue populated with nothing in flight and the UI stuck on "Crafting… N remaining" until you manually mounted and dismounted. It now dismounts synchronously before queueing, matching retail
- The recipe list is a draggable splitter and grows with the window. It was a fixed 260px while the detail pane absorbed all extra width, so enlarging the window never revealed a truncated recipe name; anything still too wide for the pane gets a hover tooltip with the full name

### Quests
- **Collect-item objectives advance as you loot.** 3.3.5a servers don't push collect counts the way they push kill credit, so a tracker relying on `SMSG_QUESTUPDATE_ADD_ITEM` alone never moved. Item objectives are now reconciled against actual bag contents on every inventory rebuild
- Newly accepted quests are tracked automatically, from both questgivers and shared-quest accepts. Login and resync loads are untouched, so the "show all when none tracked" fallback still covers quests you already had

### Character
- **Casting while mounted dismounts and then casts.** Previously any spell pressed while mounted just dismounted and dropped the cast. Airborne on a flying mount, the cast is refused with "You can't do that while flying" rather than dropping you out of the sky

### GM Tools
- **A searchable GM command browser**, on a new "GM" micro-menu button, over the existing 195-entry command reference. The left pane groups commands by first token (flattening to a filtered list while searching) with a max-permission filter; the right pane shows syntax, description and a security badge. Commands dispatch as SAY chat with the AzerothCore "." prefix, so the server still enforces the real permission level
- Command syntax is parsed into labeled form fields rather than a raw editable string — `#x` becomes a numeric input, `$x` a text input, `a/b` a dropdown, `[word]` an optional checkbox — with a live "Will send" preview, player/name fields defaulting to your current target, and an "Edit manually" escape hatch. Adds 12 more commonly-used commands (the reset family, repairitems, additemset, modify arenapoints/drunk/faction/xp/phase)
- **"Max Out Character"** detects your class and active expansion and queues a full setup: max level for the expansion (60/70/80), all class spells and talents, maxed skills, optionally 1000g, and a class-appropriate gear kit anchored on class legendaries. Commands drain one per frame to stay under the server's chat-flood protection, and per-slot toggles let you apply only the parts you want
- **`.gm fly on` actually lets you take off.** It sets the CAN_FLY movement flag, but flight physics were gated on `isPlayerFlying()`, which also wants FLYING — a flag the server only sets once you're already airborne. Flight is now driven from CAN_FLY, and the descend key (X) works on foot instead of only on a flying mount

## [v2.0.29-preview] — 2026-07-23

### World
- **Hairline seams between terrain tiles are gone.** Each tile's edge vertices were built by subtracting the chunk and per-vertex steps from the tile corner, so a tile's far edge (`…×TILE − TILE`) and its neighbour's near edge (`(…−1)×TILE`) — mathematically the same point — rounded to slightly different float32 values. The sub-yard gap opened T-junction cracks that showed as thin lines, worst far from the map origin (across Kalimdor). Vertex XY is now a single multiply from the tile index, `TILE_SIZE × (32 − tile − step/128)`, so the shared edge is bit-identical on both sides and the tiles meet exactly — no mesh-overlap or scale hacks
- **Game objects went missing after leaving an area and coming back, for the rest of the session.** Walking away dropped every instance of a mailbox or chest model, and the 60-second unused-model reaper then evicted the model itself; the reload on return did not reliably produce a drawn object. Game object models are now pinned in the renderer, so the reap/reload cycle never happens for them. They are a small bounded set — one per display ID actually encountered — and ambient doodads are still reaped normally
- Game objects are exempt from the adaptive doodad render distance. That distance collapses to its densest-scene value in any populated area, which in a city means roughly 200 units, so mailboxes and chests vanished well inside the range the server still considered them visible. They now hold a 600-unit floor; frustum and occlusion culling are unaffected
- GPU cull results are matched back to instances by ID rather than array index. The visibility buffer is read a full frame-slot cycle after it is written, and instances are appended and swap-removed in between, so a respawned object landing at the volatile tail of the array could inherit the verdict of whatever transient object held that slot two frames earlier
- Game objects no longer take the HiZ occlusion test at all. A mailbox or chest sits flush against a wall or doorframe, exactly where the coarse depth pyramid reports a false occlusion; once culled it stops being drawn, so it never regains the last-frame depth that would clear the false verdict, and it stayed invisible in place until the camera moved. These small gameplay props now opt out of occlusion culling entirely (frustum and distance culling still bound them), so they cannot vanish while in view

### Lighting
- **Hearth fires, campfires and forges cast light.** Fires express their flame as particle emitters rather than a glow card, and the path that turns emitters into a light was gated on lantern-like models — so a fireplace full of burning wood lit nothing at all. Open flame now takes that path, with a wider, warmer light than a candle wick, and forges are classified as the contained fires they are
- **Forges rendered as black windows.** `BLACKSMITHFORGE.m2` is nothing but the fire in the hearth, an effect card on a black background, but the additive override that handles exactly that shape was gated on spell effects, so it drew opaque and filled the opening with a black rectangle. Its black backing is colour-keyed too: the texture is `ARMORREFLECT`, whose name carries none of the flame or glow tokens the colour-key hint looks for
- Lamps, torches and braziers gutter. Each fixture's phase is hashed from its own placement, so no two pulse together — a synchronised row of street lamps reads as a rendering artifact rather than firelight — and the guttering drives the light each one casts, not just its glow sprite, since the pool of light on the ground is what the eye actually reads as fire
- Darkshire's town hall clock face is lit by a fire behind it. WMO emissive was a flag tuned for Stormwind's lamp glass, far too bright for a clock face, and is now a level: the new one keeps normal daylight shading, adds a warm glow that fades up as the scene darkens, and wavers on three detuned sines so the flame never visibly loops. A tight sun highlight and a Fresnel sheen sell the pane of glass over the dial
- Hearth fire light no longer turns the surrounding brickwork orange. Local lights are unshadowed, so their radius is how far the glow reaches straight through whatever surrounds the fire; at 11 units a hearth lit its entire chimney from the inside out

### UI
- **Target-gated abilities (Execute, Hammer of Wrath, ...) grey out until the target qualifies, and fail with a useful reason.** These read Spell.dbc's TargetAuraState — e.g. "target below 20% health" — which the client never consulted, so the button looked castable and the failure just said "Target aurastate". The action button now dims (with a tooltip naming the requirement) while the target isn't in the needed state, and the cast-failed message reads "Target must be below 20% health." and the like
- **The reputation panel gained real tracking controls.** Right-clicking a faction now offers, alongside "Track on Rep Bar", an **At War** toggle (declares war / makes peace via CMSG_SET_FACTION_ATWAR, disabled on peace-forced factions) and an **Inactive** toggle (parks it via CMSG_SET_FACTION_INACTIVE). Inactive factions are hidden behind a "Show inactive" checkbox — which reports how many are parked — and render dimmed when shown
- **"Your auction of … has sold!" named the wrong item ("Item #0").** The owner-notification packet was parsed with a phantom `action` field, which pushed the item-entry read to an offset holding a zero padding word; the real `item_template` sits at offset 20. It now parses correctly (and always reads as "sold", since expiry and outbids arrive on their own opcodes)
- **Readable letter/note items resolve their `$`-tokens.** The item-text window drew the body raw, so a quest letter showed literal markup like "$g himself : herself;". It now runs the same placeholder replacer as quest and chat text, filling in gender ($g), player name ($n), line breaks ($b), and the rest
- **The achievements window shows each achievement's real icon** instead of a gold star. Achievement.dbc's IconID (added to the DBC layout) resolves through SpellIcon.dbc to the artwork, rendered as a bordered 32px icon with the name and point value beside it; a star placeholder still fills the slot while an icon streams in or if it's missing
- **Raid target markers never appeared on marked enemies.** Three separate faults stacked: `GameHandler` kept its own copy of the marks that nothing ever wrote, so the target frame, nameplates, minimap and party list all read zeros; the wire format was inverted, with the full list (which carries only the icons that are set, not a fixed eight) and the single-mark form (which leads with the setter's GUID on WotLK but not on classic or TBC) swapped; and the marks were drawn as text symbols the font has no code points for, so they rendered as '?' boxes. Marks now use Blizzard's icon artwork, floating above the unit as in the original client, and are covered by tests across both wire layouts
- Solo players can set target markers. The server only broadcasts them to a group, so marking while ungrouped did nothing and the feature could not be used, or tested, without a second player. Grouped marking stays server-authoritative
- The DPS meter sits under the target frame instead of near the bottom of the screen, and can be dragged; the position persists, and right-click returns it home
- **The bank window lists the cost of each unpurchased bag slot.** The "Buy Slot" button gave no price and let you click slots out of order; it now shows the gold cost per slot from `BankBagSlotPrices.dbc`, and only the next slot in sequence is buyable — later ones are locked with their price shown. Buying now goes through an "are you sure?" confirmation that names the cost, rather than spending gold on a single click
- The bank has a Sort button that arranges the main bank and every bank bag by quality, then item ID, then stack size — the same ordering as the backpack Sort, driven client-side one swap per frame
- The bank can show every slot as one continuous grid via a "Combine bags" toggle, instead of splitting each bank bag into its own labeled section
- **Spell descriptions resolve their `$`-tokens everywhere, not just in the talent panel.** Buff/aura tooltips, the spellbook, and item "Use/Equip" effect lines showed raw markup like "Stamina and Spirit increased by $s1" or "Restores $/5;s1 health per second". Token substitution now lives on a shared `GameHandler::formatSpellDescription` used by all of them, and additionally handles the `$/N;` division token (food regen) alongside the base-point, duration, cross-spell, and plural/gender forms
- **Spell tooltips now show the full effect Description instead of the terse one-liner.** The client read Spell.dbc's short `Tooltip` string, so a food item said only "Restores 3 health per second" and never mentioned the Well Fed buff it grants. Descriptions now come from the `Description` column (falling back to `Tooltip`), so food reads "Restores 17 health over 18 sec … become well fed and gain 2 Stamina and Spirit for 15 min."
- **Talent tooltips describe what the talent actually does.** The talent panel read Spell.dbc's `Tooltip` column, whose index in the layouts pointed at an empty locale slot, so hovering a talent showed only its name and rank. Tooltips now pull the spell `Description` (added to every expansion's DBC layout at its verified column) and resolve WoW's `$`-token grammar against live spell data — `$s/$o/$m/$M` base points and `$d` durations, including cross-spell `$<spellId>` references like `$14201d`, plus `$l`/`$g` plural and gender forms — so "receive a $14201s1% damage bonus for $14201d" reads "receive a 4% damage bonus for 12 sec". Tokens with no local source (`$h` proc chance, `$t` period) are stripped cleanly rather than shown raw. An unlearned talent shows its rank-1 effect under "Effect:"
- **Spell.dbc `Tooltip` (and TBC `Rank`) columns pointed at the wrong offset.** The WotLK layout's `Tooltip` and the TBC layout's `Rank`/`Tooltip` indices assumed the wrong locale-block stride, landing on empty columns; they now point at the verified enUS strings, so spell rank text and short tooltips resolve on those clients too
- **The auction "Create Auction" item picker lists everything you can sell.** It only walked the 16-slot base backpack, so items in equipped bags never appeared; it now enumerates the backpack and every equipped bag, and posts the chosen item by its server GUID so any container works
- **New mail announces itself.** Receiving mail while online — or logging in with mail already waiting — now prints "You have new mail." to chat and plays a notification cue, on top of the existing minimap indicator; it fires once per unread state rather than repeating while the mail sits unread
- The keyring is interactive and always visible when enabled
- AddOns can be enabled and disabled from a manager on character select, and unimplemented addon widget methods fall back to no-ops rather than erroring
- The target frame no longer stays stuck at a previous target's width

### Quests
- Quest giver markers refresh as soon as an objective completes, rather than only after leaving and re-entering the area. Sweeps are coalesced behind a one-second cooldown, since one request fans out to a packet per nearby giver and completion events arrive in bursts
- Key-locked chests open by using the key item on them; unlocked and quest-gated chests open instead of being refused

### Audio
- Capital City Bells volume is independent of the ambient slider

### Character
- Barber shop appearance changes apply without a restart

---

## [v2.0.7-preview] — 2026-07-12

### Camera
- **Hills no longer clip through the third-person camera.** Terrain was only ever a floor clamp at the camera's final position, so a rise *between* the character and the camera sliced straight through the view and the clamp just popped the camera upward after the fact. The camera ray now marches the terrain heightfield (coarse ~1.25-unit steps, then bisection for a tight limit) and the resulting distance feeds the same asymmetric pull-in/recover smoothing as the WMO wall raycast. Worst case is ~28 bilinear height lookups a frame — negligible. Pull-in snaps 1:1 while the mouse or turn keys are actively rotating, since the 60 ms ease was exactly the window where a fast swing into a hillside still dipped underground
- The terrain march is skipped inside interior WMOs (terrain above a tunnel is not a real occluder) and when the pivot itself sits below the heightfield (caves, WMO basements, ADT holes), so it cannot pin the camera to first-person where the heightfield is irrelevant
- X now dives while swimming instead of toggling sit, water-exit assists are suppressed while diving, and the swim-depth gate only applies on water entry — deliberate dives can go arbitrarily deep

### UI
- Crafting panel reagent lines show live have/need counts, recounted every frame so consumption is visible mid-craft; Create/Create All disable when any reagent is short

---

## [v2.0.6-preview] — 2026-07-12

### Networking
- **Stop dropping every packet that has no payload.** `handlePacket()` ignored any packet whose body was empty, but `getSize()` is the payload length and the opcode is carried separately — for the many opcodes with no payload, the opcode *is* the message. All of them were swallowed before dispatch, which is also why no "unhandled opcode" warning ever fired for them. Eight registered opcodes were affected:
  - `SMSG_LOGOUT_COMPLETE` — the server logged the character out and moved on while the client waited forever, so the logout countdown ended and nothing happened
  - `SMSG_LOGOUT_CANCEL_ACK` — a cancelled logout was never confirmed
  - `SMSG_ATTACKSWING_NOTINRANGE` / `_BADFACING` / `_DEADTARGET` / `_CANT_ATTACK` — none of the four auto-attack errors ever reached the player
  - `SMSG_PET_BROKEN`, `SMSG_INVALIDATE_PLAYER`

### Logout
- `/quit` and `/exit` now leave the game when the server confirms the logout; `/logout` and `/camp` return to character select. They were all aliases of one command that did neither
- `/logout` was not a command at all: `aliases()` is the complete name list and it only listed camp, quit and exit, so `/help` had been advertising a command that silently did nothing
- The logout pose is a sit again. The server stuns the player to root them for the countdown, and we mapped `UNIT_FLAG_STUNNED` straight to the stun animation, which played over the sit and left the character slumped

### UI
- The breath, fatigue and feign-death bars count down. `SMSG_START_MIRROR_TIMER` hands the client a remaining time and a scale and the server then only re-syncs on change, but nothing ticked the value — so the breath bar sat frozen while you drowned. The sub-millisecond remainder is carried, so the timer does not run slow at high frame rates
- Fix the tail of `SMSG_START_MIRROR_TIMER`: it is paused(1) + spellId(4), not the reverse

---

## [v2.0.5-preview] — 2026-07-12

### Build
- **The build was shipping shaders compiled on 4 April.** `compile_shaders()` wrote its SPIR-V into the runtime tree, and the POST_BUILD step then copied the whole source `assets/` directory over it — including the `.spv` files tracked in git. The stale committed shaders won every build, so three months of GLSL edits never reached the GPU. Seven shaders were affected: `character.frag`, `character.vert`, `terrain.frag`, `water.frag`, `m2_particle.frag`, and both FSR2 compute shaders. Shaders now compile in place next to the GLSL, so the tracked `.spv` and the shader the GPU runs cannot diverge

### Character Select
- Preview is larger (panel widened, render target raised to 640x800), shows the character's equipped weapons, and stands them in their racial glue scene — Stormwind for humans, Durotar for orcs, and so on
- Scene backdrops are placed from the camera and attachment point the M2 carries (M2Loader now parses cameras); their geometry sits hundreds of units from the model origin, so nothing else can position them
- Weapons and enchant visuals no longer leak between characters: weapon attachment ran past an early return for characters whose body skin could not be composited, and fixed model ids meant every character after the first was handed the first one's weapon model

### Rendering
- Backdrops are no longer erased by the character alpha heuristics. Stormwind's walls are DXT5 with an unused alpha channel (mean alpha 17/255, every texel below the cutoff), and inferring a cutout from "the texture has alpha" discarded the whole building, leaving the sky showing through it

### Item Enhancements
- Temporary weapon enchants show as the weapon's icon with its remaining time, in the right slot. SMSG_ITEM_ENCHANT_TIME_UPDATE carries the item's *enchantment* slot (TEMP_ENCHANTMENT_SLOT = 1), not the equipment slot, so every temporary enchant was labelled "Off Hand" — even on a two-hander

### Merged
- Extract `buildFactionHostilityMap()` into a shared free function (#95)

---

## [v2.0.4-preview] — 2026-07-12

### UI
- Show the build version and date bottom-left on the login screen and right-aligned in the settings window
- `core/version.hpp` is generated from `git describe --tags --abbrev=0` by `cmake/GitVersion.cmake`, so the client always reports the last tagged release. It regenerates on every build rather than only when cmake reconfigures, and rewrites the header only when the version actually changed
- The build stamp is a date, not a timestamp: a clock time would change the header every build and force a full recompile of everything including it

### Build
- Un-ignore `cmake/*.cmake`. The repo's blanket `*.cmake` rule targets CMake build output and would have silently excluded the new hand-written module from the tree, breaking a fresh clone

---

## [v2.0.3-preview] — 2026-07-12

### Item Enhancements (sharpening stones, weightstones, weapon oils)
- Send TARGET_FLAG_ITEM in CMSG_USE_ITEM. Item-enhancement consumables cast their spell onto another item, but the client only ever wrote a unit or self target, so the server dropped the cast and the item did nothing
- Using such an item now reads the on-use spell's Spell.dbc Targets mask and arms an item-targeting cursor; the next item clicked (in bags or equipped) receives the enchant. Escape or right-click cancels
- Add the Spell.dbc `Targets` column to all four expansion layouts (Classic 13, TBC 14, WotLK 16)
- Weapon enchant visuals: resolve SpellItemEnchantment → ItemVisuals → ItemVisualEffects and attach the effect M2 (e.g. the sharpening-stone glint) to the weapon model's item-visual attachment points, rendered additive and unlit
- Applying an enchant now marks equipment dirty even though the displayInfoId is unchanged, so the visual appears without re-equipping

### Bug Fixes
- Read enchant names from the correct SpellItemEnchantment.dbc column. The name moved across expansions (Vanilla 10, TBC 13, WotLK 14) but every caller used field 8, an integer column that getString() treated as a string-block offset — so names came back garbled mid-string ("Sharpened (+2 Damage)" surfaced as "ockbiter 3"). Resolved from the record width via `detectEnchantmentNameField()`

### Tests
- New `test_use_item_packet` suite: CMSG_USE_ITEM SpellCastTargets encoding for WotLK, Classic and TBC (item, unit, and self targeting)
- DBC tests for enchant name/ItemVisual column detection and the enchant → effect-model resolution chain

---

## [v1.9.7-preview] — 2026-07-09

### Bug Fixes
- Fix world login pipeline: login-critical opcodes (AUTH_CHALLENGE, AUTH_RESPONSE, CHAR_ENUM, CHAR_CREATE, CHAR_DELETE, WARDEN_DATA) now fall back to hardcoded wire values when opcode table lookup fails, preventing "Unhandled world opcode: 0x1ec" blocking character list retrieval (issue #87)
- OpcodeTable::loadFromJson() now loads into temporaries and only swaps on success — a failed reload no longer wipes the working table
- Integrity hash is now build-aware: Classic-era DLLs (fmod.dll, ijl15.dll, dbghelp.dll, unicows.dll) only required for builds <=6005 or Turtle; TBC/WotLK clients hash only the .exe

### Animation & Camera
- Rework strafing to use walk/run animations with SpineLow bone torso twist instead of dedicated strafe/run-left/right animations
- Add `setInstanceTorsoYaw()` to CharacterRenderer for per-instance upper-body rotation
- Camera smoothing snaps 1:1 while actively dragging or keyboard turning instead of always lerping, reducing perceived input lag
- Add `travelYaw_` tracker to CameraController for movement vector heading separate from camera facing
- Mount strafing uses MOUNT_RUN_LEFT/RIGHT animations when available
- Default mouse invert changed to off

### Tests
- Add "OpcodeTable failed reload preserves existing data" test case

---

## [v1.9.1-preview] — released, captures changes since v1.8.9-preview

### Architecture
- Break Application::getInstance() singleton from GameHandler via GameServices struct
- EntityController refactoring (SOLID decomposition)
- Extract 8 domain handler classes from GameHandler
- Replace 3,300-line switch with dispatch table
- Multi-platform Docker build system (Linux, macOS arm64/x86_64, Windows cross-compilation)
- Decompose ChatPanel monolith into 15+ modules under `src/ui/chat/` with IChatCommand interface, ChatCommandRegistry, MacroEvaluator, ChatMarkupParser/Renderer, ChatBubbleManager, ChatTabManager, GameStateAdapter, and 11 command modules (PR #62)
- Decompose WorldMap (1,360 LOC) into 16 modules under `src/rendering/world_map/` with WorldMapFacade (PIMPL), CompositeRenderer, DataRepository, CoordinateProjection, ViewStateMachine, 9 overlay layers (PR #61)
- Extract reusable CatmullRomSpline module to `src/math/` with O(log n) binary search and fused position+tangent evaluation (PR #60)
- Decompose TransportManager (`transport_manager.cpp` 1,200→~370 LOC): extract TransportPathRepository, TransportClockSync, TransportAnimator; consolidate 7 duplicated spline parsers into `spline_packet.cpp` (PR #60)

### World Editor (tools/editor/)
- Standalone world editor for creating custom WoW zones (~130k LOC across ~500 source files in `tools/editor/`, including procedural mesh/texture generators)
- 6 editing modes: Sculpt, Paint, Objects, Water, NPCs, Quests
- 30+ terrain tools: procedural generators (hill, mesa, crater, canyon, island, ridge, dunes), thermal erosion, noise, mirror/rotate, stamp copy/paste with file persistence
- Multi-select objects (Ctrl+Shift+Click), Select All (Ctrl+A), Select by Type (M2/WMO)
- Time-of-day lighting with dawn/dusk/night transitions and color pickers
- Texture eyedropper (Alt+Click), brush size presets + bracket keys
- Object tools: snap to ground, align to slope, flatten terrain around buildings, scatter with auto-align
- River/road path tool with click-to-set points and translucent preview ribbon
- Quest chains with circular reference detection, inline editing, load/save
- 631 creature presets across 8 categories with patrol path editing
- Full undo/redo for ALL terrain operations (generators, transforms, paint)
- Auto-save with configurable interval, unsaved changes quit confirmation
- Zone rename, recent zones menu, adjacent tile export with edge stitching
- Zone metadata panel: configurable Map ID, Display Name, Description
- Zone gameplay flags: Allow Flying, PvP, Indoor, Sanctuary (serialized to zone.json)
- Zone audio configuration: music track, day/night ambience, volume sliders, presets
- PNG/JPG/BMP/TGA heightmap image import (any resolution, 8/16-bit, undoable)
- Collision slope overlay on minimap (steep terrain visualization)
- Client-side WOC collision loading with walkability queries
- Zone map image export: colored top-down PNG with terrain, water, objects
- SQL spawn export for AzerothCore/TrinityCore (creature_template, creature,
  waypoint_data, quest_template — ready-to-import .sql files)
- Server module generator: one-click AzerothCore module with map registration,
  spawns, teleport command, zone flags, conf snippet, and admin README
- Biome vegetation auto-population: one-click procedural placement of
  trees, rocks, bushes, ferns per biome (10 biomes with density rules)
- Live open format validation (0-7 score) in File menu

### Novel Open Formats (7/7 Blizzard format replacements)
- ADT → WOT/WHM: terrain metadata + binary heightmap with alpha maps and doodad/WMO placements
- WDT → zone.json: map definition with full placement arrays
- BLP → PNG: texture override system
- DBC → JSON: data tables via DBCFile::loadJSON()
- M2 → WOM (WOM1/WOM2): static models + animated models with bones, keyframes, skeletal binding
- WMO → WOB (WOB1): buildings with material flags/shader/blendMode, doodad rotation
- Collision → WOC (WOC1): walkability mesh with slope classification, hole support, water flags
- WCP (WCP1): content pack archive with categorized file list
- Terrain stamps: portable terrain features saved as JSON
- All formats documented in FORMAT_SPEC.md v1.1
- Client auto-loads open formats from custom_zones/ and output/ directories
- Batch convert: M2→WOM and WMO→WOB from filesystem or asset manifest
- WCP Import & Load: one-click unpack + auto-open for editing
- 328 test assertions across 84 test cases (DBC binary+JSON, WOB, WHM, WOT, WOC)

### Features
- Spell visual effects system with bone-tracked ribbons and particles (PR #58)
- GM command support: 190-command data table with dot-prefix interception, tab-completion, `/gmhelp` with category filter (PR #62)
- ZMP pixel-accurate zone hover detection on world map (PR #63)
- Textured player arrow (MinimapArrow.blp) on world map (PR #63)
- Multi-segment path interpolation for entity movement (PR #59)
- Character screen keyboard navigation (Up/Down/Enter) (PR #59)

### Bug Fixes (v1.8.10+)
- Fix walk/run animation persisting after entity arrival (PR #59)
- Fix entity teleport during dead-reckoning overrun phase (PR #59)
- Fix Vulkan crash on window resize when minimized (0×0 extent) (PR #59)
- Fix quest log not populating on quest accept (PR #59)
- Fix hit-reaction animation being overridden on next frame (PR #59)
- Fix ChatType enum values to match WoW wire protocol (SAY=0x01 not 0x00) (PR #62)
- Fix BG_SYSTEM_* values from 82–84 (UB in bitmask shifts) to 0x24–0x26 (PR #62)
- Fix infinite Enter key loop after teleport (PR #62)
- Remove stale kVOffset (-0.15) from zone hover detection causing ~15% vertical offset
- Add null guard for cachedGameHandler_ in ChatPanel input callback
- Fix cosmic highlight aspect ratio with resolution-independent square rendering
- Skip transport waypoints with broken coordinate conversion instead of silent use
- Fix spline endpoint validation bypass for entities near world origin
- Fix off-by-one in chat link insertion buffer capacity check
- Zero window border in world map to eliminate content/window gap

### Tests
- Add 19 new test files (27 total, up from 8):
  - Chat: chat_markup_parser, chat_tab_completer, gm_commands, macro_evaluator
  - World map: world_map, coordinate_projection, exploration_state, map_resolver, view_state_machine, zone_metadata
  - Transport/spline: spline, transport_components, transport_path_repo
  - Animation: animation_ids, locomotion_fsm, combat_fsm, activity_fsm, anim_capability, indoor_shadows

### Bug Fixes (v1.8.2–v1.8.9)
- Fix VkTexture ownsSampler_ flag after move/destroy (prevented double-free)
- Fix unsigned underflow in Warden PE section loading (buffer overflow on malformed modules)
- Add bounds checks to Warden readLE32/readLE16 (out-of-bounds on untrusted PE data)
- Fix undefined behavior: SDL_BUTTON(0) computed 1 << -1 (negative shift)
- Fix BigNum::toHex/toDecimal null dereference on OpenSSL allocation failure
- Remove duplicate zone weather entry silently overwriting Dustwallow Marsh
- Fix LLVM apt repo codename (jammy→noble) in macOS Docker build
- Add missing mkdir in Linux Docker build script
- Clamp player percentage stats (block/dodge/parry/crit) to prevent NaN from corrupted packets
- Guard fsPath underflow in tryLoadPngOverride

### Code Quality (v1.8.2–v1.8.9)
- 30+ named constants replacing magic numbers across game, rendering, and pipeline code
- 55+ why-comments documenting WoW protocol quirks, format specifics, and design rationale
- 8 DRY extractions (findOnUseSpellId, createFallbackTextures, finalizeSampler,
  renderClassRestriction/renderRaceRestriction, and more)
- Scope macOS -undefined dynamic_lookup linker flag to wowee target only
- Replace goto patterns with structured control flow (do/while(false), lambdas)
- Zero out GameServices in Application::shutdown to prevent dangling pointers

---

## [v1.8.1-preview] — 2026-03-23

### Performance
- Eliminate ~70 unnecessary sqrt ops per frame; constexpr reciprocals and cache optimizations
- Skip bone animation for LOD3 models; frustum-cull water surfaces
- Eliminate per-frame heap allocations in M2 renderer
- Convert entity/skill/DBC/warden maps to unordered_map; fix 3x contacts scan
- Eliminate double map lookups and dynamic_cast in render loops
- Use second GPU queue for parallel texture/buffer uploads
- Time-budget tile finalization to prevent 1+ second main-loop stalls
- Add Vulkan pipeline cache persistence for faster startup

### Bug Fixes
- Fix spline parsing with expansion context; preload DBC caches at world entry
- Fix NPC/player attack animation to use weapon-appropriate anim ID
- Fix equipment visibility and follow-target run speed
- Fix inspect (packed GUID) and client-side auto-walk for follow
- Fix mail money uint64, other-player cape textures, zone toast dedup, TCP_NODELAY
- Guard spline point loop against unsigned underflow; guard hexDecode/stoi/stof
- Fix infinite recursion in toLowerInPlace and operator precedence bugs
- Fix 3D audio coords for PLAY_OBJECT_SOUND; correct melee swing sound paths
- Prevent Vulkan sampler exhaustion crash; skip pipeline cache on NVIDIA
- Skip FSR3 frame gen on non-AMD GPUs to prevent driver crash
- Fix chest GO interaction (send GAMEOBJ_USE+LOOT together)
- Restore WMO wall collision threshold; fix off-screen bag positions
- Guard texture log dedup sets with mutex for thread safety
- Fix lua_pcall return check in ACTIONBAR_PAGE_CHANGED

### Features
- Render equipment on other players (helmets, weapons, belts, wrists, shoulders)
- Target frame right-click context menu
- Crafting sounds and Create All button
- Server-synced bag sort
- Log GPU vendor/name at init

### Security
- Add path traversal rejection and packet length validation

### Code Quality
- Packet API: add readPackedGuid, writePackedGuid, writeFloat, getRemainingSize,
  hasRemaining, hasData, skipAll (replacing 1300+ verbose expressions)
- GameHandler helpers: isInWorld, isPreWotlk, guidToUnitId, lookupName,
  getUnitByGuid, fireAddonEvent, withSoundManager
- Dispatch table: registerHandler, registerSkipHandler, registerWorldHandler,
  registerErrorHandler (replacing 120+ lambda wrappers)
- Shared ui_colors.hpp with named constants replacing 200+ inline color literals
- Promote 50+ static const arrays to constexpr across audio/core/rendering/UI
- Deduplicate class name/color functions, enchantment cache, item-set DBC keys
- Extract settings tabs, GameHandler::update() phases, loadWeaponM2 into methods
- Remove 12 duplicate dispatch registrations and C-style casts
- Extract toHexString, toLowerInPlace, duration formatting, Lua return helpers
