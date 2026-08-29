# Changelog

## [v3.1.14] - 2026-08-29

### Fixed
- **The account's session key was written to every log.** `GameHandler::connect` printed all forty bytes at INFO on each world login, and `computeAuthHash` printed them again at DEBUG along with the hash input, which ends with the same forty. That key is the shared secret the world handshake proves knowledge of and the header cipher is keyed from - anyone holding it can authenticate as the account and decrypt its traffic - and the log is the file players are routinely asked to attach to a bug report. The lengths are logged instead, which is all an AUTH_REJECT diagnosis ever needed. Thanks to @fabge (#134)
- **Nothing in the bank window could be dragged, and then it moved the wrong thing.** Two faults, one behind the other. `bankframe.lua` picks an item up with `PickupContainerItem(BANK_CONTAINER, id)` and `BANK_CONTAINER` is -1; both halves of that function read the container by hand and knew two of the six, so the pickup answered "nothing there" for every occupied square and the drop asked `wornBagContainer` for container 17. With those routed through the shared helpers the drag reached the *source* mapping, where -1 is two different places: the paperdoll records it for a worn item, and the bank's own slots are numbered with it. Every negative source read as equipment, so bank slot five went out as equipment slot four. The flag that tells the two apart was already beside the cursor and was not being read; the same hole made a worn helm dropped on bank slot one read as "back where it came from" and do nothing
- **The bank's bag row addressed the wrong slots, and the button that buys one did not exist.** A bag button's id is the container number of that bank bag - five through eleven, which `bankframe.lua` says itself by greying out an unbought slot with `(button:GetID() - 4) > GetNumBankSlots()` - and it was counted from one, so the row sat four slots along and a bag in the first slot had no button showing its icon. `PutItemInBag` took the four worn slots only, so a bag clicked onto a bank slot opened whatever was already there. And `BankFramePurchaseButton` was never created: Blizzard declares it `virtual="true"` while nesting it inside another frame's `<Frames>`, which the real client builds regardless - a template is declared at the top of a file, and a nested element has a parent, which is what an instance has. Six frames in the shipped interfaces are written that way and all six are real, `MovieProgressBar` among them, which `movierecordingprogress.lua` drives by name
- **The bank is not the same shape in every expansion.** Vanilla has 24 general slots and 6 bank bags where 2.0 onward have 28 and 7, and everything after the general slots moves with the count: turtle's `Player.h` puts `BANK_SLOT_BAG_START` at 63 and `KEYRING_SLOT_START` at 81 where 3.3.5 has 67 and 86. All of it was fixed at WotLK's figures, so on vanilla every bank bag operation named a slot four along and every keyring one five along. Only the sending side was affected - contents come off update fields, which are already per-expansion - and the 24 general slots were right by luck, both layouts starting at 39
- **Selling a stack at a vendor sold one of it.** All four sell sites asked for a count of one with the slot's own stack count in hand, and the auto-sell sweep added one unit's price to the total it reported. Thanks to @fabge (#136)
- **A chat message could be built out of nothing.** The declared length was checked against a ceiling and not against what the packet actually held, and `readUInt8` answers zero at the end of the buffer without advancing - so a length longer than the packet produced a NUL-padded line and reported success, and a length of 8192 or more skipped the message and read its first byte as the chat tag. Two `readSizedString` results were also discarded, where a false means the declared name length is a lie and reading on consumes the guid behind it. Thanks to @fabge (#135)
- **Rearranging bags on the bar with their windows open left them unusable.** `swapBagSlots` moved the bags and told the interface nothing, so an open `ContainerFrame` went on naming the container it was built for and a click in it acted on that one. The real client fires `BAG_CLOSED` and `ContainerFrame_OnEvent` hides that frame, which is the whole of what closing and reopening was doing by hand
- **The client's own addons have never been in a release.** The build copies `addons/` next to the binary and every staging step copied the binary, the extractor, the scripts, the assets and `Data` - so `WoweeAllBags`, which is the combined bag window and the Sort Bags button, existed in a local build and in no download. Both were drawn by this client until the bags were handed to FrameXML, which has neither
- **The login screen's buttons were drawn over the sheet's own edge.** 3.1.13 moved the rule under each heading down to clear the descenders and left the card's height computed off the em, so the sheet came out about a third of a title's height short of what was drawn into it and the footer, the settings gear and the close cross went over the bottom. Both halves read one measurement now
- **A stack could not be split by typing the number.** `StackSplitFrame`'s key handler passes digits straight through on purpose - they belong to OnChar, which builds the number up ten at a time - and OnChar was never dispatched: typed text went to the focused edit box and was dropped when there was none. It reaches the frame that asked for the keyboard now, one character at a time
- **Disenchanting asked whether to bind the item, and offered to destroy things it could not touch.** `completeItemUseOnItem` read the target and never the spell, so a bind-on-equip target raised "Enchanting this item will bind it to you" in front of a cast that turns it to dust. It asks the spell what it does now - disenchant, prospecting and milling take an item apart rather than change it - with the effect ids read off the client's own Spell.dbc rather than recalled. A disenchant asks its own question instead, and only about uncommon-or-better weapons and armour, which is the rule the server enforces
- **Shift-hovering compared nothing for an item whose info had not arrived.** The comparison reaches the worn item through `GameTooltip:GetItem()`, which answers from a field only one of the three tooltip-fill paths was setting - and that same path was the only one firing `OnTooltipSetItem`, which is where the comparison is asked for. So it was not comparing badly; it was never asked
- **A bag said "Container" and nothing else.** The slot count was not reachable from the tooltip builder, so `CONTAINER_SLOTS` - the client's own "%d Slot %s" - could not be written. The subclass names the kind, so a quiver and an ammo pouch read as themselves
- **A burst of warnings cost a write syscall per line.** Every warning flushed the stream by itself, and this client puts its diagnostics at warning on purpose: one session's log was 1,270 lines in 48 seconds and all but five of them warnings, including 112 while FrameXML loads and 200 for a single takeover check. Warnings take the same 250ms interval as everything else now, which turns a burst into one write while still writing a lone warning immediately; errors and worse still flush at once, because a crash runs no destructor
- **The server's time was thrown away before WotLK.** `SMSG_QUERY_TIME_RESPONSE` was refused unless it carried eight bytes and turtle's own server builds it four long, so the whole response was dropped and the client never learned the time from the one packet that carries it
- **A quest in progress had no marker on a 1.12 client.** `IncompleteQuestIcon.blp` is 2.x's art; a vanilla `GossipFrame` holds `ActiveQuestIcon` and `AvailableQuestIcon` and nothing else, so the load failed twice and the head stayed bare. It falls back to the active mark
- **An inspect on a 1.12 realm reported itself as a truncated packet.** The guid is the whole answer there, which is the shape rather than a short packet, and every inspect wrote a warning into a log that is read for faults
- **`SMSG_SPELL_GO`'s target lists are full guids.** Restoring the order after #135 turned it round: turtle's own `WriteSpellGoTargets` writes both lists with the operator that streams a `uint64`, and packed is written explicitly through `GetPackGUID` for exactly the two fields at the head - the two this parser already reads as packed
- **A sweep probe that could not see its subject failed instead of skipping.** Building the headless FrameXML runner on a checkout that has never extracted the game turned eleven probes on against an interface that had not loaded, and five failed on frames that do not exist. `Data/` is committed while the interface is not, so they ask for `Data/interface` now - what `missing_input` already requires of the sweeps beside them

## [v3.1.13] - 2026-08-28

### Fixed
- **The bank had no tooltips, its bag row addressed the wrong slots, and the button that buys a slot did not exist.** Three faults in one window. `GameTooltip:SetInventoryItem` refused a slot above nineteen and a bank slot's id lands at forty and up, so none of the twenty-eight general slots answered a hover - and the bag row along the bottom only looked like the exception, because a bag button falls back to `GameTooltip:SetText(self.tooltipText)` when this returns false, which is the "Bank Bag" line rather than the bag's own tooltip. `BankButtonIDToInvSlotID` counted a bag button from one, and a bag button's id is instead the container number of that bank bag - five through eleven, which `bankframe.lua` says itself by greying out an unbought slot with `(button:GetID() - 4) > GetNumBankSlots()` - so the row sat four slots along, and a bag in the first slot had no button showing its icon. And `PutItemInBag` took the four worn slots only, while `BankFrameItemButtonBag_OnClick` opens with it and falls through to `ToggleBag` on a false, so a bag on the cursor clicked onto a bank slot opened whatever was already there instead of going in
- **`virtual="true"` on a frame nested inside another frame is a mistake, and the real client builds the frame anyway.** A template is declared at the top of a file; a nested element has a parent, and a parent is what an instance has. Six frames in the shipped interfaces are written that way and every one of them is real - 3.3.5 has `BankFramePurchaseButton`, `GuildFrameLFGButton` and `MovieProgressBar`, which `movierecordingprogress.lua` drives by name, and 1.12 has `BankFramePurchaseButton` and both LFG buttons - and none is inherited by anything, anywhere. The bank's purchase button is the one that shows: it was registered as a template and never created, so the whole path behind it - `PurchaseSlot`, `GetBankSlotCost`, `GetNumBankSlots`, the confirmation dialog, `CMSG_BUY_BANK_SLOT` - was bound and working with nothing to start it. Emitting both full interfaces either way differs in three files and two
- **Rearranging bags on the bar with their windows open left the windows unusable until they were closed and reopened.** `swapBagSlots` moved the bags and told the interface nothing, so an open `ContainerFrame` went on naming the container it was built for and a click in it acted on that one. The real client fires `BAG_CLOSED` with the bag's number and `ContainerFrame_OnEvent` hides that frame, which is the whole of what closing and reopening was doing by hand
- **Disenchanting an item asked whether to bind it.** `completeItemUseOnItem` is the one path for every spell cast on an item and it read the target rather than the spell, so a bind-on-equip target raised "Enchanting this item will bind it to you" - the wrong question twice over, since nothing is being enchanted and the item is about to be dust. It asks the spell what it does now: disenchant, prospecting and milling take an item apart rather than change it, and neither enchant warning belongs in front of one. Effect ids read off the client's own Spell.dbc rather than recalled - 13262 is 99 in 1.12 and in 3.3.5, 31252 is 127, 51005 is 158. A disenchant asks its own question instead, which the real client does not ask at all: the item is gone and there is no undoing it. Prospecting and milling keep the silence, being spent five at a time from a stack
- **The red rule under every heading on the login screens sat inside the letters.** The four headings place it one `titleSize` below the top of the text, which was right while ImGui squeezed a face's whole ascender-to-descender span into the size it was given. 3.1.11 made a size mean an em, so the ink reaches further down than the number names: Morpheus spans 2937 units of a 2167-unit em, and the rule was landing a third of a title-height up among the descenders. `inkHeight` asks the baked face how far its ink reaches and the headings place the rule, and what follows it, off that
- **A guild vault opened for a player in no guild.** `GUILDBANKFRAME_OPENED` went out the moment the object was used, before anything had asked. The server answers `CMSG_GUILD_BANKER_ACTIVATE` from a guildless player with an error and sends no list, so the window had nothing behind it and never would - tabs to click, no contents, and nothing to tell it from a vault whose contents had not arrived
- **An addon timer ran its OnUpdate against state its OnShow had not set.** Turtle's transmog frame is `CreateFrame`, `Hide`, and only then `SetScript` for OnShow, OnHide and OnUpdate. Hide counted a toggle for a frame with no OnHide to run, so the Show that followed was the second toggle - and Show fires OnShow itself only on the first, leaving the second to the visibility pass, which answers a pair of toggles only for a frame that is `visible`. A driver frame is never that: no anchors, nothing drawn, an OnUpdate and a pair of handlers to arm it. The OnUpdate pump runs those off the chain, so the two passes disagreed about whether the same frame was running. Hide owes nothing where nothing is hooked to it, as the real client owes nothing there either, and the pair branch reads the chain. Reported in #132
- **An edit box could not say what its own limit was.** `SetMaxLetters` and `SetMaxBytes` have been bound since that widget was written and neither getter ever was, so a handler asking called a nil and lost the rest of itself - turtle's options search box does exactly that from OnTextChanged, which is the handler that would have run the search. Reported in #132
- **A `<Script>` or `<Include>` naming a file that is not there said "cannot open".** Which reads as a lock or a permission rather than as a file the install does not have. It names the places it looked now. What one missing file costs is not one file: `STATICPOPUP_NUMDIALOGS` is declared in `StaticPopup.lua`, so without it `WorldFrame_OnUpdate` raised on its first loop and was disabled after five, and every static popup in the game was dead. Reported in #132
- **`--dbc-csv` converted nothing at all.** It looked for the extracted DBCs in `<out>/db`, which is where it *writes* the CSVs, so every name reported "Missing extracted DBC". They come out in `dbfilesclient/`, lowercased like every other extracted path, which is what it reads now. Reported in #132
- **Asking a DBC layout whether it has a field was reported as the layout missing one.** Spell.dbc carries `SchoolMask` from 2.x and `SchoolEnum` before it, and the spell book asks for both and uses whichever answers - so a 1.12 layout printed "does not declare 'SchoolMask'", and that report tells the reader their `dbc_layouts.json` is stale and to re-extract, which for a column their client never had is wrong twice over. `tryField` answers without reporting; a caller with nothing to fall back on still reports. Raised as a bug against a fresh extraction in #132
- **A log line could stop grep reading the rest of the file.** `race=` and `sex=` are `uint8_t` and were written as characters, so race 9 printed as a tab and sex 0 as a NUL - and a NUL makes grep take the whole log for binary and print nothing for any pattern in it, which is most of what a log is for. Found in a log attached to #132

## [v3.1.12] - 2026-08-28

### Fixed
- **The loot window's rows were built, and were not buttons.** 3.1.11 read `<LootButton name="LootButton1"/>` as an element named after a virtual `<Button name="LootButton">`, which is not the shape the file has: 1.12's `LootFrame.xml` declares `<LootButton name="LootButtonTemplate" inherits="ItemButtonTemplate" virtual="true">` and then `<LootButton name="LootButton1" inherits="LootButtonTemplate">`. Nothing declares `LootButton` at all, and `UI.xsd` does not define it either - the real client reads the type off what the element inherits, and all three of these are Buttons because `ItemButtonTemplate` is one. Created as Frames, the template's own `RegisterForClicks` and `<OnClick>` had nothing to act on and a loot row did not answer a click. The chain is walked when the frame is created rather than when the template is declared, so a template in a file loaded later still answers. Emitting the whole 3.3.5 interface either way differs in one line, which resolves the same both ways. Reported in #132
- **The chat menu's channel list broke at the second channel.** `GetChannelList` answered an id, a name and a disabled flag per channel to every interface, and 1.12's `FCFDropDown_LoadChannels` reads two at a time - `for i=1, arg.n, 2`, then `info.value = "CHANNEL"..arg[i]` - as does 3.3.5's `CreateChatChannelList`, written with select. So the flag was taken for the next channel's id and concatenated: "attempt to concatenate field '?' (a boolean value)", and the rest of the handler was lost. The chat menu's list on turtle and the chat config panel's on wotlk, the same fault in both. The flag is 4.0's and 4.0 keeps it. Reported in #132
- **The language menu was empty on a 1.12 interface.** Stock `ChatFrame.lua` opens `LanguageMenu_LoadLanguages` with `GetNumLaguages()` - Blizzard's own misspelling, in the shipped files, so the client they were written against answers to that name. Bound here under the correct spelling only, the missing-API fallback answered an object and `for i = 1, numLanguages` raised. Reported in #132
- **Hovering the minimap's tracking button killed the handler, and the button showed a missing icon.** `GameTooltip:SetTrackingSpell` was in neither the method table nor the no-op allowlist, so the lookup answered nil and 1.12's OnEnter raised on its second line, with the tooltip already owned by a frame that then wrote nothing into it. It renders the spell's own tooltip now. And `GetTrackingTexture` answered `Interface\Minimap\Tracking\None` for nothing tracked, which is 3.x's art: a 1.12 client has no `Interface\Minimap\Tracking` folder at all, and its `Minimap.xml` hides the tracking frame outright when this answers nil. Only 1.12 changes - the tracking dropdown and its None entry are 2.x's. Reported in #132

## [v3.1.11] - 2026-08-27

### Added
- **Bit-level packet access, and an expansion id with no parsers behind it now says so.** 4.x marshals a packet as a bit stream interleaved with the byte stream in one buffer, and a GUID as a mask bit per byte followed by the non-zero bytes XOR'd with 1, each pass in an order belonging to the opcode. `Packet` had no bit accessors at all; it has `writeBit`/`readBit`, `writeBits`/`readBits` and the two GUID passes, which take the order as an argument because the per-opcode tables have to be read off a running core rather than recalled. A byte write flushes the open bit byte and a byte read closes it, so none of the 181 packed-guid call sites has to remember to. `createPacketParsers` used to hand an unrecognised expansion WotLK's parsers, which misparses rather than failing
- **The 109 Cataclysm movement layouts, derived from a 4.3.4 core rather than transcribed.** 4.3.4 chooses the order of the bit and byte streams per opcode with no pattern between them - `MSG_MOVE_START_FORWARD` writes position Y, Z, X then guid mask bits 5, 2, 0, while `MSG_MOVE_HEARTBEAT` writes Z, X, Y then pitch, timestamp and fall bits - and none of the 109 follows from another. `derive_cata_movement` parses the core's own `MovementStatusElements` arrays into `movement_sequences.json` and `cata_movement.cpp` executes it, because 109 hand-written readers would be 109 chances to transcribe one wrong

### Fixed
- **Every label in the interface was drawn and measured at 82% of its size.** ImGui asks stb_truetype to fit a face's whole ascender-to-descender span into the height it is given; `<FontHeight>` means the em square, which is what every traditional font API means by a size - stb's own header calls the em mapping "probably what traditional APIs compute". FRIZQT__ spans 1215 units of a 1000-unit em. The width is the half that moves things: anything anchored to a caption's right edge landed short by 18% of that caption's width, which is how the auction house's rarity dropdown came to sit on top of the item level boxes it hangs off. `ExtraSizeScale` carries the correction, read per face off its own `head` and `hhea`
- **The chat box drew what you typed in the wrong face, at the wrong size, with no shadow.** An EditBox, a message frame and a SimpleHTML draw text of their own, and the XML says how by putting a `<FontString>` straight in the frame rather than in a Layer. Thirty-nine do, `InputBoxTemplate` and the chat window among them, and the emitter only walked `<Layers>` - so all thirty-nine were dropped, where `ChatFontNormal` is ARIALN at fourteen with a black shadow. The setters it needs were no-ops on a frame - `SetFontObject`, `SetTextHeight`, `SetShadowOffset`, `SetShadowColor`, `SetJustifyH`, `SetJustifyV` - and the edit box and message paths drew no shadow at all
- **Nothing on the buff bar said what a buff does.** The aura tooltip gave a name, a stack count and a countdown and never the effect, so Power Word: Fortitude never mentioned the 165 Stamina. `SetUnitDebuff` had less still - no stacks, no time left - and is the same builder now with HARMFUL in its filter
- **A tooltip nothing had filled yet lost its title.** `SetText` was refused unless the flag only the fillers set was already on, and the first line of a tooltip is a `SetText`, so the frame metatable took the refusal as "this is not a tooltip" and stored the words where a button keeps its label. A GameTooltip is one before anything has been put in it
- **A mounted player looked to everyone else like one landing over and over.** Ground contact breaks for a frame at riding speed, and the frame after it read as a landing: `MSG_MOVE_FALL_LAND` went out, and every client in range plays that as a landing on the mount. Crossing open ground sent several a second. The airborne interval is counted now and a landing is reported only for one long enough to have been a fall
- **The guild vault opened on an empty panel.** `CMSG_GUILD_BANKER_ACTIVATE`'s `sendAllSlots` byte was zero and the tab query's full-update flag was false; both reach `Guild::SendBankList` as its "with content" argument, so the client asked the server for a bank and told it to leave out what was in it. The reader also gated the tab-name block on the update flag *and* on the list being tab zero, which is Cataclysm's rule - 3.3.5 and 2.4.3 write it under the flag alone, so a tab query answered with the names attached had the item count read out of the tab count and the items out of the names. `GUILDBANK_UPDATE_TABS`, `_MONEY` and `_WITHDRAWMONEY` are fired now; the first is what runs `GuildBankFrame_SelectAvailableTab`, which is what picks a tab the player may look in and draws it
- **A hunter with nothing selected shot himself until he died.** An empty `SpellCastTargets` is `TARGET_FLAG_SELF` on the wire, and the server reads that as "the caster is the target" - it fills the unit target in with the caster and casts the spell there. The cast builder wrote it for any absent target. A spell whose `EffectImplicitTargetA` names an enemy now raises "You have no target" and sends nothing, which is what the real client does; self-casts and fishing clear the target on purpose and are what that flag is for, and a friendly spell still falls back to the caster where auto self-cast asks for it
- **The cursor over a vendor was the plain hand.** Two faults, either of which was enough. `NPC_FLAG_VENDOR` was `0x04` - one of the two bits in the run that nothing sets - so no merchant in the game ever read as one; the rest of the run is written out beside it now, and the two spirit flags were the other way round, which no caller could see because both are only ever tested as a pair. And the cursor asked the picker for `closestGuid`, the nearest entry point of anything the ray touched: a game object's fallback sphere is 2.5 yards against a unit's 1.8, so the stall or forge a merchant stands at answered first. `resolve()` is what a click acts on and carries the unit-over-object bias
- **An achievement banner appeared and stayed for the rest of the session.** `AlertFrame_AnimateIn` shows it, plays three animation groups in a row, then plays the `waitAndAnimOut` whose `OnFinished` hides it - and the second call raised, so everything after it was lost. A `<Texture>` may carry `<Animations>` of its own and only a frame's were emitted, so the glow and the shine had no `animIn`; regions keep their methods on themselves rather than on a shared metatable, so `CreateAnimationGroup` is copied onto them too. And an animation's `parentKey` was written on the frame rather than on its group, so `frame.waitAndAnimOut.animOut` was nil - every one of the six sites that reads one in the interface reaches it through the group, as WoW does. The same template shape carries the dungeon-completion and BNet toasts
- **Turtle's whole options framework failed to compile, and took a run of names elsewhere with it.** `OptionsFrame.xml` pulls its two Lua files in with `<Include>`, and an include was handed to the XML parser whatever it was: "expected an element" on `Options\Options.lua` and `Options\Functions.lua`, and with them `OptionsFrame_AddCategory` and every panel that registers one. Which element named a file does not decide how to read it - the extension does, both ways round. Reported in #132
- **Turtle's loot window came up with no buttons in it.** `LootFrame.xml` declares `<Button name="LootButton" virtual="true">` and then writes `<LootButton name="LootButton1"/>`, which the real client accepts and this reported as an unknown type, building nothing inside it. A virtual frame records what kind of frame it makes now, and an element naming one is created as that kind and inherits it. Reported in #132
- **One client's asset tree answered for another's missing files.** The base fallback resolves per file with no check on where the file came from, and a thin expansion tree leans on it - the wotlk tree here holds 18,943 files against the base tree's 199,468, so nine lookups in ten are answered by the base. Cataclysm makes it visible rather than merely wrong: its Azeroth is the sundered one, so a tile a Cata tree does not cover was drawn as the old world with nothing said. `asset_extract` records the expansion in `manifest.json` and the client checks it
- **An area trigger that refused said nothing useful.** The near-miss report needed the player inside the trigger's own flat extent before it would say anything, and a portal box is three yards across - so "I walked into it and nothing happened" was never the case it covered. It reports a pass within a few yards now, and prints the three rotated axes the box test actually compared rather than only the height. The cooldown branch moves to warning level for the same reason: a suppressed portal and a broken one look identical from the chair, and the window is ten seconds after any arrival

## [v3.1.10] - 2026-08-27

### Fixed
- **3.1.9 crashed at login on every Apple Silicon Mac.** `pthread_jit_write_protect_np` answers a process without `com.apple.security.cs.allow-jit` by trapping rather than by failing, and the app is signed with the hardened runtime and no entitlements at all, so the first Warden module the realm sent took the client out in `JitWriteWindow`'s constructor. Two things were wrong at once. The window was gated on arm64 macOS while the mapping it guards is gated on macOS *and* no Unicorn, and release builds have Unicorn, so the image was never mapped `MAP_JIT` and the window was pure liability; both now read one `WOWEE_MAP_JIT`, so the guard cannot come apart from the thing it guards again. And the entitlement is granted, because a build without Unicorn does map `MAP_JIT` and cannot without it. `verify_signature.sh` fails the release if the entitlement is not on the signature, which nothing checked before: a missing entitlement is not a signing error, so every existing check passed on the build that crashed. Reported in #131
- **A random suffix named no stats.** The suffix tooltip read `itemStatName`, which answers nothing for Strength through Spirit because a query response prints those from fields of their own. A suffix has no such fields, so "of the Boar" and every other animal suffix listed a name and nothing under it. `itemModStatName` names them
- **An item dragged out of a bag onto the world came back on the next bag update.** The drag release cleared the cursor whenever it landed on nothing, which is right for an action taken off a bar and wrong for an item out of a bag. The delete confirmation reads the cursor after the dispatch returns, so it found nothing to ask about
- **The auction browse tab's column headers sorted nothing on a single page.** Clicking one re-queries with the ordering attached, which is that tab's design, but AzerothCore only sorts once the result runs past one page, so a search returning fifty rows or fewer came back in the realm's own order however the headers were clicked. The page is sorted before it is drawn with the keys the click already set, and a page the realm ordered is left alone
- **An action-bar slot naming a key never found it.** `useItemById` walked the backpack and the four bags only, and a key lives in neither. `/use` with an equipment slot number reads that slot's item id and passes it here too, so the one call site written for equipped items could not work either. Both are searched last, so nothing that already resolved moves
- **The objective tracker listed quests from every zone.** `trackerFilter` defaulted to 7, which ticks "quests in other zones"; three is the stock value. Clearing that bit filters on `CURRENT_MAP_QUESTS`, which `WatchFrame_GetCurrentMapQuests` built from the POIs the realm had sent rather than from the player's zone, so that is rebuilt from the quest log's zone headers
- **Scrolling up scrolled down.** `hybridscrollframe.lua` is `if delta == 1 then up else down`, so a wheel reporting 2 or 3 went the wrong way. Down was never affected: every negative delta fails that test into the branch it wanted. The camera keeps the magnitude
- **The helm and cloak switches came back on at every login.** `helmVisible_` was written by `toggleHelm` and nothing else, so it started true each session while the realm still had the flag set. It is read from `PLAYER_FLAGS` on create as well as on change, because login delivers the player as a CREATE block
- **Every interactable answered the same hand cursor.** A vendor shows `Buy.blp`, a hostile the attack cursor and a corpse the loot cursor
- **Every session began windowed with vsync at its default.** The window is built from a `WindowConfig` written by hand, so the saved fullscreen and vsync choices were never read. They are applied beside `applySavedAntiAliasing`, before the first frame
- **An item dropped into the auction sell slot had no icon, and Create Auction stayed disabled.** `AuctionSellItemButton_OnEvent` is the only thing that draws that slot, it runs on `NEW_AUCTION_UPDATE`, and nothing fired it. `ValidateAuction` reads the stack and total counts that handler sets
- **A quest item could not be used at all.** `UseContainerItem` chose equip on `inventoryType` alone; some quest items carry one, so the right-click sent `CMSG_AUTOEQUIP_ITEM` and the realm answered `ERR_NOT_EQUIPPABLE`. Class 12 is never equipment
- **One frame of whatever memory held when the world map was opened.** The composite image's contents are undefined until the first composite pass and `ImGui::Image` drew it unconditionally. A zone change is not this case: `invalidateComposite` only says the picture is stale
- **The world map dropdowns came up empty.** `SetMapToCurrentZone` raises a recenter flag and fires `WORLD_MAP_UPDATE` at once, but the move happens a frame later, so `GetCurrentMapContinent` answered 0 while the dropdowns were being built. Zero is the branch that runs `UIDropDownMenu_ClearAll` and asks `GetMapZones(0)`, and nothing asked again. The map now says when it has moved, and `setMapByIndex` returns whether the pair was taken
- **An off-hand weapon swung with the main hand's animation.** Both places that asked for a dual wield tested `ONE_HAND` alone, so an off-hand-only weapon (INVTYPE 22) set no `hasOffHand`: no off-hand animation and no unsheathe from that hand. The off-hand swing also chose its chain from the main hand's `isFist`/`isDagger`, so a sword and a dagger each swung with the other's

## [v3.1.8] - 2026-08-21

### Changed
- **The interface is FrameXML's now, and the client no longer draws one of its own.** The last twenty-odd elements changed hands this release: the bags, the character sheet, the spellbook, the quest log, the objective tracker, the party frames, the micro menu, the game menu, the friends list, the who list, the guild roster, the trade window, the ready check, the page text, the talents, the totem frame, the dungeon finder, the battleground scoreboard, the raid warning, the achievement badge, the inspect window, the loot roll and thirteen static popups. Each was a window this client drew beside FrameXML's own, so each was being asked twice or drawn twice; what is left on this side is five shared surfaces the client renders *into* a frame FrameXML owns - the minimap, the world map, the zone text, the taxi picker and the settings panel behind the game menu
- **`WOWEE_FRAMEXML_UI` is gone.** It named the elements FrameXML drew instead of this client, one at a time, so a replacement could be tried and backed out. That choice no longer exists: the client's own version of forty-seven of the fifty-two elements has been deleted, so `=none` had stopped meaning "use the client's interface" and started meaning "draw nothing at all". `frameXmlOwns` now answers on whether FrameXML loaded and whether an element was handed back unbuilt. `WOWEE_LOAD_FRAMEXML=0` still turns the whole interface off, and there is nothing behind it
- **Warden's module signing key comes from the expansion profile.** It was one hardcoded constant, Blizzard's, extracted from a retail client - so a realm that builds and signs its own module logged a decrypt failure on every login and carried on, the check having no answer to give but "these differ". `expansion.json` may name the realm's key as `wardenRsaModulus`; omitting it keeps Blizzard's, which is what a realm running a genuine module wants. A key that is not exactly 512 hex characters is refused rather than half-read

### Added
- **`this`, `event` and `arg1`..`argN` for interfaces written before 3.0.** A 1.12 or 2.4.3 handler takes no parameters at all - its body is `this:RegisterEvent("VARIABLES_LOADED")` and `if ( arg1 == "player" )` - and reads the frame, the event and the payload off globals the client published before making the call. The convention is chosen from FrameXML.toc's own `## Interface:` number before the first file is read, published around every script dispatch, and saved and restored so a handler running inside another does not take the outer one's arguments with it. `getglobal`, `setglobal` and `gfind` come with it
- **GPU timestamps per pass.** The CPU stage timings showed `beginFrame` and `endFrame` taking 45% of a 25ms frame, and both of those are the CPU blocking - so the client was GPU bound and nothing measured where the GPU time went. Marks are written at each pass boundary into a per-frame-slot query pool and read when that slot comes round again; terrain, grass, WMO, characters, M2, water and shadows each report their own. Support is checked on the timestamp period *and* the graphics queue's valid bits, because a queue can report zero on a device that otherwise timestamps
- **A sweep for bindings that refuse a player's action in silence.** Three of the four faults found by hand this week were the same shape: the action arrives as a Lua binding, the binding declines, and nothing is said - so a verb that refused and a verb that was never wired look identical from outside. `silent_refusal_check` reads the 134 bindings that call an action verb on the game handler and reports the guarded returns that sit before the first thing the binding does

### Fixed
- **Vanilla and TBC interfaces put every frame on the screen at once.** With `this` unset, the first line of nearly every OnLoad in a 1.12 file raised - so no frame registered for an event, positioned itself or hid itself, and the whole interface arrived unarranged and unclickable. It became visible at v3.1.7, which is the release that removed the client's own frames and made FrameXML the only interface there is
- **A lost GPU device a few seconds into the character preview.** `descriptorInfo()` returns a texture's handles exactly as they are and declares `SHADER_READ_ONLY_OPTIMAL` over them regardless, so a texture whose upload or view creation failed wrote `VK_NULL_HANDLE` into a live descriptor. Sampling that is undefined behaviour and reaches an NVIDIA driver as a graphics engine exception, not as anything this client can catch. `isValid()` did not cover it either - it checked the image and the sampler and not the view, and the view is the handle that goes missing on its own, because `createImage` logs its failure and returns the image anyway. Every descriptor write of that shape now substitutes the known-good fallback and drops the set rather than the device when even that is unsampleable. Reported in #123
- **Every whisper the player sent was addressed to the player.** `CHAT_WHISPER_INFORM_GET` is "To %s: " and takes the same argument `CHAT_WHISPER_GET` does - the other person, in both directions - and the sender's own name went there
- **A whisper arrived with no sender at all.** `SMSG_MESSAGECHAT` carries a guid and no name, and the name is looked up from the cache, a nearby entity, the party or the guild roster - a whisper is characteristically from none of those. A line whose sender is unresolved now waits for the name query rather than going out empty, released the moment the name lands or after a second if nothing answers
- **An account that had never filed a GM ticket was told at every login that it had been suspended.** Status `0x0A` is `GMTICKET_STATUS_DEFAULT`, the answer when there is no ticket, and it was read as "suspended"
- **Eleven zones played another zone's ambience.** The area-id table was checked against `AreaTable.dbc` rather than remembered. Area 4 is the Blasted Lands and was filed under beach as "Barrens coast", which is how a blighted red waste came to have seabirds and surf over it; Icecrown played jungle, Zul'Drak answered for the Storm Peaks, and both draenei isles were off by one
- **A random suffix survived nowhere.** Bracers of Arcane Protection sat in the bag as plain bracers with nothing on hover to say otherwise: the tooltip is built from the item *template*, which has no suffix and no roll, and both are per-instance. The suffix is on the title line now with what it rolled in green underneath, and `GetContainerItemLink` carries it - and the permanent enchant - so a shift-click into chat describes the item in the slot
- **The rested bar went purple with rested experience still to spend.** `GetRestState` answered from the rest-state byte, which is whether the player is standing in an inn; the server clears it on walking outside while the pool is spent over the next several levels. It answers from the pool. `IsResting` keeps the byte, which is what the "Resting" indicator means
- **Right-clicking a bag item at a vendor put it on the cursor instead of selling it.** That is how 3.3.5 sells: the item is picked up and `PickupMerchantItem()` is called with no argument, and index zero means "put what the cursor is holding into the sell slot". This returned early on it, so the item was lifted, the sale was asked for, nothing answered, and it stayed stuck to the pointer
- **Dragging a food off the action bar into a bag moved the player's equipped bracers.** The cursor recorded where its item came from as a container number and every negative value meant the paperdoll, so a food lifted off button nine read as worn equipment in slot eight and the drop sent a swap naming the bracers as its source. An action slot holds a reference to an item rather than the item, so there is no source slot at all
- **Profession windows would not open, and then Create did nothing.** Two faults stacked. `Spell.dbc`'s reagent and created-item columns are read by name from `dbc_layouts.json`, and the copy in the extracted data directory is written once at extraction and never refreshed - the installed one predated both columns, so no spell had a reagent or a created item, every recipe filtered out, and no window would open with nothing in it. A field the layout does not declare now says so once, naming the file. Under it, `tradeSkillRows` returned the cached list by value while every one of its nine call sites took a pointer into that temporary and read it on the next line, so the whole panel was reading freed memory - which is what sent row 4 with an empty name and a spell id of 2661056113 to the server. It returns a reference, and an rvalue overload makes the old shape a compile error
- **The pet menu never offered to rename a pet.** A dead code path cleared `PET_RENAMEABLE` every frame to set two members nobody read, and `PetCanBeRenamed` answers from that same flag - so the interface asked after it had been taken
- **The macOS asset extractor could not open Terminal, and there was nowhere to say yes.** Sending an Apple event needs the Automation permission, and an app whose `Info.plist` carries no `NSAppleEventsUsageDescription` is refused outright rather than prompting. It asks LaunchServices to open the script instead, which needs no permission. The launcher also looks in `~/Applications` now, and the readme says to copy both apps out of the disk image - a permission granted to something running from a mounted image does not stick. Reported in #117
- **Opening the graphics options moved a setting nobody touched.** The sheet drops a panel of controls under a pointer that is still holding the button down, and at 1280x760 the ground-clutter slider lands where the gear was. A control can now be told to ignore the rest of the press that put it there
- **Two faults in Warden's native relocation loop, both reachable from module bytes the server supplies.** The bounds check added four to a `uint32` target, so `FF FF FF FF` wrapped to 3 and let the write land at image + 0xFFFFFFFF; and the absolute form kept its form flag in the value, so every absolute entry came out past the end of any image the parser accepts and was rejected. From #122
- **A module that will not unpack says why.** "Could not parse copy/skip pairs (all known layouts failed)" was the whole account, and the module cannot be attached to a report - it arrives encrypted under a key that is not kept. The log now carries the decompressed size, the header's own figure, the leading bytes, and per layout how far it got and which way out it took. A fourth layout and a search for a longer header come with it, both accepted only on an exact fill
- **`framexml_run` could not be built on a Homebrew prefix.** It is built from wowee's own target properties, and three of the four were copied - libraries, includes, definitions - but not the link directories, which is where pkg-config puts the path for a bare library name
- **Eight of the FrameXML sweeps read an interface that was not there and reported a clean tree.** `ROOT` was one contributor's absolute home directory, so each of them ran on exactly one machine - and `loaded_files` on a directory that does not exist returns an empty set, which is indistinguishable from a sweep that looked and found nothing wrong. Root is derived from the file's own location now, the way every other tool here does it, and the interface directory can be named on the command line so a sweep can be pointed at a particular expansion's. `framexml_live_stubs` was reading the defaults list out of the takeover source as well, which went with `WOWEE_FRAMEXML_UI`
- **A macOS CI failure on Apple's developer agreement read as a broken build.** notarytool's output is captured and the 403 named, so an account problem is distinguishable from a build that does not compile. The policy is unchanged: a master build that cannot be notarized still fails

### Removed
- **The GM command window and the social panel.** The first had been unreachable since the micro menu changed hands - its only opener was a button on this client's own bar - and its command reference is `/gmhelp` already; Max Out Character had no counterpart and becomes `/maxout`. The second had owned the party frames, boss frames, guild roster, friends list, dungeon finder and who window, all of which are FrameXML's, leaving one line that asks the interface to inspect the target
- **Twenty-four members nothing ever read**, and with them `WardenModuleManager`, which was constructed once and never touched again - its constructor created a cache directory on every start for a cache with no writer. The stored-and-never-read ceiling is zero
- **`window_flag_check` and `window_route_check`.** Both watched for this client opening a window of its own without asking who owns it, and every verb they scanned for has been deleted along with the window it opened. Each was already refusing to report a clean tree on an empty population, which is the right answer for a matcher with nothing left to match

## [v3.1.7] - 2026-08-19

### Changed
- **The screens before the game are drawn rather than asked for.** The login, realm selection, character selection and character creation screens no longer use ImGui widgets. What reads as ImGui is the shape of the controls themselves - the flat rectangles, the label on the right, the uniform spacing - so restyling could not have fixed a grey title bar and a drag handle sitting on a crayon drawing of a tavern. `paper_ui.cpp` is the smallest set of controls those four screens need, drawn into an ImDrawList as ink on paper: sheets with a hand-drawn border and taped corners, fields ruled like a form, a crayon button, pills, dropdowns, lists and sliders. The waver in every line is hashed from where the line is, so a control is the same shape every frame rather than boiling. The login screen leads with the account and the password and puts the address, port, expansion and asset override behind a disclosure; character creation lays its preview, its identity and its appearance out in three columns so it fits without scrolling. The login art now carries through all four screens instead of giving way to a black window after login
- **The caret and the selection are this client's own.** `text_edit.cpp` is the half of a text field that has nothing to do with drawing - where the caret sits, what is selected, and what Ctrl+Left does when the caret is already at the start of a word. It needs no font, no device and no frame, so it is tested: word motion through a hostname, a paste that arrives with a newline on the end of it, and a byte cap that must not cut a character in half. Every motion lands on a codepoint boundary, because half of a multi-byte character is not a place a caret can be
- **Grass thins by octaves rather than by a dice re-rolled every rebuild.** The re-roll was the pop: a ride toward a stand had blades blinking in at the rebuild cadence. Each distance doubling past the 45-yard near field keeps one cell in four by nested descent, so the far blades are always an exact subset of the near ones, and each blade carries its own fade distance and sinks by live distance from the player. Build cost grows with the log of the window rather than its area, which pays for the rest: the default range rises to 150 yards and the slider reaches 2000
- **Defaults sized for the machine.** Shadows defaulted to 4096, which is 64 MB a map and two in flight - 128 MB of depth before anything is drawn, refilled every frame, and a good part of the reports of this client running badly is hardware that was never going to carry that. The default is 2048, and 1024 on Android where GPU memory is the system's memory. The asset file cache took half of available RAM, which is a desktop rule; Android kills a process that passes its per-app limit rather than swapping, so a phone with 8 GB was being handed 840 MB
- **macOS is built and published for x86-64 as well**, and the retired macos-13 runner is replaced by macos-15-intel

### Added
- **A frame budget that reports itself.** The per-stage timings only spoke above 50ms, which names the stage that stalled and says nothing about where a frame's time goes. On a phone the whole budget is 33ms, so every stage was silent and the client was slow for reasons nothing reported. Each stage keeps a running average and worst case, and the breakdown prints every ten seconds

### Fixed
- **The character fell through the world on every login on a phone.** Asking for a height answers nothing both for a hole, which the character should fall through, and for a tile that has not streamed in, which it should not. Gravity could not tell those apart, so a device slow enough to enter the world before the ground arrived fell from the spawn point until the server killed it. Not gated to Android: it is a race, and a slow disk can lose it anywhere
- **A stalled package mirror read as a broken Windows build.** The x86-64 job installed its dependencies through the setup action's list, which has no retry, so one mirror at "less than 1 bytes/sec" failed the whole transaction before a single file was compiled. It now uses the retried step the arm64 job and the release workflow already had
- **The login background leaked its decoded image on every run** - 2.4 MB of pixels that nothing freed after they were copied to the GPU
- **A disconnect window that could never appear.** `AppState::DISCONNECTED` is set by nothing; a dropped world connection returns to login and says why there. What stood on that state was an ImGui window whose Return to Login button had no implementation

### Removed
- **The player, target, pet and focus frames, the cast bar, the buff bar, the boss frames, the durability warning and the error text.** All nine are the interface's own by default and clean in the readiness report, so around 3,200 lines behind their gates were the version nobody saw. Suppression goes with each of them: it hides the interface's frames for elements this client still draws, so leaving those rows would have hidden the mirror timers, the combo points and the boss frames with nothing left to draw them. The error sound stays - the interface plays none for a refusal, so removing the callback with the overlay would have made every one of them silent

## [v3.1.6] - 2026-08-18

### Added
- **The client runs on Android.** One arm64 APK built from the same tree, and it reaches the login screen, character selection and the world on a Pixel 9a. The release workflow builds and publishes it alongside the desktop archives. Game data is still the player's own: `tools/android/make_minimal_data.py` cuts an 18 GB extraction down to a profile a phone will hold, from 787 MB for the login screen and character creation to 12 GB with every map
- **On-screen controls.** Left thumb moves, right thumb steers, two fingers zoom. The stick drives the movement keys through a virtual key layer, so the movement state machine, the animations and the packets that announce them run exactly as they do from a keyboard. It is sized in density independent pixels, asks for more deflection sideways than forward, and while a finger is steering the character faces where the camera does
- **The interface scales itself to the display.** Density decides how much bigger than a desktop layout the client's own panels are drawn, held back so the tallest dialog's buttons stay on screen. The window scale setting reaches 3x for the same reason, and its range now comes from the schema row rather than from copies of the numbers in the loader and the applier

### Fixed
- **A DXT texture was invisible on hardware that cannot sample it.** The blocks go to the GPU as BC1/BC2/BC3; mobile parts carry ASTC and ETC2 instead and sample a BC image as nothing rather than failing. Walls came up untextured and doodads solid black, while character and interface art was fine because the appearance composer builds those as RGBA. textureCompressionBC is probed with the other device features and the loader unpacks to RGBA8 without it
- **Device selection refused hardware the renderer already coped with.** samplerAnisotropy, fillModeNonSolid and the two FSR2 compute features were required of the device, and every one of them already had a fallback further in - the sampler clamps anisotropy off, the terrain wireframe is a debug view, FSR2 is a setting. Four soft degradations were being turned into one refusal to start. Selection now names every device it saw and what each one lacked when it fails
- **VMA was handed a Vulkan version newer than the headers it was built against**, which it asserts on. The two do not have to agree
- **The zone was named twice**, once above the minimap by the interface and once inside the circle by this client
- **asset_extract did not link outside Windows where StormLib is static.** StormLib compresses with bzip2 as well as zlib and leaves both to whoever links it, but only Windows was given bz2. Reported by JonasAlv against Arch in #118

## [v3.1.5] - 2026-08-17

### Added
- **Grass, drawn by the GPU.** Off by default and marked experimental where a player reads it, since it is new. What grows where is read out of the map itself: a chunk's layers are composited the way the terrain renderer paints them, the ground-effect id of whichever layer wins names the doodads that zone plants, and the blades take their shape and colour from those - so a field carries the foliage its terrain texture is painted with rather than one plant everywhere. The population is a world-anchored lattice, so a blade keeps its place while the camera moves instead of the field resetting under it, and the cull compacts survivors into a single indirect draw. Density, height and draw distance have their own page in the interface's options panel
- **Textures upload as compressed blocks.** Terrain, M2 and WMO all send DXT to the device as it comes, rather than decoding to RGBA8 and generating a mip chain on the way. Measured over a session rather than estimated from the files on disk: 2,328 textures, 72 MB uploaded against 495 MB decoded, 85% saved
- **A texture with one level skips the staging buffer** where VK_EXT_host_image_copy is present. The pixels go into the image and the layout moves on the host - no staging allocation, no memcpy into it, no queue submission and neither layout barrier. Mipped textures keep the old path, because their levels are built by a blit chain that needs the submission regardless
- **Barriers are recorded as synchronization2 dependencies** where the device has the extension, through a wrapper that lowers to the legacy call where it does not

### Fixed
- **Escape closed the game menu but could never open it.** Before asking the interface, this client closed menus, special windows and all windows itself and then skipped ToggleGameMenu whenever one of those answered - which is a fragment of the very cascade ToggleGameMenu runs, in the same order, ending differently. The press was consumed before it could reach the step that shows the menu. Where the interface owns the menu it is now asked and nothing else
- **The client listed the characters and died the instant the world was entered.** setupUICallbacks binds references by dereferencing seven owned pointers, four of which are only created if the asset manager initialises; when it does not they stay null and the first callback to touch one writes through address zero. The wiring is checked before any of it happens and names which piece is missing, the game data path being the usual reason
- **The chat opened behind an opaque white panel with the text lost in it.** The panel is a white image the interface tints, so the colour this client stores is the whole of what it looks like - and it stored white at full alpha until v3.1.4. Fixing that default left every install that had already written the white to disk unfixed, so the fault outlived its fix. A saved pure white is now read as unset rather than as a preference
- **The music kept playing with the soundtrack turned off.** The track picker filters this client's own files out of the pool when the setting is off and then falls back to the unfiltered list if that left nothing - which in a zone whose tracks are all ours is every time, so the fallback could only ever be reached when the setting was off and defeating it was its entire effect
- **A grass or minimap setting changed nothing, and changing the soundtrack applied them.** Every subsystem's apply hung off the branch of whichever setting had been touched last. Each latches its own now, and the file is read once per change rather than once per frame
- **A bag could not be dragged out of its slot.** Picking one up set this file's own cursor and not the one every drop target reads, so the drop saw nothing held and the click fell through to opening the bag instead of moving it. A bag dropped onto an occupied bag slot now changes places with what is worn there
- **The number keys cast page one's actions for the whole session.** The page they read was moved only by the pager buttons on this client's own action bar, which the interface replaced - so the bar on screen paged normally and the keys did not follow
- **Player mail showed an empty sender.** Mail from a player carries a guid rather than a name, and the resolver that turns one into the other had been reachable only from this client's own inbox
- **Any .blp on disk could read two megabytes out of an eight-byte buffer.** A BLP header states its dimensions and its mip sizes separately and nothing makes them agree, but the decompressors sized their reads from the dimensions; the bounds check meant to catch it added two uint32s from the file and wrapped. Game data is untrusted input. The decompressors take spans and skip a block that would run past the end, and the comparison is done in 64 bits
- **Two WMO chunk loops added file-supplied sizes in 32 bits**, so a large one wrapped past its own bounds check and the parse walked on. Mis-parsing rather than an overrun, since the readers below cap themselves, but widened to match what the array reader eight lines up already did
- **Textures, shader modules, descriptor sets and audio allocations outlived the device they came from.** Shutdown now releases them in an order that holds, and reports what is still allocated when it is done

### Changed
- **The frame fence is a timeline semaphore**
- **The Warden emulator's two builds agreed on nothing.** Five constants sat inside the `#ifdef`, so the stub branch could not name them and zeroed its stack base, heap base and both API stub addresses instead. Never observable, since the stub reads none of them, but a field added to one branch alone would have made it a real one
- **This client's own tooltip and item-icon cache are their own units**, rather than parts of a bag window that no longer exists
- **A large sweep for clang-tidy's findings**: designated initialisers, range-based `for` over index loops, `emplace_back`, default member initialisers, `bit_cast`, and `std::function` parameters no longer copied. Precompiled headers for the headers measured to cost, and every one of the 166 tests is sanitized rather than the first two thirds

### Removed
- **This client's own trainer, auction house, vendor, loot, mail, guild bank, bank and barber windows, and its action, stance, bag, experience and reputation bars.** All of them are the interface's by default, so around 4,700 lines behind their gates had not drawn in a long time. What those windows were carrying for the rest of the client stays: the barber's style lists, costs and apply are exposed as Lua services the interface's own panel is built on, an action slot holding an item's spell still goes out as a use rather than a cast, and the bags still open when a vendor does

## [v3.1.4] - 2026-08-15

### Fixed
- **The flight map stayed open for the whole flight.** The taxi window's flag is cleared the moment the activation is sent, so the TAXIMAP_CLOSED fired on the server's reply saw an already-closed window and never went out. This client's own map and node list poll the flag and closed; the interface's own flight map only hides on the event
- **The flight master had no portrait, and his window did not close when you walked away.** Two members left on GameHandler by the handler split were never assigned after it, and the guid the interact-NPC and walk-too-far checks read came from those. Of the four windows that close when the player leaves the NPC, the taxi one alone did not
- **The quest dialog stayed on screen after the reward was taken.** Taking it put the fields back and announced nothing, and the interface's quest frame hides on QUEST_FINISHED and nothing else
- **Bank slots kept showing an item after it was moved to the bags.** The slot was announced from the field parse, but what a slot holds is written by the inventory rebuild that runs after it, so the frame redrew from what the slot had held a moment earlier and was never told again. It took closing and reopening the bank
- **Half the heals in the game refused instead of casting on you.** The self-cast fallback asked whether a spell's aim was 21, and measured over the shipped Spell.dbc, Holy Light and Healing Wave are 45 while Hand of Protection, Beacon of Light, Levitate and Intervene are 57. Those went out at whatever was selected and came back refused
- **Scrolls and bandages were sent at the selected enemy.** The item path asked the same narrow question. A scroll now buffs whoever read it rather than raising a targeting cursor
- **The character was re-aimed at its target five times a second.** Auto-attack pushed a facing update for as long as the attack lasted, so turning away with the mouse snapped straight back and backing away from something while fighting it was impossible. The turn happens once, where the attack is commanded
- **The tailoring window was titled "Two-Handed Axes".** The skill name lookup read its argument as a spell id and followed SkillLineAbility, while every caller holds a skill line id: Tailoring is line 197, and spell 197 is a Two-Handed Axes rank. The trainer's required-skill text had the same fault
- **Every recipe showed the same icon.** The trade skill pane drew the recipe spell's own icon, and a crafting spell carries its profession's picture rather than its product's - every tailoring spell in the shipped file is the same one. It now shows what the recipe makes
- **A gap between the torso and the legs where a belt would be.** Geoset group 18 is erased and rebuilt on every equipment change, and the belt was the one group with no base variant to fall back to. The older human male carries nothing there to lose; the Legion model carries the waist itself
- **Training costs read as three numbers in a row.** The coin pictures are taken off every money frame, the backpack's own art already drawing them, which left nothing to tell gold from silver. Each amount ends in the interface's own letter again
- **Chat emptied itself while it was being read.** The chat frame template asks for lines to fade two minutes after they arrive
- **The interface's fourth action bar checkbox did nothing.** It is labelled "Right Bar 2" and drives the left bar - action page 4 - so it was never matched to the setting behind it, and turning that bar on from this client's own panel left the checkbox unticked
- **An action dragged off a bar went back to where it came from.** Dropping it away from the bar removes it now, as the real client does; a right-release puts it back

### Changed
- **A wand is not a gun.** One inventory type carries guns, crossbows and wands, and only the first two were told apart, so a wand was shouldered and fired like a rifle. WOWEE_WAND_ANIM overrides the shot animation while the right one is chosen on screen
- **The mature language filter is gone**, list and control alike. It took its words from eleven English ones written into this client, so it filtered unevenly and filtered for players who had not asked. The spam filter beside it stays

### Removed
- **This client's own crafting, taxi, stable, book, achievement, GM ticket, gossip and quest giver windows.** All eight are drawn by the interface now, so none of them had appeared in a long time. Around 1,600 lines, with their state, and the openers that reached them route to the interface's own panels

## [v3.1.3] - 2026-08-15

### Fixed
- **`.tele` arrived in the wrong place on Vanilla.** MovementInfo has three layouts, not two: WotLK writes moveFlags2 as a uint16, TBC writes it as a single byte, and Vanilla has no such field at all. The teleport acknowledgement read that byte for anything pre-WotLK, which is Vanilla and TBC alike, so on Vanilla it took one byte too many and shifted x, y, z and orientation by a byte each. The coordinates that came out were garbage from the same misalignment every time, which is why three different destinations all arrived in the same wrong region. Reported as issue 113
- **The world was frozen at midnight - stars out, no sun, all session.** Server game time defaults to 0 until the server says otherwise, and zero is a perfectly good time of day, so "never received" and "midnight" were the same number. Every reader tests it for being non-negative before trusting it, so they all trusted it, the fallback to the local clock became unreachable, and the sky never moved. The sentinel is negative now, which is what every call site already passes when there is no game handler at all
- **The character fell through steep hillsides.** The slope limit cleared the terrain floor when the ground was too steep to walk, and a floor that is not there is not a wall: the player did not fail to climb, they dropped through the hill and kept going. The limit no longer removes the ground. A slope limit belongs on the movement rather than on whether there is a floor, and that is still to be written
- **The screen could flash green or red.** The brightness overlay pushed sixteen bytes into a shader push block that had since grown a matrix at its front, so the colour landed in the matrix's first row and everything the shader read was whatever the previous draw had left there. It runs on every frame with brightness above neutral
- **Gamma applied but never persisted between runs.** Settings load before the renderer is injected, so one later block is the only place brightness ever reaches the pipeline - and it marked itself done as soon as the renderer existed, whether or not the pipeline did
- **Street lamps swayed like saplings.** "street" contains "tree", a trap this list already caught for StreetSign and not for StreetLamp

### Changed
- **The shadows toggle is off the settings panel and shadows are held on.** Turning them off loses the device within a second; GPU-assisted validation reports nothing before it goes, so the fault is inside a shader and is not found yet. A setting whose only effect is to end the session is worse than a setting that is missing
- **Fullscreen keeps the desktop's shape** when the chosen resolution is a different aspect, rather than stretching an ultrawide display to fit a 16:9 selection. Issue 112
- **The minimap's north markers are off the zone name** - both of them, since which one shows depends on the rotation CVar

## [v3.1.2] - 2026-08-14

### Fixed
- **The player trod water almost on top of it.** The feet floated 0.9 below the surface, which against a neck 1.6 above them put the waterline around the knee. It is 1.45 now, so the character treads with the shoulders out and the water at the chest, the way retail does
- **Swimming went one way, strafed another and faced a third.** Forward came from the camera's 3D direction while strafe and facing came from the character's own, so panning the camera pulled the three apart and left the stroke animation pointing wherever the body had been left. The camera steers in water now, the way holding the right button steers on land - but only while there is movement input, so treading on the spot and looking around does not spin the character
- **Foam opacity comes down by about a third**, colour and alpha together, since lowering either alone leaves it as opaque as it was and merely paler

## [v3.1.1] - 2026-08-14

### Fixed
- **The foam band was twice the width it should be.** It reached 1.8 yards of depth out from the waterline, a figure that had been tuned by eye against a depth measurement ten times too shallow. With the scale corrected the same number drew twice the band, so it is halved
- **The camera juddered and shoved along slopes.** Built geometry and the ground are handled apart now: a wall or a pillar has a side to step around and is worth reacting to sharply, while a hillside has neither an edge to clear nor a steady height - the marched ground moves a little with every step the player takes. The ground no longer makes the camera try to yaw around it, its own limit is smoothed before it is combined with anything else, and it keeps a smaller clearance, the quarter yard on top of that having been felt as a push on every rise

## [v3.1.0] - 2026-08-14

### Fixed
- **Water foam covered whole lakes in hard-edged sheets.** The water shader linearised the depth buffer against a near plane of 0.05 while the camera's is 0.5, so every depth it read came out about a tenth of its real distance. The shoreline masks are thresholds in yards - foam out to 1.8, the wet band to 0.7 - and against a depth ten times too shallow they matched water far out into the lake rather than a strip along its edge. What was left of a boundary followed whatever the depth texture did at the lake bed's own triangle edges, which is where the straight lines came from. The camera's planes are handed to the shader now instead of written out a second time
- **The water vertex and fragment stages described one push constant range two different ways**, the vertex stage naming a float where the fragment stage has a vec2. Nothing read it, so nothing was wrong yet

## [v3.0.9] - 2026-08-14

### Fixed
- **The player sank into gentle hillsides.** The floor query interpolated the four corners of a terrain cell, but the ground that gets drawn is four triangles fanned from that cell's centre vertex, and the height format puts that vertex wherever the terrain artist needed it - commonly a yard or two off the plane of its corners on a slope. The two answers disagreed by exactly that offset, so the floor came out below the visible ground. There is one sampler now, shared by the mesh builder, the floor query and the clutter scatterer, and it reads the same surface all three draw
- **The minimap's compass "N" sat on top of the zone name.** It is anchored to the middle of the minimap and lifted onto the rim, which is where this client writes the zone. Taken off: the minimap here does not rotate, so north is always up and the marker was repeating what the dial already said

## [v3.0.8] - 2026-08-14

### Fixed
- **Tall narrow trees could be walked straight through.** A trunk was only given collision when the tree was over six yards across as well as four tall, so a conifer - twenty yards tall and four across - failed the width half, was classed as soft foliage and had its collision turned off outright. Height decides it now, with the width rule kept for the shorter, broader trees it was written for. A standing trunk on its own is solid too; stumps and fallen logs keep their exemption, being things you step over rather than around

## [v3.0.7] - 2026-08-14

### Fixed
- **Characters cast the shape of their bounding box rather than their own outline.** The shadow pass bound one white fallback texture for every caster and left its alpha-test flag at zero - the flag was never written after the buffer was created - so hair, capes and cloaks stamped solid slabs into the shadow map. Alpha-keyed batches now bind their own texture and cut a proper silhouette, while opaque ones keep the white fallback and cast solid as before. The texture a batch draws with is worked out by the same code the main pass uses rather than a second copy of it

### Changed
- **F1 and F4 are development keys and are no longer built into a release.** Neither is a binding the player chose or one the interface knows about, so a stray F-key quietly turning off shadows or opening the performance overlay arrives as a bug report about rendering rather than about a keystroke. Both still work in a debug build

## [v3.0.6] - 2026-08-14

### Fixed
- **The character shadow pass declared a descriptor set nothing ever bound.** Its pipeline layout carried a dummy at set 0, which pushed its parameters to set 1 and its bones to set 2 - making it the odd one out of the four passes sharing the shadow render pass, the other three of which bind their parameters at set 0. Binding set 1 while set 0 still carried another pass's incompatible layout left the parameters disturbed rather than bound, so the fragment shader read its alpha-test flags from nothing, thousands of times a session. The dummy is gone and the sets have moved down to 0 and 1

## [v3.0.5] - 2026-08-14

### Changed
- **The camera steps around what it hits instead of only backing away from it.** Pulling straight in is what put the camera on the player's neck the moment they set their back to a wall, and from inside their own head there is nothing to steer by. A few degrees of yaw usually clears a pillar, a doorframe or the corner of a building outright; when it does not, the camera still pulls in, but only as far as the remaining clearance needs. The turn eases in and out rather than snapping. Two extra rays at most, only on a frame that is actually blocked, and only when the alternative buys a real yard of clearance

## [v3.0.4] - 2026-08-14

### Fixed
- **The camera went through WMO walls, floors and ceilings.** Its raycast dropped every hit more than 0.90 below or 0.80 above the pivot, and the camera orbits and pitches - so at any real pitch the far end of the ray sits metres outside that slice and the wall simply was not there. The band was aimed at floor geometry underfoot, which the surface test already excluded, so it was rejecting only the walls it was meant to be finding. The pass also considered the wall list alone, and then narrowed it again to near-vertical, which is why a storey's floor, a vaulted ceiling and anything leaning met nothing at all. All solid surfaces stop the camera now, and surfaces it is nearly parallel to are the only ones dropped
- **A character's shadow read a descriptor that was not bound.** The character shadow pass binds its parameters once before its loop, and runs after the M2 shadow pass, which binds a set at a lower index under a different pipeline layout - which disturbs the binding rather than keeping it. The fragment shader's alpha-test flags were being read from a set the GPU no longer considered bound, thousands of times a session. Found with the new `WOWEE_VULKAN_GPU_VALIDATION=1`

## [v3.0.3] - 2026-08-14

### Added
- **`WOWEE_VULKAN_GPU_VALIDATION=1` instruments the shaders themselves.** The plain validation layer only checks API calls, so a fault that lives inside a shader - an index past the end of a storage buffer, a descriptor read that was never written - leaves the log clean right up to the device being lost, which says nothing about where. Set alongside `WOWEE_VULKAN_VALIDATION=1`; expect a large slowdown

## [v3.0.2] - 2026-08-14

### Fixed
- **Bushes did not give way when walked past.** The player brush was gated on the sway mode a plant happened to use rather than on its size, so it reached grass and the smallest plants and nothing else. Every plant up to about head height parts now, tapering off from there so a tree still stands where it is
- **Small foliage had stopped moving at all.** Plants small enough to be sorted with ground clutter had their own animation disabled by the classifier *and* the shader wind withheld as clutter, which between them left them frozen. The two are told apart properly now: only detail doodads, which play a sequence of their own, go without the wind
- **The wind stepped between two sizes.** A plant either normalised its sway against its own height or against a flat twenty yards, so a bush an inch over the line swayed roughly ten times less than its neighbour. It is interpolated across the range now; grass and full-sized trees both keep the amount they had
- **F4 toggled shadows on the key repeat.** Held down it flipped some thirty times a second rather than once, which F8 already guarded against and F4 did not

## [v3.0.1] - 2026-08-14

Ground cover, and four things found while making it move.

### Added
- **Ground clutter parts around the player and springs back.** Grass and weeds within reach lean away from whoever walks through them, quiver while that person is moving, and recover over the following third of a second. The bend reads from the player's position and from a trailing wake position, taking the stronger of the two, so standing still does not bend the cover twice as far as walking does. Clutter at a different height is left alone, so flying over a field does not flatten it

### Fixed - ground cover that was there and could not be seen
- **Most of every tile had no ground cover at all.** The per-tile ceiling was a running total spent in chunk scan order, so a tile's whole allowance went to its first few rows of chunks and the rest got nothing. Measured against Mulgore's own layers that filled about six of sixteen rows and left ten empty, which is why standing in the wrong part of a tile showed bare ground. The ceiling is shared out per chunk now: it bounds the tile as it did while covering all of it
- **There is four times as much of it.** Clutter keeps playing its own sequences: every detail doodad ships one bone and one sequence, and that sequence is not always a sway - a number of them carry a small insect or butterfly that flits around the plant, and the ambient life of a field is made of them. The shader wind stands down for clutter rather than swinging the same plant a second time at its own rate; only the player's passage is added on top. Clutter that is further away than it draws stops being stepped at all, which is what pays for the extra tufts
- **Foliage smaller than a tree barely moved.** The wind normalised height against twenty yards and displaced by an absolute number of model units, so a one-yard tuft travelled a fraction of a millimetre. Short foliage normalises against its own height and takes a proportional amount; anything tree-sized keeps the numbers it had
- **A frame could run out of room for instances.** 16384 was enough before there was this much ground cover; past it whole models were dropped for a frame, which reads as clutter blinking rather than as anything being over budget

### Fixed - the character select screen
- **A rectangle of the character was missing, with the scene behind showing through it.** A fading instance sends every one of its batches through the pipeline that writes depth, so a fading character's own solid parts keep sorting against each other. The glue screens' backdrops are scene models, and their cloud and haze cards ask for no depth write and animate their alpha - so almost every frame those cards went down the writing pipeline and stood in front of the character as invisible occluders. The Forsaken scene alone has eight such materials

### Fixed - the video options
- **Anti-aliasing was listed twice, once without a dropdown.** The row carried the same name as the section heading above it. It is called Multisampling now, which is what the game's own video options call it
- **Windowed mode sat on top of the UI scale slider.** It is anchored to the right of vertical sync, which hung off the refresh-rate dropdown this client removes; closing that gap pulled both up a row. Closing a gap is right down a column and wrong across one, so windowed mode moves into the left column, into the space three removed checkboxes left

## [v3.0.0] - 2026-08-14

The original interface. This client now draws World of Warcraft's own FrameXML
rather than an interface of its own, and most of what follows is the work of
making that true: the windows the game ships, fed by a client that had to learn
to answer them.

Two thousand commits, so this is grouped by what changed rather than by when.
The interface sections come first, then the settings screen behind it, then the
world, the picture, the sound, and the checks that hold all of it in place.

### Corrected - work here that was built against the wrong model
Four things were written down here that the client already owned. A copy does not fail; it answers plausibly and slightly wrong, which is why none of the checks on this work caught any of them.
- **Equipment sets are the server's.** They were kept in a file beside this client, so the character's real sets were invisible and anything saved through the manager would not have existed for anyone else - this client's own paperdoll included
- **A key the client acts on is read from the client**, every time it is asked. It was copied once at startup, and the settings panel can rebind it
- **The reputation thresholds have one home**, shared by both interfaces. They agreed except at the top of the last band, which nothing else constrained: a faction at exalted drew an almost empty bar
- **The reputation list reuses the faction mapping that already existed** rather than reading the same file again to rediscover it

### Added - things that could be seen but not done
- **A party invitation could not be accepted.** Nor a guild invitation, a resurrect, or a duel: all four popups already appeared, and none of their eight buttons was connected to anything
- **The death popup announced the death and offered no way out of it** - releasing the spirit and resurrecting at the corpse both do something now
- **The mailbox** reads letters, takes coin and attachments, deletes, returns and sends. An attachment is asked for by its own id rather than by where it sits in the letter, which is what the request carries
- **The trade window** shows what each side has offered and lets items be placed and taken back. The other side's slots stay read-only
- **Equipment sets** - saved, used, deleted and shown, reading the list the server already sends and asking it to make the changes
- **The character sheet offers what fits a slot** when one is clicked, and **shows what a quest pays** in the quest log
- `/friend`, `/ignore`, `/use`, `/equip`, `/readycheck`, `/dismount` and `/script` all reach something now. Both list commands toggle, so naming someone already on the list takes them off it
- **Names complete themselves** as a whisper or mail address is typed, from the group, the guild and the friends list

### Fixed - errors the interface raised on its own
These are the ones that stopped something working rather than leaving it empty. A missing function is mostly harmless, because an undefined global answers nil and nil reads as false in a condition; what breaks the interface is a function that exists and is the wrong shape, because Lua raises rather than shrugging.
- **Abandoning a quest from the quest log.** `AbandonQuest()` is called with no arguments and the binding demanded one, which raises rather than returning nothing, so it had never once worked. Abandoning is also two steps - the log marks the quest, a confirmation shows its name - and only the second existed
- **The talent frame, the moment a point was staged.** `GetTalentInfo` returned eight values where ten are read; the frame puts the ninth into the rank it displays and compares that against the maximum. The same call also handed back no icon, which the interface reads as an empty slot, so every talent button drew blank - and reported every talent as learnable however deep in a chain it sat
- **Levelling up.** `PLAYER_LEVEL_UP` carried the level alone, of nine values the chat frame reads, and it tests the third to decide whether to mention mana. Every one of them was already parsed and simply not passed
- **Gaining talent points**, for the same reason: `CHARACTER_POINTS_CHANGED` is read as a count and was fired with nothing
- **Dying.** `GetReleaseTimeRemaining` is asked for the moment the player dies and compared against zero
- **Hovering the performance bar**, which adds up what every loaded addon uses
- **Following a player**, whose name was in scope at the call and not passed
- **Selecting anything a trainer offers**, which read a third cost the binding never returned
- **Opening the mail**, every time a letter arrived
- **Confirming a profession** at a trainer
- **The talent frame the moment a point was staged**, which is how talents are spent at all
- **Hovering the performance bar**, which adds up what every loaded addon uses

### Fixed - a window that worked but was never told anything
A panel needs three things: functions that answer, an event that opens it, and an event that says it is done. Only the first was being checked.
- **The quest frame opened on three of its four steps and closed on none.** The panel shown when a quest is handed back unfinished had its text ready and no event to draw it on, and nothing hid the frame afterwards, so it stayed open over whatever came next
- **A deleted letter stayed open showing nothing**, and applying an equipment set never told the manager it had finished, so the manager did not refresh or release the slots it was told to ignore

### Fixed - drawn wrongly rather than not at all
- **Every pet ability was greyed out**, because the frame asks whether a slot is usable and nothing answered
- **The right-click menu disabled duelling, trading and inviting**, because it asks whether the player is in control of their own character
- **A message frame kept a hundred and twenty-eight lines** whatever it asked for; twenty-two of them ask for three, and drew far past their own box

### Fixed - drawn twice
A window whose functions all return nil stays empty and unnoticed, and appears the moment they start answering. Finishing three APIs opened three windows that had been quietly present.
- **Every server refusal was shown twice** - "out of range", "not enough mana" - once by each interface, because this client fires the event the original's error frame listens for
- The extra action bars, the combo points, the pet's cast bar, the totem bar and the talent screen were each drawn by both
- **The quest log's list of quests was never suppressed at all**: the name listed for it belongs to no frame, which looks exactly like a frame that never opens

### Added
- **A reputation panel.** Standings arrive by an index that is not the faction id, and only Faction.dbc knows which is which; nothing had ever resolved it
- **Key bindings**, read from the file that declares all 275 of them, which was never loaded - the list had nothing to list
- The quest giver's dialog, its greeting, and the gossip window's list of an NPC's quests
- Talent prerequisites, totem timers, vendor prices in honor or badges, splitting a stack, and coin as a loot slot
- `RunScript`, without which `/script` and every macro body were silently inert

### Fixed - an answer of the wrong kind, which is worse than none at all
Every one of these sits behind a guard: `if ( classFileName ) then RAID_CLASS_COLORS[classFileName]`. The guard is the interface saying it already copes with the value being missing, so an answer that is merely *wrong* walks past it and raises a line later, inside a function that looks unrelated to the thing at fault.
- **The auction house never sent a search.** `QueryAuctionItems` raised on its own arguments before reaching the socket: the level boxes are handed over as the *text of an edit box*, and an empty one is `""`, which is not "absent" but "a string that will not convert". A raise inside a click handler is swallowed, so a search that never left looked exactly like one that found nothing
- **The guild roster and the who list died on their first online member.** The eleventh value is `classFileName`, a key, and this answered the numeric class id. A number is true, so the colour lookup found nothing and the next line read a field off it - taking `GuildStatus_Update` down and the whole roster with it. Only on the online branch, so a guild with everyone offline drew perfectly
- **The arena frames did not exist**, because `UnitClass` answered the string `"UNKNOWN"` for a unit it could not see. `ArenaEnemyFrame_OnLoad` guards for that and unpacks a class table anyway, so the addon raised at load - and an addon that raises at load is not degraded, it is absent
- **Every line of chat had a `0` in front of it.** Timestamps are held as the format to print them with and switched off with the word "none"; the fallback for a setting nobody has listed is `"0"`, which is a perfectly good format string that prints itself
- **Every row of an auction said "Your Bid"**, over the price it sits in front of, because the answer given was whether *anybody* held the high bid

### Fixed - the answer arrived after the question
This client's own windows read the model again on every frame they drew, so they collected a late answer without being told. An interface driven by events reads once. Each of these is a cache filled with nothing announcing it.
- **NPC dialog was blank.** Gossip carries its greeting as an id to be asked for separately, and the window was announced *before* the question was sent
- **Auction and vendor rows were "Item #41394" against a question mark**, and stayed that way: the rows carry an item id and nothing readable, and the names land after they are drawn
- **The unread-mail envelope never went out again.** Reading the last letter fired the event that redraws the *list*; the envelope answers a different one, which nothing fired
- **A hunter's ranged attack power never reached the character sheet.** Both attack powers were announced with `UNIT_ATTACK_POWER`, which this FrameXML registers and then handles nowhere

### Fixed - panels that were not there at all
A file that raises while loading is lost whole, so the fault reads as "that window does not exist" rather than "that window is buggy" - with nothing on screen and nothing in the log.
- **The spellbook**, from a guard path answering one nil where the success path answers six values
- **Inspecting another player**, from an include resolved against a path with the wrong capitalisation - the fallback for exactly that case existed and was pointed at a directory that does not exist
- **The debug tools**, for want of `getglobal`
- **The glyph panel drew in the corner of the screen with no close button.** Its only parenting happens inside `if ( name == "Blizzard_GlyphUI" )` on ADDON_LOADED, and the name was announced in the case the folders happen to have on disk rather than the case FrameXML compares against

### Fixed - laid out or drawn wrongly
- **Nothing with a scroll bar scrolled.** The wheel was hit-tested with the test written for the mouse, and those are two switches in WoW: the scroll frame template asks for the wheel and never for the mouse, so no scroll frame in the interface was ever found under the cursor
- **The talent tree was squeezed into the top third of its window**, cut off, and would not scroll - it had been made shorter than its own content. An anchor constrains both axes whether it means to or not, and the points bar's bottom edge was losing to the vertical centre that arrived with its LEFT and RIGHT anchors
- **Names, health bars and minimap markers drew through the bags and the auction house.** Both go into the same list, where the last thing added is on top, and the panels were going in first

### Fixed - a guard that asked the wrong interface
There are two interfaces in this client and only one of them is ImGui. Every place that wanted to know whether someone was typing asked ImGui's own flag, which is about ImGui's own fields and reads false for the whole time someone types into a box FrameXML draws.
- **Typing into chat did everything else as well.** `/logout` opened the quest log on the l and the social panel on the o, walked the character on the letters the movement keys sit under, sheathed the weapon on a z, and re-opened chat from inside itself on the slash - while the box it was all going into moved out from under the text, because each panel that opened re-laid out the frame beneath it
- The event path had this right all along and stops at the focused box, which is why it went unseen: everything above reads the key state directly, once a frame, and a poll is never in that loop to be stopped by it. The question has one home now and every path asks it there

### Fixed - the last thing standing on a stand-in
- **The battlefield minimap would not load** once the stand-in that answers for an unknown global was turned off. It addresses a frame the real client creates in C++ - the mini player arrow - with no guard, and unlike the world map's equivalent, which is made for exactly this reason, that one never was. So the only thing holding the addon up was a fake object
- With it made, the whole interface now loads with that stand-in **off**: no load errors, no addon failures, no login errors. That is the run worth trusting, because a stand-in keeps a file alive past a name nothing implements and hides that it was needed - `WOWEE_LUA_API_FALLBACK=0` is now written into the runner's own notes as the second way to run it

### Fixed - a bar drawn behind its own backing
- **A status bar's fill ignored the layer it asked for.** Twenty bars in the interface declare one - the cast bar asks for BORDER so that its own dark backing, which is BACKGROUND, stays behind the fill rather than over it - and all twenty declarations did nothing. The declaration is emitted as a call on the bar, a bar is a frame, and the method that receives it was a region method only, so every one of them fell to the no-op that answers for a name nothing implements
- The fill now sorts where the real client puts it: among the bar's own regions, in the layer it named. Found by turning on the call-time trace, which names the object that asked - the report on its own only ever said "SetDrawLayer", which is implemented and works, on regions

### Fixed - a workaround that outlived the thing it worked around
Something is missing, so a second place is written to cope, and the coping code explains itself in a comment - which makes it read as a considered decision for ever after. Then the gap is fixed and nothing goes back for the workaround, because nothing links the two.
- **Two scoreboard columns did nothing when clicked.** The sort refused damage and healing on the grounds that they were not parsed and read as zero for everyone. That had been true and had stopped being true; those were the only two headers in the battleground scoreboard that did nothing
- **Every player on that scoreboard wore a warrior's icon.** The packet carries no class, and the frame asks for exactly that case - `if ( classToken ) then ... else buttonClass:Hide()` - so a constant was not a safer default but a wrong one that looks like data
- **Every quest in the log drew a blank panel above its objectives.** The quest giver's own words are the third string in the query response, and the walk that fetches the fifth to get the completion line reads straight over it. The bytes were in hand and dropped
- **A quest that teaches a recipe showed no sign of it on the offer** - the details packet stopped three fields short of the reward spell, while the same row worked in the quest log, which reads a different packet
- **A failed quest read as unfinished.** The fail bit sits beside the complete bit in the same field, and only one of the two was ever read - so a timed quest that ran out looked exactly like one still running, and the tracker's branch for telling them apart could not be reached
- **A trainer offering the same spell at three ranks listed it three times, identically.** Neither the name nor the rank is on the wire; both are looked up by spell id, and only the name was being asked for
- Two comments corrected where they sat, because a wrong one that explains itself is worse than none: the completion line's, which said it was not parsed while the binding read it, and the vendor extended cost's, which said the same
- **The whole quest reward panel read as no reward.** Experience, honor, talents, arena points, the title and every reputation change were each a hard zero or nil, on one comment saying the client was not sent them. All are in the query response the log already parses - the reward XP is a QuestXP.dbc row indexed by the quest's level, the reputation an override in hundredths or a QuestFactionReward.dbc lookup - and the turn-in packet carries its own copy of the lot, three fields deeper than a first read of the layout suggests, which the offer parser's own test is what caught. Both the log and the offer now show what a quest actually pays
- **The character sheet's stat flyouts read "0.00%" for everyone.** Crit from Agility, spell crit from Intellect, every combat-rating bonus - dodge, parry, haste, defense, expertise, armor penetration - and health and mana regen from Spirit were each a constant, on a comment that the level-dependent conversion tables were not sent. They are the combat game tables (`gt*.dbc`) the client already ships, and the formulae are AzerothCore's own display path; each was checked exact against the tables in the headless harness before it landed. The armor-penetration line reused the same rating path, and defense rating now shows in the defense stat - a geared tank read the level cap alone before, with no sign of the rating that pushes it to crit immunity

### Fixed - a rating that read its neighbour
- **Every combat rating on the character sheet was off by one.** The interface's `CR_*` constants are one-based - `CR_DODGE` is 3 - but the player field array and the game tables are zero-based, and neither binding bridged the two, so the dodge tooltip showed the parry rating, defense showed dodge, and armor penetration fell off the end of the array and read zero. The conversion happens at the Lua boundary now, leaving the tables zero-based as the wire requires; the ImGui paperdoll, which called the accessor directly with zero-based indices, was right all along and is untouched

### Fixed - the interface using what it was given
- **The Send Mail tab raised before its frame was built**, so mail could be read and never written. An empty attachment slot answered no values at all, and the compose frame compares that count against one with nothing in front of it
- **The first character typed into any multi-line box raised** - the mail body, the macro editor, the guild information box. Nothing announced where the caret was, and the only thing that sets the field every one of those handlers reads is that announcement
- **The chat settings panel raised on its first colour swatch.** The interface passes a table where a colour's alpha belongs, which the real client accepts and this one refused
- **Autocomplete fought whoever kept typing.** It writes the completed name and highlights the part it added so the next character replaces it; with no way to select a run, typing "Thr" gave "Thrall" and the next keystroke made "Thralla"
- **A chat message with links ran out of room early.** The limit is in characters shown, not bytes held, and a shift-clicked item link is about sixty bytes for eighteen visible - so three of them filled a message that would otherwise have carried a dozen
- **A quest link in chat had no tooltip**, now that the log carries the giver's text
- Not done, and worth writing down because the first reason given for not doing it was wrong: the chat input border's middle section is declared to repeat and is stretched instead. That art is 32 pixels wide across a box of four hundred, and it does vary along its length, so the stretch smears something real. It stays undone because the interface has one shared sampler that clamps, and repeating needs a second one, a texture cache that knows which is which, and the surrounding plumbing - for a difference on a chat border that cannot be checked from here
- Four methods that answered nothing now answer: whether a frame is listening for an event, whether a tooltip is describing a given unit - which is the first line of the handler that colours a unit's name by reaction - stopping listening to everything at once, and clearing a box's typing history

### Fixed - the same field read two ways
- **Tilted doodads: reverted, still open.** Composing the doodad placement the way the building placement is composed followed from the two chunks sharing a rotation field - and on screen it was worse, so it is reverted. That is four permutations tried and four reverted; one step in that derivation is wrong and reading has not found which
- The trap that has caught all four: with no pitch and no roll the orders are *the same rotation*, and nearly every doodad stands upright - so any permutation looks right against a tree, and only a visibly tilted placement compared against a reference client can settle it. The test pins that, so the upright case cannot be offered as evidence again

### Fixed - a measurement taken between two moves
- **The on-screen objectives were missing, and nothing was wrong with the quests.** A frame's position was only ever written by the once-a-frame layout pass, so a frame anchored inside a handler measured as though it had never been placed - its own height, sitting at the origin. The interface anchors and measures in the same breath all over, because the real client answers a measurement whenever it is asked
- The quest tracker is the clearest case: it anchors each objective line under the one above and reads the edge it landed on to know how tall the block grew. Every read was zero, so the block had no height, so the tracker decided it had nothing to show, collapsed itself and disabled its own expand button. Nothing raised and nothing was logged - a tracker with no quests looks exactly like this
- Only what the answer depends on is resolved, not the whole tree: a loop that anchors and measures every row would otherwise turn one tracker update into hundreds of full passes
- The tracker is the case that was reported, and it was not the only one: **83 places across 31 files** move something and measure it in the same breath, among them the achievement rows (which gate their whole layout on a height read this way), the options panels, the calendar, mail, the bags and the friends list. `tools/framexml_measure_after_move.py` lists them
- **Every quest in the log wore its own number** - "26102 - Wolves Across the Border". The last value of `GetQuestLogTitle` is the developer switch that asks for the id to be *shown*, and it was being answered with the id, which is a large number and therefore true

### Fixed - a conversation that could not be moved to its own window
- **Splitting a whisper into its own chat window raised** and left the window empty. The copy walks the source frame's lines for one conversation and reads each back; the count ignored which conversation was asked for and answered every line, so the walk reached lines belonging to none, got no kind back for them, and indexed the chat-type table with nothing. Every ordinary line in the interface is one of those
- The values a line carries for the history - its id, its conversation, and the token saying what kind of chat it was - were dropped on the way in, so there was nothing to read back out. The token keeps its type now: it is a number and it is used as a table key, and one written down as text keys nothing there
- **A frame built in Lua was not told its own id.** A frame made that way is one of a numbered set and its handler finds the rest of the set through its id, so the new chat window looked itself up as `ChatFrame0` and found nothing. The frames built from XML always knew theirs

### Added - capabilities the client had and the interface could not reach
Each of these was already implemented and read by this client's own window, so handing the element over left it behind.
- **The flight map has a map behind it.** The node buttons are placed as fractions of a continent that was never drawn
- **`/who` prints its answer** when no panel is showing it, which is what `SetWhoToUI` is for
- **An auction sale's invoice** - the bid, the deposit, the house's cut and the other party - instead of the raw colon-separated body of the letter
- **Mana regen reads on the character sheet**, where every caster saw zero. The server sends both figures already computed - the rate while not casting and the reduced one during the five-second rule, with Spirit, Intellect, gear mp5 and any while-casting talent folded in - so the client reads those two fields rather than re-deriving a formula it lacks the auras to compute. The pair are private unit fields the client had never mapped; their WotLK offsets are pinned by a test so a name that stops matching the field enum fails loudly rather than silently reading zero

### Settings - one screen, and the controls on it doing something

The client used to keep its own settings window beside the game's. There is one
screen now, the game's, and the client's settings are rows on it.

Behind that, an audit of every control the panels declare. **69 of the 198
saved a choice and changed nothing**: the box moved, the value was remembered,
and the client went on as before. That number is 1.

- **Controls for things this client cannot do are gone**, and two whole pages
  with them - Voice Chat and Stereo 3D, where every control was for a feature
  that is not here. They were greyed with an explanation first, which is still
  a row to read and skip past
- **Every CVar was saved on exit and never applied again**, so a settings file
  full of choices did nothing on the next login
- **The picture settings mean what they say.** View distance covers what the
  engine can actually draw rather than stopping at the original client's limit,
  ground clutter draws the grass the distance it names, and particle density,
  weather detail, environment detail, texture filtering and shadow quality each
  thin or sharpen the thing they are named after
- **The gamma slider sat past its own maximum** and drove the screen to black
- **The interface scales to 2x**, for a screen across the room, with a
  confirmation on a timer so a scale that cannot be read can be undone
- **One control per setting.** View distance, camera distance and camera
  following each had two, the client's and the game's, disagreeing
- **The Combat Text panel's six filters** choose which numbers are drawn
- **Nameplates answer the Names panel**: totems, pets and guardians as separate
  categories, player titles, guild names, your own name, class colour, and
  plates kept off one another unless overlap is asked for
- **Chat has its spam filter and its mature language filter**, and the Guild
  Recruitment channel joins when it is ticked
- **Sound stops when the window loses focus**, and zone music can loop

### The world - things that could be done and were not

- **Dropping an item on a player opens a trade** with it
- **An item that needs a target waits for one** instead of being used on the
  player, and the cursor says so
- **Charge range is measured to a creature's edge**, not its centre, so a
  charge at something large no longer reports it out of range
- **The world's trigger creatures** were drawn, targetable and given health
  bars; they are scenery again
- **A dropped connection** says so in large letters and returns to the login
  screen, rather than leaving the player standing in a world they have left
- **The mount button dismounts** rather than dismounting and remounting
- **No slope can be climbed** now that the limit is applied to the ground
  itself, with a command to lift it for testing
- **Professions get their own spellbook tab** instead of filling General
- **Exit Game closes the program.** It asked the server to log out first and did
  nothing at all when that was refused, when there was no connection, or when a
  logout was already running

### The picture

- **Doodads drew more than twice as far as the ground they stand on**, which is
  what put trees in the sky over nothing
- **The night sky's stars** are drawn as points rather than a magnified texture
- **Distance fog** has a slider of its own and takes the colour of the sky it is
  seen against
- **The sky strobed** because "hellfire" matched the token for a flame, so the
  skybox was treated as a brazier and given a flicker
- **Every creature drew at its model's own size**, ignoring both scale fields
- **Authored collision geometry was discarded** by a guess about the model's name

### Sound

- **Picking an item up sounds like the item**, and the sounds the interface asks
  for are named rather than guessed at
- **The world-entry jump scare** was eight menu-open sounds arriving at once
- **109 wav files were loaded every session and never played** - jump and
  landing vocals with no way to be heard, a guild vault bank with no way to be
  asked for, and seventy player vocal samples

### Checks

The sweeps under `tools/` grew a rule: each must report how much it looked at,
because a matcher that has gone blind reads exactly like a clean tree. Four were
pinned at zero and could not tell the difference. Two new ones watch the
settings: one for a control whose CVar nothing reads, one that opens the
interface and confirms every control this client removes is actually gone.


## [v2.0.40] — 2026-08-08

Non-interface fixes — floor collision, chat, and liquid rendering — with no
dependency on the original-interface work. Most are backported from the
`framexml-ui-transition` branch; the slime rendering fix is new here.

### Fixed
- **Undercity's slime stopped moving in squares.** The magma/slime surface drove its flowing motion from value noise — one scalar per integer grid point — whose features sit square on the world grid, so up close the canal ooze churned in visible tiles. It now flows on gradient (Perlin) noise with each octave rotated so no two lattices align: a fractal swirl instead of a grid. Same scales and speeds, so the colour and glow are unchanged
- **Channel chat crashed on every line.** `CHAT_MSG_CHANNEL` was fired with only the message and sender, but a channel line is read positionally and `GetColoredName` builds `"CHANNEL"..arg8` — the nil channel index raised and tore the handler down, so nothing in the channel drew. The event now carries the full positional vector: the index looked up in the joined-channel list, numeric slots as numbers so a comparison does not raise, and the guid slot empty so class-colouring skips cleanly
- **The player model no longer flickers on and off every frame in Undercity.** The camera hid the player when the collision-squeezed distance dropped under the first-person threshold, and the renderer's visibility hardening forced it visible again in third person — the two wrote opposite values every frame, churning the model and its attached weapons. Hide on first-person *intent* (the zoom target), not the squeezed distance
- **An Undercity elevator no longer drags the player between two heights.** A WMO transport is registered as an ordinary instance so it renders and a rider stands on its deck, and the floor query iterated every instance — so as the elevator swept through the player's position its deck kept entering and leaving the floor candidates, at the elevator's own cycle. Transports are now skipped in the static-world floor query; the deck still reaches a rider through the dedicated instance query
- **The player is no longer kicked up to terrain height inside a building.** When the WMO floor query briefly found nothing, the pick fell back to the outdoor heightfield — the roof far overhead. Inside an interior WMO group the heightfield is meaningless and is now vetoed, so a momentary gap holds near the last floor instead of teleporting the player to the surface
- **An M2 doodad no longer drops the player through the floor.** An M2 collision surface well below a valid WMO floor is *beneath* that floor — a decoration or base under the walkway — but it won the pick and dropped the player ~6m. When a WMO floor is present, an M2 floor more than 1.5m below it is rejected
- **The player no longer walks out over terrain the artist cut away.** The Gadgetzan stairwell — and cave mouths, sunken entrances — is a hole marked in the terrain and skipped by the mesh builder, but `getHeightAt` interpolated straight across it and returned a surface at the player's feet that beat the real floor below. The hole is answered per quad now, dropping the terrain sample only when a WMO floor is underneath to take its place

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

## [v2.0.37-preview] - 2026-08-02

### Fixed
- **A broken script no longer takes the client down.** Lua errors were reported by firing `UI_ERROR_MESSAGE`, which is itself a Lua event that `UIErrorsFrame` listens for - so reporting an error ran script, and when that script errored it was reported the same way, recursing through both stacks inside a single frame until the process died. Because it died with drawing in flight, what survived in the log was a Vulkan device loss, which sent the search into the renderer for a fault that was never there. Errors now go to a path that shows them without telling any script, and event dispatch refuses to nest more than eight deep
- **Flights to anywhere but the next stop work.** Two separate faults: `CMSG_ACTIVATETAXI` carries only a source and a destination, so the server had no single path to answer with and the request timed out; and the route was worked out by a second search of the flight graph that included nodes the player had never visited, which the server refuses outright on the first one it does not recognise. Multi-stop routes are sent whole, and are built from the same discovered-node search the quoted price already came from
- **Turning shadows off no longer loses the device**
- **Casting a spell on yourself no longer turns you around**
- **A long flight path is no longer rejected for its length**, and a rejected one says why
- **Arriving in the world during a flight no longer returns the player to where they logged in**
- **The interface fonts are found whatever the case the install spells them in.** Four fixed spellings were tried, so a directory written `Misc/fonts` matched none of them and the built-in face was kept - reported at a level the log does not carry, which left wrong-looking text and no reason anywhere. Every way this can fail is now a distinct warning
- **Texture uploads no longer share one fence across two queues**, and finished batches are retired every frame rather than only while terrain is streaming

### Changed
- **The client's interface is drawn in the game's own typeface.** FRIZQT is loaded at fifteen points and added first, so it is the face ImGui uses for everything that does not ask for another - close enough to the metrics the panels were built against for their layouts to survive. The five faces stay registered at eighteen points for the widget renderer, which asks for them by name. This reverses the previous release's fix, which kept the built-in face as the default

### Fixed
- **Loading the original interface no longer covers this one.** `WOWEE_LOAD_FRAMEXML=1` builds a hundred of Blizzard's frames, most of them still half-supported, and every one of them was drawn on top of the client's own. They now appear only for the elements named in `WOWEE_FRAMEXML_UI`, so loading it to exercise the parser leaves the interface you were using on screen

## [v2.0.35-preview] - 2026-08-01

### Fixed
- **The client's own interface keeps its own font.** ImGui draws with whichever face is added first, so loading the game's typefaces made FRIZQT the default for every panel this client draws, at eighteen points - larger text and different metrics than the layouts were built against, on startup, whether or not the original interface was being loaded

## [v2.0.34-preview] - 2026-08-01

### Original interface (FrameXML)
- **The interface draws its own art.** Batching the widget renderer's texture uploads left a batch holding only ImGui textures looking empty to both ends of endUploadBatch, so the command buffer carrying every copy was freed unsubmitted and every image stayed blank while rectangles and text drew normally. Staging allocated with plain Vulkan calls is now handed to the batch and freed once its copies have run
- **The interface is laid out in its own units.** FrameXML is authored against a virtual screen 768 units tall, so a frame is the same apparent size on every display; treating those numbers as pixels drew it at half size on a 1528-tall window. The tree lays out in units and the renderer converts once, with the cursor making the same trip in reverse
- **UIParent fills the screen.** It was created with no anchors, and an unanchored frame falls to the centre-on-parent default with no size - so everything hanging off it, including FrameXML's own UIParent, inherited a zero-size box in the middle of the screen
- **Type is drawn in the game's own faces** - FRIZQT, MORPHEUS, SKURRI, ARIALN and FRIENDS - at the size, colour and outline FrameXML's 42 font objects specify, rather than one built-in face at a guessed size
- **Sliders drag, cooldowns sweep and edit boxes take text.** GetTime, which all three need and nothing had implemented, answers from one clock shared with the renderer
- **All three mouse buttons reach the interface**, gated on what each frame registered for, which is how a context menu opens on a unit frame and not on a plain button
- **The whole original interface loads.** All 139 files of Blizzard's own FrameXML - 13 Lua and 126 XML - build against this client's widget tree in around 380ms, behind `WOWEE_LOAD_FRAMEXML=1`. It was 67 files failing and a client frozen hard enough to need killing
- **CreateFrame applies the template it is given.** The fourth argument was ignored outright, and that is not a missing feature so much as a trap: OptionsList_OnLoad makes one button, divides the list's height by that button's height to decide how many fit, and loops to the result. No template means no size, a height of zero, and a count of (h-8)/0 - which Lua computes happily as infinity. The loop then created frames under fresh names until memory ran out
- **A template that inherits another applies it.** The emitter returned before `inherits` was ever read for a virtual frame, so 217 of FrameXML's 296 templates arrived without the base they are built on
- **`parentKey` binds a region to a field on its owner.** 242 declarations across 31 files, every one of them nil, and FrameXML's handlers reach for them constantly - QuestHonorFrameTemplate's OnLoad opens with `self.icon:SetTexture(...)`
- **A frame's `id` exists.** 848 declared across 57 files with no `GetID`/`SetID` behind them, and FrameXML concatenates the result straight into a name: `_G["PartyMemberFrame" .. self:GetID() .. "PetFrame"]`. It is set before any template runs, because a template's children load while the template is being applied
- **Button art declared outside a Layer is created.** `<NormalTexture>`, `<HighlightTexture>`, `<ButtonText>` and their siblings were ignored; HighlightTexture alone appears in 62 files. The setters take a file path as readily as a texture, which is how LoadMicroButtonTextures uses them
- **A scroll frame's `<ScrollChild>` is built**, and handler arguments have their real names - `OnVerticalScroll`'s body opens with `scrollbar:SetValue(offset)`, which was nil on every scroll frame in the interface
- **`$parent` skips unnamed frames to the nearest named ancestor**, and a frame loads once when it is finished rather than once per template it is built from
- **The missing-API stand-in answers methods, not data.** FrameXML guards its optional frames properly - `if (prefixText) then prefixText:GetText()` - and a stand-in that answers everything makes the correct check worse than no check. Widget methods are now enumerated rather than guessed at, because measurement showed methods and data cannot be told apart by shape: of the 307 method names FrameXML calls, eighteen read as nouns
- **A runaway script costs one file rather than the session.** The load runs on the main thread during world entry, so a script that will not return freezes the client until the server drops the connection. A wall-clock deadline aborts it and names the Lua line it was on
- **Errors carry the Lua call stack.** An error says where it happened; the interesting part is nearly always how it got there


## [v2.0.33-preview] - 2026-08-01

### Transports
- **Ships sail bow-first.** Facing was pinned to whichever orientation the server last reported - a berth heading - and held there for the whole voyage while the position ran along the route underneath. That field is set by every server update, including ones for a ship the client animates itself, and it was taken as authoritative regardless. It is authoritative only while the server is also driving position; when the client owns the animation it owns the phase, and facing comes from the route
- **Every hull's bow is at model -X, so they all take the same correction.** The table said the opposite and then listed the icebreaker and the night-elf ferry as the two exceptions - exactly inverted. Measured two ways across every .wmo under World\wmo\transports: the hulls taper to a point at -X and stay blunt at +X, and the icebreaker is a paddle steamer whose paddlewheel, which belongs at the stern, sits at x=+36.3 on a hull spanning -60.7..+50.1. The table could never have been right, because it was fitted while facing came from a frozen server yaw rather than from the route
- **A docked hull lies on the chord through its berth.** A route turns as it passes its dock - the Maiden's Fancy comes into Menethil 26 degrees off the bearing it leaves on - so taking the arrival leg alone parked it half that turn out of true, enough to walk the gangway off the plank
- **Transports run on the server's route clock.** A WotLK MO_TRANSPORT publishes its period and how far through it the hull is, as a fraction of 65535. Neither field was read, so the client animated on a period it worked out from distance over speed; when that came out short, the ferry lapped its shore until the server's schedule caught up. This syncs the cycle, not yet the position within it
- **A cross-continent boat waits at the pier between crossings** rather than ferrying its shore over and over. Each map's slice is animated on its own and was sized from its own nodes alone; it now measures the whole route and spends the difference held at the dock
- **The 180-degree correction was measuring nothing.** It compared the canonical velocity against (cos s, sin s), where a server yaw points along (sin s, cos s) - the two components swapped, a reflection rather than a rotation. A transport facing exactly along its travel scored sin(2s), which is -1 near a heading of 135 degrees, so correctly-oriented ships were flipped purely on which way their route ran
- **Sails and paddlewheels are on the ships.** They were being drawn at the world origin: WMORenderer::setM2Renderer is declared and was never called, so every path that moves or destroys a WMO's child doodads sat behind a null pointer. Static world doodads were unaffected, because terrain streaming places those itself
- **Each ship gets its own doodads.** M2 instances are deduplicated on model and position, which is right for the static world and wrong for children created at the origin and moved into place by a parent - every ship of a class was handed the first one's sails, and whichever hull unloaded first destroyed them for the other. The same collapsed thirteen barrels in a hold into one
- **A docked ship's machinery stops.** The animation was set once when the doodad spawned and never revisited, so the Kraken's paddlewheel turned while the ship sat at the pier
- **A rider can get off.** Boarding somewhere the deck query never succeeds - a gangway belonging to the pier rather than the hull - left a hold engaged that discards walking motion and reapplies the boarding offset every frame. The character ran on the spot, could not walk far enough to trigger disembark, and so could not leave. Stepping ashore onto the dock also kept them attached: the disembark footprint is larger than any hull by design, and losing the deck underfoot is what actually tells ashore from aboard
- **Zeppelins stopped flying one another's routes.** The fallback returned the first usable id in a candidate list for the whole display family, and below it a last-resort branch handed out the first moving path in an unordered map

### Mounts & Pets
- **Dismounting no longer strikes a pose.** The mount display field keeps its old value for a few frames after the request, and that was taken at face value and re-mounted the player seven milliseconds after they got off. The restored value then made the server's own dismount read as transient and get discarded. The mount blinked off, back on, and off again over about two hundred milliseconds, with the character caught holding the seated rider animation
- **A rider on a moving boat sits on their mount.** The seat position is smoothed to damp bone jitter while sitting still, and the branch that snaps instead keys off movement input - but a player standing on a boat presses nothing while the world carries them. The filter trailed them by its own time constant, two yards at ferry speed, swinging to one side as the hull turned. It now asks whether the seat is moving in the world rather than whether the player asked it to
- **Pets can be dismissed.** The action field of CMSG_PET_ACTION is a pair, not an id: the high byte says what kind of action it is. Dismiss packed action 0 under the command type, which is COMMAND_STAY - the pet planted itself. Four callers each read the field differently; one had the type and action swapped, another sent a bare 1..6 with no type byte, and the bar labelled slots off a numbering that does not exist on the wire
- **Companions can be dismissed by pressing them again.** They have no aura, so nothing appears in the buff bar to right-click, and pressing the spell only ever summoned. CMSG_DISMISS_CRITTER was in the opcode tables with nothing sending it

### World & Movement
- **Ironforge stops emptying out at doorways and hallways.** Portal culling seeded its walk from one position while testing every door against another's frustum - the camera and the character are in different rooms in a doorway, and neither is reliably the right place to start. It now seeds from both, which can only add groups. Ironforge showed it worst because exterior groups are seeded unconditionally, and it has almost none
- **Stairs leading underground can be walked down.** The terrain-penetration rescue pushes a player back to the heightfield when their feet end up beneath it, and a stairwell cut into a keep passes under that surface within a step or two, so each step down was undone. A floor already resolved by grounding that sits below the heightfield and at the feet now settles it
- **Landing a taxi flight at Booty Bay no longer throws the character under the structure.** The clamp probed for a floor at forty above the player, and the player is what the clamp rewrites every frame - so the deck was found only from far below it, and snapping onto it lifted the probe out of range. It flip-flopped between the deck at 36.5 and the terrain at 4.5 twelve times, and abandoned the player wherever the last frame landed
- **The world map opens on the zone you are in.** It worked the zone out from geometry: of the WorldMapArea boxes containing you, the one you sit deepest inside. Those boxes are axis-aligned rectangles around irregular zones and overlap heavily, so it opened on zones you were merely near. The server sends the zone id and the client already tracked it for other things

### Items & Mail
- **Mail shows who it is from.** The sender was rendered from a field that was never populated for player mail
- **Sorting bags merges partial stacks** before ordering them, so two half stacks become one
- **Disenchant can pick the item it works on.** The spell's target flags mark it as needing an item, which nothing acted on

## [v2.0.32-preview] - 2026-07-31

### Water
- **The water's edge has a shoreline.** A wet-sand band that darkens what is under it, sediment that moves with the surf, a swash line that runs up the beach and back, foam that rides the water instead of sitting still in world space, and spray thrown off the advancing front. The foam is broken up by cellular octaves at rotated, non-multiple scales with a jittered threshold, because thresholding Worley cells near their centres puts a dot in every cell and makes the lattice itself the pattern - which is the grid that was visible before
- **The ocean fades into the horizon haze** rather than ending on a hard line, and the wave fronts are phase-warped by noise so the generator's pattern stops reading as bright parallel lines at distance
- **Water churns where you move through it.** Wading lays down froth underfoot; swimming leaves a V wake off the shoulders whose arms open with distance behind. Points age out and spread as they go, and carry a bounding circle so every water pixel outside the trail rejects them in one test
- **Spray is drawn on top of the water instead of underneath it.** Water moved into a pass of its own so the refraction copy could be taken before it, which left the swim effects recording into the scene pass that now runs first - shallow water hid the droplets partly, the deeper water you swim in hid them completely. The wading spray also never spawned at all: it shared an accumulator that the swimming branch zeroes on every frame it is not swimming, so a 30/s rate could only ever reach 0.5 in a frame
- **Crossing the surface sweeps a waterline across the view** instead of the whole scene flipping at once. The line is anchored to the projected horizon rather than the middle of the screen, and the tint no longer gives out past 15 units down - the depth query's default vertical reach was rejecting the surface once you were deeper than that, so the scene snapped bright at a fixed depth
- **Refraction no longer feeds itself.** The scene copy the water samples was being taken from the finished frame, so a moving object left one sharp copy per frame - a train of ghosts - and the brightness compounded through the loop. The copy is now taken before the water draws and at half resolution

### Movement & Swimming
- **You no longer sink through hills.** Floor selection rejects any surface more than 0.60 yards above the feet as unreachable, which at the steepest walkable slope covers a mounted player for about 1/40th of a second - a 20 fps frame rises 0.83 yards and the terrain being climbed stops counting as ground. From there it compounds, because falling puts the feet further below the surface. Outdoors the heightfield has one surface per column, so feet below it are pushed back out, guarded against everything legitimately built underneath: WMO and M2 floors probed from the player, and hole-cut chunks, which is how a cave mouth is opened
- **Swimming holds its depth** instead of being pulled to the surface, and holding space keeps ascending rather than rising for a moment and stopping
- Walking out of water no longer stutters: both swim checks decided from a single depth, so a character at the boundary flipped state every frame, restarting the locomotion animation and sending a START/STOP pair each time

### Character & Equipment
- **Helmets go on the head.** All three paths - your character, other players, NPCs - attached head gear at M2 attachment 0, which is the shield mount, falling back to 11 (the helm) only if that failed. It never failed. Detaching 0 on an equipment refresh was also dropping shields
- **Your own character wears a helm at all.** The local appearance path had no head slot: six attachment calls, all weapons. Head-model resolution - race and gender suffix, base fallback, suffixed texture - now lives in one place that all three paths call
- **A circlet leaves your hair showing.** Hair was hidden for any head item, so a tiara left the character bald with nothing visible. ItemDisplayInfo points at a HelmetGeosetVisData row per gender, and the row crowns and circlets use is all zeroes where a plate helm's is not. The columns holding those references move between the 23-field and 25-field builds, so they are found by asking which columns reference the visibility table
- **Show Helm works.** It flipped a bool, sent the packet and printed a message; nothing read the flag
- **Facial features exist.** CharacterFacialHairStyles' geoset columns were read at 3, 4 and 5, which hold a constant per race in every copy of that DBC here - Draenei rows read 2010429269 on every variation. Truncated and offset they name geosets no model has, so no character had a beard, tendrils or earrings. The variants are at columns 6 to 8. The clamp that forced each channel to at least 1 goes with them, since zero means the channel has no feature
- **A face overlay authored at a different resolution than the body is fitted to its region.** The only resizing was a whole-factor upscale, so an overlay larger than its region was pasted at its own size across the regions next to it. Mismatched art sets are now reported
- A character whose appearance the data cannot draw takes the nearest face rather than none, and says so - character creation offers an unverified 0..9 range whenever its DBC scan comes up empty, and those numbers are backed by no CharSections row

### Targeting & Interaction
- **A corpse no longer outranks the living player standing on it.** A dead creature is still a UNIT and still answers isHostile(), and hostiles are selected ahead of everything; failing that, a body at ground level is nearer the camera than a player's hit sphere a metre up. The living now rank first, and a corpse stays selectable only when nothing alive is under the cursor
- **Fishing schools cannot be right-clicked empty.** They are fished, not opened. Clicking one sent CMSG_GAMEOBJ_USE and, while the object's metadata was outstanding, a CMSG_LOOT - which the server answers with the hole's loot
- Left-click targeting, the right-click world picker and the hover cursor shared one ray picker instead of three copies. The hover copy had already drifted: it had no critter case, so the hand cursor appeared over a sphere three times the size a click would test
- Warrior Charge rejects game objects and corpses, and quests marked complete can be abandoned

### Combat & Spells
- **Casting at a target actually faces it.** The renderer holds the character's yaw and the game side holds canonical yaw, and the frame loop converts render to game every frame - so a facing set only in the packet is undone before anything with a cast time completes, and the server re-checks the arc against the restored heading. Smite reported the target as not in front while the character plainly faced it
- **The conversion between those two was a mirror where it should be a rotation.** Render yaw is canonical plus 90 degrees, which falls out of the swap in canonicalToRender and the atan2(-dy, dx) canonical convention; it was written as 180 minus, which agrees at exactly one heading. Every user of the pair was wrong together, so nothing looked amiss until a value crossed to the server
- **Heals and buffs fall back to you when nothing friendly is targeted.** A heal and a nuke share an effect id and can share a school; EffectImplicitTargetA is what tells them apart. Spells that take either target, like Dispel Magic, are left alone
- Pressing the mount you are riding dismounts you instead of dismounting and immediately remounting

### Items, Mail & Bank
- **A priest robe read as a cloak.** The item query layout is guessed from the bytes, and it decided on InventoryType alone. On a server without BuyCount that read lands on AllowableClass, and Priest-only is 16 - INVTYPE_CLOAK. Both readings are now scored across several fields, with BuyCount itself breaking the tie: it is how many the vendor sells at once, and a layout read one field short puts a price there
- Mail attachment slots match what the realm's packet can carry - Vanilla writes a single item GUID, and the compose window offered twelve regardless, sending the first and leaving the rest in your bags without a word. Attached stacks show their size
- Each bank bag can be sorted on its own; sorting the whole bank pools everything into the main slots, which empties a bag being kept as a category

### Rendering
- **Rigid props stopped swaying like trees.** Foliage tokens are matched as substrings because model names run words together, so "thorn" inside Stranglethorn made every troll ruin sway - along with "corn" in Corner, "hops" in ShopSign, "tree" in StreetSign, "crop" in Outcrop and "herb" in Herbalism. Names are head-final compounds, so the match ending furthest right decides: StranglethornRuins is a ruin while DustwallowTree is still a tree. 73 models stop swaying and no plant loses its wind
- **Forges are solid, and so is Ironforge.** isForge matched the city, so all 64 of its doodads had every batch forced to additive - benches, statues, cliffs, elevators. A forge is now a forge only when the name ends on it, and the additive override applies to the flame cards rather than the whole model, which is mostly masonry
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
- **Incremental WMO loading dropped a group at every budget break.** It marked the current group done before deciding whether to stop, so Stormwind - 286 groups against a 6 ms budget - lost around 22 of them, interior floors past a doorway among them. Every non-empty group is now checked for before a model is published
- Water footsteps point at sounds that exist: the WATER surface was built from a naming convention that is real for Stone, Dirt, Grass, Wood and Snow but has no Water variant in any archive, and the movement sounds pointed at a folder that does not exist

## [v2.0.31-preview] - 2026-07-24

### UI
- **Auction listings show the item's rolled random property.** An auction carries the "of the …" suffix separately from the item template, so browse-tab tooltips rendered the base template only - a Bear's suffix looked identical to no suffix at all, and there was no way to tell what you were bidding on. Tooltips now fold the auction's random property and suffix factor into the same instance-aware view the bags use, so the Strength, Stamina and secondary-stat bonuses read the same in the auction house as they will in your inventory

### Platform
- **Holding a key on macOS opened the accent chooser instead of repeating it.** SDL2 leaves text input enabled for the whole session, so AppKit routed every keystroke through `NSTextInputContext` - and A, S, E and the other letters that take diacritics popped the press-and-hold menu over the game rather than moving the character. Mac builds now register `ApplePressAndHoldEnabled=NO` before `SDL_Init` brings up NSApplication. It lands in this process' registration domain, so nothing is written to your saved preferences

## [v2.0.30-preview] - 2026-07-24

### Rendering
- **Brightness is a true multiply again, and no longer blows out over water.** The multiplicative overlay (scene × brightness, instead of a lerp toward white) had to be reverted once because water refraction samples a scene-history image captured from the final swapchain, which already has display brightness baked in - re-applying it each frame fed back through that temporal capture and diverged, where the old white-lerp had merely converged. The brightness factor is now passed to the water shader and divided back out of the refraction sample, so refraction sees the un-brightened scene and the display gets a real multiply. This also fixes the latent inverse, water slowly creeping to black
- **FSR3 frame generation creates its upscale context.** "Path C upscale failed rc 3" was the AMD FFX Vulkan backend failing to build its compute pipelines: the device enabled `shaderFloat16` for fp16 math but not the 16-bit *storage* features the SDK's shaders need to pack fp16 into buffers. `storageBuffer16BitAccess`, `uniformAndStorageBuffer16BitAccess` and `shaderInt8` are now enabled where the device supports them
- `WOWEE_VULKAN_VALIDATION=1` turns on the Khronos validation layer in a release build and routes its output to the log - the tool that identified the FSR3 failure above as SDK-side invalid shaders (4KB push constants against a 256-byte limit, NV-only SPIR-V extensions, descriptor mismatches)
- **Steam tonks stopped glowing.** The "steam" substring in the VFX classifier also matches SteamTonk vehicle models. Gating on low-poly geometry wasn't enough - the TBC/Turtle tonk overlay models are small enough to slip under the vertex threshold, so a tonk with a smoke emitter was classified as an additive spell effect and rendered translucent. Any "tonk"/"tank" token is now excluded outright; real steam effects never carry one. Covered by a classifier regression test
- **Cloaks are textured in the world, not just in the paperdoll.** The in-world player model read the cloak texture from ItemDisplayInfo's LeftModelTexture only, but some cloaks - Jaina's Radiance among them - store it in the right field, leaving them blank in the world while the character preview (which already checked both) looked right

### Bank & Guild Bank
- **Depositing at the bank uses your purchased bank bags.** The old path scanned only the main bank slots and announced "Bank is full" once they filled, ignoring bag space entirely; deposits now go through `CMSG_AUTOBANK_ITEM` so the server places the item in any free bank slot
- **Right-clicking a bank item withdraws it.** The bank slot renderer had drag and shift-link but no right-click handler at all, so right-clicks were silently dropped. Withdrawal now uses `CMSG_AUTOSTORE_BANK_ITEM`, letting the server place the item in any free bag rather than only the backpack
- **Guild vaults open when you interact with them.** Nothing called `openGuildBank()` for a type-34 GameObject, and `openGuildBank()` never sent `CMSG_GUILD_BANKER_ACTIVATE` - it only queried a tab, so no bank list ever arrived
- **Guild bank item transfers were malformed on the wire and silently dropped.** `CMSG_GUILD_BANK_SWAP_ITEMS` was missing the toChar direction byte, wrote splitedAmount as a mid-packet u8 rather than a trailing u32, and set bankToBank=1 on deposits, which sent the server down its bank-to-bank path. Both builders now match the 3.3.5a layout, with withdrawals using the autoStore sub-format so the server auto-places into a free inventory slot
- Right-clicking a bag item while the guild bank is open deposits it into the first free slot of the viewed tab, and bags open automatically with the vault so items are reachable. Clicking a tab previously sent a query without updating the active tab, so withdraw and deposit always targeted tab 0 - the active tab now syncs from each `SMSG_GUILD_BANK_LIST`
- The guild bank renders a full 98-slot (14×7) grid and looks items up by slot ID. It previously drew only the slots the server sent, which is a sparse list - often just the occupied ones, and nothing at all for an empty tab
- The bank's "Combine bags" toggle persists across relaunches (`bank_combine_bags` in settings.cfg); it was a function-local static that reset to the split view every session

### Crafting
- **Crafting while mounted no longer freezes the window.** `startCraftQueue` filled the queue and then called `castSpell`, which bails early when mounted - leaving the queue populated with nothing in flight and the UI stuck on "Crafting… N remaining" until you manually mounted and dismounted. It now dismounts synchronously before queueing, matching retail
- The recipe list is a draggable splitter and grows with the window. It was a fixed 260px while the detail pane absorbed all extra width, so enlarging the window never revealed a truncated recipe name; anything still too wide for the pane gets a hover tooltip with the full name

### Quests
- **Collect-item objectives advance as you loot.** 3.3.5a servers don't push collect counts the way they push kill credit, so a tracker relying on `SMSG_QUESTUPDATE_ADD_ITEM` alone never moved. Item objectives are now reconciled against actual bag contents on every inventory rebuild
- Newly accepted quests are tracked automatically, from both questgivers and shared-quest accepts. Login and resync loads are untouched, so the "show all when none tracked" fallback still covers quests you already had

### Character
- **Casting while mounted dismounts and then casts.** Previously any spell pressed while mounted just dismounted and dropped the cast. Airborne on a flying mount, the cast is refused with "You can't do that while flying" rather than dropping you out of the sky

### GM Tools
- **A searchable GM command browser**, on a new "GM" micro-menu button, over the existing 195-entry command reference. The left pane groups commands by first token (flattening to a filtered list while searching) with a max-permission filter; the right pane shows syntax, description and a security badge. Commands dispatch as SAY chat with the AzerothCore "." prefix, so the server still enforces the real permission level
- Command syntax is parsed into labeled form fields rather than a raw editable string - `#x` becomes a numeric input, `$x` a text input, `a/b` a dropdown, `[word]` an optional checkbox - with a live "Will send" preview, player/name fields defaulting to your current target, and an "Edit manually" escape hatch. Adds 12 more commonly-used commands (the reset family, repairitems, additemset, modify arenapoints/drunk/faction/xp/phase)
- **"Max Out Character"** detects your class and active expansion and queues a full setup: max level for the expansion (60/70/80), all class spells and talents, maxed skills, optionally 1000g, and a class-appropriate gear kit anchored on class legendaries. Commands drain one per frame to stay under the server's chat-flood protection, and per-slot toggles let you apply only the parts you want
- **`.gm fly on` actually lets you take off.** It sets the CAN_FLY movement flag, but flight physics were gated on `isPlayerFlying()`, which also wants FLYING - a flag the server only sets once you're already airborne. Flight is now driven from CAN_FLY, and the descend key (X) works on foot instead of only on a flying mount

## [v2.0.29-preview] - 2026-07-23

### World
- **Hairline seams between terrain tiles are gone.** Each tile's edge vertices were built by subtracting the chunk and per-vertex steps from the tile corner, so a tile's far edge (`…×TILE − TILE`) and its neighbour's near edge (`(…−1)×TILE`) - mathematically the same point - rounded to slightly different float32 values. The sub-yard gap opened T-junction cracks that showed as thin lines, worst far from the map origin (across Kalimdor). Vertex XY is now a single multiply from the tile index, `TILE_SIZE × (32 − tile − step/128)`, so the shared edge is bit-identical on both sides and the tiles meet exactly - no mesh-overlap or scale hacks
- **Game objects went missing after leaving an area and coming back, for the rest of the session.** Walking away dropped every instance of a mailbox or chest model, and the 60-second unused-model reaper then evicted the model itself; the reload on return did not reliably produce a drawn object. Game object models are now pinned in the renderer, so the reap/reload cycle never happens for them. They are a small bounded set - one per display ID actually encountered - and ambient doodads are still reaped normally
- Game objects are exempt from the adaptive doodad render distance. That distance collapses to its densest-scene value in any populated area, which in a city means roughly 200 units, so mailboxes and chests vanished well inside the range the server still considered them visible. They now hold a 600-unit floor; frustum and occlusion culling are unaffected
- GPU cull results are matched back to instances by ID rather than array index. The visibility buffer is read a full frame-slot cycle after it is written, and instances are appended and swap-removed in between, so a respawned object landing at the volatile tail of the array could inherit the verdict of whatever transient object held that slot two frames earlier
- Game objects no longer take the HiZ occlusion test at all. A mailbox or chest sits flush against a wall or doorframe, exactly where the coarse depth pyramid reports a false occlusion; once culled it stops being drawn, so it never regains the last-frame depth that would clear the false verdict, and it stayed invisible in place until the camera moved. These small gameplay props now opt out of occlusion culling entirely (frustum and distance culling still bound them), so they cannot vanish while in view

### Lighting
- **Hearth fires, campfires and forges cast light.** Fires express their flame as particle emitters rather than a glow card, and the path that turns emitters into a light was gated on lantern-like models - so a fireplace full of burning wood lit nothing at all. Open flame now takes that path, with a wider, warmer light than a candle wick, and forges are classified as the contained fires they are
- **Forges rendered as black windows.** `BLACKSMITHFORGE.m2` is nothing but the fire in the hearth, an effect card on a black background, but the additive override that handles exactly that shape was gated on spell effects, so it drew opaque and filled the opening with a black rectangle. Its black backing is colour-keyed too: the texture is `ARMORREFLECT`, whose name carries none of the flame or glow tokens the colour-key hint looks for
- Lamps, torches and braziers gutter. Each fixture's phase is hashed from its own placement, so no two pulse together - a synchronised row of street lamps reads as a rendering artifact rather than firelight - and the guttering drives the light each one casts, not just its glow sprite, since the pool of light on the ground is what the eye actually reads as fire
- Darkshire's town hall clock face is lit by a fire behind it. WMO emissive was a flag tuned for Stormwind's lamp glass, far too bright for a clock face, and is now a level: the new one keeps normal daylight shading, adds a warm glow that fades up as the scene darkens, and wavers on three detuned sines so the flame never visibly loops. A tight sun highlight and a Fresnel sheen sell the pane of glass over the dial
- Hearth fire light no longer turns the surrounding brickwork orange. Local lights are unshadowed, so their radius is how far the glow reaches straight through whatever surrounds the fire; at 11 units a hearth lit its entire chimney from the inside out

### UI
- **Target-gated abilities (Execute, Hammer of Wrath, ...) grey out until the target qualifies, and fail with a useful reason.** These read Spell.dbc's TargetAuraState - e.g. "target below 20% health" - which the client never consulted, so the button looked castable and the failure just said "Target aurastate". The action button now dims (with a tooltip naming the requirement) while the target isn't in the needed state, and the cast-failed message reads "Target must be below 20% health." and the like
- **The reputation panel gained real tracking controls.** Right-clicking a faction now offers, alongside "Track on Rep Bar", an **At War** toggle (declares war / makes peace via CMSG_SET_FACTION_ATWAR, disabled on peace-forced factions) and an **Inactive** toggle (parks it via CMSG_SET_FACTION_INACTIVE). Inactive factions are hidden behind a "Show inactive" checkbox - which reports how many are parked - and render dimmed when shown
- **"Your auction of … has sold!" named the wrong item ("Item #0").** The owner-notification packet was parsed with a phantom `action` field, which pushed the item-entry read to an offset holding a zero padding word; the real `item_template` sits at offset 20. It now parses correctly (and always reads as "sold", since expiry and outbids arrive on their own opcodes)
- **Readable letter/note items resolve their `$`-tokens.** The item-text window drew the body raw, so a quest letter showed literal markup like "$g himself : herself;". It now runs the same placeholder replacer as quest and chat text, filling in gender ($g), player name ($n), line breaks ($b), and the rest
- **The achievements window shows each achievement's real icon** instead of a gold star. Achievement.dbc's IconID (added to the DBC layout) resolves through SpellIcon.dbc to the artwork, rendered as a bordered 32px icon with the name and point value beside it; a star placeholder still fills the slot while an icon streams in or if it's missing
- **Raid target markers never appeared on marked enemies.** Three separate faults stacked: `GameHandler` kept its own copy of the marks that nothing ever wrote, so the target frame, nameplates, minimap and party list all read zeros; the wire format was inverted, with the full list (which carries only the icons that are set, not a fixed eight) and the single-mark form (which leads with the setter's GUID on WotLK but not on classic or TBC) swapped; and the marks were drawn as text symbols the font has no code points for, so they rendered as '?' boxes. Marks now use Blizzard's icon artwork, floating above the unit as in the original client, and are covered by tests across both wire layouts
- Solo players can set target markers. The server only broadcasts them to a group, so marking while ungrouped did nothing and the feature could not be used, or tested, without a second player. Grouped marking stays server-authoritative
- The DPS meter sits under the target frame instead of near the bottom of the screen, and can be dragged; the position persists, and right-click returns it home
- **The bank window lists the cost of each unpurchased bag slot.** The "Buy Slot" button gave no price and let you click slots out of order; it now shows the gold cost per slot from `BankBagSlotPrices.dbc`, and only the next slot in sequence is buyable - later ones are locked with their price shown. Buying now goes through an "are you sure?" confirmation that names the cost, rather than spending gold on a single click
- The bank has a Sort button that arranges the main bank and every bank bag by quality, then item ID, then stack size - the same ordering as the backpack Sort, driven client-side one swap per frame
- The bank can show every slot as one continuous grid via a "Combine bags" toggle, instead of splitting each bank bag into its own labeled section
- **Spell descriptions resolve their `$`-tokens everywhere, not just in the talent panel.** Buff/aura tooltips, the spellbook, and item "Use/Equip" effect lines showed raw markup like "Stamina and Spirit increased by $s1" or "Restores $/5;s1 health per second". Token substitution now lives on a shared `GameHandler::formatSpellDescription` used by all of them, and additionally handles the `$/N;` division token (food regen) alongside the base-point, duration, cross-spell, and plural/gender forms
- **Spell tooltips now show the full effect Description instead of the terse one-liner.** The client read Spell.dbc's short `Tooltip` string, so a food item said only "Restores 3 health per second" and never mentioned the Well Fed buff it grants. Descriptions now come from the `Description` column (falling back to `Tooltip`), so food reads "Restores 17 health over 18 sec … become well fed and gain 2 Stamina and Spirit for 15 min."
- **Talent tooltips describe what the talent actually does.** The talent panel read Spell.dbc's `Tooltip` column, whose index in the layouts pointed at an empty locale slot, so hovering a talent showed only its name and rank. Tooltips now pull the spell `Description` (added to every expansion's DBC layout at its verified column) and resolve WoW's `$`-token grammar against live spell data - `$s/$o/$m/$M` base points and `$d` durations, including cross-spell `$<spellId>` references like `$14201d`, plus `$l`/`$g` plural and gender forms - so "receive a $14201s1% damage bonus for $14201d" reads "receive a 4% damage bonus for 12 sec". Tokens with no local source (`$h` proc chance, `$t` period) are stripped cleanly rather than shown raw. An unlearned talent shows its rank-1 effect under "Effect:"
- **Spell.dbc `Tooltip` (and TBC `Rank`) columns pointed at the wrong offset.** The WotLK layout's `Tooltip` and the TBC layout's `Rank`/`Tooltip` indices assumed the wrong locale-block stride, landing on empty columns; they now point at the verified enUS strings, so spell rank text and short tooltips resolve on those clients too
- **The auction "Create Auction" item picker lists everything you can sell.** It only walked the 16-slot base backpack, so items in equipped bags never appeared; it now enumerates the backpack and every equipped bag, and posts the chosen item by its server GUID so any container works
- **New mail announces itself.** Receiving mail while online - or logging in with mail already waiting - now prints "You have new mail." to chat and plays a notification cue, on top of the existing minimap indicator; it fires once per unread state rather than repeating while the mail sits unread
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

## [v2.0.7-preview] - 2026-07-12

### Camera
- **Hills no longer clip through the third-person camera.** Terrain was only ever a floor clamp at the camera's final position, so a rise *between* the character and the camera sliced straight through the view and the clamp just popped the camera upward after the fact. The camera ray now marches the terrain heightfield (coarse ~1.25-unit steps, then bisection for a tight limit) and the resulting distance feeds the same asymmetric pull-in/recover smoothing as the WMO wall raycast. Worst case is ~28 bilinear height lookups a frame - negligible. Pull-in snaps 1:1 while the mouse or turn keys are actively rotating, since the 60 ms ease was exactly the window where a fast swing into a hillside still dipped underground
- The terrain march is skipped inside interior WMOs (terrain above a tunnel is not a real occluder) and when the pivot itself sits below the heightfield (caves, WMO basements, ADT holes), so it cannot pin the camera to first-person where the heightfield is irrelevant
- X now dives while swimming instead of toggling sit, water-exit assists are suppressed while diving, and the swim-depth gate only applies on water entry - deliberate dives can go arbitrarily deep

### UI
- Crafting panel reagent lines show live have/need counts, recounted every frame so consumption is visible mid-craft; Create/Create All disable when any reagent is short

---

## [v2.0.6-preview] - 2026-07-12

### Networking
- **Stop dropping every packet that has no payload.** `handlePacket()` ignored any packet whose body was empty, but `getSize()` is the payload length and the opcode is carried separately - for the many opcodes with no payload, the opcode *is* the message. All of them were swallowed before dispatch, which is also why no "unhandled opcode" warning ever fired for them. Eight registered opcodes were affected:
  - `SMSG_LOGOUT_COMPLETE` - the server logged the character out and moved on while the client waited forever, so the logout countdown ended and nothing happened
  - `SMSG_LOGOUT_CANCEL_ACK` - a cancelled logout was never confirmed
  - `SMSG_ATTACKSWING_NOTINRANGE` / `_BADFACING` / `_DEADTARGET` / `_CANT_ATTACK` - none of the four auto-attack errors ever reached the player
  - `SMSG_PET_BROKEN`, `SMSG_INVALIDATE_PLAYER`

### Logout
- `/quit` and `/exit` now leave the game when the server confirms the logout; `/logout` and `/camp` return to character select. They were all aliases of one command that did neither
- `/logout` was not a command at all: `aliases()` is the complete name list and it only listed camp, quit and exit, so `/help` had been advertising a command that silently did nothing
- The logout pose is a sit again. The server stuns the player to root them for the countdown, and we mapped `UNIT_FLAG_STUNNED` straight to the stun animation, which played over the sit and left the character slumped

### UI
- The breath, fatigue and feign-death bars count down. `SMSG_START_MIRROR_TIMER` hands the client a remaining time and a scale and the server then only re-syncs on change, but nothing ticked the value - so the breath bar sat frozen while you drowned. The sub-millisecond remainder is carried, so the timer does not run slow at high frame rates
- Fix the tail of `SMSG_START_MIRROR_TIMER`: it is paused(1) + spellId(4), not the reverse

---

## [v2.0.5-preview] - 2026-07-12

### Build
- **The build was shipping shaders compiled on 4 April.** `compile_shaders()` wrote its SPIR-V into the runtime tree, and the POST_BUILD step then copied the whole source `assets/` directory over it - including the `.spv` files tracked in git. The stale committed shaders won every build, so three months of GLSL edits never reached the GPU. Seven shaders were affected: `character.frag`, `character.vert`, `terrain.frag`, `water.frag`, `m2_particle.frag`, and both FSR2 compute shaders. Shaders now compile in place next to the GLSL, so the tracked `.spv` and the shader the GPU runs cannot diverge

### Character Select
- Preview is larger (panel widened, render target raised to 640x800), shows the character's equipped weapons, and stands them in their racial glue scene - Stormwind for humans, Durotar for orcs, and so on
- Scene backdrops are placed from the camera and attachment point the M2 carries (M2Loader now parses cameras); their geometry sits hundreds of units from the model origin, so nothing else can position them
- Weapons and enchant visuals no longer leak between characters: weapon attachment ran past an early return for characters whose body skin could not be composited, and fixed model ids meant every character after the first was handed the first one's weapon model

### Rendering
- Backdrops are no longer erased by the character alpha heuristics. Stormwind's walls are DXT5 with an unused alpha channel (mean alpha 17/255, every texel below the cutoff), and inferring a cutout from "the texture has alpha" discarded the whole building, leaving the sky showing through it

### Item Enhancements
- Temporary weapon enchants show as the weapon's icon with its remaining time, in the right slot. SMSG_ITEM_ENCHANT_TIME_UPDATE carries the item's *enchantment* slot (TEMP_ENCHANTMENT_SLOT = 1), not the equipment slot, so every temporary enchant was labelled "Off Hand" - even on a two-hander

### Merged
- Extract `buildFactionHostilityMap()` into a shared free function (#95)

---

## [v2.0.4-preview] - 2026-07-12

### UI
- Show the build version and date bottom-left on the login screen and right-aligned in the settings window
- `core/version.hpp` is generated from `git describe --tags --abbrev=0` by `cmake/GitVersion.cmake`, so the client always reports the last tagged release. It regenerates on every build rather than only when cmake reconfigures, and rewrites the header only when the version actually changed
- The build stamp is a date, not a timestamp: a clock time would change the header every build and force a full recompile of everything including it

### Build
- Un-ignore `cmake/*.cmake`. The repo's blanket `*.cmake` rule targets CMake build output and would have silently excluded the new hand-written module from the tree, breaking a fresh clone

---

## [v2.0.3-preview] - 2026-07-12

### Item Enhancements (sharpening stones, weightstones, weapon oils)
- Send TARGET_FLAG_ITEM in CMSG_USE_ITEM. Item-enhancement consumables cast their spell onto another item, but the client only ever wrote a unit or self target, so the server dropped the cast and the item did nothing
- Using such an item now reads the on-use spell's Spell.dbc Targets mask and arms an item-targeting cursor; the next item clicked (in bags or equipped) receives the enchant. Escape or right-click cancels
- Add the Spell.dbc `Targets` column to all four expansion layouts (Classic 13, TBC 14, WotLK 16)
- Weapon enchant visuals: resolve SpellItemEnchantment → ItemVisuals → ItemVisualEffects and attach the effect M2 (e.g. the sharpening-stone glint) to the weapon model's item-visual attachment points, rendered additive and unlit
- Applying an enchant now marks equipment dirty even though the displayInfoId is unchanged, so the visual appears without re-equipping

### Bug Fixes
- Read enchant names from the correct SpellItemEnchantment.dbc column. The name moved across expansions (Vanilla 10, TBC 13, WotLK 14) but every caller used field 8, an integer column that getString() treated as a string-block offset - so names came back garbled mid-string ("Sharpened (+2 Damage)" surfaced as "ockbiter 3"). Resolved from the record width via `detectEnchantmentNameField()`

### Tests
- New `test_use_item_packet` suite: CMSG_USE_ITEM SpellCastTargets encoding for WotLK, Classic and TBC (item, unit, and self targeting)
- DBC tests for enchant name/ItemVisual column detection and the enchant → effect-model resolution chain

---

## [v1.9.7-preview] - 2026-07-09

### Bug Fixes
- Fix world login pipeline: login-critical opcodes (AUTH_CHALLENGE, AUTH_RESPONSE, CHAR_ENUM, CHAR_CREATE, CHAR_DELETE, WARDEN_DATA) now fall back to hardcoded wire values when opcode table lookup fails, preventing "Unhandled world opcode: 0x1ec" blocking character list retrieval (issue #87)
- OpcodeTable::loadFromJson() now loads into temporaries and only swaps on success - a failed reload no longer wipes the working table
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

## [v1.9.1-preview] - released, captures changes since v1.8.9-preview

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
  waypoint_data, quest_template - ready-to-import .sql files)
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

## [v1.8.1-preview] - 2026-03-23

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
