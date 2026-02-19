# Project Status

**Last updated**: 2026-02-19

## What This Repo Is

Wowee is a native C++ World of Warcraft client experiment focused on connecting to real emulator servers (online/multiplayer) with a custom renderer and asset pipeline.

## Current Code State

Implemented (working in normal use):

- Auth flow: SRP6a auth + realm list + world connect with header encryption
- Rendering: terrain, WMO/M2 rendering, water, sky system, particles, minimap/world map, loading video playback
- Character system: creation (including nonbinary gender), selection, 3D preview with equipment, character screen
- Core gameplay: movement, targeting, combat, action bar, inventory/equipment, chat (tabs/channels, emotes, item links)
- Quests: quest markers (! and ?) on NPCs/minimap, quest log with detail queries/retry, objective tracking, accept/complete flow, turn-in
- Trainers: spell trainer UI, buy spells, known/available/unavailable states
- Vendors, loot, gossip dialogs (including buyback for most recently sold item)
- Spellbook with class tabs, drag-drop to action bar, spell icons
- Warden anti-cheat: full module execution via Unicorn Engine x86 emulation; module caching
- Audio: ambient, movement, combat, spell, and UI sound systems
- Bag UI: separate bag windows, open-bag indicator on bag bar, optional collapse-empty mode in aggregate bag view
- Multi-expansion: Classic/Vanilla, TBC, WotLK, and Turtle WoW (1.17) protocol and asset variants

In progress / known gaps:

- Transports (ships, zeppelins, elevators): partial support, timing and edge cases still buggy
- 3D positional audio: not implemented (mono/stereo only)
- Visual edge cases: some M2/WMO rendering gaps (character shin mesh, some particle effects)

## Where To Look

- Entry point: `src/main.cpp`, `src/core/application.cpp`
- Networking/auth: `src/auth/`, `src/network/`, `src/game/game_handler.cpp`
- Rendering: `src/rendering/`
- Assets/extraction: `extract_assets.sh`, `tools/asset_extract/`, `src/pipeline/asset_manager.cpp`
