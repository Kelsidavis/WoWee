// world_map.hpp - Shim header for backward compatibility.
// Redirects to the modular world_map/world_map_facade.hpp.
// Consumers should migrate to #include "rendering/world_map/world_map_facade.hpp" directly.
#pragma once

#include "rendering/world_map/world_map_facade.hpp"

namespace wowee {
namespace rendering {

// Backward-compatible type aliases for old consumer code
// (game_screen_hud.cpp, renderer.cpp, etc.)
using WorldMapPartyDot = world_map::PartyDot;
using WorldMapRareMark = world_map::RareMark;
using WorldMapTaxiNode = world_map::TaxiNode;
using MapPOI           = world_map::POI;

// WorldMap alias is already provided by world_map_facade.hpp:
//   using WorldMap = world_map::WorldMapFacade;
// WorldMap::QuestPoi alias is provided inside WorldMapFacade:
//   using QuestPoi = QuestPOI;

} // namespace rendering
} // namespace wowee
