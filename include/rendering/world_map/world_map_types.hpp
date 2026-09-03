// world_map_types.hpp - Vulkan-free domain types for the world map system.
// Extracted from rendering/world_map.hpp (Phase 1 of refactoring plan).
// Consumers of these types do NOT need Vulkan/VMA headers.
#pragma once

#include <glm/glm.hpp>
#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace rendering {
namespace world_map {

// ── View hierarchy ───────────────────────────────────────────

enum class ViewLevel { COSMIC, WORLD, CONTINENT, ZONE };

// ── Transition animation ─────────────────────────────────────

struct TransitionState {
    bool active = false;
    float progress = 0.0f;       // 0.0 → 1.0
    float duration = 0.2f;       // seconds
    ViewLevel fromLevel = ViewLevel::ZONE;
    ViewLevel toLevel = ViewLevel::ZONE;
};

// ── Zone faction & metadata ──────────────────────────────────

enum class ZoneFaction : uint8_t { Neutral, Alliance, Horde, Contested };

struct ZoneMeta {
    uint8_t minLevel = 0, maxLevel = 0;
    ZoneFaction faction = ZoneFaction::Neutral;
};

// ── Cosmic view (cross-realm) ────────────────────────────────

struct CosmicMapEntry {
    int mapId = 0;
    std::string label;
    // Clickable region in UV space (0-1 range on the cosmic composite)
    float uvLeft = 0, uvTop = 0, uvRight = 0, uvBottom = 0;
};

// ── Zone bounds (shared between Zone and coordinate projection) ──

struct ZoneBounds {
    float locLeft = 0, locRight = 0;
    float locTop = 0, locBottom = 0;
};

/// ZMP-derived bounding rectangle in display UV [0,1] coordinates.
/// Computed by scanning the ZMP grid for each zone's area ID.
/// Maps pixel-for-pixel to the continent map tiles shown on screen.
struct ZmpRect {
    float uMin = 0, uMax = 0;
    float vMin = 0, vMax = 0;
    bool valid = false;
};

// ── Overlay entry (exploration overlay from WorldMapOverlay.dbc) ──

struct OverlayEntry {
    uint32_t areaIDs[4] = {};    // Up to 4 AreaTable IDs contributing to this overlay
    std::string textureName;     // Texture prefix (e.g., "Goldshire")
    uint16_t texWidth = 0, texHeight = 0;  // Overlay size in pixels
    uint16_t offsetX = 0, offsetY = 0;     // Pixel offset within zone map
    int tileCols = 0, tileRows = 0;
    // HitRect from WorldMapOverlay.dbc fields 13-16 - fast AABB pre-filter for
    // subzone hover detection in zone view (avoids sampling every overlay).
    uint16_t hitRectLeft = 0, hitRectRight = 0;
    uint16_t hitRectTop = 0, hitRectBottom = 0;
    // NOTE: texture pointers are managed by CompositeRenderer, not stored here.
    bool tilesLoaded = false;
};

// ── Zone (from WorldMapArea.dbc) ─────────────────────────────

struct Zone {
    uint32_t wmaID = 0;
    uint32_t mapID = 0;          // Physical map containing the area
    uint32_t areaID = 0;         // 0 = continent level
    std::string areaName;        // texture folder name (from DBC)
    ZoneBounds bounds;
    uint32_t displayMapID = 0;
    uint32_t parentWorldMapID = 0;
    float virtualOffsetWowX = 0.0f;
    float virtualOffsetWowY = 0.0f;
    std::vector<uint32_t> exploreBits;  // all AreaBit indices (zone + subzones)
    std::vector<OverlayEntry> overlays;
};

// ── Party member dot (UI layer → world map overlay) ──────────

struct PartyDot {
    glm::vec3 renderPos;   ///< Position in render-space coordinates
    uint32_t  color;       ///< RGBA packed color (IM_COL32 format)
    std::string name;      ///< Member name (shown as tooltip on hover)
};

// ── Nearby rare/rare-elite creature marker (UI layer → world map overlay) ─

struct RareMark {
    glm::vec3 renderPos;   ///< Position in render-space coordinates
    std::string name;      ///< Creature name (shown as tooltip on hover)
    int rank = 4;          ///< Creature rank: 2 = Rare Elite, 4 = Rare
};

// ── Taxi (flight master) node (UI layer → world map overlay) ─

struct TaxiNode {
    uint32_t  id = 0;      ///< TaxiNodes.dbc ID
    uint32_t  mapId = 0;   ///< WoW internal map ID (0=EK,1=Kal,530=Outland,571=Northrend)
    float     wowX = 0, wowY = 0, wowZ = 0;  ///< Canonical WoW coordinates
    std::string name;      ///< Node name (shown as tooltip)
    bool      known = false; ///< Player has discovered this node
    // Flight-map (taxi selection mode) data - only meaningful while a flight
    // master window is open and the cost map is built from the current node.
    uint32_t  costCopper = 0;   ///< Total flight cost from the current node
    bool      current = false;  ///< This is the node the player is standing at
    bool      reachable = false;///< A route exists from the current node
};

// ── Area Point of Interest from AreaPOI.dbc ──────────────────

struct POI {
    uint32_t id = 0;
    uint32_t importance = 0;    ///< 0=small, 1=medium, 2=large (capital)
    uint32_t iconType = 0;      ///< Icon category from AreaPOI.dbc
    uint32_t factionId = 0;     ///< 0=neutral, 67=Horde, 469=Alliance
    float wowX = 0, wowY = 0, wowZ = 0;  ///< Canonical WoW coordinates
    uint32_t mapId = 0;         ///< WoW internal map ID
    std::string name;
    std::string description;
};

// ── Quest POI marker (from SMSG_QUEST_POI_QUERY_RESPONSE) ────

struct QuestPOI {
    enum class Kind : uint8_t {
        OBJECTIVE,
        AVAILABLE,
        AVAILABLE_LOW,
        REWARD,
        INCOMPLETE,
    };

    float wowX = 0, wowY = 0;  ///< Canonical WoW coordinates (centroid)
    std::string name;           ///< Quest title
    Kind kind = Kind::OBJECTIVE;
    /// The objective's outline, canonical, when the map is shading it. Empty
    /// for a marker that is only a point, and for a quest not selected.
    std::vector<glm::vec2> area;
};

} // namespace world_map
} // namespace rendering
} // namespace wowee
