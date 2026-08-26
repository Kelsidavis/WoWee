// world_map_facade.hpp - Public API for the world map system.
// Drop-in replacement for the monolithic WorldMap class (Phase 10 of refactoring plan).
// Facade pattern - hides internal complexity behind the same public interface.
#pragma once

#include "rendering/world_map/world_map_types.hpp"
#include <glm/glm.hpp>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>
#include <memory>
#include <vulkan/vulkan.h>

namespace wowee {
namespace rendering {
class VkContext;
}
namespace pipeline { class AssetManager; }
namespace rendering {
namespace world_map {

class WorldMapFacade {
public:
    /// Backward-compatible alias for old WorldMap::QuestPoi usage.
    using QuestPoi = QuestPOI;

    WorldMapFacade();
    ~WorldMapFacade();

    bool initialize(VkContext* ctx, pipeline::AssetManager* am);
    void shutdown();

    /// Off-screen composite pass - call BEFORE the main render pass begins.
    void compositePass(VkCommandBuffer cmd);

    /// ImGui overlay - call INSIDE the main render pass (during ImGui frame).
    void render(const glm::vec3& playerRenderPos,
                int screenWidth, int screenHeight,
                float playerYawDeg = 0.0f);

    void setMapName(const std::string& name);
    void setServerExplorationMask(const std::vector<uint32_t>& masks, bool hasData);
    /// The zone the server says the player is in. Used in preference to working
    /// it out from map geometry, which can only guess between overlapping zones.
    void setPlayerZoneId(uint32_t zoneId);
    void setPartyDots(std::vector<PartyDot> dots);
    void setTaxiNodes(std::vector<TaxiNode> nodes);
    void setQuestPois(std::vector<QuestPOI> pois);
    void setCorpsePos(bool hasCorpse, glm::vec3 renderPos);
    /// The spirit healer a release would send the player to.
    void setGraveyardPos(bool hasGraveyard, glm::vec3 renderPos);

    /// Draw the map into this rect instead of a window of its own.
    ///
    /// In pixels from the top-left. The map is an ImGui window that centres
    /// and sizes itself, which is right when this client's interface owns it
    /// and wrong when FrameXML does: there the map belongs inside the frame
    /// FrameXML drew for it, and the title bar belongs to that frame rather
    /// than to a window underneath it. Unset, nothing changes.
    void setFrameRect(float x, float y, float w, float h);
    void clearFrameRect();
    /// Whether a frame rect is set - which, when FrameXML owns the map, is how
    /// the client knows FrameXML has its map panel on screen and wants the map
    /// drawn into it.
    [[nodiscard]] bool hasFrameRect() const;
    /// Nearby rare/rare-elite creatures currently spawned near the player.
    void setRares(std::vector<RareMark> rares);
    /// Nearby chest-type game objects currently spawned near the player.
    void setChests(std::vector<ChestMark> chests);

    [[nodiscard]] bool isOpen() const;
    void close();

    /// Flight-map (taxi selection) mode - opens the map locked to the player's
    /// continent with interactive flight nodes (see TaxiNodeLayer flight-map
    /// rendering). routeProvider maps a destination node id to the hop chain
    /// used to draw the route; onSelect fires with the chosen destination;
    /// onClose fires when the player dismisses the map (Escape / X).
    void openTaxiMap(std::function<std::vector<uint32_t>(uint32_t)> routeProvider,
                     std::function<void(uint32_t)> onSelect,
                     std::function<void()> onClose);
    /// Silent close (no onClose callback) - used when the game state already
    /// closed the flight master window.
    void closeTaxiMap();
    [[nodiscard]] bool isTaxiMapOpen() const;

    // ── What the interface needs to draw the same map itself ─────────
    //
    // The client keeps all of this and drew it only from here. FrameXML's
    // world map asks for the same two things through GetMapOverlayInfo and
    // GetMapLandmarkInfo, and had no way to reach them - the facade's whole
    // public surface was setters.

    /// The overlays for whatever the map is showing, in draw order. Empty at
    /// continent and cosmic level, which have no per-zone art.
    [[nodiscard]] std::vector<OverlayEntry> currentOverlays() const;

    /// One area POI, already projected into the [0,1] map space the interface
    /// places its pins in.
    struct Landmark {
        std::string name;
        std::string description;
        uint32_t iconType = 0;
        float x = 0.0f, y = 0.0f;
    };
    /// The POIs on the shown map. Only those belonging to it, and only those
    /// that project inside it.
    [[nodiscard]] std::vector<Landmark> currentLandmarks() const;


    /// The zone under a point on the map being shown, in [0,1] map space, or
    /// -1. Continent view only: a zone's own map has no child zones to name.
    ///
    /// The same ZMP lookup the hover highlight does, and pulled out of it so
    /// the interface can ask about a point rather than about the mouse.
    [[nodiscard]] int zoneAtMapPoint(float u, float v) const;

    /// The name of that zone, or empty.
    [[nodiscard]] std::string zoneNameAtMapPoint(float u, float v) const;

    /// Drill into the zone under that point, as clicking it would.
    bool clickMapPoint(float u, float v);

    /// Where a world position falls on the map now showing, as 0..1 across the
    /// image. False when it is off this map, which is how the interface's own
    /// markers ask to be hidden.
    ///
    /// The inverse of zoneAtMapPoint, and the thing GetPlayerMapPosition needs:
    /// FrameXML multiplies what that answers by the map frame's width, so a
    /// world coordinate handed over unconverted puts the player arrow thousands
    /// of pixels off the parchment.
    bool mapUVForCanonical(float wowX, float wowY, float wowZ,
                           float& u, float& v) const;

    /// Step out one level: zone to continent, continent to world.
    void zoomOutOneLevel();

    // ── Navigating by name, the way the interface's dropdowns do ─────
    //
    // The interface offers a continent list and a zone list and sets the map
    // from a pair of indices into them. Only clicking the map itself reached
    // this before, so the two dropdowns and the zoom-out button drove nothing.

    /// The continents, in the order 3.3.5 indexes them: Kalimdor, Eastern
    /// Kingdoms, Outland, Northrend. Fixed rather than discovered, because the
    /// index is what the interface passes back and addons hard-code it.
    /// Whether the map moved itself since this was last asked, and clears it.
    ///
    /// The interface rebuilds its two dropdowns on WORLD_MAP_UPDATE, and the
    /// only move it cannot predict is the one this class makes on its own -
    /// settling on the player's zone a frame after being asked to. Polled
    /// where the frame rect is read, and answered with the event.
    [[nodiscard]] bool takeViewChanged();

    [[nodiscard]] std::vector<std::string> continentNames() const;

    /// The zones on that continent by their display names, alphabetically.
    /// One-based continent index; empty for anything else.
    [[nodiscard]] std::vector<std::string> zoneNames(int continentIndex) const;

    /// Show a continent, or one zone on it. Both indices one-based, as the
    /// interface counts; zone zero means the continent itself. Continent zero
    /// means the world.
    bool showMap(int continentIndex, int zoneIndex);

    /// What is being shown, in those same one-based indices. Zero for the
    /// world, and zero zone for a continent.
    [[nodiscard]] int currentContinentIndex() const;
    [[nodiscard]] int currentZoneIndex() const;

    /// Whether there is a level above this one to step out to.
    [[nodiscard]] bool canZoomOut() const;

    /// Show the zone the player is standing in. The interface asks for this
    /// every time the map is shown, and used to reach nothing.
    void showPlayerZone();

    /// The WorldMapArea id of the zone being shown, or zero when the map is
    /// not showing one. This is the id the interface passes back to
    /// showWorldMapArea - not the AreaTable id, and not the physical map the
    /// terrain came from.
    [[nodiscard]] uint32_t currentWorldMapAreaId() const;

    /// Show the zone or continent with that WorldMapArea id. False if the map
    /// data has no such area.
    bool showWorldMapArea(uint32_t worldMapAreaId);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace world_map
} // namespace rendering
} // namespace wowee

// Backward-compatible alias for gradual migration
namespace wowee {
namespace rendering {
using WorldMap = world_map::WorldMapFacade;
} // namespace rendering
} // namespace wowee
