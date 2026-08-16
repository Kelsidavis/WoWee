// taxi_node_layer.hpp - Flight master markers on the world map.
// Two modes: passive diamonds on the normal world map, and the interactive
// flight map (taxi selection) shown while a flight master window is open.
#pragma once
#include "rendering/world_map/overlay_renderer.hpp"
#include "rendering/world_map/world_map_types.hpp"
#include <cstdint>
#include <functional>
#include <vector>

namespace wowee {
namespace rendering {
namespace world_map {

class TaxiNodeLayer : public IOverlayLayer {
public:
    void setNodes(const std::vector<TaxiNode>& nodes) { nodes_ = &nodes; }

    /// Enable/disable the interactive flight-map mode.
    void setTaxiMode(bool active) {
        taxiMode_ = active;
        if (!active) {
            routeProvider_ = nullptr;
            onSelect_ = nullptr;
        }
    }
    [[nodiscard]] bool taxiModeActive() const { return taxiMode_; }

    /// routeProvider: destination node id → hop chain (current → dest, inclusive).
    /// onSelect: called with the destination node id when the player clicks it.
    void setTaxiHandlers(std::function<std::vector<uint32_t>(uint32_t)> routeProvider,
                         std::function<void(uint32_t)> onSelect) {
        routeProvider_ = std::move(routeProvider);
        onSelect_ = std::move(onSelect);
    }

    void render(const LayerContext& ctx) override;

private:
    void renderWorldMapMarkers(const LayerContext& ctx, const ZoneBounds& bounds,
                               bool isContinent);
    void renderFlightMap(const LayerContext& ctx, const ZoneBounds& bounds,
                         bool isContinent);

    const std::vector<TaxiNode>* nodes_ = nullptr;
    bool taxiMode_ = false;
    std::function<std::vector<uint32_t>(uint32_t)> routeProvider_;
    std::function<void(uint32_t)> onSelect_;
};

} // namespace world_map
} // namespace rendering
} // namespace wowee
