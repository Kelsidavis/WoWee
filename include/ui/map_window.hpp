#pragma once

// The world map in a window of its own, for a second monitor.
//
// Its own SDL window, its own swapchain (rendering::AuxSwapchain, drawn into
// the main frame), its own ImGui context - the stock ImGui here has no
// multi-window support, and the SDL backend drops every event that is not for
// the window it was given - and its own map, so the in-game map and this one
// can show different places.
//
// It follows the player: it shows the zone they are in, moves with them when
// they change zones, and can be browsed - zoomed and clicked like the in-game
// map - after which it comes back to their zone once it has been left alone.
//
// Beside the map, the quest log: the player's quests by zone, theirs first,
// with each quest's objectives and how far along they are. Choosing one shows
// its zone and the areas its objectives are in, the way the game's own quest
// map does, and a box beside each tracks it.

#include <SDL3/SDL.h>
#include <glm/glm.hpp>
#include <vulkan/vulkan.h>

#include <cstdint>
#include <functional>
#include <memory>

struct ImGuiContext;

namespace wowee {
namespace rendering {
class AuxSwapchain;
class Renderer;
class VkContext;
namespace world_map { class WorldMapFacade; }
}  // namespace rendering
namespace pipeline { class AssetManager; }
namespace ui {

class UIManager;

class MapWindow {
public:
    MapWindow();
    ~MapWindow();
    MapWindow(const MapWindow&) = delete;
    MapWindow& operator=(const MapWindow&) = delete;

    /// Open the window where it was last closed, or on another monitor than
    /// the game's the first time. False, with the reason logged, when it could
    /// not be made.
    bool open(SDL_Window* gameWindow, rendering::Renderer* renderer,
              rendering::VkContext* ctx, pipeline::AssetManager* assets,
              const UIManager* ui);
    void close();
    [[nodiscard]] bool isOpen() const { return window_ != nullptr; }

    /// True once after the player closed the window with its own close
    /// button, so the setting that opened it can be turned off.
    bool takeClosedByPlayer();

    /// Handles the event if it belongs to this window, and says so. Nothing
    /// else in the client should see those: a click on the map is not a click
    /// on the world behind the game's window.
    ///
    /// Except the keyboard. The map has no use for it, and a player who clicked
    /// the map and pressed an action key meant the game - so key events are
    /// handed back addressed to the game's window, and answer false.
    bool handleEvent(SDL_Event& event);

    /// The quest chosen in this window's list, or zero. Its objective areas
    /// are the ones this window's map shades.
    [[nodiscard]] uint32_t selectedQuest() const { return selectedQuestId_; }

    /// The map, for the code that feeds it the player's party, quests and the
    /// rest. Null while closed.
    [[nodiscard]] rendering::world_map::WorldMapFacade* map() const { return map_.get(); }

    /// Run `fn` with this window's ImGui context current. The map's setters
    /// can free textures, and those belong to whichever context made them.
    void withContext(const std::function<void()>& fn);

    /// Build this frame's picture. After the game's own ImGui frame is
    /// finished and before the frame is submitted; drawn at most thirty times
    /// a second, since a map has no use for more.
    void buildFrame(bool inWorld, const glm::vec3& playerRenderPos, float playerYawDeg,
                    uint32_t playerZoneId);

private:
    void record(VkCommandBuffer cmd);
    void drawQuestPanel(float x, float width, float height, uint64_t now);
    void saveGeometry() const;
    void chooseGeometry(SDL_Window* gameWindow, int& x, int& y, int& w, int& h) const;

    SDL_Window* window_ = nullptr;
    SDL_WindowID gameWindowId_ = 0;
    rendering::Renderer* renderer_ = nullptr;
    rendering::VkContext* ctx_ = nullptr;
    std::unique_ptr<rendering::AuxSwapchain> swapchain_;
    std::unique_ptr<rendering::world_map::WorldMapFacade> map_;
    ImGuiContext* imgui_ = nullptr;

    bool frameReady_ = false;
    bool closedByPlayer_ = false;
    uint64_t lastBuildNs_ = 0;

    // Following the player: the zone last shown for them, and whether the
    // player has taken the map somewhere else since.
    uint32_t lastPlayerZone_ = 0;
    bool browsing_ = false;
    uint64_t lastInteractionNs_ = 0;
    uint32_t selectedQuestId_ = 0;
};

}  // namespace ui
}  // namespace wowee
