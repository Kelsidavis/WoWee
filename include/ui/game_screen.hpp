#pragma once

#include "game/game_handler.hpp"
#include "game/inventory.hpp"
// WorldMap is now owned by Renderer, accessed via getWorldMap()
#include "rendering/character_preview.hpp"
#include "ui/inventory_screen.hpp"
#include "ui/spellbook_screen.hpp"
#include "ui/keybinding_manager.hpp"
#include "ui/chat_panel.hpp"
#include "ui/toast_manager.hpp"
#include "ui/dialog_manager.hpp"
#include "ui/settings_panel.hpp"
#include "ui/minimap_projection.hpp"
#include "ui/combat_ui.hpp"
#include "ui/action_bar_panel.hpp"
#include "ui/window_manager.hpp"
#include "ui/ui_services.hpp"
#include <vulkan/vulkan.h>
#include <imgui.h>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "ui/scene_pick.hpp"

namespace wowee {
namespace core { class AppearanceComposer; class Window; }
namespace pipeline { class AssetManager; }
namespace rendering { class Renderer; }
namespace ui {

/**
 * In-game screen UI
 *
 * Displays player info, entity list, chat, and game controls
 */
class GameScreen {
public:
    /// Open this client's own settings window.
    ///
    /// Reached from the original interface's game-menu button, which has
    /// nothing of its own to show while GameMenuFrame is suppressed.
    /// Open the settings window, optionally on a named tab.
    ///
    /// FrameXML's game menu routes its Video, Sound and Interface buttons here.
    /// Its own options frames are shells - this client's panel is where the
    /// sixty-odd settings actually live, so handing the menu over must not hand
    /// the settings over with it.
    void openSettings(const char* tab = nullptr) {
        settingsPanel_.showSettingsWindow = true;
        if (tab) settingsPanel_.requestedTab_ = tab;
    }

    /// Put the inspect window up, whichever interface draws it.
    ///
    /// Seven places used to set a flag behind this client's own window and its
    /// render was gated on FrameXML not owning the element, so with that handed
    /// over inspecting sent the request and showed nothing. The request goes
    /// out from the caller either way - this is only the window.
    ///
    /// The last thing SocialPanel did. That class had owned the party frames,
    /// the boss frames, the guild roster, the friends list, the dungeon finder
    /// and the who window, all of which are FrameXML's now, and what was left
    /// was this one line beside four unread text buffers.
    void openInspectWindow(game::GameHandler& gameHandler);

    /// Display pacing, for the interface's gxVSync checkbox.
    ///
    /// Both halves, because there are two records of it: the window has the
    /// real state and the settings panel keeps its own copy, which it reads off
    /// the window once at init and writes to disk on save. Setting only the
    /// window would leave that copy stale, so opening this client's own options
    /// afterwards would show the old value and save it back.
    /// The pacing pair used to be here too. It is the "vsync" setting now,
    /// which keeps both records in one place and saves it as well - this held
    /// only the window and the pending field.
    ///
    /// Windowed or full screen, for gxWindow. Both records again, for the same
    /// reason.
    [[nodiscard]] bool getFullscreen() const;
    void setFullscreen(bool enabled);

    /// The resolution, by position in ui/display_modes.hpp. Both records
    /// again: the settings panel keeps pendingResolutionWidth/Height and the
    /// index it drew the combo with, and writes all three to disk.
    [[nodiscard]] int getResolutionIndex() const;
    void setResolutionIndex(int index);
    /// Anti-aliasing, as a row in the four modes the panels offer.
    [[nodiscard]] int getAntiAliasingIndex() const;
    void setAntiAliasingIndex(int index);

    // Gamma, as WoW's video options mean it: 1.0 is untouched, and the client
    // keeps the same number as a 0-100 brightness where 50 is neutral. Exposed
    // so the interface's own brightness slider drives the one setting rather
    // than a second copy of it.
    [[nodiscard]] float getGamma() const;
    void  setGamma(float gamma);

    // Nameplates over hostile and neutral units, which the V key already
    // toggles. Exposed for the same reason gamma is: the interface's
    // nameplateShowEnemies option should drive this flag rather than a second
    // copy of it that disagrees with what the key did.
    [[nodiscard]] bool getShowNameplates() const { return showNameplates_; }
    void setShowNameplates(bool shown) { showNameplates_ = shown; }

    /// Hand the saved anti-aliasing setting to the renderer.
    ///
    /// Called once at startup, before the first frame. It used to be applied
    /// from update(), which does not run until the game screen is up - so a
    /// saved setting rebuilt the swapchain around fifteen hundred frames into
    /// the session, with a world loaded and uploads in flight, rather than
    /// against a renderer that has drawn nothing yet.
    void applySavedAntiAliasing(rendering::Renderer* renderer);
    /// The saved windowed/fullscreen choice and vsync, applied before the
    /// first frame. Neither reaches the WindowConfig the window is built from.
    void applySavedDisplayMode(core::Window* window);

    GameScreen();

    /**
     * Render the UI
     * @param gameHandler Reference to game handler
     */
    void render(game::GameHandler& gameHandler);

    /**
     * Check if chat input is active
     */
    ChatPanel& getChatPanel() { return chatPanel_; }

    void saveSettings();

    /// The settings the client owns. Exposed so the Lua bridge can drive the
    /// same values FrameXML's own options panels are bound to.
    SettingsPanel& getSettingsPanel() { return settingsPanel_; }
    void loadSettings();

    // Dependency injection for extracted classes (Phase A singleton breaking)
    void setAppearanceComposer(core::AppearanceComposer* ac) { appearanceComposer_ = ac; }

    // UIServices injection (Phase B singleton breaking)
    void setServices(const UIServices& services);

    /// Save a screenshot where the client's own binding and /screenshot put it.
    ///
    /// Public because the interface's Screenshot() reaches the same call - a
    /// second path would name and place the file differently. Took a
    /// GameHandler it never read.
    void takeScreenshot();

private:
    void applyCameraControlSettings();

    // Injected UI services (Section 3.5 Phase B - replaces getInstance() calls)
    UIServices services_;
    // Legacy pointer for Phase A compatibility (will be removed when all callsites migrate)
    core::AppearanceComposer* appearanceComposer_ = nullptr;
    // Chat panel (extracted from GameScreen - owns all chat state and rendering)
    ChatPanel chatPanel_;

    // Toast manager (extracted from GameScreen - owns all toast/notification state and rendering)
    ToastManager toastManager_;

    // Dialog manager (extracted from GameScreen - owns all popup/dialog rendering)
    DialogManager dialogManager_;

    // Settings panel (extracted from GameScreen - owns all settings UI and config state)
    SettingsPanel settingsPanel_;

    // Combat UI (extracted from GameScreen - owns all combat overlay rendering)
    CombatUI combatUI_;

    // Action bar panel (extracted from GameScreen - owns action/stance/bag/xp/rep bars)
    ActionBarPanel actionBarPanel_;

    // Window manager (extracted from GameScreen - owns NPC windows, popups, overlays)
    WindowManager windowManager_;

    // UI state
    bool showEntityWindow = false;
    bool showMinimap_ = true;  // M key toggles minimap
    bool showNameplates_ = true;  // V key toggles enemy/NPC nameplates
    uint64_t nameplateCtxGuid_ = 0; // GUID of nameplate right-clicked (0 = none)
    ImVec2 nameplateCtxPos_{};      // Screen position of nameplate right-click
    uint32_t lastPlayerHp_ = 0;   // Previous frame HP for damage flash detection
    float damageFlashAlpha_ = 0.0f; // Screen edge flash intensity (fades to 0)


    // UIErrorsFrame: WoW-style center-bottom error messages (spell fails, out of range, etc.)
    bool uiErrorCallbackSet_ = false;

    bool showPlayerInfo = false;
    bool showWorldMap_ = false;  // W key toggles world map
    /// The screen width the tracker was last placed against.
    ///
    /// Its position is recomputed from the right edge whenever the window
    /// resizes, and that recompute is why the tracker could not be dragged:
    /// forcing the position every frame put it straight back. Forced only when
    /// the width actually changed now, so the rest of the time the window owns
    /// where it is and a drag survives.



    /**
     * Render player info window
     */
    void renderPlayerInfo(game::GameHandler& gameHandler);

    /**
     * Render entity list window
     */
    void renderEntityList(game::GameHandler& gameHandler);

    // Halves of the saved DPS meter position, applied once both have been read
    // (settings arrive as separate key/value lines).
    float dpsMeterSavedX_ = -1.0f;
    float dpsMeterSavedY_ = -1.0f;

    /**
     * Render pet frame (below player frame when player has an active pet)
     */

    /**
     * Process targeting input (Tab, Escape, click)
     */
    void processTargetInput(game::GameHandler& gameHandler);

    /**
     * Rebuild character geosets from current equipment state
     */
    void updateCharacterGeosets(game::Inventory& inventory);

    /**
     * Re-composite character skin texture from current equipment
     */
    void updateCharacterTextures(game::Inventory& inventory);


    void renderMinimapMarkers(game::GameHandler& gameHandler);

    /// The furniture around the minimap - mute, friends and zoom buttons, the
    /// clock, and the stack of indicators below it. Separate from the marker
    /// pass because it answers the ownership question the other way: FrameXML's
    /// cluster brings its own, and the blips it does not bring at all.
    /// Where the minimap is this frame, and how to put a thing on it.
    ///
    /// Every marker category needs the same eight values and the same three
    /// projections. They were locals of one thousand-line function, which is
    /// what kept the categories in it.
    struct MinimapFrame {
        ImDrawList* drawList = nullptr;
        float centerX = 0.0f;
        float centerY = 0.0f;
        float mapRadius = 0.0f;
        float bearing = 0.0f;
        glm::vec3 playerRender{0.0f};
        MinimapView view{};

        /// A render position onto the disc, or false when it falls outside.
        /// The rim minus three units, which keeps a blip off the border.
        bool project(const glm::vec3& worldRenderPos, float& sx, float& sy) const;

        /// An entity's own position, converted on the way. The conversion is one
        /// fact - these two coordinate systems are not the same one - and it was
        /// written at fourteen call sites before it was here.
        bool projectEntity(const game::Entity& entity, float& sx, float& sy) const;

        /// For the things that arrive as a bare pair of canonical coordinates:
        /// party members, pings, gossip points, battleground positions. They
        /// have no height and need none - the minimap is flat.
        bool projectCanonical(float wowX, float wowY, float& sx, float& sy) const;
    };

    /// The entity lists the marker categories walk, partitioned once per frame.
    using EntityList = std::vector<std::shared_ptr<game::Entity>>;
    using EntrySet = std::unordered_set<uint32_t>;

    /// One marker category each. They were fourteen blocks of one function and
    /// are independent of each other - each walks its own list and draws its own
    /// shape - so the only thing that kept them together was the locals they
    /// shared, which MinimapFrame now carries.
    /// Quest-giver status per NPC guid, as the server reports it.
    using QuestStatusMap = std::unordered_map<uint64_t, game::QuestGiverStatus>;

    void renderMinimapNpcDots(const MinimapFrame& frame, const EntityList& units,
                              const EntrySet& questEntries);
    void renderMinimapFlightMasters(const MinimapFrame& frame, const EntityList& units);
    void renderMinimapRares(const MinimapFrame& frame, const EntityList& units,
                            game::GameHandler& gameHandler);
    void renderMinimapPlayerDots(const MinimapFrame& frame, const EntityList& players,
                                 game::GameHandler& gameHandler);
    void renderMinimapLootCorpses(const MinimapFrame& frame, const EntityList& units);
    void renderMinimapObjectDots(const MinimapFrame& frame, const EntityList& objects,
                                 const EntrySet& questGoEntries,
                                 game::GameHandler& gameHandler);
    void renderMinimapChests(const MinimapFrame& frame, const EntityList& objects,
                             game::GameHandler& gameHandler);

    void renderMinimapQuestGivers(const MinimapFrame& frame, const QuestStatusMap& statuses,
                                  game::GameHandler& gameHandler);
    void renderMinimapQuestKills(const MinimapFrame& frame, const EntityList& units,
                                 const QuestStatusMap& statuses,
                                 game::GameHandler& gameHandler);
    void renderMinimapGossipPois(const MinimapFrame& frame, game::GameHandler& gameHandler);
    void renderMinimapPings(const MinimapFrame& frame, game::GameHandler& gameHandler);
    void renderMinimapPartyDots(const MinimapFrame& frame, game::GameHandler& gameHandler);
    void renderMinimapBattlegroundPositions(const MinimapFrame& frame,
                                            game::GameHandler& gameHandler);
    void renderMinimapCorpseMarker(const MinimapFrame& frame, game::GameHandler& gameHandler);
    void renderMinimapPlayerArrow(const MinimapFrame& frame);

    /// The wheel and the ctrl+click, when this client owns the ring.
    void handleMinimapInput(const MinimapFrame& frame, game::GameHandler& gameHandler,
                            bool minimapInputBlocked);

    /// The coordinates, zone name, difficulty and hover menu written on the ring.
    void renderMinimapReadouts(const MinimapFrame& frame, game::GameHandler& gameHandler,
                               bool minimapInputBlocked);

    /// The mute, friends and zoom buttons around the ring.
    void renderMinimapButtons(game::GameHandler& gameHandler, float centerX,
                              float centerY, float mapRadius);

    /// The optional clock at the bottom right of the ring.
    void renderMinimapClock(float centerX, float centerY, float mapRadius);

    /// The stack under the minimap - mail, talent points, queues, latency,
    /// durability. One function because they share a running Y and stack
    /// without gaps whichever of them apply.
    void renderMinimapIndicators(game::GameHandler& gameHandler, float centerX,
                                 float centerY, float mapRadius);

    void renderMinimapChrome(game::GameHandler& gameHandler, float centerX,
                             float centerY, float mapRadius);
    void refreshQuestObjectiveCache(game::GameHandler& gameHandler);
    void renderNameplates(game::GameHandler& gameHandler);

    /**
     * Inventory screen
     */
    void renderWorldMap(game::GameHandler& gameHandler);

    InventoryScreen inventoryScreen;
    uint64_t inventoryScreenCharGuid_ = 0;  // GUID of character inventory screen was initialized for
    SpellbookScreen spellbookScreen;
    // WorldMap is now owned by Renderer (accessed via renderer->getWorldMap())

    // Spell icon cache: spellId -> GL texture ID
    std::unordered_map<uint32_t, VkDescriptorSet> spellIconCache_;
    // SpellIconID -> icon path (from SpellIcon.dbc)
    std::unordered_map<uint32_t, std::string> spellIconPaths_;
    // SpellID -> SpellIconID (from the active expansion's Spell.dbc layout)
    std::unordered_map<uint32_t, uint32_t> spellIconIds_;
    bool spellIconDbLoaded_ = false;
    VkDescriptorSet getSpellIcon(uint32_t spellId, pipeline::AssetManager* am);
    /// The vendor cursor, drawn in place of the pointer. True when it drew, so
    /// the caller knows not to also ask for the hand.
    bool drawVendorCursor(game::GameHandler& gameHandler, const ui::ScenePick& pick);

    // Minimap quest-objective cache, rebuilt only when tracked quest progress changes.
    uint64_t minimapQuestCacheSignature_ = 0;
    std::unordered_set<uint32_t> minimapQuestCreatureEntries_;
    std::unordered_set<uint32_t> minimapQuestGameObjectEntries_;

    // Death Knight rune bar: client-predicted fill (0.0=depleted, 1.0=ready) for smooth animation
    float runeClientFill_[6] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

    // Pet rename modal (triggered from pet frame context menu)

    // Left-click targeting: distinguish click from camera drag
    glm::vec2 leftClickPressPos_ = glm::vec2(0.0f);
    bool leftClickWasPress_ = false;
    // Right-click interact/attack fires on release only if the press was a tap, not a
    // camera-rotation drag - so turning the view near a mob doesn't auto-attack it.
    glm::vec2 rightClickPressPos_ = glm::vec2(0.0f);
    bool rightClickWasPress_ = false;

    /// An item dropped onto another player, waiting for the trade it opened.
    ///
    /// CMSG_INITIATE_TRADE is a request; the item cannot be offered until the
    /// server answers with the trade window. So the slot it came from is held
    /// here and put in the first trade slot when that arrives.
    struct PendingTradeItem {
        bool     active = false;
        uint8_t  bag = 0xFF;
        uint8_t  slot = 0;
    };
    PendingTradeItem pendingTradeItem_;
    /// Offer the parked item once the trade window is up.
    void offerPendingTradeItem(game::GameHandler& gameHandler);


    bool appearanceCallbackSet_ = false;
    bool ghostOpacityStateKnown_ = false;
    bool ghostOpacityLastState_ = false;
    uint32_t ghostOpacityLastInstanceId_ = 0;


    void renderWeatherOverlay(game::GameHandler& gameHandler);

public:
    /// The server asking for the window (SMSG_OPEN_LFG_DUNGEON_FINDER).
    ///
    /// Shown rather than toggled: this is not the player pressing the key, and
    /// a toggle would shut a window that was already open.
    void openDungeonFinder(game::GameHandler& gameHandler) {
        gameHandler.runInterfaceCommand(
            "if not LFDParentFrame:IsShown() then ToggleLFDParentFrame() end");
    }
    ToastManager& toastManager() { return toastManager_; }
    /// Reached by the barber bindings, which need state this owns.
    WindowManager& windowManager() { return windowManager_; }
    ActionBarPanel& actionBarPanel() { return actionBarPanel_; }
};

} // namespace ui
} // namespace wowee
