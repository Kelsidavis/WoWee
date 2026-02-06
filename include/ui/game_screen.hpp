#pragma once

#include "game/game_handler.hpp"
#include "game/inventory.hpp"
#include "rendering/world_map.hpp"
#include "ui/inventory_screen.hpp"
#include "ui/spellbook_screen.hpp"
#include <imgui.h>
#include <string>
#include <unordered_map>

namespace wowee { namespace ui {

/**
 * In-game screen UI
 *
 * Displays player info, entity list, chat, and game controls
 */
class GameScreen {
public:
    GameScreen();

    /**
     * Render the UI
     * @param gameHandler Reference to game handler
     */
    void render(game::GameHandler& gameHandler);

    /**
     * Check if chat input is active
     */
    bool isChatInputActive() const { return chatInputActive; }

    /**
     * Toggle the teleporter panel
     */
    void toggleTeleporter() { showTeleporter = !showTeleporter; }

    /**
     * Check if teleporter panel is open
     */
    bool isTeleporterOpen() const { return showTeleporter; }

private:
    // Chat state
    char chatInputBuffer[512] = "";
    bool chatInputActive = false;
    int selectedChatType = 0;  // 0=SAY, 1=YELL, 2=PARTY, etc.

    // UI state
    bool showEntityWindow = false;
    bool showChatWindow = true;
    bool showPlayerInfo = false;
    bool refocusChatInput = false;
    bool showTeleporter = false;
    bool showEscapeMenu = false;
    bool showEscapeSettingsNotice = false;
    bool showSettingsWindow = false;
    bool settingsInit = false;
    bool pendingFullscreen = false;
    bool pendingVsync = false;
    int pendingResIndex = 0;
    bool pendingShadows = true;
    int pendingMusicVolume = 30;
    int pendingSfxVolume = 100;
    float pendingMouseSensitivity = 0.2f;
    bool pendingInvertMouse = false;

    /**
     * Render player info window
     */
    void renderPlayerInfo(game::GameHandler& gameHandler);

    /**
     * Render entity list window
     */
    void renderEntityList(game::GameHandler& gameHandler);

    /**
     * Render chat window
     */
    void renderChatWindow(game::GameHandler& gameHandler);

    /**
     * Send chat message
     */
    void sendChatMessage(game::GameHandler& gameHandler);

    /**
     * Get chat type name
     */
    const char* getChatTypeName(game::ChatType type) const;

    /**
     * Get chat type color
     */
    ImVec4 getChatTypeColor(game::ChatType type) const;

    /**
     * Render player unit frame (top-left)
     */
    void renderPlayerFrame(game::GameHandler& gameHandler);

    /**
     * Render target frame
     */
    void renderTargetFrame(game::GameHandler& gameHandler);

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

    // ---- New UI renders ----
    void renderActionBar(game::GameHandler& gameHandler);
    void renderXpBar(game::GameHandler& gameHandler);
    void renderCastBar(game::GameHandler& gameHandler);
    void renderCombatText(game::GameHandler& gameHandler);
    void renderPartyFrames(game::GameHandler& gameHandler);
    void renderGroupInvitePopup(game::GameHandler& gameHandler);
    void renderBuffBar(game::GameHandler& gameHandler);
    void renderLootWindow(game::GameHandler& gameHandler);
    void renderGossipWindow(game::GameHandler& gameHandler);
    void renderVendorWindow(game::GameHandler& gameHandler);
    void renderTeleporterPanel();
    void renderEscapeMenu();
    void renderSettingsWindow();

    /**
     * Inventory screen
     */
    void renderWorldMap(game::GameHandler& gameHandler);

    InventoryScreen inventoryScreen;
    SpellbookScreen spellbookScreen;
    rendering::WorldMap worldMap;

    bool actionSpellDbAttempted = false;
    bool actionSpellDbLoaded = false;
    std::unordered_map<uint32_t, std::string> actionSpellNames;
};

}} // namespace wowee::ui
