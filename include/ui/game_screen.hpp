#pragma once

#include "game/game_handler.hpp"
#include "game/inventory.hpp"
#include "ui/inventory_screen.hpp"
#include "ui/spellbook_screen.hpp"
#include <imgui.h>
#include <string>

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
    void renderCastBar(game::GameHandler& gameHandler);
    void renderCombatText(game::GameHandler& gameHandler);
    void renderPartyFrames(game::GameHandler& gameHandler);
    void renderGroupInvitePopup(game::GameHandler& gameHandler);
    void renderBuffBar(game::GameHandler& gameHandler);
    void renderLootWindow(game::GameHandler& gameHandler);
    void renderGossipWindow(game::GameHandler& gameHandler);
    void renderVendorWindow(game::GameHandler& gameHandler);

    /**
     * Inventory screen
     */
    InventoryScreen inventoryScreen;
    SpellbookScreen spellbookScreen;
};

}} // namespace wowee::ui
