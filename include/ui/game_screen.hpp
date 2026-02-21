#pragma once

#include "game/game_handler.hpp"
#include "game/inventory.hpp"
#include "rendering/world_map.hpp"
#include "rendering/character_preview.hpp"
#include "ui/inventory_screen.hpp"
#include "ui/quest_log_screen.hpp"
#include "ui/spellbook_screen.hpp"
#include "ui/talent_screen.hpp"
#include <GL/glew.h>
#include <imgui.h>
#include <string>
#include <unordered_map>

namespace wowee {
namespace pipeline { class AssetManager; }
namespace ui {

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

    void saveSettings();
    void loadSettings();

private:
    // Chat state
    char chatInputBuffer[512] = "";
    char whisperTargetBuffer[256] = "";
    bool chatInputActive = false;
    int selectedChatType = 0;  // 0=SAY, 1=YELL, 2=PARTY, 3=GUILD, 4=WHISPER
    int lastChatType = 0;  // Track chat type changes
    bool chatInputMoveCursorToEnd = false;

    // Chat tabs
    int activeChatTab_ = 0;
    struct ChatTab {
        std::string name;
        uint32_t typeMask;  // bitmask of ChatType values to show
    };
    std::vector<ChatTab> chatTabs_;
    void initChatTabs();
    bool shouldShowMessage(const game::MessageChatData& msg, int tabIndex) const;

    // UI state
    bool showEntityWindow = false;
    bool showChatWindow = true;
    bool showPlayerInfo = false;
    bool showGuildRoster_ = false;
    std::string selectedGuildMember_;
    bool showGuildNoteEdit_ = false;
    bool editingOfficerNote_ = false;
    char guildNoteEditBuffer_[256] = {0};
    bool refocusChatInput = false;
    bool vendorBagsOpened_ = false;  // Track if bags were auto-opened for current vendor session
    bool chatWindowLocked = true;
    ImVec2 chatWindowPos_ = ImVec2(0.0f, 0.0f);
    bool chatWindowPosInit_ = false;
    bool showEscapeMenu = false;
    bool showEscapeSettingsNotice = false;
    bool showSettingsWindow = false;
    bool settingsInit = false;
    bool pendingFullscreen = false;
    bool pendingVsync = false;
    int pendingResIndex = 0;
    bool pendingShadows = false;
    int pendingMasterVolume = 100;
    int pendingMusicVolume = 30;
    int pendingAmbientVolume = 100;
    int pendingUiVolume = 100;
    int pendingCombatVolume = 100;
    int pendingSpellVolume = 100;
    int pendingMovementVolume = 100;
    int pendingFootstepVolume = 100;
    int pendingNpcVoiceVolume = 100;
    int pendingMountVolume = 100;
    int pendingActivityVolume = 100;
    float pendingMouseSensitivity = 0.2f;
    bool pendingInvertMouse = false;
    int pendingUiOpacity = 65;
    bool pendingMinimapRotate = false;
    bool pendingMinimapSquare = false;
    bool pendingMinimapNpcDots = false;
    bool pendingSeparateBags = true;
    bool pendingAutoLoot = false;
    bool pendingUseOriginalSoundtrack = true;
    int pendingGroundClutterDensity = 100;

    // UI element transparency (0.0 = fully transparent, 1.0 = fully opaque)
    float uiOpacity_ = 0.65f;
    bool minimapRotate_ = false;
    bool minimapSquare_ = false;
    bool minimapNpcDots_ = false;
    bool minimapSettingsApplied_ = false;
    bool volumeSettingsApplied_ = false;  // True once saved volume settings applied to audio managers

    // Mute state: mute bypasses master volume without touching slider values
    bool soundMuted_ = false;
    float preMuteVolume_ = 1.0f;  // AudioEngine master volume before muting

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
    void renderBagBar(game::GameHandler& gameHandler);
    void renderXpBar(game::GameHandler& gameHandler);
    void renderCastBar(game::GameHandler& gameHandler);
    void renderCombatText(game::GameHandler& gameHandler);
    void renderPartyFrames(game::GameHandler& gameHandler);
    void renderGroupInvitePopup(game::GameHandler& gameHandler);
    void renderBuffBar(game::GameHandler& gameHandler);
    void renderLootWindow(game::GameHandler& gameHandler);
    void renderGossipWindow(game::GameHandler& gameHandler);
    void renderQuestDetailsWindow(game::GameHandler& gameHandler);
    void renderQuestRequestItemsWindow(game::GameHandler& gameHandler);
    void renderQuestOfferRewardWindow(game::GameHandler& gameHandler);
    void renderVendorWindow(game::GameHandler& gameHandler);
    void renderTrainerWindow(game::GameHandler& gameHandler);
    void renderTaxiWindow(game::GameHandler& gameHandler);
    void renderDeathScreen(game::GameHandler& gameHandler);
    void renderResurrectDialog(game::GameHandler& gameHandler);
    void renderEscapeMenu();
    void renderSettingsWindow();
    void renderQuestMarkers(game::GameHandler& gameHandler);
    void renderMinimapMarkers(game::GameHandler& gameHandler);
    void renderGuildRoster(game::GameHandler& gameHandler);
    void renderGuildInvitePopup(game::GameHandler& gameHandler);
    void renderChatBubbles(game::GameHandler& gameHandler);
    void renderMailWindow(game::GameHandler& gameHandler);
    void renderMailComposeWindow(game::GameHandler& gameHandler);
    void renderBankWindow(game::GameHandler& gameHandler);
    void renderGuildBankWindow(game::GameHandler& gameHandler);
    void renderAuctionHouseWindow(game::GameHandler& gameHandler);

    /**
     * Inventory screen
     */
    void renderWorldMap(game::GameHandler& gameHandler);

    InventoryScreen inventoryScreen;
    uint64_t inventoryScreenCharGuid_ = 0;  // GUID of character inventory screen was initialized for
    QuestLogScreen questLogScreen;
    SpellbookScreen spellbookScreen;
    TalentScreen talentScreen;
    rendering::WorldMap worldMap;

    // Spell icon cache: spellId -> GL texture ID
    std::unordered_map<uint32_t, GLuint> spellIconCache_;
    // SpellIconID -> icon path (from SpellIcon.dbc)
    std::unordered_map<uint32_t, std::string> spellIconPaths_;
    // SpellID -> SpellIconID (from Spell.dbc field 133)
    std::unordered_map<uint32_t, uint32_t> spellIconIds_;
    bool spellIconDbLoaded_ = false;
    GLuint getSpellIcon(uint32_t spellId, pipeline::AssetManager* am);

    // Action bar drag state (-1 = not dragging)
    int actionBarDragSlot_ = -1;
    GLuint actionBarDragIcon_ = 0;

    // Bag bar state
    GLuint backpackIconTexture_ = 0;
    GLuint emptyBagSlotTexture_ = 0;
    int bagBarPickedSlot_ = -1;   // Visual drag in progress (-1 = none)
    int bagBarDragSource_ = -1;   // Mouse pressed on this slot, waiting for drag or click (-1 = none)

    // Chat settings
    bool chatShowTimestamps_ = false;
    int chatFontSize_ = 1;  // 0=small, 1=medium, 2=large
    bool chatAutoJoinGeneral_ = true;
    bool chatAutoJoinTrade_ = true;
    bool chatAutoJoinLocalDefense_ = true;
    bool chatAutoJoinLFG_ = true;
    bool chatAutoJoinLocal_ = true;

    // Join channel input buffer
    char joinChannelBuffer_[128] = "";

    static std::string getSettingsPath();

    // Gender placeholder replacement
    std::string replaceGenderPlaceholders(const std::string& text, game::GameHandler& gameHandler);

    // Chat bubbles
    struct ChatBubble {
        uint64_t senderGuid = 0;
        std::string message;
        float timeRemaining = 0.0f;
        float totalDuration = 0.0f;
        bool isYell = false;
    };
    std::vector<ChatBubble> chatBubbles_;
    bool chatBubbleCallbackSet_ = false;

    // Mail compose state
    char mailRecipientBuffer_[256] = "";
    char mailSubjectBuffer_[256] = "";
    char mailBodyBuffer_[2048] = "";
    int mailComposeMoney_[3] = {0, 0, 0};  // gold, silver, copper

    // Auction house UI state
    char auctionSearchName_[256] = "";
    int auctionLevelMin_ = 0;
    int auctionLevelMax_ = 0;
    int auctionQuality_ = 0;
    int auctionSellDuration_ = 2;  // 0=12h, 1=24h, 2=48h
    int auctionSellBid_[3] = {0, 0, 0};     // gold, silver, copper
    int auctionSellBuyout_[3] = {0, 0, 0};  // gold, silver, copper
    int auctionSelectedItem_ = -1;

    // Guild bank money input
    int guildBankMoneyInput_[3] = {0, 0, 0};  // gold, silver, copper

    // Left-click targeting: distinguish click from camera drag
    glm::vec2 leftClickPressPos_ = glm::vec2(0.0f);
    bool leftClickWasPress_ = false;

    // Level-up ding animation
    static constexpr float DING_DURATION = 3.0f;
    float dingTimer_ = 0.0f;
    uint32_t dingLevel_ = 0;
    void renderDingEffect();

public:
    void triggerDing(uint32_t newLevel);
};

} // namespace ui
} // namespace wowee
