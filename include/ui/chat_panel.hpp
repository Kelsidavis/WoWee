#pragma once

#include "game/game_handler.hpp"
#include "ui/ui_services.hpp"
#include "ui/chat/chat_settings.hpp"
#include "ui/chat/chat_bubble_manager.hpp"
#include "ui/chat/cast_sequence_tracker.hpp"
#include "ui/chat/chat_markup_parser.hpp"
#include "ui/chat/chat_markup_renderer.hpp"
#include "ui/chat/chat_command_registry.hpp"
#include <vulkan/vulkan.h>
#include <imgui.h>
#include <string>
#include <unordered_map>
#include <vector>
#include <functional>

namespace wowee {
namespace pipeline { class AssetManager; }
namespace rendering { class Renderer; }
namespace ui {

class InventoryScreen;
class SpellbookScreen;
class QuestLogScreen;

/**
 * Self-contained chat UI panel extracted from GameScreen.
 *
 * Owns all chat state: input buffer, sent-history, tab filtering,
 * slash-command parsing, chat bubbles, and chat-related settings.
 */
class ChatPanel {
public:
    ChatPanel();

    // ---- Main entry points (called by GameScreen) ----

    /**
     * Render the chat window (tabs, history, input, etc.)
     */
    void render(game::GameHandler& gameHandler,
               InventoryScreen& inventoryScreen,
               SpellbookScreen& spellbookScreen,
               QuestLogScreen& questLogScreen);

    /**
     * Render 3D-projected chat bubbles above entities.
     */
    void renderBubbles(game::GameHandler& gameHandler);

    /**
     * Register one-shot callbacks on GameHandler (call once per session).
     * Sets up the chat-bubble callback.
     */
    void setupCallbacks(game::GameHandler& gameHandler);

    // ---- Input helpers (called by GameScreen keybind handling) ----

    [[nodiscard]] bool isChatInputActive() const { return chatInputActive_; }

    /** Insert a spell / item link into the chat input buffer (shift-click). */
    void insertChatLink(const std::string& link);

    /** Activate the input field with a leading '/' (slash key). */
    void activateSlashInput();

    /** Activate (focus) the input field (Enter key). */
    void activateInput();

    /** Request that the chat input be focused next frame. */
    void requestRefocus() { refocusChatInput_ = true; }

    /** Set up a whisper to the given player name and focus input. */
    void setWhisperTarget(const std::string& name);

    /** Execute a macro body (one line per 'click'). */
    void executeMacroText(game::GameHandler& gameHandler,
                          const std::string& macroText);

    /// Every command name this client's own registry answers.
    ///
    /// For bridging them into FrameXML's SlashCmdList. With chat handed over,
    /// the edit box is FrameXML's and ChatEdit_ParseText consults nothing but
    /// that table, so a command living only here cannot be typed at all.
    [[nodiscard]] std::vector<std::string> registryCommandNames() const;

    /// Run one of them, without going near SlashCmdList.
    ///
    /// sendChatMessage tries SlashCmdList first and this registry second, so a
    /// bridge that went back through it would find the entry it had just been
    /// called from and recurse. This is the registry alone.
    bool runRegistryCommand(game::GameHandler& gameHandler,
                            const std::string& alias, const std::string& args);

    // ---- Slash-command side-effects ----
    // GameScreen reads these each frame, then clears them.

    struct SlashCommands {
        bool showInspect   = false;
        bool toggleThreat  = false;
        bool showBgScore   = false;
        bool showGmTicket  = false;
        bool showWho       = false;
        bool toggleCombatLog = false;
        bool takeScreenshot = false;
    };

    /** Return accumulated slash-command flags and reset them. */
    SlashCommands consumeSlashCommands();

    // ---- Chat settings (delegated to ChatSettings) ----

    ChatSettings settings;

    // Legacy accessors - forward to settings struct for external code
    // (GameScreen save/load reads these directly)
    bool& chatShowTimestamps       = settings.showTimestamps;
    int&  chatFontSize             = settings.fontSize;
    bool& chatAutoJoinGeneral      = settings.autoJoinGeneral;
    bool& chatAutoJoinTrade        = settings.autoJoinTrade;
    bool& chatAutoJoinLocalDefense = settings.autoJoinLocalDefense;
    bool& chatAutoJoinLFG          = settings.autoJoinLFG;
    bool& chatAutoJoinLocal        = settings.autoJoinLocal;

    /** Spell icon lookup callback - set by GameScreen each frame before render(). */
    std::function<VkDescriptorSet(uint32_t, pipeline::AssetManager*)> getSpellIcon;

    /** Persist-settings callback - set once by GameScreen so the in-window
     *  quick menu can save appearance changes immediately. */
    std::function<void()> saveSettingsFn;

    /** Render the "Chat" tab inside the Settings window (delegates to settings). */
    void renderSettingsTab(const std::function<void()>& saveSettingsFn) {
        settings.renderSettingsTab(saveSettingsFn);
    }

    /** Reset all chat settings to defaults (delegates to settings). */
    void restoreDefaults() { settings.restoreDefaults(); }

    // UIServices injection (Phase B singleton breaking)
    void setServices(const UIServices& services) { services_ = services; }

    // ---- Accessors for command system (Phase 3) ----
    char* getChatInputBuffer() { return chatInputBuffer_; }
    [[nodiscard]] size_t getChatInputBufferSize() const { return sizeof(chatInputBuffer_); }
    char* getWhisperTargetBuffer() { return whisperTargetBuffer_; }
    [[nodiscard]] size_t getWhisperTargetBufferSize() const { return sizeof(whisperTargetBuffer_); }
    [[nodiscard]] int  getSelectedChatType() const { return selectedChatType_; }
    void setSelectedChatType(int t) { selectedChatType_ = t; }
    [[nodiscard]] int  getSelectedChannelIdx() const { return selectedChannelIdx_; }
    bool& macroStopped() { return macroStopped_; }
    CastSequenceTracker& getCastSeqTracker() { return castSeqTracker_; }
    SlashCommands& getSlashCmds() { return slashCmds_; }
    UIServices& getServices() { return services_; }
    ChatCommandRegistry& getCommandRegistry() { return commandRegistry_; }

private:
    // Injected UI services (Phase B singleton breaking)
    UIServices services_;

    // ---- Chat input state ----
    // These are the input buffers chat_panel*.cpp reads and writes directly.
    // A ChatInput class was written as their eventual home during the Phase-6
    // decomposition and never used by anything; it was removed rather than
    // left as a destination nobody was travelling to, since FrameXML's edit
    // box now owns typing whenever it owns chat at all.
    char chatInputBuffer_[512] = "";
    char whisperTargetBuffer_[256] = "";
    bool chatInputActive_ = false;
    int  chatInputCooldown_ = 0;  // frames to suppress re-activation after send
    int  selectedChatType_ = 0;  // 0=SAY .. 10=CHANNEL
    int  selectedChannelIdx_ = 0;
    bool chatInputMoveCursorToEnd_ = false;
    bool refocusChatInput_ = false;

    // Sent-message history (Up/Down arrow recall)
    std::vector<std::string> chatSentHistory_;
    int chatHistoryIdx_ = -1;

    // ---- History search filter ----
    char chatFilterBuffer_[128] = "";

    // Programmatic tab switch (Ctrl+wheel / quick menu); applied next frame
    // via ImGuiTabItemFlags_SetSelected, -1 = none pending.



    // Macro stop flag
    bool macroStopped_ = false;

    // /castsequence state (delegated to CastSequenceTracker, Phase 1.5)
    CastSequenceTracker castSeqTracker_;

    // Command registry (Phase 3 - replaces if/else chain)
    ChatCommandRegistry commandRegistry_;
    void registerAllCommands();

    // Markup parser + renderer (Phase 2)

    // Per-message render cache. A chat line's formatted text and parsed
    // segments are immutable once built (modulo sender-name resolution and
    // the timestamp toggle), so formatting + markup parsing runs once per
    // message instead of once per message per frame.
    struct CachedChatLine {
        std::string senderNameUsed;  // rebuild when a name query resolves
        bool tsEnabled = false;      // rebuild when timestamp toggle flips
        bool isMention = false;
        std::string fullMsg;         // plain text (for Copy Message)
        std::vector<ChatSegment> segments;
    };


    // Mention notification


    // ---- Chat bubbles (delegated to ChatBubbleManager) ----
    ChatBubbleManager bubbleManager_;
public:
    /// The chatBubbles setting, which the interface options own. Forwarded
    /// rather than exposing the manager: the panel is what the outside holds.
    [[nodiscard]] bool bubblesShown() const { return bubbleManager_.bubblesShown(); }
    void setBubblesShown(bool shown) { bubbleManager_.setBubblesShown(shown); }
private:

    // ---- Helpers ----
    void sendChatMessage(game::GameHandler& gameHandler);

    // Cached game handler for input callback (set each frame in render)
    game::GameHandler* cachedGameHandler_ = nullptr;


    // Slash command flags (accumulated, consumed by GameScreen)
    SlashCommands slashCmds_;
};

} // namespace ui
} // namespace wowee
