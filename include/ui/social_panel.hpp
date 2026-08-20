#pragma once

#include "ui/ui_services.hpp"
#include "ui/chat/chat_markup_renderer.hpp"
#include <imgui.h>
#include <vulkan/vulkan.h>
#include <string>
#include <vector>
#include <cstdint>
#include <functional>

namespace wowee {
namespace game { class GameHandler; }
namespace pipeline { class AssetManager; }
namespace ui {

class ChatPanel;
class SpellbookScreen;
class InventoryScreen;

/**
 * Social panel manager (extracted from GameScreen)
 *
 * Owns all social/group-related rendering:
 *   party frames, boss frames, guild roster, social/friends frame,
 *   dungeon finder, who window, inspect window.
 */
class SocialPanel {
public:
    SocialPanel() = default;

    // ---- Callback type for spell icon lookup (stays in GameScreen) ----
    using SpellIconFn = std::function<VkDescriptorSet(uint32_t spellId, pipeline::AssetManager*)>;

    // ---- Toggle booleans (written by slash commands / escape handler / keybinds / UI buttons) ----
    /// Put the inspect window up, whichever interface draws it.
    ///
    /// Seven places set the flag above directly and the window's render is
    /// gated on FrameXML not owning it, so with that element handed over
    /// inspecting sent the request and showed nothing. The request still goes
    /// out from the caller either way - this is only the window.
    void openInspectWindow(game::GameHandler& gameHandler);

    // ---- Guild roster state ----
    char guildNoteEditBuffer_[256] = {0};
    char guildMotdEditBuffer_[256] = {0};
    char petitionNameBuffer_[64] = {0};
    char addRankNameBuffer_[64] = {0};

    // ---- LFG state ----

    // ---- Public render methods ----
    // Takes the panels the markup renderer needs: the guild info text and the
    // MOTD are server strings that carry item, spell, quest and achievement
    // links, and rendering them as plain text left every link inert.

    // UIServices injection (singleton breaking)
    void setServices(const UIServices& services) { services_ = services; }

private:
    UIServices services_;
    // Shared with chat: the guild info text and MOTD carry the same markup.
};

} // namespace ui
} // namespace wowee
