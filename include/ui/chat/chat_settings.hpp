#pragma once

#include <functional>

// Forward declaration for ImGui (avoid pulling full imgui header)
struct ImGuiContext;

namespace wowee {
namespace ui {

/**
 * Chat appearance and auto-join settings.
 *
 * Extracted from ChatPanel (Phase 1.1 of chat_panel_ref.md).
 * Pure data + settings UI; no dependency on GameHandler or network.
 */
struct ChatSettings {
    // Appearance
    bool showTimestamps  = false;
    int  fontSize        = 1;   // 0=small, 1=medium, 2=large
    float backgroundAlpha = 0.6f;  // 0.0=transparent, 1.0=opaque
    float messageFadeTime = 20.0f; // seconds before messages fade (0=never)
    bool  fadeMessages    = true;  // enable message fade-out when not hovering

    // Auto-join channels
    bool autoJoinGeneral      = true;
    bool autoJoinTrade        = true;
    bool autoJoinLocalDefense = true;
    bool autoJoinLFG          = true;
    bool autoJoinLocal        = true;

    // No window state here. This client drew its own chat window once and kept
    // three fields for it - whether it was locked, and the size a resize left
    // it. FrameXML draws chat now, chat_panel.cpp sets no window size at all,
    // and nothing had read any of the three for a long time: the width and
    // height were written to the config every run and read back into fields
    // nobody asked. The comment on them described capturing a resize, which
    // nothing did.

    /** Reset all chat settings to defaults. */
    void restoreDefaults();

    /** Render the "Chat" tab inside the Settings window. */
    void renderSettingsTab(std::function<void()> saveSettingsFn);
};

} // namespace ui
} // namespace wowee
