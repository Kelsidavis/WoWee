#include "ui/chat/chat_settings.hpp"
#include <imgui.h>

namespace wowee { namespace ui {

// Reset all chat settings to defaults.
void ChatSettings::restoreDefaults() {
    showTimestamps       = false;
    fontSize             = 1;
    backgroundAlpha      = 0.6f;
    messageFadeTime      = 20.0f;
    fadeMessages         = true;
    autoJoinGeneral      = true;
    autoJoinTrade        = true;
    autoJoinLocalDefense = true;
    autoJoinLFG          = true;
    autoJoinLocal        = true;
    showBubbles          = true;
    partyBubbles         = false;
}

// Render the "Chat" tab inside the Settings window.
void ChatSettings::renderSettingsTab(const std::function<void()>& saveSettingsFn) {
    ImGui::Spacing();

    ImGui::Text("Appearance");
    ImGui::Separator();

    if (ImGui::Checkbox("Show Timestamps", &showTimestamps)) {
        saveSettingsFn();
    }
    ImGui::SetItemTooltip("Show [HH:MM] before each chat message");

    const char* fontSizes[] = { "Small", "Medium", "Large" };
    if (ImGui::Combo("Chat Font Size", &fontSize, fontSizes, 3)) {
        saveSettingsFn();
    }

    if (ImGui::SliderFloat("Background Opacity", &backgroundAlpha, 0.0f, 1.0f, "%.1f")) {
        saveSettingsFn();
    }
    ImGui::SetItemTooltip("Transparency of the chat window background");

    if (ImGui::Checkbox("Fade Old Messages", &fadeMessages)) {
        saveSettingsFn();
    }
    ImGui::SetItemTooltip("Messages fade out when chat is not hovered");

    if (fadeMessages) {
        ImGui::SameLine();
        ImGui::SetNextItemWidth(100);
        if (ImGui::SliderFloat("##FadeTime", &messageFadeTime, 5.0f, 120.0f, "%.0fs")) {
            saveSettingsFn();
        }
        ImGui::SetItemTooltip("Seconds before messages start fading");
    }

    ImGui::Spacing();
    ImGui::Spacing();
    ImGui::Text("Auto-Join Channels");
    ImGui::Separator();

    if (ImGui::Checkbox("General", &autoJoinGeneral)) saveSettingsFn();
    if (ImGui::Checkbox("Trade", &autoJoinTrade)) saveSettingsFn();
    if (ImGui::Checkbox("LocalDefense", &autoJoinLocalDefense)) saveSettingsFn();
    if (ImGui::Checkbox("LookingForGroup", &autoJoinLFG)) saveSettingsFn();
    if (ImGui::Checkbox("Local", &autoJoinLocal)) saveSettingsFn();

    ImGui::Spacing();
    ImGui::Spacing();
    ImGui::Text("Joined Channels");
    ImGui::Separator();

    ImGui::TextDisabled("Use /join and /leave commands in chat to manage channels.");

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    if (ImGui::Button("Restore Chat Defaults", ImVec2(-1, 0))) {
        restoreDefaults();
        saveSettingsFn();
    }
}

} // namespace ui
} // namespace wowee
