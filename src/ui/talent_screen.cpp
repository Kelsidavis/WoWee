#include "ui/talent_screen.hpp"
#include "core/input.hpp"
#include "core/application.hpp"

namespace wowee { namespace ui {

void TalentScreen::render(game::GameHandler& gameHandler) {
    // N key toggle (edge-triggered)
    bool uiWantsKeyboard = ImGui::GetIO().WantCaptureKeyboard;
    bool nDown = !uiWantsKeyboard && core::Input::getInstance().isKeyPressed(SDL_SCANCODE_N);
    if (nDown && !nKeyWasDown) {
        open = !open;
    }
    nKeyWasDown = nDown;

    if (!open) return;

    auto* window = core::Application::getInstance().getWindow();
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    float winW = 400.0f;
    float winH = 450.0f;
    float winX = (screenW - winW) * 0.5f;
    float winY = (screenH - winH) * 0.5f;

    ImGui::SetNextWindowPos(ImVec2(winX, winY), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(winW, winH), ImGuiCond_FirstUseEver);

    bool windowOpen = open;
    if (ImGui::Begin("Talents", &windowOpen)) {
        // Placeholder tabs
        if (ImGui::BeginTabBar("TalentTabs")) {
            if (ImGui::BeginTabItem("Spec 1")) {
                ImGui::Spacing();
                ImGui::TextDisabled("Talents coming soon.");
                ImGui::Spacing();
                ImGui::TextDisabled("Talent trees will be implemented in a future update.");

                uint32_t level = gameHandler.getPlayerLevel();
                uint32_t talentPoints = (level >= 10) ? (level - 9) : 0;
                ImGui::Spacing();
                ImGui::Separator();
                ImGui::Text("Level: %u", level);
                ImGui::Text("Talent points available: %u", talentPoints);

                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Spec 2")) {
                ImGui::TextDisabled("Talents coming soon.");
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Spec 3")) {
                ImGui::TextDisabled("Talents coming soon.");
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
    }
    ImGui::End();

    if (!windowOpen) {
        open = false;
    }
}

}} // namespace wowee::ui
