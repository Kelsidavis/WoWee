#include "ui/first_run_screen.hpp"

#ifdef WOWEE_HAVE_ASSET_PANEL

#include <algorithm>

#include "imgui.h"
#include "core/application.hpp"
#include "ui/paper_ui.hpp"

namespace wowee::ui {

namespace {

/// Wide enough for a path and the Browse button beside it without the field
/// collapsing, and short enough to leave the backdrop visible around it.
/// Both are in the same units the login card is laid out in, and scaled by
/// the same figure below.
constexpr float kCardWidth = 700.0f;
constexpr float kCardMargin = 40.0f;

ImVec4 fromU32(ImU32 colour) { return ImGui::ColorConvertU32ToFloat4(colour); }

/// The figure the login card sizes itself by, so this screen is drawn at the
/// same size as the one it stands in front of.
///
/// ImGui's own layout is in pixels and its built-in face is thirteen of them,
/// which on the display this was written on is about two thirds the height of
/// the card's smallest text - so the panel came out as fine print beside it.
/// A share of the window rather than a count of pixels, for the reason
/// AuthScreen gives where this is copied from.
float screenScale() {
    const ImVec2 screen = ImGui::GetIO().DisplaySize;
    return std::clamp(std::min(screen.x / 1280.0f, screen.y / 760.0f), 0.62f, 2.6f);
}

/// Dress ImGui as the page the rest of the pre-game screens are drawn on.
///
/// The panel is ImGui widgets and the login card is PaperUI's own drawing, so
/// the two cannot share code - but they can share a palette. Without this the
/// builder is a slate-grey box sitting on hand-drawn paper, which reads as a
/// different program that happened to open.
int pushPaperStyle(const PaperTheme& theme) {
    const ImVec4 paper = fromU32(theme.paperTop);
    const ImVec4 edge = fromU32(theme.paperEdge);
    const ImVec4 ink = fromU32(theme.ink);
    const ImVec4 field = fromU32(theme.fieldFill);
    const ImVec4 red = fromU32(theme.crayonRed);

    ImGui::PushStyleColor(ImGuiCol_WindowBg, paper);
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_PopupBg, paper);
    ImGui::PushStyleColor(ImGuiCol_Border, edge);
    ImGui::PushStyleColor(ImGuiCol_Text, ink);
    // inkSoft rather than pencil: pencil is the card's faintest tone, chosen
    // for short hints beside something darker. Whole paragraphs are set in
    // this, and at that length it stops being readable.
    ImGui::PushStyleColor(ImGuiCol_TextDisabled, fromU32(theme.inkSoft));
    ImGui::PushStyleColor(ImGuiCol_TitleBg, fromU32(theme.paperBottom));
    ImGui::PushStyleColor(ImGuiCol_TitleBgActive, fromU32(theme.paperBottom));
    ImGui::PushStyleColor(ImGuiCol_FrameBg, field);
    ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, fromU32(theme.highlighter));
    ImGui::PushStyleColor(ImGuiCol_FrameBgActive, fromU32(theme.highlighter));
    // Tan rather than the crayon red the login card's own button is filled
    // with. That button draws its own light text over the red; ImGui has one
    // text colour for everything in the window, so a red fill here would put
    // dark ink on red and the labels stop being readable.
    ImGui::PushStyleColor(ImGuiCol_Button, fromU32(theme.photoMat));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, fromU32(theme.highlighter));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, fromU32(theme.paperEdge));
    ImGui::PushStyleColor(ImGuiCol_CheckMark, red);
    ImGui::PushStyleColor(ImGuiCol_Header, fromU32(theme.highlighter));
    ImGui::PushStyleColor(ImGuiCol_HeaderHovered, fromU32(theme.highlighter));
    ImGui::PushStyleColor(ImGuiCol_HeaderActive, fromU32(theme.highlighter));
    ImGui::PushStyleColor(ImGuiCol_Separator, edge);
    ImGui::PushStyleColor(ImGuiCol_ScrollbarBg, ImVec4(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_ScrollbarGrab, edge);
    ImGui::PushStyleColor(ImGuiCol_ScrollbarGrabHovered, fromU32(theme.inkSoft));
    ImGui::PushStyleColor(ImGuiCol_ScrollbarGrabActive, fromU32(theme.inkSoft));
    return 23;
}

}  // namespace

FirstRunScreen::FirstRunScreen() {
    assets::initPanelDefaults(app_);
    // The crayons, so the panel's own three accents are drawn in the same
    // hand as everything around them. Its defaults are for a dark window.
    const PaperTheme theme;
    app_.goodColor = fromU32(theme.crayonGreen);
    app_.warnColor = fromU32(theme.crayonRedDim);
    app_.errorColor = fromU32(theme.crayonRed);
}

bool FirstRunScreen::render() {
    const PaperTheme theme;
    const float scale = screenScale();
    const int pushed = pushPaperStyle(theme);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 1.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 2.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(22.0f * scale, 20.0f * scale));
    // Room to breathe, at the same scale. ImGui's defaults are tight for a
    // form somebody is reading rather than a debug panel.
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f * scale, 7.0f * scale));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(7.0f * scale, 5.0f * scale));

    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    const float margin = kCardMargin * scale;
    const float width = std::min(kCardWidth * scale, viewport->WorkSize.x - margin * 2.0f);
    const float height = viewport->WorkSize.y - margin * 2.0f;

    ImGui::SetNextWindowPos(
        ImVec2(viewport->WorkPos.x + (viewport->WorkSize.x - width) * 0.5f,
               viewport->WorkPos.y + margin));
    ImGui::SetNextWindowSize(ImVec2(width, height));
    // Two ways in want two titles, and "###" keeps one window identity
    // behind them so the scroll position does not reset when it changes.
    const bool alreadyHaveAssets =
        core::Application::getInstance().getAssetInventory().anyUsable();
    ImGui::Begin(alreadyHaveAssets ? "Game assets###firstrun"
                                   : "Before you can play###firstrun", nullptr,
                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                 ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBringToFrontOnFocus);
    // Text and everything ImGui sizes from it - field heights, button
    // heights, the log pane - come up together.
    ImGui::SetWindowFontScale(scale);

    // Two ways in, and they need different sentences. A first run arrives
    // here because there is nothing to play with; somebody who came from the
    // login screen already has a game and wants another, and telling them
    // they have no assets would be wrong.
    if (alreadyHaveAssets) {
        // And a way back, which a first run neither needs nor should have:
        // there is nowhere for it to go.
        if (ImGui::Button("Back to login")) {
            core::Application::getInstance().setState(core::AppState::AUTHENTICATION);
        }
        ImGui::Spacing();
        ImGui::TextWrapped(
            "Build another game into the same folder, or rebuild what is already there. "
            "The client offers everything it finds at the login screen. Nothing is "
            "written into the installation you build from.");
    } else {
        ImGui::TextWrapped(
            "WoWee has no game assets yet, so there is nothing to log in to. Point it at a "
            "World of Warcraft installation you own and it will build what it needs. "
            "Nothing is written into that installation.");
    }
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    assets::drawPanel(app_);

    const bool done = assets::finishedSuccessfully(app_);
    if (done) {
        ImGui::Spacing();
        // Restart rather than carrying on into the game from here. The asset
        // manager, the DBC tables, the model and terrain loaders and the
        // addon environment are all built once during startup from a data
        // path that was empty at the time, and the interface's glyph atlas
        // cannot be rebuilt mid-session either. Reopening is one line to say
        // and nothing to go wrong; re-running all of that in place is not.
        ImGui::TextWrapped("Assets built. Close WoWee and open it again to use them.");
    }

    ImGui::End();
    ImGui::PopStyleVar(5);
    ImGui::PopStyleColor(pushed);
    return done;
}

}  // namespace wowee::ui

#endif  // WOWEE_HAVE_ASSET_PANEL
