#include "ui/realm_screen.hpp"
#include "ui/ui_colors.hpp"

#include <algorithm>
#include <cstdio>
#include <imgui.h>

namespace wowee { namespace ui {

namespace {

// Written in units and drawn at a scale taken from the window, the same way
// the login card is. See paper_ui.hpp.
constexpr float kSheetWidth  = 860.0f;
constexpr float kSheetHeight = 560.0f;
constexpr float kPad         = 34.0f;
constexpr float kTitleSize   = 40.0f;
constexpr float kLabelSize   = 14.0f;
constexpr float kBodySize    = 15.5f;
constexpr float kSmallSize   = 13.0f;
constexpr float kRowHeight   = 34.0f;
constexpr float kButtonH     = 44.0f;

} // namespace

RealmScreen::RealmScreen() {
}

const char* RealmScreen::getRealmType(uint8_t icon) {
    switch (icon) {
        case 0: return "Normal";
        case 1: return "PvP";
        case 6: return "RP";
        case 8: return "RP-PvP";
        default: return "Other";
    }
}

const char* RealmScreen::getPopulationName(float population) {
    if (population < 0.5f) return "Low";
    if (population < 1.5f) return "Medium";
    if (population < 2.5f) return "High";
    return "Full";
}

void RealmScreen::render(auth::AuthHandler& authHandler) {
    const ImVec2 screen = ImGui::GetIO().DisplaySize;
    const float scale = std::clamp(std::min(screen.x / 1280.0f, screen.y / 760.0f), 0.62f, 2.6f);
    ui_.begin(ImGui::GetIO().DeltaTime, scale);
    ui_.setLayer(PaperLayer::Page);

    const PaperTheme& theme = ui_.theme();
    const auto px = [this](float units) { return ui_.px(units); };

    const auto& realms = authHandler.getRealms();

    // Auto-select: prefer realm with characters, then single realm, then first available
    if (!realms.empty() && !autoSelectAttempted && !realmSelected) {
        autoSelectAttempted = true;

        int bestRealm = -1;
        for (size_t i = 0; i < realms.size(); ++i) {
            if (!realms[i].lock && realms[i].characters > 0) {
                bestRealm = static_cast<int>(i);
                break;
            }
        }

        if (realms.size() == 1 && !realms[0].lock) {
            selectedRealmIndex = 0;
            realmSelected = true;
            selectedRealmName = realms[0].name;
            selectedRealmAddress = realms[0].address;
            setStatus("Auto-selecting realm: " + realms[0].name);
            if (onRealmSelected) {
                onRealmSelected(selectedRealmName, selectedRealmAddress);
            }
        } else if (bestRealm >= 0) {
            // Pre-highlight realm with characters (don't auto-connect, let user confirm)
            selectedRealmIndex = bestRealm;
        }
    }

    const float sheetW = std::min(px(kSheetWidth), screen.x - px(40));
    const float sheetH = std::min(px(kSheetHeight), screen.y - px(40));
    const ImVec2 a((screen.x - sheetW) * 0.5f, (screen.y - sheetH) * 0.5f);
    const ImVec2 b(a.x + sheetW, a.y + sheetH);
    ui_.sheet(a, b);
    ui_.claimMouse(a, b);

    const float titleSize = px(kTitleSize);
    const float bodySize = px(kBodySize);
    const float smallSize = px(kSmallSize);
    const float labelSize = px(kLabelSize);

    Column col{a.x + px(kPad), b.x - px(kPad), a.y + px(kPad)};

    // ---- heading ----------------------------------------------------------
    ui_.text(col.at(), "Choose a Realm", titleSize, theme.ink, /*titleFace=*/true);
    {
        const float w = ui_.textWidth("Choose a Realm", titleSize, true);
        const float y = col.y + ui_.inkHeight(titleSize, true);
        ui_.squiggle(ImVec2(col.x0, y), ImVec2(col.x0 + w * 1.04f, y), theme.crayonRed,
                     px(2.0f), 0x3311u);
    }
    ui_.textRight(col.x1, col.y + titleSize * 0.55f, "Where your characters live.", smallSize,
                  theme.pencil);
    col.gap(ui_.inkHeight(titleSize, true) + px(18));

    if (!statusMessage.empty()) {
        const float h = ui_.wrappedHeight(col.width(), statusMessage.c_str(), smallSize);
        ui_.wrapped(col.at(), col.width(), statusMessage.c_str(), smallSize, theme.crayonGreen);
        col.gap(h + px(6));
    }

    // ---- the list ---------------------------------------------------------
    // Everything below it is a fixed height, so what is left over is the list.
    const float footerH = px(kButtonH) + ui_.lineHeight(smallSize) + px(20);
    const float headerH = ui_.lineHeight(labelSize) + px(6);
    const float listTop = col.y + headerH;
    const float listBottom = b.y - px(kPad) - footerH;

    // Five columns across the sheet's width: the name takes what the other
    // four do not.
    const float typeX = col.x0 + col.width() * 0.44f;
    const float popX = col.x0 + col.width() * 0.58f;
    const float charX = col.x0 + col.width() * 0.74f;
    const float statusX = col.x0 + col.width() * 0.86f;

    ui_.text(ImVec2(col.x0 + px(10), col.y), "Realm", labelSize, theme.inkSoft);
    ui_.text(ImVec2(typeX, col.y), "Type", labelSize, theme.inkSoft);
    ui_.text(ImVec2(popX, col.y), "Population", labelSize, theme.inkSoft);
    ui_.text(ImVec2(charX, col.y), "Yours", labelSize, theme.inkSoft);
    ui_.text(ImVec2(statusX, col.y), "Status", labelSize, theme.inkSoft);
    ui_.rule(ImVec2(col.x0, listTop - px(4)), ImVec2(col.x1, listTop - px(4)),
             paperFade(theme.ink, 0.5f), px(1.3f), 0x8842u);

    const auto enterRealm = [&](const auth::Realm& realm) {
        realmSelected = true;
        selectedRealmName = realm.name;
        selectedRealmAddress = realm.address;
        setStatus("Connecting to realm: " + realm.name);
        if (onRealmSelected) onRealmSelected(selectedRealmName, selectedRealmAddress);
    };

    if (realms.empty()) {
        ui_.text(ImVec2(col.x0 + px(10), listTop + px(14)),
                 "No realms yet. Asking the server for the list...", bodySize, theme.pencil);
        authHandler.requestRealmList();
    } else {
        const PaperUI::ListResult picked = ui_.list(
            "realms", ImVec2(col.x0, listTop), ImVec2(col.x1, listBottom),
            static_cast<int>(realms.size()), px(kRowHeight), selectedRealmIndex,
            [&](int index, ImVec2 rowA, ImVec2 rowB, bool selected, bool) {
                const auto& realm = realms[static_cast<size_t>(index)];
                const float y = rowA.y + ((rowB.y - rowA.y) - bodySize) * 0.5f - px(1);
                const float smallY = rowA.y + ((rowB.y - rowA.y) - smallSize) * 0.5f - px(1);
                const ImU32 nameCol = realm.lock ? theme.pencil
                                                 : (selected ? theme.ink : theme.inkSoft);

                ui_.text(ImVec2(rowA.x + px(10), y), realm.name.c_str(), bodySize, nameCol);
                ui_.text(ImVec2(typeX, smallY), getRealmType(realm.icon), smallSize,
                         theme.pencil);
                ui_.text(ImVec2(popX, smallY), getPopulationName(realm.population), smallSize,
                         ui_.onPaper(getPopulationColor(realm.population)));

                if (realm.characters > 0) {
                    char count[16];
                    std::snprintf(count, sizeof(count), "%d", realm.characters);
                    ui_.text(ImVec2(charX, smallY), count, smallSize, theme.crayonBlue);
                } else {
                    ui_.text(ImVec2(charX, smallY), "-", smallSize, theme.pencil);
                }

                if (realm.lock) {
                    ui_.text(ImVec2(statusX, smallY), "Locked", smallSize, theme.crayonRed);
                } else {
                    ui_.text(ImVec2(statusX, smallY), getRealmStatus(realm.flags), smallSize,
                             theme.crayonGreen);
                }
            });

        if (picked.clicked >= 0) selectedRealmIndex = picked.clicked;
        if (picked.activated >= 0 &&
            !realms[static_cast<size_t>(picked.activated)].lock) {
            selectedRealmIndex = picked.activated;
            enterRealm(realms[static_cast<size_t>(picked.activated)]);
        }
    }

    // ---- footer -----------------------------------------------------------
    col.y = listBottom + px(10);
    ui_.rule(ImVec2(col.x0, col.y), ImVec2(col.x1, col.y), paperFade(theme.pencil, 0.55f),
             px(1.0f), 0x1907u);
    col.gap(px(8));

    const bool haveSelection = selectedRealmIndex >= 0 &&
                               selectedRealmIndex < static_cast<int>(realms.size());
    if (haveSelection) {
        const auto& realm = realms[static_cast<size_t>(selectedRealmIndex)];
        std::string line = realm.name + "   " + realm.address;
        if (realm.characters > 0) {
            char suffix[64];
            std::snprintf(suffix, sizeof(suffix), "   %d character%s here", realm.characters,
                          realm.characters > 1 ? "s" : "");
            line += suffix;
        }
        if (realm.hasVersionInfo() && (realm.majorVersion || realm.build)) {
            char version[64];
            std::snprintf(version, sizeof(version), "   v%d.%d.%d build %d", realm.majorVersion,
                          realm.minorVersion, realm.patchVersion, realm.build);
            line += version;
        }
        ui_.text(col.at(), line.c_str(), smallSize, theme.inkSoft);
    } else {
        ui_.text(col.at(), "Pick a realm, or double-click one to go straight in.", smallSize,
                 theme.pencil);
    }
    col.gap(ui_.lineHeight(smallSize) + px(8));

    {
        const float y = col.y;
        const float h = px(kButtonH);
        const float w = px(110);
        if (ui_.button("back", ImVec2(col.x0, y), ImVec2(col.x0 + w, y + h), "Back")) {
            if (onBack) onBack();
        }
        if (ui_.button("refresh", ImVec2(col.x0 + w + px(12), y),
                       ImVec2(col.x0 + w * 2.0f + px(12), y + h), "Refresh")) {
            authHandler.requestRealmList();
            setStatus("Refreshing realm list...");
        }

        if (haveSelection) {
            const auto& realm = realms[static_cast<size_t>(selectedRealmIndex)];
            const float enterW = px(190);
            if (ui_.button("enter", ImVec2(col.x1 - enterW, y), ImVec2(col.x1, y + h),
                           realm.lock ? "Realm Locked" : "Enter Realm",
                           PaperUI::ButtonKind::Primary, !realm.lock)) {
                enterRealm(realm);
            }
        }
    }

    ui_.end();
}

void RealmScreen::setStatus(const std::string& message) {
    statusMessage = message;
}

const char* RealmScreen::getRealmStatus(uint8_t flags) const {
    if (flags & 0x02) return "Offline";
    if (flags & 0x01) return "Invalid";
    if (flags & 0x80) return "Full";
    if (flags & 0x40) return "New";
    if (flags & 0x20) return "Recommended";
    return "Online";
}

ImVec4 RealmScreen::getPopulationColor(float population) const {
    if (population < 0.5f) {
        return ui::colors::kBrightGreen;  // Green - Low
    }
    if (population < 1.5f) {
        return ui::colors::kYellow;  // Yellow - Medium
    }
    if (population < 2.5f) {
        return ImVec4(1.0f, 0.6f, 0.0f, 1.0f);  // Orange - High
    }
    return ui::colors::kRed;  // Red - Full
}

}} // namespace wowee::ui
