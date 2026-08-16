#include "ui/realm_screen.hpp"
#include "ui/selection_screen_layout.hpp"
#include "ui/ui_colors.hpp"
#include <imgui.h>

namespace wowee { namespace ui {

RealmScreen::RealmScreen() {
}

void RealmScreen::render(auth::AuthHandler& authHandler) {
    ImGuiViewport* vp = ImGui::GetMainViewport();
    const SelectionScreenLayout layout = makeSelectionScreenLayout(*vp);

    ImGui::SetNextWindowPos(layout.windowPos, ImGuiCond_Always);
    ImGui::SetNextWindowSize(layout.windowSize, ImGuiCond_Always);
    ImGui::Begin("Realm Selection", nullptr,
                 ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);
    ImGui::SetWindowFontScale(layout.scale);

    ImGui::TextColored(ui::colors::kWarmGold, "Select a Realm");
    ImGui::TextDisabled("Choose where your characters live.");
    ImGui::Separator();

    // Status message
    if (!statusMessage.empty()) {
        ImGui::PushStyleColor(ImGuiCol_Text, ui::colors::kBrightGreen);
        ImGui::TextWrapped("%s", statusMessage.c_str());
        ImGui::PopStyleColor();
        ImGui::Spacing();
    }

    // Get realm list
    const auto& realms = authHandler.getRealms();
    const float bodyHeight = std::max(
        200.0f,
        ImGui::GetContentRegionAvail().y - layout.footerHeight() -
            ImGui::GetStyle().ItemSpacing.y);

    if (realms.empty()) {
        ImGui::BeginChild("RealmListEmpty", ImVec2(0.0f, bodyHeight), true);
        ImGui::Text("No realms available. Requesting realm list...");
        authHandler.requestRealmList();
        ImGui::EndChild();
    } else {
        // Auto-select: prefer realm with characters, then single realm, then first available
        if (!autoSelectAttempted && !realmSelected) {
            autoSelectAttempted = true;

            // First: look for realm with characters
            int bestRealm = -1;
            for (size_t i = 0; i < realms.size(); ++i) {
                if (!realms[i].lock && realms[i].characters > 0) {
                    bestRealm = static_cast<int>(i);
                    break;
                }
            }

            // If only one realm and it's unlocked, auto-connect
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

        float rowHeight = std::max(28.0f * layout.scale,
                                   ImGui::GetTextLineHeight() + 12.0f * layout.scale);

        // The body consumes all flexible space; the shared footer remains at a
        // stable screen-relative position regardless of realm count or status.
        // Realm table - fills available width and height
        ImGui::PushStyleVar(ImGuiStyleVar_CellPadding,
                            ImVec2(12.0f * layout.scale, 6.0f * layout.scale));
        if (ImGui::BeginTable("RealmsTable", 5,
                ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                ImGuiTableFlags_ScrollY | ImGuiTableFlags_Resizable,
                ImVec2(0, bodyHeight))) {

            // Proportional columns
            float totalW = ImGui::GetContentRegionAvail().x;
            ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed, totalW * 0.12f);
            ImGui::TableSetupColumn("Population", ImGuiTableColumnFlags_WidthFixed, totalW * 0.14f);
            ImGui::TableSetupColumn("Characters", ImGuiTableColumnFlags_WidthFixed, totalW * 0.12f);
            ImGui::TableSetupColumn("Status", ImGuiTableColumnFlags_WidthFixed, totalW * 0.12f);
            ImGui::TableHeadersRow();

            for (size_t i = 0; i < realms.size(); ++i) {
                const auto& realm = realms[i];

                ImGui::TableNextRow(0, rowHeight);

                // Name column (selectable, double-click to enter)
                ImGui::TableSetColumnIndex(0);
                bool isSelected = (selectedRealmIndex == static_cast<int>(i));
                char nameLabel[256];
                snprintf(nameLabel, sizeof(nameLabel), "%s##realm%zu", realm.name.c_str(), i);
                if (ImGui::Selectable(nameLabel, isSelected,
                        ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowDoubleClick,
                        ImVec2(0, rowHeight - 8.0f))) {
                    selectedRealmIndex = static_cast<int>(i);
                    if (ImGui::IsMouseDoubleClicked(0) && !realm.lock) {
                        realmSelected = true;
                        selectedRealmName = realm.name;
                        selectedRealmAddress = realm.address;
                        setStatus("Connecting to realm: " + realm.name);
                        if (onRealmSelected) {
                            onRealmSelected(selectedRealmName, selectedRealmAddress);
                        }
                    }
                }

                // Type column
                ImGui::TableSetColumnIndex(1);
                if (realm.icon == 0) ImGui::Text("Normal");
                else if (realm.icon == 1) ImGui::Text("PvP");
                else if (realm.icon == 6) ImGui::Text("RP");
                else if (realm.icon == 8) ImGui::Text("RP-PvP");
                else ImGui::Text("Type %d", realm.icon);

                // Population column
                ImGui::TableSetColumnIndex(2);
                ImVec4 popColor = getPopulationColor(realm.population);
                ImGui::PushStyleColor(ImGuiCol_Text, popColor);
                if (realm.population < 0.5f) ImGui::Text("Low");
                else if (realm.population < 1.5f) ImGui::Text("Medium");
                else if (realm.population < 2.5f) ImGui::Text("High");
                else ImGui::Text("Full");
                ImGui::PopStyleColor();

                // Characters column
                ImGui::TableSetColumnIndex(3);
                if (realm.characters > 0) {
                    ImGui::TextColored(ImVec4(0.4f, 0.9f, 1.0f, 1.0f), "%d", realm.characters);
                } else {
                    ImGui::TextDisabled("0");
                }

                // Status column
                ImGui::TableSetColumnIndex(4);
                const char* status = getRealmStatus(realm.flags);
                if (realm.lock) {
                    ImGui::TextColored(ui::colors::kRed, "Locked");
                } else {
                    ImGui::TextColored(ui::colors::kBrightGreen, "%s", status);
                }
            }

            ImGui::EndTable();
        }
        ImGui::PopStyleVar(); // CellPadding
    }

    ImGui::BeginChild("RealmFooter", ImVec2(0.0f, layout.footerHeight()), true);

    // Stable footer: navigation left, primary action right.
    if (selectedRealmIndex >= 0 && selectedRealmIndex < static_cast<int>(realms.size())) {
        const auto& realm = realms[selectedRealmIndex];

        ImGui::Text("Selected: %s", realm.name.c_str());
        ImGui::SameLine();
        ImGui::TextDisabled("(%s)", realm.address.c_str());
        if (realm.characters > 0) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.4f, 0.9f, 1.0f, 1.0f),
                " - %d character%s", realm.characters, realm.characters > 1 ? "s" : "");
        }
        if (realm.hasVersionInfo() && (realm.majorVersion || realm.build)) {
            ImGui::SameLine();
            ImGui::TextDisabled(" v%d.%d.%d (build %d)",
                realm.majorVersion, realm.minorVersion, realm.patchVersion, realm.build);
        }

    } else {
        ImGui::TextDisabled("Click a realm to select it, or double-click to enter.");
    }

    ImGui::Spacing();
    if (ImGui::Button("Back", layout.button())) {
        if (onBack) onBack();
    }
    ImGui::SameLine(0.0f, layout.gap());
    if (ImGui::Button("Refresh", layout.button())) {
        authHandler.requestRealmList();
        setStatus("Refreshing realm list...");
    }

    if (selectedRealmIndex >= 0 && selectedRealmIndex < static_cast<int>(realms.size())) {
        const auto& realm = realms[selectedRealmIndex];
        const ImVec2 primarySize = layout.primaryButton(200.0f);
        ImGui::SameLine(ImGui::GetContentRegionMax().x - primarySize.x);
        if (!realm.lock) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.45f, 0.15f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.2f, 0.6f, 0.2f, 1.0f));
            if (ImGui::Button("Enter Realm", primarySize)) {
                realmSelected = true;
                selectedRealmName = realm.name;
                selectedRealmAddress = realm.address;
                setStatus("Connecting to realm: " + realm.name);
                if (onRealmSelected) onRealmSelected(selectedRealmName, selectedRealmAddress);
            }
            ImGui::PopStyleColor(2);
        } else {
            ImGui::BeginDisabled();
            ImGui::Button("Realm Locked", primarySize);
            ImGui::EndDisabled();
        }
    }
    ImGui::EndChild();

    ImGui::End();
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
