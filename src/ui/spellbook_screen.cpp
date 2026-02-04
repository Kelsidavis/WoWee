#include "ui/spellbook_screen.hpp"
#include "core/input.hpp"
#include "core/application.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_loader.hpp"
#include "core/logger.hpp"
#include <algorithm>

namespace wowee { namespace ui {

void SpellbookScreen::loadSpellDBC(pipeline::AssetManager* assetManager) {
    if (dbcLoadAttempted) return;
    dbcLoadAttempted = true;

    if (!assetManager || !assetManager->isInitialized()) return;

    auto dbc = assetManager->loadDBC("Spell.dbc");
    if (!dbc || !dbc->isLoaded()) {
        LOG_WARNING("Spellbook: Could not load Spell.dbc");
        return;
    }

    // WoW 3.3.5a Spell.dbc: field 0 = SpellID, field 136 = SpellName_enUS
    // Validate field count to determine name field index
    uint32_t fieldCount = dbc->getFieldCount();
    uint32_t nameField = 136;

    if (fieldCount < 137) {
        LOG_WARNING("Spellbook: Spell.dbc has ", fieldCount, " fields, expected 234+");
        // Try a heuristic: for smaller DBCs, name might be elsewhere
        if (fieldCount > 10) {
            nameField = fieldCount > 140 ? 136 : 1;
        } else {
            return;
        }
    }

    uint32_t count = dbc->getRecordCount();
    for (uint32_t i = 0; i < count; ++i) {
        uint32_t spellId = dbc->getUInt32(i, 0);
        std::string name = dbc->getString(i, nameField);
        if (!name.empty() && spellId > 0) {
            spellNames[spellId] = name;
        }
    }

    dbcLoaded = true;
    LOG_INFO("Spellbook: Loaded ", spellNames.size(), " spell names from Spell.dbc");
}

std::string SpellbookScreen::getSpellName(uint32_t spellId) const {
    auto it = spellNames.find(spellId);
    if (it != spellNames.end()) {
        return it->second;
    }
    char buf[32];
    snprintf(buf, sizeof(buf), "Spell #%u", spellId);
    return buf;
}

void SpellbookScreen::render(game::GameHandler& gameHandler, pipeline::AssetManager* assetManager) {
    // P key toggle (edge-triggered)
    bool uiWantsKeyboard = ImGui::GetIO().WantCaptureKeyboard;
    bool pDown = !uiWantsKeyboard && core::Input::getInstance().isKeyPressed(SDL_SCANCODE_P);
    if (pDown && !pKeyWasDown) {
        open = !open;
    }
    pKeyWasDown = pDown;

    if (!open) return;

    // Lazy-load Spell.dbc on first open
    if (!dbcLoadAttempted) {
        loadSpellDBC(assetManager);
    }

    auto* window = core::Application::getInstance().getWindow();
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    float bookW = 340.0f;
    float bookH = std::min(500.0f, screenH - 120.0f);
    float bookX = screenW - bookW - 10.0f;
    float bookY = 80.0f;

    ImGui::SetNextWindowPos(ImVec2(bookX, bookY), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(bookW, bookH), ImGuiCond_FirstUseEver);

    bool windowOpen = open;
    if (ImGui::Begin("Spellbook", &windowOpen)) {
        const auto& spells = gameHandler.getKnownSpells();

        if (spells.empty()) {
            ImGui::TextDisabled("No spells known.");
        } else {
            ImGui::Text("%zu spells known", spells.size());
            ImGui::Separator();

            // Action bar assignment mode indicator
            if (assigningSlot >= 0) {
                ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.3f, 1.0f),
                    "Click a spell to assign to slot %d", assigningSlot + 1);
                if (ImGui::SmallButton("Cancel")) {
                    assigningSlot = -1;
                }
                ImGui::Separator();
            }

            // Spell list
            ImGui::BeginChild("SpellList", ImVec2(0, -60), true);

            for (uint32_t spellId : spells) {
                ImGui::PushID(static_cast<int>(spellId));

                std::string name = getSpellName(spellId);
                float cd = gameHandler.getSpellCooldown(spellId);
                bool onCooldown = cd > 0.0f;

                // Color based on state
                if (onCooldown) {
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.5f, 0.5f, 0.5f, 1.0f));
                }

                // Spell entry - clickable
                char label[256];
                if (onCooldown) {
                    snprintf(label, sizeof(label), "%s  (%.1fs)", name.c_str(), cd);
                } else {
                    snprintf(label, sizeof(label), "%s", name.c_str());
                }

                if (ImGui::Selectable(label, false, ImGuiSelectableFlags_AllowDoubleClick)) {
                    if (assigningSlot >= 0) {
                        // Assign to action bar slot
                        gameHandler.setActionBarSlot(assigningSlot,
                            game::ActionBarSlot::SPELL, spellId);
                        assigningSlot = -1;
                    } else if (ImGui::IsMouseDoubleClicked(0)) {
                        // Double-click to cast
                        uint64_t target = gameHandler.hasTarget() ? gameHandler.getTargetGuid() : 0;
                        gameHandler.castSpell(spellId, target);
                    }
                }

                // Tooltip with spell ID
                if (ImGui::IsItemHovered()) {
                    ImGui::BeginTooltip();
                    ImGui::Text("%s", name.c_str());
                    ImGui::TextDisabled("Spell ID: %u", spellId);
                    if (!onCooldown) {
                        ImGui::TextDisabled("Double-click to cast");
                        ImGui::TextDisabled("Use action bar buttons below to assign");
                    }
                    ImGui::EndTooltip();
                }

                if (onCooldown) {
                    ImGui::PopStyleColor();
                }

                ImGui::PopID();
            }

            ImGui::EndChild();

            // Action bar quick-assign buttons
            ImGui::Separator();
            ImGui::Text("Assign to:");
            ImGui::SameLine();
            static const char* slotLabels[] = {"1","2","3","4","5","6","7","8","9","0","-","="};
            for (int i = 0; i < 12; ++i) {
                if (i > 0) ImGui::SameLine(0, 2);
                ImGui::PushID(100 + i);
                bool isAssigning = (assigningSlot == i);
                if (isAssigning) {
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.8f, 0.6f, 0.2f, 1.0f));
                }
                if (ImGui::SmallButton(slotLabels[i])) {
                    assigningSlot = isAssigning ? -1 : i;
                }
                if (isAssigning) {
                    ImGui::PopStyleColor();
                }
                ImGui::PopID();
            }
        }
    }
    ImGui::End();

    if (!windowOpen) {
        open = false;
    }
}

}} // namespace wowee::ui
