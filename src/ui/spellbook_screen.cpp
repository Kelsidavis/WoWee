#include "ui/spellbook_screen.hpp"
#include "core/input.hpp"
#include "core/application.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_loader.hpp"
#include "pipeline/blp_loader.hpp"
#include "core/logger.hpp"
#include <algorithm>

namespace wowee { namespace ui {

// General utility spells that belong in the General tab
static bool isGeneralSpell(uint32_t spellId) {
    switch (spellId) {
        case 6603:  // Attack
        case 8690:  // Hearthstone
        case 3365:  // Opening
        case 21651: // Opening
        case 21652: // Closing
            return true;
        default:
            return false;
    }
}

void SpellbookScreen::loadSpellDBC(pipeline::AssetManager* assetManager) {
    if (dbcLoadAttempted) return;
    dbcLoadAttempted = true;

    if (!assetManager || !assetManager->isInitialized()) return;

    auto dbc = assetManager->loadDBC("Spell.dbc");
    if (!dbc || !dbc->isLoaded()) {
        LOG_WARNING("Spellbook: Could not load Spell.dbc");
        return;
    }

    uint32_t fieldCount = dbc->getFieldCount();
    if (fieldCount < 142) {
        LOG_WARNING("Spellbook: Spell.dbc has ", fieldCount, " fields, expected 234+");
        return;
    }

    // WoW 3.3.5a Spell.dbc fields:
    // 0 = SpellID, 75 = Attributes, 133 = SpellIconID, 136 = SpellName, 141 = RankText
    uint32_t count = dbc->getRecordCount();
    for (uint32_t i = 0; i < count; ++i) {
        uint32_t spellId = dbc->getUInt32(i, 0);
        if (spellId == 0) continue;

        SpellInfo info;
        info.spellId = spellId;
        info.attributes = dbc->getUInt32(i, 75);
        info.iconId = dbc->getUInt32(i, 133);
        info.name = dbc->getString(i, 136);
        info.rank = dbc->getString(i, 141);

        if (!info.name.empty()) {
            spellData[spellId] = std::move(info);
        }
    }

    dbcLoaded = true;
    LOG_INFO("Spellbook: Loaded ", spellData.size(), " spells from Spell.dbc");
}

void SpellbookScreen::loadSpellIconDBC(pipeline::AssetManager* assetManager) {
    if (iconDbLoaded) return;
    iconDbLoaded = true;

    if (!assetManager || !assetManager->isInitialized()) return;

    auto dbc = assetManager->loadDBC("SpellIcon.dbc");
    if (!dbc || !dbc->isLoaded()) {
        LOG_WARNING("Spellbook: Could not load SpellIcon.dbc");
        return;
    }

    for (uint32_t i = 0; i < dbc->getRecordCount(); i++) {
        uint32_t id = dbc->getUInt32(i, 0);
        std::string path = dbc->getString(i, 1);
        if (!path.empty() && id > 0) {
            spellIconPaths[id] = path;
        }
    }

    LOG_INFO("Spellbook: Loaded ", spellIconPaths.size(), " spell icon paths");
}

void SpellbookScreen::categorizeSpells(const std::vector<uint32_t>& knownSpells) {
    generalSpells.clear();
    activeSpells.clear();
    passiveSpells.clear();

    for (uint32_t spellId : knownSpells) {
        auto it = spellData.find(spellId);
        if (it == spellData.end()) continue;

        const SpellInfo* info = &it->second;
        if (isGeneralSpell(spellId)) {
            generalSpells.push_back(info);
        } else if (info->isPassive()) {
            passiveSpells.push_back(info);
        } else {
            activeSpells.push_back(info);
        }
    }

    // Sort each tab alphabetically
    auto byName = [](const SpellInfo* a, const SpellInfo* b) { return a->name < b->name; };
    std::sort(generalSpells.begin(), generalSpells.end(), byName);
    std::sort(activeSpells.begin(), activeSpells.end(), byName);
    std::sort(passiveSpells.begin(), passiveSpells.end(), byName);

    lastKnownSpellCount = knownSpells.size();
}

GLuint SpellbookScreen::getSpellIcon(uint32_t iconId, pipeline::AssetManager* assetManager) {
    if (iconId == 0 || !assetManager) return 0;

    auto cit = spellIconCache.find(iconId);
    if (cit != spellIconCache.end()) return cit->second;

    auto pit = spellIconPaths.find(iconId);
    if (pit == spellIconPaths.end()) {
        spellIconCache[iconId] = 0;
        return 0;
    }

    std::string iconPath = pit->second + ".blp";
    auto blpData = assetManager->readFile(iconPath);
    if (blpData.empty()) {
        spellIconCache[iconId] = 0;
        return 0;
    }

    auto image = pipeline::BLPLoader::load(blpData);
    if (!image.isValid()) {
        spellIconCache[iconId] = 0;
        return 0;
    }

    GLuint texId = 0;
    glGenTextures(1, &texId);
    glBindTexture(GL_TEXTURE_2D, texId);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, image.width, image.height, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, image.data.data());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);

    spellIconCache[iconId] = texId;
    return texId;
}

const SpellInfo* SpellbookScreen::getSpellInfo(uint32_t spellId) const {
    auto it = spellData.find(spellId);
    return (it != spellData.end()) ? &it->second : nullptr;
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

    // Lazy-load DBC data on first open
    if (!dbcLoadAttempted) {
        loadSpellDBC(assetManager);
        loadSpellIconDBC(assetManager);
    }

    // Rebuild categories if spell list changed
    const auto& spells = gameHandler.getKnownSpells();
    if (spells.size() != lastKnownSpellCount) {
        categorizeSpells(spells);
    }

    auto* window = core::Application::getInstance().getWindow();
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    float bookW = 360.0f;
    float bookH = std::min(520.0f, screenH - 120.0f);
    float bookX = screenW - bookW - 10.0f;
    float bookY = 80.0f;

    ImGui::SetNextWindowPos(ImVec2(bookX, bookY), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(bookW, bookH), ImGuiCond_FirstUseEver);

    bool windowOpen = open;
    if (ImGui::Begin("Spellbook", &windowOpen)) {

        // Tab bar
        if (ImGui::BeginTabBar("SpellbookTabs")) {
            auto renderTab = [&](const char* label, SpellTab tab, const std::vector<const SpellInfo*>& spellList) {
                if (ImGui::BeginTabItem(label)) {
                    currentTab = tab;

                    // Action bar assignment mode indicator
                    if (assigningSlot >= 0) {
                        ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.3f, 1.0f),
                            "Click a spell to assign to slot %d", assigningSlot + 1);
                        if (ImGui::SmallButton("Cancel")) {
                            assigningSlot = -1;
                        }
                        ImGui::Separator();
                    }

                    if (spellList.empty()) {
                        ImGui::TextDisabled("No spells in this category.");
                    }

                    // Spell list with icons
                    ImGui::BeginChild("SpellList", ImVec2(0, -60), true);

                    float iconSize = 32.0f;
                    bool isPassiveTab = (tab == SpellTab::PASSIVE);

                    for (const SpellInfo* info : spellList) {
                        ImGui::PushID(static_cast<int>(info->spellId));

                        float cd = gameHandler.getSpellCooldown(info->spellId);
                        bool onCooldown = cd > 0.0f;

                        // Dimmer for passive or cooldown spells
                        if (isPassiveTab || onCooldown) {
                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.6f, 0.6f, 0.6f, 1.0f));
                        }

                        // Icon
                        GLuint iconTex = getSpellIcon(info->iconId, assetManager);
                        float startY = ImGui::GetCursorPosY();

                        if (iconTex) {
                            ImGui::Image((ImTextureID)(uintptr_t)iconTex,
                                         ImVec2(iconSize, iconSize));
                        } else {
                            // Placeholder colored square
                            ImVec2 pos = ImGui::GetCursorScreenPos();
                            ImGui::GetWindowDrawList()->AddRectFilled(
                                pos, ImVec2(pos.x + iconSize, pos.y + iconSize),
                                IM_COL32(60, 60, 80, 255));
                            ImGui::Dummy(ImVec2(iconSize, iconSize));
                        }

                        ImGui::SameLine();

                        // Name and rank text
                        ImGui::BeginGroup();
                        ImGui::Text("%s", info->name.c_str());
                        if (!info->rank.empty()) {
                            ImGui::TextDisabled("%s", info->rank.c_str());
                        } else if (onCooldown) {
                            ImGui::TextDisabled("%.1fs cooldown", cd);
                        }
                        ImGui::EndGroup();

                        // Make the whole row clickable
                        ImVec2 rowMin = ImVec2(ImGui::GetWindowPos().x,
                                               ImGui::GetWindowPos().y + startY - ImGui::GetScrollY());
                        ImVec2 rowMax = ImVec2(rowMin.x + ImGui::GetContentRegionAvail().x,
                                               rowMin.y + std::max(iconSize, ImGui::GetCursorPosY() - startY));

                        if (ImGui::IsMouseHoveringRect(rowMin, rowMax) && ImGui::IsWindowHovered()) {
                            // Highlight
                            ImGui::GetWindowDrawList()->AddRectFilled(
                                rowMin, rowMax, IM_COL32(255, 255, 255, 20));

                            if (ImGui::IsMouseClicked(0)) {
                                if (assigningSlot >= 0 && !isPassiveTab) {
                                    gameHandler.setActionBarSlot(assigningSlot,
                                        game::ActionBarSlot::SPELL, info->spellId);
                                    assigningSlot = -1;
                                }
                            }

                            if (ImGui::IsMouseDoubleClicked(0) && !isPassiveTab && !onCooldown) {
                                uint64_t target = gameHandler.hasTarget() ? gameHandler.getTargetGuid() : 0;
                                gameHandler.castSpell(info->spellId, target);
                            }

                            // Tooltip
                            ImGui::BeginTooltip();
                            ImGui::Text("%s", info->name.c_str());
                            if (!info->rank.empty()) {
                                ImGui::TextDisabled("%s", info->rank.c_str());
                            }
                            ImGui::TextDisabled("Spell ID: %u", info->spellId);
                            if (isPassiveTab) {
                                ImGui::TextDisabled("Passive");
                            } else {
                                if (!onCooldown) {
                                    ImGui::TextDisabled("Double-click to cast");
                                }
                                ImGui::TextDisabled("Use buttons below to assign to action bar");
                            }
                            ImGui::EndTooltip();
                        }

                        if (isPassiveTab || onCooldown) {
                            ImGui::PopStyleColor();
                        }

                        ImGui::Spacing();
                        ImGui::PopID();
                    }

                    ImGui::EndChild();
                    ImGui::EndTabItem();
                }
            };

            char generalLabel[32], activeLabel[32], passiveLabel[32];
            snprintf(generalLabel, sizeof(generalLabel), "General (%zu)", generalSpells.size());
            snprintf(activeLabel, sizeof(activeLabel), "Active (%zu)", activeSpells.size());
            snprintf(passiveLabel, sizeof(passiveLabel), "Passive (%zu)", passiveSpells.size());

            renderTab(generalLabel, SpellTab::GENERAL, generalSpells);
            renderTab(activeLabel, SpellTab::ACTIVE, activeSpells);
            renderTab(passiveLabel, SpellTab::PASSIVE, passiveSpells);

            ImGui::EndTabBar();
        }

        // Action bar quick-assign buttons (not for passive tab)
        if (currentTab != SpellTab::PASSIVE) {
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
