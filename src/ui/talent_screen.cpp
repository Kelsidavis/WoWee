#include "ui/talent_screen.hpp"
#include "core/input.hpp"
#include "core/application.hpp"
#include "core/logger.hpp"
#include "rendering/vk_context.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/blp_loader.hpp"
#include "pipeline/dbc_layout.hpp"
#include <algorithm>

namespace wowee { namespace ui {

void TalentScreen::render(game::GameHandler& gameHandler) {
    // N key toggle (edge-triggered)
    bool wantsTextInput = ImGui::GetIO().WantTextInput;
    bool nDown = !wantsTextInput && core::Input::getInstance().isKeyPressed(SDL_SCANCODE_N);
    if (nDown && !nKeyWasDown) {
        open = !open;
    }
    nKeyWasDown = nDown;

    if (!open) return;

    auto* window = core::Application::getInstance().getWindow();
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    float winW = 600.0f;  // Wider for talent grid
    float winH = 550.0f;
    float winX = (screenW - winW) * 0.5f;
    float winY = (screenH - winH) * 0.5f;

    ImGui::SetNextWindowPos(ImVec2(winX, winY), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(winW, winH), ImGuiCond_FirstUseEver);

    bool windowOpen = open;
    if (ImGui::Begin("Talents", &windowOpen)) {
        renderTalentTrees(gameHandler);
    }
    ImGui::End();

    if (!windowOpen) {
        open = false;
    }
}

void TalentScreen::renderTalentTrees(game::GameHandler& gameHandler) {
    auto* assetManager = core::Application::getInstance().getAssetManager();

    // Ensure talent DBCs are loaded (even if server hasn't sent SMSG_TALENTS_INFO)
    static bool dbcLoadAttempted = false;
    if (!dbcLoadAttempted) {
        dbcLoadAttempted = true;
        gameHandler.loadTalentDbc();
        loadSpellDBC(assetManager);
        loadSpellIconDBC(assetManager);
        LOG_INFO("Talent window opened, DBC load triggered");
    }

    uint8_t playerClass = gameHandler.getPlayerClass();
    LOG_INFO("Talent window: playerClass=", static_cast<int>(playerClass));

    // Active spec indicator and switcher
    uint8_t activeSpec = gameHandler.getActiveTalentSpec();
    ImGui::Text("Active Spec: %u", activeSpec + 1);
    ImGui::SameLine();

    // Spec buttons
    if (ImGui::SmallButton("Spec 1")) {
        gameHandler.switchTalentSpec(0);
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Spec 2")) {
        gameHandler.switchTalentSpec(1);
    }
    ImGui::SameLine();

    // Show unspent points for both specs
    ImGui::Text("| Unspent: Spec1=%u Spec2=%u",
                gameHandler.getUnspentTalentPoints(0),
                gameHandler.getUnspentTalentPoints(1));

    ImGui::Separator();

    // Debug info
    ImGui::Text("Player Class: %u", playerClass);
    ImGui::Text("Total Talent Tabs: %zu", gameHandler.getAllTalentTabs().size());
    ImGui::Text("Total Talents: %zu", gameHandler.getAllTalents().size());
    ImGui::Separator();

    if (playerClass == 0) {
        ImGui::TextDisabled("Class information not available.");
        LOG_WARNING("Talent window: getPlayerClass() returned 0");
        return;
    }

    // Get talent tabs for this class (class mask: 1 << (class - 1))
    uint32_t classMask = 1u << (playerClass - 1);
    LOG_INFO("Talent window: classMask=0x", std::hex, classMask, std::dec);

    // Collect talent tabs for this class, sorted by orderIndex
    std::vector<const game::GameHandler::TalentTabEntry*> classTabs;
    for (const auto& [tabId, tab] : gameHandler.getAllTalentTabs()) {
        if (tab.classMask & classMask) {
            classTabs.push_back(&tab);
        }
    }

    std::sort(classTabs.begin(), classTabs.end(),
        [](const auto* a, const auto* b) { return a->orderIndex < b->orderIndex; });

    LOG_INFO("Talent window: found ", classTabs.size(), " tabs for class mask 0x", std::hex, classMask, std::dec);

    ImGui::Text("Class Mask: 0x%X", classMask);
    ImGui::Text("Tabs for this class: %zu", classTabs.size());

    if (classTabs.empty()) {
        ImGui::TextDisabled("No talent trees available for your class.");
        ImGui::Spacing();
        ImGui::TextDisabled("Available tabs:");
        for (const auto& [tabId, tab] : gameHandler.getAllTalentTabs()) {
            ImGui::Text("  Tab %u: %s (mask: 0x%X)", tabId, tab.name.c_str(), tab.classMask);
        }
        return;
    }

    // Display points
    uint8_t unspentPoints = gameHandler.getUnspentTalentPoints();
    ImGui::Text("Unspent Points: %u", unspentPoints);
    ImGui::Separator();

    // Render tabs
    if (ImGui::BeginTabBar("TalentTabs")) {
        for (const auto* tab : classTabs) {
            if (ImGui::BeginTabItem(tab->name.c_str())) {
                renderTalentTree(gameHandler, tab->tabId);
                ImGui::EndTabItem();
            }
        }
        ImGui::EndTabBar();
    }
}

void TalentScreen::renderTalentTree(game::GameHandler& gameHandler, uint32_t tabId) {
    // Collect all talents for this tab
    std::vector<const game::GameHandler::TalentEntry*> talents;
    for (const auto& [talentId, talent] : gameHandler.getAllTalents()) {
        if (talent.tabId == tabId) {
            talents.push_back(&talent);
        }
    }

    if (talents.empty()) {
        ImGui::TextDisabled("No talents in this tree.");
        return;
    }

    // Find grid dimensions
    uint8_t maxRow = 0, maxCol = 0;
    for (const auto* talent : talents) {
        maxRow = std::max(maxRow, talent->row);
        maxCol = std::max(maxCol, talent->column);
    }

    const float iconSize = 40.0f;

    ImGui::BeginChild("TalentGrid", ImVec2(0, 0), false);

    // Render grid
    for (uint8_t row = 0; row <= maxRow; ++row) {
        // Row label
        ImGui::Text("Tier %u", row);
        ImGui::SameLine(80);

        for (uint8_t col = 0; col <= maxCol; ++col) {
            // Find talent at this position
            const game::GameHandler::TalentEntry* talent = nullptr;
            for (const auto* t : talents) {
                if (t->row == row && t->column == col) {
                    talent = t;
                    break;
                }
            }

            if (col > 0) ImGui::SameLine();

            if (talent) {
                renderTalent(gameHandler, *talent);
            } else {
                // Empty slot
                ImGui::InvisibleButton(("empty_" + std::to_string(row) + "_" + std::to_string(col)).c_str(),
                                     ImVec2(iconSize, iconSize));
            }
        }
    }

    ImGui::EndChild();
}

void TalentScreen::renderTalent(game::GameHandler& gameHandler,
                                const game::GameHandler::TalentEntry& talent) {
    auto* assetManager = core::Application::getInstance().getAssetManager();

    uint8_t currentRank = gameHandler.getTalentRank(talent.talentId);

    // Check if can learn
    bool canLearn = currentRank < talent.maxRank &&
                    gameHandler.getUnspentTalentPoints() > 0;

    // Check prerequisites
    bool prereqsMet = true;
    for (int i = 0; i < 3; ++i) {
        if (talent.prereqTalent[i] != 0) {
            uint8_t prereqRank = gameHandler.getTalentRank(talent.prereqTalent[i]);
            if (prereqRank < talent.prereqRank[i]) {
                prereqsMet = false;
                canLearn = false;
                break;
            }
        }
    }

    // Check tier requirement (need 5 points in previous tier)
    if (talent.row > 0) {
        // Count points spent in this tree
        uint32_t pointsInTree = 0;
        for (const auto& [tid, rank] : gameHandler.getLearnedTalents()) {
            const auto* t = gameHandler.getTalentEntry(tid);
            if (t && t->tabId == talent.tabId) {
                pointsInTree += rank;
            }
        }

        uint32_t requiredPoints = talent.row * 5;
        if (pointsInTree < requiredPoints) {
            canLearn = false;
        }
    }

    // Determine state color and tint
    ImVec4 borderColor;
    ImVec4 tint;
    if (currentRank == talent.maxRank) {
        borderColor = ImVec4(0.3f, 0.9f, 0.3f, 1.0f);  // Green border (maxed)
        tint = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);         // Full color
    } else if (currentRank > 0) {
        borderColor = ImVec4(1.0f, 0.9f, 0.3f, 1.0f);  // Yellow border (partial)
        tint = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);         // Full color
    } else if (canLearn && prereqsMet) {
        borderColor = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);  // White border (available)
        tint = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);         // Full color
    } else {
        borderColor = ImVec4(0.5f, 0.5f, 0.5f, 1.0f);  // Gray border (locked)
        tint = ImVec4(0.5f, 0.5f, 0.5f, 1.0f);         // Desaturated
    }

    const float iconSize = 40.0f;
    ImGui::PushID(static_cast<int>(talent.talentId));

    // Get spell icon
    uint32_t spellId = talent.rankSpells[0];
    VkDescriptorSet iconTex = VK_NULL_HANDLE;
    if (spellId != 0) {
        auto it = spellIconIds.find(spellId);
        if (it != spellIconIds.end()) {
            iconTex = getSpellIcon(it->second, assetManager);
        }
    }

    // Use InvisibleButton for click handling
    bool clicked = ImGui::InvisibleButton("##talent", ImVec2(iconSize, iconSize));
    bool hovered = ImGui::IsItemHovered();

    // Draw icon and border
    ImVec2 pMin = ImGui::GetItemRectMin();
    ImVec2 pMax = ImGui::GetItemRectMax();
    auto* drawList = ImGui::GetWindowDrawList();

    // Border
    float borderThickness = hovered ? 3.0f : 2.0f;
    ImU32 borderCol = IM_COL32(borderColor.x * 255, borderColor.y * 255, borderColor.z * 255, 255);
    drawList->AddRect(pMin, pMax, borderCol, 0.0f, 0, borderThickness);

    // Icon or colored background
    if (iconTex) {
        ImU32 tintCol = IM_COL32(tint.x * 255, tint.y * 255, tint.z * 255, tint.w * 255);
        drawList->AddImage((ImTextureID)(uintptr_t)iconTex,
                          ImVec2(pMin.x + 2, pMin.y + 2),
                          ImVec2(pMax.x - 2, pMax.y - 2),
                          ImVec2(0, 0), ImVec2(1, 1), tintCol);
    } else {
        ImU32 bgCol = IM_COL32(borderColor.x * 80, borderColor.y * 80, borderColor.z * 80, 255);
        drawList->AddRectFilled(ImVec2(pMin.x + 2, pMin.y + 2),
                               ImVec2(pMax.x - 2, pMax.y - 2), bgCol);
    }

    // Rank indicator overlay
    if (talent.maxRank > 1) {
        ImVec2 pMax = ImGui::GetItemRectMax();
        auto* drawList = ImGui::GetWindowDrawList();

        // Display rank: if learned, show (rank+1) since ranks are 0-indexed
        const auto& learned = gameHandler.getLearnedTalents();
        uint8_t displayRank = (learned.find(talent.talentId) != learned.end()) ? currentRank + 1 : 0;

        char rankText[16];
        snprintf(rankText, sizeof(rankText), "%u/%u", displayRank, talent.maxRank);

        ImVec2 textSize = ImGui::CalcTextSize(rankText);
        ImVec2 textPos(pMax.x - textSize.x - 2, pMax.y - textSize.y - 2);

        // Shadow
        drawList->AddText(ImVec2(textPos.x + 1, textPos.y + 1), IM_COL32(0, 0, 0, 255), rankText);
        // Text
        ImU32 rankCol = displayRank == talent.maxRank ? IM_COL32(0, 255, 0, 255) :
                        displayRank > 0 ? IM_COL32(255, 255, 0, 255) :
                        IM_COL32(255, 255, 255, 255);
        drawList->AddText(textPos, rankCol, rankText);
    }

    // Enhanced tooltip
    if (hovered) {
        ImGui::BeginTooltip();

        // Spell name
        const std::string& spellName = gameHandler.getSpellName(spellId);
        if (!spellName.empty()) {
            ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.3f, 1.0f), "%s", spellName.c_str());
        } else {
            ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.3f, 1.0f), "Talent #%u", talent.talentId);
        }

        // Rank
        ImGui::TextColored(borderColor, "Rank %u/%u", currentRank, talent.maxRank);

        // Current rank description
        if (currentRank > 0 && talent.rankSpells[currentRank - 1] != 0) {
            auto tooltipIt = spellTooltips.find(talent.rankSpells[currentRank - 1]);
            if (tooltipIt != spellTooltips.end() && !tooltipIt->second.empty()) {
                ImGui::PushTextWrapPos(ImGui::GetCursorPos().x + 300.0f);
                ImGui::TextColored(ImVec4(1.0f, 0.82f, 0.0f, 1.0f), "Current:");
                ImGui::TextWrapped("%s", tooltipIt->second.c_str());
                ImGui::PopTextWrapPos();
            }
        }

        // Next rank description
        if (currentRank < talent.maxRank && talent.rankSpells[currentRank] != 0) {
            auto tooltipIt = spellTooltips.find(talent.rankSpells[currentRank]);
            if (tooltipIt != spellTooltips.end() && !tooltipIt->second.empty()) {
                ImGui::Spacing();
                ImGui::PushTextWrapPos(ImGui::GetCursorPos().x + 300.0f);
                ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), "Next Rank:");
                ImGui::TextWrapped("%s", tooltipIt->second.c_str());
                ImGui::PopTextWrapPos();
            }
        }

        // Prerequisites
        for (int i = 0; i < 3; ++i) {
            if (talent.prereqTalent[i] != 0) {
                const auto* prereq = gameHandler.getTalentEntry(talent.prereqTalent[i]);
                if (prereq && prereq->rankSpells[0] != 0) {
                    uint8_t prereqCurrentRank = gameHandler.getTalentRank(talent.prereqTalent[i]);
                    bool met = prereqCurrentRank >= talent.prereqRank[i];
                    ImVec4 prereqColor = met ? ImVec4(0.3f, 0.9f, 0.3f, 1.0f) : ImVec4(1.0f, 0.3f, 0.3f, 1.0f);

                    const std::string& prereqName = gameHandler.getSpellName(prereq->rankSpells[0]);
                    ImGui::Spacing();
                    ImGui::TextColored(prereqColor, "Requires %u point%s in %s",
                        talent.prereqRank[i],
                        talent.prereqRank[i] > 1 ? "s" : "",
                        prereqName.empty() ? "prerequisite" : prereqName.c_str());
                }
            }
        }

        // Tier requirement
        if (talent.row > 0 && currentRank == 0) {
            uint32_t pointsInTree = 0;
            for (const auto& [tid, rank] : gameHandler.getLearnedTalents()) {
                const auto* t = gameHandler.getTalentEntry(tid);
                if (t && t->tabId == talent.tabId) {
                    pointsInTree += rank;
                }
            }
            uint32_t requiredPoints = talent.row * 5;
            if (pointsInTree < requiredPoints) {
                ImGui::Spacing();
                ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f),
                    "Requires %u points in this tree (%u/%u)",
                    requiredPoints, pointsInTree, requiredPoints);
            }
        }

        // Action hint
        if (canLearn && prereqsMet) {
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), "Click to learn");
        } else if (currentRank >= talent.maxRank) {
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.3f, 0.9f, 0.3f, 1.0f), "Maxed");
        }

        ImGui::EndTooltip();
    }

    // Handle click
    if (clicked) {
        LOG_INFO("Talent clicked: id=", talent.talentId, " canLearn=", canLearn, " prereqsMet=", prereqsMet,
                 " currentRank=", static_cast<int>(currentRank), " maxRank=", static_cast<int>(talent.maxRank),
                 " unspent=", static_cast<int>(gameHandler.getUnspentTalentPoints()));

        if (canLearn && prereqsMet) {
            // Rank is 0-indexed: first point = rank 0, second = rank 1, etc.
            // Check if talent is already learned
            const auto& learned = gameHandler.getLearnedTalents();
            uint8_t desiredRank;
            if (learned.find(talent.talentId) == learned.end()) {
                // Not learned yet, learn first rank (0)
                desiredRank = 0;
            } else {
                // Already learned, upgrade to next rank
                desiredRank = currentRank + 1;
            }
            LOG_INFO("Sending CMSG_LEARN_TALENT for talent ", talent.talentId, " rank ", static_cast<int>(desiredRank), " (0-indexed)");
            gameHandler.learnTalent(talent.talentId, desiredRank);
        } else {
            if (!canLearn) LOG_WARNING("Cannot learn: canLearn=false");
            if (!prereqsMet) LOG_WARNING("Cannot learn: prereqsMet=false");
        }
    }

    ImGui::PopID();
}

void TalentScreen::loadSpellDBC(pipeline::AssetManager* assetManager) {
    if (spellDbcLoaded) return;
    spellDbcLoaded = true;

    if (!assetManager || !assetManager->isInitialized()) return;

    auto dbc = assetManager->loadDBC("Spell.dbc");
    if (!dbc || !dbc->isLoaded()) {
        LOG_WARNING("Talent screen: Could not load Spell.dbc");
        return;
    }

    // WoW 3.3.5a Spell.dbc fields: 0=SpellID, 133=SpellIconID, 136=SpellName_enUS, 139=Tooltip_enUS
    const auto* spellL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("Spell") : nullptr;
    uint32_t count = dbc->getRecordCount();
    for (uint32_t i = 0; i < count; ++i) {
        uint32_t spellId = dbc->getUInt32(i, spellL ? (*spellL)["ID"] : 0);
        if (spellId == 0) continue;

        uint32_t iconId = dbc->getUInt32(i, spellL ? (*spellL)["IconID"] : 133);
        spellIconIds[spellId] = iconId;

        std::string tooltip = dbc->getString(i, spellL ? (*spellL)["Tooltip"] : 139);
        if (!tooltip.empty()) {
            spellTooltips[spellId] = tooltip;
        }
    }

    LOG_INFO("Talent screen: Loaded ", spellIconIds.size(), " spell icons from Spell.dbc");
}

void TalentScreen::loadSpellIconDBC(pipeline::AssetManager* assetManager) {
    if (iconDbcLoaded) return;
    iconDbcLoaded = true;

    if (!assetManager || !assetManager->isInitialized()) return;

    auto dbc = assetManager->loadDBC("SpellIcon.dbc");
    if (!dbc || !dbc->isLoaded()) {
        LOG_WARNING("Talent screen: Could not load SpellIcon.dbc");
        return;
    }

    const auto* iconL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("SpellIcon") : nullptr;
    for (uint32_t i = 0; i < dbc->getRecordCount(); i++) {
        uint32_t id = dbc->getUInt32(i, iconL ? (*iconL)["ID"] : 0);
        std::string path = dbc->getString(i, iconL ? (*iconL)["Path"] : 1);
        if (!path.empty() && id > 0) {
            spellIconPaths[id] = path;
        }
    }

    LOG_INFO("Talent screen: Loaded ", spellIconPaths.size(), " spell icon paths from SpellIcon.dbc");
}

VkDescriptorSet TalentScreen::getSpellIcon(uint32_t iconId, pipeline::AssetManager* assetManager) {
    if (iconId == 0 || !assetManager) return VK_NULL_HANDLE;

    auto cit = spellIconCache.find(iconId);
    if (cit != spellIconCache.end()) return cit->second;

    auto pit = spellIconPaths.find(iconId);
    if (pit == spellIconPaths.end()) {
        spellIconCache[iconId] = VK_NULL_HANDLE;
        return VK_NULL_HANDLE;
    }

    std::string iconPath = pit->second + ".blp";
    auto blpData = assetManager->readFile(iconPath);
    if (blpData.empty()) {
        spellIconCache[iconId] = VK_NULL_HANDLE;
        return VK_NULL_HANDLE;
    }

    auto image = pipeline::BLPLoader::load(blpData);
    if (!image.isValid()) {
        spellIconCache[iconId] = VK_NULL_HANDLE;
        return VK_NULL_HANDLE;
    }

    auto* window = core::Application::getInstance().getWindow();
    auto* vkCtx = window ? window->getVkContext() : nullptr;
    if (!vkCtx) {
        spellIconCache[iconId] = VK_NULL_HANDLE;
        return VK_NULL_HANDLE;
    }

    VkDescriptorSet ds = vkCtx->uploadImGuiTexture(image.data.data(), image.width, image.height);
    spellIconCache[iconId] = ds;
    return ds;
}

}} // namespace wowee::ui
