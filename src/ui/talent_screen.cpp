#include "ui/talent_screen.hpp"
#include "ui/ui_texture_load.hpp"
#include "ui/ui_upload_budget.hpp"
#include "ui/ui_colors.hpp"
#include "ui/keybinding_manager.hpp"
#include "core/input.hpp"
#include "core/application.hpp"
#include "core/logger.hpp"
#include "rendering/vk_context.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_layout.hpp"
#include "pipeline/spell_icon_paths.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <climits>

namespace wowee { namespace ui {

void TalentScreen::render(game::GameHandler& gameHandler) {
    // Talents toggle via keybinding (edge-triggered)
    // Customizable key (default: N) from KeybindingManager
    bool talentsDown = KeybindingManager::getInstance().isActionPressed(
        KeybindingManager::Action::TOGGLE_TALENTS, false);
    if (talentsDown && !nKeyWasDown) {
        open = !open;
    }
    nKeyWasDown = talentsDown;

    if (!open) return;

    auto* window = core::Application::getInstance().getWindow();
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    float winW = 680.0f;
    float winH = 600.0f;
    float winX = (screenW - winW) * 0.5f;
    float winY = (screenH - winH) * 0.5f;

    ImGui::SetNextWindowPos(ImVec2(winX, winY), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(winW, winH), ImGuiCond_FirstUseEver);

    // Build title with point distribution
    uint8_t playerClass = gameHandler.getPlayerClass();
    std::string title = "Talents";
    if (playerClass > 0) {
        title = std::string(game::getClassName(static_cast<game::Class>(playerClass))) + " Talents";
    }

    bool windowOpen = open;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8, 8));
    if (ImGui::Begin(title.c_str(), &windowOpen)) {
        renderTalentTrees(gameHandler);
    }
    ImGui::End();
    ImGui::PopStyleVar();

    if (!windowOpen) {
        open = false;
    }
}

void TalentScreen::renderTalentTrees(game::GameHandler& gameHandler) {
    auto* assetManager = core::Application::getInstance().getAssetManager();

    // Ensure talent DBCs are loaded once
    static bool dbcLoadAttempted = false;
    if (!dbcLoadAttempted) {
        dbcLoadAttempted = true;
        gameHandler.loadTalentDbc();
        loadSpellDBC(assetManager);
        loadSpellIconDBC(assetManager);
        loadGlyphPropertiesDBC(assetManager);
    }

    uint8_t playerClass = gameHandler.getPlayerClass();
    if (playerClass == 0) {
        ImGui::TextDisabled("Class information not available.");
        return;
    }

    // Get talent tabs for this class, sorted by orderIndex.
    // WoW class IDs are 1-indexed (Warrior=1..Druid=11); convert to bitmask for
    // TalentTab.classMask matching (Warrior=0x1, Paladin=0x2, Hunter=0x4, etc.)
    uint32_t classMask = 1u << (playerClass - 1);
    std::vector<const game::GameHandler::TalentTabEntry*> classTabs;
    for (const auto& [tabId, tab] : gameHandler.getAllTalentTabs()) {
        if (tab.classMask & classMask) {
            classTabs.push_back(&tab);
        }
    }
    std::sort(classTabs.begin(), classTabs.end(),
        [](const auto* a, const auto* b) { return a->orderIndex < b->orderIndex; });

    if (classTabs.empty()) {
        ImGui::TextDisabled("No talent trees available for your class.");
        return;
    }

    // Compute points-per-tree for display
    uint32_t treeTotals[3] = {0, 0, 0};
    for (size_t ti = 0; ti < classTabs.size() && ti < 3; ti++) {
        for (const auto& [tid, rank] : gameHandler.getLearnedTalents()) {
            const auto* t = gameHandler.getTalentEntry(tid);
            if (t && t->tabId == classTabs[ti]->tabId) {
                treeTotals[ti] += rank;
            }
        }
    }

    // Header: spec switcher + unspent points + point distribution
    uint8_t activeSpec = gameHandler.getActiveTalentSpec();
    uint8_t unspent = gameHandler.getUnspentTalentPoints();

    // Spec buttons
    for (uint8_t s = 0; s < 2; s++) {
        if (s > 0) ImGui::SameLine();
        bool isActive = (s == activeSpec);
        if (isActive) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.5f, 0.8f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.6f, 0.9f, 1.0f));
        }
        char specLabel[32];
        snprintf(specLabel, sizeof(specLabel), "Spec %u", s + 1);
        if (ImGui::Button(specLabel, ImVec2(70, 0))) {
            if (!isActive) gameHandler.switchTalentSpec(s);
        }
        if (isActive) ImGui::PopStyleColor(2);
    }

    // Point distribution
    ImGui::SameLine(0, 20);
    if (classTabs.size() >= 3) {
        ImGui::Text("(%u / %u / %u)", treeTotals[0], treeTotals[1], treeTotals[2]);
    }

    // Unspent points
    ImGui::SameLine(0, 20);
    if (unspent > 0) {
        ImGui::TextColored(ui::colors::kBrightGreen, "%u point%s available",
                          unspent, unspent > 1 ? "s" : "");
    } else {
        ImGui::TextColored(ui::colors::kDarkGray, "No points available");
    }

    if (!classTabs.empty()) {
        uint32_t totalSpent = 0;
        for (size_t ti = 0; ti < classTabs.size() && ti < 3; ti++) {
            totalSpent += treeTotals[ti];
        }

        ImGui::Spacing();
        ImGui::TextDisabled("Spent: %u", totalSpent);
        for (size_t ti = 0; ti < classTabs.size() && ti < 3; ti++) {
            ImGui::SameLine(0, 16);
            ImGui::Text("%s: %u", classTabs[ti]->name.c_str(), treeTotals[ti]);
        }
    }

    ImGui::Separator();

    // Render tabs with point counts in tab labels
    if (ImGui::BeginTabBar("TalentTabs")) {
        for (size_t ti = 0; ti < classTabs.size(); ti++) {
            const auto* tab = classTabs[ti];
            char tabLabel[128];
            uint32_t pts = (ti < 3) ? treeTotals[ti] : 0;
            snprintf(tabLabel, sizeof(tabLabel), "%s (%u)###tab%u", tab->name.c_str(), pts, tab->tabId);

            if (ImGui::BeginTabItem(tabLabel)) {
                renderTalentTree(gameHandler, tab->tabId, tab->backgroundFile);
                ImGui::EndTabItem();
            }
        }

        // Glyphs tab (WotLK only - visible when any glyph slot is populated or DBC data loaded)
        if (!glyphProperties_.empty() || [&]() {
                const auto& g = gameHandler.getGlyphs();
                for (auto id : g) if (id != 0) return true;
                return false; }()) {
            if (ImGui::BeginTabItem("Glyphs")) {
                renderGlyphs(gameHandler);
                ImGui::EndTabItem();
            }
        }

        ImGui::EndTabBar();
    }

    // Talent learn confirmation popup
    if (talentConfirmOpen_) {
        ImGui::OpenPopup("Learn Talent?##talent_confirm");
        talentConfirmOpen_ = false;
    }
    if (ImGui::BeginPopupModal("Learn Talent?##talent_confirm", nullptr,
                               ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove)) {
        ImGui::TextColored(ui::colors::kYellow, "%s", pendingTalentName_.c_str());
        ImGui::Text("Rank %u", pendingTalentRank_ + 1);
        ImGui::Spacing();
        ImGui::TextWrapped("Spend a talent point?");
        ImGui::Spacing();
        if (ImGui::Button("Learn", ImVec2(80, 0))) {
            // pendingTalentRank_ holds how many ranks are already learned, which
            // is the wire's index for the next one. learnTalent counts a talent's
            // first rank as 1 - the same as everywhere above the wire - so this
            // is the rank being asked for, and the builder converts.
            gameHandler.learnTalent(pendingTalentId_, pendingTalentRank_ + 1);
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(80, 0))) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

void TalentScreen::renderTalentTree(game::GameHandler& gameHandler, uint32_t tabId,
                                      const std::string& bgFile) {
    auto* assetManager = core::Application::getInstance().getAssetManager();

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

    // Sort talents by row then column for consistent rendering
    std::sort(talents.begin(), talents.end(), [](const auto* a, const auto* b) {
        if (a->row != b->row) return a->row < b->row;
        return a->column < b->column;
    });

    // Find grid dimensions - use int to avoid uint8_t wrap-around infinite loops
    int maxRow = 0, maxCol = 0;
    for (const auto* talent : talents) {
        maxRow = std::max(maxRow, static_cast<int>(talent->row));
        maxCol = std::max(maxCol, static_cast<int>(talent->column));
    }
    // Sanity-cap to prevent runaway loops from corrupt/unexpected DBC data
    maxRow = std::min(maxRow, 15);
    maxCol = std::min(maxCol, 15);
    // WoW talent grids are always 4 columns wide
    if (maxCol < 3) maxCol = 3;

    const float iconSize = 40.0f;
    const float spacing = 8.0f;
    const float cellSize = iconSize + spacing;
    const float gridWidth = static_cast<float>(maxCol + 1) * cellSize + spacing;
    const float gridHeight = static_cast<float>(maxRow + 1) * cellSize + spacing;

    // Points in this tree
    uint32_t pointsInTree = 0;
    for (const auto& [tid, rank] : gameHandler.getLearnedTalents()) {
        const auto* t = gameHandler.getTalentEntry(tid);
        if (t && t->tabId == tabId) {
            pointsInTree += rank;
        }
    }

    // Center the grid
    float availW = ImGui::GetContentRegionAvail().x;
    float offsetX = std::max(0.0f, (availW - gridWidth) * 0.5f);

    char childId[32];
    snprintf(childId, sizeof(childId), "TalentGrid_%u", tabId);
    ImGui::BeginChild(childId, ImVec2(0, 0), false);

    ImVec2 gridOrigin = ImGui::GetCursorScreenPos();
    gridOrigin.x += offsetX;

    // Draw background texture if available
    if (!bgFile.empty() && assetManager) {
        VkDescriptorSet bgTex = VK_NULL_HANDLE;
        auto bgIt = bgTextureCache_.find(tabId);
        if (bgIt != bgTextureCache_.end()) {
            bgTex = bgIt->second;
        } else {
            // Only load the background if icon uploads aren't saturating this frame.
            // Background is cosmetic; skip if we're already loading icons this frame.
            std::string bgPath = bgFile;
            for (auto& c : bgPath) { if (c == '\\') c = '/'; }
            bgPath += ".blp";
            bgTex = uploadUiTextureFromBlp(
                assetManager, bgPath, core::Application::getInstance().getWindow());
            // Cache even if null to avoid retrying every frame on missing files
            bgTextureCache_[tabId] = bgTex;
        }

        if (bgTex) {
            auto* drawList = ImGui::GetWindowDrawList();
            float bgW = gridWidth + spacing * 2;
            float bgH = gridHeight + spacing * 2;
            drawList->AddImage((ImTextureID)(uintptr_t)bgTex,
                ImVec2(gridOrigin.x - spacing, gridOrigin.y - spacing),
                ImVec2(gridOrigin.x + bgW - spacing, gridOrigin.y + bgH - spacing),
                ImVec2(0, 0), ImVec2(1, 1),
                IM_COL32(255, 255, 255, 60));  // Subtle background
        }
    }

    // Build a position lookup for prerequisite arrows
    struct TalentPos {
        const game::GameHandler::TalentEntry* talent;
        ImVec2 center;
    };
    std::unordered_map<uint32_t, TalentPos> talentPositions;

    // First pass: compute positions
    for (const auto* talent : talents) {
        float x = gridOrigin.x + talent->column * cellSize + spacing;
        float y = gridOrigin.y + talent->row * cellSize + spacing;
        ImVec2 center(x + iconSize * 0.5f, y + iconSize * 0.5f);
        talentPositions[talent->talentId] = {talent, center};
    }

    // Draw prerequisite arrows
    auto* drawList = ImGui::GetWindowDrawList();
    for (const auto* talent : talents) {
        for (int i = 0; i < 3; ++i) {
            if (talent->prereqTalent[i] == 0) continue;
            auto fromIt = talentPositions.find(talent->prereqTalent[i]);
            auto toIt = talentPositions.find(talent->talentId);
            if (fromIt == talentPositions.end() || toIt == talentPositions.end()) continue;

            uint8_t prereqRank = gameHandler.getTalentRank(talent->prereqTalent[i]);
            bool met = prereqRank > talent->prereqRank[i]; // storage 1-indexed, DBC 0-indexed
            ImU32 lineCol = met ? IM_COL32(100, 220, 100, 200) : IM_COL32(120, 120, 120, 150);

            ImVec2 from = fromIt->second.center;
            ImVec2 to = toIt->second.center;

            // Draw line from bottom of prerequisite to top of dependent
            ImVec2 lineStart(from.x, from.y + iconSize * 0.5f);
            ImVec2 lineEnd(to.x, to.y - iconSize * 0.5f);
            drawList->AddLine(lineStart, lineEnd, lineCol, 2.0f);

            // Arrow head
            float arrowSize = 5.0f;
            drawList->AddTriangleFilled(
                ImVec2(lineEnd.x, lineEnd.y),
                ImVec2(lineEnd.x - arrowSize, lineEnd.y - arrowSize * 1.5f),
                ImVec2(lineEnd.x + arrowSize, lineEnd.y - arrowSize * 1.5f),
                lineCol);
        }
    }

    // Render talent icons
    for (int row = 0; row <= maxRow; ++row) {
        for (int col = 0; col <= maxCol; ++col) {
            const game::GameHandler::TalentEntry* talent = nullptr;
            for (const auto* t : talents) {
                if (t->row == row && t->column == col) {
                    talent = t;
                    break;
                }
            }

            float x = gridOrigin.x + col * cellSize + spacing;
            float y = gridOrigin.y + row * cellSize + spacing;

            ImGui::SetCursorScreenPos(ImVec2(x, y));

            if (talent) {
                renderTalent(gameHandler, *talent, pointsInTree);
            } else {
                // Empty cell - invisible placeholder
                char emptyId[32];
                snprintf(emptyId, sizeof(emptyId), "e_%u_%u_%u", tabId, row, col);
                ImGui::InvisibleButton(emptyId, ImVec2(iconSize, iconSize));
            }
        }
    }

    // Reserve space for the full grid so scrolling works
    ImGui::SetCursorScreenPos(ImVec2(gridOrigin.x, gridOrigin.y + gridHeight));
    ImGui::Dummy(ImVec2(gridWidth, 0));

    ImGui::EndChild();
}

void TalentScreen::renderTalent(game::GameHandler& gameHandler,
                                const game::GameHandler::TalentEntry& talent,
                                uint32_t pointsInTree) {
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
            if (prereqRank <= talent.prereqRank[i]) { // storage 1-indexed, DBC 0-indexed
                prereqsMet = false;
                canLearn = false;
                break;
            }
        }
    }

    // Check tier requirement (need row*5 points in tree)
    if (talent.row > 0) {
        uint32_t requiredPoints = talent.row * 5;
        if (pointsInTree < requiredPoints) {
            canLearn = false;
        }
    }

    // Determine visual state
    enum TalentState { MAXED, PARTIAL, AVAILABLE, LOCKED };
    TalentState state;
    if (currentRank >= talent.maxRank) {
        state = MAXED;
    } else if (currentRank > 0) {
        state = PARTIAL;
    } else if (canLearn && prereqsMet) {
        state = AVAILABLE;
    } else {
        state = LOCKED;
    }

    // Colors per state
    ImVec4 borderColor;
    ImVec4 tint;
    switch (state) {
        case MAXED:    borderColor = ImVec4(0.2f, 0.9f, 0.2f, 1.0f); tint = ui::colors::kWhite; break;
        case PARTIAL:  borderColor = ui::colors::kHealthGreen; tint = ui::colors::kWhite; break;
        case AVAILABLE:borderColor = ImVec4(1.0f, 1.0f, 1.0f, 0.8f); tint = ui::colors::kWhite; break;
        case LOCKED:   borderColor = ImVec4(0.4f, 0.4f, 0.4f, 0.8f); tint = ImVec4(0.4f,0.4f,0.4f,1); break;
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

    // Click target
    bool clicked = ImGui::InvisibleButton("##t", ImVec2(iconSize, iconSize));
    bool hovered = ImGui::IsItemHovered();

    ImVec2 pMin = ImGui::GetItemRectMin();
    ImVec2 pMax = ImGui::GetItemRectMax();
    auto* dl = ImGui::GetWindowDrawList();

    // Background fill
    ImU32 bgCol;
    if (state == LOCKED) {
        bgCol = IM_COL32(20, 20, 25, 200);
    } else {
        bgCol = IM_COL32(30, 30, 40, 200);
    }
    dl->AddRectFilled(pMin, pMax, bgCol, 3.0f);

    // Icon
    if (iconTex) {
        ImU32 tintCol = ImGui::ColorConvertFloat4ToU32(tint);
        dl->AddImage((ImTextureID)(uintptr_t)iconTex,
                     ImVec2(pMin.x + 2, pMin.y + 2),
                     ImVec2(pMax.x - 2, pMax.y - 2),
                     ImVec2(0, 0), ImVec2(1, 1), tintCol);
    }

    // Border
    float borderThick = hovered ? 2.5f : 1.5f;
    ImU32 borderCol = ImGui::ColorConvertFloat4ToU32(borderColor);
    dl->AddRect(pMin, pMax, borderCol, 3.0f, 0, borderThick);

    // Hover glow
    if (hovered && state != LOCKED) {
        dl->AddRect(ImVec2(pMin.x - 1, pMin.y - 1), ImVec2(pMax.x + 1, pMax.y + 1),
                    IM_COL32(255, 255, 255, 60), 3.0f, 0, 1.0f);
    }

    // Rank counter (bottom-right corner)
    {
        char rankText[16];
        snprintf(rankText, sizeof(rankText), "%u/%u", currentRank, talent.maxRank);
        ImVec2 textSize = ImGui::CalcTextSize(rankText);
        ImVec2 textPos(pMax.x - textSize.x - 2, pMax.y - textSize.y - 1);

        // Background pill for readability
        dl->AddRectFilled(ImVec2(textPos.x - 2, textPos.y - 1),
                          ImVec2(pMax.x, pMax.y),
                          IM_COL32(0, 0, 0, 180), 2.0f);

        // Text shadow
        dl->AddText(ImVec2(textPos.x + 1, textPos.y + 1), IM_COL32(0, 0, 0, 255), rankText);

        // Rank text color
        ImU32 rankCol;
        switch (state) {
            case MAXED:   rankCol = IM_COL32(80, 255, 80, 255); break;
            case PARTIAL: rankCol = IM_COL32(80, 255, 80, 255); break;
            default:      rankCol = IM_COL32(200, 200, 200, 255); break;
        }
        dl->AddText(textPos, rankCol, rankText);
    }

    // Tooltip
    if (hovered) {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(320.0f);

        // Spell name
        const std::string& spellName = gameHandler.getSpellName(spellId);
        if (!spellName.empty()) {
            ImGui::TextColored(ui::colors::kYellow, "%s", spellName.c_str());
        } else {
            ImGui::TextColored(ui::colors::kYellow, "Talent #%u", talent.talentId);
        }

        // Rank display
        ImVec4 rankColor;
        switch (state) {
            case MAXED:   rankColor = ui::colors::kQueueGreen; break;
            case PARTIAL: rankColor = ui::colors::kQueueGreen; break;
            default:      rankColor = ImVec4(0.7f, 0.7f, 0.7f, 1); break;
        }
        ImGui::TextColored(rankColor, "Rank %u/%u", currentRank, talent.maxRank);

        // Current rank description
        if (currentRank > 0 && currentRank <= 5 && talent.rankSpells[currentRank - 1] != 0) {
            std::string desc = describeRankSpell(gameHandler, talent.rankSpells[currentRank - 1]);
            if (!desc.empty()) {
                ImGui::Spacing();
                ImGui::TextColored(ui::colors::kTooltipGold, "Current:");
                ImGui::TextWrapped("%s", desc.c_str());
            }
        }

        // Next rank description
        if (currentRank < talent.maxRank && currentRank < 5 && talent.rankSpells[currentRank] != 0) {
            std::string desc = describeRankSpell(gameHandler, talent.rankSpells[currentRank]);
            if (!desc.empty()) {
                ImGui::Spacing();
                ImGui::TextColored(ui::colors::kBrightGreen,
                                   currentRank == 0 ? "Effect:" : "Next Rank:");
                ImGui::TextWrapped("%s", desc.c_str());
            }
        }

        // Prerequisites
        for (int i = 0; i < 3; ++i) {
            if (talent.prereqTalent[i] == 0) continue;
            const auto* prereq = gameHandler.getTalentEntry(talent.prereqTalent[i]);
            if (!prereq || prereq->rankSpells[0] == 0) continue;

            uint8_t prereqCurrentRank = gameHandler.getTalentRank(talent.prereqTalent[i]);
            bool met = prereqCurrentRank > talent.prereqRank[i]; // storage 1-indexed, DBC 0-indexed
            ImVec4 pColor = met ? ui::colors::kQueueGreen : ui::colors::kRed;

            const std::string& prereqName = gameHandler.getSpellName(prereq->rankSpells[0]);
            ImGui::Spacing();
            const uint8_t reqRankDisplay = talent.prereqRank[i] + 1u; // DBC 0-indexed → display 1-indexed
            ImGui::TextColored(pColor, "Requires %u point%s in %s",
                reqRankDisplay,
                reqRankDisplay > 1 ? "s" : "",
                prereqName.empty() ? "prerequisite" : prereqName.c_str());
        }

        // Tier requirement
        if (talent.row > 0 && currentRank == 0) {
            uint32_t requiredPoints = talent.row * 5;
            if (pointsInTree < requiredPoints) {
                ImGui::Spacing();
                ImGui::TextColored(ui::colors::kRed,
                    "Requires %u points in this tree (%u/%u)",
                    requiredPoints, pointsInTree, requiredPoints);
            }
        }

        // Action hint
        if (canLearn && prereqsMet) {
            ImGui::Spacing();
            ImGui::TextColored(ui::colors::kBrightGreen, "Click to learn");
        }

        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }

    // Handle click - open confirmation dialog instead of learning directly
    if (clicked && canLearn && prereqsMet) {
        talentConfirmOpen_ = true;
        pendingTalentId_ = talent.talentId;
        pendingTalentRank_ = currentRank;
        uint32_t nextSpell = (currentRank < 5) ? talent.rankSpells[currentRank] : 0;
        pendingTalentName_ = nextSpell ? gameHandler.getSpellName(nextSpell) : "";
        if (pendingTalentName_.empty())
            pendingTalentName_ = spellId ? gameHandler.getSpellName(spellId) : "Talent";
    }

    ImGui::PopID();
}

void TalentScreen::loadSpellDBC(pipeline::AssetManager* assetManager) {
    if (spellDbcLoaded) return;

    if (!assetManager || !assetManager->isInitialized()) return;
    // Not an attempt: the assets are not up yet, and a caller can reach
    // this before they are. Latching here recorded "read" for a file
    // never opened, and disabled it for the rest of the session.
    spellDbcLoaded = true;

    auto dbc = assetManager->loadDBC("Spell.dbc");
    if (!dbc || !dbc->isLoaded()) return;

    const auto* spellL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("Spell") : nullptr;
    uint32_t fieldCount = dbc->getFieldCount();
    uint32_t idField = 0, iconField = 133, tooltipField = 139;
    uint32_t descField = 0xFFFFFFFF;
    if (spellL) {
        try {
            uint32_t layoutId = (*spellL)["ID"];
            uint32_t layoutIcon = (*spellL)["IconID"];
            if (layoutId < fieldCount && layoutIcon < fieldCount) {
                idField = layoutId;
                iconField = layoutIcon;
                try {
                    uint32_t layoutTooltip = (*spellL)["Tooltip"];
                    if (layoutTooltip < fieldCount) tooltipField = layoutTooltip;
                } catch (...) {}
            }
        } catch (...) {}
        // Description is the full "what it does" text; may be absent on older layouts.
        uint32_t f = spellL->field("Description");
        if (f != 0xFFFFFFFF && f < fieldCount) descField = f;
    }
    uint32_t count = dbc->getRecordCount();
    for (uint32_t i = 0; i < count; ++i) {
        uint32_t spellId = dbc->getUInt32(i, idField);
        if (spellId == 0) continue;

        uint32_t iconId = dbc->getUInt32(i, iconField);
        spellIconIds[spellId] = iconId;

        std::string tooltip = dbc->getString(i, tooltipField);
        if (!tooltip.empty()) {
            spellTooltips[spellId] = tooltip;
        }
        if (descField != 0xFFFFFFFF) {
            std::string desc = dbc->getString(i, descField);
            if (!desc.empty()) spellDescriptions[spellId] = std::move(desc);
        }
    }
}

std::string TalentScreen::describeRankSpell(game::GameHandler& gameHandler, uint32_t spellId) const {
    // Prefer the full description; fall back to the short tooltip text.
    std::string text;
    auto dIt = spellDescriptions.find(spellId);
    if (dIt != spellDescriptions.end()) text = dIt->second;
    else {
        auto tIt = spellTooltips.find(spellId);
        if (tIt != spellTooltips.end()) text = tIt->second;
    }
    if (text.empty()) return text;
    // Token substitution ($s/$o/$d/... incl. cross-spell refs) is shared with the
    // spellbook and item tooltips via GameHandler.
    return gameHandler.formatSpellDescription(spellId, text);
}

void TalentScreen::loadSpellIconDBC(pipeline::AssetManager* assetManager) {
    if (iconDbcLoaded) return;

    if (!assetManager || !assetManager->isInitialized()) return;
    // Not an attempt: the assets are not up yet, and a caller can reach
    // this before they are. Latching here recorded "read" for a file
    // never opened, and disabled it for the rest of the session.
    iconDbcLoaded = true;

    pipeline::loadSpellIconPaths(assetManager, spellIconPaths);
}

void TalentScreen::loadGlyphPropertiesDBC(pipeline::AssetManager* assetManager) {
    if (glyphDbcLoaded) return;

    if (!assetManager || !assetManager->isInitialized()) return;
    // Not an attempt: the assets are not up yet, and a caller can reach
    // this before they are. Latching here recorded "read" for a file
    // never opened, and disabled it for the rest of the session.
    glyphDbcLoaded = true;

    auto dbc = assetManager->loadDBC("GlyphProperties.dbc");
    if (!dbc || !dbc->isLoaded()) return;

    // GlyphProperties.dbc: field 0=ID, field 1=SpellID, field 2=GlyphSlotFlags (1=minor), field 3=SpellIconID
    for (uint32_t i = 0; i < dbc->getRecordCount(); i++) {
        uint32_t id      = dbc->getUInt32(i, 0);
        uint32_t spellId = dbc->getUInt32(i, 1);
        uint32_t flags   = dbc->getUInt32(i, 2);
        if (id == 0) continue;
        GlyphInfo info;
        info.spellId = spellId;
        info.isMajor = (flags == 0);  // flag 0 = major, flag 1 = minor
        glyphProperties_[id] = info;
    }
}

void TalentScreen::renderGlyphs(game::GameHandler& gameHandler) {
    auto* assetManager = core::Application::getInstance().getAssetManager();
    const auto& glyphs = gameHandler.getGlyphs();

    ImGui::Spacing();
    ImGui::TextColored(ui::colors::kBrightGold, "Major Glyphs");
    ImGui::Separator();

    // WotLK: 6 glyph slots total. Slots 0,2,4 are major by convention from the server,
    // but we check GlyphProperties.dbc flags when available.
    // Display all 6 slots grouped: show major (non-minor) first, then minor.
    std::vector<std::pair<int, bool>> majorSlots, minorSlots;
    for (int i = 0; i < game::GameHandler::MAX_GLYPH_SLOTS; i++) {
        uint16_t glyphId = glyphs[i];
        bool isMajor = true;
        if (glyphId != 0) {
            auto git = glyphProperties_.find(glyphId);
            if (git != glyphProperties_.end()) isMajor = git->second.isMajor;
            else isMajor = (i % 2 == 0);  // fallback: even slots = major
        } else {
            isMajor = (i % 2 == 0);  // empty slots follow same pattern
        }
        if (isMajor) majorSlots.emplace_back(i, true);
        else         minorSlots.emplace_back(i, false);
    }

    auto renderGlyphSlot = [&](int slotIdx) {
        uint16_t glyphId = glyphs[slotIdx];
        char label[64];
        if (glyphId == 0) {
            snprintf(label, sizeof(label), "Slot %d  [Empty]", slotIdx + 1);
            ImGui::TextDisabled("%s", label);
            return;
        }

        uint32_t spellId = 0;
        uint32_t iconId  = 0;
        auto git = glyphProperties_.find(glyphId);
        if (git != glyphProperties_.end()) {
            spellId = git->second.spellId;
            auto iit = spellIconIds.find(spellId);
            if (iit != spellIconIds.end()) iconId = iit->second;
        }

        // Icon (24x24)
        VkDescriptorSet icon = getSpellIcon(iconId, assetManager);
        if (icon != VK_NULL_HANDLE) {
            ImGui::Image((ImTextureID)(uintptr_t)icon, ImVec2(24, 24));
            ImGui::SameLine(0, 6);
        } else {
            ImGui::Dummy(ImVec2(24, 24));
            ImGui::SameLine(0, 6);
        }

        // Spell name
        const std::string& name = spellId ? gameHandler.getSpellName(spellId) : "";
        if (!name.empty()) {
            ImGui::TextColored(ImVec4(0.9f, 0.9f, 0.9f, 1.0f), "%s", name.c_str());
        } else {
            ImGui::TextColored(ui::colors::kLightGray, "Glyph #%u", static_cast<uint32_t>(glyphId));
        }
    };

    for (auto& [idx, major] : majorSlots) renderGlyphSlot(idx);

    ImGui::Spacing();
    ImGui::TextColored(ImVec4(0.6f, 0.8f, 1.0f, 1.0f), "Minor Glyphs");
    ImGui::Separator();
    for (auto& [idx, major] : minorSlots) renderGlyphSlot(idx);
}

VkDescriptorSet TalentScreen::getSpellIcon(uint32_t iconId, pipeline::AssetManager* assetManager) {
    return cachedIconTexture(iconId, assetManager,
                             core::Application::getInstance().getWindow(),
                             spellIconPaths, spellIconCache);
}

}} // namespace wowee::ui
