#include "ui/quest_log_screen.hpp"
#include "ui/ui_colors.hpp"
#include "ui/inventory_screen.hpp"
#include "ui/keybinding_manager.hpp"
#include "ui/chat/chat_utils.hpp"
#include "core/application.hpp"
#include "core/input.hpp"
#include <imgui.h>
#include <algorithm>
#include <cctype>
#include <cstdio>

namespace wowee { namespace ui {

namespace {

/// One list of reward items, with icons and tooltips.
///
/// The quest detail pane shows two of these, the rewards a quest always gives
/// and the ones the player chooses between, and they were drawn by two copies
/// of the same twenty lines. Nothing is drawn at all when the list holds no
/// real item, which is why the heading is inside here: an empty "You will
/// receive:" above nothing is worse than no heading.
template <typename Items>
void renderRewardItemList(const char* heading, const Items& items,
                          game::GameHandler& gameHandler, InventoryScreen& invScreen) {
    bool any = false;
    for (const auto& ri : items) {
        if (ri.itemId) { any = true; break; }
    }
    if (!any) return;

    ImGui::TextDisabled("%s", heading);
    for (const auto& ri : items) {
        if (!ri.itemId) continue;

        std::string name = "Item " + std::to_string(ri.itemId);
        uint32_t dispId = 0;
        const auto* info = gameHandler.getItemInfo(ri.itemId);
        if (info && info->valid) {
            if (!info->name.empty()) name = info->name;
            dispId = info->displayInfoId;
        }

        VkDescriptorSet icon = dispId ? invScreen.getItemIcon(dispId) : VK_NULL_HANDLE;
        if (icon) {
            ImGui::Image((ImTextureID)(uintptr_t)icon, ImVec2(16, 16));
            ImGui::SameLine();
        }
        if (ri.count > 1) {
            ImGui::Text("%s x%u", name.c_str(), ri.count);
        } else {
            ImGui::Text("%s", name.c_str());
        }
        if (info && info->valid && ImGui::IsItemHovered()) {
            ImGui::BeginTooltip();
            invScreen.renderItemTooltip(*info, &gameHandler.getInventory());
            ImGui::EndTooltip();
        }
    }
}


std::string cleanQuestTitleForUi(const std::string& raw, uint32_t questId) {
    std::string s = raw;

    auto looksUtf16LeBytes = [](const std::string& str) -> bool {
        if (str.size() < 6) return false;
        size_t nulCount = 0;
        size_t oddNul = 0;
        for (size_t i = 0; i < str.size(); i++) {
            if (str[i] == '\0') {
                nulCount++;
                if (i & 1) oddNul++;
            }
        }
        return (nulCount >= str.size() / 4) && (oddNul >= (nulCount * 3) / 4);
    };

    if (looksUtf16LeBytes(s)) {
        std::string collapsed;
        collapsed.reserve(s.size() / 2);
        for (size_t i = 0; i + 1 < s.size(); i += 2) {
            unsigned char lo = static_cast<unsigned char>(s[i]);
            unsigned char hi = static_cast<unsigned char>(s[i + 1]);
            if (lo == 0 && hi == 0) break;
            if (hi != 0) { collapsed.clear(); break; }
            collapsed.push_back(static_cast<char>(lo));
        }
        if (!collapsed.empty()) s = std::move(collapsed);
    }

    // Keep a stable ASCII view for list rendering; malformed multibyte/UTF-16 noise
    // is a common source of one-glyph/half-glyph quest labels.
    std::string ascii;
    ascii.reserve(s.size());
    for (unsigned char uc : s) {
        if (uc >= 0x20 && uc <= 0x7E) ascii.push_back(static_cast<char>(uc));
        else if (uc == '\t' || uc == '\n' || uc == '\r') ascii.push_back(' ');
    }
    if (ascii.size() >= 4) s = std::move(ascii);

    for (char& c : s) {
        unsigned char uc = static_cast<unsigned char>(c);
        if (uc == 0) { c = ' '; continue; }
        if (uc < 0x20 && c != '\n' && c != '\t') c = ' ';
    }
    while (!s.empty() && s.front() == ' ') s.erase(s.begin());
    while (!s.empty() && s.back() == ' ') s.pop_back();

    int shortWordCount = 0;
    int wordCount = 0;
    int currentWordLen = 0;
    for (char c : s) {
        if (c == ' ') {
            if (currentWordLen > 0) {
                wordCount++;
                if (currentWordLen <= 1) shortWordCount++;
                currentWordLen = 0;
            }
        } else {
            currentWordLen++;
        }
    }
    if (currentWordLen > 0) {
        wordCount++;
        if (currentWordLen <= 1) shortWordCount++;
    }

    // Heuristic for broken UTF-16-like text that turns into "T h e  B e g i n n i n g".
    if (wordCount >= 6 && shortWordCount == wordCount && static_cast<int>(s.size()) > 12) {
        std::string compact;
        compact.reserve(s.size());
        for (char c : s) {
            if (c != ' ') compact.push_back(c);
        }
        if (compact.size() >= 4) s = compact;
    }

    if (s.size() < 4) s = "Quest #" + std::to_string(questId);
    if (s.size() > 72) s = s.substr(0, 72) + "...";
    return s;
}

} // anonymous namespace

void QuestLogScreen::render(game::GameHandler& gameHandler, InventoryScreen& invScreen) {
    // Quests toggle via keybinding (edge-triggered)
    // Customizable key (default: L) from KeybindingManager
    bool questsDown = KeybindingManager::getInstance().isActionPressed(
        KeybindingManager::Action::TOGGLE_QUESTS, false);
    if (questsDown && !lKeyWasDown) {
        open = !open;
    }
    lKeyWasDown = questsDown;

    if (!open) return;

    auto* window = core::Application::getInstance().getWindow();
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    float logW = std::min(980.0f, screenW - 80.0f);
    float logH = std::min(620.0f, screenH - 100.0f);
    float logX = (screenW - logW) * 0.5f;
    float logY = 50.0f;

    ImGui::SetNextWindowPos(ImVec2(logX, logY), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(logW, logH), ImGuiCond_FirstUseEver);

    bool stillOpen = true;
    if (ImGui::Begin("Quest Log", &stillOpen)) {
        const float footerHeight = 42.0f;
        ImGui::BeginChild("QuestLogMain", ImVec2(0, -footerHeight), false);

        const auto& quests = gameHandler.getQuestLog();
        if (selectedIndex >= static_cast<int>(quests.size())) {
            selectedIndex = quests.empty() ? -1 : static_cast<int>(quests.size()) - 1;
        }

        int activeCount = 0;
        int completeCount = 0;
        for (const auto& q : quests) {
            if (q.complete) completeCount++;
            else activeCount++;
        }

        // Search bar + filter buttons on one row
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - 210.0f);
        ImGui::InputTextWithHint("##qsearch", "Search quests...", questSearchFilter_, sizeof(questSearchFilter_));
        ImGui::SameLine();
        if (ImGui::RadioButton("All", questFilterMode_ == 0))     questFilterMode_ = 0;
        ImGui::SameLine();
        if (ImGui::RadioButton("Active", questFilterMode_ == 1))  questFilterMode_ = 1;
        ImGui::SameLine();
        if (ImGui::RadioButton("Ready", questFilterMode_ == 2))   questFilterMode_ = 2;

        // Summary counts + capacity (server caps the log at 20/25 slots)
        ImGui::TextColored(ImVec4(0.95f, 0.85f, 0.35f, 1.0f), "Active: %d", activeCount);
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.45f, 0.95f, 0.45f, 1.0f), "Ready: %d", completeCount);
        ImGui::SameLine();
        const int maxSlots = gameHandler.getMaxQuestLogSlots();
        const int totalCount = static_cast<int>(quests.size());
        ImVec4 capColor = totalCount >= maxSlots
            ? ImVec4(1.0f, 0.45f, 0.45f, 1.0f)
            : ImVec4(0.6f, 0.6f, 0.65f, 1.0f);
        ImGui::TextColored(capColor, "(%d/%d)", totalCount, maxSlots);
        ImGui::Separator();

        if (quests.empty()) {
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.75f, 1.0f), "No active quests.");
        } else {
            float paneW = ImGui::GetContentRegionAvail().x * 0.42f;
            if (paneW < 260.0f) paneW = 260.0f;
            if (paneW > 420.0f) paneW = 420.0f;

            ImGui::BeginChild("QuestListPane", ImVec2(paneW, 0), true);
            ImGui::TextColored(ImVec4(0.85f, 0.82f, 0.74f, 1.0f), "Quest List");
            ImGui::Separator();

            // Resolve pending select from tracker click
            if (pendingSelectQuestId_ != 0) {
                for (size_t i = 0; i < quests.size(); i++) {
                    if (quests[i].questId == pendingSelectQuestId_) {
                        selectedIndex = static_cast<int>(i);
                        // Clear filter so the target quest is visible
                        questSearchFilter_[0] = '\0';
                        questFilterMode_ = 0;
                        break;
                    }
                }
                pendingSelectQuestId_ = 0;
            }

            // Build a case-insensitive lowercase copy of the search filter once
            char filterLower[64] = {};
            for (size_t fi = 0; fi < sizeof(questSearchFilter_) && questSearchFilter_[fi]; ++fi)
                filterLower[fi] = static_cast<char>(std::tolower(static_cast<unsigned char>(questSearchFilter_[fi])));

            // Group visible quests by zone: ZoneOrSort > 0 = AreaTable zone,
            // < 0 = QuestSort category (class/profession/seasonal).
            struct ZoneGroup {
                std::string name;
                std::vector<size_t> indices;
            };
            std::vector<ZoneGroup> zoneGroups;
            int visibleQuestCount = 0;
            for (size_t i = 0; i < quests.size(); i++) {
                const auto& q = quests[i];

                // Apply mode filter
                if (questFilterMode_ == 1 && q.complete) continue;
                if (questFilterMode_ == 2 && !q.complete) continue;

                // Apply name search filter
                if (filterLower[0]) {
                    std::string titleLower = cleanQuestTitleForUi(q.title, q.questId);
                    for (char& c : titleLower) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                    if (titleLower.find(filterLower) == std::string::npos) continue;
                }

                visibleQuestCount++;
                std::string zoneName;
                if (q.zoneOrSort > 0)
                    zoneName = gameHandler.getAreaName(static_cast<uint32_t>(q.zoneOrSort));
                else if (q.zoneOrSort < 0)
                    zoneName = gameHandler.getQuestSortName(static_cast<uint32_t>(-q.zoneOrSort));
                if (zoneName.empty()) zoneName = "Other";

                auto grpIt = std::find_if(zoneGroups.begin(), zoneGroups.end(),
                    [&](const ZoneGroup& g) { return g.name == zoneName; });
                if (grpIt == zoneGroups.end()) {
                    zoneGroups.push_back({.name = std::move(zoneName), .indices = {}});
                    grpIt = std::prev(zoneGroups.end());
                }
                grpIt->indices.push_back(i);
            }
            // Alphabetical zones, "Other" last
            std::sort(zoneGroups.begin(), zoneGroups.end(),
                [](const ZoneGroup& a, const ZoneGroup& b) {
                    if ((a.name == "Other") != (b.name == "Other")) return b.name == "Other";
                    return a.name < b.name;
                });

            for (const auto& zoneGroup : zoneGroups) {
                // "###" keeps the header's collapse state stable as counts change
                char zoneHdr[160];
                snprintf(zoneHdr, sizeof(zoneHdr), "%s (%zu)###zg_%s",
                         zoneGroup.name.c_str(), zoneGroup.indices.size(), zoneGroup.name.c_str());
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.85f, 0.78f, 0.55f, 1.0f));
                bool zoneOpen = ImGui::CollapsingHeader(zoneHdr, ImGuiTreeNodeFlags_DefaultOpen);
                ImGui::PopStyleColor();
                if (!zoneOpen) continue;

            for (size_t i : zoneGroup.indices) {
                const auto& q = quests[i];
                ImGui::PushID(static_cast<int>(i));

                bool selected = (selectedIndex == static_cast<int>(i));
                std::string displayTitle = cleanQuestTitleForUi(q.title, q.questId);
                std::string rowText = displayTitle + (q.complete ? " [Ready]" : "");

                float rowH = 24.0f;
                float rowW = ImGui::GetContentRegionAvail().x;
                if (rowW < 1.0f) rowW = 1.0f;
                bool clicked = ImGui::InvisibleButton("questRowBtn", ImVec2(rowW, rowH));
                bool hovered = ImGui::IsItemHovered();
                // Scroll to selected quest on the first frame after openAndSelectQuest()
                if (selected && scrollToSelected_) {
                    ImGui::SetScrollHereY(0.5f);
                    scrollToSelected_ = false;
                }

                ImVec2 rowMin = ImGui::GetItemRectMin();
                ImVec2 rowMax = ImGui::GetItemRectMax();
                ImDrawList* draw = ImGui::GetWindowDrawList();
                if (selected || hovered) {
                    ImU32 bg = selected ? IM_COL32(75, 95, 120, 190) : IM_COL32(60, 60, 60, 120);
                    draw->AddRectFilled(rowMin, rowMax, bg, 3.0f);
                }

                ImU32 txt = q.complete ? IM_COL32(120, 255, 120, 255) : IM_COL32(230, 230, 230, 255);
                draw->AddText(ImVec2(rowMin.x + 8.0f, rowMin.y + 4.0f), txt, rowText.c_str());

                if (clicked) {
                    selectedIndex = static_cast<int>(i);
                    if (q.objectives.empty()) {
                        if (!questDetailQueryNoResponse_.count(q.questId) &&
                            gameHandler.requestQuestQuery(q.questId)) {
                            lastDetailRequestQuestId_ = q.questId;
                            lastDetailRequestAt_ = ImGui::GetTime();
                        }
                    } else if (lastDetailRequestQuestId_ == q.questId) {
                        lastDetailRequestQuestId_ = 0;
                        questDetailQueryNoResponse_.erase(q.questId);
                    }
                }

                // Right-click context menu on quest row
                if (ImGui::BeginPopupContextItem("QuestRowCtx")) {
                    selectedIndex = static_cast<int>(i); // select on right-click too
                    ImGui::TextDisabled("%s", displayTitle.c_str());
                    ImGui::Separator();
                    bool tracked = gameHandler.isQuestTracked(q.questId);
                    if (ImGui::MenuItem(tracked ? "Untrack" : "Track")) {
                        gameHandler.setQuestTracked(q.questId, !tracked);
                    }
                    if (gameHandler.isInGroup() && !q.complete) {
                        if (ImGui::MenuItem("Share Quest")) {
                            gameHandler.shareQuestWithParty(q.questId);
                        }
                    }
                    // A quest that is complete but not yet turned in stays abandonable,
                    // matching retail: only the reward hand-in is gated on completion.
                    ImGui::Separator();
                    if (ImGui::MenuItem("Abandon Quest")) {
                        gameHandler.abandonQuest(q.questId);
                        gameHandler.setQuestTracked(q.questId, false);
                        selectedIndex = -1;
                    }
                    ImGui::EndPopup();
                }

                ImGui::PopID();
            }
            } // zone groups
            if (visibleQuestCount == 0) {
                ImGui::Spacing();
                if (filterLower[0] || questFilterMode_ != 0)
                    ImGui::TextDisabled("No quests match the filter.");
                else
                    ImGui::TextDisabled("No active quests.");
            }
            ImGui::EndChild();

            ImGui::SameLine();
            ImGui::BeginChild("QuestDetailsPane", ImVec2(0, 0), true);

            // Details panel for selected quest
            if (selectedIndex >= 0 && selectedIndex < static_cast<int>(quests.size())) {
                const auto& sel = quests[static_cast<size_t>(selectedIndex)];
                std::string selectedTitle = cleanQuestTitleForUi(sel.title, sel.questId);
                ImGui::TextWrapped("%s", selectedTitle.c_str());
                ImGui::TextColored(sel.complete ? ImVec4(0.45f, 1.0f, 0.45f, 1.0f) : ImVec4(1.0f, 0.84f, 0.2f, 1.0f),
                                   "%s", sel.complete ? "Ready to turn in" : "In progress");
                ImGui::SameLine();
                ImGui::TextDisabled("(Quest #%u)", sel.questId);
                ImGui::Separator();

                if (sel.objectives.empty()) {
                    bool noResponse = questDetailQueryNoResponse_.count(sel.questId) > 0;
                    bool pending = noResponse ? false : gameHandler.isQuestQueryPending(sel.questId);
                    const bool requestTimedOut =
                        (lastDetailRequestQuestId_ == sel.questId) &&
                        ((ImGui::GetTime() - lastDetailRequestAt_) > 5.0);
                    if (lastDetailRequestQuestId_ == sel.questId && !pending) {
                        lastDetailRequestQuestId_ = 0;
                        questDetailQueryNoResponse_.erase(sel.questId);
                    } else if (requestTimedOut) {
                        lastDetailRequestQuestId_ = 0;
                        pending = false;
                        questDetailQueryNoResponse_.insert(sel.questId);
                        noResponse = true;
                        gameHandler.clearQuestQueryPending(sel.questId);
                    }
                    if (pending) {
                        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.8f, 1.0f), "Loading quest details...");
                    } else {
                        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.8f, 1.0f), "Quest summary not available yet.");
                    }
                    if (ImGui::Button("Retry Details")) {
                        questDetailQueryNoResponse_.erase(sel.questId);
                        gameHandler.clearQuestQueryPending(sel.questId);
                        if (gameHandler.requestQuestQuery(sel.questId, true)) {
                            lastDetailRequestQuestId_ = sel.questId;
                            lastDetailRequestAt_ = ImGui::GetTime();
                        }
                    }
                } else {
                    if (lastDetailRequestQuestId_ == sel.questId) lastDetailRequestQuestId_ = 0;
                    questDetailQueryNoResponse_.erase(sel.questId);
                    ImGui::TextColored(ImVec4(0.82f, 0.9f, 1.0f, 1.0f), "Summary");
                    std::string processedObjectives = chat_utils::replaceGenderPlaceholders(sel.objectives, gameHandler);
                    float textHeight = ImGui::GetContentRegionAvail().y * 0.45f;
                    if (textHeight < 120.0f) textHeight = 120.0f;
                    ImGui::BeginChild("QuestObjectiveText", ImVec2(0, textHeight), true);
                    ImGui::TextWrapped("%s", processedObjectives.c_str());
                    ImGui::EndChild();
                }

                if (!sel.killCounts.empty() || !sel.itemCounts.empty()) {
                    ImGui::Separator();
                    ImGui::TextColored(ImVec4(0.8f, 0.9f, 1.0f, 1.0f), "Tracked Progress");
                    for (const auto& [entry, progress] : sel.killCounts) {
                        std::string name = gameHandler.getCachedCreatureName(entry);
                        if (name.empty()) {
                            // Game object objective: fall back to GO name cache.
                            const auto* goInfo = gameHandler.getCachedGameObjectInfo(entry);
                            if (goInfo && !goInfo->name.empty()) name = goInfo->name;
                        }
                        if (name.empty()) name = "Unknown (" + std::to_string(entry) + ")";
                        ImGui::BulletText("%s: %u/%u", name.c_str(), progress.first, progress.second);
                    }
                    for (const auto& [itemId, count] : sel.itemCounts) {
                        std::string itemLabel = "Item " + std::to_string(itemId);
                        uint32_t dispId = 0;
                        if (const auto* info = gameHandler.getItemInfo(itemId)) {
                            if (!info->name.empty()) itemLabel = info->name;
                            dispId = info->displayInfoId;
                        } else {
                            gameHandler.ensureItemInfo(itemId);
                        }
                        uint32_t required = 1;
                        auto reqIt = sel.requiredItemCounts.find(itemId);
                        if (reqIt != sel.requiredItemCounts.end()) required = reqIt->second;
                        VkDescriptorSet iconTex = dispId ? invScreen.getItemIcon(dispId) : VK_NULL_HANDLE;
                        const auto* objInfo = gameHandler.getItemInfo(itemId);
                        if (iconTex) {
                            ImGui::Image((ImTextureID)(uintptr_t)iconTex, ImVec2(14, 14));
                            if (objInfo && objInfo->valid && ImGui::IsItemHovered()) {
                                ImGui::BeginTooltip();
                                invScreen.renderItemTooltip(*objInfo);
                                ImGui::EndTooltip();
                            }
                            ImGui::SameLine();
                            ImGui::Text("%s: %u/%u", itemLabel.c_str(), count, required);
                            if (objInfo && objInfo->valid && ImGui::IsItemHovered()) {
                                ImGui::BeginTooltip();
                                invScreen.renderItemTooltip(*objInfo);
                                ImGui::EndTooltip();
                            }
                        } else {
                            ImGui::BulletText("%s: %u/%u", itemLabel.c_str(), count, required);
                            if (objInfo && objInfo->valid && ImGui::IsItemHovered()) {
                                ImGui::BeginTooltip();
                                invScreen.renderItemTooltip(*objInfo);
                                ImGui::EndTooltip();
                            }
                        }
                    }
                }

                // Reward summary
                bool hasAnyReward = (sel.rewardMoney != 0);
                for (const auto& ri : sel.rewardItems)       if (ri.itemId) hasAnyReward = true;
                for (const auto& ri : sel.rewardChoiceItems) if (ri.itemId) hasAnyReward = true;
                if (hasAnyReward) {
                    ImGui::Separator();
                    ImGui::TextColored(ImVec4(1.0f, 0.9f, 0.5f, 1.0f), "Rewards");

                    // Money reward
                    if (sel.rewardMoney > 0) {
                        uint32_t rg = static_cast<uint32_t>(sel.rewardMoney) / 10000;
                        uint32_t rs = static_cast<uint32_t>(sel.rewardMoney % 10000) / 100;
                        uint32_t rc = static_cast<uint32_t>(sel.rewardMoney % 100);
                        renderCoinsText(rg, rs, rc);
                    }

                    // Guaranteed reward items
                    renderRewardItemList("You will receive:", sel.rewardItems,
                                         gameHandler, invScreen);

                    // Choice reward items
                    renderRewardItemList("Choose one of:", sel.rewardChoiceItems,
                                         gameHandler, invScreen);
                }

                // Track / Share / Abandon buttons
                ImGui::Separator();
                bool isTracked = gameHandler.isQuestTracked(sel.questId);
                if (ImGui::Button(isTracked ? "Untrack" : "Track", ImVec2(100.0f, 0.0f))) {
                    gameHandler.setQuestTracked(sel.questId, !isTracked);
                }
                if (gameHandler.isInGroup() && !sel.complete) {
                    ImGui::SameLine();
                    if (ImGui::Button("Share", ImVec2(80.0f, 0.0f))) {
                        gameHandler.shareQuestWithParty(sel.questId);
                    }
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Share this quest with your party");
                }
                // Completed quests stay abandonable until they are turned in.
                ImGui::SameLine();
                if (ImGui::Button("Abandon Quest", ImVec2(150.0f, 0.0f))) {
                    gameHandler.abandonQuest(sel.questId);
                    gameHandler.setQuestTracked(sel.questId, false);
                    selectedIndex = -1;
                }
            } else {
                ImGui::TextColored(ImVec4(0.72f, 0.72f, 0.76f, 1.0f), "Select a quest to view details.");
            }
            ImGui::EndChild();
        }
        ImGui::EndChild();

        ImGui::Separator();
        float closeW = ImGui::GetContentRegionAvail().x;
        if (closeW < 220.0f) closeW = 220.0f;
        if (ImGui::Button("Close Quest Log", ImVec2(closeW, 34.0f))) {
            stillOpen = false;
        }
    }
    ImGui::End();

    if (!stillOpen) {
        open = false;
    }
}

}} // namespace wowee::ui
