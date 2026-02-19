#include "ui/quest_log_screen.hpp"
#include "core/application.hpp"
#include "core/input.hpp"
#include <imgui.h>
#include <cctype>

namespace wowee { namespace ui {

namespace {
// Helper function to replace gender placeholders, pronouns, and name
std::string replaceGenderPlaceholders(const std::string& text, game::GameHandler& gameHandler) {
    game::Gender gender = game::Gender::NONBINARY;
    std::string playerName = "Adventurer";
    const auto* character = gameHandler.getActiveCharacter();
    if (character) {
        gender = character->gender;
        if (!character->name.empty()) {
            playerName = character->name;
        }
    }
    game::Pronouns pronouns = game::Pronouns::forGender(gender);

    std::string result = text;

    auto trim = [](std::string& s) {
        const char* ws = " \t\n\r";
        size_t start = s.find_first_not_of(ws);
        if (start == std::string::npos) { s.clear(); return; }
        size_t end = s.find_last_not_of(ws);
        s = s.substr(start, end - start + 1);
    };

    // Replace $g placeholders
    size_t pos = 0;
    pos = 0;
    while ((pos = result.find('$', pos)) != std::string::npos) {
        if (pos + 1 >= result.length()) break;
        char marker = result[pos + 1];
        if (marker != 'g' && marker != 'G') { pos++; continue; }

        size_t endPos = result.find(';', pos);
        if (endPos == std::string::npos) { pos += 2; continue; }

        std::string placeholder = result.substr(pos + 2, endPos - pos - 2);

        std::vector<std::string> parts;
        size_t start = 0;
        size_t colonPos;
        while ((colonPos = placeholder.find(':', start)) != std::string::npos) {
            std::string part = placeholder.substr(start, colonPos - start);
            trim(part);
            parts.push_back(part);
            start = colonPos + 1;
        }
        std::string lastPart = placeholder.substr(start);
        trim(lastPart);
        parts.push_back(lastPart);

        std::string replacement;
        if (parts.size() >= 3) {
            switch (gender) {
                case game::Gender::MALE: replacement = parts[0]; break;
                case game::Gender::FEMALE: replacement = parts[1]; break;
                case game::Gender::NONBINARY: replacement = parts[2]; break;
            }
        } else if (parts.size() >= 2) {
            switch (gender) {
                case game::Gender::MALE: replacement = parts[0]; break;
                case game::Gender::FEMALE: replacement = parts[1]; break;
                case game::Gender::NONBINARY:
                    replacement = parts[0].length() <= parts[1].length() ? parts[0] : parts[1];
                    break;
            }
        } else {
            pos = endPos + 1;
            continue;
        }

        result.replace(pos, endPos - pos + 1, replacement);
        pos += replacement.length();
    }

    // Replace simple placeholders
    pos = 0;
    while ((pos = result.find('$', pos)) != std::string::npos) {
        if (pos + 1 >= result.length()) break;

        char code = result[pos + 1];
        std::string replacement;

        switch (code) {
            case 'n': case 'N': replacement = playerName; break;
            case 'p': replacement = pronouns.subject; break;
            case 'o': replacement = pronouns.object; break;
            case 's': replacement = pronouns.possessive; break;
            case 'S': replacement = pronouns.possessiveP; break;
            case 'r': replacement = pronouns.object; break;
            case 'b': replacement = "\n"; break;
            case 'g': case 'G': pos++; continue;
            default: pos++; continue;
        }

        result.replace(pos, 2, replacement);
        pos += replacement.length();
    }

    // WoW markup linebreak token
    pos = 0;
    while ((pos = result.find("|n", pos)) != std::string::npos) {
        result.replace(pos, 2, "\n");
        pos += 1;
    }

    return result;
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

    int alphaCount = 0;
    int spaceCount = 0;
    int shortWordCount = 0;
    int wordCount = 0;
    int currentWordLen = 0;
    for (char c : s) {
        if (std::isalpha(static_cast<unsigned char>(c))) alphaCount++;
        if (c == ' ') {
            spaceCount++;
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

void QuestLogScreen::render(game::GameHandler& gameHandler) {
    // L key toggle (edge-triggered)
    ImGuiIO& io = ImGui::GetIO();
    bool lDown = !io.WantTextInput && core::Input::getInstance().isKeyPressed(SDL_SCANCODE_L);
    if (lDown && !lKeyWasDown) {
        open = !open;
    }
    lKeyWasDown = lDown;

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

        ImGui::TextColored(ImVec4(0.95f, 0.85f, 0.35f, 1.0f), "Active: %d", activeCount);
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.45f, 0.95f, 0.45f, 1.0f), "Ready: %d", completeCount);
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
            for (size_t i = 0; i < quests.size(); i++) {
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
                        if (gameHandler.requestQuestQuery(q.questId)) {
                            lastDetailRequestQuestId_ = q.questId;
                        }
                    } else if (lastDetailRequestQuestId_ == q.questId) {
                        lastDetailRequestQuestId_ = 0;
                    }
                }
                ImGui::PopID();
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
                    if (lastDetailRequestQuestId_ != sel.questId) {
                        if (gameHandler.requestQuestQuery(sel.questId)) {
                            lastDetailRequestQuestId_ = sel.questId;
                        }
                    }
                    if (lastDetailRequestQuestId_ == sel.questId) {
                        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.8f, 1.0f), "Loading quest details...");
                    } else {
                        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.8f, 1.0f), "Quest summary not available yet.");
                    }
                    if (ImGui::Button("Retry Details")) {
                        if (gameHandler.requestQuestQuery(sel.questId, true)) {
                            lastDetailRequestQuestId_ = sel.questId;
                        }
                    }
                } else {
                    if (lastDetailRequestQuestId_ == sel.questId) lastDetailRequestQuestId_ = 0;
                    ImGui::TextColored(ImVec4(0.82f, 0.9f, 1.0f, 1.0f), "Summary");
                    std::string processedObjectives = replaceGenderPlaceholders(sel.objectives, gameHandler);
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
                        ImGui::BulletText("Kill %u: %u/%u", entry, progress.first, progress.second);
                    }
                    for (const auto& [itemId, count] : sel.itemCounts) {
                        std::string itemLabel = "Item " + std::to_string(itemId);
                        if (const auto* info = gameHandler.getItemInfo(itemId)) {
                            if (!info->name.empty()) itemLabel = info->name;
                        }
                        ImGui::BulletText("%s: %u", itemLabel.c_str(), count);
                    }
                }

                // Abandon button
                if (!sel.complete) {
                    ImGui::Separator();
                    if (ImGui::Button("Abandon Quest", ImVec2(150.0f, 0.0f))) {
                        gameHandler.abandonQuest(sel.questId);
                        selectedIndex = -1;
                    }
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
