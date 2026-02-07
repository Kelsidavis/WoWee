#include "ui/character_screen.hpp"
#include <imgui.h>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace wowee { namespace ui {

CharacterScreen::CharacterScreen() {
}

void CharacterScreen::render(game::GameHandler& gameHandler) {
    // Size the window to fill most of the viewport
    ImVec2 vpSize = ImGui::GetMainViewport()->Size;
    ImVec2 winSize(vpSize.x * 0.6f, vpSize.y * 0.7f);
    if (winSize.x < 700.0f) winSize.x = 700.0f;
    if (winSize.y < 500.0f) winSize.y = 500.0f;
    ImGui::SetNextWindowSize(winSize, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(
        ImVec2(vpSize.x * 0.5f, vpSize.y * 0.5f),
        ImGuiCond_FirstUseEver, ImVec2(0.5f, 0.5f));

    ImGui::Begin("Character Selection", nullptr, ImGuiWindowFlags_NoCollapse);

    // Get character list
    const auto& characters = gameHandler.getCharacters();

    // Request character list if not available
    if (characters.empty() && gameHandler.getState() == game::WorldState::READY) {
        ImGui::Text("Loading characters...");
        gameHandler.requestCharacterList();
        ImGui::End();
        return;
    }

    if (characters.empty()) {
        ImGui::Text("No characters available.");
        // Bottom buttons even when empty
        ImGui::Spacing();
        if (ImGui::Button("Back", ImVec2(120, 36))) { if (onBack) onBack(); }
        ImGui::SameLine();
        if (ImGui::Button("Refresh", ImVec2(120, 36))) {
            if (gameHandler.getState() == game::WorldState::READY ||
                gameHandler.getState() == game::WorldState::CHAR_LIST_RECEIVED) {
                gameHandler.requestCharacterList();
                setStatus("Refreshing character list...");
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Create Character", ImVec2(160, 36))) { if (onCreateCharacter) onCreateCharacter(); }
        ImGui::End();
        return;
    }

    // Restore last-selected character (once per screen visit)
    if (!restoredLastCharacter) {
        uint64_t lastGuid = loadLastCharacter();
        if (lastGuid != 0) {
            for (size_t i = 0; i < characters.size(); ++i) {
                if (characters[i].guid == lastGuid) {
                    selectedCharacterIndex = static_cast<int>(i);
                    selectedCharacterGuid = lastGuid;
                    break;
                }
            }
        }
        // Fall back to first character if nothing matched
        if (selectedCharacterIndex < 0) {
            selectedCharacterIndex = 0;
            selectedCharacterGuid = characters[0].guid;
        }
        restoredLastCharacter = true;
    }

    // Status message
    if (!statusMessage.empty()) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.3f, 1.0f, 0.3f, 1.0f));
        ImGui::TextWrapped("%s", statusMessage.c_str());
        ImGui::PopStyleColor();
        ImGui::Spacing();
    }

    // ── Two-column layout: character list (left) | details (right) ──
    float availW = ImGui::GetContentRegionAvail().x;
    float detailPanelW = 260.0f;
    float listW = availW - detailPanelW - ImGui::GetStyle().ItemSpacing.x;
    if (listW < 300.0f) { listW = availW; detailPanelW = 0.0f; }

    float listH = ImGui::GetContentRegionAvail().y - 50.0f; // reserve bottom row for buttons

    // ── Left: Character list ──
    ImGui::BeginChild("CharList", ImVec2(listW, listH), true);
    ImGui::Text("Characters");
    ImGui::Separator();

    if (ImGui::BeginTable("CharactersTable", 5,
            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
            ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Name",  ImGuiTableColumnFlags_WidthStretch, 2.0f);
        ImGui::TableSetupColumn("Level", ImGuiTableColumnFlags_WidthFixed, 45.0f);
        ImGui::TableSetupColumn("Race",  ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("Class", ImGuiTableColumnFlags_WidthStretch, 1.2f);
        ImGui::TableSetupColumn("Zone",  ImGuiTableColumnFlags_WidthFixed, 55.0f);
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableHeadersRow();

        for (size_t i = 0; i < characters.size(); ++i) {
            const auto& character = characters[i];
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);

            bool isSelected = (selectedCharacterIndex == static_cast<int>(i));
            ImVec4 factionColor = getFactionColor(character.race);
            ImGui::PushStyleColor(ImGuiCol_Text, factionColor);

            ImGui::PushID(static_cast<int>(i));
            if (ImGui::Selectable(character.name.c_str(), isSelected,
                    ImGuiSelectableFlags_SpanAllColumns)) {
                selectedCharacterIndex = static_cast<int>(i);
                selectedCharacterGuid = character.guid;
                saveLastCharacter(character.guid);
            }

            // Double-click to enter world
            if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
                selectedCharacterIndex = static_cast<int>(i);
                selectedCharacterGuid = character.guid;
                saveLastCharacter(character.guid);
                characterSelected = true;
                gameHandler.selectCharacter(character.guid);
                if (onCharacterSelected) onCharacterSelected(character.guid);
            }
            ImGui::PopID();
            ImGui::PopStyleColor();

            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%d", character.level);

            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%s", game::getRaceName(character.race));

            ImGui::TableSetColumnIndex(3);
            ImGui::Text("%s", game::getClassName(character.characterClass));

            ImGui::TableSetColumnIndex(4);
            ImGui::Text("%d", character.zoneId);
        }

        ImGui::EndTable();
    }
    ImGui::EndChild();

    // ── Right: Details panel ──
    if (detailPanelW > 0.0f &&
        selectedCharacterIndex >= 0 &&
        selectedCharacterIndex < static_cast<int>(characters.size())) {

        const auto& character = characters[selectedCharacterIndex];

        ImGui::SameLine();
        ImGui::BeginChild("CharDetails", ImVec2(detailPanelW, listH), true);

        ImGui::TextColored(getFactionColor(character.race), "%s", character.name.c_str());
        ImGui::Separator();
        ImGui::Spacing();

        ImGui::Text("Level %d", character.level);
        ImGui::Text("%s", game::getRaceName(character.race));
        ImGui::Text("%s", game::getClassName(character.characterClass));
        ImGui::Text("%s", game::getGenderName(character.gender));
        ImGui::Spacing();
        ImGui::Text("Map %d, Zone %d", character.mapId, character.zoneId);

        if (character.hasGuild()) {
            ImGui::Text("Guild ID: %d", character.guildId);
        } else {
            ImGui::TextDisabled("No Guild");
        }

        if (character.hasPet()) {
            ImGui::Spacing();
            ImGui::Text("Pet Lv%d (Family %d)", character.pet.level, character.pet.family);
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // Enter World button — full width
        float btnW = ImGui::GetContentRegionAvail().x;
        if (ImGui::Button("Enter World", ImVec2(btnW, 44))) {
            characterSelected = true;
            saveLastCharacter(character.guid);
            std::stringstream ss;
            ss << "Entering world with " << character.name << "...";
            setStatus(ss.str());
            gameHandler.selectCharacter(character.guid);
            if (onCharacterSelected) onCharacterSelected(character.guid);
        }

        ImGui::Spacing();

        // Delete
        if (!confirmDelete) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.1f, 0.1f, 1.0f));
            if (ImGui::Button("Delete Character", ImVec2(btnW, 36))) {
                confirmDelete = true;
            }
            ImGui::PopStyleColor();
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.8f, 0.0f, 0.0f, 1.0f));
            if (ImGui::Button("Confirm Delete?", ImVec2(btnW, 36))) {
                if (onDeleteCharacter) onDeleteCharacter(character.guid);
                confirmDelete = false;
                selectedCharacterIndex = -1;
                selectedCharacterGuid = 0;
            }
            ImGui::PopStyleColor();
            if (ImGui::Button("Cancel", ImVec2(btnW, 30))) {
                confirmDelete = false;
            }
        }

        ImGui::EndChild();
    }

    // ── Bottom button row ──
    ImGui::Spacing();
    if (ImGui::Button("Back", ImVec2(120, 36))) { if (onBack) onBack(); }
    ImGui::SameLine();
    if (ImGui::Button("Refresh", ImVec2(120, 36))) {
        if (gameHandler.getState() == game::WorldState::READY ||
            gameHandler.getState() == game::WorldState::CHAR_LIST_RECEIVED) {
            gameHandler.requestCharacterList();
            setStatus("Refreshing character list...");
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Create Character", ImVec2(160, 36))) {
        if (onCreateCharacter) onCreateCharacter();
    }

    ImGui::End();
}

void CharacterScreen::setStatus(const std::string& message) {
    statusMessage = message;
}

ImVec4 CharacterScreen::getFactionColor(game::Race race) const {
    // Alliance races: blue
    if (race == game::Race::HUMAN ||
        race == game::Race::DWARF ||
        race == game::Race::NIGHT_ELF ||
        race == game::Race::GNOME ||
        race == game::Race::DRAENEI) {
        return ImVec4(0.3f, 0.5f, 1.0f, 1.0f);
    }

    // Horde races: red
    if (race == game::Race::ORC ||
        race == game::Race::UNDEAD ||
        race == game::Race::TAUREN ||
        race == game::Race::TROLL ||
        race == game::Race::BLOOD_ELF) {
        return ImVec4(1.0f, 0.3f, 0.3f, 1.0f);
    }

    return ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
}

std::string CharacterScreen::getConfigDir() {
#ifdef _WIN32
    const char* appdata = std::getenv("APPDATA");
    return appdata ? std::string(appdata) + "\\wowee" : ".";
#else
    const char* home = std::getenv("HOME");
    return home ? std::string(home) + "/.wowee" : ".";
#endif
}

void CharacterScreen::saveLastCharacter(uint64_t guid) {
    std::string dir = getConfigDir();
    std::filesystem::create_directories(dir);
    std::ofstream f(dir + "/last_character.cfg");
    if (f) f << guid;
}

uint64_t CharacterScreen::loadLastCharacter() {
    std::string path = getConfigDir() + "/last_character.cfg";
    std::ifstream f(path);
    uint64_t guid = 0;
    if (f) f >> guid;
    return guid;
}

}} // namespace wowee::ui
