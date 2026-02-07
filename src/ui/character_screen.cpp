#include "ui/character_screen.hpp"
#include <imgui.h>
#include <iomanip>
#include <sstream>

namespace wowee { namespace ui {

CharacterScreen::CharacterScreen() {
}

void CharacterScreen::render(game::GameHandler& gameHandler) {
    ImGui::SetNextWindowSize(ImVec2(800, 600), ImGuiCond_FirstUseEver);
    ImGui::Begin("Character Selection", nullptr, ImGuiWindowFlags_NoCollapse);

    ImGui::Text("Select a Character");
    ImGui::Separator();
    ImGui::Spacing();

    // Status message
    if (!statusMessage.empty()) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.3f, 1.0f, 0.3f, 1.0f));
        ImGui::TextWrapped("%s", statusMessage.c_str());
        ImGui::PopStyleColor();
        ImGui::Spacing();
    }

    // Get character list
    const auto& characters = gameHandler.getCharacters();

    // Request character list if not available
    if (characters.empty() && gameHandler.getState() == game::WorldState::READY) {
        ImGui::Text("Loading characters...");
        gameHandler.requestCharacterList();
    } else if (characters.empty()) {
        ImGui::Text("No characters available.");
    } else {
        // Auto-highlight the first character if none selected yet
        if (selectedCharacterIndex < 0 && !characters.empty()) {
            selectedCharacterIndex = 0;
            selectedCharacterGuid = characters[0].guid;
        }

        // Character table
        if (ImGui::BeginTable("CharactersTable", 6, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
            ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("Level", ImGuiTableColumnFlags_WidthFixed, 50.0f);
            ImGui::TableSetupColumn("Race", ImGuiTableColumnFlags_WidthFixed, 100.0f);
            ImGui::TableSetupColumn("Class", ImGuiTableColumnFlags_WidthFixed, 120.0f);
            ImGui::TableSetupColumn("Zone", ImGuiTableColumnFlags_WidthFixed, 80.0f);
            ImGui::TableSetupColumn("Guild", ImGuiTableColumnFlags_WidthFixed, 80.0f);
            ImGui::TableHeadersRow();

            for (size_t i = 0; i < characters.size(); ++i) {
                const auto& character = characters[i];

                ImGui::TableNextRow();

                // Name column (selectable)
                ImGui::TableSetColumnIndex(0);
                bool isSelected = (selectedCharacterIndex == static_cast<int>(i));

                // Apply faction color to character name
                ImVec4 factionColor = getFactionColor(character.race);
                ImGui::PushStyleColor(ImGuiCol_Text, factionColor);

                if (ImGui::Selectable(character.name.c_str(), isSelected, ImGuiSelectableFlags_SpanAllColumns)) {
                    selectedCharacterIndex = static_cast<int>(i);
                    selectedCharacterGuid = character.guid;
                }

                ImGui::PopStyleColor();

                // Level column
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%d", character.level);

                // Race column
                ImGui::TableSetColumnIndex(2);
                ImGui::Text("%s", game::getRaceName(character.race));

                // Class column
                ImGui::TableSetColumnIndex(3);
                ImGui::Text("%s", game::getClassName(character.characterClass));

                // Zone column
                ImGui::TableSetColumnIndex(4);
                ImGui::Text("%d", character.zoneId);

                // Guild column
                ImGui::TableSetColumnIndex(5);
                if (character.hasGuild()) {
                    ImGui::Text("Yes");
                } else {
                    ImGui::TextDisabled("No");
                }
            }

            ImGui::EndTable();
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // Selected character details
        if (selectedCharacterIndex >= 0 && selectedCharacterIndex < static_cast<int>(characters.size())) {
            const auto& character = characters[selectedCharacterIndex];

            ImGui::Text("Character Details:");
            ImGui::Separator();

            ImGui::Columns(2, nullptr, false);

            // Left column
            ImGui::Text("Name:");
            ImGui::Text("Level:");
            ImGui::Text("Race:");
            ImGui::Text("Class:");
            ImGui::Text("Gender:");
            ImGui::Text("Location:");
            ImGui::Text("Guild:");
            if (character.hasPet()) {
                ImGui::Text("Pet:");
            }

            ImGui::NextColumn();

            // Right column
            ImGui::TextColored(getFactionColor(character.race), "%s", character.name.c_str());
            ImGui::Text("%d", character.level);
            ImGui::Text("%s", game::getRaceName(character.race));
            ImGui::Text("%s", game::getClassName(character.characterClass));
            ImGui::Text("%s", game::getGenderName(character.gender));
            ImGui::Text("Map %d, Zone %d", character.mapId, character.zoneId);
            if (character.hasGuild()) {
                ImGui::Text("Guild ID: %d", character.guildId);
            } else {
                ImGui::TextDisabled("None");
            }
            if (character.hasPet()) {
                ImGui::Text("Level %d (Family %d)", character.pet.level, character.pet.family);
            }

            ImGui::Columns(1);

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            // Enter World button
            if (ImGui::Button("Enter World", ImVec2(150, 40))) {
                characterSelected = true;
                std::stringstream ss;
                ss << "Entering world with " << character.name << "...";
                setStatus(ss.str());

                // Send CMSG_PLAYER_LOGIN to server
                gameHandler.selectCharacter(character.guid);

                // Call callback
                if (onCharacterSelected) {
                    onCharacterSelected(character.guid);
                }
            }

            ImGui::SameLine();

            // Delete Character button
            if (!confirmDelete) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.1f, 0.1f, 1.0f));
                if (ImGui::Button("Delete Character", ImVec2(150, 40))) {
                    confirmDelete = true;
                }
                ImGui::PopStyleColor();
            } else {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.8f, 0.0f, 0.0f, 1.0f));
                if (ImGui::Button("Confirm Delete?", ImVec2(150, 40))) {
                    if (onDeleteCharacter) {
                        onDeleteCharacter(character.guid);
                    }
                    confirmDelete = false;
                    selectedCharacterIndex = -1;
                    selectedCharacterGuid = 0;
                }
                ImGui::PopStyleColor();
                ImGui::SameLine();
                if (ImGui::Button("Cancel", ImVec2(80, 40))) {
                    confirmDelete = false;
                }
            }
        }
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Back/Refresh/Create buttons
    if (ImGui::Button("Back", ImVec2(120, 0))) {
        if (onBack) {
            onBack();
        }
    }

    ImGui::SameLine();

    if (ImGui::Button("Refresh", ImVec2(120, 0))) {
        if (gameHandler.getState() == game::WorldState::READY ||
            gameHandler.getState() == game::WorldState::CHAR_LIST_RECEIVED) {
            gameHandler.requestCharacterList();
            setStatus("Refreshing character list...");
        }
    }

    ImGui::SameLine();

    if (ImGui::Button("Create Character", ImVec2(150, 0))) {
        if (onCreateCharacter) {
            onCreateCharacter();
        }
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
        return ImVec4(0.3f, 0.5f, 1.0f, 1.0f);  // Blue
    }

    // Horde races: red
    if (race == game::Race::ORC ||
        race == game::Race::UNDEAD ||
        race == game::Race::TAUREN ||
        race == game::Race::TROLL ||
        race == game::Race::BLOOD_ELF) {
        return ImVec4(1.0f, 0.3f, 0.3f, 1.0f);  // Red
    }

    // Default: white
    return ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
}

}} // namespace wowee::ui
