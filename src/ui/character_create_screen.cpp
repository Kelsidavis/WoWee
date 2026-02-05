#include "ui/character_create_screen.hpp"
#include "game/game_handler.hpp"
#include <imgui.h>
#include <cstring>

namespace wowee {
namespace ui {

static const game::Race allRaces[] = {
    // Alliance
    game::Race::HUMAN, game::Race::DWARF, game::Race::NIGHT_ELF,
    game::Race::GNOME, game::Race::DRAENEI,
    // Horde
    game::Race::ORC, game::Race::UNDEAD, game::Race::TAUREN,
    game::Race::TROLL, game::Race::BLOOD_ELF,
};
static constexpr int RACE_COUNT = 10;
static constexpr int ALLIANCE_COUNT = 5;

static const game::Class allClasses[] = {
    game::Class::WARRIOR, game::Class::PALADIN, game::Class::HUNTER,
    game::Class::ROGUE, game::Class::PRIEST, game::Class::DEATH_KNIGHT,
    game::Class::SHAMAN, game::Class::MAGE, game::Class::WARLOCK,
    game::Class::DRUID,
};

CharacterCreateScreen::CharacterCreateScreen() {
    reset();
}

void CharacterCreateScreen::reset() {
    std::memset(nameBuffer, 0, sizeof(nameBuffer));
    raceIndex = 0;
    classIndex = 0;
    genderIndex = 0;
    skin = 0;
    face = 0;
    hairStyle = 0;
    hairColor = 0;
    facialHair = 0;
    statusMessage.clear();
    statusIsError = false;
    updateAvailableClasses();
}

void CharacterCreateScreen::setStatus(const std::string& msg, bool isError) {
    statusMessage = msg;
    statusIsError = isError;
}

void CharacterCreateScreen::updateAvailableClasses() {
    availableClasses.clear();
    game::Race race = allRaces[raceIndex];
    for (auto cls : allClasses) {
        if (game::isValidRaceClassCombo(race, cls)) {
            availableClasses.push_back(cls);
        }
    }
    // Clamp class index
    if (classIndex >= static_cast<int>(availableClasses.size())) {
        classIndex = 0;
    }
}

void CharacterCreateScreen::render(game::GameHandler& /*gameHandler*/) {
    ImVec2 displaySize = ImGui::GetIO().DisplaySize;
    ImVec2 winSize(600, 520);
    ImGui::SetNextWindowSize(winSize, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2((displaySize.x - winSize.x) * 0.5f,
                                    (displaySize.y - winSize.y) * 0.5f),
                            ImGuiCond_FirstUseEver);

    ImGui::Begin("Create Character", nullptr, ImGuiWindowFlags_NoCollapse);

    ImGui::Text("Create Character");
    ImGui::Separator();
    ImGui::Spacing();

    // Name input
    ImGui::Text("Name:");
    ImGui::SameLine(100);
    ImGui::SetNextItemWidth(200);
    ImGui::InputText("##name", nameBuffer, sizeof(nameBuffer));

    ImGui::Spacing();

    // Race selection
    ImGui::Text("Race:");
    ImGui::SameLine(100);
    ImGui::BeginGroup();
    ImGui::TextColored(ImVec4(0.3f, 0.5f, 1.0f, 1.0f), "Alliance:");
    ImGui::SameLine();
    for (int i = 0; i < ALLIANCE_COUNT; ++i) {
        if (i > 0) ImGui::SameLine();
        bool selected = (raceIndex == i);
        if (selected) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.3f, 0.5f, 1.0f, 0.8f));
        if (ImGui::SmallButton(game::getRaceName(allRaces[i]))) {
            if (raceIndex != i) {
                raceIndex = i;
                classIndex = 0;
                skin = face = hairStyle = hairColor = facialHair = 0;
                updateAvailableClasses();
            }
        }
        if (selected) ImGui::PopStyleColor();
    }
    ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "Horde:");
    ImGui::SameLine();
    for (int i = ALLIANCE_COUNT; i < RACE_COUNT; ++i) {
        if (i > ALLIANCE_COUNT) ImGui::SameLine();
        bool selected = (raceIndex == i);
        if (selected) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(1.0f, 0.3f, 0.3f, 0.8f));
        if (ImGui::SmallButton(game::getRaceName(allRaces[i]))) {
            if (raceIndex != i) {
                raceIndex = i;
                classIndex = 0;
                skin = face = hairStyle = hairColor = facialHair = 0;
                updateAvailableClasses();
            }
        }
        if (selected) ImGui::PopStyleColor();
    }
    ImGui::EndGroup();

    ImGui::Spacing();

    // Class selection
    ImGui::Text("Class:");
    ImGui::SameLine(100);
    if (!availableClasses.empty()) {
        ImGui::BeginGroup();
        for (int i = 0; i < static_cast<int>(availableClasses.size()); ++i) {
            if (i > 0) ImGui::SameLine();
            bool selected = (classIndex == i);
            if (selected) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.6f, 0.2f, 0.8f));
            if (ImGui::SmallButton(game::getClassName(availableClasses[i]))) {
                classIndex = i;
            }
            if (selected) ImGui::PopStyleColor();
        }
        ImGui::EndGroup();
    }

    ImGui::Spacing();

    // Gender
    ImGui::Text("Gender:");
    ImGui::SameLine(100);
    ImGui::RadioButton("Male", &genderIndex, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Female", &genderIndex, 1);

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Appearance sliders
    game::Race currentRace = allRaces[raceIndex];
    game::Gender currentGender = static_cast<game::Gender>(genderIndex);

    ImGui::Text("Appearance");
    ImGui::Spacing();

    auto slider = [](const char* label, int* val, int maxVal) {
        ImGui::Text("%s", label);
        ImGui::SameLine(120);
        ImGui::SetNextItemWidth(200);
        char id[32];
        snprintf(id, sizeof(id), "##%s", label);
        ImGui::SliderInt(id, val, 0, maxVal);
    };

    slider("Skin",           &skin,      game::getMaxSkin(currentRace, currentGender));
    slider("Face",           &face,      game::getMaxFace(currentRace, currentGender));
    slider("Hair Style",     &hairStyle, game::getMaxHairStyle(currentRace, currentGender));
    slider("Hair Color",     &hairColor, game::getMaxHairColor(currentRace, currentGender));
    slider("Facial Feature", &facialHair, game::getMaxFacialFeature(currentRace, currentGender));

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Status message
    if (!statusMessage.empty()) {
        ImVec4 color = statusIsError ? ImVec4(1.0f, 0.3f, 0.3f, 1.0f) : ImVec4(0.3f, 1.0f, 0.3f, 1.0f);
        ImGui::TextColored(color, "%s", statusMessage.c_str());
        ImGui::Spacing();
    }

    // Buttons
    if (ImGui::Button("Create", ImVec2(150, 35))) {
        std::string name(nameBuffer);
        if (name.empty()) {
            setStatus("Please enter a character name.", true);
        } else if (availableClasses.empty()) {
            setStatus("No valid class for this race.", true);
        } else {
            game::CharCreateData data;
            data.name = name;
            data.race = allRaces[raceIndex];
            data.characterClass = availableClasses[classIndex];
            data.gender = currentGender;
            data.skin = static_cast<uint8_t>(skin);
            data.face = static_cast<uint8_t>(face);
            data.hairStyle = static_cast<uint8_t>(hairStyle);
            data.hairColor = static_cast<uint8_t>(hairColor);
            data.facialHair = static_cast<uint8_t>(facialHair);
            if (onCreate) {
                onCreate(data);
            }
        }
    }

    ImGui::SameLine();

    if (ImGui::Button("Back", ImVec2(150, 35))) {
        if (onCancel) {
            onCancel();
        }
    }

    ImGui::End();
}

} // namespace ui
} // namespace wowee
