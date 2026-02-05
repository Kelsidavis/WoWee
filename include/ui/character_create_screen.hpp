#pragma once

#include "game/character.hpp"
#include "game/world_packets.hpp"
#include <imgui.h>
#include <string>
#include <functional>
#include <vector>

namespace wowee {
namespace game { class GameHandler; }

namespace ui {

class CharacterCreateScreen {
public:
    CharacterCreateScreen();

    void render(game::GameHandler& gameHandler);
    void setOnCreate(std::function<void(const game::CharCreateData&)> cb) { onCreate = std::move(cb); }
    void setOnCancel(std::function<void()> cb) { onCancel = std::move(cb); }
    void setStatus(const std::string& msg, bool isError = false);
    void reset();

private:
    char nameBuffer[13] = {};  // WoW max name = 12 chars + null
    int raceIndex = 0;
    int classIndex = 0;
    int genderIndex = 0;
    int skin = 0, face = 0, hairStyle = 0, hairColor = 0, facialHair = 0;
    std::string statusMessage;
    bool statusIsError = false;

    std::vector<game::Class> availableClasses;
    void updateAvailableClasses();

    std::function<void(const game::CharCreateData&)> onCreate;
    std::function<void()> onCancel;
};

} // namespace ui
} // namespace wowee
