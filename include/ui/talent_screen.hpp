#pragma once

#include "game/game_handler.hpp"
#include <imgui.h>

namespace wowee {
namespace ui {

class TalentScreen {
public:
    void render(game::GameHandler& gameHandler);
    bool isOpen() const { return open; }
    void toggle() { open = !open; }
    void setOpen(bool o) { open = o; }

private:
    bool open = false;
    bool nKeyWasDown = false;
};

} // namespace ui
} // namespace wowee
