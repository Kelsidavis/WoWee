#pragma once

#include "game/game_handler.hpp"
#include <imgui.h>
#include <cstdint>
#include <unordered_set>

namespace wowee { namespace ui {

class QuestLogScreen {
public:
    void render(game::GameHandler& gameHandler);
    bool isOpen() const { return open; }
    void toggle() { open = !open; }
    void setOpen(bool o) { open = o; }

private:
    bool open = false;
    bool lKeyWasDown = false;
    int selectedIndex = -1;
    uint32_t lastDetailRequestQuestId_ = 0;
    double lastDetailRequestAt_ = 0.0;
    std::unordered_set<uint32_t> questDetailQueryNoResponse_;
};

}} // namespace wowee::ui
