#pragma once

#include "game/game_handler.hpp"
#include <imgui.h>
#include <cstdint>
#include <unordered_set>

namespace wowee { namespace ui {

class InventoryScreen;

class QuestLogScreen {
public:
    void render(game::GameHandler& gameHandler, InventoryScreen& invScreen);
    [[nodiscard]] bool isOpen() const { return open; }
    void toggle() { open = !open; }
    void setOpen(bool o) { open = o; }
    // Open the log and scroll to the given quest (by questId)
    void openAndSelectQuest(uint32_t questId) {
        open = true;
        pendingSelectQuestId_ = questId;
        scrollToSelected_ = true;
    }

private:
    bool open = false;
    bool lKeyWasDown = false;
    int selectedIndex = -1;
    uint32_t pendingSelectQuestId_ = 0;  // non-zero: select this quest on next render
    bool scrollToSelected_ = false;       // true: call SetScrollHereY once after selection
    uint32_t lastDetailRequestQuestId_ = 0;
    double lastDetailRequestAt_ = 0.0;
    std::unordered_set<uint32_t> questDetailQueryNoResponse_;
    // Search / filter
    char questSearchFilter_[64] = {};
    // 0=all, 1=active only, 2=complete only
    int questFilterMode_ = 0;
};

}} // namespace wowee::ui
