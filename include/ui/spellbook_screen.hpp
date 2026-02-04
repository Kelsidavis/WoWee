#pragma once

#include "game/game_handler.hpp"
#include <imgui.h>
#include <string>
#include <unordered_map>

namespace wowee {

namespace pipeline { class AssetManager; }

namespace ui {

class SpellbookScreen {
public:
    void render(game::GameHandler& gameHandler, pipeline::AssetManager* assetManager);
    bool isOpen() const { return open; }
    void toggle() { open = !open; }
    void setOpen(bool o) { open = o; }

private:
    bool open = false;
    bool pKeyWasDown = false;

    // Spell name cache (loaded from Spell.dbc)
    bool dbcLoaded = false;
    bool dbcLoadAttempted = false;
    std::unordered_map<uint32_t, std::string> spellNames;

    // Action bar assignment
    int assigningSlot = -1;  // Which action bar slot is being assigned (-1 = none)

    void loadSpellDBC(pipeline::AssetManager* assetManager);
    std::string getSpellName(uint32_t spellId) const;
};

} // namespace ui
} // namespace wowee
