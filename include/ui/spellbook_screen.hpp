#pragma once

#include "game/game_handler.hpp"
#include <GL/glew.h>
#include <imgui.h>
#include <string>
#include <vector>
#include <unordered_map>

namespace wowee {

namespace pipeline { class AssetManager; }

namespace ui {

struct SpellInfo {
    uint32_t spellId = 0;
    std::string name;
    std::string rank;
    uint32_t iconId = 0;       // SpellIconID
    uint32_t attributes = 0;   // Spell attributes (field 75)
    bool isPassive() const { return (attributes & 0x40) != 0; }
};

enum class SpellTab { GENERAL, ACTIVE, PASSIVE };

class SpellbookScreen {
public:
    void render(game::GameHandler& gameHandler, pipeline::AssetManager* assetManager);
    bool isOpen() const { return open; }
    void toggle() { open = !open; }
    void setOpen(bool o) { open = o; }

private:
    bool open = false;
    bool pKeyWasDown = false;

    // Spell data (loaded from Spell.dbc)
    bool dbcLoaded = false;
    bool dbcLoadAttempted = false;
    std::unordered_map<uint32_t, SpellInfo> spellData;

    // Icon data (loaded from SpellIcon.dbc)
    bool iconDbLoaded = false;
    std::unordered_map<uint32_t, std::string> spellIconPaths; // SpellIconID -> path
    std::unordered_map<uint32_t, GLuint> spellIconCache;      // SpellIconID -> GL texture

    // Categorized spell lists (rebuilt when spell list changes)
    std::vector<const SpellInfo*> generalSpells;
    std::vector<const SpellInfo*> activeSpells;
    std::vector<const SpellInfo*> passiveSpells;
    size_t lastKnownSpellCount = 0;

    // Tab state
    SpellTab currentTab = SpellTab::GENERAL;

    // Action bar assignment
    int assigningSlot = -1;

    void loadSpellDBC(pipeline::AssetManager* assetManager);
    void loadSpellIconDBC(pipeline::AssetManager* assetManager);
    void categorizeSpells(const std::vector<uint32_t>& knownSpells);
    GLuint getSpellIcon(uint32_t iconId, pipeline::AssetManager* assetManager);
    const SpellInfo* getSpellInfo(uint32_t spellId) const;
};

} // namespace ui
} // namespace wowee
