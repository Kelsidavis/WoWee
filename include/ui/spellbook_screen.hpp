#pragma once

#include "game/game_handler.hpp"
#include <vulkan/vulkan.h>
#include <imgui.h>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>

namespace wowee {

namespace pipeline { class AssetManager; }

namespace ui {

struct SpellInfo {
    uint32_t spellId = 0;
    std::string name;
    std::string rank;
    std::string description;     // Tooltip/description text from Spell.dbc
    uint32_t iconId = 0;         // SpellIconID
    uint32_t attributes = 0;     // Spell attributes (field 4)
    uint32_t castTimeMs = 0;     // Cast time in ms (0 = instant)
    uint32_t manaCost = 0;       // Mana cost
    uint32_t powerType = 0;      // 0=mana, 1=rage, 2=focus, 3=energy
    uint32_t rangeIndex = 0;     // Range index from SpellRange.dbc
    uint32_t schoolMask = 0;     // School bitmask (1=phys,2=holy,4=fire,8=nature,16=frost,32=shadow,64=arcane)
    uint32_t casterAuraState = 0;     // Required 1-based UNIT_FIELD_AURASTATE bit
    uint32_t casterAuraStateNot = 0;  // Forbidden 1-based UNIT_FIELD_AURASTATE bit
    [[nodiscard]] bool isPassive() const { return (attributes & 0x40) != 0; }
};

class SpellbookScreen {
public:
    [[nodiscard]] bool isOpen() const { return open; }
    void toggle() { open = !open; }
    void setOpen(bool o) { open = o; }

    // Spell name lookup - triggers DBC load if needed, used by action bar tooltips

    // Rich tooltip - renders a full spell tooltip (inside an already-open BeginTooltip block).
    // Triggers DBC load if needed. Returns true if spell data was found.
    bool renderSpellInfoTooltip(uint32_t spellId, game::GameHandler& gameHandler,
                                pipeline::AssetManager* assetManager);

    // Drag-and-drop state for action bar assignment
    [[nodiscard]] bool isDraggingSpell() const { return draggingSpell_; }
    [[nodiscard]] uint32_t getDragSpellId() const { return dragSpellId_; }
    void consumeDragSpell() { draggingSpell_ = false; dragSpellId_ = 0; dragSpellIconTex_ = VK_NULL_HANDLE; }

    /// Returns the max range in yards for a spell (0 if self-cast, unknown, or melee).
    /// Triggers DBC load if needed. Used by the action bar for out-of-range tinting.
    uint32_t getSpellMaxRange(uint32_t spellId, pipeline::AssetManager* assetManager);

    /// Returns the power cost and type for a spell (cost=0 if unknown/free).
    /// powerType: 0=mana, 1=rage, 2=focus, 3=energy, 6=runic power.
    /// Triggers DBC load if needed. Used by the action bar for insufficient-power tinting.
    void getSpellPowerInfo(uint32_t spellId, pipeline::AssetManager* assetManager,
                           uint32_t& outCost, uint32_t& outPowerType);

    /// Returns the required and forbidden caster aura-state IDs (0 means none).
    void getSpellAuraStateInfo(uint32_t spellId, pipeline::AssetManager* assetManager,
                               uint32_t& outRequired, uint32_t& outForbidden);

    /// Returns a WoW spell link string if the user shift-clicked a spell, then clears it.
    std::string getAndClearPendingChatLink() {
        std::string out = std::move(pendingChatSpellLink_);
        pendingChatSpellLink_.clear();
        return out;
    }

private:
    bool open = false;

    // Spell data (loaded from Spell.dbc)
    bool dbcLoaded = false;
    bool dbcLoadAttempted = false;
    std::unordered_map<uint32_t, SpellInfo> spellData;

    // Icon data (loaded from SpellIcon.dbc)


    // Search filter

    // Drag-and-drop from spellbook to action bar
    bool draggingSpell_ = false;
    uint32_t dragSpellId_ = 0;
    VkDescriptorSet dragSpellIconTex_ = VK_NULL_HANDLE;

    // Pending chat spell link from shift-click
    std::string pendingChatSpellLink_;

    void loadSpellDBC(pipeline::AssetManager* assetManager);
    [[nodiscard]] const SpellInfo* getSpellInfo(uint32_t spellId) const;

    // Tooltip rendering helper (showUsageHints=false when called from action bar)
    void renderSpellTooltip(const SpellInfo* info, game::GameHandler& gameHandler, bool showUsageHints = true);
};

} // namespace ui
} // namespace wowee
