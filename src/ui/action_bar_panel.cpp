// ============================================================
// ActionBarPanel - extracted from GameScreen
// Owns all action bar rendering: main bar, stance bar, bag bar,
// XP bar, reputation bar, macro resolution.
// ============================================================
#include "ui/framexml_takeover.hpp"
#include "ui/ui_texture_load.hpp"
#include "ui/action_bar_panel.hpp"
#include "addons/lua_api_registrations.hpp"
#include "ui/chat_panel.hpp"
#include "ui/settings_panel.hpp"
#include "ui/spellbook_screen.hpp"
#include "ui/inventory_screen.hpp"
#include "ui/quest_log_screen.hpp"
#include "ui/ui_colors.hpp"
#include "core/application.hpp"
#include "core/world_loader.hpp"
#include "core/logger.hpp"
#include "rendering/renderer.hpp"
#include "rendering/vk_context.hpp"
#include "core/window.hpp"
#include "game/game_handler.hpp"
#include "game/protocol_constants.hpp"
#include "game/spell_classification.hpp"
#include "game/shapeshift_forms.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_layout.hpp"
#include "audio/ui_sound_manager.hpp"
#include <imgui.h>
#include <imgui_internal.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "ui/macro_text.hpp"

namespace {
    using namespace wowee::ui::colors;
    constexpr auto& kColorRed         = kRed;

} // anonymous namespace

namespace wowee {
namespace ui {

uint32_t ActionBarPanel::resolveMacroPrimarySpellId(uint32_t macroId, game::GameHandler& gameHandler) {
    // Invalidate cache when spell list changes (learning/unlearning spells)
    size_t curSpellCount = gameHandler.getKnownSpells().size();
    if (curSpellCount != macroCacheSpellCount_) {
        macroPrimarySpellCache_.clear();
        macroCacheSpellCount_ = curSpellCount;
    }
    auto cacheIt = macroPrimarySpellCache_.find(macroId);
    if (cacheIt != macroPrimarySpellCache_.end()) return cacheIt->second;

    const std::string& macroText = gameHandler.getMacroText(macroId);
    uint32_t result = 0;
    if (!macroText.empty()) {
        for (const auto& cmdLine : macroCommandLines(macroText)) {
            std::string cl = cmdLine;
            for (char& c : cl) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            bool isCast = (cl.rfind("/cast ", 0) == 0);
            bool isCastSeq = (cl.rfind("/castsequence ", 0) == 0);
            bool isUse = (cl.rfind("/use ", 0) == 0);
            if (!isCast && !isCastSeq && !isUse) continue;
            size_t sp2 = cmdLine.find(' ');
            if (sp2 == std::string::npos) continue;
            std::string spellArg = cmdLine.substr(sp2 + 1);
            // Strip conditionals [...]
            if (!spellArg.empty() && spellArg.front() == '[') {
                size_t ce = spellArg.find(']');
                if (ce != std::string::npos) spellArg = spellArg.substr(ce + 1);
            }
            // Strip reset= spec for castsequence
            if (isCastSeq) {
                std::string tmp = spellArg;
                while (!tmp.empty() && tmp.front() == ' ') tmp.erase(tmp.begin());
                if (tmp.rfind("reset=", 0) == 0) {
                    size_t spAfter = tmp.find(' ');
                    if (spAfter != std::string::npos) spellArg = tmp.substr(spAfter + 1);
                }
            }
            // Take first alternative before ';' (for /cast) or first spell before ',' (for /castsequence)
            size_t semi = spellArg.find(isCastSeq ? ',' : ';');
            if (semi != std::string::npos) spellArg = spellArg.substr(0, semi);
            size_t ss = spellArg.find_first_not_of(" \t!");
            if (ss != std::string::npos) spellArg = spellArg.substr(ss);
            size_t se = spellArg.find_last_not_of(" \t");
            if (se != std::string::npos) spellArg.resize(se + 1);
            if (spellArg.empty()) continue;
            std::string spLow = spellArg;
            for (char& c : spLow) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            if (isUse) {
                // /use resolves an item name → find the item's on-use spell ID
                for (const auto& [entry, info] : gameHandler.getItemInfoCache()) {
                    if (!info.valid) continue;
                    std::string iName = info.name;
                    for (char& c : iName) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                    if (iName == spLow) {
                        for (const auto& sp : info.spells) {
                            if (sp.spellId != 0 && sp.spellTrigger == 0) { result = sp.spellId; break; }
                        }
                        break;
                    }
                }
            } else {
                // /cast and /castsequence resolve a spell name
                for (uint32_t sid : gameHandler.getKnownSpells()) {
                    std::string sn = gameHandler.getSpellName(sid);
                    for (char& c : sn) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                    if (sn == spLow) { result = sid; break; }
                }
            }
            break;
        }
    }
    macroPrimarySpellCache_[macroId] = result;
    return result;
}

const ActionBarPanel::MacroRenderInfo& ActionBarPanel::resolveMacroRenderInfo(
        uint32_t macroId, game::GameHandler& gameHandler) {
    const std::string& text = gameHandler.getMacroText(macroId);
    const size_t spellCount = gameHandler.getKnownSpells().size();
    const size_t itemCount = gameHandler.getItemInfoCache().size();
    auto found = macroRenderCache_.find(macroId);
    if (found != macroRenderCache_.end() && found->second.sourceText == text &&
        found->second.spellCount == spellCount && found->second.itemCount == itemCount)
        return found->second;

    MacroRenderInfo info;
    info.sourceText = text;
    info.spellCount = spellCount;
    info.itemCount = itemCount;
    info.primarySpellId = resolveMacroPrimarySpellId(macroId, gameHandler);

    std::string displayArg = macroShowtooltipArg(text);
    for (const auto& cmd : macroCommandLines(text)) {
        std::string lower = cmd;
        for (char& c : lower) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        const bool use = lower.rfind("/use ", 0) == 0;
        const bool cast = lower.rfind("/cast ", 0) == 0;
        const bool sequence = lower.rfind("/castsequence ", 0) == 0;
        if (use) info.isUse = true;
        if ((!displayArg.empty() && displayArg != "__auto__") || (!use && !cast && !sequence)) continue;
        size_t space = cmd.find(' ');
        if (space == std::string::npos) continue;
        displayArg = cmd.substr(space + 1);
        if (!displayArg.empty() && displayArg.front() == '[') {
            size_t end = displayArg.find(']');
            if (end != std::string::npos) displayArg = displayArg.substr(end + 1);
        }
        if (sequence) {
            size_t reset = displayArg.find("reset=");
            if (reset != std::string::npos) {
                size_t after = displayArg.find(' ', reset);
                if (after != std::string::npos) displayArg = displayArg.substr(after + 1);
            }
        }
        size_t separator = displayArg.find(sequence ? ',' : ';');
        if (separator != std::string::npos) displayArg.resize(separator);
        break;
    }

    size_t first = displayArg.find_first_not_of(" \t!");
    if (first != std::string::npos) displayArg.erase(0, first); else displayArg.clear();
    size_t last = displayArg.find_last_not_of(" \t");
    if (last != std::string::npos) displayArg.resize(last + 1);
    size_t rank = displayArg.find('(');
    if (rank != std::string::npos) displayArg.resize(rank);
    while (!displayArg.empty() && displayArg.back() == ' ') displayArg.pop_back();
    for (char& c : displayArg) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

    if (displayArg.empty() || displayArg == "__auto__") {
        info.iconSpellId = info.primarySpellId;
    } else {
        for (uint32_t sid : gameHandler.getKnownSpells()) {
            std::string name = gameHandler.getSpellName(sid);
            for (char& c : name) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            if (name == displayArg) { info.iconSpellId = sid; break; }
        }
        if (info.isUse) {
            for (const auto& [entry, item] : gameHandler.getItemInfoCache()) {
                if (!item.valid) continue;
                std::string name = item.name;
                for (char& c : name) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                if (name == displayArg) { info.itemEntry = entry; break; }
            }
        }
    }

    return macroRenderCache_.insert_or_assign(macroId, std::move(info)).first->second;
}

void ActionBarPanel::renderActionBar(game::GameHandler& gameHandler,
                             SettingsPanel& settingsPanel,
                             ChatPanel& chatPanel,
                             InventoryScreen& inventoryScreen,
                             SpellbookScreen& spellbookScreen,
                             QuestLogScreen& /*questLogScreen*/,
                             SpellIconFn getSpellIcon) {
    // Use ImGui's display size - always in sync with the current swap-chain/frame,
    // whereas window->getWidth/Height() can lag by one frame on resize events.
    ImVec2 displaySize = ImGui::GetIO().DisplaySize;
    float screenW = displaySize.x > 0.0f ? displaySize.x : 1280.0f;
    float screenH = displaySize.y > 0.0f ? displaySize.y : 720.0f;
    auto* assetMgr = services_.assetManager;

    float slotSize = 48.0f * settingsPanel.pendingActionBarScale;
    float spacing = 4.0f;
    float padding = 8.0f;

    // Build one inventory index for all action slots. Previously each item slot
    // rescanned backpack, equipment, and every bag both for lookup and stack count.
    std::unordered_map<uint32_t, const game::ItemDef*> actionItemsById;
    std::unordered_map<uint32_t, uint32_t> actionItemCounts;
    auto indexSlot = [&](const game::ItemSlot& invSlot) {
        if (invSlot.empty() || invSlot.item.itemId == 0) return;
        actionItemsById.try_emplace(invSlot.item.itemId, &invSlot.item);
        actionItemCounts[invSlot.item.itemId] += invSlot.item.stackCount;
    };
    auto& actionInventory = gameHandler.getInventory();
    actionItemsById.reserve(32);
    actionItemCounts.reserve(32);
    for (int i = 0; i < actionInventory.getBackpackSize(); ++i)
        indexSlot(actionInventory.getBackpackSlot(i));
    for (int i = 0; i < game::Inventory::NUM_EQUIP_SLOTS; ++i)
        indexSlot(actionInventory.getEquipSlot(static_cast<game::EquipSlot>(i)));
    for (int bag = 0; bag < game::Inventory::NUM_BAG_SLOTS; ++bag)
        for (int i = 0; i < actionInventory.getBagSize(bag); ++i)
            indexSlot(actionInventory.getBagSlot(bag, i));
    float barW = 12 * slotSize + 11 * spacing + padding * 2;
    float barH = slotSize + 24.0f;
    float barX = (screenW - barW) / 2.0f;
    float barY = screenH - barH;

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                             ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(padding, padding));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0.0f, 0.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.05f, 0.05f, 0.05f, 0.9f));

    // Per-slot rendering lambda - shared by both action bars
    const auto& bar = gameHandler.getActionBar();
    static constexpr const char* keyLabels1[] = {"1","2","3","4","5","6","7","8","9","0","-","="};

    auto changeMainPage = [&](int delta) {
        mainActionBarPage_ += delta;
        if (mainActionBarPage_ < 1) mainActionBarPage_ = kFrameXmlActionBarPages;
        if (mainActionBarPage_ > kFrameXmlActionBarPages) mainActionBarPage_ = 1;
    };

    auto renderBarSlot = [&](int absSlot, const char* keyLabel) {
        ImGui::BeginGroup();
        ImGui::PushID(absSlot);

        const auto& slot = bar[absSlot];
        bool onCooldown = !slot.isReady();

        // Item cooldowns are frequently reported against the item's on-use spell
        // (especially on Wrath) rather than against the action-slot item ID.
        // Resolve that shared spell cooldown just as macro buttons do.
        float itemCooldownRemaining = 0.0f;
        float itemCooldownTotal = 0.0f;
        if (slot.type == game::ActionBarSlot::ITEM && slot.id != 0 && !onCooldown) {
            if (const auto* item = gameHandler.getItemInfo(slot.id); item && item->valid) {
                for (const auto& itemSpell : item->spells) {
                    if (itemSpell.spellId == 0 ||
                        (itemSpell.spellTrigger != 0 && itemSpell.spellTrigger != 5)) continue;
                    const float cooldown = gameHandler.getSpellCooldown(itemSpell.spellId);
                    if (cooldown > itemCooldownRemaining) itemCooldownRemaining = cooldown;
                }
            }
            if (itemCooldownRemaining > 0.0f) {
                float& rememberedTotal = itemSpellCooldownTotals_[slot.id];
                rememberedTotal = std::max(rememberedTotal, itemCooldownRemaining);
                itemCooldownTotal = rememberedTotal;
                onCooldown = true;
            } else {
                itemSpellCooldownTotals_.erase(slot.id);
            }
        }

        // Macro cooldown: check the cached primary spell's cooldown.
        float macroCooldownRemaining = 0.0f;
        float macroCooldownTotal = 0.0f;
        if (slot.type == game::ActionBarSlot::MACRO && slot.id != 0 && !onCooldown) {
            uint32_t macroSpellId = resolveMacroPrimarySpellId(slot.id, gameHandler);
            if (macroSpellId != 0) {
                float cd = gameHandler.getSpellCooldown(macroSpellId);
                if (cd > 0.0f) {
                    macroCooldownRemaining = cd;
                    macroCooldownTotal = cd;
                    onCooldown = true;
                }
            }
        }

        const bool onGCD = gameHandler.isGCDActive() && !onCooldown && !slot.isEmpty();

        // Out-of-range check: red tint when a targeted spell cannot reach the current target.
        // Applies to SPELL and MACRO slots with a known max range (>5 yd) and an active target.
        // Item range is checked below after barItemDef is populated.
        bool outOfRange = false;
        {
            uint32_t rangeCheckSpellId = 0;
            if (slot.type == game::ActionBarSlot::SPELL && slot.id != 0)
                rangeCheckSpellId = slot.id;
            else if (slot.type == game::ActionBarSlot::MACRO && slot.id != 0)
                rangeCheckSpellId = resolveMacroPrimarySpellId(slot.id, gameHandler);
            if (rangeCheckSpellId != 0 && !onCooldown && gameHandler.hasTarget()) {
                uint32_t maxRange = spellbookScreen.getSpellMaxRange(rangeCheckSpellId, assetMgr);
                if (maxRange > 5) {
                    auto& em = gameHandler.getEntityManager();
                    auto targetEnt = em.getEntity(gameHandler.getTargetGuid());
                    if (targetEnt) {
                        // The local player's network entity remains at its last
                        // server-authored position while client movement updates
                        // movementInfo. Using the entity here made every ranged
                        // action appear out of range after walking away from login.
                        const auto& playerPos = gameHandler.getMovementInfo();
                        float dx = playerPos.x - targetEnt->getX();
                        float dy = playerPos.y - targetEnt->getY();
                        float dz = playerPos.z - targetEnt->getZ();
                        float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                        // Edge to edge, not centre to centre. A spell's range
                        // is measured from where a unit's model ends, so a
                        // large creature is in reach from further out - and
                        // measuring to its centre put a charge at a giant out
                        // of range while the server, which does it properly,
                        // would have allowed it. Reach is zero until the
                        // server sends it, which falls back to the old
                        // behaviour rather than inventing a size.
                        if (auto targetUnit = std::dynamic_pointer_cast<game::Unit>(targetEnt))
                            dist -= targetUnit->getCombatReach();
                        if (auto playerUnit = std::dynamic_pointer_cast<game::Unit>(
                                em.getEntity(gameHandler.getPlayerGuid())))
                            dist -= playerUnit->getCombatReach();
                        if (dist > static_cast<float>(maxRange))
                            outOfRange = true;
                    }
                }
            }
        }

        // Insufficient-power check: tint when player doesn't have enough power to cast.
        // Applies to SPELL and MACRO slots with a known power cost.
        bool insufficientPower = false;
        bool reactiveUnavailable = false;
        bool targetStateUnavailable = false;
        {
            uint32_t powerCheckSpellId = 0;
            if (slot.type == game::ActionBarSlot::SPELL && slot.id != 0)
                powerCheckSpellId = slot.id;
            else if (slot.type == game::ActionBarSlot::MACRO && slot.id != 0)
                powerCheckSpellId = resolveMacroPrimarySpellId(slot.id, gameHandler);
            uint32_t spellCost = 0, spellPowerType = 0;
            // Auto Shot, Shoot and Throw have a dummy 1-rage/1-cost value in the
            // legacy Spell.dbc. Their actual readiness is server/weapon driven.
            if (powerCheckSpellId != 0 && !onCooldown &&
                !game::spellclass::isRangedWeaponAutoAttack(powerCheckSpellId))
                spellbookScreen.getSpellPowerInfo(powerCheckSpellId, assetMgr, spellCost, spellPowerType);
            if (spellCost > 0) {
                auto playerEnt = gameHandler.getEntityManager().getEntity(gameHandler.getPlayerGuid());
                if (playerEnt && (playerEnt->getType() == game::ObjectType::PLAYER ||
                                  playerEnt->getType() == game::ObjectType::UNIT)) {
                    auto unit = std::static_pointer_cast<game::Unit>(playerEnt);
                    // Spell.dbc identifies the resource pool. Compare that pool directly
                    // instead of assuming the character's currently displayed power type;
                    // this is shared by mana, rage, focus, energy, runes and runic power.
                    if (spellPowerType < 7 &&
                        unit->getPowerByType(static_cast<uint8_t>(spellPowerType)) < spellCost)
                        insufficientPower = true;
                }
            }

            // Reactive combat abilities declare their opportunity through Spell.dbc's
            // caster aura-state fields. The values are 1-based bit positions in
            // UNIT_FIELD_AURASTATE (Overpower, Revenge, execute windows, etc.).
            uint32_t requiredState = 0, forbiddenState = 0;
            if (powerCheckSpellId != 0 && !onCooldown)
                spellbookScreen.getSpellAuraStateInfo(powerCheckSpellId, assetMgr,
                                                       requiredState, forbiddenState);
            if ((requiredState > 0 && requiredState <= 32) ||
                (forbiddenState > 0 && forbiddenState <= 32)) {
                auto playerEnt = gameHandler.getEntityManager().getEntity(gameHandler.getPlayerGuid());
                if (playerEnt && (playerEnt->getType() == game::ObjectType::PLAYER ||
                                  playerEnt->getType() == game::ObjectType::UNIT)) {
                    auto unit = std::static_pointer_cast<game::Unit>(playerEnt);
                    const uint32_t states = unit->getAuraState();
                    if (requiredState > 0 && requiredState <= 32 &&
                        (states & (1u << (requiredState - 1))) == 0)
                        reactiveUnavailable = true;
                    if (forbiddenState > 0 && forbiddenState <= 32 &&
                        (states & (1u << (forbiddenState - 1))) != 0)
                        reactiveUnavailable = true;
                }
            }

            // Target aura-state gating: abilities like Execute or Hammer of Wrath are
            // only usable when the current target is in the required state (e.g. below
            // 20% health). The server keeps that as a bit in the target's UNIT_FIELD_AURASTATE.
            if (powerCheckSpellId != 0 && !onCooldown && gameHandler.hasTarget()) {
                uint32_t reqTargetState = gameHandler.getSpellTargetAuraState(powerCheckSpellId);
                if (reqTargetState > 0 && reqTargetState <= 32) {
                    auto targetEnt = gameHandler.getEntityManager().getEntity(gameHandler.getTargetGuid());
                    if (targetEnt && (targetEnt->getType() == game::ObjectType::UNIT ||
                                      targetEnt->getType() == game::ObjectType::PLAYER)) {
                        auto tunit = std::static_pointer_cast<game::Unit>(targetEnt);
                        if ((tunit->getAuraState() & (1u << (reqTargetState - 1))) == 0)
                            targetStateUnavailable = true;
                    }
                }
            }
        }

        auto getSpellName = [&](uint32_t spellId) -> std::string {
            std::string name = spellbookScreen.lookupSpellName(spellId, assetMgr);
            if (!name.empty()) return name;
            return "Spell #" + std::to_string(spellId);
        };

        // Try to get icon texture for this slot
        VkDescriptorSet iconTex = VK_NULL_HANDLE;
        const game::ItemDef* barItemDef = nullptr;
        uint32_t itemDisplayInfoId = 0;
        std::string itemNameFromQuery;
        if (slot.type == game::ActionBarSlot::SPELL && slot.id != 0) {
            iconTex = getSpellIcon(slot.id, assetMgr);
        } else if (slot.type == game::ActionBarSlot::ITEM && slot.id != 0) {
            if (auto it = actionItemsById.find(slot.id); it != actionItemsById.end())
                barItemDef = it->second;
            if (barItemDef && barItemDef->displayInfoId != 0)
                itemDisplayInfoId = barItemDef->displayInfoId;
            if (itemDisplayInfoId == 0) {
                if (auto* info = gameHandler.getItemInfo(slot.id)) {
                    itemDisplayInfoId = info->displayInfoId;
                    if (itemNameFromQuery.empty() && !info->name.empty())
                        itemNameFromQuery = info->name;
                }
            }
            if (itemDisplayInfoId != 0)
                iconTex = inventoryScreen.getItemIcon(itemDisplayInfoId);
        }

        // Macro icon: #showtooltip [SpellName] → show that spell's icon on the button
        bool macroIsUseCmd = false;  // tracks if the macro's primary command is /use (for item icon fallback)
        uint32_t macroItemEntry = 0;
        if (slot.type == game::ActionBarSlot::MACRO && slot.id != 0 && !iconTex) {
            const auto& macroInfo = resolveMacroRenderInfo(slot.id, gameHandler);
            macroIsUseCmd = macroInfo.isUse;
            macroItemEntry = macroInfo.itemEntry;
            if (macroInfo.iconSpellId != 0)
                iconTex = assetMgr ? getSpellIcon(macroInfo.iconSpellId, assetMgr) : VK_NULL_HANDLE;
            if (!iconTex && macroItemEntry != 0) {
                if (const auto* item = gameHandler.getItemInfo(macroItemEntry); item && item->displayInfoId != 0)
                    iconTex = inventoryScreen.getItemIcon(item->displayInfoId);
            }
        }

        // Item-missing check: grey out item slots whose item is not in the player's inventory.
        const bool itemMissing = (slot.type == game::ActionBarSlot::ITEM && slot.id != 0
                                  && barItemDef == nullptr && !onCooldown);

        // Ranged item out-of-range check (runs after barItemDef is populated above).
        // InventoryType: 15=Ranged, 25=Thrown, 26=RangedRight.
        if (!outOfRange && slot.type == game::ActionBarSlot::ITEM && barItemDef
            && !onCooldown && gameHandler.hasTarget()) {
            constexpr uint8_t INVTYPE_RANGED      = 15;
            constexpr uint8_t INVTYPE_THROWN      = game::InvType::THROWN;
            constexpr uint8_t INVTYPE_RANGEDRIGHT = game::InvType::RANGED_GUN;
            uint32_t itemMaxRange = 0;
            if (barItemDef->inventoryType == INVTYPE_RANGED ||
                barItemDef->inventoryType == INVTYPE_RANGEDRIGHT)
                itemMaxRange = 40;
            else if (barItemDef->inventoryType == INVTYPE_THROWN)
                itemMaxRange = 30;
            if (itemMaxRange > 0) {
                auto& em = gameHandler.getEntityManager();
                auto playerEnt = em.getEntity(gameHandler.getPlayerGuid());
                auto targetEnt = em.getEntity(gameHandler.getTargetGuid());
                if (playerEnt && targetEnt) {
                    float dx = playerEnt->getX() - targetEnt->getX();
                    float dy = playerEnt->getY() - targetEnt->getY();
                    float dz = playerEnt->getZ() - targetEnt->getZ();
                    if (std::sqrt(dx*dx + dy*dy + dz*dz) > static_cast<float>(itemMaxRange))
                        outOfRange = true;
                }
            }
        }

        bool clicked = false;
        if (iconTex) {
            ImVec4 tintColor(1, 1, 1, 1);
            ImVec4 bgColor(0.1f, 0.1f, 0.1f, 0.9f);
            if (onCooldown)          { tintColor = ImVec4(0.4f, 0.4f, 0.4f, 0.8f); }
            else if (onGCD)          { tintColor = ImVec4(0.6f, 0.6f, 0.6f, 0.85f); }
            else if (outOfRange)     { tintColor = ImVec4(0.85f, 0.35f, 0.35f, 0.9f); }
            else if (insufficientPower || reactiveUnavailable || targetStateUnavailable) { tintColor = ImVec4(0.38f, 0.38f, 0.38f, 0.78f); }
            else if (itemMissing)    { tintColor = ImVec4(0.35f, 0.35f, 0.35f, 0.7f); }
            clicked = ImGui::ImageButton("##icon",
                (ImTextureID)(uintptr_t)iconTex,
                ImVec2(slotSize, slotSize),
                ImVec2(0, 0), ImVec2(1, 1),
                bgColor, tintColor);
        } else {
            if (onCooldown)            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.2f, 0.2f, 0.8f));
            else if (outOfRange)       ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.45f, 0.15f, 0.15f, 0.9f));
            else if (insufficientPower || reactiveUnavailable || targetStateUnavailable)
                                             ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.16f, 0.16f, 0.16f, 0.85f));
            else if (itemMissing)      ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.12f, 0.12f, 0.7f));
            else if (slot.isEmpty())   ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.15f, 0.15f, 0.8f));
            else                       ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.3f, 0.3f, 0.5f, 0.9f));

            char label[32];
            if (slot.type == game::ActionBarSlot::SPELL) {
                std::string spellName = getSpellName(slot.id);
                if (spellName.size() > 6) spellName = spellName.substr(0, 6);
                snprintf(label, sizeof(label), "%s", spellName.c_str());
            } else if (slot.type == game::ActionBarSlot::ITEM && barItemDef) {
                std::string itemName = barItemDef->name;
                if (itemName.size() > 6) itemName = itemName.substr(0, 6);
                snprintf(label, sizeof(label), "%s", itemName.c_str());
            } else if (slot.type == game::ActionBarSlot::ITEM) {
                snprintf(label, sizeof(label), "Item");
            } else if (slot.type == game::ActionBarSlot::MACRO) {
                snprintf(label, sizeof(label), "Macro");
            } else {
                snprintf(label, sizeof(label), "--");
            }
            clicked = ImGui::Button(label, ImVec2(slotSize, slotSize));
            ImGui::PopStyleColor();
        }

        // Press-and-hold to pick a slot up for reorganizing. Holding the left mouse
        // button on a filled slot for kActionBarPickupDelay lifts its action onto the
        // cursor; a normal quick click still casts/uses as before. Once carried, click
        // another slot to swap (see the drop handling below).
        // Lock Action Bars, which the panel offers and nothing read - so an
        // action could always be dragged off, which is what the setting exists
        // to stop. Read per slot rather than hoisted: this runs inside the
        // per-slot loop, the answer comes from a map, and a bar being locked
        // mid-drag should take effect on the next press rather than never.
        const bool barsLocked =
            addons::storedCVarValue("lockActionBars", "0") != "0";

        if (actionBarDragSlot_ < 0 && !barsLocked) {
            const bool held = ImGui::IsItemActive() && ImGui::IsMouseDown(ImGuiMouseButton_Left);
            if (held && !slot.isEmpty()) {
                if (actionBarHoldSlot_ != absSlot) {
                    actionBarHoldSlot_  = absSlot;
                    actionBarHoldStart_ = ImGui::GetTime();
                } else {
                    double heldFor = ImGui::GetTime() - actionBarHoldStart_;
                    if (heldFor >= kActionBarPickupDelay) {
                        // Pick up: lift this action onto the cursor and empty its slot
                        // so it visibly leaves the bar while carried.
                        actionBarDragSlot_ = absSlot;
                        actionBarDragIcon_ = iconTex;
                        actionBarCarryType_ = static_cast<uint8_t>(slot.type);
                        actionBarCarryId_ = slot.id;
                        gameHandler.setActionBarSlot(absSlot, game::ActionBarSlot::EMPTY, 0);
                        actionBarHoldSlot_ = -1;
                        // The initiating press is still down; swallow its release so it
                        // doesn't immediately drop the action back into its own slot.
                        actionBarCarryPressActive_ = true;
                    } else {
                        // Draw a rising fill as feedback that a pickup is charging.
                        float t = static_cast<float>(heldFor / kActionBarPickupDelay);
                        ImVec2 rMin = ImGui::GetItemRectMin();
                        ImVec2 rMax = ImGui::GetItemRectMax();
                        float fillTop = rMax.y - (rMax.y - rMin.y) * t;
                        ImGui::GetWindowDrawList()->AddRectFilled(
                            ImVec2(rMin.x, fillTop), rMax,
                            IM_COL32(255, 220, 80, 70));
                    }
                }
            } else if (actionBarHoldSlot_ == absSlot) {
                // Released or dragged off before the delay - cancel this hold.
                actionBarHoldSlot_ = -1;
            }
        }

        // Error-flash overlay: red fade on spell cast failure (~0.5 s).
        // Check both spell slots directly and macro slots via their primary spell.
        {
            uint32_t flashSpellId = 0;
            if (slot.type == game::ActionBarSlot::SPELL && slot.id != 0)
                flashSpellId = slot.id;
            else if (slot.type == game::ActionBarSlot::MACRO && slot.id != 0)
                flashSpellId = resolveMacroPrimarySpellId(slot.id, gameHandler);
            auto flashIt = (flashSpellId != 0) ? actionFlashEndTimes_.find(flashSpellId) : actionFlashEndTimes_.end();
            if (flashIt != actionFlashEndTimes_.end()) {
                float now = static_cast<float>(ImGui::GetTime());
                float remaining = flashIt->second - now;
                if (remaining > 0.0f) {
                    float alpha = remaining / kActionFlashDuration;  // 1→0
                    ImVec2 rMin = ImGui::GetItemRectMin();
                    ImVec2 rMax = ImGui::GetItemRectMax();
                    ImGui::GetWindowDrawList()->AddRectFilled(
                        rMin, rMax,
                        ImGui::ColorConvertFloat4ToU32(ImVec4(1.0f, 0.1f, 0.1f, 0.55f * alpha)));
                } else {
                    actionFlashEndTimes_.erase(flashIt);
                }
            }
        }

        bool hoveredOnRelease = ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem) &&
                                ImGui::IsMouseReleased(ImGuiMouseButton_Left);

        if (hoveredOnRelease && spellbookScreen.isDraggingSpell()) {
            gameHandler.setActionBarSlot(absSlot, game::ActionBarSlot::SPELL,
                spellbookScreen.getDragSpellId());
            spellbookScreen.consumeDragSpell();
        } else if (hoveredOnRelease && inventoryScreen.isHoldingItem()) {
            const auto& held = inventoryScreen.getHeldItem();
            gameHandler.setActionBarSlot(absSlot, game::ActionBarSlot::ITEM, held.itemId);
            inventoryScreen.returnHeldItem(gameHandler.getInventory());
        } else if (clicked && actionBarDragSlot_ >= 0) {
            if (actionBarCarryPressActive_) {
                // Release of the press that completed the pickup - keep carrying.
            } else {
                // Capture whatever is here before overwriting it (the slot is a live
                // reference, so read the displaced action first).
                const auto displacedType = slot.type;
                const uint32_t displacedId = slot.id;
                gameHandler.setActionBarSlot(
                    absSlot, static_cast<game::ActionBarSlot::Type>(actionBarCarryType_),
                    actionBarCarryId_);
                // A displaced action falls back into the now-empty origin slot (a swap).
                // Dropping onto the origin itself just restores the carried action.
                if (absSlot != actionBarDragSlot_ &&
                    displacedType != game::ActionBarSlot::EMPTY && displacedId != 0) {
                    gameHandler.setActionBarSlot(actionBarDragSlot_, displacedType, displacedId);
                }
                actionBarDragSlot_ = -1;
                actionBarDragIcon_ = 0;
                actionBarCarryType_ = 0;
                actionBarCarryId_ = 0;
            }
        } else if (clicked && !slot.isEmpty()) {
            if (slot.type == game::ActionBarSlot::SPELL && slot.isReady()) {
                // Check if this spell belongs to an item (e.g., Hearthstone spell 8690).
                // Item-use spells must go through CMSG_USE_ITEM, not CMSG_CAST_SPELL.
                uint32_t itemForSpell = gameHandler.getItemIdForSpell(slot.id);
                if (itemForSpell != 0) {
                    gameHandler.useItemById(itemForSpell);
                } else {
                    uint64_t target = gameHandler.hasTarget() ? gameHandler.getTargetGuid() : 0;
                    gameHandler.castSpell(slot.id, target);
                }
            } else if (slot.type == game::ActionBarSlot::ITEM && slot.id != 0) {
                gameHandler.useItemById(slot.id);
            } else if (slot.type == game::ActionBarSlot::MACRO) {
                chatPanel.executeMacroText(gameHandler, gameHandler.getMacroText(slot.id));
            }
        }

        // Right-click context menu for non-empty slots
        if (!slot.isEmpty()) {
            // Use a unique popup ID per slot so multiple slots don't share state
            char ctxId[32];
            snprintf(ctxId, sizeof(ctxId), "##ABCtx%d", absSlot);
            if (ImGui::BeginPopupContextItem(ctxId)) {
                if (slot.type == game::ActionBarSlot::SPELL) {
                    std::string spellName = getSpellName(slot.id);
                    ImGui::TextDisabled("%s", spellName.c_str());
                    ImGui::Separator();
                    if (onCooldown) ImGui::BeginDisabled();
                    if (ImGui::MenuItem("Cast")) {
                        uint64_t target = gameHandler.hasTarget() ? gameHandler.getTargetGuid() : 0;
                        gameHandler.castSpell(slot.id, target);
                    }
                    if (onCooldown) ImGui::EndDisabled();
                } else if (slot.type == game::ActionBarSlot::ITEM) {
                    const char* iName = (barItemDef && !barItemDef->name.empty())
                        ? barItemDef->name.c_str()
                        : (!itemNameFromQuery.empty() ? itemNameFromQuery.c_str() : "Item");
                    ImGui::TextDisabled("%s", iName);
                    ImGui::Separator();
                    if (ImGui::MenuItem("Use")) {
                        gameHandler.useItemById(slot.id);
                    }
                } else if (slot.type == game::ActionBarSlot::MACRO) {
                    ImGui::TextDisabled("Macro #%u", slot.id);
                    ImGui::Separator();
                    if (ImGui::MenuItem("Execute")) {
                        chatPanel.executeMacroText(gameHandler, gameHandler.getMacroText(slot.id));
                    }
                    if (ImGui::MenuItem("Edit")) {
                        const std::string& txt = gameHandler.getMacroText(slot.id);
                        strncpy(macroEditorBuf_, txt.c_str(), sizeof(macroEditorBuf_) - 1);
                        macroEditorBuf_[sizeof(macroEditorBuf_) - 1] = '\0';
                        macroEditorId_   = slot.id;
                        macroEditorOpen_ = true;
                    }
                }
                ImGui::Separator();
                if (ImGui::MenuItem("Clear Slot")) {
                    gameHandler.setActionBarSlot(absSlot, game::ActionBarSlot::EMPTY, 0);
                }
                ImGui::EndPopup();
            }
        }

        // Tooltip
        if (ImGui::IsItemHovered() && !slot.isEmpty() && slot.id != 0) {
            if (slot.type == game::ActionBarSlot::SPELL) {
                // Use the spellbook's rich tooltip (school, cost, cast time, range, description).
                // Falls back to the simple name if DBC data isn't loaded yet.
                ImGui::BeginTooltip();
                bool richOk = spellbookScreen.renderSpellInfoTooltip(slot.id, gameHandler, assetMgr);
                if (!richOk) {
                    ImGui::Text("%s", getSpellName(slot.id).c_str());
                }
                // Hearthstone: add location note after the spell tooltip body
                if (slot.id == game::SPELL_ID_HEARTHSTONE) {
                    uint32_t mapId = 0; glm::vec3 pos;
                    if (gameHandler.getHomeBind(mapId, pos)) {
                        std::string homeLocation;
                        // Zone name (from zoneId stored in bind point)
                        uint32_t zoneId = gameHandler.getHomeBindZoneId();
                        if (zoneId != 0) {
                            homeLocation = gameHandler.getWhoAreaName(zoneId);
                        }
                        // Fall back to continent name if zone unavailable
                        if (homeLocation.empty()) {
                            const char* dn = core::WorldLoader::mapDisplayName(mapId);
                            homeLocation = dn ? dn : "Unknown";
                        }
                        ImGui::TextColored(ImVec4(0.8f, 0.9f, 1.0f, 1.0f),
                                           "Home: %s", homeLocation.c_str());
                    }
                }
                if (outOfRange) {
                    ImGui::TextColored(colors::kHostileRed, "Out of range");
                }
                if (insufficientPower) {
                    ImGui::TextColored(ImVec4(0.75f, 0.55f, 1.0f, 1.0f), "Not enough power");
                }
                if (reactiveUnavailable) {
                    ImGui::TextColored(ImVec4(0.75f, 0.75f, 0.75f, 1.0f), "Requires a combat opportunity");
                }
                if (targetStateUnavailable) {
                    // Name the requirement for the common cases (below-20% for Execute, etc.).
                    uint32_t reqTargetState = gameHandler.getSpellTargetAuraState(
                        slot.type == game::ActionBarSlot::MACRO
                            ? resolveMacroPrimarySpellId(slot.id, gameHandler) : slot.id);
                    const char* msg = "Target isn't in the required state";
                    switch (reqTargetState) {
                        case 2:  msg = "Target must be below 20% health"; break;
                        case 13: msg = "Target must be below 35% health"; break;
                        case 17: msg = "Target must be enraged"; break;
                        case 18: msg = "Target must be bleeding"; break;
                        default: break;
                    }
                    ImGui::TextColored(ImVec4(0.75f, 0.75f, 0.75f, 1.0f), "%s", msg);
                }
                if (onCooldown) {
                    float cd = slot.cooldownRemaining;
                    if (cd >= 60.0f)
                        ImGui::TextColored(kColorRed,
                            "Cooldown: %d min %d sec", static_cast<int>(cd)/60, static_cast<int>(cd)%60);
                    else
                        ImGui::TextColored(kColorRed, "Cooldown: %.1f sec", cd);
                }
                ImGui::EndTooltip();
            } else if (slot.type == game::ActionBarSlot::MACRO) {
                ImGui::BeginTooltip();
                // Show the primary spell's rich tooltip (like WoW does for macro buttons)
                uint32_t macroSpellId = resolveMacroPrimarySpellId(slot.id, gameHandler);
                bool showedRich = false;
                if (macroSpellId != 0) {
                    showedRich = spellbookScreen.renderSpellInfoTooltip(macroSpellId, gameHandler, assetMgr);
                    if (onCooldown && macroCooldownRemaining > 0.0f) {
                        float cd = macroCooldownRemaining;
                        if (cd >= 60.0f)
                            ImGui::TextColored(kColorRed,
                                "Cooldown: %d min %d sec", static_cast<int>(cd)/60, static_cast<int>(cd)%60);
                        else
                            ImGui::TextColored(kColorRed, "Cooldown: %.1f sec", cd);
                    }
                }
                if (!showedRich) {
                    // For /use macros: try showing the item tooltip instead
                    if (macroIsUseCmd && macroItemEntry != 0) {
                        if (const auto* info = gameHandler.getItemInfo(macroItemEntry); info && info->valid) {
                            inventoryScreen.renderItemTooltip(*info);
                            showedRich = true;
                        }
                    }
                    if (!showedRich) {
                        ImGui::Text("Macro #%u", slot.id);
                        const std::string& macroText = gameHandler.getMacroText(slot.id);
                        if (!macroText.empty()) {
                            ImGui::Separator();
                            ImGui::TextUnformatted(macroText.c_str());
                        } else {
                            ImGui::TextDisabled("(no text - right-click to Edit)");
                        }
                    }
                }
                ImGui::EndTooltip();
            } else if (slot.type == game::ActionBarSlot::ITEM) {
                ImGui::BeginTooltip();
                // Prefer full rich tooltip from ItemQueryResponseData (has stats, quality, set info)
                const auto* itemQueryInfo = gameHandler.getItemInfo(slot.id);
                if (itemQueryInfo && itemQueryInfo->valid) {
                    inventoryScreen.renderItemTooltip(*itemQueryInfo);
                } else if (barItemDef && !barItemDef->name.empty()) {
                    ImGui::Text("%s", barItemDef->name.c_str());
                } else if (!itemNameFromQuery.empty()) {
                    ImGui::Text("%s", itemNameFromQuery.c_str());
                } else {
                    ImGui::Text("Item #%u", slot.id);
                }
                if (onCooldown) {
                    float cd = slot.cooldownRemaining;
                    if (cd >= 60.0f)
                        ImGui::TextColored(kColorRed,
                            "Cooldown: %d min %d sec", static_cast<int>(cd)/60, static_cast<int>(cd)%60);
                    else
                        ImGui::TextColored(kColorRed, "Cooldown: %.1f sec", cd);
                }
                ImGui::EndTooltip();
            }
        }

        // Cooldown overlay: WoW-style clock-sweep + time text
        if (onCooldown) {
            ImVec2 btnMin = ImGui::GetItemRectMin();
            ImVec2 btnMax = ImGui::GetItemRectMax();
            float cx = (btnMin.x + btnMax.x) * 0.5f;
            float cy = (btnMin.y + btnMax.y) * 0.5f;
            float r  = (btnMax.x - btnMin.x) * 0.5f;
            auto* dl = ImGui::GetWindowDrawList();

            // For macros, use the resolved primary spell cooldown instead of the slot's own.
            float effCdTotal = (macroCooldownTotal > 0.0f) ? macroCooldownTotal
                               : (itemCooldownTotal > 0.0f) ? itemCooldownTotal
                               : slot.cooldownTotal;
            float effCdRemaining = (macroCooldownRemaining > 0.0f) ? macroCooldownRemaining
                                   : (itemCooldownRemaining > 0.0f) ? itemCooldownRemaining
                                   : slot.cooldownRemaining;
            float total       = (effCdTotal > 0.0f) ? effCdTotal : 1.0f;
            float elapsed     = total - effCdRemaining;
            float elapsedFrac = std::min(1.0f, std::max(0.0f, elapsed / total));
            if (elapsedFrac > 0.005f) {
                constexpr int N_SEGS = 32;
                float startAngle = -IM_PI * 0.5f;
                float endAngle   = startAngle + elapsedFrac * 2.0f * IM_PI;
                float fanR       = r * 1.5f;
                ImVec2 pts[N_SEGS + 2];
                pts[0] = ImVec2(cx, cy);
                for (int s = 0; s <= N_SEGS; ++s) {
                    float a = startAngle + (endAngle - startAngle) * s / static_cast<float>(N_SEGS);
                    pts[s + 1] = ImVec2(cx + std::cos(a) * fanR, cy + std::sin(a) * fanR);
                }
                dl->AddConvexPolyFilled(pts, N_SEGS + 2, IM_COL32(0, 0, 0, 170));
            }

            char cdText[16];
            float cd = effCdRemaining;
            if (cd >= 3600.0f)    snprintf(cdText, sizeof(cdText), "%dh", static_cast<int>(cd) / 3600);
            else if (cd >= 60.0f) snprintf(cdText, sizeof(cdText), "%dm%ds", static_cast<int>(cd) / 60, static_cast<int>(cd) % 60);
            else if (cd >= 5.0f)  snprintf(cdText, sizeof(cdText), "%ds", static_cast<int>(cd));
            else                  snprintf(cdText, sizeof(cdText), "%.1f", cd);
            ImVec2 textSize = ImGui::CalcTextSize(cdText);
            float tx = cx - textSize.x * 0.5f;
            float ty = cy - textSize.y * 0.5f;
            dl->AddText(ImVec2(tx + 1.0f, ty + 1.0f), IM_COL32(0, 0, 0, 220), cdText);
            dl->AddText(ImVec2(tx, ty), IM_COL32(255, 255, 255, 255), cdText);
        }

        // GCD overlay - subtle dark fan sweep (thinner/lighter than regular cooldown)
        if (onGCD) {
            ImVec2 btnMin = ImGui::GetItemRectMin();
            ImVec2 btnMax = ImGui::GetItemRectMax();
            float cx = (btnMin.x + btnMax.x) * 0.5f;
            float cy = (btnMin.y + btnMax.y) * 0.5f;
            float r  = (btnMax.x - btnMin.x) * 0.5f;
            auto* dl = ImGui::GetWindowDrawList();
            float gcdRem   = gameHandler.getGCDRemaining();
            float gcdTotal = gameHandler.getGCDTotal();
            if (gcdTotal > 0.0f) {
                float elapsed     = gcdTotal - gcdRem;
                float elapsedFrac = std::min(1.0f, std::max(0.0f, elapsed / gcdTotal));
                if (elapsedFrac > 0.005f) {
                    constexpr int N_SEGS = 24;
                    float startAngle = -IM_PI * 0.5f;
                    float endAngle   = startAngle + elapsedFrac * 2.0f * IM_PI;
                    float fanR       = r * 1.4f;
                    ImVec2 pts[N_SEGS + 2];
                    pts[0] = ImVec2(cx, cy);
                    for (int s = 0; s <= N_SEGS; ++s) {
                        float a = startAngle + (endAngle - startAngle) * s / static_cast<float>(N_SEGS);
                        pts[s + 1] = ImVec2(cx + std::cos(a) * fanR, cy + std::sin(a) * fanR);
                    }
                    dl->AddConvexPolyFilled(pts, N_SEGS + 2, IM_COL32(0, 0, 0, 110));
                }
            }
        }

        // Auto-attack active glow - pulsing golden border when slot 6603 (Attack) is toggled on
        if (slot.type == game::ActionBarSlot::SPELL && slot.id == game::SPELL_ID_ATTACK
            && gameHandler.isAutoAttacking()) {
            ImVec2 bMin = ImGui::GetItemRectMin();
            ImVec2 bMax = ImGui::GetItemRectMax();
            float pulse = 0.55f + 0.45f * std::sin(static_cast<float>(ImGui::GetTime()) * 5.0f);
            ImU32 glowCol = IM_COL32(
                static_cast<int>(255),
                static_cast<int>(200 * pulse),
                static_cast<int>(0),
                static_cast<int>(200 * pulse));
            ImGui::GetWindowDrawList()->AddRect(bMin, bMax, glowCol, 2.0f, 0, 2.5f);
        }

        // Item stack count overlay - bottom-right corner of icon
        if (slot.type == game::ActionBarSlot::ITEM && slot.id != 0) {
            const auto countIt = actionItemCounts.find(slot.id);
            const int totalCount = countIt != actionItemCounts.end()
                ? static_cast<int>(countIt->second) : 0;
            if (totalCount > 0) {
                char countStr[16];
                snprintf(countStr, sizeof(countStr), "%d", totalCount);
                ImVec2 btnMax = ImGui::GetItemRectMax();
                ImVec2 tsz = ImGui::CalcTextSize(countStr);
                float cx2 = btnMax.x - tsz.x - 2.0f;
                float cy2 = btnMax.y - tsz.y - 1.0f;
                auto* cdl = ImGui::GetWindowDrawList();
                cdl->AddText(ImVec2(cx2 + 1.0f, cy2 + 1.0f), IM_COL32(0, 0, 0, 200), countStr);
                cdl->AddText(ImVec2(cx2, cy2),
                    totalCount <= 1 ? IM_COL32(220, 100, 100, 255) : IM_COL32(255, 255, 255, 255),
                    countStr);
            }
        }

        // Ready glow: animate a gold border for ~1.5s when a cooldown just expires
        {
            static std::unordered_map<int, float> slotGlowTimers;    // absSlot -> remaining glow seconds
            static std::unordered_map<int, bool>  slotWasOnCooldown; // absSlot -> last frame state

            float dt = ImGui::GetIO().DeltaTime;
            bool wasOnCd = slotWasOnCooldown.count(absSlot) ? slotWasOnCooldown[absSlot] : false;

            // Trigger glow when transitioning from on-cooldown to ready (and slot isn't empty)
            if (wasOnCd && !onCooldown && !slot.isEmpty()) {
                slotGlowTimers[absSlot] = 1.5f;
            }
            slotWasOnCooldown[absSlot] = onCooldown;

            auto git = slotGlowTimers.find(absSlot);
            if (git != slotGlowTimers.end() && git->second > 0.0f) {
                git->second -= dt;
                float t = git->second / 1.5f; // 1.0 → 0.0 over lifetime
                // Pulse: bright when fresh, fading out
                float pulse = std::sin(t * IM_PI * 4.0f) * 0.5f + 0.5f; // 4 pulses
                uint8_t alpha = static_cast<uint8_t>(200 * t * (0.5f + 0.5f * pulse));
                if (alpha > 0) {
                    ImVec2 bMin = ImGui::GetItemRectMin();
                    ImVec2 bMax = ImGui::GetItemRectMax();
                    auto* gdl = ImGui::GetWindowDrawList();
                    // Gold glow border (2px inset, 3px thick)
                    gdl->AddRect(ImVec2(bMin.x - 2, bMin.y - 2),
                                 ImVec2(bMax.x + 2, bMax.y + 2),
                                 IM_COL32(255, 215, 0, alpha), 3.0f, 0, 3.0f);
                }
                if (git->second <= 0.0f) slotGlowTimers.erase(git);
            }
        }

        // Key label below
        ImGui::TextDisabled("%s", keyLabel);

        ImGui::PopID();
        ImGui::EndGroup();
    };

    // Bottom-left extra bar (FrameXML page 6, slots 60-71)
    if (settingsPanel.pendingShowActionBar2) {

        float bar2X = barX + settingsPanel.pendingActionBar2OffsetX;
        float bar2Y = barY - barH - 2.0f + settingsPanel.pendingActionBar2OffsetY;
        ImGui::SetNextWindowPos(ImVec2(bar2X, bar2Y), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(barW, barH), ImGuiCond_Always);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(padding, padding));
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0.0f, 0.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.05f, 0.05f, 0.05f, 0.85f));
        if (ImGui::Begin("##ActionBar2", nullptr, flags)) {
            for (int i = 0; i < game::GameHandler::SLOTS_PER_BAR; ++i) {
                if (i > 0) ImGui::SameLine(0, spacing);
                renderBarSlot(actionSlotForPage(kBottomLeftActionPage, i), "");
            }
        }
        ImGui::End();
        ImGui::PopStyleColor();
        ImGui::PopStyleVar(4);
    }

    // Main action bar (FrameXML pages 1-6)
    bool mainBarHovered = false;
    ImGui::SetNextWindowPos(ImVec2(barX, barY), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(barW, barH), ImGuiCond_Always);
    if (ImGui::Begin("##ActionBar", nullptr, flags)) {
        mainBarHovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem);
        for (int i = 0; i < game::GameHandler::SLOTS_PER_BAR; ++i) {
            if (i > 0) ImGui::SameLine(0, spacing);
            renderBarSlot(actionSlotForPage(mainActionBarPage_, i), keyLabels1[i]);
        }

        // Macro editor modal - opened by "Edit" in action bar context menus
        if (macroEditorOpen_) {
            ImGui::OpenPopup("Edit Macro###MacroEdit");
            macroEditorOpen_ = false;
        }
        if (ImGui::BeginPopupModal("Edit Macro###MacroEdit", nullptr,
                                   ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse)) {
            ImGui::Text("Macro #%u  (all lines execute; [cond] Spell; Default supported)", macroEditorId_);
            ImGui::SetNextItemWidth(320.0f);
            ImGui::InputTextMultiline("##MacroText", macroEditorBuf_, sizeof(macroEditorBuf_),
                                      ImVec2(320.0f, 80.0f));
            if (ImGui::Button("Save")) {
                gameHandler.setMacroText(macroEditorId_, std::string(macroEditorBuf_));
                macroPrimarySpellCache_.clear();  // invalidate resolved spell IDs
                macroRenderCache_.clear();
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel")) {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }
    }
    ImGui::End();

    if (mainBarHovered) {
        float wheel = ImGui::GetIO().MouseWheel;
        if (wheel > 0.0f) changeMainPage(-1);
        else if (wheel < 0.0f) changeMainPage(1);
    }

    const float pagerW = 34.0f;
    ImGui::SetNextWindowPos(ImVec2(barX + barW + 4.0f, barY), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(pagerW, barH), ImGuiCond_Always);
    if (ImGui::Begin("##ActionBarPager", nullptr, flags)) {
        const float buttonH = std::max(16.0f, (barH - padding * 2.0f - 18.0f) * 0.5f);
        if (ImGui::Button("^", ImVec2(pagerW - padding * 2.0f, buttonH))) {
            changeMainPage(-1);
        }
        char pageText[8];
        snprintf(pageText, sizeof(pageText), "%d/%d", mainActionBarPage_, kFrameXmlActionBarPages);
        ImVec2 textSize = ImGui::CalcTextSize(pageText);
        ImGui::SetCursorPosX((pagerW - textSize.x) * 0.5f);
        ImGui::TextUnformatted(pageText);
        if (ImGui::Button("v", ImVec2(pagerW - padding * 2.0f, buttonH))) {
            changeMainPage(1);
        }
    }
    ImGui::End();

    ImGui::PopStyleColor();
    ImGui::PopStyleVar(4);

    // Right side vertical bar (FrameXML page 3, slots 24-35)
    if (settingsPanel.pendingShowRightBar) {
        bool bar3HasContent = false;
        for (int i = 0; i < game::GameHandler::SLOTS_PER_BAR; ++i)
            if (!bar[actionSlotForPage(kRightActionPage, i)].isEmpty()) { bar3HasContent = true; break; }

        float sideBarW = slotSize + padding * 2;
        float sideBarH = game::GameHandler::SLOTS_PER_BAR * slotSize + (game::GameHandler::SLOTS_PER_BAR - 1) * spacing + padding * 2;
        float sideBarX = screenW - sideBarW - 4.0f;
        float sideBarY = (screenH - sideBarH) / 2.0f + settingsPanel.pendingRightBarOffsetY;

        ImGui::SetNextWindowPos(ImVec2(sideBarX, sideBarY), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(sideBarW, sideBarH), ImGuiCond_Always);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(padding, padding));
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0.0f, 0.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        ImGui::PushStyleColor(ImGuiCol_WindowBg,
            bar3HasContent ? ImVec4(0.05f, 0.05f, 0.05f, 0.85f) : ImVec4(0.05f, 0.05f, 0.05f, 0.4f));
        if (ImGui::Begin("##ActionBarRight", nullptr, flags)) {
            for (int i = 0; i < game::GameHandler::SLOTS_PER_BAR; ++i) {
                renderBarSlot(actionSlotForPage(kRightActionPage, i), "");
            }
        }
        ImGui::End();
        ImGui::PopStyleColor();
        ImGui::PopStyleVar(4);
    }

    // Left side vertical bar (FrameXML page 4, slots 36-47)
    if (settingsPanel.pendingShowLeftBar) {
        bool bar4HasContent = false;
        for (int i = 0; i < game::GameHandler::SLOTS_PER_BAR; ++i)
            if (!bar[actionSlotForPage(kLeftActionPage, i)].isEmpty()) { bar4HasContent = true; break; }

        float sideBarW = slotSize + padding * 2;
        float sideBarH = game::GameHandler::SLOTS_PER_BAR * slotSize + (game::GameHandler::SLOTS_PER_BAR - 1) * spacing + padding * 2;
        float sideBarX = 4.0f;
        float sideBarY = (screenH - sideBarH) / 2.0f + settingsPanel.pendingLeftBarOffsetY;

        ImGui::SetNextWindowPos(ImVec2(sideBarX, sideBarY), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(sideBarW, sideBarH), ImGuiCond_Always);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(padding, padding));
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0.0f, 0.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        ImGui::PushStyleColor(ImGuiCol_WindowBg,
            bar4HasContent ? ImVec4(0.05f, 0.05f, 0.05f, 0.85f) : ImVec4(0.05f, 0.05f, 0.05f, 0.4f));
        if (ImGui::Begin("##ActionBarLeft", nullptr, flags)) {
            for (int i = 0; i < game::GameHandler::SLOTS_PER_BAR; ++i) {
                renderBarSlot(actionSlotForPage(kLeftActionPage, i), "");
            }
        }
        ImGui::End();
        ImGui::PopStyleColor();
        ImGui::PopStyleVar(4);
    }

    // Vehicle exit button (WotLK): floating button above action bar when player is in a vehicle
    if (gameHandler.isInVehicle()) {
        const float btnW = 120.0f;
        const float btnH = 32.0f;
        const float btnX = (screenW - btnW) / 2.0f;
        const float btnY = barY - btnH - 6.0f;

        ImGui::SetNextWindowPos(ImVec2(btnX, btnY), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(btnW, btnH), ImGuiCond_Always);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(4.0f, 4.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
        ImGuiWindowFlags vFlags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                                  ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                                  ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoBackground;
        if (ImGui::Begin("##VehicleExit", nullptr, vFlags)) {
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.6f, 0.1f, 0.1f, 0.9f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, colors::kLowHealthRed);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.4f, 0.0f, 0.0f, 1.0f));
            ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 4.0f);
            if (ImGui::Button("Dismount", ImVec2(btnW - 8.0f, btnH - 8.0f))) {
                gameHandler.sendRequestVehicleExit();
            }
            ImGui::PopStyleVar();
            ImGui::PopStyleColor(3);
        }
        ImGui::End();
        ImGui::PopStyleColor();
        ImGui::PopStyleVar(3);
    }

    // Handle action bar drag: render icon at cursor and detect drop outside
    if (actionBarDragSlot_ >= 0) {
        ImVec2 mousePos = ImGui::GetMousePos();

        // Draw dragged icon at cursor
        if (actionBarDragIcon_) {
            ImGui::GetForegroundDrawList()->AddImage(
                (ImTextureID)(uintptr_t)actionBarDragIcon_,
                ImVec2(mousePos.x - 20, mousePos.y - 20),
                ImVec2(mousePos.x + 20, mousePos.y + 20));
        } else {
            ImGui::GetForegroundDrawList()->AddRectFilled(
                ImVec2(mousePos.x - 20, mousePos.y - 20),
                ImVec2(mousePos.x + 20, mousePos.y + 20),
                IM_COL32(80, 80, 120, 180));
        }

        if (actionBarCarryPressActive_) {
            // Still waiting for the initiating hold-press to be released - swallow it
            // so it doesn't count as a drop, then the carry is "armed" for real clicks.
            if (ImGui::IsMouseReleased(ImGuiMouseButton_Left))
                actionBarCarryPressActive_ = false;
        } else {
            bool insideBar = (mousePos.x >= barX && mousePos.x <= barX + barW &&
                              mousePos.y >= barY && mousePos.y <= barY + barH);

            // Helper: put the carried action back where it was lifted from.
            auto restoreCarried = [&]() {
                gameHandler.setActionBarSlot(
                    actionBarDragSlot_,
                    static_cast<game::ActionBarSlot::Type>(actionBarCarryType_),
                    actionBarCarryId_);
            };
            auto endCarry = [&]() {
                actionBarDragSlot_ = -1;
                actionBarDragIcon_ = 0;
                actionBarCarryType_ = 0;
                actionBarCarryId_ = 0;
            };

            // Left-release off the bar removes the action, which is what
            // dragging something off a bar means and what the real client
            // does. The origin slot was emptied at pickup, so there is nothing
            // left to clear. A left-release on a slot is consumed by the
            // per-slot drop logic above, which moves it there instead.
            if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) && !insideBar) {
                endCarry();
            }

            // Right-release puts it back where it was lifted from, wherever the
            // pointer is. The way out of a drag begun by mistake.
            if (ImGui::IsMouseReleased(ImGuiMouseButton_Right)) {
                restoreCarried();
                endCarry();
            }
        }
    }
}

void ActionBarPanel::renderStanceBar(game::GameHandler& gameHandler,
                             SettingsPanel& settingsPanel,
                             SpellbookScreen& spellbookScreen,
                             SpellIconFn getSpellIcon) {
    uint8_t playerClass = gameHandler.getPlayerClass();

    // The same list FrameXML's stance bar is built from, in the same order.
    //
    // This kept its own table of the five classes' forms and the Ctrl+1..8
    // bindings read that. It disagreed with this one: nine druid entries in a
    // different order against eight, because Dire Bear replaces Bear on a
    // button rather than adding one. So the key pressed a different form than
    // the bar drew, which is the hazard a shared order exists to stop.
    const auto forms = game::knownShapeshiftForms(playerClass,
                                                  gameHandler.getKnownSpells());
    if (forms.empty()) return;

    std::vector<uint32_t> available;
    available.reserve(forms.size());
    for (const auto& form : forms) available.push_back(form.spellId);

    // Detect active stance from permanent player auras (maxDurationMs == -1)
    uint32_t activeStance = 0;
    for (const auto& aura : gameHandler.getPlayerAuras()) {
        if (aura.isEmpty() || aura.maxDurationMs != -1) continue;
        for (uint32_t sid : available) {
            if (aura.spellId == sid) { activeStance = sid; break; }
        }
        if (activeStance) break;
    }

    ImVec2 displaySize = ImGui::GetIO().DisplaySize;
    float screenW = displaySize.x > 0.0f ? displaySize.x : 1280.0f;
    float screenH = displaySize.y > 0.0f ? displaySize.y : 720.0f;
    auto* assetMgr = services_.assetManager;

    // Match the action bar slot size so they align neatly
    float slotSize = 38.0f;
    float spacing  = 4.0f;
    float padding  = 6.0f;
    int   count    = static_cast<int>(available.size());

    float barW = count * slotSize + (count - 1) * spacing + padding * 2.0f;
    float barH = slotSize + padding * 2.0f;

    // Position the stance bar immediately to the left of the action bar
    float actionSlot = 48.0f * settingsPanel.pendingActionBarScale;
    float actionBarW = 12.0f * actionSlot + 11.0f * 4.0f + 8.0f * 2.0f;
    float actionBarX = (screenW - actionBarW) / 2.0f;
    float actionBarH = actionSlot + 24.0f;
    float actionBarY = screenH - actionBarH;

    float barX = actionBarX - barW - 8.0f;
    float barY = actionBarY + (actionBarH - barH) / 2.0f;

    ImGui::SetNextWindowPos(ImVec2(barX, barY), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(barW, barH), ImGuiCond_Always);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                             ImGuiWindowFlags_NoScrollbar;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(padding, padding));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0.0f, 0.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.05f, 0.05f, 0.05f, 0.9f));

    if (ImGui::Begin("##StanceBar", nullptr, flags)) {
        ImDrawList* dl = ImGui::GetWindowDrawList();

        for (int i = 0; i < count; ++i) {
            if (i > 0) ImGui::SameLine(0.0f, spacing);
            ImGui::PushID(i);

            uint32_t spellId = available[i];
            bool isActive = (spellId == activeStance);

            VkDescriptorSet iconTex = assetMgr ? getSpellIcon(spellId, assetMgr) : VK_NULL_HANDLE;

            ImVec2 pos = ImGui::GetCursorScreenPos();
            ImVec2 posEnd = ImVec2(pos.x + slotSize, pos.y + slotSize);

            // Background - green tint when active
            ImU32 bgCol     = isActive ? IM_COL32(30, 70, 30, 230) : IM_COL32(20, 20, 20, 220);
            ImU32 borderCol = isActive ? IM_COL32(80, 220, 80, 255) : IM_COL32(80, 80, 80, 200);
            dl->AddRectFilled(pos, posEnd, bgCol, 4.0f);

            if (iconTex) {
                dl->AddImage((ImTextureID)(uintptr_t)iconTex, pos, posEnd);
                // Darken inactive buttons slightly
                if (!isActive)
                    dl->AddRectFilled(pos, posEnd, IM_COL32(0, 0, 0, 70), 4.0f);
            }
            dl->AddRect(pos, posEnd, borderCol, 4.0f, 0, 2.0f);

            ImGui::InvisibleButton("##btn", ImVec2(slotSize, slotSize));

            if (ImGui::IsItemClicked(ImGuiMouseButton_Left))
                gameHandler.castSpell(spellId);

            if (ImGui::IsItemHovered()) {
                ImGui::BeginTooltip();
                std::string name = spellbookScreen.lookupSpellName(spellId, assetMgr);
                if (!name.empty()) ImGui::TextUnformatted(name.c_str());
                else               ImGui::Text("Spell #%u", spellId);
                ImGui::EndTooltip();
            }

            ImGui::PopID();
        }
    }
    ImGui::End();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar(4);
}

namespace {
/// Where a bag-bar click goes when FrameXML owns the bags.
///
/// renderBagBar is gated on the BagBar element and these buttons open the
/// *bags*, which is a different element - so wherever the two are owned
/// differently, this client drew its own bag bar and every button on it
/// toggled a window FrameXML draws and this client does not.
///
/// That needs a hand-picked WOWEE_FRAMEXML_UI. The defaults and "candidates"
/// both name mainmenubar, which covers BagBar as well as the action bar and
/// the micro menu, so this bar is not drawn in either.
///
/// The container ids are WoW's: zero is the backpack and one to four are the
/// equipped bags, where this client indexes those four from zero.
bool bagsAreFrameXml() { return wowee::ui::frameXmlOwns(wowee::ui::UiElement::Bags); }
}  // namespace

bool ActionBarPanel::renderBagBar(game::GameHandler& gameHandler,
                         SettingsPanel& settingsPanel,
                         InventoryScreen& inventoryScreen) {
    ImVec2 displaySize = ImGui::GetIO().DisplaySize;
    float screenW = displaySize.x > 0.0f ? displaySize.x : 1280.0f;
    float screenH = displaySize.y > 0.0f ? displaySize.y : 720.0f;
    auto* assetMgr = services_.assetManager;
    bool settingChanged = false;

    float slotSize = 42.0f;
    float spacing = 4.0f;
    float padding = 6.0f;

    // Mode toggle + backpack + 4 bags
    float barW = 6 * slotSize + 5 * spacing + padding * 2;
    float barH = slotSize + padding * 2;

    // Position in bottom right corner
    float barX = screenW - barW - 10.0f;
    float barY = screenH - barH - 10.0f;

    ImGui::SetNextWindowPos(ImVec2(barX, barY), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(barW, barH), ImGuiCond_Always);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                             ImGuiWindowFlags_NoScrollbar;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(padding, padding));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0.0f, 0.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.05f, 0.05f, 0.05f, 0.9f));

    if (ImGui::Begin("##BagBar", nullptr, flags)) {
        auto& inv = gameHandler.getInventory();

        // Keep this next to the bags so changing layouts never requires a trip
        // through settings. The saved setting remains the single source of truth.
        const bool combined = !inventoryScreen.isSeparateBags();
        if (ImGui::Button(combined ? "Split" : "All", ImVec2(slotSize, slotSize))) {
            inventoryScreen.toggleCombinedBags();
            settingsPanel.pendingSeparateBags = inventoryScreen.isSeparateBags();
            settingChanged = true;
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip(combined
                ? "Split into separate bag windows"
                : "Combine all bag slots into one window");
        }

        // Load backpack icon if needed
        if (!backpackIconTexture_ && assetMgr && assetMgr->isInitialized()) {
            backpackIconTexture_ = uploadUiTextureFromBlp(
                assetMgr, "Interface\\Buttons\\Button-Backpack-Up.blp",
                services_.window);
        }

        // Track bag slot screen rects for drop detection
        ImVec2 bagSlotMins[4], bagSlotMaxs[4];

        // Slots 1-4: Bag slots (leftmost)
        for (int i = 0; i < 4; ++i) {
            ImGui::SameLine(0, spacing);
            ImGui::PushID(i + 1);

            game::EquipSlot bagSlot = static_cast<game::EquipSlot>(static_cast<int>(game::EquipSlot::BAG1) + i);
            const auto& bagItem = inv.getEquipSlot(bagSlot);

            VkDescriptorSet bagIcon = VK_NULL_HANDLE;
            if (!bagItem.empty() && bagItem.item.displayInfoId != 0) {
                bagIcon = inventoryScreen.getItemIcon(bagItem.item.displayInfoId);
            }
            // Render the slot as an invisible button so we control all interaction
            ImVec2 cpos = ImGui::GetCursorScreenPos();
            ImGui::InvisibleButton("##bagSlot", ImVec2(slotSize, slotSize));
            bagSlotMins[i] = cpos;
            bagSlotMaxs[i] = ImVec2(cpos.x + slotSize, cpos.y + slotSize);

            ImDrawList* dl = ImGui::GetWindowDrawList();

            // Draw background + icon
            if (bagIcon) {
                dl->AddRectFilled(bagSlotMins[i], bagSlotMaxs[i], IM_COL32(25, 25, 25, 230));
                dl->AddImage((ImTextureID)(uintptr_t)bagIcon, bagSlotMins[i], bagSlotMaxs[i]);
            } else {
                dl->AddRectFilled(bagSlotMins[i], bagSlotMaxs[i], IM_COL32(38, 38, 38, 204));
            }

            // Hover highlight
            bool hovered = ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem);
            if (hovered && bagBarPickedSlot_ < 0) {
                dl->AddRect(bagSlotMins[i], bagSlotMaxs[i], IM_COL32(255, 255, 255, 100));
            }

            // Track which slot was pressed for drag detection
            if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left) && bagBarPickedSlot_ < 0 && bagIcon) {
                bagBarDragSource_ = i;
            }

            // Click toggles bag open/close (handled in mouse release section below)

            // Dim the slot being dragged
            if (bagBarPickedSlot_ == i) {
                dl->AddRectFilled(bagSlotMins[i], bagSlotMaxs[i], IM_COL32(0, 0, 0, 150));
            }

            // Tooltip: bag name plus its capacity ("N Slot Bag"). getBagSize is the
            // authoritative slot count for the equipped bag, independent of whether the
            // item template query has arrived.
            if (hovered && bagBarPickedSlot_ < 0) {
                if (bagIcon) {
                    int bagSlots = inv.getBagSize(i);
                    if (bagSlots > 0)
                        ImGui::SetTooltip("%s\n%d Slot Bag", bagItem.item.name.c_str(), bagSlots);
                    else
                        ImGui::SetTooltip("%s", bagItem.item.name.c_str());
                } else {
                    ImGui::SetTooltip("Empty Bag Slot");
                }
            }

            // Open bag indicator
            if (inventoryScreen.isSeparateBags() && inventoryScreen.isBagOpen(i)) {
                dl->AddRect(bagSlotMins[i], bagSlotMaxs[i], IM_COL32(255, 255, 255, 255), 3.0f, 0, 2.0f);
            }

            // Right-click context menu
            if (ImGui::BeginPopupContextItem("##bagSlotCtx")) {
                if (!bagItem.empty()) {
                    ImGui::TextDisabled("%s", bagItem.item.name.c_str());
                    ImGui::Separator();
                    bool isOpen = inventoryScreen.isSeparateBags() && inventoryScreen.isBagOpen(i);
                    if (ImGui::MenuItem(isOpen ? "Close Bag" : "Open Bag")) {
                        if (bagsAreFrameXml())
                            gameHandler.runInterfaceCommand("ToggleBag(" + std::to_string(i + 1) + ")");
                        else if (inventoryScreen.isSeparateBags())
                            inventoryScreen.toggleBag(i);
                        else
                            inventoryScreen.toggle();
                    }
                    if (ImGui::MenuItem("Unequip Bag")) {
                        gameHandler.unequipToBackpack(bagSlot);
                    }
                } else {
                    ImGui::TextDisabled("Empty Bag Slot");
                }
                ImGui::EndPopup();
            }

            // Accept dragged item from inventory
            if (hovered && inventoryScreen.isHoldingItem()) {
                const auto& heldItem = inventoryScreen.getHeldItem();
                if ((heldItem.inventoryType == 18 || heldItem.bagSlots > 0) &&
                    ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                    auto& inventory = gameHandler.getInventory();
                    inventoryScreen.dropHeldItemToEquipSlot(inventory, bagSlot);
                }
            }

            ImGui::PopID();
        }

        // Drag lifecycle: press on a slot sets bagBarDragSource_,
        // dragging 3+ pixels promotes to bagBarPickedSlot_ (visual drag),
        // releasing completes swap or click
        if (bagBarDragSource_ >= 0) {
            if (ImGui::IsMouseDragging(ImGuiMouseButton_Left, 3.0f) && bagBarPickedSlot_ < 0) {
                // If an inventory window is open, hand off drag to inventory held-item
                // so the bag can be dropped into backpack/bag slots.
                if (inventoryScreen.isOpen() || inventoryScreen.isCharacterOpen()) {
                    auto equip = static_cast<game::EquipSlot>(
                        static_cast<int>(game::EquipSlot::BAG1) + bagBarDragSource_);
                    if (inventoryScreen.beginPickupFromEquipSlot(inv, equip)) {
                        bagBarDragSource_ = -1;
                    } else {
                        bagBarPickedSlot_ = bagBarDragSource_;
                    }
                } else {
                    // Mouse moved enough - start visual drag
                    bagBarPickedSlot_ = bagBarDragSource_;
                }
            }
            if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                if (bagBarPickedSlot_ >= 0) {
                    // Was dragging - check for drop target
                    ImVec2 mousePos = ImGui::GetIO().MousePos;
                    int dropTarget = -1;
                    for (int j = 0; j < 4; ++j) {
                        if (j == bagBarPickedSlot_) continue;
                        if (mousePos.x >= bagSlotMins[j].x && mousePos.x <= bagSlotMaxs[j].x &&
                            mousePos.y >= bagSlotMins[j].y && mousePos.y <= bagSlotMaxs[j].y) {
                            dropTarget = j;
                            break;
                        }
                    }
                    if (dropTarget >= 0) {
                        gameHandler.swapBagSlots(bagBarPickedSlot_, dropTarget);
                    }
                    bagBarPickedSlot_ = -1;
                } else {
                    // Was just a click (no drag) - toggle bag
                    int slot = bagBarDragSource_;
                    auto equip = static_cast<game::EquipSlot>(static_cast<int>(game::EquipSlot::BAG1) + slot);
                    if (!inv.getEquipSlot(equip).empty()) {
                        // The plain-click path, as opposed to the icon button
                        // and the context menu above. Same routing, and easy to
                        // miss because it lives in the drag handler.
                        if (bagsAreFrameXml())
                            gameHandler.runInterfaceCommand("ToggleBag(" + std::to_string(slot + 1) + ")");
                        else if (inventoryScreen.isSeparateBags())
                            inventoryScreen.toggleBag(slot);
                        else
                            inventoryScreen.toggle();
                    }
                }
                bagBarDragSource_ = -1;
            }
        }

        // Backpack (rightmost slot)
        ImGui::SameLine(0, spacing);
        ImGui::PushID(0);
        if (backpackIconTexture_) {
            if (ImGui::ImageButton("##backpack", (ImTextureID)(uintptr_t)backpackIconTexture_,
                                   ImVec2(slotSize, slotSize),
                                   ImVec2(0, 0), ImVec2(1, 1),
                                   ImVec4(0.1f, 0.1f, 0.1f, 0.9f),
                                   colors::kWhite)) {
                if (bagsAreFrameXml())
                    gameHandler.runInterfaceCommand("ToggleBackpack()");
                else if (inventoryScreen.isSeparateBags())
                    inventoryScreen.toggleBackpack();
                else
                    inventoryScreen.toggle();
            }
        } else {
            if (ImGui::Button("B", ImVec2(slotSize, slotSize))) {
                if (bagsAreFrameXml())
                    gameHandler.runInterfaceCommand("ToggleBackpack()");
                else if (inventoryScreen.isSeparateBags())
                    inventoryScreen.toggleBackpack();
                else
                    inventoryScreen.toggle();
            }
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Backpack");
        }
        // Right-click context menu on backpack
        if (ImGui::BeginPopupContextItem("##backpackCtx")) {
            bool isOpen = inventoryScreen.isSeparateBags() && inventoryScreen.isBackpackOpen();
            if (ImGui::MenuItem(isOpen ? "Close Backpack" : "Open Backpack")) {
                if (bagsAreFrameXml())
                    gameHandler.runInterfaceCommand("ToggleBackpack()");
                else if (inventoryScreen.isSeparateBags())
                    inventoryScreen.toggleBackpack();
                else
                    inventoryScreen.toggle();
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Open All Bags")) {
                if (bagsAreFrameXml()) gameHandler.runInterfaceCommand("OpenAllBags()");
                else                   inventoryScreen.openAllBags();
            }
            if (ImGui::MenuItem("Close All Bags")) {
                inventoryScreen.closeAllBags();
            }
            ImGui::EndPopup();
        }
        if (inventoryScreen.isSeparateBags() &&
            inventoryScreen.isBackpackOpen()) {
            ImDrawList* dl = ImGui::GetWindowDrawList();
            ImVec2 r0 = ImGui::GetItemRectMin();
            ImVec2 r1 = ImGui::GetItemRectMax();
            dl->AddRect(r0, r1, IM_COL32(255, 255, 255, 255), 3.0f, 0, 2.0f);
        }
        ImGui::PopID();

    }
    ImGui::End();

    ImGui::PopStyleColor();
    ImGui::PopStyleVar(4);

    // Draw dragged bag icon following cursor
    if (bagBarPickedSlot_ >= 0) {
        auto& inv2 = gameHandler.getInventory();
        auto pickedEquip = static_cast<game::EquipSlot>(
            static_cast<int>(game::EquipSlot::BAG1) + bagBarPickedSlot_);
        const auto& pickedItem = inv2.getEquipSlot(pickedEquip);
        VkDescriptorSet pickedIcon = VK_NULL_HANDLE;
        if (!pickedItem.empty() && pickedItem.item.displayInfoId != 0) {
            pickedIcon = inventoryScreen.getItemIcon(pickedItem.item.displayInfoId);
        }
        if (pickedIcon) {
            ImVec2 mousePos = ImGui::GetIO().MousePos;
            float sz = 40.0f;
            ImVec2 p0(mousePos.x - sz * 0.5f, mousePos.y - sz * 0.5f);
            ImVec2 p1(mousePos.x + sz * 0.5f, mousePos.y + sz * 0.5f);
            ImDrawList* fg = ImGui::GetForegroundDrawList();
            fg->AddImage((ImTextureID)(uintptr_t)pickedIcon, p0, p1);
            fg->AddRect(p0, p1, IM_COL32(200, 200, 200, 255), 0.0f, 0, 2.0f);
        }
    }
    return settingChanged;
}

void ActionBarPanel::renderXpBar(game::GameHandler& gameHandler,
                        SettingsPanel& settingsPanel) {
    uint32_t nextLevelXp  = gameHandler.getPlayerNextLevelXp();
    uint32_t playerLevel  = gameHandler.getPlayerLevel();
    // At max level, server sends nextLevelXp=0. Only skip entirely when we have
    // no level info at all (not yet logged in / no update-field data).
    const bool isMaxLevel = (nextLevelXp == 0 && playerLevel > 0);
    if (nextLevelXp == 0 && !isMaxLevel) return;

    uint32_t currentXp  = gameHandler.getPlayerXp();
    uint32_t restedXp   = gameHandler.getPlayerRestedXp();
    bool     isResting  = gameHandler.isPlayerResting();
    ImVec2 displaySize = ImGui::GetIO().DisplaySize;
    float screenW = displaySize.x > 0.0f ? displaySize.x : 1280.0f;
    float screenH = displaySize.y > 0.0f ? displaySize.y : 720.0f;
    auto* window = services_.window;
    (void)window;  // Not used for positioning; kept for AssetManager if needed

    // Position just above both action bars (bar1 at screenH-barH, bar2 above that)
    float slotSize = 48.0f * settingsPanel.pendingActionBarScale;
    float spacing = 4.0f;
    float padding = 8.0f;
    float barW = 12 * slotSize + 11 * spacing + padding * 2;
    float barH = slotSize + 24.0f;

    float xpBarH = 20.0f;
    float xpBarW = barW;
    float xpBarX = (screenW - xpBarW) / 2.0f;
    // XP bar sits just above whichever bar is topmost.
    // bar1 top edge: screenH - barH
    // bar2 top edge (when visible): bar1 top - barH - 2 + bar2 vertical offset
    float bar1TopY = screenH - barH;
    float xpBarY;
    bool bottomLeftBarVisible = settingsPanel.pendingShowActionBar2;
    if (bottomLeftBarVisible) {
        float bar2TopY = bar1TopY - barH - 2.0f + settingsPanel.pendingActionBar2OffsetY;
        xpBarY = bar2TopY - xpBarH - 2.0f;
    } else {
        xpBarY = bar1TopY - xpBarH - 2.0f;
    }

    ImGui::SetNextWindowPos(ImVec2(xpBarX, xpBarY), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(xpBarW, xpBarH + 4.0f), ImGuiCond_Always);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                             ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_AlwaysAutoResize;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 2.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(2.0f, 2.0f));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.05f, 0.05f, 0.05f, 0.9f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.3f, 0.3f, 0.3f, 0.8f));

    if (ImGui::Begin("##XpBar", nullptr, flags)) {
        ImVec2 barMin  = ImGui::GetCursorScreenPos();
        ImVec2 barSize = ImVec2(ImGui::GetContentRegionAvail().x, xpBarH - 4.0f);
        ImVec2 barMax  = ImVec2(barMin.x + barSize.x, barMin.y + barSize.y);
        auto*  drawList = ImGui::GetWindowDrawList();

        if (isMaxLevel) {
            // Max-level bar: fully filled in muted gold with "Max Level" label
            ImU32 bgML  = IM_COL32(15, 12, 5, 220);
            ImU32 fgML  = IM_COL32(180, 140, 40, 200);
            drawList->AddRectFilled(barMin, barMax, bgML, 2.0f);
            drawList->AddRectFilled(barMin, barMax, fgML, 2.0f);
            drawList->AddRect(barMin, barMax, IM_COL32(100, 80, 20, 220), 2.0f);
            const char* mlLabel = "Max Level";
            ImVec2 mlSz = ImGui::CalcTextSize(mlLabel);
            drawList->AddText(
                ImVec2(barMin.x + (barSize.x - mlSz.x) * 0.5f,
                       barMin.y + (barSize.y - mlSz.y) * 0.5f),
                IM_COL32(255, 230, 120, 255), mlLabel);
            ImGui::Dummy(barSize);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Level %u - Maximum level reached", playerLevel);
        } else {
        float pct = static_cast<float>(currentXp) / static_cast<float>(nextLevelXp);
        if (pct > 1.0f) pct = 1.0f;

        // Custom segmented XP bar (20 bubbles)
        ImU32 bg      = IM_COL32(15, 15, 20, 220);
        ImU32 fg      = IM_COL32(148, 51, 238, 255);
        ImU32 fgRest  = IM_COL32(200, 170, 255, 220); // lighter purple for rested portion
        ImU32 seg     = IM_COL32(35, 35, 45, 255);
        drawList->AddRectFilled(barMin, barMax, bg, 2.0f);
        drawList->AddRect(barMin, barMax, IM_COL32(80, 80, 90, 220), 2.0f);

        float fillW = barSize.x * pct;
        if (fillW > 0.0f) {
            drawList->AddRectFilled(barMin, ImVec2(barMin.x + fillW, barMax.y), fg, 2.0f);
        }

        // Rested XP overlay: draw from current XP fill to (currentXp + restedXp) fill
        if (restedXp > 0) {
            float restedEndPct = std::min(1.0f, static_cast<float>(currentXp + restedXp)
                                                / static_cast<float>(nextLevelXp));
            float restedStartX = barMin.x + fillW;
            float restedEndX   = barMin.x + barSize.x * restedEndPct;
            if (restedEndX > restedStartX) {
                drawList->AddRectFilled(ImVec2(restedStartX, barMin.y),
                                        ImVec2(restedEndX,   barMax.y),
                                        fgRest, 2.0f);
            }
        }

        const int segments = 20;
        float segW = barSize.x / static_cast<float>(segments);
        for (int i = 1; i < segments; ++i) {
            float x = barMin.x + segW * i;
            drawList->AddLine(ImVec2(x, barMin.y + 1.0f), ImVec2(x, barMax.y - 1.0f), seg, 1.0f);
        }

        // Rest indicator "zzz" to the right of the bar when resting
        if (isResting) {
            const char* zzz = "zzz";
            ImVec2 zSize = ImGui::CalcTextSize(zzz);
            float zx = barMax.x - zSize.x - 4.0f;
            float zy = barMin.y + (barSize.y - zSize.y) * 0.5f;
            drawList->AddText(ImVec2(zx, zy), IM_COL32(180, 150, 255, 220), zzz);
        }

        char overlay[96];
        if (restedXp > 0) {
            snprintf(overlay, sizeof(overlay), "%u / %u XP  (+%u rested)", currentXp, nextLevelXp, restedXp);
        } else {
            snprintf(overlay, sizeof(overlay), "%u / %u XP", currentXp, nextLevelXp);
        }
        ImVec2 textSize = ImGui::CalcTextSize(overlay);
        float tx = barMin.x + (barSize.x - textSize.x) * 0.5f;
        float ty = barMin.y + (barSize.y - textSize.y) * 0.5f;
        drawList->AddText(ImVec2(tx, ty), IM_COL32(230, 230, 230, 255), overlay);

        ImGui::Dummy(barSize);

        // Tooltip with XP-to-level and rested details
        if (ImGui::IsItemHovered()) {
            ImGui::BeginTooltip();
            uint32_t xpToLevel = (currentXp < nextLevelXp) ? (nextLevelXp - currentXp) : 0;
            ImGui::TextColored(ImVec4(0.9f, 0.85f, 1.0f, 1.0f), "Experience");
            ImGui::Separator();
            float xpPct = nextLevelXp > 0 ? (100.0f * currentXp / nextLevelXp) : 0.0f;
            ImGui::Text("Current: %u / %u XP (%.1f%%)", currentXp, nextLevelXp, xpPct);
            ImGui::Text("To next level: %u XP", xpToLevel);
            if (restedXp > 0) {
                float restedLevels = static_cast<float>(restedXp) / static_cast<float>(nextLevelXp);
                ImGui::Spacing();
                ImGui::TextColored(ImVec4(0.78f, 0.60f, 1.0f, 1.0f),
                    "Rested: +%u XP (%.1f%% of a level)", restedXp, restedLevels * 100.0f);
                if (isResting)
                    ImGui::TextColored(ImVec4(0.6f, 0.9f, 0.6f, 1.0f),
                        "Resting - accumulating bonus XP");
            }
            ImGui::EndTooltip();
        }
    }
    }
    ImGui::End();

    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar(2);
}

void ActionBarPanel::renderRepBar(game::GameHandler& gameHandler,
                         SettingsPanel& settingsPanel) {
    uint32_t factionId = gameHandler.getWatchedFactionId();
    if (factionId == 0) return;

    const auto& standings = gameHandler.getFactionStandings();
    auto it = standings.find(factionId);
    if (it == standings.end()) return;

    int32_t standing = it->second;

    // WoW reputation rank thresholds
    struct RepRank { const char* name; int32_t min; int32_t max; ImU32 color; };
    static const RepRank kRanks[] = {
        { "Hated",      -42000, -6001,  IM_COL32(180,  40,  40, 255) },
        { "Hostile",     -6000, -3001,  IM_COL32(180,  40,  40, 255) },
        { "Unfriendly",  -3000,    -1,  IM_COL32(220, 100,  50, 255) },
        { "Neutral",         0,  2999,  IM_COL32(200, 200,  60, 255) },
        { "Friendly",     3000,  8999,  IM_COL32( 60, 180,  60, 255) },
        { "Honored",      9000, 20999,  IM_COL32( 60, 160, 220, 255) },
        { "Revered",     21000, 41999,  IM_COL32(140,  80, 220, 255) },
        { "Exalted",     42000, 42999,  IM_COL32(255, 200,  50, 255) },
    };
    constexpr int kNumRanks = static_cast<int>(sizeof(kRanks) / sizeof(kRanks[0]));

    int rankIdx = kNumRanks - 1; // default to Exalted
    for (int i = 0; i < kNumRanks; ++i) {
        if (standing <= kRanks[i].max) { rankIdx = i; break; }
    }
    const RepRank& rank = kRanks[rankIdx];

    float fraction = 1.0f;
    if (rankIdx < kNumRanks - 1) {
        float range = static_cast<float>(rank.max - rank.min + 1);
        fraction = static_cast<float>(standing - rank.min) / range;
        fraction = std::max(0.0f, std::min(1.0f, fraction));
    }

    const std::string& factionName = gameHandler.getFactionNamePublic(factionId);

    // Position directly above the XP bar
    ImVec2 displaySize = ImGui::GetIO().DisplaySize;
    float screenW = displaySize.x > 0.0f ? displaySize.x : 1280.0f;
    float screenH = displaySize.y > 0.0f ? displaySize.y : 720.0f;

    float slotSize = 48.0f * settingsPanel.pendingActionBarScale;
    float spacing  = 4.0f;
    float padding  = 8.0f;
    float barW     = 12 * slotSize + 11 * spacing + padding * 2;
    float barH_ab  = slotSize + 24.0f;
    float xpBarH   = 20.0f;
    float repBarH  = 12.0f;
    float xpBarW   = barW;
    float xpBarX   = (screenW - xpBarW) / 2.0f;

    float bar1TopY = screenH - barH_ab;
    float xpBarY;
    bool bottomLeftBarVisible = settingsPanel.pendingShowActionBar2;
    if (bottomLeftBarVisible) {
        float bar2TopY = bar1TopY - barH_ab - 2.0f + settingsPanel.pendingActionBar2OffsetY;
        xpBarY = bar2TopY - xpBarH - 2.0f;
    } else {
        xpBarY = bar1TopY - xpBarH - 2.0f;
    }
    float repBarY = xpBarY - repBarH - 2.0f;

    ImGui::SetNextWindowPos(ImVec2(xpBarX, repBarY), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(xpBarW, repBarH + 4.0f), ImGuiCond_Always);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                             ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_AlwaysAutoResize;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 2.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(2.0f, 2.0f));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.05f, 0.05f, 0.05f, 0.9f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.3f, 0.3f, 0.3f, 0.8f));

    if (ImGui::Begin("##RepBar", nullptr, flags)) {
        ImVec2 barMin  = ImGui::GetCursorScreenPos();
        ImVec2 barSize = ImVec2(ImGui::GetContentRegionAvail().x, repBarH - 4.0f);
        ImVec2 barMax  = ImVec2(barMin.x + barSize.x, barMin.y + barSize.y);
        auto* dl = ImGui::GetWindowDrawList();

        dl->AddRectFilled(barMin, barMax, IM_COL32(15, 15, 20, 220), 2.0f);
        dl->AddRect(barMin, barMax, IM_COL32(80, 80, 90, 220), 2.0f);

        float fillW = barSize.x * fraction;
        if (fillW > 0.0f)
            dl->AddRectFilled(barMin, ImVec2(barMin.x + fillW, barMax.y), rank.color, 2.0f);

        // Label: "FactionName - Rank"
        char label[96];
        snprintf(label, sizeof(label), "%s - %s", factionName.c_str(), rank.name);
        ImVec2 textSize = ImGui::CalcTextSize(label);
        float tx = barMin.x + (barSize.x - textSize.x) * 0.5f;
        float ty = barMin.y + (barSize.y - textSize.y) * 0.5f;
        dl->AddText(ImVec2(tx, ty), IM_COL32(230, 230, 230, 255), label);

        // Tooltip with exact values on hover
        ImGui::Dummy(barSize);
        if (ImGui::IsItemHovered()) {
            ImGui::BeginTooltip();
            float cr = ((rank.color      ) & 0xFF) / 255.0f;
            float cg = ((rank.color >>  8) & 0xFF) / 255.0f;
            float cb = ((rank.color >> 16) & 0xFF) / 255.0f;
            ImGui::TextColored(ImVec4(cr, cg, cb, 1.0f), "%s", rank.name);
            int32_t rankMin = rank.min;
            int32_t rankMax = (rankIdx < kNumRanks - 1) ? rank.max : 42000;
            ImGui::Text("%s: %d / %d", factionName.c_str(), standing - rankMin, rankMax - rankMin + 1);
            ImGui::EndTooltip();
        }
    }
    ImGui::End();
    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar(2);
}


} // namespace ui
} // namespace wowee
