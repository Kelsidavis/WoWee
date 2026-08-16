// ============================================================
// Item tooltip - moved out of InventoryScreen, which was the only reason
// every other surface that draws an item held a reference to the bag window.
// ============================================================
#include "ui/item_tooltip.hpp"

#include "ui/ui_colors.hpp"
#include "ui/ui_texture_load.hpp"
#include "ui/inventory_screen.hpp"
#include "game/game_handler.hpp"
#include "game/inventory.hpp"
#include "game/inventory_slots.hpp"
#include "game/reputation_standing.hpp"
#include "core/application.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_loader.hpp"
#include "pipeline/dbc_layout.hpp"
#include "core/logger.hpp"
#include <imgui.h>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <vector>

namespace wowee {
namespace ui {

namespace {

using namespace wowee::ui::colors;

bool canUseItemType(const game::GameHandler& gameHandler,
                    uint32_t itemClass,
                    uint32_t subClass) {
    if (itemClass == 2) { // Weapon
        if (gameHandler.getWeaponProficiency() == 0)
            return true;
        return gameHandler.canUseWeaponSubclass(subClass);
    }

    if (itemClass == 4 && subClass > 0) {
        // Normal armor subclass proficiency is not a reliable tooltip-level
        // rejection across Classic/TBC/WotLK and private cores. For example,
        // cloaks and some belts can be reported as cloth/leather subclass
        // while still being legally equippable. Keep the hard type warning for
        // shields, where the armor proficiency bit is a distinct equip rule.
        if (subClass == 6 && gameHandler.getArmorProficiency() != 0)
            return gameHandler.canUseArmorSubclass(subClass);
    }

    return true;
}

/// The item's icon, through the shared cache.
VkDescriptorSet itemIcon(uint32_t displayInfoId, pipeline::AssetManager* assetManager) {
    return itemIconTexture(displayInfoId, assetManager,
                           core::Application::getInstance().getWindow());
}

// Render "Classes: Warrior, Paladin" or "Races: Human, Orc" restriction text.
// Shared between quest info and item info tooltips - both use the same WoW
// allowableClass/allowableRace bitmask format with identical display logic.
void renderClassRestriction(uint32_t allowableMask, uint8_t playerClass) {
    const auto& entries = ui::kClassMasks;
    int mc = 0;
    for (const auto& e : entries) if (allowableMask & e.mask) ++mc;
    if (mc <= 0 || mc >= 10) return;  // all classes allowed or none matched
    char buf[128] = "Classes: "; bool first = true;
    for (const auto& e : entries) {
        if (!(allowableMask & e.mask)) continue;
        if (!first) strncat(buf, ", ", sizeof(buf) - strlen(buf) - 1);
        strncat(buf, e.name, sizeof(buf) - strlen(buf) - 1);
        first = false;
    }
    uint32_t pm = (playerClass > 0 && playerClass <= 10) ? (1u << (playerClass - 1)) : 0;
    bool ok = (pm == 0 || (allowableMask & pm));
    ImGui::TextColored(ok ? ImVec4(1,1,1,0.75f) : colors::kPaleRed, "%s", buf);
}

void renderRaceRestriction(uint32_t allowableMask, uint8_t playerRace) {
    constexpr uint32_t kAllPlayable = 1|2|4|8|16|32|64|128|512|1024;
    if ((allowableMask & kAllPlayable) == kAllPlayable) return;
    const auto& entries = ui::kRaceMasks;
    int mc = 0;
    for (const auto& e : entries) if (allowableMask & e.mask) ++mc;
    if (mc <= 0) return;
    char buf[160] = "Races: "; bool first = true;
    for (const auto& e : entries) {
        if (!(allowableMask & e.mask)) continue;
        if (!first) strncat(buf, ", ", sizeof(buf) - strlen(buf) - 1);
        strncat(buf, e.name, sizeof(buf) - strlen(buf) - 1);
        first = false;
    }
    uint32_t pm = (playerRace > 0 && playerRace <= 11) ? (1u << (playerRace - 1)) : 0;
    bool ok = (pm == 0 || (allowableMask & pm));
    ImGui::TextColored(ok ? ImVec4(1,1,1,0.75f) : colors::kPaleRed, "%s", buf);
}

void renderItemTypeWarningIfNeeded(const game::GameHandler* gameHandler,
                                   uint32_t itemClass,
                                   uint32_t subClass) {
    if (!gameHandler)
        return;

    if (!canUseItemType(*gameHandler, itemClass, subClass))
        ImGui::TextColored(ui::colors::kBrightRed, "You can't use this type of item.");
}

struct ComparableEquipped {
    const game::ItemSlot* slot = nullptr;
    game::EquipSlot equipSlot = game::EquipSlot::HEAD;
    explicit operator bool() const { return slot != nullptr; }
};

ComparableEquipped findComparableEquipped(const game::Inventory& inventory, uint8_t inventoryType) {
    // Which slots to try is WoW's mapping and lives beside the enum; this only
    // walks them and stops at the first one holding something.
    for (game::EquipSlot slot : game::comparableEquipSlots(inventoryType)) {
        const auto& s = inventory.getEquipSlot(slot);
        if (!s.empty()) return ComparableEquipped{&s, slot};
    }
    return {};
}

void renderEquippedEnhancements(
        game::GameHandler* gameHandler,
        const ComparableEquipped& equipped,
        const std::unordered_map<uint32_t, std::string>& enchantNames) {
    if (!gameHandler || !equipped) return;

    const uint64_t guid = gameHandler->getEquipSlotGuid(
        static_cast<int>(equipped.equipSlot));
    if (guid == 0) return;

    auto renderEnchant = [&](const char* label, uint32_t enchantId, const ImVec4& color) {
        if (enchantId == 0) return;
        auto it = enchantNames.find(enchantId);
        if (it != enchantNames.end() && !it->second.empty())
            ImGui::TextColored(color, "%s: %s", label, it->second.c_str());
        else
            ImGui::TextColored(color, "%s: Enchantment %u", label, enchantId);
    };

    const auto [permanent, temporary] = gameHandler->getItemEnchantIds(guid);
    renderEnchant("Enchanted", permanent, ui::colors::kCyan);
    renderEnchant("Temporary", temporary, ImVec4(0.8f, 1.0f, 0.4f, 1.0f));

    const auto sockets = gameHandler->getItemSocketEnchantIds(guid);
    for (uint32_t socketEnchant : sockets)
        renderEnchant("Gem", socketEnchant, ui::colors::kSocketGreen);
}

/// The short stat words the side-by-side comparison uses.
///
/// game::itemStatName answers "Defense Rating" and "Crit Rating", which is what
/// a tooltip says. The comparison puts a name and two numbers on one narrow row,
/// so it says "Defense" and "Crit" - same ids, deliberately shorter words. It
/// was written out twice, identically.
const char* comparisonStatLabel(uint32_t statType) {
    switch (statType) {
        case 0:  return "Mana";
        case 1:  return "Health";
        case 12: return "Defense";
        case 13: return "Dodge";
        case 14: return "Parry";
        case 15: return "Block Rating";
        case 16: case 17: case 18: case 31: return "Hit";
        case 19: case 20: case 21: case 32: return "Crit";
        case 28: case 29: case 30: case 36: return "Haste";
        case 35: return "Resilience";
        case 37: return "Expertise";
        case 38: return "Attack Power";
        case 39: return "Ranged AP";
        case 41: return "Healing";
        case 42: return "Spell Damage";
        case 43: return "MP5";
        case 44: return "Armor Pen";
        case 45: return "Spell Power";
        case 46: return "HP5";
        case 47: return "Spell Pen";
        case 48: return "Block Value";
        default: return nullptr;
    }
}

}  // namespace

const std::unordered_map<uint32_t, std::string>& enchantmentNames(pipeline::AssetManager* assetManager) {
    static std::unordered_map<uint32_t, std::string> s_cache;
    static bool s_loaded = false;
    if (!s_loaded && assetManager && assetManager->isInitialized()) {
        s_loaded = true;
        auto dbc = assetManager->loadDBC("SpellItemEnchantment.dbc");
        if (dbc && dbc->isLoaded()) {
            const auto* lay = pipeline::getActiveDBCLayout()
                ? pipeline::getActiveDBCLayout()->getLayout("SpellItemEnchantment") : nullptr;
            uint32_t nf = pipeline::detectEnchantmentNameField(dbc.get(), lay);
            uint32_t fc = dbc->getFieldCount();
            for (uint32_t r = 0; r < dbc->getRecordCount(); ++r) {
                uint32_t eid = dbc->getUInt32(r, 0);
                if (eid == 0 || nf >= fc) continue;
                std::string en = dbc->getString(r, nf);
                if (!en.empty()) s_cache[eid] = std::move(en);
            }
        }
    }
    return s_cache;
}

void renderItemTooltip(const game::ItemDef& item, const game::Inventory* inventory,
                       uint64_t itemGuid, game::GameHandler* gameHandler,
                       pipeline::AssetManager* assetManager) {
    const auto& s_enchLookupB = enchantmentNames(assetManager);

    ImGui::BeginTooltip();

    ImVec4 qColor = ui::getQualityColor(item.quality);
    // Append the rolled random suffix (e.g. "of the Bear") so a randomly-enchanted item
    // reads with its full name, matching how the auction house already labels it.
    std::string displayName = item.name;
    if (item.randomPropertyId != 0 && gameHandler) {
        std::string suffix = gameHandler->getRandomPropertyName(item.randomPropertyId);
        if (!suffix.empty()) displayName += " " + suffix;
    }
    ImGui::TextColored(qColor, "%s", displayName.c_str());
    if (item.itemLevel > 0) {
        ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 0.7f), "Item Level %u", item.itemLevel);
    }

    // Heroic / Unique / Unique-Equipped indicators
    if (gameHandler) {
        const auto* qi = gameHandler->getItemInfo(item.itemId);
        if (qi && qi->valid) {
            constexpr uint32_t kFlagHeroic         = 0x8;
            constexpr uint32_t kFlagUniqueEquipped = 0x1000000;
            if (qi->itemFlags & kFlagHeroic) {
                ImGui::TextColored(ImVec4(0.0f, 0.8f, 0.0f, 1.0f), "Heroic");
            }
            if (qi->maxCount == 1) {
                ImGui::TextColored(ui::colors::kTooltipGold, "Unique");
            } else if (qi->itemFlags & kFlagUniqueEquipped) {
                ImGui::TextColored(ui::colors::kTooltipGold, "Unique-Equipped");
            }
        }
    }

    // Binding type
    ui::renderBindingType(item.bindType);

    if (item.itemId == 6948 && gameHandler) {
        uint32_t mapId = 0;
        glm::vec3 pos;
        if (gameHandler->getHomeBind(mapId, pos)) {
            std::string homeLocation;
            // Prefer the specific zone name from the bind-point zone ID
            uint32_t zoneId = gameHandler->getHomeBindZoneId();
            if (zoneId != 0)
                homeLocation = gameHandler->getWhoAreaName(zoneId);
            // Fall back to continent name if zone unavailable
            if (homeLocation.empty()) {
                const char* dn = core::WorldLoader::mapDisplayName(mapId);
                homeLocation = dn ? dn : "Unknown";
            }
            ImGui::TextColored(ImVec4(0.8f, 0.9f, 1.0f, 1.0f), "Home: %s", homeLocation.c_str());
        } else {
            ImGui::TextColored(ui::colors::kLightGray, "Home: not set");
        }
        ImGui::TextDisabled("Use: Teleport home");
    }

    // Slot type
    if (item.inventoryType > 0) {
        const char* slotName = ui::getInventorySlotName(item.inventoryType);
        const auto* qi = gameHandler ? gameHandler->getItemInfo(item.itemId) : nullptr;
        // Containers (bags, quivers, ammo pouches, specialty bags) show their capacity as
        // "N Slot Bag", matching the WoW bag tooltip. containerSlots comes from the item
        // template query; itemClass 1 = Container, 11 = Quiver/ammo.
        const bool isContainer = qi && qi->valid &&
                                 (qi->itemClass == 1 || qi->itemClass == 11) &&
                                 qi->containerSlots > 0;
        if (isContainer) {
            const char* bagType = !item.subclassName.empty() ? item.subclassName.c_str()
                                : (slotName[0] ? slotName : "Bag");
            ImGui::TextColored(ui::colors::kLightGray, "%u Slot %s", qi->containerSlots, bagType);
        } else if (slotName[0]) {
            if (!item.subclassName.empty()) {
                ImGui::TextColored(ui::colors::kLightGray, "%s  %s", slotName, item.subclassName.c_str());
            } else {
                ImGui::TextColored(ui::colors::kLightGray, "%s", slotName);
            }
        }

        if (qi && qi->valid)
            renderItemTypeWarningIfNeeded(gameHandler, qi->itemClass, qi->subClass);
    }

    const bool isWeapon = game::isWeaponInventoryType(item.inventoryType);

    // Compact stats view for weapons: damage range + speed + DPS
    ImVec4 green(0.0f, 1.0f, 0.0f, 1.0f);
    if (isWeapon && item.damageMax > 0.0f && item.delayMs > 0) {
        float speed = static_cast<float>(item.delayMs) / 1000.0f;
        float dps = ((item.damageMin + item.damageMax) * 0.5f) / speed;
        ImGui::Text("%.0f - %.0f Damage", item.damageMin, item.damageMax);
        ImGui::SameLine(160.0f);
        ImGui::TextDisabled("Speed %.2f", speed);
        ImGui::TextColored(ui::colors::kLightGray, "(%.1f damage per second)", dps);
    }

    // Armor appears before stat bonuses - matches WoW tooltip order
    if (item.armor > 0) {
        ImGui::Text("%d Armor", item.armor);
    }

    // Elemental resistances from item query cache (fire resist gear, nature resist gear, etc.)
    if (gameHandler) {
        const auto* qi = gameHandler->getItemInfo(item.itemId);
        if (qi && qi->valid) {
            const int32_t resValsI[6] = { qi->holyRes, qi->fireRes, qi->natureRes,
                                          qi->frostRes, qi->shadowRes, qi->arcaneRes };
            for (int i = 0; i < 6; ++i)
                if (resValsI[i] > 0) ImGui::Text("+%d %s", resValsI[i], game::resistanceSchoolName(static_cast<uint32_t>(i)));
        }
    }

    auto appendBonus = [](std::string& out, int32_t val, const char* shortName) {
        if (val <= 0) return;
        if (!out.empty()) out += "  ";
        out += "+" + std::to_string(val) + " ";
        out += shortName;
    };
    std::string bonusLine;
    appendBonus(bonusLine, item.strength, "Str");
    appendBonus(bonusLine, item.agility, "Agi");
    appendBonus(bonusLine, item.stamina, "Sta");
    appendBonus(bonusLine, item.intellect, "Int");
    appendBonus(bonusLine, item.spirit, "Spi");
    if (!bonusLine.empty()) {
        ImGui::TextColored(green, "%s", bonusLine.c_str());
    }

    // Extra stats (hit, crit, haste, AP, SP, etc.) - one line each
    for (const auto& es : item.extraStats) {
        const char* statName = game::itemStatName(es.statType);
        char buf[64];
        if (statName) {
            std::snprintf(buf, sizeof(buf), "%+d %s", es.statValue, statName);
        } else {
            std::snprintf(buf, sizeof(buf), "%+d (stat %u)", es.statValue, es.statType);
        }
        ImGui::TextColored(green, "%s", buf);
    }

    if (item.requiredLevel > 1) {
        uint32_t playerLvl = gameHandler ? gameHandler->getPlayerLevel() : 0;
        bool meetsReq = (playerLvl >= item.requiredLevel);
        ImVec4 reqColor = meetsReq ? ImVec4(1.0f, 1.0f, 1.0f, 0.75f) : ui::colors::kPaleRed;
        ImGui::TextColored(reqColor, "Requires Level %u", item.requiredLevel);
    }
    if (item.maxDurability > 0) {
        float durPct = static_cast<float>(item.curDurability) / static_cast<float>(item.maxDurability);
        const ImVec4 durColor = ui::colors::durabilityColor(durPct);
        ImGui::TextColored(durColor, "Durability %u / %u",
                           item.curDurability, item.maxDurability);
    }
    // Item spell effects (Use/Equip/Chance on Hit)
    if (gameHandler) {
        auto* info = gameHandler->getItemInfo(item.itemId);
        if (info) {
            for (const auto& sp : info->spells) {
                if (sp.spellId == 0) continue;
                const char* trigger = game::itemSpellTriggerText(sp.spellTrigger);
                if (!trigger) continue;
                const std::string& spDesc = gameHandler->getSpellDescription(sp.spellId);
                std::string spText = spDesc.empty()
                    ? gameHandler->getSpellName(sp.spellId)
                    : gameHandler->formatSpellDescription(sp.spellId, spDesc);
                if (!spText.empty()) {
                    ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + 320.0f);
                    ImGui::TextColored(ui::colors::kCyan,
                                       "%s: %s", trigger, spText.c_str());
                    ImGui::PopTextWrapPos();
                } else {
                    ImGui::TextColored(ui::colors::kCyan,
                                       "%s: Spell #%u", trigger, sp.spellId);
                }
            }
        }
    }

    // Skill / reputation requirements from item query cache
    if (gameHandler) {
        const auto* qInfo = gameHandler->getItemInfo(item.itemId);
        if (qInfo && qInfo->valid) {
            if (qInfo->requiredSkill != 0 && qInfo->requiredSkillRank > 0) {
                static std::unordered_map<uint32_t, std::string> s_skillNamesB;
                static bool s_skillNamesLoadedB = false;
                if (!s_skillNamesLoadedB && assetManager && assetManager->isInitialized()) {
                    s_skillNamesLoadedB = true;
                    auto dbc = assetManager->loadDBC("SkillLine.dbc");
                    if (dbc && dbc->isLoaded()) {
                        const auto* layout = pipeline::getActiveDBCLayout()
                            ? pipeline::getActiveDBCLayout()->getLayout("SkillLine") : nullptr;
                        uint32_t idF   = layout ? (*layout)["ID"]   : 0;
                        uint32_t nameF = layout ? (*layout)["Name"] : 2;
                        for (uint32_t r = 0; r < dbc->getRecordCount(); ++r) {
                            uint32_t sid = dbc->getUInt32(r, idF);
                            if (!sid) continue;
                            std::string sname = dbc->getString(r, nameF);
                            if (!sname.empty()) s_skillNamesB[sid] = std::move(sname);
                        }
                    }
                }
                uint32_t playerSkillVal = 0;
                const auto& skills = gameHandler->getPlayerSkills();
                auto skPit = skills.find(qInfo->requiredSkill);
                if (skPit != skills.end()) playerSkillVal = skPit->second.effectiveValue();
                bool meetsSkill = (playerSkillVal == 0 || playerSkillVal >= qInfo->requiredSkillRank);
                ImVec4 skColor = meetsSkill ? ImVec4(1.0f, 1.0f, 1.0f, 0.75f) : ui::colors::kPaleRed;
                auto skIt = s_skillNamesB.find(qInfo->requiredSkill);
                if (skIt != s_skillNamesB.end())
                    ImGui::TextColored(skColor, "Requires %s (%u)", skIt->second.c_str(), qInfo->requiredSkillRank);
                else
                    ImGui::TextColored(skColor, "Requires Skill %u (%u)", qInfo->requiredSkill, qInfo->requiredSkillRank);
            }
            if (qInfo->requiredReputationFaction != 0 && qInfo->requiredReputationRank > 0) {
                static std::unordered_map<uint32_t, std::string> s_factionNamesB;
                static bool s_factionNamesLoadedB = false;
                if (!s_factionNamesLoadedB && assetManager && assetManager->isInitialized()) {
                    s_factionNamesLoadedB = true;
                    auto dbc = assetManager->loadDBC("Faction.dbc");
                    if (dbc && dbc->isLoaded()) {
                        const auto* layout = pipeline::getActiveDBCLayout()
                            ? pipeline::getActiveDBCLayout()->getLayout("Faction") : nullptr;
                        uint32_t idF   = layout ? (*layout)["ID"]   : 0;
                        uint32_t nameF = layout ? (*layout)["Name"] : 20;
                        for (uint32_t r = 0; r < dbc->getRecordCount(); ++r) {
                            uint32_t fid = dbc->getUInt32(r, idF);
                            if (!fid) continue;
                            std::string fname = dbc->getString(r, nameF);
                            if (!fname.empty()) s_factionNamesB[fid] = std::move(fname);
                        }
                    }
                }
                const char* rankName = game::reputationRankName(qInfo->requiredReputationRank);
                auto fIt = s_factionNamesB.find(qInfo->requiredReputationFaction);
                ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 0.75f), "Requires %s with %s",
                    rankName,
                    fIt != s_factionNamesB.end() ? fIt->second.c_str() : "Unknown Faction");
            }
            if (qInfo->allowableClass != 0)
                renderClassRestriction(qInfo->allowableClass, gameHandler->getPlayerClass());
            if (qInfo->allowableRace != 0)
                renderRaceRestriction(qInfo->allowableRace, gameHandler->getPlayerRace());
        }
    }

    // Gem socket slots and item set - look up from query cache
    if (gameHandler) {
        const auto* qi2 = gameHandler->getItemInfo(item.itemId);
        if (qi2 && qi2->valid) {
            // Gem sockets
            {
                // Get socket gem enchant IDs for this item (filled from item update fields)
                std::array<uint32_t, 3> sockGems{};
                if (itemGuid != 0 && gameHandler)
                    sockGems = gameHandler->getItemSocketEnchantIds(itemGuid);

                bool hasSocket = false;
                for (int i = 0; i < 3; ++i) {
                    if (qi2->socketColor[i] == 0) continue;
                    if (!hasSocket) { ImGui::Spacing(); hasSocket = true; }
                    for (const auto& st : kSocketTypes) {
                        if (qi2->socketColor[i] & st.mask) {
                            if (sockGems[i] != 0) {
                                auto git = s_enchLookupB.find(sockGems[i]);
                                if (git != s_enchLookupB.end())
                                    ImGui::TextColored(st.col, "%s: %s", st.label, git->second.c_str());
                                else
                                    ImGui::TextColored(st.col, "%s: (gem %u)", st.label, sockGems[i]);
                            } else {
                                ImGui::TextColored(st.col, "%s", st.label);
                            }
                            break;
                        }
                    }
                }
                if (hasSocket && qi2->socketBonus != 0) {
                    auto enchIt = s_enchLookupB.find(qi2->socketBonus);
                    if (enchIt != s_enchLookupB.end())
                        ImGui::TextColored(ui::colors::kSocketGreen, "Socket Bonus: %s", enchIt->second.c_str());
                    else
                        ImGui::TextColored(ui::colors::kSocketGreen, "Socket Bonus: (id %u)", qi2->socketBonus);
                }
            }
            // Item set membership
            if (qi2->itemSetId != 0) {
                struct SetEntryD {
                    std::string name;
                    std::array<uint32_t, 10> itemIds{};
                    std::array<uint32_t, 10> spellIds{};
                    std::array<uint32_t, 10> thresholds{};
                };
                static std::unordered_map<uint32_t, SetEntryD> s_setDataD;
                static bool s_setDataLoadedD = false;
                if (!s_setDataLoadedD && assetManager && assetManager->isInitialized()) {
                    s_setDataLoadedD = true;
                    auto dbc = assetManager->loadDBC("ItemSet.dbc");
                    if (dbc && dbc->isLoaded()) {
                        const auto* layout = pipeline::getActiveDBCLayout()
                            ? pipeline::getActiveDBCLayout()->getLayout("ItemSet") : nullptr;
                        auto lf = [&](const char* k, uint32_t def) -> uint32_t {
                            return layout ? (*layout)[k] : def;
                        };
                        uint32_t idF = lf("ID", 0), nameF = lf("Name", 1);
                        const auto& itemKeys = ui::kItemSetItemKeys;
                        const auto& spellKeys = ui::kItemSetSpellKeys;
                        const auto& thrKeys = ui::kItemSetThresholdKeys;
                        uint32_t itemFB[10], spellFB[10], thrFB[10];
                        for (int i = 0; i < 10; ++i) {
                            itemFB[i] = 18+i; spellFB[i] = 28+i; thrFB[i] = 38+i;
                        }
                        for (uint32_t r = 0; r < dbc->getRecordCount(); ++r) {
                            uint32_t id = dbc->getUInt32(r, idF);
                            if (!id) continue;
                            SetEntryD e;
                            e.name = dbc->getString(r, nameF);
                            for (int i = 0; i < 10; ++i) {
                                e.itemIds[i]    = dbc->getUInt32(r, layout ? (*layout)[itemKeys[i]]  : itemFB[i]);
                                e.spellIds[i]   = dbc->getUInt32(r, layout ? (*layout)[spellKeys[i]] : spellFB[i]);
                                e.thresholds[i] = dbc->getUInt32(r, layout ? (*layout)[thrKeys[i]]   : thrFB[i]);
                            }
                            s_setDataD[id] = std::move(e);
                        }
                    }
                }
                auto setIt = s_setDataD.find(qi2->itemSetId);
                ImGui::Spacing();
                if (setIt != s_setDataD.end()) {
                    const SetEntryD& se = setIt->second;
                    int equipped = 0, total = 0;
                    for (int i = 0; i < 10; ++i) {
                        if (se.itemIds[i] == 0) continue;
                        ++total;
                        if (inventory) {
                            for (int s = 0; s < game::Inventory::NUM_EQUIP_SLOTS; s++) {
                                const auto& eSlot = inventory->getEquipSlot(static_cast<game::EquipSlot>(s));
                                if (!eSlot.empty() && eSlot.item.itemId == se.itemIds[i]) { ++equipped; break; }
                            }
                        }
                    }
                    if (total > 0) {
                        ImGui::TextColored(ui::colors::kTooltipGold,
                            "%s (%d/%d)", se.name.empty() ? "Set" : se.name.c_str(), equipped, total);
                    } else if (!se.name.empty()) {
                        ImGui::TextColored(ui::colors::kTooltipGold, "%s", se.name.c_str());
                    }
                    for (int i = 0; i < 10; ++i) {
                        if (se.spellIds[i] == 0 || se.thresholds[i] == 0) continue;
                        const std::string& bname = gameHandler->getSpellName(se.spellIds[i]);
                        bool active = (equipped >= static_cast<int>(se.thresholds[i]));
                        ImVec4 col = active ? ui::colors::kActiveGreen
                                           : ui::colors::kInactiveGray;
                        if (!bname.empty())
                            ImGui::TextColored(col, "(%u) %s", se.thresholds[i], bname.c_str());
                        else
                            ImGui::TextColored(col, "(%u) Set Bonus", se.thresholds[i]);
                    }
                } else {
                    ImGui::TextColored(ui::colors::kTooltipGold, "Set (id %u)", qi2->itemSetId);
                }
            }
        }
    }

    // Weapon/armor enchant display for equipped items (reads from item update fields)
    if (itemGuid != 0 && gameHandler) {
        auto [permId, tempId] = gameHandler->getItemEnchantIds(itemGuid);
        if (permId != 0) {
            auto it2 = s_enchLookupB.find(permId);
            const char* ename = (it2 != s_enchLookupB.end()) ? it2->second.c_str() : nullptr;
            if (ename) ImGui::TextColored(ui::colors::kCyan, "Enchanted: %s", ename);
        }
        if (tempId != 0) {
            auto it2 = s_enchLookupB.find(tempId);
            const char* ename = (it2 != s_enchLookupB.end()) ? it2->second.c_str() : nullptr;
            if (ename) ImGui::TextColored(ImVec4(0.8f, 1.0f, 0.4f, 1.0f), "%s (temporary)", ename);
        }
    }

    // "Begins a Quest" line (shown in yellow-green like the game)
    if (item.startQuestId != 0) {
        ImGui::TextColored(ui::colors::kTooltipGold, "Begins a Quest");
    }

    // Flavor / lore text (italic yellow in WoW, just yellow here)
    if (!item.description.empty()) {
        ImGui::TextColored(ImVec4(1.0f, 0.9f, 0.5f, 0.9f), "\"%s\"", item.description.c_str());
    }

    if (item.sellPrice > 0) {
        ImGui::TextDisabled("Sell:"); ImGui::SameLine(0, 4);
        renderCoinsFromCopper(item.sellPrice);
    }

    // Shift-hover comparison with currently equipped equivalent.
    if (inventory && ImGui::GetIO().KeyShift && item.inventoryType > 0) {
        if (const auto equipped = findComparableEquipped(*inventory, item.inventoryType)) {
            const game::ItemSlot* eq = equipped.slot;
            ImGui::Separator();
            ImGui::TextDisabled("Equipped:");
            VkDescriptorSet eqIcon = itemIcon(eq->item.displayInfoId, assetManager);
            if (eqIcon) {
                ImGui::Image((ImTextureID)(uintptr_t)eqIcon, ImVec2(18.0f, 18.0f));
                ImGui::SameLine();
            }
            ImGui::TextColored(ui::getQualityColor(eq->item.quality), "%s", eq->item.name.c_str());
            renderEquippedEnhancements(gameHandler, equipped, s_enchLookupB);

            // Item level comparison (always shown when different)
            if (eq->item.itemLevel > 0 || item.itemLevel > 0) {
                char ilvlBuf[64];
                float diff = static_cast<float>(item.itemLevel) - static_cast<float>(eq->item.itemLevel);
                if (diff > 0.0f)
                    std::snprintf(ilvlBuf, sizeof(ilvlBuf), "Item Level: %u (▲%.0f)", eq->item.itemLevel, diff);
                else if (diff < 0.0f)
                    std::snprintf(ilvlBuf, sizeof(ilvlBuf), "Item Level: %u (▼%.0f)", eq->item.itemLevel, -diff);
                else
                    std::snprintf(ilvlBuf, sizeof(ilvlBuf), "Item Level: %u (=)", eq->item.itemLevel);
                ImVec4 ilvlColor = (diff > 0.0f) ? ImVec4(0.0f, 1.0f, 0.0f, 1.0f)
                                 : (diff < 0.0f) ? ui::colors::kRed
                                 : ui::colors::kLightGray;
                ImGui::TextColored(ilvlColor, "%s", ilvlBuf);
            }

            // Helper: render a stat diff line showing the equipped item's value
            auto showDiff = [](const char* label, float newVal, float eqVal) {
                if (newVal == 0.0f && eqVal == 0.0f) return;
                float diff = newVal - eqVal;
                char buf[128];
                if (diff > 0.0f) {
                    std::snprintf(buf, sizeof(buf), "%s: %.0f (▲%.0f)", label, eqVal, diff);
                    ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "%s", buf);
                } else if (diff < 0.0f) {
                    std::snprintf(buf, sizeof(buf), "%s: %.0f (▼%.0f)", label, eqVal, -diff);
                    ImGui::TextColored(ui::colors::kRed, "%s", buf);
                } else {
                    std::snprintf(buf, sizeof(buf), "%s: %.0f (=)", label, eqVal);
                    ImGui::TextColored(ui::colors::kLightGray, "%s", buf);
                }
            };

            // DPS comparison for weapons
            if (game::isWeaponInventoryType(item.inventoryType) && game::isWeaponInventoryType(eq->item.inventoryType)) {
                float newDps = 0.0f, eqDps = 0.0f;
                if (item.damageMax > 0.0f && item.delayMs > 0)
                    newDps = ((item.damageMin + item.damageMax) * 0.5f) / (item.delayMs / 1000.0f);
                if (eq->item.damageMax > 0.0f && eq->item.delayMs > 0)
                    eqDps = ((eq->item.damageMin + eq->item.damageMax) * 0.5f) / (eq->item.delayMs / 1000.0f);
                showDiff("DPS", newDps, eqDps);
            }

            // Armor
            showDiff("Armor", static_cast<float>(item.armor), static_cast<float>(eq->item.armor));

            // Primary stats
            showDiff("Str",   static_cast<float>(item.strength),  static_cast<float>(eq->item.strength));
            showDiff("Agi",   static_cast<float>(item.agility),   static_cast<float>(eq->item.agility));
            showDiff("Sta",   static_cast<float>(item.stamina),   static_cast<float>(eq->item.stamina));
            showDiff("Int",   static_cast<float>(item.intellect), static_cast<float>(eq->item.intellect));
            showDiff("Spi",   static_cast<float>(item.spirit),    static_cast<float>(eq->item.spirit));

            // Extra stats diff - union of stat types from both items
            auto findExtraStat = [](const game::ItemDef& it, uint32_t type) -> int32_t {
                for (const auto& es : it.extraStats)
                    if (es.statType == type) return es.statValue;
                return 0;
            };
            // Collect all extra stat types
            std::vector<uint32_t> allTypes;
            allTypes.reserve(item.extraStats.size());
            for (const auto& es : item.extraStats) allTypes.push_back(es.statType);
            for (const auto& es : eq->item.extraStats) {
                bool found = false;
                for (uint32_t t : allTypes) if (t == es.statType) { found = true; break; }
                if (!found) allTypes.push_back(es.statType);
            }
            for (uint32_t t : allTypes) {
                int32_t nv = findExtraStat(item, t);
                int32_t ev = findExtraStat(eq->item, t);
                // Find a label for this stat type
                const char* lbl = comparisonStatLabel(t);
                if (!lbl) continue;
                showDiff(lbl, static_cast<float>(nv), static_cast<float>(ev));
            }
        }
    } else if (inventory && !ImGui::GetIO().KeyShift && item.inventoryType > 0) {
        if (findComparableEquipped(*inventory, item.inventoryType)) {
            ImGui::TextDisabled("Hold Shift to compare");
        }
    }

    // Destroy hint (not shown for quest items)
    if (item.itemId != 0 && item.bindType != 4) {
        ImGui::Spacing();
        if (ImGui::GetIO().KeyShift) {
            ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.45f, 0.9f), "Shift+RClick to destroy");
        } else {
            ImGui::TextDisabled("Shift+RClick to destroy");
        }
    }

    ImGui::EndTooltip();
}

// ---------------------------------------------------------------------------
// Tooltip overload for ItemQueryResponseData (used by loot window, etc.)
// ---------------------------------------------------------------------------
void renderItemTooltip(const game::ItemQueryResponseData& info, const game::Inventory* inventory,
                       uint64_t itemGuid, game::GameHandler* gameHandler,
                       pipeline::AssetManager* assetManager) {
    const auto& s_enchLookup = enchantmentNames(assetManager);

    ImGui::BeginTooltip();

    ImVec4 qColor = ui::getQualityColor(static_cast<game::ItemQuality>(info.quality));
    ImGui::TextColored(qColor, "%s", info.name.c_str());
    if (info.itemLevel > 0) {
        ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 0.7f), "Item Level %u", info.itemLevel);
    }

    // Unique / Heroic indicators
    constexpr uint32_t kFlagHeroic          = 0x8;         // ITEM_FLAG_HEROIC_TOOLTIP
    constexpr uint32_t kFlagUniqueEquipped  = 0x1000000;   // ITEM_FLAG_UNIQUE_EQUIPPABLE
    if (info.itemFlags & kFlagHeroic) {
        ImGui::TextColored(ImVec4(0.0f, 0.8f, 0.0f, 1.0f), "Heroic");
    }
    if (info.maxCount == 1) {
        ImGui::TextColored(ui::colors::kTooltipGold, "Unique");
    } else if (info.itemFlags & kFlagUniqueEquipped) {
        ImGui::TextColored(ui::colors::kTooltipGold, "Unique-Equipped");
    }

    // Binding type
    ui::renderBindingType(info.bindType);

    // Slot / subclass
    if (info.inventoryType > 0) {
        const char* slotName = ui::getInventorySlotName(info.inventoryType);
        // Containers (bags, quivers, ammo pouches) show capacity as "N Slot Bag" -
        // itemClass 1 = Container, 11 = Quiver/ammo. Same treatment as the ItemDef overload.
        const bool isContainer = (info.itemClass == 1 || info.itemClass == 11) &&
                                 info.containerSlots > 0;
        if (isContainer) {
            const char* bagType = !info.subclassName.empty() ? info.subclassName.c_str()
                                : (slotName[0] ? slotName : "Bag");
            ImGui::TextColored(ui::colors::kLightGray, "%u Slot %s", info.containerSlots, bagType);
        } else if (slotName[0]) {
            if (!info.subclassName.empty())
                ImGui::TextColored(ui::colors::kLightGray, "%s  %s", slotName, info.subclassName.c_str());
            else
                ImGui::TextColored(ui::colors::kLightGray, "%s", slotName);
        }

        renderItemTypeWarningIfNeeded(gameHandler, info.itemClass, info.subClass);
    }

    // Weapon stats
    auto isWeaponInvType = [](uint32_t t) {
        return t == 13 || t == 15 || t == 17 || t == 21 || t == 25 || t == 26;
    };
    ImVec4 green(0.0f, 1.0f, 0.0f, 1.0f);
    if (isWeaponInvType(info.inventoryType) && info.damageMax > 0.0f && info.delayMs > 0) {
        float speed = static_cast<float>(info.delayMs) / 1000.0f;
        float dps = ((info.damageMin + info.damageMax) * 0.5f) / speed;
        ImGui::Text("%.0f - %.0f Damage", info.damageMin, info.damageMax);
        ImGui::SameLine(160.0f);
        ImGui::TextDisabled("Speed %.2f", speed);
        ImGui::TextColored(ui::colors::kLightGray, "(%.1f damage per second)", dps);
    }

    if (info.armor > 0) ImGui::Text("%d Armor", info.armor);

    // Elemental resistances (fire resist gear, nature resist gear, etc.)
    {
        const int32_t resVals[6]  = { info.holyRes, info.fireRes, info.natureRes,
                                      info.frostRes, info.shadowRes, info.arcaneRes };
        for (int i = 0; i < 6; ++i)
            if (resVals[i] > 0) ImGui::Text("+%d %s", resVals[i], game::resistanceSchoolName(static_cast<uint32_t>(i)));
    }

    auto appendBonus = [](std::string& out, int32_t val, const char* name) {
        if (val <= 0) return;
        if (!out.empty()) out += "  ";
        out += "+" + std::to_string(val) + " " + name;
    };
    std::string bonusLine;
    appendBonus(bonusLine, info.strength,  "Str");
    appendBonus(bonusLine, info.agility,   "Agi");
    appendBonus(bonusLine, info.stamina,   "Sta");
    appendBonus(bonusLine, info.intellect, "Int");
    appendBonus(bonusLine, info.spirit,    "Spi");
    if (!bonusLine.empty()) ImGui::TextColored(green, "%s", bonusLine.c_str());

    // Extra stats
    for (const auto& es : info.extraStats) {
        const char* statName = game::itemStatName(es.statType);
        char buf[64];
        if (statName)
            std::snprintf(buf, sizeof(buf), "%+d %s", es.statValue, statName);
        else
            std::snprintf(buf, sizeof(buf), "%+d (stat %u)", es.statValue, es.statType);
        ImGui::TextColored(green, "%s", buf);
    }

    if (info.requiredLevel > 1) {
        uint32_t playerLvl = gameHandler ? gameHandler->getPlayerLevel() : 0;
        bool meetsReq = (playerLvl >= info.requiredLevel);
        ImVec4 reqColor = meetsReq ? ImVec4(1.0f, 1.0f, 1.0f, 0.75f) : ui::colors::kPaleRed;
        ImGui::TextColored(reqColor, "Requires Level %u", info.requiredLevel);
    }

    // Required skill (e.g. "Requires Engineering (300)")
    if (info.requiredSkill != 0 && info.requiredSkillRank > 0) {
        // Lazy-load SkillLine.dbc names
        static std::unordered_map<uint32_t, std::string> s_skillNames;
        static bool s_skillNamesLoaded = false;
        if (!s_skillNamesLoaded && assetManager && assetManager->isInitialized()) {
            s_skillNamesLoaded = true;
            auto dbc = assetManager->loadDBC("SkillLine.dbc");
            if (dbc && dbc->isLoaded()) {
                const auto* layout = pipeline::getActiveDBCLayout()
                    ? pipeline::getActiveDBCLayout()->getLayout("SkillLine") : nullptr;
                uint32_t idF   = layout ? (*layout)["ID"]   : 0;
                uint32_t nameF = layout ? (*layout)["Name"] : 2;
                for (uint32_t r = 0; r < dbc->getRecordCount(); ++r) {
                    uint32_t sid = dbc->getUInt32(r, idF);
                    if (!sid) continue;
                    std::string sname = dbc->getString(r, nameF);
                    if (!sname.empty()) s_skillNames[sid] = std::move(sname);
                }
            }
        }
        uint32_t playerSkillVal = 0;
        if (gameHandler) {
            const auto& skills = gameHandler->getPlayerSkills();
            auto skPit = skills.find(info.requiredSkill);
            if (skPit != skills.end()) playerSkillVal = skPit->second.effectiveValue();
        }
        bool meetsSkill = (playerSkillVal == 0 || playerSkillVal >= info.requiredSkillRank);
        ImVec4 skColor = meetsSkill ? ImVec4(1.0f, 1.0f, 1.0f, 0.75f) : ui::colors::kPaleRed;
        auto skIt = s_skillNames.find(info.requiredSkill);
        if (skIt != s_skillNames.end())
            ImGui::TextColored(skColor, "Requires %s (%u)", skIt->second.c_str(), info.requiredSkillRank);
        else
            ImGui::TextColored(skColor, "Requires Skill %u (%u)", info.requiredSkill, info.requiredSkillRank);
    }

    // Required reputation (e.g. "Requires Exalted with Argent Dawn")
    if (info.requiredReputationFaction != 0 && info.requiredReputationRank > 0) {
        static std::unordered_map<uint32_t, std::string> s_factionNames;
        static bool s_factionNamesLoaded = false;
        if (!s_factionNamesLoaded && assetManager && assetManager->isInitialized()) {
            s_factionNamesLoaded = true;
            auto dbc = assetManager->loadDBC("Faction.dbc");
            if (dbc && dbc->isLoaded()) {
                const auto* layout = pipeline::getActiveDBCLayout()
                    ? pipeline::getActiveDBCLayout()->getLayout("Faction") : nullptr;
                uint32_t idF   = layout ? (*layout)["ID"]   : 0;
                uint32_t nameF = layout ? (*layout)["Name"] : 20;
                for (uint32_t r = 0; r < dbc->getRecordCount(); ++r) {
                    uint32_t fid = dbc->getUInt32(r, idF);
                    if (!fid) continue;
                    std::string fname = dbc->getString(r, nameF);
                    if (!fname.empty()) s_factionNames[fid] = std::move(fname);
                }
            }
        }
        const char* rankName = game::reputationRankName(info.requiredReputationRank);
        auto fIt = s_factionNames.find(info.requiredReputationFaction);
        ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 0.75f), "Requires %s with %s",
            rankName,
            fIt != s_factionNames.end() ? fIt->second.c_str() : "Unknown Faction");
    }

    if (info.allowableClass != 0 && gameHandler)
        renderClassRestriction(info.allowableClass, gameHandler->getPlayerClass());
    if (info.allowableRace != 0 && gameHandler)
        renderRaceRestriction(info.allowableRace, gameHandler->getPlayerRace());

    // Spell effects
    for (const auto& sp : info.spells) {
        if (sp.spellId == 0) continue;
        const char* trigger = game::itemSpellTriggerText(sp.spellTrigger);
        if (!trigger) continue;
        if (gameHandler) {
            // Prefer the spell's tooltip text (the actual effect description).
            // Fall back to the spell name if the description is empty.
            const std::string& spDesc = gameHandler->getSpellDescription(sp.spellId);
            std::string spName = spDesc.empty()
                ? gameHandler->getSpellName(sp.spellId)
                : gameHandler->formatSpellDescription(sp.spellId, spDesc);
            if (!spName.empty()) {
                ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + 320.0f);
                ImGui::TextColored(ui::colors::kCyan, "%s: %s", trigger, spName.c_str());
                ImGui::PopTextWrapPos();
            } else {
                ImGui::TextColored(ui::colors::kCyan, "%s: Spell #%u", trigger, sp.spellId);
            }
        }
    }

    // Gem socket slots
    {
        // Get socket gem enchant IDs for this item (filled from item update fields)
        std::array<uint32_t, 3> sockGems{};
        if (itemGuid != 0 && gameHandler)
            sockGems = gameHandler->getItemSocketEnchantIds(itemGuid);

        bool hasSocket = false;
        for (int i = 0; i < 3; ++i) {
            if (info.socketColor[i] == 0) continue;
            if (!hasSocket) { ImGui::Spacing(); hasSocket = true; }
            for (const auto& st : kSocketTypes) {
                if (info.socketColor[i] & st.mask) {
                    if (sockGems[i] != 0) {
                        auto git = s_enchLookup.find(sockGems[i]);
                        if (git != s_enchLookup.end())
                            ImGui::TextColored(st.col, "%s: %s", st.label, git->second.c_str());
                        else
                            ImGui::TextColored(st.col, "%s: (gem %u)", st.label, sockGems[i]);
                    } else {
                        ImGui::TextColored(st.col, "%s", st.label);
                    }
                    break;
                }
            }
        }
        if (hasSocket && info.socketBonus != 0) {
            auto enchIt = s_enchLookup.find(info.socketBonus);
            if (enchIt != s_enchLookup.end())
                ImGui::TextColored(ui::colors::kSocketGreen, "Socket Bonus: %s", enchIt->second.c_str());
            else
                ImGui::TextColored(ui::colors::kSocketGreen, "Socket Bonus: (id %u)", info.socketBonus);
        }
    }

    // Weapon/armor enchant display for equipped items
    if (itemGuid != 0 && gameHandler) {
        auto [permId, tempId] = gameHandler->getItemEnchantIds(itemGuid);
        if (permId != 0) {
            auto it2 = s_enchLookup.find(permId);
            const char* ename = (it2 != s_enchLookup.end()) ? it2->second.c_str() : nullptr;
            if (ename) ImGui::TextColored(ui::colors::kCyan, "Enchanted: %s", ename);
        }
        if (tempId != 0) {
            auto it2 = s_enchLookup.find(tempId);
            const char* ename = (it2 != s_enchLookup.end()) ? it2->second.c_str() : nullptr;
            if (ename) ImGui::TextColored(ImVec4(0.8f, 1.0f, 0.4f, 1.0f), "%s (temporary)", ename);
        }
    }

    // Item set membership
    if (info.itemSetId != 0) {
        // Lazy-load full ItemSet.dbc data (name + item IDs + bonus spells/thresholds)
        struct SetEntry {
            std::string name;
            std::array<uint32_t, 10> itemIds{};
            std::array<uint32_t, 10> spellIds{};
            std::array<uint32_t, 10> thresholds{};
        };
        static std::unordered_map<uint32_t, SetEntry> s_setData;
        static bool s_setDataLoaded = false;
        if (!s_setDataLoaded && assetManager && assetManager->isInitialized()) {
            s_setDataLoaded = true;
            auto dbc = assetManager->loadDBC("ItemSet.dbc");
            if (dbc && dbc->isLoaded()) {
                const auto* layout = pipeline::getActiveDBCLayout()
                    ? pipeline::getActiveDBCLayout()->getLayout("ItemSet") : nullptr;
                auto lf = [&](const char* k, uint32_t def) -> uint32_t {
                    return layout ? (*layout)[k] : def;
                };
                uint32_t idF = lf("ID", 0), nameF = lf("Name", 1);
                const auto& itemKeys = ui::kItemSetItemKeys;
                const auto& spellKeys = ui::kItemSetSpellKeys;
                const auto& thrKeys = ui::kItemSetThresholdKeys;
                uint32_t itemFallback[10], spellFallback[10], thrFallback[10];
                for (int i = 0; i < 10; ++i) {
                    itemFallback[i]  = 18 + i;
                    spellFallback[i] = 28 + i;
                    thrFallback[i]   = 38 + i;
                }
                for (uint32_t r = 0; r < dbc->getRecordCount(); ++r) {
                    uint32_t id = dbc->getUInt32(r, idF);
                    if (!id) continue;
                    SetEntry e;
                    e.name = dbc->getString(r, nameF);
                    for (int i = 0; i < 10; ++i) {
                        e.itemIds[i]    = dbc->getUInt32(r, layout ? (*layout)[itemKeys[i]]  : itemFallback[i]);
                        e.spellIds[i]   = dbc->getUInt32(r, layout ? (*layout)[spellKeys[i]] : spellFallback[i]);
                        e.thresholds[i] = dbc->getUInt32(r, layout ? (*layout)[thrKeys[i]]   : thrFallback[i]);
                    }
                    s_setData[id] = std::move(e);
                }
            }
        }

        auto setIt = s_setData.find(info.itemSetId);
        ImGui::Spacing();
        if (setIt != s_setData.end()) {
            const SetEntry& se = setIt->second;
            // Count equipped pieces
            int equipped = 0, total = 0;
            for (int i = 0; i < 10; ++i) {
                if (se.itemIds[i] == 0) continue;
                ++total;
                if (inventory) {
                    for (int s = 0; s < game::Inventory::NUM_EQUIP_SLOTS; s++) {
                        const auto& eSlot = inventory->getEquipSlot(static_cast<game::EquipSlot>(s));
                        if (!eSlot.empty() && eSlot.item.itemId == se.itemIds[i]) { ++equipped; break; }
                    }
                }
            }
            if (total > 0) {
                ImGui::TextColored(ui::colors::kTooltipGold,
                    "%s (%d/%d)", se.name.empty() ? "Set" : se.name.c_str(), equipped, total);
            } else {
                if (!se.name.empty())
                    ImGui::TextColored(ui::colors::kTooltipGold, "%s", se.name.c_str());
            }
            // Show set bonuses: gray if not reached, green if active
            if (gameHandler) {
                for (int i = 0; i < 10; ++i) {
                    if (se.spellIds[i] == 0 || se.thresholds[i] == 0) continue;
                    const std::string& bname = gameHandler->getSpellName(se.spellIds[i]);
                    bool active = (equipped >= static_cast<int>(se.thresholds[i]));
                    ImVec4 col = active ? ui::colors::kActiveGreen
                                       : ui::colors::kInactiveGray;
                    if (!bname.empty())
                        ImGui::TextColored(col, "(%u) %s", se.thresholds[i], bname.c_str());
                    else
                        ImGui::TextColored(col, "(%u) Set Bonus", se.thresholds[i]);
                }
            }
        } else {
            ImGui::TextColored(ui::colors::kTooltipGold, "Set (id %u)", info.itemSetId);
        }
    }

    if (info.startQuestId != 0) {
        ImGui::TextColored(ui::colors::kTooltipGold, "Begins a Quest");
    }
    if (!info.description.empty()) {
        ImGui::TextColored(ImVec4(1.0f, 0.9f, 0.5f, 0.9f), "\"%s\"", info.description.c_str());
    }

    if (info.sellPrice > 0) {
        ImGui::TextDisabled("Sell:"); ImGui::SameLine(0, 4);
        renderCoinsFromCopper(info.sellPrice);
    }

    // Shift-hover: compare with currently equipped item
    if (inventory && ImGui::GetIO().KeyShift && info.inventoryType > 0) {
        if (const auto equipped = findComparableEquipped(
                *inventory, static_cast<uint8_t>(info.inventoryType))) {
            const game::ItemSlot* eq = equipped.slot;
            ImGui::Separator();
            ImGui::TextDisabled("Equipped:");
            VkDescriptorSet eqIcon = itemIcon(eq->item.displayInfoId, assetManager);
            if (eqIcon) { ImGui::Image((ImTextureID)(uintptr_t)eqIcon, ImVec2(18.0f, 18.0f)); ImGui::SameLine(); }
            ImGui::TextColored(ui::getQualityColor(eq->item.quality), "%s", eq->item.name.c_str());
            renderEquippedEnhancements(gameHandler, equipped, s_enchLookup);

            auto showDiff = [](const char* label, float nv, float ev) {
                if (nv == 0.0f && ev == 0.0f) return;
                float diff = nv - ev;
                char buf[96];
                if (diff > 0.0f) { std::snprintf(buf, sizeof(buf), "%s: %.0f (▲%.0f)", label, ev, diff);  ImGui::TextColored(ImVec4(0.0f,1.0f,0.0f,1.0f), "%s", buf); }
                else if (diff < 0.0f) { std::snprintf(buf, sizeof(buf), "%s: %.0f (▼%.0f)", label, ev, -diff); ImGui::TextColored(ui::colors::kRed, "%s", buf); }
                else { std::snprintf(buf, sizeof(buf), "%s: %.0f (=)", label, ev); ImGui::TextColored(ui::colors::kLightGray, "%s", buf); }
            };

            float ilvlDiff = static_cast<float>(info.itemLevel) - static_cast<float>(eq->item.itemLevel);
            if (info.itemLevel > 0 || eq->item.itemLevel > 0) {
                char ilvlBuf[64];
                if (ilvlDiff > 0)      std::snprintf(ilvlBuf, sizeof(ilvlBuf), "Item Level: %u (▲%.0f)", eq->item.itemLevel, ilvlDiff);
                else if (ilvlDiff < 0) std::snprintf(ilvlBuf, sizeof(ilvlBuf), "Item Level: %u (▼%.0f)", eq->item.itemLevel, -ilvlDiff);
                else                   std::snprintf(ilvlBuf, sizeof(ilvlBuf), "Item Level: %u (=)", eq->item.itemLevel);
                ImVec4 ic = ilvlDiff > 0 ? ImVec4(0,1,0,1) : ilvlDiff < 0 ? ImVec4(1,0.3f,0.3f,1) : ImVec4(0.7f,0.7f,0.7f,1);
                ImGui::TextColored(ic, "%s", ilvlBuf);
            }

            // DPS comparison for weapons
            if (isWeaponInvType(info.inventoryType) && isWeaponInvType(eq->item.inventoryType)) {
                float newDps = 0.0f, eqDps = 0.0f;
                if (info.damageMax > 0.0f && info.delayMs > 0)
                    newDps = ((info.damageMin + info.damageMax) * 0.5f) / (info.delayMs / 1000.0f);
                if (eq->item.damageMax > 0.0f && eq->item.delayMs > 0)
                    eqDps = ((eq->item.damageMin + eq->item.damageMax) * 0.5f) / (eq->item.delayMs / 1000.0f);
                showDiff("DPS", newDps, eqDps);
            }

            showDiff("Armor", static_cast<float>(info.armor),     static_cast<float>(eq->item.armor));
            showDiff("Str",   static_cast<float>(info.strength),  static_cast<float>(eq->item.strength));
            showDiff("Agi",   static_cast<float>(info.agility),   static_cast<float>(eq->item.agility));
            showDiff("Sta",   static_cast<float>(info.stamina),   static_cast<float>(eq->item.stamina));
            showDiff("Int",   static_cast<float>(info.intellect), static_cast<float>(eq->item.intellect));
            showDiff("Spi",   static_cast<float>(info.spirit),    static_cast<float>(eq->item.spirit));

            // Extra stats diff - union of stat types from both items
            auto findExtraStat = [](const auto& it, uint32_t type) -> int32_t {
                for (const auto& es : it.extraStats)
                    if (es.statType == type) return es.statValue;
                return 0;
            };
            std::vector<uint32_t> allTypes;
            allTypes.reserve(info.extraStats.size());
            for (const auto& es : info.extraStats) allTypes.push_back(es.statType);
            for (const auto& es : eq->item.extraStats) {
                bool found = false;
                for (uint32_t t : allTypes) if (t == es.statType) { found = true; break; }
                if (!found) allTypes.push_back(es.statType);
            }
            for (uint32_t t : allTypes) {
                int32_t nv = findExtraStat(info, t);
                int32_t ev = findExtraStat(eq->item, t);
                const char* lbl = comparisonStatLabel(t);
                if (!lbl) continue;
                showDiff(lbl, static_cast<float>(nv), static_cast<float>(ev));
            }
        }
    } else if (info.inventoryType > 0) {
        ImGui::TextDisabled("Hold Shift to compare");
    }

    ImGui::EndTooltip();
}

}  // namespace ui
}  // namespace wowee
