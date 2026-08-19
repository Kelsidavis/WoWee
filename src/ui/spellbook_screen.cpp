#include "ui/spellbook_screen.hpp"
#include "ui/ui_texture_load.hpp"
#include "ui/ui_upload_budget.hpp"
#include "ui/ui_colors.hpp"
#include "ui/keybinding_manager.hpp"
#include "core/input.hpp"
#include "core/application.hpp"
#include "rendering/vk_context.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_loader.hpp"
#include "pipeline/dbc_layout.hpp"
#include "core/logger.hpp"
#include "pipeline/spell_icon_paths.hpp"
#include <algorithm>
#include <map>
#include <cctype>

namespace wowee { namespace ui {

// Case-insensitive substring match
static bool containsCI(const std::string& haystack, const char* needle) {
    if (!needle || !needle[0]) return true;
    size_t needleLen = strlen(needle);
    if (needleLen > haystack.size()) return false;
    for (size_t i = 0; i <= haystack.size() - needleLen; i++) {
        bool match = true;
        for (size_t j = 0; j < needleLen; j++) {
            if (std::tolower(static_cast<unsigned char>(haystack[i + j])) !=
                std::tolower(static_cast<unsigned char>(needle[j]))) {
                match = false;
                break;
            }
        }
        if (match) return true;
    }
    return false;
}

void SpellbookScreen::loadSpellDBC(pipeline::AssetManager* assetManager) {
    if (dbcLoadAttempted) return;
    dbcLoadAttempted = true;

    if (!assetManager || !assetManager->isInitialized()) return;

    auto dbc = assetManager->loadDBC("Spell.dbc");
    if (!dbc || !dbc->isLoaded()) {
        LOG_WARNING("Spellbook: Could not load Spell.dbc");
        return;
    }

    uint32_t fieldCount = dbc->getFieldCount();
    // Classic 1.12 Spell.dbc has 148 fields (Tooltip at index 147), TBC has ~220+ (SchoolMask at 215), WotLK has 234.
    // Require at least 148 fields so all expansions can load spell names/icons via the DBC layout.
    if (fieldCount < 148) {
        LOG_WARNING("Spellbook: Spell.dbc has ", fieldCount, " fields, too few to load");
        return;
    }

    const auto* spellL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("Spell") : nullptr;

    // Load SpellCastTimes.dbc: field 0=ID, field 1=Base(ms), field 2=PerLevel, field 3=Minimum
    std::unordered_map<uint32_t, uint32_t> castTimeMap;  // index → base ms
    auto castTimeDbc = assetManager->loadDBC("SpellCastTimes.dbc");
    if (castTimeDbc && castTimeDbc->isLoaded()) {
        for (uint32_t i = 0; i < castTimeDbc->getRecordCount(); ++i) {
            uint32_t id   = castTimeDbc->getUInt32(i, 0);
            int32_t  base = static_cast<int32_t>(castTimeDbc->getUInt32(i, 1));
            if (id > 0 && base > 0)
                castTimeMap[id] = static_cast<uint32_t>(base);
        }
    }

    // Load SpellRange.dbc.  Field layout differs by expansion:
    //   Classic 1.12:  0=ID, 1=MinRange, 2=MaxRange, 3=Flags, 4+=strings
    //   TBC / WotLK:   0=ID, 1=MinRangeFriendly, 2=MinRangeHostile,
    //                  3=MaxRangeFriendly, 4=MaxRangeHostile, 5=Flags, 6+=strings
    // The correct field is declared in each expansion's dbc_layouts.json.
    uint32_t spellRangeMaxField = 4;  // WotLK / TBC default: MaxRangeHostile
    const auto* spellRangeL = pipeline::getActiveDBCLayout()
                              ? pipeline::getActiveDBCLayout()->getLayout("SpellRange")
                              : nullptr;
    if (spellRangeL) {
        try { spellRangeMaxField = (*spellRangeL)["MaxRange"]; } catch (...) {}
    }
    std::unordered_map<uint32_t, float> rangeMap;  // index → max yards
    auto rangeDbc = assetManager->loadDBC("SpellRange.dbc");
    if (rangeDbc && rangeDbc->isLoaded()) {
        uint32_t rangeFieldCount = rangeDbc->getFieldCount();
        if (rangeFieldCount > spellRangeMaxField) {
            for (uint32_t i = 0; i < rangeDbc->getRecordCount(); ++i) {
                uint32_t id = rangeDbc->getUInt32(i, 0);
                float maxRange = rangeDbc->getFloat(i, spellRangeMaxField);
                if (id > 0 && maxRange > 0.0f)
                    rangeMap[id] = maxRange;
            }
        }
    }

    // schoolField / isSchoolEnum are declared before the lambda so the WotLK fallback path
    // can override them before the second tryLoad call.
    uint32_t schoolField_  = UINT32_MAX;
    bool     isSchoolEnum_ = false;

    auto tryLoad = [&](uint32_t idField, uint32_t attrField, uint32_t iconField,
                       uint32_t nameField, uint32_t rankField, uint32_t tooltipField,
                       uint32_t descriptionField,
                       uint32_t powerTypeField, uint32_t manaCostField,
                       uint32_t castTimeIndexField, uint32_t rangeIndexField,
                       uint32_t casterAuraStateField, uint32_t casterAuraStateNotField,
                       const char* label) {
        spellData.clear();
        uint32_t count = dbc->getRecordCount();
        const uint32_t fc = dbc->getFieldCount();
        for (uint32_t i = 0; i < count; ++i) {
            uint32_t spellId = dbc->getUInt32(i, idField);
            if (spellId == 0) continue;

            SpellInfo info;
            info.spellId = spellId;
            info.attributes = dbc->getUInt32(i, attrField);
            info.iconId = dbc->getUInt32(i, iconField);
            info.name = dbc->getString(i, nameField);
            if (rankField < fc)    info.rank = dbc->getString(i, rankField);
            // Prefer the fuller Description (mentions e.g. the "well fed" food buff) and
            // fall back to the short Tooltip; $-tokens are resolved at render time.
            if (descriptionField < fc) info.description = dbc->getString(i, descriptionField);
            if (info.description.empty() && tooltipField < fc)
                info.description = dbc->getString(i, tooltipField);
            // Optional fields: only read if field index is valid for this DBC version
            if (powerTypeField < fc)   info.powerType = dbc->getUInt32(i, powerTypeField);
            if (manaCostField  < fc)   info.manaCost  = dbc->getUInt32(i, manaCostField);
            if (casterAuraStateField < fc)
                info.casterAuraState = dbc->getUInt32(i, casterAuraStateField);
            if (casterAuraStateNotField < fc)
                info.casterAuraStateNot = dbc->getUInt32(i, casterAuraStateNotField);
            if (castTimeIndexField < fc) {
                uint32_t ctIdx = dbc->getUInt32(i, castTimeIndexField);
                if (ctIdx > 0) {
                    auto ctIt = castTimeMap.find(ctIdx);
                    if (ctIt != castTimeMap.end()) info.castTimeMs = ctIt->second;
                }
            }
            if (rangeIndexField < fc) {
                uint32_t rangeIdx = dbc->getUInt32(i, rangeIndexField);
                if (rangeIdx > 0) {
                    auto rangeIt = rangeMap.find(rangeIdx);
                    if (rangeIt != rangeMap.end()) info.rangeIndex = static_cast<uint32_t>(rangeIt->second);
                }
            }
            if (schoolField_ < fc) {
                uint32_t raw = dbc->getUInt32(i, schoolField_);
                // Classic/Turtle use a 0-6 school enum; TBC/WotLK use a bitmask.
                // enum→mask: schoolEnum N maps to bit (1u << N), e.g. 0→1 (physical), 4→16 (frost).
                info.schoolMask = isSchoolEnum_ ? (raw <= 6 ? (1u << raw) : 0u) : raw;
            }

            if (!info.name.empty()) {
                spellData[spellId] = std::move(info);
            }
        }
        LOG_INFO("Spellbook: Loaded ", spellData.size(), " spells from Spell.dbc (", label, ")");
    };

    if (spellL) {
        // Default to UINT32_MAX for optional fields; tryLoad will skip them if >= fieldCount.
        // Avoids reading wrong data from expansion DBCs that lack these fields (e.g. Classic/TBC).
        uint32_t tooltipField      = UINT32_MAX;
        uint32_t descriptionField  = UINT32_MAX;
        uint32_t powerTypeField    = UINT32_MAX;
        uint32_t manaCostField     = UINT32_MAX;
        uint32_t castTimeIdxField  = UINT32_MAX;
        uint32_t rangeIdxField     = UINT32_MAX;
        uint32_t casterAuraStateField = UINT32_MAX;
        uint32_t casterAuraStateNotField = UINT32_MAX;
        try { tooltipField     = (*spellL)["Tooltip"]; } catch (...) {}
        try { descriptionField = (*spellL)["Description"]; } catch (...) {}
        try { powerTypeField   = (*spellL)["PowerType"]; } catch (...) {}
        try { manaCostField    = (*spellL)["ManaCost"]; } catch (...) {}
        // From the file's own shape rather than the expansion's name for it:
        // Classic and Turtle named this 15, which is RequiresSpellFocus.
        castTimeIdxField = pipeline::detectSpellTimingFields(dbc.get(), spellL).castingTimeIndex;
        try { rangeIdxField    = (*spellL)["RangeIndex"]; } catch (...) {}
        try { casterAuraStateField = (*spellL)["CasterAuraState"]; } catch (...) {}
        try { casterAuraStateNotField = (*spellL)["CasterAuraStateNot"]; } catch (...) {}
        // Try SchoolMask (TBC/WotLK bitmask) then SchoolEnum (Classic/Turtle 0-6 value)
        schoolField_  = UINT32_MAX;
        isSchoolEnum_ = false;
        try { schoolField_ = (*spellL)["SchoolMask"]; } catch (...) {}
        if (schoolField_ == UINT32_MAX) {
            try { schoolField_ = (*spellL)["SchoolEnum"]; isSchoolEnum_ = true; } catch (...) {}
        }
        tryLoad((*spellL)["ID"], (*spellL)["Attributes"], (*spellL)["IconID"],
                (*spellL)["Name"], (*spellL)["Rank"], tooltipField, descriptionField,
                powerTypeField, manaCostField, castTimeIdxField, rangeIdxField,
                casterAuraStateField, casterAuraStateNotField,
                "expansion layout");
    }

    // If dbc_layouts.json was missing or its field names didn't match, retry with
    // hard-coded WotLK field indices as a safety net. fieldCount >= 200 distinguishes
    // WotLK (234 fields) from Classic (148) to avoid misreading shorter DBCs.
    if (spellData.empty() && fieldCount >= 200) {
        LOG_INFO("Spellbook: Retrying with WotLK field indices (DBC has ", fieldCount, " fields)");
        schoolField_  = 225;
        isSchoolEnum_ = false;
        tryLoad(0, 4, 133, 136, 153, 187, 170, 41, 42, 28, 46, 20, 22, "WotLK fallback");
    }

    dbcLoaded = !spellData.empty();
}

bool SpellbookScreen::renderSpellInfoTooltip(uint32_t spellId, game::GameHandler& gameHandler,
                                              pipeline::AssetManager* assetManager) {
    if (!dbcLoadAttempted) loadSpellDBC(assetManager);
    const SpellInfo* info = getSpellInfo(spellId);
    if (!info) return false;
    renderSpellTooltip(info, gameHandler, /*showUsageHints=*/false);
    return true;
}

uint32_t SpellbookScreen::getSpellMaxRange(uint32_t spellId, pipeline::AssetManager* assetManager) {
    if (!dbcLoadAttempted) {
        loadSpellDBC(assetManager);
    }
    auto it = spellData.find(spellId);
    if (it != spellData.end()) return it->second.rangeIndex;
    return 0;
}

void SpellbookScreen::getSpellPowerInfo(uint32_t spellId, pipeline::AssetManager* assetManager,
                                        uint32_t& outCost, uint32_t& outPowerType) {
    outCost = 0;
    outPowerType = 0;
    if (!dbcLoadAttempted) {
        loadSpellDBC(assetManager);
    }
    auto it = spellData.find(spellId);
    if (it != spellData.end()) {
        outCost = it->second.manaCost;
        outPowerType = it->second.powerType;
    }
}

void SpellbookScreen::getSpellAuraStateInfo(uint32_t spellId, pipeline::AssetManager* assetManager,
                                             uint32_t& outRequired, uint32_t& outForbidden) {
    outRequired = 0;
    outForbidden = 0;
    if (!dbcLoadAttempted) {
        loadSpellDBC(assetManager);
    }
    auto it = spellData.find(spellId);
    if (it != spellData.end()) {
        outRequired = it->second.casterAuraState;
        outForbidden = it->second.casterAuraStateNot;
    }
}

void SpellbookScreen::loadSpellIconDBC(pipeline::AssetManager* assetManager) {
    if (iconDbLoaded) return;

    if (!assetManager || !assetManager->isInitialized()) return;
    // Not an attempt: the assets are not up yet, and a caller can reach
    // this before they are. Latching here recorded "read" for a file
    // never opened, and disabled it for the rest of the session.
    iconDbLoaded = true;

    pipeline::loadSpellIconPaths(assetManager, spellIconPaths);
}

void SpellbookScreen::loadSkillLineDBCs(pipeline::AssetManager* assetManager) {
    if (skillLineDbLoaded) return;

    if (!assetManager || !assetManager->isInitialized()) return;
    // Not an attempt: the assets are not up yet, and a caller can reach
    // this before they are. Latching here recorded "read" for a file
    // never opened, and disabled it for the rest of the session.
    skillLineDbLoaded = true;

    auto skillLineDbc = assetManager->loadDBC("SkillLine.dbc");
    const auto* slL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("SkillLine") : nullptr;
    if (skillLineDbc && skillLineDbc->isLoaded()) {
        for (uint32_t i = 0; i < skillLineDbc->getRecordCount(); i++) {
            uint32_t id = skillLineDbc->getUInt32(i, slL ? (*slL)["ID"] : 0);
            uint32_t category = skillLineDbc->getUInt32(i, slL ? (*slL)["Category"] : 1);
            std::string name = skillLineDbc->getString(i, slL ? (*slL)["Name"] : 3);
            if (id > 0) {
                if (!name.empty()) {
                    skillLineNames[id] = name;
                }
                skillLineCategories[id] = category;
            }
        }
        LOG_INFO("Spellbook: Loaded ", skillLineNames.size(), " skill line names, ",
                 skillLineCategories.size(), " categories from SkillLine.dbc");
    } else {
        LOG_WARNING("Spellbook: Could not load SkillLine.dbc");
    }

    auto slaDbc = assetManager->loadDBC("SkillLineAbility.dbc");
    const auto* slaL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("SkillLineAbility") : nullptr;
    if (slaDbc && slaDbc->isLoaded()) {
        for (uint32_t i = 0; i < slaDbc->getRecordCount(); i++) {
            uint32_t skillLineId = slaDbc->getUInt32(i, slaL ? (*slaL)["SkillLineID"] : 1);
            uint32_t spellId = slaDbc->getUInt32(i, slaL ? (*slaL)["SpellID"] : 2);
            if (spellId > 0 && skillLineId > 0) {
                spellToSkillLine.emplace(spellId, skillLineId);
            }
        }
        LOG_INFO("Spellbook: Loaded ", spellToSkillLine.size(), " spell-to-skillline mappings from SkillLineAbility.dbc");
    } else {
        LOG_WARNING("Spellbook: Could not load SkillLineAbility.dbc");
    }
}

void SpellbookScreen::categorizeSpells(const std::unordered_set<uint32_t>& knownSpells) {
    spellTabs.clear();

    // SkillLine.dbc category IDs
    static constexpr uint32_t CAT_CLASS       = 7;   // Class abilities (spec trees)
    static constexpr uint32_t CAT_PROFESSION  = 11;  // Primary professions
    static constexpr uint32_t CAT_SECONDARY   = 9;   // Secondary skills (Cooking, First Aid, Fishing, Riding, Companions)

    // Special skill line IDs that get their own tabs
    static constexpr uint32_t SKILLLINE_MOUNTS     = 777;  // Mount summon spells (category 7)
    static constexpr uint32_t SKILLLINE_RIDING     = 762;  // Riding skill ranks (category 9)
    static constexpr uint32_t SKILLLINE_COMPANIONS = 778;  // Vanity/companion pets (category 7)

    // Buckets
    std::map<uint32_t, std::vector<const SpellInfo*>> specSpells;  // class spec trees
    std::map<uint32_t, std::vector<const SpellInfo*>> profSpells;  // professions + secondary
    std::vector<const SpellInfo*> mountSpells;
    std::vector<const SpellInfo*> companionSpells;
    std::vector<const SpellInfo*> generalSpells;

    for (uint32_t spellId : knownSpells) {
        auto it = spellData.find(spellId);
        if (it == spellData.end()) continue;

        const SpellInfo* info = &it->second;

        // Check all skill lines this spell belongs to, prefer class (cat 7) > profession > secondary > special
        auto range = spellToSkillLine.equal_range(spellId);
        bool categorized = false;

        uint32_t bestSkillLine = 0;
        int bestPriority = -1; // 4=class, 3=profession, 2=secondary, 1=mount/companion

        for (auto slIt = range.first; slIt != range.second; ++slIt) {
            uint32_t skillLineId = slIt->second;

            if (skillLineId == SKILLLINE_MOUNTS || skillLineId == SKILLLINE_RIDING) {
                if (bestPriority < 1) { bestPriority = 1; bestSkillLine = SKILLLINE_MOUNTS; }
                continue;
            }
            if (skillLineId == SKILLLINE_COMPANIONS) {
                if (bestPriority < 1) { bestPriority = 1; bestSkillLine = skillLineId; }
                continue;
            }

            auto catIt = skillLineCategories.find(skillLineId);
            if (catIt != skillLineCategories.end()) {
                uint32_t cat = catIt->second;
                if (cat == CAT_CLASS && bestPriority < 4) {
                    bestPriority = 4; bestSkillLine = skillLineId;
                } else if (cat == CAT_PROFESSION && bestPriority < 3) {
                    bestPriority = 3; bestSkillLine = skillLineId;
                } else if (cat == CAT_SECONDARY && bestPriority < 2) {
                    bestPriority = 2; bestSkillLine = skillLineId;
                }
            }
        }

        if (bestSkillLine > 0) {
            if (bestSkillLine == SKILLLINE_MOUNTS) {
                mountSpells.push_back(info);
                categorized = true;
            } else if (bestSkillLine == SKILLLINE_COMPANIONS) {
                companionSpells.push_back(info);
                categorized = true;
            } else {
                auto catIt = skillLineCategories.find(bestSkillLine);
                if (catIt != skillLineCategories.end()) {
                    uint32_t cat = catIt->second;
                    if (cat == CAT_CLASS) {
                        specSpells[bestSkillLine].push_back(info);
                        categorized = true;
                    } else if (cat == CAT_PROFESSION || cat == CAT_SECONDARY) {
                        profSpells[bestSkillLine].push_back(info);
                        categorized = true;
                    }
                }
            }
        }

        if (!categorized) {
            generalSpells.push_back(info);
        }
    }

    LOG_INFO("Spellbook categorize: ", specSpells.size(), " spec groups, ",
             generalSpells.size(), " general, ", profSpells.size(), " prof groups, ",
             mountSpells.size(), " mounts, ", companionSpells.size(), " companions");
    for (const auto& [slId, spells] : specSpells) {
        auto nameIt = skillLineNames.find(slId);
        LOG_INFO("  Spec tab: skillLine=", slId, " name='",
                 (nameIt != skillLineNames.end() ? nameIt->second : "?"), "' spells=", spells.size());
    }

    auto byName = [](const SpellInfo* a, const SpellInfo* b) { return a->name < b->name; };

    // Helper: add sorted skill-line-grouped tabs
    auto addGroupedTabs = [&](std::map<uint32_t, std::vector<const SpellInfo*>>& groups,
                              const char* fallbackName) {
        std::vector<std::pair<std::string, std::vector<const SpellInfo*>>> named;
        for (auto& [skillLineId, spells] : groups) {
            auto nameIt = skillLineNames.find(skillLineId);
            std::string tabName = (nameIt != skillLineNames.end()) ? nameIt->second : fallbackName;
            std::sort(spells.begin(), spells.end(), byName);
            named.emplace_back(std::move(tabName), std::move(spells));
        }
        std::sort(named.begin(), named.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });
        for (auto& [name, spells] : named) {
            spellTabs.push_back({.name = std::move(name), .spells = std::move(spells)});
        }
    };

    // 1. Class spec tabs
    addGroupedTabs(specSpells, "Spec");

    // 2. General tab
    if (!generalSpells.empty()) {
        std::sort(generalSpells.begin(), generalSpells.end(), byName);
        spellTabs.push_back({.name = "General", .spells = std::move(generalSpells)});
    }

    // 3. Professions tabs
    addGroupedTabs(profSpells, "Profession");

    // 4. Mounts tab
    if (!mountSpells.empty()) {
        std::sort(mountSpells.begin(), mountSpells.end(), byName);
        spellTabs.push_back({.name = "Mounts", .spells = std::move(mountSpells)});
    }

    // 5. Companions tab
    if (!companionSpells.empty()) {
        std::sort(companionSpells.begin(), companionSpells.end(), byName);
        spellTabs.push_back({.name = "Companions", .spells = std::move(companionSpells)});
    }

    lastKnownSpellCount = knownSpells.size();
    categorizedWithSkillLines = !spellToSkillLine.empty();
}

VkDescriptorSet SpellbookScreen::getSpellIcon(uint32_t iconId, pipeline::AssetManager* assetManager) {
    return ui::cachedIconTexture(iconId, assetManager,
                                 core::Application::getInstance().getWindow(),
                                 spellIconPaths, spellIconCache);
}

const SpellInfo* SpellbookScreen::getSpellInfo(uint32_t spellId) const {
    auto it = spellData.find(spellId);
    return (it != spellData.end()) ? &it->second : nullptr;
}

void SpellbookScreen::renderSpellTooltip(const SpellInfo* info, game::GameHandler& gameHandler, bool showUsageHints) {
    ImGui::BeginTooltip();
    ImGui::PushTextWrapPos(320.0f);

    // Spell name in yellow
    ImGui::TextColored(ui::colors::kYellow, "%s", info->name.c_str());

    // Rank in gray
    if (!info->rank.empty()) {
        ImGui::SameLine();
        ImGui::TextColored(ui::colors::kGray, "(%s)", info->rank.c_str());
    }

    // Passive indicator
    if (info->isPassive()) {
        ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "Passive");
    }

    // Spell school - only show for non-physical schools (physical is the default/implicit)
    if (info->schoolMask != 0 && info->schoolMask != 1 /*physical*/) {
        struct SchoolEntry { uint32_t mask; const char* name; ImVec4 color; };
        static constexpr SchoolEntry kSchools[] = {
            { .mask = 2,  .name = "Holy",    .color = { 1.0f, 1.0f, 0.6f, 1.0f } },
            { .mask = 4,  .name = "Fire",    .color = { 1.0f, 0.5f, 0.1f, 1.0f } },
            { .mask = 8,  .name = "Nature",  .color = { 0.4f, 0.9f, 0.3f, 1.0f } },
            { .mask = 16, .name = "Frost",   .color = { 0.5f, 0.8f, 1.0f, 1.0f } },
            { .mask = 32, .name = "Shadow",  .color = { 0.7f, 0.4f, 1.0f, 1.0f } },
            { .mask = 64, .name = "Arcane",  .color = { 0.9f, 0.5f, 1.0f, 1.0f } },
        };
        bool first = true;
        for (const auto& s : kSchools) {
            if (info->schoolMask & s.mask) {
                if (!first) ImGui::SameLine(0, 0);
                if (first) {
                    ImGui::TextColored(s.color, "%s", s.name);
                    first = false;
                } else {
                    ImGui::SameLine(0, 2);
                    ImGui::TextColored(s.color, "/%s", s.name);
                }
            }
        }
    }

    // Resource cost + cast time on same row (WoW style)
    if (!info->isPassive()) {
        // Left: resource cost (with talent flat/pct modifier applied)
        char costBuf[64] = "";
        if (info->manaCost > 0) {
            // Was its own table with Focus at 4, which is Happiness - a spell
            // costing Focus said "Mana" and one costing Happiness said "Focus".
            const char* powerName = game::powerTypeName(info->powerType);
            if (!powerName) powerName = "Mana";
            // Apply SMSG_SET_FLAT/PCT_SPELL_MODIFIER Cost modifier (SpellModOp::Cost = 14)
            int32_t flatCost = gameHandler.getSpellFlatMod(game::GameHandler::SpellModOp::Cost);
            int32_t pctCost  = gameHandler.getSpellPctMod(game::GameHandler::SpellModOp::Cost);
            uint32_t displayCost = static_cast<uint32_t>(
                game::GameHandler::applySpellMod(static_cast<int32_t>(info->manaCost), flatCost, pctCost));
            std::snprintf(costBuf, sizeof(costBuf), "%u %s", displayCost, powerName);
        }

        // Right: cast time (with talent CastingTime modifier applied)
        char castBuf[32] = "";
        if (info->castTimeMs == 0) {
            std::snprintf(castBuf, sizeof(castBuf), "Instant cast");
        } else {
            // Apply SpellModOp::CastingTime (10) modifiers
            int32_t flatCT = gameHandler.getSpellFlatMod(game::GameHandler::SpellModOp::CastingTime);
            int32_t pctCT  = gameHandler.getSpellPctMod(game::GameHandler::SpellModOp::CastingTime);
            int32_t modCT  = game::GameHandler::applySpellMod(
                static_cast<int32_t>(info->castTimeMs), flatCT, pctCT);
            float secs = static_cast<float>(modCT) / 1000.0f;
            std::snprintf(castBuf, sizeof(castBuf), "%.1f sec cast", secs > 0.0f ? secs : 0.0f);
        }

        if (costBuf[0] || castBuf[0]) {
            float wrapW = 320.0f;
            if (costBuf[0] && castBuf[0]) {
                float castW = ImGui::CalcTextSize(castBuf).x;
                ImGui::Text("%s", costBuf);
                ImGui::SameLine(wrapW - castW);
                ImGui::Text("%s", castBuf);
            } else if (castBuf[0]) {
                ImGui::Text("%s", castBuf);
            } else {
                ImGui::Text("%s", costBuf);
            }
        }

        // Range
        if (info->rangeIndex > 0) {
            char rangeBuf[32];
            if (info->rangeIndex <= 5)
                std::snprintf(rangeBuf, sizeof(rangeBuf), "Melee range");
            else
                std::snprintf(rangeBuf, sizeof(rangeBuf), "%u yd range", info->rangeIndex);
            ImGui::Text("%s", rangeBuf);
        }
    }

    // Cooldown if active
    float cd = gameHandler.getSpellCooldown(info->spellId);
    if (cd > 0.0f) {
        ImGui::TextColored(ui::colors::kRed, "Cooldown: %.1fs", cd);
    }

    // Description - resolve WoW $-tokens (e.g. "increased by $s1") to concrete values.
    if (!info->description.empty()) {
        ImGui::Spacing();
        std::string desc = gameHandler.formatSpellDescription(info->spellId, info->description);
        ImGui::TextWrapped("%s", desc.c_str());
    }

    // Usage hints - only shown when browsing the spellbook, not on action bar hover
    if (!info->isPassive() && showUsageHints) {
        ImGui::Spacing();
        ImGui::TextColored(ui::colors::kBrightGreen, "Drag to action bar");
        ImGui::TextColored(ui::colors::kBrightGreen, "Double-click to cast");
    }

    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
}

void SpellbookScreen::render(game::GameHandler& gameHandler, pipeline::AssetManager* assetManager) {
    // Spellbook toggle via keybinding (edge-triggered)
    // Customizable key (default: P) from KeybindingManager
    bool spellbookDown = KeybindingManager::getInstance().isActionPressed(
        KeybindingManager::Action::TOGGLE_SPELLBOOK, false);
    if (spellbookDown && !pKeyWasDown) {
        open = !open;
    }
    pKeyWasDown = spellbookDown;

    if (!open) return;

    // Lazy-load DBC data on first open
    if (!dbcLoadAttempted) {
        loadSpellDBC(assetManager);
    }
    if (!iconDbLoaded) {
        loadSpellIconDBC(assetManager);
    }
    if (!skillLineDbLoaded) {
        loadSkillLineDBCs(assetManager);
    }

    // Rebuild categories if spell list changed or skill line data became available
    const auto& spells = gameHandler.getKnownSpells();
    bool skillLinesNowAvailable = !spellToSkillLine.empty() && !categorizedWithSkillLines;
    if (spells.size() != lastKnownSpellCount || skillLinesNowAvailable) {
        categorizeSpells(spells);
    }

    auto* window = core::Application::getInstance().getWindow();
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    float bookW = 380.0f;
    float bookH = std::min(560.0f, screenH - 100.0f);
    float bookX = screenW - bookW - 10.0f;
    float bookY = 80.0f;

    ImGui::SetNextWindowPos(ImVec2(bookX, bookY), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(bookW, bookH), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSizeConstraints(ImVec2(300, 250), ImVec2(screenW, screenH));

    bool windowOpen = open;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8, 8));
    if (ImGui::Begin("Spellbook", &windowOpen)) {
        // Search bar
        ImGui::SetNextItemWidth(-1);
        ImGui::InputTextWithHint("##search", "Search spells...", searchFilter_, sizeof(searchFilter_));

        ImGui::Spacing();

        // Tab bar
        if (ImGui::BeginTabBar("SpellbookTabs")) {
            for (size_t tabIdx = 0; tabIdx < spellTabs.size(); tabIdx++) {
                const auto& tab = spellTabs[tabIdx];

                // Count visible spells (respecting search filter)
                int visibleCount = 0;
                for (const SpellInfo* info : tab.spells) {
                    if (containsCI(info->name, searchFilter_)) visibleCount++;
                }

                char tabLabel[128];
                snprintf(tabLabel, sizeof(tabLabel), "%s (%d)###sbtab%zu",
                         tab.name.c_str(), visibleCount, tabIdx);

                if (ImGui::BeginTabItem(tabLabel)) {
                    if (visibleCount == 0) {
                        if (searchFilter_[0])
                            ImGui::TextDisabled("No matching spells.");
                        else
                            ImGui::TextDisabled("No spells in this category.");
                    }

                    ImGui::BeginChild("SpellList", ImVec2(0, 0), true);

                    const float iconSize = 36.0f;
                    const float rowHeight = iconSize + 4.0f;

                    for (const SpellInfo* info : tab.spells) {
                        // Apply search filter
                        if (!containsCI(info->name, searchFilter_)) continue;

                        ImGui::PushID(static_cast<int>(info->spellId));

                        float cd = gameHandler.getSpellCooldown(info->spellId);
                        bool onCooldown = cd > 0.0f;
                        bool isPassive = info->isPassive();

                        VkDescriptorSet iconTex = getSpellIcon(info->iconId, assetManager);

                        // Row selectable
                        ImGui::Selectable("##row", false,
                            ImGuiSelectableFlags_AllowDoubleClick | ImGuiSelectableFlags_DontClosePopups,
                            ImVec2(0, rowHeight));
                        bool rowHovered = ImGui::IsItemHovered();
                        bool rowClicked = ImGui::IsItemClicked(0);

                        // Right-click context menu
                        if (ImGui::BeginPopupContextItem("##SpellCtx")) {
                            ImGui::TextDisabled("%s", info->name.c_str());
                            if (!info->rank.empty()) {
                                ImGui::SameLine();
                                ImGui::TextDisabled("(%s)", info->rank.c_str());
                            }
                            ImGui::Separator();
                            if (!isPassive) {
                                if (onCooldown) ImGui::BeginDisabled();
                                if (ImGui::MenuItem("Cast")) {
                                    uint64_t tgt = gameHandler.hasTarget() ? gameHandler.getTargetGuid() : 0;
                                    gameHandler.castSpell(info->spellId, tgt);
                                }
                                if (onCooldown) ImGui::EndDisabled();
                            }
                            if (!isPassive) {
                                if (ImGui::MenuItem("Add to Action Bar")) {
                                    const auto& bar = gameHandler.getActionBar();
                                    int firstEmpty = -1;
                                    for (int si = 0; si < game::GameHandler::SLOTS_PER_BAR; ++si) {
                                        if (bar[si].isEmpty()) { firstEmpty = si; break; }
                                    }
                                    if (firstEmpty >= 0) {
                                        gameHandler.setActionBarSlot(firstEmpty,
                                            game::ActionBarSlot::SPELL, info->spellId);
                                    }
                                }
                            }
                            if (ImGui::MenuItem("Copy Spell Link")) {
                                pendingChatSpellLink_ =
                                    game::spellChatLink(info->spellId, info->name);
                            }
                            ImGui::EndPopup();
                        }
                        ImVec2 rMin = ImGui::GetItemRectMin();
                        ImVec2 rMax = ImGui::GetItemRectMax();
                        auto* dl = ImGui::GetWindowDrawList();

                        // Hover highlight
                        if (rowHovered) {
                            dl->AddRectFilled(rMin, rMax, IM_COL32(255, 255, 255, 15), 3.0f);
                        }

                        // Icon background
                        ImVec2 iconMin = rMin;
                        ImVec2 iconMax(rMin.x + iconSize, rMin.y + iconSize);
                        dl->AddRectFilled(iconMin, iconMax, IM_COL32(25, 25, 35, 200), 3.0f);

                        // Icon
                        if (iconTex) {
                            ImU32 tint = (isPassive || onCooldown) ? IM_COL32(150, 150, 150, 255) : IM_COL32(255, 255, 255, 255);
                            dl->AddImage((ImTextureID)(uintptr_t)iconTex,
                                ImVec2(iconMin.x + 1, iconMin.y + 1),
                                ImVec2(iconMax.x - 1, iconMax.y - 1),
                                ImVec2(0, 0), ImVec2(1, 1), tint);
                        }

                        // Icon border
                        ImU32 borderCol;
                        if (isPassive) {
                            borderCol = IM_COL32(180, 180, 50, 200);  // Yellow for passive
                        } else if (onCooldown) {
                            borderCol = IM_COL32(120, 40, 40, 200);   // Red for cooldown
                        } else {
                            borderCol = IM_COL32(100, 100, 120, 200); // Default border
                        }
                        dl->AddRect(iconMin, iconMax, borderCol, 3.0f, 0, 1.5f);

                        // Cooldown overlay on icon
                        if (onCooldown) {
                            // Darkened sweep
                            dl->AddRectFilled(iconMin, iconMax, IM_COL32(0, 0, 0, 120), 3.0f);
                            // Cooldown text centered on icon
                            char cdBuf[16];
                            snprintf(cdBuf, sizeof(cdBuf), "%.0f", cd);
                            ImVec2 cdSize = ImGui::CalcTextSize(cdBuf);
                            ImVec2 cdPos(iconMin.x + (iconSize - cdSize.x) * 0.5f,
                                         iconMin.y + (iconSize - cdSize.y) * 0.5f);
                            dl->AddText(ImVec2(cdPos.x + 1, cdPos.y + 1), IM_COL32(0, 0, 0, 255), cdBuf);
                            dl->AddText(cdPos, IM_COL32(255, 80, 80, 255), cdBuf);
                        }

                        // Spell name
                        float textX = rMin.x + iconSize + 8.0f;
                        float nameY = rMin.y + 2.0f;

                        ImU32 nameCol;
                        if (isPassive) {
                            nameCol = IM_COL32(255, 255, 130, 255);  // Yellow-ish for passive
                        } else if (onCooldown) {
                            nameCol = IM_COL32(150, 150, 150, 255);
                        } else {
                            nameCol = IM_COL32(255, 255, 255, 255);
                        }
                        dl->AddText(ImVec2(textX, nameY), nameCol, info->name.c_str());

                        // Second line: rank or passive/cooldown indicator
                        float subY = nameY + ImGui::GetTextLineHeight() + 1.0f;
                        if (!info->rank.empty()) {
                            dl->AddText(ImVec2(textX, subY),
                                IM_COL32(150, 150, 150, 255), info->rank.c_str());
                        }
                        if (isPassive) {
                            float afterRank = textX;
                            if (!info->rank.empty()) {
                                afterRank += ImGui::CalcTextSize(info->rank.c_str()).x + 8.0f;
                            }
                            dl->AddText(ImVec2(afterRank, subY),
                                IM_COL32(200, 200, 80, 200), "Passive");
                        } else if (onCooldown) {
                            float afterRank = textX;
                            if (!info->rank.empty()) {
                                afterRank += ImGui::CalcTextSize(info->rank.c_str()).x + 8.0f;
                            }
                            char cdText[32];
                            snprintf(cdText, sizeof(cdText), "%.1fs", cd);
                            dl->AddText(ImVec2(afterRank, subY),
                                IM_COL32(255, 100, 100, 200), cdText);
                        }

                        // Interaction
                        if (rowHovered) {
                            // Shift-click to insert spell link into chat
                            if (rowClicked && ImGui::GetIO().KeyShift && !info->name.empty()) {
                                // Retail writes a spell link in ff71d5ff; this said
                                // ffd000, which is the gold an item name uses.
                                pendingChatSpellLink_ =
                                    game::spellChatLink(info->spellId, info->name);
                            }
                            // Start drag on click (not passive, not shift-click)
                            else if (rowClicked && !isPassive && !ImGui::GetIO().KeyShift) {
                                draggingSpell_ = true;
                                dragSpellId_ = info->spellId;
                                dragSpellIconTex_ = iconTex;
                            }

                            // Double-click to cast
                            if (ImGui::IsMouseDoubleClicked(0) && !isPassive && !onCooldown
                                && !ImGui::GetIO().KeyShift) {
                                draggingSpell_ = false;
                                dragSpellId_ = 0;
                                dragSpellIconTex_ = VK_NULL_HANDLE;
                                uint64_t target = gameHandler.hasTarget() ? gameHandler.getTargetGuid() : 0;
                                gameHandler.castSpell(info->spellId, target);
                            }

                            // Tooltip (only when not dragging)
                            if (!draggingSpell_) {
                                renderSpellTooltip(info, gameHandler);
                            }
                        }

                        ImGui::PopID();
                    }

                    ImGui::EndChild();
                    ImGui::EndTabItem();
                }
            }

            ImGui::EndTabBar();
        }
    }
    ImGui::End();
    ImGui::PopStyleVar();

    if (!windowOpen) {
        open = false;
    }

    // Render dragged spell icon at cursor
    if (draggingSpell_ && dragSpellId_ != 0) {
        ImVec2 mousePos = ImGui::GetMousePos();
        float dragSize = 36.0f;
        if (dragSpellIconTex_) {
            ImGui::GetForegroundDrawList()->AddImage(
                (ImTextureID)(uintptr_t)dragSpellIconTex_,
                ImVec2(mousePos.x - dragSize * 0.5f, mousePos.y - dragSize * 0.5f),
                ImVec2(mousePos.x + dragSize * 0.5f, mousePos.y + dragSize * 0.5f));
        } else {
            ImGui::GetForegroundDrawList()->AddRectFilled(
                ImVec2(mousePos.x - dragSize * 0.5f, mousePos.y - dragSize * 0.5f),
                ImVec2(mousePos.x + dragSize * 0.5f, mousePos.y + dragSize * 0.5f),
                IM_COL32(80, 80, 120, 180), 3.0f);
        }

        if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
            draggingSpell_ = false;
            dragSpellId_ = 0;
            dragSpellIconTex_ = VK_NULL_HANDLE;
        }
    }
}

}} // namespace wowee::ui
