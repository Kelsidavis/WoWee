#include "pipeline/wowee_spell_reagents.hpp"
#include "pipeline/wowee_binary_io.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>

namespace wowee {
namespace pipeline {

namespace {

constexpr char kMagic[4] = {'W', 'S', 'P', 'R'};
constexpr uint32_t kVersion = 1;
constexpr char kExtension[] = ".wspr";

} // namespace

const WoweeSpellReagent::Entry*
WoweeSpellReagent::findById(uint32_t reagentSetId) const {
    for (const auto& e : entries)
        if (e.reagentSetId == reagentSetId) return &e;
    return nullptr;
}

const WoweeSpellReagent::Entry*
WoweeSpellReagent::findBySpell(uint32_t spellId) const {
    for (const auto& e : entries)
        if (e.spellId == spellId) return &e;
    return nullptr;
}

int WoweeSpellReagent::usedSlotCount(uint32_t reagentSetId) const {
    const Entry* e = findById(reagentSetId);
    if (!e) return 0;
    int n = 0;
    for (uint32_t itemId : e->reagentItemId) {
        if (itemId != 0) ++n;
    }
    return n;
}

const char* WoweeSpellReagent::reagentKindName(uint8_t k) {
    switch (k) {
        case Standard:    return "standard";
        case SoulShard:   return "soul-shard";
        case FocusedItem: return "focused-item";
        case Catalyst:    return "catalyst";
        case Tradeable:   return "tradeable";
        default:          return "unknown";
    }
}

bool WoweeSpellReagentLoader::save(const WoweeSpellReagent& cat,
                     const std::string& basePath) {
    return saveCatalog(cat, basePath, kMagic, kVersion, kExtension,
                       [](std::ofstream& os, const WoweeSpellReagent::Entry& e) {
        writePOD(os, e.reagentSetId);
        writeStr(os, e.name);
        writeStr(os, e.description);
        writePOD(os, e.spellId);
        for (uint32_t itemId : e.reagentItemId)
            writePOD(os, itemId);
        for (uint32_t count : e.reagentCount)
            writePOD(os, count);
        writePOD(os, e.reagentKind);
        writePOD(os, e.pad0);
        writePOD(os, e.pad1);
        writePOD(os, e.pad2);
        writePOD(os, e.iconColorRGBA);
                       });
}

WoweeSpellReagent WoweeSpellReagentLoader::load(
    const std::string& basePath) {
    return loadCatalog<WoweeSpellReagent>(basePath, kMagic, kVersion, kExtension,
                              [](std::ifstream& is, WoweeSpellReagent::Entry& e) {
        if (!readPOD(is, e.reagentSetId)) { return false; }
        if (!readStr(is, e.name) || !readStr(is, e.description)) { return false; }
        if (!readPOD(is, e.spellId)) { return false; }
        for (uint32_t& itemId : e.reagentItemId) {
            if (!readPOD(is, itemId)) { return false; }
        }
        for (uint32_t& count : e.reagentCount) {
            if (!readPOD(is, count)) { return false; }
        }
        if (!readPOD(is, e.reagentKind) ||
            !readPOD(is, e.pad0) ||
            !readPOD(is, e.pad1) ||
            !readPOD(is, e.pad2) ||
            !readPOD(is, e.iconColorRGBA)) { return false; }
                                  return true;
                              });
}

bool WoweeSpellReagentLoader::exists(const std::string& basePath) {
    return catalogExists(basePath, kExtension);
}

WoweeSpellReagent WoweeSpellReagentLoader::makeMage(
    const std::string& catalogName) {
    using R = WoweeSpellReagent;
    WoweeSpellReagent c;
    c.name = catalogName;
    auto add = [&](uint32_t id, const char* name, uint32_t spellId,
                    uint32_t runeItemId, const char* desc) {
        R::Entry e;
        e.reagentSetId = id; e.name = name; e.description = desc;
        e.spellId = spellId;
        e.reagentItemId[0] = runeItemId;
        e.reagentCount[0] = 1;
        e.reagentKind = R::Standard;
        e.iconColorRGBA = packRgba(80, 140, 240);   // mage blue
        c.entries.push_back(e);
    };
    // Spell ids match WoW 3.3.5a teleport/portal spell ids;
    // item id 17031 = Rune of Teleportation (single-use
    // teleport runes), 17032 = Rune of Portals (groups).
    add(1, "TeleportStormwind", 3565,  17031,
        "Teleport: Stormwind - consumes 1 Rune of Teleportation.");
    add(2, "TeleportIronforge", 3562,  17031,
        "Teleport: Ironforge - consumes 1 Rune of Teleportation.");
    add(3, "TeleportDarnassus", 3561,  17031,
        "Teleport: Darnassus - consumes 1 Rune of Teleportation.");
    add(4, "PortalStormwind",  10059, 17032,
        "Portal: Stormwind - consumes 1 Rune of Portals.");
    return c;
}

WoweeSpellReagent WoweeSpellReagentLoader::makeWarlock(
    const std::string& catalogName) {
    using R = WoweeSpellReagent;
    WoweeSpellReagent c;
    c.name = catalogName;
    auto add = [&](uint32_t id, const char* name, uint32_t spellId,
                    const char* desc) {
        R::Entry e;
        e.reagentSetId = id; e.name = name; e.description = desc;
        e.spellId = spellId;
        // Item 6265 = Soul Shard.
        e.reagentItemId[0] = 6265;
        e.reagentCount[0] = 1;
        e.reagentKind = R::SoulShard;
        e.iconColorRGBA = packRgba(140, 60, 200);   // warlock purple
        c.entries.push_back(e);
    };
    // Warlock summon spell ids; each consumes 1 Soul Shard.
    add(100, "SummonImp",        688, "Summon Imp - consumes 1 Soul Shard.");
    add(101, "SummonVoidwalker",  697, "Summon Voidwalker - consumes 1 Soul Shard.");
    add(102, "SummonSuccubus",   712, "Summon Succubus - consumes 1 Soul Shard.");
    add(103, "SummonFelhunter",  691, "Summon Felhunter - consumes 1 Soul Shard.");
    return c;
}

WoweeSpellReagent WoweeSpellReagentLoader::makeRez(
    const std::string& catalogName) {
    using R = WoweeSpellReagent;
    WoweeSpellReagent c;
    c.name = catalogName;
    // Mix of three resurrection styles, demonstrating each
    // ReagentKind. Different presets to show kind variety.
    R::Entry ankh;
    ankh.reagentSetId = 200;
    ankh.name = "ShamanReincarnation";
    ankh.description = "Reincarnation - consumes 1 Ankh of Reincarnation.";
    ankh.spellId = 20608;
    ankh.reagentItemId[0] = 17030;   // Ankh
    ankh.reagentCount[0] = 1;
    ankh.reagentKind = R::Standard;
    ankh.iconColorRGBA = packRgba(220, 200, 100);
    c.entries.push_back(ankh);

    R::Entry priestRez;
    priestRez.reagentSetId = 201;
    priestRez.name = "PriestResurrection";
    priestRez.description = "Resurrection - requires Holy Candle "
        "(focused, NOT consumed on cast).";
    priestRez.spellId = 2006;
    priestRez.reagentItemId[0] = 17029;   // Holy Candle
    priestRez.reagentCount[0] = 1;
    priestRez.reagentKind = R::FocusedItem;
    priestRez.iconColorRGBA = packRgba(240, 240, 200);
    c.entries.push_back(priestRez);

    R::Entry druidRebirth;
    druidRebirth.reagentSetId = 202;
    druidRebirth.name = "DruidRebirth";
    druidRebirth.description = "Rebirth - combat rez, no reagent in "
        "WotLK+ (legacy entry kept for migration).";
    druidRebirth.spellId = 20484;
    // Empty reagent slots - Rebirth is reagent-free in
    // 3.3.5a but the entry is kept so the engine has a
    // consistent lookup point.
    druidRebirth.reagentKind = R::Standard;
    druidRebirth.iconColorRGBA = packRgba(100, 200, 100);
    c.entries.push_back(druidRebirth);
    return c;
}

} // namespace pipeline
} // namespace wowee
