#include "pipeline/dbc_layout.hpp"
#include "pipeline/dbc_loader.hpp"
#include "core/logger.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <mutex>
#include <set>

namespace wowee {
namespace pipeline {

static const DBCLayout* g_activeDBCLayout = nullptr;

void noteMissingLayoutField(const std::string& dbc, const std::string& field) {
    // Deduped, so this is at worst one line per missing name for the life of
    // the process - a list of what the installed layout is behind on, rather
    // than a message repeated from inside a loop.
    static std::mutex lock;
    static std::set<std::string> said;
    const std::string key = dbc + "." + field;
    {
        std::lock_guard<std::mutex> guard(lock);
        if (!said.insert(key).second) return;
    }
    LOG_WARNING("DBCLayout: ", dbc.empty() ? "a layout" : dbc.c_str(),
                " does not declare '", field,
                "', so everything read from that column is skipped. The file is "
                "dbc_layouts.json in the data directory, which is copied there "
                "when assets are extracted and does not refresh itself - "
                "re-extract, or copy the one from Data/expansions.");
}

void setActiveDBCLayout(const DBCLayout* layout) { g_activeDBCLayout = layout; }
const DBCLayout* getActiveDBCLayout() { return g_activeDBCLayout; }

bool DBCLayout::loadFromJson(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        LOG_WARNING("DBCLayout: cannot open ", path);
        return false;
    }

    std::string json((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());

    layouts_.clear();
    size_t loaded = 0;
    size_t pos = 0;

    // Parse top-level object: { "DbcName": { "FieldName": index, ... }, ... }
    // Find the first '{'
    pos = json.find('{', pos);
    if (pos == std::string::npos) return false;
    ++pos;

    while (pos < json.size()) {
        // Find DBC name key
        size_t dbcKeyStart = json.find('"', pos);
        if (dbcKeyStart == std::string::npos) break;
        size_t dbcKeyEnd = json.find('"', dbcKeyStart + 1);
        if (dbcKeyEnd == std::string::npos) break;
        std::string dbcName = json.substr(dbcKeyStart + 1, dbcKeyEnd - dbcKeyStart - 1);

        // Find the nested object '{'
        size_t objStart = json.find('{', dbcKeyEnd);
        if (objStart == std::string::npos) break;

        // Find the matching '}'
        size_t objEnd = json.find('}', objStart);
        if (objEnd == std::string::npos) break;

        // Parse the inner object
        std::string inner = json.substr(objStart + 1, objEnd - objStart - 1);
        DBCFieldMap fieldMap;
        size_t ipos = 0;
        while (ipos < inner.size()) {
            size_t fkStart = inner.find('"', ipos);
            if (fkStart == std::string::npos) break;
            size_t fkEnd = inner.find('"', fkStart + 1);
            if (fkEnd == std::string::npos) break;
            std::string fieldName = inner.substr(fkStart + 1, fkEnd - fkStart - 1);

            size_t colon = inner.find(':', fkEnd);
            if (colon == std::string::npos) break;
            size_t valStart = colon + 1;
            while (valStart < inner.size() && (inner[valStart] == ' ' || inner[valStart] == '\t' ||
                   inner[valStart] == '\r' || inner[valStart] == '\n'))
                ++valStart;
            size_t valEnd = inner.find_first_of(",}\r\n", valStart);
            if (valEnd == std::string::npos) valEnd = inner.size();
            std::string valStr = inner.substr(valStart, valEnd - valStart);
            while (!valStr.empty() && (valStr.back() == ' ' || valStr.back() == '\t'))
                valStr.pop_back();

            try {
                uint32_t idx = static_cast<uint32_t>(std::stoul(valStr));
                fieldMap.fields[fieldName] = idx;
            } catch (...) {}

            ipos = valEnd + 1;
        }

        if (!fieldMap.fields.empty()) {
            fieldMap.dbc = dbcName;
            layouts_[dbcName] = std::move(fieldMap);
            ++loaded;
        }

        pos = objEnd + 1;
    }

    LOG_INFO("DBCLayout: loaded ", loaded, " layouts from ", path);
    return loaded > 0;
}

const DBCFieldMap* DBCLayout::getLayout(const std::string& dbcName) const {
    auto it = layouts_.find(dbcName);
    return (it != layouts_.end()) ? &it->second : nullptr;
}

FacialHairFields detectFacialHairFields(const DBCFile* dbc, const DBCFieldMap* fhL) {
    static const DBCFile* s_cached = nullptr;
    static FacialHairFields s_cachedResult;
    if (dbc && dbc == s_cached) return s_cachedResult;

    FacialHairFields f;
    f.geoset100 = fhL ? (*fhL)["Geoset100"] : 6;
    f.geoset300 = fhL ? (*fhL)["Geoset300"] : 7;
    f.geoset200 = fhL ? (*fhL)["Geoset200"] : 8;
    if (!dbc || dbc->getRecordCount() == 0) return f;

    // The field count decides it, and decides it exactly: the nine-column file
    // is the only one where column 8 exists at all. Asked this way there is
    // nothing to probe and nothing to get wrong on an unusual row.
    if (dbc->getFieldCount() < 9) {
        f.geoset100 = 3;
        f.geoset300 = 4;
        f.geoset200 = 5;
    }
    s_cached = dbc;
    s_cachedResult = f;
    return f;
}

CharSectionsFields detectCharSectionsFields(const DBCFile* dbc, const DBCFieldMap* csL) {
    // Cache: avoid re-probing the same DBC on every call.
    static const DBCFile* s_cachedDbc = nullptr;
    static CharSectionsFields s_cachedResult;
    if (dbc && dbc == s_cachedDbc) return s_cachedResult;

    CharSectionsFields f;
    if (!dbc || dbc->getRecordCount() == 0) return f;

    // Start from the JSON layout (or defaults matching Classic-style: variation-first)
    f.raceId         = csL ? (*csL)["RaceID"]         : 1;
    f.sexId          = csL ? (*csL)["SexID"]           : 2;
    f.baseSection    = csL ? (*csL)["BaseSection"]     : 3;
    f.variationIndex = csL ? (*csL)["VariationIndex"]  : 4;
    f.colorIndex     = csL ? (*csL)["ColorIndex"]      : 5;
    f.texture1       = csL ? (*csL)["Texture1"]        : 6;
    f.texture2       = csL ? (*csL)["Texture2"]        : 7;
    f.texture3       = csL ? (*csL)["Texture3"]        : 8;
    f.flags          = csL ? (*csL)["Flags"]           : 9;

    // Auto-detect: probe the field that the JSON layout says is VariationIndex.
    // In Classic-style layout, VariationIndex (field 4) holds small integers 0-15.
    // In stock WotLK layout, field 4 is actually Texture1 (a string block offset, typically > 100).
    // Sample up to 20 records and check if all field-4 values are small integers.
    uint32_t probeField = f.variationIndex;
    if (probeField >= dbc->getFieldCount()) {
        s_cachedDbc = dbc;
        s_cachedResult = f;
        return f;  // safety
    }

    uint32_t sampleCount = std::min(dbc->getRecordCount(), 20u);
    uint32_t largeCount = 0;
    uint32_t smallCount = 0;
    for (uint32_t r = 0; r < sampleCount; r++) {
        uint32_t val = dbc->getUInt32(r, probeField);
        if (val > 50) {
            ++largeCount;
        } else {
            ++smallCount;
        }
    }

    // If most sampled values are large, the JSON layout's VariationIndex field
    // actually contains string offsets => this is stock WotLK (texture-first).
    // Swap to texture-first layout: Tex1=4, Tex2=5, Tex3=6, Flags=7, Var=8, Color=9.
    if (largeCount > smallCount) {
        uint32_t base = probeField;  // the field index the JSON calls VariationIndex (typically 4)
        f.texture1       = base;
        f.texture2       = base + 1;
        f.texture3       = base + 2;
        f.flags          = base + 3;
        f.variationIndex = base + 4;
        f.colorIndex     = base + 5;
        LOG_INFO("CharSections.dbc: detected stock WotLK layout (textures-first at field ", base, ")");
    } else {
        LOG_INFO("CharSections.dbc: detected Classic-style layout (variation-first at field ", probeField, ")");
    }

    s_cachedDbc = dbc;
    s_cachedResult = f;
    return f;
}

SpellTimingFields detectSpellTimingFields(const DBCFile* dbc, const DBCFieldMap* spellL) {
    // WotLK is the fallback: it is the file this client ships and the one a
    // profile without its own Spell.dbc falls through to.
    SpellTimingFields f{.castingTimeIndex = 28, .recoveryTime = 29, .categoryRecoveryTime = 30};
    if (!dbc || dbc->getRecordCount() == 0) return f;

    const uint32_t fieldCount = dbc->getFieldCount();
    if (fieldCount < 216)      f = {.castingTimeIndex = 18, .recoveryTime = 19, .categoryRecoveryTime = 20};  // Vanilla 1.12 / Turtle
    else if (fieldCount < 234) f = {.castingTimeIndex = 22, .recoveryTime = 23, .categoryRecoveryTime = 24};  // TBC 2.4.3

    // A layout may name CastingTimeIndex, and TBC's and WotLK's are right. It is
    // taken only when it matches the shape the file actually has, because the
    // two that disagreed were both pointing at a column of zeros.
    if (spellL) {
        const uint32_t named = spellL->field("CastingTimeIndex");
        if (named == f.castingTimeIndex) f.castingTimeIndex = named;
    }
    if (f.categoryRecoveryTime >= fieldCount) return {.castingTimeIndex = 0, .recoveryTime = 0, .categoryRecoveryTime = 0};
    return f;
}

uint32_t detectEnchantmentNameField(const DBCFile* dbc, const DBCFieldMap* sieL) {
    if (!dbc || dbc->getRecordCount() == 0) return 14;

    const uint32_t fieldCount = dbc->getFieldCount();

    // The record width identifies the expansion: Vanilla=21, TBC=34, WotLK=38.
    // Each added effect/gem columns ahead of the localized name block.
    uint32_t nameField;
    if (fieldCount >= 38)      nameField = 14;  // WotLK 3.3.5a
    else if (fieldCount >= 34) nameField = 13;  // TBC 2.4.3
    else                       nameField = 10;  // Vanilla 1.12 / Turtle

    // A layout override wins, but only when it is in range - a stale index here
    // reads an integer column as a string offset and garbles every enchant name.
    if (sieL) {
        uint32_t f = sieL->field("Name");
        if (f != 0xFFFFFFFF && f < fieldCount) nameField = f;
    }
    if (nameField >= fieldCount) nameField = 0;
    return nameField;
}

uint32_t detectEnchantmentItemVisualField(const DBCFile* dbc, const DBCFieldMap* sieL) {
    if (!dbc || dbc->getRecordCount() == 0) return 31;

    const uint32_t fieldCount = dbc->getFieldCount();

    uint32_t visualField;
    if (fieldCount >= 38)      visualField = 31;  // WotLK 3.3.5a
    else if (fieldCount >= 34) visualField = 30;  // TBC 2.4.3
    else                       visualField = 19;  // Vanilla 1.12 / Turtle

    if (sieL) {
        uint32_t f = sieL->field("ItemVisual");
        if (f != 0xFFFFFFFF && f < fieldCount) visualField = f;
    }
    if (visualField >= fieldCount) return 0;
    return visualField;
}

uint32_t detectEnchantmentGemItemField(const DBCFile* dbc, const DBCFieldMap* sieL) {
    if (!dbc || dbc->getRecordCount() == 0) return 0;

    const uint32_t fieldCount = dbc->getFieldCount();

    // Src_ItemID: the gem this enchantment came out of, and the only route from
    // an enchantment sitting in a socket back to what is in the socket. It
    // trails the localized name block like ItemVisual and Flags do, so it moves
    // with them between the two shapes - WotLK 38 fields, TBC 34.
    //
    // Verified against the files rather than counted off a wiki page: field 33
    // of the WotLK file has 617 non-zero values and every one is in item-id
    // range, and field 32 of the TBC file has 260 that are. Vanilla has 24
    // fields and no gems to name, so there is nothing to point at.
    uint32_t gemField;
    if (fieldCount >= 38)      gemField = 33;  // WotLK 3.3.5a
    else if (fieldCount >= 34) gemField = 32;  // TBC 2.4.3
    else                       return 0;       // Vanilla 1.12 / Turtle: no gems

    if (sieL) {
        uint32_t f = sieL->field("SrcItemID");
        if (f != 0xFFFFFFFF && f < fieldCount) gemField = f;
    }
    if (gemField >= fieldCount) return 0;
    return gemField;
}

std::array<std::string, 5> resolveItemVisualModels(uint32_t itemVisualId,
                                                   const DBCFile* itemVisuals,
                                                   const DBCFile* itemVisualEffects) {
    std::array<std::string, 5> models;
    if (itemVisualId == 0 || !itemVisuals || !itemVisualEffects) return models;

    int32_t visualRow = itemVisuals->findRecordById(itemVisualId);
    if (visualRow < 0) return models;

    // ItemVisuals.dbc: ID + 5 effect slots. Slot i attaches at the item's
    // attachment point i, so an empty slot must stay empty rather than shift.
    for (uint32_t slot = 0; slot < 5; ++slot) {
        const uint32_t field = slot + 1;
        if (field >= itemVisuals->getFieldCount()) break;
        uint32_t effectId = itemVisuals->getUInt32(static_cast<uint32_t>(visualRow), field);
        if (effectId == 0) continue;

        int32_t effectRow = itemVisualEffects->findRecordById(effectId);
        if (effectRow < 0) continue;
        models[slot] = itemVisualEffects->getString(static_cast<uint32_t>(effectRow), 1);
    }
    return models;
}

std::array<std::string, 5> resolveEnchantItemVisuals(uint32_t enchantId,
                                                     const DBCFile* spellItemEnchantment,
                                                     const DBCFile* itemVisuals,
                                                     const DBCFile* itemVisualEffects,
                                                     const DBCFieldMap* sieL) {
    std::array<std::string, 5> models;
    if (enchantId == 0 || !spellItemEnchantment) return models;

    int32_t enchantRow = spellItemEnchantment->findRecordById(enchantId);
    if (enchantRow < 0) return models;

    const uint32_t visualField = detectEnchantmentItemVisualField(spellItemEnchantment, sieL);
    const uint32_t visualId = spellItemEnchantment->getUInt32(static_cast<uint32_t>(enchantRow), visualField);
    if (visualId == 0) return models;  // most enchants have no visual

    return resolveItemVisualModels(visualId, itemVisuals, itemVisualEffects);
}

} // namespace pipeline
} // namespace wowee
