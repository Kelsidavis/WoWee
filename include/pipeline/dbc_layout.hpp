#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>

namespace wowee {
namespace pipeline {

/**
 * Maps DBC field names to column indices for a single DBC file.
 * Column indices vary between WoW expansions.
 */
/// Reports a field name the layout does not declare. Once per name, ever.
///
/// Defined in the .cpp so this header stays free of the logger.
void noteMissingLayoutField(const std::string& dbc, const std::string& field);

struct DBCFieldMap {
    std::unordered_map<std::string, uint32_t> fields;
    /// Which file this describes, so a miss can say which one it was.
    std::string dbc;

    /** Get column index by field name. Returns 0xFFFFFFFF if unknown. */
    [[nodiscard]] uint32_t field(const std::string& name) const {
        auto it = fields.find(name);
        if (it != fields.end()) return it->second;
        // A name this layout does not carry, which every caller turns into a
        // read it simply does not do. That is silent, and it is how a whole
        // feature disappears: the installed dbc_layouts.json is a copy taken
        // when the assets were extracted and does not refresh, so a field
        // added to the code outruns it. Reagent0 and EffectItemType0 outran it
        // by nine days and no profession window would open for any character,
        // with nothing logged and spell names still resolving.
        noteMissingLayoutField(dbc, name);
        return 0xFFFFFFFF;
    }

    /** Convenience operator for shorter syntax: layout["Name"] */
    uint32_t operator[](const std::string& name) const { return field(name); }

    /// The same lookup, for a caller that is asking rather than reading.
    ///
    /// A name one expansion has and another does not is not a gap in the
    /// layout, and reporting it as one sends people to re-extract data that is
    /// already correct: Spell.dbc carries SchoolMask from 2.x and SchoolEnum
    /// before it, so the spell book asks for both and uses whichever answers.
    /// Only a caller with nothing to fall back on should report a miss.
    [[nodiscard]] uint32_t tryField(const std::string& name) const {
        auto it = fields.find(name);
        return it != fields.end() ? it->second : 0xFFFFFFFFu;
    }
};

/**
 * Maps DBC file names to their field layouts.
 * Loaded from JSON (e.g. Data/expansions/wotlk/dbc_layouts.json).
 */
class DBCLayout {
public:
    /** Load from JSON file. Returns true if successful. */
    bool loadFromJson(const std::string& path);

    /** Get the field map for a DBC file. Returns nullptr if unknown. */
    [[nodiscard]] const DBCFieldMap* getLayout(const std::string& dbcName) const;

    /** Number of DBC layouts loaded. */
    [[nodiscard]] size_t size() const { return layouts_.size(); }

private:
    std::unordered_map<std::string, DBCFieldMap> layouts_;
};

/**
 * Global active DBC layout (set by Application at startup).
 */
void setActiveDBCLayout(const DBCLayout* layout);
const DBCLayout* getActiveDBCLayout();

// Forward declaration
class DBCFile;

/**
 * Resolved CharSections.dbc field indices.
 *
 * Stock WotLK 3.3.5a uses: Texture1=4, Texture2=5, Texture3=6, Flags=7,
 *   VariationIndex=8, ColorIndex=9  (textures first).
 * Classic/TBC/Turtle and HD-texture WotLK use: VariationIndex=4, ColorIndex=5,
 *   Texture1=6, Texture2=7, Texture3=8, Flags=9  (variation first).
 *
 * detectCharSectionsFields() auto-detects which layout the actual DBC uses
 * by sampling field-4 values: small integers (0-15) => variation-first,
 * large values (string offsets) => texture-first.
 */
struct CharSectionsFields {
    uint32_t raceId         = 1;
    uint32_t sexId          = 2;
    uint32_t baseSection    = 3;
    uint32_t variationIndex = 4;
    uint32_t colorIndex     = 5;
    uint32_t texture1       = 6;
    uint32_t texture2       = 7;
    uint32_t texture3       = 8;
    uint32_t flags          = 9;
};

/**
 * Detect the actual CharSections.dbc field layout by probing record data.
 * @param dbc  Loaded CharSections.dbc file (must not be null).
 * @param csL  JSON-derived field map (may be null - defaults used).
 * @return Resolved field indices for this particular DBC binary.
 */
CharSectionsFields detectCharSectionsFields(const DBCFile* dbc, const DBCFieldMap* csL);

/**
 * Which columns of CharacterFacialHairStyles.dbc hold the geoset numbers.
 *
 * Two shapes ship. The stock 3.3.5a file has nine columns with three unused
 * ones after the variation, so the geosets are at 6, 7, 8. The eight-column
 * file - which is what both installs on this machine carry - puts them at
 * 3, 4, 5 and fills 6 and 7 with zero or 0xCCCCCCCC.
 *
 * Reading the wrong pair is silent: the geosets come out zero, so a beard, a
 * moustache, a pair of sideburns or a draenei's face tendrils simply are not
 * drawn, and nothing anywhere reports it. Index 8 does not even exist in the
 * shorter file.
 *
 * The order within the triple is the documented quirk and is the same in both:
 * the first column is geoset group 100, the second 300, the third 200.
 */
struct FacialHairFields {
    uint32_t geoset100 = 6;
    uint32_t geoset300 = 7;
    uint32_t geoset200 = 8;
};

/**
 * Detect the actual CharacterFacialHairStyles.dbc geoset columns.
 * @param dbc  Loaded CharacterFacialHairStyles.dbc (may be null).
 * @param fhL  JSON-derived field map (may be null - defaults used).
 */
FacialHairFields detectFacialHairFields(const DBCFile* dbc, const DBCFieldMap* fhL);

/**
 * Resolve the SpellItemEnchantment.dbc name (description) field index.
 *
 * The record grew across expansions, so the name sits at a different column in
 * each: Vanilla/Turtle=10, TBC=13, WotLK=14. Reading the wrong column yields an
 * integer that getString() treats as a string-block offset, which silently
 * produces a garbled name ("Rockbiter 3" read as "ockbiter 3") instead of failing.
 *
 * @param dbc  Loaded SpellItemEnchantment.dbc (must not be null).
 * @param sieL JSON-derived field map (may be null - field count decides).
 * @return Name field index for this particular DBC binary.
 */
/// Spell.dbc's three timing columns, which move together as the record grows.
struct SpellTimingFields {
    uint32_t castingTimeIndex;      ///< → SpellCastTimes.dbc
    uint32_t recoveryTime;          ///< the spell's own cooldown, in ms
    uint32_t categoryRecoveryTime;  ///< its category's cooldown, in ms
};

/**
 * Resolve those three from the file rather than from dbc_layouts.json.
 *
 * The record is 173 fields in Vanilla, 216 in TBC and 234 in WotLK, and the
 * three columns sit at 18/19/20, 22/23/24 and 28/29/30 respectively. Which one
 * an install has is a property of the file, not of the expansion being played -
 * a Classic profile with no Spell.dbc of its own reads the shared WotLK one.
 *
 * The JSON had CastingTimeIndex at 15 for Classic and Turtle, which is
 * RequiresSpellFocus and reads zero for every spell, and named neither cooldown
 * column at all outside TBC. A missing name answers 0xFFFFFFFF, the read is
 * skipped, and every cooldown this client works out for itself came back zero.
 *
 * @param dbc    Loaded Spell.dbc (must not be null).
 * @param spellL JSON-derived field map, consulted only for CastingTimeIndex and
 *               only when it agrees with the shape - see the note above.
 */
SpellTimingFields detectSpellTimingFields(const DBCFile* dbc, const DBCFieldMap* spellL);

uint32_t detectEnchantmentNameField(const DBCFile* dbc, const DBCFieldMap* sieL);

/**
 * Resolve the SpellItemEnchantment.dbc ItemVisual field index, which likewise
 * shifted with the record: Vanilla/Turtle=19, TBC=30, WotLK=31.
 */
uint32_t detectEnchantmentItemVisualField(const DBCFile* dbc, const DBCFieldMap* sieL);

/// SpellItemEnchantment's Src_ItemID column - the gem an enchantment came from.
/// Zero when the file has no such column, which is every pre-TBC shape.
uint32_t detectEnchantmentGemItemField(const DBCFile* dbc, const DBCFieldMap* sieL);

/**
 * Model paths for the effect an enchant puts on the item it is applied to - the
 * glint on a freshly sharpened blade.
 *
 * Chain: SpellItemEnchantment.ItemVisual → ItemVisuals.dbc (5 effect slots) →
 * ItemVisualEffects.dbc (M2 path). Slot index is meaningful: it selects which
 * attachment point on the item model the effect hangs from, so gaps are kept.
 *
 * @return Per-slot model paths; empty strings for unused slots.
 */
std::array<std::string, 5> resolveEnchantItemVisuals(uint32_t enchantId,
                                                     const DBCFile* spellItemEnchantment,
                                                     const DBCFile* itemVisuals,
                                                     const DBCFile* itemVisualEffects,
                                                     const DBCFieldMap* sieL);

/**
 * The tail of that chain, entered directly with an ItemVisuals.dbc id.
 *
 * SMSG_CHAR_ENUM reports an equipped item's enchant as its ItemVisual id already,
 * so the character-select preview has no SpellItemEnchantment lookup to do.
 */
std::array<std::string, 5> resolveItemVisualModels(uint32_t itemVisualId,
                                                   const DBCFile* itemVisuals,
                                                   const DBCFile* itemVisualEffects);

} // namespace pipeline
} // namespace wowee
