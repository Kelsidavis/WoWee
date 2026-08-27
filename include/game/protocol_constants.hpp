#pragma once

#include <cstdint>

// WoW 3.3.5a (12340) protocol constants.
// Centralised so every handler references a single source of truth.

namespace wowee {
namespace game {

// ---------------------------------------------------------------------------
// Currency
// ---------------------------------------------------------------------------
constexpr uint32_t COPPER_PER_GOLD   = 10000;
constexpr uint32_t COPPER_PER_SILVER = 100;

// ---------------------------------------------------------------------------
// Unit flags (UNIT_FIELD_FLAGS - index 59 in UnitFields for 3.3.5a;
// 46 in Classic/TBC/Turtle. Bitmask values below are stable across expansions.)
// ---------------------------------------------------------------------------
constexpr uint32_t UNIT_FLAG_TAXI_FLIGHT = 0x00000100;
constexpr uint32_t UNIT_FLAG_IN_COMBAT   = 0x00080000;
/// The server's own mark for a unit the player is not meant to interact with:
/// the triggers and bunnies that scripts hang their effects on. They are
/// ordinary units on the wire, with health and a name, and this flag is the
/// only thing that says they are scenery.
/// Controlled by a player rather than acting on its own. This is what
/// separates a pet from a guardian: both are summons and both carry a
/// summoner, and only the one the player steers answers to this.
constexpr uint32_t UNIT_FLAG_PLAYER_CONTROLLED = 0x00000008;
constexpr uint32_t UNIT_FLAG_NOT_SELECTABLE = 0x02000000;

// Unit visibility flags (byte 2 of UNIT_FIELD_BYTES_1).
// CREEP marks a unit using the client-side stealth presentation. The server
// still decides whether an undetected stealthed unit is sent to the client.
constexpr uint8_t UNIT_VIS_FLAG_CREEP = 0x02;

// ---------------------------------------------------------------------------
// NPC flags (UNIT_NPC_FLAGS - index 82 in UnitFields for 3.3.5a;
// 147 in Classic/Turtle. Bitmask values below are stable across expansions.)
// ---------------------------------------------------------------------------
// The bits are a run: gossip, questgiver, two unused, three trainers, then the
// five vendors, repair, the flight master and the two spirits. 0x04 is one of
// the two nothing sets, which is what NPC_FLAG_VENDOR was - so no unit in the
// game ever answered it and the vendor cursor never drew.
constexpr uint32_t NPC_FLAG_GOSSIP         = 0x00000001;
constexpr uint32_t NPC_FLAG_QUESTGIVER     = 0x00000002;
constexpr uint32_t NPC_FLAG_TRAINER        = 0x00000010;
constexpr uint32_t NPC_FLAG_VENDOR         = 0x00000080;
constexpr uint32_t NPC_FLAG_VENDOR_AMMO    = 0x00000100;
constexpr uint32_t NPC_FLAG_VENDOR_FOOD    = 0x00000200;
constexpr uint32_t NPC_FLAG_VENDOR_POISON  = 0x00000400;
constexpr uint32_t NPC_FLAG_VENDOR_REAGENT = 0x00000800;
/// Anything with something to sell. A food or reagent seller usually carries
/// the plain vendor bit as well, and a few carry only their own.
constexpr uint32_t NPC_FLAG_ANY_VENDOR =
    NPC_FLAG_VENDOR | NPC_FLAG_VENDOR_AMMO | NPC_FLAG_VENDOR_FOOD |
    NPC_FLAG_VENDOR_POISON | NPC_FLAG_VENDOR_REAGENT;
constexpr uint32_t NPC_FLAG_REPAIR         = 0x00001000;
constexpr uint32_t NPC_FLAG_FLIGHT_MASTER  = 0x00002000;
// The healer comes before the guide. These two were the other way round; every
// caller tests them as a pair, so the swap changed no behaviour and would have
// waited for whoever first asked about one of them alone.
constexpr uint32_t NPC_FLAG_SPIRIT_HEALER  = 0x00004000;
constexpr uint32_t NPC_FLAG_SPIRIT_GUIDE   = 0x00008000;
constexpr uint32_t NPC_FLAG_INNKEEPER      = 0x00010000;
constexpr uint32_t NPC_FLAG_BANKER         = 0x00020000;
constexpr uint32_t NPC_FLAG_AUCTIONEER     = 0x00200000;

// ---------------------------------------------------------------------------
// Default action-bar spell IDs
// ---------------------------------------------------------------------------
constexpr uint32_t SPELL_ID_ATTACK     = 6603;
constexpr uint32_t SPELL_ID_HEARTHSTONE = 8690;
/// The stone itself. Casting it arrives as either id depending on whether the
/// item or its spell was used, so both are checked wherever one is.
constexpr uint32_t ITEM_ID_HEARTHSTONE = 6948;

/// Item class and subclass numbers, as the item query response carries them.
///
/// A bandage is the pair below, and both the inventory and the spell handler
/// needed to know it - each with its own copy of the two numbers and its own
/// predicate over them. The two agreed, but a bandage is identified by the
/// pair rather than by either half, so the fact belongs in one place.
constexpr uint32_t ITEM_CLASS_CONSUMABLE = 0;
constexpr uint32_t ITEM_SUBCLASS_SCROLL  = 4;
constexpr uint32_t ITEM_SUBCLASS_BANDAGE = 7;
/// Class 12 is the quest item. It is never equipment, whatever its INVTYPE
/// says - and some of them do carry one.
constexpr uint32_t ITEM_CLASS_QUEST      = 12;

// ---------------------------------------------------------------------------
// Class IDs
// ---------------------------------------------------------------------------
constexpr uint32_t CLASS_WARRIOR = 1;
constexpr uint32_t CLASS_PALADIN = 2;
constexpr uint32_t CLASS_HUNTER  = 3;
constexpr uint32_t CLASS_ROGUE   = 4;
constexpr uint32_t CLASS_PRIEST  = 5;
constexpr uint32_t CLASS_DK      = 6;
constexpr uint32_t CLASS_SHAMAN  = 7;
constexpr uint32_t CLASS_MAGE    = 8;
constexpr uint32_t CLASS_WARLOCK = 9;
constexpr uint32_t CLASS_DRUID   = 11;

// ---------------------------------------------------------------------------
// Class-specific stance / form / presence spell IDs
// ---------------------------------------------------------------------------
// Warrior stances
constexpr uint32_t SPELL_BATTLE_STANCE    = 2457;
constexpr uint32_t SPELL_DEFENSIVE_STANCE = 71;
constexpr uint32_t SPELL_BERSERKER_STANCE = 2458;

// Death Knight presences
constexpr uint32_t SPELL_BLOOD_PRESENCE  = 48266;
constexpr uint32_t SPELL_FROST_PRESENCE  = 48263;
constexpr uint32_t SPELL_UNHOLY_PRESENCE = 48265;

// Druid forms
constexpr uint32_t SPELL_BEAR_FORM       = 5487;
constexpr uint32_t SPELL_DIRE_BEAR_FORM  = 9634;
constexpr uint32_t SPELL_CAT_FORM        = 768;
constexpr uint32_t SPELL_AQUATIC_FORM    = 1066;
constexpr uint32_t SPELL_TRAVEL_FORM     = 783;
constexpr uint32_t SPELL_MOONKIN_FORM    = 24858;
constexpr uint32_t SPELL_FLIGHT_FORM     = 33943;
constexpr uint32_t SPELL_SWIFT_FLIGHT    = 40120;
constexpr uint32_t SPELL_TREE_OF_LIFE    = 33891;

// Rogue
constexpr uint32_t SPELL_STEALTH         = 1784;

// Priest
constexpr uint32_t SPELL_SHADOWFORM      = 15473;

// ---------------------------------------------------------------------------
// Session / network timing
// ---------------------------------------------------------------------------
constexpr uint32_t RX_SILENCE_WARNING_MS       = 10000;  // 10 s
constexpr uint32_t RX_SILENCE_CRITICAL_MS      = 15000;  // 15 s
constexpr float    WARDEN_GATE_LOG_INTERVAL_SEC = 30.0f;
constexpr float    CLASSIC_PING_INTERVAL_SEC    = 10.0f;

// ---------------------------------------------------------------------------
// Heartbeat / area-trigger intervals (seconds)
// ---------------------------------------------------------------------------
constexpr float HEARTBEAT_INTERVAL_TAXI             = 0.25f;
constexpr float HEARTBEAT_INTERVAL_STATIONARY_COMBAT = 0.75f;
constexpr float HEARTBEAT_INTERVAL_MOVING_COMBAT    = 0.20f;
constexpr float AREA_TRIGGER_CHECK_INTERVAL         = 0.25f;

// ---------------------------------------------------------------------------
// Gameplay distance thresholds
// ---------------------------------------------------------------------------
constexpr float ENTITY_UPDATE_RADIUS      = 150.0f;
constexpr float NPC_INTERACT_MAX_DISTANCE = 15.0f;

// ---------------------------------------------------------------------------
// Skill categories (from SkillLine DBC)
// ---------------------------------------------------------------------------
constexpr uint32_t SKILL_CATEGORY_PROFESSION = 11;
constexpr uint32_t SKILL_CATEGORY_SECONDARY  = 9;

// ---------------------------------------------------------------------------
// DBC field-index sentinel (field lookup failure)
// ---------------------------------------------------------------------------
constexpr uint32_t DBC_FIELD_INVALID = 0xFFFFFFFF;

// ---------------------------------------------------------------------------
// Appearance byte packing
// ---------------------------------------------------------------------------
constexpr uint32_t APPEARANCE_SKIN_MASK       = 0xFF;
constexpr uint32_t APPEARANCE_FACE_SHIFT      = 8;
constexpr uint32_t APPEARANCE_HAIRSTYLE_SHIFT = 16;
constexpr uint32_t APPEARANCE_HAIRCOLOR_SHIFT = 24;

// ---------------------------------------------------------------------------
// Critter detection
// ---------------------------------------------------------------------------
constexpr uint32_t CRITTER_MAX_HEALTH_THRESHOLD = 100;

} // namespace game
} // namespace wowee
