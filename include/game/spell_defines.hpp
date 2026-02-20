#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace game {

/**
 * Aura slot data for buff/debuff tracking
 */
struct AuraSlot {
    uint32_t spellId = 0;
    uint8_t flags = 0;         // Active, positive/negative, etc.
    uint8_t level = 0;
    uint8_t charges = 0;
    int32_t durationMs = -1;
    int32_t maxDurationMs = -1;
    uint64_t casterGuid = 0;
    uint64_t receivedAtMs = 0; // Client timestamp (ms) when durationMs was set

    bool isEmpty() const { return spellId == 0; }
    // Remaining duration in ms, counting down from when the packet was received
    int32_t getRemainingMs(uint64_t nowMs) const {
        if (durationMs < 0) return -1;
        uint64_t elapsed = (nowMs > receivedAtMs) ? (nowMs - receivedAtMs) : 0;
        int32_t remaining = durationMs - static_cast<int32_t>(elapsed);
        return (remaining > 0) ? remaining : 0;
    }
};

/**
 * Action bar slot
 */
struct ActionBarSlot {
    enum Type : uint8_t { EMPTY = 0, SPELL = 1, ITEM = 2, MACRO = 3 };
    Type type = EMPTY;
    uint32_t id = 0;              // spellId, itemId, or macroId
    float cooldownRemaining = 0.0f;
    float cooldownTotal = 0.0f;

    bool isReady() const { return cooldownRemaining <= 0.0f; }
    bool isEmpty() const { return type == EMPTY; }
};

/**
 * Floating combat text entry
 */
struct CombatTextEntry {
    enum Type : uint8_t {
        MELEE_DAMAGE, SPELL_DAMAGE, HEAL, MISS, DODGE, PARRY, BLOCK,
        CRIT_DAMAGE, CRIT_HEAL, PERIODIC_DAMAGE, PERIODIC_HEAL, ENVIRONMENTAL
    };
    Type type;
    int32_t amount = 0;
    uint32_t spellId = 0;
    float age = 0.0f;           // Seconds since creation (for fadeout)
    bool isPlayerSource = false; // True if player dealt this

    static constexpr float LIFETIME = 2.5f;
    bool isExpired() const { return age >= LIFETIME; }
};

/**
 * Spell cooldown entry received from server
 */
struct SpellCooldownEntry {
    uint32_t spellId;
    uint16_t itemId;
    uint16_t categoryId;
    uint32_t cooldownMs;
    uint32_t categoryCooldownMs;
};

/**
 * Get human-readable spell cast failure reason (WoW 3.3.5a SpellCastResult)
 */
inline const char* getSpellCastResultString(uint8_t result, int powerType = -1) {
    // AzerothCore 3.3.5a SpellCastResult enum (SharedDefines.h)
    switch (result) {
        case 0:   return nullptr; // SUCCESS — not a failure
        case 1:   return "Affecting combat";
        case 2:   return "Already at full health";
        case 3:   return "Already at full mana";
        case 4:   return "Already at full power";
        case 5:   return "Already being tamed";
        case 6:   return "Already have charm";
        case 7:   return "Already have summon";
        case 8:   return "Already open";
        case 9:   return "Aura bounced";
        case 10:  return "Autotrack interrupted";
        case 11:  return "Bad implicit targets";
        case 12:  return "Invalid target";
        case 13:  return "Can't be charmed";
        case 14:  return "Can't be disenchanted";
        case 15:  return "Can't be disenchanted (skill)";
        case 16:  return "Can't be milled";
        case 17:  return "Can't be prospected";
        case 18:  return "Can't cast on tapped";
        case 19:  return "Can't duel while invisible";
        case 20:  return "Can't duel while stealthed";
        case 21:  return "Can't stealth";
        case 22:  return "Caster aurastate";
        case 23:  return "Caster dead";
        case 24:  return "Charmed";
        case 25:  return "Chest in use";
        case 26:  return "Confused";
        case 27:  return nullptr; // DONT_REPORT — suppress message
        case 28:  return "Equipped item required";
        case 29:  return "Equipped item class (mainhand)";
        case 30:  return "Equipped item class (mainhand)";
        case 31:  return "Equipped item class (offhand)";
        case 32:  return "Error";
        case 33:  return "Fizzle";
        case 34:  return "Fleeing";
        case 35:  return "Food too low level";
        case 36:  return "Target too high level";
        case 37:  return "Hunger satiated";
        case 38:  return "Immune";
        case 39:  return "Incorrect area";
        case 40:  return "Interrupted";
        case 41:  return "Interrupted (combat)";
        case 42:  return "Item already enchanted";
        case 43:  return "Item gone";
        case 44:  return "Item not found";
        case 45:  return "Item not ready";
        case 46:  return "Level requirement";
        case 47:  return "Target not in line of sight";
        case 48:  return "Target too low level";
        case 49:  return "Low cast level";
        case 50:  return "Mainhand empty";
        case 51:  return "Can't do that while moving";
        case 52:  return "Need ammo";
        case 53:  return "Need ammo pouch";
        case 54:  return "Need exotic ammo";
        case 55:  return "Need more items";
        case 56:  return "No path";
        case 57:  return "Not behind";
        case 58:  return "Not fishable";
        case 59:  return "Not flying";
        case 60:  return "Not here";
        case 61:  return "Target needs to be in front of you";
        case 62:  return "Not in control";
        case 63:  return "Not known";
        case 64:  return "Not mounted";
        case 65:  return "Not on taxi";
        case 66:  return "Not on transport";
        case 67:  return "Not ready";
        case 68:  return "Not in shapeshift form";
        case 69:  return "Not standing";
        case 70:  return "Not tradeable";
        case 71:  return "Not while trading";
        case 72:  return "Not unsheathed";
        case 73:  return "Not while ghost";
        case 74:  return "Not while looting";
        case 75:  return "No ammo";
        case 76:  return "No charges remain";
        case 77:  return "No champion";
        case 78:  return "Not enough combo points";
        case 79:  return "No dueling";
        case 80:  return "No endurance";
        case 81:  return "No fish";
        case 82:  return "No items while shapeshifted";
        case 83:  return "No mounts allowed here";
        case 84:  return "No pet";
        case 85:
            switch (powerType) {
                case 1:  return "Not enough rage";
                case 2:  return "Not enough focus";
                case 3:  return "Not enough energy";
                case 6:  return "Not enough runic power";
                default: return "Not enough mana";
            }
        case 86:  return "Nothing to dispel";
        case 87:  return "Nothing to steal";
        case 88:  return "Only above water";
        case 89:  return "Only daytime";
        case 90:  return "Only indoors";
        case 91:  return "Only mounted";
        case 92:  return "Only nighttime";
        case 93:  return "Only outdoors";
        case 94:  return "Only shapeshift";
        case 95:  return "Only stealthed";
        case 96:  return "Only underwater";
        case 97:  return "Out of range";
        case 98:  return "Pacified";
        case 99:  return "Possessed";
        case 100: return "Reagents required";
        case 101: return "Requires area";
        case 102: return "Requires spell focus";
        case 103: return "Can't do that while rooted";
        case 104: return "Can't do that while silenced";
        case 105: return "Spell in progress";
        case 106: return "Spell learned";
        case 107: return "Spell unavailable";
        case 108: return "Stunned";
        case 109: return "Targets dead";
        case 110: return "Target affecting combat";
        case 111: return "Target aurastate";
        case 112: return "Target dueling";
        case 113: return "Target is enemy";
        case 114: return "Target enraged";
        case 115: return "Target friendly";
        case 116: return "Target in combat";
        case 117: return "Target is player";
        case 118: return "Target is player controlled";
        case 119: return "Target not dead";
        case 120: return "Target not in party";
        case 121: return "Target not looted";
        case 122: return "Target not player";
        case 123: return "Target no pockets";
        case 124: return "Target no weapons";
        case 125: return "Target no ranged weapons";
        case 126: return "Target unskinnable";
        case 127: return "Thirst satiated";
        case 128: return "Too close";
        case 129: return "Too many of item";
        case 130: return "Totem category";
        case 131: return "Totems";
        case 132: return "Try again";
        case 133: return "Unit not behind";
        case 134: return "Unit not in front";
        case 135: return "Wrong pet food";
        case 136: return "Not while fatigued";
        case 137: return "Target not in instance";
        case 138: return "Not while trading";
        case 139: return "Target not in raid";
        case 140: return "Target free for all";
        case 141: return "No edible corpses";
        case 142: return "Only battlegrounds";
        case 143: return "Target not ghost";
        case 144: return "Transform unusable";
        case 145: return "Wrong weather";
        case 146: return "Damage immune";
        case 147: return "Prevented by mechanic";
        case 148: return "Play time restriction";
        case 149: return "Reputation required";
        case 150: return "Min skill required";
        case 151: return "Not in arena";
        case 152: return "Not on shapeshift";
        case 153: return "Not on stealthed";
        case 154: return "Not on damage immune";
        case 155: return "Not on mounted";
        case 156: return "Too shallow";
        case 157: return "Target not in sanctuary";
        case 158: return "Target is trivial";
        case 159: return "BM or invis god";
        case 160: return "Expert riding required";
        case 161: return "Artisan riding required";
        case 162: return "Not idle";
        case 163: return "Not inactive";
        case 164: return "Partial playtime";
        case 165: return "No playtime";
        case 166: return "Not in battleground";
        case 167: return "Not in raid instance";
        case 168: return "Only in arena";
        case 169: return "Target locked to raid instance";
        case 170: return "On use enchant";
        case 171: return "Not on ground";
        case 172: return "Custom error";
        case 173: return "Can't do that right now";
        case 174: return "Too many sockets";
        case 175: return "Invalid glyph";
        case 176: return "Unique glyph";
        case 177: return "Glyph socket locked";
        case 178: return "No valid targets";
        case 179: return "Item at max charges";
        case 180: return "Not in barbershop";
        case 181: return "Fishing too low";
        case 182: return "Item enchant trade window";
        case 183: return "Summon pending";
        case 184: return "Max sockets";
        case 185: return "Pet can rename";
        case 186: return "Target cannot be resurrected";
        case 187: return "Unknown error";
        case 255: return nullptr; // SPELL_CAST_OK
        default:  return nullptr;
    }
}

} // namespace game
} // namespace wowee
