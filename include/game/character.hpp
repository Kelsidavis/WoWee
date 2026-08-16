#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace game {

/**
 * Race IDs (WoW 3.3.5a)
 */
enum class Race : uint8_t {
    HUMAN = 1,
    ORC = 2,
    DWARF = 3,
    NIGHT_ELF = 4,
    UNDEAD = 5,
    TAUREN = 6,
    GNOME = 7,
    TROLL = 8,
    GOBLIN = 9,
    BLOOD_ELF = 10,
    DRAENEI = 11
};

/**
 * Class IDs (WoW 3.3.5a)
 */
enum class Class : uint8_t {
    WARRIOR = 1,
    PALADIN = 2,
    HUNTER = 3,
    ROGUE = 4,
    PRIEST = 5,
    DEATH_KNIGHT = 6,
    SHAMAN = 7,
    MAGE = 8,
    WARLOCK = 9,
    DRUID = 11
};

/**
 * Gender IDs
 */
enum class Gender : uint8_t {
    MALE = 0,
    FEMALE = 1,
    NONBINARY = 2
};

/**
 * Pronoun set for text substitution
 */
struct Pronouns {
    std::string subject;      // he/she/they
    std::string object;       // him/her/them
    std::string possessive;   // his/her/their
    std::string possessiveP;  // his/hers/theirs

    static Pronouns forGender(Gender gender) {
        switch (gender) {
            case Gender::MALE:
                return {.subject = "he", .object = "him", .possessive = "his", .possessiveP = "his"};
            case Gender::FEMALE:
                return {.subject = "she", .object = "her", .possessive = "her", .possessiveP = "hers"};
            case Gender::NONBINARY:
                return {.subject = "they", .object = "them", .possessive = "their", .possessiveP = "theirs"};
            default:
                return {.subject = "they", .object = "them", .possessive = "their", .possessiveP = "theirs"};
        }
    }
};

/**
 * Convert gender to server-compatible value (WoW 3.3.5a only supports binary genders)
 * Nonbinary is mapped to MALE for server communication while preserving client-side identity
 */
inline Gender toServerGender(Gender gender) {
    return (gender == Gender::FEMALE) ? Gender::FEMALE : Gender::MALE;
}

/**
 * Equipment item data
 */
struct EquipmentItem {
    uint32_t displayModel;      // Display model ID
    uint8_t inventoryType;      // Inventory slot type
    uint32_t enchantment;       // Enchantment/effect ID

    [[nodiscard]] bool isEmpty() const { return displayModel == 0; }
};

/**
 * Pet data (optional)
 */
struct PetData {
    uint32_t displayModel;      // Pet display model ID
    uint32_t level;             // Pet level
    uint32_t family;            // Pet family ID

    [[nodiscard]] bool exists() const { return displayModel != 0; }
};

/**
 * Complete character data from SMSG_CHAR_ENUM
 */
struct Character {
    // Identity
    uint64_t guid;              // Character GUID (unique identifier)
    std::string name;           // Character name

    // Basics
    Race race;                  // Character race
    Class characterClass;       // Character class (renamed from 'class' keyword)
    Gender gender;              // Character gender
    bool useFemaleModel = false; // For nonbinary: body type preference (client-side only)
    uint8_t level;              // Character level (1-80)

    // Appearance
    uint32_t appearanceBytes;   // Custom appearance (skin, hair color, hair style, face)
    uint8_t facialFeatures;     // Facial features

    // Location
    uint32_t zoneId;            // Current zone ID
    uint32_t mapId;             // Current map ID
    float x;                    // X coordinate
    float y;                    // Y coordinate
    float z;                    // Z coordinate

    // Affiliations
    uint32_t guildId;           // Guild ID (0 if no guild)

    // State
    uint32_t flags;             // Character flags (PvP, dead, etc.)

    // Optional data
    PetData pet;                                // Pet information (if exists)
    std::vector<EquipmentItem> equipment;       // Equipment (23 slots)

    // Helper methods
    [[nodiscard]] bool hasGuild() const { return guildId != 0; }
    [[nodiscard]] bool hasPet() const { return pet.exists(); }
};

// Race/class combo and appearance range validation (WoW 3.3.5a)
bool isValidRaceClassCombo(Race race, Class cls);
uint8_t getMaxSkin(Race race, Gender gender);
uint8_t getMaxFace(Race race, Gender gender);
uint8_t getMaxHairStyle(Race race, Gender gender);
uint8_t getMaxHairColor(Race race, Gender gender);
uint8_t getMaxFacialFeature(Race race, Gender gender);

/**
 * Get human-readable race name
 */
const char* getRaceName(Race race);

/**
 * Get human-readable class name
 */
const char* getClassName(Class characterClass);

/**
 * Get human-readable gender name
 */
const char* getGenderName(Gender gender);

/**
 * Get M2 model path for a given race and gender
 * useFemaleModel allows nonbinary characters to choose body type
 */
std::string getPlayerModelPath(Race race, Gender gender, bool useFemaleModel = false);

/**
 * Get M2 model path for a character (uses character's body type preference)
 */
std::string getPlayerModelPath(const Character& character);

/// Whether this client has a character model for that race at all.
///
/// getPlayerModelPath answers HumanMale for anything it does not know, which
/// is the right thing for a player - every playable race is in it, and a
/// silhouette beats nothing. It is the wrong thing for a creature display:
/// CreatureDisplayInfoExtra gives naga, broken, skeletons and a dozen other
/// NPC-only races the same skin-and-face columns a character has, and drawing
/// one of those as a human male puts a human in the target frame where a naga
/// is standing. Those have a creature model of their own, and it is better to
/// go and load it.
bool hasPlayerModel(Race race);

} // namespace game
} // namespace wowee
