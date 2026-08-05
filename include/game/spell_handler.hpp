#pragma once

#include "game/world_packets.hpp"
#include "game/opcode_table.hpp"
#include "game/spell_defines.hpp"
#include "game/handler_types.hpp"
#include "audio/spell_sound_manager.hpp"
#include "network/packet.hpp"
#include <glm/glm.hpp>
#include <array>
#include <chrono>
#include <functional>
#include <map>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace wowee {
namespace game {

class GameHandler;

class SpellHandler {
public:
    using PacketHandler = std::function<void(network::Packet&)>;
    using DispatchTable = std::unordered_map<LogicalOpcode, PacketHandler>;

    explicit SpellHandler(GameHandler& owner);

    void registerOpcodes(DispatchTable& table);

    // Talent data structures (aliased from handler_types.hpp)
    using TalentEntry = game::TalentEntry;
    using TalentTabEntry = game::TalentTabEntry;

    // --- Spell book tabs ---
    struct SpellBookTab {
        std::string name;
        std::string texture; // icon path
        std::vector<uint32_t> spellIds; // spells in this tab
    };

    // Unit cast state (aliased from handler_types.hpp)
    using UnitCastState = game::UnitCastState;

    // Equipment set info (aliased from handler_types.hpp)
    using EquipmentSetInfo = game::EquipmentSetInfo;

    // --- Public API (delegated from GameHandler) ---
    void castSpell(uint32_t spellId, uint64_t targetGuid = 0);

    /// Spell.dbc EffectImplicitTargetA, or 0 when the spell is unknown. 21 means
    /// the spell has to be aimed at a friendly unit.
    uint32_t getSpellImplicitTargetA(uint32_t spellId) const;

    /// The last spell the player cast while on foot. When mounting is detected,
    /// this identifies which of the player's indefinite self-cast auras is the
    /// mount — scanning for one blindly can land on a racial or a tracking buff.
    uint32_t getLastGroundCastSpellId() const { return lastGroundCastSpellId_; }

    /// Record a spell cast by using an item — pre-WotLK mounts are items, and
    /// their on-use spell never passes through castSpell().
    void noteGroundCastSpell(uint32_t spellId) { lastGroundCastSpellId_ = spellId; }
    void cancelCast();
    void cancelAura(uint32_t spellId);

    // Known spells
    const std::unordered_set<uint32_t>& getKnownSpells() const { return knownSpells_; }
    const std::unordered_map<uint32_t, float>& getSpellCooldowns() const { return spellCooldowns_; }
    float getSpellCooldown(uint32_t spellId) const;

    // Cast state
    bool isCasting() const { return casting_ || restorationActive_; }
    bool isChanneling() const { return casting_ ? castIsChannel_ : restorationActive_; }
    bool isRestoring() const { return restorationActive_; }
    bool isGameObjectInteractionCasting() const;
    uint32_t getCurrentCastSpellId() const {
        return casting_ ? currentCastSpellId_ : restorationSpellId_;
    }
    float getCastProgress() const {
        const float total = casting_ ? castTimeTotal_ : restorationTimeTotal_;
        const float remaining = casting_ ? castTimeRemaining_ : restorationTimeRemaining_;
        return total > 0.0f ? (total - remaining) / total : 0.0f;
    }
    float getCastTimeRemaining() const {
        return casting_ ? castTimeRemaining_ : restorationTimeRemaining_;
    }
    float getCastTimeTotal() const {
        return casting_ ? castTimeTotal_ : restorationTimeTotal_;
    }

    // Repeat-craft queue
    void startCraftQueue(uint32_t spellId, int count);
    void cancelCraftQueue();
    int getCraftQueueRemaining() const { return craftQueueRemaining_; }
    uint32_t getCraftQueueSpellId() const { return craftQueueSpellId_; }

    // Crafting window (client-side; opened by casting a profession spell
    // like Cooking or First Aid — see tradeskillOpenerSkillLine)
    bool isCraftingWindowOpen() const { return craftingWindowOpen_; }
    uint32_t getCraftingSkillLine() const { return craftingSkillLine_; }
    void openCraftingWindow(uint32_t skillLine) { craftingWindowOpen_ = true; craftingSkillLine_ = skillLine; }
    void closeCraftingWindow() { craftingWindowOpen_ = false; }
    // Returns the skill line id if spellId is a tradeskill-window opener
    // (e.g. Cooking → 185) with at least one known recipe, else 0.
    uint32_t tradeskillOpenerSkillLine(uint32_t spellId);

    // SpellFocusObject.dbc name ("Anvil", "Cooking Fire", ...) for
    // requires-spell-focus cast failures; empty if unknown.
    const std::string& getSpellFocusName(uint32_t focusId);

    // TotemCategory.dbc name ("Blacksmith Hammer", "Mining Pick", ...) for
    // totem-category cast failures; empty if unknown.
    const std::string& getTotemCategoryName(uint32_t categoryId);

    // Spell queue (400ms window)
    uint32_t getQueuedSpellId() const { return queuedSpellId_; }
    void cancelQueuedSpell() { queuedSpellId_ = 0; queuedSpellTarget_ = 0; }

    // Unit cast state (tracked per GUID for target frame + boss frames)
    const UnitCastState* getUnitCastState(uint64_t guid) const {
        auto it = unitCastStates_.find(guid);
        return (it != unitCastStates_.end() && it->second.casting) ? &it->second : nullptr;
    }
    void clearUnitCastStates() { unitCastStates_.clear(); }
    void removeUnitCastState(uint64_t guid) { unitCastStates_.erase(guid); }

    // Aura cache mutation (formerly accessed via friend)
    void clearUnitAurasCache() { unitAurasCache_.clear(); }
    void removeUnitAuraCache(uint64_t guid) { unitAurasCache_.erase(guid); }

    // Known spells mutation (formerly accessed via friend)
    void addKnownSpell(uint32_t spellId) { knownSpells_.insert(spellId); }
    bool hasKnownSpell(uint32_t spellId) const { return knownSpells_.count(spellId) > 0; }

    // Target aura mutation (formerly accessed via friend)
    void clearTargetAuras() { for (auto& slot : targetAuras_) slot = AuraSlot{}; }

    // Player aura mutation (formerly accessed via friend)
    void resetPlayerAuras(size_t capacity) { playerAuras_.clear(); playerAuras_.resize(capacity); }
    AuraSlot& getPlayerAuraSlotRef(size_t slot) { return playerAuras_[slot]; }
    std::vector<AuraSlot>& getPlayerAurasMut() { return playerAuras_; }

    // Target cast helpers
    bool isTargetCasting() const;
    uint32_t getTargetCastSpellId() const;
    float getTargetCastProgress() const;
    float getTargetCastTimeRemaining() const;
    bool isTargetCastInterruptible() const;

    // Talents
    uint8_t getActiveTalentSpec() const { return activeTalentSpec_; }
    uint8_t getUnspentTalentPoints() const { return unspentTalentPoints_[activeTalentSpec_]; }
    uint8_t getUnspentTalentPoints(uint8_t spec) const { return spec < 2 ? unspentTalentPoints_[spec] : 0; }
    const std::unordered_map<uint32_t, uint8_t>& getLearnedTalents() const { return learnedTalents_[activeTalentSpec_]; }
    const std::unordered_map<uint32_t, uint8_t>& getLearnedTalents(uint8_t spec) const {
        static std::unordered_map<uint32_t, uint8_t> empty;
        return spec < 2 ? learnedTalents_[spec] : empty;
    }

    static constexpr uint8_t MAX_GLYPH_SLOTS = 6;
    const std::array<uint16_t, MAX_GLYPH_SLOTS>& getGlyphs() const { return learnedGlyphs_[activeTalentSpec_]; }
    const std::array<uint16_t, MAX_GLYPH_SLOTS>& getGlyphs(uint8_t spec) const {
        static std::array<uint16_t, MAX_GLYPH_SLOTS> empty{};
        return spec < 2 ? learnedGlyphs_[spec] : empty;
    }
    uint8_t getTalentRank(uint32_t talentId) const {
        auto it = learnedTalents_[activeTalentSpec_].find(talentId);
        return (it != learnedTalents_[activeTalentSpec_].end()) ? it->second : 0;
    }
    void learnTalent(uint32_t talentId, uint32_t requestedRank);
    void switchTalentSpec(uint8_t newSpec);

    // Talent DBC access
    const TalentEntry* getTalentEntry(uint32_t talentId) const {
        auto it = talentCache_.find(talentId);
        return (it != talentCache_.end()) ? &it->second : nullptr;
    }
    const TalentTabEntry* getTalentTabEntry(uint32_t tabId) const {
        auto it = talentTabCache_.find(tabId);
        return (it != talentTabCache_.end()) ? &it->second : nullptr;
    }
    const std::unordered_map<uint32_t, TalentEntry>& getAllTalents() const { return talentCache_; }
    const std::unordered_map<uint32_t, TalentTabEntry>& getAllTalentTabs() const { return talentTabCache_; }
    void loadTalentDbc();
    void syncPreWotlkTalentsFromKnownSpells();

    // Auras
    const std::vector<AuraSlot>& getPlayerAuras() const { return playerAuras_; }
    const std::vector<AuraSlot>& getTargetAuras() const { return targetAuras_; }
    const std::vector<AuraSlot>* getUnitAuras(uint64_t guid) const {
        auto it = unitAurasCache_.find(guid);
        return (it != unitAurasCache_.end()) ? &it->second : nullptr;
    }

    // Global Cooldown (GCD)
    float getGCDRemaining() const {
        if (gcdTotal_ <= 0.0f) return 0.0f;
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - gcdStartedAt_).count() / 1000.0f;
        float rem = gcdTotal_ - elapsed;
        return rem > 0.0f ? rem : 0.0f;
    }
    float getGCDTotal() const { return gcdTotal_; }
    bool isGCDActive() const { return getGCDRemaining() > 0.0f; }

    // Spell book tabs
    const std::vector<SpellBookTab>& getSpellBookTabs();

    // Talent wipe confirm dialog
    bool showTalentWipeConfirmDialog() const { return talentWipePending_; }
    uint32_t getTalentWipeCost() const { return talentWipeCost_; }
    void confirmTalentWipe();
    void cancelTalentWipe() { talentWipePending_ = false; }

    // Pet talent respec confirm
    bool showPetUnlearnDialog() const { return petUnlearnPending_; }
    uint32_t getPetUnlearnCost() const { return petUnlearnCost_; }
    void confirmPetUnlearn();
    void cancelPetUnlearn() { petUnlearnPending_ = false; }

    // Item use
    void useItemBySlot(int backpackIndex);
    void useItemInBag(int bagIndex, int slotIndex);
    void useItemById(uint32_t itemId);

    // Equipment sets — canonical data owned by InventoryHandler;
    // GameHandler::getEquipmentSets() delegates to inventoryHandler_.

    // Pet spells
    void sendPetAction(uint32_t action, uint64_t targetGuid = 0);
    void dismissPet();
    void togglePetSpellAutocast(uint32_t spellId);
    void renamePet(const std::string& newName);

    // Spell DBC accessors
    const int32_t* getSpellEffectBasePoints(uint32_t spellId) const;
    float getSpellDuration(uint32_t spellId) const;
    const std::string& getSpellName(uint32_t spellId) const;
    const std::string& getSpellRank(uint32_t spellId) const;
    const std::string& getSpellDescription(uint32_t spellId) const;
    std::string getEnchantName(uint32_t enchantId) const;
    uint8_t getSpellDispelType(uint32_t spellId) const;
    bool isSpellInterruptible(uint32_t spellId) const;
    uint32_t getSpellSchoolMask(uint32_t spellId) const;
    /// Spell.dbc Targets mask (SpellCastTargetFlags): 0x10 = TARGET_FLAG_ITEM.
    uint32_t getSpellTargetFlags(uint32_t spellId) const;
    uint32_t getSpellTargetAuraState(uint32_t spellId) const;
    /// Spell.dbc RangeIndex resolved via SpellRange.dbc, in yards. Melee ("Combat
    /// Range") is 5; self-only is 0; negative means SpellRange.dbc was unavailable.
    float getSpellMaxRange(uint32_t spellId) const;
    /// True for "Self Only" range spells (shouts, self-buffs): they always land on
    /// the caster, so they take no explicit target and skip melee range checks.
    bool isSelfCastSpell(uint32_t spellId) const;
    /// Maps a superseded spell rank to the highest rank we actually know. Returns
    /// spellId unchanged when it is already known, or has no known same-name rank.
    uint32_t resolveHighestKnownRank(uint32_t spellId) const;
    const std::string& getSkillLineName(uint32_t spellId) const;

    // Cast state
    void stopCasting();
    void resetCastState();
    void resetTalentState();
    // Full per-character reset (spells, cooldowns, auras, cast state, talents).
    // Called from GameHandler::selectCharacter so spell state doesn't bleed between characters.
    void resetAllState();
    void clearUnitCaches();

    // Aura duration
    void handleUpdateAuraDuration(uint8_t slot, uint32_t durationMs);

    // Skill DBC
    void loadSkillLineDbc();
    void extractSkillFields(const FlatFieldMap& fields);
    void extractExploredZoneFields(const FlatFieldMap& fields);

    // Update per-frame timers (call from GameHandler::update)
    void updateTimers(float dt);
    void refreshRestorationState() { refreshRestorationFromPlayerAuras(); }

    // Packet handlers dispatched from GameHandler's opcode table
    void handlePetSpells(network::Packet& packet);
    void handleListStabledPets(network::Packet& packet);

    // Pet stable commands (called via GameHandler delegation)
    void requestStabledPetList();
    void stablePet(uint8_t slot);
    void unstablePet(uint32_t petNumber);

    // DBC cache loading (called from GameHandler during login)
    void loadSpellNameCache() const;
    void loadSkillLineAbilityDbc();
    void categorizeTrainerSpells();

    /// Ask the server what the pet is called.
    ///
    /// The name a player gave a pet arrives only in answer to
    /// CMSG_PET_NAME_QUERY. Without it a pet wears its creature template's
    /// name — "Voidwalker" where the player wrote something else — which is
    /// what the pet frame, its nameplate and the pet bar's tooltip all show.
    ///
    /// The pet number in that request is a key the server echoes back and does
    /// not look anything up with: SendPetNameQuery finds the pet by guid. So
    /// this sends a number of its own making and uses it to match the reply to
    /// the pet it asked about, which is what the field would have been for.
    void requestPetName(uint64_t petGuid);

private:
    std::unordered_map<uint32_t, uint64_t> pendingPetNameQueries_;
    uint32_t nextPetNameQueryKey_ = 1;

    // --- Packet handlers ---
    void handleInitialSpells(network::Packet& packet);
    void handleCastFailed(network::Packet& packet);
    void handleSpellStart(network::Packet& packet);
    void handleSpellGo(network::Packet& packet);
    void handleSpellCooldown(network::Packet& packet);
    void handleCooldownEvent(network::Packet& packet);
    void handleAuraUpdate(network::Packet& packet, bool isAll);
    void handleLearnedSpell(network::Packet& packet);

    void handleCastResult(network::Packet& packet);
    void handleSpellFailedOther(network::Packet& packet);
    void handleClearCooldown(network::Packet& packet);
    void handleModifyCooldown(network::Packet& packet);
    void handlePlaySpellVisual(network::Packet& packet);
    void handleSpellModifier(network::Packet& packet, bool isFlat);
    void handleSpellDelayed(network::Packet& packet);
    void handleSpellLogMiss(network::Packet& packet);
    void handleSpellFailure(network::Packet& packet);
    void handleItemCooldown(network::Packet& packet);
    void handleDispelFailed(network::Packet& packet);
    void handleTotemCreated(network::Packet& packet);
    void handlePeriodicAuraLog(network::Packet& packet);
    void handleSpellEnergizeLog(network::Packet& packet);
    void handleExtraAuraInfo(network::Packet& packet, bool isInit);
    void handleSpellDispelLog(network::Packet& packet);
    void handleSpellStealLog(network::Packet& packet);
    void handleSpellChanceProcLog(network::Packet& packet);
    void handleSpellInstaKillLog(network::Packet& packet);
    void handleSpellLogExecute(network::Packet& packet);
    void handleClearExtraAuraInfo(network::Packet& packet);
    void handleItemEnchantTimeUpdate(network::Packet& packet);
    void handleResumeCastBar(network::Packet& packet);
    void handleChannelStart(network::Packet& packet);
    void handleChannelUpdate(network::Packet& packet);

    // --- Internal helpers ---

    // Resolve the magic school for a spell (for audio playback).
    // Returns MagicSchool from the spell name cache, defaulting to ARCANE.
    audio::SpellSoundManager::MagicSchool resolveSpellSchool(uint32_t spellId);

    // Play a spell cast or impact sound via audioCoordinator, if available.
    void playSpellCastSound(uint32_t spellId);
    void playSpellImpactSound(uint32_t spellId);

    // Resolve SpellVisualID from Spell.dbc cache for a given spellId.
    uint32_t resolveSpellVisualId(uint32_t spellId);
    // Resolve render-space position for a unit GUID (player or entity).
    bool resolveUnitPosition(uint64_t guid, glm::vec3& outPos);
    // Play the cast/precast visual effect at the caster's position.
    void triggerCastVisual(uint32_t spellId, uint64_t casterGuid, uint32_t castTimeMs = 0);
    // Play the impact visual effect at the target's position.
    void triggerImpactVisual(uint32_t spellId, uint64_t targetGuid);
    void launchRangedWeaponProjectile(uint32_t spellId, uint64_t targetGuid);
    void refreshRestorationFromPlayerAuras();
    void stopRestorationPresentation();

    // --- handleSpellLogExecute per-effect parsers (extracted to reduce nesting) ---
    void parseEffectPowerDrain(network::Packet& packet, uint32_t effectLogCount,
                               uint64_t caster, uint32_t spellId, bool isPlayerCaster,
                               bool usesFullGuid);
    void parseEffectHealthLeech(network::Packet& packet, uint32_t effectLogCount,
                                uint64_t caster, uint32_t spellId, bool isPlayerCaster,
                                bool usesFullGuid);
    void parseEffectCreateItem(network::Packet& packet, uint32_t effectLogCount,
                               uint64_t caster, uint32_t spellId, bool isPlayerCaster);
    void parseEffectInterruptCast(network::Packet& packet, uint32_t effectLogCount,
                                  uint64_t caster, uint32_t spellId, bool isPlayerCaster,
                                  bool usesFullGuid);
    void parseEffectFeedPet(network::Packet& packet, uint32_t effectLogCount,
                            uint64_t caster, uint32_t spellId, bool isPlayerCaster);

    // Find the on-use spell for an item (trigger=0 Use or trigger=5 NoDelay).
    // CMSG_USE_ITEM requires a valid spellId or the server silently ignores it.
    uint32_t findOnUseSpellId(uint32_t itemId) const;
    void seedCooldownFromSpellInfo(uint32_t spellId);
    void handleSupercededSpell(network::Packet& packet);
    void handleRemovedSpell(network::Packet& packet);
    void handleUnlearnSpells(network::Packet& packet);
    void handleTalentsInfo(network::Packet& packet);
    void handleAchievementEarned(network::Packet& packet);

    GameHandler& owner_;

    // --- Spell state ---
    std::unordered_set<uint32_t> knownSpells_;
    std::unordered_map<uint32_t, float> spellCooldowns_;    // spellId -> remaining seconds
    uint8_t castCount_ = 0;
    bool casting_ = false;
    bool castIsChannel_ = false;
    uint32_t currentCastSpellId_ = 0;
    float castTimeRemaining_ = 0.0f;
    float castTimeTotal_ = 0.0f;
    bool restorationActive_ = false;
    uint32_t restorationSpellId_ = 0;
    bool restorationIsFood_ = false;
    float restorationTimeRemaining_ = 0.0f;
    float restorationTimeTotal_ = 0.0f;
    float restorationSoundTimer_ = 0.0f; // repeats the consume sound while active

    // Repeat-craft queue
    uint32_t craftQueueSpellId_ = 0;
    int craftQueueRemaining_ = 0;

    // Crafting window
    bool craftingWindowOpen_ = false;
    uint32_t craftingSkillLine_ = 0;

    // SpellFocusObject.dbc names, loaded lazily
    std::unordered_map<uint32_t, std::string> spellFocusNames_;
    bool spellFocusDbcLoaded_ = false;

    // TotemCategory.dbc names, loaded lazily
    std::unordered_map<uint32_t, std::string> totemCategoryNames_;
    bool totemCategoryDbcLoaded_ = false;

    // Spell queue (400ms window)
    uint32_t lastGroundCastSpellId_ = 0;
    uint32_t queuedSpellId_ = 0;
    uint64_t queuedSpellTarget_ = 0;

    // Per-unit cast state
    std::unordered_map<uint64_t, UnitCastState> unitCastStates_;

    // Talents (dual-spec support)
    uint8_t activeTalentSpec_ = 0;
    uint8_t unspentTalentPoints_[2] = {0, 0};
    std::unordered_map<uint32_t, uint8_t> learnedTalents_[2];
    std::array<std::array<uint16_t, MAX_GLYPH_SLOTS>, 2> learnedGlyphs_{};
    std::unordered_map<uint32_t, TalentEntry> talentCache_;
    std::unordered_map<uint32_t, TalentTabEntry> talentTabCache_;
    bool talentDbcLoaded_ = false;
    bool talentsInitialized_ = false;

    // Auras
    std::vector<AuraSlot> playerAuras_;
    std::vector<AuraSlot> targetAuras_;
    std::unordered_map<uint64_t, std::vector<AuraSlot>> unitAurasCache_;

    // Global Cooldown
    float gcdTotal_ = 0.0f;
    std::chrono::steady_clock::time_point gcdStartedAt_{};

    // Spell book tabs
    std::vector<SpellBookTab> spellBookTabs_;
    size_t lastSpellCount_ = 0;
    bool spellBookTabsDirty_ = true;

    // Talent wipe confirm dialog
    bool talentWipePending_ = false;
    uint64_t talentWipeNpcGuid_ = 0;
    uint32_t talentWipeCost_ = 0;

    // Pet talent respec confirm dialog
    bool petUnlearnPending_ = false;
    uint64_t petUnlearnGuid_ = 0;
    uint32_t petUnlearnCost_ = 0;
};

} // namespace game
} // namespace wowee
