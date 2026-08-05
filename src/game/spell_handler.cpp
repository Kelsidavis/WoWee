#include "game/spell_handler.hpp"
#include "game/spell_classification.hpp"
#include "game/pet_action.hpp"
#include "game/game_handler.hpp"
#include "game/game_utils.hpp"
#include "game/packet_parsers.hpp"
#include "game/entity.hpp"
#include "rendering/renderer.hpp"
#include "rendering/camera_controller.hpp"
#include "rendering/character_renderer.hpp"
#include "rendering/spell_visual_system.hpp"
#include "audio/audio_coordinator.hpp"
#include "audio/spell_sound_manager.hpp"
#include "audio/combat_sound_manager.hpp"
#include "core/application.hpp"
#include "core/coordinates.hpp"
#include "core/logger.hpp"
#include "network/world_socket.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_loader.hpp"
#include "pipeline/dbc_layout.hpp"
#include "audio/ui_sound_manager.hpp"
#include "audio/player_voice_manager.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>

namespace wowee {
namespace game {

// Merge incoming cooldown with local remaining time — keeps local timer when
// a stale/duplicate packet arrives after local countdown has progressed.
static float mergeCooldownSeconds(float current, float incoming) {
    constexpr float kEpsilon = 0.05f;
    if (incoming <= 0.0f) return 0.0f;
    if (current <= 0.0f) return incoming;
    if (incoming > current + kEpsilon) return current;
    return incoming;
}

namespace {
constexpr uint32_t kItemClassConsumable = 0;
constexpr uint32_t kConsumableSubclassBandage = 7;
constexpr uint32_t kConsumableSubclassItemEnhancement = 6;
constexpr uint8_t kSpellFailedNotReady = 67;
constexpr uint8_t kSpellFailedAlreadyOpen = 8;
constexpr uint8_t kSpellFailedChestInUse = 25;
constexpr uint8_t kSpellFailedTryAgain = 132;

bool isBandageItem(const ItemQueryResponseData* info) {
    return info && info->valid &&
           info->itemClass == kItemClassConsumable &&
           info->subClass == kConsumableSubclassBandage;
}

bool isBandageSpell(const GameHandler& owner, uint32_t spellId) {
    if (spellId == 0) return false;
    for (const auto& [itemId, info] : owner.getItemInfoCache()) {
        (void)itemId;
        if (!isBandageItem(&info)) continue;
        for (const auto& itemSpell : info.spells) {
            if (itemSpell.spellId == spellId) return true;
        }
    }
    return false;
}

std::string castFailureMessage(const GameHandler& owner, uint32_t spellId,
                               uint8_t result, int powerType, uint32_t miscArg = 0,
                               uint32_t miscArg2 = 0) {
    if (spellclass::isFishingCast(spellId)) {
        if (result == 11 || result == 60 || result == 178)
            return "Face open water and try casting again.";
        if (result == 58)
            return "You can't fish in this area.";
        if (result == 156)
            return "The water there is too shallow.";
    }

    // Bandages use a hidden target aura to enforce the Recently Bandaged
    // lockout. Exposing the protocol label ("Target aurastate") gives the
    // player no actionable information.
    if (isBandageSpell(owner, spellId)) {
        if (result == 111)
            return "Cannot use another bandage while Recently Bandaged is active.";
        if (result == 40 || result == 41)
            return "Bandaging was interrupted. Remain still until it finishes.";
    }

    // "Requires spell focus" means a crafting station object must be nearby.
    // The packet names which one via its SpellFocusObject id — surface it.
    if (result == kCastResultRequiresSpellFocus) {
        if (miscArg != 0) {
            const std::string& focusName = owner.getSpellFocusName(miscArg);
            if (!focusName.empty())
                return "Requires " + focusName + " nearby.";
        }
        return "You must be near the right crafting station (forge, anvil, cooking fire, ...).";
    }

    // "Totems" / "Totem category" mean a required crafting tool is missing
    // (blacksmith hammer, mining pick, ...). Name it from the packet's ids:
    // TotemCategory.dbc entries for totem-category, item ids for totems.
    if (result == kCastResultTotemCategory || result == kCastResultTotems) {
        std::string tools;
        for (uint32_t id : {miscArg, miscArg2}) {
            if (id == 0) continue;
            std::string name;
            if (result == kCastResultTotemCategory) {
                name = owner.getTotemCategoryName(id);
            } else if (const auto* info = owner.getItemInfo(id); info && info->valid) {
                name = info->name;
            }
            if (name.empty()) continue;
            if (!tools.empty()) tools += " and ";
            tools += name;
        }
        if (!tools.empty()) return "Requires " + tools + " in your inventory.";
        return "Requires a crafting tool you don't have (blacksmith hammer, mining pick, ...).";
    }

    // "Target aurastate" (111) means the target isn't in the state the ability needs.
    // Translate the spell's required Spell.dbc TargetAuraState into actionable text —
    // Execute and Hammer of Wrath want a low-health target, etc.
    if (result == 111) {
        switch (owner.getSpellTargetAuraState(spellId)) {
            case 2:  return "Target must be below 20% health.";
            case 13: return "Target must be below 35% health.";
            case 14: return "Target must be affected by Immolate or Shadowflame.";
            case 16: return "Target must be afflicted by your Deadly Poison.";
            case 17: return "Target must be enraged.";
            case 18: return "Target must be bleeding.";
            default: return "The target isn't in the required state for this ability.";
        }
    }

    const char* reason = getSpellCastResultString(result, powerType);
    return reason ? reason
                  : ("Spell cast failed (error " + std::to_string(result) + ")");
}

uint64_t targetGuidForUseItem(GameHandler& owner, const ItemQueryResponseData* info) {
    if (!info || !info->valid || info->itemClass != kItemClassConsumable) return 0;
    if (isBandageItem(info)) {
        return owner.getPlayerGuid();
    }
    if (info->subClass == kConsumableSubclassItemEnhancement) return 0;
    return owner.getPlayerGuid();
}

bool isGatherSpellId(uint32_t spellId) {
    static constexpr uint32_t kGatherRanks[] = {
        2575, 2576, 3564, 10248, 29354, // Mining
        2366, 2368, 3570, 11993, 28695  // Herbalism
    };
    for (uint32_t rankSpellId : kGatherRanks) {
        if (spellId == rankSpellId) return true;
    }
    return false;
}

bool shouldDespawnGatherTarget(uint8_t result) {
    return result == kSpellFailedAlreadyOpen || result == kSpellFailedChestInUse;
}

bool isMiningGatherSpell(uint32_t spellId) {
    static constexpr uint32_t kMiningRanks[] = {2575, 2576, 3564, 10248, 29354};
    for (uint32_t rank : kMiningRanks) {
        if (spellId == rank) return true;
    }
    return false;
}

uint32_t gatherRequiredSkillRank(GameHandler& owner, uint64_t goGuid) {
    auto entity = owner.getEntityManager().getEntity(goGuid);
    if (!entity || entity->getType() != ObjectType::GAMEOBJECT) return 0;
    auto go = std::static_pointer_cast<GameObject>(entity);
    const auto* info = owner.getCachedGameObjectInfo(go->getEntry());
    if (!info || !info->hasData || info->data[0] == 0) return 0;

    auto* assets = owner.services().assetManager;
    auto lockDbc = assets ? assets->loadDBC("Lock.dbc") : nullptr;
    if (!lockDbc || !lockDbc->isLoaded() || lockDbc->getFieldCount() < 33) return 0;

    const uint32_t lockId = info->data[0];
    for (uint32_t row = 0; row < lockDbc->getRecordCount(); ++row) {
        if (lockDbc->getUInt32(row, 0) != lockId) continue;
        // Lock.dbc: Type[8], Index[8], Skill[8], Action[8]. Resource
        // nodes have a LOCK_KEY_SKILL entry whose Skill value is the exact
        // Mining/Herbalism rank enforced by the server.
        for (uint32_t slot = 0; slot < 8; ++slot) {
            constexpr uint32_t kLockKeySkill = 2;
            if (lockDbc->getUInt32(row, 1 + slot) != kLockKeySkill) continue;
            const uint32_t required = lockDbc->getUInt32(row, 17 + slot);
            if (required != 0) return required;
        }
        break;
    }
    return 0;
}

std::string gatherCastFailureMessage(GameHandler& owner, uint64_t goGuid,
                                     uint32_t spellId, uint8_t result,
                                     const std::string& fallback) {
    if (result == kSpellFailedTryAgain) return "Failed.";
    if (result == kSpellFailedChestInUse) return "Already in use.";
    // LOW_CASTLEVEL is the server's generic wording for an insufficient
    // gathering profession rank. MIN_SKILL_REQUIRED is used by some cores.
    if (result == 49 || result == 150) {
        const char* skill = isMiningGatherSpell(spellId) ? "Mining" : "Herbalism";
        const uint32_t required = gatherRequiredSkillRank(owner, goGuid);
        if (required != 0) {
            return std::string("Requires ") + skill + " skill " +
                   std::to_string(required) + ".";
        }
        return std::string("Your ") + skill + " skill is too low for this node.";
    }
    return fallback;
}

// Map a (WotLK-normalized) SpellCastResult to a character speech response.
// Results without a matching voice line return nullopt.
std::optional<audio::PlayerErrorSpeech> errorSpeechForCastResult(
        uint32_t spellId, uint8_t result, int powerType) {
    using audio::PlayerErrorSpeech;
    // Fishing has no unit target: the server validates a randomly sampled water
    // point in front of the caster. The generic "no target" voice line is both
    // misleading and contradicted by the on-screen fishing-specific guidance.
    if (spellclass::isFishingCast(spellId) && (result == 11 || result == 60 || result == 156 || result == 178))
        return std::nullopt;
    switch (result) {
        case 11:  // Bad implicit targets
        case 178: // No valid targets
            return PlayerErrorSpeech::GENERIC_NO_TARGET;
        case 12:  // Invalid target
            return PlayerErrorSpeech::INVALID_ATTACK_TARGET;
        case 25:  // Chest in use
            return PlayerErrorSpeech::CHEST_IN_USE;
        case 45:  // Item not ready
            return PlayerErrorSpeech::ITEM_COOLDOWN;
        case 52:  // Need ammo
        case 53:  // Need ammo pouch
        case 54:  // Need exotic ammo
        case 75:  // No ammo
            return PlayerErrorSpeech::NO_AMMO;
        case 67:  // Not ready
            // Ranged weapon attacks are abilities, not conventional spells. If the
            // server rejects one (weapon/ammo/attack state), use the generic "I can't
            // do that yet" response instead of "That spell isn't ready yet."
            return spellclass::isRangedWeaponAutoAttack(spellId)
                ? PlayerErrorSpeech::ABILITY_COOLDOWN
                : PlayerErrorSpeech::SPELL_COOLDOWN;
        case 85:  // Not enough power
            switch (powerType) {
                case 1:  return PlayerErrorSpeech::NO_RAGE;
                case 3:  return PlayerErrorSpeech::NO_ENERGY;
                case 2: case 6: return std::nullopt; // no focus/runic power voice lines
                default: return PlayerErrorSpeech::NO_MANA;
            }
        case 97:  // Out of range
            return PlayerErrorSpeech::OUT_OF_RANGE;
        default:
            return std::nullopt;
    }
}
} // namespace

static CombatTextEntry::Type combatTextTypeFromSpellMissInfo(uint8_t missInfo) {
    switch (missInfo) {
        case SpellMissInfo::MISS:    return CombatTextEntry::MISS;
        case SpellMissInfo::DODGE:   return CombatTextEntry::DODGE;
        case SpellMissInfo::PARRY:   return CombatTextEntry::PARRY;
        case SpellMissInfo::BLOCK:   return CombatTextEntry::BLOCK;
        case SpellMissInfo::EVADE:   return CombatTextEntry::EVADE;
        case SpellMissInfo::IMMUNE:  return CombatTextEntry::IMMUNE;
        case SpellMissInfo::DEFLECT: return CombatTextEntry::DEFLECT;
        case SpellMissInfo::ABSORB:  return CombatTextEntry::ABSORB;
        case SpellMissInfo::RESIST:  return CombatTextEntry::RESIST;
        case SpellMissInfo::IMMUNE2:
        case SpellMissInfo::IMMUNE3:
            return CombatTextEntry::IMMUNE;
        case SpellMissInfo::REFLECT: return CombatTextEntry::REFLECT;
        default: return CombatTextEntry::MISS;
    }
}

static audio::SpellSoundManager::MagicSchool schoolMaskToMagicSchool(uint32_t mask) {
    if (mask & 0x04) return audio::SpellSoundManager::MagicSchool::FIRE;
    if (mask & 0x10) return audio::SpellSoundManager::MagicSchool::FROST;
    if (mask & 0x02) return audio::SpellSoundManager::MagicSchool::HOLY;
    if (mask & 0x08) return audio::SpellSoundManager::MagicSchool::NATURE;
    if (mask & 0x20) return audio::SpellSoundManager::MagicSchool::SHADOW;
    if (mask & 0x40) return audio::SpellSoundManager::MagicSchool::ARCANE;
    return audio::SpellSoundManager::MagicSchool::ARCANE;
}

// ---- Extracted helpers to reduce nesting in handleSpellGo ----

audio::SpellSoundManager::MagicSchool SpellHandler::resolveSpellSchool(uint32_t spellId) {
    owner_.loadSpellNameCache();
    auto it = owner_.spellNameCacheRef().find(spellId);
    if (it != owner_.spellNameCacheRef().end() && it->second.schoolMask)
        return schoolMaskToMagicSchool(it->second.schoolMask);
    return audio::SpellSoundManager::MagicSchool::ARCANE;
}

void SpellHandler::playSpellCastSound(uint32_t spellId) {
    auto* ac = owner_.services().audioCoordinator;
    if (!ac) return;
    auto* ssm = ac->getSpellSoundManager();
    if (!ssm) return;
    ssm->playCast(resolveSpellSchool(spellId));
}

void SpellHandler::playSpellImpactSound(uint32_t spellId) {
    auto* ac = owner_.services().audioCoordinator;
    if (!ac) return;
    auto* ssm = ac->getSpellSoundManager();
    if (!ssm) return;
    ssm->playImpact(resolveSpellSchool(spellId),
                     audio::SpellSoundManager::SpellPower::MEDIUM);
}

// ---- Spell visual effect helpers ----

uint32_t SpellHandler::resolveSpellVisualId(uint32_t spellId) {
    owner_.loadSpellNameCache();
    auto it = owner_.spellNameCacheRef().find(spellId);
    return (it != owner_.spellNameCacheRef().end()) ? it->second.spellVisualId : 0;
}

bool SpellHandler::resolveUnitPosition(uint64_t guid, glm::vec3& outPos) {
    auto* renderer = owner_.services().renderer;
    if (!renderer) return false;
    if (guid == owner_.getPlayerGuid()) {
        outPos = renderer->getCharacterPosition();
        return true;
    }
    auto entity = owner_.getEntityManager().getEntity(guid);
    if (!entity) return false;
    glm::vec3 canonical(entity->getLatestX(), entity->getLatestY(), entity->getLatestZ());
    outPos = core::coords::canonicalToRender(canonical);
    return true;
}

void SpellHandler::triggerCastVisual(uint32_t spellId, uint64_t casterGuid, uint32_t castTimeMs) {
    LOG_INFO("SpellVisual: triggerCastVisual spellId=", spellId, " casterGuid=0x", std::hex, casterGuid, std::dec);
    auto* renderer = owner_.services().renderer;
    if (!renderer) { LOG_WARNING("SpellVisual: triggerCastVisual — no renderer"); return; }
    auto* svs = renderer->getSpellVisualSystem();
    if (!svs) { LOG_WARNING("SpellVisual: triggerCastVisual — no SpellVisualSystem"); return; }
    uint32_t visualId = resolveSpellVisualId(spellId);
    if (visualId == 0) { LOG_WARNING("SpellVisual: triggerCastVisual — visualId=0 for spellId=", spellId); return; }
    glm::vec3 casterPos;
    if (!resolveUnitPosition(casterGuid, casterPos)) { LOG_DEBUG("SpellVisual: triggerCastVisual — cannot resolve caster position for guid=0x", std::hex, casterGuid, std::dec); return; }
    LOG_INFO("SpellVisual: triggerCastVisual visualId=", visualId, " pos=(", casterPos.x, ",", casterPos.y, ",", casterPos.z, ") castTimeMs=", castTimeMs);
    svs->playSpellVisualPrecast(visualId, casterPos, castTimeMs,
                                owner_.resolveUnitRenderInstance(casterGuid));
}

void SpellHandler::triggerImpactVisual(uint32_t spellId, uint64_t targetGuid) {
    LOG_INFO("SpellVisual: triggerImpactVisual spellId=", spellId, " targetGuid=0x", std::hex, targetGuid, std::dec);
    auto* renderer = owner_.services().renderer;
    if (!renderer) return;
    auto* svs = renderer->getSpellVisualSystem();
    if (!svs) return;
    uint32_t visualId = resolveSpellVisualId(spellId);
    if (visualId == 0) { LOG_WARNING("SpellVisual: triggerImpactVisual — visualId=0 for spellId=", spellId); return; }
    glm::vec3 targetPos;
    if (!resolveUnitPosition(targetGuid, targetPos)) return;
    LOG_INFO("SpellVisual: triggerImpactVisual visualId=", visualId, " pos=(", targetPos.x, ",", targetPos.y, ",", targetPos.z, ")");
    svs->playSpellVisual(visualId, targetPos, /*useImpactKit=*/true);
}

void SpellHandler::launchRangedWeaponProjectile(uint32_t spellId, uint64_t targetGuid) {
    if (targetGuid == 0) targetGuid = owner_.getTargetGuid();
    auto* renderer = owner_.services().renderer;
    auto* assets = owner_.services().assetManager;
    if (!renderer || !assets || targetGuid == 0) return;
    auto* visuals = renderer->getSpellVisualSystem();
    auto* characters = renderer->getCharacterRenderer();
    if (!visuals || !characters) return;

    glm::vec3 start = renderer->getCharacterPosition() + glm::vec3(0.0f, 0.0f, 1.0f);
    glm::mat4 handTransform(1.0f);
    if (characters->getAttachmentTransform(renderer->getCharacterInstanceId(), 1, handTransform))
        start = glm::vec3(handTransform[3]);

    glm::vec3 end;
    if (!resolveUnitPosition(targetGuid, end)) return;
    end.z += 1.0f;

    const auto& ranged = owner_.getInventory().getEquipSlot(EquipSlot::RANGED);
    std::string modelPath;
    std::string texturePath;
    bool spin = false;

    if (spellId == 2764 || (!ranged.empty() && ranged.item.inventoryType == InvType::THROWN)) {
        spin = true;
        if (!ranged.empty() && ranged.item.displayInfoId != 0) {
            auto displayDbc = assets->loadDBC("ItemDisplayInfo.dbc");
            if (displayDbc && displayDbc->isLoaded()) {
                const int32_t row = displayDbc->findRecordById(ranged.item.displayInfoId);
                if (row >= 0) {
                    const auto* layout = pipeline::getActiveDBCLayout()
                        ? pipeline::getActiveDBCLayout()->getLayout("ItemDisplayInfo") : nullptr;
                    std::string model = displayDbc->getString(
                        static_cast<uint32_t>(row), layout ? (*layout)["LeftModel"] : 1);
                    std::string texture = displayDbc->getString(
                        static_cast<uint32_t>(row), layout ? (*layout)["LeftModelTexture"] : 3);
                    if (!model.empty()) {
                        const size_t dot = model.rfind('.');
                        if (dot != std::string::npos) model = model.substr(0, dot);
                        modelPath = "Item\\ObjectComponents\\Weapon\\" + model + ".m2";
                    }
                    if (!texture.empty())
                        texturePath = "Item\\ObjectComponents\\Weapon\\" + texture + ".blp";
                }
            }
        }
        if (modelPath.empty())
            modelPath = "Item\\ObjectComponents\\Weapon\\Thrown_1H_Dagger_A_01.m2";
    } else if (spellId == 75 || (!ranged.empty() &&
               (ranged.item.inventoryType == InvType::RANGED_BOW ||
                ranged.item.subclassName == "Bow" || ranged.item.subclassName == "Crossbow"))) {
        modelPath = "Item\\ObjectComponents\\Ammo\\ArrowFlight_01.m2";
    } else {
        modelPath = "Item\\ObjectComponents\\Ammo\\BulletFlight_01.m2";
    }

    const float distance = glm::length(end - start);
    const float duration = std::clamp(distance / 35.0f, 0.12f, 0.8f);
    visuals->playPhysicalProjectile(modelPath, texturePath, start, end, duration, spin);
}


static std::string displaySpellName(GameHandler& handler, uint32_t spellId) {
    if (spellId == 0) return {};
    const std::string& name = handler.getSpellName(spellId);
    if (!name.empty()) return name;
    return "spell " + std::to_string(spellId);
}

static std::string formatSpellNameList(GameHandler& handler,
                                       const std::vector<uint32_t>& spellIds,
                                       size_t maxShown = 3) {
    if (spellIds.empty()) return {};

    const size_t shownCount = std::min(spellIds.size(), maxShown);
    std::ostringstream oss;
    for (size_t i = 0; i < shownCount; ++i) {
        if (i > 0) {
            if (shownCount == 2) {
                oss << " and ";
            } else if (i == shownCount - 1) {
                oss << ", and ";
            } else {
                oss << ", ";
            }
        }
        oss << displaySpellName(handler, spellIds[i]);
    }

    if (spellIds.size() > shownCount) {
        oss << ", and " << (spellIds.size() - shownCount) << " more";
    }

    return oss.str();
}

SpellHandler::SpellHandler(GameHandler& owner)
    : owner_(owner) {}

void SpellHandler::registerOpcodes(DispatchTable& table) {
    table[Opcode::SMSG_INITIAL_SPELLS] = [this](network::Packet& packet) { handleInitialSpells(packet); };
    table[Opcode::SMSG_CAST_FAILED] = [this](network::Packet& packet) { handleCastFailed(packet); };
    table[Opcode::SMSG_SPELL_START] = [this](network::Packet& packet) { handleSpellStart(packet); };
    table[Opcode::SMSG_SPELL_GO] = [this](network::Packet& packet) { handleSpellGo(packet); };
    table[Opcode::SMSG_SPELL_COOLDOWN] = [this](network::Packet& packet) { handleSpellCooldown(packet); };
    table[Opcode::SMSG_COOLDOWN_EVENT] = [this](network::Packet& packet) { handleCooldownEvent(packet); };
    table[Opcode::SMSG_AURA_UPDATE] = [this](network::Packet& packet) {
        handleAuraUpdate(packet, false);
    };
    table[Opcode::SMSG_AURA_UPDATE_ALL] = [this](network::Packet& packet) {
        handleAuraUpdate(packet, true);
    };
    table[Opcode::SMSG_LEARNED_SPELL] = [this](network::Packet& packet) { handleLearnedSpell(packet); };
    table[Opcode::SMSG_SUPERCEDED_SPELL] = [this](network::Packet& packet) { handleSupercededSpell(packet); };
    table[Opcode::SMSG_REMOVED_SPELL] = [this](network::Packet& packet) { handleRemovedSpell(packet); };
    table[Opcode::SMSG_SEND_UNLEARN_SPELLS] = [this](network::Packet& packet) { handleUnlearnSpells(packet); };
    table[Opcode::SMSG_TALENTS_INFO] = [this](network::Packet& packet) { handleTalentsInfo(packet); };
    // Server asks the player to confirm a talent reset (guid + gold cost).
    // Must be handled here: this handler owns the pending-wipe state that the
    // confirm dialog reads.
    table[Opcode::MSG_TALENT_WIPE_CONFIRM] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(12)) { packet.skipAll(); return; }
        talentWipeNpcGuid_ = packet.readUInt64();
        talentWipeCost_    = packet.readUInt32();
        talentWipePending_ = true;
        LOG_INFO("MSG_TALENT_WIPE_CONFIRM: npc=0x", std::hex, talentWipeNpcGuid_, std::dec,
                 " cost=", talentWipeCost_);
        owner_.fireAddonEvent("CONFIRM_TALENT_WIPE", {std::to_string(talentWipeCost_)});
    };
    // SMSG_PET_UNLEARN_CONFIRM: uint64 petGuid + uint32 cost (copper). Handled
    // here for the same reason — this handler owns the pending-unlearn state.
    // The other pet opcodes have different formats and must NOT set unlearn state.
    table[Opcode::SMSG_PET_UNLEARN_CONFIRM] = [this](network::Packet& packet) {
        if (packet.hasRemaining(12)) {
            petUnlearnGuid_ = packet.readUInt64();
            petUnlearnCost_ = packet.readUInt32();
            petUnlearnPending_ = true;
        }
        packet.skipAll();
    };
    table[Opcode::SMSG_ACHIEVEMENT_EARNED] = [this](network::Packet& packet) {
        handleAchievementEarned(packet);
    };
    // SMSG_EQUIPMENT_SET_LIST — owned by InventoryHandler::registerOpcodes

    // ---- Cast result / spell visuals / cooldowns / modifiers ----
    table[Opcode::SMSG_CAST_RESULT] = [this](network::Packet& p) { handleCastResult(p); };
    table[Opcode::SMSG_SPELL_FAILED_OTHER] = [this](network::Packet& p) { handleSpellFailedOther(p); };
    table[Opcode::SMSG_CLEAR_COOLDOWN] = [this](network::Packet& p) { handleClearCooldown(p); };
    table[Opcode::SMSG_MODIFY_COOLDOWN] = [this](network::Packet& p) { handleModifyCooldown(p); };
    table[Opcode::SMSG_PLAY_SPELL_VISUAL] = [this](network::Packet& p) { handlePlaySpellVisual(p); };
    table[Opcode::SMSG_SET_FLAT_SPELL_MODIFIER] = [this](network::Packet& p) { handleSpellModifier(p, true); };
    table[Opcode::SMSG_SET_PCT_SPELL_MODIFIER]  = [this](network::Packet& p) { handleSpellModifier(p, false); };
    table[Opcode::SMSG_SPELL_DELAYED] = [this](network::Packet& p) { handleSpellDelayed(p); };

    // ---- Spell log / aura / dispel / totem / channel handlers ----
    table[Opcode::SMSG_SPELLLOGMISS] = [this](network::Packet& p) { handleSpellLogMiss(p); };
    table[Opcode::SMSG_SPELL_FAILURE] = [this](network::Packet& p) { handleSpellFailure(p); };
    table[Opcode::SMSG_ITEM_COOLDOWN] = [this](network::Packet& p) { handleItemCooldown(p); };
    table[Opcode::SMSG_DISPEL_FAILED] = [this](network::Packet& p) { handleDispelFailed(p); };
    table[Opcode::SMSG_TOTEM_CREATED] = [this](network::Packet& p) { handleTotemCreated(p); };
    table[Opcode::SMSG_PERIODICAURALOG] = [this](network::Packet& p) { handlePeriodicAuraLog(p); };
    table[Opcode::SMSG_SPELLENERGIZELOG] = [this](network::Packet& p) { handleSpellEnergizeLog(p); };
    table[Opcode::SMSG_INIT_EXTRA_AURA_INFO_OBSOLETE] = [this](network::Packet& p) { handleExtraAuraInfo(p, true); };
    table[Opcode::SMSG_SET_EXTRA_AURA_INFO_OBSOLETE] = [this](network::Packet& p) { handleExtraAuraInfo(p, false); };
    table[Opcode::SMSG_SET_EXTRA_AURA_INFO_NEED_UPDATE] = [this](network::Packet& p) { handleExtraAuraInfo(p, false); };
    table[Opcode::SMSG_SET_EXTRA_AURA_INFO_NEED_UPDATE_OBSOLETE] = [this](network::Packet& p) { handleExtraAuraInfo(p, false); };
    table[Opcode::SMSG_SPELLDISPELLOG] = [this](network::Packet& p) { handleSpellDispelLog(p); };
    table[Opcode::SMSG_SPELLSTEALLOG] = [this](network::Packet& p) { handleSpellStealLog(p); };
    table[Opcode::SMSG_SPELL_CHANCE_PROC_LOG] = [this](network::Packet& p) { handleSpellChanceProcLog(p); };
    table[Opcode::SMSG_SPELLINSTAKILLLOG] = [this](network::Packet& p) { handleSpellInstaKillLog(p); };
    table[Opcode::SMSG_SPELLLOGEXECUTE] = [this](network::Packet& p) { handleSpellLogExecute(p); };
    table[Opcode::SMSG_CLEAR_EXTRA_AURA_INFO] = [this](network::Packet& p) { handleClearExtraAuraInfo(p); };
    table[Opcode::SMSG_CLEAR_EXTRA_AURA_INFO_OBSOLETE] = [this](network::Packet& p) { handleClearExtraAuraInfo(p); };
    table[Opcode::SMSG_ITEM_ENCHANT_TIME_UPDATE] = [this](network::Packet& p) { handleItemEnchantTimeUpdate(p); };
    table[Opcode::SMSG_RESUME_CAST_BAR] = [this](network::Packet& p) { handleResumeCastBar(p); };
    table[Opcode::MSG_CHANNEL_START] = [this](network::Packet& p) { handleChannelStart(p); };
    table[Opcode::MSG_CHANNEL_UPDATE] = [this](network::Packet& p) { handleChannelUpdate(p); };
}

// ============================================================
// Public API
// ============================================================

bool SpellHandler::isGameObjectInteractionCasting() const {
    return casting_ && currentCastSpellId_ == 0 && owner_.pendingGameObjectInteractGuidRef() != 0;
}

bool SpellHandler::isTargetCasting() const {
    return getUnitCastState(owner_.getTargetGuid()) != nullptr;
}

uint32_t SpellHandler::getTargetCastSpellId() const {
    auto* s = getUnitCastState(owner_.getTargetGuid());
    return s ? s->spellId : 0;
}

float SpellHandler::getTargetCastProgress() const {
    auto* s = getUnitCastState(owner_.getTargetGuid());
    return (s && s->timeTotal > 0.0f)
        ? (s->timeTotal - s->timeRemaining) / s->timeTotal : 0.0f;
}

float SpellHandler::getTargetCastTimeRemaining() const {
    auto* s = getUnitCastState(owner_.getTargetGuid());
    return s ? s->timeRemaining : 0.0f;
}

bool SpellHandler::isTargetCastInterruptible() const {
    auto* s = getUnitCastState(owner_.getTargetGuid());
    return s ? s->interruptible : true;
}

void SpellHandler::castSpell(uint32_t spellId, uint64_t targetGuid) {
    LOG_DEBUG("castSpell: spellId=", spellId, " target=0x", std::hex, targetGuid, std::dec);
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;

    // Food and water are server auras, but using any action interrupts them.
    // Cancel the aura and stand first, then allow the requested action to proceed.
    if (restorationActive_) cancelCast();

    // Attack (6603) routes to auto-attack instead of cast
    if (spellId == 6603) {
        uint64_t target = targetGuid != 0 ? targetGuid : owner_.getTargetGuid();
        if (target != 0) {
            if (owner_.isAutoAttacking()) {
                owner_.stopAutoAttack();
            } else {
                owner_.startAutoAttack(target);
            }
        }
        return;
    }

    // Action bars restored from the server can still hold a rank that a higher rank
    // has since superseded. The server drops casts of superseded ranks without
    // sending any error, so swap in the highest rank we actually know.
    spellId = resolveHighestKnownRank(spellId);

    // Fishing places a bobber in front of the caster using the facing the server has
    // on record. The client only sends MSG_MOVE_SET_FACING when the aim changes by >3°
    // (throttled to 10 Hz), so a small final aim adjustment before pressing cast may not
    // have reached the server — leaving it with a slightly stale facing that drops the
    // bobber off to the side or on land ("Face open water"). Push the exact current
    // facing right before the cast so the bobber lands where the player is aiming.
    if (spellclass::isFishingCast(spellId) && owner_.getSocket()) {
        // Snap the character to face where the camera is looking, then push that facing to
        // the server, so the bobber lands in front of the player's view. Standing still the
        // character yaw doesn't follow the free-look camera, so without this the bobber
        // dropped toward a stale heading (off to the side / on land → "Face open water").
        const float canonO = owner_.faceCameraDirection();
        owner_.setOrientation(canonO);
        owner_.sendMovement(Opcode::MSG_MOVE_SET_FACING);
    }

    // Profession spells (Cooking, First Aid, Alchemy, ...) open the crafting
    // window client-side instead of being sent as casts — matching the real
    // client, where these spells just open the tradeskill UI.
    if (uint32_t craftSkillLine = tradeskillOpenerSkillLine(spellId)) {
        LOG_INFO("castSpell: spell ", spellId, " opens crafting window for skill line ", craftSkillLine);
        openCraftingWindow(craftSkillLine);
        return;
    }

    // Pressing the companion you already have out puts it away. A companion has
    // no aura, so there is nothing in the buff bar to right-click and this is
    // the only way to send one back — without it the button could only ever
    // summon, which is why pressing it again just produced the same critter.
    if (spellId != 0 && spellId == owner_.getActiveCritterSpellId() &&
        owner_.getActiveCritterGuid() != 0) {
        owner_.dismissCritter();
        return;
    }

    // Casting any spell while mounted → dismount first, then cast (retail
    // behavior: the mount is a cancellable aura that a cast removes). Exception:
    // while actively flying (airborne on a flying mount) retail blocks the cast
    // entirely rather than dropping the player out of the sky, so bail there.
    // dismount() clears the local mount state synchronously, so control falls
    // through and the cast is sent from the ground in the same action.
    if (owner_.isMounted()) {
        if (owner_.isPlayerFlying()) {
            owner_.addUIError("You can't do that while flying.");
            return;
        }
        // Pressing the mount you are already riding dismounts you and stops
        // there. Falling through to the cast would put the player straight back
        // on the same mount, so the button appeared to do nothing.
        const bool ridingThisMount =
            spellId != 0 && spellId == owner_.getMountAuraSpellId();
        owner_.dismount();
        if (ridingThisMount) {
            LOG_INFO("Dismount via mount action: spell=", spellId);
            return;
        }
    }

    if (casting_) {
        // Spell queue: if we're within 400ms of the cast completing (and not channeling),
        // store the spell so it fires automatically when the cast finishes.
        if (!castIsChannel_ && castTimeRemaining_ > 0.0f && castTimeRemaining_ <= 0.4f) {
            queuedSpellId_     = spellId;
            queuedSpellTarget_ = targetGuid != 0 ? targetGuid : owner_.getTargetGuid();
            LOG_INFO("Spell queue: queued spellId=", spellId, " (", castTimeRemaining_ * 1000.0f,
                     "ms remaining)");
        }
        return;
    }

    // Stop movement before casting — servers reject cast-time spells while moving
    const uint32_t moveFlags = owner_.movementInfoRef().flags;
    const bool isMoving = (moveFlags & 0x0Fu) != 0; // FORWARD|BACKWARD|STRAFE_LEFT|STRAFE_RIGHT
    if (isMoving) {
        owner_.movementInfoRef().flags &= ~0x0Fu;
        owner_.sendMovement(Opcode::MSG_MOVE_STOP);
    }

    // Remembered so that, if this cast turns out to be what mounted the player,
    // the mount aura can be identified exactly rather than guessed at.
    if (!owner_.isMounted()) lastGroundCastSpellId_ = spellId;

    const bool fishingCast = spellclass::isFishingCast(spellId);
    uint64_t target = targetGuid != 0 ? targetGuid : owner_.getTargetGuid();
    // Self-targeted spells (hearthstone, shouts, self-buffs) always land on the
    // caster, so they must not carry the current target along.
    const bool selfCast = (spellId == 8690) || isSelfCastSpell(spellId);
    if (selfCast || fishingCast) target = 0;

    // Spells cast at an item — Disenchant, Prospecting, Milling, the enchant
    // formulas — carry TARGET_FLAG_ITEM in Spell.dbc's Targets. Sending one with
    // no item in SpellCastTargets is what made the server answer "can't be
    // disenchanted": it evaluated the question against nothing. Arm the same
    // item-picking cursor an enchanting scroll uses and send the cast once the
    // player chooses.
    constexpr uint32_t kSpellTargetFlagItem = 0x10;
    if ((getSpellTargetFlags(spellId) & kSpellTargetFlagItem) != 0) {
        if (owner_.isAwaitingItemTarget()) {
            owner_.addSystemChatMessage("Choose an item first.");
            return;
        }
        owner_.beginSpellItemTargeting(spellId, getSpellName(spellId));
        return;
    }

    // Auto self-cast: a spell that has to be aimed at a friendly unit falls back
    // to the caster when nothing friendly is selected — no target at all, or an
    // enemy, which is the usual state mid-fight. Without it, healing yourself
    // while fighting means dropping the target, casting, and picking it back up.
    if (!selfCast && !fishingCast && getSpellImplicitTargetA(spellId) != 0 &&
        spellclass::requiresFriendlyTarget(getSpellImplicitTargetA(spellId))) {
        bool haveFriendlyTarget = false;
        if (target != 0 && target != owner_.getPlayerGuid()) {
            if (auto entity = owner_.getEntityManager().getEntity(target)) {
                // Players and their pets are friendly unless flagged otherwise;
                // hostility is the faction check the nameplate colour uses.
                if (entity->isUnit()) {
                    haveFriendlyTarget =
                        !static_cast<Unit*>(entity.get())->isHostile();
                }
            }
        }
        if (!haveFriendlyTarget) {
            target = owner_.getPlayerGuid();
        }
    }

    // Track whether a spell-specific block already handled facing so the generic
    // facing block below doesn't send redundant SET_FACING packets.
    //
    // A spell that lands on you never turns you. Facing is worked out from the
    // vector between you and your target, and when the target is you that
    // vector is the gap between the position this client has moved you to and
    // the one the server last confirmed — which, while running, points
    // backwards along your own path. An instant self-cast on auto-run
    // therefore spun the character a half-turn and sent them back the way they
    // came. The gap clears the length check whenever you are moving at all,
    // which is why it only happened in motion.
    //
    // selfCast above catches the spells that are self-targeted by nature;
    // this catches aiming a friendly spell at yourself, where the target guid
    // is your own and every facing block below would otherwise use it.
    bool facingHandled = (target != 0 && target == owner_.getPlayerGuid());

    // Warrior Charge (ranks 1-3): client-side range check + charge callback
    if (spellId == 100 || spellId == 6178 || spellId == 11578) {
        // Charge is an opener: it cannot be used once the fight has started.
        if (owner_.isInCombat()) {
            owner_.addSystemChatMessage("You can't do that while in combat.");
            return;
        }
        if (target == 0) {
            owner_.addSystemChatMessage("You have no target.");
            return;
        }
        auto entity = owner_.getEntityManager().getEntity(target);
        if (!entity) {
            owner_.addSystemChatMessage("You have no target.");
            return;
        }
        // Charge needs an attackable unit. Game objects (mining nodes, herb nodes,
        // chests, doors) and corpse objects can all be the current target in this
        // client, and none of them derive from Unit — so they used to slip past the
        // hostility checks below untouched and let the player charge at scenery.
        auto unit = std::dynamic_pointer_cast<Unit>(entity);
        const ObjectType targetType = entity->getType();
        if (!unit || (targetType != ObjectType::UNIT && targetType != ObjectType::PLAYER)) {
            owner_.addSystemChatMessage("You cannot attack that target.");
            return;
        }
        // Corpses cannot be charged.
        if (unit->getHealth() == 0) {
            owner_.addSystemChatMessage("You cannot attack that target.");
            return;
        }
        if (targetType == ObjectType::UNIT) {
            // Neutral combat creatures (yellow-name mobs such as Goretusks)
            // are valid Charge targets even though they are not inherently
            // hostile. Match normal right-click combat by rejecting only
            // service NPCs and targets the server explicitly marks as
            // non-attackable/immune, rather than requiring hostile faction.
            constexpr uint32_t UNIT_FLAG_NON_ATTACKABLE = 0x00000002;
            constexpr uint32_t UNIT_FLAG_IMMUNE_TO_PC   = 0x00000100;
            constexpr uint32_t UNIT_FLAG_NOT_SELECTABLE = 0x02000000;
            constexpr uint32_t kBlockedChargeFlags =
                UNIT_FLAG_NON_ATTACKABLE |
                UNIT_FLAG_IMMUNE_TO_PC |
                UNIT_FLAG_NOT_SELECTABLE;
            const bool hostileOrAggressive =
                unit->isHostile() || owner_.isAggressiveTowardPlayer(target);
            const bool clearlyFriendly = unit->isInteractable() && !hostileOrAggressive;
            if (clearlyFriendly || (unit->getUnitFlags() & kBlockedChargeFlags) != 0) {
                owner_.addSystemChatMessage("You cannot attack that target.");
                return;
            }
        }
        float tx = entity->getX(), ty = entity->getY(), tz = entity->getZ();
        float dx = tx - owner_.movementInfoRef().x;
        float dy = ty - owner_.movementInfoRef().y;
        float dz = tz - owner_.movementInfoRef().z;
        float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (dist < 8.0f) {
            owner_.addSystemChatMessage("Target is too close.");
            return;
        }
        if (dist > 25.0f) {
            owner_.addSystemChatMessage("Out of range.");
            return;
        }
        float yaw = std::atan2(-dy, dx);
        owner_.movementInfoRef().orientation = yaw;
        owner_.sendMovement(Opcode::MSG_MOVE_SET_FACING);
        if (owner_.chargeCallbackRef()) {
            owner_.chargeCallbackRef()(target, tx, ty, tz);
        }
        facingHandled = true;
    }

    // Instant melee abilities: client-side range + facing check.
    // Melee is decided by the spell's own range, not by its school. SpellRange calls
    // melee "Combat Range" (5 yards), while physical-school abilities that are cast at
    // range — Steady Shot, Multi-Shot, Taunt, Deadly Throw — carry a 30-35 yard range.
    // Testing the school instead would range-check those at 8 yards and swallow the cast.
    // An unknown range (SpellRange.dbc unavailable) is not treated as melee, so the
    // server arbitrates rather than the client blocking a legitimate cast.
    if (!facingHandled) {
        const bool isMeleeAbility = spellclass::isMeleeRange(getSpellMaxRange(spellId));
        if (isMeleeAbility && target != 0) {
            auto entity = owner_.getEntityManager().getEntity(target);
            if (entity) {
                float dx = entity->getX() - owner_.movementInfoRef().x;
                float dy = entity->getY() - owner_.movementInfoRef().y;
                float dz = entity->getZ() - owner_.movementInfoRef().z;
                float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (dist > 8.0f) {
                    owner_.addSystemChatMessage("Out of range.");
                    return;
                }
                owner_.faceCanonicalYaw(std::atan2(-dy, dx));
                facingHandled = true;
            }
        }
    }

    // Face the target before casting any targeted spell (server checks facing arc).
    // Only send if a spell-specific block above didn't already handle facing,
    // to avoid redundant SET_FACING packets that waste bandwidth.
    if (!facingHandled && target != 0) {
        auto entity = owner_.getEntityManager().getEntity(target);
        if (entity) {
            float dx = entity->getX() - owner_.movementInfoRef().x;
            float dy = entity->getY() - owner_.movementInfoRef().y;
            float lenSq = dx * dx + dy * dy;
            if (lenSq > 0.01f) {
                owner_.faceCanonicalYaw(std::atan2(-dy, dx));
            }
        }
    }
    // Heartbeat ensures the server has the updated orientation before the cast packet.
    if (target != 0 || fishingCast) {
        owner_.sendMovement(Opcode::MSG_MOVE_HEARTBEAT);
    }

    auto packet = owner_.getPacketParsers()
        ? owner_.getPacketParsers()->buildCastSpell(spellId, target, ++castCount_)
        : CastSpellPacket::build(spellId, target, ++castCount_);
    LOG_DEBUG("CMSG_CAST_SPELL: spellId=", spellId, " target=0x", std::hex, target, std::dec,
              " castCount=", static_cast<int>(castCount_), " packetSize=", packet.getSize());
    owner_.getSocket()->send(packet);
    LOG_INFO("Casting spell: ", spellId, " on 0x", std::hex, target, std::dec);

    // Fire UNIT_SPELLCAST_SENT for cast bar addons
    if (owner_.addonEventCallbackRef()) {
        std::string targetName;
        if (target != 0) targetName = owner_.lookupName(target);
        owner_.addonEventCallbackRef()("UNIT_SPELLCAST_SENT", {"player", targetName, std::to_string(spellId)});
    }

    // Optimistically start GCD immediately on cast
    if (!isGCDActive()) {
        gcdTotal_ = 1.5f;
        gcdStartedAt_ = std::chrono::steady_clock::now();
    }
}

void SpellHandler::cancelCast() {
    if (restorationActive_) {
        const uint32_t auraSpell = restorationSpellId_;
        cancelAura(auraSpell);
        owner_.setStandState(0);
        stopRestorationPresentation();
        queuedSpellId_ = 0;
        queuedSpellTarget_ = 0;
        LOG_INFO("Cancelled restoration aura: spellId=", auraSpell);
        return;
    }
    if (!casting_) return;
    // GameObject interaction cast is client-side timing only.
    if (owner_.pendingGameObjectInteractGuidRef() == 0 &&
        owner_.getState() == WorldState::IN_WORLD && owner_.getSocket() &&
        currentCastSpellId_ != 0) {
        auto packet = CancelCastPacket::build(currentCastSpellId_);
        owner_.getSocket()->send(packet);
    }
    owner_.pendingGameObjectInteractGuidRef() = 0;
    owner_.lastInteractedGoGuidRef() = 0;
    casting_ = false;
    castIsChannel_ = false;
    currentCastSpellId_ = 0;
    castTimeRemaining_ = 0.0f;
    craftQueueSpellId_ = 0;
    craftQueueRemaining_ = 0;
    queuedSpellId_ = 0;
    queuedSpellTarget_ = 0;
    if (owner_.addonEventCallbackRef())
        owner_.addonEventCallbackRef()("UNIT_SPELLCAST_STOP", {"player"});
    // Remove lingering precast visual effects
    if (auto* renderer = owner_.services().renderer) {
        if (auto* svs = renderer->getSpellVisualSystem())
            svs->cancelAllPrecastVisuals();
    }
}

void SpellHandler::startCraftQueue(uint32_t spellId, int count) {
    // castSpell() dismounts a ground mount and then casts, so crafting while
    // mounted just works. But while airborne on a flying mount it blocks the
    // cast — guard that here too so we don't populate the queue with a cast
    // that will never fire (which would freeze the crafting UI on
    // "Crafting... N remaining").
    if (owner_.isMounted() && owner_.isPlayerFlying()) {
        owner_.addUIError("You can't do that while flying.");
        return;
    }
    craftQueueSpellId_ = spellId;
    craftQueueRemaining_ = count;
    castSpell(spellId, 0);
}

void SpellHandler::cancelCraftQueue() {
    craftQueueSpellId_ = 0;
    craftQueueRemaining_ = 0;
}

void SpellHandler::cancelAura(uint32_t spellId) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = CancelAuraPacket::build(spellId);
    owner_.getSocket()->send(packet);
}

float SpellHandler::getSpellCooldown(uint32_t spellId) const {
    auto it = spellCooldowns_.find(spellId);
    return (it != spellCooldowns_.end()) ? it->second : 0.0f;
}

void SpellHandler::seedCooldownFromSpellInfo(uint32_t spellId) {
    if (spellId == 0) return;
    auto existing = spellCooldowns_.find(spellId);
    if (existing != spellCooldowns_.end() && existing->second > 0.5f) return;

    owner_.loadSpellNameCache();
    auto it = owner_.spellNameCacheRef().find(spellId);
    if (it == owner_.spellNameCacheRef().end()) return;

    const uint32_t cooldownMs = std::max(it->second.recoveryMs, it->second.categoryRecoveryMs);
    if (cooldownMs <= 1500) return; // ignore GCD-sized recovery

    const float seconds = cooldownMs / 1000.0f;
    spellCooldowns_[spellId] = seconds;
    for (auto& slot : owner_.actionBarRef()) {
        if (slot.type != ActionBarSlot::SPELL || slot.id != spellId) continue;
        slot.cooldownRemaining = seconds;
        slot.cooldownTotal = seconds;
    }

    LOG_DEBUG("Seeded cooldown from Spell.dbc: spell=", spellId, " ms=", cooldownMs);
    if (owner_.addonEventCallbackRef()) {
        owner_.addonEventCallbackRef()("SPELL_UPDATE_COOLDOWN", {});
        owner_.addonEventCallbackRef()("ACTIONBAR_UPDATE_COOLDOWN", {});
    }
}

void SpellHandler::learnTalent(uint32_t talentId, uint32_t requestedRank) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) {
        LOG_WARNING("learnTalent: Not in world or no socket connection");
        return;
    }

    LOG_INFO("Requesting to learn talent: id=", talentId, " rank=", requestedRank);

    auto packet = LearnTalentPacket::build(talentId, requestedRank);
    owner_.getSocket()->send(packet);
}

void SpellHandler::switchTalentSpec(uint8_t newSpec) {
    if (newSpec > 1) {
        LOG_WARNING("Invalid talent spec: ", (int)newSpec);
        return;
    }
    if (newSpec == activeTalentSpec_) {
        LOG_INFO("Already on spec ", (int)newSpec);
        return;
    }
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;

    // Switching spec is a spell cast, not a message. AzerothCore lists
    // CMSG_SET_ACTIVE_TALENT_GROUP_OBSOLETE as Handle_NULL — it reads the
    // packet and does nothing — and the effect that actually moves a player
    // between specs is SPELL_EFFECT_TALENT_SPEC_SELECT, cast at themselves.
    //
    // This sent that dead opcode and then set the active spec locally anyway.
    // So the client believed it was on the second spec while the server had
    // never left the first, and everything the talent frame reads came from
    // the second spec's slots — which the server never fills, because it keeps
    // reporting the first. That is both halves of what was seen: a spec that
    // switched itself, and a first spec that never received the points earned
    // by levelling.
    //
    // Named the way Spell.dbc names them, which is the opposite way round from
    // how the ids read: 63645 is the primary and 63644 is the secondary.
    constexpr uint32_t kActivatePrimarySpec = 63645;
    constexpr uint32_t kActivateSecondarySpec = 63644;
    const uint32_t spellId = (newSpec == 0) ? kActivatePrimarySpec
                                            : kActivateSecondarySpec;

    // Both are taught when dual specialisation is bought, so not knowing one
    // means the character has only the single spec.
    if (!knownSpells_.count(spellId)) {
        owner_.addUIError("You have not learned Dual Talent Specialization.");
        owner_.addSystemChatMessage("You have not learned Dual Talent Specialization.");
        return;
    }

    castSpell(spellId, owner_.getPlayerGuid());
    LOG_INFO("Activating talent spec ", (int)newSpec, " by casting ", spellId);
    // Deliberately not set here. The server answers a successful switch with
    // SMSG_TALENTS_INFO naming the group it moved to, and that is the only
    // thing that should move this — a local guess is what caused the drift.
}

void SpellHandler::confirmTalentWipe() {
    if (!talentWipePending_) return;
    talentWipePending_ = false;

    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;

    network::Packet pkt(wireOpcode(Opcode::MSG_TALENT_WIPE_CONFIRM));
    pkt.writeUInt64(talentWipeNpcGuid_);
    owner_.getSocket()->send(pkt);

    LOG_INFO("confirmTalentWipe: sent confirm for npc=0x", std::hex, talentWipeNpcGuid_, std::dec);
    owner_.addSystemChatMessage("Talent reset confirmed. The server will update your talents.");
    talentWipeNpcGuid_ = 0;
    talentWipeCost_ = 0;
}

void SpellHandler::confirmPetUnlearn() {
    if (!petUnlearnPending_) return;
    petUnlearnPending_ = false;
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;

    network::Packet pkt(wireOpcode(Opcode::CMSG_PET_UNLEARN_TALENTS));
    owner_.getSocket()->send(pkt);
    LOG_INFO("confirmPetUnlearn: sent CMSG_PET_UNLEARN_TALENTS");
    owner_.addSystemChatMessage("Pet talent reset confirmed.");
    petUnlearnGuid_ = 0;
    petUnlearnCost_ = 0;
}

uint32_t SpellHandler::findOnUseSpellId(uint32_t itemId) const {
    if (auto* info = owner_.getItemInfo(itemId)) {
        for (const auto& sp : info->spells) {
            // spellTrigger 0 = "Use", 5 = "No Delay" — both are player-activated on-use effects
            if (sp.spellId != 0 && (sp.spellTrigger == 0 || sp.spellTrigger == 5)) {
                return sp.spellId;
            }
        }
    }
    return 0;
}

void SpellHandler::useItemBySlot(int backpackIndex) {
    if (backpackIndex < 0 || backpackIndex >= owner_.inventoryRef().getBackpackSize()) return;
    const auto& slot = owner_.inventoryRef().getBackpackSlot(backpackIndex);
    if (slot.empty()) return;

    uint64_t itemGuid = owner_.backpackSlotGuidsRef()[backpackIndex];
    if (itemGuid == 0) {
        itemGuid = owner_.resolveOnlineItemGuid(slot.item.itemId);
    }

    if (itemGuid != 0 && owner_.getState() == WorldState::IN_WORLD && owner_.getSocket()) {
        uint32_t useSpellId = findOnUseSpellId(slot.item.itemId);
        const auto* itemInfo = owner_.getItemInfo(slot.item.itemId);
        const uint64_t targetGuid = targetGuidForUseItem(owner_, itemInfo);
        auto packet = owner_.getPacketParsers()
            ? owner_.getPacketParsers()->buildUseItem(0xFF, static_cast<uint8_t>(Inventory::NUM_EQUIP_SLOTS + backpackIndex), itemGuid, useSpellId, targetGuid)
            : UseItemPacket::build(0xFF, static_cast<uint8_t>(Inventory::NUM_EQUIP_SLOTS + backpackIndex), itemGuid, useSpellId, targetGuid);
        owner_.getSocket()->send(packet);
    } else if (itemGuid == 0) {
        owner_.addSystemChatMessage("Cannot use that item right now.");
    }
}

void SpellHandler::useItemInBag(int bagIndex, int slotIndex) {
    if (bagIndex < 0 || bagIndex >= owner_.inventoryRef().NUM_BAG_SLOTS) return;
    if (slotIndex < 0 || slotIndex >= owner_.inventoryRef().getBagSize(bagIndex)) return;
    const auto& slot = owner_.inventoryRef().getBagSlot(bagIndex, slotIndex);
    if (slot.empty()) return;

    uint64_t itemGuid = 0;
    uint64_t bagGuid = owner_.equipSlotGuidsRef()[Inventory::FIRST_BAG_EQUIP_SLOT + bagIndex];
    if (bagGuid != 0) {
        auto it = owner_.containerContentsRef().find(bagGuid);
        if (it != owner_.containerContentsRef().end() && slotIndex < static_cast<int>(it->second.numSlots)) {
            itemGuid = it->second.slotGuids[slotIndex];
        }
    }
    if (itemGuid == 0) {
        itemGuid = owner_.resolveOnlineItemGuid(slot.item.itemId);
    }

    LOG_INFO("useItemInBag: bag=", bagIndex, " slot=", slotIndex, " itemId=", slot.item.itemId,
             " itemGuid=0x", std::hex, itemGuid, std::dec);

    if (itemGuid != 0 && owner_.getState() == WorldState::IN_WORLD && owner_.getSocket()) {
        uint32_t useSpellId = findOnUseSpellId(slot.item.itemId);
        uint8_t wowBag = static_cast<uint8_t>(Inventory::FIRST_BAG_EQUIP_SLOT + bagIndex);
        const auto* itemInfo = owner_.getItemInfo(slot.item.itemId);
        const uint64_t targetGuid = targetGuidForUseItem(owner_, itemInfo);
        auto packet = owner_.getPacketParsers()
            ? owner_.getPacketParsers()->buildUseItem(wowBag, static_cast<uint8_t>(slotIndex), itemGuid, useSpellId, targetGuid)
            : UseItemPacket::build(wowBag, static_cast<uint8_t>(slotIndex), itemGuid, useSpellId, targetGuid);
        LOG_INFO("useItemInBag: sending CMSG_USE_ITEM, bag=", (int)wowBag, " slot=", slotIndex,
                 " packetSize=", packet.getSize());
        owner_.getSocket()->send(packet);
    } else if (itemGuid == 0) {
        LOG_WARNING("Use item in bag failed: missing item GUID for bag ", bagIndex, " slot ", slotIndex);
        owner_.addSystemChatMessage("Cannot use that item right now.");
    }
}

void SpellHandler::useItemById(uint32_t itemId) {
    if (itemId == 0) return;
    LOG_DEBUG("useItemById: searching for itemId=", itemId);
    for (int i = 0; i < owner_.inventoryRef().getBackpackSize(); i++) {
        const auto& slot = owner_.inventoryRef().getBackpackSlot(i);
        if (!slot.empty() && slot.item.itemId == itemId) {
            LOG_DEBUG("useItemById: found itemId=", itemId, " at backpack slot ", i);
            useItemBySlot(i);
            return;
        }
    }
    for (int bag = 0; bag < owner_.inventoryRef().NUM_BAG_SLOTS; bag++) {
        int bagSize = owner_.inventoryRef().getBagSize(bag);
        for (int slot = 0; slot < bagSize; slot++) {
            const auto& bagSlot = owner_.inventoryRef().getBagSlot(bag, slot);
            if (!bagSlot.empty() && bagSlot.item.itemId == itemId) {
                LOG_DEBUG("useItemById: found itemId=", itemId, " in bag ", bag, " slot ", slot);
                useItemInBag(bag, slot);
                return;
            }
        }
    }
    LOG_WARNING("useItemById: itemId=", itemId, " not found in inventory");
}

const std::vector<SpellHandler::SpellBookTab>& SpellHandler::getSpellBookTabs() {
    // Must be an instance member, not static — a static is shared across all
    // SpellHandler instances, so switching characters with the same spell count
    // would skip the rebuild and return the previous character's tabs.
    if (lastSpellCount_ == knownSpells_.size() && !spellBookTabsDirty_)
        return spellBookTabs_;
    lastSpellCount_ = knownSpells_.size();
    spellBookTabsDirty_ = false;
    spellBookTabs_.clear();

    static constexpr uint32_t SKILLLINE_CATEGORY_CLASS = 7;

    std::map<uint32_t, std::vector<uint32_t>> bySkillLine;
    std::vector<uint32_t> general;

    for (uint32_t spellId : knownSpells_) {
        auto slIt = owner_.spellToSkillLineRef().find(spellId);
        if (slIt != owner_.spellToSkillLineRef().end()) {
            uint32_t skillLineId = slIt->second;
            auto catIt = owner_.skillLineCategoriesRef().find(skillLineId);
            if (catIt != owner_.skillLineCategoriesRef().end() && catIt->second == SKILLLINE_CATEGORY_CLASS) {
                bySkillLine[skillLineId].push_back(spellId);
                continue;
            }
        }
        general.push_back(spellId);
    }

    auto byName = [this](uint32_t a, uint32_t b) {
        return owner_.getSpellName(a) < owner_.getSpellName(b);
    };

    if (!general.empty()) {
        std::sort(general.begin(), general.end(), byName);
        spellBookTabs_.push_back({"General", "Interface\\Icons\\INV_Misc_Book_09", std::move(general)});
    }

    std::vector<std::pair<std::string, std::vector<uint32_t>>> named;
    for (auto& [skillLineId, spells] : bySkillLine) {
        auto nameIt = owner_.skillLineNamesRef().find(skillLineId);
        std::string tabName = (nameIt != owner_.skillLineNamesRef().end()) ? nameIt->second : "Unknown";
        std::sort(spells.begin(), spells.end(), byName);
        named.emplace_back(std::move(tabName), std::move(spells));
    }
    std::sort(named.begin(), named.end(), [](const auto& a, const auto& b) { return a.first < b.first; });

    for (auto& [name, spells] : named) {
        spellBookTabs_.push_back({std::move(name), "Interface\\Icons\\INV_Misc_Book_09", std::move(spells)});
    }

    return spellBookTabs_;
}

void SpellHandler::loadTalentDbc() {
    if (talentDbcLoaded_) return;
    talentDbcLoaded_ = true;

    auto* am = owner_.services().assetManager;
    if (!am || !am->isInitialized()) return;

    // Load Talent.dbc
    auto talentDbc = am->loadDBC("Talent.dbc");
    if (talentDbc && talentDbc->isLoaded()) {
        const auto* talL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("Talent") : nullptr;
        const uint32_t tID = talL ? (*talL)["ID"] : 0;
        const uint32_t tTabID = talL ? (*talL)["TabID"] : 1;
        const uint32_t tRow = talL ? (*talL)["Row"] : 2;
        const uint32_t tCol = talL ? (*talL)["Column"] : 3;
        const uint32_t tRank0 = talL ? (*talL)["RankSpell0"] : 4;
        const uint32_t tPrereq0 = talL ? (*talL)["PrereqTalent0"] : 9;
        const uint32_t tPrereqR0 = talL ? (*talL)["PrereqRank0"] : 12;

        uint32_t count = talentDbc->getRecordCount();
        for (uint32_t i = 0; i < count; ++i) {
            TalentEntry entry;
            entry.talentId = talentDbc->getUInt32(i, tID);
            if (entry.talentId == 0) continue;

            entry.tabId = talentDbc->getUInt32(i, tTabID);
            entry.row = static_cast<uint8_t>(talentDbc->getUInt32(i, tRow));
            entry.column = static_cast<uint8_t>(talentDbc->getUInt32(i, tCol));

            for (int r = 0; r < 5; ++r) {
                entry.rankSpells[r] = talentDbc->getUInt32(i, tRank0 + r);
            }

            for (int p = 0; p < 3; ++p) {
                entry.prereqTalent[p] = talentDbc->getUInt32(i, tPrereq0 + p);
                entry.prereqRank[p] = static_cast<uint8_t>(talentDbc->getUInt32(i, tPrereqR0 + p));
            }

            entry.maxRank = 0;
            for (int r = 0; r < 5; ++r) {
                if (entry.rankSpells[r] != 0) {
                    entry.maxRank = r + 1;
                }
            }

            talentCache_[entry.talentId] = entry;
        }
        LOG_INFO("Loaded ", talentCache_.size(), " talents from Talent.dbc");
    } else {
        LOG_WARNING("Could not load Talent.dbc");
    }

    // Load TalentTab.dbc
    auto tabDbc = am->loadDBC("TalentTab.dbc");
    if (tabDbc && tabDbc->isLoaded()) {
        const auto* ttL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("TalentTab") : nullptr;
        // Cache field indices before the loop
        const uint32_t ttIdField    = ttL ? (*ttL)["ID"]             : 0;
        const uint32_t ttNameField  = ttL ? (*ttL)["Name"]           : 1;
        const uint32_t ttClassField = ttL ? (*ttL)["ClassMask"]      : 20;
        const uint32_t ttOrderField = ttL ? (*ttL)["OrderIndex"]     : 22;
        const uint32_t ttBgField    = ttL ? (*ttL)["BackgroundFile"] : 23;

        uint32_t count = tabDbc->getRecordCount();
        for (uint32_t i = 0; i < count; ++i) {
            TalentTabEntry entry;
            entry.tabId = tabDbc->getUInt32(i, ttIdField);
            if (entry.tabId == 0) continue;

            entry.name = tabDbc->getString(i, ttNameField);
            entry.classMask = tabDbc->getUInt32(i, ttClassField);
            entry.orderIndex = static_cast<uint8_t>(tabDbc->getUInt32(i, ttOrderField));
            entry.backgroundFile = tabDbc->getString(i, ttBgField);

            talentTabCache_[entry.tabId] = entry;

            if (talentTabCache_.size() <= 10) {
                LOG_INFO("  Tab ", entry.tabId, ": ", entry.name, " (classMask=0x", std::hex, entry.classMask, std::dec, ")");
            }
        }
        LOG_INFO("Loaded ", talentTabCache_.size(), " talent tabs from TalentTab.dbc");
    } else {
        LOG_WARNING("Could not load TalentTab.dbc");
    }

    syncPreWotlkTalentsFromKnownSpells();
}

void SpellHandler::syncPreWotlkTalentsFromKnownSpells() {
    if (!isPreWotlk() || talentCache_.empty() || knownSpells_.empty()) return;

    std::unordered_map<uint32_t, uint8_t> derived;
    uint32_t spentPoints = 0;
    for (const auto& [talentId, talent] : talentCache_) {
        uint8_t rankKnown = 0;
        for (int rank = 0; rank < 5; ++rank) {
            uint32_t rankSpell = talent.rankSpells[rank];
            if (rankSpell != 0 && knownSpells_.count(rankSpell) > 0) {
                rankKnown = static_cast<uint8_t>(rank + 1);
            }
        }
        if (rankKnown > 0) {
            derived[talentId] = rankKnown;
            spentPoints += rankKnown;
        }
    }

    const uint32_t playerLevel = owner_.getPlayerLevel();
    const uint32_t earnedPoints = (playerLevel > 9)
        ? std::min<uint32_t>(playerLevel - 9, 61u)
        : 0u;
    const uint8_t unspent = static_cast<uint8_t>(
        earnedPoints > spentPoints ? std::min<uint32_t>(earnedPoints - spentPoints, 255u) : 0u);

    if (learnedTalents_[0] == derived && activeTalentSpec_ == 0 &&
        unspentTalentPoints_[0] == unspent) {
        return;
    }

    activeTalentSpec_ = 0;
    learnedTalents_[0] = std::move(derived);
    learnedTalents_[1].clear();
    learnedGlyphs_[0].fill(0);
    learnedGlyphs_[1].fill(0);
    unspentTalentPoints_[0] = unspent;
    unspentTalentPoints_[1] = 0;

    LOG_INFO("[pre-WotLK] Derived ", learnedTalents_[0].size(),
             " learned talent(s) from known spells; spent=", spentPoints,
             " unspent=", static_cast<int>(unspent));

    if (owner_.addonEventCallbackRef()) {
        owner_.addonEventCallbackRef()("CHARACTER_POINTS_CHANGED", {});
        owner_.addonEventCallbackRef()("PLAYER_TALENT_UPDATE", {});
    }
}

void SpellHandler::updateTimers(float dt) {
    // Tick down cast bar
    if (casting_ && castTimeRemaining_ > 0.0f) {
        castTimeRemaining_ -= dt;
        if (castTimeRemaining_ < 0.0f) castTimeRemaining_ = 0.0f;
    }
    // Eating/drinking munch-gulp repeats for the whole restoration aura,
    // matching the real client's looping consume sound.
    if (restorationActive_) {
        if (restorationTimeRemaining_ > 0.0f) {
            restorationTimeRemaining_ = std::max(0.0f, restorationTimeRemaining_ - dt);
        }
        restorationSoundTimer_ -= dt;
        if (restorationSoundTimer_ <= 0.0f) {
            restorationSoundTimer_ = 1.0f;
            if (auto* ac = owner_.services().audioCoordinator) {
                if (auto* sfx = ac->getUiSoundManager()) {
                    if (restorationIsFood_) sfx->playEating();
                    else                    sfx->playDrinking();
                }
            }
        }
    }
    // Tick down spell cooldowns
    for (auto it = spellCooldowns_.begin(); it != spellCooldowns_.end(); ) {
        it->second -= dt;
        if (it->second <= 0.0f) {
            it = spellCooldowns_.erase(it);
        } else {
            ++it;
        }
    }
    // Tick down unit cast states
    for (auto it = unitCastStates_.begin(); it != unitCastStates_.end(); ) {
        if (it->second.casting && it->second.timeRemaining > 0.0f) {
            it->second.timeRemaining -= dt;
            if (it->second.timeRemaining <= 0.0f) {
                it->second.timeRemaining = 0.0f;
                it->second.casting = false;
                it = unitCastStates_.erase(it);
                continue;
            }
        }
        ++it;
    }
}

void SpellHandler::stopRestorationPresentation() {
    if (!restorationActive_) return;
    const uint32_t stoppedSpell = restorationSpellId_;
    if (owner_.spellCastAnimCallbackRef()) {
        owner_.spellCastAnimCallbackRef()(owner_.getPlayerGuid(), false, true,
                                          SpellCastType::OMNI);
    }
    restorationActive_ = false;
    restorationSpellId_ = 0;
    restorationTimeRemaining_ = 0.0f;
    restorationTimeTotal_ = 0.0f;
    if (owner_.addonEventCallbackRef()) {
        owner_.addonEventCallbackRef()("UNIT_SPELLCAST_CHANNEL_STOP",
                                       {"player", std::to_string(stoppedSpell)});
    }
}

void SpellHandler::refreshRestorationFromPlayerAuras() {
    owner_.loadSpellNameCache();
    const uint64_t nowMs = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());

    const AuraSlot* restorationAura = nullptr;
    bool restorationIsFood = false;
    for (const auto& aura : playerAuras_) {
        if (aura.isEmpty()) continue;
        auto nameIt = owner_.spellNameCacheRef().find(aura.spellId);
        if (nameIt == owner_.spellNameCacheRef().end()) continue;
        const auto kind = spellclass::classifyRestChannel(nameIt->second.name);
        if (kind == spellclass::RestChannelKind::FOOD ||
            kind == spellclass::RestChannelKind::DRINK) {
            restorationAura = &aura;
            restorationIsFood = kind == spellclass::RestChannelKind::FOOD;
            break;
        }
    }

    if (!restorationAura) {
        stopRestorationPresentation();
        return;
    }

    const uint32_t spellId = restorationAura->spellId;
    const bool newlyActive = !restorationActive_ || restorationSpellId_ != spellId;
    if (restorationActive_ && restorationSpellId_ != spellId) {
        stopRestorationPresentation();
    }

    int32_t totalMs = restorationAura->maxDurationMs;
    if (totalMs <= 0) totalMs = restorationAura->durationMs;
    if (totalMs <= 0) {
        const float dbcSeconds = getSpellDuration(spellId);
        totalMs = dbcSeconds > 0.0f ? static_cast<int32_t>(dbcSeconds * 1000.0f) : 30000;
    }
    int32_t remainingMs = restorationAura->getRemainingMs(nowMs);
    if (remainingMs < 0) {
        remainingMs = (!newlyActive && restorationTimeRemaining_ > 0.0f)
            ? static_cast<int32_t>(restorationTimeRemaining_ * 1000.0f)
            : totalMs;
    }

    restorationActive_ = true;
    restorationSpellId_ = spellId;
    restorationIsFood_ = restorationIsFood;
    restorationTimeTotal_ = static_cast<float>(totalMs) / 1000.0f;
    restorationTimeRemaining_ = static_cast<float>(remainingMs) / 1000.0f;

    if (newlyActive) {
        restorationSoundTimer_ = 0.0f;
        LOG_INFO("Restoration aura started: spellId=", spellId,
                 " durationMs=", totalMs);
        if (owner_.spellCastAnimCallbackRef()) {
            owner_.spellCastAnimCallbackRef()(owner_.getPlayerGuid(), true, true,
                                              SpellCastType::OMNI);
        }
        if (owner_.addonEventCallbackRef()) {
            owner_.addonEventCallbackRef()("UNIT_SPELLCAST_CHANNEL_START",
                                           {"player", std::to_string(spellId)});
        }
    }
}

// ============================================================
// Packet handlers
// ============================================================

void SpellHandler::handleInitialSpells(network::Packet& packet) {
    InitialSpellsData data;
    if (!owner_.getPacketParsers()->parseInitialSpells(packet, data)) return;

    knownSpells_ = {data.spellIds.begin(), data.spellIds.end()};

    LOG_DEBUG("Initial spells include: 527=", knownSpells_.count(527u),
              " 988=", knownSpells_.count(988u), " 1180=", knownSpells_.count(1180u));

    // Ensure Attack (6603) and Hearthstone (8690) are always present
    knownSpells_.insert(6603u);
    knownSpells_.insert(8690u);
    if (isPreWotlk()) {
        loadTalentDbc();
        syncPreWotlkTalentsFromKnownSpells();
    }

    // Set initial cooldowns
    for (const auto& cd : data.cooldowns) {
        uint32_t effectiveMs = std::max(cd.cooldownMs, cd.categoryCooldownMs);
        if (effectiveMs > 0) {
            spellCooldowns_[cd.spellId] = effectiveMs / 1000.0f;
        }
    }

    // Load saved action bar or use defaults
    owner_.actionBarRef()[0].type = ActionBarSlot::SPELL;
    owner_.actionBarRef()[0].id = 6603;  // Attack
    owner_.actionBarRef()[11].type = ActionBarSlot::SPELL;
    owner_.actionBarRef()[11].id = 8690;  // Hearthstone
    owner_.loadCharacterConfig();

    // Sync login-time cooldowns into action bar slot overlays
    for (auto& slot : owner_.actionBarRef()) {
        if (slot.type == ActionBarSlot::SPELL && slot.id != 0) {
            auto it = spellCooldowns_.find(slot.id);
            if (it != spellCooldowns_.end() && it->second > 0.0f) {
                slot.cooldownTotal     = it->second;
                slot.cooldownRemaining = it->second;
            }
        } else if (slot.type == ActionBarSlot::ITEM && slot.id != 0) {
            const auto* qi = owner_.getItemInfo(slot.id);
            if (qi && qi->valid) {
                for (const auto& sp : qi->spells) {
                    if (sp.spellId == 0) continue;
                    auto it = spellCooldowns_.find(sp.spellId);
                    if (it != spellCooldowns_.end() && it->second > 0.0f) {
                        slot.cooldownTotal     = it->second;
                        slot.cooldownRemaining = it->second;
                        break;
                    }
                }
            }
        }
    }

    // Pre-load skill line DBCs
    owner_.loadSkillLineDbc();
    owner_.loadSkillLineAbilityDbc();

    LOG_INFO("Learned ", knownSpells_.size(), " spells");

    if (owner_.addonEventCallbackRef()) {
        owner_.addonEventCallbackRef()("SPELLS_CHANGED", {});
        owner_.addonEventCallbackRef()("LEARNED_SPELL_IN_TAB", {});
    }
}

void SpellHandler::handleCastFailed(network::Packet& packet) {
    CastFailedData data;
    bool ok = owner_.getPacketParsers() ? owner_.getPacketParsers()->parseCastFailed(packet, data)
                                    : CastFailedParser::parse(packet, data);
    if (!ok) return;

    if (spellclass::isFishingCast(data.spellId)) {
        const auto& movement = owner_.movementInfoRef();
        LOG_WARNING("Fishing cast failed: spell=", data.spellId,
                    " result=", static_cast<int>(data.result),
                    " pos=(", movement.x, ",", movement.y, ",", movement.z, ")",
                    " facing=", movement.orientation,
                    " selectedTarget=0x", std::hex, owner_.getTargetGuid(), std::dec);
    }

    const uint64_t gatherGoGuid = owner_.lastInteractedGoGuidRef();
    const bool gatherCast = gatherGoGuid != 0 && isGatherSpellId(data.spellId);

    casting_ = false;
    castIsChannel_ = false;
    currentCastSpellId_ = 0;
    castTimeRemaining_ = 0.0f;
    owner_.lastInteractedGoGuidRef() = 0;
    owner_.pendingGameObjectInteractGuidRef() = 0;
    craftQueueSpellId_ = 0;
    craftQueueRemaining_ = 0;
    // Remove lingering precast visual effects
    if (auto* renderer = owner_.services().renderer) {
        if (auto* svs = renderer->getSpellVisualSystem())
            svs->cancelAllPrecastVisuals();
    }
    queuedSpellId_ = 0;
    queuedSpellTarget_ = 0;

    // Stop precast sound
    if (auto* ac = owner_.services().audioCoordinator) {
        if (auto* ssm = ac->getSpellSoundManager()) {
            ssm->stopPrecast();
        }
    }

    // Show failure reason
    int powerType = -1;
    auto playerEntity = owner_.getEntityManager().getEntity(owner_.getPlayerGuid());
    if (auto playerUnit = std::dynamic_pointer_cast<Unit>(playerEntity)) {
        powerType = playerUnit->getPowerType();
    }
    if (data.result == kSpellFailedNotReady) {
        seedCooldownFromSpellInfo(data.spellId);
    }
    // Totem failures name tool item ids; request their info so a retry of the
    // craft can show the tool's name even if it wasn't cached yet.
    if (data.result == kCastResultTotems) {
        if (data.miscArg != 0) owner_.ensureItemInfo(data.miscArg);
        if (data.miscArg2 != 0) owner_.ensureItemInfo(data.miscArg2);
    }
    std::string errMsg = castFailureMessage(owner_, data.spellId, data.result, powerType,
                                            data.miscArg, data.miscArg2);
    if (gatherCast) {
        errMsg = gatherCastFailureMessage(owner_, gatherGoGuid, data.spellId,
                                          data.result, errMsg);
        if (shouldDespawnGatherTarget(data.result)) {
            owner_.despawnGameObjectLocally(gatherGoGuid);
        }
    }
    owner_.addUIError(errMsg);
    MessageChatData msg;
    msg.type = ChatType::SYSTEM;
    msg.language = ChatLanguage::UNIVERSAL;
    msg.message = errMsg;
    owner_.addLocalChatMessage(msg);

    if (auto* ac = owner_.services().audioCoordinator) {
        if (auto* sfx = ac->getUiSoundManager())
            sfx->playError();
    }

    // Character speech response ("Not enough mana", "I'm out of range", ...)
    // Suppressed for gather casts, whose failures are routine and already rephrased.
    if (!gatherCast) {
        if (auto speech = errorSpeechForCastResult(data.spellId, data.result, powerType))
            owner_.playErrorSpeech(*speech);
    }

    if (owner_.addonEventCallbackRef()) {
        owner_.addonEventCallbackRef()("UNIT_SPELLCAST_FAILED", {"player", std::to_string(data.spellId)});
        owner_.addonEventCallbackRef()("UNIT_SPELLCAST_STOP", {"player", std::to_string(data.spellId)});
    }
    if (owner_.spellCastFailedCallbackRef()) owner_.spellCastFailedCallbackRef()(data.spellId);
}

void SpellHandler::handleSpellStart(network::Packet& packet) {
    SpellStartData data;
    if (!owner_.getPacketParsers()->parseSpellStart(packet, data)) {
        LOG_WARNING("Failed to parse SMSG_SPELL_START, size=", packet.getSize());
        return;
    }
    LOG_DEBUG("SMSG_SPELL_START: caster=0x", std::hex, data.casterUnit, std::dec,
              " spell=", data.spellId, " castTime=", data.castTime,
              " target=0x", std::hex, data.targetGuid, std::dec);

    // Classify spell targeting for animation selection:
    //   DIRECTED — targets a specific other unit (Frostbolt, Heal)
    //   OMNI     — self-cast or no explicit target (Arcane Explosion, buffs)
    //   AREA     — ground-targeted AoE with no unit target (Blizzard, Rain of Fire)
    auto classifyCast = [](uint64_t targetGuid, uint64_t casterGuid) -> SpellCastType {
        if (targetGuid == 0)            return SpellCastType::AREA;
        if (targetGuid == casterGuid)   return SpellCastType::OMNI;
        return SpellCastType::DIRECTED;
    };
    const SpellCastType castType = classifyCast(data.targetGuid, data.casterUnit);
    const bool rangedWeaponAttack = spellclass::isRangedWeaponAutoAttack(data.spellId);

    // Track cast bar for any non-player caster
    if (data.casterUnit != owner_.getPlayerGuid() && data.castTime > 0 && !rangedWeaponAttack) {
        auto& s = unitCastStates_[data.casterUnit];
        s.casting        = true;
        s.isChannel      = false;
        s.spellId        = data.spellId;
        s.timeTotal      = data.castTime / 1000.0f;
        s.timeRemaining  = s.timeTotal;
        s.interruptible  = owner_.isSpellInterruptible(data.spellId);
        s.castType       = castType;
        if (owner_.spellCastAnimCallbackRef()) {
            owner_.spellCastAnimCallbackRef()(data.casterUnit, true, false, castType);
        }
    }

    // Player's own cast
    if (data.casterUnit == owner_.getPlayerGuid() && data.castTime > 0 && !rangedWeaponAttack) {
        // Cancel pending GO retries
        owner_.pendingGameObjectLootRetriesRef().erase(
            std::remove_if(owner_.pendingGameObjectLootRetriesRef().begin(), owner_.pendingGameObjectLootRetriesRef().end(),
                [](const GameHandler::PendingLootRetry&) { return true; }),
            owner_.pendingGameObjectLootRetriesRef().end());

        casting_ = true;
        castIsChannel_ = false;
        currentCastSpellId_ = data.spellId;
        castTimeTotal_ = data.castTime / 1000.0f;
        castTimeRemaining_ = castTimeTotal_;
        if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("CURRENT_SPELL_CAST_CHANGED", {});

        // Play precast sound — skip profession/tradeskill spells
        if (!owner_.isProfessionSpell(data.spellId)) {
            if (auto* ac = owner_.services().audioCoordinator) {
                if (auto* ssm = ac->getSpellSoundManager()) {
                    owner_.loadSpellNameCache();
                    auto it = owner_.spellNameCacheRef().find(data.spellId);
                    auto school = (it != owner_.spellNameCacheRef().end() && it->second.schoolMask)
                        ? schoolMaskToMagicSchool(it->second.schoolMask)
                        : audio::SpellSoundManager::MagicSchool::ARCANE;
                    ssm->playPrecast(school, audio::SpellSoundManager::SpellPower::MEDIUM);
                }
            }
        }

        if (owner_.spellCastAnimCallbackRef()) {
            owner_.spellCastAnimCallbackRef()(owner_.getPlayerGuid(), true, false, castType);
        }

        // Hearthstone: pre-load terrain at bind point
        const bool isHearthstone = (data.spellId == 6948 || data.spellId == 8690);
        if (isHearthstone && owner_.hasHomeBindRef() && owner_.hearthstonePreloadCallbackRef()) {
            owner_.hearthstonePreloadCallbackRef()(owner_.homeBindMapIdRef(), owner_.homeBindPosRef().x, owner_.homeBindPosRef().y, owner_.homeBindPosRef().z);
        }
    }

    // Fire UNIT_SPELLCAST_START
    if (owner_.addonEventCallbackRef() && !rangedWeaponAttack) {
        std::string unitId = owner_.guidToUnitId(data.casterUnit);
        if (!unitId.empty())
            owner_.addonEventCallbackRef()("UNIT_SPELLCAST_START", {unitId, std::to_string(data.spellId)});
    }

    // Trigger cast visual effect (precast/cast kit M2) at the caster's position.
    // Skip profession spells (crafting has no flashy cast effects).
    if (!owner_.isProfessionSpell(data.spellId) && !rangedWeaponAttack) {
        triggerCastVisual(data.spellId, data.casterUnit, data.castTime);
    }
}

void SpellHandler::handleSpellGo(network::Packet& packet) {
    SpellGoData data;
    if (!owner_.getPacketParsers()->parseSpellGo(packet, data)) return;
    const bool rangedWeaponAttack = spellclass::isRangedWeaponAutoAttack(data.spellId);

    if (data.casterUnit == owner_.getPlayerGuid()) {
        owner_.loadSpellNameCache();
        spellclass::RestChannelKind restKind = spellclass::RestChannelKind::NONE;
        auto spellNameIt = owner_.spellNameCacheRef().find(data.spellId);
        if (spellNameIt != owner_.spellNameCacheRef().end()) {
            restKind = spellclass::classifyRestChannel(spellNameIt->second.name);
            if (spellclass::hasInebriateEffect(spellNameIt->second.effectIds, 3)) {
                restKind = spellclass::RestChannelKind::ALCOHOL;
            }
        }
        const bool restorationLoop = restKind == spellclass::RestChannelKind::FOOD ||
                                     restKind == spellclass::RestChannelKind::DRINK;
        // The real client sits itself when consuming food/water. Servers apply
        // the regen aura with AURA_INTERRUPT_FLAG_NOT_SEATED and remove it
        // immediately unless the client sends CMSG_STANDSTATECHANGE(SIT) —
        // without this the whole eat/drink presentation never starts.
        if (restorationLoop) {
            owner_.setStandState(1); // UNIT_STAND_STATE_SIT
        }
        // Play cast-complete sound
        if (!owner_.isProfessionSpell(data.spellId) && !rangedWeaponAttack)
            playSpellCastSound(data.spellId);

        // Ranged auto-attack spells (Auto Shot, Shoot, Throw) complete as timed
        // casts and are NOT classified as instant melee abilities, so trigger the
        // ranged shot animation explicitly here.
        uint32_t sid = data.spellId;
        if (spellclass::isRangedWeaponAutoAttack(sid)) {
            if (owner_.meleeSwingCallbackRef()) owner_.meleeSwingCallbackRef()(sid);
            owner_.suppressNextMeleeSwingAnim();
            uint64_t projectileTarget = data.targetGuid;
            if (projectileTarget == 0 && !data.hitTargets.empty())
                projectileTarget = data.hitTargets.front();
            if (projectileTarget == 0 && !data.missTargets.empty())
                projectileTarget = data.missTargets.front().targetGuid;
            launchRangedWeaponProjectile(sid, projectileTarget);
        }

        // Instant melee abilities → trigger attack animation. Range decides this, not
        // school: physical abilities cast at range (Steady Shot, Taunt) would otherwise
        // play a melee swing.
        bool isMeleeAbility = false;
        if (!owner_.isProfessionSpell(sid) && !spellclass::isRangedWeaponAutoAttack(sid)) {
            if (spellclass::isMeleeRange(getSpellMaxRange(sid))) {
                isMeleeAbility = (currentCastSpellId_ != sid);
            }
        }
        if (isMeleeAbility) {
            if (owner_.meleeSwingCallbackRef()) owner_.meleeSwingCallbackRef()(sid);
            if (auto* ac = owner_.services().audioCoordinator) {
                if (auto* csm = ac->getCombatSoundManager()) {
                    csm->playWeaponSwing(audio::CombatSoundManager::WeaponSize::MEDIUM, false);
                    csm->playImpact(audio::CombatSoundManager::WeaponSize::MEDIUM,
                                    audio::CombatSoundManager::ImpactType::FLESH, false);
                }
            }
        }

        const bool wasInTimedCast = casting_ && (data.spellId == currentCastSpellId_);

        // Instant spell cast animation — if this wasn't a timed cast and isn't a
        // melee ability, play a brief spell cast animation (one-shot)
        if (!wasInTimedCast && !isMeleeAbility && !rangedWeaponAttack && !restorationLoop &&
            !owner_.isProfessionSpell(data.spellId)) {
            // Classify instant spell from SPELL_GO packet target info
            SpellCastType goType = SpellCastType::OMNI;
            if (data.targetGuid != 0 && data.targetGuid != data.casterUnit)
                goType = SpellCastType::DIRECTED;
            else if (data.targetGuid == 0 && data.hitCount > 1)
                goType = SpellCastType::AREA;
            if (owner_.spellCastAnimCallbackRef()) {
                // Instant item spells have no SMSG_SPELL_START, so publish the
                // SPELL_GO id just long enough for the animation callback to
                // distinguish potions from generic instant magic.
                currentCastSpellId_ = data.spellId;
                owner_.spellCastAnimCallbackRef()(owner_.getPlayerGuid(), true, false, goType);
            }
        }

        LOG_DEBUG("[GO-DIAG] SPELL_GO: spellId=", data.spellId,
                    " casting=", casting_, " currentCast=", currentCastSpellId_,
                    " wasInTimedCast=", wasInTimedCast,
                    " lastGoGuid=0x", std::hex, owner_.lastInteractedGoGuidRef(),
                    " pendingGoGuid=0x", owner_.pendingGameObjectInteractGuidRef(), std::dec);

        casting_ = false;
        castIsChannel_ = false;
        currentCastSpellId_ = 0;
        castTimeRemaining_ = 0.0f;
        castTimeTotal_ = 0.0f;

        // Gather node looting: re-send CMSG_LOOT now that the cast completed.
        if (wasInTimedCast && owner_.lastInteractedGoGuidRef() != 0) {
            LOG_DEBUG("[GO-DIAG] Sending CMSG_LOOT for GO 0x", std::hex,
                        owner_.lastInteractedGoGuidRef(), std::dec);
            owner_.lootTarget(owner_.lastInteractedGoGuidRef());
        }
        // Clear the GO interaction guard so future cancelCast() calls work
        // normally. Without this, pendingGameObjectInteractGuid_ stays stale
        // and suppresses CMSG_CANCEL_CAST for ALL subsequent spell casts.
        owner_.pendingGameObjectInteractGuidRef() = 0;

        if (owner_.spellCastAnimCallbackRef() && !rangedWeaponAttack && !restorationLoop) {
            owner_.spellCastAnimCallbackRef()(owner_.getPlayerGuid(), false, false, SpellCastType::OMNI);
        }

        if (owner_.addonEventCallbackRef() && !rangedWeaponAttack)
            owner_.addonEventCallbackRef()("UNIT_SPELLCAST_STOP", {"player", std::to_string(data.spellId)});

        // Craft queue: re-cast if more crafts remaining
        if (craftQueueRemaining_ > 0 && craftQueueSpellId_ == data.spellId) {
            --craftQueueRemaining_;
            if (craftQueueRemaining_ > 0) {
                LOG_INFO("Craft queue: re-casting spell=", craftQueueSpellId_,
                         " remaining=", craftQueueRemaining_);
                castSpell(craftQueueSpellId_, 0);
            } else {
                craftQueueSpellId_ = 0;
            }
        }
        // Spell queue: fire the next queued spell
        else if (queuedSpellId_ != 0) {
            uint32_t nextSpell  = queuedSpellId_;
            uint64_t nextTarget = queuedSpellTarget_;
            queuedSpellId_     = 0;
            queuedSpellTarget_ = 0;
            LOG_INFO("Spell queue: firing queued spellId=", nextSpell);
            castSpell(nextSpell, nextTarget);
        }
    } else {
        // For non-player casters: if no tracked cast state exists, this was an
        // instant cast — play a brief one-shot spell animation before stopping
        auto castIt = unitCastStates_.find(data.casterUnit);
        bool wasTrackedCast = (castIt != unitCastStates_.end());
        // Classify NPC instant spell from SPELL_GO target info
        SpellCastType npcGoType = SpellCastType::OMNI;
        if (data.targetGuid != 0 && data.targetGuid != data.casterUnit)
            npcGoType = SpellCastType::DIRECTED;
        else if (data.targetGuid == 0 && data.hitCount > 1)
            npcGoType = SpellCastType::AREA;
        if (!wasTrackedCast && !rangedWeaponAttack && owner_.spellCastAnimCallbackRef()) {
            owner_.spellCastAnimCallbackRef()(data.casterUnit, true, false, npcGoType);
        }
        if (!rangedWeaponAttack && owner_.spellCastAnimCallbackRef()) {
            owner_.spellCastAnimCallbackRef()(data.casterUnit, false, false, SpellCastType::OMNI);
        }
        bool targetsPlayer = false;
        for (const auto& tgt : data.hitTargets) {
            if (tgt == owner_.getPlayerGuid()) { targetsPlayer = true; break; }
        }
        if (targetsPlayer && !rangedWeaponAttack)
            playSpellCastSound(data.spellId);
    }

    // Clear unit cast bar
    unitCastStates_.erase(data.casterUnit);

    // Miss combat text
    if (!data.missTargets.empty()) {
        const uint64_t spellCasterGuid = data.casterUnit != 0 ? data.casterUnit : data.casterGuid;
        const bool playerIsCaster = (spellCasterGuid == owner_.getPlayerGuid());

        for (const auto& m : data.missTargets) {
            if (!playerIsCaster && m.targetGuid != owner_.getPlayerGuid()) {
                continue;
            }
            CombatTextEntry::Type ct = combatTextTypeFromSpellMissInfo(m.missType);
            owner_.addCombatText(ct, 0, data.spellId, playerIsCaster, 0, spellCasterGuid, m.targetGuid);
        }
    }

    // Impact sound
    bool playerIsHit = false;
    bool playerHitEnemy = false;
    for (const auto& tgt : data.hitTargets) {
        if (tgt == owner_.getPlayerGuid()) { playerIsHit = true; }
        if (data.casterUnit == owner_.getPlayerGuid() && tgt != owner_.getPlayerGuid() && tgt != 0) { playerHitEnemy = true; }
    }

    // Fire UNIT_SPELLCAST_SUCCEEDED
    if (owner_.addonEventCallbackRef()) {
        std::string unitId = owner_.guidToUnitId(data.casterUnit);
        if (!unitId.empty())
            owner_.addonEventCallbackRef()("UNIT_SPELLCAST_SUCCEEDED", {unitId, std::to_string(data.spellId)});
    }

    if ((playerIsHit || playerHitEnemy) && !rangedWeaponAttack)
        playSpellImpactSound(data.spellId);

    // Trigger spell visual effects: cast kit at caster + impact kit at each hit target.
    // Skip profession spells and melee (schoolMask == 1) abilities.
    if (!owner_.isProfessionSpell(data.spellId) && !rangedWeaponAttack) {
        uint32_t visualId = resolveSpellVisualId(data.spellId);
        if (visualId != 0) {
            // Cast-complete visual at caster (for instant spells that skip SPELL_START)
            glm::vec3 casterPos;
            if (resolveUnitPosition(data.casterUnit, casterPos)) {
                if (auto* renderer = owner_.services().renderer) {
                    if (auto* svs = renderer->getSpellVisualSystem()) {
                        svs->playSpellVisual(visualId, casterPos, /*useImpactKit=*/false,
                                             owner_.resolveUnitRenderInstance(data.casterUnit));
                    }
                }
            }
            // Impact visual at each hit target
            for (const auto& tgt : data.hitTargets) {
                if (tgt != 0) {
                    triggerImpactVisual(data.spellId, tgt);
                }
            }
        }
    }
}

void SpellHandler::handleSpellCooldown(network::Packet& packet) {
    const bool isClassicFormat = isClassicLikeExpansion();

    if (!packet.hasRemaining(8)) return;
    /*guid*/ packet.readUInt64();

    if (!isClassicFormat) {
        if (!packet.hasRemaining(1)) return;
        /*flags*/ packet.readUInt8();
    }

    const size_t entrySize = isClassicFormat ? 12u : 8u;
    while (packet.getRemainingSize() >= entrySize) {
        uint32_t spellId    = packet.readUInt32();
        uint32_t cdItemId   = 0;
        if (isClassicFormat) cdItemId = packet.readUInt32();
        uint32_t cooldownMs = packet.readUInt32();

        float seconds = cooldownMs / 1000.0f;

        // spellId=0 is the Global Cooldown marker
        if (spellId == 0 && cooldownMs > 0 && cooldownMs <= 2000) {
            gcdTotal_ = seconds;
            gcdStartedAt_ = std::chrono::steady_clock::now();
            continue;
        }

        auto it = spellCooldowns_.find(spellId);
        if (it == spellCooldowns_.end()) {
            spellCooldowns_[spellId] = seconds;
        } else {
            it->second = mergeCooldownSeconds(it->second, seconds);
        }
        for (auto& slot : owner_.actionBarRef()) {
            bool match = (slot.type == ActionBarSlot::SPELL && slot.id == spellId)
                      || (cdItemId != 0 && slot.type == ActionBarSlot::ITEM && slot.id == cdItemId);
            if (match) {
                float prevRemaining = slot.cooldownRemaining;
                float merged = mergeCooldownSeconds(slot.cooldownRemaining, seconds);
                slot.cooldownRemaining = merged;
                if (slot.cooldownTotal <= 0.0f || prevRemaining <= 0.0f) {
                    slot.cooldownTotal = seconds;
                } else {
                    slot.cooldownTotal = std::max(slot.cooldownTotal, merged);
                }
            }
        }
    }
    LOG_DEBUG("handleSpellCooldown: parsed for ",
              isClassicFormat ? "Classic" : "TBC/WotLK", " format");
    if (owner_.addonEventCallbackRef()) {
        owner_.addonEventCallbackRef()("SPELL_UPDATE_COOLDOWN", {});
        owner_.addonEventCallbackRef()("ACTIONBAR_UPDATE_COOLDOWN", {});
    }
}

void SpellHandler::handleCooldownEvent(network::Packet& packet) {
    if (!packet.hasRemaining(4)) return;
    uint32_t spellId = packet.readUInt32();
    if (packet.hasRemaining(8))
        packet.readUInt64();
    spellCooldowns_.erase(spellId);
    for (auto& slot : owner_.actionBarRef()) {
        if (slot.type == ActionBarSlot::SPELL && slot.id == spellId) {
            slot.cooldownRemaining = 0.0f;
        }
    }
    if (owner_.addonEventCallbackRef()) {
        owner_.addonEventCallbackRef()("SPELL_UPDATE_COOLDOWN", {});
        owner_.addonEventCallbackRef()("ACTIONBAR_UPDATE_COOLDOWN", {});
    }
}

void SpellHandler::handleAuraUpdate(network::Packet& packet, bool isAll) {
    AuraUpdateData data;
    if (!owner_.getPacketParsers()->parseAuraUpdate(packet, data, isAll)) return;

    std::vector<AuraSlot>* auraList = nullptr;
    if (data.guid == owner_.getPlayerGuid()) {
        auraList = &playerAuras_;
    } else if (data.guid == owner_.getTargetGuid()) {
        auraList = &targetAuras_;
    }
    if (data.guid != 0 && data.guid != owner_.getPlayerGuid() && data.guid != owner_.getTargetGuid()) {
        auraList = &unitAurasCache_[data.guid];
    }

    if (auraList) {
        if (isAll) {
            auraList->clear();
        }
        uint64_t nowMs = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
        // Grow once to fit the highest slot, instead of push_back-in-a-loop per update.
        if (!data.updates.empty()) {
            size_t maxSlot = 0;
            for (const auto& [slot, aura] : data.updates) {
                if (slot > maxSlot) maxSlot = slot;
            }
            if (auraList->size() <= maxSlot) {
                auraList->resize(maxSlot + 1);
            }
        }
        for (auto [slot, aura] : data.updates) {
            if (aura.durationMs >= 0) {
                aura.receivedAtMs = nowMs;
            }
            (*auraList)[slot] = aura;
        }

        if (owner_.addonEventCallbackRef()) {
            std::string unitId;
            if (data.guid == owner_.getPlayerGuid()) unitId = "player";
            else if (data.guid == owner_.getTargetGuid()) unitId = "target";
            else if (data.guid == owner_.focusGuidRef()) unitId = "focus";
            else if (data.guid == owner_.petGuidRef()) unitId = "pet";
            if (!unitId.empty())
                owner_.addonEventCallbackRef()("UNIT_AURA", {unitId});
        }

        // Mount aura detection
        if (data.guid == owner_.getPlayerGuid() && owner_.currentMountDisplayIdRef() != 0 && owner_.mountAuraSpellIdRef() == 0) {
            for (const auto& [slot, aura] : data.updates) {
                if (!aura.isEmpty() && aura.maxDurationMs < 0 && aura.casterGuid == owner_.getPlayerGuid()) {
                    owner_.mountAuraSpellIdRef() = aura.spellId;
                    LOG_INFO("Mount aura detected from aura update: spellId=", aura.spellId);
                }
            }
        }

        // Sprint aura detection — check if any sprint/dash speed buff is active
        if (data.guid == owner_.getPlayerGuid() && owner_.sprintAuraCallbackRef()) {
            static constexpr uint32_t sprintSpells[] = {
                2983, 8696, 11305,   // Rogue Sprint (ranks 1-3)
                1850, 9821, 33357,   // Druid Dash (ranks 1-3)
                36554,               // Shadowstep (speed component)
                68992, 68991,        // Darkflight (worgen racial)
                58984,               // Aspect of the Pack speed 
            };
            bool hasSprint = false;
            for (const auto& a : playerAuras_) {
                if (a.isEmpty()) continue;
                for (uint32_t sid : sprintSpells) {
                    if (a.spellId == sid) { hasSprint = true; break; }
                }
                if (hasSprint) break;
            }
            owner_.sprintAuraCallbackRef()(hasSprint);
        }

        if (data.guid == owner_.getPlayerGuid()) {
            refreshRestorationFromPlayerAuras();
        }
    }
}

void SpellHandler::handleLearnedSpell(network::Packet& packet) {
    const bool classicSpellId = isClassicLikeExpansion();
    const size_t minSz = classicSpellId ? 2u : 4u;
    if (packet.getRemainingSize() < minSz) return;
    uint32_t spellId = classicSpellId ? packet.readUInt16() : packet.readUInt32();

    const bool alreadyKnown = knownSpells_.count(spellId) > 0;
    knownSpells_.insert(spellId);
    LOG_INFO("Learned spell: ", spellId, alreadyKnown ? " (already known, skipping chat)" : "");
    if (isPreWotlk()) {
        loadTalentDbc();
        syncPreWotlkTalentsFromKnownSpells();
    }

    // Check if this spell corresponds to a talent rank
    bool isTalentSpell = false;
    for (const auto& [talentId, talent] : talentCache_) {
        for (int rank = 0; rank < 5; ++rank) {
            if (talent.rankSpells[rank] == spellId) {
                uint8_t newRank = rank + 1;
                learnedTalents_[activeTalentSpec_][talentId] = newRank;
                LOG_INFO("Talent learned: id=", talentId, " rank=", (int)newRank,
                         " (spell ", spellId, ") in spec ", (int)activeTalentSpec_);
                isTalentSpell = true;
                if (owner_.addonEventCallbackRef()) {
                    owner_.addonEventCallbackRef()("CHARACTER_POINTS_CHANGED", {});
                    owner_.addonEventCallbackRef()("PLAYER_TALENT_UPDATE", {});
                }
                break;
            }
        }
        if (isTalentSpell) break;
    }

    if (!alreadyKnown && owner_.addonEventCallbackRef()) {
        owner_.addonEventCallbackRef()("LEARNED_SPELL_IN_TAB", {std::to_string(spellId)});
        owner_.addonEventCallbackRef()("SPELLS_CHANGED", {});
    }

    if (isTalentSpell) return;

    if (!alreadyKnown) {
        const std::string& name = owner_.getSpellName(spellId);
        if (!name.empty()) {
            owner_.addSystemChatMessage("You have learned a new spell: " + name + ".");
        } else {
            owner_.addSystemChatMessage("You have learned a new spell.");
        }
    }
}

void SpellHandler::handleRemovedSpell(network::Packet& packet) {
    const bool classicSpellId = isClassicLikeExpansion();
    const size_t minSz = classicSpellId ? 2u : 4u;
    if (packet.getRemainingSize() < minSz) return;
    uint32_t spellId = classicSpellId ? packet.readUInt16() : packet.readUInt32();
    knownSpells_.erase(spellId);
    syncPreWotlkTalentsFromKnownSpells();
    LOG_INFO("Removed spell: ", spellId);
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("SPELLS_CHANGED", {});

    // Learning a new talent rank legitimately removes/replaces its internal
    // talent spell. That is rank bookkeeping, not the player unlearning the
    // talent, and presenting it as "You have unlearned" is backwards.
    loadTalentDbc();
    bool isTalentRankSpell = false;
    for (const auto& [talentId, talent] : talentCache_) {
        (void)talentId;
        for (uint32_t rankSpell : talent.rankSpells) {
            if (rankSpell == spellId) {
                isTalentRankSpell = true;
                break;
            }
        }
        if (isTalentRankSpell) break;
    }

    if (!isTalentRankSpell) {
        const std::string& name = owner_.getSpellName(spellId);
        if (!name.empty())
            owner_.addSystemChatMessage("You have unlearned: " + name + ".");
        else
            owner_.addSystemChatMessage("A spell has been removed.");
    } else {
        LOG_DEBUG("Suppressed talent-rank removal chat for spell ", spellId);
    }

    bool barChanged = false;
    for (auto& slot : owner_.actionBarRef()) {
        if (slot.type == ActionBarSlot::SPELL && slot.id == spellId) {
            slot = ActionBarSlot{};
            barChanged = true;
        }
    }
    if (barChanged) owner_.saveCharacterConfig();
}

void SpellHandler::handleSupercededSpell(network::Packet& packet) {
    const bool classicSpellId = isClassicLikeExpansion();
    const size_t minSz = classicSpellId ? 4u : 8u;
    if (packet.getRemainingSize() < minSz) return;
    uint32_t oldSpellId = classicSpellId ? packet.readUInt16() : packet.readUInt32();
    uint32_t newSpellId = classicSpellId ? packet.readUInt16() : packet.readUInt32();

    knownSpells_.erase(oldSpellId);

    const bool newSpellAlreadyAnnounced = knownSpells_.count(newSpellId) > 0;

    knownSpells_.insert(newSpellId);
    syncPreWotlkTalentsFromKnownSpells();

    LOG_INFO("Spell superceded: ", oldSpellId, " -> ", newSpellId);

    bool barChanged = false;
    for (auto& slot : owner_.actionBarRef()) {
        if (slot.type == ActionBarSlot::SPELL && slot.id == oldSpellId) {
            slot.id = newSpellId;
            slot.cooldownRemaining = 0.0f;
            slot.cooldownTotal = 0.0f;
            barChanged = true;
            LOG_DEBUG("Action bar slot upgraded: spell ", oldSpellId, " -> ", newSpellId);
        }
    }
    if (barChanged) {
        owner_.saveCharacterConfig();
        if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("ACTIONBAR_SLOT_CHANGED", {});
    }

    if (!newSpellAlreadyAnnounced) {
        const std::string& newName = owner_.getSpellName(newSpellId);
        if (!newName.empty()) {
            owner_.addSystemChatMessage("Upgraded to " + newName);
        }
    }
}

void SpellHandler::handleUnlearnSpells(network::Packet& packet) {
    if (!packet.hasRemaining(4)) return;
    uint32_t spellCount = packet.readUInt32();
    LOG_INFO("Unlearning ", spellCount, " spells");

    bool barChanged = false;
    for (uint32_t i = 0; i < spellCount && packet.getRemainingSize() >= 4; ++i) {
        uint32_t spellId = packet.readUInt32();
        knownSpells_.erase(spellId);
        LOG_INFO("  Unlearned spell: ", spellId);
        for (auto& slot : owner_.actionBarRef()) {
            if (slot.type == ActionBarSlot::SPELL && slot.id == spellId) {
                slot = ActionBarSlot{};
                barChanged = true;
            }
        }
    }
    if (barChanged) owner_.saveCharacterConfig();

    if (spellCount > 0) {
        owner_.addSystemChatMessage("Unlearned " + std::to_string(spellCount) + " spells");
    }
}

void SpellHandler::handleTalentsInfo(network::Packet& packet) {
    if (!packet.hasRemaining(1)) return;
    uint8_t talentType = packet.readUInt8();
    if (talentType != 0) {
        return;
    }
    if (!packet.hasRemaining(6)) {
        LOG_WARNING("handleTalentsInfo: packet too short for header");
        return;
    }

    uint32_t unspentTalents    = packet.readUInt32();
    uint8_t  talentGroupCount  = packet.readUInt8();
    uint8_t  activeTalentGroup = packet.readUInt8();
    if (activeTalentGroup > 1) activeTalentGroup = 0;

    loadTalentDbc();

    activeTalentSpec_ = activeTalentGroup;

    for (uint8_t g = 0; g < talentGroupCount && g < 2; ++g) {
        if (!packet.hasRemaining(1)) break;
        uint8_t talentCount = packet.readUInt8();
        learnedTalents_[g].clear();
        for (uint8_t t = 0; t < talentCount; ++t) {
            if (!packet.hasRemaining(5)) break;
            uint32_t talentId = packet.readUInt32();
            uint8_t  rank     = packet.readUInt8();
            learnedTalents_[g][talentId] = rank + 1u;
        }
        learnedGlyphs_[g].fill(0);
        if (!packet.hasRemaining(1)) break;
        uint8_t glyphCount = packet.readUInt8();
        for (uint8_t gl = 0; gl < glyphCount; ++gl) {
            if (!packet.hasRemaining(2)) break;
            uint16_t glyphId = packet.readUInt16();
            if (gl < MAX_GLYPH_SLOTS) learnedGlyphs_[g][gl] = glyphId;
        }
    }

    unspentTalentPoints_[activeTalentGroup] =
        static_cast<uint8_t>(unspentTalents > 255 ? 255 : unspentTalents);

    LOG_INFO("handleTalentsInfo: unspent=", unspentTalents,
             " groups=", (int)talentGroupCount, " active=", (int)activeTalentGroup,
             " learned=", learnedTalents_[activeTalentGroup].size());

    if (owner_.addonEventCallbackRef()) {
        owner_.addonEventCallbackRef()("CHARACTER_POINTS_CHANGED", {});
        owner_.addonEventCallbackRef()("ACTIVE_TALENT_GROUP_CHANGED", {});
        owner_.addonEventCallbackRef()("PLAYER_TALENT_UPDATE", {});
    }

    if (!talentsInitialized_) {
        talentsInitialized_ = true;
        if (unspentTalents > 0) {
            owner_.addSystemChatMessage("You have " + std::to_string(unspentTalents)
                + " unspent talent point" + (unspentTalents != 1 ? "s" : "") + ".");
        }
    }
}

void SpellHandler::handleAchievementEarned(network::Packet& packet) {
    // WotLK SMSG_ACHIEVEMENT_EARNED: packGUID player + uint32 achievementId + packedTime.
    // The player GUID is a PACKED guid, not a full uint64 — reading it as uint64 swallowed
    // 4 bytes of the achievement id (showing the raw guid + a garbage id), so decode the
    // packed guid and the fixed fields follow at the correct offset.
    if (!packet.hasFullPackedGuid()) return;
    uint64_t guid = packet.readPackedGuid();
    if (!packet.hasRemaining(8)) return;  // achievementId(4) + packedTime(4)
    uint32_t achievementId = packet.readUInt32();
    uint32_t earnDate      = packet.readUInt32();

    owner_.loadAchievementNameCache();
    auto nameIt = owner_.achievementNameCacheRef().find(achievementId);
    const std::string& achName = (nameIt != owner_.achievementNameCacheRef().end())
        ? nameIt->second : std::string();

    bool isSelf = (guid == owner_.getPlayerGuid());
    if (isSelf) {
        char buf[256];
        if (!achName.empty()) {
            std::snprintf(buf, sizeof(buf), "Achievement earned: %s", achName.c_str());
        } else {
            std::snprintf(buf, sizeof(buf), "Achievement earned! (ID %u)", achievementId);
        }
        owner_.addSystemChatMessage(buf);

        owner_.earnedAchievementsRef().insert(achievementId);
        owner_.achievementDatesRef()[achievementId] = earnDate;
        if (auto* ac = owner_.services().audioCoordinator) {
            if (auto* sfx = ac->getUiSoundManager())
                sfx->playAchievementAlert();
        }
        if (owner_.achievementEarnedCallbackRef()) {
            owner_.achievementEarnedCallbackRef()(achievementId, achName);
        }
    } else {
        std::string senderName;
        auto entity = owner_.getEntityManager().getEntity(guid);
        if (auto* unit = dynamic_cast<Unit*>(entity.get())) {
            senderName = unit->getName();
        }
        if (senderName.empty()) {
            auto nit = owner_.getPlayerNameCache().find(guid);
            if (nit != owner_.getPlayerNameCache().end())
                senderName = nit->second;
        }
        if (senderName.empty()) {
            char tmp[32];
            std::snprintf(tmp, sizeof(tmp), "0x%llX",
                          static_cast<unsigned long long>(guid));
            senderName = tmp;
        }
        // Use std::string instead of fixed char[256] — achievement names can be
        // long and combined with senderName could exceed 256 bytes, silently truncating.
        std::string msg = senderName + (!achName.empty()
            ? " has earned the achievement: " + achName
            : " has earned an achievement! (ID " + std::to_string(achievementId) + ")");
        owner_.addSystemChatMessage(msg);
    }

    LOG_INFO("SMSG_ACHIEVEMENT_EARNED: guid=0x", std::hex, guid, std::dec,
             " achievementId=", achievementId, " self=", isSelf,
             achName.empty() ? "" : " name=", achName);
    if (owner_.addonEventCallbackRef())
        owner_.addonEventCallbackRef()("ACHIEVEMENT_EARNED", {std::to_string(achievementId)});
}

// SMSG_EQUIPMENT_SET_LIST — moved to InventoryHandler

// ============================================================
// Pet spell methods (moved from GameHandler)
// ============================================================

void SpellHandler::handlePetSpells(network::Packet& packet) {
    const size_t remaining = packet.getRemainingSize();
    if (remaining < 8) {
        owner_.petGuidRef() = 0;
        owner_.petSpellListRef().clear();
        owner_.petAutocastSpellsRef().clear();
        memset(owner_.petActionSlotsRef(), 0, sizeof(owner_.petActionSlotsRef()));
        LOG_INFO("SMSG_PET_SPELLS: pet cleared");
        owner_.fireAddonEvent("UNIT_PET", {"player"});
        return;
    }

    owner_.petGuidRef() = packet.readUInt64();
    if (owner_.petGuidRef() == 0) {
        owner_.petSpellListRef().clear();
        owner_.petAutocastSpellsRef().clear();
        memset(owner_.petActionSlotsRef(), 0, sizeof(owner_.petActionSlotsRef()));
        LOG_INFO("SMSG_PET_SPELLS: pet cleared (guid=0)");
        owner_.fireAddonEvent("UNIT_PET", {"player"});
        return;
    }

    // Parse optional pet fields — bail on truncated packets but always log+fire below.
    do {
        if (!packet.hasRemaining(4)) break;
        /*uint16_t dur =*/ packet.readUInt16();
        /*uint16_t timer =*/ packet.readUInt16();

        if (!packet.hasRemaining(2)) break;
        owner_.petReactRef()   = packet.readUInt8();
        owner_.petCommandRef() = packet.readUInt8();

        if (!packet.hasRemaining(GameHandler::PET_ACTION_BAR_SLOTS * 4u)) break;
        for (int i = 0; i < GameHandler::PET_ACTION_BAR_SLOTS; ++i) {
            owner_.petActionSlotsRef()[i] = packet.readUInt32();
        }

        if (!packet.hasRemaining(1)) break;
        uint8_t spellCount = packet.readUInt8();
        owner_.petSpellListRef().clear();
        owner_.petAutocastSpellsRef().clear();
        for (uint8_t i = 0; i < spellCount; ++i) {
            if (!packet.hasRemaining(6)) break;
            uint32_t spellId = packet.readUInt32();
            uint16_t activeFlags = packet.readUInt16();
            owner_.petSpellListRef().push_back(spellId);
            if (activeFlags & 0x0001) {
                owner_.petAutocastSpellsRef().insert(spellId);
            }
        }
    } while (false);

    LOG_INFO("SMSG_PET_SPELLS: petGuid=0x", std::hex, owner_.petGuidRef(), std::dec,
             " react=", static_cast<int>(owner_.petReactRef()), " command=", static_cast<int>(owner_.petCommandRef()),
             " spells=", owner_.petSpellListRef().size());
    owner_.fireAddonEvent("UNIT_PET", {"player"});
    owner_.fireAddonEvent("PET_BAR_UPDATE", {});
}

void SpellHandler::sendPetAction(uint32_t action, uint64_t targetGuid) {
    if (!owner_.hasPet() || owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto pkt = PetActionPacket::build(owner_.petGuidRef(), action, targetGuid);
    owner_.getSocket()->send(pkt);
    LOG_DEBUG("sendPetAction: petGuid=0x", std::hex, owner_.petGuidRef(),
              " action=0x", action, " target=0x", targetGuid, std::dec);
}

void SpellHandler::dismissPet() {
    if (owner_.petGuidRef() == 0 || owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    // Dismiss is COMMAND_ABANDON. Packing action 0 here sent COMMAND_STAY, so
    // the pet planted itself instead of leaving.
    auto packet = PetActionPacket::build(
        owner_.petGuidRef(), pet::packPetAction(pet::ActionType::Command, pet::kAbandon));
    owner_.getSocket()->send(packet);
}

void SpellHandler::togglePetSpellAutocast(uint32_t spellId) {
    if (owner_.petGuidRef() == 0 || spellId == 0 || owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    bool currentlyOn = owner_.petAutocastSpellsRef().count(spellId) != 0;
    uint8_t newState = currentlyOn ? 0 : 1;
    network::Packet pkt(wireOpcode(Opcode::CMSG_PET_SPELL_AUTOCAST));
    pkt.writeUInt64(owner_.petGuidRef());
    pkt.writeUInt32(spellId);
    pkt.writeUInt8(newState);
    owner_.getSocket()->send(pkt);
    if (newState)
        owner_.petAutocastSpellsRef().insert(spellId);
    else
        owner_.petAutocastSpellsRef().erase(spellId);
    LOG_DEBUG("togglePetSpellAutocast: spellId=", spellId, " autocast=", static_cast<int>(newState));
}

void SpellHandler::renamePet(const std::string& newName) {
    if (owner_.petGuidRef() == 0 || owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    if (newName.empty() || newName.size() > 12) return;
    auto packet = PetRenamePacket::build(owner_.petGuidRef(), newName, 0);
    owner_.getSocket()->send(packet);
    LOG_INFO("Sent CMSG_PET_RENAME: petGuid=0x", std::hex, owner_.petGuidRef(), std::dec, " name='", newName, "'");
}

void SpellHandler::handleListStabledPets(network::Packet& packet) {
    constexpr size_t kMinHeader = 8 + 1 + 1;
    if (!packet.hasRemaining(kMinHeader)) {
        LOG_WARNING("MSG_LIST_STABLED_PETS: packet too short (", packet.getSize(), ")");
        return;
    }
    owner_.stableMasterGuidRef() = packet.readUInt64();
    uint8_t petCount  = packet.readUInt8();
    owner_.stableNumSlotsRef()   = packet.readUInt8();

    owner_.stabledPetsRef().clear();
    owner_.stabledPetsRef().reserve(petCount);

    for (uint8_t i = 0; i < petCount; ++i) {
        // petNumber(4) + entry(4) + level(4) = 12 bytes before the name string
        if (!packet.hasRemaining(12)) break;
        GameHandler::StabledPet pet;
        pet.petNumber = packet.readUInt32();
        pet.entry     = packet.readUInt32();
        pet.level     = packet.readUInt32();
        pet.name      = packet.readString();
        // displayId(4) + isActive(1) = 5 bytes after the name string
        if (!packet.hasRemaining(5)) break;
        pet.displayId = packet.readUInt32();
        pet.isActive  = (packet.readUInt8() != 0);
        owner_.stabledPetsRef().push_back(std::move(pet));
    }

    owner_.stableWindowOpenRef() = true;
    LOG_INFO("MSG_LIST_STABLED_PETS: stableMasterGuid=0x", std::hex, owner_.stableMasterGuidRef(), std::dec,
             " petCount=", static_cast<int>(petCount), " numSlots=", static_cast<int>(owner_.stableNumSlotsRef()));
    for (const auto& p : owner_.stabledPetsRef()) {
        LOG_DEBUG("  Pet: number=", p.petNumber, " entry=", p.entry,
                  " level=", p.level, " name='", p.name, "' displayId=", p.displayId,
                  " active=", p.isActive);
    }
}

// ============================================================
// Cast state methods (moved from GameHandler)
// ============================================================

void SpellHandler::stopCasting() {
    if (!owner_.isInWorld()) {
        LOG_WARNING("Cannot stop casting: not in world or not connected");
        return;
    }

    if (!casting_) {
        return;
    }

    if (owner_.pendingGameObjectInteractGuidRef() == 0 && currentCastSpellId_ != 0) {
        auto packet = CancelCastPacket::build(currentCastSpellId_);
        owner_.getSocket()->send(packet);
    }

    casting_ = false;
    castIsChannel_ = false;
    currentCastSpellId_ = 0;
    castTimeRemaining_ = 0.0f;
    castTimeTotal_ = 0.0f;
    owner_.pendingGameObjectInteractGuidRef() = 0;
    owner_.lastInteractedGoGuidRef() = 0;
    craftQueueSpellId_ = 0;
    craftQueueRemaining_ = 0;
    queuedSpellId_ = 0;
    queuedSpellTarget_ = 0;

    LOG_INFO("Cancelled spell cast");
}

void SpellHandler::resetCastState() {
    restorationActive_ = false;
    restorationSpellId_ = 0;
    restorationTimeRemaining_ = 0.0f;
    restorationTimeTotal_ = 0.0f;
    casting_ = false;
    castIsChannel_ = false;
    currentCastSpellId_ = 0;
    castTimeRemaining_ = 0.0f;
    castTimeTotal_ = 0.0f;  // Must match castTimeRemaining_ to keep getCastProgress() == 0
    craftQueueSpellId_ = 0;
    craftQueueRemaining_ = 0;
    queuedSpellId_ = 0;
    queuedSpellTarget_ = 0;
    owner_.pendingGameObjectInteractGuidRef() = 0;
    // lastInteractedGoGuid_ is intentionally NOT cleared here — it must survive
    // until the server delivers SMSG_LOOT_RESPONSE after the cast completes.
    // InventoryHandler::handleLootResponse() clears it once loot has opened
    // (src/game/inventory_handler.cpp). Previously it was cleared here, which
    // meant the client-side cast-timer fallback destroyed the guid before
    // SMSG_SPELL_GO arrived, preventing loot from opening on quest chests.
}

void SpellHandler::resetAllState() {
    knownSpells_.clear();
    spellCooldowns_.clear();
    playerAuras_.clear();
    targetAuras_.clear();
    unitAurasCache_.clear();
    unitCastStates_.clear();
    resetCastState();
    resetTalentState();
}

void SpellHandler::resetTalentState() {
    talentsInitialized_ = false;
    learnedTalents_[0].clear();
    learnedTalents_[1].clear();
    learnedGlyphs_[0].fill(0);
    learnedGlyphs_[1].fill(0);
    unspentTalentPoints_[0] = 0;
    unspentTalentPoints_[1] = 0;
    activeTalentSpec_ = 0;
}

void SpellHandler::clearUnitCaches() {
    unitCastStates_.clear();
    unitAurasCache_.clear();
}

// ============================================================
// Aura duration update (moved from GameHandler)
// ============================================================

void SpellHandler::handleUpdateAuraDuration(uint8_t slot, uint32_t durationMs) {
    if (slot >= playerAuras_.size()) return;
    if (playerAuras_[slot].isEmpty()) return;
    playerAuras_[slot].durationMs = static_cast<int32_t>(durationMs);
    playerAuras_[slot].receivedAtMs = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    refreshRestorationFromPlayerAuras();
}

// ============================================================
// Spell DBC / Cache methods (moved from GameHandler)
// ============================================================

static const std::string SPELL_EMPTY_STRING;

void SpellHandler::loadSpellNameCache() const {
    if (owner_.spellNameCacheLoadedRef()) return;
    owner_.spellNameCacheLoadedRef() = true;

    auto* am = owner_.services().assetManager;
    if (!am || !am->isInitialized()) return;

    auto dbc = am->loadDBC("Spell.dbc");
    if (!dbc || !dbc->isLoaded()) {
        LOG_WARNING("Trainer: Could not load Spell.dbc for spell names");
        return;
    }

    if (dbc->getFieldCount() < 148) {
        LOG_WARNING("Trainer: Spell.dbc has too few fields (", dbc->getFieldCount(), ")");
        return;
    }

    const auto* spellL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("Spell") : nullptr;

    uint32_t schoolMaskField = 0, schoolEnumField = 0;
    bool hasSchoolMask = false, hasSchoolEnum = false;
    if (spellL) {
        uint32_t f = spellL->field("SchoolMask");
        if (f != 0xFFFFFFFF && f < dbc->getFieldCount()) { schoolMaskField = f; hasSchoolMask = true; }
        f = spellL->field("SchoolEnum");
        if (f != 0xFFFFFFFF && f < dbc->getFieldCount()) { schoolEnumField = f; hasSchoolEnum = true; }
    }

    uint32_t dispelField = 0xFFFFFFFF;
    bool hasDispelField = false;
    if (spellL) {
        uint32_t f = spellL->field("DispelType");
        if (f != 0xFFFFFFFF && f < dbc->getFieldCount()) { dispelField = f; hasDispelField = true; }
    }

    uint32_t attrExField = 0xFFFFFFFF;
    bool hasAttrExField = false;
    if (spellL) {
        uint32_t f = spellL->field("AttributesEx");
        if (f != 0xFFFFFFFF && f < dbc->getFieldCount()) { attrExField = f; hasAttrExField = true; }
    }

    uint32_t tooltipField = 0xFFFFFFFF;
    if (spellL) {
        uint32_t f = spellL->field("Tooltip");
        if (f != 0xFFFFFFFF && f < dbc->getFieldCount()) tooltipField = f;
    }
    // The full effect Description (e.g. "Restores X health over Y sec. ...become well
    // fed...") is richer than the short Tooltip; prefer it and fall back to Tooltip.
    uint32_t descriptionField = 0xFFFFFFFF;
    if (spellL) {
        uint32_t f = spellL->field("Description");
        if (f != 0xFFFFFFFF && f < dbc->getFieldCount()) descriptionField = f;
    }

    // Targets: SpellCastTargets mask the spell demands. Item-enhancement spells
    // (sharpening stones, weightstones, weapon oils) set TARGET_FLAG_ITEM here.
    uint32_t targetsField = 0xFFFFFFFF;
    if (spellL) {
        uint32_t f = spellL->field("Targets");
        if (f != 0xFFFFFFFF && f < dbc->getFieldCount()) targetsField = f;
    }

    // Cache field indices before the loop to avoid repeated layout lookups
    const uint32_t idField   = spellL ? (*spellL)["ID"]   : 0;
    const uint32_t nameField = spellL ? (*spellL)["Name"] : 136;
    const uint32_t rankField = spellL ? (*spellL)["Rank"] : 153;
    const uint32_t fieldCount = dbc->getFieldCount();
    const bool hasEffectFields = (fieldCount > 109);
    const bool hasReagentFields = (fieldCount > 67);
    const uint32_t ebp0Field = spellL ? spellL->field("EffectBasePoints0") : 0xFFFFFFFF;
    const uint32_t ebp1Field = spellL ? spellL->field("EffectBasePoints1") : 0xFFFFFFFF;
    const uint32_t ebp2Field = spellL ? spellL->field("EffectBasePoints2") : 0xFFFFFFFF;
    const uint32_t effect0Field = spellL ? spellL->field("Effect0") : 0xFFFFFFFF;
    const uint32_t effect1Field = spellL ? spellL->field("Effect1") : 0xFFFFFFFF;
    const uint32_t effect2Field = spellL ? spellL->field("Effect2") : 0xFFFFFFFF;
    const uint32_t implicitTargetAField =
        spellL ? spellL->field("EffectImplicitTargetA") : 0xFFFFFFFF;
    const uint32_t durIdxField = spellL ? spellL->field("DurationIndex") : 0xFFFFFFFF;
    const uint32_t rangeIdxField = spellL ? spellL->field("RangeIndex") : 0xFFFFFFFF;
    const uint32_t targetAuraStateField = spellL ? spellL->field("TargetAuraState") : 0xFFFFFFFF;
    const uint32_t spellVisualIdField = spellL ? spellL->field("SpellVisualID") : 0xFFFFFFFF;
    const uint32_t recoveryField = spellL ? spellL->field("RecoveryTime") : 0xFFFFFFFF;
    const uint32_t categoryRecoveryField = spellL ? spellL->field("CategoryRecoveryTime") : 0xFFFFFFFF;

    uint32_t count = dbc->getRecordCount();
    for (uint32_t i = 0; i < count; ++i) {
        uint32_t id = dbc->getUInt32(i, idField);
        if (id == 0) continue;
        std::string name = dbc->getString(i, nameField);
        std::string rank = dbc->getString(i, rankField);
        if (!name.empty()) {
            GameHandler::SpellNameEntry entry;
            entry.name = std::move(name);
            entry.rank = std::move(rank);
            if (descriptionField != 0xFFFFFFFF) {
                entry.description = dbc->getString(i, descriptionField);
            }
            if (entry.description.empty() && tooltipField != 0xFFFFFFFF) {
                entry.description = dbc->getString(i, tooltipField);
            }
            if (hasSchoolMask) {
                entry.schoolMask = dbc->getUInt32(i, schoolMaskField);
            } else if (hasSchoolEnum) {
                static constexpr uint32_t enumToBitmask[] = {0x01,0x02,0x04,0x08,0x10,0x20,0x40};
                uint32_t e = dbc->getUInt32(i, schoolEnumField);
                entry.schoolMask = (e < 7) ? enumToBitmask[e] : 0;
            }
            if (hasDispelField) {
                entry.dispelType = static_cast<uint8_t>(dbc->getUInt32(i, dispelField));
            }
            if (hasAttrExField) {
                entry.attrEx = dbc->getUInt32(i, attrExField);
            }
            if (targetsField != 0xFFFFFFFF) {
                entry.targetFlags = dbc->getUInt32(i, targetsField);
            }
            if (targetAuraStateField != 0xFFFFFFFF && targetAuraStateField < fieldCount) {
                entry.targetAuraState = dbc->getUInt32(i, targetAuraStateField);
            }
            // Load effect base points for $s1/$s2/$s3 tooltip substitution
            if (ebp0Field != 0xFFFFFFFF) entry.effectBasePoints[0] = static_cast<int32_t>(dbc->getUInt32(i, ebp0Field));
            if (ebp1Field != 0xFFFFFFFF) entry.effectBasePoints[1] = static_cast<int32_t>(dbc->getUInt32(i, ebp1Field));
            if (ebp2Field != 0xFFFFFFFF) entry.effectBasePoints[2] = static_cast<int32_t>(dbc->getUInt32(i, ebp2Field));
            if (implicitTargetAField != 0xFFFFFFFF && implicitTargetAField < fieldCount) {
                entry.implicitTargetA = dbc->getUInt32(i, implicitTargetAField);
            }
            const uint32_t effectFields[3] = {effect0Field, effect1Field, effect2Field};
            for (size_t effect = 0; effect < 3; ++effect) {
                if (effectFields[effect] != 0xFFFFFFFF && effectFields[effect] < fieldCount) {
                    entry.effectIds[effect] = dbc->getUInt32(i, effectFields[effect]);
                }
            }
            // Duration: read DurationIndex and resolve via SpellDuration.dbc later
            if (durIdxField != 0xFFFFFFFF)
                entry.durationSec = static_cast<float>(dbc->getUInt32(i, durIdxField)); // store index temporarily
            // Range: read RangeIndex and resolve via SpellRange.dbc later
            if (rangeIdxField != 0xFFFFFFFF)
                entry.maxRange = static_cast<float>(dbc->getUInt32(i, rangeIdxField)); // store index temporarily
            // SpellVisualID: references SpellVisual.dbc for cast/impact M2 effects
            if (spellVisualIdField != 0xFFFFFFFF && spellVisualIdField < dbc->getFieldCount())
                entry.spellVisualId = dbc->getUInt32(i, spellVisualIdField);
            if (recoveryField != 0xFFFFFFFF && recoveryField < fieldCount)
                entry.recoveryMs = dbc->getUInt32(i, recoveryField);
            if (categoryRecoveryField != 0xFFFFFFFF && categoryRecoveryField < fieldCount)
                entry.categoryRecoveryMs = dbc->getUInt32(i, categoryRecoveryField);
            if (hasEffectFields) {
                for (int e = 0; e < 3; ++e) {
                    if (dbc->getUInt32(i, 71 + e) == 24 || dbc->getUInt32(i, 71 + e) == 114) {
                        entry.createdItemId = dbc->getUInt32(i, 107 + e);
                        break;
                    }
                }
            }
            if (hasReagentFields) {
                for (int r = 0; r < 8; ++r) {
                    entry.reagents[r].itemId = dbc->getUInt32(i, 52 + r);
                    entry.reagents[r].count  = dbc->getUInt32(i, 60 + r);
                }
            }
            owner_.spellNameCacheRef()[id] = std::move(entry);
        }
    }
    auto durDbc = am->loadDBC("SpellDuration.dbc");
    if (durDbc && durDbc->isLoaded()) {
        std::unordered_map<uint32_t, float> durMap;
        for (uint32_t di = 0; di < durDbc->getRecordCount(); ++di) {
            uint32_t durId = durDbc->getUInt32(di, 0);
            int32_t baseMs = static_cast<int32_t>(durDbc->getUInt32(di, 1));
            if (baseMs > 0 && baseMs < 100000000)
                durMap[durId] = baseMs / 1000.0f;
        }
        for (auto& [sid, entry] : owner_.spellNameCacheRef()) {
            uint32_t durIdx = static_cast<uint32_t>(entry.durationSec);
            if (durIdx > 0) {
                auto it = durMap.find(durIdx);
                entry.durationSec = (it != durMap.end()) ? it->second : 0.0f;
            }
        }
    }
    // Resolve the stored RangeIndex into an actual max range. Entries that cannot
    // be resolved fall back to -1 (unknown) so callers keep their old behaviour
    // rather than mistaking a raw index for a distance in yards.
    const auto* rangeL = pipeline::getActiveDBCLayout()
        ? pipeline::getActiveDBCLayout()->getLayout("SpellRange") : nullptr;
    auto rangeDbc = am->loadDBC("SpellRange.dbc");
    std::unordered_map<uint32_t, float> rangeMap;
    if (rangeDbc && rangeDbc->isLoaded() && rangeL) {
        const uint32_t maxRangeField = rangeL->field("MaxRange");
        if (maxRangeField != 0xFFFFFFFF && maxRangeField < rangeDbc->getFieldCount()) {
            for (uint32_t ri = 0; ri < rangeDbc->getRecordCount(); ++ri) {
                rangeMap[rangeDbc->getUInt32(ri, 0)] = rangeDbc->getFloat(ri, maxRangeField);
            }
        }
    }
    for (auto& [sid, entry] : owner_.spellNameCacheRef()) {
        if (entry.maxRange < 0.0f) continue; // no RangeIndex field in this layout
        auto it = rangeMap.find(static_cast<uint32_t>(entry.maxRange));
        entry.maxRange = (it != rangeMap.end()) ? it->second : -1.0f;
    }
    LOG_INFO("Trainer: Loaded ", owner_.spellNameCacheRef().size(), " spell names from Spell.dbc");
}

void SpellHandler::loadSkillLineAbilityDbc() {
    if (owner_.skillLineAbilityLoadedRef()) return;
    owner_.skillLineAbilityLoadedRef() = true;

    auto* am = owner_.services().assetManager;
    if (!am || !am->isInitialized()) return;

    auto slaDbc = am->loadDBC("SkillLineAbility.dbc");
    if (slaDbc && slaDbc->isLoaded()) {
        const auto* slaL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("SkillLineAbility") : nullptr;
        const uint32_t slaSkillField = slaL ? (*slaL)["SkillLineID"] : 1;
        const uint32_t slaSpellField = slaL ? (*slaL)["SpellID"]     : 2;
        const uint32_t slaFieldCount = slaDbc->getFieldCount();
        const bool hasDiffFields = (slaFieldCount > 11);
        for (uint32_t i = 0; i < slaDbc->getRecordCount(); i++) {
            uint32_t skillLineId = slaDbc->getUInt32(i, slaSkillField);
            uint32_t spellId = slaDbc->getUInt32(i, slaSpellField);
            if (spellId > 0 && skillLineId > 0) {
                owner_.spellToSkillLineRef()[spellId] = skillLineId;
                if (hasDiffFields) {
                    uint32_t trivHigh = slaDbc->getUInt32(i, 10);
                    uint32_t trivLow  = slaDbc->getUInt32(i, 11);
                    uint32_t minRank  = slaDbc->getUInt32(i, 7);
                    if (trivHigh > 0 || trivLow > 0) {
                        auto cit = owner_.spellNameCacheRef().find(spellId);
                        if (cit != owner_.spellNameCacheRef().end()) {
                            cit->second.trivialSkillHigh = trivHigh;
                            cit->second.trivialSkillLow  = trivLow;
                            cit->second.minSkillRank     = minRank;
                        }
                    }
                }
            }
        }
        LOG_INFO("Trainer: Loaded ", owner_.spellToSkillLineRef().size(), " skill line abilities");
    }
}

const std::string& SpellHandler::getSpellFocusName(uint32_t focusId) {
    static const std::string kEmpty;
    if (!spellFocusDbcLoaded_) {
        spellFocusDbcLoaded_ = true;
        auto* am = owner_.services().assetManager;
        if (am && am->isInitialized()) {
            auto dbc = am->loadDBC("SpellFocusObject.dbc");
            // Layout is stable across expansions: ID(0) + Name locstring
            // whose English text sits at field 1.
            if (dbc && dbc->isLoaded() && dbc->getFieldCount() >= 2) {
                for (uint32_t i = 0; i < dbc->getRecordCount(); ++i) {
                    uint32_t id = dbc->getUInt32(i, 0);
                    std::string name = dbc->getString(i, 1);
                    if (id != 0 && !name.empty())
                        spellFocusNames_[id] = std::move(name);
                }
                LOG_INFO("Loaded ", spellFocusNames_.size(), " spell focus object names");
            }
        }
    }
    auto it = spellFocusNames_.find(focusId);
    return it != spellFocusNames_.end() ? it->second : kEmpty;
}

const std::string& SpellHandler::getTotemCategoryName(uint32_t categoryId) {
    static const std::string kEmpty;
    if (!totemCategoryDbcLoaded_) {
        totemCategoryDbcLoaded_ = true;
        auto* am = owner_.services().assetManager;
        if (am && am->isInitialized()) {
            // TBC/WotLK only — absent in Vanilla, where totem failures carry
            // item ids instead of category ids.
            auto dbc = am->loadDBC("TotemCategory.dbc");
            // ID(0) + Name locstring whose English text sits at field 1.
            if (dbc && dbc->isLoaded() && dbc->getFieldCount() >= 2) {
                for (uint32_t i = 0; i < dbc->getRecordCount(); ++i) {
                    uint32_t id = dbc->getUInt32(i, 0);
                    std::string name = dbc->getString(i, 1);
                    if (id != 0 && !name.empty())
                        totemCategoryNames_[id] = std::move(name);
                }
                LOG_INFO("Loaded ", totemCategoryNames_.size(), " totem category names");
            }
        }
    }
    auto it = totemCategoryNames_.find(categoryId);
    return it != totemCategoryNames_.end() ? it->second : kEmpty;
}

uint32_t SpellHandler::tradeskillOpenerSkillLine(uint32_t spellId) {
    owner_.loadSpellNameCache();
    owner_.loadSkillLineDbc();
    owner_.loadSkillLineAbilityDbc();

    // SkillLine.dbc categories that hold crafting recipes
    static constexpr uint32_t CAT_SECONDARY  = 9;   // Cooking, First Aid, Fishing
    static constexpr uint32_t CAT_PROFESSION = 11;  // Alchemy, Blacksmithing, ...
    // Smelting shares the Mining skill line but isn't named after it
    static constexpr uint32_t SPELL_SMELTING = 2656;

    auto slIt = owner_.spellToSkillLineRef().find(spellId);
    if (slIt == owner_.spellToSkillLineRef().end()) return 0;
    const uint32_t skillLine = slIt->second;

    auto catIt = owner_.skillLineCategoriesRef().find(skillLine);
    if (catIt == owner_.skillLineCategoriesRef().end()) return 0;
    if (catIt->second != CAT_SECONDARY && catIt->second != CAT_PROFESSION) return 0;

    // Opener heuristic: the window-opening spell is named after its skill line
    // ("Cooking" opens Cooking). Recipes, gathering casts (Disenchant, Fishing
    // bobber cast is filtered below by the recipe requirement), and utility
    // spells never share the skill line's name.
    if (spellId != SPELL_SMELTING) {
        const std::string& spellName = owner_.getSpellName(spellId);
        auto nameIt = owner_.skillLineNamesRef().find(skillLine);
        if (spellName.empty() || nameIt == owner_.skillLineNamesRef().end() ||
            spellName != nameIt->second) {
            return 0;
        }
    }

    // Only open a window that will actually list something: require at least
    // one known recipe (creates an item or consumes reagents) in this line.
    // This keeps Fishing falling through to a normal bobber cast.
    for (uint32_t known : knownSpells_) {
        if (known == spellId) continue;
        auto kslIt = owner_.spellToSkillLineRef().find(known);
        if (kslIt == owner_.spellToSkillLineRef().end() || kslIt->second != skillLine) continue;
        auto cacheIt = owner_.spellNameCacheRef().find(known);
        if (cacheIt == owner_.spellNameCacheRef().end()) continue;
        const auto& entry = cacheIt->second;
        bool hasReagents = false;
        for (const auto& reagent : entry.reagents) {
            if (reagent.itemId != 0) { hasReagents = true; break; }
        }
        if (entry.createdItemId != 0 || hasReagents) return skillLine;
    }
    return 0;
}

void SpellHandler::categorizeTrainerSpells() {
    owner_.trainerTabsRef().clear();

    static constexpr uint32_t SKILLLINE_CATEGORY_CLASS = 7;

    std::map<uint32_t, std::vector<const TrainerSpell*>> specialtySpells;
    std::vector<const TrainerSpell*> generalSpells;

    for (const auto& spell : owner_.currentTrainerListRef().spells) {
        auto slIt = owner_.spellToSkillLineRef().find(spell.spellId);
        if (slIt != owner_.spellToSkillLineRef().end()) {
            uint32_t skillLineId = slIt->second;
            auto catIt = owner_.skillLineCategoriesRef().find(skillLineId);
            if (catIt != owner_.skillLineCategoriesRef().end() && catIt->second == SKILLLINE_CATEGORY_CLASS) {
                specialtySpells[skillLineId].push_back(&spell);
                continue;
            }
        }
        generalSpells.push_back(&spell);
    }

    auto byName = [this](const TrainerSpell* a, const TrainerSpell* b) {
        return getSpellName(a->spellId) < getSpellName(b->spellId);
    };

    std::vector<std::pair<std::string, std::vector<const TrainerSpell*>>> named;
    for (auto& [skillLineId, spells] : specialtySpells) {
        auto nameIt = owner_.skillLineNamesRef().find(skillLineId);
        std::string tabName = (nameIt != owner_.skillLineNamesRef().end()) ? nameIt->second : "Specialty";
        std::sort(spells.begin(), spells.end(), byName);
        named.push_back({std::move(tabName), std::move(spells)});
    }
    std::sort(named.begin(), named.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; });

    for (auto& [name, spells] : named) {
        owner_.trainerTabsRef().push_back({std::move(name), std::move(spells)});
    }

    if (!generalSpells.empty()) {
        std::sort(generalSpells.begin(), generalSpells.end(), byName);
        owner_.trainerTabsRef().push_back({"General", std::move(generalSpells)});
    }

    LOG_INFO("Trainer: Categorized into ", owner_.trainerTabsRef().size(), " tabs");
}

const int32_t* SpellHandler::getSpellEffectBasePoints(uint32_t spellId) const {
    loadSpellNameCache();
    auto it = owner_.spellNameCacheRef().find(spellId);
    return (it != owner_.spellNameCacheRef().end()) ? it->second.effectBasePoints : nullptr;
}

float SpellHandler::getSpellDuration(uint32_t spellId) const {
    loadSpellNameCache();
    auto it = owner_.spellNameCacheRef().find(spellId);
    return (it != owner_.spellNameCacheRef().end()) ? it->second.durationSec : 0.0f;
}

const std::string& SpellHandler::getSpellName(uint32_t spellId) const {
    // Lazy-load Spell.dbc so callers don't need to know about initialization order.
    // Every other DBC-backed getter (getSpellDescription, getSpellSchoolMask, etc.)
    // already does this; these two were missed.
    loadSpellNameCache();
    auto it = owner_.spellNameCacheRef().find(spellId);
    return (it != owner_.spellNameCacheRef().end()) ? it->second.name : SPELL_EMPTY_STRING;
}

const std::string& SpellHandler::getSpellRank(uint32_t spellId) const {
    loadSpellNameCache();
    auto it = owner_.spellNameCacheRef().find(spellId);
    return (it != owner_.spellNameCacheRef().end()) ? it->second.rank : SPELL_EMPTY_STRING;
}

const std::string& SpellHandler::getSpellDescription(uint32_t spellId) const {
    loadSpellNameCache();
    auto it = owner_.spellNameCacheRef().find(spellId);
    return (it != owner_.spellNameCacheRef().end()) ? it->second.description : SPELL_EMPTY_STRING;
}

std::string SpellHandler::getEnchantName(uint32_t enchantId) const {
    if (enchantId == 0) return {};
    auto* am = owner_.services().assetManager;
    if (!am || !am->isInitialized()) return {};
    auto dbc = am->loadDBC("SpellItemEnchantment.dbc");
    if (!dbc || !dbc->isLoaded()) return {};
    const auto* sieL = pipeline::getActiveDBCLayout()
        ? pipeline::getActiveDBCLayout()->getLayout("SpellItemEnchantment") : nullptr;
    const uint32_t nameField = pipeline::detectEnchantmentNameField(dbc.get(), sieL);
    for (uint32_t i = 0; i < dbc->getRecordCount(); ++i) {
        if (dbc->getUInt32(i, 0) == enchantId) {
            return dbc->getString(i, nameField);
        }
    }
    return {};
}

uint8_t SpellHandler::getSpellDispelType(uint32_t spellId) const {
    loadSpellNameCache();
    auto it = owner_.spellNameCacheRef().find(spellId);
    return (it != owner_.spellNameCacheRef().end()) ? it->second.dispelType : 0;
}

bool SpellHandler::isSpellInterruptible(uint32_t spellId) const {
    if (spellId == 0) return true;
    loadSpellNameCache();
    auto it = owner_.spellNameCacheRef().find(spellId);
    if (it == owner_.spellNameCacheRef().end()) return true;
    return (it->second.attrEx & 0x00000010u) == 0;
}

uint32_t SpellHandler::getSpellSchoolMask(uint32_t spellId) const {
    if (spellId == 0) return 0;
    loadSpellNameCache();
    auto it = owner_.spellNameCacheRef().find(spellId);
    return (it != owner_.spellNameCacheRef().end()) ? it->second.schoolMask : 0;
}

uint32_t SpellHandler::getSpellTargetFlags(uint32_t spellId) const {
    if (spellId == 0) return 0;
    loadSpellNameCache();
    auto it = owner_.spellNameCacheRef().find(spellId);
    return (it != owner_.spellNameCacheRef().end()) ? it->second.targetFlags : 0;
}

uint32_t SpellHandler::getSpellTargetAuraState(uint32_t spellId) const {
    if (spellId == 0) return 0;
    loadSpellNameCache();
    auto it = owner_.spellNameCacheRef().find(spellId);
    return (it != owner_.spellNameCacheRef().end()) ? it->second.targetAuraState : 0;
}

uint32_t SpellHandler::resolveHighestKnownRank(uint32_t spellId) const {
    if (spellId == 0 || knownSpells_.count(spellId) > 0) return spellId;

    loadSpellNameCache();
    const auto& cache = owner_.spellNameCacheRef();

    // Adapt the spell name cache to the pure resolver in spell_classification.hpp.
    thread_local spellclass::SpellRankInfo scratch;
    auto lookup = [&cache](uint32_t id) -> const spellclass::SpellRankInfo* {
        auto it = cache.find(id);
        if (it == cache.end()) return nullptr;
        scratch.name = it->second.name;
        scratch.rank = it->second.rank;
        return &scratch;
    };

    const uint32_t best = spellclass::resolveHighestKnownRank(spellId, knownSpells_, lookup);
    if (best != spellId) {
        LOG_INFO("Superseded rank: casting ", best, " instead of ", spellId);
    }
    return best;
}

float SpellHandler::getSpellMaxRange(uint32_t spellId) const {
    if (spellId == 0) return -1.0f;
    loadSpellNameCache();
    auto it = owner_.spellNameCacheRef().find(spellId);
    return (it != owner_.spellNameCacheRef().end()) ? it->second.maxRange : -1.0f;
}

uint32_t SpellHandler::getSpellImplicitTargetA(uint32_t spellId) const {
    if (spellId == 0) return 0;
    loadSpellNameCache();
    auto it = owner_.spellNameCacheRef().find(spellId);
    return (it != owner_.spellNameCacheRef().end()) ? it->second.implicitTargetA : 0;
}

bool SpellHandler::isSelfCastSpell(uint32_t spellId) const {
    if (spellId == 0) return false;
    loadSpellNameCache();
    auto it = owner_.spellNameCacheRef().find(spellId);
    if (it == owner_.spellNameCacheRef().end()) return false;
    // SpellRange "Self Only" has a max range of 0 — the spell cannot reach any
    // other unit, so it is cast on the caster no matter what is targeted.
    // A negative range means SpellRange.dbc was unavailable; assume not self-cast.
    return spellclass::isSelfCastRange(it->second.maxRange);
}

const std::string& SpellHandler::getSkillLineName(uint32_t spellId) const {
    auto slIt = owner_.spellToSkillLineRef().find(spellId);
    if (slIt == owner_.spellToSkillLineRef().end()) return SPELL_EMPTY_STRING;
    auto nameIt = owner_.skillLineNamesRef().find(slIt->second);
    return (nameIt != owner_.skillLineNamesRef().end()) ? nameIt->second : SPELL_EMPTY_STRING;
}

// ============================================================
// Skill DBC methods (moved from GameHandler)
// ============================================================

void SpellHandler::loadSkillLineDbc() {
    if (owner_.skillLineDbcLoadedRef()) return;
    owner_.skillLineDbcLoadedRef() = true;

    auto* am = owner_.services().assetManager;
    if (!am || !am->isInitialized()) return;

    auto dbc = am->loadDBC("SkillLine.dbc");
    if (!dbc || !dbc->isLoaded()) {
        LOG_WARNING("GameHandler: Could not load SkillLine.dbc");
        return;
    }

    const auto* slL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("SkillLine") : nullptr;
    const uint32_t slIdField   = slL ? (*slL)["ID"]       : 0;
    const uint32_t slCatField  = slL ? (*slL)["Category"] : 1;
    const uint32_t slNameField = slL ? (*slL)["Name"]     : 3;
    for (uint32_t i = 0; i < dbc->getRecordCount(); i++) {
        uint32_t id = dbc->getUInt32(i, slIdField);
        uint32_t category = dbc->getUInt32(i, slCatField);
        std::string name = dbc->getString(i, slNameField);
        if (id > 0 && !name.empty()) {
            owner_.skillLineNamesRef()[id] = name;
            owner_.skillLineCategoriesRef()[id] = category;
        }
    }
    LOG_INFO("GameHandler: Loaded ", owner_.skillLineNamesRef().size(), " skill line names");
}

void SpellHandler::extractSkillFields(const FlatFieldMap& fields) {
    loadSkillLineDbc();

    const uint16_t PLAYER_SKILL_INFO_START = fieldIndex(UF::PLAYER_SKILL_INFO_START);
    static constexpr int MAX_SKILL_SLOTS = 128;

    std::unordered_map<uint32_t, PlayerSkill> newSkills;

    for (int slot = 0; slot < MAX_SKILL_SLOTS; slot++) {
        uint16_t baseField = PLAYER_SKILL_INFO_START + slot * 3;

        auto idIt = fields.find(baseField);
        if (idIt == fields.end()) continue;

        uint32_t raw0 = idIt->second;
        uint16_t skillId = raw0 & 0xFFFF;
        if (skillId == 0) continue;

        auto valIt = fields.find(baseField + 1);
        if (valIt == fields.end()) continue;

        uint32_t raw1 = valIt->second;
        uint16_t value = raw1 & 0xFFFF;
        uint16_t maxValue = (raw1 >> 16) & 0xFFFF;

        uint16_t bonusTemp = 0;
        uint16_t bonusPerm = 0;
        auto bonusIt = fields.find(static_cast<uint16_t>(baseField + 2));
        if (bonusIt != fields.end()) {
            bonusTemp = bonusIt->second & 0xFFFF;
            bonusPerm = (bonusIt->second >> 16) & 0xFFFF;
        }

        PlayerSkill skill;
        skill.skillId = skillId;
        skill.value = value;
        skill.maxValue = maxValue;
        skill.bonusTemp = bonusTemp;
        skill.bonusPerm = bonusPerm;
        newSkills[skillId] = skill;
    }

    for (const auto& [skillId, skill] : newSkills) {
        if (skill.value == 0) continue;
        auto oldIt = owner_.playerSkillsRef().find(skillId);
        if (oldIt != owner_.playerSkillsRef().end() && skill.value > oldIt->second.value) {
            auto catIt = owner_.skillLineCategoriesRef().find(skillId);
            if (catIt != owner_.skillLineCategoriesRef().end()) {
                uint32_t category = catIt->second;
                if (category == 5 || category == 10 || category == 12) {
                    continue;
                }
            }

            const std::string& name = owner_.getSkillName(skillId);
            std::string skillName = name.empty() ? ("Skill #" + std::to_string(skillId)) : name;
            owner_.addSystemChatMessage("Your skill in " + skillName + " has increased to " + std::to_string(skill.value) + ".");
        }
    }

    bool skillsChanged = (newSkills.size() != owner_.playerSkillsRef().size());
    if (!skillsChanged) {
        for (const auto& [id, sk] : newSkills) {
            auto it = owner_.playerSkillsRef().find(id);
            if (it == owner_.playerSkillsRef().end() || it->second.value != sk.value) {
                skillsChanged = true;
                break;
            }
        }
    }
    owner_.playerSkillsRef() = std::move(newSkills);
    if (skillsChanged)
        owner_.fireAddonEvent("SKILL_LINES_CHANGED", {});
}

void SpellHandler::extractExploredZoneFields(const FlatFieldMap& fields) {
    const size_t zoneCount = owner_.getPacketParsers()
        ? static_cast<size_t>(owner_.getPacketParsers()->exploredZonesCount())
        : GameHandler::PLAYER_EXPLORED_ZONES_COUNT;

    if (owner_.playerExploredZonesRef().size() != GameHandler::PLAYER_EXPLORED_ZONES_COUNT) {
        owner_.playerExploredZonesRef().assign(GameHandler::PLAYER_EXPLORED_ZONES_COUNT, 0u);
    }

    bool foundAny = false;
    for (size_t i = 0; i < zoneCount; i++) {
        const uint16_t fieldIdx = static_cast<uint16_t>(fieldIndex(UF::PLAYER_EXPLORED_ZONES_START) + i);
        auto it = fields.find(fieldIdx);
        if (it == fields.end()) continue;
        owner_.playerExploredZonesRef()[i] = it->second;
        foundAny = true;
    }
    for (size_t i = zoneCount; i < GameHandler::PLAYER_EXPLORED_ZONES_COUNT; i++) {
        owner_.playerExploredZonesRef()[i] = 0u;
    }

    if (foundAny) {
        owner_.hasPlayerExploredZonesRef() = true;
    }
}

// ============================================================
// Moved opcode handlers (from GameHandler::registerOpcodeHandlers)
// ============================================================

void SpellHandler::handleCastResult(network::Packet& packet) {
    uint32_t castResultSpellId = 0;
    uint8_t  castResult        = 0;
    uint32_t castResultMiscArg = 0;
    uint32_t castResultMiscArg2 = 0;
    if (owner_.getPacketParsers()->parseCastResult(packet, castResultSpellId, castResult,
                                                   castResultMiscArg, castResultMiscArg2)) {
        LOG_DEBUG("SMSG_CAST_RESULT: spellId=", castResultSpellId, " result=", static_cast<int>(castResult));
        if (castResult != 0) {
            if (spellclass::isFishingCast(castResultSpellId)) {
                const auto& movement = owner_.movementInfoRef();
                LOG_WARNING("Fishing cast failed: spell=", castResultSpellId,
                            " result=", static_cast<int>(castResult),
                            " pos=(", movement.x, ",", movement.y, ",", movement.z, ")",
                            " facing=", movement.orientation,
                            " selectedTarget=0x", std::hex, owner_.getTargetGuid(), std::dec);
            }
            const uint64_t gatherGoGuid = owner_.lastInteractedGoGuidRef();
            const bool gatherCast = gatherGoGuid != 0 && isGatherSpellId(castResultSpellId);
            casting_ = false; castIsChannel_ = false; currentCastSpellId_ = 0; castTimeRemaining_ = 0.0f;
            owner_.lastInteractedGoGuidRef() = 0;
            owner_.pendingGameObjectInteractGuidRef() = 0;
            craftQueueSpellId_ = 0; craftQueueRemaining_ = 0;
            queuedSpellId_ = 0; queuedSpellTarget_ = 0;
            int playerPowerType = -1;
            if (auto pe = owner_.getEntityManager().getEntity(owner_.getPlayerGuid())) {
                if (auto pu = std::dynamic_pointer_cast<Unit>(pe))
                    playerPowerType = static_cast<int>(pu->getPowerType());
            }
            if (castResult == kSpellFailedNotReady) {
                seedCooldownFromSpellInfo(castResultSpellId);
            }
            // Totem failures name tool item ids; request their info so a retry
            // of the craft can show the tool's name even if it wasn't cached.
            if (castResult == kCastResultTotems) {
                if (castResultMiscArg != 0) owner_.ensureItemInfo(castResultMiscArg);
                if (castResultMiscArg2 != 0) owner_.ensureItemInfo(castResultMiscArg2);
            }
            std::string errMsg = castFailureMessage(owner_, castResultSpellId,
                                                     castResult, playerPowerType,
                                                     castResultMiscArg, castResultMiscArg2);
            if (gatherCast) {
                errMsg = gatherCastFailureMessage(owner_, gatherGoGuid,
                                                  castResultSpellId, castResult, errMsg);
                if (shouldDespawnGatherTarget(castResult)) {
                    owner_.despawnGameObjectLocally(gatherGoGuid);
                }
            }
            owner_.addUIError(errMsg);
            if (owner_.spellCastFailedCallbackRef()) owner_.spellCastFailedCallbackRef()(castResultSpellId);
                owner_.fireAddonEvent("UNIT_SPELLCAST_FAILED", {"player", std::to_string(castResultSpellId)});
                owner_.fireAddonEvent("UNIT_SPELLCAST_STOP",   {"player", std::to_string(castResultSpellId)});
            MessageChatData msg;
            msg.type     = ChatType::SYSTEM;
            msg.language = ChatLanguage::UNIVERSAL;
            msg.message  = errMsg;
            owner_.addLocalChatMessage(msg);
        }
    }
}

void SpellHandler::handleSpellFailedOther(network::Packet& packet) {
    const bool tbcLike2 = isPreWotlk();
    uint64_t failOtherGuid = tbcLike2
        ? (packet.hasRemaining(8) ? packet.readUInt64() : 0)
        : packet.readPackedGuid();
    if (failOtherGuid != 0 && failOtherGuid != owner_.getPlayerGuid()) {
        unitCastStates_.erase(failOtherGuid);
        if (owner_.addonEventCallbackRef()) {
            std::string unitId;
            if (failOtherGuid == owner_.getTargetGuid())     unitId = "target";
            else if (failOtherGuid == owner_.focusGuidRef()) unitId = "focus";
            if (!unitId.empty()) {
                owner_.fireAddonEvent("UNIT_SPELLCAST_FAILED", {unitId});
                owner_.fireAddonEvent("UNIT_SPELLCAST_STOP",   {unitId});
            }
        }
    }
    packet.skipAll();
}

void SpellHandler::handleClearCooldown(network::Packet& packet) {
    if (packet.hasRemaining(4)) {
        uint32_t spellId = packet.readUInt32();
        spellCooldowns_.erase(spellId);
        for (auto& slot : owner_.actionBarRef()) {
            if (slot.type == ActionBarSlot::SPELL && slot.id == spellId)
                slot.cooldownRemaining = 0.0f;
        }
    }
}

void SpellHandler::handleModifyCooldown(network::Packet& packet) {
    if (packet.hasRemaining(8)) {
        uint32_t spellId = packet.readUInt32();
        int32_t  diffMs  = static_cast<int32_t>(packet.readUInt32());
        float diffSec = diffMs / 1000.0f;
        auto it = spellCooldowns_.find(spellId);
        if (it != spellCooldowns_.end()) {
            it->second = std::max(0.0f, it->second + diffSec);
            for (auto& slot : owner_.actionBarRef()) {
                if (slot.type == ActionBarSlot::SPELL && slot.id == spellId)
                    slot.cooldownRemaining = std::max(0.0f, slot.cooldownRemaining + diffSec);
            }
        }
    }
}

void SpellHandler::handlePlaySpellVisual(network::Packet& packet) {
    if (!packet.hasRemaining(12)) return;
    uint64_t casterGuid = packet.readUInt64();
    uint32_t visualId   = packet.readUInt32();
    if (visualId == 0) return;
    auto* renderer = owner_.services().renderer;
    if (!renderer) return;
    glm::vec3 spawnPos;
    if (casterGuid == owner_.getPlayerGuid()) {
        spawnPos = renderer->getCharacterPosition();
    } else {
        auto entity = owner_.getEntityManager().getEntity(casterGuid);
        if (!entity) return;
        glm::vec3 canonical(entity->getLatestX(), entity->getLatestY(), entity->getLatestZ());
        spawnPos = core::coords::canonicalToRender(canonical);
    }
    if (auto* sv = renderer->getSpellVisualSystem())
        sv->playSpellVisual(visualId, spawnPos, /*useImpactKit=*/false,
                            owner_.resolveUnitRenderInstance(casterGuid));
}

void SpellHandler::handleSpellModifier(network::Packet& packet, bool isFlat) {
    auto& modMap = isFlat ? owner_.spellFlatModsRef() : owner_.spellPctModsRef();
    while (packet.hasRemaining(6)) {
        uint8_t groupIndex = packet.readUInt8();
        uint8_t modOpRaw   = packet.readUInt8();
        int32_t value      = static_cast<int32_t>(packet.readUInt32());
        if (groupIndex > 5 || modOpRaw >= GameHandler::SPELL_MOD_OP_COUNT) continue;
        GameHandler::SpellModKey key{ static_cast<GameHandler::SpellModOp>(modOpRaw), groupIndex };
        modMap[key] = value;
    }
    packet.skipAll();
}

void SpellHandler::handleSpellDelayed(network::Packet& packet) {
    const bool spellDelayTbcLike = isPreWotlk();
    if (!packet.hasRemaining(spellDelayTbcLike ? 8u : 1u) ) return;
    uint64_t caster = spellDelayTbcLike
        ? packet.readUInt64() : packet.readPackedGuid();
    if (!packet.hasRemaining(4)) return;
    uint32_t delayMs = packet.readUInt32();
    if (delayMs == 0) return;
    float delaySec = delayMs / 1000.0f;
    if (caster == owner_.getPlayerGuid()) {
        if (casting_) {
            castTimeRemaining_ += delaySec;
            castTimeTotal_     += delaySec;
        }
    } else {
        auto it = unitCastStates_.find(caster);
        if (it != unitCastStates_.end() && it->second.casting) {
            it->second.timeRemaining += delaySec;
            it->second.timeTotal     += delaySec;
        }
    }
}

// ============================================================
// Extracted opcode handlers (from registerOpcodeHandlers)
// ============================================================

void SpellHandler::handleSpellLogMiss(network::Packet& packet) {
    // All expansions: uint32 spellId first.
    // WotLK/Classic: spellId(4) + packed_guid caster + uint8 unk + uint32 count
    //                 + count × (packed_guid victim + uint8 missInfo)
    // TBC:            spellId(4) + uint64 caster + uint8 unk + uint32 count
    //                 + count × (uint64 victim + uint8 missInfo)
    // All expansions append uint32 reflectSpellId + uint8 reflectResult when
    // missInfo==REFLECT (11).
    const bool spellMissUsesFullGuid = isActiveExpansion("tbc");
    auto readSpellMissGuid = [&]() -> uint64_t {
        if (spellMissUsesFullGuid)
            return (packet.hasRemaining(8)) ? packet.readUInt64() : 0;
        return packet.readPackedGuid();
    };
    // spellId prefix present in all expansions
    if (!packet.hasRemaining(4)) return;
    uint32_t spellId = packet.readUInt32();
    if (!packet.hasRemaining(spellMissUsesFullGuid ? 8u : 1u)
        || (!spellMissUsesFullGuid && !packet.hasFullPackedGuid())) {
        packet.skipAll(); return;
    }
    uint64_t casterGuid = readSpellMissGuid();
    if (!packet.hasRemaining(5)) return;
    /*uint8_t unk =*/ packet.readUInt8();
    const uint32_t rawCount = packet.readUInt32();
    if (rawCount > 128) {
        LOG_WARNING("SMSG_SPELLLOGMISS: miss count capped (requested=", rawCount, ")");
    }
    const uint32_t storedLimit = std::min<uint32_t>(rawCount, 128u);

    struct SpellMissLogEntry {
        uint64_t victimGuid = 0;
        uint8_t missInfo = 0;
        uint32_t reflectSpellId = 0;  // Only valid when missInfo==REFLECT
    };
    std::vector<SpellMissLogEntry> parsedMisses;
    parsedMisses.reserve(storedLimit);

    bool truncated = false;
    for (uint32_t i = 0; i < rawCount; ++i) {
        if (!packet.hasRemaining(spellMissUsesFullGuid ? 9u : 2u)
            || (!spellMissUsesFullGuid && !packet.hasFullPackedGuid())) {
            truncated = true;
            return;
        }
        const uint64_t victimGuid = readSpellMissGuid();
        if (!packet.hasRemaining(1)) {
            truncated = true;
            return;
        }
        const uint8_t missInfo = packet.readUInt8();
        // REFLECT: extra uint32 reflectSpellId + uint8 reflectResult
        uint32_t reflectSpellId = 0;
        if (missInfo == SpellMissInfo::REFLECT) {
            if (packet.hasRemaining(5)) {
                reflectSpellId = packet.readUInt32();
                /*uint8_t reflectResult =*/ packet.readUInt8();
            } else {
                truncated = true;
                return;
            }
        }
        if (i < storedLimit) {
            parsedMisses.push_back({victimGuid, missInfo, reflectSpellId});
        }
    }

    if (truncated) {
        packet.skipAll();
        return;
    }

    for (const auto& miss : parsedMisses) {
        const uint64_t victimGuid = miss.victimGuid;
        const uint8_t missInfo = miss.missInfo;
        CombatTextEntry::Type ct = combatTextTypeFromSpellMissInfo(missInfo);
        // For REFLECT, use the reflected spell ID so combat text shows the spell name
        uint32_t combatSpellId = (ct == CombatTextEntry::REFLECT && miss.reflectSpellId != 0)
                                 ? miss.reflectSpellId : spellId;
        if (casterGuid == owner_.getPlayerGuid()) {
            // We cast a spell and it missed the target
            owner_.addCombatText(ct, 0, combatSpellId, true, 0, casterGuid, victimGuid);
        } else if (victimGuid == owner_.getPlayerGuid()) {
            // Enemy spell missed us (we dodged/parried/blocked/resisted/etc.)
            owner_.addCombatText(ct, 0, combatSpellId, false, 0, casterGuid, victimGuid);
        }
    }
}

void SpellHandler::handleSpellFailure(network::Packet& packet) {
    // WotLK: packed_guid + uint8 castCount + uint32 spellId + uint8 failReason
    // TBC:   full uint64 + uint8 castCount + uint32 spellId + uint8 failReason
    // Classic: full uint64 + uint32 spellId + uint8 failReason  (NO castCount)
    const bool isClassic = isClassicLikeExpansion();
    const bool isTbc     = isActiveExpansion("tbc");
    uint64_t failGuid = (isClassic || isTbc)
        ? (packet.hasRemaining(8) ? packet.readUInt64() : 0)
        : packet.readPackedGuid();
    // Classic omits the castCount byte; TBC and WotLK include it
    const size_t remainingFields = isClassic ? 5u : 6u;  // spellId(4)+reason(1) [+castCount(1)]
    if (packet.hasRemaining(remainingFields)) {
        if (!isClassic) /*uint8_t castCount =*/ packet.readUInt8();
        uint32_t failSpellId = packet.readUInt32();
        uint8_t rawFailReason = packet.readUInt8();
        // Classic result enum starts at 0=AFFECTING_COMBAT; shift +1 for WotLK table
        uint8_t failReason = isClassic ? static_cast<uint8_t>(rawFailReason + 1) : rawFailReason;
        if (failGuid == owner_.getPlayerGuid() && failReason != 0) {
            // Show interruption/failure reason in chat and error overlay for player
            int pt = -1;
            if (auto pe = owner_.getEntityManager().getEntity(owner_.getPlayerGuid()))
                if (auto pu = std::dynamic_pointer_cast<Unit>(pe))
                    pt = static_cast<int>(pu->getPowerType());
            std::string reason = castFailureMessage(owner_, failSpellId, failReason, pt);
            if (!reason.empty()) {
                // Prefix with spell name for context, e.g. "Fireball: Not in range"
                const std::string& sName = owner_.getSpellName(failSpellId);
                std::string fullMsg = sName.empty() ? reason
                                                    : sName + ": " + reason;
                owner_.addUIError(fullMsg);
                MessageChatData emsg;
                emsg.type = ChatType::SYSTEM;
                emsg.language = ChatLanguage::UNIVERSAL;
                emsg.message = std::move(fullMsg);
                owner_.addLocalChatMessage(emsg);
            }
        }
    }
    // Fire UNIT_SPELLCAST_INTERRUPTED for Lua addons
    if (owner_.addonEventCallbackRef()) {
        auto unitId = (failGuid == 0) ? std::string("player") : owner_.guidToUnitId(failGuid);
        if (!unitId.empty()) {
            owner_.fireAddonEvent("UNIT_SPELLCAST_INTERRUPTED", {unitId});
            owner_.fireAddonEvent("UNIT_SPELLCAST_STOP", {unitId});
        }
    }
    if (failGuid == owner_.getPlayerGuid() || failGuid == 0) {
        // Player's own cast failed — clear gather-node loot target so the
        // next timed cast doesn't try to loot a stale interrupted gather node.
        casting_ = false; castIsChannel_ = false; currentCastSpellId_ = 0;
        owner_.lastInteractedGoGuidRef() = 0;
        craftQueueSpellId_ = 0;
        craftQueueRemaining_ = 0;
        queuedSpellId_ = 0;
        queuedSpellTarget_ = 0;
        // Remove lingering precast visual effects
        if (auto* renderer = owner_.services().renderer) {
            if (auto* svs = renderer->getSpellVisualSystem())
                svs->cancelAllPrecastVisuals();
        }
        if (auto* ac = owner_.services().audioCoordinator) {
            if (auto* ssm = ac->getSpellSoundManager()) {
                ssm->stopPrecast();
            }
        }
        if (owner_.spellCastAnimCallbackRef()) {
            owner_.spellCastAnimCallbackRef()(owner_.getPlayerGuid(), false, false, SpellCastType::OMNI);
        }
    } else {
        // Another unit's cast failed — clear their tracked cast bar
        unitCastStates_.erase(failGuid);
        if (owner_.spellCastAnimCallbackRef()) {
            owner_.spellCastAnimCallbackRef()(failGuid, false, false, SpellCastType::OMNI);
        }
    }
}

void SpellHandler::handleItemCooldown(network::Packet& packet) {
    // uint64 itemGuid + uint32 spellId + uint32 cooldownMs
    size_t rem = packet.getRemainingSize();
    if (rem >= 16) {
        uint64_t itemGuid = packet.readUInt64();
        uint32_t spellId  = packet.readUInt32();
        uint32_t cdMs     = packet.readUInt32();
        float cdSec = cdMs / 1000.0f;
        if (cdSec > 0.0f) {
            if (spellId != 0) {
                auto it = spellCooldowns_.find(spellId);
                if (it == spellCooldowns_.end()) {
                    spellCooldowns_[spellId] = cdSec;
                } else {
                    it->second = mergeCooldownSeconds(it->second, cdSec);
                }
            }
            // Resolve itemId from the GUID so item-type slots are also updated
            uint32_t itemId = 0;
            auto iit = owner_.onlineItemsRef().find(itemGuid);
            if (iit != owner_.onlineItemsRef().end()) itemId = iit->second.entry;
            for (auto& slot : owner_.actionBarRef()) {
                bool match = (spellId != 0 && slot.type == ActionBarSlot::SPELL && slot.id == spellId)
                          || (itemId  != 0 && slot.type == ActionBarSlot::ITEM  && slot.id == itemId);
                if (match) {
                    float prevRemaining = slot.cooldownRemaining;
                    float merged = mergeCooldownSeconds(slot.cooldownRemaining, cdSec);
                    slot.cooldownRemaining = merged;
                    if (slot.cooldownTotal <= 0.0f || prevRemaining <= 0.0f) {
                        slot.cooldownTotal = cdSec;
                    } else {
                        slot.cooldownTotal = std::max(slot.cooldownTotal, merged);
                    }
                }
            }
            LOG_DEBUG("SMSG_ITEM_COOLDOWN: itemGuid=0x", std::hex, itemGuid, std::dec,
                      " spellId=", spellId, " itemId=", itemId, " cd=", cdSec, "s");
        }
    }
}

void SpellHandler::handleDispelFailed(network::Packet& packet) {
    // WotLK:       uint32 dispelSpellId + packed_guid caster + packed_guid victim
    //              [+ count × uint32 failedSpellId]
    // Classic:     uint32 dispelSpellId + packed_guid caster + packed_guid victim
    //              [+ count × uint32 failedSpellId]
    // TBC:         uint64 caster + uint64 victim + uint32 spellId
    //              [+ count × uint32 failedSpellId]
    const bool dispelUsesFullGuid = isActiveExpansion("tbc");
    uint32_t dispelSpellId = 0;
    uint64_t dispelCasterGuid = 0;
    if (dispelUsesFullGuid) {
        if (!packet.hasRemaining(20)) return;
        dispelCasterGuid = packet.readUInt64();
        /*uint64_t victim =*/ packet.readUInt64();
        dispelSpellId = packet.readUInt32();
    } else {
        if (!packet.hasRemaining(4)) return;
        dispelSpellId = packet.readUInt32();
        if (!packet.hasFullPackedGuid()) {
            packet.skipAll(); return;
        }
        dispelCasterGuid = packet.readPackedGuid();
        if (!packet.hasFullPackedGuid()) {
            packet.skipAll(); return;
        }
        /*uint64_t victim =*/ packet.readPackedGuid();
    }
    // Only show failure to the player who attempted the dispel
    if (dispelCasterGuid == owner_.getPlayerGuid()) {
        const auto& name = owner_.getSpellName(dispelSpellId);
        char buf[128];
        if (!name.empty())
            std::snprintf(buf, sizeof(buf), "%s failed to dispel.", name.c_str());
        else
            std::snprintf(buf, sizeof(buf), "Dispel failed! (spell %u)", dispelSpellId);
        owner_.addSystemChatMessage(buf);
    }
}

void SpellHandler::handleTotemCreated(network::Packet& packet) {
    // WotLK:       uint8 slot + packed_guid + uint32 duration + uint32 spellId
    // TBC/Classic: uint8 slot + uint64 guid  + uint32 duration + uint32 spellId
    const bool totemTbcLike = isPreWotlk();
    if (!packet.hasRemaining(totemTbcLike ? 17u : 9u) ) return;
    uint8_t slot = packet.readUInt8();
    if (totemTbcLike)
        /*uint64_t guid =*/ packet.readUInt64();
    else
        /*uint64_t guid =*/ packet.readPackedGuid();
    if (!packet.hasRemaining(8)) return;
    uint32_t duration = packet.readUInt32();
    uint32_t spellId  = packet.readUInt32();
    LOG_DEBUG("SMSG_TOTEM_CREATED: slot=", static_cast<int>(slot),
              " spellId=", spellId, " duration=", duration, "ms");
    if (slot < GameHandler::NUM_TOTEM_SLOTS) {
        owner_.activeTotemSlotsRef()[slot].spellId    = spellId;
        owner_.activeTotemSlotsRef()[slot].durationMs = duration;
        owner_.activeTotemSlotsRef()[slot].placedAt   = std::chrono::steady_clock::now();
    }
}

void SpellHandler::handlePeriodicAuraLog(network::Packet& packet) {
    // Classic, TBC, and WotLK all serialize victim and caster as packed GUIDs.
    if (!packet.hasFullPackedGuid()) return;
    uint64_t victimGuid = packet.readPackedGuid();
    if (!packet.hasFullPackedGuid()) return;
    uint64_t casterGuid = packet.readPackedGuid();
    if (!packet.hasRemaining(8)) return;
    uint32_t spellId = packet.readUInt32();
    uint32_t count   = packet.readUInt32();
    bool isPlayerVictim = (victimGuid == owner_.getPlayerGuid());
    bool isPlayerCaster = (casterGuid == owner_.getPlayerGuid());
    if (!isPlayerVictim && !isPlayerCaster) {
        packet.skipAll();
        return;
    }
    // SpellPeriodicAuraLogInfo serializes AuraType as uint32 on the wire.
    // Reading one byte leaves three zero bytes in front of the amount and
    // turns ordinary poison ticks into corrupt multi-byte damage values.
    if (count > 64) {
        LOG_WARNING("SMSG_PERIODICAURALOG: unreasonable effect count ", count);
        return;
    }
    for (uint32_t i = 0; i < count && packet.hasRemaining(4); ++i) {
        uint32_t auraType = packet.readUInt32();
        if (auraType == 3 || auraType == 89) {
            // Classic/TBC: damage(4)+school(4)+absorbed(4)+resisted(4)  = 16 bytes
            // WotLK 3.3.5a: damage(4)+overkill(4)+school(4)+absorbed(4)+resisted(4)+isCrit(1) = 21 bytes
            const bool periodicWotlk = isActiveExpansion("wotlk");
            const size_t dotSz = periodicWotlk ? 21u : 16u;
            if (!packet.hasRemaining(dotSz)) break;
            uint32_t dmg      = packet.readUInt32();
            if (periodicWotlk) /*uint32_t overkill=*/ packet.readUInt32();
            /*uint32_t school=*/ packet.readUInt32();
            uint32_t abs      = packet.readUInt32();
            uint32_t res      = packet.readUInt32();
            bool dotCrit = false;
            if (periodicWotlk) dotCrit = (packet.readUInt8() != 0);
            if (dmg > 0)
                owner_.addCombatText(dotCrit ? CombatTextEntry::CRIT_DAMAGE : CombatTextEntry::PERIODIC_DAMAGE,
                              static_cast<int32_t>(dmg),
                              spellId, isPlayerCaster, 0, casterGuid, victimGuid);
            if (abs > 0)
                owner_.addCombatText(CombatTextEntry::ABSORB, static_cast<int32_t>(abs),
                              spellId, isPlayerCaster, 0, casterGuid, victimGuid);
            if (res > 0)
                owner_.addCombatText(CombatTextEntry::RESIST, static_cast<int32_t>(res),
                              spellId, isPlayerCaster, 0, casterGuid, victimGuid);
        } else if (auraType == 8 || auraType == 20) {
            // Classic/TBC: heal(4)
            // WotLK: heal(4)+overheal(4)+absorbed(4)+isCrit(1)
            const bool healWotlk = isActiveExpansion("wotlk");
            const size_t hotSz = healWotlk ? 13u : 4u;
            if (!packet.hasRemaining(hotSz)) break;
            uint32_t heal    = packet.readUInt32();
            uint32_t hotAbs  = 0;
            bool hotCrit = false;
            if (healWotlk) {
                /*uint32_t overheal=*/ packet.readUInt32();
                hotAbs = packet.readUInt32();
                hotCrit = (packet.readUInt8() != 0);
            }
            owner_.addCombatText(hotCrit ? CombatTextEntry::CRIT_HEAL : CombatTextEntry::PERIODIC_HEAL,
                          static_cast<int32_t>(heal),
                          spellId, isPlayerCaster, 0, casterGuid, victimGuid);
            if (hotAbs > 0)
                owner_.addCombatText(CombatTextEntry::ABSORB, static_cast<int32_t>(hotAbs),
                              spellId, isPlayerCaster, 0, casterGuid, victimGuid);
        } else if (auraType == 21 || auraType == 24) {
            // OBS_MOD_POWER / PERIODIC_ENERGIZE: miscValue(powerType) + amount
            // Common in WotLK: Replenishment, Mana Spring Totem, Divine Plea, etc.
            if (!packet.hasRemaining(8)) break;
            uint8_t periodicPowerType = static_cast<uint8_t>(packet.readUInt32());
            uint32_t amount = packet.readUInt32();
            if ((isPlayerVictim || isPlayerCaster) && amount > 0)
                owner_.addCombatText(CombatTextEntry::ENERGIZE, static_cast<int32_t>(amount),
                              spellId, isPlayerCaster, periodicPowerType, casterGuid, victimGuid);
        } else if (auraType == 64) {
            // PERIODIC_MANA_LEECH: miscValue(powerType) + amount + float multiplier
            if (!packet.hasRemaining(12)) break;
            uint8_t powerType = static_cast<uint8_t>(packet.readUInt32());
            uint32_t amount = packet.readUInt32();
            float multiplier = packet.readFloat();
            if (isPlayerVictim && amount > 0)
                owner_.addCombatText(CombatTextEntry::POWER_DRAIN, static_cast<int32_t>(amount),
                              spellId, false, powerType, casterGuid, victimGuid);
            if (isPlayerCaster && amount > 0 && multiplier > 0.0f && std::isfinite(multiplier)) {
                const uint32_t gainedAmount = static_cast<uint32_t>(
                    std::lround(static_cast<double>(amount) * static_cast<double>(multiplier)));
                if (gainedAmount > 0) {
                    owner_.addCombatText(CombatTextEntry::ENERGIZE, static_cast<int32_t>(gainedAmount),
                                  spellId, true, powerType, casterGuid, casterGuid);
                }
            }
        } else {
            // Unknown/untracked aura type — stop parsing this event safely
            packet.skipAll();
            break;
        }
    }
    packet.skipAll();
}

void SpellHandler::handleSpellEnergizeLog(network::Packet& packet) {
    // WotLK: packed_guid victim + packed_guid caster + uint32 spellId + uint32 powerType + int32 amount
    // TBC: full uint64 victim + uint64 caster + uint32 spellId + uint32 powerType + int32 amount
    // Classic/Vanilla: packed_guid (same as WotLK). PowerType is uint32 in every expansion.
    const bool energizeTbc = isActiveExpansion("tbc");
    auto readEnergizeGuid = [&]() -> uint64_t {
        if (energizeTbc)
            return (packet.hasRemaining(8)) ? packet.readUInt64() : 0;
        return packet.readPackedGuid();
    };
    if (!packet.hasRemaining(energizeTbc ? 8u : 1u)
        || (!energizeTbc && !packet.hasFullPackedGuid())) {
        packet.skipAll(); return;
    }
    uint64_t victimGuid = readEnergizeGuid();
    if (!packet.hasRemaining(energizeTbc ? 8u : 1u)
        || (!energizeTbc && !packet.hasFullPackedGuid())) {
        packet.skipAll(); return;
    }
    uint64_t casterGuid = readEnergizeGuid();
    // spellId(4) + powerType(4) + amount(4) = 12. PowerType is a uint32 on the wire in
    // every expansion (Vanilla/TBC/WotLK); reading it as a uint8 left 3 bytes in front of
    // amount, turning a small power gain (e.g. 10 = 0x0000000A after powerType 0x00000001)
    // into 0x0A000000 = 167772160 floating combat text.
    if (!packet.hasRemaining(12)) {
        packet.skipAll(); return;
    }
    uint32_t spellId       = packet.readUInt32();
    uint8_t  energizePowerType = static_cast<uint8_t>(packet.readUInt32());
    int32_t  amount        = static_cast<int32_t>(packet.readUInt32());
    bool isPlayerVictim = (victimGuid == owner_.getPlayerGuid());
    bool isPlayerCaster = (casterGuid == owner_.getPlayerGuid());
    if ((isPlayerVictim || isPlayerCaster) && amount > 0)
        owner_.addCombatText(CombatTextEntry::ENERGIZE, amount, spellId, isPlayerCaster, energizePowerType, casterGuid, victimGuid);
    packet.skipAll();
}

void SpellHandler::handleExtraAuraInfo(network::Packet& packet, bool isInit) {
    // TBC 2.4.3 aura tracking: replaces SMSG_AURA_UPDATE which doesn't exist in TBC.
    // Format: uint64 targetGuid + uint8 count + N×{uint8 slot, uint32 spellId,
    //         uint8 effectIndex, uint8 flags, uint32 durationMs, uint32 maxDurationMs}
    auto remaining = [&]() { return packet.getRemainingSize(); };
    if (remaining() < 9) { packet.skipAll(); return; }
    uint64_t auraTargetGuid = packet.readUInt64();
    uint8_t count = packet.readUInt8();

    std::vector<AuraSlot>* auraList = nullptr;
    if (auraTargetGuid == owner_.getPlayerGuid())       auraList = &playerAuras_;
    else if (auraTargetGuid == owner_.getTargetGuid())   auraList = &targetAuras_;
    else if (auraTargetGuid != 0)                   auraList = &unitAurasCache_[auraTargetGuid];

    if (auraList && isInit) auraList->clear();

    uint64_t nowMs = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());

    for (uint8_t i = 0; i < count && remaining() >= 15; i++) {
        uint8_t  slot        = packet.readUInt8();   // 1 byte
        uint32_t spellId     = packet.readUInt32();  // 4 bytes
        (void)               packet.readUInt8();     // effectIndex: 1 byte (unused for slot display)
        uint8_t  flags       = packet.readUInt8();   // 1 byte
        uint32_t durationMs  = packet.readUInt32();  // 4 bytes
        uint32_t maxDurMs    = packet.readUInt32();  // 4 bytes — total 15 bytes per entry

        if (auraList) {
            while (auraList->size() <= slot) auraList->push_back(AuraSlot{});
            AuraSlot& a = (*auraList)[slot];
            a.spellId      = spellId;
            // TBC uses same flag convention as Classic: 0x02=harmful, 0x04=beneficial.
            // Normalize to WotLK SMSG_AURA_UPDATE convention: 0x80=debuff, 0=buff.
            a.flags        = (flags & 0x02) ? 0x80u : 0u;
            a.durationMs   = (durationMs == 0xFFFFFFFF) ? -1 : static_cast<int32_t>(durationMs);
            a.maxDurationMs= (maxDurMs   == 0xFFFFFFFF) ? -1 : static_cast<int32_t>(maxDurMs);
            a.receivedAtMs = nowMs;
        }
    }
    if (auraList && owner_.addonEventCallbackRef()) {
        std::string unitId;
        if (auraTargetGuid == owner_.getPlayerGuid()) unitId = "player";
        else if (auraTargetGuid == owner_.getTargetGuid()) unitId = "target";
        else if (auraTargetGuid == owner_.focusGuidRef()) unitId = "focus";
        else if (auraTargetGuid == owner_.petGuidRef()) unitId = "pet";
        if (!unitId.empty()) owner_.addonEventCallbackRef()("UNIT_AURA", {unitId});
    }
    if (auraTargetGuid == owner_.getPlayerGuid()) {
        refreshRestorationFromPlayerAuras();
    }
    packet.skipAll();
}

void SpellHandler::handleSpellDispelLog(network::Packet& packet) {
    // WotLK/Classic/Turtle: packed casterGuid + packed victimGuid + uint32 dispelSpell + uint8 isStolen
    // TBC:                  full uint64 casterGuid + full uint64 victimGuid + ...
    // + uint32 count + count × (uint32 dispelled_spellId + uint32 unk)
    const bool dispelUsesFullGuid = isActiveExpansion("tbc");
    if (!packet.hasRemaining(dispelUsesFullGuid ? 8u : 1u)
        || (!dispelUsesFullGuid && !packet.hasFullPackedGuid())) {
        packet.skipAll(); return;
    }
    uint64_t casterGuid = dispelUsesFullGuid
        ? packet.readUInt64() : packet.readPackedGuid();
    if (!packet.hasRemaining(dispelUsesFullGuid ? 8u : 1u)
        || (!dispelUsesFullGuid && !packet.hasFullPackedGuid())) {
        packet.skipAll(); return;
    }
    uint64_t victimGuid = dispelUsesFullGuid
        ? packet.readUInt64() : packet.readPackedGuid();
    if (!packet.hasRemaining(9)) return;
    /*uint32_t dispelSpell =*/ packet.readUInt32();
    uint8_t isStolen = packet.readUInt8();
    uint32_t count   = packet.readUInt32();
    // Preserve every dispelled aura in the combat log instead of collapsing
    // multi-aura packets down to the first entry only.
    const size_t dispelEntrySize = dispelUsesFullGuid ? 8u : 5u;
    std::vector<uint32_t> dispelledIds;
    dispelledIds.reserve(count);
    for (uint32_t i = 0; i < count && packet.hasRemaining(dispelEntrySize); ++i) {
        uint32_t dispelledId = packet.readUInt32();
        if (dispelUsesFullGuid) {
            /*uint32_t unk =*/ packet.readUInt32();
        } else {
            /*uint8_t isPositive =*/ packet.readUInt8();
        }
        if (dispelledId != 0) {
            dispelledIds.push_back(dispelledId);
        }
    }
    // Show system message if player was victim or caster
    if (victimGuid == owner_.getPlayerGuid() || casterGuid == owner_.getPlayerGuid()) {
        std::vector<uint32_t> loggedIds;
        if (isStolen) {
            loggedIds.reserve(dispelledIds.size());
            for (uint32_t dispelledId : dispelledIds) {
                if (owner_.shouldLogSpellstealAura(casterGuid, victimGuid, dispelledId))
                    loggedIds.push_back(dispelledId);
            }
        } else {
            loggedIds = dispelledIds;
        }

        const std::string displaySpellNames = formatSpellNameList(owner_, loggedIds);
        if (!displaySpellNames.empty()) {
            char buf[256];
            const char* passiveVerb = loggedIds.size() == 1 ? "was" : "were";
            if (isStolen) {
                if (victimGuid == owner_.getPlayerGuid() && casterGuid != owner_.getPlayerGuid())
                    std::snprintf(buf, sizeof(buf), "%s %s stolen.",
                                  displaySpellNames.c_str(), passiveVerb);
                else if (casterGuid == owner_.getPlayerGuid())
                    std::snprintf(buf, sizeof(buf), "You steal %s.", displaySpellNames.c_str());
                else
                    std::snprintf(buf, sizeof(buf), "%s %s stolen.",
                                  displaySpellNames.c_str(), passiveVerb);
            } else {
                if (victimGuid == owner_.getPlayerGuid() && casterGuid != owner_.getPlayerGuid())
                    std::snprintf(buf, sizeof(buf), "%s %s dispelled.",
                                  displaySpellNames.c_str(), passiveVerb);
                else if (casterGuid == owner_.getPlayerGuid())
                    std::snprintf(buf, sizeof(buf), "You dispel %s.", displaySpellNames.c_str());
                else
                    std::snprintf(buf, sizeof(buf), "%s %s dispelled.",
                                  displaySpellNames.c_str(), passiveVerb);
            }
            owner_.addSystemChatMessage(buf);
        }
        // Preserve stolen auras as spellsteal events so the log wording stays accurate.
        if (!loggedIds.empty()) {
            bool isPlayerCaster = (casterGuid == owner_.getPlayerGuid());
            for (uint32_t dispelledId : loggedIds) {
                owner_.addCombatText(isStolen ? CombatTextEntry::STEAL : CombatTextEntry::DISPEL,
                              0, dispelledId, isPlayerCaster, 0,
                              casterGuid, victimGuid);
            }
        }
    }
    packet.skipAll();
}

void SpellHandler::handleSpellStealLog(network::Packet& packet) {
    // Sent to the CASTER (Mage) when Spellsteal succeeds.
    // Wire format mirrors SPELLDISPELLOG:
    // WotLK/Classic/Turtle: packed victim + packed caster + uint32 spellId + uint8 isStolen + uint32 count
    //                        + count × (uint32 stolenSpellId + uint8 isPositive)
    // TBC:                   full uint64 victim + full uint64 caster + same tail
    const bool stealUsesFullGuid = isActiveExpansion("tbc");
    if (!packet.hasRemaining(stealUsesFullGuid ? 8u : 1u)
        || (!stealUsesFullGuid && !packet.hasFullPackedGuid())) {
        packet.skipAll(); return;
    }
    uint64_t stealVictim = stealUsesFullGuid
        ? packet.readUInt64() : packet.readPackedGuid();
    if (!packet.hasRemaining(stealUsesFullGuid ? 8u : 1u)
        || (!stealUsesFullGuid && !packet.hasFullPackedGuid())) {
        packet.skipAll(); return;
    }
    uint64_t stealCaster = stealUsesFullGuid
        ? packet.readUInt64() : packet.readPackedGuid();
    if (!packet.hasRemaining(9)) {
        packet.skipAll(); return;
    }
    /*uint32_t stealSpellId =*/ packet.readUInt32();
    /*uint8_t  isStolen    =*/ packet.readUInt8();
    uint32_t stealCount   = packet.readUInt32();
    // Preserve every stolen aura in the combat log instead of only the first.
    const size_t stealEntrySize = stealUsesFullGuid ? 8u : 5u;
    std::vector<uint32_t> stolenIds;
    stolenIds.reserve(stealCount);
    for (uint32_t i = 0; i < stealCount && packet.hasRemaining(stealEntrySize); ++i) {
        uint32_t stolenId = packet.readUInt32();
        if (stealUsesFullGuid) {
            /*uint32_t unk =*/ packet.readUInt32();
        } else {
            /*uint8_t isPos  =*/ packet.readUInt8();
        }
        if (stolenId != 0) {
            stolenIds.push_back(stolenId);
        }
    }
    if (stealCaster == owner_.getPlayerGuid() || stealVictim == owner_.getPlayerGuid()) {
        std::vector<uint32_t> loggedIds;
        loggedIds.reserve(stolenIds.size());
        for (uint32_t stolenId : stolenIds) {
            if (owner_.shouldLogSpellstealAura(stealCaster, stealVictim, stolenId))
                loggedIds.push_back(stolenId);
        }

        const std::string stealDisplayNames = formatSpellNameList(owner_, loggedIds);
        if (!stealDisplayNames.empty()) {
            char buf[256];
            if (stealCaster == owner_.getPlayerGuid())
                std::snprintf(buf, sizeof(buf), "You stole %s.", stealDisplayNames.c_str());
            else
                std::snprintf(buf, sizeof(buf), "%s %s stolen.", stealDisplayNames.c_str(),
                              loggedIds.size() == 1 ? "was" : "were");
            owner_.addSystemChatMessage(buf);
        }
        // Some servers emit both SPELLDISPELLOG(isStolen=1) and SPELLSTEALLOG
        // for the same aura. Keep the first event and suppress the duplicate.
        if (!loggedIds.empty()) {
            bool isPlayerCaster = (stealCaster == owner_.getPlayerGuid());
            for (uint32_t stolenId : loggedIds) {
                owner_.addCombatText(CombatTextEntry::STEAL, 0, stolenId, isPlayerCaster, 0,
                              stealCaster, stealVictim);
            }
        }
    }
    packet.skipAll();
}

void SpellHandler::handleSpellChanceProcLog(network::Packet& packet) {
    // WotLK/Classic/Turtle: packed_guid target + packed_guid caster + uint32 spellId + ...
    // TBC:                  uint64 target + uint64 caster + uint32 spellId + ...
    const bool procChanceUsesFullGuid = isActiveExpansion("tbc");
    auto readProcChanceGuid = [&]() -> uint64_t {
        if (procChanceUsesFullGuid)
            return (packet.hasRemaining(8)) ? packet.readUInt64() : 0;
        return packet.readPackedGuid();
    };
    if (!packet.hasRemaining(procChanceUsesFullGuid ? 8u : 1u)
        || (!procChanceUsesFullGuid && !packet.hasFullPackedGuid())) {
        packet.skipAll(); return;
    }
    uint64_t procTargetGuid = readProcChanceGuid();
    if (!packet.hasRemaining(procChanceUsesFullGuid ? 8u : 1u)
        || (!procChanceUsesFullGuid && !packet.hasFullPackedGuid())) {
        packet.skipAll(); return;
    }
    uint64_t procCasterGuid = readProcChanceGuid();
    if (!packet.hasRemaining(4)) {
        packet.skipAll(); return;
    }
    uint32_t procSpellId = packet.readUInt32();
    // Show a "PROC!" floating text when the player triggers the proc
    if (procCasterGuid == owner_.getPlayerGuid() && procSpellId > 0)
        owner_.addCombatText(CombatTextEntry::PROC_TRIGGER, 0, procSpellId, true, 0,
                      procCasterGuid, procTargetGuid);
    packet.skipAll();
}

void SpellHandler::handleSpellInstaKillLog(network::Packet& packet) {
    // Sent when a unit is killed by a spell with SPELL_ATTR_EX2_INSTAKILL (e.g. Execute, Obliterate, etc.)
    // WotLK/Classic/Turtle: packed_guid caster + packed_guid victim + uint32 spellId
    // TBC:                  full uint64 caster + full uint64 victim + uint32 spellId
    const bool ikUsesFullGuid = isActiveExpansion("tbc");
    auto ik_rem = [&]() { return packet.getRemainingSize(); };
    if (ik_rem() < (ikUsesFullGuid ? 8u : 1u)
        || (!ikUsesFullGuid && !packet.hasFullPackedGuid())) {
        packet.skipAll(); return;
    }
    uint64_t ikCaster = ikUsesFullGuid
        ? packet.readUInt64() : packet.readPackedGuid();
    if (ik_rem() < (ikUsesFullGuid ? 8u : 1u)
        || (!ikUsesFullGuid && !packet.hasFullPackedGuid())) {
        packet.skipAll(); return;
    }
    uint64_t ikVictim = ikUsesFullGuid
        ? packet.readUInt64() : packet.readPackedGuid();
    if (ik_rem() < 4) {
        packet.skipAll(); return;
    }
    uint32_t ikSpell = packet.readUInt32();
    // Show kill/death feedback for the local player
    if (ikCaster == owner_.getPlayerGuid()) {
        owner_.addCombatText(CombatTextEntry::INSTAKILL, 0, ikSpell, true, 0, ikCaster, ikVictim);
    } else if (ikVictim == owner_.getPlayerGuid()) {
        owner_.addCombatText(CombatTextEntry::INSTAKILL, 0, ikSpell, false, 0, ikCaster, ikVictim);
        owner_.addUIError("You were killed by an instant-kill effect.");
        owner_.addSystemChatMessage("You were killed by an instant-kill effect.");
    }
    LOG_DEBUG("SMSG_SPELLINSTAKILLLOG: caster=0x", std::hex, ikCaster,
              " victim=0x", ikVictim, std::dec, " spell=", ikSpell);
    packet.skipAll();
}

// ---- handleSpellLogExecute per-effect parsers (extracted to reduce nesting) ----

void SpellHandler::parseEffectPowerDrain(network::Packet& packet, uint32_t effectLogCount,
                                          uint64_t caster, uint32_t spellId,
                                          bool isPlayerCaster, bool usesFullGuid) {
    // SPELL_EFFECT_POWER_DRAIN: packed_guid target + uint32 amount + uint32 powerType + float multiplier
    const uint64_t playerGuid = owner_.getPlayerGuid();
    for (uint32_t li = 0; li < effectLogCount; ++li) {
        if (!packet.hasRemaining(usesFullGuid ? 8u : 1u)
            || (!usesFullGuid && !packet.hasFullPackedGuid())) {
            packet.skipAll(); break;
        }
        uint64_t drainTarget = usesFullGuid ? packet.readUInt64() : packet.readPackedGuid();
        if (!packet.hasRemaining(12)) { packet.skipAll(); break; }
        uint32_t drainAmount = packet.readUInt32();
        uint32_t drainPower  = packet.readUInt32(); // 0=mana,1=rage,3=energy,6=runic
        float    drainMult   = packet.readFloat();

        LOG_DEBUG("SMSG_SPELLLOGEXECUTE POWER_DRAIN: spell=", spellId,
                  " power=", drainPower, " amount=", drainAmount,
                  " multiplier=", drainMult);
        if (drainAmount == 0) continue;

        const auto powerByte = static_cast<uint8_t>(drainPower);
        if (drainTarget == playerGuid)
            owner_.addCombatText(CombatTextEntry::POWER_DRAIN,
                                 static_cast<int32_t>(drainAmount), spellId, false,
                                 powerByte, caster, drainTarget);
        if (!isPlayerCaster) continue;
        if (drainTarget != playerGuid)
            owner_.addCombatText(CombatTextEntry::POWER_DRAIN,
                                 static_cast<int32_t>(drainAmount), spellId, true,
                                 powerByte, caster, drainTarget);
        if (drainMult <= 0.0f || !std::isfinite(drainMult)) continue;
        const uint32_t gained = static_cast<uint32_t>(
            std::lround(static_cast<double>(drainAmount) * static_cast<double>(drainMult)));
        if (gained > 0)
            owner_.addCombatText(CombatTextEntry::ENERGIZE,
                                 static_cast<int32_t>(gained), spellId, true,
                                 powerByte, caster, caster);
    }
}

void SpellHandler::parseEffectHealthLeech(network::Packet& packet, uint32_t effectLogCount,
                                           uint64_t caster, uint32_t spellId,
                                           bool isPlayerCaster, bool usesFullGuid) {
    // SPELL_EFFECT_HEALTH_LEECH: packed_guid target + uint32 amount + float multiplier
    const uint64_t playerGuid = owner_.getPlayerGuid();
    for (uint32_t li = 0; li < effectLogCount; ++li) {
        if (!packet.hasRemaining(usesFullGuid ? 8u : 1u)
            || (!usesFullGuid && !packet.hasFullPackedGuid())) {
            packet.skipAll(); break;
        }
        uint64_t leechTarget = usesFullGuid ? packet.readUInt64() : packet.readPackedGuid();
        if (!packet.hasRemaining(8)) { packet.skipAll(); break; }
        uint32_t leechAmount = packet.readUInt32();
        float    leechMult   = packet.readFloat();

        LOG_DEBUG("SMSG_SPELLLOGEXECUTE HEALTH_LEECH: spell=", spellId,
                  " amount=", leechAmount, " multiplier=", leechMult);
        if (leechAmount == 0) continue;

        if (leechTarget == playerGuid) {
            owner_.addCombatText(CombatTextEntry::SPELL_DAMAGE,
                                 static_cast<int32_t>(leechAmount), spellId, false, 0,
                                 caster, leechTarget);
        } else if (isPlayerCaster) {
            owner_.addCombatText(CombatTextEntry::SPELL_DAMAGE,
                                 static_cast<int32_t>(leechAmount), spellId, true, 0,
                                 caster, leechTarget);
        }
        if (!isPlayerCaster || leechMult <= 0.0f || !std::isfinite(leechMult)) continue;
        const uint32_t gained = static_cast<uint32_t>(
            std::lround(static_cast<double>(leechAmount) * static_cast<double>(leechMult)));
        if (gained > 0)
            owner_.addCombatText(CombatTextEntry::HEAL,
                                 static_cast<int32_t>(gained), spellId, true, 0,
                                 caster, caster);
    }
}

void SpellHandler::parseEffectCreateItem(network::Packet& packet, uint32_t effectLogCount,
                                          uint64_t /*caster*/, uint32_t spellId,
                                          bool isPlayerCaster) {
    // SPELL_EFFECT_CREATE_ITEM / CREATE_ITEM2: uint32 itemEntry per log entry
    for (uint32_t li = 0; li < effectLogCount; ++li) {
        if (!packet.hasRemaining(4)) break;
        uint32_t itemEntry = packet.readUInt32();
        if (!isPlayerCaster || itemEntry == 0) continue;

        owner_.ensureItemInfo(itemEntry);
        const ItemQueryResponseData* info = owner_.getItemInfo(itemEntry);
        std::string itemName = (info && !info->name.empty())
            ? info->name : ("item #" + std::to_string(itemEntry));
        const auto& spellName = owner_.getSpellName(spellId);
        std::string msg = spellName.empty()
            ? ("You create: " + itemName + ".")
            : ("You create " + itemName + " using " + spellName + ".");
        owner_.addSystemChatMessage(msg);
        LOG_DEBUG("SMSG_SPELLLOGEXECUTE CREATE_ITEM: spell=", spellId,
                  " item=", itemEntry, " name=", itemName);
    }
}

void SpellHandler::parseEffectInterruptCast(network::Packet& packet, uint32_t effectLogCount,
                                             uint64_t caster, uint32_t spellId,
                                             bool isPlayerCaster, bool usesFullGuid) {
    // SPELL_EFFECT_INTERRUPT_CAST: packed_guid target + uint32 interrupted_spell_id
    const uint64_t playerGuid = owner_.getPlayerGuid();
    for (uint32_t li = 0; li < effectLogCount; ++li) {
        if (!packet.hasRemaining(usesFullGuid ? 8u : 1u)
            || (!usesFullGuid && !packet.hasFullPackedGuid())) {
            packet.skipAll(); break;
        }
        uint64_t icTarget = usesFullGuid ? packet.readUInt64() : packet.readPackedGuid();
        if (!packet.hasRemaining(4)) { packet.skipAll(); break; }
        uint32_t icSpellId = packet.readUInt32();
        // Clear the interrupted unit's cast bar immediately
        unitCastStates_.erase(icTarget);
        // Record interrupt in combat log when player is involved
        if (isPlayerCaster || icTarget == playerGuid)
            owner_.addCombatText(CombatTextEntry::INTERRUPT, 0, icSpellId, isPlayerCaster, 0,
                                 caster, icTarget);
        LOG_DEBUG("SMSG_SPELLLOGEXECUTE INTERRUPT_CAST: spell=", spellId,
                  " interrupted=", icSpellId, " target=0x", std::hex, icTarget, std::dec);
    }
}

void SpellHandler::parseEffectFeedPet(network::Packet& packet, uint32_t effectLogCount,
                                       uint64_t /*caster*/, uint32_t /*spellId*/,
                                       bool isPlayerCaster) {
    // SPELL_EFFECT_FEED_PET: uint32 itemEntry per log entry
    for (uint32_t li = 0; li < effectLogCount; ++li) {
        if (!packet.hasRemaining(4)) break;
        uint32_t feedItem = packet.readUInt32();
        if (!isPlayerCaster || feedItem == 0) continue;

        owner_.ensureItemInfo(feedItem);
        const ItemQueryResponseData* info = owner_.getItemInfo(feedItem);
        std::string itemName = (info && !info->name.empty())
            ? info->name : ("item #" + std::to_string(feedItem));
        uint32_t feedQuality = info ? info->quality : 1u;
        owner_.addSystemChatMessage("You feed your pet " +
                                     buildItemLink(feedItem, feedQuality, itemName) + ".");
        LOG_DEBUG("SMSG_SPELLLOGEXECUTE FEED_PET: item=", feedItem, " name=", itemName);
    }
}

void SpellHandler::handleSpellLogExecute(network::Packet& packet) {
    // WotLK/Classic/Turtle: packed_guid caster + uint32 spellId + uint32 effectCount
    // TBC:                  uint64 caster + uint32 spellId + uint32 effectCount
    // Per-effect: uint8 effectType + uint32 effectLogCount + effect-specific data
    // Effect 10 = POWER_DRAIN:   packed_guid target + uint32 amount + uint32 powerType + float multiplier
    // Effect 11 = HEALTH_LEECH:  packed_guid target + uint32 amount + float multiplier
    // Effect 24 = CREATE_ITEM:   uint32 itemEntry
    // Effect 26 = INTERRUPT_CAST: packed_guid target + uint32 interrupted_spell_id
    // Effect 49 = FEED_PET:      uint32 itemEntry
    // Effect 114= CREATE_ITEM2:  uint32 itemEntry (same layout as CREATE_ITEM)
    const bool exeUsesFullGuid = isActiveExpansion("tbc");
    if (!packet.hasRemaining(exeUsesFullGuid ? 8u : 1u) ) {
        packet.skipAll(); return;
    }
    if (!exeUsesFullGuid && !packet.hasFullPackedGuid()) {
        packet.skipAll(); return;
    }
    uint64_t exeCaster = exeUsesFullGuid
        ? packet.readUInt64() : packet.readPackedGuid();
    if (!packet.hasRemaining(8)) {
        packet.skipAll(); return;
    }
    uint32_t exeSpellId = packet.readUInt32();
    uint32_t exeEffectCount = packet.readUInt32();
    exeEffectCount = std::min(exeEffectCount, 32u); // sanity

    const bool isPlayerCaster = (exeCaster == owner_.getPlayerGuid());
    for (uint32_t ei = 0; ei < exeEffectCount; ++ei) {
        if (!packet.hasRemaining(8)) break;
        uint32_t effectType     = packet.readUInt32();
        uint32_t effectLogCount = packet.readUInt32();
        effectLogCount = std::min(effectLogCount, 64u); // sanity

        if (effectType == SpellEffect::POWER_DRAIN) {
            parseEffectPowerDrain(packet, effectLogCount, exeCaster, exeSpellId,
                                  isPlayerCaster, exeUsesFullGuid);
        } else if (effectType == SpellEffect::HEALTH_LEECH) {
            parseEffectHealthLeech(packet, effectLogCount, exeCaster, exeSpellId,
                                   isPlayerCaster, exeUsesFullGuid);
        } else if (effectType == SpellEffect::CREATE_ITEM || effectType == SpellEffect::CREATE_ITEM2) {
            parseEffectCreateItem(packet, effectLogCount, exeCaster, exeSpellId,
                                  isPlayerCaster);
        } else if (effectType == SpellEffect::INTERRUPT_CAST) {
            parseEffectInterruptCast(packet, effectLogCount, exeCaster, exeSpellId,
                                     isPlayerCaster, exeUsesFullGuid);
        } else if (effectType == SpellEffect::FEED_PET) {
            parseEffectFeedPet(packet, effectLogCount, exeCaster, exeSpellId,
                               isPlayerCaster);
        } else {
            // Unknown effect type — stop parsing to avoid misalignment
            packet.skipAll();
            break;
        }
    }
    packet.skipAll();
}

void SpellHandler::handleClearExtraAuraInfo(network::Packet& packet) {
    // TBC 2.4.3: clear a single aura slot for a unit
    // Format: uint64 targetGuid + uint8 slot
    if (packet.hasRemaining(9)) {
        uint64_t clearGuid  = packet.readUInt64();
        uint8_t  slot       = packet.readUInt8();
        std::vector<AuraSlot>* auraList = nullptr;
        if (clearGuid == owner_.getPlayerGuid())       auraList = &playerAuras_;
        else if (clearGuid == owner_.getTargetGuid())   auraList = &targetAuras_;
        else if (clearGuid != 0)                         auraList = &unitAurasCache_[clearGuid];
        if (auraList && slot < auraList->size()) {
            (*auraList)[slot] = AuraSlot{};
        }
        if (auraList && owner_.addonEventCallbackRef()) {
            std::string unitId;
            if (clearGuid == owner_.getPlayerGuid()) unitId = "player";
            else if (clearGuid == owner_.getTargetGuid()) unitId = "target";
            else if (clearGuid == owner_.focusGuidRef()) unitId = "focus";
            else if (clearGuid == owner_.petGuidRef()) unitId = "pet";
            if (!unitId.empty()) owner_.addonEventCallbackRef()("UNIT_AURA", {unitId});
        }
        if (clearGuid == owner_.getPlayerGuid()) {
            refreshRestorationFromPlayerAuras();
        }
    }
    packet.skipAll();
}

void SpellHandler::handleItemEnchantTimeUpdate(network::Packet& packet) {
    // Format: uint64 itemGuid + uint32 enchantmentSlot + uint32 durationSec + uint64 playerGuid
    //
    // The slot here is the item's *enchantment* slot (TEMP_ENCHANTMENT_SLOT = 1),
    // not the equipment slot — reading it as one labels every temporary enchant
    // "Off Hand", even on a two-hander. The item GUID is what says where it sits.
    if (!packet.hasRemaining(24)) {
        packet.skipAll(); return;
    }
    uint64_t itemGuid    = packet.readUInt64();
    /*uint32_t enchantmentSlot =*/ packet.readUInt32();
    uint32_t durationSec = packet.readUInt32();
    /*uint64_t playerGuid =*/ packet.readUInt64();

    if (itemGuid == 0) return;

    // Map the enchanted item to the weapon slot it is equipped in.
    static constexpr EquipSlot kWeaponSlots[] = {
        EquipSlot::MAIN_HAND, EquipSlot::OFF_HAND, EquipSlot::RANGED
    };
    uint32_t enchSlot = 0xFFFFFFFF;
    for (uint32_t i = 0; i < 3; ++i) {
        if (owner_.getEquipSlotGuid(static_cast<int>(kWeaponSlots[i])) == itemGuid) {
            enchSlot = i;
            break;
        }
    }
    if (enchSlot > 2) return;  // enchanted item is not an equipped weapon

    uint64_t nowMs = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());

    if (durationSec == 0) {
        // Enchant expired / removed — erase the slot entry
        owner_.tempEnchantTimersRef().erase(
            std::remove_if(owner_.tempEnchantTimersRef().begin(), owner_.tempEnchantTimersRef().end(),
                           [enchSlot](const GameHandler::TempEnchantTimer& t) { return t.slot == enchSlot; }),
            owner_.tempEnchantTimersRef().end());
    } else {
        uint64_t expireMs = nowMs + static_cast<uint64_t>(durationSec) * 1000u;
        bool found = false;
        for (auto& t : owner_.tempEnchantTimersRef()) {
            if (t.slot == enchSlot) { t.expireMs = expireMs; found = true; break; }
        }
        if (!found) owner_.tempEnchantTimersRef().push_back({enchSlot, expireMs});

        // Warn at important thresholds
        if (durationSec <= 60 && durationSec > 55) {
            const char* slotName = (enchSlot < 3) ? owner_.kTempEnchantSlotNames[enchSlot] : "weapon";
            char buf[80];
            std::snprintf(buf, sizeof(buf), "Weapon enchant (%s) expires in 1 minute!", slotName);
            owner_.addSystemChatMessage(buf);
        } else if (durationSec <= 300 && durationSec > 295) {
            const char* slotName = (enchSlot < 3) ? owner_.kTempEnchantSlotNames[enchSlot] : "weapon";
            char buf[80];
            std::snprintf(buf, sizeof(buf), "Weapon enchant (%s) expires in 5 minutes.", slotName);
            owner_.addSystemChatMessage(buf);
        }
    }
    LOG_DEBUG("SMSG_ITEM_ENCHANT_TIME_UPDATE: slot=", enchSlot, " dur=", durationSec, "s");
}

void SpellHandler::handleResumeCastBar(network::Packet& packet) {
    // WotLK: packed_guid caster + packed_guid target + uint32 spellId + uint32 remainingMs + uint32 totalMs + uint8 schoolMask
    // TBC/Classic: uint64 caster + uint64 target + ...
    const bool rcbTbc = isPreWotlk();
    auto remaining = [&]() { return packet.getRemainingSize(); };
    if (remaining() < (rcbTbc ? 8u : 1u)) return;
    uint64_t caster = rcbTbc
        ? packet.readUInt64() : packet.readPackedGuid();
    if (remaining() < (rcbTbc ? 8u : 1u)) return;
    if (rcbTbc) packet.readUInt64(); // target (discard)
    else (void)packet.readPackedGuid(); // target
    if (remaining() < 12) return;
    uint32_t spellId   = packet.readUInt32();
    uint32_t remainMs  = packet.readUInt32();
    uint32_t totalMs   = packet.readUInt32();
    if (totalMs > 0) {
        if (caster == owner_.getPlayerGuid()) {
            casting_            = true;
            castIsChannel_      = false;
            currentCastSpellId_ = spellId;
            castTimeTotal_      = totalMs  / 1000.0f;
            castTimeRemaining_  = remainMs / 1000.0f;
        } else {
            auto& s = unitCastStates_[caster];
            s.casting       = true;
            s.spellId       = spellId;
            s.timeTotal     = totalMs  / 1000.0f;
            s.timeRemaining = remainMs / 1000.0f;
        }
        LOG_DEBUG("SMSG_RESUME_CAST_BAR: caster=0x", std::hex, caster, std::dec,
                  " spell=", spellId, " remaining=", remainMs, "ms total=", totalMs, "ms");
    }
}

void SpellHandler::handleChannelStart(network::Packet& packet) {
    // casterGuid + uint32 spellId + uint32 totalDurationMs
    const bool tbcOrClassic = isPreWotlk();
    uint64_t chanCaster = tbcOrClassic
        ? (packet.hasRemaining(8) ? packet.readUInt64() : 0)
        : packet.readPackedGuid();
    if (!packet.hasRemaining(8)) return;
    uint32_t chanSpellId = packet.readUInt32();
    uint32_t chanTotalMs = packet.readUInt32();
    if (chanTotalMs > 0 && chanCaster != 0) {
        if (chanCaster == owner_.getPlayerGuid()) {
            casting_            = true;
            castIsChannel_      = true;
            currentCastSpellId_ = chanSpellId;
            castTimeTotal_      = chanTotalMs / 1000.0f;
            castTimeRemaining_  = castTimeTotal_;
        } else {
            auto& s = unitCastStates_[chanCaster];
            s.casting        = true;
            s.isChannel      = true;
            s.spellId        = chanSpellId;
            s.timeTotal      = chanTotalMs / 1000.0f;
            s.timeRemaining  = s.timeTotal;
            s.interruptible  = owner_.isSpellInterruptible(chanSpellId);
        }
        LOG_DEBUG("MSG_CHANNEL_START: caster=0x", std::hex, chanCaster, std::dec,
                  " spell=", chanSpellId, " total=", chanTotalMs, "ms");

        // Play channeling animation (looping)
        // Channel packets don't carry targetGuid — use player's current target as hint
        SpellCastType chanType = SpellCastType::OMNI;
        if (chanCaster == owner_.getPlayerGuid() && owner_.getTargetGuid() != 0)
            chanType = SpellCastType::DIRECTED;
        if (owner_.spellCastAnimCallbackRef()) {
            owner_.spellCastAnimCallbackRef()(chanCaster, true, true, chanType);
        }

        // Fire UNIT_SPELLCAST_CHANNEL_START for Lua addons
        if (owner_.addonEventCallbackRef()) {
            auto unitId = owner_.guidToUnitId(chanCaster);
            if (!unitId.empty())
                owner_.fireAddonEvent("UNIT_SPELLCAST_CHANNEL_START", {unitId, std::to_string(chanSpellId)});
        }
    }
}

void SpellHandler::handleChannelUpdate(network::Packet& packet) {
    // casterGuid + uint32 remainingMs
    const bool tbcOrClassic2 = isPreWotlk();
    uint64_t chanCaster2 = tbcOrClassic2
        ? (packet.hasRemaining(8) ? packet.readUInt64() : 0)
        : packet.readPackedGuid();
    if (!packet.hasRemaining(4)) return;
    uint32_t chanRemainMs = packet.readUInt32();
    if (chanCaster2 == owner_.getPlayerGuid()) {
        castTimeRemaining_ = chanRemainMs / 1000.0f;
        if (chanRemainMs == 0) {
            casting_ = false;
            castIsChannel_ = false;
            currentCastSpellId_ = 0;
        }
    } else if (chanCaster2 != 0) {
        auto it = unitCastStates_.find(chanCaster2);
        if (it != unitCastStates_.end()) {
            it->second.timeRemaining = chanRemainMs / 1000.0f;
            if (chanRemainMs == 0) unitCastStates_.erase(it);
        }
    }
    LOG_DEBUG("MSG_CHANNEL_UPDATE: caster=0x", std::hex, chanCaster2, std::dec,
              " remaining=", chanRemainMs, "ms");
    // Fire UNIT_SPELLCAST_CHANNEL_STOP when channel ends
    if (chanRemainMs == 0) {
        // Stop channeling animation — return to idle
        if (owner_.spellCastAnimCallbackRef()) {
            owner_.spellCastAnimCallbackRef()(chanCaster2, false, true, SpellCastType::OMNI);
        }
        auto unitId = owner_.guidToUnitId(chanCaster2);
        if (!unitId.empty())
            owner_.fireAddonEvent("UNIT_SPELLCAST_CHANNEL_STOP", {unitId});
    }
}

// ============================================================
// Pet Stable
// ============================================================

void SpellHandler::requestStabledPetList() {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket() || owner_.stableMasterGuidRef() == 0) return;
    auto pkt = ListStabledPetsPacket::build(owner_.stableMasterGuidRef());
    owner_.getSocket()->send(pkt);
    LOG_INFO("Sent MSG_LIST_STABLED_PETS to npc=0x", std::hex, owner_.stableMasterGuidRef(), std::dec);
}

void SpellHandler::stablePet(uint8_t slot) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket() || owner_.stableMasterGuidRef() == 0) return;
    if (owner_.petGuidRef() == 0) {
        owner_.addSystemChatMessage("You do not have an active pet to stable.");
        return;
    }
    auto pkt = StablePetPacket::build(owner_.stableMasterGuidRef(), slot);
    owner_.getSocket()->send(pkt);
    LOG_INFO("Sent CMSG_STABLE_PET: slot=", static_cast<int>(slot));
}

void SpellHandler::unstablePet(uint32_t petNumber) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket() || owner_.stableMasterGuidRef() == 0 || petNumber == 0) return;
    auto pkt = UnstablePetPacket::build(owner_.stableMasterGuidRef(), petNumber);
    owner_.getSocket()->send(pkt);
    LOG_INFO("Sent CMSG_UNSTABLE_PET: petNumber=", petNumber);
}

} // namespace game
} // namespace wowee
