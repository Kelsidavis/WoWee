// GameStateAdapter - concrete IGameState wrapping GameHandler + Renderer.
// Phase 4.2 of chat_panel_ref.md.
#include "ui/chat/game_state_adapter.hpp"
#include "game/game_handler.hpp"
#include "game/character.hpp"
#include "rendering/renderer.hpp"
#include <algorithm>
#include <cctype>

namespace wowee { namespace ui {

GameStateAdapter::GameStateAdapter(game::GameHandler& gameHandler,
                                   rendering::Renderer* renderer)
    : gameHandler_(gameHandler), renderer_(renderer) {}

// --- GUIDs ---
uint64_t GameStateAdapter::getPlayerGuid() const { return gameHandler_.getPlayerGuid(); }
uint64_t GameStateAdapter::getTargetGuid() const { return gameHandler_.getTargetGuid(); }
uint64_t GameStateAdapter::getFocusGuid() const  { return gameHandler_.getFocusGuid(); }
uint64_t GameStateAdapter::getPetGuid() const    { return gameHandler_.getPetGuid(); }
uint64_t GameStateAdapter::getMouseoverGuid() const { return gameHandler_.getMouseoverGuid(); }

// --- Player state ---
bool GameStateAdapter::isInCombat() const  { return gameHandler_.isInCombat(); }
bool GameStateAdapter::isMounted() const   { return gameHandler_.isMounted(); }
bool GameStateAdapter::isSwimming() const  { return gameHandler_.isSwimming(); }
bool GameStateAdapter::isFlying() const    { return gameHandler_.isPlayerFlying(); }
bool GameStateAdapter::isCasting() const   { return gameHandler_.isCasting(); }
bool GameStateAdapter::isChanneling() const { return gameHandler_.isChanneling(); }

bool GameStateAdapter::isStealthed() const {
    auto pe = gameHandler_.getEntityManager().getEntity(gameHandler_.getPlayerGuid());
    if (!pe) return false;
    auto pu = std::dynamic_pointer_cast<game::Unit>(pe);
    return pu && pu->hasCreepVisibility();
}

bool GameStateAdapter::hasPet() const   { return gameHandler_.hasPet(); }
bool GameStateAdapter::isInGroup() const { return gameHandler_.isInGroup(); }

bool GameStateAdapter::isInRaid() const {
    return gameHandler_.isInGroup() &&
           game::groupIsRaid(gameHandler_.getPartyData().groupType);
}

bool GameStateAdapter::isIndoors() const {
    return renderer_ && renderer_->isPlayerIndoors();
}

// --- Numeric ---
uint8_t GameStateAdapter::getActiveTalentSpec() const { return gameHandler_.getActiveTalentSpec(); }
uint32_t GameStateAdapter::getVehicleId() const { return gameHandler_.getVehicleId(); }
uint32_t GameStateAdapter::getCurrentCastSpellId() const { return gameHandler_.getCurrentCastSpellId(); }

// --- Spell/aura ---
std::string GameStateAdapter::getSpellName(uint32_t spellId) const {
    return gameHandler_.getSpellName(spellId);
}

bool GameStateAdapter::hasAuraByName(uint64_t targetGuid, const std::string& spellName,
                                     bool wantDebuff) const {
    // If targetGuid is player or invalid, check player auras; otherwise target auras
    const std::vector<game::AuraSlot>* auras = nullptr;
    uint64_t playerGuid = gameHandler_.getPlayerGuid();
    if (targetGuid != static_cast<uint64_t>(-1) && targetGuid != 0 &&
        targetGuid != playerGuid) {
        auras = &gameHandler_.getTargetAuras();
    } else {
        auras = &gameHandler_.getPlayerAuras();
    }

    std::string nameLow = spellName;
    for (char& ch : nameLow) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));

    for (const auto& a : *auras) {
        if (a.isEmpty() || a.spellId == 0) continue;
        bool isDebuff = (a.flags & 0x80) != 0;
        if (wantDebuff ? !isDebuff : isDebuff) continue;
        std::string sn = gameHandler_.getSpellName(a.spellId);
        for (char& ch : sn) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        if (sn == nameLow) return true;
    }
    return false;
}

bool GameStateAdapter::hasFormAura() const {
    for (const auto& a : gameHandler_.getPlayerAuras()) {
        if (!a.isEmpty() && a.maxDurationMs == -1) return true;
    }
    return false;
}

// --- Entity queries ---
bool GameStateAdapter::entityExists(uint64_t guid) const {
    if (guid == 0 || guid == static_cast<uint64_t>(-1)) return false;
    return gameHandler_.getEntityManager().getEntity(guid) != nullptr;
}

bool GameStateAdapter::entityIsDead(uint64_t guid) const {
    auto entity = gameHandler_.getEntityManager().getEntity(guid);
    if (!entity) return false;
    auto unit = std::dynamic_pointer_cast<game::Unit>(entity);
    return unit && unit->getHealth() == 0;
}

bool GameStateAdapter::entityIsHostile(uint64_t guid) const {
    auto entity = gameHandler_.getEntityManager().getEntity(guid);
    if (!entity) return false;
    auto unit = std::dynamic_pointer_cast<game::Unit>(entity);
    return unit && gameHandler_.isHostileFactionPublic(unit->getFactionTemplate());
}

} // namespace ui
} // namespace wowee
