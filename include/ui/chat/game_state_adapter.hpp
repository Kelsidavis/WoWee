// GameStateAdapter - wraps GameHandler + Renderer to implement IGameState.
// Phase 4.2 of chat_panel_ref.md.
#pragma once

#include "ui/chat/i_game_state.hpp"

namespace wowee {
namespace game { class GameHandler; }
namespace rendering { class Renderer; }

namespace ui {

/**
 * Concrete adapter from GameHandler + Renderer → IGameState.
 * Flatten complex entity/aura queries into the simple IGameState interface.
 */
class GameStateAdapter : public IGameState {
public:
    GameStateAdapter(game::GameHandler& gameHandler, rendering::Renderer* renderer);

    // --- GUIDs ---
    [[nodiscard]] uint64_t getPlayerGuid() const override;
    [[nodiscard]] uint64_t getTargetGuid() const override;
    [[nodiscard]] uint64_t getFocusGuid() const override;
    [[nodiscard]] uint64_t getPetGuid() const override;
    [[nodiscard]] uint64_t getMouseoverGuid() const override;

    // --- Player state ---
    [[nodiscard]] bool isInCombat() const override;
    [[nodiscard]] bool isMounted() const override;
    [[nodiscard]] bool isSwimming() const override;
    [[nodiscard]] bool isFlying() const override;
    [[nodiscard]] bool isCasting() const override;
    [[nodiscard]] bool isChanneling() const override;
    [[nodiscard]] bool isStealthed() const override;
    [[nodiscard]] bool hasPet() const override;
    [[nodiscard]] bool isInGroup() const override;
    [[nodiscard]] bool isInRaid() const override;
    [[nodiscard]] bool isIndoors() const override;

    // --- Numeric ---
    [[nodiscard]] uint8_t getActiveTalentSpec() const override;
    [[nodiscard]] uint32_t getVehicleId() const override;
    [[nodiscard]] uint32_t getCurrentCastSpellId() const override;

    // --- Spell/aura ---
    [[nodiscard]] std::string getSpellName(uint32_t spellId) const override;
    [[nodiscard]] bool hasAuraByName(uint64_t targetGuid, const std::string& spellName,
                       bool wantDebuff) const override;
    [[nodiscard]] bool hasFormAura() const override;

    // --- Entity queries ---
    [[nodiscard]] bool entityExists(uint64_t guid) const override;
    [[nodiscard]] bool entityIsDead(uint64_t guid) const override;
    [[nodiscard]] bool entityIsHostile(uint64_t guid) const override;

private:
    game::GameHandler& gameHandler_;
    rendering::Renderer* renderer_;
};

} // namespace ui
} // namespace wowee
