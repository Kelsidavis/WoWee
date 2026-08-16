// IGameState - abstract interface for game state queries used by macro evaluation.
// Allows unit testing with mock state. Phase 4.1 of chat_panel_ref.md.
#pragma once

#include <cstdint>
#include <string>

namespace wowee {
namespace ui {

/**
 * Read-only view of game state for macro conditional evaluation.
 *
 * All entity/aura queries are flattened to simple types so callers
 * don't need to depend on game::Entity, game::Unit, etc.
 */
class IGameState {
public:
    virtual ~IGameState() = default;

    // --- GUIDs ---
    [[nodiscard]] virtual uint64_t getPlayerGuid() const = 0;
    [[nodiscard]] virtual uint64_t getTargetGuid() const = 0;
    [[nodiscard]] virtual uint64_t getFocusGuid() const = 0;
    [[nodiscard]] virtual uint64_t getPetGuid() const = 0;
    [[nodiscard]] virtual uint64_t getMouseoverGuid() const = 0;

    // --- Player state booleans ---
    [[nodiscard]] virtual bool isInCombat() const = 0;
    [[nodiscard]] virtual bool isMounted() const = 0;
    [[nodiscard]] virtual bool isSwimming() const = 0;
    [[nodiscard]] virtual bool isFlying() const = 0;
    [[nodiscard]] virtual bool isCasting() const = 0;
    [[nodiscard]] virtual bool isChanneling() const = 0;
    [[nodiscard]] virtual bool isStealthed() const = 0;
    [[nodiscard]] virtual bool hasPet() const = 0;
    [[nodiscard]] virtual bool isInGroup() const = 0;
    [[nodiscard]] virtual bool isInRaid() const = 0;
    [[nodiscard]] virtual bool isIndoors() const = 0;

    // --- Numeric state ---
    [[nodiscard]] virtual uint8_t getActiveTalentSpec() const = 0;   // 0-based index
    [[nodiscard]] virtual uint32_t getVehicleId() const = 0;
    [[nodiscard]] virtual uint32_t getCurrentCastSpellId() const = 0;

    // --- Spell/aura queries ---
    [[nodiscard]] virtual std::string getSpellName(uint32_t spellId) const = 0;

    /** Check if target (or player if guid==playerGuid) has a buff/debuff by name. */
    [[nodiscard]] virtual bool hasAuraByName(uint64_t targetGuid, const std::string& spellName,
                               bool wantDebuff) const = 0;

    /** Check if player has a form/stance aura (permanent aura, maxDurationMs == -1). */
    [[nodiscard]] virtual bool hasFormAura() const = 0;

    // --- Entity queries (flattened, no Entity* exposure) ---
    [[nodiscard]] virtual bool entityExists(uint64_t guid) const = 0;
    [[nodiscard]] virtual bool entityIsDead(uint64_t guid) const = 0;
    [[nodiscard]] virtual bool entityIsHostile(uint64_t guid) const = 0;
};

} // namespace ui
} // namespace wowee
