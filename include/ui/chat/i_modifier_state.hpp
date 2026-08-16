// IModifierState - abstract interface for keyboard modifier queries.
// Allows unit testing macro conditionals without real input system. Phase 4.1.
#pragma once

namespace wowee {
namespace ui {

/**
 * Read-only view of keyboard modifier state for macro conditional evaluation.
 */
class IModifierState {
public:
    virtual ~IModifierState() = default;

    [[nodiscard]] virtual bool isShiftHeld() const = 0;
    [[nodiscard]] virtual bool isCtrlHeld() const = 0;
    [[nodiscard]] virtual bool isAltHeld() const = 0;
};

} // namespace ui
} // namespace wowee
