#pragma once

#include "ui/ui_services.hpp"
#include <imgui.h>
#include <string>
#include <cstdint>

namespace wowee {
namespace game { class GameHandler; }
namespace ui {

class ChatPanel;
class InventoryScreen;

/**
 * Dialog / popup overlay manager
 *
 * Owns all yes/no popup rendering:
 *   group invite, duel request, duel countdown, loot roll, trade request,
 *   trade window, summon request, shared quest, item text, guild invite,
 *   ready check, BG invite, BF manager invite, LFG proposal, LFG role check,
 *   resurrect, talent wipe confirm, pet unlearn confirm.
 */
class DialogManager {
public:
    DialogManager() = default;

    /// Render "early" dialogs (group invite through LFG role check)
    /// called in render() before guild roster / social frame
    void renderDialogs(game::GameHandler& gameHandler);

    /// Render "late" dialogs (resurrect, talent wipe, pet unlearn)
    /// called in render() after reclaim corpse button
    void renderLateDialogs(game::GameHandler& gameHandler);

    // UIServices injection (Phase B singleton breaking)
    void setServices(const UIServices& services) { services_ = services; }

private:
    // Injected UI services
    UIServices services_;
    // Common ImGui window flags for popup dialogs
    static constexpr ImGuiWindowFlags kDialogFlags =
        ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize;

    // ---- LFG role state ----

    // ---- Individual dialog renderers ----
    void renderDuelCountdown(game::GameHandler& gameHandler);
    void renderPetUnlearnConfirmDialog(game::GameHandler& gameHandler);
};

} // namespace ui
} // namespace wowee
