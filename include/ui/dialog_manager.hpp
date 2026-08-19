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
    void renderDialogs(game::GameHandler& gameHandler,
                       InventoryScreen& inventoryScreen,
                       ChatPanel& chatPanel);

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
    uint8_t lfgRoles_ = 0x08;  // default: DPS (0x02=tank, 0x04=healer, 0x08=dps)

    // ---- Individual dialog renderers ----
    void renderGroupInvitePopup(game::GameHandler& gameHandler);
    void renderDuelRequestPopup(game::GameHandler& gameHandler);
    void renderDuelCountdown(game::GameHandler& gameHandler);
    void renderItemTextWindow(game::GameHandler& gameHandler);
    void renderSharedQuestPopup(game::GameHandler& gameHandler);
    void renderSummonRequestPopup(game::GameHandler& gameHandler);
    void renderTradeRequestPopup(game::GameHandler& gameHandler);
    void renderTradeWindow(game::GameHandler& gameHandler,
                           InventoryScreen& inventoryScreen,
                           ChatPanel& chatPanel);
    void renderGuildInvitePopup(game::GameHandler& gameHandler);
    void renderReadyCheckPopup(game::GameHandler& gameHandler);
    void renderBgInvitePopup(game::GameHandler& gameHandler);
    void renderBfMgrInvitePopup(game::GameHandler& gameHandler);
    void renderLfgProposalPopup(game::GameHandler& gameHandler);
    void renderLfgRoleCheckPopup(game::GameHandler& gameHandler);
    void renderResurrectDialog(game::GameHandler& gameHandler);
    void renderTalentWipeConfirmDialog(game::GameHandler& gameHandler);
    void renderPetUnlearnConfirmDialog(game::GameHandler& gameHandler);
};

} // namespace ui
} // namespace wowee
