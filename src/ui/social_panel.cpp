// ============================================================
// SocialPanel - extracted from GameScreen
// Owns all social/group-related UI rendering: party frames,
// boss frames, guild roster, social/friends frame, dungeon finder,
// who window, inspect window.
// ============================================================
#include "ui/framexml_takeover.hpp"
#include "ui/social_panel.hpp"
#include "ui/ui_raid_icons.hpp"
#include "ui/chat_panel.hpp"
#include "ui/spellbook_screen.hpp"
#include "ui/inventory_screen.hpp"
#include "ui/ui_colors.hpp"
#include "ui/ui_helpers.hpp"
#include "core/application.hpp"
#include "core/logger.hpp"
#include "rendering/renderer.hpp"
#include "game/game_handler.hpp"
#include "game/game_utils.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_layout.hpp"
#include "ui/keybinding_manager.hpp"
#include "game/zone_manager.hpp"
#include <imgui.h>
#include <imgui_internal.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <string>

namespace {
    using namespace wowee::ui::colors;
    using namespace wowee::ui::helpers;
    constexpr auto& kColorRed         = kRed;
    constexpr auto& kColorGray        = kGray;
} // anonymous namespace

namespace wowee {
namespace ui {


namespace {
// LFG role bitmask: 0x02=Tank, 0x04=Healer, 0x08=DPS (0x01=Leader, unused here).
constexpr uint8_t kLfgRoleTank   = 0x02;
constexpr uint8_t kLfgRoleHealer = 0x04;
constexpr uint8_t kLfgRoleDps    = 0x08;

// Which roles a class can actually fill in the Dungeon Finder (WotLK). Every
// class can DPS. An unknown class (0, not yet received) allows all roles so we
// never wrongly lock the player out before their character data arrives.
uint8_t allowedLfgRolesForClass(uint8_t classId) {
    switch (classId) {
        case 1:  return kLfgRoleTank | kLfgRoleDps;                    // Warrior
        case 2:  return kLfgRoleTank | kLfgRoleHealer | kLfgRoleDps;   // Paladin
        case 3:  return kLfgRoleDps;                                   // Hunter
        case 4:  return kLfgRoleDps;                                   // Rogue
        case 5:  return kLfgRoleHealer | kLfgRoleDps;                  // Priest
        case 6:  return kLfgRoleTank | kLfgRoleDps;                    // Death Knight
        case 7:  return kLfgRoleHealer | kLfgRoleDps;                  // Shaman
        case 8:  return kLfgRoleDps;                                   // Mage
        case 9:  return kLfgRoleDps;                                   // Warlock
        case 11: return kLfgRoleTank | kLfgRoleHealer | kLfgRoleDps;   // Druid
        default: return kLfgRoleTank | kLfgRoleHealer | kLfgRoleDps;   // unknown
    }
}
} // namespace

void SocialPanel::renderDungeonFinderWindow(game::GameHandler& gameHandler,
                                               ChatPanel& chatPanel) {
    // Toggle Dungeon Finder (customizable keybind)
    if (!chatPanel.isChatInputActive() && !ImGui::GetIO().WantTextInput &&
        KeybindingManager::getInstance().isActionPressed(KeybindingManager::Action::TOGGLE_DUNGEON_FINDER)) {
        showDungeonFinder_ = !showDungeonFinder_;
    }

    if (!showDungeonFinder_) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth())  : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) :  720.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW * 0.5f - 175.0f, screenH * 0.2f),
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(350, 0), ImGuiCond_Always);

    bool open = true;
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize;
    if (!ImGui::Begin("Dungeon Finder", &open, flags)) {
        ImGui::End();
        if (!open) showDungeonFinder_ = false;
        return;
    }
    if (!open) {
        ImGui::End();
        showDungeonFinder_ = false;
        return;
    }

    using LfgState = game::GameHandler::LfgState;
    LfgState state = gameHandler.getLfgState();

    // ---- Status banner ----
    switch (state) {
        case LfgState::None:
            ImGui::TextColored(kColorGray, "Status: Not queued");
            break;
        case LfgState::RoleCheck:
            ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "Status: Role check in progress...");
            break;
        case LfgState::Queued: {
            int32_t avgSec  = gameHandler.getLfgAvgWaitSec();
            uint32_t qMs    = gameHandler.getLfgTimeInQueueMs();
            int      qMin   = static_cast<int>(qMs / 60000);
            int      qSec   = static_cast<int>((qMs % 60000) / 1000);
            std::string dName = gameHandler.getCurrentLfgDungeonName();
            if (!dName.empty())
                ImGui::TextColored(colors::kQueueGreen,
                                   "Status: In queue for %s (%d:%02d)", dName.c_str(), qMin, qSec);
            else
                ImGui::TextColored(colors::kQueueGreen, "Status: In queue (%d:%02d)", qMin, qSec);
            if (avgSec >= 0) {
                int aMin = avgSec / 60;
                int aSec = avgSec % 60;
                ImGui::TextColored(colors::kSilver,
                                   "Avg wait: %d:%02d", aMin, aSec);
            }
            break;
        }
        case LfgState::Proposal: {
            std::string dName = gameHandler.getCurrentLfgDungeonName();
            if (!dName.empty())
                ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.1f, 1.0f), "Status: Group found for %s!", dName.c_str());
            else
                ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.1f, 1.0f), "Status: Group found!");
            break;
        }
        case LfgState::Boot:
            ImGui::TextColored(kColorRed, "Status: Vote kick in progress");
            break;
        case LfgState::InDungeon: {
            std::string dName = gameHandler.getCurrentLfgDungeonName();
            if (!dName.empty())
                ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Status: In dungeon (%s)", dName.c_str());
            else
                ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Status: In dungeon");
            break;
        }
        case LfgState::FinishedDungeon: {
            std::string dName = gameHandler.getCurrentLfgDungeonName();
            if (!dName.empty())
                ImGui::TextColored(colors::kLightGreen, "Status: %s complete", dName.c_str());
            else
                ImGui::TextColored(colors::kLightGreen, "Status: Dungeon complete");
            break;
        }
        case LfgState::RaidBrowser:
            ImGui::TextColored(ImVec4(0.8f, 0.6f, 1.0f, 1.0f), "Status: Raid browser");
            break;
    }

    ImGui::Separator();

    // ---- Proposal accept/decline ----
    if (state == LfgState::Proposal) {
        std::string dName = gameHandler.getCurrentLfgDungeonName();
        if (!dName.empty())
            ImGui::TextColored(ImVec4(1.0f, 0.9f, 0.3f, 1.0f),
                               "A group has been found for %s!", dName.c_str());
        else
            ImGui::TextColored(ImVec4(1.0f, 0.9f, 0.3f, 1.0f),
                               "A group has been found for your dungeon!");
        ImGui::Spacing();
        if (ImGui::Button("Accept", ImVec2(120, 0))) {
            gameHandler.lfgAcceptProposal(gameHandler.getLfgProposalId(), true);
        }
        ImGui::SameLine();
        if (ImGui::Button("Decline", ImVec2(120, 0))) {
            gameHandler.lfgAcceptProposal(gameHandler.getLfgProposalId(), false);
        }
        ImGui::Separator();
    }

    // ---- Vote-to-kick buttons ----
    if (state == LfgState::Boot) {
        ImGui::TextColored(kColorRed, "Vote to kick in progress:");
        const std::string& bootTarget = gameHandler.getLfgBootTargetName();
        const std::string& bootReason = gameHandler.getLfgBootReason();
        if (!bootTarget.empty()) {
            ImGui::Text("Player: ");
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f), "%s", bootTarget.c_str());
        }
        if (!bootReason.empty()) {
            ImGui::Text("Reason: ");
            ImGui::SameLine();
            ImGui::TextWrapped("%s", bootReason.c_str());
        }
        uint32_t bootVotes   = gameHandler.getLfgBootVotes();
        uint32_t bootTotal   = gameHandler.getLfgBootTotal();
        uint32_t bootNeeded  = gameHandler.getLfgBootNeeded();
        uint32_t bootTimeLeft= gameHandler.getLfgBootTimeLeft();
        if (bootNeeded > 0) {
            ImGui::Text("Votes: %u / %u  (need %u)   %us left",
                        bootVotes, bootTotal, bootNeeded, bootTimeLeft);
        }
        ImGui::Spacing();
        if (ImGui::Button("Vote Yes (kick)", ImVec2(140, 0))) {
            gameHandler.lfgSetBootVote(true);
        }
        ImGui::SameLine();
        if (ImGui::Button("Vote No (keep)", ImVec2(140, 0))) {
            gameHandler.lfgSetBootVote(false);
        }
        ImGui::Separator();
    }

    // ---- Teleport button (in dungeon) ----
    if (state == LfgState::InDungeon) {
        if (ImGui::Button("Teleport to Dungeon", ImVec2(-1, 0))) {
            gameHandler.lfgTeleport(true);
        }
        ImGui::Separator();
    }

    // ---- Role selection (only when not queued/in dungeon) ----
    bool canConfigure = (state == LfgState::None || state == LfgState::FinishedDungeon);

    if (canConfigure) {
        // Only offer roles the player's class can perform; drop any stale bits
        // (e.g. a role picked before the class was known, or a class change).
        const uint8_t allowedRoles = allowedLfgRolesForClass(gameHandler.getPlayerClass());
        lfgRoles_ &= allowedRoles;

        ImGui::Text("Role:");
        ImGui::SameLine();
        auto roleCheckbox = [&](const char* label, uint8_t bit) {
            const bool available = (allowedRoles & bit) != 0;
            bool checked = (lfgRoles_ & bit) != 0;
            if (!available) ImGui::BeginDisabled();
            if (ImGui::Checkbox(label, &checked))
                lfgRoles_ = (lfgRoles_ & ~bit) | (checked ? bit : 0);
            if (!available) {
                ImGui::EndDisabled();
                if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
                    ImGui::SetTooltip("Your class cannot fill this role.");
            }
        };
        roleCheckbox("Tank", kLfgRoleTank);
        ImGui::SameLine();
        roleCheckbox("Healer", kLfgRoleHealer);
        ImGui::SameLine();
        roleCheckbox("DPS", kLfgRoleDps);

        ImGui::Spacing();

        // ---- Dungeon selection ----
        ImGui::Text("Dungeon:");

        // Category 0=Random, 1=Classic, 2=TBC, 3=WotLK. minLvl is the lowest
        // character level allowed to queue; kept conservative (erring low) so a
        // valid queue is never blocked - the server remains the authority.
        struct DungeonEntryEx { uint32_t id; const char* name; uint8_t cat; uint8_t minLvl; };
        static const DungeonEntryEx kDungeons[] = {
            { .id = 861, .name = "Random Dungeon",               .cat = 0, .minLvl = 15 },
            { .id = 862, .name = "Random Heroic",                .cat = 0, .minLvl = 80 },
            {  .id = 36, .name = "Deadmines",                    .cat = 1, .minLvl = 15 },
            {  .id = 43, .name = "Ragefire Chasm",               .cat = 1, .minLvl = 15 },
            {  .id = 47, .name = "Razorfen Kraul",               .cat = 1, .minLvl = 24 },
            {  .id = 48, .name = "Blackfathom Deeps",            .cat = 1, .minLvl = 21 },
            {  .id = 52, .name = "Uldaman",                      .cat = 1, .minLvl = 34 },
            {  .id = 57, .name = "Dire Maul: East",              .cat = 1, .minLvl = 46 },
            {  .id = 70, .name = "Onyxia's Lair",                .cat = 1, .minLvl = 80 },
            { .id = 264, .name = "The Blood Furnace",            .cat = 2, .minLvl = 58 },
            { .id = 269, .name = "The Shattered Halls",          .cat = 2, .minLvl = 67 },
            { .id = 576, .name = "The Nexus",                    .cat = 3, .minLvl = 68 },
            { .id = 578, .name = "The Oculus",                   .cat = 3, .minLvl = 76 },
            { .id = 595, .name = "The Culling of Stratholme",    .cat = 3, .minLvl = 77 },
            { .id = 599, .name = "Halls of Stone",               .cat = 3, .minLvl = 74 },
            { .id = 600, .name = "Drak'Tharon Keep",             .cat = 3, .minLvl = 71 },
            { .id = 601, .name = "Azjol-Nerub",                  .cat = 3, .minLvl = 70 },
            { .id = 604, .name = "Gundrak",                      .cat = 3, .minLvl = 74 },
            { .id = 608, .name = "Violet Hold",                  .cat = 3, .minLvl = 72 },
            { .id = 619, .name = "Ahn'kahet: Old Kingdom",       .cat = 3, .minLvl = 71 },
            { .id = 623, .name = "Halls of Lightning",           .cat = 3, .minLvl = 77 },
            { .id = 632, .name = "The Forge of Souls",           .cat = 3, .minLvl = 79 },
            { .id = 650, .name = "Trial of the Champion",        .cat = 3, .minLvl = 79 },
            { .id = 658, .name = "Pit of Saron",                 .cat = 3, .minLvl = 79 },
            { .id = 668, .name = "Halls of Reflection",          .cat = 3, .minLvl = 79 },
        };
        static constexpr const char* kCatHeaders[] = { nullptr, "-- Classic --", "-- TBC --", "-- WotLK --" };
        constexpr int kDungeonCount = static_cast<int>(sizeof(kDungeons)/sizeof(kDungeons[0]));
        const uint32_t playerLvl = gameHandler.getPlayerLevel();

        // Format a dungeon's label with its minimum level, e.g. "Gundrak  (Lv 74+)".
        auto dungeonLabel = [](const DungeonEntryEx& d, char* buf, size_t n) {
            snprintf(buf, n, "%s  (Lv %u+)", d.name, static_cast<unsigned>(d.minLvl));
        };

        // Find current index
        int curIdx = 0;
        for (int i = 0; i < kDungeonCount; ++i) {
            if (kDungeons[i].id == lfgSelectedDungeon_) { curIdx = i; break; }
        }

        char curLabel[96];
        dungeonLabel(kDungeons[curIdx], curLabel, sizeof(curLabel));
        ImGui::SetNextItemWidth(-1);
        if (ImGui::BeginCombo("##dungeon", curLabel)) {
            uint8_t lastCat = 255;
            for (const auto& kDungeon : kDungeons) {
                if (kDungeon.cat != lastCat && kCatHeaders[kDungeon.cat]) {
                    if (lastCat != 255) ImGui::Separator();
                    ImGui::TextDisabled("%s", kCatHeaders[kDungeon.cat]);
                    lastCat = kDungeon.cat;
                } else if (kDungeon.cat != lastCat) {
                    lastCat = kDungeon.cat;
                }
                // Grey out dungeons the player is too low to queue for.
                const bool tooLow = (playerLvl != 0 && playerLvl < kDungeon.minLvl);
                char label[96];
                dungeonLabel(kDungeon, label, sizeof(label));
                bool selected = (kDungeon.id == lfgSelectedDungeon_);
                if (tooLow) {
                    ImGui::TextDisabled("%s", label);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Requires level %u.", static_cast<unsigned>(kDungeon.minLvl));
                } else {
                    if (ImGui::Selectable(label, selected))
                        lfgSelectedDungeon_ = kDungeon.id;
                    if (selected) ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }

        ImGui::Spacing();

        // ---- Join button ----
        const bool rolesOk = (lfgRoles_ != 0);
        const bool levelOk = (playerLvl == 0 || playerLvl >= kDungeons[curIdx].minLvl);
        const bool canJoin = rolesOk && levelOk;
        if (!canJoin) {
            ImGui::BeginDisabled();
        }
        if (ImGui::Button("Join Dungeon Finder", ImVec2(-1, 0))) {
            gameHandler.lfgJoin({lfgSelectedDungeon_}, lfgRoles_);
        }
        if (!canJoin) {
            ImGui::EndDisabled();
            if (!rolesOk)
                ImGui::TextColored(colors::kSoftRed, "Select at least one role.");
            else if (!levelOk)
                ImGui::TextColored(colors::kSoftRed, "Requires level %u for this dungeon.",
                                   static_cast<unsigned>(kDungeons[curIdx].minLvl));
        }
    }

    // ---- Leave button (when queued or role check) ----
    if (state == LfgState::Queued || state == LfgState::RoleCheck) {
        if (ImGui::Button("Leave Queue", ImVec2(-1, 0))) {
            gameHandler.lfgLeave();
        }
    }

    ImGui::End();
}

void SocialPanel::openInspectWindow(game::GameHandler& gameHandler) {
    // Loads Blizzard_InspectUI and shows it. The inspect request itself is
    // already on its way from the caller; this only puts a window up.
    gameHandler.runInterfaceCommand("InspectUnit(\"target\")");
}

} // namespace ui
} // namespace wowee
