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
} // anonymous namespace

namespace wowee {
namespace ui {

void SocialPanel::openInspectWindow(game::GameHandler& gameHandler) {
    // Loads Blizzard_InspectUI and shows it. The inspect request itself is
    // already on its way from the caller; this only puts a window up.
    gameHandler.runInterfaceCommand("InspectUnit(\"target\")");
}

} // namespace ui
} // namespace wowee
