#include "ui/game_screen.hpp"
#include "ui/ui_raid_icons.hpp"
#include "ui/ui_colors.hpp"
#include "ui/ui_helpers.hpp"
#include "rendering/vk_context.hpp"
#include "core/application.hpp"
#include "core/appearance_composer.hpp"
#include "addons/addon_manager.hpp"
#include "core/coordinates.hpp"
#include "game/pet_action.hpp"
#include "core/input.hpp"
#include "rendering/renderer.hpp"
#include "rendering/post_process_pipeline.hpp"
#include "rendering/animation_controller.hpp"
#include "rendering/wmo_renderer.hpp"
#include "rendering/terrain_manager.hpp"
#include "rendering/minimap.hpp"
#include "rendering/world_map.hpp"
#include "rendering/character_renderer.hpp"
#include "rendering/camera.hpp"
#include "rendering/camera_controller.hpp"
#include "audio/audio_coordinator.hpp"
#include "audio/audio_engine.hpp"
#include "audio/music_manager.hpp"
#include "game/zone_manager.hpp"
#include "audio/footstep_manager.hpp"
#include "audio/activity_sound_manager.hpp"
#include "audio/mount_sound_manager.hpp"
#include "audio/npc_voice_manager.hpp"
#include "audio/ambient_sound_manager.hpp"
#include "audio/ui_sound_manager.hpp"
#include "audio/combat_sound_manager.hpp"
#include "audio/spell_sound_manager.hpp"
#include "audio/movement_sound_manager.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_loader.hpp"
#include "pipeline/dbc_layout.hpp"

#include "game/expansion_profile.hpp"
#include "game/character.hpp"
#include "core/logger.hpp"
#include <imgui.h>
#include <imgui_internal.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <sstream>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <cctype>
#include <chrono>
#include <ctime>

#include <unordered_set>

namespace {
    using namespace wowee::ui::colors;
    using namespace wowee::ui::helpers;


}

namespace wowee { namespace ui {

// ============================================================
// Totem Frame (Shaman - below pet frame / player frame)
// ============================================================

void GameScreen::renderTotemFrame(game::GameHandler& gameHandler) {
    // Only show if at least one totem is active
    bool anyActive = false;
    for (int i = 0; i < game::GameHandler::NUM_TOTEM_SLOTS; ++i) {
        if (gameHandler.getTotemSlot(i).active()) { anyActive = true; break; }
    }
    if (!anyActive) return;

    static constexpr struct { const char* name; ImU32 color; } kTotemInfo[4] = {
        { .name = "Earth", .color = IM_COL32(139, 90,  43, 255) },   // brown
        { .name = "Fire",  .color = IM_COL32(220, 80,  30, 255) },   // red-orange
        { .name = "Water", .color = IM_COL32( 30,120, 220, 255) },   // blue
        { .name = "Air",   .color = IM_COL32(180,220, 255, 255) },   // light blue
    };

    // Position: below pet frame / player frame, left side
    // Pet frame is at ~y=200 if active, player frame is at y=20; totem frame near y=300
    // We anchor relative to screen left edge like pet frame
    ImGui::SetNextWindowPos(ImVec2(8.0f, 300.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(130.0f, 0.0f), ImGuiCond_Always);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
                             ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_AlwaysAutoResize |
                             ImGuiWindowFlags_NoTitleBar;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.08f, 0.06f, 0.88f));

    if (ImGui::Begin("##TotemFrame", nullptr, flags)) {
        ImGui::TextColored(ImVec4(0.9f, 0.75f, 0.3f, 1.0f), "Totems");
        ImGui::Separator();

        for (int i = 0; i < game::GameHandler::NUM_TOTEM_SLOTS; ++i) {
            const auto& slot = gameHandler.getTotemSlot(i);
            if (!slot.active()) continue;

            ImGui::PushID(i);

            // Colored element dot
            ImVec2 dotPos = ImGui::GetCursorScreenPos();
            dotPos.x += 4.0f; dotPos.y += 6.0f;
            ImGui::GetWindowDrawList()->AddCircleFilled(
                ImVec2(dotPos.x + 4.0f, dotPos.y + 4.0f), 4.0f, kTotemInfo[i].color);
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 14.0f);

            // Totem name or spell name
            const std::string& spellName = gameHandler.getSpellName(slot.spellId);
            const char* displayName = spellName.empty() ? kTotemInfo[i].name : spellName.c_str();
            ImGui::Text("%s", displayName);

            // Duration countdown bar
            float remMs  = slot.remainingMs();
            float totMs  = static_cast<float>(slot.durationMs);
            float frac   = (totMs > 0.0f) ? std::min(remMs / totMs, 1.0f) : 0.0f;
            float remSec = remMs / 1000.0f;

            // Color bar with totem element tint
            ImVec4 barCol(
                static_cast<float>((kTotemInfo[i].color >> IM_COL32_R_SHIFT) & 0xFF) / 255.0f,
                static_cast<float>((kTotemInfo[i].color >> IM_COL32_G_SHIFT) & 0xFF) / 255.0f,
                static_cast<float>((kTotemInfo[i].color >> IM_COL32_B_SHIFT) & 0xFF) / 255.0f,
                0.9f);
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, barCol);
            char timeBuf[16];
            snprintf(timeBuf, sizeof(timeBuf), "%.0fs", remSec);
            ImGui::ProgressBar(frac, ImVec2(-1, 8), timeBuf);
            ImGui::PopStyleColor();

            ImGui::PopID();
        }
    }
    ImGui::End();

    ImGui::PopStyleColor();
    ImGui::PopStyleVar();
}


}} // namespace wowee::ui
