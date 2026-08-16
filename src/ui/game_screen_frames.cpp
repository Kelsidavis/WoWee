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
    constexpr auto& kColorRed        = kRed;
    constexpr auto& kColorGreen      = kGreen;
    constexpr auto& kColorBrightGreen= kBrightGreen;
    constexpr auto& kColorGray       = kGray;
    constexpr auto& kColorDarkGray   = kDarkGray;



    // Name a built-in pet bar slot. Both halves of the packed value matter:
    // the ids 0/1/2 mean stay/follow/attack as commands and passive/defensive/
    // aggressive as stances, so reading the id alone labelled half the bar
    // wrongly and left the id-0 slots blank.
    const char* petBuiltinName(uint32_t slotVal) {
        namespace pet = wowee::game::pet;
        const uint32_t id = pet::petActionId(slotVal);
        switch (pet::petActionType(slotVal)) {
            case pet::ActionType::Command:
                if (id == pet::kStay)    return "Stay";
                if (id == pet::kFollow)  return "Follow";
                if (id == pet::kAttack)  return "Attack";
                if (id == pet::kAbandon) return "Dismiss";
                if (id == pet::kMoveTo)  return "Move To";
                return nullptr;
            case pet::ActionType::Reaction:
                if (id == pet::kPassive)    return "Passive";
                if (id == pet::kDefensive)  return "Defensive";
                if (id == pet::kAggressive) return "Aggressive";
                return nullptr;
            default:
                return nullptr;
        }
    }

    const char* petBuiltinShortLabel(uint32_t slotVal) {
        const char* full = petBuiltinName(slotVal);
        if (!full) return nullptr;
        namespace pet = wowee::game::pet;
        const uint32_t id = pet::petActionId(slotVal);
        if (pet::petActionType(slotVal) == pet::ActionType::Command) {
            if (id == pet::kStay)    return "Sty";
            if (id == pet::kFollow)  return "Fol";
            if (id == pet::kAttack)  return "Atk";
            if (id == pet::kAbandon) return "Dis";
            return "Mov";
        }
        if (id == pet::kPassive)   return "Psv";
        if (id == pet::kDefensive) return "Def";
        return "Agg";
    }


}

namespace wowee { namespace ui {

void GameScreen::renderPlayerFrame(game::GameHandler& gameHandler) {
    bool isDead = gameHandler.isPlayerDead();
    ImGui::SetNextWindowPos(ImVec2(10.0f, 30.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(250.0f, 0.0f), ImGuiCond_Always);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                             ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_AlwaysAutoResize;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.1f, 0.1f, 0.85f));
    const bool inCombatConfirmed = gameHandler.isInCombat();
    const bool attackIntentOnly = gameHandler.hasAutoAttackIntent() && !inCombatConfirmed;
    ImVec4 playerBorder = isDead
        ? kColorDarkGray
        : (inCombatConfirmed
            ? colors::kBrightRed
            : (attackIntentOnly
                ? ImVec4(1.0f, 0.7f, 0.2f, 1.0f)
                : ImVec4(0.4f, 0.4f, 0.4f, 1.0f)));
    ImGui::PushStyleColor(ImGuiCol_Border, playerBorder);

    if (ImGui::Begin("##PlayerFrame", nullptr, flags)) {
        // Use selected character info if available, otherwise defaults
        std::string playerName = "Adventurer";
        uint32_t playerLevel = 1;
        uint32_t playerHp = 100;
        uint32_t playerMaxHp = 100;

        const auto& characters = gameHandler.getCharacters();
        uint64_t activeGuid = gameHandler.getActiveCharacterGuid();
        const game::Character* activeChar = nullptr;
        for (const auto& c : characters) {
            if (c.guid == activeGuid) { activeChar = &c; break; }
        }
        if (!activeChar && !characters.empty()) activeChar = &characters[0];
        if (activeChar) {
            const auto& ch = *activeChar;
            playerName = ch.name;
            // Use live server level if available, otherwise character struct
            playerLevel = gameHandler.getPlayerLevel();
            if (playerLevel == 0) playerLevel = ch.level;
            playerMaxHp = 20 + playerLevel * 10;
            playerHp = playerMaxHp;
        }

        // Derive class color via shared helper
        ImVec4 classColor = activeChar
            ? classColorVec4(static_cast<uint8_t>(activeChar->characterClass))
            : kColorBrightGreen;

        // Name in class color - clickable for self-target, right-click for menu
        ImGui::PushStyleColor(ImGuiCol_Text, classColor);
        if (ImGui::Selectable(playerName.c_str(), false, 0, ImVec2(0, 0))) {
            gameHandler.setTarget(gameHandler.getPlayerGuid());
        }
        if (ImGui::BeginPopupContextItem("PlayerSelfCtx")) {
            ImGui::TextDisabled("%s", playerName.c_str());
            ImGui::Separator();
            if (ImGui::MenuItem("Open Character")) {
                inventoryScreen.setCharacterOpen(true);
            }
            if (ImGui::MenuItem("Toggle PvP")) {
                gameHandler.togglePvp();
            }
            ImGui::Separator();
            bool afk = gameHandler.isAfk();
            bool dnd = gameHandler.isDnd();
            if (ImGui::MenuItem(afk ? "Cancel AFK" : "Set AFK")) {
                gameHandler.toggleAfk();
            }
            if (ImGui::MenuItem(dnd ? "Cancel DND" : "Set DND")) {
                gameHandler.toggleDnd();
            }
            if (gameHandler.isInGroup()) {
                ImGui::Separator();
                if (ImGui::MenuItem("Leave Group")) {
                    gameHandler.leaveGroup();
                }
            }
            ImGui::EndPopup();
        }
        ImGui::PopStyleColor();
        ImGui::SameLine();
        ImGui::TextDisabled("Lv %u", playerLevel);
        if (isDead) {
            ImGui::SameLine();
            ImGui::TextColored(colors::kDarkRed, "DEAD");
        }
        // Group leader crown on self frame when you lead the party/raid
        if (gameHandler.isInGroup() &&
            gameHandler.getPartyData().leaderGuid == gameHandler.getPlayerGuid()) {
            ImGui::SameLine(0, 4);
            ImGui::TextColored(colors::kSymbolGold, "\xe2\x99\x9b");
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("You are the group leader");
        }
        if (gameHandler.isAfk()) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.3f, 1.0f), "<AFK>");
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Away from keyboard - /afk to cancel");
        } else if (gameHandler.isDnd()) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.9f, 0.5f, 0.2f, 1.0f), "<DND>");
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Do not disturb - /dnd to cancel");
        }
        if (auto* ren = services_.renderer) {
            if (auto* cam = ren->getCameraController()) {
                if (cam->isAutoRunning()) {
                    ImGui::SameLine();
                    ImGui::TextColored(ImVec4(0.4f, 0.9f, 1.0f, 1.0f), "[Auto-Run]");
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Auto-running - press ` or NumLock to stop");
                }
            }
        }
        if (inCombatConfirmed && !isDead) {
            float combatPulse = 0.75f + 0.25f * std::sin(static_cast<float>(ImGui::GetTime()) * 4.0f);
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 0.2f * combatPulse, 0.2f * combatPulse, 1.0f), "[Combat]");
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("You are in combat");
        }

        // Active title - shown in gold below the name/level line
        {
            int32_t titleBit = gameHandler.getChosenTitleBit();
            if (titleBit >= 0) {
                const std::string titleText = gameHandler.getFormattedTitle(
                    static_cast<uint32_t>(titleBit));
                if (!titleText.empty()) {
                    ImGui::TextColored(ImVec4(1.0f, 0.84f, 0.0f, 0.9f), "%s", titleText.c_str());
                }
            }
        }

        // Try to get real HP/mana from the player entity
        auto playerEntity = gameHandler.getEntityManager().getEntity(gameHandler.getPlayerGuid());
        if (playerEntity && (playerEntity->getType() == game::ObjectType::PLAYER || playerEntity->getType() == game::ObjectType::UNIT)) {
            auto unit = std::static_pointer_cast<game::Unit>(playerEntity);
            if (unit->getMaxHealth() > 0) {
                playerHp = unit->getHealth();
                playerMaxHp = unit->getMaxHealth();
            }
        }

        // Health bar - color transitions green→yellow→red as HP drops
        float pct = static_cast<float>(playerHp) / static_cast<float>(playerMaxHp);
        ImVec4 hpColor;
        if (isDead) {
            hpColor = kColorDarkGray;
        } else if (pct > 0.5f) {
            hpColor = colors::kHealthGreen;              // green
        } else if (pct > 0.2f) {
            float t = (pct - 0.2f) / 0.3f;  // 0 at 20%, 1 at 50%
            hpColor = ImVec4(0.9f - 0.7f * t, 0.4f + 0.4f * t, 0.0f, 1.0f); // orange→yellow
        } else {
            // Critical - pulse red when < 20%
            float pulse = 0.7f + 0.3f * std::sin(static_cast<float>(ImGui::GetTime()) * 3.5f);
            hpColor = ImVec4(0.9f * pulse, 0.05f, 0.05f, 1.0f);    // pulsing red
        }
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, hpColor);
        char overlay[64];
        snprintf(overlay, sizeof(overlay), "%u / %u", playerHp, playerMaxHp);
        ImGui::ProgressBar(pct, ImVec2(-1, 18), overlay);
        ImGui::PopStyleColor();

        // Mana/Power bar
        if (playerEntity && (playerEntity->getType() == game::ObjectType::PLAYER || playerEntity->getType() == game::ObjectType::UNIT)) {
            auto unit = std::static_pointer_cast<game::Unit>(playerEntity);
            uint8_t powerType = unit->getPowerType();
            uint32_t power = unit->getPower();
            uint32_t maxPower = unit->getMaxPower();
            // Rage (1), Focus (2), Energy (3), and Runic Power (6) always cap at 100.
            // Show bar even if server hasn't sent UNIT_FIELD_MAXPOWER1 yet.
            if (maxPower == 0 && (powerType == 1 || powerType == 2 || powerType == 3 || powerType == 6)) maxPower = 100;
            if (maxPower > 0) {
                float mpPct = static_cast<float>(power) / static_cast<float>(maxPower);
                ImVec4 powerColor = colors::powerTypeColor(powerType);
                // The player's own frame pulses its mana bar below a fifth, which
                // no other frame does.
                if (powerType == 0 && mpPct < 0.2f) {
                    float pulse = 0.6f + 0.4f * std::sin(static_cast<float>(ImGui::GetTime()) * 3.0f);
                    powerColor = ImVec4(0.1f, 0.1f, 0.8f * pulse, 1.0f);
                }
                ImGui::PushStyleColor(ImGuiCol_PlotHistogram, powerColor);
                char mpOverlay[64];
                snprintf(mpOverlay, sizeof(mpOverlay), "%u / %u", power, maxPower);
                ImGui::ProgressBar(mpPct, ImVec2(-1, 18), mpOverlay);
                ImGui::PopStyleColor();
            }
        }

        // Death Knight rune bar (class 6) - 6 colored squares with fill fraction
        if (gameHandler.getPlayerClass() == 6) {
            const auto& runes = gameHandler.getPlayerRunes();
            float dt = ImGui::GetIO().DeltaTime;

            ImGui::Spacing();
            ImVec2 cursor = ImGui::GetCursorScreenPos();
            float totalW  = ImGui::GetContentRegionAvail().x;
            float spacing = 3.0f;
            float squareW = (totalW - spacing * 5.0f) / 6.0f;
            float squareH = 14.0f;
            ImDrawList* dl = ImGui::GetWindowDrawList();

            for (int i = 0; i < 6; i++) {
                // Client-side prediction: advance fill over ~10s cooldown
                runeClientFill_[i] = runes[i].ready ? 1.0f
                    : std::min(runeClientFill_[i] + dt / 10.0f, runes[i].readyFraction + 0.02f);
                runeClientFill_[i] = std::clamp(runeClientFill_[i], 0.0f, runes[i].ready ? 1.0f : 0.97f);

                float x0 = cursor.x + i * (squareW + spacing);
                float y0 = cursor.y;
                float x1 = x0 + squareW;
                float y1 = y0 + squareH;

                // Background (dark)
                dl->AddRectFilled(ImVec2(x0, y0), ImVec2(x1, y1),
                                  IM_COL32(30, 30, 30, 200), 2.0f);

                // Fill color by rune type
                ImVec4 fc;
                switch (runes[i].type) {
                    case game::GameHandler::RuneType::Blood:  fc = ImVec4(0.85f, 0.12f, 0.12f, 1.0f); break;
                    case game::GameHandler::RuneType::Unholy: fc = ImVec4(0.20f, 0.72f, 0.20f, 1.0f); break;
                    case game::GameHandler::RuneType::Frost:  fc = ImVec4(0.30f, 0.55f, 0.90f, 1.0f); break;
                    case game::GameHandler::RuneType::Death:  fc = ImVec4(0.55f, 0.20f, 0.70f, 1.0f); break;
                    default:                                  fc = ImVec4(0.6f,  0.6f,  0.6f,  1.0f); break;
                }
                float fillX = x0 + (x1 - x0) * runeClientFill_[i];
                dl->AddRectFilled(ImVec2(x0, y0), ImVec2(fillX, y1),
                                  ImGui::ColorConvertFloat4ToU32(fc), 2.0f);

                // Border
                ImU32 borderCol = runes[i].ready
                    ? IM_COL32(220, 220, 220, 180)
                    : IM_COL32(100, 100, 100, 160);
                dl->AddRect(ImVec2(x0, y0), ImVec2(x1, y1), borderCol, 2.0f);
            }
            ImGui::Dummy(ImVec2(totalW, squareH));
        }

        // Combo point display - Rogue (4) and Druid (11) in Cat Form
        {
            uint8_t cls = gameHandler.getPlayerClass();
            const bool isRogue  = (cls == 4);
            const bool isDruid  = (cls == 11);
            if (isRogue || isDruid) {
                uint8_t cp = gameHandler.getComboPoints();
                if (cp > 0 || isRogue) {  // always show for rogue; only when non-zero for druid
                    ImGui::Spacing();
                    ImVec2 cursor = ImGui::GetCursorScreenPos();
                    float totalW  = ImGui::GetContentRegionAvail().x;
                    constexpr int MAX_CP = 5;
                    constexpr float DOT_R = 7.0f;
                    constexpr float SPACING = 4.0f;
                    float totalDotsW = MAX_CP * (DOT_R * 2.0f) + (MAX_CP - 1) * SPACING;
                    float startX = cursor.x + (totalW - totalDotsW) * 0.5f;
                    float cy = cursor.y + DOT_R;
                    ImDrawList* dl = ImGui::GetWindowDrawList();
                    for (int i = 0; i < MAX_CP; ++i) {
                        float cx = startX + i * (DOT_R * 2.0f + SPACING) + DOT_R;
                        ImU32 col = (i < static_cast<int>(cp))
                            ? IM_COL32(255, 210, 0, 240)   // bright gold - active
                            : IM_COL32(60,  60, 60, 160);  // dark - empty
                        dl->AddCircleFilled(ImVec2(cx, cy), DOT_R, col);
                        dl->AddCircle(ImVec2(cx, cy), DOT_R, IM_COL32(160, 140, 0, 180), 0, 1.5f);
                    }
                    ImGui::Dummy(ImVec2(totalW, DOT_R * 2.0f));
                }
            }
        }

        // Shaman totem bar (class 7) - 4 slots: Earth, Fire, Water, Air
        if (gameHandler.getPlayerClass() == 7) {
            static constexpr ImVec4 kTotemColors[] = {
                ImVec4(0.80f, 0.55f, 0.25f, 1.0f), // Earth - brown
                ImVec4(1.00f, 0.35f, 0.10f, 1.0f), // Fire  - orange-red
                ImVec4(0.20f, 0.55f, 0.90f, 1.0f), // Water - blue
                ImVec4(0.70f, 0.90f, 1.00f, 1.0f), // Air   - pale sky
            };
            static constexpr const char* kTotemNames[] = { "Earth", "Fire", "Water", "Air" };

            ImGui::Spacing();
            ImVec2 cursor = ImGui::GetCursorScreenPos();
            float totalW  = ImGui::GetContentRegionAvail().x;
            float spacing = 3.0f;
            float slotW   = (totalW - spacing * 3.0f) / 4.0f;
            float slotH   = 14.0f;
            ImDrawList* tdl = ImGui::GetWindowDrawList();

            for (int i = 0; i < game::GameHandler::NUM_TOTEM_SLOTS; i++) {
                const auto& ts = gameHandler.getTotemSlot(i);
                float x0 = cursor.x + i * (slotW + spacing);
                float y0 = cursor.y;
                float x1 = x0 + slotW;
                float y1 = y0 + slotH;

                // Background
                tdl->AddRectFilled(ImVec2(x0, y0), ImVec2(x1, y1), IM_COL32(20, 20, 20, 200), 2.0f);

                if (ts.active()) {
                    float rem = ts.remainingMs();
                    float frac = rem / static_cast<float>(ts.durationMs);
                    float fillX = x0 + (x1 - x0) * frac;
                    tdl->AddRectFilled(ImVec2(x0, y0), ImVec2(fillX, y1),
                                      ImGui::ColorConvertFloat4ToU32(kTotemColors[i]), 2.0f);
                    // Remaining seconds label
                    char secBuf[16];
                    snprintf(secBuf, sizeof(secBuf), "%.0f", rem / 1000.0f);
                    ImVec2 tsz = ImGui::CalcTextSize(secBuf);
                    float lx = x0 + (slotW - tsz.x) * 0.5f;
                    float ly = y0 + (slotH - tsz.y) * 0.5f;
                    tdl->AddText(ImVec2(lx + 1, ly + 1), IM_COL32(0, 0, 0, 180), secBuf);
                    tdl->AddText(ImVec2(lx, ly), IM_COL32(255, 255, 255, 230), secBuf);
                } else {
                    // Inactive - show element letter
                    const char* letter = kTotemNames[i];
                    char single[2] = { letter[0], '\0' };
                    ImVec2 tsz = ImGui::CalcTextSize(single);
                    float lx = x0 + (slotW - tsz.x) * 0.5f;
                    float ly = y0 + (slotH - tsz.y) * 0.5f;
                    tdl->AddText(ImVec2(lx, ly), IM_COL32(80, 80, 80, 200), single);
                }

                // Border
                ImU32 borderCol = ts.active()
                    ? ImGui::ColorConvertFloat4ToU32(kTotemColors[i])
                    : IM_COL32(60, 60, 60, 160);
                tdl->AddRect(ImVec2(x0, y0), ImVec2(x1, y1), borderCol, 2.0f);

                // Tooltip on hover
                ImGui::SetCursorScreenPos(ImVec2(x0, y0));
                char totemBtnId[16]; snprintf(totemBtnId, sizeof(totemBtnId), "##totem%d", i);
                ImGui::InvisibleButton(totemBtnId, ImVec2(slotW, slotH));
                if (ImGui::IsItemHovered()) {
                    ImGui::BeginTooltip();
                    if (ts.active()) {
                        const std::string& spellNm = gameHandler.getSpellName(ts.spellId);
                        ImGui::TextColored(ImVec4(kTotemColors[i].x, kTotemColors[i].y,
                                                  kTotemColors[i].z, 1.0f),
                                           "%s Totem", kTotemNames[i]);
                        if (!spellNm.empty()) ImGui::Text("%s", spellNm.c_str());
                        ImGui::Text("%.1fs remaining", ts.remainingMs() / 1000.0f);
                    } else {
                        ImGui::TextDisabled("%s Totem (empty)", kTotemNames[i]);
                    }
                    ImGui::EndTooltip();
                }
            }
            ImGui::SetCursorScreenPos(ImVec2(cursor.x, cursor.y + slotH + 2.0f));
            ImGui::Dummy(ImVec2(totalW, 0.0f));
        }
    }

    // Melee swing timer - shown when player is auto-attacking
    if (gameHandler.isAutoAttacking()) {
        const uint64_t lastSwingMs = gameHandler.getLastMeleeSwingMs();
        if (lastSwingMs > 0) {
            // Determine weapon speed from the equipped main-hand weapon
            uint32_t weaponDelayMs = 2000;  // Default: 2.0s unarmed
            const auto& mainSlot = gameHandler.getInventory().getEquipSlot(game::EquipSlot::MAIN_HAND);
            if (!mainSlot.empty() && mainSlot.item.itemId != 0) {
                const auto* info = gameHandler.getItemInfo(mainSlot.item.itemId);
                if (info && info->delayMs > 0) {
                    weaponDelayMs = info->delayMs;
                }
            }

            // Compute elapsed since last swing
            uint64_t nowMs = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count());
            uint64_t elapsedMs = (nowMs >= lastSwingMs) ? (nowMs - lastSwingMs) : 0;

            // Clamp to weapon delay (cap at 1.0 so the bar fills but doesn't exceed)
            float pct = std::min(static_cast<float>(elapsedMs) / static_cast<float>(weaponDelayMs), 1.0f);

            // Light silver-orange color indicating auto-attack readiness
            ImVec4 swingColor = (pct >= 0.95f)
                ? ImVec4(1.0f, 0.75f, 0.15f, 1.0f)   // gold when ready to swing
                : ImVec4(0.65f, 0.55f, 0.40f, 1.0f);  // muted brown-orange while filling
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, swingColor);
            ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.15f, 0.12f, 0.08f, 0.8f));
            char swingLabel[24];
            float remainSec = std::max(0.0f, (weaponDelayMs - static_cast<float>(elapsedMs)) / 1000.0f);
            if (pct >= 0.98f)
                snprintf(swingLabel, sizeof(swingLabel), "Swing!");
            else
                snprintf(swingLabel, sizeof(swingLabel), "%.1fs", remainSec);
            ImGui::ProgressBar(pct, ImVec2(-1.0f, 8.0f), swingLabel);
            ImGui::PopStyleColor(2);
        }
    }

    ImGui::End();

    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar();
}

void GameScreen::renderPetFrame(game::GameHandler& gameHandler) {
    uint64_t petGuid = gameHandler.getPetGuid();
    if (petGuid == 0) return;

    auto petEntity = gameHandler.getEntityManager().getEntity(petGuid);
    if (!petEntity) return;
    auto* petUnit = petEntity->isUnit() ? static_cast<game::Unit*>(petEntity.get()) : nullptr;
    if (!petUnit) return;

    // Position below player frame. If in a group, push below party frames
    // (party frame at y=120, each member ~50px, up to 4 members → max ~320px + y=120 = ~440).
    // When not grouped, the player frame ends at ~110px so y=125 is fine.
    const int partyMemberCount = gameHandler.isInGroup()
        ? static_cast<int>(gameHandler.getPartyData().members.size()) : 0;
    float petY = (partyMemberCount > 0)
        ? 120.0f + partyMemberCount * 52.0f + 8.0f
        : 125.0f;
    ImGui::SetNextWindowPos(ImVec2(10.0f, petY), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(200.0f, 0.0f), ImGuiCond_Always);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                             ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_AlwaysAutoResize;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.08f, 0.1f, 0.08f, 0.85f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.2f, 0.6f, 0.2f, 1.0f));

    if (ImGui::Begin("##PetFrame", nullptr, flags)) {
        const std::string& petName = petUnit->getName();
        uint32_t petLevel = petUnit->getLevel();

        // Name + level on one row - clicking the pet name targets it
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.4f, 0.9f, 0.4f, 1.0f));
        char petLabel[96];
        snprintf(petLabel, sizeof(petLabel), "%s",
                 petName.empty() ? "Pet" : petName.c_str());
        if (ImGui::Selectable(petLabel, false, 0, ImVec2(0, 0))) {
            gameHandler.setTarget(petGuid);
        }
        // Right-click context menu on pet name
        if (ImGui::BeginPopupContextItem("PetNameCtx")) {
            ImGui::TextDisabled("%s", petLabel);
            ImGui::Separator();
            if (ImGui::MenuItem("Target Pet")) {
                gameHandler.setTarget(petGuid);
            }
            if (ImGui::MenuItem("Rename Pet")) {
                ImGui::CloseCurrentPopup();
                petRenameOpen_ = true;
                petRenameBuf_[0] = '\0';
            }
            if (ImGui::MenuItem("Dismiss Pet")) {
                gameHandler.dismissPet();
            }
            ImGui::EndPopup();
        }
        // Pet rename modal (opened via context menu)
        if (petRenameOpen_) {
            ImGui::OpenPopup("Rename Pet###PetRename");
            petRenameOpen_ = false;
        }
        if (ImGui::BeginPopupModal("Rename Pet###PetRename", nullptr,
                                   ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse)) {
            ImGui::Text("Enter new pet name (max 12 characters):");
            ImGui::SetNextItemWidth(180.0f);
            bool submitted = ImGui::InputText("##PetRenameInput", petRenameBuf_, sizeof(petRenameBuf_),
                                              ImGuiInputTextFlags_EnterReturnsTrue);
            ImGui::SameLine();
            if (ImGui::Button("OK") || submitted) {
                std::string newName(petRenameBuf_);
                if (!newName.empty() && newName.size() <= 12) {
                    gameHandler.renamePet(newName);
                }
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel")) {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }
        ImGui::PopStyleColor();
        if (petLevel > 0) {
            ImGui::SameLine();
            ImGui::TextDisabled("Lv %u", petLevel);
        }

        // Health bar
        uint32_t hp    = petUnit->getHealth();
        uint32_t maxHp = petUnit->getMaxHealth();
        if (maxHp > 0) {
            float pct = static_cast<float>(hp) / static_cast<float>(maxHp);
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, colors::healthBarColor(pct));
            char hpText[32];
            snprintf(hpText, sizeof(hpText), "%u/%u", hp, maxHp);
            ImGui::ProgressBar(pct, ImVec2(-1, 14), hpText);
            ImGui::PopStyleColor();
        }

        // Power/mana bar (hunters' pets use focus)
        uint8_t  powerType = petUnit->getPowerType();
        uint32_t power     = petUnit->getPower();
        uint32_t maxPower  = petUnit->getMaxPower();
        if (maxPower == 0 && (powerType == 1 || powerType == 2 || powerType == 3)) maxPower = 100;
        if (maxPower > 0) {
            float mpPct = static_cast<float>(power) / static_cast<float>(maxPower);
            ImVec4 powerColor;
            powerColor = colors::powerTypeColor(static_cast<uint8_t>(powerType));
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, powerColor);
            char mpText[32];
            snprintf(mpText, sizeof(mpText), "%u/%u", power, maxPower);
            ImGui::ProgressBar(mpPct, ImVec2(-1, 14), mpText);
            ImGui::PopStyleColor();
        }

        // Happiness bar - hunter pets store happiness as power type 4
        {
            uint32_t happiness = petUnit->getPowerByType(4);
            uint32_t maxHappiness = petUnit->getMaxPowerByType(4);
            if (maxHappiness > 0 && happiness > 0) {
                float hapPct = static_cast<float>(happiness) / static_cast<float>(maxHappiness);
                // Tier: < 33% = Unhappy (red), < 67% = Content (yellow), >= 67% = Happy (green)
                ImVec4 hapColor = hapPct >= 0.667f ? ImVec4(0.2f, 0.85f, 0.2f, 1.0f)
                                : hapPct >= 0.333f ? ImVec4(0.9f, 0.75f, 0.1f, 1.0f)
                                :                   ImVec4(0.85f, 0.2f, 0.2f, 1.0f);
                const char* hapLabel = hapPct >= 0.667f ? "Happy" : hapPct >= 0.333f ? "Content" : "Unhappy";
                ImGui::PushStyleColor(ImGuiCol_PlotHistogram, hapColor);
                ImGui::ProgressBar(hapPct, ImVec2(-1, 8), hapLabel);
                ImGui::PopStyleColor();
            }
        }

        // Pet cast bar
        if (auto* pcs = gameHandler.getUnitCastState(petGuid)) {
            float castPct = (pcs->timeTotal > 0.0f)
                ? (pcs->timeTotal - pcs->timeRemaining) / pcs->timeTotal : 0.0f;
            // Orange color to distinguish from health/power bars
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.85f, 0.55f, 0.1f, 1.0f));
            char petCastLabel[48];
            const std::string& spellNm = gameHandler.getSpellName(pcs->spellId);
            if (!spellNm.empty())
                snprintf(petCastLabel, sizeof(petCastLabel), "%s (%.1fs)", spellNm.c_str(), pcs->timeRemaining);
            else
                snprintf(petCastLabel, sizeof(petCastLabel), "Casting... (%.1fs)", pcs->timeRemaining);
            ImGui::ProgressBar(castPct, ImVec2(-1, 10), petCastLabel);
            ImGui::PopStyleColor();
        }

        // Stance row: Passive / Defensive / Aggressive - with Dismiss right-aligned
        {
            static constexpr const char* kReactLabels[]     = { "Psv", "Def", "Agg" };
            static constexpr const char* kReactTooltips[]   = { "Passive", "Defensive", "Aggressive" };
            static constexpr ImVec4 kReactColors[]    = {
                colors::kLightBlue,  // passive  - blue
                ImVec4(0.3f, 0.85f, 0.3f, 1.0f), // defensive - green
                colors::kHostileRed,// aggressive - red
            };
            static constexpr ImVec4 kReactDimColors[] = {
                ImVec4(0.15f, 0.2f, 0.4f, 0.8f),
                ImVec4(0.1f, 0.3f, 0.1f, 0.8f),
                ImVec4(0.4f, 0.1f, 0.1f, 0.8f),
            };
            uint8_t curReact = gameHandler.getPetReact(); // 0=passive,1=defensive,2=aggressive

            // A stance slot is identified by its type as well as its id - the
            // ids 0/1/2 also name stay/follow/attack under the command type, so
            // matching on the id alone found the wrong slot.
            static constexpr uint32_t kReactActionIds[] = {
                game::pet::kPassive, game::pet::kDefensive, game::pet::kAggressive
            };
            uint32_t reactSlotVals[3] = { 0, 0, 0 };
            const int slotTotal = game::GameHandler::PET_ACTION_BAR_SLOTS;
            for (int i = 0; i < slotTotal; ++i) {
                uint32_t sv = gameHandler.getPetActionSlot(i);
                if (game::pet::petActionType(sv) != game::pet::ActionType::Reaction) continue;
                uint32_t aid = game::pet::petActionId(sv);
                for (int r = 0; r < 3; ++r) {
                    if (aid == kReactActionIds[r]) { reactSlotVals[r] = sv; break; }
                }
            }

            for (int r = 0; r < 3; ++r) {
                if (r > 0) ImGui::SameLine(0.0f, 3.0f);
                bool active = (curReact == static_cast<uint8_t>(r));
                ImVec4 btnCol = active ? kReactColors[r] : kReactDimColors[r];
                ImGui::PushID(r + 1000);
                ImGui::PushStyleColor(ImGuiCol_Button,        btnCol);
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, kReactColors[r]);
                ImGui::PushStyleColor(ImGuiCol_ButtonActive,  kReactColors[r]);
                if (ImGui::Button(kReactLabels[r], ImVec2(34.0f, 16.0f))) {
                    // Use server-provided slot value if available; fall back to raw ID
                    uint32_t action = (reactSlotVals[r] != 0)
                        ? reactSlotVals[r]
                        : game::pet::packPetAction(game::pet::ActionType::Reaction,
                                                   kReactActionIds[r]);
                    gameHandler.sendPetAction(action, 0);
                }
                ImGui::PopStyleColor(3);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("%s", kReactTooltips[r]);
                ImGui::PopID();
            }

            // Dismiss button right-aligned on the same row
            ImGui::SameLine(ImGui::GetContentRegionAvail().x - 58.0f);
            if (ImGui::SmallButton("Dismiss")) {
                gameHandler.dismissPet();
            }
        }

        // Pet action bar - show up to 10 action slots from SMSG_PET_SPELLS
        {
            const int slotCount = game::GameHandler::PET_ACTION_BAR_SLOTS;
            // Filter to non-zero slots; lay them out as small icon/text buttons.
            // Raw slot value layout (WotLK 3.3.5): low 24 bits = spell/action ID,
            // high byte = type. The built-in commands and stances share the ids
            // 0/1/2, so what a slot means depends on both halves - see
            // game/pet_action.hpp.
            auto* assetMgr = services_.assetManager;
            const float iconSz = 20.0f;
            const float spacing = 2.0f;
            ImGui::Separator();

            int rendered = 0;
            for (int i = 0; i < slotCount; ++i) {
                uint32_t slotVal = gameHandler.getPetActionSlot(i);
                if (slotVal == 0) continue;

                uint32_t actionId = game::pet::petActionId(slotVal);
                const bool isSpell = game::pet::isPetSpellAction(slotVal);
                // Use the authoritative autocast set from SMSG_PET_SPELLS spell list flags.
                bool autocastOn   = isSpell && gameHandler.isPetSpellAutocast(actionId);

                // Cooldowns only apply to castable spells, not commands/stances.
                float petCd = isSpell ? gameHandler.getSpellCooldown(actionId) : 0.0f;
                bool petOnCd = (petCd > 0.0f);

                ImGui::PushID(i);
                if (rendered > 0) ImGui::SameLine(0.0f, spacing);

                // Try to show spell icon; fall back to abbreviated text label.
                VkDescriptorSet iconTex = VK_NULL_HANDLE;
                const char* builtinLabel = petBuiltinShortLabel(slotVal);
                if (!builtinLabel && assetMgr) iconTex = getSpellIcon(actionId, assetMgr);

                // Dim when on cooldown; tint green when autocast is on
                ImVec4 tint = petOnCd
                    ? ImVec4(0.35f, 0.35f, 0.35f, 0.7f)
                    : (autocastOn ? colors::kLightGreen : ui::colors::kWhite);
                bool clicked = false;
                if (iconTex) {
                    clicked = ImGui::ImageButton("##pa",
                        (ImTextureID)(uintptr_t)iconTex,
                        ImVec2(iconSz, iconSz),
                        ImVec2(0,0), ImVec2(1,1),
                        ImVec4(0.1f,0.1f,0.1f,0.9f), tint);
                } else {
                    char label[8];
                    if (builtinLabel) {
                        snprintf(label, sizeof(label), "%s", builtinLabel);
                    } else {
                        // Show first 3 chars of spell name or spell ID.
                        std::string nm = gameHandler.getSpellName(actionId);
                        if (nm.empty()) snprintf(label, sizeof(label), "?%u", actionId % 100);
                        else            snprintf(label, sizeof(label), "%.3s", nm.c_str());
                    }
                    ImVec4 btnCol = petOnCd ? ImVec4(0.1f,0.1f,0.15f,0.9f)
                                   : (autocastOn ? ImVec4(0.2f,0.5f,0.2f,0.9f)
                                                 : ImVec4(0.2f,0.2f,0.3f,0.9f));
                    ImGui::PushStyleColor(ImGuiCol_Button, btnCol);
                    clicked = ImGui::Button(label, ImVec2(iconSz + 4.0f, iconSz));
                    ImGui::PopStyleColor();
                }

                // Cooldown overlay: dark fill + time text centered on the button
                if (petOnCd && !builtinLabel) {
                    ImVec2 bMin = ImGui::GetItemRectMin();
                    ImVec2 bMax = ImGui::GetItemRectMax();
                    auto* cdDL = ImGui::GetWindowDrawList();
                    cdDL->AddRectFilled(bMin, bMax, IM_COL32(0, 0, 0, 140));
                    char cdTxt[8];
                    if (petCd >= 60.0f)
                        snprintf(cdTxt, sizeof(cdTxt), "%dm", static_cast<int>(petCd / 60.0f));
                    else if (petCd >= 1.0f)
                        snprintf(cdTxt, sizeof(cdTxt), "%d", static_cast<int>(petCd));
                    else
                        snprintf(cdTxt, sizeof(cdTxt), "%.1f", petCd);
                    ImVec2 tsz = ImGui::CalcTextSize(cdTxt);
                    float cx = (bMin.x + bMax.x) * 0.5f;
                    float cy = (bMin.y + bMax.y) * 0.5f;
                    cdDL->AddText(ImVec2(cx - tsz.x * 0.5f, cy - tsz.y * 0.5f),
                                  IM_COL32(255, 255, 255, 230), cdTxt);
                }

                if (clicked && !petOnCd) {
                    // Send pet action; use current target for spells.
                    // Attack takes the current target; so does any castable spell.
                    const bool wantsTarget =
                        isSpell || (game::pet::petActionType(slotVal) == game::pet::ActionType::Command &&
                                    actionId == game::pet::kAttack);
                    uint64_t targetGuid = wantsTarget ? gameHandler.getTargetGuid() : 0u;
                    gameHandler.sendPetAction(slotVal, targetGuid);
                }
                // Right-click toggles autocast for castable pet spells (actionId > 6)
                if (isSpell && ImGui::IsItemClicked(ImGuiMouseButton_Right)) {
                    gameHandler.togglePetSpellAutocast(actionId);
                }

                // Tooltip: rich spell info for pet spells, simple label for built-in commands
                if (ImGui::IsItemHovered()) {
                    if (builtinLabel) {
                        const char* tip = petBuiltinName(slotVal);
                        if (tip) ImGui::SetTooltip("%s", tip);
                    } else if (isSpell) {
                        auto* spellAsset = services_.assetManager;
                        ImGui::BeginTooltip();
                        bool richOk = spellbookScreen.renderSpellInfoTooltip(actionId, gameHandler, spellAsset);
                        if (!richOk) {
                            std::string nm = gameHandler.getSpellName(actionId);
                            if (nm.empty()) nm = "Spell #" + std::to_string(actionId);
                            ImGui::Text("%s", nm.c_str());
                        }
                        ImGui::TextColored(autocastOn
                            ? kColorGreen
                            : kColorGray,
                            "Autocast: %s (right-click to toggle)", autocastOn ? "On" : "Off");
                        if (petOnCd) {
                            if (petCd >= 60.0f)
                                ImGui::TextColored(kColorRed,
                                    "Cooldown: %d min %d sec",
                                    static_cast<int>(petCd) / 60, static_cast<int>(petCd) % 60);
                            else
                                ImGui::TextColored(kColorRed,
                                    "Cooldown: %.1f sec", petCd);
                        }
                        ImGui::EndTooltip();
                    }
                }

                ImGui::PopID();
                ++rendered;
            }
        }
    }
    ImGui::End();

    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar();
}

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
        { "Earth", IM_COL32(139, 90,  43, 255) },   // brown
        { "Fire",  IM_COL32(220, 80,  30, 255) },   // red-orange
        { "Water", IM_COL32( 30,120, 220, 255) },   // blue
        { "Air",   IM_COL32(180,220, 255, 255) },   // light blue
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

void GameScreen::renderTargetFrame(game::GameHandler& gameHandler) {
    auto target = gameHandler.getTarget();
    if (!target) {
        lastTargetFrameBottom_ = -1.0f;  // nothing targeted: frame is not drawn
        return;
    }

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;

    // The frame auto-sizes (AlwaysAutoResize) to its widest content, so long names,
    // subtitles, guild tags, and the level/classification line all fit without
    // clipping. A 250px floor keeps the default look; the health/power bars use -1
    // width so they fill whatever the frame grows to. Centering uses last frame's
    // measured width since the position is set before the window lays out.
    float frameW = lastTargetFrameWidth_;
    float frameX = (screenW - frameW) / 2.0f;

    ImGui::SetNextWindowPos(ImVec2(frameX, 30.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSizeConstraints(ImVec2(250.0f, 0.0f),
                                        ImVec2(screenW * 0.6f, static_cast<float>(window ? window->getHeight() : 720)));

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                             ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_AlwaysAutoResize;

    // Determine hostility/level color for border and name (WoW-canonical)
    ImVec4 hostileColor(0.7f, 0.7f, 0.7f, 1.0f);
    if (target->getType() == game::ObjectType::PLAYER) {
        hostileColor = kColorBrightGreen;
    } else if (target->getType() == game::ObjectType::UNIT) {
        auto u = std::static_pointer_cast<game::Unit>(target);
        if (u->getHealth() == 0 && u->getMaxHealth() > 0) {
            hostileColor = kColorDarkGray;
        } else if (u->isHostile()) {
            // Check tapped-by-other: grey name for mobs tagged by someone else
            uint32_t tgtDynFlags = u->getDynamicFlags();
            bool tgtTapped = (tgtDynFlags & 0x0004) != 0 && (tgtDynFlags & 0x0008) == 0;
            if (tgtTapped) {
                hostileColor = kColorGray;  // grey - tapped by someone else
            } else {
                hostileColor = helpers::levelDifficultyColor(gameHandler.getPlayerLevel(),
                                                             u->getLevel());
            }
        } else {
            hostileColor = kColorBrightGreen; // Friendly
        }
    }

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.1f, 0.1f, 0.85f));
    const uint64_t targetGuid = target->getGuid();
    const bool confirmedCombatWithTarget = gameHandler.isInCombatWith(targetGuid);
    const bool intentTowardTarget =
        gameHandler.hasAutoAttackIntent() &&
        gameHandler.getAutoAttackTargetGuid() == targetGuid &&
        !confirmedCombatWithTarget;
    ImVec4 borderColor = ImVec4(hostileColor.x * 0.8f, hostileColor.y * 0.8f, hostileColor.z * 0.8f, 1.0f);
    if (confirmedCombatWithTarget) {
        float t = ImGui::GetTime();
        float pulse = (std::fmod(t, 0.6f) < 0.3f) ? 1.0f : 0.0f;
        borderColor = ImVec4(1.0f, 0.1f, 0.1f, pulse);
    } else if (intentTowardTarget) {
        borderColor = ImVec4(1.0f, 0.7f, 0.2f, 1.0f);
    }
    ImGui::PushStyleColor(ImGuiCol_Border, borderColor);

    if (ImGui::Begin("##TargetFrame", nullptr, flags)) {
        // Record the auto-fitted width so next frame can center the window correctly.
        frameW = ImGui::GetWindowSize().x;
        lastTargetFrameWidth_ = frameW;
        // Same one-frame lag as the width above: the auto-resized size is last
        // frame's, which is close enough for parking another window underneath.
        lastTargetFrameBottom_ = ImGui::GetWindowPos().y + ImGui::GetWindowSize().y;
        // Raid mark icon (Star/Circle/Diamond/Triangle/Moon/Square/Cross/Skull),
        // drawn from the Blizzard artwork - the font has no glyphs for most of
        // these symbols, so the previous text version rendered as '?' boxes.
        uint8_t mark = gameHandler.getEntityRaidMark(target->getGuid());
        if (mark < game::GameHandler::kRaidMarkCount) {
            if (VkDescriptorSet markTex = ui::getRaidTargetIcon(mark, services_.assetManager)) {
                constexpr float kMarkSize = 16.0f;
                ImVec2 p = ImGui::GetCursorScreenPos();
                ImGui::GetWindowDrawList()->AddImage(
                    (ImTextureID)(uintptr_t)markTex, p,
                    ImVec2(p.x + kMarkSize, p.y + kMarkSize));
                ImGui::SetCursorPosX(ImGui::GetCursorPosX() + kMarkSize + 2.0f);
            }
        }

        // Entity name and type - Selectable so we can attach a right-click context menu
        std::string name = game::entityDisplayName(target);

        // Player targets: use class color instead of the generic green
        ImVec4 nameColor = hostileColor;
        if (target->getType() == game::ObjectType::PLAYER) {
            uint8_t cid = entityClassId(target.get());
            if (cid != 0) nameColor = classColorVec4(cid);
        }

        ImGui::SameLine(0.0f, 0.0f);
        ImGui::PushStyleColor(ImGuiCol_Text, nameColor);
        ImGui::PushStyleColor(ImGuiCol_Header,        ImVec4(0,0,0,0));
        ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImVec4(1,1,1,0.08f));
        ImGui::PushStyleColor(ImGuiCol_HeaderActive,  ImVec4(1,1,1,0.12f));
        ImGui::Selectable(name.c_str(), false, ImGuiSelectableFlags_DontClosePopups,
                          ImVec2(ImGui::CalcTextSize(name.c_str()).x, 0));
        ImGui::PopStyleColor(4);

        // Right-click context menu on target frame
        if (ImGui::BeginPopupContextItem("##TargetFrameCtx")) {
            const bool isPlayer = (target->getType() == game::ObjectType::PLAYER);
            const uint64_t tGuid = target->getGuid();

            ImGui::TextDisabled("%s", name.c_str());
            ImGui::Separator();

            if (ImGui::MenuItem("Set Focus"))
                gameHandler.setFocus(tGuid);
            if (ImGui::MenuItem("Clear Target"))
                gameHandler.clearTarget();
            if (isPlayer) {
                ImGui::Separator();
                if (ImGui::MenuItem("Whisper")) {
                    chatPanel_.setWhisperTarget(name);
                }
                if (ImGui::MenuItem("Follow"))
                    gameHandler.followTarget();
                if (ImGui::MenuItem("Invite to Group"))
                    gameHandler.inviteToGroup(name);
                if (ImGui::MenuItem("Trade"))
                    gameHandler.initiateTrade(tGuid);
                if (ImGui::MenuItem("Duel"))
                    gameHandler.proposeDuel(tGuid);
                if (ImGui::MenuItem("Inspect")) {
                    gameHandler.inspectTarget();
                    socialPanel_.openInspectWindow(gameHandler);
                }
                ImGui::Separator();
                if (ImGui::MenuItem("Add Friend"))
                    gameHandler.addFriend(name);
                if (ImGui::MenuItem("Ignore"))
                    gameHandler.addIgnore(name);
                if (ImGui::MenuItem("Report Player"))
                    gameHandler.reportPlayer(tGuid, "Reported via UI");
            }
            ImGui::Separator();
            if (ImGui::BeginMenu("Set Raid Mark")) {
                for (int mi = 0; mi < 8; ++mi) {
                    if (ImGui::MenuItem(kRaidMarkNames[mi]))
                        gameHandler.setRaidMark(tGuid, static_cast<uint8_t>(mi));
                }
                ImGui::Separator();
                if (ImGui::MenuItem("Clear Mark"))
                    gameHandler.setRaidMark(tGuid, 0xFF);
                ImGui::EndMenu();
            }
            ImGui::EndPopup();
        }

        // Group leader crown - golden ♛ when the targeted player is the party/raid leader
        if (gameHandler.isInGroup() && target->getType() == game::ObjectType::PLAYER) {
            if (gameHandler.getPartyData().leaderGuid == target->getGuid()) {
                ImGui::SameLine(0, 4);
                ImGui::TextColored(colors::kSymbolGold, "\xe2\x99\x9b");
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("Group Leader");
            }
        }

        // Quest giver indicator - "!" for available quests, "?" for completable quests
        {
            const auto marker = game::questGiverMarker(
                gameHandler.getQuestGiverStatus(target->getGuid()));
            if (marker.symbol) {
                ImGui::SameLine(0, 4);
                ImGui::TextColored(marker.dim ? kColorGray : colors::kBrightGold,
                                   "%s", marker.symbol);
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", marker.tooltip);
            }
        }

        // Player class, tinted with the class colour, flowed inline after the name.
        // Note: do NOT right-align this to GetWindowContentRegionMax() - on an
        // AlwaysAutoResize window that edge is the *previous* frame's width, so
        // pinning content to it makes the frame keep any width a prior (wider)
        // target gave it and never shrink back. Inline keeps it "wide enough for
        // the text, but no wider".
        if (target->getType() == game::ObjectType::PLAYER) {
            uint8_t cid = entityClassId(target.get());
            if (cid != 0) {  // 0 = class not received yet; would read as "Unknown"
                const char* cls = classNameStr(cid);
                ImGui::SameLine(0.0f, 8.0f);
                ImGui::TextColored(classColorVec4(cid), "%s", cls);
            }
        }

        // Creature subtitle (e.g. "<Warchief of the Horde>", "Captain of the Guard")
        if (target->getType() == game::ObjectType::UNIT) {
            auto unit = std::static_pointer_cast<game::Unit>(target);
            const std::string sub = gameHandler.getCachedCreatureSubName(unit->getEntry());
            if (!sub.empty()) {
                ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 0.9f), "<%s>", sub.c_str());
            }
        }

        // Player guild name (e.g. "<My Guild>") - mirrors NPC subtitle styling
        if (target->getType() == game::ObjectType::PLAYER) {
            uint32_t guildId = gameHandler.getEntityGuildId(target->getGuid());
            if (guildId != 0) {
                const std::string& gn = gameHandler.lookupGuildName(guildId);
                if (!gn.empty()) {
                    ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 0.9f), "<%s>", gn.c_str());
                }
            }
        }

        // Right-click context menu on the target name
        if (ImGui::BeginPopupContextItem("##TargetNameCtx")) {
            const bool isPlayer = (target->getType() == game::ObjectType::PLAYER);
            const uint64_t tGuid = target->getGuid();

            ImGui::TextDisabled("%s", name.c_str());
            ImGui::Separator();

            if (ImGui::MenuItem("Set Focus")) {
                gameHandler.setFocus(tGuid);
            }
            if (ImGui::MenuItem("Clear Target")) {
                gameHandler.clearTarget();
            }
            if (isPlayer) {
                ImGui::Separator();
                if (ImGui::MenuItem("Whisper")) {
                    chatPanel_.setWhisperTarget(name);
                }
                if (ImGui::MenuItem("Follow")) {
                    gameHandler.followTarget();
                }
                if (ImGui::MenuItem("Invite to Group")) {
                    gameHandler.inviteToGroup(name);
                }
                if (ImGui::MenuItem("Trade")) {
                    gameHandler.initiateTrade(tGuid);
                }
                if (ImGui::MenuItem("Duel")) {
                    gameHandler.proposeDuel(tGuid);
                }
                if (ImGui::MenuItem("Inspect")) {
                    gameHandler.inspectTarget();
                    socialPanel_.openInspectWindow(gameHandler);
                }
                ImGui::Separator();
                if (ImGui::MenuItem("Add Friend")) {
                    gameHandler.addFriend(name);
                }
                if (ImGui::MenuItem("Ignore")) {
                    gameHandler.addIgnore(name);
                }
            }
            ImGui::Separator();
            if (ImGui::BeginMenu("Set Raid Mark")) {
                for (int mi = 0; mi < 8; ++mi) {
                    if (ImGui::MenuItem(kRaidMarkNames[mi]))
                        gameHandler.setRaidMark(tGuid, static_cast<uint8_t>(mi));
                }
                ImGui::Separator();
                if (ImGui::MenuItem("Clear Mark"))
                    gameHandler.setRaidMark(tGuid, 0xFF);
                ImGui::EndMenu();
            }
            ImGui::EndPopup();
        }

        // Level (for units/players) - colored by difficulty
        if (target->getType() == game::ObjectType::UNIT || target->getType() == game::ObjectType::PLAYER) {
            auto unit = std::static_pointer_cast<game::Unit>(target);
            ImGui::SameLine();
            // Level color matches the hostility/difficulty color
            ImVec4 levelColor = hostileColor;
            if (target->getType() == game::ObjectType::PLAYER) {
                levelColor = ui::colors::kLightGray;
            }
            if (unit->getLevel() == 0)
                ImGui::TextColored(levelColor, "Lv ??");
            else
                ImGui::TextColored(levelColor, "Lv %u", unit->getLevel());
            // Classification badge: Elite / Rare Elite / Boss / Rare
            if (target->getType() == game::ObjectType::UNIT) {
                int rank = gameHandler.getCreatureRank(unit->getEntry());
                if (rank == 1) {
                    ImGui::SameLine(0, 4);
                    ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "[Elite]");
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Elite - requires a group");
                } else if (rank == 2) {
                    ImGui::SameLine(0, 4);
                    ImGui::TextColored(ImVec4(0.8f, 0.4f, 1.0f, 1.0f), "[Rare Elite]");
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Rare Elite - uncommon spawn, group recommended");
                } else if (rank == 3) {
                    ImGui::SameLine(0, 4);
                    ImGui::TextColored(kColorRed, "[Boss]");
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Boss - raid / dungeon boss");
                } else if (rank == 4) {
                    ImGui::SameLine(0, 4);
                    ImGui::TextColored(ImVec4(0.5f, 0.9f, 1.0f, 1.0f), "[Rare]");
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Rare - uncommon spawn with better loot");
                }
            }
            // Creature type label (Beast, Humanoid, Demon, etc.)
            if (target->getType() == game::ObjectType::UNIT) {
                uint32_t ctype = gameHandler.getCreatureType(unit->getEntry());
                const char* ctypeName = nullptr;
                switch (ctype) {
                    case 1:  ctypeName = "Beast"; break;
                    case 2:  ctypeName = "Dragonkin"; break;
                    case 3:  ctypeName = "Demon"; break;
                    case 4:  ctypeName = "Elemental"; break;
                    case 5:  ctypeName = "Giant"; break;
                    case 6:  ctypeName = "Undead"; break;
                    case 7:  ctypeName = "Humanoid"; break;
                    case 8:  ctypeName = "Critter"; break;
                    case 9:  ctypeName = "Mechanical"; break;
                    case 11: ctypeName = "Totem"; break;
                    case 12: ctypeName = "Non-combat Pet"; break;
                    case 13: ctypeName = "Gas Cloud"; break;
                    default: break;
                }
                if (ctypeName) {
                    ImGui::SameLine(0, 4);
                    ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 0.9f), "(%s)", ctypeName);
                }
            }
            if (confirmedCombatWithTarget) {
                float cPulse = 0.75f + 0.25f * std::sin(static_cast<float>(ImGui::GetTime()) * 4.0f);
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(1.0f, 0.2f * cPulse, 0.2f * cPulse, 1.0f), "[Attacking]");
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("Engaged in combat with this target");
            }

            // Health bar
            uint32_t hp = unit->getHealth();
            uint32_t maxHp = unit->getMaxHealth();
            if (maxHp > 0) {
                float pct = static_cast<float>(hp) / static_cast<float>(maxHp);
                ImGui::PushStyleColor(ImGuiCol_PlotHistogram, colors::healthBarColor(pct));

                char overlay[64];
                snprintf(overlay, sizeof(overlay), "%u / %u", hp, maxHp);
                ImGui::ProgressBar(pct, ImVec2(-1, 18), overlay);
                ImGui::PopStyleColor();
                // Target power bar (mana/rage/energy)
                uint8_t targetPowerType = unit->getPowerType();
                uint32_t targetPower = unit->getPower();
                uint32_t targetMaxPower = unit->getMaxPower();
                if (targetMaxPower == 0 && (targetPowerType == 1 || targetPowerType == 3)) targetMaxPower = 100;
                if (targetMaxPower > 0) {
                    float mpPct = static_cast<float>(targetPower) / static_cast<float>(targetMaxPower);
                    ImVec4 targetPowerColor;
                    targetPowerColor = colors::powerTypeColor(static_cast<uint8_t>(targetPowerType));
                    ImGui::PushStyleColor(ImGuiCol_PlotHistogram, targetPowerColor);
                    char mpOverlay[64];
                    snprintf(mpOverlay, sizeof(mpOverlay), "%u / %u", targetPower, targetMaxPower);
                    ImGui::ProgressBar(mpPct, ImVec2(-1, 18), mpOverlay);
                    ImGui::PopStyleColor();
                }
            } else {
                ImGui::TextDisabled("No health data");
            }
        }

        // Combo points - shown when the player has combo points on this target
        {
            uint8_t cp = gameHandler.getComboPoints();
            if (cp > 0 && gameHandler.getComboTarget() == target->getGuid()) {
                const float dotSize = 12.0f;
                const float dotSpacing = 4.0f;
                const int maxCP = 5;
                float totalW = maxCP * dotSize + (maxCP - 1) * dotSpacing;
                float startX = (frameW - totalW) * 0.5f;
                ImGui::SetCursorPosX(startX);
                ImVec2 cursor = ImGui::GetCursorScreenPos();
                ImDrawList* dl = ImGui::GetWindowDrawList();
                for (int ci = 0; ci < maxCP; ++ci) {
                    float cx = cursor.x + ci * (dotSize + dotSpacing) + dotSize * 0.5f;
                    float cy = cursor.y + dotSize * 0.5f;
                    if (ci < static_cast<int>(cp)) {
                        // Lit: yellow for 1-4, red glow for 5
                        ImU32 col = (cp >= 5)
                            ? IM_COL32(255, 50, 30, 255)
                            : IM_COL32(255, 210, 30, 255);
                        dl->AddCircleFilled(ImVec2(cx, cy), dotSize * 0.45f, col);
                        // Subtle glow
                        dl->AddCircle(ImVec2(cx, cy), dotSize * 0.5f, IM_COL32(255, 255, 200, 80), 0, 1.5f);
                    } else {
                        // Unlit: dark outline
                        dl->AddCircle(ImVec2(cx, cy), dotSize * 0.4f, IM_COL32(80, 80, 80, 180), 0, 1.5f);
                    }
                }
                ImGui::Dummy(ImVec2(totalW, dotSize + 2.0f));
            }
        }

        // Target cast bar - shown when the target is casting
        if (gameHandler.isTargetCasting()) {
            float castPct   = gameHandler.getTargetCastProgress();
            float castLeft  = gameHandler.getTargetCastTimeRemaining();
            uint32_t tspell = gameHandler.getTargetCastSpellId();
            bool interruptible = gameHandler.isTargetCastInterruptible();
            const std::string& castName = (tspell != 0) ? gameHandler.getSpellName(tspell) : "";
            // Color: interruptible = green (can Kick/CS), not interruptible = red, both pulse when >80%
            ImVec4 castBarColor;
            if (castPct > 0.8f) {
                float pulse = 0.7f + 0.3f * std::sin(static_cast<float>(ImGui::GetTime()) * 8.0f);
                if (interruptible)
                    castBarColor = ImVec4(0.2f * pulse, 0.9f * pulse, 0.2f * pulse, 1.0f);  // green pulse
                else
                    castBarColor = ImVec4(1.0f * pulse, 0.1f * pulse, 0.1f * pulse, 1.0f);  // red pulse
            } else {
                castBarColor = interruptible ? colors::kCastGreen   // green = can interrupt
                                             : ImVec4(0.85f, 0.15f, 0.15f, 1.0f); // red = uninterruptible
            }
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, castBarColor);
            char castLabel[72];
            if (!castName.empty())
                snprintf(castLabel, sizeof(castLabel), "%s (%.1fs)", castName.c_str(), castLeft);
            else if (tspell != 0)
                snprintf(castLabel, sizeof(castLabel), "Spell #%u (%.1fs)", tspell, castLeft);
            else
                snprintf(castLabel, sizeof(castLabel), "Casting... (%.1fs)", castLeft);
            {
                auto* tcastAsset = services_.assetManager;
                VkDescriptorSet tIcon = (tspell != 0 && tcastAsset)
                    ? getSpellIcon(tspell, tcastAsset) : VK_NULL_HANDLE;
                if (tIcon) {
                    ImGui::Image((ImTextureID)(uintptr_t)tIcon, ImVec2(14, 14));
                    ImGui::SameLine(0, 2);
                    ImGui::ProgressBar(castPct, ImVec2(-1, 14), castLabel);
                } else {
                    ImGui::ProgressBar(castPct, ImVec2(-1, 14), castLabel);
                }
            }
            ImGui::PopStyleColor();
        }

        // Target-of-Target (ToT): show who the current target is targeting
        {
            const uint64_t totGuid = game::unitTargetGuid(target);
            if (totGuid != 0 && totGuid != targetGuid) {
                auto totEnt = gameHandler.getEntityManager().getEntity(totGuid);
                std::string totName;
                ImVec4 totColor(0.7f, 0.7f, 0.7f, 1.0f);
                if (totGuid == gameHandler.getPlayerGuid()) {
                    auto playerEnt = gameHandler.getEntityManager().getEntity(totGuid);
                    totName = playerEnt ? game::entityDisplayName(playerEnt) : "You";
                    totColor = kColorBrightGreen;
                } else if (totEnt) {
                    totName = game::entityDisplayName(totEnt);
                    uint8_t cid = entityClassId(totEnt.get());
                    if (cid != 0) totColor = classColorVec4(cid);
                }
                if (!totName.empty()) {
                    ImGui::TextDisabled("▶");
                    ImGui::SameLine(0, 2);
                    ImGui::TextColored(totColor, "%s", totName.c_str());
                    if (ImGui::IsItemHovered()) {
                        ImGui::SetTooltip("Target's target: %s\nClick to target", totName.c_str());
                    }
                    if (ImGui::IsItemClicked()) {
                        gameHandler.setTarget(totGuid);
                    }

                    // Compact health bar for the ToT - essential for healers tracking boss target
                    if (totEnt) {
                        auto totUnit = std::dynamic_pointer_cast<game::Unit>(totEnt);
                        if (totUnit && totUnit->getMaxHealth() > 0) {
                            uint32_t totHp    = totUnit->getHealth();
                            uint32_t totMaxHp = totUnit->getMaxHealth();
                            float totPct = static_cast<float>(totHp) / static_cast<float>(totMaxHp);
                            ImGui::PushStyleColor(ImGuiCol_PlotHistogram,
                                                  colors::healthBarColor(totPct));
                            ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.15f, 0.15f, 0.15f, 0.8f));
                            char totOverlay[32];
                            snprintf(totOverlay, sizeof(totOverlay), "%u%%",
                                     static_cast<unsigned>(std::lround(totPct * 100.0f)));
                            ImGui::ProgressBar(totPct, ImVec2(-1, 10), totOverlay);
                            ImGui::PopStyleColor(2);
                        }
                    }
                }
            }
        }

        // Distance
        const auto& movement = gameHandler.getMovementInfo();
        float dx = target->getX() - movement.x;
        float dy = target->getY() - movement.y;
        float dz = target->getZ() - movement.z;
        float distance = std::sqrt(dx*dx + dy*dy + dz*dz);
        ImGui::TextDisabled("%.1f yd", distance);

        // Threat button (shown when in combat and threat data is available)
        if (gameHandler.getTargetThreatList()) {
            ImGui::SameLine();
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.5f, 0.1f, 0.1f, 0.8f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.7f, 0.2f, 0.2f, 0.9f));
            if (ImGui::SmallButton("Threat")) combatUI_.showThreatWindow_ = !combatUI_.showThreatWindow_;
            ImGui::PopStyleColor(2);
        }

        // Target auras (buffs/debuffs)
        const auto& targetAuras = gameHandler.getTargetAuras();
        int activeAuras = 0;
        for (const auto& a : targetAuras) {
            if (!a.isEmpty()) activeAuras++;
        }
        if (activeAuras > 0) {
            auto* assetMgr = services_.assetManager;
            constexpr float ICON_SIZE = 24.0f;
            constexpr int ICONS_PER_ROW = 8;

            ImGui::Separator();

            // Build sorted index list: debuffs before buffs, shorter duration first
            uint64_t tNowSort = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count());
            std::vector<size_t> sortedIdx;
            sortedIdx.reserve(targetAuras.size());
            for (size_t i = 0; i < targetAuras.size(); ++i)
                if (!targetAuras[i].isEmpty()) sortedIdx.push_back(i);
            std::sort(sortedIdx.begin(), sortedIdx.end(), [&](size_t a, size_t b) {
                const auto& aa = targetAuras[a]; const auto& ab = targetAuras[b];
                bool aDebuff = (aa.flags & 0x80) != 0;
                bool bDebuff = (ab.flags & 0x80) != 0;
                if (aDebuff != bDebuff) return aDebuff > bDebuff; // debuffs first
                int32_t ra = aa.getRemainingMs(tNowSort);
                int32_t rb = ab.getRemainingMs(tNowSort);
                // Permanent (-1) goes last; shorter remaining goes first
                if (ra < 0 && rb < 0) return false;
                if (ra < 0) return false;
                if (rb < 0) return true;
                return ra < rb;
            });

            int shown = 0;
            for (size_t si = 0; si < sortedIdx.size() && shown < 16; ++si) {
                size_t i = sortedIdx[si];
                const auto& aura = targetAuras[i];
                if (aura.isEmpty()) continue;

                if (shown > 0 && shown % ICONS_PER_ROW != 0) ImGui::SameLine();

                ImGui::PushID(static_cast<int>(10000 + i));

                bool isBuff = (aura.flags & 0x80) == 0;
                const ImVec4 auraBorderColor =
                    wowee::ui::auraBorderColor(isBuff, gameHandler.getSpellDispelType(aura.spellId));

                VkDescriptorSet iconTex = VK_NULL_HANDLE;
                if (assetMgr) {
                    iconTex = getSpellIcon(aura.spellId, assetMgr);
                }

                if (iconTex) {
                    ImGui::PushStyleColor(ImGuiCol_Button, auraBorderColor);
                    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(1, 1));
                    ImGui::ImageButton("##taura",
                        (ImTextureID)(uintptr_t)iconTex,
                        ImVec2(ICON_SIZE - 2, ICON_SIZE - 2));
                    ImGui::PopStyleVar();
                    ImGui::PopStyleColor();
                } else {
                    ImGui::PushStyleColor(ImGuiCol_Button, auraBorderColor);
                    const std::string& tAuraName = gameHandler.getSpellName(aura.spellId);
                    char label[32];
                    if (!tAuraName.empty())
                        snprintf(label, sizeof(label), "%.6s", tAuraName.c_str());
                    else
                        snprintf(label, sizeof(label), "%u", aura.spellId);
                    ImGui::Button(label, ImVec2(ICON_SIZE, ICON_SIZE));
                    ImGui::PopStyleColor();
                }

                // Compute remaining once for overlay + tooltip
                uint64_t tNowMs = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count());
                int32_t tRemainMs = aura.getRemainingMs(tNowMs);

                // Clock-sweep overlay (elapsed = dark area, WoW style)
                if (tRemainMs > 0 && aura.maxDurationMs > 0) {
                    ImVec2 tIconMin = ImGui::GetItemRectMin();
                    ImVec2 tIconMax = ImGui::GetItemRectMax();
                    float tcx = (tIconMin.x + tIconMax.x) * 0.5f;
                    float tcy = (tIconMin.y + tIconMax.y) * 0.5f;
                    float tR  = (tIconMax.x - tIconMin.x) * 0.5f;
                    float tTot = static_cast<float>(aura.maxDurationMs);
                    float tFrac = std::clamp(
                        1.0f - static_cast<float>(tRemainMs) / tTot, 0.0f, 1.0f);
                    if (tFrac > 0.005f) {
                        constexpr int TSEGS = 24;
                        float tSa = -IM_PI * 0.5f;
                        float tEa = tSa + tFrac * 2.0f * IM_PI;
                        ImVec2 tPts[TSEGS + 2];
                        tPts[0] = ImVec2(tcx, tcy);
                        for (int s = 0; s <= TSEGS; ++s) {
                            float a = tSa + (tEa - tSa) * s / static_cast<float>(TSEGS);
                            tPts[s + 1] = ImVec2(tcx + std::cos(a) * tR,
                                                 tcy + std::sin(a) * tR);
                        }
                        ImGui::GetWindowDrawList()->AddConvexPolyFilled(
                            tPts, TSEGS + 2, IM_COL32(0, 0, 0, 145));
                    }
                }

                // Duration countdown overlay
                if (tRemainMs > 0) {
                    ImVec2 iconMin = ImGui::GetItemRectMin();
                    ImVec2 iconMax = ImGui::GetItemRectMax();
                    char timeStr[12];
                    int secs = (tRemainMs + 999) / 1000;
                    if (secs >= 3600)
                        snprintf(timeStr, sizeof(timeStr), "%dh", secs / 3600);
                    else if (secs >= 60)
                        snprintf(timeStr, sizeof(timeStr), "%d:%02d", secs / 60, secs % 60);
                    else
                        snprintf(timeStr, sizeof(timeStr), "%d", secs);
                    ImVec2 textSize = ImGui::CalcTextSize(timeStr);
                    float cx = iconMin.x + (iconMax.x - iconMin.x - textSize.x) * 0.5f;
                    float cy = iconMax.y - textSize.y - 1.0f;
                    // Color by urgency (matches player buff bar)
                    ImU32 tTimerColor;
                    if (tRemainMs < 10000) {
                        float pulse = 0.7f + 0.3f * std::sin(
                            static_cast<float>(ImGui::GetTime()) * 6.0f);
                        tTimerColor = IM_COL32(
                            static_cast<int>(255 * pulse),
                            static_cast<int>(80 * pulse),
                            static_cast<int>(60 * pulse), 255);
                    } else if (tRemainMs < 30000) {
                        tTimerColor = IM_COL32(255, 165, 0, 255);
                    } else {
                        tTimerColor = IM_COL32(255, 255, 255, 255);
                    }
                    ImGui::GetWindowDrawList()->AddText(ImVec2(cx + 1, cy + 1),
                        IM_COL32(0, 0, 0, 200), timeStr);
                    ImGui::GetWindowDrawList()->AddText(ImVec2(cx, cy),
                        tTimerColor, timeStr);
                }

                // Stack / charge count - upper-left corner
                if (aura.charges > 1) {
                    ImVec2 iconMin = ImGui::GetItemRectMin();
                    char chargeStr[8];
                    snprintf(chargeStr, sizeof(chargeStr), "%u", static_cast<unsigned>(aura.charges));
                    ImGui::GetWindowDrawList()->AddText(ImVec2(iconMin.x + 3, iconMin.y + 3),
                        IM_COL32(0, 0, 0, 200), chargeStr);
                    ImGui::GetWindowDrawList()->AddText(ImVec2(iconMin.x + 2, iconMin.y + 2),
                        IM_COL32(255, 220, 50, 255), chargeStr);
                }

                // Tooltip: rich spell info + remaining duration
                if (ImGui::IsItemHovered()) {
                    ImGui::BeginTooltip();
                    bool richOk = spellbookScreen.renderSpellInfoTooltip(aura.spellId, gameHandler, assetMgr);
                    if (!richOk) {
                        std::string name = spellbookScreen.lookupSpellName(aura.spellId, assetMgr);
                        if (name.empty()) name = "Spell #" + std::to_string(aura.spellId);
                        ImGui::Text("%s", name.c_str());
                    }
                    renderAuraRemaining(tRemainMs);
                    ImGui::EndTooltip();
                }

                ImGui::PopID();
                shown++;
            }
        }
    }
    ImGui::End();

    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar();

    // ---- Target-of-Target (ToT) mini frame ----
    // Who the target has selected. See game::unitTargetGuid.
    if (target) {
        const uint64_t totGuid = game::unitTargetGuid(target);

        if (totGuid != 0 && totGuid != targetGuid) {
            auto totEntity = gameHandler.getEntityManager().getEntity(totGuid);
            if (totEntity) {
                // Position ToT frame just below and right-aligned with the target frame
                float totW = 160.0f;
                float totX = (screenW - totW) / 2.0f + (frameW - totW);
                ImGui::SetNextWindowPos(ImVec2(totX, 30.0f + 130.0f), ImGuiCond_Always);
                ImGui::SetNextWindowSize(ImVec2(totW, 0.0f), ImGuiCond_Always);

                ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 3.0f);
                ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.08f, 0.08f, 0.08f, 0.80f));
                ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.4f, 0.4f, 0.4f, 0.7f));

                if (ImGui::Begin("##ToTFrame", nullptr,
                        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                        ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                        ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar)) {
                    std::string totName = game::entityDisplayName(totEntity);
                    // Class color for players; gray for NPCs
                    ImVec4 totNameColor = colors::kSilver;
                    if (totEntity->getType() == game::ObjectType::PLAYER) {
                        uint8_t cid = entityClassId(totEntity.get());
                        if (cid != 0) totNameColor = classColorVec4(cid);
                    }
                    // Selectable so we can attach a right-click context menu
                    ImGui::PushStyleColor(ImGuiCol_Text, totNameColor);
                    ImGui::PushStyleColor(ImGuiCol_Header,        ImVec4(0,0,0,0));
                    ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImVec4(1,1,1,0.08f));
                    ImGui::PushStyleColor(ImGuiCol_HeaderActive,  ImVec4(1,1,1,0.12f));
                    if (ImGui::Selectable(totName.c_str(), false,
                            ImGuiSelectableFlags_DontClosePopups,
                            ImVec2(ImGui::CalcTextSize(totName.c_str()).x, 0))) {
                        gameHandler.setTarget(totGuid);
                    }
                    ImGui::PopStyleColor(4);

                    if (ImGui::BeginPopupContextItem("##ToTCtx")) {
                        ImGui::TextDisabled("%s", totName.c_str());
                        ImGui::Separator();
                        if (ImGui::MenuItem("Target"))
                            gameHandler.setTarget(totGuid);
                        if (ImGui::MenuItem("Set Focus"))
                            gameHandler.setFocus(totGuid);
                        ImGui::EndPopup();
                    }

                    if (totEntity->getType() == game::ObjectType::UNIT ||
                        totEntity->getType() == game::ObjectType::PLAYER) {
                        auto totUnit = std::static_pointer_cast<game::Unit>(totEntity);
                        if (totUnit->getLevel() > 0) {
                            ImGui::SameLine();
                            ImGui::TextDisabled("Lv%u", totUnit->getLevel());
                        }
                        uint32_t hp = totUnit->getHealth();
                        uint32_t maxHp = totUnit->getMaxHealth();
                        if (maxHp > 0) {
                            float pct = static_cast<float>(hp) / static_cast<float>(maxHp);
                            ImGui::PushStyleColor(ImGuiCol_PlotHistogram,
                                                  colors::healthBarColor(pct));
                            ImGui::ProgressBar(pct, ImVec2(-1, 10), "");
                            ImGui::PopStyleColor();
                        }

                        // ToT cast bar - green if interruptible, red if not; pulses near completion
                        if (auto* totCs = gameHandler.getUnitCastState(totGuid)) {
                            float totCastPct = (totCs->timeTotal > 0.0f)
                                ? (totCs->timeTotal - totCs->timeRemaining) / totCs->timeTotal : 0.0f;
                            ImVec4 tcColor;
                            if (totCastPct > 0.8f) {
                                float pulse = 0.7f + 0.3f * std::sin(static_cast<float>(ImGui::GetTime()) * 8.0f);
                                tcColor = totCs->interruptible
                                    ? ImVec4(0.2f * pulse, 0.9f * pulse, 0.2f * pulse, 1.0f)
                                    : ImVec4(1.0f * pulse, 0.1f * pulse, 0.1f * pulse, 1.0f);
                            } else {
                                tcColor = totCs->interruptible
                                    ? colors::kCastGreen
                                    : ImVec4(0.85f, 0.15f, 0.15f, 1.0f);
                            }
                            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, tcColor);
                            char tcLabel[48];
                            const std::string& tcName = gameHandler.getSpellName(totCs->spellId);
                            if (!tcName.empty())
                                snprintf(tcLabel, sizeof(tcLabel), "%s (%.1fs)", tcName.c_str(), totCs->timeRemaining);
                            else
                                snprintf(tcLabel, sizeof(tcLabel), "Casting... (%.1fs)", totCs->timeRemaining);
                            ImGui::ProgressBar(totCastPct, ImVec2(-1, 8), tcLabel);
                            ImGui::PopStyleColor();
                        }

                        // ToT aura row - compact icons, debuffs first
                        {
                            const std::vector<game::AuraSlot>* totAuras = nullptr;
                            if (totGuid == gameHandler.getPlayerGuid())
                                totAuras = &gameHandler.getPlayerAuras();
                            else if (totGuid == gameHandler.getTargetGuid())
                                totAuras = &gameHandler.getTargetAuras();
                            else
                                totAuras = gameHandler.getUnitAuras(totGuid);

                            if (totAuras) {
                                int totActive = 0;
                                for (const auto& a : *totAuras) if (!a.isEmpty()) totActive++;
                                if (totActive > 0) {
                                    auto* totAsset = services_.assetManager;
                                    constexpr float TA_ICON = 16.0f;
                                    constexpr int   TA_PER_ROW = 8;

                                    ImGui::Separator();

                                    uint64_t taNowMs = static_cast<uint64_t>(
                                        std::chrono::duration_cast<std::chrono::milliseconds>(
                                            std::chrono::steady_clock::now().time_since_epoch()).count());

                                    std::vector<size_t> taIdx;
                                    taIdx.reserve(totAuras->size());
                                    for (size_t i = 0; i < totAuras->size(); ++i)
                                        if (!(*totAuras)[i].isEmpty()) taIdx.push_back(i);
                                    std::sort(taIdx.begin(), taIdx.end(), [&](size_t a, size_t b) {
                                        bool aD = ((*totAuras)[a].flags & 0x80) != 0;
                                        bool bD = ((*totAuras)[b].flags & 0x80) != 0;
                                        if (aD != bD) return aD > bD;
                                        int32_t ra = (*totAuras)[a].getRemainingMs(taNowMs);
                                        int32_t rb = (*totAuras)[b].getRemainingMs(taNowMs);
                                        if (ra < 0 && rb < 0) return false;
                                        if (ra < 0) return false;
                                        if (rb < 0) return true;
                                        return ra < rb;
                                    });

                                    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(2.0f, 2.0f));
                                    int taShown = 0;
                                    for (size_t si = 0; si < taIdx.size() && taShown < 16; ++si) {
                                        const auto& aura = (*totAuras)[taIdx[si]];
                                        bool isBuff = (aura.flags & 0x80) == 0;

                                        if (taShown > 0 && taShown % TA_PER_ROW != 0) ImGui::SameLine();
                                        ImGui::PushID(static_cast<int>(taIdx[si]) + 5000);

                                        const ImVec4 borderCol =
                                            wowee::ui::auraBorderColor(isBuff, gameHandler.getSpellDispelType(aura.spellId));

                                        VkDescriptorSet taIcon = (totAsset)
                                            ? getSpellIcon(aura.spellId, totAsset) : VK_NULL_HANDLE;
                                        if (taIcon) {
                                            ImGui::PushStyleColor(ImGuiCol_Button, borderCol);
                                            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(1, 1));
                                            ImGui::ImageButton("##taura",
                                                (ImTextureID)(uintptr_t)taIcon,
                                                ImVec2(TA_ICON - 2, TA_ICON - 2));
                                            ImGui::PopStyleVar();
                                            ImGui::PopStyleColor();
                                        } else {
                                            ImGui::PushStyleColor(ImGuiCol_Button, borderCol);
                                            char lab[8];
                                            snprintf(lab, sizeof(lab), "%u", aura.spellId % 10000);
                                            ImGui::Button(lab, ImVec2(TA_ICON, TA_ICON));
                                            ImGui::PopStyleColor();
                                        }

                                        // Duration overlay
                                        int32_t taRemain = aura.getRemainingMs(taNowMs);
                                        if (taRemain > 0) {
                                            ImVec2 imin = ImGui::GetItemRectMin();
                                            ImVec2 imax = ImGui::GetItemRectMax();
                                            char ts[12];
                                            fmtDurationCompact(ts, sizeof(ts), (taRemain + 999) / 1000);
                                            ImVec2 tsz = ImGui::CalcTextSize(ts);
                                            float cx = imin.x + (imax.x - imin.x - tsz.x) * 0.5f;
                                            float cy = imax.y - tsz.y;
                                            ImGui::GetWindowDrawList()->AddText(ImVec2(cx + 1, cy + 1), IM_COL32(0, 0, 0, 180), ts);
                                            ImGui::GetWindowDrawList()->AddText(ImVec2(cx, cy), IM_COL32(255, 255, 255, 220), ts);
                                        }

                                        // Tooltip
                                        if (ImGui::IsItemHovered()) {
                                            ImGui::BeginTooltip();
                                            bool richOk = spellbookScreen.renderSpellInfoTooltip(
                                                aura.spellId, gameHandler, totAsset);
                                            if (!richOk) {
                                                std::string nm = spellbookScreen.lookupSpellName(aura.spellId, totAsset);
                                                if (nm.empty()) nm = "Spell #" + std::to_string(aura.spellId);
                                                ImGui::Text("%s", nm.c_str());
                                            }
                                            renderAuraRemaining(taRemain);
                                            ImGui::EndTooltip();
                                        }

                                        ImGui::PopID();
                                        taShown++;
                                    }
                                    ImGui::PopStyleVar();
                                }
                            }
                        }
                    }
                }
                ImGui::End();
                ImGui::PopStyleColor(2);
                ImGui::PopStyleVar();
            }
        }
    }
}

void GameScreen::renderFocusFrame(game::GameHandler& gameHandler) {
    auto focus = gameHandler.getFocus();
    if (!focus) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;

    // Position: right side of screen, mirroring the target frame on the opposite side
    float frameW = 200.0f;
    float frameX = screenW - frameW - 10.0f;

    ImGui::SetNextWindowPos(ImVec2(frameX, 30.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(frameW, 0.0f), ImGuiCond_Always);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                             ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_AlwaysAutoResize;

    // Determine color based on relation (same logic as target frame)
    ImVec4 focusColor(0.7f, 0.7f, 0.7f, 1.0f);
    if (focus->getType() == game::ObjectType::PLAYER) {
        // Use class color for player focus targets
        uint8_t cid = entityClassId(focus.get());
        focusColor = (cid != 0) ? classColorVec4(cid) : kColorBrightGreen;
    } else if (focus->getType() == game::ObjectType::UNIT) {
        auto u = std::static_pointer_cast<game::Unit>(focus);
        if (u->getHealth() == 0 && u->getMaxHealth() > 0) {
            focusColor = kColorDarkGray;
        } else if (u->isHostile()) {
            // Tapped-by-other: grey focus frame name
            uint32_t focDynFlags = u->getDynamicFlags();
            bool focTapped = (focDynFlags & 0x0004) != 0 && (focDynFlags & 0x0008) == 0;
            if (focTapped) {
                focusColor = kColorGray;
            } else {
                focusColor = helpers::levelDifficultyColor(gameHandler.getPlayerLevel(),
                                                           u->getLevel());
            }
        } else {
            focusColor = kColorBrightGreen;
        }
    }

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.08f, 0.08f, 0.15f, 0.85f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.5f, 0.5f, 0.9f, 0.8f));  // Blue tint = focus

    if (ImGui::Begin("##FocusFrame", nullptr, flags)) {
        // "Focus" label
        ImGui::TextDisabled("[Focus]");
        ImGui::SameLine();

        // Raid mark icon (star, circle, diamond, …) preceding the name
        {
            uint8_t fmark = gameHandler.getEntityRaidMark(focus->getGuid());
            if (fmark < game::GameHandler::kRaidMarkCount) {
                if (VkDescriptorSet markTex = ui::getRaidTargetIcon(fmark, services_.assetManager)) {
                    constexpr float kMarkSize = 16.0f;
                    ImVec2 p = ImGui::GetCursorScreenPos();
                    ImGui::GetWindowDrawList()->AddImage(
                        (ImTextureID)(uintptr_t)markTex, p,
                        ImVec2(p.x + kMarkSize, p.y + kMarkSize));
                    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + kMarkSize + 2.0f);
                }
            }
        }

        std::string focusName = game::entityDisplayName(focus);
        ImGui::PushStyleColor(ImGuiCol_Text, focusColor);
        ImGui::PushStyleColor(ImGuiCol_Header,        ImVec4(0,0,0,0));
        ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImVec4(1,1,1,0.08f));
        ImGui::PushStyleColor(ImGuiCol_HeaderActive,  ImVec4(1,1,1,0.12f));
        ImGui::Selectable(focusName.c_str(), false, ImGuiSelectableFlags_DontClosePopups,
                          ImVec2(ImGui::CalcTextSize(focusName.c_str()).x, 0));
        ImGui::PopStyleColor(4);

        // Right-click context menu on focus frame
        if (ImGui::BeginPopupContextItem("##FocusFrameCtx")) {
            ImGui::TextDisabled("%s", focusName.c_str());
            ImGui::Separator();
            if (ImGui::MenuItem("Target"))
                gameHandler.setTarget(focus->getGuid());
            if (ImGui::MenuItem("Clear Focus"))
                gameHandler.clearFocus();
            if (focus->getType() == game::ObjectType::PLAYER) {
                ImGui::Separator();
                if (ImGui::MenuItem("Whisper")) {
                    chatPanel_.setWhisperTarget(focusName);
                }
                if (ImGui::MenuItem("Invite to Group"))
                    gameHandler.inviteToGroup(focusName);
                if (ImGui::MenuItem("Trade"))
                    gameHandler.initiateTrade(focus->getGuid());
                if (ImGui::MenuItem("Duel"))
                    gameHandler.proposeDuel(focus->getGuid());
                if (ImGui::MenuItem("Inspect")) {
                    gameHandler.setTarget(focus->getGuid());
                    gameHandler.inspectTarget();
                    socialPanel_.openInspectWindow(gameHandler);
                }
                ImGui::Separator();
                if (ImGui::MenuItem("Add Friend"))
                    gameHandler.addFriend(focusName);
                if (ImGui::MenuItem("Ignore"))
                    gameHandler.addIgnore(focusName);
            }
            ImGui::EndPopup();
        }

        // Group leader crown - golden ♛ when the focused player is the party/raid leader
        if (gameHandler.isInGroup() && focus->getType() == game::ObjectType::PLAYER) {
            if (gameHandler.getPartyData().leaderGuid == focus->getGuid()) {
                ImGui::SameLine(0, 4);
                ImGui::TextColored(colors::kSymbolGold, "\xe2\x99\x9b");
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("Group Leader");
            }
        }

        // Quest giver indicator and classification badge for NPC focus targets
        if (focus->getType() == game::ObjectType::UNIT) {
            auto focusUnit = std::static_pointer_cast<game::Unit>(focus);

            // Quest indicator: ! / ?
            {
                const auto marker = game::questGiverMarker(
                    gameHandler.getQuestGiverStatus(focus->getGuid()));
                if (marker.symbol) {
                    ImGui::SameLine(0, 4);
                    ImGui::TextColored(marker.dim ? kColorGray : colors::kBrightGold,
                                       "%s", marker.symbol);
                }
            }

            // Classification badge
            int fRank = gameHandler.getCreatureRank(focusUnit->getEntry());
            if (fRank == 1)      { ImGui::SameLine(0,4); ImGui::TextColored(ImVec4(1.0f,0.8f,0.2f,1.0f), "[Elite]"); }
            else if (fRank == 2) { ImGui::SameLine(0,4); ImGui::TextColored(ImVec4(0.8f,0.4f,1.0f,1.0f), "[Rare Elite]"); }
            else if (fRank == 3) { ImGui::SameLine(0,4); ImGui::TextColored(colors::kRed, "[Boss]"); }
            else if (fRank == 4) { ImGui::SameLine(0,4); ImGui::TextColored(ImVec4(0.5f,0.9f,1.0f,1.0f), "[Rare]"); }

            // Creature type
            {
                uint32_t fctype = gameHandler.getCreatureType(focusUnit->getEntry());
                const char* fctName = nullptr;
                switch (fctype) {
                    case 1: fctName="Beast"; break;     case 2: fctName="Dragonkin"; break;
                    case 3: fctName="Demon"; break;     case 4: fctName="Elemental"; break;
                    case 5: fctName="Giant"; break;     case 6: fctName="Undead"; break;
                    case 7: fctName="Humanoid"; break;  case 8: fctName="Critter"; break;
                    case 9: fctName="Mechanical"; break; case 11: fctName="Totem"; break;
                    case 12: fctName="Non-combat Pet"; break; case 13: fctName="Gas Cloud"; break;
                    default: break;
                }
                if (fctName) {
                    ImGui::SameLine(0, 4);
                    ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 0.9f), "(%s)", fctName);
                }
            }

            // Creature subtitle
            const std::string fSub = gameHandler.getCachedCreatureSubName(focusUnit->getEntry());
            if (!fSub.empty())
                ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 0.9f), "<%s>", fSub.c_str());
        }

        // Player guild name on focus frame
        if (focus->getType() == game::ObjectType::PLAYER) {
            uint32_t guildId = gameHandler.getEntityGuildId(focus->getGuid());
            if (guildId != 0) {
                const std::string& gn = gameHandler.lookupGuildName(guildId);
                if (!gn.empty()) {
                    ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 0.9f), "<%s>", gn.c_str());
                }
            }
        }

        if (ImGui::BeginPopupContextItem("##FocusNameCtx")) {
            const bool focusIsPlayer = (focus->getType() == game::ObjectType::PLAYER);
            const uint64_t fGuid = focus->getGuid();
            ImGui::TextDisabled("%s", focusName.c_str());
            ImGui::Separator();
            if (ImGui::MenuItem("Target"))
                gameHandler.setTarget(fGuid);
            if (ImGui::MenuItem("Clear Focus"))
                gameHandler.clearFocus();
            if (focusIsPlayer) {
                ImGui::Separator();
                if (ImGui::MenuItem("Whisper")) {
                    chatPanel_.setWhisperTarget(focusName);
                }
                if (ImGui::MenuItem("Invite to Group"))
                    gameHandler.inviteToGroup(focusName);
                if (ImGui::MenuItem("Trade"))
                    gameHandler.initiateTrade(fGuid);
                if (ImGui::MenuItem("Duel"))
                    gameHandler.proposeDuel(fGuid);
                if (ImGui::MenuItem("Inspect")) {
                    gameHandler.setTarget(fGuid);
                    gameHandler.inspectTarget();
                    socialPanel_.openInspectWindow(gameHandler);
                }
                ImGui::Separator();
                if (ImGui::MenuItem("Add Friend"))
                    gameHandler.addFriend(focusName);
                if (ImGui::MenuItem("Ignore"))
                    gameHandler.addIgnore(focusName);
            }
            ImGui::EndPopup();
        }

        if (focus->getType() == game::ObjectType::UNIT ||
            focus->getType() == game::ObjectType::PLAYER) {
            auto unit = std::static_pointer_cast<game::Unit>(focus);

            // Level + health on same row
            ImGui::SameLine();
            if (unit->getLevel() == 0)
                ImGui::TextDisabled("Lv ??");
            else
                ImGui::TextDisabled("Lv %u", unit->getLevel());

            uint32_t hp = unit->getHealth();
            uint32_t maxHp = unit->getMaxHealth();
            if (maxHp > 0) {
                float pct = static_cast<float>(hp) / static_cast<float>(maxHp);
                ImGui::PushStyleColor(ImGuiCol_PlotHistogram, colors::healthBarColor(pct));
                char overlay[32];
                snprintf(overlay, sizeof(overlay), "%u / %u", hp, maxHp);
                ImGui::ProgressBar(pct, ImVec2(-1, 14), overlay);
                ImGui::PopStyleColor();

                // Power bar
                uint8_t pType = unit->getPowerType();
                uint32_t pwr = unit->getPower();
                uint32_t maxPwr = unit->getMaxPower();
                if (maxPwr == 0 && (pType == 1 || pType == 3)) maxPwr = 100;
                if (maxPwr > 0) {
                    float mpPct = static_cast<float>(pwr) / static_cast<float>(maxPwr);
                    ImVec4 pwrColor;
                    pwrColor = colors::powerTypeColor(static_cast<uint8_t>(pType));
                    ImGui::PushStyleColor(ImGuiCol_PlotHistogram, pwrColor);
                    ImGui::ProgressBar(mpPct, ImVec2(-1, 10), "");
                    ImGui::PopStyleColor();
                }
            }

            // Focus cast bar
            const auto* focusCast = gameHandler.getUnitCastState(focus->getGuid());
            if (focusCast) {
                float total = focusCast->timeTotal > 0.f ? focusCast->timeTotal : 1.f;
                float rem   = focusCast->timeRemaining;
                float prog  = std::clamp(1.0f - rem / total, 0.f, 1.f);
                const std::string& spName = gameHandler.getSpellName(focusCast->spellId);
                // Pulse orange when > 80% complete - interrupt window closing
                ImVec4 focusCastColor;
                if (prog > 0.8f) {
                    float pulse = 0.7f + 0.3f * std::sin(static_cast<float>(ImGui::GetTime()) * 8.0f);
                    focusCastColor = ImVec4(1.0f * pulse, 0.5f * pulse, 0.0f, 1.0f);
                } else {
                    focusCastColor = ImVec4(0.9f, 0.3f, 0.2f, 1.0f);
                }
                ImGui::PushStyleColor(ImGuiCol_PlotHistogram, focusCastColor);
                char castBuf[64];
                if (!spName.empty())
                    snprintf(castBuf, sizeof(castBuf), "%s (%.1fs)", spName.c_str(), rem);
                else
                    snprintf(castBuf, sizeof(castBuf), "Casting... (%.1fs)", rem);
                {
                    auto* fcAsset = services_.assetManager;
                    VkDescriptorSet fcIcon = (focusCast->spellId != 0 && fcAsset)
                        ? getSpellIcon(focusCast->spellId, fcAsset) : VK_NULL_HANDLE;
                    if (fcIcon) {
                        ImGui::Image((ImTextureID)(uintptr_t)fcIcon, ImVec2(12, 12));
                        ImGui::SameLine(0, 2);
                        ImGui::ProgressBar(prog, ImVec2(-1, 12), castBuf);
                    } else {
                        ImGui::ProgressBar(prog, ImVec2(-1, 12), castBuf);
                    }
                }
                ImGui::PopStyleColor();
            }
        }

        // Focus auras - buffs first, then debuffs, up to 8 icons wide
        {
            const std::vector<game::AuraSlot>* focusAuras =
                (focus->getGuid() == gameHandler.getTargetGuid())
                    ? &gameHandler.getTargetAuras()
                    : gameHandler.getUnitAuras(focus->getGuid());

            if (focusAuras) {
                int activeCount = 0;
                for (const auto& a : *focusAuras) if (!a.isEmpty()) activeCount++;
                if (activeCount > 0) {
                    auto* focusAsset = services_.assetManager;
                    constexpr float FA_ICON = 20.0f;
                    constexpr int   FA_PER_ROW = 10;

                    ImGui::Separator();

                    uint64_t faNowMs = static_cast<uint64_t>(
                        std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now().time_since_epoch()).count());

                    // Sort: debuffs first (so hostile-caster info is prominent), then buffs
                    std::vector<size_t> faIdx;
                    faIdx.reserve(focusAuras->size());
                    for (size_t i = 0; i < focusAuras->size(); ++i)
                        if (!(*focusAuras)[i].isEmpty()) faIdx.push_back(i);
                    std::sort(faIdx.begin(), faIdx.end(), [&](size_t a, size_t b) {
                        bool aD = ((*focusAuras)[a].flags & 0x80) != 0;
                        bool bD = ((*focusAuras)[b].flags & 0x80) != 0;
                        if (aD != bD) return aD > bD; // debuffs first
                        int32_t ra = (*focusAuras)[a].getRemainingMs(faNowMs);
                        int32_t rb = (*focusAuras)[b].getRemainingMs(faNowMs);
                        if (ra < 0 && rb < 0) return false;
                        if (ra < 0) return false;
                        if (rb < 0) return true;
                        return ra < rb;
                    });

                    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(2.0f, 2.0f));
                    int faShown = 0;
                    for (size_t si = 0; si < faIdx.size() && faShown < 20; ++si) {
                        const auto& aura = (*focusAuras)[faIdx[si]];
                        bool isBuff = (aura.flags & 0x80) == 0;

                        if (faShown > 0 && faShown % FA_PER_ROW != 0) ImGui::SameLine();
                        ImGui::PushID(static_cast<int>(faIdx[si]) + 3000);

                        const ImVec4 borderCol =
                            wowee::ui::auraBorderColor(isBuff, gameHandler.getSpellDispelType(aura.spellId));

                        VkDescriptorSet faIcon = (focusAsset)
                            ? getSpellIcon(aura.spellId, focusAsset) : VK_NULL_HANDLE;
                        if (faIcon) {
                            ImGui::PushStyleColor(ImGuiCol_Button, borderCol);
                            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(1, 1));
                            ImGui::ImageButton("##faura",
                                (ImTextureID)(uintptr_t)faIcon,
                                ImVec2(FA_ICON - 2, FA_ICON - 2));
                            ImGui::PopStyleVar();
                            ImGui::PopStyleColor();
                        } else {
                            ImGui::PushStyleColor(ImGuiCol_Button, borderCol);
                            char lab[8];
                            snprintf(lab, sizeof(lab), "%u", aura.spellId);
                            ImGui::Button(lab, ImVec2(FA_ICON, FA_ICON));
                            ImGui::PopStyleColor();
                        }

                        // Duration overlay
                        int32_t faRemain = aura.getRemainingMs(faNowMs);
                        if (faRemain > 0) {
                            ImVec2 imin = ImGui::GetItemRectMin();
                            ImVec2 imax = ImGui::GetItemRectMax();
                            char ts[12];
                            fmtDurationCompact(ts, sizeof(ts), (faRemain + 999) / 1000);
                            ImVec2 tsz = ImGui::CalcTextSize(ts);
                            float cx = imin.x + (imax.x - imin.x - tsz.x) * 0.5f;
                            float cy = imax.y - tsz.y - 1.0f;
                            ImGui::GetWindowDrawList()->AddText(ImVec2(cx + 1, cy + 1), IM_COL32(0, 0, 0, 180), ts);
                            ImGui::GetWindowDrawList()->AddText(ImVec2(cx, cy), IM_COL32(255, 255, 255, 220), ts);
                        }

                        // Stack / charge count - upper-left corner (parity with target frame)
                        if (aura.charges > 1) {
                            ImVec2 faMin = ImGui::GetItemRectMin();
                            char chargeStr[8];
                            snprintf(chargeStr, sizeof(chargeStr), "%u", static_cast<unsigned>(aura.charges));
                            ImGui::GetWindowDrawList()->AddText(ImVec2(faMin.x + 3, faMin.y + 3),
                                IM_COL32(0, 0, 0, 200), chargeStr);
                            ImGui::GetWindowDrawList()->AddText(ImVec2(faMin.x + 2, faMin.y + 2),
                                IM_COL32(255, 220, 50, 255), chargeStr);
                        }

                        // Tooltip
                        if (ImGui::IsItemHovered()) {
                            ImGui::BeginTooltip();
                            bool richOk = spellbookScreen.renderSpellInfoTooltip(
                                aura.spellId, gameHandler, focusAsset);
                            if (!richOk) {
                                std::string nm = spellbookScreen.lookupSpellName(aura.spellId, focusAsset);
                                if (nm.empty()) nm = "Spell #" + std::to_string(aura.spellId);
                                ImGui::Text("%s", nm.c_str());
                            }
                            renderAuraRemaining(faRemain);
                            ImGui::EndTooltip();
                        }

                        ImGui::PopID();
                        faShown++;
                    }
                    ImGui::PopStyleVar();
                }
            }
        }

        // Target-of-Focus: who the focus target is currently targeting
        {
            const uint64_t fofGuid = game::unitTargetGuid(focus);
            if (fofGuid != 0) {
                auto fofEnt = gameHandler.getEntityManager().getEntity(fofGuid);
                std::string fofName;
                ImVec4 fofColor(0.7f, 0.7f, 0.7f, 1.0f);
                if (fofGuid == gameHandler.getPlayerGuid()) {
                    fofName = "You";
                    fofColor = kColorBrightGreen;
                } else if (fofEnt) {
                    fofName = game::entityDisplayName(fofEnt);
                    uint8_t fcid = entityClassId(fofEnt.get());
                    if (fcid != 0) fofColor = classColorVec4(fcid);
                }
                if (!fofName.empty()) {
                    ImGui::TextDisabled("▶");
                    ImGui::SameLine(0, 2);
                    ImGui::TextColored(fofColor, "%s", fofName.c_str());
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Focus's target: %s\nClick to target", fofName.c_str());
                    if (ImGui::IsItemClicked())
                        gameHandler.setTarget(fofGuid);

                    // Compact health bar for target-of-focus
                    if (fofEnt) {
                        auto fofUnit = std::dynamic_pointer_cast<game::Unit>(fofEnt);
                        if (fofUnit && fofUnit->getMaxHealth() > 0) {
                            float fofPct = static_cast<float>(fofUnit->getHealth()) /
                                           static_cast<float>(fofUnit->getMaxHealth());
                            ImGui::PushStyleColor(ImGuiCol_PlotHistogram,
                                                  colors::healthBarColor(fofPct));
                            ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.15f, 0.15f, 0.15f, 0.8f));
                            char fofOverlay[32];
                            snprintf(fofOverlay, sizeof(fofOverlay), "%u%%",
                                     static_cast<unsigned>(std::lround(fofPct * 100.0f)));
                            ImGui::ProgressBar(fofPct, ImVec2(-1, 10), fofOverlay);
                            ImGui::PopStyleColor(2);
                        }
                    }
                }
            }
        }

        // Distance to focus target
        {
            const auto& mv = gameHandler.getMovementInfo();
            float fdx = focus->getX() - mv.x;
            float fdy = focus->getY() - mv.y;
            float fdz = focus->getZ() - mv.z;
            float fdist = std::sqrt(fdx * fdx + fdy * fdy + fdz * fdz);
            ImGui::TextDisabled("%.1f yd", fdist);
        }

        // Clicking the focus frame targets it
        if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(0)) {
            gameHandler.setTarget(focus->getGuid());
        }
    }
    ImGui::End();

    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar();
}


}} // namespace wowee::ui
