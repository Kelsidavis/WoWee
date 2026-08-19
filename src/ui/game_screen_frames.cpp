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
                        const std::string& nm = gameHandler.getSpellName(actionId);
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
