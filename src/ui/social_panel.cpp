// ============================================================
// SocialPanel — extracted from GameScreen
// Owns all social/group-related UI rendering: party frames,
// boss frames, guild roster, social/friends frame, dungeon finder,
// who window, inspect window.
// ============================================================
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
    constexpr auto& kColorDarkGray    = kDarkGray;
} // anonymous namespace

namespace wowee {
namespace ui {


void SocialPanel::renderPartyFrames(game::GameHandler& gameHandler,
                                       ChatPanel& chatPanel,
                                       SpellIconFn getSpellIcon) {
    auto* assetMgr = services_.assetManager;
    const auto& partyData = gameHandler.getPartyData();

    if (!gameHandler.isInGroup()) return;

    if (partyData.members.empty()) {
        ImGui::SetNextWindowPos(ImVec2(10.0f, 120.0f), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(180.0f, 0.0f), ImGuiCond_Always);

        ImGuiWindowFlags emptyFlags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                                      ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                                      ImGuiWindowFlags_AlwaysAutoResize;
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.1f, 0.1f, 0.65f));

        if (ImGui::Begin("##PartyFramesEmpty", nullptr, emptyFlags))
            ImGui::TextDisabled("Party: waiting for roster");
        ImGui::End();
        ImGui::PopStyleColor();
        ImGui::PopStyleVar();
        return;
    }

    const bool isRaid = (partyData.groupType == 1);
    float frameY = 120.0f;

    // ---- Raid frame layout ----
    if (isRaid) {
        // Organize members by subgroup (0-7, up to 5 members each)
        constexpr int MAX_SUBGROUPS = 8;
        constexpr int MAX_PER_GROUP = 5;
        std::vector<const game::GroupMember*> subgroups[MAX_SUBGROUPS];
        for (const auto& m : partyData.members) {
            int sg = m.subGroup < MAX_SUBGROUPS ? m.subGroup : 0;
            if (static_cast<int>(subgroups[sg].size()) < MAX_PER_GROUP)
                subgroups[sg].push_back(&m);
        }

        // Count non-empty subgroups to determine layout
        int activeSgs = 0;
        for (int sg = 0; sg < MAX_SUBGROUPS; sg++)
            if (!subgroups[sg].empty()) activeSgs++;

        // Compact raid cell: name + 2 narrow bars
        constexpr float CELL_W = 90.0f;
        constexpr float CELL_H = 42.0f;
        constexpr float BAR_H  = 7.0f;
        constexpr float CELL_PAD = 3.0f;

        float winW = activeSgs * (CELL_W + CELL_PAD) + CELL_PAD + 8.0f;
        float winH = MAX_PER_GROUP * (CELL_H + CELL_PAD) + CELL_PAD + 20.0f;

        auto* window = services_.window;
        float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
        float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;
        float raidX = (screenW - winW) / 2.0f;
        float raidY = screenH - winH - 120.0f;  // above action bar area

        ImGui::SetNextWindowPos(ImVec2(raidX, raidY), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(winW, winH), ImGuiCond_Always);

        ImGuiWindowFlags raidFlags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                                     ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                                     ImGuiWindowFlags_NoScrollbar;
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(CELL_PAD, CELL_PAD));
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.07f, 0.07f, 0.1f, 0.85f));

        if (ImGui::Begin("##RaidFrames", nullptr, raidFlags)) {
            ImDrawList* draw = ImGui::GetWindowDrawList();
            ImVec2 winPos = ImGui::GetWindowPos();

            int colIdx = 0;
            for (int sg = 0; sg < MAX_SUBGROUPS; sg++) {
                if (subgroups[sg].empty()) continue;

                float colX = winPos.x + CELL_PAD + colIdx * (CELL_W + CELL_PAD);

                for (int row = 0; row < static_cast<int>(subgroups[sg].size()); row++) {
                    const auto& m = *subgroups[sg][row];
                    float cellY = winPos.y + CELL_PAD + 14.0f + row * (CELL_H + CELL_PAD);

                    ImVec2 cellMin(colX, cellY);
                    ImVec2 cellMax(colX + CELL_W, cellY + CELL_H);

                    // Cell background
                    bool isTarget = (gameHandler.getTargetGuid() == m.guid);
                    ImU32 bg = isTarget ? IM_COL32(60, 80, 120, 200) : IM_COL32(30, 30, 40, 180);
                    draw->AddRectFilled(cellMin, cellMax, bg, 3.0f);
                    if (isTarget)
                        draw->AddRect(cellMin, cellMax, IM_COL32(100, 150, 255, 200), 3.0f);

                    // Dead/ghost overlay
                    bool isOnline = (m.onlineStatus & 0x0001) != 0;
                    bool isDead   = (m.onlineStatus & 0x0020) != 0;
                    bool isGhost  = (m.onlineStatus & 0x0010) != 0;

                    // Out-of-range check (40 yard threshold)
                    bool isOOR = false;
                    if (m.hasPartyStats && isOnline && !isDead && !isGhost && m.zoneId != 0) {
                        auto playerEnt = gameHandler.getEntityManager().getEntity(gameHandler.getPlayerGuid());
                        if (playerEnt) {
                            float dx = playerEnt->getX() - static_cast<float>(m.posX);
                            float dy = playerEnt->getY() - static_cast<float>(m.posY);
                            isOOR = (dx * dx + dy * dy) > (40.0f * 40.0f);
                        }
                    }
                    // Dim cell overlay when out of range
                    if (isOOR)
                        draw->AddRectFilled(cellMin, cellMax, IM_COL32(0, 0, 0, 80), 3.0f);

                    // Name text (truncated) — class color when alive+online, gray when dead/offline
                    char truncName[16];
                    snprintf(truncName, sizeof(truncName), "%.12s", m.name.c_str());
                    bool isMemberLeader = (m.guid == partyData.leaderGuid);
                    ImU32 nameCol;
                    if (!isOnline || isDead || isGhost) {
                        nameCol = IM_COL32(140, 140, 140, 200); // gray for dead/offline
                    } else {
                        // Default: gold for leader, light gray for others
                        nameCol = isMemberLeader ? IM_COL32(255, 215, 0, 255) : IM_COL32(220, 220, 220, 255);
                        // Override with WoW class color if entity is loaded
                        auto mEnt = gameHandler.getEntityManager().getEntity(m.guid);
                        uint8_t cid = entityClassId(mEnt.get());
                        if (cid != 0) nameCol = classColorU32(cid);
                    }
                    draw->AddText(ImVec2(cellMin.x + 4.0f, cellMin.y + 3.0f), nameCol, truncName);

                    // Leader crown star in top-right of cell
                    if (isMemberLeader)
                        draw->AddText(ImVec2(cellMax.x - 10.0f, cellMin.y + 2.0f), IM_COL32(255, 215, 0, 255), "*");

                    // Raid mark symbol — small, just to the left of the leader crown
                    {
                        uint8_t rmk = gameHandler.getEntityRaidMark(m.guid);
                        if (rmk < game::GameHandler::kRaidMarkCount) {
                            VkDescriptorSet markTex = ui::getRaidTargetIcon(rmk, services_.assetManager);
                            if (markTex) {
                            constexpr float kMarkSize = 11.0f;
                            float rmX = cellMax.x - 10.0f - 2.0f - kMarkSize;
                            draw->AddImage((ImTextureID)(uintptr_t)markTex,
                                ImVec2(rmX, cellMin.y + 2.0f),
                                ImVec2(rmX + kMarkSize, cellMin.y + 2.0f + kMarkSize));
                            }
                        }
                    }

                    // LFG role badge in bottom-right corner of cell
                    if (m.roles & 0x02)
                        draw->AddText(ImVec2(cellMax.x - 11.0f, cellMax.y - 11.0f), IM_COL32(80, 130, 255, 230), "T");
                    else if (m.roles & 0x04)
                        draw->AddText(ImVec2(cellMax.x - 11.0f, cellMax.y - 11.0f), IM_COL32(60, 220, 80, 230), "H");
                    else if (m.roles & 0x08)
                        draw->AddText(ImVec2(cellMax.x - 11.0f, cellMax.y - 11.0f), IM_COL32(220, 80, 80, 230), "D");

                    // Tactical role badge in bottom-left corner (flags from SMSG_GROUP_LIST / SMSG_REAL_GROUP_UPDATE)
                    // 0x01=Assistant, 0x02=Main Tank, 0x04=Main Assist
                    if (m.flags & 0x02)
                        draw->AddText(ImVec2(cellMin.x + 2.0f, cellMax.y - 11.0f), IM_COL32(255, 140, 0, 230), "MT");
                    else if (m.flags & 0x04)
                        draw->AddText(ImVec2(cellMin.x + 2.0f, cellMax.y - 11.0f), IM_COL32(100, 180, 255, 230), "MA");
                    else if (m.flags & 0x01)
                        draw->AddText(ImVec2(cellMin.x + 2.0f, cellMax.y - 11.0f), IM_COL32(180, 215, 255, 180), "A");

                    // Health bar
                    uint32_t hp = m.hasPartyStats ? m.curHealth : 0;
                    uint32_t maxHp = m.hasPartyStats ? m.maxHealth : 0;
                    if (maxHp > 0) {
                        float pct = static_cast<float>(hp) / static_cast<float>(maxHp);
                        float barY = cellMin.y + 16.0f;
                        ImVec2 barBg(cellMin.x + 3.0f, barY);
                        ImVec2 barBgEnd(cellMax.x - 3.0f, barY + BAR_H);
                        draw->AddRectFilled(barBg, barBgEnd, IM_COL32(40, 40, 40, 200), 2.0f);
                        ImVec2 barFill(barBg.x, barBg.y);
                        ImVec2 barFillEnd(barBg.x + (barBgEnd.x - barBg.x) * pct, barBgEnd.y);
                        ImU32 hpCol = isOOR ? IM_COL32(100, 100, 100, 160) :
                                     pct > 0.5f ? IM_COL32(60, 180, 60, 255) :
                                     pct > 0.2f ? IM_COL32(200, 180, 50, 255) :
                                                  IM_COL32(200, 60, 60, 255);
                        draw->AddRectFilled(barFill, barFillEnd, hpCol, 2.0f);
                        // HP percentage or OOR text centered on bar
                        char hpPct[8];
                        if (isOOR)
                            snprintf(hpPct, sizeof(hpPct), "OOR");
                        else
                            snprintf(hpPct, sizeof(hpPct), "%d%%", static_cast<int>(pct * 100.0f + 0.5f));
                        ImVec2 ts = ImGui::CalcTextSize(hpPct);
                        float tx = (barBg.x + barBgEnd.x - ts.x) * 0.5f;
                        float ty = barBg.y + (BAR_H - ts.y) * 0.5f;
                        draw->AddText(ImVec2(tx + 1.0f, ty + 1.0f), IM_COL32(0, 0, 0, 180), hpPct);
                        draw->AddText(ImVec2(tx, ty), IM_COL32(255, 255, 255, 230), hpPct);
                    }

                    // Power bar
                    if (m.hasPartyStats && m.maxPower > 0) {
                        float pct = static_cast<float>(m.curPower) / static_cast<float>(m.maxPower);
                        float barY = cellMin.y + 16.0f + BAR_H + 2.0f;
                        ImVec2 barBg(cellMin.x + 3.0f, barY);
                        ImVec2 barBgEnd(cellMax.x - 3.0f, barY + BAR_H - 2.0f);
                        draw->AddRectFilled(barBg, barBgEnd, IM_COL32(30, 30, 40, 200), 2.0f);
                        ImVec2 barFill(barBg.x, barBg.y);
                        ImVec2 barFillEnd(barBg.x + (barBgEnd.x - barBg.x) * pct, barBgEnd.y);
                        ImU32 pwrCol;
                        switch (m.powerType) {
                            case 0: pwrCol = IM_COL32(50, 80, 220, 255); break; // Mana
                            case 1: pwrCol = IM_COL32(200, 50, 50, 255); break; // Rage
                            case 3: pwrCol = IM_COL32(220, 210, 50, 255); break; // Energy
                            case 6: pwrCol = IM_COL32(180, 30, 50, 255); break; // Runic Power
                            default: pwrCol = IM_COL32(80, 120, 80, 255); break;
                        }
                        draw->AddRectFilled(barFill, barFillEnd, pwrCol, 2.0f);
                    }

                    // Dispellable debuff dots at the bottom of the raid cell
                    // Mirrors party frame debuff indicators for healers in 25/40-man raids
                    if (!isDead && !isGhost) {
                        const std::vector<game::AuraSlot>* unitAuras = nullptr;
                        if (m.guid == gameHandler.getPlayerGuid())
                            unitAuras = &gameHandler.getPlayerAuras();
                        else if (m.guid == gameHandler.getTargetGuid())
                            unitAuras = &gameHandler.getTargetAuras();
                        else
                            unitAuras = gameHandler.getUnitAuras(m.guid);

                        if (unitAuras) {
                            bool shown[5] = {};
                            float dotX = cellMin.x + 4.0f;
                            const float dotY  = cellMax.y - 5.0f;
                            const float DOT_R = 3.5f;
                            ImVec2 mouse = ImGui::GetMousePos();
                            for (const auto& aura : *unitAuras) {
                                if (aura.isEmpty()) continue;
                                if ((aura.flags & 0x80) == 0) continue; // debuffs only
                                uint8_t dt = gameHandler.getSpellDispelType(aura.spellId);
                                if (dt == 0 || dt > 4 || shown[dt]) continue;
                                shown[dt] = true;
                                ImVec4 dc;
                                switch (dt) {
                                    case 1: dc = ImVec4(0.25f, 0.50f, 1.00f, 0.90f); break; // Magic: blue
                                    case 2: dc = ImVec4(0.70f, 0.15f, 0.90f, 0.90f); break; // Curse: purple
                                    case 3: dc = ImVec4(0.65f, 0.45f, 0.10f, 0.90f); break; // Disease: brown
                                    case 4: dc = ImVec4(0.10f, 0.75f, 0.10f, 0.90f); break; // Poison: green
                                    default: continue;
                                }
                                ImU32 dotColU = ImGui::ColorConvertFloat4ToU32(dc);
                                draw->AddCircleFilled(ImVec2(dotX, dotY), DOT_R, dotColU);
                                draw->AddCircle(ImVec2(dotX, dotY), DOT_R + 0.5f, IM_COL32(0, 0, 0, 160), 8, 1.0f);

                                float mdx = mouse.x - dotX, mdy = mouse.y - dotY;
                                if (mdx * mdx + mdy * mdy < (DOT_R + 4.0f) * (DOT_R + 4.0f)) {
                                    ImGui::BeginTooltip();
                                    ImGui::TextColored(dc, "%s", kDispelNames[dt]);
                                    for (const auto& da : *unitAuras) {
                                        if (da.isEmpty() || (da.flags & 0x80) == 0) continue;
                                        if (gameHandler.getSpellDispelType(da.spellId) != dt) continue;
                                        const std::string& dName = gameHandler.getSpellName(da.spellId);
                                        if (!dName.empty())
                                            ImGui::Text("  %s", dName.c_str());
                                    }
                                    ImGui::EndTooltip();
                                }
                                dotX += 9.0f;
                            }
                        }
                    }

                    // Clickable invisible region over the whole cell
                    ImGui::SetCursorScreenPos(cellMin);
                    ImGui::PushID(static_cast<int>(m.guid));
                    if (ImGui::InvisibleButton("raidCell", ImVec2(CELL_W, CELL_H))) {
                        gameHandler.setTarget(m.guid);
                    }
                    if (ImGui::IsItemHovered()) {
                        gameHandler.setMouseoverGuid(m.guid);
                    }
                    if (ImGui::BeginPopupContextItem("RaidMemberCtx")) {
                        ImGui::TextDisabled("%s", m.name.c_str());
                        ImGui::Separator();
                        if (ImGui::MenuItem("Target"))
                            gameHandler.setTarget(m.guid);
                        if (ImGui::MenuItem("Set Focus"))
                            gameHandler.setFocus(m.guid);
                        if (ImGui::MenuItem("Whisper")) {
                            chatPanel.setWhisperTarget(m.name);
                        }
                        if (ImGui::MenuItem("Trade"))
                            gameHandler.initiateTrade(m.guid);
                        if (ImGui::MenuItem("Inspect")) {
                            gameHandler.setTarget(m.guid);
                            gameHandler.inspectTarget();
                            showInspectWindow_ = true;
                        }
                        bool isLeader = (partyData.leaderGuid == gameHandler.getPlayerGuid());
                        if (isLeader) {
                            ImGui::Separator();
                            if (ImGui::MenuItem("Kick from Raid"))
                                gameHandler.uninvitePlayer(m.name);
                        }
                        ImGui::Separator();
                        if (ImGui::BeginMenu("Set Raid Mark")) {
                            for (int mi = 0; mi < 8; ++mi) {
                                if (ImGui::MenuItem(kRaidMarkNames[mi]))
                                    gameHandler.setRaidMark(m.guid, static_cast<uint8_t>(mi));
                            }
                            ImGui::Separator();
                            if (ImGui::MenuItem("Clear Mark"))
                                gameHandler.setRaidMark(m.guid, 0xFF);
                            ImGui::EndMenu();
                        }
                        ImGui::EndPopup();
                    }
                    ImGui::PopID();
                }
                colIdx++;
            }

            // Subgroup header row
            colIdx = 0;
            for (int sg = 0; sg < MAX_SUBGROUPS; sg++) {
                if (subgroups[sg].empty()) continue;
                float colX = winPos.x + CELL_PAD + colIdx * (CELL_W + CELL_PAD);
                char sgLabel[8];
                snprintf(sgLabel, sizeof(sgLabel), "G%d", sg + 1);
                draw->AddText(ImVec2(colX + CELL_W / 2 - 8.0f, winPos.y + CELL_PAD), IM_COL32(160, 160, 180, 200), sgLabel);
                colIdx++;
            }
        }
        ImGui::End();
        ImGui::PopStyleColor();
        ImGui::PopStyleVar(2);
        return;
    }

    // ---- Party frame layout (5-man) ----
    ImGui::SetNextWindowPos(ImVec2(10.0f, frameY), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(200.0f, 0.0f), ImGuiCond_Always);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                             ImGuiWindowFlags_AlwaysAutoResize;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.1f, 0.1f, 0.8f));

    if (ImGui::Begin("##PartyFrames", nullptr, flags)) {
        const uint64_t leaderGuid = partyData.leaderGuid;
        for (const auto& member : partyData.members) {
            ImGui::PushID(static_cast<int>(member.guid));

            bool isLeader = (member.guid == leaderGuid);

            // Name with level and status info — leader gets a gold star prefix
            std::string label = (isLeader ? "* " : "  ") + member.name;
            if (member.hasPartyStats && member.level > 0) {
                label += " [" + std::to_string(member.level) + "]";
            }
            if (member.hasPartyStats) {
                bool isOnline = (member.onlineStatus & 0x0001) != 0;
                bool isDead = (member.onlineStatus & 0x0020) != 0;
                bool isGhost = (member.onlineStatus & 0x0010) != 0;
                if (!isOnline) label += " (offline)";
                else if (isDead || isGhost) label += " (dead)";
            }

            // Clickable name to target — use WoW class colors when entity is loaded,
            // fall back to gold for leader / light gray for others
            ImVec4 nameColor = isLeader
                ? colors::kBrightGold
                : colors::kVeryLightGray;
            {
                auto memberEntity = gameHandler.getEntityManager().getEntity(member.guid);
                uint8_t cid = entityClassId(memberEntity.get());
                if (cid != 0) nameColor = classColorVec4(cid);
            }
            ImGui::PushStyleColor(ImGuiCol_Text, nameColor);
            if (ImGui::Selectable(label.c_str(), gameHandler.getTargetGuid() == member.guid)) {
                gameHandler.setTarget(member.guid);
            }
            // Set mouseover for [target=mouseover] macro conditionals
            if (ImGui::IsItemHovered()) {
                gameHandler.setMouseoverGuid(member.guid);
            }
            // Zone tooltip on name hover
            if (ImGui::IsItemHovered() && member.hasPartyStats && member.zoneId != 0) {
                std::string zoneName = gameHandler.getWhoAreaName(member.zoneId);
                if (!zoneName.empty())
                    ImGui::SetTooltip("%s", zoneName.c_str());
            }
            ImGui::PopStyleColor();

            // LFG role badge (Tank/Healer/DPS) — shown on same line as name when set
            if (member.roles != 0) {
                ImGui::SameLine();
                if (member.roles & 0x02) ImGui::TextColored(ImVec4(0.3f, 0.5f, 1.0f, 1.0f), "[T]");
                if (member.roles & 0x04) { ImGui::SameLine(); ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.3f, 1.0f), "[H]"); }
                if (member.roles & 0x08) { ImGui::SameLine(); ImGui::TextColored(ImVec4(0.9f, 0.3f, 0.3f, 1.0f), "[D]"); }
            }

            // Tactical role badge (MT/MA/Asst) from group flags
            if (member.flags & 0x02) {
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.0f, 0.9f), "[MT]");
            } else if (member.flags & 0x04) {
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(0.4f, 0.7f, 1.0f, 0.9f), "[MA]");
            } else if (member.flags & 0x01) {
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(0.7f, 0.85f, 1.0f, 0.7f), "[A]");
            }

            // Raid mark symbol — shown on same line as name when this party member has a mark
            {
                uint8_t pmk = gameHandler.getEntityRaidMark(member.guid);
                if (pmk < game::GameHandler::kRaidMarkCount) {
                    if (VkDescriptorSet markTex = ui::getRaidTargetIcon(pmk, services_.assetManager)) {
                        ImGui::SameLine();
                        ImGui::Image((ImTextureID)(uintptr_t)markTex, ImVec2(14.0f, 14.0f));
                    }
                }
            }

            // Health bar: prefer party stats, fall back to entity
            uint32_t hp = 0, maxHp = 0;
            if (member.hasPartyStats && member.maxHealth > 0) {
                hp = member.curHealth;
                maxHp = member.maxHealth;
            } else {
                auto entity = gameHandler.getEntityManager().getEntity(member.guid);
                if (entity && (entity->getType() == game::ObjectType::PLAYER || entity->getType() == game::ObjectType::UNIT)) {
                    auto unit = std::static_pointer_cast<game::Unit>(entity);
                    hp = unit->getHealth();
                    maxHp = unit->getMaxHealth();
                }
            }
            // Check dead/ghost state for health bar rendering
            bool memberDead = false;
            bool memberOffline = false;
            if (member.hasPartyStats) {
                bool isOnline2 = (member.onlineStatus & 0x0001) != 0;
                bool isDead2   = (member.onlineStatus & 0x0020) != 0;
                bool isGhost2  = (member.onlineStatus & 0x0010) != 0;
                memberDead    = isDead2 || isGhost2;
                memberOffline = !isOnline2;
            }

            // Out-of-range check: compare player position to member's reported position
            // Range threshold: 40 yards (standard heal/spell range)
            bool memberOutOfRange = false;
            if (member.hasPartyStats && !memberOffline && !memberDead &&
                member.zoneId != 0) {
                // Same map: use 2D Euclidean distance in WoW coordinates (yards)
                auto playerEntity = gameHandler.getEntityManager().getEntity(gameHandler.getPlayerGuid());
                if (playerEntity) {
                    float dx = playerEntity->getX() - static_cast<float>(member.posX);
                    float dy = playerEntity->getY() - static_cast<float>(member.posY);
                    float distSq = dx * dx + dy * dy;
                    memberOutOfRange = (distSq > 40.0f * 40.0f);
                }
            }

            if (memberDead) {
                // Gray "Dead" bar for fallen party members
                ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.35f, 0.35f, 0.35f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_FrameBg,       ImVec4(0.15f, 0.15f, 0.15f, 1.0f));
                ImGui::ProgressBar(0.0f, ImVec2(-1, 14), "Dead");
                ImGui::PopStyleColor(2);
            } else if (memberOffline) {
                // Dim bar for offline members
                ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.25f, 0.25f, 0.25f, 0.6f));
                ImGui::PushStyleColor(ImGuiCol_FrameBg,       ImVec4(0.1f, 0.1f, 0.1f, 0.6f));
                ImGui::ProgressBar(0.0f, ImVec2(-1, 14), "Offline");
                ImGui::PopStyleColor(2);
            } else if (maxHp > 0) {
                float pct = static_cast<float>(hp) / static_cast<float>(maxHp);
                // Out-of-range: desaturate health bar to gray
                ImVec4 hpBarColor = memberOutOfRange
                    ? ImVec4(0.45f, 0.45f, 0.45f, 0.7f)
                    : (pct > 0.5f ? colors::kHealthGreen :
                       pct > 0.2f ? colors::kMidHealthYellow :
                                    colors::kLowHealthRed);
                ImGui::PushStyleColor(ImGuiCol_PlotHistogram, hpBarColor);
                char hpText[32];
                if (memberOutOfRange) {
                    snprintf(hpText, sizeof(hpText), "OOR");
                } else if (maxHp >= 10000) {
                    snprintf(hpText, sizeof(hpText), "%dk/%dk",
                             static_cast<int>(hp) / 1000, static_cast<int>(maxHp) / 1000);
                } else {
                    snprintf(hpText, sizeof(hpText), "%u/%u", hp, maxHp);
                }
                ImGui::ProgressBar(pct, ImVec2(-1, 14), hpText);
                ImGui::PopStyleColor();
            }

            // Power bar (mana/rage/energy) from party stats — hidden for dead/offline/OOR
            if (!memberDead && !memberOffline && member.hasPartyStats && member.maxPower > 0) {
                float powerPct = static_cast<float>(member.curPower) / static_cast<float>(member.maxPower);
                ImVec4 powerColor;
                switch (member.powerType) {
                    case 0: powerColor = colors::kManaBlue; break; // Mana (blue)
                    case 1: powerColor = colors::kDarkRed; break; // Rage (red)
                    case 2: powerColor = colors::kOrange; break; // Focus (orange)
                    case 3: powerColor = colors::kEnergyYellow; break; // Energy (yellow)
                    case 4: powerColor = colors::kHappinessGreen; break; // Happiness (green)
                    case 6: powerColor = colors::kRunicRed; break; // Runic Power (crimson)
                    case 7: powerColor = colors::kSoulShardPurple; break; // Soul Shards (purple)
                    default: powerColor = kColorDarkGray; break;
                }
                ImGui::PushStyleColor(ImGuiCol_PlotHistogram, powerColor);
                ImGui::ProgressBar(powerPct, ImVec2(-1, 8), "");
                ImGui::PopStyleColor();
            }

            // Dispellable debuff indicators — small colored dots for party member debuffs
            // Only show magic/curse/disease/poison (types 1-4); skip non-dispellable
            if (!memberDead && !memberOffline) {
                const std::vector<game::AuraSlot>* unitAuras = nullptr;
                if (member.guid == gameHandler.getPlayerGuid())
                    unitAuras = &gameHandler.getPlayerAuras();
                else if (member.guid == gameHandler.getTargetGuid())
                    unitAuras = &gameHandler.getTargetAuras();
                else
                    unitAuras = gameHandler.getUnitAuras(member.guid);

                if (unitAuras) {
                    bool anyDebuff = false;
                    for (const auto& aura : *unitAuras) {
                        if (aura.isEmpty()) continue;
                        if ((aura.flags & 0x80) == 0) continue; // only debuffs
                        uint8_t dt = gameHandler.getSpellDispelType(aura.spellId);
                        if (dt == 0) continue; // skip non-dispellable
                        anyDebuff = true;
                        break;
                    }
                    if (anyDebuff) {
                        // Render one dot per unique dispel type present
                        bool shown[5] = {};
                        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(2.0f, 1.0f));
                        for (const auto& aura : *unitAuras) {
                            if (aura.isEmpty()) continue;
                            if ((aura.flags & 0x80) == 0) continue;
                            uint8_t dt = gameHandler.getSpellDispelType(aura.spellId);
                            if (dt == 0 || dt > 4 || shown[dt]) continue;
                            shown[dt] = true;
                            ImVec4 dotCol;
                            switch (dt) {
                                case 1: dotCol = ImVec4(0.25f, 0.50f, 1.00f, 1.0f); break; // Magic: blue
                                case 2: dotCol = ImVec4(0.70f, 0.15f, 0.90f, 1.0f); break; // Curse: purple
                                case 3: dotCol = ImVec4(0.65f, 0.45f, 0.10f, 1.0f); break; // Disease: brown
                                case 4: dotCol = ImVec4(0.10f, 0.75f, 0.10f, 1.0f); break; // Poison: green
                                default: break;
                            }
                            ImGui::PushStyleColor(ImGuiCol_Button, dotCol);
                            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, dotCol);
                            ImGui::Button("##d", ImVec2(8.0f, 8.0f));
                            ImGui::PopStyleColor(2);
                            if (ImGui::IsItemHovered()) {
                                // Find spell name(s) of this dispel type
                                ImGui::BeginTooltip();
                                ImGui::TextColored(dotCol, "%s", kDispelNames[dt]);
                                for (const auto& da : *unitAuras) {
                                    if (da.isEmpty() || (da.flags & 0x80) == 0) continue;
                                    if (gameHandler.getSpellDispelType(da.spellId) != dt) continue;
                                    const std::string& dName = gameHandler.getSpellName(da.spellId);
                                    if (!dName.empty())
                                        ImGui::Text("  %s", dName.c_str());
                                }
                                ImGui::EndTooltip();
                            }
                            ImGui::SameLine();
                        }
                        ImGui::NewLine();
                        ImGui::PopStyleVar();
                    }
                }
            }

            // Party member cast bar — shows when the party member is casting
            if (auto* cs = gameHandler.getUnitCastState(member.guid)) {
                float castPct = (cs->timeTotal > 0.0f)
                    ? (cs->timeTotal - cs->timeRemaining) / cs->timeTotal : 0.0f;
                ImGui::PushStyleColor(ImGuiCol_PlotHistogram, colors::kMidHealthYellow);
                char pcastLabel[48];
                const std::string& spellNm = gameHandler.getSpellName(cs->spellId);
                if (!spellNm.empty())
                    snprintf(pcastLabel, sizeof(pcastLabel), "%s (%.1fs)", spellNm.c_str(), cs->timeRemaining);
                else
                    snprintf(pcastLabel, sizeof(pcastLabel), "Casting... (%.1fs)", cs->timeRemaining);
                {
                    VkDescriptorSet pIcon = (cs->spellId != 0 && assetMgr)
                        ? getSpellIcon(cs->spellId, assetMgr) : VK_NULL_HANDLE;
                    if (pIcon) {
                        ImGui::Image((ImTextureID)(uintptr_t)pIcon, ImVec2(10, 10));
                        ImGui::SameLine(0, 2);
                        ImGui::ProgressBar(castPct, ImVec2(-1, 10), pcastLabel);
                    } else {
                        ImGui::ProgressBar(castPct, ImVec2(-1, 10), pcastLabel);
                    }
                }
                ImGui::PopStyleColor();
            }

            // Right-click context menu for party member actions
            if (ImGui::BeginPopupContextItem("PartyMemberCtx")) {
                ImGui::TextDisabled("%s", member.name.c_str());
                ImGui::Separator();
                if (ImGui::MenuItem("Target")) {
                    gameHandler.setTarget(member.guid);
                }
                if (ImGui::MenuItem("Set Focus")) {
                    gameHandler.setFocus(member.guid);
                }
                if (ImGui::MenuItem("Whisper")) {
                    chatPanel.setWhisperTarget(member.name);
                }
                if (ImGui::MenuItem("Follow")) {
                    gameHandler.setTarget(member.guid);
                    gameHandler.followTarget();
                }
                if (ImGui::MenuItem("Trade")) {
                    gameHandler.initiateTrade(member.guid);
                }
                if (ImGui::MenuItem("Duel")) {
                    gameHandler.proposeDuel(member.guid);
                }
                if (ImGui::MenuItem("Inspect")) {
                    gameHandler.setTarget(member.guid);
                    gameHandler.inspectTarget();
                    showInspectWindow_ = true;
                }
                ImGui::Separator();
                if (!member.name.empty()) {
                    if (ImGui::MenuItem("Add Friend")) {
                        gameHandler.addFriend(member.name);
                    }
                    if (ImGui::MenuItem("Ignore")) {
                        gameHandler.addIgnore(member.name);
                    }
                }
                // Leader-only actions
                bool isLeader = (gameHandler.getPartyData().leaderGuid == gameHandler.getPlayerGuid());
                if (isLeader) {
                    ImGui::Separator();
                    if (ImGui::MenuItem("Kick from Group")) {
                        gameHandler.uninvitePlayer(member.name);
                    }
                }
                ImGui::Separator();
                if (ImGui::BeginMenu("Set Raid Mark")) {
                    for (int mi = 0; mi < 8; ++mi) {
                        if (ImGui::MenuItem(kRaidMarkNames[mi]))
                            gameHandler.setRaidMark(member.guid, static_cast<uint8_t>(mi));
                    }
                    ImGui::Separator();
                    if (ImGui::MenuItem("Clear Mark"))
                        gameHandler.setRaidMark(member.guid, 0xFF);
                    ImGui::EndMenu();
                }
                ImGui::EndPopup();
            }

            ImGui::Separator();
            ImGui::PopID();
        }
    }
    ImGui::End();

    ImGui::PopStyleColor();
    ImGui::PopStyleVar();
}

void SocialPanel::renderBossFrames(game::GameHandler& gameHandler,
                                      SpellbookScreen& spellbookScreen,
                                      SpellIconFn getSpellIcon) {
    auto* assetMgr = services_.assetManager;

    // Collect active boss unit slots
    struct BossSlot { uint32_t slot; uint64_t guid; };
    std::vector<BossSlot> active;
    for (uint32_t s = 0; s < game::GameHandler::kMaxEncounterSlots; ++s) {
        uint64_t g = gameHandler.getEncounterUnitGuid(s);
        if (g != 0) active.push_back({s, g});
    }
    if (active.empty()) return;

    const float frameW = 200.0f;
    const float startX = ImGui::GetIO().DisplaySize.x - frameW - 10.0f;
    float frameY = 120.0f;

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                             ImGuiWindowFlags_AlwaysAutoResize;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.15f, 0.05f, 0.05f, 0.85f));

    ImGui::SetNextWindowPos(ImVec2(startX, frameY), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(frameW, 0.0f), ImGuiCond_Always);

    if (ImGui::Begin("##BossFrames", nullptr, flags)) {
        for (const auto& bs : active) {
            ImGui::PushID(static_cast<int>(bs.guid));

            // Try to resolve name, health, and power from entity manager
            std::string name = "Boss";
            uint32_t hp = 0, maxHp = 0;
            uint8_t  bossPowerType = 0;
            uint32_t bossPower = 0, bossMaxPower = 0;
            auto entity = gameHandler.getEntityManager().getEntity(bs.guid);
            if (entity && (entity->getType() == game::ObjectType::UNIT ||
                           entity->getType() == game::ObjectType::PLAYER)) {
                auto unit = std::static_pointer_cast<game::Unit>(entity);
                const auto& n = unit->getName();
                if (!n.empty()) name = n;
                hp           = unit->getHealth();
                maxHp        = unit->getMaxHealth();
                bossPowerType = unit->getPowerType();
                bossPower     = unit->getPower();
                bossMaxPower  = unit->getMaxPower();
            }

            // Clickable name to target
            if (ImGui::Selectable(name.c_str(), gameHandler.getTargetGuid() == bs.guid)) {
                gameHandler.setTarget(bs.guid);
            }

            if (maxHp > 0) {
                float pct = static_cast<float>(hp) / static_cast<float>(maxHp);
                // Boss health bar in red shades
                ImGui::PushStyleColor(ImGuiCol_PlotHistogram,
                    pct > 0.5f ? colors::kLowHealthRed :
                    pct > 0.2f ? ImVec4(0.9f, 0.5f, 0.1f, 1.0f) :
                                 ImVec4(1.0f, 0.8f, 0.1f, 1.0f));
                char label[32];
                std::snprintf(label, sizeof(label), "%u / %u", hp, maxHp);
                ImGui::ProgressBar(pct, ImVec2(-1, 14), label);
                ImGui::PopStyleColor();
            }

            // Boss power bar — shown when boss has a non-zero power pool
            // Energy bosses (type 3) are particularly important: full energy signals ability use
            if (bossMaxPower > 0 && bossPower > 0) {
                float bpPct = static_cast<float>(bossPower) / static_cast<float>(bossMaxPower);
                ImVec4 bpColor;
                switch (bossPowerType) {
                    case 0: bpColor = ImVec4(0.2f, 0.3f, 0.9f, 1.0f); break; // Mana: blue
                    case 1: bpColor = colors::kDarkRed; break; // Rage: red
                    case 2: bpColor = colors::kOrange; break; // Focus: orange
                    case 3: bpColor = ImVec4(0.9f, 0.9f, 0.1f, 1.0f); break; // Energy: yellow
                    default: bpColor = ImVec4(0.4f, 0.8f, 0.4f, 1.0f); break;
                }
                ImGui::PushStyleColor(ImGuiCol_PlotHistogram, bpColor);
                char bpLabel[24];
                std::snprintf(bpLabel, sizeof(bpLabel), "%u", bossPower);
                ImGui::ProgressBar(bpPct, ImVec2(-1, 6), bpLabel);
                ImGui::PopStyleColor();
            }

            // Boss cast bar — shown when the boss is casting (critical for interrupt)
            if (auto* cs = gameHandler.getUnitCastState(bs.guid)) {
                float castPct  = (cs->timeTotal > 0.0f)
                    ? (cs->timeTotal - cs->timeRemaining) / cs->timeTotal : 0.0f;
                uint32_t bspell = cs->spellId;
                const std::string& bcastName = (bspell != 0)
                    ? gameHandler.getSpellName(bspell) : "";
                // Green = interruptible, Red = immune; pulse when > 80% complete
                ImVec4 bcastColor;
                if (castPct > 0.8f) {
                    float pulse = 0.7f + 0.3f * std::sin(static_cast<float>(ImGui::GetTime()) * 8.0f);
                    bcastColor = cs->interruptible
                        ? ImVec4(0.2f * pulse, 0.9f * pulse, 0.2f * pulse, 1.0f)
                        : ImVec4(1.0f * pulse, 0.1f * pulse, 0.1f * pulse, 1.0f);
                } else {
                    bcastColor = cs->interruptible
                        ? colors::kCastGreen
                        : ImVec4(0.9f, 0.15f, 0.15f, 1.0f);
                }
                ImGui::PushStyleColor(ImGuiCol_PlotHistogram, bcastColor);
                char bcastLabel[72];
                if (!bcastName.empty())
                    snprintf(bcastLabel, sizeof(bcastLabel), "%s (%.1fs)",
                             bcastName.c_str(), cs->timeRemaining);
                else
                    snprintf(bcastLabel, sizeof(bcastLabel), "Casting... (%.1fs)", cs->timeRemaining);
                {
                    VkDescriptorSet bIcon = (bspell != 0 && assetMgr)
                        ? getSpellIcon(bspell, assetMgr) : VK_NULL_HANDLE;
                    if (bIcon) {
                        ImGui::Image((ImTextureID)(uintptr_t)bIcon, ImVec2(12, 12));
                        ImGui::SameLine(0, 2);
                        ImGui::ProgressBar(castPct, ImVec2(-1, 12), bcastLabel);
                    } else {
                        ImGui::ProgressBar(castPct, ImVec2(-1, 12), bcastLabel);
                    }
                }
                ImGui::PopStyleColor();
            }

            // Boss aura row: debuffs first (player DoTs), then boss buffs
            {
                const std::vector<game::AuraSlot>* bossAuras = nullptr;
                if (bs.guid == gameHandler.getTargetGuid())
                    bossAuras = &gameHandler.getTargetAuras();
                else
                    bossAuras = gameHandler.getUnitAuras(bs.guid);

                if (bossAuras) {
                    int bossActive = 0;
                    for (const auto& a : *bossAuras) if (!a.isEmpty()) bossActive++;
                    if (bossActive > 0) {
                        constexpr float BA_ICON = 16.0f;
                        constexpr int   BA_PER_ROW = 10;

                        uint64_t baNowMs = static_cast<uint64_t>(
                            std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now().time_since_epoch()).count());

                        // Sort: player-applied debuffs first (most relevant), then others
                        const uint64_t pguid = gameHandler.getPlayerGuid();
                        std::vector<size_t> baIdx;
                        baIdx.reserve(bossAuras->size());
                        for (size_t i = 0; i < bossAuras->size(); ++i)
                            if (!(*bossAuras)[i].isEmpty()) baIdx.push_back(i);
                        std::sort(baIdx.begin(), baIdx.end(), [&](size_t a, size_t b) {
                            const auto& aa = (*bossAuras)[a];
                            const auto& ab = (*bossAuras)[b];
                            bool aPlayerDot = (aa.flags & 0x80) != 0 && aa.casterGuid == pguid;
                            bool bPlayerDot = (ab.flags & 0x80) != 0 && ab.casterGuid == pguid;
                            if (aPlayerDot != bPlayerDot) return aPlayerDot > bPlayerDot;
                            bool aDebuff = (aa.flags & 0x80) != 0;
                            bool bDebuff = (ab.flags & 0x80) != 0;
                            if (aDebuff != bDebuff) return aDebuff > bDebuff;
                            int32_t ra = aa.getRemainingMs(baNowMs);
                            int32_t rb = ab.getRemainingMs(baNowMs);
                            if (ra < 0 && rb < 0) return false;
                            if (ra < 0) return false;
                            if (rb < 0) return true;
                            return ra < rb;
                        });

                        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(2.0f, 2.0f));
                        int baShown = 0;
                        for (size_t si = 0; si < baIdx.size() && baShown < 20; ++si) {
                            const auto& aura = (*bossAuras)[baIdx[si]];
                            bool isBuff = (aura.flags & 0x80) == 0;
                            bool isPlayerCast = (aura.casterGuid == pguid);

                            if (baShown > 0 && baShown % BA_PER_ROW != 0) ImGui::SameLine();
                            ImGui::PushID(static_cast<int>(baIdx[si]) + 7000);

                            ImVec4 borderCol;
                            if (isBuff) {
                                // Boss buffs: gold for important enrage/shield types
                                borderCol = ImVec4(0.8f, 0.6f, 0.1f, 0.9f);
                            } else {
                                uint8_t dt = gameHandler.getSpellDispelType(aura.spellId);
                                switch (dt) {
                                    case 1: borderCol = ImVec4(0.15f, 0.50f, 1.00f, 0.9f); break;
                                    case 2: borderCol = ImVec4(0.70f, 0.20f, 0.90f, 0.9f); break;
                                    case 3: borderCol = ImVec4(0.55f, 0.30f, 0.10f, 0.9f); break;
                                    case 4: borderCol = ImVec4(0.10f, 0.70f, 0.10f, 0.9f); break;
                                    default: borderCol = isPlayerCast
                                        ? ImVec4(0.90f, 0.30f, 0.10f, 0.9f)   // player DoT: orange-red
                                        : ImVec4(0.60f, 0.20f, 0.20f, 0.9f);  // other debuff: dark red
                                        break;
                                }
                            }

                            VkDescriptorSet baIcon = assetMgr
                                ? getSpellIcon(aura.spellId, assetMgr) : VK_NULL_HANDLE;
                            if (baIcon) {
                                ImGui::PushStyleColor(ImGuiCol_Button, borderCol);
                                ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(1, 1));
                                ImGui::ImageButton("##baura",
                                    (ImTextureID)(uintptr_t)baIcon,
                                    ImVec2(BA_ICON - 2, BA_ICON - 2));
                                ImGui::PopStyleVar();
                                ImGui::PopStyleColor();
                            } else {
                                ImGui::PushStyleColor(ImGuiCol_Button, borderCol);
                                char lab[8];
                                snprintf(lab, sizeof(lab), "%u", aura.spellId % 10000);
                                ImGui::Button(lab, ImVec2(BA_ICON, BA_ICON));
                                ImGui::PopStyleColor();
                            }

                            // Duration overlay
                            int32_t baRemain = aura.getRemainingMs(baNowMs);
                            if (baRemain > 0) {
                                ImVec2 imin = ImGui::GetItemRectMin();
                                ImVec2 imax = ImGui::GetItemRectMax();
                                char ts[12];
                                fmtDurationCompact(ts, sizeof(ts), (baRemain + 999) / 1000);
                                ImVec2 tsz = ImGui::CalcTextSize(ts);
                                float cx = imin.x + (imax.x - imin.x - tsz.x) * 0.5f;
                                float cy = imax.y - tsz.y;
                                ImGui::GetWindowDrawList()->AddText(ImVec2(cx + 1, cy + 1), IM_COL32(0, 0, 0, 180), ts);
                                ImGui::GetWindowDrawList()->AddText(ImVec2(cx, cy), IM_COL32(255, 255, 255, 220), ts);
                            }

                            // Stack / charge count — upper-left corner (parity with target/focus frames)
                            if (aura.charges > 1) {
                                ImVec2 baMin = ImGui::GetItemRectMin();
                                char chargeStr[8];
                                snprintf(chargeStr, sizeof(chargeStr), "%u", static_cast<unsigned>(aura.charges));
                                ImGui::GetWindowDrawList()->AddText(ImVec2(baMin.x + 2, baMin.y + 2),
                                    IM_COL32(0, 0, 0, 200), chargeStr);
                                ImGui::GetWindowDrawList()->AddText(ImVec2(baMin.x + 1, baMin.y + 1),
                                    IM_COL32(255, 220, 50, 255), chargeStr);
                            }

                            // Tooltip
                            if (ImGui::IsItemHovered()) {
                                ImGui::BeginTooltip();
                                bool richOk = spellbookScreen.renderSpellInfoTooltip(
                                    aura.spellId, gameHandler, assetMgr);
                                if (!richOk) {
                                    std::string nm = spellbookScreen.lookupSpellName(aura.spellId, assetMgr);
                                    if (nm.empty()) nm = "Spell #" + std::to_string(aura.spellId);
                                    ImGui::Text("%s", nm.c_str());
                                }
                                if (isPlayerCast && !isBuff)
                                    ImGui::TextColored(ImVec4(0.9f, 0.7f, 0.3f, 1.0f), "Your DoT");
                                renderAuraRemaining(baRemain);
                                ImGui::EndTooltip();
                            }

                            ImGui::PopID();
                            baShown++;
                        }
                        ImGui::PopStyleVar();
                    }
                }
            }

            ImGui::PopID();
            ImGui::Spacing();
        }
    }
    ImGui::End();

    ImGui::PopStyleColor();
    ImGui::PopStyleVar();
}

void SocialPanel::renderGuildRoster(game::GameHandler& gameHandler,
                                       ChatPanel& chatPanel) {
    // Guild Roster toggle (customizable keybind)
    // Chat and text fields, not WantCaptureKeyboard. That flag is true whenever
    // any ImGui window wants the keyboard at all, and opening this window is
    // what gives it focus — so the key that opened it could not close it again.
    // Every other toggle key in the client gates on text focus alone, and
    // game_screen says why where it does it: a window having focus is not a
    // reason for its own toggle to stop working.
    if (!chatPanel.isChatInputActive() && !ImGui::GetIO().WantTextInput &&
        KeybindingManager::getInstance().isActionPressed(KeybindingManager::Action::TOGGLE_GUILD_ROSTER)) {
        showGuildRoster_ = !showGuildRoster_;
        if (showGuildRoster_) {
            // Open friends tab directly if not in guild
            if (!gameHandler.isInGuild()) {
                guildRosterTab_ = 2;  // Friends tab
            } else {
                // Re-query guild name if we have guildId but no name yet
                if (gameHandler.getGuildName().empty()) {
                    const auto* ch = gameHandler.getActiveCharacter();
                    if (ch && ch->hasGuild()) {
                        gameHandler.queryGuildInfo(ch->guildId);
                    }
                }
                gameHandler.requestGuildRoster();
                gameHandler.requestGuildInfo();
            }
        }
    }

    // Petition creation dialog (shown when NPC sends SMSG_PETITION_SHOWLIST)
    if (gameHandler.hasPetitionShowlist()) {
        ImGui::OpenPopup("CreateGuildPetition");
        gameHandler.clearPetitionDialog();
    }
    if (ImGui::BeginPopupModal("CreateGuildPetition", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("Create Guild Charter");
        ImGui::Separator();
        uint32_t cost = gameHandler.getPetitionCost();
        ImGui::TextDisabled("Cost:"); ImGui::SameLine(0, 4);
        renderCoinsFromCopper(cost);
        ImGui::Spacing();
        ImGui::Text("Guild Name:");
        ImGui::InputText("##petitionname", petitionNameBuffer_, sizeof(petitionNameBuffer_));
        ImGui::Spacing();
        if (ImGui::Button("Create", ImVec2(120, 0))) {
            if (petitionNameBuffer_[0] != '\0') {
                gameHandler.buyPetition(gameHandler.getPetitionNpcGuid(), petitionNameBuffer_);
                petitionNameBuffer_[0] = '\0';
                ImGui::CloseCurrentPopup();
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(120, 0))) {
            petitionNameBuffer_[0] = '\0';
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    // Petition signatures window (shown when a petition item is used or offered)
    if (gameHandler.hasPetitionSignaturesUI()) {
        ImGui::OpenPopup("PetitionSignatures");
        gameHandler.clearPetitionSignaturesUI();
    }
    if (ImGui::BeginPopupModal("PetitionSignatures", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        const auto& pInfo = gameHandler.getPetitionInfo();
        if (!pInfo.guildName.empty())
            ImGui::Text("Guild Charter: %s", pInfo.guildName.c_str());
        else
            ImGui::Text("Guild Charter");
        ImGui::Separator();

        ImGui::Text("Signatures: %u / %u", pInfo.signatureCount, pInfo.signaturesRequired);
        ImGui::Spacing();

        if (!pInfo.signatures.empty()) {
            for (size_t i = 0; i < pInfo.signatures.size(); ++i) {
                const auto& sig = pInfo.signatures[i];
                // Try to resolve name from entity manager
                std::string sigName;
                if (sig.playerGuid != 0) {
                    auto entity = gameHandler.getEntityManager().getEntity(sig.playerGuid);
                    if (entity) {
                        auto* unit = entity->isUnit() ? static_cast<game::Unit*>(entity.get()) : nullptr;
                        if (unit) sigName = unit->getName();
                    }
                }
                if (sigName.empty())
                    sigName = "Player " + std::to_string(i + 1);
                ImGui::BulletText("%s", sigName.c_str());
            }
            ImGui::Spacing();
        }

        // If we're not the owner, show Sign button
        bool isOwner = (pInfo.ownerGuid == gameHandler.getPlayerGuid());
        if (!isOwner) {
            if (ImGui::Button("Sign", ImVec2(120, 0))) {
                gameHandler.signPetition(pInfo.petitionGuid);
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
        } else if (pInfo.signatureCount >= pInfo.signaturesRequired) {
            // Owner with enough sigs — turn in
            if (ImGui::Button("Turn In", ImVec2(120, 0))) {
                gameHandler.turnInPetition(pInfo.petitionGuid);
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
        }
        if (ImGui::Button("Close", ImVec2(120, 0)))
            ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    if (!showGuildRoster_) return;

    // Get zone manager for name lookup
    game::ZoneManager* zoneManager = nullptr;
    if (auto* renderer = services_.renderer) {
        zoneManager = renderer->getZoneManager();
    }

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - 375, screenH / 2 - 250), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(750, 500), ImGuiCond_Once);

    std::string title = gameHandler.isInGuild() ? (gameHandler.getGuildName() + " - Social") : "Social";
    bool open = showGuildRoster_;
    if (ImGui::Begin(title.c_str(), &open, ImGuiWindowFlags_NoCollapse)) {
        // Tab bar: Roster | Guild Info
        if (ImGui::BeginTabBar("GuildTabs")) {
            if (ImGui::BeginTabItem("Roster")) {
                guildRosterTab_ = 0;
                if (!gameHandler.hasGuildRoster()) {
                    ImGui::Text("Loading roster...");
                } else {
                    const auto& roster = gameHandler.getGuildRoster();

                    // MOTD
                    if (!roster.motd.empty()) {
                        ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "MOTD: %s", roster.motd.c_str());
                        ImGui::Separator();
                    }

                    // Count online
                    int onlineCount = 0;
                    for (const auto& m : roster.members) {
                        if (m.online) ++onlineCount;
                    }
                    ImGui::Text("%d members (%d online)", static_cast<int>(roster.members.size()), onlineCount);
                    ImGui::Separator();

                    const auto& rankNames = gameHandler.getGuildRankNames();

                    // Table
                    if (ImGui::BeginTable("GuildRoster", 7,
                            ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                            ImGuiTableFlags_Sortable)) {
                        ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_DefaultSort);
                        ImGui::TableSetupColumn("Rank");
                        ImGui::TableSetupColumn("Level", ImGuiTableColumnFlags_WidthFixed, 40.0f);
                        ImGui::TableSetupColumn("Class", ImGuiTableColumnFlags_WidthFixed, 70.0f);
                        ImGui::TableSetupColumn("Zone", ImGuiTableColumnFlags_WidthFixed, 120.0f);
                        ImGui::TableSetupColumn("Note");
                        ImGui::TableSetupColumn("Officer Note");
                        ImGui::TableHeadersRow();

                        // Online members first, then offline
                        auto sortedMembers = roster.members;
                        std::sort(sortedMembers.begin(), sortedMembers.end(), [](const auto& a, const auto& b) {
                            if (a.online != b.online) return a.online > b.online;
                            return a.name < b.name;
                        });

                        for (const auto& m : sortedMembers) {
                            ImGui::TableNextRow();
                            ImVec4 textColor = m.online ? ui::colors::kWhite
                                                        : kColorDarkGray;
                            ImVec4 nameColor = m.online ? classColorVec4(m.classId) : textColor;

                            ImGui::TableNextColumn();
                            ImGui::TextColored(nameColor, "%s", m.name.c_str());

                            // Right-click context menu
                            if (ImGui::IsItemClicked(ImGuiMouseButton_Right)) {
                                selectedGuildMember_ = m.name;
                                ImGui::OpenPopup("GuildMemberContext");
                            }

                            ImGui::TableNextColumn();
                            // Show rank name instead of index
                            if (m.rankIndex < rankNames.size()) {
                                ImGui::TextColored(textColor, "%s", rankNames[m.rankIndex].c_str());
                            } else {
                                ImGui::TextColored(textColor, "Rank %u", m.rankIndex);
                            }

                            ImGui::TableNextColumn();
                            ImGui::TextColored(textColor, "%u", m.level);

                            ImGui::TableNextColumn();
                            const char* className = classNameStr(m.classId);
                            ImVec4 classCol = m.online ? classColorVec4(m.classId) : textColor;
                            ImGui::TextColored(classCol, "%s", className);

                            ImGui::TableNextColumn();
                            // Zone name lookup
                            if (zoneManager) {
                                const auto* zoneInfo = zoneManager->getZoneInfo(m.zoneId);
                                if (zoneInfo && !zoneInfo->name.empty()) {
                                    ImGui::TextColored(textColor, "%s", zoneInfo->name.c_str());
                                } else {
                                    ImGui::TextColored(textColor, "%u", m.zoneId);
                                }
                            } else {
                                ImGui::TextColored(textColor, "%u", m.zoneId);
                            }

                            ImGui::TableNextColumn();
                            ImGui::TextColored(textColor, "%s", m.publicNote.c_str());

                            ImGui::TableNextColumn();
                            ImGui::TextColored(textColor, "%s", m.officerNote.c_str());
                        }
                        ImGui::EndTable();
                    }

                    // Context menu popup
                    if (ImGui::BeginPopup("GuildMemberContext")) {
                        ImGui::TextDisabled("%s", selectedGuildMember_.c_str());
                        ImGui::Separator();
                        // Social actions — only for online members
                        bool memberOnline = false;
                        for (const auto& mem : roster.members) {
                            if (mem.name == selectedGuildMember_) { memberOnline = mem.online; break; }
                        }
                        if (memberOnline) {
                            if (ImGui::MenuItem("Whisper")) {
                                chatPanel.setWhisperTarget(selectedGuildMember_);
                            }
                            if (ImGui::MenuItem("Invite to Group")) {
                                gameHandler.inviteToGroup(selectedGuildMember_);
                            }
                            ImGui::Separator();
                        }
                        if (!selectedGuildMember_.empty()) {
                            if (ImGui::MenuItem("Add Friend"))
                                gameHandler.addFriend(selectedGuildMember_);
                            if (ImGui::MenuItem("Ignore"))
                                gameHandler.addIgnore(selectedGuildMember_);
                            ImGui::Separator();
                        }
                        if (ImGui::MenuItem("Promote")) {
                            gameHandler.promoteGuildMember(selectedGuildMember_);
                        }
                        if (ImGui::MenuItem("Demote")) {
                            gameHandler.demoteGuildMember(selectedGuildMember_);
                        }
                        if (ImGui::MenuItem("Kick")) {
                            gameHandler.kickGuildMember(selectedGuildMember_);
                        }
                        ImGui::Separator();
                        if (ImGui::MenuItem("Set Public Note...")) {
                            showGuildNoteEdit_ = true;
                            editingOfficerNote_ = false;
                            guildNoteEditBuffer_[0] = '\0';
                            // Pre-fill with existing note
                            for (const auto& mem : roster.members) {
                                if (mem.name == selectedGuildMember_) {
                                    snprintf(guildNoteEditBuffer_, sizeof(guildNoteEditBuffer_), "%s", mem.publicNote.c_str());
                                    break;
                                }
                            }
                        }
                        if (ImGui::MenuItem("Set Officer Note...")) {
                            showGuildNoteEdit_ = true;
                            editingOfficerNote_ = true;
                            guildNoteEditBuffer_[0] = '\0';
                            for (const auto& mem : roster.members) {
                                if (mem.name == selectedGuildMember_) {
                                    snprintf(guildNoteEditBuffer_, sizeof(guildNoteEditBuffer_), "%s", mem.officerNote.c_str());
                                    break;
                                }
                            }
                        }
                        ImGui::Separator();
                        if (ImGui::MenuItem("Set as Leader")) {
                            gameHandler.setGuildLeader(selectedGuildMember_);
                        }
                        ImGui::EndPopup();
                    }

                    // Note edit modal
                    if (showGuildNoteEdit_) {
                        ImGui::OpenPopup("EditGuildNote");
                        showGuildNoteEdit_ = false;
                    }
                    if (ImGui::BeginPopupModal("EditGuildNote", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
                        ImGui::Text("%s %s for %s:",
                            editingOfficerNote_ ? "Officer" : "Public", "Note", selectedGuildMember_.c_str());
                        ImGui::InputText("##guildnote", guildNoteEditBuffer_, sizeof(guildNoteEditBuffer_));
                        if (ImGui::Button("Save")) {
                            if (editingOfficerNote_) {
                                gameHandler.setGuildOfficerNote(selectedGuildMember_, guildNoteEditBuffer_);
                            } else {
                                gameHandler.setGuildPublicNote(selectedGuildMember_, guildNoteEditBuffer_);
                            }
                            ImGui::CloseCurrentPopup();
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("Cancel")) {
                            ImGui::CloseCurrentPopup();
                        }
                        ImGui::EndPopup();
                    }
                }
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Guild Info")) {
                guildRosterTab_ = 1;
                const auto& infoData = gameHandler.getGuildInfoData();
                const auto& queryData = gameHandler.getGuildQueryData();
                const auto& roster = gameHandler.getGuildRoster();
                const auto& rankNames = gameHandler.getGuildRankNames();

                // Guild name (large, gold)
                ImGui::PushFont(nullptr);  // default font
                ImGui::TextColored(ui::colors::kTooltipGold, "<%s>", gameHandler.getGuildName().c_str());
                ImGui::PopFont();
                ImGui::Separator();

                // Creation date
                if (infoData.isValid()) {
                    ImGui::Text("Created: %u/%u/%u", infoData.creationDay, infoData.creationMonth, infoData.creationYear);
                    ImGui::Text("Members: %u  |  Accounts: %u", infoData.numMembers, infoData.numAccounts);
                }
                ImGui::Spacing();

                // Guild description / info text
                if (!roster.guildInfo.empty()) {
                    ImGui::TextColored(colors::kSilver, "Description:");
                    ImGui::TextWrapped("%s", roster.guildInfo.c_str());
                }
                ImGui::Spacing();

                // MOTD with edit button
                ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "MOTD:");
                ImGui::SameLine();
                if (!roster.motd.empty()) {
                    ImGui::TextWrapped("%s", roster.motd.c_str());
                } else {
                    ImGui::TextColored(kColorDarkGray, "(not set)");
                }
                if (ImGui::Button("Set MOTD")) {
                    showMotdEdit_ = true;
                    snprintf(guildMotdEditBuffer_, sizeof(guildMotdEditBuffer_), "%s", roster.motd.c_str());
                }
                ImGui::Spacing();

                // MOTD edit modal
                if (showMotdEdit_) {
                    ImGui::OpenPopup("EditMotd");
                    showMotdEdit_ = false;
                }
                if (ImGui::BeginPopupModal("EditMotd", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
                    ImGui::Text("Set Message of the Day:");
                    ImGui::InputText("##motdinput", guildMotdEditBuffer_, sizeof(guildMotdEditBuffer_));
                    if (ImGui::Button("Save", ImVec2(120, 0))) {
                        gameHandler.setGuildMotd(guildMotdEditBuffer_);
                        ImGui::CloseCurrentPopup();
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Cancel", ImVec2(120, 0))) {
                        ImGui::CloseCurrentPopup();
                    }
                    ImGui::EndPopup();
                }

                // Emblem info
                if (queryData.isValid()) {
                    ImGui::Separator();
                    ImGui::Text("Emblem: Style %u, Color %u  |  Border: Style %u, Color %u  |  BG: %u",
                        queryData.emblemStyle, queryData.emblemColor,
                        queryData.borderStyle, queryData.borderColor, queryData.backgroundColor);
                }

                // Rank list
                ImGui::Separator();
                ImGui::TextColored(ui::colors::kTooltipGold, "Ranks:");
                for (size_t i = 0; i < rankNames.size(); ++i) {
                    if (rankNames[i].empty()) continue;
                    // Show rank permission summary from roster data
                    if (i < roster.ranks.size()) {
                        uint32_t rights = roster.ranks[i].rights;
                        std::string perms;
                        if (rights & 0x01) perms += "Invite ";
                        if (rights & 0x02) perms += "Remove ";
                        if (rights & 0x40) perms += "Promote ";
                        if (rights & 0x80) perms += "Demote ";
                        if (rights & 0x04) perms += "OChat ";
                        if (rights & 0x10) perms += "MOTD ";
                        ImGui::Text("  %zu. %s", i + 1, rankNames[i].c_str());
                        if (!perms.empty()) {
                            ImGui::SameLine();
                            ImGui::TextColored(kColorDarkGray, "[%s]", perms.c_str());
                        }
                    } else {
                        ImGui::Text("  %zu. %s", i + 1, rankNames[i].c_str());
                    }
                }

                // Rank management buttons
                ImGui::Spacing();
                if (ImGui::Button("Add Rank")) {
                    showAddRankModal_ = true;
                    addRankNameBuffer_[0] = '\0';
                }
                ImGui::SameLine();
                if (ImGui::Button("Delete Last Rank")) {
                    gameHandler.deleteGuildRank();
                }

                // Add rank modal
                if (showAddRankModal_) {
                    ImGui::OpenPopup("AddGuildRank");
                    showAddRankModal_ = false;
                }
                if (ImGui::BeginPopupModal("AddGuildRank", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
                    ImGui::Text("New Rank Name:");
                    ImGui::InputText("##rankname", addRankNameBuffer_, sizeof(addRankNameBuffer_));
                    if (ImGui::Button("Add", ImVec2(120, 0))) {
                        if (addRankNameBuffer_[0] != '\0') {
                            gameHandler.addGuildRank(addRankNameBuffer_);
                            ImGui::CloseCurrentPopup();
                        }
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Cancel", ImVec2(120, 0))) {
                        ImGui::CloseCurrentPopup();
                    }
                    ImGui::EndPopup();
                }

                ImGui::EndTabItem();
            }

            // ---- Friends tab ----
            if (ImGui::BeginTabItem("Friends")) {
                guildRosterTab_ = 2;
                const auto& contacts = gameHandler.getContacts();

                // Add Friend row
                static char addFriendBuf[64] = {};
                ImGui::SetNextItemWidth(180.0f);
                ImGui::InputText("##addfriend", addFriendBuf, sizeof(addFriendBuf));
                ImGui::SameLine();
                if (ImGui::Button("Add Friend") && addFriendBuf[0] != '\0') {
                    gameHandler.addFriend(addFriendBuf);
                    addFriendBuf[0] = '\0';
                }
                ImGui::Separator();

                // Note-edit state
                static std::string friendNoteTarget;
                static char friendNoteBuf[256] = {};
                static bool openNotePopup = false;

                // Filter to friends only
                int friendCount = 0;
                for (size_t ci = 0; ci < contacts.size(); ++ci) {
                    const auto& c = contacts[ci];
                    if (!c.isFriend()) continue;
                    ++friendCount;

                    ImGui::PushID(static_cast<int>(ci));

                    // Status dot
                    ImU32 dotColor = c.isOnline()
                        ? IM_COL32(80, 200, 80, 255)
                        : IM_COL32(120, 120, 120, 255);
                    ImVec2 cursor = ImGui::GetCursorScreenPos();
                    ImGui::GetWindowDrawList()->AddCircleFilled(
                        ImVec2(cursor.x + 6.0f, cursor.y + 8.0f), 5.0f, dotColor);
                    ImGui::Dummy(ImVec2(14.0f, 0.0f));
                    ImGui::SameLine();

                    // Name as Selectable for right-click context menu
                    const char* displayName = c.name.empty() ? "(unknown)" : c.name.c_str();
                    ImVec4 nameCol = c.isOnline()
                        ? ui::colors::kWhite
                        : colors::kInactiveGray;
                    ImGui::PushStyleColor(ImGuiCol_Text, nameCol);
                    ImGui::Selectable(displayName, false, ImGuiSelectableFlags_AllowOverlap, ImVec2(130.0f, 0.0f));
                    ImGui::PopStyleColor();

                    // Double-click to whisper
                    if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)
                        && !c.name.empty()) {
                        chatPanel.setWhisperTarget(c.name);
                    }

                    // Right-click context menu
                    if (ImGui::BeginPopupContextItem("FriendCtx")) {
                        ImGui::TextDisabled("%s", displayName);
                        ImGui::Separator();
                        if (ImGui::MenuItem("Whisper") && !c.name.empty()) {
                            chatPanel.setWhisperTarget(c.name);
                        }
                        if (c.isOnline() && ImGui::MenuItem("Invite to Group") && !c.name.empty()) {
                            gameHandler.inviteToGroup(c.name);
                        }
                        if (ImGui::MenuItem("Edit Note")) {
                            friendNoteTarget = c.name;
                            strncpy(friendNoteBuf, c.note.c_str(), sizeof(friendNoteBuf) - 1);
                            friendNoteBuf[sizeof(friendNoteBuf) - 1] = '\0';
                            openNotePopup = true;
                        }
                        ImGui::Separator();
                        if (ImGui::MenuItem("Remove Friend")) {
                            gameHandler.removeFriend(c.name);
                        }
                        ImGui::EndPopup();
                    }

                    // Note tooltip on hover
                    if (ImGui::IsItemHovered() && !c.note.empty()) {
                        ImGui::BeginTooltip();
                        ImGui::TextDisabled("Note: %s", c.note.c_str());
                        ImGui::EndTooltip();
                    }

                    // Level, class, and status
                    if (c.isOnline()) {
                        ImGui::SameLine(150.0f);
                        const char* statusLabel =
                            (c.status == 2) ? " (AFK)" :
                            (c.status == 3) ? " (DND)" : "";
                        // Class color for the level/class display
                        ImVec4 friendClassCol = classColorVec4(static_cast<uint8_t>(c.classId));
                        const char* friendClassName = classNameStr(static_cast<uint8_t>(c.classId));
                        if (c.level > 0 && c.classId > 0) {
                            ImGui::TextColored(friendClassCol, "Lv%u %s%s", c.level, friendClassName, statusLabel);
                        } else if (c.level > 0) {
                            ImGui::TextDisabled("Lv %u%s", c.level, statusLabel);
                        } else if (*statusLabel) {
                            ImGui::TextDisabled("%s", statusLabel + 1);
                        }

                        // Tooltip: zone info
                        if (ImGui::IsItemHovered() && c.areaId != 0) {
                            ImGui::BeginTooltip();
                            if (zoneManager) {
                                const auto* zi = zoneManager->getZoneInfo(c.areaId);
                                if (zi && !zi->name.empty())
                                    ImGui::Text("Zone: %s", zi->name.c_str());
                                else
                                    ImGui::TextDisabled("Area ID: %u", c.areaId);
                            } else {
                                ImGui::TextDisabled("Area ID: %u", c.areaId);
                            }
                            ImGui::EndTooltip();
                        }
                    }

                    ImGui::PopID();
                }

                if (friendCount == 0) {
                    ImGui::TextDisabled("No friends found.");
                }

                // Note edit modal
                if (openNotePopup) {
                    ImGui::OpenPopup("EditFriendNote");
                    openNotePopup = false;
                }
                if (ImGui::BeginPopupModal("EditFriendNote", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
                    ImGui::Text("Note for %s:", friendNoteTarget.c_str());
                    ImGui::SetNextItemWidth(240.0f);
                    ImGui::InputText("##fnote", friendNoteBuf, sizeof(friendNoteBuf));
                    if (ImGui::Button("Save", ImVec2(110, 0))) {
                        gameHandler.setFriendNote(friendNoteTarget, friendNoteBuf);
                        ImGui::CloseCurrentPopup();
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Cancel", ImVec2(110, 0))) {
                        ImGui::CloseCurrentPopup();
                    }
                    ImGui::EndPopup();
                }

                ImGui::EndTabItem();
            }

            // ---- Ignore List tab ----
            if (ImGui::BeginTabItem("Ignore")) {
                guildRosterTab_ = 3;
                const auto& contacts = gameHandler.getContacts();

                // Add Ignore row
                static char addIgnoreBuf[64] = {};
                ImGui::SetNextItemWidth(180.0f);
                ImGui::InputText("##addignore", addIgnoreBuf, sizeof(addIgnoreBuf));
                ImGui::SameLine();
                if (ImGui::Button("Ignore Player") && addIgnoreBuf[0] != '\0') {
                    gameHandler.addIgnore(addIgnoreBuf);
                    addIgnoreBuf[0] = '\0';
                }
                ImGui::Separator();

                int ignoreCount = 0;
                for (size_t ci = 0; ci < contacts.size(); ++ci) {
                    const auto& c = contacts[ci];
                    if (!c.isIgnored()) continue;
                    ++ignoreCount;

                    ImGui::PushID(static_cast<int>(ci) + 10000);
                    const char* displayName = c.name.empty() ? "(unknown)" : c.name.c_str();
                    ImGui::Selectable(displayName, false, ImGuiSelectableFlags_AllowOverlap);
                    if (ImGui::BeginPopupContextItem("IgnoreCtx")) {
                        ImGui::TextDisabled("%s", displayName);
                        ImGui::Separator();
                        if (ImGui::MenuItem("Remove Ignore")) {
                            gameHandler.removeIgnore(c.name);
                        }
                        ImGui::EndPopup();
                    }
                    ImGui::PopID();
                }

                if (ignoreCount == 0) {
                    ImGui::TextDisabled("Ignore list is empty.");
                }

                ImGui::EndTabItem();
            }

            ImGui::EndTabBar();
        }
    }
    ImGui::End();
    showGuildRoster_ = open;
}

void SocialPanel::renderSocialFrame(game::GameHandler& gameHandler,
                                       ChatPanel& chatPanel) {
    if (!showSocialFrame_) return;

    const auto& contacts = gameHandler.getContacts();
    // Count online friends for early-out
    int onlineCount = 0;
    for (const auto& c : contacts)
        if (c.isFriend() && c.isOnline()) ++onlineCount;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW - 230.0f, 240.0f), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(220.0f, 0.0f), ImGuiCond_Always);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.1f, 0.1f, 0.92f));

    // State for "Set Note" inline editing
    static int  noteEditContactIdx = -1;
    static char noteEditBuf[128]   = {};

    bool open = showSocialFrame_;
    char socialTitle[32];
    snprintf(socialTitle, sizeof(socialTitle), "Social (%d online)##SocialFrame", onlineCount);
    if (ImGui::Begin(socialTitle, &open,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
            ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar)) {

        // Get zone manager for area name lookups
        game::ZoneManager* socialZoneMgr = nullptr;
        if (auto* rend = services_.renderer)
            socialZoneMgr = rend->getZoneManager();

        if (ImGui::BeginTabBar("##SocialTabs")) {
            // ---- Friends tab ----
            if (ImGui::BeginTabItem("Friends")) {
                ImGui::BeginChild("##FriendsList", ImVec2(200, 200), false);

                // Online friends first
                int shown = 0;
                for (int pass = 0; pass < 2; ++pass) {
                    bool wantOnline = (pass == 0);
                    for (size_t ci = 0; ci < contacts.size(); ++ci) {
                        const auto& c = contacts[ci];
                        if (!c.isFriend()) continue;
                        if (c.isOnline() != wantOnline) continue;

                        ImGui::PushID(static_cast<int>(ci));

                        // Status dot
                        ImU32 dotColor;
                        if (!c.isOnline())        dotColor = IM_COL32(100, 100, 100, 200);
                        else if (c.status == 2)   dotColor = IM_COL32(255, 200,  50, 255); // AFK
                        else if (c.status == 3)   dotColor = IM_COL32(255, 120,  50, 255); // DND
                        else                      dotColor = IM_COL32( 50, 220,  50, 255); // online

                        ImVec2 dotMin = ImGui::GetCursorScreenPos();
                        dotMin.y += 4.0f;
                        ImGui::GetWindowDrawList()->AddCircleFilled(
                            ImVec2(dotMin.x + 5.0f, dotMin.y + 5.0f), 4.5f, dotColor);
                        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 14.0f);

                        const char* displayName = c.name.empty() ? "(unknown)" : c.name.c_str();
                        ImVec4 nameCol = c.isOnline()
                            ? classColorVec4(static_cast<uint8_t>(c.classId))
                            : kColorDarkGray;
                        ImGui::TextColored(nameCol, "%s", displayName);

                        if (c.isOnline() && c.level > 0) {
                            ImGui::SameLine();
                            // Show level and class name in class color
                            ImGui::TextColored(classColorVec4(static_cast<uint8_t>(c.classId)),
                                "Lv%u %s", c.level, classNameStr(static_cast<uint8_t>(c.classId)));
                        }

                        // Tooltip: zone info and note
                        if (ImGui::IsItemHovered() || (c.isOnline() && ImGui::IsItemHovered())) {
                            if (c.isOnline() && (c.areaId != 0 || !c.note.empty())) {
                                ImGui::BeginTooltip();
                                if (c.areaId != 0) {
                                    const char* zoneName = nullptr;
                                    if (socialZoneMgr) {
                                        const auto* zi = socialZoneMgr->getZoneInfo(c.areaId);
                                        if (zi && !zi->name.empty()) zoneName = zi->name.c_str();
                                    }
                                    if (zoneName)
                                        ImGui::Text("Zone: %s", zoneName);
                                    else
                                        ImGui::Text("Area ID: %u", c.areaId);
                                }
                                if (!c.note.empty())
                                    ImGui::TextDisabled("Note: %s", c.note.c_str());
                                ImGui::EndTooltip();
                            }
                        }

                        // Right-click context menu
                        if (ImGui::BeginPopupContextItem("FriendCtx")) {
                            ImGui::TextDisabled("%s", displayName);
                            ImGui::Separator();
                            if (c.isOnline()) {
                                if (ImGui::MenuItem("Whisper")) {
                                    showSocialFrame_ = false;
                                    chatPanel.setWhisperTarget(c.name);
                                }
                                if (ImGui::MenuItem("Invite to Group"))
                                    gameHandler.inviteToGroup(c.name);
                                if (c.guid != 0 && ImGui::MenuItem("Trade"))
                                    gameHandler.initiateTrade(c.guid);
                            }
                            if (ImGui::MenuItem("Set Note")) {
                                noteEditContactIdx = static_cast<int>(ci);
                                strncpy(noteEditBuf, c.note.c_str(), sizeof(noteEditBuf) - 1);
                                noteEditBuf[sizeof(noteEditBuf) - 1] = '\0';
                                ImGui::OpenPopup("##SetFriendNote");
                            }
                            if (ImGui::MenuItem("Remove Friend"))
                                gameHandler.removeFriend(c.name);
                            ImGui::EndPopup();
                        }

                        ++shown;
                        ImGui::PopID();
                    }
                    // Separator between online and offline if there are both
                    if (pass == 0 && shown > 0) {
                        ImGui::Separator();
                    }
                }

                if (shown == 0) {
                    ImGui::TextDisabled("No friends yet.");
                }

                ImGui::EndChild();

                // "Set Note" modal popup
                if (ImGui::BeginPopup("##SetFriendNote")) {
                    const std::string& noteName = (noteEditContactIdx >= 0 &&
                        noteEditContactIdx < static_cast<int>(contacts.size()))
                        ? contacts[noteEditContactIdx].name : "";
                    ImGui::TextDisabled("Note for %s:", noteName.c_str());
                    ImGui::SetNextItemWidth(180.0f);
                    bool confirm = ImGui::InputText("##noteinput", noteEditBuf, sizeof(noteEditBuf),
                        ImGuiInputTextFlags_EnterReturnsTrue);
                    ImGui::SameLine();
                    if (confirm || ImGui::Button("OK")) {
                        if (!noteName.empty())
                            gameHandler.setFriendNote(noteName, noteEditBuf);
                        noteEditContactIdx = -1;
                        ImGui::CloseCurrentPopup();
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Cancel")) {
                        noteEditContactIdx = -1;
                        ImGui::CloseCurrentPopup();
                    }
                    ImGui::EndPopup();
                }

                ImGui::Separator();

                // Add friend
                static char addFriendBuf[64] = {};
                ImGui::SetNextItemWidth(140.0f);
                ImGui::InputText("##sf_addfriend", addFriendBuf, sizeof(addFriendBuf));
                ImGui::SameLine();
                if (ImGui::Button("+##addfriend") && addFriendBuf[0] != '\0') {
                    gameHandler.addFriend(addFriendBuf);
                    addFriendBuf[0] = '\0';
                }

                ImGui::EndTabItem();
            }

            // ---- Ignore tab ----
            if (ImGui::BeginTabItem("Ignore")) {
                const auto& ignores = gameHandler.getIgnoreCache();
                ImGui::BeginChild("##IgnoreList", ImVec2(200, 200), false);

                if (ignores.empty()) {
                    ImGui::TextDisabled("Ignore list is empty.");
                } else {
                    for (const auto& kv : ignores) {
                        ImGui::PushID(kv.first.c_str());
                        ImGui::TextUnformatted(kv.first.c_str());
                        if (ImGui::BeginPopupContextItem("IgnoreCtx")) {
                            ImGui::TextDisabled("%s", kv.first.c_str());
                            ImGui::Separator();
                            if (ImGui::MenuItem("Unignore"))
                                gameHandler.removeIgnore(kv.first);
                            ImGui::EndPopup();
                        }
                        ImGui::PopID();
                    }
                }

                ImGui::EndChild();
                ImGui::Separator();

                // Add ignore
                static char addIgnBuf[64] = {};
                ImGui::SetNextItemWidth(140.0f);
                ImGui::InputText("##sf_addignore", addIgnBuf, sizeof(addIgnBuf));
                ImGui::SameLine();
                if (ImGui::Button("+##addignore") && addIgnBuf[0] != '\0') {
                    gameHandler.addIgnore(addIgnBuf);
                    addIgnBuf[0] = '\0';
                }

                ImGui::EndTabItem();
            }

            // ---- Channels tab ----
            if (ImGui::BeginTabItem("Channels")) {
                const auto& channels = gameHandler.getJoinedChannels();
                ImGui::BeginChild("##ChannelList", ImVec2(200, 200), false);

                if (channels.empty()) {
                    ImGui::TextDisabled("Not in any channels.");
                } else {
                    for (size_t ci = 0; ci < channels.size(); ++ci) {
                        ImGui::PushID(static_cast<int>(ci));
                        ImGui::TextUnformatted(channels[ci].c_str());
                        if (ImGui::BeginPopupContextItem("ChanCtx")) {
                            ImGui::TextDisabled("%s", channels[ci].c_str());
                            ImGui::Separator();
                            if (ImGui::MenuItem("Leave Channel"))
                                gameHandler.leaveChannel(channels[ci]);
                            ImGui::EndPopup();
                        }
                        ImGui::PopID();
                    }
                }

                ImGui::EndChild();
                ImGui::Separator();

                // Join a channel
                static char joinChanBuf[64] = {};
                ImGui::SetNextItemWidth(140.0f);
                ImGui::InputText("##sf_joinchan", joinChanBuf, sizeof(joinChanBuf));
                ImGui::SameLine();
                if (ImGui::Button("+##joinchan") && joinChanBuf[0] != '\0') {
                    gameHandler.joinChannel(joinChanBuf);
                    joinChanBuf[0] = '\0';
                }

                ImGui::EndTabItem();
            }

            // ---- Arena tab (WotLK: shows per-team rating/record + roster) ----
            const auto& arenaStats = gameHandler.getArenaTeamStats();
            if (!arenaStats.empty()) {
                if (ImGui::BeginTabItem("Arena")) {
                    ImGui::BeginChild("##ArenaList", ImVec2(0, 0), false);

                    for (size_t ai = 0; ai < arenaStats.size(); ++ai) {
                        const auto& ts = arenaStats[ai];
                        ImGui::PushID(static_cast<int>(ai));

                        // Team header: "2v2: Team Name" or fallback "Team #id"
                        std::string teamLabel;
                        if (ts.teamType > 0)
                            teamLabel = std::to_string(ts.teamType) + "v" + std::to_string(ts.teamType) + ": ";
                        if (!ts.teamName.empty())
                            teamLabel += ts.teamName;
                        else
                            teamLabel += "Team #" + std::to_string(ts.teamId);
                        ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.2f, 1.0f), "%s", teamLabel.c_str());

                        ImGui::Indent(8.0f);
                        // Rating and rank
                        ImGui::Text("Rating: %u", ts.rating);
                        if (ts.rank > 0) {
                            ImGui::SameLine(0, 6);
                            ImGui::TextDisabled("(Rank #%u)", ts.rank);
                        }

                        // Weekly record
                        uint32_t weekLosses = ts.weekGames > ts.weekWins
                                              ? ts.weekGames - ts.weekWins : 0;
                        ImGui::Text("Week:   %u W / %u L", ts.weekWins, weekLosses);

                        // Season record
                        uint32_t seasLosses = ts.seasonGames > ts.seasonWins
                                              ? ts.seasonGames - ts.seasonWins : 0;
                        ImGui::Text("Season: %u W / %u L", ts.seasonWins, seasLosses);

                        // Roster members (from SMSG_ARENA_TEAM_ROSTER)
                        const auto* roster = gameHandler.getArenaTeamRoster(ts.teamId);
                        if (roster && !roster->members.empty()) {
                            ImGui::Spacing();
                            ImGui::TextDisabled("-- Roster (%zu members) --",
                                                roster->members.size());
                            ImGui::SameLine();
                            if (ImGui::SmallButton("Refresh"))
                                gameHandler.requestArenaTeamRoster(ts.teamId);

                            // Column headers
                            ImGui::Columns(4, "##arenaRosterCols", false);
                            ImGui::SetColumnWidth(0, 110.0f);
                            ImGui::SetColumnWidth(1, 60.0f);
                            ImGui::SetColumnWidth(2, 60.0f);
                            ImGui::SetColumnWidth(3, 60.0f);
                            ImGui::TextDisabled("Name");      ImGui::NextColumn();
                            ImGui::TextDisabled("Rating");    ImGui::NextColumn();
                            ImGui::TextDisabled("Week");      ImGui::NextColumn();
                            ImGui::TextDisabled("Season");    ImGui::NextColumn();
                            ImGui::Separator();

                            for (const auto& m : roster->members) {
                                // Name coloured green (online) or grey (offline)
                                if (m.online)
                                    ImGui::TextColored(ImVec4(0.4f,1.0f,0.4f,1.0f),
                                                       "%s", m.name.c_str());
                                else
                                    ImGui::TextDisabled("%s", m.name.c_str());
                                ImGui::NextColumn();

                                ImGui::Text("%u", m.personalRating);
                                ImGui::NextColumn();

                                uint32_t wL = m.weekGames > m.weekWins
                                              ? m.weekGames - m.weekWins : 0;
                                ImGui::Text("%uW/%uL", m.weekWins, wL);
                                ImGui::NextColumn();

                                uint32_t sL = m.seasonGames > m.seasonWins
                                              ? m.seasonGames - m.seasonWins : 0;
                                ImGui::Text("%uW/%uL", m.seasonWins, sL);
                                ImGui::NextColumn();
                            }
                            ImGui::Columns(1);
                        } else {
                            ImGui::Spacing();
                            if (ImGui::SmallButton("Load Roster"))
                                gameHandler.requestArenaTeamRoster(ts.teamId);
                        }

                        ImGui::Unindent(8.0f);

                        if (ai + 1 < arenaStats.size())
                            ImGui::Separator();

                        ImGui::PopID();
                    }

                    ImGui::EndChild();
                    ImGui::EndTabItem();
                }
            }

            ImGui::EndTabBar();
        }
    }
    ImGui::End();
    showSocialFrame_ = open;

    ImGui::PopStyleColor();
    ImGui::PopStyleVar();
}

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
        // valid queue is never blocked — the server remains the authority.
        struct DungeonEntryEx { uint32_t id; const char* name; uint8_t cat; uint8_t minLvl; };
        static const DungeonEntryEx kDungeons[] = {
            { 861, "Random Dungeon",               0, 15 },
            { 862, "Random Heroic",                0, 80 },
            {  36, "Deadmines",                    1, 15 },
            {  43, "Ragefire Chasm",               1, 15 },
            {  47, "Razorfen Kraul",               1, 24 },
            {  48, "Blackfathom Deeps",            1, 21 },
            {  52, "Uldaman",                      1, 34 },
            {  57, "Dire Maul: East",              1, 46 },
            {  70, "Onyxia's Lair",                1, 80 },
            { 264, "The Blood Furnace",            2, 58 },
            { 269, "The Shattered Halls",          2, 67 },
            { 576, "The Nexus",                    3, 68 },
            { 578, "The Oculus",                   3, 76 },
            { 595, "The Culling of Stratholme",    3, 77 },
            { 599, "Halls of Stone",               3, 74 },
            { 600, "Drak'Tharon Keep",             3, 71 },
            { 601, "Azjol-Nerub",                  3, 70 },
            { 604, "Gundrak",                      3, 74 },
            { 608, "Violet Hold",                  3, 72 },
            { 619, "Ahn'kahet: Old Kingdom",       3, 71 },
            { 623, "Halls of Lightning",           3, 77 },
            { 632, "The Forge of Souls",           3, 79 },
            { 650, "Trial of the Champion",        3, 79 },
            { 658, "Pit of Saron",                 3, 79 },
            { 668, "Halls of Reflection",          3, 79 },
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
            for (int i = 0; i < kDungeonCount; ++i) {
                if (kDungeons[i].cat != lastCat && kCatHeaders[kDungeons[i].cat]) {
                    if (lastCat != 255) ImGui::Separator();
                    ImGui::TextDisabled("%s", kCatHeaders[kDungeons[i].cat]);
                    lastCat = kDungeons[i].cat;
                } else if (kDungeons[i].cat != lastCat) {
                    lastCat = kDungeons[i].cat;
                }
                // Grey out dungeons the player is too low to queue for.
                const bool tooLow = (playerLvl != 0 && playerLvl < kDungeons[i].minLvl);
                char label[96];
                dungeonLabel(kDungeons[i], label, sizeof(label));
                bool selected = (kDungeons[i].id == lfgSelectedDungeon_);
                if (tooLow) {
                    ImGui::TextDisabled("%s", label);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Requires level %u.", static_cast<unsigned>(kDungeons[i].minLvl));
                } else {
                    if (ImGui::Selectable(label, selected))
                        lfgSelectedDungeon_ = kDungeons[i].id;
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
            gameHandler.lfgJoin(lfgSelectedDungeon_, lfgRoles_);
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

void SocialPanel::renderWhoWindow(game::GameHandler& gameHandler,
                                     ChatPanel& chatPanel) {
    if (!showWhoWindow_) return;

    const auto& results = gameHandler.getWhoResults();

    ImGui::SetNextWindowSize(ImVec2(500, 300), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(200, 180), ImGuiCond_FirstUseEver);

    char title[64];
    uint32_t onlineCount = gameHandler.getWhoOnlineCount();
    if (onlineCount > 0)
        snprintf(title, sizeof(title), "Players Online: %u###WhoWindow", onlineCount);
    else
        snprintf(title, sizeof(title), "Who###WhoWindow");

    if (!ImGui::Begin(title, &showWhoWindow_)) {
        ImGui::End();
        return;
    }

    // Search bar with Send button
    static char whoSearchBuf[64] = {};
    bool doSearch = false;
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - 60.0f);
    if (ImGui::InputTextWithHint("##whosearch", "Search players...", whoSearchBuf, sizeof(whoSearchBuf),
            ImGuiInputTextFlags_EnterReturnsTrue))
        doSearch = true;
    ImGui::SameLine();
    if (ImGui::Button("Search", ImVec2(-1, 0)))
        doSearch = true;
    if (doSearch) {
        gameHandler.queryWho(std::string(whoSearchBuf));
    }
    ImGui::Separator();

    if (results.empty()) {
        ImGui::TextDisabled("No results. Type a filter above or use /who [filter].");
        ImGui::End();
        return;
    }

    // Table: Name | Guild | Level | Class | Zone
    if (ImGui::BeginTable("##WhoTable", 5,
            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
            ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingStretchProp,
            ImVec2(0, 0))) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("Name",  ImGuiTableColumnFlags_WidthStretch, 0.22f);
        ImGui::TableSetupColumn("Guild", ImGuiTableColumnFlags_WidthStretch, 0.20f);
        ImGui::TableSetupColumn("Level", ImGuiTableColumnFlags_WidthFixed,   40.0f);
        ImGui::TableSetupColumn("Class", ImGuiTableColumnFlags_WidthStretch, 0.20f);
        ImGui::TableSetupColumn("Zone",  ImGuiTableColumnFlags_WidthStretch, 0.28f);
        ImGui::TableHeadersRow();

        for (size_t i = 0; i < results.size(); ++i) {
            const auto& e = results[i];
            ImGui::TableNextRow();
            ImGui::PushID(static_cast<int>(i));

            // Name (class-colored if class is known)
            ImGui::TableSetColumnIndex(0);
            uint8_t cid = static_cast<uint8_t>(e.classId);
            ImVec4 nameCol = classColorVec4(cid);
            ImGui::TextColored(nameCol, "%s", e.name.c_str());

            // Right-click context menu on the name
            if (ImGui::BeginPopupContextItem("##WhoCtx")) {
                ImGui::TextDisabled("%s", e.name.c_str());
                ImGui::Separator();
                if (ImGui::MenuItem("Whisper")) {
                    chatPanel.setWhisperTarget(e.name);
                }
                if (ImGui::MenuItem("Invite to Group"))
                    gameHandler.inviteToGroup(e.name);
                if (ImGui::MenuItem("Add Friend"))
                    gameHandler.addFriend(e.name);
                if (ImGui::MenuItem("Ignore"))
                    gameHandler.addIgnore(e.name);
                ImGui::EndPopup();
            }

            // Guild
            ImGui::TableSetColumnIndex(1);
            if (!e.guildName.empty())
                ImGui::TextDisabled("<%s>", e.guildName.c_str());

            // Level
            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%u", e.level);

            // Class
            ImGui::TableSetColumnIndex(3);
            const char* className = game::getClassName(static_cast<game::Class>(e.classId));
            ImGui::TextColored(nameCol, "%s", className);

            // Zone
            ImGui::TableSetColumnIndex(4);
            if (e.zoneId != 0) {
                std::string zoneName = gameHandler.getWhoAreaName(e.zoneId);
                if (!zoneName.empty())
                    ImGui::TextUnformatted(zoneName.c_str());
                else {
                    char zfb[32];
                    snprintf(zfb, sizeof(zfb), "Zone #%u", e.zoneId);
                    ImGui::TextUnformatted(zfb);
                }
            }

            ImGui::PopID();
        }

        ImGui::EndTable();
    }

    ImGui::End();
}

void SocialPanel::renderInspectWindow(game::GameHandler& gameHandler,
                                         InventoryScreen& inventoryScreen) {
    if (!showInspectWindow_) {
        inspectWindowAutoRequestGuid_ = 0;
        return;
    }

    // Lazy-load SpellItemEnchantment.dbc for enchant name lookup
    static std::unordered_map<uint32_t, std::string> s_enchantNames;
    static bool s_enchantDbLoaded = false;
    auto* assetMgrEnchant = services_.assetManager;
    if (!s_enchantDbLoaded && assetMgrEnchant && assetMgrEnchant->isInitialized()) {
        s_enchantDbLoaded = true;
        auto dbc = assetMgrEnchant->loadDBC("SpellItemEnchantment.dbc");
        if (dbc && dbc->isLoaded()) {
            const auto* layout = pipeline::getActiveDBCLayout()
                                 ? pipeline::getActiveDBCLayout()->getLayout("SpellItemEnchantment")
                                 : nullptr;
            uint32_t idField   = layout ? (*layout)["ID"]   : 0;
            uint32_t nameField = pipeline::detectEnchantmentNameField(dbc.get(), layout);
            for (uint32_t i = 0; i < dbc->getRecordCount(); ++i) {
                uint32_t id = dbc->getUInt32(i, idField);
                if (id == 0) continue;
                std::string nm = dbc->getString(i, nameField);
                if (!nm.empty()) s_enchantNames[id] = std::move(nm);
            }
        }
    }

    // Slot index 0..18 maps to equipment slots 1..19 (WoW convention: slot 0 unused on server)
    static constexpr const char* kSlotNames[19] = {
        "Head", "Neck", "Shoulder", "Shirt", "Chest",
        "Waist", "Legs", "Feet", "Wrist", "Hands",
        "Finger 1", "Finger 2", "Trinket 1", "Trinket 2", "Back",
        "Main Hand", "Off Hand", "Ranged", "Tabard"
    };

    ImGui::SetNextWindowSize(ImVec2(360, 440), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(350, 120), ImGuiCond_FirstUseEver);

    const game::GameHandler::InspectResult* result = gameHandler.getInspectResult();
    const uint64_t targetGuid = gameHandler.getTargetGuid();
    auto target = gameHandler.getTarget();
    const bool targetIsPlayer =
        target && target->getType() == game::ObjectType::PLAYER && targetGuid != 0;
    if (targetIsPlayer &&
        inspectWindowAutoRequestGuid_ != targetGuid &&
        (!result || result->guid != targetGuid)) {
        inspectWindowAutoRequestGuid_ = targetGuid;
        gameHandler.inspectTarget();
        result = gameHandler.getInspectResult();
    }

    std::string title = result ? ("Inspect: " + result->playerName + "###InspectWin")
                                : "Inspect###InspectWin";
    if (!ImGui::Begin(title.c_str(), &showInspectWindow_, ImGuiWindowFlags_NoCollapse)) {
        ImGui::End();
        return;
    }

    if (!result) {
        ImGui::TextDisabled("No inspect data yet. Target a player and use Inspect.");
        ImGui::End();
        return;
    }

    // Player name — class-colored if entity is loaded, else gold
    {
        auto ent = gameHandler.getEntityManager().getEntity(result->guid);
        uint8_t cid = entityClassId(ent.get());
        ImVec4 nameColor = (cid != 0) ? classColorVec4(cid) : ui::colors::kTooltipGold;
        ImGui::PushStyleColor(ImGuiCol_Text, nameColor);
        ImGui::Text("%s", result->playerName.c_str());
        ImGui::PopStyleColor();
        if (cid != 0) {
            ImGui::SameLine();
            ImGui::TextColored(classColorVec4(cid), "(%s)", classNameStr(cid));
        }
    }

    ImGui::Separator();

    // Equipment list
    bool hasAnyGear = false;
    for (int s = 0; s < 19; ++s) {
        if (result->itemEntries[s] != 0) { hasAnyGear = true; break; }
    }
    const auto* visibleEquipment = game::isActiveExpansion("tbc")
        ? gameHandler.getOtherPlayerVisibleEquipment(result->guid)
        : nullptr;
    bool hasVisibleEquipment = false;
    if (visibleEquipment) {
        for (uint32_t displayId : *visibleEquipment) {
            if (displayId != 0) {
                hasVisibleEquipment = true;
                break;
            }
        }
    }

    struct InspectGearSlot {
        uint32_t entry = 0;
        uint32_t displayId = 0;
        uint16_t enchantId = 0;
        const game::ItemQueryResponseData* info = nullptr;
        bool loading = false;
    };

    const bool usingVisibleFallback = !hasAnyGear && hasVisibleEquipment;
    std::array<InspectGearSlot, 19> gearSlots{};

    for (int s = 0; s < 19; ++s) {
        if (hasAnyGear) {
            const uint32_t entry = result->itemEntries[s];
            if (entry == 0) continue;
            gearSlots[s].entry = entry;
            gearSlots[s].enchantId = result->enchantIds[s];
            gearSlots[s].info = gameHandler.getItemInfo(entry);
            if (gearSlots[s].info) {
                gearSlots[s].displayId = gearSlots[s].info->displayInfoId;
            } else {
                gearSlots[s].loading = true;
                gameHandler.ensureItemInfo(entry);
            }
        } else if (usingVisibleFallback) {
            const uint32_t entry = (*visibleEquipment)[s];
            if (entry == 0) continue;
            gearSlots[s].entry = entry;
            gearSlots[s].info = gameHandler.getItemInfo(entry);
            if (gearSlots[s].info) {
                gearSlots[s].displayId = gearSlots[s].info->displayInfoId;
            } else {
                gearSlots[s].loading = true;
                gameHandler.ensureItemInfo(entry);
            }
        }
    }

    auto renderInspectSlot = [&](int slotIndex, float size) {
        const auto& slot = gearSlots[slotIndex];
        const bool empty = slot.entry == 0 && slot.displayId == 0;
        const char* label = kSlotNames[slotIndex];
        ImDrawList* drawList = ImGui::GetWindowDrawList();
        ImVec2 pos = ImGui::GetCursorScreenPos();

        ImVec4 qColor = slot.info
            ? InventoryScreen::getQualityColor(static_cast<game::ItemQuality>(slot.info->quality))
            : ImVec4(0.45f, 0.48f, 0.56f, 1.0f);
        ImU32 borderCol = empty
            ? IM_COL32(70, 70, 80, 190)
            : ImGui::ColorConvertFloat4ToU32(qColor);
        ImU32 bgCol = empty ? IM_COL32(25, 25, 32, 190) : IM_COL32(40, 35, 30, 220);

        VkDescriptorSet iconTex = (!empty && slot.displayId != 0)
            ? inventoryScreen.getItemIcon(slot.displayId)
            : VK_NULL_HANDLE;
        if (iconTex) {
            drawList->AddImage((ImTextureID)(uintptr_t)iconTex, pos,
                               ImVec2(pos.x + size, pos.y + size));
            drawList->AddRect(pos, ImVec2(pos.x + size, pos.y + size),
                              borderCol, 0.0f, 0, 2.0f);
        } else {
            drawList->AddRectFilled(pos, ImVec2(pos.x + size, pos.y + size), bgCol);
            drawList->AddRect(pos, ImVec2(pos.x + size, pos.y + size),
                              borderCol, 0.0f, 0, empty ? 1.0f : 2.0f);

            char abbr[4] = {};
            if (slot.loading) {
                abbr[0] = '.';
                abbr[1] = '.';
            } else if (slot.info && !slot.info->name.empty()) {
                abbr[0] = slot.info->name[0];
                if (slot.info->name.size() > 1) abbr[1] = slot.info->name[1];
            } else {
                abbr[0] = label[0];
                if (label[1]) abbr[1] = label[1];
            }
            float textW = ImGui::CalcTextSize(abbr).x;
            drawList->AddText(ImVec2(pos.x + (size - textW) * 0.5f, pos.y + size * 0.3f),
                              empty ? IM_COL32(85, 85, 95, 180) : borderCol, abbr);
        }

        if (slot.enchantId != 0) {
            drawList->AddText(ImVec2(pos.x + size - 10.0f, pos.y + 1.0f),
                              IM_COL32(150, 220, 255, 240), "*");
        }

        ImGui::InvisibleButton("slot", ImVec2(size, size));
        if (ImGui::IsItemHovered()) {
            if (slot.info && slot.info->valid) {
                inventoryScreen.renderItemTooltip(*slot.info);
            } else {
                ImGui::BeginTooltip();
                ImGui::TextDisabled("%s", label);
                if (slot.entry != 0) {
                    ImGui::Text("Item #%u", static_cast<unsigned>(slot.entry));
                    ImGui::TextDisabled("Loading item details...");
                } else if (slot.displayId != 0) {
                    ImGui::Text("Display ID %u", static_cast<unsigned>(slot.displayId));
                } else {
                    ImGui::TextDisabled("Empty");
                }
                ImGui::EndTooltip();
            }
        }
    };

    auto renderInspectPaperDoll = [&]() {
        static constexpr int leftSlots[] = {0, 1, 2, 14, 4, 3, 18, 8};
        static constexpr int rightSlots[] = {9, 5, 6, 7, 10, 11, 12, 13};
        static constexpr int weaponSlots[] = {15, 16, 17};
        constexpr float slotSize = 36.0f;
        constexpr float previewW = 140.0f;

        ImGui::TextColored(ui::colors::kWarmGold, "Equipment");
        if (usingVisibleFallback) {
            ImGui::SameLine();
            ImGui::TextDisabled("(visible)");
            if (ImGui::IsItemHovered()) {
                ImGui::BeginTooltip();
                ImGui::TextDisabled("Showing public visible equipment fields.");
                ImGui::EndTooltip();
            }
        }
        ImGui::Separator();

        float contentStartX = ImGui::GetCursorPosX();
        float rightColX = contentStartX + slotSize + 8.0f + previewW + 8.0f;
        float previewStartY = ImGui::GetCursorScreenPos().y;

        for (int r = 0; r < 8; ++r) {
            ImGui::PushID(leftSlots[r]);
            renderInspectSlot(leftSlots[r], slotSize);
            ImGui::PopID();

            ImGui::SameLine(rightColX);
            ImGui::PushID(rightSlots[r]);
            renderInspectSlot(rightSlots[r], slotSize);
            ImGui::PopID();
        }

        float previewEndY = ImGui::GetCursorScreenPos().y;
        float previewX = ImGui::GetWindowPos().x + contentStartX + slotSize + 8.0f;
        float previewH = previewEndY - previewStartY;
        ImVec2 pMin(previewX, previewStartY);
        ImVec2 pMax(previewX + previewW, previewStartY + previewH);
        ImDrawList* drawList = ImGui::GetWindowDrawList();
        drawList->AddRectFilled(pMin, pMax, IM_COL32(13, 13, 25, 210));
        drawList->AddRect(pMin, pMax, IM_COL32(60, 60, 80, 200));

        std::string centerName = result->playerName;
        ImVec2 nameSize = ImGui::CalcTextSize(centerName.c_str());
        drawList->AddText(ImVec2(pMin.x + (previewW - nameSize.x) * 0.5f, pMin.y + 14.0f),
                          IM_COL32(220, 220, 235, 230), centerName.c_str());
        auto ent = gameHandler.getEntityManager().getEntity(result->guid);
        uint8_t cid = entityClassId(ent.get());
        if (cid != 0) {
            const char* cls = classNameStr(cid);
            ImVec2 classSize = ImGui::CalcTextSize(cls);
            drawList->AddText(ImVec2(pMin.x + (previewW - classSize.x) * 0.5f, pMin.y + 34.0f),
                              ImGui::ColorConvertFloat4ToU32(classColorVec4(cid)), cls);
        }
        const char* sourceText = usingVisibleFallback ? "Visible gear" : "Inspect gear";
        ImVec2 srcSize = ImGui::CalcTextSize(sourceText);
        drawList->AddText(ImVec2(pMin.x + (previewW - srcSize.x) * 0.5f, pMax.y - 24.0f),
                          IM_COL32(150, 150, 165, 210), sourceText);

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::SetCursorPosX(contentStartX + slotSize + 8.0f);
        for (int i = 0; i < 3; ++i) {
            if (i > 0) ImGui::SameLine();
            ImGui::PushID(weaponSlots[i]);
            renderInspectSlot(weaponSlots[i], slotSize);
            ImGui::PopID();
        }
    };

    if (!hasAnyGear && !hasVisibleEquipment) {
        ImGui::TextDisabled("Equipment data not yet available.");
        ImGui::TextDisabled("(Gear loads after the player is inspected in-range)");
    } else {
        // Average item level (only slots that have loaded info and are not shirt/tabard)
        // Shirt=slot3, Tabard=slot18 — excluded from gear score by WoW convention
        uint32_t iLevelSum = 0;
        int iLevelCount = 0;
        for (int s = 0; s < 19; ++s) {
            if (s == 3 || s == 18) continue; // shirt, tabard
            uint32_t entry = result->itemEntries[s];
            if (entry == 0) continue;
            const game::ItemQueryResponseData* info = gameHandler.getItemInfo(entry);
            if (info && info->valid && info->itemLevel > 0) {
                iLevelSum += info->itemLevel;
                ++iLevelCount;
            }
        }
        if (iLevelCount > 0) {
            float avgIlvl = static_cast<float>(iLevelSum) / static_cast<float>(iLevelCount);
            ImGui::TextColored(ImVec4(0.8f, 0.9f, 1.0f, 1.0f), "Avg iLvl: %.1f", avgIlvl);
            ImGui::SameLine();
            ImGui::TextDisabled("(%d/%d slots loaded)", iLevelCount,
                [&]{ int c=0; for(int s=0;s<19;++s){if(s==3||s==18)continue;if(result->itemEntries[s])++c;} return c; }());
        }
        renderInspectPaperDoll();
    }

    // Arena teams (WotLK — from MSG_INSPECT_ARENA_TEAMS)
    if (!result->arenaTeams.empty()) {
        ImGui::Separator();
        ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.2f, 1.0f), "Arena Teams");
        ImGui::Spacing();
        for (const auto& team : result->arenaTeams) {
            const char* bracket = (team.type == 2) ? "2v2"
                                : (team.type == 3) ? "3v3"
                                : (team.type == 5) ? "5v5" : "?v?";
            ImGui::TextColored(ImVec4(0.9f, 0.9f, 0.9f, 1.0f),
                               "[%s]  %s", bracket, team.name.c_str());
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.4f, 0.85f, 1.0f, 1.0f),
                               "  Rating: %u", team.personalRating);
            if (team.weekGames > 0 || team.seasonGames > 0) {
                ImGui::TextDisabled("    Week: %u/%u  Season: %u/%u",
                                    team.weekWins, team.weekGames,
                                    team.seasonWins, team.seasonGames);
            }
        }
    }

    ImGui::End();
}

} // namespace ui
} // namespace wowee
