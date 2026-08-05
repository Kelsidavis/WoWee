#include "ui/game_screen.hpp"
#include "ui/ui_raid_icons.hpp"
#include "ui/ui_colors.hpp"
#include "ui/ui_helpers.hpp"
#include "rendering/vk_context.hpp"
#include "core/application.hpp"
#include "core/appearance_composer.hpp"
#include "addons/addon_manager.hpp"
#include "core/coordinates.hpp"
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
#include "audio/player_voice_manager.hpp"
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
#include <glm/gtc/constants.hpp>
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

    bool raySphereIntersect(const wowee::rendering::Ray& ray, const glm::vec3& center, float radius, float& tOut) {
        glm::vec3 oc = ray.origin - center;
        float b = glm::dot(oc, ray.direction);
        float c = glm::dot(oc, oc) - radius * radius;
        float discriminant = b * b - c;
        if (discriminant < 0.0f) return false;
        float t = -b - std::sqrt(discriminant);
        if (t < 0.0f) t = -b + std::sqrt(discriminant);
        if (t < 0.0f) return false;
        tOut = t;
        return true;
    }

    std::string getEntityName(const std::shared_ptr<wowee::game::Entity>& entity) {
        if (entity->getType() == wowee::game::ObjectType::PLAYER) {
            auto player = std::static_pointer_cast<wowee::game::Player>(entity);
            if (!player->getName().empty()) return player->getName();
        } else if (entity->getType() == wowee::game::ObjectType::UNIT) {
            auto unit = std::static_pointer_cast<wowee::game::Unit>(entity);
            if (!unit->getName().empty()) return unit->getName();
        } else if (entity->getType() == wowee::game::ObjectType::GAMEOBJECT) {
            auto go = std::static_pointer_cast<wowee::game::GameObject>(entity);
            if (!go->getName().empty()) return go->getName();
        }
        return "Unknown";
    }

}

namespace wowee { namespace ui {

void GameScreen::refreshQuestObjectiveCache(game::GameHandler& gameHandler) {
    uint64_t signature = 1469598103934665603ull;
    auto mix = [&](uint64_t value) {
        signature ^= value;
        signature *= 1099511628211ull;
    };
    const auto& mapVisible = gameHandler.getMapVisibleQuestIds();
    mix(mapVisible.size());
    for (const auto& quest : gameHandler.getQuestLog()) {
        if (quest.complete || quest.questId == 0 ||
            !mapVisible.count(quest.questId)) continue;
        mix(quest.questId);
        for (const auto& objective : quest.killObjectives) {
            if (objective.required == 0) continue;
            const uint32_t entry = static_cast<uint32_t>(objective.npcOrGoId > 0
                ? objective.npcOrGoId : -objective.npcOrGoId);
            auto count = quest.killCounts.find(entry);
            mix(static_cast<uint32_t>(objective.npcOrGoId));
            mix(objective.required);
            mix(count == quest.killCounts.end() ? 0 : count->second.first);
        }
    }
    if (signature == minimapQuestCacheSignature_) return;

    minimapQuestCacheSignature_ = signature;
    minimapQuestCreatureEntries_.clear();
    minimapQuestGameObjectEntries_.clear();
    for (const auto& quest : gameHandler.getQuestLog()) {
        if (quest.complete || quest.questId == 0 ||
            !mapVisible.count(quest.questId)) continue;
        for (const auto& objective : quest.killObjectives) {
            if (objective.required == 0 || objective.npcOrGoId == 0) continue;
            const uint32_t entry = static_cast<uint32_t>(objective.npcOrGoId > 0
                ? objective.npcOrGoId : -objective.npcOrGoId);
            auto count = quest.killCounts.find(entry);
            if (count != quest.killCounts.end() && count->second.first >= count->second.second) continue;
            if (objective.npcOrGoId > 0) minimapQuestCreatureEntries_.insert(entry);
            else minimapQuestGameObjectEntries_.insert(entry);
        }
    }
}

void GameScreen::renderMinimapMarkers(game::GameHandler& gameHandler) {
    const auto& statuses = gameHandler.getNpcQuestStatuses();
    auto* renderer = services_.renderer;
    auto* camera = renderer ? renderer->getCamera() : nullptr;
    auto* minimap = renderer ? renderer->getMinimap() : nullptr;
    auto* window = services_.window;
    if (!camera || !minimap || !window) return;

    float screenW = static_cast<float>(window->getWidth());

    // Minimap parameters (matching minimap.cpp)
    float mapSize = 200.0f;
    float margin = 10.0f;
    float mapRadius = mapSize * 0.5f;
    float centerX = screenW - margin - mapRadius;
    float centerY = margin + mapRadius;
    float viewRadius = minimap->getViewRadius();

    // Use the exact same minimap center as Renderer::renderWorld() to keep markers anchored.
    glm::vec3 playerRender = camera->getPosition();
    if (renderer->getCharacterInstanceId() != 0) {
        playerRender = renderer->getCharacterPosition();
    }

    // Camera bearing for minimap rotation
    float bearing = 0.0f;
    float cosB = 1.0f;
    float sinB = 0.0f;
    if (minimap->isRotateWithCamera()) {
        glm::vec3 fwd = camera->getForward();
        // Render space: +X=West, +Y=North. Camera fwd=(cos(yaw),sin(yaw)).
        // Clockwise bearing from North: atan2(fwd.y, -fwd.x).
        bearing = std::atan2(fwd.y, -fwd.x);
        cosB = std::cos(bearing);
        sinB = std::sin(bearing);
    }

    auto* drawList = ImGui::GetForegroundDrawList();

    // Partition the entity map once. Marker categories below can traverse their
    // compact type-specific lists instead of rescanning every entity 4-5 times.
    // Reused across frames to avoid three vector allocations per frame; the
    // clear below releases the previous frame's shared_ptrs on re-entry.
    static thread_local std::vector<std::shared_ptr<game::Entity>> minimapUnits;
    static thread_local std::vector<std::shared_ptr<game::Entity>> minimapPlayers;
    static thread_local std::vector<std::shared_ptr<game::Entity>> minimapGameObjects;
    minimapUnits.clear();
    minimapPlayers.clear();
    minimapGameObjects.clear();
    const auto& allEntities = gameHandler.getEntityManager().getEntities();
    minimapUnits.reserve(allEntities.size() / 2);
    minimapPlayers.reserve(allEntities.size() / 8);
    minimapGameObjects.reserve(allEntities.size() / 4);
    for (const auto& [guid, entity] : allEntities) {
        (void)guid;
        if (!entity) continue;
        switch (entity->getType()) {
            case game::ObjectType::UNIT:       minimapUnits.push_back(entity); break;
            case game::ObjectType::PLAYER:     minimapPlayers.push_back(entity); break;
            case game::ObjectType::GAMEOBJECT: minimapGameObjects.push_back(entity); break;
            default: break;
        }
    }

    auto projectToMinimap = [&](const glm::vec3& worldRenderPos, float& sx, float& sy) -> bool {
        float dx = worldRenderPos.x - playerRender.x;
        float dy = worldRenderPos.y - playerRender.y;

        // Exact inverse of minimap display shader:
        //   shader: mapUV = playerUV + vec2(rotated.y, -rotated.x) * zoom * 2
        //   where rotated = R(bearing) * vec2(-center.x, center.y).
        // Render +X is west and +Y is north, while composite UV grows east/south.
        float rx = -(dy * cosB - dx * sinB);
        float ry = -(dy * sinB + dx * cosB);

        // Scale to minimap pixels
        float px = rx / viewRadius * mapRadius;
        float py = ry / viewRadius * mapRadius;

        float distFromCenter = std::sqrt(px * px + py * py);
        if (distFromCenter > mapRadius - 3.0f) {
            return false;
        }

        sx = centerX + px;
        sy = centerY + py;
        return true;
    };

    // Build sets of entries that are incomplete objectives for tracked quests.
    // minimapQuestEntries: NPC creature entries (npcOrGoId > 0)
    // minimapQuestGoEntries: game object entries (npcOrGoId < 0, stored as abs value)
    refreshQuestObjectiveCache(gameHandler);
    const auto& minimapQuestEntries = minimapQuestCreatureEntries_;
    const auto& minimapQuestGoEntries = minimapQuestGameObjectEntries_;

    // Optional base nearby NPC dots (independent of quest status packets).
    if (settingsPanel_.minimapNpcDots_) {
        ImVec2 mouse = ImGui::GetMousePos();
        for (const auto& entity : minimapUnits) {

            auto unit = std::static_pointer_cast<game::Unit>(entity);
            if (!unit || unit->getHealth() == 0) continue;

            glm::vec3 npcRender = core::coords::canonicalToRender(glm::vec3(entity->getX(), entity->getY(), entity->getZ()));
            float sx = 0.0f, sy = 0.0f;
            if (!projectToMinimap(npcRender, sx, sy)) continue;

            bool isQuestTarget = minimapQuestEntries.count(unit->getEntry()) != 0;
            if (isQuestTarget) {
                // Quest kill objective: larger gold dot with dark outline
                drawList->AddCircleFilled(ImVec2(sx, sy), 3.5f, IM_COL32(255, 210, 30, 240));
                drawList->AddCircle(ImVec2(sx, sy), 3.5f, IM_COL32(80, 50, 0, 180), 0, 1.0f);
                // Tooltip on hover showing unit name
                float mdx = mouse.x - sx, mdy = mouse.y - sy;
                if (mdx * mdx + mdy * mdy < 64.0f) {
                    const std::string& nm = unit->getName();
                    if (!nm.empty()) ImGui::SetTooltip("%s (quest)", nm.c_str());
                }
            } else {
                ImU32 baseDot = unit->isHostile() ? IM_COL32(220, 70, 70, 220) : IM_COL32(245, 245, 245, 210);
                drawList->AddCircleFilled(ImVec2(sx, sy), 1.0f, baseDot);
            }
        }
    }

    // Flight masters are a standard minimap service marker, independent of
    // the optional generic NPC-dot overlay. Use the live UNIT_NPC_FLAGS value
    // so an undiscovered flight master is visible before the taxi window has
    // ever been opened (and therefore before the known-node mask is available).
    {
        ImVec2 mouse = ImGui::GetMousePos();
        for (const auto& entity : minimapUnits) {
            auto unit = std::static_pointer_cast<game::Unit>(entity);
            if (!unit || unit->getHealth() == 0 ||
                (unit->getNpcFlags() & game::NPC_FLAG_FLIGHT_MASTER) == 0) {
                continue;
            }

            glm::vec3 flightMasterRender = core::coords::canonicalToRender(
                glm::vec3(entity->getX(), entity->getY(), entity->getZ()));
            float sx = 0.0f, sy = 0.0f;
            if (!projectToMinimap(flightMasterRender, sx, sy)) continue;

            constexpr float halfSize = 5.5f;
            const ImVec2 top(sx, sy - halfSize);
            const ImVec2 right(sx + halfSize, sy);
            const ImVec2 bottom(sx, sy + halfSize);
            const ImVec2 left(sx - halfSize, sy);
            drawList->AddQuadFilled(top, right, bottom, left,
                                    IM_COL32(255, 215, 0, 245));
            drawList->AddQuad(top, right, bottom, left,
                              IM_COL32(70, 45, 0, 230), 1.5f);
            drawList->AddCircleFilled(ImVec2(sx, sy), 1.7f,
                                      IM_COL32(255, 250, 205, 255));

            float mdx = mouse.x - sx, mdy = mouse.y - sy;
            if (mdx * mdx + mdy * mdy <= 64.0f) {
                const std::string& name = unit->getName();
                ImGui::SetTooltip("%s\nFlight Master",
                                  name.empty() ? "Flight Master" : name.c_str());
            }
        }
    }

    // Rare tracker: use the same live creature-rank classification as the world map.
    // This is independent of generic NPC dots so enabling rare tracking consistently
    // shows spawned rares on both maps without adding every nearby creature.
    if (settingsPanel_.showRareTracker_) {
        ImVec2 mouse = ImGui::GetMousePos();
        for (const auto& entity : minimapUnits) {
            auto unit = std::static_pointer_cast<game::Unit>(entity);
            if (!unit || unit->getHealth() == 0) continue;

            const int rank = gameHandler.getCreatureRank(unit->getEntry());
            if (rank != 2 && rank != 4) continue; // 2 = Rare Elite, 4 = Rare

            glm::vec3 rareRender = core::coords::canonicalToRender(
                glm::vec3(entity->getX(), entity->getY(), entity->getZ()));
            float sx = 0.0f, sy = 0.0f;
            if (!projectToMinimap(rareRender, sx, sy)) continue;

            // Match the world-map tracker: gold for Rare, silver for Rare Elite.
            const bool isElite = rank == 2;
            const ImU32 fill = isElite
                ? IM_COL32(210, 210, 225, 255)
                : IM_COL32(255, 190, 60, 255);
            constexpr float halfSize = 5.0f;
            const ImVec2 top(sx, sy - halfSize);
            const ImVec2 right(sx + halfSize, sy);
            const ImVec2 bottom(sx, sy + halfSize);
            const ImVec2 left(sx - halfSize, sy);
            drawList->AddQuadFilled(top, right, bottom, left, fill);
            drawList->AddQuad(top, right, bottom, left, IM_COL32(0, 0, 0, 220), 1.5f);

            float mdx = mouse.x - sx, mdy = mouse.y - sy;
            if (mdx * mdx + mdy * mdy <= 49.0f) {
                const std::string& name = unit->getName();
                ImGui::SetTooltip("%s\n%s",
                                  name.empty() ? "Unknown creature" : name.c_str(),
                                  isElite ? "Rare Elite" : "Rare");
            }
        }
    }

    // Nearby other-player dots — shown when NPC dots are enabled.
    // Party members are already drawn as squares above; other players get a small circle.
    if (settingsPanel_.minimapNpcDots_) {
        const uint64_t selfGuid = gameHandler.getPlayerGuid();
        const auto& partyData = gameHandler.getPartyData();
        for (const auto& entity : minimapPlayers) {
            const uint64_t guid = entity->getGuid();
            if (entity->getGuid() == selfGuid) continue;  // skip self (already drawn as arrow)

            // Skip party members (already drawn as squares above)
            bool isPartyMember = false;
            for (const auto& m : partyData.members) {
                if (m.guid == guid) { isPartyMember = true; break; }
            }
            if (isPartyMember) continue;

            glm::vec3 pRender = core::coords::canonicalToRender(
                glm::vec3(entity->getX(), entity->getY(), entity->getZ()));
            float sx = 0.0f, sy = 0.0f;
            if (!projectToMinimap(pRender, sx, sy)) continue;

            // Blue dot for other nearby players
            drawList->AddCircleFilled(ImVec2(sx, sy), 2.0f, IM_COL32(80, 160, 255, 220));
        }
    }

    // Lootable corpse dots: small yellow-green diamonds on dead, lootable units.
    // Shown whenever NPC dots are enabled (or always, since they're always useful).
    {
        for (const auto& entity : minimapUnits) {
            auto unit = std::static_pointer_cast<game::Unit>(entity);
            if (!unit) continue;
            // Must be dead (health == 0) and marked lootable
            if (unit->getHealth() != 0) continue;
            if (!(unit->getDynamicFlags() & game::UNIT_DYNFLAG_LOOTABLE)) continue;

            glm::vec3 npcRender = core::coords::canonicalToRender(
                glm::vec3(entity->getX(), entity->getY(), entity->getZ()));
            float sx = 0.0f, sy = 0.0f;
            if (!projectToMinimap(npcRender, sx, sy)) continue;

            // Draw a small diamond (rotated square) in light yellow-green
            const float dr = 3.5f;
            ImVec2 top  (sx,      sy - dr);
            ImVec2 right(sx + dr, sy     );
            ImVec2 bot  (sx,      sy + dr);
            ImVec2 left (sx - dr, sy     );
            drawList->AddQuadFilled(top, right, bot, left, IM_COL32(180, 230, 80, 230));
            drawList->AddQuad      (top, right, bot, left, IM_COL32(60,  80,  20, 200), 1.0f);

            // Tooltip on hover
            if (ImGui::IsMouseHoveringRect(ImVec2(sx - dr, sy - dr), ImVec2(sx + dr, sy + dr))) {
                const std::string& nm = unit->getName();
                ImGui::BeginTooltip();
                ImGui::TextColored(ImVec4(0.7f, 0.9f, 0.3f, 1.0f), "%s",
                                   nm.empty() ? "Lootable corpse" : nm.c_str());
                ImGui::EndTooltip();
            }
        }
    }

    // Interactable game object dots (chests, resource nodes) when NPC dots are enabled.
    // Shown as small orange triangles to distinguish from unit dots and loot corpses.
    if (settingsPanel_.minimapNpcDots_) {
        ImVec2 mouse = ImGui::GetMousePos();
        for (const auto& entity : minimapGameObjects) {

            // Only show objects that are likely interactive (chests/nodes: type 3;
            // also show type 0=Door when open, but filter by dynamic-flag ACTIVATED).
            // For simplicity, show all game objects that have a non-empty cached name.
            auto go = std::static_pointer_cast<game::GameObject>(entity);
            if (!go) continue;

            // Only show if we have name data (avoids cluttering with unknown objects)
            const auto* goInfo = gameHandler.getCachedGameObjectInfo(go->getEntry());
            if (!goInfo || !goInfo->isValid()) continue;
            // Skip transport objects (boats/zeppelins): type 15 = MO_TRANSPORT, 11 = TRANSPORT
            if (goInfo->type == 11 || goInfo->type == 15) continue;

            glm::vec3 goRender = core::coords::canonicalToRender(
                glm::vec3(entity->getX(), entity->getY(), entity->getZ()));
            float sx = 0.0f, sy = 0.0f;
            if (!projectToMinimap(goRender, sx, sy)) continue;

            // Triangle size and color: bright cyan for quest objectives, amber for others
            bool isQuestGO = minimapQuestGoEntries.count(go->getEntry()) != 0;
            const float ts = isQuestGO ? 4.5f : 3.5f;
            ImVec2 goTip  (sx,        sy - ts);
            ImVec2 goLeft (sx - ts,   sy + ts * 0.6f);
            ImVec2 goRight(sx + ts,   sy + ts * 0.6f);
            if (isQuestGO) {
                drawList->AddTriangleFilled(goTip, goLeft, goRight, IM_COL32(50, 230, 255, 240));
                drawList->AddTriangle(goTip, goLeft, goRight, IM_COL32(0, 60, 80, 200), 1.5f);
            } else {
                drawList->AddTriangleFilled(goTip, goLeft, goRight, IM_COL32(255, 185, 30, 220));
                drawList->AddTriangle(goTip, goLeft, goRight, IM_COL32(100, 60, 0, 180), 1.0f);
            }

            // Tooltip on hover
            float mdx = mouse.x - sx, mdy = mouse.y - sy;
            if (mdx * mdx + mdy * mdy < 64.0f) {
                if (isQuestGO)
                    ImGui::SetTooltip("%s (quest)", goInfo->name.c_str());
                else
                    ImGui::SetTooltip("%s", goInfo->name.c_str());
            }
        }
    }

    // Chest tracker: GAMEOBJECT_TYPE_CHEST also covers gathering nodes on WoW
    // servers, so explicitly exclude the existing mining/herbalism classification.
    if (settingsPanel_.showChestTracker_) {
        ImVec2 mouse = ImGui::GetMousePos();
        for (const auto& entity : minimapGameObjects) {
            auto chest = std::static_pointer_cast<game::GameObject>(entity);
            if (!chest) continue;
            const auto* info = gameHandler.getCachedGameObjectInfo(chest->getEntry());
            if (!info || !info->isValid() || info->type != 3) continue;
            if (gameHandler.isGatherGameObject(chest->getGuid())) continue;

            glm::vec3 chestRender = core::coords::canonicalToRender(
                glm::vec3(entity->getX(), entity->getY(), entity->getZ()));
            float sx = 0.0f, sy = 0.0f;
            if (!projectToMinimap(chestRender, sx, sy)) continue;

            constexpr float halfW = 5.5f;
            constexpr float halfH = 4.0f;
            constexpr ImU32 fill = IM_COL32(205, 125, 35, 255);
            constexpr ImU32 outline = IM_COL32(45, 25, 5, 230);
            drawList->AddRectFilled(ImVec2(sx - halfW, sy - halfH),
                                    ImVec2(sx + halfW, sy + halfH), fill, 1.5f);
            drawList->AddRect(ImVec2(sx - halfW, sy - halfH),
                              ImVec2(sx + halfW, sy + halfH), outline, 1.5f, 0, 1.5f);
            drawList->AddLine(ImVec2(sx - halfW, sy - 0.8f),
                              ImVec2(sx + halfW, sy - 0.8f), outline, 1.0f);
            drawList->AddRectFilled(ImVec2(sx - 1.0f, sy - 1.2f),
                                    ImVec2(sx + 1.0f, sy + 1.3f),
                                    IM_COL32(255, 220, 80, 255), 0.5f);

            if (mouse.x >= sx - halfW - 2.0f && mouse.x <= sx + halfW + 2.0f &&
                mouse.y >= sy - halfH - 2.0f && mouse.y <= sy + halfH + 2.0f) {
                const std::string& name = chest->getName().empty() ? info->name : chest->getName();
                ImGui::SetTooltip("%s\nChest", name.empty() ? "Chest" : name.c_str());
            }
        }
    }

    // Party member dots on minimap — small colored squares with name tooltip on hover
    if (gameHandler.isInGroup()) {
        const auto& partyData = gameHandler.getPartyData();
        ImVec2 mouse = ImGui::GetMousePos();
        for (const auto& member : partyData.members) {
            if (!member.hasPartyStats) continue;
            bool isOnline = (member.onlineStatus & 0x0001) != 0;
            bool isDead   = (member.onlineStatus & 0x0020) != 0;
            bool isGhost  = (member.onlineStatus & 0x0010) != 0;
            if (!isOnline) continue;
            if (member.posX == 0 && member.posY == 0) continue;

            // Party stat positions: posY = canonical X (north), posX = canonical Y (west)
            glm::vec3 memberRender = core::coords::canonicalToRender(
                glm::vec3(static_cast<float>(member.posY),
                          static_cast<float>(member.posX), 0.0f));
            float sx = 0.0f, sy = 0.0f;
            if (!projectToMinimap(memberRender, sx, sy)) continue;

            // Determine dot color: class color > leader gold > light blue
            ImU32 dotCol;
            if (isDead || isGhost) {
                dotCol = IM_COL32(140, 140, 140, 200);  // gray for dead
            } else {
                auto mEnt = gameHandler.getEntityManager().getEntity(member.guid);
                uint8_t cid = entityClassId(mEnt.get());
                if (cid != 0) {
                    ImVec4 cv = classColorVec4(cid);
                    dotCol = IM_COL32(
                        static_cast<int>(cv.x * 255),
                        static_cast<int>(cv.y * 255),
                        static_cast<int>(cv.z * 255), 230);
                } else if (member.guid == partyData.leaderGuid) {
                    dotCol = IM_COL32(255, 210, 0, 230);  // gold for leader
                } else {
                    dotCol = IM_COL32(100, 180, 255, 230); // blue for others
                }
            }

            // Draw a small square (WoW-style party member dot)
            const float hs = 3.5f;
            drawList->AddRectFilled(ImVec2(sx - hs, sy - hs), ImVec2(sx + hs, sy + hs), dotCol, 1.0f);
            drawList->AddRect(ImVec2(sx - hs, sy - hs), ImVec2(sx + hs, sy + hs),
                              IM_COL32(0, 0, 0, 180), 1.0f, 0, 1.0f);

            // Name tooltip on hover
            float mdx = mouse.x - sx, mdy = mouse.y - sy;
            if (mdx * mdx + mdy * mdy < 64.0f && !member.name.empty()) {
                ImGui::SetTooltip("%s", member.name.c_str());
            }
        }
    }

    for (const auto& [guid, status] : statuses) {
        ImU32 dotColor;
        const char* marker = nullptr;
        if (status == game::QuestGiverStatus::AVAILABLE) {
            dotColor = IM_COL32(255, 210, 0, 255);
            marker = "!";
        } else if (status == game::QuestGiverStatus::AVAILABLE_LOW) {
            dotColor = IM_COL32(160, 160, 160, 255);
            marker = "!";
        } else if (status == game::QuestGiverStatus::REWARD ||
                   status == game::QuestGiverStatus::REWARD_REP) {
            dotColor = IM_COL32(255, 210, 0, 255);
            marker = "?";
        } else if (status == game::QuestGiverStatus::INCOMPLETE) {
            dotColor = IM_COL32(160, 160, 160, 255);
            marker = "?";
        } else {
            continue;
        }

        auto entity = gameHandler.getEntityManager().getEntity(guid);
        if (!entity) continue;

        glm::vec3 canonical(entity->getX(), entity->getY(), entity->getZ());
        glm::vec3 npcRender = core::coords::canonicalToRender(canonical);

        float sx = 0.0f, sy = 0.0f;
        if (!projectToMinimap(npcRender, sx, sy)) continue;

        // Draw dot with marker text
        drawList->AddCircleFilled(ImVec2(sx, sy), 5.0f, dotColor);
        ImFont* font = ImGui::GetFont();
        ImVec2 textSize = font->CalcTextSizeA(11.0f, FLT_MAX, 0.0f, marker);
        drawList->AddText(font, 11.0f,
            ImVec2(sx - textSize.x * 0.5f, sy - textSize.y * 0.5f),
            IM_COL32(0, 0, 0, 255), marker);

        // Show NPC name and quest status on hover
        {
            ImVec2 mouse = ImGui::GetMousePos();
            float mdx = mouse.x - sx, mdy = mouse.y - sy;
            if (mdx * mdx + mdy * mdy < 64.0f) {
                std::string npcName;
                if (entity->getType() == game::ObjectType::UNIT) {
                    auto npcUnit = std::static_pointer_cast<game::Unit>(entity);
                    npcName = npcUnit->getName();
                }
                if (!npcName.empty()) {
                    bool hasQuest = (status == game::QuestGiverStatus::AVAILABLE ||
                                     status == game::QuestGiverStatus::AVAILABLE_LOW);
                    ImGui::SetTooltip("%s\n%s", npcName.c_str(),
                                      hasQuest ? "Has a quest for you" : "Quest ready to turn in");
                }
            }
        }
    }

    // Quest kill objective markers — highlight live NPCs matching active quest kill objectives
    {
        // Build map of NPC entry → (quest title, current, required) for tooltips
        struct KillInfo { std::string questTitle; uint32_t current = 0; uint32_t required = 0; };
        std::unordered_map<uint32_t, KillInfo> killInfoMap;
        const auto& mapVisibleIds = gameHandler.getMapVisibleQuestIds();
        for (const auto& quest : gameHandler.getQuestLog()) {
            if (quest.complete) continue;
            if (!mapVisibleIds.count(quest.questId)) continue;
            for (const auto& obj : quest.killObjectives) {
                if (obj.npcOrGoId <= 0 || obj.required == 0) continue;
                uint32_t npcEntry = static_cast<uint32_t>(obj.npcOrGoId);
                auto it = quest.killCounts.find(npcEntry);
                uint32_t current = (it != quest.killCounts.end()) ? it->second.first : 0;
                if (current < obj.required) {
                    killInfoMap[npcEntry] = { quest.title, current, obj.required };
                }
            }
        }

        if (!killInfoMap.empty()) {
            ImVec2 mouse = ImGui::GetMousePos();
            for (const auto& entity : minimapUnits) {
                auto unit = std::static_pointer_cast<game::Unit>(entity);
                if (!unit || unit->getHealth() == 0) continue;
                // A quest giver/turn-in marker is more specific than an objective
                // marker at the same NPC. Do not paint the objective X over ! or ?.
                if (statuses.find(entity->getGuid()) != statuses.end()) continue;
                auto infoIt = killInfoMap.find(unit->getEntry());
                if (infoIt == killInfoMap.end()) continue;

                glm::vec3 unitRender = core::coords::canonicalToRender(
                    glm::vec3(entity->getX(), entity->getY(), entity->getZ()));
                float sx = 0.0f, sy = 0.0f;
                if (!projectToMinimap(unitRender, sx, sy)) continue;

                // Gold circle with a dark "x" mark — indicates a quest kill target
                drawList->AddCircleFilled(ImVec2(sx, sy), 5.0f, IM_COL32(255, 185, 0, 240));
                drawList->AddCircle(ImVec2(sx, sy), 5.5f, IM_COL32(0, 0, 0, 180), 12, 1.0f);
                drawList->AddLine(ImVec2(sx - 2.5f, sy - 2.5f), ImVec2(sx + 2.5f, sy + 2.5f),
                                  IM_COL32(20, 20, 20, 230), 1.2f);
                drawList->AddLine(ImVec2(sx + 2.5f, sy - 2.5f), ImVec2(sx - 2.5f, sy + 2.5f),
                                  IM_COL32(20, 20, 20, 230), 1.2f);

                // Tooltip on hover
                float mdx = mouse.x - sx, mdy = mouse.y - sy;
                if (mdx * mdx + mdy * mdy < 64.0f) {
                    const auto& ki = infoIt->second;
                    const std::string& npcName = unit->getName();
                    if (!npcName.empty()) {
                        ImGui::SetTooltip("%s\n%s: %u/%u",
                            npcName.c_str(),
                            ki.questTitle.empty() ? "Quest" : ki.questTitle.c_str(),
                            ki.current, ki.required);
                    } else {
                        ImGui::SetTooltip("%s: %u/%u",
                            ki.questTitle.empty() ? "Quest" : ki.questTitle.c_str(),
                            ki.current, ki.required);
                    }
                }
            }
        }
    }

    // Gossip POI markers (quest / NPC navigation targets)
    for (const auto& poi : gameHandler.getGossipPois()) {
        if (poi.questObjectiveIndex != -2 &&
            !gameHandler.isQuestShownOnMap(poi.data)) {
            continue;
        }
        // Convert WoW canonical coords to render coords for minimap projection
        glm::vec3 poiRender = core::coords::canonicalToRender(glm::vec3(poi.x, poi.y, 0.0f));
        float sx = 0.0f, sy = 0.0f;
        if (!projectToMinimap(poiRender, sx, sy)) continue;

        // Draw as a cyan diamond with tooltip on hover
        const float d = 5.0f;
        ImVec2 pts[4] = {
            { sx,     sy - d },
            { sx + d, sy     },
            { sx,     sy + d },
            { sx - d, sy     },
        };
        drawList->AddConvexPolyFilled(pts, 4, IM_COL32(0, 210, 255, 220));
        drawList->AddPolyline(pts, 4, IM_COL32(255, 255, 255, 160), true, 1.0f);

        // Show name label if cursor is within ~8px
        ImVec2 cursorPos = ImGui::GetMousePos();
        float dx = cursorPos.x - sx, dy = cursorPos.y - sy;
        if (!poi.name.empty() && (dx * dx + dy * dy) < 64.0f) {
            ImGui::SetTooltip("%s", poi.name.c_str());
        }
    }

    // Minimap pings from party members
    for (const auto& ping : gameHandler.getMinimapPings()) {
        glm::vec3 pingRender = core::coords::canonicalToRender(glm::vec3(ping.wowX, ping.wowY, 0.0f));
        float sx = 0.0f, sy = 0.0f;
        if (!projectToMinimap(pingRender, sx, sy)) continue;

        float t = ping.age / game::GameHandler::MinimapPing::LIFETIME;
        float alpha = 1.0f - t;
        float pulse = 1.0f + 1.5f * t;  // expands outward as it fades

        ImU32 col  = IM_COL32(255, 220, 0, static_cast<int>(alpha * 200));
        ImU32 col2 = IM_COL32(255, 150, 0, static_cast<int>(alpha * 100));
        float r1 = 4.0f * pulse;
        float r2 = 8.0f * pulse;
        drawList->AddCircle(ImVec2(sx, sy), r1, col, 16, 2.0f);
        drawList->AddCircle(ImVec2(sx, sy), r2, col2, 16, 1.0f);
        drawList->AddCircleFilled(ImVec2(sx, sy), 2.5f, col);
    }

    // Party member dots on minimap
    {
        const auto& partyData = gameHandler.getPartyData();
        const uint64_t leaderGuid = partyData.leaderGuid;
        for (const auto& member : partyData.members) {
            if (!member.isOnline || !member.hasPartyStats) continue;
            if (member.posX == 0 && member.posY == 0) continue;

            // posX/posY follow same server axis convention as minimap pings:
            // server posX = east/west axis → canonical Y (west)
            // server posY = north/south axis → canonical X (north)
            float wowX = static_cast<float>(member.posY);
            float wowY = static_cast<float>(member.posX);
            glm::vec3 memberRender = core::coords::canonicalToRender(glm::vec3(wowX, wowY, 0.0f));

            float sx = 0.0f, sy = 0.0f;
            if (!projectToMinimap(memberRender, sx, sy)) continue;

            ImU32 dotColor;
            {
                auto mEnt = gameHandler.getEntityManager().getEntity(member.guid);
                uint8_t cid = entityClassId(mEnt.get());
                dotColor = (cid != 0)
                    ? classColorU32(cid, 235)
                    : (member.guid == leaderGuid)
                        ? IM_COL32(255, 210, 0, 235)
                        : IM_COL32(100, 180, 255, 235);
            }
            drawList->AddCircleFilled(ImVec2(sx, sy), 4.0f, dotColor);
            drawList->AddCircle(ImVec2(sx, sy), 4.0f, IM_COL32(255, 255, 255, 160), 12, 1.0f);

            // Raid mark: the marker artwork drawn small above the dot
            {
                uint8_t pmk = gameHandler.getEntityRaidMark(member.guid);
                if (pmk < game::GameHandler::kRaidMarkCount) {
                    if (VkDescriptorSet markTex = ui::getRaidTargetIcon(pmk, services_.assetManager)) {
                        constexpr float kMarkSize = 10.0f;
                        drawList->AddImage((ImTextureID)(uintptr_t)markTex,
                            ImVec2(sx - kMarkSize * 0.5f, sy - 4.0f - kMarkSize),
                            ImVec2(sx + kMarkSize * 0.5f, sy - 4.0f));
                    }
                }
            }

            ImVec2 cursorPos = ImGui::GetMousePos();
            float mdx = cursorPos.x - sx, mdy = cursorPos.y - sy;
            if (!member.name.empty() && (mdx * mdx + mdy * mdy) < 64.0f) {
                uint8_t pmk2 = gameHandler.getEntityRaidMark(member.guid);
                if (pmk2 < game::GameHandler::kRaidMarkCount) {
                    static constexpr const char* kMarkNames[] = {
                        "Star", "Circle", "Diamond", "Triangle",
                        "Moon", "Square", "Cross", "Skull"
                    };
                    ImGui::SetTooltip("%s {%s}", member.name.c_str(), kMarkNames[pmk2]);
                } else {
                    ImGui::SetTooltip("%s", member.name.c_str());
                }
            }
        }
    }

    // BG flag carrier / important player positions (MSG_BATTLEGROUND_PLAYER_POSITIONS)
    {
        const auto& bgPositions = gameHandler.getBgPlayerPositions();
        if (!bgPositions.empty()) {
            ImVec2 mouse = ImGui::GetMousePos();
            // group 0 = typically ally-held flag / first list; group 1 = enemy
            static const ImU32 kBgGroupColors[2] = {
                IM_COL32( 80, 180, 255, 240),  // group 0: blue (alliance)
                IM_COL32(220,  50,  50, 240),  // group 1: red  (horde)
            };
            for (const auto& bp : bgPositions) {
                // Packet coords: wowX=canonical X (north), wowY=canonical Y (west)
                glm::vec3 bpRender = core::coords::canonicalToRender(glm::vec3(bp.wowX, bp.wowY, 0.0f));
                float sx = 0.0f, sy = 0.0f;
                if (!projectToMinimap(bpRender, sx, sy)) continue;

                ImU32 col = kBgGroupColors[bp.group & 1];

                // Draw a flag-like diamond icon
                const float r = 5.0f;
                ImVec2 top  (sx,       sy - r);
                ImVec2 right(sx + r,   sy    );
                ImVec2 bot  (sx,       sy + r);
                ImVec2 left (sx - r,   sy    );
                drawList->AddQuadFilled(top, right, bot, left, col);
                drawList->AddQuad(top, right, bot, left, IM_COL32(255, 255, 255, 180), 1.0f);

                float mdx = mouse.x - sx, mdy = mouse.y - sy;
                if (mdx * mdx + mdy * mdy < 64.0f) {
                    // Show entity name if available, otherwise guid
                    auto ent = gameHandler.getEntityManager().getEntity(bp.guid);
                    if (ent) {
                        std::string nm;
                        if (ent->getType() == game::ObjectType::PLAYER) {
                            auto pl = std::static_pointer_cast<game::Unit>(ent);
                            nm = pl ? pl->getName() : "";
                        }
                        if (!nm.empty())
                            ImGui::SetTooltip("Flag carrier: %s", nm.c_str());
                        else
                            ImGui::SetTooltip("Flag carrier");
                    } else {
                        ImGui::SetTooltip("Flag carrier");
                    }
                }
            }
        }
    }

    // Corpse direction indicator — shown when player is a ghost
    if (gameHandler.isPlayerGhost()) {
        float corpseCanX = 0.0f, corpseCanY = 0.0f;
        if (gameHandler.getCorpseCanonicalPos(corpseCanX, corpseCanY)) {
            glm::vec3 corpseRender = core::coords::canonicalToRender(glm::vec3(corpseCanX, corpseCanY, 0.0f));
            float csx = 0.0f, csy = 0.0f;
            bool onMap = projectToMinimap(corpseRender, csx, csy);

            if (onMap) {
                // Draw a small skull-like X marker at the corpse position
                const float r = 5.0f;
                drawList->AddCircleFilled(ImVec2(csx, csy), r + 1.0f, IM_COL32(0, 0, 0, 140), 12);
                drawList->AddCircle(ImVec2(csx, csy), r + 1.0f, IM_COL32(200, 200, 220, 220), 12, 1.5f);
                // Draw an X in the circle
                drawList->AddLine(ImVec2(csx - 3.0f, csy - 3.0f), ImVec2(csx + 3.0f, csy + 3.0f),
                                  IM_COL32(180, 180, 220, 255), 1.5f);
                drawList->AddLine(ImVec2(csx + 3.0f, csy - 3.0f), ImVec2(csx - 3.0f, csy + 3.0f),
                                  IM_COL32(180, 180, 220, 255), 1.5f);
                // Tooltip on hover
                ImVec2 mouse = ImGui::GetMousePos();
                float mdx = mouse.x - csx, mdy = mouse.y - csy;
                if (mdx * mdx + mdy * mdy < 64.0f) {
                    float dist = gameHandler.getCorpseDistance();
                    if (dist >= 0.0f)
                        ImGui::SetTooltip("Your corpse (%.0f yd)", dist);
                    else
                        ImGui::SetTooltip("Your corpse");
                }
            } else {
                // Corpse is outside minimap — draw an edge arrow pointing toward it
                float dx = corpseRender.x - playerRender.x;
                float dy = corpseRender.y - playerRender.y;
                // Rotate delta into minimap frame (same as projectToMinimap)
                float rx = -(dy * cosB - dx * sinB);
                float ry = -(dy * sinB + dx * cosB);
                float len = std::sqrt(rx * rx + ry * ry);
                if (len > 0.001f) {
                    float nx = rx / len;
                    float ny = ry / len;
                    // Place arrow at the minimap edge
                    float edgeR = mapRadius - 7.0f;
                    float ax = centerX + nx * edgeR;
                    float ay = centerY + ny * edgeR;
                    // Arrow pointing outward (toward corpse)
                    float arrowLen = 6.0f;
                    float arrowW = 3.5f;
                    ImVec2 tip(ax + nx * arrowLen, ay + ny * arrowLen);
                    ImVec2 left(ax - ny * arrowW - nx * arrowLen * 0.4f,
                                ay + nx * arrowW - ny * arrowLen * 0.4f);
                    ImVec2 right(ax + ny * arrowW - nx * arrowLen * 0.4f,
                                 ay - nx * arrowW - ny * arrowLen * 0.4f);
                    drawList->AddTriangleFilled(tip, left, right, IM_COL32(180, 180, 240, 230));
                    drawList->AddTriangle(tip, left, right, IM_COL32(0, 0, 0, 180), 1.0f);
                    // Tooltip on hover
                    ImVec2 mouse = ImGui::GetMousePos();
                    float mdx = mouse.x - ax, mdy = mouse.y - ay;
                    if (mdx * mdx + mdy * mdy < 100.0f) {
                        float dist = gameHandler.getCorpseDistance();
                        if (dist >= 0.0f)
                            ImGui::SetTooltip("Your corpse (%.0f yd)", dist);
                        else
                            ImGui::SetTooltip("Your corpse");
                    }
                }
            }
        }
    }

    // Player position arrow at minimap center, pointing in camera facing direction.
    // On a rotating minimap the map already turns so forward = screen-up; on a fixed
    // minimap we rotate the arrow to match the player's compass heading.
    {
        // Compute screen-space facing direction for the arrow.
        // bearing = clockwise angle from screen-north (0 = facing north/up).
        float arrowAngle = 0.0f; // 0 = pointing up (north)
        if (!minimap->isRotateWithCamera()) {
            // Fixed minimap: arrow must show actual facing relative to north.
            // Match the mirrored minimap texture by flipping the arrow's
            // visual north/south component.
            arrowAngle = -glm::radians(renderer->getCharacterYaw());
        }
        // Screen direction the arrow tip points toward
        float nx =  std::sin(arrowAngle); // screen +X = east
        float ny = -std::cos(arrowAngle); // screen -Y = north

        // Draw a chevron-style arrow: tip, two base corners, and a notch at the back
        const float tipLen  = 8.0f;  // tip forward distance
        const float baseW   = 5.0f;  // half-width at base
        const float notchIn = 3.0f;  // how far back the center notch sits
        // Perpendicular direction (rotated 90°)
        float px =  ny; // perpendicular x
        float py = -nx; // perpendicular y

        ImVec2 tip  (centerX + nx * tipLen,  centerY + ny * tipLen);
        ImVec2 baseL(centerX - nx * baseW + px * baseW,  centerY - ny * baseW + py * baseW);
        ImVec2 baseR(centerX - nx * baseW - px * baseW,  centerY - ny * baseW - py * baseW);
        ImVec2 notch(centerX - nx * (baseW - notchIn),   centerY - ny * (baseW - notchIn));

        // Fill: bright white with slight gold tint, dark outline for readability
        drawList->AddTriangleFilled(tip, baseL, notch, IM_COL32(255, 248, 200, 245));
        drawList->AddTriangleFilled(tip, notch, baseR, IM_COL32(255, 248, 200, 245));
        drawList->AddTriangle(tip, baseL, notch, IM_COL32(60, 40, 0, 200), 1.2f);
        drawList->AddTriangle(tip, notch, baseR, IM_COL32(60, 40, 0, 200), 1.2f);
    }

    // Skip minimap input when an ImGui window (bag, settings, etc.) is in front.
    ImGuiContext& g = *ImGui::GetCurrentContext();
    bool minimapInputBlocked = (g.HoveredWindow != nullptr);

    // Scroll wheel over minimap → zoom in/out
    if (!minimapInputBlocked) {
        float wheel = ImGui::GetIO().MouseWheel;
        if (wheel != 0.0f) {
            ImVec2 mouse = ImGui::GetMousePos();
            float mdx = mouse.x - centerX;
            float mdy = mouse.y - centerY;
            if (mdx * mdx + mdy * mdy <= mapRadius * mapRadius) {
                if (wheel > 0.0f)
                    minimap->zoomIn();
                else
                    minimap->zoomOut();
            }
        }
    }

    // Ctrl+click on minimap → send minimap ping to party
    if (!minimapInputBlocked && ImGui::IsMouseClicked(0) && ImGui::GetIO().KeyCtrl) {
        ImVec2 mouse = ImGui::GetMousePos();
        float mdx = mouse.x - centerX;
        float mdy = mouse.y - centerY;
        float distSq = mdx * mdx + mdy * mdy;
        if (distSq <= mapRadius * mapRadius) {
            // Invert projectToMinimap: px=mdx, py=mdy → rx=px*viewRadius/mapRadius
            float rx = mdx * viewRadius / mapRadius;
            float ry = mdy * viewRadius / mapRadius;
            // rx/ry are in rotated minimap frame; invert the same transform
            // used by projectToMinimap, including the horizontal mirror.
            float oldRx = -rx;
            float rotX = oldRx * cosB - ry * sinB;
            float rotY = oldRx * sinB + ry * cosB;
            float wdx = -rotY;
            float wdy =  rotX;
            // playerRender is in render coords; add delta to get render position then convert to canonical
            glm::vec3 clickRender = playerRender + glm::vec3(wdx, wdy, 0.0f);
            glm::vec3 clickCanon = core::coords::renderToCanonical(clickRender);
            gameHandler.sendMinimapPing(clickCanon.x, clickCanon.y);
        }
    }

    // Optional persistent coordinate display below the minimap.
    if (settingsPanel_.showMinimapCoordinates_) {
        glm::vec3 playerCanon = core::coords::renderToCanonical(playerRender);
        char coordBuf[32];
        std::snprintf(coordBuf, sizeof(coordBuf), "%.1f, %.1f", playerCanon.x, playerCanon.y);

        ImFont* font = ImGui::GetFont();
        float fontSize = ImGui::GetFontSize();
        ImVec2 textSz = font->CalcTextSizeA(fontSize, FLT_MAX, 0.0f, coordBuf);

        float tx = centerX - textSz.x * 0.5f;
        float ty = centerY + mapRadius + 3.0f;

        // Semi-transparent dark background pill
        float pad = 3.0f;
        drawList->AddRectFilled(
            ImVec2(tx - pad, ty - pad),
            ImVec2(tx + textSz.x + pad, ty + textSz.y + pad),
            IM_COL32(0, 0, 0, 140), 4.0f);
        // Coordinate text in warm yellow
        drawList->AddText(font, fontSize, ImVec2(tx, ty), IM_COL32(230, 220, 140, 255), coordBuf);
    }

    // Zone name display — drawn inside the top edge of the minimap circle
    {
        std::string zoneName;
        // The live terrain-derived zone first; the server's only if that has
        // nothing. The server tells us its zone on SMSG_INIT_WORLD_STATES and
        // at no other time, so preferring it left this label naming the last
        // zone the server announced rather than the one being walked through.
        uint32_t zoneId = renderer ? renderer->getCurrentZoneId() : 0u;
        const uint32_t serverZoneId = gameHandler.getWorldStateZoneId();
        if (zoneId == 0) zoneId = serverZoneId;
        // Said once each time the pair changes. The two disagreeing is not by
        // itself a fault — the server only revises its answer when it notices
        // a zone change — but a disagreement that persists while standing
        // still means the server has the player somewhere else, which is also
        // why no creatures would arrive. This prints both names and the
        // position, which is what tells those two cases apart.
        if (serverZoneId != 0 && zoneId != 0 && serverZoneId != zoneId) {
            static uint64_t lastReported = 0;
            const uint64_t pair = (static_cast<uint64_t>(serverZoneId) << 32) | zoneId;
            if (pair != lastReported) {
                lastReported = pair;
                const auto& mi = gameHandler.getMovementInfo();
                LOG_WARNING("Zone disagreement: server says ", serverZoneId, " (",
                            gameHandler.getWhoAreaName(serverZoneId),
                            "), terrain under the player says ", zoneId, " (",
                            gameHandler.getWhoAreaName(zoneId),
                            ") at canonical ", mi.x, ", ", mi.y, ", ", mi.z);
            }
        }
        if (zoneId != 0) {
            zoneName = gameHandler.getWhoAreaName(zoneId);
            if (zoneName.empty()) {
                if (auto* zmRenderer = renderer ? renderer->getZoneManager() : nullptr) {
                    if (const game::ZoneInfo* zi = zmRenderer->getZoneInfo(zoneId)) {
                        zoneName = zi->name;
                    }
                }
            }
        }
        if (zoneName.empty() && renderer) {
            zoneName = renderer->getCurrentZoneName();
        }
        if (!zoneName.empty()) {
            ImFont* font = ImGui::GetFont();
            float fontSize = ImGui::GetFontSize();

            const char* weatherIcon = nullptr;
            ImU32 weatherColor = IM_COL32(255, 255, 255, 200);
            const uint32_t weatherType = gameHandler.getWeatherType();
            const float weatherIntensity = gameHandler.getWeatherIntensity();
            if (weatherType == 1 && weatherIntensity > 0.05f) {
                weatherIcon = " \xe2\x9b\x86"; // Rain
                weatherColor = IM_COL32(140, 180, 240, 220);
            } else if (weatherType == 2 && weatherIntensity > 0.05f) {
                weatherIcon = " \xe2\x9d\x84"; // Snow
                weatherColor = IM_COL32(210, 230, 255, 220);
            } else if (weatherType == 3 && weatherIntensity > 0.05f) {
                weatherIcon = " \xe2\x98\x81"; // Storm/fog
                weatherColor = IM_COL32(160, 160, 190, 220);
            }

            const std::string fullLabel = weatherIcon ? zoneName + weatherIcon : zoneName;
            ImVec2 ts = font->CalcTextSizeA(fontSize, FLT_MAX, 0.0f, fullLabel.c_str());
            float tx = centerX - ts.x * 0.5f;
            float ty = centerY - mapRadius + 4.0f;  // just inside top edge of the circle
            float pad = 2.0f;
            drawList->AddRectFilled(
                ImVec2(tx - pad, ty - pad),
                ImVec2(tx + ts.x + pad, ty + ts.y + pad),
                IM_COL32(0, 0, 0, 160), 2.0f);
            drawList->AddText(font, fontSize, ImVec2(tx + 1.0f, ty + 1.0f),
                              IM_COL32(0, 0, 0, 180), zoneName.c_str());
            drawList->AddText(font, fontSize, ImVec2(tx, ty),
                              IM_COL32(255, 230, 150, 220), zoneName.c_str());
            if (weatherIcon) {
                const ImVec2 nameSize = font->CalcTextSizeA(
                    fontSize, FLT_MAX, 0.0f, zoneName.c_str());
                drawList->AddText(font, fontSize, ImVec2(tx + nameSize.x, ty),
                                  weatherColor, weatherIcon);
            }
        }
    }

    // Instance difficulty indicator — just below zone name, inside minimap top edge
    if (gameHandler.isInInstance()) {
        static constexpr const char* kDiffLabels[] = {"Normal", "Heroic", "25 Normal", "25 Heroic"};
        uint32_t diff = gameHandler.getInstanceDifficulty();
        const char* label = (diff < 4) ? kDiffLabels[diff] : "Unknown";

        ImFont* font = ImGui::GetFont();
        float fontSize = ImGui::GetFontSize() * 0.85f;
        ImVec2 ts = font->CalcTextSizeA(fontSize, FLT_MAX, 0.0f, label);
        float tx = centerX - ts.x * 0.5f;
        // Position below zone name: top edge + zone font size + small gap
        float ty = centerY - mapRadius + 4.0f + ImGui::GetFontSize() + 2.0f;
        float pad = 2.0f;

        // Color-code: heroic=orange, normal=light gray
        ImU32 bgCol = gameHandler.isInstanceHeroic() ? IM_COL32(120, 60, 0, 180) : IM_COL32(0, 0, 0, 160);
        ImU32 textCol = gameHandler.isInstanceHeroic() ? IM_COL32(255, 180, 50, 255) : IM_COL32(200, 200, 200, 220);

        drawList->AddRectFilled(
            ImVec2(tx - pad, ty - pad),
            ImVec2(tx + ts.x + pad, ty + ts.y + pad),
            bgCol, 2.0f);
        drawList->AddText(font, fontSize, ImVec2(tx, ty), textCol, label);
    }

    // Hover tooltip and right-click context menu
    {
        ImVec2 mouse = ImGui::GetMousePos();
        float mdx = mouse.x - centerX;
        float mdy = mouse.y - centerY;
        bool overMinimap = !minimapInputBlocked && (mdx * mdx + mdy * mdy <= mapRadius * mapRadius);

        if (overMinimap) {
            ImGui::BeginTooltip();
            if (settingsPanel_.showMinimapCoordinates_) {
                // Compute the world coordinate under the mouse cursor.
                float rxW = mdx / mapRadius * viewRadius;
                float ryW = mdy / mapRadius * viewRadius;
                float hoverOldRx = -rxW;
                float hoverRotX = hoverOldRx * cosB - ryW * sinB;
                float hoverRotY = hoverOldRx * sinB + ryW * cosB;
                float hoverDx = -hoverRotY;
                float hoverDy =  hoverRotX;
                glm::vec3 hoverRender(playerRender.x + hoverDx, playerRender.y + hoverDy, playerRender.z);
                glm::vec3 hoverCanon = core::coords::renderToCanonical(hoverRender);
                ImGui::TextColored(ImVec4(0.9f, 0.85f, 0.5f, 1.0f), "%.1f, %.1f", hoverCanon.x, hoverCanon.y);
            }
            ImGui::TextColored(colors::kMediumGray, "Ctrl+click to ping");
            ImGui::EndTooltip();

            if (ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
                ImGui::OpenPopup("##minimapContextMenu");
            }
        }

        if (ImGui::BeginPopup("##minimapContextMenu")) {
            ImGui::TextColored(ui::colors::kTooltipGold, "Minimap");
            ImGui::Separator();

            // Zoom controls
            if (ImGui::MenuItem("Zoom In")) {
                minimap->zoomIn();
            }
            if (ImGui::MenuItem("Zoom Out")) {
                minimap->zoomOut();
            }

            ImGui::Separator();

            // Toggle options with checkmarks
            bool rotWithCam = minimap->isRotateWithCamera();
            if (ImGui::MenuItem("Rotate with Camera", nullptr, rotWithCam)) {
                minimap->setRotateWithCamera(!rotWithCam);
            }

            bool squareShape = minimap->isSquareShape();
            if (ImGui::MenuItem("Square Shape", nullptr, squareShape)) {
                minimap->setSquareShape(!squareShape);
            }

            bool npcDots = settingsPanel_.minimapNpcDots_;
            if (ImGui::MenuItem("Show NPC Dots", nullptr, npcDots)) {
                settingsPanel_.minimapNpcDots_ = !settingsPanel_.minimapNpcDots_;
            }

            ImGui::EndPopup();
        }
    }

    auto applyMuteState = [&]() {
        auto* ac = services_.audioCoordinator;
        float masterScale = settingsPanel_.soundMuted_ ? 0.0f : static_cast<float>(settingsPanel_.pendingMasterVolume) / 100.0f;
        audio::AudioEngine::instance().setMasterVolume(masterScale);
        if (!ac) return;
        if (auto* music = ac->getMusicManager()) {
            music->setVolume(settingsPanel_.pendingMusicVolume);
        }
        if (auto* ambient = ac->getAmbientSoundManager()) {
            ambient->setVolumeScale(settingsPanel_.pendingAmbientVolume / 100.0f);
            ambient->setBellVolumeScale(settingsPanel_.pendingBellVolume / 100.0f);
        }
        if (auto* ui = ac->getUiSoundManager()) {
            ui->setVolumeScale(settingsPanel_.pendingUiVolume / 100.0f);
        }
        if (auto* combat = ac->getCombatSoundManager()) {
            combat->setVolumeScale(settingsPanel_.pendingCombatVolume / 100.0f);
        }
        if (auto* spell = ac->getSpellSoundManager()) {
            spell->setVolumeScale(settingsPanel_.pendingSpellVolume / 100.0f);
        }
        if (auto* movement = ac->getMovementSoundManager()) {
            movement->setVolumeScale(settingsPanel_.pendingMovementVolume / 100.0f);
        }
        if (auto* footstep = ac->getFootstepManager()) {
            footstep->setVolumeScale(settingsPanel_.pendingFootstepVolume / 100.0f);
        }
        if (auto* npcVoice = ac->getNpcVoiceManager()) {
            npcVoice->setVolumeScale(settingsPanel_.pendingNpcVoiceVolume / 100.0f);
        }
        if (auto* playerVoice = ac->getPlayerVoiceManager()) {
            playerVoice->setEnabled(settingsPanel_.pendingCharacterSpeech);
        }
        if (auto* mount = ac->getMountSoundManager()) {
            mount->setVolumeScale(settingsPanel_.pendingMountVolume / 100.0f);
        }
        if (auto* activity = ac->getActivitySoundManager()) {
            activity->setVolumeScale(settingsPanel_.pendingActivityVolume / 100.0f);
        }
    };

    // Speaker mute button at the minimap top-right corner
    ImGui::SetNextWindowPos(ImVec2(centerX + mapRadius - 26.0f, centerY - mapRadius + 4.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(22.0f, 22.0f), ImGuiCond_Always);
    ImGuiWindowFlags muteFlags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
                                 ImGuiWindowFlags_NoBackground;
    if (ImGui::Begin("##MinimapMute", nullptr, muteFlags)) {
        ImDrawList* draw = ImGui::GetWindowDrawList();
        ImVec2 p = ImGui::GetCursorScreenPos();
        ImVec2 size(20.0f, 20.0f);
        if (ImGui::InvisibleButton("##MinimapMuteButton", size)) {
            settingsPanel_.soundMuted_ = !settingsPanel_.soundMuted_;
            if (settingsPanel_.soundMuted_) {
                settingsPanel_.preMuteVolume_ = audio::AudioEngine::instance().getMasterVolume();
            }
            applyMuteState();
            saveSettings();
        }
        bool hovered = ImGui::IsItemHovered();
        ImU32 bg = settingsPanel_.soundMuted_ ? IM_COL32(135, 42, 42, 230) : IM_COL32(38, 38, 38, 210);
        if (hovered) bg = settingsPanel_.soundMuted_ ? IM_COL32(160, 58, 58, 230) : IM_COL32(65, 65, 65, 220);
        ImU32 fg = IM_COL32(255, 255, 255, 245);
        draw->AddRectFilled(p, ImVec2(p.x + size.x, p.y + size.y), bg, 4.0f);
        draw->AddRect(ImVec2(p.x + 0.5f, p.y + 0.5f), ImVec2(p.x + size.x - 0.5f, p.y + size.y - 0.5f),
                      IM_COL32(255, 255, 255, 42), 4.0f);
        draw->AddRectFilled(ImVec2(p.x + 4.0f, p.y + 8.0f), ImVec2(p.x + 7.0f, p.y + 12.0f), fg, 1.0f);
        draw->AddTriangleFilled(ImVec2(p.x + 7.0f, p.y + 7.0f),
                                ImVec2(p.x + 7.0f, p.y + 13.0f),
                                ImVec2(p.x + 11.8f, p.y + 10.0f), fg);
        if (settingsPanel_.soundMuted_) {
            draw->AddLine(ImVec2(p.x + 13.5f, p.y + 6.2f), ImVec2(p.x + 17.2f, p.y + 13.8f), fg, 1.8f);
            draw->AddLine(ImVec2(p.x + 17.2f, p.y + 6.2f), ImVec2(p.x + 13.5f, p.y + 13.8f), fg, 1.8f);
        } else {
            draw->PathArcTo(ImVec2(p.x + 11.8f, p.y + 10.0f), 3.6f, -0.7f, 0.7f, 12);
            draw->PathStroke(fg, 0, 1.4f);
            draw->PathArcTo(ImVec2(p.x + 11.8f, p.y + 10.0f), 5.5f, -0.7f, 0.7f, 12);
            draw->PathStroke(fg, 0, 1.2f);
        }
        if (hovered) ImGui::SetTooltip(settingsPanel_.soundMuted_ ? "Unmute" : "Mute");
    }
    ImGui::End();

    // Friends button at top-left of minimap
    {
        const auto& contacts = gameHandler.getContacts();
        int onlineCount = 0;
        for (const auto& c : contacts)
            if (c.isFriend() && c.isOnline()) ++onlineCount;

        ImGui::SetNextWindowPos(ImVec2(centerX - mapRadius + 4.0f, centerY - mapRadius + 4.0f), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(22.0f, 22.0f), ImGuiCond_Always);
        ImGuiWindowFlags friendsBtnFlags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                                           ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
                                           ImGuiWindowFlags_NoBackground;
        if (ImGui::Begin("##MinimapFriendsBtn", nullptr, friendsBtnFlags)) {
            ImDrawList* draw = ImGui::GetWindowDrawList();
            ImVec2 p = ImGui::GetCursorScreenPos();
            ImVec2 sz(20.0f, 20.0f);
            if (ImGui::InvisibleButton("##FriendsBtnInv", sz)) {
                socialPanel_.showSocialFrame_ = !socialPanel_.showSocialFrame_;
            }
            bool hovered = ImGui::IsItemHovered();
            ImU32 bg = socialPanel_.showSocialFrame_
                ? IM_COL32(42, 100, 42, 230)
                : IM_COL32(38, 38, 38, 210);
            if (hovered) bg = socialPanel_.showSocialFrame_ ? IM_COL32(58, 130, 58, 230) : IM_COL32(65, 65, 65, 220);
            draw->AddRectFilled(p, ImVec2(p.x + sz.x, p.y + sz.y), bg, 4.0f);
            draw->AddRect(ImVec2(p.x + 0.5f, p.y + 0.5f),
                          ImVec2(p.x + sz.x - 0.5f, p.y + sz.y - 0.5f),
                          IM_COL32(255, 255, 255, 42), 4.0f);
            // Simple smiley-face dots as "social" icon
            ImU32 fg = IM_COL32(255, 255, 255, 245);
            draw->AddCircle(ImVec2(p.x + 10.0f, p.y + 10.0f), 6.5f, fg, 16, 1.2f);
            draw->AddCircleFilled(ImVec2(p.x + 7.5f, p.y + 8.0f), 1.2f, fg);
            draw->AddCircleFilled(ImVec2(p.x + 12.5f, p.y + 8.0f), 1.2f, fg);
            draw->PathArcTo(ImVec2(p.x + 10.0f, p.y + 11.5f), 3.0f, 0.2f, 2.9f, 8);
            draw->PathStroke(fg, 0, 1.2f);
            // Small green dot if friends online
            if (onlineCount > 0) {
                draw->AddCircleFilled(ImVec2(p.x + sz.x - 3.5f, p.y + 3.5f),
                                      3.5f, IM_COL32(50, 220, 50, 255));
            }
            if (hovered) {
                if (onlineCount > 0)
                    ImGui::SetTooltip("Friends (%d online)", onlineCount);
                else
                    ImGui::SetTooltip("Friends");
            }
        }
        ImGui::End();
    }

    // Zoom buttons at the bottom edge of the minimap
    ImGui::SetNextWindowPos(ImVec2(centerX - 22, centerY + mapRadius - 30), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(44, 24), ImGuiCond_Always);
    ImGuiWindowFlags zoomFlags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                                  ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
                                  ImGuiWindowFlags_NoBackground;
    if (ImGui::Begin("##MinimapZoom", nullptr, zoomFlags)) {
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(2, 2));
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(2, 0));
        if (ImGui::SmallButton("-")) {
            if (minimap) minimap->zoomOut();
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("+")) {
            if (minimap) minimap->zoomIn();
        }
        ImGui::PopStyleVar(2);
    }
    ImGui::End();

    // Optional clock display at bottom-right of minimap (local time).
    if (settingsPanel_.showMinimapClock_) {
        auto now = std::chrono::system_clock::now();
        auto tt  = std::chrono::system_clock::to_time_t(now);
        std::tm tmBuf{};
#ifdef _WIN32
        localtime_s(&tmBuf, &tt);
#else
        localtime_r(&tt, &tmBuf);
#endif
        char clockText[16];
        std::snprintf(clockText, sizeof(clockText), "%d:%02d %s",
                      (tmBuf.tm_hour % 12 == 0) ? 12 : tmBuf.tm_hour % 12,
                      tmBuf.tm_min,
                      tmBuf.tm_hour >= 12 ? "PM" : "AM");
        ImVec2 clockSz = ImGui::CalcTextSize(clockText);
        float clockW = clockSz.x + 10.0f;
        float clockH = clockSz.y + 6.0f;
        ImGui::SetNextWindowPos(ImVec2(centerX + mapRadius - clockW - 2.0f,
                                       centerY + mapRadius - clockH - 2.0f), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(clockW, clockH), ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.45f);
        ImGuiWindowFlags clockFlags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                                      ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
                                      ImGuiWindowFlags_NoInputs;
        if (ImGui::Begin("##MinimapClock", nullptr, clockFlags)) {
            ImGui::TextColored(ImVec4(0.9f, 0.9f, 0.8f, 0.85f), "%s", clockText);
        }
        ImGui::End();
    }

    // Indicators below the minimap (stacked: new mail, then BG queue, then latency)
    float indicatorX = centerX - mapRadius;
    float nextIndicatorY = centerY + mapRadius + 4.0f;
    const float indicatorW = mapRadius * 2.0f;
    constexpr float kIndicatorH = 22.0f;
    ImGuiWindowFlags indicatorFlags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                                       ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
                                       ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoInputs;

    // "New Mail" indicator
    if (gameHandler.hasNewMail()) {
        ImGui::SetNextWindowPos(ImVec2(indicatorX, nextIndicatorY), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(indicatorW, kIndicatorH), ImGuiCond_Always);
        if (ImGui::Begin("##NewMailIndicator", nullptr, indicatorFlags)) {
            float pulse = 0.7f + 0.3f * std::sin(static_cast<float>(ImGui::GetTime()) * 3.0f);
            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.0f, pulse), "New Mail!");
        }
        ImGui::End();
        nextIndicatorY += kIndicatorH;
    }

    // Unspent talent points indicator
    {
        uint8_t unspent = gameHandler.getUnspentTalentPoints();
        if (unspent > 0) {
            ImGui::SetNextWindowPos(ImVec2(indicatorX, nextIndicatorY), ImGuiCond_Always);
            ImGui::SetNextWindowSize(ImVec2(indicatorW, kIndicatorH), ImGuiCond_Always);
            if (ImGui::Begin("##TalentIndicator", nullptr, indicatorFlags)) {
                float pulse = 0.7f + 0.3f * std::sin(static_cast<float>(ImGui::GetTime()) * 2.5f);
                char talentBuf[40];
                snprintf(talentBuf, sizeof(talentBuf), "! %u Talent Point%s Available",
                         static_cast<unsigned>(unspent), unspent == 1 ? "" : "s");
                ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f * pulse, pulse), "%s", talentBuf);
            }
            ImGui::End();
            nextIndicatorY += kIndicatorH;
        }
    }

    // BG queue status indicator (when in queue but not yet invited)
    for (const auto& slot : gameHandler.getBgQueues()) {
        if (slot.statusId != 1) continue;  // STATUS_WAIT_QUEUE only

        std::string bgName;
        if (slot.arenaType > 0) {
            bgName = std::to_string(slot.arenaType) + "v" + std::to_string(slot.arenaType) + " Arena";
        } else {
            switch (slot.bgTypeId) {
                case 1: bgName = "AV"; break;
                case 2: bgName = "WSG"; break;
                case 3: bgName = "AB"; break;
                case 7: bgName = "EotS"; break;
                case 9: bgName = "SotA"; break;
                case 11: bgName = "IoC"; break;
                default: bgName = "BG"; break;
            }
        }

        ImGui::SetNextWindowPos(ImVec2(indicatorX, nextIndicatorY), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(indicatorW, kIndicatorH), ImGuiCond_Always);
        if (ImGui::Begin("##BgQueueIndicator", nullptr, indicatorFlags)) {
            float pulse = 0.6f + 0.4f * std::sin(static_cast<float>(ImGui::GetTime()) * 1.5f);
            if (slot.avgWaitTimeSec > 0) {
                int avgMin = static_cast<int>(slot.avgWaitTimeSec) / 60;
                int avgSec = static_cast<int>(slot.avgWaitTimeSec) % 60;
                ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, pulse),
                    "Queue: %s (~%d:%02d)", bgName.c_str(), avgMin, avgSec);
            } else {
                ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, pulse),
                    "In Queue: %s", bgName.c_str());
            }
        }
        ImGui::End();
        nextIndicatorY += kIndicatorH;
        break;  // Show at most one queue slot indicator
    }

    // LFG queue indicator — shown when Dungeon Finder queue is active (Queued or RoleCheck)
    {
        using LfgState = game::GameHandler::LfgState;
        LfgState lfgSt = gameHandler.getLfgState();
        if (lfgSt == LfgState::Queued || lfgSt == LfgState::RoleCheck) {
            ImGui::SetNextWindowPos(ImVec2(indicatorX, nextIndicatorY), ImGuiCond_Always);
            ImGui::SetNextWindowSize(ImVec2(indicatorW, kIndicatorH), ImGuiCond_Always);
            if (ImGui::Begin("##LfgQueueIndicator", nullptr, indicatorFlags)) {
                if (lfgSt == LfgState::RoleCheck) {
                    float pulse = 0.6f + 0.4f * std::sin(static_cast<float>(ImGui::GetTime()) * 3.0f);
                    ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, pulse), "LFG: Role Check...");
                } else {
                    uint32_t qMs  = gameHandler.getLfgTimeInQueueMs();
                    int      qMin = static_cast<int>(qMs / 60000);
                    int      qSec = static_cast<int>((qMs % 60000) / 1000);
                    float pulse = 0.6f + 0.4f * std::sin(static_cast<float>(ImGui::GetTime()) * 1.2f);
                    ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, pulse),
                        "LFG: %d:%02d", qMin, qSec);
                }
            }
            ImGui::End();
            nextIndicatorY += kIndicatorH;
        }
    }

    // Calendar pending invites indicator (WotLK only)
    {
        auto* expReg = services_.expansionRegistry;
        bool isWotLK = expReg && expReg->getActive() && expReg->getActive()->id == "wotlk";
        if (isWotLK) {
            uint32_t calPending = gameHandler.getCalendarPendingInvites();
            if (calPending > 0) {
                ImGui::SetNextWindowPos(ImVec2(indicatorX, nextIndicatorY), ImGuiCond_Always);
                ImGui::SetNextWindowSize(ImVec2(indicatorW, kIndicatorH), ImGuiCond_Always);
                if (ImGui::Begin("##CalendarIndicator", nullptr, indicatorFlags)) {
                    float pulse = 0.7f + 0.3f * std::sin(static_cast<float>(ImGui::GetTime()) * 2.0f);
                    char calBuf[48];
                    snprintf(calBuf, sizeof(calBuf), "Calendar: %u Invite%s",
                             calPending, calPending == 1 ? "" : "s");
                    ImGui::TextColored(ImVec4(0.6f, 0.5f, 1.0f, pulse), "%s", calBuf);
                }
                ImGui::End();
                nextIndicatorY += kIndicatorH;
            }
        }
    }

    // Taxi flight indicator — shown while on a flight path
    if (gameHandler.isOnTaxiFlight()) {
        ImGui::SetNextWindowPos(ImVec2(indicatorX, nextIndicatorY), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(indicatorW, kIndicatorH), ImGuiCond_Always);
        if (ImGui::Begin("##TaxiIndicator", nullptr, indicatorFlags)) {
            const std::string& dest = gameHandler.getTaxiDestName();
            float pulse = 0.7f + 0.3f * std::sin(static_cast<float>(ImGui::GetTime()) * 1.0f);
            if (dest.empty()) {
                ImGui::TextColored(ImVec4(0.6f, 0.85f, 1.0f, pulse), "\xe2\x9c\x88 In Flight");
            } else {
                char buf[64];
                snprintf(buf, sizeof(buf), "\xe2\x9c\x88 \xe2\x86\x92 %s", dest.c_str());
                ImGui::TextColored(ImVec4(0.6f, 0.85f, 1.0f, pulse), "%s", buf);
            }
        }
        ImGui::End();
        nextIndicatorY += kIndicatorH;
    }

    // Latency + FPS indicator — centered at top of screen
    uint32_t latMs = gameHandler.getLatencyMs();
    if (settingsPanel_.showLatencyMeter_ && gameHandler.getState() == game::WorldState::IN_WORLD) {
        float currentFps = ImGui::GetIO().Framerate;
        ImVec4 latColor;
        if      (latMs < 100) latColor = ImVec4(0.3f, 1.0f, 0.3f, 0.9f);
        else if (latMs < 250) latColor = ImVec4(1.0f, 1.0f, 0.3f, 0.9f);
        else if (latMs < 500) latColor = ImVec4(1.0f, 0.6f, 0.1f, 0.9f);
        else                  latColor = ImVec4(1.0f, 0.2f, 0.2f, 0.9f);

        ImVec4 fpsColor;
        if      (currentFps >= 60.0f) fpsColor = ImVec4(0.3f, 1.0f, 0.3f, 0.9f);
        else if (currentFps >= 30.0f) fpsColor = ImVec4(1.0f, 1.0f, 0.3f, 0.9f);
        else                          fpsColor = ImVec4(1.0f, 0.3f, 0.3f, 0.9f);

        char infoText[64];
        if (latMs > 0)
            snprintf(infoText, sizeof(infoText), "%.0f fps  |  %u ms", currentFps, latMs);
        else
            snprintf(infoText, sizeof(infoText), "%.0f fps", currentFps);

        ImVec2 textSize = ImGui::CalcTextSize(infoText);
        float latW = textSize.x + 16.0f;
        float latH = textSize.y + 8.0f;
        ImGuiIO& lio = ImGui::GetIO();
        float latX = (lio.DisplaySize.x - latW) * 0.5f;
        ImGui::SetNextWindowPos(ImVec2(latX, 4.0f), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(latW, latH), ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.45f);
        if (ImGui::Begin("##LatencyIndicator", nullptr, indicatorFlags)) {
            // Color the FPS and latency portions differently
            ImGui::TextColored(fpsColor, "%.0f fps", currentFps);
            if (latMs > 0) {
                ImGui::SameLine(0, 4);
                ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 0.7f), "|");
                ImGui::SameLine(0, 4);
                ImGui::TextColored(latColor, "%u ms", latMs);
            }
        }
        ImGui::End();
    }

    // Low durability warning — shown when any equipped item has < 20% durability
    if (gameHandler.getState() == game::WorldState::IN_WORLD) {
        const auto& inv = gameHandler.getInventory();
        float lowestDurPct = 1.0f;
        for (int i = 0; i < game::Inventory::NUM_EQUIP_SLOTS; ++i) {
            const auto& slot = inv.getEquipSlot(static_cast<game::EquipSlot>(i));
            if (slot.empty()) continue;
            const auto& it = slot.item;
            if (it.maxDurability > 0) {
                float pct = static_cast<float>(it.curDurability) / static_cast<float>(it.maxDurability);
                if (pct < lowestDurPct) lowestDurPct = pct;
            }
        }
        if (lowestDurPct < 0.20f) {
            bool critical = (lowestDurPct < 0.05f);
            float pulse = critical
                ? (0.7f + 0.3f * std::sin(static_cast<float>(ImGui::GetTime()) * 4.0f))
                : 1.0f;
            ImVec4 durWarnColor = critical
                ? ImVec4(1.0f, 0.2f, 0.2f, pulse)
                : ImVec4(1.0f, 0.65f, 0.1f, 0.9f);
            const char* durWarnText = critical ? "Item breaking!" : "Low durability";

            ImGui::SetNextWindowPos(ImVec2(indicatorX, nextIndicatorY), ImGuiCond_Always);
            ImGui::SetNextWindowSize(ImVec2(indicatorW, kIndicatorH), ImGuiCond_Always);
            if (ImGui::Begin("##DurabilityIndicator", nullptr, indicatorFlags)) {
                ImGui::TextColored(durWarnColor, "%s", durWarnText);
            }
            ImGui::End();
            nextIndicatorY += kIndicatorH;
        }
    }

}

void GameScreen::saveSettings() {
    std::string path = SettingsPanel::getSettingsPath();
    std::filesystem::path dir = std::filesystem::path(path).parent_path();
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);

    std::ofstream out(path);
    if (!out.is_open()) {
        LOG_WARNING("Could not save settings to ", path);
        return;
    }

    // Interface
    out << "ui_opacity=" << settingsPanel_.pendingUiOpacity << "\n";
    out << "window_ui_scale=" << settingsPanel_.pendingWindowUiScale << "\n";
    out << "minimap_rotate=" << (settingsPanel_.pendingMinimapRotate ? 1 : 0) << "\n";
    out << "minimap_square=" << (settingsPanel_.pendingMinimapSquare ? 1 : 0) << "\n";
    out << "minimap_npc_dots=" << (settingsPanel_.pendingMinimapNpcDots ? 1 : 0) << "\n";
    out << "show_minimap_clock=" << (settingsPanel_.pendingShowMinimapClock ? 1 : 0) << "\n";
    out << "show_minimap_coordinates=" << (settingsPanel_.pendingShowMinimapCoordinates ? 1 : 0) << "\n";
    out << "show_latency_meter=" << (settingsPanel_.pendingShowLatencyMeter ? 1 : 0) << "\n";
    out << "show_dps_meter=" << (settingsPanel_.showDPSMeter_ ? 1 : 0) << "\n";
    {
        // Only written once the user has dragged it; otherwise the meter keeps
        // following the target frame on the next launch.
        ImVec2 dpsPos = combatUI_.getDPSMeterPos();
        if (dpsPos.x >= 0.0f) {
            out << "dps_meter_x=" << dpsPos.x << "\n";
            out << "dps_meter_y=" << dpsPos.y << "\n";
        }
    }
    out << "show_cooldown_tracker=" << (settingsPanel_.showCooldownTracker_ ? 1 : 0) << "\n";
    out << "show_rare_tracker=" << (settingsPanel_.showRareTracker_ ? 1 : 0) << "\n";
    out << "show_chest_tracker=" << (settingsPanel_.showChestTracker_ ? 1 : 0) << "\n";
    out << "separate_bags=" << (settingsPanel_.pendingSeparateBags ? 1 : 0) << "\n";
    out << "bank_combine_bags=" << (windowManager_.bankCombineBags_ ? 1 : 0) << "\n";
    out << "show_keyring=" << (settingsPanel_.pendingShowKeyring ? 1 : 0) << "\n";
    out << "bag_scale=" << settingsPanel_.pendingBagScale << "\n";
    out << "show_micro_menu=" << (settingsPanel_.pendingShowMicroMenu ? 1 : 0) << "\n";
    out << "idle_camera_orbit=" << (settingsPanel_.pendingIdleCameraOrbit ? 1 : 0) << "\n";
    out << "buff_bar_scale=" << settingsPanel_.pendingBuffBarScale << "\n";
    out << "action_bar_scale=" << settingsPanel_.pendingActionBarScale << "\n";
    out << "nameplate_scale=" << settingsPanel_.nameplateScale_ << "\n";
    out << "show_friendly_nameplates=" << (settingsPanel_.showFriendlyNameplates_ ? 1 : 0) << "\n";
    out << "show_action_bar2=" << (settingsPanel_.pendingShowActionBar2 ? 1 : 0) << "\n";
    out << "action_bar2_offset_x=" << settingsPanel_.pendingActionBar2OffsetX << "\n";
    out << "action_bar2_offset_y=" << settingsPanel_.pendingActionBar2OffsetY << "\n";
    out << "show_right_bar=" << (settingsPanel_.pendingShowRightBar ? 1 : 0) << "\n";
    out << "show_left_bar=" << (settingsPanel_.pendingShowLeftBar ? 1 : 0) << "\n";
    out << "right_bar_offset_y=" << settingsPanel_.pendingRightBarOffsetY << "\n";
    out << "left_bar_offset_y=" << settingsPanel_.pendingLeftBarOffsetY << "\n";
    out << "damage_flash=" << (settingsPanel_.damageFlashEnabled_ ? 1 : 0) << "\n";
    out << "low_health_vignette=" << (settingsPanel_.lowHealthVignetteEnabled_ ? 1 : 0) << "\n";

    // Audio
    out << "sound_muted=" << (settingsPanel_.soundMuted_ ? 1 : 0) << "\n";
    out << "use_original_soundtrack=" << (settingsPanel_.pendingUseOriginalSoundtrack ? 1 : 0) << "\n";
    out << "master_volume=" << settingsPanel_.pendingMasterVolume << "\n";
    out << "music_volume=" << settingsPanel_.pendingMusicVolume << "\n";
    out << "ambient_volume=" << settingsPanel_.pendingAmbientVolume << "\n";
    out << "bell_volume=" << settingsPanel_.pendingBellVolume << "\n";
    out << "ui_volume=" << settingsPanel_.pendingUiVolume << "\n";
    out << "combat_volume=" << settingsPanel_.pendingCombatVolume << "\n";
    out << "spell_volume=" << settingsPanel_.pendingSpellVolume << "\n";
    out << "movement_volume=" << settingsPanel_.pendingMovementVolume << "\n";
    out << "footstep_volume=" << settingsPanel_.pendingFootstepVolume << "\n";
    out << "npc_voice_volume=" << settingsPanel_.pendingNpcVoiceVolume << "\n";
    out << "mount_volume=" << settingsPanel_.pendingMountVolume << "\n";
    out << "activity_volume=" << settingsPanel_.pendingActivityVolume << "\n";
    out << "character_speech=" << (settingsPanel_.pendingCharacterSpeech ? 1 : 0) << "\n";

    // Gameplay / display pacing
    out << "fullscreen=" << (settingsPanel_.pendingFullscreen ? 1 : 0) << "\n";
    out << "resolution_width=" << settingsPanel_.pendingResolutionWidth << "\n";
    out << "resolution_height=" << settingsPanel_.pendingResolutionHeight << "\n";
    out << "vsync=" << (settingsPanel_.pendingVsync ? 1 : 0) << "\n";
    out << "auto_loot=" << (settingsPanel_.pendingAutoLoot ? 1 : 0) << "\n";
    out << "auto_sell_grey=" << (settingsPanel_.pendingAutoSellGrey ? 1 : 0) << "\n";
    out << "auto_repair=" << (settingsPanel_.pendingAutoRepair ? 1 : 0) << "\n";
    out << "graphics_preset=" << static_cast<int>(settingsPanel_.currentGraphicsPreset) << "\n";
    out << "ground_clutter_density=" << settingsPanel_.pendingGroundClutterDensity << "\n";
    out << "shadows=" << (settingsPanel_.pendingShadows ? 1 : 0) << "\n";
    out << "shadow_distance=" << settingsPanel_.pendingShadowDistance << "\n";
    out << "view_distance=" << settingsPanel_.pendingViewDistance << "\n";
    out << "brightness=" << settingsPanel_.pendingBrightness << "\n";
    out << "water_refraction=" << (settingsPanel_.pendingWaterRefraction ? 1 : 0) << "\n";
    out << "antialiasing=" << settingsPanel_.pendingAntiAliasing << "\n";
    out << "fxaa=" << (settingsPanel_.pendingFXAA ? 1 : 0) << "\n";
    out << "normal_mapping=" << (settingsPanel_.pendingNormalMapping ? 1 : 0) << "\n";
    out << "normal_map_strength=" << settingsPanel_.pendingNormalMapStrength << "\n";
    out << "pom=" << (settingsPanel_.pendingPOM ? 1 : 0) << "\n";
    out << "pom_quality=" << settingsPanel_.pendingPOMQuality << "\n";
    out << "upscaling_mode=" << settingsPanel_.pendingUpscalingMode << "\n";
    out << "fsr=" << (settingsPanel_.pendingFSR ? 1 : 0) << "\n";
    out << "fsr_quality=" << settingsPanel_.pendingFSRQuality << "\n";
    out << "fsr_sharpness=" << settingsPanel_.pendingFSRSharpness << "\n";
    out << "fsr2_jitter_sign=" << settingsPanel_.pendingFSR2JitterSign << "\n";
    out << "fsr2_mv_scale_x=" << settingsPanel_.pendingFSR2MotionVecScaleX << "\n";
    out << "fsr2_mv_scale_y=" << settingsPanel_.pendingFSR2MotionVecScaleY << "\n";
    out << "amd_fsr3_framegen=" << (settingsPanel_.pendingAMDFramegen ? 1 : 0) << "\n";

    // Controls
    out << "mouse_sensitivity=" << settingsPanel_.pendingMouseSensitivity << "\n";
    out << "invert_mouse=" << (settingsPanel_.pendingInvertMouse ? 1 : 0) << "\n";
    out << "extended_zoom=" << (settingsPanel_.pendingExtendedZoom ? 1 : 0) << "\n";
    out << "camera_stiffness=" << settingsPanel_.pendingCameraStiffness << "\n";
    out << "camera_pivot_height=" << settingsPanel_.pendingPivotHeight << "\n";
    out << "camera_smooth_follow=" << (settingsPanel_.pendingSmoothCameraFollow ? 1 : 0) << "\n";
    out << "fov=" << settingsPanel_.pendingFov << "\n";

    // Quest tracker position/size
    out << "quest_tracker_right_offset=" << questTrackerRightOffset_ << "\n";
    out << "quest_tracker_y=" << questTrackerPos_.y << "\n";
    out << "quest_tracker_w=" << questTrackerSize_.x << "\n";
    out << "quest_tracker_h=" << questTrackerSize_.y << "\n";
    out << "quest_tracker_filter=" << questTrackerFilter_ << "\n";
    out << "quest_tracker_collapsed=" << (questTrackerCollapsed_ ? 1 : 0) << "\n";

    // Chat
    out << "chat_active_tab=" << chatPanel_.activeChatTab << "\n";
    out << "chat_timestamps=" << (chatPanel_.chatShowTimestamps ? 1 : 0) << "\n";
    out << "chat_font_size=" << chatPanel_.chatFontSize << "\n";
    out << "chat_bg_alpha=" << chatPanel_.settings.backgroundAlpha << "\n";
    out << "chat_window_w=" << chatPanel_.settings.windowWidth << "\n";
    out << "chat_window_h=" << chatPanel_.settings.windowHeight << "\n";
    out << "chat_fade_messages=" << (chatPanel_.settings.fadeMessages ? 1 : 0) << "\n";
    out << "chat_fade_time=" << chatPanel_.settings.messageFadeTime << "\n";
    out << "chat_autojoin_general=" << (chatPanel_.chatAutoJoinGeneral ? 1 : 0) << "\n";
    out << "chat_autojoin_trade=" << (chatPanel_.chatAutoJoinTrade ? 1 : 0) << "\n";
    out << "chat_autojoin_localdefense=" << (chatPanel_.chatAutoJoinLocalDefense ? 1 : 0) << "\n";
    out << "chat_autojoin_lfg=" << (chatPanel_.chatAutoJoinLFG ? 1 : 0) << "\n";
    out << "chat_autojoin_local=" << (chatPanel_.chatAutoJoinLocal ? 1 : 0) << "\n";

    out.close();

    // Save keybindings to the same config file (appends [Keybindings] section)
    KeybindingManager::getInstance().saveToConfigFile(path);

    LOG_INFO("Settings saved to ", path);
}

void GameScreen::loadSettings() {
    std::string path = SettingsPanel::getSettingsPath();
    std::ifstream in(path);
    float currentDisplayHeight = settingsPanel_.pendingResolutionHeight;
    if (ImGui::GetCurrentContext() && ImGui::GetIO().DisplaySize.y > 0.0f) {
        currentDisplayHeight = ImGui::GetIO().DisplaySize.y;
    }
    settingsPanel_.pendingBagScale =
        InventoryScreen::recommendedBagScale(currentDisplayHeight);
    inventoryScreen.setBagScale(settingsPanel_.pendingBagScale);
    if (!in.is_open()) return;

    bool bagScaleLoaded = false;
    std::string line;
    while (std::getline(in, line)) {
        size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string key = line.substr(0, eq);
        std::string val = line.substr(eq + 1);

        try {
            // Interface
            if (key == "ui_opacity") {
                int v = std::stoi(val);
                if (v >= 20 && v <= 100) {
                    settingsPanel_.pendingUiOpacity = v;
                    settingsPanel_.uiOpacity_ = static_cast<float>(v) / 100.0f;
                }
            } else if (key == "window_ui_scale") {
                settingsPanel_.pendingWindowUiScale = std::clamp(std::stof(val), 0.75f, 1.5f);
            } else if (key == "minimap_rotate") {
                // Ignore persisted rotate state; keep north-up.
                settingsPanel_.minimapRotate_ = false;
                settingsPanel_.pendingMinimapRotate = false;
            } else if (key == "minimap_square") {
                int v = std::stoi(val);
                settingsPanel_.minimapSquare_ = (v != 0);
                settingsPanel_.pendingMinimapSquare = settingsPanel_.minimapSquare_;
            } else if (key == "minimap_npc_dots") {
                int v = std::stoi(val);
                settingsPanel_.minimapNpcDots_ = (v != 0);
                settingsPanel_.pendingMinimapNpcDots = settingsPanel_.minimapNpcDots_;
            } else if (key == "show_minimap_clock") {
                settingsPanel_.showMinimapClock_ = (std::stoi(val) != 0);
                settingsPanel_.pendingShowMinimapClock = settingsPanel_.showMinimapClock_;
            } else if (key == "show_minimap_coordinates") {
                settingsPanel_.showMinimapCoordinates_ = (std::stoi(val) != 0);
                settingsPanel_.pendingShowMinimapCoordinates = settingsPanel_.showMinimapCoordinates_;
            } else if (key == "show_latency_meter") {
                settingsPanel_.showLatencyMeter_ = (std::stoi(val) != 0);
                settingsPanel_.pendingShowLatencyMeter = settingsPanel_.showLatencyMeter_;
            } else if (key == "show_dps_meter") {
                settingsPanel_.showDPSMeter_ = (std::stoi(val) != 0);
            } else if (key == "dps_meter_x") {
                dpsMeterSavedX_ = std::stof(val);
                if (dpsMeterSavedY_ >= 0.0f) combatUI_.setDPSMeterPos(dpsMeterSavedX_, dpsMeterSavedY_);
            } else if (key == "dps_meter_y") {
                dpsMeterSavedY_ = std::stof(val);
                if (dpsMeterSavedX_ >= 0.0f) combatUI_.setDPSMeterPos(dpsMeterSavedX_, dpsMeterSavedY_);
            } else if (key == "show_cooldown_tracker") {
                settingsPanel_.showCooldownTracker_ = (std::stoi(val) != 0);
            } else if (key == "show_rare_tracker") {
                settingsPanel_.showRareTracker_ = (std::stoi(val) != 0);
            } else if (key == "show_chest_tracker") {
                settingsPanel_.showChestTracker_ = (std::stoi(val) != 0);
            } else if (key == "bank_combine_bags") {
                windowManager_.bankCombineBags_ = (std::stoi(val) != 0);
            } else if (key == "separate_bags") {
                settingsPanel_.pendingSeparateBags = (std::stoi(val) != 0);
                inventoryScreen.setSeparateBags(settingsPanel_.pendingSeparateBags);
            } else if (key == "show_keyring") {
                settingsPanel_.pendingShowKeyring = (std::stoi(val) != 0);
                inventoryScreen.setShowKeyring(settingsPanel_.pendingShowKeyring);
            } else if (key == "bag_scale") {
                settingsPanel_.pendingBagScale = std::clamp(std::stof(val), 0.75f, 1.5f);
                inventoryScreen.setBagScale(settingsPanel_.pendingBagScale);
                bagScaleLoaded = true;
            } else if (key == "show_micro_menu") {
                settingsPanel_.pendingShowMicroMenu = (std::stoi(val) != 0);
            } else if (key == "idle_camera_orbit") {
                settingsPanel_.pendingIdleCameraOrbit = (std::stoi(val) != 0);
            } else if (key == "buff_bar_scale") {
                settingsPanel_.pendingBuffBarScale = std::clamp(std::stof(val), 0.75f, 1.5f);
            } else if (key == "action_bar_scale") {
                settingsPanel_.pendingActionBarScale = std::clamp(std::stof(val), 0.5f, 1.5f);
            } else if (key == "nameplate_scale") {
                settingsPanel_.nameplateScale_ = std::clamp(std::stof(val), 0.5f, 2.0f);
            } else if (key == "show_friendly_nameplates") {
                settingsPanel_.showFriendlyNameplates_ = (std::stoi(val) != 0);
            } else if (key == "show_action_bar2") {
                settingsPanel_.pendingShowActionBar2 = (std::stoi(val) != 0);
            } else if (key == "action_bar2_offset_x") {
                settingsPanel_.pendingActionBar2OffsetX = std::clamp(std::stof(val), -600.0f, 600.0f);
            } else if (key == "action_bar2_offset_y") {
                settingsPanel_.pendingActionBar2OffsetY = std::clamp(std::stof(val), -400.0f, 400.0f);
            } else if (key == "show_right_bar") {
                settingsPanel_.pendingShowRightBar = (std::stoi(val) != 0);
            } else if (key == "show_left_bar") {
                settingsPanel_.pendingShowLeftBar = (std::stoi(val) != 0);
            } else if (key == "right_bar_offset_y") {
                settingsPanel_.pendingRightBarOffsetY = std::clamp(std::stof(val), -400.0f, 400.0f);
            } else if (key == "left_bar_offset_y") {
                settingsPanel_.pendingLeftBarOffsetY = std::clamp(std::stof(val), -400.0f, 400.0f);
            } else if (key == "damage_flash") {
                settingsPanel_.damageFlashEnabled_ = (std::stoi(val) != 0);
            } else if (key == "low_health_vignette") {
                settingsPanel_.lowHealthVignetteEnabled_ = (std::stoi(val) != 0);
            }
            // Audio
            else if (key == "sound_muted") {
                settingsPanel_.soundMuted_ = (std::stoi(val) != 0);
                if (settingsPanel_.soundMuted_) {
                    // Apply mute on load; settingsPanel_.preMuteVolume_ will be set when AudioEngine is available
                    audio::AudioEngine::instance().setMasterVolume(0.0f);
                }
            }
            else if (key == "use_original_soundtrack") settingsPanel_.pendingUseOriginalSoundtrack = (std::stoi(val) != 0);
            else if (key == "master_volume") settingsPanel_.pendingMasterVolume = std::clamp(std::stoi(val), 0, 100);
            else if (key == "music_volume") settingsPanel_.pendingMusicVolume = std::clamp(std::stoi(val), 0, 100);
            else if (key == "ambient_volume") settingsPanel_.pendingAmbientVolume = std::clamp(std::stoi(val), 0, 100);
            else if (key == "bell_volume") settingsPanel_.pendingBellVolume = std::clamp(std::stoi(val), 0, 100);
            else if (key == "ui_volume") settingsPanel_.pendingUiVolume = std::clamp(std::stoi(val), 0, 100);
            else if (key == "combat_volume") settingsPanel_.pendingCombatVolume = std::clamp(std::stoi(val), 0, 100);
            else if (key == "spell_volume") settingsPanel_.pendingSpellVolume = std::clamp(std::stoi(val), 0, 100);
            else if (key == "movement_volume") settingsPanel_.pendingMovementVolume = std::clamp(std::stoi(val), 0, 100);
            else if (key == "footstep_volume") settingsPanel_.pendingFootstepVolume = std::clamp(std::stoi(val), 0, 100);
            else if (key == "npc_voice_volume") settingsPanel_.pendingNpcVoiceVolume = std::clamp(std::stoi(val), 0, 100);
            else if (key == "mount_volume") settingsPanel_.pendingMountVolume = std::clamp(std::stoi(val), 0, 100);
            else if (key == "activity_volume") settingsPanel_.pendingActivityVolume = std::clamp(std::stoi(val), 0, 100);
            else if (key == "character_speech") settingsPanel_.pendingCharacterSpeech = (std::stoi(val) != 0);
            // Gameplay / display pacing
            else if (key == "fullscreen") {
                settingsPanel_.pendingFullscreen = (std::stoi(val) != 0);
                settingsPanel_.displaySettingsLoaded_ = true;
            }
            else if (key == "resolution_width") {
                settingsPanel_.pendingResolutionWidth = std::clamp(std::stoi(val), 640, 7680);
                settingsPanel_.displaySettingsLoaded_ = true;
            }
            else if (key == "resolution_height") {
                settingsPanel_.pendingResolutionHeight = std::clamp(std::stoi(val), 480, 4320);
                settingsPanel_.displaySettingsLoaded_ = true;
            }
            else if (key == "vsync") settingsPanel_.pendingVsync = (std::stoi(val) != 0);
            else if (key == "auto_loot") settingsPanel_.pendingAutoLoot = (std::stoi(val) != 0);
            else if (key == "auto_sell_grey") settingsPanel_.pendingAutoSellGrey = (std::stoi(val) != 0);
            else if (key == "auto_repair") settingsPanel_.pendingAutoRepair = (std::stoi(val) != 0);
            else if (key == "graphics_preset") {
                int presetVal = std::clamp(std::stoi(val), 0, 4);
                settingsPanel_.currentGraphicsPreset = static_cast<SettingsPanel::GraphicsPreset>(presetVal);
                settingsPanel_.pendingGraphicsPreset = settingsPanel_.currentGraphicsPreset;
            }
            else if (key == "ground_clutter_density") settingsPanel_.pendingGroundClutterDensity = std::clamp(std::stoi(val), 0, 150);
            else if (key == "shadows") settingsPanel_.pendingShadows = (std::stoi(val) != 0);
            else if (key == "shadow_distance") settingsPanel_.pendingShadowDistance = std::clamp(std::stof(val), 40.0f, 500.0f);
            else if (key == "view_distance") settingsPanel_.pendingViewDistance = std::clamp(std::stof(val), 400.0f, 2400.0f);
            else if (key == "brightness") {
                settingsPanel_.pendingBrightness = std::clamp(std::stoi(val), 0, 100);
                if (auto* r = services_.renderer)
                    r->getPostProcessPipeline()->setBrightness(static_cast<float>(settingsPanel_.pendingBrightness) / 50.0f);
            }
            else if (key == "water_refraction") settingsPanel_.pendingWaterRefraction = (std::stoi(val) != 0);
            else if (key == "antialiasing") settingsPanel_.pendingAntiAliasing = std::clamp(std::stoi(val), 0, 3);
            else if (key == "fxaa") settingsPanel_.pendingFXAA = (std::stoi(val) != 0);
            else if (key == "normal_mapping") settingsPanel_.pendingNormalMapping = (std::stoi(val) != 0);
            else if (key == "normal_map_strength") settingsPanel_.pendingNormalMapStrength = std::clamp(std::stof(val), 0.0f, 2.0f);
            else if (key == "pom") settingsPanel_.pendingPOM = (std::stoi(val) != 0);
            else if (key == "pom_quality") settingsPanel_.pendingPOMQuality = std::clamp(std::stoi(val), 0, 2);
            else if (key == "upscaling_mode") {
                settingsPanel_.pendingUpscalingMode = std::clamp(std::stoi(val), 0, 2);
                settingsPanel_.pendingFSR = (settingsPanel_.pendingUpscalingMode == 1);
            } else if (key == "fsr") {
                settingsPanel_.pendingFSR = (std::stoi(val) != 0);
                // Backward compatibility: old configs only had fsr=0/1.
                if (settingsPanel_.pendingUpscalingMode == 0 && settingsPanel_.pendingFSR) settingsPanel_.pendingUpscalingMode = 1;
            }
            else if (key == "fsr_quality") settingsPanel_.pendingFSRQuality = std::clamp(std::stoi(val), 0, 3);
            else if (key == "fsr_sharpness") settingsPanel_.pendingFSRSharpness = std::clamp(std::stof(val), 0.0f, 2.0f);
            else if (key == "fsr2_jitter_sign") settingsPanel_.pendingFSR2JitterSign = std::clamp(std::stof(val), -2.0f, 2.0f);
            else if (key == "fsr2_mv_scale_x") settingsPanel_.pendingFSR2MotionVecScaleX = std::clamp(std::stof(val), -2.0f, 2.0f);
            else if (key == "fsr2_mv_scale_y") settingsPanel_.pendingFSR2MotionVecScaleY = std::clamp(std::stof(val), -2.0f, 2.0f);
            else if (key == "amd_fsr3_framegen") settingsPanel_.pendingAMDFramegen = (std::stoi(val) != 0);
            // Controls
            else if (key == "mouse_sensitivity") settingsPanel_.pendingMouseSensitivity = std::clamp(std::stof(val), 0.05f, 1.0f);
            else if (key == "invert_mouse") settingsPanel_.pendingInvertMouse = (std::stoi(val) != 0);
            else if (key == "extended_zoom") settingsPanel_.pendingExtendedZoom = (std::stoi(val) != 0);
            else if (key == "camera_stiffness") settingsPanel_.pendingCameraStiffness = std::clamp(std::stof(val), 5.0f, 100.0f);
            else if (key == "camera_pivot_height") settingsPanel_.pendingPivotHeight = std::clamp(std::stof(val), 0.0f, 3.0f);
            else if (key == "camera_smooth_follow") settingsPanel_.pendingSmoothCameraFollow = (std::stoi(val) != 0);
            else if (key == "fov") {
                settingsPanel_.pendingFov = std::clamp(std::stof(val), 45.0f, 110.0f);
                if (auto* renderer = services_.renderer) {
                    if (auto* camera = renderer->getCamera()) camera->setFov(settingsPanel_.pendingFov);
                }
            }
            // Quest tracker position/size
            else if (key == "quest_tracker_x") {
                // Legacy: ignore absolute X (right_offset supersedes it)
                (void)val;
            }
            else if (key == "quest_tracker_right_offset") {
                questTrackerRightOffset_ = std::stof(val);
                questTrackerPosInit_ = true;
            }
            else if (key == "quest_tracker_y") {
                questTrackerPos_.y = std::stof(val);
                questTrackerPosInit_ = true;
            }
            else if (key == "quest_tracker_w") {
                questTrackerSize_.x = std::max(100.0f, std::stof(val));
            }
            else if (key == "quest_tracker_h") {
                questTrackerSize_.y = std::max(60.0f, std::stof(val));
            }
            else if (key == "quest_tracker_filter") {
                questTrackerFilter_ = std::clamp(std::stoi(val), 0, 3);
            }
            else if (key == "quest_tracker_collapsed") {
                questTrackerCollapsed_ = (std::stoi(val) != 0);
            }
            // Chat
            else if (key == "chat_active_tab") chatPanel_.activeChatTab = std::clamp(std::stoi(val), 0, 3);
            else if (key == "chat_timestamps") chatPanel_.chatShowTimestamps = (std::stoi(val) != 0);
            else if (key == "chat_font_size") chatPanel_.chatFontSize = std::clamp(std::stoi(val), 0, 2);
            else if (key == "chat_bg_alpha") chatPanel_.settings.backgroundAlpha = std::clamp(std::stof(val), 0.0f, 1.0f);
            else if (key == "chat_window_w") chatPanel_.settings.windowWidth = std::max(0.0f, std::stof(val));
            else if (key == "chat_window_h") chatPanel_.settings.windowHeight = std::max(0.0f, std::stof(val));
            else if (key == "chat_fade_messages") chatPanel_.settings.fadeMessages = (std::stoi(val) != 0);
            else if (key == "chat_fade_time") chatPanel_.settings.messageFadeTime = std::clamp(std::stof(val), 5.0f, 120.0f);
            else if (key == "chat_autojoin_general") chatPanel_.chatAutoJoinGeneral = (std::stoi(val) != 0);
            else if (key == "chat_autojoin_trade") chatPanel_.chatAutoJoinTrade = (std::stoi(val) != 0);
            else if (key == "chat_autojoin_localdefense") chatPanel_.chatAutoJoinLocalDefense = (std::stoi(val) != 0);
            else if (key == "chat_autojoin_lfg") chatPanel_.chatAutoJoinLFG = (std::stoi(val) != 0);
            else if (key == "chat_autojoin_local") chatPanel_.chatAutoJoinLocal = (std::stoi(val) != 0);
        } catch (...) {}
    }

    if (!bagScaleLoaded) {
        const float defaultHeight = settingsPanel_.displaySettingsLoaded_
            ? static_cast<float>(settingsPanel_.pendingResolutionHeight)
            : currentDisplayHeight;
        settingsPanel_.pendingBagScale = InventoryScreen::recommendedBagScale(defaultHeight);
        inventoryScreen.setBagScale(settingsPanel_.pendingBagScale);
    }

    // Load keybindings from the same config file
    KeybindingManager::getInstance().loadFromConfigFile(path);

    // Apply immediately if services are already wired. The constructor loads
    // settings before renderer injection, so setServices() applies them again.
    applyCameraControlSettings();

    LOG_INFO("Settings loaded from ", path);
}

// ============================================================
// Mail Window
// ============================================================



// ============================================================
// Bank Window
// ============================================================


// ============================================================
// Guild Bank Window
// ============================================================


// ============================================================
// Auction House Window
// ============================================================



// ---------------------------------------------------------------------------
// Screen-space weather overlay (rain / snow / storm)
// ---------------------------------------------------------------------------
void GameScreen::renderWeatherOverlay(game::GameHandler& gameHandler) {
    uint32_t wType     = gameHandler.getWeatherType();
    float    intensity = gameHandler.getWeatherIntensity();
    if (wType == 0 || intensity < 0.05f) return;

    const ImGuiIO& io = ImGui::GetIO();
    float sw = io.DisplaySize.x;
    float sh = io.DisplaySize.y;
    if (sw <= 0.0f || sh <= 0.0f) return;

    // Seeded RNG for weather particle positions — replaces std::rand() which
    // shares global state and has modulo bias.
    static std::mt19937 wxRng(std::random_device{}());
    auto wxRandInt = [](int maxExcl) {
        return std::uniform_int_distribution<int>(0, std::max(0, maxExcl - 1))(wxRng);
    };

    ImDrawList* dl = ImGui::GetForegroundDrawList();
    const float dt = std::min(io.DeltaTime, 0.05f);   // cap delta at 50ms to avoid teleporting particles

    if (wType == 1 || wType == 3) {
        // ── Rain / Storm ─────────────────────────────────────────────────────
        constexpr int MAX_DROPS = 300;
        struct RainState {
            float x[MAX_DROPS], y[MAX_DROPS];
            bool  initialized = false;
            uint32_t lastType = 0;
            float lastW = 0.0f, lastH = 0.0f;
        };
        static RainState rs;

        // Re-seed if weather type or screen size changed
        if (!rs.initialized || rs.lastType != wType ||
            rs.lastW != sw   || rs.lastH != sh) {
            for (int i = 0; i < MAX_DROPS; ++i) {
                rs.x[i] = static_cast<float>(wxRandInt(static_cast<int>(sw) + 200)) - 100.0f;
                rs.y[i] = static_cast<float>(wxRandInt(static_cast<int>(sh)));
            }
            rs.initialized = true;
            rs.lastType = wType;
            rs.lastW = sw;
            rs.lastH = sh;
        }

        const float fallSpeed = (wType == 3) ? 680.0f : 440.0f;
        const float windSpeed = (wType == 3) ? 110.0f :  65.0f;
        const int   numDrops  = static_cast<int>(MAX_DROPS * std::min(1.0f, intensity));
        const float alpha     = std::min(1.0f, 0.28f + intensity * 0.38f);
        const uint8_t alphaU8 = static_cast<uint8_t>(alpha * 255.0f);
        const ImU32  dropCol  = IM_COL32(175, 195, 225, alphaU8);
        const float  dropLen  = 7.0f + intensity * 7.0f;
        // Normalised wind direction for the trail endpoint
        const float invSpeed  = 1.0f / std::sqrt(fallSpeed * fallSpeed + windSpeed * windSpeed);
        const float trailDx   = -windSpeed * invSpeed * dropLen;
        const float trailDy   = -fallSpeed * invSpeed * dropLen;

        for (int i = 0; i < numDrops; ++i) {
            rs.x[i] += windSpeed * dt;
            rs.y[i] += fallSpeed * dt;
            if (rs.y[i] > sh + 10.0f) {
                rs.y[i] = -10.0f;
                rs.x[i] = static_cast<float>(wxRandInt(static_cast<int>(sw) + 200)) - 100.0f;
            }
            if (rs.x[i] > sw + 100.0f) rs.x[i] -= sw + 200.0f;
            dl->AddLine(ImVec2(rs.x[i], rs.y[i]),
                        ImVec2(rs.x[i] + trailDx, rs.y[i] + trailDy),
                        dropCol, 1.0f);
        }

        // Storm: dark fog-vignette at screen edges
        if (wType == 3) {
            const float vigAlpha = std::min(1.0f, 0.12f + intensity * 0.18f);
            const ImU32 vigCol   = IM_COL32(60, 65, 80, static_cast<uint8_t>(vigAlpha * 255.0f));
            const float vigW = sw * 0.22f;
            const float vigH = sh * 0.22f;
            dl->AddRectFilledMultiColor(ImVec2(0,       0),      ImVec2(vigW, sh),     vigCol, IM_COL32_BLACK_TRANS, IM_COL32_BLACK_TRANS, vigCol);
            dl->AddRectFilledMultiColor(ImVec2(sw-vigW, 0),      ImVec2(sw,   sh),     IM_COL32_BLACK_TRANS, vigCol, vigCol, IM_COL32_BLACK_TRANS);
            dl->AddRectFilledMultiColor(ImVec2(0,       0),      ImVec2(sw,   vigH),   vigCol, vigCol, IM_COL32_BLACK_TRANS, IM_COL32_BLACK_TRANS);
            dl->AddRectFilledMultiColor(ImVec2(0,       sh-vigH),ImVec2(sw,   sh),     IM_COL32_BLACK_TRANS, IM_COL32_BLACK_TRANS, vigCol, vigCol);
        }

    } else if (wType == 2) {
        // ── Snow ─────────────────────────────────────────────────────────────
        constexpr int MAX_FLAKES = 120;
        struct SnowState {
            float x[MAX_FLAKES], y[MAX_FLAKES], phase[MAX_FLAKES];
            bool  initialized = false;
            float lastW = 0.0f, lastH = 0.0f;
        };
        static SnowState ss;

        if (!ss.initialized || ss.lastW != sw || ss.lastH != sh) {
            for (int i = 0; i < MAX_FLAKES; ++i) {
                ss.x[i]     = static_cast<float>(wxRandInt(static_cast<int>(sw)));
                ss.y[i]     = static_cast<float>(wxRandInt(static_cast<int>(sh)));
                ss.phase[i] = static_cast<float>(wxRandInt(628)) * 0.01f;
            }
            ss.initialized = true;
            ss.lastW = sw;
            ss.lastH = sh;
        }

        const float fallSpeed = 45.0f + intensity * 45.0f;
        const int   numFlakes = static_cast<int>(MAX_FLAKES * std::min(1.0f, intensity));
        const float alpha     = std::min(1.0f, 0.5f + intensity * 0.3f);
        const uint8_t alphaU8 = static_cast<uint8_t>(alpha * 255.0f);
        const float   radius  = 1.5f + intensity * 1.5f;
        const float   time    = static_cast<float>(ImGui::GetTime());

        for (int i = 0; i < numFlakes; ++i) {
            float sway = std::sin(time * 0.7f + ss.phase[i]) * 18.0f;
            ss.x[i] += sway * dt;
            ss.y[i] += fallSpeed * dt;
            ss.phase[i] += dt * 0.25f;
            if (ss.y[i] > sh + 5.0f) {
                ss.y[i] = -5.0f;
                ss.x[i] = static_cast<float>(wxRandInt(static_cast<int>(sw)));
            }
            if (ss.x[i] < -5.0f) ss.x[i] += sw + 10.0f;
            if (ss.x[i] > sw + 5.0f) ss.x[i] -= sw + 10.0f;
            // Two-tone: bright centre dot + transparent outer ring for depth
            dl->AddCircleFilled(ImVec2(ss.x[i], ss.y[i]), radius, IM_COL32(220, 235, 255, alphaU8));
            dl->AddCircleFilled(ImVec2(ss.x[i], ss.y[i]), radius * 0.45f, IM_COL32(245, 250, 255, std::min(255, alphaU8 + 30)));
        }
    }
}

// ---------------------------------------------------------------------------
// Dungeon Finder window (toggle with hotkey or bag-bar button)
// ---------------------------------------------------------------------------
// ============================================================
// Instance Lockouts
// ============================================================




// ─── Threat Window ────────────────────────────────────────────────────────────
// ─── BG Scoreboard ────────────────────────────────────────────────────────────






}} // namespace wowee::ui
