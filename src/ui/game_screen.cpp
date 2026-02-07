#include "ui/game_screen.hpp"
#include "rendering/character_preview.hpp"
#include "core/application.hpp"
#include "core/coordinates.hpp"
#include "core/spawn_presets.hpp"
#include "core/input.hpp"
#include "rendering/renderer.hpp"
#include "rendering/minimap.hpp"
#include "rendering/character_renderer.hpp"
#include "rendering/camera.hpp"
#include "rendering/camera_controller.hpp"
#include "audio/music_manager.hpp"
#include "audio/footstep_manager.hpp"
#include "audio/activity_sound_manager.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_loader.hpp"
#include "pipeline/blp_loader.hpp"
#include "core/logger.hpp"
#include <imgui.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <unordered_set>

namespace {
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
        }
        return "Unknown";
    }
}

namespace wowee { namespace ui {

GameScreen::GameScreen() {
}

void GameScreen::render(game::GameHandler& gameHandler) {
    // Process targeting input before UI windows
    processTargetInput(gameHandler);

    // Player unit frame (top-left)
    renderPlayerFrame(gameHandler);

    // Target frame (only when we have a target)
    if (gameHandler.hasTarget()) {
        renderTargetFrame(gameHandler);
    }

    // Render windows
    if (showPlayerInfo) {
        renderPlayerInfo(gameHandler);
    }

    if (showEntityWindow) {
        renderEntityList(gameHandler);
    }

    if (showChatWindow) {
        renderChatWindow(gameHandler);
    }

    // ---- New UI elements ----
    renderActionBar(gameHandler);
    renderXpBar(gameHandler);
    renderCastBar(gameHandler);
    renderCombatText(gameHandler);
    renderPartyFrames(gameHandler);
    renderGroupInvitePopup(gameHandler);
    renderBuffBar(gameHandler);
    renderLootWindow(gameHandler);
    renderGossipWindow(gameHandler);
    renderQuestDetailsWindow(gameHandler);
    renderVendorWindow(gameHandler);
    renderQuestMarkers(gameHandler);
    renderMinimapMarkers(gameHandler);
    renderDeathScreen(gameHandler);
    renderEscapeMenu();
    renderSettingsWindow();

    // World map (M key toggle handled inside)
    renderWorldMap(gameHandler);

    // Teleporter panel (T key toggle handled in Application event loop)
    renderTeleporterPanel();

    // Quest Log (L key toggle handled inside)
    questLogScreen.render(gameHandler);

    // Spellbook (P key toggle handled inside)
    spellbookScreen.render(gameHandler, core::Application::getInstance().getAssetManager());

    // Talents (N key toggle handled inside)
    talentScreen.render(gameHandler);

    // Set up inventory screen asset manager + player appearance (once)
    {
        static bool inventoryScreenInit = false;
        if (!inventoryScreenInit) {
            auto* am = core::Application::getInstance().getAssetManager();
            if (am) {
                inventoryScreen.setAssetManager(am);
                const auto* ch = gameHandler.getActiveCharacter();
                if (ch) {
                    uint8_t skin = ch->appearanceBytes & 0xFF;
                    uint8_t face = (ch->appearanceBytes >> 8) & 0xFF;
                    uint8_t hairStyle = (ch->appearanceBytes >> 16) & 0xFF;
                    uint8_t hairColor = (ch->appearanceBytes >> 24) & 0xFF;
                    inventoryScreen.setPlayerAppearance(
                        ch->race, ch->gender, skin, face,
                        hairStyle, hairColor, ch->facialFeatures);
                    inventoryScreenInit = true;
                }
            }
        }
    }

    // Set vendor mode before rendering inventory
    inventoryScreen.setVendorMode(gameHandler.isVendorWindowOpen(), &gameHandler);

    // Auto-open bags when vendor window opens
    if (gameHandler.isVendorWindowOpen() && !inventoryScreen.isOpen()) {
        inventoryScreen.setOpen(true);
    }

    // Bags (B key toggle handled inside)
    inventoryScreen.setGameHandler(&gameHandler);
    inventoryScreen.render(gameHandler.getInventory(), gameHandler.getMoneyCopper());

    // Character screen (C key toggle handled inside render())
    inventoryScreen.renderCharacterScreen(gameHandler);

    if (inventoryScreen.consumeInventoryDirty()) {
        gameHandler.notifyInventoryChanged();
    }

    if (inventoryScreen.consumeEquipmentDirty() || gameHandler.consumeOnlineEquipmentDirty()) {
        updateCharacterGeosets(gameHandler.getInventory());
        updateCharacterTextures(gameHandler.getInventory());
        core::Application::getInstance().loadEquippedWeapons();
        gameHandler.notifyEquipmentChanged();
        inventoryScreen.markPreviewDirty();
        // Update renderer weapon type for animation selection
        auto* r = core::Application::getInstance().getRenderer();
        if (r) {
            const auto& mh = gameHandler.getInventory().getEquipSlot(game::EquipSlot::MAIN_HAND);
            r->setEquippedWeaponType(mh.empty() ? 0 : mh.item.inventoryType);
        }
    }

    // Update renderer face-target position and selection circle
    auto* renderer = core::Application::getInstance().getRenderer();
    if (renderer) {
        static glm::vec3 targetGLPos;
        if (gameHandler.hasTarget()) {
            auto target = gameHandler.getTarget();
            if (target) {
                targetGLPos = core::coords::canonicalToRender(glm::vec3(target->getX(), target->getY(), target->getZ()));
                renderer->setTargetPosition(&targetGLPos);

                // Selection circle color: WoW-canonical level-based colors
                glm::vec3 circleColor(1.0f, 1.0f, 0.3f); // default yellow
                float circleRadius = 1.5f;
                {
                    glm::vec3 boundsCenter;
                    float boundsRadius = 0.0f;
                    if (core::Application::getInstance().getRenderBoundsForGuid(target->getGuid(), boundsCenter, boundsRadius)) {
                        float r = boundsRadius * 1.1f;
                        circleRadius = std::min(std::max(r, 0.8f), 8.0f);
                    }
                }
                if (target->getType() == game::ObjectType::UNIT) {
                    auto unit = std::static_pointer_cast<game::Unit>(target);
                    if (unit->getHealth() == 0 && unit->getMaxHealth() > 0) {
                        circleColor = glm::vec3(0.5f, 0.5f, 0.5f); // gray (dead)
                    } else if (unit->isHostile() || gameHandler.isAggressiveTowardPlayer(target->getGuid())) {
                        uint32_t playerLv = gameHandler.getPlayerLevel();
                        uint32_t mobLv = unit->getLevel();
                        int32_t diff = static_cast<int32_t>(mobLv) - static_cast<int32_t>(playerLv);
                        if (game::GameHandler::killXp(playerLv, mobLv) == 0) {
                            circleColor = glm::vec3(0.6f, 0.6f, 0.6f); // grey
                        } else if (diff >= 10) {
                            circleColor = glm::vec3(1.0f, 0.1f, 0.1f); // red
                        } else if (diff >= 5) {
                            circleColor = glm::vec3(1.0f, 0.5f, 0.1f); // orange
                        } else if (diff >= -2) {
                            circleColor = glm::vec3(1.0f, 1.0f, 0.1f); // yellow
                        } else {
                            circleColor = glm::vec3(0.3f, 1.0f, 0.3f); // green
                        }
                    } else {
                        circleColor = glm::vec3(0.3f, 1.0f, 0.3f); // green (friendly)
                    }
                } else if (target->getType() == game::ObjectType::PLAYER) {
                    circleColor = glm::vec3(0.3f, 1.0f, 0.3f); // green (player)
                }
                renderer->setSelectionCircle(targetGLPos, circleRadius, circleColor);
            } else {
                renderer->setTargetPosition(nullptr);
                renderer->clearSelectionCircle();
            }
        } else {
            renderer->setTargetPosition(nullptr);
            renderer->clearSelectionCircle();
        }
    }
}

void GameScreen::renderPlayerInfo(game::GameHandler& gameHandler) {
    ImGui::SetNextWindowSize(ImVec2(350, 250), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(10, 30), ImGuiCond_FirstUseEver);
    ImGui::Begin("Player Info", &showPlayerInfo);

    const auto& movement = gameHandler.getMovementInfo();

    ImGui::Text("Position & Movement");
    ImGui::Separator();
    ImGui::Spacing();

    // Position
    ImGui::Text("Position:");
    ImGui::Indent();
    ImGui::Text("X: %.2f", movement.x);
    ImGui::Text("Y: %.2f", movement.y);
    ImGui::Text("Z: %.2f", movement.z);
    ImGui::Text("Orientation: %.2f rad (%.1f deg)", movement.orientation, movement.orientation * 180.0f / 3.14159f);
    ImGui::Unindent();

    ImGui::Spacing();

    // Movement flags
    ImGui::Text("Movement Flags: 0x%08X", movement.flags);
    ImGui::Text("Time: %u ms", movement.time);

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Connection state
    ImGui::Text("Connection State:");
    ImGui::Indent();
    auto state = gameHandler.getState();
    switch (state) {
        case game::WorldState::IN_WORLD:
            ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), "In World");
            break;
        case game::WorldState::AUTHENTICATED:
            ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.3f, 1.0f), "Authenticated");
            break;
        case game::WorldState::ENTERING_WORLD:
            ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.3f, 1.0f), "Entering World...");
            break;
        default:
            ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "State: %d", static_cast<int>(state));
            break;
    }
    ImGui::Unindent();

    ImGui::End();
}

void GameScreen::renderEntityList(game::GameHandler& gameHandler) {
    ImGui::SetNextWindowSize(ImVec2(500, 400), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(10, 290), ImGuiCond_FirstUseEver);
    ImGui::Begin("Entities", &showEntityWindow);

    const auto& entityManager = gameHandler.getEntityManager();
    const auto& entities = entityManager.getEntities();

    ImGui::Text("Entities in View: %zu", entities.size());
    ImGui::Separator();
    ImGui::Spacing();

    if (entities.empty()) {
        ImGui::TextDisabled("No entities in view");
    } else {
        // Entity table
        if (ImGui::BeginTable("EntitiesTable", 5, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY)) {
            ImGui::TableSetupColumn("GUID", ImGuiTableColumnFlags_WidthFixed, 140.0f);
            ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed, 100.0f);
            ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("Position", ImGuiTableColumnFlags_WidthFixed, 150.0f);
            ImGui::TableSetupColumn("Distance", ImGuiTableColumnFlags_WidthFixed, 80.0f);
            ImGui::TableHeadersRow();

            const auto& playerMovement = gameHandler.getMovementInfo();
            float playerX = playerMovement.x;
            float playerY = playerMovement.y;
            float playerZ = playerMovement.z;

            for (const auto& [guid, entity] : entities) {
                ImGui::TableNextRow();

                // GUID
                ImGui::TableSetColumnIndex(0);
                char guidStr[24];
                snprintf(guidStr, sizeof(guidStr), "0x%016llX", (unsigned long long)guid);
                ImGui::Text("%s", guidStr);

                // Type
                ImGui::TableSetColumnIndex(1);
                switch (entity->getType()) {
                    case game::ObjectType::PLAYER:
                        ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), "Player");
                        break;
                    case game::ObjectType::UNIT:
                        ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.3f, 1.0f), "Unit");
                        break;
                    case game::ObjectType::GAMEOBJECT:
                        ImGui::TextColored(ImVec4(0.3f, 0.8f, 1.0f, 1.0f), "GameObject");
                        break;
                    default:
                        ImGui::Text("Object");
                        break;
                }

                // Name (for players and units)
                ImGui::TableSetColumnIndex(2);
                if (entity->getType() == game::ObjectType::PLAYER) {
                    auto player = std::static_pointer_cast<game::Player>(entity);
                    ImGui::Text("%s", player->getName().c_str());
                } else if (entity->getType() == game::ObjectType::UNIT) {
                    auto unit = std::static_pointer_cast<game::Unit>(entity);
                    if (!unit->getName().empty()) {
                        ImGui::Text("%s", unit->getName().c_str());
                    } else {
                        ImGui::TextDisabled("--");
                    }
                } else {
                    ImGui::TextDisabled("--");
                }

                // Position
                ImGui::TableSetColumnIndex(3);
                ImGui::Text("%.1f, %.1f, %.1f", entity->getX(), entity->getY(), entity->getZ());

                // Distance from player
                ImGui::TableSetColumnIndex(4);
                float dx = entity->getX() - playerX;
                float dy = entity->getY() - playerY;
                float dz = entity->getZ() - playerZ;
                float distance = std::sqrt(dx*dx + dy*dy + dz*dz);
                ImGui::Text("%.1f", distance);
            }

            ImGui::EndTable();
        }
    }

    ImGui::End();
}

void GameScreen::renderChatWindow(game::GameHandler& gameHandler) {
    auto* window = core::Application::getInstance().getWindow();
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;
    float chatW = std::min(500.0f, screenW * 0.4f);
    float chatH = 220.0f;
    float chatX = 8.0f;
    float chatY = screenH - chatH - 80.0f;  // Above action bar
    if (!chatWindowPosInit_) {
        chatWindowPos_ = ImVec2(chatX, chatY);
        chatWindowPosInit_ = true;
    }
    if (chatWindowLocked) {
        ImGui::SetNextWindowSize(ImVec2(chatW, chatH), ImGuiCond_Always);
        ImGui::SetNextWindowPos(chatWindowPos_, ImGuiCond_Always);
    } else {
        ImGui::SetNextWindowSize(ImVec2(chatW, chatH), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowPos(chatWindowPos_, ImGuiCond_FirstUseEver);
    }
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize;
    if (chatWindowLocked) flags |= ImGuiWindowFlags_NoMove;
    ImGui::Begin("Chat", nullptr, flags);

    if (!chatWindowLocked) {
        chatWindowPos_ = ImGui::GetWindowPos();
    }

    // Chat history
    const auto& chatHistory = gameHandler.getChatHistory();

    ImGui::BeginChild("ChatHistory", ImVec2(0, -70), true, ImGuiWindowFlags_HorizontalScrollbar);

    for (const auto& msg : chatHistory) {
        ImVec4 color = getChatTypeColor(msg.type);
        ImGui::PushStyleColor(ImGuiCol_Text, color);

        if (msg.type == game::ChatType::SYSTEM) {
            // System messages: just yellow text, no header
            ImGui::TextWrapped("%s", msg.message.c_str());
        } else if (msg.type == game::ChatType::TEXT_EMOTE) {
            ImGui::TextWrapped("You %s", msg.message.c_str());
        } else if (!msg.senderName.empty()) {
            if (msg.type == game::ChatType::MONSTER_SAY || msg.type == game::ChatType::MONSTER_YELL) {
                ImGui::TextWrapped("%s says: %s", msg.senderName.c_str(), msg.message.c_str());
            } else {
                ImGui::TextWrapped("[%s] %s: %s", getChatTypeName(msg.type), msg.senderName.c_str(), msg.message.c_str());
            }
        } else {
            ImGui::TextWrapped("[%s] %s", getChatTypeName(msg.type), msg.message.c_str());
        }
        ImGui::PopStyleColor();
    }

    // Auto-scroll to bottom
    if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY()) {
        ImGui::SetScrollHereY(1.0f);
    }

    ImGui::EndChild();

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Lock toggle
    ImGui::Checkbox("Lock", &chatWindowLocked);
    ImGui::SameLine();
    ImGui::TextDisabled(chatWindowLocked ? "(locked)" : "(movable)");

    // Chat input
    ImGui::Text("Type:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100);
    const char* chatTypes[] = { "SAY", "YELL", "PARTY", "GUILD" };
    ImGui::Combo("##ChatType", &selectedChatType, chatTypes, 4);

    ImGui::SameLine();
    ImGui::Text("Message:");
    ImGui::SameLine();

    ImGui::SetNextItemWidth(-1);
    if (refocusChatInput) {
        ImGui::SetKeyboardFocusHere();
        refocusChatInput = false;
    }
    auto inputCallback = [](ImGuiInputTextCallbackData* data) -> int {
        auto* self = static_cast<GameScreen*>(data->UserData);
        if (self && self->chatInputMoveCursorToEnd) {
            int len = static_cast<int>(std::strlen(data->Buf));
            data->CursorPos = len;
            data->SelectionStart = len;
            data->SelectionEnd = len;
            self->chatInputMoveCursorToEnd = false;
        }
        return 0;
    };

    ImGuiInputTextFlags inputFlags = ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_CallbackAlways;
    if (ImGui::InputText("##ChatInput", chatInputBuffer, sizeof(chatInputBuffer), inputFlags, inputCallback, this)) {
        sendChatMessage(gameHandler);
        refocusChatInput = true;
    }

    if (ImGui::IsItemActive()) {
        chatInputActive = true;
    } else {
        chatInputActive = false;
    }

    ImGui::End();
}

void GameScreen::processTargetInput(game::GameHandler& gameHandler) {
    auto& io = ImGui::GetIO();
    auto& input = core::Input::getInstance();

    // Tab targeting (when keyboard not captured by UI)
    if (!io.WantCaptureKeyboard) {
        if (input.isKeyJustPressed(SDL_SCANCODE_TAB)) {
            const auto& movement = gameHandler.getMovementInfo();
            gameHandler.tabTarget(movement.x, movement.y, movement.z);
        }

        if (input.isKeyJustPressed(SDL_SCANCODE_ESCAPE)) {
            if (showEscapeMenu) {
                showEscapeMenu = false;
                showEscapeSettingsNotice = false;
                showSettingsWindow = false;
            } else if (showTeleporter) {
                showTeleporter = false;
            } else if (gameHandler.isCasting()) {
                gameHandler.cancelCast();
            } else if (gameHandler.isLootWindowOpen()) {
                gameHandler.closeLoot();
            } else if (gameHandler.isGossipWindowOpen()) {
                gameHandler.closeGossip();
            } else {
                showEscapeMenu = true;
            }
        }

        // Action bar keys (1-9, 0, -, =)
        static const SDL_Scancode actionBarKeys[] = {
            SDL_SCANCODE_1, SDL_SCANCODE_2, SDL_SCANCODE_3, SDL_SCANCODE_4,
            SDL_SCANCODE_5, SDL_SCANCODE_6, SDL_SCANCODE_7, SDL_SCANCODE_8,
            SDL_SCANCODE_9, SDL_SCANCODE_0, SDL_SCANCODE_MINUS, SDL_SCANCODE_EQUALS
        };
        for (int i = 0; i < 12; ++i) {
            if (input.isKeyJustPressed(actionBarKeys[i])) {
                const auto& bar = gameHandler.getActionBar();
                if (bar[i].type == game::ActionBarSlot::SPELL && bar[i].isReady()) {
                    uint64_t target = gameHandler.hasTarget() ? gameHandler.getTargetGuid() : 0;
                    gameHandler.castSpell(bar[i].id, target);
                } else if (bar[i].type == game::ActionBarSlot::ITEM && bar[i].id != 0) {
                    gameHandler.useItemById(bar[i].id);
                }
            }
        }

    }

    // Slash key: focus chat input
    if (!io.WantCaptureKeyboard && input.isKeyJustPressed(SDL_SCANCODE_SLASH)) {
        refocusChatInput = true;
        chatInputBuffer[0] = '/';
        chatInputBuffer[1] = '\0';
        chatInputMoveCursorToEnd = true;
    }

    // Enter key: focus chat input (empty)
    if (!io.WantCaptureKeyboard && input.isKeyJustPressed(SDL_SCANCODE_RETURN)) {
        refocusChatInput = true;
    }

    // Left-click targeting (when mouse not captured by UI)
    // Suppress when right button is held (both-button run)
    if (!io.WantCaptureMouse && input.isMouseButtonJustPressed(SDL_BUTTON_LEFT) && !input.isMouseButtonPressed(SDL_BUTTON_RIGHT)) {
        auto* renderer = core::Application::getInstance().getRenderer();
        auto* camera = renderer ? renderer->getCamera() : nullptr;
        auto* window = core::Application::getInstance().getWindow();

        if (camera && window) {
            glm::vec2 mousePos = input.getMousePosition();
            float screenW = static_cast<float>(window->getWidth());
            float screenH = static_cast<float>(window->getHeight());

            rendering::Ray ray = camera->screenToWorldRay(mousePos.x, mousePos.y, screenW, screenH);

            float closestT = 1e30f;
            uint64_t closestGuid = 0;

            const uint64_t myGuid = gameHandler.getPlayerGuid();
            for (const auto& [guid, entity] : gameHandler.getEntityManager().getEntities()) {
                auto t = entity->getType();
                if (t != game::ObjectType::UNIT && t != game::ObjectType::PLAYER) continue;
                if (guid == myGuid) continue;  // Don't target self

                glm::vec3 hitCenter;
                float hitRadius = 0.0f;
                bool hasBounds = core::Application::getInstance().getRenderBoundsForGuid(guid, hitCenter, hitRadius);
                if (!hasBounds) {
                    // Fallback hitbox based on entity type
                    float heightOffset = 1.5f;
                    hitRadius = 1.5f;
                    if (t == game::ObjectType::UNIT) {
                        auto unit = std::static_pointer_cast<game::Unit>(entity);
                        // Critters have very low max health (< 100)
                        if (unit->getMaxHealth() > 0 && unit->getMaxHealth() < 100) {
                            hitRadius = 0.5f;
                            heightOffset = 0.3f;
                        }
                    }
                    hitCenter = core::coords::canonicalToRender(glm::vec3(entity->getX(), entity->getY(), entity->getZ()));
                    hitCenter.z += heightOffset;
                } else {
                    hitRadius = std::max(hitRadius * 1.1f, 0.6f);
                }

                float hitT;
                if (raySphereIntersect(ray, hitCenter, hitRadius, hitT)) {
                    if (hitT < closestT) {
                        closestT = hitT;
                        closestGuid = guid;
                    }
                }
            }

            if (closestGuid != 0) {
                gameHandler.setTarget(closestGuid);
            } else {
                // Clicked empty space — deselect current target
                gameHandler.clearTarget();
            }
        }
    }

    // Right-click: select NPC (if needed) then interact / loot / auto-attack
    // Suppress when left button is held (both-button run)
    if (!io.WantCaptureMouse && input.isMouseButtonJustPressed(SDL_BUTTON_RIGHT) && !input.isMouseButtonPressed(SDL_BUTTON_LEFT)) {
        // If no target or right-clicking in world, try to pick one under cursor
        {
            auto* renderer = core::Application::getInstance().getRenderer();
            auto* camera = renderer ? renderer->getCamera() : nullptr;
            auto* window = core::Application::getInstance().getWindow();
            if (camera && window) {
                glm::vec2 mousePos = input.getMousePosition();
                float screenW = static_cast<float>(window->getWidth());
                float screenH = static_cast<float>(window->getHeight());
                rendering::Ray ray = camera->screenToWorldRay(mousePos.x, mousePos.y, screenW, screenH);
                float closestT = 1e30f;
                uint64_t closestGuid = 0;
                const uint64_t myGuid = gameHandler.getPlayerGuid();
                for (const auto& [guid, entity] : gameHandler.getEntityManager().getEntities()) {
                    auto t = entity->getType();
                    if (t != game::ObjectType::UNIT && t != game::ObjectType::PLAYER) continue;
                    if (guid == myGuid) continue;
                    glm::vec3 hitCenter;
                    float hitRadius = 0.0f;
                    bool hasBounds = core::Application::getInstance().getRenderBoundsForGuid(guid, hitCenter, hitRadius);
                    if (!hasBounds) {
                        float heightOffset = 1.5f;
                        hitRadius = 1.5f;
                        if (t == game::ObjectType::UNIT) {
                            auto unit = std::static_pointer_cast<game::Unit>(entity);
                            if (unit->getMaxHealth() > 0 && unit->getMaxHealth() < 100) {
                                hitRadius = 0.5f;
                                heightOffset = 0.3f;
                            }
                        }
                        hitCenter = core::coords::canonicalToRender(
                            glm::vec3(entity->getX(), entity->getY(), entity->getZ()));
                        hitCenter.z += heightOffset;
                    } else {
                        hitRadius = std::max(hitRadius * 1.1f, 0.6f);
                    }
                    float hitT;
                    if (raySphereIntersect(ray, hitCenter, hitRadius, hitT)) {
                        if (hitT < closestT) {
                            closestT = hitT;
                            closestGuid = guid;
                        }
                    }
                }
                if (closestGuid != 0) {
                    gameHandler.setTarget(closestGuid);
                }
            }
        }
        if (gameHandler.hasTarget()) {
            auto target = gameHandler.getTarget();
            if (target) {
                if (target->getType() == game::ObjectType::UNIT) {
                    // Check if unit is dead (health == 0) → loot, otherwise interact/attack
                    auto unit = std::static_pointer_cast<game::Unit>(target);
                    if (unit->getHealth() == 0 && unit->getMaxHealth() > 0) {
                        gameHandler.lootTarget(target->getGuid());
                    } else if (gameHandler.isSinglePlayerMode()) {
                        // Single-player: interact with friendly NPCs, otherwise attack
                        if (!unit->isHostile() && unit->isInteractable()) {
                            gameHandler.interactWithNpc(target->getGuid());
                        } else {
                            gameHandler.startAutoAttack(target->getGuid());
                        }
                    } else {
                        // Online mode: interact with friendly NPCs, otherwise attack
                        if (!unit->isHostile() && unit->isInteractable()) {
                            gameHandler.interactWithNpc(target->getGuid());
                        } else {
                            gameHandler.startAutoAttack(target->getGuid());
                        }
                    }
                } else if (target->getType() == game::ObjectType::PLAYER) {
                    // Right-click another player could start attack in PvP context
                }
            }
        }
    }
}

void GameScreen::renderPlayerFrame(game::GameHandler& gameHandler) {
    bool isDead = gameHandler.isPlayerDead();
    ImGui::SetNextWindowPos(ImVec2(10.0f, 30.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(250.0f, 0.0f), ImGuiCond_Always);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                             ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_AlwaysAutoResize;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.1f, 0.1f, 0.85f));
    ImVec4 playerBorder = isDead
        ? ImVec4(0.5f, 0.5f, 0.5f, 1.0f)
        : (gameHandler.isAutoAttacking()
            ? ImVec4(1.0f, 0.2f, 0.2f, 1.0f)
            : ImVec4(0.4f, 0.4f, 0.4f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_Border, playerBorder);

    if (ImGui::Begin("##PlayerFrame", nullptr, flags)) {
        // Use selected character info if available, otherwise defaults
        std::string playerName = "Adventurer";
        uint32_t playerLevel = 1;
        uint32_t playerHp = 100;
        uint32_t playerMaxHp = 100;

        const auto& characters = gameHandler.getCharacters();
        if (!characters.empty()) {
            const auto& ch = characters[0];
            playerName = ch.name;
            // Use live server level if available, otherwise character struct
            playerLevel = gameHandler.getPlayerLevel();
            if (playerLevel == 0) playerLevel = ch.level;
            playerMaxHp = 20 + playerLevel * 10;
            playerHp = playerMaxHp;
        }

        // Name in green (friendly player color) — clickable for self-target
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.3f, 1.0f, 0.3f, 1.0f));
        if (ImGui::Selectable(playerName.c_str(), false, 0, ImVec2(0, 0))) {
            gameHandler.setTarget(gameHandler.getPlayerGuid());
        }
        ImGui::PopStyleColor();
        ImGui::SameLine();
        ImGui::TextDisabled("Lv %u", playerLevel);
        if (isDead) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.9f, 0.2f, 0.2f, 1.0f), "DEAD");
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

        // Override with local player stats in single-player mode
        if (gameHandler.isSinglePlayerMode() && gameHandler.getLocalPlayerMaxHealth() > 0) {
            playerHp = gameHandler.getLocalPlayerHealth();
            playerMaxHp = gameHandler.getLocalPlayerMaxHealth();
        }

        // Health bar
        float pct = static_cast<float>(playerHp) / static_cast<float>(playerMaxHp);
        ImVec4 hpColor = isDead ? ImVec4(0.5f, 0.5f, 0.5f, 1.0f) : ImVec4(0.2f, 0.8f, 0.2f, 1.0f);
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, hpColor);
        char overlay[64];
        snprintf(overlay, sizeof(overlay), "%u / %u", playerHp, playerMaxHp);
        ImGui::ProgressBar(pct, ImVec2(-1, 18), overlay);
        ImGui::PopStyleColor();

        // Mana/Power bar (Phase 2)
        if (playerEntity && (playerEntity->getType() == game::ObjectType::PLAYER || playerEntity->getType() == game::ObjectType::UNIT)) {
            auto unit = std::static_pointer_cast<game::Unit>(playerEntity);
            uint32_t power = unit->getPower();
            uint32_t maxPower = unit->getMaxPower();
            if (maxPower > 0) {
                float mpPct = static_cast<float>(power) / static_cast<float>(maxPower);
                // Color by power type
                ImVec4 powerColor;
                switch (unit->getPowerType()) {
                    case 0: powerColor = ImVec4(0.2f, 0.2f, 0.9f, 1.0f); break; // Mana (blue)
                    case 1: powerColor = ImVec4(0.9f, 0.2f, 0.2f, 1.0f); break; // Rage (red)
                    case 3: powerColor = ImVec4(0.9f, 0.9f, 0.2f, 1.0f); break; // Energy (yellow)
                    default: powerColor = ImVec4(0.2f, 0.2f, 0.9f, 1.0f); break;
                }
                ImGui::PushStyleColor(ImGuiCol_PlotHistogram, powerColor);
                char mpOverlay[64];
                snprintf(mpOverlay, sizeof(mpOverlay), "%u / %u", power, maxPower);
                ImGui::ProgressBar(mpPct, ImVec2(-1, 14), mpOverlay);
                ImGui::PopStyleColor();
            }
        }
    }
    ImGui::End();

    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar();
}

void GameScreen::renderTargetFrame(game::GameHandler& gameHandler) {
    auto target = gameHandler.getTarget();
    if (!target) return;

    auto* window = core::Application::getInstance().getWindow();
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;

    float frameW = 250.0f;
    float frameX = (screenW - frameW) / 2.0f;

    ImGui::SetNextWindowPos(ImVec2(frameX, 30.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(frameW, 0.0f), ImGuiCond_Always);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                             ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_AlwaysAutoResize;

    // Determine hostility/level color for border and name (WoW-canonical)
    ImVec4 hostileColor(0.7f, 0.7f, 0.7f, 1.0f);
    if (target->getType() == game::ObjectType::PLAYER) {
        hostileColor = ImVec4(0.3f, 1.0f, 0.3f, 1.0f);
    } else if (target->getType() == game::ObjectType::UNIT) {
        auto u = std::static_pointer_cast<game::Unit>(target);
        if (u->getHealth() == 0 && u->getMaxHealth() > 0) {
            hostileColor = ImVec4(0.5f, 0.5f, 0.5f, 1.0f);
        } else if (u->isHostile()) {
            // WoW level-based color for hostile mobs
            uint32_t playerLv = gameHandler.getPlayerLevel();
            uint32_t mobLv = u->getLevel();
            int32_t diff = static_cast<int32_t>(mobLv) - static_cast<int32_t>(playerLv);
            if (game::GameHandler::killXp(playerLv, mobLv) == 0) {
                hostileColor = ImVec4(0.6f, 0.6f, 0.6f, 1.0f); // Grey - no XP
            } else if (diff >= 10) {
                hostileColor = ImVec4(1.0f, 0.1f, 0.1f, 1.0f); // Red - skull/very hard
            } else if (diff >= 5) {
                hostileColor = ImVec4(1.0f, 0.5f, 0.1f, 1.0f); // Orange - hard
            } else if (diff >= -2) {
                hostileColor = ImVec4(1.0f, 1.0f, 0.1f, 1.0f); // Yellow - even
            } else {
                hostileColor = ImVec4(0.3f, 1.0f, 0.3f, 1.0f); // Green - easy
            }
        } else {
            hostileColor = ImVec4(0.3f, 1.0f, 0.3f, 1.0f); // Friendly
        }
    }

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.1f, 0.1f, 0.85f));
    ImVec4 borderColor = ImVec4(hostileColor.x * 0.8f, hostileColor.y * 0.8f, hostileColor.z * 0.8f, 1.0f);
    if (gameHandler.isAutoAttacking()) {
        borderColor = ImVec4(1.0f, 0.2f, 0.2f, 1.0f);
    }
    ImGui::PushStyleColor(ImGuiCol_Border, borderColor);

    if (ImGui::Begin("##TargetFrame", nullptr, flags)) {
        // Entity name and type
        std::string name = getEntityName(target);

        ImVec4 nameColor = hostileColor;

        ImGui::TextColored(nameColor, "%s", name.c_str());

        // Level (for units/players) — colored by difficulty
        if (target->getType() == game::ObjectType::UNIT || target->getType() == game::ObjectType::PLAYER) {
            auto unit = std::static_pointer_cast<game::Unit>(target);
            ImGui::SameLine();
            // Level color matches the hostility/difficulty color
            ImVec4 levelColor = hostileColor;
            if (target->getType() == game::ObjectType::PLAYER) {
                levelColor = ImVec4(0.7f, 0.7f, 0.7f, 1.0f);
            }
            ImGui::TextColored(levelColor, "Lv %u", unit->getLevel());

            // Health bar
            uint32_t hp = unit->getHealth();
            uint32_t maxHp = unit->getMaxHealth();
            if (maxHp > 0) {
                float pct = static_cast<float>(hp) / static_cast<float>(maxHp);
                ImGui::PushStyleColor(ImGuiCol_PlotHistogram,
                    pct > 0.5f ? ImVec4(0.2f, 0.8f, 0.2f, 1.0f) :
                    pct > 0.2f ? ImVec4(0.8f, 0.8f, 0.2f, 1.0f) :
                                 ImVec4(0.8f, 0.2f, 0.2f, 1.0f));

                char overlay[64];
                snprintf(overlay, sizeof(overlay), "%u / %u", hp, maxHp);
                ImGui::ProgressBar(pct, ImVec2(-1, 18), overlay);
                ImGui::PopStyleColor();
                // Target mana bar
                uint32_t targetPower = unit->getPower();
                uint32_t targetMaxPower = unit->getMaxPower();
                if (targetMaxPower > 0) {
                    float mpPct = static_cast<float>(targetPower) / static_cast<float>(targetMaxPower);
                    ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.2f, 0.2f, 0.9f, 1.0f));
                    char mpOverlay[64];
                    snprintf(mpOverlay, sizeof(mpOverlay), "%u / %u", targetPower, targetMaxPower);
                    ImGui::ProgressBar(mpPct, ImVec2(-1, 14), mpOverlay);
                    ImGui::PopStyleColor();
                }
            } else {
                ImGui::TextDisabled("No health data");
            }
        }

        // Distance
        const auto& movement = gameHandler.getMovementInfo();
        float dx = target->getX() - movement.x;
        float dy = target->getY() - movement.y;
        float dz = target->getZ() - movement.z;
        float distance = std::sqrt(dx*dx + dy*dy + dz*dz);
        ImGui::TextDisabled("%.1f yd", distance);
    }
    ImGui::End();

    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar();
}

void GameScreen::sendChatMessage(game::GameHandler& gameHandler) {
    if (strlen(chatInputBuffer) > 0) {
        std::string input(chatInputBuffer);

        // Check for slash command emotes
        if (input.size() > 1 && input[0] == '/') {
            std::string command = input.substr(1);
            // Convert to lowercase
            for (char& c : command) c = std::tolower(c);

            if (command == "logout") {
                core::Application::getInstance().logoutToLogin();
                chatInputBuffer[0] = '\0';
                return;
            }

            std::string emoteText = rendering::Renderer::getEmoteText(command);
            if (!emoteText.empty()) {
                // Play the emote animation
                auto* renderer = core::Application::getInstance().getRenderer();
                if (renderer) {
                    renderer->playEmote(command);
                }

                // Build emote message — targeted or untargeted
                std::string chatText;
                if (gameHandler.hasTarget()) {
                    auto target = gameHandler.getTarget();
                    if (target) {
                        std::string targetName = getEntityName(target);
                        chatText = command + " at " + targetName + ".";
                    } else {
                        chatText = emoteText;
                    }
                } else {
                    chatText = command + ".";  // First person: "You wave."
                }

                // Add local chat message
                game::MessageChatData msg;
                msg.type = game::ChatType::TEXT_EMOTE;
                msg.language = game::ChatLanguage::COMMON;
                msg.message = chatText;
                gameHandler.addLocalChatMessage(msg);

                chatInputBuffer[0] = '\0';
                return;
            }
            // /invite command (Phase 4)
            if (command.size() > 7 && command.substr(0, 7) == "invite ") {
                std::string targetName = input.substr(8);
                gameHandler.inviteToGroup(targetName);
                chatInputBuffer[0] = '\0';
                return;
            }

            // Not a recognized emote — fall through and send as normal chat
        }

        game::ChatType type;
        switch (selectedChatType) {
            case 0: type = game::ChatType::SAY; break;
            case 1: type = game::ChatType::YELL; break;
            case 2: type = game::ChatType::PARTY; break;
            case 3: type = game::ChatType::GUILD; break;
            default: type = game::ChatType::SAY; break;
        }

        gameHandler.sendChatMessage(type, chatInputBuffer);

        // Clear input
        chatInputBuffer[0] = '\0';
    }
}

const char* GameScreen::getChatTypeName(game::ChatType type) const {
    switch (type) {
        case game::ChatType::SAY: return "SAY";
        case game::ChatType::YELL: return "YELL";
        case game::ChatType::EMOTE: return "EMOTE";
        case game::ChatType::TEXT_EMOTE: return "EMOTE";
        case game::ChatType::PARTY: return "PARTY";
        case game::ChatType::GUILD: return "GUILD";
        case game::ChatType::OFFICER: return "OFFICER";
        case game::ChatType::RAID: return "RAID";
        case game::ChatType::RAID_LEADER: return "RAID LEADER";
        case game::ChatType::RAID_WARNING: return "RAID WARNING";
        case game::ChatType::WHISPER: return "WHISPER";
        case game::ChatType::WHISPER_INFORM: return "TO";
        case game::ChatType::SYSTEM: return "SYSTEM";
        case game::ChatType::CHANNEL: return "CHANNEL";
        case game::ChatType::ACHIEVEMENT: return "ACHIEVEMENT";
        default: return "UNKNOWN";
    }
}

ImVec4 GameScreen::getChatTypeColor(game::ChatType type) const {
    switch (type) {
        case game::ChatType::SAY:
            return ImVec4(1.0f, 1.0f, 1.0f, 1.0f);  // White
        case game::ChatType::YELL:
            return ImVec4(1.0f, 0.3f, 0.3f, 1.0f);  // Red
        case game::ChatType::EMOTE:
            return ImVec4(1.0f, 0.7f, 0.3f, 1.0f);  // Orange
        case game::ChatType::TEXT_EMOTE:
            return ImVec4(1.0f, 0.7f, 0.3f, 1.0f);  // Orange
        case game::ChatType::PARTY:
            return ImVec4(0.5f, 0.5f, 1.0f, 1.0f);  // Light blue
        case game::ChatType::GUILD:
            return ImVec4(0.3f, 1.0f, 0.3f, 1.0f);  // Green
        case game::ChatType::OFFICER:
            return ImVec4(0.3f, 0.8f, 0.3f, 1.0f);  // Dark green
        case game::ChatType::RAID:
            return ImVec4(1.0f, 0.5f, 0.0f, 1.0f);  // Orange
        case game::ChatType::WHISPER:
            return ImVec4(1.0f, 0.5f, 1.0f, 1.0f);  // Pink
        case game::ChatType::WHISPER_INFORM:
            return ImVec4(1.0f, 0.5f, 1.0f, 1.0f);  // Pink
        case game::ChatType::SYSTEM:
            return ImVec4(1.0f, 1.0f, 0.3f, 1.0f);  // Yellow
        case game::ChatType::CHANNEL:
            return ImVec4(1.0f, 0.7f, 0.7f, 1.0f);  // Light pink
        case game::ChatType::ACHIEVEMENT:
            return ImVec4(1.0f, 1.0f, 0.0f, 1.0f);  // Bright yellow
        default:
            return ImVec4(0.7f, 0.7f, 0.7f, 1.0f);  // Gray
    }
}

void GameScreen::updateCharacterGeosets(game::Inventory& inventory) {
    auto& app = core::Application::getInstance();
    auto* renderer = app.getRenderer();
    if (!renderer) return;

    uint32_t instanceId = renderer->getCharacterInstanceId();
    if (instanceId == 0) return;

    auto* charRenderer = renderer->getCharacterRenderer();
    if (!charRenderer) return;

    auto* assetManager = app.getAssetManager();

    // Load ItemDisplayInfo.dbc for geosetGroup lookup
    std::shared_ptr<pipeline::DBCFile> displayInfoDbc;
    if (assetManager) {
        displayInfoDbc = assetManager->loadDBC("ItemDisplayInfo.dbc");
    }

    // Helper: get geosetGroup field for an equipped item's displayInfoId
    // DBC binary fields: 7=geosetGroup_1, 8=geosetGroup_2, 9=geosetGroup_3
    auto getGeosetGroup = [&](uint32_t displayInfoId, int groupField) -> uint32_t {
        if (!displayInfoDbc || displayInfoId == 0) return 0;
        int32_t recIdx = displayInfoDbc->findRecordById(displayInfoId);
        if (recIdx < 0) return 0;
        return displayInfoDbc->getUInt32(static_cast<uint32_t>(recIdx), 7 + groupField);
    };

    // Helper: find first equipped item matching inventoryType, return its displayInfoId
    auto findEquippedDisplayId = [&](std::initializer_list<uint8_t> types) -> uint32_t {
        for (int s = 0; s < game::Inventory::NUM_EQUIP_SLOTS; s++) {
            const auto& slot = inventory.getEquipSlot(static_cast<game::EquipSlot>(s));
            if (!slot.empty()) {
                for (uint8_t t : types) {
                    if (slot.item.inventoryType == t)
                        return slot.item.displayInfoId;
                }
            }
        }
        return 0;
    };

    // Helper: check if any equipment slot has the given inventoryType
    auto hasEquippedType = [&](std::initializer_list<uint8_t> types) -> bool {
        for (int s = 0; s < game::Inventory::NUM_EQUIP_SLOTS; s++) {
            const auto& slot = inventory.getEquipSlot(static_cast<game::EquipSlot>(s));
            if (!slot.empty()) {
                for (uint8_t t : types) {
                    if (slot.item.inventoryType == t) return true;
                }
            }
        }
        return false;
    };

    // Base geosets always present
    std::unordered_set<uint16_t> geosets;
    for (uint16_t i = 0; i <= 18; i++) {
        geosets.insert(i);
    }
    geosets.insert(101);  // Hair
    geosets.insert(201);  // Facial
    geosets.insert(701);  // Ears

    // Chest/Shirt: inventoryType 4 (shirt), 5 (chest), 20 (robe)
    // geosetGroup_1 > 0 → use mesh variant (502+), otherwise bare (501) + texture only
    {
        uint32_t did = findEquippedDisplayId({4, 5, 20});
        uint32_t gg = getGeosetGroup(did, 0);
        geosets.insert(static_cast<uint16_t>(gg > 0 ? 501 + gg : (did > 0 ? 501 : 501)));
        // geosetGroup_3 > 0 on robes also shows kilt legs (1302)
        uint32_t gg3 = getGeosetGroup(did, 2);
        if (gg3 > 0) {
            geosets.insert(static_cast<uint16_t>(1301 + gg3));
        }
    }

    // Legs: inventoryType 7
    // geosetGroup_1 > 0 → kilt/skirt mesh (1302+), otherwise bare legs (1301) + texture
    {
        uint32_t did = findEquippedDisplayId({7});
        uint32_t gg = getGeosetGroup(did, 0);
        // Only add leg geoset if robe hasn't already set a kilt geoset
        if (geosets.count(1302) == 0 && geosets.count(1303) == 0) {
            geosets.insert(static_cast<uint16_t>(gg > 0 ? 1301 + gg : 1301));
        }
    }

    // Feet/Boots: inventoryType 8
    // geosetGroup_1 > 0 → boot mesh (402+), otherwise bare feet (401) + texture
    {
        uint32_t did = findEquippedDisplayId({8});
        uint32_t gg = getGeosetGroup(did, 0);
        geosets.insert(static_cast<uint16_t>(gg > 0 ? 401 + gg : 401));
    }

    // Gloves/Hands: inventoryType 10
    // geosetGroup_1 > 0 → glove mesh (302+), otherwise bare hands (301)
    {
        uint32_t did = findEquippedDisplayId({10});
        uint32_t gg = getGeosetGroup(did, 0);
        geosets.insert(static_cast<uint16_t>(gg > 0 ? 301 + gg : 301));
    }

    // Back/Cloak: inventoryType 16 — geoset only, no skin texture (cloaks are separate models)
    geosets.insert(hasEquippedType({16}) ? 1502 : 1501);

    // Tabard: inventoryType 19
    if (hasEquippedType({19})) {
        geosets.insert(1201);
    }

    charRenderer->setActiveGeosets(instanceId, geosets);
}

void GameScreen::updateCharacterTextures(game::Inventory& inventory) {
    auto& app = core::Application::getInstance();
    auto* renderer = app.getRenderer();
    if (!renderer) return;

    auto* charRenderer = renderer->getCharacterRenderer();
    if (!charRenderer) return;

    auto* assetManager = app.getAssetManager();
    if (!assetManager) return;

    const auto& bodySkinPath = app.getBodySkinPath();
    const auto& underwearPaths = app.getUnderwearPaths();
    uint32_t skinSlot = app.getSkinTextureSlotIndex();

    if (bodySkinPath.empty()) return;

    // Component directory names indexed by region
    static const char* componentDirs[] = {
        "ArmUpperTexture",   // 0
        "ArmLowerTexture",   // 1
        "HandTexture",       // 2
        "TorsoUpperTexture", // 3
        "TorsoLowerTexture", // 4
        "LegUpperTexture",   // 5
        "LegLowerTexture",   // 6
        "FootTexture",       // 7
    };

    // Load ItemDisplayInfo.dbc
    auto displayInfoDbc = assetManager->loadDBC("ItemDisplayInfo.dbc");
    if (!displayInfoDbc) return;

    // Collect equipment texture regions from all equipped items
    std::vector<std::pair<int, std::string>> regionLayers;

    for (int s = 0; s < game::Inventory::NUM_EQUIP_SLOTS; s++) {
        const auto& slot = inventory.getEquipSlot(static_cast<game::EquipSlot>(s));
        if (slot.empty() || slot.item.displayInfoId == 0) continue;

        int32_t recIdx = displayInfoDbc->findRecordById(slot.item.displayInfoId);
        if (recIdx < 0) continue;

        // DBC fields 15-22 = texture_1 through texture_8 (regions 0-7)
        // (binary DBC has inventoryIcon_2 at field 6, shifting fields +1 vs CSV)
        for (int region = 0; region < 8; region++) {
            uint32_t fieldIdx = 15 + region;
            std::string texName = displayInfoDbc->getString(static_cast<uint32_t>(recIdx), fieldIdx);
            if (texName.empty()) continue;

            // Actual MPQ files have a gender suffix: _M (male), _F (female), _U (unisex)
            // Try gender-specific first, then unisex fallback
            std::string base = "Item\\TextureComponents\\" +
                std::string(componentDirs[region]) + "\\" + texName;
            std::string malePath = base + "_M.blp";
            std::string unisexPath = base + "_U.blp";
            std::string fullPath;
            if (assetManager->fileExists(malePath)) {
                fullPath = malePath;
            } else if (assetManager->fileExists(unisexPath)) {
                fullPath = unisexPath;
            } else {
                // Last resort: try without suffix
                fullPath = base + ".blp";
            }
            regionLayers.emplace_back(region, fullPath);
        }
    }

    // Re-composite: base skin + underwear + equipment regions
    GLuint newTex = charRenderer->compositeWithRegions(bodySkinPath, underwearPaths, regionLayers);
    if (newTex != 0) {
        charRenderer->setModelTexture(1, skinSlot, newTex);
    }

    // Cloak cape texture — separate from skin atlas, uses texture slot type-2 (Object Skin)
    uint32_t cloakSlot = app.getCloakTextureSlotIndex();
    if (cloakSlot > 0) {
        // Find equipped cloak (inventoryType 16)
        uint32_t cloakDisplayId = 0;
        for (int s = 0; s < game::Inventory::NUM_EQUIP_SLOTS; s++) {
            const auto& slot = inventory.getEquipSlot(static_cast<game::EquipSlot>(s));
            if (!slot.empty() && slot.item.inventoryType == 16 && slot.item.displayInfoId != 0) {
                cloakDisplayId = slot.item.displayInfoId;
                break;
            }
        }

        if (cloakDisplayId > 0) {
            int32_t recIdx = displayInfoDbc->findRecordById(cloakDisplayId);
            if (recIdx >= 0) {
                // DBC field 3 = modelTexture_1 (cape texture name)
                std::string capeName = displayInfoDbc->getString(static_cast<uint32_t>(recIdx), 3);
                if (!capeName.empty()) {
                    std::string capePath = "Item\\ObjectComponents\\Cape\\" + capeName + ".blp";
                    GLuint capeTex = charRenderer->loadTexture(capePath);
                    if (capeTex != 0) {
                        charRenderer->setModelTexture(1, cloakSlot, capeTex);
                        LOG_INFO("Cloak texture applied: ", capePath);
                    }
                }
            }
        } else {
            // No cloak equipped — reset to white fallback
            charRenderer->resetModelTexture(1, cloakSlot);
        }
    }
}

// ============================================================
// World Map
// ============================================================

void GameScreen::renderWorldMap(game::GameHandler& /* gameHandler */) {
    auto& app = core::Application::getInstance();
    auto* renderer = app.getRenderer();
    auto* assetMgr = app.getAssetManager();
    if (!renderer || !assetMgr) return;

    worldMap.initialize(assetMgr);

    // Keep map name in sync with minimap's map name
    auto* minimap = renderer->getMinimap();
    if (minimap) {
        worldMap.setMapName(minimap->getMapName());
    }

    glm::vec3 playerPos = renderer->getCharacterPosition();
    auto* window = app.getWindow();
    int screenW = window ? window->getWidth() : 1280;
    int screenH = window ? window->getHeight() : 720;
    worldMap.render(playerPos, screenW, screenH);
}

// ============================================================
// Action Bar (Phase 3)
// ============================================================

GLuint GameScreen::getSpellIcon(uint32_t spellId, pipeline::AssetManager* am) {
    if (spellId == 0 || !am) return 0;

    // Check cache first
    auto cit = spellIconCache_.find(spellId);
    if (cit != spellIconCache_.end()) return cit->second;

    // Lazy-load SpellIcon.dbc and Spell.dbc icon IDs
    if (!spellIconDbLoaded_) {
        spellIconDbLoaded_ = true;

        // Load SpellIcon.dbc: field 0 = ID, field 1 = icon path
        auto iconDbc = am->loadDBC("SpellIcon.dbc");
        if (iconDbc && iconDbc->isLoaded()) {
            for (uint32_t i = 0; i < iconDbc->getRecordCount(); i++) {
                uint32_t id = iconDbc->getUInt32(i, 0);
                std::string path = iconDbc->getString(i, 1);
                if (!path.empty() && id > 0) {
                    spellIconPaths_[id] = path;
                }
            }
        }

        // Load Spell.dbc: field 133 = SpellIconID
        auto spellDbc = am->loadDBC("Spell.dbc");
        if (spellDbc && spellDbc->isLoaded() && spellDbc->getFieldCount() > 133) {
            for (uint32_t i = 0; i < spellDbc->getRecordCount(); i++) {
                uint32_t id = spellDbc->getUInt32(i, 0);
                uint32_t iconId = spellDbc->getUInt32(i, 133);
                if (id > 0 && iconId > 0) {
                    spellIconIds_[id] = iconId;
                }
            }
        }
    }

    // Look up spellId -> SpellIconID -> icon path
    auto iit = spellIconIds_.find(spellId);
    if (iit == spellIconIds_.end()) {
        spellIconCache_[spellId] = 0;
        return 0;
    }

    auto pit = spellIconPaths_.find(iit->second);
    if (pit == spellIconPaths_.end()) {
        spellIconCache_[spellId] = 0;
        return 0;
    }

    // Path from DBC has no extension — append .blp
    std::string iconPath = pit->second + ".blp";
    auto blpData = am->readFile(iconPath);
    if (blpData.empty()) {
        spellIconCache_[spellId] = 0;
        return 0;
    }

    auto image = pipeline::BLPLoader::load(blpData);
    if (!image.isValid()) {
        spellIconCache_[spellId] = 0;
        return 0;
    }

    GLuint texId = 0;
    glGenTextures(1, &texId);
    glBindTexture(GL_TEXTURE_2D, texId);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, image.width, image.height, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, image.data.data());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);

    spellIconCache_[spellId] = texId;
    return texId;
}

void GameScreen::renderActionBar(game::GameHandler& gameHandler) {
    auto* window = core::Application::getInstance().getWindow();
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;
    auto* assetMgr = core::Application::getInstance().getAssetManager();

    float slotSize = 48.0f;
    float spacing = 4.0f;
    float padding = 8.0f;
    float barW = 12 * slotSize + 11 * spacing + padding * 2;
    float barH = slotSize + 24.0f;
    float barX = (screenW - barW) / 2.0f;
    float barY = screenH - barH;

    ImGui::SetNextWindowPos(ImVec2(barX, barY), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(barW, barH), ImGuiCond_Always);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                             ImGuiWindowFlags_NoScrollbar;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.05f, 0.05f, 0.05f, 0.9f));

    if (ImGui::Begin("##ActionBar", nullptr, flags)) {
        const auto& bar = gameHandler.getActionBar();
        static const char* keyLabels[] = {"1","2","3","4","5","6","7","8","9","0","-","="};

        for (int i = 0; i < 12; ++i) {
            if (i > 0) ImGui::SameLine(0, spacing);

            ImGui::BeginGroup();
            ImGui::PushID(i);

            const auto& slot = bar[i];
            bool onCooldown = !slot.isReady();

            auto getSpellName = [&](uint32_t spellId) -> std::string {
                if (!actionSpellDbAttempted) {
                    actionSpellDbAttempted = true;
                    if (assetMgr && assetMgr->isInitialized()) {
                        auto dbc = assetMgr->loadDBC("Spell.dbc");
                        if (dbc && dbc->isLoaded()) {
                            uint32_t fieldCount = dbc->getFieldCount();
                            uint32_t nameField = 136;
                            if (fieldCount < 137) {
                                if (fieldCount > 10) {
                                    nameField = fieldCount > 140 ? 136 : 1;
                                } else {
                                    nameField = 1;
                                }
                            }
                            uint32_t count = dbc->getRecordCount();
                            actionSpellNames.reserve(count);
                            for (uint32_t r = 0; r < count; ++r) {
                                uint32_t id = dbc->getUInt32(r, 0);
                                std::string name = dbc->getString(r, nameField);
                                if (!name.empty() && id > 0) {
                                    actionSpellNames[id] = name;
                                }
                            }
                            actionSpellDbLoaded = true;
                        }
                    }
                }
                auto it = actionSpellNames.find(spellId);
                if (it != actionSpellNames.end()) return it->second;
                return "Spell #" + std::to_string(spellId);
            };

            // Try to get icon texture for this slot
            GLuint iconTex = 0;
            const game::ItemDef* barItemDef = nullptr;
            if (slot.type == game::ActionBarSlot::SPELL && slot.id != 0) {
                iconTex = getSpellIcon(slot.id, assetMgr);
            } else if (slot.type == game::ActionBarSlot::ITEM && slot.id != 0) {
                // Look up item in inventory for icon and name
                auto& inv = gameHandler.getInventory();
                for (int bi = 0; bi < inv.getBackpackSize(); bi++) {
                    const auto& bs = inv.getBackpackSlot(bi);
                    if (!bs.empty() && bs.item.itemId == slot.id) {
                        barItemDef = &bs.item;
                        break;
                    }
                }
                if (barItemDef && barItemDef->displayInfoId != 0) {
                    iconTex = inventoryScreen.getItemIcon(barItemDef->displayInfoId);
                }
            }

            bool clicked = false;
            if (iconTex) {
                // Render icon-based button
                ImVec4 tintColor(1, 1, 1, 1);
                ImVec4 bgColor(0.1f, 0.1f, 0.1f, 0.9f);
                if (onCooldown) {
                    tintColor = ImVec4(0.4f, 0.4f, 0.4f, 0.8f);
                    bgColor = ImVec4(0.1f, 0.1f, 0.1f, 0.8f);
                }
                clicked = ImGui::ImageButton("##icon",
                    (ImTextureID)(uintptr_t)iconTex,
                    ImVec2(slotSize - 4, slotSize - 4),
                    ImVec2(0, 0), ImVec2(1, 1),
                    bgColor, tintColor);
            } else {
                // Fallback to text button
                if (onCooldown) {
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.2f, 0.2f, 0.8f));
                } else if (slot.isEmpty()) {
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.15f, 0.15f, 0.8f));
                } else {
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.3f, 0.3f, 0.5f, 0.9f));
                }

                char label[32];
                if (slot.type == game::ActionBarSlot::SPELL) {
                    std::string spellName = getSpellName(slot.id);
                    if (spellName.size() > 6) spellName = spellName.substr(0, 6);
                    snprintf(label, sizeof(label), "%s", spellName.c_str());
                } else if (slot.type == game::ActionBarSlot::ITEM && barItemDef) {
                    std::string itemName = barItemDef->name;
                    if (itemName.size() > 6) itemName = itemName.substr(0, 6);
                    snprintf(label, sizeof(label), "%s", itemName.c_str());
                } else if (slot.type == game::ActionBarSlot::ITEM) {
                    snprintf(label, sizeof(label), "Item");
                } else if (slot.type == game::ActionBarSlot::MACRO) {
                    snprintf(label, sizeof(label), "Macro");
                } else {
                    snprintf(label, sizeof(label), "--");
                }

                clicked = ImGui::Button(label, ImVec2(slotSize, slotSize));
                ImGui::PopStyleColor();
            }

            // Drop held item from inventory onto action bar
            if (clicked && inventoryScreen.isHoldingItem()) {
                const auto& held = inventoryScreen.getHeldItem();
                gameHandler.setActionBarSlot(i, game::ActionBarSlot::ITEM, held.itemId);
                inventoryScreen.returnHeldItem(gameHandler.getInventory());
            } else if (clicked && actionBarDragSlot_ >= 0) {
                // Dropping a dragged action bar slot onto another slot - swap or place
                if (i != actionBarDragSlot_) {
                    // Swap the two slots
                    const auto& dragSrc = bar[actionBarDragSlot_];
                    auto srcType = dragSrc.type;
                    auto srcId = dragSrc.id;
                    gameHandler.setActionBarSlot(actionBarDragSlot_, slot.type, slot.id);
                    gameHandler.setActionBarSlot(i, srcType, srcId);
                }
                actionBarDragSlot_ = -1;
                actionBarDragIcon_ = 0;
            } else if (clicked && !slot.isEmpty()) {
                // Pick up this action bar slot for dragging
                actionBarDragSlot_ = i;
                actionBarDragIcon_ = iconTex;
            } else if (clicked) {
                if (slot.type == game::ActionBarSlot::SPELL && slot.isReady()) {
                    uint64_t target = gameHandler.hasTarget() ? gameHandler.getTargetGuid() : 0;
                    gameHandler.castSpell(slot.id, target);
                } else if (slot.type == game::ActionBarSlot::ITEM && slot.id != 0) {
                    gameHandler.useItemById(slot.id);
                }
            }

            // Tooltip
            if (ImGui::IsItemHovered() && slot.type == game::ActionBarSlot::SPELL && slot.id != 0) {
                std::string fullName = getSpellName(slot.id);
                ImGui::BeginTooltip();
                ImGui::Text("%s", fullName.c_str());
                ImGui::TextDisabled("Spell ID: %u", slot.id);
                ImGui::EndTooltip();
            } else if (ImGui::IsItemHovered() && slot.type == game::ActionBarSlot::ITEM && slot.id != 0) {
                ImGui::BeginTooltip();
                if (barItemDef) {
                    ImGui::Text("%s", barItemDef->name.c_str());
                } else {
                    ImGui::Text("Item #%u", slot.id);
                }
                ImGui::EndTooltip();
            }

            // Cooldown overlay
            if (onCooldown && iconTex) {
                // Draw cooldown text centered over the icon
                ImVec2 btnMin = ImGui::GetItemRectMin();
                ImVec2 btnMax = ImGui::GetItemRectMax();
                char cdText[16];
                snprintf(cdText, sizeof(cdText), "%.0f", slot.cooldownRemaining);
                ImVec2 textSize = ImGui::CalcTextSize(cdText);
                float cx = btnMin.x + (btnMax.x - btnMin.x - textSize.x) * 0.5f;
                float cy = btnMin.y + (btnMax.y - btnMin.y - textSize.y) * 0.5f;
                ImGui::GetWindowDrawList()->AddText(ImVec2(cx, cy),
                    IM_COL32(255, 255, 0, 255), cdText);
            } else if (onCooldown) {
                char cdText[16];
                snprintf(cdText, sizeof(cdText), "%.0f", slot.cooldownRemaining);
                ImGui::SetCursorPosY(ImGui::GetCursorPosY() - slotSize / 2 - 8);
                ImGui::TextColored(ImVec4(1, 1, 0, 1), "%s", cdText);
            }

            // Key label below
            ImGui::TextDisabled("%s", keyLabels[i]);

            ImGui::PopID();
            ImGui::EndGroup();
        }
    }
    ImGui::End();

    ImGui::PopStyleColor();
    ImGui::PopStyleVar();

    // Handle action bar drag: render icon at cursor and detect drop outside
    if (actionBarDragSlot_ >= 0) {
        ImVec2 mousePos = ImGui::GetMousePos();

        // Draw dragged icon at cursor
        if (actionBarDragIcon_) {
            ImGui::GetForegroundDrawList()->AddImage(
                (ImTextureID)(uintptr_t)actionBarDragIcon_,
                ImVec2(mousePos.x - 20, mousePos.y - 20),
                ImVec2(mousePos.x + 20, mousePos.y + 20));
        } else {
            ImGui::GetForegroundDrawList()->AddRectFilled(
                ImVec2(mousePos.x - 20, mousePos.y - 20),
                ImVec2(mousePos.x + 20, mousePos.y + 20),
                IM_COL32(80, 80, 120, 180));
        }

        // On mouse release, check if outside the action bar area
        if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
            bool insideBar = (mousePos.x >= barX && mousePos.x <= barX + barW &&
                              mousePos.y >= barY && mousePos.y <= barY + barH);
            if (!insideBar) {
                // Dropped outside - clear the slot
                gameHandler.setActionBarSlot(actionBarDragSlot_, game::ActionBarSlot::EMPTY, 0);
            }
            actionBarDragSlot_ = -1;
            actionBarDragIcon_ = 0;
        }
    }
}

// ============================================================
// XP Bar
// ============================================================

void GameScreen::renderXpBar(game::GameHandler& gameHandler) {
    uint32_t nextLevelXp = gameHandler.getPlayerNextLevelXp();
    if (nextLevelXp == 0) return; // No XP data yet (level 80 or not initialized)

    uint32_t currentXp = gameHandler.getPlayerXp();
    auto* window = core::Application::getInstance().getWindow();
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    // Position just above the action bar
    float slotSize = 48.0f;
    float spacing = 4.0f;
    float padding = 8.0f;
    float barW = 12 * slotSize + 11 * spacing + padding * 2;
    float barH = slotSize + 24.0f;
    float actionBarY = screenH - barH;

    float xpBarH = 20.0f;
    float xpBarW = barW;
    float xpBarX = (screenW - xpBarW) / 2.0f;
    float xpBarY = actionBarY - xpBarH - 2.0f;

    ImGui::SetNextWindowPos(ImVec2(xpBarX, xpBarY), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(xpBarW, xpBarH + 4.0f), ImGuiCond_Always);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                             ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_AlwaysAutoResize;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 2.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(2.0f, 2.0f));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.05f, 0.05f, 0.05f, 0.9f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.3f, 0.3f, 0.3f, 0.8f));

    if (ImGui::Begin("##XpBar", nullptr, flags)) {
        float pct = static_cast<float>(currentXp) / static_cast<float>(nextLevelXp);
        if (pct > 1.0f) pct = 1.0f;

        // Custom segmented XP bar (20 bubbles)
        ImVec2 barMin = ImGui::GetCursorScreenPos();
        ImVec2 barSize = ImVec2(ImGui::GetContentRegionAvail().x, xpBarH - 4.0f);
        ImVec2 barMax = ImVec2(barMin.x + barSize.x, barMin.y + barSize.y);
        auto* drawList = ImGui::GetWindowDrawList();

        ImU32 bg = IM_COL32(15, 15, 20, 220);
        ImU32 fg = IM_COL32(148, 51, 238, 255);
        ImU32 seg = IM_COL32(35, 35, 45, 255);
        drawList->AddRectFilled(barMin, barMax, bg, 2.0f);
        drawList->AddRect(barMin, barMax, IM_COL32(80, 80, 90, 220), 2.0f);

        float fillW = barSize.x * pct;
        if (fillW > 0.0f) {
            drawList->AddRectFilled(barMin, ImVec2(barMin.x + fillW, barMax.y), fg, 2.0f);
        }

        const int segments = 20;
        float segW = barSize.x / static_cast<float>(segments);
        for (int i = 1; i < segments; ++i) {
            float x = barMin.x + segW * i;
            drawList->AddLine(ImVec2(x, barMin.y + 1.0f), ImVec2(x, barMax.y - 1.0f), seg, 1.0f);
        }

        char overlay[96];
        snprintf(overlay, sizeof(overlay), "%u / %u XP", currentXp, nextLevelXp);
        ImVec2 textSize = ImGui::CalcTextSize(overlay);
        float tx = barMin.x + (barSize.x - textSize.x) * 0.5f;
        float ty = barMin.y + (barSize.y - textSize.y) * 0.5f;
        drawList->AddText(ImVec2(tx, ty), IM_COL32(230, 230, 230, 255), overlay);

        ImGui::Dummy(barSize);
    }
    ImGui::End();

    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar(2);
}

// ============================================================
// Cast Bar (Phase 3)
// ============================================================

void GameScreen::renderCastBar(game::GameHandler& gameHandler) {
    if (!gameHandler.isCasting()) return;

    auto* window = core::Application::getInstance().getWindow();
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    float barW = 300.0f;
    float barX = (screenW - barW) / 2.0f;
    float barY = screenH - 120.0f;

    ImGui::SetNextWindowPos(ImVec2(barX, barY), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(barW, 40), ImGuiCond_Always);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                             ImGuiWindowFlags_NoScrollbar;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.1f, 0.1f, 0.9f));

    if (ImGui::Begin("##CastBar", nullptr, flags)) {
        float progress = gameHandler.getCastProgress();
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.8f, 0.6f, 0.2f, 1.0f));

        char overlay[64];
        snprintf(overlay, sizeof(overlay), "Spell %u (%.1fs)",
                 gameHandler.getCurrentCastSpellId(), gameHandler.getCastTimeRemaining());
        ImGui::ProgressBar(progress, ImVec2(-1, 20), overlay);
        ImGui::PopStyleColor();
    }
    ImGui::End();

    ImGui::PopStyleColor();
    ImGui::PopStyleVar();
}

// ============================================================
// Floating Combat Text (Phase 2)
// ============================================================

void GameScreen::renderCombatText(game::GameHandler& gameHandler) {
    const auto& entries = gameHandler.getCombatText();
    if (entries.empty()) return;

    auto* window = core::Application::getInstance().getWindow();
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;

    // Render combat text entries overlaid on screen
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(screenW, 400));

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoDecoration |
                             ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoNav;

    if (ImGui::Begin("##CombatText", nullptr, flags)) {
        float centerX = screenW / 2.0f;
        int index = 0;
        for (const auto& entry : entries) {
            float alpha = 1.0f - (entry.age / game::CombatTextEntry::LIFETIME);
            float yOffset = 200.0f - entry.age * 60.0f;

            ImVec4 color;
            char text[64];
            switch (entry.type) {
                case game::CombatTextEntry::MELEE_DAMAGE:
                case game::CombatTextEntry::SPELL_DAMAGE:
                    snprintf(text, sizeof(text), "-%d", entry.amount);
                    color = entry.isPlayerSource ?
                        ImVec4(1.0f, 1.0f, 0.3f, alpha) :   // Outgoing = yellow
                        ImVec4(1.0f, 0.3f, 0.3f, alpha);     // Incoming = red
                    break;
                case game::CombatTextEntry::CRIT_DAMAGE:
                    snprintf(text, sizeof(text), "-%d!", entry.amount);
                    color = ImVec4(1.0f, 0.5f, 0.0f, alpha);  // Orange for crit
                    break;
                case game::CombatTextEntry::HEAL:
                    snprintf(text, sizeof(text), "+%d", entry.amount);
                    color = ImVec4(0.3f, 1.0f, 0.3f, alpha);
                    break;
                case game::CombatTextEntry::CRIT_HEAL:
                    snprintf(text, sizeof(text), "+%d!", entry.amount);
                    color = ImVec4(0.3f, 1.0f, 0.3f, alpha);
                    break;
                case game::CombatTextEntry::MISS:
                    snprintf(text, sizeof(text), "Miss");
                    color = ImVec4(0.7f, 0.7f, 0.7f, alpha);
                    break;
                case game::CombatTextEntry::DODGE:
                    snprintf(text, sizeof(text), "Dodge");
                    color = ImVec4(0.7f, 0.7f, 0.7f, alpha);
                    break;
                case game::CombatTextEntry::PARRY:
                    snprintf(text, sizeof(text), "Parry");
                    color = ImVec4(0.7f, 0.7f, 0.7f, alpha);
                    break;
                default:
                    snprintf(text, sizeof(text), "%d", entry.amount);
                    color = ImVec4(1.0f, 1.0f, 1.0f, alpha);
                    break;
            }

            // Stagger entries horizontally
            float xOffset = centerX + (index % 3 - 1) * 80.0f;
            ImGui::SetCursorPos(ImVec2(xOffset, yOffset));
            ImGui::TextColored(color, "%s", text);
            index++;
        }
    }
    ImGui::End();
}

// ============================================================
// Party Frames (Phase 4)
// ============================================================

void GameScreen::renderPartyFrames(game::GameHandler& gameHandler) {
    if (!gameHandler.isInGroup()) return;

    const auto& partyData = gameHandler.getPartyData();
    float frameY = 120.0f;

    ImGui::SetNextWindowPos(ImVec2(10.0f, frameY), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(200.0f, 0.0f), ImGuiCond_Always);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                             ImGuiWindowFlags_AlwaysAutoResize;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.1f, 0.1f, 0.8f));

    if (ImGui::Begin("##PartyFrames", nullptr, flags)) {
        for (const auto& member : partyData.members) {
            ImGui::PushID(static_cast<int>(member.guid));

            ImVec4 nameColor = member.isOnline ?
                ImVec4(0.3f, 0.8f, 1.0f, 1.0f) :
                ImVec4(0.5f, 0.5f, 0.5f, 1.0f);

            // Clickable name to target
            if (ImGui::Selectable(member.name.c_str(), gameHandler.getTargetGuid() == member.guid)) {
                gameHandler.setTarget(member.guid);
            }

            // Try to show health from entity
            auto entity = gameHandler.getEntityManager().getEntity(member.guid);
            if (entity && (entity->getType() == game::ObjectType::PLAYER || entity->getType() == game::ObjectType::UNIT)) {
                auto unit = std::static_pointer_cast<game::Unit>(entity);
                uint32_t hp = unit->getHealth();
                uint32_t maxHp = unit->getMaxHealth();
                if (maxHp > 0) {
                    float pct = static_cast<float>(hp) / static_cast<float>(maxHp);
                    ImGui::PushStyleColor(ImGuiCol_PlotHistogram,
                        pct > 0.5f ? ImVec4(0.2f, 0.8f, 0.2f, 1.0f) :
                        pct > 0.2f ? ImVec4(0.8f, 0.8f, 0.2f, 1.0f) :
                                     ImVec4(0.8f, 0.2f, 0.2f, 1.0f));
                    ImGui::ProgressBar(pct, ImVec2(-1, 12), "");
                    ImGui::PopStyleColor();
                }
            }

            ImGui::Separator();
            ImGui::PopID();
        }
    }
    ImGui::End();

    ImGui::PopStyleColor();
    ImGui::PopStyleVar();
}

// ============================================================
// Group Invite Popup (Phase 4)
// ============================================================

void GameScreen::renderGroupInvitePopup(game::GameHandler& gameHandler) {
    if (!gameHandler.hasPendingGroupInvite()) return;

    auto* window = core::Application::getInstance().getWindow();
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - 150, 200), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(300, 0), ImGuiCond_Always);

    if (ImGui::Begin("Group Invite", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize)) {
        ImGui::Text("%s has invited you to a group.", gameHandler.getPendingInviterName().c_str());
        ImGui::Spacing();

        if (ImGui::Button("Accept", ImVec2(130, 30))) {
            gameHandler.acceptGroupInvite();
        }
        ImGui::SameLine();
        if (ImGui::Button("Decline", ImVec2(130, 30))) {
            gameHandler.declineGroupInvite();
        }
    }
    ImGui::End();
}

// ============================================================
// Buff/Debuff Bar (Phase 3)
// ============================================================

void GameScreen::renderBuffBar(game::GameHandler& gameHandler) {
    const auto& auras = gameHandler.getPlayerAuras();
    if (auras.empty()) return;

    // Count non-empty auras
    int activeCount = 0;
    for (const auto& a : auras) {
        if (!a.isEmpty()) activeCount++;
    }
    if (activeCount == 0) return;

    auto* window = core::Application::getInstance().getWindow();
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW - 400, 30), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(390, 0), ImGuiCond_Always);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                             ImGuiWindowFlags_AlwaysAutoResize;

    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));

    if (ImGui::Begin("##BuffBar", nullptr, flags)) {
        int shown = 0;
        for (size_t i = 0; i < auras.size() && shown < 16; ++i) {
            const auto& aura = auras[i];
            if (aura.isEmpty()) continue;

            if (shown > 0 && shown % 8 != 0) ImGui::SameLine();

            ImGui::PushID(static_cast<int>(i));

            // Green border for buffs, red for debuffs
            bool isBuff = (aura.flags & 0x02) != 0; // POSITIVE flag
            ImVec4 borderColor = isBuff ? ImVec4(0.2f, 0.8f, 0.2f, 1.0f) : ImVec4(0.8f, 0.2f, 0.2f, 1.0f);
            ImGui::PushStyleColor(ImGuiCol_Button, borderColor);

            char label[16];
            snprintf(label, sizeof(label), "%u", aura.spellId);
            if (ImGui::Button(label, ImVec2(40, 40))) {
                // Right-click to cancel own buffs
                if (isBuff) {
                    gameHandler.cancelAura(aura.spellId);
                }
            }
            ImGui::PopStyleColor();

            // Duration text
            if (aura.durationMs > 0) {
                int seconds = aura.durationMs / 1000;
                if (seconds < 60) {
                    ImGui::Text("%ds", seconds);
                } else {
                    ImGui::Text("%dm", seconds / 60);
                }
            }

            ImGui::PopID();
            shown++;
        }
    }
    ImGui::End();

    ImGui::PopStyleColor();
}

// ============================================================
// Loot Window (Phase 5)
// ============================================================

void GameScreen::renderLootWindow(game::GameHandler& gameHandler) {
    if (!gameHandler.isLootWindowOpen()) return;

    auto* window = core::Application::getInstance().getWindow();
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - 150, 200), ImGuiCond_Appearing);
    ImGui::SetNextWindowSize(ImVec2(300, 0), ImGuiCond_Always);

    bool open = true;
    if (ImGui::Begin("Loot", &open, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize)) {
        const auto& loot = gameHandler.getCurrentLoot();

        // Gold
        if (loot.gold > 0) {
            ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f), "%ug %us %uc",
                               loot.getGold(), loot.getSilver(), loot.getCopper());
            ImGui::Separator();
        }

        // Items with icons and labels
        constexpr float iconSize = 32.0f;
        int lootSlotClicked = -1;  // defer loot pickup to avoid iterator invalidation
        for (const auto& item : loot.items) {
            ImGui::PushID(item.slotIndex);

            // Get item info for name and quality
            const auto* info = gameHandler.getItemInfo(item.itemId);
            std::string itemName;
            game::ItemQuality quality = game::ItemQuality::COMMON;
            if (info && !info->name.empty()) {
                itemName = info->name;
                quality = static_cast<game::ItemQuality>(info->quality);
            } else {
                // Fallback: look up name from item template DB (single-player)
                auto tplName = gameHandler.getItemTemplateName(item.itemId);
                if (!tplName.empty()) {
                    itemName = tplName;
                    quality = gameHandler.getItemTemplateQuality(item.itemId);
                } else {
                    itemName = "Item #" + std::to_string(item.itemId);
                }
            }
            ImVec4 qColor = InventoryScreen::getQualityColor(quality);

            // Get item icon
            uint32_t displayId = item.displayInfoId;
            if (displayId == 0 && info) displayId = info->displayInfoId;
            GLuint iconTex = inventoryScreen.getItemIcon(displayId);

            ImVec2 cursor = ImGui::GetCursorScreenPos();
            float rowH = std::max(iconSize, ImGui::GetTextLineHeight() * 2.0f);

            // Invisible selectable for click handling
            if (ImGui::Selectable("##loot", false, 0, ImVec2(0, rowH))) {
                lootSlotClicked = item.slotIndex;
            }
            if (ImGui::IsItemClicked(ImGuiMouseButton_Right)) {
                lootSlotClicked = item.slotIndex;
            }
            bool hovered = ImGui::IsItemHovered();

            ImDrawList* drawList = ImGui::GetWindowDrawList();

            // Draw hover highlight
            if (hovered) {
                drawList->AddRectFilled(cursor,
                    ImVec2(cursor.x + ImGui::GetContentRegionAvail().x + iconSize + 8.0f,
                           cursor.y + rowH),
                    IM_COL32(255, 255, 255, 30));
            }

            // Draw icon
            if (iconTex) {
                drawList->AddImage((ImTextureID)(uintptr_t)iconTex,
                    cursor, ImVec2(cursor.x + iconSize, cursor.y + iconSize));
                drawList->AddRect(cursor, ImVec2(cursor.x + iconSize, cursor.y + iconSize),
                    ImGui::ColorConvertFloat4ToU32(qColor));
            } else {
                drawList->AddRectFilled(cursor,
                    ImVec2(cursor.x + iconSize, cursor.y + iconSize),
                    IM_COL32(40, 40, 50, 200));
                drawList->AddRect(cursor, ImVec2(cursor.x + iconSize, cursor.y + iconSize),
                    IM_COL32(80, 80, 80, 200));
            }

            // Draw item name
            float textX = cursor.x + iconSize + 6.0f;
            float textY = cursor.y + 2.0f;
            drawList->AddText(ImVec2(textX, textY),
                ImGui::ColorConvertFloat4ToU32(qColor), itemName.c_str());

            // Draw count if > 1
            if (item.count > 1) {
                char countStr[32];
                snprintf(countStr, sizeof(countStr), "x%u", item.count);
                float countY = textY + ImGui::GetTextLineHeight();
                drawList->AddText(ImVec2(textX, countY), IM_COL32(200, 200, 200, 220), countStr);
            }

            ImGui::PopID();
        }

        // Process deferred loot pickup (after loop to avoid iterator invalidation)
        if (lootSlotClicked >= 0) {
            gameHandler.lootItem(static_cast<uint8_t>(lootSlotClicked));
        }

        if (loot.items.empty() && loot.gold == 0) {
            gameHandler.closeLoot();
        }

        ImGui::Spacing();
        if (ImGui::Button("Close", ImVec2(-1, 0))) {
            gameHandler.closeLoot();
        }
    }
    ImGui::End();

    if (!open) {
        gameHandler.closeLoot();
    }
}

// ============================================================
// Gossip Window (Phase 5)
// ============================================================

void GameScreen::renderGossipWindow(game::GameHandler& gameHandler) {
    if (!gameHandler.isGossipWindowOpen()) return;

    auto* window = core::Application::getInstance().getWindow();
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - 200, 150), ImGuiCond_Appearing);
    ImGui::SetNextWindowSize(ImVec2(400, 0), ImGuiCond_Always);

    bool open = true;
    if (ImGui::Begin("NPC Dialog", &open, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize)) {
        const auto& gossip = gameHandler.getCurrentGossip();

        // NPC name (from creature cache)
        auto npcEntity = gameHandler.getEntityManager().getEntity(gossip.npcGuid);
        if (npcEntity && npcEntity->getType() == game::ObjectType::UNIT) {
            auto unit = std::static_pointer_cast<game::Unit>(npcEntity);
            if (!unit->getName().empty()) {
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f), "%s", unit->getName().c_str());
                ImGui::Separator();
            }
        }

        ImGui::Spacing();

        // Gossip options
        static const char* gossipIcons[] = {"[Chat]", "[Vendor]", "[Taxi]", "[Trainer]", "[Spiritguide]",
                                            "[Tabardvendor]", "[Battlemaster]", "[Banker]", "[Petitioner]",
                                            "[Tabarddesigner]", "[Auctioneer]"};

        for (const auto& opt : gossip.options) {
            ImGui::PushID(static_cast<int>(opt.id));
            const char* icon = (opt.icon < 11) ? gossipIcons[opt.icon] : "[Option]";
            char label[256];
            snprintf(label, sizeof(label), "%s %s", icon, opt.text.c_str());
            if (ImGui::Selectable(label)) {
                if (opt.icon == 4) { // Spirit guide
                    gameHandler.selectGossipOption(opt.id);
                    gameHandler.activateSpiritHealer(gossip.npcGuid);
                    gameHandler.closeGossip();
                } else {
                    gameHandler.selectGossipOption(opt.id);
                }
            }
            ImGui::PopID();
        }

        // Quest items
        if (!gossip.quests.empty()) {
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.3f, 1.0f), "Quests:");
            for (size_t qi = 0; qi < gossip.quests.size(); qi++) {
                const auto& quest = gossip.quests[qi];
                ImGui::PushID(static_cast<int>(qi));
                char qlabel[256];
                snprintf(qlabel, sizeof(qlabel), "[%d] %s", quest.questLevel, quest.title.c_str());
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 0.3f, 1.0f));
                if (ImGui::Selectable(qlabel)) {
                    gameHandler.selectGossipQuest(quest.questId);
                }
                ImGui::PopStyleColor();
                ImGui::PopID();
            }
        }

        ImGui::Spacing();
        if (ImGui::Button("Close", ImVec2(-1, 0))) {
            gameHandler.closeGossip();
        }
    }
    ImGui::End();

    if (!open) {
        gameHandler.closeGossip();
    }
}

// ============================================================
// Quest Details Window
// ============================================================

void GameScreen::renderQuestDetailsWindow(game::GameHandler& gameHandler) {
    if (!gameHandler.isQuestDetailsOpen()) return;

    auto* window = core::Application::getInstance().getWindow();
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - 225, screenH / 2 - 200), ImGuiCond_Appearing);
    ImGui::SetNextWindowSize(ImVec2(450, 400), ImGuiCond_Appearing);

    bool open = true;
    const auto& quest = gameHandler.getQuestDetails();
    if (ImGui::Begin(quest.title.c_str(), &open)) {
        // Quest description
        if (!quest.details.empty()) {
            ImGui::TextWrapped("%s", quest.details.c_str());
        }

        // Objectives
        if (!quest.objectives.empty()) {
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::TextColored(ImVec4(1.0f, 0.82f, 0.0f, 1.0f), "Objectives:");
            ImGui::TextWrapped("%s", quest.objectives.c_str());
        }

        // Rewards
        if (quest.rewardXp > 0 || quest.rewardMoney > 0) {
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::TextColored(ImVec4(1.0f, 0.82f, 0.0f, 1.0f), "Rewards:");
            if (quest.rewardXp > 0) {
                ImGui::Text("  %u experience", quest.rewardXp);
            }
            if (quest.rewardMoney > 0) {
                uint32_t gold = quest.rewardMoney / 10000;
                uint32_t silver = (quest.rewardMoney % 10000) / 100;
                uint32_t copper = quest.rewardMoney % 100;
                if (gold > 0) ImGui::Text("  %ug %us %uc", gold, silver, copper);
                else if (silver > 0) ImGui::Text("  %us %uc", silver, copper);
                else ImGui::Text("  %uc", copper);
            }
        }

        if (quest.suggestedPlayers > 1) {
            ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                "Suggested players: %u", quest.suggestedPlayers);
        }

        // Accept / Decline buttons
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();
        float buttonW = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
        if (ImGui::Button("Accept", ImVec2(buttonW, 0))) {
            gameHandler.acceptQuest();
        }
        ImGui::SameLine();
        if (ImGui::Button("Decline", ImVec2(buttonW, 0))) {
            gameHandler.declineQuest();
        }
    }
    ImGui::End();

    if (!open) {
        gameHandler.declineQuest();
    }
}

// ============================================================
// Vendor Window (Phase 5)
// ============================================================

void GameScreen::renderVendorWindow(game::GameHandler& gameHandler) {
    if (!gameHandler.isVendorWindowOpen()) return;

    auto* window = core::Application::getInstance().getWindow();
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - 200, 100), ImGuiCond_Appearing);
    ImGui::SetNextWindowSize(ImVec2(450, 400), ImGuiCond_Appearing);

    bool open = true;
    if (ImGui::Begin("Vendor", &open)) {
        const auto& vendor = gameHandler.getVendorItems();

        // Show player money
        uint64_t money = gameHandler.getMoneyCopper();
        uint32_t mg = static_cast<uint32_t>(money / 10000);
        uint32_t ms = static_cast<uint32_t>((money / 100) % 100);
        uint32_t mc = static_cast<uint32_t>(money % 100);
        ImGui::Text("Your money: %ug %us %uc", mg, ms, mc);
        ImGui::Separator();

        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "Right-click bag items to sell");
        ImGui::Separator();

        if (vendor.items.empty()) {
            ImGui::TextDisabled("This vendor has nothing for sale.");
        } else {
            if (ImGui::BeginTable("VendorTable", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY)) {
                ImGui::TableSetupColumn("Item", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn("Price", ImGuiTableColumnFlags_WidthFixed, 120.0f);
                ImGui::TableSetupColumn("Stock", ImGuiTableColumnFlags_WidthFixed, 60.0f);
                ImGui::TableSetupColumn("Buy", ImGuiTableColumnFlags_WidthFixed, 50.0f);
                ImGui::TableHeadersRow();

                // Quality colors (matching WoW)
                static const ImVec4 qualityColors[] = {
                    ImVec4(0.6f, 0.6f, 0.6f, 1.0f),  // 0 Poor (gray)
                    ImVec4(1.0f, 1.0f, 1.0f, 1.0f),  // 1 Common (white)
                    ImVec4(0.12f, 1.0f, 0.0f, 1.0f),  // 2 Uncommon (green)
                    ImVec4(0.0f, 0.44f, 0.87f, 1.0f), // 3 Rare (blue)
                    ImVec4(0.64f, 0.21f, 0.93f, 1.0f),// 4 Epic (purple)
                    ImVec4(1.0f, 0.5f, 0.0f, 1.0f),   // 5 Legendary (orange)
                };

                for (const auto& item : vendor.items) {
                    ImGui::TableNextRow();
                    ImGui::PushID(static_cast<int>(item.slot));

                    ImGui::TableSetColumnIndex(0);
                    auto* info = gameHandler.getItemInfo(item.itemId);
                    if (info && info->valid) {
                        uint32_t q = info->quality < 6 ? info->quality : 1;
                        ImGui::TextColored(qualityColors[q], "%s", info->name.c_str());
                        // Tooltip with stats on hover
                        if (ImGui::IsItemHovered()) {
                            ImGui::BeginTooltip();
                            ImGui::TextColored(qualityColors[q], "%s", info->name.c_str());
                            if (info->armor > 0) ImGui::Text("Armor: %d", info->armor);
                            if (info->stamina > 0) ImGui::Text("+%d Stamina", info->stamina);
                            if (info->strength > 0) ImGui::Text("+%d Strength", info->strength);
                            if (info->agility > 0) ImGui::Text("+%d Agility", info->agility);
                            if (info->intellect > 0) ImGui::Text("+%d Intellect", info->intellect);
                            if (info->spirit > 0) ImGui::Text("+%d Spirit", info->spirit);
                            ImGui::EndTooltip();
                        }
                    } else {
                        ImGui::Text("Item %u", item.itemId);
                    }

                    ImGui::TableSetColumnIndex(1);
                    uint32_t g = item.buyPrice / 10000;
                    uint32_t s = (item.buyPrice / 100) % 100;
                    uint32_t c = item.buyPrice % 100;
                    bool canAfford = money >= item.buyPrice;
                    if (!canAfford) ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.3f, 0.3f, 1.0f));
                    ImGui::Text("%ug %us %uc", g, s, c);
                    if (!canAfford) ImGui::PopStyleColor();

                    ImGui::TableSetColumnIndex(2);
                    if (item.maxCount < 0) {
                        ImGui::Text("Inf");
                    } else {
                        ImGui::Text("%d", item.maxCount);
                    }

                    ImGui::TableSetColumnIndex(3);
                    if (ImGui::SmallButton("Buy")) {
                        gameHandler.buyItem(vendor.vendorGuid, item.itemId, item.slot, 1);
                    }

                    ImGui::PopID();
                }

                ImGui::EndTable();
            }
        }
    }
    ImGui::End();

    if (!open) {
        gameHandler.closeVendor();
    }
}

// ============================================================
// Teleporter Panel
// ============================================================

void GameScreen::renderTeleporterPanel() {
    if (!showTeleporter) return;

    auto* window = core::Application::getInstance().getWindow();
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    float panelW = 280.0f;
    float panelH = 0.0f;  // Auto-size height
    ImGui::SetNextWindowPos(ImVec2((screenW - panelW) / 2.0f, screenH * 0.25f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(panelW, panelH), ImGuiCond_Always);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 6.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.08f, 0.08f, 0.15f, 0.92f));

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize;

    if (ImGui::Begin("Teleporter", &showTeleporter, flags)) {
        ImGui::Spacing();

        for (int i = 0; i < core::SPAWN_PRESET_COUNT; i++) {
            const auto& preset = core::SPAWN_PRESETS[i];
            char label[128];
            snprintf(label, sizeof(label), "%s\n(%.0f, %.0f, %.0f)",
                     preset.label,
                     preset.spawnCanonical.x, preset.spawnCanonical.y, preset.spawnCanonical.z);

            if (ImGui::Button(label, ImVec2(-1, 50))) {
                core::Application::getInstance().teleportTo(i);
                showTeleporter = false;
            }

            if (i < core::SPAWN_PRESET_COUNT - 1) {
                ImGui::Spacing();
            }
        }

        ImGui::Spacing();
    }
    ImGui::End();

    ImGui::PopStyleColor();
    ImGui::PopStyleVar();
}

// ============================================================
// Escape Menu
// ============================================================

void GameScreen::renderEscapeMenu() {
    if (!showEscapeMenu) return;

    ImGuiIO& io = ImGui::GetIO();
    float screenW = io.DisplaySize.x;
    float screenH = io.DisplaySize.y;
    ImVec2 size(260.0f, 220.0f);
    ImVec2 pos((screenW - size.x) * 0.5f, (screenH - size.y) * 0.5f);

    ImGui::SetNextWindowPos(pos, ImGuiCond_Always);
    ImGui::SetNextWindowSize(size, ImGuiCond_Always);
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar;

    if (ImGui::Begin("##EscapeMenu", nullptr, flags)) {
        ImGui::Text("Game Menu");
        ImGui::Separator();

        if (ImGui::Button("Logout", ImVec2(-1, 0))) {
            core::Application::getInstance().logoutToLogin();
            showEscapeMenu = false;
            showEscapeSettingsNotice = false;
        }
        if (ImGui::Button("Quit", ImVec2(-1, 0))) {
            auto* renderer = core::Application::getInstance().getRenderer();
            if (renderer) {
                if (auto* music = renderer->getMusicManager()) {
                    music->stopMusic(0.0f);
                }
            }
            core::Application::getInstance().shutdown();
        }
        if (ImGui::Button("Settings", ImVec2(-1, 0))) {
            showEscapeSettingsNotice = false;
            showSettingsWindow = true;
            settingsInit = false;
        }

        ImGui::Spacing();
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(10.0f, 10.0f));
        if (ImGui::Button("Back to Game", ImVec2(-1, 0))) {
            showEscapeMenu = false;
            showEscapeSettingsNotice = false;
        }
        ImGui::PopStyleVar();
    }
    ImGui::End();
}

// ============================================================
// Death Screen
// ============================================================

void GameScreen::renderDeathScreen(game::GameHandler& gameHandler) {
    if (!gameHandler.isPlayerDead()) return;

    auto* window = core::Application::getInstance().getWindow();
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    // Dark red overlay covering the whole screen
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(screenW, screenH));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.15f, 0.0f, 0.0f, 0.45f));
    ImGui::Begin("##DeathOverlay", nullptr,
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoInputs |
        ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoFocusOnAppearing);
    ImGui::End();
    ImGui::PopStyleColor();

    // "Release Spirit" dialog centered on screen
    float dlgW = 280.0f;
    float dlgH = 100.0f;
    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - dlgW / 2, screenH * 0.35f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(dlgW, dlgH), ImGuiCond_Always);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 8.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.0f, 0.0f, 0.9f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.6f, 0.1f, 0.1f, 1.0f));

    if (ImGui::Begin("##DeathDialog", nullptr,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar)) {

        ImGui::Spacing();
        // Center "You are dead." text
        const char* deathText = "You are dead.";
        float textW = ImGui::CalcTextSize(deathText).x;
        ImGui::SetCursorPosX((dlgW - textW) / 2);
        ImGui::TextColored(ImVec4(1.0f, 0.2f, 0.2f, 1.0f), "%s", deathText);

        ImGui::Spacing();
        ImGui::Spacing();

        // Center the Release Spirit button
        float btnW = 180.0f;
        ImGui::SetCursorPosX((dlgW - btnW) / 2);
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.5f, 0.1f, 0.1f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.7f, 0.15f, 0.15f, 1.0f));
        if (ImGui::Button("Release Spirit", ImVec2(btnW, 30))) {
            gameHandler.releaseSpirit();
        }
        ImGui::PopStyleColor(2);
    }
    ImGui::End();
    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar();
}

// ============================================================
// Settings Window
// ============================================================

void GameScreen::renderSettingsWindow() {
    if (!showSettingsWindow) return;

    auto* window = core::Application::getInstance().getWindow();
    auto* renderer = core::Application::getInstance().getRenderer();
    if (!window) return;

    static const int kResolutions[][2] = {
        {1280, 720},
        {1600, 900},
        {1920, 1080},
        {2560, 1440},
        {3840, 2160},
    };
    static const int kResCount = sizeof(kResolutions) / sizeof(kResolutions[0]);
    constexpr int kDefaultResW = 1920;
    constexpr int kDefaultResH = 1080;
    constexpr bool kDefaultFullscreen = false;
    constexpr bool kDefaultVsync = true;
    constexpr bool kDefaultShadows = true;
    constexpr int kDefaultMusicVolume = 30;
    constexpr int kDefaultSfxVolume = 100;
    constexpr float kDefaultMouseSensitivity = 0.2f;
    constexpr bool kDefaultInvertMouse = false;

    int defaultResIndex = 0;
    for (int i = 0; i < kResCount; i++) {
        if (kResolutions[i][0] == kDefaultResW && kResolutions[i][1] == kDefaultResH) {
            defaultResIndex = i;
            break;
        }
    }

    if (!settingsInit) {
        pendingFullscreen = window->isFullscreen();
        pendingVsync = window->isVsyncEnabled();
        pendingShadows = renderer ? renderer->areShadowsEnabled() : true;
        if (renderer) {
            if (auto* music = renderer->getMusicManager()) {
                pendingMusicVolume = music->getVolume();
            }
            if (auto* footstep = renderer->getFootstepManager()) {
                float scale = footstep->getVolumeScale();
                pendingSfxVolume = static_cast<int>(scale * 100.0f + 0.5f);
                if (pendingSfxVolume < 0) pendingSfxVolume = 0;
                if (pendingSfxVolume > 100) pendingSfxVolume = 100;
            } else if (auto* activity = renderer->getActivitySoundManager()) {
                float scale = activity->getVolumeScale();
                pendingSfxVolume = static_cast<int>(scale * 100.0f + 0.5f);
                if (pendingSfxVolume < 0) pendingSfxVolume = 0;
                if (pendingSfxVolume > 100) pendingSfxVolume = 100;
            }
            if (auto* cameraController = renderer->getCameraController()) {
                pendingMouseSensitivity = cameraController->getMouseSensitivity();
                pendingInvertMouse = cameraController->isInvertMouse();
            }
        }
        pendingResIndex = 0;
        int curW = window->getWidth();
        int curH = window->getHeight();
        for (int i = 0; i < kResCount; i++) {
            if (kResolutions[i][0] == curW && kResolutions[i][1] == curH) {
                pendingResIndex = i;
                break;
            }
        }
        if (auto* gameHandler = core::Application::getInstance().getGameHandler()) {
            if (gameHandler->isSinglePlayerMode()) {
                game::GameHandler::SinglePlayerSettings spSettings;
                if (gameHandler->getSinglePlayerSettings(spSettings)) {
                    pendingFullscreen = spSettings.fullscreen;
                    pendingVsync = spSettings.vsync;
                    pendingShadows = spSettings.shadows;
                    pendingMusicVolume = spSettings.musicVolume;
                    pendingSfxVolume = spSettings.sfxVolume;
                    pendingMouseSensitivity = spSettings.mouseSensitivity;
                    pendingInvertMouse = spSettings.invertMouse;
                    for (int i = 0; i < kResCount; i++) {
                        if (kResolutions[i][0] == spSettings.resWidth &&
                            kResolutions[i][1] == spSettings.resHeight) {
                            pendingResIndex = i;
                            break;
                        }
                    }
                }
            }
        }
        settingsInit = true;
    }

    ImGuiIO& io = ImGui::GetIO();
    float screenW = io.DisplaySize.x;
    float screenH = io.DisplaySize.y;
    ImVec2 size(440.0f, 520.0f);
    ImVec2 pos((screenW - size.x) * 0.5f, (screenH - size.y) * 0.5f);

    ImGui::SetNextWindowPos(pos, ImGuiCond_Always);
    ImGui::SetNextWindowSize(size, ImGuiCond_Always);
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar;

    if (ImGui::Begin("##SettingsWindow", nullptr, flags)) {
        ImGui::Text("Settings");
        ImGui::Separator();

        ImGui::Text("Video");
        ImGui::Checkbox("Fullscreen", &pendingFullscreen);
        ImGui::Checkbox("VSync", &pendingVsync);
        ImGui::Checkbox("Shadows", &pendingShadows);

        const char* resLabel = "Resolution";
        const char* resItems[kResCount];
        char resBuf[kResCount][16];
        for (int i = 0; i < kResCount; i++) {
            snprintf(resBuf[i], sizeof(resBuf[i]), "%dx%d", kResolutions[i][0], kResolutions[i][1]);
            resItems[i] = resBuf[i];
        }
        ImGui::Combo(resLabel, &pendingResIndex, resItems, kResCount);
        if (ImGui::Button("Restore Video Defaults", ImVec2(-1, 0))) {
            pendingFullscreen = kDefaultFullscreen;
            pendingVsync = kDefaultVsync;
            pendingShadows = kDefaultShadows;
            pendingResIndex = defaultResIndex;
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        ImGui::Text("Audio");
        ImGui::SliderInt("Music Volume", &pendingMusicVolume, 0, 100, "%d");
        ImGui::SliderInt("SFX Volume", &pendingSfxVolume, 0, 100, "%d");
        if (ImGui::Button("Restore Audio Defaults", ImVec2(-1, 0))) {
            pendingMusicVolume = kDefaultMusicVolume;
            pendingSfxVolume = kDefaultSfxVolume;
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        ImGui::Text("Controls");
        ImGui::SliderFloat("Mouse Sensitivity", &pendingMouseSensitivity, 0.05f, 1.0f, "%.2f");
        ImGui::Checkbox("Invert Mouse", &pendingInvertMouse);
        if (ImGui::Button("Restore Control Defaults", ImVec2(-1, 0))) {
            pendingMouseSensitivity = kDefaultMouseSensitivity;
            pendingInvertMouse = kDefaultInvertMouse;
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        if (ImGui::Button("Apply", ImVec2(-1, 0))) {
            window->setVsync(pendingVsync);
            window->setFullscreen(pendingFullscreen);
            window->applyResolution(kResolutions[pendingResIndex][0], kResolutions[pendingResIndex][1]);
            if (renderer) {
                renderer->setShadowsEnabled(pendingShadows);
                if (auto* music = renderer->getMusicManager()) {
                    music->setVolume(pendingMusicVolume);
                }
                float sfxScale = static_cast<float>(pendingSfxVolume) / 100.0f;
                if (auto* footstep = renderer->getFootstepManager()) {
                    footstep->setVolumeScale(sfxScale);
                }
                if (auto* activity = renderer->getActivitySoundManager()) {
                    activity->setVolumeScale(sfxScale);
                }
                if (auto* cameraController = renderer->getCameraController()) {
                    cameraController->setMouseSensitivity(pendingMouseSensitivity);
                    cameraController->setInvertMouse(pendingInvertMouse);
                }
            }
            if (auto* gameHandler = core::Application::getInstance().getGameHandler()) {
                if (gameHandler->isSinglePlayerMode()) {
                    game::GameHandler::SinglePlayerSettings spSettings;
                    spSettings.fullscreen = pendingFullscreen;
                    spSettings.vsync = pendingVsync;
                    spSettings.shadows = pendingShadows;
                    spSettings.resWidth = kResolutions[pendingResIndex][0];
                    spSettings.resHeight = kResolutions[pendingResIndex][1];
                    spSettings.musicVolume = pendingMusicVolume;
                    spSettings.sfxVolume = pendingSfxVolume;
                    spSettings.mouseSensitivity = pendingMouseSensitivity;
                    spSettings.invertMouse = pendingInvertMouse;
                    gameHandler->setSinglePlayerSettings(spSettings);
                }
            }
        }
        ImGui::Spacing();
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(10.0f, 10.0f));
        if (ImGui::Button("Back to Game", ImVec2(-1, 0))) {
            showSettingsWindow = false;
        }
        ImGui::PopStyleVar();
    }
    ImGui::End();
}

void GameScreen::renderQuestMarkers(game::GameHandler& gameHandler) {
    const auto& statuses = gameHandler.getNpcQuestStatuses();
    if (statuses.empty()) return;

    auto* renderer = core::Application::getInstance().getRenderer();
    auto* camera = renderer ? renderer->getCamera() : nullptr;
    auto* window = core::Application::getInstance().getWindow();
    if (!camera || !window) return;

    float screenW = static_cast<float>(window->getWidth());
    float screenH = static_cast<float>(window->getHeight());
    glm::mat4 viewProj = camera->getViewProjectionMatrix();
    auto* drawList = ImGui::GetForegroundDrawList();

    for (const auto& [guid, status] : statuses) {
        // Only show markers for available (!) and reward/completable (?)
        const char* marker = nullptr;
        ImU32 color = IM_COL32(255, 210, 0, 255); // yellow
        if (status == game::QuestGiverStatus::AVAILABLE) {
            marker = "!";
        } else if (status == game::QuestGiverStatus::AVAILABLE_LOW) {
            marker = "!";
            color = IM_COL32(160, 160, 160, 255); // gray
        } else if (status == game::QuestGiverStatus::REWARD) {
            marker = "?";
        } else if (status == game::QuestGiverStatus::INCOMPLETE) {
            marker = "?";
            color = IM_COL32(160, 160, 160, 255); // gray
        } else {
            continue;
        }

        // Get entity position (canonical coords)
        auto entity = gameHandler.getEntityManager().getEntity(guid);
        if (!entity) continue;

        glm::vec3 canonical(entity->getX(), entity->getY(), entity->getZ());
        glm::vec3 renderPos = core::coords::canonicalToRender(canonical);

        // Get model height for offset
        float heightOffset = 3.0f;
        glm::vec3 boundsCenter;
        float boundsRadius = 0.0f;
        if (core::Application::getInstance().getRenderBoundsForGuid(guid, boundsCenter, boundsRadius)) {
            heightOffset = boundsRadius * 2.0f + 1.0f;
        }
        renderPos.z += heightOffset;

        // Project to screen
        glm::vec4 clipPos = viewProj * glm::vec4(renderPos, 1.0f);
        if (clipPos.w <= 0.0f) continue;

        glm::vec2 ndc(clipPos.x / clipPos.w, clipPos.y / clipPos.w);
        float sx = (ndc.x + 1.0f) * 0.5f * screenW;
        float sy = (1.0f - ndc.y) * 0.5f * screenH;

        // Skip if off-screen
        if (sx < -50 || sx > screenW + 50 || sy < -50 || sy > screenH + 50) continue;

        // Scale text size based on distance
        float dist = clipPos.w;
        float fontSize = std::clamp(800.0f / dist, 14.0f, 48.0f);

        // Draw outlined text: 4 shadow copies then main text
        ImFont* font = ImGui::GetFont();
        ImU32 outlineColor = IM_COL32(0, 0, 0, 220);
        float off = std::max(1.0f, fontSize * 0.06f);
        ImVec2 textSize = font->CalcTextSizeA(fontSize, FLT_MAX, 0.0f, marker);
        float tx = sx - textSize.x * 0.5f;
        float ty = sy - textSize.y * 0.5f;

        drawList->AddText(font, fontSize, ImVec2(tx - off, ty), outlineColor, marker);
        drawList->AddText(font, fontSize, ImVec2(tx + off, ty), outlineColor, marker);
        drawList->AddText(font, fontSize, ImVec2(tx, ty - off), outlineColor, marker);
        drawList->AddText(font, fontSize, ImVec2(tx, ty + off), outlineColor, marker);
        drawList->AddText(font, fontSize, ImVec2(tx, ty), color, marker);
    }
}

void GameScreen::renderMinimapMarkers(game::GameHandler& gameHandler) {
    const auto& statuses = gameHandler.getNpcQuestStatuses();
    if (statuses.empty()) return;

    auto* renderer = core::Application::getInstance().getRenderer();
    auto* camera = renderer ? renderer->getCamera() : nullptr;
    auto* minimap = renderer ? renderer->getMinimap() : nullptr;
    auto* window = core::Application::getInstance().getWindow();
    if (!camera || !minimap || !window) return;

    float screenW = static_cast<float>(window->getWidth());

    // Minimap parameters (matching minimap.cpp)
    float mapSize = 200.0f;
    float margin = 10.0f;
    float mapRadius = mapSize * 0.5f;
    float centerX = screenW - margin - mapRadius;
    float centerY = margin + mapRadius;
    float viewRadius = 400.0f;

    // Player position in render coords
    auto& mi = gameHandler.getMovementInfo();
    glm::vec3 playerRender = core::coords::canonicalToRender(glm::vec3(mi.x, mi.y, mi.z));

    // Camera bearing for minimap rotation
    glm::vec3 fwd = camera->getForward();
    float bearing = std::atan2(-fwd.x, fwd.y);
    float cosB = std::cos(bearing);
    float sinB = std::sin(bearing);

    auto* drawList = ImGui::GetForegroundDrawList();

    for (const auto& [guid, status] : statuses) {
        ImU32 dotColor;
        const char* marker = nullptr;
        if (status == game::QuestGiverStatus::AVAILABLE) {
            dotColor = IM_COL32(255, 210, 0, 255);
            marker = "!";
        } else if (status == game::QuestGiverStatus::AVAILABLE_LOW) {
            dotColor = IM_COL32(160, 160, 160, 255);
            marker = "!";
        } else if (status == game::QuestGiverStatus::REWARD) {
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

        // Offset from player in render coords
        float dx = npcRender.x - playerRender.x;
        float dy = npcRender.y - playerRender.y;

        // Rotate by camera bearing (minimap north-up rotation)
        float rx = dx * cosB - dy * sinB;
        float ry = dx * sinB + dy * cosB;

        // Scale to minimap pixels
        float px = rx / viewRadius * mapRadius;
        float py = -ry / viewRadius * mapRadius; // screen Y is inverted

        // Clamp to circle
        float distFromCenter = std::sqrt(px * px + py * py);
        if (distFromCenter > mapRadius - 4.0f) {
            float scale = (mapRadius - 4.0f) / distFromCenter;
            px *= scale;
            py *= scale;
        }

        float sx = centerX + px;
        float sy = centerY + py;

        // Draw dot with marker text
        drawList->AddCircleFilled(ImVec2(sx, sy), 5.0f, dotColor);
        ImFont* font = ImGui::GetFont();
        ImVec2 textSize = font->CalcTextSizeA(11.0f, FLT_MAX, 0.0f, marker);
        drawList->AddText(font, 11.0f,
            ImVec2(sx - textSize.x * 0.5f, sy - textSize.y * 0.5f),
            IM_COL32(0, 0, 0, 255), marker);
    }
}

}} // namespace wowee::ui
