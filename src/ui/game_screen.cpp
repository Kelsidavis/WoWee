#include "ui/game_screen.hpp"
#include "core/application.hpp"
#include "core/coordinates.hpp"
#include "core/spawn_presets.hpp"
#include "core/input.hpp"
#include "rendering/renderer.hpp"
#include "rendering/minimap.hpp"
#include "rendering/character_renderer.hpp"
#include "rendering/camera.hpp"
#include "audio/music_manager.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_loader.hpp"
#include "core/logger.hpp"
#include <imgui.h>
#include <cmath>
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
    renderVendorWindow(gameHandler);
    renderEscapeMenu();
    renderSettingsWindow();

    // World map (M key toggle handled inside)
    renderWorldMap(gameHandler);

    // Teleporter panel (T key toggle handled in Application event loop)
    renderTeleporterPanel();

    // Spellbook (P key toggle handled inside)
    spellbookScreen.render(gameHandler, core::Application::getInstance().getAssetManager());

    // Inventory (B key toggle handled inside)
    inventoryScreen.render(gameHandler.getInventory(), gameHandler.getMoneyCopper());

    if (inventoryScreen.consumeInventoryDirty()) {
        gameHandler.notifyInventoryChanged();
    }

    if (inventoryScreen.consumeEquipmentDirty()) {
        updateCharacterGeosets(gameHandler.getInventory());
        updateCharacterTextures(gameHandler.getInventory());
        core::Application::getInstance().loadEquippedWeapons();
        gameHandler.notifyEquipmentChanged();
    }

    // Update renderer face-target position
    auto* renderer = core::Application::getInstance().getRenderer();
    if (renderer) {
        static glm::vec3 targetGLPos;
        if (gameHandler.hasTarget()) {
            auto target = gameHandler.getTarget();
            if (target) {
                targetGLPos = core::coords::canonicalToRender(glm::vec3(target->getX(), target->getY(), target->getZ()));
                renderer->setTargetPosition(&targetGLPos);
            } else {
                renderer->setTargetPosition(nullptr);
            }
        } else {
            renderer->setTargetPosition(nullptr);
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
    ImGui::SetNextWindowSize(ImVec2(chatW, chatH), ImGuiCond_Always);
    ImGui::SetNextWindowPos(ImVec2(chatX, chatY), ImGuiCond_Always);
    ImGui::Begin("Chat", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);

    // Chat history
    const auto& chatHistory = gameHandler.getChatHistory();

    ImGui::BeginChild("ChatHistory", ImVec2(0, -70), true, ImGuiWindowFlags_HorizontalScrollbar);

    for (const auto& msg : chatHistory) {
        ImVec4 color = getChatTypeColor(msg.type);
        ImGui::PushStyleColor(ImGuiCol_Text, color);

        if (msg.type == game::ChatType::TEXT_EMOTE) {
            ImGui::TextWrapped("You %s", msg.message.c_str());
        } else if (!msg.senderName.empty()) {
            ImGui::TextWrapped("[%s] %s: %s", getChatTypeName(msg.type), msg.senderName.c_str(), msg.message.c_str());
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
    if (ImGui::InputText("##ChatInput", chatInputBuffer, sizeof(chatInputBuffer), ImGuiInputTextFlags_EnterReturnsTrue)) {
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
                }
            }
        }

    }

    // Slash key: focus chat input
    if (!io.WantCaptureKeyboard && input.isKeyJustPressed(SDL_SCANCODE_SLASH)) {
        refocusChatInput = true;
        chatInputBuffer[0] = '/';
        chatInputBuffer[1] = '\0';
    }

    // Enter key: focus chat input (empty)
    if (!io.WantCaptureKeyboard && input.isKeyJustPressed(SDL_SCANCODE_RETURN)) {
        refocusChatInput = true;
    }

    // Left-click targeting (when mouse not captured by UI)
    if (!io.WantCaptureMouse && input.isMouseButtonJustPressed(SDL_BUTTON_LEFT)) {
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

            for (const auto& [guid, entity] : gameHandler.getEntityManager().getEntities()) {
                auto t = entity->getType();
                if (t != game::ObjectType::UNIT && t != game::ObjectType::PLAYER) continue;

                glm::vec3 entityGL = core::coords::canonicalToRender(glm::vec3(entity->getX(), entity->getY(), entity->getZ()));
                // Add half-height offset so we target the body center, not feet
                entityGL.z += 3.0f;

                float hitT;
                if (raySphereIntersect(ray, entityGL, 3.0f, hitT)) {
                    if (hitT < closestT) {
                        closestT = hitT;
                        closestGuid = guid;
                    }
                }
            }

            if (closestGuid != 0) {
                gameHandler.setTarget(closestGuid);
            }
            // Don't clear on miss — left-click is also used for camera orbit
        }
    }

    // Right-click on target for NPC interaction / loot / auto-attack
    if (!io.WantCaptureMouse && input.isMouseButtonJustPressed(SDL_BUTTON_RIGHT)) {
        if (gameHandler.hasTarget()) {
            auto target = gameHandler.getTarget();
            if (target) {
                if (target->getType() == game::ObjectType::UNIT) {
                    // Check if unit is dead (health == 0) → loot, otherwise interact/attack
                    auto unit = std::static_pointer_cast<game::Unit>(target);
                    if (unit->getHealth() == 0 && unit->getMaxHealth() > 0) {
                        gameHandler.lootTarget(target->getGuid());
                    } else if (gameHandler.isSinglePlayerMode()) {
                        // Single-player: toggle auto-attack
                        if (gameHandler.isAutoAttacking()) {
                            gameHandler.stopAutoAttack();
                        } else {
                            gameHandler.startAutoAttack(target->getGuid());
                        }
                    } else {
                        // Try NPC interaction first (gossip), fall back to attack
                        gameHandler.interactWithNpc(target->getGuid());
                    }
                } else if (target->getType() == game::ObjectType::PLAYER) {
                    // Right-click another player could start attack in PvP context
                }
            }
        }
    }
}

void GameScreen::renderPlayerFrame(game::GameHandler& gameHandler) {
    ImGui::SetNextWindowPos(ImVec2(10.0f, 30.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(250.0f, 0.0f), ImGuiCond_Always);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                             ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_AlwaysAutoResize;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.1f, 0.1f, 0.85f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.4f, 0.4f, 0.4f, 1.0f));

    if (ImGui::Begin("##PlayerFrame", nullptr, flags)) {
        // Use selected character info if available, otherwise defaults
        std::string playerName = "Adventurer";
        uint32_t playerLevel = 1;
        uint32_t playerHp = 100;
        uint32_t playerMaxHp = 100;

        const auto& characters = gameHandler.getCharacters();
        if (!characters.empty()) {
            // Use the first (or most recently selected) character
            const auto& ch = characters[0];
            playerName = ch.name;
            playerLevel = ch.level;
            // Characters don't store HP; use level-scaled estimate
            playerMaxHp = 20 + playerLevel * 10;
            playerHp = playerMaxHp;
        }

        // Name in green (friendly player color)
        ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), "%s", playerName.c_str());
        ImGui::SameLine();
        ImGui::TextDisabled("Lv %u", playerLevel);

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
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.2f, 0.8f, 0.2f, 1.0f));
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

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.1f, 0.1f, 0.85f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.4f, 0.4f, 0.4f, 1.0f));

    if (ImGui::Begin("##TargetFrame", nullptr, flags)) {
        // Entity name and type
        std::string name = getEntityName(target);

        ImVec4 nameColor;
        switch (target->getType()) {
            case game::ObjectType::PLAYER:
                nameColor = ImVec4(0.3f, 1.0f, 0.3f, 1.0f);  // Green
                break;
            case game::ObjectType::UNIT:
                nameColor = ImVec4(1.0f, 1.0f, 0.3f, 1.0f);  // Yellow
                break;
            default:
                nameColor = ImVec4(0.7f, 0.7f, 0.7f, 1.0f);
                break;
        }

        ImGui::TextColored(nameColor, "%s", name.c_str());

        // Level (for units/players)
        if (target->getType() == game::ObjectType::UNIT || target->getType() == game::ObjectType::PLAYER) {
            auto unit = std::static_pointer_cast<game::Unit>(target);
            ImGui::SameLine();
            ImGui::TextDisabled("Lv %u", unit->getLevel());

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

            if (onCooldown) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.2f, 0.2f, 0.8f));
            } else if (slot.isEmpty()) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.15f, 0.15f, 0.8f));
            } else {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.3f, 0.3f, 0.5f, 0.9f));
            }

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
                            for (uint32_t i = 0; i < count; ++i) {
                                uint32_t id = dbc->getUInt32(i, 0);
                                std::string name = dbc->getString(i, nameField);
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

            char label[32];
            std::string spellName;
            if (slot.type == game::ActionBarSlot::SPELL) {
                spellName = getSpellName(slot.id);
                if (spellName.size() > 6) {
                    spellName = spellName.substr(0, 6);
                }
                snprintf(label, sizeof(label), "%s", spellName.c_str());
            } else if (slot.type == game::ActionBarSlot::ITEM) {
                snprintf(label, sizeof(label), "Item");
            } else if (slot.type == game::ActionBarSlot::MACRO) {
                snprintf(label, sizeof(label), "Macro");
            } else {
                snprintf(label, sizeof(label), "--");
            }

            if (ImGui::Button(label, ImVec2(slotSize, slotSize))) {
                if (slot.type == game::ActionBarSlot::SPELL && slot.isReady()) {
                    uint64_t target = gameHandler.hasTarget() ? gameHandler.getTargetGuid() : 0;
                    gameHandler.castSpell(slot.id, target);
                }
            }
            ImGui::PopStyleColor();

            if (ImGui::IsItemHovered() && slot.type == game::ActionBarSlot::SPELL && slot.id != 0) {
                std::string fullName = getSpellName(slot.id);
                ImGui::BeginTooltip();
                ImGui::Text("%s", fullName.c_str());
                ImGui::TextDisabled("Spell ID: %u", slot.id);
                ImGui::EndTooltip();
            }

            // Cooldown overlay text
            if (onCooldown) {
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
}

// ============================================================
// XP Bar
// ============================================================

void GameScreen::renderXpBar(game::GameHandler& gameHandler) {
    uint32_t nextLevelXp = gameHandler.getPlayerNextLevelXp();
    if (nextLevelXp == 0) return; // No XP data yet (level 80 or not initialized)

    uint32_t currentXp = gameHandler.getPlayerXp();
    uint32_t level = gameHandler.getPlayerLevel();

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

    float xpBarH = 14.0f;
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

        // Purple XP bar (WoW-style)
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.58f, 0.2f, 0.93f, 1.0f));

        char overlay[96];
        snprintf(overlay, sizeof(overlay), "Lv %u  -  %u / %u XP", level, currentXp, nextLevelXp);
        ImGui::ProgressBar(pct, ImVec2(-1, xpBarH - 4.0f), overlay);

        ImGui::PopStyleColor();
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

    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - 150, 200), ImGuiCond_FirstUseEver);
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

        // Items
        for (const auto& item : loot.items) {
            ImGui::PushID(item.slotIndex);
            char label[64];
            snprintf(label, sizeof(label), "Item %u (x%u)", item.itemId, item.count);
            if (ImGui::Selectable(label)) {
                gameHandler.lootItem(item.slotIndex);
            }
            ImGui::PopID();
        }

        if (loot.items.empty() && loot.gold == 0) {
            ImGui::TextDisabled("Empty");
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

    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - 200, 150), ImGuiCond_FirstUseEver);
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
                gameHandler.selectGossipOption(opt.id);
            }
            ImGui::PopID();
        }

        // Quest items
        if (!gossip.quests.empty()) {
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.3f, 1.0f), "Quests:");
            for (const auto& quest : gossip.quests) {
                ImGui::BulletText("[%d] %s", quest.questLevel, quest.title.c_str());
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
// Vendor Window (Phase 5)
// ============================================================

void GameScreen::renderVendorWindow(game::GameHandler& gameHandler) {
    if (!gameHandler.isVendorWindowOpen()) return;

    auto* window = core::Application::getInstance().getWindow();
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - 200, 100), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(450, 400), ImGuiCond_FirstUseEver);

    bool open = true;
    if (ImGui::Begin("Vendor", &open)) {
        const auto& vendor = gameHandler.getVendorItems();

        if (vendor.items.empty()) {
            ImGui::TextDisabled("This vendor has nothing for sale.");
        } else {
            if (ImGui::BeginTable("VendorTable", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY)) {
                ImGui::TableSetupColumn("Item", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn("Price", ImGuiTableColumnFlags_WidthFixed, 120.0f);
                ImGui::TableSetupColumn("Stock", ImGuiTableColumnFlags_WidthFixed, 60.0f);
                ImGui::TableSetupColumn("Buy", ImGuiTableColumnFlags_WidthFixed, 50.0f);
                ImGui::TableHeadersRow();

                for (const auto& item : vendor.items) {
                    ImGui::TableNextRow();
                    ImGui::PushID(static_cast<int>(item.slot));

                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("Item %u", item.itemId);

                    ImGui::TableSetColumnIndex(1);
                    uint32_t g = item.buyPrice / 10000;
                    uint32_t s = (item.buyPrice / 100) % 100;
                    uint32_t c = item.buyPrice % 100;
                    ImGui::Text("%ug %us %uc", g, s, c);

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
        // Close vendor - just hide UI, no server packet needed
        // The vendor window state will be reset on next interaction
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
    ImVec2 size(400.0f, 420.0f);
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

}} // namespace wowee::ui
