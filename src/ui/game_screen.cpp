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
#include "audio/mount_sound_manager.hpp"
#include "audio/npc_voice_manager.hpp"
#include "audio/ambient_sound_manager.hpp"
#include "audio/ui_sound_manager.hpp"
#include "audio/combat_sound_manager.hpp"
#include "audio/spell_sound_manager.hpp"
#include "audio/movement_sound_manager.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_loader.hpp"
#include "pipeline/blp_loader.hpp"
#include "pipeline/dbc_layout.hpp"
#include "pipeline/hd_pack_manager.hpp"
#include "game/expansion_profile.hpp"
#include "core/logger.hpp"
#include <imgui.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <cctype>
#include <unordered_set>

namespace {
    std::string trim(const std::string& s) {
        size_t first = s.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) return "";
        size_t last = s.find_last_not_of(" \t\r\n");
        return s.substr(first, last - first + 1);
    }

    std::string toLower(std::string s) {
        std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        });
        return s;
    }

    bool isPortBotTarget(const std::string& target) {
        std::string t = toLower(trim(target));
        return t == "portbot" || t == "gmbot" || t == "telebot";
    }

    std::string buildPortBotCommand(const std::string& rawInput) {
        std::string input = trim(rawInput);
        if (input.empty()) return "";

        std::string lower = toLower(input);
        if (lower == "help" || lower == "?") {
            return "__help__";
        }

        if (lower.rfind(".tele ", 0) == 0 || lower.rfind(".go ", 0) == 0) {
            return input;
        }

        if (lower.rfind("xyz ", 0) == 0) {
            return ".go " + input;
        }

        if (lower == "sw" || lower == "stormwind") return ".tele stormwind";
        if (lower == "if" || lower == "ironforge") return ".tele ironforge";
        if (lower == "darn" || lower == "darnassus") return ".tele darnassus";
        if (lower == "org" || lower == "orgrimmar") return ".tele orgrimmar";
        if (lower == "tb" || lower == "thunderbluff") return ".tele thunderbluff";
        if (lower == "uc" || lower == "undercity") return ".tele undercity";
        if (lower == "shatt" || lower == "shattrath") return ".tele shattrath";
        if (lower == "dal" || lower == "dalaran") return ".tele dalaran";

        return ".tele " + input;
    }

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

GameScreen::GameScreen() {
    loadSettings();
    initChatTabs();
}

void GameScreen::initChatTabs() {
    chatTabs_.clear();
    // General tab: shows everything
    chatTabs_.push_back({"General", 0xFFFFFFFF});
    // Combat tab: system + loot messages
    chatTabs_.push_back({"Combat", (1u << static_cast<uint8_t>(game::ChatType::SYSTEM)) |
                                    (1u << static_cast<uint8_t>(game::ChatType::LOOT))});
    // Whispers tab
    chatTabs_.push_back({"Whispers", (1u << static_cast<uint8_t>(game::ChatType::WHISPER)) |
                                      (1u << static_cast<uint8_t>(game::ChatType::WHISPER_INFORM))});
    // Trade/LFG tab: channel messages
    chatTabs_.push_back({"Trade/LFG", (1u << static_cast<uint8_t>(game::ChatType::CHANNEL))});
}

bool GameScreen::shouldShowMessage(const game::MessageChatData& msg, int tabIndex) const {
    if (tabIndex < 0 || tabIndex >= static_cast<int>(chatTabs_.size())) return true;
    const auto& tab = chatTabs_[tabIndex];
    if (tab.typeMask == 0xFFFFFFFF) return true;  // General tab shows all

    uint32_t typeBit = 1u << static_cast<uint8_t>(msg.type);

    // For Trade/LFG tab, also filter by channel name
    if (tabIndex == 3 && msg.type == game::ChatType::CHANNEL) {
        const std::string& ch = msg.channelName;
        if (ch.find("Trade") == std::string::npos &&
            ch.find("General") == std::string::npos &&
            ch.find("LookingForGroup") == std::string::npos) {
            return false;
        }
        return true;
    }

    return (tab.typeMask & typeBit) != 0;
}

void GameScreen::render(game::GameHandler& gameHandler) {
    // Set up chat bubble callback (once)
    if (!chatBubbleCallbackSet_) {
        gameHandler.setChatBubbleCallback([this](uint64_t guid, const std::string& msg, bool isYell) {
            float duration = 8.0f + static_cast<float>(msg.size()) * 0.06f;
            if (isYell) duration += 2.0f;
            if (duration > 15.0f) duration = 15.0f;

            // Replace existing bubble for same sender
            for (auto& b : chatBubbles_) {
                if (b.senderGuid == guid) {
                    b.message = msg;
                    b.timeRemaining = duration;
                    b.totalDuration = duration;
                    b.isYell = isYell;
                    return;
                }
            }
            // Evict oldest if too many
            if (chatBubbles_.size() >= 10) {
                chatBubbles_.erase(chatBubbles_.begin());
            }
            chatBubbles_.push_back({guid, msg, duration, duration, isYell});
        });
        chatBubbleCallbackSet_ = true;
    }

    // Apply UI transparency setting
    float prevAlpha = ImGui::GetStyle().Alpha;
    ImGui::GetStyle().Alpha = uiOpacity_;

    // Apply initial minimap settings when renderer becomes available
    if (!minimapSettingsApplied_) {
        auto* renderer = core::Application::getInstance().getRenderer();
        if (renderer) {
            if (auto* minimap = renderer->getMinimap()) {
                minimapRotate_ = false;
                pendingMinimapRotate = false;
                minimap->setRotateWithCamera(false);
                minimap->setSquareShape(minimapSquare_);
                minimapSettingsApplied_ = true;
            }
        }
    }

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
    renderBagBar(gameHandler);
    renderXpBar(gameHandler);
    renderCastBar(gameHandler);
    renderCombatText(gameHandler);
    renderPartyFrames(gameHandler);
    renderGroupInvitePopup(gameHandler);
    renderGuildInvitePopup(gameHandler);
    renderGuildRoster(gameHandler);
    renderBuffBar(gameHandler);
    renderLootWindow(gameHandler);
    renderGossipWindow(gameHandler);
    renderQuestDetailsWindow(gameHandler);
    renderQuestRequestItemsWindow(gameHandler);
    renderQuestOfferRewardWindow(gameHandler);
    renderVendorWindow(gameHandler);
    renderTrainerWindow(gameHandler);
    renderTaxiWindow(gameHandler);
    // renderQuestMarkers(gameHandler);  // Disabled - using 3D billboard markers now
    renderMinimapMarkers(gameHandler);
    renderDeathScreen(gameHandler);
    renderResurrectDialog(gameHandler);
    renderChatBubbles(gameHandler);
    renderEscapeMenu();
    renderSettingsWindow();

    // World map (M key toggle handled inside)
    renderWorldMap(gameHandler);

    // Quest Log (L key toggle handled inside)
    questLogScreen.render(gameHandler);

    // Spellbook (P key toggle handled inside)
    spellbookScreen.render(gameHandler, core::Application::getInstance().getAssetManager());

    // Talents (N key toggle handled inside)
    talentScreen.render(gameHandler);

    // Set up inventory screen asset manager + player appearance (re-init on character switch)
    {
        uint64_t activeGuid = gameHandler.getActiveCharacterGuid();
        if (activeGuid != 0 && activeGuid != inventoryScreenCharGuid_) {
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
                    inventoryScreenCharGuid_ = activeGuid;
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

    if (inventoryScreen.consumeEquipmentDirty() || gameHandler.consumeOnlineEquipmentDirty()) {
        updateCharacterGeosets(gameHandler.getInventory());
        updateCharacterTextures(gameHandler.getInventory());
        core::Application::getInstance().loadEquippedWeapons();
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
        renderer->setInCombat(gameHandler.isAutoAttacking());
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

    // Restore previous alpha
    ImGui::GetStyle().Alpha = prevAlpha;
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
    if (chatWindowLocked) {
        flags |= ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar;
    }
    ImGui::Begin("Chat", nullptr, flags);

    if (!chatWindowLocked) {
        chatWindowPos_ = ImGui::GetWindowPos();
    }

    // Chat tabs
    if (ImGui::BeginTabBar("ChatTabs")) {
        for (int i = 0; i < static_cast<int>(chatTabs_.size()); ++i) {
            if (ImGui::BeginTabItem(chatTabs_[i].name.c_str())) {
                activeChatTab_ = i;
                ImGui::EndTabItem();
            }
        }
        ImGui::EndTabBar();
    }

    // Chat history
    const auto& chatHistory = gameHandler.getChatHistory();

    ImGui::BeginChild("ChatHistory", ImVec2(0, -70), true, ImGuiWindowFlags_HorizontalScrollbar);
    bool chatHistoryHovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem);

    // Helper: parse WoW color code |cAARRGGBB → ImVec4
    auto parseWowColor = [](const std::string& text, size_t pos) -> ImVec4 {
        // |cAARRGGBB (10 chars total: |c + 8 hex)
        if (pos + 10 > text.size()) return ImVec4(1, 1, 1, 1);
        auto hexByte = [&](size_t offset) -> float {
            const char* s = text.c_str() + pos + offset;
            char buf[3] = {s[0], s[1], '\0'};
            return static_cast<float>(strtol(buf, nullptr, 16)) / 255.0f;
        };
        float a = hexByte(2);
        float r = hexByte(4);
        float g = hexByte(6);
        float b = hexByte(8);
        return ImVec4(r, g, b, a);
    };

    // Helper: render an item tooltip from ItemQueryResponseData
    auto renderItemLinkTooltip = [&](uint32_t itemEntry) {
        const auto* info = gameHandler.getItemInfo(itemEntry);
        if (!info || !info->valid) return;

        ImGui::BeginTooltip();
        // Quality color for name
        ImVec4 qColor(1, 1, 1, 1);
        switch (info->quality) {
            case 0: qColor = ImVec4(0.62f, 0.62f, 0.62f, 1.0f); break; // Poor
            case 1: qColor = ImVec4(1.0f, 1.0f, 1.0f, 1.0f); break;    // Common
            case 2: qColor = ImVec4(0.12f, 1.0f, 0.0f, 1.0f); break;   // Uncommon
            case 3: qColor = ImVec4(0.0f, 0.44f, 0.87f, 1.0f); break;  // Rare
            case 4: qColor = ImVec4(0.64f, 0.21f, 0.93f, 1.0f); break; // Epic
            case 5: qColor = ImVec4(1.0f, 0.50f, 0.0f, 1.0f); break;   // Legendary
        }
        ImGui::TextColored(qColor, "%s", info->name.c_str());

        // Slot type
        if (info->inventoryType > 0) {
            const char* slotName = "";
            switch (info->inventoryType) {
                case 1:  slotName = "Head"; break;
                case 2:  slotName = "Neck"; break;
                case 3:  slotName = "Shoulder"; break;
                case 4:  slotName = "Shirt"; break;
                case 5:  slotName = "Chest"; break;
                case 6:  slotName = "Waist"; break;
                case 7:  slotName = "Legs"; break;
                case 8:  slotName = "Feet"; break;
                case 9:  slotName = "Wrist"; break;
                case 10: slotName = "Hands"; break;
                case 11: slotName = "Finger"; break;
                case 12: slotName = "Trinket"; break;
                case 13: slotName = "One-Hand"; break;
                case 14: slotName = "Shield"; break;
                case 15: slotName = "Ranged"; break;
                case 16: slotName = "Back"; break;
                case 17: slotName = "Two-Hand"; break;
                case 18: slotName = "Bag"; break;
                case 19: slotName = "Tabard"; break;
                case 20: slotName = "Robe"; break;
                case 21: slotName = "Main Hand"; break;
                case 22: slotName = "Off Hand"; break;
                case 23: slotName = "Held In Off-hand"; break;
                case 25: slotName = "Thrown"; break;
                case 26: slotName = "Ranged"; break;
            }
            if (slotName[0]) {
                if (!info->subclassName.empty())
                    ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "%s  %s", slotName, info->subclassName.c_str());
                else
                    ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "%s", slotName);
            }
        }
        if (info->armor > 0) ImGui::Text("%d Armor", info->armor);
        ImVec4 green(0.0f, 1.0f, 0.0f, 1.0f);
        auto renderStat = [&](int32_t val, const char* name) {
            if (val > 0) ImGui::TextColored(green, "+%d %s", val, name);
            else if (val < 0) ImGui::TextColored(ImVec4(1, 0.2f, 0.2f, 1), "%d %s", val, name);
        };
        renderStat(info->stamina, "Stamina");
        renderStat(info->strength, "Strength");
        renderStat(info->agility, "Agility");
        renderStat(info->intellect, "Intellect");
        renderStat(info->spirit, "Spirit");
        if (info->sellPrice > 0) {
            uint32_t g = info->sellPrice / 10000;
            uint32_t s = (info->sellPrice / 100) % 100;
            uint32_t c = info->sellPrice % 100;
            ImGui::Separator();
            ImGui::TextColored(ImVec4(1.0f, 0.84f, 0.0f, 1.0f), "Sell Price: %ug %us %uc", g, s, c);
        }
        ImGui::EndTooltip();
    };

    // Helper: render text with clickable URLs and WoW item links
    auto renderTextWithLinks = [&](const std::string& text, const ImVec4& color) {
        size_t pos = 0;
        while (pos < text.size()) {
            // Find next special element: URL or WoW link
            size_t urlStart = std::string::npos;
            size_t httpPos = text.find("http://", pos);
            size_t httpsPos = text.find("https://", pos);
            if (httpPos != std::string::npos && (httpsPos == std::string::npos || httpPos < httpsPos))
                urlStart = httpPos;
            else if (httpsPos != std::string::npos)
                urlStart = httpsPos;

            // Find next WoW item link: |cXXXXXXXX|Hitem:ENTRY:...|h[Name]|h|r
            size_t linkStart = text.find("|c", pos);
            // Also handle bare |Hitem: without color prefix
            size_t bareLinkStart = text.find("|Hitem:", pos);

            // Determine which comes first
            size_t nextSpecial = std::min({urlStart, linkStart, bareLinkStart});

            if (nextSpecial == std::string::npos) {
                // No more special elements, render remaining text
                std::string remaining = text.substr(pos);
                if (!remaining.empty()) {
                    ImGui::PushStyleColor(ImGuiCol_Text, color);
                    ImGui::TextWrapped("%s", remaining.c_str());
                    ImGui::PopStyleColor();
                }
                break;
            }

            // Render plain text before special element
            if (nextSpecial > pos) {
                std::string before = text.substr(pos, nextSpecial - pos);
                ImGui::PushStyleColor(ImGuiCol_Text, color);
                ImGui::TextWrapped("%s", before.c_str());
                ImGui::PopStyleColor();
                ImGui::SameLine(0, 0);
            }

            // Handle WoW item link
            if (nextSpecial == linkStart || nextSpecial == bareLinkStart) {
                ImVec4 linkColor = color;
                size_t hStart = std::string::npos;

                if (nextSpecial == linkStart && text.size() > linkStart + 10) {
                    // Parse |cAARRGGBB color
                    linkColor = parseWowColor(text, linkStart);
                    hStart = text.find("|Hitem:", linkStart + 10);
                } else if (nextSpecial == bareLinkStart) {
                    hStart = bareLinkStart;
                }

                if (hStart != std::string::npos) {
                    // Parse item entry: |Hitem:ENTRY:...
                    size_t entryStart = hStart + 7; // skip "|Hitem:"
                    size_t entryEnd = text.find(':', entryStart);
                    uint32_t itemEntry = 0;
                    if (entryEnd != std::string::npos) {
                        itemEntry = static_cast<uint32_t>(strtoul(
                            text.substr(entryStart, entryEnd - entryStart).c_str(), nullptr, 10));
                    }

                    // Find display name: |h[Name]|h
                    size_t nameTagStart = text.find("|h[", hStart);
                    size_t nameTagEnd = (nameTagStart != std::string::npos)
                        ? text.find("]|h", nameTagStart + 3) : std::string::npos;

                    std::string itemName = "Unknown Item";
                    if (nameTagStart != std::string::npos && nameTagEnd != std::string::npos) {
                        itemName = text.substr(nameTagStart + 3, nameTagEnd - nameTagStart - 3);
                    }

                    // Find end of entire link sequence (|r or after ]|h)
                    size_t linkEnd = (nameTagEnd != std::string::npos) ? nameTagEnd + 3 : hStart + 7;
                    size_t resetPos = text.find("|r", linkEnd);
                    if (resetPos != std::string::npos && resetPos <= linkEnd + 2) {
                        linkEnd = resetPos + 2;
                    }

                    // Ensure item info is cached (trigger query if needed)
                    if (itemEntry > 0) {
                        gameHandler.ensureItemInfo(itemEntry);
                    }

                    // Render bracketed item name in quality color
                    std::string display = "[" + itemName + "]";
                    ImGui::PushStyleColor(ImGuiCol_Text, linkColor);
                    ImGui::TextWrapped("%s", display.c_str());
                    ImGui::PopStyleColor();

                    if (ImGui::IsItemHovered()) {
                        ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
                        if (itemEntry > 0) {
                            renderItemLinkTooltip(itemEntry);
                        }
                    }

                    // Shift-click: insert item link into chat input
                    if (ImGui::IsItemClicked() && ImGui::GetIO().KeyShift) {
                        std::string linkText = text.substr(nextSpecial, linkEnd - nextSpecial);
                        size_t curLen = strlen(chatInputBuffer);
                        if (curLen + linkText.size() + 1 < sizeof(chatInputBuffer)) {
                            strncat(chatInputBuffer, linkText.c_str(), sizeof(chatInputBuffer) - curLen - 1);
                            chatInputMoveCursorToEnd = true;
                        }
                    }

                    pos = linkEnd;
                    continue;
                }

                // Not an item link — treat as colored text: |cAARRGGBB...text...|r
                if (nextSpecial == linkStart && text.size() > linkStart + 10) {
                    ImVec4 cColor = parseWowColor(text, linkStart);
                    size_t textStart = linkStart + 10; // after |cAARRGGBB
                    size_t resetPos2 = text.find("|r", textStart);
                    std::string coloredText;
                    if (resetPos2 != std::string::npos) {
                        coloredText = text.substr(textStart, resetPos2 - textStart);
                        pos = resetPos2 + 2; // skip |r
                    } else {
                        coloredText = text.substr(textStart);
                        pos = text.size();
                    }
                    // Strip any remaining WoW markup from the colored segment
                    // (e.g. |H...|h pairs that aren't item links)
                    std::string clean;
                    for (size_t i = 0; i < coloredText.size(); i++) {
                        if (coloredText[i] == '|' && i + 1 < coloredText.size()) {
                            char next = coloredText[i + 1];
                            if (next == 'H') {
                                // Skip |H...|h
                                size_t hEnd = coloredText.find("|h", i + 2);
                                if (hEnd != std::string::npos) { i = hEnd + 1; continue; }
                            } else if (next == 'h') {
                                i += 1; continue; // skip |h
                            } else if (next == 'r') {
                                i += 1; continue; // skip |r
                            }
                        }
                        clean += coloredText[i];
                    }
                    if (!clean.empty()) {
                        ImGui::PushStyleColor(ImGuiCol_Text, cColor);
                        ImGui::TextWrapped("%s", clean.c_str());
                        ImGui::PopStyleColor();
                        ImGui::SameLine(0, 0);
                    }
                } else {
                    // Bare |c without enough chars for color — render literally
                    ImGui::PushStyleColor(ImGuiCol_Text, color);
                    ImGui::TextWrapped("|c");
                    ImGui::PopStyleColor();
                    ImGui::SameLine(0, 0);
                    pos = nextSpecial + 2;
                }
                continue;
            }

            // Handle URL
            if (nextSpecial == urlStart) {
                size_t urlEnd = text.find_first_of(" \t\n\r", urlStart);
                if (urlEnd == std::string::npos) urlEnd = text.size();
                std::string url = text.substr(urlStart, urlEnd - urlStart);

                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.4f, 0.7f, 1.0f, 1.0f));
                ImGui::TextWrapped("%s", url.c_str());
                if (ImGui::IsItemHovered()) {
                    ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
                    ImGui::SetTooltip("Open: %s", url.c_str());
                }
                if (ImGui::IsItemClicked()) {
                    std::string cmd = "xdg-open '" + url + "' &";
                    [[maybe_unused]] int result = system(cmd.c_str());
                }
                ImGui::PopStyleColor();

                pos = urlEnd;
                continue;
            }
        }
    };

    for (const auto& msg : chatHistory) {
        if (!shouldShowMessage(msg, activeChatTab_)) continue;

        ImVec4 color = getChatTypeColor(msg.type);

        if (msg.type == game::ChatType::SYSTEM) {
            renderTextWithLinks(msg.message, color);
        } else if (msg.type == game::ChatType::TEXT_EMOTE) {
            renderTextWithLinks(msg.message, color);
        } else if (!msg.senderName.empty()) {
            if (msg.type == game::ChatType::MONSTER_SAY || msg.type == game::ChatType::MONSTER_YELL) {
                std::string prefix = msg.senderName + " says: ";
                ImGui::PushStyleColor(ImGuiCol_Text, color);
                ImGui::TextWrapped("%s", prefix.c_str());
                ImGui::PopStyleColor();
                ImGui::SameLine(0, 0);
                renderTextWithLinks(msg.message, color);
            } else {
                std::string prefix = "[" + std::string(getChatTypeName(msg.type)) + "] " + msg.senderName + ": ";
                ImGui::PushStyleColor(ImGuiCol_Text, color);
                ImGui::TextWrapped("%s", prefix.c_str());
                ImGui::PopStyleColor();
                ImGui::SameLine(0, 0);
                renderTextWithLinks(msg.message, color);
            }
        } else {
            std::string prefix = "[" + std::string(getChatTypeName(msg.type)) + "] ";
            ImGui::PushStyleColor(ImGuiCol_Text, color);
            ImGui::TextWrapped("%s", prefix.c_str());
            ImGui::PopStyleColor();
            ImGui::SameLine(0, 0);
            renderTextWithLinks(msg.message, color);
        }
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
    const char* chatTypes[] = { "SAY", "YELL", "PARTY", "GUILD", "WHISPER", "RAID", "OFFICER", "BATTLEGROUND", "RAID WARNING", "INSTANCE" };
    ImGui::Combo("##ChatType", &selectedChatType, chatTypes, 10);

    // Auto-fill whisper target when switching to WHISPER mode
    if (selectedChatType == 4 && lastChatType != 4) {
        // Just switched to WHISPER mode
        if (gameHandler.hasTarget()) {
            auto target = gameHandler.getTarget();
            if (target && target->getType() == game::ObjectType::PLAYER) {
                auto player = std::static_pointer_cast<game::Player>(target);
                if (!player->getName().empty()) {
                    strncpy(whisperTargetBuffer, player->getName().c_str(), sizeof(whisperTargetBuffer) - 1);
                    whisperTargetBuffer[sizeof(whisperTargetBuffer) - 1] = '\0';
                }
            }
        }
    }
    lastChatType = selectedChatType;

    // Show whisper target field if WHISPER is selected
    if (selectedChatType == 4) {
        ImGui::SameLine();
        ImGui::Text("To:");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120);
        ImGui::InputText("##WhisperTarget", whisperTargetBuffer, sizeof(whisperTargetBuffer));
    }

    ImGui::SameLine();
    ImGui::Text("Message:");
    ImGui::SameLine();

    ImGui::SetNextItemWidth(-1);
    if (refocusChatInput) {
        ImGui::SetKeyboardFocusHere();
        refocusChatInput = false;
    }

    // Detect chat channel prefix as user types and switch the dropdown
    {
        std::string buf(chatInputBuffer);
        if (buf.size() >= 2 && buf[0] == '/') {
            // Find the command and check if there's a space after it
            size_t sp = buf.find(' ', 1);
            if (sp != std::string::npos) {
                std::string cmd = buf.substr(1, sp - 1);
                for (char& c : cmd) c = std::tolower(c);
                int detected = -1;
                if (cmd == "s" || cmd == "say") detected = 0;
                else if (cmd == "y" || cmd == "yell" || cmd == "shout") detected = 1;
                else if (cmd == "p" || cmd == "party") detected = 2;
                else if (cmd == "g" || cmd == "guild") detected = 3;
                else if (cmd == "w" || cmd == "whisper" || cmd == "tell" || cmd == "t") detected = 4;
                else if (cmd == "raid" || cmd == "rsay" || cmd == "ra") detected = 5;
                else if (cmd == "o" || cmd == "officer" || cmd == "osay") detected = 6;
                else if (cmd == "bg" || cmd == "battleground") detected = 7;
                else if (cmd == "rw" || cmd == "raidwarning") detected = 8;
                else if (cmd == "i" || cmd == "instance") detected = 9;
                if (detected >= 0 && selectedChatType != detected) {
                    selectedChatType = detected;
                    // Strip the prefix, keep only the message part
                    std::string remaining = buf.substr(sp + 1);
                    // For whisper, first word after /w is the target
                    if (detected == 4) {
                        size_t msgStart = remaining.find(' ');
                        if (msgStart != std::string::npos) {
                            std::string wTarget = remaining.substr(0, msgStart);
                            strncpy(whisperTargetBuffer, wTarget.c_str(), sizeof(whisperTargetBuffer) - 1);
                            whisperTargetBuffer[sizeof(whisperTargetBuffer) - 1] = '\0';
                            remaining = remaining.substr(msgStart + 1);
                        } else {
                            // Just the target name so far, no message yet
                            strncpy(whisperTargetBuffer, remaining.c_str(), sizeof(whisperTargetBuffer) - 1);
                            whisperTargetBuffer[sizeof(whisperTargetBuffer) - 1] = '\0';
                            remaining = "";
                        }
                    }
                    strncpy(chatInputBuffer, remaining.c_str(), sizeof(chatInputBuffer) - 1);
                    chatInputBuffer[sizeof(chatInputBuffer) - 1] = '\0';
                    chatInputMoveCursorToEnd = true;
                }
            }
        }
    }

    // Color the input text based on current chat type
    ImVec4 inputColor;
    switch (selectedChatType) {
        case 1: inputColor = ImVec4(1.0f, 0.3f, 0.3f, 1.0f); break;  // YELL - red
        case 2: inputColor = ImVec4(0.4f, 0.6f, 1.0f, 1.0f); break;  // PARTY - blue
        case 3: inputColor = ImVec4(0.3f, 1.0f, 0.3f, 1.0f); break;  // GUILD - green
        case 4: inputColor = ImVec4(1.0f, 0.5f, 1.0f, 1.0f); break;  // WHISPER - pink
        case 5: inputColor = ImVec4(1.0f, 0.5f, 0.0f, 1.0f); break;  // RAID - orange
        case 6: inputColor = ImVec4(0.3f, 1.0f, 0.3f, 1.0f); break;  // OFFICER - green
        case 7: inputColor = ImVec4(1.0f, 0.5f, 0.0f, 1.0f); break;  // BG - orange
        case 8: inputColor = ImVec4(1.0f, 0.3f, 0.0f, 1.0f); break;  // RAID WARNING - red-orange
        case 9: inputColor = ImVec4(0.4f, 0.6f, 1.0f, 1.0f); break;  // INSTANCE - blue
        default: inputColor = ImVec4(1.0f, 1.0f, 1.0f, 1.0f); break; // SAY - white
    }
    ImGui::PushStyleColor(ImGuiCol_Text, inputColor);

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
    ImGui::PopStyleColor();

    if (ImGui::IsItemActive()) {
        chatInputActive = true;
    } else {
        chatInputActive = false;
    }

    // Click in chat history area (received messages) → focus input.
    {
        if (chatHistoryHovered && ImGui::IsMouseClicked(0)) {
            refocusChatInput = true;
        }
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
            if (showSettingsWindow) {
                // Close settings window if open
                showSettingsWindow = false;
            } else if (showEscapeMenu) {
                showEscapeMenu = false;
                showEscapeSettingsNotice = false;
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

    // Left-click targeting: only on mouse-up if the mouse didn't drag (camera rotate)
    // Record press position on mouse-down
    if (!io.WantCaptureMouse && input.isMouseButtonJustPressed(SDL_BUTTON_LEFT) && !input.isMouseButtonPressed(SDL_BUTTON_RIGHT)) {
        leftClickPressPos_ = input.getMousePosition();
        leftClickWasPress_ = true;
    }

    // On mouse-up, check if it was a click (not a drag)
    if (leftClickWasPress_ && input.isMouseButtonJustReleased(SDL_BUTTON_LEFT)) {
        leftClickWasPress_ = false;
        glm::vec2 releasePos = input.getMousePosition();
        float dragDist = glm::length(releasePos - leftClickPressPos_);
        constexpr float CLICK_THRESHOLD = 5.0f;  // pixels

        if (dragDist < CLICK_THRESHOLD) {
            auto* renderer = core::Application::getInstance().getRenderer();
            auto* camera = renderer ? renderer->getCamera() : nullptr;
            auto* window = core::Application::getInstance().getWindow();

            if (camera && window) {
                float screenW = static_cast<float>(window->getWidth());
                float screenH = static_cast<float>(window->getHeight());

                rendering::Ray ray = camera->screenToWorldRay(leftClickPressPos_.x, leftClickPressPos_.y, screenW, screenH);

                float closestT = 1e30f;
                uint64_t closestGuid = 0;

                const uint64_t myGuid = gameHandler.getPlayerGuid();
                for (const auto& [guid, entity] : gameHandler.getEntityManager().getEntities()) {
                    auto t = entity->getType();
                    if (t != game::ObjectType::UNIT &&
                        t != game::ObjectType::PLAYER &&
                        t != game::ObjectType::GAMEOBJECT) continue;
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
                        } else if (t == game::ObjectType::GAMEOBJECT) {
                            hitRadius = 1.2f;
                            heightOffset = 0.8f;
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
                    if (t != game::ObjectType::UNIT &&
                        t != game::ObjectType::PLAYER &&
                        t != game::ObjectType::GAMEOBJECT) continue;
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
                        } else if (t == game::ObjectType::GAMEOBJECT) {
                            hitRadius = 1.2f;
                            heightOffset = 0.8f;
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
                    } else {
                        // Interact with friendly NPCs; hostile units just get targeted
                        auto isSpiritNpc = [&]() -> bool {
                            constexpr uint32_t NPC_FLAG_SPIRIT_GUIDE = 0x00004000;
                            constexpr uint32_t NPC_FLAG_SPIRIT_HEALER = 0x00008000;
                            if (unit->getNpcFlags() & (NPC_FLAG_SPIRIT_GUIDE | NPC_FLAG_SPIRIT_HEALER)) {
                                return true;
                            }
                            std::string name = unit->getName();
                            std::transform(name.begin(), name.end(), name.begin(),
                                           [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
                            return (name.find("spirit healer") != std::string::npos) ||
                                   (name.find("spirit guide") != std::string::npos);
                        };
                        bool allowSpiritInteract = (gameHandler.isPlayerDead() || gameHandler.isPlayerGhost()) && isSpiritNpc();
                        if (!unit->isHostile() && (unit->isInteractable() || allowSpiritInteract)) {
                            gameHandler.interactWithNpc(target->getGuid());
                        }
                    }
                } else if (target->getType() == game::ObjectType::GAMEOBJECT) {
                    gameHandler.interactWithGameObject(target->getGuid());
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
    bool isHostileTarget = gameHandler.isHostileAttacker(target->getGuid());
    if (!isHostileTarget && target->getType() == game::ObjectType::UNIT) {
        auto u = std::static_pointer_cast<game::Unit>(target);
        isHostileTarget = u->isHostile();
    }
    ImVec4 borderColor = ImVec4(hostileColor.x * 0.8f, hostileColor.y * 0.8f, hostileColor.z * 0.8f, 1.0f);
    if (isHostileTarget) {
        float t = ImGui::GetTime();
        float pulse = (std::fmod(t, 0.6f) < 0.3f) ? 1.0f : 0.0f;
        borderColor = ImVec4(1.0f, 0.1f, 0.1f, pulse);
    } else if (gameHandler.isAutoAttacking()) {
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
        game::ChatType type;
        std::string message = input;
        std::string target;

        // Track if a channel shortcut should change the chat type dropdown
        int switchChatType = -1;

        // Check for slash commands
        if (input.size() > 1 && input[0] == '/') {
            std::string command = input.substr(1);
            size_t spacePos = command.find(' ');
            std::string cmd = (spacePos != std::string::npos) ? command.substr(0, spacePos) : command;

            // Convert command to lowercase for comparison
            std::string cmdLower = cmd;
            for (char& c : cmdLower) c = std::tolower(c);

            // Special commands
            if (cmdLower == "logout") {
                core::Application::getInstance().logoutToLogin();
                chatInputBuffer[0] = '\0';
                return;
            }

            // /invite command
            if (cmdLower == "invite" && spacePos != std::string::npos) {
                std::string targetName = command.substr(spacePos + 1);
                gameHandler.inviteToGroup(targetName);
                chatInputBuffer[0] = '\0';
                return;
            }

            // /inspect command
            if (cmdLower == "inspect") {
                gameHandler.inspectTarget();
                chatInputBuffer[0] = '\0';
                return;
            }

            // /time command
            if (cmdLower == "time") {
                gameHandler.queryServerTime();
                chatInputBuffer[0] = '\0';
                return;
            }

            // /played command
            if (cmdLower == "played") {
                gameHandler.requestPlayedTime();
                chatInputBuffer[0] = '\0';
                return;
            }

            // /who commands
            if (cmdLower == "who" || cmdLower == "whois" || cmdLower == "online" || cmdLower == "players") {
                std::string query;
                if (spacePos != std::string::npos) {
                    query = command.substr(spacePos + 1);
                    // Trim leading/trailing whitespace
                    size_t first = query.find_first_not_of(" \t\r\n");
                    if (first == std::string::npos) {
                        query.clear();
                    } else {
                        size_t last = query.find_last_not_of(" \t\r\n");
                        query = query.substr(first, last - first + 1);
                    }
                }

                if ((cmdLower == "whois") && query.empty()) {
                    game::MessageChatData msg;
                    msg.type = game::ChatType::SYSTEM;
                    msg.language = game::ChatLanguage::UNIVERSAL;
                    msg.message = "Usage: /whois <playerName>";
                    gameHandler.addLocalChatMessage(msg);
                    chatInputBuffer[0] = '\0';
                    return;
                }

                if (cmdLower == "who" && (query == "help" || query == "?")) {
                    game::MessageChatData msg;
                    msg.type = game::ChatType::SYSTEM;
                    msg.language = game::ChatLanguage::UNIVERSAL;
                    msg.message = "Who commands: /who [name/filter], /whois <name>, /online";
                    gameHandler.addLocalChatMessage(msg);
                    chatInputBuffer[0] = '\0';
                    return;
                }

                gameHandler.queryWho(query);
                chatInputBuffer[0] = '\0';
                return;
            }

            // /roll command
            if (cmdLower == "roll" || cmdLower == "random" || cmdLower == "rnd") {
                uint32_t minRoll = 1;
                uint32_t maxRoll = 100;

                if (spacePos != std::string::npos) {
                    std::string args = command.substr(spacePos + 1);
                    size_t dashPos = args.find('-');
                    size_t spacePos2 = args.find(' ');

                    if (dashPos != std::string::npos) {
                        // Format: /roll 1-100
                        try {
                            minRoll = std::stoul(args.substr(0, dashPos));
                            maxRoll = std::stoul(args.substr(dashPos + 1));
                        } catch (...) {}
                    } else if (spacePos2 != std::string::npos) {
                        // Format: /roll 1 100
                        try {
                            minRoll = std::stoul(args.substr(0, spacePos2));
                            maxRoll = std::stoul(args.substr(spacePos2 + 1));
                        } catch (...) {}
                    } else {
                        // Format: /roll 100 (means 1-100)
                        try {
                            maxRoll = std::stoul(args);
                        } catch (...) {}
                    }
                }

                gameHandler.randomRoll(minRoll, maxRoll);
                chatInputBuffer[0] = '\0';
                return;
            }

            // /friend or /addfriend command
            if (cmdLower == "friend" || cmdLower == "addfriend") {
                if (spacePos != std::string::npos) {
                    std::string args = command.substr(spacePos + 1);
                    size_t subCmdSpace = args.find(' ');

                    if (cmdLower == "friend" && subCmdSpace != std::string::npos) {
                        std::string subCmd = args.substr(0, subCmdSpace);
                        std::transform(subCmd.begin(), subCmd.end(), subCmd.begin(), ::tolower);

                        if (subCmd == "add") {
                            std::string playerName = args.substr(subCmdSpace + 1);
                            gameHandler.addFriend(playerName);
                            chatInputBuffer[0] = '\0';
                            return;
                        } else if (subCmd == "remove" || subCmd == "delete" || subCmd == "rem") {
                            std::string playerName = args.substr(subCmdSpace + 1);
                            gameHandler.removeFriend(playerName);
                            chatInputBuffer[0] = '\0';
                            return;
                        }
                    } else {
                        // /addfriend name or /friend name (assume add)
                        gameHandler.addFriend(args);
                        chatInputBuffer[0] = '\0';
                        return;
                    }
                }

                game::MessageChatData msg;
                msg.type = game::ChatType::SYSTEM;
                msg.language = game::ChatLanguage::UNIVERSAL;
                msg.message = "Usage: /friend add <name> or /friend remove <name>";
                gameHandler.addLocalChatMessage(msg);
                chatInputBuffer[0] = '\0';
                return;
            }

            // /removefriend or /delfriend command
            if (cmdLower == "removefriend" || cmdLower == "delfriend" || cmdLower == "remfriend") {
                if (spacePos != std::string::npos) {
                    std::string playerName = command.substr(spacePos + 1);
                    gameHandler.removeFriend(playerName);
                    chatInputBuffer[0] = '\0';
                    return;
                }

                game::MessageChatData msg;
                msg.type = game::ChatType::SYSTEM;
                msg.language = game::ChatLanguage::UNIVERSAL;
                msg.message = "Usage: /removefriend <name>";
                gameHandler.addLocalChatMessage(msg);
                chatInputBuffer[0] = '\0';
                return;
            }

            // /ignore command
            if (cmdLower == "ignore") {
                if (spacePos != std::string::npos) {
                    std::string playerName = command.substr(spacePos + 1);
                    gameHandler.addIgnore(playerName);
                    chatInputBuffer[0] = '\0';
                    return;
                }

                game::MessageChatData msg;
                msg.type = game::ChatType::SYSTEM;
                msg.language = game::ChatLanguage::UNIVERSAL;
                msg.message = "Usage: /ignore <name>";
                gameHandler.addLocalChatMessage(msg);
                chatInputBuffer[0] = '\0';
                return;
            }

            // /unignore command
            if (cmdLower == "unignore") {
                if (spacePos != std::string::npos) {
                    std::string playerName = command.substr(spacePos + 1);
                    gameHandler.removeIgnore(playerName);
                    chatInputBuffer[0] = '\0';
                    return;
                }

                game::MessageChatData msg;
                msg.type = game::ChatType::SYSTEM;
                msg.language = game::ChatLanguage::UNIVERSAL;
                msg.message = "Usage: /unignore <name>";
                gameHandler.addLocalChatMessage(msg);
                chatInputBuffer[0] = '\0';
                return;
            }

            // /dismount command
            if (cmdLower == "dismount") {
                gameHandler.dismount();
                chatInputBuffer[0] = '\0';
                return;
            }

            // /sit command
            if (cmdLower == "sit") {
                gameHandler.setStandState(1);  // 1 = sit
                chatInputBuffer[0] = '\0';
                return;
            }

            // /stand command
            if (cmdLower == "stand") {
                gameHandler.setStandState(0);  // 0 = stand
                chatInputBuffer[0] = '\0';
                return;
            }

            // /kneel command
            if (cmdLower == "kneel") {
                gameHandler.setStandState(8);  // 8 = kneel
                chatInputBuffer[0] = '\0';
                return;
            }

            // /logout command (already exists but using /logout instead of going to login)
            if (cmdLower == "logout" || cmdLower == "camp") {
                gameHandler.requestLogout();
                chatInputBuffer[0] = '\0';
                return;
            }

            // /cancellogout command
            if (cmdLower == "cancellogout") {
                gameHandler.cancelLogout();
                chatInputBuffer[0] = '\0';
                return;
            }

            // /helm command
            if (cmdLower == "helm" || cmdLower == "helmet" || cmdLower == "showhelm") {
                gameHandler.toggleHelm();
                chatInputBuffer[0] = '\0';
                return;
            }

            // /cloak command
            if (cmdLower == "cloak" || cmdLower == "showcloak") {
                gameHandler.toggleCloak();
                chatInputBuffer[0] = '\0';
                return;
            }

            // /follow command
            if (cmdLower == "follow" || cmdLower == "f") {
                gameHandler.followTarget();
                chatInputBuffer[0] = '\0';
                return;
            }

            // /assist command
            if (cmdLower == "assist") {
                gameHandler.assistTarget();
                chatInputBuffer[0] = '\0';
                return;
            }

            // /pvp command
            if (cmdLower == "pvp") {
                gameHandler.togglePvp();
                chatInputBuffer[0] = '\0';
                return;
            }

            // /ginfo command
            if (cmdLower == "ginfo" || cmdLower == "guildinfo") {
                gameHandler.requestGuildInfo();
                chatInputBuffer[0] = '\0';
                return;
            }

            // /groster command
            if (cmdLower == "groster" || cmdLower == "guildroster") {
                gameHandler.requestGuildRoster();
                chatInputBuffer[0] = '\0';
                return;
            }

            // /gmotd command
            if (cmdLower == "gmotd" || cmdLower == "guildmotd") {
                if (spacePos != std::string::npos) {
                    std::string motd = command.substr(spacePos + 1);
                    gameHandler.setGuildMotd(motd);
                    chatInputBuffer[0] = '\0';
                    return;
                }

                game::MessageChatData msg;
                msg.type = game::ChatType::SYSTEM;
                msg.language = game::ChatLanguage::UNIVERSAL;
                msg.message = "Usage: /gmotd <message>";
                gameHandler.addLocalChatMessage(msg);
                chatInputBuffer[0] = '\0';
                return;
            }

            // /gpromote command
            if (cmdLower == "gpromote" || cmdLower == "guildpromote") {
                if (spacePos != std::string::npos) {
                    std::string playerName = command.substr(spacePos + 1);
                    gameHandler.promoteGuildMember(playerName);
                    chatInputBuffer[0] = '\0';
                    return;
                }

                game::MessageChatData msg;
                msg.type = game::ChatType::SYSTEM;
                msg.language = game::ChatLanguage::UNIVERSAL;
                msg.message = "Usage: /gpromote <player>";
                gameHandler.addLocalChatMessage(msg);
                chatInputBuffer[0] = '\0';
                return;
            }

            // /gdemote command
            if (cmdLower == "gdemote" || cmdLower == "guilddemote") {
                if (spacePos != std::string::npos) {
                    std::string playerName = command.substr(spacePos + 1);
                    gameHandler.demoteGuildMember(playerName);
                    chatInputBuffer[0] = '\0';
                    return;
                }

                game::MessageChatData msg;
                msg.type = game::ChatType::SYSTEM;
                msg.language = game::ChatLanguage::UNIVERSAL;
                msg.message = "Usage: /gdemote <player>";
                gameHandler.addLocalChatMessage(msg);
                chatInputBuffer[0] = '\0';
                return;
            }

            // /gquit command
            if (cmdLower == "gquit" || cmdLower == "guildquit" || cmdLower == "leaveguild") {
                gameHandler.leaveGuild();
                chatInputBuffer[0] = '\0';
                return;
            }

            // /ginvite command
            if (cmdLower == "ginvite" || cmdLower == "guildinvite") {
                if (spacePos != std::string::npos) {
                    std::string playerName = command.substr(spacePos + 1);
                    gameHandler.inviteToGuild(playerName);
                    chatInputBuffer[0] = '\0';
                    return;
                }

                game::MessageChatData msg;
                msg.type = game::ChatType::SYSTEM;
                msg.language = game::ChatLanguage::UNIVERSAL;
                msg.message = "Usage: /ginvite <player>";
                gameHandler.addLocalChatMessage(msg);
                chatInputBuffer[0] = '\0';
                return;
            }

            // /gkick command
            if (cmdLower == "gkick" || cmdLower == "guildkick") {
                if (spacePos != std::string::npos) {
                    std::string playerName = command.substr(spacePos + 1);
                    gameHandler.kickGuildMember(playerName);
                    chatInputBuffer[0] = '\0';
                    return;
                }

                game::MessageChatData msg;
                msg.type = game::ChatType::SYSTEM;
                msg.language = game::ChatLanguage::UNIVERSAL;
                msg.message = "Usage: /gkick <player>";
                gameHandler.addLocalChatMessage(msg);
                chatInputBuffer[0] = '\0';
                return;
            }

            // /readycheck command
            if (cmdLower == "readycheck" || cmdLower == "rc") {
                gameHandler.initiateReadyCheck();
                chatInputBuffer[0] = '\0';
                return;
            }

            // /ready command (respond yes to ready check)
            if (cmdLower == "ready") {
                gameHandler.respondToReadyCheck(true);
                chatInputBuffer[0] = '\0';
                return;
            }

            // /notready command (respond no to ready check)
            if (cmdLower == "notready" || cmdLower == "nr") {
                gameHandler.respondToReadyCheck(false);
                chatInputBuffer[0] = '\0';
                return;
            }

            // /yield or /forfeit command
            if (cmdLower == "yield" || cmdLower == "forfeit" || cmdLower == "surrender") {
                gameHandler.forfeitDuel();
                chatInputBuffer[0] = '\0';
                return;
            }

            // AFK command
            if (cmdLower == "afk" || cmdLower == "away") {
                std::string afkMsg = (spacePos != std::string::npos) ? command.substr(spacePos + 1) : "";
                gameHandler.toggleAfk(afkMsg);
                chatInputBuffer[0] = '\0';
                return;
            }

            // DND command
            if (cmdLower == "dnd" || cmdLower == "busy") {
                std::string dndMsg = (spacePos != std::string::npos) ? command.substr(spacePos + 1) : "";
                gameHandler.toggleDnd(dndMsg);
                chatInputBuffer[0] = '\0';
                return;
            }

            // Reply command
            if (cmdLower == "r" || cmdLower == "reply") {
                std::string replyMsg = (spacePos != std::string::npos) ? command.substr(spacePos + 1) : "";
                gameHandler.replyToLastWhisper(replyMsg);
                chatInputBuffer[0] = '\0';
                return;
            }

            // Party/Raid management commands
            if (cmdLower == "uninvite" || cmdLower == "kick") {
                if (spacePos != std::string::npos) {
                    std::string playerName = command.substr(spacePos + 1);
                    gameHandler.uninvitePlayer(playerName);
                } else {
                    game::MessageChatData msg;
                    msg.type = game::ChatType::SYSTEM;
                    msg.language = game::ChatLanguage::UNIVERSAL;
                    msg.message = "Usage: /uninvite <player name>";
                    gameHandler.addLocalChatMessage(msg);
                }
                chatInputBuffer[0] = '\0';
                return;
            }

            if (cmdLower == "leave" || cmdLower == "leaveparty") {
                gameHandler.leaveParty();
                chatInputBuffer[0] = '\0';
                return;
            }

            if (cmdLower == "maintank" || cmdLower == "mt") {
                if (gameHandler.hasTarget()) {
                    gameHandler.setMainTank(gameHandler.getTargetGuid());
                } else {
                    game::MessageChatData msg;
                    msg.type = game::ChatType::SYSTEM;
                    msg.language = game::ChatLanguage::UNIVERSAL;
                    msg.message = "You must target a player to set as main tank.";
                    gameHandler.addLocalChatMessage(msg);
                }
                chatInputBuffer[0] = '\0';
                return;
            }

            if (cmdLower == "mainassist" || cmdLower == "ma") {
                if (gameHandler.hasTarget()) {
                    gameHandler.setMainAssist(gameHandler.getTargetGuid());
                } else {
                    game::MessageChatData msg;
                    msg.type = game::ChatType::SYSTEM;
                    msg.language = game::ChatLanguage::UNIVERSAL;
                    msg.message = "You must target a player to set as main assist.";
                    gameHandler.addLocalChatMessage(msg);
                }
                chatInputBuffer[0] = '\0';
                return;
            }

            if (cmdLower == "clearmaintank") {
                gameHandler.clearMainTank();
                chatInputBuffer[0] = '\0';
                return;
            }

            if (cmdLower == "clearmainassist") {
                gameHandler.clearMainAssist();
                chatInputBuffer[0] = '\0';
                return;
            }

            if (cmdLower == "raidinfo") {
                gameHandler.requestRaidInfo();
                chatInputBuffer[0] = '\0';
                return;
            }

            // Combat and Trade commands
            if (cmdLower == "duel") {
                if (gameHandler.hasTarget()) {
                    gameHandler.proposeDuel(gameHandler.getTargetGuid());
                } else if (spacePos != std::string::npos) {
                    // Target player by name (would need name-to-GUID lookup)
                    game::MessageChatData msg;
                    msg.type = game::ChatType::SYSTEM;
                    msg.language = game::ChatLanguage::UNIVERSAL;
                    msg.message = "You must target a player to challenge to a duel.";
                    gameHandler.addLocalChatMessage(msg);
                } else {
                    game::MessageChatData msg;
                    msg.type = game::ChatType::SYSTEM;
                    msg.language = game::ChatLanguage::UNIVERSAL;
                    msg.message = "You must target a player to challenge to a duel.";
                    gameHandler.addLocalChatMessage(msg);
                }
                chatInputBuffer[0] = '\0';
                return;
            }

            if (cmdLower == "trade") {
                if (gameHandler.hasTarget()) {
                    gameHandler.initiateTrade(gameHandler.getTargetGuid());
                } else {
                    game::MessageChatData msg;
                    msg.type = game::ChatType::SYSTEM;
                    msg.language = game::ChatLanguage::UNIVERSAL;
                    msg.message = "You must target a player to trade with.";
                    gameHandler.addLocalChatMessage(msg);
                }
                chatInputBuffer[0] = '\0';
                return;
            }

            if (cmdLower == "startattack") {
                if (gameHandler.hasTarget()) {
                    gameHandler.startAutoAttack(gameHandler.getTargetGuid());
                } else {
                    game::MessageChatData msg;
                    msg.type = game::ChatType::SYSTEM;
                    msg.language = game::ChatLanguage::UNIVERSAL;
                    msg.message = "You have no target.";
                    gameHandler.addLocalChatMessage(msg);
                }
                chatInputBuffer[0] = '\0';
                return;
            }

            if (cmdLower == "stopattack") {
                gameHandler.stopAutoAttack();
                chatInputBuffer[0] = '\0';
                return;
            }

            if (cmdLower == "stopcasting") {
                gameHandler.stopCasting();
                chatInputBuffer[0] = '\0';
                return;
            }

            // Targeting commands
            if (cmdLower == "cleartarget") {
                gameHandler.clearTarget();
                chatInputBuffer[0] = '\0';
                return;
            }

            if (cmdLower == "targetenemy") {
                gameHandler.targetEnemy(false);
                chatInputBuffer[0] = '\0';
                return;
            }

            if (cmdLower == "targetfriend") {
                gameHandler.targetFriend(false);
                chatInputBuffer[0] = '\0';
                return;
            }

            if (cmdLower == "targetlasttarget" || cmdLower == "targetlast") {
                gameHandler.targetLastTarget();
                chatInputBuffer[0] = '\0';
                return;
            }

            if (cmdLower == "targetlastenemy") {
                gameHandler.targetEnemy(true);  // Reverse direction
                chatInputBuffer[0] = '\0';
                return;
            }

            if (cmdLower == "targetlastfriend") {
                gameHandler.targetFriend(true);  // Reverse direction
                chatInputBuffer[0] = '\0';
                return;
            }

            if (cmdLower == "focus") {
                if (gameHandler.hasTarget()) {
                    gameHandler.setFocus(gameHandler.getTargetGuid());
                } else {
                    game::MessageChatData msg;
                    msg.type = game::ChatType::SYSTEM;
                    msg.language = game::ChatLanguage::UNIVERSAL;
                    msg.message = "You must target a unit to set as focus.";
                    gameHandler.addLocalChatMessage(msg);
                }
                chatInputBuffer[0] = '\0';
                return;
            }

            if (cmdLower == "clearfocus") {
                gameHandler.clearFocus();
                chatInputBuffer[0] = '\0';
                return;
            }

            // /unstuck command — resets player position to floor height
            if (cmdLower == "unstuck") {
                gameHandler.unstuck();
                chatInputBuffer[0] = '\0';
                return;
            }
            // /unstuckgy command — move to nearest graveyard
            if (cmdLower == "unstuckgy") {
                gameHandler.unstuckGy();
                chatInputBuffer[0] = '\0';
                return;
            }

            // /transport board — board test transport
            if (cmdLower == "transport board") {
                auto* tm = gameHandler.getTransportManager();
                if (tm) {
                    // Test transport GUID
                    uint64_t testTransportGuid = 0x1000000000000001ULL;
                    // Place player at center of deck (rough estimate)
                    glm::vec3 deckCenter(0.0f, 0.0f, 5.0f);
                    gameHandler.setPlayerOnTransport(testTransportGuid, deckCenter);
                    game::MessageChatData msg;
                    msg.type = game::ChatType::SYSTEM;
                    msg.language = game::ChatLanguage::UNIVERSAL;
                    msg.message = "Boarded test transport. Use '/transport leave' to disembark.";
                    gameHandler.addLocalChatMessage(msg);
                } else {
                    game::MessageChatData msg;
                    msg.type = game::ChatType::SYSTEM;
                    msg.language = game::ChatLanguage::UNIVERSAL;
                    msg.message = "Transport system not available.";
                    gameHandler.addLocalChatMessage(msg);
                }
                chatInputBuffer[0] = '\0';
                return;
            }

            // /transport leave — disembark from transport
            if (cmdLower == "transport leave") {
                if (gameHandler.isOnTransport()) {
                    gameHandler.clearPlayerTransport();
                    game::MessageChatData msg;
                    msg.type = game::ChatType::SYSTEM;
                    msg.language = game::ChatLanguage::UNIVERSAL;
                    msg.message = "Disembarked from transport.";
                    gameHandler.addLocalChatMessage(msg);
                } else {
                    game::MessageChatData msg;
                    msg.type = game::ChatType::SYSTEM;
                    msg.language = game::ChatLanguage::UNIVERSAL;
                    msg.message = "You are not on a transport.";
                    gameHandler.addLocalChatMessage(msg);
                }
                chatInputBuffer[0] = '\0';
                return;
            }

            // Chat channel slash commands
            // If used without a message (e.g. just "/s"), switch the chat type dropdown
            bool isChannelCommand = false;
            if (cmdLower == "s" || cmdLower == "say") {
                type = game::ChatType::SAY;
                message = (spacePos != std::string::npos) ? command.substr(spacePos + 1) : "";
                isChannelCommand = true;
                switchChatType = 0;
            } else if (cmdLower == "y" || cmdLower == "yell" || cmdLower == "shout") {
                type = game::ChatType::YELL;
                message = (spacePos != std::string::npos) ? command.substr(spacePos + 1) : "";
                isChannelCommand = true;
                switchChatType = 1;
            } else if (cmdLower == "p" || cmdLower == "party") {
                type = game::ChatType::PARTY;
                message = (spacePos != std::string::npos) ? command.substr(spacePos + 1) : "";
                isChannelCommand = true;
                switchChatType = 2;
            } else if (cmdLower == "g" || cmdLower == "guild") {
                type = game::ChatType::GUILD;
                message = (spacePos != std::string::npos) ? command.substr(spacePos + 1) : "";
                isChannelCommand = true;
                switchChatType = 3;
            } else if (cmdLower == "raid" || cmdLower == "rsay" || cmdLower == "ra") {
                type = game::ChatType::RAID;
                message = (spacePos != std::string::npos) ? command.substr(spacePos + 1) : "";
                isChannelCommand = true;
                switchChatType = 5;
            } else if (cmdLower == "raidwarning" || cmdLower == "rw") {
                type = game::ChatType::RAID_WARNING;
                message = (spacePos != std::string::npos) ? command.substr(spacePos + 1) : "";
                isChannelCommand = true;
                switchChatType = 8;
            } else if (cmdLower == "officer" || cmdLower == "o" || cmdLower == "osay") {
                type = game::ChatType::OFFICER;
                message = (spacePos != std::string::npos) ? command.substr(spacePos + 1) : "";
                isChannelCommand = true;
                switchChatType = 6;
            } else if (cmdLower == "battleground" || cmdLower == "bg") {
                type = game::ChatType::BATTLEGROUND;
                message = (spacePos != std::string::npos) ? command.substr(spacePos + 1) : "";
                isChannelCommand = true;
                switchChatType = 7;
            } else if (cmdLower == "instance" || cmdLower == "i") {
                // Instance chat uses PARTY chat type
                type = game::ChatType::PARTY;
                message = (spacePos != std::string::npos) ? command.substr(spacePos + 1) : "";
                isChannelCommand = true;
                switchChatType = 9;
            } else if (cmdLower == "join") {
                // /join ChannelName [password]
                if (spacePos != std::string::npos) {
                    std::string rest = command.substr(spacePos + 1);
                    size_t pwStart = rest.find(' ');
                    std::string channelName = (pwStart != std::string::npos) ? rest.substr(0, pwStart) : rest;
                    std::string password = (pwStart != std::string::npos) ? rest.substr(pwStart + 1) : "";
                    gameHandler.joinChannel(channelName, password);
                }
                chatInputBuffer[0] = '\0';
                return;
            } else if (cmdLower == "leave") {
                // /leave ChannelName
                if (spacePos != std::string::npos) {
                    std::string channelName = command.substr(spacePos + 1);
                    gameHandler.leaveChannel(channelName);
                }
                chatInputBuffer[0] = '\0';
                return;
            } else if (cmdLower.size() == 1 && cmdLower[0] >= '1' && cmdLower[0] <= '9') {
                // /1 msg, /2 msg — channel shortcuts
                int channelIdx = cmdLower[0] - '0';
                std::string channelName = gameHandler.getChannelByIndex(channelIdx);
                if (!channelName.empty() && spacePos != std::string::npos) {
                    message = command.substr(spacePos + 1);
                    type = game::ChatType::CHANNEL;
                    target = channelName;
                    isChannelCommand = true;
                } else if (channelName.empty()) {
                    game::MessageChatData errMsg;
                    errMsg.type = game::ChatType::SYSTEM;
                    errMsg.message = "You are not in channel " + std::to_string(channelIdx) + ".";
                    gameHandler.addLocalChatMessage(errMsg);
                    chatInputBuffer[0] = '\0';
                    return;
                } else {
                    chatInputBuffer[0] = '\0';
                    return;
                }
            } else if (cmdLower == "w" || cmdLower == "whisper" || cmdLower == "tell" || cmdLower == "t") {
                switchChatType = 4;
                if (spacePos != std::string::npos) {
                    std::string rest = command.substr(spacePos + 1);
                    size_t msgStart = rest.find(' ');
                    if (msgStart != std::string::npos) {
                        // /w PlayerName message — send whisper immediately
                        target = rest.substr(0, msgStart);
                        message = rest.substr(msgStart + 1);
                        type = game::ChatType::WHISPER;
                        isChannelCommand = true;
                        // Set whisper target for future messages
                        strncpy(whisperTargetBuffer, target.c_str(), sizeof(whisperTargetBuffer) - 1);
                        whisperTargetBuffer[sizeof(whisperTargetBuffer) - 1] = '\0';
                    } else {
                        // /w PlayerName — switch to whisper mode with target set
                        strncpy(whisperTargetBuffer, rest.c_str(), sizeof(whisperTargetBuffer) - 1);
                        whisperTargetBuffer[sizeof(whisperTargetBuffer) - 1] = '\0';
                        message = "";
                        isChannelCommand = true;
                    }
                } else {
                    // Just "/w" — switch to whisper mode
                    message = "";
                    isChannelCommand = true;
                }
            }

            // Check for emote commands
            if (!isChannelCommand) {
                std::string targetName;
                const std::string* targetNamePtr = nullptr;
                if (gameHandler.hasTarget()) {
                    auto targetEntity = gameHandler.getTarget();
                    if (targetEntity) {
                        targetName = getEntityName(targetEntity);
                        if (!targetName.empty()) targetNamePtr = &targetName;
                    }
                }

                std::string emoteText = rendering::Renderer::getEmoteText(cmdLower, targetNamePtr);
                if (!emoteText.empty()) {
                    // Play the emote animation
                    auto* renderer = core::Application::getInstance().getRenderer();
                    if (renderer) {
                        renderer->playEmote(cmdLower);
                    }

                    // Send CMSG_TEXT_EMOTE to server
                    uint32_t dbcId = rendering::Renderer::getEmoteDbcId(cmdLower);
                    if (dbcId != 0) {
                        uint64_t targetGuid = gameHandler.hasTarget() ? gameHandler.getTargetGuid() : 0;
                        gameHandler.sendTextEmote(dbcId, targetGuid);
                    }

                    // Add local chat message
                    game::MessageChatData msg;
                    msg.type = game::ChatType::TEXT_EMOTE;
                    msg.language = game::ChatLanguage::COMMON;
                    msg.message = emoteText;
                    gameHandler.addLocalChatMessage(msg);

                    chatInputBuffer[0] = '\0';
                    return;
                }

                // Not a recognized command — fall through and send as normal chat
                if (!isChannelCommand) {
                    message = input;
                }
            }

            // If no valid command found and starts with /, just send as-is
            if (!isChannelCommand && message == input) {
                // Use the selected chat type from dropdown
                switch (selectedChatType) {
                    case 0: type = game::ChatType::SAY; break;
                    case 1: type = game::ChatType::YELL; break;
                    case 2: type = game::ChatType::PARTY; break;
                    case 3: type = game::ChatType::GUILD; break;
                    case 4: type = game::ChatType::WHISPER; target = whisperTargetBuffer; break;
                    case 5: type = game::ChatType::RAID; break;
                    case 6: type = game::ChatType::OFFICER; break;
                    case 7: type = game::ChatType::BATTLEGROUND; break;
                    case 8: type = game::ChatType::RAID_WARNING; break;
                    case 9: type = game::ChatType::PARTY; break; // INSTANCE uses PARTY
                    default: type = game::ChatType::SAY; break;
                }
            }
        } else {
            // No slash command, use the selected chat type from dropdown
            switch (selectedChatType) {
                case 0: type = game::ChatType::SAY; break;
                case 1: type = game::ChatType::YELL; break;
                case 2: type = game::ChatType::PARTY; break;
                case 3: type = game::ChatType::GUILD; break;
                case 4: type = game::ChatType::WHISPER; target = whisperTargetBuffer; break;
                case 5: type = game::ChatType::RAID; break;
                case 6: type = game::ChatType::OFFICER; break;
                case 7: type = game::ChatType::BATTLEGROUND; break;
                case 8: type = game::ChatType::RAID_WARNING; break;
                case 9: type = game::ChatType::PARTY; break; // INSTANCE uses PARTY
                default: type = game::ChatType::SAY; break;
            }
        }

        // Whisper shortcuts to PortBot/GMBot: translate to GM teleport commands.
        if (type == game::ChatType::WHISPER && isPortBotTarget(target)) {
            std::string cmd = buildPortBotCommand(message);
            game::MessageChatData msg;
            msg.type = game::ChatType::SYSTEM;
            msg.language = game::ChatLanguage::UNIVERSAL;
            if (cmd.empty() || cmd == "__help__") {
                msg.message = "PortBot: /w PortBot <dest>. Aliases: sw if darn org tb uc shatt dal. Also supports '.tele ...' or 'xyz x y z [map [o]]'.";
                gameHandler.addLocalChatMessage(msg);
                chatInputBuffer[0] = '\0';
                return;
            }

            gameHandler.sendChatMessage(game::ChatType::SAY, cmd, "");
            msg.message = "PortBot executed: " + cmd;
            gameHandler.addLocalChatMessage(msg);
            chatInputBuffer[0] = '\0';
            return;
        }

        // Validate whisper has a target
        if (type == game::ChatType::WHISPER && target.empty()) {
            game::MessageChatData msg;
            msg.type = game::ChatType::SYSTEM;
            msg.language = game::ChatLanguage::UNIVERSAL;
            msg.message = "You must specify a player name for whisper.";
            gameHandler.addLocalChatMessage(msg);
            chatInputBuffer[0] = '\0';
            return;
        }

        // Don't send empty messages — but switch chat type if a channel shortcut was used
        if (!message.empty()) {
            gameHandler.sendChatMessage(type, message, target);
        }

        // Switch chat type dropdown when channel shortcut used (with or without message)
        if (switchChatType >= 0) {
            selectedChatType = switchChatType;
        }

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
        case game::ChatType::BATTLEGROUND: return "BATTLEGROUND";
        case game::ChatType::BATTLEGROUND_LEADER: return "BG LEADER";
        case game::ChatType::WHISPER: return "WHISPER";
        case game::ChatType::WHISPER_INFORM: return "TO";
        case game::ChatType::SYSTEM: return "SYSTEM";
        case game::ChatType::MONSTER_SAY: return "SAY";
        case game::ChatType::MONSTER_YELL: return "YELL";
        case game::ChatType::MONSTER_EMOTE: return "EMOTE";
        case game::ChatType::CHANNEL: return "CHANNEL";
        case game::ChatType::ACHIEVEMENT: return "ACHIEVEMENT";
        case game::ChatType::DND: return "DND";
        case game::ChatType::AFK: return "AFK";
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
        case game::ChatType::RAID_LEADER:
            return ImVec4(1.0f, 0.4f, 0.0f, 1.0f);  // Darker orange
        case game::ChatType::RAID_WARNING:
            return ImVec4(1.0f, 0.0f, 0.0f, 1.0f);  // Red
        case game::ChatType::BATTLEGROUND:
            return ImVec4(1.0f, 0.6f, 0.0f, 1.0f);  // Orange-gold
        case game::ChatType::BATTLEGROUND_LEADER:
            return ImVec4(1.0f, 0.5f, 0.0f, 1.0f);  // Orange
        case game::ChatType::WHISPER:
            return ImVec4(1.0f, 0.5f, 1.0f, 1.0f);  // Pink
        case game::ChatType::WHISPER_INFORM:
            return ImVec4(1.0f, 0.5f, 1.0f, 1.0f);  // Pink
        case game::ChatType::SYSTEM:
            return ImVec4(1.0f, 1.0f, 0.3f, 1.0f);  // Yellow
        case game::ChatType::MONSTER_SAY:
            return ImVec4(1.0f, 1.0f, 1.0f, 1.0f);  // White (same as SAY)
        case game::ChatType::MONSTER_YELL:
            return ImVec4(1.0f, 0.3f, 0.3f, 1.0f);  // Red (same as YELL)
        case game::ChatType::MONSTER_EMOTE:
            return ImVec4(1.0f, 0.7f, 0.3f, 1.0f);  // Orange (same as EMOTE)
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
    // Hair/facial geosets must match the active character's appearance, otherwise
    // we end up forcing a default hair mesh (often perceived as "wrong hair").
    {
        uint8_t hairStyleId = 0;
        uint8_t facialId = 0;
        if (auto* gh = app.getGameHandler()) {
            if (const auto* ch = gh->getActiveCharacter()) {
                hairStyleId = static_cast<uint8_t>((ch->appearanceBytes >> 16) & 0xFF);
                facialId = ch->facialFeatures;
            }
        }
        geosets.insert(static_cast<uint16_t>(100 + hairStyleId + 1)); // Group 1 hair
        geosets.insert(static_cast<uint16_t>(200 + facialId + 1));    // Group 2 facial
    }
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
    const auto* idiL = pipeline::getActiveDBCLayout()
        ? pipeline::getActiveDBCLayout()->getLayout("ItemDisplayInfo") : nullptr;
    // Texture component region fields (8 regions: ArmUpper..Foot)
    // Binary DBC (23 fields) has textures at 14+
    const uint32_t texRegionFields[8] = {
        idiL ? (*idiL)["TextureArmUpper"]  : 14u,
        idiL ? (*idiL)["TextureArmLower"]  : 15u,
        idiL ? (*idiL)["TextureHand"]      : 16u,
        idiL ? (*idiL)["TextureTorsoUpper"]: 17u,
        idiL ? (*idiL)["TextureTorsoLower"]: 18u,
        idiL ? (*idiL)["TextureLegUpper"]  : 19u,
        idiL ? (*idiL)["TextureLegLower"]  : 20u,
        idiL ? (*idiL)["TextureFoot"]      : 21u,
    };

    // Collect equipment texture regions from all equipped items
    std::vector<std::pair<int, std::string>> regionLayers;

    for (int s = 0; s < game::Inventory::NUM_EQUIP_SLOTS; s++) {
        const auto& slot = inventory.getEquipSlot(static_cast<game::EquipSlot>(s));
        if (slot.empty() || slot.item.displayInfoId == 0) continue;

        int32_t recIdx = displayInfoDbc->findRecordById(slot.item.displayInfoId);
        if (recIdx < 0) continue;

        for (int region = 0; region < 8; region++) {
            std::string texName = displayInfoDbc->getString(
                static_cast<uint32_t>(recIdx), texRegionFields[region]);
            if (texName.empty()) continue;

            // Actual MPQ files have a gender suffix: _M (male), _F (female), _U (unisex)
            // Try gender-specific first, then unisex fallback
            std::string base = "Item\\TextureComponents\\" +
                std::string(componentDirs[region]) + "\\" + texName;
            // Determine gender suffix from active character
            bool isFemale = false;
            if (auto* gh = app.getGameHandler()) {
                if (auto* ch = gh->getActiveCharacter()) {
                    isFemale = (ch->gender == game::Gender::FEMALE) ||
                               (ch->gender == game::Gender::NONBINARY && ch->useFemaleModel);
                }
            }
            std::string genderPath = base + (isFemale ? "_F.blp" : "_M.blp");
            std::string unisexPath = base + "_U.blp";
            std::string fullPath;
            if (assetManager->fileExists(genderPath)) {
                fullPath = genderPath;
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
                const auto* dispL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("ItemDisplayInfo") : nullptr;
                std::string capeName = displayInfoDbc->getString(static_cast<uint32_t>(recIdx), dispL ? (*dispL)["LeftModelTexture"] : 3);
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

void GameScreen::renderWorldMap(game::GameHandler& gameHandler) {
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
    worldMap.setServerExplorationMask(
        gameHandler.getPlayerExploredZoneMasks(),
        gameHandler.hasPlayerExploredZoneMasks());

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
        const auto* iconL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("SpellIcon") : nullptr;
        if (iconDbc && iconDbc->isLoaded()) {
            for (uint32_t i = 0; i < iconDbc->getRecordCount(); i++) {
                uint32_t id = iconDbc->getUInt32(i, iconL ? (*iconL)["ID"] : 0);
                std::string path = iconDbc->getString(i, iconL ? (*iconL)["Path"] : 1);
                if (!path.empty() && id > 0) {
                    spellIconPaths_[id] = path;
                }
            }
        }

        // Load Spell.dbc: field 133 = SpellIconID
        auto spellDbc = am->loadDBC("Spell.dbc");
        const auto* spellL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("Spell") : nullptr;
        if (spellDbc && spellDbc->isLoaded() && spellDbc->getFieldCount() > 133) {
            for (uint32_t i = 0; i < spellDbc->getRecordCount(); i++) {
                uint32_t id = spellDbc->getUInt32(i, spellL ? (*spellL)["ID"] : 0);
                uint32_t iconId = spellDbc->getUInt32(i, spellL ? (*spellL)["IconID"] : 133);
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
                            const auto* actionSpellL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("Spell") : nullptr;
                            uint32_t fieldCount = dbc->getFieldCount();
                            uint32_t nameField = actionSpellL ? (*actionSpellL)["Name"] : 136;
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
                                uint32_t id = dbc->getUInt32(r, actionSpellL ? (*actionSpellL)["ID"] : 0);
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

            bool rightClicked = ImGui::IsItemClicked(ImGuiMouseButton_Right);
            bool hoveredOnRelease = ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem) &&
                                    ImGui::IsMouseReleased(ImGuiMouseButton_Left);

            // Drop dragged spell from spellbook onto this slot
            // (mouse release over slot — button click won't fire since press was in spellbook)
            if (hoveredOnRelease && spellbookScreen.isDraggingSpell()) {
                gameHandler.setActionBarSlot(i, game::ActionBarSlot::SPELL,
                    spellbookScreen.getDragSpellId());
                spellbookScreen.consumeDragSpell();
            } else if (clicked && inventoryScreen.isHoldingItem()) {
                // Drop held item from inventory onto action bar
                const auto& held = inventoryScreen.getHeldItem();
                gameHandler.setActionBarSlot(i, game::ActionBarSlot::ITEM, held.itemId);
                inventoryScreen.returnHeldItem(gameHandler.getInventory());
            } else if (clicked && actionBarDragSlot_ >= 0) {
                // Dropping a dragged action bar slot onto another slot - swap or place
                if (i != actionBarDragSlot_) {
                    const auto& dragSrc = bar[actionBarDragSlot_];
                    auto srcType = dragSrc.type;
                    auto srcId = dragSrc.id;
                    gameHandler.setActionBarSlot(actionBarDragSlot_, slot.type, slot.id);
                    gameHandler.setActionBarSlot(i, srcType, srcId);
                }
                actionBarDragSlot_ = -1;
                actionBarDragIcon_ = 0;
            } else if (clicked && !slot.isEmpty()) {
                // Left-click on non-empty slot: cast spell or use item
                if (slot.type == game::ActionBarSlot::SPELL && slot.isReady()) {
                    uint64_t target = gameHandler.hasTarget() ? gameHandler.getTargetGuid() : 0;
                    gameHandler.castSpell(slot.id, target);
                } else if (slot.type == game::ActionBarSlot::ITEM && slot.id != 0) {
                    gameHandler.useItemById(slot.id);
                }
            } else if (rightClicked && !slot.isEmpty()) {
                // Right-click on non-empty slot: pick up for dragging
                actionBarDragSlot_ = i;
                actionBarDragIcon_ = iconTex;
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

        // On right mouse release, check if outside the action bar area
        if (ImGui::IsMouseReleased(ImGuiMouseButton_Right)) {
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
// Bag Bar
// ============================================================

void GameScreen::renderBagBar(game::GameHandler& gameHandler) {
    auto* window = core::Application::getInstance().getWindow();
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;
    auto* assetMgr = core::Application::getInstance().getAssetManager();

    float slotSize = 42.0f;
    float spacing = 4.0f;
    float padding = 6.0f;

    // 5 slots: backpack + 4 bags
    float barW = 5 * slotSize + 4 * spacing + padding * 2;
    float barH = slotSize + padding * 2;

    // Position in bottom right corner
    float barX = screenW - barW - 10.0f;
    float barY = screenH - barH - 10.0f;

    ImGui::SetNextWindowPos(ImVec2(barX, barY), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(barW, barH), ImGuiCond_Always);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                             ImGuiWindowFlags_NoScrollbar;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.05f, 0.05f, 0.05f, 0.9f));

    if (ImGui::Begin("##BagBar", nullptr, flags)) {
        auto& inv = gameHandler.getInventory();

        // Load backpack icon if needed
        if (!backpackIconTexture_ && assetMgr && assetMgr->isInitialized()) {
            auto blpData = assetMgr->readFile("Interface\\Buttons\\Button-Backpack-Up.blp");
            if (!blpData.empty()) {
                auto image = pipeline::BLPLoader::load(blpData);
                if (image.isValid()) {
                    glGenTextures(1, &backpackIconTexture_);
                    glBindTexture(GL_TEXTURE_2D, backpackIconTexture_);
                    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, image.width, image.height, 0,
                                 GL_RGBA, GL_UNSIGNED_BYTE, image.data.data());
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
                    glBindTexture(GL_TEXTURE_2D, 0);
                }
            }
        }

        // Slots 1-4: Bag slots (leftmost)
        for (int i = 0; i < 4; ++i) {
            if (i > 0) ImGui::SameLine(0, spacing);
            ImGui::PushID(i + 1);

            game::EquipSlot bagSlot = static_cast<game::EquipSlot>(static_cast<int>(game::EquipSlot::BAG1) + i);
            const auto& bagItem = inv.getEquipSlot(bagSlot);

            GLuint bagIcon = 0;
            if (!bagItem.empty() && bagItem.item.displayInfoId != 0) {
                bagIcon = inventoryScreen.getItemIcon(bagItem.item.displayInfoId);
            }

            if (bagIcon) {
                if (ImGui::ImageButton("##bag", (ImTextureID)(uintptr_t)bagIcon,
                                       ImVec2(slotSize - 4, slotSize - 4),
                                       ImVec2(0, 0), ImVec2(1, 1),
                                       ImVec4(0.1f, 0.1f, 0.1f, 0.9f),
                                       ImVec4(1, 1, 1, 1))) {
                    if (inventoryScreen.isSeparateBags())
                        inventoryScreen.toggleBag(i);
                    else
                        inventoryScreen.toggle();
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("%s", bagItem.item.name.c_str());
                }
            } else {
                // Empty bag slot
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.15f, 0.15f, 0.8f));
                if (ImGui::Button("##empty", ImVec2(slotSize, slotSize))) {
                    // Empty slot - no bag equipped
                }
                ImGui::PopStyleColor();
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Empty Bag Slot");
                }
            }

            // Accept dragged item from inventory
            if (ImGui::IsItemHovered() && inventoryScreen.isHoldingItem()) {
                const auto& heldItem = inventoryScreen.getHeldItem();
                // Check if held item is a bag (bagSlots > 0)
                if (heldItem.bagSlots > 0 && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                    // Equip the bag to inventory
                    auto& inventory = gameHandler.getInventory();
                    inventory.setEquipSlot(bagSlot, heldItem);
                    inventoryScreen.returnHeldItem(inventory);
                }
            }

            ImGui::PopID();
        }

        // Backpack (rightmost slot)
        ImGui::SameLine(0, spacing);
        ImGui::PushID(0);
        if (backpackIconTexture_) {
            if (ImGui::ImageButton("##backpack", (ImTextureID)(uintptr_t)backpackIconTexture_,
                                   ImVec2(slotSize - 4, slotSize - 4),
                                   ImVec2(0, 0), ImVec2(1, 1),
                                   ImVec4(0.1f, 0.1f, 0.1f, 0.9f),
                                   ImVec4(1, 1, 1, 1))) {
                if (inventoryScreen.isSeparateBags())
                    inventoryScreen.toggleBackpack();
                else
                    inventoryScreen.toggle();
            }
        } else {
            if (ImGui::Button("B", ImVec2(slotSize, slotSize))) {
                if (inventoryScreen.isSeparateBags())
                    inventoryScreen.toggleBackpack();
                else
                    inventoryScreen.toggle();
            }
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Backpack");
        }
        ImGui::PopID();
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

void GameScreen::renderGuildInvitePopup(game::GameHandler& gameHandler) {
    if (!gameHandler.hasPendingGuildInvite()) return;

    auto* window = core::Application::getInstance().getWindow();
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - 175, 250), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(350, 0), ImGuiCond_Always);

    if (ImGui::Begin("Guild Invite", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize)) {
        ImGui::TextWrapped("%s has invited you to join %s.",
                           gameHandler.getPendingGuildInviterName().c_str(),
                           gameHandler.getPendingGuildInviteGuildName().c_str());
        ImGui::Spacing();

        if (ImGui::Button("Accept", ImVec2(155, 30))) {
            gameHandler.acceptGuildInvite();
        }
        ImGui::SameLine();
        if (ImGui::Button("Decline", ImVec2(155, 30))) {
            gameHandler.declineGuildInvite();
        }
    }
    ImGui::End();
}

void GameScreen::renderGuildRoster(game::GameHandler& gameHandler) {
    // O key toggle (WoW default Social/Guild keybind)
    if (!ImGui::GetIO().WantCaptureKeyboard && ImGui::IsKeyPressed(ImGuiKey_O)) {
        showGuildRoster_ = !showGuildRoster_;
        if (showGuildRoster_) {
            if (!gameHandler.isInGuild()) {
                gameHandler.addLocalChatMessage(game::MessageChatData{
                    game::ChatType::SYSTEM, game::ChatLanguage::UNIVERSAL, 0, "", 0, "", "You are not in a guild.", "", 0});
                showGuildRoster_ = false;
                return;
            }
            // Re-query guild name if we have guildId but no name yet
            if (gameHandler.getGuildName().empty()) {
                const auto* ch = gameHandler.getActiveCharacter();
                if (ch && ch->hasGuild()) {
                    gameHandler.queryGuildInfo(ch->guildId);
                }
            }
            gameHandler.requestGuildRoster();
        }
    }

    if (!showGuildRoster_) return;

    auto* window = core::Application::getInstance().getWindow();
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - 300, screenH / 2 - 250), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(600, 500), ImGuiCond_Once);

    std::string title = gameHandler.isInGuild() ? (gameHandler.getGuildName() + " - Roster") : "Guild Roster";
    bool open = showGuildRoster_;
    if (ImGui::Begin(title.c_str(), &open, ImGuiWindowFlags_NoCollapse)) {
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
            ImGui::Text("%d members (%d online)", (int)roster.members.size(), onlineCount);
            ImGui::Separator();

            // Table
            if (ImGui::BeginTable("GuildRoster", 6,
                    ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                    ImGuiTableFlags_Sortable)) {
                ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_DefaultSort);
                ImGui::TableSetupColumn("Rank");
                ImGui::TableSetupColumn("Level", ImGuiTableColumnFlags_WidthFixed, 40.0f);
                ImGui::TableSetupColumn("Class", ImGuiTableColumnFlags_WidthFixed, 70.0f);
                ImGui::TableSetupColumn("Zone", ImGuiTableColumnFlags_WidthFixed, 80.0f);
                ImGui::TableSetupColumn("Note");
                ImGui::TableHeadersRow();

                // Online members first, then offline
                auto sortedMembers = roster.members;
                std::sort(sortedMembers.begin(), sortedMembers.end(), [](const auto& a, const auto& b) {
                    if (a.online != b.online) return a.online > b.online;
                    return a.name < b.name;
                });

                static const char* classNames[] = {
                    "Unknown", "Warrior", "Paladin", "Hunter", "Rogue",
                    "Priest", "Death Knight", "Shaman", "Mage", "Warlock",
                    "", "Druid"
                };

                for (const auto& m : sortedMembers) {
                    ImGui::TableNextRow();
                    ImVec4 textColor = m.online ? ImVec4(1.0f, 1.0f, 1.0f, 1.0f)
                                                : ImVec4(0.5f, 0.5f, 0.5f, 1.0f);

                    ImGui::TableNextColumn();
                    ImGui::TextColored(textColor, "%s", m.name.c_str());

                    ImGui::TableNextColumn();
                    ImGui::TextColored(textColor, "%u", m.rankIndex);

                    ImGui::TableNextColumn();
                    ImGui::TextColored(textColor, "%u", m.level);

                    ImGui::TableNextColumn();
                    const char* className = (m.classId < 12) ? classNames[m.classId] : "Unknown";
                    ImGui::TextColored(textColor, "%s", className);

                    ImGui::TableNextColumn();
                    ImGui::TextColored(textColor, "%u", m.zoneId);

                    ImGui::TableNextColumn();
                    ImGui::TextColored(textColor, "%s", m.publicNote.c_str());
                }
                ImGui::EndTable();
            }
        }
    }
    ImGui::End();
    showGuildRoster_ = open;
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

    auto* assetMgr = core::Application::getInstance().getAssetManager();

    // Position below the player frame in top-left
    constexpr float ICON_SIZE = 32.0f;
    constexpr int ICONS_PER_ROW = 8;
    float barW = ICONS_PER_ROW * (ICON_SIZE + 4.0f) + 8.0f;
    // Dock under player frame in top-left (player frame is at 10, 30 with ~110px height)
    ImGui::SetNextWindowPos(ImVec2(10.0f, 145.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(barW, 0), ImGuiCond_Always);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                             ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar;

    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(2.0f, 2.0f));

    if (ImGui::Begin("##BuffBar", nullptr, flags)) {
        int shown = 0;
        for (size_t i = 0; i < auras.size() && shown < 16; ++i) {
            const auto& aura = auras[i];
            if (aura.isEmpty()) continue;

            if (shown > 0 && shown % ICONS_PER_ROW != 0) ImGui::SameLine();

            ImGui::PushID(static_cast<int>(i));

            bool isBuff = (aura.flags & 0x02) != 0;
            ImVec4 borderColor = isBuff ? ImVec4(0.2f, 0.8f, 0.2f, 0.9f) : ImVec4(0.8f, 0.2f, 0.2f, 0.9f);

            // Try to get spell icon
            GLuint iconTex = 0;
            if (assetMgr) {
                iconTex = getSpellIcon(aura.spellId, assetMgr);
            }

            if (iconTex) {
                ImGui::PushStyleColor(ImGuiCol_Button, borderColor);
                ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(2, 2));
                ImGui::ImageButton("##aura",
                    (ImTextureID)(uintptr_t)iconTex,
                    ImVec2(ICON_SIZE - 4, ICON_SIZE - 4));
                ImGui::PopStyleVar();
                ImGui::PopStyleColor();
            } else {
                ImGui::PushStyleColor(ImGuiCol_Button, borderColor);
                char label[8];
                snprintf(label, sizeof(label), "%u", aura.spellId);
                ImGui::Button(label, ImVec2(ICON_SIZE, ICON_SIZE));
                ImGui::PopStyleColor();
            }

            // Right-click to cancel buffs / dismount
            if (ImGui::IsItemClicked(ImGuiMouseButton_Right)) {
                if (gameHandler.isMounted()) {
                    gameHandler.dismount();
                } else if (isBuff) {
                    gameHandler.cancelAura(aura.spellId);
                }
            }

            // Tooltip with spell name and duration
            if (ImGui::IsItemHovered()) {
                std::string name;
                auto it = actionSpellNames.find(aura.spellId);
                if (it != actionSpellNames.end()) {
                    name = it->second;
                } else {
                    name = "Spell #" + std::to_string(aura.spellId);
                }
                if (aura.durationMs > 0) {
                    int seconds = aura.durationMs / 1000;
                    if (seconds < 60) {
                        ImGui::SetTooltip("%s (%ds)", name.c_str(), seconds);
                    } else {
                        ImGui::SetTooltip("%s (%dm %ds)", name.c_str(), seconds / 60, seconds % 60);
                    }
                } else {
                    ImGui::SetTooltip("%s", name.c_str());
                }
            }

            ImGui::PopID();
            shown++;
        }
    }
    ImGui::End();

    ImGui::PopStyleVar();
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
                itemName = "Item #" + std::to_string(item.itemId);
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
            std::string processedText = replaceGenderPlaceholders(opt.text, gameHandler);
            char label[256];
            snprintf(label, sizeof(label), "%s %s", icon, processedText.c_str());
            if (ImGui::Selectable(label)) {
                gameHandler.selectGossipOption(opt.id);
            }
            ImGui::PopID();
        }

        // Fallback: some spirit healers don't send gossip options.
        if (gossip.options.empty() && gameHandler.isPlayerGhost()) {
            bool isSpirit = false;
            if (npcEntity && npcEntity->getType() == game::ObjectType::UNIT) {
                auto unit = std::static_pointer_cast<game::Unit>(npcEntity);
                std::string name = unit->getName();
                std::transform(name.begin(), name.end(), name.begin(),
                               [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
                if (name.find("spirit healer") != std::string::npos ||
                    name.find("spirit guide") != std::string::npos) {
                    isSpirit = true;
                }
            }
            if (isSpirit) {
                if (ImGui::Selectable("[Spiritguide] Return to Graveyard")) {
                    gameHandler.activateSpiritHealer(gossip.npcGuid);
                    gameHandler.closeGossip();
                }
            }
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
    std::string processedTitle = replaceGenderPlaceholders(quest.title, gameHandler);
    if (ImGui::Begin(processedTitle.c_str(), &open)) {
        // Quest description
        if (!quest.details.empty()) {
            std::string processedDetails = replaceGenderPlaceholders(quest.details, gameHandler);
            ImGui::TextWrapped("%s", processedDetails.c_str());
        }

        // Objectives
        if (!quest.objectives.empty()) {
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::TextColored(ImVec4(1.0f, 0.82f, 0.0f, 1.0f), "Objectives:");
            std::string processedObjectives = replaceGenderPlaceholders(quest.objectives, gameHandler);
            ImGui::TextWrapped("%s", processedObjectives.c_str());
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
// Quest Request Items Window (turn-in progress check)
// ============================================================

void GameScreen::renderQuestRequestItemsWindow(game::GameHandler& gameHandler) {
    if (!gameHandler.isQuestRequestItemsOpen()) return;

    auto* window = core::Application::getInstance().getWindow();
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - 225, screenH / 2 - 200), ImGuiCond_Appearing);
    ImGui::SetNextWindowSize(ImVec2(450, 350), ImGuiCond_Appearing);

    bool open = true;
    const auto& quest = gameHandler.getQuestRequestItems();
    std::string processedTitle = replaceGenderPlaceholders(quest.title, gameHandler);
    if (ImGui::Begin(processedTitle.c_str(), &open, ImGuiWindowFlags_NoCollapse)) {
        if (!quest.completionText.empty()) {
            std::string processedCompletionText = replaceGenderPlaceholders(quest.completionText, gameHandler);
            ImGui::TextWrapped("%s", processedCompletionText.c_str());
        }

        // Required items
        if (!quest.requiredItems.empty()) {
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::TextColored(ImVec4(1.0f, 0.82f, 0.0f, 1.0f), "Required Items:");
            for (const auto& item : quest.requiredItems) {
                auto* info = gameHandler.getItemInfo(item.itemId);
                if (info && info->valid)
                    ImGui::Text("  %s x%u", info->name.c_str(), item.count);
                else
                    ImGui::Text("  Item %u x%u", item.itemId, item.count);
            }
        }

        if (quest.requiredMoney > 0) {
            ImGui::Spacing();
            uint32_t g = quest.requiredMoney / 10000;
            uint32_t s = (quest.requiredMoney % 10000) / 100;
            uint32_t c = quest.requiredMoney % 100;
            ImGui::Text("Required money: %ug %us %uc", g, s, c);
        }

        // Complete / Cancel buttons
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();
        float buttonW = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
        if (quest.isCompletable()) {
            if (ImGui::Button("Complete Quest", ImVec2(buttonW, 0))) {
                gameHandler.completeQuest();
            }
        } else {
            ImGui::BeginDisabled();
            ImGui::Button("Incomplete", ImVec2(buttonW, 0));
            ImGui::EndDisabled();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(buttonW, 0))) {
            gameHandler.closeQuestRequestItems();
        }
    }
    ImGui::End();

    if (!open) {
        gameHandler.closeQuestRequestItems();
    }
}

// ============================================================
// Quest Offer Reward Window (choose reward)
// ============================================================

void GameScreen::renderQuestOfferRewardWindow(game::GameHandler& gameHandler) {
    if (!gameHandler.isQuestOfferRewardOpen()) return;

    auto* window = core::Application::getInstance().getWindow();
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - 225, screenH / 2 - 200), ImGuiCond_Appearing);
    ImGui::SetNextWindowSize(ImVec2(450, 400), ImGuiCond_Appearing);

    bool open = true;
    const auto& quest = gameHandler.getQuestOfferReward();
    static int selectedChoice = -1;

    // Auto-select if only one choice reward
    if (quest.choiceRewards.size() == 1 && selectedChoice == -1) {
        selectedChoice = 0;
    }

    std::string processedTitle = replaceGenderPlaceholders(quest.title, gameHandler);
    if (ImGui::Begin(processedTitle.c_str(), &open, ImGuiWindowFlags_NoCollapse)) {
        if (!quest.rewardText.empty()) {
            std::string processedRewardText = replaceGenderPlaceholders(quest.rewardText, gameHandler);
            ImGui::TextWrapped("%s", processedRewardText.c_str());
        }

        // Choice rewards (pick one)
        if (!quest.choiceRewards.empty()) {
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::TextColored(ImVec4(1.0f, 0.82f, 0.0f, 1.0f), "Choose a reward:");

            for (size_t i = 0; i < quest.choiceRewards.size(); ++i) {
                const auto& item = quest.choiceRewards[i];
                auto* info = gameHandler.getItemInfo(item.itemId);

                bool selected = (selectedChoice == static_cast<int>(i));

                // Get item icon if we have displayInfoId
                uint32_t iconTex = 0;
                if (info && info->valid && info->displayInfoId != 0) {
                    iconTex = inventoryScreen.getItemIcon(info->displayInfoId);
                }

                // Quality color
                ImVec4 qualityColor = ImVec4(1.0f, 1.0f, 1.0f, 1.0f); // White (poor)
                if (info && info->valid) {
                    switch (info->quality) {
                        case 1: qualityColor = ImVec4(1.0f, 1.0f, 1.0f, 1.0f); break; // Common (white)
                        case 2: qualityColor = ImVec4(0.0f, 1.0f, 0.0f, 1.0f); break; // Uncommon (green)
                        case 3: qualityColor = ImVec4(0.0f, 0.5f, 1.0f, 1.0f); break; // Rare (blue)
                        case 4: qualityColor = ImVec4(0.64f, 0.21f, 0.93f, 1.0f); break; // Epic (purple)
                        case 5: qualityColor = ImVec4(1.0f, 0.5f, 0.0f, 1.0f); break; // Legendary (orange)
                    }
                }

                // Render item with icon
                ImGui::PushID(static_cast<int>(i));
                if (ImGui::Selectable("##reward", selected, 0, ImVec2(0, 40))) {
                    selectedChoice = static_cast<int>(i);
                }

                // Draw icon and text over the selectable
                ImGui::SameLine();
                ImGui::SetCursorPosX(ImGui::GetCursorPosX() - ImGui::GetItemRectSize().x + 4);

                if (iconTex) {
                    ImGui::Image((void*)(intptr_t)iconTex, ImVec2(36, 36));
                    ImGui::SameLine();
                }

                ImGui::BeginGroup();
                if (info && info->valid) {
                    ImGui::TextColored(qualityColor, "%s", info->name.c_str());
                    if (item.count > 1) {
                        ImGui::SameLine();
                        ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 0.7f), "x%u", item.count);
                    }
                    // Show stats
                    if (info->armor > 0 || info->stamina > 0 || info->strength > 0 ||
                        info->agility > 0 || info->intellect > 0 || info->spirit > 0) {
                        std::string stats;
                        if (info->armor > 0) stats += std::to_string(info->armor) + " Armor ";
                        if (info->stamina > 0) stats += "+" + std::to_string(info->stamina) + " Sta ";
                        if (info->strength > 0) stats += "+" + std::to_string(info->strength) + " Str ";
                        if (info->agility > 0) stats += "+" + std::to_string(info->agility) + " Agi ";
                        if (info->intellect > 0) stats += "+" + std::to_string(info->intellect) + " Int ";
                        if (info->spirit > 0) stats += "+" + std::to_string(info->spirit) + " Spi ";
                        ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "%s", stats.c_str());
                    }
                } else {
                    ImGui::TextColored(qualityColor, "Item %u", item.itemId);
                    if (item.count > 0) {
                        ImGui::SameLine();
                        ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 0.7f), "x%u", item.count);
                    }
                }
                ImGui::EndGroup();
                ImGui::PopID();
            }
        }

        // Fixed rewards (always given)
        if (!quest.fixedRewards.empty()) {
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::TextColored(ImVec4(1.0f, 0.82f, 0.0f, 1.0f), "You will also receive:");
            for (const auto& item : quest.fixedRewards) {
                auto* info = gameHandler.getItemInfo(item.itemId);
                if (info && info->valid)
                    ImGui::Text("  %s x%u", info->name.c_str(), item.count);
                else
                    ImGui::Text("  Item %u x%u", item.itemId, item.count);
            }
        }

        // Money / XP rewards
        if (quest.rewardXp > 0 || quest.rewardMoney > 0) {
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::TextColored(ImVec4(1.0f, 0.82f, 0.0f, 1.0f), "Rewards:");
            if (quest.rewardXp > 0)
                ImGui::Text("  %u experience", quest.rewardXp);
            if (quest.rewardMoney > 0) {
                uint32_t g = quest.rewardMoney / 10000;
                uint32_t s = (quest.rewardMoney % 10000) / 100;
                uint32_t c = quest.rewardMoney % 100;
                if (g > 0) ImGui::Text("  %ug %us %uc", g, s, c);
                else if (s > 0) ImGui::Text("  %us %uc", s, c);
                else ImGui::Text("  %uc", c);
            }
        }

        // Complete button
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();
        float buttonW = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;

        bool canComplete = quest.choiceRewards.empty() || selectedChoice >= 0;
        if (!canComplete) ImGui::BeginDisabled();
        if (ImGui::Button("Complete Quest", ImVec2(buttonW, 0))) {
            uint32_t rewardIdx = quest.choiceRewards.empty() ? 0 : static_cast<uint32_t>(selectedChoice);
            gameHandler.chooseQuestReward(rewardIdx);
            selectedChoice = -1;
        }
        if (!canComplete) ImGui::EndDisabled();

        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(buttonW, 0))) {
            gameHandler.closeQuestOfferReward();
            selectedChoice = -1;
        }
    }
    ImGui::End();

    if (!open) {
        gameHandler.closeQuestOfferReward();
        selectedChoice = -1;
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
// Trainer
// ============================================================

void GameScreen::renderTrainerWindow(game::GameHandler& gameHandler) {
    if (!gameHandler.isTrainerWindowOpen()) return;

    auto* window = core::Application::getInstance().getWindow();
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - 225, 100), ImGuiCond_Appearing);
    ImGui::SetNextWindowSize(ImVec2(500, 450), ImGuiCond_Appearing);

    bool open = true;
    if (ImGui::Begin("Trainer", &open)) {
        const auto& trainer = gameHandler.getTrainerSpells();

        // NPC name
        auto npcEntity = gameHandler.getEntityManager().getEntity(trainer.trainerGuid);
        if (npcEntity && npcEntity->getType() == game::ObjectType::UNIT) {
            auto unit = std::static_pointer_cast<game::Unit>(npcEntity);
            if (!unit->getName().empty()) {
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f), "%s", unit->getName().c_str());
            }
        }

        // Greeting
        if (!trainer.greeting.empty()) {
            ImGui::TextWrapped("%s", trainer.greeting.c_str());
        }
        ImGui::Separator();

        // Player money
        uint64_t money = gameHandler.getMoneyCopper();
        uint32_t mg = static_cast<uint32_t>(money / 10000);
        uint32_t ms = static_cast<uint32_t>((money / 100) % 100);
        uint32_t mc = static_cast<uint32_t>(money % 100);
        ImGui::Text("Your money: %ug %us %uc", mg, ms, mc);

        // Filter checkbox
        static bool showUnavailable = false;
        ImGui::Checkbox("Show unavailable spells", &showUnavailable);
        ImGui::Separator();

        if (trainer.spells.empty()) {
            ImGui::TextDisabled("This trainer has nothing to teach you.");
        } else {
            // Known spells for checking
            const auto& knownSpells = gameHandler.getKnownSpells();
            auto isKnown = [&](uint32_t id) {
                if (id == 0) return true;
                // Check if spell is in knownSpells list
                bool found = std::find(knownSpells.begin(), knownSpells.end(), id) != knownSpells.end();
                if (found) return true;

                // Also check if spell is in trainer list with state=2 (explicitly known)
                // state=0 means unavailable (could be no prereqs, wrong level, etc.) - don't count as known
                for (const auto& ts : trainer.spells) {
                    if (ts.spellId == id && ts.state == 2) {
                        return true;
                    }
                }
                return false;
            };
            uint32_t playerLevel = gameHandler.getPlayerLevel();

            // Renders spell rows into the current table
            auto renderSpellRows = [&](const std::vector<const game::TrainerSpell*>& spells) {
                for (const auto* spell : spells) {
                    // Check prerequisites client-side first
                    bool prereq1Met = isKnown(spell->chainNode1);
                    bool prereq2Met = isKnown(spell->chainNode2);
                    bool prereq3Met = isKnown(spell->chainNode3);
                    bool prereqsMet = prereq1Met && prereq2Met && prereq3Met;
                    bool levelMet = (spell->reqLevel == 0 || playerLevel >= spell->reqLevel);
                    bool alreadyKnown = isKnown(spell->spellId);

                    // Dynamically determine effective state based on current prerequisites
                    // Server sends state, but we override if prerequisites are now met
                    uint8_t effectiveState = spell->state;
                    if (spell->state == 1 && prereqsMet && levelMet) {
                        // Server said unavailable, but we now meet all requirements
                        effectiveState = 0;  // Treat as available
                    }

                    // Filter: skip unavailable spells if checkbox is unchecked
                    // Use effectiveState so spells with newly met prereqs aren't filtered
                    if (!showUnavailable && effectiveState == 1) {
                        continue;
                    }

                    ImGui::TableNextRow();
                    ImGui::PushID(static_cast<int>(spell->spellId));

                    ImVec4 color;
                    const char* statusLabel;
                    // WotLK trainer states: 0=available, 1=unavailable, 2=known
                    if (effectiveState == 2 || alreadyKnown) {
                        color = ImVec4(0.3f, 0.9f, 0.3f, 1.0f);
                        statusLabel = "Known";
                    } else if (effectiveState == 0) {
                        color = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
                        statusLabel = "Available";
                    } else {
                        color = ImVec4(0.6f, 0.3f, 0.3f, 1.0f);
                        statusLabel = "Unavailable";
                    }

                    // Spell name
                    ImGui::TableSetColumnIndex(0);
                    const std::string& name = gameHandler.getSpellName(spell->spellId);
                    const std::string& rank = gameHandler.getSpellRank(spell->spellId);
                    if (!name.empty()) {
                        if (!rank.empty())
                            ImGui::TextColored(color, "%s (%s)", name.c_str(), rank.c_str());
                        else
                            ImGui::TextColored(color, "%s", name.c_str());
                    } else {
                        ImGui::TextColored(color, "Spell #%u", spell->spellId);
                    }

                    if (ImGui::IsItemHovered()) {
                        ImGui::BeginTooltip();
                        if (!name.empty()) {
                            ImGui::Text("%s", name.c_str());
                            if (!rank.empty()) ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "%s", rank.c_str());
                        }
                        ImGui::Text("Status: %s", statusLabel);
                        if (spell->reqLevel > 0) {
                            ImVec4 lvlColor = levelMet ? ImVec4(0.7f, 0.7f, 0.7f, 1.0f) : ImVec4(1.0f, 0.3f, 0.3f, 1.0f);
                            ImGui::TextColored(lvlColor, "Required Level: %u", spell->reqLevel);
                        }
                        if (spell->reqSkill > 0) ImGui::Text("Required Skill: %u (value %u)", spell->reqSkill, spell->reqSkillValue);
                        auto showPrereq = [&](uint32_t node) {
                            if (node == 0) return;
                            bool met = isKnown(node);
                            const std::string& pname = gameHandler.getSpellName(node);
                            ImVec4 pcolor = met ? ImVec4(0.3f, 0.9f, 0.3f, 1.0f) : ImVec4(1.0f, 0.3f, 0.3f, 1.0f);
                            if (!pname.empty())
                                ImGui::TextColored(pcolor, "Requires: %s%s", pname.c_str(), met ? " (known)" : "");
                            else
                                ImGui::TextColored(pcolor, "Requires: Spell #%u%s", node, met ? " (known)" : "");
                        };
                        showPrereq(spell->chainNode1);
                        showPrereq(spell->chainNode2);
                        showPrereq(spell->chainNode3);
                        ImGui::EndTooltip();
                    }

                    // Level
                    ImGui::TableSetColumnIndex(1);
                    ImGui::TextColored(color, "%u", spell->reqLevel);

                    // Cost
                    ImGui::TableSetColumnIndex(2);
                    if (spell->spellCost > 0) {
                        uint32_t g = spell->spellCost / 10000;
                        uint32_t s = (spell->spellCost / 100) % 100;
                        uint32_t c = spell->spellCost % 100;
                        bool canAfford = money >= spell->spellCost;
                        ImVec4 costColor = canAfford ? color : ImVec4(1.0f, 0.3f, 0.3f, 1.0f);
                        ImGui::TextColored(costColor, "%ug %us %uc", g, s, c);
                    } else {
                        ImGui::TextColored(color, "Free");
                    }

                    // Train button - only enabled if available, affordable, prereqs met
                    ImGui::TableSetColumnIndex(3);
                    // Use effectiveState so newly available spells (after learning prereqs) can be trained
                    bool canTrain = !alreadyKnown && effectiveState == 0
                                  && prereqsMet && levelMet
                                  && (money >= spell->spellCost);

                    // Debug logging for first 3 spells to see why buttons are disabled
                    static int logCount = 0;
                    static uint64_t lastTrainerGuid = 0;
                    if (trainer.trainerGuid != lastTrainerGuid) {
                        logCount = 0;
                        lastTrainerGuid = trainer.trainerGuid;
                    }
                    if (logCount < 3) {
                        LOG_INFO("Trainer button debug: spellId=", spell->spellId,
                                " alreadyKnown=", alreadyKnown, " state=", (int)spell->state,
                                " prereqsMet=", prereqsMet, " (", prereq1Met, ",", prereq2Met, ",", prereq3Met, ")",
                                " levelMet=", levelMet,
                                " reqLevel=", spell->reqLevel, " playerLevel=", playerLevel,
                                " chain1=", spell->chainNode1, " chain2=", spell->chainNode2, " chain3=", spell->chainNode3,
                                " canAfford=", (money >= spell->spellCost),
                                " canTrain=", canTrain);
                        logCount++;
                    }

                    if (!canTrain) ImGui::BeginDisabled();
                    if (ImGui::SmallButton("Train")) {
                        gameHandler.trainSpell(spell->spellId);
                    }
                    if (!canTrain) ImGui::EndDisabled();

                    ImGui::PopID();
                }
            };

            auto renderSpellTable = [&](const char* tableId, const std::vector<const game::TrainerSpell*>& spells) {
                if (ImGui::BeginTable(tableId, 4,
                        ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY)) {
                    ImGui::TableSetupColumn("Spell", ImGuiTableColumnFlags_WidthStretch);
                    ImGui::TableSetupColumn("Level", ImGuiTableColumnFlags_WidthFixed, 40.0f);
                    ImGui::TableSetupColumn("Cost", ImGuiTableColumnFlags_WidthFixed, 120.0f);
                    ImGui::TableSetupColumn("##action", ImGuiTableColumnFlags_WidthFixed, 55.0f);
                    ImGui::TableHeadersRow();
                    renderSpellRows(spells);
                    ImGui::EndTable();
                }
            };

            const auto& tabs = gameHandler.getTrainerTabs();
            if (tabs.size() > 1) {
                // Multiple tabs - show tab bar
                if (ImGui::BeginTabBar("TrainerTabs")) {
                    for (size_t i = 0; i < tabs.size(); i++) {
                        char tabLabel[64];
                        snprintf(tabLabel, sizeof(tabLabel), "%s (%zu)",
                            tabs[i].name.c_str(), tabs[i].spells.size());

                        if (ImGui::BeginTabItem(tabLabel)) {
                            char tableId[32];
                            snprintf(tableId, sizeof(tableId), "TT%zu", i);
                            renderSpellTable(tableId, tabs[i].spells);
                            ImGui::EndTabItem();
                        }
                    }
                    ImGui::EndTabBar();
                }
            } else {
                // Single tab or no categorization - flat list
                std::vector<const game::TrainerSpell*> allSpells;
                for (const auto& spell : trainer.spells) {
                    allSpells.push_back(&spell);
                }
                renderSpellTable("TrainerTable", allSpells);
            }
        }
    }
    ImGui::End();

    if (!open) {
        gameHandler.closeTrainer();
    }
}

// ============================================================
// Teleporter Panel
// ============================================================

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
            showEscapeMenu = false;
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
// Taxi Window
// ============================================================

void GameScreen::renderTaxiWindow(game::GameHandler& gameHandler) {
    if (!gameHandler.isTaxiWindowOpen()) return;

    auto* window = core::Application::getInstance().getWindow();
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;

    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - 200, 150), ImGuiCond_Appearing);
    ImGui::SetNextWindowSize(ImVec2(400, 0), ImGuiCond_Always);

    bool open = true;
    if (ImGui::Begin("Flight Master", &open, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize)) {
        const auto& taxiData = gameHandler.getTaxiData();
        const auto& nodes = gameHandler.getTaxiNodes();
        uint32_t currentNode = gameHandler.getTaxiCurrentNode();

        // Get current node's map to filter destinations
        uint32_t currentMapId = 0;
        auto curIt = nodes.find(currentNode);
        if (curIt != nodes.end()) {
            currentMapId = curIt->second.mapId;
            ImGui::TextColored(ImVec4(0.5f, 1.0f, 0.5f, 1.0f), "Current: %s", curIt->second.name.c_str());
            ImGui::Separator();
        }

        ImGui::Text("Select a destination:");
        ImGui::Spacing();

        static uint32_t selectedNodeId = 0;
        int destCount = 0;
        if (ImGui::BeginTable("TaxiNodes", 3, ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg)) {
            ImGui::TableSetupColumn("Destination", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("Cost", ImGuiTableColumnFlags_WidthFixed, 120.0f);
            ImGui::TableSetupColumn("Action", ImGuiTableColumnFlags_WidthFixed, 60.0f);
            ImGui::TableHeadersRow();

            for (const auto& [nodeId, node] : nodes) {
                if (nodeId == currentNode) continue;
                if (node.mapId != currentMapId) continue;
                if (!taxiData.isNodeKnown(nodeId)) continue;

                uint32_t costCopper = gameHandler.getTaxiCostTo(nodeId);
                uint32_t gold = costCopper / 10000;
                uint32_t silver = (costCopper / 100) % 100;
                uint32_t copper = costCopper % 100;

                ImGui::PushID(static_cast<int>(nodeId));
                ImGui::TableNextRow();

                ImGui::TableSetColumnIndex(0);
                bool isSelected = (selectedNodeId == nodeId);
                if (ImGui::Selectable(node.name.c_str(), isSelected,
                                      ImGuiSelectableFlags_SpanAllColumns |
                                      ImGuiSelectableFlags_AllowDoubleClick)) {
                    selectedNodeId = nodeId;
                    LOG_INFO("Taxi UI: Selected dest=", nodeId);
                    if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                        LOG_INFO("Taxi UI: Double-click activate dest=", nodeId);
                        gameHandler.activateTaxi(nodeId);
                    }
                }

                ImGui::TableSetColumnIndex(1);
                if (gold > 0) {
                    ImGui::TextColored(ImVec4(0.9f, 0.8f, 0.3f, 1.0f), "%ug %us %uc", gold, silver, copper);
                } else if (silver > 0) {
                    ImGui::TextColored(ImVec4(0.75f, 0.75f, 0.75f, 1.0f), "%us %uc", silver, copper);
                } else {
                    ImGui::TextColored(ImVec4(0.72f, 0.45f, 0.2f, 1.0f), "%uc", copper);
                }

                ImGui::TableSetColumnIndex(2);
                if (ImGui::SmallButton("Fly")) {
                    selectedNodeId = nodeId;
                    LOG_INFO("Taxi UI: Fly clicked dest=", nodeId);
                    gameHandler.activateTaxi(nodeId);
                }

                ImGui::PopID();
                destCount++;
            }
            ImGui::EndTable();
        }

        if (destCount == 0) {
            ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "No destinations available.");
        }

        ImGui::Spacing();
        ImGui::Separator();
        if (selectedNodeId != 0 && ImGui::Button("Fly Selected", ImVec2(-1, 0))) {
            LOG_INFO("Taxi UI: Fly Selected dest=", selectedNodeId);
            gameHandler.activateTaxi(selectedNodeId);
        }
        if (ImGui::Button("Close", ImVec2(-1, 0))) {
            gameHandler.closeTaxi();
        }
    }
    ImGui::End();

    if (!open) {
        gameHandler.closeTaxi();
    }
}

// ============================================================
// Death Screen
// ============================================================

void GameScreen::renderDeathScreen(game::GameHandler& gameHandler) {
    if (!gameHandler.showDeathDialog()) return;

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

void GameScreen::renderResurrectDialog(game::GameHandler& gameHandler) {
    if (!gameHandler.showResurrectDialog()) return;

    auto* window = core::Application::getInstance().getWindow();
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    float dlgW = 300.0f;
    float dlgH = 110.0f;
    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - dlgW / 2, screenH * 0.3f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(dlgW, dlgH), ImGuiCond_Always);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 8.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.1f, 0.15f, 0.95f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.4f, 0.4f, 0.8f, 1.0f));

    if (ImGui::Begin("##ResurrectDialog", nullptr,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar)) {

        ImGui::Spacing();
        const char* text = "Return to life?";
        float textW = ImGui::CalcTextSize(text).x;
        ImGui::SetCursorPosX((dlgW - textW) / 2);
        ImGui::TextColored(ImVec4(0.8f, 0.9f, 1.0f, 1.0f), "%s", text);

        ImGui::Spacing();
        ImGui::Spacing();

        float btnW = 100.0f;
        float spacing = 20.0f;
        ImGui::SetCursorPosX((dlgW - btnW * 2 - spacing) / 2);

        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.5f, 0.2f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.7f, 0.3f, 1.0f));
        if (ImGui::Button("Accept", ImVec2(btnW, 30))) {
            gameHandler.acceptResurrect();
        }
        ImGui::PopStyleColor(2);

        ImGui::SameLine(0, spacing);

        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.5f, 0.2f, 0.2f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.7f, 0.3f, 0.3f, 1.0f));
        if (ImGui::Button("Decline", ImVec2(btnW, 30))) {
            gameHandler.declineResurrect();
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
    constexpr bool kDefaultShadows = false;
    constexpr int kDefaultMusicVolume = 30;
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
            // Load volumes from all audio managers
            if (auto* music = renderer->getMusicManager()) {
                pendingMusicVolume = music->getVolume();
            }
            if (auto* ambient = renderer->getAmbientSoundManager()) {
                pendingAmbientVolume = static_cast<int>(ambient->getVolumeScale() * 100.0f + 0.5f);
            }
            if (auto* ui = renderer->getUiSoundManager()) {
                pendingUiVolume = static_cast<int>(ui->getVolumeScale() * 100.0f + 0.5f);
            }
            if (auto* combat = renderer->getCombatSoundManager()) {
                pendingCombatVolume = static_cast<int>(combat->getVolumeScale() * 100.0f + 0.5f);
            }
            if (auto* spell = renderer->getSpellSoundManager()) {
                pendingSpellVolume = static_cast<int>(spell->getVolumeScale() * 100.0f + 0.5f);
            }
            if (auto* movement = renderer->getMovementSoundManager()) {
                pendingMovementVolume = static_cast<int>(movement->getVolumeScale() * 100.0f + 0.5f);
            }
            if (auto* footstep = renderer->getFootstepManager()) {
                pendingFootstepVolume = static_cast<int>(footstep->getVolumeScale() * 100.0f + 0.5f);
            }
            if (auto* npcVoice = renderer->getNpcVoiceManager()) {
                pendingNpcVoiceVolume = static_cast<int>(npcVoice->getVolumeScale() * 100.0f + 0.5f);
            }
            if (auto* mount = renderer->getMountSoundManager()) {
                pendingMountVolume = static_cast<int>(mount->getVolumeScale() * 100.0f + 0.5f);
            }
            if (auto* activity = renderer->getActivitySoundManager()) {
                pendingActivityVolume = static_cast<int>(activity->getVolumeScale() * 100.0f + 0.5f);
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
        pendingUiOpacity = static_cast<int>(uiOpacity_ * 100.0f + 0.5f);
        pendingMinimapRotate = minimapRotate_;
        pendingMinimapSquare = minimapSquare_;
        if (renderer) {
            if (auto* minimap = renderer->getMinimap()) {
                minimap->setRotateWithCamera(minimapRotate_);
                minimap->setSquareShape(minimapSquare_);
            }
        }
        settingsInit = true;
    }

    ImGuiIO& io = ImGui::GetIO();
    float screenW = io.DisplaySize.x;
    float screenH = io.DisplaySize.y;
    ImVec2 size(520.0f, std::min(screenH * 0.9f, 720.0f));
    ImVec2 pos((screenW - size.x) * 0.5f, (screenH - size.y) * 0.5f);

    ImGui::SetNextWindowPos(pos, ImGuiCond_Always);
    ImGui::SetNextWindowSize(size, ImGuiCond_Always);
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar;

    if (ImGui::Begin("##SettingsWindow", nullptr, flags)) {
        ImGui::Text("Settings");
        ImGui::Separator();

        if (ImGui::BeginTabBar("SettingsTabs", ImGuiTabBarFlags_None)) {
            // ============================================================
            // VIDEO TAB
            // ============================================================
            if (ImGui::BeginTabItem("Video")) {
                ImGui::Spacing();

                if (ImGui::Checkbox("Fullscreen", &pendingFullscreen)) {
                    window->setFullscreen(pendingFullscreen);
                    saveSettings();
                }
                if (ImGui::Checkbox("VSync", &pendingVsync)) {
                    window->setVsync(pendingVsync);
                    saveSettings();
                }
                if (ImGui::Checkbox("Shadows", &pendingShadows)) {
                    if (renderer) renderer->setShadowsEnabled(pendingShadows);
                    saveSettings();
                }

                const char* resLabel = "Resolution";
                const char* resItems[kResCount];
                char resBuf[kResCount][16];
                for (int i = 0; i < kResCount; i++) {
                    snprintf(resBuf[i], sizeof(resBuf[i]), "%dx%d", kResolutions[i][0], kResolutions[i][1]);
                    resItems[i] = resBuf[i];
                }
                if (ImGui::Combo(resLabel, &pendingResIndex, resItems, kResCount)) {
                    window->applyResolution(kResolutions[pendingResIndex][0], kResolutions[pendingResIndex][1]);
                    saveSettings();
                }

                ImGui::Spacing();
                ImGui::Separator();
                ImGui::Spacing();

                if (ImGui::Button("Restore Video Defaults", ImVec2(-1, 0))) {
                    pendingFullscreen = kDefaultFullscreen;
                    pendingVsync = kDefaultVsync;
                    pendingShadows = kDefaultShadows;
                    pendingResIndex = defaultResIndex;
                    window->setFullscreen(pendingFullscreen);
                    window->setVsync(pendingVsync);
                    window->applyResolution(kResolutions[pendingResIndex][0], kResolutions[pendingResIndex][1]);
                    if (renderer) renderer->setShadowsEnabled(pendingShadows);
                    saveSettings();
                }

                ImGui::EndTabItem();
            }

            // ============================================================
            // AUDIO TAB
            // ============================================================
            if (ImGui::BeginTabItem("Audio")) {
                ImGui::Spacing();
                ImGui::BeginChild("AudioSettings", ImVec2(0, 360), true);

                // Helper lambda to apply audio settings
                auto applyAudioSettings = [&]() {
                    if (!renderer) return;
                    float masterScale = static_cast<float>(pendingMasterVolume) / 100.0f;
                    if (auto* music = renderer->getMusicManager()) {
                        music->setVolume(static_cast<int>(pendingMusicVolume * masterScale));
                    }
                    if (auto* ambient = renderer->getAmbientSoundManager()) {
                        ambient->setVolumeScale(pendingAmbientVolume / 100.0f * masterScale);
                    }
                    if (auto* ui = renderer->getUiSoundManager()) {
                        ui->setVolumeScale(pendingUiVolume / 100.0f * masterScale);
                    }
                    if (auto* combat = renderer->getCombatSoundManager()) {
                        combat->setVolumeScale(pendingCombatVolume / 100.0f * masterScale);
                    }
                    if (auto* spell = renderer->getSpellSoundManager()) {
                        spell->setVolumeScale(pendingSpellVolume / 100.0f * masterScale);
                    }
                    if (auto* movement = renderer->getMovementSoundManager()) {
                        movement->setVolumeScale(pendingMovementVolume / 100.0f * masterScale);
                    }
                    if (auto* footstep = renderer->getFootstepManager()) {
                        footstep->setVolumeScale(pendingFootstepVolume / 100.0f * masterScale);
                    }
                    if (auto* npcVoice = renderer->getNpcVoiceManager()) {
                        npcVoice->setVolumeScale(pendingNpcVoiceVolume / 100.0f * masterScale);
                    }
                    if (auto* mount = renderer->getMountSoundManager()) {
                        mount->setVolumeScale(pendingMountVolume / 100.0f * masterScale);
                    }
                    if (auto* activity = renderer->getActivitySoundManager()) {
                        activity->setVolumeScale(pendingActivityVolume / 100.0f * masterScale);
                    }
                    saveSettings();
                };

                ImGui::Text("Master Volume");
                if (ImGui::SliderInt("##MasterVolume", &pendingMasterVolume, 0, 100, "%d%%")) {
                    applyAudioSettings();
                }
                ImGui::Separator();

                ImGui::Text("Music");
                if (ImGui::SliderInt("##MusicVolume", &pendingMusicVolume, 0, 100, "%d%%")) {
                    applyAudioSettings();
                }

                ImGui::Spacing();
                ImGui::Text("Ambient Sounds");
                if (ImGui::SliderInt("##AmbientVolume", &pendingAmbientVolume, 0, 100, "%d%%")) {
                    applyAudioSettings();
                }
                ImGui::TextWrapped("Weather, zones, cities, emitters");

                ImGui::Spacing();
                ImGui::Text("UI Sounds");
                if (ImGui::SliderInt("##UiVolume", &pendingUiVolume, 0, 100, "%d%%")) {
                    applyAudioSettings();
                }
                ImGui::TextWrapped("Buttons, loot, quest complete");

                ImGui::Spacing();
                ImGui::Text("Combat Sounds");
                if (ImGui::SliderInt("##CombatVolume", &pendingCombatVolume, 0, 100, "%d%%")) {
                    applyAudioSettings();
                }
                ImGui::TextWrapped("Weapon swings, impacts, grunts");

                ImGui::Spacing();
                ImGui::Text("Spell Sounds");
                if (ImGui::SliderInt("##SpellVolume", &pendingSpellVolume, 0, 100, "%d%%")) {
                    applyAudioSettings();
                }
                ImGui::TextWrapped("Magic casting and impacts");

                ImGui::Spacing();
                ImGui::Text("Movement Sounds");
                if (ImGui::SliderInt("##MovementVolume", &pendingMovementVolume, 0, 100, "%d%%")) {
                    applyAudioSettings();
                }
                ImGui::TextWrapped("Water splashes, jump/land");

                ImGui::Spacing();
                ImGui::Text("Footsteps");
                if (ImGui::SliderInt("##FootstepVolume", &pendingFootstepVolume, 0, 100, "%d%%")) {
                    applyAudioSettings();
                }

                ImGui::Spacing();
                ImGui::Text("NPC Voices");
                if (ImGui::SliderInt("##NpcVoiceVolume", &pendingNpcVoiceVolume, 0, 100, "%d%%")) {
                    applyAudioSettings();
                }

                ImGui::Spacing();
                ImGui::Text("Mount Sounds");
                if (ImGui::SliderInt("##MountVolume", &pendingMountVolume, 0, 100, "%d%%")) {
                    applyAudioSettings();
                }

                ImGui::Spacing();
                ImGui::Text("Activity Sounds");
                if (ImGui::SliderInt("##ActivityVolume", &pendingActivityVolume, 0, 100, "%d%%")) {
                    applyAudioSettings();
                }
                ImGui::TextWrapped("Swimming, eating, drinking");

                ImGui::EndChild();

                if (ImGui::Button("Restore Audio Defaults", ImVec2(-1, 0))) {
                    pendingMasterVolume = 100;
                    pendingMusicVolume = kDefaultMusicVolume;
                    pendingAmbientVolume = 100;
                    pendingUiVolume = 100;
                    pendingCombatVolume = 100;
                    pendingSpellVolume = 100;
                    pendingMovementVolume = 100;
                    pendingFootstepVolume = 100;
                    pendingNpcVoiceVolume = 100;
                    pendingMountVolume = 100;
                    pendingActivityVolume = 100;
                    applyAudioSettings();
                }

                ImGui::EndTabItem();
            }

            // ============================================================
            // GAMEPLAY TAB
            // ============================================================
            if (ImGui::BeginTabItem("Gameplay")) {
                ImGui::Spacing();

                ImGui::Text("Controls");
                ImGui::Separator();
                if (ImGui::SliderFloat("Mouse Sensitivity", &pendingMouseSensitivity, 0.05f, 1.0f, "%.2f")) {
                    if (renderer) {
                        if (auto* cameraController = renderer->getCameraController()) {
                            cameraController->setMouseSensitivity(pendingMouseSensitivity);
                        }
                    }
                    saveSettings();
                }
                if (ImGui::Checkbox("Invert Mouse", &pendingInvertMouse)) {
                    if (renderer) {
                        if (auto* cameraController = renderer->getCameraController()) {
                            cameraController->setInvertMouse(pendingInvertMouse);
                        }
                    }
                    saveSettings();
                }

                ImGui::Spacing();
                ImGui::Spacing();

                ImGui::Text("Interface");
                ImGui::Separator();
                if (ImGui::SliderInt("UI Opacity", &pendingUiOpacity, 20, 100, "%d%%")) {
                    uiOpacity_ = static_cast<float>(pendingUiOpacity) / 100.0f;
                    saveSettings();
                }
                if (ImGui::Checkbox("Rotate Minimap", &pendingMinimapRotate)) {
                    // Force north-up minimap.
                    minimapRotate_ = false;
                    pendingMinimapRotate = false;
                    if (renderer) {
                        if (auto* minimap = renderer->getMinimap()) {
                            minimap->setRotateWithCamera(false);
                        }
                    }
                    saveSettings();
                }
                if (ImGui::Checkbox("Square Minimap", &pendingMinimapSquare)) {
                    minimapSquare_ = pendingMinimapSquare;
                    if (renderer) {
                        if (auto* minimap = renderer->getMinimap()) {
                            minimap->setSquareShape(minimapSquare_);
                        }
                    }
                    saveSettings();
                }
                // Zoom controls
                ImGui::Text("Minimap Zoom:");
                ImGui::SameLine();
                if (ImGui::Button("  -  ")) {
                    if (renderer) {
                        if (auto* minimap = renderer->getMinimap()) {
                            minimap->zoomOut();
                            saveSettings();
                        }
                    }
                }
                ImGui::SameLine();
                if (ImGui::Button("  +  ")) {
                    if (renderer) {
                        if (auto* minimap = renderer->getMinimap()) {
                            minimap->zoomIn();
                            saveSettings();
                        }
                    }
                }

                ImGui::Spacing();
                ImGui::Text("Bags");
                ImGui::Separator();
                if (ImGui::Checkbox("Separate Bag Windows", &pendingSeparateBags)) {
                    inventoryScreen.setSeparateBags(pendingSeparateBags);
                    saveSettings();
                }

                ImGui::Spacing();
                ImGui::Separator();
                ImGui::Spacing();

                if (ImGui::Button("Restore Gameplay Defaults", ImVec2(-1, 0))) {
                    pendingMouseSensitivity = kDefaultMouseSensitivity;
                    pendingInvertMouse = kDefaultInvertMouse;
                    pendingUiOpacity = 65;
                    pendingMinimapRotate = false;
                    pendingMinimapSquare = false;
                    pendingSeparateBags = true;
                    inventoryScreen.setSeparateBags(true);
                    uiOpacity_ = 0.65f;
                    minimapRotate_ = false;
                    minimapSquare_ = false;
                    if (renderer) {
                        if (auto* cameraController = renderer->getCameraController()) {
                            cameraController->setMouseSensitivity(pendingMouseSensitivity);
                            cameraController->setInvertMouse(pendingInvertMouse);
                        }
                        if (auto* minimap = renderer->getMinimap()) {
                            minimap->setRotateWithCamera(minimapRotate_);
                            minimap->setSquareShape(minimapSquare_);
                        }
                    }
                    saveSettings();
                }

                ImGui::EndTabItem();
            }

            // ============================================================
            // HD TEXTURES TAB
            // ============================================================
            if (ImGui::BeginTabItem("HD Textures")) {
                ImGui::Spacing();

                auto& app = core::Application::getInstance();
                auto* hdMgr = app.getHDPackManager();

                if (hdMgr) {
                    const auto& packs = hdMgr->getAllPacks();
                    if (packs.empty()) {
                        ImGui::TextWrapped("No HD texture packs found.");
                        ImGui::Spacing();
                        ImGui::TextWrapped("Place packs in Data/hd/<pack_name>/ with a pack.json and manifest.json.");
                    } else {
                        ImGui::Text("Available HD Texture Packs:");
                        ImGui::Spacing();

                        bool changed = false;
                        for (const auto& pack : packs) {
                            bool enabled = pack.enabled;
                            if (ImGui::Checkbox(pack.name.c_str(), &enabled)) {
                                hdMgr->setPackEnabled(pack.id, enabled);
                                changed = true;
                            }
                            ImGui::SameLine(0, 10);
                            ImGui::TextDisabled("(%u MB)", pack.totalSizeMB);
                            if (!pack.group.empty()) {
                                ImGui::SameLine(0, 10);
                                ImGui::TextDisabled("[%s]", pack.group.c_str());
                            }
                            if (!pack.expansions.empty()) {
                                std::string expList;
                                for (const auto& e : pack.expansions) {
                                    if (!expList.empty()) expList += ", ";
                                    expList += e;
                                }
                                ImGui::TextDisabled("  Compatible: %s", expList.c_str());
                            }
                        }

                        if (changed) {
                            ImGui::Spacing();
                            ImGui::Separator();
                            ImGui::Spacing();
                            if (ImGui::Button("Apply HD Packs", ImVec2(-1, 0))) {
                                std::string expansionId = "wotlk";
                                if (app.getExpansionRegistry() && app.getExpansionRegistry()->getActive()) {
                                    expansionId = app.getExpansionRegistry()->getActive()->id;
                                }
                                hdMgr->applyToAssetManager(app.getAssetManager(), expansionId);

                                // Save settings
                                std::string settingsDir;
                                const char* xdg = std::getenv("XDG_DATA_HOME");
                                if (xdg && *xdg) {
                                    settingsDir = std::string(xdg) + "/wowee";
                                } else {
                                    const char* home = std::getenv("HOME");
                                    settingsDir = std::string(home ? home : ".") + "/.local/share/wowee";
                                }
                                hdMgr->saveSettings(settingsDir + "/settings.cfg");
                            }
                        }
                    }
                } else {
                    ImGui::Text("HD Pack Manager not available.");
                }

                ImGui::EndTabItem();
            }

            ImGui::EndTabBar();
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
    float bearing = 0.0f;
    float cosB = 1.0f;
    float sinB = 0.0f;
    if (minimap->isRotateWithCamera()) {
        glm::vec3 fwd = camera->getForward();
        bearing = std::atan2(-fwd.x, fwd.y);
        cosB = std::cos(bearing);
        sinB = std::sin(bearing);
    }

    if (statuses.empty()) return;

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

    // Add zoom buttons at the bottom edge of the minimap
    ImGui::SetNextWindowPos(ImVec2(centerX - 30, centerY + mapRadius - 30), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(60, 24), ImGuiCond_Always);
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
}

std::string GameScreen::getSettingsPath() {
    std::string dir;
#ifdef _WIN32
    const char* appdata = std::getenv("APPDATA");
    dir = appdata ? std::string(appdata) + "\\wowee" : ".";
#else
    const char* home = std::getenv("HOME");
    dir = home ? std::string(home) + "/.wowee" : ".";
#endif
    return dir + "/settings.cfg";
}

std::string GameScreen::replaceGenderPlaceholders(const std::string& text, game::GameHandler& gameHandler) {
    // Get player gender, pronouns, and name
    game::Gender gender = game::Gender::NONBINARY;
    std::string playerName = "Adventurer";
    const auto* character = gameHandler.getActiveCharacter();
    if (character) {
        gender = character->gender;
        if (!character->name.empty()) {
            playerName = character->name;
        }
    }
    game::Pronouns pronouns = game::Pronouns::forGender(gender);

    std::string result = text;

    // Helper to trim whitespace
    auto trim = [](std::string& s) {
        s.erase(0, s.find_first_not_of(" \t\n\r"));
        s.erase(s.find_last_not_of(" \t\n\r") + 1);
    };

    // Replace simple placeholders first
    // $n = player name
    // $p = subject pronoun (he/she/they)
    // $o = object pronoun (him/her/them)
    // $s = possessive adjective (his/her/their)
    // $S = possessive pronoun (his/hers/theirs)
    size_t pos = 0;
    while ((pos = result.find('$', pos)) != std::string::npos) {
        if (pos + 1 >= result.length()) break;

        char code = result[pos + 1];
        std::string replacement;

        switch (code) {
            case 'n': replacement = playerName; break;
            case 'p': replacement = pronouns.subject; break;
            case 'o': replacement = pronouns.object; break;
            case 's': replacement = pronouns.possessive; break;
            case 'S': replacement = pronouns.possessiveP; break;
            case 'g':
                // Handle $g separately below
                pos++;
                continue;
            default:
                pos++;
                continue;
        }

        // Replace the placeholder
        result.replace(pos, 2, replacement);
        pos += replacement.length();
    }

    // Find and replace all $g placeholders (gender-specific text)
    // Format: $g<male>:<female>; or $g<male>:<female>:<nonbinary>;
    pos = 0;
    while ((pos = result.find("$g", pos)) != std::string::npos) {
        size_t endPos = result.find(';', pos);
        if (endPos == std::string::npos) break;

        std::string placeholder = result.substr(pos + 2, endPos - pos - 2);

        // Split by colons
        std::vector<std::string> parts;
        size_t start = 0;
        size_t colonPos;
        while ((colonPos = placeholder.find(':', start)) != std::string::npos) {
            std::string part = placeholder.substr(start, colonPos - start);
            trim(part);
            parts.push_back(part);
            start = colonPos + 1;
        }
        // Add the last part
        std::string lastPart = placeholder.substr(start);
        trim(lastPart);
        parts.push_back(lastPart);

        // Select appropriate text based on gender
        std::string replacement;
        if (parts.size() >= 3) {
            // Three options: male, female, nonbinary
            switch (gender) {
                case game::Gender::MALE:
                    replacement = parts[0];
                    break;
                case game::Gender::FEMALE:
                    replacement = parts[1];
                    break;
                case game::Gender::NONBINARY:
                    replacement = parts[2];
                    break;
            }
        } else if (parts.size() >= 2) {
            // Two options: male, female (use first for nonbinary)
            switch (gender) {
                case game::Gender::MALE:
                    replacement = parts[0];
                    break;
                case game::Gender::FEMALE:
                    replacement = parts[1];
                    break;
                case game::Gender::NONBINARY:
                    // Default to gender-neutral: use the shorter/simpler option
                    replacement = parts[0].length() <= parts[1].length() ? parts[0] : parts[1];
                    break;
            }
        } else {
            // Malformed placeholder
            pos = endPos + 1;
            continue;
        }

        result.replace(pos, endPos - pos + 1, replacement);
        pos += replacement.length();
    }

    return result;
}

void GameScreen::renderChatBubbles(game::GameHandler& gameHandler) {
    if (chatBubbles_.empty()) return;

    auto* renderer = core::Application::getInstance().getRenderer();
    auto* camera = renderer ? renderer->getCamera() : nullptr;
    if (!camera) return;

    auto* window = core::Application::getInstance().getWindow();
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    // Get delta time from ImGui
    float dt = ImGui::GetIO().DeltaTime;

    glm::mat4 viewProj = camera->getProjectionMatrix() * camera->getViewMatrix();

    // Update and render bubbles
    for (int i = static_cast<int>(chatBubbles_.size()) - 1; i >= 0; --i) {
        auto& bubble = chatBubbles_[i];
        bubble.timeRemaining -= dt;
        if (bubble.timeRemaining <= 0.0f) {
            chatBubbles_.erase(chatBubbles_.begin() + i);
            continue;
        }

        // Get entity position
        auto entity = gameHandler.getEntityManager().getEntity(bubble.senderGuid);
        if (!entity) continue;

        // Convert canonical → render coordinates, offset up by 2.5 units for bubble above head
        glm::vec3 canonical(entity->getX(), entity->getY(), entity->getZ() + 2.5f);
        glm::vec3 renderPos = core::coords::canonicalToRender(canonical);

        // Project to screen
        glm::vec4 clipPos = viewProj * glm::vec4(renderPos, 1.0f);
        if (clipPos.w <= 0.0f) continue;  // Behind camera

        glm::vec2 ndc(clipPos.x / clipPos.w, clipPos.y / clipPos.w);
        float screenX = (ndc.x * 0.5f + 0.5f) * screenW;
        float screenY = (1.0f - (ndc.y * 0.5f + 0.5f)) * screenH;  // Flip Y

        // Skip if off-screen
        if (screenX < -200.0f || screenX > screenW + 200.0f ||
            screenY < -100.0f || screenY > screenH + 100.0f) continue;

        // Fade alpha over last 2 seconds
        float alpha = 1.0f;
        if (bubble.timeRemaining < 2.0f) {
            alpha = bubble.timeRemaining / 2.0f;
        }

        // Draw bubble window
        std::string winId = "##ChatBubble" + std::to_string(bubble.senderGuid);
        ImGui::SetNextWindowPos(ImVec2(screenX, screenY), ImGuiCond_Always, ImVec2(0.5f, 1.0f));
        ImGui::SetNextWindowBgAlpha(0.7f * alpha);
        ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
            ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoInputs |
            ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav;

        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 8.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8, 4));

        ImGui::Begin(winId.c_str(), nullptr, flags);

        ImVec4 textColor = bubble.isYell
            ? ImVec4(1.0f, 0.2f, 0.2f, alpha)
            : ImVec4(1.0f, 1.0f, 1.0f, alpha);

        ImGui::PushStyleColor(ImGuiCol_Text, textColor);
        ImGui::PushTextWrapPos(200.0f);
        ImGui::TextWrapped("%s", bubble.message.c_str());
        ImGui::PopTextWrapPos();
        ImGui::PopStyleColor();

        ImGui::End();
        ImGui::PopStyleVar(2);
    }
}

void GameScreen::saveSettings() {
    std::string path = getSettingsPath();
    std::filesystem::path dir = std::filesystem::path(path).parent_path();
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);

    std::ofstream out(path);
    if (!out.is_open()) {
        LOG_WARNING("Could not save settings to ", path);
        return;
    }

    // Interface
    out << "ui_opacity=" << pendingUiOpacity << "\n";
    out << "minimap_rotate=" << (pendingMinimapRotate ? 1 : 0) << "\n";
    out << "minimap_square=" << (pendingMinimapSquare ? 1 : 0) << "\n";
    out << "separate_bags=" << (pendingSeparateBags ? 1 : 0) << "\n";

    // Audio
    out << "master_volume=" << pendingMasterVolume << "\n";
    out << "music_volume=" << pendingMusicVolume << "\n";
    out << "ambient_volume=" << pendingAmbientVolume << "\n";
    out << "ui_volume=" << pendingUiVolume << "\n";
    out << "combat_volume=" << pendingCombatVolume << "\n";
    out << "spell_volume=" << pendingSpellVolume << "\n";
    out << "movement_volume=" << pendingMovementVolume << "\n";
    out << "footstep_volume=" << pendingFootstepVolume << "\n";
    out << "npc_voice_volume=" << pendingNpcVoiceVolume << "\n";
    out << "mount_volume=" << pendingMountVolume << "\n";
    out << "activity_volume=" << pendingActivityVolume << "\n";

    // Controls
    out << "mouse_sensitivity=" << pendingMouseSensitivity << "\n";
    out << "invert_mouse=" << (pendingInvertMouse ? 1 : 0) << "\n";

    // Chat
    out << "chat_active_tab=" << activeChatTab_ << "\n";

    LOG_INFO("Settings saved to ", path);
}

void GameScreen::loadSettings() {
    std::string path = getSettingsPath();
    std::ifstream in(path);
    if (!in.is_open()) return;

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
                    pendingUiOpacity = v;
                    uiOpacity_ = static_cast<float>(v) / 100.0f;
                }
            } else if (key == "minimap_rotate") {
                // Ignore persisted rotate state; keep north-up.
                minimapRotate_ = false;
                pendingMinimapRotate = false;
            } else if (key == "minimap_square") {
                int v = std::stoi(val);
                minimapSquare_ = (v != 0);
                pendingMinimapSquare = minimapSquare_;
            } else if (key == "separate_bags") {
                pendingSeparateBags = (std::stoi(val) != 0);
                inventoryScreen.setSeparateBags(pendingSeparateBags);
            }
            // Audio
            else if (key == "master_volume") pendingMasterVolume = std::clamp(std::stoi(val), 0, 100);
            else if (key == "music_volume") pendingMusicVolume = std::clamp(std::stoi(val), 0, 100);
            else if (key == "ambient_volume") pendingAmbientVolume = std::clamp(std::stoi(val), 0, 100);
            else if (key == "ui_volume") pendingUiVolume = std::clamp(std::stoi(val), 0, 100);
            else if (key == "combat_volume") pendingCombatVolume = std::clamp(std::stoi(val), 0, 100);
            else if (key == "spell_volume") pendingSpellVolume = std::clamp(std::stoi(val), 0, 100);
            else if (key == "movement_volume") pendingMovementVolume = std::clamp(std::stoi(val), 0, 100);
            else if (key == "footstep_volume") pendingFootstepVolume = std::clamp(std::stoi(val), 0, 100);
            else if (key == "npc_voice_volume") pendingNpcVoiceVolume = std::clamp(std::stoi(val), 0, 100);
            else if (key == "mount_volume") pendingMountVolume = std::clamp(std::stoi(val), 0, 100);
            else if (key == "activity_volume") pendingActivityVolume = std::clamp(std::stoi(val), 0, 100);
            // Controls
            else if (key == "mouse_sensitivity") pendingMouseSensitivity = std::clamp(std::stof(val), 0.05f, 1.0f);
            else if (key == "invert_mouse") pendingInvertMouse = (std::stoi(val) != 0);
            // Chat
            else if (key == "chat_active_tab") activeChatTab_ = std::clamp(std::stoi(val), 0, 3);
        } catch (...) {}
    }
    LOG_INFO("Settings loaded from ", path);
}

}} // namespace wowee::ui
