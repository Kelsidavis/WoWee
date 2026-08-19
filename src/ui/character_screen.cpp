#include "ui/character_screen.hpp"
#include "game/equipment_hash.hpp"
#include "ui/ui_colors.hpp"
#include "rendering/character_preview.hpp"
#include "rendering/renderer.hpp"
#include "pipeline/asset_manager.hpp"
#include "core/application.hpp"
#include "core/config_paths.hpp"
#include "core/logger.hpp"
#include "addons/addon_manager.hpp"
#include <imgui.h>
#include <imgui_internal.h>
#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>

namespace wowee { namespace ui {

namespace {

// Written in units and drawn at a scale taken from the window, the same way
// the login card is. See paper_ui.hpp.
constexpr float kSheetWidth  = 1020.0f;
constexpr float kSheetHeight = 640.0f;
constexpr float kPad         = 34.0f;
constexpr float kTitleSize   = 40.0f;
constexpr float kNameSize    = 26.0f;
constexpr float kLabelSize   = 14.0f;
constexpr float kBodySize    = 15.5f;
constexpr float kSmallSize   = 13.0f;
constexpr float kRowHeight   = 36.0f;
constexpr float kButtonH     = 44.0f;

/// Enter is claimed for the rest of the press when it takes a character into
/// the world, so the game screen does not read the same keystroke as "open
/// chat" the moment it appears. Any distinct id will do; this one is not a
/// window's, because this screen no longer has one.
constexpr ImGuiID kCharListOwner = 0xC4A21151u;

ImVec4 classColor(uint8_t classId) { return ui::getClassColor(classId); }

} // namespace

CharacterScreen::CharacterScreen() {
}

void CharacterScreen::render(game::GameHandler& gameHandler) {
    const ImVec2 screen = ImGui::GetIO().DisplaySize;
    const float scale = std::clamp(std::min(screen.x / 1280.0f, screen.y / 760.0f), 0.62f, 2.6f);
    ui_.begin(ImGui::GetIO().DeltaTime, scale);
    ui_.setLayer(PaperLayer::Page);

    const PaperTheme& theme = ui_.theme();
    const auto px = [this](float units) { return ui_.px(units); };

    // Ensure we can render a preview even if the state transition hook didn't
    // inject the AssetManager.
    if (!assetManager_) {
        assetManager_ = core::Application::getInstance().getAssetManager();
    }

    const auto& characters = gameHandler.getCharacters();
    if (characters.empty()) {
        renderNotice(gameHandler, screen.x, screen.y);
        ui_.end();
        return;
    }

    // If the list refreshed, keep selection stable by GUID.
    if (selectedCharacterGuid != 0) {
        const bool needReselect =
            (selectedCharacterIndex < 0) ||
            (selectedCharacterIndex >= static_cast<int>(characters.size())) ||
            (characters[static_cast<size_t>(selectedCharacterIndex)].guid != selectedCharacterGuid);
        if (needReselect) {
            for (size_t i = 0; i < characters.size(); ++i) {
                if (characters[i].guid == selectedCharacterGuid) {
                    selectedCharacterIndex = static_cast<int>(i);
                    break;
                }
            }
        }
    }

    // Restore last-selected character (once per screen visit)
    if (!restoredLastCharacter) {
        // Priority 1: Select newly created character if set
        if (!newlyCreatedCharacterName.empty()) {
            for (size_t i = 0; i < characters.size(); ++i) {
                if (characters[i].name == newlyCreatedCharacterName) {
                    selectedCharacterIndex = static_cast<int>(i);
                    selectedCharacterGuid = characters[i].guid;
                    saveLastCharacter(characters[i].guid);
                    newlyCreatedCharacterName.clear();
                    break;
                }
            }
        }
        // Priority 2: Restore last selected character
        if (selectedCharacterIndex < 0) {
            uint64_t lastGuid = loadLastCharacter();
            if (lastGuid != 0) {
                for (size_t i = 0; i < characters.size(); ++i) {
                    if (characters[i].guid == lastGuid) {
                        selectedCharacterIndex = static_cast<int>(i);
                        selectedCharacterGuid = lastGuid;
                        break;
                    }
                }
            }
        }
        // Fall back to first character if nothing matched
        if (selectedCharacterIndex < 0) {
            selectedCharacterIndex = 0;
            selectedCharacterGuid = characters[0].guid;
        }
        restoredLastCharacter = true;
    }

    const bool modalUp = (deleteConfirmStage != 0) || showAddonsWindow_;
    if (modalUp) ui_.pushInert();

    const float sheetW = std::min(px(kSheetWidth), screen.x - px(40));
    const float sheetH = std::min(px(kSheetHeight), screen.y - px(40));
    const ImVec2 a((screen.x - sheetW) * 0.5f, (screen.y - sheetH) * 0.5f);
    const ImVec2 b(a.x + sheetW, a.y + sheetH);
    ui_.sheet(a, b);
    ui_.claimMouse(a, b);

    const float titleSize = px(kTitleSize);
    const float bodySize = px(kBodySize);
    const float smallSize = px(kSmallSize);
    const float labelSize = px(kLabelSize);

    Column col{a.x + px(kPad), b.x - px(kPad), a.y + px(kPad)};

    // ---- heading ----------------------------------------------------------
    ui_.text(col.at(), "Choose a Hero", titleSize, theme.ink, /*titleFace=*/true);
    {
        const float w = ui_.textWidth("Choose a Hero", titleSize, true);
        const float y = col.y + titleSize;
        ui_.squiggle(ImVec2(col.x0, y), ImVec2(col.x0 + w * 1.04f, y), theme.crayonRed,
                     px(2.0f), 0x4417u);
    }
    ui_.textRight(col.x1, col.y + titleSize * 0.55f, "Or make a new one.", smallSize,
                  theme.pencil);
    col.gap(titleSize + px(18));

    if (!statusMessage.empty()) {
        const float h = ui_.wrappedHeight(col.width(), statusMessage.c_str(), smallSize);
        ui_.wrapped(col.at(), col.width(), statusMessage.c_str(), smallSize,
                    statusIsError ? theme.crayonRed : theme.crayonGreen);
        col.gap(h + px(6));
    }

    // ---- the two halves ---------------------------------------------------
    const float footerH = px(kButtonH) + ui_.lineHeight(smallSize) + px(20);
    const float bodyTop = col.y;
    const float bodyBottom = b.y - px(kPad) - footerH;
    const float gap = px(26);
    const float listW = col.width() * 0.56f;
    const float detailX = col.x0 + listW + gap;

    // The divider between them, drawn full height so the two halves read as
    // two pages rather than as one crowded one.
    ui_.rule(ImVec2(col.x0 + listW + gap * 0.5f, bodyTop),
             ImVec2(col.x0 + listW + gap * 0.5f, bodyBottom), paperFade(theme.pencil, 0.55f),
             px(1.0f), 0x6621u);

    const float levelX = col.x0 + listW * 0.40f;
    const float raceX = col.x0 + listW * 0.50f;
    const float classX = col.x0 + listW * 0.67f;
    const float zoneX = col.x0 + listW * 0.83f;

    ui_.text(ImVec2(col.x0 + px(10), bodyTop), "Name", labelSize, theme.inkSoft);
    ui_.text(ImVec2(levelX, bodyTop), "Lv", labelSize, theme.inkSoft);
    ui_.text(ImVec2(raceX, bodyTop), "Race", labelSize, theme.inkSoft);
    ui_.text(ImVec2(classX, bodyTop), "Class", labelSize, theme.inkSoft);
    ui_.text(ImVec2(zoneX, bodyTop), "Where", labelSize, theme.inkSoft);

    const float listTop = bodyTop + ui_.lineHeight(labelSize) + px(6);
    ui_.rule(ImVec2(col.x0, listTop - px(4)), ImVec2(col.x0 + listW, listTop - px(4)),
             paperFade(theme.ink, 0.5f), px(1.3f), 0x9931u);

    const auto enterWorld = [&](const game::Character& character) {
        characterSelected = true;
        saveLastCharacter(character.guid);
        setStatus("Entering world with " + character.name + "...");
        gameHandler.selectCharacter(character.guid);
        if (onCharacterSelected) onCharacterSelected(character.guid);
    };

    {
        const PaperUI::ListResult picked = ui_.list(
            "characters", ImVec2(col.x0, listTop), ImVec2(col.x0 + listW, bodyBottom),
            static_cast<int>(characters.size()), px(kRowHeight), selectedCharacterIndex,
            [&](int index, ImVec2 rowA, ImVec2 rowB, bool selected, bool) {
                const auto& character = characters[static_cast<size_t>(index)];
                const float y = rowA.y + ((rowB.y - rowA.y) - bodySize) * 0.5f - px(1);
                const float smallY = rowA.y + ((rowB.y - rowA.y) - smallSize) * 0.5f - px(1);

                ui_.text(ImVec2(rowA.x + px(10), y), character.name.c_str(), bodySize,
                         ui_.onPaper(getFactionColor(character.race)));

                char level[8];
                std::snprintf(level, sizeof(level), "%d", character.level);
                ui_.text(ImVec2(levelX, smallY), level, smallSize,
                         selected ? theme.ink : theme.inkSoft);
                ui_.text(ImVec2(raceX, smallY), game::getRaceName(character.race), smallSize,
                         theme.inkSoft);
                ui_.text(ImVec2(classX, smallY), game::getClassName(character.characterClass),
                         smallSize,
                         ui_.onPaper(classColor(static_cast<uint8_t>(character.characterClass))));

                std::string zone = gameHandler.getWhoAreaName(character.zoneId);
                if (zone.empty()) {
                    char fallback[16];
                    std::snprintf(fallback, sizeof(fallback), "%u", character.zoneId);
                    zone = fallback;
                }
                ui_.text(ImVec2(zoneX, smallY), zone.c_str(), smallSize, theme.pencil);
            });

        if (picked.clicked >= 0) {
            selectedCharacterIndex = picked.clicked;
            selectedCharacterGuid = characters[static_cast<size_t>(picked.clicked)].guid;
            saveLastCharacter(selectedCharacterGuid);
        }
        if (picked.activated >= 0) {
            enterWorld(characters[static_cast<size_t>(picked.activated)]);
        }
    }

    // Keyboard: up and down move the selection, Enter takes it into the world.
    if (!modalUp) {
        if (ImGui::IsKeyPressed(ImGuiKey_UpArrow, true) && selectedCharacterIndex > 0) {
            selectedCharacterIndex--;
            selectedCharacterGuid = characters[static_cast<size_t>(selectedCharacterIndex)].guid;
            saveLastCharacter(selectedCharacterGuid);
        }
        if (ImGui::IsKeyPressed(ImGuiKey_DownArrow, true) &&
            selectedCharacterIndex < static_cast<int>(characters.size()) - 1) {
            selectedCharacterIndex++;
            selectedCharacterGuid = characters[static_cast<size_t>(selectedCharacterIndex)].guid;
            saveLastCharacter(selectedCharacterGuid);
        }
        if ((ImGui::IsKeyPressed(ImGuiKey_Enter, false) ||
             ImGui::IsKeyPressed(ImGuiKey_KeypadEnter, false)) &&
            selectedCharacterIndex >= 0 &&
            selectedCharacterIndex < static_cast<int>(characters.size())) {
            // Claim Enter so the game screen doesn't activate chat on the same press
            ImGui::SetKeyOwner(ImGuiKey_Enter, kCharListOwner, ImGuiInputFlags_LockUntilRelease);
            ImGui::SetKeyOwner(ImGuiKey_KeypadEnter, kCharListOwner,
                               ImGuiInputFlags_LockUntilRelease);
            enterWorld(characters[static_cast<size_t>(selectedCharacterIndex)]);
        }
    }

    const bool haveSelection = selectedCharacterIndex >= 0 &&
                               selectedCharacterIndex < static_cast<int>(characters.size());
    if (haveSelection) {
        renderDetails(gameHandler, characters[static_cast<size_t>(selectedCharacterIndex)],
                      ImVec2(detailX, bodyTop), ImVec2(col.x1, bodyBottom));
    }

    // ---- footer -----------------------------------------------------------
    col.y = bodyBottom + px(10);
    ui_.rule(ImVec2(col.x0, col.y), ImVec2(col.x1, col.y), paperFade(theme.pencil, 0.55f),
             px(1.0f), 0x2288u);
    col.gap(px(8));
    ui_.text(col.at(), "Double-click a hero, or press Enter, to go in.", smallSize,
             theme.pencil);
    col.gap(ui_.lineHeight(smallSize) + px(8));

    {
        const float y = col.y;
        const float h = px(kButtonH);
        float x = col.x0;
        const auto quiet = [&](const char* id, const char* label, float w) {
            const bool hit = ui_.button(id, ImVec2(x, y), ImVec2(x + w, y + h), label);
            x += w + px(10);
            return hit;
        };

        if (quiet("back", "Back", px(100))) {
            if (onBack) onBack();
        }
        if (quiet("refresh", "Refresh", px(110))) {
            if (gameHandler.getState() == game::WorldState::READY ||
                gameHandler.getState() == game::WorldState::CHAR_LIST_RECEIVED) {
                gameHandler.requestCharacterList();
                setStatus("Refreshing character list...");
            }
        }
        if (quiet("create", "New Hero", px(130))) {
            if (onCreateCharacter) onCreateCharacter();
        }
        if (quiet("addons", "AddOns", px(110))) {
            showAddonsWindow_ = true;
        }

        if (haveSelection) {
            const auto& character = characters[static_cast<size_t>(selectedCharacterIndex)];
            const float enterW = px(190);
            const float deleteW = px(100);
            if (ui_.button("delete", ImVec2(col.x1 - enterW - px(12) - deleteW, y),
                           ImVec2(col.x1 - enterW - px(12), y + h), "Delete")) {
                deleteConfirmStage = 1;
            }
            const bool disconnected = gameHandler.getState() == game::WorldState::DISCONNECTED ||
                                      gameHandler.getState() == game::WorldState::FAILED;
            if (ui_.button("enter", ImVec2(col.x1 - enterW, y), ImVec2(col.x1, y + h),
                           "Enter World", PaperUI::ButtonKind::Primary, !disconnected)) {
                enterWorld(character);
            }
        }
    }

    if (modalUp) ui_.popInert();

    if (deleteConfirmStage != 0 && haveSelection) {
        renderDeleteConfirm(characters[static_cast<size_t>(selectedCharacterIndex)], screen.x,
                            screen.y);
    } else if (deleteConfirmStage != 0) {
        deleteConfirmStage = 0;  // the character it asked about is gone
    }
    if (showAddonsWindow_) renderAddonsSheet(screen.x, screen.y);

    ui_.end();
}

void CharacterScreen::renderNotice(game::GameHandler& gameHandler, float screenW,
                                   float screenH) {
    const PaperTheme& theme = ui_.theme();
    const auto px = [this](float units) { return ui_.px(units); };

    const auto state = gameHandler.getState();
    const bool loading = (state == game::WorldState::READY ||
                          state == game::WorldState::CHAR_LIST_REQUESTED);
    const bool disconnected = (state == game::WorldState::DISCONNECTED ||
                               state == game::WorldState::FAILED);
    // Also show a loading state while CHAR_LIST_REQUESTED is in-flight
    // (characters may be cleared to avoid stale UI).
    if (state == game::WorldState::READY) {
        gameHandler.requestCharacterList();
    }

    const char* heading = loading ? "One moment" : (disconnected ? "Disconnected" : "Nobody here");
    const char* body =
        loading ? "Asking the realm who lives here..."
                : (disconnected
                       ? "The server closed the connection before it sent the character list."
                       : "This account has no characters on this realm yet.");

    const float w = std::min(px(520), screenW - px(40));
    const float h = px(250);
    const ImVec2 a((screenW - w) * 0.5f, (screenH - h) * 0.5f);
    const ImVec2 b(a.x + w, a.y + h);
    ui_.sheet(a, b);
    ui_.claimMouse(a, b);

    Column col{a.x + px(kPad), b.x - px(kPad), a.y + px(kPad)};
    ui_.text(col.at(), heading, px(kTitleSize) * 0.8f, theme.ink, /*titleFace=*/true);
    col.gap(px(kTitleSize) * 0.8f + px(12));
    col.gap(ui_.wrapped(col.at(), col.width(), body, px(kBodySize),
                        disconnected ? theme.crayonRed : theme.inkSoft) + px(18));

    const float y = b.y - px(kPad) - px(kButtonH);
    const float bh = px(kButtonH);
    float x = col.x0;
    const auto quiet = [&](const char* id, const char* label, float bw) {
        const bool hit = ui_.button(id, ImVec2(x, y), ImVec2(x + bw, y + bh), label);
        x += bw + px(10);
        return hit;
    };
    if (quiet("notice.back", "Back", px(100))) {
        if (onBack) onBack();
    }
    if (!loading) {
        if (quiet("notice.refresh", "Refresh", px(110))) {
            if (gameHandler.getState() == game::WorldState::READY ||
                gameHandler.getState() == game::WorldState::CHAR_LIST_RECEIVED) {
                gameHandler.requestCharacterList();
                setStatus("Refreshing character list...");
            }
        }
        if (ui_.button("notice.create", ImVec2(col.x1 - px(150), y), ImVec2(col.x1, y + bh),
                       "New Hero", PaperUI::ButtonKind::Primary)) {
            if (onCreateCharacter) onCreateCharacter();
        }
    }
}

void CharacterScreen::renderDetails(game::GameHandler& gameHandler,
                                    const game::Character& character, ImVec2 a, ImVec2 b) {
    const PaperTheme& theme = ui_.theme();
    const auto px = [this](float units) { return ui_.px(units); };

    // Keep the 3D preview in sync with the selected character.
    if (assetManager_ && assetManager_->isInitialized()) {
        if (!preview_) {
            preview_ = std::make_unique<rendering::CharacterPreview>();
        }
        if (!previewInitialized_) {
            previewInitialized_ = preview_->initialize(assetManager_);
            if (!previewInitialized_) {
                LOG_WARNING("CharacterScreen: failed to init CharacterPreview");
                preview_.reset();
            } else {
                auto* renderer = core::Application::getInstance().getRenderer();
                if (renderer) renderer->registerPreview(preview_.get());
            }
        }
        if (preview_) {
            const uint64_t equipHash = game::hashEquipmentAppearance(character.equipment);
            const bool changed =
                (previewGuid_ != character.guid) ||
                (previewAppearanceBytes_ != character.appearanceBytes) ||
                (previewFacialFeatures_ != character.facialFeatures) ||
                (previewUseFemaleModel_ != character.useFemaleModel) ||
                (previewEquipHash_ != equipHash);

            if (changed) {
                uint8_t skin = character.appearanceBytes & 0xFF;
                uint8_t face = (character.appearanceBytes >> 8) & 0xFF;
                uint8_t hairStyle = (character.appearanceBytes >> 16) & 0xFF;
                uint8_t hairColor = (character.appearanceBytes >> 24) & 0xFF;

                if (preview_->loadCharacter(character.race, character.gender,
                                            skin, face, hairStyle, hairColor,
                                            character.facialFeatures, character.useFemaleModel)) {
                    preview_->applyEquipment(character.equipment);
                }

                previewGuid_ = character.guid;
                previewAppearanceBytes_ = character.appearanceBytes;
                previewFacialFeatures_ = character.facialFeatures;
                previewUseFemaleModel_ = character.useFemaleModel;
                previewEquipHash_ = equipHash;
            }

            // Drive preview animation and request composite for next beginFrame.
            preview_->update(ImGui::GetIO().DeltaTime);
            preview_->render();
            preview_->requestComposite();
        }
    }

    const float bodySize = px(kBodySize);
    const float smallSize = px(kSmallSize);
    Column col{a.x, b.x, a.y};

    // The facts take a known amount of room; the picture gets what is left.
    const float factsH = px(kNameSize) * 1.1f + px(10) + bodySize * 1.35f * 4.0f +
                         ui_.lineHeight(smallSize) * 2.0f + px(16);

    if (preview_ && preview_->getTextureId() && preview_->getWidth() > 0 &&
        preview_->getHeight() > 0) {
        const float aspect = static_cast<float>(preview_->getHeight()) /
                             static_cast<float>(preview_->getWidth());
        const float mat = px(9);
        float imgW = col.width() - mat * 2.0f;
        float imgH = imgW * aspect;
        const float roomH = (b.y - a.y) - factsH - mat * 2.0f;
        if (imgH > roomH) {
            imgH = std::max(roomH, px(80));
            imgW = imgH / aspect;
        }
        const float imgX = col.x0 + (col.width() - imgW) * 0.5f;
        const ImVec2 imgA(imgX, col.y + mat);
        const ImVec2 imgB(imgX + imgW, col.y + mat + imgH);
        ui_.image(reinterpret_cast<ImTextureID>(preview_->getTextureId()), imgA, imgB);
        if (ui_.hover(imgA, imgB) && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
            preview_->rotate(ImGui::GetIO().MouseDelta.x * 0.2f);
        }
        col.y = imgB.y + mat + px(14);
    } else if (!assetManager_ || !assetManager_->isInitialized()) {
        ui_.text(col.at(), "No picture - the assets are not loaded.", smallSize, theme.pencil);
        col.gap(ui_.lineHeight(smallSize) + px(10));
    }

    ui_.text(col.at(), character.name.c_str(), px(kNameSize),
             ui_.onPaper(getFactionColor(character.race)), /*titleFace=*/true);
    col.gap(px(kNameSize) * 1.1f);
    ui_.rule(ImVec2(col.x0, col.y), ImVec2(col.x1, col.y), paperFade(theme.pencil, 0.6f),
             px(1.0f), 0x5A20u);
    col.gap(px(10));

    char level[32];
    std::snprintf(level, sizeof(level), "Level %d %s", character.level,
                  game::getGenderName(character.gender));
    ui_.text(col.at(), level, bodySize, theme.inkSoft);
    col.gap(bodySize * 1.35f);

    ui_.text(col.at(), game::getRaceName(character.race), bodySize, theme.inkSoft);
    col.gap(bodySize * 1.35f);

    ui_.text(col.at(), game::getClassName(character.characterClass), bodySize,
             ui_.onPaper(classColor(static_cast<uint8_t>(character.characterClass))));
    col.gap(bodySize * 1.35f);

    {
        std::string zone = gameHandler.getWhoAreaName(character.zoneId);
        if (zone.empty()) {
            char fallback[32];
            std::snprintf(fallback, sizeof(fallback), "Zone %u", character.zoneId);
            zone = fallback;
        }
        ui_.text(col.at(), zone.c_str(), bodySize, theme.pencil);
        col.gap(bodySize * 1.35f);
    }

    if (character.hasGuild()) {
        const std::string& guildName = gameHandler.lookupGuildName(character.guildId);
        const std::string line = guildName.empty() ? std::string("Guild: resolving...")
                                                   : "<" + guildName + ">";
        ui_.text(col.at(), line.c_str(), smallSize, theme.inkSoft);
    } else {
        ui_.text(col.at(), "No guild", smallSize, theme.pencil);
    }
    col.gap(ui_.lineHeight(smallSize));

    if (character.hasPet()) {
        char pet[64];
        std::snprintf(pet, sizeof(pet), "Pet, level %d (family %d)", character.pet.level,
                      character.pet.family);
        ui_.text(col.at(), pet, smallSize, theme.pencil);
    }
}

void CharacterScreen::renderDeleteConfirm(const game::Character& character, float screenW,
                                          float screenH) {
    const PaperTheme& theme = ui_.theme();
    const auto px = [this](float units) { return ui_.px(units); };

    ui_.setLayer(PaperLayer::Overlay);
    ui_.scrim(0.5f);

    const bool final = (deleteConfirmStage == 2);
    const float w = std::min(px(480), screenW - px(40));
    const float h = px(final ? 270.0f : 250.0f);
    const ImVec2 a((screenW - w) * 0.5f, (screenH - h) * 0.5f);
    const ImVec2 b(a.x + w, a.y + h);
    ui_.sheet(a, b, /*taped=*/false);
    ui_.claimMouse(a, b);

    Column col{a.x + px(28), b.x - px(28), a.y + px(28)};

    char line[192];
    if (final) {
        ui_.text(col.at(), "This cannot be undone", px(26), theme.crayonRed,
                 /*titleFace=*/true);
        col.gap(px(30));
        std::snprintf(line, sizeof(line),
                      "%s will be gone for good. Are you certain?", character.name.c_str());
    } else {
        ui_.text(col.at(), "Delete this hero?", px(26), theme.ink, /*titleFace=*/true);
        col.gap(px(30));
        std::snprintf(line, sizeof(line), "%s, level %d %s %s.", character.name.c_str(),
                      character.level, game::getRaceName(character.race),
                      game::getClassName(character.characterClass));
    }
    col.gap(ui_.wrapped(col.at(), col.width(), line, px(kBodySize), theme.inkSoft) + px(10));

    const float y = b.y - px(28) - px(kButtonH);
    const float bh = px(kButtonH);
    if (ui_.button("del.cancel", ImVec2(col.x0, y), ImVec2(col.x0 + px(110), y + bh),
                   "Keep")) {
        deleteConfirmStage = 0;
    }
    const float confirmW = px(final ? 200.0f : 170.0f);
    if (ui_.button("del.confirm", ImVec2(col.x1 - confirmW, y), ImVec2(col.x1, y + bh),
                   final ? "Delete for good" : "Yes, delete",
                   PaperUI::ButtonKind::Primary)) {
        if (final) {
            if (onDeleteCharacter) onDeleteCharacter(character.guid);
            deleteConfirmStage = 0;
            selectedCharacterIndex = -1;
            selectedCharacterGuid = 0;
        } else {
            deleteConfirmStage = 2;
        }
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) deleteConfirmStage = 0;

    ui_.setLayer(PaperLayer::Page);
}

void CharacterScreen::setStatus(const std::string& message, bool isError) {
    statusMessage = message;
    statusIsError = isError;
}

void CharacterScreen::renderAddonsSheet(float screenW, float screenH) {
    const PaperTheme& theme = ui_.theme();
    const auto px = [this](float units) { return ui_.px(units); };

    ui_.setLayer(PaperLayer::Overlay);
    ui_.scrim(0.45f);

    const float w = std::min(px(560), screenW - px(40));
    const float h = std::min(px(520), screenH - px(40));
    const ImVec2 a((screenW - w) * 0.5f, (screenH - h) * 0.5f);
    const ImVec2 b(a.x + w, a.y + h);
    ui_.sheet(a, b, /*taped=*/false);
    ui_.claimMouse(a, b);

    Column col{a.x + px(28), b.x - px(28), a.y + px(28)};
    const float smallSize = px(kSmallSize);

    ui_.text(col.at(), "AddOns", px(30), theme.ink, /*titleFace=*/true);
    {
        const float r = smallSize * 0.95f;
        if (ui_.glyphButton("addons.close", ImVec2(col.x1 - r, col.y + r), r,
                            PaperUI::Glyph::Cross)) {
            showAddonsWindow_ = false;
        }
    }
    col.gap(px(34));

    auto* am = services_.addonManager;
    if (!am) {
        ui_.text(col.at(), "The addon system is not running.", px(kBodySize), theme.pencil);
        ui_.setLayer(PaperLayer::Page);
        return;
    }

    const auto& addons = am->getAddons();
    ui_.text(col.at(), "Enabled addons load when you enter the world, or on /reload.",
             smallSize, theme.pencil);
    col.gap(ui_.lineHeight(smallSize) + px(8));
    ui_.rule(ImVec2(col.x0, col.y), ImVec2(col.x1, col.y), paperFade(theme.ink, 0.45f),
             px(1.2f), 0x7712u);
    col.gap(px(8));

    const float listBottom = b.y - px(28);
    if (addons.empty()) {
        col.gap(ui_.wrapped(col.at(), col.width(),
                            "None installed. Put addon folders under interface/AddOns/ in your "
                            "data path, then restart the client.",
                            px(kBodySize), theme.pencil));
        ui_.setLayer(PaperLayer::Page);
        return;
    }

    const float rowH = px(38);
    const PaperUI::ListResult picked = ui_.list(
        "addons", ImVec2(col.x0, col.y), ImVec2(col.x1, listBottom),
        static_cast<int>(addons.size()), rowH, -1,
        [&](int index, ImVec2 rowA, ImVec2 rowB, bool, bool) {
            const auto& addon = addons[static_cast<size_t>(index)];
            const bool enabled = am->isAddonEnabled(addon.addonName);
            const float boxSize = px(17);
            const float cy = (rowA.y + rowB.y) * 0.5f;

            // The checkbox is drawn here rather than left to the row's click,
            // so an addon can be turned on without being pointed at exactly.
            bool value = enabled;
            if (ui_.checkbox(addon.addonName.c_str(),
                             ImVec2(rowA.x + px(8), cy - boxSize * 0.5f), boxSize, nullptr,
                             &value)) {
                am->setAddonEnabled(addon.addonName, value);
            }

            const float textX = rowA.x + px(8) + boxSize + px(12);
            const std::string title = addon.getTitle();
            ui_.text(ImVec2(textX, rowA.y + px(4)), title.c_str(), px(kBodySize),
                     enabled ? theme.ink : theme.pencil);

            std::string sub;
            if (auto it = addon.directives.find("Version");
                it != addon.directives.end() && !it->second.empty()) {
                sub = "v" + it->second;
            }
            if (auto it = addon.directives.find("Author");
                it != addon.directives.end() && !it->second.empty()) {
                if (!sub.empty()) sub += "  ";
                sub += "by " + it->second;
            }
            if (!sub.empty()) {
                ui_.text(ImVec2(textX, rowA.y + px(4) + px(kBodySize) * 1.15f), sub.c_str(),
                         smallSize, theme.pencil);
            }
        });
    (void)picked;

    if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) showAddonsWindow_ = false;
    ui_.setLayer(PaperLayer::Page);
}

void CharacterScreen::selectCharacterByName(const std::string& name) {
    newlyCreatedCharacterName = name;
    restoredLastCharacter = false;  // Allow re-selection in render()
    selectedCharacterIndex = -1;
}

ImVec4 CharacterScreen::getFactionColor(game::Race race) const {
    // Alliance races: blue
    if (race == game::Race::HUMAN ||
        race == game::Race::DWARF ||
        race == game::Race::NIGHT_ELF ||
        race == game::Race::GNOME ||
        race == game::Race::DRAENEI) {
        return ImVec4(0.3f, 0.5f, 1.0f, 1.0f);
    }

    // Horde races: red
    if (race == game::Race::ORC ||
        race == game::Race::UNDEAD ||
        race == game::Race::TAUREN ||
        race == game::Race::TROLL ||
        race == game::Race::BLOOD_ELF) {
        return ui::colors::kRed;
    }

    return ui::colors::kWhite;
}

std::string CharacterScreen::getConfigDir() {
    return core::getConfigRoot();
}

void CharacterScreen::saveLastCharacter(uint64_t guid) {
    std::string dir = getConfigDir();
    std::filesystem::create_directories(dir);
    std::ofstream f(dir + "/last_character.cfg");
    if (f) f << guid;
}

uint64_t CharacterScreen::loadLastCharacter() {
    std::string path = getConfigDir() + "/last_character.cfg";
    std::ifstream f(path);
    uint64_t guid = 0;
    if (f) f >> guid;
    return guid;
}

}} // namespace wowee::ui
