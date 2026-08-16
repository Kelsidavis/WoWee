// ============================================================
// WindowManager - extracted from GameScreen
// Owns all NPC interaction windows, popup dialogs, etc.
// ============================================================
#include "core/local_time.hpp"
#include "ui/window_manager.hpp"
#include "ui/ui_texture_load.hpp"
#include "game/item_text.hpp"
#include "ui/framexml_takeover.hpp"
#include "ui/ui_upload_budget.hpp"
#include "game/inventory_slots.hpp"
#include "ui/chat_panel.hpp"
#include "ui/chat/chat_utils.hpp"
#include "ui/settings_panel.hpp"
#include "ui/spellbook_screen.hpp"
#include "ui/inventory_screen.hpp"
#include "ui/ui_colors.hpp"
#include "core/application.hpp"
#include "core/logger.hpp"
#include "rendering/renderer.hpp"
#include "rendering/character_preview.hpp"
#include "rendering/vk_context.hpp"
#include "core/window.hpp"
#include "game/game_handler.hpp"
#include "game/auction_filters.hpp"
#include "game/packed_time.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_layout.hpp"
#include "audio/audio_coordinator.hpp"
#include "audio/ui_sound_manager.hpp"
#include "audio/music_manager.hpp"
#include "pipeline/spell_icon_paths.hpp"
#include <imgui.h>
#include <imgui_internal.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <string>
#include <fstream>

namespace {
    using namespace wowee::ui::colors;

    // Abbreviated month names (indexed 0-11)
    constexpr const char* kMonthAbbrev[12] = {
        "Jan","Feb","Mar","Apr","May","Jun",
        "Jul","Aug","Sep","Oct","Nov","Dec"
    };

    constexpr auto& kColorRed         = kRed;
    constexpr auto& kColorGreen       = kGreen;
    constexpr auto& kColorYellow      = kYellow;
    constexpr auto& kColorGray        = kGray;
    constexpr auto& kColorDarkGray    = kDarkGray;

    // Common ImGui window flags for popup dialogs
    const ImGuiWindowFlags kDialogFlags = ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize;

} // anonymous namespace

namespace wowee {
namespace ui {

WindowManager::~WindowManager() = default;









void WindowManager::renderEscapeMenu(SettingsPanel& settingsPanel,
                                     game::GameHandler& gameHandler) {
    if (!showEscapeMenu) return;

    ImGuiIO& io = ImGui::GetIO();
    float screenW = io.DisplaySize.x;
    float screenH = io.DisplaySize.y;
    ImVec2 size(260.0f, 248.0f);
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
            settingsPanel.showEscapeSettingsNotice = false;
        }
        if (ImGui::Button("Quit", ImVec2(-1, 0))) {
            auto* ac = services_.audioCoordinator;
            if (ac) {
                if (auto* music = ac->getMusicManager()) {
                    music->stopMusic(0.0f);
                }
            }
            if (auto* window = services_.window) {
                window->setShouldClose(true);
            } else if (auto* window = core::Application::getInstance().getWindow()) {
                window->setShouldClose(true);
            }
            showEscapeMenu = false;
        }
        if (ImGui::Button("Settings", ImVec2(-1, 0))) {
            settingsPanel.showEscapeSettingsNotice = false;
            // The same routing as the Help button below, and for the same
            // reason: this menu is the client's own, the settings behind it
            // need not be.
            //
            // There is one settings screen, not two. Every category this client
            // adds - Graphics, Upscaling, Display, Camera, Interface, Gameplay,
            // Combat, Chat, Minimap, Nameplates - is built as a panel inside
            // the game's own options frames, so opening a second window of our
            // own beside them offered the same settings twice, in two places
            // that looked nothing like each other. The window is still what a
            // client without the interface up falls back to.
            if (frameXmlOwns(UiElement::GameMenu)) {
                gameHandler.runInterfaceCommand("VideoOptionsFrame_Toggle()");
            } else {
                settingsPanel.showSettingsWindow = true;
                settingsPanel.settingsInit = false;
            }
            showEscapeMenu = false;
        }
        if (ImGui::Button("Instance Lockouts", ImVec2(-1, 0))) {
            showInstanceLockouts_ = true;
            showEscapeMenu = false;
        }
        if (ImGui::Button("Help / GM Ticket", ImVec2(-1, 0))) {
            // This menu is this client's own; the help panel behind the
            // button is FrameXML's. The same routing as the slash command.
            gameHandler.runInterfaceCommand("ToggleHelpFrame()");
            showEscapeMenu = false;
        }

        ImGui::Spacing();
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(10.0f, 10.0f));
        if (ImGui::Button("Back to Game", ImVec2(-1, 0))) {
            showEscapeMenu = false;
            settingsPanel.showEscapeSettingsNotice = false;
        }
        ImGui::PopStyleVar();
    }
    ImGui::End();
}

// ---------------------------------------------------------------------------
// Barber shop state
//
// Built once each time the chair opens, and kept out of the render function so
// that the answers survive the panel being handed to FrameXML.
// ---------------------------------------------------------------------------
int WindowManager::barberFindAppearance(const std::vector<BarberStyleOption>& options,
                                        uint8_t id) {
    const auto it = std::find_if(options.begin(), options.end(),
                                 [id](const auto& option) { return option.appearanceId == id; });
    return it == options.end() ? -1 : static_cast<int>(std::distance(options.begin(), it));
}

uint8_t WindowManager::barberSelectedAppearance(const std::vector<BarberStyleOption>& options,
                                                int index, uint8_t fallback) {
    return index >= 0 && index < static_cast<int>(options.size())
        ? options[static_cast<size_t>(index)].appearanceId : fallback;
}

void WindowManager::rebuildBarberHairColors(uint8_t hairStyle, uint8_t preferredColor,
                                            uint32_t raceId, uint32_t sexId) {
    barberHairColors_.clear();
    if (services_.assetManager) {
        auto dbc = services_.assetManager->loadDBC("CharSections.dbc");
        if (dbc && dbc->isLoaded()) {
            const auto* layout = pipeline::getActiveDBCLayout()
                ? pipeline::getActiveDBCLayout()->getLayout("CharSections") : nullptr;
            const auto fields = pipeline::detectCharSectionsFields(dbc.get(), layout);
            for (uint32_t row = 0; row < dbc->getRecordCount(); ++row) {
                if (dbc->getUInt32(row, fields.raceId) != raceId ||
                    dbc->getUInt32(row, fields.sexId) != sexId ||
                    dbc->getUInt32(row, fields.baseSection) != 3 ||
                    dbc->getUInt32(row, fields.variationIndex) != hairStyle) {
                    continue;
                }
                const uint32_t color = dbc->getUInt32(row, fields.colorIndex);
                if (color <= 255) barberHairColors_.push_back(static_cast<uint8_t>(color));
            }
        }
    }
    std::sort(barberHairColors_.begin(), barberHairColors_.end());
    barberHairColors_.erase(std::unique(barberHairColors_.begin(), barberHairColors_.end()),
                            barberHairColors_.end());
    const auto preferred = std::find(barberHairColors_.begin(), barberHairColors_.end(),
                                     preferredColor);
    barberHairColor_ = preferred == barberHairColors_.end()
        ? (barberHairColors_.empty() ? -1 : 0)
        : static_cast<int>(std::distance(barberHairColors_.begin(), preferred));
    barberColorsForHairStyle_ = hairStyle;
}

void WindowManager::ensureBarberState(game::GameHandler& gameHandler) {
    // Leaving the chair clears the state so the next visit rebuilds it against
    // whatever the character wears by then. The render function used to do this
    // on its early return, which stops happening the moment the panel is handed
    // over - and stale originals would price a change against the wrong hair.
    if (!gameHandler.isBarberShopOpen()) {
        barberInitialized_ = false;
        return;
    }
    if (barberInitialized_) return;
    const auto* ch = gameHandler.getActiveCharacter();
    if (!ch) return;

    const uint32_t raceId = static_cast<uint32_t>(ch->race);
    const uint32_t sexId = (ch->gender == game::Gender::FEMALE || ch->useFemaleModel) ? 1u : 0u;

    // BarberShopStyle IDs, rather than the visible appearance numbers, are the
    // wire values expected by a WotLK server. Build the race/sex-specific lists
    // once each time the chair opens.
    barberOrigSkinColor_ = static_cast<uint8_t>(ch->appearanceBytes & 0xFF);
    barberOrigHairStyle_ = static_cast<uint8_t>((ch->appearanceBytes >> 16) & 0xFF);
    barberOrigHairColor_ = static_cast<uint8_t>((ch->appearanceBytes >> 24) & 0xFF);
    barberOrigFacialHair_ = ch->facialFeatures;
    barberHairStyles_.clear();
    barberFacialStyles_.clear();
    barberSkinStyles_.clear();
    // Entry zero tells the server to retain the existing skin.
    barberSkinStyles_.push_back({0, barberOrigSkinColor_, "Current"});

    if (services_.assetManager) {
        auto dbc = services_.assetManager->loadDBC("BarberShopStyle.dbc");
        if (dbc && dbc->isLoaded() && dbc->getFieldCount() >= 40) {
            for (uint32_t row = 0; row < dbc->getRecordCount(); ++row) {
                if (dbc->getUInt32(row, 37) != raceId || dbc->getUInt32(row, 38) != sexId)
                    continue;
                const uint32_t appearance = dbc->getUInt32(row, 39);
                if (appearance > 255) continue;
                std::string name;
                for (uint32_t field = 2; field <= 17 && name.empty(); ++field)
                    name = dbc->getString(row, field);
                if (name.empty()) name = "Style " + std::to_string(appearance);
                BarberStyleOption option{dbc->getUInt32(row, 0),
                                         static_cast<uint8_t>(appearance), std::move(name)};
                switch (dbc->getUInt32(row, 1)) {
                    case 0: barberHairStyles_.push_back(std::move(option)); break;
                    case 2: barberFacialStyles_.push_back(std::move(option)); break;
                    case 3: barberSkinStyles_.push_back(std::move(option)); break;
                    default: break;
                }
            }
        } else {
            LOG_WARNING("Barber Shop: WotLK BarberShopStyle.dbc is unavailable or malformed");
        }

        auto baseCost = services_.assetManager->loadDBC("gtBarberShopCostBase.dbc");
        if (baseCost && baseCost->isLoaded() && baseCost->getRecordCount() > 0) {
            const uint32_t level = std::max<uint32_t>(1, ch->level);
            const uint32_t row = std::min(level, baseCost->getRecordCount()) - 1;
            barberBaseCost_ = baseCost->getFloat(row, 0);
        } else {
            barberBaseCost_ = 0.0f;
        }
    }

    auto normalizeOptions = [](std::vector<BarberStyleOption>& options) {
        std::sort(options.begin(), options.end(), [](const auto& a, const auto& b) {
            return a.appearanceId != b.appearanceId
                ? a.appearanceId < b.appearanceId : a.entryId < b.entryId;
        });
        options.erase(std::unique(options.begin(), options.end(), [](const auto& a, const auto& b) {
            return a.appearanceId == b.appearanceId;
        }), options.end());
    };
    normalizeOptions(barberHairStyles_);
    normalizeOptions(barberFacialStyles_);
    if (barberSkinStyles_.size() > 1) {
        std::sort(barberSkinStyles_.begin() + 1, barberSkinStyles_.end(),
                  [](const auto& a, const auto& b) { return a.appearanceId < b.appearanceId; });
        barberSkinStyles_.erase(std::unique(barberSkinStyles_.begin() + 1,
                                            barberSkinStyles_.end(),
                                            [](const auto& a, const auto& b) {
                                                return a.appearanceId == b.appearanceId;
                                            }), barberSkinStyles_.end());
    }

    barberHairStyle_ = barberFindAppearance(barberHairStyles_, barberOrigHairStyle_);
    barberFacialHair_ = barberFindAppearance(barberFacialStyles_, barberOrigFacialHair_);
    barberSkinColor_ = 0;
    rebuildBarberHairColors(barberOrigHairStyle_, barberOrigHairColor_, raceId, sexId);
    barberPreviewSkin_ = barberPreviewHairStyle_ = barberPreviewHairColor_ =
        barberPreviewFacialHair_ = -1;
    barberInitialized_ = true;
}

WindowManager::BarberSelection WindowManager::barberSelection(game::GameHandler& gameHandler) {
    BarberSelection out;
    ensureBarberState(gameHandler);
    const auto* ch = gameHandler.getActiveCharacter();
    if (!ch) return out;
    const uint32_t raceId = static_cast<uint32_t>(ch->race);
    const uint32_t sexId = (ch->gender == game::Gender::FEMALE || ch->useFemaleModel) ? 1u : 0u;

    out.hairStyle = barberSelectedAppearance(barberHairStyles_, barberHairStyle_,
                                             barberOrigHairStyle_);
    // The colours on offer depend on the hair style, so a style change has to
    // rebuild them before the colour selection can be read back.
    if (out.hairStyle != barberColorsForHairStyle_) {
        const uint8_t previousColor =
            barberHairColor_ >= 0 && barberHairColor_ < static_cast<int>(barberHairColors_.size())
                ? barberHairColors_[static_cast<size_t>(barberHairColor_)] : barberOrigHairColor_;
        rebuildBarberHairColors(out.hairStyle, previousColor, raceId, sexId);
    }
    out.hairColor = barberHairColor_ >= 0 &&
                    barberHairColor_ < static_cast<int>(barberHairColors_.size())
        ? barberHairColors_[static_cast<size_t>(barberHairColor_)] : barberOrigHairColor_;
    out.facialHair = barberSelectedAppearance(barberFacialStyles_, barberFacialHair_,
                                              barberOrigFacialHair_);
    out.skin = barberSelectedAppearance(barberSkinStyles_, barberSkinColor_,
                                        barberOrigSkinColor_);
    return out;
}

uint32_t WindowManager::barberTotalCostCopper(game::GameHandler& gameHandler) {
    const BarberSelection sel = barberSelection(gameHandler);
    float cost = 0.0f;
    if (sel.hairStyle != barberOrigHairStyle_)      cost += barberBaseCost_;
    else if (sel.hairColor != barberOrigHairColor_) cost += barberBaseCost_ * 0.5f;
    if (sel.facialHair != barberOrigFacialHair_)    cost += barberBaseCost_ * 0.75f;
    if (sel.skin != barberOrigSkinColor_)           cost += barberBaseCost_ * 0.75f;
    return static_cast<uint32_t>(cost);
}

void WindowManager::barberResetSelections(game::GameHandler& gameHandler) {
    ensureBarberState(gameHandler);
    const auto* ch = gameHandler.getActiveCharacter();
    if (!ch) return;
    const uint32_t raceId = static_cast<uint32_t>(ch->race);
    const uint32_t sexId = (ch->gender == game::Gender::FEMALE || ch->useFemaleModel) ? 1u : 0u;
    barberHairStyle_ = barberFindAppearance(barberHairStyles_, barberOrigHairStyle_);
    barberFacialHair_ = barberFindAppearance(barberFacialStyles_, barberOrigFacialHair_);
    barberSkinColor_ = 0;
    rebuildBarberHairColors(barberOrigHairStyle_, barberOrigHairColor_, raceId, sexId);
}

void WindowManager::barberApplySelection(game::GameHandler& gameHandler) {
    ensureBarberState(gameHandler);
    const BarberSelection sel = barberSelection(gameHandler);
    // The server is sent BarberShopStyle.dbc entry ids, not the appearance
    // numbers the preview uses - the two are different numbering spaces and
    // only the entry id means anything on the wire.
    auto entryOf = [](const std::vector<BarberStyleOption>& options, int index) {
        return index >= 0 && index < static_cast<int>(options.size())
            ? options[static_cast<size_t>(index)].entryId : 0u;
    };
    gameHandler.sendAlterAppearance(entryOf(barberHairStyles_, barberHairStyle_),
                                    sel.hairColor,
                                    entryOf(barberFacialStyles_, barberFacialHair_),
                                    entryOf(barberSkinStyles_, barberSkinColor_));
}

bool WindowManager::barberStyleInfo(game::GameHandler& gameHandler, int selector,
                                    std::string& name, bool& isCurrent) {
    ensureBarberState(gameHandler);
    const BarberSelection sel = barberSelection(gameHandler);
    switch (selector) {
        case 1:
            if (barberHairStyle_ < 0 ||
                barberHairStyle_ >= static_cast<int>(barberHairStyles_.size())) return false;
            name = barberHairStyles_[static_cast<size_t>(barberHairStyle_)].name;
            isCurrent = sel.hairStyle == barberOrigHairStyle_;
            return true;
        case 2: {
            if (barberHairColor_ < 0 ||
                barberHairColor_ >= static_cast<int>(barberHairColors_.size())) return false;
            // The colours have no names of their own in the data, so they are
            // numbered the way the character creation screen numbers them.
            name = "Color " + std::to_string(barberHairColor_ + 1);
            isCurrent = sel.hairColor == barberOrigHairColor_;
            return true;
        }
        case 3:
            if (barberFacialHair_ < 0 ||
                barberFacialHair_ >= static_cast<int>(barberFacialStyles_.size())) return false;
            name = barberFacialStyles_[static_cast<size_t>(barberFacialHair_)].name;
            isCurrent = sel.facialHair == barberOrigFacialHair_;
            return true;
        case 4:
            if (barberSkinColor_ < 0 ||
                barberSkinColor_ >= static_cast<int>(barberSkinStyles_.size())) return false;
            name = barberSkinStyles_[static_cast<size_t>(barberSkinColor_)].name;
            isCurrent = sel.skin == barberOrigSkinColor_;
            return true;
        default:
            return false;
    }
}

void WindowManager::barberCycleStyle(game::GameHandler& gameHandler, int selector,
                                     int direction) {
    ensureBarberState(gameHandler);
    const int step = direction >= 0 ? 1 : -1;
    auto advance = [step](int& index, size_t count) {
        if (count == 0) { index = -1; return; }
        const int n = static_cast<int>(count);
        index = ((index < 0 ? 0 : index) + step % n + n) % n;
    };
    switch (selector) {
        case 1: advance(barberHairStyle_, barberHairStyles_.size()); break;
        case 2: advance(barberHairColor_, barberHairColors_.size()); break;
        case 3: advance(barberFacialHair_, barberFacialStyles_.size()); break;
        case 4: advance(barberSkinColor_, barberSkinStyles_.size()); break;
        default: break;
    }
    // Re-resolve so a hair style change rebuilds the colours behind it.
    barberSelection(gameHandler);
}

void WindowManager::renderBarberShopWindow(game::GameHandler& gameHandler) {
    if (!gameHandler.isBarberShopOpen()) {
        barberInitialized_ = false;
        return;
    }

    const auto* ch = gameHandler.getActiveCharacter();
    if (!ch) return;

    auto selectedEntry = [](const std::vector<BarberStyleOption>& options, int index) {
        return index >= 0 && index < static_cast<int>(options.size())
            ? options[static_cast<size_t>(index)].entryId : 0u;
    };

    const bool firstFrame = !barberInitialized_;
    ensureBarberState(gameHandler);
    if (firstFrame) {
        if (!barberPreview_ && services_.assetManager && services_.renderer) {
            barberPreview_ = std::make_unique<rendering::CharacterPreview>();
            if (barberPreview_->initialize(services_.assetManager)) {
                services_.renderer->registerPreview(barberPreview_.get());
                barberPreview_->resetView();
            } else {
                LOG_WARNING("Barber Shop: failed to initialize character preview");
                barberPreview_.reset();
            }
        }
    }

    // Resolves the selections and rebuilds the colour list when the hair style
    // has moved, which is the same work the interface's own barber asks for.
    const BarberSelection sel = barberSelection(gameHandler);
    const uint8_t hairStyle = sel.hairStyle;
    const uint8_t hairColor = sel.hairColor;
    const uint8_t facial    = sel.facialHair;
    const uint8_t skin      = sel.skin;

    if (barberPreview_ && (barberPreviewSkin_ != skin ||
                           barberPreviewHairStyle_ != hairStyle ||
                           barberPreviewHairColor_ != hairColor ||
                           barberPreviewFacialHair_ != facial)) {
        const uint8_t face = static_cast<uint8_t>((ch->appearanceBytes >> 8) & 0xFF);
        if (barberPreview_->loadCharacter(ch->race, ch->gender, skin, face, hairStyle,
                                          hairColor, facial, ch->useFemaleModel)) {
            barberPreview_->applyEquipment(ch->equipment);
        }
        barberPreviewSkin_ = skin;
        barberPreviewHairStyle_ = hairStyle;
        barberPreviewHairColor_ = hairColor;
        barberPreviewFacialHair_ = facial;
    }
    if (barberPreview_) {
        barberPreview_->update(ImGui::GetIO().DeltaTime);
        barberPreview_->render();
        barberPreview_->requestComposite();
    }

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;
    float winW = 720.0f;
    float winH = 520.0f;
    ImGui::SetNextWindowPos(ImVec2((screenW - winW) / 2.0f, (screenH - winH) / 2.0f), ImGuiCond_Appearing);
    ImGui::SetNextWindowSize(ImVec2(winW, winH), ImGuiCond_Appearing);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse;
    bool open = true;
    if (ImGui::Begin("Barber Shop", &open, flags)) {
        ImGui::Text("Choose your new look:");
        ImGui::Separator();
        ImGui::Spacing();

        const float previewW = 300.0f;
        ImGui::BeginChild("##BarberPreview", ImVec2(previewW, 410.0f), true);
        if (barberPreview_ && barberPreview_->getTextureId()) {
            const float imageW = ImGui::GetContentRegionAvail().x;
            const float imageH = imageW * static_cast<float>(barberPreview_->getHeight()) /
                                 static_cast<float>(barberPreview_->getWidth());
            ImGui::Image(reinterpret_cast<ImTextureID>(barberPreview_->getTextureId()),
                         ImVec2(imageW, std::min(imageH, 365.0f)));
            if (ImGui::IsItemHovered() && ImGui::IsMouseDragging(ImGuiMouseButton_Left))
                barberPreview_->rotate(ImGui::GetIO().MouseDelta.x * 0.2f);
            if (ImGui::IsItemHovered() && ImGui::GetIO().MouseWheel != 0.0f)
                barberPreview_->zoom(ImGui::GetIO().MouseWheel);
            ImGui::TextDisabled("Drag to rotate - Scroll to zoom");
        } else {
            ImGui::TextDisabled("Preview unavailable");
        }
        ImGui::EndChild();

        ImGui::SameLine();
        ImGui::BeginChild("##BarberControls", ImVec2(0, 410.0f), false);
        ImGui::PushItemWidth(-1);

        auto styleCombo = [](const char* label, int& selected,
                             const std::vector<BarberStyleOption>& options) {
            const char* preview = selected >= 0 && selected < static_cast<int>(options.size())
                ? options[static_cast<size_t>(selected)].name.c_str() : "Unavailable";
            if (ImGui::BeginCombo(label, preview)) {
                for (int i = 0; i < static_cast<int>(options.size()); ++i) {
                    const bool active = i == selected;
                    if (ImGui::Selectable(options[static_cast<size_t>(i)].name.c_str(), active))
                        selected = i;
                    if (active) ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
        };
        styleCombo("Hair Style", barberHairStyle_, barberHairStyles_);

        const char* colorPreview = "Unavailable";
        std::string colorLabel;
        if (barberHairColor_ >= 0 && barberHairColor_ < static_cast<int>(barberHairColors_.size())) {
            colorLabel = "Color " + std::to_string(hairColor);
            colorPreview = colorLabel.c_str();
        }
        if (ImGui::BeginCombo("Hair Color", colorPreview)) {
            for (int i = 0; i < static_cast<int>(barberHairColors_.size()); ++i) {
                const std::string label = "Color " + std::to_string(barberHairColors_[i]);
                const bool active = i == barberHairColor_;
                if (ImGui::Selectable(label.c_str(), active)) barberHairColor_ = i;
                if (active) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }

        const bool female = (ch->gender == game::Gender::FEMALE || ch->useFemaleModel);
        const char* facialLabel = female ? "Piercings / Features" : "Facial Hair / Features";
        styleCombo(facialLabel, barberFacialHair_, barberFacialStyles_);
        if (barberSkinStyles_.size() > 1)
            styleCombo("Skin Color", barberSkinColor_, barberSkinStyles_);
        ImGui::PopItemWidth();

        ImGui::Spacing();
        ImGui::Separator();

        const bool valid = barberHairStyle_ >= 0 && barberFacialHair_ >= 0 &&
                           barberHairColor_ >= 0;
        const bool changed = valid && (hairStyle != barberOrigHairStyle_ ||
                                       hairColor != barberOrigHairColor_ ||
                                       facial != barberOrigFacialHair_ ||
                                       skin != barberOrigSkinColor_);
        float costFloat = 0.0f;
        if (hairStyle != barberOrigHairStyle_) costFloat += barberBaseCost_;
        else if (hairColor != barberOrigHairColor_) costFloat += barberBaseCost_ * 0.5f;
        if (facial != barberOrigFacialHair_) costFloat += barberBaseCost_ * 0.75f;
        if (skin != barberOrigSkinColor_) costFloat += barberBaseCost_ * 0.75f;
        const uint32_t cost = static_cast<uint32_t>(costFloat);
        const bool canAfford = gameHandler.getMoneyCopper() >= cost;

        ImGui::Text("Price:");
        ImGui::SameLine();
        if (!canAfford) ImGui::PushStyleColor(ImGuiCol_Text, kColorRed);
        renderCoinsFromCopper(cost);
        if (!canAfford) ImGui::PopStyleColor();
        if (!valid)
            ImGui::TextColored(kColorRed, "No valid barber styles were found for this character.");
        ImGui::EndChild();

        // OK / Reset / Cancel buttons
        float btnW = 80.0f;
        float totalW = btnW * 3 + ImGui::GetStyle().ItemSpacing.x * 2;
        ImGui::SetCursorPosX((ImGui::GetWindowWidth() - totalW) / 2.0f);

        if (!changed || !canAfford) ImGui::BeginDisabled();
        if (ImGui::Button("OK", ImVec2(btnW, 0))) {
            gameHandler.sendAlterAppearance(
                selectedEntry(barberHairStyles_, barberHairStyle_),
                hairColor,
                selectedEntry(barberFacialStyles_, barberFacialHair_),
                selectedEntry(barberSkinStyles_, barberSkinColor_));
            // Keep window open - server will respond with SMSG_BARBER_SHOP_RESULT
        }
        if (!changed || !canAfford) ImGui::EndDisabled();

        ImGui::SameLine();
        if (!changed) ImGui::BeginDisabled();
        if (ImGui::Button("Reset", ImVec2(btnW, 0))) {
            barberResetSelections(gameHandler);
        }
        if (!changed) ImGui::EndDisabled();

        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(btnW, 0))) {
            gameHandler.closeBarberShop();
        }
    }
    ImGui::End();

    if (!open) {
        gameHandler.closeBarberShop();
    }
}



void WindowManager::renderLogoutCountdown(game::GameHandler& gameHandler) {
    if (!gameHandler.isLoggingOut()) return;
    // FrameXML counts the same seconds down. uiparent.lua raises the CAMP and
    // QUIT popups from PLAYER_CAMPING and PLAYER_QUITING, both of which this
    // client fires from social_handler, and "dialogs" has been handed over
    // since the branch started - so every /logout put two countdowns on
    // screen, one above the other.
    if (frameXmlOwns(UiElement::Dialogs)) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth())  : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    constexpr float W = 280.0f;
    constexpr float H = 80.0f;
    ImGui::SetNextWindowPos(ImVec2((screenW - W) * 0.5f, screenH * 0.5f - H * 0.5f - 60.0f),
                            ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(W, H), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.88f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 6.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.08f, 0.08f, 0.18f, 0.95f));
    ImGui::PushStyleColor(ImGuiCol_Border,   ImVec4(0.5f, 0.5f, 0.8f, 1.0f));

    if (ImGui::Begin("##LogoutCountdown", nullptr,
            ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoBringToFrontOnFocus)) {

        float cd = gameHandler.getLogoutCountdown();
        if (cd > 0.0f) {
            ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 6.0f);
            ImGui::SetCursorPosX((W - ImGui::CalcTextSize("Logging out in 20s...").x) * 0.5f);
            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.3f, 1.0f),
                               "Logging out in %ds...", static_cast<int>(std::ceil(cd)));

            // Progress bar (20 second countdown)
            float frac = 1.0f - std::min(cd / 20.0f, 1.0f);
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.5f, 0.5f, 0.9f, 1.0f));
            ImGui::ProgressBar(frac, ImVec2(-1.0f, 8.0f), "");
            ImGui::PopStyleColor();
            ImGui::Spacing();
        } else {
            ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 14.0f);
            ImGui::SetCursorPosX((W - ImGui::CalcTextSize("Logging out...").x) * 0.5f);
            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.3f, 1.0f), "Logging out...");
            ImGui::Spacing();
        }

        // Cancel button - only while countdown is still running
        if (cd > 0.0f) {
            float btnW = 100.0f;
            ImGui::SetCursorPosX((W - btnW) * 0.5f);
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.5f, 0.1f, 0.1f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.7f, 0.15f, 0.15f, 1.0f));
            if (ImGui::Button("Cancel", ImVec2(btnW, 0))) {
                gameHandler.cancelLogout();
            }
            ImGui::PopStyleColor(2);
        }
    }
    ImGui::End();
    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar();
}

void WindowManager::renderDeathScreen(game::GameHandler& gameHandler) {
    if (!gameHandler.showDeathDialog()) {
        deathTimerRunning_ = false;
        deathElapsed_ = 0.0f;
        return;
    }
    float dt = ImGui::GetIO().DeltaTime;
    if (!deathTimerRunning_) {
        deathElapsed_ = 0.0f;
        deathTimerRunning_ = true;
    } else {
        deathElapsed_ += dt;
    }

    auto* window = services_.window;
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

    // The prompt below is FrameXML's StaticPopup "DEATH", raised from
    // PLAYER_DEAD which this client fires. The red wash above it is not - WoW
    // desaturates the screen from C, so nothing on FrameXML's side draws it and
    // it stays whoever owns the dialogs.
    if (frameXmlOwns(UiElement::Dialogs)) return;

    // "Release Spirit" dialog centered on screen
    const bool hasSelfRes = gameHandler.canSelfRes();
    float dlgW = 280.0f;
    // Extra height when self-res button is available; +20 for the "wait for res" hint
    float dlgH = hasSelfRes ? 190.0f : 150.0f;
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
        ImGui::TextColored(colors::kBrightRed, "%s", deathText);

        // Respawn timer: show how long until the server auto-releases the spirit
        float timeLeft = kForcedReleaseSec - deathElapsed_;
        if (timeLeft > 0.0f) {
            int mins = static_cast<int>(timeLeft) / 60;
            int secs = static_cast<int>(timeLeft) % 60;
            char timerBuf[48];
            snprintf(timerBuf, sizeof(timerBuf), "Auto-release in %d:%02d", mins, secs);
            float tw = ImGui::CalcTextSize(timerBuf).x;
            ImGui::SetCursorPosX((dlgW - tw) / 2);
            ImGui::TextColored(colors::kMediumGray, "%s", timerBuf);
        }

        ImGui::Spacing();
        ImGui::Spacing();

        // Self-resurrection button (Reincarnation / Twisting Nether / Deathpact)
        if (hasSelfRes) {
            float btnW2 = 220.0f;
            ImGui::SetCursorPosX((dlgW - btnW2) / 2);
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.35f, 0.55f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.2f, 0.5f, 0.75f, 1.0f));
            if (ImGui::Button("Use Self-Resurrection", ImVec2(btnW2, 30))) {
                gameHandler.useSelfRes();
            }
            ImGui::PopStyleColor(2);
            ImGui::Spacing();
        }

        // Center the Release Spirit button
        float btnW = 180.0f;
        ImGui::SetCursorPosX((dlgW - btnW) / 2);
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.5f, 0.1f, 0.1f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.7f, 0.15f, 0.15f, 1.0f));
        if (ImGui::Button("Release Spirit", ImVec2(btnW, 30))) {
            gameHandler.releaseSpirit();
        }
        ImGui::PopStyleColor(2);

        // Hint: player can stay dead and wait for another player to cast Resurrection
        const char* resHint = "Or wait for a player to resurrect you.";
        float hw = ImGui::CalcTextSize(resHint).x;
        ImGui::SetCursorPosX((dlgW - hw) / 2);
        ImGui::TextColored(ImVec4(0.5f, 0.6f, 0.5f, 0.85f), "%s", resHint);
    }
    ImGui::End();
    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar();
}

void WindowManager::renderReclaimCorpseButton(game::GameHandler& gameHandler) {
    if (!gameHandler.isPlayerGhost() || !gameHandler.canReclaimCorpse()) return;
    // StaticPopup "RECOVER_CORPSE" now has the CORPSE_IN_RANGE edge it needs,
    // so the prompt appears on FrameXML's side and this one steps aside.
    if (frameXmlOwns(UiElement::Dialogs)) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) : 720.0f;

    float delaySec = gameHandler.getCorpseReclaimDelaySec();
    bool onDelay = (delaySec > 0.0f);

    float btnW = 220.0f, btnH = 36.0f;
    float winH = btnH + 16.0f + (onDelay ? 20.0f : 0.0f);
    ImGui::SetNextWindowPos(ImVec2(screenW / 2 - btnW / 2, screenH * 0.72f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(btnW + 16.0f, winH), ImGuiCond_Always);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 6.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8.0f, 8.0f));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.7f));
    if (ImGui::Begin("##ReclaimCorpse", nullptr,
            ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoBringToFrontOnFocus)) {
        if (onDelay) {
            // Greyed-out button while PvP reclaim timer ticks down
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.25f, 0.25f, 0.25f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.25f, 0.25f, 1.0f));
            ImGui::BeginDisabled(true);
            char delayLabel[64];
            snprintf(delayLabel, sizeof(delayLabel), "Resurrect from Corpse (%.0fs)", delaySec);
            ImGui::Button(delayLabel, ImVec2(btnW, btnH));
            ImGui::EndDisabled();
            ImGui::PopStyleColor(2);
            const char* waitMsg = "You cannot reclaim your corpse yet.";
            float tw = ImGui::CalcTextSize(waitMsg).x;
            ImGui::SetCursorPosX((btnW + 16.0f - tw) * 0.5f);
            ImGui::TextColored(ImVec4(0.8f, 0.5f, 0.2f, 1.0f), "%s", waitMsg);
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.15f, 0.35f, 0.15f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.55f, 0.25f, 1.0f));
            if (ImGui::Button("Resurrect from Corpse", ImVec2(btnW, btnH))) {
                gameHandler.reclaimCorpse();
            }
            ImGui::PopStyleColor(2);
            float corpDist = gameHandler.getCorpseDistance();
            if (corpDist >= 0.0f) {
                char distBuf[48];
                snprintf(distBuf, sizeof(distBuf), "Corpse: %.0f yards away", corpDist);
                float dw = ImGui::CalcTextSize(distBuf).x;
                ImGui::SetCursorPosX((btnW + 16.0f - dw) * 0.5f);
                ImGui::TextDisabled("%s", distBuf);
            }
        }
    }
    ImGui::End();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar(2);
}






void WindowManager::renderInstanceLockouts(game::GameHandler& gameHandler) {
    if (!showInstanceLockouts_) return;

    ImGui::SetNextWindowSize(ImVec2(480, 0), ImGuiCond_Appearing);
    ImGui::SetNextWindowPos(
        ImVec2(ImGui::GetIO().DisplaySize.x / 2 - 240, 140), ImGuiCond_Appearing);

    if (!ImGui::Begin("Instance Lockouts", &showInstanceLockouts_,
                      ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::End();
        return;
    }

    const auto& lockouts = gameHandler.getInstanceLockouts();

    if (lockouts.empty()) {
        ImGui::TextColored(kColorGray, "No active instance lockouts.");
    } else {
        auto difficultyLabel = [](uint32_t diff) -> const char* {
            switch (diff) {
                case 0: return "Normal";
                case 1: return "Heroic";
                case 2: return "25-Man";
                case 3: return "25-Man Heroic";
                default: return "Unknown";
            }
        };

        // Current UTC time for reset countdown
        auto nowSec = static_cast<uint64_t>(std::time(nullptr));

        if (ImGui::BeginTable("lockouts", 4,
                              ImGuiTableFlags_SizingStretchProp |
                              ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersOuter)) {
            ImGui::TableSetupColumn("Instance",   ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("Difficulty", ImGuiTableColumnFlags_WidthFixed, 110.0f);
            ImGui::TableSetupColumn("Resets In",  ImGuiTableColumnFlags_WidthFixed, 100.0f);
            ImGui::TableSetupColumn("Status",     ImGuiTableColumnFlags_WidthFixed, 60.0f);
            ImGui::TableHeadersRow();

            for (const auto& lo : lockouts) {
                ImGui::TableNextRow();

                // Instance name - use GameHandler's Map.dbc cache (avoids duplicate DBC load)
                ImGui::TableSetColumnIndex(0);
                std::string mapName = gameHandler.getMapName(lo.mapId);
                if (!mapName.empty()) {
                    ImGui::TextUnformatted(mapName.c_str());
                } else {
                    ImGui::Text("Map %u", lo.mapId);
                }

                // Difficulty
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(difficultyLabel(lo.difficulty));

                // Reset countdown
                ImGui::TableSetColumnIndex(2);
                if (lo.resetTime > nowSec) {
                    uint64_t remaining = lo.resetTime - nowSec;
                    uint64_t days  = remaining / 86400;
                    uint64_t hours = (remaining % 86400) / 3600;
                    if (days > 0) {
                        ImGui::Text("%llud %lluh",
                            static_cast<unsigned long long>(days),
                            static_cast<unsigned long long>(hours));
                    } else {
                        uint64_t mins = (remaining % 3600) / 60;
                        ImGui::Text("%lluh %llum",
                            static_cast<unsigned long long>(hours),
                            static_cast<unsigned long long>(mins));
                    }
                } else {
                    ImGui::TextColored(kColorDarkGray, "Expired");
                }

                // Locked / Extended status
                ImGui::TableSetColumnIndex(3);
                if (lo.extended) {
                    ImGui::TextColored(ImVec4(0.3f, 0.7f, 1.0f, 1.0f), "Ext");
                } else if (lo.locked) {
                    ImGui::TextColored(colors::kSoftRed, "Locked");
                } else {
                    ImGui::TextColored(ImVec4(0.5f, 0.9f, 0.5f, 1.0f), "Open");
                }
            }

            ImGui::EndTable();
        }
    }

    ImGui::End();
}

// ============================================================================
// Battleground score frame
//
// Displays the current score for the player's battleground using world states.
// Shown in the top-centre of the screen whenever SMSG_INIT_WORLD_STATES has
// been received for a known BG map.  The layout adapts per battleground:
//
//   WSG  489 – Alliance / Horde flag captures (max 3)
//   AB   529 – Alliance / Horde resource scores (max 1600)
//   AV    30 – Alliance / Horde reinforcements
//   EotS 566 – Alliance / Horde resource scores (max 1600)
// ============================================================================
// ─── Who Results Window ───────────────────────────────────────────────────────
// ─── Combat Log Window ────────────────────────────────────────────────────────




// ─── Inspect Window ───────────────────────────────────────────────────────────
// ─── Titles Window ────────────────────────────────────────────────────────────
void WindowManager::renderTitlesWindow(game::GameHandler& gameHandler) {
    if (!showTitlesWindow_) return;

    ImGui::SetNextWindowSize(ImVec2(320, 400), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(240, 170), ImGuiCond_FirstUseEver);

    if (!ImGui::Begin("Titles", &showTitlesWindow_)) {
        ImGui::End();
        return;
    }

    const auto& knownBits = gameHandler.getKnownTitleBits();
    const int32_t chosen  = gameHandler.getChosenTitleBit();

    if (knownBits.empty()) {
        ImGui::TextDisabled("No titles earned yet.");
        ImGui::End();
        return;
    }

    ImGui::TextUnformatted("Select a title to display:");
    ImGui::Separator();

    // "No Title" option
    bool noTitle = (chosen < 0);
    if (ImGui::Selectable("(No Title)", noTitle)) {
        if (!noTitle) gameHandler.sendSetTitle(-1);
    }
    if (noTitle) {
        ImGui::SameLine();
        ImGui::TextColored(colors::kBrightGold, "<-- active");
    }

    ImGui::Separator();

    // Sort known bits for stable display order
    std::vector<uint32_t> sortedBits(knownBits.begin(), knownBits.end());
    std::sort(sortedBits.begin(), sortedBits.end());

    ImGui::BeginChild("##titlelist", ImVec2(0, 0), false);
    for (uint32_t bit : sortedBits) {
        const std::string title = gameHandler.getFormattedTitle(bit);
        const std::string display = title.empty()
            ? ("Title #" + std::to_string(bit)) : title;

        bool isActive = (chosen >= 0 && static_cast<uint32_t>(chosen) == bit);
        ImGui::PushID(static_cast<int>(bit));

        if (isActive) {
            ImGui::PushStyleColor(ImGuiCol_Text, colors::kBrightGold);
        }
        if (ImGui::Selectable(display.c_str(), isActive)) {
            if (!isActive) gameHandler.sendSetTitle(static_cast<int32_t>(bit));
        }
        if (isActive) {
            ImGui::PopStyleColor();
            ImGui::SameLine();
            ImGui::TextDisabled("<-- active");
        }

        ImGui::PopID();
    }
    ImGui::EndChild();

    ImGui::End();
}

// ─── Equipment Set Manager Window ─────────────────────────────────────────────
// Nothing sets showEquipSetWindow_ true, anywhere - this window has no way to
// be opened and never draws. Left rather than removed because the equipment-set
// packets it reads were repaired this week and FrameXML's GearManagerDialog is
// the path that reaches them now: the manager is behind the equipmentManager
// CVar, which the interface options can set and which persists since cvars.cfg
// exists. If this client ever wants its own again, the window is here.
void WindowManager::renderEquipSetWindow(game::GameHandler& gameHandler) {
    if (!showEquipSetWindow_) return;

    ImGui::SetNextWindowSize(ImVec2(280, 320), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(260, 180), ImGuiCond_FirstUseEver);

    if (!ImGui::Begin("Equipment Sets##equipsets", &showEquipSetWindow_)) {
        ImGui::End();
        return;
    }

    const auto& sets = gameHandler.getEquipmentSets();

    if (sets.empty()) {
        ImGui::TextDisabled("No equipment sets saved.");
        ImGui::Spacing();
        ImGui::TextWrapped("Create equipment sets in-game using the default WoW equipment manager (Shift+click the Equipment Sets button).");
        ImGui::End();
        return;
    }

    ImGui::TextUnformatted("Click a set to equip it:");
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::BeginChild("##equipsetlist", ImVec2(0, 0), false);
    for (const auto& set : sets) {
        ImGui::PushID(static_cast<int>(set.setId));

        // Icon placeholder (use a coloured square if no icon texture available)
        ImVec2 iconSize(32.0f, 32.0f);
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.25f, 0.20f, 0.10f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.40f, 0.30f, 0.15f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.60f, 0.45f, 0.20f, 1.0f));
        if (ImGui::Button("##icon", iconSize)) {
            gameHandler.useEquipmentSet(set.setId);
        }
        ImGui::PopStyleColor(3);

        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Equip set: %s", set.name.c_str());
        }

        ImGui::SameLine();

        // Name and equip button
        ImGui::BeginGroup();
        ImGui::TextUnformatted(set.name.c_str());
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.35f, 0.15f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.30f, 0.50f, 0.22f, 1.0f));
        if (ImGui::SmallButton("Equip")) {
            gameHandler.useEquipmentSet(set.setId);
        }
        ImGui::PopStyleColor(2);
        ImGui::EndGroup();

        ImGui::Spacing();
        ImGui::PopID();
    }
    ImGui::EndChild();

    ImGui::End();
}

void WindowManager::renderSkillsWindow(game::GameHandler& gameHandler) {
    if (!showSkillsWindow_) return;

    ImGui::SetNextWindowSize(ImVec2(380, 480), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(220, 130), ImGuiCond_FirstUseEver);

    if (!ImGui::Begin("Skills & Professions", &showSkillsWindow_)) {
        ImGui::End();
        return;
    }

    const auto& skills = gameHandler.getPlayerSkills();
    if (skills.empty()) {
        ImGui::TextDisabled("No skill data received yet.");
        ImGui::End();
        return;
    }

    // Organise skills by category
    // WoW SkillLine.dbc categories: 6=Weapon, 7=Class, 8=Armor, 9=Secondary, 11=Professions, others=Misc
    struct SkillEntry {
        uint32_t skillId;
        const game::PlayerSkill* skill;
    };
    std::map<uint32_t, std::vector<SkillEntry>> byCategory;
    for (const auto& [id, sk] : skills) {
        uint32_t cat = gameHandler.getSkillCategory(id);
        byCategory[cat].push_back({id, &sk});
    }

    static constexpr struct { uint32_t cat; const char* label; } kCatOrder[] = {
        {11, "Professions"},
        { 9, "Secondary Skills"},
        { 7, "Class Skills"},
        { 6, "Weapon Skills"},
        { 8, "Armor"},
        { 5, "Languages"},
        { 0, "Other"},
    };

    // Collect handled categories to fall back to "Other" for unknowns
    static constexpr uint32_t kKnownCats[] = {11, 9, 7, 6, 8, 5};

    // Redirect unknown categories into bucket 0
    for (auto& [cat, vec] : byCategory) {
        bool known = false;
        for (uint32_t kc : kKnownCats) if (cat == kc) { known = true; break; }
        if (!known && cat != 0) {
            auto& other = byCategory[0];
            other.insert(other.end(), vec.begin(), vec.end());
            vec.clear();
        }
    }

    ImGui::BeginChild("##skillscroll", ImVec2(0, 0), false);

    for (const auto& [cat, label] : kCatOrder) {
        auto it = byCategory.find(cat);
        if (it == byCategory.end() || it->second.empty()) continue;

        auto& entries = it->second;
        // Sort alphabetically within each category
        std::sort(entries.begin(), entries.end(), [&](const SkillEntry& a, const SkillEntry& b) {
            return gameHandler.getSkillName(a.skillId) < gameHandler.getSkillName(b.skillId);
        });

        if (ImGui::CollapsingHeader(label, ImGuiTreeNodeFlags_DefaultOpen)) {
            for (const auto& e : entries) {
                const std::string& name = gameHandler.getSkillName(e.skillId);
                const char* displayName = name.empty() ? "Unknown" : name.c_str();
                uint16_t val = e.skill->effectiveValue();
                uint16_t maxVal = e.skill->maxValue;

                ImGui::PushID(static_cast<int>(e.skillId));

                // Name column
                ImGui::TextUnformatted(displayName);
                ImGui::SameLine(170.0f);

                // Progress bar
                float fraction = (maxVal > 0) ? static_cast<float>(val) / static_cast<float>(maxVal) : 0.0f;
                char overlay[32];
                snprintf(overlay, sizeof(overlay), "%u / %u", val, maxVal);
                ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.20f, 0.55f, 0.20f, 1.0f));
                ImGui::ProgressBar(fraction, ImVec2(160.0f, 14.0f), overlay);
                ImGui::PopStyleColor();

                if (ImGui::IsItemHovered()) {
                    ImGui::BeginTooltip();
                    ImGui::Text("%s", displayName);
                    ImGui::Separator();
                    ImGui::Text("Base: %u", e.skill->value);
                    if (e.skill->bonusPerm > 0)
                        ImGui::Text("Permanent bonus: +%u", e.skill->bonusPerm);
                    if (e.skill->bonusTemp > 0)
                        ImGui::Text("Temporary bonus: +%u", e.skill->bonusTemp);
                    ImGui::Text("Max: %u", maxVal);
                    ImGui::EndTooltip();
                }

                ImGui::PopID();
            }
            ImGui::Spacing();
        }
    }

    ImGui::EndChild();
    ImGui::End();
}





} // namespace ui
} // namespace wowee
