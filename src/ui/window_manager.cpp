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

    constexpr auto& kColorGray        = kGray;
    constexpr auto& kColorDarkGray    = kDarkGray;

} // anonymous namespace

namespace wowee {
namespace ui {

WindowManager::~WindowManager() = default;









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
        // Out of the chair with the preview still on them - closed by the
        // server, walked away from, or escaped out of. Put back what they came
        // in wearing; a style is only theirs once it is paid for.
        if (barberPreviewActive_) {
            barberPreviewActive_ = false;
            gameHandler.previewPlayerAppearance(barberOrigAppearanceBytes_,
                                                barberOrigFacialHair_);
        }
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
    barberOrigAppearanceBytes_ = ch->appearanceBytes;
    barberPreviewActive_ = false;
    barberOrigSkinColor_ = static_cast<uint8_t>(ch->appearanceBytes & 0xFF);
    barberOrigHairStyle_ = static_cast<uint8_t>((ch->appearanceBytes >> 16) & 0xFF);
    barberOrigHairColor_ = static_cast<uint8_t>((ch->appearanceBytes >> 24) & 0xFF);
    barberOrigFacialHair_ = ch->facialFeatures;
    barberHairStyles_.clear();
    barberFacialStyles_.clear();
    barberSkinStyles_.clear();
    // Entry zero tells the server to retain the existing skin.
    barberSkinStyles_.push_back({.entryId = 0, .appearanceId = barberOrigSkinColor_, .name = "Current"});

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
                BarberStyleOption option{.entryId = dbc->getUInt32(row, 0),
                                         .appearanceId = static_cast<uint8_t>(appearance), .name = std::move(name)};
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
    // And put the character back the way they walked in, since the preview has
    // been changing them as the selectors moved.
    barberPreview(gameHandler);
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
    barberPreview(gameHandler);
}

/// Put the current selection on the character standing in the shop.
///
/// The whole of the barber shop is a preview - WoW shows the change on the
/// character itself and charges for it when Accept is pressed - and this was
/// the missing half: the selectors named a style and cycling them changed a
/// number, with nothing to see. Nothing is sent by this; barberApplySelection
/// is still the only thing that talks to the server.
void WindowManager::barberPreview(game::GameHandler& gameHandler) {
    const auto* ch = gameHandler.getActiveCharacter();
    if (!ch) return;
    const BarberSelection sel = barberSelection(gameHandler);
    // Skin, face, hair style and hair colour, one byte each, in the order
    // PLAYER_BYTES packs them. The face is not the barber's to change, so it is
    // kept from what is already there.
    const uint32_t bytes = static_cast<uint32_t>(sel.skin)
                         | (ch->appearanceBytes & 0x0000FF00u)
                         | (static_cast<uint32_t>(sel.hairStyle) << 16)
                         | (static_cast<uint32_t>(sel.hairColor) << 24);
    barberPreviewActive_ = bytes != barberOrigAppearanceBytes_ ||
                           sel.facialHair != barberOrigFacialHair_;
    gameHandler.previewPlayerAppearance(bytes, sel.facialHair);
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
        byCategory[cat].push_back({.skillId = id, .skill = &sk});
    }

    static constexpr struct { uint32_t cat; const char* label; } kCatOrder[] = {
        {.cat = 11, .label = "Professions"},
        { .cat = 9, .label = "Secondary Skills"},
        { .cat = 7, .label = "Class Skills"},
        { .cat = 6, .label = "Weapon Skills"},
        { .cat = 8, .label = "Armor"},
        { .cat = 5, .label = "Languages"},
        { .cat = 0, .label = "Other"},
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
