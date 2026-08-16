#include "ui/character_create_screen.hpp"
#include "ui/selection_screen_layout.hpp"
#include "core/logger.hpp"
#include "ui/ui_colors.hpp"
#include "rendering/character_preview.hpp"
#include "rendering/renderer.hpp"
#include "core/application.hpp"
#include "game/game_handler.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_layout.hpp"
#include <imgui.h>
#include <cstring>
#include <algorithm>

namespace wowee {
namespace ui {

// Full WotLK race/class lists (used as defaults when no expansion constraints set)
static constexpr game::Race kAllRaces[] = {
    // Alliance
    game::Race::HUMAN, game::Race::DWARF, game::Race::NIGHT_ELF,
    game::Race::GNOME, game::Race::DRAENEI,
    // Horde
    game::Race::ORC, game::Race::UNDEAD, game::Race::TAUREN,
    game::Race::TROLL, game::Race::BLOOD_ELF,
};
static constexpr int kAllRaceCount = 10;
static constexpr int kAllianceCount = 5;

static constexpr game::Class kAllClasses[] = {
    game::Class::WARRIOR, game::Class::PALADIN, game::Class::HUNTER,
    game::Class::ROGUE, game::Class::PRIEST, game::Class::DEATH_KNIGHT,
    game::Class::SHAMAN, game::Class::MAGE, game::Class::WARLOCK,
    game::Class::DRUID,
};

namespace {

uint8_t selectedAppearanceId(const std::vector<uint8_t>& ids, int index) {
    if (!ids.empty() && index >= 0 && index < static_cast<int>(ids.size())) {
        return ids[static_cast<size_t>(index)];
    }
    return static_cast<uint8_t>(std::max(index, 0));
}

void sortUnique(std::vector<uint8_t>& ids) {
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
}

// Last resort when the DBC scan found nothing for this race and sex. The
// numbers it invents are not backed by any CharSections row, so a character
// created from them has an appearance the client cannot draw - a face picked
// this way never resolves to a texture and the head renders bare. Keeping the
// fallback (an empty slider is worse than a wrong one) but saying so, because
// silently offering unrenderable choices is how a character ends up broken for
// good at the one moment it cannot be changed.
void useFallbackRange(const char* what, std::vector<uint8_t>& ids, int maxId) {
    if (!ids.empty()) return;
    core::Logger::getInstance().warning(
        "Character creation: no DBC entries for ", what,
        " - offering an unverified 0..", maxId,
        " range; choices may not render");
    ids.reserve(static_cast<size_t>(maxId + 1));
    for (int id = 0; id <= maxId; ++id) {
        ids.push_back(static_cast<uint8_t>(id));
    }
}

} // namespace


CharacterCreateScreen::CharacterCreateScreen() {
    reset();
}

CharacterCreateScreen::~CharacterCreateScreen() = default;

void CharacterCreateScreen::setExpansionConstraints(
        const std::vector<uint32_t>& races, const std::vector<uint32_t>& classes) {
    // Build filtered race list: alliance first, then horde
    availableRaces_.clear();
    expansionClasses_.clear();

    if (!races.empty()) {
        // Alliance races in display order
        for (auto r : std::initializer_list<game::Race>{
                game::Race::HUMAN, game::Race::DWARF, game::Race::NIGHT_ELF,
                game::Race::GNOME, game::Race::DRAENEI}) {
            if (std::find(races.begin(), races.end(), static_cast<uint32_t>(r)) != races.end()) {
                availableRaces_.push_back(r);
            }
        }
        allianceRaceCount_ = static_cast<int>(availableRaces_.size());

        // Horde races in display order
        for (auto r : std::initializer_list<game::Race>{
                game::Race::ORC, game::Race::UNDEAD, game::Race::TAUREN,
                game::Race::TROLL, game::Race::BLOOD_ELF}) {
            if (std::find(races.begin(), races.end(), static_cast<uint32_t>(r)) != races.end()) {
                availableRaces_.push_back(r);
            }
        }
    }

    if (!classes.empty()) {
        for (auto cls : kAllClasses) {
            if (std::find(classes.begin(), classes.end(), static_cast<uint32_t>(cls)) != classes.end()) {
                expansionClasses_.push_back(cls);
            }
        }
    }

    // If no constraints provided, fall back to WotLK defaults
    if (availableRaces_.empty()) {
        availableRaces_.assign(kAllRaces, kAllRaces + kAllRaceCount);
        allianceRaceCount_ = kAllianceCount;
    }

    raceIndex = 0;
    classIndex = 0;
    updateAvailableClasses();
}

void CharacterCreateScreen::reset() {
    std::memset(nameBuffer, 0, sizeof(nameBuffer));
    raceIndex = 0;
    classIndex = 0;
    genderIndex = 0;
    bodyTypeIndex = 0;
    skin = 0;
    face = 0;
    hairStyle = 0;
    hairColor = 0;
    facialHair = 0;
    // The same limits game::getMax* answers, asked rather than repeated: this
    // is the fallback before the DBC scan has run, and the two have to agree
    // or the sliders offer a face the scan will not produce.
    const game::Race race = availableRaces_.empty()
        ? game::Race::HUMAN : availableRaces_[0];
    const game::Gender gender = game::Gender::MALE;
    maxSkin = game::getMaxSkin(race, gender);
    maxFace = game::getMaxFace(race, gender);
    maxHairStyle = game::getMaxHairStyle(race, gender);
    maxHairColor = game::getMaxHairColor(race, gender);
    maxFacialHair = game::getMaxFacialFeature(race, gender);
    statusMessage.clear();
    statusIsError = false;
    createTimer_ = -1.0f;
    skinIds_.clear();
    faceIds_.clear();
    hairStyleIds_.clear();
    hairColorIds_.clear();
    facialHairIds_.clear();

    // Populate default races if not yet set by setExpansionConstraints
    if (availableRaces_.empty()) {
        availableRaces_.assign(kAllRaces, kAllRaces + kAllRaceCount);
        allianceRaceCount_ = kAllianceCount;
    }

    updateAvailableClasses();

    // Reset preview tracking to force model reload on next render
    prevRaceIndex_ = -1;
    prevGenderIndex_ = -1;
    prevBodyTypeIndex_ = -1;
    prevSkin_ = -1;
    prevFace_ = -1;
    prevHairStyle_ = -1;
    prevHairColor_ = -1;
    prevFacialHair_ = -1;
    prevRangeRace_ = -1;
    prevRangeGender_ = -1;
    prevRangeBodyType_ = -1;
    prevRangeSkin_ = -1;
    prevRangeHairStyle_ = -1;
}

void CharacterCreateScreen::initializePreview(pipeline::AssetManager* am) {
    assetManager_ = am;
    if (!preview_) {
        preview_ = std::make_unique<rendering::CharacterPreview>();
        if (preview_->initialize(am)) {
            auto* renderer = core::Application::getInstance().getRenderer();
            if (renderer) renderer->registerPreview(preview_.get());
        }
    }
    if (preview_) preview_->resetView();
    // Force model reload
    prevRaceIndex_ = -1;
}

void CharacterCreateScreen::update(float deltaTime) {
    if (preview_) {
        preview_->update(deltaTime);
    }
    // Timeout waiting for server response
    if (createTimer_ >= 0.0f) {
        createTimer_ += deltaTime;
        if (createTimer_ > 10.0f) {
            createTimer_ = -1.0f;
            setStatus("Server did not respond. Try again.", true);
        }
    }
}

void CharacterCreateScreen::setStatus(const std::string& msg, bool isError) {
    statusMessage = msg;
    statusIsError = isError;
    if (isError || msg.empty()) {
        createTimer_ = -1.0f;  // Stop waiting on error/clear
    }
}

void CharacterCreateScreen::updateAvailableClasses() {
    availableClasses.clear();
    if (availableRaces_.empty() || raceIndex >= static_cast<int>(availableRaces_.size())) return;
    game::Race race = availableRaces_[raceIndex];
    for (auto cls : kAllClasses) {
        if (!game::isValidRaceClassCombo(race, cls)) continue;
        // If expansion constraints set, only allow listed classes
        if (!expansionClasses_.empty()) {
            if (std::find(expansionClasses_.begin(), expansionClasses_.end(), cls) == expansionClasses_.end())
                continue;
        }
        availableClasses.push_back(cls);
    }
    // Clamp class index
    if (classIndex >= static_cast<int>(availableClasses.size())) {
        classIndex = 0;
    }
}

void CharacterCreateScreen::updatePreviewIfNeeded() {
    if (!preview_) return;

    bool changed = (raceIndex != prevRaceIndex_ ||
                    genderIndex != prevGenderIndex_ ||
                    bodyTypeIndex != prevBodyTypeIndex_ ||
                    skin != prevSkin_ ||
                    face != prevFace_ ||
                    hairStyle != prevHairStyle_ ||
                    hairColor != prevHairColor_ ||
                    facialHair != prevFacialHair_);

    if (changed) {
        bool useFemaleModel = (genderIndex == 2 && bodyTypeIndex == 1);  // Nonbinary + Feminine
        preview_->loadCharacter(
            availableRaces_[raceIndex],
            static_cast<game::Gender>(genderIndex),
            selectedAppearanceId(skinIds_, skin),
            selectedAppearanceId(faceIds_, face),
            selectedAppearanceId(hairStyleIds_, hairStyle),
            selectedAppearanceId(hairColorIds_, hairColor),
            selectedAppearanceId(facialHairIds_, facialHair),
            useFemaleModel);

        prevRaceIndex_ = raceIndex;
        prevGenderIndex_ = genderIndex;
        prevBodyTypeIndex_ = bodyTypeIndex;
        prevSkin_ = skin;
        prevFace_ = face;
        prevHairStyle_ = hairStyle;
        prevHairColor_ = hairColor;
        prevFacialHair_ = facialHair;
    }
}

void CharacterCreateScreen::updateAppearanceRanges() {
    if (raceIndex == prevRangeRace_ &&
        genderIndex == prevRangeGender_ &&
        bodyTypeIndex == prevRangeBodyType_ &&
        skin == prevRangeSkin_ &&
        hairStyle == prevRangeHairStyle_) {
        return;
    }

    // The fallback ranges, before the DBC scan narrows them to what this race
    // and sex actually have art for.
    const game::Race race = raceIndex < static_cast<int>(availableRaces_.size())
        ? availableRaces_[raceIndex] : game::Race::HUMAN;
    const game::Gender gender =
        genderIndex == 0 ? game::Gender::MALE : game::Gender::FEMALE;
    maxSkin = game::getMaxSkin(race, gender);
    maxFace = game::getMaxFace(race, gender);
    maxHairStyle = game::getMaxHairStyle(race, gender);
    maxHairColor = game::getMaxHairColor(race, gender);
    maxFacialHair = game::getMaxFacialFeature(race, gender);
    skinIds_.clear();
    faceIds_.clear();
    hairStyleIds_.clear();
    hairColorIds_.clear();
    facialHairIds_.clear();

    if (!assetManager_ || availableRaces_.empty()) return;
    auto dbc = assetManager_->loadDBC("CharSections.dbc");
    if (!dbc) return;

    uint32_t targetRaceId = static_cast<uint32_t>(availableRaces_[raceIndex]);
    const bool useFemaleModel = genderIndex == 1 || (genderIndex == 2 && bodyTypeIndex == 1);
    uint32_t targetSexId = useFemaleModel ? 1u : 0u;

    const auto* csL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("CharSections") : nullptr;
    auto csF = pipeline::detectCharSectionsFields(dbc.get(), csL);
    for (uint32_t r = 0; r < dbc->getRecordCount(); r++) {
        uint32_t raceId = dbc->getUInt32(r, csF.raceId);
        uint32_t sexId = dbc->getUInt32(r, csF.sexId);
        if (raceId != targetRaceId || sexId != targetSexId) continue;

        uint32_t baseSection = dbc->getUInt32(r, csF.baseSection);
        uint32_t variationIndex = dbc->getUInt32(r, csF.variationIndex);
        uint32_t colorIndex = dbc->getUInt32(r, csF.colorIndex);

        if (baseSection == 0 && variationIndex == 0 && colorIndex <= 255) {
            skinIds_.push_back(static_cast<uint8_t>(colorIndex));
        } else if (baseSection == 3 && variationIndex <= 255) {
            hairStyleIds_.push_back(static_cast<uint8_t>(variationIndex));
        }
    }

    sortUnique(skinIds_);
    sortUnique(hairStyleIds_);
    useFallbackRange("skin", skinIds_, maxSkin);
    useFallbackRange("hair style", hairStyleIds_, maxHairStyle);
    maxSkin = static_cast<int>(skinIds_.size()) - 1;
    maxHairStyle = static_cast<int>(hairStyleIds_.size()) - 1;
    skin = std::clamp(skin, 0, maxSkin);
    hairStyle = std::clamp(hairStyle, 0, maxHairStyle);

    const uint8_t skinId = selectedAppearanceId(skinIds_, skin);
    const uint8_t hairStyleId = selectedAppearanceId(hairStyleIds_, hairStyle);

    for (uint32_t r = 0; r < dbc->getRecordCount(); r++) {
        uint32_t raceId = dbc->getUInt32(r, csF.raceId);
        uint32_t sexId = dbc->getUInt32(r, csF.sexId);
        if (raceId != targetRaceId || sexId != targetSexId) continue;

        uint32_t baseSection = dbc->getUInt32(r, csF.baseSection);
        uint32_t variationIndex = dbc->getUInt32(r, csF.variationIndex);
        uint32_t colorIndex = dbc->getUInt32(r, csF.colorIndex);

        if (baseSection == 1 && colorIndex == skinId && variationIndex <= 255) {
            faceIds_.push_back(static_cast<uint8_t>(variationIndex));
        } else if (baseSection == 3 && variationIndex == hairStyleId) {
            if (colorIndex <= 255) {
                hairColorIds_.push_back(static_cast<uint8_t>(colorIndex));
            }
        }
    }

    sortUnique(faceIds_);
    sortUnique(hairColorIds_);
    useFallbackRange("face", faceIds_, maxFace);
    useFallbackRange("hair colour", hairColorIds_, maxHairColor);
    maxFace = static_cast<int>(faceIds_.size()) - 1;
    maxHairColor = static_cast<int>(hairColorIds_.size()) - 1;
    face = std::clamp(face, 0, maxFace);
    hairColor = std::clamp(hairColor, 0, maxHairColor);

    auto facialDbc = assetManager_->loadDBC("CharacterFacialHairStyles.dbc");
    const auto* fhL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("CharacterFacialHairStyles") : nullptr;
    if (facialDbc) {
        for (uint32_t r = 0; r < facialDbc->getRecordCount(); r++) {
            uint32_t raceId = facialDbc->getUInt32(r, fhL ? (*fhL)["RaceID"] : 0);
            uint32_t sexId = facialDbc->getUInt32(r, fhL ? (*fhL)["SexID"] : 1);
            if (raceId != targetRaceId || sexId != targetSexId) continue;
            uint32_t variation = facialDbc->getUInt32(r, fhL ? (*fhL)["Variation"] : 2);
            if (variation <= 255) {
                facialHairIds_.push_back(static_cast<uint8_t>(variation));
            }
        }
    }
    sortUnique(facialHairIds_);
    useFallbackRange("facial hair", facialHairIds_, targetSexId == 1 ? 0 : maxFacialHair);
    maxFacialHair = static_cast<int>(facialHairIds_.size()) - 1;
    facialHair = std::clamp(facialHair, 0, maxFacialHair);

    prevRangeRace_ = raceIndex;
    prevRangeGender_ = genderIndex;
    prevRangeBodyType_ = bodyTypeIndex;
    prevRangeSkin_ = skin;
    prevRangeHairStyle_ = hairStyle;
}

void CharacterCreateScreen::render(game::GameHandler& /*gameHandler*/) {
    // Resolve valid DBC option IDs before loading the preview. Doing this after
    // loadCharacter left race/style changes using the previous option mapping.
    updateAppearanceRanges();

    // Render the preview to FBO before the ImGui frame
    if (preview_) {
        updatePreviewIfNeeded();
        preview_->render();
        preview_->requestComposite();
    }

    bool hasPreview = (preview_ && preview_->getTextureId() != nullptr);
    ImGuiViewport* vp = ImGui::GetMainViewport();
    const SelectionScreenLayout layout = makeSelectionScreenLayout(*vp);

    ImGui::SetNextWindowPos(layout.windowPos, ImGuiCond_Always);
    ImGui::SetNextWindowSize(layout.windowSize, ImGuiCond_Always);

    ImGui::Begin("Create Character", nullptr,
        ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);
    ImGui::SetWindowFontScale(layout.scale);

    ImGui::TextColored(ui::colors::kWarmGold, "Create a Character");
    ImGui::TextDisabled("Choose your hero's identity and appearance.");
    ImGui::Separator();

    const float bodyHeight = std::max(
        320.0f,
        ImGui::GetContentRegionAvail().y - layout.footerHeight() -
            ImGui::GetStyle().ItemSpacing.y);
    const float availableWidth = ImGui::GetContentRegionAvail().x;
    const float previewWidth = hasPreview
        ? std::min(680.0f * layout.scale,
                   std::max(360.0f * layout.scale, availableWidth * 0.44f))
        : 0.0f;

    if (hasPreview) {
        // Left panel: 3D preview
        ImGui::BeginChild("##preview_panel", ImVec2(previewWidth, bodyHeight), true);
        {
            float imgW = ImGui::GetContentRegionAvail().x;
            float imgH = imgW * (static_cast<float>(preview_->getHeight()) /
                                  static_cast<float>(preview_->getWidth()));
            const float maxImageH = std::max(200.0f, bodyHeight - 42.0f * layout.scale);
            if (imgH > maxImageH) {
                imgH = maxImageH;
                imgW = imgH * (static_cast<float>(preview_->getWidth()) /
                               static_cast<float>(preview_->getHeight()));
            }

            const float indent = (ImGui::GetContentRegionAvail().x - imgW) * 0.5f;
            if (indent > 0.0f) ImGui::SetCursorPosX(ImGui::GetCursorPosX() + indent);

            if (preview_->getTextureId()) {
                ImGui::Image(
                    reinterpret_cast<ImTextureID>(preview_->getTextureId()),
                    ImVec2(imgW, imgH));
            }

            const bool previewHovered = ImGui::IsItemHovered();

            // Mouse drag rotation and hover-only wheel zoom on the preview image.
            if (previewHovered && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
                float deltaX = ImGui::GetIO().MouseDelta.x;
                preview_->rotate(deltaX * 0.2f);
            }
            if (previewHovered && ImGui::GetIO().MouseWheel != 0.0f) {
                preview_->zoom(ImGui::GetIO().MouseWheel);
            }

            ImGui::TextDisabled("Drag to rotate  -  Scroll to zoom");
        }
        ImGui::EndChild();

        ImGui::SameLine(0.0f, layout.gap());
    }

    // Right panel: controls remain independently scrollable on short windows.
    ImGui::BeginChild("##controls_panel", ImVec2(0.0f, bodyHeight), true);

    // Name input
    ImGui::Text("Name:");
    ImGui::SameLine(100.0f * layout.scale);
    ImGui::SetNextItemWidth(-1.0f);
    ImGui::InputText("##name", nameBuffer, sizeof(nameBuffer));

    ImGui::Spacing();

    // Race selection (filtered by expansion)
    int raceCount = static_cast<int>(availableRaces_.size());
    ImGui::Text("Race:");
    ImGui::Spacing();
    ImGui::Indent(10.0f * layout.scale);
    auto continueButtonRow = [&](const char* label, bool first) {
        if (first) return;
        const float buttonWidth = ImGui::CalcTextSize(label).x +
                                  ImGui::GetStyle().FramePadding.x * 2.0f;
        const float nextRight = ImGui::GetItemRectMax().x +
                                ImGui::GetStyle().ItemSpacing.x + buttonWidth;
        const float contentRight = ImGui::GetWindowPos().x +
                                   ImGui::GetContentRegionMax().x;
        if (nextRight <= contentRight) ImGui::SameLine();
    };
    if (allianceRaceCount_ > 0) {
        ImGui::TextColored(ImVec4(0.3f, 0.5f, 1.0f, 1.0f), "Alliance:");
        for (int i = 0; i < allianceRaceCount_; ++i) {
            const char* raceName = game::getRaceName(availableRaces_[i]);
            continueButtonRow(raceName, i == 0);
            bool selected = (raceIndex == i);
            if (selected) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.3f, 0.5f, 1.0f, 0.8f));
            if (ImGui::SmallButton(raceName)) {
                if (raceIndex != i) {
                    raceIndex = i;
                    classIndex = 0;
                    skin = face = hairStyle = hairColor = facialHair = 0;
                    updateAvailableClasses();
                }
            }
            if (selected) ImGui::PopStyleColor();
        }
    }
    if (allianceRaceCount_ < raceCount) {
        ImGui::TextColored(ui::colors::kRed, "Horde:");
        for (int i = allianceRaceCount_; i < raceCount; ++i) {
            const char* raceName = game::getRaceName(availableRaces_[i]);
            continueButtonRow(raceName, i == allianceRaceCount_);
            bool selected = (raceIndex == i);
            if (selected) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(1.0f, 0.3f, 0.3f, 0.8f));
            if (ImGui::SmallButton(raceName)) {
                if (raceIndex != i) {
                    raceIndex = i;
                    classIndex = 0;
                    skin = face = hairStyle = hairColor = facialHair = 0;
                    updateAvailableClasses();
                }
            }
            if (selected) ImGui::PopStyleColor();
        }
    }
    ImGui::Unindent(10.0f * layout.scale);

    ImGui::Spacing();

    // Class selection
    ImGui::Text("Class:");
    if (!availableClasses.empty()) {
        ImGui::BeginGroup();
        for (int i = 0; i < static_cast<int>(availableClasses.size()); ++i) {
            const char* className = game::getClassName(availableClasses[i]);
            continueButtonRow(className, i == 0);
            bool selected = (classIndex == i);
            if (selected) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.6f, 0.2f, 0.8f));
            if (ImGui::SmallButton(className)) {
                classIndex = i;
            }
            if (selected) ImGui::PopStyleColor();
        }
        ImGui::EndGroup();
    }

    ImGui::Spacing();

    // Gender
    ImGui::Text("Gender:");
    ImGui::SameLine(100.0f * layout.scale);
    ImGui::RadioButton("Male", &genderIndex, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Female", &genderIndex, 1);

    // TODO(server): Re-enable the nonbinary radio button and body-type controls
    // once character creation accepts gender=2 plus the selected model body.
    // The renderer/data plumbing remains in place so server support can enable it.
    // ImGui::SameLine();
    // ImGui::RadioButton("Nonbinary", &genderIndex, 2);
    // if (genderIndex == 2) {
    //     ImGui::Text("Body Type:");
    //     ImGui::SameLine(80);
    //     ImGui::RadioButton("Masculine", &bodyTypeIndex, 0);
    //     ImGui::SameLine();
    //     ImGui::RadioButton("Feminine", &bodyTypeIndex, 1);
    // }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Appearance sliders
    // Race/body controls above may have changed this frame; refresh their option
    // lists before presenting customization controls.
    updateAppearanceRanges();
    game::Gender currentGender = static_cast<game::Gender>(genderIndex);

    ImGui::Text("Appearance");
    ImGui::Spacing();

    const float labelCol = 120.0f * layout.scale;

    auto slider = [&](const char* label, int* val, int maxVal) {
        ImGui::Text("%s", label);
        ImGui::SameLine(labelCol);
        ImGui::SetNextItemWidth(-1.0f);
        char id[32];
        snprintf(id, sizeof(id), "##%s", label);
        ImGui::SliderInt(id, val, 0, maxVal);
    };

    slider("Skin",           &skin,      maxSkin);
    slider("Face",           &face,      maxFace);
    slider("Hair Style",     &hairStyle, maxHairStyle);
    slider("Hair Color",     &hairColor, maxHairColor);
    slider("Facial Feature", &facialHair, maxFacialHair);

    // Skin and hairstyle choose the valid face/color subsets. Refresh now so a
    // Create click in this same frame sends IDs from the newly selected subset.
    updateAppearanceRanges();

    ImGui::Spacing();

    ImGui::EndChild(); // controls_panel

    // Shared footer: Back remains left, primary Create action remains right.
    ImGui::BeginChild("CharacterCreateFooter", ImVec2(0.0f, layout.footerHeight()), true);
    if (!statusMessage.empty()) {
        ImVec4 color = statusIsError ? ui::colors::kRed : ui::colors::kBrightGreen;
        ImGui::TextColored(color, "%s", statusMessage.c_str());
    } else {
        ImGui::TextDisabled("Names may contain up to 12 characters.");
    }
    ImGui::Spacing();

    if (ImGui::Button("Back", layout.button())) {
        if (onCancel) onCancel();
    }

    const ImVec2 createSize = layout.primaryButton(180.0f);
    ImGui::SameLine(ImGui::GetContentRegionMax().x - createSize.x);
    if (ImGui::Button("Create Character", createSize)) {
        std::string name(nameBuffer);
        // Trim whitespace
        size_t start = name.find_first_not_of(" \t\r\n");
        size_t end = name.find_last_not_of(" \t\r\n");
        if (start == std::string::npos) {
            name.clear();
        } else {
            name = name.substr(start, end - start + 1);
        }
        if (name.empty()) {
            setStatus("Please enter a character name.", true);
        } else if (availableClasses.empty()) {
            setStatus("No valid class for this race.", true);
        } else {
            setStatus("Creating character...", false);
            createTimer_ = 0.0f;
            game::CharCreateData data;
            data.name = name;
            data.race = availableRaces_[raceIndex];
            data.characterClass = availableClasses[classIndex];
            data.gender = currentGender;
            data.useFemaleModel = (genderIndex == 2 && bodyTypeIndex == 1);  // Nonbinary + Feminine
            data.skin = selectedAppearanceId(skinIds_, skin);
            data.face = selectedAppearanceId(faceIds_, face);
            data.hairStyle = selectedAppearanceId(hairStyleIds_, hairStyle);
            data.hairColor = selectedAppearanceId(hairColorIds_, hairColor);
            data.facialHair = selectedAppearanceId(facialHairIds_, facialHair);
            if (onCreate) {
                onCreate(data);
            }
        }
    }

    ImGui::EndChild();

    ImGui::End();
}

} // namespace ui
} // namespace wowee
