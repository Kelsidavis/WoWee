#pragma once

#include "game/character.hpp"
#include "game/world_packets.hpp"
#include "ui/paper_ui.hpp"
#include "ui/text_edit.hpp"
#include <imgui.h>
#include <string>
#include <functional>
#include <vector>
#include <memory>
#include <cstdint>

namespace wowee {
namespace game { class GameHandler; }
namespace pipeline { class AssetManager; }
namespace rendering { class CharacterPreview; }

namespace ui {

class CharacterCreateScreen {
public:
    CharacterCreateScreen();
    ~CharacterCreateScreen();

    void render(game::GameHandler& gameHandler);
    void update(float deltaTime);
    void setOnCreate(std::function<void(const game::CharCreateData&)> cb) { onCreate = std::move(cb); }
    void setOnCancel(std::function<void()> cb) { onCancel = std::move(cb); }
    void setStatus(const std::string& msg, bool isError = false);
    void reset();
    void initializePreview(pipeline::AssetManager* am);

    /** Set allowed races/classes from expansion profile. Empty = allow all (WotLK default). */
    void setExpansionConstraints(const std::vector<uint32_t>& races, const std::vector<uint32_t>& classes);

private:
    /// WoW allows twelve characters in a name. See paper_ui.hpp for why this
    /// screen draws its own box around them rather than asking ImGui for one.
    TextEdit name_{12};
    /// The controls, and the frame-to-frame state behind them.
    PaperUI ui_;
    int raceIndex = 0;
    int classIndex = 0;
    int genderIndex = 0;
    int bodyTypeIndex = 0;  // For nonbinary: 0=masculine, 1=feminine
    int skin = 0, face = 0, hairStyle = 0, hairColor = 0, facialHair = 0;
    int maxSkin = 9, maxFace = 9, maxHairStyle = 11, maxHairColor = 9, maxFacialHair = 8;
    std::string statusMessage;
    bool statusIsError = false;

    // Character customization IDs are DBC keys, not guaranteed contiguous ranges.
    // The sliders hold indices into these vectors; preview and create packets use
    // the corresponding real IDs.
    std::vector<uint8_t> skinIds_;
    std::vector<uint8_t> faceIds_;
    std::vector<uint8_t> hairStyleIds_;
    std::vector<uint8_t> hairColorIds_;
    std::vector<uint8_t> facialHairIds_;

    std::vector<game::Class> availableClasses;
    void updateAvailableClasses();

    // Expansion-filtered race/class lists
    std::vector<game::Race> availableRaces_;    // Alliance-first, then horde order
    int allianceRaceCount_ = 0;                 // How many of availableRaces_ are alliance
    std::vector<game::Class> expansionClasses_; // Allowed classes (empty = all)

    std::function<void(const game::CharCreateData&)> onCreate;
    std::function<void()> onCancel;

    // 3D model preview
    std::unique_ptr<rendering::CharacterPreview> preview_;
    pipeline::AssetManager* assetManager_ = nullptr;
    int prevRaceIndex_ = -1;
    int prevGenderIndex_ = -1;
    int prevBodyTypeIndex_ = -1;
    int prevSkin_ = -1;
    int prevFace_ = -1;
    int prevHairStyle_ = -1;
    int prevHairColor_ = -1;
    int prevFacialHair_ = -1;
    int prevRangeRace_ = -1;
    int prevRangeGender_ = -1;
    int prevRangeBodyType_ = -1;
    int prevRangeSkin_ = -1;
    int prevRangeHairStyle_ = -1;
    float createTimer_ = -1.0f;  // >=0 while waiting for SMSG_CHAR_CREATE response

    void updatePreviewIfNeeded();
    void updateAppearanceRanges();
};

} // namespace ui
} // namespace wowee
