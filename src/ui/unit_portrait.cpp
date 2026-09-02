#include "ui/unit_portrait.hpp"
#include "game/equipment_hash.hpp"

#include "core/logger.hpp"
#include "game/game_handler.hpp"
#include "rendering/character_preview.hpp"
#include "rendering/renderer.hpp"

namespace wowee::ui {

namespace {

/// FNV-1a over what actually changes a character's look, so a reload happens
/// when the gear changes and not when a stat does.

} // namespace

UnitPortrait::UnitPortrait() = default;

UnitPortrait::~UnitPortrait() = default;

void UnitPortrait::update(game::GameHandler& gameHandler,
                          pipeline::AssetManager* assets,
                          rendering::Renderer* renderer, float deltaTime) {
    if (!assets || !renderer) return;

    // The character list is where a player's appearance is described; the unit
    // in the world carries a display id, not the pieces it was built from.
    const game::Character* self = nullptr;
    for (const auto& c : gameHandler.getCharacters()) {
        if (c.guid == gameHandler.getPlayerGuid()) { self = &c; break; }
    }
    if (!self) return;

    if (!preview_) {
        preview_ = std::make_unique<rendering::CharacterPreview>();
        // Small: this is drawn into a circle a few dozen pixels across, and the
        // cost of the pass scales with the target.
        initialized_ = preview_->initialize(assets, targetWidth_, targetHeight_);
        if (!initialized_) {
            LOG_WARNING("UnitPortrait: could not build the offscreen view");
            preview_.reset();
            return;
        }
        renderer->registerPreview(preview_.get());
        registered_ = true;
    }

    // What they are wearing now, not what they wore when they logged in.
    //
    // The character list carries the equipment SMSG_CHAR_ENUM described and
    // nothing updates it afterwards, so this portrait kept the gear the
    // character was wearing at the select screen for the whole session. The
    // inventory is the live answer, and it is the same one the target frame
    // gets when this character is the target - the two portraits are of the
    // same person and must not disagree. The login list stands in until the
    // item queries have come back, which is a moment at most.
    std::vector<game::EquipmentItem> worn;
    {
        std::array<uint32_t, 19> displayIds{};
        std::array<uint8_t, 19> invTypes{};
        if (gameHandler.getOtherPlayerEquipment(self->guid, displayIds, invTypes)) {
            for (size_t slot = 0; slot < displayIds.size(); ++slot) {
                if (displayIds[slot] == 0) continue;
                worn.push_back({.displayModel = displayIds[slot],
                                .inventoryType = invTypes[slot],
                                .enchantment = 0u});
            }
        }
        if (worn.empty()) worn = self->equipment;
    }

    const uint64_t equipHash = game::hashEquipmentAppearance(worn);
    const bool changed = (loadedGuid_ != self->guid) ||
                         (loadedAppearance_ != self->appearanceBytes) ||
                         (loadedFacialFeatures_ != self->facialFeatures) ||
                         (loadedEquipHash_ != equipHash);
    if (changed) {
        const uint8_t skin      =  self->appearanceBytes        & 0xFF;
        const uint8_t face      = (self->appearanceBytes >> 8)  & 0xFF;
        const uint8_t hairStyle = (self->appearanceBytes >> 16) & 0xFF;
        const uint8_t hairColor = (self->appearanceBytes >> 24) & 0xFF;

        // Declared before the model loads, so the racial backdrop is never
        // built in the first place.
        preview_->setTransparentBackground(true);
        if (preview_->loadCharacter(self->race, self->gender, skin, face,
                                    hairStyle, hairColor, self->facialFeatures,
                                    self->useFemaleModel)) {
            preview_->applyEquipment(worn);
            // After the model, because its bounds are what the framing is
            // measured against.
            if (framing_ == Framing::Face) {
                preview_->setPortraitFraming();
            } else {
                // The whole figure, facing the viewer. resetView is the
                // character-select framing, which is what a paperdoll wants.
                preview_->resetView();
            }
        }
        // Logged because a portrait that rebuilds every frame looks like one
        // that flickers, and the two are indistinguishable from outside.
        LOG_INFO("UnitPortrait: rebuilt for guid ", self->guid,
                 " appearance ", self->appearanceBytes);
        loadedCreaturePath_.clear();
        loadedGuid_ = self->guid;
        loadedAppearance_ = self->appearanceBytes;
        loadedFacialFeatures_ = self->facialFeatures;
        loadedEquipHash_ = equipHash;
    }

    preview_->update(deltaTime);
    preview_->render();
    preview_->requestComposite();
}

bool UnitPortrait::updatePlayer(uint8_t race, uint8_t gender,
                                uint32_t appearanceBytes, uint8_t facialFeatures,
                                const std::vector<game::EquipmentItem>& equipment,
                                pipeline::AssetManager* assets,
                                rendering::Renderer* renderer, float deltaTime) {
    if (!assets || !renderer) return false;

    if (!preview_) {
        preview_ = std::make_unique<rendering::CharacterPreview>();
        initialized_ = preview_->initialize(assets, targetWidth_, targetHeight_);
        if (!initialized_) {
            LOG_WARNING("UnitPortrait: could not build the offscreen view");
            preview_.reset();
            return false;
        }
        renderer->registerPreview(preview_.get());
        registered_ = true;
    }

    // The same three keys the player's own portrait compares, minus the guid -
    // this is asked per unit and the caller has already decided which.
    const uint64_t equipHash = game::hashEquipmentAppearance(equipment);
    const bool changed = (loadedAppearance_ != appearanceBytes) ||
                         (loadedFacialFeatures_ != facialFeatures) ||
                         (loadedRace_ != race) || (loadedGender_ != gender) ||
                         (loadedEquipHash_ != equipHash) ||
                         (loadedBake_ != pendingBake_) ||
                         !loadedCreaturePath_.empty();
    if (changed) {
        const uint8_t skin      =  appearanceBytes        & 0xFF;
        const uint8_t face      = (appearanceBytes >> 8)  & 0xFF;
        const uint8_t hairStyle = (appearanceBytes >> 16) & 0xFF;
        const uint8_t hairColor = (appearanceBytes >> 24) & 0xFF;

        preview_->setTransparentBackground(true);
        if (preview_->loadCharacter(static_cast<game::Race>(race),
                                    static_cast<game::Gender>(gender),
                                    skin, face, hairStyle, hairColor,
                                    facialFeatures, gender == 1)) {
            // After the model, because applyEquipment reads its geosets, and
            // only where there is something to apply - an empty list is
            // "nothing known yet", and dressing a model in it strips it.
            if (!equipment.empty()) preview_->applyEquipment(equipment);
            // After the equipment, because both write the skin slot and the
            // bake is the more complete answer: it already has the armour on
            // it, which is the half applyEquipment cannot composite for an NPC.
            if (!pendingBake_.empty()) preview_->setBakedSkin(pendingBake_);
            if (framing_ == Framing::Face) preview_->setPortraitFraming();
            else                           preview_->resetView();
        }
        LOG_INFO("UnitPortrait: rebuilt for player race ", static_cast<int>(race),
                 " appearance ", appearanceBytes);
        loadedCreaturePath_.clear();
        loadedGuid_ = 0;
        loadedRace_ = race;
        loadedGender_ = gender;
        loadedAppearance_ = appearanceBytes;
        loadedFacialFeatures_ = facialFeatures;
        loadedEquipHash_ = equipHash;
        loadedBake_ = pendingBake_;
    }

    preview_->update(deltaTime);
    preview_->render();
    preview_->requestComposite();
    return preview_->isModelLoaded();
}

bool UnitPortrait::updateCreature(const std::string& m2Path,
                                  const std::vector<std::pair<uint32_t, std::string>>& skins,
                                  pipeline::AssetManager* assets,
                                  rendering::Renderer* renderer,
                                  float deltaTime) {
    if (!assets || !renderer || m2Path.empty()) return false;

    if (!preview_) {
        preview_ = std::make_unique<rendering::CharacterPreview>();
        initialized_ = preview_->initialize(assets, targetWidth_, targetHeight_);
        if (!initialized_) {
            LOG_WARNING("UnitPortrait: could not build the offscreen view");
            preview_.reset();
            return false;
        }
        renderer->registerPreview(preview_.get());
        registered_ = true;
    }

    if (loadedCreaturePath_ != m2Path) {
        // Declared before the model loads, so the racial backdrop is never
        // built in the first place - the same order loadCharacter needs.
        preview_->setTransparentBackground(true);
        if (preview_->loadCreature(m2Path, skins)) {
            if (framing_ == Framing::Face) {
                preview_->setPortraitFraming();
            } else {
                preview_->resetView();
            }
        }
        LOG_INFO("UnitPortrait: rebuilt for creature ", m2Path);
        loadedCreaturePath_ = m2Path;
        // A player and a creature share one preview, so loading either has to
        // forget what the other was, or switching back would find nothing
        // changed and keep drawing the wrong one.
        loadedGuid_ = 0;
        loadedAppearance_ = 0;
        loadedFacialFeatures_ = 0;
        loadedEquipHash_ = 0;
    }

    preview_->update(deltaTime);
    preview_->render();
    preview_->requestComposite();
    return preview_->isModelLoaded();
}

uint64_t UnitPortrait::textureId() const {
    if (!preview_) return 0;
    return reinterpret_cast<uint64_t>(preview_->getTextureId());
}

void UnitPortrait::rotate(float yawDelta) {
    if (preview_ && yawDelta != 0.0f) preview_->rotate(yawDelta);
}

void UnitPortrait::shutdown(rendering::Renderer* renderer) {
    if (preview_ && registered_ && renderer) renderer->unregisterPreview(preview_.get());
    registered_ = false;
    preview_.reset();
    initialized_ = false;
}

} // namespace wowee::ui
