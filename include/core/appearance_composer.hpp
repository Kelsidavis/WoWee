#pragma once

#include "core/geoset_rules.hpp"
#include "game/character.hpp"
#include <string>
#include <vector>
#include <unordered_set>
#include <cstdint>

namespace wowee {

namespace rendering { class Renderer; }
namespace pipeline { class AssetManager; class DBCLayout; struct M2Model; }
namespace game { class GameHandler; class Inventory; }

namespace core {

class EntitySpawner;

/// Resolved texture paths from CharSections.dbc for player character compositing.
struct PlayerTextureInfo {
    std::string bodySkinPath;
    std::string faceLowerPath;
    std::string faceUpperPath;
    std::string hairTexturePath;
    /// CharSections' second texture on the skin row - the "Extra" art an HD
    /// character model asks for as texture type 8. Empty on the stock models,
    /// which have no type-8 texture and never look for one.
    std::string skinExtraPath;
    std::vector<std::string> underwearPaths;
};

/// Handles player character visual appearance: skin compositing, geoset selection,
/// texture path lookups, and equipment weapon rendering.
class AppearanceComposer {
public:
    AppearanceComposer(rendering::Renderer* renderer,
                       pipeline::AssetManager* assetManager,
                       game::GameHandler* gameHandler,
                       EntitySpawner* entitySpawner);

    // Player model path resolution

    // Resolve texture paths from CharSections.dbc and fill model texture slots.
    // Call BEFORE charRenderer->loadModel().
    /// `useFemaleModel` is the body a nonbinary character chose. The skin
    /// textures are picked per sex, so a female body reading male skins is
    /// the same mismatch the model path had.
    PlayerTextureInfo resolvePlayerTextures(pipeline::M2Model& model,
                                            game::Race race, game::Gender gender,
                                            uint32_t appearanceBytes,
                                            bool useFemaleModel = false);

    // Apply composited textures to loaded model instance.
    // Call AFTER charRenderer->loadModel(). Saves skin state for re-compositing.
    void compositePlayerSkin(uint32_t modelSlotId, const PlayerTextureInfo& texInfo);

    // Build default active geosets for player character
    std::unordered_set<uint16_t> buildDefaultPlayerGeosets(uint8_t raceId, uint8_t sexId,
                                                           uint8_t hairStyleId, uint8_t facialId);

    // Equipment weapon loading (reads inventory, attaches weapon M2 models)
    void loadEquippedWeapons();

    // Weapon sheathe state
    void setWeaponsSheathed(bool sheathed) { weaponsSheathed_ = sheathed; }
    [[nodiscard]] bool isWeaponsSheathed() const { return weaponsSheathed_; }
    void toggleWeaponsSheathed() { weaponsSheathed_ = !weaponsSheathed_; }

    // Ranged weapon swap: temporarily show ranged weapon in right hand
    void showRangedWeapon(bool show);
    [[nodiscard]] bool isShowingRanged() const { return showingRanged_; }

    // Mining casts temporarily replace the held main-hand model with a pickaxe.
    void showMiningPick(bool show);

    // Fishing casts temporarily replace the held main-hand model with a pole
    // found in the player's equipped slots or bags.
    void showFishingPole(bool show);

    // Saved skin state accessors (used by game_screen.cpp for equipment re-compositing)
    [[nodiscard]] const std::string& getBodySkinPath() const { return bodySkinPath_; }
    [[nodiscard]] const std::vector<std::string>& getUnderwearPaths() const { return underwearPaths_; }
    [[nodiscard]] uint32_t getSkinTextureSlotIndex() const { return skinTextureSlotIndex_; }
    [[nodiscard]] uint32_t getCloakTextureSlotIndex() const { return cloakTextureSlotIndex_; }

private:
    bool loadWeaponM2(const std::string& m2Path, pipeline::M2Model& outModel);

    /// Attach the equipped head item's model. Other players resolve this through
    /// EntitySpawner; the local character had no equivalent at all.
    void loadEquippedHelm(game::Inventory& inventory);

    // Attach the enchant visual (sharpening-stone glint, weapon glow) of the item in
    // the given equipment slot to the weapon already attached at attachmentId.
    void applyEnchantVisuals(uint32_t charInstanceId, int equipSlotIndex, uint32_t attachmentId);

    rendering::Renderer* renderer_;
    pipeline::AssetManager* assetManager_;
    game::GameHandler* gameHandler_;
    EntitySpawner* entitySpawner_;

    // Saved at spawn for skin re-compositing on equipment changes
    std::string bodySkinPath_;
    std::vector<std::string> underwearPaths_;
    uint32_t skinTextureSlotIndex_ = 0;
    uint32_t cloakTextureSlotIndex_ = 0;

    bool weaponsSheathed_ = false;
    bool showingRanged_ = false;
    bool showingMiningPick_ = false;
    uint32_t miningPickInstanceId_ = 0;
    bool showingFishingPole_ = false;
};

} // namespace core
} // namespace wowee
