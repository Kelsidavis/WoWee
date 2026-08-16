#include "core/appearance_composer.hpp"
#include "core/geoset_rules.hpp"
#include "pipeline/item_textures.hpp"
#include "pipeline/m2_asset_loader.hpp"
#include "core/character_paths.hpp"
#include "core/helm_visual.hpp"
#include "core/entity_spawner.hpp"
#include "core/logger.hpp"
#include "rendering/renderer.hpp"
#include "rendering/character_renderer.hpp"
#include "rendering/animation_controller.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/m2_loader.hpp"
#include "pipeline/dbc_loader.hpp"
#include "pipeline/char_sections.hpp"
#include "pipeline/dbc_layout.hpp"
#include "game/game_handler.hpp"
#include <glm/gtc/matrix_transform.hpp>

namespace wowee {
namespace core {

namespace {

constexpr uint32_t kAttachShield = 0;
// M2 attachment 11 is the helm; 0 is the shield mount, which is where head
// gear was going - attached successfully, on the forearm, invisible on the head.
constexpr uint32_t kAttachRightHand = 1;
constexpr uint32_t kAttachLeftHand = 2;
constexpr uint32_t kAttachRightHip = 9;
constexpr uint32_t kAttachLeftHip = 10;
constexpr uint32_t kAttachBack = 12;

uint32_t weaponAttachment(bool sheathed, game::EquipSlot slot, uint8_t inventoryType) {
    if (!sheathed) {
        return slot == game::EquipSlot::OFF_HAND ? kAttachLeftHand : kAttachRightHand;
    }

    if (inventoryType == game::InvType::TWO_HAND) return kAttachBack;
    if (inventoryType == game::InvType::SHIELD) return kAttachShield;
    if (inventoryType == game::InvType::ONE_HAND ||
        inventoryType == game::InvType::MAIN_HAND) {
        return slot == game::EquipSlot::OFF_HAND ? kAttachLeftHip : kAttachRightHip;
    }

    // Holdables and other items with no sheath position are hidden, matching
    // the original client rather than pinning books/orbs to an arbitrary bone.
    return UINT32_MAX;
}

glm::mat4 weaponLocalTransform(bool sheathed, game::EquipSlot /*slot*/,
                               uint8_t inventoryType) {
    glm::mat4 transform(1.0f);
    if (!sheathed || inventoryType == game::InvType::SHIELD) return transform;

    if (inventoryType == game::InvType::TWO_HAND) {
        // Weapon models are authored for a hand with their long axis pointing
        // forward. Stand that axis up, cant it across the back, and move it off
        // the spine so the grip sits below the opposite shoulder.
        // Weapon models are authored along local X, which is also the character's
        // front/back axis. First rotate weapon X completely onto character Z, then
        // cant that vertical axis within the Y/Z back plane. This ordering is
        // important: rotating around X cannot change an X-aligned blade.
        // The final innermost roll spins the blade about its own long axis so
        // the flat rests against the back instead of the sharp edge.
        // Values tuned against the live attachment frame. Its Z axis moves the
        // weapon laterally rather than vertically.
        transform = glm::translate(transform, glm::vec3(-0.03f, -0.10f, 0.0f));
        transform = glm::rotate(transform, glm::radians(33.0f), glm::vec3(1, 0, 0));
        transform = glm::rotate(transform, glm::radians(90.0f), glm::vec3(0, 1, 0));
        transform = glm::rotate(transform, glm::radians(90.0f), glm::vec3(1, 0, 0));
    } else {
        // Hip-sheathed one-handers have the same X-aligned long axis. Rotate it
        // onto -Z so the blade points down alongside the leg.
        transform = glm::rotate(transform, glm::radians(90.0f), glm::vec3(0, 1, 0));
    }
    return transform;
}

} // namespace

AppearanceComposer::AppearanceComposer(rendering::Renderer* renderer,
                                       pipeline::AssetManager* assetManager,
                                       game::GameHandler* gameHandler,
                                       EntitySpawner* entitySpawner)
    : renderer_(renderer)
    , assetManager_(assetManager)
    , gameHandler_(gameHandler)
    , entitySpawner_(entitySpawner)
{
}

PlayerTextureInfo AppearanceComposer::resolvePlayerTextures(pipeline::M2Model& model,
                                                            game::Race race, game::Gender gender,
                                                            uint32_t appearanceBytes,
                                                            bool useFemaleModel) {
    PlayerTextureInfo result;

    uint32_t targetRaceId = static_cast<uint32_t>(race);
    // The same rule the model path uses: a nonbinary character wears the body
    // they chose, and the skins have to be that body's.
    const bool female = (gender == game::Gender::FEMALE) ||
                        (gender == game::Gender::NONBINARY && useFemaleModel);
    uint32_t targetSexId = female ? 1u : 0u;

    const char* raceFolderName = raceModelFolder(targetRaceId);
    result.bodySkinPath = defaultBodySkinPath(targetRaceId, targetSexId);

    const AppearanceBytes look = unpackAppearanceBytes(appearanceBytes);
    const uint8_t charSkinId = look.skinId;
    const uint8_t charFaceId = look.faceId;
    const uint8_t charHairStyleId = look.hairStyleId;
    const uint8_t charHairColorId = look.hairColorId;
    LOG_INFO("Appearance: skin=", static_cast<int>(charSkinId), " face=", static_cast<int>(charFaceId),
             " hairStyle=", static_cast<int>(charHairStyleId), " hairColor=", static_cast<int>(charHairColorId));

    // CharSections, through the one reader in pipeline/char_sections.hpp.
    //
    // This scan used to be written out here, and again in entity_spawner for
    // NPCs, and again in character_preview for the portrait - three readings of
    // one table that did not agree on what they read. Only this one looked at
    // the skin row's second texture, so ears and eyelashes were unbound on the
    // other two; only this one had a fallback for a face the table does not
    // carry. Every fault in this area had to be fixed two or three times.
    auto charSectionsDbc = assetManager_->loadDBC("CharSections.dbc");
    if (charSectionsDbc) {
        const auto* csL = pipeline::getActiveDBCLayout()
            ? pipeline::getActiveDBCLayout()->getLayout("CharSections") : nullptr;
        const auto csF = pipeline::detectCharSectionsFields(charSectionsDbc.get(), csL);

        pipeline::CharacterAppearance who;
        who.raceId = targetRaceId;
        who.sexId = targetSexId;
        who.skinId = charSkinId;
        who.faceId = charFaceId;
        who.hairStyleId = charHairStyleId;
        who.hairColorId = charHairColorId;

        // The underwear rows name art that is not always on disk, and this
        // caller can ask.
        const auto sections = pipeline::resolveCharacterSections(
            charSectionsDbc.get(), csF, who,
            [](const std::string& path, void* ctx) {
                return static_cast<pipeline::AssetManager*>(ctx)->fileExists(path);
            },
            assetManager_);

        if (!sections.bodySkin.empty()) result.bodySkinPath = sections.bodySkin;
        result.skinExtraPath = sections.skinExtra;
        result.faceLowerPath = sections.faceLower;
        result.faceUpperPath = sections.faceUpper;
        if (!sections.hair.empty()) result.hairTexturePath = sections.hair;
        result.underwearPaths = sections.underwear;

        if (!sections.exactFace) {
            LOG_WARNING("No DBC face match for face=", static_cast<int>(charFaceId),
                        " skin=", static_cast<int>(charSkinId),
                        " race=", targetRaceId, " sex=", targetSexId,
                        sections.haveFace ? " - using the nearest face instead"
                                          : " - this character will render with no face");
        }
        if (!sections.haveHair) {
            LOG_WARNING("No DBC hair match for style=", static_cast<int>(charHairStyleId),
                        " color=", static_cast<int>(charHairColorId),
                        " race=", targetRaceId, " sex=", targetSexId);
        }
    } else {
        LOG_WARNING("Failed to load CharSections.dbc, using hardcoded textures");
    }

    // pipeline/char_sections.hpp fills the runtime slots - the same rules the
    // portrait and the NPC path use, in one place.
    {
        pipeline::CharacterSectionTextures resolved;
        resolved.bodySkin  = result.bodySkinPath;
        resolved.skinExtra = result.skinExtraPath;
        resolved.hair      = result.hairTexturePath;
        resolved.underwear = result.underwearPaths;
        pipeline::applyCharacterTextures(model, resolved, raceFolderName);
    }

    // Everything the head detail depends on, in one line, whichever way it went.
    // Skin-coloured eyelashes are what you see when type 8 falls back to the
    // body or pelvis art, and the three things that decide it - whether the
    // model asks for type 8, whether CharSections offered an extra texture, and
    // what was bound in the end - cannot be told apart from a screenshot.
    {
        bool modelWantsExtra = false;
        std::string bound;
        for (const auto& tex : model.textures) {
            if (tex.type == 8) { modelWantsExtra = true; bound = tex.filename; break; }
        }
        LOG_WARNING("Character head detail: model asks for type 8: ",
                    (modelWantsExtra ? "yes" : "no"),
                    " | CharSections extra: '", result.skinExtraPath,
                    "' | bound: '", bound,
                    "' | body: '", result.bodySkinPath, "'");
    }

    return result;
}

void AppearanceComposer::compositePlayerSkin(uint32_t modelSlotId, const PlayerTextureInfo& texInfo) {
    if (!renderer_) return;
    auto* charRenderer = renderer_->getCharacterRenderer();
    if (!charRenderer) return;

    // Save skin composite state for re-compositing on equipment changes
    // Include face textures so compositeWithRegions can rebuild the full base
    bodySkinPath_ = texInfo.bodySkinPath;
    underwearPaths_.clear();
    if (!texInfo.faceLowerPath.empty()) underwearPaths_.push_back(texInfo.faceLowerPath);
    if (!texInfo.faceUpperPath.empty()) underwearPaths_.push_back(texInfo.faceUpperPath);
    for (const auto& up : texInfo.underwearPaths) underwearPaths_.push_back(up);

    // Composite body skin + face + underwear overlays
    {
        std::vector<std::string> layers;
        layers.push_back(texInfo.bodySkinPath);
        if (!texInfo.faceLowerPath.empty()) layers.push_back(texInfo.faceLowerPath);
        if (!texInfo.faceUpperPath.empty()) layers.push_back(texInfo.faceUpperPath);
        for (const auto& up : texInfo.underwearPaths) {
            layers.push_back(up);
        }
        if (layers.size() > 1) {
            rendering::VkTexture* compositeTex = charRenderer->compositeTextures(layers);
            if (compositeTex != nullptr) {
                // Find type-1 (skin) texture slot and replace with composite
                // We need model texture info - walk slots via charRenderer
                // Use the model slot ID to find the right texture index
                auto* modelData = charRenderer->getModelData(modelSlotId);
                if (modelData) {
                    for (size_t ti = 0; ti < modelData->textures.size(); ti++) {
                        if (modelData->textures[ti].type == 1) {
                            charRenderer->setModelTexture(modelSlotId, static_cast<uint32_t>(ti), compositeTex);
                            skinTextureSlotIndex_ = static_cast<uint32_t>(ti);
                            LOG_INFO("Replaced type-1 texture slot ", ti, " with composited body+face+underwear");
                            break;
                        }
                    }
                }
            }
        }
    }

    // Override hair texture on GPU (type-6 slot) after model load
    if (!texInfo.hairTexturePath.empty()) {
        rendering::VkTexture* hairTex = charRenderer->loadTexture(texInfo.hairTexturePath);
        if (hairTex) {
            auto* modelData = charRenderer->getModelData(modelSlotId);
            if (modelData) {
                for (size_t ti = 0; ti < modelData->textures.size(); ti++) {
                    if (modelData->textures[ti].type == 6) {
                        charRenderer->setModelTexture(modelSlotId, static_cast<uint32_t>(ti), hairTex);
                        LOG_INFO("Applied DBC hair texture to slot ", ti, ": ", texInfo.hairTexturePath);
                        break;
                    }
                }
            }
        }
    }

    // Find cloak (type-2, Object Skin) texture slot index
    {
        auto* modelData = charRenderer->getModelData(modelSlotId);
        if (modelData) {
            for (size_t ti = 0; ti < modelData->textures.size(); ti++) {
                if (modelData->textures[ti].type == 2) {
                    cloakTextureSlotIndex_ = static_cast<uint32_t>(ti);
                    LOG_INFO("Cloak texture slot: ", ti);
                    break;
                }
            }
        }
    }
}

std::unordered_set<uint16_t> AppearanceComposer::buildDefaultPlayerGeosets(uint8_t raceId, uint8_t sexId,
                                                                           uint8_t hairStyleId, uint8_t facialId) {
    // Look up the hair scalp and the facial features this character wears, then
    // ask for the bare set around them. Which geosets a character shows with
    // nothing equipped is one answer, in core/geoset_rules.hpp, shared with the
    // portrait - the two used to keep their own and had drifted.
    uint16_t selectedHairScalp = 1;
    uint16_t facial100 = 0, facial200 = 0, facial300 = 0;
    bool haveFacial = false;
    if (entitySpawner_) {
        const auto& hairMap = entitySpawner_->getHairGeosetMap();
        auto itHair = hairMap.find(appearanceKey(raceId, sexId, hairStyleId));
        if (itHair != hairMap.end() && itHair->second > 0) selectedHairScalp = itHair->second;

        const auto& facialMap = entitySpawner_->getFacialHairGeosetMap();
        auto itFacial = facialMap.find(appearanceKey(raceId, sexId, facialId));
        if (itFacial != facialMap.end()) {
            facial100 = itFacial->second.geoset100;
            facial200 = itFacial->second.geoset200;
            facial300 = itFacial->second.geoset300;
            haveFacial = true;
        }
    }
    if (!haveFacial) {
        // No row for this character: the "none" variant of all three channels.
        facial100 = facial200 = facial300 = 1;
    }

    std::unordered_set<uint16_t> activeGeosets =
        bareCharacterGeosets(selectedHairScalp, facial100, facial200, facial300, raceId);

    return activeGeosets;
}

void AppearanceComposer::applyEnchantVisuals(uint32_t charInstanceId, int equipSlotIndex,
                                             uint32_t attachmentId) {
    auto* charRenderer = renderer_ ? renderer_->getCharacterRenderer() : nullptr;
    if (!charRenderer || !gameHandler_ || !assetManager_ || !entitySpawner_) return;

    charRenderer->detachWeaponEffects(charInstanceId, attachmentId);

    uint64_t itemGuid = gameHandler_->getEquipSlotGuid(equipSlotIndex);
    if (itemGuid == 0) return;

    // A temporary enchant (sharpening stone, oil) masks the permanent one's visual.
    auto [permEnchantId, tempEnchantId] = gameHandler_->getItemEnchantIds(itemGuid);
    uint32_t enchantId = (tempEnchantId != 0) ? tempEnchantId : permEnchantId;
    if (enchantId == 0) return;

    auto sieDbc     = assetManager_->loadDBC("SpellItemEnchantment.dbc");
    auto visualsDbc = assetManager_->loadDBC("ItemVisuals.dbc");
    auto effectsDbc = assetManager_->loadDBC("ItemVisualEffects.dbc");
    if (!sieDbc || !sieDbc->isLoaded() || !visualsDbc || !visualsDbc->isLoaded() ||
        !effectsDbc || !effectsDbc->isLoaded()) {
        return;
    }

    const auto* sieL = pipeline::getActiveDBCLayout()
        ? pipeline::getActiveDBCLayout()->getLayout("SpellItemEnchantment") : nullptr;
    auto effectModels = pipeline::resolveEnchantItemVisuals(enchantId, sieDbc.get(),
                                                            visualsDbc.get(), effectsDbc.get(), sieL);

    for (uint32_t visualSlot = 0; visualSlot < effectModels.size(); ++visualSlot) {
        const std::string& modelName = effectModels[visualSlot];
        if (modelName.empty()) continue;

        // DBC stores .mdx paths; the shipped assets are .m2.
        std::string m2Path = modelName;
        size_t dotPos = m2Path.rfind('.');
        m2Path = (dotPos != std::string::npos ? m2Path.substr(0, dotPos) : m2Path) + ".m2";

        pipeline::M2Model effectModel;
        if (!loadWeaponM2(m2Path, effectModel)) {
            LOG_WARNING("Enchant visual: failed to load ", m2Path);
            continue;
        }

        uint32_t effectModelId = entitySpawner_->allocateWeaponModelId();
        if (charRenderer->attachWeaponEffect(charInstanceId, attachmentId, visualSlot,
                                             effectModel, effectModelId)) {
            LOG_INFO("Enchant visual: ", m2Path, " on attachment ", attachmentId,
                     " (enchant ", enchantId, ", visual slot ", visualSlot, ")");
        }
    }
}

bool AppearanceComposer::loadWeaponM2(const std::string& m2Path, pipeline::M2Model& outModel) {
    // pipeline/m2_asset_loader.hpp. Kept as a method because a dozen call sites
    // read better for it.
    return pipeline::loadM2WithSkin(*assetManager_, m2Path, outModel);
}

// Head gear, which only other players used to get. The local character's
// appearance is assembled here while everyone else's goes through
// EntitySpawner, and the head slot was simply missing from this side: no
// helmet model, and hair left showing through where one should be.
void AppearanceComposer::loadEquippedHelm(game::Inventory& inventory) {
    auto* charRenderer = renderer_ ? renderer_->getCharacterRenderer() : nullptr;
    const uint32_t charInstanceId = renderer_ ? renderer_->getCharacterInstanceId() : 0;
    if (!charRenderer || charInstanceId == 0 || !assetManager_ || !gameHandler_) return;

    // Only the helm point. Detaching 0 as well would drop the shield.
    charRenderer->detachWeapon(charInstanceId, kAttachHelm);

    // Hiding the helm is a display choice, not an unequip: the item stays on,
    // the model comes off, and the hair comes back.
    if (!gameHandler_->isHelmVisible()) return;

    const auto& headSlot = inventory.getEquipSlot(game::EquipSlot::HEAD);
    if (headSlot.empty()) return;
    const auto* info = gameHandler_->getItemInfo(headSlot.item.itemId);
    const uint32_t displayId = info && info->valid ? info->displayInfoId
                                                   : headSlot.item.displayInfoId;
    if (displayId == 0) return;

    uint8_t raceId = 0;
    uint8_t genderId = 0;
    if (const auto* ch = gameHandler_->getActiveCharacter()) {
        raceId = static_cast<uint8_t>(ch->race);
        genderId = static_cast<uint8_t>(ch->gender);
    }

    const core::HelmVisual helm =
        core::resolveHelmVisual(*assetManager_, displayId, raceId, genderId);
    if (!helm.valid()) return;

    pipeline::M2Model helmModel;
    std::string helmPath;
    if (!helm.racialModelPath.empty()) {
        helmPath = helm.racialModelPath;
        if (!loadWeaponM2(helmPath, helmModel)) helmModel = {};
    }
    if (!helmModel.isValid()) {
        helmPath = helm.baseModelPath;
        if (!loadWeaponM2(helmPath, helmModel)) return;
    }

    const uint32_t helmModelId = entitySpawner_ ? entitySpawner_->allocateWeaponModelId() : 0;
    const bool attached = charRenderer->attachWeapon(charInstanceId, kAttachHelm, helmModel,
                                                     helmModelId, helm.texturePath);
    if (attached) {
        LOG_INFO("Equipped helm: ", helmPath, " tex: ", helm.texturePath);
    }
}

void AppearanceComposer::loadEquippedWeapons() {
    // Equipment refreshes can arrive during a gather cast. Keep the temporary
    // tool authoritative until the cast-end callback restores real equipment.
    const uint32_t currentInstanceId = renderer_ ? renderer_->getCharacterInstanceId() : 0;
    if (showingMiningPick_ && currentInstanceId == miningPickInstanceId_) return;
    if (showingMiningPick_) {
        showingMiningPick_ = false;
        miningPickInstanceId_ = 0;
    }
    showingRanged_ = false;
    if (renderer_ && renderer_->getAnimationController())
        renderer_->getAnimationController()->setRangedWeaponActive(false);
    if (!renderer_ || !renderer_->getCharacterRenderer() || !assetManager_ || !assetManager_->isInitialized())
        return;
    if (!gameHandler_) return;

    auto* charRenderer = renderer_->getCharacterRenderer();
    uint32_t charInstanceId = renderer_->getCharacterInstanceId();
    if (charInstanceId == 0) return;

    auto& inventory = gameHandler_->getInventory();

    loadEquippedHelm(inventory);

    // Load ItemDisplayInfo.dbc
    auto displayInfoDbc = assetManager_->loadDBC("ItemDisplayInfo.dbc");
    if (!displayInfoDbc) {
        LOG_WARNING("loadEquippedWeapons: failed to load ItemDisplayInfo.dbc");
        return;
    }
    // Mapping: EquipSlot → held attachment. Sheathed attachment is resolved
    // from the item's InventoryType below.
    struct WeaponSlot {
        game::EquipSlot slot;
        uint32_t attachmentId;
    };
    WeaponSlot weaponSlots[] = {
        { game::EquipSlot::MAIN_HAND, kAttachRightHand },
        { game::EquipSlot::OFF_HAND,  kAttachLeftHand },
    };

    // Equipment reloads and Z toggles can move models between these points.
    // Clear both held and sheathed locations so old copies never remain behind.
    const uint32_t weaponAttachmentPoints[] = {
        kAttachShield, kAttachRightHand, kAttachLeftHand,
        kAttachRightHip, kAttachLeftHip, kAttachBack
    };
    for (uint32_t attachmentId : weaponAttachmentPoints) {
        charRenderer->detachWeapon(charInstanceId, attachmentId);
    }

    bool rightHandFilled = false;

    for (const auto& ws : weaponSlots) {
        const auto& equipSlot = inventory.getEquipSlot(ws.slot);

        // If slot is empty or has no displayInfoId, detach any existing weapon
        if (equipSlot.empty() || equipSlot.item.displayInfoId == 0) {
            charRenderer->detachWeapon(charInstanceId, ws.attachmentId);
            continue;
        }

        const uint32_t attachmentId = weaponAttachment(
            weaponsSheathed_, ws.slot, equipSlot.item.inventoryType);
        if (attachmentId == UINT32_MAX) continue;

        uint32_t displayInfoId = equipSlot.item.displayInfoId;
        int32_t recIdx = displayInfoDbc->findRecordById(displayInfoId);
        if (recIdx < 0) {
            LOG_WARNING("loadEquippedWeapons: displayInfoId ", displayInfoId, " not found in DBC");
            charRenderer->detachWeapon(charInstanceId, ws.attachmentId);
            continue;
        }

        // The left pair first, the right one when there is none - the rule this
        // copy did not have, which is why a weapon whose display names only the
        // right model rendered on an NPC and not on the player holding it.
        const auto art = pipeline::readItemDisplayArt(*displayInfoDbc,
                                                      static_cast<uint32_t>(recIdx));
        const std::string& textureName = art.textureName;

        if (art.modelFile.empty()) {
            LOG_WARNING("loadEquippedWeapons: empty model name for displayInfoId ", displayInfoId);
            charRenderer->detachWeapon(charInstanceId, ws.attachmentId);
            continue;
        }

        const std::string& modelFile = art.modelFile;

        // Try Weapon directory first, then Shield
        std::string m2Path = "Item\\ObjectComponents\\Weapon\\" + modelFile;
        pipeline::M2Model weaponModel;
        if (!loadWeaponM2(m2Path, weaponModel)) {
            m2Path = "Item\\ObjectComponents\\Shield\\" + modelFile;
            if (!loadWeaponM2(m2Path, weaponModel)) {
                LOG_WARNING("loadEquippedWeapons: failed to load ", modelFile);
                charRenderer->detachWeapon(charInstanceId, ws.attachmentId);
                continue;
            }
        }

        // Build texture path
        std::string texturePath;
        if (!textureName.empty()) {
            texturePath = "Item\\ObjectComponents\\Weapon\\" + textureName + ".blp";
            if (!assetManager_->fileExists(texturePath)) {
                texturePath = "Item\\ObjectComponents\\Shield\\" + textureName + ".blp";
            }
        }

        uint32_t weaponModelId = entitySpawner_->allocateWeaponModelId();
        const glm::mat4 localTransform = weaponLocalTransform(
            weaponsSheathed_, ws.slot, equipSlot.item.inventoryType);
        bool ok = charRenderer->attachWeapon(charInstanceId, attachmentId,
                                              weaponModel, weaponModelId, texturePath,
                                              localTransform);
        if (ok) {
            LOG_INFO("Equipped weapon: ", m2Path, " at attachment ", attachmentId,
                     weaponsSheathed_ ? " (sheathed)" : " (held)");
            if (ws.slot == game::EquipSlot::MAIN_HAND) rightHandFilled = true;
            applyEnchantVisuals(charInstanceId, static_cast<int>(ws.slot), attachmentId);
        }
    }

    // --- RANGED slot (bow, gun, crossbow, thrown) ---
    // Show ranged weapon in right hand when main hand is empty.
    const auto& rangedSlot = inventory.getEquipSlot(game::EquipSlot::RANGED);
    if (!rightHandFilled && !rangedSlot.empty() && rangedSlot.item.displayInfoId != 0) {
        uint32_t displayInfoId = rangedSlot.item.displayInfoId;
        int32_t recIdx = displayInfoDbc->findRecordById(displayInfoId);
        if (recIdx >= 0) {
            const auto art = pipeline::readItemDisplayArt(*displayInfoDbc,
                                                          static_cast<uint32_t>(recIdx));
            const std::string& textureName = art.textureName;

            if (!art.modelFile.empty()) {
                const std::string& modelFile = art.modelFile;

                std::string m2Path = "Item\\ObjectComponents\\Weapon\\" + modelFile;
                pipeline::M2Model weaponModel;
                if (!loadWeaponM2(m2Path, weaponModel)) {
                    m2Path = "Item\\ObjectComponents\\Shield\\" + modelFile;
                    loadWeaponM2(m2Path, weaponModel);
                }

                if (weaponModel.vertices.size() > 0) {
                    std::string texturePath;
                    if (!textureName.empty()) {
                        texturePath = "Item\\ObjectComponents\\Weapon\\" + textureName + ".blp";
                        if (!assetManager_->fileExists(texturePath)) {
                            texturePath = "Item\\ObjectComponents\\Shield\\" + textureName + ".blp";
                        }
                    }

                    uint32_t weaponModelId = entitySpawner_->allocateWeaponModelId();
                    const uint32_t rangedAttachment = weaponsSheathed_
                        ? kAttachBack : kAttachRightHand;
                    const glm::mat4 localTransform = weaponsSheathed_
                        ? weaponLocalTransform(true, game::EquipSlot::MAIN_HAND,
                                               game::InvType::TWO_HAND)
                        : glm::mat4(1.0f);
                    bool ok = charRenderer->attachWeapon(charInstanceId, rangedAttachment,
                                                          weaponModel, weaponModelId, texturePath,
                                                          localTransform);
                    if (ok) {
                        LOG_INFO("Equipped ranged weapon: ", m2Path, " at attachment ",
                                 rangedAttachment, weaponsSheathed_ ? " (sheathed)" : " (held)");
                    }
                }
            }
        }
    }
}

void AppearanceComposer::showMiningPick(bool show) {
    if (show == showingMiningPick_) return;

    if (!show) {
        showingMiningPick_ = false;
        miningPickInstanceId_ = 0;
        loadEquippedWeapons();
        return;
    }

    if (!renderer_ || !renderer_->getCharacterRenderer() || !assetManager_ ||
        !assetManager_->isInitialized() || !entitySpawner_) {
        return;
    }

    auto* charRenderer = renderer_->getCharacterRenderer();
    const uint32_t charInstanceId = renderer_->getCharacterInstanceId();
    if (charInstanceId == 0) return;

    // Item 2901 (Mining Pick) resolves to ItemDisplayInfo 6568 in the WotLK DBC.
    constexpr uint32_t kMiningPickDisplayId = 6568;
    auto displayInfoDbc = assetManager_->loadDBC("ItemDisplayInfo.dbc");
    if (!displayInfoDbc) return;

    const int32_t recIdx = displayInfoDbc->findRecordById(kMiningPickDisplayId);
    if (recIdx < 0) {
        LOG_WARNING("showMiningPick: displayInfoId ", kMiningPickDisplayId,
                    " not found in DBC");
        return;
    }

    const auto* idiL = pipeline::getActiveDBCLayout()
        ? pipeline::getActiveDBCLayout()->getLayout("ItemDisplayInfo") : nullptr;
    std::string modelName = displayInfoDbc->getString(
        static_cast<uint32_t>(recIdx), idiL ? (*idiL)["LeftModel"] : 1);
    std::string textureName = displayInfoDbc->getString(
        static_cast<uint32_t>(recIdx), idiL ? (*idiL)["LeftModelTexture"] : 3);
    if (modelName.empty()) return;

    const size_t dotPos = modelName.rfind('.');
    std::string modelFile = dotPos == std::string::npos
        ? modelName + ".m2" : modelName.substr(0, dotPos) + ".m2";
    const std::string m2Path = "Item\\ObjectComponents\\Weapon\\" + modelFile;

    pipeline::M2Model pickModel;
    if (!loadWeaponM2(m2Path, pickModel)) {
        LOG_WARNING("showMiningPick: failed to load ", m2Path);
        return;
    }

    std::string texturePath;
    if (!textureName.empty()) {
        texturePath = "Item\\ObjectComponents\\Weapon\\" + textureName + ".blp";
    }

    charRenderer->detachWeapon(charInstanceId, kAttachRightHand);
    const uint32_t modelId = entitySpawner_->allocateWeaponModelId();
    if (charRenderer->attachWeapon(charInstanceId, kAttachRightHand, pickModel,
                                   modelId, texturePath)) {
        showingMiningPick_ = true;
        miningPickInstanceId_ = charInstanceId;
        showingRanged_ = false;
        if (renderer_->getAnimationController())
            renderer_->getAnimationController()->setRangedWeaponActive(false);
        LOG_INFO("Mining pick attached at right hand: ", m2Path);
    } else {
        // Do not leave the player empty-handed if the temporary model failed.
        loadEquippedWeapons();
    }
}

void AppearanceComposer::showFishingPole(bool show) {
    if (show == showingFishingPole_) return;

    if (!show) {
        showingFishingPole_ = false;
        loadEquippedWeapons();
        return;
    }
    if (!renderer_ || !renderer_->getCharacterRenderer() || !gameHandler_ ||
        !assetManager_ || !assetManager_->isInitialized() || !entitySpawner_) {
        return;
    }

    const auto isFishingPole = [this](const game::ItemSlot& slot) {
        if (slot.empty()) return false;
        if (slot.item.subclassName == "Fishing Pole") return true;
        const auto* info = gameHandler_->getItemInfo(slot.item.itemId);
        return info && info->valid && info->itemClass == 2 && info->subClass == 20;
    };

    const game::ItemSlot* pole = nullptr;
    const auto& inventory = gameHandler_->getInventory();
    const auto& mainHand = inventory.getEquipSlot(game::EquipSlot::MAIN_HAND);
    if (isFishingPole(mainHand)) pole = &mainHand;
    for (int i = 0; !pole && i < inventory.getBackpackSize(); ++i) {
        const auto& slot = inventory.getBackpackSlot(i);
        if (isFishingPole(slot)) pole = &slot;
    }
    for (int bag = 0; !pole && bag < game::Inventory::NUM_BAG_SLOTS; ++bag) {
        for (int slotIndex = 0; !pole && slotIndex < inventory.getBagSize(bag); ++slotIndex) {
            const auto& slot = inventory.getBagSlot(bag, slotIndex);
            if (isFishingPole(slot)) pole = &slot;
        }
    }
    if (!pole || pole->item.displayInfoId == 0) {
        LOG_WARNING("showFishingPole: no fishing pole with display data found in inventory");
        return;
    }

    auto displayInfoDbc = assetManager_->loadDBC("ItemDisplayInfo.dbc");
    if (!displayInfoDbc) return;
    const int32_t recIdx = displayInfoDbc->findRecordById(pole->item.displayInfoId);
    if (recIdx < 0) return;

    const auto* idiL = pipeline::getActiveDBCLayout()
        ? pipeline::getActiveDBCLayout()->getLayout("ItemDisplayInfo") : nullptr;
    std::string modelName = displayInfoDbc->getString(
        static_cast<uint32_t>(recIdx), idiL ? (*idiL)["LeftModel"] : 1);
    std::string textureName = displayInfoDbc->getString(
        static_cast<uint32_t>(recIdx), idiL ? (*idiL)["LeftModelTexture"] : 3);
    if (modelName.empty()) return;

    const size_t dotPos = modelName.rfind('.');
    const std::string modelFile = dotPos == std::string::npos
        ? modelName + ".m2" : modelName.substr(0, dotPos) + ".m2";
    std::string m2Path = "Item\\ObjectComponents\\Weapon\\" + modelFile;
    pipeline::M2Model poleModel;
    if (!loadWeaponM2(m2Path, poleModel)) return;

    std::string texturePath;
    if (!textureName.empty()) {
        texturePath = "Item\\ObjectComponents\\Weapon\\" + textureName + ".blp";
    }

    auto* charRenderer = renderer_->getCharacterRenderer();
    const uint32_t charInstanceId = renderer_->getCharacterInstanceId();
    if (charInstanceId == 0) return;
    charRenderer->detachWeapon(charInstanceId, kAttachRightHand);
    const uint32_t modelId = entitySpawner_->allocateWeaponModelId();
    if (charRenderer->attachWeapon(charInstanceId, kAttachRightHand, poleModel,
                                   modelId, texturePath)) {
        showingFishingPole_ = true;
        showingRanged_ = false;
        if (renderer_->getAnimationController())
            renderer_->getAnimationController()->setRangedWeaponActive(false);
        LOG_INFO("Fishing pole attached at right hand: ", m2Path);
    } else {
        loadEquippedWeapons();
    }
}

void AppearanceComposer::showRangedWeapon(bool show) {
    if (show == showingRanged_) return;

    if (!renderer_ || !renderer_->getCharacterRenderer() || !gameHandler_ || !assetManager_ || !assetManager_->isInitialized())
        return;

    auto* charRenderer = renderer_->getCharacterRenderer();
    uint32_t charInstanceId = renderer_->getCharacterInstanceId();
    if (charInstanceId == 0) return;

    if (!show) {
        showingRanged_ = false;
        if (renderer_->getAnimationController())
            renderer_->getAnimationController()->setRangedWeaponActive(false);
        // Swap back to normal melee weapons
        loadEquippedWeapons();
        return;
    }

    auto& inventory = gameHandler_->getInventory();
    const auto& rangedSlot = inventory.getEquipSlot(game::EquipSlot::RANGED);
    if (rangedSlot.empty() || rangedSlot.item.displayInfoId == 0) return;

    auto displayInfoDbc = assetManager_->loadDBC("ItemDisplayInfo.dbc");
    if (!displayInfoDbc) return;

    uint32_t displayInfoId = rangedSlot.item.displayInfoId;
    int32_t recIdx = displayInfoDbc->findRecordById(displayInfoId);
    if (recIdx < 0) return;

    const auto* idiL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("ItemDisplayInfo") : nullptr;
    std::string modelName = displayInfoDbc->getString(static_cast<uint32_t>(recIdx), idiL ? (*idiL)["LeftModel"] : 1);
    std::string textureName = displayInfoDbc->getString(static_cast<uint32_t>(recIdx), idiL ? (*idiL)["LeftModelTexture"] : 3);
    if (modelName.empty()) return;

    std::string modelFile = modelName;
    {
        size_t dotPos = modelFile.rfind('.');
        if (dotPos != std::string::npos)
            modelFile = modelFile.substr(0, dotPos) + ".m2";
        else
            modelFile += ".m2";
    }

    std::string m2Path = "Item\\ObjectComponents\\Weapon\\" + modelFile;
    pipeline::M2Model weaponModel;
    if (!loadWeaponM2(m2Path, weaponModel)) {
        m2Path = "Item\\ObjectComponents\\Shield\\" + modelFile;
        if (!loadWeaponM2(m2Path, weaponModel)) return;
    }

    std::string texturePath;
    if (!textureName.empty()) {
        texturePath = "Item\\ObjectComponents\\Weapon\\" + textureName + ".blp";
        if (!assetManager_->fileExists(texturePath))
            texturePath = "Item\\ObjectComponents\\Shield\\" + textureName + ".blp";
    }

    // Detach current right-hand weapon and attach ranged weapon
    charRenderer->detachWeapon(charInstanceId, 1);
    uint32_t weaponModelId = entitySpawner_->allocateWeaponModelId();
    bool ok = charRenderer->attachWeapon(charInstanceId, 1, weaponModel, weaponModelId, texturePath);
    if (ok) {
        showingRanged_ = true;
        if (renderer_->getAnimationController())
            renderer_->getAnimationController()->setRangedWeaponActive(true);
        LOG_INFO("Swapped to ranged weapon: ", m2Path, " at attachment 1 (right hand)");
    }
}

} // namespace core
} // namespace wowee
