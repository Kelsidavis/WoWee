#include "core/entity_spawner.hpp"
#include "core/helm_visual.hpp"

// M2 attachment 11 is the helm. 0 is the shield mount, which is where head gear
// was going: it attached, reported success, and hung off the forearm.
namespace { constexpr uint32_t kAttachHelm = 11; }
#include "core/coordinates.hpp"
#include "core/logger.hpp"
#include "rendering/renderer.hpp"
#include "rendering/animation_controller.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/character_renderer.hpp"
#include "rendering/wmo_renderer.hpp"
#include "rendering/m2_renderer.hpp"
#include "audio/npc_voice_manager.hpp"
#include "pipeline/m2_loader.hpp"
#include "pipeline/wmo_loader.hpp"
#include "rendering/animation/animation_ids.hpp"
#include "pipeline/dbc_loader.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_layout.hpp"
#include "game/game_handler.hpp"
#include "game/game_services.hpp"
#include "game/transport_manager.hpp"

#include <cmath>
#include <algorithm>
#include <cctype>
#include <sstream>
#include <cstring>
#include <unordered_map>
#include <unordered_set>

namespace wowee {
namespace core {

namespace {
// Default (bare) geoset IDs per equipment group.
// Each group's base is groupNumber * 100; variant 01 is typically bare/default.
constexpr uint16_t kGeosetDefaultConnector = 101;   // Group  1: default hair connector
constexpr uint16_t kGeosetBareForearms     = 401;   // Group  4: no gloves
constexpr uint16_t kGeosetBareShins        = 501;   // Group  5: no boots
constexpr uint16_t kGeosetDefaultEars      = 702;   // Group  7: ears
constexpr uint16_t kGeosetBareSleeves      = 801;   // Group  8: no chest armor sleeves
constexpr uint16_t kGeosetDefaultKneepads  = 902;   // Group  9: kneepads
constexpr uint16_t kGeosetDefaultTabard    = 1201;  // Group 12: tabard base
constexpr uint16_t kGeosetBarePants        = 1301;  // Group 13: no leggings
constexpr uint16_t kGeosetNoCape           = 1501;  // Group 15: no cape
constexpr uint16_t kGeosetWithCape         = 1502;  // Group 15: with cape
constexpr uint16_t kGeosetBareFeet         = 2002;  // Group 20: bare feet

uint16_t selectHairScalpGeoset(const std::unordered_map<uint32_t, uint16_t>& hairGeosets,
                               uint8_t raceId,
                               uint8_t genderId,
                               uint8_t hairStyleId) {
    const uint32_t key = (static_cast<uint32_t>(raceId) << 16) |
                         (static_cast<uint32_t>(genderId) << 8) |
                         static_cast<uint32_t>(hairStyleId);
    auto it = hairGeosets.find(key);
    if (it != hairGeosets.end() && it->second > 0) {
        return it->second;
    }
    return 1;
}
} // namespace

void EntitySpawner::spawnOnlinePlayer(uint64_t guid,
                                    uint8_t raceId,
                                    uint8_t genderId,
                                    uint32_t appearanceBytes,
                                    uint8_t facialFeatures,
                                    float x, float y, float z, float orientation) {
    if (!renderer_ || !renderer_->getCharacterRenderer() || !assetManager_ || !assetManager_->isInitialized()) return;
    if (playerInstances_.count(guid)) return;

    // Skip local player — already spawned as the main character
    if (gameHandler_) {
        uint64_t localGuid = gameHandler_->getPlayerGuid();
        uint64_t activeGuid = gameHandler_->getActiveCharacterGuid();
        if ((localGuid != 0 && guid == localGuid) ||
            (activeGuid != 0 && guid == activeGuid) ||
            (spawnedPlayerGuid_ != 0 && guid == spawnedPlayerGuid_)) {
            return;
        }
    }
    auto* charRenderer = renderer_->getCharacterRenderer();

    // Base geometry model: cache by (race, gender)
    uint32_t cacheKey = (static_cast<uint32_t>(raceId) << 8) | static_cast<uint32_t>(genderId & 0xFF);
    uint32_t modelId = 0;
    auto itCache = playerModelCache_.find(cacheKey);
    if (itCache != playerModelCache_.end()) {
        modelId = itCache->second;
        if (!charRenderer->getModelData(modelId)) {
            LOG_WARNING("spawnOnlinePlayer: cached player model missing after world reload, reloading modelId=",
                        modelId, " race=", static_cast<int>(raceId),
                        " gender=", static_cast<int>(genderId));
            playerTextureSlotsByModelId_.erase(modelId);
            playerModelCache_.erase(itCache);
            modelId = 0;
        }
    }
    if (modelId == 0) {
        game::Race race = static_cast<game::Race>(raceId);
        game::Gender gender = (genderId == 1) ? game::Gender::FEMALE : game::Gender::MALE;
        std::string m2Path = game::getPlayerModelPath(race, gender);
        if (m2Path.empty()) {
            LOG_WARNING("spawnOnlinePlayer: unknown race/gender for guid 0x", std::hex, guid, std::dec,
                        " race=", static_cast<int>(raceId), " gender=", static_cast<int>(genderId));
            return;
        }

        // Parse modelDir/baseName for skin/anim loading
        std::string modelDir;
        std::string baseName;
        {
            size_t slash = m2Path.rfind('\\');
            if (slash != std::string::npos) {
                modelDir = m2Path.substr(0, slash + 1);
                baseName = m2Path.substr(slash + 1);
            } else {
                baseName = m2Path;
            }
            size_t dot = baseName.rfind('.');
            if (dot != std::string::npos) baseName = baseName.substr(0, dot);
        }

        auto m2Data = assetManager_->readFile(m2Path);
        if (m2Data.empty()) {
            LOG_WARNING("spawnOnlinePlayer: failed to read M2: ", m2Path);
            return;
        }

        pipeline::M2Model model = pipeline::M2Loader::load(m2Data);
        if (model.vertices.empty()) {
            LOG_WARNING("spawnOnlinePlayer: failed to parse M2: ", m2Path);
            return;
        }

        // Skin file (only for WotLK M2s - vanilla has embedded skin)
        std::string skinPath = modelDir + baseName + "00.skin";
        auto skinData = assetManager_->readFile(skinPath);
        if (!skinData.empty() && model.version >= 264) {
            pipeline::M2Loader::loadSkin(skinData, model);
        }

        // After skin loading, full model must be valid (vertices + indices)
        if (!model.isValid()) {
            LOG_WARNING("spawnOnlinePlayer: failed to load skin for M2: ", m2Path);
            return;
        }

        // Load only core external animations (stand/walk/run) to avoid stalls
        for (uint32_t si = 0; si < model.sequences.size(); si++) {
            if (!(model.sequences[si].flags & 0x20)) {
                uint32_t animId = model.sequences[si].id;
                if (animId != rendering::anim::STAND && animId != rendering::anim::WALK && animId != rendering::anim::RUN) continue;
                char animFileName[256];
                snprintf(animFileName, sizeof(animFileName),
                         "%s%s%04u-%02u.anim",
                         modelDir.c_str(),
                         baseName.c_str(),
                         animId,
                         model.sequences[si].variationIndex);
                auto animData = assetManager_->readFileOptional(animFileName);
                if (!animData.empty()) {
                    pipeline::M2Loader::loadAnimFile(m2Data, animData, si, model);
                }
            }
        }

        modelId = nextPlayerModelId_++;
        if (!charRenderer->loadModel(model, modelId)) {
            LOG_WARNING("spawnOnlinePlayer: failed to load model to GPU: ", m2Path);
            return;
        }

        playerModelCache_[cacheKey] = modelId;
    }

    // Determine texture slots once per model
    {
        auto [slotIt, inserted] = playerTextureSlotsByModelId_.try_emplace(modelId);
        if (inserted) {
            PlayerTextureSlots slots;
            if (const auto* md = charRenderer->getModelData(modelId)) {
                for (size_t ti = 0; ti < md->textures.size(); ti++) {
                    uint32_t t = md->textures[ti].type;
                    if (t == 1 && slots.skin < 0) slots.skin = static_cast<int>(ti);
                    else if (t == 6 && slots.hair < 0) slots.hair = static_cast<int>(ti);
                    else if (t == 8 && slots.underwear < 0) slots.underwear = static_cast<int>(ti);
                }
            }
            slotIt->second = slots;
        }
    }

    // Create instance at server position
    glm::vec3 renderPos = core::coords::canonicalToRender(glm::vec3(x, y, z));
    float renderYaw = orientation + glm::radians(90.0f);
    uint32_t instanceId = charRenderer->createInstance(modelId, renderPos, glm::vec3(0.0f, 0.0f, renderYaw), 1.0f);
    if (instanceId == 0) return;

    // Resolve skin/hair texture paths via CharSections, then apply as per-instance overrides
    const char* raceFolderName = "Human";
    switch (static_cast<game::Race>(raceId)) {
        case game::Race::HUMAN: raceFolderName = "Human"; break;
        case game::Race::ORC: raceFolderName = "Orc"; break;
        case game::Race::DWARF: raceFolderName = "Dwarf"; break;
        case game::Race::NIGHT_ELF: raceFolderName = "NightElf"; break;
        case game::Race::UNDEAD: raceFolderName = "Scourge"; break;
        case game::Race::TAUREN: raceFolderName = "Tauren"; break;
        case game::Race::GNOME: raceFolderName = "Gnome"; break;
        case game::Race::TROLL: raceFolderName = "Troll"; break;
        case game::Race::BLOOD_ELF: raceFolderName = "BloodElf"; break;
        case game::Race::DRAENEI: raceFolderName = "Draenei"; break;
        default: break;
    }
    const char* genderFolder = (genderId == 1) ? "Female" : "Male";
    std::string raceGender = std::string(raceFolderName) + genderFolder;
    std::string bodySkinPath = std::string("Character\\") + raceFolderName + "\\" + genderFolder + "\\" + raceGender + "Skin00_00.blp";
    std::string pelvisPath = std::string("Character\\") + raceFolderName + "\\" + genderFolder + "\\" + raceGender + "NakedPelvisSkin00_00.blp";
    std::vector<std::string> underwearPaths;
    std::string hairTexturePath;
    std::string faceLowerPath;
    std::string faceUpperPath;

    uint8_t skinId = appearanceBytes & 0xFF;
    uint8_t faceId = (appearanceBytes >> 8) & 0xFF;
    uint8_t hairStyleId = (appearanceBytes >> 16) & 0xFF;
    uint8_t hairColorId = (appearanceBytes >> 24) & 0xFF;

    if (auto charSectionsDbc = assetManager_->loadDBC("CharSections.dbc"); charSectionsDbc && charSectionsDbc->isLoaded()) {
        const auto* csL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("CharSections") : nullptr;
        auto csF = pipeline::detectCharSectionsFields(charSectionsDbc.get(), csL);
        uint32_t targetRaceId = raceId;
        uint32_t targetSexId = genderId;

        bool foundSkin = false;
        bool foundUnderwear = false;
        bool foundHair = false;
        bool foundFaceLower = false;
        // Same nearest-face fallback the local player's composer uses. These two
        // appearance paths have to stay in step: one is every other player in
        // the world, the other is you, and a face that resolves for one and not
        // the other is exactly the kind of difference nobody thinks to check.
        std::string faceAltLower, faceAltUpper;
        bool haveFaceAlt = false;

        for (uint32_t r = 0; r < charSectionsDbc->getRecordCount(); r++) {
            uint32_t rRace = charSectionsDbc->getUInt32(r, csF.raceId);
            uint32_t rSex = charSectionsDbc->getUInt32(r, csF.sexId);
            uint32_t baseSection = charSectionsDbc->getUInt32(r, csF.baseSection);
            uint32_t variationIndex = charSectionsDbc->getUInt32(r, csF.variationIndex);
            uint32_t colorIndex = charSectionsDbc->getUInt32(r, csF.colorIndex);

            if (rRace != targetRaceId || rSex != targetSexId) continue;

            if (baseSection == 0 && !foundSkin && colorIndex == skinId) {
                std::string tex1 = charSectionsDbc->getString(r, csF.texture1);
                if (!tex1.empty()) { bodySkinPath = tex1; foundSkin = true; }
            } else if (baseSection == 3 && !foundHair &&
                       variationIndex == hairStyleId && colorIndex == hairColorId) {
                hairTexturePath = charSectionsDbc->getString(r, csF.texture1);
                if (!hairTexturePath.empty()) foundHair = true;
            } else if (baseSection == 4 && !foundUnderwear && colorIndex == skinId) {
                // Verify textures exist — some DBC entries reference BLPs
                // that were never shipped (e.g. Draenei skin colors 10-16).
                bool allExist = true;
                std::vector<std::string> candidateUW;
                for (uint32_t f = csF.texture1; f <= csF.texture1 + 2; f++) {
                    std::string tex = charSectionsDbc->getString(r, f);
                    if (!tex.empty()) {
                        if (assetManager_->fileExists(tex))
                            candidateUW.push_back(tex);
                        else
                            allExist = false;
                    }
                }
                if (allExist || !candidateUW.empty()) {
                    underwearPaths = std::move(candidateUW);
                    foundUnderwear = true;
                }
            } else if (baseSection == 1 && !foundFaceLower &&
                       variationIndex == faceId && colorIndex == skinId) {
                std::string tex1 = charSectionsDbc->getString(r, csF.texture1);
                std::string tex2 = charSectionsDbc->getString(r, csF.texture2);
                if (!tex1.empty()) faceLowerPath = tex1;
                if (!tex2.empty()) faceUpperPath = tex2;
                foundFaceLower = true;
            } else if (baseSection == 1 && !foundFaceLower && !haveFaceAlt &&
                       (variationIndex == faceId || colorIndex == skinId)) {
                std::string tex1 = charSectionsDbc->getString(r, csF.texture1);
                if (!tex1.empty()) {
                    faceAltLower = tex1;
                    faceAltUpper = charSectionsDbc->getString(r, csF.texture2);
                    haveFaceAlt = true;
                }
            }

            if (foundSkin && foundUnderwear && foundHair && foundFaceLower) break;
        }

        if (!foundFaceLower) {
            LOG_WARNING("spawnOnlinePlayer: no DBC face match for face=",
                        static_cast<int>(faceId), " skin=", static_cast<int>(skinId),
                        " race=", targetRaceId, " sex=", targetSexId,
                        haveFaceAlt ? " — using the nearest face instead"
                                    : " — this player will render with no face");
            if (haveFaceAlt) {
                faceLowerPath = faceAltLower;
                faceUpperPath = faceAltUpper;
            }
        }
    }

    // Composite base skin + face + underwear overlays
    rendering::VkTexture* compositeTex = nullptr;
    {
        std::vector<std::string> layers;
        layers.push_back(bodySkinPath);
        if (!faceLowerPath.empty()) layers.push_back(faceLowerPath);
        if (!faceUpperPath.empty()) layers.push_back(faceUpperPath);
        for (const auto& up : underwearPaths) layers.push_back(up);
        if (layers.size() > 1) {
            compositeTex = charRenderer->compositeTextures(layers);
        } else {
            compositeTex = charRenderer->loadTexture(bodySkinPath);
        }
    }

    rendering::VkTexture* hairTex = nullptr;
    if (!hairTexturePath.empty()) {
        hairTex = charRenderer->loadTexture(hairTexturePath);
    }
    rendering::VkTexture* underwearTex = nullptr;
    if (!underwearPaths.empty()) underwearTex = charRenderer->loadTexture(underwearPaths[0]);
    else underwearTex = charRenderer->loadTexture(pelvisPath);

    const PlayerTextureSlots& slots = playerTextureSlotsByModelId_[modelId];
    if (slots.skin >= 0 && compositeTex) {
        charRenderer->setTextureSlotOverride(instanceId, static_cast<uint16_t>(slots.skin), compositeTex);
    }
    if (slots.hair >= 0 && hairTex) {
        charRenderer->setTextureSlotOverride(instanceId, static_cast<uint16_t>(slots.hair), hairTex);
    }
    if (slots.underwear >= 0 && underwearTex) {
        charRenderer->setTextureSlotOverride(instanceId, static_cast<uint16_t>(slots.underwear), underwearTex);
    }

    // Geosets: body + selected hair/facial hair. Do not enable every group-0
    // submesh; that activates all hair scalp variants at once.
    std::unordered_set<uint16_t> activeGeosets;
    const uint16_t selectedHairScalp = selectHairScalpGeoset(hairGeosetMap_, raceId, genderId, hairStyleId);
    activeGeosets.insert(0);
    activeGeosets.insert(selectedHairScalp);
    const uint32_t facialKey = (static_cast<uint32_t>(raceId) << 16) |
                               (static_cast<uint32_t>(genderId) << 8) |
                               static_cast<uint32_t>(facialFeatures);
    auto itFacial = facialHairGeosetMap_.find(facialKey);
    if (itFacial != facialHairGeosetMap_.end()) {
        activeGeosets.insert(static_cast<uint16_t>(100 + itFacial->second.geoset100));
        activeGeosets.insert(static_cast<uint16_t>(200 + itFacial->second.geoset200));
        activeGeosets.insert(static_cast<uint16_t>(300 + itFacial->second.geoset300));
    } else {
        activeGeosets.insert(101);
        activeGeosets.insert(201);
        activeGeosets.insert(301);
    }
    activeGeosets.insert(kGeosetBareForearms);
    activeGeosets.insert(kGeosetBareShins);
    activeGeosets.insert(kGeosetDefaultEars);
    activeGeosets.insert(kGeosetBareSleeves);
    activeGeosets.insert(kGeosetDefaultKneepads);
    activeGeosets.insert(kGeosetBarePants);
    activeGeosets.insert(kGeosetNoCape);
    activeGeosets.insert(kGeosetBareFeet);
    charRenderer->setActiveGeosets(instanceId, activeGeosets);

    if (deadCreatureGuids_.count(guid)) {
        charRenderer->playAnimation(instanceId, rendering::anim::DEATH, false);
    } else {
        charRenderer->playAnimation(instanceId, rendering::anim::STAND, true);
    }
    playerInstances_[guid] = instanceId;

    // The mount field may have arrived before this render instance, or the
    // player may be re-created without another values update. Reconcile from
    // retained entity state so already-mounted players are never left on foot.
    if (gameHandler_) {
        auto entity = gameHandler_->getEntityManager().getEntity(guid);
        auto unit = std::dynamic_pointer_cast<game::Unit>(entity);
        if (unit && unit->getMountDisplayId() != 0) {
            setRemotePlayerMountDisplayId(guid, unit->getMountDisplayId());
        }
    }

    OnlinePlayerAppearanceState st;
    st.instanceId = instanceId;
    st.modelId = modelId;
    st.raceId = raceId;
    st.genderId = genderId;
    st.appearanceBytes = appearanceBytes;
    st.facialFeatures = facialFeatures;
    st.bodySkinPath = bodySkinPath;
    // Include face textures so compositeWithRegions can rebuild the full base
    if (!faceLowerPath.empty()) st.underwearPaths.push_back(faceLowerPath);
    if (!faceUpperPath.empty()) st.underwearPaths.push_back(faceUpperPath);
    for (const auto& up : underwearPaths) st.underwearPaths.push_back(up);
    onlinePlayerAppearance_[guid] = std::move(st);
}

void EntitySpawner::setOnlinePlayerEquipment(uint64_t guid,
                                          const std::array<uint32_t, 19>& displayInfoIds,
                                          const std::array<uint8_t, 19>& inventoryTypes) {
    if (!renderer_ || !renderer_->getCharacterRenderer() || !assetManager_ || !assetManager_->isInitialized()) return;

    // Skip local player — equipment handled by GameScreen::updateCharacterGeosets/Textures
    // via consumeOnlineEquipmentDirty(), which fires on the same server update.
    if (gameHandler_) {
        uint64_t localGuid = gameHandler_->getPlayerGuid();
        if (localGuid != 0 && guid == localGuid) return;
    }

    // If the player isn't spawned yet, store equipment until spawn.
    auto appIt = onlinePlayerAppearance_.find(guid);
    if (!playerInstances_.count(guid) || appIt == onlinePlayerAppearance_.end()) {
        pendingOnlinePlayerEquipment_[guid] = {displayInfoIds, inventoryTypes};
        return;
    }

    const OnlinePlayerAppearanceState& st = appIt->second;

    auto* charRenderer = renderer_->getCharacterRenderer();
    if (!charRenderer) return;
    if (st.instanceId == 0 || st.modelId == 0) return;

    if (st.bodySkinPath.empty()) {
        LOG_DEBUG("setOnlinePlayerEquipment: bodySkinPath empty for guid=0x", std::hex, guid, std::dec,
                    " instanceId=", st.instanceId, " — skipping equipment");
        return;
    }

    int nonZeroDisplay = 0;
    for (uint32_t d : displayInfoIds) if (d != 0) nonZeroDisplay++;
    LOG_DEBUG("setOnlinePlayerEquipment: guid=0x", std::hex, guid, std::dec,
                " instanceId=", st.instanceId, " nonZeroDisplayIds=", nonZeroDisplay,
                " head=", displayInfoIds[0], " chest=", displayInfoIds[4],
                " legs=", displayInfoIds[6], " mainhand=", displayInfoIds[15]);

    auto displayInfoDbc = assetManager_->loadDBC("ItemDisplayInfo.dbc");
    if (!displayInfoDbc) return;
    const auto* idiL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("ItemDisplayInfo") : nullptr;

    auto getGeosetGroup = [&](uint32_t displayInfoId, uint32_t fieldIdx) -> uint32_t {
        if (displayInfoId == 0) return 0;
        int32_t recIdx = displayInfoDbc->findRecordById(displayInfoId);
        if (recIdx < 0) return 0;
        return displayInfoDbc->getUInt32(static_cast<uint32_t>(recIdx), fieldIdx);
    };

    auto findDisplayIdByInvType = [&](std::initializer_list<uint8_t> types) -> uint32_t {
        for (int s = 0; s < 19; s++) {
            uint8_t inv = inventoryTypes[s];
            if (inv == 0 || displayInfoIds[s] == 0) continue;
            for (uint8_t t : types) {
                if (inv == t) return displayInfoIds[s];
            }
        }
        return 0;
    };

    auto hasInvType = [&](std::initializer_list<uint8_t> types) -> bool {
        for (int s = 0; s < 19; s++) {
            uint8_t inv = inventoryTypes[s];
            if (inv == 0) continue;
            for (uint8_t t : types) {
                if (inv == t) return true;
            }
        }
        return false;
    };

    // --- Geosets ---
    // Mirror the same group-range logic as CharacterPreview::applyEquipment to
    // keep other-player rendering consistent with the local character preview.
    // Group 4 (4xx) = forearms/gloves, 5 (5xx) = shins/boots, 8 (8xx) = wrists/sleeves,
    // 13 (13xx) = legs/trousers.  Missing defaults caused the shin-mesh gap (status.md).
    std::unordered_set<uint16_t> geosets;
    uint8_t hairStyleId = static_cast<uint8_t>((st.appearanceBytes >> 16) & 0xFF);
    const uint16_t selectedHairScalp = selectHairScalpGeoset(hairGeosetMap_, st.raceId, st.genderId, hairStyleId);
    geosets.insert(0);
    geosets.insert(selectedHairScalp);
    const uint32_t facialKey = (static_cast<uint32_t>(st.raceId) << 16) |
                               (static_cast<uint32_t>(st.genderId) << 8) |
                               static_cast<uint32_t>(st.facialFeatures);
    auto itFacial = facialHairGeosetMap_.find(facialKey);
    if (itFacial != facialHairGeosetMap_.end()) {
        geosets.insert(static_cast<uint16_t>(100 + itFacial->second.geoset100));
        geosets.insert(static_cast<uint16_t>(200 + itFacial->second.geoset200));
        geosets.insert(static_cast<uint16_t>(300 + itFacial->second.geoset300));
    } else {
        geosets.insert(101);
        geosets.insert(201);
        geosets.insert(301);
    }
    geosets.insert(701);                  // Ears
    geosets.insert(kGeosetDefaultKneepads); // Kneepads
    geosets.insert(kGeosetBareFeet);        // Bare feet mesh

    const uint32_t geosetGroup1Field = idiL ? (*idiL)["GeosetGroup1"] : 7;
    const uint32_t geosetGroup3Field = idiL ? (*idiL)["GeosetGroup3"] : 9;

    std::unordered_set<uint16_t> modelGeosets;
    if (const auto* modelData = charRenderer->getModelData(st.modelId)) {
        for (const auto& batch : modelData->batches) {
            modelGeosets.insert(batch.submeshId);
        }
    }

    auto eraseGroup = [&](uint16_t group) {
        for (auto it = geosets.begin(); it != geosets.end();) {
            if ((*it / 100) == group) {
                it = geosets.erase(it);
            } else {
                ++it;
            }
        }
    };

    auto pickGeoset = [&](uint16_t preferred, uint16_t fallback) -> uint16_t {
        if (preferred != 0 && modelGeosets.count(preferred) > 0) return preferred;
        if (fallback != 0 && modelGeosets.count(fallback) > 0) return fallback;
        return 0;
    };

    // Find the lowest submesh ID in a group (e.g., group 5 → lowest 5xx).
    // Races like Gnome (no 501) and Tauren (only 505) need this fallback.
    auto lowestInGroup = [&](uint16_t group) -> uint16_t {
        uint16_t best = 0;
        for (uint16_t g : modelGeosets) {
            if (g / 100 == group && (best == 0 || g < best)) best = g;
        }
        return best;
    };

    // Per-group defaults — overridden below when equipment provides a geoset value.
    uint16_t geosetGloves  = pickGeoset(kGeosetBareForearms, kGeosetBareForearms);
    uint16_t geosetBoots   = pickGeoset(kGeosetBareShins, lowestInGroup(5));
    uint16_t geosetSleeves = pickGeoset(kGeosetBareSleeves, kGeosetBareSleeves);
    uint16_t geosetPants   = pickGeoset(kGeosetBarePants, kGeosetBarePants);

    // Chest/Shirt/Robe (invType 4,5,20) → wrist/sleeve group 8
    {
        uint32_t did = findDisplayIdByInvType({4, 5, 20});
        uint32_t gg1 = getGeosetGroup(did, geosetGroup1Field);
        if (gg1 > 0) geosetSleeves = pickGeoset(static_cast<uint16_t>(kGeosetBareSleeves + gg1), kGeosetBareSleeves);
        // Robe kilt → leg group 13
        uint32_t gg3 = getGeosetGroup(did, geosetGroup3Field);
        if (gg3 > 0) geosetPants = pickGeoset(static_cast<uint16_t>(kGeosetBarePants + gg3), kGeosetBarePants);
    }

    // Legs (invType 7) → leg group 13
    {
        uint32_t did = findDisplayIdByInvType({7});
        uint32_t gg1 = getGeosetGroup(did, geosetGroup1Field);
        if (gg1 > 0) geosetPants = pickGeoset(static_cast<uint16_t>(kGeosetBarePants + gg1), kGeosetBarePants);
    }

    // Feet/Boots (invType 8) → shin group 5
    {
        uint32_t did = findDisplayIdByInvType({8});
        uint32_t gg1 = getGeosetGroup(did, geosetGroup1Field);
        if (gg1 > 0) geosetBoots = pickGeoset(static_cast<uint16_t>(501 + gg1), lowestInGroup(5));
    }

    // Hands/Gloves (invType 10) → forearm group 4
    {
        uint32_t did = findDisplayIdByInvType({10});
        uint32_t gg1 = getGeosetGroup(did, geosetGroup1Field);
        if (gg1 > 0) geosetGloves = pickGeoset(static_cast<uint16_t>(kGeosetBareForearms + gg1), kGeosetBareForearms);
    }

    // Wrists/Bracers (invType 9) → sleeve group 8 (only if chest/shirt didn't set it)
    {
        uint32_t did = findDisplayIdByInvType({9});
        if (did != 0 && geosetSleeves == kGeosetBareSleeves) {
            uint32_t gg1 = getGeosetGroup(did, geosetGroup1Field);
            if (gg1 > 0) geosetSleeves = pickGeoset(static_cast<uint16_t>(kGeosetBareSleeves + gg1), kGeosetBareSleeves);
        }
    }

    // Waist/Belt (invType 6) → buckle group 18
    uint16_t geosetBelt = 0;
    {
        uint32_t did = findDisplayIdByInvType({6});
        uint32_t gg1 = getGeosetGroup(did, geosetGroup1Field);
        if (gg1 > 0) geosetBelt = pickGeoset(static_cast<uint16_t>(1801 + gg1), 0);
    }

    eraseGroup(4);
    eraseGroup(5);
    eraseGroup(8);
    eraseGroup(13);
    eraseGroup(15);
    eraseGroup(18);
    if (geosetGloves != 0) geosets.insert(geosetGloves);
    if (geosetBoots != 0) geosets.insert(geosetBoots);
    if (geosetSleeves != 0) geosets.insert(geosetSleeves);
    if (geosetPants != 0) geosets.insert(geosetPants);
    if (geosetBelt != 0) geosets.insert(geosetBelt);
    // Back/Cloak (invType 16)
    uint16_t geosetCape = pickGeoset(
        hasInvType({16}) ? kGeosetWithCape : kGeosetNoCape,
        kGeosetNoCape);
    if (geosetCape != 0) geosets.insert(geosetCape);
    // Tabard (invType 19)
    if (hasInvType({19})) geosets.insert(kGeosetDefaultTabard);

    // Hide hair under helmets: replace style-specific scalp with bald scalp
    // HEAD slot is index 0 in the 19-element equipment array
    if (displayInfoIds[0] != 0 && hairStyleId > 0 &&
        core::helmHidesHair(*assetManager_, displayInfoIds[0], st.genderId)) {
        geosets.erase(selectedHairScalp);                              // Remove style scalp
        geosets.insert(1);    // Bald scalp cap (group 0)
    }

    charRenderer->setActiveGeosets(st.instanceId, geosets);

    // --- Helmet model attachment ---
    // HEAD slot is index 0 in the 19-element equipment array.
    // Helmet M2s are race/gender-specific (e.g. Helm_Plate_B_01_HuM.m2 for Human Male).
    if (displayInfoIds[0] != 0) {
        // Only the helm point — detaching 0 as well would drop the shield.
        charRenderer->detachWeapon(st.instanceId, kAttachHelm);

        const core::HelmVisual helm = core::resolveHelmVisual(
            *assetManager_, displayInfoIds[0], st.raceId, st.genderId);
        if (helm.valid()) {
            pipeline::M2Model helmModel;
            std::string helmPath;
            if (!helm.racialModelPath.empty()) {
                helmPath = helm.racialModelPath;
                if (!loadWeaponM2(helmPath, helmModel)) helmModel = {};
            }
            if (!helmModel.isValid()) {
                helmPath = helm.baseModelPath;
                loadWeaponM2(helmPath, helmModel);
            }

            if (helmModel.isValid()) {
                const uint32_t helmModelId = nextWeaponModelId_++;
                // Attachment point 0 (head bone), fallback to 11 (explicit head).
                const bool attached = charRenderer->attachWeapon(
                    st.instanceId, kAttachHelm, helmModel, helmModelId, helm.texturePath);
                if (attached) {
                    LOG_DEBUG("Attached player helmet: ", helmPath, " tex: ", helm.texturePath);
                }
            }
        }
    } else {
        // No helmet equipped — detach any existing helmet model
        charRenderer->detachWeapon(st.instanceId, kAttachHelm);
    }

    // --- Shoulder model attachment ---
    // SHOULDERS slot is index 2 in the 19-element equipment array.
    // Shoulders have TWO M2 models (left + right) attached at points 5 and 6.
    // ItemDisplayInfo.dbc: LeftModel → left shoulder, RightModel → right shoulder.
    if (displayInfoIds[2] != 0) {
        // Detach any previously attached shoulder models
        charRenderer->detachWeapon(st.instanceId, 5);
        charRenderer->detachWeapon(st.instanceId, 6);

        int32_t shoulderIdx = displayInfoDbc->findRecordById(displayInfoIds[2]);
        if (shoulderIdx >= 0) {
            const uint32_t leftModelField = idiL ? (*idiL)["LeftModel"] : 1u;
            const uint32_t rightModelField = idiL ? (*idiL)["RightModel"] : 2u;
            const uint32_t leftTexField = idiL ? (*idiL)["LeftModelTexture"] : 3u;
            const uint32_t rightTexField = idiL ? (*idiL)["RightModelTexture"] : 4u;

            // Race/gender suffix for shoulder variants (same as helmets)
            static const std::unordered_map<uint8_t, std::string> shoulderRacePrefix = {
                {1, "Hu"}, {2, "Or"}, {3, "Dw"}, {4, "Ni"}, {5, "Sc"},
                {6, "Ta"}, {7, "Gn"}, {8, "Tr"}, {10, "Be"}, {11, "Dr"}
            };
            std::string genderSuffix = (st.genderId == 0) ? "M" : "F";
            std::string raceSuffix;
            auto itRace = shoulderRacePrefix.find(st.raceId);
            if (itRace != shoulderRacePrefix.end()) {
                raceSuffix = "_" + itRace->second + genderSuffix;
            }

            // Attach left shoulder (attachment point 5) using LeftModel
            std::string leftModelName = displayInfoDbc->getString(static_cast<uint32_t>(shoulderIdx), leftModelField);
            if (!leftModelName.empty()) {
                size_t dotPos = leftModelName.rfind('.');
                if (dotPos != std::string::npos) leftModelName = leftModelName.substr(0, dotPos);

                std::string leftPath;
                pipeline::M2Model leftModel;
                if (!raceSuffix.empty()) {
                    leftPath = "Item\\ObjectComponents\\Shoulder\\" + leftModelName + raceSuffix + ".m2";
                    if (!loadWeaponM2(leftPath, leftModel)) leftModel = {};
                }
                if (!leftModel.isValid()) {
                    leftPath = "Item\\ObjectComponents\\Shoulder\\" + leftModelName + ".m2";
                    loadWeaponM2(leftPath, leftModel);
                }

                if (leftModel.isValid()) {
                    uint32_t leftModelId = nextWeaponModelId_++;
                    std::string leftTexName = displayInfoDbc->getString(static_cast<uint32_t>(shoulderIdx), leftTexField);
                    std::string leftTexPath;
                    if (!leftTexName.empty()) {
                        if (!raceSuffix.empty()) {
                            std::string suffixedTex = "Item\\ObjectComponents\\Shoulder\\" + leftTexName + raceSuffix + ".blp";
                            if (assetManager_->fileExists(suffixedTex)) leftTexPath = suffixedTex;
                        }
                        if (leftTexPath.empty()) {
                            leftTexPath = "Item\\ObjectComponents\\Shoulder\\" + leftTexName + ".blp";
                        }
                    }
                    bool attached = charRenderer->attachWeapon(st.instanceId, 5, leftModel, leftModelId, leftTexPath);
                    if (attached) {
                        LOG_DEBUG("Attached left shoulder: ", leftPath, " tex: ", leftTexPath);
                    }
                }
            }

            // Attach right shoulder (attachment point 6) using RightModel
            std::string rightModelName = displayInfoDbc->getString(static_cast<uint32_t>(shoulderIdx), rightModelField);
            if (!rightModelName.empty()) {
                size_t dotPos = rightModelName.rfind('.');
                if (dotPos != std::string::npos) rightModelName = rightModelName.substr(0, dotPos);

                std::string rightPath;
                pipeline::M2Model rightModel;
                if (!raceSuffix.empty()) {
                    rightPath = "Item\\ObjectComponents\\Shoulder\\" + rightModelName + raceSuffix + ".m2";
                    if (!loadWeaponM2(rightPath, rightModel)) rightModel = {};
                }
                if (!rightModel.isValid()) {
                    rightPath = "Item\\ObjectComponents\\Shoulder\\" + rightModelName + ".m2";
                    loadWeaponM2(rightPath, rightModel);
                }

                if (rightModel.isValid()) {
                    uint32_t rightModelId = nextWeaponModelId_++;
                    std::string rightTexName = displayInfoDbc->getString(static_cast<uint32_t>(shoulderIdx), rightTexField);
                    std::string rightTexPath;
                    if (!rightTexName.empty()) {
                        if (!raceSuffix.empty()) {
                            std::string suffixedTex = "Item\\ObjectComponents\\Shoulder\\" + rightTexName + raceSuffix + ".blp";
                            if (assetManager_->fileExists(suffixedTex)) rightTexPath = suffixedTex;
                        }
                        if (rightTexPath.empty()) {
                            rightTexPath = "Item\\ObjectComponents\\Shoulder\\" + rightTexName + ".blp";
                        }
                    }
                    bool attached = charRenderer->attachWeapon(st.instanceId, 6, rightModel, rightModelId, rightTexPath);
                    if (attached) {
                        LOG_DEBUG("Attached right shoulder: ", rightPath, " tex: ", rightTexPath);
                    }
                }
            }
        }
    } else {
        // No shoulders equipped — detach any existing shoulder models
        charRenderer->detachWeapon(st.instanceId, 5);
        charRenderer->detachWeapon(st.instanceId, 6);
    }

    // --- Cape texture (group 15 / texture type 2) ---
    // The geoset above enables the cape mesh, but without a texture it renders blank.
    if (hasInvType({16})) {
        // Back/cloak is WoW equipment slot 14 (BACK) in the 19-element array.
        uint32_t capeDid = displayInfoIds[14];
        if (capeDid != 0) {
            int32_t capeRecIdx = displayInfoDbc->findRecordById(capeDid);
            if (capeRecIdx >= 0) {
                const uint32_t leftTexField = idiL ? (*idiL)["LeftModelTexture"] : 3u;
                // RightModelTexture is the field right after LeftModelTexture.
                // Some cloaks (e.g. Jaina's Radiance) carry their texture only in
                // the right field; the character-preview screen checks both, so
                // match it here — otherwise the world model shows the cape mesh
                // untextured even though the paperdoll preview looks correct.
                const uint32_t rightTexField = leftTexField + 1;
                std::string leftName = displayInfoDbc->getString(
                    static_cast<uint32_t>(capeRecIdx), leftTexField);
                std::string rightName = displayInfoDbc->getString(
                    static_cast<uint32_t>(capeRecIdx), rightTexField);

                std::vector<std::string> capeNames;
                auto addCapeName = [&](const std::string& n) {
                    if (!n.empty() &&
                        std::find(capeNames.begin(), capeNames.end(), n) == capeNames.end())
                        capeNames.push_back(n);
                };
                if (st.genderId == 1) { addCapeName(rightName); addCapeName(leftName); }
                else                  { addCapeName(leftName);  addCapeName(rightName); }

                if (!capeNames.empty()) {
                    auto hasBlpExt = [](const std::string& p) {
                        if (p.size() < 4) return false;
                        std::string ext = p.substr(p.size() - 4);
                        std::transform(ext.begin(), ext.end(), ext.begin(),
                                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                        return ext == ".blp";
                    };

                    std::vector<std::string> capeCandidates;
                    auto addCapeCandidate = [&](const std::string& p) {
                        if (p.empty()) return;
                        if (std::find(capeCandidates.begin(), capeCandidates.end(), p) == capeCandidates.end()) {
                            capeCandidates.push_back(p);
                        }
                    };

                    for (std::string capeName : capeNames) {
                        std::replace(capeName.begin(), capeName.end(), '/', '\\');
                        const bool hasDir = (capeName.find('\\') != std::string::npos);
                        const bool hasExt = hasBlpExt(capeName);
                        if (hasDir) {
                            if (hasExt) addCapeCandidate(capeName);
                            else addCapeCandidate(capeName + ".blp");
                        } else {
                            std::string baseObj = "Item\\ObjectComponents\\Cape\\" + capeName;
                            std::string baseTex = "Item\\TextureComponents\\Cape\\" + capeName;
                            if (hasExt) {
                                addCapeCandidate(baseObj);
                                addCapeCandidate(baseTex);
                            } else {
                                addCapeCandidate(baseObj + ".blp");
                                addCapeCandidate(baseTex + ".blp");
                            }
                            addCapeCandidate(baseObj + (st.genderId == 1 ? "_F.blp" : "_M.blp"));
                            addCapeCandidate(baseObj + "_U.blp");
                            addCapeCandidate(baseTex + (st.genderId == 1 ? "_F.blp" : "_M.blp"));
                            addCapeCandidate(baseTex + "_U.blp");
                        }
                    }

                    const rendering::VkTexture* whiteTex = charRenderer->loadTexture("");
                    rendering::VkTexture* capeTexture = nullptr;
                    for (const auto& candidate : capeCandidates) {
                        rendering::VkTexture* tex = charRenderer->loadTexture(candidate);
                        if (tex && tex != whiteTex) {
                            capeTexture = tex;
                            break;
                        }
                    }

                    if (capeTexture) {
                        charRenderer->setGroupTextureOverride(st.instanceId, 15, capeTexture);
                        if (const auto* md = charRenderer->getModelData(st.modelId)) {
                            for (size_t ti = 0; ti < md->textures.size(); ti++) {
                                if (md->textures[ti].type == 2) {
                                    charRenderer->setTextureSlotOverride(
                                        st.instanceId, static_cast<uint16_t>(ti), capeTexture);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // --- Textures (skin atlas compositing) ---
    static constexpr const char* componentDirs[] = {
        "ArmUpperTexture",
        "ArmLowerTexture",
        "HandTexture",
        "TorsoUpperTexture",
        "TorsoLowerTexture",
        "LegUpperTexture",
        "LegLowerTexture",
        "FootTexture",
    };

    uint32_t texRegionFields[8];
    pipeline::getItemDisplayInfoTextureFields(*displayInfoDbc, idiL, texRegionFields);

    std::vector<std::pair<int, std::string>> regionLayers;
    const bool isFemale = (st.genderId == 1);

    for (int s = 0; s < 19; s++) {
        uint32_t did = displayInfoIds[s];
        if (did == 0) continue;
        int32_t recIdx = displayInfoDbc->findRecordById(did);
        if (recIdx < 0) continue;

        for (int region = 0; region < 8; region++) {
            std::string texName = displayInfoDbc->getString(
                static_cast<uint32_t>(recIdx), texRegionFields[region]);
            if (texName.empty()) continue;

            std::string base = "Item\\TextureComponents\\" + std::string(componentDirs[region]) + "\\" + texName;
            std::string genderPath = base + (isFemale ? "_F.blp" : "_M.blp");
            std::string unisexPath = base + "_U.blp";
            std::string basePath = base + ".blp";
            std::string fullPath;
            if (assetManager_->fileExists(genderPath)) fullPath = genderPath;
            else if (assetManager_->fileExists(unisexPath)) fullPath = unisexPath;
            else if (assetManager_->fileExists(basePath)) fullPath = basePath;
            else continue;

            regionLayers.emplace_back(region, fullPath);
        }
    }

    const auto slotsIt = playerTextureSlotsByModelId_.find(st.modelId);
    if (slotsIt == playerTextureSlotsByModelId_.end()) return;
    const PlayerTextureSlots& slots = slotsIt->second;
    if (slots.skin < 0) return;

    rendering::VkTexture* newTex = charRenderer->compositeWithRegions(st.bodySkinPath, st.underwearPaths, regionLayers);
    if (newTex) {
        charRenderer->setTextureSlotOverride(st.instanceId, static_cast<uint16_t>(slots.skin), newTex);
    }

    // --- Weapon model attachment ---
    // Slot indices in the 19-element EquipSlot array:
    //   15 = MAIN_HAND → attachment 1 (right hand)
    //   16 = OFF_HAND  → attachment 2 (left hand)
    struct OnlineWeaponSlot {
        int slotIndex;
        uint32_t attachmentId;
    };
    static constexpr OnlineWeaponSlot weaponSlots[] = {
        { 15, 1 },  // MAIN_HAND → right hand
        { 16, 2 },  // OFF_HAND  → left hand
    };

    const uint32_t modelFieldL = idiL ? (*idiL)["LeftModel"] : 1u;
    const uint32_t modelFieldR = idiL ? (*idiL)["RightModel"] : 2u;
    const uint32_t texFieldL   = idiL ? (*idiL)["LeftModelTexture"] : 3u;
    const uint32_t texFieldR   = idiL ? (*idiL)["RightModelTexture"] : 4u;

    for (const auto& ws : weaponSlots) {
        uint32_t weapDisplayId = displayInfoIds[ws.slotIndex];
        if (weapDisplayId == 0) {
            charRenderer->detachWeapon(st.instanceId, ws.attachmentId);
            continue;
        }

        int32_t recIdx = displayInfoDbc->findRecordById(weapDisplayId);
        if (recIdx < 0) {
            charRenderer->detachWeapon(st.instanceId, ws.attachmentId);
            continue;
        }

        // Prefer LeftModel (full weapon), fall back to RightModel (hilt variants)
        std::string modelName = displayInfoDbc->getString(static_cast<uint32_t>(recIdx), modelFieldL);
        std::string textureName = displayInfoDbc->getString(static_cast<uint32_t>(recIdx), texFieldL);
        if (modelName.empty()) {
            modelName = displayInfoDbc->getString(static_cast<uint32_t>(recIdx), modelFieldR);
            textureName = displayInfoDbc->getString(static_cast<uint32_t>(recIdx), texFieldR);
        }
        if (modelName.empty()) {
            charRenderer->detachWeapon(st.instanceId, ws.attachmentId);
            continue;
        }

        // Convert .mdx → .m2
        std::string modelFile = modelName;
        {
            size_t dotPos = modelFile.rfind('.');
            if (dotPos != std::string::npos) modelFile = modelFile.substr(0, dotPos);
            modelFile += ".m2";
        }

        // Try Weapon directory first, then Shield
        std::string m2Path = "Item\\ObjectComponents\\Weapon\\" + modelFile;
        pipeline::M2Model weaponModel;
        if (!loadWeaponM2(m2Path, weaponModel)) {
            m2Path = "Item\\ObjectComponents\\Shield\\" + modelFile;
            if (!loadWeaponM2(m2Path, weaponModel)) {
                charRenderer->detachWeapon(st.instanceId, ws.attachmentId);
                continue;
            }
        }

        // Build texture path
        std::string texturePath;
        if (!textureName.empty()) {
            texturePath = "Item\\ObjectComponents\\Weapon\\" + textureName + ".blp";
            if (!assetManager_->fileExists(texturePath)) {
                texturePath = "Item\\ObjectComponents\\Shield\\" + textureName + ".blp";
                if (!assetManager_->fileExists(texturePath)) texturePath.clear();
            }
        }

        uint32_t weaponModelId = nextWeaponModelId_++;
        charRenderer->attachWeapon(st.instanceId, ws.attachmentId,
                                   weaponModel, weaponModelId, texturePath);
    }
}

void EntitySpawner::despawnPlayer(uint64_t guid) {
    if (!renderer_ || !renderer_->getCharacterRenderer()) return;
    pendingRemotePlayerMounts_.erase(guid);
    removeRemotePlayerMount(guid);
    auto it = playerInstances_.find(guid);
    if (it == playerInstances_.end()) return;
    auto* charRenderer = renderer_->getCharacterRenderer();
    // Player composites get a fresh model id per spawn (unlike displayId-keyed
    // creature models, which stay cached), so free the model with the instance.
    const uint32_t compositeModelId = charRenderer->getInstanceModelId(it->second);
    charRenderer->removeInstance(it->second);
    if (compositeModelId != 0) charRenderer->unloadModelIfUnused(compositeModelId);
    playerInstances_.erase(it);
    onlinePlayerAppearance_.erase(guid);
    pendingOnlinePlayerEquipment_.erase(guid);
    deadCreatureGuids_.erase(guid);
    creatureRenderPosCache_.erase(guid);
    creatureSwimmingState_.erase(guid);
    creatureWalkingState_.erase(guid);
    creatureFlyingState_.erase(guid);
    creatureWasMoving_.erase(guid);
    creatureWasSwimming_.erase(guid);
    creatureWasFlying_.erase(guid);
    creatureWasWalking_.erase(guid);
}

// ---------------------------------------------------------------------------
// Which hull a transport is drawn with
//
// Entry and displayId are different numbering spaces and this table used to mix
// them, which is how an elevator became a zeppelin. Every ship and zeppelin
// entry here shares one of five displayIds -- 3015, 3031, 7087, 7446, 7546 --
// and the numbers that were being compared against displayId were entries:
// 164871, 175080 and 176495 are the three vanilla zeppelins, and no displayId
// reaches six figures, so those three could never match.
//
// What did match was worse. 807 and 808 are the displayIds of Gnomeregan lifts
// (the "Vator" and the "Plunger"), 2454 belongs to the Searing Gorge scaffold
// cars and 1587 to a GameObject named, plainly, "Elevator" -- so every one of
// them was being drawn as an airship. Elevators are transports too, which is
// why the caller's guard lets them in.
//
// Values verified against gameobject_template.
std::string EntitySpawner::transportModelPath(uint32_t entry, uint32_t displayId) {
    struct TransportModel { uint32_t entry; uint32_t displayId; const char* path; };
    static constexpr const char* kShip     = "World\\wmo\\transports\\transport_ship\\transportship.wmo";
    static constexpr const char* kZeppelin = "World\\wmo\\transports\\transport_zeppelin\\transport_zeppelin.wmo";
    static constexpr const char* kHordeZep = "World\\wmo\\transports\\transport_horde_zeppelin\\Transport_Horde_Zeppelin.wmo";
    static constexpr const char* kIceship  = "World\\wmo\\transports\\icebreaker\\Transport_Icebreaker_ship.wmo";
    static constexpr TransportModel kTransportModels[] = {
        // Ships (display 3015)
        {  20808, 3015, kShip },      // The Maiden's Fancy
        { 176231, 3015, kShip },      // The Lady Mehley
        { 176310, 3015, kShip },      // The Bravery
        // Zeppelins (display 3031)
        { 164871, 3031, kZeppelin },  // The Thundercaller
        { 175080, 3031, kZeppelin },  // The Iron Eagle
        { 176495, 3031, kZeppelin },  // The Purple Princess
        { 186371, 3031, kZeppelin },
        { 190549, 3031, kZeppelin },  // The Zephyr
        // Horde zeppelins (display 7546)
        { 181689, 7546, kHordeZep },  // Cloudkisser
        { 186238, 7546, kHordeZep },  // The Mighty Wind
        { 201834, 7546, kHordeZep },
        // Icebreakers (display 7446)
        { 181688, 7446, kIceship },   // Northspear
        { 190536, 7446, kIceship },   // Stormwind's Pride
    };

    for (const TransportModel& t : kTransportModels) {
        if (entry == t.entry || displayId == t.displayId) return t.path;
    }
    // The Deeprun Tram car, which is an M2 rather than a WMO and is keyed on a
    // displayId that is genuinely a displayId: entries 176080-176086 all carry
    // 3831.
    if (displayId == 3831) {
        return "World\\Generic\\Gnome\\Passive Doodads\\Subway\\SubwayCar.m2";
    }
    return "";
}

void EntitySpawner::spawnOnlineGameObject(uint64_t guid, uint32_t entry, uint32_t displayId, float x, float y, float z, float orientation, float scale) {
    if (!renderer_ || !assetManager_) return;

    if (!gameObjectLookupsBuilt_) {
        buildGameObjectDisplayLookups();
    }
    if (!gameObjectLookupsBuilt_) return;

    LOG_DEBUG("GO spawn attempt: guid=0x", std::hex, guid, std::dec,
             " displayId=", displayId, " entry=", entry,
             " pos=(", x, ", ", y, ", ", z, ")");

    auto goIt = gameObjectInstances_.find(guid);
    if (goIt != gameObjectInstances_.end()) {
        // A tracked instance ID is only meaningful while the renderer still holds
        // it. Renderer-wide clears (map change, device reset) drop instances
        // without going through despawnGameObject(), which used to leave this map
        // pointing at a dead handle — every later server CREATE for that GUID then
        // took the position-update path below and the object stayed invisible for
        // the rest of the session. Treat a dead handle as "not spawned".
        bool instanceAlive = false;
        if (renderer_) {
            if (goIt->second.isWmo) {
                auto* wr = renderer_->getWMORenderer();
                instanceAlive = wr && wr->hasInstance(goIt->second.instanceId);
            } else {
                auto* mr = renderer_->getM2Renderer();
                instanceAlive = mr && mr->hasInstance(goIt->second.instanceId);
            }
        }
        if (!instanceAlive) {
            LOG_WARNING("GO render instance vanished — respawning: guid=0x", std::hex, guid, std::dec,
                        " displayId=", displayId, " instanceId=", goIt->second.instanceId);
            gameObjectInstances_.erase(goIt);
            goIt = gameObjectInstances_.end();
        }
    }
    if (goIt != gameObjectInstances_.end()) {
        if (gameHandler_ && gameHandler_->isTransportGuid(guid)) {
            if (auto* transportManager = gameHandler_->getTransportManager()) {
                if (transportManager->getTransport(guid)) {
                    transportManager->rebindTransportInstance(
                        guid, goIt->second.instanceId, !goIt->second.isWmo, displayId);
                    transportManager->updateServerTransport(
                        guid, glm::vec3(x, y, z), orientation);
                } else {
                    gameHandler_->notifyTransportSpawned(guid, entry, displayId, x, y, z, orientation);
                }
            } else {
                gameHandler_->notifyTransportSpawned(guid, entry, displayId, x, y, z, orientation);
            }
            return;
        }

        // Already have a render instance — update its position (e.g. transport re-creation)
        auto& info = goIt->second;
        glm::vec3 renderPos = core::coords::canonicalToRender(glm::vec3(x, y, z));
        LOG_DEBUG("GameObject position update: displayId=", displayId, " guid=0x", std::hex, guid, std::dec,
                 " pos=(", x, ", ", y, ", ", z, ")");
        if (renderer_) {
            if (info.isWmo) {
                if (auto* wr = renderer_->getWMORenderer()) {
                    glm::mat4 transform(1.0f);
                    transform = glm::translate(transform, renderPos);
                    transform = glm::rotate(transform, orientation, glm::vec3(0, 0, 1));
                    wr->setInstanceTransform(info.instanceId, transform);
                }
            } else {
                if (auto* mr = renderer_->getM2Renderer()) {
                    glm::mat4 transform(1.0f);
                    transform = glm::translate(transform, renderPos);
                    mr->setInstanceTransform(info.instanceId, transform);
                }
            }
        }
        return;
    }

    std::string modelPath;

    if (gameHandler_ && gameHandler_->isTransportGuid(guid)) {
        modelPath = transportModelPath(entry, displayId);
        if (!modelPath.empty()) {
            LOG_INFO("Transport entry/display ", entry, "/", displayId, " → ", modelPath);
        }
    }

    // Fallback to normal displayId lookup if not a transport or no override matched
    if (modelPath.empty()) {
        modelPath = getGameObjectModelPathForDisplayId(displayId);
    }

    if (modelPath.empty()) {
        LOG_WARNING("No model path for gameobject displayId ", displayId, " (guid 0x", std::hex, guid, std::dec, ")");
        return;
    }

    // Log spawns to help debug duplicate objects (e.g., cathedral issue)
    LOG_DEBUG("GameObject spawn: displayId=", displayId, " guid=0x", std::hex, guid, std::dec,
             " model=", modelPath, " pos=(", x, ", ", y, ", ", z, ")");

    std::string lowerPath = modelPath;
    std::transform(lowerPath.begin(), lowerPath.end(), lowerPath.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    bool isWmo = lowerPath.size() >= 4 && lowerPath.substr(lowerPath.size() - 4) == ".wmo";

    glm::vec3 renderPos = core::coords::canonicalToRender(glm::vec3(x, y, z));
    const float renderYawWmo = orientation;
    // M2 game objects: model default faces +renderX. renderYaw = canonical + 90° = server_yaw
    // (same offset as creature/character renderer_ so all M2 models face consistently)
    const float renderYawM2go = orientation + glm::radians(90.0f);

    bool loadedAsWmo = false;
    if (isWmo) {
        auto* wmoRenderer = renderer_->getWMORenderer();
        if (!wmoRenderer) return;

        uint32_t modelId = 0;
        auto itCache = gameObjectDisplayIdWmoCache_.find(displayId);
        if (itCache != gameObjectDisplayIdWmoCache_.end()) {
            modelId = itCache->second;
            // Only use cached entry if the model is still resident in the renderer_
            if (wmoRenderer->isModelLoaded(modelId)) {
                loadedAsWmo = true;
            } else {
                gameObjectDisplayIdWmoCache_.erase(itCache);
                modelId = 0;
            }
        }
        if (!loadedAsWmo && modelId == 0) {
            auto wmoData = assetManager_->readFile(modelPath);
            if (!wmoData.empty()) {
                pipeline::WMOModel wmoModel = pipeline::WMOLoader::load(wmoData);
                LOG_DEBUG("Gameobject WMO root loaded: ", modelPath, " nGroups=", wmoModel.nGroups);
                int loadedGroups = 0;
                if (wmoModel.nGroups > 0) {
                    std::string basePath = modelPath;
                    std::string extension;
                    if (basePath.size() > 4) {
                        extension = basePath.substr(basePath.size() - 4);
                        std::string extLower = extension;
                        for (char& c : extLower) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                        if (extLower == ".wmo") {
                            basePath = basePath.substr(0, basePath.size() - 4);
                        }
                    }

                    for (uint32_t gi = 0; gi < wmoModel.nGroups; gi++) {
                        char groupSuffix[16];
                        snprintf(groupSuffix, sizeof(groupSuffix), "_%03u%s", gi, extension.c_str());
                        std::string groupPath = basePath + groupSuffix;
                        std::vector<uint8_t> groupData = assetManager_->readFile(groupPath);
                        if (groupData.empty()) {
                            snprintf(groupSuffix, sizeof(groupSuffix), "_%03u.wmo", gi);
                            groupData = assetManager_->readFile(basePath + groupSuffix);
                        }
                        if (groupData.empty()) {
                            snprintf(groupSuffix, sizeof(groupSuffix), "_%03u.WMO", gi);
                            groupData = assetManager_->readFile(basePath + groupSuffix);
                        }
                        if (!groupData.empty()) {
                            pipeline::WMOLoader::loadGroup(groupData, wmoModel, gi);
                            loadedGroups++;
                        } else {
                            LOG_WARNING("  Failed to load WMO group ", gi, " for: ", basePath);
                        }
                    }
                }

                if (loadedGroups > 0 || wmoModel.nGroups == 0) {
                    modelId = nextGameObjectWmoModelId_++;
                    if (wmoRenderer->loadModel(wmoModel, modelId)) {
                        gameObjectDisplayIdWmoCache_[displayId] = modelId;
                        loadedAsWmo = true;
                    } else {
                        LOG_WARNING("Failed to load gameobject WMO model: ", modelPath);
                    }
                } else {
                    LOG_WARNING("No WMO groups loaded for gameobject: ", modelPath,
                                " — falling back to M2");
                }
            } else {
                LOG_WARNING("Failed to read gameobject WMO: ", modelPath, " — falling back to M2");
            }
        }

        if (loadedAsWmo) {
            uint32_t instanceId = wmoRenderer->createInstance(modelId, renderPos,
                glm::vec3(0.0f, 0.0f, renderYawWmo), scale);
            if (instanceId == 0) {
                LOG_WARNING("Failed to create gameobject WMO instance for guid 0x", std::hex, guid, std::dec);
                return;
            }

            gameObjectInstances_[guid] = {modelId, instanceId, true};
            LOG_DEBUG("Spawned gameobject WMO: guid=0x", std::hex, guid, std::dec,
                     " displayId=", displayId, " at (", x, ", ", y, ", ", z, ")");

            // Spawn transport WMO doodads (chairs, furniture, etc.) as child M2 instances
            bool isTransport = false;
            if (gameHandler_) {
                std::string lowerModelPath = modelPath;
                std::transform(lowerModelPath.begin(), lowerModelPath.end(), lowerModelPath.begin(),
                               [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                isTransport = (lowerModelPath.find("transport") != std::string::npos);
            }

            auto* m2Renderer = renderer_->getM2Renderer();
            if (m2Renderer && isTransport) {
                const auto* doodadTemplates = wmoRenderer->getDoodadTemplates(modelId);
                if (doodadTemplates && !doodadTemplates->empty()) {
                    constexpr size_t kMaxTransportDoodads = 192;
                    const size_t doodadBudget = std::min(doodadTemplates->size(), kMaxTransportDoodads);
                    LOG_DEBUG("Queueing ", doodadBudget, "/", doodadTemplates->size(),
                             " transport doodads for WMO instance ", instanceId);
                    pendingTransportDoodadBatches_.push_back(PendingTransportDoodadBatch{
                        guid,
                        modelId,
                        instanceId,
                        0,
                        doodadBudget,
                        0,
                        x, y, z,
                        orientation
                    });
                } else {
                LOG_DEBUG("Transport WMO has no doodads or templates not available");
            }
            }

            // Transport GameObjects are not always named "transport" in their WMO path
            // (e.g. elevators/lifts). If the server marks it as a transport, always
            // notify so TransportManager can animate/carry passengers.
            bool isTG = gameHandler_ && gameHandler_->isTransportGuid(guid);
            LOG_DEBUG("WMO GO spawned: guid=0x", std::hex, guid, std::dec,
                       " entry=", entry, " displayId=", displayId,
                       " isTransport=", isTG,
                       " pos=(", x, ", ", y, ", ", z, ")");
            if (isTG) {
                gameHandler_->notifyTransportSpawned(guid, entry, displayId, x, y, z, orientation);
            }

            return;
        }

        // WMO failed — fall through to try as M2
        // Convert .wmo path to .m2 for fallback
        modelPath = modelPath.substr(0, modelPath.size() - 4) + ".m2";
    }

    {
        auto* m2Renderer = renderer_->getM2Renderer();
        if (!m2Renderer) return;

        // Skip displayIds that permanently failed to load (e.g. empty/unsupported M2s).
        // Without this guard the same empty model is re-parsed every frame, causing
        // sustained log spam and wasted CPU.
        if (gameObjectDisplayIdFailedCache_.count(displayId)) return;

        uint32_t modelId = 0;
        auto itCache = gameObjectDisplayIdModelCache_.find(displayId);
        if (itCache != gameObjectDisplayIdModelCache_.end()) {
            modelId = itCache->second;
            if (!m2Renderer->hasModel(modelId)) {
                LOG_WARNING("GO M2 cache hit but model gone: displayId=", displayId,
                            " modelId=", modelId, " path=", modelPath,
                            " — reloading");
                gameObjectDisplayIdModelCache_.erase(itCache);
                itCache = gameObjectDisplayIdModelCache_.end();
            }
        }
        if (itCache == gameObjectDisplayIdModelCache_.end()) {
            modelId = nextGameObjectModelId_++;

            auto m2Data = assetManager_->readFile(modelPath);
            if (m2Data.empty()) {
                LOG_WARNING("Failed to read gameobject M2: ", modelPath);
                gameObjectDisplayIdFailedCache_.insert(displayId);
                return;
            }

            pipeline::M2Model model = pipeline::M2Loader::load(m2Data);
            // Collision classification needs the asset path. Embedded M2 names
            // are often generic and caused herb/grass gameobjects to be treated
            // as solid props.
            model.name = modelPath;
            if (model.vertices.empty()) {
                LOG_WARNING("Failed to parse gameobject M2: ", modelPath);
                gameObjectDisplayIdFailedCache_.insert(displayId);
                return;
            }

            std::string skinPath = modelPath.substr(0, modelPath.size() - 3) + "00.skin";
            auto skinData = assetManager_->readFile(skinPath);
            if (!skinData.empty() && model.version >= 264) {
                pipeline::M2Loader::loadSkin(skinData, model);
            } else if (skinData.empty() && model.version >= 264) {
                LOG_WARNING("GO skin file MISSING for WotLK M2 (no indices/batches): ", skinPath);
            }

            LOG_DEBUG("GO model: ", modelPath, " v=", model.version,
                     " verts=", model.vertices.size(),
                     " idx=", model.indices.size(),
                     " batches=", model.batches.size(),
                     " bones=", model.bones.size(),
                     " skin=", (skinData.empty() ? "MISSING" : "ok"));

            if (!m2Renderer->loadModel(model, modelId)) {
                LOG_WARNING("Failed to load gameobject model: ", modelPath);
                gameObjectDisplayIdFailedCache_.insert(displayId);
                return;
            }

            // Keep game object models resident across the away-and-back cycle.
            // Leaving town drops every instance of them, and the 60s reaper then
            // evicted the model — the log showed PostBoxHuman.m2 (the mailbox)
            // going through exactly that reap/reload churn on every return trip.
            m2Renderer->setModelPinned(modelId, true);
            gameObjectDisplayIdModelCache_[displayId] = modelId;
        }

        uint32_t instanceId = m2Renderer->createInstance(modelId, renderPos,
            glm::vec3(0.0f, 0.0f, renderYawM2go), scale);
        if (instanceId == 0) {
            LOG_WARNING("Failed to create gameobject instance for guid 0x", std::hex, guid, std::dec);
            return;
        }

        // Server game objects are gameplay props, not scenery: exempt them from the
        // adaptive doodad render distance, which drops to ~200 units in a city and
        // was hiding mailboxes/chests well inside the range the server still
        // considers them visible.
        m2Renderer->setInstanceIsGameObject(instanceId, true);

        // Deeprun Tram cars: riding never used real mesh collision to begin with (Z is
        // fully code-locked to the transport's simulated position while boarded, not
        // derived from a floor query), so the solid SubwayCar.m2 body was only ever in
        // the way - reported live as getting physically stuck walking back across a car
        // after crossing it once. Skip collision so the model is purely visual/decorative
        // for movement purposes, matching how the boarding logic already treats it (a
        // proximity/footprint check, not a physical block).
        if (displayId == 3831u) {
            m2Renderer->setSkipCollision(instanceId, true);
        }

        // Transports are driven by the transport system rather than a looping idle.
        bool isTransportGO = gameHandler_ && gameHandler_->isTransportGuid(guid);
        if (!isTransportGO) {
            // Totems play an explicit idle rather than the model's default sequence.
            bool isTotem = false;
            for (uint32_t totemAnim : {245u, 246u, 247u}) {  // TOTEM_SMALL/MEDIUM/LARGE
                if (m2Renderer->hasAnimation(instanceId, totemAnim)) {
                    m2Renderer->setInstanceAnimation(instanceId, totemAnim, true);
                    isTotem = true;
                    break;
                }
            }
            if (!isTotem) applyGameObjectAnimationPolicy(guid, entry, instanceId);
        }

        gameObjectInstances_[guid] = {modelId, instanceId, false};

        // Notify transport system for M2 transports (e.g. Deeprun Tram cars)
        if (gameHandler_ && gameHandler_->isTransportGuid(guid)) {
            LOG_DEBUG("M2 transport spawned: guid=0x", std::hex, guid, std::dec,
                       " entry=", entry, " displayId=", displayId,
                       " instanceId=", instanceId);
            gameHandler_->notifyTransportSpawned(guid, entry, displayId, x, y, z, orientation);
        }
    }

    LOG_DEBUG("Spawned gameobject: guid=0x", std::hex, guid, std::dec,
             " displayId=", displayId, " at (", x, ", ", y, ", ", z, ")");
}

} // namespace core
} // namespace wowee
