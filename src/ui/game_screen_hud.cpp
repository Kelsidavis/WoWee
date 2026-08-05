#include "ui/game_screen.hpp"
#include "core/helm_visual.hpp"
#include "ui/ui_raid_icons.hpp"
#include "ui/ui_colors.hpp"
#include "ui/ui_helpers.hpp"
#include "rendering/vk_context.hpp"
#include "core/application.hpp"
#include "core/appearance_composer.hpp"
#include "addons/addon_manager.hpp"
#include "core/coordinates.hpp"
#include "core/input.hpp"
#include "rendering/renderer.hpp"
#include "rendering/post_process_pipeline.hpp"
#include "rendering/animation_controller.hpp"
#include "rendering/wmo_renderer.hpp"
#include "rendering/terrain_manager.hpp"
#include "rendering/minimap.hpp"
#include "rendering/world_map.hpp"
#include "rendering/character_renderer.hpp"
#include "rendering/camera.hpp"
#include "rendering/camera_controller.hpp"
#include "audio/audio_coordinator.hpp"
#include "audio/audio_engine.hpp"
#include "audio/music_manager.hpp"
#include "game/zone_manager.hpp"
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
#include "pipeline/dbc_layout.hpp"

#include "game/expansion_profile.hpp"
#include "game/character.hpp"
#include "core/logger.hpp"
#include <imgui.h>
#include <imgui_internal.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <sstream>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <cctype>
#include <chrono>
#include <ctime>
#include <limits>

#include <unordered_set>

namespace {
    using namespace wowee::ui::colors;
    using namespace wowee::ui::helpers;
    constexpr auto& kColorGreen      = kGreen;
    constexpr auto& kColorGray       = kGray;

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
    const auto* idiL = pipeline::getActiveDBCLayout()
        ? pipeline::getActiveDBCLayout()->getLayout("ItemDisplayInfo") : nullptr;
    const uint32_t geosetGroup1Field = idiL ? (*idiL)["GeosetGroup1"] : 7;
    const uint32_t geosetGroup3Field = idiL ? (*idiL)["GeosetGroup3"] : 9;

    auto getGeosetGroup = [&](uint32_t displayInfoId, uint32_t fieldIdx) -> uint32_t {
        if (!displayInfoDbc || displayInfoId == 0) return 0;
        int32_t recIdx = displayInfoDbc->findRecordById(displayInfoId);
        if (recIdx < 0) return 0;
        return displayInfoDbc->getUInt32(static_cast<uint32_t>(recIdx), fieldIdx);
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

    std::unordered_set<uint16_t> geosets;
    if (appearanceComposer_) {
        if (auto* gh = app.getGameHandler()) {
            if (const auto* ch = gh->getActiveCharacter()) {
                const uint8_t raceId = static_cast<uint8_t>(ch->race);
                const uint8_t sexId = static_cast<uint8_t>(ch->gender);
                const uint8_t hairStyleId = static_cast<uint8_t>((ch->appearanceBytes >> 16) & 0xFF);
                const uint8_t facialId = ch->facialFeatures;
                geosets = appearanceComposer_->buildDefaultPlayerGeosets(raceId, sexId, hairStyleId, facialId);
            }
        }
    }
    if (geosets.empty()) {
        geosets.insert(0);
        geosets.insert(101);
        geosets.insert(201);
        geosets.insert(301);
        geosets.insert(702);
        geosets.insert(2002);
    }

    auto eraseGroup = [&](uint16_t group) {
        for (auto it = geosets.begin(); it != geosets.end();) {
            if ((*it / 100) == group) it = geosets.erase(it);
            else ++it;
        }
    };

    // Build set of geoset IDs present in the model for validation.
    // Races like Gnome (no 501) and Tauren (only 505) need fallback.
    std::unordered_set<uint16_t> modelGeosets;
    if (const auto* modelData = charRenderer->getInstanceModelData(instanceId)) {
        for (const auto& batch : modelData->batches) {
            modelGeosets.insert(batch.submeshId);
        }
    }

    auto pickGeoset = [&](uint16_t preferred, uint16_t fallback) -> uint16_t {
        if (modelGeosets.empty()) return preferred;
        if (preferred != 0 && modelGeosets.count(preferred) > 0) return preferred;
        if (fallback != 0 && modelGeosets.count(fallback) > 0) return fallback;
        return preferred;
    };

    auto lowestInGroup = [&](uint16_t group) -> uint16_t {
        uint16_t best = 0;
        for (uint16_t g : modelGeosets) {
            if (g / 100 == group && (best == 0 || g < best)) best = g;
        }
        return best;
    };

    eraseGroup(4);
    eraseGroup(5);
    eraseGroup(8);
    eraseGroup(13);
    eraseGroup(15);
    eraseGroup(12);

    // CharGeosets mapping (verified via vertex bounding boxes):
    //   Group 4 (401+) = GLOVES (forearm area, Z~1.1-1.4)
    //   Group 5 (501+) = BOOTS  (shin area, Z~0.1-0.6)
    //   Group 8 (801+) = WRISTBANDS/SLEEVES (controlled by chest armor)
    //   Group 9 (901+) = KNEEPADS
    //   Group 13 (1301+) = TROUSERS/PANTS
    //   Group 15 (1501+) = CAPE/CLOAK
    //   Group 20 (2002) = FEET

    // Gloves: inventoryType 10 → group 4 (forearms)
    {
        uint32_t did = findEquippedDisplayId({10});
        uint32_t gg = getGeosetGroup(did, geosetGroup1Field);
        geosets.insert(pickGeoset(static_cast<uint16_t>(gg > 0 ? 401 + gg : 401), lowestInGroup(4)));
    }

    // Boots: inventoryType 8 → group 5 (shins/lower legs)
    {
        uint32_t did = findEquippedDisplayId({8});
        uint32_t gg = getGeosetGroup(did, geosetGroup1Field);
        uint16_t selectedShin = pickGeoset(static_cast<uint16_t>(gg > 0 ? 501 + gg : core::kGeosetBareShins), lowestInGroup(5));
        geosets.insert(selectedShin);
    }

    // Chest/Shirt: inventoryType 4 (shirt), 5 (chest), 20 (robe)
    // Controls group 8 (wristbands/sleeve length): 801=bare wrists, 802+=sleeve styles
    // Also controls group 13 (trousers) via GeosetGroup[2] for robes
    {
        uint32_t did = findEquippedDisplayId({4, 5, 20});
        uint32_t gg = getGeosetGroup(did, geosetGroup1Field);
        geosets.insert(static_cast<uint16_t>(gg > 0 ? 801 + gg : 801));
        uint32_t gg3 = getGeosetGroup(did, geosetGroup3Field);
        if (gg3 > 0) {
            geosets.insert(static_cast<uint16_t>(1301 + gg3));
        }
    }

    // Kneepads: group 9 (always default 902)
    geosets.insert(902);

    // Legs/Pants: inventoryType 7 → group 13 (trousers/thighs)
    // 1301=bare legs, 1302+=pant/kilt styles
    {
        uint32_t did = findEquippedDisplayId({7});
        uint32_t gg = getGeosetGroup(did, geosetGroup1Field);
        // Only add if robe hasn't already set a kilt geoset
        if (geosets.count(1302) == 0 && geosets.count(1303) == 0) {
            geosets.insert(static_cast<uint16_t>(gg > 0 ? 1301 + gg : 1301));
        }
    }

    // Whether a helm and a cloak are *shown*, which is not the same question as
    // whether one is worn. Both toggles hide the model and neither was reaching
    // the geosets, so turning the cloak off left the cloak's own body geoset on
    // and turning the helm off left the player bald under nothing.
    bool helmShown = true, cloakShown = true;
    if (auto* gh = app.getGameHandler()) {
        helmShown = gh->isHelmVisible();
        cloakShown = gh->isCloakVisible();
    }

    // Back/Cloak: inventoryType 16 → group 15
    geosets.insert((hasEquippedType({16}) && cloakShown) ? 1502 : 1501);

    // Tabard: inventoryType 19 → group 12
    if (hasEquippedType({19})) {
        geosets.insert(1201);
    }

    // Hide hair under a helm: drop the style scalp and put the bald cap on.
    // Group 0 holds the body plus one scalp; 0 itself is the body and stays.
    // The other-player path has always done this, so a helm covered their hair
    // and not yours.
    // Only while it is actually on the head. The show-helm toggle takes the
    // model away and left this branch running, so hiding a helm hid the hair
    // with it and the player stood there bald wearing nothing.
    if (hasEquippedType({1}) && helmShown) {
        const uint32_t headDisplayId = findEquippedDisplayId({1});
        uint8_t genderId = 0;
        if (auto* gh = app.getGameHandler()) {
            if (const auto* ch = gh->getActiveCharacter()) {
                genderId = static_cast<uint8_t>(ch->gender);
            }
        }
        // A circlet, tiara or crown sits over the hair rather than covering it,
        // and the data says which does what — see core::helmHidesHair.
        if (auto* assets = app.getAssetManager();
            assets && core::helmHidesHair(*assets, headDisplayId, genderId)) {
            for (auto it = geosets.begin(); it != geosets.end();) {
                if (*it != 0 && (*it / 100) == 0) it = geosets.erase(it);
                else ++it;
            }
            geosets.insert(1);
        }
    }

    // Groups 17 and 18 are the Death Knight / Night Elf eye glow. Nothing here
    // should ever select one, and the renderer only auto-skips them when no
    // geoset filter is applied — with a filter, whatever is in this set is what
    // gets drawn. Glowing eyes on a character that should not have them come
    // from exactly this, so say so rather than leaving it to be spotted.
    for (uint16_t g : geosets) {
        const uint16_t group = g / 100;
        if (group == 17 || group == 18) {
            LOG_WARNING("Player geosets include eye-glow geoset ", g,
                        " (group ", group, ") — this will draw glowing eyes");
        }
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
    static constexpr const char* componentDirs[] = {
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
    uint32_t texRegionFields[8];
    pipeline::getItemDisplayInfoTextureFields(*displayInfoDbc, idiL, texRegionFields);

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
            } else if (assetManager->fileExists(base + ".blp")) {
                fullPath = base + ".blp";
            } else {
                continue;
            }
            regionLayers.emplace_back(region, fullPath);
        }
    }

    // Re-composite: base skin + underwear + equipment regions
    // Clear composite cache first to prevent stale textures from being reused
    charRenderer->clearCompositeCache();
    // Use per-instance texture override (not model-level) to avoid deleting cached composites.
    uint32_t instanceId = renderer->getCharacterInstanceId();
    auto* newTex = charRenderer->compositeWithRegions(bodySkinPath, underwearPaths, regionLayers);
    if (newTex != nullptr && instanceId != 0) {
        charRenderer->setTextureSlotOverride(instanceId, static_cast<uint16_t>(skinSlot), newTex);
    }

    // Cloak cape texture — separate from skin atlas, uses texture slot type-2 (Object Skin)
    uint32_t cloakSlot = app.getCloakTextureSlotIndex();
    if (cloakSlot > 0 && instanceId != 0) {
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
                    auto* capeTex = charRenderer->loadTexture(capePath);
                    if (capeTex != nullptr) {
                        charRenderer->setTextureSlotOverride(instanceId, static_cast<uint16_t>(cloakSlot), capeTex);
                        LOG_INFO("Cloak texture applied: ", capePath);
                    }
                }
            }
        } else {
            // No cloak equipped — clear override so model's default (white) shows
            charRenderer->clearTextureSlotOverride(instanceId, static_cast<uint16_t>(cloakSlot));
        }
    }
}

// ============================================================
// World Map
// ============================================================

void GameScreen::renderWorldMap(game::GameHandler& gameHandler) {
    auto& app = core::Application::getInstance();
    auto* renderer = app.getRenderer();
    if (!renderer) return;

    auto* wm = renderer->getWorldMap();
    if (!wm) return;

    // Flight master window drives the world map's flight-map (taxi selection)
    // mode: opening SMSG_SHOWTAXINODES opens the map, activating a flight or
    // closing the gossip closes it. A user-dismissed map (Escape / X) closes
    // the flight master window through the onClose handler.
    const bool taxiWanted = gameHandler.isTaxiWindowOpen();
    if (taxiWanted && !wm->isTaxiMapOpen()) {
        auto* gh = &gameHandler;
        wm->openTaxiMap(
            [gh](uint32_t dest) { return gh->getTaxiRouteTo(dest); },
            [gh](uint32_t dest) { gh->activateTaxi(dest); },
            [gh]() { gh->closeTaxi(); });
    } else if (!taxiWanted && wm->isTaxiMapOpen()) {
        wm->closeTaxiMap();
    }

    if (!showWorldMap_ && !wm->isTaxiMapOpen()) return;

    // Keep map name in sync with minimap's map name
    auto* minimap = renderer->getMinimap();
    if (minimap) {
        wm->setMapName(minimap->getMapName());
    }
    wm->setServerExplorationMask(
        gameHandler.getPlayerExploredZoneMasks(),
        gameHandler.hasPlayerExploredZoneMasks());
    // Which zone the player is actually in, rather than which WorldMapArea box
    // they happen to sit deepest inside. The boxes are axis-aligned rectangles
    // around irregular zones and overlap their neighbours heavily, so opening
    // the map could land on a zone the player was only near.
    wm->setPlayerZoneId(gameHandler.getWorldStateZoneId());

    // Party member dots on world map
    {
        std::vector<rendering::WorldMapPartyDot> dots;
        if (gameHandler.isInGroup()) {
            const auto& partyData = gameHandler.getPartyData();
            for (const auto& member : partyData.members) {
                if (!member.isOnline || !member.hasPartyStats) continue;
                if (member.posX == 0 && member.posY == 0) continue;
                // posY → canonical X (north), posX → canonical Y (west)
                float wowX = static_cast<float>(member.posY);
                float wowY = static_cast<float>(member.posX);
                glm::vec3 rpos = core::coords::canonicalToRender(glm::vec3(wowX, wowY, 0.0f));
                auto ent = gameHandler.getEntityManager().getEntity(member.guid);
                uint8_t cid = entityClassId(ent.get());
                ImU32 col = (cid != 0)
                    ? classColorU32(cid, 230)
                    : (member.guid == partyData.leaderGuid
                       ? IM_COL32(255, 210, 0, 230)
                       : IM_COL32(100, 180, 255, 230));
                dots.push_back({ rpos, col, member.name });
            }
        }
        wm->setPartyDots(std::move(dots));
    }

    // Taxi node markers on world map
    {
        std::vector<rendering::WorldMapTaxiNode> taxiNodes;
        const auto& nodes = gameHandler.getTaxiNodes();
        uint32_t currentTaxiNode = gameHandler.getTaxiCurrentNode();
        const bool playerAlliance = gameHandler.isPlayerAlliance();
        taxiNodes.reserve(nodes.size());
        for (const auto& [id, node] : nodes) {
            const bool known = gameHandler.isKnownTaxiNode(id);
            // Undiscovered nodes are shown so the player can see where flight
            // paths exist, but only ones their faction can actually use. A node's
            // faction is inferred from which taxi mount TaxiNodes.dbc lists:
            // own-faction mount → show; both mounts → neutral flight point, show;
            // opposite-faction-only OR no mount at all (boat/zeppelin/script
            // nodes) → hide. Known nodes are always shown.
            if (!known) {
                const bool hasAlliance = node.mountDisplayIdAlliance != 0;
                const bool hasHorde    = node.mountDisplayIdHorde != 0;
                const bool bothFactions = hasAlliance && hasHorde;   // neutral hub
                const bool ownFaction   = playerAlliance ? hasAlliance : hasHorde;
                if (!bothFactions && !ownFaction) continue;
            }
            rendering::WorldMapTaxiNode wtn;
            wtn.id    = node.id;
            wtn.mapId = node.mapId;
            // TaxiNodes.dbc stores server/wire-order coordinates — convert to
            // canonical (X=north, Y=west) like the taxi flight path code does,
            // or the markers land transposed on the map.
            glm::vec3 canonical = core::coords::serverToCanonical(
                glm::vec3(node.x, node.y, node.z));
            wtn.wowX  = canonical.x;
            wtn.wowY  = canonical.y;
            wtn.wowZ  = canonical.z;
            wtn.name  = node.name;
            wtn.known = known;
            wtn.costCopper = gameHandler.getTaxiCostTo(id);
            wtn.current    = (id == currentTaxiNode);
            wtn.reachable  = gameHandler.hasTaxiRouteTo(id);
            taxiNodes.push_back(std::move(wtn));
        }
        wm->setTaxiNodes(std::move(taxiNodes));
    }

    // Quest objective and quest-giver markers on the world map.
    {
        std::vector<rendering::WorldMap::QuestPoi> qpois;
        const auto& questStatuses = gameHandler.getNpcQuestStatuses();
        // Add authoritative NPC statuses first. Some servers also emit a
        // generic POI at the NPC's position; that duplicate is filtered below
        // so it cannot leave a teal objective circle on a quest giver.
        for (const auto& [guid, status] : questStatuses) {
            auto entity = gameHandler.getEntityManager().getEntity(guid);
            if (!entity || entity->getType() != game::ObjectType::UNIT) continue;

            rendering::WorldMap::QuestPoi qp;
            qp.wowX = entity->getX();
            qp.wowY = entity->getY();
            qp.name = std::static_pointer_cast<game::Unit>(entity)->getName();
            switch (status) {
                case game::QuestGiverStatus::AVAILABLE:
                    qp.kind = rendering::WorldMap::QuestPoi::Kind::AVAILABLE;
                    break;
                case game::QuestGiverStatus::AVAILABLE_LOW:
                    qp.kind = rendering::WorldMap::QuestPoi::Kind::AVAILABLE_LOW;
                    break;
                case game::QuestGiverStatus::REWARD:
                case game::QuestGiverStatus::REWARD_REP:
                    qp.kind = rendering::WorldMap::QuestPoi::Kind::REWARD;
                    break;
                case game::QuestGiverStatus::INCOMPLETE:
                    qp.kind = rendering::WorldMap::QuestPoi::Kind::INCOMPLETE;
                    break;
                default:
                    continue;
            }
            qpois.push_back(std::move(qp));
        }

        constexpr float kQuestGiverPoiMergeDistance = 15.0f;
        constexpr float kQuestGiverPoiMergeDistanceSq =
            kQuestGiverPoiMergeDistance * kQuestGiverPoiMergeDistance;
        for (const auto& poi : gameHandler.getGossipPois()) {
            // Keep ordinary gossip navigation POIs, but only include quest
            // objectives/endpoints explicitly enabled from the tracker.
            if (poi.questObjectiveIndex != -2 &&
                !gameHandler.isQuestShownOnMap(poi.data)) {
                continue;
            }
            bool duplicatesQuestGiver = false;
            for (const auto& existing : qpois) {
                if (existing.kind == rendering::WorldMap::QuestPoi::Kind::OBJECTIVE) continue;
                const float dx = existing.wowX - poi.x;
                const float dy = existing.wowY - poi.y;
                if (dx * dx + dy * dy <= kQuestGiverPoiMergeDistanceSq) {
                    duplicatesQuestGiver = true;
                    break;
                }
            }
            if (duplicatesQuestGiver) continue;

            rendering::WorldMap::QuestPoi qp;
            qp.wowX = poi.x;
            qp.wowY = poi.y;
            qp.name = poi.name;
            if (poi.questObjectiveIndex == -1) {
                // A quest POI with no objective index is the quest endpoint,
                // not an objective area. Completed quests use a yellow ?,
                // while in-progress endpoints use a gray ?.
                qp.kind = rendering::WorldMap::QuestPoi::Kind::INCOMPLETE;
                for (const auto& quest : gameHandler.getQuestLog()) {
                    if (quest.questId != poi.data) continue;
                    if (quest.complete) {
                        qp.kind = rendering::WorldMap::QuestPoi::Kind::REWARD;
                    }
                    break;
                }
            }
            qpois.push_back(std::move(qp));
        }
        wm->setQuestPois(std::move(qpois));
    }

    // Corpse marker: show skull X on world map when ghost with unclaimed corpse
    {
        float corpseCanX = 0.0f, corpseCanY = 0.0f;
        bool ghostWithCorpse = gameHandler.isPlayerGhost() &&
                               gameHandler.getCorpseCanonicalPos(corpseCanX, corpseCanY);
        glm::vec3 corpseRender = ghostWithCorpse
            ? core::coords::canonicalToRender(glm::vec3(corpseCanX, corpseCanY, 0.0f))
            : glm::vec3{};
        wm->setCorpsePos(ghostWithCorpse, corpseRender);
    }

    // Rare tracker: mark every spawned rare / rare-elite the client currently has loaded.
    // Entities only exist while near the player, so a marker means that rare is out now.
    // Opt-in via the Interface setting; when off, feed an empty list so markers clear.
    {
        std::vector<rendering::WorldMapRareMark> rares;
        if (settingsPanel_.showRareTracker_)
        for (const auto& [guid, entity] : gameHandler.getEntityManager().getEntities()) {
            if (!entity || entity->getType() != game::ObjectType::UNIT) continue;
            auto unit = std::static_pointer_cast<game::Unit>(entity);
            const int rank = gameHandler.getCreatureRank(unit->getEntry());
            if (rank != 2 && rank != 4) continue;      // 2 = Rare Elite, 4 = Rare
            if (unit->getHealth() == 0) continue;       // skip dead/looted rares
            rendering::WorldMapRareMark m;
            m.renderPos = core::coords::canonicalToRender(
                glm::vec3(unit->getX(), unit->getY(), unit->getZ()));
            m.name = unit->getName();
            m.rank = rank;
            rares.push_back(std::move(m));
        }
        wm->setRares(std::move(rares));
    }

    // Chest tracker: mark loaded type-3 chest objects while excluding mining and
    // herbalism nodes, which share GAMEOBJECT_TYPE_CHEST on the wire.
    {
        std::vector<rendering::WorldMapChestMark> chests;
        if (settingsPanel_.showChestTracker_) {
            for (const auto& [guid, entity] : gameHandler.getEntityManager().getEntities()) {
                if (!entity || entity->getType() != game::ObjectType::GAMEOBJECT) continue;
                auto chest = std::static_pointer_cast<game::GameObject>(entity);
                const auto* info = gameHandler.getCachedGameObjectInfo(chest->getEntry());
                if (!info || !info->isValid() || info->type != 3) continue;
                if (gameHandler.isGatherGameObject(guid)) continue;

                rendering::WorldMapChestMark mark;
                mark.renderPos = core::coords::canonicalToRender(
                    glm::vec3(chest->getX(), chest->getY(), chest->getZ()));
                mark.name = chest->getName().empty() ? info->name : chest->getName();
                chests.push_back(std::move(mark));
            }
        }
        wm->setChests(std::move(chests));
    }

    glm::vec3 playerPos = renderer->getCharacterPosition();
    float playerYaw = renderer->getCharacterYaw();
    auto* window = app.getWindow();
    int screenW = window ? window->getWidth() : 1280;
    int screenH = window ? window->getHeight() : 720;
    wm->render(playerPos, screenW, screenH, playerYaw);

    // Sync showWorldMap_ if the map closed itself (e.g. ESC key inside the overlay).
    if (!wm->isOpen()) showWorldMap_ = false;
}

// ============================================================
// Action Bar
// ============================================================

VkDescriptorSet GameScreen::getSpellIcon(uint32_t spellId, pipeline::AssetManager* am) {
    if (spellId == 0 || !am) return VK_NULL_HANDLE;

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

        // Load Spell.dbc: SpellIconID field
        auto spellDbc = am->loadDBC("Spell.dbc");
        const auto* spellL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("Spell") : nullptr;
        if (spellDbc && spellDbc->isLoaded()) {
            uint32_t fieldCount = spellDbc->getFieldCount();
            // Helper to load icons for a given field layout
            auto tryLoadIcons = [&](uint32_t idField, uint32_t iconField) {
                spellIconIds_.clear();
                if (iconField >= fieldCount) return;
                for (uint32_t i = 0; i < spellDbc->getRecordCount(); i++) {
                    uint32_t id = spellDbc->getUInt32(i, idField);
                    uint32_t iconId = spellDbc->getUInt32(i, iconField);
                    if (id > 0 && iconId > 0) {
                        spellIconIds_[id] = iconId;
                    }
                }
            };

            // Use the active expansion layout when its fields are present in
            // the loaded DBC. TBC/WotLK/Classic place IconID in different
            // columns, so reading the WotLK default for every client leaves
            // action bars and spell UI without icons.
            uint32_t iconField = 133; // WotLK default
            uint32_t idField = 0;
            if (spellL) {
                try {
                    uint32_t layoutId = (*spellL)["ID"];
                    uint32_t layoutIcon = (*spellL)["IconID"];
                    if (layoutId < fieldCount && layoutIcon < fieldCount) {
                        iconField = layoutIcon;
                        idField = layoutId;
                    }
                } catch (...) {}
            }
            tryLoadIcons(idField, iconField);
        }
    }

    // Rate-limit GPU uploads per frame to prevent stalls when many icons are uncached
    // (e.g., first login, after loading screen, or many new auras appearing at once).
    static int gsLoadsThisFrame = 0;
    static int gsLastImGuiFrame = -1;
    int gsCurFrame = ImGui::GetFrameCount();
    if (gsCurFrame != gsLastImGuiFrame) { gsLoadsThisFrame = 0; gsLastImGuiFrame = gsCurFrame; }
    if (gsLoadsThisFrame >= 4) return VK_NULL_HANDLE;  // defer — do NOT cache null here

    // Look up spellId -> SpellIconID -> icon path
    auto iit = spellIconIds_.find(spellId);
    if (iit == spellIconIds_.end()) {
        spellIconCache_[spellId] = VK_NULL_HANDLE;
        return VK_NULL_HANDLE;
    }

    auto pit = spellIconPaths_.find(iit->second);
    if (pit == spellIconPaths_.end()) {
        spellIconCache_[spellId] = VK_NULL_HANDLE;
        return VK_NULL_HANDLE;
    }

    // Path from DBC has no extension — append .blp
    std::string iconPath = pit->second + ".blp";
    auto blpData = am->readFile(iconPath);
    if (blpData.empty()) {
        spellIconCache_[spellId] = VK_NULL_HANDLE;
        return VK_NULL_HANDLE;
    }

    auto image = pipeline::BLPLoader::load(blpData);
    if (!image.isValid()) {
        spellIconCache_[spellId] = VK_NULL_HANDLE;
        return VK_NULL_HANDLE;
    }

    // Upload to Vulkan via VkContext
    auto* window = services_.window;
    auto* vkCtx = window ? window->getVkContext() : nullptr;
    if (!vkCtx) {
        spellIconCache_[spellId] = VK_NULL_HANDLE;
        return VK_NULL_HANDLE;
    }

    ++gsLoadsThisFrame;
    VkDescriptorSet ds = vkCtx->uploadImGuiTexture(image.data.data(), image.width, image.height);
    spellIconCache_[spellId] = ds;
    return ds;
}

// ============================================================
// Mirror Timers (breath / fatigue / feign death)
// ============================================================

void GameScreen::renderMirrorTimers(game::GameHandler& gameHandler) {
    ImVec2 displaySize = ImGui::GetIO().DisplaySize;
    float screenW = displaySize.x > 0.0f ? displaySize.x : 1280.0f;
    float screenH = displaySize.y > 0.0f ? displaySize.y : 720.0f;

    static constexpr struct { const char* label; ImVec4 color; } kTimerInfo[3] = {
        { "Fatigue", ImVec4(0.8f, 0.4f, 0.1f, 1.0f) },
        { "Breath",  ImVec4(0.2f, 0.5f, 1.0f, 1.0f) },
        { "Feign",   kColorGray },
    };

    float barW  = 280.0f;
    float barH  = 36.0f;
    float barX  = (screenW - barW) / 2.0f;
    float baseY = screenH - 160.0f;  // Just above the cast bar slot

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
                             ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar |
                             ImGuiWindowFlags_NoInputs;

    for (int i = 0; i < 3; ++i) {
        const auto& t = gameHandler.getMirrorTimer(i);
        if (!t.active || t.maxValue <= 0) continue;

        float frac = static_cast<float>(t.value) / static_cast<float>(t.maxValue);
        frac = std::max(0.0f, std::min(1.0f, frac));

        char winId[32];
        std::snprintf(winId, sizeof(winId), "##MirrorTimer%d", i);
        ImGui::SetNextWindowPos(ImVec2(barX, baseY - i * (barH + 4.0f)), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(barW, barH), ImGuiCond_Always);

        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.05f, 0.05f, 0.05f, 0.88f));
        if (ImGui::Begin(winId, nullptr, flags)) {
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, kTimerInfo[i].color);
            char overlay[48];
            float sec = static_cast<float>(t.value) / 1000.0f;
            std::snprintf(overlay, sizeof(overlay), "%s  %.0fs", kTimerInfo[i].label, sec);
            ImGui::ProgressBar(frac, ImVec2(-1, 20), overlay);
            ImGui::PopStyleColor();
        }
        ImGui::End();
        ImGui::PopStyleColor();
        ImGui::PopStyleVar();
    }
}

// ============================================================
// Cooldown Tracker — floating panel showing all active spell CDs
// ============================================================

// ============================================================
// Quest Objective Tracker (right-side HUD)
// ============================================================

void GameScreen::renderQuestObjectiveTracker(game::GameHandler& gameHandler) {
    const auto& questLog = gameHandler.getQuestLog();
    if (questLog.empty()) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;

    constexpr float TRACKER_W = 220.0f;
    constexpr float RIGHT_MARGIN = 10.0f;
    // Build display list: tracked quests only, or all quests if none tracked
    const auto& trackedIds = gameHandler.getTrackedQuestIds();
    const bool anyTracked = !trackedIds.empty();
    std::vector<const game::GameHandler::QuestLogEntry*> toShow;
    toShow.reserve(questLog.size());
    if (anyTracked) {
        for (const auto& q : questLog) {
            if (q.questId == 0) continue;
            if (trackedIds.count(q.questId)) toShow.push_back(&q);
        }
    }
    // Fallback: show all quests if nothing is tracked
    if (toShow.empty()) {
        for (const auto& q : questLog) {
            if (q.questId == 0) continue;
            toShow.push_back(&q);
        }
    }
    if (toShow.empty()) return;

    // Apply filter (0=All, 1=Active, 2=Done, 3=Zone). Keep the window open even when
    // the filter hides everything so the user can cycle it back.
    // Zone filter needs the player's current zone; if it isn't known yet (0), fall
    // back to showing everything rather than an empty tracker.
    const uint32_t currentZoneId = gameHandler.getWorldStateZoneId();
    if (questTrackerFilter_ == 1 || questTrackerFilter_ == 2) {
        std::erase_if(toShow, [this](const game::GameHandler::QuestLogEntry* q) {
            return questTrackerFilter_ == 1 ? q->complete : !q->complete;
        });
    } else if (questTrackerFilter_ == 3 && currentZoneId != 0) {
        std::erase_if(toShow, [currentZoneId](const game::GameHandler::QuestLogEntry* q) {
            // zoneOrSort > 0 is an AreaTable zone id; keep only quests for this zone.
            return q->zoneOrSort <= 0 || static_cast<uint32_t>(q->zoneOrSort) != currentZoneId;
        });
    }

    // Sort lowest level first so stale quests surface at the top;
    // unknown level (0) and player-scaling (-1) sort last
    std::stable_sort(toShow.begin(), toShow.end(),
                     [](const game::GameHandler::QuestLogEntry* a,
                        const game::GameHandler::QuestLogEntry* b) {
        const int la = a->level > 0 ? a->level : std::numeric_limits<int>::max();
        const int lb = b->level > 0 ? b->level : std::numeric_limits<int>::max();
        if (la != lb) return la < lb;
        return a->title < b->title;
    });

    float screenH = ImGui::GetIO().DisplaySize.y > 0.0f ? ImGui::GetIO().DisplaySize.y : 720.0f;

    // Default position: top-right, below minimap + buff bar space.
    // questTrackerRightOffset_ stores pixels from the right edge so the tracker
    // stays anchored to the right side when the window is resized.
    if (!questTrackerPosInit_ || questTrackerRightOffset_ < 0.0f) {
        questTrackerRightOffset_ = TRACKER_W + RIGHT_MARGIN; // default: right-aligned
        questTrackerPos_.y = 320.0f;
        questTrackerPosInit_ = true;
    }
    // Recompute X from right offset every frame (handles window resize)
    questTrackerPos_.x = screenW - questTrackerRightOffset_;

    // Collapsed: draw a small draggable bubble at the tracker anchor instead;
    // click (without dragging) to expand back to the full tracker
    if (questTrackerCollapsed_) {
        ImGui::SetNextWindowPos(questTrackerPos_, ImGuiCond_Always);
        ImGuiWindowFlags bubbleFlags = ImGuiWindowFlags_NoTitleBar |
                                       ImGuiWindowFlags_NoCollapse |
                                       ImGuiWindowFlags_NoNav |
                                       ImGuiWindowFlags_NoBringToFrontOnFocus |
                                       ImGuiWindowFlags_AlwaysAutoResize |
                                       ImGuiWindowFlags_NoScrollbar;
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.55f));
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 12.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(9.0f, 5.0f));
        if (ImGui::Begin("##QuestTrackerBubble", nullptr, bubbleFlags)) {
            ImGui::TextColored(colors::kWarmGold, "! %d", static_cast<int>(toShow.size()));
            if (ImGui::IsWindowHovered()) {
                ImGui::SetTooltip("Quest tracker — click to expand, drag to move");
                // Expand on click, but not when the press was a drag
                if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) &&
                    ImGui::GetIO().MouseDragMaxDistanceSqr[ImGuiMouseButton_Left] < 9.0f) {
                    questTrackerCollapsed_ = false;
                    saveSettings();
                }
            }
            // Capture drag so the bubble and tracker share one anchor
            ImVec2 newPos = ImGui::GetWindowPos();
            newPos.x = std::clamp(newPos.x, 0.0f, screenW - ImGui::GetWindowSize().x);
            newPos.y = std::clamp(newPos.y, 0.0f, screenH - 40.0f);
            if (std::abs(newPos.x - questTrackerPos_.x) > 0.5f ||
                std::abs(newPos.y - questTrackerPos_.y) > 0.5f) {
                questTrackerPos_ = newPos;
                questTrackerRightOffset_ = screenW - newPos.x;
                saveSettings();
            }
        }
        ImGui::End();
        ImGui::PopStyleVar(2);
        ImGui::PopStyleColor();
        return;
    }

    ImGui::SetNextWindowPos(questTrackerPos_, ImGuiCond_Always);
    ImGui::SetNextWindowSize(questTrackerSize_, ImGuiCond_FirstUseEver);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar |
                             ImGuiWindowFlags_NoCollapse |
                             ImGuiWindowFlags_NoNav |
                             ImGuiWindowFlags_NoBringToFrontOnFocus;

    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.55f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(6.0f, 6.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4.0f, 2.0f));

    if (ImGui::Begin("##QuestTracker", nullptr, flags)) {
        // Header row: quest count + completion filter (click to cycle) + hide
        static const char* kFilterNames[] = {"All", "Active", "Done", "Zone"};
        ImGui::TextDisabled("Quests (%d)", static_cast<int>(toShow.size()));
        {
            const ImGuiStyle& style = ImGui::GetStyle();
            float filterW = ImGui::CalcTextSize("Active").x + style.FramePadding.x * 2.0f;
            float hideW = ImGui::CalcTextSize("-").x + style.FramePadding.x * 2.0f;
            ImGui::SameLine();
            float off = ImGui::GetContentRegionAvail().x -
                        (filterW + style.ItemSpacing.x + hideW);
            if (off > 0.0f) ImGui::SetCursorPosX(ImGui::GetCursorPosX() + off);
            if (ImGui::SmallButton(kFilterNames[questTrackerFilter_])) {
                questTrackerFilter_ = (questTrackerFilter_ + 1) % 4;
                saveSettings();
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Filter: All / Active / Done / Zone (click to cycle)");
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("-")) {
                questTrackerCollapsed_ = true;
                saveSettings();
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Hide (collapse to bubble)");
            }
        }
        ImGui::Separator();
        if (toShow.empty()) {
            const char* emptyMsg = "No quests";
            if (questTrackerFilter_ == 1)      emptyMsg = "No active quests";
            else if (questTrackerFilter_ == 2) emptyMsg = "No completed quests";
            else if (questTrackerFilter_ == 3) emptyMsg = "No quests in this zone";
            ImGui::TextDisabled("%s", emptyMsg);
        }

        for (int i = 0; i < static_cast<int>(toShow.size()); ++i) {
            const auto& q = *toShow[i];

            // Per-quest map checkbox + clickable title + small [x] untrack button.
            ImGui::PushID(q.questId);
            bool shownOnMap = gameHandler.isQuestShownOnMap(q.questId);
            if (ImGui::Checkbox("##ShowOnMap", &shownOnMap)) {
                gameHandler.setQuestShownOnMap(q.questId, shownOnMap);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Show this quest's objectives on the minimap and world map");
            }
            ImGui::SameLine();
            float untrackW = ImGui::CalcTextSize("x").x + ImGui::GetStyle().FramePadding.x * 2.0f;
            float titleW = ImGui::GetContentRegionAvail().x - untrackW -
                           ImGui::GetStyle().ItemSpacing.x;
            ImVec4 titleCol = q.complete ? colors::kWarmGold
                                         : ImVec4(1.0f, 1.0f, 0.85f, 1.0f);
            std::string titleLabel = q.level > 0
                ? "[" + std::to_string(q.level) + "] " + q.title
                : q.title;
            ImGui::PushStyleColor(ImGuiCol_Text, titleCol);
            if (ImGui::Selectable(titleLabel.c_str(), false,
                                   ImGuiSelectableFlags_DontClosePopups, ImVec2(titleW, 0))) {
                questLogScreen.openAndSelectQuest(q.questId);
            }
            if (ImGui::IsItemHovered() && !ImGui::IsPopupOpen("##QTCtx")) {
                ImGui::SetTooltip("Click: open Quest Log  |  Right-click: tracking options");
            }
            ImGui::PopStyleColor();

            // Right-click context menu for quest tracker entry (attaches to the
            // title Selectable — must come before the untrack button below)
            if (ImGui::BeginPopupContextItem("##QTCtx")) {
                ImGui::TextDisabled("%s", q.title.c_str());
                ImGui::Separator();
                if (ImGui::MenuItem("Open in Quest Log")) {
                    questLogScreen.openAndSelectQuest(q.questId);
                }
                bool mapVisible = gameHandler.isQuestShownOnMap(q.questId);
                if (ImGui::MenuItem("Show on Map", nullptr, mapVisible)) {
                    gameHandler.setQuestShownOnMap(q.questId, !mapVisible);
                }
                bool tracked = gameHandler.isQuestTracked(q.questId);
                if (tracked) {
                    if (ImGui::MenuItem("Stop Tracking")) {
                        gameHandler.setQuestTracked(q.questId, false);
                    }
                } else {
                    if (ImGui::MenuItem("Track")) {
                        gameHandler.setQuestTracked(q.questId, true);
                    }
                }
                if (gameHandler.isInGroup() && !q.complete) {
                    if (ImGui::MenuItem("Share Quest")) {
                        gameHandler.shareQuestWithParty(q.questId);
                    }
                }
                // Completed quests stay abandonable until they are turned in.
                ImGui::Separator();
                if (ImGui::MenuItem("Abandon Quest")) {
                    gameHandler.abandonQuest(q.questId);
                }
                ImGui::EndPopup();
            }

            // Untrack button at the end of the title row
            ImGui::SameLine();
            if (ImGui::SmallButton("x")) {
                if (anyTracked) {
                    gameHandler.setQuestTracked(q.questId, false);
                } else {
                    // Nothing explicitly tracked (tracker shows all quests):
                    // hide this one by tracking every other quest instead
                    for (const auto& other : questLog) {
                        if (other.questId != 0 && other.questId != q.questId) {
                            gameHandler.setQuestTracked(other.questId, true);
                        }
                    }
                }
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Untrack (remove from tracker)");
            }
            ImGui::PopID();

            // Objectives — every required objective with progress, done ones marked
            if (q.complete) {
                ImGui::TextColored(colors::kActiveGreen, "  Ready to turn in");
            } else {
                constexpr ImVec4 kPendingGray(0.75f, 0.75f, 0.75f, 1.0f);

                // Kill/interact objective line — green + "(done)" when finished
                auto renderKillLine = [&](uint32_t entry, uint32_t count, uint32_t required) {
                    bool objDone = (required > 0 && count >= required);
                    ImVec4 objColor = objDone ? kColorGreen : kPendingGray;
                    std::string name = gameHandler.getCachedCreatureName(entry);
                    if (name.empty()) {
                        const auto* goInfo = gameHandler.getCachedGameObjectInfo(entry);
                        if (goInfo && !goInfo->name.empty()) name = goInfo->name;
                    }
                    if (!name.empty()) {
                        ImGui::TextColored(objColor, "  - %s: %u/%u%s", name.c_str(),
                                           count, required, objDone ? " (done)" : "");
                    } else {
                        ImGui::TextColored(objColor, "  - %u/%u%s",
                                           count, required, objDone ? " (done)" : "");
                    }
                };
                // Quest query data first (covers objectives with no progress yet),
                // then any progress entries the query didn't know about
                std::unordered_set<uint32_t> shownKills;
                for (const auto& obj : q.killObjectives) {
                    if (obj.npcOrGoId == 0 || obj.required == 0) continue;
                    uint32_t entry = static_cast<uint32_t>(
                        obj.npcOrGoId > 0 ? obj.npcOrGoId : -obj.npcOrGoId);
                    uint32_t count = 0;
                    auto it = q.killCounts.find(entry);
                    if (it != q.killCounts.end()) count = it->second.first;
                    renderKillLine(entry, count, obj.required);
                    shownKills.insert(entry);
                }
                for (const auto& [entry, progress] : q.killCounts) {
                    if (shownKills.count(entry)) continue;
                    renderKillLine(entry, progress.first, progress.second);
                }

                // Item objective line — small icon + name, green + "(done)" when finished
                auto renderItemLine = [&](uint32_t itemId, uint32_t count, uint32_t required) {
                    bool objDone = (required > 0 && count >= required);
                    ImVec4 objColor = objDone ? kColorGreen : kPendingGray;
                    const auto* info = gameHandler.getItemInfo(itemId);
                    const char* itemName = (info && !info->name.empty()) ? info->name.c_str() : nullptr;

                    uint32_t dispId = (info && info->displayInfoId) ? info->displayInfoId : 0;
                    VkDescriptorSet iconTex = dispId ? inventoryScreen.getItemIcon(dispId) : VK_NULL_HANDLE;
                    if (iconTex) {
                        ImGui::Image((ImTextureID)(uintptr_t)iconTex, ImVec2(12, 12));
                        if (info && info->valid && ImGui::IsItemHovered()) {
                            ImGui::BeginTooltip();
                            inventoryScreen.renderItemTooltip(*info);
                            ImGui::EndTooltip();
                        }
                        ImGui::SameLine(0, 3);
                        ImGui::TextColored(objColor, "%s: %u/%u%s",
                                           itemName ? itemName : "Item",
                                           count, required, objDone ? " (done)" : "");
                        if (info && info->valid && ImGui::IsItemHovered()) {
                            ImGui::BeginTooltip();
                            inventoryScreen.renderItemTooltip(*info);
                            ImGui::EndTooltip();
                        }
                    } else {
                        ImGui::TextColored(objColor, "  - %s: %u/%u%s",
                                           itemName ? itemName : "Item",
                                           count, required, objDone ? " (done)" : "");
                        if (info && info->valid && ImGui::IsItemHovered()) {
                            ImGui::BeginTooltip();
                            inventoryScreen.renderItemTooltip(*info);
                            ImGui::EndTooltip();
                        }
                    }
                };
                // requiredItemCounts is the authoritative required list (includes
                // 0-progress items); then show stray progress entries not in it
                std::unordered_set<uint32_t> shownItems;
                for (const auto& obj : q.itemObjectives) {
                    if (obj.itemId == 0 || obj.required == 0) continue;
                    uint32_t count = 0;
                    auto it = q.itemCounts.find(obj.itemId);
                    if (it != q.itemCounts.end()) count = it->second;
                    renderItemLine(obj.itemId, count, obj.required);
                    shownItems.insert(obj.itemId);
                }
                for (const auto& [itemId, required] : q.requiredItemCounts) {
                    if (shownItems.count(itemId)) continue;
                    uint32_t count = 0;
                    auto it = q.itemCounts.find(itemId);
                    if (it != q.itemCounts.end()) count = it->second;
                    renderItemLine(itemId, count, required);
                    shownItems.insert(itemId);
                }
                for (const auto& [itemId, count] : q.itemCounts) {
                    if (shownItems.count(itemId)) continue;
                    renderItemLine(itemId, count, 1);
                }

                bool hasCounted = !shownKills.empty() || !shownItems.empty() ||
                                  !q.killCounts.empty() || !q.itemCounts.empty();
                if (!hasCounted && !q.objectives.empty()) {
                    const std::string& obj = q.objectives;
                    if (obj.size() > 40) {
                        ImGui::TextColored(ImVec4(0.75f, 0.75f, 0.75f, 1.0f),
                                           "  %.37s...", obj.c_str());
                    } else {
                        ImGui::TextColored(ImVec4(0.75f, 0.75f, 0.75f, 1.0f),
                                           "  %s", obj.c_str());
                    }
                }
            }

            if (i < static_cast<int>(toShow.size()) - 1) {
                ImGui::Spacing();
            }
        }

        // Capture position and size after drag/resize
        ImVec2 newPos  = ImGui::GetWindowPos();
        ImVec2 newSize = ImGui::GetWindowSize();
        bool changed = false;

        // Clamp within screen
        newPos.x = std::clamp(newPos.x, 0.0f, screenW - newSize.x);
        newPos.y = std::clamp(newPos.y, 0.0f, screenH - 40.0f);

        if (std::abs(newPos.x - questTrackerPos_.x) > 0.5f ||
            std::abs(newPos.y - questTrackerPos_.y) > 0.5f) {
            questTrackerPos_ = newPos;
            // Update right offset so resizes keep the new position anchored
            questTrackerRightOffset_ = screenW - newPos.x;
            changed = true;
        }
        if (std::abs(newSize.x - questTrackerSize_.x) > 0.5f ||
            std::abs(newSize.y - questTrackerSize_.y) > 0.5f) {
            questTrackerSize_ = newSize;
            changed = true;
        }
        if (changed) saveSettings();
    }
    ImGui::End();

    ImGui::PopStyleVar(2);
    ImGui::PopStyleColor();
}

// ============================================================
// Nameplates — world-space health bars projected to screen
// ============================================================

void GameScreen::renderNameplates(game::GameHandler& gameHandler) {
    if (gameHandler.getState() != game::WorldState::IN_WORLD) return;

    // Reset mouseover each frame; we'll set it below when the cursor is over a nameplate
    gameHandler.setMouseoverGuid(0);

    auto* appRenderer = services_.renderer;
    if (!appRenderer) return;
    rendering::Camera* camera = appRenderer->getCamera();
    if (!camera) return;

    auto* window = services_.window;
    if (!window) return;
    const float screenW = static_cast<float>(window->getWidth());
    const float screenH = static_cast<float>(window->getHeight());

    const glm::mat4 viewProj = camera->getProjectionMatrix() * camera->getViewMatrix();
    const glm::vec3 camPos   = camera->getPosition();
    const uint64_t  playerGuid = gameHandler.getPlayerGuid();
    const uint64_t  targetGuid = gameHandler.getTargetGuid();

    refreshQuestObjectiveCache(gameHandler);
    const auto& questKillEntries = minimapQuestCreatureEntries_;

    ImDrawList* drawList = ImGui::GetBackgroundDrawList();
    static thread_local std::vector<std::shared_ptr<game::Entity>> nameplateEntities;
    glm::vec3 playerCanonical(0.0f);
    if (auto player = gameHandler.getEntityManager().getEntity(playerGuid))
        playerCanonical = glm::vec3(player->getX(), player->getY(), player->getZ());
    gameHandler.getEntityManager().getEntitiesNear(
        playerCanonical.x, playerCanonical.y, 150.0f, nameplateEntities);

    for (const auto& entityPtr : nameplateEntities) {
        if (!entityPtr) continue;
        const uint64_t guid = entityPtr->getGuid();
        if (!entityPtr || guid == playerGuid) continue;

        if (!entityPtr->isUnit()) continue;
        auto* unit = static_cast<game::Unit*>(entityPtr.get());
        if (unit->getMaxHealth() == 0) continue;

        bool isPlayer = (entityPtr->getType() == game::ObjectType::PLAYER);
        bool isTarget = (guid == targetGuid);
        const bool isHostile = unit->isHostile();

        // Friendly player nameplates use Shift+V; enemy players and hostile/NPC
        // nameplates use V. Reaction, not object type, owns the visual category.
        // The current target ALWAYS gets a nameplate so it's clear what is
        // selected even with nameplates toggled off.
        if (isPlayer && !isHostile && !settingsPanel_.showFriendlyNameplates_ && !isTarget) continue;
        if ((!isPlayer || isHostile) && !showNameplates_ && !isTarget) continue;

        // For corpses (dead units), only show a minimal grey nameplate if selected
        bool isCorpse = (unit->getHealth() == 0);
        if (isCorpse && !isTarget) continue;

        // Prefer the renderer's actual instance position so the nameplate tracks the
        // rendered model exactly (avoids drift from the parallel entity interpolator).
        glm::vec3 renderPos;
        if (!core::Application::getInstance().getRenderPositionForGuid(guid, renderPos)) {
            renderPos = core::coords::canonicalToRender(
                glm::vec3(unit->getX(), unit->getY(), unit->getZ()));
        }
        renderPos.z += 2.3f;

        // Cull distance: the current target stays visible to 60 units so its
        // bar doesn't vanish at combat range; players 40; other NPCs 20.
        glm::vec3 nameDelta = renderPos - camPos;
        float distSq = glm::dot(nameDelta, nameDelta);
        float cullDist = isTarget ? 60.0f : (isPlayer ? 40.0f : 20.0f);
        if (distSq > cullDist * cullDist) continue;

        // Project to clip space
        glm::vec4 clipPos = viewProj * glm::vec4(renderPos, 1.0f);
        if (clipPos.w <= 0.01f) continue;  // Behind camera

        glm::vec3 ndc = glm::vec3(clipPos) / clipPos.w;
        if (ndc.x < -1.2f || ndc.x > 1.2f || ndc.y < -1.2f || ndc.y > 1.2f) continue;

        // NDC → screen pixels.
        // The camera bakes the Vulkan Y-flip into the projection matrix, so
        // NDC y = -1 is the top of the screen and y = 1 is the bottom.
        // Map directly: sy = (ndc.y + 1) / 2 * screenH  (no extra inversion).
        float sx = (ndc.x * 0.5f + 0.5f) * screenW;
        float sy = (ndc.y * 0.5f + 0.5f) * screenH;

        // Fade out in the last 5 units of cull range
        float fadeSq = (cullDist - 5.0f) * (cullDist - 5.0f);
        float dist = std::sqrt(distSq);
        float alpha = distSq < fadeSq ? 1.0f : 1.0f - (dist - (cullDist - 5.0f)) / 5.0f;
        auto A = [&](int v) { return static_cast<int>(v * alpha); };

        // Bar colour by hostility (grey for corpses)
        ImU32 barColor, bgColor;
        if (isCorpse) {
            // Minimal grey bar for selected corpses (loot/skin targets)
            barColor = IM_COL32(140, 140, 140, A(200));
            bgColor  = IM_COL32(70,  70,  70,  A(160));
        } else if (isHostile) {
            // Check if mob is tapped by another player (grey nameplate)
            uint32_t dynFlags = unit->getDynamicFlags();
            bool tappedByOther = (dynFlags & 0x0004) != 0 && (dynFlags & 0x0008) == 0; // TAPPED but not TAPPED_BY_ALL_THREAT_LIST
            if (tappedByOther) {
                barColor = IM_COL32(160, 160, 160, A(200));
                bgColor  = IM_COL32(80,  80,  80,  A(160));
            } else {
                barColor = IM_COL32(220, 60,  60,  A(200));
                bgColor  = IM_COL32(100, 25,  25,  A(160));
            }
        } else if (isPlayer) {
            // World-space bars communicate reaction. Class colors belong in
            // party/social UI, where red DK and pink Paladin cannot imply hostility.
            barColor = IM_COL32(60,  200, 80,  A(200));
            bgColor  = IM_COL32(25,  100, 35,  A(160));
        } else {
            barColor = IM_COL32(60,  200, 80,  A(200));
            bgColor  = IM_COL32(25,  100, 35,  A(160));
        }
        // Check if this unit is targeting the local player (threat indicator)
        bool isTargetingPlayer = false;
        if (unit->isHostile() && !isCorpse) {
            const auto& fields = entityPtr->getFields();
            auto loIt = fields.find(game::fieldIndex(game::UF::UNIT_FIELD_TARGET_LO));
            if (loIt != fields.end() && loIt->second != 0) {
                uint64_t unitTarget = loIt->second;
                auto hiIt = fields.find(game::fieldIndex(game::UF::UNIT_FIELD_TARGET_HI));
                if (hiIt != fields.end())
                    unitTarget |= (static_cast<uint64_t>(hiIt->second) << 32);
                isTargetingPlayer = (unitTarget == playerGuid);
            }
        }
        // Creature rank for border styling (Elite=gold double border, Boss=red, Rare=silver)
        int creatureRank = -1;
        if (!isPlayer) creatureRank = gameHandler.getCreatureRank(unit->getEntry());

        // Border: gold = currently selected, orange = targeting player, dark = default
        ImU32 borderColor = isTarget
            ? IM_COL32(255, 215, 0,  A(255))
            : isTargetingPlayer
              ? IM_COL32(255, 140, 0,  A(220))   // orange = this mob is targeting you
              : IM_COL32(20,  20,  20, A(180));

        // Bar geometry
        const float barW = 80.0f * settingsPanel_.nameplateScale_;
        const float barH = 8.0f * settingsPanel_.nameplateScale_;
        const float barX = sx - barW * 0.5f;

        // Guard against division by zero when maxHealth hasn't been populated yet
        // (freshly spawned entity with default fields). 0/0 produces NaN which
        // poisons all downstream geometry; +inf is clamped but still wasteful.
        float healthPct = (unit->getMaxHealth() > 0)
            ? std::clamp(static_cast<float>(unit->getHealth()) / static_cast<float>(unit->getMaxHealth()), 0.0f, 1.0f)
            : 0.0f;

        drawList->AddRectFilled(ImVec2(barX,                 sy), ImVec2(barX + barW,               sy + barH), bgColor,    2.0f);
        // For corpses, don't fill health bar (just show grey background)
        if (!isCorpse) {
            drawList->AddRectFilled(ImVec2(barX,                 sy), ImVec2(barX + barW * healthPct,   sy + barH), barColor,   2.0f);
        }
        drawList->AddRect       (ImVec2(barX - 1.0f, sy - 1.0f), ImVec2(barX + barW + 1.0f, sy + barH + 1.0f), borderColor, 2.0f);

        // Elite/Boss/Rare decoration: extra outer border with rank-specific color
        if (creatureRank == 1 || creatureRank == 2) {
            // Elite / Rare Elite: gold double border
            drawList->AddRect(ImVec2(barX - 3.0f, sy - 3.0f),
                              ImVec2(barX + barW + 3.0f, sy + barH + 3.0f),
                              IM_COL32(255, 200, 50, A(200)), 3.0f);
        } else if (creatureRank == 3) {
            // Boss: red double border
            drawList->AddRect(ImVec2(barX - 3.0f, sy - 3.0f),
                              ImVec2(barX + barW + 3.0f, sy + barH + 3.0f),
                              IM_COL32(255, 40, 40, A(200)), 3.0f);
        } else if (creatureRank == 4) {
            // Rare: silver double border
            drawList->AddRect(ImVec2(barX - 3.0f, sy - 3.0f),
                              ImVec2(barX + barW + 3.0f, sy + barH + 3.0f),
                              IM_COL32(170, 200, 230, A(200)), 3.0f);
        }

        // HP % text centered on health bar (non-corpse, non-full-health for readability)
        if (!isCorpse && unit->getMaxHealth() > 0) {
            int hpPct = static_cast<int>(healthPct * 100.0f + 0.5f);
            char hpBuf[8];
            snprintf(hpBuf, sizeof(hpBuf), "%d%%", hpPct);
            ImVec2 hpTextSz = ImGui::CalcTextSize(hpBuf);
            float hpTx = sx - hpTextSz.x * 0.5f;
            float hpTy = sy + (barH - hpTextSz.y) * 0.5f;
            drawList->AddText(ImVec2(hpTx + 1.0f, hpTy + 1.0f), IM_COL32(0, 0, 0, A(140)), hpBuf);
            drawList->AddText(ImVec2(hpTx,         hpTy),         IM_COL32(255, 255, 255, A(200)), hpBuf);
        }

        // Cast bar below health bar when unit is casting
        float castBarBaseY = sy + barH + 2.0f;
        float nameplateBottom = castBarBaseY;  // tracks lowest drawn element for debuff dots
        {
            const auto* cs = gameHandler.getUnitCastState(guid);
            if (cs && cs->casting && cs->timeTotal > 0.0f) {
                float castPct = std::clamp((cs->timeTotal - cs->timeRemaining) / cs->timeTotal, 0.0f, 1.0f);
                const float cbH = 6.0f * settingsPanel_.nameplateScale_;

                // Spell icon + name above the cast bar
                const std::string& spellName = gameHandler.getSpellName(cs->spellId);
                {
                    auto* castAm = services_.assetManager;
                    VkDescriptorSet castIcon = (cs->spellId && castAm)
                        ? getSpellIcon(cs->spellId, castAm) : VK_NULL_HANDLE;
                    float iconSz = cbH + 8.0f;
                    if (castIcon) {
                        // Draw icon to the left of the cast bar
                        float iconX = barX - iconSz - 2.0f;
                        float iconY = castBarBaseY;
                        drawList->AddImage((ImTextureID)(uintptr_t)castIcon,
                                           ImVec2(iconX, iconY),
                                           ImVec2(iconX + iconSz, iconY + iconSz));
                        drawList->AddRect(ImVec2(iconX - 1.0f, iconY - 1.0f),
                                          ImVec2(iconX + iconSz + 1.0f, iconY + iconSz + 1.0f),
                                          IM_COL32(0, 0, 0, A(180)), 1.0f);
                    }
                    if (!spellName.empty()) {
                        ImVec2 snSz = ImGui::CalcTextSize(spellName.c_str());
                        float snX = sx - snSz.x * 0.5f;
                        float snY = castBarBaseY;
                        drawList->AddText(ImVec2(snX + 1.0f, snY + 1.0f), IM_COL32(0, 0, 0, A(140)), spellName.c_str());
                        drawList->AddText(ImVec2(snX,         snY),         IM_COL32(255, 210, 100, A(220)), spellName.c_str());
                        castBarBaseY += snSz.y + 2.0f;
                    }
                }

                // Cast bar: green = interruptible, red = uninterruptible; both pulse when >80% complete
                ImU32 cbBg = IM_COL32(30, 25, 40, A(180));
                ImU32 cbFill;
                if (castPct > 0.8f && unit->isHostile()) {
                    float pulse = 0.7f + 0.3f * std::sin(static_cast<float>(ImGui::GetTime()) * 8.0f);
                    cbFill = cs->interruptible
                        ? IM_COL32(static_cast<int>(40  * pulse), static_cast<int>(220 * pulse), static_cast<int>(40  * pulse), A(220))  // green pulse
                        : IM_COL32(static_cast<int>(255 * pulse), static_cast<int>(30  * pulse), static_cast<int>(30  * pulse), A(220)); // red pulse
                } else {
                    cbFill = cs->interruptible
                        ? IM_COL32(50,  190, 50,  A(200))   // green = interruptible
                        : IM_COL32(190, 40,  40,  A(200));  // red = uninterruptible
                }
                drawList->AddRectFilled(ImVec2(barX,                   castBarBaseY),
                                        ImVec2(barX + barW,             castBarBaseY + cbH), cbBg,    2.0f);
                drawList->AddRectFilled(ImVec2(barX,                   castBarBaseY),
                                        ImVec2(barX + barW * castPct,   castBarBaseY + cbH), cbFill,  2.0f);
                drawList->AddRect      (ImVec2(barX - 1.0f, castBarBaseY - 1.0f),
                                        ImVec2(barX + barW + 1.0f, castBarBaseY + cbH + 1.0f),
                                        IM_COL32(20, 10, 40, A(200)), 2.0f);

                // Time remaining text
                char timeBuf[12];
                snprintf(timeBuf, sizeof(timeBuf), "%.1fs", cs->timeRemaining);
                ImVec2 timeSz = ImGui::CalcTextSize(timeBuf);
                float timeX = sx - timeSz.x * 0.5f;
                float timeY = castBarBaseY + (cbH - timeSz.y) * 0.5f;
                drawList->AddText(ImVec2(timeX + 1.0f, timeY + 1.0f), IM_COL32(0, 0, 0, A(140)), timeBuf);
                drawList->AddText(ImVec2(timeX,         timeY),         IM_COL32(220, 200, 255, A(220)), timeBuf);
                nameplateBottom = castBarBaseY + cbH + 2.0f;
            }
        }

        // Debuff dot indicators: small colored squares below the nameplate showing
        // player-applied auras on the current hostile target.
        // Colors: Magic=blue, Curse=purple, Disease=yellow, Poison=green, Other=grey
        if (isTarget && unit->isHostile() && !isCorpse) {
            const auto& auras = gameHandler.getTargetAuras();
            const uint64_t pguid = gameHandler.getPlayerGuid();
            const float dotSize = 6.0f * settingsPanel_.nameplateScale_;
            const float dotGap  = 2.0f;
            float dotX = barX;
            for (const auto& aura : auras) {
                if (aura.isEmpty() || aura.casterGuid != pguid) continue;
                uint8_t dispelType = gameHandler.getSpellDispelType(aura.spellId);
                ImU32 dotCol;
                switch (dispelType) {
                    case 1:  dotCol = IM_COL32( 64, 128, 255, A(210)); break; // Magic   - blue
                    case 2:  dotCol = IM_COL32(160,  32, 240, A(210)); break; // Curse   - purple
                    case 3:  dotCol = IM_COL32(180, 140,  40, A(210)); break; // Disease - yellow-brown
                    case 4:  dotCol = IM_COL32( 50, 200,  50, A(210)); break; // Poison  - green
                    default: dotCol = IM_COL32(170, 170, 170, A(170)); break; // Other   - grey
                }
                drawList->AddRectFilled(ImVec2(dotX,          nameplateBottom),
                                        ImVec2(dotX + dotSize, nameplateBottom + dotSize), dotCol, 1.0f);
                drawList->AddRect      (ImVec2(dotX - 1.0f,          nameplateBottom - 1.0f),
                                        ImVec2(dotX + dotSize + 1.0f, nameplateBottom + dotSize + 1.0f),
                                        IM_COL32(0, 0, 0, A(150)), 1.0f);

                // Duration clock-sweep overlay (like target frame auras)
                uint64_t nowMs = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count());
                int32_t remainMs = aura.getRemainingMs(nowMs);
                if (aura.maxDurationMs > 0 && remainMs > 0) {
                    float pct = 1.0f - static_cast<float>(remainMs) / static_cast<float>(aura.maxDurationMs);
                    pct = std::clamp(pct, 0.0f, 1.0f);
                    float cx = dotX + dotSize * 0.5f;
                    float cy = nameplateBottom + dotSize * 0.5f;
                    float r  = dotSize * 0.5f;
                    float startAngle = -IM_PI * 0.5f;
                    float endAngle   = startAngle + pct * IM_PI * 2.0f;
                    ImVec2 center(cx, cy);
                    const int segments = 12;
                    for (int seg = 0; seg < segments; seg++) {
                        float a0 = startAngle + (endAngle - startAngle) * seg / segments;
                        float a1 = startAngle + (endAngle - startAngle) * (seg + 1) / segments;
                        drawList->AddTriangleFilled(
                            center,
                            ImVec2(cx + r * std::cos(a0), cy + r * std::sin(a0)),
                            ImVec2(cx + r * std::cos(a1), cy + r * std::sin(a1)),
                            IM_COL32(0, 0, 0, A(100)));
                    }
                }

                // Stack count on dot (upper-left corner)
                if (aura.charges > 1) {
                    char stackBuf[8];
                    snprintf(stackBuf, sizeof(stackBuf), "%d", aura.charges);
                    drawList->AddText(ImVec2(dotX + 1.0f, nameplateBottom), IM_COL32(0, 0, 0, A(200)), stackBuf);
                    drawList->AddText(ImVec2(dotX,         nameplateBottom - 1.0f), IM_COL32(255, 255, 255, A(240)), stackBuf);
                }

                // Duration text below dot
                if (remainMs > 0) {
                    char durBuf[8];
                    if (remainMs >= 60000)
                        snprintf(durBuf, sizeof(durBuf), "%dm", remainMs / 60000);
                    else
                        snprintf(durBuf, sizeof(durBuf), "%d", remainMs / 1000);
                    ImVec2 durSz = ImGui::CalcTextSize(durBuf);
                    float durX = dotX + (dotSize - durSz.x) * 0.5f;
                    float durY = nameplateBottom + dotSize + 1.0f;
                    drawList->AddText(ImVec2(durX + 1.0f, durY + 1.0f), IM_COL32(0, 0, 0, A(180)), durBuf);
                    // Color: red if < 5s, yellow if < 15s, white otherwise
                    ImU32 durCol = remainMs < 5000 ? IM_COL32(255, 60, 60, A(240))
                                 : remainMs < 15000 ? IM_COL32(255, 200, 60, A(240))
                                 : IM_COL32(230, 230, 230, A(220));
                    drawList->AddText(ImVec2(durX, durY), durCol, durBuf);
                }

                // Spell name + duration tooltip on hover
                {
                    ImVec2 mouse = ImGui::GetMousePos();
                    if (mouse.x >= dotX && mouse.x < dotX + dotSize &&
                        mouse.y >= nameplateBottom && mouse.y < nameplateBottom + dotSize) {
                        const std::string& dotSpellName = gameHandler.getSpellName(aura.spellId);
                        if (!dotSpellName.empty()) {
                            if (remainMs > 0) {
                                int secs = remainMs / 1000;
                                int mins = secs / 60;
                                secs %= 60;
                                char tipBuf[128];
                                if (mins > 0)
                                    snprintf(tipBuf, sizeof(tipBuf), "%s (%dm %ds)", dotSpellName.c_str(), mins, secs);
                                else
                                    snprintf(tipBuf, sizeof(tipBuf), "%s (%ds)", dotSpellName.c_str(), secs);
                                ImGui::SetTooltip("%s", tipBuf);
                            } else {
                                ImGui::SetTooltip("%s", dotSpellName.c_str());
                            }
                        }
                    }
                }

                dotX += dotSize + dotGap;
                if (dotX + dotSize > barX + barW) break;
            }
        }

        // Name + level label above health bar
        uint32_t level = unit->getLevel();
        const std::string& unitName = unit->getName();
        char labelBuf[96];
        if (isPlayer) {
            // Player nameplates: show name only (no level clutter).
            // Fall back to level as placeholder while the name query is pending.
            if (!unitName.empty())
                snprintf(labelBuf, sizeof(labelBuf), "%s", unitName.c_str());
            else {
                // Name query may be pending; request it now to ensure it gets resolved
                gameHandler.queryPlayerName(unit->getGuid());
                if (level > 0)
                    snprintf(labelBuf, sizeof(labelBuf), "Player (%u)", level);
                else
                    snprintf(labelBuf, sizeof(labelBuf), "Player");
            }
        } else if (level > 0) {
            uint32_t playerLevel = gameHandler.getPlayerLevel();
            // Show skull for units more than 10 levels above the player
            if (playerLevel > 0 && level > playerLevel + 10)
                snprintf(labelBuf, sizeof(labelBuf), "?? %s", unitName.c_str());
            else
                snprintf(labelBuf, sizeof(labelBuf), "%u %s", level, unitName.c_str());
        } else {
            snprintf(labelBuf, sizeof(labelBuf), "%s", unitName.c_str());
        }
        ImVec2 textSize = ImGui::CalcTextSize(labelBuf);
        float nameX = sx - textSize.x * 0.5f;
        float nameY = sy - barH - 12.0f;
        // Name color communicates reaction too: friendly players blue, enemies red.
        ImU32 nameColor;
        if (isPlayer) {
            nameColor = isHostile
                ? IM_COL32(220, 80, 80, A(230))
                : IM_COL32(102, 153, 255, A(230));
        } else {
            nameColor = isHostile
                ? IM_COL32(220,  80,  80, A(230))   // red  — hostile NPC
                : IM_COL32(240, 200, 100, A(230));  // yellow — friendly NPC
        }
        // Sub-label below the name: guild tag for players, subtitle for NPCs
        std::string subLabel;
        if (isPlayer) {
            uint32_t guildId = gameHandler.getEntityGuildId(guid);
            if (guildId != 0) {
                const std::string& gn = gameHandler.lookupGuildName(guildId);
                if (!gn.empty()) subLabel = "<" + gn + ">";
            }
        } else {
            // NPC subtitle (e.g. "<Reagent Vendor>", "<Innkeeper>")
            std::string sub = gameHandler.getCachedCreatureSubName(unit->getEntry());
            if (!sub.empty()) subLabel = "<" + sub + ">";
        }
        if (!subLabel.empty()) nameY -= 10.0f;  // shift name up for sub-label line

        drawList->AddText(ImVec2(nameX + 1.0f, nameY + 1.0f), IM_COL32(0, 0, 0, A(160)), labelBuf);
        drawList->AddText(ImVec2(nameX,         nameY),         nameColor, labelBuf);

        // Gold chevron above the current target's plate — a gently bobbing
        // down-arrow so the selected enemy is unmistakable at a glance.
        if (isTarget) {
            float bob = 2.0f * std::sin(static_cast<float>(ImGui::GetTime()) * 5.0f);
            float tipY  = nameY - 5.0f + bob;
            float baseY = tipY - 8.0f;
            float halfW = 6.0f;
            drawList->AddTriangleFilled(
                ImVec2(sx - halfW + 1.0f, baseY + 1.0f), ImVec2(sx + halfW + 1.0f, baseY + 1.0f),
                ImVec2(sx + 1.0f, tipY + 1.0f), IM_COL32(0, 0, 0, A(140)));
            drawList->AddTriangleFilled(
                ImVec2(sx - halfW, baseY), ImVec2(sx + halfW, baseY),
                ImVec2(sx, tipY), IM_COL32(255, 215, 0, A(240)));
        }

        // Sub-label below the name (WoW-style <Guild Name> or <NPC Title> in lighter color)
        if (!subLabel.empty()) {
            ImVec2 subSz = ImGui::CalcTextSize(subLabel.c_str());
            float subX = sx - subSz.x * 0.5f;
            float subY = nameY + textSize.y + 1.0f;
            drawList->AddText(ImVec2(subX + 1.0f, subY + 1.0f), IM_COL32(0, 0, 0, A(120)), subLabel.c_str());
            drawList->AddText(ImVec2(subX,         subY),         IM_COL32(180, 180, 180, A(200)), subLabel.c_str());
        }

        // Group leader crown to the right of the name on player nameplates
        if (isPlayer && gameHandler.isInGroup() &&
            gameHandler.getPartyData().leaderGuid == guid) {
            float crownX = nameX + textSize.x + 3.0f;
            const char* crownSym = "\xe2\x99\x9b";  // ♛
            drawList->AddText(ImVec2(crownX + 1.0f, nameY + 1.0f), IM_COL32(0, 0, 0, A(160)), crownSym);
            drawList->AddText(ImVec2(crownX,         nameY),         IM_COL32(255, 215, 0, A(240)), crownSym);
        }

        // Raid mark: the real Blizzard icon art floating above the unit, the way
        // the original client presents it. This used to be a text glyph beside
        // the name, but the ImGui font carries no skull/moon/cross code points,
        // so every mark drew as a '?' box.
        {
            uint8_t raidMark = gameHandler.getEntityRaidMark(guid);
            if (raidMark < game::GameHandler::kRaidMarkCount) {
                VkDescriptorSet markTex = ui::getRaidTargetIcon(raidMark, services_.assetManager);
                if (markTex) {
                    // Sits above the name, and above the target chevron when this
                    // is the current target, so the three never overlap.
                    constexpr float kMarkSize = 32.0f;
                    const float markBottom = nameY - (isTarget ? 16.0f : 3.0f);
                    const float markTop    = markBottom - kMarkSize;
                    drawList->AddImage(
                        (ImTextureID)(uintptr_t)markTex,
                        ImVec2(sx - kMarkSize * 0.5f, markTop),
                        ImVec2(sx + kMarkSize * 0.5f, markBottom),
                        ImVec2(0.0f, 0.0f), ImVec2(1.0f, 1.0f),
                        IM_COL32(255, 255, 255, A(255)));
                }
            }

            // Quest kill objective indicator: small yellow sword icon to the right of the name
            float questIconX = nameX + textSize.x + 4.0f;
            if (!isPlayer && questKillEntries.count(unit->getEntry())) {
                const char* objSym = "\xe2\x9a\x94";  // ⚔ crossed swords (UTF-8)
                drawList->AddText(ImVec2(questIconX + 1.0f, nameY + 1.0f), IM_COL32(0, 0, 0, A(160)), objSym);
                drawList->AddText(ImVec2(questIconX,         nameY),         IM_COL32(255, 220, 0, A(230)), objSym);
                questIconX += ImGui::CalcTextSize("\xe2\x9a\x94").x + 2.0f;
            }

            // Quest giver indicator: "!" for available quests, "?" for completable/incomplete
            if (!isPlayer) {
                using QGS = game::QuestGiverStatus;
                QGS qgs = gameHandler.getQuestGiverStatus(guid);
                const char* qSym = nullptr;
                ImU32 qCol = IM_COL32(255, 210, 0, A(255));
                if (qgs == QGS::AVAILABLE) {
                    qSym = "!";
                } else if (qgs == QGS::AVAILABLE_LOW) {
                    qSym = "!";
                    qCol = IM_COL32(160, 160, 160, A(220));
                } else if (qgs == QGS::REWARD || qgs == QGS::REWARD_REP) {
                    qSym = "?";
                } else if (qgs == QGS::INCOMPLETE) {
                    qSym = "?";
                    qCol = IM_COL32(160, 160, 160, A(220));
                }
                if (qSym) {
                    drawList->AddText(ImVec2(questIconX + 1.0f, nameY + 1.0f), IM_COL32(0, 0, 0, A(160)), qSym);
                    drawList->AddText(ImVec2(questIconX,         nameY),         qCol, qSym);
                }
            }
        }

        // Click to target / right-click context: detect clicks inside the nameplate region.
        // Use the wider of name text or health bar for the horizontal hit area so short
        // names like "Wolf" don't produce a tiny clickable strip narrower than the bar.
        if (!ImGui::GetIO().WantCaptureMouse) {
            ImVec2 mouse = ImGui::GetIO().MousePos;
            float hitLeft  = std::min(nameX, barX) - 2.0f;
            float hitRight = std::max(nameX + textSize.x, barX + barW) + 2.0f;
            float ny0 = nameY - 1.0f;
            float ny1 = sy + barH + 2.0f;
            float nx0 = hitLeft;
            float nx1 = hitRight;
            if (mouse.x >= nx0 && mouse.x <= nx1 && mouse.y >= ny0 && mouse.y <= ny1) {
                // Track mouseover for [target=mouseover] macro conditionals
                gameHandler.setMouseoverGuid(guid);
                // Hover tooltip: name, level/class, guild
                ImGui::BeginTooltip();
                ImGui::TextUnformatted(unitName.c_str());
                if (isPlayer) {
                    uint8_t cid = entityClassId(unit);
                    ImGui::Text("Level %u %s", level, classNameStr(cid));
                } else if (level > 0) {
                    ImGui::Text("Level %u", level);
                }
                if (!subLabel.empty()) ImGui::TextColored(ImVec4(0.7f,0.7f,0.7f,1.0f), "%s", subLabel.c_str());
                ImGui::EndTooltip();
                if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                    gameHandler.setTarget(guid);
                } else if (ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
                    nameplateCtxGuid_ = guid;
                    nameplateCtxPos_  = mouse;
                    ImGui::OpenPopup("##NameplateCtx");
                }
            }
        }
    }

    // Render nameplate context popup (uses a tiny overlay window as host)
    if (nameplateCtxGuid_ != 0) {
        ImGui::SetNextWindowPos(nameplateCtxPos_, ImGuiCond_Appearing);
        ImGui::SetNextWindowSize(ImVec2(0, 0), ImGuiCond_Always);
        ImGuiWindowFlags ctxHostFlags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                                         ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
                                         ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoFocusOnAppearing |
                                         ImGuiWindowFlags_AlwaysAutoResize;
        if (ImGui::Begin("##NameplateCtxHost", nullptr, ctxHostFlags)) {
            if (ImGui::BeginPopup("##NameplateCtx")) {
                auto entityPtr = gameHandler.getEntityManager().getEntity(nameplateCtxGuid_);
                std::string ctxName = entityPtr ? getEntityName(entityPtr) : "";
                if (!ctxName.empty()) {
                    ImGui::TextDisabled("%s", ctxName.c_str());
                    ImGui::Separator();
                }
                if (ImGui::MenuItem("Target"))
                    gameHandler.setTarget(nameplateCtxGuid_);
                if (ImGui::MenuItem("Set Focus"))
                    gameHandler.setFocus(nameplateCtxGuid_);
                bool isPlayer = entityPtr && entityPtr->getType() == game::ObjectType::PLAYER;
                if (isPlayer && !ctxName.empty()) {
                    ImGui::Separator();
                    if (ImGui::MenuItem("Whisper")) {
                        chatPanel_.setWhisperTarget(ctxName);
                    }
                    if (ImGui::MenuItem("Invite to Group"))
                        gameHandler.inviteToGroup(ctxName);
                    if (ImGui::MenuItem("Trade"))
                        gameHandler.initiateTrade(nameplateCtxGuid_);
                    if (ImGui::MenuItem("Duel"))
                        gameHandler.proposeDuel(nameplateCtxGuid_);
                    if (ImGui::MenuItem("Inspect")) {
                        gameHandler.setTarget(nameplateCtxGuid_);
                        gameHandler.inspectTarget();
                        socialPanel_.showInspectWindow_ = true;
                    }
                    ImGui::Separator();
                    if (ImGui::MenuItem("Add Friend"))
                        gameHandler.addFriend(ctxName);
                    if (ImGui::MenuItem("Ignore"))
                        gameHandler.addIgnore(ctxName);
                }
                ImGui::EndPopup();
            } else {
                nameplateCtxGuid_ = 0;
            }
        }
        ImGui::End();
    }
}

// ============================================================
// Durability Warning (equipment damage indicator)
// ============================================================

void GameScreen::takeScreenshot(game::GameHandler& /*gameHandler*/) {
    auto* renderer = services_.renderer;
    if (!renderer) return;

    // Build path: ~/.wowee/screenshots/WoWee_YYYYMMDD_HHMMSS.png
    const char* home = std::getenv("HOME");
    if (!home) home = std::getenv("USERPROFILE");
    if (!home) home = "/tmp";
    std::string dir = std::string(home) + "/.wowee/screenshots";

    auto now = std::chrono::system_clock::now();
    auto tt  = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &tt);
#else
    localtime_r(&tt, &tm);
#endif

    char filename[128];
    std::snprintf(filename, sizeof(filename),
                  "WoWee_%04d%02d%02d_%02d%02d%02d.png",
                  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                  tm.tm_hour, tm.tm_min, tm.tm_sec);

    std::string path = dir + "/" + filename;

    if (renderer->captureScreenshot(path)) {
        game::MessageChatData sysMsg;
        sysMsg.type = game::ChatType::SYSTEM;
        sysMsg.language = game::ChatLanguage::UNIVERSAL;
        sysMsg.message = "Screenshot saved: " + path;
        services_.gameHandler->addLocalChatMessage(sysMsg);
    }
}

void GameScreen::renderDurabilityWarning(game::GameHandler& gameHandler) {
    if (gameHandler.getPlayerGuid() == 0) return;

    const auto& inv = gameHandler.getInventory();

    // Scan all equipment slots (skip bag slots which have no durability)
    float minDurPct = 1.0f;
    bool hasBroken = false;

    for (int i = static_cast<int>(game::EquipSlot::HEAD);
             i < static_cast<int>(game::EquipSlot::BAG1); ++i) {
        const auto& slot = inv.getEquipSlot(static_cast<game::EquipSlot>(i));
        if (slot.empty() || slot.item.maxDurability == 0) continue;
        if (slot.item.curDurability == 0) {
            hasBroken = true;
        }
        float pct = static_cast<float>(slot.item.curDurability) /
                    static_cast<float>(slot.item.maxDurability);
        if (pct < minDurPct) minDurPct = pct;
    }

    // Only show warning below 20%
    if (minDurPct >= 0.2f && !hasBroken) return;

    ImGuiIO& io = ImGui::GetIO();
    const float screenW = io.DisplaySize.x;
    const float screenH = io.DisplaySize.y;

    // Position: just above the XP bar / action bar area (bottom-center)
    const float warningW = 220.0f;
    const float warningH = 26.0f;
    const float posX = (screenW - warningW) * 0.5f;
    const float posY = screenH - 140.0f;  // above action bar

    ImGui::SetNextWindowPos(ImVec2(posX, posY), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(warningW, warningH), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.75f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(6, 4));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowMinSize, ImVec2(0, 0));

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                             ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoInputs |
                             ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings |
                             ImGuiWindowFlags_NoBringToFrontOnFocus;

    if (ImGui::Begin("##durability_warn", nullptr, flags)) {
        if (hasBroken) {
            ImGui::TextColored(ImVec4(1.0f, 0.15f, 0.15f, 1.0f),
                               "\xef\x94\x9b Gear broken! Visit a repair NPC");
        } else {
            int pctInt = static_cast<int>(minDurPct * 100.0f);
            ImGui::TextColored(colors::kSymbolGold,
                               "\xef\x94\x9b Low durability: %d%%", pctInt);
        }
        if (ImGui::IsWindowHovered())
            ImGui::SetTooltip("Your equipment is damaged. Visit any blacksmith or repair NPC.");
    }
    ImGui::End();
    ImGui::PopStyleVar(3);
}

// ============================================================
// UI Error Frame (WoW-style center-bottom error overlay)
// ============================================================

void GameScreen::renderUIErrors(game::GameHandler& /*gameHandler*/, float deltaTime) {
    // Age out old entries
    for (auto& e : uiErrors_) e.age += deltaTime;
    uiErrors_.erase(
        std::remove_if(uiErrors_.begin(), uiErrors_.end(),
            [](const UIErrorEntry& e) { return e.age >= kUIErrorLifetime; }),
        uiErrors_.end());

    if (uiErrors_.empty()) return;

    auto* window = services_.window;
    float screenW = window ? static_cast<float>(window->getWidth()) : 1280.0f;
    float screenH = window ? static_cast<float>(window->getHeight()) :  720.0f;

    // Fixed invisible overlay
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(screenW, screenH));
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoDecoration |
                             ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoNav |
                             ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    if (ImGui::Begin("##UIErrors", nullptr, flags)) {
        // Render messages stacked above the action bar (~200px from bottom)
        // The newest message is on top; older ones fade below it.
        const float baseY = screenH - 200.0f;
        const float lineH = 20.0f;
        const int   count = static_cast<int>(uiErrors_.size());

        ImDrawList* draw = ImGui::GetWindowDrawList();
        for (int i = count - 1; i >= 0; --i) {
            const auto& e = uiErrors_[i];
            float alpha = 1.0f - (e.age / kUIErrorLifetime);
            alpha = std::max(0.0f, std::min(1.0f, alpha));

            // Fade fast in the last 0.5 s
            if (e.age > kUIErrorLifetime - 0.5f)
                alpha *= (kUIErrorLifetime - e.age) / 0.5f;

            uint8_t a8 = static_cast<uint8_t>(alpha * 255.0f);
            ImU32 textCol  = IM_COL32(255, 50,  50, a8);
            ImU32 shadowCol= IM_COL32(  0,  0,   0, static_cast<uint8_t>(alpha * 180));

            const char* txt = e.text.c_str();
            ImVec2 sz = ImGui::CalcTextSize(txt);
            float x = std::round((screenW - sz.x) * 0.5f);
            float y = std::round(baseY - (count - 1 - i) * lineH);

            // Drop shadow
            draw->AddText(ImVec2(x + 1, y + 1), shadowCol, txt);
            draw->AddText(ImVec2(x, y), textCol, txt);
        }
    }
    ImGui::End();
    ImGui::PopStyleVar();
}

void GameScreen::renderQuestMarkers(game::GameHandler& gameHandler) {
    const auto& statuses = gameHandler.getNpcQuestStatuses();
    if (statuses.empty()) return;

    auto* renderer = services_.renderer;
    auto* camera = renderer ? renderer->getCamera() : nullptr;
    auto* window = services_.window;
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
        } else if (status == game::QuestGiverStatus::REWARD ||
                   status == game::QuestGiverStatus::REWARD_REP) {
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


}} // namespace wowee::ui
