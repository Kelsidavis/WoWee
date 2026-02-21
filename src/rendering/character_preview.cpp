#include "rendering/character_preview.hpp"
#include "rendering/character_renderer.hpp"
#include "rendering/camera.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/m2_loader.hpp"
#include "pipeline/dbc_loader.hpp"
#include "pipeline/dbc_layout.hpp"
#include "core/logger.hpp"
#include <GL/glew.h>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <unordered_set>

namespace wowee {
namespace rendering {

CharacterPreview::CharacterPreview() = default;

CharacterPreview::~CharacterPreview() {
    shutdown();
}

bool CharacterPreview::initialize(pipeline::AssetManager* am) {
    assetManager_ = am;

    charRenderer_ = std::make_unique<CharacterRenderer>();
    if (!charRenderer_->initialize()) {
        LOG_ERROR("CharacterPreview: failed to initialize CharacterRenderer");
        return false;
    }
    charRenderer_->setAssetManager(am);

    // Disable fog and shadows for the preview
    charRenderer_->setFog(glm::vec3(0.05f, 0.05f, 0.1f), 9999.0f, 10000.0f);
    charRenderer_->clearShadowMap();

    camera_ = std::make_unique<Camera>();
    // Portrait-style camera: WoW Z-up coordinate system
    // Model at origin, camera positioned along +Y looking toward -Y
    camera_->setFov(30.0f);
    camera_->setAspectRatio(static_cast<float>(fboWidth_) / static_cast<float>(fboHeight_));
    // Pull camera back far enough to see full body + head with margin
    // Human ~2 units tall, Tauren ~2.5. At distance 4.5 with FOV 30:
    // vertical visible = 2 * 4.5 * tan(15°) ≈ 2.41 units
    camera_->setPosition(glm::vec3(0.0f, 4.5f, 0.9f));
    camera_->setRotation(270.0f, 0.0f);

    createFBO();

    LOG_INFO("CharacterPreview initialized (", fboWidth_, "x", fboHeight_, ")");
    return true;
}

void CharacterPreview::shutdown() {
    destroyFBO();
    if (charRenderer_) {
        charRenderer_->shutdown();
        charRenderer_.reset();
    }
    camera_.reset();
    modelLoaded_ = false;
    instanceId_ = 0;
}

void CharacterPreview::createFBO() {
    // Create color texture
    glGenTextures(1, &colorTexture_);
    glBindTexture(GL_TEXTURE_2D, colorTexture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, fboWidth_, fboHeight_, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);

    // Create depth renderbuffer
    glGenRenderbuffers(1, &depthRenderbuffer_);
    glBindRenderbuffer(GL_RENDERBUFFER, depthRenderbuffer_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, fboWidth_, fboHeight_);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);

    // Create FBO
    glGenFramebuffers(1, &fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, colorTexture_, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthRenderbuffer_);

    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        LOG_ERROR("CharacterPreview: FBO incomplete, status=", status);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void CharacterPreview::destroyFBO() {
    if (fbo_) { glDeleteFramebuffers(1, &fbo_); fbo_ = 0; }
    if (colorTexture_) { glDeleteTextures(1, &colorTexture_); colorTexture_ = 0; }
    if (depthRenderbuffer_) { glDeleteRenderbuffers(1, &depthRenderbuffer_); depthRenderbuffer_ = 0; }
}

bool CharacterPreview::loadCharacter(game::Race race, game::Gender gender,
                                      uint8_t skin, uint8_t face,
                                      uint8_t hairStyle, uint8_t hairColor,
                                      uint8_t facialHair, bool useFemaleModel) {
    if (!charRenderer_ || !assetManager_ || !assetManager_->isInitialized()) {
        return false;
    }

    // Remove existing instance
    if (instanceId_ > 0) {
        charRenderer_->removeInstance(instanceId_);
        instanceId_ = 0;
        modelLoaded_ = false;
    }

    std::string m2Path = game::getPlayerModelPath(race, gender, useFemaleModel);
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
        if (dot != std::string::npos) {
            baseName = baseName.substr(0, dot);
        }
    }

    auto m2Data = assetManager_->readFile(m2Path);
    if (m2Data.empty()) {
        LOG_WARNING("CharacterPreview: failed to read M2: ", m2Path);
        return false;
    }

    auto model = pipeline::M2Loader::load(m2Data);

    // Load skin file (only for WotLK M2s - vanilla has embedded skin)
    std::string skinPath = modelDir + baseName + "00.skin";
    auto skinData = assetManager_->readFile(skinPath);
    if (!skinData.empty() && model.version >= 264) {
        pipeline::M2Loader::loadSkin(skinData, model);
    }

    if (!model.isValid()) {
        LOG_WARNING("CharacterPreview: invalid model: ", m2Path);
        return false;
    }

    // Look up CharSections.dbc for all appearance textures
    uint32_t targetRaceId = static_cast<uint32_t>(race);
    uint32_t targetSexId = (gender == game::Gender::FEMALE) ? 1u : 0u;

    std::string faceLowerPath;
    std::string faceUpperPath;
    std::string hairScalpPath;
    std::vector<std::string> underwearPaths;
    bodySkinPath_.clear();
    baseLayers_.clear();

    auto charSectionsDbc = assetManager_->loadDBC("CharSections.dbc");
    if (charSectionsDbc) {
        bool foundSkin = false;
        bool foundFace = false;
        bool foundHair = false;
        bool foundUnderwear = false;

        const auto* csL = pipeline::getActiveDBCLayout() ? pipeline::getActiveDBCLayout()->getLayout("CharSections") : nullptr;

        uint32_t fRace = csL ? (*csL)["RaceID"] : 1;
        uint32_t fSex = csL ? (*csL)["SexID"] : 2;
        uint32_t fBase = csL ? (*csL)["BaseSection"] : 3;
        uint32_t fVar = csL ? (*csL)["VariationIndex"] : 4;
        uint32_t fColor = csL ? (*csL)["ColorIndex"] : 5;
        for (uint32_t r = 0; r < charSectionsDbc->getRecordCount(); r++) {
            uint32_t raceId = charSectionsDbc->getUInt32(r, fRace);
            uint32_t sexId = charSectionsDbc->getUInt32(r, fSex);
            uint32_t baseSection = charSectionsDbc->getUInt32(r, fBase);
            uint32_t variationIndex = charSectionsDbc->getUInt32(r, fVar);
            uint32_t colorIndex = charSectionsDbc->getUInt32(r, fColor);

            if (raceId != targetRaceId || sexId != targetSexId) continue;

            // Section 0: Body skin (variation=0, colorIndex = skin color)
            if (baseSection == 0 && !foundSkin &&
                variationIndex == 0 && colorIndex == static_cast<uint32_t>(skin)) {
                std::string tex1 = charSectionsDbc->getString(r, csL ? (*csL)["Texture1"] : 6);
                if (!tex1.empty()) {
                    bodySkinPath_ = tex1;
                    foundSkin = true;
                }
            }
            // Section 1: Face (variation = face index, colorIndex = skin color)
            else if (baseSection == 1 && !foundFace &&
                     variationIndex == static_cast<uint32_t>(face) &&
                     colorIndex == static_cast<uint32_t>(skin)) {
                std::string tex1 = charSectionsDbc->getString(r, csL ? (*csL)["Texture1"] : 6);
                std::string tex2 = charSectionsDbc->getString(r, csL ? (*csL)["Texture2"] : 7);
                if (!tex1.empty()) faceLowerPath = tex1;
                if (!tex2.empty()) faceUpperPath = tex2;
                foundFace = true;
            }
            // Section 3: Hair (variation = hair style, colorIndex = hair color)
            else if (baseSection == 3 && !foundHair &&
                     variationIndex == static_cast<uint32_t>(hairStyle) &&
                     colorIndex == static_cast<uint32_t>(hairColor)) {
                std::string tex1 = charSectionsDbc->getString(r, csL ? (*csL)["Texture1"] : 6);
                if (!tex1.empty()) {
                    hairScalpPath = tex1;
                    foundHair = true;
                }
            }
            // Section 4: Underwear (variation=0, colorIndex = skin color)
            else if (baseSection == 4 && !foundUnderwear &&
                     variationIndex == 0 && colorIndex == static_cast<uint32_t>(skin)) {
                uint32_t texBase = csL ? (*csL)["Texture1"] : 6;
                for (uint32_t f = texBase; f <= texBase + 2; f++) {
                    std::string tex = charSectionsDbc->getString(r, f);
                    if (!tex.empty()) {
                        underwearPaths.push_back(tex);
                    }
                }
                foundUnderwear = true;
            }
        }

        LOG_INFO("CharSections lookup: skin=", foundSkin ? bodySkinPath_ : "(not found)",
                 " face=", foundFace ? (faceLowerPath.empty() ? "(empty)" : faceLowerPath) : "(not found)",
                 " hair=", foundHair ? (hairScalpPath.empty() ? "(empty)" : hairScalpPath) : "(not found)",
                 " underwear=", foundUnderwear, " (", underwearPaths.size(), " textures)");
    } else {
        LOG_WARNING("CharSections.dbc not loaded — no character textures");
    }

    // Assign texture filenames on model before GPU upload
    for (size_t ti = 0; ti < model.textures.size(); ti++) {
        auto& tex = model.textures[ti];
        LOG_INFO("  Model texture[", ti, "]: type=", tex.type,
                 " filename='", tex.filename, "'");
        if (tex.type == 1 && tex.filename.empty() && !bodySkinPath_.empty()) {
            tex.filename = bodySkinPath_;
        } else if (tex.type == 6 && tex.filename.empty() && !hairScalpPath.empty()) {
            tex.filename = hairScalpPath;
        }
    }

    // Load external .anim files
    for (uint32_t si = 0; si < model.sequences.size(); si++) {
        if (!(model.sequences[si].flags & 0x20)) {
            char animFileName[256];
            snprintf(animFileName, sizeof(animFileName),
                "%s%s%04u-%02u.anim",
                modelDir.c_str(),
                baseName.c_str(),
                model.sequences[si].id,
                model.sequences[si].variationIndex);
            auto animFileData = assetManager_->readFileOptional(animFileName);
            if (!animFileData.empty()) {
                pipeline::M2Loader::loadAnimFile(m2Data, animFileData, si, model);
            }
        }
    }

    if (!charRenderer_->loadModel(model, PREVIEW_MODEL_ID)) {
        LOG_WARNING("CharacterPreview: failed to load model to GPU");
        return false;
    }
    // Composite body skin + face + underwear overlays
    if (!bodySkinPath_.empty()) {
        std::vector<std::string> layers;
        layers.push_back(bodySkinPath_);
        // Face lower texture composited onto body at the face region
        if (!faceLowerPath.empty()) {
            layers.push_back(faceLowerPath);
        }
        if (!faceUpperPath.empty()) {
            layers.push_back(faceUpperPath);
        }
        for (const auto& up : underwearPaths) {
            layers.push_back(up);
        }

        // Cache for later equipment compositing.
        // Keep baseLayers_ without the base skin (compositeWithRegions takes basePath separately).
        if (!faceLowerPath.empty()) baseLayers_.push_back(faceLowerPath);
        if (!faceUpperPath.empty()) baseLayers_.push_back(faceUpperPath);
        for (const auto& up : underwearPaths) baseLayers_.push_back(up);

        if (layers.size() > 1) {
            GLuint compositeTex = charRenderer_->compositeTextures(layers);
            if (compositeTex != 0) {
                for (size_t ti = 0; ti < model.textures.size(); ti++) {
                    if (model.textures[ti].type == 1) {
                        charRenderer_->setModelTexture(PREVIEW_MODEL_ID, static_cast<uint32_t>(ti), compositeTex);
                        break;
                    }
                }
            }
        }
    }

    // If hair scalp texture was found, ensure it's loaded for type-6 slot
    if (!hairScalpPath.empty()) {
        GLuint hairTex = charRenderer_->loadTexture(hairScalpPath);
        if (hairTex != 0) {
            for (size_t ti = 0; ti < model.textures.size(); ti++) {
                if (model.textures[ti].type == 6) {
                    charRenderer_->setModelTexture(PREVIEW_MODEL_ID, static_cast<uint32_t>(ti), hairTex);
                    break;
                }
            }
        }
    }

    // Create instance at origin with current yaw
    instanceId_ = charRenderer_->createInstance(PREVIEW_MODEL_ID,
        glm::vec3(0.0f, 0.0f, 0.0f),
        glm::vec3(0.0f, 0.0f, modelYaw_),
        1.0f);

    if (instanceId_ == 0) {
        LOG_WARNING("CharacterPreview: failed to create instance");
        return false;
    }

    // Set default geosets (naked character)
    std::unordered_set<uint16_t> activeGeosets;
    // Body parts (group 0: IDs 0-99, vanilla models use up to 27)
    for (uint16_t i = 0; i <= 99; i++) {
        activeGeosets.insert(i);
    }
    // Hair style geoset: group 1 = 100 + variation + 1
    activeGeosets.insert(static_cast<uint16_t>(100 + hairStyle + 1));
    // Facial hair geoset: group 2 = 200 + variation + 1
    activeGeosets.insert(static_cast<uint16_t>(200 + facialHair + 1));
    activeGeosets.insert(401);   // Bare forearms (no gloves) — group 4
    activeGeosets.insert(502);   // Bare shins (no boots) — group 5
    activeGeosets.insert(702);   // Ears: default
    activeGeosets.insert(801);   // Bare wrists (no sleeves) — group 8
    activeGeosets.insert(902);   // Kneepads: default — group 9
    activeGeosets.insert(1301);  // Bare legs (no pants) — group 13
    activeGeosets.insert(1502);  // No cloak — group 15
    activeGeosets.insert(2002);  // Bare feet mesh — group 20
    charRenderer_->setActiveGeosets(instanceId_, activeGeosets);

    // Play idle animation (Stand = animation ID 0)
    charRenderer_->playAnimation(instanceId_, 0, true);

    // Cache core appearance for later equipment geosets.
    race_ = race;
    gender_ = gender;
    useFemaleModel_ = useFemaleModel;
    hairStyle_ = hairStyle;
    facialHair_ = facialHair;

    // Cache the type-1 texture slot index so applyEquipment can update it.
    skinTextureSlotIndex_ = 0;
    for (size_t ti = 0; ti < model.textures.size(); ti++) {
        if (model.textures[ti].type == 1) {
            skinTextureSlotIndex_ = static_cast<uint32_t>(ti);
            break;
        }
    }

    modelLoaded_ = true;
    LOG_INFO("CharacterPreview: loaded ", m2Path,
             " skin=", (int)skin, " face=", (int)face,
             " hair=", (int)hairStyle, " hairColor=", (int)hairColor,
             " facial=", (int)facialHair);
    return true;
}

bool CharacterPreview::applyEquipment(const std::vector<game::EquipmentItem>& equipment) {
    if (!modelLoaded_ || instanceId_ == 0 || !charRenderer_ || !assetManager_ || !assetManager_->isInitialized()) {
        return false;
    }

    auto displayInfoDbc = assetManager_->loadDBC("ItemDisplayInfo.dbc");
    if (!displayInfoDbc || !displayInfoDbc->isLoaded()) {
        return false;
    }

    auto hasInvType = [&](std::initializer_list<uint8_t> types) -> bool {
        for (const auto& it : equipment) {
            if (it.displayModel == 0) continue;
            for (uint8_t t : types) {
                if (it.inventoryType == t) return true;
            }
        }
        return false;
    };

    auto findDisplayId = [&](std::initializer_list<uint8_t> types) -> uint32_t {
        for (const auto& it : equipment) {
            if (it.displayModel == 0) continue;
            for (uint8_t t : types) {
                if (it.inventoryType == t) return it.displayModel; // ItemDisplayInfo ID (3.3.5a char enum)
            }
        }
        return 0;
    };

    auto getGeosetGroup = [&](uint32_t displayInfoId, int groupField) -> uint32_t {
        if (displayInfoId == 0) return 0;
        int32_t recIdx = displayInfoDbc->findRecordById(displayInfoId);
        if (recIdx < 0) return 0;
        return displayInfoDbc->getUInt32(static_cast<uint32_t>(recIdx), 7 + groupField);
    };

    // --- Geosets ---
    std::unordered_set<uint16_t> geosets;
    for (uint16_t i = 0; i <= 99; i++) geosets.insert(i);
    geosets.insert(static_cast<uint16_t>(100 + hairStyle_ + 1));    // Hair style
    geosets.insert(static_cast<uint16_t>(200 + facialHair_ + 1));  // Facial hair
    geosets.insert(701);   // Ears
    geosets.insert(902);   // Kneepads: default (group 9)
    geosets.insert(2002);  // Bare feet mesh (group 20 = CG_FEET)

    // CharGeosets: group 4=gloves(forearm), 5=boots(shin), 8=sleeves, 13=pants
    uint16_t geosetGloves = 401;   // Bare forearms (group 4)
    uint16_t geosetBoots = 502;    // Bare shins (group 5)
    uint16_t geosetSleeves = 801;  // Bare wrists (group 8)
    uint16_t geosetPants = 1301;   // Bare legs (group 13)

    // Chest/Shirt/Robe → group 8 (sleeves)
    {
        uint32_t did = findDisplayId({4, 5, 20});
        uint32_t gg = getGeosetGroup(did, 0);
        if (gg > 0) geosetSleeves = static_cast<uint16_t>(801 + gg);
        // Robe kilt legs
        uint32_t gg3 = getGeosetGroup(did, 2);
        if (gg3 > 0) geosetPants = static_cast<uint16_t>(1301 + gg3);
    }
    // Legs → group 13 (trousers)
    {
        uint32_t did = findDisplayId({7});
        uint32_t gg = getGeosetGroup(did, 0);
        if (gg > 0) geosetPants = static_cast<uint16_t>(1301 + gg);
    }
    // Boots → group 5 (shins)
    {
        uint32_t did = findDisplayId({8});
        uint32_t gg = getGeosetGroup(did, 0);
        if (gg > 0) geosetBoots = static_cast<uint16_t>(501 + gg);
    }
    // Gloves → group 4 (forearms)
    {
        uint32_t did = findDisplayId({10});
        uint32_t gg = getGeosetGroup(did, 0);
        if (gg > 0) geosetGloves = static_cast<uint16_t>(401 + gg);
    }

    geosets.insert(geosetGloves);
    geosets.insert(geosetBoots);
    geosets.insert(geosetSleeves);
    geosets.insert(geosetPants);
    geosets.insert(hasInvType({16}) ? 1502 : 1501); // Cloak mesh toggle (visual may still be limited)
    if (hasInvType({19})) geosets.insert(1201);     // Tabard mesh toggle

    // Hide hair under helmets (helmets are separate models; this still avoids hair clipping)
    if (hasInvType({1})) {
        geosets.erase(static_cast<uint16_t>(100 + hairStyle_ + 1));
        geosets.insert(1);   // Bald scalp cap
        geosets.insert(101); // Default group-1 connector
    }

    charRenderer_->setActiveGeosets(instanceId_, geosets);

    // --- Textures (equipment overlays onto body skin) ---
    if (bodySkinPath_.empty()) return true; // geosets applied, but can't composite

    static const char* componentDirs[] = {
        "ArmUpperTexture", "ArmLowerTexture", "HandTexture",
        "TorsoUpperTexture", "TorsoLowerTexture",
        "LegUpperTexture", "LegLowerTexture", "FootTexture",
    };

    std::vector<std::pair<int, std::string>> regionLayers;
    regionLayers.reserve(32);

    for (const auto& it : equipment) {
        if (it.displayModel == 0) continue;
        int32_t recIdx = displayInfoDbc->findRecordById(it.displayModel);
        if (recIdx < 0) continue;

        for (int region = 0; region < 8; region++) {
            uint32_t fieldIdx = 14 + region; // texture_1..texture_8
            std::string texName = displayInfoDbc->getString(static_cast<uint32_t>(recIdx), fieldIdx);
            if (texName.empty()) continue;

            std::string base = "Item\\TextureComponents\\" +
                std::string(componentDirs[region]) + "\\" + texName;

            std::string genderSuffix = (gender_ == game::Gender::FEMALE) ? "_F.blp" : "_M.blp";
            std::string genderPath = base + genderSuffix;
            std::string unisexPath = base + "_U.blp";
            std::string fullPath;
            std::string basePath = base + ".blp";
            if (assetManager_->fileExists(genderPath)) {
                fullPath = genderPath;
            } else if (assetManager_->fileExists(unisexPath)) {
                fullPath = unisexPath;
            } else if (assetManager_->fileExists(basePath)) {
                fullPath = basePath;
            } else {
                continue;
            }
            regionLayers.emplace_back(region, fullPath);
        }
    }

    if (!regionLayers.empty()) {
        GLuint newTex = charRenderer_->compositeWithRegions(bodySkinPath_, baseLayers_, regionLayers);
        if (newTex != 0) {
            charRenderer_->setModelTexture(PREVIEW_MODEL_ID, skinTextureSlotIndex_, newTex);
        }
    }

    // Cloak texture (group 15) is separate from body compositing.
    if (hasInvType({16})) {
        uint32_t capeDisplayId = findDisplayId({16});
        if (capeDisplayId != 0) {
            int32_t capeRecIdx = displayInfoDbc->findRecordById(capeDisplayId);
            if (capeRecIdx >= 0) {
                std::vector<std::string> capeNames;
                auto addName = [&](const std::string& n) {
                    if (!n.empty() && std::find(capeNames.begin(), capeNames.end(), n) == capeNames.end()) {
                        capeNames.push_back(n);
                    }
                };
                std::string leftName = displayInfoDbc->getString(static_cast<uint32_t>(capeRecIdx), 3);
                std::string rightName = displayInfoDbc->getString(static_cast<uint32_t>(capeRecIdx), 4);
                if (gender_ == game::Gender::FEMALE) {
                    addName(rightName);
                    addName(leftName);
                } else {
                    addName(leftName);
                    addName(rightName);
                }

                auto hasBlpExt = [](const std::string& p) {
                    if (p.size() < 4) return false;
                    std::string ext = p.substr(p.size() - 4);
                    std::transform(ext.begin(), ext.end(), ext.begin(),
                                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                    return ext == ".blp";
                };
                std::vector<std::string> candidates;
                auto addCandidate = [&](const std::string& p) {
                    if (!p.empty() && std::find(candidates.begin(), candidates.end(), p) == candidates.end()) {
                        candidates.push_back(p);
                    }
                };
                for (const auto& nameRaw : capeNames) {
                    std::string name = nameRaw;
                    std::replace(name.begin(), name.end(), '/', '\\');
                    bool hasDir = (name.find('\\') != std::string::npos);
                    bool hasExt = hasBlpExt(name);
                    if (hasDir) {
                        addCandidate(name);
                        if (!hasExt) addCandidate(name + ".blp");
                    } else {
                        std::string baseObj = "Item\\ObjectComponents\\Cape\\" + name;
                        std::string baseTex = "Item\\TextureComponents\\Cape\\" + name;
                        addCandidate(baseObj);
                        addCandidate(baseTex);
                        if (!hasExt) {
                            addCandidate(baseObj + ".blp");
                            addCandidate(baseTex + ".blp");
                        }
                        addCandidate(baseObj + (gender_ == game::Gender::FEMALE ? "_F.blp" : "_M.blp"));
                        addCandidate(baseObj + "_U.blp");
                        addCandidate(baseTex + (gender_ == game::Gender::FEMALE ? "_F.blp" : "_M.blp"));
                        addCandidate(baseTex + "_U.blp");
                    }
                }
                const GLuint whiteTex = charRenderer_->loadTexture("");
                for (const auto& c : candidates) {
                    GLuint capeTex = charRenderer_->loadTexture(c);
                    if (capeTex != 0 && capeTex != whiteTex) {
                        charRenderer_->setGroupTextureOverride(instanceId_, 15, capeTex);
                        if (const auto* md = charRenderer_->getModelData(PREVIEW_MODEL_ID)) {
                            for (size_t ti = 0; ti < md->textures.size(); ti++) {
                                if (md->textures[ti].type == 2) {
                                    charRenderer_->setTextureSlotOverride(instanceId_, static_cast<uint16_t>(ti), capeTex);
                                }
                            }
                        }
                        break;
                    }
                }
            }
        }
    } else {
        if (const auto* md = charRenderer_->getModelData(PREVIEW_MODEL_ID)) {
            for (size_t ti = 0; ti < md->textures.size(); ti++) {
                if (md->textures[ti].type == 2) {
                    charRenderer_->clearTextureSlotOverride(instanceId_, static_cast<uint16_t>(ti));
                }
            }
        }
    }

    return true;
}

void CharacterPreview::update(float deltaTime) {
    if (charRenderer_ && modelLoaded_) {
        charRenderer_->update(deltaTime);
    }
}

void CharacterPreview::render() {
    if (!fbo_ || !charRenderer_ || !camera_ || !modelLoaded_) {
        return;
    }

    // Save current viewport
    GLint prevViewport[4];
    glGetIntegerv(GL_VIEWPORT, prevViewport);

    // Save current FBO binding
    GLint prevFbo;
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &prevFbo);

    // Bind our FBO
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    glViewport(0, 0, fboWidth_, fboHeight_);

    // Clear with dark blue background
    glClearColor(0.05f, 0.05f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    // Render the character model
    charRenderer_->render(*camera_, camera_->getViewMatrix(), camera_->getProjectionMatrix());

    // Restore previous FBO and viewport
    glBindFramebuffer(GL_FRAMEBUFFER, static_cast<GLuint>(prevFbo));
    glViewport(prevViewport[0], prevViewport[1], prevViewport[2], prevViewport[3]);
}

void CharacterPreview::rotate(float yawDelta) {
    modelYaw_ += yawDelta;
    if (instanceId_ > 0 && charRenderer_) {
        charRenderer_->setInstanceRotation(instanceId_, glm::vec3(0.0f, 0.0f, modelYaw_));
    }
}

} // namespace rendering
} // namespace wowee
