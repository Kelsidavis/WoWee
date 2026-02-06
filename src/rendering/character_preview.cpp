#include "rendering/character_preview.hpp"
#include "rendering/character_renderer.hpp"
#include "rendering/camera.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/m2_loader.hpp"
#include "pipeline/dbc_loader.hpp"
#include "core/logger.hpp"
#include <GL/glew.h>
#include <glm/gtc/matrix_transform.hpp>
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
                                      uint8_t facialHair) {
    if (!charRenderer_ || !assetManager_ || !assetManager_->isInitialized()) {
        return false;
    }

    // Remove existing instance
    if (instanceId_ > 0) {
        charRenderer_->removeInstance(instanceId_);
        instanceId_ = 0;
        modelLoaded_ = false;
    }

    std::string m2Path = game::getPlayerModelPath(race, gender);
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

    // Load skin file
    std::string skinPath = modelDir + baseName + "00.skin";
    auto skinData = assetManager_->readFile(skinPath);
    if (!skinData.empty()) {
        pipeline::M2Loader::loadSkin(skinData, model);
    }

    if (!model.isValid()) {
        LOG_WARNING("CharacterPreview: invalid model: ", m2Path);
        return false;
    }

    // Look up CharSections.dbc for all appearance textures
    uint32_t targetRaceId = static_cast<uint32_t>(race);
    uint32_t targetSexId = (gender == game::Gender::FEMALE) ? 1u : 0u;

    std::string bodySkinPath;
    std::string faceLowerPath;
    std::string faceUpperPath;
    std::string hairScalpPath;
    std::vector<std::string> underwearPaths;

    auto charSectionsDbc = assetManager_->loadDBC("CharSections.dbc");
    if (charSectionsDbc) {
        bool foundSkin = false;
        bool foundFace = false;
        bool foundHair = false;
        bool foundUnderwear = false;

        for (uint32_t r = 0; r < charSectionsDbc->getRecordCount(); r++) {
            uint32_t raceId = charSectionsDbc->getUInt32(r, 1);
            uint32_t sexId = charSectionsDbc->getUInt32(r, 2);
            uint32_t baseSection = charSectionsDbc->getUInt32(r, 3);
            uint32_t variationIndex = charSectionsDbc->getUInt32(r, 8);
            uint32_t colorIndex = charSectionsDbc->getUInt32(r, 9);

            if (raceId != targetRaceId || sexId != targetSexId) continue;

            // Section 0: Body skin (variation=0, colorIndex = skin color)
            if (baseSection == 0 && !foundSkin &&
                variationIndex == 0 && colorIndex == static_cast<uint32_t>(skin)) {
                std::string tex1 = charSectionsDbc->getString(r, 4);
                if (!tex1.empty()) {
                    bodySkinPath = tex1;
                    foundSkin = true;
                }
            }
            // Section 1: Face (variation = face index, colorIndex = skin color)
            else if (baseSection == 1 && !foundFace &&
                     variationIndex == static_cast<uint32_t>(face) &&
                     colorIndex == static_cast<uint32_t>(skin)) {
                std::string tex1 = charSectionsDbc->getString(r, 4);
                std::string tex2 = charSectionsDbc->getString(r, 5);
                if (!tex1.empty()) faceLowerPath = tex1;
                if (!tex2.empty()) faceUpperPath = tex2;
                foundFace = true;
            }
            // Section 3: Hair (variation = hair style, colorIndex = hair color)
            else if (baseSection == 3 && !foundHair &&
                     variationIndex == static_cast<uint32_t>(hairStyle) &&
                     colorIndex == static_cast<uint32_t>(hairColor)) {
                std::string tex1 = charSectionsDbc->getString(r, 4);
                if (!tex1.empty()) {
                    hairScalpPath = tex1;
                    foundHair = true;
                }
            }
            // Section 4: Underwear (variation=0, colorIndex = skin color)
            else if (baseSection == 4 && !foundUnderwear &&
                     variationIndex == 0 && colorIndex == static_cast<uint32_t>(skin)) {
                for (int f = 4; f <= 6; f++) {
                    std::string tex = charSectionsDbc->getString(r, f);
                    if (!tex.empty()) {
                        underwearPaths.push_back(tex);
                    }
                }
                foundUnderwear = true;
            }
        }
    }

    // Assign texture filenames on model before GPU upload
    for (auto& tex : model.textures) {
        if (tex.type == 1 && tex.filename.empty() && !bodySkinPath.empty()) {
            tex.filename = bodySkinPath;
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
            auto animFileData = assetManager_->readFile(animFileName);
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
    if (!bodySkinPath.empty()) {
        std::vector<std::string> layers;
        layers.push_back(bodySkinPath);
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
    // Body parts (group 0: IDs 0-18)
    for (uint16_t i = 0; i <= 18; i++) {
        activeGeosets.insert(i);
    }
    // Hair style geoset: group 1 = 100 + variation + 1
    activeGeosets.insert(static_cast<uint16_t>(100 + hairStyle + 1));
    // Facial hair geoset: group 2 = 200 + variation + 1
    activeGeosets.insert(static_cast<uint16_t>(200 + facialHair + 1));
    activeGeosets.insert(301);   // Gloves: bare hands
    activeGeosets.insert(401);   // Boots: bare feet
    activeGeosets.insert(501);   // Chest: bare
    activeGeosets.insert(701);   // Ears: default
    activeGeosets.insert(1301);  // Trousers: bare legs
    activeGeosets.insert(1501);  // Back body (cloak=none)
    charRenderer_->setActiveGeosets(instanceId_, activeGeosets);

    // Play idle animation (Stand = animation ID 0)
    charRenderer_->playAnimation(instanceId_, 0, true);

    modelLoaded_ = true;
    LOG_INFO("CharacterPreview: loaded ", m2Path,
             " skin=", (int)skin, " face=", (int)face,
             " hair=", (int)hairStyle, " hairColor=", (int)hairColor,
             " facial=", (int)facialHair);
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
