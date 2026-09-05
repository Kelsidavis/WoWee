#pragma once

#include "game/character.hpp"
#include <glm/glm.hpp>
#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>
#include <memory>
#include <cstdint>
#include <string>
#include <utility>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace wowee {
namespace pipeline { class AssetManager; struct M2Model; }
namespace rendering {

class CharacterRenderer;
class Camera;
class VkContext;
class VkTexture;
class VkRenderTarget;

class CharacterPreview {
public:
    CharacterPreview();
    ~CharacterPreview();

    /// Build the offscreen view. The size is the render target's, in pixels,
    /// and it is worth choosing: this is a full character pass every frame,
    /// and the paperdoll's 640x800 is enormously oversized for a portrait
    /// drawn into a circle fifty pixels across. Defaults to the paperdoll's,
    /// which is what every caller wanted before there was a choice.
    bool initialize(pipeline::AssetManager* am, int width = 640, int height = 800);
    void shutdown();

    bool loadCharacter(game::Race race, game::Gender gender,
                       uint8_t skin, uint8_t face,
                       uint8_t hairStyle, uint8_t hairColor,
                       uint8_t facialHair, bool useFemaleModel = false);

    /// Any model by path, with none of the appearance work - for a creature,
    /// whose M2 names its own textures and has no geosets to choose between.
    /// Put a pre-composited skin on the loaded character, replacing the one
    /// built from CharSections.
    ///
    /// CreatureDisplayInfoExtra carries one of these for nearly every humanoid
    /// NPC - 15,453 of 15,475 rows here - and it is the whole appearance
    /// already baked: skin, face, hair and the armour they are wearing. The
    /// client cannot composite an NPC's armour, and does not have to.
    ///
    /// After loadCharacter, because it overrides the slot that one fills.
    bool setBakedSkin(const std::string& bakePath);

    /// skins are (M2 texture type, path) pairs - a creature's model declares
    /// texture slots and the display row fills them, so the M2 alone is an
    /// untextured shape. EntitySpawner::getCreatureSkinPaths answers with
    /// exactly this.
    bool loadCreature(const std::string& m2Path,
                      const std::vector<std::pair<uint32_t, std::string>>& skins = {});

    // Apply equipment overlays/geosets using SMSG_CHAR_ENUM equipment data (ItemDisplayInfo.dbc).
    bool applyEquipment(const std::vector<game::EquipmentItem>& equipment);

    void update(float deltaTime);
    void render();
    void rotate(float yawDelta);
    void zoom(float wheelDelta);
    void resetView();

    /// Frames the head straight on, for a portrait.
    ///
    /// The viewing direction otherwise comes from whichever racial backdrop
    /// scene was loaded, and the model is turned to match only in that same
    /// call - so without a scene the camera sits along one axis while the model
    /// still faces another, and the portrait shows a profile. This sets both
    /// together and zooms to the face.
    void setPortraitFraming();

    /// Draws the character against nothing rather than a lit backdrop, so what
    /// surrounds it is transparent. A portrait is masked by the frame art
    /// around it, and anything opaque behind the head shows as a block of
    /// colour inside that frame.
    void setTransparentBackground(bool transparent);

    // Off-screen composite pass - call from Renderer::beginFrame() before main render pass
    void compositePass(VkCommandBuffer cmd, uint32_t frameIndex);

    // Mark that the preview needs compositing this frame (call from UI each frame)
    void requestComposite() { compositeRequested_ = true; }

    // Returns the ImGui texture handle. Returns VK_NULL_HANDLE until the first
    // compositePass has run (image is in UNDEFINED layout before that).
    [[nodiscard]] VkDescriptorSet getTextureId() const { return compositeRendered_ ? imguiTextureId_ : VK_NULL_HANDLE; }
    [[nodiscard]] int getWidth() const { return fboWidth_; }
    [[nodiscard]] int getHeight() const { return fboHeight_; }

    CharacterRenderer* getCharacterRenderer() { return charRenderer_.get(); }
    [[nodiscard]] uint32_t getInstanceId() const { return instanceId_; }
    [[nodiscard]] uint32_t getModelId() const { return PREVIEW_MODEL_ID; }
    [[nodiscard]] bool isModelLoaded() const { return modelLoaded_; }

private:
    struct FacialHairGeosets {
        uint16_t geoset100 = 0;
        uint16_t geoset300 = 0;
        uint16_t geoset200 = 0;
    };

    void createFBO();
    void destroyFBO();
    void ensureAppearanceGeosetsLoaded();
    std::unordered_set<uint16_t> buildBaseGeosets();
    [[nodiscard]] uint16_t selectedHairScalpGeoset() const;

    // Read an M2 (plus its .skin for WotLK-era models) through the asset manager.
    bool loadPreviewM2(const std::string& m2Path, pipeline::M2Model& outModel);
    // Hang the character's weapons off the preview model's hand attachments.
    void attachWeapons(const std::vector<game::EquipmentItem>& equipment);
    // Put the weapon's enchant glint on it (char enum reports the ItemVisual id directly).
    void attachWeaponEnchantVisual(uint32_t attachmentId, uint32_t itemVisualId);
    // Load the race's glue scene (Stormwind for humans, Orgrimmar for orcs, ...) as a backdrop.
    void loadRacialBackdrop(game::Race race);
    void applyPreviewView();

    pipeline::AssetManager* assetManager_ = nullptr;
    VkContext* vkCtx_ = nullptr;
    std::unique_ptr<CharacterRenderer> charRenderer_;
    std::unique_ptr<Camera> camera_;

    // Off-screen render target (color + depth)
    std::unique_ptr<VkRenderTarget> renderTarget_;

    // Per-frame UBO for preview camera/lighting (double-buffered)
    static constexpr uint32_t MAX_FRAMES = 2;
    VkDescriptorPool previewDescPool_ = VK_NULL_HANDLE;
    VkBuffer previewUBO_[MAX_FRAMES] = {};
    VmaAllocation previewUBOAlloc_[MAX_FRAMES] = {};
    void* previewUBOMapped_[MAX_FRAMES] = {};
    VkDescriptorSet previewPerFrameSet_[MAX_FRAMES] = {};

    // Dummy 1x1 depth texture for shadow map placeholder (sampler2DShadow compatible)
    VkImage dummyShadowImage_ = VK_NULL_HANDLE;
    VkImageView dummyShadowView_ = VK_NULL_HANDLE;
    VmaAllocation dummyShadowAlloc_ = VK_NULL_HANDLE;

    // ImGui texture handle for displaying the preview (VkDescriptorSet in Vulkan backend)
    VkDescriptorSet imguiTextureId_ = VK_NULL_HANDLE;

    // 4:5 portrait aspect ratio - taller than wide to show full character body
    // from head to feet in the character creation/selection screen. Rendered at
    // roughly the size it is displayed at, so the larger panel is not upscaled mush.
    int fboWidth_ = 640;
    int fboHeight_ = 800;

    static constexpr uint32_t PREVIEW_MODEL_ID = 9999;
    static constexpr uint32_t PREVIEW_BACKDROP_MODEL_ID = 9996;

    // CharacterRenderer::loadModel() keeps a model cache keyed by id and skips
    // loading when the id is already present, so a fixed id per hand would hand
    // every subsequent character the first character's weapon. Key the id by what
    // is actually being loaded instead: same weapon reuses the model, different
    // weapon gets its own.
    uint32_t previewModelIdFor(const std::string& assetKey);
    std::unordered_map<std::string, uint32_t> previewModelIds_;
    uint32_t nextPreviewModelId_ = 20000;
    uint32_t backdropInstanceId_ = 0;
    int backdropRace_ = -1;   // race whose glue scene is currently loaded (-1 = none)
    uint32_t instanceId_ = 0;
    bool modelLoaded_ = false;
    bool compositeRequested_ = false;
    bool compositeRendered_ = false;  // True after first successful compositePass
    float modelYaw_ = 90.0f;

    // Character-creation portrait rig. The full-body distance is recomputed for
    // each race; wheel zoom interpolates its focus upward toward the face.
    float zoomLevel_ = 0.0f;
    float fullBodyDistance_ = 4.5f;
    /// The model's own portrait camera, where it has one. Framing a face by a
    /// fraction of the bind-pose height assumes the pose the model stands in is
    /// the pose it is drawn in, and a gnoll's is not: it stands upright in bind
    /// pose and hunches when animated, so the guess aimed a head above the head.
    bool portraitCameraValid_ = false;
    /// The field of view the preview was built with. A portrait camera carries
    /// its own, and one preview is shared by players and creatures, so the
    /// creature's has to be handed back or the next player portrait wears it.
    float defaultFovDegrees_ = 0.0f;
    glm::vec3 portraitCameraPos_{0.0f};
    glm::vec3 portraitCameraTarget_{0.0f};
    float portraitCameraFovDegrees_ = 0.0f;
    float modelBoundMinZ_ = 0.0f;
    float modelBoundMaxZ_ = 2.0f;
    glm::vec3 previewStandPosition_{0.0f};
    glm::vec3 previewViewDirection_{0.0f, 1.0f, 0.0f};
    bool transparentBackground_ = false;

    // Cached info from loadCharacter() for later recompositing.
    game::Race race_ = game::Race::HUMAN;
    game::Gender gender_ = game::Gender::MALE;
    bool useFemaleModel_ = false;
    uint8_t hairStyle_ = 0;
    uint8_t facialHair_ = 0;
    std::string bodySkinPath_;
    /// CharSections' second texture on the skin row: the Skin Extra art an
    /// HD model draws its ears, eyes and mouth from.
    std::string skinExtraPath_;
    std::vector<std::string> baseLayers_; // face + underwear, etc.
    uint32_t skinTextureSlotIndex_ = 0;

    bool appearanceGeosetsLoaded_ = false;
    std::unordered_map<uint32_t, uint16_t> hairGeosetMap_;
    std::unordered_map<uint32_t, FacialHairGeosets> facialHairGeosetMap_;
};

} // namespace rendering
} // namespace wowee
