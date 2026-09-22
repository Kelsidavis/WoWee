#pragma once

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>
#include <glm/glm.hpp>
#include <cstdint>

namespace wowee {
namespace rendering {

class VkContext;

/**
 * Fog that light moves through: lit by the sun past the shadow map, so trees
 * and buildings cast shafts through it, by the zone's own fog colour where
 * the sun does not reach, and by the torches, braziers and lava the renderer
 * already gathers for the surfaces around them.
 *
 * It is a volume over the camera's frustum - a grid of cells, 'froxels', each
 * a pixel tile deep by a slice of distance, the slices growing exponentially
 * so the near air is fine and the far air coarse. Two compute passes build it
 * every frame after the shadow map:
 *
 *   inject     fills each cell with how much its air scatters and absorbs
 *              and how much light arrives there, sampled at a jittered point
 *              and blended with last frame's cell, reprojected, so the jitter
 *              averages out instead of crawling;
 *   integrate  walks each column away from the camera and stores, at every
 *              depth, the light scattered toward the camera so far and how
 *              much of what lies behind still shows through.
 *
 * Every world shader then reads that volume at its own position where it used
 * to add fog alone. The zone's distance fog is kept and applied first: it is
 * the far haze the sky is painted to meet, and the volume is the air in front
 * of it - which is also what lets the sky take the volume and keep matching
 * the terrain at the horizon.
 *
 * The volumes are double-buffered per frame in flight, as the shadow maps are:
 * a frame writes its own and reads the other one's cells as history.
 */
class VolumetricFog {
public:
    /// Off, then three volume sizes. The order is persisted in the settings
    /// file as an index, so add at the end.
    enum class Quality : int { Off = 0, Low = 1, Medium = 2, High = 3 };

    /// How deep the volume reaches, in yards along the view. The air thins
    /// to nothing over its back two thirds, so the last slice holds all of it
    /// and stands for everything further, which is what the sky reads; the
    /// zone's distance fog is the haze beyond.
    ///
    /// The near air only, on purpose. A shaft is a few yards of sunlit air,
    /// and it shows only where those yards are a real share of all the air
    /// the eye looks through. Air thick enough for that all the way out
    /// whites the horizon long before a shaft reads; air that thins with
    /// distance can be thick where the trees and doorways are.
    static constexpr float kNear = 0.5f;
    static constexpr float kFar = 400.0f;

    /// What one frame's volume is built from.
    struct FrameInputs {
        glm::mat4 view{1.0f};
        glm::mat4 projection{1.0f};
        glm::vec3 cameraPos{0.0f};
        float time = 0.0f;           ///< seconds, for the drift of the mist
        /// Extinction per yard at and below the layer's base.
        float density = 0.0f;
        /// World height the densest air sits on.
        float layerBase = 0.0f;
        /// Yards over which the density falls by a factor of e above that.
        float layerHeight = 30.0f;
        /// Share of the density still left high above the layer, so a shaft
        /// seen from a flying mount does not vanish for want of air. Kept low:
        /// the zenith looks through the whole depth of it.
        float layerFloor = 0.1f;
        /// 0 is even mist; 1 lets it thin to nothing and pile to double.
        float noiseAmount = 0.6f;
        /// Brightness of the sunlit air. Most of it goes forward, so this is
        /// mostly what the air toward the sun does.
        float sunScatter = 0.8f;
        /// Brightness of the air the sun does not reach, lit by the zone's fog
        /// colour. Half, so a shaft stands against the shade beside it rather
        /// than against a haze nearly as bright.
        float ambientScatter = 0.5f;
        /// Brightness of the glow around local lights. Well above one because
        /// a torch's reach is a dozen yards, and that little air scatters
        /// little of it.
        float localLightScatter = 8.0f;
    };

    VolumetricFog() = default;
    ~VolumetricFog();
    VolumetricFog(const VolumetricFog&) = delete;
    VolumetricFog& operator=(const VolumetricFog&) = delete;

    /// The parts the per-frame descriptor layout needs before it exists: the
    /// sampler it bakes in and the neutral volume every set can bind.
    [[nodiscard]] bool initialize(VkContext* ctx);

    /// The compute passes. Set 0 is the renderer's per-frame layout, whose
    /// UBO they read; the shadow views are the per-frame-slot shadow maps.
    [[nodiscard]] bool createPipelines(VkDescriptorSetLayout perFrameLayout,
                                       const VkImageView shadowViews[2]);

    void shutdown();

    /// Takes effect at the next applyPendingQuality().
    void setQuality(Quality quality) { pendingQuality_ = quality; }

    /// Builds or frees the volumes for a quality change. Called between
    /// frames; waits for the device when there is something to change.
    /// Returns true when getVolumeView() now answers differently, so the
    /// per-frame descriptor sets must be written again.
    [[nodiscard]] bool applyPendingQuality();

    /// Volumes exist and a frame can build one.
    [[nodiscard]] bool isOn() const { return volumesReady_; }

    /// GPUPerFrameData::volumetricParams for a frame that builds its volume.
    [[nodiscard]] glm::vec4 frameParams() const;

    [[nodiscard]] VkSampler getSampler() const { return sampler_; }
    [[nodiscard]] VkImageView getNeutralView() const { return neutral_.view; }
    /// What set 0 binding 2 should hold for this frame slot: its integrated
    /// volume, or the neutral one while the fog is off.
    [[nodiscard]] VkImageView getVolumeView(uint32_t frame) const;

    /// Build this frame slot's volume. Outside any render pass, after the
    /// shadow map is drawn and left readable, and before anything samples
    /// binding 2.
    void record(VkCommandBuffer cmd, uint32_t frame, VkDescriptorSet perFrameSet,
                const FrameInputs& in);

    /// Forget the previous frames, after a teleport or a loading screen:
    /// reprojecting air from somewhere else would smear it across the view.
    void resetHistory() { historyValid_ = false; }

private:
    static constexpr uint32_t MAX_FRAMES = 2;

    struct Volume {
        VkImage image = VK_NULL_HANDLE;
        VmaAllocation alloc = VK_NULL_HANDLE;
        VkImageView view = VK_NULL_HANDLE;
    };

    bool createVolume(Volume& v, glm::uvec3 size, VkImageUsageFlags usage);
    void destroyVolume(Volume& v);
    bool createVolumes(glm::uvec3 size);
    void destroyVolumes();
    void writeComputeSets();

    VkContext* ctx_ = nullptr;
    VkSampler sampler_ = VK_NULL_HANDLE;       // trilinear, clamped; owned by the context cache
    VkSampler depthSampler_ = VK_NULL_HANDLE;  // nearest, for reading the shadow map's depth
    Volume neutral_;                           // 1x1x1: no light scattered, everything through

    // Per frame slot: the lit cells and their running sum.
    Volume scatter_[MAX_FRAMES];
    Volume integrated_[MAX_FRAMES];
    glm::uvec3 size_{0};
    bool volumesReady_ = false;
    bool formatSupported_ = true;
    bool pipelinesReady_ = false;

    Quality pendingQuality_ = Quality::Off;
    Quality builtQuality_ = Quality::Off;

    VkImageView shadowViews_[MAX_FRAMES] = {};
    VkDescriptorSetLayout setLayout_ = VK_NULL_HANDLE;
    VkPipelineLayout pipelineLayout_ = VK_NULL_HANDLE;
    VkPipeline injectPipeline_ = VK_NULL_HANDLE;
    VkPipeline integratePipeline_ = VK_NULL_HANDLE;
    VkDescriptorPool descPool_ = VK_NULL_HANDLE;
    VkDescriptorSet computeSets_[MAX_FRAMES] = {};

    VkBuffer paramsUBO_[MAX_FRAMES] = {};
    VmaAllocation paramsAlloc_[MAX_FRAMES] = {};
    void* paramsMapped_[MAX_FRAMES] = {};

    // Reprojection state.
    glm::mat4 prevViewProj_{1.0f};
    glm::vec3 prevCameraPos_{0.0f};
    bool historyValid_ = false;
    uint32_t frameCounter_ = 0;
    int lastRecordedFrame_ = -1;
};

} // namespace rendering
} // namespace wowee
