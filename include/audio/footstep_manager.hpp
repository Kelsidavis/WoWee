#pragma once

#include <cstdint>
#include <random>
#include <string>
#include <vector>
#include <chrono>

namespace wowee {
namespace pipeline { class AssetManager; }

namespace audio {

enum class FootstepSurface : uint8_t {
    STONE = 0,
    DIRT,
    GRASS,
    WOOD,
    METAL,
    WATER,
    SNOW
};

// Which sample bank a step draws from.
// Note: the heavy bank is named HEAVY (not HUGE) because Windows math headers
// define HUGE as a macro.
enum class FootstepBank : uint8_t {
    CHARACTER = 0,  // medium/large biped footsteps (also padded mounts: wolf/tiger/raptor)
    HORSE,          // hoofed mounts (horse/skeletal horse/ram)
    HEAVY           // heavy mounts (kodo) - mFootHuge* sample set
};

class FootstepManager {
public:
    FootstepManager();
    ~FootstepManager();

    bool initialize(pipeline::AssetManager* assets);
    void shutdown();

    void update(float deltaTime);
    void playFootstep(FootstepSurface surface, bool sprinting);
    void playMountFootstep(FootstepSurface surface, FootstepBank bank);
    void setVolumeScale(float scale) { volumeScale = scale; }
    [[nodiscard]] float getVolumeScale() const { return volumeScale; }

    [[nodiscard]] bool isInitialized() const { return assetManager != nullptr; }
    [[nodiscard]] bool hasAnySamples() const { return sampleCount > 0; }

private:
    struct Sample {
        std::string path;
        std::vector<uint8_t> data;
    };

    struct SurfaceSamples {
        std::vector<Sample> clips;
    };

    void preloadSurface(SurfaceSamples* bank, FootstepSurface surface,
                        const std::vector<std::string>& candidates, const char* bankName);
    bool playRandomStep(FootstepSurface surface, FootstepBank bank,
                        float minInterval, float volumeMul);
    static const char* surfaceName(FootstepSurface surface);

    pipeline::AssetManager* assetManager = nullptr;
    SurfaceSamples surfaces[7];
    SurfaceSamples horseSurfaces[7];
    SurfaceSamples hugeSurfaces[7];
    size_t sampleCount = 0;

    std::chrono::steady_clock::time_point lastPlayTime = std::chrono::steady_clock::time_point{};

    std::mt19937 rng;
    float volumeScale = 1.0f;
};

} // namespace audio
} // namespace wowee
