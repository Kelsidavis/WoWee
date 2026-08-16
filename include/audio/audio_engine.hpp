#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <string>
#include <vector>
#include <cstdint>

// Forward declare miniaudio types to avoid exposing implementation in header
struct ma_engine;
struct ma_sound;

namespace wowee {
namespace pipeline { class AssetManager; }
namespace audio {

/**
 * AudioEngine: Singleton managing miniaudio device and playback.
 * Replaces process-spawning audio system with proper non-blocking library.
 */
class AudioEngine {
public:
    static AudioEngine& instance();

    ~AudioEngine();

    // Initialization
    [[nodiscard]] bool initialize();
    void shutdown();
    [[nodiscard]] bool isInitialized() const { return initialized_; }

    /// The playback device miniaudio actually opened, by name.
    ///
    /// For the Sound panel's output dropdown, which lists the drivers the
    /// client can use. This client opens whichever device the system offers
    /// and does not switch between them, so the list is this one name - the
    /// truth rather than a guess, and the same shape the video panel's
    /// resolution list already takes. Empty if the engine is not up.
    [[nodiscard]] std::string getOutputDeviceName() const;

    // Master volume (0.0 = silent, 1.0 = full)
    void setMasterVolume(float volume);
    [[nodiscard]] float getMasterVolume() const { return masterVolume_; }

    /// Silence the output without forgetting the volume the player chose.
    ///
    /// For the window losing focus, where the client should go quiet and come
    /// back at the same level. Winding masterVolume_ down to zero instead
    /// would work once and then answer the volume slider wrongly, and would
    /// trip the masterVolume_ <= 0 early-outs that stop a sound being started
    /// at all - so a suspended client would come back to silence where a
    /// looping track used to be.
    void setSuspended(bool suspended);
    [[nodiscard]] bool isSuspended() const { return suspended_; }

    // Asset manager (enables sound loading by MPQ path)
    void setAssetManager(pipeline::AssetManager* am) { assetManager_ = am; }

    // 3D listener position (for positional audio)
    void setListenerPosition(const glm::vec3& position);
    void setListenerOrientation(const glm::vec3& forward, const glm::vec3& up);
    [[nodiscard]] const glm::vec3& getListenerPosition() const { return listenerPosition_; }

    // Simple 2D sound playback (non-blocking)
    bool playSound2D(const std::vector<uint8_t>& wavData, float volume = 1.0f, float pitch = 1.0f);
    bool playSound2D(const std::string& mpqPath, float volume = 1.0f, float pitch = 1.0f);

    // Stoppable 2D sound - returns a non-zero handle, or 0 on failure
    uint32_t playSound2DStoppable(const std::vector<uint8_t>& wavData, float volume = 1.0f);
    // Stop a sound started with playSound2DStoppable (no-op if already finished)
    void stopSound(uint32_t id);

    // 3D positional sound playback
    bool playSound3D(const std::vector<uint8_t>& wavData, const glm::vec3& position,
                     float volume = 1.0f, float pitch = 1.0f, float maxDistance = 100.0f);
    bool playSound3D(const std::string& mpqPath, const glm::vec3& position,
                     float volume = 1.0f, float pitch = 1.0f, float maxDistance = 100.0f);

    // Music streaming (for background music)
    // Retains shared ownership: the decoder streams directly from encoded tracks that
    // run to several MB, so cached music must not be copied for every playback.
    bool playMusic(std::shared_ptr<const std::vector<uint8_t>> musicData,
                   float volume = 1.0f, bool loop = true);
    void stopMusic();
    [[nodiscard]] bool isMusicPlaying() const;
    void setMusicVolume(float volume);

    // Update (call once per frame for cleanup/position sync)
    void update(float deltaTime);

private:
    AudioEngine();
    AudioEngine(const AudioEngine&) = delete;
    AudioEngine& operator=(const AudioEngine&) = delete;

    // Track active one-shot sounds for cleanup
    struct ActiveSound {
        ma_sound* sound;
        void* buffer;  // ma_audio_buffer* - Keep audio buffer alive
        std::shared_ptr<const std::vector<uint8_t>> pcmDataRef;  // Keep decoded PCM alive
        uint32_t id = 0;  // 0 = anonymous (not stoppable)
    };
    std::vector<ActiveSound> activeSounds_;
    uint32_t nextSoundId_ = 1;

    // Music track state
    ma_sound* musicSound_ = nullptr;
    void* musicDecoder_ = nullptr;  // ma_decoder* - Keep decoder alive for streaming
    std::shared_ptr<const std::vector<uint8_t>> musicData_;  // Keep encoded music data alive
    float musicVolume_ = 1.0f;

    bool initialized_ = false;
    float masterVolume_ = 1.0f;
    bool suspended_ = false;
    glm::vec3 listenerPosition_{0.0f, 0.0f, 0.0f};
    glm::vec3 listenerForward_{0.0f, 0.0f, -1.0f};
    glm::vec3 listenerUp_{0.0f, 1.0f, 0.0f};

    pipeline::AssetManager* assetManager_ = nullptr;

    // miniaudio engine (opaque pointer)
    ma_engine* engine_ = nullptr;
};

} // namespace audio
} // namespace wowee
