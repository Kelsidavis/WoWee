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
    bool initialize();
    void shutdown();
    bool isInitialized() const { return initialized_; }

    // Master volume (0.0 = silent, 1.0 = full)
    void setMasterVolume(float volume);
    float getMasterVolume() const { return masterVolume_; }

    // 3D listener position (for positional audio)
    void setListenerPosition(const glm::vec3& position);
    void setListenerOrientation(const glm::vec3& forward, const glm::vec3& up);
    const glm::vec3& getListenerPosition() const { return listenerPosition_; }

    // Simple 2D sound playback (non-blocking)
    bool playSound2D(const std::vector<uint8_t>& wavData, float volume = 1.0f, float pitch = 1.0f);
    bool playSound2D(const std::string& mpqPath, float volume = 1.0f, float pitch = 1.0f);

    // 3D positional sound playback
    bool playSound3D(const std::vector<uint8_t>& wavData, const glm::vec3& position,
                     float volume = 1.0f, float pitch = 1.0f, float maxDistance = 100.0f);
    bool playSound3D(const std::string& mpqPath, const glm::vec3& position,
                     float volume = 1.0f, float pitch = 1.0f, float maxDistance = 100.0f);

    // Music streaming (for background music)
    bool playMusic(const std::vector<uint8_t>& musicData, float volume = 1.0f, bool loop = true);
    void stopMusic();
    bool isMusicPlaying() const;
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
    };
    std::vector<ActiveSound> activeSounds_;

    // Music track state
    ma_sound* musicSound_ = nullptr;
    void* musicDecoder_ = nullptr;  // ma_decoder* - Keep decoder alive for streaming
    std::vector<uint8_t> musicData_;  // Keep encoded music data alive
    float musicVolume_ = 1.0f;

    bool initialized_ = false;
    float masterVolume_ = 1.0f;
    glm::vec3 listenerPosition_{0.0f, 0.0f, 0.0f};
    glm::vec3 listenerForward_{0.0f, 0.0f, -1.0f};
    glm::vec3 listenerUp_{0.0f, 1.0f, 0.0f};

    // miniaudio engine (opaque pointer)
    ma_engine* engine_ = nullptr;
};

} // namespace audio
} // namespace wowee
