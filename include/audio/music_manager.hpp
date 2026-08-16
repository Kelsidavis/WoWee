#pragma once

#include <string>
#include <cstdint>
#include <future>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

namespace wowee {
namespace pipeline { class AssetManager; }

namespace audio {

class MusicManager {
public:
    MusicManager();
    ~MusicManager();

    bool initialize(pipeline::AssetManager* assets);
    void shutdown();

    void playMusic(const std::string& mpqPath, bool loop = true, float fadeInMs = 0.0f);
    void playFilePath(const std::string& filePath, bool loop = true, float fadeInMs = 0.0f);
    void stopMusic(float fadeMs = 2000.0f);
    /// Loop Music, from the audio options. Takes effect on the next track.
    void setLooping(bool loop) { loopTracks_ = loop; }
    [[nodiscard]] bool isLooping() const { return loopTracks_; }

    void crossfadeTo(const std::string& mpqPath, float fadeMs = 3000.0f);
    void crossfadeToFile(const std::string& filePath, float fadeMs = 3000.0f);
    void update(float deltaTime);
    void setVolume(int volume);
    [[nodiscard]] int getVolume() const { return volumePercent; }
    void setUnderwaterMode(bool underwater);
    void preloadMusic(const std::string& mpqPath);

    [[nodiscard]] bool isPlaying() const { return playing; }
    /// True while a track is being read on the worker but has not started yet.
    /// Callers must treat this as "in progress", not as failure.
    [[nodiscard]] bool isLoading() const { return pendingFileLoad_.has_value(); }
    [[nodiscard]] bool isInitialized() const { return assetManager != nullptr; }
    [[nodiscard]] const std::string& getCurrentTrack() const { return currentTrack; }
    /// True when the current track came from the filesystem (WoWee original
    /// music) rather than a game archive.
    [[nodiscard]] bool isCurrentTrackFile() const { return currentTrackIsFile; }

private:
    [[nodiscard]] float effectiveMusicVolume() const;

    // Tracks run to several MB, so reading one on the calling thread stalls whatever
    // frame asked for it - starting the login music was costing ~200ms of render time.
    // The read happens on a worker; update() hands the bytes to the AudioEngine once
    // they land, keeping every miniaudio call on the main thread.
    struct PendingFileLoad {
        std::future<std::shared_ptr<const std::vector<uint8_t>>> future;
        std::string path;
        bool loop = true;
        float fadeInMs = 0.0f;
    };
    std::optional<PendingFileLoad> pendingFileLoad_;
    void cancelPendingFileLoad();
    void pollPendingFileLoad();

    pipeline::AssetManager* assetManager = nullptr;
    std::string currentTrack;
    bool currentTrackIsFile = false;
    bool playing = false;
    int volumePercent = 30;
    bool underwaterMode = false;

    // Crossfade state
    /// Loop Music: whether a zone track runs on instead of stopping at its
    /// end. Every place a track is started reads this, including the one that
    /// starts the pending track after a crossfade - a flag applied at three of
    /// four sites is worse than none, because the exception is the one nobody
    /// remembers.
    bool loopTracks_ = false;
    bool crossfading = false;
    std::string pendingTrack;
    bool pendingIsFile = false;
    float fadeTimer = 0.0f;
    float fadeDuration = 0.0f;
    bool fadingIn = false;
    float fadeInTimer = 0.0f;
    float fadeInDuration = 0.0f;
    float fadeInTargetVolume = 0.0f;
    // Fade-out state (for stopMusic with fadeMs > 0)
    bool fadingOut = false;
    float fadeOutTimer = 0.0f;
    float fadeOutDuration = 0.0f;
    float fadeOutStartVolume = 0.0f;

    std::unordered_map<std::string, std::shared_ptr<const std::vector<uint8_t>>> musicDataCache_;
};

} // namespace audio
} // namespace wowee
