#include "audio/music_manager.hpp"
#include "audio/audio_engine.hpp"
#include "pipeline/asset_manager.hpp"
#include "core/logger.hpp"
#include <algorithm>
#include <filesystem>
#include <fstream>

namespace wowee {
namespace audio {

MusicManager::MusicManager() = default;

MusicManager::~MusicManager() {
    shutdown();
}

bool MusicManager::initialize(pipeline::AssetManager* assets) {
    assetManager = assets;
    LOG_INFO("Music manager initialized");
    return true;
}

float MusicManager::effectiveMusicVolume() const {
    float vol = volumePercent / 100.0f;
    if (underwaterMode) {
        vol *= 0.3f;
    }
    return vol;
}

void MusicManager::shutdown() {
    AudioEngine::instance().stopMusic();
    playing = false;
    fadingIn = false;
    fadeInTimer = 0.0f;
    fadeInDuration = 0.0f;
    fadeInTargetVolume = 0.0f;
    currentTrack.clear();
    musicDataCache_.clear();
}

void MusicManager::preloadMusic(const std::string& mpqPath) {
    if (!assetManager || mpqPath.empty()) return;
    if (musicDataCache_.find(mpqPath) != musicDataCache_.end()) return;

    auto data = assetManager->readFile(mpqPath);
    if (!data.empty()) {
        musicDataCache_[mpqPath] = std::move(data);
    }
}

void MusicManager::playMusic(const std::string& mpqPath, bool loop, float fadeInMs) {
    if (!assetManager) return;
    if (mpqPath == currentTrack && playing) return;

    // Check if AudioEngine is ready
    if (!AudioEngine::instance().isInitialized()) {
        LOG_WARNING("Music: AudioEngine not initialized");
        return;
    }

    // Read music file from cache or MPQ
    auto cacheIt = musicDataCache_.find(mpqPath);
    if (cacheIt == musicDataCache_.end()) {
        preloadMusic(mpqPath);
        cacheIt = musicDataCache_.find(mpqPath);
    }
    if (cacheIt == musicDataCache_.end() || cacheIt->second.empty()) {
        LOG_WARNING("Music: Could not read: ", mpqPath);
        return;
    }

    // Play with AudioEngine (non-blocking, streams from memory)
    float targetVolume = effectiveMusicVolume();
    float startVolume = (fadeInMs > 0.0f) ? 0.0f : targetVolume;
    if (AudioEngine::instance().playMusic(cacheIt->second, startVolume, loop)) {
        playing = true;
        fadingIn = false;
        if (fadeInMs > 0.0f) {
            fadingIn = true;
            fadeInTimer = 0.0f;
            fadeInDuration = std::max(0.05f, fadeInMs / 1000.0f);
            fadeInTargetVolume = targetVolume;
            AudioEngine::instance().setMusicVolume(0.0f);
        }
        currentTrack = mpqPath;
        currentTrackIsFile = false;
        LOG_INFO("Music: Playing ", mpqPath);
    } else {
        LOG_ERROR("Music: Failed to play music via AudioEngine");
    }
}

void MusicManager::playFilePath(const std::string& filePath, bool loop, float fadeInMs) {
    if (filePath.empty()) return;
    if (filePath == currentTrack && playing) return;
    if (!std::filesystem::exists(filePath)) {
        LOG_WARNING("Music: file not found: ", filePath);
        return;
    }

    // Check if AudioEngine is ready
    if (!AudioEngine::instance().isInitialized()) {
        LOG_WARNING("Music: AudioEngine not initialized");
        return;
    }

    // Read file into memory
    std::ifstream file(filePath, std::ios::binary | std::ios::ate);
    if (!file) {
        LOG_ERROR("Music: Could not open file: ", filePath);
        return;
    }

    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<uint8_t> data(static_cast<size_t>(size));
    if (!file.read(reinterpret_cast<char*>(data.data()), size)) {
        LOG_ERROR("Music: Could not read file: ", filePath);
        return;
    }

    // Play with AudioEngine
    float targetVolume = effectiveMusicVolume();
    float startVolume = (fadeInMs > 0.0f) ? 0.0f : targetVolume;
    if (AudioEngine::instance().playMusic(data, startVolume, loop)) {
        playing = true;
        fadingIn = false;
        if (fadeInMs > 0.0f) {
            fadingIn = true;
            fadeInTimer = 0.0f;
            fadeInDuration = std::max(0.05f, fadeInMs / 1000.0f);
            fadeInTargetVolume = targetVolume;
            AudioEngine::instance().setMusicVolume(0.0f);
        }
        currentTrack = filePath;
        currentTrackIsFile = true;
        LOG_INFO("Music: Playing file ", filePath);
    } else {
        LOG_ERROR("Music: Failed to play music via AudioEngine");
    }
}

void MusicManager::stopMusic(float fadeMs) {
    (void)fadeMs;  // Fade not implemented yet
    AudioEngine::instance().stopMusic();
    playing = false;
    fadingIn = false;
    fadeInTimer = 0.0f;
    fadeInDuration = 0.0f;
    fadeInTargetVolume = 0.0f;
    currentTrack.clear();
    currentTrackIsFile = false;
}

void MusicManager::setVolume(int volume) {
    if (volume < 0) volume = 0;
    if (volume > 100) volume = 100;
    if (volumePercent == volume) return;
    volumePercent = volume;

    // Update AudioEngine music volume directly (no restart needed!)
    float vol = effectiveMusicVolume();
    if (fadingIn) {
        fadeInTargetVolume = vol;
        float t = std::clamp(fadeInTimer / std::max(fadeInDuration, 0.001f), 0.0f, 1.0f);
        AudioEngine::instance().setMusicVolume(fadeInTargetVolume * t);
    } else {
        AudioEngine::instance().setMusicVolume(vol);
    }
}

void MusicManager::setUnderwaterMode(bool underwater) {
    if (underwaterMode == underwater) return;
    underwaterMode = underwater;

    // Apply volume change immediately
    float vol = effectiveMusicVolume();
    if (fadingIn) {
        fadeInTargetVolume = vol;
        float t = std::clamp(fadeInTimer / std::max(fadeInDuration, 0.001f), 0.0f, 1.0f);
        AudioEngine::instance().setMusicVolume(fadeInTargetVolume * t);
    } else {
        AudioEngine::instance().setMusicVolume(vol);
    }
}

void MusicManager::crossfadeTo(const std::string& mpqPath, float fadeMs) {
    if (mpqPath == currentTrack && playing) return;

    // Simple implementation: stop and start (no actual crossfade yet)
    if (fadeMs > 0 && playing) {
        crossfading = true;
        pendingTrack = mpqPath;
        pendingIsFile = false;
        fadeTimer = 0.0f;
        fadeDuration = fadeMs / 1000.0f;
        AudioEngine::instance().stopMusic();
    } else {
        playMusic(mpqPath, false);
    }
}

void MusicManager::crossfadeToFile(const std::string& filePath, float fadeMs) {
    if (filePath == currentTrack && playing) return;

    if (fadeMs > 0 && playing) {
        crossfading = true;
        pendingTrack = filePath;
        pendingIsFile = true;
        fadeTimer = 0.0f;
        fadeDuration = fadeMs / 1000.0f;
        AudioEngine::instance().stopMusic();
    } else {
        playFilePath(filePath, false);
    }
}

void MusicManager::update(float deltaTime) {
    // Check if music is still playing
    if (playing && !AudioEngine::instance().isMusicPlaying()) {
        playing = false;
    }

    if (fadingIn) {
        fadeInTimer += deltaTime;
        float t = std::clamp(fadeInTimer / std::max(fadeInDuration, 0.001f), 0.0f, 1.0f);
        AudioEngine::instance().setMusicVolume(fadeInTargetVolume * t);
        if (t >= 1.0f) {
            fadingIn = false;
        }
    }

    // Handle crossfade
    if (crossfading) {
        fadeTimer += deltaTime;
        if (fadeTimer >= fadeDuration * 0.3f) {
            // Start new track after brief pause
            crossfading = false;
            if (pendingIsFile) {
                playFilePath(pendingTrack, false);
            } else {
                playMusic(pendingTrack, false);
            }
            pendingTrack.clear();
            pendingIsFile = false;
        }
    }
}

} // namespace audio
} // namespace wowee
