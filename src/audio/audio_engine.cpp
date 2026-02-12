#define MINIAUDIO_IMPLEMENTATION
#include "audio/audio_engine.hpp"
#include "core/logger.hpp"
#include "pipeline/asset_manager.hpp"

#include "../../extern/miniaudio.h"

#include <cstring>
#include <memory>
#include <unordered_map>

namespace wowee {
namespace audio {

namespace {

struct DecodedWavCacheEntry {
    ma_format format = ma_format_unknown;
    ma_uint32 channels = 0;
    ma_uint32 sampleRate = 0;
    ma_uint64 frames = 0;
    std::shared_ptr<std::vector<uint8_t>> pcmData;
};

static std::unordered_map<uint64_t, DecodedWavCacheEntry> gDecodedWavCache;

static uint64_t makeWavCacheKey(const std::vector<uint8_t>& wavData) {
    uint64_t ptr = static_cast<uint64_t>(reinterpret_cast<uintptr_t>(wavData.data()));
    uint64_t sz = static_cast<uint64_t>(wavData.size());
    return (ptr * 11400714819323198485ull) ^ (sz + 0x9e3779b97f4a7c15ull + (ptr << 6) + (ptr >> 2));
}

static bool decodeWavCached(const std::vector<uint8_t>& wavData, DecodedWavCacheEntry& out) {
    if (wavData.empty()) return false;

    const uint64_t key = makeWavCacheKey(wavData);
    if (auto it = gDecodedWavCache.find(key); it != gDecodedWavCache.end()) {
        out = it->second;
        return true;
    }

    ma_decoder decoder;
    ma_decoder_config decoderConfig = ma_decoder_config_init_default();
    ma_result result = ma_decoder_init_memory(
        wavData.data(),
        wavData.size(),
        &decoderConfig,
        &decoder
    );
    if (result != MA_SUCCESS) {
        LOG_ERROR("AudioEngine: Failed to decode WAV data (", wavData.size(), " bytes): error ", result);
        return false;
    }

    ma_uint64 totalFrames = 0;
    result = ma_decoder_get_length_in_pcm_frames(&decoder, &totalFrames);
    if (result != MA_SUCCESS) totalFrames = 0;

    ma_format format = decoder.outputFormat;
    ma_uint32 channels = decoder.outputChannels;
    ma_uint32 sampleRate = decoder.outputSampleRate;
    ma_uint64 maxFrames = sampleRate * 60;
    if (totalFrames == 0 || totalFrames > maxFrames) totalFrames = maxFrames;

    size_t bufferSize = totalFrames * channels * ma_get_bytes_per_sample(format);
    auto pcmData = std::make_shared<std::vector<uint8_t>>(bufferSize);
    ma_uint64 framesRead = 0;
    result = ma_decoder_read_pcm_frames(&decoder, pcmData->data(), totalFrames, &framesRead);
    ma_decoder_uninit(&decoder);
    if (result != MA_SUCCESS || framesRead == 0) {
        LOG_ERROR("AudioEngine: Failed to read frames from WAV: error ", result, ", framesRead=", framesRead);
        return false;
    }

    pcmData->resize(framesRead * channels * ma_get_bytes_per_sample(format));

    DecodedWavCacheEntry entry;
    entry.format = format;
    entry.channels = channels;
    entry.sampleRate = sampleRate;
    entry.frames = framesRead;
    entry.pcmData = pcmData;
    gDecodedWavCache.emplace(key, entry);
    out = entry;
    return true;
}

} // namespace

AudioEngine& AudioEngine::instance() {
    static AudioEngine instance;
    return instance;
}

AudioEngine::AudioEngine() = default;

AudioEngine::~AudioEngine() {
    shutdown();
}

bool AudioEngine::initialize() {
    if (initialized_) {
        LOG_WARNING("AudioEngine already initialized");
        return true;
    }

    // Allocate miniaudio engine
    engine_ = new ma_engine();

    // Initialize with default config
    ma_result result = ma_engine_init(nullptr, engine_);
    if (result != MA_SUCCESS) {
        LOG_ERROR("Failed to initialize miniaudio engine: ", result);
        delete engine_;
        engine_ = nullptr;
        return false;
    }

    // Set default master volume
    ma_engine_set_volume(engine_, masterVolume_);

    // Log audio backend info
    ma_backend backend = ma_engine_get_device(engine_)->pContext->backend;
    const char* backendName = "unknown";
    switch (backend) {
        case ma_backend_wasapi: backendName = "WASAPI"; break;
        case ma_backend_dsound: backendName = "DirectSound"; break;
        case ma_backend_winmm: backendName = "WinMM"; break;
        case ma_backend_coreaudio: backendName = "CoreAudio"; break;
        case ma_backend_sndio: backendName = "sndio"; break;
        case ma_backend_audio4: backendName = "audio(4)"; break;
        case ma_backend_oss: backendName = "OSS"; break;
        case ma_backend_pulseaudio: backendName = "PulseAudio"; break;
        case ma_backend_alsa: backendName = "ALSA"; break;
        case ma_backend_jack: backendName = "JACK"; break;
        case ma_backend_aaudio: backendName = "AAudio"; break;
        case ma_backend_opensl: backendName = "OpenSL|ES"; break;
        case ma_backend_webaudio: backendName = "WebAudio"; break;
        case ma_backend_custom: backendName = "Custom"; break;
        case ma_backend_null: backendName = "Null (no output)"; break;
        default: break;
    }

    initialized_ = true;
    LOG_INFO("AudioEngine initialized (miniaudio, backend: ", backendName, ")");
    return true;
}

void AudioEngine::shutdown() {
    if (!initialized_) {
        return;
    }

    // Stop music
    stopMusic();

    // Clean up all active sounds
    for (auto& activeSound : activeSounds_) {
        ma_sound_uninit(activeSound.sound);
        delete activeSound.sound;
        ma_audio_buffer* buffer = static_cast<ma_audio_buffer*>(activeSound.buffer);
        ma_audio_buffer_uninit(buffer);
        delete buffer;
    }
    activeSounds_.clear();

    if (engine_) {
        ma_engine_uninit(engine_);
        delete engine_;
        engine_ = nullptr;
    }

    initialized_ = false;
    LOG_INFO("AudioEngine shutdown");
}

void AudioEngine::setMasterVolume(float volume) {
    masterVolume_ = glm::clamp(volume, 0.0f, 1.0f);
    if (engine_) {
        ma_engine_set_volume(engine_, masterVolume_);
    }
}

void AudioEngine::setListenerPosition(const glm::vec3& position) {
    listenerPosition_ = position;
    if (engine_) {
        ma_engine_listener_set_position(engine_, 0, position.x, position.y, position.z);
    }
}

void AudioEngine::setListenerOrientation(const glm::vec3& forward, const glm::vec3& up) {
    listenerForward_ = forward;
    listenerUp_ = up;
    if (engine_) {
        ma_engine_listener_set_direction(engine_, 0, forward.x, forward.y, forward.z);
        ma_engine_listener_set_world_up(engine_, 0, up.x, up.y, up.z);
    }
}

bool AudioEngine::playSound2D(const std::vector<uint8_t>& wavData, float volume, float pitch) {
    (void)pitch;
    if (!initialized_ || !engine_ || wavData.empty()) return false;

    DecodedWavCacheEntry decoded;
    if (!decodeWavCached(wavData, decoded) || !decoded.pcmData || decoded.frames == 0) {
        return false;
    }

    // Create audio buffer from decoded PCM data (heap allocated to keep alive)
    ma_audio_buffer_config bufferConfig = ma_audio_buffer_config_init(
        decoded.format,
        decoded.channels,
        decoded.frames,
        decoded.pcmData->data(),
        nullptr  // No custom allocator
    );
    bufferConfig.sampleRate = decoded.sampleRate;  // Critical: preserve original sample rate!

    ma_audio_buffer* audioBuffer = new ma_audio_buffer();
    ma_result result = ma_audio_buffer_init(&bufferConfig, audioBuffer);
    if (result != MA_SUCCESS) {
        LOG_WARNING("Failed to create audio buffer: ", result);
        delete audioBuffer;
        return false;
    }

    // Create sound from audio buffer
    ma_sound* sound = new ma_sound();
    result = ma_sound_init_from_data_source(
        engine_,
        audioBuffer,
        MA_SOUND_FLAG_DECODE | MA_SOUND_FLAG_ASYNC | MA_SOUND_FLAG_NO_PITCH | MA_SOUND_FLAG_NO_SPATIALIZATION,
        nullptr,
        sound
    );

    if (result != MA_SUCCESS) {
        LOG_WARNING("Failed to create sound: ", result);
        ma_audio_buffer_uninit(audioBuffer);
        delete audioBuffer;
        delete sound;
        return false;
    }

    // Set volume (pitch not supported with NO_PITCH flag)
    ma_sound_set_volume(sound, volume * masterVolume_);

    // Start playback
    result = ma_sound_start(sound);
    if (result != MA_SUCCESS) {
        LOG_WARNING("Failed to start sound: ", result);
        ma_sound_uninit(sound);
        ma_audio_buffer_uninit(audioBuffer);
        delete audioBuffer;
        delete sound;
        return false;
    }

    // Track this sound for cleanup (decoded PCM shared across plays)
    activeSounds_.push_back({sound, audioBuffer, decoded.pcmData});

    return true;
}

bool AudioEngine::playSound2D(const std::string& mpqPath, float volume, float pitch) {
    // TODO: Load from AssetManager
    // For now, return false (not implemented)
    LOG_WARNING("AudioEngine::playSound2D from MPQ path not yet implemented");
    return false;
}

bool AudioEngine::playSound3D(const std::vector<uint8_t>& wavData, const glm::vec3& position,
                              float volume, float pitch, float maxDistance) {
    if (!initialized_ || !engine_ || wavData.empty()) return false;

    DecodedWavCacheEntry decoded;
    if (!decodeWavCached(wavData, decoded) || !decoded.pcmData || decoded.frames == 0) {
        return false;
    }

    LOG_DEBUG("playSound3D: cached WAV - format:", decoded.format,
              " channels:", decoded.channels, " sampleRate:", decoded.sampleRate,
              " pitch:", pitch);

    // Create audio buffer with correct sample rate
    ma_audio_buffer_config bufferConfig = ma_audio_buffer_config_init(
        decoded.format,
        decoded.channels,
        decoded.frames,
        decoded.pcmData->data(),
        nullptr
    );
    bufferConfig.sampleRate = decoded.sampleRate;  // Critical: preserve original sample rate!

    ma_audio_buffer* audioBuffer = new ma_audio_buffer();
    ma_result result = ma_audio_buffer_init(&bufferConfig, audioBuffer);
    if (result != MA_SUCCESS) {
        delete audioBuffer;
        return false;
    }

    // Create 3D sound (spatialization enabled, pitch enabled)
    ma_sound* sound = new ma_sound();
    result = ma_sound_init_from_data_source(
        engine_,
        audioBuffer,
        MA_SOUND_FLAG_DECODE | MA_SOUND_FLAG_ASYNC,  // Removed NO_PITCH flag
        nullptr,
        sound
    );

    if (result != MA_SUCCESS) {
        LOG_WARNING("playSound3D: Failed to create sound, error: ", result);
        ma_audio_buffer_uninit(audioBuffer);
        delete audioBuffer;
        delete sound;
        return false;
    }

    // Set 3D position and attenuation
    ma_sound_set_position(sound, position.x, position.y, position.z);
    ma_sound_set_volume(sound, volume * masterVolume_);
    ma_sound_set_pitch(sound, pitch);  // Enable pitch variation
    ma_sound_set_attenuation_model(sound, ma_attenuation_model_inverse);
    ma_sound_set_min_gain(sound, 0.0f);
    ma_sound_set_max_gain(sound, 1.0f);
    ma_sound_set_min_distance(sound, 1.0f);
    ma_sound_set_max_distance(sound, maxDistance);
    ma_sound_set_rolloff(sound, 1.0f);

    result = ma_sound_start(sound);
    if (result != MA_SUCCESS) {
        ma_sound_uninit(sound);
        ma_audio_buffer_uninit(audioBuffer);
        delete audioBuffer;
        delete sound;
        return false;
    }

    // Track for cleanup
    activeSounds_.push_back({sound, audioBuffer, decoded.pcmData});

    return true;
}

bool AudioEngine::playSound3D(const std::string& mpqPath, const glm::vec3& position,
                              float volume, float pitch, float maxDistance) {
    // TODO: Implement 3D positional audio
    return playSound2D(mpqPath, volume, pitch);
}

bool AudioEngine::playMusic(const std::vector<uint8_t>& musicData, float volume, bool loop) {
    if (!initialized_ || !engine_ || musicData.empty()) {
        return false;
    }

    LOG_INFO("AudioEngine::playMusic - data size: ", musicData.size(), " bytes, volume: ", volume);

    // Stop any currently playing music
    stopMusic();

    // Keep the music data alive
    musicData_ = musicData;
    musicVolume_ = volume;

    // Create decoder from memory (for streaming MP3/OGG)
    ma_decoder* decoder = new ma_decoder();
    ma_decoder_config decoderConfig = ma_decoder_config_init_default();
    ma_result result = ma_decoder_init_memory(
        musicData_.data(),
        musicData_.size(),
        &decoderConfig,
        decoder
    );

    if (result != MA_SUCCESS) {
        LOG_ERROR("Failed to create music decoder: ", result);
        delete decoder;
        return false;
    }

    LOG_INFO("Decoder created - format: ", decoder->outputFormat,
             ", channels: ", decoder->outputChannels,
             ", sampleRate: ", decoder->outputSampleRate);

    musicDecoder_ = decoder;

    // Create streaming sound from decoder
    musicSound_ = new ma_sound();
    result = ma_sound_init_from_data_source(
        engine_,
        decoder,
        MA_SOUND_FLAG_STREAM | MA_SOUND_FLAG_NO_PITCH | MA_SOUND_FLAG_NO_SPATIALIZATION,
        nullptr,
        musicSound_
    );

    if (result != MA_SUCCESS) {
        LOG_ERROR("Failed to create music sound: ", result);
        ma_decoder_uninit(decoder);
        delete decoder;
        musicDecoder_ = nullptr;
        delete musicSound_;
        musicSound_ = nullptr;
        return false;
    }

    // Set volume and looping
    ma_sound_set_volume(musicSound_, volume * masterVolume_);
    ma_sound_set_looping(musicSound_, loop ? MA_TRUE : MA_FALSE);

    // Start playback
    result = ma_sound_start(musicSound_);
    if (result != MA_SUCCESS) {
        LOG_ERROR("Failed to start music playback: ", result);
        ma_sound_uninit(musicSound_);
        delete musicSound_;
        musicSound_ = nullptr;
        ma_decoder_uninit(decoder);
        delete decoder;
        musicDecoder_ = nullptr;
        return false;
    }

    LOG_INFO("Music playback started successfully - volume: ", volume,
             ", loop: ", loop,
             ", is_playing: ", ma_sound_is_playing(musicSound_));

    return true;
}

void AudioEngine::stopMusic() {
    if (musicSound_) {
        ma_sound_uninit(musicSound_);
        delete musicSound_;
        musicSound_ = nullptr;
    }
    if (musicDecoder_) {
        ma_decoder* decoder = static_cast<ma_decoder*>(musicDecoder_);
        ma_decoder_uninit(decoder);
        delete decoder;
        musicDecoder_ = nullptr;
    }
    musicData_.clear();
}

bool AudioEngine::isMusicPlaying() const {
    if (!musicSound_) {
        return false;
    }
    return ma_sound_is_playing(musicSound_) == MA_TRUE;
}

void AudioEngine::setMusicVolume(float volume) {
    musicVolume_ = glm::clamp(volume, 0.0f, 1.0f);
    if (musicSound_) {
        ma_sound_set_volume(musicSound_, musicVolume_ * masterVolume_);
    }
}

void AudioEngine::update(float deltaTime) {
    (void)deltaTime;

    if (!initialized_ || !engine_) {
        return;
    }

    // Clean up finished sounds
    for (auto it = activeSounds_.begin(); it != activeSounds_.end(); ) {
        if (!ma_sound_is_playing(it->sound)) {
            // Sound finished, clean up
            ma_sound_uninit(it->sound);
            delete it->sound;
            ma_audio_buffer* buffer = static_cast<ma_audio_buffer*>(it->buffer);
            ma_audio_buffer_uninit(buffer);
            delete buffer;
            it = activeSounds_.erase(it);
        } else {
            ++it;
        }
    }
}

} // namespace audio
} // namespace wowee
