#include "audio/audio_coordinator.hpp"
#include "audio/audio_engine.hpp"
#include "audio/music_manager.hpp"
#include "audio/footstep_manager.hpp"
#include "audio/activity_sound_manager.hpp"
#include "audio/mount_sound_manager.hpp"
#include "audio/npc_voice_manager.hpp"
#include "audio/player_voice_manager.hpp"
#include "audio/ambient_sound_manager.hpp"
#include "audio/ui_sound_manager.hpp"
#include "audio/combat_sound_manager.hpp"
#include "audio/spell_sound_manager.hpp"
#include "audio/movement_sound_manager.hpp"
#include "pipeline/asset_manager.hpp"
#include "game/zone_manager.hpp"
#include "core/logger.hpp"

namespace wowee {
namespace audio {

AudioCoordinator::AudioCoordinator() = default;

AudioCoordinator::~AudioCoordinator() {
    shutdown();
}

bool AudioCoordinator::initialize() {
    // Initialize AudioEngine (singleton)
    if (!AudioEngine::instance().initialize()) {
        LOG_WARNING("Failed to initialize AudioEngine - audio will be disabled");
        audioAvailable_ = false;
        return false;
    }
    audioAvailable_ = true;

    // Create all audio managers (initialized later with asset manager)
    musicManager_ = std::make_unique<MusicManager>();
    footstepManager_ = std::make_unique<FootstepManager>();
    activitySoundManager_ = std::make_unique<ActivitySoundManager>();
    mountSoundManager_ = std::make_unique<MountSoundManager>();
    npcVoiceManager_ = std::make_unique<NpcVoiceManager>();
    playerVoiceManager_ = std::make_unique<PlayerVoiceManager>();
    ambientSoundManager_ = std::make_unique<AmbientSoundManager>();
    uiSoundManager_ = std::make_unique<UiSoundManager>();
    combatSoundManager_ = std::make_unique<CombatSoundManager>();
    spellSoundManager_ = std::make_unique<SpellSoundManager>();
    movementSoundManager_ = std::make_unique<MovementSoundManager>();

    LOG_INFO("AudioCoordinator initialized with ", 11, " audio managers");
    return true;
}
void AudioCoordinator::shutdown() {
    // Reset all managers first (they may reference AudioEngine)
    movementSoundManager_.reset();
    spellSoundManager_.reset();
    combatSoundManager_.reset();
    uiSoundManager_.reset();
    ambientSoundManager_.reset();
    playerVoiceManager_.reset();
    npcVoiceManager_.reset();
    mountSoundManager_.reset();
    activitySoundManager_.reset();
    footstepManager_.reset();
    musicManager_.reset();

    // Shutdown audio engine last
    if (audioAvailable_) {
        AudioEngine::instance().shutdown();
        audioAvailable_ = false;
    }

    LOG_INFO("AudioCoordinator shutdown complete");
}

void AudioCoordinator::playZoneMusic(const std::string& music) {
    if (music.empty() || !musicManager_) return;
    if (music.rfind("file:", 0) == 0) {
        musicManager_->crossfadeToFile(music.substr(5));
    } else {
        musicManager_->crossfadeTo(music);
    }
}

void AudioCoordinator::setZoneMusicLooping(bool loop) {
    if (musicManager_) musicManager_->setLooping(loop);
}

void AudioCoordinator::onOriginalSoundtrackDisabled(game::ZoneManager* zm) {
    if (!zm || !musicManager_) return;
    if (!musicManager_->isCurrentTrackFile()) return;
    if (!musicManager_->isPlaying() && !musicManager_->isLoading()) return;
    // Only act in-world with a known zone; the login screen intentionally
    // plays a file track and is not part of the zone rotation this setting
    // controls.
    if (currentZoneId_ == 0) return;

    std::string music = zm->getRandomMusic(currentZoneId_);
    if (!music.empty() && music.rfind("file:", 0) != 0) {
        playZoneMusic(music);
        musicSwitchCooldown_ = 6.0f;
    } else {
        musicManager_->stopMusic();
    }
}

void AudioCoordinator::updateZoneAudio(const ZoneAudioContext& ctx) {
    float deltaTime = ctx.deltaTime;
    if (musicSwitchCooldown_ > 0.0f) {
        musicSwitchCooldown_ = std::max(0.0f, musicSwitchCooldown_ - deltaTime);
    }

    // Resolve the spatial zone before updating ambience. Zone ambience used to
    // run first, leaving it one zone behind and permanently on its noon default.
    auto* zm = ctx.zoneManager;
    const uint32_t tileZoneId = (zm && ctx.hasTile)
        ? zm->getZoneId(ctx.tileX, ctx.tileY)
        : 0;
    const uint32_t serverZoneId = (zm && ctx.serverZoneId != 0)
        ? zm->resolveAreaZoneId(ctx.serverZoneId)
        : ctx.serverZoneId;
    uint32_t zoneId = serverZoneId != 0 ? serverZoneId : tileZoneId;

    // ── Ambient weather audio sync ──
    if (ambientSoundManager_) {
        bool isBlacksmith = (ctx.insideWmoId == 96048);

        if (zoneId != 0) {
            ambientSoundManager_->setZoneId(zoneId);
        }
        ambientSoundManager_->setGameTime(ctx.gameTimeHours);

        // Map visual weather type to ambient sound weather type
        AmbientSoundManager::WeatherType audioWeatherType = AmbientSoundManager::WeatherType::NONE;
        if (ctx.weatherType == 1) { // RAIN
            if (ctx.weatherIntensity < 0.33f)      audioWeatherType = AmbientSoundManager::WeatherType::RAIN_LIGHT;
            else if (ctx.weatherIntensity < 0.66f)  audioWeatherType = AmbientSoundManager::WeatherType::RAIN_MEDIUM;
            else                                     audioWeatherType = AmbientSoundManager::WeatherType::RAIN_HEAVY;
        } else if (ctx.weatherType == 2) { // SNOW
            if (ctx.weatherIntensity < 0.33f)      audioWeatherType = AmbientSoundManager::WeatherType::SNOW_LIGHT;
            else if (ctx.weatherIntensity < 0.66f)  audioWeatherType = AmbientSoundManager::WeatherType::SNOW_MEDIUM;
            else                                     audioWeatherType = AmbientSoundManager::WeatherType::SNOW_HEAVY;
        }
        ambientSoundManager_->setWeather(audioWeatherType);
        ambientSoundManager_->update(deltaTime, ctx.cameraPosition, ctx.insideWmo, ctx.isSwimming, isBlacksmith);
    }

    // ── Zone detection and music transitions ──
    if (!zm || !musicManager_ || !ctx.hasTile) return;

    bool insideTavern = false;
    bool insideBlacksmith = false;
    std::string tavernMusic;

    // WMO-based location overrides (taverns, blacksmiths, city zones)
    if (ctx.insideWmo) {
        uint32_t wmoModelId = ctx.insideWmoId;

        // Stormwind WMO → force Stormwind City zone
        if (wmoModelId == 10047) zoneId = 1519;

        // Log WMO transitions
        static uint32_t lastLoggedWmoId = 0;
        if (wmoModelId != lastLoggedWmoId) {
            LOG_INFO("Inside WMO model ID: ", wmoModelId);
            lastLoggedWmoId = wmoModelId;
        }

        // Blacksmith detection (ambient forge sounds)
        if (wmoModelId == 96048) {
            insideBlacksmith = true;
            LOG_INFO("Detected blacksmith WMO ", wmoModelId);
        }

        // Tavern / inn detection
        if (wmoModelId == 191 || wmoModelId == 71414 || wmoModelId == 190 ||
            wmoModelId == 220 || wmoModelId == 221 ||
            wmoModelId == 5392 || wmoModelId == 5393) {
            insideTavern = true;
            // The first slot is our own tavern track. It falls back to the
            // Blizzard one it replaces when the file is missing, and the
            // original-soundtrack setting drops it the same way zone music does.
            static const std::string tavernRemix =
                game::ZoneManager::resolveOriginalMusicFile("TavernAllianceREMIX.mp3");
            const bool useRemix = !tavernRemix.empty() &&
                (!ctx.zoneManager || ctx.zoneManager->getUseOriginalSoundtrack());
            const std::vector<std::string> tavernTracks = {
                useRemix ? tavernRemix
                         : std::string("Sound\\Music\\ZoneMusic\\TavernAlliance\\TavernAlliance01.mp3"),
                "Sound\\Music\\ZoneMusic\\TavernAlliance\\TavernAlliance02.mp3",
                "Sound\\Music\\ZoneMusic\\TavernHuman\\RA_HumanTavern1A.mp3",
                "Sound\\Music\\ZoneMusic\\TavernHuman\\RA_HumanTavern2A.mp3",
            };
            static int tavernTrackIndex = 0;
            tavernMusic = tavernTracks[tavernTrackIndex++ % tavernTracks.size()];
            LOG_INFO("Detected tavern WMO ", wmoModelId, ", playing: ", tavernMusic);
        }
    }

    // Tavern music transitions
    if (insideTavern) {
        if (!inTavern_ && !tavernMusic.empty()) {
            inTavern_ = true;
            LOG_INFO("Entered tavern");
            musicManager_->playMusic(tavernMusic, true);
            musicSwitchCooldown_ = 6.0f;
        }
    } else if (inTavern_) {
        inTavern_ = false;
        LOG_INFO("Exited tavern");
        auto* info = zm->getZoneInfo(currentZoneId_);
        if (info) {
            std::string music = zm->getRandomMusic(currentZoneId_);
            if (!music.empty()) {
                playZoneMusic(music);
                musicSwitchCooldown_ = 6.0f;
            }
        }
    }

    // Blacksmith transitions (stop music, let ambience play)
    if (insideBlacksmith) {
        if (!inBlacksmith_) {
            inBlacksmith_ = true;
            LOG_INFO("Entered blacksmith - stopping music");
            musicManager_->stopMusic();
        }
    } else if (inBlacksmith_) {
        inBlacksmith_ = false;
        LOG_INFO("Exited blacksmith - restoring music");
        auto* info = zm->getZoneInfo(currentZoneId_);
        if (info) {
            std::string music = zm->getRandomMusic(currentZoneId_);
            if (!music.empty()) {
                playZoneMusic(music);
                musicSwitchCooldown_ = 6.0f;
            }
        }
    }

    // Normal zone transitions
    if (!insideTavern && !insideBlacksmith && zoneId != currentZoneId_ && zoneId != 0) {
        currentZoneId_ = zoneId;
        auto* info = zm->getZoneInfo(zoneId);
        if (info) {
            currentZoneName_ = info->name;
            LOG_INFO("Entered zone: ", info->name);
            if (musicSwitchCooldown_ <= 0.0f) {
                std::string music = zm->getRandomMusic(zoneId);
                if (!music.empty()) {
                    playZoneMusic(music);
                    musicSwitchCooldown_ = 6.0f;
                }
            }
        }
    }

    musicManager_->update(deltaTime);

    // When a track finishes, pick a new random track from the current zone
    if (!musicManager_->isPlaying() && !inTavern_ && !inBlacksmith_ &&
        currentZoneId_ != 0 && musicSwitchCooldown_ <= 0.0f) {
        std::string music = zm->getRandomMusic(currentZoneId_);
        if (!music.empty()) {
            playZoneMusic(music);
            musicSwitchCooldown_ = 2.0f;
        }
    }
}

} // namespace audio
} // namespace wowee
