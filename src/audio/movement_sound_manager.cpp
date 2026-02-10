#include "audio/movement_sound_manager.hpp"
#include "audio/audio_engine.hpp"
#include "pipeline/asset_manager.hpp"
#include "core/logger.hpp"
#include <random>

namespace wowee {
namespace audio {

namespace {
    std::random_device rd;
    std::mt19937 gen(rd());
}

bool MovementSoundManager::initialize(pipeline::AssetManager* assets) {
    if (!assets) {
        LOG_ERROR("MovementSoundManager: AssetManager is null");
        return false;
    }

    LOG_INFO("MovementSoundManager: Initializing...");

    // Load water splash sounds - entering water
    enterWaterSmallSounds_.resize(1);
    loadSound("Sound\\Spells\\EnterWaterSmall.wav", enterWaterSmallSounds_[0], assets);

    enterWaterMediumSounds_.resize(1);
    loadSound("Sound\\Spells\\EnterWaterMedium.wav", enterWaterMediumSounds_[0], assets);

    enterWaterGiantSounds_.resize(1);
    loadSound("Sound\\Spells\\EnterWaterGiant.wav", enterWaterGiantSounds_[0], assets);

    // Load water footstep sounds - walking in water (5 variations each)
    waterFootstepSmallSounds_.resize(5);
    loadSound("Sound\\Spells\\WaterFootstepSmall1.wav", waterFootstepSmallSounds_[0], assets);
    loadSound("Sound\\Spells\\WaterFootstepSmall2.wav", waterFootstepSmallSounds_[1], assets);
    loadSound("Sound\\Spells\\WaterFootstepSmall3.wav", waterFootstepSmallSounds_[2], assets);
    loadSound("Sound\\Spells\\WaterFootstepSmall4.wav", waterFootstepSmallSounds_[3], assets);
    loadSound("Sound\\Spells\\WaterFootstepSmall5.wav", waterFootstepSmallSounds_[4], assets);

    waterFootstepMediumSounds_.resize(5);
    loadSound("Sound\\Spells\\WaterFootstepMedium1.wav", waterFootstepMediumSounds_[0], assets);
    loadSound("Sound\\Spells\\WaterFootstepMedium2.wav", waterFootstepMediumSounds_[1], assets);
    loadSound("Sound\\Spells\\WaterFootstepMedium3.wav", waterFootstepMediumSounds_[2], assets);
    loadSound("Sound\\Spells\\WaterFootstepMedium4.wav", waterFootstepMediumSounds_[3], assets);
    loadSound("Sound\\Spells\\WaterFootstepMedium5.wav", waterFootstepMediumSounds_[4], assets);

    waterFootstepHugeSounds_.resize(5);
    loadSound("Sound\\Spells\\WaterFootstepHuge1.wav", waterFootstepHugeSounds_[0], assets);
    loadSound("Sound\\Spells\\WaterFootstepHuge2.wav", waterFootstepHugeSounds_[1], assets);
    loadSound("Sound\\Spells\\WaterFootstepHuge3.wav", waterFootstepHugeSounds_[2], assets);
    loadSound("Sound\\Spells\\WaterFootstepHuge4.wav", waterFootstepHugeSounds_[3], assets);
    loadSound("Sound\\Spells\\WaterFootstepHuge5.wav", waterFootstepHugeSounds_[4], assets);

    // Load jump/land vocalizations for all races
    // Human Male
    jumpHumanMaleSounds_.resize(1);
    loadSound("Sound\\Character\\Human\\HumanMaleJump1.wav", jumpHumanMaleSounds_[0], assets);
    landHumanMaleSounds_.resize(1);
    loadSound("Sound\\Character\\Human\\HumanMaleLand1.wav", landHumanMaleSounds_[0], assets);

    // Human Female
    jumpHumanFemaleSounds_.resize(1);
    loadSound("Sound\\Character\\Human\\HumanFemaleJump1.wav", jumpHumanFemaleSounds_[0], assets);
    landHumanFemaleSounds_.resize(1);
    loadSound("Sound\\Character\\Human\\HumanFemaleLand1.wav", landHumanFemaleSounds_[0], assets);

    // Dwarf Male
    jumpDwarfMaleSounds_.resize(1);
    loadSound("Sound\\Character\\Dwarf\\DwarfMaleJump1.wav", jumpDwarfMaleSounds_[0], assets);
    landDwarfMaleSounds_.resize(1);
    loadSound("Sound\\Character\\Dwarf\\DwarfMaleLand1.wav", landDwarfMaleSounds_[0], assets);

    // Dwarf Female
    jumpDwarfFemaleSounds_.resize(1);
    loadSound("Sound\\Character\\Dwarf\\DwarfFemaleJump1.wav", jumpDwarfFemaleSounds_[0], assets);
    landDwarfFemaleSounds_.resize(1);
    loadSound("Sound\\Character\\Dwarf\\DwarfFemaleLand1.wav", landDwarfFemaleSounds_[0], assets);

    // Night Elf Male
    jumpNightElfMaleSounds_.resize(1);
    loadSound("Sound\\Character\\NightElf\\NightElfMaleJump1.wav", jumpNightElfMaleSounds_[0], assets);
    landNightElfMaleSounds_.resize(1);
    loadSound("Sound\\Character\\NightElf\\NightElfMaleLand1.wav", landNightElfMaleSounds_[0], assets);

    // Night Elf Female
    jumpNightElfFemaleSounds_.resize(1);
    loadSound("Sound\\Character\\NightElf\\NightElfFemaleJump1.wav", jumpNightElfFemaleSounds_[0], assets);
    landNightElfFemaleSounds_.resize(1);
    loadSound("Sound\\Character\\NightElf\\NightElfFemaleLand1.wav", landNightElfFemaleSounds_[0], assets);

    // Orc Male
    jumpOrcMaleSounds_.resize(1);
    loadSound("Sound\\Character\\Orc\\OrcMaleJump1.wav", jumpOrcMaleSounds_[0], assets);
    landOrcMaleSounds_.resize(1);
    loadSound("Sound\\Character\\Orc\\OrcMaleLand1.wav", landOrcMaleSounds_[0], assets);

    // Orc Female
    jumpOrcFemaleSounds_.resize(1);
    loadSound("Sound\\Character\\Orc\\OrcFemaleJump1.wav", jumpOrcFemaleSounds_[0], assets);
    landOrcFemaleSounds_.resize(1);
    loadSound("Sound\\Character\\Orc\\OrcFemaleLand1.wav", landOrcFemaleSounds_[0], assets);

    // Tauren Male
    jumpTaurenMaleSounds_.resize(1);
    loadSound("Sound\\Character\\Tauren\\TaurenMaleJump1.wav", jumpTaurenMaleSounds_[0], assets);
    landTaurenMaleSounds_.resize(1);
    loadSound("Sound\\Character\\Tauren\\TaurenMaleLand1.wav", landTaurenMaleSounds_[0], assets);

    // Tauren Female
    jumpTaurenFemaleSounds_.resize(1);
    loadSound("Sound\\Character\\Tauren\\TaurenFemaleJump1.wav", jumpTaurenFemaleSounds_[0], assets);
    landTaurenFemaleSounds_.resize(1);
    loadSound("Sound\\Character\\Tauren\\TaurenFemaleLand1.wav", landTaurenFemaleSounds_[0], assets);

    // Troll Male
    jumpTrollMaleSounds_.resize(1);
    loadSound("Sound\\Character\\Troll\\TrollMaleJump1.wav", jumpTrollMaleSounds_[0], assets);
    landTrollMaleSounds_.resize(1);
    loadSound("Sound\\Character\\Troll\\TrollMaleLand1.wav", landTrollMaleSounds_[0], assets);

    // Troll Female
    jumpTrollFemaleSounds_.resize(1);
    loadSound("Sound\\Character\\Troll\\TrollFemaleJump1.wav", jumpTrollFemaleSounds_[0], assets);
    landTrollFemaleSounds_.resize(1);
    loadSound("Sound\\Character\\Troll\\TrollFemaleLand1.wav", landTrollFemaleSounds_[0], assets);

    // Undead Male
    jumpUndeadMaleSounds_.resize(1);
    loadSound("Sound\\Character\\Scourge\\ScourgeMaleJump1.wav", jumpUndeadMaleSounds_[0], assets);
    landUndeadMaleSounds_.resize(1);
    loadSound("Sound\\Character\\Scourge\\ScourgeMaleLand1.wav", landUndeadMaleSounds_[0], assets);

    // Undead Female
    jumpUndeadFemaleSounds_.resize(1);
    loadSound("Sound\\Character\\Scourge\\ScourgeFemaleJump1.wav", jumpUndeadFemaleSounds_[0], assets);
    landUndeadFemaleSounds_.resize(1);
    loadSound("Sound\\Character\\Scourge\\ScourgeFemaleLand1.wav", landUndeadFemaleSounds_[0], assets);

    // Gnome Male
    jumpGnomeMaleSounds_.resize(1);
    loadSound("Sound\\Character\\Gnome\\GnomeMaleJump1.wav", jumpGnomeMaleSounds_[0], assets);
    landGnomeMaleSounds_.resize(1);
    loadSound("Sound\\Character\\Gnome\\GnomeMaleLand1.wav", landGnomeMaleSounds_[0], assets);

    // Gnome Female
    jumpGnomeFemaleSounds_.resize(1);
    loadSound("Sound\\Character\\Gnome\\GnomeFemaleJump1.wav", jumpGnomeFemaleSounds_[0], assets);
    landGnomeFemaleSounds_.resize(1);
    loadSound("Sound\\Character\\Gnome\\GnomeFemaleLand1.wav", landGnomeFemaleSounds_[0], assets);

    LOG_INFO("MovementSoundManager: Water sounds - Enter small: ", enterWaterSmallSounds_[0].loaded ? "YES" : "NO",
             ", Enter medium: ", enterWaterMediumSounds_[0].loaded ? "YES" : "NO",
             ", Enter giant: ", enterWaterGiantSounds_[0].loaded ? "YES" : "NO");
    LOG_INFO("MovementSoundManager: Jump/Land - Human: ", jumpHumanMaleSounds_[0].loaded ? "YES" : "NO",
             ", Orc: ", jumpOrcMaleSounds_[0].loaded ? "YES" : "NO",
             ", Tauren: ", jumpTaurenMaleSounds_[0].loaded ? "YES" : "NO");

    initialized_ = true;
    LOG_INFO("MovementSoundManager: Initialization complete");
    return true;
}

void MovementSoundManager::shutdown() {
    initialized_ = false;
}

bool MovementSoundManager::loadSound(const std::string& path, MovementSample& sample, pipeline::AssetManager* assets) {
    sample.path = path;
    sample.loaded = false;

    try {
        sample.data = assets->readFile(path);
        if (!sample.data.empty()) {
            sample.loaded = true;
            return true;
        }
    } catch (const std::exception& e) {
        // Silently fail - not all sounds may exist
    }

    return false;
}

void MovementSoundManager::playSound(const std::vector<MovementSample>& library, float volumeMultiplier) {
    if (!initialized_ || library.empty() || !library[0].loaded) return;

    float volume = 0.7f * volumeScale_ * volumeMultiplier;
    AudioEngine::instance().playSound2D(library[0].data, volume, 1.0f);
}

void MovementSoundManager::playRandomSound(const std::vector<MovementSample>& library, float volumeMultiplier) {
    if (!initialized_ || library.empty()) return;

    // Count loaded sounds
    std::vector<const MovementSample*> loadedSounds;
    for (const auto& sample : library) {
        if (sample.loaded) {
            loadedSounds.push_back(&sample);
        }
    }

    if (loadedSounds.empty()) return;

    // Pick random sound
    std::uniform_int_distribution<size_t> dist(0, loadedSounds.size() - 1);
    size_t index = dist(gen);

    float volume = 0.7f * volumeScale_ * volumeMultiplier;
    AudioEngine::instance().playSound2D(loadedSounds[index]->data, volume, 1.0f);
}

void MovementSoundManager::setVolumeScale(float scale) {
    volumeScale_ = std::max(0.0f, std::min(1.0f, scale));
}

void MovementSoundManager::playEnterWater(CharacterSize size) {
    switch (size) {
        case CharacterSize::SMALL:
            playSound(enterWaterSmallSounds_, 0.8f);
            break;
        case CharacterSize::MEDIUM:
            playSound(enterWaterMediumSounds_, 1.0f);
            break;
        case CharacterSize::LARGE:
        case CharacterSize::GIANT:
            playSound(enterWaterGiantSounds_, 1.2f);
            break;
    }
}

void MovementSoundManager::playWaterFootstep(CharacterSize size) {
    switch (size) {
        case CharacterSize::SMALL:
            playRandomSound(waterFootstepSmallSounds_, 0.6f);
            break;
        case CharacterSize::MEDIUM:
            playRandomSound(waterFootstepMediumSounds_, 0.8f);
            break;
        case CharacterSize::LARGE:
        case CharacterSize::GIANT:
            playRandomSound(waterFootstepHugeSounds_, 1.0f);
            break;
    }
}

void MovementSoundManager::playJump(PlayerRace race) {
    switch (race) {
        case PlayerRace::HUMAN_MALE:
            playSound(jumpHumanMaleSounds_);
            break;
        case PlayerRace::HUMAN_FEMALE:
            playSound(jumpHumanFemaleSounds_);
            break;
        case PlayerRace::DWARF_MALE:
            playSound(jumpDwarfMaleSounds_);
            break;
        case PlayerRace::DWARF_FEMALE:
            playSound(jumpDwarfFemaleSounds_);
            break;
        case PlayerRace::NIGHT_ELF_MALE:
            playSound(jumpNightElfMaleSounds_);
            break;
        case PlayerRace::NIGHT_ELF_FEMALE:
            playSound(jumpNightElfFemaleSounds_);
            break;
        case PlayerRace::ORC_MALE:
            playSound(jumpOrcMaleSounds_);
            break;
        case PlayerRace::ORC_FEMALE:
            playSound(jumpOrcFemaleSounds_);
            break;
        case PlayerRace::TAUREN_MALE:
            playSound(jumpTaurenMaleSounds_);
            break;
        case PlayerRace::TAUREN_FEMALE:
            playSound(jumpTaurenFemaleSounds_);
            break;
        case PlayerRace::TROLL_MALE:
            playSound(jumpTrollMaleSounds_);
            break;
        case PlayerRace::TROLL_FEMALE:
            playSound(jumpTrollFemaleSounds_);
            break;
        case PlayerRace::UNDEAD_MALE:
            playSound(jumpUndeadMaleSounds_);
            break;
        case PlayerRace::UNDEAD_FEMALE:
            playSound(jumpUndeadFemaleSounds_);
            break;
        case PlayerRace::GNOME_MALE:
            playSound(jumpGnomeMaleSounds_);
            break;
        case PlayerRace::GNOME_FEMALE:
            playSound(jumpGnomeFemaleSounds_);
            break;
    }
}

void MovementSoundManager::playLand(PlayerRace race) {
    switch (race) {
        case PlayerRace::HUMAN_MALE:
            playSound(landHumanMaleSounds_);
            break;
        case PlayerRace::HUMAN_FEMALE:
            playSound(landHumanFemaleSounds_);
            break;
        case PlayerRace::DWARF_MALE:
            playSound(landDwarfMaleSounds_);
            break;
        case PlayerRace::DWARF_FEMALE:
            playSound(landDwarfFemaleSounds_);
            break;
        case PlayerRace::NIGHT_ELF_MALE:
            playSound(landNightElfMaleSounds_);
            break;
        case PlayerRace::NIGHT_ELF_FEMALE:
            playSound(landNightElfFemaleSounds_);
            break;
        case PlayerRace::ORC_MALE:
            playSound(landOrcMaleSounds_);
            break;
        case PlayerRace::ORC_FEMALE:
            playSound(landOrcFemaleSounds_);
            break;
        case PlayerRace::TAUREN_MALE:
            playSound(landTaurenMaleSounds_);
            break;
        case PlayerRace::TAUREN_FEMALE:
            playSound(landTaurenFemaleSounds_);
            break;
        case PlayerRace::TROLL_MALE:
            playSound(landTrollMaleSounds_);
            break;
        case PlayerRace::TROLL_FEMALE:
            playSound(landTrollFemaleSounds_);
            break;
        case PlayerRace::UNDEAD_MALE:
            playSound(landUndeadMaleSounds_);
            break;
        case PlayerRace::UNDEAD_FEMALE:
            playSound(landUndeadFemaleSounds_);
            break;
        case PlayerRace::GNOME_MALE:
            playSound(landGnomeMaleSounds_);
            break;
        case PlayerRace::GNOME_FEMALE:
            playSound(landGnomeFemaleSounds_);
            break;
    }
}

} // namespace audio
} // namespace wowee
