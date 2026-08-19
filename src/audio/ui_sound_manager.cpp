#include "audio/ui_sound_manager.hpp"
#include "audio/sample_load.hpp"
#include "audio/audio_engine.hpp"
#include "pipeline/asset_manager.hpp"
#include "core/logger.hpp"

namespace wowee {
namespace audio {

bool UiSoundManager::initialize(pipeline::AssetManager* assets) {
    if (!assets) {
        LOG_ERROR("UISoundManager: AssetManager is null");
        return false;
    }

    assets_ = assets;
    LOG_INFO("UISoundManager: Initializing...");

    // Load window sounds
    bagOpenSounds_.resize(1);
    bool bagOpenLoaded = loadSound("Sound\\Interface\\iBackPackOpen.wav", bagOpenSounds_[0], assets);

    bagCloseSounds_.resize(1);
    bool bagCloseLoaded = loadSound("Sound\\Interface\\iBackPackClose.wav", bagCloseSounds_[0], assets);

    questLogOpenSounds_.resize(1);
    bool questLogOpenLoaded = loadSound("Sound\\Interface\\iQuestLogOpenA.wav", questLogOpenSounds_[0], assets);

    questLogCloseSounds_.resize(1);
    bool questLogCloseLoaded = loadSound("Sound\\Interface\\iQuestLogCloseA.wav", questLogCloseSounds_[0], assets);

    characterSheetOpenSounds_.resize(1);
    bool charSheetOpenLoaded = loadSound("Sound\\Interface\\iAbilitiesOpenA.wav", characterSheetOpenSounds_[0], assets);

    characterSheetCloseSounds_.resize(1);
    bool charSheetCloseLoaded = loadSound("Sound\\Interface\\iAbilitiesCloseA.wav", characterSheetCloseSounds_[0], assets);

    auctionOpenSounds_.resize(1);
    loadSound("Sound\\Interface\\AuctionWindowOpen.wav", auctionOpenSounds_[0], assets);

    auctionCloseSounds_.resize(1);
    loadSound("Sound\\Interface\\AuctionWindowClose.wav", auctionCloseSounds_[0], assets);

    guildBankOpenSounds_.resize(1);
    loadSound("Sound\\Interface\\GuildVaultOpen.wav", guildBankOpenSounds_[0], assets);

    guildBankCloseSounds_.resize(1);
    loadSound("Sound\\Interface\\GuildVaultClose.wav", guildBankCloseSounds_[0], assets);

    // Load button sounds
    buttonClickSounds_.resize(1);
    bool buttonClickLoaded = loadSound("Sound\\Interface\\iUiInterfaceButtonA.wav", buttonClickSounds_[0], assets);

    menuButtonSounds_.resize(1);
    bool menuButtonLoaded = loadSound("Sound\\Interface\\iUIMainMenuButtonA.wav", menuButtonSounds_[0], assets);

    // Load quest sounds
    questActivateSounds_.resize(1);
    bool questActivateLoaded = loadSound("Sound\\Interface\\iQuestActivate.wav", questActivateSounds_[0], assets);

    questCompleteSounds_.resize(1);
    bool questCompleteLoaded = loadSound("Sound\\Interface\\iQuestComplete.wav", questCompleteSounds_[0], assets);

    questFailedSounds_.resize(1);
    bool questFailedLoaded = loadSound("Sound\\Interface\\igQuestFailed.wav", questFailedSounds_[0], assets);

    questUpdateSounds_.resize(1);
    loadSound("Sound\\Interface\\iQuestUpdate.wav", questUpdateSounds_[0], assets);

    fishingBiteSounds_.resize(3);
    loadSound("Sound\\Spell\\FishingBobber_Ver2_1.wav", fishingBiteSounds_[0], assets);
    loadSound("Sound\\Spell\\FishingBobber_Ver2_2.wav", fishingBiteSounds_[1], assets);
    loadSound("Sound\\Spell\\FishingBobber_Ver2_3.wav", fishingBiteSounds_[2], assets);

    // Load loot sounds
    lootCoinSmallSounds_.resize(1);
    bool lootCoinSmallLoaded = loadSound("Sound\\Interface\\LootCoinSmall.wav", lootCoinSmallSounds_[0], assets);

    lootCoinLargeSounds_.resize(1);
    bool lootCoinLargeLoaded = loadSound("Sound\\Interface\\LootCoinLarge.wav", lootCoinLargeSounds_[0], assets);

    lootItemSounds_.resize(1);
    bool lootItemLoaded = loadSound("Sound\\Interface\\igLootCreature.wav", lootItemSounds_[0], assets);

    // Load item pickup sounds
    dropSounds_.resize(1);
    bool dropLoaded = loadSound("Sound\\Interface\\DropOnGround.wav", dropSounds_[0], assets);

    pickupBagSounds_.resize(1);
    bool pickupBagLoaded = loadSound("Sound\\Interface\\PickUp\\PickUpBag.wav", pickupBagSounds_[0], assets);

    pickupBookSounds_.resize(1);
    bool pickupBookLoaded = loadSound("Sound\\Interface\\PickUp\\PickUpBook.wav", pickupBookSounds_[0], assets);




    // Load eating/drinking sounds
    eatingSounds_.resize(1);
    bool eatingLoaded = loadSound("Sound\\Interface\\iEating1.wav", eatingSounds_[0], assets);

    drinkingSounds_.resize(1);
    bool drinkingLoaded = loadSound("Sound\\Interface\\iDrinking1.wav", drinkingSounds_[0], assets);

    // Load level up sound
    levelUpSounds_.resize(1);
    bool levelUpLoaded = loadSound("Sound\\Interface\\LevelUp.wav", levelUpSounds_[0], assets);

    // Load achievement sound (WotLK: Sound\Interface\AchievementSound.wav)
    achievementSounds_.resize(1);
    if (!loadSound("Sound\\Interface\\AchievementSound.wav", achievementSounds_[0], assets)) {
        // Fallback to level-up sound if achievement sound is missing
        achievementSounds_ = levelUpSounds_;
    }

    // Load error/feedback sounds
    errorSounds_.resize(1);
    loadSound("Sound\\Interface\\Error.wav", errorSounds_[0], assets);

    selectTargetSounds_.resize(1);
    loadSound("Sound\\Interface\\iSelectTarget.wav", selectTargetSounds_[0], assets);

    deselectTargetSounds_.resize(1);
    loadSound("Sound\\Interface\\iDeselectTarget.wav", deselectTargetSounds_[0], assets);

    // Whisper notification (falls back to iSelectTarget if the dedicated file is absent)
    whisperSounds_.resize(1);
    if (!loadSound("Sound\\Interface\\Whisper_TellMale.wav", whisperSounds_[0], assets)) {
        if (!loadSound("Sound\\Interface\\Whisper_TellFemale.wav", whisperSounds_[0], assets)) {
            whisperSounds_ = selectTargetSounds_;
        }
    }

    // New-mail notification (dedicated cue, falling back to the whisper sound)
    mailSounds_.resize(1);
    if (!loadSound("Sound\\Interface\\iTellMessage.wav", mailSounds_[0], assets)) {
        mailSounds_ = whisperSounds_;
    }

    // Minimap ping sound
    minimapPingSounds_.resize(1);
    if (!loadSound("Sound\\Interface\\MapPing.wav", minimapPingSounds_[0], assets)) {
        minimapPingSounds_ = selectTargetSounds_;  // fallback to target select sound
    }

    LOG_INFO("UISoundManager: Window sounds - Bag: ", (bagOpenLoaded && bagCloseLoaded) ? "YES" : "NO",
             ", QuestLog: ", (questLogOpenLoaded && questLogCloseLoaded) ? "YES" : "NO",
             ", CharSheet: ", (charSheetOpenLoaded && charSheetCloseLoaded) ? "YES" : "NO");
    LOG_INFO("UISoundManager: Button sounds - Click: ", buttonClickLoaded ? "YES" : "NO",
             ", Menu: ", menuButtonLoaded ? "YES" : "NO");
    LOG_INFO("UISoundManager: Quest sounds - Activate: ", questActivateLoaded ? "YES" : "NO",
             ", Complete: ", questCompleteLoaded ? "YES" : "NO",
             ", Failed: ", questFailedLoaded ? "YES" : "NO");
    LOG_INFO("UISoundManager: Loot sounds - Coins: ", (lootCoinSmallLoaded && lootCoinLargeLoaded) ? "YES" : "NO",
             ", Items: ", lootItemLoaded ? "YES" : "NO");
    LOG_INFO("UISoundManager: Item sounds - Pickup: ", (pickupBagLoaded && pickupBookLoaded) ? "YES" : "NO",
             ", Drop: ", dropLoaded ? "YES" : "NO");
    LOG_INFO("UISoundManager: Misc sounds - Eating: ", eatingLoaded ? "YES" : "NO",
             ", Drinking: ", drinkingLoaded ? "YES" : "NO",
             ", LevelUp: ", levelUpLoaded ? "YES" : "NO");

    initialized_ = true;
    LOG_INFO("UISoundManager: Initialization complete");
    return true;
}

void UiSoundManager::shutdown() {
    initialized_ = false;
}

bool UiSoundManager::loadSound(const std::string& path, UISample& sample, pipeline::AssetManager* assets) {
    return loadSampleFile(path, sample, assets, "UISoundManager");
}

void UiSoundManager::ensureSoundEntriesLoaded() {
    if (soundEntriesBuilt_) return;
    soundEntriesBuilt_ = true;          // once, whether or not it works
    if (!assets_) return;

    auto dbc = assets_->loadDBC("SoundEntries.dbc");
    if (!dbc || !dbc->isLoaded()) {
        LOG_WARNING("UISoundManager: SoundEntries.dbc not available; "
                    "PlaySound falls back to the names mapped by hand");
        return;
    }
    // 3.3.5a layout: 0 ID, 1 SoundType, 2 Name, 3..12 File[0..9],
    // 13..22 Freq[0..9], 23 DirectoryBase. The same reading zone_manager and
    // npc_voice_manager already make of this table.
    if (dbc->getFieldCount() < 24) {
        LOG_WARNING("UISoundManager: SoundEntries.dbc has ", dbc->getFieldCount(),
                    " fields, expected at least 24");
        return;
    }
    for (uint32_t row = 0; row < dbc->getRecordCount(); ++row) {
        std::string name = dbc->getString(row, 2);
        if (name.empty()) continue;
        for (char& ch : name) {
            ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
        }
        const std::string dir = dbc->getString(row, 23);
        std::vector<std::string> paths;
        for (uint32_t f = 3; f <= 12; ++f) {
            const std::string file = dbc->getString(row, f);
            if (file.empty()) continue;
            paths.push_back(dir.empty() ? file : dir + "\\" + file);
        }
        // The first row wins. Names repeat in this table and the later rows
        // are variants; a UI click wants one sound, not a different one each
        // time.
        if (!paths.empty()) soundPathsByName_.emplace(name, std::move(paths));
    }
    LOG_INFO("UISoundManager: ", soundPathsByName_.size(),
             " sound names from SoundEntries.dbc");
}

bool UiSoundManager::playByName(const std::string& soundName) {
    if (!initialized_ || soundName.empty()) return false;
    ensureSoundEntriesLoaded();

    std::string key = soundName;
    for (char& ch : key) {
        ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    }

    auto cached = namedSamples_.find(key);
    if (cached == namedSamples_.end()) {
        auto it = soundPathsByName_.find(key);
        if (it == soundPathsByName_.end()) return false;
        UISample sample;
        sample.loaded = false;
        for (const std::string& path : it->second) {
            if (loadSound(path, sample, assets_)) break;
        }
        // Remembered even when nothing loaded, so a name whose file this
        // install does not have is looked for once rather than on every click.
        cached = namedSamples_.emplace(key, std::move(sample)).first;
    }
    if (!cached->second.loaded) return false;

    // What happened, not what was attempted. playSound2D decodes WAV and
    // answers false for anything else, and SoundEntries lists a couple of
    // thousand mp3s among its wavs - claiming success for one of those would
    // skip the fallback below this and play nothing at all.
    return AudioEngine::instance().playSound2D(cached->second.data,
                                               0.7f * volumeScale_, 1.0f);
}

bool UiSoundManager::playFile(const std::string& path) {
    if (!initialized_ || path.empty() || !assets_) return false;

    auto cached = namedSamples_.find(path);
    if (cached == namedSamples_.end()) {
        UISample sample;
        sample.loaded = false;
        loadSound(path, sample, assets_);
        // Remembered either way, so a path this install does not have is read
        // for once rather than on every call.
        cached = namedSamples_.emplace(path, std::move(sample)).first;
    }
    if (!cached->second.loaded) return false;
    return AudioEngine::instance().playSound2D(cached->second.data,
                                               0.7f * volumeScale_, 1.0f);
}

void UiSoundManager::playSound(const std::vector<UISample>& library) {
    if (!initialized_ || library.empty() || !library[0].loaded) return;

    float volume = 0.7f * volumeScale_;
    AudioEngine::instance().playSound2D(library[0].data, volume, 1.0f);
}

void UiSoundManager::setVolumeScale(float scale) {
    volumeScale_ = std::max(0.0f, std::min(1.0f, scale));
}

// Window sounds
void UiSoundManager::playBagOpen() { playSound(bagOpenSounds_); }
void UiSoundManager::playBagClose() { playSound(bagCloseSounds_); }
void UiSoundManager::playQuestLogOpen() { playSound(questLogOpenSounds_); }
void UiSoundManager::playQuestLogClose() { playSound(questLogCloseSounds_); }
void UiSoundManager::playCharacterSheetOpen() { playSound(characterSheetOpenSounds_); }
void UiSoundManager::playCharacterSheetClose() { playSound(characterSheetCloseSounds_); }
void UiSoundManager::playGuildBankOpen() { playSound(guildBankOpenSounds_); }
void UiSoundManager::playGuildBankClose() { playSound(guildBankCloseSounds_); }
void UiSoundManager::playAuctionHouseOpen() { playSound(auctionOpenSounds_); }
void UiSoundManager::playAuctionHouseClose() { playSound(auctionCloseSounds_); }
// Button sounds
void UiSoundManager::playButtonClick() { playSound(buttonClickSounds_); }
void UiSoundManager::playMenuButtonClick() { playSound(menuButtonSounds_); }

// Quest sounds
void UiSoundManager::playQuestActivate() { playSound(questActivateSounds_); }
void UiSoundManager::playQuestComplete() { playSound(questCompleteSounds_); }
void UiSoundManager::playQuestFailed() { playSound(questFailedSounds_); }
void UiSoundManager::playQuestUpdate() { playSound(questUpdateSounds_); }
void UiSoundManager::playFishingBite() { playSound(fishingBiteSounds_); }

// Loot sounds
void UiSoundManager::playLootCoinSmall() { playSound(lootCoinSmallSounds_); }
void UiSoundManager::playLootCoinLarge() { playSound(lootCoinLargeSounds_); }
void UiSoundManager::playLootItem() { playSound(lootItemSounds_); }

// Item sounds
void UiSoundManager::playDropOnGround() { playSound(dropSounds_); }
void UiSoundManager::playPickupBag() { playSound(pickupBagSounds_); }
void UiSoundManager::playPickupBook() { playSound(pickupBookSounds_); }
// Eating/drinking
void UiSoundManager::playEating() { playSound(eatingSounds_); }
void UiSoundManager::playDrinking() { playSound(drinkingSounds_); }

// Level up
void UiSoundManager::playLevelUp() { playSound(levelUpSounds_); }

// Achievement
void UiSoundManager::playAchievementAlert() { playSound(achievementSounds_); }

// Error/feedback
void UiSoundManager::playError() { playSound(errorSounds_); }
void UiSoundManager::playTargetSelect() { playSound(selectTargetSounds_); }
void UiSoundManager::playTargetDeselect() { playSound(deselectTargetSounds_); }

// Chat notifications
void UiSoundManager::playWhisperReceived() { playSound(whisperSounds_); }
void UiSoundManager::playMailReceived() { playSound(mailSounds_); }

// Minimap ping
void UiSoundManager::playMinimapPing() { playSound(minimapPingSounds_); }

} // namespace audio
} // namespace wowee
