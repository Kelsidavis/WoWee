#include "audio/ui_sound_manager.hpp"
#include "audio/audio_engine.hpp"
#include "pipeline/asset_manager.hpp"
#include "core/logger.hpp"

namespace wowee {
namespace audio {

bool UISoundManager::initialize(pipeline::AssetManager* assets) {
    if (!assets) {
        LOG_ERROR("UISoundManager: AssetManager is null");
        return false;
    }

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
    bool auctionOpenLoaded = loadSound("Sound\\Interface\\AuctionWindowOpen.wav", auctionOpenSounds_[0], assets);

    auctionCloseSounds_.resize(1);
    bool auctionCloseLoaded = loadSound("Sound\\Interface\\AuctionWindowClose.wav", auctionCloseSounds_[0], assets);

    guildBankOpenSounds_.resize(1);
    bool guildBankOpenLoaded = loadSound("Sound\\Interface\\GuildVaultOpen.wav", guildBankOpenSounds_[0], assets);

    guildBankCloseSounds_.resize(1);
    bool guildBankCloseLoaded = loadSound("Sound\\Interface\\GuildVaultClose.wav", guildBankCloseSounds_[0], assets);

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
    bool questUpdateLoaded = loadSound("Sound\\Interface\\iQuestUpdate.wav", questUpdateSounds_[0], assets);

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

    pickupClothSounds_.resize(1);
    bool pickupClothLoaded = loadSound("Sound\\Interface\\PickUp\\PickUpCloth_Leather01.wav", pickupClothSounds_[0], assets);

    pickupFoodSounds_.resize(1);
    bool pickupFoodLoaded = loadSound("Sound\\Interface\\PickUp\\PickUpFoodGeneric.wav", pickupFoodSounds_[0], assets);

    pickupGemSounds_.resize(1);
    bool pickupGemLoaded = loadSound("Sound\\Interface\\PickUp\\PickUpGems.wav", pickupGemSounds_[0], assets);

    // Load eating/drinking sounds
    eatingSounds_.resize(1);
    bool eatingLoaded = loadSound("Sound\\Interface\\iEating1.wav", eatingSounds_[0], assets);

    drinkingSounds_.resize(1);
    bool drinkingLoaded = loadSound("Sound\\Interface\\iDrinking1.wav", drinkingSounds_[0], assets);

    // Load level up sound
    levelUpSounds_.resize(1);
    bool levelUpLoaded = loadSound("Sound\\Interface\\LevelUp.wav", levelUpSounds_[0], assets);

    // Load error/feedback sounds
    errorSounds_.resize(1);
    bool errorLoaded = loadSound("Sound\\Interface\\Error.wav", errorSounds_[0], assets);

    selectTargetSounds_.resize(1);
    bool selectTargetLoaded = loadSound("Sound\\Interface\\iSelectTarget.wav", selectTargetSounds_[0], assets);

    deselectTargetSounds_.resize(1);
    bool deselectTargetLoaded = loadSound("Sound\\Interface\\iDeselectTarget.wav", deselectTargetSounds_[0], assets);

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

void UISoundManager::shutdown() {
    initialized_ = false;
}

bool UISoundManager::loadSound(const std::string& path, UISample& sample, pipeline::AssetManager* assets) {
    sample.path = path;
    sample.loaded = false;

    try {
        sample.data = assets->readFile(path);
        if (!sample.data.empty()) {
            sample.loaded = true;
            return true;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("UISoundManager: Failed to load ", path, ": ", e.what());
    }

    return false;
}

void UISoundManager::playSound(const std::vector<UISample>& library) {
    if (!initialized_ || library.empty() || !library[0].loaded) return;

    float volume = 0.7f * volumeScale_;
    AudioEngine::instance().playSound2D(library[0].data, volume, 1.0f);
}

void UISoundManager::setVolumeScale(float scale) {
    volumeScale_ = std::max(0.0f, std::min(1.0f, scale));
}

// Window sounds
void UISoundManager::playBagOpen() { playSound(bagOpenSounds_); }
void UISoundManager::playBagClose() { playSound(bagCloseSounds_); }
void UISoundManager::playQuestLogOpen() { playSound(questLogOpenSounds_); }
void UISoundManager::playQuestLogClose() { playSound(questLogCloseSounds_); }
void UISoundManager::playCharacterSheetOpen() { playSound(characterSheetOpenSounds_); }
void UISoundManager::playCharacterSheetClose() { playSound(characterSheetCloseSounds_); }
void UISoundManager::playAuctionHouseOpen() { playSound(auctionOpenSounds_); }
void UISoundManager::playAuctionHouseClose() { playSound(auctionCloseSounds_); }
void UISoundManager::playGuildBankOpen() { playSound(guildBankOpenSounds_); }
void UISoundManager::playGuildBankClose() { playSound(guildBankCloseSounds_); }

// Button sounds
void UISoundManager::playButtonClick() { playSound(buttonClickSounds_); }
void UISoundManager::playMenuButtonClick() { playSound(menuButtonSounds_); }

// Quest sounds
void UISoundManager::playQuestActivate() { playSound(questActivateSounds_); }
void UISoundManager::playQuestComplete() { playSound(questCompleteSounds_); }
void UISoundManager::playQuestFailed() { playSound(questFailedSounds_); }
void UISoundManager::playQuestUpdate() { playSound(questUpdateSounds_); }

// Loot sounds
void UISoundManager::playLootCoinSmall() { playSound(lootCoinSmallSounds_); }
void UISoundManager::playLootCoinLarge() { playSound(lootCoinLargeSounds_); }
void UISoundManager::playLootItem() { playSound(lootItemSounds_); }

// Item sounds
void UISoundManager::playDropOnGround() { playSound(dropSounds_); }
void UISoundManager::playPickupBag() { playSound(pickupBagSounds_); }
void UISoundManager::playPickupBook() { playSound(pickupBookSounds_); }
void UISoundManager::playPickupCloth() { playSound(pickupClothSounds_); }
void UISoundManager::playPickupFood() { playSound(pickupFoodSounds_); }
void UISoundManager::playPickupGem() { playSound(pickupGemSounds_); }

// Eating/drinking
void UISoundManager::playEating() { playSound(eatingSounds_); }
void UISoundManager::playDrinking() { playSound(drinkingSounds_); }

// Level up
void UISoundManager::playLevelUp() { playSound(levelUpSounds_); }

// Error/feedback
void UISoundManager::playError() { playSound(errorSounds_); }
void UISoundManager::playTargetSelect() { playSound(selectTargetSounds_); }
void UISoundManager::playTargetDeselect() { playSound(deselectTargetSounds_); }

} // namespace audio
} // namespace wowee
