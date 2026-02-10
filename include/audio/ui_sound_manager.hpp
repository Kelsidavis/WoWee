#pragma once

#include <vector>
#include <memory>
#include <string>
#include <cstdint>

namespace wowee {
namespace pipeline {
class AssetManager;
}

namespace audio {

class UISoundManager {
public:
    UISoundManager() = default;
    ~UISoundManager() = default;

    // Initialization
    bool initialize(pipeline::AssetManager* assets);
    void shutdown();

    // Volume control
    void setVolumeScale(float scale);

    // Window sounds
    void playBagOpen();
    void playBagClose();
    void playQuestLogOpen();
    void playQuestLogClose();
    void playCharacterSheetOpen();
    void playCharacterSheetClose();
    void playAuctionHouseOpen();
    void playAuctionHouseClose();
    void playGuildBankOpen();
    void playGuildBankClose();

    // Button sounds
    void playButtonClick();
    void playMenuButtonClick();

    // Quest sounds
    void playQuestActivate();
    void playQuestComplete();
    void playQuestFailed();
    void playQuestUpdate();

    // Loot sounds
    void playLootCoinSmall();
    void playLootCoinLarge();
    void playLootItem();

    // Item sounds
    void playDropOnGround();
    void playPickupBag();
    void playPickupBook();
    void playPickupCloth();
    void playPickupFood();
    void playPickupGem();

    // Eating/drinking
    void playEating();
    void playDrinking();

    // Level up
    void playLevelUp();

    // Error/feedback
    void playError();
    void playTargetSelect();
    void playTargetDeselect();

private:
    struct UISample {
        std::string path;
        std::vector<uint8_t> data;
        bool loaded;
    };

    // Sound libraries
    std::vector<UISample> bagOpenSounds_;
    std::vector<UISample> bagCloseSounds_;
    std::vector<UISample> questLogOpenSounds_;
    std::vector<UISample> questLogCloseSounds_;
    std::vector<UISample> characterSheetOpenSounds_;
    std::vector<UISample> characterSheetCloseSounds_;
    std::vector<UISample> auctionOpenSounds_;
    std::vector<UISample> auctionCloseSounds_;
    std::vector<UISample> guildBankOpenSounds_;
    std::vector<UISample> guildBankCloseSounds_;

    std::vector<UISample> buttonClickSounds_;
    std::vector<UISample> menuButtonSounds_;

    std::vector<UISample> questActivateSounds_;
    std::vector<UISample> questCompleteSounds_;
    std::vector<UISample> questFailedSounds_;
    std::vector<UISample> questUpdateSounds_;

    std::vector<UISample> lootCoinSmallSounds_;
    std::vector<UISample> lootCoinLargeSounds_;
    std::vector<UISample> lootItemSounds_;

    std::vector<UISample> dropSounds_;
    std::vector<UISample> pickupBagSounds_;
    std::vector<UISample> pickupBookSounds_;
    std::vector<UISample> pickupClothSounds_;
    std::vector<UISample> pickupFoodSounds_;
    std::vector<UISample> pickupGemSounds_;

    std::vector<UISample> eatingSounds_;
    std::vector<UISample> drinkingSounds_;

    std::vector<UISample> levelUpSounds_;

    std::vector<UISample> errorSounds_;
    std::vector<UISample> selectTargetSounds_;
    std::vector<UISample> deselectTargetSounds_;

    // State tracking
    float volumeScale_ = 1.0f;
    bool initialized_ = false;

    // Helper methods
    bool loadSound(const std::string& path, UISample& sample, pipeline::AssetManager* assets);
    void playSound(const std::vector<UISample>& library);
};

} // namespace audio
} // namespace wowee
