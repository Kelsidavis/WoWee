#include "core/audio_callback_handler.hpp"
#include "core/coordinates.hpp"
#include "core/logger.hpp"
#include "rendering/renderer.hpp"
#include "rendering/animation_controller.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_loader.hpp"
#include "game/game_handler.hpp"
#include "audio/audio_coordinator.hpp"
#include "audio/music_manager.hpp"
#include "audio/audio_engine.hpp"
#include "ui/ui_manager.hpp"

namespace wowee { namespace core {

AudioCallbackHandler::AudioCallbackHandler(
    pipeline::AssetManager& assetManager,
    audio::AudioCoordinator* audioCoordinator,
    rendering::Renderer* renderer,
    ui::UIManager* uiManager,
    game::GameHandler& gameHandler)
    : assetManager_(assetManager)
    , audioCoordinator_(audioCoordinator)
    , renderer_(renderer)
    , uiManager_(uiManager)
    , gameHandler_(gameHandler)
{
}

std::optional<std::string> AudioCallbackHandler::resolveSoundEntryPath(uint32_t soundId) const {
    auto dbc = assetManager_.loadDBC("SoundEntries.dbc");
    if (!dbc || !dbc->isLoaded()) return std::nullopt;

    int32_t idx = dbc->findRecordById(soundId);
    if (idx < 0) return std::nullopt;

    // SoundEntries.dbc (WotLK): field 2 = Name (label), fields 3-12 = File[0..9], field 23 = DirectoryBase
    const uint32_t row = static_cast<uint32_t>(idx);
    std::string dir = dbc->getString(row, 23);
    for (uint32_t f = 3; f <= 12; ++f) {
        std::string name = dbc->getString(row, f);
        if (name.empty()) continue;
        return dir.empty() ? name : dir + "\\" + name;
    }
    return std::nullopt;
}

void AudioCallbackHandler::setupCallbacks() {
    // Level-up callback - play sound, cheer emote, and trigger UI ding overlay + 3D effect
    gameHandler_.setLevelUpCallback([this](uint32_t newLevel) {
        if (uiManager_) {
            uiManager_->getGameScreen().toastManager().triggerDing(newLevel);
        }
        if (renderer_) {
            if (auto* ac = renderer_->getAnimationController()) ac->triggerLevelUpEffect(renderer_->getCharacterPosition());
        }
    });

    // Achievement earned callback - show toast banner
    gameHandler_.setAchievementEarnedCallback([this](uint32_t /*achievementId*/,
                                                    const std::string& /*name*/) {
        if (uiManager_) {
            uiManager_->getGameScreen().toastManager().playAchievementEarned();
        }
    });

    // Server-triggered music callback (SMSG_PLAY_MUSIC)
    // Resolves soundId → SoundEntries.dbc → MPQ path → MusicManager.
    gameHandler_.setPlayMusicCallback([this](uint32_t soundId) {
        if (!renderer_) return;
        auto* music = audioCoordinator_ ? audioCoordinator_->getMusicManager() : nullptr;
        if (!music) return;

        auto path = resolveSoundEntryPath(soundId);
        if (path) {
            music->playMusic(*path, /*loop=*/false);
        }
    });

    // SMSG_PLAY_SOUND: look up SoundEntries.dbc and play 2-D sound effect
    gameHandler_.setPlaySoundCallback([this](uint32_t soundId) {
        auto path = resolveSoundEntryPath(soundId);
        if (path) {
            audio::AudioEngine::instance().playSound2D(*path);
        }
    });

    // SMSG_PLAY_OBJECT_SOUND / SMSG_PLAY_SPELL_IMPACT: play as 3D positional sound at source entity
    gameHandler_.setPlayPositionalSoundCallback([this](uint32_t soundId, uint64_t sourceGuid) {
        auto path = resolveSoundEntryPath(soundId);
        if (!path) return;

        // Play as 3D sound if source entity position is available.
        // Entity stores canonical coords; listener uses render coords (camera).
        auto entity = gameHandler_.getEntityManager().getEntity(sourceGuid);
        if (entity) {
            glm::vec3 canonical{entity->getLatestX(), entity->getLatestY(), entity->getLatestZ()};
            glm::vec3 pos = core::coords::canonicalToRender(canonical);
            audio::AudioEngine::instance().playSound3D(*path, pos);
        } else {
            audio::AudioEngine::instance().playSound2D(*path);
        }
    });

    // Other player level-up callback - trigger 3D effect + chat notification
    gameHandler_.setOtherPlayerLevelUpCallback([this](uint64_t guid, uint32_t newLevel) {
        if (!renderer_) return;

        // Trigger 3D effect at the other player's position
        auto entity = gameHandler_.getEntityManager().getEntity(guid);
        if (entity) {
            glm::vec3 canonical(entity->getX(), entity->getY(), entity->getZ());
            glm::vec3 renderPos = core::coords::canonicalToRender(canonical);
            if (auto* ac = renderer_->getAnimationController()) ac->triggerLevelUpEffect(renderPos);
        }

        // Show chat message if in group
        if (gameHandler_.isInGroup()) {
            std::string name = gameHandler_.getCachedPlayerName(guid);
            if (name.empty()) name = "A party member";
            game::MessageChatData msg;
            msg.type = game::ChatType::SYSTEM;
            msg.language = game::ChatLanguage::UNIVERSAL;
            msg.message = name + " has reached level " + std::to_string(newLevel) + "!";
            gameHandler_.addLocalChatMessage(msg);
        }
    });

    // Open dungeon finder callback - server sends SMSG_OPEN_LFG_DUNGEON_FINDER
    gameHandler_.setOpenLfgCallback([this]() {
        if (uiManager_) uiManager_->getGameScreen().openDungeonFinder(gameHandler_);
    });
}

}} // namespace wowee::core
