#pragma once

#include "core/window.hpp"
#include "ui/unit_portrait.hpp"
#include "ui/widget_renderer.hpp"
#include "core/input.hpp"
#include "core/entity_spawner.hpp"
#include "core/appearance_composer.hpp"
#include "core/world_loader.hpp"
#include "game/character.hpp"
#include "game/game_services.hpp"
#include "pipeline/blp_loader.hpp"
#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <array>
#include <optional>
#include <future>
#include <mutex>
#include <thread>
#include <atomic>

namespace wowee {

// Forward declarations
namespace rendering { class Renderer; }
namespace ui { class UIManager; }
namespace auth { class AuthHandler; }
namespace game { class GameHandler; class World; class ExpansionRegistry; struct ExpansionProfile; }
namespace pipeline { class AssetManager; class DBCLayout; struct M2Model; struct WMOModel; }
namespace audio { enum class VoiceType; class AudioCoordinator; }
namespace addons { class AddonManager; }

namespace core {

// Handler forward declarations
class NPCInteractionCallbackHandler;
class AudioCallbackHandler;
class EntitySpawnCallbackHandler;
class AnimationCallbackHandler;
class TransportCallbackHandler;
class WorldEntryCallbackHandler;
class UIScreenCallbackHandler;

enum class AppState {
    AUTHENTICATION,
    REALM_SELECTION,
    CHARACTER_CREATION,
    CHARACTER_SELECTION,
    IN_GAME,
    DISCONNECTED
};

class Application {
    friend class WorldLoader;

public:
    Application();
    ~Application();

    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    bool initialize();
    void run();
    void shutdown();

    /// Tell the stall watchdog we are alive. Call from long synchronous work that
    /// keeps presenting frames itself (e.g. the world load) so it is not mistaken
    /// for a hung main loop.
    void beatWatchdog();

    // State management
    [[nodiscard]] AppState getState() const { return state; }
    void setState(AppState newState);
    /// The world connection dropped: tear the world down and say so.
    void handleWorldDisconnect();

    // Accessors
    Window* getWindow() { return window.get(); }
    rendering::Renderer* getRenderer() { return renderer.get(); }
    ui::UIManager* getUIManager() { return uiManager.get(); }
    auth::AuthHandler* getAuthHandler() { return authHandler.get(); }
    game::GameHandler* getGameHandler() { return gameHandler.get(); }
    game::World* getWorld() { return world.get(); }
    pipeline::AssetManager* getAssetManager() { return assetManager.get(); }
    addons::AddonManager* getAddonManager() { return addonManager_.get(); }
    game::ExpansionRegistry* getExpansionRegistry() { return expansionRegistry_.get(); }
    pipeline::DBCLayout* getDBCLayout() { return dbcLayout_.get(); }
    bool setAssetExpansionOverride(const std::string& id);
    [[nodiscard]] const std::string& getAssetExpansionOverride() const { return assetExpansionOverrideId_; }
    void reloadExpansionData();
    /// The opcode table, update fields, packet parsers and DBC layouts a
    /// protocol profile brings with it. Loaded at startup and on every
    /// expansion change, which used to be two copies.
    void loadExpansionTables(const game::ExpansionProfile& profile); // Reload DBC layouts, opcodes, etc. after expansion change

    // Singleton access
    static Application& getInstance() { return *instance; }



    // Logout to login screen
    void logoutToLogin();

    // Render bounds lookup (for click targeting / selection) - delegates to EntitySpawner
    bool getRenderBoundsForGuid(uint64_t guid, glm::vec3& outCenter, float& outRadius) const;
    bool getRenderFootZForGuid(uint64_t guid, float& outFootZ) const;
    bool getRenderPositionForGuid(uint64_t guid, glm::vec3& outPos) const;

    // Character skin composite state - delegated to AppearanceComposer
    [[nodiscard]] const std::string& getBodySkinPath() const { return appearanceComposer_ ? appearanceComposer_->getBodySkinPath() : emptyString_; }
    [[nodiscard]] const std::vector<std::string>& getUnderwearPaths() const { return appearanceComposer_ ? appearanceComposer_->getUnderwearPaths() : emptyStringVec_; }
    [[nodiscard]] uint32_t getSkinTextureSlotIndex() const { return appearanceComposer_ ? appearanceComposer_->getSkinTextureSlotIndex() : 0; }
    [[nodiscard]] uint32_t getCloakTextureSlotIndex() const { return appearanceComposer_ ? appearanceComposer_->getCloakTextureSlotIndex() : 0; }
    [[nodiscard]] uint32_t getGryphonDisplayId() const { return entitySpawner_ ? entitySpawner_->getGryphonDisplayId() : 0; }
    [[nodiscard]] uint32_t getWyvernDisplayId() const { return entitySpawner_ ? entitySpawner_->getWyvernDisplayId() : 0; }

    // Entity spawner access
    EntitySpawner* getEntitySpawner() { return entitySpawner_.get(); }

    // Appearance composer access
    AppearanceComposer* getAppearanceComposer() { return appearanceComposer_.get(); }

    // World loader access
    WorldLoader* getWorldLoader() { return worldLoader_.get(); }

    // Audio coordinator access
    audio::AudioCoordinator* getAudioCoordinator() { return audioCoordinator_.get(); }

private:
    void update(float deltaTime);

    /// One frame of being in the world - the largest arm of update()'s state
    /// switch. updateCheckpoint travels by reference because the caller's catch
    /// reports it: an exception here has to say where here was.
    void updateInGame(float deltaTime, const char*& updateCheckpoint);

    /// Everything the server says about how the player moves - speeds, rooting,
    /// gravity, feather fall, water walking - handed to the camera that owns it.
    void applyServerMovementState(float deltaTime);

    /// Keep render instances on top of what the server says. A model is placed
    /// once at spawn and would otherwise stay there while its target circle
    /// follows the entity, which reads as a ring sliding off a still NPC.
    void syncRenderInstancesToEntities(float deltaTime);
    void render();
    void performLogoutToLogin();
    void processDeferredLogoutToLogin();
    void setupUICallbacks();
    void spawnPlayerCharacter();
    // Re-spawn the in-world player model in place after a live appearance change
    // (barber shop), so the new hair/facial hair shows without a restart.
    void refreshPlayerCharacterModel();
    void buildFactionHostilityMap(uint8_t playerRace);
    void setupTestTransport();  // Test transport boat for development

    static Application* instance;

    game::GameServices gameServices_;
    std::unique_ptr<Window> window;
    std::unique_ptr<rendering::Renderer> renderer;
    std::unique_ptr<ui::UIManager> uiManager;
    std::unique_ptr<auth::AuthHandler> authHandler;
    std::unique_ptr<game::GameHandler> gameHandler;
    std::unique_ptr<game::World> world;
    std::unique_ptr<pipeline::AssetManager> assetManager;
    std::unique_ptr<addons::AddonManager> addonManager_;
    // Set by ReloadUI() from inside Lua, acted on between frames - the reload
    // destroys the state that asked for it.
    bool reloadUiPending_ = false;
    /// Draws the widget tree addons build through CreateFrame/CreateTexture.
    /// Holds the texture cache for Interface\ art, so it lives as long as the app.
    ui::WidgetRenderer widgetRenderer_;
    /// The live head-and-shoulders view the interface's portrait draws.
    ui::UnitPortrait unitPortrait_;
    /// Where the portrait's widget was found last time. Ids are stable, so
    /// this saves a scan of every widget by name on every frame; the name is
    /// still checked, because reloading the interface rebuilds the tree and
    /// the id could then belong to something else.
    uint32_t portraitWidgetId_ = 0;
    /// The FrameXML frame the real minimap is drawn into, when the original
    /// interface owns it. Looked up by name and remembered, the same as the
    /// portrait.
    uint32_t minimapWidgetId_ = 0;
    /// The FrameXML frame the world map is drawn into, when the original
    /// interface owns it. WorldMapDetailFrame rather than WorldMapFrame: the
    /// first is the map area, the second is the panel around it.
    uint32_t worldMapWidgetId_ = 0;
    /// The target's face, on the same terms as the player's. A third offscreen
    /// pass because all three are on screen at once and each holds a different
    /// model - the alternative is reloading a model per frame, which is what a
    /// shared view would amount to.
    ui::UnitPortrait targetPortrait_;
    /// And the focus, which is deliberate enough to be worth its own pass.
    ui::UnitPortrait focusPortrait_;
    /// The dressing room's figure, and its frame. The player plus whatever
    /// has been tried on, which is the paperdoll with an overlay.
    ui::UnitPortrait dressUpModel_;
    uint32_t dressUpWidgetId_ = 0;
    /// The auction house's own dressing room, which is a second frame with a
    /// second try-on list rather than the same one reused.
    ui::UnitPortrait auctionDressUpModel_;
    uint32_t auctionDressUpWidgetId_ = 0;
    /// The pet tab's figure. A creature by display id, like its portrait.
    ui::UnitPortrait petModel_;
    uint32_t petModelWidgetId_ = 0;
    /// The stable's preview, which shows whichever slot the window selected
    /// rather than the pet that is out.
    ui::UnitPortrait stableModel_;
    uint32_t stableModelWidgetId_ = 0;
    /// The mount or critter the companion tab has selected.
    ui::UnitPortrait companionModel_;
    uint32_t companionModelWidgetId_ = 0;
    /// The inspect window's figure, and the frame it is drawn into. Whole
    /// body at the paperdoll's size, because that is what it is: someone
    /// else's paperdoll.
    ui::UnitPortrait inspectModel_;
    uint32_t inspectModelWidgetId_ = 0;
    /// And whoever this client is dealing with - the face in the gossip,
    /// quest, merchant, flight master and trade panels. One view for all of
    /// them because only one such window is open at a time.
    ui::UnitPortrait npcPortrait_;
    /// And the four party members. Affordable now that a portrait is sized for
    /// the circle it is drawn into rather than for the paperdoll: at 160x200 a
    /// party face is a sixteenth of the pixels the paperdoll's target is, and
    /// four of them together cost a quarter of one of it.
    std::array<ui::UnitPortrait, 4> partyPortraits_;
    /// And the pet's, which is always a creature and so always has one.
    ui::UnitPortrait petPortrait_;

    /// The paperdoll's model view, and the frame it is drawn into. A second
    /// offscreen pass rather than a shared one: the portrait shows the face
    /// and this shows the whole figure, and they are on screen together.
    ui::UnitPortrait paperdollModel_;
    /// The facing already applied to the paperdoll, so only the change since
    /// last frame is turned.
    float paperdollFacing_ = 0.0f;
    uint32_t paperdollWidgetId_ = 0;
    bool addonsLoaded_ = false;
    std::unique_ptr<game::ExpansionRegistry> expansionRegistry_;
    // Empty means assets follow the active protocol profile. "legacy" selects
    // the root WOW_DATA_PATH manifest; otherwise this is an expansion id.
    std::string assetExpansionOverrideId_;
    std::unique_ptr<pipeline::DBCLayout> dbcLayout_;
    std::unique_ptr<EntitySpawner> entitySpawner_;
    std::unique_ptr<AppearanceComposer> appearanceComposer_;
    std::unique_ptr<WorldLoader> worldLoader_;
    std::unique_ptr<audio::AudioCoordinator> audioCoordinator_;

    // Callback handlers (extracted from setupUICallbacks)
    std::unique_ptr<NPCInteractionCallbackHandler> npcInteractionCallbacks_;
    std::unique_ptr<AudioCallbackHandler> audioCallbacks_;
    std::unique_ptr<EntitySpawnCallbackHandler> entitySpawnCallbacks_;
    std::unique_ptr<AnimationCallbackHandler> animationCallbacks_;
    std::unique_ptr<TransportCallbackHandler> transportCallbacks_;
    std::unique_ptr<WorldEntryCallbackHandler> worldEntryCallbacks_;
    std::unique_ptr<UIScreenCallbackHandler> uiScreenCallbacks_;

    // Beat by the main loop each iteration; the watchdog treats a long silence as a
    // hang. Long but healthy work that renders its own frames (world load) must beat
    // it too, or the watchdog mistakes the load for a hang.
    std::atomic<int64_t> watchdogHeartbeatMs_{0};

    AppState state = AppState::AUTHENTICATION;
    bool running = false;
    bool renderingFrame_ = false;
    bool logoutToLoginPending_ = false;
    /// Why the player is back at the login screen, when it was not their idea.
    std::string disconnectNotice_;
    bool playerCharacterSpawned = false;
    bool npcsSpawned = false;
    bool spawnSnapToGround = true;
    float lastFrameTime = 0.0f;

    // Player character info (for model spawning)
    game::Race playerRace_ = game::Race::HUMAN;
    game::Gender playerGender_ = game::Gender::MALE;
    game::Class playerClass_ = game::Class::WARRIOR;
    uint64_t spawnedPlayerGuid_ = 0;
    uint32_t spawnedAppearanceBytes_ = 0;
    uint8_t spawnedFacialFeatures_ = 0;

    // Static empty values for null-safe delegation
    static inline const std::string emptyString_;
    static inline const std::vector<std::string> emptyStringVec_;

    float facingSendCooldown_ = 0.0f;        // Rate-limits MSG_MOVE_SET_FACING
    float lastSentCanonicalYaw_ = 1000.0f;   // Sentinel - triggers first send
    float taxiStreamCooldown_ = 0.0f;
    bool idleYawned_ = false;

    // M2 transport riding: last frame's locked (canonical) render position, used to
    // detect how far the player tried to walk this frame so that delta can be applied
    // on top of the fixed ride offset instead of either fully locking movement or
    // recomputing the offset from an absolute position (see application.cpp's "M2
    // transport riding" block for why the latter is a no-op identity).
    glm::vec3 lastM2RideLockedCanonical_ = glm::vec3(0.0f);
    bool hasM2RideLock_ = false;
    glm::vec3 lastWMORideLockedRender_ = glm::vec3(0.0f);
    bool hasWMORideLock_ = false;
    uint64_t lastWMORideTransportGuid_ = 0;
    uint32_t lastWMORideMapId_ = 0xFFFFFFFFu;
    // Set when a rider boards or transfers onto a WMO ship whose deck collision hasn't
    // finished loading yet - holds the boarding-time offset (and freezes camera follow)
    // until this exact transport instance's deck floor exists, instead of letting gravity
    // fold into the attachment and drop the rider through the hull.
    bool deckFloorPending_ = false;

    bool wasAutoAttacking_ = false;
    /// Whether the player was swimming last frame, so weapons are put away
    /// once on entering the water rather than every frame in it.
    bool wasSwimmingForSheath_ = false;

    // Quest marker billboard sprites (above NPCs)
    void loadQuestMarkerModels();  // Now loads BLP textures
    void updateQuestMarkers();     // Updates billboard positions
};

} // namespace core
} // namespace wowee
