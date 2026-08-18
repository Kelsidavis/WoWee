#pragma once

#include "ui/buff_bar_layout.hpp"
#include "ui/graphics_defaults.hpp"
#include "ui/ui_services.hpp"
#include <vulkan/vulkan.h>
#include <algorithm>
#include <string>
#include <functional>
#include <cstdint>

namespace wowee {
namespace rendering { class Renderer; }
namespace audio { class AudioCoordinator; }
namespace ui {

class InventoryScreen;
class ChatPanel;
struct ChatSettings;


/**
 * Settings panel (extracted from GameScreen)
 *
 * Owns all settings UI rendering, settings state variables, and
 * graphics preset logic.  Save/load remains in GameScreen since
 * it serialises cross-cutting state (chat, quest tracker, etc.).
 */
class SettingsPanel {
public:
    /// Push every graphics setting loaded from the config file to the thing
    /// it affects.
    ///
    /// Loading fills the pending fields and nothing more, so without this a
    /// saved view distance or clutter density is only a number the panel
    /// shows. Call it once the renderer is wired.
    void applyLoadedSettings();

    // ---- Settings UI visibility flags (written by EscapeMenu / Escape key) ----
    bool showEscapeSettingsNotice = false;
    bool showSettingsWindow = false;
    /// Which tab to jump to on the next draw, set by whoever opened the window.
    /// Cleared once honoured, so a later click on a tab is not fought over.
    std::string requestedTab_;
    bool settingsInit = false;

    // ---- Pending video / graphics settings ----
    bool pendingFullscreen = false;
    bool pendingVsync = true;
    int pendingResIndex = 0;
    int pendingResolutionWidth = 1920;
    int pendingResolutionHeight = 1080;
    bool displaySettingsLoaded_ = false;
    bool pendingShadows = true;
    float pendingShadowDistance = 300.0f;
    float pendingViewDistance = kDefaultViewDistance;
    /// How far the distance fog takes the sky's colour. See
    /// LightingManager::setFogSkyBlend.
    float pendingFogSkyBlend = 0.7f;
    /// How much distance fog. See LightingManager::setFogStrength.
    float pendingFogStrength = 0.4f;
    bool pendingWaterRefraction = true;
    int pendingBrightness = 50; // 0-100, maps to 0.0-2.0 (50 = 1.0 default)

    // ---- Pending audio settings ----
    int pendingMasterVolume = 100;
    int pendingMusicVolume = 30;
    int pendingAmbientVolume = 100;
    int pendingBellVolume = 50;
    /// One scale over every sound that is not music or ambience.
    ///
    /// WoW has a single Sound Effects slider and this client has seven - ui,
    /// combat, spell, movement, footsteps, activity, mount - plus the two voice
    /// managers. This multiplies them all, so Blizzard's slider has something to
    /// be, and the individual ones stay as the balance between them.
    int pendingEffectsVolume = 100;
    int pendingUiVolume = 100;
    int pendingCombatVolume = 100;
    int pendingSpellVolume = 100;
    int pendingMovementVolume = 100;
    int pendingFootstepVolume = 100;
    int pendingNpcVoiceVolume = 100;
    int pendingMountVolume = 70;
    int pendingActivityVolume = 100;
    bool pendingCharacterSpeech = true;

    // ---- Pending camera / controls ----
    float pendingMouseSensitivity = 0.2f;
    bool pendingInvertMouse = false;
    float pendingCameraStiffness = 30.0f;  // Camera smooth speed (higher = tighter, less sway)
    float pendingPivotHeight = 1.6f;       // Camera pivot height above feet (lower = less detached feel)
    bool pendingSmoothCameraFollow = false; // Keep lerping while turning (floaty, detached follow)
    float pendingFov = 70.0f;
    float pendingCameraShake = 1.0f;  // 0 = off, 1 = the full amount  // degrees, default matches WoW's ~70° horizontal FOV

    // ---- Pending UI / interface ----
    int pendingUiOpacity = 65;
    float pendingWindowUiScale = 1.0f;

    /// What something drawn in pixels should be scaled to on a screen of a
    /// given height, before the player says otherwise.
    ///
    /// The interface's own frames are laid out in a fixed canvas and scaled to
    /// fit, so they follow a tall screen by themselves. What this client draws
    /// itself is in pixels and does not: on a 2160-line screen it comes up at
    /// the size it would have on a 768-line one.
    ///
    /// The buff bar's rule, which is the one that keeps the HUD's proportions:
    /// the height over the reference height, held between the same bounds. The
    /// bags used steps of their own - 1.1 above 1300 lines, 1.2 above 2000 -
    /// and the two disagreed by more the further from 1080 the screen was. On a
    /// 2160-line screen the buff bar came up at 2.0 and everything beside it at
    /// 1.2, which is two neighbouring parts of one HUD at different sizes.
    ///
    /// The constants come from BuffBarMetrics rather than being written again,
    /// so there is one rule here and not a third curve.
    static float recommendedPixelScale(float displayHeight) {
        return std::clamp(displayHeight / BuffBarMetrics::kReferenceHeight,
                          BuffBarMetrics::kMinAutoScale,
                          BuffBarMetrics::kMaxAutoScale);
    }

    /// The same, held to what one setting's own row will accept.
    ///
    /// A default is assigned straight to the field rather than going through
    /// setSettingValue, so nothing clamps it on the way: the window scale came
    /// out at 2 on a tall screen where its row stops at 1.5, and sat outside
    /// its own control until something else happened to clamp it.
    static float recommendedPixelScale(float displayHeight, float lo, float hi) {
        return std::clamp(recommendedPixelScale(displayHeight), lo, hi);
    }
    bool pendingMinimapRotate = false;
    bool pendingMinimapSquare = false;
    bool pendingMinimapNpcDots = false;
    bool pendingShowMinimapClock = false;
    bool pendingShowMinimapCoordinates = false;
    bool pendingShowLatencyMeter = true;
    bool pendingSeparateBags = true;
    bool pendingShowKeyring = true;
    float pendingBagScale = 1.0f;
    bool pendingShowMicroMenu = false;

    // ---- Pending gameplay ----
    bool pendingAutoLoot = false;
    bool pendingAutoSellGrey = false;
    bool pendingAutoRepair = false;
    bool pendingIdleCameraOrbit = true;

    // ---- Pending soundtrack ----
    bool pendingUseOriginalSoundtrack = true;

    // ---- Pending buff bar layout ----
    // Multiplier for the buff/debuff bar icons, on top of the automatic
    // resolution scaling (0.75–1.5).
    float pendingBuffBarScale = 1.0f;

    // ---- Pending action bar layout ----
    bool pendingShowActionBar2 = false;  // Show bottom-left extra action bar above main bar
    float pendingActionBarScale = 1.0f;  // Multiplier for action bar slot size (0.5–1.5)
    float pendingActionBar2OffsetX = 0.0f;  // Horizontal offset from default center position
    float pendingActionBar2OffsetY = 0.0f;  // Vertical offset from default (above bar 1)
    bool pendingShowRightBar = false;   // Right-edge vertical action bar (FrameXML page 3)
    bool pendingShowLeftBar  = false;   // Left-edge vertical action bar (FrameXML page 4)
    float pendingRightBarOffsetY = 0.0f;  // Vertical offset from screen center
    float pendingLeftBarOffsetY  = 0.0f;  // Vertical offset from screen center

    // ---- Pending graphics quality ----
    int pendingGroundClutterDensity = kDefaultGroundClutter;
    // Grass, as percentages of the generator's own defaults. Separate from
    // ground clutter: clutter is M2 doodads with per-instance cost, grass is
    // one indirect draw, and a player who turns one down does not necessarily
    // mean the other.
    /// Off unless asked for: it is new, it costs generation time on the main
    /// thread, and a player who has not gone looking for it should not pay.
    bool pendingGrassEnabled = false;
    int pendingGrassDensity = 100;   // 0-300
    int pendingGrassHeight = 100;    // 50-300
    int pendingGrassDistance = 150;  // 30-2000 yards; density thins past 45
    int pendingAntiAliasing = 1;  // 0=Off, 1=2x, 2=4x, 3=8x
    bool pendingFXAA = false;     // FXAA post-process (combinable with MSAA)
    bool pendingNormalMapping = true;   // on by default
    float pendingNormalMapStrength = 0.8f;  // 0.0-2.0
    float pendingLensFlare = 1.0f;          // 0.0-2.0, sun flare strength
    int pendingFrameCap = 0;                // index into the frame-limit choices
    bool pendingPOM = true;             // on by default
    bool pendingSharpStars = true;
    int pendingPOMQuality = 1;          // 0=Low(16), 1=Medium(32), 2=High(64)
    bool pendingFSR = false;
    int pendingUpscalingMode = 0;       // 0=Off, 1=FSR1, 2=FSR3
    int pendingFSRQuality = 3;          // 0=UltraQuality, 1=Quality, 2=Balanced, 3=Native(100%)
    float pendingFSRSharpness = 1.6f;
    float pendingFSR2JitterSign = 0.38f;
    float pendingFSR2MotionVecScaleX = 1.0f;
    float pendingFSR2MotionVecScaleY = 1.0f;
    bool pendingAMDFramegen = false;

    // ---- Graphics quality presets ----
    enum class GraphicsPreset : int {
        CUSTOM = 0,
        LOW = 1,
        MEDIUM = 2,
        HIGH = 3,
        ULTRA = 4
    };
    GraphicsPreset currentGraphicsPreset = GraphicsPreset::CUSTOM;
    GraphicsPreset pendingGraphicsPreset = GraphicsPreset::CUSTOM;

    // ---- Applied-once flags (used by GameScreen::render() one-time-apply blocks) ----
    bool fsrSettingsApplied_ = false;
    float uiOpacity_ = 0.65f;  // UI element transparency (0.0 = fully transparent, 1.0 = fully opaque)
    bool minimapRotate_ = false;
    bool minimapSquare_ = false;
    bool minimapNpcDots_ = false;
    bool showMinimapClock_ = false;
    bool showMinimapCoordinates_ = false;
    bool showLatencyMeter_ = true;           // Show server latency indicator
    bool minimapSettingsApplied_ = false;
    // Separate from the apply latches. The file is re-read once, when the
    // renderer first exists; the subsystems it feeds are built at different
    // moments and each latches when it has been given its own settings.
    // Sharing one latch meant either re-reading the file every frame until the
    // slowest subsystem arrived - clobbering anything the player changed in
    // that window - or latching on the first and never feeding the rest.
    bool settingsRereadFromDisk_ = false;
    bool zoneSettingsApplied_ = false;
    bool terrainSettingsApplied_ = false;
    bool volumeSettingsApplied_ = false;  // True once saved volume settings applied to audio managers
    bool msaaSettingsApplied_ = false;   // True once saved MSAA setting applied to renderer
    bool fxaaSettingsApplied_ = false;   // True once saved FXAA setting applied to renderer
    bool lightingSettingsApplied_ = false; // True once saved shadows/brightness are applied
    bool lensFlareApplied_ = false;        // True once the saved flare strength reached the sky
    bool frameCapApplied_ = false;         // True once the saved frame limit reached the window
    bool waterRefractionApplied_ = false;
    bool normalMapSettingsApplied_ = false;  // True once saved normal map/POM settings applied

    // ---- Mute state: mute bypasses master volume without touching slider values ----
    bool soundMuted_ = false;
    float preMuteVolume_ = 1.0f;  // AudioEngine master volume before muting

    // ---- Config toggles (read by GameScreen rendering, edited by Interface tab) ----
    float nameplateScale_ = 1.0f; // Scale multiplier for nameplate bar dimensions
    bool showFriendlyNameplates_ = true;  // Shift+V toggles friendly player nameplates
    bool showDPSMeter_ = false;
    bool showCooldownTracker_ = false;
    bool showRareTracker_ = false;  // Mark nearby spawned rares/rare-elites on both maps
    bool showChestTracker_ = false; // Mark nearby spawned non-gather chests on both maps
    bool damageFlashEnabled_ = true;
    bool lowHealthVignetteEnabled_ = true; // Persistent pulsing red vignette below 20% HP

    // ---- Public methods ----

    /// Render the settings window (call from GameScreen::render)
    ///
    /// The bag windows used to be handed in as well, so that three settings
    /// could be pushed at them. They are reached through setInventoryScreen
    /// now, which is what lets those three work when they are changed from the
    /// interface's options rather than only from these sliders.
    void renderSettingsWindow(ChatPanel& chatPanel, const std::function<void()>& saveCallback);

    /// Apply audio volume levels to all audio coordinator sound managers
    void applyAudioVolumes(audio::AudioCoordinator* ac);

    /// Read or write a setting by the key the schema names it with.
    ///
    /// Here rather than in the Lua bridge because this is where the fields are.
    /// The bridge was growing a second copy of the mapping - a branch per
    /// setting in a lambda in Application - which is the shape that ends with
    /// two lists of settings that disagree about which ones exist.
    ///
    /// Values are strings, because a CVar is a string. Unknown keys answer
    /// empty and change nothing.
    [[nodiscard]] std::string settingValue(const std::string& key) const;
    bool setSettingValue(const std::string& key, const std::string& value);

    /// Push a setting that has just changed at the thing it affects.
    ///
    /// The settings window does this at each slider. A change arriving from
    /// FrameXML's options or the Wowee panel comes through setSettingValue
    /// instead, and without this it would update the number, save it, and
    /// change nothing on screen.
    void applySettingSideEffects(const std::string& key);

    /// Draw every setting the schema files under `category`, as ImGui
    /// controls, with a heading wherever the section changes.
    ///
    /// The label, the range, the choices and the hover text all come from the
    /// schema - the same rows the interface's options panels are built from -
    /// so the two windows cannot end up describing one setting differently, and
    /// a setting added to the schema appears in both.
    void drawSchemaCategory(const char* category, const std::function<void()>& saveCallback);

    /// Put every setting in `category` back to what the schema says it is when
    /// nobody has chosen. A null category restores all of them.
    ///
    /// One list rather than one per restore button - there were three, and each
    /// of them was free to disagree with the value the client actually starts
    /// with, which is the same list a fourth time.
    void restoreSchemaDefaults(const char* category);

    /// Apply the persisted global ImGui window scale without compounding ratios.
    void applyWindowUiScale();

    /// Return the platform-specific settings file path
    static std::string getSettingsPath();

    /// Set services (dependency injection)
    void setServices(const UIServices& services) { services_ = services; }

    /// The chat panel's settings, so that the chat keys can be answered here
    /// alongside every other setting rather than through a second bridge.
    void setChatSettings(ChatSettings* settings) { chatSettings_ = settings; }

    /// The bag windows, so that the three settings that only they can apply -
    /// separate windows, the keyring, the scale - take effect when they are
    /// changed from the interface's options rather than only from the slider.
    void setInventoryScreen(InventoryScreen* screen) { inventoryScreen_ = screen; }

private:
    UIServices services_;  // Injected service references
    ChatSettings* chatSettings_ = nullptr;      // Owned by ChatPanel, not by this
    InventoryScreen* inventoryScreen_ = nullptr;  // Owned by GameScreen
    float appliedWindowUiScale_ = 1.0f;
    bool windowUiScaleEditing_ = false;

    // Keybinding customization (private - only used in Controls tab)
    int pendingRebindAction_ = -1;  // -1 = not rebinding, otherwise action index
    bool awaitingKeyPress_ = false;

    // Settings tab rendering
    void renderSettingsInterfaceTab(const std::function<void()>& saveCallback);
    void renderSettingsGameplayTab(const std::function<void()>& saveCallback);
    void renderSettingsControlsTab(const std::function<void()>& saveCallback);
    void renderSettingsAudioTab(std::function<void()> saveCallback);
    void renderSettingsAboutTab();
    void applyGraphicsPreset(GraphicsPreset preset);
    void updateGraphicsPresetFromCurrentSettings();
};

} // namespace ui
} // namespace wowee
