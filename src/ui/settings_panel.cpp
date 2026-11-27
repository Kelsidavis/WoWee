// ============================================================
// SettingsPanel - extracted from GameScreen
// Owns the client's settings: their values, the file they are
// saved in, and what each one does when it changes. FrameXML's
// options panels and the Wowee panel are the controls.
// ============================================================
#include "addons/addon_lua_snippets.hpp"
#include "ui/graphics_choices.hpp"
#include "ui/graphics_presets.hpp"
#include "ui/settings_panel.hpp"
#include "addons/addon_manager.hpp"
#include "ui/settings_schema.hpp"
#include "addons/lua_api_registrations.hpp"
#include "ui/display_modes.hpp"
#include "ui/inventory_screen.hpp"
#include "ui/chat_panel.hpp"
#include "ui/chat/chat_settings.hpp"
#include "ui/keybinding_manager.hpp"
#include "ui/gamepad_controls.hpp"
#include "core/gamepad.hpp"
#include "core/application.hpp"
#include "core/config_paths.hpp"
#include "core/logger.hpp"
#include "core/version.hpp"
#include "rendering/renderer.hpp"
#include "rendering/lens_flare.hpp"
#include "rendering/post_process_pipeline.hpp"
#include "rendering/lighting_manager.hpp"
#include "rendering/camera.hpp"
#include "rendering/camera_controller.hpp"
#include "rendering/minimap.hpp"
#include "rendering/terrain_manager.hpp"
// The four the moved graphics settings reach into: doodads and particles,
// the weather, and the sampler the texture filtering level sets.
#include "rendering/m2_renderer.hpp"
#include "rendering/weather.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/wmo_renderer.hpp"
#include "rendering/character_renderer.hpp"
#include "game/zone_manager.hpp"
#include "audio/audio_coordinator.hpp"
#include "audio/audio_engine.hpp"
#include "audio/music_manager.hpp"
#include "audio/ambient_sound_manager.hpp"
#include "audio/ui_sound_manager.hpp"
#include "audio/combat_sound_manager.hpp"
#include "audio/spell_sound_manager.hpp"
#include "audio/movement_sound_manager.hpp"
#include "audio/footstep_manager.hpp"
#include "audio/npc_voice_manager.hpp"
#include "audio/player_voice_manager.hpp"
#include "audio/mount_sound_manager.hpp"
#include "audio/activity_sound_manager.hpp"
#include <imgui.h>
#include <imgui_internal.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>

namespace wowee { namespace ui {

void SettingsPanel::applyWindowUiScale() {
    if (!ImGui::GetCurrentContext()) return;

    // From the row, not from a copy of its numbers: the two drifted once
    // already and the setting snapped back on the next start.
    float lo = 0.75f, hi = 3.0f;
    settingRange("windowuiscale", lo, hi);
    pendingWindowUiScale = std::clamp(pendingWindowUiScale, lo, hi);
    if (std::abs(appliedWindowUiScale_ - pendingWindowUiScale) < 0.0001f) {
        ImGui::GetIO().FontGlobalScale = pendingWindowUiScale;
        return;
    }

    // Reapply from a snapshot of the style as this panel first saw it -
    // theme and display scale already on it - instead of scaling the
    // already-scaled style. ScaleAllSizes truncates every field toward zero
    // (ImTrunc), so the incremental ratio compounded rounding on every slider
    // step: a handful of steps down walked WindowBorderHoverPadding from 4
    // to 0, and ImGui's NewFrame sanity check ("Invalid style setting!")
    // aborted the process on the next frame. From one fixed base the smallest
    // value reachable is trunc(4 * 0.75) = 3, whatever path the slider took.
    if (!windowScaleBaseCaptured_) {
        windowScaleBaseStyle_ = ImGui::GetStyle();
        windowScaleBaseCaptured_ = true;
    }
    ImGui::GetStyle() = windowScaleBaseStyle_;
    ImGui::GetStyle().ScaleAllSizes(pendingWindowUiScale);
    ImGui::GetIO().FontGlobalScale = pendingWindowUiScale;
    appliedWindowUiScale_ = pendingWindowUiScale;
}

namespace {


/// The settings a preset has an opinion about, in the order it sets them.
constexpr const char* kGraphicsPresetKeys[] = {
    "viewdistance", "shadows", "shadowdistance", "antialiasing", "fxaa",
    "normalmapping", "normalmapstrength", "parallax", "parallaxquality",
    "groundclutter",
    "grassenabled", "grassdensity", "grassheight", "grassdistance",
};

/// Every graphics setting that has to reach something when it is loaded.
///
/// A value read from the config file only lands in a pending field. Until one
/// of these is applied it is a number the panel displays and nothing else,
/// which is why the graphics settings saved from the login screen did nothing
/// until a slider was touched.
// The cameramaxdistance row stops at 50 yards, which has to be where the
// camera itself stops; see cameraDistanceFactor in the header.
static_assert(cameraDistanceFactor(50) - rendering::CameraController::kMaxDistanceFactorLimit < 0.001f &&
              rendering::CameraController::kMaxDistanceFactorLimit - cameraDistanceFactor(50) < 0.001f,
              "the cameramaxdistance row's 50 yards is not the camera's own limit");

constexpr const char* kGraphicsApplyKeys[] = {
    "viewdistance", "shadows", "shadowdistance", "antialiasing", "fxaa",
    "normalmapping", "normalmapstrength", "parallax", "parallaxquality",
    "groundclutter", "grassenabled", "grassdensity", "grassheight",
    "grassdistance", "waterrefraction", "upscaling", "fsrquality",
    "fsrsharpness", "framegen", "brightness", "uiopacity", "minimapsquare",
    "minimapnpcdots", "minimapclock", "minimapcoords", "minimaprotate", "latencymeter",
    "fogskyblend", "fogstrength", "sharpstars", "lightshafts", "mistdensity", "sunshafts",
    "raytracedlighting",
    // Moved off the game's own Effects panel, so this list is now what
    // applies them at startup; the cvar store used to do it.
    "groundclutterdistance", "particledensity", "weatherdetail",
    "environmentdetail", "texturefiltering",
    // The sound switches off the game's own panel. The stored cvars are
    // replayed at startup and reach them that way too, but a setting whose
    // only route in is the cvar behind it is one rename away from silence.
    "soundinbackground", "enablemusic", "loopmusic", "enableambience",
    "errorspeech", "enablesoundeffects",
};

/// Whether a quality preset has an opinion about this setting.
///
/// Changing one of these by hand means the settings are no longer that preset,
/// and the dropdown has to say Custom. Each of the video tab's controls used to
/// call for that itself, which is why the ones that were never given the call -
/// and every control on the interface's own options panel - could turn shadows
/// off under a preset that says they are on.
bool isGraphicsPresetKey(const std::string& key) {
    for (const char* k : kGraphicsPresetKeys) {
        if (key == k) return true;
    }
    return false;
}

}  // namespace

void SettingsPanel::applyLoadedSettings() {
    // Everything the config file just filled in, handed to the thing it
    // affects. Same route the sliders and the presets take.
    for (const char* key : kGraphicsApplyKeys) applySettingSideEffects(key);
}

void SettingsPanel::applyGraphicsPreset(GraphicsPreset preset) {
    // Custom is not a set of values - it is the name for "these are whatever
    // you made them", so it changes nothing but the marker.
    const int index = static_cast<int>(preset) - 1;
    if (index >= 0 && index < static_cast<int>(std::size(kGraphicsPresets))) {
        const auto& p = kGraphicsPresets[index];
        pendingViewDistance      = p.viewDistance;
        pendingShadows           = p.shadows;
        pendingShadowDistance    = p.shadowDistance;
        pendingAntiAliasing      = p.antiAliasing;
        pendingFXAA              = p.fxaa;
        pendingNormalMapping     = p.normalMapping;
        pendingNormalMapStrength = p.normalMapStrength;
        pendingPOM               = p.parallax;
        pendingPOMQuality        = p.parallaxQuality;
        pendingGroundClutterDensity = p.groundClutter;
        pendingGrassEnabled      = p.grass;
        pendingGrassDensity      = p.grassDensity;
        pendingGrassHeight       = p.grassHeight;
        pendingGrassDistance     = p.grassDistance;
        // Each one goes to the thing it affects through the one function that
        // knows where that is, rather than through a second copy of the same
        // renderer calls written out here.
        //
        // And to the CVar store, as setSettingValue does: view distance and
        // ground clutter are bound to CVars, the store is applied over the
        // settings file at start-up, and a preset that left it alone was
        // undone at the next start.
        for (const char* key : kGraphicsPresetKeys) {
            applySettingSideEffects(key);
            addons::noteClientSettingChanged(key, settingValue(key));
        }
    }

    currentGraphicsPreset = preset;
    pendingGraphicsPreset = preset;
}

void SettingsPanel::updateGraphicsPresetFromCurrentSettings() {
    // A preset is the current one when the settings are what it sets. The
    // floats are compared with a little room because they arrive off sliders.
    //
    // This was a second copy of the table above, written as a range per field
    // per preset - the same numbers again, plus or minus twenty. A preset whose
    // values were changed in one place and not the other would have stopped
    // recognising itself and read as Custom for good.
    for (int i = 0; i < static_cast<int>(std::size(kGraphicsPresets)); ++i) {
        const auto& p = kGraphicsPresets[i];
        const bool matches =
            std::abs(pendingViewDistance - p.viewDistance) <= 20.0f &&
            pendingShadows == p.shadows &&
            // A preset with shadows off says nothing about how far they reach.
            (!p.shadows || std::abs(pendingShadowDistance - p.shadowDistance) <= 20.0f) &&
            pendingAntiAliasing == p.antiAliasing &&
            pendingFXAA == p.fxaa &&
            pendingNormalMapping == p.normalMapping &&
            pendingPOM == p.parallax &&
            std::abs(pendingGroundClutterDensity - p.groundClutter) <= 10 &&
            pendingGrassEnabled == p.grass &&
            // As with shadows: a preset that grows no grass says nothing about
            // how dense, how tall or how far it would have been.
            (!p.grass || (std::abs(pendingGrassDensity - p.grassDensity) <= 5 &&
                          std::abs(pendingGrassHeight - p.grassHeight) <= 5 &&
                          std::abs(pendingGrassDistance - p.grassDistance) <= 10));
        if (matches) {
            pendingGraphicsPreset = static_cast<GraphicsPreset>(i + 1);
            return;
        }
    }
    pendingGraphicsPreset = GraphicsPreset::CUSTOM;
}

std::string SettingsPanel::getSettingsPath() {
    return core::getConfigRoot() + "/settings.cfg";
}


namespace {

/// Which field a setting key names.
///
/// settingValue and setSettingValue used to spell this out separately - one
/// chain of branches reading the fields, another writing them, and nothing at
/// all to say when the two stopped agreeing about which key meant which field.
/// Adding a setting meant remembering both. This is the fact once; both
/// directions read it.
///
/// Exactly one of the three pointers is set.
///
/// There was a `fraction` flag here for the two settings that crossed the
/// bridge as 0-1 while their field held 0-100. Both are schema rows now and
/// travel in the units their slider declares; the cvar keeps its own through
/// the scale in kClientCVars, which is the one place that conversion lives.
struct FieldBinding {
    const char* key;
    bool  SettingsPanel::* asBool  = nullptr;
    int   SettingsPanel::* asInt   = nullptr;
    float SettingsPanel::* asFloat = nullptr;
};

constexpr FieldBinding kFieldBindings[] = {
    // Bound to a Blizzard control as well, through kClientCVars. These are the
    // six the game's own Video, Sound and Interface panels drive, so they are
    // not in the schema - but they still have to be readable and writable,
    // because that is how those panels reach them.
    {.key = "viewdistance",   .asFloat = &SettingsPanel::pendingViewDistance},
    {.key = "fogskyblend",    .asFloat = &SettingsPanel::pendingFogSkyBlend},
    {.key = "fogstrength",    .asFloat = &SettingsPanel::pendingFogStrength},
    {.key = "lightshafts",    .asInt   = &SettingsPanel::pendingVolumetricFog},
    {.key = "raytracedlighting", .asInt = &SettingsPanel::pendingRtLighting},
    {.key = "mistdensity",    .asFloat = &SettingsPanel::pendingVolumetricDensity},
    {.key = "mousespeed",     .asFloat = &SettingsPanel::pendingMouseSensitivity},
    {.key = "minimapclock",   .asBool  = &SettingsPanel::pendingShowMinimapClock},
    {.key = "friendlyplates", .asBool  = &SettingsPanel::showFriendlyNameplates_},
    {.key = "enemyplates",    .asBool  = &SettingsPanel::showEnemyNameplates_},
    {.key = "grassenabled",   .asBool  = &SettingsPanel::pendingGrassEnabled},
    {.key = "grassdensity",   .asInt   = &SettingsPanel::pendingGrassDensity},
    {.key = "grassheight",    .asInt   = &SettingsPanel::pendingGrassHeight},
    {.key = "grassdistance",  .asInt   = &SettingsPanel::pendingGrassDistance},
    // Not a fraction any more. These two crossed the bridge as 0-1.5 and 0-1
    // while their fields held 0-150 and 0-100, which was invisible while the
    // only control for them was drawn here from the field. They are schema
    // rows now, and a schema row is read and written through the bridge - so
    // a slider declared 0-150 was showing 0.7 and writing a value the field
    // then multiplied by a hundred and clamped. The cvar keeps its own units
    // through the scale in kClientCVars, which is where that conversion
    // belongs: one place, named, rather than two that have to agree.
    {.key = "groundclutter",  .asInt   = &SettingsPanel::pendingGroundClutterDensity},
    {.key = "groundclutterdistance", .asInt = &SettingsPanel::pendingGroundClutterDistance},
    {.key = "particledensity",   .asInt = &SettingsPanel::pendingParticleDensity},
    {.key = "weatherdetail",     .asInt = &SettingsPanel::pendingWeatherDetail},
    {.key = "environmentdetail", .asInt = &SettingsPanel::pendingEnvironmentDetail},
    {.key = "texturefiltering",  .asInt = &SettingsPanel::pendingTextureFiltering},
    {.key = "effectsvolume",  .asInt   = &SettingsPanel::pendingEffectsVolume},
    {.key = "mastervolume",  .asInt  = &SettingsPanel::pendingMasterVolume},
    {.key = "mutesound",     .asBool = &SettingsPanel::soundMuted_},
    {.key = "soundinbackground",  .asBool = &SettingsPanel::pendingSoundInBackground},
    {.key = "enablemusic",        .asBool = &SettingsPanel::pendingEnableMusic},
    {.key = "loopmusic",          .asBool = &SettingsPanel::pendingLoopMusic},
    {.key = "enableambience",     .asBool = &SettingsPanel::pendingEnableAmbience},
    {.key = "errorspeech",        .asBool = &SettingsPanel::pendingErrorSpeech},
    {.key = "enablesoundeffects", .asBool = &SettingsPanel::pendingEnableSoundEffects},

    // --- Graphics ---
    {.key = "shadows",           .asBool  = &SettingsPanel::pendingShadows},
    {.key = "shadowdistance",    .asFloat = &SettingsPanel::pendingShadowDistance},
    {.key = "waterrefraction",   .asBool  = &SettingsPanel::pendingWaterRefraction},
    {.key = "antialiasing",      .asInt   = &SettingsPanel::pendingAntiAliasing},
    {.key = "fxaa",              .asBool  = &SettingsPanel::pendingFXAA},
    {.key = "normalmapping",     .asBool  = &SettingsPanel::pendingNormalMapping},
    {.key = "normalmapstrength", .asFloat = &SettingsPanel::pendingNormalMapStrength},
    {.key = "lensflare",         .asFloat = &SettingsPanel::pendingLensFlare},
    {.key = "framecap",          .asInt   = &SettingsPanel::pendingFrameCap},
    {.key = "parallax",          .asBool  = &SettingsPanel::pendingPOM},
    {.key = "sharpstars",        .asBool  = &SettingsPanel::pendingSharpStars},
    {.key = "sunshafts",         .asBool  = &SettingsPanel::pendingSunShafts},
    {.key = "parallaxquality",   .asInt   = &SettingsPanel::pendingPOMQuality},

    // --- Upscaling ---
    {.key = "upscaling",     .asInt   = &SettingsPanel::pendingUpscalingMode},
    {.key = "fsrquality",    .asInt   = &SettingsPanel::pendingFSRQuality},
    {.key = "fsrsharpness",  .asFloat = &SettingsPanel::pendingFSRSharpness},
    {.key = "framegen",      .asBool  = &SettingsPanel::pendingAMDFramegen},
    {.key = "fsrjittersign", .asFloat = &SettingsPanel::pendingFSR2JitterSign},

    // --- Display ---
    {.key = "fullscreen", .asBool = &SettingsPanel::pendingFullscreen},
    {.key = "vsync",      .asBool = &SettingsPanel::pendingVsync},
    {.key = "mapwindow",  .asBool = &SettingsPanel::showMapWindow_},
    {.key = "brightness", .asInt  = &SettingsPanel::pendingBrightness},

    // --- Camera ---
    {.key = "fov",             .asFloat = &SettingsPanel::pendingFov},
    {.key = "camerashake",     .asFloat = &SettingsPanel::pendingCameraShake},
    {.key = "camerastiffness", .asFloat = &SettingsPanel::pendingCameraStiffness},
    {.key = "cameramaxdistance", .asInt = &SettingsPanel::pendingCameraMaxDistance},
    {.key = "pivotheight",     .asFloat = &SettingsPanel::pendingPivotHeight},
    {.key = "smoothfollow",    .asBool  = &SettingsPanel::pendingSmoothCameraFollow},
    {.key = "idleorbit",       .asBool  = &SettingsPanel::pendingIdleCameraOrbit},
    {.key = "invertmouse",     .asBool  = &SettingsPanel::pendingInvertMouse},
    {.key = "gamepad",          .asBool  = &SettingsPanel::pendingGamepadEnabled},
    {.key = "gamepadlookspeed", .asFloat = &SettingsPanel::pendingGamepadLookSpeed},
    {.key = "gamepadinvertlook", .asBool = &SettingsPanel::pendingGamepadInvertLook},
    {.key = "gamepaddeadzone",  .asFloat = &SettingsPanel::pendingGamepadDeadzone},

    // --- Interface ---
    {.key = "uiopacity",     .asInt   = &SettingsPanel::pendingUiOpacity},
    {.key = "windowuiscale", .asFloat = &SettingsPanel::pendingWindowUiScale},
    {.key = "scrollspeed",   .asFloat = &SettingsPanel::pendingScrollSpeed},
    {.key = "latencymeter",  .asBool  = &SettingsPanel::pendingShowLatencyMeter},
    {.key = "checkforupdates",   .asBool  = &SettingsPanel::pendingCheckForUpdates},
    {.key = "micromenu",     .asBool  = &SettingsPanel::pendingShowMicroMenu},
    {.key = "chatboxvisible", .asBool = &SettingsPanel::pendingChatBoxVisible},
    {.key = "bagscale",      .asFloat = &SettingsPanel::pendingBagScale},
    {.key = "separatebags",  .asBool  = &SettingsPanel::pendingSeparateBags},
    {.key = "showkeyring",   .asBool  = &SettingsPanel::pendingShowKeyring},

    // --- Minimap ---
    {.key = "minimapsquare",  .asBool = &SettingsPanel::pendingMinimapSquare},
    {.key = "minimapnpcdots", .asBool = &SettingsPanel::pendingMinimapNpcDots},
    {.key = "minimapcoords",  .asBool = &SettingsPanel::pendingShowMinimapCoordinates},
    {.key = "minimaprotate",  .asBool  = &SettingsPanel::pendingMinimapRotate},

    // --- Action bars ---
    {.key = "actionbarscale",  .asFloat = &SettingsPanel::pendingActionBarScale},
    {.key = "buffbarscale",    .asFloat = &SettingsPanel::pendingBuffBarScale},
    {.key = "showbar2",        .asBool  = &SettingsPanel::pendingShowActionBar2},
    {.key = "bar2offsetx",     .asFloat = &SettingsPanel::pendingActionBar2OffsetX},
    {.key = "bar2offsety",     .asFloat = &SettingsPanel::pendingActionBar2OffsetY},
    {.key = "showrightbar",    .asBool  = &SettingsPanel::pendingShowRightBar},
    {.key = "rightbaroffsety", .asFloat = &SettingsPanel::pendingRightBarOffsetY},
    {.key = "showleftbar",     .asBool  = &SettingsPanel::pendingShowLeftBar},
    {.key = "leftbaroffsety",  .asFloat = &SettingsPanel::pendingLeftBarOffsetY},

    // --- Combat and HUD ---
    {.key = "nameplatescale",     .asFloat = &SettingsPanel::nameplateScale_},
    {.key = "dpsmeter",           .asBool  = &SettingsPanel::showDPSMeter_},
    {.key = "dpsmeterleft",       .asFloat = &SettingsPanel::dpsMeterLeft_},
    {.key = "dpsmetertop",        .asFloat = &SettingsPanel::dpsMeterTop_},
    {.key = "cooldowntracker",    .asBool  = &SettingsPanel::showCooldownTracker_},
    {.key = "raretracker",        .asBool  = &SettingsPanel::showRareTracker_},
    {.key = "chesttracker",       .asBool  = &SettingsPanel::showChestTracker_},
    {.key = "damageflash",        .asBool  = &SettingsPanel::damageFlashEnabled_},
    {.key = "lowhealthvignette",  .asBool  = &SettingsPanel::lowHealthVignetteEnabled_},

    // --- Sound ---
    {.key = "musicvolume",     .asInt  = &SettingsPanel::pendingMusicVolume},
    {.key = "ambientvolume",   .asInt  = &SettingsPanel::pendingAmbientVolume},
    {.key = "bellvolume",      .asInt  = &SettingsPanel::pendingBellVolume},
    {.key = "uivolume",        .asInt  = &SettingsPanel::pendingUiVolume},
    {.key = "combatvolume",    .asInt  = &SettingsPanel::pendingCombatVolume},
    {.key = "spellvolume",     .asInt  = &SettingsPanel::pendingSpellVolume},
    {.key = "movementvolume",  .asInt  = &SettingsPanel::pendingMovementVolume},
    {.key = "footstepvolume",  .asInt  = &SettingsPanel::pendingFootstepVolume},
    {.key = "mountvolume",     .asInt  = &SettingsPanel::pendingMountVolume},
    {.key = "activityvolume",  .asInt  = &SettingsPanel::pendingActivityVolume},
    {.key = "npcvoicevolume",  .asInt  = &SettingsPanel::pendingNpcVoiceVolume},
    {.key = "characterspeech", .asBool = &SettingsPanel::pendingCharacterSpeech},
    {.key = "woweemusic",      .asBool = &SettingsPanel::pendingUseOriginalSoundtrack},

    // --- Gameplay ---
    {.key = "autoloot",     .asBool = &SettingsPanel::pendingAutoLoot},
    {.key = "autofacetarget", .asBool = &SettingsPanel::pendingAutoFaceTarget},
    {.key = "autosellgrey", .asBool = &SettingsPanel::pendingAutoSellGrey},
    {.key = "autorepair",   .asBool = &SettingsPanel::pendingAutoRepair},
    {.key = "secureabilitytoggle", .asBool = &SettingsPanel::pendingSecureAbilityToggle},
};

/// The same, for the settings that belong to the chat panel rather than to
/// this one.
///
/// A separate table because they are fields of a different struct, not because
/// they are a different kind of setting: one lookup tries both, and a caller
/// asking for a setting by name never learns which side answered.
struct ChatFieldBinding {
    const char* key;
    bool ChatSettings::* asBool;
};

constexpr ChatFieldBinding kChatFieldBindings[] = {
    {"joingeneral",      &ChatSettings::autoJoinGeneral},
    {"jointrade",        &ChatSettings::autoJoinTrade},
    {"joinlocaldefense", &ChatSettings::autoJoinLocalDefense},
    {"joinlfg",          &ChatSettings::autoJoinLFG},
    {"joinlocal",        &ChatSettings::autoJoinLocal},
    {"chatbubbles",      &ChatSettings::showBubbles},
    {"chatbubblesparty", &ChatSettings::partyBubbles},
    // Chat's appearance is deliberately absent. Timestamps, the font size, the
    // background and the fade are all fields of this struct too, and all four
    // drive the chat panel this client draws - which is not drawn at all while
    // FrameXML owns chat. The interface has its own controls for each of them,
    // and the timestamp one already reaches the value the chat frame reads.
};

const ChatFieldBinding* findChatFieldBinding(const std::string& key) {
    for (const auto& b : kChatFieldBindings) {
        if (key == b.key) return &b;
    }
    return nullptr;
}

const FieldBinding* findFieldBinding(const std::string& key) {
    for (const auto& b : kFieldBindings) {
        if (key == b.key) return &b;
    }
    return nullptr;
}

/// Whether changing this setting means the audio coordinator has to work the
/// volumes out again.
///
/// Every one of them does, including the two that are not volumes: character
/// speech switches the player voice manager on and off inside the same call,
/// and the effects slider scales seven of the others.
bool isVolumeKey(const std::string& key) {
    return key == "mastervolume" || key == "mutesound" ||
           key == "effectsvolume" || key == "musicvolume" || key == "ambientvolume" ||
           key == "bellvolume" || key == "uivolume" || key == "combatvolume" ||
           key == "spellvolume" || key == "movementvolume" || key == "footstepvolume" ||
           key == "mountvolume" || key == "activityvolume" || key == "npcvoicevolume" ||
           key == "characterspeech";
}

}  // namespace

void SettingsPanel::applySettingSideEffects(const std::string& key) {
    // The settings window applies each value where its slider is, so a change
    // made through FrameXML or the Wowee options panel used to update the number
    // and save it and nothing else - the option looked dead until the client was
    // restarted or the same slider was touched in the other window.
    //
    // These are the same calls the sliders make, and nothing more: a setting
    // whose only effect is to be read later, like auto-repair, has no line here
    // and needs none.
    auto* renderer = services_.renderer;
    auto* camera = renderer ? renderer->getCamera() : nullptr;
    auto* cameraController = renderer ? renderer->getCameraController() : nullptr;
    auto* post = renderer ? renderer->getPostProcessPipeline() : nullptr;
    auto* wmo = renderer ? renderer->getWMORenderer() : nullptr;
    auto* chars = renderer ? renderer->getCharacterRenderer() : nullptr;

    // The switches the game's own Sound panel used to own. Writing the setting
    // has already written the cvar behind it - see noteClientSettingChanged -
    // and this is what re-reads them. applySoundCVars is where the knowledge
    // of what each one silences lives, so it stays the only copy.
    if (key == "soundinbackground" || key == "enablemusic" || key == "loopmusic" ||
        key == "enableambience" || key == "errorspeech" || key == "enablesoundeffects") {
        if (services_.addonManager) {
            if (auto* engine = services_.addonManager->getLuaEngine()) {
                if (auto* L = engine->getState()) addons::applySoundCVarSideEffects(L);
            }
        }
        return;
    }
    if (key == "viewdistance") {
        if (renderer) renderer->setViewDistance(pendingViewDistance);
    } else if (key == "shadows") {
        if (renderer) renderer->setShadowsEnabled(pendingShadows);
    } else if (key == "shadowdistance") {
        if (renderer) renderer->setShadowDistance(pendingShadowDistance);
    } else if (key == "waterrefraction") {
        if (renderer) renderer->setWaterRefractionEnabled(pendingWaterRefraction);
    } else if (key == "groundclutter") {
        if (renderer) {
            if (auto* tm = renderer->getTerrainManager()) {
                tm->setGroundClutterDensityScale(
                    static_cast<float>(pendingGroundClutterDensity) / 100.0f);
            }
        }
    } else if (key == "groundclutterdistance") {
        // The five below reach the same code the game's own Effects panel
        // reached through LuaServices. They are settings rather than cvars
        // now; kClientCVars keeps the cvar pointing at the same value.
        if (renderer) {
            if (auto* m2 = renderer->getM2Renderer())
                m2->setGroundDetailDistance(static_cast<float>(pendingGroundClutterDistance));
        }
    } else if (key == "particledensity") {
        if (renderer) {
            if (auto* m2 = renderer->getM2Renderer())
                m2->setParticleDensity(static_cast<float>(pendingParticleDensity) / 100.0f);
        }
    } else if (key == "weatherdetail") {
        // Offered as four steps and passed on as a fraction of the full
        // amount, which is what the weather system takes. Off is no weather.
        if (renderer) {
            constexpr float kWeatherSteps = 3.0f;
            if (auto* w = renderer->getWeather())
                w->setDensityScale(static_cast<float>(pendingWeatherDetail) / kWeatherSteps);
        }
    } else if (key == "environmentdetail") {
        if (renderer) {
            if (auto* m2 = renderer->getM2Renderer())
                m2->setEnvironmentDetail(static_cast<float>(pendingEnvironmentDetail) / 100.0f);
        }
    } else if (key == "texturefiltering") {
        // Levels rather than a sample count, each step doubling: 0 is off
        // and 4 is the 16x every desktop card stops at.
        if (services_.window) {
            if (auto* ctx = services_.window->getVkContext()) {
                const int level = std::clamp(pendingTextureFiltering, 0, 4);
                ctx->setAnisotropyLimit(static_cast<float>(std::min(1 << level, 16)));
            }
        }
    } else if (key == "framecap") {
        if (services_.window) services_.window->setFrameCap(frameCapFpsForChoice(pendingFrameCap));
    } else if (key == "lensflare") {
        if (renderer) {
            if (auto* lf = renderer->getLensFlare()) lf->setIntensity(pendingLensFlare);
        }
    } else if (key == "fov") {
        if (camera) camera->setFov(pendingFov);
    } else if (key == "camerashake") {
        if (cameraController) cameraController->setShakeScale(pendingCameraShake);
    } else if (key == "mousespeed") {
        if (cameraController) cameraController->setMouseSensitivity(pendingMouseSensitivity);
    } else if (key == "camerastiffness") {
        if (cameraController) cameraController->setCameraSmoothSpeed(pendingCameraStiffness);
    } else if (key == "cameramaxdistance") {
        if (cameraController) {
            cameraController->setMaxDistanceFactor(cameraDistanceFactor(pendingCameraMaxDistance));
        }
    } else if (key == "smoothfollow") {
        if (cameraController) cameraController->setSmoothCameraFollow(pendingSmoothCameraFollow);
    } else if (key == "pivotheight") {
        if (cameraController) cameraController->setPivotHeight(pendingPivotHeight);
    } else if (key == "idleorbit") {
        if (cameraController) cameraController->setIdleOrbitEnabled(pendingIdleCameraOrbit);
    } else if (key == "uiopacity") {
        uiOpacity_ = static_cast<float>(pendingUiOpacity) / 100.0f;
    } else if (key == "minimaprotate") {
        minimapRotate_ = pendingMinimapRotate;
        if (renderer) {
            if (auto* mm = renderer->getMinimap()) mm->setRotateWithCamera(pendingMinimapRotate);
        }
    } else if (key == "minimapsquare") {
        minimapSquare_ = pendingMinimapSquare;
        if (renderer) {
            if (auto* mm = renderer->getMinimap()) mm->setSquareShape(pendingMinimapSquare);
        }
    } else if (key == "invertmouse") {
        if (cameraController) cameraController->setInvertMouse(pendingInvertMouse);
    } else if (key == "gamepad") {
        ui::gamepadControls().setEnabled(pendingGamepadEnabled);
    } else if (key == "gamepadlookspeed") {
        ui::gamepadControls().setLookDegreesPerSecond(pendingGamepadLookSpeed);
    } else if (key == "gamepadinvertlook") {
        ui::gamepadControls().setInvertLook(pendingGamepadInvertLook);
    } else if (key == "gamepaddeadzone") {
        core::gamepad().setStickDeadzone(pendingGamepadDeadzone);
    } else if (key == "graphicspreset") {
        applyGraphicsPreset(pendingGraphicsPreset);
    } else if (key == "antialiasing") {
        if (renderer) {
            // Clamped here as well as inside the renderer, so the control ends
            // up showing what is actually in force.
            //
            // Apple silicon stops at 4x. Choosing 8x was clamped silently and,
            // when 4x was already running, changed nothing at all - so the
            // dropdown sat on a mode the client was not using and reads as a
            // setting that does nothing. The same is true of any device whose
            // ceiling is below the four this offers.
            const VkSampleCountFlagBits want = msaaSamplesForChoice(pendingAntiAliasing);
            VkSampleCountFlagBits allowed = want;
            if (auto* ctx = renderer->getVkContext()) {
                allowed = std::min(want, ctx->getMaxUsableSampleCount());
            }
            if (allowed != want) {
                const int granted = msaaChoiceForSamples(allowed);
                LOG_WARNING("Anti-aliasing ", static_cast<int>(want),
                            "x is more than this GPU offers; using ",
                            static_cast<int>(allowed), "x");
                pendingAntiAliasing = granted;
            }
            renderer->setMsaaSamples(allowed);
        }
    } else if (key == "fxaa") {
        if (post) post->setFXAAEnabled(pendingFXAA);
    } else if (key == "normalmapping") {
        if (wmo) wmo->setNormalMappingEnabled(pendingNormalMapping);
        if (chars) chars->setNormalMappingEnabled(pendingNormalMapping);
    } else if (key == "normalmapstrength") {
        if (wmo) wmo->setNormalMapStrength(pendingNormalMapStrength);
        if (chars) chars->setNormalMapStrength(pendingNormalMapStrength);
    } else if (key == "parallax") {
        if (wmo) wmo->setPOMEnabled(pendingPOM);
        if (chars) chars->setPOMEnabled(pendingPOM);
    } else if (key == "sharpstars") {
        if (renderer) renderer->setSharpStars(pendingSharpStars);
    } else if (key == "sunshafts") {
        if (renderer) renderer->setSunShaftsEnabled(pendingSunShafts);
    } else if (key == "parallaxquality") {
        if (wmo) wmo->setPOMQuality(pendingPOMQuality);
        if (chars) chars->setPOMQuality(pendingPOMQuality);
    } else if (key == "upscaling") {
        // pendingFSR is the older flag for "FSR 1 is on" and is what the saved
        // settings still carry, so the mode and the flag are set together
        // rather than left to disagree.
        pendingFSR = (pendingUpscalingMode == 1);
        if (renderer) {
            renderer->setFSREnabled(pendingUpscalingMode == 1);
            renderer->setFSR2Enabled(pendingUpscalingMode == 2);
            // Multisampling is the player's own setting and survives the
            // upscaler either way: re-asserted here so a device that had to
            // give it up while upscaling gets it back the moment upscaling is
            // off, rather than at the next touch of that control. A no-op
            // where nothing changed.
            renderer->setMsaaSamples(msaaSamplesForChoice(pendingAntiAliasing));
        }
    } else if (key == "fsrquality") {
        // How far below the display resolution the world is drawn, in the same
        // order the schema lists the choices.
        if (post) post->setFSRQuality(fsrScaleForChoice(pendingFSRQuality));
    } else if (key == "fsrsharpness") {
        if (post) post->setFSRSharpness(pendingFSRSharpness);
    } else if (key == "fogstrength") {
        if (renderer) {
            if (auto* lighting = renderer->getLightingManager()) {
                lighting->setFogStrength(pendingFogStrength);
            }
        }
    } else if (key == "lightshafts") {
        if (renderer) renderer->setVolumetricFogQuality(pendingVolumetricFog);
    } else if (key == "raytracedlighting") {
        if (renderer) renderer->setRtLightingMode(pendingRtLighting);
    } else if (key == "mistdensity") {
        if (renderer) renderer->setVolumetricFogDensity(pendingVolumetricDensity);
    } else if (key == "fogskyblend") {
        if (renderer) {
            if (auto* lighting = renderer->getLightingManager()) {
                lighting->setFogSkyBlend(pendingFogSkyBlend);
            }
        }
    } else if (key == "framegen") {
        if (post) post->setAmdFsr3FramegenEnabled(pendingAMDFramegen);
    } else if (key == "fsrjittersign") {
        if (post) {
            post->setFSR2DebugTuning(pendingFSR2JitterSign, pendingFSR2MotionVecScaleX,
                                     pendingFSR2MotionVecScaleY);
        }
    } else if (key == "brightness") {
        // 50 is neutral, so the field is twice the multiplier the pipeline wants.
        if (post) post->setBrightness(static_cast<float>(pendingBrightness) / 50.0f);
    } else if (key == "fullscreen") {
        if (services_.window) {
            services_.window->setFullscreen(pendingFullscreen);
            if (pendingFullscreen) {
                services_.window->applyResolution(pendingResolutionWidth,
                                                  pendingResolutionHeight);
            }
        }
    } else if (key == "vsync") {
        if (services_.window) services_.window->setVsync(pendingVsync);
    } else if (key == "windowuiscale") {
        applyWindowUiScale();
    } else if (key == "minimapnpcdots") {
        minimapNpcDots_ = pendingMinimapNpcDots;
    } else if (key == "minimapclock") {
        showMinimapClock_ = pendingShowMinimapClock;
    } else if (key == "minimapcoords") {
        showMinimapCoordinates_ = pendingShowMinimapCoordinates;
    } else if (key == "latencymeter") {
        showLatencyMeter_ = pendingShowLatencyMeter;
    } else if (key == "woweemusic") {
        // Not a volume: it changes which tracks the zone rotation can pick, and
        // switching it off has to stop whichever of ours is playing now - the
        // rotation would otherwise honour it only at the next zone change.
        //
        // The interface's options panel has offered this since the schema grew
        // and it did nothing but store the answer, because the only copy of
        // this lived beside the checkbox in the settings window.
        if (renderer) {
            if (auto* zm = renderer->getZoneManager()) {
                zm->setUseOriginalSoundtrack(pendingUseOriginalSoundtrack);
                if (!pendingUseOriginalSoundtrack) {
                    if (auto* ac = renderer->getAudioCoordinator()) {
                        ac->onOriginalSoundtrackDisabled(zm);
                    }
                }
            }
        }
    } else if (key == "grassenabled" || key == "grassdensity" || key == "grassheight" ||
               key == "grassdistance") {
        // Grass belonged in its own branch all along. These calls were sitting
        // inside the soundtrack's, so changing the soundtrack applied the grass
        // settings and changing a grass setting did nothing at all - the key
        // matched no branch and fell out of the bottom.
        if (renderer) {
            renderer->setGrassEnabled(pendingGrassEnabled);
            renderer->setGrassScales(static_cast<float>(pendingGrassDensity) / 100.0f,
                                     static_cast<float>(pendingGrassHeight) / 100.0f);
            renderer->setGrassDistance(static_cast<float>(pendingGrassDistance));
        }
    } else if (key == "chatboxvisible") {
        // Applied by deactivating every box that is not being typed in - see
        // kChatBoxVisibilityLua, which is where the script lives so that
        // something parses it before a player does.
        if (services_.gameHandler) {
            services_.gameHandler->runInterfaceCommand(
                wowee::addons::kChatBoxVisibilityLua);
        }
    } else if (isVolumeKey(key)) {
        // Every volume goes through one call, because each of them is a balance
        // against the others and the coordinator works them all out together.
        applyAudioVolumes(services_.audioCoordinator);
    }

    // And say so, for the settings whose effect is not this client's to apply.
    //
    // Interface > Bags is three of them: the bag window they describe is drawn
    // by the bundled all-bags addon now, which reads them through
    // WoweeGetSetting. Reading is not enough on its own - a checkbox has to
    // work when it is clicked, not at the next reload - and an addon has no way
    // to notice a value it is not told about. So every applied setting names
    // itself here, and anything that cares registers for it.
    if (services_.addonManager) {
        services_.addonManager->fireEvent("WOWEE_SETTING_CHANGED", {key});
    }
}


std::string SettingsPanel::settingValue(const std::string& key) const {
    // The graphics preset is an enum class rather than one of the three field
    // types, and it is the only setting whose value is derived: touching any of
    // the settings it covers moves it to Custom, so what it reads is whatever
    // the others currently amount to.
    if (key == "graphicspreset") {
        return settingNumberText(static_cast<int>(pendingGraphicsPreset));
    }
    if (const ChatFieldBinding* c = findChatFieldBinding(key)) {
        if (!chatSettings_) return {};
        return chatSettings_->*(c->asBool) ? "1" : "0";
    }
    const FieldBinding* b = findFieldBinding(key);
    if (!b) return {};
    if (b->asBool)  return this->*(b->asBool) ? "1" : "0";
    if (b->asInt) {
        const int v = this->*(b->asInt);
        return settingNumberText(static_cast<double>(v));
    }
    const float v = this->*(b->asFloat);
    return settingNumberText(v);
}

namespace {

/// A value held to the range its schema row declares.
///
/// The sliders are built from that range and the config loader clamps to it, so
/// the one way past it was the one nothing bounded: setSettingValue, which is
/// what FrameXML's panels and any addon calling WoweeSetSetting go through. It
/// took whatever it was handed - a field of view of 500, a nameplate scale of
/// -40, an interface opacity of 1000 - stored it, showed it on the control and
/// wrote it to the config, where the loader clamped it on the way back in next
/// time. The consumers that clamp for themselves, like the multisampling table,
/// were what kept it from being worse than wrong.
///
/// Keys with no schema row are returned untouched.
double clampedToSchema(const std::string& key, double value) {
    std::size_t count = 0;
    const SettingDesc* schema = clientSettingsSchema(count);
    for (std::size_t i = 0; i < count; ++i) {
        if (key != schema[i].key) continue;
        if (schema[i].kind == SettingKind::Bool) return value;
        return std::clamp(value, static_cast<double>(schema[i].minValue),
                          static_cast<double>(schema[i].maxValue));
    }
    return value;
}

}  // namespace

bool SettingsPanel::setSettingValue(const std::string& key, const std::string& value) {
    const double v = std::atof(value.c_str());
    const bool on = settingIsOn(value);

    if (key == "graphicspreset") {
        const int idx = std::clamp(static_cast<int>(v + 0.5), 0, 4);
        pendingGraphicsPreset = static_cast<GraphicsPreset>(idx);
        applySettingSideEffects(key);
        return true;
    }
    if (const ChatFieldBinding* c = findChatFieldBinding(key)) {
        if (!chatSettings_) return false;
        chatSettings_->*(c->asBool) = on;
        // The store too, as below: the speech-bubble rows are bound to the
        // CVars the Social page's boxes wrote, and returned early past the
        // write-back until the control check said a change here was undone
        // at the next start.
        addons::noteClientSettingChanged(key, settingValue(key));
        return true;
    }
    const FieldBinding* b = findFieldBinding(key);
    if (!b) return false;
    if (b->asBool) {
        this->*(b->asBool) = on;
    } else if (b->asInt) {
        this->*(b->asInt) = static_cast<int>(clampedToSchema(key, v) + 0.5);
    } else {
        this->*(b->asFloat) = static_cast<float>(clampedToSchema(key, v));
    }
    applySettingSideEffects(key);
    // And the CVar store, for the settings a Blizzard control also drives. It
    // is applied over the settings file at start-up, so without this a change
    // made here was undone at the next start by a CVar nobody had touched.
    addons::noteClientSettingChanged(key, settingValue(key));
    // Anything a preset covers, changed by hand, means these settings are no
    // longer that preset. applyGraphicsPreset does not come through here - it
    // assigns the fields itself - so there is nothing for this to fight with.
    if (isGraphicsPresetKey(key)) updateGraphicsPresetFromCurrentSettings();
    return true;
}

void SettingsPanel::applyAudioVolumes(audio::AudioCoordinator* ac) {
    if (!ac) return;
    // Every effect volume is its own balance; this is the one slider over them,
    // which is what Blizzard's Sound Effects control drives.
    const float fx = static_cast<float>(pendingEffectsVolume) / 100.0f;
    float masterScale = soundMuted_ ? 0.0f : static_cast<float>(pendingMasterVolume) / 100.0f;
    audio::AudioEngine::instance().setMasterVolume(masterScale);
    if (auto* music = ac->getMusicManager())
        music->setVolume(pendingMusicVolume);
    if (auto* ambient = ac->getAmbientSoundManager())
    {
        ambient->setVolumeScale(pendingAmbientVolume / 100.0f);
        ambient->setBellVolumeScale(pendingBellVolume / 100.0f);
    }
    if (auto* ui = ac->getUiSoundManager())
        ui->setVolumeScale(fx * pendingUiVolume / 100.0f);
    if (auto* combat = ac->getCombatSoundManager())
        combat->setVolumeScale(fx * pendingCombatVolume / 100.0f);
    if (auto* spell = ac->getSpellSoundManager())
        spell->setVolumeScale(fx * pendingSpellVolume / 100.0f);
    if (auto* movement = ac->getMovementSoundManager())
        movement->setVolumeScale(fx * pendingMovementVolume / 100.0f);
    if (auto* footstep = ac->getFootstepManager())
        footstep->setVolumeScale(fx * pendingFootstepVolume / 100.0f);
    if (auto* npcVoice = ac->getNpcVoiceManager())
        npcVoice->setVolumeScale(fx * pendingNpcVoiceVolume / 100.0f);
    if (auto* playerVoice = ac->getPlayerVoiceManager()) {
        playerVoice->setEnabled(pendingCharacterSpeech);
        // And its volume, which has no slider of its own but is an effect
        // channel like the eight above. The interface's Sound Effects switch
        // zeroes this scale along with theirs; restoring only `enabled` left
        // the character silent for good once that switch had been off.
        playerVoice->setVolumeScale(fx);
    }
    if (auto* mount = ac->getMountSoundManager())
        mount->setVolumeScale(fx * pendingMountVolume / 100.0f);
    if (auto* activity = ac->getActivitySoundManager())
        activity->setVolumeScale(fx * pendingActivityVolume / 100.0f);
}


} // namespace ui
} // namespace wowee
