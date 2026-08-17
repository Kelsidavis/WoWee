#include "ui/graphics_choices.hpp"
#include "ui/game_screen.hpp"
#include "addons/lua_api_registrations.hpp"
#include "ui/escape_action.hpp"
#include "ui/display_modes.hpp"
#include "ui/framexml_takeover.hpp"
#include "ui/scene_pick.hpp"
#include "ui/ui_colors.hpp"
#include "ui/ui_helpers.hpp"
#include "rendering/vk_context.hpp"
#include "core/application.hpp"
#include "core/appearance_composer.hpp"
#include "addons/addon_manager.hpp"
#include "core/coordinates.hpp"
#include "core/input.hpp"
#include "rendering/renderer.hpp"
#include "rendering/post_process_pipeline.hpp"
#include "rendering/animation_controller.hpp"
#include "rendering/lens_flare.hpp"
#include "rendering/wmo_renderer.hpp"
#include "rendering/terrain_manager.hpp"
#include "rendering/minimap.hpp"
#include "rendering/world_map.hpp"
#include "rendering/character_renderer.hpp"
#include "rendering/camera.hpp"
#include "rendering/camera_controller.hpp"
#include "audio/audio_coordinator.hpp"
#include "audio/audio_engine.hpp"
#include "audio/music_manager.hpp"
#include "game/zone_manager.hpp"
#include "audio/footstep_manager.hpp"
#include "audio/activity_sound_manager.hpp"
#include "audio/mount_sound_manager.hpp"
#include "audio/npc_voice_manager.hpp"
#include "audio/ambient_sound_manager.hpp"
#include "audio/ui_sound_manager.hpp"
#include "audio/combat_sound_manager.hpp"
#include "audio/spell_sound_manager.hpp"
#include "audio/movement_sound_manager.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_loader.hpp"
#include "pipeline/dbc_layout.hpp"

#include "game/expansion_profile.hpp"
#include "game/character.hpp"
#include "game/shapeshift_forms.hpp"
#include "core/logger.hpp"
#include <imgui.h>
#include <imgui_internal.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <sstream>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <cctype>
#include <chrono>
#include <ctime>

#include <unordered_set>

namespace {
    using namespace wowee::ui::colors;
    using namespace wowee::ui::helpers;
    constexpr auto& kColorRed        = kRed;
    constexpr auto& kColorBrightGreen= kBrightGreen;
    constexpr auto& kColorYellow     = kYellow;

    // Upper bound on a game object's cursor-pick sphere. A wide/tall M2 GO (a forge, an
    // anvil, a large brazier) reports a legitimately big visual radius, but its bounding
    // sphere is a poor click target: the half-diagonal reaches well past the geometry and
    // swallows NPCs standing next to it, so those units become unclickable. Clamping the
    // GO pick radius keeps large objects clickable near their center without stealing the
    // click from a neighbor. Units are never clamped - this is a GO-only correction, the
    // same reasoning that already makes WMO GOs fall back to a conservative fixed sphere.



    // Draw a four-edge screen vignette (gradient overlay along each edge).
    // Used for damage flash, low-health pulse, and level-up golden burst.
    void drawScreenEdgeVignette(uint8_t r, uint8_t g, uint8_t b,
                                int alpha, float thicknessRatio) {
        if (alpha <= 0) return;
        ImDrawList* fg = ImGui::GetForegroundDrawList();
        const float W = ImGui::GetIO().DisplaySize.x;
        const float H = ImGui::GetIO().DisplaySize.y;
        const float thickness = std::min(W, H) * thicknessRatio;
        const ImU32 edgeCol = IM_COL32(r, g, b, alpha);
        const ImU32 fadeCol = IM_COL32(r, g, b, 0);
        // Top
        fg->AddRectFilledMultiColor(ImVec2(0, 0), ImVec2(W, thickness),
                                    edgeCol, edgeCol, fadeCol, fadeCol);
        // Bottom
        fg->AddRectFilledMultiColor(ImVec2(0, H - thickness), ImVec2(W, H),
                                    fadeCol, fadeCol, edgeCol, edgeCol);
        // Left
        fg->AddRectFilledMultiColor(ImVec2(0, 0), ImVec2(thickness, H),
                                    edgeCol, fadeCol, fadeCol, edgeCol);
        // Right
        fg->AddRectFilledMultiColor(ImVec2(W - thickness, 0), ImVec2(W, H),
                                    fadeCol, edgeCol, edgeCol, fadeCol);
    }

}

namespace wowee { namespace ui {

GameScreen::GameScreen() {
    loadSettings();
}

void GameScreen::applySavedAntiAliasing(rendering::Renderer* renderer) {
    if (!renderer) return;
    settingsPanel_.msaaSettingsApplied_ = true;
    if (settingsPanel_.pendingAntiAliasing <= 0) return;
    renderer->setMsaaSamples(msaaSamplesForChoice(settingsPanel_.pendingAntiAliasing));
}

// Set UI services and propagate to child components

namespace {
/// How far the cursor may travel between press and release and still count as a
/// click rather than a camera drag.
///
/// This was a flat five pixels, which is a different gesture on different
/// screens: five pixels is a third of a percent of a 1280-wide window and a
/// tenth of that on a 3840-wide one, so the same small hand movement that reads
/// as a click on a modest display reads as a drag on a large one - and a
/// discarded right-click is an NPC that cannot be interacted with at all.
/// Scaled by the window, with a floor so it never becomes stricter than it was.
float clickDragThreshold() {
    const ImGuiIO& io = ImGui::GetIO();
    const float shorter = std::min(io.DisplaySize.x, io.DisplaySize.y);
    return std::max(5.0f, shorter * 0.008f);
}
}

bool GameScreen::getFullscreen() const {
    return services_.window ? services_.window->isFullscreen()
                            : settingsPanel_.pendingFullscreen;
}

void GameScreen::setFullscreen(bool enabled) {
    settingsPanel_.pendingFullscreen = enabled;
    if (services_.window) services_.window->setFullscreen(enabled);
}

int GameScreen::getResolutionIndex() const {
    if (!services_.window) return settingsPanel_.pendingResIndex;
    return displayResolutionIndexFor(services_.window->getWidth(),
                                     services_.window->getHeight());
}

void GameScreen::setResolutionIndex(int index) {
    if (index < 0 || index >= kNumDisplayResolutions) return;
    settingsPanel_.pendingResIndex = index;
    settingsPanel_.pendingResolutionWidth  = kDisplayResolutions[index][0];
    settingsPanel_.pendingResolutionHeight = kDisplayResolutions[index][1];
    if (services_.window)
        services_.window->applyResolution(settingsPanel_.pendingResolutionWidth,
                                          settingsPanel_.pendingResolutionHeight);
}

/// Anti-aliasing, shared with the interface's own Multisampling dropdown.
///
/// Through applySettingSideEffects rather than by writing the field, so the
/// renderer is told: the field alone is the value with nothing applying it,
/// which is the shape [[state_without_an_event]] describes.
int GameScreen::getAntiAliasingIndex() const {
    return settingsPanel_.pendingAntiAliasing;
}

void GameScreen::setAntiAliasingIndex(int index) {
    if (index < 0 || index > 3) return;
    settingsPanel_.pendingAntiAliasing = index;
    settingsPanel_.applySettingSideEffects("antialiasing");
}

void GameScreen::setServices(const UIServices& services) {
    services_ = services;
    // Settings are loaded by the constructor before services are injected.
    // Apply the saved display pacing as soon as the actual window is available.
    if (services_.window &&
        services_.window->isVsyncEnabled() != settingsPanel_.pendingVsync) {
        services_.window->setVsync(settingsPanel_.pendingVsync);
    }
    // The chat channels the player wants joined, pushed as soon as the handler
    // exists rather than only from the in-game frame loop.
    //
    // The handler joins its default channels once, on first world entry, and
    // that happens during the world load - before the first in-game frame and
    // so before the per-frame sync had ever run. The handler was therefore reading
    // its own defaults, which are all true, and a channel the player had turned
    // off was joined anyway. Being the only entry that joins, there was no
    // second chance to get it right.
    if (services_.gameHandler) {
        auto& caj = services_.gameHandler->chatAutoJoin;
        caj.general      = chatPanel_.chatAutoJoinGeneral;
        caj.trade        = chatPanel_.chatAutoJoinTrade;
        caj.localDefense = chatPanel_.chatAutoJoinLocalDefense;
        caj.lfg          = chatPanel_.chatAutoJoinLFG;
        caj.local        = chatPanel_.chatAutoJoinLocal;
    }

    if (services_.window && settingsPanel_.displaySettingsLoaded_) {
        services_.window->setFullscreen(settingsPanel_.pendingFullscreen);
        services_.window->applyResolution(settingsPanel_.pendingResolutionWidth,
                                          settingsPanel_.pendingResolutionHeight);
    }
    // Update legacy pointer for compatibility
    appearanceComposer_ = services.appearanceComposer;
    // Propagate to child panels
    chatPanel_.setServices(services);
    toastManager_.setServices(services);
    dialogManager_.setServices(services);
    settingsPanel_.setServices(services);
    // The chat settings are the chat panel's, and the options panels ask the
    // settings panel for every setting by name. Handing it the chat's own
    // struct is what lets those keys be answered from the same place as the
    // rest rather than through a second bridge with its own list of names.
    settingsPanel_.setChatSettings(&chatPanel_.settings);
    settingsPanel_.setInventoryScreen(&inventoryScreen);
    combatUI_.setServices(services);
    socialPanel_.setServices(services);
    windowManager_.setServices(services);
    applyCameraControlSettings();
    // The settings file is read in the constructor, before there is a renderer
    // to hand the graphics values to. This is where one arrives.
    settingsPanel_.applyLoadedSettings();
}

void GameScreen::applyCameraControlSettings() {
    auto* renderer = services_.renderer;
    if (!renderer) return;

    // Field of view is on the camera rather than its controller, and the
    // camera is built asking for 60 - so a saved fov, and the 70 the schema
    // defaults to, only ever reached it when the slider was moved.
    if (auto* cam = renderer->getCamera()) cam->setFov(settingsPanel_.pendingFov);

    if (auto* cam = renderer->getCameraController()) {
        cam->setMouseSensitivity(settingsPanel_.pendingMouseSensitivity);
        cam->setInvertMouse(settingsPanel_.pendingInvertMouse);
        cam->setCameraSmoothSpeed(settingsPanel_.pendingCameraStiffness);
        cam->setPivotHeight(settingsPanel_.pendingPivotHeight);
        cam->setIdleOrbitEnabled(settingsPanel_.pendingIdleCameraOrbit);
        cam->setSmoothCameraFollow(settingsPanel_.pendingSmoothCameraFollow);
        cam->setShakeScale(settingsPanel_.pendingCameraShake);
    }
}

namespace {

/// Put the bags up, for a vendor or the guild bank.
///
/// Not OpenAllBags: that one toggles, and its reopen path hides every open bag
/// before showing them all again. ContainerFrame_GenerateFrame indexes its bag
/// list by a counter ContainerFrame_OnHide maintains, and this client runs
/// OnHide at the end of the frame rather than from Hide, so the count is stale
/// for the whole sequence and the reopened bags write themselves over one
/// index. The anchor pass then had one name where three belonged, and every
/// bag but the first kept the position its XML gave it - the same one. Walking
/// up to a vendor with bags already open stacked them.
///
/// OpenBackpack and OpenBag are the interface's own openers and no-op on a bag
/// that is already showing, so nothing is hidden and the list stays whole.
/// This is what FrameXML's own merchant frame does with the backpack.
///
/// The bag *keybind* still goes through OpenAllBags, deliberately: that one is
/// a toggle and the player is asking it to toggle. It is still exposed to the
/// same fault, which goes away when OnHide runs from Hide.
constexpr const char* kOpenBagsCommand =
    "OpenBackpack() for i = 1, 4 do OpenBag(i) end";

void openBagsForTrading(game::GameHandler& gameHandler,
                        InventoryScreen& inventoryScreen) {
    if (frameXmlOwns(UiElement::Bags)) {
        gameHandler.runInterfaceCommand(kOpenBagsCommand);
    } else if (inventoryScreen.isSeparateBags()) {
        inventoryScreen.openAllBags();
    } else if (!inventoryScreen.isOpen()) {
        inventoryScreen.setOpen(true);
    }
}

}  // namespace

void GameScreen::render(game::GameHandler& gameHandler) {
    // Apply before any Begin() calls so a scale change cannot alter style
    // metrics halfway through an ImGui frame.
    settingsPanel_.applyWindowUiScale();

    // Set up chat bubble callback (once) and cache game handler in ChatPanel
    chatPanel_.setupCallbacks(gameHandler);
    toastManager_.setupCallbacks(gameHandler);

    // Set up appearance-changed callback to refresh inventory preview (barber shop, etc.)
    if (!appearanceCallbackSet_) {
        gameHandler.setAppearanceChangedCallback([this]() {
            inventoryScreenCharGuid_ = 0;  // force preview re-sync on next frame
        });
        appearanceCallbackSet_ = true;
    }

    // Set up UI error frame callback (once)
    if (!uiErrorCallbackSet_) {
        gameHandler.setUIErrorCallback([this](const std::string& msg) {
            // The sound first, and outside the gate. It belongs to no
            // element - the error is the same error whoever draws the text -
            // and it sat after the return, so handing the errors over would
            // have taken the sound with them.
            if (auto* ac = services_.audioCoordinator) {
                if (auto* sfx = ac->getUiSoundManager()) sfx->playError();
            }
            // Both interfaces show these, and the event that carries them
            // reaches whichever is drawing.
            if (frameXmlOwns(UiElement::UiErrors)) return;
            uiErrors_.push_back({.text = msg, .age = 0.0f});
            if (uiErrors_.size() > 5) uiErrors_.erase(uiErrors_.begin());
        });
        uiErrorCallbackSet_ = true;
    }

    // No flash callback here. The red overlay on a failed cast belonged to
    // this client's own action bar; FrameXML draws the bars and flashes its
    // own button from UI_ERROR_MESSAGE.

    // Apply UI transparency setting
    float prevAlpha = ImGui::GetStyle().Alpha;
    ImGui::GetStyle().Alpha = settingsPanel_.uiOpacity_;

    // Sync minimap opacity with UI opacity
    {
        auto* renderer = services_.renderer;
        if (renderer) {
            if (auto* minimap = renderer->getMinimap()) {
                minimap->setOpacity(settingsPanel_.uiOpacity_);
            }
        }
    }

    // Apply initial settings when renderer becomes available
    if (!settingsPanel_.minimapSettingsApplied_ || !settingsPanel_.zoneSettingsApplied_ ||
        !settingsPanel_.terrainSettingsApplied_) {
        auto* renderer = services_.renderer;
        if (renderer) {
            // Re-read from disk once. This screen is constructed by UIManager's
            // constructor, which runs before the login screen exists, so
            // loadSettings() in the constructor saw the file as it was at
            // startup; the login screen's graphics page writes the same file.
            // Once, not every frame: the subsystems below arrive at different
            // moments, and re-reading until the slowest one turned up threw
            // away anything the player changed while waiting.
            if (!settingsPanel_.settingsRereadFromDisk_) {
                settingsPanel_.settingsRereadFromDisk_ = true;
                loadSettings();
            }

            // Each subsystem latches when it has been given its settings. One
            // latch for all three meant the first to exist marked the whole
            // apply done and the others never got theirs, which is how a saved
            // soundtrack setting could be ignored for a whole run.
            if (!settingsPanel_.minimapSettingsApplied_) {
                if (auto* minimap = renderer->getMinimap()) {
                    settingsPanel_.minimapRotate_ = false;
                    settingsPanel_.pendingMinimapRotate = false;
                    minimap->setRotateWithCamera(false);
                    minimap->setSquareShape(settingsPanel_.minimapSquare_);
                    settingsPanel_.minimapSettingsApplied_ = true;
                }
            }
            if (!settingsPanel_.zoneSettingsApplied_) {
            if (auto* zm = renderer->getZoneManager()) {
                zm->setUseOriginalSoundtrack(settingsPanel_.pendingUseOriginalSoundtrack);
                // Setting the flag alone is not the whole apply: music that
                // already started during the loading screen keeps playing
                // through a disabled setting.
                if (!settingsPanel_.pendingUseOriginalSoundtrack) {
                    if (auto* ac = renderer->getAudioCoordinator()) {
                        ac->onOriginalSoundtrackDisabled(zm);
                    }
                }
                settingsPanel_.zoneSettingsApplied_ = true;
            }
            }
            if (!settingsPanel_.terrainSettingsApplied_) {
            if (auto* tm = renderer->getTerrainManager()) {
                tm->setGroundClutterDensityScale(
                    static_cast<float>(settingsPanel_.pendingGroundClutterDensity) / 100.0f);
                renderer->setGrassEnabled(settingsPanel_.pendingGrassEnabled);
                renderer->setGrassScales(
                    static_cast<float>(settingsPanel_.pendingGrassDensity) / 100.0f,
                    static_cast<float>(settingsPanel_.pendingGrassHeight) / 100.0f);
                settingsPanel_.terrainSettingsApplied_ = true;
            }
            }
            // Restore mute state: save actual master volume first, then apply mute
            if (settingsPanel_.soundMuted_) {
                float actual = audio::AudioEngine::instance().getMasterVolume();
                settingsPanel_.preMuteVolume_ = (actual > 0.0f) ? actual
                    : static_cast<float>(settingsPanel_.pendingMasterVolume) / 100.0f;
                audio::AudioEngine::instance().setMasterVolume(0.0f);
            }
        }
    }

    // Settings are loaded before renderer services are injected. Apply the
    // saved lighting state once the renderer exists so opening the settings
    // window cannot become the first operation that changes scene lighting.
    if (!settingsPanel_.lightingSettingsApplied_) {
        if (auto* renderer = services_.renderer) {
            renderer->setShadowsEnabled(settingsPanel_.pendingShadows);
            renderer->setShadowDistance(settingsPanel_.pendingShadowDistance);
            renderer->setViewDistance(settingsPanel_.pendingViewDistance);
            // The latch waits for the pipeline, not just the renderer.
            //
            // Settings are loaded in the constructor, before the renderer is
            // injected, so the apply that happens while reading the file does
            // nothing and this is the only place brightness ever reaches the
            // pipeline. Marking the work done while the pipeline was still null
            // meant it never did: the value was read from the file, held
            // correctly in the settings panel, and never handed to anything
            // that draws - which reads as gamma not being saved at all.
            auto* post = renderer->getPostProcessPipeline();
            if (post) {
                post->setBrightness(static_cast<float>(settingsPanel_.pendingBrightness) / 50.0f);
                settingsPanel_.lightingSettingsApplied_ = true;
            }
        }
    }

    // Apply saved volume settings once when audio managers first become available
    if (!settingsPanel_.volumeSettingsApplied_) {
        auto* ac = services_.audioCoordinator;
        if (ac && ac->getUiSoundManager()) {
            settingsPanel_.applyAudioVolumes(ac);
            settingsPanel_.volumeSettingsApplied_ = true;
        }
    }

    // Normally already done at startup - see applySavedAntiAliasing, which the
    // application calls before the first frame. This is the fallback for a
    // renderer that was not there yet, and it latches either way.
    if (!settingsPanel_.msaaSettingsApplied_ && settingsPanel_.pendingAntiAliasing > 0) {
        if (auto* renderer = services_.renderer) {
            applySavedAntiAliasing(renderer);
        }
    } else {
        settingsPanel_.msaaSettingsApplied_ = true;
    }

    // Apply saved FXAA setting once the post-process pipeline is available.
    //
    // The pipeline, not the renderer. It is built after the renderer is, and
    // the same shape of mistake next door - latching as soon as the renderer
    // existed - is why gamma was read from the file and never reached anything
    // that draws. Dereferencing it unchecked was the other half: on the frames
    // where it is genuinely absent this was a null call, not a missed setting.
    if (!settingsPanel_.fxaaSettingsApplied_) {
        auto* renderer = services_.renderer;
        auto* post = renderer ? renderer->getPostProcessPipeline() : nullptr;
        if (post) {
            post->setFXAAEnabled(settingsPanel_.pendingFXAA);
            settingsPanel_.fxaaSettingsApplied_ = true;
        }
    }

    // Apply saved water refraction setting once when renderer is available
    if (!settingsPanel_.waterRefractionApplied_) {
        auto* renderer = services_.renderer;
        if (renderer) {
            renderer->setWaterRefractionEnabled(settingsPanel_.pendingWaterRefraction);
            settingsPanel_.waterRefractionApplied_ = true;
        }
    }

    // The saved frame limit, once the window exists to take it.
    if (!settingsPanel_.frameCapApplied_) {
        if (services_.window) {
            services_.window->setFrameCap(frameCapFpsForChoice(settingsPanel_.pendingFrameCap));
            settingsPanel_.frameCapApplied_ = true;
        }
    }

    // The saved sun flare strength, once the sky exists to take it.
    //
    // Its own latch, and it waits for the flare rather than the renderer: the
    // sky is built later than the renderer is, and marking the work done while
    // it was still absent is how brightness came to be read from the file and
    // never handed to anything.
    if (!settingsPanel_.lensFlareApplied_) {
        if (auto* renderer = services_.renderer) {
            if (auto* lf = renderer->getLensFlare()) {
                lf->setIntensity(settingsPanel_.pendingLensFlare);
                settingsPanel_.lensFlareApplied_ = true;
            }
        }
    }

    // Apply saved normal mapping / POM settings once when WMO renderer is available
    if (!settingsPanel_.normalMapSettingsApplied_) {
        auto* renderer = services_.renderer;
        if (renderer) {
            if (auto* wr = renderer->getWMORenderer()) {
                wr->setNormalMappingEnabled(settingsPanel_.pendingNormalMapping);
                wr->setNormalMapStrength(settingsPanel_.pendingNormalMapStrength);
                wr->setPOMEnabled(settingsPanel_.pendingPOM);
                wr->setPOMQuality(settingsPanel_.pendingPOMQuality);
                if (auto* cr = renderer->getCharacterRenderer()) {
                    cr->setNormalMappingEnabled(settingsPanel_.pendingNormalMapping);
                    cr->setNormalMapStrength(settingsPanel_.pendingNormalMapStrength);
                    cr->setPOMEnabled(settingsPanel_.pendingPOM);
                    cr->setPOMQuality(settingsPanel_.pendingPOMQuality);
                    // Both told before this is done with. The two renderers are
                    // built one after the other in the same function today, so
                    // the character one is never missing while the WMO one is
                    // here - but marking it applied from inside the outer guard
                    // meant that if it ever were, characters would keep the
                    // defaults and nothing would try again. Re-telling the WMO
                    // renderer on a later frame costs four setters.
                    settingsPanel_.normalMapSettingsApplied_ = true;
                }
            }
        }
    }

    // Apply saved upscaling setting once when renderer is available
    if (!settingsPanel_.fsrSettingsApplied_) {
        auto* renderer = services_.renderer;
        // Every branch below reaches through getPostProcessPipeline(), so the
        // pipeline is what this waits for - see the FXAA block above.
        auto* fsrPost = renderer ? renderer->getPostProcessPipeline() : nullptr;
        if (renderer && fsrPost) {
#ifdef __APPLE__
            // FidelityFX and AMD frame generation are unsupported through the
            // macOS MoltenVK path. Old settings files must not silently retain
            // either feature after the controls are hidden.
            settingsPanel_.pendingUpscalingMode = 0;
            settingsPanel_.pendingFSR = false;
            settingsPanel_.pendingAMDFramegen = false;
            renderer->getPostProcessPipeline()->setAmdFsr3FramegenEnabled(false);
            renderer->setFSREnabled(false);
            renderer->setFSR2Enabled(false);
            settingsPanel_.fsrSettingsApplied_ = true;
#else
            renderer->getPostProcessPipeline()->setFSRQuality(
                fsrScaleForChoice(settingsPanel_.pendingFSRQuality));
            renderer->getPostProcessPipeline()->setFSRSharpness(settingsPanel_.pendingFSRSharpness);
            renderer->getPostProcessPipeline()->setFSR2DebugTuning(settingsPanel_.pendingFSR2JitterSign, settingsPanel_.pendingFSR2MotionVecScaleX, settingsPanel_.pendingFSR2MotionVecScaleY);
            renderer->getPostProcessPipeline()->setAmdFsr3FramegenEnabled(settingsPanel_.pendingAMDFramegen);
            int effectiveMode = settingsPanel_.pendingUpscalingMode;

            // Defer FSR2/FSR3 activation until fully in-world to avoid
            // init issues during login/character selection screens.
            if (effectiveMode == 2 && gameHandler.getState() != game::WorldState::IN_WORLD) {
                renderer->setFSREnabled(false);
                renderer->setFSR2Enabled(false);
            } else {
                renderer->setFSREnabled(effectiveMode == 1);
                renderer->setFSR2Enabled(effectiveMode == 2);
                settingsPanel_.fsrSettingsApplied_ = true;
            }
#endif
        }
    }

    // Apply auto-loot / auto-sell settings to GameHandler every frame (cheap bool sync)
    gameHandler.setAutoLoot(settingsPanel_.pendingAutoLoot);
    gameHandler.setAutoSellGrey(settingsPanel_.pendingAutoSellGrey);
    gameHandler.setAutoRepair(settingsPanel_.pendingAutoRepair);

    // Sync chat auto-join settings to GameHandler
    gameHandler.chatAutoJoin.general = chatPanel_.chatAutoJoinGeneral;
    gameHandler.chatAutoJoin.trade = chatPanel_.chatAutoJoinTrade;
    gameHandler.chatAutoJoin.localDefense = chatPanel_.chatAutoJoinLocalDefense;
    gameHandler.chatAutoJoin.lfg = chatPanel_.chatAutoJoinLFG;
    gameHandler.chatAutoJoin.local = chatPanel_.chatAutoJoinLocal;

    // Process targeting input before UI windows
    processTargetInput(gameHandler);

    if (!frameXmlOwns(UiElement::PlayerFrame)) renderPlayerFrame(gameHandler);

    // Pet frame (below player frame, only when player has an active pet)
    if (gameHandler.hasPet() && !frameXmlOwns(UiElement::PetFrame)) {
        renderPetFrame(gameHandler);
    }

    // Auto-open pet rename modal when server signals the pet is renameable (first tame)
    if (gameHandler.consumePetRenameablePending()) {
        petRenameOpen_ = true;
        petRenameBuf_[0] = '\0';
    }

    // Totem frame (Shaman only, when any totem is active)
    if (gameHandler.getPlayerClass() == 7 && !frameXmlOwns(UiElement::Totems)) {
        renderTotemFrame(gameHandler);
    }

    // Target frame (only when we have a target)
    if (gameHandler.hasTarget() && !frameXmlOwns(UiElement::TargetFrame)) {
        renderTargetFrame(gameHandler);
    }

    // Focus target frame (only when we have a focus)
    if (gameHandler.hasFocus() && !frameXmlOwns(UiElement::FocusFrame)) {
        renderFocusFrame(gameHandler);
    }

    // Render windows
    if (showPlayerInfo) {
        renderPlayerInfo(gameHandler);
    }

    if (showEntityWindow) {
        renderEntityList(gameHandler);
    }

    // The chat window is FrameXML's and this client no longer draws one, but
    // the slash commands still belong here: FrameXML's edit box hands an
    // unknown command to runClientChatCommand, which is this client's own
    // registry, and the handlers set the flags read below.
    //
    // These used to sit inside the same gate as the window that is gone, so
    // every one of them ran, set its flag, and had it read by nobody:
    // /inspect, /threat, /bgscore, /gm and /who all did nothing at all.
    {
        auto cmds = chatPanel_.consumeSlashCommands();
        if (cmds.showInspect) socialPanel_.openInspectWindow(gameHandler);
        if (cmds.toggleThreat) combatUI_.showThreatWindow_ = !combatUI_.showThreatWindow_;
        // Both windows are gated on their element, so with either handed over
        // the slash command set a flag whose only reader is switched off.
        if (cmds.showBgScore) {
            if (frameXmlOwns(UiElement::BattlegroundScore))
                gameHandler.runInterfaceCommand("ToggleWorldStateScoreFrame()");
            else
                combatUI_.showBgScoreboard_ = !combatUI_.showBgScoreboard_;
        }
        if (cmds.showGmTicket) {
            gameHandler.runInterfaceCommand("ToggleHelpFrame()");
        }
        // Tab two is the who list. The client's own who window is gated on
        // the social element like the rest of that panel's windows.
        if (cmds.showWho) {
            if (frameXmlOwns(UiElement::Social))
                gameHandler.runInterfaceCommand("ToggleFriendsFrame(2)");
            else
                socialPanel_.showWhoWindow_ = true;
        }
        if (cmds.toggleCombatLog) combatUI_.showCombatLog_ = !combatUI_.showCombatLog_;
        if (cmds.takeScreenshot) takeScreenshot();
    }

    // ---- New UI elements ----
    if (!frameXmlOwns(UiElement::MicroMenu)) renderMicroMenu(gameHandler);
    auto spellIconFn = [this](uint32_t id, pipeline::AssetManager* am) { return getSpellIcon(id, am); };
    if (!frameXmlOwns(UiElement::CastBar))
        combatUI_.renderCastBar(gameHandler, spellIconFn);
    if (!frameXmlOwns(UiElement::PlayerFrame)) {
        renderMirrorTimers(gameHandler);
    }
    combatUI_.renderCooldownTracker(gameHandler, settingsPanel_, spellIconFn);
    if (!frameXmlOwns(UiElement::QuestTracker))
        renderQuestObjectiveTracker(gameHandler);
    renderNameplates(gameHandler);  // player names always shown; NPC plates gated by showNameplates_
    combatUI_.renderBattlegroundScore(gameHandler);
    combatUI_.renderRaidWarningOverlay(gameHandler);
    // Blizzard_CombatText draws its own once it is loaded, which happens the
    // moment the player touches the float-mode dropdown in the interface
    // options. Not an element gate: the addon arrives mid-run, so this has to
    // keep drawing until it does and then stand down.
    offerPendingTradeItem(gameHandler);
    // The master switch for the damage and healing numbers, which the panel
    // offers and nothing read: the numbers were drawn whatever it said. Its own
    // eighteen filters - fctDamage, fctHealing and the rest - are still
    // unread; this is the one that turns the whole thing off, which is what
    // somebody who does not want numbers over the world actually reaches for.
    if (!frameXmlDrawsCombatText() &&
        addons::storedCVarValue("enableCombatText", "1") != "0") {
        combatUI_.renderCombatText(gameHandler);
    }
    combatUI_.renderDPSMeter(gameHandler, settingsPanel_, lastTargetFrameBottom_);
    if (!frameXmlOwns(UiElement::Durability)) {
        renderDurabilityWarning(gameHandler);
    }
    // Half of this handover was in place: UIErrorsFrame is suppressed when
    // this client owns the errors, and this client kept drawing its own when
    // FrameXML owned them. So every refusal the server sent was still shown
    // twice, in the one direction the suppression could not reach.
    if (!frameXmlOwns(UiElement::UiErrors)) {
        renderUIErrors(gameHandler, ImGui::GetIO().DeltaTime);
    }
    toastManager_.renderEarlyToasts(ImGui::GetIO().DeltaTime, gameHandler);
    if (socialPanel_.showRaidFrames_) {
        if (!frameXmlOwns(UiElement::PartyFrames)) {
            socialPanel_.renderPartyFrames(gameHandler, chatPanel_, spellIconFn);
        }
    }
    if (!frameXmlOwns(UiElement::TargetFrame)) {
        socialPanel_.renderBossFrames(gameHandler, spellbookScreen, spellIconFn);
    }
    dialogManager_.renderDialogs(gameHandler, inventoryScreen, chatPanel_);
    // FriendsFrame is what "social" suppresses, and its tabs are the friends
    // list, the who list and the guild roster. Only the friends list was gated
    // on this side, so handing social over left two of this client's three
    // windows drawing beside FrameXML's tabs.
    if (!frameXmlOwns(UiElement::Social)) {
        socialPanel_.renderGuildRoster(gameHandler, chatPanel_, inventoryScreen,
                                       spellbookScreen, questLogScreen, spellIconFn);
    }
    if (!frameXmlOwns(UiElement::Social)) {
        socialPanel_.renderSocialFrame(gameHandler, chatPanel_);
    }
    // FrameXML's buff frame is checked as in use beside the minimap cluster,
    // and this one was drawn regardless - two bars of the same auras.
    if (!frameXmlOwns(UiElement::Buffs)) {
        combatUI_.renderBuffBar(gameHandler, spellbookScreen, inventoryScreen, settingsPanel_, spellIconFn);
    }
    windowManager_.renderGmCommandScreen(gameHandler);
    if (!frameXmlOwns(UiElement::DungeonFinder)) {
        socialPanel_.renderDungeonFinderWindow(gameHandler, chatPanel_);
    }
    windowManager_.renderInstanceLockouts(gameHandler);
    if (!frameXmlOwns(UiElement::Social)) {
        socialPanel_.renderWhoWindow(gameHandler, chatPanel_);
    }
    combatUI_.renderCombatLog(gameHandler, spellbookScreen);
    windowManager_.renderSkillsWindow(gameHandler);
    windowManager_.renderTitlesWindow(gameHandler);
    windowManager_.renderEquipSetWindow(gameHandler);
    if (!frameXmlOwns(UiElement::Inspect)) {
        socialPanel_.renderInspectWindow(gameHandler, inventoryScreen);
    }
    combatUI_.renderThreatWindow(gameHandler);
    if (!frameXmlOwns(UiElement::BattlegroundScore)) {
        combatUI_.renderBgScoreboard(gameHandler);
    }
    // The blips, whoever draws the ring around them.
    //
    // This was gated on the element the way every other pass is, and the
    // minimap is the one place that is wrong: WoW's minimap blips come from
    // the C client, not from the interface. minimap.xml declares the border,
    // the buttons, the mail and battlefield icons and the north tag, and not
    // one frame for a party member, a flight master or a corpse - so handing
    // the minimap over and standing this down left the ring drawn, the map
    // inside it drawn, and nothing on it at all.
    //
    // The pass stands down where FrameXML genuinely does own the work: its
    // input, and the mail and battlefield indicators. Both gates are inside
    // it now.
    if (showMinimap_) {
        renderMinimapMarkers(gameHandler);
    }
    windowManager_.renderLogoutCountdown(gameHandler);
    windowManager_.renderDeathScreen(gameHandler);
    windowManager_.renderReclaimCorpseButton(gameHandler);
    dialogManager_.renderLateDialogs(gameHandler);
    chatPanel_.renderBubbles(gameHandler);
    if (!frameXmlOwns(UiElement::GameMenu)) {
        windowManager_.renderEscapeMenu(settingsPanel_, gameHandler);
    }
    // Not gated on GameMenu ownership. FrameXML's own options frames are shells
    // - its menu buttons are routed here through ShowUIPanel - and this window
    // draws nothing unless something opened it, so leaving it out cost every
    // setting in it the moment the menu was handed over.
    settingsPanel_.renderSettingsWindow(chatPanel_, [this]() { saveSettings(); });
    toastManager_.renderLateToasts(gameHandler);
    renderWeatherOverlay(gameHandler);

    // Always, because this is the only thing that feeds the map its data and
    // the only thing that draws it. Skipping it when FrameXML owns the map
    // left the panel FrameXML drew with nothing in it - the half of that
    // handover that positions the map inside WorldMapDetailFrame was built and
    // the half that renders it was not. renderWorldMap decides for itself
    // whether the map is wanted; under FrameXML that is FrameXML's frame being
    // on screen rather than this client's own flag.
    renderWorldMap(gameHandler);

    if (!frameXmlOwns(UiElement::QuestLog)) {
        questLogScreen.render(gameHandler, inventoryScreen);
    }

    if (!frameXmlOwns(UiElement::Spellbook)) {
        spellbookScreen.render(gameHandler, services_.assetManager);
    }

    // Insert spell link into chat if player shift-clicked a spellbook entry
    {
        std::string pendingSpellLink = spellbookScreen.getAndClearPendingChatLink();
        if (!pendingSpellLink.empty()) {
            chatPanel_.insertChatLink(pendingSpellLink);
        }
    }

    // Talents (N key toggle handled inside)
    if (!frameXmlOwns(UiElement::Talents)) {
        talentScreen.render(gameHandler);
    }

    // Set up inventory screen asset manager + player appearance (re-init on character switch)
    {
        uint64_t activeGuid = gameHandler.getActiveCharacterGuid();
        if (activeGuid != 0 && activeGuid != inventoryScreenCharGuid_) {
            auto* am = services_.assetManager;
            if (am) {
                inventoryScreen.setAssetManager(am);
                const auto* ch = gameHandler.getActiveCharacter();
                if (ch) {
                    uint8_t skin = ch->appearanceBytes & 0xFF;
                    uint8_t face = (ch->appearanceBytes >> 8) & 0xFF;
                    uint8_t hairStyle = (ch->appearanceBytes >> 16) & 0xFF;
                    uint8_t hairColor = (ch->appearanceBytes >> 24) & 0xFF;
                    inventoryScreen.setPlayerAppearance(
                        ch->race, ch->gender, skin, face,
                        hairStyle, hairColor, ch->facialFeatures);
                    inventoryScreenCharGuid_ = activeGuid;
                }
            }
        }
    }

    // Set vendor mode before rendering inventory
    inventoryScreen.setVendorMode(gameHandler.isVendorWindowOpen(), &gameHandler);

    // Auto-open bags once when vendor window first opens
    if (gameHandler.isVendorWindowOpen()) {
        if (!windowManager_.vendorBagsOpened_) {
            windowManager_.vendorBagsOpened_ = true;
            openBagsForTrading(gameHandler, inventoryScreen);
        }
    } else {
        windowManager_.vendorBagsOpened_ = false;
    }

    // Auto-open bags once when the guild bank first opens, so items can be
    // right-clicked to deposit into the vault.
    if (gameHandler.isGuildBankOpen()) {
        if (!windowManager_.guildBankBagsOpened_) {
            windowManager_.guildBankBagsOpened_ = true;
            openBagsForTrading(gameHandler, inventoryScreen);
        }
    } else {
        windowManager_.guildBankBagsOpened_ = false;
    }

    inventoryScreen.setGameHandler(&gameHandler);
    if (!frameXmlOwns(UiElement::Bags)) {
        inventoryScreen.render(gameHandler.getInventory(), gameHandler.getMoneyCopper());
    }

    // Character screen (C key toggle handled inside render())
    if (!frameXmlOwns(UiElement::CharacterFrame)) {
        inventoryScreen.renderCharacterScreen(gameHandler);
    }

    // Item-target cursor (sharpening stone / oil awaiting the item it applies to)
    inventoryScreen.renderItemTargetCursor();

    // Insert item link into chat if player shift-clicked any inventory/equipment slot
    {
        std::string pendingLink = inventoryScreen.getAndClearPendingChatLink();
        if (!pendingLink.empty()) {
            chatPanel_.insertChatLink(pendingLink);
        }
    }

    if (inventoryScreen.consumeEquipmentDirty() || gameHandler.consumeOnlineEquipmentDirty()) {
        updateCharacterGeosets(gameHandler.getInventory());
        updateCharacterTextures(gameHandler.getInventory());
        if (appearanceComposer_) appearanceComposer_->loadEquippedWeapons();
        inventoryScreen.markPreviewDirty();
        // Update renderer weapon type for animation selection
        auto* r = services_.renderer;
        if (r) {
            const auto& mh = gameHandler.getInventory().getEquipSlot(game::EquipSlot::MAIN_HAND);
            const auto& oh = gameHandler.getInventory().getEquipSlot(game::EquipSlot::OFF_HAND);
            if (mh.empty()) {
                if (auto* ac = r->getAnimationController()) ac->setEquippedWeaponType(0, false);
            } else {
                // Polearms and staves use ATTACK_2H_LOOSE instead of ATTACK_2H
                bool is2HLoose = (mh.item.subclassName == "Polearm" || mh.item.subclassName == "Staff");
                bool isFist = (mh.item.subclassName == "Fist Weapon");
                bool isDagger = (mh.item.subclassName == "Dagger");
                bool hasOffHand = !oh.empty() &&
                    (oh.item.inventoryType == game::InvType::ONE_HAND ||
                     oh.item.subclassName == "Fist Weapon");
                bool hasShield = !oh.empty() && oh.item.inventoryType == game::InvType::SHIELD;
                if (auto* ac = r->getAnimationController()) ac->setEquippedWeaponType(mh.item.inventoryType, is2HLoose, isFist, isDagger, hasOffHand, hasShield);
            }
            // Detect ranged weapon type from RANGED slot
            const auto& rangedSlot = gameHandler.getInventory().getEquipSlot(game::EquipSlot::RANGED);
            if (rangedSlot.empty()) {
                if (auto* ac = r->getAnimationController()) ac->setEquippedRangedType(rendering::RangedWeaponType::NONE);
            } else if (rangedSlot.item.inventoryType == game::InvType::RANGED_BOW) {
                // subclassName distinguishes Bow vs Crossbow
                if (rangedSlot.item.subclassName == "Crossbow") {
                    if (auto* ac = r->getAnimationController()) ac->setEquippedRangedType(rendering::RangedWeaponType::CROSSBOW);
                } else {
                    if (auto* ac = r->getAnimationController()) ac->setEquippedRangedType(rendering::RangedWeaponType::BOW);
                }
            } else if (rangedSlot.item.inventoryType == game::InvType::RANGED_GUN) {
                // That inventory type is guns, crossbows and wands together, so
                // the subclass is what tells them apart - as it already does for
                // a crossbow above. Without this a wand was shouldered and fired
                // like a rifle.
                if (rangedSlot.item.subclassName == "Wand") {
                    if (auto* ac = r->getAnimationController()) ac->setEquippedRangedType(rendering::RangedWeaponType::WAND);
                } else if (auto* ac = r->getAnimationController()) {
                    ac->setEquippedRangedType(rendering::RangedWeaponType::GUN);
                }
            } else if (rangedSlot.item.inventoryType == game::InvType::THROWN) {
                if (auto* ac = r->getAnimationController()) ac->setEquippedRangedType(rendering::RangedWeaponType::THROWN);
            } else {
                if (auto* ac = r->getAnimationController()) ac->setEquippedRangedType(rendering::RangedWeaponType::NONE);
            }
        }
    }

    // Update renderer face-target position and selection circle
    auto* renderer = services_.renderer;
    if (renderer) {
        if (auto* ac = renderer->getAnimationController()) ac->setInCombat(gameHandler.isInCombat() &&
                              !gameHandler.isPlayerDead() &&
                              !gameHandler.isPlayerGhost());
        if (auto* cr = renderer->getCharacterRenderer()) {
            uint32_t charInstId = renderer->getCharacterInstanceId();
            if (charInstId != 0) {
                const bool isGhost = gameHandler.isPlayerGhost();
                if (!ghostOpacityStateKnown_ ||
                    ghostOpacityLastState_ != isGhost ||
                    ghostOpacityLastInstanceId_ != charInstId) {
                    cr->setInstanceOpacity(charInstId, isGhost ? 0.5f : 1.0f);
                    ghostOpacityStateKnown_ = true;
                    ghostOpacityLastState_ = isGhost;
                    ghostOpacityLastInstanceId_ = charInstId;
                }
            }
        }
        static glm::vec3 targetGLPos;
        if (gameHandler.hasTarget()) {
            auto target = gameHandler.getTarget();
            if (target) {
                // Prefer the renderer's actual instance position so the selection
                // circle tracks the rendered model (not a parallel entity-space
                // interpolator that can drift from the visual position).
                glm::vec3 instPos;
                if (core::Application::getInstance().getRenderPositionForGuid(target->getGuid(), instPos)) {
                    targetGLPos = instPos;
                    // Override Z with foot position to sit the circle on the ground.
                    float footZ = 0.0f;
                    if (core::Application::getInstance().getRenderFootZForGuid(target->getGuid(), footZ)) {
                        targetGLPos.z = footZ;
                    }
                } else {
                    // Fallback: entity game-logic position (no CharacterRenderer instance yet)
                    targetGLPos = core::coords::canonicalToRender(
                        glm::vec3(target->getX(), target->getY(), target->getZ()));
                    float footZ = 0.0f;
                    if (core::Application::getInstance().getRenderFootZForGuid(target->getGuid(), footZ)) {
                        targetGLPos.z = footZ;
                    }
                }
                if (auto* ac = renderer->getAnimationController()) ac->setTargetPosition(&targetGLPos);

                // Selection circle color: WoW-canonical level-based colors
                bool showSelectionCircle = false;
                glm::vec3 circleColor(1.0f, 1.0f, 0.3f); // default yellow
                float circleRadius = 1.5f;
                {
                    glm::vec3 boundsCenter;
                    float boundsRadius = 0.0f;
                    if (core::Application::getInstance().getRenderBoundsForGuid(target->getGuid(), boundsCenter, boundsRadius)) {
                        float r = boundsRadius * 1.1f;
                        circleRadius = std::min(std::max(r, 0.8f), 8.0f);
                    }
                }
                if (target->getType() == game::ObjectType::UNIT) {
                    showSelectionCircle = true;
                    auto unit = std::static_pointer_cast<game::Unit>(target);
                    if (unit->getHealth() == 0 && unit->getMaxHealth() > 0) {
                        circleColor = glm::vec3(0.5f, 0.5f, 0.5f); // gray (dead)
                    } else if (unit->isHostile() || gameHandler.isAggressiveTowardPlayer(target->getGuid())) {
                        const ImVec4 c = ui::helpers::levelDifficultyColor(
                            gameHandler.getPlayerLevel(), unit->getLevel());
                        circleColor = glm::vec3(c.x, c.y, c.z);
                    } else {
                        circleColor = glm::vec3(0.3f, 1.0f, 0.3f); // green (friendly)
                    }
                } else if (target->getType() == game::ObjectType::PLAYER) {
                    showSelectionCircle = true;
                    circleColor = glm::vec3(0.3f, 1.0f, 0.3f); // green (player)
                } else if (target->getType() == game::ObjectType::GAMEOBJECT) {
                    showSelectionCircle = true;
                    circleColor = glm::vec3(0.2f, 0.8f, 1.0f); // cyan (game object)
                }
                if (showSelectionCircle) {
                    renderer->setSelectionCircle(targetGLPos, circleRadius, circleColor);
                } else {
                    renderer->clearSelectionCircle();
                }
            } else {
                if (auto* ac = renderer->getAnimationController()) ac->setTargetPosition(nullptr);
                renderer->clearSelectionCircle();
            }
        } else {
            if (auto* ac = renderer->getAnimationController()) ac->setTargetPosition(nullptr);
            renderer->clearSelectionCircle();
        }
    }

    // Screen edge damage flash - red vignette that fires on HP decrease
    {
        const bool deadOrGhost = gameHandler.isPlayerDead() || gameHandler.isPlayerGhost();
        auto playerEntity = gameHandler.getEntityManager().getEntity(gameHandler.getPlayerGuid());
        uint32_t currentHp = 0;
        if (playerEntity && (playerEntity->getType() == game::ObjectType::PLAYER ||
                             playerEntity->getType() == game::ObjectType::UNIT)) {
            auto unit = std::static_pointer_cast<game::Unit>(playerEntity);
            if (unit->getMaxHealth() > 0)
                currentHp = unit->getHealth();
        }

        // Detect HP drop (ignore transitions from 0 - entity just spawned or uninitialized)
        if (!deadOrGhost && settingsPanel_.damageFlashEnabled_ &&
            lastPlayerHp_ > 0 && currentHp < lastPlayerHp_ && currentHp > 0) {
            damageFlashAlpha_ = 1.0f;
        }
        lastPlayerHp_ = currentHp;

        // Spirit release can leave a low/non-zero health value on the local
        // entity. Never carry a pre-death damage flash into ghost form.
        if (deadOrGhost) {
            damageFlashAlpha_ = 0.0f;
        }

        // Fade out over ~0.5 seconds
        if (damageFlashAlpha_ > 0.0f) {
            damageFlashAlpha_ -= ImGui::GetIO().DeltaTime * 2.0f;
            if (damageFlashAlpha_ < 0.0f) damageFlashAlpha_ = 0.0f;
            drawScreenEdgeVignette(200, 0, 0,
                                   static_cast<int>(damageFlashAlpha_ * 100.0f), 0.12f);
        }
    }

    // Persistent low-health vignette - pulsing red edges when HP < 20%
    {
        auto playerEntity = gameHandler.getEntityManager().getEntity(gameHandler.getPlayerGuid());
        const bool deadOrGhost = gameHandler.isPlayerDead() || gameHandler.isPlayerGhost();
        float hpPct = 1.0f;
        if (!deadOrGhost && playerEntity &&
            (playerEntity->getType() == game::ObjectType::PLAYER ||
             playerEntity->getType() == game::ObjectType::UNIT)) {
            auto unit = std::static_pointer_cast<game::Unit>(playerEntity);
            if (unit->getMaxHealth() > 0)
                hpPct = static_cast<float>(unit->getHealth()) / static_cast<float>(unit->getMaxHealth());
        }

        // Only show when alive and below 20% HP; intensity increases as HP drops
        if (settingsPanel_.lowHealthVignetteEnabled_ && !deadOrGhost &&
            hpPct < 0.20f && hpPct > 0.0f) {
            // Base intensity from HP deficit (0 at 20%, 1 at 0%); pulse at ~1.5 Hz
            float danger = (0.20f - hpPct) / 0.20f;
            float pulse  = 0.55f + 0.45f * std::sin(static_cast<float>(ImGui::GetTime()) * 9.4f);
            int   alpha  = static_cast<int>(danger * pulse * 90.0f);  // max ~90 alpha, subtle
            drawScreenEdgeVignette(200, 0, 0, alpha, 0.15f);
        }
    }

    // Level-up golden burst overlay
    if (toastManager_.levelUpFlashAlpha > 0.0f) {
        toastManager_.levelUpFlashAlpha -= ImGui::GetIO().DeltaTime * 1.0f;  // fade over ~1 second
        if (toastManager_.levelUpFlashAlpha < 0.0f) toastManager_.levelUpFlashAlpha = 0.0f;

        const int alpha = static_cast<int>(toastManager_.levelUpFlashAlpha * 160.0f);
        drawScreenEdgeVignette(255, 210, 50, alpha, 0.18f);

        // "Level X!" text in the center during the first half of the animation
        if (toastManager_.levelUpFlashAlpha > 0.5f && toastManager_.levelUpDisplayLevel > 0) {
            ImDrawList* fg = ImGui::GetForegroundDrawList();
            const float W = ImGui::GetIO().DisplaySize.x;
            const float H = ImGui::GetIO().DisplaySize.y;
            char lvlText[32];
            snprintf(lvlText, sizeof(lvlText), "Level %u!", toastManager_.levelUpDisplayLevel);
            ImVec2 ts = ImGui::CalcTextSize(lvlText);
            float tx = (W - ts.x) * 0.5f;
            float ty = H * 0.35f;
            // Large shadow + bright gold text
            fg->AddText(nullptr, 28.0f, ImVec2(tx + 2, ty + 2), IM_COL32(0, 0, 0, alpha), lvlText);
            fg->AddText(nullptr, 28.0f, ImVec2(tx, ty), IM_COL32(255, 230, 80, alpha), lvlText);
        }
    }

    // Restore previous alpha
    ImGui::GetStyle().Alpha = prevAlpha;
}

/// Put the item dropped on a player into the trade it opened.
///
/// CMSG_INITIATE_TRADE is a request and the window is the server's answer, so
/// the item cannot be offered at the moment of the drop. It waits here until
/// the trade is up, and is dropped if the trade never opens - a refusal, or a
/// player who walked away - rather than being offered into the next one.
void GameScreen::offerPendingTradeItem(game::GameHandler& gameHandler) {
    if (!pendingTradeItem_.active) return;
    if (!gameHandler.isTradeOpen()) {
        // Only while the request could still be answered. Leaving the world
        // ends that, and so does putting the item somewhere else.
        if (!gameHandler.isInWorld()) pendingTradeItem_ = PendingTradeItem{};
        return;
    }
    gameHandler.setTradeItem(0, pendingTradeItem_.bag, pendingTradeItem_.slot);
    pendingTradeItem_ = PendingTradeItem{};
}

void GameScreen::renderMicroMenu(game::GameHandler& gameHandler) {
    if (!settingsPanel_.pendingShowMicroMenu) return;

    ImGuiIO& io = ImGui::GetIO();
    constexpr float buttonSize = 28.0f;
    constexpr float margin = 10.0f;
    const float y = std::max(8.0f, io.DisplaySize.y - buttonSize - 18.0f);

    ImGui::SetNextWindowPos(ImVec2(margin, y), ImGuiCond_Always);
    ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoTitleBar |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoSavedSettings |
        ImGuiWindowFlags_AlwaysAutoResize;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(4.0f, 4.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(2.0f, 0.0f));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, IM_COL32(8, 8, 12, 145));
    ImGui::PushStyleColor(ImGuiCol_Border, IM_COL32(120, 130, 165, 90));

    if (ImGui::Begin("##MicroMenu", nullptr, flags)) {
        auto button = [&](const char* label, const char* tooltip, bool active) {
            if (active) {
                ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(64, 82, 132, 210));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, IM_COL32(82, 102, 160, 230));
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, IM_COL32(98, 120, 180, 245));
            }
            const bool clicked = ImGui::Button(label, ImVec2(buttonSize, buttonSize));
            if (active) ImGui::PopStyleColor(3);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tooltip);
            return clicked;
        };

        // Each of these toggles a window whose draw is gated on the same
        // ownership, so a button for a handed-over panel set a flag nothing
        // read. Only Social asked; the rest are the same shape and were
        // missing it.
        //
        // Reachable only from a hand-picked WOWEE_FRAMEXML_UI that names one
        // of these panels without naming mainmenubar. Both the defaults and
        // "candidates" include mainmenubar, which covers MicroMenu - so this
        // whole menu is gated off there and none of these buttons is drawn at
        // all. I claimed otherwise when writing this and was wrong: the group
        // cover in coveredByGroup is easy to read past, and "bags is a default
        // element" is true without meaning what it looks like it means.
        if (button("C##MicroCharacter", "Character", inventoryScreen.isCharacterOpen())) {
            if (frameXmlOwns(UiElement::CharacterFrame)) {
                gameHandler.runInterfaceCommand("ToggleCharacter(\"PaperDollFrame\")");
            } else {
                const bool wasOpen = inventoryScreen.isCharacterOpen();
                inventoryScreen.toggleCharacter();
                if (!wasOpen && gameHandler.isConnected()) gameHandler.requestPlayedTime();
            }
        }
        ImGui::SameLine();
        if (button("B##MicroBags", "Backpack", inventoryScreen.isBackpackOpen())) {
            if (frameXmlOwns(UiElement::Bags)) gameHandler.runInterfaceCommand("ToggleBackpack()");
            else                               inventoryScreen.toggleBackpack();
        }
        ImGui::SameLine();
        if (button("P##MicroSpellbook", "Spellbook", spellbookScreen.isOpen())) {
            if (frameXmlOwns(UiElement::Spellbook))
                gameHandler.runInterfaceCommand("ToggleSpellBook(BOOKTYPE_SPELL)");
            else
                spellbookScreen.toggle();
        }
        ImGui::SameLine();
        if (button("N##MicroTalents", "Talents", talentScreen.isOpen())) {
            if (frameXmlOwns(UiElement::Talents)) gameHandler.runInterfaceCommand("ToggleTalentFrame()");
            else                                  talentScreen.toggle();
        }
        ImGui::SameLine();
        if (button("L##MicroQuests", "Quest Log", questLogScreen.isOpen())) {
            if (frameXmlOwns(UiElement::QuestLog))
                gameHandler.runInterfaceCommand("ToggleFrame(QuestLogFrame)");
            else
                questLogScreen.toggle();
        }
        ImGui::SameLine();
        // Same as the K key does, which already routes: in FrameXML the skills
        // list is a tab of the character sheet rather than a window of its own.
        // The key and the button opened two different things.
        if (button("K##MicroSkills", "Skills", windowManager_.showSkillsWindow_)) {
            if (frameXmlOwns(UiElement::CharacterFrame))
                gameHandler.runInterfaceCommand("ToggleCharacter(\"SkillFrame\")");
            else
                windowManager_.showSkillsWindow_ = !windowManager_.showSkillsWindow_;
        }
        ImGui::SameLine();
        if (button("O##MicroSocial", "Social", socialPanel_.showSocialFrame_)) {
            if (frameXmlOwns(UiElement::Social)) {
                gameHandler.runInterfaceCommand("ToggleFriendsFrame(1)");
            } else {
                socialPanel_.showSocialFrame_ = !socialPanel_.showSocialFrame_;
            }
        }
        ImGui::SameLine();
        // Not routed, and there is nothing to route it to: FrameXML has no
        // toggle for party frames, which simply appear while there is a party.
        // With that element handed over this flips a flag whose only reader is
        // gated off, which is a button that does nothing - but calling
        // something that does not exist would be worse, and the frames are
        // already on screen at that point.
        if (button("G##MicroGroup", "Party/Raid Frames", socialPanel_.showRaidFrames_)) {
            socialPanel_.showRaidFrames_ = !socialPanel_.showRaidFrames_;
        }
        ImGui::SameLine();
        if (button("M##MicroMap", "World Map", showWorldMap_)) {
            if (frameXmlOwns(UiElement::WorldMap))
                gameHandler.runInterfaceCommand("ToggleFrame(WorldMapFrame)");
            else
                showWorldMap_ = !showWorldMap_;
        }
        ImGui::SameLine();
        if (button("GM##MicroGM", "GM Commands", windowManager_.showGmCommandScreen_)) {
            windowManager_.showGmCommandScreen_ = !windowManager_.showGmCommandScreen_;
        }
        ImGui::SameLine();
        if (button("*##MicroSettings", "Settings", settingsPanel_.showSettingsWindow)) {
            // To the one settings screen, as the escape menu's Settings button
            // does. See the note there.
            if (frameXmlOwns(UiElement::GameMenu)) {
                gameHandler.runInterfaceCommand("VideoOptionsFrame_Toggle()");
            } else {
                settingsPanel_.showSettingsWindow = !settingsPanel_.showSettingsWindow;
            }
        }
    }

    ImGui::End();
    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar(2);
}

void GameScreen::renderPlayerInfo(game::GameHandler& gameHandler) {
    ImGui::SetNextWindowSize(ImVec2(350, 250), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(10, 30), ImGuiCond_FirstUseEver);
    ImGui::Begin("Player Info", &showPlayerInfo);

    const auto& movement = gameHandler.getMovementInfo();

    ImGui::Text("Position & Movement");
    ImGui::Separator();
    ImGui::Spacing();

    // Position
    ImGui::Text("Position:");
    ImGui::Indent();
    ImGui::Text("X: %.2f", movement.x);
    ImGui::Text("Y: %.2f", movement.y);
    ImGui::Text("Z: %.2f", movement.z);
    ImGui::Text("Orientation: %.2f rad (%.1f deg)", movement.orientation, movement.orientation * 180.0f / core::coords::PI);
    ImGui::Unindent();

    ImGui::Spacing();

    // Movement flags
    ImGui::Text("Movement Flags: 0x%08X", movement.flags);
    ImGui::Text("Time: %u ms", movement.time);

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Connection state
    ImGui::Text("Connection State:");
    ImGui::Indent();
    auto state = gameHandler.getState();
    switch (state) {
        case game::WorldState::IN_WORLD:
            ImGui::TextColored(kColorBrightGreen, "In World");
            break;
        case game::WorldState::AUTHENTICATED:
            ImGui::TextColored(kColorYellow, "Authenticated");
            break;
        case game::WorldState::ENTERING_WORLD:
            ImGui::TextColored(kColorYellow, "Entering World...");
            break;
        default:
            ImGui::TextColored(kColorRed, "State: %d", static_cast<int>(state));
            break;
    }
    ImGui::Unindent();

    ImGui::End();
}

void GameScreen::renderEntityList(game::GameHandler& gameHandler) {
    ImGui::SetNextWindowSize(ImVec2(500, 400), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(10, 290), ImGuiCond_FirstUseEver);
    ImGui::Begin("Entities", &showEntityWindow);

    const auto& entityManager = gameHandler.getEntityManager();
    const auto& entities = entityManager.getEntities();

    ImGui::Text("Entities in View: %zu", entities.size());
    ImGui::Separator();
    ImGui::Spacing();

    if (entities.empty()) {
        ImGui::TextDisabled("No entities in view");
    } else {
        // Entity table
        if (ImGui::BeginTable("EntitiesTable", 5, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY)) {
            ImGui::TableSetupColumn("GUID", ImGuiTableColumnFlags_WidthFixed, 140.0f);
            ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed, 100.0f);
            ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("Position", ImGuiTableColumnFlags_WidthFixed, 150.0f);
            ImGui::TableSetupColumn("Distance", ImGuiTableColumnFlags_WidthFixed, 80.0f);
            ImGui::TableHeadersRow();

            const auto& playerMovement = gameHandler.getMovementInfo();
            float playerX = playerMovement.x;
            float playerY = playerMovement.y;
            float playerZ = playerMovement.z;

            for (const auto& [guid, entity] : entities) {
                ImGui::TableNextRow();

                // GUID
                ImGui::TableSetColumnIndex(0);
                char guidStr[24];
                snprintf(guidStr, sizeof(guidStr), "0x%016llX", (unsigned long long)guid);
                ImGui::Text("%s", guidStr);

                // Type
                ImGui::TableSetColumnIndex(1);
                switch (entity->getType()) {
                    case game::ObjectType::PLAYER:
                        ImGui::TextColored(kColorBrightGreen, "Player");
                        break;
                    case game::ObjectType::UNIT:
                        ImGui::TextColored(kColorYellow, "Unit");
                        break;
                    case game::ObjectType::GAMEOBJECT:
                        ImGui::TextColored(ImVec4(0.3f, 0.8f, 1.0f, 1.0f), "GameObject");
                        break;
                    default:
                        ImGui::Text("Object");
                        break;
                }

                // Name (for players and units)
                ImGui::TableSetColumnIndex(2);
                if (entity->getType() == game::ObjectType::PLAYER) {
                    auto player = std::static_pointer_cast<game::Player>(entity);
                    ImGui::Text("%s", player->getName().c_str());
                } else if (entity->getType() == game::ObjectType::UNIT) {
                    auto unit = std::static_pointer_cast<game::Unit>(entity);
                    if (!unit->getName().empty()) {
                        ImGui::Text("%s", unit->getName().c_str());
                    } else {
                        ImGui::TextDisabled("--");
                    }
                } else {
                    ImGui::TextDisabled("--");
                }

                // Position
                ImGui::TableSetColumnIndex(3);
                ImGui::Text("%.1f, %.1f, %.1f", entity->getX(), entity->getY(), entity->getZ());

                // Distance from player
                ImGui::TableSetColumnIndex(4);
                float dx = entity->getX() - playerX;
                float dy = entity->getY() - playerY;
                float dz = entity->getZ() - playerZ;
                float distance = std::sqrt(dx*dx + dy*dy + dz*dz);
                ImGui::Text("%.1f", distance);
            }

            ImGui::EndTable();
        }
    }

    ImGui::End();
}

void GameScreen::processTargetInput(game::GameHandler& gameHandler) {
    auto& io = ImGui::GetIO();
    auto& input = core::Input::getInstance();

    // If the user is typing (or about to focus chat this frame), do not allow
    // A-Z or 1-0 shortcuts to fire.
    // Enter and slash open the chat box. Which chat box depends on who owns it.
    //
    // These reached this client's panel unconditionally, and its render is
    // gated on the same ownership - so with the chat handed over they set a
    // focus flag on a panel that is never drawn, and the interface's own edit
    // box stayed shut. There was no other way in: FrameXML's OPENCHAT and
    // OPENCHATSLASH bindings are not in this client's route table, so with the
    // chat owned there was no key at all that opened it.
    //
    // The keystrokes that follow do need a guard, and the reason they were
    // thought not to is worth keeping: the application's key loop does hand
    // FrameXML's focused box every press and stop there - but only on the
    // event path. What is below reads the key state directly, once a frame,
    // and a poll never went through that loop to be stopped by it. So typing
    // did walk the character and did fire bindings, and the one question both
    // paths now ask is interfaceTakingTypedInput.
    const bool frameXmlChat = frameXmlOwns(UiElement::Chat);
    if (!io.WantTextInput && !interfaceTakingTypedInput() &&
        !chatPanel_.isChatInputActive() && input.isKeyJustPressed(SDL_SCANCODE_SLASH)) {
        if (frameXmlChat) gameHandler.runInterfaceCommand("ChatFrame_OpenChat(\"/\")");
        else              chatPanel_.activateSlashInput();
    }
    // The same one line as Escape, for the same reason: the chain reads sound
    // and the key does nothing, so what is wanted is which branch ran. The
    // guards are reported too, because being refused before the branch is the
    // likeliest answer and is the one that leaves no other trace.
    if (KeybindingManager::getInstance().isActionPressed(KeybindingManager::Action::TOGGLE_CHAT, false)) {
        if (io.WantTextInput || interfaceTakingTypedInput() ||
            interfaceConsumedKey(ImGuiKey_Enter) ||
            chatPanel_.isChatInputActive()) {
            LOG_WARNING("Chat key: refused - ImGui wants text: ",
                     io.WantTextInput ? "yes" : "no",
                     ", the interface's box has focus: ",
                     interfaceTakingTypedInput() ? "yes" : "no",
                     ", the interface's box already took this press: ",
                     interfaceConsumedKey(ImGuiKey_Enter) ? "yes" : "no",
                     ", this client's chat input: ",
                     chatPanel_.isChatInputActive() ? "active" : "idle");
        } else {
            LOG_WARNING("Chat key: opening ",
                     frameXmlChat ? "the interface's box" : "this client's box");
        }
    }
    // The interface's box gets this press first when it has focus, and Enter
    // is where it says it is done: ChatEdit_OnEnterPressed sends the line and
    // ends in ChatEdit_OnEscapePressed, which hides the box. That is also what
    // clears the focus this poll's guard reads - so the press that sent the
    // message opened the box again behind it.
    if (!io.WantTextInput && !chatPanel_.isChatInputActive() &&
        !interfaceConsumedKey(ImGuiKey_Enter) &&
        KeybindingManager::getInstance().isActionPressed(KeybindingManager::Action::TOGGLE_CHAT, false)) {
        if (frameXmlChat) gameHandler.runInterfaceCommand("ChatFrame_OpenChat(\"\")");
        else              chatPanel_.activateInput();
    }

    // Anything at all is taking typed input: this client's chat box, an ImGui
    // field, or one of the interface's own edit boxes.
    //
    // That third one was left out, and the block of game hotkeys further down
    // asks only this. So every one of them fired while the player was typing
    // into a FrameXML box - H opened the titles window from the middle of a
    // mail recipient, C opened the character sheet, I the bags, and the digits
    // fired action bar slots. The two places that had noticed named the
    // interface's box separately, which is how the third came to be missing
    // from the rest.
    const bool textFocus = chatPanel_.isChatInputActive() || io.WantTextInput ||
                           interfaceTakingTypedInput();

    // Game hotkeys - gate on textFocus (chat/text-input active) rather than
    // WantCaptureKeyboard so that toggle keys like M, C, I still work when an
    // ImGui window (character panel, map, etc.) happens to have focus.
    {
        // Two guards, because Tab reaches a focused box two ways. While one
        // holds focus the probe answers; when the box's own OnTabPressed moves
        // to the next field, the box that had it has let go by the time this
        // is asked - and cycling between two fields of a form would have
        // changed the player's target on every press.
        if (!textFocus &&
            !interfaceConsumedKey(ImGuiKey_Tab) &&
            input.isKeyJustPressed(SDL_SCANCODE_TAB)) {
            const auto& movement = gameHandler.getMovementInfo();
            gameHandler.tabTarget(movement.x, movement.y, movement.z);
        }

        // Escape (TOGGLE_SETTINGS) must not fire while chat input is active -
        // otherwise pressing Escape to close chat also closes any open window or
        // opens the escape menu, since ImGui deactivates InputText on Escape but
        // the same press still propagates here. KeybindingManager only blocks
        // A-Z/0-9 during text input, not Escape.
        // Said out loud, at info, because this chain has now been read end to
        // end five times without the fault appearing in it. Every link checks
        // out on paper and the key still does nothing, which means the answer
        // is which branch actually runs - and that is the one thing reading
        // cannot tell you. One line per press, invisible unless someone asks
        // for info, and it names the branch rather than the key.
        //
        // Asked of the key rather than of the binding, deliberately. The
        // binding declines to fire while either interface is taking typed
        // input, so routing this through it means a press that was swallowed
        // for that reason produces no line - which reads exactly like a press
        // that never arrived, and those are the two remaining explanations.
        // Every press of the bound key now says something.
        const ImGuiKey escapeKey = KeybindingManager::getInstance().getKeyForAction(
            KeybindingManager::Action::TOGGLE_SETTINGS);
        const bool escapePressed =
            escapeKey != ImGuiKey_None && ImGui::IsKeyPressed(escapeKey, true);
        // Nothing bound is its own explanation and looks identical to every
        // other one from a log: no line at all. The keybindings are read from a
        // config file that can rebind or clear this, and setKeyForAction turns
        // a movement key into ImGuiKey_None outright - so "the action has no
        // key" is a state the client can genuinely be in, and it would make
        // Escape do nothing while every branch below remains correct.
        //
        // Said once rather than per frame, since it is a fact about the
        // configuration and not about a press.
        if (escapeKey == ImGuiKey_None) {
            static bool saidUnbound = false;
            if (!saidUnbound) {
                saidUnbound = true;
                LOG_WARNING("Escape: nothing is bound to the game-menu action, "
                            "so no press can reach the chain");
            }
        }
        if (escapePressed && textFocus) {
            LOG_WARNING("Escape: swallowed before the chain - chat input ",
                     chatPanel_.isChatInputActive() ? "active" : "idle",
                     ", ImGui wants text: ", io.WantTextInput ? "yes" : "no",
                     ", the interface's box has focus: ",
                     interfaceTakingTypedInput() ? "yes" : "no");
        }
        if (!textFocus &&
            KeybindingManager::getInstance().isActionPressed(KeybindingManager::Action::TOGGLE_SETTINGS, true)) {
            // Gathered, then decided, then done - rather than decided while
            // being done. The order of these branches is the whole of what
            // Escape means, and as a chain of else-if inside a draw there was
            // no way to ask what the key would do without being in the
            // situation. resolveEscape is that question on its own, and its
            // test states each situation directly.
            EscapeState st;
            st.interfaceConsumedKey  = interfaceConsumedKey(ImGuiKey_Escape);
            st.settingsWindowShown   = settingsPanel_.showSettingsWindow;
            // Either cursor. With the bags handed over an item is picked up
            // through FrameXML's, and Escape has to put that one down too -
            // this read only the bag window's, so Escape opened the menu with
            // an item still stuck to the pointer.
            st.holdingItem           = inventoryScreen.isHoldingItem() ||
                                       !frameXmlCursorItem().empty();
            st.clientMenuShown       = windowManager_.showEscapeMenu;
            st.casting               = gameHandler.isCasting();
            st.lootOpen              = gameHandler.isLootWindowOpen();
            st.gossipOpen            = gameHandler.isGossipWindowOpen();
            st.vendorOpen            = gameHandler.isVendorWindowOpen();
            st.barberShopOpen        = gameHandler.isBarberShopOpen();
            st.bankOpen              = gameHandler.isBankOpen();
            st.guildBankOpen         = gameHandler.isGuildBankOpen();
            st.trainerOpen           = gameHandler.isTrainerWindowOpen();
            st.mailboxOpen           = gameHandler.isMailboxOpen();
            st.auctionHouseOpen      = gameHandler.isAuctionHouseOpen();
            st.questDetailsOpen      = gameHandler.isQuestDetailsOpen();
            st.questOfferRewardOpen  = gameHandler.isQuestOfferRewardOpen();
            st.questRequestItemsOpen = gameHandler.isQuestRequestItemsOpen();
            st.tradeOpen             = gameHandler.isTradeOpen();

            const EscapeAction action = resolveEscape(st);
            // At warning, like the three in the pump it pairs with.
            //
            // This chain has been read end to end more times than any other in
            // the client and every link checks out on paper; what is missing is
            // which branch actually runs, and that is the one thing reading
            // cannot tell you. It was said at info, and the log a report
            // arrives with is warnings only - so a session that reproduced
            // "Escape does nothing" came back with no Escape line in it at all,
            // and silence there meant nothing, because it is also exactly what
            // a working press sounds like.
            //
            // One line per press, and Escape is not a key anyone holds down.
            LOG_WARNING("Escape: ", escapeActionName(action),
                        " (the interface owns the game menu: ",
                        frameXmlOwns(UiElement::GameMenu) ? "yes" : "no", ")");
            switch (action) {
                case EscapeAction::CloseSettingsWindow:
                    settingsPanel_.showSettingsWindow = false;
                    break;
                case EscapeAction::CloseClientMenu:
                    windowManager_.showEscapeMenu = false;
                    settingsPanel_.showEscapeSettingsNotice = false;
                    break;
                case EscapeAction::CancelCast:             gameHandler.cancelCast(); break;
                case EscapeAction::CloseLoot:              gameHandler.closeLoot(); break;
                case EscapeAction::CloseGossip:            gameHandler.closeGossip(); break;
                case EscapeAction::ReturnHeldItem:
                    if (inventoryScreen.isHoldingItem())
                        inventoryScreen.returnHeldItem(gameHandler.getInventory());
                    else
                        frameXmlPutCursorDown();
                    break;
                case EscapeAction::CloseVendor:            gameHandler.closeVendor(); break;
                case EscapeAction::CloseBarberShop:        gameHandler.closeBarberShop(); break;
                case EscapeAction::CloseBank:              gameHandler.closeBank(); break;
                case EscapeAction::CloseGuildBank:         gameHandler.closeGuildBank(); break;
                case EscapeAction::CloseTrainer:           gameHandler.closeTrainer(); break;
                case EscapeAction::CloseMailbox:           gameHandler.closeMailbox(); break;
                case EscapeAction::CloseAuctionHouse:      gameHandler.closeAuctionHouse(); break;
                case EscapeAction::DeclineQuest:           gameHandler.declineQuest(); break;
                case EscapeAction::CloseQuestOfferReward:  gameHandler.closeQuestOfferReward(); break;
                case EscapeAction::CloseQuestRequestItems: gameHandler.closeQuestRequestItems(); break;
                case EscapeAction::CancelTrade:            gameHandler.cancelTrade(); break;
                case EscapeAction::None: break;
                case EscapeAction::AskTheInterface: {
                    // Asked only here, because asking closes things. Everything
                    // above is a window the *server* knows about and each has
                    // to go through the client so the closing packet is sent -
                    // CloseAllWindows would hide the frame and leave the server
                    // believing the vendor was still open.
                    //
                    // In ToggleGameMenu's order, stopping at the first that
                    // answers. A dropdown open over a panel has to go before
                    // the panel does, or Escape closes the window out from
                    // under the menu; the special windows are the list addons
                    // add themselves to, and nothing has ever walked it.
                    // Which frame answered, before asking anything to close.
                    // Escape reported closing a panel on every press with
                    // nothing open, so the menu never opened - and the chain
                    // can only say that something was shown, not what. Named
                    // once, because a frame that is always shown is the bug
                    // rather than the closing.
                    static bool namedTheOpenFrame = false;
                    if (!namedTheOpenFrame) {
                        namedTheOpenFrame = true;
                        gameHandler.askInterface(
                            "local n = {} "
                            "for _, v in pairs(UISpecialFrames or {}) do "
                            "  local f = _G[v] "
                            "  if f and f.IsShown and f:IsShown() then n[#n+1] = v end "
                            "end "
                            "if #n > 0 then "
                            "  print('Escape: UISpecialFrames already shown: ' "
                            "        .. table.concat(n, ', ')) "
                            "else print('Escape: no UISpecialFrame is shown') end "
                            "return false");
                    }

                    const bool closed = gameHandler.askInterface(
                        "(CloseMenus and CloseMenus()) or "
                        "(CloseSpecialWindows and CloseSpecialWindows()) or "
                        "(CloseAllWindows and CloseAllWindows()) or false");
                    const EscapeOutcome outcome = resolveAfterInterface(
                        closed, frameXmlOwns(UiElement::GameMenu));
                    // At warning, because this is the press the report is
                    // about and the default log is warnings only.
                    //
                    // Every other branch of this chain speaks at info, so a
                    // session that reproduces "Escape does nothing" came back
                    // with no Escape line in it at all - and silence there
                    // meant nothing, since it is also what a working press
                    // sounds like. One line per press that gets this far, and
                    // it names which of the two menus was chosen. If that line
                    // is absent from a log where Escape was pressed with
                    // nothing open, the key never reached this chain and the
                    // fault is in the input path rather than here.
                    if (outcome == EscapeOutcome::InterfaceClosedAPanel) {
                        LOG_INFO("Escape: ", escapeOutcomeName(outcome));
                    } else {
                        LOG_WARNING("Escape: ", escapeOutcomeName(outcome));
                    }
                    switch (outcome) {
                        case EscapeOutcome::InterfaceClosedAPanel:
                            break;
                        case EscapeOutcome::ToggleInterfaceMenu:
                            // The interface's own way in, and the same function
                            // its own Escape binding calls. This branch used to
                            // set the flag behind *this* client's menu, and
                            // that menu is only drawn while the element is not
                            // handed over - so with it handed over, Escape set
                            // a flag nobody read and nothing appeared.
                            gameHandler.runInterfaceCommand("ToggleGameMenu()");
                            // And whether it worked, which the line above
                            // cannot say. Asked straight afterwards so the two
                            // faults separate: shown but not on screen is a
                            // drawing problem, not shown is a problem in
                            // ToggleGameMenu - which runs clean headlessly.
                            //
                            // At warning when it did not work, because that is
                            // the one outcome nobody would think to look for.
                            // The whole of this chain has been read seven
                            // times and every link checks out; the interface
                            // side is settled too - headlessly ToggleGameMenu
                            // shows GameMenuFrame at 195x240, visible, alpha
                            // one, parented to UIParent. So the answer is in
                            // the running client, and a line only the info
                            // level shows is a line nobody has. This says it
                            // where the default log will keep it.
                            if (gameHandler.askInterface(
                                    "GameMenuFrame and GameMenuFrame:IsShown()")) {
                                LOG_INFO("Escape: the interface's menu is now shown");
                            } else {
                                LOG_WARNING(
                                    "Escape: asked the interface for its game "
                                    "menu and GameMenuFrame is still not shown "
                                    "- ToggleGameMenu ran and left it hidden");
                            }
                            break;
                        case EscapeOutcome::OpenClientMenu:
                            windowManager_.showEscapeMenu = true;
                            break;
                    }
                    break;
                }
            }
        }

        if (!textFocus) {
            // Toggle character screen (C) and inventory/bags (I)
            if (KeybindingManager::getInstance().isActionPressed(KeybindingManager::Action::TOGGLE_CHARACTER_SCREEN)) {
                // Whichever interface owns the character sheet is the one the
                // key has to reach. Toggling this client's own state while
                // FrameXML draws the window left the key doing nothing
                // visible at all.
                if (frameXmlOwns(UiElement::CharacterFrame)) {
                    // Nothing, deliberately: the handover route table in
                    // application.cpp already calls ToggleCharacter for this
                    // key. Calling it here as well was the whole of the bug
                    // reported three times. IsKeyPressed does not consume, so
                    // both sites saw the same press and toggled in the same
                    // frame - open, then shut, and nothing on screen. Every
                    // link in the chain read correct because the chain was;
                    // it simply ran twice.
                    //
                    // The other two handovers written here hid the same fault
                    // by being broken: ToggleAllBags and ToggleWorldMap do not
                    // exist in 3.3.5, so those calls did nothing and the route
                    // table's single call was left to work.
                } else {
                    inventoryScreen.toggleCharacter();
                }
                if (gameHandler.isConnected()) gameHandler.requestPlayedTime();
            }

            if (KeybindingManager::getInstance().isActionPressed(KeybindingManager::Action::TOGGLE_INVENTORY)) {
                if (frameXmlOwns(UiElement::Bags)) {
                    // ToggleAllBags is a later addition and is not in this
                    // FrameXML, so this key did nothing at all once the bags
                    // were handed over. OpenAllBags is 3.3.5's own name for
                    // it, and it toggles rather than only opening.
                    gameHandler.runInterfaceCommand("OpenAllBags()");
                } else {
                    inventoryScreen.toggle();
                }
            }

            if (KeybindingManager::getInstance().isActionPressed(KeybindingManager::Action::TOGGLE_NAMEPLATES)) {
                if (ImGui::GetIO().KeyShift)
                    settingsPanel_.showFriendlyNameplates_ = !settingsPanel_.showFriendlyNameplates_;
                else
                    showNameplates_ = !showNameplates_;
            }

            if (KeybindingManager::getInstance().isActionPressed(KeybindingManager::Action::TOGGLE_WORLD_MAP)) {
                if (frameXmlOwns(UiElement::WorldMap)) {
                    // Nothing: the route table owns this key too. The call
                    // written here named ToggleWorldMap, which 3.3.5 does not
                    // have, so it silently did nothing and left the map
                    // working on the route table's single toggle.
                } else {
                    showWorldMap_ = !showWorldMap_;
                }
            }

            if (KeybindingManager::getInstance().isActionPressed(KeybindingManager::Action::TOGGLE_MINIMAP)) {
                showMinimap_ = !showMinimap_;
                // This flag gates this client's marker pass, which is drawn
                // over FrameXML's minimap on purpose rather than instead of it
                // - so on its own the key hid the markers and left the minimap
                // underneath them up. ToggleMinimap is what the interface's own
                // TOGGLEMINIMAP binding calls, and it hides the frame the
                // markers sit on, which is what makes the two agree.
                if (frameXmlOwns(UiElement::Minimap)) {
                    gameHandler.runInterfaceCommand("ToggleMinimap()");
                }
            }

            if (KeybindingManager::getInstance().isActionPressed(KeybindingManager::Action::TOGGLE_RAID_FRAMES)) {
                socialPanel_.showRaidFrames_ = !socialPanel_.showRaidFrames_;
            }

            if (KeybindingManager::getInstance().isActionPressed(KeybindingManager::Action::TOGGLE_ACHIEVEMENTS)) {
                gameHandler.runInterfaceCommand("ToggleAchievementFrame()");
            }
            if (KeybindingManager::getInstance().isActionPressed(KeybindingManager::Action::TOGGLE_SKILLS)) {
                // The skills list is a tab of the character sheet in FrameXML
                // rather than a window of its own.
                if (frameXmlOwns(UiElement::CharacterFrame)) {
                    gameHandler.runInterfaceCommand("ToggleCharacter(\"SkillFrame\")");
                } else {
                    windowManager_.showSkillsWindow_ = !windowManager_.showSkillsWindow_;
                }
            }

            // Toggle Titles window with H (hero/title screen - no conflicting keybinding)
            if (input.isKeyJustPressed(SDL_SCANCODE_H)) {
                windowManager_.showTitlesWindow_ = !windowManager_.showTitlesWindow_;
            }

            // Screenshot (PrintScreen key)
            if (input.isKeyJustPressed(SDL_SCANCODE_PRINTSCREEN)) {
                takeScreenshot();
            }

            // Action bar keys (1-9, 0, -, =)
            static const SDL_Scancode actionBarKeys[] = {
                SDL_SCANCODE_1, SDL_SCANCODE_2, SDL_SCANCODE_3, SDL_SCANCODE_4,
                SDL_SCANCODE_5, SDL_SCANCODE_6, SDL_SCANCODE_7, SDL_SCANCODE_8,
                SDL_SCANCODE_9, SDL_SCANCODE_0, SDL_SCANCODE_MINUS, SDL_SCANCODE_EQUALS
            };
            const bool shiftDown = input.isKeyPressed(SDL_SCANCODE_LSHIFT) || input.isKeyPressed(SDL_SCANCODE_RSHIFT);
            const bool ctrlDown  = input.isKeyPressed(SDL_SCANCODE_LCTRL)  || input.isKeyPressed(SDL_SCANCODE_RCTRL);
            const auto& bar = gameHandler.getActionBar();

            // Ctrl+1..Ctrl+8 → switch stance/form/presence (WoW default bindings).
            // Only fires for classes that use a stance bar; same slot ordering as
            // renderStanceBar: Warrior, DK, Druid, Rogue, Priest.
            if (ctrlDown) {
                // The list the bar is drawn from, so key N presses the form
                // in position N rather than the Nth of a differently ordered
                // table - which is what a second copy of these forms caused.
                const auto forms = game::knownShapeshiftForms(
                    gameHandler.getPlayerClass(), gameHandler.getKnownSpells());
                if (!forms.empty()) {
                    std::vector<uint32_t> avail;
                    avail.reserve(forms.size());
                    for (const auto& form : forms) avail.push_back(form.spellId);
                    // Ctrl+1 = first stance, Ctrl+2 = second, …
                    for (int i = 0; i < static_cast<int>(avail.size()) && i < 8; ++i) {
                        if (input.isKeyJustPressed(actionBarKeys[i]))
                            gameHandler.castSpell(avail[i]);
                    }
                }
            }

            for (int i = 0; i < game::GameHandler::SLOTS_PER_BAR; ++i) {
                if (!ctrlDown && input.isKeyJustPressed(actionBarKeys[i])) {
                    int slotIdx = shiftDown
                        ? ActionBarPanel::actionSlotForPage(ActionBarPanel::kBottomLeftActionPage, i)
                        : ActionBarPanel::actionSlotForPage(actionBarPanel_.getMainActionBarPage(), i);
                    if (bar[slotIdx].type == game::ActionBarSlot::SPELL && bar[slotIdx].isReady()) {
                        uint64_t target = gameHandler.hasTarget() ? gameHandler.getTargetGuid() : 0;
                        gameHandler.castSpell(bar[slotIdx].id, target);
                    } else if (bar[slotIdx].type == game::ActionBarSlot::ITEM && bar[slotIdx].id != 0) {
                        gameHandler.useItemById(bar[slotIdx].id);
                    } else if (bar[slotIdx].type == game::ActionBarSlot::MACRO) {
                        chatPanel_.executeMacroText(gameHandler, gameHandler.getMacroText(bar[slotIdx].id));
                    }
                }
            }
        }

    }

    // Cursor affordance: show hand cursor over interactable entities.
    // Not while the cursor is over a frame FrameXML owns: ImGui has never heard
    // of those, so its own answer is no wherever they are.
    if (!io.WantCaptureMouse && !frameXmlOwnsMouse()) {
        auto* renderer = services_.renderer;
        auto* camera = renderer ? renderer->getCamera() : nullptr;
        auto* window = services_.window;
        if (camera && window) {
            glm::vec2 mousePos = input.getMousePosition();
            float screenW = static_cast<float>(window->getWidth());
            float screenH = static_cast<float>(window->getHeight());
            rendering::Ray ray = camera->screenToWorldRay(mousePos.x, mousePos.y, screenW, screenH);
            // The same picker the click uses, so the cursor affordance cannot
            // disagree with what clicking would actually select - including the
            // tighter sphere critters get, which this copy did not have.
            const ui::ScenePick hoverPick =
                ui::pickScene(gameHandler, ray, ui::ScenePickParams{});
            if (hoverPick.closestGuid != 0) {
                ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
            }
        }
    }

    // Left-click targeting: only on mouse-up if the mouse didn't drag (camera rotate)
    // Record press position on mouse-down
    if (!io.WantCaptureMouse && !frameXmlOwnsMouse() &&
        input.isMouseButtonJustPressed(SDL_BUTTON_LEFT) && !input.isMouseButtonPressed(SDL_BUTTON_RIGHT)) {
        leftClickPressPos_ = input.getMousePosition();
        leftClickWasPress_ = true;
    }

    // On mouse-up, check if it was a click (not a drag)
    if (leftClickWasPress_ && input.isMouseButtonJustReleased(SDL_BUTTON_LEFT)) {
        leftClickWasPress_ = false;
        glm::vec2 releasePos = input.getMousePosition();
        glm::vec2 dragDelta = releasePos - leftClickPressPos_;
        float dragDistSq = glm::dot(dragDelta, dragDelta);
        const float CLICK_THRESHOLD = clickDragThreshold();

        if (dragDistSq < CLICK_THRESHOLD * CLICK_THRESHOLD) {
            auto* renderer = services_.renderer;
            auto* camera = renderer ? renderer->getCamera() : nullptr;
            auto* window = services_.window;

            if (camera && window) {
                float screenW = static_cast<float>(window->getWidth());
                float screenH = static_cast<float>(window->getHeight());

                rendering::Ray ray = camera->screenToWorldRay(leftClickPressPos_.x, leftClickPressPos_.y, screenW, screenH);

                // Use the same authoritative position and forgiving click volume
                // as right-click for a hooked bobber. Its tiny, partly submerged
                // render bounds must not let a click fall through to a character.
                const uint64_t hookedBobber = gameHandler.getHookedFishingBobberGuid();
                if (hookedBobber != 0) {
                    auto bobber = gameHandler.getEntityManager().getEntity(hookedBobber);
                    if (bobber && bobber->getType() == game::ObjectType::GAMEOBJECT) {
                        glm::vec3 bobberCenter = core::coords::canonicalToRender(
                            glm::vec3(bobber->getX(), bobber->getY(), bobber->getZ()));
                        bobberCenter.z += 0.35f;
                        float hitT = 0.0f;
                        constexpr float kFishingBobberClickRadius = 4.0f;
                        if (raySphereIntersect(ray, bobberCenter, kFishingBobberClickRadius, hitT)) {
                            LOG_WARNING("Fishing bobber direct left click: guid=0x", std::hex,
                                        hookedBobber, std::dec, " hitDistance=", hitT);
                            gameHandler.interactWithGameObject(hookedBobber);
                            return;
                        }
                    }
                }

                const ui::ScenePick pick = ui::pickScene(
                    gameHandler, ray, ui::ScenePickParams{});
                uint64_t closestGuid = pick.resolve();

                // Dropping what the cursor is carrying onto another player
                // opens a trade with them, which is how a trade begins in WoW
                // and was reachable here only by typing /trade.
                // Either cursor again: the item being offered usually comes
                // from FrameXML's bags now, and asking only the bag window's
                // meant a drag onto a player did nothing.
                const bool carrying = inventoryScreen.isHoldingItem() ||
                                      !frameXmlCursorItem().empty();
                if (closestGuid != 0 && carrying) {
                    auto entity = gameHandler.getEntityManager().getEntity(closestGuid);
                    if (entity && entity->getType() == game::ObjectType::PLAYER) {
                        uint8_t srcBag = 0xFF, srcSlot = 0;
                        const bool haveSource =
                            inventoryScreen.isHoldingItem()
                                ? inventoryScreen.heldItemSource(srcBag, srcSlot)
                                : frameXmlCursorWireSlot(srcBag, srcSlot);
                        if (haveSource) {
                            pendingTradeItem_ = PendingTradeItem{.active = true, .bag = srcBag, .slot = srcSlot};
                            gameHandler.initiateTrade(closestGuid);
                            // Whichever cursor was carrying it.
                            if (inventoryScreen.isHoldingItem())
                                inventoryScreen.releaseHeldItem();
                            else
                                frameXmlPutCursorDown();
                            return;
                        }
                    }
                }

                // An item waiting for a unit takes this click instead of it
                // selecting anyone: "right-click the treat, then click the dog"
                // is the whole of what the cursor is asking for.
                if (closestGuid != 0 && gameHandler.isAwaitingUnitTarget()) {
                    gameHandler.completeItemUseOnUnit(closestGuid);
                } else if (closestGuid != 0) {
                    if (closestGuid == gameHandler.getHookedFishingBobberGuid()) {
                        gameHandler.interactWithGameObject(closestGuid);
                    } else {
                        gameHandler.setTarget(closestGuid);
                    }
                } else if (addons::storedCVarValue("deselectOnClick", "1") != "0") {
                    // Clicked empty space - deselect current target, unless the
                    // player asked otherwise. The panel calls this Sticky
                    // Targeting and ticks it to mean "do not", which is why the
                    // sense reads backwards here: self.invert on the checkbox
                    // turns the tick into a zero.
                    //
                    // Clearing was unconditional, so the setting moved and the
                    // target went anyway. The default is the clearing this
                    // client already did, so nobody's targeting changes by the
                    // switch beginning to work.
                    gameHandler.clearTarget();
                }
            }
        }
    }

    // Right-click: select NPC (if needed) then interact / loot / auto-attack.
    // Record the press position; the action only fires on release for a tap (below),
    // never for a right-drag camera rotate - otherwise turning the view toward a nearby
    // mob would auto-attack it without the player intending to engage.
    if (!io.WantCaptureMouse && !frameXmlOwnsMouse() &&
        input.isMouseButtonJustPressed(SDL_BUTTON_RIGHT) && !input.isMouseButtonPressed(SDL_BUTTON_LEFT)) {
        rightClickPressPos_ = input.getMousePosition();
        rightClickWasPress_ = true;
    }

    // On right mouse-up, act only if it was a click (not a drag / camera rotate).
    if (rightClickWasPress_ && input.isMouseButtonJustReleased(SDL_BUTTON_RIGHT)) {
        rightClickWasPress_ = false;
        glm::vec2 rDragDelta = input.getMousePosition() - rightClickPressPos_;
        const float RCLICK_THRESHOLD = clickDragThreshold();
        if (glm::dot(rDragDelta, rDragDelta) >= RCLICK_THRESHOLD * RCLICK_THRESHOLD) {
            // Treated as a camera rotate - do not interact/attack.
            return;
        }
        // Fishing bobbers are tiny and partly submerged, so their model bounds can
        // miss a cursor ray that visibly lands on the float. Test the authoritative
        // water position first with a forgiving sphere and reel directly, before
        // normal unit targeting can fall through to the player/self target.
        const uint64_t hookedBobber = gameHandler.getHookedFishingBobberGuid();
        if (hookedBobber != 0) {
            auto bobber = gameHandler.getEntityManager().getEntity(hookedBobber);
            auto* renderer = services_.renderer;
            auto* camera = renderer ? renderer->getCamera() : nullptr;
            auto* window = services_.window;
            if (bobber && bobber->getType() == game::ObjectType::GAMEOBJECT && camera && window) {
                const glm::vec2 mousePos = input.getMousePosition();
                const rendering::Ray ray = camera->screenToWorldRay(
                    mousePos.x, mousePos.y,
                    static_cast<float>(window->getWidth()),
                    static_cast<float>(window->getHeight()));
                glm::vec3 bobberCenter = core::coords::canonicalToRender(
                    glm::vec3(bobber->getX(), bobber->getY(), bobber->getZ()));
                bobberCenter.z += 0.35f;
                float hitT = 0.0f;
                constexpr float kFishingBobberClickRadius = 4.0f;
                if (raySphereIntersect(ray, bobberCenter, kFishingBobberClickRadius, hitT)) {
                    LOG_WARNING("Fishing bobber direct click: guid=0x", std::hex,
                                hookedBobber, std::dec, " hitDistance=", hitT);
                    gameHandler.interactWithGameObject(hookedBobber);
                    return;
                }
            }
        }

        // If a gameobject is already targeted, prioritize interacting with that target
        // instead of re-picking under cursor (which can hit nearby decorative GOs).
        // Exclude chair-type GOs (type 7): otherwise any right-click (including the
        // start of a camera rotate) auto-sits the player whenever a chair is targeted.
        if (gameHandler.hasTarget()) {
            auto target = gameHandler.getTarget();
            if (target && target->getType() == game::ObjectType::GAMEOBJECT) {
                auto go = std::static_pointer_cast<game::GameObject>(target);
                auto* goInfo = gameHandler.getCachedGameObjectInfo(go->getEntry());
                if (!goInfo || goInfo->type != 7) {
                    LOG_DEBUG("[GO-DIAG] Right-click: re-interacting with targeted GO 0x",
                                std::hex, target->getGuid(), std::dec);
                    gameHandler.setTarget(target->getGuid());
                    gameHandler.interactWithGameObject(target->getGuid());
                    return;
                }
            }
        }

        // If no target or right-clicking in world, try to pick one under cursor
        {
            auto* renderer = services_.renderer;
            auto* camera = renderer ? renderer->getCamera() : nullptr;
            auto* window = services_.window;
            if (camera && window) {
                // If a quest objective gameobject is under the cursor, prefer it over
                // hostile units so quest pickups (e.g. "Bundle of Wood") are reliable.
                std::unordered_set<uint32_t> questObjectiveGoEntries;
                {
                    const auto& ql = gameHandler.getQuestLog();
                    questObjectiveGoEntries.reserve(32);
                    for (const auto& q : ql) {
                        if (q.complete) continue;
                        for (const auto& obj : q.killObjectives) {
                            if (obj.npcOrGoId >= 0 || obj.required == 0) continue;
                            uint32_t entry = static_cast<uint32_t>(-obj.npcOrGoId);
                            uint32_t cur = 0;
                            auto it = q.killCounts.find(entry);
                            if (it != q.killCounts.end()) cur = it->second.first;
                            if (cur < obj.required) questObjectiveGoEntries.insert(entry);
                        }
                    }
                }

                glm::vec2 mousePos = input.getMousePosition();
                float screenW = static_cast<float>(window->getWidth());
                float screenH = static_cast<float>(window->getHeight());
                rendering::Ray ray = camera->screenToWorldRay(mousePos.x, mousePos.y, screenW, screenH);
                // The right-click picker wants a slightly taller, tighter fallback
                // sphere than the target picker, and must ignore chairs: their wide
                // sphere is easy to catch while right-drag-rotating the camera, which
                // sits the player down.
                ui::ScenePickParams params;
                params.unitHitRadius = 1.5f;
                params.unitHeightOffset = 1.5f;
                params.skipChairs = true;

                float closestQuestGoT = 1e30f;
                uint64_t closestQuestGoGuid = 0;
                float hookedBobberT = 1e30f;
                uint64_t hookedBobberGuid = 0;
                const ui::ScenePick pick = ui::pickScene(
                    gameHandler, ray, params,
                    [&](uint64_t guid, const std::shared_ptr<game::Entity>& entity,
                        float hitT, float /*centerT*/) {
                        if (entity->getType() != game::ObjectType::GAMEOBJECT) return;
                        if (guid == gameHandler.getHookedFishingBobberGuid() &&
                            hitT < hookedBobberT) {
                            hookedBobberT = hitT;
                            hookedBobberGuid = guid;
                        }
                        if (questObjectiveGoEntries.empty()) return;
                        auto go = std::static_pointer_cast<game::GameObject>(entity);
                        if (questObjectiveGoEntries.count(go->getEntry()) &&
                            hitT < closestQuestGoT) {
                            closestQuestGoT = hitT;
                            closestQuestGoGuid = guid;
                        }
                    });

                uint64_t closestGuid = 0;
                game::ObjectType closestType = game::ObjectType::OBJECT;

                // A hooked fishing bobber is time-sensitive and intentionally small:
                // if its click sphere was hit, reel it rather than selecting an
                // overlapping unit or decorative object.
                if (hookedBobberGuid != 0) {
                    closestGuid = hookedBobberGuid;
                    closestType = game::ObjectType::GAMEOBJECT;
                } else if (closestQuestGoGuid != 0) {
                    closestGuid = closestQuestGoGuid;
                    closestType = game::ObjectType::GAMEOBJECT;
                } else {
                    closestGuid = pick.resolve();
                    closestType = (closestGuid != 0 && closestGuid == pick.objectGuid)
                                      ? game::ObjectType::GAMEOBJECT
                                      : game::ObjectType::UNIT;
                }

                if (closestGuid != 0) {
                    if (closestType == game::ObjectType::GAMEOBJECT) {
                        LOG_DEBUG("[GO-DIAG] Right-click: raypick hit GO 0x",
                                    std::hex, closestGuid, std::dec);
                        gameHandler.setTarget(closestGuid);
                        gameHandler.interactWithGameObject(closestGuid);
                        return;
                    }
                    gameHandler.setTarget(closestGuid);
                }
            }
        }
        if (gameHandler.hasTarget()) {
            auto target = gameHandler.getTarget();
            if (target) {
                if (target->getType() == game::ObjectType::UNIT) {
                    // Check if unit is dead (health == 0) → loot, otherwise interact/attack
                    auto unit = std::static_pointer_cast<game::Unit>(target);
                    if (unit->getHealth() == 0 && unit->getMaxHealth() > 0) {
                        gameHandler.lootTarget(target->getGuid());
                    } else {
                        // Interact with service NPCs; otherwise treat non-interactable living units
                        // as attackable fallback (covers bad faction-template classification).
                        auto isSpiritNpc = [&]() -> bool {
                            constexpr uint32_t NPC_FLAG_SPIRIT_GUIDE = 0x00004000;
                            constexpr uint32_t NPC_FLAG_SPIRIT_HEALER = 0x00008000;
                            if (unit->getNpcFlags() & (NPC_FLAG_SPIRIT_GUIDE | NPC_FLAG_SPIRIT_HEALER)) {
                                return true;
                            }
                            std::string name = unit->getName();
                            std::transform(name.begin(), name.end(), name.begin(),
                                           [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
                            return (name.find("spirit healer") != std::string::npos) ||
                                   (name.find("spirit guide") != std::string::npos);
                        };
                        bool allowSpiritInteract = (gameHandler.isPlayerDead() || gameHandler.isPlayerGhost()) && isSpiritNpc();
                        bool canInteractNpc = unit->isInteractable() || allowSpiritInteract;
                        bool shouldAttackByFallback = !canInteractNpc;
                        if (!unit->isHostile() && canInteractNpc) {
                            gameHandler.interactWithNpc(target->getGuid());
                        } else if (unit->isHostile() || shouldAttackByFallback) {
                            gameHandler.startAutoAttack(target->getGuid());
                        }
                    }
                } else if (target->getType() == game::ObjectType::GAMEOBJECT) {
                    // Skip chairs: auto-sit must only happen when the chair is
                    // under the cursor, never as a side effect of right-click.
                    auto go = std::static_pointer_cast<game::GameObject>(target);
                    auto* goInfo = gameHandler.getCachedGameObjectInfo(go->getEntry());
                    if (!goInfo || goInfo->type != 7) {
                        gameHandler.interactWithGameObject(target->getGuid());
                    }
                } else if (target->getType() == game::ObjectType::PLAYER) {
                    // Right-click another player could start attack in PvP context
                }
            }
        }
    }
}


}} // namespace wowee::ui
