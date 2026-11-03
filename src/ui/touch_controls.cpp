#include "ui/touch_controls.hpp"

#include "ui/keybinding_manager.hpp"
#include "ui/framexml_takeover.hpp"

#include "core/input.hpp"
#include "core/logger.hpp"
#include "rendering/camera_controller.hpp"

#include <imgui.h>

#include <algorithm>
#include <cmath>

namespace wowee {
namespace ui {

float TouchControls::stickRadius() const {
    if (cachedRadius_ > 0.0f) return cachedRadius_;
    // SDL3 dropped SDL_GetDisplayDPI for a content scale, which is the same
    // number this was deriving: dpi over Android's 160 baseline is 1x.
    float density = 2.0f;
    if (const float scale = SDL_GetDisplayContentScale(SDL_GetPrimaryDisplay());
        scale > 0.0f) {
        density = scale;
    }
    cachedRadius_ = kStickRadiusDp * std::max(density, 1.0f);
    return cachedRadius_;
}

bool TouchControls::held(float axis, float threshold, bool wasOn) {
    const float release = std::max(threshold - kReleaseHysteresis, 0.05f);
    return axis > (wasOn ? release : threshold);
}

TouchControls& touchControls() {
    static TouchControls instance;
    return instance;
}

void TouchControls::setInWorld(bool inWorld) {
    if (inWorld_ == inWorld) return;
    inWorld_ = inWorld;
    if (!inWorld_) reset();
}

void TouchControls::setMovementKeys(bool forward, bool back, bool left, bool right) const {
    auto& bindings = KeybindingManager::getInstance();
    bindings.setVirtualCommand("MOVEFORWARD", "touch", forward);
    bindings.setVirtualCommand("MOVEBACKWARD", "touch", back);
    bindings.setVirtualCommand("STRAFELEFT", "touch", left);
    bindings.setVirtualCommand("STRAFERIGHT", "touch", right);
}

void TouchControls::handleEvent(const SDL_Event& event, int windowWidth, int windowHeight) {
    if (event.type == SDL_EVENT_WINDOW_FOCUS_LOST || event.type == SDL_EVENT_WILL_ENTER_BACKGROUND) {
        reset();
        return;
    }
    if (!inWorld_) return;
    if (event.type != SDL_EVENT_FINGER_DOWN && event.type != SDL_EVENT_FINGER_MOTION &&
        event.type != SDL_EVENT_FINGER_UP) {
        return;
    }

    const float w = static_cast<float>(std::max(windowWidth, 1));
    const float h = static_cast<float>(std::max(windowHeight, 1));
    const float x = event.tfinger.x * w;
    const float y = event.tfinger.y * h;
    const SDL_FingerID id = event.tfinger.fingerID;

    // The zone is a fraction of what the display leaves usable, not of the
    // whole panel: a fraction of the panel put its top and its edge under the
    // gesture bar and the cutout on a phone that has them.
    const bool haveSafe = safe_.w > 0 && safe_.h > 0;
    const float zoneLeft = haveSafe ? static_cast<float>(safe_.x) : 0.0f;
    const float zoneW = haveSafe ? static_cast<float>(safe_.w) : w;

    if (event.type == SDL_EVENT_FINGER_DOWN) {
        if (ImGui::GetIO().WantCaptureMouse || frameXmlOwnsMouse() ||
            ImGui::GetIO().WantTextInput || interfaceTakingTypedInput()) return;
        // The interface is asked first, so a bag or an action button drawn over
        // the corner still gets the press. A stick that ate those would be
        // maddening, and the corner is only a default. The answer has to wait
        // for the next update though: the renderer only knows whether the
        // cursor is over a frame that takes the mouse once it has drawn a frame
        // with the finger reported as a cursor, so the press is recorded as a
        // candidate and promoted in update().
        if (stickFingerId_ == kNoFinger && stickCandidate_ == kNoFinger &&
            x < zoneLeft + zoneW * kStickZoneWidth &&
            !ImGui::GetIO().WantCaptureMouse) {
            LOG_DEBUG("touch: stick candidate at ", x, ",", y);
            stickCandidate_ = id;
            stickCandidateX_ = x;
            stickCandidateY_ = y;
            stickOriginX_ = x;
            stickOriginY_ = y;
            stickX_ = stickY_ = 0.0f;
            return;
        }
        // The look stick claims the right-hand corner the same way, but only as
        // a candidate: the ability ring and the attack button are there too, and
        // a press that lands on one belongs to it. The promotion waits for the
        // next update, when the renderer can say whether the cursor is over a
        // frame that takes the mouse.
        // The thumbs split the screen down the middle: the right half turns,
        // the left half walks. Same width as the movement zone, so the halves
        // meet at the centre instead of leaving a strip that belongs to nobody.
        if (lookStickFingerId_ == kNoFinger && lookCandidate_ == kNoFinger &&
            x > zoneLeft + zoneW * kLookZoneWidth &&
            !ImGui::GetIO().WantCaptureMouse) {
            lookCandidate_ = id;
            lookStickOriginX_ = x;
            lookStickOriginY_ = y;
            lookStickX_ = lookStickY_ = 0.0f;
            return;
        }
        LOG_DEBUG("touch: finger down at ", x, ",", y, " of ", w, "x", h,
                  " uiWants=", ImGui::GetIO().WantCaptureMouse, " - not the stick");
        // A drag turns the view only from the right half. Anywhere left of the
        // middle is not the camera's, so the centre stops steering something
        // that shows nothing.
        if (lookFingerId_ == kNoFinger && x > zoneLeft + zoneW * kLookZoneWidth) {
            lookFingerId_ = id;
            lookX_ = x;
            lookY_ = y;
            lookMoved_ = false;
            lookTravel_ = 0.0f;
        }
        if (pinchA_ == kNoFinger) {
            pinchA_ = id; pinchAX_ = x; pinchAY_ = y;
        } else if (pinchB_ == kNoFinger) {
            pinchB_ = id; pinchBX_ = x; pinchBY_ = y;
            lastPinchSpacing_ = std::hypot(pinchAX_ - pinchBX_, pinchAY_ - pinchBY_);
            pinching_ = true;
        }
        return;
    }

    if (event.type == SDL_EVENT_FINGER_UP) {
        if (id == stickCandidate_) {
            stickCandidate_ = kNoFinger;
            return;
        }
        if (id == lookCandidate_) lookCandidate_ = kNoFinger;
        if (id == lookStickFingerId_) {
            lookStickFingerId_ = kNoFinger;
            lookStickX_ = lookStickY_ = 0.0f;
            return;
        }
        if (id == stickFingerId_) {
            stickFingerId_ = kNoFinger;
            stickX_ = stickY_ = 0.0f;
            setMovementKeys(false, false, false, false);
        } else {
            if (id == lookFingerId_) {
                lookFingerId_ = kNoFinger;
                lookMoved_ = false;
            }
            if (id == pinchA_) { pinchA_ = kNoFinger; pinching_ = false; }
            if (id == pinchB_) { pinchB_ = kNoFinger; pinching_ = false; }
        }
        return;
    }

    // SDL_EVENT_FINGER_MOTION
    if (id == stickCandidate_) {
        // Still undecided: the thumb may slide a little before the update that
        // promotes it, and the stick should start from where it went down.
        return;
    }
    if (id == lookStickFingerId_) {
        const float radius = stickRadius();
        lookStickX_ = std::clamp((x - lookStickOriginX_) / radius, -1.0f, 1.0f);
        lookStickY_ = std::clamp((y - lookStickOriginY_) / radius, -1.0f, 1.0f);
        return;
    }
    if (id == lookCandidate_) {
        // Still undecided: the thumb may slide a little before the update that
        // promotes it, and the stick should start from where it went down.
        return;
    }
    if (id == stickFingerId_) {
        const float radius = stickRadius();
        stickX_ = std::clamp((x - stickOriginX_) / radius, -1.0f, 1.0f);
        stickY_ = std::clamp((y - stickOriginY_) / radius, -1.0f, 1.0f);
        return;
    }
    // One finger dragging is a look. Two is a pinch, and the view should not
    // swing while the fingers close.
    if (id == lookFingerId_ && pinchB_ == kNoFinger) {
        const float dx = x - lookX_;
        const float dy = y - lookY_;
        lookX_ = x;
        lookY_ = y;
        // Distance travelled, not distance from where the finger went down: a
        // slow drag arrives in many small steps and would never clear a
        // per-event threshold.
        if (!lookMoved_) {
            lookTravel_ += std::hypot(dx, dy);
            if (lookTravel_ >= kLookSlopPixels) lookMoved_ = true;
        }
        if (lookMoved_ && camera_) camera_->applyLookDelta(dx, dy);
    }

    if (id == pinchA_) { pinchAX_ = x; pinchAY_ = y; }
    else if (id == pinchB_) { pinchBX_ = x; pinchBY_ = y; }
    else return;

    if (!pinching_ || pinchA_ == kNoFinger || pinchB_ == kNoFinger) return;
    const float spacing = std::hypot(pinchAX_ - pinchBX_, pinchAY_ - pinchBY_);
    const float moved = spacing - lastPinchSpacing_;
    if (std::abs(moved) < kPinchPixelsPerNotch) return;
    lastPinchSpacing_ = spacing;

    // An analog camera feed, like pad triggers: rebinding the mouse wheel must
    // not rebind a pinch. Keep the camera's existing calibration and clamping.
    if (camera_) camera_->processMouseWheel(moved / kPinchPixelsPerNotch);
}

void TouchControls::update() {
    if (ImGui::GetIO().WantTextInput || interfaceTakingTypedInput()) {
        reset();
        return;
    }
    if (!inWorld_) return;

    // A candidate becomes the stick only if the interface has nothing at the
    // point it went down on. Asked of the widget tree directly, not of the flag
    // the renderer publishes: that one answers for where the cursor was last
    // dispatched, and a finger that has just landed is a frame ahead of it -
    // which is exactly the press that must not turn into a stick.
    if (stickFingerId_ == kNoFinger && stickCandidate_ != kNoFinger) {
        if (ImGui::GetIO().WantCaptureMouse ||
            frameXmlPointOwnsMouse(stickCandidateX_, stickCandidateY_)) {
            LOG_DEBUG("touch: move stick refused over the interface at ",
                        stickCandidateX_, ",", stickCandidateY_);
            stickCandidate_ = kNoFinger;
        } else {
            LOG_DEBUG("touch: move stick claimed at ",
                        stickCandidateX_, ",", stickCandidateY_);
            stickFingerId_ = stickCandidate_;
            stickCandidate_ = kNoFinger;
        }
    }

    // The look stick asks the same question of the same tree: a thumb on the
    // ability ring is casting, not turning.
    if (lookStickFingerId_ == kNoFinger && lookCandidate_ != kNoFinger) {
        if (ImGui::GetIO().WantCaptureMouse ||
            frameXmlPointOwnsMouse(lookStickOriginX_, lookStickOriginY_)) {
            LOG_DEBUG("touch: look stick refused over the interface at ",
                        lookStickOriginX_, ",", lookStickOriginY_);
            lookCandidate_ = kNoFinger;
        } else {
            LOG_DEBUG("touch: look stick claimed at ",
                        lookStickOriginX_, ",", lookStickOriginY_);
            lookStickFingerId_ = lookCandidate_;
            lookCandidate_ = kNoFinger;
        }
    }
    // Held at a deflection, the look stick turns the view at a rate rather than
    // one-to-one: it is a stick, and a stick that had to keep moving to keep
    // turning would be a drag with a ring drawn on it.
    if (lookStickFingerId_ != kNoFinger && camera_) {
        // The inner ring is a threshold on this stick too, the way it is on
        // the movement one: a thumb inside it does not turn the camera. Past
        // the ring the turn starts from nothing, so the edge is not a jump.
        const float mag = std::hypot(lookStickX_, lookStickY_);
        if (mag > kStrafeDeadzone) {
            const float step = kLookStickPixelsPerSecond * ImGui::GetIO().DeltaTime;
            const float gain = (mag - kStrafeDeadzone) / (1.0f - kStrafeDeadzone);
            camera_->applyLookDelta(lookStickX_ / mag * gain * step,
                                    lookStickY_ / mag * gain * step);
        }
    }

    if (stickFingerId_ == kNoFinger) {
        wasForward_ = wasBack_ = wasLeft_ = wasRight_ = false;
        setMovementKeys(false, false, false, false);
        return;
    }
    // Up is forward, sideways is a strafe: the stick moves the character and
    // never the view, which is the other thumb's job.
    //
    // Sideways asks for more of the stick than forward, and each direction lets
    // go later than it took hold, so that holding a heading does not need the
    // thumb kept still to the pixel.
    wasForward_ = held(-stickY_, kWalkDeadzone, wasForward_);
    wasBack_    = held( stickY_, kWalkDeadzone, wasBack_);
    wasLeft_    = held(-stickX_, kStrafeDeadzone, wasLeft_);
    wasRight_   = held( stickX_, kStrafeDeadzone, wasRight_);
    setMovementKeys(wasForward_, wasBack_, wasLeft_, wasRight_);
}

void TouchControls::draw() const {
    if (!inWorld_) return;
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    if (!dl) return;
    const float radius = stickRadius();
    // Both thumbs get the same two rings and knob, so the pair reads as one
    // control: the inner circle used to be the movement stick's alone.
    auto ring = [&](float ox, float oy, float sx, float sy) {
        const ImVec2 origin(ox, oy);
        const ImVec2 knob(ox + sx * radius, oy + sy * radius);
        dl->AddCircle(origin, radius, IM_COL32(255, 255, 255, 60), 48, 3.0f);
        dl->AddCircle(origin, radius * kStrafeDeadzone,
                      IM_COL32(255, 255, 255, 35), 40, 2.0f);
        dl->AddCircleFilled(knob, radius * 0.28f, IM_COL32(255, 255, 255, 95), 32);
    };
    if (stickFingerId_ != kNoFinger) {
        ring(stickOriginX_, stickOriginY_, stickX_, stickY_);
    }
    if (lookStickFingerId_ != kNoFinger) {
        ring(lookStickOriginX_, lookStickOriginY_, lookStickX_, lookStickY_);
    }
}

void TouchControls::reset() {
    stickFingerId_ = kNoFinger;
    stickCandidate_ = kNoFinger;
    lookStickFingerId_ = kNoFinger;
    lookCandidate_ = kNoFinger;
    lookStickX_ = lookStickY_ = 0.0f;
    lookFingerId_ = kNoFinger;
    lookMoved_ = false;
    lookTravel_ = 0.0f;
    pinchA_ = pinchB_ = kNoFinger;
    pinching_ = false;
    stickX_ = stickY_ = 0.0f;
    wasForward_ = wasBack_ = wasLeft_ = wasRight_ = false;
    // Release only this adapter's contribution; pad/keyboard may still hold it.
    setMovementKeys(false, false, false, false);
}

}  // namespace ui
}  // namespace wowee
