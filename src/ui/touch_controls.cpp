#include "ui/touch_controls.hpp"

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
    float diagonalDpi = 0.0f;
    float density = 2.0f;
    if (SDL_GetDisplayDPI(0, &diagonalDpi, nullptr, nullptr) == 0 && diagonalDpi > 0.0f) {
        density = diagonalDpi / 160.0f;  // Android's own baseline for 1x
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
    auto& input = core::Input::getInstance();
    input.setVirtualKey(SDL_SCANCODE_W, forward);
    input.setVirtualKey(SDL_SCANCODE_S, back);
    // Q and E rather than A and D. With the right button up - which it always
    // is on a phone - this client turns the character on A and D and strafes on
    // Q and E. Turning meant the stick swung the view as well as the feet,
    // which is not what a stick pushed sideways should do.
    input.setVirtualKey(SDL_SCANCODE_Q, left);
    input.setVirtualKey(SDL_SCANCODE_E, right);
}

void TouchControls::handleEvent(const SDL_Event& event, int windowWidth, int windowHeight) {
    if (!inWorld_) return;
    if (event.type != SDL_FINGERDOWN && event.type != SDL_FINGERMOTION &&
        event.type != SDL_FINGERUP) {
        return;
    }

    const float w = static_cast<float>(std::max(windowWidth, 1));
    const float h = static_cast<float>(std::max(windowHeight, 1));
    const float x = event.tfinger.x * w;
    const float y = event.tfinger.y * h;
    const SDL_FingerID id = event.tfinger.fingerId;

    if (event.type == SDL_FINGERDOWN) {
        // The interface is asked first, so a bag or an action button drawn over
        // the corner still gets the press. A stick that ate those would be
        // maddening, and the corner is only a default.
        if (stickFingerId_ == kNoFinger && x < w * kStickZoneWidth &&
            y > h * kStickZoneTop && !ImGui::GetIO().WantCaptureMouse) {
            LOG_DEBUG("touch: stick claimed at ", x, ",", y);
            stickFingerId_ = id;
            stickOriginX_ = x;
            stickOriginY_ = y;
            stickX_ = stickY_ = 0.0f;
            return;
        }
        LOG_DEBUG("touch: finger down at ", x, ",", y, " of ", w, "x", h,
                  " uiWants=", ImGui::GetIO().WantCaptureMouse, " - not the stick");
        if (lookFingerId_ == kNoFinger) {
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

    if (event.type == SDL_FINGERUP) {
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

    // SDL_FINGERMOTION
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

    // Fingers spreading pulls the camera in, the way a wheel forward does. Sent
    // as a wheel event so it goes through the same clamping and the same
    // setting as a wheel, rather than reaching into the camera here.
    SDL_Event wheel{};
    wheel.type = SDL_MOUSEWHEEL;
    wheel.wheel.timestamp = SDL_GetTicks();
    wheel.wheel.y = moved > 0.0f ? 1 : -1;
    wheel.wheel.preciseY = moved / kPinchPixelsPerNotch;
    SDL_PushEvent(&wheel);
}

void TouchControls::update() {
    if (!inWorld_ || stickFingerId_ == kNoFinger) {
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
    if (!inWorld_ || stickFingerId_ == kNoFinger) return;
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    if (!dl) return;
    const float radius = stickRadius();
    const ImVec2 origin(stickOriginX_, stickOriginY_);
    const ImVec2 knob(stickOriginX_ + stickX_ * radius, stickOriginY_ + stickY_ * radius);
    dl->AddCircle(origin, radius, IM_COL32(255, 255, 255, 60), 48, 3.0f);
    // The ring the thumb has to cross to start strafing, so the player can see
    // why walking straight is the easy thing to do.
    dl->AddCircle(origin, radius * kStrafeDeadzone, IM_COL32(255, 255, 255, 35), 40, 2.0f);
    dl->AddCircleFilled(knob, radius * 0.28f, IM_COL32(255, 255, 255, 95), 32);
}

void TouchControls::reset() {
    stickFingerId_ = kNoFinger;
    lookFingerId_ = kNoFinger;
    lookMoved_ = false;
    lookTravel_ = 0.0f;
    pinchA_ = pinchB_ = kNoFinger;
    pinching_ = false;
    stickX_ = stickY_ = 0.0f;
    wasForward_ = wasBack_ = wasLeft_ = wasRight_ = false;
    // Everything the stick could be holding, not just the four it uses now.
    core::Input::getInstance().clearVirtualKeys();
}

}  // namespace ui
}  // namespace wowee
