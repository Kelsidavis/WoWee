#pragma once

#include <SDL2/SDL.h>

namespace wowee {
namespace rendering { class CameraController; }
namespace ui {

/**
 * The on-screen controls a phone needs, and nothing a desktop does.
 *
 * A touch screen already drives most of this client. SDL reports the first
 * finger as a mouse, so taps press buttons and a drag orbits the camera exactly
 * as a held left button does, and every panel, slider and action button works
 * without knowing a finger touched it. Two things a finger cannot do are hold W
 * and turn a wheel. Those are what this adds.
 *
 * It reads the finger events SDL sends alongside the mouse ones, and claims
 * only what it needs: a thumb in the lower-left corner becomes a movement
 * stick, and two fingers spreading become the camera zoom. Everything else is
 * left alone.
 *
 * The stick's own finger is also the one SDL is reporting as a held mouse
 * button, which would orbit the camera the whole time the player walked. So
 * while the stick is held the camera is told to ignore mouse motion. That is
 * the one place this has to reach outside itself.
 */
class TouchControls {
public:
    /// The stick and the pinch only apply in the world. On the login and
    /// character screens every touch is left to be a mouse.
    void setInWorld(bool inWorld);
    [[nodiscard]] bool isInWorld() const { return inWorld_; }

    /// True while a thumb is on the stick, which is when the camera has to be
    /// held still.
    [[nodiscard]] bool isStickHeld() const { return stickFingerId_ != kNoFinger; }

    /// The camera to steer. Without one the look finger does nothing.
    void setCameraController(rendering::CameraController* camera) { camera_ = camera; }

    /// True while a finger is dragging the view, which is when the character
    /// should face where the camera does.
    [[nodiscard]] bool isSteering() const { return lookFingerId_ != kNoFinger && lookMoved_; }

    void handleEvent(const SDL_Event& event, int windowWidth, int windowHeight);

    /// Applies the stick to the movement keys. Once a frame, after events.
    void update();

    /// Draws the stick where the thumb put it. Nothing when idle.
    void draw() const;

    /// Drops the stick and releases the keys.
    void reset();

    /// Where the stick may start, as a fraction of the window.
    static constexpr float kStickZoneWidth = 0.38f;
    static constexpr float kStickZoneTop = 0.30f;

private:
    static constexpr SDL_FingerID kNoFinger = -1;

    /// How far the thumb travels for a full deflection, in Android's density
    /// independent pixels. A radius in raw pixels is a different size on every
    /// phone: 150 of them is a third of an inch on this one, which is why the
    /// first version was unusable.
    static constexpr float kStickRadiusDp = 130.0f;

    /// Below this the stick reads as centred, so a resting thumb does not walk.
    /// Sideways takes more deflection than forward, so that walking straight
    /// ahead does not need a steady hand.
    static constexpr float kWalkDeadzone = 0.30f;
    static constexpr float kStrafeDeadzone = 0.45f;
    /// How far back inside the deadzone a direction has to fall before it lets
    /// go, so a thumb resting on the edge does not stutter.
    static constexpr float kReleaseHysteresis = 0.10f;
    /// What a pinch has to cover before it is worth a notch of the wheel.
    static constexpr float kPinchPixelsPerNotch = 90.0f;
    /// How far a look finger travels before it counts as a drag rather than a
    /// tap, so that selecting a target does not also swing the view.
    static constexpr float kLookSlopPixels = 16.0f;

    void setMovementKeys(bool forward, bool back, bool left, bool right) const;
    /// Full deflection in real pixels, from the display's density.
    [[nodiscard]] float stickRadius() const;
    /// Held with hysteresis: a direction already on lets go later than it came on.
    [[nodiscard]] static bool held(float axis, float threshold, bool wasOn);

    bool inWorld_ = false;

    SDL_FingerID stickFingerId_ = kNoFinger;
    float stickOriginX_ = 0.0f, stickOriginY_ = 0.0f;
    float stickX_ = 0.0f, stickY_ = 0.0f;   // -1..1

    // Pinch: the two fingers that are not the stick.
    rendering::CameraController* camera_ = nullptr;

    // The finger steering the view. Not the stick's, and not the second of a
    // pinch.
    SDL_FingerID lookFingerId_ = kNoFinger;
    float lookX_ = 0.0f, lookY_ = 0.0f;
    bool lookMoved_ = false;
    float lookTravel_ = 0.0f;

    SDL_FingerID pinchA_ = kNoFinger, pinchB_ = kNoFinger;
    float pinchAX_ = 0.0f, pinchAY_ = 0.0f;
    float pinchBX_ = 0.0f, pinchBY_ = 0.0f;
    float lastPinchSpacing_ = 0.0f;
    bool pinching_ = false;

    // What the stick asked for last frame, for the hysteresis.
    mutable bool wasForward_ = false, wasBack_ = false, wasLeft_ = false, wasRight_ = false;

    mutable float cachedRadius_ = 0.0f;
};

/// The one the client uses. One screen, one pair of thumbs.
TouchControls& touchControls();

}  // namespace ui
}  // namespace wowee
