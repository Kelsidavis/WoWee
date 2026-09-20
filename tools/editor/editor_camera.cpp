#include "editor_camera.hpp"
#include <glm/gtc/constants.hpp>
#include <algorithm>
#include <cmath>

namespace wowee {
namespace editor {

EditorCamera::EditorCamera() {
    camera_.setPosition(glm::vec3(0.0f, 0.0f, 200.0f));
    camera_.setFov(60.0f);
    camera_.setRotation(0.0f, -30.0f);
    yaw_ = 0.0f;
    pitch_ = -30.0f;
}

void EditorCamera::update(float deltaTime) {
    float moveSpeed = speed_ * deltaTime;
    if (keyShift_) moveSpeed *= 3.0f;

    glm::vec3 forward = camera_.getForward();
    glm::vec3 right = camera_.getRight();
    glm::vec3 up(0.0f, 0.0f, 1.0f); // Z-up (WoW render coords)

    glm::vec3 pos = camera_.getPosition();
    if (keyW_) pos += forward * moveSpeed;
    if (keyS_) pos -= forward * moveSpeed;
    if (keyD_) pos += right * moveSpeed;
    if (keyA_) pos -= right * moveSpeed;
    if (keyE_) pos += up * moveSpeed;
    if (keyQ_) pos -= up * moveSpeed;

    camera_.setPosition(pos);
}

void EditorCamera::processMouseMotion(int dx, int dy) {
    if (!rightMouseDown_) return;

    constexpr float sensitivity = 0.15f; // degrees per pixel
    yaw_ += static_cast<float>(dx) * sensitivity;
    pitch_ -= static_cast<float>(dy) * sensitivity;
    pitch_ = std::clamp(pitch_, -89.0f, 89.0f);

    camera_.setRotation(yaw_, pitch_);
}

void EditorCamera::processMouseWheel(float delta, bool shiftHeld) {
    if (shiftHeld) {
        speed_ = std::clamp(speed_ + delta * 20.0f, 10.0f, 2000.0f);
    } else {
        glm::vec3 pos = camera_.getPosition();
        pos += camera_.getForward() * delta * speed_ * 0.3f;
        camera_.setPosition(pos);
    }
}

void EditorCamera::processKeyEvent(const SDL_KeyboardEvent& event) {
    bool pressed = (event.type == SDL_EVENT_KEY_DOWN);
    switch (event.scancode) {
        case SDL_SCANCODE_W: keyW_ = pressed; break;
        case SDL_SCANCODE_A: keyA_ = pressed; break;
        case SDL_SCANCODE_S: keyS_ = pressed; break;
        case SDL_SCANCODE_D: keyD_ = pressed; break;
        case SDL_SCANCODE_Q: keyQ_ = pressed; break;
        case SDL_SCANCODE_E: keyE_ = pressed; break;
        case SDL_SCANCODE_LSHIFT:
        case SDL_SCANCODE_RSHIFT: keyShift_ = pressed; break;
        default: break;
    }
}

void EditorCamera::processMouseButton(const SDL_MouseButtonEvent& event) {
    if (event.button == SDL_BUTTON_RIGHT)
        rightMouseDown_ = (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN);
    if (event.button == SDL_BUTTON_MIDDLE)
        middleMouseDown_ = (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN);
}

void EditorCamera::processMiddleMouseMotion(int dx, int dy, const glm::vec3& pivotPoint) {
    if (!middleMouseDown_) return;

    constexpr float sensitivity = 0.3f;
    yaw_ += static_cast<float>(dx) * sensitivity;
    pitch_ -= static_cast<float>(dy) * sensitivity;
    pitch_ = std::clamp(pitch_, -89.0f, 89.0f);
    camera_.setRotation(yaw_, pitch_);

    // Orbit: maintain distance from pivot. Reject NaN pivot - would
    // poison the camera position permanently and the next frame would
    // produce NaN view/proj matrices.
    if (!std::isfinite(pivotPoint.x) || !std::isfinite(pivotPoint.y) ||
        !std::isfinite(pivotPoint.z)) return;
    glm::vec3 toPivot = pivotPoint - camera_.getPosition();
    float dist = glm::length(toPivot);
    if (!std::isfinite(dist)) return;
    glm::vec3 newPos = pivotPoint - camera_.getForward() * dist;
    camera_.setPosition(newPos);
}

void EditorCamera::setPosition(const glm::vec3& pos) {
    if (!std::isfinite(pos.x) || !std::isfinite(pos.y) || !std::isfinite(pos.z)) return;
    camera_.setPosition(pos);
}

void EditorCamera::setYawPitch(float yaw, float pitch) {
    if (!std::isfinite(yaw) || !std::isfinite(pitch)) return;
    yaw_ = yaw;
    // Match the mouse-motion clamp so out-of-range pitch from a saved
    // bookmark doesn't roll the camera upside down.
    pitch_ = std::clamp(pitch, -89.0f, 89.0f);
    camera_.setRotation(yaw_, pitch_);
}

} // namespace editor
} // namespace wowee
