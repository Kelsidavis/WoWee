#pragma once

/// Where the sun is, from the direction its light travels.
///
/// The lighting system gives a directional vector - the way the light goes -
/// so the sun is the other way. Below the horizon it stays below: this used to
/// be written inline in SkySystem::getSunPosition, which mirrored a sun under
/// the ground back up into the sky (`sunDir = dir` rather than `-dir`).
///
/// Only the lens flare asked where the sun was then, so all the mirror achieved
/// was to invent one for the flare to draw around, at a position no sun was at.
/// The flare's own height attenuation then read the mirrored height as a sun
/// climbing the sky and let it through unweakened. The sun shafts ask too now,
/// and would have streamed from the same invented sun.

#include <glm/glm.hpp>

namespace wowee::rendering {

/// The unit direction from the eye toward the sun.
///
/// A zero or degenerate directional means no sun has been given yet; straight
/// down is what the rest of the sky code falls back to, so the sun reads as
/// being straight up and everything that gates on height turns it off.
inline glm::vec3 sunDirectionFromLightDir(const glm::vec3& directionalDir) {
    const float lenSq = glm::dot(directionalDir, directionalDir);
    if (lenSq < 1e-8f) return glm::vec3(0.0f, 0.0f, 1.0f);
    return -directionalDir * glm::inversesqrt(lenSq);
}

/// Where the sun falls on the screen.
struct SunOnScreen {
    /// False when the sun is behind the eye, where it has no place on screen.
    bool inFront = false;
    /// Framebuffer uv, (0,0) at the top left - the way every full-screen pass
    /// samples. Not clamped: a sun just past the edge still streams rays in.
    glm::vec2 uv{0.0f};
};

/// The sun's place on screen, for the view and projection the frame was drawn
/// with. A direction, so it is projected with w = 0: the sun is infinitely far,
/// and moving the camera does not move it.
inline SunOnScreen sunScreenPosition(const glm::mat4& view, const glm::mat4& projection,
                                     const glm::vec3& sunDir) {
    const glm::vec4 clip = projection * view * glm::vec4(sunDir, 0.0f);
    if (clip.w <= 1e-4f) return {};
    return {.inFront = true, .uv = glm::vec2(clip) / clip.w * 0.5f + 0.5f};
}

}  // namespace wowee::rendering
