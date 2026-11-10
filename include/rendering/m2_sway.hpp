#pragma once

#include <algorithm>

namespace wowee::rendering {

/// How a model bends in the wind, as both passes that draw it must agree.
///
/// The main pass and the shadow pass each displace foliage vertices, and they
/// have to displace them identically: a tree whose shadow sways by a different
/// amount, or with a different profile up its trunk, drops a dappled pattern
/// that drifts against its own canopy as the wind phase advances. From the
/// ground that reads as the shadow flickering under a tree that looks fine.
///
/// They did diverge. shadow.vert.glsl said "matches m2.vert.glsl" while
/// normalising height against a hardcoded twenty yards at an amplitude of one,
/// where the main pass uses the model's own height and its own throw. The two
/// agreed for a tree exactly twenty yards tall and for nothing else.
///
/// So neither pass computes this any more; both ask here.
struct M2Sway {
    /// What the vertex shaders switch on: -1 sky, 0 none, 1 foliage,
    /// 2 ground clutter, 3 hanging cloth, 4 cloth on a planted pole.
    int mode = 0;
    /// The height the bend is normalised against, in model space.
    float refHeight = 20.0f;
    /// Amplitude scale; 1.0 is the tree-sized default.
    float amp = 1.0f;
    /// The model's own height, for the brush a player pushes through it with.
    float plantHeight = 0.0f;
};

/// Suppress only procedural foliage wind during a diagnostic comparison.
/// Ground-detail animation and cloth use their own paths and remain unchanged.
/// Both the main pass and the shadow pass must use the same value.
inline float m2SwayAmplitude(const M2Sway& sway, bool disableFoliageWind) {
    return disableFoliageWind && sway.mode == 1 ? 0.0f : sway.amp;
}

/// The sway a model gets from what it is and how big it is.
///
/// `boundMinZ`/`boundMaxZ` are the model's own bounds. Height is measured from
/// the model's base rather than from its origin: a few detail doodads sit with
/// geometry below z=0.
/// `standingCloth` says the cloth is carried on a pole planted in the ground
/// rather than hung from a bar. The two are the same wind with the weight
/// turned end for end, and nothing in the bounds tells them apart - both reach
/// z=0 - so the caller decides from the geometry and passes the answer here.
inline M2Sway m2SwayFor(bool sky, bool hangingCloth, bool windFoliage, bool groundDetail,
                        float boundMinZ, float boundMaxZ, bool standingCloth = false) {
    M2Sway out;
    // The model's own height, whatever it is made of. It used to be filled in
    // only on the paths that sway, so anything else reached the shader with
    // zero and ModelHeight came out as a flat 1.0 - which is no use to a
    // fragment that wants to know how far up the model it is. A fire is not
    // foliage and still has a top.
    out.plantHeight = std::max(boundMaxZ - std::min(boundMinZ, 0.0f), 0.05f);
    if (sky) {
        out.mode = -1;
        return out;
    }
    if (hangingCloth) {
        // Held at one end and free at the other, so the shader is given the
        // top and the span rather than a height to normalise against - which
        // end is held is the mode. The throw is a twentieth of the cloth's own
        // drop: a banner indoors breathes, it does not flap.
        const float span = std::max(boundMaxZ - boundMinZ, 0.05f);
        out.mode = standingCloth ? 4 : 3;
        out.refHeight = boundMaxZ;
        out.plantHeight = span;
        out.amp = span * 0.05f;
        return out;
    }
    if (!windFoliage) {
        out.mode = 0;
        return out;
    }
    out.mode = groundDetail ? 2 : 1;

    const float height = std::max(boundMaxZ - std::min(boundMinZ, 0.0f), 0.05f);
    out.refHeight = height;
    out.plantHeight = height;

    // How far the tip travels as a fraction of the plant's own height: about a
    // tenth for grass, a fiftieth for a tree, and the blend between them for
    // everything in the middle. 0.35 is the trunk layer's throw in the shader,
    // so dividing by it turns a fraction back into that scale.
    const float t = std::clamp((height - 1.0f) / 19.0f, 0.0f, 1.0f);
    const float relativeThrow = 0.105f + (0.0175f - 0.105f) * t;
    out.amp = relativeThrow * height / 0.35f;
    return out;
}

}  // namespace wowee::rendering
