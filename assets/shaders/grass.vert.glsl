#version 450

// A grass blade: a tapered strip of five segments bent along a quadratic
// Bezier, moved by wind and by whoever walks through it.
//
// Wind and player interaction are ported from m2.vert.glsl rather than
// invented. That system is already tuned and already world-stable, and grass
// that used its own constants would move at a different rate from the foliage
// standing next to it - two systems rather than one field. The differences are
// deliberate and marked below.

layout(set = 0, binding = 0) uniform PerFrame {
    mat4 view;
    mat4 projection;
    mat4 lightSpaceMatrix;
    vec4 lightDir;
    vec4 lightColor;
    vec4 ambientColor;
    vec4 viewPos;
    vec4 fogColor;
    vec4 fogParams;   // z = wind time
    vec4 shadowParams;
    vec4 playerPos;   // xyz = player world position, w = horizontal speed
    vec4 playerWake;  // xyz = trailing player position (springback reference)
};

// Matches GrassBladeGPU in include/rendering/grass_blade.hpp (std430).
struct GrassBlade {
    vec4 positionHeight;
    vec4 facingWidthPhase;
};

layout(std430, set = 1, binding = 0) readonly buffer GrassSource {
    GrassBlade blades[];
};

// Written by grass_cull.comp.glsl. Indexed by gl_InstanceIndex, which the
// indirect draw bounds to the count that shader produced.
layout(std430, set = 1, binding = 1) readonly buffer VisibleIndices {
    uint visibleIndices[];
};

layout(location = 0) out float vHeightT;
layout(location = 1) out vec3 vNormal;

// Five segments, six rows of two vertices.
const float kSegments = 5.0;

// How far the wind can lay a blade over, as a fraction of its height.
const float kWindBend = 0.22;

void main() {
    GrassBlade blade = blades[visibleIndices[gl_InstanceIndex]];

    vec3  root   = blade.positionHeight.xyz;
    float height = blade.positionHeight.w;
    float facing = blade.facingWidthPhase.x;
    float width  = blade.facingWidthPhase.y;
    float seed   = blade.facingWidthPhase.w;

    // Row up the blade and which side of it this vertex is.
    float row  = floor(float(gl_VertexIndex) * 0.5);
    float side = (gl_VertexIndex - row * 2.0) * 2.0 - 1.0;  // -1 or +1
    float t    = row / kSegments;

    // Width: near constant over the lower half, then to a point. A blade cut
    // square at the top reads as a strip of card, which is what the first
    // version looked like.
    float halfWidth = 0.5 * width * (1.0 - 0.25 * t) * (1.0 - smoothstep(0.45, 1.0, t));

    // ---- Wind -------------------------------------------------------------
    // Two layers rather than m2.vert's three, multiplied rather than summed
    // (spec §27): a gust that crosses the field and a faster rustle riding on
    // it, so the field breathes instead of every blade beating in time. Phase
    // comes from the blade's world position, never from the frame, so a blade
    // does not jump when the camera moves (spec §36).
    float windTime = fogParams.z;
    float gust   = sin(windTime * 0.8 + dot(root.xy, vec2(0.10, 0.13)));
    float rustle = sin(windTime * 1.7 + dot(root.xy, vec2(0.37, 0.71))
                       + seed * 6.2831853);
    float windAmount = (0.5 + 0.5 * gust) * (0.75 + 0.25 * rustle);

    // A single world direction, so the whole field leans together.
    vec2 windDir = normalize(vec2(0.80, 0.60));
    vec2 bendVec = windDir * (windAmount * height * kWindBend);

    // ---- The player brushing past -----------------------------------------
    // Ported from m2.vert.glsl:139-199. The size gate is dropped - every blade
    // here is grass, so there is no tree to exempt - and the reach is smaller,
    // because a tuft gives way closer in than a waist-high fern does.
    float levelGate = 1.0 - smoothstep(1.5, 4.0, abs(root.z - playerPos.z));
    if (levelGate > 0.0) {
        float reach = 1.1 + height * 0.35;

        // Bend away from the player, and from where the player was a moment
        // ago. The stronger of the two rather than the sum, so a standing
        // player - where both points coincide - does not bend it twice as far
        // as a walking one.
        vec2 toNow  = root.xy - playerPos.xy;
        vec2 toWake = root.xy - playerWake.xy;
        float dNow  = length(toNow);
        float dWake = length(toWake);
        float infNow  = 1.0 - smoothstep(0.0, reach, dNow);
        float infWake = 1.0 - smoothstep(0.0, reach, dWake);

        vec2 dir;
        float influence;
        if (infNow >= infWake) {
            influence = infNow;
            dir = toNow / max(dNow, 0.001);
        } else {
            influence = infWake;
            dir = toWake / max(dWake, 0.001);
        }
        influence *= levelGate;

        // Trodden grass lies most of the way over, unlike the clutter this is
        // ported from, which only leans.
        bendVec += dir * (influence * height * 0.85);

        // A quiver riding on the bend, only while the player is moving, phased
        // per blade so the patch shivers rather than pulsing in unison.
        float speed = clamp(playerPos.w / 7.0, 0.0, 1.0);
        float quiver = sin(windTime * 17.0 + dot(root.xy, vec2(3.1, 2.7)))
                     * influence * speed * height * 0.12;
        bendVec += vec2(-dir.y, dir.x) * quiver;
    }

    // ---- Bend the blade along a quadratic Bezier ---------------------------
    // The curve is what makes it a blade rather than a plank on a hinge: the
    // root stays planted and the motion collects at the tip, which is the same
    // quadratic weighting m2.vert.glsl:91-95 applies to its sway.
    float bend = length(bendVec);
    vec2  bendDir = (bend > 1e-5) ? bendVec / bend : vec2(0.0);
    bend = min(bend, height * 0.95);              // never past lying flat
    float bendFrac = bend / max(height, 1e-4);

    // Leaning costs height, or a bent blade would stand as tall as a straight
    // one and the field would look inflated whenever the wind picked up.
    vec3 p0 = vec3(0.0);
    vec3 p1 = vec3(bendDir * (bend * 0.33), height * 0.55);
    vec3 p2 = vec3(bendDir * bend,          height * (1.0 - 0.30 * bendFrac));

    float u = 1.0 - t;
    vec3 curve = u * u * p0 + 2.0 * u * t * p1 + t * t * p2;
    // Derivative of the same curve: the blade's own up direction at this row.
    vec3 tangent = normalize(2.0 * u * (p1 - p0) + 2.0 * t * (p2 - p1));

    // Z is up in render space (renderX = wowY, renderY = wowX, renderZ = wowZ).
    vec3 across = vec3(cos(facing), sin(facing), 0.0);
    vec3 world  = root + curve + across * (halfWidth * side);

    vHeightT = t;
    // Face out of the blade, and let it curl a little across its width so it
    // catches light along the edge instead of reading as a flat cutout.
    vNormal = normalize(cross(across, tangent) + across * (side * 0.35));

    gl_Position = projection * view * vec4(world, 1.0);
}
