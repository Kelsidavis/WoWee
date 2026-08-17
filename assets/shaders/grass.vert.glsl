#version 450

// A grass blade: a tapered strip of five segments bent along a quadratic
// Bezier, moved by wind and by whoever walks through it.
//
// The player interaction is ported from m2.vert.glsl. The wind is not, any
// more: it began as that system's constants, and was reworked into travelling
// waves - fronts that sweep across the field and gusts that roll through in
// bands - because grass in place-to-place unison reads as a texture, and a
// field the wind visibly crosses reads as weather. Phase still comes from
// world position and never from the frame (spec §36).
//
// Some blades have gone to seed and some carry blooms. Which ones is decided
// here, from the blade's own seed and a slow world-space patch function, so
// seeding drifts across a meadow in swathes rather than speckling it evenly -
// and the same blade is the same blade on every frame and every rebuild.

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
    vec4 groundColor;      // xyz = terrain colour under the root, w = 1 if real
};

layout(std430, set = 1, binding = 0) readonly buffer GrassSource {
    GrassBlade blades[];
};

// Written by grass_cull.comp.glsl. Indexed by gl_InstanceIndex, which the
// indirect draw bounds to the count that shader produced.
layout(std430, set = 1, binding = 1) readonly buffer VisibleIndices {
    uint visibleIndices[];
};

// Matches GrassProfileGPU in include/rendering/grass_blade.hpp (std430).
// What grows on this patch of ground, derived from the detail doodads the map
// plants there.
struct GrassProfile {
    vec4 rootColor;
    vec4 tipColor;
    vec4 params;   // x = colour variation, y = stiffness, z = bloom, w = seed
};

layout(std430, set = 1, binding = 2) readonly buffer GrassProfiles {
    GrassProfile profiles[];
};

layout(location = 0) out float vHeightT;
layout(location = 1) out vec3 vNormal;
layout(location = 2) out vec3 vRootColor;
layout(location = 3) out vec3 vTipColor;
layout(location = 4) out vec4 vGroundColor;
layout(location = 5) out vec4 vHeadColor;   // rgb head colour, a = strength

// Five segments, six rows of two vertices. Row 4 sits at t = 0.8, which is
// where the head envelopes below put their bulge - a peak between rows would
// never be sampled.
const float kSegments = 5.0;

// How far the wind can lay a blade over, as a fraction of its height.
const float kWindBend = 0.32;

// The bloom palette. Four is enough for a meadow to read as mixed wildflowers;
// which one a blade gets is its own seed's business.
const vec3 kBloomColors[4] = vec3[4](
    vec3(0.95, 0.80, 0.25),   // yellow
    vec3(0.90, 0.45, 0.65),   // pink
    vec3(0.92, 0.92, 0.85),   // white
    vec3(0.55, 0.45, 0.85));  // violet

// A seed head runs from pale straw to the dark plum-brown of a fescue
// panicle; where a blade sits between the two is its own seed's business.
const vec3 kSeedStraw = vec3(0.72, 0.64, 0.42);
const vec3 kSeedPlum  = vec3(0.32, 0.22, 0.24);

// A slow world-space blob field, 0..1, blobs a couple of dozen yards across.
// What turns "20% of blades" into "most blades over there, few here".
float patchField(vec2 p, float scale, float shift) {
    return 0.5 + 0.5 * sin(p.x * scale + shift) * sin(p.y * scale * 1.27 + shift * 1.7);
}

void main() {
    GrassBlade blade = blades[visibleIndices[gl_InstanceIndex]];

    vec3  root   = blade.positionHeight.xyz;
    float height = blade.positionHeight.w;
    float facing = blade.facingWidthPhase.x;
    float width  = blade.facingWidthPhase.y;
    float seed   = blade.facingWidthPhase.w;

    GrassProfile profile = profiles[uint(blade.facingWidthPhase.z + 0.5)];
    // Stiff growth resists both the wind and being trodden on, so it divides
    // every bend rather than being subtracted from one of them.
    float give = 1.0 / max(profile.params.y, 0.01);

    // ---- What kind of blade this is ----------------------------------------
    // Two more uniform randoms unpacked from the one seed; the patch fields
    // gate them so seeding and blooming come in drifts. Seeded wins ties -
    // a blade cannot be both.
    float rSeeded = fract(seed * 127.31);
    float rBloom  = fract(seed * 311.73);
    float seedGate  = smoothstep(0.35, 0.75, patchField(root.xy, 0.31, 0.0));
    float bloomGate = smoothstep(0.40, 0.80, patchField(root.xy, 0.47, 2.9));
    bool seeded = rSeeded < profile.params.w * (0.15 + 1.85 * seedGate);
    bool bloom  = !seeded && rBloom < profile.params.z * (0.20 + 1.80 * bloomGate);

    if (seeded) {
        // Bolted: the tall wispy stems that stand above the sward.
        height *= 1.6;
    } else if (bloom) {
        height *= 0.92;
    }

    // Sink into the ground over the last stretch before the cull distance
    // (GrassRenderer::kCullDistance / kFadeStart - change together), so the
    // field thins away instead of ending on a cut line.
    height *= 1.0 - smoothstep(40.0, 55.0, distance(root.xy, viewPos.xy));

    // Row up the blade and which side of it this vertex is.
    float row  = floor(float(gl_VertexIndex) * 0.5);
    float side = (gl_VertexIndex - row * 2.0) * 2.0 - 1.0;  // -1 or +1
    float t    = row / kSegments;

    // ---- Width envelope -----------------------------------------------------
    // A plain blade tapers to a point. A seeded one thins to a stalk and
    // swells at the head row before its point; a bloom swells wider still and
    // stays blunt at the top, which is what makes it read as petals rather
    // than as a fat blade.
    float halfWidth;
    float panicle = 0.0;
    if (seeded) {
        // A panicle, not a bulge: a thin stalk to halfway, then the top rows
        // thrown alternately off the centreline so the silhouette is an open
        // zigzag spray of spikelets - the way a fescue head actually branches.
        panicle = smoothstep(0.5, 0.7, t);
        float stalk = 0.55 * (1.0 - smoothstep(0.3, 0.5, t) * 0.4);
        float spikelet = 1.1 * panicle * (1.0 - 0.55 * smoothstep(0.8, 1.0, t));
        halfWidth = 0.5 * width * max(stalk, spikelet);
    } else if (bloom) {
        float stalk = (1.0 - 0.45 * smoothstep(0.3, 0.7, t));
        float head  = 2.4 * smoothstep(0.55, 0.8, t) * (1.0 - 0.6 * smoothstep(0.8, 1.0, t));
        halfWidth = 0.5 * width * max(stalk * (1.0 - smoothstep(0.55, 0.8, t)), head);
    } else {
        halfWidth = 0.5 * width * (1.0 - 0.25 * t) * (1.0 - smoothstep(0.45, 1.0, t));
    }

    // ---- Wind ---------------------------------------------------------------
    // Travelling waves. `wave` is a front about thirteen yards wide sweeping
    // along the wind at a few yards a second; `gustBand` is a far larger
    // swell rolling through on top, so the field surges and rests instead of
    // pulsing; `rustle` is the per-blade flutter riding on both. The bend
    // direction also wobbles perpendicular to the wind as a front passes,
    // which is what makes a wave look like it pushes through the grass rather
    // than dimming and brightening it.
    float windTime = fogParams.z;
    vec2 windDir = normalize(vec2(0.80, 0.60));
    float alongWind = dot(root.xy, windDir);

    float wave     = sin(alongWind * 0.48 - windTime * 2.1);
    float gustBand = 0.55 + 0.45 * sin(alongWind * 0.11 - windTime * 0.7);
    float rustle   = sin(windTime * 1.9 + dot(root.xy, vec2(0.37, 0.71))
                         + seed * 6.2831853);

    float windAmount = (0.30 + 0.50 * (0.5 + 0.5 * wave) * gustBand)
                     * (0.82 + 0.18 * rustle);
    // A bolted stem catches more wind than the sward it stands above.
    if (seeded) windAmount *= 1.25;

    vec2 perp = vec2(-windDir.y, windDir.x);
    vec2 dirNow = normalize(windDir
                            + perp * (0.30 * sin(alongWind * 0.48 - windTime * 2.1 + 1.3)
                                      + (seed - 0.5) * 0.35));
    vec2 bendVec = dirNow * (windAmount * height * kWindBend * give);

    // ---- The player brushing past -------------------------------------------
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
        bendVec += dir * (influence * height * 0.85 * give);

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

    // The spray itself: head rows swing off the stem line, alternating sides
    // row by row, further out for some blades than others. Rows are at
    // t = 0.6, 0.8, 1.0 up there, so the alternation is what the silhouette
    // is made of.
    if (panicle > 0.0) {
        float alt = (fract(row * 0.5) < 0.25) ? 1.0 : -1.0;
        float sprayReach = width * (2.0 + 4.0 * fract(seed * 41.77));
        world += across * (alt * panicle * sprayReach);
        // And the head nods: the top leans a little further leeward and down,
        // which is what a heavy seed head does that a blade tip does not.
        world.xy += dirNow * (panicle * height * 0.06);
        world.z  -= panicle * panicle * height * 0.05;
    }

    // Blade-to-blade colour, varied by as much as the profile allows. Seeded
    // from the blade, so a blade keeps its own shade rather than shimmering.
    float tint = 1.0 + (seed - 0.5) * 2.0 * profile.params.x;
    vRootColor = profile.rootColor.rgb * tint;
    vTipColor  = profile.tipColor.rgb * tint;
    vGroundColor = blade.groundColor;

    // The head's colour, applied by the fragment shader over the top of the
    // blade. Seed heads also dry the blade below them a little, which is what
    // sells "gone to seed" rather than "green grass wearing a hat".
    if (seeded) {
        vec3 headCol = mix(kSeedStraw, kSeedPlum, fract(seed * 53.71));
        vHeadColor = vec4(headCol * tint, 1.0);
        // The stem below a seed head has dried toward straw, whatever colour
        // the head above it ripened to.
        vTipColor = mix(vTipColor, kSeedStraw, 0.45);
    } else if (bloom) {
        vHeadColor = vec4(kBloomColors[int(fract(seed * 71.13) * 4.0) & 3], 1.0);
    } else {
        vHeadColor = vec4(0.0);
    }

    vHeightT = t;
    // Face out of the blade, curled a little across its width - then blended
    // toward straight up (spec 31). Grass is lit mostly as the ground it
    // stands on: a field shades like a field, and unblended per-blade normals
    // make every blade a separate glint that flickers as it moves.
    vec3 bladeNormal = normalize(cross(across, tangent) + across * (side * 0.35));
    vNormal = normalize(mix(bladeNormal, vec3(0.0, 0.0, 1.0), 0.5));

    gl_Position = projection * view * vec4(world, 1.0);
}
