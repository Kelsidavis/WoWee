#version 450

layout(set = 0, binding = 0) uniform PerFrame {
    mat4 view;
    mat4 projection;
    mat4 lightSpaceMatrix;
    vec4 lightDir;
    vec4 lightColor;
    vec4 ambientColor;
    vec4 viewPos;
    vec4 fogColor;
    vec4 fogParams;
    vec4 shadowParams;
    vec4 playerPos;   // xyz = player world position, w = horizontal speed
    vec4 playerWake;  // xyz = trailing player position (springback reference)
};

// Per-draw push constants (batch-level data only)
layout(push_constant) uniform Push {
    int texCoordSet;         // UV set index (0 or 1)
    int isFoliage;           // -1 sky, 0 none, 1 foliage, 2 clutter, 3 hanging cloth
    int instanceDataOffset;  // Base index into InstanceSSBO for this draw group
    float swayRefHeight;     // Model height the sway normalises against (model space)
    float swayAmp;           // Sway amplitude scale; 1.0 = the tree-sized default
    float plantHeight;       // The model's own height (model space), for the player brush
} push;

layout(set = 2, binding = 0) readonly buffer BoneSSBO {
    mat4 bones[];
};

// Per-instance data read via gl_InstanceIndex (GPU instancing)
struct InstanceData {
    mat4 model;
    vec2 uvOffset;
    float fadeAlpha;
    int useBones;
    int boneBase;
    int boneCount;
    // 0 for an ordinary instance, 1 while the player is pressing on it. Sits
    // in what was padding, so the entry is the same 96 bytes it always was.
    float highlight;
};
layout(set = 3, binding = 0) readonly buffer InstanceSSBO {
    InstanceData instanceData[];
};

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aTexCoord;
layout(location = 3) in vec4 aBoneWeights;
layout(location = 4) in vec4 aBoneIndicesF;
layout(location = 5) in vec2 aTexCoord2;

layout(location = 0) out vec3 FragPos;
layout(location = 1) out vec3 Normal;
layout(location = 2) out vec2 TexCoord;
layout(location = 3) flat out vec3 InstanceOrigin;
layout(location = 4) out float ModelHeight;
layout(location = 5) out float vFadeAlpha;
layout(location = 6) flat out int vSkyMode;
layout(location = 7) flat out float vHighlight;

void main() {
    // Fetch per-instance data from SSBO
    int instIdx = push.instanceDataOffset + gl_InstanceIndex;
    mat4 model = instanceData[instIdx].model;
    vec2 uvOff = instanceData[instIdx].uvOffset;
    float fade = instanceData[instIdx].fadeAlpha;
    int uBones = instanceData[instIdx].useBones;
    int bBase  = instanceData[instIdx].boneBase;

    vec4 pos = vec4(aPos, 1.0);
    vec4 norm = vec4(aNormal, 0.0);

    if (uBones != 0) {
        // Clamp to the range this instance actually owns. A model whose bone
        // count exceeds the ceiling would otherwise index into the next
        // instance's matrices and fling its vertices across the world.
        int bCount = max(instanceData[instIdx].boneCount, 1);
        ivec4 bi = clamp(ivec4(aBoneIndicesF), ivec4(0), ivec4(bCount - 1));
        mat4 skinMat = bones[bBase + bi.x] * aBoneWeights.x
                     + bones[bBase + bi.y] * aBoneWeights.y
                     + bones[bBase + bi.z] * aBoneWeights.z
                     + bones[bBase + bi.w] * aBoneWeights.w;
        pos = skinMat * pos;
        norm = skinMat * norm;
    }

    // How far up the model this vertex sits, 0 at the base and 1 at the top.
    // Quadratic so the roots stay planted and the motion collects at the tip.
    // swayRefHeight is 20 for anything tree-sized, which is the constant this
    // used to hardcode; ground clutter passes its own height instead, because
    // normalising a one-yard tuft against twenty moves it by nothing at all.
    float heightFactor = 0.0;
    if (push.isFoliage > 0) {
        heightFactor = clamp(pos.z / max(push.swayRefHeight, 0.01), 0.0, 1.0);
        heightFactor *= heightFactor; // quadratic - base stays grounded
    }

    // Wind animation for foliage.
    //
    // Ground clutter (mode 2) is left out of this on purpose. Every detail
    // doodad has its own one-bone sequence and plays it, so a shader wind on
    // top would be two swings of the same plant at two different rates. The
    // player brush below still applies to it: that is motion the authored
    // animation has no way to know about.
    if (push.isFoliage == 1) {
        float windTime = fogParams.z;
        vec3 worldRef = model[3].xyz;
        float amp = push.swayAmp * heightFactor;

        // Layer 1: Trunk sway - slow, large amplitude
        float trunkPhase = windTime * 0.8 + dot(worldRef.xy, vec2(0.1, 0.13));
        float trunkSwayX = sin(trunkPhase) * 0.35 * amp;
        float trunkSwayY = cos(trunkPhase * 0.7) * 0.25 * amp;

        // Layer 2: Branch sway - medium frequency, per-branch phase
        float branchPhase = windTime * 1.7 + dot(worldRef.xy, vec2(0.37, 0.71));
        float branchSwayX = sin(branchPhase + pos.y * 0.4) * 0.15 * amp;
        float branchSwayY = cos(branchPhase * 1.1 + pos.x * 0.3) * 0.12 * amp;

        // Layer 3: Leaf flutter - fast, small amplitude, per-vertex
        float leafPhase = windTime * 4.5 + dot(aPos, vec3(1.7, 2.3, 0.9));
        float leafFlutterX = sin(leafPhase) * 0.06 * amp;
        float leafFlutterY = cos(leafPhase * 1.3) * 0.05 * amp;

        pos.x += trunkSwayX + branchSwayX + leafFlutterX;
        pos.y += trunkSwayY + branchSwayY + leafFlutterY;
    }

    // Cloth hung from its top edge: a banner, a flag, a tapestry.
    //
    // The opposite taper to a plant's. A tree is held at the root and swings
    // at the tip; a banner is nailed to its bar and moves at the hem, so the
    // weight is the drop below the top rather than the height above the base -
    // which also leaves the pole or the bar in the same model still, its
    // vertices being at the top where the weight is zero.
    //
    // Two rates: a slow sway of the whole cloth and a smaller ripple across it,
    // so it breathes rather than swinging like a sign. The amplitude is a
    // twentieth of the cloth's own drop, set on the client side.
    if (push.isFoliage == 3) {
        float windTime = fogParams.z;
        vec3 worldRef = model[3].xyz;
        float span = max(push.plantHeight, 0.01);
        float drop = clamp((push.swayRefHeight - pos.z) / span, 0.0, 1.0);
        drop *= drop;
        float amp = push.swayAmp * drop;

        // Per banner rather than per vertex, so two on the same wall are not
        // in step - the phase comes from where the instance stands.
        float phase = windTime * 1.1 + dot(worldRef.xy, vec2(0.21, 0.17));
        vec3 sway = vec3(
            (sin(phase) * 0.7 + sin(phase * 2.7 + pos.y * 1.9) * 0.25) * amp,
            (cos(phase * 0.9) * 0.6 + cos(phase * 3.1 + pos.x * 1.7) * 0.2) * amp,
            // A little in and out as well, so the cloth is not a flat sheet
            // sliding sideways.
            sin(phase * 1.6 + pos.x * 1.3) * 0.12 * amp);

        // Away from the wall, never into it.
        //
        // A banner hangs flush against stone, so any motion toward its own
        // back face goes through the masonry - a quarter of it was still
        // enough to show. The whole perpendicular component is taken out and a
        // third of it given back outward only: the cloth billows away from the
        // wall and slides in its own plane, and the half-cycle that used to
        // push it backwards now does nothing at all.
        //
        // Outward is each face's own normal, so a two-sided banner puffs
        // slightly rather than parting - which is what a cloth in a draught
        // does anyway.
        vec3 clothN = normalize(norm.xyz);
        float perp = dot(sway, clothN);
        sway -= clothN * perp;
        sway += clothN * max(perp, 0.0) * 0.35;
        pos.xyz += sway;
    }

    vec4 worldPos = model * pos;

    // Foliage parts around whoever walks through it, then springs back. Applied
    // in world space after the model transform: the displacement is a distance
    // in yards from the player, not something the model's own scale and rotation
    // should be turning.
    //
    // Grass, ferns and bushes all give way; a tree does not, and the taper
    // between them is on the plant's own height rather than on which of the two
    // sway modes it happens to use. A shoulder-high bush and a waist-high one
    // should not behave differently because a bounding box crossed a threshold.
    if (push.isFoliage > 0 && push.isFoliage != 3 && push.plantHeight > 0.0) {
        vec3 base = model[3].xyz;                      // instance origin, on the ground
        float zScale = length(model[2].xyz);
        float plantHeight = push.plantHeight * zScale;

        // Full effect up to about head height, nothing from a small tree up.
        float sizeGate = 1.0 - smoothstep(4.0, 8.0, plantHeight);

        // Only foliage at the player's own level reacts. Flying over a field
        // must not flatten it, and neither must standing on the roof above it.
        float levelGate = 1.0 - smoothstep(1.5, 4.0, abs(base.z - playerPos.z));
        levelGate *= sizeGate;

        // Height along the plant, from its own base rather than from whatever
        // the wind normalised against: mode 2 does not compute the wind at all.
        float brushT = clamp(pos.z / max(push.plantHeight, 0.01), 0.0, 1.0);
        brushT *= brushT;

        if (levelGate > 0.0 && brushT > 0.0) {
            // Reach grows a little with the plant, so a waist-high fern gives
            // way sooner than a tuft of grass does.
            float reach = 1.1 + plantHeight * 0.35;

            // Bend away from the player, and from where the player was a
            // moment ago. Taking the stronger of the two rather than the sum
            // keeps a standing player - where the two points coincide - from
            // bending the clutter twice as far as a walking one.
            vec2 toNow  = base.xy - playerPos.xy;
            vec2 toWake = base.xy - playerWake.xy;
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

            // Bend, capped so tall clutter doesn't lie flat on the ground.
            float bend = influence * brushT * min(plantHeight * 0.45, 0.75);

            // Rustle: a fast quiver riding on the bend, present only while the
            // player is actually moving. Phase is per-plant, so a field
            // shivers rather than pulsing in unison.
            float speed = clamp(playerPos.w / 7.0, 0.0, 1.0);
            float rustlePhase = fogParams.z * 17.0 + dot(base.xy, vec2(3.1, 2.7));
            float rustle = sin(rustlePhase) * influence * brushT
                         * speed * min(plantHeight * 0.10, 0.15);

            worldPos.xy += dir * bend + vec2(-dir.y, dir.x) * rustle;
            // Trodden clutter also settles a little, rather than pivoting on
            // its base and standing just as tall.
            worldPos.z -= influence * brushT * plantHeight * 0.12;
        }
    }

    FragPos = worldPos.xyz;
    Normal = mat3(model) * norm.xyz;

    TexCoord = (push.texCoordSet == 1 ? aTexCoord2 : aTexCoord) + uvOff;

    InstanceOrigin = model[3].xyz;
    ModelHeight = pos.z;
    vFadeAlpha = fade;
    vSkyMode = push.isFoliage < 0 ? 1 : 0;
    vHighlight = instanceData[instIdx].highlight;

    gl_Position = projection * view * worldPos;
    // A sky model sits on the far plane whatever its radius, so the depth
    // test rejects it wherever terrain has already been drawn. The sky is
    // drawn after the ground now, and this is what makes that free.
    if (push.isFoliage < 0) gl_Position.z = gl_Position.w;
}
