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
    vec4 playerPos;
    vec4 playerWake;
    vec4 localLightPosRadius[64];
    vec4 localLightColorIntensity[64];
    ivec4 localLightMeta;
    vec4 volumetricParams;  // x = on, y = near, z = 1 / ln(far / near), w = slices
};

layout(set = 0, binding = 2) uniform sampler3D uFogVolume;

layout(push_constant) uniform Push {
    vec4 zenithColor;     // DBC skyTopColor
    vec4 midColor;        // DBC skyMiddleColor
    vec4 horizonColor;    // DBC skyBand1Color
    vec4 fogColorPush;    // DBC skyBand2Color
    vec4 sunDirAndTime;   // xyz = sun direction, w = timeOfDay
} push;

layout(location = 0) in vec2 TexCoord;

layout(location = 0) out vec4 outColor;

// The whole depth of the fog volume at this point of the screen, for the sky:
// it lies behind everything, so it takes all the air there is.
vec4 fogVolumeSky(vec2 uv) {
    return textureLod(uFogVolume, vec3(uv, 1.0), 0.0);
}

void main() {
    // Reconstruct world-space ray direction from screen position.
    float ndcX =  TexCoord.x * 2.0 - 1.0;
    float ndcY = -(TexCoord.y * 2.0 - 1.0);

    vec3 viewDir = vec3(ndcX / projection[0][0],
                        ndcY / abs(projection[1][1]),
                        -1.0);

    mat3 invViewRot = transpose(mat3(view));
    vec3 worldDir = normalize(invViewRot * viewDir);

    vec3 sunDir = push.sunDirAndTime.xyz;
    float timeOfDay = push.sunDirAndTime.w;

    // Elevation: +1 = zenith, 0 = horizon, -1 = nadir
    float elev = worldDir.z;
    float elevClamped = clamp(elev, 0.0, 1.0);

    // --- 3-band sky gradient using DBC colors ---
    // Zenith dominates upper sky, mid color fills the middle,
    // horizon band at the bottom with a thin fog fringe.
    vec3 sky;
    if (elevClamped > 0.4) {
        // Upper sky: mid -> zenith
        float t = (elevClamped - 0.4) / 0.6;
        sky = mix(push.midColor.rgb, push.zenithColor.rgb, t);
    } else if (elevClamped > 0.05) {
        // Lower sky: horizon -> mid (wide band)
        float t = (elevClamped - 0.05) / 0.35;
        sky = mix(push.horizonColor.rgb, push.midColor.rgb, t);
    } else {
        // Thin fog fringe right at horizon
        float t = elevClamped / 0.05;
        sky = mix(push.fogColorPush.rgb, push.horizonColor.rgb, t);
    }

    // --- Below-horizon darkening (nadir) ---
    if (elev < 0.0) {
        float nadirFade = clamp(-elev * 3.0, 0.0, 1.0);
        vec3 nadirColor = push.fogColorPush.rgb * 0.3;
        sky = mix(push.fogColorPush.rgb, nadirColor, nadirFade);
    }

    // --- Rayleigh-like scattering (subtle warm glow near sun) ---
    float sunDot = max(dot(worldDir, sunDir), 0.0);
    float sunAboveHorizon = clamp(sunDir.z, 0.0, 1.0);

    float rayleighStrength = pow(1.0 - elevClamped, 3.0) * 0.15;
    vec3 scatterColor = mix(vec3(0.8, 0.45, 0.15), vec3(0.3, 0.5, 1.0), elevClamped);
    sky += scatterColor * rayleighStrength * sunDot * sunAboveHorizon;

    // --- Mie-like forward scatter (sun disk glow) ---
    float mieSharp = pow(sunDot, 64.0) * 0.4;
    float mieSoft  = pow(sunDot, 8.0) * 0.1;
    vec3 sunGlowColor = mix(vec3(1.0, 0.85, 0.55), vec3(1.0, 1.0, 0.95), elevClamped);
    sky += sunGlowColor * (mieSharp + mieSoft) * sunAboveHorizon;

    // --- Subtle horizon haze ---
    float hazeDensity = exp(-elevClamped * 12.0) * 0.06;
    sky += push.horizonColor.rgb * hazeDensity * sunAboveHorizon;

    // --- Night: slight moonlight tint ---
    if (sunDir.z < 0.0) {
        float moonlight = clamp(-sunDir.z * 0.5, 0.0, 0.15);
        sky += vec3(0.02, 0.03, 0.08) * moonlight;
    }

    if (volumetricParams.x > 0.5) {
        vec4 air = fogVolumeSky(TexCoord);
        sky = sky * air.a + air.rgb;
    }

    outColor = vec4(sky, 1.0);
}
