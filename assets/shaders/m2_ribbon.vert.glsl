#version 450

// M2 ribbon emitter vertex shader.
// Ribbon geometry is generated CPU-side as a triangle strip.
// Vertex format: pos(3) + color(3) + alpha(1) + uv(2) = 9 floats.

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

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColor;
layout(location = 2) in float aAlpha;
layout(location = 3) in vec2 aUV;

layout(location = 0) out vec3 vColor;
layout(location = 1) out float vAlpha;
layout(location = 2) out vec2 vUV;
layout(location = 3) out float vFogFactor;

// The air between the camera and this point, out of the fog volume: rgb is
// the light it scatters toward the camera, a how much of the point shows
// through it. See VolumetricFog.
vec4 fogVolumeAt(vec3 worldPos) {
    vec4 clip = projection * view * vec4(worldPos, 1.0);
    float depth = max(clip.w, 1e-4);
    vec2 uv = clip.xy / depth * 0.5 + 0.5;
    float slice = log(max(depth, volumetricParams.y) / volumetricParams.y) * volumetricParams.z;
    // Each slice holds the air up to its far edge, so a point is read half a
    // slice back from where it stands.
    return textureLod(uFogVolume, vec3(uv, slice - 0.5 / volumetricParams.w), 0.0);
}

void main() {
    vec4 worldPos = vec4(aPos, 1.0);
    vec4 viewPos4  = view * worldPos;
    gl_Position    = projection * viewPos4;

    float dist      = length(viewPos4.xyz);
    float fogStart  = fogParams.x;
    float fogEnd    = fogParams.y;
    vFogFactor      = clamp((fogEnd - dist) / max(fogEnd - fogStart, 0.001), 0.0, 1.0);
    if (volumetricParams.x > 0.5) vFogFactor *= fogVolumeAt(aPos).a;

    vColor = aColor;
    vAlpha = aAlpha;
    vUV    = aUV;
}
