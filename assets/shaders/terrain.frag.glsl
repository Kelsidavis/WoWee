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
    vec4 localLightPosRadius[64];
    vec4 localLightColorIntensity[64];
    ivec4 localLightMeta;
};

layout(set = 1, binding = 0) uniform sampler2D uBaseTexture;
layout(set = 1, binding = 1) uniform sampler2D uLayer1Texture;
layout(set = 1, binding = 2) uniform sampler2D uLayer2Texture;
layout(set = 1, binding = 3) uniform sampler2D uLayer3Texture;
layout(set = 1, binding = 4) uniform sampler2D uLayer1Alpha;
layout(set = 1, binding = 5) uniform sampler2D uLayer2Alpha;
layout(set = 1, binding = 6) uniform sampler2D uLayer3Alpha;

layout(set = 1, binding = 7) uniform TerrainParams {
    int layerCount;
    int hasLayer1;
    int hasLayer2;
    int hasLayer3;
};

layout(set = 0, binding = 1) uniform sampler2DShadow uShadowMap;

layout(location = 0) in vec3 FragPos;
layout(location = 1) in vec3 Normal;
layout(location = 2) in vec2 TexCoord;
layout(location = 3) in vec2 LayerUV;

layout(location = 0) out vec4 outColor;

// One texel of the shadow map, handed in by the renderer. The map is 512,
// 1024, 2048 or 4096 a side by the quality setting; this used to be a
// constant for 4096, so at 512 the filter taps all landed inside one texel
// and the bias shrank eightfold. The fallback covers a per-frame block that
// never filled the slot in, such as the character preview's.
float shadowTexel() {
    return shadowParams.z > 0.0 ? shadowParams.z : 1.0 / 4096.0;
}

float sampleShadowPCF(sampler2DShadow smap, vec3 coords) {
    float shadow = 0.0;
    for (int x = -1; x <= 1; ++x) {
        for (int y = -1; y <= 1; ++y) {
            shadow += texture(smap, vec3(coords.xy + vec2(x, y) * shadowTexel(), coords.z));
        }
    }
    return shadow / 9.0;
}

vec3 localLightContribution(vec3 pos, vec3 normal, vec3 albedo) {
    vec3 sum = vec3(0.0);
    for (int i = 0; i < min(localLightMeta.x, 64); ++i) {
        vec3 toLight = localLightPosRadius[i].xyz - pos;
        float radius = localLightPosRadius[i].w;
        // Rejected on the squared distance, before the square root and the
        // divide: most of the sixty-four are out of range of any one pixel,
        // and this is what each of them costs.
        float distSq = dot(toLight, toLight);
        if (radius <= 0.0 || distSq >= radius * radius) continue;
        float dist = sqrt(distSq);
        float attenuation = 1.0 - dist / radius;
        attenuation *= attenuation;
        float wrappedDiffuse = 0.22 + 0.78 * max(dot(normal, toLight / max(dist, 0.001)), 0.0);
        sum += albedo * localLightColorIntensity[i].rgb *
               (localLightColorIntensity[i].w * attenuation * wrappedDiffuse);
    }
    return sum;
}

float sampleAlpha(sampler2D tex, vec2 uv) {
    // Smooth 9-tap box near chunk edges to hide alpha-map seams;
    // blends gradually to avoid a visible ring at the transition.
    // Wider feather (8 texels) makes per-chunk alpha differences
    // bleed across the boundary so the chunk grid stops reading
    // as a hard step.
    vec2 edge = min(uv, 1.0 - uv);
    float border = min(edge.x, edge.y);
    float blurWeight = 1.0 - smoothstep(1.0 / 64.0, 8.0 / 64.0, border);
    float center = texture(tex, uv).r;
    if (blurWeight < 0.001) return center;
    // Four taps at half-texel offsets, not nine at whole ones. The sampler
    // is linear, so each tap already averages a 2x2 block, and the four
    // together cover the same 3x3 footprint as a tent rather than a box.
    // The band this runs in is 44% of every chunk, on the pass that
    // covers most of the screen, so the tap count is what this costs.
    vec2 h = vec2(0.5 / 64.0);
    float avg = texture(tex, uv + vec2(-h.x, -h.y)).r
              + texture(tex, uv + vec2( h.x, -h.y)).r
              + texture(tex, uv + vec2(-h.x,  h.y)).r
              + texture(tex, uv + vec2( h.x,  h.y)).r;
    avg *= 0.25;
    return mix(center, avg, blurWeight);
}

void main() {
    vec4 baseColor = texture(uBaseTexture, TexCoord);

    // WoW terrain: layers are blended sequentially, each on top of the previous result.
    // Alpha=1 means the layer fully covers everything below; alpha=0 means invisible.
    vec4 finalColor = baseColor;
    if (hasLayer1 != 0) {
        float a1 = sampleAlpha(uLayer1Alpha, LayerUV);
        finalColor = mix(finalColor, texture(uLayer1Texture, TexCoord), a1);
    }
    if (hasLayer2 != 0) {
        float a2 = sampleAlpha(uLayer2Alpha, LayerUV);
        finalColor = mix(finalColor, texture(uLayer2Texture, TexCoord), a2);
    }
    if (hasLayer3 != 0) {
        float a3 = sampleAlpha(uLayer3Alpha, LayerUV);
        finalColor = mix(finalColor, texture(uLayer3Texture, TexCoord), a3);
    }

    vec3 norm = normalize(Normal);

    // Derivative-based normal mapping: perturb vertex normal using texture detail.
    // Fade out with distance and near chunk edges (dFdx/dFdy are invalid across
    // chunk draw-call boundaries, producing visible seams if not faded).
    float fragDist = length(viewPos.xyz - FragPos);
    float bumpFade = 1.0 - smoothstep(50.0, 125.0, fragDist);
    float edgeDist = min(min(LayerUV.x, 1.0 - LayerUV.x), min(LayerUV.y, 1.0 - LayerUV.y));
    bumpFade *= smoothstep(0.0, 0.06, edgeDist);
    if (bumpFade > 0.001) {
        float lum = dot(finalColor.rgb, vec3(0.299, 0.587, 0.114));
        float dLdx = dFdx(lum);
        float dLdy = dFdy(lum);
        vec3 dpdx = dFdx(FragPos);
        vec3 dpdy = dFdy(FragPos);
        float bumpStrength = 9.0 * bumpFade;
        vec3 perturbation = (dLdx * cross(norm, dpdy) + dLdy * cross(dpdx, norm)) * bumpStrength;
        vec3 candidate = norm - perturbation;
        float len2 = dot(candidate, candidate);
        norm = (len2 > 1e-8) ? candidate * inversesqrt(len2) : norm;
    }

    vec3 lightDir2 = normalize(-lightDir.xyz);
    vec3 ambient = ambientColor.rgb * finalColor.rgb;
    // Lambert, as every other surface has it. This took abs() of the angle
    // and floored it at 0.2, so a slope turned from the sun was lit as one
    // turned toward it, and hills had no shape at low sun. The ambient term
    // is what keeps the shaded side from black, as it does for the models
    // standing on it.
    float diff = max(dot(norm, lightDir2), 0.0);
    vec3 diffuse = diff * lightColor.rgb * finalColor.rgb;

    float shadow = 1.0;
    if (shadowParams.x > 0.5) {
        vec3 ldir = normalize(-lightDir.xyz);
        float normalOffset = shadowTexel() * 2.0 * (1.0 - abs(dot(norm, ldir)));
        vec3 biasedPos = FragPos + norm * normalOffset;
        vec4 lsPos = lightSpaceMatrix * vec4(biasedPos, 1.0);
        vec3 proj = lsPos.xyz / lsPos.w;
        proj.xy = proj.xy * 0.5 + 0.5;
        if (proj.x >= 0.0 && proj.x <= 1.0 && proj.y >= 0.0 && proj.y <= 1.0 && proj.z >= 0.0 && proj.z <= 1.0) {
            float bias = max(0.0005 * (1.0 - abs(dot(norm, ldir))), 0.00005);
            shadow = sampleShadowPCF(uShadowMap, vec3(proj.xy, proj.z - bias));
            shadow = mix(1.0, shadow, shadowParams.y);
        }
    }

    vec3 result = ambient + shadow * diffuse;
    result += localLightContribution(FragPos, norm, finalColor.rgb);

    float fogFactor = clamp((fogParams.y - fragDist) / (fogParams.y - fogParams.x), 0.0, 1.0);
    result = mix(fogColor.rgb, result, fogFactor);

    outColor = vec4(result, 1.0);
}
