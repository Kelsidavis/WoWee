#version 330 core

in vec3 FragPos;
in vec3 Normal;
in vec2 TexCoord;
in vec2 LayerUV;

out vec4 FragColor;

// Texture layers (up to 4)
uniform sampler2D uBaseTexture;
uniform sampler2D uLayer1Texture;
uniform sampler2D uLayer2Texture;
uniform sampler2D uLayer3Texture;

// Alpha maps for blending
uniform sampler2D uLayer1Alpha;
uniform sampler2D uLayer2Alpha;
uniform sampler2D uLayer3Alpha;

// Layer control
uniform int uLayerCount;
uniform bool uHasLayer1;
uniform bool uHasLayer2;
uniform bool uHasLayer3;

// Lighting
uniform vec3 uLightDir;
uniform vec3 uLightColor;
uniform vec3 uAmbientColor;

// Camera
uniform vec3 uViewPos;

// Fog
uniform vec3 uFogColor;
uniform float uFogStart;
uniform float uFogEnd;

// Shadow mapping
uniform sampler2DShadow uShadowMap;
uniform mat4 uLightSpaceMatrix;
uniform bool uShadowEnabled;
uniform float uShadowStrength;

float calcShadow() {
    vec4 lsPos = uLightSpaceMatrix * vec4(FragPos, 1.0);
    vec3 proj = lsPos.xyz / lsPos.w * 0.5 + 0.5;
    if (proj.z > 1.0) return 1.0;
    float edgeDist = max(abs(proj.x - 0.5), abs(proj.y - 0.5));
    float coverageFade = 1.0 - smoothstep(0.40, 0.49, edgeDist);
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(-uLightDir);
    float bias = max(0.005 * (1.0 - dot(norm, lightDir)), 0.001);
    // 5-tap PCF tuned for slightly sharper detail while keeping stability.
    vec2 texel = vec2(1.0 / 1536.0);
    float ref = proj.z - bias;
    vec2 off = texel * 0.7;
    float shadow = 0.0;
    shadow += texture(uShadowMap, vec3(proj.xy, ref)) * 0.55;
    shadow += texture(uShadowMap, vec3(proj.xy + vec2(off.x, 0.0), ref)) * 0.1125;
    shadow += texture(uShadowMap, vec3(proj.xy - vec2(off.x, 0.0), ref)) * 0.1125;
    shadow += texture(uShadowMap, vec3(proj.xy + vec2(0.0, off.y), ref)) * 0.1125;
    shadow += texture(uShadowMap, vec3(proj.xy - vec2(0.0, off.y), ref)) * 0.1125;
    return mix(1.0, shadow, coverageFade);
}

float sampleAlpha(sampler2D tex, vec2 uv) {
    // Slight blur near alpha-map borders to hide seams between chunks.
    vec2 edge = min(uv, 1.0 - uv);
    float border = min(edge.x, edge.y);
    float doBlur = step(border, 2.0 / 64.0); // within ~2 texels of edge
    if (doBlur < 0.5) {
        return texture(tex, uv).r;
    }
    vec2 texel = vec2(1.0 / 64.0);
    float a = 0.0;
    a += texture(tex, uv + vec2(-texel.x, 0.0)).r;
    a += texture(tex, uv + vec2(texel.x, 0.0)).r;
    a += texture(tex, uv + vec2(0.0, -texel.y)).r;
    a += texture(tex, uv + vec2(0.0, texel.y)).r;
    return a * 0.25;
}

void main() {
    // Sample base texture
    vec4 baseColor = texture(uBaseTexture, TexCoord);
    vec4 finalColor = baseColor;

    // Apply texture layers with alpha blending
    // TexCoord = tiling UVs for texture sampling (repeats across chunk)
    // LayerUV = 0-1 per-chunk UVs for alpha map sampling
    float a1 = uHasLayer1 ? sampleAlpha(uLayer1Alpha, LayerUV) : 0.0;
    float a2 = uHasLayer2 ? sampleAlpha(uLayer2Alpha, LayerUV) : 0.0;
    float a3 = uHasLayer3 ? sampleAlpha(uLayer3Alpha, LayerUV) : 0.0;

    // Normalize weights to reduce quilting seams at chunk borders.
    float w0 = 1.0;
    float w1 = a1;
    float w2 = a2;
    float w3 = a3;
    float sum = w0 + w1 + w2 + w3;
    if (sum > 0.0) {
        w0 /= sum; w1 /= sum; w2 /= sum; w3 /= sum;
    }

    finalColor = baseColor * w0;
    if (uHasLayer1) {
        vec4 layer1Color = texture(uLayer1Texture, TexCoord);
        finalColor += layer1Color * w1;
    }
    if (uHasLayer2) {
        vec4 layer2Color = texture(uLayer2Texture, TexCoord);
        finalColor += layer2Color * w2;
    }
    if (uHasLayer3) {
        vec4 layer3Color = texture(uLayer3Texture, TexCoord);
        finalColor += layer3Color * w3;
    }

    // Normalize normal
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(-uLightDir);

    // Ambient lighting
    vec3 ambient = uAmbientColor * finalColor.rgb;

    // Diffuse lighting (two-sided for terrain hills)
    float diff = abs(dot(norm, lightDir));
    diff = max(diff, 0.2);  // Minimum light to prevent completely dark faces
    vec3 diffuse = diff * uLightColor * finalColor.rgb;

    // Shadow
    float shadow = uShadowEnabled ? calcShadow() : 1.0;
    shadow = mix(1.0, shadow, clamp(uShadowStrength, 0.0, 1.0));

    // Combine lighting (terrain is purely diffuse — no specular on ground)
    vec3 result = ambient + shadow * diffuse;

    // Apply fog
    float distance = length(uViewPos - FragPos);
    float fogFactor = clamp((uFogEnd - distance) / (uFogEnd - uFogStart), 0.0, 1.0);
    result = mix(uFogColor, result, fogFactor);

    FragColor = vec4(result, 1.0);
}
