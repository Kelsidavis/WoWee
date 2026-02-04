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

void main() {
    // Sample base texture
    vec4 baseColor = texture(uBaseTexture, TexCoord);
    vec4 finalColor = baseColor;

    // Apply texture layers with alpha blending
    // TexCoord = tiling UVs for texture sampling (repeats across chunk)
    // LayerUV = 0-1 per-chunk UVs for alpha map sampling
    if (uHasLayer1) {
        vec4 layer1Color = texture(uLayer1Texture, TexCoord);
        float alpha1 = texture(uLayer1Alpha, LayerUV).r;
        finalColor = mix(finalColor, layer1Color, alpha1);
    }

    if (uHasLayer2) {
        vec4 layer2Color = texture(uLayer2Texture, TexCoord);
        float alpha2 = texture(uLayer2Alpha, LayerUV).r;
        finalColor = mix(finalColor, layer2Color, alpha2);
    }

    if (uHasLayer3) {
        vec4 layer3Color = texture(uLayer3Texture, TexCoord);
        float alpha3 = texture(uLayer3Alpha, LayerUV).r;
        finalColor = mix(finalColor, layer3Color, alpha3);
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

    // Combine lighting (terrain is purely diffuse — no specular on ground)
    vec3 result = ambient + diffuse;

    // Apply fog
    float distance = length(uViewPos - FragPos);
    float fogFactor = clamp((uFogEnd - distance) / (uFogEnd - uFogStart), 0.0, 1.0);
    result = mix(uFogColor, result, fogFactor);

    FragColor = vec4(result, 1.0);
}
