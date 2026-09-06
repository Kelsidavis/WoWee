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

layout(push_constant) uniform Push {
    mat4 model;
    float waveAmp;
    float waveFreq;
    float waveSpeed;
    float liquidBasicType; // 0=water, 1=ocean, 2=magma, 3=slime
    // The rest belongs to the fragment stage, and is spelled the same way here
    // so the two declarations of one range agree. This said `float brightness`
    // where the fragment stage has a vec2, which is a field that does not exist
    // sitting at another field's offset - harmless only for as long as nobody
    // reads it.
    vec2 screenSize;
    vec2 depthRange;
    float sceneValid;
    float pad0, pad1, pad2;
} push;

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec2 aTexCoord;

layout(location = 0) out vec3 FragPos;
layout(location = 1) out vec3 Normal;
layout(location = 2) out vec2 TexCoord;
layout(location = 3) out float WaveOffset;
layout(location = 4) out vec2 ScreenUV;

// --- Gerstner wave ---
// Coordinate system: X,Y = horizontal plane, Z = up (height)
// displacement.xy = horizontal, displacement.z = vertical
struct GerstnerResult {
    vec3 displacement;
    vec3 tangent;   // along X
    vec3 binormal;  // along Y
    float waveHeight; // raw wave height for foam
};

GerstnerResult evaluateGerstnerWaves(vec2 pos, float time, float amp, float freq, float spd, float basicType) {
    GerstnerResult r;
    r.displacement = vec3(0.0);
    r.tangent = vec3(1.0, 0.0, 0.0);
    r.binormal = vec3(0.0, 1.0, 0.0);
    r.waveHeight = 0.0;

    // Magma/slime: simple slow undulation
    if (basicType >= 1.5) {
        float wave = sin(pos.x * freq * 0.5 + time * spd * 0.3) * 0.4
                   + sin(pos.y * freq * 0.3 + time * spd * 0.5) * 0.3;
        r.displacement.z = wave * amp * 0.5;
        float dx = cos(pos.x * freq * 0.5 + time * spd * 0.3) * freq * 0.5 * amp * 0.5 * 0.4;
        float dy = cos(pos.y * freq * 0.3 + time * spd * 0.5) * freq * 0.3 * amp * 0.5 * 0.3;
        r.tangent = vec3(1.0, 0.0, dx);
        r.binormal = vec3(0.0, 1.0, dy);
        r.waveHeight = wave;
        return r;
    }

    // 6 wave directions for more chaotic, natural-looking water
    // Spread across many angles to avoid visible patterns
    vec2 dirs[6] = vec2[6](
        normalize(vec2(0.86, 0.51)),
        normalize(vec2(-0.47, 0.88)),
        normalize(vec2(0.32, -0.95)),
        normalize(vec2(-0.93, -0.37)),
        normalize(vec2(0.67, -0.29)),
        normalize(vec2(-0.15, 0.74))
    );
    float amps[6];
    float freqs[6];
    float spds_arr[6];
    float steepness[6];

    if (basicType > 0.5) {
        // Ocean: broader range of wave scales for realistic chop
        amps[0] = amp * 1.0;   amps[1] = amp * 0.55;  amps[2] = amp * 0.30;
        amps[3] = amp * 0.18;  amps[4] = amp * 0.10;  amps[5] = amp * 0.06;
        freqs[0] = freq * 0.7; freqs[1] = freq * 1.3;  freqs[2] = freq * 2.1;
        freqs[3] = freq * 3.4; freqs[4] = freq * 5.0;  freqs[5] = freq * 7.5;
        spds_arr[0] = spd * 0.8; spds_arr[1] = spd * 1.0; spds_arr[2] = spd * 1.3;
        spds_arr[3] = spd * 1.6; spds_arr[4] = spd * 2.0; spds_arr[5] = spd * 2.5;
        steepness[0] = 0.35; steepness[1] = 0.30; steepness[2] = 0.25;
        steepness[3] = 0.20; steepness[4] = 0.15; steepness[5] = 0.10;
    } else {
        // Inland water: gentle but multi-scale ripples
        amps[0] = amp * 0.5;   amps[1] = amp * 0.25;  amps[2] = amp * 0.15;
        amps[3] = amp * 0.08;  amps[4] = amp * 0.05;  amps[5] = amp * 0.03;
        freqs[0] = freq * 1.0; freqs[1] = freq * 1.8;  freqs[2] = freq * 3.0;
        freqs[3] = freq * 4.5; freqs[4] = freq * 7.0;  freqs[5] = freq * 10.0;
        spds_arr[0] = spd * 0.6; spds_arr[1] = spd * 0.9; spds_arr[2] = spd * 1.2;
        spds_arr[3] = spd * 1.5; spds_arr[4] = spd * 1.9; spds_arr[5] = spd * 2.3;
        steepness[0] = 0.20; steepness[1] = 0.18; steepness[2] = 0.15;
        steepness[3] = 0.12; steepness[4] = 0.10; steepness[5] = 0.08;
    }

    float totalWave = 0.0;
    for (int i = 0; i < 6; i++) {
        float w = freqs[i];
        float A = amps[i];
        float phi = spds_arr[i] * w; // phase speed
        float Q = steepness[i] / (w * A * 6.0);
        Q = clamp(Q, 0.0, 1.0);

        float phase = w * dot(dirs[i], pos) + phi * time;
        float s = sin(phase);
        float c = cos(phase);

        // Gerstner displacement: xy = horizontal, z = vertical (up)
        r.displacement.x += Q * A * dirs[i].x * c;
        r.displacement.y += Q * A * dirs[i].y * c;
        r.displacement.z += A * s;

        // Tangent/binormal accumulation for analytical normal
        float WA = w * A;
        r.tangent.x -= Q * dirs[i].x * dirs[i].x * WA * s;
        r.tangent.y -= Q * dirs[i].x * dirs[i].y * WA * s;
        r.tangent.z += dirs[i].x * WA * c;

        r.binormal.x -= Q * dirs[i].x * dirs[i].y * WA * s;
        r.binormal.y -= Q * dirs[i].y * dirs[i].y * WA * s;
        r.binormal.z += dirs[i].y * WA * c;

        totalWave += A * s;
    }

    r.waveHeight = totalWave;
    return r;
}

void main() {
    float time = fogParams.z;
    vec4 worldPos = push.model * vec4(aPos, 1.0);

    // Evaluate Gerstner waves using X,Y horizontal plane
    GerstnerResult waves = evaluateGerstnerWaves(
        vec2(worldPos.x, worldPos.y), time,
        push.waveAmp, push.waveFreq, push.waveSpeed, push.liquidBasicType
    );

    // Apply displacement: xy = horizontal, z = vertical (up)
    worldPos.x += waves.displacement.x;
    worldPos.y += waves.displacement.y;
    worldPos.z += waves.displacement.z;
    WaveOffset = waves.waveHeight; // raw wave height for fragment shader foam

    // Player interaction ripples - concentric waves emanating from player position
    vec2 rippleOrigin = playerPos.xy;
    float rippleStrength = fogParams.w;
    float d = length(worldPos.xy - rippleOrigin);
    float ripple = rippleStrength * 0.12 * exp(-d * 0.12) * sin(d * 2.5 - time * 6.0);
    worldPos.z += ripple;

    // Analytical normal from Gerstner tangent/binormal (cross product gives Z-up normal)
    Normal = normalize(cross(waves.binormal, waves.tangent));

    FragPos = worldPos.xyz;
    TexCoord = aTexCoord;
    vec4 clipPos = projection * view * worldPos;
    gl_Position = clipPos;
    vec2 ndc = clipPos.xy / max(clipPos.w, 1e-5);
    ScreenUV = ndc * 0.5 + 0.5;
}
