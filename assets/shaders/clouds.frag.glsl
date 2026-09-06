#version 450

layout(push_constant) uniform Push {
    vec4 cloudColor;      // xyz = DBC-derived base cloud color, w = unused
    vec4 sunDirDensity;   // xyz = sun direction, w = density
    vec4 windAndLight;    // x = windOffset, y = sunIntensity, z = ambient, w = unused
} push;

layout(location = 0) in vec3 vWorldDir;

layout(location = 0) out vec4 outColor;

// --- Gradient noise (smoother than hash-based) ---
vec2 hash2(vec2 p) {
    p = vec2(dot(p, vec2(127.1, 311.7)),
             dot(p, vec2(269.5, 183.3)));
    return fract(sin(p) * 43758.5453);
}

float gradientNoise(vec2 p) {
    vec2 i = floor(p);
    vec2 f = fract(p);

    // Quintic interpolation for smoother results
    vec2 u = f * f * f * (f * (f * 6.0 - 15.0) + 10.0);

    float a = dot(hash2(i + vec2(0.0, 0.0)) * 2.0 - 1.0, f - vec2(0.0, 0.0));
    float b = dot(hash2(i + vec2(1.0, 0.0)) * 2.0 - 1.0, f - vec2(1.0, 0.0));
    float c = dot(hash2(i + vec2(0.0, 1.0)) * 2.0 - 1.0, f - vec2(0.0, 1.0));
    float d = dot(hash2(i + vec2(1.0, 1.0)) * 2.0 - 1.0, f - vec2(1.0, 1.0));

    return mix(mix(a, b, u.x), mix(c, d, u.x), u.y) * 0.5 + 0.5;
}

// Octave count per call: the shape and its sun-ward re-sample keep all five,
// because the self-shadow is their difference and the two have to match.
// The warp, the erosion and the cirrus are modulators on top of it, and
// their fourth and fifth octaves were below a pixel from any distance the
// sky is seen at. Seven evaluations a pixel became about five.
float fbm(vec2 p, int octaves) {
    float val = 0.0;
    float amp = 0.5;
    // Rotate between octaves so ridge artifacts don't align to the axes
    const mat2 rot = mat2(0.8, 0.6, -0.6, 0.8);
    for (int i = 0; i < octaves; i++) {
        val += amp * gradientNoise(p);
        p = rot * p * 2.02;
        amp *= 0.5;
    }
    return val;
}

void main() {
    vec3 dir = normalize(vWorldDir);
    float altitude = dir.z;
    if (altitude < 0.0) discard;

    vec3 sunDir = normalize(push.sunDirDensity.xyz);
    float density = push.sunDirDensity.w;
    float windOffset = push.windAndLight.x;
    float sunIntensity = push.windAndLight.y;
    float ambient = push.windAndLight.z;

    // Project the view ray onto a flat cloud layer. The +0.08 bias tempers the
    // extreme UV stretching right at the horizon.
    vec2 uv = dir.xy / (altitude + 0.08);
    vec2 wind = vec2(windOffset, windOffset * 0.6);

    // --- Cumulus layer: domain-warped FBM for billowy, irregular shapes ---
    vec2 p = uv * 0.8 + wind;
    vec2 q = vec2(fbm(p, 4), fbm(p + vec2(5.2, 1.3), 4));
    float shape = fbm(p + q * 1.4, 5);

    // Coverage: density opens the threshold; erosion breaks up the edges
    float coverage = smoothstep(0.42 - density * 0.22, 0.74 - density * 0.10, shape);
    float erosion = fbm(uv * 3.1 + wind * 1.6 + q, 3);
    float cumulus = coverage * smoothstep(0.22, 0.55, erosion + coverage * 0.4);

    // --- Cirrus layer: thin, stretched, faster-drifting streaks ---
    vec2 cuv = vec2(uv.x * 0.32, uv.y * 1.5) + wind * 1.8 + vec2(3.7, 9.1);
    float cirrus = fbm(cuv, 3) * fbm(cuv * 2.3 + 4.0, 3);
    cirrus = smoothstep(0.16, 0.5, cirrus) * 0.30 * (0.35 + 0.65 * density);

    float cloud = clamp(cumulus + cirrus * (1.0 - cumulus), 0.0, 1.0);

    // Overall visibility still follows DBC density (0 = clear sky)
    cloud *= clamp(density * 1.6, 0.0, 1.0);

    // Horizon fade
    cloud *= smoothstep(0.0, 0.15, altitude);
    if (cloud < 0.01) discard;

    // --- Lighting ---
    float sunUp = clamp(sunDir.z, 0.0, 1.0);      // day factor
    float sunView = max(dot(dir, sunDir), 0.0);   // view alignment with the sun

    // Self-shadowing: re-sample the shape a step toward the sun; if the cloud
    // is denser upstream, this point sits in its own shadow.
    float towardSun = fbm(p + q * 1.4 + sunDir.xy * 0.35, 5);
    float shadow = clamp(1.0 - (towardSun - shape) * 2.2, 0.35, 1.0);

    // Thick cores read darker - sunlight doesn't penetrate deep cloud
    float coreDarken = mix(1.0, 0.55, cumulus * cumulus);

    vec3 baseColor = push.cloudColor.rgb;
    vec3 shadowColor = baseColor * ambient * 0.75;
    vec3 litColor = baseColor * (ambient + sunIntensity * 0.85 * sunUp);
    vec3 cloudRgb = mix(shadowColor, litColor, shadow) * coreDarken;

    // Forward scattering: thin cloud near the sun glows warm
    float scatter = pow(sunView, 6.0) * sunIntensity * sunUp;
    cloudRgb += vec3(1.0, 0.92, 0.82) * (1.0 - cumulus) * scatter * 0.9;

    // Silver lining on sunlit cloud edges
    float edge = smoothstep(0.0, 0.35, cloud) * (1.0 - smoothstep(0.35, 0.85, cloud));
    cloudRgb += vec3(1.0, 0.95, 0.88) * edge * scatter * 0.6;

    // --- Edge softness for alpha ---
    float alpha = cloud * smoothstep(0.0, 0.25, cloud);

    if (alpha < 0.01) discard;
    outColor = vec4(cloudRgb, alpha);
}
