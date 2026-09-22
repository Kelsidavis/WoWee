#version 450

// Crepuscular rays, one pixel of the quarter-size frame at a time. See
// sun_shafts.hpp.
//
// Each pixel walks toward the sun's place on screen and gathers the bright sky
// it passes, the nearer samples counting for more. A pixel whose walk crosses a
// gap in the leaves lights up; one whose walk runs into a trunk does not - so
// the gaps pour streaks out across the picture and the branches cast dark ones.
// What counts as a source is bright and near the sun: the sky and clouds
// around it, not a white wall on the other side of the screen.

layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;

layout(set = 0, binding = 0) uniform sampler2D uFrame;
layout(set = 0, binding = 1, rgba16f) uniform writeonly image2D uRays;

layout(push_constant) uniform Push {
    vec4 sun;   // xy = the sun's place on screen, z = width / height, w = strength
    vec4 tint;  // rgb = the sun's colour
} push;

const int kSamples = 48;
// How much each step toward the sun counts against the one before it, and how
// much of the way there a pixel looks. Together they set how far a streak runs:
// at this the samples by the sun still count for two fifths of the ones by the
// pixel, so a gap a quarter of the screen from the sun still casts one.
const float kDecay = 0.98;
const float kReach = 0.8;
// Brightness at which the frame starts to count as sky near the sun, and at
// which it counts fully. In the swapchain's own values.
const float kThresholdLow = 0.55;
const float kThresholdHigh = 0.95;
// How far from the sun, in screen heights, the sources fade out.
const float kSourceRadius = 0.4;

void main() {
    ivec2 pixel = ivec2(gl_GlobalInvocationID.xy);
    ivec2 size = imageSize(uRays);
    if (any(greaterThanEqual(pixel, size))) return;

    vec2 uv = (vec2(pixel) + 0.5) / vec2(size);
    vec2 step = (push.sun.xy - uv) * (kReach / float(kSamples));

    // Where along its first step each pixel starts, fixed per pixel. Without
    // it the samples fall at the same distances everywhere and the rays show
    // as rings around the sun.
    float dither = fract(52.9829189 * fract(dot(vec2(pixel), vec2(0.06711056, 0.00583715))));
    vec2 at = uv + step * dither;

    vec3 gathered = vec3(0.0);
    float weight = 1.0;
    float total = 0.0;
    for (int i = 0; i < kSamples; ++i) {
        total += weight;
        if (at.x >= 0.0 && at.x <= 1.0 && at.y >= 0.0 && at.y <= 1.0) {
            vec3 c = textureLod(uFrame, at, 0.0).rgb;
            float bright = smoothstep(kThresholdLow, kThresholdHigh,
                                      dot(c, vec3(0.2126, 0.7152, 0.0722)));
            vec2 fromSun = (at - push.sun.xy) * vec2(push.sun.z, 1.0);
            float nearSun = exp(-dot(fromSun, fromSun) / (kSourceRadius * kSourceRadius));
            gathered += c * (bright * nearSun * weight);
        }
        weight *= kDecay;
        at += step;
    }

    // As a share of a walk that was bright all the way: 1 is the sky's own
    // colour, before the strength scales it.
    vec3 rays = gathered * (push.sun.w / total) * push.tint.rgb;
    imageStore(uRays, pixel, vec4(rays, 1.0));
}
