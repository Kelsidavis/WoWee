#version 450
// Accumulate this frame's samples into last frame's, reprojected.
//
// A pixel's history is found by projecting its world position with last
// frame's matrices and is kept only when last frame saw the same surface
// there - same distance within a few percent, similar normal. The blend
// weight is 1 / history length, capped, so a newly revealed pixel converges
// in a handful of frames and a settled one averages dozens.

#include "rt_common.glsli"

layout(set = 0, binding = 3) uniform sampler2D uGbufCur;
layout(set = 0, binding = 6) uniform sampler2D uGbufPrev;
layout(set = 0, binding = 7) uniform sampler2D uRaw;
layout(set = 0, binding = 8) uniform sampler2D uRawGi;
layout(set = 0, binding = 9) uniform sampler2D uHistPrev;
layout(set = 0, binding = 10) uniform sampler2D uHistGiPrev;
layout(set = 0, binding = 11, rgba16f) uniform writeonly image2D uHist;
layout(set = 0, binding = 12, rgba16f) uniform writeonly image2D uHistGi;

layout(local_size_x = 8, local_size_y = 8) in;

const float MAX_HISTORY = 24.0;

void main() {
    ivec2 px = ivec2(gl_GlobalInvocationID.xy);
    if (any(greaterThanEqual(px, ivec2(pass.extent.xy)))) return;

    vec4 g = texelFetch(uGbufCur, px, 0);
    vec4 raw = texelFetch(uRaw, px, 0);
    vec4 rawGi = texelFetch(uRawGi, px, 0);
    float dist = rtGbufDist(g);
    vec3 n = rtGbufNormal(g);
    if (dist <= 0.0) {
        imageStore(uHist, px, vec4(raw.rg, 1.0, 0.0));
        imageStore(uHistGi, px, rawGi);
        return;
    }

    vec3 p = rtGbufWorld(g, px);

    float len = 0.0;
    vec4 hist = vec4(0.0);
    vec4 histGi = vec4(0.0);
    vec4 clip = pass.prevViewProj * vec4(p, 1.0);
    if (pass.misc.y > 0.5 && clip.w > 0.0) {
        vec2 puv = clip.xy / clip.w * 0.5 + 0.5;
        if (all(greaterThanEqual(puv, vec2(0.0))) && all(lessThan(puv, vec2(1.0)))) {
            // Bilinear by hand, dropping the taps that were another surface.
            vec2 f = puv * pass.extent.xy - 0.5;
            ivec2 base = ivec2(floor(f));
            vec2 w = fract(f);
            float expected = length(p - pass.prevCameraPos.xyz);
            float wsum = 0.0;
            for (int i = 0; i < 4; ++i) {
                ivec2 o = ivec2(i & 1, i >> 1);
                ivec2 q = clamp(base + o, ivec2(0), ivec2(pass.extent.xy) - 1);
                vec4 pg = texelFetch(uGbufPrev, q, 0);
                float pd = rtGbufDist(pg);
                bool same = pd > 0.0 && abs(pd - expected) < 0.05 * expected + 0.1 &&
                            dot(rtGbufNormal(pg), n) > 0.8;
                float bw = (o.x == 1 ? w.x : 1.0 - w.x) * (o.y == 1 ? w.y : 1.0 - w.y);
                if (same && bw > 0.0) {
                    hist += texelFetch(uHistPrev, q, 0) * bw;
                    histGi += texelFetch(uHistGiPrev, q, 0) * bw;
                    wsum += bw;
                }
            }
            if (wsum > 0.05) {
                hist /= wsum;
                histGi /= wsum;
                len = hist.b;
            }
        }
    }

    len = min(len + 1.0, MAX_HISTORY);
    float a = 1.0 / len;
    vec2 lightOut = mix(hist.rg, raw.rg, a);
    vec3 giOut = mix(histGi.rgb, rawGi.rgb, a);
    imageStore(uHist, px, vec4(lightOut, len, 0.0));
    imageStore(uHistGi, px, vec4(giOut, 0.0));
}
