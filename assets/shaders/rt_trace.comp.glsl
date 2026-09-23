#version 460
#ifdef RT_HARDWARE
#extension GL_EXT_ray_query : require
#endif
// One sample per trace pixel of each enabled term:
//   sun visibility - a ray toward a point on the sun's disc
//   ambient occlusion - a short cosine-weighted ray
//   one-bounce diffuse - a cosine-weighted ray, the hit lit by the sun (with
//     its own shadow ray) and the sky; a miss sees the sky
// Noisy by design; rt_temporal and rt_filter turn it into something usable.

#include "rt_common.glsli"
#ifdef RT_HARDWARE
#include "rt_trace_hw.glsli"
#else
#include "rt_trace_sw.glsli"
#endif

layout(set = 0, binding = 3) uniform sampler2D uGbufCur;
layout(set = 0, binding = 4, rgba16f) uniform writeonly image2D uRaw;
layout(set = 0, binding = 5, rgba16f) uniform writeonly image2D uRawGi;

layout(local_size_x = 8, local_size_y = 8) in;

void main() {
    ivec2 px = ivec2(gl_GlobalInvocationID.xy);
    if (any(greaterThanEqual(px, ivec2(pass.extent.xy)))) return;

    vec4 g = texelFetch(uGbufCur, px, 0);
    float dist = rtGbufDist(g);
    if (dist <= 0.0) {
        imageStore(uRaw, px, vec4(1.0, 1.0, 0.0, 0.0));
        imageStore(uRawGi, px, vec4(pass.skyColor.rgb, 0.0));
        return;
    }
    vec3 n = rtGbufNormal(g);
    vec3 p = rtGbufWorld(g, px);
    vec3 origin = p + n * rtSurfaceOffset(dist);
    uint rng = rtSeed(uvec2(px), uint(pass.cameraPos.w));
    uint mode = uint(pass.params.x + 0.5);

    float sun = 0.0;
    vec3 L = normalize(pass.sunDir.xyz);
    if (pass.sunColor.w > 0.5 && dot(n, L) > 0.0) {
        vec3 sd = rtConeSample(L, pass.sunDir.w, rng);
        sun = rtOccluded(origin, sd, pass.params.w, rng) ? 0.0 : 1.0;
    }

    float ao = 1.0;
    if (mode >= RT_MODE_AO) {
        vec3 ad = rtCosineHemisphere(n, rng);
        float t;
        vec3 hn;
        vec4 hs;
        if (rtClosest(origin, ad, pass.params.y, rng, t, hn, hs)) ao = t / pass.params.y;
    }

    vec3 gi = vec3(0.0);
    if (mode >= RT_MODE_GI) {
        vec3 gd = rtCosineHemisphere(n, rng);
        float t;
        vec3 hn;
        vec4 hs;
        if (rtClosest(origin, gd, pass.params.z, rng, t, hn, hs)) {
            vec3 hp = origin + gd * t;
            vec3 radiance = pass.skyColor.rgb;  // the sky the hit surface sees, unshadowed
            float ndl = dot(hn, L);
            if (pass.sunColor.w > 0.5 && ndl > 0.0 &&
                !rtOccluded(hp + hn * rtSurfaceOffset(t), L, pass.params.w, rng)) {
                radiance += pass.sunColor.rgb * ndl;
            }
            gi = hs.rgb * radiance;
        } else {
            gi = pass.skyColor.rgb;
        }
    }

    imageStore(uRaw, px, vec4(sun, ao, 0.0, 0.0));
    imageStore(uRawGi, px, vec4(gi, 0.0));
}
