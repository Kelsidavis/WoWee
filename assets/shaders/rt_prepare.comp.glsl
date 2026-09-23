#version 450
// Scene depth -> the tracer's G-buffer, at trace resolution: world normal
// reconstructed from depth, and the distance to the camera (0 for sky).
//
// Built with RT_MSAA when the depth is multisampled and could not be
// resolved; sample 0 stands for the pixel then.

#include "rt_common.glsli"

#ifdef RT_MSAA
layout(set = 0, binding = 1) uniform sampler2DMS uSceneDepth;
#else
layout(set = 0, binding = 1) uniform sampler2D uSceneDepth;
#endif
layout(set = 0, binding = 2, rgba16f) uniform writeonly image2D uGbuf;

layout(local_size_x = 8, local_size_y = 8) in;

float loadDepth(ivec2 p) {
    p = clamp(p, ivec2(0), ivec2(pass.sceneExtent.xy) - 1);
#ifdef RT_MSAA
    return texelFetch(uSceneDepth, p, 0).r;
#else
    return texelFetch(uSceneDepth, p, 0).r;
#endif
}

vec3 worldAt(ivec2 p, float depth) {
    return rtWorldFromDepth((vec2(p) + 0.5) * pass.sceneExtent.zw, depth);
}

void main() {
    ivec2 px = ivec2(gl_GlobalInvocationID.xy);
    if (any(greaterThanEqual(px, ivec2(pass.extent.xy)))) return;

    // The nearest surface in this texel's block of scene pixels.
    ivec2 base = rtBlockBase(px);
    int sub = 0;
    float d = 1.0;
    for (int i = 0; i < 4; ++i) {
        float di = loadDepth(base + ivec2(i & 1, i >> 1));
        if (di < d) {
            d = di;
            sub = i;
        }
    }
    if (d >= 1.0) {
        imageStore(uGbuf, px, vec4(0.0, 0.0, 0.0, 0.0));
        return;
    }
    ivec2 sp = base + ivec2(sub & 1, sub >> 1);
    vec3 p = worldAt(sp, d);

    // Pick, on each axis, the neighbour on the same surface - the one whose
    // depth continues the slope best - so edges do not bend the normal.
    float dl = loadDepth(sp + ivec2(-1, 0)), dr = loadDepth(sp + ivec2(1, 0));
    float dd = loadDepth(sp + ivec2(0, -1)), du = loadDepth(sp + ivec2(0, 1));
    vec3 dx = abs(dr - d) < abs(d - dl) ? worldAt(sp + ivec2(1, 0), dr) - p
                                        : p - worldAt(sp + ivec2(-1, 0), dl);
    vec3 dy = abs(du - d) < abs(d - dd) ? worldAt(sp + ivec2(0, 1), du) - p
                                        : p - worldAt(sp + ivec2(0, -1), dd);
    vec3 n = normalize(cross(dx, dy));
    vec3 toCam = pass.cameraPos.xyz - p;
    if (dot(n, toCam) < 0.0) n = -n;

    imageStore(uGbuf, px, vec4(rtOctEncode(n), length(toCam), float(sub)));
}
