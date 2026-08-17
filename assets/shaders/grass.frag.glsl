#version 450

// Phase 1 grass: a root-to-tip gradient and nothing else. Real shading -
// terrain colour influence, upright-normal blend, two-sided lighting,
// subsurface - is Phase 6. The gradient is here only so the blades read as
// blades in a screenshot rather than as a flat green field, which makes an
// upside-down or zero-height blade obvious at a glance.

layout(location = 0) in float vHeightT;

layout(location = 0) out vec4 outColor;

void main() {
    const vec3 rootColor = vec3(0.10, 0.22, 0.06);
    const vec3 tipColor  = vec3(0.35, 0.55, 0.18);
    outColor = vec4(mix(rootColor, tipColor, vHeightT), 1.0);
}
