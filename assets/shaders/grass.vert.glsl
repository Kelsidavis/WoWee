#version 450

// Phase 1 grass: one flat quad per surviving blade, no curvature and no wind.
// The shape is deliberately crude - this phase exists to prove the GPU-driven
// path (compute cull -> compaction -> indirect draw) end to end, and a wrong
// blade shape is visible where a wrong barrier is not.

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
};

// Matches GrassBladeGPU in include/rendering/grass_renderer.hpp (std430).
struct GrassBlade {
    vec4 positionHeight;
    vec4 facingWidthPhase;
};

layout(std430, set = 1, binding = 0) readonly buffer GrassSource {
    GrassBlade blades[];
};

// Written by grass_cull.comp.glsl. Indexed by gl_InstanceIndex, which the
// indirect draw bounds to the count that shader produced.
layout(std430, set = 1, binding = 1) readonly buffer VisibleIndices {
    uint visibleIndices[];
};

// The same origin grass_cull.comp.glsl adds, which the cull carries in its own
// uniform block. This stage does not see that block, so it is pushed instead.
layout(push_constant) uniform Push {
    vec4 fieldOrigin;
} push;

layout(location = 0) out float vHeightT;

void main() {
    GrassBlade blade = blades[visibleIndices[gl_InstanceIndex]];

    vec3  root   = blade.positionHeight.xyz + push.fieldOrigin.xyz;
    float height = blade.positionHeight.w;
    float facing = blade.facingWidthPhase.x;
    float width  = blade.facingWidthPhase.y;

    // Four corners from the vertex index; the index buffer makes two triangles
    // of them. x = across the blade, y = up it.
    vec2 corner = vec2((gl_VertexIndex == 1 || gl_VertexIndex == 2) ? 0.5 : -0.5,
                       (gl_VertexIndex >= 2) ? 1.0 : 0.0);

    // Z is up in render space (renderX = wowY, renderY = wowX, renderZ = wowZ),
    // so the blade rises along Z and widens in the XY plane.
    vec3 across = vec3(cos(facing), sin(facing), 0.0);
    vec3 world  = root + across * (corner.x * width) + vec3(0.0, 0.0, corner.y * height);

    vHeightT = corner.y;
    gl_Position = projection * view * vec4(world, 1.0);
}
