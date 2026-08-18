#version 450

// GPU culling and compaction for grass blades.
//
// One thread per blade. Survivors claim a slot with a single atomicAdd on the
// draw command's instanceCount and write their own index there, so the draw
// that follows reads a packed list and the count it needs is already the
// field the indirect draw reads. Nothing is read back on the CPU.
//
// Modelled on m2_cull.comp.glsl, which writes a visibility mask for the CPU
// to compact instead. This is the first path in WoWee where the GPU decides
// how many instances to draw.

layout(local_size_x = 64) in;

// Matches GrassBladeGPU in include/rendering/grass_blade.hpp (std430). All
// four vec4s must be declared even though the cull only reads the first two:
// this struct sets the array stride the shader indexes the shared buffer
// with, and a two-vec4 version read every blade at half its real offset -
// testing one blade's position and drawing another's.
struct GrassBlade {
    vec4 positionHeight;    // xyz = root world position, w = height (yards)
    vec4 facingWidthPhase;  // x = facing (radians), y = width, z = profile index, w = phase seed
    vec4 groundShadow;      // unread here; part of the stride
    vec4 groundHighlight;   // unread here; part of the stride
};

layout(std140, set = 0, binding = 0) uniform GrassCullUniforms {
    vec4 frustumPlanes[6];  // xyz = normal, w = distance
    vec4 rangeCenter;       // xyz = player position, w = maxDistSq
    uint bladeCount;
    uint debugFlags;   // bit 1 = skip frustum test, bit 2 = skip distance test
    uint _pad1;
    uint _pad2;
};

layout(std430, set = 0, binding = 1) readonly buffer GrassSource {
    GrassBlade blades[];
};

layout(std430, set = 0, binding = 2) writeonly buffer VisibleIndices {
    uint visibleIndices[];
};

// The draw command itself. instanceCount is the compaction counter: the host
// zeroes that one field before the dispatch and never reads it back.
layout(std430, set = 0, binding = 3) buffer IndirectDraw {
    uint indexCount;
    uint instanceCount;
    uint firstIndex;
    int  vertexOffset;
    uint firstInstance;
};

void main() {
    uint id = gl_GlobalInvocationID.x;
    if (id >= bladeCount) return;

    GrassBlade blade = blades[id];
    vec3 root = blade.positionHeight.xyz;
    float height = blade.positionHeight.w;

    // Distance is measured from the player, not the camera. The population
    // window is generated around the player, and this game's camera orbits:
    // measured from the camera, a swing of the view put the far edge of the
    // draw range outside the generated window, and which side of the field
    // was missing followed the camera around.
    vec3 toCenter = root - rangeCenter.xyz;
    float distSq = dot(toCenter, toCenter);
    if ((debugFlags & 2u) == 0u && distSq > rangeCenter.w) return;

    // Frustum cull. The bounding sphere is centred at half height and sized to
    // cover the blade at full bend, so a blade leaning out of its upright
    // bounds under wind is still tested against the volume it can reach.
    // Z is up: WoWee is Z-up in both canonical and render space.
    float radius = max(height, blade.facingWidthPhase.y) * 0.75;
    vec3 center = root + vec3(0.0, 0.0, height * 0.5);
    if ((debugFlags & 1u) == 0u) {
        for (int i = 0; i < 6; i++) {
            float d = dot(frustumPlanes[i].xyz, center) + frustumPlanes[i].w;
            if (d < -radius) return;
        }
    }

    // Survived: claim a slot and record which blade owns it.
    uint slot = atomicAdd(instanceCount, 1u);
    visibleIndices[slot] = id;
}
