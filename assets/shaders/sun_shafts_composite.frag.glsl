#version 450

// The sun shafts, screened over the finished frame ahead of the interface -
// the blend does the screening. The quarter-size rays are filtered up on the
// way; there is nothing finer in them to lose. See sun_shafts.hpp.

layout(set = 0, binding = 0) uniform sampler2D uRays;

layout(location = 0) in vec2 TexCoord;

layout(location = 0) out vec4 outColor;

void main() {
    outColor = vec4(texture(uRays, TexCoord).rgb, 0.0);
}
