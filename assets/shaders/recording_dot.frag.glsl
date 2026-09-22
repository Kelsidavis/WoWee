#version 450

// The dot that says the screen is being recorded, drawn after the frame has
// been copied for the recording so it is on the screen and not in the video.
// See screen_capture.hpp.

layout(push_constant) uniform Push {
    vec4 dot;  // xy = centre in pixels, z = radius in pixels, w = opacity
} push;

layout(location = 0) out vec4 outColor;

void main() {
    float d = length(gl_FragCoord.xy - push.dot.xy);
    float r = push.dot.z;
    float disc = 1.0 - smoothstep(r - 1.0, r + 1.0, d);
    // A dark rim, so it reads against a bright sky as well as a dark cave.
    float rim = (1.0 - smoothstep(r + 1.0, r + 3.0, d)) * 0.55;
    float cover = max(disc, rim);
    vec3 colour = vec3(0.92, 0.1, 0.08) * (disc / max(cover, 1e-4));
    outColor = vec4(colour, cover * push.dot.w);
}
