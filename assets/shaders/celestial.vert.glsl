#version 450

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

layout(push_constant) uniform Push {
    mat4 model;
    vec4 celestialColor; // xyz = color, w = unused
    float intensity;
    float moonPhase;
    float animTime;
} push;

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec2 aTexCoord;

layout(location = 0) out vec2 TexCoord;

void main() {
    TexCoord = aTexCoord;
    // Sky object: remove camera translation so celestial bodies are at infinite distance
    mat4 rotView = mat4(mat3(view));
    vec4 clip = projection * rotView * push.model * vec4(aPos, 1.0);
    // On the far plane, as the clouds are, so the depth test the sky is now
    // drawn under rejects it wherever the world has already been drawn.
    gl_Position = vec4(clip.xy, clip.w, clip.w);
}
