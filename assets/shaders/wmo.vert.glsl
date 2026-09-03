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
    /// Cloth hung inside this building's own mesh: (top z, drop, centre x, y)
    /// in the model's local space, and zero drop for everything else.
    ///
    /// A banner painted into a wall is not a doodad and has no model of its
    /// own to sway - Stormwind's gate banners are geometry in the city's WMO,
    /// with STORMWINDBANNER01 for a texture - so the batch that draws one is
    /// told where its bar is and moves the cloth below it here.
    vec4 cloth;
} push;

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aTexCoord;
layout(location = 3) in vec4 aColor;
layout(location = 4) in vec4 aTangent;

layout(location = 0) out vec3 FragPos;
layout(location = 1) out vec3 Normal;
layout(location = 2) out vec2 TexCoord;
layout(location = 3) out vec4 VertColor;
layout(location = 4) out vec3 Tangent;
layout(location = 5) out vec3 Bitangent;

void main() {
    vec3 pos = aPos;

    // The same sway a banner doodad gets, in the building's own coordinates:
    // held at the top edge, free at the hem, a slow swing with a smaller
    // ripple across it. The phase comes from where the cloth hangs, so two
    // banners on one gate are not in step.
    if (push.cloth.y > 0.0) {
        float span = max(push.cloth.y, 0.01);
        float drop = clamp((push.cloth.x - pos.z) / span, 0.0, 1.0);
        drop *= drop;
        float amp = span * 0.05 * drop;
        float phase = fogParams.z * 1.1 + dot(push.cloth.zw, vec2(0.21, 0.17));
        vec3 sway = vec3(
            (sin(phase) * 0.7 + sin(phase * 2.7 + pos.y * 1.9) * 0.25) * amp,
            (cos(phase * 0.9) * 0.6 + cos(phase * 3.1 + pos.x * 1.7) * 0.2) * amp,
            sin(phase * 1.6 + pos.x * 1.3) * 0.12 * amp);

        // Away from the wall, never into it.
        //
        // A banner hangs flush against stone, so any motion toward its own
        // back face goes through the masonry - a quarter of it was still
        // enough to show. The whole perpendicular component is taken out and a
        // third of it given back outward only: the cloth billows away from the
        // wall and slides in its own plane, and the half-cycle that used to
        // push it backwards now does nothing at all.
        //
        // Outward is each face's own normal, so a two-sided banner puffs
        // slightly rather than parting - which is what a cloth in a draught
        // does anyway.
        vec3 clothN = normalize(aNormal);
        float perp = dot(sway, clothN);
        sway -= clothN * perp;
        sway += clothN * max(perp, 0.0) * 0.35;
        pos += sway;
    }

    vec4 worldPos = push.model * vec4(pos, 1.0);
    FragPos = worldPos.xyz;

    mat3 normalMatrix = mat3(push.model);
    Normal = normalMatrix * aNormal;
    TexCoord = aTexCoord;
    VertColor = aColor;

    // Compute TBN basis vectors for normal mapping
    vec3 T = normalize(normalMatrix * aTangent.xyz);
    vec3 N = normalize(Normal);
    // Gram-Schmidt re-orthogonalize
    T = normalize(T - dot(T, N) * N);
    vec3 B = cross(N, T) * aTangent.w;

    Tangent = T;
    Bitangent = B;

    gl_Position = projection * view * worldPos;
}
