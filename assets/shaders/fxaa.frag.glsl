#version 450

// FXAA 3.11, quality preset 12 (Timothy Lottes). Reads the resolved scene
// colour and writes the smoothed result.
// Push constants: rcpFrame, sharpness, intoxication (0 = sober, 1 = smashed).
//
// A port of the reference rather than a paraphrase of it. The pass this
// replaces had the shape of FXAA and two of its numbers wrong. Its
// end-of-edge search compared each sample against lumaM minus the average of
// the two neighbours - a signed contrast, close to zero - where the reference
// subtracts the luma midway between the pixel and the neighbour across the
// edge, so the search ended on its first step nearly everywhere. The offset
// it then applied was dst/span where the reference has 0.5 - dst/span, so the
// middle of every edge got the full half-pixel blend and the stair-steps at
// the ends got none. Together that drew a half-pixel smear along every edge
// and removed no aliasing, which is what "FXAA makes everything blurry and
// does not help" looks like from the chair.

layout(set = 0, binding = 0) uniform sampler2D uScene;

layout(location = 0) in vec2 TexCoord;
layout(location = 0) out vec4 outColor;

layout(push_constant) uniform PC {
    vec2  rcpFrame;
    float sharpness;    // 0 = no sharpen, 2 = max (matches FSR2 RCAS range)
    float intoxication;
} pc;

// The reference's default tuning. Subpix 0.75 is what it calls the default
// amount of filtering; 0.50 is the value it names as sharper, if this still
// reads soft on fine texture.
#define FXAA_EDGE_THRESHOLD      0.166   // local contrast below this is left alone
#define FXAA_EDGE_THRESHOLD_MIN  0.0833  // and darks below this, whatever their contrast
#define FXAA_SUBPIX              0.75    // sub-pixel aliasing removal, 0 = off
#define FXAA_SEARCH_STEPS        12

// Quality preset 12: fine steps next to the pixel, coarse ones far out along
// the edge. Eleven samples a side at most; the last step only positions.
const float kSearchStep[FXAA_SEARCH_STEPS] =
    float[](1.0, 1.5, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 4.0, 8.0);

float luma(vec3 c) {
    return dot(c, vec3(0.299, 0.587, 0.114));
}

void main() {
    float drunk = clamp(pc.intoxication, 0.0, 1.0);
    vec2 uv     = TexCoord + vec2(
        sin(TexCoord.y * 34.0) * pc.rcpFrame.x,
        cos(TexCoord.x * 29.0) * pc.rcpFrame.y) * (3.0 * drunk);
    vec2 rcp    = pc.rcpFrame;

    // --- Centre, with the drunk blur folded in before anything reads it ---
    vec3 rgbM = texture(uScene, uv).rgb;
    if (drunk > 0.0) {
        vec2 radius = rcp * mix(1.0, 5.0, drunk);
        vec3 blur = rgbM;
        blur += texture(uScene, uv + vec2( radius.x, 0.0)).rgb;
        blur += texture(uScene, uv + vec2(-radius.x, 0.0)).rgb;
        blur += texture(uScene, uv + vec2(0.0,  radius.y)).rgb;
        blur += texture(uScene, uv + vec2(0.0, -radius.y)).rgb;
        blur += texture(uScene, uv + radius).rgb;
        blur += texture(uScene, uv - radius).rgb;
        blur += texture(uScene, uv + vec2(radius.x, -radius.y)).rgb;
        blur += texture(uScene, uv + vec2(-radius.x, radius.y)).rgb;
        rgbM = mix(rgbM, blur / 9.0, 0.75 * drunk);
    }

    // --- Cardinal neighbours: is there an edge here at all ---
    float lumaM = luma(rgbM);
    float lumaN = luma(texture(uScene, uv + vec2( 0.0, -1.0) * rcp).rgb);
    float lumaS = luma(texture(uScene, uv + vec2( 0.0,  1.0) * rcp).rgb);
    float lumaE = luma(texture(uScene, uv + vec2( 1.0,  0.0) * rcp).rgb);
    float lumaW = luma(texture(uScene, uv + vec2(-1.0,  0.0) * rcp).rgb);

    float rangeMax = max(lumaM, max(max(lumaN, lumaS), max(lumaE, lumaW)));
    float rangeMin = min(lumaM, min(min(lumaN, lumaS), min(lumaE, lumaW)));
    float range    = rangeMax - rangeMin;

    if (range < max(FXAA_EDGE_THRESHOLD_MIN, rangeMax * FXAA_EDGE_THRESHOLD)) {
        outColor = vec4(rgbM, 1.0);
        return;
    }

    float lumaNW = luma(texture(uScene, uv + vec2(-1.0, -1.0) * rcp).rgb);
    float lumaNE = luma(texture(uScene, uv + vec2( 1.0, -1.0) * rcp).rgb);
    float lumaSW = luma(texture(uScene, uv + vec2(-1.0,  1.0) * rcp).rgb);
    float lumaSE = luma(texture(uScene, uv + vec2( 1.0,  1.0) * rcp).rgb);

    float lumaNS   = lumaN + lumaS;
    float lumaWE   = lumaW + lumaE;
    float lumaNWSW = lumaNW + lumaSW;
    float lumaNESE = lumaNE + lumaSE;
    float lumaNWNE = lumaNW + lumaNE;
    float lumaSWSE = lumaSW + lumaSE;

    // --- Sub-pixel aliasing: how far the pixel sits from its 3x3 lowpass ---
    float subpixA = (lumaNS + lumaWE) * 2.0 + (lumaNWSW + lumaNESE);
    float subpixB = subpixA * (1.0 / 12.0) - lumaM;
    float subpixC = clamp(abs(subpixB) / range, 0.0, 1.0);
    float subpixF = (-2.0 * subpixC + 3.0) * subpixC * subpixC;   // smoothstep
    float subpixH = subpixF * subpixF * FXAA_SUBPIX;

    // --- Edge orientation ---
    float edgeHorz = abs(-2.0 * lumaW + lumaNWSW)
                   + abs(-2.0 * lumaM + lumaNS) * 2.0
                   + abs(-2.0 * lumaE + lumaNESE);
    float edgeVert = abs(-2.0 * lumaS + lumaSWSE)
                   + abs(-2.0 * lumaM + lumaWE) * 2.0
                   + abs(-2.0 * lumaN + lumaNWNE);
    bool horzSpan = edgeHorz >= edgeVert;

    // The neighbour across the edge is the one with the larger gradient from
    // the pixel; lengthSign points at it.
    float luma1 = horzSpan ? lumaN : lumaW;
    float luma2 = horzSpan ? lumaS : lumaE;
    float gradient1 = luma1 - lumaM;
    float gradient2 = luma2 - lumaM;
    bool  pair1 = abs(gradient1) >= abs(gradient2);
    float gradientScaled = max(abs(gradient1), abs(gradient2)) * 0.25;
    float lengthSign = horzSpan ? rcp.y : rcp.x;
    if (pair1) lengthSign = -lengthSign;

    // The luma half-way between the pixel and that neighbour. The search walks
    // along the edge, half a pixel over onto the line between the two, and a
    // side is done where its sample leaves this midpoint by more than a
    // quarter of the gradient: the end of the edge on that side.
    float lumaMid     = ((pair1 ? luma1 : luma2) + lumaM) * 0.5;
    bool  lumaMLTZero = (lumaM - lumaMid) < 0.0;

    vec2 posB  = uv;
    vec2 offNP = horzSpan ? vec2(rcp.x, 0.0) : vec2(0.0, rcp.y);
    if ( horzSpan) posB.y += lengthSign * 0.5;
    if (!horzSpan) posB.x += lengthSign * 0.5;

    vec2  posN = posB - offNP * kSearchStep[0];
    vec2  posP = posB + offNP * kSearchStep[0];
    float lumaEndN = luma(texture(uScene, posN).rgb) - lumaMid;
    float lumaEndP = luma(texture(uScene, posP).rgb) - lumaMid;
    bool  doneN = abs(lumaEndN) >= gradientScaled;
    bool  doneP = abs(lumaEndP) >= gradientScaled;
    if (!doneN) posN -= offNP * kSearchStep[1];
    if (!doneP) posP += offNP * kSearchStep[1];
    for (int i = 2; i < FXAA_SEARCH_STEPS && !(doneN && doneP); ++i) {
        if (!doneN) lumaEndN = luma(texture(uScene, posN).rgb) - lumaMid;
        if (!doneP) lumaEndP = luma(texture(uScene, posP).rgb) - lumaMid;
        doneN = doneN || abs(lumaEndN) >= gradientScaled;
        doneP = doneP || abs(lumaEndP) >= gradientScaled;
        if (!doneN) posN -= offNP * kSearchStep[i];
        if (!doneP) posP += offNP * kSearchStep[i];
    }

    // --- Where along the span this pixel is, and how much to blend for it ---
    float dstN = horzSpan ? (uv.x - posN.x) : (uv.y - posN.y);
    float dstP = horzSpan ? (posP.x - uv.x) : (posP.y - uv.y);
    bool  directionN = dstN < dstP;
    float dst        = min(dstN, dstP);
    float spanLength = dstN + dstP;
    bool  goodSpanN  = (lumaEndN < 0.0) != lumaMLTZero;
    bool  goodSpanP  = (lumaEndP < 0.0) != lumaMLTZero;
    bool  goodSpan   = directionN ? goodSpanN : goodSpanP;

    // Half a pixel at the near end of the span, where the stair-step is,
    // falling to nothing at its middle, where the edge is already straight.
    float pixelOffset      = 0.5 - dst / spanLength;
    float pixelOffsetGood  = goodSpan ? pixelOffset : 0.0;
    float pixelOffsetFinal = max(pixelOffsetGood, subpixH);

    vec2 finalUV = uv;
    if ( horzSpan) finalUV.y += pixelOffsetFinal * lengthSign;
    if (!horzSpan) finalUV.x += pixelOffsetFinal * lengthSign;

    vec3 fxaaResult = texture(uScene, finalUV).rgb;
    fxaaResult = mix(fxaaResult, rgbM, 0.75 * drunk);

    // Post-FXAA contrast-adaptive sharpening (unsharp mask), only when FSR2
    // hands over its sharpness: it restores what the upscale softened.
    if (pc.sharpness > 0.0) {
        vec2 r = pc.rcpFrame;
        vec3 blur = (texture(uScene, uv + vec2(-r.x, 0)).rgb
                   + texture(uScene, uv + vec2( r.x, 0)).rgb
                   + texture(uScene, uv + vec2(0, -r.y)).rgb
                   + texture(uScene, uv + vec2(0,  r.y)).rgb) * 0.25;
        // scale sharpness from [0,2] to a modest [0, 0.3] boost factor
        float s = pc.sharpness * 0.15;
        fxaaResult = clamp(fxaaResult + s * (fxaaResult - blur), 0.0, 1.0);
    }

    outColor = vec4(fxaaResult, 1.0);
}
