#version 450

// Vulkan port of OpenGL's drawrect.vert. Handles FillRect/FilterRect/DrawSprite/
// DrawSpriteRawMasked/DrawSpriteSolid/DrawGlyph/DrawTTFBitmap - all issued as instanced quads
// (4 vertices via gl_VertexIndex, no separate vertex buffer needed) with one DrawRectCommand
// per instance. Depth is derived from a global increasing draw-order counter so a plain
// depth test (LESS) reproduces the correct painter's-algorithm ordering even though all
// commands of this type are submitted in a single instanced draw call.

layout(push_constant) uniform PushConstants
{
    vec2 uScreenSize;
}
pc;

layout(location = 0) in ivec4 vClip;
layout(location = 1) in int vTexColourAtlas;
layout(location = 2) in vec4 vTexColourBounds;
layout(location = 3) in int vTexMaskAtlas;
layout(location = 4) in vec4 vTexMaskBounds;
layout(location = 5) in ivec3 vPalettes;
layout(location = 6) in int vFlags;
layout(location = 7) in uint vColour;
layout(location = 8) in ivec4 vBounds;
layout(location = 9) in int vDepth;
layout(location = 10) in float vZoom;

layout(location = 0) flat out vec2 fPosition;
layout(location = 1) flat out int fFlags;
layout(location = 2) flat out uint fColour;
layout(location = 3) flat out vec4 fTexColour;
layout(location = 4) flat out vec4 fTexMask;
layout(location = 5) flat out vec3 fPalettes;
layout(location = 6) flat out float fZoom;
layout(location = 7) flat out int fTexColourAtlas;
layout(location = 8) flat out int fTexMaskAtlas;
layout(location = 9) flat out int fScreenHeight;

const float kDepthIncrement = 1.0 / float(1u << 22u);

// Equivalent to OpenGL's constexpr kVertexData (4 vertices, each a mat4x2 + vec2) - hard-coded
// here since gl_VertexIndex replaces the separate non-instanced vertex buffer OpenGL uses.
const mat4x2 kVertMat[4] = mat4x2[4](
    mat4x2(1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0), mat4x2(0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0),
    mat4x2(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0), mat4x2(0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0));
const vec2 kVertVec[4] = vec2[4](vec2(0.0, 0.0), vec2(1.0, 0.0), vec2(0.0, 1.0), vec2(1.0, 1.0));

void main()
{
    mat4x2 vertMat = kVertMat[gl_VertexIndex];
    vec2 vertVec = kVertVec[gl_VertexIndex];

    vec2 m = clamp(
        ((vertMat * vec4(vClip)) - (vertMat * vec4(vBounds))) / vec2(vBounds.zw - vBounds.xy) + vertVec, 0.0, 1.0);
    vec2 pos = mix(vec2(vBounds.xy), vec2(vBounds.zw), m);

    fTexColour = vTexColourBounds;
    fTexMask = vTexMaskBounds;
    fPosition = vec2(vBounds.xy);
    fZoom = vZoom;
    fTexColourAtlas = vTexColourAtlas;
    fTexMaskAtlas = vTexMaskAtlas;

    float depth = 1.0 - (float(vDepth) + 1.0) * kDepthIncrement;
    pos = pos / pc.uScreenSize;

    fFlags = vFlags;
    fColour = vColour;
    fPalettes = vec3(vPalettes);
    fScreenHeight = int(pc.uScreenSize.y);

    pos = pos * 2.0 - 1.0;
    gl_Position = vec4(pos, depth, 1.0);
}
