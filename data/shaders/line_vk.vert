#version 450

// Vulkan port of OpenGL's drawline.vert. Draws instanced 2-vertex lines (one instance per
// DrawLineCommand), sharing the same global depth-order counter as rect_vk so lines interleave
// correctly with rects/sprites/text despite being a separate draw call.

layout(push_constant) uniform PushConstants
{
    vec2 uScreenSize;
}
pc;

layout(location = 0) in ivec4 vBounds;
layout(location = 1) in uint vColour;
layout(location = 2) in int vDepth;

layout(location = 0) flat out uint fColour;

const float kDepthIncrement = 1.0 / float(1u << 22u);

void main()
{
    vec2 pos = (gl_VertexIndex == 0) ? vec2(vBounds.xy) : vec2(vBounds.zw);

    pos = (pos * (2.0 / pc.uScreenSize)) - 1.0;

    float depth = 1.0 - (float(vDepth) + 1.0) * kDepthIncrement;

    fColour = vColour;

    gl_Position = vec4(pos, depth, 1.0);
}
