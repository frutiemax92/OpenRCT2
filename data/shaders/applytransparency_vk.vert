#version 450

// Vulkan port of applytransparency.vert - fullscreen triangle, no vertex buffer required (same
// gl_VertexIndex trick as applypalette_vk.vert). Runs once per depth-peeling iteration,
// compositing the transparent layer just rendered on top of the current opaque buffer.
layout(location = 0) out vec2 fTextureCoordinate;

void main()
{
    vec2 uv = vec2((gl_VertexIndex << 1) & 2, gl_VertexIndex & 2);
    fTextureCoordinate = uv;
    gl_Position = vec4(uv * 2.0 - 1.0, 0.0, 1.0);
}
