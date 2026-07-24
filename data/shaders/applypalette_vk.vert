#version 450

// Fullscreen triangle, no vertex buffer required: the 3 vertex positions/texture coordinates
// are derived purely from gl_VertexIndex, a common Vulkan idiom for a full-screen pass.
layout(location = 0) out vec2 fTextureCoordinate;

void main()
{
    vec2 uv = vec2((gl_VertexIndex << 1) & 2, gl_VertexIndex & 2);
    fTextureCoordinate = uv;
    gl_Position = vec4(uv * 2.0 - 1.0, 0.0, 1.0);
}
