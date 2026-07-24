#version 450

// Vulkan equivalent of applypalette.frag (the OpenGL renderer's palette shader): the CPU
// only uploads raw 8-bit palette indices, and this shader does the index -> RGBA colour
// lookup on the GPU instead of the CPU doing it once per pixel every frame.
layout(set = 0, binding = 0) uniform usampler2D uTexture;

layout(std140, set = 0, binding = 1) uniform PaletteUBO
{
    vec4 uPalette[256];
};

layout(location = 0) in vec2 fTextureCoordinate;
layout(location = 0) out vec4 oColour;

void main()
{
    uint index = texture(uTexture, fTextureCoordinate).r;
    oColour = uPalette[index];
}
