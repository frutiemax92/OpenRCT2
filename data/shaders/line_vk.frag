#version 450

// Vulkan port of OpenGL's drawline.frag - writes an 8-bit palette index (see rect_vk.frag).

layout(location = 0) flat in uint fColour;
layout(location = 0) out uint oColour;

void main()
{
    oColour = fColour;
}
