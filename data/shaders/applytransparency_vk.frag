#version 450

// Vulkan port of applytransparency.frag - composites the transparent layer just rendered (one
// depth-peeling iteration) on top of the current opaque buffer into the mix buffer. See
// SwapFramebuffer::ApplyTransparency in the OpenGL renderer for the reference implementation;
// this is a direct, unmodified port of that fragment shader's logic.
layout(set = 0, binding = 0) uniform usampler2D uOpaqueTex;
layout(set = 0, binding = 1) uniform sampler2D uOpaqueDepth;
layout(set = 0, binding = 2) uniform usampler2D uTransparentTex;
layout(set = 0, binding = 3) uniform sampler2D uTransparentDepth;
layout(set = 0, binding = 4) uniform usampler2D uPaletteTex;
layout(set = 0, binding = 5) uniform usampler2D uBlendPaletteTex;

layout(location = 0) in vec2 fTextureCoordinate;
layout(location = 0) out uint oColour;

void main()
{
    uint opaque = texture(uOpaqueTex, fTextureCoordinate).r;
    float opaqueDepth = texture(uOpaqueDepth, fTextureCoordinate).r;
    uint transparent = texture(uTransparentTex, fTextureCoordinate).r;
    float transparentDepth = texture(uTransparentDepth, fTextureCoordinate).r;

    if (opaqueDepth <= transparentDepth)
    {
        transparent = 0u;
    }

    uint blendColour = (transparent & 0xff00u) >> 8;
    if (blendColour > 0u)
    {
        if ((transparent & 0x00ffu) != 0u)
        {
            oColour = blendColour;
        }
        else
        {
            oColour = texture(uBlendPaletteTex, vec2(opaque, blendColour) / 256.0).r;
        }
    }
    else
    {
        oColour = texture(uPaletteTex, vec2(opaque, transparent) / 256.0).r;
    }
}
