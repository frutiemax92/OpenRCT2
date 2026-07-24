#version 450

// Vulkan port of OpenGL's drawrect.frag. Writes an 8-bit palette index (not final RGBA) to
// an R8_UINT colour attachment; final RGBA conversion happens once at the end of the frame via
// the existing applypalette_vk pass. Depth-peeling based transparency is not implemented yet
// (Phase 2) - transparent draws are currently composited the same as opaque ones.

const int kFlagNoTexture = (1 << 2);
const int kFlagMask = (1 << 3);
const int kFlagCrossHatch = (1 << 4);
const int kFlagTTFText = (1 << 5);
const int kMaskRemapCount = 3;

layout(set = 0, binding = 0) uniform usampler2DArray uTexture;
layout(set = 0, binding = 1) uniform usampler2D uPaletteTex;

layout(location = 0) flat in vec2 fPosition;
layout(location = 1) flat in int fFlags;
layout(location = 2) flat in uint fColour;
layout(location = 3) flat in vec4 fTexColour;
layout(location = 4) flat in vec4 fTexMask;
layout(location = 5) flat in vec3 fPalettes;
layout(location = 6) flat in float fZoom;
layout(location = 7) flat in int fTexColourAtlas;
layout(location = 8) flat in int fTexMaskAtlas;
layout(location = 9) flat in int fScreenHeight;

layout(location = 0) out uint oColour;

void main()
{
    vec2 fragCoord = vec2(floor(gl_FragCoord.x), floor(gl_FragCoord.y));
    vec2 position = (fragCoord - fPosition) * fZoom;

    uint texel;
    if ((fFlags & kFlagNoTexture) == 0)
    {
        float colourU = (fTexColour.x + position.x) / fTexColour.z;
        float colourV = (fTexColour.y + position.y) / fTexColour.w;
        texel = texture(uTexture, vec3(colourU, colourV, float(fTexColourAtlas))).r;

        if (texel == 0u)
            discard;

        if ((fFlags & kFlagTTFText) == 0)
        {
            texel += fColour;
        }
        else
        {
            uint hintThreshold = uint(fFlags & 0xff00) >> 8;
            if (hintThreshold > 0u)
            {
                bool solidColour = texel > 180u;
                texel = (texel > hintThreshold) ? fColour : 0u;
                texel = texel << 8;
                if (solidColour)
                    texel += 1u;
            }
            else
            {
                texel = fColour;
            }
        }
    }
    else
    {
        texel = fColour;
    }

    int paletteCount = fFlags & kMaskRemapCount;
    if (paletteCount >= 3 && texel >= 0x2Eu && texel < 0x3Au)
    {
        texel = texture(uPaletteTex, vec2(float(texel) + 0xC5, fPalettes.z) / 256.0).r;
    }
    else if (paletteCount >= 2 && texel >= 0xCAu && texel < 0xD6u)
    {
        texel = texture(uPaletteTex, vec2(float(texel) + 0x29, fPalettes.y) / 256.0).r;
    }
    else if (paletteCount >= 1)
    {
        texel = texture(uPaletteTex, vec2(float(texel), fPalettes.x) / 256.0).r;
    }

    if (texel == 0u)
        discard;

    if ((fFlags & kFlagCrossHatch) != 0)
    {
        int posSum = int(position.x) + int(position.y);
        if ((posSum & 1) != 0)
            discard;
    }

    if ((fFlags & kFlagMask) != 0)
    {
        float maskU = (fTexMask.x + position.x) / fTexMask.z;
        float maskV = (fTexMask.y + position.y) / fTexMask.w;
        uint mask = texture(uTexture, vec3(maskU, maskV, float(fTexMaskAtlas))).r;
        if (mask == 0u)
            discard;
    }

    oColour = texel;
}
