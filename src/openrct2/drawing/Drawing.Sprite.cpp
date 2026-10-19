/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Drawing.Sprite.h"

#include "../Context.h"
#include "../Diagnostic.h"
#include "../OpenRCT2.h"
#include "../PlatformEnvironment.h"
#include "../SpriteIds.h"
#include "../config/Config.h"
#include "../core/FileStream.h"
#include "../core/Guard.hpp"
#include "../core/MemoryStream.h"
#include "../core/Path.hpp"
#include "../interface/ZoomLevel.h"
#include "../platform/Platform.h"
#include "../ui/UiContext.h"
#include "Drawing.h"
#include "RenderTarget.h"
#include "ScrollingText.h"

#include <cassert>
#include <memory>
#include <stdexcept>
#include <vector>

using namespace OpenRCT2;
using namespace OpenRCT2::Drawing;
using namespace OpenRCT2::Ui;

static constexpr uint8_t kPaletteLengthRemap = 12;

/**
 * 12 elements from 0xF3 are the peep top colour, 12 elements from 0xCA are peep trouser colour
 *
 * rct2: 0x0009ABE0C
 */
// clang-format off
static thread_local uint8_t kSecondaryRemapPalette[256] = {
    0x00, 0xF3, 0xF4, 0xF5, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
    0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F,
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F,
    0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F,
    0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F,
    0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F,
    0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7E, 0x7F,
    0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F,
    0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0x9B, 0x9C, 0x9D, 0x9E, 0x9F,
    0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF,
    0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF,
    0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF,
    0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE, 0xDF,
    0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xEB, 0xEC, 0xED, 0xEE, 0xEF,
    0xF0, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF,
};

/** rct2: 0x009ABF0C */
static thread_local uint8_t kTertiaryRemapPalette[256] = {
    0x00, 0xF3, 0xF4, 0xF5, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
    0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F,
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F,
    0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F,
    0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F,
    0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F,
    0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7E, 0x7F,
    0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F,
    0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0x9B, 0x9C, 0x9D, 0x9E, 0x9F,
    0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF,
    0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF,
    0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF,
    0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE, 0xDF,
    0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xEB, 0xEC, 0xED, 0xEE, 0xEF,
    0xF0, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF,
};

// clang-format on
static auto GetMaskFunction()
{
    if (Platform::AVX2Available())
    {
        LOG_VERBOSE("registering AVX2 mask function");
        return MaskAvx2;
    }
    else if (Platform::SSE41Available())
    {
        LOG_VERBOSE("registering SSE4.1 mask function");
        return MaskSse4_1;
    }
    else
    {
        LOG_VERBOSE("registering scalar mask function");
        return MaskScalar;
    }
}

static const auto MaskFunc = GetMaskFunction();

void MaskFn(
    int32_t width, int32_t height, const uint8_t* RESTRICT maskSrc, const uint8_t* RESTRICT colourSrc,
    PaletteIndex* RESTRICT dst, int32_t maskWrap, int32_t colourWrap, int32_t dstWrap)
{
    MaskFunc(width, height, maskSrc, colourSrc, dst, maskWrap, colourWrap, dstWrap);
}

static void ReadAndConvertGxDat(IStream* stream, size_t count, G1Element* elements)
{
    auto g1Elements32 = std::make_unique<StoredG1Element[]>(count);
    stream->Read(g1Elements32.get(), count * sizeof(StoredG1Element));
    for (size_t i = 0; i < count; i++)
    {
        const StoredG1Element& src = g1Elements32[i];

        // Double cast to silence compiler warning about casting to
        // pointer from integer of mismatched length.
        elements[i].offset = reinterpret_cast<uint8_t*>(static_cast<uintptr_t>(src.offset));
        elements[i].width = src.width;
        elements[i].height = src.height;
        elements[i].xOffset = src.xOffset;
        elements[i].yOffset = src.yOffset;
        elements[i].flags = src.flags;
        elements[i].zoomedOffset = src.zoomedOffset;
    }
}

void MaskScalar(
    int32_t width, int32_t height, const uint8_t* RESTRICT maskSrc, const uint8_t* RESTRICT colourSrc,
    PaletteIndex* RESTRICT dst, int32_t maskWrap, int32_t colourWrap, int32_t dstWrap)
{
    for (int32_t yy = 0; yy < height; yy++)
    {
        for (int32_t xx = 0; xx < width; xx++)
        {
            auto colour = static_cast<PaletteIndex>((*colourSrc) & (*maskSrc));
            if (colour != PaletteIndex::transparent)
            {
                *dst = colour;
            }

            maskSrc++;
            colourSrc++;
            dst++;
        }
        maskSrc += maskWrap;
        colourSrc += colourWrap;
        dst += dstWrap;
    }
}

static void MaskMagnify(
    const ZoomLevel zoom, int32_t width, int32_t height, const uint8_t* RESTRICT maskSrc, const uint8_t* RESTRICT colourSrc,
    PaletteIndex* RESTRICT dst, int32_t maskStride, int32_t colourStride, int32_t dstStride, int32_t srcX, int32_t srcY)
{
    for (int32_t y = 0; y < height; y++)
    {
        auto nextDst = dst + dstStride;
        for (int32_t x = 0; x < width; x++, dst++)
        {
            auto srcMask = maskSrc + (maskStride * zoom.ApplyTo(srcY + y) + zoom.ApplyTo(srcX + x));
            auto srcColour = colourSrc + (colourStride * zoom.ApplyTo(srcY + y) + zoom.ApplyTo(srcX + x));
            const auto colour = static_cast<PaletteIndex>((*srcColour) & (*srcMask));
            if (colour != PaletteIndex::transparent)
            {
                *dst = colour;
            }
        }
        dst = nextDst;
    }
}

// Base graphics pack: images of the g1 and g2 index ranges (AST-002).
static Gx _base = {};
static Gx _fonts = {};
static Gx _palettes = {};
static G1Element _scrollingText[ScrollingText::kMaxEntries]{};

static G1Element _g1Temp[kTempSpriteCount] = {};
static std::vector<G1Element> _imageListElements;

bool GfxLoadBaseGraphics(const IPlatformEnvironment& env)
{
    LOG_VERBOSE("GfxLoadBaseGraphics(...)");
    auto path = Path::Combine(env.GetDirectoryPath(DirBase::openrct2), kBaseGraphicsFileName);
    try
    {
        auto fs = FileStream(path, FileMode::open);
        _base.header = fs.ReadValue<G1Header>();
        if (_base.header.numEntries > SPR_G2_END)
        {
            throw std::runtime_error("too many entries");
        }

        // Read element headers, then pad to the end of the range: slots the pack does not provide are empty images.
        _base.elements.resize(SPR_G2_END);
        ReadAndConvertGxDat(&fs, _base.header.numEntries, _base.elements.data());
        _base.data = fs.ReadArray<uint8_t>(_base.header.totalSize);

        for (uint32_t i = 0; i < _base.header.numEntries; i++)
        {
            if (_base.elements[i].offset == nullptr)
            {
                _base.elements[i].offset = _base.data.get();
            }
            else
            {
                _base.elements[i].offset += reinterpret_cast<uintptr_t>(_base.data.get());
            }
        }
        for (size_t i = _base.header.numEntries; i < _base.elements.size(); i++)
        {
            _base.elements[i].offset = _base.data.get();
        }
        return true;
    }
    catch (const std::exception& e)
    {
        GfxUnloadBaseGraphics();

        LOG_FATAL("Unable to load base graphics %s: %s", path.c_str(), e.what());
        if (!gOpenRCT2Headless)
        {
            auto& uiContext = GetContext()->GetUiContext();
            uiContext.ShowMessageBox(
                std::string("Unable to load ") + kBaseGraphicsFileName + ". Your installation may be damaged.");
        }
        return false;
    }
}

void GfxUnloadBaseGraphics()
{
    _base.data.reset();
    _base.elements.clear();
    _base.elements.shrink_to_fit();
    _base.header = {};
}

void GfxUnloadPalettesFonts()
{
    _palettes.data.reset();
    _palettes.elements.clear();
    _palettes.elements.shrink_to_fit();

    _fonts.data.reset();
    _fonts.elements.clear();
    _fonts.elements.shrink_to_fit();
}

static bool GfxLoadOpenRCT2Gx(std::string filename, Gx& target, size_t expectedNumItems)
{
    LOG_VERBOSE("GfxLoadOpenRCT2Gx(\"%s\")", filename.c_str());
    auto& env = GetContext()->GetPlatformEnvironment();

    std::string path = Path::Combine(env.GetDirectoryPath(DirBase::openrct2), filename.c_str());

    try
    {
        auto fs = FileStream(path, FileMode::open);
        target.header = fs.ReadValue<G1Header>();

        // Read element headers
        target.elements.resize(target.header.numEntries);
        ReadAndConvertGxDat(&fs, target.header.numEntries, target.elements.data());

        // Read element data
        target.data = fs.ReadArray<uint8_t>(target.header.totalSize);

        if (target.header.numEntries != expectedNumItems)
        {
            std::string errorMessage = "Mismatched " + filename + " size.\nExpected: " + std::to_string(expectedNumItems)
                + "\nActual: " + std::to_string(target.header.numEntries) + "\n" + filename
                + " may be installed improperly.\nPath to " + filename + ": " + path;

            LOG_ERROR(errorMessage.c_str());

            if (!gOpenRCT2Headless)
            {
                auto& uiContext = GetContext()->GetUiContext();
                uiContext.ShowMessageBox(errorMessage);
                uiContext.ShowMessageBox(
                    "Warning: You may experience graphical glitches if you continue. It's recommended "
                    "that you update "
                    + filename + " if you're seeing this message");
            }
        }

        // Fix entry data offsets
        for (uint32_t i = 0; i < target.header.numEntries; i++)
        {
            if (target.elements[i].offset == nullptr)
            {
                target.elements[i].offset = target.data.get();
            }
            else
            {
                target.elements[i].offset += reinterpret_cast<uintptr_t>(target.data.get());
            }
        }
        return true;
    }
    catch (const std::exception&)
    {
        target.elements.clear();
        target.elements.shrink_to_fit();

        LOG_FATAL("Unable to load %s graphics", filename.c_str());
        if (!gOpenRCT2Headless)
        {
            auto& uiContext = GetContext()->GetUiContext();
            uiContext.ShowMessageBox("Unable to load " + filename);
        }
    }
    return false;
}

void GfxLoadPalettesFonts()
{
    GfxLoadOpenRCT2Gx("palettes.dat", _palettes, kPalettesDatSpriteCount);
    GfxLoadOpenRCT2Gx("fonts.dat", _fonts, kFontsDatSpriteCount);
}

std::optional<Gx> GfxLoadGx(const std::vector<uint8_t>& buffer)
{
    try
    {
        MemoryStream istream(buffer.data(), buffer.size());
        Gx gx;

        gx.header = istream.ReadValue<G1Header>();

        // Read element headers
        gx.elements.resize(gx.header.numEntries);
        ReadAndConvertGxDat(&istream, gx.header.numEntries, gx.elements.data());

        // Read element data
        gx.data = istream.ReadArray<uint8_t>(gx.header.totalSize);

        return std::make_optional(std::move(gx));
    }
    catch (const std::exception&)
    {
        LOG_VERBOSE("Unable to load Gx graphics");
    }
    return std::nullopt;
}

static std::optional<PaletteMap> FASTCALL GfxDrawSpriteGetPalette(ImageId imageId)
{
    if (!imageId.HasSecondary())
    {
        uint8_t paletteId = imageId.GetRemap();
        if (!imageId.IsBlended())
        {
            paletteId &= 0x7F;
        }
        return GetPaletteMapForColour(static_cast<FilterPaletteID>(paletteId));
    }

    auto paletteMap = PaletteMap(std::span(reinterpret_cast<PaletteIndex*>(kSecondaryRemapPalette), 256));
    if (imageId.HasTertiary())
    {
        paletteMap = PaletteMap(std::span(reinterpret_cast<PaletteIndex*>(kTertiaryRemapPalette), 256));
        auto tertiaryPaletteMap = GetPaletteMapForColour(static_cast<FilterPaletteID>(imageId.GetTertiary()));
        if (tertiaryPaletteMap.has_value())
        {
            paletteMap.Copy(
                PaletteIndex::tertiaryRemap0, tertiaryPaletteMap.value(), PaletteIndex::primaryRemap0, kPaletteLengthRemap);
        }
    }

    auto primaryPaletteMap = GetPaletteMapForColour(static_cast<FilterPaletteID>(imageId.GetPrimary()));
    if (primaryPaletteMap.has_value())
    {
        paletteMap.Copy(
            PaletteIndex::primaryRemap0, primaryPaletteMap.value(), PaletteIndex::primaryRemap0, kPaletteLengthRemap);
    }

    auto secondaryPaletteMap = GetPaletteMapForColour(static_cast<FilterPaletteID>(imageId.GetSecondary()));
    if (secondaryPaletteMap.has_value())
    {
        paletteMap.Copy(
            PaletteIndex::secondaryRemap0, secondaryPaletteMap.value(), PaletteIndex::primaryRemap0, kPaletteLengthRemap);
    }

    return paletteMap;
}

void FASTCALL GfxDrawSpriteSoftware(RenderTarget& rt, const ImageId imageId, const ScreenCoordsXY& spriteCoords)
{
    if (imageId.HasValue())
    {
        auto palette = GfxDrawSpriteGetPalette(imageId);
        if (!palette)
        {
            palette = PaletteMap::GetDefault();
        }
        GfxDrawSpritePaletteSetSoftware(rt, imageId, spriteCoords, *palette);
    }
}

/*
 * rct: 0x0067A46E
 * image_id (ebx) and also (0x00EDF81C)
 * palette_pointer (0x9ABDA4)
 * unknown_pointer (0x9E3CDC)
 * rt (edi)
 * x (cx)
 * y (dx)
 */
void FASTCALL GfxDrawSpritePaletteSetSoftware(
    RenderTarget& rt, const ImageId imageId, const ScreenCoordsXY& coords, const PaletteMap& paletteMap)
{
    const auto zoomLevel = rt.zoom_level;
    int32_t x = coords.x;
    int32_t y = coords.y;

    const auto* g1 = GfxGetG1Element(imageId);
    if (g1 == nullptr)
    {
        return;
    }

    if (zoomLevel > ZoomLevel{ 0 } && g1->flags.has(G1Flag::hasZoomSprite))
    {
        RenderTarget zoomedRT = rt;
        zoomedRT.bits = rt.bits;
        zoomedRT.x = rt.x;
        zoomedRT.y = rt.y;
        zoomedRT.height = rt.height;
        zoomedRT.width = rt.width;
        zoomedRT.pitch = rt.pitch;
        zoomedRT.zoom_level = zoomLevel - 1;

        const auto spriteCoords = ScreenCoordsXY{ coords.x / 2, coords.y / 2 };
        GfxDrawSpritePaletteSetSoftware(
            zoomedRT, imageId.WithIndex(imageId.GetIndex() - g1->zoomedOffset), spriteCoords, paletteMap);
        return;
    }

    if (zoomLevel > ZoomLevel{ 0 } && g1->flags.has(G1Flag::noZoomDraw))
    {
        return;
    }

    // mber: There should not be two separate code paths for minifying and magnifying sprites.
    //       I haven't been able to refactor the code in a way that handles both cases properly with one code path.
    //       For the moment, I've added this block here just for magnification with the old code continuing below.
    if (zoomLevel < ZoomLevel{ 0 })
    {
        ScreenCoordsXY spriteTopLeft = { zoomLevel.ApplyInversedTo(coords.x + g1->xOffset),
                                         zoomLevel.ApplyInversedTo(coords.y + g1->yOffset) };

        ScreenCoordsXY spriteBottomLeft{ zoomLevel.ApplyInversedTo(coords.x + g1->xOffset + g1->width),
                                         zoomLevel.ApplyInversedTo(coords.y + g1->yOffset + g1->height) };

        const int32_t width = std::min(spriteBottomLeft.x, rt.x + rt.width) - std::max(spriteTopLeft.x, rt.x);
        const int32_t height = std::min(spriteBottomLeft.y, rt.y + rt.height) - std::max(spriteTopLeft.y, rt.y);

        if (width <= 0 || height <= 0)
            return;

        const int32_t offsetX = rt.x - spriteTopLeft.x;
        const int32_t offsetY = rt.y - spriteTopLeft.y;
        const int32_t srcX = std::max(0, offsetX);
        const int32_t srcY = std::max(0, offsetY);
        PaletteIndex* dst = rt.bits + std::max(0, -offsetX) + std::max(0, -offsetY) * rt.LineStride();

        DrawSpriteArgs args(imageId, paletteMap, *g1, srcX, srcY, width, height, dst);
        GfxSpriteToBuffer(rt, args);
        return;
    }

    const int32_t zoom_mask = zoomLevel > ZoomLevel{ 0 } ? zoomLevel.ApplyTo(0xFFFFFFFF) : 0xFFFFFFFF;

    if (zoomLevel > ZoomLevel{ 0 } && g1->flags.has(G1Flag::hasRLECompression))
    {
        x -= ~zoom_mask;
        y -= ~zoom_mask;
    }

    // This will be the height of the drawn image
    int32_t height = g1->height;

    // This is the start y coordinate on the destination
    int32_t dest_start_y = y + g1->yOffset;

    // For whatever reason the RLE version does not use
    // the zoom mask on the y coordinate but does on x.
    if (g1->flags.has(G1Flag::hasRLECompression))
    {
        dest_start_y -= rt.WorldY();
    }
    else
    {
        dest_start_y = (dest_start_y & zoom_mask) - rt.WorldY();
    }
    // This is the start y coordinate on the source
    int32_t source_start_y = 0;

    if (dest_start_y < 0)
    {
        // If the destination y is negative reduce the height of the
        // image as we will cut off the bottom
        height += dest_start_y;
        // If the image is no longer visible nothing to draw
        if (height <= 0)
        {
            return;
        }
        // The source image will start a further up the image
        source_start_y -= dest_start_y;
        // The destination start is now reset to 0
        dest_start_y = 0;
    }
    else
    {
        if (g1->flags.has(G1Flag::hasRLECompression) && zoomLevel > ZoomLevel{ 0 })
        {
            source_start_y -= dest_start_y & ~zoom_mask;
            height += dest_start_y & ~zoom_mask;
        }
    }

    int32_t dest_end_y = dest_start_y + height;

    if (dest_end_y > rt.WorldHeight())
    {
        // If the destination y is outside of the drawing
        // image reduce the height of the image
        height -= dest_end_y - rt.WorldHeight();
    }
    // If the image no longer has anything to draw
    if (height <= 0)
        return;

    dest_start_y = zoomLevel.ApplyInversedTo(dest_start_y);

    // This will be the width of the drawn image
    int32_t width = g1->width;

    // This is the source start x coordinate
    int32_t source_start_x = 0;
    // This is the destination start x coordinate
    int32_t dest_start_x = ((x + g1->xOffset + ~zoom_mask) & zoom_mask) - rt.WorldX();

    if (dest_start_x < 0)
    {
        // If the destination is negative reduce the width
        // image will cut off the side
        width += dest_start_x;
        // If there is no image to draw
        if (width <= 0)
        {
            return;
        }
        // The source start will also need to cut off the side
        source_start_x -= dest_start_x;
        // Reset the destination to 0
        dest_start_x = 0;
    }
    else
    {
        if (g1->flags.has(G1Flag::hasRLECompression) && zoomLevel > ZoomLevel{ 0 })
        {
            source_start_x -= dest_start_x & ~zoom_mask;
        }
    }

    int32_t dest_end_x = dest_start_x + width;

    if (dest_end_x > rt.WorldWidth())
    {
        // If the destination x is outside of the drawing area
        // reduce the image width.
        width -= dest_end_x - rt.WorldWidth();
        // If there is no image to draw.
        if (width <= 0)
            return;
    }

    dest_start_x = zoomLevel.ApplyInversedTo(dest_start_x);

    PaletteIndex* dest_pointer = rt.bits;
    // Move the pointer to the start point of the destination
    dest_pointer += (zoomLevel.ApplyInversedTo(rt.WorldWidth()) + rt.pitch) * dest_start_y + dest_start_x;

    DrawSpriteArgs args(imageId, paletteMap, *g1, source_start_x, source_start_y, width, height, dest_pointer);
    GfxSpriteToBuffer(rt, args);
}

void FASTCALL GfxSpriteToBuffer(RenderTarget& rt, const DrawSpriteArgs& args)
{
    if (args.SourceImage.flags.has(G1Flag::hasRLECompression))
    {
        GfxRleSpriteToBuffer(rt, args);
    }
    else if (!args.SourceImage.flags.has(G1Flag::one))
    {
        GfxBmpSpriteToBuffer(rt, args);
    }
}

/**
 * Draws the given colour image masked out by the given mask image. This can currently only cope with bitmap formatted mask and
 * colour images. Presumably the original game never used RLE images for masking. Colour 0 represents transparent.
 *
 *  rct2: 0x00681DE2
 */
void FASTCALL GfxDrawSpriteRawMaskedSoftware(
    RenderTarget& rt, const ScreenCoordsXY& scrCoords, const ImageId maskImage, const ImageId colourImage)
{
    int32_t left, top, right, bottom, width, height;
    auto imgMask = GfxGetG1Element(maskImage);
    auto imgColour = GfxGetG1Element(colourImage);
    if (imgMask == nullptr || imgColour == nullptr)
    {
        return;
    }

    // Masking only works with non-RLE images. Fall back to regular drawing for RLE images.
    if (imgMask->flags.has(G1Flag::hasRLECompression) || imgColour->flags.has(G1Flag::hasRLECompression))
    {
        GfxDrawSpriteSoftware(rt, colourImage, scrCoords);
        return;
    }

    ZoomLevel zoom = rt.zoom_level;
    if (rt.zoom_level > ZoomLevel{ 0 })
    {
        assert(false);
    }

    width = zoom.ApplyInversedTo(std::min(imgMask->width, imgColour->width));
    height = zoom.ApplyInversedTo(std::min(imgMask->height, imgColour->height));

    ScreenCoordsXY offsetCoords = scrCoords + ScreenCoordsXY{ imgMask->xOffset, imgMask->yOffset };
    offsetCoords.x = zoom.ApplyInversedTo(offsetCoords.x);
    offsetCoords.y = zoom.ApplyInversedTo(offsetCoords.y);

    left = std::max(rt.x, offsetCoords.x);
    top = std::max(rt.y, offsetCoords.y);
    right = std::min(rt.x + rt.width, offsetCoords.x + width);
    bottom = std::min(rt.y + rt.height, offsetCoords.y + height);

    width = right - left;
    height = bottom - top;
    if (width < 0 || height < 0)
        return;

    PaletteIndex* dst = rt.bits + (left - rt.x) + ((top - rt.y) * rt.LineStride());
    int32_t skipX = left - offsetCoords.x;
    int32_t skipY = top - offsetCoords.y;
    if (zoom < ZoomLevel{ 0 })
    {
        MaskMagnify(
            zoom, width, height, imgMask->offset, imgColour->offset, dst, imgMask->width, imgColour->width, rt.LineStride(),
            skipX, skipY);
        return;
    }

    uint8_t const* maskSrc = imgMask->offset + (skipY * imgMask->width) + skipX;
    uint8_t const* colourSrc = imgColour->offset + (skipY * imgColour->width) + skipX;

    int32_t maskWrap = imgMask->width - width;
    int32_t colourWrap = imgColour->width - width;
    int32_t dstWrap = rt.LineStride() - width;

    MaskFn(width, height, maskSrc, colourSrc, dst, maskWrap, colourWrap, dstWrap);
}

const G1Element* GfxGetG1Element(const ImageId imageId)
{
    return GfxGetG1Element(imageId.GetIndex());
}

const G1Element* GfxGetG1Element(ImageIndex image_id)
{
    Guard::Assert(!gOpenRCT2NoGraphics, "GfxGetG1Element called on headless instance");

    auto offset = static_cast<size_t>(image_id);
    if (offset == kImageIndexUndefined)
    {
        return nullptr;
    }

    if (offset >= SPR_TEMP_BEGIN && offset < SPR_TEMP_END)
    {
        return &_g1Temp[offset - SPR_TEMP_BEGIN];
    }

    if (offset < SPR_G2_END)
    {
        if (offset < _base.elements.size())
        {
            return &_base.elements[offset];
        }
    }
    else if (offset < SPR_PALETTE_END)
    {
        size_t idx = offset - SPR_PALETTE_START;
        if (idx < _palettes.header.numEntries)
        {
            return &_palettes.elements[idx];
        }

        LOG_WARNING("Invalid entry in palettes.dat requested, idx = %u. You may have to update your palettes.dat.", idx);
    }
    else if (offset < SPR_FONTS_END)
    {
        size_t idx = offset - SPR_FONTS_BEGIN;
        if (idx < _fonts.header.numEntries)
        {
            return &_fonts.elements[idx];
        }

        LOG_WARNING("Invalid entry in fonts.dat requested, idx = %u. You may have to update your fonts.dat.", idx);
    }
    else if (offset < SPR_CSG_END)
    {
        // Reserved ranges (formerly ride track and RCT1 images), nothing is loaded there.
        return nullptr;
    }
    else if (offset < SPR_SCROLLING_TEXT_END)
    {
        size_t idx = offset - SPR_SCROLLING_TEXT_START;
        if (idx < std::size(_scrollingText))
        {
            return &_scrollingText[idx];
        }
    }
    else if (offset < SPR_IMAGE_LIST_END)
    {
        size_t idx = offset - SPR_IMAGE_LIST_BEGIN;
        if (idx < _imageListElements.size())
        {
            return &_imageListElements[idx];
        }
    }
    return nullptr;
}

const G1Palette* GfxGetG1Palette(ImageIndex imageId)
{
    const auto* element = GfxGetG1Element(imageId);
    if (element == nullptr)
        return nullptr;

    return element->asPalette();
}

void GfxSetG1Element(ImageIndex imageId, const G1Element* g1)
{
    bool isTemp = (imageId >= SPR_TEMP_BEGIN && imageId < SPR_TEMP_END);
    bool isValid = (imageId >= SPR_IMAGE_LIST_BEGIN && imageId < SPR_IMAGE_LIST_END)
        || (imageId >= SPR_SCROLLING_TEXT_START && imageId < SPR_SCROLLING_TEXT_END);

#if DEBUG > 0
    Guard::Assert(!gOpenRCT2NoGraphics, "GfxSetG1Element called on headless instance");
    Guard::Assert(isValid || isTemp, "GfxSetG1Element called with unexpected image id");
    Guard::Assert(g1 != nullptr, "g1 was nullptr");
#endif

    if (g1 != nullptr)
    {
        if (isTemp)
        {
            _g1Temp[imageId - SPR_TEMP_BEGIN] = *g1;
        }
        else if (isValid)
        {
            if (imageId < SPR_SCROLLING_TEXT_END)
            {
                size_t idx = static_cast<size_t>(imageId) - SPR_SCROLLING_TEXT_START;
                if (idx < std::size(_scrollingText))
                {
                    _scrollingText[idx] = *g1;
                }
            }
            else
            {
                size_t idx = static_cast<size_t>(imageId) - SPR_IMAGE_LIST_BEGIN;
                // Grow the element buffer if necessary
                while (idx >= _imageListElements.size())
                {
                    _imageListElements.resize(std::max<size_t>(256, _imageListElements.size() * 2));
                }
                _imageListElements[idx] = *g1;
            }
        }
    }
}


size_t G1CalculateDataSize(const G1Element* g1)
{
    if (const auto* asPalette = g1->asPalette())
    {
        return asPalette->numColours * 3;
    }

    if (g1->flags.has(G1Flag::hasRLECompression))
    {
        if (g1->offset == nullptr)
        {
            return 0;
        }

        auto idx = (g1->height - 1) * 2;
        uint16_t offset = g1->offset[idx] | (g1->offset[idx + 1] << 8);
        uint8_t* ptr = g1->offset + offset;
        bool endOfLine = false;
        do
        {
            uint8_t chunk0 = *ptr++;
            ptr++; // offset
            uint8_t chunkSize = chunk0 & 0x7F;
            ptr += chunkSize;
            endOfLine = (chunk0 & 0x80) != 0;
        } while (!endOfLine);
        return ptr - g1->offset;
    }

    return g1->width * g1->height;
}
