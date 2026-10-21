/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "ImageIndexType.h"

#include <array>
#include <cstdint>
#include <vector>

namespace OpenRCT2::Drawing
{
    // The procedural sprites of the hole markers (HOL-002, HOL-003): a tee marker per tee category, the flag pin and the
    // dark ring of the cup, and the aim point. Made in code, like the ball sprites, so the game needs no art for them.
    struct HoleMarkerSprites
    {
        static constexpr size_t kTeeCategories = 3;

        std::array<ImageIndex, kTeeCategories> tee{ kImageIndexUndefined, kImageIndexUndefined, kImageIndexUndefined };
        ImageIndex pin = kImageIndexUndefined;
        ImageIndex cup = kImageIndexUndefined;
        ImageIndex aim = kImageIndexUndefined;

        static const HoleMarkerSprites& Get();
        static void Release();
    };

    // A sprite as pixels, with the offset that puts the foot of the sprite on the projected point. Index 0 is transparent.
    struct MarkerPixels
    {
        int32_t width = 0;
        int32_t height = 0;
        int32_t xOffset = 0;
        int32_t yOffset = 0;
        std::vector<uint8_t> pixels;
    };

    // Colours of the tee markers by category, as RGB (men blue, women red, children yellow).
    struct MarkerColour
    {
        uint8_t red, green, blue;
    };
    MarkerColour TeeMarkerColour(size_t category);

    MarkerPixels MakeTeeMarkerPixels(size_t category);
    MarkerPixels MakePinPixels();
    MarkerPixels MakeCupPixels();
    MarkerPixels MakeAimPixels();
    // The palette index closest to a colour.
    uint8_t FindPaletteIndex(uint8_t red, uint8_t green, uint8_t blue);
} // namespace OpenRCT2::Drawing
