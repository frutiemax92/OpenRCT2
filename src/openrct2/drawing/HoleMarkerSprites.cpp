/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "HoleMarkerSprites.h"

#include "G1Element.h"
#include "Image.h"
#include "ImageImporter.h"

#include <cstdlib>
#include <limits>
#include <string>
#include <unordered_map>

namespace OpenRCT2::Drawing
{
    namespace
    {
        struct SpriteData
        {
            HoleMarkerSprites sprites;
            std::unordered_map<int32_t, ImageIndex> numbers;
            std::vector<MarkerPixels> keep; // the pixels must live as long as the images are registered
            bool created = false;
        };

        SpriteData& data()
        {
            static SpriteData d;
            return d;
        }

        ImageIndex allocate(MarkerPixels& marker)
        {
            G1Element g1{};
            g1.offset = marker.pixels.data();
            g1.width = static_cast<int16_t>(marker.width);
            g1.height = static_cast<int16_t>(marker.height);
            g1.xOffset = static_cast<int16_t>(marker.xOffset);
            g1.yOffset = static_cast<int16_t>(marker.yOffset);
            g1.flags = { G1Flag::hasTransparency };
            return GfxObjectAllocateImages(&g1, 1);
        }

        MarkerPixels blank(int32_t width, int32_t height, int32_t xOffset, int32_t yOffset)
        {
            MarkerPixels m;
            m.width = width;
            m.height = height;
            m.xOffset = xOffset;
            m.yOffset = yOffset;
            m.pixels.assign(static_cast<size_t>(width * height), 0);
            return m;
        }

        void plot(MarkerPixels& m, int32_t x, int32_t y, uint8_t colour)
        {
            if (x >= 0 && y >= 0 && x < m.width && y < m.height)
                m.pixels[static_cast<size_t>(y * m.width + x)] = colour;
        }

        void rect(MarkerPixels& m, int32_t x, int32_t y, int32_t w, int32_t h, uint8_t colour)
        {
            for (int32_t j = 0; j < h; j++)
                for (int32_t i = 0; i < w; i++)
                    plot(m, x + i, y + j, colour);
        }
    } // namespace

    uint8_t FindPaletteIndex(uint8_t red, uint8_t green, uint8_t blue)
    {
        int32_t best = std::numeric_limits<int32_t>::max();
        uint8_t bestIndex = 1;
        for (int32_t i = 1; i < 256; i++)
        {
            const int32_t dr = red - StandardPalette[i].red;
            const int32_t dg = green - StandardPalette[i].green;
            const int32_t db = blue - StandardPalette[i].blue;
            const int32_t distance = dr * dr + dg * dg + db * db;
            if (distance < best)
            {
                best = distance;
                bestIndex = static_cast<uint8_t>(i);
            }
        }
        return bestIndex;
    }

    MarkerColour TeeMarkerColour(size_t category)
    {
        switch (category)
        {
            case 0:
                return { 50, 100, 230 }; // men
            case 1:
                return { 225, 50, 60 }; // women
            default:
                return { 245, 205, 40 }; // children
        }
    }

    MarkerPixels MakeTeeMarkerPixels(size_t category)
    {
        // A short post with a ball on top, 3 wide and 6 tall; the foot is the bottom middle.
        const auto c = TeeMarkerColour(category);
        const uint8_t colour = FindPaletteIndex(c.red, c.green, c.blue);
        const uint8_t dark = FindPaletteIndex(c.red / 2, c.green / 2, c.blue / 2);
        auto m = blank(3, 6, -1, -6);
        rect(m, 0, 0, 3, 3, colour);
        plot(m, 0, 2, dark);
        plot(m, 2, 2, dark);
        rect(m, 1, 3, 1, 3, dark);
        return m;
    }

    MarkerPixels MakePinPixels()
    {
        // A white pole 1 pixel wide and 20 tall with a red flag on top; the foot is the bottom of the pole.
        const uint8_t white = FindPaletteIndex(255, 255, 255);
        const uint8_t red = FindPaletteIndex(225, 40, 40);
        auto m = blank(7, 20, -1, -20);
        rect(m, 0, 0, 1, 20, white);
        rect(m, 1, 0, 6, 4, red);
        plot(m, 6, 0, 0);
        plot(m, 6, 3, 0);
        return m;
    }

    MarkerPixels MakeCupPixels()
    {
        // The dark hole in the green, a flat ellipse of 6 x 3 pixels.
        const uint8_t dark = FindPaletteIndex(20, 20, 20);
        auto m = blank(6, 3, -3, -2);
        constexpr uint8_t rows[3][6] = {
            { 0, 1, 1, 1, 1, 0 },
            { 1, 1, 1, 1, 1, 1 },
            { 0, 1, 1, 1, 1, 0 },
        };
        for (int32_t y = 0; y < 3; y++)
            for (int32_t x = 0; x < 6; x++)
                if (rows[y][x])
                    plot(m, x, y, dark);
        return m;
    }

    MarkerPixels MakeAimPixels()
    {
        // A small orange diamond with a white middle, 7 x 7.
        const uint8_t orange = FindPaletteIndex(250, 140, 20);
        const uint8_t white = FindPaletteIndex(255, 255, 255);
        auto m = blank(7, 7, -3, -6);
        for (int32_t y = 0; y < 7; y++)
        {
            const int32_t half = 3 - std::abs(y - 3);
            rect(m, 3 - half, y, 2 * half + 1, 1, orange);
        }
        plot(m, 3, 3, white);
        return m;
    }

    MarkerPixels MakeHittingTilePixels()
    {
        // A diamond 28 wide and 14 high, a darker rim round a lighter mat.
        const uint8_t mat = FindPaletteIndex(150, 190, 90);
        const uint8_t rim = FindPaletteIndex(250, 240, 200);
        auto m = blank(28, 14, -14, -7);
        for (int32_t y = 0; y < 14; y++)
        {
            const int32_t half = 14 - std::abs(2 * y - 13) - 1;
            const int32_t left = 14 - half - 1;
            const int32_t width = 2 * half + 2;
            for (int32_t x = 0; x < width; x++)
            {
                const bool edge = x < 2 || x >= width - 2 || y == 0 || y == 13;
                plot(m, left + x, y, edge ? rim : mat);
            }
        }
        return m;
    }

    namespace
    {
        // A 3 x 5 pixel font for the digits, one row of three bits each (bit 2 is the left pixel).
        constexpr uint8_t kDigitRows[10][5] = {
            { 7, 5, 5, 5, 7 }, { 2, 6, 2, 2, 7 }, { 7, 1, 7, 4, 7 }, { 7, 1, 7, 1, 7 }, { 5, 5, 7, 1, 1 },
            { 7, 4, 7, 1, 7 }, { 7, 4, 7, 5, 7 }, { 7, 1, 1, 1, 1 }, { 7, 5, 7, 5, 7 }, { 7, 5, 7, 1, 7 },
        };
    } // namespace

    MarkerPixels MakeNumberPlatePixels(int32_t number)
    {
        const std::string digits = std::to_string(number < 0 ? 0 : number);
        const int32_t width = static_cast<int32_t>(digits.size()) * 4 + 2;
        const int32_t height = 9;
        const uint8_t white = FindPaletteIndex(250, 250, 250);
        const uint8_t black = FindPaletteIndex(10, 10, 10);
        // The foot of the plate is the point it hangs over: the bottom middle.
        auto m = blank(width, height, -width / 2, -height);
        rect(m, 0, 0, width, height, white);
        for (size_t i = 0; i < digits.size(); i++)
        {
            const auto& rows = kDigitRows[digits[i] - '0'];
            for (int32_t row = 0; row < 5; row++)
                for (int32_t column = 0; column < 3; column++)
                    if (rows[row] & (4 >> column))
                        plot(m, 1 + static_cast<int32_t>(i) * 4 + column, 2 + row, black);
        }
        return m;
    }

    void HoleMarkerSprites::EnsureNumber(int32_t number)
    {
        auto& d = data();
        if (d.numbers.find(number) != d.numbers.end())
            return;
        // The pixels must outlive the image, like the others: they are kept with the sprites.
        d.keep.push_back(MakeNumberPlatePixels(number));
        d.numbers[number] = allocate(d.keep.back());
    }

    ImageIndex HoleMarkerSprites::NumberImage(int32_t number)
    {
        auto& d = data();
        const auto it = d.numbers.find(number);
        return it == d.numbers.end() ? kImageIndexUndefined : it->second;
    }

    const HoleMarkerSprites& HoleMarkerSprites::Get()
    {
        auto& d = data();
        if (!d.created)
        {
            d.keep.reserve(kTeeCategories + 4);
            for (size_t i = 0; i < kTeeCategories; i++)
            {
                d.keep.push_back(MakeTeeMarkerPixels(i));
                d.sprites.tee[i] = allocate(d.keep.back());
            }
            d.keep.push_back(MakePinPixels());
            d.sprites.pin = allocate(d.keep.back());
            d.keep.push_back(MakeCupPixels());
            d.sprites.cup = allocate(d.keep.back());
            d.keep.push_back(MakeAimPixels());
            d.sprites.aim = allocate(d.keep.back());
            d.keep.push_back(MakeHittingTilePixels());
            d.sprites.hittingTile = allocate(d.keep.back());
            d.created = true;
        }
        return d.sprites;
    }

    void HoleMarkerSprites::Release()
    {
        auto& d = data();
        if (!d.created)
            return;
        for (auto index : d.sprites.tee)
            if (index != kImageIndexUndefined)
                GfxObjectFreeImages(index, 1);
        for (auto index : { d.sprites.pin, d.sprites.cup, d.sprites.aim, d.sprites.hittingTile })
            if (index != kImageIndexUndefined)
                GfxObjectFreeImages(index, 1);
        for (const auto& [number, index] : d.numbers)
            if (index != kImageIndexUndefined)
                GfxObjectFreeImages(index, 1);
        d = SpriteData{};
    }
} // namespace OpenRCT2::Drawing
