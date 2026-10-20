/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "GolfBallSprites.h"

#include "G1Element.h"
#include "Image.h"
#include "ImageImporter.h"

#include <limits>

namespace OpenRCT2::Drawing
{
    namespace
    {
        struct SpriteData
        {
            GolfBallSprites sprites;
            std::vector<uint8_t> ball;
            std::vector<uint8_t> trail;
            std::vector<uint8_t> shadow;
            bool created = false;
        };

        SpriteData& data()
        {
            static SpriteData d;
            return d;
        }

        ImageIndex allocate(std::vector<uint8_t>& pixels, int32_t width, int32_t height, int32_t xOffset, int32_t yOffset)
        {
            G1Element g1{};
            g1.offset = pixels.data();
            g1.width = static_cast<int16_t>(width);
            g1.height = static_cast<int16_t>(height);
            g1.xOffset = static_cast<int16_t>(xOffset);
            g1.yOffset = static_cast<int16_t>(yOffset);
            g1.flags = { G1Flag::hasTransparency };
            return GfxObjectAllocateImages(&g1, 1);
        }
    } // namespace

    uint8_t FindWhiteIndex()
    {
        int32_t best = std::numeric_limits<int32_t>::max();
        uint8_t bestIndex = 1;
        for (int32_t i = 1; i < 256; i++)
        {
            const int32_t dr = 255 - StandardPalette[i].red;
            const int32_t dg = 255 - StandardPalette[i].green;
            const int32_t db = 255 - StandardPalette[i].blue;
            const int32_t distance = dr * dr + dg * dg + db * db;
            if (distance < best)
            {
                best = distance;
                bestIndex = static_cast<uint8_t>(i);
            }
        }
        return bestIndex;
    }

    std::vector<uint8_t> MakeGolfBallPixels(uint8_t colour)
    {
        return std::vector<uint8_t>(GolfBallSprites::kBallSize * GolfBallSprites::kBallSize, colour);
    }

    std::vector<uint8_t> MakeGolfBallShadowPixels()
    {
        // A flat ellipse; every drawn pixel is a mask for the darkening filter.
        constexpr int32_t w = GolfBallSprites::kShadowWidth;
        constexpr int32_t h = GolfBallSprites::kShadowHeight;
        std::vector<uint8_t> pixels(static_cast<size_t>(w * h), 0);
        constexpr uint8_t rows[h][w] = {
            { 0, 1, 1, 1, 1, 0 },
            { 1, 1, 1, 1, 1, 1 },
            { 0, 1, 1, 1, 1, 0 },
        };
        for (int32_t y = 0; y < h; y++)
            for (int32_t x = 0; x < w; x++)
                pixels[static_cast<size_t>(y * w + x)] = rows[y][x];
        return pixels;
    }

    const GolfBallSprites& GolfBallSprites::Get()
    {
        auto& d = data();
        if (!d.created)
        {
            const auto white = FindWhiteIndex();
            d.ball = MakeGolfBallPixels(white);
            d.trail = { white };
            d.shadow = MakeGolfBallShadowPixels();
            // Offsets put the sprite centre on the projected position of the entity.
            d.sprites.ball = allocate(d.ball, kBallSize, kBallSize, -kBallSize / 2, -kBallSize / 2);
            d.sprites.trail = allocate(d.trail, 1, 1, 0, 0);
            d.sprites.shadow = allocate(d.shadow, kShadowWidth, kShadowHeight, -kShadowWidth / 2, -kShadowHeight / 2);
            d.created = true;
        }
        return d.sprites;
    }

    void GolfBallSprites::Release()
    {
        auto& d = data();
        if (!d.created)
            return;
        for (auto index : { d.sprites.ball, d.sprites.trail, d.sprites.shadow })
            if (index != kImageIndexUndefined)
                GfxObjectFreeImages(index, 1);
        d = SpriteData{};
    }
} // namespace OpenRCT2::Drawing
