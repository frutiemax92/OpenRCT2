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
    // The procedural sprites of the golf ball (BAL-002): a 2 x 2 pixel ball, a dot for the trail and a shadow that
    // darkens the ground. They are made in code so the game needs no ball art, and registered once on first use.
    struct GolfBallSprites
    {
        static constexpr int32_t kBallSize = 2;
        static constexpr int32_t kShadowWidth = 6;
        static constexpr int32_t kShadowHeight = 3;

        ImageIndex ball = kImageIndexUndefined;
        ImageIndex trail = kImageIndexUndefined;
        ImageIndex shadow = kImageIndexUndefined;

        static const GolfBallSprites& Get();
        // Frees the images (shutdown, tests).
        static void Release();
    };

    // Pixel data of the sprites, exposed for tests. Index 0 is transparent.
    std::vector<uint8_t> MakeGolfBallPixels(uint8_t colour);
    std::vector<uint8_t> MakeGolfBallShadowPixels();
    // The palette index closest to white.
    uint8_t FindWhiteIndex();
} // namespace OpenRCT2::Drawing
