/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include <cstdint>

struct TileCoordsXY;

namespace OpenRCT2::World::MapGenerator
{
    enum class SmoothStrength : uint8_t
    {
        strong, // tiles may move up to 4 Q from their own height
        weak,   // up to 2 Q
    };

    // Turns the base heights of the interior tiles into continuous quarter-step terrain without cliffs (TER-017):
    // each tile corner takes the average height of the tiles around it.
    void smoothMap(TileCoordsXY mapSize, SmoothStrength strength);
} // namespace OpenRCT2::World::MapGenerator
