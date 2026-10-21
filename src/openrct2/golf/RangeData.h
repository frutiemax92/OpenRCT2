/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "Hole.h"

#include <cstdint>
#include <optional>
#include <vector>

namespace OpenRCT2::Golf
{
    // The data of a driving range (section 17), kept with the courses and saved with them.

    struct DistanceSign
    {
        uint32_t id = 0;
        HolePoint position;
    };

    struct DrivingRange
    {
        uint32_t id = 0;
        TileCoordsXY areaMin;                  // DRV-001: the area drawn with the zone tool, inclusive
        TileCoordsXY areaMax;
        std::vector<HolePoint> hittingTiles;   // where golfers stand; world positions of the tile centres
        std::optional<HolePoint> kiosk;        // the one driving range kiosk building
        std::vector<DistanceSign> signs;
        int64_t bucketPrice = 300;
        int32_t ballsPerBucket = 30;           // DRV-005
        int32_t shootingDirectionDegrees = 0;  // 0 = +x, 90 = +y: where the golfers aim

        bool containsArea(const HolePoint& p) const;
        bool containsTile(const TileCoordsXY& tile) const
        {
            return tile.x >= areaMin.x && tile.x <= areaMax.x && tile.y >= areaMin.y && tile.y <= areaMax.y;
        }
    };
} // namespace OpenRCT2::Golf
