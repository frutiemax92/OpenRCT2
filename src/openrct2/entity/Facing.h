/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../world/Location.hpp"

#include <cmath>
#include <cstdint>

namespace OpenRCT2
{
    // Entities are placed on whole units, so the step between two ticks of a slow walker is something like (1, 0) then
    // (1, 1): a heading taken from it flips between the 8 directions. The heading is instead taken from the distance
    // travelled since the last time it was updated, once that is long enough to give a fine angle.
    constexpr int32_t kFacingMinimumTravel = 6;

    // Yaw in turns of 256, counter-clockwise from above, 0 facing -x, of an entity looking along a map direction.
    inline uint8_t YawFacing(double dx, double dy)
    {
        // atan2 gives 0 for +x; the direction numbers start at -x, so a half turn is added.
        double turns = std::atan2(dy, dx) / (2.0 * 3.14159265358979323846) + 0.5;
        turns -= std::floor(turns);
        return static_cast<uint8_t>(static_cast<int32_t>(turns * 256.0) & 0xFF);
    }

    inline void UpdateFacingFromTravel(uint8_t& yaw, CoordsXY& from, const CoordsXY& now)
    {
        const double dx = static_cast<double>(now.x - from.x);
        const double dy = static_cast<double>(now.y - from.y);
        if (dx * dx + dy * dy < static_cast<double>(kFacingMinimumTravel) * kFacingMinimumTravel)
            return;
        yaw = YawFacing(dx, dy);
        from = now;
    }
} // namespace OpenRCT2
