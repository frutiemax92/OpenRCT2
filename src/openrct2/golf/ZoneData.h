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

#include <cstdint>

namespace OpenRCT2::Golf
{
    // The zones the player draws (section 22): cottage lots, and the areas a tournament needs. They are kept with the courses
    // and saved with them.

    struct CottageLot
    {
        uint32_t id = 0;
        TileCoordsXY min;            // tile rectangle, inclusive
        TileCoordsXY max;
        bool sold = false;
        uint32_t ownerGolferId = 0;  // the VIP who bought it
        uint32_t buildingObject = 0; // the cottage placed on it
        int64_t pricePaid = 0;

        int32_t width() const
        {
            return max.x - min.x + 1;
        }
        int32_t depth() const
        {
            return max.y - min.y + 1;
        }
        bool contains(const TileCoordsXY& tile) const
        {
            return tile.x >= min.x && tile.x <= max.x && tile.y >= min.y && tile.y <= max.y;
        }
    };

    // ECO-041: the reserved areas of a tournament.
    enum class TournamentZoneKind : uint8_t
    {
        spectator,
        media,
        extraParking,
        grandstand,
        count,
    };

    struct TournamentZone
    {
        uint32_t id = 0;
        TournamentZoneKind kind = TournamentZoneKind::spectator;
        TileCoordsXY min; // tile rectangle, inclusive
        TileCoordsXY max;

        int32_t tiles() const
        {
            return (max.x - min.x + 1) * (max.y - min.y + 1);
        }
        bool contains(const TileCoordsXY& tile) const
        {
            return tile.x >= min.x && tile.x <= max.x && tile.y >= min.y && tile.y <= max.y;
        }
    };
} // namespace OpenRCT2::Golf
