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

namespace OpenRCT2::Golf
{
    // What the player chose for a building with services, and how it is doing (section 9.3).
    struct BuildingBook
    {
        bool closed = false;
        int64_t price = -1;          // what a visit costs; -1 for the usual price of its kind
        int64_t revenueThisMonth = 0;
        int64_t revenueLastMonth = 0;
        int32_t visitorsThisMonth = 0;
        int32_t visitorsLastMonth = 0;

        void endMonth()
        {
            revenueLastMonth = revenueThisMonth;
            visitorsLastMonth = visitorsThisMonth;
            revenueThisMonth = 0;
            visitorsThisMonth = 0;
        }
    };

    // The id of a building: the tile it stands on, the one at its origin.
    constexpr uint32_t buildingKey(int32_t tileX, int32_t tileY)
    {
        return static_cast<uint32_t>(tileY) * 1024u + static_cast<uint32_t>(tileX) + 1u;
    }

    // Game days in the month the building books are kept by (30, as for the loan interest).
    constexpr uint32_t kBuildingMonthDays = 30;
} // namespace OpenRCT2::Golf
