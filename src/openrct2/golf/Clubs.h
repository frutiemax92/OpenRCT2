/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../core/Fixed.hpp"

#include <array>
#include <cstdint>
#include <span>

namespace OpenRCT2::Golf
{
    // Golf clubs (BAL-010, BAL-011), a hard-coded table first; a `club_set` object may replace it later.
    enum class ClubId : uint8_t
    {
        driver,
        wood3,
        wood5,
        iron3,
        iron4,
        iron5,
        iron6,
        iron7,
        iron8,
        iron9,
        pitchingWedge,
        sandWedge,
        putter,
        miniPutter,
        count,
    };

    constexpr size_t kClubCount = static_cast<size_t>(ClubId::count);

    struct ClubInfo
    {
        ClubId id;
        const char* name;
        int32_t loftDegrees;       // launch angle; 0 for the putters
        int32_t carryYards;        // base carry for skill 50 on flat ground without wind (BAL-011)
        Fixed backSpin;            // back spin given to the ball, 0..1
        Fixed accuracyModifier;    // multiplier on the error spread: long clubs are harder to control
        Fixed launchScale;         // calibration of the launch speed so the simulated carry matches carryYards
        bool isPutter;
    };

    const ClubInfo& clubInfo(ClubId id);
    std::span<const ClubInfo> allClubs();

    // 1 tile = 5 yards = 32 world units.
    constexpr Fixed yardsToUnits(int64_t yards)
    {
        return Fixed::ratio(yards * 32, 5);
    }
    constexpr Fixed unitsToYards(Fixed units)
    {
        return units * 5 / 32;
    }
} // namespace OpenRCT2::Golf
