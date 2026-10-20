/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Clubs.h"

namespace OpenRCT2::Golf
{
    namespace
    {
        constexpr Fixed kScale1 = Fixed::ratio(1, 1); // putters do not fly

        // id, name, loft, carry yards, back spin, accuracy modifier, launch scale, putter. The launch scale makes the
        // simulated carry (BallParams defaults, flat ground, perfect shot) equal the base carry; ShotTests checks it.
        constexpr std::array<ClubInfo, kClubCount> kClubs = { {
            { ClubId::driver, "Driver", 11, 230, Fixed::ratio(20, 100), Fixed::ratio(130, 100), Fixed::ratio(10131, 10000), false },
            { ClubId::wood3, "3 wood", 15, 210, Fixed::ratio(25, 100), Fixed::ratio(115, 100), Fixed::ratio(10149, 10000), false },
            { ClubId::wood5, "5 wood", 18, 195, Fixed::ratio(30, 100), Fixed::ratio(110, 100), Fixed::ratio(10129, 10000), false },
            { ClubId::iron3, "3 iron", 21, 180, Fixed::ratio(40, 100), Fixed::ratio(100, 100), Fixed::ratio(10101, 10000), false },
            { ClubId::iron4, "4 iron", 24, 170, Fixed::ratio(45, 100), Fixed::ratio(100, 100), Fixed::ratio(10109, 10000), false },
            { ClubId::iron5, "5 iron", 27, 160, Fixed::ratio(50, 100), Fixed::ratio(95, 100), Fixed::ratio(10104, 10000), false },
            { ClubId::iron6, "6 iron", 31, 150, Fixed::ratio(55, 100), Fixed::ratio(95, 100), Fixed::ratio(10122, 10000), false },
            { ClubId::iron7, "7 iron", 35, 140, Fixed::ratio(60, 100), Fixed::ratio(90, 100), Fixed::ratio(10118, 10000), false },
            { ClubId::iron8, "8 iron", 39, 130, Fixed::ratio(70, 100), Fixed::ratio(90, 100), Fixed::ratio(10121, 10000), false },
            { ClubId::iron9, "9 iron", 43, 120, Fixed::ratio(75, 100), Fixed::ratio(85, 100), Fixed::ratio(10115, 10000), false },
            { ClubId::pitchingWedge, "Pitching wedge", 47, 105, Fixed::ratio(85, 100), Fixed::ratio(80, 100), Fixed::ratio(10109, 10000), false },
            { ClubId::sandWedge, "Sand wedge", 55, 80, Fixed::ratio(90, 100), Fixed::ratio(80, 100), Fixed::ratio(10150, 10000), false },
            // The putters have no carry: the base distance is the longest putt (power 1) rolled along the ground.
            { ClubId::putter, "Putter", 0, 40, Fixed{}, Fixed::ratio(50, 100), kScale1, true },
            { ClubId::miniPutter, "Mini-putt club", 0, 30, Fixed{}, Fixed::ratio(40, 100), kScale1, true },
        } };
    } // namespace

    const ClubInfo& clubInfo(ClubId id)
    {
        const auto index = static_cast<size_t>(id);
        return kClubs[index < kClubCount ? index : 0];
    }

    std::span<const ClubInfo> allClubs()
    {
        return kClubs;
    }
} // namespace OpenRCT2::Golf
