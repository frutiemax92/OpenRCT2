/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "BallPhysics.h"
#include "GameClock.h"
#include "Golfer.h"
#include "Popularity.h"
#include "GolferSkills.h"
#include "Shot.h"
#include "../world/Weather.h"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace OpenRCT2::Golf
{
    // How golfers arrive (GLF-001 to GLF-005): in groups of one to four, at a rate driven by popularity, the green fee,
    // the time of day and the weather, within the opening hours of the course.

    // GLF-005: the opening hours, in minutes since midnight. Golfers stop arriving an hour before closing and have left
    // by closing time.
    struct OpeningHours
    {
        int32_t open = hhmm(6, 0);
        int32_t close = hhmm(20, 0);
        int32_t lastArrivalBeforeClose = kMinutesPerHour;
    };

    bool isOpenAt(const OpeningHours& hours, int32_t minuteOfDay);
    bool acceptsArrivalsAt(const OpeningHours& hours, int32_t minuteOfDay);
    // GLF-005: every golfer must be gone from the course at closing time.
    bool mustHaveLeft(const OpeningHours& hours, int32_t minuteOfDay);

    constexpr int32_t kDefaultMaxGolfers = 500; // GLF-004
    constexpr int32_t kMaxGroupSize = 4;        // Q6: groups of 1-4

    // Parameters of the arrival rate, all in one place (GLF-004).
    constexpr int32_t kBaseArrivalsPer1000Minutes = 220;   // groups per 1000 game minutes at popularity 1000 and a fair price
    constexpr int32_t kFairFeePerHole = 250;                // currency units per hole that golfers find fair
    constexpr int32_t kRainArrivalPercent = 50;
    constexpr int32_t kHeavyWeatherArrivalPercent = 20;
    // Share of arriving groups that are returning golfers when the registry has some.
    constexpr int32_t kReturningGolferPercent = 40;

    struct ArrivalContext
    {
        int32_t popularity = 500;      // 0..1000 (ECO-001)
        int64_t greenFee = 2500;
        int32_t holes = 9;             // playable holes of the course
        Weather::Type weather = Weather::Type::sunny;
        int32_t minuteOfDay = hhmm(9, 0);
        int32_t golfersOnCourse = 0;
        int32_t maxGolfers = kDefaultMaxGolfers;
        OpeningHours hours;
        bool courseOpen = true;
    };

    // How fair a green fee is for a number of holes: 100 at the fair price, falling to 0 at three times the price and
    // rising (capped at 150) for bargains. Used for arrivals and for the thought "prices too high".
    int32_t priceFairnessPercent(int64_t greenFee, int32_t holes);
    // Popularity of golfers around the clock: busiest in the morning and early afternoon, quiet at the edges.
    int32_t timeOfDayPercent(int32_t minuteOfDay);
    int32_t weatherPercent(Weather::Type weather);

    // Chance, per game minute and out of 1,000,000, that a new group arrives. Zero outside the arrival hours, with the
    // course closed, or when the course is full.
    int32_t arrivalChancePerMillion(const ArrivalContext& context);
    // Rolls the chance with the given random number.
    bool groupArrives(const ArrivalContext& context, IBallRandom& rng);

    // GLF-001: size of an arriving group, mostly twos and fours.
    int32_t rollGroupSize(IBallRandom& rng, int32_t spaceLeft = kMaxGroupSize);

    // GLF-003: a golfer's attributes at spawn.
    struct NewGolferTemplate
    {
        GolferSkills skills;
        uint8_t equipmentLevel = 0;
        int64_t money = 0;
        bool vip = false;
        Fixed riskTolerance = Fixed::ratio(5, 10);
    };
    // Money is random within a range that grows with popularity; VIPs are richer. Skills are around 50 with a spread, with
    // better equipment lifting them a little (GLF-050).
    NewGolferTemplate rollNewGolfer(IBallRandom& rng, int32_t popularity, bool vip);
    // A name from the lists in `firstNames` and `lastNames` (peep_names object); falls back to a numbered golfer.
    std::string rollName(IBallRandom& rng, const std::vector<std::string>& firstNames, const std::vector<std::string>& lastNames, uint32_t fallbackNumber);
    // Like rollName, but never a name for which `isTaken` is true, so no two golfers share a name. It draws random
    // combinations first; when those keep hitting taken names it walks through every combination from a random start. With
    // every combination taken it numbers the name ("Ann Birdie 2").
    std::string rollUniqueName(
        IBallRandom& rng, const std::vector<std::string>& firstNames, const std::vector<std::string>& lastNames,
        uint32_t fallbackNumber, const std::function<bool(const std::string&)>& isTaken);
} // namespace OpenRCT2::Golf
