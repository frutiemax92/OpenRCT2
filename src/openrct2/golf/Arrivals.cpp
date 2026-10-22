/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Arrivals.h"

#include <algorithm>

namespace OpenRCT2::Golf
{
    bool isOpenAt(const OpeningHours& hours, int32_t minuteOfDay)
    {
        return minuteOfDay >= hours.open && minuteOfDay < hours.close;
    }

    bool acceptsArrivalsAt(const OpeningHours& hours, int32_t minuteOfDay)
    {
        return minuteOfDay >= hours.open && minuteOfDay < hours.close - hours.lastArrivalBeforeClose;
    }

    bool mustHaveLeft(const OpeningHours& hours, int32_t minuteOfDay)
    {
        return !isOpenAt(hours, minuteOfDay) && minuteOfDay >= hours.close;
    }

    int32_t priceFairnessPercent(int64_t greenFee, int32_t holes)
    {
        const int64_t fair = static_cast<int64_t>(kFairFeePerHole) * std::max(1, holes);
        if (greenFee <= 0)
            return 150;
        // At the fair price 100; a free round is 150 (capped); three times the fair price is 0.
        const int64_t ratio = greenFee * 100 / fair; // percent of the fair price
        if (ratio <= 100)
            return static_cast<int32_t>(std::min<int64_t>(150, 100 + (100 - ratio) / 2));
        return static_cast<int32_t>(std::clamp<int64_t>(100 - (ratio - 100) / 2, 0, 100));
    }

    int32_t timeOfDayPercent(int32_t minuteOfDay)
    {
        // A trapezoid: 40% at opening, 100% from 08:00 to 14:00, back to 40% by closing.
        const int32_t m = std::clamp(minuteOfDay, 0, kMinutesPerDay - 1);
        if (m < hhmm(8, 0))
            return 40 + 60 * std::max(0, m - hhmm(6, 0)) / hhmm(2, 0);
        if (m <= hhmm(14, 0))
            return 100;
        return std::max(40, 100 - 60 * (m - hhmm(14, 0)) / hhmm(6, 0));
    }

    int32_t weatherPercent(Weather::Type weather)
    {
        switch (weather)
        {
            case Weather::Type::sunny:
                return 100;
            case Weather::Type::partiallyCloudy:
            case Weather::Type::cloudy:
                return 90;
            case Weather::Type::rain:
            case Weather::Type::snow:
                return kRainArrivalPercent;
            default:
                return kHeavyWeatherArrivalPercent;
        }
    }

    int32_t arrivalChancePerMillion(const ArrivalContext& c)
    {
        if (!c.courseOpen || c.holes <= 0 || c.golfersOnCourse >= c.maxGolfers || !acceptsArrivalsAt(c.hours, c.minuteOfDay))
            return 0;
        const int64_t popularity = std::clamp(c.popularity, 0, kPopularityMax);
        // groups per minute = base / 1000 * popularity/1000 * time * weather * price, all as percentages.
        int64_t perMillion = static_cast<int64_t>(kBaseArrivalsPer1000Minutes) * 1000; // per million minutes at full popularity
        perMillion = perMillion * popularity / kPopularityMax;
        perMillion = perMillion * timeOfDayPercent(c.minuteOfDay) / 100;
        perMillion = perMillion * weatherPercent(c.weather) / 100;
        perMillion = perMillion * priceFairnessPercent(c.greenFee, c.holes) / 100;
        return static_cast<int32_t>(perMillion);
    }

    bool groupArrives(const ArrivalContext& context, IBallRandom& rng)
    {
        const int32_t chance = arrivalChancePerMillion(context);
        if (chance <= 0)
            return false;
        return static_cast<int32_t>(rng.next() % 1000000) < chance;
    }

    int32_t rollGroupSize(IBallRandom& rng, int32_t spaceLeft)
    {
        // Singles 15%, pairs 35%, threes 15%, fours 35%.
        const uint32_t roll = rng.next() % 100;
        int32_t size = roll < 15 ? 1 : (roll < 50 ? 2 : (roll < 65 ? 3 : 4));
        return std::clamp(std::min(size, spaceLeft), 1, kMaxGroupSize);
    }

    NewGolferTemplate rollNewGolfer(IBallRandom& rng, int32_t popularity, bool vip)
    {
        NewGolferTemplate t;
        auto skill = [&]() {
            // Around 50, spread of about +-25 (sum of two rolls).
            const int32_t a = static_cast<int32_t>(rng.next() % 26), b = static_cast<int32_t>(rng.next() % 26);
            return static_cast<uint8_t>(std::clamp(25 + a + b, 0, 100));
        };
        t.skills = { skill(), skill(), skill(), skill() };
        t.vip = vip;
        // Equipment: better for richer golfers.
        const int32_t equipmentRoll = static_cast<int32_t>(rng.next() % 100);
        t.equipmentLevel = static_cast<uint8_t>(vip ? 2 + equipmentRoll / 60 : equipmentRoll / 40);
        t.equipmentLevel = std::min<uint8_t>(t.equipmentLevel, 3);
        // GLF-003: money in a range set by the reputation; VIPs have much more.
        const int64_t base = 6000 + static_cast<int64_t>(std::clamp(popularity, 0, kPopularityMax)) * 8;
        const int64_t spread = static_cast<int64_t>(rng.next() % 6000);
        t.money = (base + spread) * (vip ? 5 : 1);
        t.riskTolerance = Fixed::ratio(static_cast<int64_t>(rng.next() % 101), 100);
        return t;
    }

    std::string rollName(IBallRandom& rng, const std::vector<std::string>& firstNames, const std::vector<std::string>& lastNames, uint32_t fallbackNumber)
    {
        if (firstNames.empty() && lastNames.empty())
            return "Golfer " + std::to_string(fallbackNumber);
        std::string name;
        if (!firstNames.empty())
            name = firstNames[rng.next() % firstNames.size()];
        if (!lastNames.empty())
        {
            if (!name.empty())
                name += ' ';
            name += lastNames[rng.next() % lastNames.size()];
        }
        return name;
    }

    std::string rollUniqueName(
        IBallRandom& rng, const std::vector<std::string>& firstNames, const std::vector<std::string>& lastNames,
        uint32_t fallbackNumber, const std::function<bool(const std::string&)>& isTaken)
    {
        constexpr int32_t kRandomTries = 32;
        for (int32_t i = 0; i < kRandomTries; i++)
        {
            auto name = rollName(rng, firstNames, lastNames, fallbackNumber);
            if (!isTaken(name))
                return name;
        }
        // Crowded: every combination in turn from a random place, so a free one is found if there is any.
        const size_t firsts = std::max<size_t>(firstNames.size(), 1);
        const size_t lasts = std::max<size_t>(lastNames.size(), 1);
        const size_t total = firsts * lasts;
        const size_t start = rng.next() % total;
        std::string base;
        for (size_t i = 0; i < total; i++)
        {
            const size_t k = (start + i) % total;
            std::string name;
            if (!firstNames.empty())
                name = firstNames[k / lasts];
            if (!lastNames.empty())
            {
                if (!name.empty())
                    name += ' ';
                name += lastNames[k % lasts];
            }
            if (name.empty())
                name = "Golfer " + std::to_string(fallbackNumber);
            if (base.empty())
                base = name;
            if (!isTaken(name))
                return name;
        }
        for (uint32_t n = 2;; n++)
        {
            auto name = base + ' ' + std::to_string(n);
            if (!isTaken(name))
                return name;
        }
    }
} // namespace OpenRCT2::Golf
