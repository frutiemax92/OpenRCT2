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
    // The clock of the golf course runs in real time: a game minute is 60 seconds of the 40 ticks a second, 2400 ticks, so
    // a tile of 5 yards takes a golfer 3.33 seconds to cross and a tee slot of 12 minutes is 12 minutes at normal speed.
    // The game speed buttons fast-forward it.
    constexpr int32_t kTicksPerGameMinute = 2400;
    constexpr int32_t kMinutesPerHour = 60;
    constexpr int32_t kMinutesPerDay = 24 * kMinutesPerHour;
    constexpr int32_t kTicksPerGameDay = kTicksPerGameMinute * kMinutesPerDay;

    // The game minute since the start of the game for a tick count.
    constexpr uint32_t gameMinuteAt(uint32_t ticks)
    {
        return ticks / static_cast<uint32_t>(kTicksPerGameMinute);
    }
    constexpr int32_t minuteOfDayAt(uint32_t ticks)
    {
        return static_cast<int32_t>(gameMinuteAt(ticks) % static_cast<uint32_t>(kMinutesPerDay));
    }
    constexpr uint32_t gameDayAt(uint32_t ticks)
    {
        return gameMinuteAt(ticks) / static_cast<uint32_t>(kMinutesPerDay);
    }
    constexpr int32_t hhmm(int32_t hours, int32_t minutes)
    {
        return hours * kMinutesPerHour + minutes;
    }
} // namespace OpenRCT2::Golf
