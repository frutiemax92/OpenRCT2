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
    // The clock of the golf course. Golf is played in minutes (a tee slot of 12 minutes, 30 seconds of searching for a
    // ball), so the golf game clock runs on its own scale next to the OpenRCT2 calendar: one game minute is 200 ticks,
    // 5 seconds at normal speed. A round of 18 holes of about 12 minutes then takes an hour of real time, and the opening
    // hours 06:00 to 20:00 (GLF-005) a bit under four hours.
    constexpr int32_t kTicksPerGameMinute = 200;
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
