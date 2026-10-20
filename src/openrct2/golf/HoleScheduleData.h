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
    // Hole scheduling (HOL-020 to HOL-024): the time slot of a hole is the minutes of game time between two groups
    // teeing off on it. In Auto mode it is estimated from the hole and refined from what groups really take; in Manual
    // mode the player sets it. The data lives with the hole; the logic is in HoleSchedule.h.

    constexpr int32_t kMinSlotMinutes = 4;
    constexpr int32_t kMaxSlotMinutes = 30;
    constexpr int32_t kDefaultManualSlotMinutes = 10;

    // Documented constants of the Auto estimate (HOL-021).
    constexpr int32_t kWalkYardsPerMinute = 90;        // a walking pace of about 5 km/h
    constexpr int32_t kTransitionYards = 60;           // from the green to the next tee
    constexpr int32_t kMinutesPerStrokeTenths = 10;    // 1.0 minute per stroke, for the routine and the swing
    constexpr int32_t kGreenMinutesTenths = 10;        // reading and marking on the green
    constexpr int32_t kAverageSkill = 50;              // "average skill" for the expected strokes
    constexpr int32_t kOverParAtSkillZeroTenths = 20;  // expected strokes over par for a golfer of skill 0
    constexpr int32_t kBlendedSamples = 3;             // measured plays needed before they count in Auto mode
    constexpr int32_t kBottleneckPercent = 110;        // measured time over this share of the slot is a bottleneck

    struct HoleSchedule
    {
        bool manual = false;
        int32_t manualSlotMinutes = kDefaultManualSlotMinutes;

        // Measured by groups playing the hole: moving averages in tenths of a minute, and the number of groups counted.
        int32_t measuredPlayTenths = 0;
        int32_t measuredWaitTenths = 0;
        int32_t samples = 0;
    };
} // namespace OpenRCT2::Golf
