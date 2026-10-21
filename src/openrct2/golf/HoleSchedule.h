/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "GameClock.h"
#include "Hole.h"
#include "HoleScheduleData.h"

#include <cstdint>
#include <vector>

namespace OpenRCT2::Golf
{
    // Hole scheduling (HOL-020 to HOL-024): the time slot of a hole is the minutes of game time between two groups
    // teeing off on it. In Auto mode it is estimated from the hole and refined from what groups really take; in Manual
    // mode the player sets it.

    // HOL-021: the estimated minutes for a group to play a hole of this length and par: walking along the line and to
    // the next tee, time per expected stroke for a golfer of average skill (par plus the expected strokes over par,
    // which grow with the difficulty in tenths, 0..100), and time on the green.
    int32_t estimateHoleMinutes(int32_t lengthYards, int32_t par, int32_t difficultyTenths);

    // The same in tenths of a minute, before rounding up.
    int32_t estimateHoleTenths(int32_t lengthYards, int32_t par, int32_t difficultyTenths);

    // The slot of a hole in minutes: the manual value, or the estimate blended with measured plays once there are enough
    // (HOL-020, HOL-021). Always within kMinSlotMinutes..kMaxSlotMinutes.
    int32_t slotMinutes(const Hole& hole, int32_t difficultyTenths = 0);

    // Adds one group's measured play of the hole (HOL-021, HOL-023).
    void recordPlay(HoleSchedule& schedule, int32_t playMinutes, int32_t waitedMinutes);

    // HOL-024: the groups are slower than the slot allows, so the next ones will queue.
    bool isBottleneck(const Hole& hole, int32_t difficultyTenths = 0);

    // The tee sheet of a course: when each hole is free for the next group to tee off. Holes are indexed by their
    // position in the course. All times are game minutes.
    class TeeSheet
    {
    public:
        explicit TeeSheet(size_t holeCount = 0);

        void reset(size_t holeCount);
        size_t holeCount() const
        {
            return _lastTeeOff.size();
        }

        // The earliest minute a group arriving at `arrival` may tee off on a hole: not before the previous group's
        // tee-off plus the slot of the hole (HOL-022). Two groups never get the same minute.
        uint32_t earliestTeeOff(size_t hole, uint32_t arrival, int32_t slot) const;
        // Books the tee-off; returns the minute booked and how long the group waits (HOL-023).
        struct Booking
        {
            uint32_t minute = 0;
            uint32_t waited = 0;
        };
        Booking bookTeeOff(size_t hole, uint32_t arrival, int32_t slot);
        // Tee time on hole 1 for a group that checks in at `now`: the first free slot that is not in the past.
        Booking bookFirstTee(uint32_t now, int32_t slot);

        // When a group that teed off on hole `hole` at `teeOff` is expected on the next hole (HOL-022).
        static uint32_t expectedNextTee(uint32_t teeOff, int32_t slot)
        {
            return teeOff + static_cast<uint32_t>(slot);
        }

        bool hasPlayed(size_t hole) const
        {
            return hole < _played.size() && _played[hole];
        }

    private:
        friend struct GolfSaveAccess;
        std::vector<uint32_t> _lastTeeOff;
        std::vector<bool> _played;
    };
} // namespace OpenRCT2::Golf
