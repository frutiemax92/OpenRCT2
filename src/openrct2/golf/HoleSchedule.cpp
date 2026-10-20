/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "HoleSchedule.h"

#include <algorithm>

namespace OpenRCT2::Golf
{
    namespace
    {
        // Integer division rounded to the nearest, halves away from zero.
        int32_t roundedDivide(int32_t value, int32_t divisor)
        {
            return value >= 0 ? (value + divisor / 2) / divisor : -((-value + divisor / 2) / divisor);
        }
    } // namespace

    int32_t estimateHoleMinutes(int32_t lengthYards, int32_t par, int32_t difficultyTenths)
    {
        return (estimateHoleTenths(lengthYards, par, difficultyTenths) + 9) / 10; // whole minutes, rounded up
    }

    int32_t estimateHoleTenths(int32_t lengthYards, int32_t par, int32_t difficultyTenths)
    {
        if (lengthYards <= 0 || par <= 0)
            return kMinSlotMinutes * 10;
        // Walking: the playing line and the way to the next tee, rounded up to whole tenths.
        const int32_t walkTenths = ((lengthYards + kTransitionYards) * 10 + kWalkYardsPerMinute - 1) / kWalkYardsPerMinute;
        // Strokes: par plus what an average golfer loses, more on hard holes. Skill 0 loses 2.0 strokes, skill 100 none,
        // and a hole of difficulty 10 adds half a stroke.
        const int32_t overParTenths = kOverParAtSkillZeroTenths * (100 - kAverageSkill) / 100 + difficultyTenths / 20;
        const int32_t strokesTenths = par * 10 + overParTenths;
        const int32_t strokeMinutesTenths = strokesTenths * kMinutesPerStrokeTenths / 10;
        return walkTenths + strokeMinutesTenths + kGreenMinutesTenths;
    }

    int32_t slotMinutes(const Hole& hole, int32_t difficultyTenths)
    {
        const auto& schedule = hole.schedule;
        if (schedule.manual)
            return std::clamp(schedule.manualSlotMinutes, kMinSlotMinutes, kMaxSlotMinutes);
        const auto tee = hole.firstTee();
        if (!tee.has_value() || !hole.cup.has_value())
            return kDefaultManualSlotMinutes;
        const int32_t estimate = estimateHoleMinutes(hole.lengthYards(*tee), hole.par(), difficultyTenths);
        int32_t result = estimate;
        if (schedule.samples >= kBlendedSamples)
        {
            // Half estimate, half what the groups really take, with the wait not counted: a queue is not the hole's fault.
            result = (estimate * 10 + schedule.measuredPlayTenths + 10) / 20;
        }
        return std::clamp(result, kMinSlotMinutes, kMaxSlotMinutes);
    }

    void recordPlay(HoleSchedule& schedule, int32_t playMinutes, int32_t waitedMinutes)
    {
        const int32_t play = std::max(0, playMinutes) * 10;
        const int32_t wait = std::max(0, waitedMinutes) * 10;
        // A running mean for the first ten groups, then a moving average that gives each new group a tenth.
        const int32_t weight = std::min(schedule.samples + 1, 10);
        if (schedule.samples == 0)
        {
            schedule.measuredPlayTenths = play;
            schedule.measuredWaitTenths = wait;
        }
        else
        {
            schedule.measuredPlayTenths += roundedDivide(play - schedule.measuredPlayTenths, weight);
            schedule.measuredWaitTenths += roundedDivide(wait - schedule.measuredWaitTenths, weight);
        }
        schedule.samples++;
    }

    bool isBottleneck(const Hole& hole, int32_t difficultyTenths)
    {
        const auto& schedule = hole.schedule;
        if (schedule.samples < kBlendedSamples)
            return false;
        const int32_t slot = slotMinutes(hole, difficultyTenths);
        return schedule.measuredPlayTenths * 100 > slot * 10 * kBottleneckPercent;
    }

    TeeSheet::TeeSheet(size_t holeCount)
    {
        reset(holeCount);
    }

    void TeeSheet::reset(size_t holeCount)
    {
        _lastTeeOff.assign(holeCount, 0);
        _played.assign(holeCount, false);
    }

    uint32_t TeeSheet::earliestTeeOff(size_t hole, uint32_t arrival, int32_t slot) const
    {
        if (hole >= _lastTeeOff.size() || !_played[hole])
            return arrival;
        return std::max(arrival, _lastTeeOff[hole] + static_cast<uint32_t>(std::max(1, slot)));
    }

    TeeSheet::Booking TeeSheet::bookTeeOff(size_t hole, uint32_t arrival, int32_t slot)
    {
        Booking booking;
        booking.minute = earliestTeeOff(hole, arrival, slot);
        booking.waited = booking.minute - arrival;
        if (hole < _lastTeeOff.size())
        {
            _lastTeeOff[hole] = booking.minute;
            _played[hole] = true;
        }
        return booking;
    }

    TeeSheet::Booking TeeSheet::bookFirstTee(uint32_t now, int32_t slot)
    {
        return bookTeeOff(0, now, slot);
    }
} // namespace OpenRCT2::Golf
