/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <gtest/gtest.h>
#include <openrct2/golf/HoleSchedule.h>

using namespace OpenRCT2::Golf;

namespace
{
    Hole holeOfYards(int yards)
    {
        Hole h;
        h.id = 1;
        h.tees[0] = HolePoint{ 320, 3200 };
        h.cup = HolePoint{ 320 + yards * 32 / 5, 3200 };
        return h;
    }
} // namespace

TEST(GameClock, OneGameMinuteIsTwoHundredTicks)
{
    EXPECT_EQ(kTicksPerGameMinute, 200);
    EXPECT_EQ(gameMinuteAt(0), 0u);
    EXPECT_EQ(gameMinuteAt(199), 0u);
    EXPECT_EQ(gameMinuteAt(200), 1u);
    EXPECT_EQ(minuteOfDayAt(static_cast<uint32_t>(kTicksPerGameDay) + 600), 3);
    EXPECT_EQ(gameDayAt(static_cast<uint32_t>(kTicksPerGameDay) * 3 + 5), 3u);
    EXPECT_EQ(hhmm(6, 30), 390);
    EXPECT_EQ(kMinutesPerDay, 1440);
}

TEST(HoleSchedule, TheAutoEstimateForATypicalHoleIsTenToFifteenMinutes)
{
    // Acceptance criterion: a plausible slot in Auto mode (10-15 game minutes) for a normal hole.
    const auto four = holeOfYards(400);
    const int32_t minutes = slotMinutes(four);
    EXPECT_GE(minutes, 10);
    EXPECT_LE(minutes, 15);
    const auto three = holeOfYards(180);
    EXPECT_GE(slotMinutes(three), 7);
    EXPECT_LT(slotMinutes(three), slotMinutes(four));
    const auto five = holeOfYards(540);
    EXPECT_GT(slotMinutes(five), slotMinutes(four));
    EXPECT_LE(slotMinutes(five), 17);
}

TEST(HoleSchedule, LongerAndHarderHolesTakeLonger)
{
    EXPECT_GT(estimateHoleMinutes(450, 4, 0), estimateHoleMinutes(300, 4, 0));
    EXPECT_GT(estimateHoleTenths(400, 4, 100), estimateHoleTenths(400, 4, 0));
    EXPECT_GE(estimateHoleMinutes(400, 4, 100), estimateHoleMinutes(400, 4, 0));
    EXPECT_GT(estimateHoleMinutes(400, 5, 0), estimateHoleMinutes(400, 4, 0));
    EXPECT_EQ(estimateHoleMinutes(0, 4, 0), kMinSlotMinutes);
    EXPECT_EQ(estimateHoleMinutes(400, 0, 0), kMinSlotMinutes);
}

TEST(HoleSchedule, TheEstimateIsAWholeNumberOfMinutesAndSumsItsParts)
{
    // 400 yards: walk (400 + 60) / 90 = 5.2 min, strokes (4 + 1.0) x 1.0 = 5.0, green 1.0 -> 11.2, rounded up to 12.
    EXPECT_EQ(estimateHoleMinutes(400, 4, 0), 12);
}

TEST(HoleSchedule, ManualSlotsAreUsedAsTheyAreWithinLimits)
{
    auto hole = holeOfYards(400);
    hole.schedule.manual = true;
    hole.schedule.manualSlotMinutes = 9;
    EXPECT_EQ(slotMinutes(hole), 9);
    hole.schedule.manualSlotMinutes = 100;
    EXPECT_EQ(slotMinutes(hole), kMaxSlotMinutes);
    hole.schedule.manualSlotMinutes = 1;
    EXPECT_EQ(slotMinutes(hole), kMinSlotMinutes);
}

TEST(HoleSchedule, AnIncompleteHoleUsesTheDefaultSlot)
{
    Hole hole;
    EXPECT_EQ(slotMinutes(hole), kDefaultManualSlotMinutes);
}

TEST(HoleSchedule, MeasuredPlaysPullTheAutoSlotTowardsReality)
{
    auto hole = holeOfYards(400);
    const int32_t estimate = slotMinutes(hole);
    // Groups take 20 minutes, much more than the estimate.
    recordPlay(hole.schedule, 20, 0);
    recordPlay(hole.schedule, 20, 0);
    EXPECT_EQ(slotMinutes(hole), estimate); // too few groups yet
    recordPlay(hole.schedule, 20, 0);
    const int32_t blended = slotMinutes(hole);
    EXPECT_GT(blended, estimate);
    EXPECT_LT(blended, 20);
    EXPECT_EQ(hole.schedule.samples, 3);
    // Manual mode ignores the measurements.
    hole.schedule.manual = true;
    hole.schedule.manualSlotMinutes = 10;
    EXPECT_EQ(slotMinutes(hole), 10);
}

TEST(HoleSchedule, MeasurementsAreMovingAveragesAndWaitIsTrackedSeparately)
{
    HoleSchedule s;
    recordPlay(s, 10, 2);
    EXPECT_EQ(s.measuredPlayTenths, 100);
    EXPECT_EQ(s.measuredWaitTenths, 20);
    recordPlay(s, 14, 0);
    EXPECT_EQ(s.measuredPlayTenths, 120);
    EXPECT_EQ(s.measuredWaitTenths, 10);
    for (int i = 0; i < 100; i++)
        recordPlay(s, 20, 5);
    EXPECT_NEAR(s.measuredPlayTenths, 200, 5); // the moving average stops within half a weight of the target
    EXPECT_NEAR(s.measuredWaitTenths, 50, 5);
    recordPlay(s, -5, -5); // nonsense is treated as zero
    EXPECT_GE(s.measuredPlayTenths, 0);
}

TEST(HoleSchedule, SlowHolesAreBottlenecks)
{
    auto hole = holeOfYards(400);
    EXPECT_FALSE(isBottleneck(hole)); // no measurements
    for (int i = 0; i < 5; i++)
        recordPlay(hole.schedule, 12, 0);
    EXPECT_FALSE(isBottleneck(hole));
    hole.schedule.manual = true;
    hole.schedule.manualSlotMinutes = 8; // the groups need 12 minutes but the slot gives 8
    EXPECT_TRUE(isBottleneck(hole));
    hole.schedule.manualSlotMinutes = 12;
    EXPECT_FALSE(isBottleneck(hole));
    hole.schedule.manualSlotMinutes = 11; // 12 against 11 is within the tolerance of 110%
    EXPECT_FALSE(isBottleneck(hole));
    hole.schedule.manualSlotMinutes = 10;
    EXPECT_TRUE(isBottleneck(hole));
}

TEST(TeeSheet, TheFirstGroupTeesOffAtOnce)
{
    TeeSheet sheet(3);
    const auto booking = sheet.bookFirstTee(100, 12);
    EXPECT_EQ(booking.minute, 100u);
    EXPECT_EQ(booking.waited, 0u);
    EXPECT_TRUE(sheet.hasPlayed(0));
    EXPECT_FALSE(sheet.hasPlayed(1));
}

TEST(TeeSheet, TwoGroupsNeverTeeOffAtTheSameTime)
{
    TeeSheet sheet(1);
    const auto first = sheet.bookFirstTee(100, 10);
    const auto second = sheet.bookFirstTee(100, 10);
    const auto third = sheet.bookFirstTee(101, 10);
    EXPECT_EQ(first.minute, 100u);
    EXPECT_EQ(second.minute, 110u);
    EXPECT_EQ(second.waited, 10u);
    EXPECT_EQ(third.minute, 120u);
    EXPECT_EQ(third.waited, 19u);
    EXPECT_GE(second.minute - first.minute, 10u);
    EXPECT_GE(third.minute - second.minute, 10u);
}

TEST(TeeSheet, AGroupThatArrivesLateDoesNotWait)
{
    TeeSheet sheet(1);
    sheet.bookFirstTee(100, 10);
    const auto late = sheet.bookFirstTee(500, 10);
    EXPECT_EQ(late.minute, 500u);
    EXPECT_EQ(late.waited, 0u);
}

TEST(TeeSheet, EveryHoleHasItsOwnQueue)
{
    TeeSheet sheet(3);
    EXPECT_EQ(sheet.bookTeeOff(0, 100, 10).minute, 100u);
    EXPECT_EQ(sheet.bookTeeOff(1, 100, 12).minute, 100u);
    EXPECT_EQ(sheet.bookTeeOff(0, 101, 10).minute, 110u);
    EXPECT_EQ(sheet.bookTeeOff(1, 105, 12).minute, 112u);
    EXPECT_EQ(sheet.earliestTeeOff(2, 50, 8), 50u);
    // Asking is not booking.
    EXPECT_EQ(sheet.earliestTeeOff(0, 105, 10), 120u);
    EXPECT_EQ(sheet.earliestTeeOff(0, 105, 10), 120u);
}

TEST(TeeSheet, SlotsChangeBetweenBookings)
{
    TeeSheet sheet(1);
    sheet.bookFirstTee(100, 10);
    // The player shortens the slot: the next group is spaced with the new value from the last tee-off.
    EXPECT_EQ(sheet.bookFirstTee(100, 5).minute, 105u);
    EXPECT_EQ(sheet.bookFirstTee(100, 0).minute, 106u); // at least one minute apart
}

TEST(TeeSheet, OutOfRangeHolesAreHarmless)
{
    TeeSheet sheet(2);
    EXPECT_EQ(sheet.bookTeeOff(9, 100, 10).minute, 100u);
    EXPECT_EQ(sheet.earliestTeeOff(9, 100, 10), 100u);
    EXPECT_FALSE(sheet.hasPlayed(9));
    sheet.reset(4);
    EXPECT_EQ(sheet.holeCount(), 4u);
}

TEST(TeeSheet, TheNextHoleIsExpectedOneSlotLater)
{
    EXPECT_EQ(TeeSheet::expectedNextTee(130, 12), 142u);
}

TEST(HoleSchedule, AGroupOfFoursomesFlowsThroughTheCourse)
{
    // Acceptance criterion: groups follow each other through a nine hole course without teeing off together.
    Course course;
    for (uint32_t i = 0; i < 9; i++)
        course.holes.push_back(holeOfYards(300 + static_cast<int>(i) * 20));
    TeeSheet sheet(course.holes.size());
    std::vector<std::vector<uint32_t>> teeOffs(course.holes.size());
    uint32_t arrival = 360;
    for (int group = 0; group < 6; group++)
    {
        uint32_t clock = sheet.bookFirstTee(arrival, slotMinutes(course.holes[0])).minute;
        teeOffs[0].push_back(clock);
        for (size_t h = 1; h < course.holes.size(); h++)
        {
            const auto prevSlot = slotMinutes(course.holes[h - 1]);
            const auto booking = sheet.bookTeeOff(h, TeeSheet::expectedNextTee(clock, prevSlot), slotMinutes(course.holes[h]));
            clock = booking.minute;
            teeOffs[h].push_back(clock);
        }
        arrival += 4; // groups check in every four minutes, faster than the slots allow
    }
    for (size_t h = 0; h < teeOffs.size(); h++)
        for (size_t g = 1; g < teeOffs[h].size(); g++)
            EXPECT_GE(teeOffs[h][g] - teeOffs[h][g - 1], static_cast<uint32_t>(slotMinutes(course.holes[h]))) << h << " " << g;
}
