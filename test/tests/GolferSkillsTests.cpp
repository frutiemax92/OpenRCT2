/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "GolfTestWorld.h"

#include <gtest/gtest.h>
#include <map>
#include <openrct2/golf/GolferSkills.h>

using namespace OpenRCT2::Golf;
using GolfTest::SeqRandom;

TEST(GolferSkills, ConvertToAndFromBytes)
{
    const GolferSkills s{ 10, 20, 30, 40 };
    const auto points = SkillPoints::from(s);
    EXPECT_EQ(points.longPlay, 10 * 256);
    const auto back = points.toSkills();
    EXPECT_EQ(back.longPlay, 10);
    EXPECT_EQ(back.accuracy, 20);
    EXPECT_EQ(back.foolproof, 30);
    EXPECT_EQ(back.physique, 40);
    SkillPoints over;
    over.longPlay = 150 * 256;
    EXPECT_EQ(over.toSkills().longPlay, 100);
}

TEST(GolferSkills, TheRangeTrainsLongPlayOnly)
{
    SkillPoints s;
    gainSkills(s, Activity::drivingRangeBall, 100);
    EXPECT_GT(s.longPlay, 50 * 256 + 20 * 256); // 100 balls give well over 20 points at the start
    EXPECT_EQ(s.accuracy, 50 * 256);
    EXPECT_EQ(s.foolproof, 50 * 256);
    EXPECT_EQ(s.physique, 50 * 256);
}

TEST(GolferSkills, MiniPuttTrainsAccuracyOnly)
{
    SkillPoints s;
    gainSkills(s, Activity::miniPuttHole, 18);
    EXPECT_GT(s.accuracy, 50 * 256 + 8 * 256);
    EXPECT_EQ(s.longPlay, 50 * 256);
}

TEST(GolferSkills, ACourseHoleTrainsAllFourSkills)
{
    SkillPoints s;
    gainSkills(s, Activity::courseHole, 18);
    EXPECT_GT(s.longPlay, 50 * 256);
    EXPECT_GT(s.accuracy, 50 * 256);
    EXPECT_GT(s.foolproof, 50 * 256);
    EXPECT_GT(s.physique, 50 * 256);
    // Physique grows the slowest.
    EXPECT_LT(s.physique - 50 * 256, s.accuracy - 50 * 256);
}

TEST(GolferSkills, ReturnsDiminishAndNeverPassHundred)
{
    SkillPoints beginner{ 10 * 256, 0, 0, 0 };
    SkillPoints expert{ 95 * 256, 0, 0, 0 };
    gainSkills(beginner, Activity::drivingRangeBall, 10);
    gainSkills(expert, Activity::drivingRangeBall, 10);
    EXPECT_GT(beginner.longPlay - 10 * 256, (expert.longPlay - 95 * 256) * 5);
    SkillPoints maxed{ 100 * 256, 0, 0, 0 };
    gainSkills(maxed, Activity::drivingRangeBall, 1000);
    EXPECT_EQ(maxed.longPlay, 100 * 256);
    SkillPoints nearly{ 99 * 256 + 250, 0, 0, 0 };
    gainSkills(nearly, Activity::drivingRangeBall, 100000);
    EXPECT_LE(nearly.longPlay, 100 * 256);
    gainSkills(beginner, Activity::drivingRangeBall, 0);
    gainSkills(beginner, Activity::drivingRangeBall, -4);
}

TEST(GolferSkills, SkillsDecayEachDayAwayButNotBelowTheFloor)
{
    SkillPoints initial{ 40 * 256, 40 * 256, 40 * 256, 40 * 256 };
    SkillPoints s{ 70 * 256, 70 * 256, 45 * 256, 30 * 256 };
    decaySkills(s, initial, 10);
    EXPECT_EQ(s.longPlay, 70 * 256 - 10 * kDecayPerDay256);
    // Foolproof decays towards initial - 10 = 30.
    EXPECT_EQ(s.foolproof, 45 * 256 - 10 * kDecayPerDay256);
    // Physique is already at the floor.
    EXPECT_EQ(s.physique, 30 * 256);
    decaySkills(s, initial, 100000);
    EXPECT_EQ(s.longPlay, 30 * 256);
    EXPECT_EQ(s.accuracy, 30 * 256);
    EXPECT_EQ(s.foolproof, 30 * 256);
    decaySkills(s, initial, 0);
    EXPECT_EQ(s.longPlay, 30 * 256);
}

TEST(GolferSkills, TheFloorNeverGoesNegative)
{
    SkillPoints initial{ 3 * 256, 0, 0, 0 };
    SkillPoints s{ 5 * 256, 0, 0, 0 };
    decaySkills(s, initial, 100000);
    EXPECT_EQ(s.longPlay, 0);
}

namespace
{
    RegistryEntry entry(uint32_t id, uint8_t happiness = 128, uint32_t lastSeen = 0)
    {
        RegistryEntry e;
        e.id = id;
        e.name = "Golfer " + std::to_string(id);
        e.lastHappiness = happiness;
        e.lastSeenDay = lastSeen;
        return e;
    }
} // namespace

TEST(GolferRegistry, RemembersAndFindsGolfers)
{
    GolferRegistry registry;
    EXPECT_EQ(registry.capacity(), 2000u);
    registry.remember(entry(7));
    registry.remember(entry(8));
    EXPECT_EQ(registry.size(), 2u);
    ASSERT_NE(registry.find(7), nullptr);
    EXPECT_EQ(registry.find(7)->name, "Golfer 7");
    EXPECT_EQ(registry.find(99), nullptr);
    // Remembering again replaces rather than duplicates.
    auto changed = entry(7, 200);
    registry.remember(changed);
    EXPECT_EQ(registry.size(), 2u);
    EXPECT_EQ(registry.find(7)->lastHappiness, 200);
    EXPECT_TRUE(registry.forget(7));
    EXPECT_FALSE(registry.forget(7));
    EXPECT_EQ(registry.size(), 1u);
    EXPECT_NE(registry.find(8), nullptr); // the index stays right after removals
}

TEST(GolferRegistry, EvictsTheLeastRecentlySeenWhenFull)
{
    GolferRegistry registry(3);
    registry.remember(entry(1));
    registry.remember(entry(2));
    registry.remember(entry(3));
    registry.recordVisit(1, 5, 100); // 1 is now the most recent
    registry.remember(entry(4));
    EXPECT_EQ(registry.size(), 3u);
    EXPECT_EQ(registry.find(2), nullptr); // the oldest went
    EXPECT_NE(registry.find(1), nullptr);
    EXPECT_NE(registry.find(3), nullptr);
    EXPECT_NE(registry.find(4), nullptr);
    registry.setCapacity(1);
    EXPECT_EQ(registry.size(), 1u);
    EXPECT_NE(registry.find(4), nullptr); // the newest stays
}

TEST(GolferRegistry, TheDefaultCapacityIsBounded)
{
    GolferRegistry registry;
    for (uint32_t i = 1; i <= 2500; i++)
        registry.remember(entry(i));
    EXPECT_EQ(registry.size(), 2000u);
    EXPECT_EQ(registry.find(1), nullptr);
    EXPECT_NE(registry.find(2500), nullptr);
    EXPECT_NE(registry.find(501), nullptr);
}

TEST(GolferRegistry, VisitsUpdateTheRecord)
{
    GolferRegistry registry;
    registry.remember(entry(1));
    EXPECT_TRUE(registry.recordVisit(1, 12, 33));
    EXPECT_EQ(registry.find(1)->lastSeenDay, 12u);
    EXPECT_EQ(registry.find(1)->lastHappiness, 33);
    EXPECT_EQ(registry.find(1)->visits, 1u);
    EXPECT_FALSE(registry.recordVisit(404, 1, 1));
}

TEST(GolferRegistry, DecayAppliesOnceForTheDaysAway)
{
    GolferRegistry registry;
    auto e = entry(1, 128, 10);
    e.initial = SkillPoints::from({ 40, 40, 40, 40 });
    e.skills = SkillPoints::from({ 60, 60, 60, 60 });
    registry.remember(e);
    registry.decayTo(20);
    const auto afterTen = registry.find(1)->skills.longPlay;
    EXPECT_EQ(afterTen, 60 * 256 - 10 * kDecayPerDay256);
    registry.decayTo(20); // the same day again changes nothing
    EXPECT_EQ(registry.find(1)->skills.longPlay, afterTen);
    registry.decayTo(15); // going back is ignored
    EXPECT_EQ(registry.find(1)->skills.longPlay, afterTen);
}

TEST(GolferRegistry, HappyGolfersReturnMoreOften)
{
    GolferRegistry registry;
    registry.remember(entry(1, 240));
    registry.remember(entry(2, 20));
    SeqRandom rng(9);
    std::map<uint32_t, int> picks;
    constexpr int kDraws = 6000;
    for (int i = 0; i < kDraws; i++)
        picks[registry.pickReturning(rng)->id]++;
    // Chances proportional to happiness + 16: 256 against 36.
    const double ratio = static_cast<double>(picks[1]) / picks[2];
    EXPECT_NEAR(ratio, 256.0 / 36.0, 1.5);
    EXPECT_GT(picks[2], 0);
}

TEST(GolferRegistry, GolfersOnTheCourseAreNotPicked)
{
    GolferRegistry registry;
    registry.remember(entry(1));
    registry.remember(entry(2));
    SeqRandom rng(2);
    for (int i = 0; i < 100; i++)
        EXPECT_EQ(registry.pickReturning(rng, { 1 })->id, 2u);
    EXPECT_EQ(registry.pickReturning(rng, { 1, 2 }), nullptr);
    GolferRegistry empty;
    EXPECT_EQ(empty.pickReturning(rng), nullptr);
}

TEST(GolferRegistry, PickingIsDeterministic)
{
    GolferRegistry registry;
    for (uint32_t i = 1; i <= 20; i++)
        registry.remember(entry(i, static_cast<uint8_t>(i * 10)));
    SeqRandom a(77), b(77);
    for (int i = 0; i < 50; i++)
        EXPECT_EQ(registry.pickReturning(a)->id, registry.pickReturning(b)->id);
}
