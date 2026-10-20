/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <gtest/gtest.h>
#include <string>
#include <openrct2/golf/GolferNeeds.h>

using namespace OpenRCT2::Golf;

TEST(GolferNeeds, StartFullAndWithMiddlingHappiness)
{
    const Needs n;
    EXPECT_EQ(n.hungerByte(), 255);
    EXPECT_EQ(n.thirstByte(), 255);
    EXPECT_EQ(n.energyByte(), 255);
    EXPECT_EQ(n.bladderByte(), 0);
    EXPECT_EQ(n.happinessByte(), 128);
}

TEST(GolferNeeds, DrainRunsPerMinuteAndPhysiqueSlowsIt)
{
    Needs average, fit, unfit;
    drainMinutes(average, 20, 50, false);
    drainMinutes(fit, 20, 100, false);
    drainMinutes(unfit, 20, 0, false);
    // 1.0 hunger per minute at physique 50 (100% drain).
    EXPECT_EQ(average.hungerByte(), 255 - 20);
    EXPECT_GT(fit.hungerByte(), average.hungerByte());
    EXPECT_LT(unfit.hungerByte(), average.hungerByte());
    // High physique drains half as fast, low physique one and a half times.
    EXPECT_EQ(fit.hungerByte(), 255 - 10);
    EXPECT_EQ(unfit.hungerByte(), 255 - 30);
    // The bladder rises, thirst drains faster than hunger.
    EXPECT_GT(average.bladderByte(), 0);
    EXPECT_LT(average.thirstByte(), average.hungerByte());
}

TEST(GolferNeeds, WalkingAndSwingingCostEnergy)
{
    Needs idle, walking, swinging;
    drainMinutes(idle, 10, 50, false);
    drainMinutes(walking, 10, 50, true);
    EXPECT_GT(idle.energyByte(), walking.energyByte());
    for (int i = 0; i < 20; i++)
        drainSwing(swinging, 50);
    EXPECT_LT(swinging.energyByte(), 255);
    EXPECT_EQ(swinging.hungerByte(), 255); // a swing only costs energy
}

TEST(GolferNeeds, NeedsStayWithinTheirRange)
{
    Needs n;
    drainMinutes(n, 100000, 0, true);
    EXPECT_EQ(n.hungerByte(), 0);
    EXPECT_EQ(n.thirstByte(), 0);
    EXPECT_EQ(n.energyByte(), 0);
    EXPECT_EQ(n.bladderByte(), 255);
    drainMinutes(n, 0, 50, true);
    drainMinutes(n, -5, 50, true);
    EXPECT_EQ(n.hungerByte(), 0);
    EXPECT_EQ(physiqueDrainPercent(-10), 150);
    EXPECT_EQ(physiqueDrainPercent(500), 50);
}

TEST(GolferNeeds, ASatisfiedGolferSeeksNothing)
{
    Needs n;
    bool critical = true;
    EXPECT_EQ(mostUrgentNeed(n, &critical), NeedKind::none);
    EXPECT_FALSE(critical);
}

TEST(GolferNeeds, HungerBelowTheThresholdSendsTheGolferToABuilding)
{
    Needs n;
    n.hunger = (kSeekHungerBelow - 1) * 16;
    bool critical = true;
    EXPECT_EQ(mostUrgentNeed(n, &critical), NeedKind::hunger);
    EXPECT_FALSE(critical);
    n.hunger = kSeekHungerBelow * 16; // exactly at the threshold: not yet
    EXPECT_EQ(mostUrgentNeed(n), NeedKind::none);
}

TEST(GolferNeeds, CriticalNeedsComeFirstAndAreFlagged)
{
    Needs n;
    n.thirst = 50 * 16;   // ordinary
    n.energy = 10 * 16;   // critical
    bool critical = false;
    EXPECT_EQ(mostUrgentNeed(n, &critical), NeedKind::energy);
    EXPECT_TRUE(critical);
    n.hunger = 5 * 16; // more critical than energy
    EXPECT_EQ(mostUrgentNeed(n, &critical), NeedKind::hunger);
    EXPECT_TRUE(critical);
}

TEST(GolferNeeds, TheWorstOrdinaryNeedWins)
{
    Needs n;
    n.hunger = 80 * 16;   // 10 below the threshold
    n.bladder = 220 * 16; // 70 over... (critical at 220 is > 220, so ordinary)
    bool critical = true;
    EXPECT_EQ(mostUrgentNeed(n, &critical), NeedKind::bladder);
    EXPECT_FALSE(critical);
}

TEST(GolferNeeds, TheBladderIsHighWhenItNeedsRelief)
{
    Needs n;
    n.bladder = (kSeekBladderAbove + 1) * 16;
    EXPECT_EQ(mostUrgentNeed(n), NeedKind::bladder);
    n.bladder = 100 * 16;
    EXPECT_EQ(mostUrgentNeed(n), NeedKind::none);
}

TEST(GolferNeeds, HappinessModifiersHaveTheRightSign)
{
    for (auto good : { HappinessEvent::goodShot, HappinessEvent::parOrBetter, HappinessEvent::birdie, HappinessEvent::eagle,
                       HappinessEvent::holeInOne, HappinessEvent::distanceSign, HappinessEvent::sceneryNearHole,
                       HappinessEvent::cleanPaths, HappinessEvent::shortWait })
        EXPECT_GT(happinessDelta(good), 0) << static_cast<int>(good);
    for (auto bad : { HappinessEvent::waitedAMinute, HappinessEvent::bogey, HappinessEvent::doubleBogey,
                      HappinessEvent::worseThanDoubleBogey, HappinessEvent::lostBall, HappinessEvent::ballInHazard,
                      HappinessEvent::holeTooHard, HappinessEvent::hungry, HappinessEvent::thirsty, HappinessEvent::bladderFull,
                      HappinessEvent::litter, HappinessEvent::hitByBall, HappinessEvent::crowdedPath, HappinessEvent::highPrices })
        EXPECT_LT(happinessDelta(bad), 0) << static_cast<int>(bad);
    // Better scores earn more.
    EXPECT_GT(happinessDelta(HappinessEvent::eagle), happinessDelta(HappinessEvent::birdie));
    EXPECT_GT(happinessDelta(HappinessEvent::holeInOne), happinessDelta(HappinessEvent::eagle));
    EXPECT_LT(happinessDelta(HappinessEvent::worseThanDoubleBogey), happinessDelta(HappinessEvent::doubleBogey));
}

TEST(GolferNeeds, HappinessChangesAndStaysInRange)
{
    Needs n;
    applyHappiness(n, HappinessEvent::birdie);
    EXPECT_EQ(n.happinessByte(), 128 + happinessDelta(HappinessEvent::birdie));
    applyHappiness(n, HappinessEvent::holeInOne, 10);
    EXPECT_EQ(n.happinessByte(), 255);
    applyHappiness(n, HappinessEvent::hitByBall, 100);
    EXPECT_EQ(n.happinessByte(), 0);
    applyHappiness(n, HappinessEvent::hitByBall, 0); // zero times, no change
    applyHappiness(n, HappinessEvent::goodShot, -3); // negative counts are ignored
    EXPECT_EQ(n.happinessByte(), 0);
}

TEST(GolferNeeds, WaitingCostsHappinessPerMinute)
{
    Needs n;
    applyHappiness(n, HappinessEvent::waitedAMinute, 5);
    EXPECT_EQ(n.happinessByte(), 128 + 5 * happinessDelta(HappinessEvent::waitedAMinute));
}

TEST(GolferNeeds, BadNeedsHurtHappinessOverTime)
{
    Needs fine;
    applyNeedPenalties(fine, 30);
    EXPECT_EQ(fine.happinessByte(), 128);
    Needs hungry;
    hungry.hunger = 40 * 16;
    hungry.thirst = 40 * 16;
    hungry.bladder = 200 * 16;
    applyNeedPenalties(hungry, 10);
    EXPECT_LT(hungry.happinessByte(), 128 - 10);
    applyNeedPenalties(hungry, 0);
}

TEST(GolferNeeds, ThoughtsMatchTheirNeed)
{
    EXPECT_EQ(thoughtForNeed(NeedKind::hunger), ThoughtType::hungry);
    EXPECT_EQ(thoughtForNeed(NeedKind::thirst), ThoughtType::thirsty);
    EXPECT_EQ(thoughtForNeed(NeedKind::bladder), ThoughtType::needToilet);
    EXPECT_EQ(thoughtForNeed(NeedKind::energy), ThoughtType::tired);
    EXPECT_EQ(thoughtForNeed(NeedKind::none), ThoughtType::enjoyingTheRound);
    for (int i = 0; i < static_cast<int>(ThoughtType::count); i++)
        EXPECT_GT(std::string(thoughtName(static_cast<ThoughtType>(i))).size(), 5u) << i;
}
