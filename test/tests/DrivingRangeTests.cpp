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
#include <openrct2/golf/DrivingRange.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Golf;
using namespace GolfTest;

namespace
{
    // A range 40 tiles long running east from x = 20 tiles, five hitting tiles along its west edge.
    DrivingRange makeRange(int balls = 6)
    {
        DrivingRange r;
        r.id = 1;
        r.areaMin = { 20, 20 };
        r.areaMax = { 60, 30 };
        r.ballsPerBucket = balls;
        r.bucketPrice = 300;
        for (int i = 0; i < 5; i++)
            r.hittingTiles.push_back(HolePoint{ 20 * 32 + 16, 22 * 32 + 16 + i * 32 });
        r.kiosk = HolePoint{ 18 * 32, 24 * 32 }; // behind the hitting tiles
        r.shootingDirectionDegrees = 0;
        return r;
    }

    Golfer aGolfer(uint32_t id, int64_t money = 5000)
    {
        Golfer g;
        g.id = id;
        g.money = money;
        g.position = vec(300, 700, 0);
        return g;
    }

    struct Rig
    {
        FunctionWorld world;
        SeqRandom rng{ 3 };
        RangeSimulation sim;
        explicit Rig(DrivingRange range)
            : sim(&world, &rng, std::move(range))
        {
        }
    };

    void runAll(Rig& rig, std::vector<Golfer>& golfers, int maxTicks = 200000)
    {
        for (int t = 0; t < maxTicks; t++)
        {
            bool allDone = true;
            for (auto& g : golfers)
                if (rig.sim.isOnRange(g.id))
                {
                    rig.sim.tick(g);
                    allDone = false;
                }
            if (allDone)
                break;
        }
    }
} // namespace

TEST(DrivingRange, RoundingToFive)
{
    EXPECT_EQ(roundToFive(0), 0);
    EXPECT_EQ(roundToFive(2), 0);
    EXPECT_EQ(roundToFive(3), 5);
    EXPECT_EQ(roundToFive(147), 145);
    EXPECT_EQ(roundToFive(148), 150);
    EXPECT_EQ(roundToFive(150), 150);
}

TEST(DrivingRange, SignsShowTheDistanceFromTheKioskRoundedToFive)
{
    const HolePoint kiosk{ 1000, 1000 };
    // 100 tiles away = 500 yards... 30 tiles = 150 yards.
    EXPECT_EQ(signDistance(kiosk, { 1000 + 30 * 32, 1000 }), 150);
    // 31 tiles + 3 units = 155.5 yards -> 155 or 160: nearest multiple of five.
    EXPECT_EQ(signDistance(kiosk, { 1000 + 31 * 32 + 3, 1000 }), 155);
    EXPECT_EQ(signDistance(kiosk, { 1000 + 31 * 32 + 20, 1000 }), 160);
    EXPECT_EQ(signDistance(kiosk, kiosk), 0);
    // Along the diagonal: 30 tiles each way is 42.4 tiles = 212 yards.
    EXPECT_EQ(signDistance(kiosk, { 1000 + 30 * 32, 1000 + 30 * 32 }), 210);
}

TEST(DrivingRange, MetresAreConvertedAndRoundedToFiveToo)
{
    const HolePoint kiosk{ 0, 0 };
    // 200 yards = 182.9 m.
    EXPECT_EQ(signDistance(kiosk, { 40 * 32, 0 }, DistanceUnit::yards), 200);
    EXPECT_EQ(signDistance(kiosk, { 40 * 32, 0 }, DistanceUnit::metres), 185);
    EXPECT_EQ(signDistance(kiosk, { 10 * 32, 0 }, DistanceUnit::metres), 45); // 50 yd = 45.7 m
}

TEST(DrivingRange, SignDistancesFollowTheKioskWhenItMoves)
{
    auto range = makeRange();
    const HolePoint sign{ 40 * 32, 24 * 32 };
    const int32_t before = signDistance(*range.kiosk, sign);
    range.kiosk = HolePoint{ 10 * 32, 24 * 32 };
    EXPECT_EQ(signDistance(*range.kiosk, sign), before + 40);
}

TEST(DrivingRange, AKioskBehindTheHittingLineIsSafe)
{
    auto range = makeRange();
    EXPECT_FALSE(kioskMayBeHit(range));
    range.kiosk = HolePoint{ 19 * 32, 24 * 32 }; // just behind the tees (x = 20.5 tiles)
    EXPECT_FALSE(kioskMayBeHit(range));
}

TEST(DrivingRange, AKioskLevelWithOrInFrontOfTheTeesGetsAWarning)
{
    auto range = makeRange();
    range.kiosk = HolePoint{ 20 * 32 + 16, 28 * 32 }; // level with the tees, beside them
    EXPECT_TRUE(kioskMayBeHit(range));
    range.kiosk = HolePoint{ 30 * 32, 24 * 32 }; // in the field
    EXPECT_TRUE(kioskMayBeHit(range));
    range.kiosk = HolePoint{ 19 * 32, 24 * 32 };
    EXPECT_FALSE(kioskMayBeHit(range));
    // Shooting the other way makes the same kiosk the one in front.
    range.shootingDirectionDegrees = 180;
    EXPECT_TRUE(kioskMayBeHit(range));
    // No kiosk, no tiles: nothing to warn about.
    range.kiosk.reset();
    EXPECT_FALSE(kioskMayBeHit(range));
    auto noTiles = makeRange();
    noTiles.hittingTiles.clear();
    EXPECT_FALSE(kioskMayBeHit(noTiles));
}

TEST(DrivingRange, TheShootingDirectionPointsIntoTheArea)
{
    auto range = makeRange();
    EXPECT_NEAR(defaultShootingDirection(range), 0, 12);
    // Tees along the south edge of an area that stretches to +y.
    DrivingRange north;
    north.areaMin = { 20, 20 };
    north.areaMax = { 30, 60 };
    north.hittingTiles = { HolePoint{ 25 * 32, 21 * 32 } };
    EXPECT_NEAR(defaultShootingDirection(north), 90, 10);
    DrivingRange west;
    west.areaMin = { 0, 20 };
    west.areaMax = { 40, 30 };
    west.hittingTiles = { HolePoint{ 41 * 32, 25 * 32 + 16 } };
    EXPECT_NEAR(defaultShootingDirection(west), 180, 12);
    DrivingRange empty;
    empty.shootingDirectionDegrees = 77;
    EXPECT_EQ(defaultShootingDirection(empty), 77);
}

TEST(DrivingRange, TheAreaIsATileRectangle)
{
    const auto range = makeRange();
    EXPECT_TRUE(range.containsArea({ 20 * 32, 20 * 32 }));
    EXPECT_TRUE(range.containsArea({ 60 * 32 + 31, 30 * 32 + 31 }));
    EXPECT_FALSE(range.containsArea({ 61 * 32, 25 * 32 }));
    EXPECT_FALSE(range.containsArea({ 40 * 32, 19 * 32 + 31 }));
}

TEST(RangeSimulation, AGolferBuysABucketAndGetsATile)
{
    Rig rig(makeRange());
    auto g = aGolfer(1);
    EXPECT_EQ(rig.sim.freeTiles(), 5);
    EXPECT_EQ(rig.sim.enter(g), RangeSimulation::Enter::ok);
    EXPECT_EQ(g.money, 5000 - 300);
    EXPECT_EQ(rig.sim.income(), 300);
    EXPECT_EQ(rig.sim.freeTiles(), 4);
    EXPECT_TRUE(rig.sim.isOnRange(1));
    EXPECT_EQ(g.state, GolferState::atDrivingRange);
}

TEST(RangeSimulation, GolfersWhoCannotPayOrFindNoTileAreTurnedAway)
{
    Rig rig(makeRange());
    auto poor = aGolfer(1, 100);
    EXPECT_EQ(rig.sim.enter(poor), RangeSimulation::Enter::cannotPay);
    EXPECT_EQ(poor.money, 100);
    EXPECT_FALSE(rig.sim.isOnRange(1));
    std::vector<Golfer> golfers;
    for (uint32_t i = 10; i < 15; i++)
    {
        golfers.push_back(aGolfer(i));
        EXPECT_EQ(rig.sim.enter(golfers.back()), RangeSimulation::Enter::ok);
    }
    auto sixth = aGolfer(20);
    EXPECT_EQ(rig.sim.enter(sixth), RangeSimulation::Enter::noFreeTile);
    EXPECT_EQ(sixth.money, 5000); // not charged
    EXPECT_EQ(rig.sim.freeTiles(), 0);
}

TEST(RangeSimulation, ARangeNeedsAKioskAndTiles)
{
    auto range = makeRange();
    range.kiosk.reset();
    Rig noKiosk(range);
    auto g = aGolfer(1);
    EXPECT_EQ(noKiosk.sim.enter(g), RangeSimulation::Enter::noKiosk);
    auto empty = makeRange();
    empty.hittingTiles.clear();
    Rig noTiles(empty);
    EXPECT_EQ(noTiles.sim.enter(g), RangeSimulation::Enter::noTiles);
}

TEST(RangeSimulation, EveryBallImprovesLongPlayAndTheBucketRunsOut)
{
    // Acceptance criterion: golfers use a range with 5 hitting tiles; Long play increases.
    Rig rig(makeRange(8));
    std::vector<Golfer> golfers;
    for (uint32_t i = 1; i <= 5; i++)
    {
        golfers.push_back(aGolfer(i));
        golfers.back().skills = SkillPoints::from({ 40, 40, 40, 40 });
        ASSERT_EQ(rig.sim.enter(golfers.back()), RangeSimulation::Enter::ok);
    }
    runAll(rig, golfers);
    for (const auto& g : golfers)
    {
        EXPECT_FALSE(rig.sim.isOnRange(g.id));
        EXPECT_GT(g.skills.longPlay, 40 * 256 + 8 * 80); // eight balls give about 92 sixteenths each at skill 40
        EXPECT_EQ(g.skills.accuracy, 40 * 256);           // only Long play improves
    }
    EXPECT_EQ(rig.sim.freeTiles(), 5);
    std::map<RangeEvent::Type, int> counts;
    for (const auto& e : rig.sim.takeEvents())
        counts[e.type]++;
    EXPECT_EQ(counts[RangeEvent::Type::boughtBucket], 5);
    EXPECT_EQ(counts[RangeEvent::Type::ballHit], 40);
    EXPECT_EQ(counts[RangeEvent::Type::ballLanded], 40);
    EXPECT_EQ(counts[RangeEvent::Type::finished], 5);
    EXPECT_EQ(rig.sim.income(), 5 * 300);
}

TEST(RangeSimulation, BallsLandingNearASignMakeGolfersHappy)
{
    auto range = makeRange(10);
    uint32_t id = 1;
    for (int yards = 40; yards <= 300; yards += 15)
        range.signs.push_back({ id++, HolePoint{ 20 * 32 + 16 + yards * 32 / 5, 24 * 32 } });
    Rig rig(range);
    std::vector<Golfer> golfers = { aGolfer(1) };
    ASSERT_EQ(rig.sim.enter(golfers[0]), RangeSimulation::Enter::ok);
    const int before = golfers[0].needs.happinessByte();
    runAll(rig, golfers);
    int near = 0, landed = 0;
    for (const auto& e : rig.sim.takeEvents())
    {
        near += e.type == RangeEvent::Type::nearSign;
        landed += e.type == RangeEvent::Type::ballLanded;
    }
    EXPECT_EQ(landed, 10);
    EXPECT_GE(near, 5); // signs every 15 yards: most balls land within 10 yards of one
    EXPECT_GT(golfers[0].needs.happinessByte(), before);
}

TEST(RangeSimulation, WithoutSignsThereIsNoSignHappiness)
{
    Rig rig(makeRange(6));
    std::vector<Golfer> golfers = { aGolfer(1) };
    rig.sim.enter(golfers[0]);
    const int before = golfers[0].needs.happinessByte();
    runAll(rig, golfers);
    for (const auto& e : rig.sim.takeEvents())
        EXPECT_NE(e.type, RangeEvent::Type::nearSign);
    EXPECT_LE(golfers[0].needs.happinessByte(), before);
}

TEST(RangeSimulation, BallsLandingOutsideTheAreaAreReported)
{
    auto range = makeRange(10);
    range.areaMax = { 24, 30 }; // a very short range: most balls fly past it
    Rig rig(range);
    std::vector<Golfer> golfers = { aGolfer(1) };
    rig.sim.enter(golfers[0]);
    runAll(rig, golfers);
    int outside = 0;
    for (const auto& e : rig.sim.takeEvents())
        outside += e.type == RangeEvent::Type::ballLeftArea;
    EXPECT_GE(outside, 5);
}

TEST(RangeSimulation, LeavingEarlyFreesTheTile)
{
    Rig rig(makeRange());
    auto g = aGolfer(1);
    rig.sim.enter(g);
    EXPECT_EQ(rig.sim.freeTiles(), 4);
    rig.sim.leave(1);
    EXPECT_EQ(rig.sim.freeTiles(), 5);
    EXPECT_FALSE(rig.sim.isOnRange(1));
    rig.sim.leave(1); // harmless the second time
    rig.sim.leave(99);
    EXPECT_TRUE(rig.sim.tick(g)); // not on the range any more: done
}
