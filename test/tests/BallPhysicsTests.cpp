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
#include <openrct2/golf/BallPhysics.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Golf;
using namespace GolfTest;

namespace
{
    Ball ballAt(const TestWorld& world, int x, int y, BallState state = BallState::InFlight)
    {
        Ball ball;
        placeBall(ball, world, Fixed::fromInt(x), Fixed::fromInt(y), state);
        return ball;
    }

    int runUntilSettled(Ball& ball, const TestWorld& world, SeqRandom& rng, int maxTicks = 4000, BallEvents* total = nullptr)
    {
        int ticks = 0;
        while (!isSettled(ball.state) && ticks < maxTicks)
        {
            const auto ev = stepBall(ball, world, rng);
            if (total != nullptr)
            {
                total->impacts += ev.impacts;
                total->obstacleHits += ev.obstacleHits;
                total->holed |= ev.holed;
                total->lippedOut |= ev.lippedOut;
                total->enteredWater |= ev.enteredWater;
                total->wentOutOfBounds |= ev.wentOutOfBounds;
                total->hitGolfer |= ev.hitGolfer;
                total->golferId = ev.golferId ? ev.golferId : total->golferId;
            }
            ticks++;
        }
        return ticks;
    }
} // namespace

TEST(BallPhysics, ATeedBallDoesNotMove)
{
    TestWorld world;
    SeqRandom rng;
    auto ball = ballAt(world, 100, 100, BallState::Teed);
    ball.velocity = vec(1, 0, 0);
    stepBall(ball, world, rng);
    EXPECT_EQ(ball.position.x, Fixed::fromInt(100));
    EXPECT_EQ(ball.state, BallState::Teed);
}

TEST(BallPhysics, AFlyingBallFollowsAnArcAndLands)
{
    TestWorld world;
    SeqRandom rng;
    auto ball = ballAt(world, 100, 100);
    ball.position.z = Fixed::fromInt(1);
    ball.velocity = vec(8, 0, 6);
    Fixed apex = ball.position.z;
    int ticks = 0;
    BallEvents events;
    while (ball.state == BallState::InFlight && ticks < 1000)
    {
        events = stepBall(ball, world, rng);
        apex = max(apex, ball.position.z);
        ticks++;
    }
    EXPECT_GT(apex, Fixed::fromInt(100));
    EXPECT_GT(ball.position.x, Fixed::fromInt(400));
    EXPECT_EQ(ball.position.y, Fixed::fromInt(100)); // no side spin: no curve
    EXPECT_EQ(ball.state, BallState::Bouncing);
    EXPECT_EQ(events.impacts, 1);
    EXPECT_GT(events.strongestImpact, Fixed::fromInt(3));
}

TEST(BallPhysics, DragSlowsTheBall)
{
    TestWorld world;
    SeqRandom rng;
    auto ball = ballAt(world, 0, 100);
    ball.position.z = Fixed::fromInt(5000);
    ball.velocity = vec(10, 0, 0);
    for (int i = 0; i < 40; i++)
        stepBall(ball, world, rng);
    EXPECT_LT(ball.velocity.x, Fixed::fromInt(10));
    EXPECT_GT(ball.velocity.x, Fixed::fromInt(9));
}

TEST(BallPhysics, SideSpinCurvesToTheRightOfTheTravelDirection)
{
    TestWorld world;
    SeqRandom rng;
    auto slice = ballAt(world, 100, 500);
    slice.position.z = Fixed::fromInt(1);
    slice.velocity = vec(10, 0, 5);
    slice.sideSpin = Fixed::ratio(1, 2);
    auto hook = slice;
    hook.sideSpin = -Fixed::ratio(1, 2);
    auto straight = slice;
    straight.sideSpin = {};
    for (auto* b : { &slice, &hook, &straight })
        while (b->state == BallState::InFlight)
            stepBall(*b, world, rng);
    // Moving along +x, the right hand side is -y.
    EXPECT_LT(slice.position.y, Fixed::fromInt(500) - Fixed::fromInt(20));
    EXPECT_GT(hook.position.y, Fixed::fromInt(500) + Fixed::fromInt(20));
    EXPECT_EQ(straight.position.y, Fixed::fromInt(500));
}

TEST(BallPhysics, BackSpinLiftsTheBall)
{
    TestWorld world;
    SeqRandom rng;
    auto plain = ballAt(world, 100, 100);
    plain.position.z = Fixed::fromInt(1);
    plain.velocity = vec(10, 0, 5);
    auto spun = plain;
    spun.backSpin = Fixed::fromInt(1);
    Fixed plainApex{}, spunApex{};
    while (plain.state == BallState::InFlight)
    {
        stepBall(plain, world, rng);
        plainApex = max(plainApex, plain.position.z);
    }
    while (spun.state == BallState::InFlight)
    {
        stepBall(spun, world, rng);
        spunApex = max(spunApex, spun.position.z);
    }
    EXPECT_GT(spunApex, plainApex);
}

TEST(BallPhysics, WindPushesTheBall)
{
    TestWorld world;
    world.windVelocity = vec(0, 2, 0);
    SeqRandom rng;
    auto ball = ballAt(world, 100, 100);
    ball.position.z = Fixed::fromInt(1);
    ball.velocity = vec(8, 0, 6);
    while (ball.state == BallState::InFlight)
        stepBall(ball, world, rng);
    EXPECT_GT(ball.position.y, Fixed::fromInt(100) + Fixed::fromInt(10));
}

TEST(BallPhysics, ABounceKeepsTheSurfaceBounciness)
{
    TestWorld hard;
    hard.surface.bounciness = Fixed::ratio(8, 10);
    TestWorld soft;
    soft.surface.bounciness = Fixed::ratio(1, 10);
    SeqRandom rng;
    auto a = ballAt(hard, 100, 100);
    a.position.z = Fixed::fromInt(50);
    a.velocity = vec(0, 0, -5);
    auto b = a;
    while (a.bounces == 0)
        stepBall(a, hard, rng);
    while (b.bounces == 0)
        stepBall(b, soft, rng);
    EXPECT_GT(a.velocity.z, Fixed::ratio(3, 1));       // keeps about 80% of the drop speed
    EXPECT_LT(b.velocity.z, Fixed::ratio(1, 1));       // keeps about 10%
    EXPECT_GT(a.velocity.z, b.velocity.z * 3);
}

TEST(BallPhysics, RoughnessRemovesTangentialSpeedAtImpact)
{
    SeqRandom rng;
    auto land = [&](Fixed roughness) {
        TestWorld world;
        world.surface.roughness = roughness;
        world.surface.bounciness = Fixed::ratio(3, 10);
        auto ball = ballAt(world, 100, 100);
        ball.position.z = Fixed::fromInt(10);
        ball.velocity = vec(6, 0, -4);
        while (ball.bounces == 0)
            stepBall(ball, world, rng);
        return ball.velocity.x;
    };
    const auto smooth = land(Fixed::ratio(5, 100));
    const auto rough = land(Fixed::ratio(9, 10));
    EXPECT_GT(smooth, rough * 2);
    EXPECT_GT(rough, Fixed{});
}

TEST(BallPhysics, SlowImpactsTurnIntoRolling)
{
    TestWorld world;
    SeqRandom rng;
    auto ball = ballAt(world, 100, 100);
    ball.position.z = Fixed::ratio(1, 100);
    ball.velocity = { Fixed::fromInt(2), Fixed{}, -Fixed::ratio(1, 100) };
    for (int i = 0; i < 4 && ball.state != BallState::Rolling; i++)
        stepBall(ball, world, rng);
    EXPECT_EQ(ball.state, BallState::Rolling);
    EXPECT_EQ(ball.velocity.z, Fixed{});
}

TEST(BallPhysics, RollingBallStopsAndRoughnessShortensTheRoll)
{
    SeqRandom rng;
    auto roll = [&](Fixed roughness) {
        TestWorld world;
        world.surface.roughness = roughness;
        auto ball = ballAt(world, 100, 100, BallState::Rolling);
        ball.velocity = vec(1, 0, 0);
        runUntilSettled(ball, world, rng);
        EXPECT_EQ(ball.state, BallState::AtRest);
        return ball.position.x - Fixed::fromInt(100);
    };
    const auto green = roll(Fixed::ratio(5, 100));
    const auto fairway = roll(Fixed::ratio(15, 100));
    const auto sand = roll(Fixed::ratio(9, 10));
    EXPECT_GT(green, fairway * 2);
    EXPECT_GT(fairway, sand * 4);
    // v^2 / (2 a) with a = 0.05 * roughness: 1 / (2 * 0.0025) = 200 units on a green.
    EXPECT_NEAR(green.toDouble(), 200.0, 15.0);
}

TEST(BallPhysics, BallsRollDownhillEvenOnAGreen)
{
    TestWorld world;
    world.slopeX = Fixed::ratio(-1, 10); // height falls towards +x: a 10% slope
    world.surface.roughness = Fixed::ratio(5, 100);
    SeqRandom rng;
    auto ball = ballAt(world, 100, 100, BallState::Rolling);
    for (int i = 0; i < 200; i++)
        stepBall(ball, world, rng);
    EXPECT_GT(ball.position.x, Fixed::fromInt(110));
    EXPECT_GT(ball.velocity.x, Fixed{});
    // The ball stays on the sloping ground.
    EXPECT_EQ(ball.position.z, world.surfaceAt(ball.position.x, ball.position.y).height);
}

namespace
{
    // Two planes meeting in a crease along x = 100, like two tiles of a terrain valley.
    class ValleyWorld : public TestWorld
    {
    public:
        SurfaceSample surfaceAt(Fixed x, Fixed y) const override
        {
            SurfaceSample s = surface;
            const Fixed slope = Fixed::ratio(25, 100);
            const bool right = x >= Fixed::fromInt(100);
            s.height = right ? (x - Fixed::fromInt(100)) * slope : (Fixed::fromInt(100) - x) * slope;
            s.normal = normalised(FixedVec3{ right ? -slope : slope, Fixed{}, Fixed::fromInt(1) });
            return s;
        }
    };
} // namespace

TEST(BallPhysics, ASlowBallInAValleyComesToRestOnTheCrease)
{
    ValleyWorld world;
    world.surface.roughness = Fixed::ratio(5, 100);
    SeqRandom rng;
    auto ball = ballAt(world, 103, 100, BallState::Rolling);
    const int ticks = runUntilSettled(ball, world, rng);
    EXPECT_LT(ticks, 4000);
    EXPECT_EQ(ball.state, BallState::AtRest);
    EXPECT_NEAR(ball.position.x.toDouble(), 100.0, 3.0);
}

TEST(BallPhysics, AGentleSlopeIsHeldByFriction)
{
    TestWorld world;
    world.slopeX = Fixed::ratio(-2, 100); // 2% slope
    world.surface.roughness = Fixed::ratio(5, 100);
    SeqRandom rng;
    auto ball = ballAt(world, 100, 100, BallState::Rolling);
    runUntilSettled(ball, world, rng);
    EXPECT_EQ(ball.state, BallState::AtRest);
    EXPECT_LT(ball.position.x, Fixed::fromInt(110));
}

TEST(BallPhysics, ARollingBallLeavesAnEdgeAndFliesOn)
{
    TestWorld world;
    world.cliffX = Fixed::fromInt(120); // ground drops 40 units beyond x = 120
    SeqRandom rng;
    auto ball = ballAt(world, 100, 100, BallState::Rolling);
    ball.velocity = vec(2, 0, 0);
    bool flew = false;
    for (int i = 0; i < 400 && !flew; i++)
    {
        stepBall(ball, world, rng);
        flew = ball.state == BallState::InFlight;
    }
    EXPECT_TRUE(flew);
    runUntilSettled(ball, world, rng);
    EXPECT_EQ(ball.position.z, world.surfaceAt(ball.position.x, ball.position.y).height);
    EXPECT_LT(ball.position.z, Fixed::fromInt(-30));
}

TEST(BallPhysics, WaterLosesTheBallAndRemembersWhereItCrossed)
{
    TestWorld world;
    world.waterFromX = Fixed::fromInt(300);
    SeqRandom rng;
    auto ball = ballAt(world, 100, 100);
    ball.position.z = Fixed::fromInt(1);
    ball.velocity = vec(8, 0, 6);
    BallEvents total;
    runUntilSettled(ball, world, rng, 4000, &total);
    EXPECT_EQ(ball.state, BallState::Lost);
    EXPECT_TRUE(total.enteredWater);
    EXPECT_LT(ball.hazardEntry.x, Fixed::fromInt(300));
    EXPECT_GT(ball.hazardEntry.x, Fixed::fromInt(200));
}

TEST(BallPhysics, ARollingBallThatEntersWaterIsLost)
{
    TestWorld world;
    world.waterFromX = Fixed::fromInt(150);
    SeqRandom rng;
    auto ball = ballAt(world, 100, 100, BallState::Rolling);
    ball.velocity = vec(1, 0, 0);
    runUntilSettled(ball, world, rng);
    EXPECT_EQ(ball.state, BallState::Lost);
    EXPECT_LT(ball.hazardEntry.x, Fixed::fromInt(150));
}

TEST(BallPhysics, RollingBallCrossingStreamLosesEightyPercentSpeed)
{
    TestWorld world;
    world.surface.roughness = Fixed::ratio(5, 100);
    world.streamFromX = Fixed::fromInt(150);
    world.streamToX = Fixed::fromInt(155);
    SeqRandom rng;
    auto ball = ballAt(world, 140, 100, BallState::Rolling);
    ball.velocity = vec(10, 0, 0);

    // Roll until past 150
    while (ball.position.x < Fixed::fromInt(150))
        stepBall(ball, world, rng);

    // Right after entering the stream at x >= 150, speed should be ~20% of 10 = ~2
    EXPECT_GE(ball.position.x, Fixed::fromInt(150));
    EXPECT_NEAR(ball.velocity.x.toDouble(), 2.0, 0.4);

    // Ball continues and makes it across
    runUntilSettled(ball, world, rng);
    EXPECT_EQ(ball.state, BallState::AtRest);
    EXPECT_GT(ball.position.x, Fixed::fromInt(155));
}

TEST(BallPhysics, SlowBallStoppingInStreamIsWaterHazard)
{
    TestWorld world;
    world.streamFromX = Fixed::fromInt(150);
    world.streamToX = Fixed::fromInt(160);
    SeqRandom rng;
    auto ball = ballAt(world, 148, 100, BallState::Rolling);
    ball.velocity = FixedVec3{ Fixed::ratio(4, 10), Fixed{}, Fixed{} };
    runUntilSettled(ball, world, rng);
    EXPECT_EQ(ball.state, BallState::Lost);
    EXPECT_GE(ball.position.x, Fixed::fromInt(150));
    EXPECT_LE(ball.position.x, Fixed::fromInt(160));
}

TEST(BallPhysics, OutOfBoundsEndsTheShot)
{
    TestWorld world;
    world.outOfBoundsFromX = Fixed::fromInt(300);
    SeqRandom rng;
    auto ball = ballAt(world, 100, 100);
    ball.position.z = Fixed::fromInt(1);
    ball.velocity = vec(8, 0, 6);
    BallEvents total;
    runUntilSettled(ball, world, rng, 4000, &total);
    EXPECT_EQ(ball.state, BallState::OutOfBounds);
    EXPECT_TRUE(total.wentOutOfBounds);
}

TEST(BallPhysics, ASlowBallDropsIntoTheCup)
{
    TestWorld world;
    world.cup = { vec(200, 100, 0), Fixed::fromInt(3) };
    world.hasCup = true;
    SeqRandom rng;
    auto ball = ballAt(world, 150, 100, BallState::Rolling);
    ball.velocity = { Fixed::ratio(3, 10), Fixed{}, Fixed{} };
    world.surface.roughness = Fixed::ratio(1, 100);
    BallEvents total;
    runUntilSettled(ball, world, rng, 4000, &total);
    EXPECT_EQ(ball.state, BallState::Holed);
    EXPECT_TRUE(total.holed);
    EXPECT_EQ(ball.position, world.cup.position);
    EXPECT_EQ(ball.velocity, FixedVec3{});
}

TEST(BallPhysics, AFastBallLipsOutOfTheCup)
{
    TestWorld world;
    world.cup = { vec(200, 100, 0), Fixed::fromInt(3) };
    world.hasCup = true;
    world.surface.roughness = Fixed::ratio(1, 100);
    SeqRandom rng;
    auto ball = ballAt(world, 150, 101, BallState::Rolling);
    ball.velocity = { Fixed::fromInt(2), Fixed{}, Fixed{} };
    BallEvents total;
    for (int i = 0; i < 100; i++)
    {
        const auto ev = stepBall(ball, world, rng);
        total.lippedOut |= ev.lippedOut;
        total.holed |= ev.holed;
    }
    EXPECT_TRUE(total.lippedOut);
    EXPECT_FALSE(total.holed);
    EXPECT_NE(ball.velocity.y, Fixed{}); // deflected
    EXPECT_GT(ball.position.x, Fixed::fromInt(205));
}

TEST(BallPhysics, ABallFlyingHighOverTheCupIsNotCaptured)
{
    TestWorld world;
    world.cup = { vec(200, 100, 0), Fixed::fromInt(3) };
    world.hasCup = true;
    SeqRandom rng;
    auto ball = ballAt(world, 190, 100);
    ball.position.z = Fixed::fromInt(30);
    ball.velocity = vec(1, 0, 0);
    BallEvents total;
    for (int i = 0; i < 30; i++)
    {
        const auto ev = stepBall(ball, world, rng);
        total.holed |= ev.holed;
    }
    EXPECT_FALSE(total.holed);
}

TEST(BallPhysics, ABallFiredAtABuildingBouncesBack)
{
    TestWorld world;
    world.box = { vec(300, 0, 0), vec(320, 1000, 200) };
    world.hasBox = true;
    world.boxRestitution = Fixed::ratio(5, 10);
    SeqRandom rng;
    auto ball = ballAt(world, 100, 100);
    ball.position.z = Fixed::fromInt(30);
    ball.velocity = vec(6, 0, 1);
    BallEvents total;
    runUntilSettled(ball, world, rng, 4000, &total);
    EXPECT_GT(total.obstacleHits, 0);
    EXPECT_LT(ball.position.x, Fixed::fromInt(300)); // came back to this side of the wall
    EXPECT_NE(ball.state, BallState::Teed);
}

TEST(BallPhysics, ObstacleRestitutionDecidesHowHardItBouncesBack)
{
    SeqRandom rng;
    auto bounceSpeed = [&](Fixed restitution) {
        TestWorld world;
        world.box = { vec(300, 0, 0), vec(320, 1000, 200) };
        world.hasBox = true;
        world.boxRestitution = restitution;
        world.boxFriction = {};
        auto ball = ballAt(world, 290, 100);
        ball.position.z = Fixed::fromInt(30);
        ball.velocity = vec(5, 0, 0);
        while (ball.velocity.x > Fixed{})
            stepBall(ball, world, rng);
        return -ball.velocity.x;
    };
    EXPECT_GT(bounceSpeed(Fixed::ratio(8, 10)), bounceSpeed(Fixed::ratio(2, 10)) * 3);
}

TEST(BallPhysics, PassThroughChanceLetsSomeBallsThroughDeterministically)
{
    auto fire = [&](Fixed chance, uint32_t seed) {
        TestWorld world;
        world.box = { vec(300, 0, 0), vec(320, 1000, 200) };
        world.hasBox = true;
        world.boxPassThrough = chance;
        SeqRandom rng(seed);
        auto ball = ballAt(world, 280, 100);
        ball.position.z = Fixed::fromInt(30);
        ball.velocity = vec(3, 0, 0);
        for (int i = 0; i < 60; i++)
            stepBall(ball, world, rng);
        return ball.position.x;
    };
    int through = 0;
    for (uint32_t seed = 0; seed < 40; seed++)
    {
        const auto x = fire(Fixed::ratio(1, 2), seed * 7919 + 13);
        if (x > Fixed::fromInt(320))
            through++;
        EXPECT_EQ(x, fire(Fixed::ratio(1, 2), seed * 7919 + 13)); // same seed, same result
    }
    EXPECT_GT(through, 8);
    EXPECT_LT(through, 32);
    EXPECT_LT(fire(Fixed{}, 5), Fixed::fromInt(300));
    EXPECT_GT(fire(Fixed::fromInt(1), 5), Fixed::fromInt(320));
}

TEST(BallPhysics, AGolferStopsTheBallAndIsHit)
{
    TestWorld world;
    world.box = { vec(300, 90, 0), vec(310, 110, 60) };
    world.hasBox = true;
    world.boxIsGolfer = true;
    SeqRandom rng;
    auto ball = ballAt(world, 250, 100);
    ball.position.z = Fixed::fromInt(20);
    ball.velocity = vec(5, 0, 0);
    BallEvents total;
    runUntilSettled(ball, world, rng, 4000, &total);
    EXPECT_TRUE(total.hitGolfer);
    EXPECT_EQ(total.golferId, 77u);
    EXPECT_LT(ball.position.x, Fixed::fromInt(312));
    EXPECT_EQ(ball.state, BallState::AtRest);
}

TEST(BallPhysics, SettledBallsStaySettled)
{
    TestWorld world;
    SeqRandom rng;
    auto ball = ballAt(world, 100, 100, BallState::AtRest);
    ball.velocity = vec(5, 0, 0);
    stepBall(ball, world, rng);
    EXPECT_EQ(ball.position.x, Fixed::fromInt(100));
}

TEST(BallPhysics, SimulationIsDeterministic)
{
    // 1000 random shots run twice end at identical positions (BAL AC, ARC-020).
    auto run = [&](uint32_t seed) {
        TestWorld world;
        world.slopeX = Fixed::ratio(-3, 100);
        world.slopeY = Fixed::ratio(2, 100);
        SeqRandom rng(seed);
        uint64_t hash = 1469598103934665603ull;
        for (int shot = 0; shot < 1000; shot++)
        {
            auto ball = ballAt(world, 1000, 1000);
            ball.position.z += Fixed::fromInt(1);
            ball.velocity = { Fixed::fromRaw(static_cast<int64_t>(rng.next() % 900000) - 100000),
                              Fixed::fromRaw(static_cast<int64_t>(rng.next() % 600000) - 300000),
                              Fixed::fromRaw(static_cast<int64_t>(rng.next() % 400000)) };
            ball.sideSpin = Fixed::fromRaw(static_cast<int64_t>(rng.next() % 40000) - 20000);
            ball.backSpin = Fixed::fromRaw(static_cast<int64_t>(rng.next() % 40000));
            runUntilSettled(ball, world, rng, 3000);
            for (auto v : { ball.position.x.raw(), ball.position.y.raw(), ball.position.z.raw(),
                            static_cast<int64_t>(ball.state) })
                hash = (hash ^ static_cast<uint64_t>(v)) * 1099511628211ull;
        }
        return hash;
    };
    EXPECT_EQ(run(1), run(1));
    EXPECT_NE(run(1), run(2));
}
