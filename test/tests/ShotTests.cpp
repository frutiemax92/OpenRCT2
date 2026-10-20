/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "GolfTestWorld.h"

#include <cmath>
#include <gtest/gtest.h>
#include <openrct2/golf/Shot.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Golf;
using namespace GolfTest;

namespace
{
    // A request with no random error is not possible, so tests measure the distribution or the intended values.
    ShotRequest request(ClubId club, int longPlay = 50, int accuracy = 50)
    {
        ShotRequest r;
        r.club = club;
        r.skills.longPlay = static_cast<uint8_t>(longPlay);
        r.skills.accuracy = static_cast<uint8_t>(accuracy);
        return r;
    }

    // Flies a ball and returns where it first touches the ground, relative to the start (x along the aim).
    double firstImpact(const ShotRequest& r, uint32_t seed, double* sideways = nullptr)
    {
        TestWorld world;
        SeqRandom rng(seed);
        Ball ball;
        placeBall(ball, world, Fixed::fromInt(100), Fixed::fromInt(100));
        const auto shot = computeShot(r, rng);
        applyShot(ball, shot);
        if (shot.isPutt)
            return 0;
        while (ball.state == BallState::InFlight)
            stepBall(ball, world, rng);
        if (sideways != nullptr)
            *sideways = (ball.position.y - Fixed::fromInt(100)).toDouble();
        return (ball.position.x - Fixed::fromInt(100)).toDouble();
    }
} // namespace

TEST(Clubs, TheTableCoversEveryClubWithTheSpecifiedCarries)
{
    ASSERT_EQ(allClubs().size(), kClubCount);
    const int expected[] = { 230, 210, 195, 180, 170, 160, 150, 140, 130, 120, 105, 80 };
    for (size_t i = 0; i < 12; i++)
    {
        const auto& club = allClubs()[i];
        EXPECT_EQ(static_cast<size_t>(club.id), i);
        EXPECT_EQ(club.carryYards, expected[i]) << club.name;
        EXPECT_FALSE(club.isPutter);
        EXPECT_GT(club.loftDegrees, 0);
    }
    EXPECT_TRUE(clubInfo(ClubId::putter).isPutter);
    EXPECT_TRUE(clubInfo(ClubId::miniPutter).isPutter);
    EXPECT_EQ(clubInfo(ClubId::putter).loftDegrees, 0);
    // Loft rises as the carry falls.
    for (size_t i = 1; i < 12; i++)
    {
        EXPECT_GT(allClubs()[i].loftDegrees, allClubs()[i - 1].loftDegrees);
        EXPECT_LT(allClubs()[i].carryYards, allClubs()[i - 1].carryYards);
    }
}

TEST(Clubs, YardsConvertToWorldUnits)
{
    EXPECT_EQ(yardsToUnits(5), Fixed::fromInt(32)); // one tile
    EXPECT_EQ(unitsToYards(Fixed::fromInt(32)), Fixed::fromInt(5));
}

TEST(Shot, LongPlayFactorRunsFromSixtyToOneHundredThirtyPercent)
{
    EXPECT_EQ(longPlayFactor(0), Fixed::ratio(60, 100));
    EXPECT_EQ(longPlayFactor(100), Fixed::ratio(130, 100));
    EXPECT_NEAR(longPlayFactor(50).toDouble(), 0.95, 1e-3);
    EXPECT_EQ(longPlayFactor(200), longPlayFactor(100)); // clamped
}

TEST(Shot, FatigueDependsOnEnergyAndFoolproof)
{
    EXPECT_EQ(fatigue(255, 0), Fixed{});
    EXPECT_NEAR(fatigue(0, 0).toDouble(), 1.0, 1e-3);
    EXPECT_NEAR(fatigue(0, 50).toDouble(), 0.5, 1e-3);
    EXPECT_EQ(fatigue(0, 100), Fixed{});
    EXPECT_GT(fatigue(50, 20), fatigue(50, 80));
}

TEST(Shot, DistanceScalesWithPowerSkillLieAndFatigue)
{
    auto base = request(ClubId::iron7);
    const auto full = shotDistance(base);
    EXPECT_NEAR(unitsToYards(full).toDouble(), 140 * 0.95, 0.2);

    auto half = base;
    half.power = Fixed::ratio(1, 2);
    EXPECT_NEAR(shotDistance(half).toDouble(), full.toDouble() / 2, 0.5);

    EXPECT_GT(shotDistance(request(ClubId::iron7, 100)), full);
    EXPECT_LT(shotDistance(request(ClubId::iron7, 0)), full);

    auto rough = base;
    rough.lie.distanceFactor = Fixed::ratio(85, 100);
    EXPECT_NEAR(shotDistance(rough).toDouble(), full.toDouble() * 0.85, 0.5);

    auto tired = base;
    tired.energy = 0;
    tired.skills.foolproof = 0;
    EXPECT_NEAR(shotDistance(tired).toDouble(), full.toDouble() * 0.8, 0.5);
    tired.skills.foolproof = 100;
    EXPECT_EQ(shotDistance(tired), full); // Foolproof makes fatigue irrelevant

    auto equipped = base;
    equipped.equipmentLevel = 3;
    EXPECT_GT(shotDistance(equipped), full);

    auto overpowered = base;
    overpowered.power = Fixed::fromInt(3);
    EXPECT_EQ(shotDistance(overpowered), full); // power is clamped to 1
}

TEST(Shot, SpreadShrinksWithAccuracyAndLieAndGrowsWithFatigue)
{
    const auto base = request(ClubId::iron7, 50, 50);
    EXPECT_LT(directionSpread(request(ClubId::iron7, 50, 90)), directionSpread(base));
    EXPECT_LT(distanceSpread(request(ClubId::iron7, 50, 90)), distanceSpread(base));

    auto goodLie = base;
    goodLie.lie.accuracyFactor = Fixed::fromInt(1);
    auto badLie = base;
    badLie.lie.accuracyFactor = Fixed::ratio(5, 10);
    EXPECT_GT(directionSpread(badLie), directionSpread(goodLie) * 3 / 2);

    auto tired = base;
    tired.energy = 20;
    tired.skills.foolproof = 0;
    EXPECT_GT(directionSpread(tired), directionSpread(base) * 2);
    tired.skills.foolproof = 100;
    EXPECT_EQ(directionSpread(tired), directionSpread(base)); // unless Foolproof is high

    // Long clubs are less forgiving than short ones; putters are the most precise.
    EXPECT_GT(directionSpread(request(ClubId::driver)), directionSpread(request(ClubId::sandWedge)));
    EXPECT_LT(directionSpread(request(ClubId::putter)), directionSpread(request(ClubId::sandWedge)));
}

TEST(Shot, EveryClubCarriesItsBaseDistanceInTheSimulation)
{
    // BAL-020: "tune constants so that real carry ~ requested carry". The mean of many shots matches within 3%.
    for (const auto& club : allClubs())
    {
        if (club.isPutter)
            continue;
        const auto r = request(club.id, 50, 100);
        const double wanted = shotDistance(r).toDouble();
        double total = 0;
        constexpr int kShots = 60;
        for (int i = 0; i < kShots; i++)
            total += firstImpact(r, 1000 + i);
        EXPECT_NEAR(total / kShots / wanted, 1.0, 0.03) << club.name;
    }
}

TEST(Shot, HighLongPlayGolfersHitFurther)
{
    auto mean = [&](int longPlay) {
        double total = 0;
        for (int i = 0; i < 30; i++)
            total += firstImpact(request(ClubId::driver, longPlay, 100), 50 + i);
        return total / 30;
    };
    EXPECT_GT(mean(100), mean(50) * 1.15);
    EXPECT_GT(mean(50), mean(0) * 1.3);
}

TEST(Shot, AccuracyDecidesTheSpreadOfTheLandingPoints)
{
    auto spread = [&](int accuracy) {
        double sum = 0, sumSq = 0;
        constexpr int kShots = 80;
        for (int i = 0; i < kShots; i++)
        {
            double side = 0;
            firstImpact(request(ClubId::iron7, 50, accuracy), 300 + i, &side);
            sum += side;
            sumSq += side * side;
        }
        const double mean = sum / kShots;
        return std::sqrt(sumSq / kShots - mean * mean);
    };
    const auto poor = spread(0);
    const auto good = spread(100);
    EXPECT_GT(poor, good * 3);
    EXPECT_GT(good, 0.0);
}

TEST(Shot, ErrorsAreCentredOnTheAim)
{
    double sum = 0;
    constexpr int kShots = 200;
    for (int i = 0; i < kShots; i++)
    {
        double side = 0;
        firstImpact(request(ClubId::driver, 50, 30), 7000 + i, &side);
        sum += side;
    }
    // No systematic hook or slice: the mean miss is small next to the 20 unit spread of a driver.
    EXPECT_LT(std::fabs(sum / kShots), 12.0);
}

TEST(Shot, TheAimDirectionIsHonoured)
{
    TestWorld world;
    SeqRandom rng(3);
    auto r = request(ClubId::iron7, 50, 100);
    r.aimX = Fixed::fromInt(0);
    r.aimY = Fixed::fromInt(5); // straight towards +y
    Ball ball;
    placeBall(ball, world, Fixed::fromInt(100), Fixed::fromInt(100));
    applyShot(ball, computeShot(r, rng));
    while (ball.state == BallState::InFlight)
        stepBall(ball, world, rng);
    EXPECT_GT(ball.position.y - Fixed::fromInt(100), Fixed::fromInt(250));
    EXPECT_LT(abs(ball.position.x - Fixed::fromInt(100)), Fixed::fromInt(40));
}

TEST(Shot, ADegenerateAimStillFires)
{
    SeqRandom rng(1);
    auto r = request(ClubId::iron7);
    r.aimX = {};
    r.aimY = {};
    const auto shot = computeShot(r, rng);
    EXPECT_GT(lengthXY(shot.velocity), Fixed{});
}

TEST(Shot, SameSeedGivesTheSameShot)
{
    SeqRandom a(42), b(42), c(43);
    const auto r = request(ClubId::wood3);
    const auto first = computeShot(r, a);
    const auto second = computeShot(r, b);
    const auto other = computeShot(r, c);
    EXPECT_EQ(first.velocity, second.velocity);
    EXPECT_EQ(first.sideSpin, second.sideSpin);
    EXPECT_NE(first.velocity, other.velocity);
}

TEST(Shot, PuttsRollTheRequestedDistanceOnTheGroundSurface)
{
    for (auto roughness : { Fixed::ratio(5, 100), Fixed::ratio(15, 100) })
    {
        double total = 0;
        constexpr int kShots = 30;
        for (int i = 0; i < kShots; i++)
        {
            TestWorld world;
            world.surface.roughness = roughness;
            SeqRandom rng(900 + i);
            auto r = request(ClubId::putter, 50, 100);
            r.surfaceRoughness = roughness;
            r.power = Fixed::ratio(1, 2);
            Ball ball;
            placeBall(ball, world, Fixed::fromInt(100), Fixed::fromInt(100));
            const auto shot = computeShot(r, rng);
            EXPECT_TRUE(shot.isPutt);
            EXPECT_EQ(shot.backSpin, Fixed{});
            applyShot(ball, shot);
            EXPECT_EQ(ball.state, BallState::Rolling);
            for (int t = 0; t < 4000 && !isSettled(ball.state); t++)
                stepBall(ball, world, rng);
            total += (ball.position.x - Fixed::fromInt(100)).toDouble();
        }
        const double wanted = shotDistance([&] {
            auto r = request(ClubId::putter, 50, 100);
            r.power = Fixed::ratio(1, 2);
            return r;
        }()).toDouble();
        EXPECT_NEAR(total / kShots / wanted, 1.0, 0.06) << roughness.toDouble();
    }
}

TEST(Shot, ApplyingAShotRemembersTheOriginAndStartsTheRightState)
{
    TestWorld world;
    SeqRandom rng;
    Ball ball;
    placeBall(ball, world, Fixed::fromInt(50), Fixed::fromInt(60));
    ball.bounces = 5;
    applyShot(ball, computeShot(request(ClubId::driver), rng));
    EXPECT_EQ(ball.state, BallState::InFlight);
    EXPECT_EQ(ball.shotOrigin.x, Fixed::fromInt(50));
    EXPECT_EQ(ball.shotOrigin.y, Fixed::fromInt(60));
    EXPECT_EQ(ball.bounces, 0);
    EXPECT_GT(ball.velocity.z, Fixed{});
    EXPECT_GT(ball.backSpin, Fixed{});
}
