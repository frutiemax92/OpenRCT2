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
#include <openrct2/golf/MapBallWorld.h>
#include <openrct2/golf/Shot.h>
#include <openrct2/world/Weather.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Weather;
using OpenRCT2::Golf::applyShot;
using OpenRCT2::Golf::computeShot;
using OpenRCT2::Golf::ShotRequest;
using OpenRCT2::Golf::ClubId;

TEST(Wind, EveryWeatherHasASensibleStrengthRange)
{
    for (int i = 0; i < kNumWeatherTypes; i++)
    {
        const auto range = windRangeFor(static_cast<Type>(i));
        EXPECT_LE(range.min, range.max) << i;
        EXPECT_LE(range.max, kWindStrengthMax) << i;
    }
    // Storms blow harder than sunny days.
    EXPECT_GT(windRangeFor(Type::thunder).min, windRangeFor(Type::sunny).max);
    EXPECT_GT(windRangeFor(Type::blizzard).min, windRangeFor(Type::partiallyCloudy).max);
    EXPECT_GT(windRangeFor(Type::heavyRain).max, windRangeFor(Type::rain).max);
}

TEST(Wind, StepsMoveTowardsTheTargetAtALimitedRate)
{
    WindState wind;
    wind.direction = 10;
    wind.targetDirection = 100;
    wind.strength = 0;
    wind.targetStrength = 500;
    stepWind(wind, Type::sunny);
    EXPECT_EQ(wind.direction, 13);
    EXPECT_EQ(wind.strength, kWindStrengthStep);
    EXPECT_EQ(wind.targetDirection, 100); // not there yet: the target stays
    EXPECT_EQ(wind.targetStrength, 500);
}

TEST(Wind, TheDirectionTurnsTheShortWayRoundTheCircle)
{
    WindState wind;
    wind.direction = 350;
    wind.targetDirection = 10;
    wind.strength = wind.targetStrength = 100;
    stepWind(wind, Type::sunny);
    EXPECT_EQ(wind.direction, 353); // through 0, not back through 180
    wind.direction = 10;
    wind.targetDirection = 350;
    stepWind(wind, Type::sunny);
    EXPECT_EQ(wind.direction, 7);
}

TEST(Wind, ArrivingPicksANewTargetFromTheWeather)
{
    WindState wind;
    wind.direction = wind.targetDirection = 45;
    wind.strength = wind.targetStrength = 300;
    stepWind(wind, Type::thunder);
    const auto range = windRangeFor(Type::thunder);
    EXPECT_GE(wind.targetStrength, range.min);
    EXPECT_LE(wind.targetStrength, range.max);
    // The new direction is within a quarter turn of the old one.
    int diff = std::abs(wind.targetDirection - 45);
    diff = std::min(diff, 360 - diff);
    EXPECT_LE(diff, 90);
}

TEST(Wind, ItStaysWithinTheRangeOfTheWeatherOverAWholeSeason)
{
    WindState wind;
    const auto range = windRangeFor(Type::rain);
    int changes = 0;
    int16_t last = wind.direction;
    for (uint32_t tick = 0; tick < 40u * 60 * 60 * 24 * 7; tick++)
    {
        updateWind(wind, Type::rain, tick);
        EXPECT_LE(wind.strength, kWindStrengthMax);
        EXPECT_LT(wind.direction, 360);
        EXPECT_GE(wind.direction, 0);
        if (tick > 20000)
        {
            EXPECT_GE(wind.targetStrength, std::min<int>(range.min, windRangeFor(Type::sunny).min));
        }
        if (wind.direction != last)
            changes++;
        last = wind.direction;
    }
    EXPECT_GT(changes, 100);
    // After settling into rain, the strength sits inside the rain range for most of the time.
    int inside = 0;
    for (uint32_t tick = 0; tick < 128 * 400; tick++)
    {
        updateWind(wind, Type::rain, tick);
        if (wind.strength >= range.min - kWindStrengthStep && wind.strength <= range.max + kWindStrengthStep)
            inside++;
    }
    EXPECT_GT(inside, 128 * 400 * 9 / 10);
}

TEST(Wind, OnlyTheIntervalTicksStepTheWind)
{
    WindState wind;
    wind.targetStrength = 800;
    updateWind(wind, Type::sunny, 1);
    updateWind(wind, Type::sunny, 127);
    EXPECT_EQ(wind.strength, 0);
    updateWind(wind, Type::sunny, kWindUpdateInterval);
    EXPECT_GT(wind.strength, 0);
}

TEST(Wind, IsDeterministicAndIndependentOfTheScenarioRandomNumbers)
{
    auto run = [] {
        WindState wind;
        uint64_t hash = 0;
        for (uint32_t tick = 0; tick < 128 * 2000; tick++)
        {
            updateWind(wind, Type::cloudy, tick);
            hash = hash * 31 + wind.strength * 360u + static_cast<uint32_t>(wind.direction);
        }
        return hash;
    };
    EXPECT_EQ(run(), run());
}

TEST(Wind, CompassNamesFollowTheMapAxes)
{
    EXPECT_STREQ(windCompassName(0), "E");
    EXPECT_STREQ(windCompassName(90), "S");
    EXPECT_STREQ(windCompassName(180), "W");
    EXPECT_STREQ(windCompassName(270), "N");
    EXPECT_STREQ(windCompassName(45), "SE");
    EXPECT_STREQ(windCompassName(359), "E");
    EXPECT_STREQ(windCompassName(-90), "N");
}

TEST(Wind, TheAirVelocityPointsWhereTheWindBlows)
{
    WindState wind;
    wind.direction = 0;
    wind.strength = 1000;
    auto v = Golf::windVelocity(wind);
    EXPECT_NEAR(v.x.toDouble(), 1.5, 0.01);
    EXPECT_NEAR(v.y.toDouble(), 0.0, 0.01);
    wind.direction = 90;
    wind.strength = 500;
    v = Golf::windVelocity(wind);
    EXPECT_NEAR(v.x.toDouble(), 0.0, 0.01);
    EXPECT_NEAR(v.y.toDouble(), 0.75, 0.01);
    wind.strength = 0;
    EXPECT_EQ(Golf::windVelocity(wind), FixedVec3{});
}

TEST(Wind, AStrongWindMovesAShotOffLine)
{
    using namespace GolfTest;
    TestWorld calm;
    TestWorld windy;
    WindState wind;
    wind.direction = 90;
    wind.strength = 1000;
    windy.windVelocity = Golf::windVelocity(wind);
    SeqRandom rng(3);
    auto fly = [&](TestWorld& world) {
        Ball ball;
        placeBall(ball, world, Fixed::fromInt(100), Fixed::fromInt(500));
        ShotRequest request;
        request.club = ClubId::iron7;
        request.skills.accuracy = 100;
        class Perfect final : public IBallRandom
        {
        public:
            uint32_t next() override
            {
                return 0x00800000u;
            }
        } perfect;
        applyShot(ball, computeShot(request, perfect));
        while (ball.state == BallState::InFlight)
            stepBall(ball, world, rng);
        return ball.position;
    };
    const auto straight = fly(calm);
    const auto blown = fly(windy);
    EXPECT_NEAR(straight.y.toDouble(), 500.0, 0.5);
    EXPECT_GT(blown.y.toDouble(), 510.0); // pushed towards +y
}

TEST(Wind, WindCompassNamesAndPanelFormatting)
{
    WindState wind;
    wind.direction = 225;
    wind.strength = 650;
    std::string str = std::string(windCompassName(wind.direction)) + " " + std::to_string(wind.strength / 10) + "%";
    EXPECT_EQ(str, "NW 65%");

    wind.direction = 270;
    wind.strength = 1000;
    str = std::string(windCompassName(wind.direction)) + " " + std::to_string(wind.strength / 10) + "%";
    EXPECT_EQ(str, "N 100%");
}

