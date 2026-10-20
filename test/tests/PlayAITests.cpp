/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "GolfTestWorld.h"

#include <chrono>
#include <cmath>
#include <gtest/gtest.h>
#include <openrct2/golf/PlayAI.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Golf;
using namespace GolfTest;

namespace
{
    constexpr int kTeeX = 320, kY = 3200;

    // A straight hole along +x with the tee at x = 320.
    PlayContext contextAt(const FunctionWorld& world, double ballX, int yards, double ballY = kY)
    {
        PlayContext c;
        c.world = &world;
        c.ball = vec(static_cast<int>(ballX), static_cast<int>(ballY), 0);
        const int cupX = kTeeX + yards * 32 / 5;
        c.line = { HolePoint{ kTeeX, kY }, HolePoint{ cupX, kY } };
        c.cup = vec(cupX, kY, 0);
        return c;
    }

    double yardsOf(Fixed units)
    {
        return unitsToYards(units).toDouble();
    }
} // namespace

TEST(PlayAI, ATeeShotOnALongOpenHoleIsADriveStraightAtTheGreen)
{
    FunctionWorld world;
    auto c = contextAt(world, kTeeX, 400);
    c.teeShot = true;
    const auto d = decideShot(c);
    EXPECT_FALSE(d.putt);
    EXPECT_TRUE(d.club == ClubId::driver || d.club == ClubId::wood3) << static_cast<int>(d.club);
    EXPECT_GE(d.power.toDouble(), 0.85);
    EXPECT_NEAR(d.aimX.toDouble(), 1.0, 0.02);
    EXPECT_NEAR(d.aimY.toDouble(), 0.0, 0.02);
    EXPECT_GT(yardsOf(d.expectedDistance), 180);
}

TEST(PlayAI, ShorterApproachesUseShorterClubsAtAboutFullSwing)
{
    FunctionWorld world;
    double lastCarry = 1e9;
    for (int yards : { 200, 150, 120, 90, 60 })
    {
        auto c = contextAt(world, kTeeX, yards + 150);
        c.ball = vec(kTeeX + 150 * 32 / 5, kY, 0); // yards from the cup
        const auto d = decideShot(c);
        ASSERT_FALSE(d.putt);
        ShotRequest r;
        r.club = d.club;
        const double carry = yardsOf(shotDistance(r));
        // The club reaches the cup with a swing between 30% and a full one, and the hole is not shot past.
        EXPECT_GE(carry, yards * 0.99) << yards;
        EXPECT_GE(d.power.toDouble(), 0.7) << yards; // and the closer the better the fit: near a full swing
        // Closer holes never get a longer club.
        EXPECT_LE(carry, lastCarry + 0.01) << yards;
        lastCarry = carry;
        // The shot is aimed at the cup.
        EXPECT_NEAR(d.aimX.toDouble(), 1.0, 0.02);
    }
}

TEST(PlayAI, WithinReachTheShotTargetsTheCup)
{
    FunctionWorld world;
    auto c = contextAt(world, kTeeX + 60 * 32 / 5, 100); // 40 yards left
    const auto d = decideShot(c);
    EXPECT_NEAR(d.target.x.toDouble(), c.cup.x.toDouble(), 1.0);
    EXPECT_NEAR(yardsOf(d.expectedDistance), 40, 1.5);
}

TEST(PlayAI, ALakeAtTheLandingZoneMakesACarefulGolferLayUp)
{
    FunctionWorld lake;
    // Water across the fairway 200 to 245 yards from the tee, wide enough that an aimed drive would land in it.
    lake.hazard = [](double x, double) { return x > kTeeX + 200 * 6.4 && x < kTeeX + 245 * 6.4; };
    auto careful = contextAt(lake, kTeeX, 420);
    careful.riskTolerance = Fixed{};
    careful.skills = { 50, 50, 50, 50 };
    const auto layUp = decideShot(careful);
    ShotRequest r;
    r.club = layUp.club;
    r.skills = careful.skills;
    EXPECT_LT(yardsOf(shotDistance(r)), 200) << "carry of the chosen club";
    EXPECT_FALSE(layUp.club == ClubId::driver);

    FunctionWorld open;
    auto same = contextAt(open, kTeeX, 420);
    same.riskTolerance = Fixed{};
    EXPECT_TRUE(decideShot(same).club == ClubId::driver || decideShot(same).club == ClubId::wood3);
}

TEST(PlayAI, RecklessGolfersTakeMoreRiskThanCarefulOnes)
{
    FunctionWorld lake;
    lake.hazard = [](double x, double) { return x > kTeeX + 200 * 6.4 && x < kTeeX + 215 * 6.4; };
    auto reach = [&](Fixed risk) {
        auto c = contextAt(lake, kTeeX, 420);
        c.riskTolerance = risk;
        return yardsOf(decideShot(c).expectedDistance);
    };
    EXPECT_GE(reach(Fixed::fromInt(1)), reach(Fixed{}));
}

TEST(PlayAI, SkillsChangeTheChoice)
{
    FunctionWorld world;
    auto weak = contextAt(world, kTeeX, 300);
    weak.skills = { 0, 0, 0, 0 };
    auto strong = contextAt(world, kTeeX, 300);
    strong.skills = { 100, 100, 100, 100 };
    ShotRequest rw;
    rw.club = decideShot(weak).club;
    rw.skills = weak.skills;
    ShotRequest rs;
    rs.club = decideShot(strong).club;
    rs.skills = strong.skills;
    // A weak golfer needs more club than a strong one to cover the same ground.
    EXPECT_LE(static_cast<int>(decideShot(weak).club), static_cast<int>(decideShot(strong).club));
}

TEST(PlayAI, ADoglegAimsAlongTheBentLine)
{
    FunctionWorld world;
    PlayContext c;
    c.world = &world;
    c.ball = vec(kTeeX, kY, 0);
    // 100 yards along +x, then the hole turns 90 degrees towards +y for 300 yards.
    const int bendX = kTeeX + 100 * 32 / 5;
    c.line = { HolePoint{ kTeeX, kY }, HolePoint{ bendX, kY }, HolePoint{ bendX, kY + 300 * 32 / 5 } };
    c.cup = vec(bendX, kY + 300 * 32 / 5, 0);
    c.teeShot = true;
    const auto d = decideShot(c);
    // The target is round the corner, so the shot turns towards +y a little but is still mostly down the first leg.
    EXPECT_GT(d.target.y.toDouble(), kY + 100);
    EXPECT_GT(d.aimX.toDouble(), 0.3);
    EXPECT_GT(d.aimY.toDouble(), 0.3);
    EXPECT_NEAR(d.target.x.toDouble(), bendX, 1.0);
}

TEST(PlayAI, OnTheGreenTheGolferPutts)
{
    FunctionWorld green;
    green.puttable = [](double, double) { return true; };
    green.roughness = [](double, double) { return 0.05; };
    auto c = contextAt(green, kTeeX + 90 * 32 / 5, 100); // 10 yards away
    const auto d = decideShot(c);
    EXPECT_TRUE(d.putt);
    EXPECT_EQ(d.club, ClubId::putter);
    EXPECT_GT(d.power.toDouble(), 0.1);
    EXPECT_LT(d.power.toDouble(), 1.0);
    // The putt is meant to run past the cup: further than the distance to it.
    const double toCup = 10 * 6.4;
    EXPECT_GT(d.expectedDistance.toDouble(), toCup);
    EXPECT_LT(d.expectedDistance.toDouble(), toCup * 1.5);
    EXPECT_NEAR(d.aimX.toDouble(), 1.0, 0.001);
}

TEST(PlayAI, PuttsUphillGetMoreAndDownhillLess)
{
    auto putt = [](double slope) {
        FunctionWorld green;
        green.puttable = [](double, double) { return true; };
        green.roughness = [](double, double) { return 0.05; };
        green.height = [slope](double x, double) { return x * slope; };
        // The ball 30 yards... no, 5 yards (32 units) from the cup, along +x.
        auto c = contextAt(green, kTeeX + 68 * 32 / 5, 100);
        c.ball.z = Fixed::fromRaw(static_cast<int64_t>(green.height(c.ball.x.toDouble(), kY) * 65536));
        c.cup.z = Fixed::fromRaw(static_cast<int64_t>(green.height(c.cup.x.toDouble(), kY) * 65536));
        return decideShot(c).expectedDistance.toDouble();
    };
    const double flat = putt(0.0);
    EXPECT_GT(putt(0.02), flat);   // rising towards the cup
    EXPECT_LT(putt(-0.02), flat);  // falling
}

TEST(PlayAI, PuttsReadTheBreakAccordingToAccuracy)
{
    // The ground falls towards -y (to the left of a putt along +x? left is +y for a heading of +x), so the ball drifts.
    auto aimY = [](int accuracy, double slopeY) {
        FunctionWorld green;
        green.puttable = [](double, double) { return true; };
        green.roughness = [](double, double) { return 0.05; };
        green.height = [slopeY](double, double y) { return y * slopeY; };
        auto c = contextAt(green, kTeeX + 60 * 32 / 5, 100);
        c.skills.accuracy = static_cast<uint8_t>(accuracy);
        return decideShot(c).aimY.toDouble();
    };
    // Height rising with y means the ground falls towards -y, which is to the right of a putt heading +x: the ball breaks
    // right, so the golfer aims left (+y).
    EXPECT_GT(aimY(100, 0.03), 0.02);
    EXPECT_LT(aimY(100, -0.03), -0.02);
    // A better player reads more of the break.
    EXPECT_GT(aimY(100, 0.03), aimY(30, 0.03));
    EXPECT_NEAR(aimY(0, 0.03), 0.0, 1e-3);
    EXPECT_NEAR(aimY(100, 0.0), 0.0, 1e-3);
}

TEST(PlayAI, TheBreakOfAPuttIsTheSidewaysSlope)
{
    FunctionWorld flat;
    EXPECT_EQ(puttBreak(flat, vec(0, 0, 0), vec(320, 0, 0)), Fixed{});
    FunctionWorld tilted;
    tilted.height = [](double, double y) { return -y * 0.04; }; // falls towards +y = the left of a putt along +x
    EXPECT_NEAR(puttBreak(tilted, vec(0, 0, 0), vec(320, 0, 0)).toDouble(), 0.04, 0.005);
    // Seen from the other end the same slope is to the right.
    EXPECT_NEAR(puttBreak(tilted, vec(320, 0, 0), vec(0, 0, 0)).toDouble(), -0.04, 0.005);
    EXPECT_EQ(puttBreak(flat, vec(5, 5, 0), vec(5, 5, 0)), Fixed{});
}

TEST(PlayAI, WithoutAWorldTheDecisionIsHarmless)
{
    PlayContext c;
    const auto d = decideShot(c);
    EXPECT_FALSE(d.putt);
}

TEST(PlayAI, DecisionsAreDeterministic)
{
    FunctionWorld world;
    world.hazard = [](double x, double y) { return x > 1800 && std::fabs(y - kY) < 200; };
    auto c = contextAt(world, kTeeX, 420);
    const auto a = decideShot(c);
    const auto b = decideShot(c);
    EXPECT_EQ(a.club, b.club);
    EXPECT_EQ(a.power, b.power);
    EXPECT_EQ(a.target, b.target);
}

TEST(PlayAI, ADecisionTakesWellUnderHalfAMillisecond)
{
    // HOL-032, with the most expensive case: a hazard-laden world.
    FunctionWorld world;
    world.hazard = [](double x, double y) { return (static_cast<int>(x / 50) + static_cast<int>(y / 50)) % 7 == 0; };
    auto c = contextAt(world, kTeeX, 420);
    constexpr int kRuns = 400;
    const auto start = std::chrono::steady_clock::now();
    int64_t sink = 0;
    for (int i = 0; i < kRuns; i++)
    {
        c.ball = vec(kTeeX + i * 4, kY, 0);
        sink += decideShot(c).power.raw();
    }
    const auto elapsed = std::chrono::steady_clock::now() - start;
    const double perDecisionMs = std::chrono::duration<double, std::milli>(elapsed).count() / kRuns;
    EXPECT_LT(perDecisionMs, 0.5) << sink;
}
