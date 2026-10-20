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
#include <openrct2/golf/Rules.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Golf;
using namespace GolfTest;

namespace
{
    SurfaceSample roughSurface(int roughnessPercent, bool hazard = false)
    {
        SurfaceSample s;
        s.roughness = Fixed::ratio(roughnessPercent, 100);
        s.hazard = hazard;
        return s;
    }
} // namespace

TEST(GolfRules, TheLieComesFromTheSurfaceObject)
{
    SurfaceSample sand;
    sand.distanceFactor = Fixed::ratio(55, 100);
    sand.accuracyFactor = Fixed::ratio(75, 100);
    const auto lie = lieOf(sand);
    EXPECT_EQ(lie.distanceFactor, Fixed::ratio(55, 100));
    EXPECT_EQ(lie.accuracyFactor, Fixed::ratio(75, 100));
}

TEST(GolfRules, PathsAndStreetsAreHardpan)
{
    SurfaceSample path;
    path.path = true;
    path.distanceFactor = Fixed::fromInt(1);
    const auto lie = lieOf(path);
    EXPECT_EQ(lie.distanceFactor, Fixed::ratio(90, 100));
    EXPECT_EQ(lie.accuracyFactor, Fixed::ratio(80, 100));
}

TEST(GolfRules, OnlyTheGreenIsPutted)
{
    SurfaceSample green;
    green.puttable = true;
    EXPECT_TRUE(mustPutt(green));
    EXPECT_FALSE(mustPutt(SurfaceSample{}));
}

TEST(GolfRules, SandStopsABallMuchFasterThanFairway)
{
    // Section 15 acceptance: the lie and the surface together. Roll a ball at the same speed over each surface.
    auto rollDistance = [](int roughnessPercent) {
        TestWorld world;
        world.surface.roughness = Fixed::ratio(roughnessPercent, 100);
        SeqRandom rng;
        Ball ball;
        placeBall(ball, world, Fixed::fromInt(100), Fixed::fromInt(100), BallState::Rolling);
        ball.velocity = vec(2, 0, 0);
        for (int t = 0; t < 4000 && !isSettled(ball.state); t++)
            stepBall(ball, world, rng);
        return (ball.position.x - Fixed::fromInt(100)).toDouble();
    };
    EXPECT_GT(rollDistance(15), rollDistance(90) * 5);
}

TEST(GolfRules, WaterCostsAStrokeAndDropsAtTheEdge)
{
    TestWorld world;
    world.waterFromX = Fixed::fromInt(300);
    SeqRandom rng;
    Ball ball;
    placeBall(ball, world, Fixed::fromInt(100), Fixed::fromInt(100));
    ball.position.z = Fixed::fromInt(1);
    ball.velocity = vec(8, 0, 6);
    ball.state = BallState::InFlight;
    while (!isSettled(ball.state))
        stepBall(ball, world, rng);
    ASSERT_EQ(ball.state, BallState::Lost);

    const auto outcome = judgeShot(ball, world);
    EXPECT_EQ(outcome.kind, ShotOutcome::Kind::penalty);
    EXPECT_EQ(outcome.penalty, Penalty::water);
    EXPECT_EQ(outcome.penaltyStrokes, 1);
    EXPECT_FALSE(outcome.replayFromPrevious);
    // The drop is on dry land just short of the water.
    EXPECT_LT(outcome.dropPosition.x, Fixed::fromInt(300));
    EXPECT_GT(outcome.dropPosition.x, Fixed::fromInt(200));
    EXPECT_FALSE(world.surfaceAt(outcome.dropPosition.x, outcome.dropPosition.y).water);
}

TEST(GolfRules, AnInvalidDropPointFallsBackToThePreviousShot)
{
    TestWorld world;
    world.waterFromX = Fixed::fromInt(0); // water everywhere beyond x = 0 ... the entry point itself is wet
    Ball ball;
    ball.state = BallState::Lost;
    ball.hazardEntry = vec(50, 50, 0);
    ball.shotOrigin = vec(-100, 50, 0);
    const auto outcome = judgeShot(ball, world);
    EXPECT_EQ(outcome.penalty, Penalty::water);
    EXPECT_TRUE(outcome.replayFromPrevious);
    EXPECT_EQ(outcome.dropPosition, ball.shotOrigin);
}

TEST(GolfRules, OutOfBoundsReplaysFromThePreviousPositionWithAStroke)
{
    TestWorld world;
    world.outOfBoundsFromX = Fixed::fromInt(300);
    SeqRandom rng;
    Ball ball;
    placeBall(ball, world, Fixed::fromInt(100), Fixed::fromInt(100));
    ball.position.z = Fixed::fromInt(1);
    ball.velocity = vec(8, 0, 6);
    ball.state = BallState::InFlight;
    while (!isSettled(ball.state))
        stepBall(ball, world, rng);
    ASSERT_EQ(ball.state, BallState::OutOfBounds);
    const auto outcome = judgeShot(ball, world);
    EXPECT_EQ(outcome.penalty, Penalty::outOfBounds);
    EXPECT_EQ(outcome.penaltyStrokes, 1);
    EXPECT_TRUE(outcome.replayFromPrevious);
    EXPECT_EQ(outcome.dropPosition, ball.shotOrigin);
}

TEST(GolfRules, AHoledBallEndsTheHoleAndARestingBallIsInPlay)
{
    TestWorld world;
    Ball holed;
    holed.state = BallState::Holed;
    EXPECT_EQ(judgeShot(holed, world).kind, ShotOutcome::Kind::holed);

    Ball resting;
    resting.state = BallState::AtRest;
    resting.position = vec(100, 100, 0);
    const auto outcome = judgeShot(resting, world);
    EXPECT_EQ(outcome.kind, ShotOutcome::Kind::inPlay);
    EXPECT_EQ(outcome.penalty, Penalty::none);
    EXPECT_FALSE(outcome.needsSearch);

    Ball flying;
    flying.state = BallState::InFlight;
    EXPECT_EQ(judgeShot(flying, world).kind, ShotOutcome::Kind::inPlay); // not stopped yet: nothing to judge
}

TEST(GolfRules, ABallThatRestsInWaterOrOutOfBoundsIsPenalised)
{
    TestWorld world;
    world.outOfBoundsFromX = Fixed::fromInt(300);
    Ball ball;
    ball.state = BallState::AtRest;
    ball.position = vec(310, 100, 0);
    ball.shotOrigin = vec(100, 100, 0);
    EXPECT_EQ(judgeShot(ball, world).penalty, Penalty::outOfBounds);
}

TEST(GolfRules, LostBallChanceDependsOnTheRoughness)
{
    EXPECT_EQ(lostBallChancePercent(roughSurface(15)), 0);   // fairway
    EXPECT_EQ(lostBallChancePercent(roughSurface(45)), 0);   // rough
    EXPECT_GT(lostBallChancePercent(roughSurface(75)), 0);   // deep rough
    EXPECT_GT(lostBallChancePercent(roughSurface(80)), lostBallChancePercent(roughSurface(75))); // woods
    EXPECT_EQ(lostBallChancePercent(roughSurface(90, true)), 0); // sand is a hazard: the ball is visible
    EXPECT_LE(lostBallChancePercent(roughSurface(100)), kMaxLostBallChancePercent);
    SurfaceSample water = roughSurface(80);
    water.water = true;
    EXPECT_EQ(lostBallChancePercent(water), 0);
}

TEST(GolfRules, SearchingFindsTheBallMostOfTheTimeInDeepRoughButNotAlways)
{
    SeqRandom rng(11);
    int found = 0;
    constexpr int kSearches = 2000;
    const auto woods = roughSurface(80);
    for (int i = 0; i < kSearches; i++)
        found += searchFindsBall(woods, rng) ? 1 : 0;
    const double expected = 1.0 - lostBallChancePercent(woods) / 100.0;
    EXPECT_NEAR(static_cast<double>(found) / kSearches, expected, 0.05);
    EXPECT_LT(found, kSearches);
    // On a fairway the ball is always found.
    for (int i = 0; i < 100; i++)
        EXPECT_TRUE(searchFindsBall(roughSurface(15), rng));
    EXPECT_EQ(kSearchTicks, 2400); // one game minute at 40 ticks per second
}

TEST(GolfRules, ALostBallCostsAStrokeAndIsPlayedAgainFromTheLastShot)
{
    Ball ball;
    ball.shotOrigin = vec(40, 60, 8);
    const auto outcome = lostBallOutcome(ball);
    EXPECT_EQ(outcome.penalty, Penalty::lostBall);
    EXPECT_EQ(outcome.penaltyStrokes, 1);
    EXPECT_TRUE(outcome.replayFromPrevious);
    EXPECT_EQ(outcome.dropPosition, ball.shotOrigin);
}

TEST(GolfRules, TheHoleEndsAtParPlusFourStrokes)
{
    EXPECT_EQ(maxStrokes(3), 7);
    EXPECT_EQ(maxStrokes(5), 9);
    HoleScore score;
    score.par = 4;
    ShotOutcome water;
    water.kind = ShotOutcome::Kind::penalty;
    water.penaltyStrokes = 1;
    for (int i = 0; i < 4; i++)
    {
        score.addShot();
        EXPECT_FALSE(score.apply({}));
    }
    // Shots 5 and 6 each land in the water: 6 shots + 2 penalties = 8 strokes = par + 4.
    score.addShot();
    EXPECT_FALSE(score.apply(water));
    EXPECT_EQ(score.strokes(), 6);
    score.addShot();
    EXPECT_TRUE(score.apply(water));
    EXPECT_EQ(score.strokes(), 8);
    EXPECT_TRUE(score.pickedUp);
    EXPECT_TRUE(score.finished());
    EXPECT_EQ(score.toPar(), 4);
}

TEST(GolfRules, HolingOutOnTheLastAllowedStrokeCountsAsHoled)
{
    HoleScore score;
    score.par = 3;
    for (int i = 0; i < 7; i++)
        score.addShot();
    ShotOutcome holed;
    holed.kind = ShotOutcome::Kind::holed;
    EXPECT_FALSE(score.apply(holed));
    EXPECT_TRUE(score.holed);
    EXPECT_FALSE(score.pickedUp);
    EXPECT_EQ(score.toPar(), 4);
}

TEST(GolfRules, ScoreNames)
{
    EXPECT_EQ(scoreName(1, 4), ScoreName::holeInOne);
    EXPECT_EQ(scoreName(2, 5), ScoreName::albatross);
    EXPECT_EQ(scoreName(2, 4), ScoreName::eagle);
    EXPECT_EQ(scoreName(3, 4), ScoreName::birdie);
    EXPECT_EQ(scoreName(4, 4), ScoreName::par);
    EXPECT_EQ(scoreName(5, 4), ScoreName::bogey);
    EXPECT_EQ(scoreName(6, 4), ScoreName::doubleBogey);
    EXPECT_EQ(scoreName(8, 4), ScoreName::worse);
}
