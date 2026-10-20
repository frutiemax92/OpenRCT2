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
#include <functional>
#include <gtest/gtest.h>
#include <openrct2/golf/HoleDifficulty.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Golf;
using namespace GolfTest;

namespace
{
    // A world defined by functions, so each test describes the course it needs.
    class FunctionWorld final : public IBallWorld
    {
    public:
        std::function<double(double, double)> height = [](double, double) { return 0.0; };
        std::function<bool(double, double)> hazard = [](double, double) { return false; };
        std::function<bool(double, double)> blocked = [](double, double) { return false; };

        SurfaceSample surfaceAt(Fixed x, Fixed y) const override
        {
            SurfaceSample s;
            const double px = x.toDouble(), py = y.toDouble();
            s.height = Fixed::fromRaw(static_cast<int64_t>(height(px, py) * 65536.0));
            // Normal from finite differences of the height function.
            const double gx = (height(px + 1, py) - height(px - 1, py)) / 2;
            const double gy = (height(px, py + 1) - height(px, py - 1)) / 2;
            s.normal = normalised(FixedVec3{ Fixed::fromRaw(static_cast<int64_t>(-gx * 65536.0)),
                                             Fixed::fromRaw(static_cast<int64_t>(-gy * 65536.0)), Fixed::fromInt(1) });
            s.hazard = hazard(px, py);
            return s;
        }
        bool obstacleAt(const FixedVec3& p, Fixed, ObstacleContact&) const override
        {
            return blocked(p.x.toDouble(), p.y.toDouble());
        }
    };

    Hole holeOfYards(int yards)
    {
        Hole h;
        h.id = 1;
        h.tees[0] = HolePoint{ 320, 3200 };
        h.cup = HolePoint{ 320 + yards * 32 / 5, 3200 };
        return h;
    }
} // namespace

TEST(HoleDifficulty, IncompleteHolesHaveNone)
{
    FunctionWorld world;
    Hole h;
    EXPECT_EQ(rateHole(h, world).total, 0);
    h.tees[0] = HolePoint{ 0, 0 };
    EXPECT_EQ(rateHole(h, world).total, 0);
}

TEST(HoleDifficulty, AShortOpenFlatHoleIsEasy)
{
    FunctionWorld world;
    const auto d = rateHole(holeOfYards(150), world);
    EXPECT_EQ(d.hazards, 0);
    EXPECT_EQ(d.obstacles, 0);
    EXPECT_EQ(d.greenSlope, 0);
    EXPECT_EQ(d.elevation, 0);
    EXPECT_LE(d.total, 15);
    EXPECT_LT(d.rating(), 2.0);
}

TEST(HoleDifficulty, LongHolesForTheirParAreHarder)
{
    FunctionWorld world;
    // Par 3 holes: the longer one is closer to the upper end of the table.
    EXPECT_GT(rateHole(holeOfYards(240), world).length, rateHole(holeOfYards(120), world).length);
    // The same distance is harder as a par 3 than as a par 5's lay-up distance.
    EXPECT_GT(rateHole(holeOfYards(250), world).length, rateHole(holeOfYards(251), world).length);
    EXPECT_EQ(rateHole(holeOfYards(250), world).length, kMaxLengthDifficulty);
    // Length never exceeds its share of the total.
    EXPECT_LE(rateHole(holeOfYards(900), world).length, kMaxLengthDifficulty);
}

TEST(HoleDifficulty, HazardsBesideTheLineRaiseTheRating)
{
    FunctionWorld clear;
    FunctionWorld lake;
    // A lake along one side of the fairway, 2 tiles from the line.
    lake.hazard = [](double, double y) { return y > 3200 + 64 && y < 3200 + 160; };
    const auto before = rateHole(holeOfYards(380), clear);
    const auto after = rateHole(holeOfYards(380), lake);
    EXPECT_GT(after.hazards, 8);
    EXPECT_GT(after.total, before.total);
    EXPECT_EQ(before.hazards, 0);

    // Hazards far from the line do not count.
    FunctionWorld far;
    far.hazard = [](double, double y) { return y > 3200 + 32 * 10; };
    EXPECT_EQ(rateHole(holeOfYards(380), far).hazards, 0);

    // Everything beside the line being a hazard maxes out the part.
    FunctionWorld all;
    all.hazard = [](double, double y) { return y != 3200; };
    EXPECT_EQ(rateHole(holeOfYards(380), all).hazards, kMaxHazardDifficulty);
}

TEST(HoleDifficulty, ObstaclesOnTheLineRaiseTheRating)
{
    FunctionWorld trees;
    // A line of trees across the middle of the fairway.
    trees.blocked = [](double x, double y) { return x > 1500 && x < 1560 && y > 3150 && y < 3250; };
    const auto d = rateHole(holeOfYards(380), trees);
    EXPECT_GT(d.obstacles, 0);
    EXPECT_GT(d.total, rateHole(holeOfYards(380), FunctionWorld{}).total);
}

TEST(HoleDifficulty, SteepGreensAreHarder)
{
    FunctionWorld flat;
    FunctionWorld tilted;
    // A 10% slope across the whole course.
    tilted.height = [](double x, double) { return x * 0.1; };
    const auto flatGreen = rateHole(holeOfYards(300), flat);
    const auto steep = rateHole(holeOfYards(300), tilted);
    EXPECT_EQ(flatGreen.greenSlope, 0);
    EXPECT_GT(steep.greenSlope, 10);
    EXPECT_LE(steep.greenSlope, kMaxGreenSlopeDifficulty);
}

TEST(HoleDifficulty, ElevationChangeAlongTheLineCounts)
{
    FunctionWorld hilly;
    hilly.height = [](double x, double) { return 24.0 * std::sin(x / 120.0); };
    const auto d = rateHole(holeOfYards(380), hilly);
    EXPECT_GT(d.elevation, 3);
    EXPECT_EQ(rateHole(holeOfYards(380), FunctionWorld{}).elevation, 0);
}

TEST(HoleDifficulty, TheTotalIsCappedAtTen)
{
    FunctionWorld awful;
    awful.hazard = [](double, double y) { return y != 3200; };
    awful.blocked = [](double x, double) { return x > 1000; };
    awful.height = [](double x, double) { return x * 0.1 + 16.0 * std::sin(x / 50.0); };
    const auto d = rateHole(holeOfYards(600), awful);
    EXPECT_EQ(d.total, 100);
    EXPECT_DOUBLE_EQ(d.rating(), 10.0);
    EXPECT_GE(d.length + d.hazards + d.greenSlope + d.obstacles + d.elevation, d.total);
}

TEST(HoleDifficulty, DoglegsAreRatedAlongTheBentLine)
{
    FunctionWorld lake;
    lake.hazard = [](double x, double y) { return x > 1400 && x < 1500 && y > 3200 + 64; };
    Hole straight = holeOfYards(300);
    Hole bent = straight;
    bent.aimPoints[0] = HolePoint{ 1500, 3200 + 320 };
    // The bend goes into the lake's area, so more of its corridor is wet.
    EXPECT_GT(rateHole(bent, lake).hazards, rateHole(straight, lake).hazards);
}

TEST(HoleDifficulty, GolferLevelIsTheAverageSkill)
{
    GolferSkills s;
    s.longPlay = 20;
    s.accuracy = 40;
    s.foolproof = 60;
    s.physique = 80;
    EXPECT_EQ(golferLevelTenths(s), 50);
    EXPECT_EQ(golferLevelTenths(GolferSkills{ 0, 0, 0, 0 }), 0);
    EXPECT_EQ(golferLevelTenths(GolferSkills{ 100, 100, 100, 100 }), 100);
}

TEST(HoleDifficulty, ATooHardHoleLowersHappiness)
{
    // Acceptance criterion: a too-hard hole is flagged for the golfer.
    HoleDifficulty hard;
    hard.total = 85;
    HoleDifficulty easy;
    easy.total = 20;
    const GolferSkills beginner{ 10, 10, 10, 10 };
    const GolferSkills veteran{ 90, 90, 90, 90 };
    EXPECT_TRUE(isTooHard(hard, beginner));
    EXPECT_FALSE(isTooHard(hard, veteran));
    EXPECT_FALSE(isTooHard(easy, beginner));
    // Right on the threshold is not too hard yet.
    HoleDifficulty edge;
    edge.total = 10 + kTooHardThresholdTenths;
    EXPECT_FALSE(isTooHard(edge, beginner));
    edge.total++;
    EXPECT_TRUE(isTooHard(edge, beginner));
    EXPECT_GT(kTooHardHappinessLoss, 0);
}
