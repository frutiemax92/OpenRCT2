/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <gtest/gtest.h>
#include <openrct2/golf/Hole.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Golf;

namespace
{
    // A straight hole along +x: `tiles` tiles from the tee to the cup.
    Hole straightHole(int tiles, uint32_t id = 1)
    {
        Hole h;
        h.id = id;
        h.tees[0] = HolePoint{ 10 * 32, 10 * 32 };
        h.cup = HolePoint{ (10 + tiles) * 32, 10 * 32 };
        return h;
    }
}

TEST(GolfHole, ParFollowsTheLengthTables)
{
    // Men: par 3 up to 260, par 4 up to 490, par 5 up to 710, par 6 beyond.
    EXPECT_EQ(parForLength(0), 3);
    EXPECT_EQ(parForLength(260), 3);
    EXPECT_EQ(parForLength(261), 4);
    EXPECT_EQ(parForLength(490), 4);
    EXPECT_EQ(parForLength(491), 5);
    EXPECT_EQ(parForLength(710), 5);
    EXPECT_EQ(parForLength(711), 6);
    // Women and children have shorter thresholds.
    EXPECT_EQ(parForLength(300, TeeCategory::men), 4);
    EXPECT_EQ(parForLength(300, TeeCategory::women), 4);
    EXPECT_EQ(parForLength(220, TeeCategory::women), 3);
    EXPECT_EQ(parForLength(221, TeeCategory::women), 4);
    EXPECT_EQ(parForLength(420, TeeCategory::women), 4);
    EXPECT_EQ(parForLength(450, TeeCategory::women), 5);
    EXPECT_EQ(parForLength(601, TeeCategory::women), 6);
    EXPECT_EQ(parForLength(250, TeeCategory::child), 4);
    EXPECT_EQ(parForLength(150, TeeCategory::child), 3);
    EXPECT_EQ(parForLength(290, TeeCategory::child), 5);
}

TEST(GolfHole, DistancesAreInYardsFiveToATile)
{
    EXPECT_EQ(distanceYards({ 0, 0 }, { 32, 0 }), 5);
    EXPECT_EQ(distanceYards({ 0, 0 }, { 0, 32 * 80 }), 400);
    EXPECT_EQ(distanceYards({ 0, 0 }, { 32 * 3, 32 * 4 }), 25); // 3-4-5 triangle: 5 tiles
    EXPECT_EQ(distanceYards({ 100, 100 }, { 100, 100 }), 0);
    EXPECT_EQ(distanceYards({ 0, 0 }, { 32 * 3, 32 * 4 }), distanceYards({ 32 * 3, 32 * 4 }, { 0, 0 }));
}

TEST(GolfHole, AFourHundredYardHoleIsPar4)
{
    // Acceptance criterion: a 400-yard hole (80 tiles) shows par 4.
    const auto hole = straightHole(80);
    EXPECT_EQ(hole.lengthYards(TeeCategory::men), 400);
    EXPECT_EQ(hole.par(), 4);
    EXPECT_EQ(hole.computedPar(), 4);
    EXPECT_EQ(straightHole(40).par(), 3);   // 200 yards
    EXPECT_EQ(straightHole(52).par(), 3);   // exactly 260
    EXPECT_EQ(straightHole(53).par(), 4);   // 265
    EXPECT_EQ(straightHole(98).par(), 4);   // exactly 490
    EXPECT_EQ(straightHole(100).par(), 5);  // 500
}

TEST(GolfHole, NinetyNineTilesIsPar5)
{
    EXPECT_EQ(straightHole(98).par(), 4);  // 490
    EXPECT_EQ(straightHole(99).par(), 5);  // 495
    EXPECT_EQ(straightHole(142).par(), 5); // 710
    EXPECT_EQ(straightHole(143).par(), 6); // 715
}

TEST(GolfHole, AimPointsBendThePlayingLine)
{
    Hole h = straightHole(60); // 300 yards straight
    EXPECT_EQ(h.lengthYards(TeeCategory::men), 300);
    // A dogleg through a point 30 tiles along and 40 tiles to the side: 50 + 50 tiles = 100 tiles = 500 yards.
    h.aimPoints[0] = HolePoint{ (10 + 30) * 32, (10 + 40) * 32 };
    h.cup = HolePoint{ (10 + 60) * 32, 10 * 32 };
    const auto line = h.playingLine(TeeCategory::men);
    ASSERT_EQ(line.size(), 3u);
    EXPECT_EQ(line[1], *h.aimPoints[0]);
    EXPECT_EQ(line[2], *h.cup);
    EXPECT_NEAR(h.lengthYards(TeeCategory::men), 5 * (50 + 50), 2);
    EXPECT_EQ(h.par(), 5);
    // Two aim points are used in order.
    h.aimPoints[1] = HolePoint{ (10 + 50) * 32, (10 + 40) * 32 };
    EXPECT_EQ(h.playingLine(TeeCategory::men).size(), 4u);
    // The first aim point is the one that counts when the second is missing.
    h.aimPoints[1].reset();
    EXPECT_EQ(h.playingLine(TeeCategory::men).size(), 3u);
}

TEST(GolfHole, ParCanBeOverridden)
{
    Hole h = straightHole(80);
    h.parOverride = 5;
    EXPECT_EQ(h.par(), 5);
    EXPECT_EQ(h.computedPar(), 4);
    h.parOverride = 0;
    EXPECT_EQ(h.par(), 4);
}

TEST(GolfHole, AHoleNeedsATeeAndACup)
{
    Hole h;
    EXPECT_FALSE(h.isComplete());
    EXPECT_EQ(h.par(), 0);
    EXPECT_EQ(h.lengthYards(TeeCategory::men), 0);
    h.tees[1] = HolePoint{ 320, 320 };
    EXPECT_FALSE(h.isComplete());
    h.cup = HolePoint{ 320 * 3, 320 };
    EXPECT_TRUE(h.isComplete()); // any single tee will do
    EXPECT_EQ(h.firstTee(), TeeCategory::women);
    EXPECT_EQ(h.lengthYards(TeeCategory::men), 0); // no men's tee: nothing to measure from
    EXPECT_GT(h.lengthYards(TeeCategory::women), 0);
    EXPECT_GT(h.par(), 0); // par comes from the first tee that exists
    h.tees[0] = HolePoint{ 100, 320 };
    EXPECT_EQ(h.firstTee(), TeeCategory::men);
    h.cup.reset();
    EXPECT_FALSE(h.isComplete());
}

TEST(GolfHole, TeesOfDifferentCategoriesHaveTheirOwnLengths)
{
    Hole h = straightHole(80);
    h.tees[1] = HolePoint{ (10 + 10) * 32, 10 * 32 }; // forward tee: 70 tiles
    h.tees[2] = HolePoint{ (10 + 40) * 32, 10 * 32 }; // children: 40 tiles
    EXPECT_EQ(h.lengthYards(TeeCategory::men), 400);
    EXPECT_EQ(h.lengthYards(TeeCategory::women), 350);
    EXPECT_EQ(h.lengthYards(TeeCategory::child), 200);
    EXPECT_EQ(h.par(), 4); // always the men's tee
}

TEST(GolfCourse, HolesAreNumberedInOrderAndCanBeReordered)
{
    Course c;
    for (uint32_t i = 1; i <= 4; i++)
        c.holes.push_back(straightHole(40 + static_cast<int>(i), i * 10));
    EXPECT_EQ(c.holeNumber(10), 1u);
    EXPECT_EQ(c.holeNumber(40), 4u);
    EXPECT_EQ(c.holeNumber(99), 0u);

    EXPECT_TRUE(c.moveHole(3, 0)); // the last hole becomes the first
    EXPECT_EQ(c.holeNumber(40), 1u);
    EXPECT_EQ(c.holeNumber(10), 2u);
    EXPECT_EQ(c.holeNumber(30), 4u);
    EXPECT_TRUE(c.moveHole(0, 2));
    EXPECT_EQ(c.holeNumber(40), 3u);
    EXPECT_TRUE(c.moveHole(1, 1));
    EXPECT_FALSE(c.moveHole(0, 4));
    EXPECT_FALSE(c.moveHole(9, 0));
    // The ids did not change, only the order.
    EXPECT_NE(c.findHole(20), nullptr);
    EXPECT_EQ(c.findHole(20)->id, 20u);
    EXPECT_EQ(c.findHole(77), nullptr);
}

TEST(GolfCourse, TotalsAndLimits)
{
    Course c;
    EXPECT_EQ(c.totalPar(), 0);
    c.holes.push_back(straightHole(40, 1)); // par 3
    c.holes.push_back(straightHole(80, 2)); // par 4
    c.holes.push_back(straightHole(100, 3)); // par 5
    Hole incomplete;
    incomplete.id = 4;
    incomplete.tees[0] = HolePoint{ 0, 0 };
    c.holes.push_back(incomplete);
    EXPECT_EQ(c.totalPar(), 12);
    EXPECT_EQ(c.totalYards(TeeCategory::men), 200 + 400 + 500);
    EXPECT_EQ(c.playableHoles(), 3u);
    EXPECT_FALSE(c.isFull());
    for (uint32_t i = 5; c.holes.size() < kMaxHolesPerCourse; i++)
        c.holes.push_back(straightHole(40, i));
    EXPECT_TRUE(c.isFull());
    EXPECT_EQ(kMaxHolesPerCourse, 18u);
}
