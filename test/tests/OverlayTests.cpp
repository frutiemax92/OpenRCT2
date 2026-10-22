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
#include <openrct2/golf/CourseManager.h>
#include <openrct2/golf/Hole.h>
#include <openrct2/golf/Overlays.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Golf;
using namespace GolfTest;

// UI-001: Overlays: hole playing line, ball trajectories (last N shots),
// heat map of landing zones, lie map (colour per surface), wait-time map.

TEST(GolfOverlayTests, HolePlayingLineReturnsTeeAimPointsAndCupInOrder)
{
    Hole hole;
    hole.tees[EnumValue(TeeCategory::men)] = HolePoint{ 100, 200 };
    hole.aimPoints = { HolePoint{ 300, 250 }, HolePoint{ 500, 300 } };
    hole.cup = HolePoint{ 700, 350 };

    const auto line = GolfOverlayManager::getPlayingLine(hole, TeeCategory::men);
    ASSERT_EQ(line.size(), 4u);
    EXPECT_EQ(line[0], (HolePoint{ 100, 200 }));
    EXPECT_EQ(line[1], (HolePoint{ 300, 250 }));
    EXPECT_EQ(line[2], (HolePoint{ 500, 300 }));
    EXPECT_EQ(line[3], (HolePoint{ 700, 350 }));
}

TEST(GolfOverlayTests, BallTrajectoriesTracksLastNShots)
{
    GolfOverlayManager manager;
    manager.setMaxTrajectories(3);
    EXPECT_EQ(manager.maxTrajectories(), 3u);

    for (uint32_t i = 1; i <= 5; i++)
    {
        ShotTrajectory traj;
        traj.shotId = i;
        traj.golferId = 100 + i;
        traj.club = ClubId::driver;
        traj.points = { vec(0, 0, 0), vec(100, 0, 50), vec(200 + i * 10, 0, 0) };
        traj.landingPosition = vec(200 + i * 10, 0, 0);
        manager.recordShotTrajectory(traj);
    }

    const auto& recents = manager.getRecentTrajectories();
    ASSERT_EQ(recents.size(), 3u);
    // Oldest 1 and 2 discarded, 3, 4, 5 kept
    EXPECT_EQ(recents[0].shotId, 3u);
    EXPECT_EQ(recents[1].shotId, 4u);
    EXPECT_EQ(recents[2].shotId, 5u);
}

TEST(GolfOverlayTests, LandingHeatMapTracksLandingFrequenciesAndIntensity)
{
    GolfOverlayManager manager;
    const TileCoordsXY tileA{ 10, 10 };
    const TileCoordsXY tileB{ 12, 10 };

    EXPECT_EQ(manager.getLandingCount(tileA), 0u);
    EXPECT_EQ(manager.getHeatIntensity(tileA), 0u);

    // 3 landings on A, 1 on B
    manager.recordLanding(tileA);
    manager.recordLanding(tileA);
    manager.recordLanding(tileA);
    manager.recordLanding(tileB);

    EXPECT_EQ(manager.getLandingCount(tileA), 3u);
    EXPECT_EQ(manager.getLandingCount(tileB), 1u);

    // Maximum count tile gets 255
    EXPECT_EQ(manager.getHeatIntensity(tileA), 255u);
    // Tile B gets ~ 1/3 of 255 = 85
    EXPECT_EQ(manager.getHeatIntensity(tileB), 85u);
}

TEST(GolfOverlayTests, LieMapClassifiesSurfaceAndColoursCorrectly)
{
    SurfaceSample sample{};
    // Putting surface
    sample.puttable = true;
    EXPECT_EQ(GolfOverlayManager::classifyLie(sample), LieColourCategory::green);
    EXPECT_EQ(GolfOverlayManager::getLieColour(sample), Drawing::Colour::brightGreen);

    // Fairway
    sample.puttable = false;
    sample.roughness = Fixed::ratio(10, 100);
    EXPECT_EQ(GolfOverlayManager::classifyLie(sample), LieColourCategory::fairway);
    EXPECT_EQ(GolfOverlayManager::getLieColour(sample), Drawing::Colour::darkGreen);

    // Rough
    sample.roughness = Fixed::ratio(30, 100);
    EXPECT_EQ(GolfOverlayManager::classifyLie(sample), LieColourCategory::rough);
    EXPECT_EQ(GolfOverlayManager::getLieColour(sample), Drawing::Colour::oliveGreen);

    // Sand bunker
    sample.hazard = true;
    EXPECT_EQ(GolfOverlayManager::classifyLie(sample), LieColourCategory::sand);
    EXPECT_EQ(GolfOverlayManager::getLieColour(sample), Drawing::Colour::yellow);

    // Water hazard
    sample.water = true;
    EXPECT_EQ(GolfOverlayManager::classifyLie(sample), LieColourCategory::water);
    EXPECT_EQ(GolfOverlayManager::getLieColour(sample), Drawing::Colour::lightBlue);

    // Hardpan / Path
    sample.water = false;
    sample.hazard = false;
    sample.path = true;
    EXPECT_EQ(GolfOverlayManager::classifyLie(sample), LieColourCategory::hardpan);
    EXPECT_EQ(GolfOverlayManager::getLieColour(sample), Drawing::Colour::grey);

    // Out of bounds
    sample.outOfBounds = true;
    EXPECT_EQ(GolfOverlayManager::classifyLie(sample), LieColourCategory::outOfBounds);
    EXPECT_EQ(GolfOverlayManager::getLieColour(sample), Drawing::Colour::brightRed);
}

TEST(GolfOverlayTests, WaitTimeMapClassifiesWaitTimesAndColoursCorrectly)
{
    EXPECT_EQ(GolfOverlayManager::classifyWaitTime(1), GolfOverlayManager::WaitLevel::clear);
    EXPECT_EQ(GolfOverlayManager::getWaitLevelColour(GolfOverlayManager::WaitLevel::clear), Drawing::Colour::brightGreen);

    EXPECT_EQ(GolfOverlayManager::classifyWaitTime(4), GolfOverlayManager::WaitLevel::moderate);
    EXPECT_EQ(GolfOverlayManager::getWaitLevelColour(GolfOverlayManager::WaitLevel::moderate), Drawing::Colour::brightYellow);

    EXPECT_EQ(GolfOverlayManager::classifyWaitTime(7), GolfOverlayManager::WaitLevel::busy);
    EXPECT_EQ(GolfOverlayManager::getWaitLevelColour(GolfOverlayManager::WaitLevel::busy), Drawing::Colour::lightOrange);

    EXPECT_EQ(GolfOverlayManager::classifyWaitTime(12), GolfOverlayManager::WaitLevel::bottleneck);
    EXPECT_EQ(GolfOverlayManager::getWaitLevelColour(GolfOverlayManager::WaitLevel::bottleneck), Drawing::Colour::brightRed);
}
