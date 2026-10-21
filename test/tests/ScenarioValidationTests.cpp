/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <gtest/gtest.h>
#include <openrct2/GameState.h>
#include <openrct2/golf/CourseManager.h>
#include <openrct2/golf/Hole.h>
#include <openrct2/golf/ScenarioValidation.h>
#include <openrct2/golf/StreetNetwork.h>
#include <openrct2/golf/Traffic.h>
#include <openrct2/world/Map.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Golf;

TEST(MapSize, DefaultNewMapSizeIs256x256)
{
    // TER-001: Default new map size 256x256 tiles (~1280x1280 yd)
    EXPECT_EQ(kDefaultMapSize.x, 256);
    EXPECT_EQ(kDefaultMapSize.y, 256);

    // 1 tile = 32 world units = 5 yards
    constexpr int32_t kYardsPerTile = 5;
    EXPECT_EQ(kDefaultMapSize.x * kYardsPerTile, 1280);
    EXPECT_EQ(kDefaultMapSize.y * kYardsPerTile, 1280);
}

TEST(ScenarioValidation, EmptyScenarioGeneratesAllWarnings)
{
    // SAV-003: warn if no road entrance, no parking, no 18 holes
    auto& gameState = getGameState();
    gameState.golf.courses().clear();

    const auto report = validateScenario(gameState, nullptr, nullptr);
    EXPECT_FALSE(report.passed);
    EXPECT_TRUE(report.hasWarning(ScenarioWarning::noRoadEntrance));
    EXPECT_TRUE(report.hasWarning(ScenarioWarning::noParking));
    EXPECT_TRUE(report.hasWarning(ScenarioWarning::fewerThan18Holes));
    EXPECT_FALSE(report.hasWarning(ScenarioWarning::holePathDisconnected));
}

TEST(ScenarioValidation, RoadEntranceClearsEntranceWarning)
{
    auto& gameState = getGameState();
    StreetNetwork streets;
    StreetTile tile;
    tile.position = { 0, 10 };
    tile.connections = 0b0001;
    streets.setTile(tile);
    streets.setRoadEntrance({ 0, 10 }, true, { 256, 256 });

    const auto report = validateScenario(gameState, &streets, nullptr);
    EXPECT_FALSE(report.hasWarning(ScenarioWarning::noRoadEntrance));
    EXPECT_TRUE(report.hasWarning(ScenarioWarning::noParking));
}

TEST(ScenarioValidation, ParkingLotsClearParkingWarning)
{
    auto& gameState = getGameState();
    StreetNetwork streets;
    TrafficSimulation traffic(&streets);
    auto& lot = traffic.addLot({ 10, 10 }, { 10, 11 });
    uint32_t nextId = 1;
    lot.addTile({ 10, 10 }, 0, nextId);

    const auto report = validateScenario(gameState, nullptr, &traffic);
    EXPECT_FALSE(report.hasWarning(ScenarioWarning::noParking));
    EXPECT_TRUE(report.hasWarning(ScenarioWarning::noRoadEntrance));
}

TEST(ScenarioValidation, EighteenHoleCourseAndConnectivityPasses)
{
    // SAV-003: 18-hole course with valid hole transitions passes
    auto& gameState = getGameState();
    gameState.golf.courses().clear();
    auto* course = gameState.golf.createCourse("Championship Links");
    ASSERT_NE(course, nullptr);

    // Build 18 consecutive holes, spaced 40 yards apart from cup to next tee
    for (int i = 0; i < 18; ++i)
    {
        Hole hole;
        hole.id = static_cast<uint32_t>(i + 1);
        // 1 tile = 32 world units = 5 yards. 40 yards = 8 tiles = 256 units.
        int32_t teeX = 1000 + i * 1500;
        int32_t cupX = teeX + 1200; // 1200 units = 37.5 tiles = 187.5 yards hole
        hole.tees[EnumValue(TeeCategory::men)] = HolePoint{ teeX, 2000 };
        hole.cup = HolePoint{ cupX, 2000 };
        course->holes.push_back(hole);
    }
    ASSERT_EQ(course->playableHoles(), 18u);

    StreetNetwork streets;
    StreetTile tile;
    tile.position = { 0, 10 };
    tile.connections = 0b0001;
    streets.setTile(tile);
    streets.setRoadEntrance({ 0, 10 }, true, { 256, 256 });

    TrafficSimulation traffic(&streets);
    auto& lot = traffic.addLot({ 10, 10 }, { 10, 11 });
    uint32_t nextId = 1;
    lot.addTile({ 10, 10 }, 0, nextId);

    const auto report = validateScenario(gameState, &streets, &traffic, 120);
    EXPECT_TRUE(report.passed);
    EXPECT_FALSE(report.hasWarning(ScenarioWarning::noRoadEntrance));
    EXPECT_FALSE(report.hasWarning(ScenarioWarning::noParking));
    EXPECT_FALSE(report.hasWarning(ScenarioWarning::fewerThan18Holes));
    EXPECT_FALSE(report.hasWarning(ScenarioWarning::holePathDisconnected));
}

TEST(ScenarioValidation, DisconnectedHolesWarnsWhenCupFarFromNextTee)
{
    // SAV-003: warn if holes have no path to next tee
    auto& gameState = getGameState();
    gameState.golf.courses().clear();
    auto* course = gameState.golf.createCourse("Links");
    ASSERT_NE(course, nullptr);

    Hole h1;
    h1.id = 1;
    h1.tees[EnumValue(TeeCategory::men)] = HolePoint{ 1000, 1000 };
    h1.cup = HolePoint{ 2000, 1000 };
    course->holes.push_back(h1);

    // Hole 2 tee is far away (8000 units = 250 tiles = 1250 yards)
    Hole h2;
    h2.id = 2;
    h2.tees[EnumValue(TeeCategory::men)] = HolePoint{ 10000, 1000 };
    h2.cup = HolePoint{ 11000, 1000 };
    course->holes.push_back(h2);

    const auto report = validateScenario(gameState, nullptr, nullptr, 120);
    EXPECT_TRUE(report.hasWarning(ScenarioWarning::holePathDisconnected));

    bool foundHole1Issue = false;
    for (const auto& issue : report.issues)
    {
        if (issue.warning == ScenarioWarning::holePathDisconnected && issue.holeNumber == 1)
        {
            foundHole1Issue = true;
            EXPECT_NE(issue.description.find("Hole 1 cup is too far"), std::string::npos);
        }
    }
    EXPECT_TRUE(foundHole1Issue);
}
