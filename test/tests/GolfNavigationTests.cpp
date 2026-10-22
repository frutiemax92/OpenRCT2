/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <gtest/gtest.h>
#include <openrct2/Context.h>
#include <openrct2/GameState.h>
#include <openrct2/OpenRCT2.h>
#include <openrct2/actions/ResultWithMessage.h>
#include <openrct2/drawing/Colour.h>
#include <openrct2/golf/GolfRuntime.h>
#include <openrct2/entity/GolferEntity.h>
#include <openrct2/object/DefaultObjects.h>
#include <openrct2/object/ObjectManager.h>
#include <openrct2/peep/PeepAnimations.h>
#include <openrct2/scenes/editor/EditorController.h>
#include <openrct2/world/Map.h>
#include <openrct2/world/Park.h>
#include <openrct2/world/tile_element/SurfaceElement.h>

using namespace OpenRCT2;

class GolfNavigationTests : public testing::Test
{
protected:
    void SetUp() override
    {
        gOpenRCT2Headless = true;
        gOpenRCT2NoGraphics = true;
        _context = CreateContext();
        ASSERT_TRUE(_context->Initialise());
    }

    void TearDown() override
    {
        _context = nullptr;
    }

    std::unique_ptr<IContext> _context;
};

// NAV-001: Auto-load all golf objects in DefaultObjects
TEST_F(GolfNavigationTests, DefaultGolfObjectsArrayContainsAllCoreGolfObjects)
{
    EXPECT_EQ(kCommonScenarioAndTrackDesignerObjects.size(), 40u);

    bool hasClubHouse = false;
    bool hasRoad = false;
    bool hasCartRental = false;
    bool hasBallWasher = false;
    bool hasPathSign = false;

    for (const auto& id : kCommonScenarioAndTrackDesignerObjects)
    {
        EXPECT_TRUE(id.starts_with("golf."));
        if (id == "golf.building.club_house")
            hasClubHouse = true;
        if (id == "golf.street.road")
            hasRoad = true;
        if (id == "golf.building.cart_rental")
            hasCartRental = true;
        if (id == "golf.path_addition.ball_washer")
            hasBallWasher = true;
        if (id == "golf.path_addition.path_sign")
            hasPathSign = true;
    }

    EXPECT_TRUE(hasClubHouse);
    EXPECT_TRUE(hasRoad);
    EXPECT_TRUE(hasCartRental);
    EXPECT_TRUE(hasBallWasher);
    EXPECT_TRUE(hasPathSign);
}

// The footpath window refuses to open without a footpath surface and railings, so the defaults must provide them.
TEST_F(GolfNavigationTests, DefaultObjectsLoadAFootpathSurfaceAndRailings)
{
    auto& objectManager = _context->GetObjectManager();
    for (const auto& entry : kCommonScenarioAndTrackDesignerObjects)
        objectManager.LoadObject(entry);

    EXPECT_NE(objectManager.GetLoadedObjectEntryIndex("golf.footpath_surface.gravel"), kObjectEntryIndexNull);
    EXPECT_NE(objectManager.GetLoadedObjectEntryIndex("golf.footpath_surface.tee_queue"), kObjectEntryIndexNull);
    EXPECT_NE(objectManager.GetLoadedObjectEntryIndex("golf.footpath_railings.none"), kObjectEntryIndexNull);
}

// Hiring a greenskeeper and opening the staff window read the handyman sprite animations, so the defaults must have them.
TEST_F(GolfNavigationTests, DefaultObjectsLoadTheHandymanAnimations)
{
    auto& objectManager = _context->GetObjectManager();
    for (const auto& entry : kCommonScenarioAndTrackDesignerObjects)
        objectManager.LoadObject(entry);

    EXPECT_NE(findPeepAnimationsIndexForType(AnimationPeepType::handyman), kObjectEntryIndexNull);
}

// A golfer over the ball looks along the shot, not the way it last walked; without an aim it looks the way it walks.
TEST_F(GolfNavigationTests, AGolferPlayingAShotFacesTheAimAndOtherwiseItsWalk)
{
    auto* golfer = GolferEntity::create(1, { 640, 640, 0 });
    ASSERT_NE(golfer, nullptr);
    const CoordsXY alongPlusY{ 0, 500 };
    golfer->follow({ 640, 640, 0 }, Golf::GolferState::swinging, &alongPlusY);
    EXPECT_EQ(golfer->yaw, 192);
    // Walking +x, far enough to count, turns it again.
    golfer->follow({ 700, 640, 0 }, Golf::GolferState::walkingToBall);
    EXPECT_EQ(golfer->yaw, 128);
}

// NAV-004: Editor::CheckPark succeeds on golf courses without legacy theme-park gates
TEST_F(GolfNavigationTests, EditorCheckParkSucceedsWithoutThemeParkEntrance)
{
    auto& gameState = getGameState();
    MapInit({ 100, 100 }, Drawing::Colour::black);

    // If all tiles are unowned, UpdateSize returns 0 and CheckPark returns false
    for (int32_t y = 0; y < 100; y++)
    {
        for (int32_t x = 0; x < 100; x++)
        {
            auto* surface = MapGetSurfaceElementAt(TileCoordsXY{ x, y });
            if (surface != nullptr)
            {
                surface->setOwnership(kUnowned);
            }
        }
    }
    Park::UpdateSize(gameState.park);
    auto resultNoLand = Editor::CheckPark();
    EXPECT_FALSE(resultNoLand.Successful);

    // Own some land
    for (int32_t y = 1; y < 10; y++)
    {
        for (int32_t x = 1; x < 10; x++)
        {
            auto* surface = MapGetSurfaceElementAt(TileCoordsXY{ x, y });
            if (surface != nullptr)
            {
                surface->setOwnership(OwnershipFlag::landOwned);
            }
        }
    }
    Park::UpdateSize(gameState.park);

    // Even with empty entrances and peep spawns, golf courses pass CheckPark!
    EXPECT_TRUE(gameState.park.entrances.empty());
    EXPECT_TRUE(gameState.peepSpawns.empty());
    auto resultWithLand = Editor::CheckPark();
    EXPECT_TRUE(resultWithLand.Successful);
}

// NAV-005: Empty Golf Sandbox Setup
TEST_F(GolfNavigationTests, SetupEmptyGolfSandboxInitialisesMapAndObjects)
{
    auto& gameState = getGameState();
    Golf::SetupEmptyGolfSandbox(gameState);

    EXPECT_EQ(gameState.mapSize.x, 150);
    EXPECT_EQ(gameState.mapSize.y, 150);
    EXPECT_EQ(gameState.park.cash, 100000.00_GBP);
    EXPECT_TRUE(gameState.park.flags.has(ParkFlag::parkOpen));
    EXPECT_TRUE(gameState.park.flags.has(ParkFlag::freeEntry));

    // Verify land is owned
    auto* surface = MapGetSurfaceElementAt(TileCoordsXY{ 75, 75 });
    ASSERT_NE(surface, nullptr);
    EXPECT_TRUE(surface->hasOwnership(OwnershipFlag::landOwned));
}

// NAV-005: Playable Demo Golf Course Setup
TEST_F(GolfNavigationTests, SetupDemoGolfCourseInitialisesPlayableCourseWithTrafficAndClub)
{
    auto& gameState = getGameState();
    Golf::SetupDemoGolfCourse(gameState);

    EXPECT_EQ(gameState.park.cash, 50000.00_GBP);
    EXPECT_TRUE(gameState.park.flags.has(ParkFlag::parkOpen));

    // Verify buildings scan finds the clubhouse
    auto scan = Golf::scanBuildings();
    EXPECT_TRUE(scan.hasClub);

    // Verify regular and mini-putt courses exist
    const auto& courses = gameState.golf.courses();
    ASSERT_GE(courses.size(), 2u);

    const auto* regular = gameState.golf.findCourse(courses[0].id);
    ASSERT_NE(regular, nullptr);
    EXPECT_EQ(regular->kind, Golf::CourseKind::regular);
    EXPECT_EQ(regular->holes.size(), 3u);
    EXPECT_EQ(regular->greenFee, 2500);

    const auto* mini = gameState.golf.findCourse(courses[1].id);
    ASSERT_NE(mini, nullptr);
    EXPECT_EQ(mini->kind, Golf::CourseKind::miniPutt);
    EXPECT_EQ(mini->holes.size(), 1u);
    EXPECT_EQ(mini->greenFee, 800);

    // Verify driving range and lots exist
    EXPECT_FALSE(gameState.golf.ranges().empty());
    EXPECT_FALSE(gameState.golf.lots().empty());

    // Update golf runtime and check active simulation
    Golf::updateGolf(gameState);
    ASSERT_NE(gameState.golfRuntime, nullptr);
    EXPECT_TRUE(gameState.golfRuntime->active());
    EXPECT_TRUE(gameState.golfRuntime->hasCarTraffic());
}
