/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "GolfTestWorld.h"
#include "TestData.h"

#include <gtest/gtest.h>
#include <memory>
#include <openrct2/Context.h>
#include <openrct2/Game.h>
#include <openrct2/GameState.h>
#include <openrct2/OpenRCT2.h>
#include <openrct2/ParkImporter.h>
#include <openrct2/actions/GameActionRunner.h>
#include <openrct2/actions/golf/CourseBoundaryAction.h>
#include <openrct2/drawing/Drawing.h>
#include <openrct2/drawing/Palette.h>
#include <openrct2/entity/EntityList.h>
#include <openrct2/entity/EntityRegistry.h>
#include <openrct2/entity/EntityTweener.h>
#include <openrct2/entity/GolfBall.h>
#include <openrct2/golf/MapBallWorld.h>
#include <openrct2/object/ObjectManager.h>
#include <openrct2/world/Map.h>
#include <openrct2/world/MapAnimation.h>
#include <openrct2/world/MapOwnership.h>
#include <openrct2/world/tile_element/SurfaceElement.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Golf;
using namespace GolfTest;

namespace
{
    std::unique_ptr<IContext> startPark()
    {
        gOpenRCT2Headless = true;
        gOpenRCT2NoGraphics = true;

        auto context = CreateContext();
        if (!context->Initialise())
            return {};

        auto importer = ParkImporter::CreateParkFile(context->GetObjectRepository());
        auto loadResult = importer->LoadSavedGame(TestData::GetParkPath("small_park_with_ferris_wheel.park").c_str(), false);
        context->GetObjectManager().LoadObjects(loadResult.RequiredObjects);

        MapAnimations::ClearAll();
        auto& gameState = getGameState();
        importer->Import(gameState);
        gameState.entities.resetEntitySpatialIndices();
        ResetAllSpriteQuadrantPlacements();
        Drawing::LoadPalette();
        EntityTweener::get().reset();
        gGameSpeed = 1;
        gameState.golf.clear();
        gameState.mapSize = { 60, 60 };
        return context;
    }

    void ownOnly(int minTile, int maxTile)
    {
        for (int32_t y = 0; y < getGameState().mapSize.y; y++)
            for (int32_t x = 0; x < getGameState().mapSize.x; x++)
            {
                auto* surface = MapGetSurfaceElementAt(TileCoordsXY{ x, y });
                const bool inside = x >= minTile && x <= maxTile && y >= minTile && y <= maxTile;
                surface->setOwnership(inside ? OwnershipFlags{ OwnershipFlag::landOwned } : OwnershipFlags{});
            }
    }
} // namespace

TEST(CourseWorld, WithoutCoursesNothingIsOutOfBoundsExceptTheEdgeOfTheMap)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    ownOnly(10, 12);
    EXPECT_FALSE(isOutOfCourseBounds({ 50 * 32, 50 * 32 }));
}

TEST(CourseWorld, TheDefaultBoundaryIsAllOwnedLand)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    getGameState().golf.createCourse("Links");
    ownOnly(10, 12);
    EXPECT_FALSE(isOutOfCourseBounds({ 11 * 32 + 5, 11 * 32 + 5 }));
    EXPECT_TRUE(isOutOfCourseBounds({ 20 * 32, 11 * 32 }));
    EXPECT_TRUE(isOutOfCourseBounds({ 9 * 32 + 31, 11 * 32 }));
    EXPECT_TRUE(isTileOwned({ 12, 12 }));
    EXPECT_FALSE(isTileOwned({ 13, 12 }));
}

TEST(CourseWorld, ThePlayerDrawsTheBoundary)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    getGameState().golf.createCourse("Links");
    ownOnly(10, 14);
    auto& golf = getGameState().golf;

    // Taking tile row 12 out of play starts from the owned land and edits it.
    GameActions::CourseBoundaryAction cut({ 10 * 32, 12 * 32 }, { 14 * 32, 12 * 32 }, GameActions::CourseBoundaryAction::kOutOfPlay);
    ASSERT_EQ(GameActions::Query(&cut, getGameState()).error, GameActions::Status::ok);
    ASSERT_EQ(GameActions::ExecuteNested(&cut, getGameState()).error, GameActions::Status::ok);
    EXPECT_TRUE(golf.boundary().defined());
    EXPECT_TRUE(isOutOfCourseBounds({ 12 * 32, 12 * 32 + 8 }));
    EXPECT_FALSE(isOutOfCourseBounds({ 12 * 32, 11 * 32 + 8 }));
    EXPECT_TRUE(isOutOfCourseBounds({ 20 * 32, 11 * 32 })); // unowned land stayed out

    // Putting land outside the owned area in play is allowed too: the boundary is the player's choice.
    GameActions::CourseBoundaryAction add({ 20 * 32, 11 * 32 }, { 21 * 32, 11 * 32 }, GameActions::CourseBoundaryAction::kInPlay);
    ASSERT_EQ(GameActions::ExecuteNested(&add, getGameState()).error, GameActions::Status::ok);
    EXPECT_FALSE(isOutOfCourseBounds({ 20 * 32 + 4, 11 * 32 + 4 }));

    GameActions::CourseBoundaryAction reset({ 0, 0 }, { 0, 0 }, GameActions::CourseBoundaryAction::kReset);
    ASSERT_EQ(GameActions::ExecuteNested(&reset, getGameState()).error, GameActions::Status::ok);
    EXPECT_FALSE(golf.boundary().defined());
    EXPECT_FALSE(isOutOfCourseBounds({ 12 * 32, 12 * 32 + 8 }));

    GameActions::CourseBoundaryAction bad({ 0, 0 }, { 0, 0 }, 9);
    EXPECT_EQ(GameActions::Query(&bad, getGameState()).error, GameActions::Status::invalidParameters);
}

TEST(CourseWorld, ABallLeavingTheBoundaryIsOutOfBounds)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    getGameState().golf.createCourse("Links");
    ownOnly(10, 30);
    MapBallWorld world;
    world.useCourses();
    SeqRandom rng;
    Ball ball;
    placeBall(ball, world, Fixed::fromInt(12 * 32), Fixed::fromInt(20 * 32));
    ball.position.z += Fixed::fromInt(2);
    ball.velocity = vec(8, 0, 6);
    ball.state = BallState::InFlight;
    BallEvents seen;
    for (int t = 0; t < 4000 && !isSettled(ball.state); t++)
    {
        const auto ev = stepBall(ball, world, rng);
        seen.wentOutOfBounds |= ev.wentOutOfBounds;
    }
    // 8 units per tick for a long flight goes well past tile 30.
    EXPECT_EQ(ball.state, BallState::OutOfBounds);
    EXPECT_TRUE(seen.wentOutOfBounds);
}

TEST(CourseWorld, TheCupOfAHoleHoldsASlowBall)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& golf = getGameState().golf;
    const auto courseId = golf.createCourse("Links")->id;
    auto* hole = golf.createHole(courseId);
    hole->cup = HolePoint{ 20 * 32 + 16, 20 * 32 + 16 };
    ownOnly(0, 100);
    // A flat, dry runway for the putt.
    for (int32_t tx = 17; tx <= 21; tx++)
    {
        auto* surface = MapGetSurfaceElementAt(TileCoordsXY{ tx, 20 });
        surface->setBaseZ(16);
        surface->setTerrainShape(Terrain::TerrainShape{ 0, 0, 0, 0 });
        surface->setWaterHeight(0);
    }

    MapBallWorld world;
    world.useCourses();
    SeqRandom rng;
    Ball ball;
    placeBall(ball, world, Fixed::fromInt(20 * 32 + 16 - 40), Fixed::fromInt(20 * 32 + 16), BallState::Rolling);
    // Reach the cup at 0.3 units per tick whatever the roughness of the ground: v^2 = 0.09 + 2 a d, with a = roughness / 20.
    const Fixed deceleration = world.surfaceAt(ball.position.x, ball.position.y).roughness / 20;
    ball.velocity = { sqrt(Fixed::ratio(9, 100) + deceleration * 2 * 37), Fixed{}, Fixed{} };
    BallEvents total;
    for (int t = 0; t < 4000 && !isSettled(ball.state); t++)
    {
        const auto ev = stepBall(ball, world, rng);
        total.holed |= ev.holed;
    }
    ASSERT_EQ(ball.state, BallState::Holed);
    EXPECT_TRUE(total.holed);
    EXPECT_EQ(ball.position.x, Fixed::fromInt(20 * 32 + 16));
}

TEST(CourseWorld, CupsAreFoundOnlyNearTheBall)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& golf = getGameState().golf;
    const auto courseId = golf.createCourse("Links")->id;
    golf.createHole(courseId)->cup = HolePoint{ 900, 900 };
    MapBallWorld world;
    CupInfo cup;
    EXPECT_FALSE(world.cupNear(Fixed::fromInt(900), Fixed::fromInt(900), cup)); // courses not enabled
    world.useCourses();
    EXPECT_TRUE(world.cupNear(Fixed::fromInt(903), Fixed::fromInt(897), cup));
    EXPECT_EQ(cup.position.x, Fixed::fromInt(900));
    EXPECT_EQ(cup.position.y, Fixed::fromInt(900));
    EXPECT_FALSE(world.cupNear(Fixed::fromInt(950), Fixed::fromInt(900), cup));
    EXPECT_FALSE(world.cupNear(Fixed::fromInt(900), Fixed::fromInt(-100), cup));
}

TEST(CourseWorld, TheFlagPinIsAThinObstacleOnTheCup)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& golf = getGameState().golf;
    const auto courseId = golf.createCourse("Links")->id;
    golf.createHole(courseId)->cup = HolePoint{ 20 * 32 + 16, 20 * 32 + 16 };
    MapBallWorld world;
    ObstacleContact contact;
    const Fixed cx = Fixed::fromInt(20 * 32 + 16), cy = Fixed::fromInt(20 * 32 + 16);
    const Fixed ground = world.surfaceAt(cx, cy).height;
    // Without courses there is no pin.
    EXPECT_FALSE(world.obstacleAt({ cx + Fixed::ratio(3, 2), cy, ground + Fixed::fromInt(10) }, Fixed::ratio(1, 2), contact));
    world.useCourses();
    EXPECT_TRUE(world.obstacleAt({ cx + Fixed::ratio(3, 2), cy, ground + Fixed::fromInt(10) }, Fixed::ratio(1, 2), contact));
    EXPECT_GT(contact.normal.x, Fixed{}); // pushes the ball away from the pole
    EXPECT_FALSE(world.obstacleAt({ cx + Fixed::fromInt(3), cy, ground + Fixed::fromInt(10) }, Fixed::ratio(1, 2), contact));
    EXPECT_FALSE(world.obstacleAt({ cx + Fixed::ratio(3, 2), cy, ground + Fixed::fromInt(kPinHeight + 5) }, Fixed::ratio(1, 2), contact));
}
