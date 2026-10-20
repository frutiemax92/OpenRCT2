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
#include <optional>
#include <openrct2/Context.h>
#include <openrct2/Game.h>
#include <openrct2/GameState.h>
#include <openrct2/OpenRCT2.h>
#include <openrct2/ParkImporter.h>
#include <openrct2/drawing/Drawing.h>
#include <openrct2/drawing/Palette.h>
#include <openrct2/entity/EntityRegistry.h>
#include <openrct2/entity/EntityTweener.h>
#include <openrct2/golf/MapBallWorld.h>
#include <openrct2/golf/MapObstacles.h>
#include <openrct2/object/ObjectManager.h>
#include <openrct2/object/SmallSceneryObject.h>
#include <openrct2/world/Map.h>
#include <openrct2/world/MapAnimation.h>
#include <openrct2/world/TileElementsView.h>
#include <openrct2/world/tile_element/SmallSceneryElement.h>
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
        gameState.mapSize = { 60, 60 }; // the test park is only 15 tiles wide
        GetMapObstacles().reset();
        return context;
    }

    struct Tree
    {
        CoordsXY tile;
        SmallSceneryElement* element;
        const SmallSceneryObject* object;
    };

    // Puts a full tile small scenery object with a known collision block on a bare dry tile.
    std::optional<Tree> placeTree(IContext& context, const CoordsXY& tile)
    {
        auto& objectManager = context.GetObjectManager();
        for (ObjectEntryIndex i = 0; i < 2048; i++)
        {
            auto* object = objectManager.GetLoadedObject<SmallSceneryObject>(i);
            if (object == nullptr)
                continue;
            const auto baseZ = MapGetSurfaceElementAt(tile)->getBaseZ(); // inserting may move the tile elements
            auto* element = TileElementInsert<SmallSceneryElement>({ tile.x, tile.y, baseZ }, 0b1111);
            element->setEntryIndex(i);
            element->setSceneryQuadrant(0);
            element->setClearanceZ(baseZ + 32);
            return Tree{ tile, element, object };
        }
        return std::nullopt;
    }

    std::optional<CoordsXY> findBareTile(int skip = 0)
    {
        for (int32_t y = 3; y < 60; y++)
            for (int32_t x = 3; x < 60; x++)
            {
                const CoordsXY tile{ x * 32, y * 32 };
                auto* surface = MapGetSurfaceElementAt(tile);
                if (surface == nullptr || surface->getWaterHeight() > 0)
                    continue;
                int count = 0;
                for (auto* element : TileElementsView<TileElement>(tile))
                {
                    (void)element;
                    count++;
                }
                if (count == 1 && skip-- == 0)
                    return tile;
            }
        return std::nullopt;
    }
} // namespace

TEST(MapObstacles, SceneryBecomesAnObstacleAndRemovalIsNoticed)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    getGameState().cheats.sandboxMode = true;
    const auto tile = findBareTile();
    ASSERT_TRUE(tile.has_value());
    auto& obstacles = GetMapObstacles();

    const Fixed cx = Fixed::fromInt(tile->x + 16), cy = Fixed::fromInt(tile->y + 16);
    const Fixed z = Fixed::fromInt(MapGetSurfaceElementAt(*tile)->getBaseZ() + 2);
    ObstacleContact c;
    EXPECT_FALSE(obstacles.query({ cx, cy, z }, Fixed::ratio(1, 2), c));

    auto tree = placeTree(*context, *tile);
    ASSERT_TRUE(tree.has_value());
    const auto& collision = tree->object->Collision;
    if (collision.shape == CollisionShape::none)
        GTEST_SKIP() << "the first small scenery object has no collision";

    // The tile changed since the first query, so the next one rebuilds it and finds the tree.
    const auto before = obstacles.rebuildCount();
    EXPECT_TRUE(obstacles.query({ cx, cy, z }, Fixed::ratio(1, 2), c));
    EXPECT_GT(obstacles.rebuildCount(), before);
    EXPECT_NEAR(c.restitution.toDouble(), collision.restitution, 0.01);
    EXPECT_NEAR(c.friction.toDouble(), collision.friction, 0.01);

    // Querying again changes nothing: the tile is not rebuilt without a change.
    const auto steady = obstacles.rebuildCount();
    EXPECT_TRUE(obstacles.query({ cx, cy, z }, Fixed::ratio(1, 2), c));
    EXPECT_EQ(obstacles.rebuildCount(), steady);

    // Above the object there is nothing.
    EXPECT_FALSE(obstacles.query({ cx, cy, z + Fixed::fromInt(collision.height + 20) }, Fixed::ratio(1, 2), c));

    // Removing the element is noticed on the next query, and only this tile is rebuilt.
    TileElementRemove(reinterpret_cast<TileElement*>(tree->element));
    EXPECT_FALSE(obstacles.query({ cx, cy, z }, Fixed::ratio(1, 2), c));
    EXPECT_EQ(obstacles.rebuildCount(), steady + 1);
    EXPECT_EQ(obstacles.obstacleCount(), 0u);
}

TEST(MapObstacles, OnlyChangedTilesAreRebuilt)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    getGameState().cheats.sandboxMode = true;
    const auto a = findBareTile(0);
    const auto b = findBareTile(5);
    ASSERT_TRUE(a.has_value());
    ASSERT_TRUE(b.has_value());
    auto& obstacles = GetMapObstacles();
    ObstacleContact c;
    const Fixed r = Fixed::ratio(1, 2);
    auto probe = [&](const CoordsXY& t) {
        return obstacles.query({ Fixed::fromInt(t.x + 16), Fixed::fromInt(t.y + 16), Fixed::fromInt(MapGetSurfaceElementAt(t)->getBaseZ() + 2) }, r, c);
    };
    probe(*a);
    probe(*b);
    const auto base = obstacles.rebuildCount();
    auto tree = placeTree(*context, *a);
    ASSERT_TRUE(tree.has_value());
    probe(*a);
    probe(*b);
    EXPECT_EQ(obstacles.rebuildCount(), base + 1); // tile a only
    TileElementRemove(reinterpret_cast<TileElement*>(tree->element));
}

TEST(MapObstacles, DynamicObstaclesJoinTheSameGrid)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& obstacles = GetMapObstacles();
    Obstacle golfer;
    golfer.shape = ObstacleShape::cylinder;
    golfer.centre = vec(200, 200, 0);
    golfer.halfX = Fixed::fromInt(3);
    golfer.height = Fixed::fromInt(40);
    golfer.isGolfer = true;
    golfer.golferId = 12;
    obstacles.setDynamic(12, golfer);

    ObstacleContact c;
    ASSERT_TRUE(obstacles.query(vec(200, 200, 10), Fixed::ratio(1, 2), c));
    EXPECT_TRUE(c.isGolfer);
    EXPECT_EQ(c.golferId, 12u);
    EXPECT_GE(c.id, MapObstacles::kDynamicBase);

    golfer.centre = vec(300, 300, 0);
    obstacles.setDynamic(12, golfer); // moved
    EXPECT_FALSE(obstacles.query(vec(200, 200, 10), Fixed::ratio(1, 2), c));
    EXPECT_TRUE(obstacles.query(vec(300, 300, 10), Fixed::ratio(1, 2), c));
    obstacles.removeDynamic(12);
    EXPECT_FALSE(obstacles.query(vec(300, 300, 10), Fixed::ratio(1, 2), c));
}

TEST(MapObstacles, ABallFlyingIntoATreeInTheMapIsDeflected)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    getGameState().cheats.sandboxMode = true;
    const auto tile = findBareTile(2);
    ASSERT_TRUE(tile.has_value());
    auto tree = placeTree(*context, *tile);
    ASSERT_TRUE(tree.has_value());
    if (tree->object->Collision.shape == CollisionShape::none || tree->object->Collision.passThroughChance > 0)
        GTEST_SKIP() << "the first small scenery object cannot be hit reliably";

    MapBallWorld world;
    world.setObstacles(&GetMapObstacles());
    SeqRandom rng;
    Ball ball;
    const Fixed baseZ = Fixed::fromInt(MapGetSurfaceElementAt(*tile)->getBaseZ());
    // Start a few tiles west, at the height of the tree, flying straight at it.
    placeBall(ball, world, Fixed::fromInt(tile->x - 64), Fixed::fromInt(tile->y + 16));
    ball.position.z = baseZ + Fixed::fromInt(4);
    ball.velocity = vec(3, 0, 0);
    ball.state = BallState::InFlight;
    int hits = 0;
    for (int t = 0; t < 400 && !isSettled(ball.state); t++)
        hits += stepBall(ball, world, rng).obstacleHits;
    EXPECT_GT(hits, 0);
    EXPECT_LT(ball.position.x, Fixed::fromInt(tile->x + 16));
    TileElementRemove(reinterpret_cast<TileElement*>(tree->element));
}
