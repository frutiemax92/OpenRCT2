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

#include <cmath>
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
#include <openrct2/object/ObjectManager.h>
#include <openrct2/world/Map.h>
#include <openrct2/world/MapAnimation.h>
#include <openrct2/world/TileElementsView.h>
#include <openrct2/world/tile_element/PathElement.h>
#include <openrct2/world/tile_element/StreamElement.h>
#include <openrct2/world/tile_element/SurfaceElement.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Golf;
using namespace OpenRCT2::Terrain;

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
        return context;
    }

    Fixed fx(double v)
    {
        return Fixed::fromRaw(static_cast<int64_t>(std::llround(v * 65536.0)));
    }
} // namespace

TEST(MapBallWorld, TerrainHeightMatchesTheIntegerSurfaceHeightForEveryShape)
{
    for (size_t index = 0; index < kShapeCount; index++)
    {
        const auto shape = GetShapeFromIndex(static_cast<uint16_t>(index));
        for (int32_t yl = 0; yl < 32; yl += 5)
            for (int32_t xl = 0; xl < 32; xl += 5)
            {
                const auto exact = terrainHeightAt(40, shape, Fixed::fromInt(xl), Fixed::fromInt(yl));
                const auto integer = GetSurfaceZAt(40, shape, xl, yl);
                // The integer function floors; the fixed one is exact.
                EXPECT_GE(exact.toDouble(), integer) << index << " " << xl << "," << yl;
                EXPECT_LT(exact.toDouble(), integer + 1.0) << index << " " << xl << "," << yl;
            }
    }
}

TEST(MapBallWorld, TerrainHeightIsContinuousAcrossTheDiagonal)
{
    const TerrainShape ramp{ 4, 2, 0, 2 };
    const auto onDiagonal = terrainHeightAt(0, ramp, Fixed::fromInt(10), Fixed::fromInt(10));
    const auto justBefore = terrainHeightAt(0, ramp, fx(10.01), fx(10));
    const auto justAfter = terrainHeightAt(0, ramp, fx(10), fx(10.01));
    EXPECT_NEAR(justBefore.toDouble(), onDiagonal.toDouble(), 0.5);
    EXPECT_NEAR(justAfter.toDouble(), onDiagonal.toDouble(), 0.5);
}

TEST(MapBallWorld, TerrainHeightHitsTheCornersExactly)
{
    const TerrainShape shape{ 1, 2, 0, 3 };
    EXPECT_EQ(terrainHeightAt(16, shape, Fixed{}, Fixed{}), Fixed::fromInt(16));                         // south
    EXPECT_EQ(terrainHeightAt(16, shape, Fixed::fromInt(32), Fixed{}), Fixed::fromInt(16 + 16));         // east
    EXPECT_EQ(terrainHeightAt(16, shape, Fixed::fromInt(32), Fixed::fromInt(32)), Fixed::fromInt(16 + 8)); // north
    EXPECT_EQ(terrainHeightAt(16, shape, Fixed{}, Fixed::fromInt(32)), Fixed::fromInt(16 + 24));         // west
}

TEST(MapBallWorld, NormalsPointUpAndAgreeWithTheIntegerNormals)
{
    EXPECT_EQ(terrainNormalAt(TerrainShape{ 0, 0, 0, 0 }, Fixed::fromInt(3), Fixed::fromInt(9)),
              (FixedVec3{ Fixed{}, Fixed{}, Fixed::fromInt(1) }));
    for (size_t index = 0; index < kShapeCount; index += 7)
    {
        const auto shape = GetShapeFromIndex(static_cast<uint16_t>(index));
        for (int32_t yl = 1; yl < 32; yl += 9)
            for (int32_t xl = 1; xl < 32; xl += 9)
            {
                const auto n = terrainNormalAt(shape, Fixed::fromInt(xl), Fixed::fromInt(yl));
                const auto ref = GetSurfaceNormalAt(shape, xl, yl);
                EXPECT_GT(n.z, Fixed{});
                EXPECT_NEAR(n.x.toDouble(), ref.x / 65536.0, 0.01) << index;
                EXPECT_NEAR(n.y.toDouble(), ref.y / 65536.0, 0.01) << index;
                EXPECT_NEAR(n.z.toDouble(), ref.z / 65536.0, 0.01) << index;
                EXPECT_NEAR(length(n).toDouble(), 1.0, 0.002);
            }
    }
}

TEST(MapBallWorld, ARampPushesTheNormalAwayFromTheHighSide)
{
    // North (x = 32, y = 32) raised: the triangle south-east of the diagonal faces -y, the other one faces -x.
    const TerrainShape ramp{ 4, 0, 0, 0 };
    const auto southEast = terrainNormalAt(ramp, Fixed::fromInt(24), Fixed::fromInt(20));
    EXPECT_EQ(southEast.x, Fixed{});
    EXPECT_LT(southEast.y, Fixed{});
    const auto northWest = terrainNormalAt(ramp, Fixed::fromInt(20), Fixed::fromInt(24));
    EXPECT_LT(northWest.x, Fixed{});
    EXPECT_EQ(northWest.y, Fixed{});
}

TEST(MapBallWorld, StreamDistanceFollowsTheConnectedEdges)
{
    constexpr uint8_t kWest = 1, kSouthY = 8, kEast = 4;
    // No connection: a disc around the centre.
    EXPECT_EQ(distanceToStream(0, Fixed::fromInt(16), Fixed::fromInt(16)), Fixed{});
    EXPECT_NEAR(distanceToStream(0, Fixed::fromInt(26), Fixed::fromInt(16)).toDouble(), 10.0, 0.01);
    // A stream running east-west through the centre.
    const uint8_t eastWest = kWest | kEast;
    EXPECT_EQ(distanceToStream(eastWest, Fixed::fromInt(2), Fixed::fromInt(16)), Fixed{});
    EXPECT_EQ(distanceToStream(eastWest, Fixed::fromInt(30), Fixed::fromInt(16)), Fixed{});
    EXPECT_NEAR(distanceToStream(eastWest, Fixed::fromInt(5), Fixed::fromInt(4)).toDouble(), 12.0, 0.01);
    // An end piece reaching only to the west does not cover the east half.
    EXPECT_NEAR(distanceToStream(kWest, Fixed::fromInt(30), Fixed::fromInt(16)).toDouble(), 14.0, 0.01);
    EXPECT_EQ(distanceToStream(kSouthY, Fixed::fromInt(16), Fixed::fromInt(3)), Fixed{});
}

TEST(MapBallWorld, ReadsTheTerrainOfARealMap)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    MapBallWorld world;

    // Find two flat dry tiles and tilt the second one.
    std::optional<CoordsXY> flat, sloped;
    for (int32_t ty = 3; ty < 60 && !(flat && sloped); ty++)
        for (int32_t tx = 3; tx < 60; tx++)
        {
            const CoordsXY tile{ tx * 32, ty * 32 };
            auto* surface = MapGetSurfaceElementAt(tile);
            if (surface == nullptr || surface->getWaterHeight() > 0 || surface->getTerrainShape() != TerrainShape{ 0, 0, 0, 0 })
                continue;
            if (!flat)
                flat = tile;
            else if (!sloped)
                sloped = tile;
        }
    ASSERT_TRUE(flat.has_value());
    ASSERT_TRUE(sloped.has_value());
    MapGetSurfaceElementAt(*sloped)->setTerrainShape(TerrainShape{ 0, 2, 0, 0 });

    const auto flatSample = world.surfaceAt(Fixed::fromInt(flat->x + 11), Fixed::fromInt(flat->y + 7));
    EXPECT_EQ(flatSample.height, Fixed::fromInt(MapGetSurfaceElementAt(*flat)->getBaseZ()));
    EXPECT_EQ(flatSample.normal, (FixedVec3{ Fixed{}, Fixed{}, Fixed::fromInt(1) }));
    EXPECT_FALSE(flatSample.water);
    EXPECT_FALSE(flatSample.outOfBounds);

    auto* slopedSurface = MapGetSurfaceElementAt(*sloped);
    const auto slopedSample = world.surfaceAt(Fixed::fromInt(sloped->x + 20), Fixed::fromInt(sloped->y + 9));
    EXPECT_EQ(slopedSample.height,
              terrainHeightAt(slopedSurface->getBaseZ(), slopedSurface->getTerrainShape(), Fixed::fromInt(20), Fixed::fromInt(9)));
    EXPECT_LT(slopedSample.normal.z, Fixed::fromInt(1));
    EXPECT_GT(slopedSample.normal.z, Fixed{});

    // The properties of the surface object come through (all initial surfaces define a roughness above zero).
    EXPECT_GT(flatSample.roughness, Fixed{});
    EXPECT_GT(flatSample.bounciness, Fixed{});
}

TEST(MapBallWorld, OutsideTheMapIsOutOfBounds)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    MapBallWorld world;
    EXPECT_TRUE(world.surfaceAt(Fixed::fromInt(-5), Fixed::fromInt(100)).outOfBounds);
    EXPECT_TRUE(world.surfaceAt(Fixed::fromInt(100), Fixed::fromInt(-1)).outOfBounds);
    EXPECT_TRUE(world.surfaceAt(Fixed::fromInt(100000), Fixed::fromInt(100)).outOfBounds);
    EXPECT_FALSE(world.surfaceAt(Fixed::fromInt(100), Fixed::fromInt(100)).outOfBounds);
}

TEST(MapBallWorld, TheCourseBoundaryComesFromThePredicate)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    MapBallWorld world;
    EXPECT_FALSE(world.surfaceAt(Fixed::fromInt(200), Fixed::fromInt(200)).outOfBounds);
    world.setOutOfBounds([](const CoordsXY& p) { return p.x >= 150; });
    EXPECT_TRUE(world.surfaceAt(Fixed::fromInt(200), Fixed::fromInt(200)).outOfBounds);
    EXPECT_FALSE(world.surfaceAt(Fixed::fromInt(100), Fixed::fromInt(200)).outOfBounds);
}

TEST(MapBallWorld, WaterIsAHazardAtTheWaterLevel)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    MapBallWorld world;
    std::optional<CoordsXY> dry;
    for (int32_t ty = 3; ty < 60 && !dry; ty++)
        for (int32_t tx = 3; tx < 60; tx++)
        {
            const CoordsXY tile{ tx * 32, ty * 32 };
            auto* surface = MapGetSurfaceElementAt(tile);
            if (surface != nullptr && surface->getWaterHeight() == 0 && surface->getTerrainShape() == TerrainShape{ 0, 0, 0, 0 })
            {
                dry = tile;
                break;
            }
        }
    ASSERT_TRUE(dry.has_value());
    auto* surface = MapGetSurfaceElementAt(*dry);
    const Fixed centreX = Fixed::fromInt(dry->x + 16), centreY = Fixed::fromInt(dry->y + 16);
    EXPECT_FALSE(world.surfaceAt(centreX, centreY).water);

    // Fill the tile with water 16 units above the ground.
    surface->setWaterHeight(surface->getBaseZ() + 16);
    const auto s = world.surfaceAt(centreX, centreY);
    EXPECT_TRUE(s.water);
    EXPECT_EQ(s.height, Fixed::fromInt(surface->getWaterHeight()));
    EXPECT_EQ(s.normal, (FixedVec3{ Fixed{}, Fixed{}, Fixed::fromInt(1) }));

    // On a slope the water only covers the part of the tile below the water level.
    surface->setTerrainShape(TerrainShape{ 0, 0, 0, 4 });
    EXPECT_TRUE(world.surfaceAt(Fixed::fromInt(dry->x + 2), Fixed::fromInt(dry->y + 4)).water); // 4 high
    EXPECT_FALSE(world.surfaceAt(Fixed::fromInt(dry->x + 2), Fixed::fromInt(dry->y + 31)).water); // west corner, 32 high
}

TEST(MapBallWorld, PathsAreHardpan)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    MapBallWorld world;
    std::optional<CoordsXY> pathTile;
    for (int32_t ty = 1; ty < 100 && !pathTile; ty++)
        for (int32_t tx = 1; tx < 100; tx++)
        {
            const CoordsXY tile{ tx * 32, ty * 32 };
            auto* surface = MapGetSurfaceElementAt(tile);
            if (surface == nullptr || surface->getWaterHeight() > 0)
                continue;
            for (auto* path : TileElementsView<PathElement>(tile))
            {
                if (path->getBaseZ() == surface->getBaseZ() && !path->isSloped() && surface->getTerrainShape() == TerrainShape{ 0, 0, 0, 0 })
                {
                    pathTile = tile;
                    break;
                }
            }
            if (pathTile)
                break;
        }
    if (!pathTile)
        GTEST_SKIP() << "the test park has no flat path";
    const auto s = world.surfaceAt(Fixed::fromInt(pathTile->x + 16), Fixed::fromInt(pathTile->y + 16));
    EXPECT_TRUE(s.path);
    EXPECT_EQ(s.roughness, Fixed::ratio(10, 100));
    EXPECT_EQ(s.bounciness, Fixed::ratio(55, 100));
    EXPECT_FALSE(s.puttable);
}

TEST(MapBallWorld, TheWindIsPassedToTheSimulation)
{
    MapBallWorld world;
    EXPECT_EQ(world.wind(), FixedVec3{});
    world.setWind({ Fixed::fromInt(1), Fixed{}, Fixed{} });
    EXPECT_EQ(world.wind().x, Fixed::fromInt(1));
}
