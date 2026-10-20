/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "TestData.h"

#include <gtest/gtest.h>
#include <memory>
#include <optional>
#include <openrct2/Context.h>
#include <openrct2/Game.h>
#include <openrct2/GameState.h>
#include <openrct2/OpenRCT2.h>
#include <openrct2/ParkImporter.h>
#include <openrct2/actions/GameActionRunner.h>
#include <openrct2/actions/footpath/FootpathPlaceAction.h>
#include <openrct2/actions/scenery/BuildingPlaceAction.h>
#include <openrct2/actions/scenery/BuildingRemoveAction.h>
#include <openrct2/drawing/Drawing.h>
#include <openrct2/drawing/Palette.h>
#include <openrct2/entity/EntityRegistry.h>
#include <openrct2/entity/EntityTweener.h>
#include <openrct2/object/ObjectManager.h>
#include <openrct2/object/BuildingObject.h>
#include <openrct2/world/Map.h>
#include <openrct2/world/MapAnimation.h>
#include <openrct2/world/Building.h>
#include <openrct2/world/TileElementsView.h>
#include <openrct2/world/tile_element/PathElement.h>
#include <openrct2/world/tile_element/BuildingElement.h>
#include <openrct2/world/tile_element/SurfaceElement.h>

using namespace OpenRCT2;

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

    bool isBareFlat(const CoordsXY& tile, int32_t z)
    {
        if (!MapIsLocationValid(tile))
            return false;
        auto* surface = MapGetSurfaceElementAt(tile);
        if (surface == nullptr || surface->getWaterHeight() > 0 || surface->getSlope() != 0 || surface->getBaseZ() != z)
            return false;
        int count = 0;
        for (auto* element : TileElementsView<TileElement>(tile))
        {
            (void)element;
            count++;
        }
        return count == 1;
    }

    // The corner of a bare, flat w x h block of tiles with a bare ring around it.
    std::optional<CoordsXY> findFlatBlock(int32_t w, int32_t h)
    {
        for (int32_t y = 4; y < 60; y++)
        {
            for (int32_t x = 4; x < 60; x++)
            {
                const CoordsXY corner{ x * 32, y * 32 };
                if (!MapIsLocationValid(corner))
                    continue;
                auto* first = MapGetSurfaceElementAt(corner);
                if (first == nullptr)
                    continue;
                const auto z = first->getBaseZ();
                bool ok = true;
                for (int32_t dy = -1; dy <= h && ok; dy++)
                    for (int32_t dx = -1; dx <= w && ok; dx++)
                        ok = isBareFlat({ (x + dx) * 32, (y + dy) * 32 }, z);
                if (ok)
                    return corner;
            }
        }
        return std::nullopt;
    }

    void insertPath(const CoordsXY& tile)
    {
        const auto baseZ = MapGetSurfaceElementAt(tile)->getBaseZ();
        auto* path = TileElementInsert<PathElement>({ tile.x, tile.y, baseZ }, 0b1111);
        path->setClearanceZ(baseZ + 16);
    }

    BuildingObject* loadClub(IContext& context, ObjectEntryIndex& index)
    {
        auto& objectManager = context.GetObjectManager();
        auto* object = objectManager.LoadObject("golf.building.club_house");
        if (object == nullptr)
            return nullptr;
        index = objectManager.GetLoadedObjectEntryIndex(object);
        return BuildingObject::GetById(index);
    }
} // namespace

TEST(BuildingActionTests, FootprintFollowsTheRotation)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    ObjectEntryIndex index{};
    auto* club = loadClub(*context, index);
    if (club == nullptr)
        GTEST_SKIP() << "the club house object is not installed";
    ASSERT_EQ(club->FootprintX, 3);
    ASSERT_EQ(club->FootprintY, 2);

    const CoordsXY origin{ 320, 320 };
    for (Direction rotation = 0; rotation < 4; rotation++)
    {
        const auto parts = Building::GetParts(*club, origin, rotation);
        ASSERT_EQ(parts.size(), 6u);
        int32_t maxX = 0, maxY = 0;
        for (const auto& part : parts)
        {
            EXPECT_GE(part.tile.x, origin.x);
            EXPECT_GE(part.tile.y, origin.y);
            maxX = std::max(maxX, (part.tile.x - origin.x) / 32 + 1);
            maxY = std::max(maxY, (part.tile.y - origin.y) / 32 + 1);
        }
        EXPECT_EQ(maxX, club->getFootprintX(rotation));
        EXPECT_EQ(maxY, club->getFootprintY(rotation));
    }

    // Rotation 0: the entrance tile is local (1, 0) and opens on edge 3 (-y).
    const auto entrance = Building::GetEntrance(*club, origin, 0);
    EXPECT_EQ(entrance.insideTile, (CoordsXY{ 352, 320 }));
    EXPECT_EQ(entrance.outsideTile, (CoordsXY{ 352, 288 }));
    EXPECT_EQ(entrance.edge, 3);
    // A quarter turn takes the entrance from -y to +x.
    EXPECT_EQ(Building::GetEntrance(*club, origin, 1).edge, 2);
}

TEST(BuildingActionTests, PlaceNeedsAPathAtTheEntranceAndRemoveTakesTheWholeBuilding)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    gameState.cheats.sandboxMode = true;
    ObjectEntryIndex index{};
    auto* club = loadClub(*context, index);
    if (club == nullptr)
        GTEST_SKIP() << "the club house object is not installed";

    const auto found = findFlatBlock(3, 2);
    ASSERT_TRUE(found.has_value());
    const CoordsXY origin = *found;
    const auto entrance = Building::GetEntrance(*club, origin, 0);

    GameActions::BuildingPlaceAction place(origin, index, 0);
    EXPECT_EQ(GameActions::Query(&place, gameState).error, GameActions::Status::disallowed);

    insertPath(entrance.outsideTile);
    auto res = GameActions::ExecuteNested(&place, gameState);
    ASSERT_EQ(res.error, GameActions::Status::ok);
    EXPECT_EQ(res.cost, club->BuildCost);

    for (const auto& part : Building::GetParts(*club, origin, 0))
    {
        auto* element = Building::FindAt(part.tile);
        ASSERT_NE(element, nullptr);
        EXPECT_EQ(element->getObjectIndex(), index);
        EXPECT_EQ(element->getPartX(), part.localX);
        EXPECT_EQ(element->getPartY(), part.localY);
        EXPECT_EQ(Building::GetOrigin(part.tile, *element), origin);
        EXPECT_EQ(element->getClearanceZ() - element->getBaseZ(), club->Height);
    }

    // The footprint is taken.
    GameActions::BuildingPlaceAction twice(origin, index, 0);
    EXPECT_NE(GameActions::Query(&twice, gameState).error, GameActions::Status::ok);

    // Removing from any tile of the footprint takes all of it.
    GameActions::BuildingRemoveAction remove({ origin.x + 64, origin.y + 32 });
    ASSERT_EQ(GameActions::ExecuteNested(&remove, gameState).error, GameActions::Status::ok);
    for (const auto& part : Building::GetParts(*club, origin, 0))
        EXPECT_EQ(Building::FindAt(part.tile), nullptr);

    GameActions::BuildingRemoveAction again(origin);
    EXPECT_EQ(GameActions::Query(&again, gameState).error, GameActions::Status::invalidParameters);
}

TEST(BuildingActionTests, RotatedBuildingRemovesCleanly)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    gameState.cheats.sandboxMode = true;
    ObjectEntryIndex index{};
    auto* club = loadClub(*context, index);
    if (club == nullptr)
        GTEST_SKIP() << "the club house object is not installed";

    const auto found = findFlatBlock(2, 3);
    ASSERT_TRUE(found.has_value());
    const CoordsXY origin = *found;
    insertPath(Building::GetEntrance(*club, origin, 1).outsideTile);

    GameActions::BuildingPlaceAction place(origin, index, 1);
    ASSERT_EQ(GameActions::ExecuteNested(&place, gameState).error, GameActions::Status::ok);
    for (const auto& part : Building::GetParts(*club, origin, 1))
        EXPECT_EQ(Building::GetOrigin(part.tile, *Building::FindAt(part.tile)), origin);

    // Remove from the far corner.
    GameActions::BuildingRemoveAction remove({ origin.x + 32, origin.y + 64 });
    ASSERT_EQ(GameActions::ExecuteNested(&remove, gameState).error, GameActions::Status::ok);
    for (const auto& part : Building::GetParts(*club, origin, 1))
        EXPECT_EQ(Building::FindAt(part.tile), nullptr);
}

TEST(BuildingActionTests, PlaceRejectsUnknownObjectAndOffMapTiles)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    gameState.cheats.sandboxMode = true;

    GameActions::BuildingPlaceAction unknown({ 320, 320 }, 0x7FFE, 0);
    EXPECT_EQ(GameActions::Query(&unknown, gameState).error, GameActions::Status::invalidParameters);

    GameActions::BuildingPlaceAction offMap({ -32, -32 }, 0, 0);
    EXPECT_EQ(GameActions::Query(&offMap, gameState).error, GameActions::Status::invalidParameters);

    GameActions::BuildingRemoveAction removeOffMap({ -32, -32 });
    EXPECT_EQ(GameActions::Query(&removeOffMap, gameState).error, GameActions::Status::invalidParameters);
}

TEST(BuildingActionTests, TheNearestTilePaintsTheWholeView)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    ObjectEntryIndex index{};
    auto* club = loadClub(*context, index);
    if (club == nullptr)
        GTEST_SKIP() << "the club house object is not installed";

    const CoordsXY origin{ 320, 320 };
    // Camera rotation 0, building rotation 0: the +x +y corner of a 3x2 footprint is the nearest.
    EXPECT_EQ(Building::GetPaintTile(*club, origin, 0, 0), (CoordsXY{ 384, 352 }));
    // The opposite camera picks the opposite corner.
    EXPECT_EQ(Building::GetPaintTile(*club, origin, 0, 2), (CoordsXY{ 320, 320 }));

    for (Direction rotation = 0; rotation < 4; rotation++)
    {
        const auto parts = Building::GetParts(*club, origin, rotation);
        for (uint8_t camera = 0; camera < 4; camera++)
        {
            const auto tile = Building::GetPaintTile(*club, origin, rotation, camera);
            const auto seen = tile.rotate(camera);
            bool found = false;
            for (const auto& part : parts)
            {
                found |= part.tile == tile;
                const auto other = part.tile.rotate(camera);
                EXPECT_LE(other.x + other.y, seen.x + seen.y);
            }
            EXPECT_TRUE(found);
        }
    }
}

TEST(PathSlope, PlaceActionStoresTheRiseAndRaisesTheClearance)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    gameState.cheats.sandboxMode = true;
    gameState.cheats.disableClearanceChecks = true;

    auto block = findFlatBlock(1, 1);
    ASSERT_TRUE(block.has_value());
    const auto z = MapGetSurfaceElementAt(*block)->getBaseZ();

    for (uint8_t rise = 1; rise <= 4; rise++)
    {
        const CoordsXY tile{ block->x, block->y };
        GameActions::FootpathPlaceAction place(
            { tile.x, tile.y, z }, FootpathSlope{ FootpathSlopeType::sloped, 1, rise }, 0, 0, kInvalidDirection,
            PathConstructFlag::IsLegacyPathObject);
        ASSERT_EQ(GameActions::ExecuteNested(&place, gameState).error, GameActions::Status::ok);

        PathElement* path = nullptr;
        for (auto* element : TileElementsView<PathElement>(tile))
            path = element;
        ASSERT_NE(path, nullptr);
        EXPECT_TRUE(path->isSloped());
        EXPECT_EQ(path->getSlopeRiseQ(), rise);
        EXPECT_EQ(path->getSlopeDirection(), 1);
        EXPECT_EQ(path->getClearanceZ(), z + kPathClearance + rise * kCoordsZStep);
        path->remove();
    }
}
