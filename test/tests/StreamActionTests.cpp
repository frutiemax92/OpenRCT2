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
#include <openrct2/actions/scenery/StreamPlaceAction.h>
#include <openrct2/actions/scenery/StreamRemoveAction.h>
#include <openrct2/drawing/Drawing.h>
#include <openrct2/drawing/Palette.h>
#include <openrct2/entity/EntityRegistry.h>
#include <openrct2/entity/EntityTweener.h>
#include <openrct2/object/ObjectManager.h>
#include <openrct2/object/StreamObject.h>
#include <openrct2/world/Map.h>
#include <openrct2/world/MapAnimation.h>
#include <openrct2/world/Stream.h>
#include <openrct2/world/TileElementsView.h>
#include <openrct2/world/tile_element/StreamElement.h>
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

    // A tile that holds nothing but dry ground, with a bare dry neighbour along +x.
    bool isBare(const CoordsXY& tile)
    {
        if (!MapIsLocationValid(tile))
            return false;
        auto* surface = MapGetSurfaceElementAt(tile);
        if (surface == nullptr || surface->getWaterHeight() > 0)
            return false;
        int count = 0;
        for (auto* element : TileElementsView<TileElement>(tile))
        {
            (void)element;
            count++;
        }
        return count == 1;
    }

    std::optional<CoordsXY> findBarePair()
    {
        for (int32_t y = 3; y < 60; y++)
        {
            for (int32_t x = 3; x < 60; x++)
            {
                const CoordsXY a{ x * 32, y * 32 };
                const CoordsXY b{ (x + 1) * 32, y * 32 };
                if (isBare(a) && isBare(b) && isBare({ (x + 2) * 32, y * 32 }))
                    return a;
            }
        }
        return std::nullopt;
    }
} // namespace

TEST(StreamActionTests, NeighbourHelpersAndRemove)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    gameState.cheats.sandboxMode = true;
    gameState.cheats.disableClearanceChecks = true;

    const auto start = findBarePair();
    ASSERT_TRUE(start.has_value());
    const CoordsXY a = *start;
    const CoordsXY b{ a.x + 32, a.y };
    const CoordsXY c{ a.x + 64, a.y };

    // Insert streams directly, as the actions do, to test the map helpers without an object.
    auto insert = [](const CoordsXY& tile) {
        const auto baseZ = MapGetSurfaceElementAt(tile)->getBaseZ();
        auto* el = TileElementInsert<StreamElement>({ tile.x, tile.y, baseZ }, 0b1111);
        el->setClearanceZ(baseZ);
        el->setEdges(Stream::AutoConnectEdges(Stream::GetNeighbourMask(tile)));
        Stream::UpdateNeighbours(tile, true);
        return el;
    };
    insert(a);
    insert(c);
    EXPECT_EQ(Stream::FindAt(a)->getEdges(), 0);
    EXPECT_EQ(Stream::GetNeighbourMask(b), Stream::EdgeBit(0) | Stream::EdgeBit(2));

    // b joins a (west of it, edge 0) and c (east, edge 2); a and c learn about b through edge 2 and edge 0.
    insert(b);
    EXPECT_EQ(Stream::FindAt(b)->getEdges(), Stream::EdgeBit(0) | Stream::EdgeBit(2));
    EXPECT_EQ(Stream::FindAt(a)->getEdges(), Stream::EdgeBit(2));
    EXPECT_EQ(Stream::FindAt(c)->getEdges(), Stream::EdgeBit(0));

    // Removing b disconnects both neighbours again.
    GameActions::StreamRemoveAction remove(b);
    EXPECT_EQ(GameActions::Query(&remove, gameState).error, GameActions::Status::ok);
    EXPECT_EQ(GameActions::ExecuteNested(&remove, gameState).error, GameActions::Status::ok);
    EXPECT_EQ(Stream::FindAt(b), nullptr);
    EXPECT_EQ(Stream::FindAt(a)->getEdges(), 0);
    EXPECT_EQ(Stream::FindAt(c)->getEdges(), 0);

    // Nothing left to remove.
    GameActions::StreamRemoveAction again(b);
    EXPECT_EQ(GameActions::Query(&again, gameState).error, GameActions::Status::invalidParameters);
}

TEST(StreamActionTests, PlaceRejectsUnknownObjectAndOffMapTiles)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    gameState.cheats.sandboxMode = true;

    const auto start = findBarePair();
    ASSERT_TRUE(start.has_value());

    GameActions::StreamPlaceAction unknown(*start, 0x7FFE, 0);
    EXPECT_EQ(GameActions::Query(&unknown, gameState).error, GameActions::Status::invalidParameters);

    GameActions::StreamPlaceAction offMap({ -32, -32 }, 0, 0);
    EXPECT_EQ(GameActions::Query(&offMap, gameState).error, GameActions::Status::invalidParameters);

    GameActions::StreamRemoveAction removeOffMap({ -32, -32 });
    EXPECT_EQ(GameActions::Query(&removeOffMap, gameState).error, GameActions::Status::invalidParameters);
}

TEST(StreamActionTests, PlaceConnectsToNeighboursWhenTheObjectIsLoaded)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    gameState.cheats.sandboxMode = true;
    gameState.cheats.disableClearanceChecks = true;

    auto& objectManager = context->GetObjectManager();
    auto* object = objectManager.LoadObject("golf.stream.creek");
    if (object == nullptr)
        GTEST_SKIP() << "the creek stream object is not installed";
    const auto index = objectManager.GetLoadedObjectEntryIndex(object);
    ASSERT_NE(index, kObjectEntryIndexNull);

    const auto start = findBarePair();
    ASSERT_TRUE(start.has_value());
    const CoordsXY a = *start;
    const CoordsXY b{ a.x + 32, a.y };

    GameActions::StreamPlaceAction placeA(a, index, 2);
    ASSERT_EQ(GameActions::ExecuteNested(&placeA, gameState).error, GameActions::Status::ok);
    EXPECT_EQ(Stream::FindAt(a)->getEdges(), 0);
    EXPECT_EQ(Stream::FindAt(a)->getFlowDirection(), 2);

    GameActions::StreamPlaceAction placeB(b, index, 2);
    ASSERT_EQ(GameActions::ExecuteNested(&placeB, gameState).error, GameActions::Status::ok);
    EXPECT_EQ(Stream::FindAt(b)->getEdges(), Stream::EdgeBit(0));
    EXPECT_EQ(Stream::FindAt(a)->getEdges(), Stream::EdgeBit(2));
    EXPECT_TRUE(Stream::IsEndPiece(Stream::FindAt(a)->getEdges()));

    // A tile holds a single stream.
    GameActions::StreamPlaceAction twice(b, index, 0);
    EXPECT_EQ(GameActions::Query(&twice, gameState).error, GameActions::Status::itemAlreadyPlaced);
}
