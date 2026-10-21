/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <gtest/gtest.h>
#include "TestData.h"
#include <openrct2/Context.h>
#include <openrct2/Game.h>
#include <openrct2/GameState.h>
#include <openrct2/OpenRCT2.h>
#include <openrct2/ParkImporter.h>
#include <openrct2/drawing/TerrainSpriteGenerator.h>
#include <openrct2/drawing/TerrainSpriteStore.h>
#include <openrct2/golf/StreetNetwork.h>
#include <openrct2/object/ObjectManager.h>
#include <openrct2/world/Map.h>
#include <openrct2/world/Stream.h>
#include <openrct2/world/tile_element/PathElement.h>
#include <openrct2/world/tile_element/StreamElement.h>
#include <openrct2/world/tile_element/SurfaceElement.h>
#include <openrct2/world/tile_element/TileElement.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Stream;
using OpenRCT2::Terrain::TerrainShape;

TEST(Stream, ElementFitsATileElementSlot)
{
    EXPECT_EQ(sizeof(StreamElement), sizeof(TileElement));
    EXPECT_EQ(static_cast<uint8_t>(TileElementType::stream), 8);
}

TEST(Stream, ElementStoresTypeEdgesAndFlow)
{
    TileElement slot{};
    slot.clearAs(TileElementType::stream);
    auto* stream = slot.asStream();
    ASSERT_NE(stream, nullptr);
    EXPECT_EQ(slot.asPath(), nullptr);
    EXPECT_EQ(stream->getEdges(), 0);

    stream->setObjectIndex(300);
    stream->setEdges(0xFF);
    EXPECT_EQ(stream->getEdges(), 0b1111);
    stream->setEdge(1, false);
    EXPECT_FALSE(stream->hasEdge(1));
    EXPECT_TRUE(stream->hasEdge(0));
    EXPECT_TRUE(stream->hasEdge(2));
    EXPECT_TRUE(stream->hasEdge(3));
    stream->setEdge(1, true);
    EXPECT_EQ(stream->getEdges(), 0b1111);

    stream->setFlowDirection(6);
    EXPECT_EQ(stream->getFlowDirection(), 2);
    EXPECT_EQ(stream->getObjectIndex(), 300);
    EXPECT_EQ(slot.getType(), TileElementType::stream);
}

TEST(Stream, EdgeHeightsFollowTheCorners)
{
    // Raised north corner: the +x and +y edges touch it.
    const TerrainShape shape{ 4, 0, 0, 0 };
    EXPECT_EQ(GetEdgeHeightSum(shape, 0), 0);
    EXPECT_EQ(GetEdgeHeightSum(shape, 1), 4);
    EXPECT_EQ(GetEdgeHeightSum(shape, 2), 4);
    EXPECT_EQ(GetEdgeHeightSum(shape, 3), 0);
}

TEST(Stream, FlatTileHasNoDownhillEdge)
{
    EXPECT_FALSE(GetDownhillEdge({ 0, 0, 0, 0 }).has_value());
    // A saddle is also level edge-wise.
    EXPECT_FALSE(GetDownhillEdge({ 2, 0, 2, 0 }).has_value());
}

TEST(Stream, WaterFlowsDownhillOnEveryRamp)
{
    // Ramps rising towards +x, so the low edge is -x (0).
    EXPECT_EQ(GetDownhillEdge({ 2, 2, 0, 0 }), 0);
    // Rising towards -x: low edge is +x (2).
    EXPECT_EQ(GetDownhillEdge({ 0, 0, 2, 2 }), 2);
    // Rising towards +y (north and west corners high): low edge is -y (3).
    EXPECT_EQ(GetDownhillEdge({ 2, 0, 0, 2 }), 3);
    // Rising towards -y: low edge is +y (1).
    EXPECT_EQ(GetDownhillEdge({ 0, 2, 2, 0 }), 1);
}

TEST(Stream, FlowResolvesFromSlopeConnectionsAndPlacement)
{
    const TerrainShape ramp{ 2, 2, 0, 0 }; // downhill edge 0
    EXPECT_EQ(ResolveFlowDirection(ramp, 0b0101, 3), 0);
    EXPECT_EQ(ResolveFlowDirection(ramp, 0, 3), 0);
    // The downhill edge is not connected: the lowest connected edge wins.
    EXPECT_EQ(ResolveFlowDirection(ramp, 0b1100, 3), 3);
    EXPECT_EQ(ResolveFlowDirection(ramp, 0b0110, 3), 1);
    // Two equally low connected edges: the first one is taken.
    EXPECT_EQ(ResolveFlowDirection(ramp, 0b1110, 3), 1);

    // Flat: the stored (placement) direction is used.
    const TerrainShape flat{ 0, 0, 0, 0 };
    EXPECT_EQ(ResolveFlowDirection(flat, 0b0101, 2), 2);
    EXPECT_EQ(ResolveFlowDirection(flat, 0b0101, 7), 3);
}

TEST(Stream, EveryShapeResolvesToADirection)
{
    for (uint16_t i = 0; i < Terrain::kShapeCount; i++)
    {
        const auto shape = Terrain::GetShapeFromIndex(i);
        const auto flow = ResolveFlowDirection(shape, kAllEdges, 1);
        ASSERT_LT(flow, 4);
        const auto downhill = GetDownhillEdge(shape);
        if (downhill.has_value())
        {
            // The chosen edge is never higher than any other edge.
            for (Direction e = 0; e < 4; e++)
                ASSERT_LE(GetEdgeHeightSum(shape, flow), GetEdgeHeightSum(shape, e));
        }
    }
}

TEST(Stream, AutoConnectFollowsNeighbours)
{
    EXPECT_EQ(AutoConnectEdges(0), 0);
    EXPECT_EQ(AutoConnectEdges(EdgeBit(0) | EdgeBit(2)), 0b0101);
    EXPECT_EQ(AutoConnectEdges(0xFF), kAllEdges);

    auto edges = UpdateEdge(0b0001, 2, true);
    EXPECT_EQ(edges, 0b0101);
    edges = UpdateEdge(edges, 0, false);
    EXPECT_EQ(edges, 0b0100);
    EXPECT_TRUE(IsEndPiece(edges));
    EXPECT_FALSE(IsEndPiece(0));
    EXPECT_FALSE(IsEndPiece(0b0101));
    EXPECT_EQ(OppositeEdge(0), 2);
    EXPECT_EQ(OppositeEdge(3), 1);
}

TEST(Stream, ProceduralStreamSpriteGeneratesForVariousShapesAndConnections)
{
    // STR-005: stream texture masked to its connection pattern and projected onto surface triangles
    using namespace OpenRCT2::Drawing;
    std::vector<uint8_t> pixels(16 * 16, 50); // solid colour palette index 50
    auto texture = TerrainTexture::Create(16, pixels);
    auto shades = TerrainShadeTable::Identity();

    const TerrainShape flat{ 0, 0, 0, 0 };
    // Straight stream (edges 0 and 2 = 0b0101)
    auto straightSprite = GenerateTerrainStreamSprite(flat, texture, 0b0101, 12, shades, 0, 0);
    EXPECT_GT(straightSprite.width, 0);
    EXPECT_GT(straightSprite.height, 0);
    EXPECT_EQ(straightSprite.pixels.size(), static_cast<size_t>(straightSprite.width) * straightSprite.height);
    size_t straightCovered = 0;
    for (auto p : straightSprite.pixels)
        if (p != 0)
            straightCovered++;
    EXPECT_GT(straightCovered, 0u);

    // Corner stream (edges 0 and 1 = 0b0011)
    auto cornerSprite = GenerateTerrainStreamSprite(flat, texture, 0b0011, 12, shades, 0, 0);
    size_t cornerCovered = 0;
    for (auto p : cornerSprite.pixels)
        if (p != 0)
            cornerCovered++;
    EXPECT_GT(cornerCovered, 0u);

    // Cross stream (edges 0b1111) covers more area than straight stream
    auto crossSprite = GenerateTerrainStreamSprite(flat, texture, 0b1111, 12, shades, 0, 0);
    size_t crossCovered = 0;
    for (auto p : crossSprite.pixels)
        if (p != 0)
            crossCovered++;
    EXPECT_GT(crossCovered, straightCovered);

    // Sloped terrain: ramp shape {2, 2, 0, 0}
    const TerrainShape ramp{ 2, 2, 0, 0 };
    for (int rot = 0; rot < 4; rot++)
    {
        auto slopedSprite = GenerateTerrainStreamSprite(ramp, texture, 0b0101, 12, shades, rot, 0);
        EXPECT_GT(slopedSprite.width, 0);
        EXPECT_GT(slopedSprite.height, 0);
        size_t slopedCovered = 0;
        for (auto p : slopedSprite.pixels)
            if (p != 0)
                slopedCovered++;
        EXPECT_GT(slopedCovered, 0u);
    }
}

TEST(Stream, StreamCrossingRulesForGolfersAndVehicles)
{
    // STR-007: Golfers walk across streams only on paths/bridges; vehicles only on streets/bridges
    gOpenRCT2Headless = true;
    gOpenRCT2NoGraphics = true;
    auto context = CreateContext();
    ASSERT_TRUE(context->Initialise());

    auto importer = ParkImporter::CreateParkFile(context->GetObjectRepository());
    auto loadResult = importer->LoadSavedGame(TestData::GetParkPath("small_park_with_ferris_wheel.park").c_str(), false);
    context->GetObjectManager().LoadObjects(loadResult.RequiredObjects);
    auto& gameState = getGameState();
    importer->Import(gameState);

    const CoordsXY tile{ 320, 320 }; // tile coordinates (10, 10)
    ASSERT_TRUE(MapIsLocationValid(tile));

    // Initially: no stream on tile
    EXPECT_EQ(FindAt(tile), nullptr);
    EXPECT_TRUE(CanGolferCross(tile));
    EXPECT_TRUE(CanVehicleCross(tile, nullptr));

    auto* surface = MapGetSurfaceElementAt(tile);
    ASSERT_NE(surface, nullptr);
    const int32_t baseZ = surface->getBaseZ();

    // Place a StreamElement on tile
    auto* streamElem = TileElementInsert<StreamElement>({ tile.x, tile.y, baseZ }, 0b1111);
    ASSERT_NE(streamElem, nullptr);
    streamElem->setEdges(0b0101); // straight stream
    EXPECT_NE(FindAt(tile), nullptr);

    // Now stream is present and tile has no path or street:
    EXPECT_FALSE(HasPathOrBridge(tile));
    EXPECT_FALSE(CanGolferCross(tile));
    EXPECT_FALSE(CanVehicleCross(tile, nullptr));

    // CanMove into this stream tile is blocked:
    const CoordsXY fromTile{ 320, 288 }; // (10, 9)
    EXPECT_FALSE(CanMove(fromTile, tile, false)); // golfer blocked
    EXPECT_FALSE(CanMove(fromTile, tile, true));  // vehicle blocked

    // Test vehicle with StreetNetwork:
    Golf::StreetNetwork streets;
    streets.placeTile(TileCoordsXY{ 10, 10 });
    EXPECT_TRUE(CanVehicleCross(tile, &streets));
    EXPECT_TRUE(CanMove(fromTile, tile, true, &streets));
    // Golfer is still blocked because there's no path/footpath bridge:
    EXPECT_FALSE(CanGolferCross(tile));
    EXPECT_FALSE(CanMove(fromTile, tile, false));

    // Add a PathElement on the stream tile (footpath / bridge):
    auto* pathElem = TileElementInsert<PathElement>({ tile.x, tile.y, baseZ }, 0b1111);
    ASSERT_NE(pathElem, nullptr);
    EXPECT_TRUE(HasPathOrBridge(tile));

    // With a path/bridge, both golfers and vehicles can cross:
    EXPECT_TRUE(CanGolferCross(tile));
    EXPECT_TRUE(CanVehicleCross(tile, nullptr));
    EXPECT_TRUE(CanMove(fromTile, tile, false));
    EXPECT_TRUE(CanMove(fromTile, tile, true));
}
