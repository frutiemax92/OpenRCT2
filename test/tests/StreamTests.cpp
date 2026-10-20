/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <gtest/gtest.h>
#include <openrct2/world/Stream.h>
#include <openrct2/world/tile_element/StreamElement.h>
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
