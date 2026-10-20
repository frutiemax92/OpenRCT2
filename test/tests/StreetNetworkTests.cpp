/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <gtest/gtest.h>
#include <openrct2/golf/StreetNetwork.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Golf;

namespace
{
    // A straight street along +x from (x0, y) to (x1, y).
    void lay(StreetNetwork& n, int x0, int x1, int y, bool oneWay = false, int flow = 2)
    {
        for (int x = x0; x <= x1; x++)
            n.placeTile({ x, y }, oneWay, static_cast<uint8_t>(flow));
    }
} // namespace

TEST(StreetShapes, ComeFromTheConnections)
{
    EXPECT_EQ(shapeOf(0), StreetShape::isolated);
    EXPECT_EQ(shapeOf(edgeBit(2)), StreetShape::deadEnd);
    EXPECT_EQ(shapeOf(edgeBit(0) | edgeBit(2)), StreetShape::straight);
    EXPECT_EQ(shapeOf(edgeBit(1) | edgeBit(3)), StreetShape::straight);
    EXPECT_EQ(shapeOf(edgeBit(0) | edgeBit(1)), StreetShape::corner);
    EXPECT_EQ(shapeOf(edgeBit(2) | edgeBit(3)), StreetShape::corner);
    EXPECT_EQ(shapeOf(edgeBit(0) | edgeBit(1) | edgeBit(2)), StreetShape::tJunction);
    EXPECT_EQ(shapeOf(kAllEdges), StreetShape::crossing);
    EXPECT_FALSE(isJunction(edgeBit(0) | edgeBit(2)));
    EXPECT_TRUE(isJunction(edgeBit(0) | edgeBit(1) | edgeBit(2)));
    EXPECT_TRUE(isJunction(kAllEdges));
    EXPECT_FALSE(isJunction(edgeBit(0) | edgeBit(3)));
}

TEST(StreetShapes, DirectionsFollowTheMapConvention)
{
    EXPECT_EQ(directionStep(0).x, -1);
    EXPECT_EQ(directionStep(1).y, 1);
    EXPECT_EQ(directionStep(2).x, 1);
    EXPECT_EQ(directionStep(3).y, -1);
    EXPECT_EQ(oppositeDirection(0), 2);
    EXPECT_EQ(oppositeDirection(3), 1);
    EXPECT_EQ(edgeBit(5), edgeBit(1)); // directions wrap
}

TEST(StreetNetwork, TilesConnectToTheirNeighboursWhenPlaced)
{
    StreetNetwork n;
    n.placeTile({ 5, 5 });
    EXPECT_EQ(n.tileAt({ 5, 5 })->connections, 0);
    n.placeTile({ 6, 5 });
    EXPECT_EQ(n.tileAt({ 5, 5 })->connections, edgeBit(2));
    EXPECT_EQ(n.tileAt({ 6, 5 })->connections, edgeBit(0));
    n.placeTile({ 7, 5 });
    EXPECT_EQ(n.tileAt({ 6, 5 })->connections, edgeBit(0) | edgeBit(2));
    EXPECT_EQ(shapeOf(n.tileAt({ 6, 5 })->connections), StreetShape::straight);
    // A tile above the middle makes a T junction there.
    n.placeTile({ 6, 6 });
    EXPECT_EQ(shapeOf(n.tileAt({ 6, 5 })->connections), StreetShape::tJunction);
    EXPECT_EQ(n.tileAt({ 6, 6 })->connections, edgeBit(3));
    EXPECT_EQ(n.tileCount(), 4u);
}

TEST(StreetNetwork, RemovingATileDisconnectsItsNeighbours)
{
    StreetNetwork n;
    lay(n, 5, 7, 5);
    EXPECT_TRUE(n.removeTile({ 6, 5 }));
    EXPECT_FALSE(n.removeTile({ 6, 5 }));
    EXPECT_EQ(n.tileAt({ 5, 5 })->connections, 0);
    EXPECT_EQ(n.tileAt({ 7, 5 })->connections, 0);
    EXPECT_EQ(n.tileAt({ 6, 5 }), nullptr);
}

TEST(StreetNetwork, TwoWayStreetsCanBeDrivenBothWays)
{
    StreetNetwork n;
    lay(n, 5, 8, 5);
    EXPECT_TRUE(n.canDrive({ 5, 5 }, 2));
    EXPECT_TRUE(n.canDrive({ 8, 5 }, 0));
    EXPECT_FALSE(n.canDrive({ 5, 5 }, 0)); // nothing there
    EXPECT_FALSE(n.canDrive({ 5, 5 }, 1));
    EXPECT_FALSE(n.canDrive({ 1, 1 }, 2));
    EXPECT_EQ(n.successors({ 6, 5 }).size(), 2u);
}

TEST(StreetNetwork, OneWayStreetsAreOnlyDrivenWithTheFlow)
{
    StreetNetwork n;
    lay(n, 5, 8, 5, true, 2); // flows towards +x
    EXPECT_TRUE(n.canDrive({ 5, 5 }, 2));
    EXPECT_TRUE(n.canDrive({ 7, 5 }, 2));
    EXPECT_FALSE(n.canDrive({ 8, 5 }, 0));
    EXPECT_FALSE(n.canDrive({ 6, 5 }, 0));
    EXPECT_EQ(n.successors({ 6, 5 }).size(), 1u);
}

TEST(StreetNetwork, RoutesFollowTheShortestDrive)
{
    StreetNetwork n;
    lay(n, 0, 5, 0);
    for (int y = 1; y <= 5; y++)
        n.placeTile({ 5, y });
    const auto route = n.findRoute({ 0, 0 }, { 5, 5 });
    ASSERT_EQ(route.size(), 11u);
    EXPECT_EQ(route.front(), (TileCoordsXY{ 0, 0 }));
    EXPECT_EQ(route.back(), (TileCoordsXY{ 5, 5 }));
    for (size_t i = 1; i < route.size(); i++)
        EXPECT_EQ(std::abs(route[i].x - route[i - 1].x) + std::abs(route[i].y - route[i - 1].y), 1);
    EXPECT_EQ(n.findRoute({ 0, 0 }, { 0, 0 }).size(), 1u);
    EXPECT_TRUE(n.findRoute({ 0, 0 }, { 9, 9 }).empty());
    EXPECT_TRUE(n.findRoute({ 9, 9 }, { 0, 0 }).empty());
}

TEST(StreetNetwork, RoutesPickTheShorterOfTwoWays)
{
    StreetNetwork n;
    // A square block: the direct side and the long way round.
    lay(n, 0, 4, 0);
    lay(n, 0, 4, 4);
    for (int y = 1; y <= 3; y++)
    {
        n.placeTile({ 0, y });
        n.placeTile({ 4, y });
    }
    const auto route = n.findRoute({ 0, 0 }, { 4, 0 });
    EXPECT_EQ(route.size(), 5u);
    const auto other = n.findRoute({ 0, 0 }, { 4, 4 });
    EXPECT_EQ(other.size(), 9u);
}

TEST(StreetNetwork, OneWayStreetsAreRespectedByRoutes)
{
    // A ring: the street along the top is one-way towards +x between two two-way corners. Going from the right end to the
    // left end has to use the way round.
    StreetNetwork n;
    lay(n, 1, 3, 0, true, 2);   // one-way eastward
    lay(n, 0, 4, 3);            // two-way along the bottom
    for (int y = 0; y <= 2; y++)
    {
        n.placeTile({ 0, y });
        n.placeTile({ 4, y });
    }
    const auto east = n.findRoute({ 0, 0 }, { 4, 0 });
    ASSERT_FALSE(east.empty());
    EXPECT_EQ(east.size(), 5u); // straight along the one-way street
    const auto west = n.findRoute({ 4, 0 }, { 0, 0 });
    ASSERT_FALSE(west.empty());
    EXPECT_GT(west.size(), 5u); // cannot drive back along it: down, across the bottom and up
    for (size_t i = 1; i < west.size(); i++)
    {
        EXPECT_FALSE(west[i - 1].y == 0 && west[i].y == 0) << "drove against the one-way street at " << i;
    }
    EXPECT_EQ(west.size(), 11u); // three down, four across, three up, plus both ends
}

TEST(StreetNetwork, ADeadOneWayCannotBeEnteredAgainstItsFlow)
{
    StreetNetwork n;
    lay(n, 0, 3, 0, true, 0); // flows towards -x
    EXPECT_FALSE(n.canDrive({ 2, 0 }, 2));
    EXPECT_TRUE(n.canDrive({ 3, 0 }, 0));
    EXPECT_TRUE(n.findRoute({ 0, 0 }, { 3, 0 }).empty());
    EXPECT_EQ(n.findRoute({ 3, 0 }, { 0, 0 }).size(), 4u);
}

TEST(StreetNetwork, DeadEndsAreOnlyAllowedAtRoadEntrances)
{
    StreetNetwork n;
    lay(n, 1, 6, 10);
    const TileCoordsXY mapSize{ 60, 60 };
    // Both ends of the street are dead ends.
    EXPECT_EQ(n.invalidDeadEnds().size(), 2u);
    // An entrance has to be on the edge of the map.
    EXPECT_FALSE(n.setRoadEntrance({ 1, 10 }, true, mapSize) && false);
    EXPECT_TRUE(n.setRoadEntrance({ 1, 10 }, true, mapSize)); // x = 1 is the first playable ring
    EXPECT_FALSE(n.setRoadEntrance({ 4, 10 }, true, mapSize));
    EXPECT_FALSE(n.setRoadEntrance({ 20, 20 }, true, mapSize)); // no street there
    const auto bad = n.invalidDeadEnds();
    ASSERT_EQ(bad.size(), 1u);
    EXPECT_EQ(bad[0], (TileCoordsXY{ 6, 10 }));
    EXPECT_EQ(n.roadEntrances().size(), 1u);
    EXPECT_TRUE(n.setRoadEntrance({ 1, 10 }, false, mapSize));
    EXPECT_TRUE(n.roadEntrances().empty());
}

TEST(StreetNetwork, TheNearestEntranceIsFound)
{
    StreetNetwork n;
    lay(n, 1, 30, 10);
    const TileCoordsXY mapSize{ 60, 60 };
    n.setRoadEntrance({ 1, 10 }, true, mapSize);
    n.setRoadEntrance({ 58, 10 }, true, mapSize); // not on a street: ignored
    lay(n, 31, 58, 10);
    n.setRoadEntrance({ 58, 10 }, true, mapSize);
    const auto fromLeft = n.routeToNearestEntrance({ 5, 10 });
    ASSERT_FALSE(fromLeft.empty());
    EXPECT_EQ(fromLeft.back(), (TileCoordsXY{ 1, 10 }));
    const auto fromRight = n.routeToNearestEntrance({ 50, 10 });
    EXPECT_EQ(fromRight.back(), (TileCoordsXY{ 58, 10 }));
    StreetNetwork lonely;
    lay(lonely, 1, 3, 10);
    EXPECT_TRUE(lonely.routeToNearestEntrance({ 2, 10 }).empty());
}

TEST(StreetNetwork, SlopesAndTurnsMakeCrossingSlower)
{
    StreetTile tile;
    tile.speedLimitKph = 40;
    const int32_t flat = StreetNetwork::crossingTicks(tile, 0, 0);
    EXPECT_GT(StreetNetwork::crossingTicks(tile, 1, 0), flat);
    EXPECT_GT(StreetNetwork::crossingTicks(tile, 0, 16), flat);
    tile.speedLimitKph = 80;
    EXPECT_LT(StreetNetwork::crossingTicks(tile, 0, 0), flat);
    EXPECT_GE(StreetNetwork::crossingTicks(tile, 0, 0), 1);
}

TEST(StreetNetwork, TheTilesAreListedInAStableOrder)
{
    StreetNetwork n;
    n.placeTile({ 9, 2 });
    n.placeTile({ 3, 8 });
    n.placeTile({ 3, 1 });
    const auto list = n.tiles();
    ASSERT_EQ(list.size(), 3u);
    EXPECT_EQ(list[0], (TileCoordsXY{ 3, 1 }));
    EXPECT_EQ(list[1], (TileCoordsXY{ 3, 8 }));
    EXPECT_EQ(list[2], (TileCoordsXY{ 9, 2 }));
}
