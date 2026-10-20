/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../world/Location.hpp"

#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

namespace OpenRCT2::Golf
{
    // The street network (section 12.1): tiles with connections to their neighbours, two-way or one-way, and the route
    // finding cars use on them. The tile data is what the StreetElement stores; the map code fills a StreetNetwork from the
    // map and the cars (TrafficSimulation) drive on it.

    // Directions follow OpenRCT2: 0 = -x, 1 = +y, 2 = +x, 3 = -y.
    constexpr int32_t kStreetDirections = 4;
    constexpr uint8_t kAllEdges = 0b1111;
    constexpr uint8_t edgeBit(int32_t direction)
    {
        return static_cast<uint8_t>(1u << (direction & 3));
    }
    constexpr int32_t oppositeDirection(int32_t direction)
    {
        return (direction + 2) & 3;
    }
    // The step in tiles for a direction.
    constexpr TileCoordsXY directionStep(int32_t direction)
    {
        constexpr TileCoordsXY steps[4] = { { -1, 0 }, { 0, 1 }, { 1, 0 }, { 0, -1 } };
        return steps[direction & 3];
    }

    // STE-002: the shape of a street tile from its connections.
    enum class StreetShape : uint8_t
    {
        isolated,
        deadEnd,
        straight,
        corner,
        tJunction,
        crossing,
    };
    StreetShape shapeOf(uint8_t connections);
    // Junctions are the tiles where cars must give way to each other (T and cross).
    bool isJunction(uint8_t connections);

    struct StreetTile
    {
        TileCoordsXY position;
        uint8_t connections = 0;       // one bit per direction
        bool oneWay = false;           // STE-002
        uint8_t flow = 0;              // direction traffic leaves a one-way tile through
        int32_t height = 0;            // z units, for the slope of the road
        int32_t speedLimitKph = 40;    // STE-004
        bool roadEntrance = false;     // STE-005: cars spawn and despawn here
    };

    // STE-001: connect a new tile to the neighbours that are streets: the edges that face a street tile which has an edge
    // facing back (or none at all yet).
    uint8_t autoConnect(uint8_t neighbourMask);

    class StreetNetwork
    {
    public:
        // Adds or replaces a tile. The connections are as given; use connectTile for automatic connection.
        void setTile(const StreetTile& tile);
        // STE-001: adds a tile and connects it to every neighbouring street tile, updating the neighbours too.
        StreetTile& placeTile(const TileCoordsXY& position, bool oneWay = false, uint8_t flow = 0);
        bool removeTile(const TileCoordsXY& position);
        const StreetTile* tileAt(const TileCoordsXY& position) const;
        size_t tileCount() const
        {
            return _tiles.size();
        }
        std::vector<TileCoordsXY> tiles() const;

        // The direction cars can take from a tile into a neighbour: connected on both sides, and the one-way rules allow
        // leaving this tile that way and entering the other from that side.
        bool canDrive(const TileCoordsXY& from, int32_t direction) const;
        // The neighbours reachable by driving, in direction order.
        std::vector<TileCoordsXY> successors(const TileCoordsXY& from) const;

        // STE-002: dead ends are allowed only at the edge of the map where cars spawn. Returns the tiles that break the
        // rule: dead ends that are not road entrances.
        std::vector<TileCoordsXY> invalidDeadEnds() const;
        // STE-005: road entrances exist at the map edge on a street tile.
        bool setRoadEntrance(const TileCoordsXY& position, bool entrance, const TileCoordsXY& mapSize);
        std::vector<TileCoordsXY> roadEntrances() const;

        // Shortest drive between two tiles (A*), honouring one-way streets; empty when there is none. The route includes
        // both ends.
        std::vector<TileCoordsXY> findRoute(const TileCoordsXY& from, const TileCoordsXY& to) const;
        // The nearest road entrance reachable from a tile; the route to it.
        std::vector<TileCoordsXY> routeToNearestEntrance(const TileCoordsXY& from) const;

        // Time to cross a tile in ticks at the speed limit of the tile, with a penalty for slopes and turns (STE-020).
        static int32_t crossingTicks(const StreetTile& tile, int32_t turnAngleQuarters, int32_t rise);

    private:
        static int64_t key(const TileCoordsXY& p)
        {
            return (static_cast<int64_t>(p.x) << 32) ^ static_cast<int64_t>(static_cast<uint32_t>(p.y));
        }
        // Whether traffic may enter `tile` from the side facing direction `fromSide`, and leave through `toSide`.
        static bool allowsPassage(const StreetTile& tile, int32_t fromSide, int32_t toSide);

        std::unordered_map<int64_t, StreetTile> _tiles;
    };
} // namespace OpenRCT2::Golf
