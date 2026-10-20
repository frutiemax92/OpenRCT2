/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "StreetNetwork.h"

#include <algorithm>
#include <bit>
#include <queue>
#include <set>

namespace OpenRCT2::Golf
{
    StreetShape shapeOf(uint8_t connections)
    {
        connections &= kAllEdges;
        switch (std::popcount(connections))
        {
            case 0:
                return StreetShape::isolated;
            case 1:
                return StreetShape::deadEnd;
            case 2:
                // Opposite edges: 0b0101 (0 and 2) or 0b1010 (1 and 3).
                return (connections == 0b0101 || connections == 0b1010) ? StreetShape::straight : StreetShape::corner;
            case 3:
                return StreetShape::tJunction;
            default:
                return StreetShape::crossing;
        }
    }

    bool isJunction(uint8_t connections)
    {
        const auto shape = shapeOf(connections);
        return shape == StreetShape::tJunction || shape == StreetShape::crossing;
    }

    uint8_t autoConnect(uint8_t neighbourMask)
    {
        return neighbourMask & kAllEdges;
    }

    void StreetNetwork::setTile(const StreetTile& tile)
    {
        _tiles[key(tile.position)] = tile;
    }

    StreetTile& StreetNetwork::placeTile(const TileCoordsXY& position, bool oneWay, uint8_t flow)
    {
        StreetTile tile;
        tile.position = position;
        tile.oneWay = oneWay;
        tile.flow = flow & 3;
        uint8_t mask = 0;
        for (int32_t d = 0; d < kStreetDirections; d++)
        {
            const auto step = directionStep(d);
            const TileCoordsXY neighbour{ position.x + step.x, position.y + step.y };
            if (tileAt(neighbour) != nullptr)
                mask |= edgeBit(d);
        }
        tile.connections = autoConnect(mask);
        _tiles[key(position)] = tile;
        // The neighbours gain the edge facing the new tile.
        for (int32_t d = 0; d < kStreetDirections; d++)
        {
            if (!(tile.connections & edgeBit(d)))
                continue;
            const auto step = directionStep(d);
            auto it = _tiles.find(key({ position.x + step.x, position.y + step.y }));
            if (it != _tiles.end())
                it->second.connections |= edgeBit(oppositeDirection(d));
        }
        return _tiles[key(position)];
    }

    bool StreetNetwork::removeTile(const TileCoordsXY& position)
    {
        auto it = _tiles.find(key(position));
        if (it == _tiles.end())
            return false;
        _tiles.erase(it);
        for (int32_t d = 0; d < kStreetDirections; d++)
        {
            const auto step = directionStep(d);
            auto n = _tiles.find(key({ position.x + step.x, position.y + step.y }));
            if (n != _tiles.end())
                n->second.connections &= static_cast<uint8_t>(~edgeBit(oppositeDirection(d)));
        }
        return true;
    }

    const StreetTile* StreetNetwork::tileAt(const TileCoordsXY& position) const
    {
        auto it = _tiles.find(key(position));
        return it != _tiles.end() ? &it->second : nullptr;
    }

    std::vector<TileCoordsXY> StreetNetwork::tiles() const
    {
        std::vector<TileCoordsXY> list;
        list.reserve(_tiles.size());
        for (const auto& [k, tile] : _tiles)
            list.push_back(tile.position);
        // A stable order, whatever the hash map does.
        std::sort(list.begin(), list.end(), [](const TileCoordsXY& a, const TileCoordsXY& b) {
            return a.x != b.x ? a.x < b.x : a.y < b.y;
        });
        return list;
    }

    bool StreetNetwork::allowsPassage(const StreetTile& tile, int32_t fromSide, int32_t toSide)
    {
        if (!tile.oneWay)
            return true;
        // One-way: traffic leaves through the flow direction. A tile with an entry on the opposite side is a straight; on a
        // corner or junction any connected side may be the entry, but the exit is the flow side only.
        return toSide == tile.flow && fromSide != toSide;
    }

    bool StreetNetwork::canDrive(const TileCoordsXY& from, int32_t direction) const
    {
        const auto* here = tileAt(from);
        if (here == nullptr || !(here->connections & edgeBit(direction)))
            return false;
        const auto step = directionStep(direction);
        const auto* there = tileAt({ from.x + step.x, from.y + step.y });
        if (there == nullptr || !(there->connections & edgeBit(oppositeDirection(direction))))
            return false;
        // Leaving the first tile through `direction`; the car may come from anywhere on a two-way tile.
        if (here->oneWay && here->flow != (direction & 3))
            return false;
        // Entering the second tile from the side facing back: a one-way tile must be able to send traffic on from there.
        if (there->oneWay && there->flow == oppositeDirection(direction))
            return false; // would drive against the flow of the next tile
        return true;
    }

    std::vector<TileCoordsXY> StreetNetwork::successors(const TileCoordsXY& from) const
    {
        std::vector<TileCoordsXY> list;
        for (int32_t d = 0; d < kStreetDirections; d++)
            if (canDrive(from, d))
            {
                const auto step = directionStep(d);
                list.push_back({ from.x + step.x, from.y + step.y });
            }
        return list;
    }

    std::vector<TileCoordsXY> StreetNetwork::invalidDeadEnds() const
    {
        std::vector<TileCoordsXY> bad;
        for (const auto& position : tiles())
        {
            const auto* tile = tileAt(position);
            if (shapeOf(tile->connections) == StreetShape::deadEnd && !tile->roadEntrance)
                bad.push_back(position);
        }
        return bad;
    }

    bool StreetNetwork::setRoadEntrance(const TileCoordsXY& position, bool entrance, const TileCoordsXY& mapSize)
    {
        auto it = _tiles.find(key(position));
        if (it == _tiles.end())
            return false;
        if (entrance)
        {
            // The outermost playable ring of tiles (the border ring itself is not buildable).
            const bool atEdge = position.x <= 1 || position.y <= 1 || position.x >= mapSize.x - 2 || position.y >= mapSize.y - 2;
            if (!atEdge)
                return false;
        }
        it->second.roadEntrance = entrance;
        return true;
    }

    std::vector<TileCoordsXY> StreetNetwork::roadEntrances() const
    {
        std::vector<TileCoordsXY> list;
        for (const auto& position : tiles())
            if (tileAt(position)->roadEntrance)
                list.push_back(position);
        return list;
    }

    int32_t StreetNetwork::crossingTicks(const StreetTile& tile, int32_t turnAngleQuarters, int32_t rise)
    {
        // 32 units at the speed limit: 40 km/h is about 1.9 units per tick.
        const int32_t speed100 = std::max(40, tile.speedLimitKph * 4 + tile.speedLimitKph / 2); // hundredths of a unit per tick
        int32_t ticks = 3200 / speed100;
        if (turnAngleQuarters != 0)
            ticks += 5;                       // slowing for a corner
        ticks += std::abs(rise) / 4;           // climbing
        return std::max(1, ticks);
    }

    std::vector<TileCoordsXY> StreetNetwork::findRoute(const TileCoordsXY& from, const TileCoordsXY& to) const
    {
        if (tileAt(from) == nullptr || tileAt(to) == nullptr)
            return {};
        if (from == to)
            return { from };
        struct Node
        {
            int32_t cost;
            int32_t estimate;
            TileCoordsXY position;
            bool operator>(const Node& o) const
            {
                return estimate != o.estimate ? estimate > o.estimate : (cost != o.cost ? cost > o.cost : (position.x != o.position.x ? position.x > o.position.x : position.y > o.position.y));
            }
        };
        auto heuristic = [&](const TileCoordsXY& p) { return (std::abs(p.x - to.x) + std::abs(p.y - to.y)) * 3; };
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
        std::unordered_map<int64_t, int32_t> best;
        std::unordered_map<int64_t, TileCoordsXY> parent;
        open.push({ 0, heuristic(from), from });
        best[key(from)] = 0;
        while (!open.empty())
        {
            const Node node = open.top();
            open.pop();
            if (node.cost > best[key(node.position)])
                continue;
            if (node.position == to)
            {
                std::vector<TileCoordsXY> route{ to };
                while (!(route.back() == from))
                    route.push_back(parent[key(route.back())]);
                std::reverse(route.begin(), route.end());
                return route;
            }
            const auto* here = tileAt(node.position);
            for (int32_t d = 0; d < kStreetDirections; d++)
            {
                if (!canDrive(node.position, d))
                    continue;
                const auto step = directionStep(d);
                const TileCoordsXY next{ node.position.x + step.x, node.position.y + step.y };
                const auto* there = tileAt(next);
                const int32_t cost = node.cost + crossingTicks(*there, 0, there->height - here->height);
                auto it = best.find(key(next));
                if (it == best.end() || cost < it->second)
                {
                    best[key(next)] = cost;
                    parent[key(next)] = node.position;
                    open.push({ cost, cost + heuristic(next), next });
                }
            }
        }
        return {};
    }

    std::vector<TileCoordsXY> StreetNetwork::routeToNearestEntrance(const TileCoordsXY& from) const
    {
        std::vector<TileCoordsXY> best;
        for (const auto& entrance : roadEntrances())
        {
            auto route = findRoute(from, entrance);
            if (!route.empty() && (best.empty() || route.size() < best.size()))
                best = std::move(route);
        }
        return best;
    }
} // namespace OpenRCT2::Golf
