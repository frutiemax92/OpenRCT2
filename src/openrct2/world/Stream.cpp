/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#include "Stream.h"

#include <algorithm>
#include <bit>

namespace OpenRCT2::Stream
{
    using Terrain::TerrainShape;

    int32_t GetEdgeHeightSum(const TerrainShape& shape, Direction edge)
    {
        // Corner positions: S = (0,0), E = (32,0), N = (32,32), W = (0,32).
        switch (edge & 3)
        {
            case 0: // x = 0
                return shape.south + shape.west;
            case 1: // y = 32
                return shape.north + shape.west;
            case 2: // x = 32
                return shape.east + shape.north;
            default: // y = 0
                return shape.south + shape.east;
        }
    }

    std::optional<Direction> GetDownhillEdge(const TerrainShape& shape)
    {
        // Ties (diagonal slopes) go to the first lowest edge, so the result is deterministic.
        int32_t lowest = GetEdgeHeightSum(shape, 0);
        int32_t highest = lowest;
        Direction lowestEdge = 0;
        for (Direction edge = 1; edge < 4; edge++)
        {
            const auto h = GetEdgeHeightSum(shape, edge);
            if (h < lowest)
            {
                lowest = h;
                lowestEdge = edge;
            }
            highest = std::max(highest, h);
        }
        if (highest == lowest)
            return std::nullopt;
        return lowestEdge;
    }

    Direction ResolveFlowDirection(const TerrainShape& shape, uint8_t edges, Direction storedFlow)
    {
        edges &= kAllEdges;
        const auto downhill = GetDownhillEdge(shape);
        if (!downhill.has_value())
            return storedFlow & 3;
        if (edges == 0 || (edges & EdgeBit(*downhill)) != 0)
            return *downhill;

        // The downhill edge is not connected: leave by the lowest connected edge.
        Direction best = storedFlow & 3;
        int32_t bestHeight = INT32_MAX;
        for (Direction edge = 0; edge < 4; edge++)
        {
            if ((edges & EdgeBit(edge)) == 0)
                continue;
            const auto h = GetEdgeHeightSum(shape, edge);
            if (h < bestHeight)
            {
                bestHeight = h;
                best = edge;
            }
        }
        return best;
    }

    uint8_t AutoConnectEdges(uint8_t neighbourMask)
    {
        return neighbourMask & kAllEdges;
    }

    uint8_t UpdateEdge(uint8_t edges, Direction edge, bool neighbourHasStream)
    {
        const auto bit = EdgeBit(edge);
        return static_cast<uint8_t>(neighbourHasStream ? (edges | bit) : (edges & ~bit)) & kAllEdges;
    }

    bool IsEndPiece(uint8_t edges)
    {
        return std::popcount(static_cast<unsigned>(edges & kAllEdges)) == 1;
    }
} // namespace OpenRCT2::Stream
