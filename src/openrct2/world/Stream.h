/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "Location.hpp"
#include "TerrainShape.h"

#include <cstdint>
#include <optional>

// Connection and flow rules of water streams (STR-002, STR-003). Edges are Directions: 0 = -x, 1 = +y, 2 = +x, 3 = -y.
namespace OpenRCT2
{
    struct StreamElement;
    namespace Golf
    {
        class StreetNetwork;
    }
}

namespace OpenRCT2::Stream
{
    constexpr uint8_t kAllEdges = 0b1111;

    constexpr uint8_t EdgeBit(Direction edge)
    {
        return static_cast<uint8_t>(1 << (edge & 3));
    }

    constexpr Direction OppositeEdge(Direction edge)
    {
        return (edge + 2) & 3;
    }

    // Sum of the two corner offsets (in Q) of an edge, so edges can be compared without halving.
    int32_t GetEdgeHeightSum(const Terrain::TerrainShape& shape, Direction edge);

    // The edge the water flows out through on this terrain: the lowest of the four edges. None when every edge is
    // equally high (flat, or a saddle), where the placement order decides.
    std::optional<Direction> GetDownhillEdge(const Terrain::TerrainShape& shape);

    // The direction the water flows through the tile (STR-003). The downhill edge wins when it is connected, otherwise the
    // lowest connected edge. Flat tiles, and slopes where no edge is lower than the others, use the stored direction.
    Direction ResolveFlowDirection(const Terrain::TerrainShape& shape, uint8_t edges, Direction storedFlow);

    // The connection set of a new stream: the edges whose neighbour holds a stream (STR-002). neighbourMask has bit d set
    // when the tile beyond edge d contains a stream.
    uint8_t AutoConnectEdges(uint8_t neighbourMask);

    // The connection set of an existing stream after the neighbour beyond edge changed.
    uint8_t UpdateEdge(uint8_t edges, Direction edge, bool neighbourHasStream);

    // True for an end piece (a source or a mouth): exactly one connection.
    bool IsEndPiece(uint8_t edges);

    // Map helpers. FindAt returns the stream of a tile (a tile holds at most one), or nullptr.
    StreamElement* FindAt(const CoordsXY& tile);
    // Bit d set when the tile beyond edge d of `tile` contains a stream.
    uint8_t GetNeighbourMask(const CoordsXY& tile);
    // Tells the stream neighbours of `tile` that a stream appeared (or disappeared) next to them.
    void UpdateNeighbours(const CoordsXY& tile, bool hasStream);

    // STR-007: Golfers walk across streams only on paths/bridges; vehicles only on streets/bridges.
    bool HasPathOrBridge(const CoordsXY& tile);
    bool CanGolferCross(const CoordsXY& tile);
    bool CanVehicleCross(const CoordsXY& tile, const Golf::StreetNetwork* streets = nullptr);
    bool CanMove(const CoordsXY& from, const CoordsXY& to, bool isVehicle, const Golf::StreetNetwork* streets = nullptr);
} // namespace OpenRCT2::Stream
