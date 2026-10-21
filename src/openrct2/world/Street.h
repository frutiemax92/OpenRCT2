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

// The street tiles of the map (STE-001 to STE-005): finding them, connecting them to their neighbours and handing them to
// the street network the cars drive on. Edges are Directions: 0 = -x, 1 = +y, 2 = +x, 3 = -y.
namespace OpenRCT2
{
    struct StreetElement;
    namespace Golf
    {
        class StreetNetwork;
    }
}

namespace OpenRCT2::Street
{
    // The street of a tile (a tile holds at most one), or nullptr.
    StreetElement* FindAt(const CoordsXY& tile);
    // Bit d set when the tile beyond edge d of `tile` holds a street whose own edge back faces it. A one-way street still
    // connects on both sides: whether cars may use it that way is decided by the network.
    uint8_t GetNeighbourMask(const CoordsXY& tile);
    // The connection set a new street at `tile` should get.
    uint8_t AutoConnect(const CoordsXY& tile);
    // Tells the street neighbours of `tile` that a street appeared (or disappeared) next to them, so they connect to it
    // (or let go of it).
    void UpdateNeighbours(const CoordsXY& tile, bool hasStreet);
    // How a road lies on a terrain shape (STE-003): a flat tile, or a ramp rising by `rise` quarter steps towards
    // `direction`. Corners raised unevenly (a saddle, a single raised corner) are no road: nullopt.
    struct RoadSlope
    {
        uint8_t rise = 0;
        Direction direction = 0;
        bool operator==(const RoadSlope&) const = default;
    };
    std::optional<RoadSlope> SlopeOf(const Terrain::TerrainShape& shape);

    // Fills a network with every street of the map (STE-001, STE-002, STE-005).
    void BuildNetwork(Golf::StreetNetwork& network);
} // namespace OpenRCT2::Street
