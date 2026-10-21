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

#include <cstdint>
#include <vector>

// The parking tiles of the map and the parking lots they make (STE-010). Edges are Directions: 0 = -x, 1 = +y, 2 = +x,
// 3 = -y.
namespace OpenRCT2
{
    struct ParkingElement;
}

namespace OpenRCT2::Parking
{
    // The parking tile of a tile (a tile holds at most one), or nullptr.
    ParkingElement* FindAt(const CoordsXY& tile);

    // A parking lot as the map shows it: the tiles that share a street tile on the edge they face, and a path next to them.
    struct LotDescription
    {
        TileCoordsXY streetTile;
        TileCoordsXY pathTile;
        struct Tile
        {
            TileCoordsXY tile;
            uint8_t facing = 0;
        };
        std::vector<Tile> tiles;
    };

    // Finds the parking lots of the map (STE-010). A parking tile belongs to the lot of the street tile it faces; a lot needs
    // a path next to one of its tiles, on an edge other than the one facing the street. Tiles that face no street, and lots
    // with no path, are left out. Ghosts are ignored. The order is stable: by street tile, then by tile.
    std::vector<LotDescription> FindLots();
} // namespace OpenRCT2::Parking
