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

// Footprint geometry of buildings (BLD-001, BLD-004). Directions: 0 = -x, 1 = +y, 2 = +x, 3 = -y.
namespace OpenRCT2
{
    class BuildingObject;
    struct BuildingElement;
}

namespace OpenRCT2::Building
{
    struct Part
    {
        CoordsXY tile;  // world tile
        uint8_t localX; // position in the building's own frame
        uint8_t localY;
    };

    // Where a tile of the building's own frame lands once the building is turned by `rotation` quarter turns and its
    // lowest-x, lowest-y corner sits on `origin`.
    CoordsXY LocalToWorld(const BuildingObject& object, const CoordsXY& origin, Direction rotation, int32_t localX, int32_t localY);

    // Every tile of the footprint.
    std::vector<Part> GetParts(const BuildingObject& object, const CoordsXY& origin, Direction rotation);

    // The tile just outside the entrance and the edge of the building it opens on, once placed.
    struct Entrance
    {
        CoordsXY insideTile;
        CoordsXY outsideTile;
        Direction edge;
    };
    Entrance GetEntrance(const BuildingObject& object, const CoordsXY& origin, Direction rotation);

    // The building element of a tile, or nullptr.
    BuildingElement* FindAt(const CoordsXY& tile);

    // The lowest-x, lowest-y corner of the footprint that `element` belongs to.
    CoordsXY GetOrigin(const CoordsXY& tile, const BuildingElement& element);

    // The tile of the footprint painted last for a camera rotation, i.e. the one nearest to the camera. It draws the whole
    // view of the building (BLD-001).
    CoordsXY GetPaintTile(const BuildingObject& object, const CoordsXY& origin, Direction rotation, uint8_t cameraRotation);

    // True when a path lies on the tile (BLD-004).
    bool HasPathAt(const CoordsXY& tile);
}
