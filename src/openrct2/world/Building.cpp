/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Building.h"

#include "../object/BuildingObject.h"
#include "Map.h"
#include "TileElementsView.h"
#include "tile_element/BuildingElement.h"
#include "tile_element/PathElement.h"

namespace OpenRCT2::Building
{
    namespace
    {
        // Offset in tiles from the lowest corner of the turned footprint to a local tile.
        void localToOffset(const BuildingObject& object, Direction rotation, int32_t lx, int32_t ly, int32_t& ox, int32_t& oy)
        {
            const auto fx = object.FootprintX;
            const auto fy = object.FootprintY;
            switch (rotation & 3)
            {
                default:
                case 0:
                    ox = lx;
                    oy = ly;
                    break;
                case 1:
                    ox = fy - 1 - ly;
                    oy = lx;
                    break;
                case 2:
                    ox = fx - 1 - lx;
                    oy = fy - 1 - ly;
                    break;
                case 3:
                    ox = ly;
                    oy = fx - 1 - lx;
                    break;
            }
        }
    } // namespace

    CoordsXY LocalToWorld(const BuildingObject& object, const CoordsXY& origin, Direction rotation, int32_t localX, int32_t localY)
    {
        int32_t ox = 0, oy = 0;
        localToOffset(object, rotation, localX, localY, ox, oy);
        return { origin.x + ox * kCoordsXYStep, origin.y + oy * kCoordsXYStep };
    }

    std::vector<Part> GetParts(const BuildingObject& object, const CoordsXY& origin, Direction rotation)
    {
        std::vector<Part> parts;
        for (int32_t ly = 0; ly < object.FootprintY; ly++)
        {
            for (int32_t lx = 0; lx < object.FootprintX; lx++)
            {
                parts.push_back({ LocalToWorld(object, origin, rotation, lx, ly), static_cast<uint8_t>(lx), static_cast<uint8_t>(ly) });
            }
        }
        return parts;
    }

    Entrance GetEntrance(const BuildingObject& object, const CoordsXY& origin, Direction rotation)
    {
        Entrance entrance;
        entrance.insideTile = LocalToWorld(object, origin, rotation, object.EntranceTileX, object.EntranceTileY);
        entrance.edge = (object.EntranceEdge - rotation) & 3;
        entrance.outsideTile = { entrance.insideTile.x + CoordsDirectionDelta[entrance.edge].x,
                                 entrance.insideTile.y + CoordsDirectionDelta[entrance.edge].y };
        return entrance;
    }

    BuildingElement* FindAt(const CoordsXY& tile)
    {
        for (auto* element : TileElementsView<BuildingElement>(tile))
            return element;
        return nullptr;
    }

    CoordsXY GetOrigin(const CoordsXY& tile, const BuildingElement& element)
    {
        const auto* object = element.getObject();
        if (object == nullptr)
            return tile;
        int32_t ox = 0, oy = 0;
        localToOffset(*object, element.getRotation(), element.getPartX(), element.getPartY(), ox, oy);
        return { tile.x - ox * kCoordsXYStep, tile.y - oy * kCoordsXYStep };
    }

    bool HasPathAt(const CoordsXY& tile)
    {
        if (!MapIsLocationValid(tile))
            return false;
        for (auto* path : TileElementsView<PathElement>(tile))
        {
            (void)path;
            return true;
        }
        return false;
    }
} // namespace OpenRCT2::Building
