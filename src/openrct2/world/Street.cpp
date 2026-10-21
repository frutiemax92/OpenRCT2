/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Street.h"

#include "../GameState.h"
#include "../golf/StreetNetwork.h"
#include "../object/StreetObject.h"
#include "Map.h"
#include "TileElementsView.h"
#include "tile_element/StreetElement.h"

namespace OpenRCT2::Street
{
    StreetElement* FindAt(const CoordsXY& tile)
    {
        if (!MapIsLocationValid(tile))
            return nullptr;
        for (auto* element : TileElementsView<StreetElement>(tile))
            return element;
        return nullptr;
    }

    uint8_t GetNeighbourMask(const CoordsXY& tile)
    {
        uint8_t mask = 0;
        for (Direction edge = 0; edge < 4; edge++)
        {
            const auto neighbour = CoordsXY{ tile.x + CoordsDirectionDelta[edge].x, tile.y + CoordsDirectionDelta[edge].y };
            // A ghost is a preview: nothing connects to it.
            const auto* street = FindAt(neighbour);
            if (street != nullptr && !street->isGhost())
                mask |= Golf::edgeBit(edge);
        }
        return mask;
    }

    uint8_t AutoConnect(const CoordsXY& tile)
    {
        return Golf::autoConnect(GetNeighbourMask(tile));
    }

    void UpdateNeighbours(const CoordsXY& tile, bool hasStreet)
    {
        for (Direction edge = 0; edge < 4; edge++)
        {
            const auto neighbour = CoordsXY{ tile.x + CoordsDirectionDelta[edge].x, tile.y + CoordsDirectionDelta[edge].y };
            if (!MapIsLocationValid(neighbour))
                continue;
            if (auto* element = FindAt(neighbour); element != nullptr)
            {
                // The edge of the neighbour that faces this tile.
                element->setConnection(Golf::oppositeDirection(edge), hasStreet);
                MapInvalidateTileFull(neighbour);
            }
        }
    }

    void BuildNetwork(Golf::StreetNetwork& network)
    {
        const auto size = getGameState().mapSize;
        for (int32_t y = 0; y < size.y; y++)
        {
            for (int32_t x = 0; x < size.x; x++)
            {
                const CoordsXY tile{ x * kCoordsXYStep, y * kCoordsXYStep };
                const auto* element = FindAt(tile);
                if (element == nullptr || element->isGhost())
                    continue;
                const auto* object = element->getObject();
                Golf::StreetTile out;
                out.position = TileCoordsXY{ x, y };
                out.connections = element->getConnections();
                out.oneWay = object != nullptr && object->OneWay;
                out.flow = element->getFlowDirection();
                out.height = element->getBaseZ();
                out.speedLimitKph = object != nullptr ? object->SpeedLimitKph : kStreetDefaultSpeedLimitKph;
                out.roadEntrance = element->isRoadEntrance();
                network.setTile(out);
            }
        }
    }
} // namespace OpenRCT2::Street
