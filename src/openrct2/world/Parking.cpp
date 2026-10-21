/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Parking.h"

#include "../GameState.h"
#include "Map.h"
#include "Street.h"
#include "TileElementsView.h"
#include "tile_element/ParkingElement.h"
#include "tile_element/PathElement.h"
#include "tile_element/StreetElement.h"

#include <algorithm>
#include <map>

namespace OpenRCT2::Parking
{
    ParkingElement* FindAt(const CoordsXY& tile)
    {
        if (!MapIsLocationValid(tile))
            return nullptr;
        for (auto* element : TileElementsView<ParkingElement>(tile))
            return element;
        return nullptr;
    }

    namespace
    {
        bool hasPath(const CoordsXY& tile)
        {
            if (!MapIsLocationValid(tile))
                return false;
            for (const auto* element : TileElementsView<PathElement>(tile))
            {
                if (!element->isGhost())
                    return true;
            }
            return false;
        }

        CoordsXY neighbourOf(const CoordsXY& tile, Direction edge)
        {
            return { tile.x + CoordsDirectionDelta[edge].x, tile.y + CoordsDirectionDelta[edge].y };
        }
    } // namespace

    std::vector<LotDescription> FindLots()
    {
        // Lots keyed by the street tile they face, in tile order.
        std::map<std::pair<int32_t, int32_t>, LotDescription> lots;
        const auto size = getGameState().mapSize;
        for (int32_t y = 0; y < size.y; y++)
        {
            for (int32_t x = 0; x < size.x; x++)
            {
                const CoordsXY tile{ x * kCoordsXYStep, y * kCoordsXYStep };
                const auto* parking = FindAt(tile);
                if (parking == nullptr || parking->isGhost())
                    continue;
                const auto facing = parking->getFacing();
                const auto streetCoords = neighbourOf(tile, facing);
                const auto* street = Street::FindAt(streetCoords);
                if (street == nullptr || street->isGhost())
                    continue;

                const TileCoordsXY streetTile{ streetCoords.x / kCoordsXYStep, streetCoords.y / kCoordsXYStep };
                auto& lot = lots[{ streetTile.y, streetTile.x }];
                lot.streetTile = streetTile;
                lot.tiles.push_back({ TileCoordsXY{ x, y }, static_cast<uint8_t>(facing) });
            }
        }

        std::vector<LotDescription> result;
        for (auto& [key, lot] : lots)
        {
            bool found = false;
            // The path may be next to any tile of the lot, on a side that does not face the street.
            for (const auto& member : lot.tiles)
            {
                for (Direction edge = 0; edge < 4 && !found; edge++)
                {
                    if (edge == member.facing)
                        continue;
                    const auto next = neighbourOf({ member.tile.x * kCoordsXYStep, member.tile.y * kCoordsXYStep }, edge);
                    if (hasPath(next))
                    {
                        lot.pathTile = TileCoordsXY{ next.x / kCoordsXYStep, next.y / kCoordsXYStep };
                        found = true;
                    }
                }
                if (found)
                    break;
            }
            if (found)
                result.push_back(std::move(lot));
        }
        return result;
    }
} // namespace OpenRCT2::Parking
