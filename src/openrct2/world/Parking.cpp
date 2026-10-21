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
#include <cstdlib>
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
        // Every parking tile that faces a street, in raster order.
        struct Member
        {
            TileCoordsXY tile;
            uint8_t facing;
            TileCoordsXY street;
            size_t group;
        };
        std::vector<Member> members;
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
                members.push_back({ TileCoordsXY{ x, y }, static_cast<uint8_t>(facing),
                                    TileCoordsXY{ streetCoords.x / kCoordsXYStep, streetCoords.y / kCoordsXYStep },
                                    members.size() });
            }
        }

        // Tiles next to each other, and tiles that face the same street tile, are one lot.
        std::vector<size_t> parent(members.size());
        for (size_t i = 0; i < parent.size(); i++)
            parent[i] = i;
        auto find = [&](size_t i) {
            while (parent[i] != i)
            {
                parent[i] = parent[parent[i]];
                i = parent[i];
            }
            return i;
        };
        for (size_t i = 0; i < members.size(); i++)
        {
            for (size_t j = i + 1; j < members.size(); j++)
            {
                const bool adjacent = std::abs(members[i].tile.x - members[j].tile.x)
                        + std::abs(members[i].tile.y - members[j].tile.y) == 1;
                const bool sameStreet = members[i].street == members[j].street;
                if (adjacent || sameStreet)
                    parent[find(j)] = find(i);
            }
        }

        // Lots in the order of their first tile, which the raster order of the members gives.
        std::vector<LotDescription> result;
        std::map<size_t, size_t> lotOfRoot;
        for (size_t i = 0; i < members.size(); i++)
        {
            const auto root = find(i);
            auto it = lotOfRoot.find(root);
            if (it == lotOfRoot.end())
            {
                LotDescription lot;
                lot.streetTile = members[i].street;
                it = lotOfRoot.emplace(root, result.size()).first;
                result.push_back(std::move(lot));
            }
            result[it->second].tiles.push_back({ members[i].tile, members[i].facing });
        }

        // A lot needs a path next to one of its tiles, on a side that does not face the street.
        std::vector<LotDescription> withPath;
        for (auto& lot : result)
        {
            bool found = false;
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
                withPath.push_back(std::move(lot));
        }
        return withPath;
    }
} // namespace OpenRCT2::Parking
