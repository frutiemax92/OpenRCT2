/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "StreetRoadEntranceAction.h"

#include "../../GameState.h"
#include "../../localisation/StringIds.h"
#include "../../golf/StreetNetwork.h"
#include "../../world/Map.h"
#include "../../world/Street.h"
#include "../../world/tile_element/StreetElement.h"

namespace OpenRCT2::GameActions
{
    StreetRoadEntranceAction::StreetRoadEntranceAction(const CoordsXY& location, bool entrance)
        : _loc(location)
        , _entrance(entrance)
    {
    }

    void StreetRoadEntranceAction::AcceptParameters(GameActionParameterVisitor& visitor)
    {
        visitor.Visit(_loc);
        visitor.Visit("entrance", _entrance);
    }

    uint16_t StreetRoadEntranceAction::GetActionFlags() const
    {
        return GameAction::GetActionFlags();
    }

    void StreetRoadEntranceAction::Serialise(DataSerialiser& stream)
    {
        GameAction::Serialise(stream);

        stream << DS_TAG(_loc) << DS_TAG(_entrance);
    }

    Result StreetRoadEntranceAction::Query(GameState_t& gameState, Park::ParkData& park) const
    {
        if (!MapIsLocationValid(_loc))
        {
            return Result(Status::invalidParameters, STR_CANT_BUILD_THIS_HERE, STR_OFF_EDGE_OF_MAP);
        }

        auto* street = Street::FindAt(_loc);
        if (street == nullptr)
        {
            return Result(Status::invalidParameters, STR_CANT_BUILD_THIS_HERE, STR_INVALID_SELECTION_OF_OBJECTS);
        }
        // Ghosts are previews.
        if (street->isGhost())
        {
            return Result(Status::invalidParameters, STR_CANT_BUILD_THIS_HERE, STR_INVALID_SELECTION_OF_OBJECTS);
        }

        if (_entrance && !Golf::isAtMapEdge(TileCoordsXY{ _loc }, gameState.mapSize))
        {
            return Result(Status::disallowed, STR_CANT_BUILD_THIS_HERE, STR_ROAD_ENTRANCE_AT_MAP_EDGE);
        }

        auto res = Result();
        res.position = { _loc.x + 16, _loc.y + 16, street->getBaseZ() };
        return res;
    }

    Result StreetRoadEntranceAction::Execute(GameState_t& gameState, Park::ParkData& park) const
    {
        auto res = Query(gameState, park);
        if (res.error != Status::ok)
            return res;

        Street::FindAt(_loc)->setRoadEntrance(_entrance);
        MapInvalidateTileFull(_loc);
        return res;
    }
} // namespace OpenRCT2::GameActions
