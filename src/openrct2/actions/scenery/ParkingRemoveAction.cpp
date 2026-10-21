/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "ParkingRemoveAction.h"

#include "../../Cheats.h"
#include "../../GameState.h"
#include "../../OpenRCT2.h"
#include "../../localisation/StringIds.h"
#include "../../management/Finance.h"
#include "../../object/StreetObject.h"
#include "../../world/Map.h"
#include "../../world/Parking.h"
#include "../../world/tile_element/ParkingElement.h"

namespace OpenRCT2::GameActions
{
    ParkingRemoveAction::ParkingRemoveAction(const CoordsXY& location)
        : _loc(location)
    {
    }

    void ParkingRemoveAction::AcceptParameters(GameActionParameterVisitor& visitor)
    {
        visitor.Visit(_loc);
    }

    uint16_t ParkingRemoveAction::GetActionFlags() const
    {
        return GameAction::GetActionFlags();
    }

    void ParkingRemoveAction::Serialise(DataSerialiser& stream)
    {
        GameAction::Serialise(stream);

        stream << DS_TAG(_loc);
    }

    Result ParkingRemoveAction::Query(GameState_t& gameState, Park::ParkData& park) const
    {
        if (!MapIsLocationValid(_loc))
        {
            return Result(Status::invalidParameters, STR_CANT_REMOVE_THIS, STR_OFF_EDGE_OF_MAP);
        }

        auto* element = Parking::FindAt(_loc);
        if (element == nullptr)
        {
            return Result(Status::invalidParameters, STR_CANT_REMOVE_THIS, STR_INVALID_SELECTION_OF_OBJECTS);
        }

        if (gLegacyScene != LegacyScene::scenarioEditor && !GetFlags().has(CommandFlag::ghost) && !gameState.cheats.sandboxMode
            && !MapIsLocationOwned({ _loc.x, _loc.y, element->getBaseZ() }))
        {
            return Result(Status::noClearance, STR_CANT_REMOVE_THIS, STR_LAND_NOT_OWNED_BY_PARK);
        }

        auto res = Result();
        if (const auto* object = element->getObject(); object != nullptr)
            res.cost = object->RemoveCost;
        res.expenditure = ExpenditureType::landscaping;
        res.position = { _loc.x + 16, _loc.y + 16, element->getBaseZ() };
        return res;
    }

    Result ParkingRemoveAction::Execute(GameState_t& gameState, Park::ParkData& park) const
    {
        auto res = Query(gameState, park);
        if (res.error != Status::ok)
            return res;

        auto* element = Parking::FindAt(_loc);
        const bool wasGhost = element->isGhost();
        MapInvalidateTileFull(_loc);
        TileElementRemove(element->as<TileElement>());
        (void)wasGhost;
        return res;
    }
} // namespace OpenRCT2::GameActions
