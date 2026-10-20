/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "BuildingRemoveAction.h"

#include "../../Cheats.h"
#include "../../GameState.h"
#include "../../OpenRCT2.h"
#include "../../localisation/StringIds.h"
#include "../../management/Finance.h"
#include "../../object/BuildingObject.h"
#include "../../world/Building.h"
#include "../../world/Footpath.h"
#include "../../world/Map.h"
#include "../../world/TileElementsView.h"
#include "../../world/tile_element/BuildingElement.h"
#include "../../world/tile_element/PathElement.h"

namespace OpenRCT2::GameActions
{
    BuildingRemoveAction::BuildingRemoveAction(const CoordsXY& location)
        : _loc(location)
    {
    }

    void BuildingRemoveAction::AcceptParameters(GameActionParameterVisitor& visitor)
    {
        visitor.Visit(_loc);
    }

    uint16_t BuildingRemoveAction::GetActionFlags() const
    {
        return GameAction::GetActionFlags();
    }

    void BuildingRemoveAction::Serialise(DataSerialiser& stream)
    {
        GameAction::Serialise(stream);

        stream << DS_TAG(_loc);
    }

    Result BuildingRemoveAction::Query(GameState_t& gameState, Park::ParkData& park) const
    {
        if (!MapIsLocationValid(_loc))
        {
            return Result(Status::invalidParameters, STR_CANT_REMOVE_THIS, STR_OFF_EDGE_OF_MAP);
        }

        auto* element = Building::FindAt(_loc);
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

    Result BuildingRemoveAction::Execute(GameState_t& gameState, Park::ParkData& park) const
    {
        auto res = Query(gameState, park);
        if (res.error != Status::ok)
            return res;

        auto* element = Building::FindAt(_loc);
        const auto* object = element->getObject();
        const auto rotation = element->getRotation();
        const auto index = element->getObjectIndex();
        const auto origin = Building::GetOrigin(_loc, *element);

        const auto entrance = (object != nullptr && object->HasEntrance)
            ? Building::GetEntrance(*object, origin, rotation)
            : Building::Entrance{};
        const bool hasEntrance = (object != nullptr && object->HasEntrance);

        // Take out the element of every tile of the footprint; the pointers are looked up again for each one.
        if (object == nullptr)
        {
            MapInvalidateTileFull(_loc);
            TileElementRemove(element->as<TileElement>());
            return res;
        }
        for (const auto& part : Building::GetParts(*object, origin, rotation))
        {
            if (!MapIsLocationValid(part.tile))
                continue;
            auto* partElement = Building::FindAt(part.tile);
            if (partElement == nullptr || partElement->getObjectIndex() != index)
                continue;
            MapInvalidateTileFull(part.tile);
            TileElementRemove(partElement->as<TileElement>());
        }

        if (hasEntrance && !GetFlags().has(CommandFlag::ghost))
        {
            for (auto* pathElement : TileElementsView<PathElement>(entrance.outsideTile))
            {
                pathElement->setEdges(0);
                FootpathConnectEdges(entrance.outsideTile, pathElement->as<TileElement>(), GetFlags());
            }
        }
        return res;
    }
} // namespace OpenRCT2::GameActions
