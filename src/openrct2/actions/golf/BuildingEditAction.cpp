/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "BuildingEditAction.h"

#include "../../GameState.h"
#include "../../golf/GolfRuntime.h"
#include "../../localisation/StringIds.h"
#include "../../object/BuildingObject.h"
#include "../../world/Building.h"
#include "../../world/Map.h"
#include "../../world/tile_element/BuildingElement.h"

namespace OpenRCT2::GameActions
{
    namespace
    {
        constexpr int64_t kMaxServicePrice = 100000;
    }

    BuildingEditAction::BuildingEditAction(const CoordsXY& location, BuildingEditType type, int64_t value)
        : _loc(location)
        , _type(type)
        , _value(value)
    {
    }

    void BuildingEditAction::AcceptParameters(GameActionParameterVisitor& visitor)
    {
        visitor.Visit(_loc);
        visitor.Visit("type", _type);
        visitor.Visit("value", _value);
    }

    uint16_t BuildingEditAction::GetActionFlags() const
    {
        return GameAction::GetActionFlags();
    }

    void BuildingEditAction::Serialise(DataSerialiser& stream)
    {
        GameAction::Serialise(stream);
        stream << DS_TAG(_loc) << DS_TAG(_type) << DS_TAG(_value);
    }

    Result BuildingEditAction::Query(GameState_t& gameState, Park::ParkData& park) const
    {
        auto bad = [](StringId why = kStringIdNone) { return Result(Status::invalidParameters, STR_BUILDING_CANT_CHANGE, why); };
        if (!MapIsLocationValid(_loc))
            return bad(STR_OFF_EDGE_OF_MAP);
        const auto* element = Building::FindAt(_loc);
        const auto* object = element != nullptr ? element->getObject() : nullptr;
        if (object == nullptr)
            return bad(STR_BUILDING_NOT_FOUND);
        // Only buildings that serve golfers have anything to set.
        if (!object->Services.hasAny(
                BuildingService::food, BuildingService::drink, BuildingService::toilet, BuildingService::proShop,
                BuildingService::rest))
            return bad(STR_BUILDING_NO_SERVICES);
        switch (_type)
        {
            case BuildingEditType::setClosed:
                if (_value != 0 && _value != 1)
                    return bad();
                break;
            case BuildingEditType::setPrice:
                if (_value < -1 || _value > kMaxServicePrice)
                    return bad();
                break;
            default:
                return bad();
        }
        return Result();
    }

    Result BuildingEditAction::Execute(GameState_t& gameState, Park::ParkData& park) const
    {
        auto res = Query(gameState, park);
        if (res.error != Status::ok)
            return res;

        const auto* element = Building::FindAt(_loc);
        const auto origin = Building::GetOrigin(_loc, *element);
        auto& director = Golf::runtimeOf(gameState).director();
        auto& book = director.buildings[Golf::buildingKey(origin.x / 32, origin.y / 32)];
        if (_type == BuildingEditType::setClosed)
            book.closed = _value != 0;
        else
            book.price = _value;
        return res;
    }
} // namespace OpenRCT2::GameActions
