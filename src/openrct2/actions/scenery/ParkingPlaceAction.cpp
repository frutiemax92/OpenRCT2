/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "ParkingPlaceAction.h"

#include "../../Cheats.h"
#include "../../GameState.h"
#include "../../OpenRCT2.h"
#include "../../localisation/StringIds.h"
#include "../../management/Finance.h"
#include "../../object/StreetObject.h"
#include "../../world/Map.h"
#include "../../world/Parking.h"
#include "../../world/Street.h"
#include "../../world/TileElementsView.h"
#include "../../world/tile_element/ParkingElement.h"
#include "../../world/tile_element/SurfaceElement.h"

namespace OpenRCT2::GameActions
{
    ParkingPlaceAction::ParkingPlaceAction(const CoordsXY& location, ObjectEntryIndex streetType, Direction facing)
        : _loc(location)
        , _streetType(streetType)
        , _facing(facing)
    {
    }

    void ParkingPlaceAction::AcceptParameters(GameActionParameterVisitor& visitor)
    {
        visitor.Visit(_loc);
        visitor.Visit("object", _streetType);
        visitor.Visit("facing", _facing);
    }

    uint16_t ParkingPlaceAction::GetActionFlags() const
    {
        return GameAction::GetActionFlags();
    }

    void ParkingPlaceAction::Serialise(DataSerialiser& stream)
    {
        GameAction::Serialise(stream);

        stream << DS_TAG(_loc) << DS_TAG(_streetType) << DS_TAG(_facing);
    }

    Result ParkingPlaceAction::Query(GameState_t& gameState, Park::ParkData& park) const
    {
        if (!MapIsLocationValid(_loc))
        {
            return Result(Status::invalidParameters, STR_CANT_BUILD_THIS_HERE, STR_OFF_EDGE_OF_MAP);
        }

        auto* object = StreetObject::GetById(_streetType);
        if (object == nullptr)
        {
            return Result(Status::invalidParameters, STR_CANT_BUILD_THIS_HERE, STR_INVALID_SELECTION_OF_OBJECTS);
        }

        auto* surface = MapGetSurfaceElementAt(_loc);
        if (surface == nullptr)
        {
            return Result(Status::invalidParameters, STR_CANT_BUILD_THIS_HERE, kStringIdNone);
        }

        const auto baseZ = surface->getBaseZ();
        if (gLegacyScene != LegacyScene::scenarioEditor && !gameState.cheats.sandboxMode
            && !MapIsLocationOwned({ _loc.x, _loc.y, baseZ }))
        {
            return Result(Status::notOwned, STR_CANT_BUILD_THIS_HERE, STR_LAND_NOT_OWNED_BY_PARK);
        }

        if (surface->getWaterHeight() > 0 && !gameState.cheats.disableClearanceChecks)
        {
            return Result(Status::disallowed, STR_CANT_BUILD_THIS_HERE, STR_CANT_BUILD_THIS_UNDERWATER);
        }

        // Cars park on level ground.
        const auto slope = Street::SlopeOf(surface->getTerrainShape());
        if (!slope.has_value() || slope->rise != 0)
        {
            return Result(Status::disallowed, STR_CANT_BUILD_THIS_HERE, STR_LAND_SLOPE_UNSUITABLE);
        }

        // A tile holds a single parking tile, and nothing solid may stand on the ground it covers.
        for (auto* element : TileElementsView<TileElement>(_loc))
        {
            const auto type = element->getType();
            if (type == TileElementType::parking || type == TileElementType::street)
            {
                return Result(Status::itemAlreadyPlaced, STR_CANT_BUILD_THIS_HERE, kStringIdNone);
            }
            if (type == TileElementType::surface || type == TileElementType::banner || element->isGhost())
                continue;
            if (!gameState.cheats.disableClearanceChecks && element->getBaseZ() < baseZ + 16)
            {
                return Result(Status::noClearance, STR_CANT_BUILD_THIS_HERE, STR_X_IN_THE_WAY);
            }
        }

        auto res = Result();
        res.cost = object->BuildCost;
        res.expenditure = ExpenditureType::landscaping;
        res.position = { _loc.x + 16, _loc.y + 16, baseZ };
        return res;
    }

    Result ParkingPlaceAction::Execute(GameState_t& gameState, Park::ParkData& park) const
    {
        auto res = Query(gameState, park);
        if (res.error != Status::ok)
            return res;

        auto* surface = MapGetSurfaceElementAt(_loc);
        const auto baseZ = surface->getBaseZ();

        auto* element = TileElementInsert<ParkingElement>({ _loc.x, _loc.y, baseZ }, 0b1111);
        if (element == nullptr)
        {
            return Result(Status::noFreeElements, STR_CANT_BUILD_THIS_HERE, STR_TILE_ELEMENT_LIMIT_REACHED);
        }

        const bool isGhost = GetFlags().has(CommandFlag::ghost);
        element->setClearanceZ(baseZ);
        element->setObjectIndex(_streetType);
        element->setFacing(_facing);
        element->setGhost(isGhost);
        MapInvalidateTileFull(_loc);

        return res;
    }
} // namespace OpenRCT2::GameActions
