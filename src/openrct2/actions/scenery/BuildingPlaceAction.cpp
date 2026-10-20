/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "BuildingPlaceAction.h"

#include "../../Cheats.h"
#include "../../GameState.h"
#include "../../OpenRCT2.h"
#include "../../localisation/StringIds.h"
#include "../../management/Finance.h"
#include "../../object/BuildingObject.h"
#include "../../world/Building.h"
#include "../../world/Map.h"
#include "../../world/TileElementsView.h"
#include "../../world/tile_element/BuildingElement.h"
#include "../../world/tile_element/SurfaceElement.h"

#include <algorithm>

namespace OpenRCT2::GameActions
{
    BuildingPlaceAction::BuildingPlaceAction(const CoordsXY& location, ObjectEntryIndex buildingType, Direction rotation)
        : _loc(location)
        , _buildingType(buildingType)
        , _rotation(rotation & 3)
    {
    }

    void BuildingPlaceAction::AcceptParameters(GameActionParameterVisitor& visitor)
    {
        visitor.Visit(_loc);
        visitor.Visit("object", _buildingType);
        visitor.Visit("rotation", _rotation);
    }

    uint16_t BuildingPlaceAction::GetActionFlags() const
    {
        return GameAction::GetActionFlags();
    }

    void BuildingPlaceAction::Serialise(DataSerialiser& stream)
    {
        GameAction::Serialise(stream);

        stream << DS_TAG(_loc) << DS_TAG(_buildingType) << DS_TAG(_rotation);
    }

    Result BuildingPlaceAction::Query(GameState_t& gameState, Park::ParkData& park) const
    {
        if (!MapIsLocationValid(_loc))
        {
            return Result(Status::invalidParameters, STR_CANT_BUILD_THIS_HERE, STR_OFF_EDGE_OF_MAP);
        }

        auto* object = BuildingObject::GetById(_buildingType);
        if (object == nullptr)
        {
            return Result(Status::invalidParameters, STR_CANT_BUILD_THIS_HERE, STR_INVALID_SELECTION_OF_OBJECTS);
        }

        const auto parts = Building::GetParts(*object, _loc, _rotation);

        // Every tile must exist, be dry, owned and (unless the object allows slopes) flat, all at one height.
        int32_t lowest = INT32_MAX;
        int32_t highest = INT32_MIN;
        for (const auto& part : parts)
        {
            if (!MapIsLocationValid(part.tile))
                return Result(Status::invalidParameters, STR_CANT_BUILD_THIS_HERE, STR_OFF_EDGE_OF_MAP);

            auto* surface = MapGetSurfaceElementAt(part.tile);
            if (surface == nullptr)
                return Result(Status::invalidParameters, STR_CANT_BUILD_THIS_HERE, kStringIdNone);

            const auto z = surface->getBaseZ();
            if (gLegacyScene != LegacyScene::scenarioEditor && !gameState.cheats.sandboxMode
                && !MapIsLocationOwned({ part.tile.x, part.tile.y, z }))
            {
                return Result(Status::notOwned, STR_CANT_BUILD_THIS_HERE, STR_LAND_NOT_OWNED_BY_PARK);
            }
            if (surface->getWaterHeight() > 0 && !gameState.cheats.disableClearanceChecks)
                return Result(Status::disallowed, STR_CANT_BUILD_THIS_HERE, STR_CANT_BUILD_THIS_UNDERWATER);
            if (!object->AllowSloped && surface->getSlope() != 0)
                return Result(Status::disallowed, STR_CANT_BUILD_THIS_HERE, STR_LEVEL_LAND_REQUIRED);

            lowest = std::min(lowest, z);
            highest = std::max(highest, z);
        }
        if (!object->AllowSloped && lowest != highest)
            return Result(Status::disallowed, STR_CANT_BUILD_THIS_HERE, STR_LEVEL_LAND_REQUIRED);

        // Nothing may stand where the building does.
        const auto baseZ = highest;
        const auto clearanceZ = baseZ + object->Height;
        for (const auto& part : parts)
        {
            for (auto* element : TileElementsView<TileElement>(part.tile))
            {
                const auto type = element->getType();
                if (type == TileElementType::surface || type == TileElementType::banner || element->isGhost())
                    continue;
                if (gameState.cheats.disableClearanceChecks)
                    continue;
                if (type == TileElementType::building)
                    return Result(Status::itemAlreadyPlaced, STR_CANT_BUILD_THIS_HERE, kStringIdNone);
                if (element->getBaseZ() < clearanceZ && element->getClearanceZ() > baseZ)
                    return Result(Status::noClearance, STR_CANT_BUILD_THIS_HERE, STR_X_IN_THE_WAY);
            }
        }

        // The entrance must open onto a path (BLD-004).
        if (object->HasEntrance)
        {
            const auto entrance = Building::GetEntrance(*object, _loc, _rotation);
            if (!Building::HasPathAt(entrance.outsideTile))
                return Result(Status::disallowed, STR_CANT_BUILD_THIS_HERE, STR_BUILDING_ENTRANCE_NEEDS_PATH);
        }

        auto res = Result();
        res.cost = object->BuildCost;
        res.expenditure = ExpenditureType::landscaping;
        res.position = { _loc.x + 16, _loc.y + 16, baseZ };
        return res;
    }

    Result BuildingPlaceAction::Execute(GameState_t& gameState, Park::ParkData& park) const
    {
        auto res = Query(gameState, park);
        if (res.error != Status::ok)
            return res;

        auto* object = BuildingObject::GetById(_buildingType);
        const auto parts = Building::GetParts(*object, _loc, _rotation);

        int32_t baseZ = INT32_MIN;
        for (const auto& part : parts)
            baseZ = std::max(baseZ, MapGetSurfaceElementAt(part.tile)->getBaseZ());

        const bool isGhost = GetFlags().has(CommandFlag::ghost);
        for (const auto& part : parts)
        {
            auto* element = TileElementInsert<BuildingElement>({ part.tile.x, part.tile.y, baseZ }, 0b1111);
            if (element == nullptr)
            {
                return Result(Status::noFreeElements, STR_CANT_BUILD_THIS_HERE, STR_TILE_ELEMENT_LIMIT_REACHED);
            }
            element->setClearanceZ(baseZ + object->Height);
            element->setObjectIndex(_buildingType);
            element->setRotation(_rotation);
            element->setPart(part.localX, part.localY);
            element->setGhost(isGhost);
            MapInvalidateTileFull(part.tile);
        }
        return res;
    }
} // namespace OpenRCT2::GameActions
