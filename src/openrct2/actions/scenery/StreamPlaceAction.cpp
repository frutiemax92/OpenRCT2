/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "StreamPlaceAction.h"

#include "../../Cheats.h"
#include "../../GameState.h"
#include "../../OpenRCT2.h"
#include "../../localisation/StringIds.h"
#include "../../management/Finance.h"
#include "../../object/StreamObject.h"
#include "../../world/Map.h"
#include "../../world/Stream.h"
#include "../../world/TileElementsView.h"
#include "../../world/tile_element/StreamElement.h"
#include "../../world/tile_element/SurfaceElement.h"

namespace OpenRCT2::GameActions
{
    StreamPlaceAction::StreamPlaceAction(const CoordsXY& location, ObjectEntryIndex streamType, Direction flowDirection)
        : _loc(location)
        , _streamType(streamType)
        , _flowDirection(flowDirection)
    {
    }

    void StreamPlaceAction::AcceptParameters(GameActionParameterVisitor& visitor)
    {
        visitor.Visit(_loc);
        visitor.Visit("object", _streamType);
        visitor.Visit("flowDirection", _flowDirection);
    }

    uint16_t StreamPlaceAction::GetActionFlags() const
    {
        return GameAction::GetActionFlags();
    }

    void StreamPlaceAction::Serialise(DataSerialiser& stream)
    {
        GameAction::Serialise(stream);

        stream << DS_TAG(_loc) << DS_TAG(_streamType) << DS_TAG(_flowDirection);
    }

    Result StreamPlaceAction::Query(GameState_t& gameState, Park::ParkData& park) const
    {
        if (!MapIsLocationValid(_loc))
        {
            return Result(Status::invalidParameters, STR_CANT_BUILD_THIS_HERE, STR_OFF_EDGE_OF_MAP);
        }

        auto* object = StreamObject::GetById(_streamType);
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

        // A tile holds a single stream, and nothing solid may stand on the ground it covers.
        for (auto* element : TileElementsView<TileElement>(_loc))
        {
            const auto type = element->getType();
            if (type == TileElementType::stream)
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

    Result StreamPlaceAction::Execute(GameState_t& gameState, Park::ParkData& park) const
    {
        auto res = Query(gameState, park);
        if (res.error != Status::ok)
            return res;

        auto* surface = MapGetSurfaceElementAt(_loc);
        const auto baseZ = surface->getBaseZ();

        auto* element = TileElementInsert<StreamElement>({ _loc.x, _loc.y, baseZ }, 0b1111);
        if (element == nullptr)
        {
            return Result(Status::noFreeElements, STR_CANT_BUILD_THIS_HERE, STR_TILE_ELEMENT_LIMIT_REACHED);
        }

        const bool isGhost = GetFlags().has(CommandFlag::ghost);
        element->setClearanceZ(baseZ);
        element->setObjectIndex(_streamType);
        element->setFlowDirection(_flowDirection);
        element->setEdges(Stream::AutoConnectEdges(Stream::GetNeighbourMask(_loc)));
        element->setGhost(isGhost);

        if (!isGhost)
        {
            Stream::UpdateNeighbours(_loc, true);
        }
        MapInvalidateTileFull(_loc);

        return res;
    }
} // namespace OpenRCT2::GameActions
