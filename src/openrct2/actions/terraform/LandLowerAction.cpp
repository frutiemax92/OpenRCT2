/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "LandLowerAction.h"

#include "../../GameState.h"
#include "../../OpenRCT2.h"
#include "../../audio/Audio.h"
#include "../../localisation/StringIds.h"
#include "../../management/Finance.h"
#include "../../ride/RideConstruction.h"
#include "../../world/Map.h"
#include "../../world/MapSelection.h"
#include "../../world/SurfaceData.h"
#include "../../world/tile_element/Slope.h"
#include "../../world/tile_element/SurfaceElement.h"
#include "../GameActionRunner.h"
#include "LandHeightEdit.h"
#include "LandSetHeightAction.h"

namespace OpenRCT2::GameActions
{
    LandLowerAction::LandLowerAction(const CoordsXY& coords, MapRange range, MapSelectType selectionType, uint8_t stepQ)
        : _coords(coords)
        , _range(range)
        , _selectionType(selectionType)
        , _stepQ(stepQ)
    {
    }

    void LandLowerAction::AcceptParameters(GameActionParameterVisitor& visitor)
    {
        visitor.Visit(_coords);
        visitor.Visit(_range);
        visitor.Visit("selectionType", _selectionType);
        visitor.Visit("stepQ", _stepQ);
    }

    uint16_t LandLowerAction::GetActionFlags() const
    {
        return GameAction::GetActionFlags();
    }

    void LandLowerAction::Serialise(DataSerialiser& stream)
    {
        GameAction::Serialise(stream);

        stream << DS_TAG(_coords) << DS_TAG(_range) << DS_TAG(_selectionType) << DS_TAG(_stepQ);
    }

    Result LandLowerAction::Query(GameState_t& gameState, Park::ParkData& park) const
    {
        return QueryExecute(gameState, false);
    }

    Result LandLowerAction::Execute(GameState_t& gameState, Park::ParkData& park) const
    {
        return QueryExecute(gameState, true);
    }

    Result LandLowerAction::QueryExecute(GameState_t& gameState, bool isExecuting) const
    {
        if (_stepQ == 0 || _stepQ > Terrain::kMaxCornerOffset)
        {
            return Result(Status::invalidParameters, STR_CANT_LOWER_LAND_HERE, STR_ERR_INVALID_PARAMETER);
        }

        const auto position = CoordsXYZ{ _coords.x, _coords.y, TileElementHeight(_coords) };
        if (isExecuting)
        {
            Audio::Play3D(Audio::SoundId::placeItem, position);
        }

        auto res = ChangeLandHeight(gameState, _range, _selectionType, _stepQ, false, isExecuting, GetFlags(), STR_CANT_LOWER_LAND_HERE);
        res.position = position;
        res.expenditure = ExpenditureType::landscaping;
        if (res.error != Status::ok)
            return res;

        // Force ride construction to recheck area
        _currentTrackSelectionFlags.set(TrackSelectionFlag::recheck);

        return res;
    }
} // namespace OpenRCT2::GameActions
