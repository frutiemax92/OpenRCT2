/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "CourseBoundaryAction.h"

#include "../../GameState.h"
#include "../../golf/MapBallWorld.h"
#include "../../localisation/StringIds.h"
#include "../../world/Map.h"
#include "../../world/tile_element/SurfaceElement.h"

namespace OpenRCT2::GameActions
{
    CourseBoundaryAction::CourseBoundaryAction(const CoordsXY& min, const CoordsXY& max, uint8_t mode)
        : _min(min)
        , _max(max)
        , _mode(mode)
    {
    }

    void CourseBoundaryAction::AcceptParameters(GameActionParameterVisitor& visitor)
    {
        visitor.Visit("minX", _min.x);
        visitor.Visit("minY", _min.y);
        visitor.Visit("maxX", _max.x);
        visitor.Visit("maxY", _max.y);
        visitor.Visit("mode", _mode);
    }

    uint16_t CourseBoundaryAction::GetActionFlags() const
    {
        return GameAction::GetActionFlags();
    }

    void CourseBoundaryAction::Serialise(DataSerialiser& stream)
    {
        GameAction::Serialise(stream);
        stream << DS_TAG(_min) << DS_TAG(_max) << DS_TAG(_mode);
    }

    Result CourseBoundaryAction::Query(GameState_t& gameState, Park::ParkData& park) const
    {
        if (_mode > kReset)
            return Result(Status::invalidParameters, STR_COURSE_CANT_CHANGE, kStringIdNone);
        return Result();
    }

    Result CourseBoundaryAction::Execute(GameState_t& gameState, Park::ParkData& park) const
    {
        auto res = Query(gameState, park);
        if (res.error != Status::ok)
            return res;

        auto& boundary = gameState.golf.boundary();
        if (_mode == kReset)
        {
            boundary.clear();
            return res;
        }
        const TileCoordsXY mapSize = gameState.mapSize;
        if (!boundary.defined())
            boundary.defineFrom(mapSize, [](const TileCoordsXY& tile) { return Golf::isTileOwned(tile); });
        boundary.setRegion(
            TileCoordsXY{ _min.x / kCoordsXYStep, _min.y / kCoordsXYStep },
            TileCoordsXY{ _max.x / kCoordsXYStep, _max.y / kCoordsXYStep }, _mode == kInPlay, mapSize, false);
        return res;
    }
} // namespace OpenRCT2::GameActions
