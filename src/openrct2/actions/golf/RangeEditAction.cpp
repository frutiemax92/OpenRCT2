/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "RangeEditAction.h"

#include "../../GameState.h"
#include "../../golf/DrivingRange.h"
#include "../../localisation/StringIds.h"
#include "../../world/Map.h"
#include "../../world/tile_element/SurfaceElement.h"

#include <algorithm>

namespace OpenRCT2::GameActions
{
    namespace
    {
        constexpr int64_t kMaxBucketPrice = 100000;
        constexpr int64_t kMaxBalls = 100;

        TileCoordsXY tileOf(const CoordsXY& position)
        {
            return { position.x >> 5, position.y >> 5 };
        }
    } // namespace

    RangeEditAction::RangeEditAction(
        RangeEditType type, uint32_t rangeId, const CoordsXY& cornerA, const CoordsXY& cornerB, int64_t value)
        : _type(type)
        , _rangeId(rangeId)
        , _cornerA(cornerA)
        , _cornerB(cornerB)
        , _value(value)
    {
    }

    void RangeEditAction::AcceptParameters(GameActionParameterVisitor& visitor)
    {
        visitor.Visit("type", _type);
        visitor.Visit("range", _rangeId);
        visitor.Visit("ax", _cornerA.x);
        visitor.Visit("ay", _cornerA.y);
        visitor.Visit("bx", _cornerB.x);
        visitor.Visit("by", _cornerB.y);
        visitor.Visit("value", _value);
    }

    uint16_t RangeEditAction::GetActionFlags() const
    {
        return GameAction::GetActionFlags();
    }

    void RangeEditAction::Serialise(DataSerialiser& stream)
    {
        GameAction::Serialise(stream);
        stream << DS_TAG(_type) << DS_TAG(_rangeId) << DS_TAG(_cornerA) << DS_TAG(_cornerB) << DS_TAG(_value);
    }

    Result RangeEditAction::Query(GameState_t& gameState, Park::ParkData& park) const
    {
        const auto& golf = gameState.golf;
        auto bad = [](StringId why = kStringIdNone) { return Result(Status::invalidParameters, STR_RANGE_CANT_CHANGE, why); };

        const bool needsCorners = _type == RangeEditType::create || _type == RangeEditType::setArea;
        if (needsCorners && (!MapIsLocationValid(_cornerA) || !MapIsLocationValid(_cornerB)))
            return bad(STR_OFF_EDGE_OF_MAP);

        if (_type == RangeEditType::create)
        {
            if (golf.ranges().size() >= Golf::CourseManager::kMaxRanges)
                return Result(Status::disallowed, STR_RANGE_CANT_CHANGE, STR_RANGE_TOO_MANY);
            return Result();
        }

        const auto* range = golf.findRange(_rangeId);
        if (range == nullptr)
            return bad(STR_RANGE_NOT_FOUND);

        switch (_type)
        {
            case RangeEditType::addHittingTile:
            case RangeEditType::removeHittingTile:
            {
                if (!MapIsLocationValid(_cornerA))
                    return bad(STR_OFF_EDGE_OF_MAP);
                if (!range->containsTile(tileOf(_cornerA)))
                    return bad(STR_RANGE_TILE_OUTSIDE);
                if (_type == RangeEditType::addHittingTile)
                {
                    const auto* surface = MapGetSurfaceElementAt(_cornerA);
                    if (surface == nullptr || surface->getWaterHeight() > 0)
                        return bad(STR_CANT_BUILD_THIS_UNDERWATER);
                }
                break;
            }
            case RangeEditType::setPrice:
                if (_value < 0 || _value > kMaxBucketPrice)
                    return bad();
                break;
            case RangeEditType::setBalls:
                if (_value < 1 || _value > kMaxBalls)
                    return bad();
                break;
            case RangeEditType::setDirection:
                if (_value < 0 || _value > 359)
                    return bad();
                break;
            default:
                break;
        }
        return Result();
    }

    Result RangeEditAction::Execute(GameState_t& gameState, Park::ParkData& park) const
    {
        auto res = Query(gameState, park);
        if (res.error != Status::ok)
            return res;

        auto& golf = gameState.golf;
        RangeEditResult data;
        data.rangeId = _rangeId;

        if (_type == RangeEditType::create)
        {
            auto* range = golf.createRange(tileOf(_cornerA), tileOf(_cornerB));
            if (range == nullptr)
                return Result(Status::disallowed, STR_RANGE_CANT_CHANGE, STR_RANGE_TOO_MANY);
            // DRV-005: the golfers shoot towards the middle of the area from the tiles they are given.
            data.rangeId = range->id;
            res.setData(std::move(data));
            return res;
        }
        if (_type == RangeEditType::remove)
        {
            golf.removeRange(_rangeId);
            res.setData(std::move(data));
            return res;
        }

        auto* range = golf.findRange(_rangeId);
        switch (_type)
        {
            case RangeEditType::setArea:
            {
                const auto a = tileOf(_cornerA), b = tileOf(_cornerB);
                range->areaMin = { std::min(a.x, b.x), std::min(a.y, b.y) };
                range->areaMax = { std::max(a.x, b.x), std::max(a.y, b.y) };
                // The tiles that fell outside the area are no longer hitting tiles.
                range->hittingTiles.erase(
                    std::remove_if(
                        range->hittingTiles.begin(), range->hittingTiles.end(),
                        [&](const Golf::HolePoint& p) { return !range->containsTile(tileOf({ p.x, p.y })); }),
                    range->hittingTiles.end());
                break;
            }
            case RangeEditType::addHittingTile:
            {
                const Golf::HolePoint centre{ (_cornerA.x & ~31) + 16, (_cornerA.y & ~31) + 16 };
                const bool known = std::any_of(
                    range->hittingTiles.begin(), range->hittingTiles.end(),
                    [&](const Golf::HolePoint& p) { return p.x == centre.x && p.y == centre.y; });
                if (!known)
                    range->hittingTiles.push_back(centre);
                break;
            }
            case RangeEditType::removeHittingTile:
            {
                const Golf::HolePoint centre{ (_cornerA.x & ~31) + 16, (_cornerA.y & ~31) + 16 };
                range->hittingTiles.erase(
                    std::remove_if(
                        range->hittingTiles.begin(), range->hittingTiles.end(),
                        [&](const Golf::HolePoint& p) { return p.x == centre.x && p.y == centre.y; }),
                    range->hittingTiles.end());
                break;
            }
            case RangeEditType::setPrice:
                range->bucketPrice = _value;
                break;
            case RangeEditType::setBalls:
                range->ballsPerBucket = static_cast<int32_t>(_value);
                break;
            case RangeEditType::setDirection:
                range->shootingDirectionDegrees = static_cast<int32_t>(_value);
                range->autoDirection = false;
                break;
            case RangeEditType::autoDirection:
                range->autoDirection = true;
                break;
            default:
                break;
        }
        res.setData(std::move(data));
        return res;
    }
} // namespace OpenRCT2::GameActions
