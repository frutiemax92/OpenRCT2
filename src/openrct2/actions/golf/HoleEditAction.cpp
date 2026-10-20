/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "HoleEditAction.h"

#include "../../Cheats.h"
#include "../../Context.h"
#include "../../GameState.h"
#include "../../OpenRCT2.h"
#include "../../localisation/StringIds.h"
#include "../../object/ObjectManager.h"
#include "../../object/TerrainSurfaceObject.h"
#include "../../world/Map.h"
#include "../../world/tile_element/SurfaceElement.h"
#include "CourseEditAction.h"

namespace OpenRCT2::GameActions
{
    namespace
    {
        constexpr int32_t kMaxPar = 9;
        constexpr const char* kTeeSurfaceIdentifier = "golf.terrain_surface.tee";

        // A hole point is a position on dry land inside the map (and owned unless building freely).
        Result checkPoint(const GameState_t& gameState, const CoordsXY& location)
        {
            if (!MapIsLocationValid(location))
                return Result(Status::invalidParameters, STR_HOLE_CANT_CHANGE, STR_OFF_EDGE_OF_MAP);
            auto* surface = MapGetSurfaceElementAt(location);
            if (surface == nullptr)
                return Result(Status::invalidParameters, STR_HOLE_CANT_CHANGE, kStringIdNone);
            if (gLegacyScene != LegacyScene::scenarioEditor && !gameState.cheats.sandboxMode
                && !MapIsLocationOwned({ location.x, location.y, surface->getBaseZ() }))
                return Result(Status::notOwned, STR_HOLE_CANT_CHANGE, STR_LAND_NOT_OWNED_BY_PARK);
            // Water is a hazard, never a place for a tee, a cup or an aim point.
            if (surface->getWaterHeight() > surface->getBaseZ())
                return Result(Status::disallowed, STR_HOLE_CANT_CHANGE, STR_CANT_BUILD_THIS_UNDERWATER);
            return Result();
        }

        bool surfaceIsPuttable(const CoordsXY& location)
        {
            auto* surface = MapGetSurfaceElementAt(location);
            if (surface == nullptr)
                return false;
            const auto quadrant = Terrain::GetQuadrantAt(location.x & 31, location.y & 31);
            const auto* object = TerrainSurfaceObject::GetById(surface->getQuadrantSurfaceIndex(quadrant));
            return object != nullptr && object->Golf.isPuttable;
        }

        // Centre of the tile: a tee box is one tile and the golfers stand in the middle of it.
        CoordsXY tileCentre(const CoordsXY& location)
        {
            return { (location.x & ~31) + 16, (location.y & ~31) + 16 };
        }
    } // namespace

    HoleEditAction::HoleEditAction(uint32_t courseId, uint32_t holeId, HoleEditType type, int32_t index, const CoordsXY& location)
        : _courseId(courseId)
        , _holeId(holeId)
        , _type(type)
        , _index(index)
        , _loc(location)
    {
    }

    void HoleEditAction::AcceptParameters(GameActionParameterVisitor& visitor)
    {
        visitor.Visit("course", _courseId);
        visitor.Visit("hole", _holeId);
        visitor.Visit("type", _type);
        visitor.Visit("index", _index);
        visitor.Visit(_loc);
    }

    uint16_t HoleEditAction::GetActionFlags() const
    {
        return GameAction::GetActionFlags();
    }

    void HoleEditAction::Serialise(DataSerialiser& stream)
    {
        GameAction::Serialise(stream);
        stream << DS_TAG(_courseId) << DS_TAG(_holeId) << DS_TAG(_type) << DS_TAG(_index) << DS_TAG(_loc);
    }

    Result HoleEditAction::Query(GameState_t& gameState, Park::ParkData& park) const
    {
        const auto* course = gameState.golf.findCourse(_courseId);
        if (course == nullptr)
            return Result(Status::invalidParameters, STR_HOLE_CANT_CHANGE, STR_COURSE_NOT_FOUND);
        if (_holeId == 0)
        {
            // Only placing something starts a new hole.
            if (_type != HoleEditType::placeTee && _type != HoleEditType::placeCup && _type != HoleEditType::placeAim)
                return Result(Status::invalidParameters, STR_HOLE_CANT_CHANGE, STR_HOLE_NOT_FOUND);
            if (course->isFull())
                return Result(Status::disallowed, STR_HOLE_CANT_CHANGE, STR_COURSE_FULL);
        }
        else if (course->findHole(_holeId) == nullptr)
        {
            return Result(Status::invalidParameters, STR_HOLE_CANT_CHANGE, STR_HOLE_NOT_FOUND);
        }

        switch (_type)
        {
            case HoleEditType::placeTee:
            case HoleEditType::removeTee:
                if (_index < 0 || _index >= static_cast<int32_t>(Golf::kTeeCategoryCount))
                    return Result(Status::invalidParameters, STR_HOLE_CANT_CHANGE, kStringIdNone);
                break;
            case HoleEditType::placeAim:
            case HoleEditType::removeAim:
                if (_index < 0 || _index >= static_cast<int32_t>(Golf::kMaxAimPoints))
                    return Result(Status::invalidParameters, STR_HOLE_CANT_CHANGE, kStringIdNone);
                break;
            case HoleEditType::setPar:
                if (_index < 0 || _index > kMaxPar)
                    return Result(Status::invalidParameters, STR_HOLE_CANT_CHANGE, kStringIdNone);
                break;
            default:
                break;
        }

        if (_type == HoleEditType::placeTee || _type == HoleEditType::placeCup || _type == HoleEditType::placeAim)
        {
            auto res = checkPoint(gameState, _loc);
            if (res.error != Status::ok)
                return res;
            if (_type == HoleEditType::placeCup && !surfaceIsPuttable(_loc))
                return Result(Status::disallowed, STR_HOLE_CANT_CHANGE, STR_HOLE_CUP_NEEDS_GREEN);
        }
        return Result();
    }

    Result HoleEditAction::Execute(GameState_t& gameState, Park::ParkData& park) const
    {
        auto res = Query(gameState, park);
        if (res.error != Status::ok)
            return res;

        auto& golf = gameState.golf;
        auto* course = golf.findCourse(_courseId);
        auto* hole = _holeId == 0 ? golf.createHole(_courseId) : course->findHole(_holeId);
        if (hole == nullptr)
            return Result(Status::disallowed, STR_HOLE_CANT_CHANGE, STR_COURSE_FULL);

        switch (_type)
        {
            case HoleEditType::placeTee:
            {
                const auto centre = tileCentre(_loc);
                hole->tees[static_cast<size_t>(_index)] = centre;
                // HOL-002: the tee box forces the Tee surface onto its tile when that surface is loaded.
                auto& objectManager = GetContext()->GetObjectManager();
                if (auto* tee = ObjectManagerGetLoadedObject(ObjectEntryDescriptor(kTeeSurfaceIdentifier)))
                {
                    const auto index = objectManager.GetLoadedObjectEntryIndex(tee);
                    if (index != kObjectEntryIndexNull)
                    {
                        if (auto* surface = MapGetSurfaceElementAt(centre))
                        {
                            for (auto q : { Terrain::Quadrant::northEast, Terrain::Quadrant::southEast,
                                            Terrain::Quadrant::southWest, Terrain::Quadrant::northWest })
                                surface->setQuadrantSurfaceIndex(q, index);
                            MapInvalidateTileFull(centre);
                        }
                    }
                }
                break;
            }
            case HoleEditType::removeTee:
                hole->tees[static_cast<size_t>(_index)].reset();
                break;
            case HoleEditType::placeCup:
                hole->cup = _loc;
                break;
            case HoleEditType::removeCup:
                hole->cup.reset();
                break;
            case HoleEditType::placeAim:
                hole->aimPoints[static_cast<size_t>(_index)] = _loc;
                break;
            case HoleEditType::removeAim:
                hole->aimPoints[static_cast<size_t>(_index)].reset();
                break;
            case HoleEditType::setPar:
                hole->parOverride = _index;
                break;
        }

        CourseEditResult data;
        data.courseId = _courseId;
        data.holeId = hole->id;
        res.setData(std::move(data));
        return res;
    }
} // namespace OpenRCT2::GameActions
