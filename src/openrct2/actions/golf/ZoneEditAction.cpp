/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "ZoneEditAction.h"

#include "../../Cheats.h"
#include "../../GameState.h"
#include "../../OpenRCT2.h"
#include "../../golf/Cottages.h"
#include "../../localisation/StringIds.h"
#include "../../world/Map.h"
#include "../../world/Street.h"
#include "../../world/TileElementsView.h"
#include "../../world/tile_element/PathElement.h"
#include "../../world/tile_element/StreetElement.h"

#include <algorithm>

namespace OpenRCT2::GameActions
{
    namespace
    {
        TileCoordsXY tileOf(const CoordsXY& position)
        {
            return { position.x >> 5, position.y >> 5 };
        }

        bool hasPathOrStreet(const TileCoordsXY& tile)
        {
            const CoordsXY at{ tile.x * kCoordsXYStep, tile.y * kCoordsXYStep };
            if (!MapIsLocationValid(at))
                return false;
            for (const auto* path : TileElementsView<PathElement>(at))
                if (!path->isGhost())
                    return true;
            const auto* street = Street::FindAt(at);
            return street != nullptr && !street->isGhost();
        }

        // ECO-030: a lot is next to a path or a street when one touches the rectangle on any side.
        bool nextToRoad(const Golf::CottageLot& lot)
        {
            for (int32_t x = lot.min.x; x <= lot.max.x; x++)
                if (hasPathOrStreet({ x, lot.min.y - 1 }) || hasPathOrStreet({ x, lot.max.y + 1 }))
                    return true;
            for (int32_t y = lot.min.y; y <= lot.max.y; y++)
                if (hasPathOrStreet({ lot.min.x - 1, y }) || hasPathOrStreet({ lot.max.x + 1, y }))
                    return true;
            return false;
        }

        // A lot may not take land from a hole: no tee, cup or aim point inside it.
        bool overlapsHole(const Golf::CourseManager& golf, const Golf::CottageLot& lot)
        {
            auto inside = [&](const Golf::HolePoint& p) { return lot.contains({ p.x >> 5, p.y >> 5 }); };
            for (const auto& course : golf.courses())
            {
                for (const auto& hole : course.holes)
                {
                    for (const auto& tee : hole.tees)
                        if (tee && inside(*tee))
                            return true;
                    if (hole.cup && inside(*hole.cup))
                        return true;
                    for (const auto& aim : hole.aimPoints)
                        if (aim && inside(*aim))
                            return true;
                }
            }
            return false;
        }
    } // namespace

    ZoneEditAction::ZoneEditAction(
        ZoneEditType type, uint32_t zoneId, const CoordsXY& cornerA, const CoordsXY& cornerB, int32_t kind)
        : _type(type)
        , _zone(zoneId)
        , _cornerA(cornerA)
        , _cornerB(cornerB)
        , _kind(kind)
    {
    }

    void ZoneEditAction::AcceptParameters(GameActionParameterVisitor& visitor)
    {
        visitor.Visit("type", _type);
        visitor.Visit("zone", _zone);
        visitor.Visit("ax", _cornerA.x);
        visitor.Visit("ay", _cornerA.y);
        visitor.Visit("bx", _cornerB.x);
        visitor.Visit("by", _cornerB.y);
        visitor.Visit("kind", _kind);
    }

    uint16_t ZoneEditAction::GetActionFlags() const
    {
        return GameAction::GetActionFlags();
    }

    void ZoneEditAction::Serialise(DataSerialiser& stream)
    {
        GameAction::Serialise(stream);
        stream << DS_TAG(_type) << DS_TAG(_zone) << DS_TAG(_cornerA) << DS_TAG(_cornerB) << DS_TAG(_kind);
    }

    Result ZoneEditAction::Query(GameState_t& gameState, Park::ParkData& park) const
    {
        const auto& golf = gameState.golf;
        auto bad = [](StringId why = kStringIdNone) { return Result(Status::invalidParameters, STR_ZONE_CANT_CHANGE, why); };

        switch (_type)
        {
            case ZoneEditType::createLot:
            case ZoneEditType::createTournamentZone:
            {
                if (!MapIsLocationValid(_cornerA) || !MapIsLocationValid(_cornerB))
                    return bad(STR_OFF_EDGE_OF_MAP);
                const auto a = tileOf(_cornerA), b = tileOf(_cornerB);
                const TileCoordsXY min{ std::min(a.x, b.x), std::min(a.y, b.y) };
                const TileCoordsXY max{ std::max(a.x, b.x), std::max(a.y, b.y) };
                if (_type == ZoneEditType::createTournamentZone)
                {
                    if (_kind < 0 || _kind >= static_cast<int32_t>(Golf::TournamentZoneKind::count))
                        return bad();
                    if (golf.tournamentZones().size() >= Golf::CourseManager::kMaxTournamentZones)
                        return Result(Status::disallowed, STR_ZONE_CANT_CHANGE, STR_ZONE_TOO_MANY);
                    return Result();
                }
                if (golf.lots().size() >= Golf::CourseManager::kMaxLots)
                    return Result(Status::disallowed, STR_ZONE_CANT_CHANGE, STR_ZONE_TOO_MANY);
                Golf::CottageLot lot;
                lot.min = min;
                lot.max = max;
                switch (Golf::checkLot(lot, nextToRoad(lot), golf.lots(), overlapsHole(golf, lot)))
                {
                    case Golf::LotCheck::ok:
                        break;
                    case Golf::LotCheck::tooSmall:
                        return bad(STR_ZONE_LOT_TOO_SMALL);
                    case Golf::LotCheck::tooLarge:
                        return bad(STR_ZONE_LOT_TOO_LARGE);
                    case Golf::LotCheck::notNextToRoad:
                        return bad(STR_ZONE_LOT_NEEDS_ROAD);
                    case Golf::LotCheck::overlapsLot:
                        return bad(STR_ZONE_LOT_OVERLAPS);
                    case Golf::LotCheck::overlapsHole:
                        return bad(STR_ZONE_LOT_OVERLAPS_HOLE);
                }
                // A lot is on land the park owns.
                if (gLegacyScene != LegacyScene::scenarioEditor && !gameState.cheats.sandboxMode)
                {
                    for (int32_t y = min.y; y <= max.y; y++)
                        for (int32_t x = min.x; x <= max.x; x++)
                            if (!MapIsLocationOwned({ x * kCoordsXYStep, y * kCoordsXYStep, 0 }))
                                return bad(STR_LAND_NOT_OWNED_BY_PARK);
                }
                return Result();
            }
            case ZoneEditType::removeLot:
            {
                const auto* lot = golf.findLot(_zone);
                if (lot == nullptr)
                    return bad(STR_ZONE_NOT_FOUND);
                // A lot that has been sold belongs to its owner now.
                if (lot->sold)
                    return bad(STR_ZONE_LOT_SOLD);
                return Result();
            }
            case ZoneEditType::removeTournamentZone:
            {
                if (std::none_of(
                        golf.tournamentZones().begin(), golf.tournamentZones().end(),
                        [&](const Golf::TournamentZone& z) { return z.id == _zone; }))
                    return bad(STR_ZONE_NOT_FOUND);
                return Result();
            }
        }
        return bad();
    }

    Result ZoneEditAction::Execute(GameState_t& gameState, Park::ParkData& park) const
    {
        auto res = Query(gameState, park);
        if (res.error != Status::ok)
            return res;

        auto& golf = gameState.golf;
        ZoneEditResult data;
        data.zoneId = _zone;
        switch (_type)
        {
            case ZoneEditType::createLot:
            {
                auto* lot = golf.createLot(tileOf(_cornerA), tileOf(_cornerB));
                data.zoneId = lot->id;
                break;
            }
            case ZoneEditType::removeLot:
                golf.removeLot(_zone);
                break;
            case ZoneEditType::createTournamentZone:
            {
                auto* zone = golf.createTournamentZone(static_cast<Golf::TournamentZoneKind>(_kind), tileOf(_cornerA), tileOf(_cornerB));
                data.zoneId = zone->id;
                break;
            }
            case ZoneEditType::removeTournamentZone:
                golf.removeTournamentZone(_zone);
                break;
        }
        res.setData(std::move(data));
        return res;
    }
} // namespace OpenRCT2::GameActions
