/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "LandHeightEdit.h"

#include "../../Diagnostic.h"
#include "../../GameState.h"
#include "../../OpenRCT2.h"
#include "../../localisation/StringIds.h"
#include "../../world/Map.h"
#include "../../world/MapLimits.h"
#include "../../world/MapSelection.h"
#include "../../world/TerrainHeightField.h"
#include "../../world/tile_element/SurfaceElement.h"
#include "../GameActionRunner.h"
#include "LandSetHeightAction.h"

#include <algorithm>
#include <set>

namespace OpenRCT2::GameActions
{
    // Smoothing can reach as far as the full height range divided by the 4 Q a tile may climb.
    static constexpr int32_t kSmoothingMargin = (kMaximumLandHeight / Terrain::kMaxCornerOffset) + 2;

    // Vertices of a tile touched by a selection, in the grid of Terrain::HeightField.
    static void AddSelectedVertices(
        std::vector<std::pair<int32_t, int32_t>>& out, int32_t tx, int32_t ty, MapSelectType type)
    {
        const std::pair<int32_t, int32_t> north{ tx + 1, ty + 1 }, east{ tx + 1, ty }, south{ tx, ty }, west{ tx, ty + 1 };
        switch (type)
        {
            case MapSelectType::corner0:
                out.push_back(north);
                break;
            case MapSelectType::corner1:
                out.push_back(east);
                break;
            case MapSelectType::corner2:
                out.push_back(south);
                break;
            case MapSelectType::corner3:
                out.push_back(west);
                break;
            case MapSelectType::edge0:
                out.insert(out.end(), { south, west });
                break;
            case MapSelectType::edge1:
                out.insert(out.end(), { north, west });
                break;
            case MapSelectType::edge2:
                out.insert(out.end(), { north, east });
                break;
            case MapSelectType::edge3:
                out.insert(out.end(), { east, south });
                break;
            default:
                out.insert(out.end(), { north, east, south, west });
                break;
        }
    }

    Result ChangeLandHeight(
        GameState_t& gameState, const MapRange& range, MapSelectType selectionType, int32_t stepQ, bool raise,
        bool isExecuting, CommandFlags flags, StringId errorTitle)
    {
        auto res = Result();
        res.expenditure = ExpenditureType::landscaping;

        const auto validRange = ClampRangeWithinMap(range);
        const bool needsOwnership = gLegacyScene != LegacyScene::scenarioEditor && !gameState.cheats.sandboxMode;

        std::vector<std::pair<int32_t, int32_t>> selected;
        int32_t minTx = INT32_MAX, minTy = INT32_MAX, maxTx = INT32_MIN, maxTy = INT32_MIN;
        for (int32_t y = validRange.getY1(); y <= validRange.getY2(); y += kCoordsXYStep)
        {
            for (int32_t x = validRange.getX1(); x <= validRange.getX2(); x += kCoordsXYStep)
            {
                if (!MapIsLocationValid({ x, y }) || MapGetSurfaceElementAt(CoordsXY{ x, y }) == nullptr)
                    continue;
                if (needsOwnership && !MapIsLocationInPark(CoordsXY{ x, y }))
                    continue;
                const int32_t tx = x / kCoordsXYStep, ty = y / kCoordsXYStep;
                AddSelectedVertices(selected, tx, ty, selectionType);
                minTx = std::min(minTx, tx);
                minTy = std::min(minTy, ty);
                maxTx = std::max(maxTx, tx);
                maxTy = std::max(maxTy, ty);
            }
        }
        if (selected.empty())
        {
            return Result(Status::disallowed, errorTitle, STR_LAND_NOT_OWNED_BY_PARK);
        }

        // A window of the map around the selection, big enough for the smoothing to settle inside it.
        const int32_t mapW = gameState.mapSize.x, mapH = gameState.mapSize.y;
        const int32_t x0 = std::max(0, minTx - kSmoothingMargin), y0 = std::max(0, minTy - kSmoothingMargin);
        const int32_t x1 = std::min(mapW - 1, maxTx + kSmoothingMargin), y1 = std::min(mapH - 1, maxTy + kSmoothingMargin);
        Terrain::HeightField field(x1 - x0 + 1, y1 - y0 + 1, kMaximumLandHeight, kMinimumLandHeight);
        for (int32_t ty = y0; ty <= y1; ty++)
        {
            for (int32_t tx = x0; tx <= x1; tx++)
            {
                const auto* surface = MapGetSurfaceElementAt(TileCoordsXY{ tx, ty });
                if (surface == nullptr)
                    continue;
                field.SetTile(tx - x0, ty - y0, surface->baseHeight, surface->getTerrainShape());
            }
        }

        for (auto& v : selected)
        {
            v.first -= x0;
            v.second -= y0;
        }
        const bool changed = raise ? field.Raise(selected, stepQ) : field.Lower(selected, stepQ);
        if (!changed)
        {
            return Result(Status::disallowed, errorTitle, raise ? STR_TOO_HIGH : STR_TOO_LOW);
        }

        std::set<std::pair<int32_t, int32_t>> tiles;
        for (auto [vx, vy] : field.GetChangedVertices())
        {
            for (int32_t ty = vy - 1; ty <= vy; ty++)
                for (int32_t tx = vx - 1; tx <= vx; tx++)
                    if (tx >= 0 && ty >= 0 && tx < field.GetTilesX() && ty < field.GetTilesY())
                        tiles.insert({ tx, ty });
        }

        for (auto [tx, ty] : tiles)
        {
            const auto [base, shape] = field.GetTile(tx, ty);
            const CoordsXY pos{ (tx + x0) * kCoordsXYStep, (ty + y0) * kCoordsXYStep };
            const auto* surface = MapGetSurfaceElementAt(pos);
            if (surface == nullptr || (surface->baseHeight == base && surface->getTerrainShape() == shape))
                continue;

            auto landSetHeightAction = LandSetHeightAction(pos, static_cast<uint8_t>(base), shape);
            landSetHeightAction.SetFlags(flags);
            auto result = isExecuting ? ExecuteNested(&landSetHeightAction, gameState)
                                      : QueryNested(&landSetHeightAction, gameState);
            if (result.error != Status::ok)
            {
                result.errorTitle = errorTitle;
                return result;
            }
            res.cost += result.cost;
        }
        return res;
    }
} // namespace OpenRCT2::GameActions
