/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "BallPhysics.h"
#include "../world/Location.hpp"

#include <cstdint>
#include <vector>

namespace OpenRCT2::Golf
{
    // Golf cottages (ECO-030 to ECO-033): lots the player designates, which a VIP golfer buys when the course is popular
    // enough. The land price depends on the popularity and the quality of the lot.

    struct CottageLot
    {
        uint32_t id = 0;
        TileCoordsXY min;            // tile rectangle, inclusive
        TileCoordsXY max;
        bool sold = false;
        uint32_t ownerGolferId = 0;  // the VIP who bought it
        uint32_t buildingObject = 0; // the cottage placed on it
        int64_t pricePaid = 0;

        int32_t width() const
        {
            return max.x - min.x + 1;
        }
        int32_t depth() const
        {
            return max.y - min.y + 1;
        }
        bool contains(const TileCoordsXY& tile) const
        {
            return tile.x >= min.x && tile.x <= max.x && tile.y >= min.y && tile.y <= max.y;
        }
    };

    // ECO-030: a lot is a rectangle sized for a cottage footprint, next to a path or street.
    constexpr int32_t kMinLotSide = 2;
    constexpr int32_t kMaxLotSide = 6;
    enum class LotCheck : uint8_t
    {
        ok,
        tooSmall,
        tooLarge,
        notNextToRoad,
        overlapsLot,
        overlapsHole,   // a lot may not take land from a hole
    };
    LotCheck checkLot(const CottageLot& lot, bool nextToPathOrStreet, const std::vector<CottageLot>& existing, bool overlapsHole);

    // What makes a lot attractive: the view (ECO-031).
    struct LotFeatures
    {
        int32_t holesInView = 0;      // tees, greens and fairway lines within sight
        int32_t waterTiles = 0;       // water within sight
        int32_t sceneryItems = 0;     // trees and flowers within sight
        int32_t distanceToClubTiles = 20;
    };
    // 0..100.
    int32_t lotAttractiveness(const LotFeatures& features);

    // The popularity needed before a VIP buys a lot: lower for attractive lots.
    constexpr int32_t kBaseSalePopularity = 650;
    int32_t salePopularityThreshold(int32_t attractiveness);
    // Price of the land in currency units from the popularity and the lot (ECO-031).
    int64_t lotPrice(int32_t popularity, int32_t attractiveness, int32_t tiles);

    // Chance, in per million per game day, that one lot is bought (zero below the threshold).
    int32_t salePerMillionPerDay(int32_t popularity, int32_t attractiveness);

    // ECO-032: green fee a cottage owner pays, in percent of the normal fee (configurable, default free).
    constexpr int32_t kDefaultVipFeePercent = 0;
    int64_t vipGreenFee(int64_t fee, int32_t vipFeePercent);
    // VIPs from a cottage play more often: arrivals per day for each cottage owner, in per mille.
    constexpr int32_t kVipPlayChancePerMille = 600;

    // ECO-033: sold lots are locked: no terrain or scenery changes, except cosmetic paths.
    enum class LotEdit : uint8_t
    {
        terrain,
        scenery,
        building,
        path,       // cosmetic paths are still allowed
        water,
    };
    bool isEditAllowed(const std::vector<CottageLot>& lots, const TileCoordsXY& tile, LotEdit edit);
    const CottageLot* lotAt(const std::vector<CottageLot>& lots, const TileCoordsXY& tile);
} // namespace OpenRCT2::Golf
