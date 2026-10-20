/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Cottages.h"

#include <algorithm>

namespace OpenRCT2::Golf
{
    LotCheck checkLot(const CottageLot& lot, bool nextToPathOrStreet, const std::vector<CottageLot>& existing, bool overlapsHole)
    {
        if (lot.width() < kMinLotSide || lot.depth() < kMinLotSide)
            return LotCheck::tooSmall;
        if (lot.width() > kMaxLotSide || lot.depth() > kMaxLotSide)
            return LotCheck::tooLarge;
        if (!nextToPathOrStreet)
            return LotCheck::notNextToRoad;
        for (const auto& other : existing)
        {
            if (other.id == lot.id)
                continue;
            if (lot.min.x <= other.max.x && lot.max.x >= other.min.x && lot.min.y <= other.max.y && lot.max.y >= other.min.y)
                return LotCheck::overlapsLot;
        }
        if (overlapsHole)
            return LotCheck::overlapsHole;
        return LotCheck::ok;
    }

    int32_t lotAttractiveness(const LotFeatures& f)
    {
        // The view counts most; water and scenery add; being far from the club takes a little off.
        int32_t score = std::min(40, f.holesInView * 8) + std::min(25, f.waterTiles) + std::min(25, f.sceneryItems * 2);
        score += 10 - std::min(10, std::max(0, f.distanceToClubTiles - 10) / 3);
        return std::clamp(score, 0, 100);
    }

    int32_t salePopularityThreshold(int32_t attractiveness)
    {
        // 650 for an unremarkable lot, down to 350 for the best.
        return kBaseSalePopularity - std::clamp(attractiveness, 0, 100) * 3;
    }

    int64_t lotPrice(int32_t popularity, int32_t attractiveness, int32_t tiles)
    {
        const int64_t perTile = 400 + static_cast<int64_t>(std::clamp(popularity, 0, 1000)) * 2
            + static_cast<int64_t>(std::clamp(attractiveness, 0, 100)) * 12;
        return perTile * std::max(0, tiles);
    }

    int32_t salePerMillionPerDay(int32_t popularity, int32_t attractiveness)
    {
        const int32_t threshold = salePopularityThreshold(attractiveness);
        if (popularity < threshold)
            return 0;
        // 2% a day just over the threshold, growing with the margin up to 20% a day.
        return std::min(200000, 20000 + (popularity - threshold) * 600);
    }

    int64_t vipGreenFee(int64_t fee, int32_t vipFeePercent)
    {
        return fee * std::clamp(vipFeePercent, 0, 100) / 100;
    }

    const CottageLot* lotAt(const std::vector<CottageLot>& lots, const TileCoordsXY& tile)
    {
        for (const auto& lot : lots)
            if (lot.contains(tile))
                return &lot;
        return nullptr;
    }

    bool isEditAllowed(const std::vector<CottageLot>& lots, const TileCoordsXY& tile, LotEdit edit)
    {
        const auto* lot = lotAt(lots, tile);
        if (lot == nullptr || !lot->sold)
            return true;
        return edit == LotEdit::path;
    }
} // namespace OpenRCT2::Golf
