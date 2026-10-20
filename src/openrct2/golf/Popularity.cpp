/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Popularity.h"

#include <algorithm>

namespace OpenRCT2::Golf
{
    int32_t computePopularity(const PopularityInputs& in)
    {
        const int64_t happiness = std::clamp(in.averageHappiness, 0, 255) * 1000 / 255;
        const int64_t balance = std::clamp(in.difficultyBalance, 0, 100) * 10;
        const int64_t scenery = std::clamp(in.sceneryScore, 0, 100) * 10;
        const int64_t facilities = std::clamp(in.facilityScore, 0, 100) * 10;
        // 100 is the fair price and counts fully; cheap prices count a little more but never past a full score.
        const int64_t price = std::clamp(in.priceFairness, 0, 100) * 10;

        int64_t total = (happiness * kHappinessShare + balance * kBalanceShare + scenery * kSceneryShare
                         + facilities * kFacilityShare + price * kPriceShare)
            / 1000;

        // Rage quits are a strong negative: their share of the rounds, with a floor so that a single one is felt.
        if (in.recentRageQuits > 0)
        {
            const int64_t rounds = std::max(1, in.recentRounds + in.recentRageQuits);
            const int64_t share = static_cast<int64_t>(in.recentRageQuits) * 1000 / rounds; // per mille of the rounds
            const int64_t penalty = std::min<int64_t>(kRageQuitMaxPenalty, kRageQuitPenalty + share * 2);
            total -= penalty;
        }
        total += std::min(kTournamentMaxBonus, std::max(0, in.tournamentsHosted) * kTournamentBonus);
        return static_cast<int32_t>(std::clamp<int64_t>(total, 0, kPopularityMax));
    }

    int32_t movePopularity(int32_t current, int32_t target)
    {
        current = std::clamp(current, 0, kPopularityMax);
        target = std::clamp(target, 0, kPopularityMax);
        if (current < target)
            return std::min(target, current + kPopularityDailyStep);
        return std::max(target, current - kPopularityDailyStep);
    }

    void Ledger::add(const Ledger& o)
    {
        greenFees += o.greenFees;
        cartFees += o.cartFees;
        rangeBuckets += o.rangeBuckets;
        miniPuttFees += o.miniPuttFees;
        buildingServices += o.buildingServices;
        cottageSales += o.cottageSales;
        tournamentRevenue += o.tournamentRevenue;
        construction += o.construction;
        buildingUpkeep += o.buildingUpkeep;
        staffWages += o.staffWages;
        cartPurchases += o.cartPurchases;
        loanInterest += o.loanInterest;
        marketing += o.marketing;
    }

    int64_t dailyLoanInterest(int64_t loan, int32_t ratePerMille)
    {
        if (loan <= 0 || ratePerMille <= 0)
            return 0;
        return loan * ratePerMille / 1000 / (30 * 12);
    }
} // namespace OpenRCT2::Golf
