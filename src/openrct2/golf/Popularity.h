/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include <cstdint>

namespace OpenRCT2::Golf
{
    // Course popularity (ECO-001) and the daily accounts (ECO-010, ECO-011). All integer arithmetic.

    constexpr int32_t kPopularityMax = 1000;

    // What the popularity is made of, each part on its own scale so the course window can show them.
    struct PopularityInputs
    {
        int32_t averageHappiness = 128;   // 0..255, over golfers who finished a round recently
        int32_t difficultyBalance = 100;  // 0..100: how well hole difficulty matches the skill of the golfers
        int32_t sceneryScore = 0;         // 0..100: scenery near the holes (SCN-003)
        int32_t facilityScore = 0;        // 0..100: food, drink, toilets, pro shop, benches per golfer
        int32_t priceFairness = 100;      // 0..150 (Arrivals::priceFairnessPercent)
        int32_t recentRageQuits = 0;      // in the last ten game days
        int32_t recentRounds = 0;         // rounds finished in the same time
        int32_t tournamentsHosted = 0;    // in the last season
    };

    // The shares of the popularity in per mille (they add up to 1000 before the penalties and bonuses).
    constexpr int32_t kHappinessShare = 350;
    constexpr int32_t kBalanceShare = 150;
    constexpr int32_t kSceneryShare = 100;
    constexpr int32_t kFacilityShare = 100;
    constexpr int32_t kPriceShare = 300;
    // Each rage quit takes this much popularity, scaled by how rare they are among the rounds; "strong negative".
    constexpr int32_t kRageQuitPenalty = 60;
    constexpr int32_t kRageQuitMaxPenalty = 500;
    // A tournament hosted lifts the popularity by this much each, up to a cap (ECO-042).
    constexpr int32_t kTournamentBonus = 40;
    constexpr int32_t kTournamentMaxBonus = 120;

    int32_t computePopularity(const PopularityInputs& inputs);

    // The popularity moves towards the target a little each game day instead of jumping.
    constexpr int32_t kPopularityDailyStep = 25;
    int32_t movePopularity(int32_t current, int32_t target);

    // ECO-010 and ECO-011: the money of a day, by category, in currency units.
    struct Ledger
    {
        int64_t greenFees = 0;
        int64_t cartFees = 0;
        int64_t rangeBuckets = 0;
        int64_t miniPuttFees = 0;
        int64_t buildingServices = 0;
        int64_t cottageSales = 0;
        int64_t tournamentRevenue = 0;

        int64_t construction = 0;
        int64_t buildingUpkeep = 0;
        int64_t staffWages = 0;
        int64_t cartPurchases = 0;
        int64_t loanInterest = 0;
        int64_t marketing = 0;

        int64_t income() const
        {
            return greenFees + cartFees + rangeBuckets + miniPuttFees + buildingServices + cottageSales + tournamentRevenue;
        }
        int64_t costs() const
        {
            return construction + buildingUpkeep + staffWages + cartPurchases + loanInterest + marketing;
        }
        int64_t profit() const
        {
            return income() - costs();
        }
        void add(const Ledger& other);
    };

    // Yearly interest on a loan at a rate in per mille, for one game day (30 days a month, 12 months).
    int64_t dailyLoanInterest(int64_t loan, int32_t ratePerMille);
} // namespace OpenRCT2::Golf
