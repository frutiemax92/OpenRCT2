/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "Tournament.h"

#include <cstdint>

namespace OpenRCT2::Golf
{
    // Scenario objectives for golf (ECO-050): reach a profit by a date, reach a popularity, host a tournament of a tier,
    // have a number of golfers per day. Evaluated every game day; the scenario framework turns the status into the usual
    // won / lost / pending.
    enum class GolfObjectiveType : uint8_t
    {
        none,
        profitBy,
        popularityBy,
        hostTournament,
        golfersPerDay,
    };

    struct GolfObjective
    {
        GolfObjectiveType type = GolfObjectiveType::none;
        int64_t target = 0;                 // profit, popularity or golfers
        TournamentTier tier = TournamentTier::local;
        uint32_t deadlineDay = 0;           // game day by which it must be reached; 0 for no deadline
    };

    struct GolfObjectiveProgress
    {
        int64_t totalProfit = 0;            // since the scenario began
        int32_t popularity = 0;
        uint8_t tournamentTiersHosted = 0;  // bit per tier hosted successfully
        int32_t golfersToday = 0;
        int32_t bestGolfersInADay = 0;
        uint32_t today = 0;
    };

    enum class ObjectiveStatus : uint8_t
    {
        pending,
        achieved,
        failed,
    };

    ObjectiveStatus evaluateObjective(const GolfObjective& objective, const GolfObjectiveProgress& progress);
    // How far along, 0..100, for the objective window.
    int32_t objectiveProgressPercent(const GolfObjective& objective, const GolfObjectiveProgress& progress);
    const char* objectiveDescription(GolfObjectiveType type);
} // namespace OpenRCT2::Golf
