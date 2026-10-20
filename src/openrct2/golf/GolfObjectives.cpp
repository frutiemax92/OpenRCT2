/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "GolfObjectives.h"

#include <algorithm>

namespace OpenRCT2::Golf
{
    namespace
    {
        bool reached(const GolfObjective& o, const GolfObjectiveProgress& p)
        {
            switch (o.type)
            {
                case GolfObjectiveType::profitBy:
                    return p.totalProfit >= o.target;
                case GolfObjectiveType::popularityBy:
                    return p.popularity >= o.target;
                case GolfObjectiveType::hostTournament:
                    // Hosting a tournament of at least the asked tier counts.
                    for (int32_t tier = static_cast<int32_t>(o.tier); tier < static_cast<int32_t>(TournamentTier::count); tier++)
                        if (p.tournamentTiersHosted & (1u << tier))
                            return true;
                    return false;
                case GolfObjectiveType::golfersPerDay:
                    return std::max(p.golfersToday, p.bestGolfersInADay) >= o.target;
                default:
                    return false;
            }
        }
    } // namespace

    ObjectiveStatus evaluateObjective(const GolfObjective& o, const GolfObjectiveProgress& p)
    {
        if (o.type == GolfObjectiveType::none)
            return ObjectiveStatus::pending;
        if (reached(o, p))
            return ObjectiveStatus::achieved;
        if (o.deadlineDay != 0 && p.today > o.deadlineDay)
            return ObjectiveStatus::failed;
        return ObjectiveStatus::pending;
    }

    int32_t objectiveProgressPercent(const GolfObjective& o, const GolfObjectiveProgress& p)
    {
        auto ratio = [](int64_t value, int64_t target) {
            if (target <= 0)
                return static_cast<int32_t>(100);
            return static_cast<int32_t>(std::clamp<int64_t>(value * 100 / target, 0, 100));
        };
        switch (o.type)
        {
            case GolfObjectiveType::profitBy:
                return ratio(p.totalProfit, o.target);
            case GolfObjectiveType::popularityBy:
                return ratio(p.popularity, o.target);
            case GolfObjectiveType::golfersPerDay:
                return ratio(std::max(p.golfersToday, p.bestGolfersInADay), o.target);
            case GolfObjectiveType::hostTournament:
                return reached(o, p) ? 100 : 0;
            default:
                return 0;
        }
    }

    const char* objectiveDescription(GolfObjectiveType type)
    {
        switch (type)
        {
            case GolfObjectiveType::profitBy:
                return "Reach a profit by the deadline";
            case GolfObjectiveType::popularityBy:
                return "Reach a popularity by the deadline";
            case GolfObjectiveType::hostTournament:
                return "Host a tournament";
            case GolfObjectiveType::golfersPerDay:
                return "Have this many golfers in a day";
            default:
                return "";
        }
    }
} // namespace OpenRCT2::Golf
