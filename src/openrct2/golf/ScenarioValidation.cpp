/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "ScenarioValidation.h"

#include "../GameState.h"
#include "Hole.h"

#include <sstream>

namespace OpenRCT2::Golf
{
    ScenarioValidationReport validateScenario(
        const GameState_t& gameState,
        const StreetNetwork* streets,
        const TrafficSimulation* traffic,
        int32_t maxWalkWithoutPathYards)
    {
        ScenarioValidationReport report;

        // 1. Check road entrance
        bool hasRoadEntrance = false;
        if (streets != nullptr && !streets->roadEntrances().empty())
        {
            hasRoadEntrance = true;
        }
        if (!hasRoadEntrance)
        {
            report.passed = false;
            report.issues.push_back({
                ScenarioWarning::noRoadEntrance,
                "No road entrance designated for vehicles to arrive",
                0,
                0,
            });
        }

        // 2. Check parking spaces
        int32_t parkingSpaces = 0;
        if (traffic != nullptr)
        {
            for (const auto& lot : traffic->lots())
            {
                parkingSpaces += static_cast<int32_t>(lot.spaces.size());
            }
        }
        if (parkingSpaces == 0)
        {
            report.passed = false;
            report.issues.push_back({
                ScenarioWarning::noParking,
                "No parking spaces designated for visitor cars",
                0,
                0,
            });
        }

        // 3. Check 18-hole course
        const auto& courses = gameState.golf.courses();
        bool has18HoleCourse = false;
        size_t maxHolesFound = 0;
        for (const auto& course : courses)
        {
            if (course.kind == CourseKind::regular)
            {
                size_t playable = course.playableHoles();
                if (playable > maxHolesFound)
                {
                    maxHolesFound = playable;
                }
                if (playable >= 18)
                {
                    has18HoleCourse = true;
                    break;
                }
            }
        }

        if (!has18HoleCourse)
        {
            report.passed = false;
            std::ostringstream ss;
            if (courses.empty())
            {
                ss << "No golf course built (18 complete holes required)";
            }
            else
            {
                ss << "No 18-hole course found (best regular course has " << maxHolesFound << " of 18 complete holes)";
            }
            report.issues.push_back({
                ScenarioWarning::fewerThan18Holes,
                ss.str(),
                0,
                0,
            });
        }

        // 4. Check consecutive hole connectivity (path/distance from cup to next tee)
        for (const auto& course : courses)
        {
            for (size_t i = 0; i + 1 < course.holes.size(); ++i)
            {
                const auto& currentHole = course.holes[i];
                const auto& nextHole = course.holes[i + 1];

                if (!currentHole.cup.has_value() || !nextHole.firstTee().has_value())
                    continue;

                auto cupPos = *currentHole.cup;
                auto nextTeeCategory = *nextHole.firstTee();
                auto nextTeePos = *nextHole.tees[EnumValue(nextTeeCategory)];

                int32_t dist = distanceYards(cupPos, nextTeePos);
                if (dist > maxWalkWithoutPathYards)
                {
                    report.passed = false;
                    std::ostringstream ss;
                    ss << "Hole " << (i + 1) << " cup is too far from Hole " << (i + 2) << " tee ("
                       << dist << " yd, limit without path is " << maxWalkWithoutPathYards << " yd)";
                    report.issues.push_back({
                        ScenarioWarning::holePathDisconnected,
                        ss.str(),
                        course.id,
                        i + 1,
                    });
                }
            }
        }

        return report;
    }
} // namespace OpenRCT2::Golf
