/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "CourseManager.h"
#include "StreetNetwork.h"
#include "Traffic.h"

#include <cstdint>
#include <string>
#include <vector>

namespace OpenRCT2
{
    struct GameState_t;
}

namespace OpenRCT2::Golf
{
    // SAV-003: Scenario editor validation warnings for golf parks
    enum class ScenarioWarning : uint32_t
    {
        noRoadEntrance = 1 << 0,
        noParking = 1 << 1,
        fewerThan18Holes = 1 << 2,
        holePathDisconnected = 1 << 3,
    };

    struct ScenarioValidationIssue
    {
        ScenarioWarning warning;
        std::string description;
        uint32_t courseId = 0;
        size_t holeNumber = 0;
    };

    struct ScenarioValidationReport
    {
        bool passed = true;
        std::vector<ScenarioValidationIssue> issues;

        bool hasWarning(ScenarioWarning w) const
        {
            for (const auto& issue : issues)
            {
                if (issue.warning == w)
                    return true;
            }
            return false;
        }
    };

    /**
     * SAV-003: Validates scenario conditions for a golf park.
     * Warns if:
     * - no road entrance exists on the street network
     * - no parking spaces exist for visitors
     * - no 18-hole golf course is present
     * - holes have no path / exceed walking distance to the next tee
     */
    ScenarioValidationReport validateScenario(
        const GameState_t& gameState,
        const StreetNetwork* streets = nullptr,
        const TrafficSimulation* traffic = nullptr,
        int32_t maxWalkWithoutPathYards = 120);
} // namespace OpenRCT2::Golf
