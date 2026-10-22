/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../core/FixedVec.hpp"
#include "../drawing/Colour.h"
#include "BallPhysics.h"
#include "CourseManager.h"
#include "Hole.h"
#include "HoleSchedule.h"
#include "Shot.h"

#include <cstdint>
#include <deque>
#include <unordered_map>
#include <vector>

namespace OpenRCT2::Golf
{
    // UI-001: Overlays: hole playing line, ball trajectories (last N shots),
    // heat map of landing zones, lie map (colour per surface), wait-time map.

    enum class OverlayMode : uint8_t
    {
        none,
        playingLine,
        ballTrajectories,
        landingHeatMap,
        lieMap,
        waitTimeMap,
    };

    struct TrajectoryPoint
    {
        FixedVec3 position;
        uint32_t tick = 0;
    };

    struct ShotTrajectory
    {
        uint32_t shotId = 0;
        uint32_t golferId = 0;
        uint32_t holeNumber = 0;
        ClubId club = ClubId::driver;
        std::vector<FixedVec3> points;
        FixedVec3 landingPosition;
        bool landedInHazard = false;
    };

    enum class LieColourCategory : uint8_t
    {
        green,       // Putting surface
        fairway,     // Short grass
        rough,       // Heavy grass / woods
        sand,        // Bunker
        water,       // Hazard
        hardpan,     // Path / cart path
        outOfBounds, // Out of bounds
    };

    class GolfOverlayManager
    {
    public:
        static constexpr size_t kDefaultMaxTrajectories = 32;

        OverlayMode currentMode() const
        {
            return _mode;
        }
        void setMode(OverlayMode mode)
        {
            _mode = mode;
        }

        // 1. Hole playing line (UI-001)
        static std::vector<HolePoint> getPlayingLine(const Hole& hole, TeeCategory tee = TeeCategory::men);

        // 2. Ball trajectories (last N shots) (UI-001)
        void recordShotTrajectory(const ShotTrajectory& traj);
        const std::deque<ShotTrajectory>& getRecentTrajectories() const
        {
            return _trajectories;
        }
        void clearTrajectories()
        {
            _trajectories.clear();
        }
        size_t maxTrajectories() const
        {
            return _maxTrajectories;
        }
        void setMaxTrajectories(size_t maxCount);

        // 3. Heat map of landing zones (UI-001)
        void recordLanding(const TileCoordsXY& tile);
        uint32_t getLandingCount(const TileCoordsXY& tile) const;
        uint8_t getHeatIntensity(const TileCoordsXY& tile) const; // 0..255 normalized intensity
        void clearLandingHeatMap()
        {
            _landingCounts.clear();
            _maxLandingCount = 0;
        }

        // 4. Lie map: color classification per surface / lie (UI-001)
        static LieColourCategory classifyLie(const SurfaceSample& sample);
        static Drawing::Colour getLieColour(const SurfaceSample& sample);

        // 5. Wait-time map: congestion level per hole / tee (UI-001)
        enum class WaitLevel : uint8_t
        {
            clear,      // < 2 mins wait
            moderate,   // 2..5 mins wait
            busy,       // 5..8 mins wait
            bottleneck, // > 8 mins wait
        };
        static WaitLevel classifyWaitTime(int32_t waitMinutes);
        static Drawing::Colour getWaitLevelColour(WaitLevel level);

    private:
        OverlayMode _mode = OverlayMode::none;
        size_t _maxTrajectories = kDefaultMaxTrajectories;
        std::deque<ShotTrajectory> _trajectories;
        std::unordered_map<uint32_t, uint32_t> _landingCounts; // packed tile coord -> count
        uint32_t _maxLandingCount = 0;

        static uint32_t packTile(const TileCoordsXY& tile)
        {
            return (static_cast<uint32_t>(tile.x & 0xFFFF) << 16) | static_cast<uint32_t>(tile.y & 0xFFFF);
        }
    };
} // namespace OpenRCT2::Golf
