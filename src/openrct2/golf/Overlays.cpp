/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Overlays.h"

#include <algorithm>

namespace OpenRCT2::Golf
{
    std::vector<HolePoint> GolfOverlayManager::getPlayingLine(const Hole& hole, TeeCategory tee)
    {
        return hole.playingLine(tee);
    }

    void GolfOverlayManager::setMaxTrajectories(size_t maxCount)
    {
        _maxTrajectories = std::max<size_t>(1, maxCount);
        while (_trajectories.size() > _maxTrajectories)
        {
            _trajectories.pop_front();
        }
    }

    void GolfOverlayManager::recordShotTrajectory(const ShotTrajectory& traj)
    {
        _trajectories.push_back(traj);
        while (_trajectories.size() > _maxTrajectories)
        {
            _trajectories.pop_front();
        }

        const TileCoordsXY landingTile{
            static_cast<int32_t>(traj.landingPosition.x.floor()) >> 5,
            static_cast<int32_t>(traj.landingPosition.y.floor()) >> 5,
        };
        recordLanding(landingTile);
    }

    void GolfOverlayManager::recordLanding(const TileCoordsXY& tile)
    {
        const uint32_t key = packTile(tile);
        const uint32_t newCount = ++_landingCounts[key];
        if (newCount > _maxLandingCount)
        {
            _maxLandingCount = newCount;
        }
    }

    uint32_t GolfOverlayManager::getLandingCount(const TileCoordsXY& tile) const
    {
        const auto it = _landingCounts.find(packTile(tile));
        return it != _landingCounts.end() ? it->second : 0;
    }

    uint8_t GolfOverlayManager::getHeatIntensity(const TileCoordsXY& tile) const
    {
        if (_maxLandingCount == 0)
        {
            return 0;
        }
        const uint32_t count = getLandingCount(tile);
        return static_cast<uint8_t>(std::min<uint32_t>(255, (count * 255) / _maxLandingCount));
    }

    LieColourCategory GolfOverlayManager::classifyLie(const SurfaceSample& sample)
    {
        if (sample.outOfBounds)
            return LieColourCategory::outOfBounds;
        if (sample.water)
            return LieColourCategory::water;
        if (sample.path)
            return LieColourCategory::hardpan;
        if (sample.hazard)
            return LieColourCategory::sand;
        if (sample.puttable)
            return LieColourCategory::green;

        const int32_t roughnessPct = static_cast<int32_t>((sample.roughness * 100).round());
        if (roughnessPct < 20)
            return LieColourCategory::fairway;

        return LieColourCategory::rough;
    }

    Drawing::Colour GolfOverlayManager::getLieColour(const SurfaceSample& sample)
    {
        switch (classifyLie(sample))
        {
            case LieColourCategory::green:
                return Drawing::Colour::brightGreen;
            case LieColourCategory::fairway:
                return Drawing::Colour::darkGreen;
            case LieColourCategory::rough:
                return Drawing::Colour::oliveGreen;
            case LieColourCategory::sand:
                return Drawing::Colour::yellow;
            case LieColourCategory::water:
                return Drawing::Colour::lightBlue;
            case LieColourCategory::hardpan:
                return Drawing::Colour::grey;
            case LieColourCategory::outOfBounds:
            default:
                return Drawing::Colour::brightRed;
        }
    }

    GolfOverlayManager::WaitLevel GolfOverlayManager::classifyWaitTime(int32_t waitMinutes)
    {
        if (waitMinutes <= 2)
            return WaitLevel::clear;
        if (waitMinutes <= 5)
            return WaitLevel::moderate;
        if (waitMinutes <= 8)
            return WaitLevel::busy;
        return WaitLevel::bottleneck;
    }

    Drawing::Colour GolfOverlayManager::getWaitLevelColour(WaitLevel level)
    {
        switch (level)
        {
            case WaitLevel::clear:
                return Drawing::Colour::brightGreen;
            case WaitLevel::moderate:
                return Drawing::Colour::brightYellow;
            case WaitLevel::busy:
                return Drawing::Colour::lightOrange;
            case WaitLevel::bottleneck:
            default:
                return Drawing::Colour::brightRed;
        }
    }
} // namespace OpenRCT2::Golf
