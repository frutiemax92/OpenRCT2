/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../world/Location.hpp"
#include "BallPhysics.h"
#include "Shot.h"

#include <optional>
#include <vector>

namespace OpenRCT2
{
    struct GolfBall;
}

namespace OpenRCT2::Golf
{
    // The debug shot tool (BAL-060): pick a tee on the map, choose a club, power and direction, fire a ball and read its
    // trajectory, bounces, lie, speed and state.

    struct ShotDebugSample
    {
        FixedVec3 position;
        Fixed speed{};
        BallState state = BallState::InFlight;
    };

    struct ShotDebugImpact
    {
        FixedVec3 position;
        Fixed normalSpeed{}; // how hard it hit
        uint32_t tick = 0;
    };

    struct ShotDebugSettings
    {
        ClubId club = ClubId::iron7;
        Fixed power = Fixed::fromInt(1);
        int32_t directionDegrees = 0; // 0 = +x, 90 = +y
        GolferSkills skills;
        bool perfect = true;          // no random error
        bool wind = true;
    };

    class ShotDebug
    {
    public:
        // Sets the tee on the ground at a world position. Returns false off the map or in water.
        bool setTee(const CoordsXY& position);
        const std::optional<FixedVec3>& tee() const
        {
            return _tee;
        }

        ShotDebugSettings settings;

        // Fires a ball from the tee. Returns false without a tee or when no entity can be created. An earlier test ball is
        // removed first.
        bool fire();
        void clear();

        // Called by the ball entity after each simulation tick of the tracked ball.
        void observe(const GolfBall& ball, const BallEvents& events);

        bool hasShot() const
        {
            return _active;
        }
        bool finished() const
        {
            return _active && isSettled(_lastState);
        }
        const std::vector<ShotDebugSample>& samples() const
        {
            return _samples;
        }
        const std::vector<ShotDebugImpact>& impacts() const
        {
            return _impacts;
        }
        BallState state() const
        {
            return _lastState;
        }
        Fixed currentSpeed() const
        {
            return _samples.empty() ? Fixed{} : _samples.back().speed;
        }
        // Horizontal distance from the tee to the first impact (the carry) and to where the ball is now, in world units.
        Fixed carry() const;
        Fixed distance() const;
        Fixed maxHeight() const
        {
            return _maxHeight;
        }
        uint32_t ticks() const
        {
            return _ticks;
        }
        // The name of the surface under the ball, and whether the next shot from there would be a putt.
        uint8_t lieSurface() const
        {
            return _lieSurface;
        }
        bool lieIsHazard() const
        {
            return _lieHazard;
        }
        bool liePuttable() const
        {
            return _liePuttable;
        }
        bool lieOnPath() const
        {
            return _lieOnPath;
        }

        // Aim direction as a unit vector for a number of degrees.
        static FixedVec3 aimVector(int32_t degrees);
        static constexpr size_t kMaxSamples = 4000;

    private:
        std::optional<FixedVec3> _tee;
        bool _active = false;
        uint16_t _entityId = 0xFFFF;
        std::vector<ShotDebugSample> _samples;
        std::vector<ShotDebugImpact> _impacts;
        BallState _lastState = BallState::Teed;
        FixedVec3 _lastPosition;
        Fixed _maxHeight{};
        uint32_t _ticks = 0;
        uint8_t _lieSurface = 0;
        bool _lieHazard = false;
        bool _liePuttable = false;
        bool _lieOnPath = false;
    };

    ShotDebug& GetShotDebug();
} // namespace OpenRCT2::Golf
