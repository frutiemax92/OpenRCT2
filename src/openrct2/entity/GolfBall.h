/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../golf/BallPhysics.h"
#include "../golf/Shot.h"
#include "../world/Location.hpp"
#include "EntityBase.h"

#include <array>

namespace OpenRCT2
{
    class DataSerialiser;

    // The golf ball entity (BAL-001): the fixed-point simulation state of section 15 plus what is needed to draw it.
    struct GolfBall : EntityBase
    {
        static constexpr auto kEntityType = EntityType::golfBall;

        static constexpr int32_t kTrailLength = 6;
        // A resting ball is removed this many ticks after the last update that moved it, unless a golfer owns it.
        static constexpr uint32_t kAbandonedBallTicks = 40 * 60 * 5;

        Golf::Ball ball;
        uint32_t restingTicks = 0;
        // Recent positions for the trail drawn while the ball is in flight; cosmetic, not saved.
        std::array<CoordsXYZ, kTrailLength> trail;
        uint8_t trailCount = 0;
        // A ball that the golf simulation plays (a golfer's or one at the driving range) is only shown here: the simulation
        // moves it and update() leaves it alone. Not saved.
        bool mirrored = false;

        // Creates a ball at rest at a position (world units, z on the ground). Returns nullptr when the entity limit is hit.
        static GolfBall* create(const FixedVec3& position, Golf::BallState state = Golf::BallState::Teed, uint32_t owner = 0);
        // Hits the ball: sets its velocity and spin from a shot.
        void hit(const Golf::ShotResult& shot);

        // Takes the state of a ball the simulation plays, and follows it on the map.
        void mirror(const Golf::Ball& source);
        void update();
        void serialise(DataSerialiser& stream);

        bool isMoving() const
        {
            return !Golf::isSettled(ball.state) && ball.state != Golf::BallState::Teed;
        }
    };
} // namespace OpenRCT2
