/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "GolfBall.h"

#include "../GameState.h"
#include "../core/DataSerialiser.h"
#include "../golf/MapBallWorld.h"
#include "../golf/MapObstacles.h"
#include "../golf/ShotDebug.h"
#include "../scenario/Scenario.h"
#include "EntityRegistry.h"

namespace OpenRCT2
{
    template<>
    bool EntityBase::is<GolfBall>() const
    {
        return type == EntityType::golfBall;
    }

    namespace
    {
        // All randomness goes through the scenario RNG so replays match (ARC-021).
        class ScenarioBallRandom final : public Golf::IBallRandom
        {
        public:
            uint32_t next() override
            {
                return ScenarioRand();
            }
        };

        CoordsXYZ toCoords(const FixedVec3& p)
        {
            return { static_cast<int32_t>(p.x.floor()), static_cast<int32_t>(p.y.floor()), static_cast<int32_t>(p.z.floor()) };
        }

        void serialiseFixed(DataSerialiser& stream, Fixed& value)
        {
            int64_t raw = value.raw();
            stream << raw;
            value = Fixed::fromRaw(raw);
        }

        void serialiseVec(DataSerialiser& stream, FixedVec3& v)
        {
            serialiseFixed(stream, v.x);
            serialiseFixed(stream, v.y);
            serialiseFixed(stream, v.z);
        }
    } // namespace

    GolfBall* GolfBall::create(const FixedVec3& position, Golf::BallState state, uint32_t owner)
    {
        auto* entity = getGameState().entities.createEntity<GolfBall>();
        if (entity == nullptr)
            return nullptr;

        // BAL-002: a ball is 2 x 2 pixels, so its sprite box is tiny.
        entity->spriteData.width = 2;
        entity->spriteData.heightMin = 2;
        entity->spriteData.heightMax = 2;
        entity->ball = {};
        entity->ball.position = position;
        entity->ball.lastSafePosition = position;
        entity->ball.shotOrigin = position;
        entity->ball.state = state;
        entity->ball.owner = owner;
        entity->restingTicks = 0;
        entity->trailCount = 0;
        entity->moveToAndUpdateSpatialIndex(toCoords(position));
        return entity;
    }

    void GolfBall::hit(const Golf::ShotResult& shot)
    {
        Golf::applyShot(ball, shot);
        restingTicks = 0;
        trailCount = 0;
    }

    void GolfBall::update()
    {
        if (Golf::isSettled(ball.state) || ball.state == Golf::BallState::Teed)
        {
            restingTicks++;
            if (ball.owner == 0 && restingTicks > kAbandonedBallTicks)
            {
                invalidate();
                getGameState().entities.entityRemove(this);
            }
            return;
        }

        Golf::MapBallWorld world;
        world.setObstacles(&Golf::GetMapObstacles());
        if (Golf::GetShotDebug().windEnabledFor(*this))
            world.useGameWind();
        world.useCourses();
        ScenarioBallRandom rng;
        const auto before = getLocation();
        const auto events = Golf::stepBall(ball, world, rng);
        restingTicks = 0;
        Golf::GetShotDebug().observe(*this, events);

        const auto location = toCoords(ball.position);
        const bool flying = ball.state == Golf::BallState::InFlight || ball.state == Golf::BallState::Bouncing;
        if (!flying)
            trailCount = 0;
        if (location != before)
        {
            if (flying)
            {
                if (trailCount < kTrailLength)
                    trailCount++;
                for (int32_t i = trailCount - 1; i > 0; i--)
                    trail[i] = trail[i - 1];
                trail[0] = before;
            }
            invalidate();
            moveToAndUpdateSpatialIndex(location);
        }
    }

    void GolfBall::serialise(DataSerialiser& stream)
    {
        EntityBase::serialise(stream);
        serialiseVec(stream, ball.position);
        serialiseVec(stream, ball.velocity);
        serialiseFixed(stream, ball.sideSpin);
        serialiseFixed(stream, ball.backSpin);
        stream << ball.state;
        stream << ball.owner;
        serialiseVec(stream, ball.lastSafePosition);
        serialiseVec(stream, ball.hazardEntry);
        serialiseVec(stream, ball.shotOrigin);
        stream << ball.passingObstacle;
        stream << ball.lippedOut;
        stream << ball.bounces;
        stream << ball.lieSurface;
        stream << restingTicks;
    }
} // namespace OpenRCT2
