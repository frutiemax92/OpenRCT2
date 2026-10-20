/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "ShotDebug.h"

#include "../GameState.h"
#include "../entity/EntityRegistry.h"
#include "../entity/GolfBall.h"
#include "../scenario/Scenario.h"
#include "MapBallWorld.h"
#include "Rules.h"

namespace OpenRCT2::Golf
{
    namespace
    {
        // Gives the middle of the range every time, so the bell shaped error comes out as zero.
        class PerfectRandom final : public IBallRandom
        {
        public:
            uint32_t next() override
            {
                return 0x00800000u;
            }
        };

        class ScenarioRandom final : public IBallRandom
        {
        public:
            uint32_t next() override
            {
                return ScenarioRand();
            }
        };
    } // namespace

    FixedVec3 ShotDebug::aimVector(int32_t degrees)
    {
        const Fixed radians = Fixed::fromInt(degrees) * kFixedPi / 180;
        return { cos(radians), sin(radians), Fixed{} };
    }

    bool ShotDebug::setTee(const CoordsXY& position)
    {
        MapBallWorld world;
        const Fixed x = Fixed::fromInt(position.x), y = Fixed::fromInt(position.y);
        const auto surface = world.surfaceAt(x, y);
        if (surface.water || surface.outOfBounds)
            return false;
        _tee = FixedVec3{ x, y, surface.height };
        return true;
    }

    void ShotDebug::clear()
    {
        if (_active && _entityId != 0xFFFF)
        {
            if (auto* ball = getGameState().entities.getEntity<GolfBall>(EntityId::FromUnderlying(_entityId)))
            {
                ball->invalidate();
                getGameState().entities.entityRemove(ball);
            }
        }
        _active = false;
        _entityId = 0xFFFF;
        _samples.clear();
        _impacts.clear();
        _maxHeight = {};
        _ticks = 0;
        _lastState = BallState::Teed;
    }

    bool ShotDebug::fire()
    {
        if (!_tee.has_value())
            return false;
        clear();

        auto* ball = GolfBall::create(*_tee, BallState::Teed, 0);
        if (ball == nullptr)
            return false;

        ShotRequest request;
        request.club = settings.club;
        request.power = clamp(settings.power, Fixed{}, Fixed::fromInt(1));
        const auto aim = aimVector(settings.directionDegrees);
        request.aimX = aim.x;
        request.aimY = aim.y;
        request.skills = settings.skills;
        MapBallWorld world;
        request.lie = lieOf(world.surfaceAt(_tee->x, _tee->y));

        ShotResult shot;
        if (settings.perfect)
        {
            PerfectRandom rng;
            shot = computeShot(request, rng);
        }
        else
        {
            ScenarioRandom rng;
            shot = computeShot(request, rng);
        }
        _entityId = ball->id.ToUnderlying();
        _active = true;
        _lastPosition = ball->ball.position;
        _samples.push_back({ ball->ball.position, Fixed{}, BallState::Teed });
        ball->hit(shot);
        _lastState = ball->ball.state;
        // The abandoned-ball timer removes unowned balls; a test ball stays until the tool clears it.
        ball->ball.owner = 0xFFFFFFFFu;
        return true;
    }

    bool ShotDebug::windEnabledFor(const GolfBall& ball) const
    {
        const bool isTestBall = _active && ball.id.ToUnderlying() == _entityId;
        return !isTestBall || settings.wind;
    }

    void ShotDebug::observe(const GolfBall& ball, const BallEvents& events)
    {
        if (!_active || ball.id.ToUnderlying() != _entityId)
            return;
        _ticks++;
        _lastState = ball.ball.state;
        _lastPosition = ball.ball.position;
        _maxHeight = max(_maxHeight, ball.ball.position.z - _tee->z);
        if (_samples.size() < kMaxSamples)
            _samples.push_back({ ball.ball.position, length(ball.ball.velocity), ball.ball.state });
        if (events.impacts > 0)
            _impacts.push_back({ ball.ball.position, events.strongestImpact, _ticks });

        MapBallWorld world;
        const auto surface = world.surfaceAt(ball.ball.position.x, ball.ball.position.y);
        _lieSurface = surface.surface;
        _lieHazard = surface.hazard || surface.water;
        _liePuttable = surface.puttable;
        _lieOnPath = surface.path;
    }

    Fixed ShotDebug::carry() const
    {
        if (!_tee.has_value() || _impacts.empty())
            return Fixed{};
        const auto& p = _impacts.front().position;
        return length(FixedVec3{ p.x - _tee->x, p.y - _tee->y, Fixed{} });
    }

    Fixed ShotDebug::distance() const
    {
        if (!_tee.has_value() || !_active)
            return Fixed{};
        return length(FixedVec3{ _lastPosition.x - _tee->x, _lastPosition.y - _tee->y, Fixed{} });
    }

    ShotDebug& GetShotDebug()
    {
        static ShotDebug debug;
        return debug;
    }
} // namespace OpenRCT2::Golf
