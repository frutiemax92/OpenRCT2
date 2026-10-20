/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "BallPhysics.h"

namespace OpenRCT2::Golf
{
    namespace
    {
        const Fixed kOne = Fixed::fromInt(1);

        struct Context
        {
            Ball& ball;
            const IBallWorld& world;
            IBallRandom& rng;
            const BallParams& params;
            BallEvents& events;
            Fixed dt;
        };

        void markSafe(Ball& ball, const SurfaceSample& s, Fixed x, Fixed y)
        {
            if (!s.water && !s.outOfBounds)
                ball.lastSafePosition = { x, y, s.height };
        }

        void loseInHazard(Context& c, const SurfaceSample& s)
        {
            auto& ball = c.ball;
            ball.velocity = {};
            ball.hazardEntry = ball.lastSafePosition;
            if (s.water)
            {
                ball.state = BallState::Lost;
                c.events.enteredWater = true;
            }
            else
            {
                ball.state = BallState::OutOfBounds;
                c.events.wentOutOfBounds = true;
            }
        }

        // Rotates a horizontal vector by an angle (radians).
        FixedVec3 rotateXY(const FixedVec3& v, Fixed angle)
        {
            const Fixed cs = cos(angle);
            const Fixed sn = sin(angle);
            return { v.x * cs - v.y * sn, v.x * sn + v.y * cs, v.z };
        }

        void handleCup(Context& c)
        {
            auto& ball = c.ball;
            CupInfo cup;
            if (!c.world.cupNear(ball.position.x, ball.position.y, cup))
            {
                ball.lippedOut = false;
                return;
            }
            const FixedVec3 offset{ ball.position.x - cup.position.x, ball.position.y - cup.position.y, Fixed{} };
            const Fixed distance = length(offset);
            if (distance >= cup.radius)
            {
                ball.lippedOut = false;
                return;
            }
            if (ball.lippedOut || ball.position.z - cup.position.z > c.params.cupHeight)
                return;
            const Fixed speed = length(ball.velocity);
            if (speed <= c.params.captureSpeed)
            {
                ball.position = cup.position;
                ball.velocity = {};
                ball.state = BallState::Holed;
                c.events.holed = true;
                return;
            }
            // Too fast: the ball lips out, deflected to the side it passed on.
            const Fixed side = cross(ball.velocity, offset).z;
            const Fixed angle = side >= Fixed{} ? -c.params.lipOutAngle : c.params.lipOutAngle;
            ball.velocity = rotateXY(ball.velocity, angle) * c.params.lipOutSpeedLoss;
            ball.lippedOut = true;
            c.events.lippedOut = true;
        }

        // Returns true when the ball stopped moving on this step because of an obstacle that ends the shot.
        void handleObstacles(Context& c)
        {
            auto& ball = c.ball;
            ObstacleContact contact;
            if (!c.world.obstacleAt(ball.position, c.params.ballRadius, contact))
            {
                ball.passingObstacle = 0;
                return;
            }
            if (contact.id != 0 && contact.id == ball.passingObstacle)
                return;
            if (contact.passThroughChance > Fixed{})
            {
                const auto roll = static_cast<int64_t>(c.rng.next() % 1000);
                if (Fixed::fromInt(roll) < contact.passThroughChance * 1000)
                {
                    ball.passingObstacle = contact.id;
                    return;
                }
            }
            ball.passingObstacle = 0;
            if (contact.isGolfer)
            {
                // The ball stops dead and drops; the golfer is hit (BAL-034).
                c.events.hitGolfer = true;
                c.events.golferId = contact.golferId;
                ball.velocity.x = Fixed{};
                ball.velocity.y = Fixed{};
                ball.velocity.z = min(ball.velocity.z, Fixed{});
                ball.position += contact.normal * contact.depth;
                if (ball.state != BallState::Rolling)
                    ball.state = BallState::Bouncing;
                return;
            }
            ball.position += contact.normal * contact.depth;
            const Fixed vn = dot(ball.velocity, contact.normal);
            if (vn < Fixed{})
            {
                const FixedVec3 normalPart = contact.normal * vn;
                const FixedVec3 tangent = ball.velocity - normalPart;
                ball.velocity = tangent * (kOne - contact.friction) - normalPart * contact.restitution;
                c.events.obstacleHits++;
                if (ball.state == BallState::InFlight)
                    ball.state = BallState::Bouncing;
            }
        }

        void stepFlight(Context& c)
        {
            auto& ball = c.ball;
            const auto& p = c.params;
            const FixedVec3 relative = ball.velocity - c.world.wind();
            const Fixed speed = length(relative);

            FixedVec3 acceleration{ Fixed{}, Fixed{}, -p.gravity };
            const Fixed horizontal = lengthXY(ball.velocity);
            if (horizontal > Fixed::ratio(1, 1000))
            {
                const Fixed hx = ball.velocity.x / horizontal;
                const Fixed hy = ball.velocity.y / horizontal;
                const Fixed side = ball.sideSpin * p.magnusSide * speed;
                acceleration.x += hy * side; // to the right of the direction of travel
                acceleration.y -= hx * side;
            }
            acceleration.z += ball.backSpin * p.magnusLift * speed;

            ball.velocity -= relative * (p.drag * c.dt);
            ball.velocity += acceleration * c.dt;
            const Fixed decay = kOne - p.spinDecay * c.dt;
            ball.sideSpin *= decay;
            ball.backSpin *= decay;
            ball.position += ball.velocity * c.dt;

            handleObstacles(c);

            const SurfaceSample s = c.world.surfaceAt(ball.position.x, ball.position.y);
            if (ball.position.z > s.height)
            {
                markSafe(ball, s, ball.position.x, ball.position.y);
                return;
            }

            // Impact with the ground.
            ball.position.z = s.height;
            if (s.water || s.outOfBounds)
            {
                loseInHazard(c, s);
                return;
            }
            markSafe(ball, s, ball.position.x, ball.position.y);
            ball.lieSurface = s.surface;
            const Fixed vn = dot(ball.velocity, s.normal);
            if (vn >= Fixed{})
                return; // already moving away from a surface that tilts towards the ball
            const Fixed impactSpeed = -vn;
            c.events.impacts++;
            c.events.strongestImpact = max(c.events.strongestImpact, impactSpeed);

            const FixedVec3 normalPart = s.normal * vn;
            FixedVec3 tangent = ball.velocity - normalPart;
            if (impactSpeed < p.restVelocity)
            {
                ball.velocity = tangent;
                ball.state = BallState::Rolling;
                return;
            }
            tangent = tangent * max(Fixed{}, kOne - s.roughness * p.impactLoss);
            const Fixed tangentSpeed = length(tangent);
            if (tangentSpeed > Fixed::ratio(1, 1000))
            {
                // Back spin makes the ball bite and slow down, even roll back; top spin speeds it up.
                const Fixed bite = ball.backSpin * p.backSpinBite * impactSpeed;
                tangent = tangent - tangent * (bite / tangentSpeed);
            }
            ball.velocity = tangent - normalPart * s.bounciness;
            ball.sideSpin = ball.sideSpin / 2;
            ball.backSpin = ball.backSpin / 2;
            ball.state = BallState::Bouncing;
            if (ball.bounces < 255)
                ball.bounces++;
        }

        void stepRolling(Context& c)
        {
            auto& ball = c.ball;
            const auto& p = c.params;
            const SurfaceSample here = c.world.surfaceAt(ball.position.x, ball.position.y);
            ball.lieSurface = here.surface;

            // Gravity along the slope: g - n (g . n), with g = (0, 0, -gravity).
            const FixedVec3 slope = here.normal * (p.gravity * here.normal.z) - FixedVec3{ Fixed{}, Fixed{}, p.gravity };
            ball.velocity += slope * c.dt;

            const Fixed deceleration = p.rollResistance * here.roughness;
            const Fixed speed = length(ball.velocity);
            const Fixed removed = deceleration * c.dt;
            if (speed <= removed)
                ball.velocity = {};
            else
                ball.velocity = ball.velocity * ((speed - removed) / speed);

            if (length(ball.velocity) < p.stopSpeed && length(slope) <= deceleration)
            {
                ball.velocity = {};
                ball.state = BallState::AtRest;
                markSafe(ball, here, ball.position.x, ball.position.y);
                return;
            }

            const Fixed oldX = ball.position.x;
            const Fixed oldY = ball.position.y;
            const Fixed oldZ = ball.position.z;
            ball.position.x += ball.velocity.x * c.dt;
            ball.position.y += ball.velocity.y * c.dt;

            const SurfaceSample next = c.world.surfaceAt(ball.position.x, ball.position.y);
            if (next.water || next.outOfBounds)
            {
                ball.position.z = next.height;
                loseInHazard(c, next);
                return;
            }
            if (next.height - oldZ > p.leaveGroundDrop * 2)
            {
                // A wall or cliff face: the ball bounces back off it.
                ball.position.x = oldX;
                ball.position.y = oldY;
                ball.velocity.x = -ball.velocity.x * here.bounciness;
                ball.velocity.y = -ball.velocity.y * here.bounciness;
                c.events.impacts++;
                return;
            }
            if (oldZ - next.height > p.leaveGroundDrop)
            {
                // Rolled off an edge: the ball flies on.
                ball.position.z = oldZ;
                ball.state = BallState::InFlight;
                return;
            }
            ball.position.z = next.height;
            ball.velocity = ball.velocity - next.normal * dot(ball.velocity, next.normal);
            markSafe(ball, next, ball.position.x, ball.position.y);

            handleObstacles(c);
        }
    } // namespace

    BallEvents stepBall(Ball& ball, const IBallWorld& world, IBallRandom& rng, const BallParams& params)
    {
        BallEvents events;
        if (ball.state == BallState::Teed || isSettled(ball.state))
            return events;

        Context c{ ball, world, rng, params, events, Fixed::ratio(1, params.subSteps > 0 ? params.subSteps : 1) };
        for (int32_t i = 0; i < params.subSteps && !isSettled(ball.state); i++)
        {
            if (ball.state == BallState::Rolling)
                stepRolling(c);
            else
                stepFlight(c);
            if (!isSettled(ball.state))
                handleCup(c);
        }
        return events;
    }

    void placeBall(Ball& ball, const IBallWorld& world, Fixed x, Fixed y, BallState state)
    {
        const SurfaceSample s = world.surfaceAt(x, y);
        ball.position = { x, y, s.height };
        ball.velocity = {};
        ball.sideSpin = {};
        ball.backSpin = {};
        ball.state = state;
        ball.lastSafePosition = ball.position;
        ball.shotOrigin = ball.position;
        ball.passingObstacle = 0;
        ball.lippedOut = false;
        ball.bounces = 0;
        ball.lieSurface = s.surface;
    }
} // namespace OpenRCT2::Golf
