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

#include <cstdint>

namespace OpenRCT2::Golf
{
    // Golf ball physics (BAL-030 to BAL-036). Everything here is deterministic fixed-point (ARC-020). Units: world
    // units (1/32 tile, z has the same scale), time in game ticks (40 per second).

    enum class BallState : uint8_t
    {
        Teed,
        InFlight,
        Bouncing,
        Rolling,
        AtRest,
        Holed,
        Lost,
        OutOfBounds,
    };

    inline bool isSettled(BallState s)
    {
        return s == BallState::AtRest || s == BallState::Holed || s == BallState::Lost || s == BallState::OutOfBounds;
    }

    // The ground under the ball (BAL-032).
    struct SurfaceSample
    {
        Fixed height{};                    // surface z at the queried position
        FixedVec3 normal{ Fixed{}, Fixed{}, Fixed::fromInt(1) }; // unit length, pointing up
        Fixed roughness = Fixed::ratio(15, 100);
        Fixed bounciness = Fixed::ratio(45, 100);
        uint8_t surface = 0;               // surface object index, for the lie (BAL-040)
        bool water = false;                // water or stream: the ball is lost (BAL-050)
        bool outOfBounds = false;          // beyond the course boundary or off the map (BAL-051)
        bool path = false;                 // path or street ("hardpan")
        bool hazard = false;               // sand-like hazard (TER-041 isHazard)
        bool puttable = false;             // a green: the putter is used here
        Fixed distanceFactor = Fixed::fromInt(1); // lie modifiers for the next shot (BAL-040)
        Fixed accuracyFactor = Fixed::fromInt(1);
    };

    // An obstacle the ball touches (BAL-034). `normal` points out of the obstacle, `depth` is how far the ball is inside.
    struct ObstacleContact
    {
        FixedVec3 normal{ Fixed{}, Fixed{}, Fixed::fromInt(1) };
        Fixed depth{};
        Fixed restitution = Fixed::ratio(4, 10);
        Fixed friction = Fixed::ratio(3, 10);
        Fixed passThroughChance{}; // 0..1
        uint32_t id = 0;           // stable identity, so a ball passing through is not re-rolled every sub-step
        bool isGolfer = false;
        uint32_t golferId = 0;
    };

    // The cup of the hole the ball is on, when there is one within reach (BAL-036).
    struct CupInfo
    {
        FixedVec3 position{};
        Fixed radius = Fixed::fromInt(3);
    };

    class IBallWorld
    {
    public:
        virtual ~IBallWorld() = default;
        virtual SurfaceSample surfaceAt(Fixed x, Fixed y) const = 0;
        // True and fills `contact` when a ball of `radius` at `position` overlaps an obstacle.
        virtual bool obstacleAt(const FixedVec3& position, Fixed radius, ObstacleContact& contact) const
        {
            (void)position;
            (void)radius;
            (void)contact;
            return false;
        }
        virtual bool cupNear(Fixed x, Fixed y, CupInfo& cup) const
        {
            (void)x;
            (void)y;
            (void)cup;
            return false;
        }
        // Wind as an acceleration-free air velocity in units per tick (BAL-031); zero when there is none.
        virtual FixedVec3 wind() const
        {
            return {};
        }
    };

    class IBallRandom
    {
    public:
        virtual ~IBallRandom() = default;
        virtual uint32_t next() = 0;
    };

    struct Ball
    {
        FixedVec3 position{};
        FixedVec3 velocity{};
        Fixed sideSpin{};  // positive curves to the right of the direction of travel, -1..1
        Fixed backSpin{};  // positive lifts in flight and bites on landing, -1..1 (negative is top spin)
        BallState state = BallState::Teed;
        uint32_t owner = 0; // golfer entity id
        // Bookkeeping for the rules (BAL-050, BAL-051).
        FixedVec3 lastSafePosition{};  // last point over ground that is neither water nor out of bounds
        FixedVec3 hazardEntry{};       // where the ball crossed into the hazard it was lost in
        FixedVec3 shotOrigin{};        // where the current shot was hit from
        uint32_t passingObstacle = 0;  // obstacle currently being passed through, 0 for none
        bool lippedOut = false;        // the ball lipped out of the cup and has not left its rim yet
        uint8_t bounces = 0;
        uint8_t lieSurface = 0;
    };

    // What happened during a tick, for the rules and for sound / thought effects.
    struct BallEvents
    {
        uint8_t impacts = 0;       // terrain impacts
        uint8_t obstacleHits = 0;
        bool enteredWater = false;
        bool wentOutOfBounds = false;
        bool holed = false;
        bool lippedOut = false;
        bool hitGolfer = false;
        uint32_t golferId = 0;
        Fixed strongestImpact{};   // normal speed of the hardest impact, for the sound volume
    };

    // Tunable constants, documented in one place (BAL-030..BAL-036). Defaults are tuned so that the carry of every club
    // matches its base carry distance (see BallCalibrationTests).
    struct BallParams
    {
        int32_t subSteps = 4;
        Fixed gravity = Fixed::ratio(4291, 100000);     // 9.81 m/s^2 in units per tick^2
        Fixed drag = Fixed::ratio(5, 10000);            // fraction of the air-relative speed lost per tick
        Fixed magnusSide = Fixed::ratio(1, 1000);       // sideways acceleration per unit of side spin and speed
        Fixed magnusLift = Fixed::ratio(8, 10000);      // lift per unit of back spin and speed
        Fixed spinDecay = Fixed::ratio(3, 1000);        // fraction of the spin lost per tick
        Fixed impactLoss = Fixed::ratio(6, 10);         // tangential speed lost at an impact, times roughness
        Fixed backSpinBite = Fixed::ratio(35, 100);     // back spin slowing the ball on landing, times normal speed
        Fixed restVelocity = Fixed::ratio(5, 100);      // normal speed below which an impact turns into rolling
        Fixed rollResistance = Fixed::ratio(1, 20);     // rolling deceleration per unit of roughness, units / tick^2
        Fixed stopSpeed = Fixed::ratio(1, 100);         // speed below which a rolling ball may stop
        Fixed leaveGroundDrop = Fixed::fromInt(2);      // a rolling ball that finds ground this much lower flies on
        Fixed ballRadius = Fixed::ratio(1, 2);
        Fixed captureSpeed = Fixed::ratio(45, 100);     // fastest speed at which the cup holds the ball
        Fixed lipOutSpeedLoss = Fixed::ratio(8, 10);    // speed kept when the ball lips out
        Fixed lipOutAngle = Fixed::ratio(6, 10);        // radians the ball is deflected when it lips out
        Fixed cupHeight = Fixed::fromInt(2);            // the ball must be this close to the ground to drop in
    };

    // Advances a ball by one game tick (BAL-030). Returns what happened.
    BallEvents stepBall(Ball& ball, const IBallWorld& world, IBallRandom& rng, const BallParams& params = {});

    // Places a ball on the tee or ground at a position (z from the surface), at rest.
    void placeBall(Ball& ball, const IBallWorld& world, Fixed x, Fixed y, BallState state = BallState::Teed);
} // namespace OpenRCT2::Golf
