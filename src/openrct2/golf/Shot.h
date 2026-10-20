/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "BallPhysics.h"
#include "Clubs.h"

namespace OpenRCT2::Golf
{
    // The shot model (BAL-020, BAL-021): turns a golfer's choice and skills into the launch of a ball.

    // Golfer skills, each 0..100 (GLF-020).
    struct GolferSkills
    {
        uint8_t longPlay = 50;   // maximum shot distance factor
        uint8_t accuracy = 50;   // spread of direction and distance error
        uint8_t foolproof = 50;  // resistance of the other skills to fatigue
        uint8_t physique = 50;   // need drain rate; unused by the shot itself
    };

    // The lie modifiers of the surface the ball rests on (BAL-040).
    struct Lie
    {
        Fixed distanceFactor = Fixed::fromInt(1);
        Fixed accuracyFactor = Fixed::fromInt(1);
    };

    struct ShotRequest
    {
        ClubId club = ClubId::driver;
        Fixed power = Fixed::fromInt(1);     // 0..1, share of the club's distance
        Fixed aimX = Fixed::fromInt(1);      // horizontal direction of the target, any length
        Fixed aimY{};
        uint8_t energy = 255;                // 0..255 (GLF-010)
        GolferSkills skills;
        Lie lie;
        int32_t equipmentLevel = 0;          // 0..3 (GLF-050)
        Fixed surfaceRoughness = Fixed::ratio(5, 100); // of the ground under a putt
        Fixed windSpeed{};                   // unused by the shot; the simulation applies the wind
    };

    struct ShotResult
    {
        FixedVec3 velocity;
        Fixed sideSpin{};
        Fixed backSpin{};
        Fixed intendedDistance{};  // units, before the random error
        Fixed requestedDistance{}; // units, including the distance error
        Fixed directionError{};    // radians, positive turns to the left
        bool isPutt = false;       // starts rolling on the ground instead of flying
    };

    // BAL-021: f(Long play), 0.6 at skill 0 up to 1.3 at skill 100, linear.
    Fixed longPlayFactor(int32_t longPlay);
    // Fatigue (0..1): 0 when energy is full, grows as energy drops; Foolproof shrinks it by up to 100%.
    Fixed fatigue(uint8_t energy, int32_t foolproof);
    // The base carry of a club in world units for a golfer: base * f(Long play) * lie * (1 - 0.2 fatigue).
    Fixed shotDistance(const ShotRequest& request);
    // Standard deviation of the direction error in radians (BAL-020).
    Fixed directionSpread(const ShotRequest& request);
    // Standard deviation of the distance error as a fraction of the distance.
    Fixed distanceSpread(const ShotRequest& request);

    ShotResult computeShot(const ShotRequest& request, IBallRandom& rng, const BallParams& params = {});

    // Hits `ball` with the result: sets velocity, spin and state, and remembers where the shot started.
    void applyShot(Ball& ball, const ShotResult& shot);
} // namespace OpenRCT2::Golf
