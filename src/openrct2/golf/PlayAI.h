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
#include "Hole.h"
#include "Rules.h"
#include "Shot.h"

#include <vector>

namespace OpenRCT2::Golf
{
    // The golfer's decision for a shot (HOL-030, HOL-031): which club, how hard, and where to aim.
    struct ShotDecision
    {
        ClubId club = ClubId::driver;
        Fixed power = Fixed::fromInt(1); // 0..1
        FixedVec3 target;                // where the golfer is aiming the ball to land (or to stop, for a putt)
        Fixed aimX{};                    // horizontal unit direction of the aim
        Fixed aimY{};
        bool putt = false;
        Fixed expectedDistance{};        // the distance the shot is meant to cover, in world units
    };

    struct PlayContext
    {
        const IBallWorld* world = nullptr;
        GolferSkills skills;
        uint8_t energy = 255;
        int32_t equipmentLevel = 0;
        // 0 (careful) .. 1 (reckless): how much a golfer is willing to risk a hazard to get closer (HOL-030).
        Fixed riskTolerance = Fixed::ratio(5, 10);
        FixedVec3 ball;                  // where the ball lies
        std::vector<HolePoint> line;     // the playing line from the tee: tee, aim points, cup
        FixedVec3 cup;                   // z from the ground
        bool teeShot = false;
        bool miniPutt = false;           // only the mini-putt club is allowed (MPT-001)
    };

    // Margin past the cup that a putt is aimed to run if it misses, as a fraction of the distance with a minimum, so it
    // does not die short (HOL-031).
    constexpr int32_t kPuttMarginPercent = 12;
    constexpr int32_t kPuttMarginMinUnits = 5;
    // Longest putt a golfer attempts from a green before taking a lagging approach (the putter's longest roll anyway).
    // Weight in world units of a landing in a hazard against the progress towards the cup, scaled by (1 - risk).
    constexpr int32_t kHazardPenaltyUnits = 600;
    // Smallest swing a club is taken for, in percent of its power.
    constexpr int32_t kMinSwingPercent = 30;
    // Cost in world units of a swing at the extreme ends of the power range.
    constexpr int32_t kSwingComfortUnits = 40;
    // Weight of the quality of the lie where the ball is expected to land, in world units per 1.0 of lie quality.
    constexpr int32_t kLieQualityUnits = 160;

    // Chooses the next shot. Deterministic: the same lie and skills always give the same decision.
    ShotDecision decideShot(const PlayContext& context);

    // The breaking of a putt: the sideways slope across the line from the ball to the cup, as rise over run, positive when
    // the ground falls to the left of the direction of play.
    Fixed puttBreak(const IBallWorld& world, const FixedVec3& ball, const FixedVec3& cup);
} // namespace OpenRCT2::Golf
