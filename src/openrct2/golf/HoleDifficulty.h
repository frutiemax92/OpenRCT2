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
#include "Shot.h"

namespace OpenRCT2::Golf
{
    // Difficulty of a hole (HOL-040): 0 to 10 from the length against the par, the hazards beside the playing line, the
    // slope of the green, the obstacles on the line and the elevation change. The parts are kept so the course window
    // can explain the number.
    struct HoleDifficulty
    {
        // Each part is in tenths of a point; the total is capped at 100 (10.0).
        int32_t length = 0;     // up to 30: long for its par
        int32_t hazards = 0;    // up to 30: water, sand, out of bounds and deep rough beside the line
        int32_t greenSlope = 0; // up to 15: the green around the cup is steep
        int32_t obstacles = 0;  // up to 20: scenery and buildings on the line
        int32_t elevation = 0;  // up to 15: the line climbs and falls
        int32_t total = 0;      // 0 to 100

        double rating() const
        {
            return total / 10.0;
        }
    };

    constexpr int32_t kMaxLengthDifficulty = 30;
    constexpr int32_t kMaxHazardDifficulty = 30;
    constexpr int32_t kMaxGreenSlopeDifficulty = 15;
    constexpr int32_t kMaxObstacleDifficulty = 20;
    constexpr int32_t kMaxElevationDifficulty = 15;
    // The line is sampled every half tile, and the hazard corridor reaches this many tiles to each side.
    constexpr int32_t kDifficultySampleStep = 16;
    constexpr int32_t kHazardCorridorTiles = 3;
    // Slope of the ground around the cup that counts as a full point of green difficulty, as rise over run.
    constexpr int32_t kSteepGreenPercent = 10;

    // Rates a hole by sampling the world along its playing line from the men's tee (or the first tee that exists). A
    // hole that is not complete has no difficulty (all zero).
    HoleDifficulty rateHole(const Hole& hole, const IBallWorld& world);

    // HOL-041: the level of a golfer in tenths (0 to 100) from the average of the four skills.
    int32_t golferLevelTenths(const GolferSkills& skills);
    // How much harder than the golfer's level a hole may be before the golfer finds it too hard, in tenths.
    constexpr int32_t kTooHardThresholdTenths = 25;
    // True when the hole is too hard for the golfer (difficulty - level > threshold).
    bool isTooHard(const HoleDifficulty& difficulty, const GolferSkills& skills);
    // Happiness lost per hole played that is too hard, 0..255 scale (GLF-013).
    constexpr int32_t kTooHardHappinessLoss = 20;

    // Result of testing a hole with AI shots (HOL-012).
    struct TestHoleResult
    {
        int32_t par = 0;
        int32_t strokes = 0;
        int32_t scoreAgainstPar = 0;
        HoleDifficulty difficulty;
        int32_t estimatedDifficulty = 0; // 0..100: estimated difficulty rating
        std::vector<HolePoint> shotPositions;
        bool reachedCup = false;
    };

    // HOL-012: fires AI shots with average skill (50) from the first tee to the cup to test the hole and estimate
    // its difficulty.
    TestHoleResult testHole(const Hole& hole, const IBallWorld& world, IBallRandom* rng = nullptr);
} // namespace OpenRCT2::Golf
