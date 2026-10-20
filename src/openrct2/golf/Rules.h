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
#include "Shot.h"

namespace OpenRCT2::Golf
{
    // Lie, penalties and rules of simplified stroke play (BAL-040, BAL-050 to BAL-053).

    // BAL-040: the lie of a ball resting on a surface. Paths and streets are "hardpan": hard ground that bounces the
    // ball and is awkward to hit from.
    constexpr int32_t kHardpanDistancePercent = 90;
    constexpr int32_t kHardpanAccuracyPercent = 80;

    Lie lieOf(const SurfaceSample& surface);
    // The club a golfer takes from this lie when it is up to the golfer: the putter on a green (TER-041).
    bool mustPutt(const SurfaceSample& surface);

    enum class Penalty : uint8_t
    {
        none,
        water,         // BAL-050
        outOfBounds,   // BAL-051
        lostBall,      // BAL-052
        maxStrokes,    // BAL-053
    };

    constexpr int32_t kHazardMarginDropBack = 2; // units the drop point is moved away from the hazard edge
    constexpr int32_t kSearchTicks = 40 * 60;     // a golfer searches for a ball for at most one game minute
    constexpr int32_t kLostBallRoughness = 70;    // percent: rougher ground can hide the ball
    constexpr int32_t kMaxLostBallChancePercent = 60;
    constexpr int32_t kMaxStrokesOverPar = 4;

    struct ShotOutcome
    {
        enum class Kind : uint8_t
        {
            inPlay,   // the ball is at rest on playable ground
            holed,
            penalty,  // the ball goes to dropPosition after penaltyStrokes extra strokes
        };
        Kind kind = Kind::inPlay;
        Penalty penalty = Penalty::none;
        int32_t penaltyStrokes = 0;
        FixedVec3 dropPosition{};     // where the next shot is played from after a penalty
        bool replayFromPrevious = false; // the drop is the position of the previous shot
        bool needsSearch = false;     // the ball rests where it may be lost (BAL-052)
    };

    // Judges a ball that has stopped (water, out of bounds, holed, at rest) and says where to play on from.
    ShotOutcome judgeShot(const Ball& ball, const IBallWorld& world);

    // Chance, in percent, that the ball is not found in this ground (BAL-052). Zero outside deep rough and woods.
    int32_t lostBallChancePercent(const SurfaceSample& surface);
    // True when the golfer finds the ball after searching.
    bool searchFindsBall(const SurfaceSample& surface, IBallRandom& rng);

    // The penalty for a ball that was not found: one stroke and a drop at the previous position.
    ShotOutcome lostBallOutcome(const Ball& ball);

    // BAL-053: strokes per hole are capped at par + 4, after which the golfer picks up.
    constexpr int32_t maxStrokes(int32_t par)
    {
        return par + kMaxStrokesOverPar;
    }

    // Strokes of one golfer on one hole.
    struct HoleScore
    {
        int32_t par = 4;
        int32_t shots = 0;
        int32_t penalties = 0;
        bool holed = false;
        bool pickedUp = false;

        int32_t strokes() const
        {
            return shots + penalties;
        }
        // A shot was played.
        void addShot();
        // Applies the outcome of a shot; returns true when the golfer must pick up (max strokes reached).
        bool apply(const ShotOutcome& outcome);
        bool finished() const
        {
            return holed || pickedUp;
        }
        // Strokes against par for the hole; a golfer who picked up scores the maximum.
        int32_t toPar() const;
    };

    enum class ScoreName : uint8_t
    {
        holeInOne,
        albatross,
        eagle,
        birdie,
        par,
        bogey,
        doubleBogey,
        worse,
    };
    ScoreName scoreName(int32_t strokes, int32_t par);
} // namespace OpenRCT2::Golf
