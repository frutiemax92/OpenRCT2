/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Rules.h"

#include <algorithm>

namespace OpenRCT2::Golf
{
    Lie lieOf(const SurfaceSample& surface)
    {
        Lie lie;
        if (surface.path)
        {
            lie.distanceFactor = Fixed::ratio(kHardpanDistancePercent, 100);
            lie.accuracyFactor = Fixed::ratio(kHardpanAccuracyPercent, 100);
            return lie;
        }
        lie.distanceFactor = surface.distanceFactor;
        lie.accuracyFactor = surface.accuracyFactor;
        return lie;
    }

    bool mustPutt(const SurfaceSample& surface)
    {
        return surface.puttable;
    }

    namespace
    {
        bool isPlayable(const SurfaceSample& s)
        {
            return !s.water && !s.outOfBounds;
        }

        ShotOutcome penaltyOutcome(Penalty penalty, const FixedVec3& drop, bool replay)
        {
            ShotOutcome o;
            o.kind = ShotOutcome::Kind::penalty;
            o.penalty = penalty;
            o.penaltyStrokes = 1;
            o.dropPosition = drop;
            o.replayFromPrevious = replay;
            return o;
        }
    } // namespace

    ShotOutcome judgeShot(const Ball& ball, const IBallWorld& world)
    {
        switch (ball.state)
        {
            case BallState::Holed:
            {
                ShotOutcome o;
                o.kind = ShotOutcome::Kind::holed;
                return o;
            }
            case BallState::Lost:
            {
                // BAL-050: drop where the ball crossed into the hazard, or at the previous shot if that spot is no good.
                FixedVec3 drop = ball.hazardEntry;
                const SurfaceSample at = world.surfaceAt(drop.x, drop.y);
                bool replay = false;
                if (!isPlayable(at))
                {
                    drop = ball.shotOrigin;
                    replay = true;
                }
                else
                {
                    // Step back from the water's edge, towards where the shot was played from.
                    const FixedVec3 back = normalised(FixedVec3{ ball.shotOrigin.x - drop.x, ball.shotOrigin.y - drop.y, Fixed{} });
                    const Fixed nx = drop.x + back.x * kHazardMarginDropBack;
                    const Fixed ny = drop.y + back.y * kHazardMarginDropBack;
                    const SurfaceSample moved = world.surfaceAt(nx, ny);
                    if (isPlayable(moved))
                        drop = { nx, ny, moved.height };
                }
                return penaltyOutcome(Penalty::water, drop, replay);
            }
            case BallState::OutOfBounds:
                // BAL-051: stroke and distance, replay from where the shot was played.
                return penaltyOutcome(Penalty::outOfBounds, ball.shotOrigin, true);
            case BallState::AtRest:
            {
                ShotOutcome o;
                const SurfaceSample here = world.surfaceAt(ball.position.x, ball.position.y);
                if (!isPlayable(here))
                    return penaltyOutcome(here.water ? Penalty::water : Penalty::outOfBounds, ball.shotOrigin, true);
                o.needsSearch = lostBallChancePercent(here) > 0;
                return o;
            }
            default:
                return {};
        }
    }

    int32_t lostBallChancePercent(const SurfaceSample& surface)
    {
        if (surface.hazard || surface.water || surface.path)
            return 0;
        const int32_t roughness = static_cast<int32_t>((surface.roughness * 100).round());
        if (roughness < kLostBallRoughness)
            return 0;
        // 0 at the threshold, growing with roughness up to the cap.
        return std::min(kMaxLostBallChancePercent, (roughness - kLostBallRoughness) * 4 + 10);
    }

    bool searchFindsBall(const SurfaceSample& surface, IBallRandom& rng)
    {
        const int32_t chance = lostBallChancePercent(surface);
        if (chance == 0)
            return true;
        return static_cast<int32_t>(rng.next() % 100) >= chance;
    }

    ShotOutcome lostBallOutcome(const Ball& ball)
    {
        return penaltyOutcome(Penalty::lostBall, ball.shotOrigin, true);
    }

    void HoleScore::addShot()
    {
        shots++;
    }

    bool HoleScore::apply(const ShotOutcome& outcome)
    {
        if (outcome.kind == ShotOutcome::Kind::holed)
            holed = true;
        else if (outcome.kind == ShotOutcome::Kind::penalty)
            penalties += outcome.penaltyStrokes;
        if (!holed && strokes() >= maxStrokes(par))
            pickedUp = true;
        return pickedUp;
    }

    int32_t HoleScore::toPar() const
    {
        if (pickedUp && !holed)
            return maxStrokes(par) - par;
        return strokes() - par;
    }

    ScoreName scoreName(int32_t strokes, int32_t par)
    {
        if (strokes == 1)
            return ScoreName::holeInOne;
        switch (strokes - par)
        {
            case -3:
                return ScoreName::albatross;
            case -2:
                return ScoreName::eagle;
            case -1:
                return ScoreName::birdie;
            case 0:
                return ScoreName::par;
            case 1:
                return ScoreName::bogey;
            case 2:
                return ScoreName::doubleBogey;
            default:
                return strokes - par < 0 ? ScoreName::albatross : ScoreName::worse;
        }
    }
} // namespace OpenRCT2::Golf
