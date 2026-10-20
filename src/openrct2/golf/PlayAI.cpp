/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "PlayAI.h"

#include <algorithm>

namespace OpenRCT2::Golf
{
    namespace
    {
        const Fixed kOne = Fixed::fromInt(1);

        Fixed horizontalDistance(const FixedVec3& a, const FixedVec3& b)
        {
            return length(FixedVec3{ a.x - b.x, a.y - b.y, Fixed{} });
        }

        bool isBad(const SurfaceSample& s)
        {
            return s.water || s.outOfBounds || s.hazard;
        }

        // The point `distance` along the playing line, starting from the point of the line nearest to the ball.
        FixedVec3 pointAlongLine(const std::vector<HolePoint>& line, const FixedVec3& ball, Fixed distance, const IBallWorld& world)
        {
            if (line.size() < 2)
                return ball;
            // Find the segment nearest to the ball and the distance already covered along it.
            size_t bestSegment = 0;
            Fixed bestDistance = Fixed::fromInt(1000000);
            Fixed bestAlong{};
            for (size_t i = 1; i < line.size(); i++)
            {
                const FixedVec3 a{ Fixed::fromInt(line[i - 1].x), Fixed::fromInt(line[i - 1].y), Fixed{} };
                const FixedVec3 b{ Fixed::fromInt(line[i].x), Fixed::fromInt(line[i].y), Fixed{} };
                const FixedVec3 ab = b - a;
                const Fixed len = length(ab);
                if (len == Fixed{})
                    continue;
                const FixedVec3 dir{ ab.x / len, ab.y / len, Fixed{} };
                const Fixed t = clamp(dot(FixedVec3{ ball.x - a.x, ball.y - a.y, Fixed{} }, dir), Fixed{}, len);
                const FixedVec3 nearest{ a.x + dir.x * t, a.y + dir.y * t, Fixed{} };
                const Fixed d = horizontalDistance(nearest, ball);
                if (d < bestDistance)
                {
                    bestDistance = d;
                    bestSegment = i;
                    bestAlong = t;
                }
            }
            // Walk forward from there.
            Fixed remaining = distance;
            FixedVec3 position{};
            for (size_t i = bestSegment; i < line.size(); i++)
            {
                const FixedVec3 a{ Fixed::fromInt(line[i - 1].x), Fixed::fromInt(line[i - 1].y), Fixed{} };
                const FixedVec3 b{ Fixed::fromInt(line[i].x), Fixed::fromInt(line[i].y), Fixed{} };
                const FixedVec3 ab = b - a;
                const Fixed len = length(ab);
                const Fixed start = i == bestSegment ? bestAlong : Fixed{};
                const Fixed available = len - start;
                if (len == Fixed{})
                    continue;
                if (remaining <= available || i + 1 == line.size())
                {
                    const Fixed t = min(start + remaining, len);
                    position = { a.x + ab.x * t / len, a.y + ab.y * t / len, Fixed{} };
                    remaining = Fixed{};
                    break;
                }
                remaining -= available;
                position = b;
            }
            position.z = world.surfaceAt(position.x, position.y).height;
            return position;
        }

        FixedVec3 directionTo(const FixedVec3& from, const FixedVec3& to)
        {
            return normalised(FixedVec3{ to.x - from.x, to.y - from.y, Fixed{} });
        }

        ShotRequest requestFor(const PlayContext& c, ClubId club, const Lie& lie)
        {
            ShotRequest r;
            r.club = club;
            r.skills = c.skills;
            r.energy = c.energy;
            r.equipmentLevel = c.equipmentLevel;
            r.lie = lie;
            return r;
        }

        // Quality of a landing: fairway and green are good, deep rough and sand bad. 0..1.
        Fixed lieQuality(const SurfaceSample& s)
        {
            if (isBad(s))
                return Fixed{};
            return clamp((s.distanceFactor + s.accuracyFactor) / 2, Fixed{}, kOne);
        }

        struct Candidate
        {
            ClubId club;
            Fixed power;
            FixedVec3 target;
            Fixed score;
        };

        // How exposed a landing at `target` is: the share of a ring of points around it, as wide as the club's error, that
        // lies in a hazard (0..1), and the lie quality at the target.
        void evaluateLanding(
            const PlayContext& c, const ShotRequest& r, const FixedVec3& from, const FixedVec3& target, Fixed& badShare,
            Fixed& quality)
        {
            const Fixed distance = horizontalDistance(from, target);
            const Fixed lateral = directionSpread(r) * 2 * distance;   // two deviations to the side
            const Fixed longitudinal = distanceSpread(r) * distance;   // one deviation short and long
            const FixedVec3 dir = directionTo(from, target);
            const FixedVec3 side{ -dir.y, dir.x, Fixed{} };
            const FixedVec3 offsets[9] = {
                { Fixed{}, Fixed{}, Fixed{} },          { Fixed{}, lateral, Fixed{} },      { Fixed{}, -lateral, Fixed{} },
                { longitudinal, Fixed{}, Fixed{} },     { -longitudinal, Fixed{}, Fixed{} }, { longitudinal, lateral, Fixed{} },
                { longitudinal, -lateral, Fixed{} },    { -longitudinal, lateral, Fixed{} }, { -longitudinal, -lateral, Fixed{} },
            };
            int32_t bad = 0;
            for (const auto& o : offsets)
            {
                const Fixed x = target.x + dir.x * o.x + side.x * o.y;
                const Fixed y = target.y + dir.y * o.x + side.y * o.y;
                if (isBad(c.world->surfaceAt(x, y)))
                    bad++;
            }
            badShare = Fixed::ratio(bad, 9);
            quality = lieQuality(c.world->surfaceAt(target.x, target.y));
        }
    } // namespace

    Fixed puttBreak(const IBallWorld& world, const FixedVec3& ball, const FixedVec3& cup)
    {
        const FixedVec3 delta{ cup.x - ball.x, cup.y - ball.y, Fixed{} };
        const Fixed len = length(delta);
        if (len == Fixed{})
            return Fixed{};
        const FixedVec3 dir{ delta.x / len, delta.y / len, Fixed{} };
        const FixedVec3 left{ -dir.y, dir.x, Fixed{} };
        // Sideways slope at five points along the putt: the ground falls to the left when the normal leans to the left.
        Fixed sum{};
        int32_t count = 0;
        for (int32_t i = 1; i <= 5; i++)
        {
            const Fixed t = Fixed::ratio(i, 6);
            const auto s = world.surfaceAt(ball.x + dir.x * len * t, ball.y + dir.y * len * t);
            if (s.normal.z <= Fixed{})
                continue;
            sum += (s.normal.x * left.x + s.normal.y * left.y) / s.normal.z;
            count++;
        }
        return count == 0 ? Fixed{} : sum / count;
    }

    ShotDecision decideShot(const PlayContext& c)
    {
        ShotDecision decision;
        if (c.world == nullptr)
            return decision;
        const auto here = c.world->surfaceAt(c.ball.x, c.ball.y);
        const Lie lie = lieOf(here);
        const BallParams params;

        const FixedVec3 cup{ c.cup.x, c.cup.y, c.cup.z };
        const Fixed toCup = horizontalDistance(c.ball, cup);

        if (mustPutt(here))
        {
            // HOL-031: read the break in proportion to Accuracy and aim uphill of it; reach the cup plus a margin.
            ShotRequest r = requestFor(c, ClubId::putter, lie);
            r.surfaceRoughness = here.roughness;
            r.power = kOne;
            const Fixed maxDistance = shotDistance(r);

            const Fixed sideSlope = puttBreak(*c.world, c.ball, cup);
            const Fixed reading = Fixed::ratio(c.skills.accuracy, 100);
            // Ground falling to the left pushes the ball left, so aim to the right of the cup: a lateral offset of about
            // the slope times the distance, scaled by how well the golfer reads it.
            const Fixed lateral = sideSlope * toCup * reading * 4;
            const FixedVec3 dir = directionTo(c.ball, cup);
            const FixedVec3 left{ -dir.y, dir.x, Fixed{} };
            FixedVec3 aimPoint{ cup.x - left.x * lateral, cup.y - left.y * lateral, cup.z };
            const FixedVec3 aim = directionTo(c.ball, aimPoint);

            // Uphill needs more, downhill less: the rise costs g * dh of energy, which the green's friction pays for over
            // d = g * dh / a extra units of roll.
            const Fixed deceleration = max(Fixed::ratio(1, 1000), params.rollResistance * here.roughness);
            const Fixed rise = cup.z - c.ball.z;
            Fixed effective = toCup + rise * params.gravity / deceleration;
            effective = max(effective, toCup / 3);
            const Fixed margin = max(Fixed::fromInt(kPuttMarginMinUnits), toCup * kPuttMarginPercent / 100);

            decision.club = ClubId::putter;
            decision.putt = true;
            decision.target = cup;
            decision.aimX = aim.x;
            decision.aimY = aim.y;
            decision.expectedDistance = effective + margin;
            decision.power = maxDistance > Fixed{} ? clamp((effective + margin) / maxDistance, Fixed::ratio(5, 100), kOne) : kOne;
            return decision;
        }

        // A full shot. Candidate clubs aim at the point along the playing line that they reach with a full swing, or at
        // the cup itself when it is within reach.
        Candidate best{ ClubId::driver, kOne, c.ball, Fixed::fromInt(-1000000) };
        bool haveBest = false;
        for (const auto& club : allClubs())
        {
            if (club.isPutter)
                continue;
            ShotRequest r = requestFor(c, club.id, lie);
            const Fixed carry = shotDistance(r);
            if (carry <= Fixed{})
                continue;

            Fixed power = kOne;
            FixedVec3 target;
            if (carry >= toCup)
            {
                // The cup is within reach of this club: take a shorter swing. A swing under a third of the power is a
                // job for a shorter club.
                power = toCup / carry;
                if (power < Fixed::ratio(kMinSwingPercent, 100))
                    continue;
                target = cup;
            }
            else
            {
                target = pointAlongLine(c.line, c.ball, carry, *c.world);
            }
            r.power = power;

            Fixed badShare, quality;
            evaluateLanding(c, r, c.ball, target, badShare, quality);

            // Progress: distance left to the cup after the shot (smaller is better); the hazard share costs more for the
            // careful, and the lie where it lands is worth something for the next shot.
            const Fixed remaining = horizontalDistance(target, cup);
            const Fixed careful = kOne - c.riskTolerance / 2;
            Fixed score = -remaining - badShare * kHazardPenaltyUnits * careful + quality * kLieQualityUnits;
            // The miss to expect: the error of the club in direction and distance over the length of the shot.
            const Fixed shotLength = horizontalDistance(c.ball, target);
            score -= shotLength * (directionSpread(r) + distanceSpread(r));
            // Golfers like a full, controlled swing: anything away from nine tenths of the power costs a little.
            score -= abs(power - Fixed::ratio(9, 10)) * kSwingComfortUnits;

            if (!haveBest || score > best.score)
            {
                best = { club.id, power, target, score };
                haveBest = true;
            }
        }
        if (!haveBest)
        {
            // Closer than any club likes: a short chip with the sand wedge.
            ShotRequest r = requestFor(c, ClubId::sandWedge, lie);
            const Fixed carry = shotDistance(r);
            best.club = ClubId::sandWedge;
            best.power = carry > Fixed{} ? clamp(toCup / carry, Fixed::ratio(15, 100), kOne) : kOne;
            best.target = toCup <= carry ? cup : pointAlongLine(c.line, c.ball, carry, *c.world);
        }

        const FixedVec3 aim = directionTo(c.ball, best.target);
        decision.club = best.club;
        decision.power = best.power;
        decision.target = best.target;
        decision.aimX = aim.x;
        decision.aimY = aim.y;
        decision.putt = false;
        decision.expectedDistance = horizontalDistance(c.ball, best.target);
        return decision;
    }
} // namespace OpenRCT2::Golf
