/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "HoleDifficulty.h"

#include "PlayAI.h"
#include "Rules.h"

#include <algorithm>

namespace OpenRCT2::Golf
{
    namespace
    {
        struct SamplePoint
        {
            Fixed x, y;
            Fixed dirX, dirY; // unit direction of the line at this point
        };

        // Upper end of the par table for a par: the longest hole of that par.
        int32_t parUpperYards(int32_t par)
        {
            const auto& table = kParTables[0];
            if (par <= 3)
                return table.par3Max;
            if (par == 4)
                return table.par4Max;
            if (par == 5)
                return table.par5Max;
            return table.par5Max + (table.par5Max - table.par4Max) / 2; // par 6s run on past the par 5 range
        }

        std::vector<SamplePoint> sampleLine(const std::vector<HolePoint>& line)
        {
            std::vector<SamplePoint> points;
            for (size_t i = 1; i < line.size(); i++)
            {
                const Fixed ax = Fixed::fromInt(line[i - 1].x), ay = Fixed::fromInt(line[i - 1].y);
                const Fixed bx = Fixed::fromInt(line[i].x), by = Fixed::fromInt(line[i].y);
                const FixedVec3 delta{ bx - ax, by - ay, Fixed{} };
                const Fixed len = length(delta);
                if (len == Fixed{})
                    continue;
                const Fixed dx = delta.x / len, dy = delta.y / len;
                const int32_t steps = std::max<int32_t>(1, static_cast<int32_t>(len.floor()) / kDifficultySampleStep);
                for (int32_t s = 0; s <= steps; s++)
                {
                    const Fixed t = Fixed::ratio(s, steps);
                    points.push_back({ ax + (bx - ax) * t, ay + (by - ay) * t, dx, dy });
                }
            }
            return points;
        }

        bool isBadLanding(const SurfaceSample& s)
        {
            return s.water || s.outOfBounds || s.hazard || s.roughness >= Fixed::ratio(70, 100);
        }

        int32_t scaled(Fixed value, Fixed fullAt, int32_t maxPoints)
        {
            const Fixed ratio = clamp(value / fullAt, Fixed{}, Fixed::fromInt(1));
            return static_cast<int32_t>((ratio * maxPoints).round());
        }
    } // namespace

    HoleDifficulty rateHole(const Hole& hole, const IBallWorld& world)
    {
        HoleDifficulty d;
        const auto tee = hole.firstTee();
        if (!tee.has_value() || !hole.cup.has_value())
            return d;
        const auto line = hole.playingLine(*tee);
        const auto samples = sampleLine(line);
        if (samples.empty())
            return d;

        // Length against the par: the square of how close the hole is to the longest of its par.
        const int32_t par = hole.par();
        const int32_t yards = hole.lengthYards(*tee);
        {
            const Fixed ratio = clamp(Fixed::ratio(yards, parUpperYards(par)), Fixed{}, Fixed::fromInt(1));
            d.length = static_cast<int32_t>((ratio * ratio * kMaxLengthDifficulty).round());
        }

        // Hazards in a corridor beside the line, and obstacles right on it.
        int64_t corridorSamples = 0, badSamples = 0, lineSamples = 0, blockedSamples = 0;
        Fixed totalRise{};
        Fixed lastHeight{};
        bool haveLast = false;
        for (const auto& p : samples)
        {
            const auto onLine = world.surfaceAt(p.x, p.y);
            if (haveLast)
                totalRise += abs(onLine.height - lastHeight);
            lastHeight = onLine.height;
            haveLast = true;

            for (int32_t side = -kHazardCorridorTiles; side <= kHazardCorridorTiles; side++)
            {
                // Perpendicular offset in tiles; the line itself counts too.
                const Fixed offset = Fixed::fromInt(side * 32);
                const Fixed sx = p.x - p.dirY * offset;
                const Fixed sy = p.y + p.dirX * offset;
                const auto s = world.surfaceAt(sx, sy);
                corridorSamples++;
                if (isBadLanding(s))
                    badSamples++;
            }

            lineSamples++;
            ObstacleContact contact;
            // A ball flying along the line passes at the height of a tree trunk or a roof.
            const Fixed flight = onLine.height + Fixed::fromInt(6);
            if (world.obstacleAt({ p.x, p.y, flight }, Fixed::ratio(1, 2), contact))
                blockedSamples++;
        }
        if (corridorSamples > 0)
            d.hazards = scaled(Fixed::ratio(static_cast<int64_t>(badSamples), corridorSamples), Fixed::ratio(1, 3), kMaxHazardDifficulty);
        if (lineSamples > 0)
            d.obstacles = scaled(Fixed::ratio(static_cast<int64_t>(blockedSamples), lineSamples), Fixed::ratio(1, 10), kMaxObstacleDifficulty);

        // Green slope: the tilt of the ground in a disc of about 3 tiles around the cup.
        {
            Fixed tilt{};
            int32_t count = 0;
            const Fixed cx = Fixed::fromInt(hole.cup->x), cy = Fixed::fromInt(hole.cup->y);
            for (int32_t dy = -3; dy <= 3; dy++)
                for (int32_t dx = -3; dx <= 3; dx++)
                {
                    if (dx * dx + dy * dy > 9)
                        continue;
                    const auto s = world.surfaceAt(cx + Fixed::fromInt(dx * 16), cy + Fixed::fromInt(dy * 16));
                    if (s.normal.z > Fixed{})
                        tilt += sqrt(s.normal.x * s.normal.x + s.normal.y * s.normal.y) / s.normal.z;
                    count++;
                }
            if (count > 0)
                d.greenSlope = scaled(tilt / count, Fixed::ratio(kSteepGreenPercent, 100), kMaxGreenSlopeDifficulty);
        }

        // Elevation: how far the ground rises and falls along the line compared to its length (a 10% average grade is the
        // worst).
        {
            const Fixed lineUnits = Fixed::ratio(static_cast<int64_t>(yards) * 32, 5);
            if (lineUnits > Fixed{})
                d.elevation = scaled(totalRise / lineUnits, Fixed::ratio(1, 10), kMaxElevationDifficulty);
        }

        d.total = std::min(100, d.length + d.hazards + d.greenSlope + d.obstacles + d.elevation);
        return d;
    }

    int32_t golferLevelTenths(const GolferSkills& skills)
    {
        // The average of the four skills, 0..100, is the level in tenths: skill 50 is level 5.0.
        return (skills.longPlay + skills.accuracy + skills.foolproof + skills.physique) / 4;
    }

    bool isTooHard(const HoleDifficulty& difficulty, const GolferSkills& skills)
    {
        return difficulty.total - golferLevelTenths(skills) > kTooHardThresholdTenths;
    }

    namespace
    {
        Fixed horizontal(const FixedVec3& a, const FixedVec3& b)
        {
            return length(FixedVec3{ a.x - b.x, a.y - b.y, Fixed{} });
        }

        class DefaultTestRandom final : public IBallRandom
        {
        private:
            uint32_t _seed = 0x12345678;

        public:
            uint32_t next() override
            {
                _seed = _seed * 1103515245 + 12345;
                return _seed;
            }
        };

        class HoleCupWorld final : public IBallWorld
        {
        private:
            const IBallWorld& _inner;
            FixedVec3 _cupPos;

        public:
            HoleCupWorld(const IBallWorld& inner, const FixedVec3& cupPos)
                : _inner(inner)
                , _cupPos(cupPos)
            {
            }
            SurfaceSample surfaceAt(Fixed x, Fixed y) const override
            {
                return _inner.surfaceAt(x, y);
            }
            bool obstacleAt(const FixedVec3& position, Fixed radius, ObstacleContact& contact) const override
            {
                return _inner.obstacleAt(position, radius, contact);
            }
            FixedVec3 wind() const override
            {
                return _inner.wind();
            }
            bool cupNear(Fixed x, Fixed y, CupInfo& cup) const override
            {
                if (_inner.cupNear(x, y, cup))
                    return true;
                if (abs(x - _cupPos.x) <= Fixed::fromInt(8) && abs(y - _cupPos.y) <= Fixed::fromInt(8))
                {
                    cup.position = _cupPos;
                    cup.radius = Fixed::fromInt(3);
                    return true;
                }
                return false;
            }
        };
    } // namespace

    TestHoleResult testHole(const Hole& hole, const IBallWorld& world, IBallRandom* rng)
    {
        TestHoleResult result;
        const auto teeCategory = hole.firstTee();
        if (!teeCategory.has_value() || !hole.cup.has_value())
            return result;

        DefaultTestRandom defaultRng;
        auto& activeRng = (rng != nullptr) ? *rng : defaultRng;

        result.par = hole.par();
        result.difficulty = rateHole(hole, world);

        const HolePoint tee = *hole.tees[static_cast<size_t>(*teeCategory)];
        const HolePoint cup = *hole.cup;
        const auto line = hole.playingLine(*teeCategory);

        const auto cupSurface = world.surfaceAt(Fixed::fromInt(cup.x), Fixed::fromInt(cup.y));
        const FixedVec3 cupPos = { Fixed::fromInt(cup.x), Fixed::fromInt(cup.y), cupSurface.height };
        HoleCupWorld testWorld(world, cupPos);

        Ball ball;
        placeBall(ball, testWorld, Fixed::fromInt(tee.x), Fixed::fromInt(tee.y), BallState::Teed);

        result.shotPositions.push_back(tee);

        const GolferSkills skills{ kAverageSkill, kAverageSkill, kAverageSkill, kAverageSkill };
        const int32_t strokeLimit = maxStrokes(result.par);

        while (result.strokes < strokeLimit)
        {
            if (horizontal(ball.position, cupPos) <= Fixed::fromInt(4))
            {
                result.reachedCup = true;
                break;
            }

            PlayContext ctx;
            ctx.world = &testWorld;
            ctx.skills = skills;
            ctx.energy = 255;
            ctx.equipmentLevel = 0;
            ctx.riskTolerance = Fixed::ratio(5, 10);
            ctx.ball = ball.position;
            ctx.line = line;
            ctx.cup = cupPos;
            ctx.teeShot = (result.strokes == 0);
            ctx.miniPutt = false;

            const auto decision = decideShot(ctx);

            ShotRequest req;
            req.club = decision.club;
            req.power = decision.power;
            req.aimX = decision.aimX;
            req.aimY = decision.aimY;
            req.energy = 255;
            req.skills = skills;
            req.equipmentLevel = 0;
            const auto here = testWorld.surfaceAt(ball.position.x, ball.position.y);
            req.lie = lieOf(here);
            req.surfaceRoughness = here.roughness;

            const auto shot = computeShot(req, activeRng);
            applyShot(ball, shot);
            result.strokes++;

            int32_t stepCount = 0;
            while (!isSettled(ball.state) && stepCount < 2000)
            {
                stepBall(ball, testWorld, activeRng);
                stepCount++;
            }

            result.shotPositions.push_back(HolePoint{ static_cast<int32_t>(ball.position.x.floor()), static_cast<int32_t>(ball.position.y.floor()) });

            const auto outcome = judgeShot(ball, testWorld);
            if (outcome.kind == ShotOutcome::Kind::holed || ball.state == BallState::Holed)
            {
                result.reachedCup = true;
                break;
            }
            if (horizontal(ball.position, cupPos) <= Fixed::fromInt(4))
            {
                result.reachedCup = true;
                break;
            }
            if (outcome.kind == ShotOutcome::Kind::penalty)
            {
                result.strokes += outcome.penaltyStrokes;
                placeBall(ball, testWorld, outcome.dropPosition.x, outcome.dropPosition.y, BallState::AtRest);
            }
        }

        result.scoreAgainstPar = result.strokes - result.par;
        int32_t adjusted = result.difficulty.total + result.scoreAgainstPar * 10;
        if (!result.reachedCup)
            adjusted = std::max(adjusted, 80);
        result.estimatedDifficulty = std::clamp(adjusted, 0, 100);

        return result;
    }
} // namespace OpenRCT2::Golf
