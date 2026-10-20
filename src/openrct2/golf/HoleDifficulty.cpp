/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "HoleDifficulty.h"

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
            return table.par4Max + (table.par4Max - table.par3Max) / 2 + 60; // par 5s run to about 600 yards
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
} // namespace OpenRCT2::Golf
