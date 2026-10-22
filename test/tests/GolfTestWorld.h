/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include <functional>
#include <openrct2/golf/BallPhysics.h>

// A configurable flat or sloped world for the ball physics tests.
namespace GolfTest
{
    using namespace OpenRCT2;
    using namespace OpenRCT2::Golf;

    inline FixedVec3 vec(Fixed x, Fixed y, Fixed z)
    {
        return { x, y, z };
    }
    inline FixedVec3 vec(int x, int y, int z)
    {
        return { Fixed::fromInt(x), Fixed::fromInt(y), Fixed::fromInt(z) };
    }

    struct Box
    {
        FixedVec3 min;
        FixedVec3 max;
    };

    class SeqRandom final : public IBallRandom
    {
    public:
        explicit SeqRandom(uint32_t seed = 1)
            : _state(seed * 2654435761u + 12345u)
        {
        }
        uint32_t next() override
        {
            _state ^= _state << 13;
            _state ^= _state >> 17;
            _state ^= _state << 5;
            return _state;
        }

    private:
        uint32_t _state;
    };

    class TestWorld final : public IBallWorld
    {
    public:
        SurfaceSample surface;       // properties of the ground everywhere
        Fixed slopeX{};              // height change per unit x
        Fixed slopeY{};
        Fixed cliffX = Fixed::fromInt(100000); // beyond this x the ground is 40 lower
        Fixed waterFromX = Fixed::fromInt(100000);
        Fixed streamFromX = Fixed::fromInt(100000);
        Fixed streamToX = Fixed::fromInt(100000);
        Fixed outOfBoundsFromX = Fixed::fromInt(100000);
        FixedVec3 windVelocity{};
        bool hasCup = false;
        CupInfo cup;
        bool hasBox = false;
        Box box;
        Fixed boxRestitution = Fixed::ratio(4, 10);
        Fixed boxFriction = Fixed::ratio(3, 10);
        Fixed boxPassThrough{};
        bool boxIsGolfer = false;

        SurfaceSample surfaceAt(Fixed x, Fixed y) const override
        {
            SurfaceSample s = surface;
            s.height = x * slopeX + y * slopeY;
            if (x >= cliffX)
                s.height -= Fixed::fromInt(40);
            // normal of the plane z = slopeX x + slopeY y: (-sx, -sy, 1) normalised
            s.normal = normalised(FixedVec3{ -slopeX, -slopeY, Fixed::fromInt(1) });
            s.water = x >= waterFromX;
            if (x >= streamFromX && x <= streamToX)
            {
                s.water = true;
                s.stream = true;
            }
            s.outOfBounds = x >= outOfBoundsFromX;
            return s;
        }

        bool obstacleAt(const FixedVec3& p, Fixed radius, ObstacleContact& contact) const override
        {
            if (!hasBox)
                return false;
            if (p.x + radius < box.min.x || p.x - radius > box.max.x || p.y + radius < box.min.y || p.y - radius > box.max.y
                || p.z + radius < box.min.z || p.z - radius > box.max.z)
                return false;
            // Push out along the axis of least penetration.
            const Fixed pen[6] = { p.x + radius - box.min.x, box.max.x - (p.x - radius), p.y + radius - box.min.y,
                                   box.max.y - (p.y - radius), p.z + radius - box.min.z, box.max.z - (p.z - radius) };
            const FixedVec3 normals[6] = { vec(-1, 0, 0), vec(1, 0, 0), vec(0, -1, 0), vec(0, 1, 0), vec(0, 0, -1), vec(0, 0, 1) };
            int best = 0;
            for (int i = 1; i < 6; i++)
                if (pen[i] < pen[best])
                    best = i;
            contact.normal = normals[best];
            contact.depth = pen[best];
            contact.restitution = boxRestitution;
            contact.friction = boxFriction;
            contact.passThroughChance = boxPassThrough;
            contact.id = 5;
            contact.isGolfer = boxIsGolfer;
            contact.golferId = boxIsGolfer ? 77 : 0;
            return true;
        }

        bool cupNear(Fixed, Fixed, CupInfo& out) const override
        {
            if (!hasCup)
                return false;
            out = cup;
            return true;
        }

        FixedVec3 wind() const override
        {
            return windVelocity;
        }
    };

    // A world defined by functions, so each test describes the course it needs.
    class FunctionWorld final : public IBallWorld
    {
    public:
        std::function<double(double, double)> height = [](double, double) { return 0.0; };
        std::function<bool(double, double)> hazard = [](double, double) { return false; };
        std::function<bool(double, double)> blocked = [](double, double) { return false; };
        std::function<bool(double, double)> puttable = [](double, double) { return false; };
        std::function<bool(double, double)> water = [](double, double) { return false; };
        std::function<double(double, double)> roughness = [](double, double) { return 0.15; };

        SurfaceSample surfaceAt(Fixed x, Fixed y) const override
        {
            SurfaceSample s;
            const double px = x.toDouble(), py = y.toDouble();
            s.height = Fixed::fromRaw(static_cast<int64_t>(height(px, py) * 65536.0));
            // Normal from finite differences of the height function.
            const double gx = (height(px + 1, py) - height(px - 1, py)) / 2;
            const double gy = (height(px, py + 1) - height(px, py - 1)) / 2;
            s.normal = normalised(FixedVec3{ Fixed::fromRaw(static_cast<int64_t>(-gx * 65536.0)),
                                             Fixed::fromRaw(static_cast<int64_t>(-gy * 65536.0)), Fixed::fromInt(1) });
            s.hazard = hazard(px, py);
            s.puttable = puttable(px, py);
            s.water = water(px, py);
            s.roughness = Fixed::fromRaw(static_cast<int64_t>(roughness(px, py) * 65536.0));
            if (s.hazard)
            {
                s.distanceFactor = Fixed::ratio(55, 100);
                s.accuracyFactor = Fixed::ratio(75, 100);
            }
            return s;
        }
        // Obstacles with an identity (a golfer, an animal): when set, it gives the contact for a position.
        std::function<bool(const FixedVec3&, ObstacleContact&)> obstacle;

        bool obstacleAt(const FixedVec3& p, Fixed, ObstacleContact& contact) const override
        {
            if (obstacle && obstacle(p, contact))
                return true;
            return blocked(p.x.toDouble(), p.y.toDouble());
        }
    };
} // namespace GolfTest
