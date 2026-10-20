/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "Fixed.hpp"

namespace OpenRCT2
{
    // 3D vector of fixed-point numbers for simulation state (ARC-020). Same overflow rules as Fixed.
    struct FixedVec3
    {
        Fixed x{};
        Fixed y{};
        Fixed z{};

        constexpr FixedVec3() = default;
        constexpr FixedVec3(Fixed _x, Fixed _y, Fixed _z)
            : x(_x)
            , y(_y)
            , z(_z)
        {
        }

        constexpr FixedVec3 operator+(const FixedVec3& o) const
        {
            return { x + o.x, y + o.y, z + o.z };
        }
        constexpr FixedVec3 operator-(const FixedVec3& o) const
        {
            return { x - o.x, y - o.y, z - o.z };
        }
        constexpr FixedVec3 operator-() const
        {
            return { -x, -y, -z };
        }
        constexpr FixedVec3 operator*(Fixed s) const
        {
            return { x * s, y * s, z * s };
        }
        constexpr FixedVec3& operator+=(const FixedVec3& o)
        {
            return *this = *this + o;
        }
        constexpr FixedVec3& operator-=(const FixedVec3& o)
        {
            return *this = *this - o;
        }
        constexpr bool operator==(const FixedVec3&) const = default;
    };

    constexpr Fixed dot(const FixedVec3& a, const FixedVec3& b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    constexpr FixedVec3 cross(const FixedVec3& a, const FixedVec3& b)
    {
        return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
    }

    inline Fixed lengthSquared(const FixedVec3& v)
    {
        return dot(v, v);
    }

    // Length computed from the raw integers, so tiny vectors keep their precision.
    inline Fixed length(const FixedVec3& v)
    {
        constexpr int64_t kLimit = int64_t{ 1 } << 30; // 3 squares of this stay below 2^64
        int64_t rx = v.x.raw(), ry = v.y.raw(), rz = v.z.raw();
        int32_t shift = 0;
        while ((rx >= kLimit || rx <= -kLimit || ry >= kLimit || ry <= -kLimit || rz >= kLimit || rz <= -kLimit) && shift < 20)
        {
            rx /= 2;
            ry /= 2;
            rz /= 2;
            shift++;
        }
        const auto sum = static_cast<uint64_t>(rx * rx) + static_cast<uint64_t>(ry * ry) + static_cast<uint64_t>(rz * rz);
        return Fixed::fromRaw(static_cast<int64_t>(integerSqrt(sum)) << shift);
    }

    // Unit vector, or the zero vector for a zero input.
    inline FixedVec3 normalised(const FixedVec3& v)
    {
        const Fixed len = length(v);
        if (len == Fixed{})
            return {};
        return { v.x / len, v.y / len, v.z / len };
    }

    // Horizontal (xy) length.
    inline Fixed lengthXY(const FixedVec3& v)
    {
        return length(FixedVec3{ v.x, v.y, Fixed{} });
    }
} // namespace OpenRCT2
