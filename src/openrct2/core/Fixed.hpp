/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include <compare>
#include <cstdint>

namespace OpenRCT2
{
    // Deterministic fixed-point number for simulation state (ARC-020): ball physics, golfer AI, vehicles, animals.
    // 48.16 in a 64 bit integer. Every operation is integer arithmetic, so results are identical on every platform.
    // Products must stay below 2^63 in raw form (|a| * |b| < 2^47 as real numbers), which holds for positions in world
    // units times velocities or ratios; never multiply two positions.
    class Fixed
    {
    public:
        static constexpr int32_t kFractionBits = 16;
        static constexpr int64_t kOne = int64_t{ 1 } << kFractionBits;

        constexpr Fixed() = default;

        static constexpr Fixed fromRaw(int64_t raw)
        {
            Fixed f;
            f._raw = raw;
            return f;
        }
        static constexpr Fixed fromInt(int64_t value)
        {
            return fromRaw(value * kOne);
        }
        // numerator / denominator, rounded towards zero. Use this for constants instead of floating point literals.
        static constexpr Fixed ratio(int64_t numerator, int64_t denominator)
        {
            return fromRaw(numerator * kOne / denominator);
        }

        constexpr int64_t raw() const
        {
            return _raw;
        }
        // Largest integer not above the value.
        constexpr int64_t floor() const
        {
            return _raw >> kFractionBits;
        }
        // Rounded to the nearest integer, halves away from zero.
        constexpr int64_t round() const
        {
            return _raw >= 0 ? (_raw + kOne / 2) >> kFractionBits : -((-_raw + kOne / 2) >> kFractionBits);
        }
        // For rendering, UI and tests only: never feed the result back into simulation state.
        constexpr double toDouble() const
        {
            return static_cast<double>(_raw) / static_cast<double>(kOne);
        }

        constexpr Fixed operator-() const
        {
            return fromRaw(-_raw);
        }
        constexpr Fixed operator+(Fixed o) const
        {
            return fromRaw(_raw + o._raw);
        }
        constexpr Fixed operator-(Fixed o) const
        {
            return fromRaw(_raw - o._raw);
        }
        constexpr Fixed operator*(Fixed o) const
        {
            return fromRaw((_raw * o._raw) >> kFractionBits);
        }
        // Division by zero yields zero, so a broken input cannot crash the simulation.
        constexpr Fixed operator/(Fixed o) const
        {
            return o._raw == 0 ? Fixed{} : fromRaw((_raw * kOne) / o._raw);
        }
        constexpr Fixed operator*(int64_t o) const
        {
            return fromRaw(_raw * o);
        }
        constexpr Fixed operator/(int64_t o) const
        {
            return o == 0 ? Fixed{} : fromRaw(_raw / o);
        }
        constexpr Fixed& operator+=(Fixed o)
        {
            _raw += o._raw;
            return *this;
        }
        constexpr Fixed& operator-=(Fixed o)
        {
            _raw -= o._raw;
            return *this;
        }
        constexpr Fixed& operator*=(Fixed o)
        {
            return *this = *this * o;
        }

        constexpr bool operator==(const Fixed&) const = default;
        constexpr auto operator<=>(const Fixed&) const = default;

    private:
        int64_t _raw = 0;
    };

    constexpr Fixed operator*(int64_t a, Fixed b)
    {
        return b * a;
    }

    constexpr Fixed abs(Fixed v)
    {
        return v < Fixed{} ? -v : v;
    }
    constexpr Fixed min(Fixed a, Fixed b)
    {
        return a < b ? a : b;
    }
    constexpr Fixed max(Fixed a, Fixed b)
    {
        return a < b ? b : a;
    }
    constexpr Fixed clamp(Fixed v, Fixed lo, Fixed hi)
    {
        return v < lo ? lo : (hi < v ? hi : v);
    }
    constexpr Fixed lerp(Fixed a, Fixed b, Fixed t)
    {
        return a + (b - a) * t;
    }

    // Square root of a non-negative value (negative gives zero), exact to the last bit of the integer square root.
    Fixed sqrt(Fixed v);
    // Sine and cosine of an angle in radians, accurate to about 1e-4.
    Fixed sin(Fixed radians);
    Fixed cos(Fixed radians);

    constexpr Fixed kFixedPi = Fixed::fromRaw(205887); // pi * 65536
    constexpr Fixed kFixedTwoPi = Fixed::fromRaw(411775);
    constexpr Fixed kFixedHalfPi = Fixed::fromRaw(102944);
} // namespace OpenRCT2
