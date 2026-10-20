/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Fixed.hpp"

namespace OpenRCT2
{
    namespace
    {
        uint64_t isqrt(uint64_t n)
        {
            uint64_t result = 0;
            uint64_t bit = uint64_t{ 1 } << 62;
            while (bit > n)
                bit >>= 2;
            while (bit != 0)
            {
                if (n >= result + bit)
                {
                    n -= result + bit;
                    result = (result >> 1) + bit;
                }
                else
                    result >>= 1;
                bit >>= 2;
            }
            return result;
        }

        // Taylor series on [-pi/2, pi/2].
        Fixed sinHalfTurn(Fixed x)
        {
            const Fixed x2 = x * x;
            const Fixed x3 = x2 * x;
            const Fixed x5 = x3 * x2;
            const Fixed x7 = x5 * x2;
            const Fixed x9 = x7 * x2;
            return x - x3 / 6 + x5 / 120 - x7 / 5040 + x9 / 362880;
        }
    } // namespace

    Fixed sqrt(Fixed v)
    {
        if (v <= Fixed{})
            return Fixed{};
        return Fixed::fromRaw(static_cast<int64_t>(isqrt(static_cast<uint64_t>(v.raw()) << Fixed::kFractionBits)));
    }

    Fixed sin(Fixed radians)
    {
        // Reduce to [-pi, pi].
        int64_t turns = (radians + kFixedPi).raw() / kFixedTwoPi.raw();
        Fixed x = radians - kFixedTwoPi * turns;
        if (x > kFixedPi)
            x -= kFixedTwoPi;
        else if (x < -kFixedPi)
            x += kFixedTwoPi;
        // Fold to [-pi/2, pi/2] using sin(pi - x) = sin(x).
        if (x > kFixedHalfPi)
            x = kFixedPi - x;
        else if (x < -kFixedHalfPi)
            x = -kFixedPi - x;
        return sinHalfTurn(x);
    }

    Fixed cos(Fixed radians)
    {
        return sin(radians + kFixedHalfPi);
    }
} // namespace OpenRCT2
