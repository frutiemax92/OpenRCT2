/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <gtest/gtest.h>
#include <openrct2/core/FixedVec.hpp>

using namespace OpenRCT2;

namespace
{
    FixedVec3 v(int x, int y, int z)
    {
        return { Fixed::fromInt(x), Fixed::fromInt(y), Fixed::fromInt(z) };
    }
} // namespace

TEST(FixedVec, ArithmeticIsComponentWise)
{
    EXPECT_EQ(v(1, 2, 3) + v(4, 5, 6), v(5, 7, 9));
    EXPECT_EQ(v(4, 5, 6) - v(1, 2, 3), v(3, 3, 3));
    EXPECT_EQ(-v(1, -2, 3), v(-1, 2, -3));
    EXPECT_EQ(v(1, 2, 3) * Fixed::fromInt(2), v(2, 4, 6));
    auto a = v(1, 1, 1);
    a += v(1, 2, 3);
    a -= v(0, 0, 1);
    EXPECT_EQ(a, v(2, 3, 3));
}

TEST(FixedVec, DotAndCross)
{
    EXPECT_EQ(dot(v(1, 2, 3), v(4, 5, 6)), Fixed::fromInt(32));
    EXPECT_EQ(cross(v(1, 0, 0), v(0, 1, 0)), v(0, 0, 1));
    EXPECT_EQ(cross(v(0, 1, 0), v(1, 0, 0)), v(0, 0, -1));
    EXPECT_EQ(cross(v(2, 3, 4), v(2, 3, 4)), v(0, 0, 0));
}

TEST(FixedVec, LengthAndNormalise)
{
    EXPECT_EQ(length(v(3, 4, 0)), Fixed::fromInt(5));
    EXPECT_EQ(length(v(0, 0, 0)), Fixed{});
    EXPECT_EQ(lengthXY(v(3, 4, 100)), Fixed::fromInt(5));
    EXPECT_EQ(lengthSquared(v(1, 2, 2)), Fixed::fromInt(9));
    const auto n = normalised(v(0, 0, 7));
    EXPECT_EQ(n, v(0, 0, 1));
    EXPECT_EQ(normalised(v(0, 0, 0)), v(0, 0, 0));
    const auto diag = normalised(v(3, 4, 0));
    EXPECT_NEAR(diag.x.toDouble(), 0.6, 1e-4);
    EXPECT_NEAR(diag.y.toDouble(), 0.8, 1e-4);
}

TEST(FixedVec, LengthOfHugeVectorsDoesNotOverflow)
{
    EXPECT_NEAR(length(v(30000, 40000, 0)).toDouble(), 50000.0, 2.0);
}
