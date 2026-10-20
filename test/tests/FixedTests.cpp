/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <cmath>
#include <gtest/gtest.h>
#include <openrct2/core/Fixed.hpp>

using namespace OpenRCT2;

TEST(Fixed, ConstructsFromIntegersAndRatios)
{
    EXPECT_EQ(Fixed::fromInt(3).raw(), 3 * 65536);
    EXPECT_EQ(Fixed::ratio(1, 2).raw(), 32768);
    EXPECT_EQ(Fixed::ratio(-1, 4).raw(), -16384);
    EXPECT_EQ(Fixed{}.raw(), 0);
    EXPECT_EQ(Fixed::fromInt(5).floor(), 5);
    EXPECT_EQ(Fixed::ratio(-1, 2).floor(), -1);
    EXPECT_EQ(Fixed::ratio(5, 2).round(), 3);
    EXPECT_EQ(Fixed::ratio(-5, 2).round(), -3);
    EXPECT_EQ(Fixed::ratio(7, 4).round(), 2);
}

TEST(Fixed, ArithmeticIsExactForRepresentableValues)
{
    const auto a = Fixed::ratio(3, 2);
    const auto b = Fixed::ratio(5, 4);
    EXPECT_EQ(a + b, Fixed::ratio(11, 4));
    EXPECT_EQ(a - b, Fixed::ratio(1, 4));
    EXPECT_EQ(a * b, Fixed::ratio(15, 8));
    EXPECT_EQ(a / b, Fixed::ratio(6, 5)); // 1.2 is not representable but both sides truncate the same way
    EXPECT_EQ(-a, Fixed::ratio(-3, 2));
    EXPECT_EQ(a * 4, Fixed::fromInt(6));
    EXPECT_EQ(4 * a, Fixed::fromInt(6));
    EXPECT_EQ(a / 3, Fixed::ratio(1, 2));
    auto c = a;
    c += b;
    c -= Fixed::fromInt(1);
    c *= Fixed::fromInt(2);
    EXPECT_EQ(c, Fixed::ratio(7, 2));
}

TEST(Fixed, DivisionByZeroIsZero)
{
    EXPECT_EQ(Fixed::fromInt(5) / Fixed{}, Fixed{});
    EXPECT_EQ(Fixed::fromInt(5) / int64_t{ 0 }, Fixed{});
}

TEST(Fixed, ComparesAndClamps)
{
    const auto one = Fixed::fromInt(1);
    const auto two = Fixed::fromInt(2);
    EXPECT_LT(one, two);
    EXPECT_GE(two, two);
    EXPECT_EQ(min(one, two), one);
    EXPECT_EQ(max(one, two), two);
    EXPECT_EQ(abs(-two), two);
    EXPECT_EQ(clamp(Fixed::fromInt(5), one, two), two);
    EXPECT_EQ(clamp(Fixed::fromInt(-5), one, two), one);
    EXPECT_EQ(clamp(Fixed::ratio(3, 2), one, two), Fixed::ratio(3, 2));
    EXPECT_EQ(lerp(one, Fixed::fromInt(3), Fixed::ratio(1, 2)), two);
}

TEST(Fixed, SquareRoot)
{
    EXPECT_EQ(sqrt(Fixed::fromInt(0)), Fixed{});
    EXPECT_EQ(sqrt(Fixed::fromInt(-4)), Fixed{});
    EXPECT_EQ(sqrt(Fixed::fromInt(4)), Fixed::fromInt(2));
    EXPECT_EQ(sqrt(Fixed::fromInt(144)), Fixed::fromInt(12));
    EXPECT_EQ(sqrt(Fixed::ratio(1, 4)), Fixed::ratio(1, 2));
    EXPECT_NEAR(sqrt(Fixed::fromInt(2)).toDouble(), std::sqrt(2.0), 1.0 / 65536);
    EXPECT_NEAR(sqrt(Fixed::fromInt(1000000)).toDouble(), 1000.0, 1.0 / 65536);
}

TEST(Fixed, SineAndCosineAreAccurateOverManyTurns)
{
    double worst = 0;
    for (int deg = -1080; deg <= 1080; deg += 5)
    {
        const double radians = deg * 3.14159265358979 / 180.0;
        const auto fixedRadians = Fixed::fromRaw(static_cast<int64_t>(std::llround(radians * 65536.0)));
        worst = std::max(worst, std::fabs(sin(fixedRadians).toDouble() - std::sin(radians)));
        worst = std::max(worst, std::fabs(cos(fixedRadians).toDouble() - std::cos(radians)));
    }
    EXPECT_LT(worst, 3e-4);
    EXPECT_EQ(sin(Fixed{}), Fixed{});
    EXPECT_NEAR(cos(Fixed{}).toDouble(), 1.0, 1e-4);
}

TEST(Fixed, ArithmeticHasNoFloatingPointDependency)
{
    // A fixed chain of operations must give the same raw value everywhere; this one is hard coded.
    Fixed v = Fixed::fromInt(1);
    for (int i = 0; i < 100; i++)
        v = v * Fixed::ratio(99, 100) + Fixed::ratio(1, 1000);
    EXPECT_EQ(v.raw(), 28050); // checked against an independent integer implementation
}
