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
#include <openrct2/model/TerrainAlignment.h>

using namespace OpenRCT2::Model;

namespace
{
    constexpr float kPi = 3.14159265f;

    HeightSampler flat(float z)
    {
        return [z](float, float) { return z; };
    }
    // Rises by `slope` map units per unit towards +x.
    HeightSampler risingEast(float slope)
    {
        return [slope](float x, float) { return x * slope; };
    }
    HeightSampler risingNorth(float slope)
    {
        return [slope](float, float y) { return y * slope; };
    }
} // namespace

TEST(TerrainAlignment, FlatGroundIsUpright)
{
    const auto a = computeAlignment(flat(16), 100, 100, 0.7f, {});
    EXPECT_FLOAT_EQ(a.pitch, 0);
    EXPECT_FLOAT_EQ(a.roll, 0);
    EXPECT_FLOAT_EQ(a.height, 16);
}

TEST(TerrainAlignment, ClimbingTipsTheNoseUp)
{
    // Heading +x up a 1:2 slope. Positive pitch is nose down in the rasteriser, so climbing is negative.
    const auto a = computeAlignment(risingEast(0.5f), 100, 100, 0, {});
    EXPECT_NEAR(a.pitch, -std::atan(0.5f), 1e-5f);
    EXPECT_NEAR(a.roll, 0, 1e-5f);
    EXPECT_NEAR(a.height, 50, 1e-4f);
    // Going down the same slope tips the nose down.
    const auto down = computeAlignment(risingEast(0.5f), 100, 100, kPi, {});
    EXPECT_NEAR(down.pitch, std::atan(0.5f), 1e-5f);
}

TEST(TerrainAlignment, ASlopeAcrossTheHeadingRolls)
{
    // Heading +x, the ground rises towards +y which is the left side.
    const auto a = computeAlignment(risingNorth(0.25f), 100, 100, 0, {});
    EXPECT_NEAR(a.roll, std::atan(0.25f), 1e-5f);
    EXPECT_NEAR(a.pitch, 0, 1e-5f);
    // Heading +y, the same ground now rises ahead.
    const auto ahead = computeAlignment(risingNorth(0.25f), 100, 100, kPi / 2, {});
    EXPECT_NEAR(ahead.pitch, -std::atan(0.25f), 1e-5f);
    EXPECT_NEAR(ahead.roll, 0, 1e-5f);
}

TEST(TerrainAlignment, ContactDistancesScaleTheTilt)
{
    // A step 8 high between the contacts tilts less for a long wheelbase than for a short one.
    auto step = [](float x, float) { return x > 100 ? 8.0f : 0.0f; };
    ContactPoints shortBase{ 4, 4 };
    ContactPoints longBase{ 16, 4 };
    const auto s = computeAlignment(step, 100, 100, 0, shortBase);
    const auto l = computeAlignment(step, 100, 100, 0, longBase);
    EXPECT_NEAR(s.pitch, -std::atan(8.0f / 8), 1e-5f);
    EXPECT_NEAR(l.pitch, -std::atan(8.0f / 32), 1e-5f);
    // Zero length contacts do not divide by zero.
    const auto none = computeAlignment(step, 100, 100, 0, { 0, 0 });
    EXPECT_FLOAT_EQ(none.pitch, 0);
    EXPECT_FLOAT_EQ(none.roll, 0);
}

TEST(TerrainAlignment, FeetStayOnTheHighestPoint)
{
    EXPECT_FLOAT_EQ(feetHeight(flat(24), 50, 50, 0.3f, 4), 24);
    // On a slope the upright golfer stands at the highest foot so nothing sinks.
    EXPECT_NEAR(feetHeight(risingEast(1.0f), 100, 100, 0, 4), 102, 1e-4f);
    EXPECT_NEAR(feetHeight(risingEast(1.0f), 100, 100, kPi / 2, 4), 102, 1e-4f);
}

TEST(TerrainAlignment, SmootherSnapsFirstThenEasesTowardsTheTarget)
{
    AlignmentSmoother smoother(0.5f);
    Alignment up;
    up.pitch = 0.4f;
    up.height = 10;
    EXPECT_FLOAT_EQ(smoother.update(up).pitch, 0.4f);

    Alignment flatTarget;
    flatTarget.height = 12;
    auto a = smoother.update(flatTarget);
    EXPECT_FLOAT_EQ(a.pitch, 0.2f);
    EXPECT_FLOAT_EQ(a.height, 12); // height follows at once
    a = smoother.update(flatTarget);
    EXPECT_FLOAT_EQ(a.pitch, 0.1f);
    for (int i = 0; i < 20; i++)
        a = smoother.update(flatTarget);
    EXPECT_NEAR(a.pitch, 0, 1e-5f);
}

TEST(TerrainAlignment, SmootherClampsSteepSurfacesAndResets)
{
    AlignmentSmoother smoother;
    Alignment cliff;
    cliff.pitch = 1.5f;
    cliff.roll = -1.5f;
    const auto a = smoother.update(cliff);
    EXPECT_FLOAT_EQ(a.pitch, AlignmentSmoother::kMaxTilt);
    EXPECT_FLOAT_EQ(a.roll, -AlignmentSmoother::kMaxTilt);

    smoother.reset();
    EXPECT_FLOAT_EQ(smoother.current().pitch, 0);
    Alignment small;
    small.pitch = 0.1f;
    EXPECT_FLOAT_EQ(smoother.update(small).pitch, 0.1f); // snaps again after a reset
}

TEST(TerrainAlignment, AlignmentMatchesTheRasteriserConvention)
{
    // The rasteriser maps the forward axis (1,0,0) to (cos p, 0, -sin p) for pitch p: a climbing vehicle's nose must
    // end up higher than its tail.
    const auto a = computeAlignment(risingEast(0.5f), 0, 0, 0, {});
    EXPECT_GT(-std::sin(a.pitch), 0);
    EXPECT_NEAR(-std::sin(a.pitch) / std::cos(a.pitch), 0.5f, 1e-5f);
}
