/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <gtest/gtest.h>
#include <openrct2/entity/Facing.h>

#include <cmath>
#include <set>

using namespace OpenRCT2;

// A slow walker moves one unit a tick, alternating (1, 0) and (1, 1) steps for a heading of about 27 degrees. The heading
// used to be taken from each step, so it flipped between 0 and 45 degrees.
TEST(Facing, ASlowWalkerKeepsOneFineHeadingInsteadOfFlippingBetweenEightDirections)
{
    uint8_t yaw = 0;
    CoordsXY from{ 1000, 1000 };
    CoordsXY at = from;
    std::set<int> yaws;
    for (int i = 0; i < 200; i++)
    {
        at.x += 1;
        if (i % 2 == 0)
            at.y += 1;
        UpdateFacingFromTravel(yaw, from, at);
        if (i > 20)
            yaws.insert(yaw);
    }
    // +x with half a unit of +y per unit is 26.6 degrees; yaw 0 faces -x, so +x is 128 and the heading is 128 + 18.9.
    for (const auto y : yaws)
        EXPECT_NEAR(y, 128 + 19, 6);
}

TEST(Facing, ATurnShowsAnyAngleNotJustMultiplesOf45Degrees)
{
    uint8_t yaw = 0;
    CoordsXY from{ 0, 0 };
    // About 70 degrees from +x.
    UpdateFacingFromTravel(yaw, from, { 10, 27 });
    EXPECT_NE(yaw % 32, 0);
    EXPECT_EQ(from.x, 10);
}

TEST(Facing, StandingStillOrShortStepsLeaveTheHeadingAlone)
{
    uint8_t yaw = 77;
    CoordsXY from{ 50, 50 };
    UpdateFacingFromTravel(yaw, from, { 51, 50 });
    UpdateFacingFromTravel(yaw, from, { 51, 50 });
    EXPECT_EQ(yaw, 77);
    EXPECT_EQ(from.x, 50);
}

// Yaw 0 faces -x and grows counter-clockwise: +x is 128, +y is 192 and -y is 64.
TEST(Facing, YawFacingAMapDirection)
{
    EXPECT_EQ(YawFacing(-1, 0), 0);
    EXPECT_EQ(YawFacing(1, 0), 128);
    EXPECT_EQ(YawFacing(0, 1), 192);
    EXPECT_EQ(YawFacing(0, -1), 64);
}
