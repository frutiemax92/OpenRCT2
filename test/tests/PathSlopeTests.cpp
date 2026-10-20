/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <gtest/gtest.h>
#include <openrct2/world/Map.h>
#include <openrct2/world/tile_element/PathElement.h>

using namespace OpenRCT2;

TEST(PathSlope, DefaultsToLegacyRise)
{
    PathElement el{};
    el.setSlopeDirection(3);
    EXPECT_EQ(el.getSlopeRiseQ(), 2);
    EXPECT_EQ(el.getSlopeRiseZ(), 16);
}

TEST(PathSlope, RiseAndDirectionAreIndependent)
{
    PathElement el{};
    for (uint8_t rise = 1; rise <= 4; rise++)
    {
        for (Direction dir = 0; dir < 4; dir++)
        {
            el.setSlopeRiseQ(rise);
            el.setSlopeDirection(dir);
            EXPECT_EQ(el.getSlopeRiseQ(), rise);
            EXPECT_EQ(el.getSlopeDirection(), dir);
        }
    }
}

TEST(PathSlope, RiseIsClamped)
{
    PathElement el{};
    el.setSlopeRiseQ(9);
    EXPECT_EQ(el.getSlopeRiseQ(), 4);
    el.setSlopeRiseQ(0);
    EXPECT_EQ(el.getSlopeRiseQ(), 1);
}

TEST(PathSlope, WalkingHeightScalesWithTheRise)
{
    // Slope direction 2 rises towards +x: the far edge of the tile is the full rise.
    EXPECT_EQ(MapHeightFromSlope({ 0, 0 }, 2, true, 32), 0);
    EXPECT_EQ(MapHeightFromSlope({ 31, 0 }, 2, true, 32), 31);
    EXPECT_EQ(MapHeightFromSlope({ 31, 0 }, 2, true, 8), 7);
    EXPECT_EQ(MapHeightFromSlope({ 16, 0 }, 2, true), 8);
    EXPECT_EQ(MapHeightFromSlope({ 31, 0 }, 2, false, 32), 0);
}
