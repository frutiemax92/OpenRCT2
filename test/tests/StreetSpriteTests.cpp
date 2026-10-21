/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <algorithm>
#include <gtest/gtest.h>
#include <openrct2/drawing/TerrainSpriteGenerator.h>
#include <openrct2/object/StreetObject.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Drawing;
using OpenRCT2::Terrain::TerrainShape;

namespace
{
    constexpr uint8_t kRoad = 50;
    constexpr uint8_t kCentre = 70;
    constexpr uint8_t kEdge = 71;
    constexpr uint8_t kArrow = 72;

    struct Textures
    {
        TerrainTexture road;
        TerrainTexture markings;
        std::array<bool, 256> isMarking{};
    };

    // A 64 x 64 mask like the placeholder streets: edge lines near the verges, then either a dashed centre line or an arrow
    // along the middle.
    Textures makeTextures(bool oneWay)
    {
        Textures t;
        t.road = TerrainTexture::Create(64, std::vector<uint8_t>(64 * 64, kRoad));
        std::vector<uint8_t> mask(64 * 64, 0);
        for (int x = 0; x < 64; x++)
        {
            for (int row : { 3, 4, 5, 6, 57, 58, 59, 60 })
                mask[row * 64 + x] = kEdge;
            for (int row = 29; row < 33; row++)
            {
                if (!oneWay && (x / 8) % 2 == 0)
                    mask[row * 64 + x] = kCentre;
                if (oneWay && x >= 22 && x < 40)
                    mask[row * 64 + x] = kArrow;
            }
        }
        t.markings = TerrainTexture::Create(64, mask);
        t.isMarking[kCentre] = t.isMarking[kEdge] = t.isMarking[kArrow] = true;
        return t;
    }

    TerrainSprite street(
        const TerrainShape& shape, const Textures& t, uint8_t connections, bool oneWay = false, int flow = 0, int rotation = 0)
    {
        return GenerateTerrainStreetSprite(
            shape, t.road, t.markings, t.isMarking, connections, oneWay, flow, kStreetRoadWidth, TerrainShadeTable::Identity(),
            rotation, 0);
    }

    size_t covered(const TerrainSprite& sprite)
    {
        return static_cast<size_t>(std::count_if(sprite.pixels.begin(), sprite.pixels.end(), [](uint8_t p) { return p != 0; }));
    }

    size_t count(const TerrainSprite& sprite, uint8_t value)
    {
        return static_cast<size_t>(std::count(sprite.pixels.begin(), sprite.pixels.end(), value));
    }
} // namespace

TEST(StreetSprite, CoverageGrowsWithTheConnections)
{
    const TerrainShape flat{ 0, 0, 0, 0 };
    const auto textures = makeTextures(false);
    const auto isolated = covered(street(flat, textures, 0));
    const auto deadEnd = covered(street(flat, textures, 0b0001));
    const auto straight = covered(street(flat, textures, 0b0101));
    const auto corner = covered(street(flat, textures, 0b0011));
    const auto tee = covered(street(flat, textures, 0b0111));
    const auto cross = covered(street(flat, textures, 0b1111));
    EXPECT_GT(isolated, 0u);
    EXPECT_GT(deadEnd, isolated);
    EXPECT_GT(straight, deadEnd);
    EXPECT_GT(corner, deadEnd);
    EXPECT_GT(tee, straight);
    EXPECT_GT(cross, tee);
}

TEST(StreetSprite, RotatingTheViewKeepsTheSameRoadArea)
{
    const TerrainShape flat{ 0, 0, 0, 0 };
    const auto textures = makeTextures(false);
    const auto base = covered(street(flat, textures, 0b0101, false, 0, 0));
    for (int rotation = 1; rotation < 4; rotation++)
    {
        const auto turned = covered(street(flat, textures, 0b0101, false, 0, rotation));
        // The diamond is rasterised on a grid, so allow a pixel row or two of difference.
        EXPECT_NEAR(static_cast<double>(turned), static_cast<double>(base), base * 0.05) << "rotation " << rotation;
    }
}

TEST(StreetSprite, TwoWayRoadsHaveACentreLineAndEdgeLinesButNoArrows)
{
    const TerrainShape flat{ 0, 0, 0, 0 };
    const auto textures = makeTextures(false);
    const auto sprite = street(flat, textures, 0b0101);
    EXPECT_GT(count(sprite, kCentre), 0u);
    EXPECT_GT(count(sprite, kEdge), 0u);
    EXPECT_EQ(count(sprite, kArrow), 0u);
    // Most of the road is plain asphalt.
    EXPECT_GT(count(sprite, kRoad), count(sprite, kCentre) + count(sprite, kEdge));
}

TEST(StreetSprite, OneWayRoadsHaveArrowsAndNoCentreLine)
{
    const TerrainShape flat{ 0, 0, 0, 0 };
    const auto textures = makeTextures(true);
    const auto sprite = street(flat, textures, 0b0101, true, 2);
    EXPECT_GT(count(sprite, kArrow), 0u);
    EXPECT_EQ(count(sprite, kCentre), 0u);
}

TEST(StreetSprite, JunctionsAndCornersHaveNoMarkingsInTheirMiddle)
{
    const TerrainShape flat{ 0, 0, 0, 0 };
    const auto textures = makeTextures(false);
    // A cross has edge lines and centre dashes on the arms, but the markings stop at the middle square, so there is less
    // marking than four times a quarter of a straight road would give.
    const auto straight = street(flat, textures, 0b0101);
    const auto cross = street(flat, textures, 0b1111);
    EXPECT_GT(count(cross, kCentre), 0u);
    EXPECT_GT(covered(cross), covered(straight));
    EXPECT_LT(count(cross, kEdge), 2 * count(straight, kEdge));
}

TEST(StreetSprite, LiesOnRampsAtAnyRotation)
{
    const auto textures = makeTextures(false);
    const TerrainShape ramp{ 2, 2, 0, 0 };
    const TerrainShape steep{ 4, 4, 0, 0 };
    for (int rotation = 0; rotation < 4; rotation++)
    {
        for (const auto& shape : { ramp, steep })
        {
            // A ramp of four quarter steps rises exactly as the view looks down it, so seen from two sides the surface is
            // edge on and has no area to draw.
            if (shape.north == 4 && (rotation == 0 || rotation == 3))
                continue;
            const auto sprite = street(shape, textures, 0b0101, false, 0, rotation);
            EXPECT_GT(sprite.width, 0);
            EXPECT_GT(sprite.height, 0);
            EXPECT_GT(covered(sprite), 0u) << "rotation " << rotation << " rise " << int(shape.north);
        }
    }
}

TEST(StreetSprite, TheFlowOnlyChangesAOneWayRoad)
{
    const TerrainShape flat{ 0, 0, 0, 0 };
    const auto oneWay = makeTextures(true);
    const auto east = street(flat, oneWay, 0b0101, true, 2);
    const auto west = street(flat, oneWay, 0b0101, true, 0);
    // The arrow points the other way: the same pixels in a different place.
    EXPECT_NEAR(static_cast<double>(count(east, kArrow)), static_cast<double>(count(west, kArrow)), 3.0);
    EXPECT_NE(east.pixels, west.pixels);

    const auto twoWay = makeTextures(false);
    EXPECT_EQ(street(flat, twoWay, 0b0101, false, 2).pixels, street(flat, twoWay, 0b0101, false, 0).pixels);
}
