/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <gtest/gtest.h>
#include <openrct2/world/TerrainShape.h>
#include <set>

using namespace OpenRCT2::Terrain;

namespace
{
    std::vector<TerrainShape> AllShapes()
    {
        std::vector<TerrainShape> v;
        for (int n = 0; n <= 4; n++)
            for (int e = 0; e <= 4; e++)
                for (int s = 0; s <= 4; s++)
                    for (int w = 0; w <= 4; w++)
                    {
                        TerrainShape sh{ uint8_t(n), uint8_t(e), uint8_t(s), uint8_t(w) };
                        if (IsValidShape(sh))
                            v.push_back(sh);
                    }
        return v;
    }
} // namespace

TEST(TerrainShape, ThereAre369NormalisedShapes)
{
    EXPECT_EQ(AllShapes().size(), 369u);
    EXPECT_EQ(kShapeCount, 369u);
}

TEST(TerrainShape, IndexTableIsABijection)
{
    std::set<uint16_t> seen;
    for (const auto& sh : AllShapes())
    {
        const auto idx = GetShapeIndex(sh);
        ASSERT_LT(idx, kShapeCount);
        EXPECT_TRUE(seen.insert(idx).second);
        EXPECT_EQ(GetShapeFromIndex(idx), sh);
    }
    EXPECT_EQ(seen.size(), 369u);
    EXPECT_EQ(GetShapeIndex({ 0, 0, 0, 0 }), 0u);
}

TEST(TerrainShape, InvalidShapesAreRejected)
{
    EXPECT_FALSE(IsValidShape({ 1, 1, 1, 1 })); // not normalised
    EXPECT_FALSE(IsValidShape({ 5, 0, 0, 0 })); // out of range
    EXPECT_EQ(GetShapeIndex({ 1, 1, 1, 1 }), 0xFFFF);
}

TEST(TerrainShape, EncodeDecodeRoundTrip)
{
    for (const auto& sh : AllShapes())
        EXPECT_EQ(DecodeShape(EncodeShape(sh)), sh);
}

TEST(TerrainShape, MakeShapeNormalises)
{
    int32_t base = 0;
    TerrainShape sh{};
    ASSERT_TRUE(MakeShape({ 40, 48, 32, 56 }, base, sh));
    EXPECT_EQ(base, 32);
    EXPECT_EQ(sh, (TerrainShape{ 1, 2, 0, 3 }));
    EXPECT_FALSE(MakeShape({ 0, 4, 0, 0 }, base, sh));  // not a multiple of Q
    EXPECT_FALSE(MakeShape({ 0, 40, 0, 0 }, base, sh)); // more than 4 Q
    const auto corners = GetCornerHeights(base, sh);
    EXPECT_EQ(corners.north, 40);
    EXPECT_EQ(corners.east, 48);
    EXPECT_EQ(corners.south, 32);
    EXPECT_EQ(corners.west, 56);
}

TEST(TerrainShape, SplitDiagonalRule)
{
    EXPECT_EQ(GetSplitDiagonal({ 0, 0, 0, 0 }), SplitDiagonal::northSouth); // tie
    EXPECT_EQ(GetSplitDiagonal({ 2, 0, 2, 0 }), SplitDiagonal::northSouth);
    EXPECT_EQ(GetSplitDiagonal({ 0, 3, 0, 1 }), SplitDiagonal::eastWest);
    EXPECT_EQ(GetSplitDiagonal({ 1, 0, 0, 0 }), SplitDiagonal::northSouth);
    EXPECT_EQ(GetSplitDiagonal({ 0, 0, 0, 1 }), SplitDiagonal::eastWest);
}

TEST(TerrainShape, HeightAtCornersIsExact)
{
    for (const auto& sh : AllShapes())
    {
        EXPECT_EQ(GetSurfaceZAt(16, sh, 0, 0), 16 + sh.south * 8);
        // The far corners are one unit short of 32: at most within 2 z units of the corner height.
        EXPECT_NEAR(GetSurfaceZAt(16, sh, 31, 0), 16 + sh.east * 8, 1);
        EXPECT_NEAR(GetSurfaceZAt(16, sh, 0, 31), 16 + sh.west * 8, 1);
        EXPECT_NEAR(GetSurfaceZAt(16, sh, 31, 31), 16 + sh.north * 8, 2);
    }
}

TEST(TerrainShape, FlatAndPlanarShapes)
{
    EXPECT_EQ(GetSurfaceZAt(24, { 0, 0, 0, 0 }, 13, 7), 24);
    // Slope rising towards +x over the whole tile (E and N corners up): z = 8 * 4 * x / 32 at the middle of a row.
    const TerrainShape ramp{ 4, 4, 0, 0 };
    EXPECT_EQ(GetSurfaceZAt(0, ramp, 0, 10), 0);
    EXPECT_EQ(GetSurfaceZAt(0, ramp, 16, 10), 16);
    EXPECT_EQ(GetSurfaceZAt(0, ramp, 8, 3), 8);
}

TEST(TerrainShape, HeightIsContinuousAcrossTheSplit)
{
    // Neighbouring points never differ by more than the steepest possible gradient (4 Q over 32 units => 1 z per unit).
    for (const auto& sh : AllShapes())
    {
        for (int y = 0; y < 31; y++)
            for (int x = 0; x < 31; x++)
            {
                const int z = GetSurfaceZAt(0, sh, x, y);
                EXPECT_LE(std::abs(GetSurfaceZAt(0, sh, x + 1, y) - z), 2);
                EXPECT_LE(std::abs(GetSurfaceZAt(0, sh, x, y + 1) - z), 2);
            }
    }
}

TEST(TerrainShape, HeightStaysBetweenCornerExtremes)
{
    for (const auto& sh : AllShapes())
    {
        const int hi = std::max({ sh.north, sh.east, sh.south, sh.west }) * 8;
        for (int y = 0; y < 32; y++)
            for (int x = 0; x < 32; x++)
            {
                const int z = GetSurfaceZAt(0, sh, x, y);
                EXPECT_GE(z, 0);
                EXPECT_LE(z, hi);
            }
    }
}

TEST(TerrainShape, SurfaceZMatchesTriangleSplitForRandomPoints)
{
    // The height must lie on the plane of the triangle reported by GetTriangleAt (rendering and physics share it).
    unsigned seed = 12345;
    auto next = [&]() { seed = seed * 1103515245u + 12345u; return (seed >> 16) & 0x7FFF; };
    for (const auto& sh : AllShapes())
    {
        for (int i = 0; i < 40; i++)
        {
            const int x = next() % 32, y = next() % 32;
            const int tri = GetTriangleAt(sh, x, y);
            const int hN = sh.north * 8, hE = sh.east * 8, hS = sh.south * 8, hW = sh.west * 8;
            int expected32;
            if (GetSplitDiagonal(sh) == SplitDiagonal::northSouth)
                expected32 = tri == 0 ? (32 - x) * hS + (x - y) * hE + y * hN : (32 - y) * hS + x * hN + (y - x) * hW;
            else
                expected32 = tri == 0 ? (32 - x - y) * hS + x * hE + y * hW : (x + y - 32) * hN + (32 - y) * hE + (32 - x) * hW;
            EXPECT_EQ(GetSurfaceZAt(0, sh, x, y), expected32 / 32);
        }
    }
}

TEST(TerrainShape, QuadrantsMatchTheirEdges)
{
    EXPECT_EQ(GetQuadrantAt(31, 16), Quadrant::northEast); // near x = 32
    EXPECT_EQ(GetQuadrantAt(16, 0), Quadrant::southEast);  // near y = 0
    EXPECT_EQ(GetQuadrantAt(0, 16), Quadrant::southWest);  // near x = 0
    EXPECT_EQ(GetQuadrantAt(16, 31), Quadrant::northWest); // near y = 32
}

TEST(TerrainShape, NormalIsUnitAndFlatPointsUp)
{
    const auto up = GetSurfaceNormalAt({ 0, 0, 0, 0 }, 5, 5);
    EXPECT_EQ(up.x, 0);
    EXPECT_EQ(up.y, 0);
    EXPECT_EQ(up.z, kFixedOne);
    for (const auto& sh : AllShapes())
    {
        const auto n = GetSurfaceNormalAt(sh, 10, 20);
        const double len = std::sqrt(double(n.x) * n.x + double(n.y) * n.y + double(n.z) * n.z) / kFixedOne;
        EXPECT_NEAR(len, 1.0, 0.001);
        EXPECT_GT(n.z, 0);
    }
}

TEST(TerrainShape, NormalTiltsAwayFromTheRise)
{
    // Rising towards +x: the normal leans towards -x.
    const auto n = GetSurfaceNormalAt({ 4, 4, 0, 0 }, 16, 10);
    EXPECT_LT(n.x, 0);
    EXPECT_EQ(n.y, 0);
}
