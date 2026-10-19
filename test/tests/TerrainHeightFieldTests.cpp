/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <gtest/gtest.h>
#include <openrct2/world/TerrainHeightField.h>

using namespace OpenRCT2::Terrain;
using V = std::pair<int32_t, int32_t>;

namespace
{
    HeightField Flat(int32_t size = 8, int32_t height = 10, int32_t maxQ = 250)
    {
        HeightField f(size, size, maxQ);
        for (int y = 0; y <= size; y++)
            for (int x = 0; x <= size; x++)
                f.Set(x, y, height);
        return f;
    }
}

TEST(TerrainHeightField, RaisingOneVertexByAQuarterMakesQuarterSlopes)
{
    auto f = Flat();
    const V v[] = { { 4, 4 } };
    ASSERT_TRUE(f.Raise(v, 1));
    EXPECT_EQ(f.Get(4, 4), 11);
    EXPECT_EQ(f.GetChangedVertices().size(), 1u);
    // The four tiles around the vertex each have one corner raised by 1 Q.
    EXPECT_EQ(f.GetTile(4, 4).second, (TerrainShape{ 0, 0, 1, 0 })); // vertex (4,4) is the S corner of tile (4,4)
    EXPECT_EQ(f.GetTile(3, 3).second, (TerrainShape{ 1, 0, 0, 0 })); // and the N corner of tile (3,3)
    EXPECT_EQ(f.GetTile(3, 4).second, (TerrainShape{ 0, 1, 0, 0 })); // E corner of (3,4)
    EXPECT_EQ(f.GetTile(4, 3).second, (TerrainShape{ 0, 0, 0, 1 })); // W corner of (4,3)
    EXPECT_TRUE(f.IsConsistent());
}

TEST(TerrainHeightField, RaiseThenLowerRestoresTheField)
{
    auto f = Flat();
    const V v[] = { { 3, 3 } };
    for (int step : { 1, 2, 3, 4 })
    {
        ASSERT_TRUE(f.Raise(v, step));
        ASSERT_TRUE(f.Lower(v, step));
        EXPECT_EQ(f.Get(3, 3), 10) << step;
        EXPECT_TRUE(f.IsConsistent());
    }
}

TEST(TerrainHeightField, OnlyTheLowestSelectedVerticesRise)
{
    auto f = Flat();
    f.Set(2, 2, 12);
    const V v[] = { { 2, 2 }, { 3, 2 }, { 2, 3 }, { 3, 3 } };
    ASSERT_TRUE(f.Raise(v, 2));
    EXPECT_EQ(f.Get(2, 2), 12);
    EXPECT_EQ(f.Get(3, 2), 12);
    EXPECT_EQ(f.Get(2, 3), 12);
    EXPECT_EQ(f.Get(3, 3), 12);
}

TEST(TerrainHeightField, OnlyTheHighestSelectedVerticesFallWhenLowering)
{
    auto f = Flat();
    f.Set(2, 2, 12);
    const V v[] = { { 2, 2 }, { 3, 3 } };
    ASSERT_TRUE(f.Lower(v, 1));
    EXPECT_EQ(f.Get(2, 2), 11);
    EXPECT_EQ(f.Get(3, 3), 10);
}

TEST(TerrainHeightField, SteepPeaksDragNeighboursAlong)
{
    auto f = Flat();
    const V v[] = { { 4, 4 } };
    ASSERT_TRUE(f.Raise(v, 4));
    EXPECT_EQ(f.Get(4, 4), 14);
    EXPECT_TRUE(f.IsConsistent());
    ASSERT_TRUE(f.Raise(v, 4)); // 18: the ring around it must reach 14 -> every tile within 4 Q
    EXPECT_EQ(f.Get(4, 4), 18);
    EXPECT_EQ(f.Get(3, 3), 14);
    EXPECT_EQ(f.Get(5, 5), 14);
    EXPECT_EQ(f.Get(3, 5), 14);
    EXPECT_TRUE(f.IsConsistent());
    EXPECT_GT(f.GetChangedVertices().size(), 1u);
}

TEST(TerrainHeightField, SmoothingPropagatesFarAcrossASteepSlope)
{
    auto f = Flat(16, 10);
    const V v[] = { { 8, 8 } };
    for (int i = 0; i < 6; i++)
        ASSERT_TRUE(f.Raise(v, 4)); // 34 Q peak
    EXPECT_EQ(f.Get(8, 8), 34);
    EXPECT_TRUE(f.IsConsistent());
    // Vertices stay within one tile-step (4 Q diagonal neighbours) of the peak, falling off outwards.
    EXPECT_GE(f.Get(7, 7), 30);
    EXPECT_GE(f.Get(6, 6), 26);
    EXPECT_EQ(f.Get(0, 0), 10);
}

TEST(TerrainHeightField, LoweringFollowsToo)
{
    auto f = Flat(16, 100);
    const V v[] = { { 8, 8 } };
    for (int i = 0; i < 4; i++)
        ASSERT_TRUE(f.Lower(v, 4));
    EXPECT_EQ(f.Get(8, 8), 84);
    EXPECT_TRUE(f.IsConsistent());
    EXPECT_LE(f.Get(7, 7), 88);
}

TEST(TerrainHeightField, LimitsRejectTheWholeChange)
{
    auto f = Flat(8, 10, 12);
    const V v[] = { { 4, 4 } };
    EXPECT_FALSE(f.Raise(v, 3)); // 13 > 12
    EXPECT_EQ(f.Get(4, 4), 10);
    EXPECT_TRUE(f.GetChangedVertices().empty());

    auto low = Flat(8, 1, 12);
    EXPECT_FALSE(low.Lower(v, 2));
    EXPECT_EQ(low.Get(4, 4), 1);

    // A propagated height beyond the limit also rejects, leaving no partial change.
    auto g = Flat(8, 10, 14);
    ASSERT_TRUE(g.Raise(v, 4));
    EXPECT_FALSE(g.Raise(v, 4)); // peak 18 > 14
    EXPECT_EQ(g.Get(4, 4), 14);
    EXPECT_EQ(g.Get(3, 3), 10);
}

TEST(TerrainHeightField, EdgesAndInvalidInput)
{
    auto f = Flat(4);
    const V corner[] = { { 0, 0 } };
    ASSERT_TRUE(f.Raise(corner, 2));
    EXPECT_TRUE(f.IsConsistent());
    const V outside[] = { { -1, 0 }, { 9, 9 } };
    EXPECT_FALSE(f.Raise(outside, 1));
    EXPECT_FALSE(f.Raise(corner, 0));
    EXPECT_FALSE(f.Lower(std::span<const V>{}, 1));
}

TEST(TerrainHeightField, TileRoundTrip)
{
    auto f = Flat(4, 0);
    f.SetTile(1, 1, 20, { 1, 4, 0, 2 });
    const auto [base, shape] = f.GetTile(1, 1);
    EXPECT_EQ(base, 20);
    EXPECT_EQ(shape, (TerrainShape{ 1, 4, 0, 2 }));
}

TEST(TerrainHeightField, SmoothLiftsLowVerticesUntilEveryTileFits)
{
    HeightField f(6, 6, 250);
    for (int y = 0; y <= 6; y++)
        for (int x = 0; x <= 6; x++)
            f.Set(x, y, 10);
    f.Set(3, 3, 40); // a spike
    EXPECT_FALSE(f.IsConsistent());
    f.Smooth();
    EXPECT_TRUE(f.IsConsistent());
    EXPECT_EQ(f.Get(3, 3), 40); // the peak stays, the ground around it rises
    EXPECT_GE(f.Get(2, 2), 36);
}

TEST(TerrainHeightField, VerticesAverageTheTilesAroundThem)
{
    HeightField f(2, 2, 250, 2);
    const int32_t base[4] = { 10, 12, 14, 16 }; // row by row
    f.SetFromTileHeights(base, 4);
    EXPECT_EQ(f.Get(1, 1), 13);                 // the shared corner: average of all four
    EXPECT_EQ(f.Get(0, 0), 10);                 // a map corner touches one tile
    EXPECT_EQ(f.Get(2, 0), 12);
    EXPECT_TRUE(f.IsConsistent());

    // A tight deviation keeps each vertex near the tiles it touches.
    const int32_t gentle[4] = { 10, 11, 12, 11 };
    HeightField tight(2, 2, 250, 2);
    tight.SetFromTileHeights(gentle, 1);
    EXPECT_EQ(tight.Get(1, 1), 11); // the only height within 1 of all four tiles
    EXPECT_TRUE(tight.IsConsistent());

    // When no height fits all the tiles the plain average is used.
    HeightField loose(2, 2, 250, 2);
    loose.SetFromTileHeights(base, 1);
    EXPECT_EQ(loose.Get(1, 1), 13);
}

TEST(TerrainHeightField, SpikyTileHeightsStayConsistentAndRespectLimits)
{
    constexpr int kN = 12;
    std::vector<int32_t> base(kN * kN);
    unsigned seed = 5;
    for (auto& b : base)
    {
        seed = seed * 1103515245u + 12345u;
        b = 2 + ((seed >> 16) % 60);
    }
    HeightField f(kN, kN, 100, 2);
    f.SetFromTileHeights(base, 4);
    EXPECT_TRUE(f.IsConsistent());
    for (int y = 0; y <= kN; y++)
        for (int x = 0; x <= kN; x++)
        {
            EXPECT_GE(f.Get(x, y), 2);
            EXPECT_LE(f.Get(x, y), 100);
        }
}
