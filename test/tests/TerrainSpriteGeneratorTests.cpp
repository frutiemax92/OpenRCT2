/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <algorithm>
#include <array>
#include <cmath>
#include <gtest/gtest.h>
#include <map>
#include <openrct2/drawing/TerrainSpriteGenerator.h>

using namespace OpenRCT2::Drawing;
using namespace OpenRCT2::Terrain;

namespace
{
    TerrainTexture Solid(uint8_t colour, int32_t size = 64)
    {
        return TerrainTexture::Create(size, std::vector<uint8_t>(static_cast<size_t>(size) * size, colour));
    }

    std::vector<TerrainShape> AllShapes()
    {
        std::vector<TerrainShape> v;
        for (uint16_t i = 0; i < kShapeCount; i++)
            v.push_back(GetShapeFromIndex(i));
        return v;
    }

    size_t CountCovered(const TerrainSprite& s)
    {
        return static_cast<size_t>(std::count_if(s.pixels.begin(), s.pixels.end(), [](uint8_t p) { return p != kTerrainTransparent; }));
    }
} // namespace

TEST(TerrainTextureTest, MipsHalveDownToOnePixelOrFourLevels)
{
    std::vector<uint8_t> px(64 * 64);
    for (size_t i = 0; i < px.size(); i++)
        px[i] = static_cast<uint8_t>(1 + (i % 7));
    auto tex = TerrainTexture::Create(64, px);
    ASSERT_EQ(tex.mips.size(), static_cast<size_t>(kTerrainZoomLevels));
    EXPECT_EQ(tex.mips[1].size(), 32u * 32u);
    EXPECT_EQ(tex.mips[2].size(), 16u * 16u);
    EXPECT_EQ(tex.mips[3].size(), 8u * 8u);
    EXPECT_EQ(tex.MipSize(3), 8);

    auto tiny = TerrainTexture::Create(2, { 1, 2, 3, 4 });
    EXPECT_EQ(tiny.mips.size(), 2u);
    EXPECT_EQ(tiny.MipSize(5), 1); // never below one pixel
}

TEST(TerrainTextureTest, MipKeepsTheMajorityColourAndSamplingWraps)
{
    auto tex = TerrainTexture::Create(2, { 5, 5, 5, 9 });
    EXPECT_EQ(tex.Sample(1, 0, 0), 5);
    EXPECT_EQ(tex.Sample(0, 3, 3), 9); // wraps
    EXPECT_EQ(tex.Sample(0, -1, 0), 5);
}

TEST(TerrainShadeTableTest, IdentityAndPaletteTables)
{
    auto id = TerrainShadeTable::Identity();
    for (int b = 0; b < kTerrainShadeBands; b++)
        EXPECT_EQ(id.Shade(b, 77), 77);

    std::array<std::array<uint8_t, 3>, 256> pal{};
    for (int i = 0; i < 256; i++)
        pal[i] = { uint8_t(i), uint8_t(i), uint8_t(i) };
    auto t = TerrainShadeTable::FromPalette(pal);
    EXPECT_EQ(t.Shade(kTerrainNeutralShadeBand, 100), 100);
    EXPECT_LT(t.Shade(0, 100), t.Shade(1, 100));
    EXPECT_LT(t.Shade(1, 100), 100);
    EXPECT_GT(t.Shade(kTerrainShadeBands - 1, 100), 100);
    EXPECT_NE(t.Shade(0, 100), kTerrainTransparent);
}

TEST(TerrainShadeBandTest, FlatIsNeutralAndSlopesFacingTheLightAreBrighter)
{
    EXPECT_EQ(GetTerrainShadeBand(GetSurfaceNormalAt({ 0, 0, 0, 0 }, 5, 5), 0), kTerrainNeutralShadeBand);
    int32_t lighter = 0, darker = 0;
    for (const auto& sh : AllShapes())
    {
        for (int rot = 0; rot < 4; rot++)
        {
            const int band = GetTerrainShadeBand(GetSurfaceNormalAt(sh, 8, 24), rot);
            EXPECT_GE(band, 0);
            EXPECT_LT(band, kTerrainShadeBands);
            lighter += band > kTerrainNeutralShadeBand;
            darker += band < kTerrainNeutralShadeBand;
        }
    }
    EXPECT_GT(lighter, 0);
    EXPECT_GT(darker, 0);
}

TEST(TerrainSpriteTest, FlatTileIsADiamondOfTheExpectedSize)
{
    auto tex = Solid(10);
    const std::array<const TerrainTexture*, 4> quads = { &tex, &tex, &tex, &tex };
    auto s = GenerateTerrainSprite({ 0, 0, 0, 0 }, quads, TerrainShadeTable::Identity(), 0, 0);
    EXPECT_EQ(s.width, 64);
    EXPECT_EQ(s.height, 32);
    EXPECT_EQ(s.xOffset, -32);
    EXPECT_EQ(s.yOffset, -16);
    EXPECT_NEAR(static_cast<double>(CountCovered(s)), 64.0 * 32.0 / 2.0, 64.0);
    for (auto p : s.pixels)
        EXPECT_TRUE(p == 0 || p == 10);
}

TEST(TerrainSpriteTest, QuadrantsUseTheirOwnTexture)
{
    auto ne = Solid(1), se = Solid(2), sw = Solid(3), nw = Solid(4);
    const std::array<const TerrainTexture*, 4> quads = { &ne, &se, &sw, &nw }; // indexed by Quadrant
    auto s = GenerateTerrainSprite({ 0, 0, 0, 0 }, quads, TerrainShadeTable::Identity(), 0, 0);
    auto at = [&](int sx, int sy) { return s.pixels[(sy - s.yOffset) * s.width + (sx - s.xOffset)]; };
    // Rotation 0: local (x, y) maps to screen (y - x, (x + y) / 2 - 16) around the centre.
    // Edge x = 32 (north-east quadrant) is the lower-left side of the diamond, edge y = 0 (south-east) the upper-left.
    EXPECT_EQ(at(-12, 6), 1);  // local (28, 16), near x = 32
    EXPECT_EQ(at(-12, -6), 2); // local (16, 4), near y = 0
    EXPECT_EQ(at(12, -6), 3);  // local (4, 16), near x = 0
    EXPECT_EQ(at(12, 6), 4);   // local (16, 28), near y = 32
}

TEST(TerrainSpriteTest, EveryShapeRotationAndZoomGenerates)
{
    auto tex = Solid(7);
    const std::array<const TerrainTexture*, 4> quads = { &tex, &tex, &tex, &tex };
    const auto shades = TerrainShadeTable::Identity();
    size_t empty = 0;
    for (const auto& sh : AllShapes())
    {
        for (int rot = 0; rot < 4; rot++)
        {
            for (int zoom = 0; zoom < kTerrainZoomLevels; zoom++)
            {
                auto s = GenerateTerrainSprite(sh, quads, shades, rot, zoom);
                ASSERT_EQ(s.pixels.size(), static_cast<size_t>(s.width) * s.height);
                // Only the shade bands of the flat texture can appear.
                for (auto p : s.pixels)
                    ASSERT_EQ(p, p == 0 ? 0 : 7);
                if (CountCovered(s) == 0)
                    empty++;
            }
        }
    }
    // A surface seen exactly edge-on has no pixels; that needs two opposite corners 4 Q apart and the others at 2 Q
    // or similar. They are rare, and never at zoom 0 for flat or gently sloped shapes.
    EXPECT_LT(empty, 369u * 16u / 20u);
}

TEST(TerrainSpriteTest, CoverageScalesWithZoom)
{
    auto tex = Solid(7);
    const std::array<const TerrainTexture*, 4> quads = { &tex, &tex, &tex, &tex };
    const auto shades = TerrainShadeTable::Identity();
    for (const TerrainShape& sh : { TerrainShape{ 0, 0, 0, 0 }, TerrainShape{ 2, 1, 0, 3 }, TerrainShape{ 4, 4, 0, 0 } })
    {
        const double full = static_cast<double>(CountCovered(GenerateTerrainSprite(sh, quads, shades, 0, 0)));
        for (int zoom = 1; zoom < kTerrainZoomLevels; zoom++)
        {
            const double reduced = static_cast<double>(CountCovered(GenerateTerrainSprite(sh, quads, shades, 0, zoom)));
            const double expected = full / (1 << (2 * zoom));
            EXPECT_NEAR(reduced, expected, expected * 0.2 + 8.0) << zoom;
        }
    }
}

namespace
{
    struct Pt
    {
        double x, y;
    };

    void RotateGlobal(int rot, int& x, int& y)
    {
        const int ox = x, oy = y;
        switch (rot & 3)
        {
            case 1: x = oy; y = -ox; break;
            case 2: x = -ox; y = -oy; break;
            case 3: x = -oy; y = ox; break;
            default: break;
        }
    }

    // Barycentric test of a point against a screen triangle; returns the smallest coordinate (negative = outside).
    double MinBarycentric(const std::array<Pt, 3>& t, double px, double py)
    {
        const double d = (t[1].y - t[2].y) * (t[0].x - t[2].x) + (t[2].x - t[1].x) * (t[0].y - t[2].y);
        if (std::abs(d) < 1e-9)
            return -1.0; // edge-on, covers nothing
        const double a = ((t[1].y - t[2].y) * (px - t[2].x) + (t[2].x - t[1].x) * (py - t[2].y)) / d;
        const double b = ((t[2].y - t[0].y) * (px - t[2].x) + (t[0].x - t[2].x) * (py - t[2].y)) / d;
        return std::min({ a, b, 1.0 - a - b });
    }
} // namespace

TEST(TerrainSpriteTest, NeighbouringTilesLeaveNoGaps)
{
    // A 3x3 block of tiles from a random corner height field: neighbours share their edge heights.
    auto tex = Solid(7);
    const std::array<const TerrainTexture*, 4> quads = { &tex, &tex, &tex, &tex };
    const auto shades = TerrainShadeTable::Identity();
    unsigned seed = 99;
    auto next = [&]() { seed = seed * 1103515245u + 12345u; return (seed >> 16) & 0x7FFF; };

    constexpr int kN = 3;
    int checked = 0;
    for (int trial = 0; trial < 80; trial++)
    {
        int cornerQ[kN + 1][kN + 1];
        bool valid = true;
        for (int y = 0; y <= kN; y++)
            for (int x = 0; x <= kN; x++)
            {
                const int ref = x ? cornerQ[y][x - 1] : (y ? cornerQ[y - 1][x] : 8);
                cornerQ[y][x] = std::max(0, ref + static_cast<int>(next() % 9) - 4);
            }
        for (int ty = 0; ty < kN && valid; ty++)
            for (int tx = 0; tx < kN; tx++)
            {
                const int hs[4] = { cornerQ[ty][tx], cornerQ[ty][tx + 1], cornerQ[ty + 1][tx + 1], cornerQ[ty + 1][tx] };
                if (*std::max_element(hs, hs + 4) - *std::min_element(hs, hs + 4) > 4)
                    valid = false;
            }
        if (!valid)
            continue;

        for (int rot = 0; rot < 4; rot++)
        {
            for (int zoom = 0; zoom < kTerrainZoomLevels; zoom++)
            {
                std::map<std::pair<int, int>, int> coverage;
                std::vector<std::array<Pt, 3>> triangles;
                for (int ty = 0; ty < kN; ty++)
                {
                    for (int tx = 0; tx < kN; tx++)
                    {
                        // Local corners: S = (0,0), E = (32,0), N = (32,32), W = (0,32).
                        const int hS = cornerQ[ty][tx], hE = cornerQ[ty][tx + 1], hN = cornerQ[ty + 1][tx + 1],
                                  hW = cornerQ[ty + 1][tx];
                        const int lowest = std::min({ hS, hE, hN, hW });
                        const TerrainShape sh{ uint8_t(hN - lowest), uint8_t(hE - lowest), uint8_t(hS - lowest),
                                               uint8_t(hW - lowest) };
                        auto sprite = GenerateTerrainSprite(sh, quads, shades, rot, zoom);

                        int rx = tx * 32 + 16, ry = ty * 32 + 16;
                        RotateGlobal(rot, rx, ry);
                        const int gx = ry - rx;
                        const int gy = ((rx + ry) >> 1) - lowest * 8;
                        ASSERT_EQ(gx % (1 << zoom), 0);
                        ASSERT_EQ(gy % (1 << zoom), 0);
                        const int px0 = gx / (1 << zoom) + sprite.xOffset, py0 = gy / (1 << zoom) + sprite.yOffset;
                        for (int y = 0; y < sprite.height; y++)
                            for (int x = 0; x < sprite.width; x++)
                                if (sprite.pixels[static_cast<size_t>(y) * sprite.width + x] != kTerrainTransparent)
                                    coverage[{ px0 + x, py0 + y }]++;

                        // Exact triangles of this tile in global screen space.
                        auto project = [&](int lx, int ly, int hq) {
                            int dx = lx - 16, dy = ly - 16;
                            RotateGlobal(rot, dx, dy);
                            return Pt{ double(gx + dy - dx), double(gy + ((dx + dy) >> 1) - (hq - lowest) * 8) };
                        };
                        const Pt pS = project(0, 0, hS), pE = project(32, 0, hE), pN = project(32, 32, hN), pW = project(0, 32, hW);
                        if (GetSplitDiagonal(sh) == SplitDiagonal::northSouth)
                        {
                            triangles.push_back({ pS, pE, pN });
                            triangles.push_back({ pS, pN, pW });
                        }
                        else
                        {
                            triangles.push_back({ pS, pE, pW });
                            triangles.push_back({ pE, pN, pW });
                        }
                    }
                }

                // Pixels strictly inside a triangle must be drawn; pixels strictly outside every triangle must not.
                int minX = INT32_MAX, minY = INT32_MAX, maxX = INT32_MIN, maxY = INT32_MIN;
                for (const auto& [pos, count] : coverage)
                {
                    minX = std::min(minX, pos.first);
                    maxX = std::max(maxX, pos.first);
                    minY = std::min(minY, pos.second);
                    maxY = std::max(maxY, pos.second);
                }
                for (int py = minY - 2; py <= maxY + 2; py++)
                {
                    for (int px = minX - 2; px <= maxX + 2; px++)
                    {
                        const double cx = (px + 0.5) * (1 << zoom), cy = (py + 0.5) * (1 << zoom);
                        double best = -1e9;
                        for (const auto& tri : triangles)
                            best = std::max(best, MinBarycentric(tri, cx, cy));
                        const bool drawn = coverage.count({ px, py }) > 0;
                        if (best > 1e-6)
                        {
                            ASSERT_TRUE(drawn) << "gap at " << px << "," << py << " rot " << rot << " zoom " << zoom;
                        }
                        else if (best < -1e-6)
                        {
                            ASSERT_FALSE(drawn) << "stray pixel at " << px << "," << py << " rot " << rot << " zoom " << zoom;
                        }
                        checked++;
                    }
                }
            }
        }
    }
    EXPECT_GT(checked, 100000);
}

TEST(TerrainSpriteTest, ShadingDarkensSlopesAwayFromTheLight)
{
    auto tex = Solid(100);
    const std::array<const TerrainTexture*, 4> quads = { &tex, &tex, &tex, &tex };
    std::array<std::array<uint8_t, 3>, 256> pal{};
    for (int i = 0; i < 256; i++)
        pal[i] = { uint8_t(i), uint8_t(i), uint8_t(i) };
    const auto shades = TerrainShadeTable::FromPalette(pal);
    auto flat = GenerateTerrainSprite({ 0, 0, 0, 0 }, quads, shades, 0, 0);
    for (auto p : flat.pixels)
        ASSERT_TRUE(p == 0 || p == 100);
    // Rising towards -x and -y vs +x and +y gives different brightness.
    auto a = GenerateTerrainSprite({ 4, 4, 0, 0 }, quads, shades, 0, 0);
    auto b = GenerateTerrainSprite({ 0, 0, 4, 4 }, quads, shades, 0, 0);
    auto first = [](const TerrainSprite& s) { for (auto p : s.pixels) if (p) return p; return uint8_t(0); };
    EXPECT_NE(first(a), first(b));
}

namespace
{
    TerrainTexture WithPriority(uint8_t colour, int32_t priority)
    {
        auto tex = Solid(colour);
        tex.smoothingPriority = priority;
        return tex;
    }

    // Colour at a position inside the tile for rotation 0 (local = centre + inverse projection).
    uint8_t PixelAt(const TerrainSprite& s, int lx, int ly)
    {
        const int dx = lx - 16, dy = ly - 16;
        const int sx = dy - dx, sy = (dx + dy) >> 1;
        const int ox = sx - s.xOffset, oy = sy - s.yOffset;
        return s.pixels[static_cast<size_t>(oy) * s.width + ox];
    }
} // namespace

TEST(TerrainSmoothingTest, HigherPriorityBleedsIntoTheLowerOneInsideATile)
{
    auto low = WithPriority(10, 1);
    auto high = WithPriority(20, 5);
    // South-east quadrant is the high priority one, the others low.
    const std::array<const TerrainTexture*, 4> quads = { &low, &high, &low, &low };
    TerrainSmoothing smoothing;
    smoothing.enabled = true;
    auto s = GenerateTerrainSprite({ 0, 0, 0, 0 }, quads, TerrainShadeTable::Identity(), 0, 0, smoothing);

    int lowNearBoundaryWithHigh = 0, highInLowTerritory = 0, lowInHighTerritory = 0, highFar = 0;
    for (int ly = 1; ly < 31; ly++)
    {
        for (int lx = 1; lx < 31; lx++)
        {
            const auto q = GetQuadrantAt(lx, ly);
            const auto c = PixelAt(s, lx, ly);
            if (c == 0)
                continue;
            if (q == Quadrant::southEast)
                lowInHighTerritory += c == 10;
            else if (c == 20)
            {
                highInLowTerritory++;
                // Only near the south-east quadrant: within the band of its diagonals.
                const bool nearSE = std::abs(lx - ly) * 0.71 < 3.5 || std::abs(lx + ly - 32) * 0.71 < 3.5;
                EXPECT_TRUE(nearSE) << lx << "," << ly;
            }
            if (q == Quadrant::northWest && c == 20)
                highFar++;
        }
    }
    EXPECT_EQ(lowInHighTerritory, 0); // the higher priority surface never gets bled over
    EXPECT_GT(highInLowTerritory, 5);
    EXPECT_EQ(highFar, 0);
    (void)lowNearBoundaryWithHigh;
}

TEST(TerrainSmoothingTest, DisabledOrEqualPriorityChangesNothing)
{
    auto a = WithPriority(10, 3);
    auto b = WithPriority(20, 3);
    const std::array<const TerrainTexture*, 4> quads = { &a, &b, &a, &b };
    const auto shades = TerrainShadeTable::Identity();
    TerrainSmoothing off;
    TerrainSmoothing on;
    on.enabled = true;
    EXPECT_EQ(
        GenerateTerrainSprite({ 1, 2, 0, 1 }, quads, shades, 0, 0, off).pixels,
        GenerateTerrainSprite({ 1, 2, 0, 1 }, quads, shades, 0, 0, on).pixels); // same priority: no bleeding

    auto high = WithPriority(30, 9);
    const std::array<const TerrainTexture*, 4> mixed = { &a, &high, &a, &a };
    EXPECT_NE(
        GenerateTerrainSprite({ 0, 0, 0, 0 }, mixed, shades, 0, 0, off).pixels,
        GenerateTerrainSprite({ 0, 0, 0, 0 }, mixed, shades, 0, 0, on).pixels);
    EXPECT_EQ( // and disabled equals the default
        GenerateTerrainSprite({ 0, 0, 0, 0 }, mixed, shades, 0, 0, off).pixels,
        GenerateTerrainSprite({ 0, 0, 0, 0 }, mixed, shades, 0, 0).pixels);
}

TEST(TerrainSmoothingTest, BleedsAcrossTileEdgesFromTheNeighbour)
{
    auto own = WithPriority(10, 1);
    auto neighbour = WithPriority(20, 5);
    const std::array<const TerrainTexture*, 4> quads = { &own, &own, &own, &own };
    TerrainSmoothing smoothing;
    smoothing.enabled = true;
    smoothing.edgeNeighbours[static_cast<size_t>(Quadrant::northEast)] = &neighbour; // across the edge x = 32
    auto s = GenerateTerrainSprite({ 0, 0, 0, 0 }, quads, TerrainShadeTable::Identity(), 0, 0, smoothing);
    int nearEdge = 0, farFromEdge = 0;
    for (int ly = 1; ly < 31; ly++)
        for (int lx = 1; lx < 31; lx++)
        {
            if (PixelAt(s, lx, ly) != 20)
                continue;
            (lx >= 28 ? nearEdge : farFromEdge)++;
        }
    EXPECT_GT(nearEdge, 3);
    EXPECT_EQ(farFromEdge, 0);
}

TEST(TerrainSmoothingTest, IsDeterministicAndKeepsEveryShapeGapFree)
{
    auto low = WithPriority(10, 1);
    auto high = WithPriority(20, 5);
    const std::array<const TerrainTexture*, 4> quads = { &low, &high, &low, &high };
    TerrainSmoothing smoothing;
    smoothing.enabled = true;
    smoothing.edgeNeighbours = { &high, nullptr, &high, nullptr };
    const auto shades = TerrainShadeTable::Identity();
    for (uint16_t i = 0; i < kShapeCount; i += 7)
    {
        const auto sh = GetShapeFromIndex(i);
        auto plain = GenerateTerrainSprite(sh, quads, shades, 1, 0);
        auto a = GenerateTerrainSprite(sh, quads, shades, 1, 0, smoothing);
        auto b = GenerateTerrainSprite(sh, quads, shades, 1, 0, smoothing);
        EXPECT_EQ(a.pixels, b.pixels);
        // Smoothing only recolours: the covered pixels are the same.
        ASSERT_EQ(a.pixels.size(), plain.pixels.size());
        for (size_t p = 0; p < a.pixels.size(); p++)
            ASSERT_EQ(a.pixels[p] == 0, plain.pixels[p] == 0);
    }
}

namespace
{
    // Texture whose colour tells the row: colour = row + 1.
    TerrainTexture RowTexture()
    {
        std::vector<uint8_t> px(64 * 64);
        for (int y = 0; y < 64; y++)
            for (int x = 0; x < 64; x++)
                px[y * 64 + x] = static_cast<uint8_t>(y + 1);
        return TerrainTexture::Create(64, px);
    }
} // namespace

TEST(TerrainEdgeTest, TwoOfTheFourEdgesFaceTheViewer)
{
    for (int rot = 0; rot < 4; rot++)
    {
        int facing = 0;
        for (uint8_t q = 0; q < 4; q++)
            facing += IsEdgeFacingViewer(static_cast<Quadrant>(q), rot);
        EXPECT_EQ(facing, 2) << rot;
    }
    EXPECT_TRUE(IsEdgeFacingViewer(Quadrant::northEast, 0));
    EXPECT_TRUE(IsEdgeFacingViewer(Quadrant::northWest, 0));
    EXPECT_FALSE(IsEdgeFacingViewer(Quadrant::southEast, 0));
    EXPECT_FALSE(IsEdgeFacingViewer(Quadrant::southWest, 0));
}

TEST(TerrainEdgeTest, FaceAreaFollowsTheHeightDifference)
{
    auto tex = Solid(9);
    const auto shades = TerrainShadeTable::Identity();
    auto count = [&](TerrainEdgeFace f, int zoom = 0) {
        return static_cast<double>(CountCovered(GenerateTerrainEdgeSprite(tex, f, shades, 0, zoom)));
    };
    // Rectangle 32 wide on screen, 16 tall.
    EXPECT_NEAR(count({ Quadrant::northWest, 16, 16, 0, 0, 0 }), 32.0 * 16.0, 40.0);
    // Wedge: half of it.
    EXPECT_NEAR(count({ Quadrant::northWest, 16, 0, 0, 0, 0 }), 32.0 * 16.0 / 2, 40.0);
    // No height, no face.
    EXPECT_EQ(count({ Quadrant::northWest, 8, 8, 8, 8, 0 }), 0.0);
    // Zoomed out sprites cover a quarter per zoom step.
    EXPECT_NEAR(count({ Quadrant::northWest, 16, 16, 0, 0, 0 }, 1), 32.0 * 16.0 / 4, 24.0);
}

TEST(TerrainEdgeTest, RowsLineUpWithAbsoluteHeight)
{
    auto tex = RowTexture();
    const auto shades = TerrainShadeTable::Identity();
    // Edge y = 0 at rotation 0: the point (16, 0) at height z sits at screen (-16, -8 - z).
    for (int base : { 0, 24, 40 })
    {
        auto s = GenerateTerrainEdgeSprite(tex, { Quadrant::southEast, 24, 24, 0, 0, base }, shades, 0, 0);
        for (int z : { 4, 12, 20 })
        {
            const int sx = -16, sy = -8 - z;
            const int ox = sx - s.xOffset, oy = sy - s.yOffset;
            ASSERT_GE(ox, 0);
            ASSERT_LT(ox, s.width);
            ASSERT_GE(oy, 0);
            ASSERT_LT(oy, s.height);
            const int colour = s.pixels[static_cast<size_t>(oy) * s.width + ox];
            const int expectedRow = ((-(base + z)) % 64 + 64) % 64;
            EXPECT_NEAR(colour - 1, expectedRow, 2) << "base " << base << " z " << z;
        }
    }
}

TEST(TerrainEdgeTest, DarkerThanTheFlatSurfaceAndNeverOverEmptyPixels)
{
    std::array<std::array<uint8_t, 3>, 256> pal{};
    for (int i = 0; i < 256; i++)
        pal[i] = { uint8_t(i), uint8_t(i), uint8_t(i) };
    const auto shades = TerrainShadeTable::FromPalette(pal);
    auto tex = Solid(100);
    for (uint8_t q = 0; q < 4; q++)
    {
        auto s = GenerateTerrainEdgeSprite(tex, { static_cast<Quadrant>(q), 20, 12, 0, 4, 0 }, shades, 1, 0);
        ASSERT_EQ(s.pixels.size(), static_cast<size_t>(s.width) * s.height);
        for (auto p : s.pixels)
            ASSERT_TRUE(p == 0 || p < 100);
        EXPECT_GT(CountCovered(s), 0u);
    }
}

TEST(TerrainWaterTest, WaterShowsOnlyWhereTheTerrainIsBelowIt)
{
    auto cover = [](const TerrainShape& sh, int base, WaterPattern p = WaterPattern::mask, int rot = 0, int zoom = 0) {
        return static_cast<double>(CountCovered(GenerateTerrainWaterSprite(sh, base, p, rot, zoom)));
    };
    const double diamond = 64.0 * 32.0 / 2.0;
    EXPECT_NEAR(cover({ 0, 0, 0, 0 }, -8), diamond, 64.0);  // flat, under water
    EXPECT_EQ(cover({ 0, 0, 0, 0 }, 0), 0.0);               // exactly at the water level: dry
    EXPECT_EQ(cover({ 0, 0, 0, 0 }, 8), 0.0);               // above
    // Rising 16 z over the tile along +x (east and north corners up): half of it is under a level 16 above the base.
    EXPECT_NEAR(cover({ 2, 2, 0, 0 }, -16), diamond, 64.0); // the highest point reaches the level: all of it is below
    EXPECT_NEAR(cover({ 4, 4, 0, 0 }, -16), diamond * 0.5, 96.0);
    // A rotation shows the same amount of water.
    for (int rot = 1; rot < 4; rot++)
        EXPECT_NEAR(cover({ 4, 4, 0, 0 }, -16, WaterPattern::mask, rot), diamond * 0.5, 96.0) << rot;
    // Zoomed out: a quarter of the pixels per zoom step.
    EXPECT_NEAR(cover({ 0, 0, 0, 0 }, -8, WaterPattern::mask, 0, 1), diamond / 4, 32.0);
}

TEST(TerrainWaterTest, PatternsUseTheAnimatedWaterPaletteIndices)
{
    const TerrainShape flat{ 0, 0, 0, 0 };
    auto mask = GenerateTerrainWaterSprite(flat, -8, WaterPattern::mask, 0, 0);
    auto sparse = GenerateTerrainWaterSprite(flat, -8, WaterPattern::sparse, 0, 0);
    auto opaque = GenerateTerrainWaterSprite(flat, -8, WaterPattern::opaque, 0, 0);
    for (auto p : mask.pixels)
        ASSERT_TRUE(p == 0 || p == 1);
    for (auto p : sparse.pixels)
        ASSERT_TRUE(p == 0 || (p >= 230 && p <= 239));
    for (auto p : opaque.pixels)
        ASSERT_TRUE(p == 0 || (p >= 230 && p <= 239));
    EXPECT_EQ(CountCovered(opaque), CountCovered(mask)); // opaque water covers the whole mask
    EXPECT_LT(CountCovered(sparse), CountCovered(mask) / 2); // transparent water is mostly clear
    EXPECT_GT(CountCovered(sparse), 0u);
    // Deterministic.
    EXPECT_EQ(sparse.pixels, GenerateTerrainWaterSprite(flat, -8, WaterPattern::sparse, 0, 0).pixels);
}

TEST(TerrainWaterTest, EveryShapeGenerates)
{
    for (uint16_t i = 0; i < kShapeCount; i += 5)
        for (int rot = 0; rot < 4; rot++)
        {
            auto s = GenerateTerrainWaterSprite(GetShapeFromIndex(i), -16, WaterPattern::opaque, rot, 0);
            ASSERT_EQ(s.pixels.size(), static_cast<size_t>(s.width) * s.height);
        }
}

TEST(TerrainOverlayTest, FillMatchesTheSurfaceForEveryShape)
{
    auto tex = Solid(9);
    const std::array<const TerrainTexture*, 4> quads = { &tex, &tex, &tex, &tex };
    const auto shades = TerrainShadeTable::Identity();
    for (uint16_t i = 0; i < kShapeCount; i += 3)
    {
        const auto sh = GetShapeFromIndex(i);
        for (int rot = 0; rot < 4; rot++)
        {
            auto surface = GenerateTerrainSprite(sh, quads, shades, rot, 0);
            auto fill = GenerateTerrainOverlaySprite(sh, { TerrainOverlayKind::fill, 0, 77 }, rot, 0);
            ASSERT_EQ(surface.width, fill.width);
            ASSERT_EQ(surface.height, fill.height);
            ASSERT_EQ(surface.xOffset, fill.xOffset);
            for (size_t p = 0; p < surface.pixels.size(); p++)
            {
                ASSERT_EQ(surface.pixels[p] != 0, fill.pixels[p] != 0);
                ASSERT_TRUE(fill.pixels[p] == 0 || fill.pixels[p] == 77);
            }
        }
    }
}

TEST(TerrainOverlayTest, QuartersTileTheSurfaceExactly)
{
    for (uint16_t i = 0; i < kShapeCount; i += 11)
    {
        const auto sh = GetShapeFromIndex(i);
        auto fill = GenerateTerrainOverlaySprite(sh, { TerrainOverlayKind::fill }, 1, 0);
        std::vector<int> hits(fill.pixels.size(), 0);
        for (uint8_t q = 0; q < 4; q++)
        {
            auto part = GenerateTerrainOverlaySprite(sh, { TerrainOverlayKind::quarter, q }, 1, 0);
            ASSERT_EQ(part.pixels.size(), fill.pixels.size());
            for (size_t p = 0; p < part.pixels.size(); p++)
                hits[p] += part.pixels[p] != 0;
        }
        for (size_t p = 0; p < hits.size(); p++)
        {
            // Every surface pixel is in some quarter; steep shapes can fold onto themselves and count twice.
            ASSERT_EQ(hits[p] > 0, fill.pixels[p] != 0) << i;
            ASSERT_LE(hits[p], 2) << i;
        }
    }
}

TEST(TerrainOverlayTest, OutlineDottedCornerAndEdgeAreSubsetsOfTheFill)
{
    const TerrainShape sh{ 1, 2, 0, 1 };
    auto fill = GenerateTerrainOverlaySprite(sh, { TerrainOverlayKind::fill }, 0, 0);
    auto outline = GenerateTerrainOverlaySprite(sh, { TerrainOverlayKind::outline }, 0, 0);
    auto dotted = GenerateTerrainOverlaySprite(sh, { TerrainOverlayKind::dotted }, 0, 0);
    EXPECT_GT(CountCovered(outline), 40u);
    EXPECT_LT(CountCovered(outline), CountCovered(fill) / 3);
    EXPECT_LT(CountCovered(dotted), CountCovered(outline));
    EXPECT_GT(CountCovered(dotted), CountCovered(outline) / 4);
    auto subset = [&](const TerrainSprite& s) {
        for (size_t p = 0; p < s.pixels.size(); p++)
            if (s.pixels[p] != 0 && fill.pixels[p] == 0)
                return false;
        return true;
    };
    EXPECT_TRUE(subset(outline));
    EXPECT_TRUE(subset(dotted));
    for (uint8_t i = 0; i < 4; i++)
    {
        auto corner = GenerateTerrainOverlaySprite(sh, { TerrainOverlayKind::corner, i }, 0, 0);
        auto edge = GenerateTerrainOverlaySprite(sh, { TerrainOverlayKind::edge, i }, 0, 0);
        EXPECT_TRUE(subset(corner)) << int(i);
        EXPECT_TRUE(subset(edge)) << int(i);
        EXPECT_GT(CountCovered(corner), 15u) << int(i);
        EXPECT_GT(CountCovered(edge), 15u) << int(i);
        EXPECT_LT(CountCovered(corner), CountCovered(fill) / 2) << int(i);
    }
}

TEST(TerrainOverlayTest, CornerHighlightsTheRightCorner)
{
    // Flat tile at rotation 0: the north corner (32, 32) is the bottom of the diamond, east (32, 0) the left one.
    const TerrainShape flat{ 0, 0, 0, 0 };
    auto north = GenerateTerrainOverlaySprite(flat, { TerrainOverlayKind::corner, 0 }, 0, 0);
    auto at = [&](const TerrainSprite& s, int sx, int sy) {
        return s.pixels[static_cast<size_t>(sy - s.yOffset) * s.width + (sx - s.xOffset)] != 0;
    };
    EXPECT_TRUE(at(north, 0, 12));   // near the bottom corner (screen 0, 16 around the centre)
    EXPECT_FALSE(at(north, 0, -12)); // not near the top one
    auto east = GenerateTerrainOverlaySprite(flat, { TerrainOverlayKind::corner, 1 }, 0, 0);
    EXPECT_TRUE(at(east, -24, 0));
    EXPECT_FALSE(at(east, 24, 0));
}
