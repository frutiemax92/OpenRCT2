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
