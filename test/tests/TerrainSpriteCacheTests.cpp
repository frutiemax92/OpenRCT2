/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *
 *****************************************************************************/

#include <gtest/gtest.h>
#include <openrct2/drawing/TerrainSpriteCache.h>

using namespace OpenRCT2::Drawing;

namespace
{
    TerrainSpriteKey Key(uint16_t shape, uint8_t zoom = 0)
    {
        return { shape, { 1, 2, 3, 4 }, 0, zoom };
    }

    TerrainSprite Sprite(size_t bytes)
    {
        TerrainSprite s;
        s.width = static_cast<int32_t>(bytes);
        s.height = 1;
        s.pixels.assign(bytes, 1);
        return s;
    }
} // namespace

TEST(TerrainSpriteCacheTest, GeneratesOnceThenHits)
{
    TerrainSpriteCache cache(1 << 20, 100);
    int generated = 0;
    auto gen = [&] { generated++; return Sprite(100); };
    const auto* a = cache.Get(Key(1), gen);
    ASSERT_NE(a, nullptr);
    const auto* b = cache.Get(Key(1), gen);
    EXPECT_EQ(a, b);
    EXPECT_EQ(generated, 1);
    EXPECT_EQ(cache.GetCount(), 1u);
}

TEST(TerrainSpriteCacheTest, KeyFieldsAllMatter)
{
    TerrainSpriteCache cache(1 << 20, 100);
    int generated = 0;
    auto gen = [&] { generated++; return Sprite(10); };
    cache.Get(Key(1), gen);
    cache.Get(Key(2), gen);
    cache.Get(Key(1, 1), gen);
    auto other = Key(1);
    other.quadrantSurfaces[3] = 9;
    cache.Get(other, gen);
    other = Key(1);
    other.rotation = 2;
    cache.Get(other, gen);
    EXPECT_EQ(generated, 5);
}

TEST(TerrainSpriteCacheTest, EvictsLeastRecentlyUsedWhenOverBudget)
{
    const size_t entrySize = sizeof(TerrainSprite) + 1000; // approximate; budget fits two entries, not three
    TerrainSpriteCache cache(entrySize * 2 + 400, 100);
    auto gen = [] { return Sprite(1000); };
    cache.Get(Key(1), gen);
    cache.Get(Key(2), gen);
    cache.Peek(Key(1)); // 1 is now more recent than 2
    cache.Get(Key(3), gen);
    EXPECT_NE(cache.Peek(Key(1)), nullptr);
    EXPECT_EQ(cache.Peek(Key(2)), nullptr);
    EXPECT_NE(cache.Peek(Key(3)), nullptr);
    EXPECT_LE(cache.GetBytesUsed(), entrySize * 2 + 400);
}

TEST(TerrainSpriteCacheTest, FrameLimitReturnsNullInsteadOfGenerating)
{
    TerrainSpriteCache cache(1 << 20, 2);
    int generated = 0;
    auto gen = [&] { generated++; return Sprite(10); };
    EXPECT_NE(cache.Get(Key(1), gen), nullptr);
    EXPECT_NE(cache.Get(Key(2), gen), nullptr);
    EXPECT_EQ(cache.Get(Key(3), gen), nullptr); // limit reached, caller draws the fallback
    EXPECT_EQ(generated, 2);
    EXPECT_NE(cache.Get(Key(1), gen), nullptr); // hits are never limited
    cache.BeginFrame();
    EXPECT_NE(cache.Get(Key(3), gen), nullptr);
    EXPECT_EQ(generated, 3);
}

TEST(TerrainSpriteCacheTest, SpriteLargerThanTheBudgetIsNotKept)
{
    TerrainSpriteCache cache(100, 10);
    EXPECT_EQ(cache.Get(Key(1), [] { return Sprite(5000); }), nullptr);
    EXPECT_EQ(cache.GetCount(), 0u);
    EXPECT_EQ(cache.GetBytesUsed(), 0u);
}

TEST(TerrainSpriteCacheTest, ShrinkingTheBudgetEvictsAndClearEmpties)
{
    TerrainSpriteCache cache(1 << 20, 100);
    for (uint16_t i = 0; i < 10; i++)
        cache.Get(Key(i), [] { return Sprite(1000); });
    EXPECT_EQ(cache.GetCount(), 10u);
    cache.SetBudget(3 * (sizeof(TerrainSprite) + 1000 + 64));
    EXPECT_LT(cache.GetCount(), 10u);
    EXPECT_NE(cache.Peek(Key(9)), nullptr); // most recent survives
    cache.Clear();
    EXPECT_EQ(cache.GetCount(), 0u);
    EXPECT_EQ(cache.GetBytesUsed(), 0u);
}
