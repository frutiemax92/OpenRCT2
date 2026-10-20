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
#include <openrct2/model/ImposterCache.h>

using namespace OpenRCT2::Model;

namespace
{
    PaletteImage imageOfSize(size_t bytes)
    {
        PaletteImage image;
        image.width = static_cast<int32_t>(bytes);
        image.height = 1;
        image.pixels.assign(bytes, 1);
        return image;
    }

    ImposterKey keyWithYaw(uint8_t yaw)
    {
        ImposterKey key;
        key.yawBucket = yaw;
        return key;
    }

    GlbModel modelWithClip(float duration)
    {
        GlbModel model;
        Clip clip;
        clip.duration = duration;
        model.clips.push_back(clip);
        return model;
    }

    // One horizontal square of a remappable material.
    GlbModel remapSquare()
    {
        GlbModel model;
        Node root;
        root.mesh = 0;
        model.nodes.push_back(root);
        model.rootNodes.push_back(0);
        model.meshPrimitives.push_back({ 0 });
        Material material;
        material.name = kRemapMaterialNames[0];
        model.materials.push_back(material);
        Primitive prim;
        prim.material = 0;
        auto vertex = [](float x, float z) {
            Vertex v;
            v.position = { x, 0, z };
            v.normal = { 0, 1, 0 };
            return v;
        };
        prim.vertices = { vertex(0, 0), vertex(0, 1), vertex(1, 1), vertex(1, 0) };
        prim.indices = { 0, 1, 3, 1, 2, 3 };
        model.primitives.push_back(std::move(prim));
        return model;
    }
} // namespace

TEST(ImposterCache, YawHas32BucketsThatWrap)
{
    EXPECT_EQ(yawToBucket(0), 0);
    EXPECT_EQ(yawToBucket(6.28318530718f / 4), 8);
    EXPECT_EQ(yawToBucket(6.28318530718f), 0);
    EXPECT_EQ(yawToBucket(-6.28318530718f / 32), 31);
    // Both sides of a bucket boundary land in different buckets, values inside one bucket share it.
    EXPECT_EQ(yawToBucket(0.05f), yawToBucket(-0.05f));
    EXPECT_NE(yawToBucket(0.3f), yawToBucket(0));
    for (int32_t b = 0; b < kYawBuckets; b++)
        EXPECT_EQ(yawToBucket(bucketToYaw(static_cast<uint8_t>(b))), b);
}

TEST(ImposterCache, TiltBucketsAreFiveDegreesAndUprightIsExact)
{
    constexpr float deg = 3.14159265f / 180.0f;
    EXPECT_FLOAT_EQ(bucketToTilt(tiltToBucket(0)), 0.0f);
    EXPECT_EQ(tiltToBucket(0), tiltToBucket(2 * deg));
    EXPECT_EQ(tiltToBucket(5 * deg), tiltToBucket(0) + 1);
    EXPECT_EQ(tiltToBucket(-5 * deg), tiltToBucket(0) - 1);
    // Clamped to the outermost of the 8 buckets.
    EXPECT_EQ(tiltToBucket(90 * deg), kTiltBuckets - 1);
    EXPECT_EQ(tiltToBucket(-90 * deg), 0);
    EXPECT_NEAR(bucketToTilt(tiltToBucket(10 * deg)), 10 * deg, 1e-4f);
}

TEST(ImposterCache, AnimationFramesRunAtTwelveFpsAndLoop)
{
    const auto model = modelWithClip(1.0f);
    EXPECT_EQ(animationFrame(model, 0, 0.0f), 0);
    EXPECT_EQ(animationFrame(model, 0, 0.09f), 1);
    EXPECT_EQ(animationFrame(model, 0, 0.5f), 6);
    EXPECT_EQ(animationFrame(model, 0, 1.0f), 0); // loops after 12 frames
    EXPECT_EQ(animationFrame(model, 0, 1.2f), 2);
    EXPECT_EQ(animationFrame(model, 0, 0.5f, 24.0f), 12);
    EXPECT_EQ(animationFrame(model, -1, 5.0f), 0); // bind pose
    EXPECT_EQ(animationFrame(model, 3, 5.0f), 0);  // unknown clip
    EXPECT_EQ(animationFrame(model, 0, -0.1f), 10); // negative times wrap too
}

TEST(ImposterCache, KeysQuantisePosesSoNearbyPosesShareAnImage)
{
    const auto model = modelWithClip(1.0f);
    Orientation a;
    a.yaw = 1.0f;
    Orientation b;
    b.yaw = 1.0f + 0.02f;
    const std::array<uint8_t, kRemapSlots> remap{ 1, 2, 3 };
    const auto keyA = makeImposterKey(7, model, 0, 0.51f, a, 0, 0, remap);
    const auto keyB = makeImposterKey(7, model, 0, 0.53f, b, 0, 0, remap);
    EXPECT_EQ(keyA, keyB);
    EXPECT_EQ(ImposterKeyHash{}(keyA), ImposterKeyHash{}(keyB));

    // Every part of the key matters.
    auto differs = [&](auto change) {
        auto other = keyA;
        change(other);
        return !(other == keyA);
    };
    EXPECT_TRUE(differs([](ImposterKey& k) { k.modelId++; }));
    EXPECT_TRUE(differs([](ImposterKey& k) { k.clip = -1; }));
    EXPECT_TRUE(differs([](ImposterKey& k) { k.frame++; }));
    EXPECT_TRUE(differs([](ImposterKey& k) { k.yawBucket++; }));
    EXPECT_TRUE(differs([](ImposterKey& k) { k.pitchBucket++; }));
    EXPECT_TRUE(differs([](ImposterKey& k) { k.rollBucket++; }));
    EXPECT_TRUE(differs([](ImposterKey& k) { k.rotation++; }));
    EXPECT_TRUE(differs([](ImposterKey& k) { k.zoom++; }));
    EXPECT_TRUE(differs([](ImposterKey& k) { k.remap[2]++; }));

    // A missing clip falls back to the bind pose, frame 0.
    const auto bind = makeImposterKey(7, model, 4, 3.0f, a, 0, 0, remap);
    EXPECT_EQ(bind.clip, -1);
    EXPECT_EQ(bind.frame, 0);
}

TEST(ImposterCache, FindsInsertedImagesAndCountsHits)
{
    ImposterCache cache;
    EXPECT_EQ(cache.find(keyWithYaw(1)), nullptr);
    cache.insert(keyWithYaw(1), imageOfSize(10));
    const auto found = cache.find(keyWithYaw(1));
    ASSERT_NE(found, nullptr);
    EXPECT_EQ(found->pixels.size(), 10u);
    EXPECT_EQ(cache.hits(), 1u);
    EXPECT_EQ(cache.misses(), 1u);
    EXPECT_EQ(cache.count(), 1u);
    EXPECT_EQ(cache.sizeBytes(), 10u);
}

TEST(ImposterCache, GetOrCreateRastersOnlyOnce)
{
    ImposterCache cache;
    int32_t calls = 0;
    auto factory = [&] {
        calls++;
        return imageOfSize(4);
    };
    const auto first = cache.getOrCreate(keyWithYaw(3), factory);
    const auto second = cache.getOrCreate(keyWithYaw(3), factory);
    EXPECT_EQ(calls, 1);
    EXPECT_EQ(first, second);
    cache.getOrCreate(keyWithYaw(4), factory);
    EXPECT_EQ(calls, 2);
}

TEST(ImposterCache, ReplacingAKeyKeepsTheByteCountRight)
{
    ImposterCache cache;
    cache.insert(keyWithYaw(1), imageOfSize(10));
    cache.insert(keyWithYaw(1), imageOfSize(3));
    EXPECT_EQ(cache.count(), 1u);
    EXPECT_EQ(cache.sizeBytes(), 3u);
}

TEST(ImposterCache, EvictsLeastRecentlyUsedWhenOverBudget)
{
    ImposterCache cache(25);
    cache.insert(keyWithYaw(1), imageOfSize(10));
    cache.insert(keyWithYaw(2), imageOfSize(10));
    cache.find(keyWithYaw(1)); // 2 is now the oldest
    cache.insert(keyWithYaw(3), imageOfSize(10));
    EXPECT_LE(cache.sizeBytes(), 25u);
    EXPECT_NE(cache.find(keyWithYaw(1)), nullptr);
    EXPECT_NE(cache.find(keyWithYaw(3)), nullptr);
    EXPECT_EQ(cache.find(keyWithYaw(2)), nullptr);
}

TEST(ImposterCache, KeepsTheNewestImageEvenWhenItExceedsTheBudget)
{
    ImposterCache cache(5);
    cache.insert(keyWithYaw(1), imageOfSize(3));
    const auto big = cache.insert(keyWithYaw(2), imageOfSize(50));
    EXPECT_EQ(cache.count(), 1u);
    EXPECT_EQ(cache.find(keyWithYaw(2)), big);
    EXPECT_EQ(cache.find(keyWithYaw(1)), nullptr);
}

TEST(ImposterCache, EvictedImagesStayValidForHoldersAndBudgetCanShrink)
{
    ImposterCache cache(100);
    const auto held = cache.insert(keyWithYaw(1), imageOfSize(40));
    cache.insert(keyWithYaw(2), imageOfSize(40));
    cache.setBudget(40);
    EXPECT_EQ(cache.count(), 1u);
    EXPECT_EQ(held->pixels.size(), 40u); // still readable
    cache.clear();
    EXPECT_EQ(cache.count(), 0u);
    EXPECT_EQ(cache.sizeBytes(), 0u);
}

TEST(ImposterCache, RendersTheBucketCentreAndRemapsMaterials)
{
    const auto model = remapSquare();
    const std::pair<uint8_t, PaletteColour> palette[] = {
        { 10, { 255, 0, 0 } },
        { 20, { 0, 0, 255 } },
    };
    ImposterRenderSettings settings;
    settings.unitsPerMetre = 16.0f;
    settings.quantise.palette = palette;

    ImposterKey red;
    red.remap = { 10, 0, 0 };
    ImposterKey blue = red;
    blue.remap = { 20, 0, 0 };

    const auto redImage = renderImposter(model, red, settings);
    const auto blueImage = renderImposter(model, blue, settings);
    ASSERT_FALSE(redImage.pixels.empty());
    EXPECT_EQ(redImage.width, blueImage.width);
    auto count = [](const PaletteImage& image, uint8_t index) {
        return std::count(image.pixels.begin(), image.pixels.end(), index);
    };
    EXPECT_GT(count(redImage, 10), 50);
    EXPECT_EQ(count(redImage, 20), 0);
    EXPECT_GT(count(blueImage, 20), 50);
    EXPECT_EQ(count(blueImage, 10), 0);

    // Rendering is a pure function of the key.
    EXPECT_EQ(renderImposter(model, red, settings).pixels, redImage.pixels);

    // One zoom level out halves the size.
    ImposterKey zoomed = red;
    zoomed.zoom = 1;
    const auto small = renderImposter(model, zoomed, settings);
    EXPECT_NEAR(small.width, redImage.width / 2, 3);
}

TEST(ImposterCache, EvictCallbackSeesEveryDroppedKey)
{
    ImposterCache cache(15, false);
    std::vector<uint8_t> dropped;
    cache.setEvictCallback([&](const ImposterKey& key) { dropped.push_back(key.yawBucket); });
    cache.insert(keyWithYaw(1), imageOfSize(10));
    cache.insert(keyWithYaw(2), imageOfSize(10));
    // Without trim on insert nothing is dropped until trim is called, so images in use stay valid during a frame.
    EXPECT_TRUE(dropped.empty());
    EXPECT_EQ(cache.count(), 2u);
    cache.trim();
    EXPECT_EQ(dropped, (std::vector<uint8_t>{ 1 }));
    cache.insert(keyWithYaw(2), imageOfSize(5)); // replacing reports the old image
    EXPECT_EQ(dropped.size(), 2u);
    cache.clear();
    EXPECT_EQ(dropped.size(), 3u);
}
