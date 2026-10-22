/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "GlbTestHelpers.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <gtest/gtest.h>
#include <openrct2/drawing/Image.h>
#include <openrct2/drawing/ModelSpriteStore.h>
#include <openrct2/model/ImposterWorker.h>
#include <openrct2/model/ModelViewer.h>
#include <set>
#include <thread>

using namespace OpenRCT2::Model;

namespace
{
    ImposterKey keyWithYaw(uint8_t yaw)
    {
        ImposterKey key;
        key.yawBucket = yaw;
        return key;
    }

    PaletteImage imageOf(uint8_t value)
    {
        PaletteImage image;
        image.width = 1;
        image.height = 1;
        image.pixels = { value };
        return image;
    }
} // namespace

TEST(ImposterWorker, WithoutThreadsRequestsRunWhenAsked)
{
    ImposterWorker worker(0);
    EXPECT_EQ(worker.threadCount(), 0u);
    EXPECT_TRUE(worker.request(keyWithYaw(1), [] { return imageOf(11); }));
    EXPECT_TRUE(worker.request(keyWithYaw(2), [] { return imageOf(22); }));
    EXPECT_EQ(worker.pending(), 2u);
    EXPECT_TRUE(worker.takeFinished().empty());

    EXPECT_EQ(worker.runPending(), 2u);
    EXPECT_EQ(worker.pending(), 0u);
    const auto finished = worker.takeFinished();
    ASSERT_EQ(finished.size(), 2u);
    EXPECT_EQ(finished[0].key.yawBucket, 1);
    EXPECT_EQ(finished[0].image.pixels[0], 11);
    EXPECT_EQ(finished[1].image.pixels[0], 22);
    EXPECT_TRUE(worker.takeFinished().empty());
}

TEST(ImposterWorker, DuplicateRequestsAreIgnoredUntilCollected)
{
    ImposterWorker worker(0);
    std::atomic<int> renders = 0;
    auto render = [&] {
        renders++;
        return imageOf(1);
    };
    EXPECT_TRUE(worker.request(keyWithYaw(5), render));
    EXPECT_FALSE(worker.request(keyWithYaw(5), render)); // queued
    worker.runPending();
    EXPECT_FALSE(worker.request(keyWithYaw(5), render)); // finished but not collected
    EXPECT_EQ(worker.takeFinished().size(), 1u);
    EXPECT_TRUE(worker.request(keyWithYaw(5), render)); // after collection it may be asked again
    worker.runPending();
    EXPECT_EQ(renders.load(), 2);
}

TEST(ImposterWorker, WorkerThreadsRenderEverythingOnce)
{
    ImposterWorker worker(3);
    EXPECT_EQ(worker.threadCount(), 3u);
    std::atomic<int> renders = 0;
    std::set<std::thread::id> threads;
    std::mutex threadsMutex;
    constexpr int kCount = 40;
    for (int i = 0; i < kCount; i++)
    {
        worker.request(keyWithYaw(static_cast<uint8_t>(i)), [&, i] {
            renders++;
            {
                std::lock_guard lock(threadsMutex);
                threads.insert(std::this_thread::get_id());
            }
            return imageOf(static_cast<uint8_t>(i));
        });
    }
    worker.waitIdle();
    EXPECT_EQ(renders.load(), kCount);
    EXPECT_EQ(worker.pending(), 0u);
    EXPECT_EQ(threads.count(std::this_thread::get_id()), 0u); // never the calling thread

    auto finished = worker.takeFinished();
    ASSERT_EQ(finished.size(), static_cast<size_t>(kCount));
    std::sort(finished.begin(), finished.end(), [](const auto& a, const auto& b) { return a.key.yawBucket < b.key.yawBucket; });
    for (int i = 0; i < kCount; i++)
        EXPECT_EQ(finished[i].image.pixels[0], i);
}

TEST(ImposterWorker, AThrowingRenderYieldsAnEmptyImageAndTheWorkerSurvives)
{
    ImposterWorker worker(1);
    worker.request(keyWithYaw(1), []() -> PaletteImage { throw std::runtime_error("bad model"); });
    worker.request(keyWithYaw(2), [] { return imageOf(7); });
    worker.waitIdle();
    auto finished = worker.takeFinished();
    ASSERT_EQ(finished.size(), 2u);
    for (const auto& f : finished)
    {
        if (f.key.yawBucket == 1)
            EXPECT_TRUE(f.image.pixels.empty());
        else
            EXPECT_EQ(f.image.pixels[0], 7);
    }
}

TEST(ImposterWorker, DestructorFinishesQueuedWorkWithoutHanging)
{
    std::atomic<int> renders = 0;
    {
        ImposterWorker worker(2);
        for (int i = 0; i < 10; i++)
            worker.request(keyWithYaw(static_cast<uint8_t>(i)), [&] {
                renders++;
                return imageOf(1);
            });
    }
    EXPECT_EQ(renders.load(), 10);
}

TEST(ModelSpriteStore, WorkersFillTheCacheAndTheNextFrameDrawsIt)
{
    auto& store = OpenRCT2::Drawing::ModelSpriteStore::Instance();
    store.Clear();
    store.SetWorkerThreads(2);

    auto model = std::make_shared<GlbModel>();
    auto loaded = loadGlb(GlbTest::triangleGlb());
    ASSERT_TRUE(loaded.ok) << loaded.error;
    *model = std::move(loaded.model);
    const auto id = store.RegisterModel(model, 16.0f);

    ImposterKey key;
    key.modelId = id;
    key.rotation = 2; // a rotation that shows the triangle's front face
    // The first request only queues the render.
    EXPECT_EQ(store.GetImage(key), kImageIndexUndefined);
    store.WaitForWorkers();
    EXPECT_EQ(store.GetSpriteCount(), 1u);
    // Whatever the pose draws, asking again must not queue again nor crash.
    const auto second = store.GetImage(key);
    EXPECT_EQ(store.GetImage(key), second);

    store.SetWorkerThreads(0);
    store.Clear();
    store.UnregisterModel(id);
}

// A model standing still is only drawn again if the frame that takes its finished sprite says so (the screen is then
// invalidated), otherwise a staff member or golfer who stands still would stay invisible.
TEST(ModelSpriteStore, BeginFrameCountsTheSpritesTheWorkersFinished)
{
    auto& store = OpenRCT2::Drawing::ModelSpriteStore::Instance();
    store.Clear();
    store.SetWorkerThreads(1);

    auto model = std::make_shared<GlbModel>();
    auto loaded = loadGlb(GlbTest::triangleGlb());
    ASSERT_TRUE(loaded.ok) << loaded.error;
    *model = std::move(loaded.model);
    const auto id = store.RegisterModel(model, 16.0f);

    ImposterKey key;
    key.modelId = id;
    key.rotation = 2;
    EXPECT_EQ(store.GetImage(key), kImageIndexUndefined);

    size_t finished = 0;
    for (int i = 0; i < 500 && finished == 0; i++)
    {
        finished = store.BeginFrame();
        if (finished == 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_EQ(finished, 1u);
    EXPECT_EQ(store.BeginFrame(), 0u);

    store.SetWorkerThreads(0);
    store.Clear();
    store.UnregisterModel(id);
}
