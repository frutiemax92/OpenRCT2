/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "ImposterCache.h"

#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <unordered_set>
#include <vector>

namespace OpenRCT2::Model
{
    // Worker thread pool that rasterises imposters off the main thread (R3D-009). The main thread requests poses it
    // is missing and later collects the finished images to put into the cache, so it only composites cached sprites.
    class ImposterWorker
    {
    public:
        struct Finished
        {
            ImposterKey key;
            PaletteImage image;
        };

        // threads == 0 starts none: requests then only run when runPending() is called (used for deterministic tests).
        explicit ImposterWorker(size_t threads);
        ~ImposterWorker();
        ImposterWorker(const ImposterWorker&) = delete;
        ImposterWorker& operator=(const ImposterWorker&) = delete;

        // Queues a render; returns false without queuing when the same key is already pending or finished-uncollected.
        bool request(const ImposterKey& key, std::function<PaletteImage()> render);
        // Finished images since the last call. Main thread.
        std::vector<Finished> takeFinished();
        // Number of requests that are queued or running.
        size_t pending() const;
        // Blocks until every request has been rendered.
        void waitIdle();
        // Runs queued requests on the calling thread (only meaningful without worker threads).
        size_t runPending();
        size_t threadCount() const
        {
            return _threads.size();
        }

    private:
        struct Task
        {
            ImposterKey key;
            std::function<PaletteImage()> render;
        };

        void loop();
        void execute(Task& task);

        mutable std::mutex _mutex;
        std::condition_variable _wake;
        std::condition_variable _idle;
        std::deque<Task> _queue;
        std::vector<Finished> _finished;
        std::unordered_set<ImposterKey, ImposterKeyHash> _known; // queued, running or finished and not yet collected
        size_t _running = 0;
        bool _stop = false;
        std::vector<std::thread> _threads;
    };
} // namespace OpenRCT2::Model
