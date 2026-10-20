/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "ImposterWorker.h"

namespace OpenRCT2::Model
{
    ImposterWorker::ImposterWorker(size_t threads)
    {
        for (size_t i = 0; i < threads; i++)
            _threads.emplace_back([this] { loop(); });
    }

    ImposterWorker::~ImposterWorker()
    {
        {
            std::lock_guard lock(_mutex);
            _stop = true;
        }
        _wake.notify_all();
        for (auto& thread : _threads)
            thread.join();
    }

    bool ImposterWorker::request(const ImposterKey& key, std::function<PaletteImage()> render)
    {
        {
            std::lock_guard lock(_mutex);
            if (!_known.insert(key).second)
                return false;
            _queue.push_back(Task{ key, std::move(render) });
        }
        _wake.notify_one();
        return true;
    }

    std::vector<ImposterWorker::Finished> ImposterWorker::takeFinished()
    {
        std::lock_guard lock(_mutex);
        std::vector<Finished> result;
        result.swap(_finished);
        for (const auto& f : result)
            _known.erase(f.key);
        return result;
    }

    size_t ImposterWorker::pending() const
    {
        std::lock_guard lock(_mutex);
        return _queue.size() + _running;
    }

    void ImposterWorker::waitIdle()
    {
        std::unique_lock lock(_mutex);
        _idle.wait(lock, [&] { return _queue.empty() && _running == 0; });
    }

    size_t ImposterWorker::runPending()
    {
        size_t count = 0;
        for (;;)
        {
            Task task;
            {
                std::lock_guard lock(_mutex);
                if (_queue.empty())
                    break;
                task = std::move(_queue.front());
                _queue.pop_front();
                _running++;
            }
            execute(task);
            count++;
        }
        return count;
    }

    void ImposterWorker::execute(Task& task)
    {
        PaletteImage image;
        try
        {
            image = task.render();
        }
        catch (...)
        {
            // A failed render leaves an empty image, which draws nothing, instead of killing the worker.
        }
        {
            std::lock_guard lock(_mutex);
            _finished.push_back(Finished{ task.key, std::move(image) });
            _running--;
        }
        _idle.notify_all();
    }

    void ImposterWorker::loop()
    {
        for (;;)
        {
            Task task;
            {
                std::unique_lock lock(_mutex);
                _wake.wait(lock, [&] { return _stop || !_queue.empty(); });
                if (_stop && _queue.empty())
                    return;
                task = std::move(_queue.front());
                _queue.pop_front();
                _running++;
            }
            execute(task);
        }
    }
} // namespace OpenRCT2::Model
