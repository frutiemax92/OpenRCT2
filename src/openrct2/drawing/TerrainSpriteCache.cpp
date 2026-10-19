/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "TerrainSpriteCache.h"

namespace OpenRCT2::Drawing
{
    size_t TerrainSpriteKeyHash::operator()(const TerrainSpriteKey& k) const
    {
        uint64_t h = 1469598103934665603ULL; // FNV-1a
        auto mix = [&h](uint64_t v) {
            h ^= v;
            h *= 1099511628211ULL;
        };
        mix(k.shapeIndex);
        for (auto q : k.quadrantSurfaces)
            mix(q);
        mix(k.rotation);
        mix(k.zoom);
        for (auto n : k.neighbourSurfaces)
            mix(n);
        mix(k.smoothing ? 1 : 0);
        return static_cast<size_t>(h);
    }

    TerrainSpriteCache::TerrainSpriteCache(size_t budgetBytes, int32_t maxNewPerFrame)
        : _budget(budgetBytes)
        , _maxNewPerFrame(maxNewPerFrame)
    {
    }

    void TerrainSpriteCache::SetEvictCallback(EvictCallback callback)
    {
        _onEvict = std::move(callback);
    }

    void TerrainSpriteCache::SetBudget(size_t budgetBytes)
    {
        _budget = budgetBytes;
        Evict();
    }

    void TerrainSpriteCache::BeginFrame()
    {
        _newThisFrame = 0;
        Evict();
    }

    const TerrainSprite* TerrainSpriteCache::Peek(const TerrainSpriteKey& key)
    {
        auto it = _entries.find(key);
        if (it == _entries.end())
            return nullptr;
        _lru.splice(_lru.begin(), _lru, it->second);
        return &it->second->sprite;
    }

    const TerrainSprite* TerrainSpriteCache::Get(const TerrainSpriteKey& key, const std::function<TerrainSprite()>& generate)
    {
        if (const auto* cached = Peek(key))
            return cached;
        if (_newThisFrame >= _maxNewPerFrame)
            return nullptr;
        _newThisFrame++;

        Entry entry{ key, generate(), 0 };
        entry.bytes = sizeof(Entry) + entry.sprite.pixels.size();
        _bytes += entry.bytes;
        _lru.push_front(std::move(entry));
        _entries[key] = _lru.begin();
        return &_lru.front().sprite;
    }

    void TerrainSpriteCache::Clear()
    {
        if (_onEvict)
        {
            for (auto& entry : _lru)
                _onEvict(entry.key, entry.sprite);
        }
        _lru.clear();
        _entries.clear();
        _bytes = 0;
    }

    void TerrainSpriteCache::Evict()
    {
        while (_bytes > _budget && !_lru.empty())
        {
            auto& last = _lru.back();
            if (_onEvict)
                _onEvict(last.key, last.sprite);
            _bytes -= last.bytes;
            _entries.erase(last.key);
            _lru.pop_back();
        }
    }
} // namespace OpenRCT2::Drawing
