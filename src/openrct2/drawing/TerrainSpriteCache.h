/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "TerrainSpriteGenerator.h"

#include <cstdint>
#include <functional>
#include <list>
#include <unordered_map>

namespace OpenRCT2::Drawing
{
    constexpr uint16_t kEdgeFaceShapeIndex = 0xFFFE;
    constexpr uint16_t kOverlayShapeIndexBase = 0x2000; // plus the shape index, for overlay sprites
    constexpr uint16_t kWaterShapeIndexBase = 0x4000; // plus the shape index, for water surface sprites
    constexpr uint16_t kStreamShapeIndexBase = 0x6000; // plus the shape index, for stream surface sprites
    constexpr uint16_t kStreetShapeIndexBase = 0x7000; // plus the shape index, for street surface sprites

    // Key of a generated terrain sprite (TER-032).
    struct TerrainSpriteKey
    {
        uint16_t shapeIndex;
        std::array<uint8_t, 4> quadrantSurfaces;
        uint8_t rotation;
        uint8_t zoom;
        // Surfaces across the tile edge each quadrant touches, and whether smoothing is on (TER-050).
        std::array<uint8_t, 4> neighbourSurfaces{};
        bool smoothing = false;
        // Edge faces: top and bottom heights of both ends and the height phase (shapeIndex is kEdgeFaceShapeIndex).
        std::array<int16_t, 5> edgeData{};

        bool operator==(const TerrainSpriteKey&) const = default;
    };

    struct TerrainSpriteKeyHash
    {
        size_t operator()(const TerrainSpriteKey& k) const;
    };

    // LRU cache of generated terrain sprites with a memory budget, and a limit on how many sprites may be generated
    // per frame so scrolling never stalls: when the limit is reached the caller gets nullptr and draws a flat-shaded
    // fallback for that tile.
    class TerrainSpriteCache
    {
    public:
        static constexpr size_t kDefaultBudgetBytes = 64u * 1024u * 1024u;
        static constexpr int32_t kDefaultMaxNewPerFrame = 256;

        explicit TerrainSpriteCache(size_t budgetBytes = kDefaultBudgetBytes, int32_t maxNewPerFrame = kDefaultMaxNewPerFrame);

        // Called for each sprite just before it is dropped (eviction or Clear).
        using EvictCallback = std::function<void(const TerrainSpriteKey&, TerrainSprite&)>;
        void SetEvictCallback(EvictCallback callback);

        // Applies a new budget at once. Not to be used while sprites of the current frame are still in use.
        void SetBudget(size_t budgetBytes);

        // Starts a frame: resets the generation limit and evicts least recently used sprites down to the budget.
        // Eviction only happens here, so every sprite handed out during a frame stays valid until the next one.
        void BeginFrame();

        // Returns the cached sprite, generating it with `generate` when missing and the frame limit allows. Returns
        // nullptr when it is not cached and the limit was reached. The pointer stays valid until the next BeginFrame.
        const TerrainSprite* Get(const TerrainSpriteKey& key, const std::function<TerrainSprite()>& generate);

        // Looks a sprite up without generating (still refreshes its recency).
        const TerrainSprite* Peek(const TerrainSpriteKey& key);

        void Clear();
        size_t GetBytesUsed() const { return _bytes; }
        size_t GetCount() const { return _entries.size(); }

    private:
        struct Entry
        {
            TerrainSpriteKey key;
            TerrainSprite sprite;
            size_t bytes;
        };
        using List = std::list<Entry>;

        void Evict();

        size_t _budget;
        int32_t _maxNewPerFrame;
        int32_t _newThisFrame = 0;
        EvictCallback _onEvict;
        size_t _bytes = 0;
        List _lru; // front = most recently used
        std::unordered_map<TerrainSpriteKey, List::iterator, TerrainSpriteKeyHash> _entries;
    };
} // namespace OpenRCT2::Drawing
