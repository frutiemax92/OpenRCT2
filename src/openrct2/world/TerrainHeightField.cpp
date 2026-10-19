/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "TerrainHeightField.h"

#include <algorithm>
#include <cassert>

namespace OpenRCT2::Terrain
{
    HeightField::HeightField(int32_t tilesX, int32_t tilesY, int32_t maxHeightQ, int32_t minHeightQ)
        : _tilesX(tilesX)
        , _tilesY(tilesY)
        , _maxHeightQ(maxHeightQ)
        , _minHeightQ(minHeightQ)
        , _heights(static_cast<size_t>(tilesX + 1) * static_cast<size_t>(tilesY + 1), 0)
    {
    }

    bool HeightField::IsVertexValid(int32_t vx, int32_t vy) const
    {
        return vx >= 0 && vy >= 0 && vx <= _tilesX && vy <= _tilesY;
    }

    int32_t HeightField::Get(int32_t vx, int32_t vy) const
    {
        assert(IsVertexValid(vx, vy));
        return _heights[Index(vx, vy)];
    }

    void HeightField::Set(int32_t vx, int32_t vy, int32_t heightQ)
    {
        assert(IsVertexValid(vx, vy));
        _heights[Index(vx, vy)] = heightQ;
    }

    std::pair<int32_t, TerrainShape> HeightField::GetTile(int32_t tx, int32_t ty) const
    {
        const int32_t s = Get(tx, ty), e = Get(tx + 1, ty), n = Get(tx + 1, ty + 1), w = Get(tx, ty + 1);
        const int32_t lowest = std::min({ s, e, n, w });
        return { lowest,
                 { static_cast<uint8_t>(n - lowest), static_cast<uint8_t>(e - lowest), static_cast<uint8_t>(s - lowest),
                   static_cast<uint8_t>(w - lowest) } };
    }

    void HeightField::SetTile(int32_t tx, int32_t ty, int32_t baseQ, const TerrainShape& shape)
    {
        Set(tx, ty, baseQ + shape.south);
        Set(tx + 1, ty, baseQ + shape.east);
        Set(tx + 1, ty + 1, baseQ + shape.north);
        Set(tx, ty + 1, baseQ + shape.west);
    }

    bool HeightField::IsConsistent() const
    {
        for (int32_t ty = 0; ty < _tilesY; ty++)
            for (int32_t tx = 0; tx < _tilesX; tx++)
            {
                const auto [base, shape] = GetTile(tx, ty);
                if (!IsValidShape(shape) || base < _minHeightQ || base > _maxHeightQ)
                    return false;
            }
        return true;
    }

    void HeightField::Enforce(std::vector<std::pair<int32_t, int32_t>>& queue, bool raising)
    {
        while (!queue.empty())
        {
            const auto [vx, vy] = queue.back();
            queue.pop_back();
            // The four tiles touching this vertex.
            for (int32_t ty = vy - 1; ty <= vy; ty++)
            {
                for (int32_t tx = vx - 1; tx <= vx; tx++)
                {
                    if (tx < 0 || ty < 0 || tx >= _tilesX || ty >= _tilesY)
                        continue;
                    const std::pair<int32_t, int32_t> corners[4] = { { tx, ty }, { tx + 1, ty }, { tx + 1, ty + 1 }, { tx, ty + 1 } };
                    int32_t lo = INT32_MAX, hi = INT32_MIN;
                    for (auto [cx, cy] : corners)
                    {
                        lo = std::min(lo, Get(cx, cy));
                        hi = std::max(hi, Get(cx, cy));
                    }
                    if (hi - lo <= kMaxCornerOffset)
                        continue;
                    for (auto [cx, cy] : corners)
                    {
                        const int32_t h = Get(cx, cy);
                        if (raising && h < hi - kMaxCornerOffset)
                        {
                            Set(cx, cy, hi - kMaxCornerOffset);
                            queue.emplace_back(cx, cy);
                        }
                        else if (!raising && h > lo + kMaxCornerOffset)
                        {
                            Set(cx, cy, lo + kMaxCornerOffset);
                            queue.emplace_back(cx, cy);
                        }
                    }
                }
            }
        }
    }

    void HeightField::Smooth()
    {
        std::vector<std::pair<int32_t, int32_t>> queue;
        for (int32_t vy = 0; vy <= _tilesY; vy++)
            for (int32_t vx = 0; vx <= _tilesX; vx++)
                queue.emplace_back(vx, vy);
        Enforce(queue, true);
    }

    void HeightField::SetFromTileHeights(std::span<const int32_t> tileBase, int32_t maxDeviationQ)
    {
        assert(tileBase.size() == static_cast<size_t>(_tilesX) * static_cast<size_t>(_tilesY));
        for (int32_t vy = 0; vy <= _tilesY; vy++)
        {
            for (int32_t vx = 0; vx <= _tilesX; vx++)
            {
                int32_t sum = 0, count = 0, lo = INT32_MIN, hi = INT32_MAX;
                for (int32_t ty = vy - 1; ty <= vy; ty++)
                {
                    for (int32_t tx = vx - 1; tx <= vx; tx++)
                    {
                        if (tx < 0 || ty < 0 || tx >= _tilesX || ty >= _tilesY)
                            continue;
                        const int32_t h = tileBase[static_cast<size_t>(ty) * _tilesX + tx];
                        sum += h;
                        count++;
                        lo = std::max(lo, h - maxDeviationQ);
                        hi = std::min(hi, h + maxDeviationQ);
                    }
                }
                int32_t value = (sum + count / 2) / count;
                if (lo <= hi)
                    value = std::clamp(value, lo, hi);
                Set(vx, vy, std::clamp(value, _minHeightQ, _maxHeightQ));
            }
        }
        Smooth();
    }

    bool HeightField::Apply(std::span<const std::pair<int32_t, int32_t>> vertices, int32_t stepQ, bool raising)
    {
        _changed.clear();
        if (stepQ <= 0)
            return false;

        std::vector<std::pair<int32_t, int32_t>> valid;
        for (auto v : vertices)
            if (IsVertexValid(v.first, v.second))
                valid.push_back(v);
        if (valid.empty())
            return false;

        int32_t extreme = Get(valid[0].first, valid[0].second);
        for (auto [vx, vy] : valid)
            extreme = raising ? std::min(extreme, Get(vx, vy)) : std::max(extreme, Get(vx, vy));

        const auto backup = _heights;
        std::vector<std::pair<int32_t, int32_t>> queue;
        for (auto [vx, vy] : valid)
        {
            if (Get(vx, vy) != extreme)
                continue;
            Set(vx, vy, extreme + (raising ? stepQ : -stepQ));
            queue.emplace_back(vx, vy);
        }
        Enforce(queue, raising);

        const bool inRange = std::all_of(
            _heights.begin(), _heights.end(), [&](int32_t h) { return h >= _minHeightQ && h <= _maxHeightQ; });
        if (!inRange)
        {
            _heights = backup;
            return false;
        }
        for (int32_t vy = 0; vy <= _tilesY; vy++)
            for (int32_t vx = 0; vx <= _tilesX; vx++)
                if (_heights[Index(vx, vy)] != backup[Index(vx, vy)])
                    _changed.emplace_back(vx, vy);
        return true;
    }

    bool HeightField::Raise(std::span<const std::pair<int32_t, int32_t>> vertices, int32_t stepQ)
    {
        return Apply(vertices, stepQ, true);
    }

    bool HeightField::Lower(std::span<const std::pair<int32_t, int32_t>> vertices, int32_t stepQ)
    {
        return Apply(vertices, stepQ, false);
    }
} // namespace OpenRCT2::Terrain
