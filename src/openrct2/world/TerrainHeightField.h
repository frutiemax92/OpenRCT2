/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "TerrainShape.h"

#include <cstdint>
#include <span>
#include <utility>
#include <vector>

namespace OpenRCT2::Terrain
{
    // Heights of the grid vertices of a map region, in Q, for the land tools (TER-015). Tile (x, y) has its corners at
    // vertices S = (x, y), E = (x + 1, y), N = (x + 1, y + 1) and W = (x, y + 1). Raising or lowering vertices keeps
    // every tile within kMaxCornerOffset Q by dragging the neighbouring vertices along (smoothing).
    class HeightField
    {
    public:
        HeightField(int32_t tilesX, int32_t tilesY, int32_t maxHeightQ, int32_t minHeightQ = 0);

        int32_t GetTilesX() const { return _tilesX; }
        int32_t GetTilesY() const { return _tilesY; }
        bool IsVertexValid(int32_t vx, int32_t vy) const;
        int32_t Get(int32_t vx, int32_t vy) const;
        void Set(int32_t vx, int32_t vy, int32_t heightQ);

        // Base height (Q) and shape of a tile.
        std::pair<int32_t, TerrainShape> GetTile(int32_t tx, int32_t ty) const;

        // Sets the four vertices of a tile from a base height and a shape, without smoothing.
        void SetTile(int32_t tx, int32_t ty, int32_t baseQ, const TerrainShape& shape);

        // Raises the lowest of the given vertices by stepQ (all of them when they are level), or lowers the highest.
        // Neighbours follow so no tile exceeds kMaxCornerOffset. Returns false, changing nothing, when a height would
        // leave minHeightQ..maxHeightQ. Vertices outside the field are ignored.
        bool Raise(std::span<const std::pair<int32_t, int32_t>> vertices, int32_t stepQ);
        bool Lower(std::span<const std::pair<int32_t, int32_t>> vertices, int32_t stepQ);

        // Vertices changed by the last successful Raise or Lower.
        const std::vector<std::pair<int32_t, int32_t>>& GetChangedVertices() const { return _changed; }

        // True when every tile fits the shape model.
        bool IsConsistent() const;

    private:
        bool Apply(std::span<const std::pair<int32_t, int32_t>> vertices, int32_t stepQ, bool raising);
        void Enforce(std::vector<std::pair<int32_t, int32_t>>& queue, bool raising);
        size_t Index(int32_t vx, int32_t vy) const { return static_cast<size_t>(vy) * (_tilesX + 1) + vx; }

        int32_t _tilesX;
        int32_t _tilesY;
        int32_t _maxHeightQ;
        int32_t _minHeightQ;
        std::vector<int32_t> _heights;
        std::vector<std::pair<int32_t, int32_t>> _changed;
    };
} // namespace OpenRCT2::Terrain
