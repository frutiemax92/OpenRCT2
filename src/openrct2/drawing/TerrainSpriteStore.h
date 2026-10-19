/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "ImageIndexType.h"
#include "../object/Object.h"
#include "../world/Location.hpp"
#include "TerrainSpriteCache.h"

#include <cstdint>
#include <mutex>
#include <unordered_map>

namespace OpenRCT2
{
    struct SurfaceElement;
}

namespace OpenRCT2::Drawing
{
    // Glue between the procedural terrain sprites and the image list: generated sprites are cached (TER-032) and
    // registered as images so the paint code can draw them like any other sprite.
    class TerrainSpriteStore
    {
    public:
        static TerrainSpriteStore& Instance();

        // Main thread, once per frame before painting: evicts old sprites and resets the generation limit.
        void BeginFrame();

        // Drops every sprite, e.g. when terrain objects change. Main thread only.
        void Clear();

        // The image of the surface of a tile for the view rotation, or kImageIndexUndefined when it cannot be drawn
        // procedurally yet (missing texture, generation limit reached): the caller draws a fallback. The image stays
        // valid until the next BeginFrame. Safe to call from paint threads.
        // tilePos is the tile's map position, used to look up the surfaces of its neighbours for smoothing.
        ImageIndex GetImage(const SurfaceElement& surface, const CoordsXY& tilePos, int32_t rotation);

        // The image of a vertical edge face of a tile (TER-060), or kImageIndexUndefined without an edge texture.
        ImageIndex GetEdgeImage(ObjectEntryIndex edgeObject, const TerrainEdgeFace& face, int32_t rotation);

        // The water surface of a tile (WAT-001), drawn at the water level: only where the terrain is below it. Returns
        // kImageIndexUndefined when there is no water or the sprite is not ready yet.
        ImageIndex GetWaterImage(const SurfaceElement& surface, WaterPattern pattern, int32_t rotation);

        size_t GetSpriteCount() const;

    private:
        TerrainSpriteStore();
        ImageIndex RegisterImage(const TerrainSpriteKey& key, const TerrainSprite& sprite);

        mutable std::mutex _mutex;
        TerrainSpriteCache _cache;
        std::unordered_map<TerrainSpriteKey, ImageIndex, TerrainSpriteKeyHash> _images;
        TerrainShadeTable _shades;
        bool _shadesBuilt = false;
    };
} // namespace OpenRCT2::Drawing
