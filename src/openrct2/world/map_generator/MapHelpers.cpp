/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "MapHelpers.h"

#include "../Map.h"
#include "../MapLimits.h"
#include "../TerrainHeightField.h"
#include "../tile_element/SurfaceElement.h"

#include <algorithm>
#include <vector>

namespace OpenRCT2::World::MapGenerator
{
    void smoothMap(TileCoordsXY mapSize, SmoothStrength strength)
    {
        // The border ring of tiles is left alone, like the rest of the generator.
        const int32_t tilesX = mapSize.x - 2, tilesY = mapSize.y - 2;
        if (tilesX <= 0 || tilesY <= 0)
            return;

        std::vector<int32_t> base(static_cast<size_t>(tilesX) * static_cast<size_t>(tilesY));
        for (int32_t y = 0; y < tilesY; y++)
        {
            for (int32_t x = 0; x < tilesX; x++)
            {
                const auto* surface = MapGetSurfaceElementAt(TileCoordsXY{ x + 1, y + 1 });
                base[static_cast<size_t>(y) * tilesX + x] = surface != nullptr ? surface->baseHeight : kMinimumLandHeight;
            }
        }

        Terrain::HeightField field(tilesX, tilesY, kMaximumLandHeight, kMinimumLandHeight);
        field.SetFromTileHeights(base, strength == SmoothStrength::strong ? Terrain::kMaxCornerOffset : 2);

        for (int32_t y = 0; y < tilesY; y++)
        {
            for (int32_t x = 0; x < tilesX; x++)
            {
                auto* surface = MapGetSurfaceElementAt(TileCoordsXY{ x + 1, y + 1 });
                if (surface == nullptr)
                    continue;
                const auto [tileBase, shape] = field.GetTile(x, y);
                const bool hadWater = surface->getWaterHeight() > surface->getBaseZ();
                surface->baseHeight = static_cast<uint8_t>(tileBase);
                surface->setTerrainShape(shape);
                surface->clearanceHeight = static_cast<uint8_t>(
                    tileBase + std::max({ shape.north, shape.east, shape.south, shape.west }));
                if (!hadWater && surface->getWaterHeight() <= surface->getBaseZ())
                    surface->setWaterHeight(0);
            }
        }
    }
} // namespace OpenRCT2::World::MapGenerator
