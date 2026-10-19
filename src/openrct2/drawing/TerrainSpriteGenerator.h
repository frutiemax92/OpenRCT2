/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../world/TerrainShape.h"

#include <array>
#include <cstdint>
#include <vector>

// Procedural isometric terrain sprites (TER-030 to TER-033).
//
// A sprite is the projection (2:1 isometric, same as the viewport) of the two surface triangles of a tile shape, each
// pixel sampling the square texture of the quadrant it falls in. Sprites are positioned relative to the projection of
// the tile centre at the base height, in pixels of the requested zoom level.
namespace OpenRCT2::Drawing
{
    constexpr int32_t kTerrainZoomLevels = 4; // 1x, 2x, 4x, 8x reduced
    constexpr uint8_t kTerrainTransparent = 0;
    constexpr int32_t kTerrainShadeBands = 5;
    constexpr int32_t kTerrainNeutralShadeBand = 2;

    // Square tileable palettised texture with its reduced versions (TER-033).
    struct TerrainTexture
    {
        int32_t size = 0; // power of two
        std::vector<std::vector<uint8_t>> mips; // mips[0] is size x size, mips[n] is (size >> n) squared

        // pixels: size * size palette indices. Builds as many reduced levels as the size allows (up to 4 levels).
        static TerrainTexture Create(int32_t size, const std::vector<uint8_t>& pixels);
        uint8_t Sample(int32_t mip, int32_t u, int32_t v) const; // u, v wrap
        int32_t MipSize(int32_t mip) const;
    };

    // Maps (band, palette index) to a shaded palette index.
    struct TerrainShadeTable
    {
        std::vector<uint8_t> table; // kTerrainShadeBands * 256

        static TerrainShadeTable Identity();
        // Builds each band by scaling the palette colour and picking the nearest palette entry (index 0 is never chosen
        // for a non-transparent source).
        static TerrainShadeTable FromPalette(const std::array<std::array<uint8_t, 3>, 256>& rgb);
        uint8_t Shade(int32_t band, uint8_t index) const
        {
            return table[static_cast<size_t>(band) * 256 + index];
        }
    };

    struct TerrainSprite
    {
        int32_t xOffset = 0; // top-left, relative to the projected tile centre at base height
        int32_t yOffset = 0;
        int32_t width = 0;
        int32_t height = 0;
        std::vector<uint8_t> pixels; // width * height palette indices, kTerrainTransparent where empty
    };

    // Shade band 0..kTerrainShadeBands - 1 of a surface triangle for the given view rotation.
    int32_t GetTerrainShadeBand(const Terrain::Vector3Fixed& normal, int32_t rotation);

    // quadrants are indexed by Terrain::Quadrant. Textures must outlive the call and must not be null.
    TerrainSprite GenerateTerrainSprite(
        const Terrain::TerrainShape& shape, const std::array<const TerrainTexture*, 4>& quadrants,
        const TerrainShadeTable& shades, int32_t rotation, int32_t zoom);
} // namespace OpenRCT2::Drawing
