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
        int32_t smoothingPriority = 0; // the higher one bleeds over the lower where two surfaces meet (TER-050)
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

    // Smoothing between surfaces (TER-050, TER-051): where a lower priority surface borders a higher one, a dithered band
    // of the higher one bleeds over it, along the diagonals inside the tile and along the tile edges.
    struct TerrainSmoothing
    {
        bool enabled = false;
        int32_t bandWidth = 3; // in tile units, about pixels at 1x zoom
        // Texture across the tile edge each quadrant touches, indexed by Terrain::Quadrant; null = none.
        std::array<const TerrainTexture*, 4> edgeNeighbours{};
    };

    // quadrants are indexed by Terrain::Quadrant. Textures must outlive the call and must not be null.
    TerrainSprite GenerateTerrainSprite(
        const Terrain::TerrainShape& shape, const std::array<const TerrainTexture*, 4>& quadrants,
        const TerrainShadeTable& shades, int32_t rotation, int32_t zoom, const TerrainSmoothing& smoothing = {});

    // A vertical face along one tile edge, hanging from this tile's edge down to the lower neighbouring edge
    // (TER-060). Heights are z units relative to the base height of the tile; A and B are the edge ends in increasing
    // x (edges y = 0 and y = 32) or increasing y (edges x = 0 and x = 32).
    struct TerrainEdgeFace
    {
        Terrain::Quadrant edge; // the quadrant touching the edge: north-east x = 32, south-east y = 0, south-west x = 0, north-west y = 32
        int32_t topA;
        int32_t topB;
        int32_t bottomA;
        int32_t bottomB;
        int32_t baseZ; // absolute base height of the tile, so the texture lines up between tiles
    };

    // Sprite of an edge face, positioned like the surface sprites. Empty when the face has no area.
    TerrainSprite GenerateTerrainEdgeSprite(
        const TerrainTexture& texture, const TerrainEdgeFace& face, const TerrainShadeTable& shades, int32_t rotation,
        int32_t zoom);

    // True when the vertical face of the tile edge points towards the viewer for the rotation.
    bool IsEdgeFacingViewer(Terrain::Quadrant edge, int32_t rotation);

    // Procedural water surface (WAT-001, WAT-002): the flat plane of the water over a tile, kept only where the terrain
    // is below it. Palette indices are the animated water ones, so the palette cycling makes it move.
    enum class WaterPattern : uint8_t
    {
        mask,   // plain coverage, drawn blended to tint what is under the water
        sparse, // scattered waves and sparkles, over transparent water
        opaque, // full waves, for opaque water
    };

    // terrainBaseZ is the tile's base height relative to the water level (z units, negative when the tile is below it).
    // The sprite is positioned like the surface sprites, with the water level as base height.
    TerrainSprite GenerateTerrainWaterSprite(
        const Terrain::TerrainShape& shape, int32_t terrainBaseZ, WaterPattern pattern, int32_t rotation, int32_t zoom);

    // Overlays drawn over the terrain surface (TER-034): grid lines, ownership outlines, selection highlights. They follow
    // the exact surface of any shape.
    enum class TerrainOverlayKind : uint8_t
    {
        fill,    // the whole surface of the tile
        outline, // a one pixel line along the tile edges
        dotted,  // the same, every other pixel
        corner,  // the surface within reach of one corner; index = Terrain corner N, E, S, W = 0..3
        edge,    // a band along one edge; index = quadrant touching the edge (Terrain::Quadrant)
        quarter, // one of the four square quarters of the tile; index = the quarter around corner N, E, S, W = 0..3
    };

    struct TerrainOverlay
    {
        TerrainOverlayKind kind = TerrainOverlayKind::fill;
        uint8_t index = 0;
        uint8_t colour = 1; // palette index of the opaque pixels
    };

    TerrainSprite GenerateTerrainOverlaySprite(
        const Terrain::TerrainShape& shape, const TerrainOverlay& overlay, int32_t rotation, int32_t zoom);

    // Procedural stream surface (STR-005): the stream texture masked to its connection pattern and projected onto the
    // surface triangles.
    TerrainSprite GenerateTerrainStreamSprite(
        const Terrain::TerrainShape& shape,
        const TerrainTexture& texture,
        uint8_t edges,
        int32_t streamWidth,
        const TerrainShadeTable& shades,
        int32_t rotation,
        int32_t zoom);

    // Procedural street surface (STE-003): the road texture and the markings over it, cut to the connection pattern of the
    // tile and projected onto the surface triangles, so a road lies on any ramp. The road is `roadWidth` units wide; a
    // one-way road flows towards `flowDirection`. isMarking says which palette indices of the mask are markings.
    TerrainSprite GenerateTerrainStreetSprite(
        const Terrain::TerrainShape& shape,
        const TerrainTexture& road,
        const TerrainTexture& markings,
        const std::array<bool, 256>& isMarking,
        uint8_t connections,
        bool oneWay,
        int32_t flowDirection,
        int32_t roadWidth,
        const TerrainShadeTable& shades,
        int32_t rotation,
        int32_t zoom);

    // Procedural parking surface (STE-010): the road texture over the whole tile with the lines between the two bays, which
    // open onto the edge `facing`.
    TerrainSprite GenerateTerrainParkingSprite(
        const Terrain::TerrainShape& shape,
        const TerrainTexture& road,
        uint8_t lineColour,
        int32_t facing,
        const TerrainShadeTable& shades,
        int32_t rotation,
        int32_t zoom);
} // namespace OpenRCT2::Drawing
