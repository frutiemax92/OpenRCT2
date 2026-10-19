/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "Location.hpp"

#include <array>
#include <cstdint>

// Quarter-step terrain height model (TER-010 to TER-014).
//
// A tile surface has a base height and four corner offsets in Q (quarter steps, 8 z units each), 0..4 per corner,
// with at least one corner at 0. Corner positions inside the 32x32 tile, following the existing TileCornersZ
// naming: S = (0,0), E = (32,0), N = (32,32), W = (0,32).
namespace OpenRCT2::Terrain
{
    constexpr int32_t kQuarterStepZ = 8;
    constexpr int32_t kMaxCornerOffset = 4;
    constexpr size_t kShapeCount = 369; // 5^4 - 4^4

    struct TerrainShape
    {
        uint8_t north;
        uint8_t east;
        uint8_t south;
        uint8_t west;

        constexpr bool operator==(const TerrainShape&) const = default;
    };

    // The surface is cut into two triangles along one diagonal (TER-013).
    enum class SplitDiagonal : uint8_t
    {
        northSouth, // S(0,0) - N(32,32)
        eastWest,   // E(32,0) - W(0,32)
    };

    // Quadrants of the tile, each adjacent to one edge, named after the two corners the edge joins (TER-020).
    enum class Quadrant : uint8_t
    {
        northEast, // edge x = 32
        southEast, // edge y = 0
        southWest, // edge x = 0
        northWest, // edge y = 32
    };

    constexpr bool IsValidShape(const TerrainShape& s)
    {
        const auto max = static_cast<uint8_t>(kMaxCornerOffset);
        if (s.north > max || s.east > max || s.south > max || s.west > max)
            return false;
        return s.north == 0 || s.east == 0 || s.south == 0 || s.west == 0;
    }

    // 4 x 3 bits, N in the lowest bits then E, S, W.
    constexpr uint16_t EncodeShape(const TerrainShape& s)
    {
        return static_cast<uint16_t>(s.north | (s.east << 3) | (s.south << 6) | (s.west << 9));
    }
    constexpr TerrainShape DecodeShape(uint16_t code)
    {
        return { static_cast<uint8_t>(code & 7), static_cast<uint8_t>((code >> 3) & 7), static_cast<uint8_t>((code >> 6) & 7),
                 static_cast<uint8_t>((code >> 9) & 7) };
    }

    // Shape index table, built at startup. Indices 0..368 follow ascending encoded value, so flat is index 0.
    uint16_t GetShapeIndex(const TerrainShape& s);
    TerrainShape GetShapeFromIndex(uint16_t index);

    // Splits absolute corner heights (in z units) into a base height and a normalised shape. Heights must be
    // multiples of kQuarterStepZ and within kMaxCornerOffset of the lowest corner.
    bool MakeShape(const TileCornersZ& corners, int32_t& baseZ, TerrainShape& shape);

    // The diagonal joining the two corners whose offsets sum to the larger value; ties use north-south.
    SplitDiagonal GetSplitDiagonal(const TerrainShape& s);

    TileCornersZ GetCornerHeights(int32_t baseZ, const TerrainShape& s);

    // xl, yl are the position inside the tile, 0..31.
    int32_t GetSurfaceZAt(int32_t baseZ, const TerrainShape& s, int32_t xl, int32_t yl);
    int32_t GetSurfaceZAt(int32_t baseZ, const TerrainShape& s, const CoordsXY& worldPos);

    // Triangle index (0 or 1) under the point, for the split of the given shape. Shared by rendering and physics.
    int32_t GetTriangleAt(const TerrainShape& s, int32_t xl, int32_t yl);

    Quadrant GetQuadrantAt(int32_t xl, int32_t yl);

    // 16.16 fixed point unit vector.
    struct Vector3Fixed
    {
        int32_t x;
        int32_t y;
        int32_t z;
    };
    constexpr int32_t kFixedOne = 1 << 16;

    Vector3Fixed GetSurfaceNormalAt(const TerrainShape& s, int32_t xl, int32_t yl);
    Vector3Fixed GetSurfaceNormalAt(const TerrainShape& s, const CoordsXY& worldPos);
} // namespace OpenRCT2::Terrain
