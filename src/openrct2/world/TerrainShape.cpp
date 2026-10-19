/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "TerrainShape.h"

#include "tile_element/Slope.h"

#include <algorithm>
#include <cassert>
#include <vector>

namespace OpenRCT2::Terrain
{
    namespace
    {
        constexpr int32_t kTile = 32;

        struct ShapeTables
        {
            std::array<uint16_t, 4096> indexOfCode{};
            std::array<uint16_t, kShapeCount> codeOfIndex{};

            ShapeTables()
            {
                indexOfCode.fill(0xFFFF);
                size_t n = 0;
                for (uint16_t code = 0; code < 4096; code++)
                {
                    if (!IsValidShape(DecodeShape(code)))
                        continue;
                    indexOfCode[code] = static_cast<uint16_t>(n);
                    codeOfIndex[n] = code;
                    n++;
                }
                assert(n == kShapeCount);
            }
        };

        const ShapeTables& GetTables()
        {
            static const ShapeTables tables;
            return tables;
        }

        uint32_t ISqrt(uint64_t v)
        {
            uint64_t r = 0;
            uint64_t bit = 1ULL << 62;
            while (bit > v)
                bit >>= 2;
            while (bit != 0)
            {
                if (v >= r + bit)
                {
                    v -= r + bit;
                    r = (r >> 1) + bit;
                }
                else
                {
                    r >>= 1;
                }
                bit >>= 2;
            }
            return static_cast<uint32_t>(r);
        }

        // Triangle ids: northSouth split: 0 = S,E,N (x >= y), 1 = S,N,W. eastWest split: 0 = S,E,W (x + y <= 32), 1 = E,N,W.
        // Heights are returned in z units.
        int32_t H(uint8_t offset)
        {
            return offset * kQuarterStepZ;
        }
    } // namespace

    uint16_t GetShapeIndex(const TerrainShape& s)
    {
        if (!IsValidShape(s))
            return 0xFFFF;
        return GetTables().indexOfCode[EncodeShape(s)];
    }

    TerrainShape GetShapeFromIndex(uint16_t index)
    {
        assert(index < kShapeCount);
        return DecodeShape(GetTables().codeOfIndex[std::min<size_t>(index, kShapeCount - 1)]);
    }

    bool MakeShape(const TileCornersZ& corners, int32_t& baseZ, TerrainShape& shape)
    {
        const int32_t lowest = std::min({ corners.north, corners.east, corners.south, corners.west });
        const int32_t offs[4] = { corners.north - lowest, corners.east - lowest, corners.south - lowest, corners.west - lowest };
        for (int32_t o : offs)
        {
            if (o % kQuarterStepZ != 0 || o / kQuarterStepZ > kMaxCornerOffset)
                return false;
        }
        baseZ = lowest;
        shape = { static_cast<uint8_t>(offs[0] / kQuarterStepZ), static_cast<uint8_t>(offs[1] / kQuarterStepZ),
                  static_cast<uint8_t>(offs[2] / kQuarterStepZ), static_cast<uint8_t>(offs[3] / kQuarterStepZ) };
        return true;
    }

    SplitDiagonal GetSplitDiagonal(const TerrainShape& s)
    {
        const int32_t ns = s.north + s.south;
        const int32_t ew = s.east + s.west;
        return ew > ns ? SplitDiagonal::eastWest : SplitDiagonal::northSouth;
    }

    TileCornersZ GetCornerHeights(int32_t baseZ, const TerrainShape& s)
    {
        return { baseZ + H(s.north), baseZ + H(s.east), baseZ + H(s.south), baseZ + H(s.west) };
    }

    int32_t GetTriangleAt(const TerrainShape& s, int32_t xl, int32_t yl)
    {
        if (GetSplitDiagonal(s) == SplitDiagonal::northSouth)
            return xl >= yl ? 0 : 1;
        return xl + yl <= kTile ? 0 : 1;
    }

    int32_t GetSurfaceZAt(int32_t baseZ, const TerrainShape& s, int32_t xl, int32_t yl)
    {
        const int32_t hN = H(s.north), hE = H(s.east), hS = H(s.south), hW = H(s.west);
        int32_t scaled; // height * 32
        if (GetSplitDiagonal(s) == SplitDiagonal::northSouth)
        {
            if (xl >= yl)
                scaled = (kTile - xl) * hS + (xl - yl) * hE + yl * hN;
            else
                scaled = (kTile - yl) * hS + xl * hN + (yl - xl) * hW;
        }
        else
        {
            if (xl + yl <= kTile)
                scaled = (kTile - xl - yl) * hS + xl * hE + yl * hW;
            else
                scaled = (xl + yl - kTile) * hN + (kTile - yl) * hE + (kTile - xl) * hW;
        }
        return baseZ + scaled / kTile;
    }

    int32_t GetSurfaceZAt(int32_t baseZ, const TerrainShape& s, const CoordsXY& worldPos)
    {
        return GetSurfaceZAt(baseZ, s, worldPos.x & (kTile - 1), worldPos.y & (kTile - 1));
    }

    TerrainShape ShapeFromLegacySlope(uint8_t legacySlope, uint8_t* baseOffsetQ)
    {
        const auto rel = GetSlopeRelativeCornerHeights(legacySlope);
        // Same corner naming as GetSlopeCornerHeights: north = bottom, east = left, south = top, west = right.
        const auto lowest = std::min({ rel.bottom, rel.left, rel.top, rel.right });
        if (baseOffsetQ != nullptr)
            *baseOffsetQ = static_cast<uint8_t>(lowest * 2);
        return { static_cast<uint8_t>((rel.bottom - lowest) * 2), static_cast<uint8_t>((rel.left - lowest) * 2),
                 static_cast<uint8_t>((rel.top - lowest) * 2), static_cast<uint8_t>((rel.right - lowest) * 2) };
    }

    uint8_t LegacySlopeFromShape(const TerrainShape& shape)
    {
        const bool even = shape.north % 2 == 0 && shape.east % 2 == 0 && shape.south % 2 == 0 && shape.west % 2 == 0;
        if (even)
        {
            for (uint8_t slope = 0; slope <= kTileSlopeMask; slope++)
            {
                if (ShapeFromLegacySlope(slope) == shape)
                    return slope;
            }
        }
        uint8_t mask = 0;
        mask |= shape.north >= 2 ? kTileSlopeNCornerUp : 0;
        mask |= shape.east >= 2 ? kTileSlopeECornerUp : 0;
        mask |= shape.south >= 2 ? kTileSlopeSCornerUp : 0;
        mask |= shape.west >= 2 ? kTileSlopeWCornerUp : 0;
        return mask == kTileSlopeRaisedCornersMask ? kTileSlopeFlat : mask;
    }

    Quadrant GetQuadrantAt(int32_t xl, int32_t yl)
    {
        const int32_t a = xl - yl;       // > 0 on the south-east side of the north-south diagonal
        const int32_t b = xl + yl - kTile; // > 0 on the north-east side of the east-west diagonal
        if (a >= 0)
            return b >= 0 ? Quadrant::northEast : Quadrant::southEast;
        return b >= 0 ? Quadrant::northWest : Quadrant::southWest;
    }

    Vector3Fixed GetSurfaceNormalAt(const TerrainShape& s, int32_t xl, int32_t yl)
    {
        const int32_t hN = H(s.north), hE = H(s.east), hS = H(s.south), hW = H(s.west);
        int32_t gx, gy; // gradient * 32
        const bool ns = GetSplitDiagonal(s) == SplitDiagonal::northSouth;
        if (GetTriangleAt(s, xl, yl) == 0)
        {
            if (ns)
            {
                gx = hE - hS;
                gy = hN - hE;
            }
            else
            {
                gx = hE - hS;
                gy = hW - hS;
            }
        }
        else
        {
            if (ns)
            {
                gx = hN - hW;
                gy = hW - hS;
            }
            else
            {
                gx = hN - hW;
                gy = hN - hE;
            }
        }
        const int64_t nx = -gx, ny = -gy, nz = kTile;
        // The length is computed with 8 extra bits so truncation stays far below the fixed point resolution.
        const auto len = static_cast<int64_t>(ISqrt(static_cast<uint64_t>(nx * nx + ny * ny + nz * nz) << 16));
        constexpr int64_t kScale = static_cast<int64_t>(kFixedOne) << 8;
        return { static_cast<int32_t>(nx * kScale / len), static_cast<int32_t>(ny * kScale / len),
                 static_cast<int32_t>(nz * kScale / len) };
    }

    Vector3Fixed GetSurfaceNormalAt(const TerrainShape& s, const CoordsXY& worldPos)
    {
        return GetSurfaceNormalAt(s, worldPos.x & (kTile - 1), worldPos.y & (kTile - 1));
    }
} // namespace OpenRCT2::Terrain
