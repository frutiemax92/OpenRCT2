/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "TerrainSpriteGenerator.h"

#include <algorithm>
#include <cassert>
#include <cstdlib>

namespace OpenRCT2::Drawing
{
    using namespace Terrain;

    namespace
    {
        constexpr int32_t kTile = 32;
        constexpr int32_t kHalfTile = 16;

        struct Vertex
        {
            int32_t lx, ly, z; // local position inside the tile (0..32) and height
            int64_t sx, sy;    // screen position at 1x, relative to the projected tile centre at base height
        };

        // Rotation of an offset from the tile centre, same convention as CoordsXY::rotate.
        void RotateOffset(int32_t rotation, int32_t& x, int32_t& y)
        {
            const int32_t ox = x, oy = y;
            switch (rotation & 3)
            {
                case 1:
                    x = oy;
                    y = -ox;
                    break;
                case 2:
                    x = -ox;
                    y = -oy;
                    break;
                case 3:
                    x = -oy;
                    y = ox;
                    break;
                default:
                    break;
            }
        }

        Vertex MakeVertex(int32_t lx, int32_t ly, int32_t z, int32_t rotation)
        {
            int32_t dx = lx - kHalfTile, dy = ly - kHalfTile;
            RotateOffset(rotation, dx, dy);
            return { lx, ly, z, dy - dx, ((dx + dy) >> 1) - z };
        }

        // Twice the signed area of the triangle (a, b, c), screen coordinates with a common scale.
        int64_t Edge(int64_t ax, int64_t ay, int64_t bx, int64_t by, int64_t px, int64_t py)
        {
            return (bx - ax) * (py - ay) - (by - ay) * (px - ax);
        }

        // Top-left fill rule for a triangle with positive area under the Edge convention above.
        bool IsTopLeft(int64_t ax, int64_t ay, int64_t bx, int64_t by)
        {
            const int64_t dx = bx - ax, dy = by - ay;
            return (dy < 0) || (dy == 0 && dx > 0);
        }

        Quadrant AcrossNorthSouth(Quadrant q)
        {
            switch (q)
            {
                case Quadrant::northEast:
                    return Quadrant::northWest;
                case Quadrant::northWest:
                    return Quadrant::northEast;
                case Quadrant::southEast:
                    return Quadrant::southWest;
                default:
                    return Quadrant::southEast;
            }
        }

        Quadrant AcrossEastWest(Quadrant q)
        {
            switch (q)
            {
                case Quadrant::northEast:
                    return Quadrant::southEast;
                case Quadrant::southEast:
                    return Quadrant::northEast;
                case Quadrant::northWest:
                    return Quadrant::southWest;
                default:
                    return Quadrant::northWest;
            }
        }

        // Fixed noise in 0..255 for a position inside the tile, so the dithering never flickers.
        uint32_t DitherNoise(int32_t x, int32_t y)
        {
            uint32_t h = static_cast<uint32_t>(x) * 374761393u + static_cast<uint32_t>(y) * 668265263u;
            h = (h ^ (h >> 13)) * 1274126177u;
            return (h ^ (h >> 16)) & 0xFF;
        }

        // The texture of a higher priority surface bleeding over `own` at this point, or null. Positions and distances
        // are in 1/256 tile units.
        const TerrainTexture* PickBleed(
            const TerrainTexture* own, Quadrant quadrant, int64_t lx, int64_t ly, const std::array<const TerrainTexture*, 4>& quads,
            const TerrainSmoothing& smoothing)
        {
            const int64_t band = static_cast<int64_t>(smoothing.bandWidth) * 256;
            struct Candidate
            {
                const TerrainTexture* texture;
                int64_t distance;
            };
            const int64_t full = int64_t{ kTile } * 256;
            const int64_t edgeDistance[4] = { full - lx, ly, lx, full - ly }; // by quadrant
            const Candidate candidates[3] = {
                { quads[static_cast<size_t>(AcrossNorthSouth(quadrant))], (std::abs(lx - ly) * 181) >> 8 },
                { quads[static_cast<size_t>(AcrossEastWest(quadrant))], (std::abs(lx + ly - full) * 181) >> 8 },
                { smoothing.edgeNeighbours[static_cast<size_t>(quadrant)], edgeDistance[static_cast<size_t>(quadrant)] },
            };
            const uint32_t noise = DitherNoise(static_cast<int32_t>(lx >> 8), static_cast<int32_t>(ly >> 8));
            const TerrainTexture* best = nullptr;
            for (const auto& c : candidates)
            {
                if (c.texture == nullptr || c.texture == own || c.texture->smoothingPriority <= own->smoothingPriority
                    || c.distance >= band)
                    continue;
                // At most half of the pixels on the boundary, fading to none at the end of the band.
                const int64_t threshold = ((band - c.distance) * 128) / band;
                if (static_cast<int64_t>(noise) < threshold
                    && (best == nullptr || c.texture->smoothingPriority > best->smoothingPriority))
                    best = c.texture;
            }
            return best;
        }

        int32_t FloorDiv(int64_t a, int64_t b)
        {
            int64_t q = a / b;
            if ((a % b != 0) && ((a < 0) != (b < 0)))
                q--;
            return static_cast<int32_t>(q);
        }
        int32_t CeilDiv(int64_t a, int64_t b)
        {
            return -FloorDiv(-a, b);
        }
    } // namespace

    TerrainTexture TerrainTexture::Create(int32_t size, const std::vector<uint8_t>& pixels)
    {
        assert(size > 0 && (size & (size - 1)) == 0);
        assert(pixels.size() == static_cast<size_t>(size) * static_cast<size_t>(size));
        TerrainTexture tex;
        tex.size = size;
        tex.mips.push_back(pixels);
        for (int32_t level = 1; level < kTerrainZoomLevels && (size >> level) >= 1; level++)
        {
            const int32_t prev = size >> (level - 1);
            const int32_t cur = size >> level;
            const auto& src = tex.mips.back();
            std::vector<uint8_t> dst(static_cast<size_t>(cur) * static_cast<size_t>(cur));
            for (int32_t y = 0; y < cur; y++)
            {
                for (int32_t x = 0; x < cur; x++)
                {
                    // Most frequent colour of the 2x2 block; ties keep the first one.
                    const uint8_t c[4] = { src[(2 * y) * prev + 2 * x], src[(2 * y) * prev + 2 * x + 1],
                                           src[(2 * y + 1) * prev + 2 * x], src[(2 * y + 1) * prev + 2 * x + 1] };
                    int best = 0, bestCount = 0;
                    for (int i = 0; i < 4; i++)
                    {
                        int count = 0;
                        for (int j = 0; j < 4; j++)
                            count += c[i] == c[j];
                        if (count > bestCount)
                        {
                            bestCount = count;
                            best = i;
                        }
                    }
                    dst[static_cast<size_t>(y) * cur + x] = c[best];
                }
            }
            tex.mips.push_back(std::move(dst));
        }
        return tex;
    }

    int32_t TerrainTexture::MipSize(int32_t mip) const
    {
        return std::max(1, size >> mip);
    }

    uint8_t TerrainTexture::Sample(int32_t mip, int32_t u, int32_t v) const
    {
        mip = std::min<int32_t>(mip, static_cast<int32_t>(mips.size()) - 1);
        const int32_t s = MipSize(mip);
        u &= s - 1;
        v &= s - 1;
        return mips[static_cast<size_t>(mip)][static_cast<size_t>(v) * s + u];
    }

    TerrainShadeTable TerrainShadeTable::Identity()
    {
        TerrainShadeTable t;
        t.table.resize(static_cast<size_t>(kTerrainShadeBands) * 256);
        for (int32_t b = 0; b < kTerrainShadeBands; b++)
            for (int32_t i = 0; i < 256; i++)
                t.table[static_cast<size_t>(b) * 256 + i] = static_cast<uint8_t>(i);
        return t;
    }

    TerrainShadeTable TerrainShadeTable::FromPalette(const std::array<std::array<uint8_t, 3>, 256>& rgb)
    {
        // Brightness factor per band in 1/16: darker, dark, neutral, light, lighter.
        constexpr int32_t kFactor[kTerrainShadeBands] = { 11, 13, 16, 19, 22 };
        auto t = Identity();
        for (int32_t b = 0; b < kTerrainShadeBands; b++)
        {
            if (b == kTerrainNeutralShadeBand)
                continue;
            for (int32_t i = 1; i < 256; i++)
            {
                int32_t target[3];
                for (int32_t c = 0; c < 3; c++)
                    target[c] = std::min(255, rgb[static_cast<size_t>(i)][static_cast<size_t>(c)] * kFactor[b] / 16);
                int32_t bestIndex = i, bestDist = INT32_MAX;
                for (int32_t j = 1; j < 256; j++)
                {
                    int32_t d = 0;
                    for (int32_t c = 0; c < 3; c++)
                    {
                        const int32_t diff = target[c] - rgb[static_cast<size_t>(j)][static_cast<size_t>(c)];
                        d += diff * diff;
                    }
                    if (d < bestDist)
                    {
                        bestDist = d;
                        bestIndex = j;
                    }
                }
                t.table[static_cast<size_t>(b) * 256 + i] = static_cast<uint8_t>(bestIndex);
            }
        }
        return t;
    }

    int32_t GetTerrainShadeBand(const Vector3Fixed& normal, int32_t rotation)
    {
        int32_t nx = normal.x, ny = normal.y;
        RotateOffset(rotation, nx, ny);
        // Light direction in rotated world axes (16.16): from the screen's top-left, above the ground. A face tilted
        // towards the light is brighter than a flat one.
        constexpr int64_t kLightX = 13107; // 0.2
        constexpr int64_t kLightY = 30146; // 0.46
        constexpr int64_t kLightZ = 45875; // 0.7
        const int64_t dot = (-static_cast<int64_t>(nx) * kLightX - static_cast<int64_t>(ny) * kLightY
                             + static_cast<int64_t>(normal.z) * kLightZ)
            >> 16;
        const int64_t delta = dot - kLightZ; // difference with a flat face
        // One band per quarter of a unit vector, rounded to nearest.
        const int64_t scaled = delta * 4;
        const int64_t rounded = (scaled >= 0 ? scaled + kFixedOne / 2 : scaled - kFixedOne / 2) / kFixedOne;
        const int64_t band = kTerrainNeutralShadeBand + std::clamp<int64_t>(rounded, -kTerrainNeutralShadeBand, 2);
        return static_cast<int32_t>(band);
    }

    TerrainSprite GenerateTerrainSprite(
        const TerrainShape& shape, const std::array<const TerrainTexture*, 4>& quadrants, const TerrainShadeTable& shades,
        int32_t rotation, int32_t zoom, const TerrainSmoothing& smoothing)
    {
        assert(zoom >= 0 && zoom < kTerrainZoomLevels);
        rotation &= 3;

        const int32_t hN = shape.north * kQuarterStepZ, hE = shape.east * kQuarterStepZ;
        const int32_t hS = shape.south * kQuarterStepZ, hW = shape.west * kQuarterStepZ;
        const Vertex vS = MakeVertex(0, 0, hS, rotation);
        const Vertex vE = MakeVertex(kTile, 0, hE, rotation);
        const Vertex vN = MakeVertex(kTile, kTile, hN, rotation);
        const Vertex vW = MakeVertex(0, kTile, hW, rotation);

        const bool northSouth = GetSplitDiagonal(shape) == SplitDiagonal::northSouth;
        const std::array<std::array<Vertex, 3>, 2> tris = {{
            northSouth ? std::array<Vertex, 3>{ vS, vE, vN } : std::array<Vertex, 3>{ vS, vE, vW },
            northSouth ? std::array<Vertex, 3>{ vS, vN, vW } : std::array<Vertex, 3>{ vE, vN, vW },
        }};

        // Bounds in pixels of the zoom level.
        const int64_t step = int64_t{ 1 } << zoom;
        int64_t minX = INT64_MAX, minY = INT64_MAX, maxX = INT64_MIN, maxY = INT64_MIN;
        for (const auto& tri : tris)
            for (const auto& v : tri)
            {
                minX = std::min(minX, v.sx);
                maxX = std::max(maxX, v.sx);
                minY = std::min(minY, v.sy);
                maxY = std::max(maxY, v.sy);
            }
        TerrainSprite sprite;
        sprite.xOffset = FloorDiv(minX, step);
        sprite.yOffset = FloorDiv(minY, step);
        sprite.width = CeilDiv(maxX, step) - sprite.xOffset;
        sprite.height = CeilDiv(maxY, step) - sprite.yOffset;
        sprite.pixels.assign(static_cast<size_t>(sprite.width) * static_cast<size_t>(sprite.height), kTerrainTransparent);

        // A point well inside each triangle, to look up its plane (same triangle numbering as GetTriangleAt).
        const int32_t insideX[2] = { northSouth ? 24 : 8, northSouth ? 8 : 24 };
        const int32_t insideY[2] = { 8, 24 };

        for (int32_t ti = 0; ti < 2; ti++)
        {
            Vertex a = tris[ti][0], b = tris[ti][1], c = tris[ti][2];
            // Work in doubled screen coordinates so that pixel centres are integers.
            auto area2 = [&](const Vertex& p, const Vertex& q, const Vertex& r) {
                return Edge(2 * p.sx, 2 * p.sy, 2 * q.sx, 2 * q.sy, 2 * r.sx, 2 * r.sy);
            };
            int64_t area = area2(a, b, c);
            if (area == 0)
                continue; // seen edge-on
            if (area < 0)
            {
                std::swap(b, c);
                area = -area;
            }

            // Shade from the plane of this triangle.
            assert(GetTriangleAt(shape, insideX[ti], insideY[ti]) == ti);
            const int32_t band = GetTerrainShadeBand(GetSurfaceNormalAt(shape, insideX[ti], insideY[ti]), rotation);

            const int64_t tminX = std::min({ a.sx, b.sx, c.sx }), tmaxX = std::max({ a.sx, b.sx, c.sx });
            const int64_t tminY = std::min({ a.sy, b.sy, c.sy }), tmaxY = std::max({ a.sy, b.sy, c.sy });
            const int32_t px0 = FloorDiv(tminX, step), px1 = CeilDiv(tmaxX, step);
            const int32_t py0 = FloorDiv(tminY, step), py1 = CeilDiv(tmaxY, step);

            const int64_t bias0 = IsTopLeft(2 * b.sx, 2 * b.sy, 2 * c.sx, 2 * c.sy) ? 0 : -1;
            const int64_t bias1 = IsTopLeft(2 * c.sx, 2 * c.sy, 2 * a.sx, 2 * a.sy) ? 0 : -1;
            const int64_t bias2 = IsTopLeft(2 * a.sx, 2 * a.sy, 2 * b.sx, 2 * b.sy) ? 0 : -1;

            for (int32_t py = py0; py < py1; py++)
            {
                for (int32_t px = px0; px < px1; px++)
                {
                    // Pixel centre in doubled 1x coordinates.
                    const int64_t cx = (2 * static_cast<int64_t>(px) + 1) * step;
                    const int64_t cy = (2 * static_cast<int64_t>(py) + 1) * step;
                    const int64_t w0 = Edge(2 * b.sx, 2 * b.sy, 2 * c.sx, 2 * c.sy, cx, cy);
                    const int64_t w1 = Edge(2 * c.sx, 2 * c.sy, 2 * a.sx, 2 * a.sy, cx, cy);
                    const int64_t w2 = Edge(2 * a.sx, 2 * a.sy, 2 * b.sx, 2 * b.sy, cx, cy);
                    if (w0 + bias0 < 0 || w1 + bias1 < 0 || w2 + bias2 < 0)
                        continue;

                    // Interpolated local position, in 1/256 tile units.
                    const int64_t lx = (w0 * a.lx + w1 * b.lx + w2 * c.lx) * 256 / area;
                    const int64_t ly = (w0 * a.ly + w1 * b.ly + w2 * c.ly) * 256 / area;
                    const int32_t ixl = static_cast<int32_t>(std::clamp<int64_t>(lx >> 8, 0, kTile - 1));
                    const int32_t iyl = static_cast<int32_t>(std::clamp<int64_t>(ly >> 8, 0, kTile - 1));
                    const auto quadrant = GetQuadrantAt(ixl, iyl);
                    const TerrainTexture* tex = quadrants[static_cast<size_t>(quadrant)];
                    if (smoothing.enabled)
                    {
                        if (const auto* bleed = PickBleed(tex, quadrant, lx, ly, quadrants, smoothing))
                            tex = bleed;
                    }

                    // Texture space = tile space, sampled in the mip matching the zoom.
                    const int32_t mipSize = tex->MipSize(zoom);
                    const int32_t u = static_cast<int32_t>(std::clamp<int64_t>(lx * mipSize / (kTile * 256), 0, mipSize - 1));
                    const int32_t v = static_cast<int32_t>(std::clamp<int64_t>(ly * mipSize / (kTile * 256), 0, mipSize - 1));
                    const uint8_t colour = tex->Sample(zoom, u, v);

                    const int32_t ox = px - sprite.xOffset, oy = py - sprite.yOffset;
                    if (ox < 0 || oy < 0 || ox >= sprite.width || oy >= sprite.height)
                        continue;
                    sprite.pixels[static_cast<size_t>(oy) * sprite.width + ox] = shades.Shade(band, colour);
                }
            }
        }
        return sprite;
    }

    bool IsEdgeFacingViewer(Quadrant edge, int32_t rotation)
    {
        int32_t nx = 0, ny = 0;
        switch (edge)
        {
            case Quadrant::northEast:
                nx = 1;
                break;
            case Quadrant::southEast:
                ny = -1;
                break;
            case Quadrant::southWest:
                nx = -1;
                break;
            default:
                ny = 1;
                break;
        }
        RotateOffset(rotation, nx, ny);
        return nx + ny > 0;
    }

    TerrainSprite GenerateTerrainEdgeSprite(
        const TerrainTexture& texture, const TerrainEdgeFace& face, const TerrainShadeTable& shades, int32_t rotation,
        int32_t zoom)
    {
        assert(zoom >= 0 && zoom < kTerrainZoomLevels);
        rotation &= 3;

        // Ends of the edge in tile coordinates.
        int32_t ax = 0, ay = 0, bx = 0, by = 0, nx = 0, ny = 0;
        switch (face.edge)
        {
            case Quadrant::northEast:
                ax = bx = kTile;
                ay = 0;
                by = kTile;
                nx = 1;
                break;
            case Quadrant::southEast:
                ay = by = 0;
                ax = 0;
                bx = kTile;
                ny = -1;
                break;
            case Quadrant::southWest:
                ax = bx = 0;
                ay = 0;
                by = kTile;
                nx = -1;
                break;
            default:
                ay = by = kTile;
                ax = 0;
                bx = kTile;
                ny = 1;
                break;
        }

        struct FaceVertex
        {
            Vertex v;
            int32_t u; // 0..32 along the edge
        };
        const FaceVertex corners[4] = {
            { MakeVertex(ax, ay, face.topA, rotation), 0 },
            { MakeVertex(bx, by, face.topB, rotation), kTile },
            { MakeVertex(bx, by, face.bottomB, rotation), kTile },
            { MakeVertex(ax, ay, face.bottomA, rotation), 0 },
        };
        const int32_t triangles[2][3] = { { 0, 1, 2 }, { 0, 2, 3 } };

        const int64_t step = int64_t{ 1 } << zoom;
        int64_t minX = INT64_MAX, minY = INT64_MAX, maxX = INT64_MIN, maxY = INT64_MIN;
        for (const auto& c : corners)
        {
            minX = std::min(minX, c.v.sx);
            maxX = std::max(maxX, c.v.sx);
            minY = std::min(minY, c.v.sy);
            maxY = std::max(maxY, c.v.sy);
        }
        TerrainSprite sprite;
        sprite.xOffset = FloorDiv(minX, step);
        sprite.yOffset = FloorDiv(minY, step);
        sprite.width = CeilDiv(maxX, step) - sprite.xOffset;
        sprite.height = CeilDiv(maxY, step) - sprite.yOffset;
        sprite.pixels.assign(static_cast<size_t>(sprite.width) * static_cast<size_t>(sprite.height), kTerrainTransparent);

        // Vertical faces: shade from the horizontal normal.
        const int32_t band = GetTerrainShadeBand({ nx * kFixedOne, ny * kFixedOne, 0 }, rotation);
        const int32_t mipSize = texture.MipSize(zoom);

        for (const auto& tri : triangles)
        {
            const FaceVertex* a = &corners[tri[0]];
            const FaceVertex* b = &corners[tri[1]];
            const FaceVertex* c = &corners[tri[2]];
            int64_t area = Edge(2 * a->v.sx, 2 * a->v.sy, 2 * b->v.sx, 2 * b->v.sy, 2 * c->v.sx, 2 * c->v.sy);
            if (area == 0)
                continue;
            if (area < 0)
            {
                std::swap(b, c);
                area = -area;
            }
            const int64_t bias0 = IsTopLeft(2 * b->v.sx, 2 * b->v.sy, 2 * c->v.sx, 2 * c->v.sy) ? 0 : -1;
            const int64_t bias1 = IsTopLeft(2 * c->v.sx, 2 * c->v.sy, 2 * a->v.sx, 2 * a->v.sy) ? 0 : -1;
            const int64_t bias2 = IsTopLeft(2 * a->v.sx, 2 * a->v.sy, 2 * b->v.sx, 2 * b->v.sy) ? 0 : -1;

            const int64_t tminX = std::min({ a->v.sx, b->v.sx, c->v.sx }), tmaxX = std::max({ a->v.sx, b->v.sx, c->v.sx });
            const int64_t tminY = std::min({ a->v.sy, b->v.sy, c->v.sy }), tmaxY = std::max({ a->v.sy, b->v.sy, c->v.sy });
            for (int32_t py = FloorDiv(tminY, step); py < CeilDiv(tmaxY, step); py++)
            {
                for (int32_t px = FloorDiv(tminX, step); px < CeilDiv(tmaxX, step); px++)
                {
                    const int64_t cx = (2 * static_cast<int64_t>(px) + 1) * step;
                    const int64_t cy = (2 * static_cast<int64_t>(py) + 1) * step;
                    const int64_t w0 = Edge(2 * b->v.sx, 2 * b->v.sy, 2 * c->v.sx, 2 * c->v.sy, cx, cy);
                    const int64_t w1 = Edge(2 * c->v.sx, 2 * c->v.sy, 2 * a->v.sx, 2 * a->v.sy, cx, cy);
                    const int64_t w2 = Edge(2 * a->v.sx, 2 * a->v.sy, 2 * b->v.sx, 2 * b->v.sy, cx, cy);
                    if (w0 + bias0 < 0 || w1 + bias1 < 0 || w2 + bias2 < 0)
                        continue;

                    const int64_t u256 = (w0 * a->u + w1 * b->u + w2 * c->u) * 256 / area;
                    const int64_t z256 = (w0 * a->v.z + w1 * b->v.z + w2 * c->v.z) * 256 / area;
                    const int32_t col = static_cast<int32_t>(std::clamp<int64_t>(u256 * mipSize / (kTile * 256), 0, mipSize - 1));
                    // Rows count down from the top of the world so the texture lines up between tiles.
                    const int64_t zAbs = face.baseZ + (z256 >> 8);
                    const int32_t row = static_cast<int32_t>(((-(zAbs >> zoom)) % mipSize + mipSize) % mipSize);
                    const int32_t ox = px - sprite.xOffset, oy = py - sprite.yOffset;
                    if (ox < 0 || oy < 0 || ox >= sprite.width || oy >= sprite.height)
                        continue;
                    sprite.pixels[static_cast<size_t>(oy) * sprite.width + ox] = shades.Shade(band, texture.Sample(zoom, col, row));
                }
            }
        }
        return sprite;
    }

    TerrainSprite GenerateTerrainWaterSprite(
        const TerrainShape& shape, int32_t terrainBaseZ, WaterPattern pattern, int32_t rotation, int32_t zoom)
    {
        assert(zoom >= 0 && zoom < kTerrainZoomLevels);
        rotation &= 3;

        const Vertex vS = MakeVertex(0, 0, 0, rotation), vE = MakeVertex(kTile, 0, 0, rotation);
        const Vertex vN = MakeVertex(kTile, kTile, 0, rotation), vW = MakeVertex(0, kTile, 0, rotation);
        const Vertex tris[2][3] = { { vS, vE, vN }, { vS, vN, vW } };

        const int64_t step = int64_t{ 1 } << zoom;
        int64_t minX = INT64_MAX, minY = INT64_MAX, maxX = INT64_MIN, maxY = INT64_MIN;
        for (const auto& v : { vS, vE, vN, vW })
        {
            minX = std::min(minX, v.sx);
            maxX = std::max(maxX, v.sx);
            minY = std::min(minY, v.sy);
            maxY = std::max(maxY, v.sy);
        }
        TerrainSprite sprite;
        sprite.xOffset = FloorDiv(minX, step);
        sprite.yOffset = FloorDiv(minY, step);
        sprite.width = CeilDiv(maxX, step) - sprite.xOffset;
        sprite.height = CeilDiv(maxY, step) - sprite.yOffset;
        sprite.pixels.assign(static_cast<size_t>(sprite.width) * static_cast<size_t>(sprite.height), kTerrainTransparent);

        constexpr uint8_t kWaves = 230, kWaveCount = 5, kSparkles = 235, kSparkleCount = 5;
        for (const auto& tri : tris)
        {
            const Vertex &a = tri[0], &b = tri[1], &c = tri[2];
            int64_t area = Edge(2 * a.sx, 2 * a.sy, 2 * b.sx, 2 * b.sy, 2 * c.sx, 2 * c.sy);
            const Vertex* pb = &b;
            const Vertex* pc = &c;
            if (area == 0)
                continue;
            if (area < 0)
            {
                std::swap(pb, pc);
                area = -area;
            }
            const int64_t bias0 = IsTopLeft(2 * pb->sx, 2 * pb->sy, 2 * pc->sx, 2 * pc->sy) ? 0 : -1;
            const int64_t bias1 = IsTopLeft(2 * pc->sx, 2 * pc->sy, 2 * a.sx, 2 * a.sy) ? 0 : -1;
            const int64_t bias2 = IsTopLeft(2 * a.sx, 2 * a.sy, 2 * pb->sx, 2 * pb->sy) ? 0 : -1;
            const int64_t tminX = std::min({ a.sx, pb->sx, pc->sx }), tmaxX = std::max({ a.sx, pb->sx, pc->sx });
            const int64_t tminY = std::min({ a.sy, pb->sy, pc->sy }), tmaxY = std::max({ a.sy, pb->sy, pc->sy });
            for (int32_t py = FloorDiv(tminY, step); py < CeilDiv(tmaxY, step); py++)
            {
                for (int32_t px = FloorDiv(tminX, step); px < CeilDiv(tmaxX, step); px++)
                {
                    const int64_t cx = (2 * static_cast<int64_t>(px) + 1) * step;
                    const int64_t cy = (2 * static_cast<int64_t>(py) + 1) * step;
                    const int64_t w0 = Edge(2 * pb->sx, 2 * pb->sy, 2 * pc->sx, 2 * pc->sy, cx, cy);
                    const int64_t w1 = Edge(2 * pc->sx, 2 * pc->sy, 2 * a.sx, 2 * a.sy, cx, cy);
                    const int64_t w2 = Edge(2 * a.sx, 2 * a.sy, 2 * pb->sx, 2 * pb->sy, cx, cy);
                    if (w0 + bias0 < 0 || w1 + bias1 < 0 || w2 + bias2 < 0)
                        continue;

                    const int64_t lx = (w0 * a.lx + w1 * pb->lx + w2 * pc->lx) * 256 / area;
                    const int64_t ly = (w0 * a.ly + w1 * pb->ly + w2 * pc->ly) * 256 / area;
                    const int32_t ix = static_cast<int32_t>(std::clamp<int64_t>(lx >> 8, 0, kTile - 1));
                    const int32_t iy = static_cast<int32_t>(std::clamp<int64_t>(ly >> 8, 0, kTile - 1));
                    // Water only shows where the terrain is below its level.
                    if (GetSurfaceZAt(terrainBaseZ, shape, ix, iy) >= 0)
                        continue;

                    uint8_t colour = 1; // mask: any opaque index
                    if (pattern != WaterPattern::mask)
                    {
                        // Diagonal wave bands with a fixed hash for scattered sparkles; same pattern on every tile.
                        const uint32_t noise = DitherNoise(ix, iy);
                        const bool sparkle = noise < 6;
                        const bool wave = ((ix + 2 * iy) / 2 + (noise >> 6)) % 4 == 0;
                        if (sparkle)
                            colour = static_cast<uint8_t>(kSparkles + noise % kSparkleCount);
                        else if (pattern == WaterPattern::opaque)
                            colour = static_cast<uint8_t>(kWaves + ((ix + iy) / 4 + (wave ? 2 : 0)) % kWaveCount);
                        else if (wave && noise < 96)
                            colour = static_cast<uint8_t>(kWaves + noise % kWaveCount);
                        else
                            continue;
                    }
                    const int32_t ox = px - sprite.xOffset, oy = py - sprite.yOffset;
                    if (ox < 0 || oy < 0 || ox >= sprite.width || oy >= sprite.height)
                        continue;
                    sprite.pixels[static_cast<size_t>(oy) * sprite.width + ox] = colour;
                }
            }
        }
        return sprite;
    }
} // namespace OpenRCT2::Drawing
