/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Rasteriser.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace OpenRCT2::Model
{
    namespace
    {
        constexpr float kNoDepth = -std::numeric_limits<float>::infinity();

        Vec3 sub(const Vec3& a, const Vec3& b)
        {
            return { a[0] - b[0], a[1] - b[1], a[2] - b[2] };
        }

        Vec3 cross(const Vec3& a, const Vec3& b)
        {
            return { a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0] };
        }

        float dot(const Vec3& a, const Vec3& b)
        {
            return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        }

        Vec3 normalised(const Vec3& v)
        {
            const float len = std::sqrt(dot(v, v));
            return len > 1e-12f ? Vec3{ v[0] / len, v[1] / len, v[2] / len } : Vec3{ 0, 0, 1 };
        }

        Vec3 transformPoint(const Mat4& m, const Vec3& p)
        {
            return { m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12], m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13],
                     m[2] * p[0] + m[6] * p[1] + m[10] * p[2] + m[14] };
        }

        Vec3 transformDirection(const Mat4& m, const Vec3& p)
        {
            return { m[0] * p[0] + m[4] * p[1] + m[8] * p[2], m[1] * p[0] + m[5] * p[1] + m[9] * p[2],
                     m[2] * p[0] + m[6] * p[1] + m[10] * p[2] };
        }

        Vec3 mapRotate(const Vec3& v, const Orientation& o)
        {
            // Roll about x, then pitch about y, then yaw about z.
            float x = v[0], y = v[1], z = v[2];
            float c = std::cos(o.roll), s = std::sin(o.roll);
            float y1 = y * c - z * s, z1 = y * s + z * c;
            y = y1;
            z = z1;
            c = std::cos(o.pitch);
            s = std::sin(o.pitch);
            float x1 = x * c + z * s;
            z1 = -x * s + z * c;
            x = x1;
            z = z1;
            c = std::cos(o.yaw);
            s = std::sin(o.yaw);
            x1 = x * c - y * s;
            y1 = x * s + y * c;
            return { x1, y1, z };
        }

        // Vertex positions and normals of a primitive in model space, skinned when the node has a skin.
        std::vector<std::pair<Vec3, Vec3>> posePrimitive(const Primitive& prim, const Mat4& nodeWorld, const std::vector<Mat4>& skin)
        {
            std::vector<std::pair<Vec3, Vec3>> posed(prim.vertices.size());
            for (size_t i = 0; i < prim.vertices.size(); i++)
            {
                const auto& vtx = prim.vertices[i];
                if (!skin.empty())
                {
                    Vec3 p{ 0, 0, 0 }, n{ 0, 0, 0 };
                    for (size_t c = 0; c < 4; c++)
                    {
                        if (vtx.weights[c] <= 0 || vtx.joints[c] >= skin.size())
                            continue;
                        const auto sp = transformPoint(skin[vtx.joints[c]], vtx.position);
                        const auto sn = transformDirection(skin[vtx.joints[c]], vtx.normal);
                        for (size_t k = 0; k < 3; k++)
                        {
                            p[k] += sp[k] * vtx.weights[c];
                            n[k] += sn[k] * vtx.weights[c];
                        }
                    }
                    posed[i] = { p, n };
                }
                else
                    posed[i] = { transformPoint(nodeWorld, vtx.position), transformDirection(nodeWorld, vtx.normal) };
            }
            return posed;
        }

        struct ScreenVertex
        {
            float x, y, depth;
            Vec3 normal; // map space
            std::array<float, 4> colour;
        };
    } // namespace

    Vec3 modelToMap(const Vec3& m)
    {
        return { m[0], -m[2], m[1] };
    }

    std::array<float, 2> project(const Vec3& p, float pixelsPerUnit)
    {
        return { (p[0] - p[1]) * pixelsPerUnit, ((p[0] + p[1]) * 0.5f - p[2]) * pixelsPerUnit };
    }

    size_t RenderedImage::coveredPixels() const
    {
        return static_cast<size_t>(
            std::count_if(rgba.begin(), rgba.end(), [](const std::array<uint8_t, 4>& p) { return p[3] != 0; }));
    }

    RenderedImage render(const GlbModel& model, const RenderSettings& settings)
    {
        const Clip* clip = nullptr;
        if (settings.clipIndex >= 0 && static_cast<size_t>(settings.clipIndex) < model.clips.size())
            clip = &model.clips[settings.clipIndex];
        const auto world = model.worldTransforms(clip, settings.time);

        struct Triangle
        {
            ScreenVertex v[3];
            Vec3 faceNormal;
        };
        std::vector<Triangle> triangles;

        auto finish = [&](const Vec3& modelPos, const Vec3& modelNormal, const std::array<float, 4>& colour) {
            auto mapPos = mapRotate(modelToMap(modelPos), settings.orientation);
            for (auto& c : mapPos)
                c *= settings.unitsPerMetre;
            const auto proj = project(mapPos, settings.pixelsPerUnit);
            ScreenVertex v;
            v.x = proj[0];
            v.y = proj[1];
            v.depth = (mapPos[0] + mapPos[1] + mapPos[2]) * settings.pixelsPerUnit;
            v.normal = normalised(mapRotate(modelToMap(modelNormal), settings.orientation));
            v.colour = colour;
            return v;
        };

        for (size_t nodeIndex = 0; nodeIndex < model.nodes.size(); nodeIndex++)
        {
            const auto& node = model.nodes[nodeIndex];
            if (node.mesh < 0)
                continue;
            std::vector<Mat4> skin;
            if (node.skin >= 0)
                skin = model.skinMatrices(node.skin, world);

            for (auto primIndex : model.meshPrimitives[node.mesh])
            {
                const auto& prim = model.primitives[primIndex];
                std::array<float, 4> colour{ 0.8f, 0.8f, 0.8f, 1.0f };
                if (prim.material >= 0)
                {
                    colour = model.materials[prim.material].baseColour;
                    if (static_cast<size_t>(prim.material) < settings.materialColours.size()
                        && settings.materialColours[prim.material].has_value())
                        colour = *settings.materialColours[prim.material];
                }

                const auto posed = posePrimitive(prim, world[nodeIndex], skin);

                for (size_t t = 0; t + 2 < prim.indices.size(); t += 3)
                {
                    Triangle tri;
                    std::array<Vec3, 3> mp;
                    for (size_t k = 0; k < 3; k++)
                    {
                        const auto& pv = posed[prim.indices[t + k]];
                        tri.v[k] = finish(pv.first, pv.second, colour);
                        auto p = mapRotate(modelToMap(pv.first), settings.orientation);
                        for (auto& c : p)
                            c *= settings.unitsPerMetre;
                        mp[k] = p;
                    }
                    tri.faceNormal = normalised(cross(sub(mp[1], mp[0]), sub(mp[2], mp[0])));
                    triangles.push_back(tri);
                }
            }
        }

        const auto light = normalised(settings.lightDirection);
        RenderedImage image;
        if (triangles.empty())
            return image;

        float minX = std::numeric_limits<float>::max(), minY = minX;
        float maxX = std::numeric_limits<float>::lowest(), maxY = maxX;
        for (const auto& tri : triangles)
            for (const auto& v : tri.v)
            {
                minX = std::min(minX, v.x);
                maxX = std::max(maxX, v.x);
                minY = std::min(minY, v.y);
                maxY = std::max(maxY, v.y);
            }
        const int32_t x0 = static_cast<int32_t>(std::floor(minX)) - 1;
        const int32_t y0 = static_cast<int32_t>(std::floor(minY)) - 1;
        image.width = static_cast<int32_t>(std::ceil(maxX)) + 1 - x0 + 1;
        image.height = static_cast<int32_t>(std::ceil(maxY)) + 1 - y0 + 1;
        image.originX = -x0;
        image.originY = -y0;
        image.rgba.assign(static_cast<size_t>(image.width) * image.height, { 0, 0, 0, 0 });
        image.depth.assign(image.rgba.size(), kNoDepth);

        const Vec3 towardsViewer = normalised({ 1, 1, 1 });
        for (const auto& tri : triangles)
        {
            // Back faces point away from the viewer.
            if (dot(tri.faceNormal, towardsViewer) <= 0)
                continue;

            const float ax = tri.v[0].x - x0, ay = tri.v[0].y - y0;
            const float bx = tri.v[1].x - x0, by = tri.v[1].y - y0;
            const float cx = tri.v[2].x - x0, cy = tri.v[2].y - y0;
            const float area = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay);
            if (std::fabs(area) < 1e-6f)
                continue;
            const int32_t px0 = std::max(0, static_cast<int32_t>(std::floor(std::min({ ax, bx, cx }))));
            const int32_t px1 = std::min(image.width - 1, static_cast<int32_t>(std::ceil(std::max({ ax, bx, cx }))));
            const int32_t py0 = std::max(0, static_cast<int32_t>(std::floor(std::min({ ay, by, cy }))));
            const int32_t py1 = std::min(image.height - 1, static_cast<int32_t>(std::ceil(std::max({ ay, by, cy }))));
            for (int32_t py = py0; py <= py1; py++)
            {
                for (int32_t px = px0; px <= px1; px++)
                {
                    const float sx = px + 0.5f, sy = py + 0.5f;
                    const float w0 = ((bx - sx) * (cy - sy) - (cx - sx) * (by - sy)) / area;
                    const float w1 = ((cx - sx) * (ay - sy) - (ax - sx) * (cy - sy)) / area;
                    const float w2 = 1.0f - w0 - w1;
                    constexpr float kEdge = -1e-4f;
                    if (w0 < kEdge || w1 < kEdge || w2 < kEdge)
                        continue;
                    const float depth = w0 * tri.v[0].depth + w1 * tri.v[1].depth + w2 * tri.v[2].depth;
                    const size_t index = static_cast<size_t>(py) * image.width + px;
                    if (depth <= image.depth[index])
                        continue;
                    image.depth[index] = depth;

                    Vec3 n{ 0, 0, 0 };
                    for (size_t k = 0; k < 3; k++)
                    {
                        const float w = k == 0 ? w0 : (k == 1 ? w1 : w2);
                        for (size_t c = 0; c < 3; c++)
                            n[c] += tri.v[k].normal[c] * w;
                    }
                    n = normalised(n);
                    // Flip interpolated normals that disagree with the face (bad source data).
                    if (dot(n, tri.faceNormal) < 0)
                        n = tri.faceNormal;
                    const float lit = std::min(1.0f, settings.ambient + (1.0f - settings.ambient) * std::max(0.0f, dot(n, light)));
                    std::array<float, 4> col{};
                    for (size_t c = 0; c < 4; c++)
                        col[c] = tri.v[0].colour[c];
                    auto to8 = [](float v) { return static_cast<uint8_t>(std::clamp(v, 0.0f, 1.0f) * 255.0f + 0.5f); };
                    image.rgba[index] = { to8(col[0] * lit), to8(col[1] * lit), to8(col[2] * lit), 255 };
                }
            }
        }
        return image;
    }

    PaletteImage quantise(const RenderedImage& image, const QuantiseSettings& settings)
    {
        PaletteImage out;
        out.width = image.width;
        out.height = image.height;
        out.originX = image.originX;
        out.originY = image.originY;
        out.pixels.assign(image.rgba.size(), 0);
        if (settings.palette.empty())
            return out;

        for (size_t i = 0; i < image.rgba.size(); i++)
        {
            const auto& p = image.rgba[i];
            if (p[3] == 0)
                continue;
            int32_t best = std::numeric_limits<int32_t>::max();
            uint8_t bestIndex = 0;
            for (const auto& [index, colour] : settings.palette)
            {
                const int32_t dr = p[0] - colour.r, dg = p[1] - colour.g, db = p[2] - colour.b;
                const int32_t distance = dr * dr + dg * dg + db * db;
                if (distance < best)
                {
                    best = distance;
                    bestIndex = index;
                }
            }
            out.pixels[i] = bestIndex;
        }

        if (settings.outlineIndex.has_value())
        {
            // Add a ring of transparent pixels next to drawn ones; the image has a 1 pixel margin by construction.
            const auto source = out.pixels;
            for (int32_t y = 0; y < out.height; y++)
                for (int32_t x = 0; x < out.width; x++)
                {
                    const size_t i = static_cast<size_t>(y) * out.width + x;
                    if (source[i] != 0)
                        continue;
                    bool near = false;
                    for (int32_t dy = -1; dy <= 1 && !near; dy++)
                        for (int32_t dx = -1; dx <= 1 && !near; dx++)
                        {
                            if ((dx == 0) == (dy == 0))
                                continue; // 4-neighbourhood
                            const int32_t nx = x + dx, ny = y + dy;
                            if (nx < 0 || ny < 0 || nx >= out.width || ny >= out.height)
                                continue;
                            near = source[static_cast<size_t>(ny) * out.width + nx] != 0;
                        }
                    if (near)
                        out.pixels[i] = *settings.outlineIndex;
                }
        }
        return out;
    }

    ModelBounds computeBounds(const GlbModel& model, const RenderSettings& settings)
    {
        const Clip* clip = nullptr;
        if (settings.clipIndex >= 0 && static_cast<size_t>(settings.clipIndex) < model.clips.size())
            clip = &model.clips[settings.clipIndex];
        const auto world = model.worldTransforms(clip, settings.time);

        ModelBounds bounds;
        for (size_t nodeIndex = 0; nodeIndex < model.nodes.size(); nodeIndex++)
        {
            const auto& node = model.nodes[nodeIndex];
            if (node.mesh < 0)
                continue;
            std::vector<Mat4> skin;
            if (node.skin >= 0)
                skin = model.skinMatrices(node.skin, world);
            for (auto primIndex : model.meshPrimitives[node.mesh])
            {
                for (const auto& [position, normal] : posePrimitive(model.primitives[primIndex], world[nodeIndex], skin))
                {
                    auto p = mapRotate(modelToMap(position), settings.orientation);
                    for (size_t k = 0; k < 3; k++)
                    {
                        p[k] *= settings.unitsPerMetre;
                        bounds.min[k] = std::min(bounds.min[k], p[k]);
                        bounds.max[k] = std::max(bounds.max[k], p[k]);
                    }
                    bounds.valid = true;
                }
            }
        }
        return bounds;
    }
} // namespace OpenRCT2::Model
