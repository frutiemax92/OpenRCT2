/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace OpenRCT2::Model
{
    // glTF 2.0 binary (.glb) loader (R3D-001). Supports skinned triangle meshes, materials, node hierarchies and
    // named animation clips; no cameras, lights, sparse accessors or external buffers.

    using Vec3 = std::array<float, 3>;
    using Quat = std::array<float, 4>; // x, y, z, w
    using Mat4 = std::array<float, 16>; // column-major, as in glTF

    struct Vertex
    {
        Vec3 position{};
        Vec3 normal{ 0, 0, 1 };
        std::array<float, 2> uv{};
        std::array<uint16_t, 4> joints{};
        std::array<float, 4> weights{ 1, 0, 0, 0 };
    };

    struct Material
    {
        std::string name;
        std::array<float, 4> baseColour{ 1, 1, 1, 1 };
        int32_t textureImage = -1; // index into GlbModel::images, or -1
    };

    struct Primitive
    {
        std::vector<Vertex> vertices;
        std::vector<uint32_t> indices; // triangle list
        int32_t material = -1;
    };

    struct Node
    {
        std::string name;
        int32_t parent = -1;
        std::vector<int32_t> children;
        int32_t mesh = -1;
        int32_t skin = -1;
        Vec3 translation{ 0, 0, 0 };
        Quat rotation{ 0, 0, 0, 1 };
        Vec3 scale{ 1, 1, 1 };
    };

    struct Skin
    {
        std::vector<int32_t> joints; // node indices
        std::vector<Mat4> inverseBind;
    };

    enum class ChannelPath : uint8_t
    {
        translation,
        rotation,
        scale,
    };

    struct Channel
    {
        int32_t node = -1;
        ChannelPath path = ChannelPath::translation;
        bool step = false; // STEP interpolation, otherwise LINEAR
        std::vector<float> times;
        std::vector<float> values; // 3 floats per key (translation, scale) or 4 (rotation)
    };

    struct Clip
    {
        std::string name;
        float duration = 0;
        std::vector<Channel> channels;
    };

    struct Image
    {
        std::string mimeType;
        std::vector<uint8_t> bytes; // still encoded (PNG/JPEG)
    };

    struct GlbModel
    {
        std::vector<Primitive> primitives; // flattened: nodes reference meshes through meshPrimitives
        std::vector<std::vector<int32_t>> meshPrimitives; // mesh index -> primitive indices
        std::vector<Material> materials;
        std::vector<Node> nodes;
        std::vector<int32_t> rootNodes;
        std::vector<Skin> skins;
        std::vector<Clip> clips;
        std::vector<Image> images;

        size_t triangleCount() const;
        const Clip* findClip(const std::string& name) const;
        // Local transform of a node, optionally overridden by an animated pose at `time` of `clip`.
        std::vector<Mat4> worldTransforms(const Clip* clip, float time) const;
        // Skin matrices (joint world * inverse bind) for a skin at the given pose.
        std::vector<Mat4> skinMatrices(int32_t skin, const std::vector<Mat4>& world) const;
    };

    struct LoadResult
    {
        bool ok = false;
        std::string error;
        GlbModel model;
    };

    // `maxTriangles` of 0 disables the budget check; otherwise models above 2 x maxTriangles are rejected (R3D-003).
    // `maxTextureSize` of 0 disables the texture size check (it needs the PNG header only).
    LoadResult loadGlb(std::span<const uint8_t> data, size_t maxTriangles = 0, uint32_t maxTextureSize = 0);

    Mat4 identity();
    Mat4 multiply(const Mat4& a, const Mat4& b);
    Mat4 composeTRS(const Vec3& t, const Quat& r, const Vec3& s);
} // namespace OpenRCT2::Model
