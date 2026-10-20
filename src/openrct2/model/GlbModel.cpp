/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "GlbModel.h"

#include "../core/Json.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace OpenRCT2::Model
{
    namespace
    {
        constexpr uint32_t kMagic = 0x46546C67; // "glTF"
        constexpr uint32_t kChunkJson = 0x4E4F534A;
        constexpr uint32_t kChunkBin = 0x004E4942;

        struct Fail : std::runtime_error
        {
            using std::runtime_error::runtime_error;
        };

        uint32_t readU32(const uint8_t* p)
        {
            return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) | (static_cast<uint32_t>(p[2]) << 16)
                | (static_cast<uint32_t>(p[3]) << 24);
        }

        size_t componentCount(const std::string& type)
        {
            if (type == "SCALAR")
                return 1;
            if (type == "VEC2")
                return 2;
            if (type == "VEC3")
                return 3;
            if (type == "VEC4" || type == "MAT2")
                return 4;
            if (type == "MAT3")
                return 9;
            if (type == "MAT4")
                return 16;
            throw Fail("unknown accessor type " + type);
        }

        size_t componentSize(int type)
        {
            switch (type)
            {
                case 5120:
                case 5121:
                    return 1;
                case 5122:
                case 5123:
                    return 2;
                case 5125:
                case 5126:
                    return 4;
                default:
                    throw Fail("unknown component type " + std::to_string(type));
            }
        }

        float readComponent(const uint8_t* p, int type, bool normalized)
        {
            switch (type)
            {
                case 5120:
                {
                    const auto v = static_cast<int8_t>(*p);
                    return normalized ? std::max(v / 127.0f, -1.0f) : static_cast<float>(v);
                }
                case 5121:
                    return normalized ? *p / 255.0f : static_cast<float>(*p);
                case 5122:
                {
                    int16_t v;
                    std::memcpy(&v, p, 2);
                    return normalized ? std::max(v / 32767.0f, -1.0f) : static_cast<float>(v);
                }
                case 5123:
                {
                    uint16_t v;
                    std::memcpy(&v, p, 2);
                    return normalized ? v / 65535.0f : static_cast<float>(v);
                }
                case 5125:
                {
                    uint32_t v;
                    std::memcpy(&v, p, 4);
                    return static_cast<float>(v);
                }
                default:
                {
                    float v;
                    std::memcpy(&v, p, 4);
                    return v;
                }
            }
        }

        class Reader
        {
        public:
            Reader(const json_t& doc, std::span<const uint8_t> bin)
                : _doc(doc)
                , _bin(bin)
            {
            }

            // Reads an accessor into a flat float array; `components` receives the per-element component count.
            std::vector<float> floats(int32_t accessorIndex, size_t& components) const
            {
                const auto& accessors = _doc.value("accessors", json_t::array());
                if (accessorIndex < 0 || static_cast<size_t>(accessorIndex) >= accessors.size())
                    throw Fail("accessor index out of range");
                const auto& acc = accessors[accessorIndex];
                if (acc.contains("sparse"))
                    throw Fail("sparse accessors are not supported");
                const auto type = acc.value("componentType", 0);
                const bool normalized = acc.value("normalized", false);
                const auto count = acc.value("count", static_cast<size_t>(0));
                components = componentCount(acc.value("type", std::string()));
                std::vector<float> out(count * components, 0.0f);
                if (!acc.contains("bufferView"))
                    return out;

                const auto& views = _doc.value("bufferViews", json_t::array());
                const auto viewIndex = acc["bufferView"].get<size_t>();
                if (viewIndex >= views.size())
                    throw Fail("bufferView index out of range");
                const auto& view = views[viewIndex];
                if (view.value("buffer", 0) != 0)
                    throw Fail("only the embedded buffer is supported");
                const auto viewOffset = view.value("byteOffset", static_cast<size_t>(0));
                const auto viewLength = view.value("byteLength", static_cast<size_t>(0));
                const auto csize = componentSize(type);
                auto stride = view.value("byteStride", static_cast<size_t>(0));
                if (stride == 0)
                    stride = csize * components;
                const auto offset = viewOffset + acc.value("byteOffset", static_cast<size_t>(0));
                if (viewOffset + viewLength > _bin.size())
                    throw Fail("bufferView exceeds the binary chunk");
                if (count > 0 && offset + (count - 1) * stride + csize * components > viewOffset + viewLength)
                    throw Fail("accessor exceeds its bufferView");
                for (size_t i = 0; i < count; i++)
                    for (size_t c = 0; c < components; c++)
                        out[i * components + c] = readComponent(_bin.data() + offset + i * stride + c * csize, type, normalized);
                return out;
            }

            std::vector<uint32_t> indices(int32_t accessorIndex) const
            {
                size_t comps = 0;
                auto values = floats(accessorIndex, comps);
                if (comps != 1)
                    throw Fail("index accessor must be scalar");
                std::vector<uint32_t> out(values.size());
                for (size_t i = 0; i < values.size(); i++)
                    out[i] = static_cast<uint32_t>(values[i]);
                return out;
            }

            std::vector<uint8_t> viewBytes(int32_t viewIndex) const
            {
                const auto& views = _doc.value("bufferViews", json_t::array());
                if (viewIndex < 0 || static_cast<size_t>(viewIndex) >= views.size())
                    throw Fail("image bufferView out of range");
                const auto offset = views[viewIndex].value("byteOffset", static_cast<size_t>(0));
                const auto length = views[viewIndex].value("byteLength", static_cast<size_t>(0));
                if (offset + length > _bin.size())
                    throw Fail("image exceeds the binary chunk");
                return { _bin.begin() + offset, _bin.begin() + offset + length };
            }

        private:
            const json_t& _doc;
            std::span<const uint8_t> _bin;
        };

        template<size_t N>
        std::array<float, N> readArray(const json_t& j, const char* key, const std::array<float, N>& fallback)
        {
            if (!j.contains(key) || !j[key].is_array() || j[key].size() != N)
                return fallback;
            std::array<float, N> out{};
            for (size_t i = 0; i < N; i++)
                out[i] = j[key][i].get<float>();
            return out;
        }

        void parsePrimitive(const json_t& prim, const Reader& reader, GlbModel& model, std::vector<int32_t>& list)
        {
            if (prim.value("mode", 4) != 4)
                throw Fail("only triangle lists are supported");
            const auto& attrs = prim.value("attributes", json_t::object());
            if (!attrs.contains("POSITION"))
                throw Fail("primitive has no POSITION");

            Primitive out;
            size_t comps = 0;
            const auto pos = reader.floats(attrs["POSITION"].get<int32_t>(), comps);
            if (comps != 3)
                throw Fail("POSITION must be VEC3");
            const size_t count = pos.size() / 3;
            out.vertices.resize(count);
            for (size_t i = 0; i < count; i++)
                out.vertices[i].position = { pos[i * 3], pos[i * 3 + 1], pos[i * 3 + 2] };

            if (attrs.contains("NORMAL"))
            {
                const auto v = reader.floats(attrs["NORMAL"].get<int32_t>(), comps);
                if (comps != 3 || v.size() / 3 != count)
                    throw Fail("NORMAL does not match POSITION");
                for (size_t i = 0; i < count; i++)
                    out.vertices[i].normal = { v[i * 3], v[i * 3 + 1], v[i * 3 + 2] };
            }
            if (attrs.contains("TEXCOORD_0"))
            {
                const auto v = reader.floats(attrs["TEXCOORD_0"].get<int32_t>(), comps);
                if (comps != 2 || v.size() / 2 != count)
                    throw Fail("TEXCOORD_0 does not match POSITION");
                for (size_t i = 0; i < count; i++)
                    out.vertices[i].uv = { v[i * 2], v[i * 2 + 1] };
            }
            if (attrs.contains("JOINTS_0") && attrs.contains("WEIGHTS_0"))
            {
                const auto j = reader.floats(attrs["JOINTS_0"].get<int32_t>(), comps);
                if (comps != 4 || j.size() / 4 != count)
                    throw Fail("JOINTS_0 does not match POSITION");
                const auto w = reader.floats(attrs["WEIGHTS_0"].get<int32_t>(), comps);
                if (comps != 4 || w.size() / 4 != count)
                    throw Fail("WEIGHTS_0 does not match POSITION");
                for (size_t i = 0; i < count; i++)
                {
                    for (size_t c = 0; c < 4; c++)
                    {
                        out.vertices[i].joints[c] = static_cast<uint16_t>(j[i * 4 + c]);
                        out.vertices[i].weights[c] = w[i * 4 + c];
                    }
                }
            }

            if (prim.contains("indices"))
                out.indices = reader.indices(prim["indices"].get<int32_t>());
            else
            {
                out.indices.resize(count);
                for (size_t i = 0; i < count; i++)
                    out.indices[i] = static_cast<uint32_t>(i);
            }
            if (out.indices.size() % 3 != 0)
                throw Fail("index count is not a multiple of 3");
            for (auto index : out.indices)
                if (index >= count)
                    throw Fail("index out of range");
            out.material = prim.value("material", -1);

            list.push_back(static_cast<int32_t>(model.primitives.size()));
            model.primitives.push_back(std::move(out));
        }

        void parseAnimation(const json_t& anim, const Reader& reader, size_t nodeCount, GlbModel& model)
        {
            Clip clip;
            clip.name = anim.value("name", std::string());
            const auto& samplers = anim.value("samplers", json_t::array());
            for (const auto& ch : anim.value("channels", json_t::array()))
            {
                const auto& target = ch.value("target", json_t::object());
                const auto pathName = target.value("path", std::string());
                Channel out;
                if (pathName == "translation")
                    out.path = ChannelPath::translation;
                else if (pathName == "rotation")
                    out.path = ChannelPath::rotation;
                else if (pathName == "scale")
                    out.path = ChannelPath::scale;
                else
                    continue; // weights (morph targets) are ignored
                out.node = target.value("node", -1);
                if (out.node < 0 || static_cast<size_t>(out.node) >= nodeCount)
                    throw Fail("animation targets a missing node");
                const auto samplerIndex = ch.value("sampler", -1);
                if (samplerIndex < 0 || static_cast<size_t>(samplerIndex) >= samplers.size())
                    throw Fail("animation channel has no sampler");
                const auto& sampler = samplers[samplerIndex];
                const auto interpolation = sampler.value("interpolation", std::string("LINEAR"));
                if (interpolation == "CUBICSPLINE")
                    throw Fail("CUBICSPLINE interpolation is not supported");
                out.step = interpolation == "STEP";
                size_t comps = 0;
                out.times = reader.floats(sampler.value("input", -1), comps);
                out.values = reader.floats(sampler.value("output", -1), comps);
                const size_t expected = out.path == ChannelPath::rotation ? 4 : 3;
                if (comps != expected || out.values.size() / expected != out.times.size())
                    throw Fail("animation sampler sizes do not match");
                if (!out.times.empty())
                    clip.duration = std::max(clip.duration, out.times.back());
                clip.channels.push_back(std::move(out));
            }
            model.clips.push_back(std::move(clip));
        }

        void normalise(Quat& q)
        {
            const float len = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
            if (len > 0)
                for (auto& v : q)
                    v /= len;
        }

        Quat slerp(Quat a, Quat b, float t)
        {
            float dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
            if (dot < 0)
            {
                for (auto& v : b)
                    v = -v;
                dot = -dot;
            }
            Quat out;
            if (dot > 0.9995f)
            {
                for (size_t i = 0; i < 4; i++)
                    out[i] = a[i] + (b[i] - a[i]) * t;
            }
            else
            {
                const float theta = std::acos(dot);
                const float sa = std::sin((1 - t) * theta) / std::sin(theta);
                const float sb = std::sin(t * theta) / std::sin(theta);
                for (size_t i = 0; i < 4; i++)
                    out[i] = a[i] * sa + b[i] * sb;
            }
            normalise(out);
            return out;
        }

        // Samples a channel at `time` (clamped to its key range).
        void sample(const Channel& ch, float time, float* out, size_t comps)
        {
            if (ch.times.empty())
                return;
            if (time <= ch.times.front() || ch.times.size() == 1)
            {
                std::copy_n(ch.values.begin(), comps, out);
                return;
            }
            if (time >= ch.times.back())
            {
                std::copy_n(ch.values.end() - comps, comps, out);
                return;
            }
            const auto it = std::upper_bound(ch.times.begin(), ch.times.end(), time);
            const size_t hi = static_cast<size_t>(it - ch.times.begin());
            const size_t lo = hi - 1;
            if (ch.step)
            {
                std::copy_n(ch.values.begin() + lo * comps, comps, out);
                return;
            }
            const float t = (time - ch.times[lo]) / (ch.times[hi] - ch.times[lo]);
            if (comps == 4)
            {
                Quat a, b;
                std::copy_n(ch.values.begin() + lo * 4, 4, a.begin());
                std::copy_n(ch.values.begin() + hi * 4, 4, b.begin());
                const auto q = slerp(a, b, t);
                std::copy(q.begin(), q.end(), out);
            }
            else
            {
                for (size_t i = 0; i < comps; i++)
                    out[i] = ch.values[lo * comps + i] + (ch.values[hi * comps + i] - ch.values[lo * comps + i]) * t;
            }
        }

        void checkTexture(const Image& image, uint32_t maxTextureSize)
        {
            if (maxTextureSize == 0)
                return;
            static constexpr uint8_t kPng[8] = { 0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A };
            if (image.bytes.size() < 24 || std::memcmp(image.bytes.data(), kPng, 8) != 0)
                throw Fail("textures must be PNG images");
            const auto w = (static_cast<uint32_t>(image.bytes[16]) << 24) | (static_cast<uint32_t>(image.bytes[17]) << 16)
                | (static_cast<uint32_t>(image.bytes[18]) << 8) | image.bytes[19];
            const auto h = (static_cast<uint32_t>(image.bytes[20]) << 24) | (static_cast<uint32_t>(image.bytes[21]) << 16)
                | (static_cast<uint32_t>(image.bytes[22]) << 8) | image.bytes[23];
            if (w > maxTextureSize || h > maxTextureSize)
                throw Fail("texture is larger than " + std::to_string(maxTextureSize) + "x" + std::to_string(maxTextureSize));
        }

        GlbModel parse(std::span<const uint8_t> data, size_t maxTriangles, uint32_t maxTextureSize)
        {
            if (data.size() < 20 || readU32(data.data()) != kMagic)
                throw Fail("not a GLB file");
            if (readU32(data.data() + 4) != 2)
                throw Fail("only glTF version 2 is supported");
            const auto total = readU32(data.data() + 8);
            if (total > data.size())
                throw Fail("GLB is truncated");

            std::span<const uint8_t> jsonChunk, binChunk;
            size_t pos = 12;
            while (pos + 8 <= total)
            {
                const auto length = readU32(data.data() + pos);
                const auto type = readU32(data.data() + pos + 4);
                pos += 8;
                if (length > total - pos)
                    throw Fail("GLB chunk exceeds the file");
                if (type == kChunkJson && jsonChunk.empty())
                    jsonChunk = data.subspan(pos, length);
                else if (type == kChunkBin && binChunk.empty())
                    binChunk = data.subspan(pos, length);
                pos += (length + 3) & ~3u;
            }
            if (jsonChunk.empty())
                throw Fail("GLB has no JSON chunk");

            json_t doc;
            try
            {
                doc = json_t::parse(jsonChunk.begin(), jsonChunk.end());
            }
            catch (const std::exception& e)
            {
                throw Fail(std::string("invalid JSON: ") + e.what());
            }

            Reader reader(doc, binChunk);
            GlbModel model;

            for (const auto& mat : doc.value("materials", json_t::array()))
            {
                Material m;
                m.name = mat.value("name", std::string());
                const auto& pbr = mat.value("pbrMetallicRoughness", json_t::object());
                m.baseColour = readArray<4>(pbr, "baseColorFactor", m.baseColour);
                if (pbr.contains("baseColorTexture"))
                {
                    const auto texIndex = pbr["baseColorTexture"].value("index", -1);
                    const auto& textures = doc.value("textures", json_t::array());
                    if (texIndex >= 0 && static_cast<size_t>(texIndex) < textures.size())
                        m.textureImage = textures[texIndex].value("source", -1);
                }
                model.materials.push_back(std::move(m));
            }

            for (const auto& img : doc.value("images", json_t::array()))
            {
                Image image;
                image.mimeType = img.value("mimeType", std::string());
                if (!img.contains("bufferView"))
                    throw Fail("only embedded images are supported");
                image.bytes = reader.viewBytes(img["bufferView"].get<int32_t>());
                checkTexture(image, maxTextureSize);
                model.images.push_back(std::move(image));
            }

            for (const auto& mesh : doc.value("meshes", json_t::array()))
            {
                std::vector<int32_t> list;
                for (const auto& prim : mesh.value("primitives", json_t::array()))
                    parsePrimitive(prim, reader, model, list);
                model.meshPrimitives.push_back(std::move(list));
            }
            for (const auto& prim : model.primitives)
                if (prim.material >= static_cast<int32_t>(model.materials.size()))
                    throw Fail("primitive references a missing material");

            const auto& nodes = doc.value("nodes", json_t::array());
            for (const auto& n : nodes)
            {
                Node node;
                node.name = n.value("name", std::string());
                node.mesh = n.value("mesh", -1);
                node.skin = n.value("skin", -1);
                if (node.mesh >= static_cast<int32_t>(model.meshPrimitives.size()))
                    throw Fail("node references a missing mesh");
                node.translation = readArray<3>(n, "translation", node.translation);
                node.rotation = readArray<4>(n, "rotation", node.rotation);
                node.scale = readArray<3>(n, "scale", node.scale);
                if (n.contains("matrix"))
                    throw Fail("node matrices are not supported, use translation/rotation/scale");
                for (const auto& c : n.value("children", json_t::array()))
                {
                    const auto child = c.get<int32_t>();
                    if (child < 0 || static_cast<size_t>(child) >= nodes.size())
                        throw Fail("node child out of range");
                    node.children.push_back(child);
                }
                model.nodes.push_back(std::move(node));
            }
            for (size_t i = 0; i < model.nodes.size(); i++)
            {
                for (auto child : model.nodes[i].children)
                {
                    if (model.nodes[child].parent != -1 || static_cast<size_t>(child) == i)
                        throw Fail("node hierarchy is not a tree");
                    model.nodes[child].parent = static_cast<int32_t>(i);
                }
            }
            // Cycle check: every node must reach a root within nodes.size() steps.
            for (size_t i = 0; i < model.nodes.size(); i++)
            {
                int32_t cur = static_cast<int32_t>(i);
                size_t steps = 0;
                while (model.nodes[cur].parent != -1)
                {
                    cur = model.nodes[cur].parent;
                    if (++steps > model.nodes.size())
                        throw Fail("node hierarchy contains a cycle");
                }
            }
            const auto& scenes = doc.value("scenes", json_t::array());
            const auto sceneIndex = doc.value("scene", 0);
            if (!scenes.empty() && sceneIndex >= 0 && static_cast<size_t>(sceneIndex) < scenes.size())
            {
                for (const auto& r : scenes[sceneIndex].value("nodes", json_t::array()))
                {
                    const auto root = r.get<int32_t>();
                    if (root < 0 || static_cast<size_t>(root) >= model.nodes.size())
                        throw Fail("scene node out of range");
                    model.rootNodes.push_back(root);
                }
            }
            else
            {
                for (size_t i = 0; i < model.nodes.size(); i++)
                    if (model.nodes[i].parent == -1)
                        model.rootNodes.push_back(static_cast<int32_t>(i));
            }

            for (const auto& s : doc.value("skins", json_t::array()))
            {
                Skin skin;
                for (const auto& j : s.value("joints", json_t::array()))
                {
                    const auto joint = j.get<int32_t>();
                    if (joint < 0 || static_cast<size_t>(joint) >= model.nodes.size())
                        throw Fail("skin joint out of range");
                    skin.joints.push_back(joint);
                }
                if (s.contains("inverseBindMatrices"))
                {
                    size_t comps = 0;
                    const auto v = reader.floats(s["inverseBindMatrices"].get<int32_t>(), comps);
                    if (comps != 16 || v.size() / 16 < skin.joints.size())
                        throw Fail("inverseBindMatrices do not match the joints");
                    for (size_t i = 0; i < skin.joints.size(); i++)
                    {
                        Mat4 m;
                        std::copy_n(v.begin() + i * 16, 16, m.begin());
                        skin.inverseBind.push_back(m);
                    }
                }
                else
                    skin.inverseBind.assign(skin.joints.size(), identity());
                model.skins.push_back(std::move(skin));
            }
            for (const auto& node : model.nodes)
                if (node.skin >= static_cast<int32_t>(model.skins.size()))
                    throw Fail("node references a missing skin");
            for (const auto& prim : model.primitives)
            {
                // Joint indices must address a skin; check against the largest skin.
                size_t maxJoints = 0;
                for (const auto& s : model.skins)
                    maxJoints = std::max(maxJoints, s.joints.size());
                for (const auto& v : prim.vertices)
                    for (size_t c = 0; c < 4; c++)
                        if (v.weights[c] > 0 && v.joints[c] >= std::max<size_t>(maxJoints, 1))
                            throw Fail("vertex joint index out of range");
            }

            for (const auto& a : doc.value("animations", json_t::array()))
                parseAnimation(a, reader, model.nodes.size(), model);

            if (maxTriangles != 0 && model.triangleCount() > 2 * maxTriangles)
                throw Fail(
                    "model has " + std::to_string(model.triangleCount()) + " triangles, over twice the budget of "
                    + std::to_string(maxTriangles));
            return model;
        }
    } // namespace

    Mat4 identity()
    {
        return { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
    }

    Mat4 multiply(const Mat4& a, const Mat4& b)
    {
        Mat4 out{};
        for (size_t col = 0; col < 4; col++)
            for (size_t row = 0; row < 4; row++)
            {
                float sum = 0;
                for (size_t k = 0; k < 4; k++)
                    sum += a[k * 4 + row] * b[col * 4 + k];
                out[col * 4 + row] = sum;
            }
        return out;
    }

    Mat4 composeTRS(const Vec3& t, const Quat& r, const Vec3& s)
    {
        const float x = r[0], y = r[1], z = r[2], w = r[3];
        Mat4 m = identity();
        m[0] = (1 - 2 * (y * y + z * z)) * s[0];
        m[1] = (2 * (x * y + z * w)) * s[0];
        m[2] = (2 * (x * z - y * w)) * s[0];
        m[4] = (2 * (x * y - z * w)) * s[1];
        m[5] = (1 - 2 * (x * x + z * z)) * s[1];
        m[6] = (2 * (y * z + x * w)) * s[1];
        m[8] = (2 * (x * z + y * w)) * s[2];
        m[9] = (2 * (y * z - x * w)) * s[2];
        m[10] = (1 - 2 * (x * x + y * y)) * s[2];
        m[12] = t[0];
        m[13] = t[1];
        m[14] = t[2];
        return m;
    }

    size_t GlbModel::triangleCount() const
    {
        size_t total = 0;
        for (const auto& p : primitives)
            total += p.indices.size() / 3;
        return total;
    }

    const Clip* GlbModel::findClip(const std::string& name) const
    {
        for (const auto& c : clips)
            if (c.name == name)
                return &c;
        return nullptr;
    }

    std::vector<Mat4> GlbModel::worldTransforms(const Clip* clip, float time) const
    {
        struct Pose
        {
            Vec3 t;
            Quat r;
            Vec3 s;
        };
        std::vector<Pose> pose(nodes.size());
        for (size_t i = 0; i < nodes.size(); i++)
            pose[i] = { nodes[i].translation, nodes[i].rotation, nodes[i].scale };
        if (clip != nullptr)
        {
            for (const auto& ch : clip->channels)
            {
                auto& p = pose[ch.node];
                switch (ch.path)
                {
                    case ChannelPath::translation:
                        sample(ch, time, p.t.data(), 3);
                        break;
                    case ChannelPath::scale:
                        sample(ch, time, p.s.data(), 3);
                        break;
                    case ChannelPath::rotation:
                        sample(ch, time, p.r.data(), 4);
                        break;
                }
            }
        }

        std::vector<Mat4> world(nodes.size());
        std::vector<bool> done(nodes.size(), false);
        // Parents are resolved recursively; the hierarchy was validated as acyclic on load.
        auto resolve = [&](auto&& self, size_t i) -> void
        {
            if (done[i])
                return;
            auto local = composeTRS(pose[i].t, pose[i].r, pose[i].s);
            if (nodes[i].parent >= 0)
            {
                self(self, static_cast<size_t>(nodes[i].parent));
                world[i] = multiply(world[nodes[i].parent], local);
            }
            else
                world[i] = local;
            done[i] = true;
        };
        for (size_t i = 0; i < nodes.size(); i++)
            resolve(resolve, i);
        return world;
    }

    std::vector<Mat4> GlbModel::skinMatrices(int32_t skin, const std::vector<Mat4>& world) const
    {
        std::vector<Mat4> out;
        if (skin < 0 || static_cast<size_t>(skin) >= skins.size())
            return out;
        const auto& s = skins[skin];
        for (size_t i = 0; i < s.joints.size(); i++)
            out.push_back(multiply(world[s.joints[i]], s.inverseBind[i]));
        return out;
    }

    LoadResult loadGlb(std::span<const uint8_t> data, size_t maxTriangles, uint32_t maxTextureSize)
    {
        LoadResult result;
        try
        {
            result.model = parse(data, maxTriangles, maxTextureSize);
            result.ok = true;
        }
        catch (const std::exception& e)
        {
            result.error = e.what();
        }
        return result;
    }
} // namespace OpenRCT2::Model
