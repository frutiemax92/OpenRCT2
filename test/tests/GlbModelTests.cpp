/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <cstring>
#include <gtest/gtest.h>
#include <openrct2/core/Json.hpp>
#include <openrct2/model/GlbModel.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Model;

namespace
{
    void putU32(std::vector<uint8_t>& out, uint32_t v)
    {
        for (int i = 0; i < 4; i++)
            out.push_back(static_cast<uint8_t>(v >> (8 * i)));
    }

    void putFloats(std::vector<uint8_t>& bin, std::initializer_list<float> values)
    {
        for (float f : values)
        {
            uint8_t b[4];
            std::memcpy(b, &f, 4);
            bin.insert(bin.end(), b, b + 4);
        }
    }

    std::vector<uint8_t> makeGlb(const json_t& doc, const std::vector<uint8_t>& bin)
    {
        auto jsonText = doc.dump();
        while (jsonText.size() % 4 != 0)
            jsonText.push_back(' ');
        auto padded = bin;
        while (padded.size() % 4 != 0)
            padded.push_back(0);
        std::vector<uint8_t> out;
        putU32(out, 0x46546C67);
        putU32(out, 2);
        putU32(out, static_cast<uint32_t>(12 + 8 + jsonText.size() + 8 + padded.size()));
        putU32(out, static_cast<uint32_t>(jsonText.size()));
        putU32(out, 0x4E4F534A);
        out.insert(out.end(), jsonText.begin(), jsonText.end());
        putU32(out, static_cast<uint32_t>(padded.size()));
        putU32(out, 0x004E4942);
        out.insert(out.end(), padded.begin(), padded.end());
        return out;
    }

    // One triangle (3 vertices, u16 indices) plus an optional animation moving the node along +x.
    std::vector<uint8_t> triangleGlb(uint16_t index2 = 2, bool animated = false, size_t triangles = 1)
    {
        std::vector<uint8_t> bin;
        putFloats(bin, { 0, 0, 0, 1, 0, 0, 0, 1, 0 }); // 0..35 positions
        for (size_t i = 0; i < triangles; i++)
            bin.insert(bin.end(), { 0, 0, 1, 0, static_cast<uint8_t>(index2), 0 }); // indices
        const size_t indexBytes = triangles * 6;
        while (bin.size() % 4 != 0)
            bin.push_back(0);
        const size_t timesOffset = bin.size();
        putFloats(bin, { 0.0f, 1.0f });       // times
        putFloats(bin, { 0, 0, 0, 2, 0, 0 }); // translations
        json_t doc = {
            { "asset", { { "version", "2.0" } } },
            { "scene", 0 },
            { "scenes", json_t::array({ { { "nodes", { 0 } } } }) },
            { "nodes", json_t::array({ { { "name", "root" }, { "mesh", 0 } } }) },
            { "meshes", json_t::array({ { { "primitives", json_t::array({ { { "attributes", { { "POSITION", 0 } } }, { "indices", 1 } } }) } } }) },
            { "accessors",
              json_t::array({ { { "bufferView", 0 }, { "componentType", 5126 }, { "count", 3 }, { "type", "VEC3" } },
                              { { "bufferView", 1 }, { "componentType", 5123 }, { "count", triangles * 3 }, { "type", "SCALAR" } },
                              { { "bufferView", 2 }, { "componentType", 5126 }, { "count", 2 }, { "type", "SCALAR" } },
                              { { "bufferView", 3 }, { "componentType", 5126 }, { "count", 2 }, { "type", "VEC3" } } }) },
            { "bufferViews",
              json_t::array({ { { "buffer", 0 }, { "byteOffset", 0 }, { "byteLength", 36 } },
                              { { "buffer", 0 }, { "byteOffset", 36 }, { "byteLength", indexBytes } },
                              { { "buffer", 0 }, { "byteOffset", timesOffset }, { "byteLength", 8 } },
                              { { "buffer", 0 }, { "byteOffset", timesOffset + 8 }, { "byteLength", 24 } } }) },
        };
        if (animated)
        {
            doc["animations"] = json_t::array(
                { { { "name", "slide" },
                    { "samplers", json_t::array({ { { "input", 2 }, { "output", 3 }, { "interpolation", "LINEAR" } } }) },
                    { "channels", json_t::array({ { { "sampler", 0 }, { "target", { { "node", 0 }, { "path", "translation" } } } } }) } } });
        }
        return makeGlb(doc, bin);
    }
} // namespace

TEST(GlbModel, LoadsAnIndexedTriangle)
{
    const auto data = triangleGlb();
    const auto result = loadGlb(data);
    ASSERT_TRUE(result.ok) << result.error;
    ASSERT_EQ(result.model.primitives.size(), 1u);
    EXPECT_EQ(result.model.triangleCount(), 1u);
    EXPECT_EQ(result.model.primitives[0].vertices[1].position[0], 1.0f);
    EXPECT_EQ(result.model.primitives[0].vertices[2].position[1], 1.0f);
    ASSERT_EQ(result.model.rootNodes.size(), 1u);
    EXPECT_EQ(result.model.nodes[0].name, "root");
}

TEST(GlbModel, RejectsBrokenFiles)
{
    auto data = triangleGlb();
    auto badMagic = data;
    badMagic[0] = 'X';
    EXPECT_FALSE(loadGlb(badMagic).ok);

    auto truncated = data;
    truncated.resize(40);
    EXPECT_FALSE(loadGlb(truncated).ok);

    EXPECT_FALSE(loadGlb({}).ok);

    const auto badIndex = triangleGlb(9);
    const auto result = loadGlb(badIndex);
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.error.find("index out of range"), std::string::npos);
}

TEST(GlbModel, EnforcesTheTriangleBudget)
{
    // R3D-003: models above twice the budget are rejected.
    EXPECT_TRUE(loadGlb(triangleGlb(2, false, 2), 1).ok);
    const auto over = loadGlb(triangleGlb(2, false, 3), 1);
    EXPECT_FALSE(over.ok);
    EXPECT_NE(over.error.find("budget"), std::string::npos);
    EXPECT_TRUE(loadGlb(triangleGlb(2, false, 3), 0).ok); // 0 disables the check
}

TEST(GlbModel, SamplesAnimationClips)
{
    const auto data = triangleGlb(2, true);
    const auto result = loadGlb(data);
    ASSERT_TRUE(result.ok) << result.error;
    const auto* clip = result.model.findClip("slide");
    ASSERT_NE(clip, nullptr);
    EXPECT_FLOAT_EQ(clip->duration, 1.0f);
    EXPECT_EQ(result.model.findClip("missing"), nullptr);

    EXPECT_FLOAT_EQ(result.model.worldTransforms(clip, 0.0f)[0][12], 0.0f);
    EXPECT_FLOAT_EQ(result.model.worldTransforms(clip, 0.5f)[0][12], 1.0f);
    EXPECT_FLOAT_EQ(result.model.worldTransforms(clip, 1.0f)[0][12], 2.0f);
    EXPECT_FLOAT_EQ(result.model.worldTransforms(clip, 5.0f)[0][12], 2.0f); // clamped
    EXPECT_FLOAT_EQ(result.model.worldTransforms(nullptr, 0.5f)[0][12], 0.0f); // bind pose
}

TEST(GlbModel, ComposesTransforms)
{
    // 90 degrees about z maps +x to +y.
    const float s = std::sqrt(0.5f);
    const auto rot = composeTRS({ 0, 0, 0 }, { 0, 0, s, s }, { 1, 1, 1 });
    const auto moved = multiply(composeTRS({ 5, 0, 0 }, { 0, 0, 0, 1 }, { 1, 1, 1 }), rot);
    EXPECT_NEAR(moved[0], 0.0f, 1e-5f);
    EXPECT_NEAR(moved[1], 1.0f, 1e-5f);
    EXPECT_FLOAT_EQ(moved[12], 5.0f);
}
