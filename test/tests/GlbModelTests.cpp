/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <gtest/gtest.h>
#include "GlbTestHelpers.h"

#include <openrct2/core/Json.hpp>
#include <openrct2/model/GlbModel.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Model;

using namespace GlbTest;


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
