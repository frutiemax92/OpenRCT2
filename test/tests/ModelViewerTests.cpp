/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "GlbTestHelpers.h"

#include <gtest/gtest.h>
#include <openrct2/drawing/ModelSpriteStore.h>
#include <openrct2/model/ModelViewer.h>

using namespace OpenRCT2::Model;

TEST(ModelViewer, StartsEmpty)
{
    ModelViewer viewer;
    EXPECT_FALSE(viewer.isLoaded());
    EXPECT_EQ(viewer.rowCount(), 0);
    EXPECT_EQ(viewer.model(), nullptr);
    EXPECT_EQ(viewer.summary(), "No model loaded");
    EXPECT_EQ(viewer.keyFor(0, 0).modelId, 0u);
}

TEST(ModelViewer, LoadsABinaryModelAndListsItsClips)
{
    ModelViewer viewer;
    std::string error;
    ASSERT_TRUE(viewer.loadBytes(GlbTest::triangleGlb(2, true), "tri.glb", error)) << error;
    EXPECT_TRUE(viewer.isLoaded());
    EXPECT_EQ(viewer.name(), "tri.glb");
    ASSERT_EQ(viewer.rowCount(), 2); // bind pose + the "slide" clip
    EXPECT_EQ(viewer.rowName(0), "(bind pose)");
    EXPECT_EQ(viewer.rowName(1), "slide");
    EXPECT_EQ(viewer.rowName(2), "");
    EXPECT_EQ(viewer.clipOfRow(0), -1);
    EXPECT_EQ(viewer.clipOfRow(1), 0);
    EXPECT_EQ(viewer.clipOfRow(99), 0); // clamped
    EXPECT_NE(viewer.summary().find("1 triangles, 1 clips"), std::string::npos);
    EXPECT_NE(OpenRCT2::Drawing::ModelSpriteStore::Instance().GetModel(viewer.modelId()), nullptr);
}

TEST(ModelViewer, KeysCoverEveryRowAndRotation)
{
    ModelViewer viewer;
    std::string error;
    ASSERT_TRUE(viewer.loadBytes(GlbTest::triangleGlb(2, true), "tri.glb", error));
    for (int32_t row = 0; row < viewer.rowCount(); row++)
        for (int32_t rotation = 0; rotation < ModelViewer::kRotations; rotation++)
        {
            const auto key = viewer.keyFor(row, rotation);
            EXPECT_EQ(key.modelId, viewer.modelId());
            EXPECT_EQ(key.clip, viewer.clipOfRow(row));
            EXPECT_EQ(key.rotation, rotation);
        }
    // Different cells never share an image.
    EXPECT_FALSE(viewer.keyFor(0, 0) == viewer.keyFor(0, 1));
    EXPECT_FALSE(viewer.keyFor(0, 0) == viewer.keyFor(1, 0));
}

TEST(ModelViewer, AnimationTimeAdvancesUnlessPaused)
{
    ModelViewer viewer;
    std::string error;
    ASSERT_TRUE(viewer.loadBytes(GlbTest::triangleGlb(2, true), "tri.glb", error));
    const auto first = viewer.keyFor(1, 0);
    EXPECT_EQ(first.frame, 0);
    viewer.advance(0.5f);
    EXPECT_FLOAT_EQ(viewer.time(), 0.5f);
    EXPECT_EQ(viewer.keyFor(1, 0).frame, 6); // 12 fps
    EXPECT_EQ(viewer.keyFor(0, 0).frame, 0); // the bind pose does not animate

    viewer.setPaused(true);
    viewer.advance(5.0f);
    EXPECT_FLOAT_EQ(viewer.time(), 0.5f);
    viewer.setPaused(false);
    viewer.advance(0.25f);
    EXPECT_FLOAT_EQ(viewer.time(), 0.75f);
}

TEST(ModelViewer, YawTurnsTheShownModel)
{
    ModelViewer viewer;
    std::string error;
    ASSERT_TRUE(viewer.loadBytes(GlbTest::triangleGlb(), "tri.glb", error));
    EXPECT_EQ(viewer.keyFor(0, 0).yawBucket, 0);
    viewer.setYaw(3.14159265f);
    EXPECT_EQ(viewer.keyFor(0, 0).yawBucket, 16);
}

TEST(ModelViewer, RejectsBrokenFilesAndKeepsTheOldModel)
{
    ModelViewer viewer;
    std::string error;
    ASSERT_TRUE(viewer.loadBytes(GlbTest::triangleGlb(), "good.glb", error));
    const auto id = viewer.modelId();
    EXPECT_FALSE(viewer.loadBytes(std::vector<uint8_t>{ 1, 2, 3 }, "bad.glb", error));
    EXPECT_FALSE(error.empty());
    EXPECT_EQ(viewer.name(), "good.glb");
    EXPECT_EQ(viewer.modelId(), id);

    std::string fileError;
    EXPECT_FALSE(viewer.loadFile("/definitely/not/here.glb", fileError));
    EXPECT_FALSE(fileError.empty());
}

TEST(ModelViewer, ReloadingAndUnloadingReleaseTheModel)
{
    ModelViewer viewer;
    std::string error;
    ASSERT_TRUE(viewer.loadBytes(GlbTest::triangleGlb(), "a.glb", error));
    const auto first = viewer.modelId();
    ASSERT_TRUE(viewer.loadBytes(GlbTest::triangleGlb(), "b.glb", error));
    EXPECT_NE(viewer.modelId(), first);
    EXPECT_EQ(OpenRCT2::Drawing::ModelSpriteStore::Instance().GetModel(first), nullptr);
    const auto second = viewer.modelId();
    viewer.unload();
    EXPECT_FALSE(viewer.isLoaded());
    EXPECT_EQ(OpenRCT2::Drawing::ModelSpriteStore::Instance().GetModel(second), nullptr);
}

TEST(ModelViewer, EveryCellRendersThroughTheSpriteStore)
{
    ModelViewer viewer;
    std::string error;
    ASSERT_TRUE(viewer.loadBytes(GlbTest::triangleGlb(2, true), "tri.glb", error));
    auto& store = OpenRCT2::Drawing::ModelSpriteStore::Instance();
    for (int32_t row = 0; row < viewer.rowCount(); row++)
        for (int32_t rotation = 0; rotation < ModelViewer::kRotations; rotation++)
        {
            // The triangle is a back face for some rotations; the store must not crash and the call must be repeatable.
            const auto image = store.GetImage(viewer.keyFor(row, rotation));
            EXPECT_EQ(store.GetImage(viewer.keyFor(row, rotation)), image);
        }
}
