/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <gtest/gtest.h>
#include <openrct2/drawing/Drawing.h>
#include <openrct2/drawing/Drawing.Sprite.h>
#include <openrct2/drawing/Image.h>
#include <openrct2/drawing/ModelSpriteStore.h>
#include <openrct2/paint/entity/Paint.Model.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Model;

namespace
{
    // A 1 metre square slab of one material at the given height.
    std::shared_ptr<GlbModel> slab(float y, float sizeX = 1)
    {
        auto model = std::make_shared<GlbModel>();
        Node root;
        root.mesh = 0;
        model->nodes.push_back(root);
        model->rootNodes.push_back(0);
        model->meshPrimitives.push_back({ 0 });
        Material material;
        material.baseColour = { 0.8f, 0.2f, 0.2f, 1 };
        model->materials.push_back(material);
        Primitive prim;
        prim.material = 0;
        auto vertex = [&](float x, float z) {
            Vertex v;
            v.position = { x, y, z };
            v.normal = { 0, 1, 0 };
            return v;
        };
        prim.vertices = { vertex(0, 0), vertex(0, 1), vertex(sizeX, 1), vertex(sizeX, 0) };
        prim.indices = { 0, 1, 3, 1, 2, 3 };
        model->primitives.push_back(std::move(prim));
        return model;
    }
} // namespace

TEST(ModelSpriteStore, RegistersModelsAndForgetsThem)
{
    auto& store = Drawing::ModelSpriteStore::Instance();
    const auto model = slab(0);
    const auto id = store.RegisterModel(model, 12.0f);
    EXPECT_NE(id, 0u);
    EXPECT_EQ(store.GetModel(id), model.get());
    EXPECT_FLOAT_EQ(store.GetUnitsPerMetre(id), 12.0f);
    const auto other = store.RegisterModel(slab(0));
    EXPECT_NE(other, id);
    store.UnregisterModel(id);
    EXPECT_EQ(store.GetModel(id), nullptr);
    EXPECT_NE(store.GetModel(other), nullptr);
    store.UnregisterModel(other);
}

TEST(ModelSpriteStore, DrawsAPoseAsAnImageAndSharesIdenticalPoses)
{
    auto& store = Drawing::ModelSpriteStore::Instance();
    store.Clear();
    const auto id = store.RegisterModel(slab(0), 16.0f);

    ImposterKey key;
    key.modelId = id;
    const auto first = store.GetImage(key);
    ASSERT_NE(first, kImageIndexUndefined);
    EXPECT_EQ(store.GetImage(key), first); // cached, not rasterised again
    EXPECT_EQ(store.GetSpriteCount(), 1u);

    const auto* g1 = GfxGetG1Element(first);
    ASSERT_NE(g1, nullptr);
    EXPECT_GT(g1->width, 20);
    EXPECT_GT(g1->height, 8);
    // The image is placed relative to the model origin, which lies inside the image.
    EXPECT_LT(g1->xOffset, 0);
    EXPECT_LT(g1->yOffset, 0);
    EXPECT_GT(g1->xOffset + g1->width, 0);
    EXPECT_GT(g1->yOffset + g1->height, 0);

    ImposterKey turned = key;
    turned.yawBucket = 4;
    EXPECT_NE(store.GetImage(turned), first);
    EXPECT_EQ(store.GetSpriteCount(), 2u);

    store.Clear();
    EXPECT_EQ(store.GetSpriteCount(), 0u);
    store.UnregisterModel(id);
}

TEST(ModelSpriteStore, UnknownOrEmptyModelsDrawNothing)
{
    auto& store = Drawing::ModelSpriteStore::Instance();
    ImposterKey missing;
    missing.modelId = 987654;
    EXPECT_EQ(store.GetImage(missing), kImageIndexUndefined);

    const auto empty = store.RegisterModel(std::make_shared<GlbModel>());
    ImposterKey key;
    key.modelId = empty;
    EXPECT_EQ(store.GetImage(key), kImageIndexUndefined);
    store.UnregisterModel(empty);
}

TEST(ModelSpriteStore, BeginFrameTrimsToTheBudgetAndFreesImages)
{
    auto& store = Drawing::ModelSpriteStore::Instance();
    store.Clear();
    const auto id = store.RegisterModel(slab(0), 16.0f);
    for (uint8_t yaw = 0; yaw < 6; yaw++)
    {
        ImposterKey key;
        key.modelId = id;
        key.yawBucket = yaw;
        ASSERT_NE(store.GetImage(key), kImageIndexUndefined);
    }
    EXPECT_EQ(store.GetSpriteCount(), 6u);
    store.SetBudget(1);
    EXPECT_EQ(store.GetSpriteCount(), 1u);
    store.SetBudget(Model::kDefaultImposterBudgetBytes);
    store.Clear();
    store.UnregisterModel(id);
}

TEST(PaintModel, BoundBoxCoversTheModelAtLeastOneUnitThick)
{
    RenderSettings settings;
    settings.unitsPerMetre = 16.0f;
    const auto box = ComputeModelBoundBox(*slab(0.5f, 2), settings);
    EXPECT_EQ(box.offset.x, 0);
    EXPECT_EQ(box.length.x, 32);
    EXPECT_EQ(box.offset.y, -16);
    EXPECT_EQ(box.length.y, 16);
    EXPECT_EQ(box.offset.z, 8);
    EXPECT_EQ(box.length.z, 1); // flat model
    EXPECT_EQ(ComputeModelBoundBox(GlbModel{}, settings).length.x, 0);
}
