/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <cmath>
#include <gtest/gtest.h>
#include <openrct2/model/Rasteriser.h>

using namespace OpenRCT2::Model;

namespace
{
    // A horizontal square of side `size` metres at height `y`, facing up (visible) or down (back face).
    GlbModel squareModel(float y, float size, std::array<float, 4> colour, bool faceUp = true, GlbModel base = {})
    {
        if (base.nodes.empty())
        {
            Node root;
            root.mesh = 0;
            base.nodes.push_back(root);
            base.rootNodes.push_back(0);
            base.meshPrimitives.push_back({});
        }
        Material mat;
        mat.baseColour = colour;
        base.materials.push_back(mat);

        Primitive prim;
        prim.material = static_cast<int32_t>(base.materials.size()) - 1;
        auto vertex = [&](float x, float z) {
            Vertex v;
            v.position = { x, y, z };
            v.normal = { 0, faceUp ? 1.0f : -1.0f, 0 };
            return v;
        };
        prim.vertices = { vertex(0, 0), vertex(0, size), vertex(size, size), vertex(size, 0) };
        if (faceUp)
            prim.indices = { 0, 1, 3, 1, 2, 3 };
        else
            prim.indices = { 0, 3, 1, 1, 3, 2 };
        base.meshPrimitives[0].push_back(static_cast<int32_t>(base.primitives.size()));
        base.primitives.push_back(std::move(prim));
        return base;
    }

    RenderSettings plain()
    {
        RenderSettings s;
        s.unitsPerMetre = 16.0f;
        return s;
    }
} // namespace

TEST(Rasteriser, ProjectsLikeTheMap)
{
    // One tile east (+x) moves right and down by half; +z moves straight up.
    const auto east = project({ 32, 0, 0 }, 1.0f);
    EXPECT_FLOAT_EQ(east[0], 32.0f);
    EXPECT_FLOAT_EQ(east[1], 16.0f);
    const auto up = project({ 0, 0, 8 }, 1.0f);
    EXPECT_FLOAT_EQ(up[0], 0.0f);
    EXPECT_FLOAT_EQ(up[1], -8.0f);
    EXPECT_FLOAT_EQ(project({ 32, 0, 0 }, 0.5f)[0], 16.0f);
    // glTF +Y is map up, glTF -Z is map +y.
    EXPECT_EQ(modelToMap({ 1, 2, 3 }), (Vec3{ 1, -3, 2 }));
}

TEST(Rasteriser, DrawsAFaceThatLooksAtTheCamera)
{
    const auto image = render(squareModel(0, 1, { 1, 0, 0, 1 }), plain());
    ASSERT_FALSE(image.empty());
    EXPECT_GT(image.coveredPixels(), 100u);
    // The square is a 16 unit diamond: 32 wide, 16 tall.
    EXPECT_NEAR(image.width, 34, 3);
    EXPECT_NEAR(image.height, 18, 3);
    // The origin (a corner of the square) lies inside the image bounds.
    EXPECT_GE(image.originX, 0);
    EXPECT_LT(image.originX, image.width);
    // Faces lit from above are red-ish.
    for (size_t i = 0; i < image.rgba.size(); i++)
        if (image.rgba[i][3] != 0)
        {
            EXPECT_GT(image.rgba[i][0], 0);
            EXPECT_EQ(image.rgba[i][1], 0);
        }
}

TEST(Rasteriser, CullsBackFaces)
{
    const auto image = render(squareModel(0, 1, { 1, 1, 1, 1 }, false), plain());
    EXPECT_EQ(image.coveredPixels(), 0u);
}

TEST(Rasteriser, TheNearerSurfaceWins)
{
    // A red square at height 0 and a green one 0.5 m above it, same footprint: green is closer and hides red.
    auto model = squareModel(0, 1, { 1, 0, 0, 1 });
    model = squareModel(0.5f, 1, { 0, 1, 0, 1 }, true, model);
    const auto both = render(model, plain());
    ASSERT_FALSE(both.empty());
    size_t reddish = 0, greenish = 0;
    for (const auto& p : both.rgba)
    {
        if (p[3] == 0)
            continue;
        if (p[0] > 0)
            reddish++;
        if (p[1] > 0)
            greenish++;
    }
    EXPECT_GT(greenish, 100u);
    // The lower square only shows where the raised one does not cover it.
    EXPECT_LT(reddish, greenish);

    // Drawing order must not matter.
    auto reversed = squareModel(0.5f, 1, { 0, 1, 0, 1 });
    reversed = squareModel(0, 1, { 1, 0, 0, 1 }, true, reversed);
    const auto other = render(reversed, plain());
    EXPECT_EQ(both.width, other.width);
    EXPECT_EQ(both.height, other.height);
    EXPECT_EQ(both.rgba, other.rgba);
}

TEST(Rasteriser, ZoomScalesTheSprite)
{
    auto full = plain();
    auto half = plain();
    half.pixelsPerUnit = 0.5f;
    const auto a = render(squareModel(0, 1, { 1, 1, 1, 1 }), full);
    const auto b = render(squareModel(0, 1, { 1, 1, 1, 1 }), half);
    EXPECT_NEAR(b.width * 2, a.width, 6);
    EXPECT_LT(b.coveredPixels(), a.coveredPixels() / 2);
}

TEST(Rasteriser, YawRotatesAndColoursCanBeRemapped)
{
    auto settings = plain();
    const auto base = render(squareModel(0, 1, { 1, 1, 1, 1 }), settings);
    settings.orientation.yaw = 3.14159265f / 2;
    const auto turned = render(squareModel(0, 1, { 1, 1, 1, 1 }), settings);
    // A square turned by a quarter turn covers the same area but not the same place.
    EXPECT_NEAR(static_cast<double>(turned.coveredPixels()), static_cast<double>(base.coveredPixels()), base.coveredPixels() * 0.05);
    EXPECT_NE(turned.originX, base.originX);

    auto remap = plain();
    remap.materialColours = { std::array<float, 4>{ 0, 0, 1, 1 } };
    const auto blue = render(squareModel(0, 1, { 1, 0, 0, 1 }), remap);
    for (const auto& p : blue.rgba)
        if (p[3] != 0)
        {
            EXPECT_EQ(p[0], 0);
            EXPECT_GT(p[2], 0);
        }
}

TEST(Rasteriser, QuantisesToThePaletteWithAnOutline)
{
    const auto image = render(squareModel(0, 1, { 1, 0, 0, 1 }), plain());
    const std::vector<std::pair<uint8_t, PaletteColour>> palette = { { 10, { 255, 0, 0 } }, { 11, { 0, 0, 255 } }, { 12, { 40, 0, 0 } } };
    QuantiseSettings q;
    q.palette = palette;
    const auto plainImage = quantise(image, q);
    size_t drawn = 0;
    for (auto p : plainImage.pixels)
    {
        if (p == 0)
            continue;
        drawn++;
        EXPECT_TRUE(p == 10 || p == 12) << int(p); // never the blue entry
    }
    EXPECT_EQ(drawn, image.coveredPixels());

    q.outlineIndex = 99;
    const auto outlined = quantise(image, q);
    size_t ring = 0;
    for (auto p : outlined.pixels)
        ring += p == 99;
    EXPECT_GT(ring, 0u);
    // The outline never replaces model pixels.
    for (size_t i = 0; i < outlined.pixels.size(); i++)
    {
        const bool modelPixel = plainImage.pixels[i] != 0;
        if (modelPixel)
        {
            EXPECT_EQ(outlined.pixels[i], plainImage.pixels[i]);
        }
    }
}

TEST(Rasteriser, PosesFollowTheAnimationClip)
{
    // A node that slides 2 m along +x over one second: the sprite origin moves with it.
    auto model = squareModel(0, 1, { 1, 1, 1, 1 });
    Clip clip;
    clip.name = "slide";
    clip.duration = 1;
    Channel ch;
    ch.node = 0;
    ch.path = ChannelPath::translation;
    ch.times = { 0, 1 };
    ch.values = { 0, 0, 0, 2, 0, 0 };
    clip.channels.push_back(ch);
    model.clips.push_back(clip);

    auto s0 = plain();
    s0.clipIndex = 0;
    s0.time = 0;
    auto s1 = s0;
    s1.time = 1;
    const auto a = render(model, s0);
    const auto b = render(model, s1);
    // Moving along +x shifts the picture right of the origin by the projected distance (2 m = 32 units).
    EXPECT_EQ(a.originX - b.originX, 32);
}

TEST(Rasteriser, BoundsFollowTheModelAndItsYaw)
{
    // A 2 x 1 metre slab at height 0.5 metres, 16 units per metre.
    GlbModel model = squareModel(0.5f, 1, { 1, 1, 1, 1 });
    auto& prim = model.primitives[0];
    prim.vertices[2].position[0] = 2;
    prim.vertices[3].position[0] = 2;

    const auto bounds = computeBounds(model, plain());
    ASSERT_TRUE(bounds.valid);
    EXPECT_FLOAT_EQ(bounds.min[0], 0);
    EXPECT_FLOAT_EQ(bounds.max[0], 32);
    // glTF -Z is map +y, so the slab spans y from -16 to 0.
    EXPECT_FLOAT_EQ(bounds.min[1], -16);
    EXPECT_FLOAT_EQ(bounds.max[1], 0);
    EXPECT_FLOAT_EQ(bounds.min[2], 8);
    EXPECT_FLOAT_EQ(bounds.max[2], 8);

    auto turned = plain();
    turned.orientation.yaw = 3.14159265f / 2;
    const auto rotated = computeBounds(model, turned);
    EXPECT_NEAR(rotated.max[1], 32, 1e-3f);
    EXPECT_NEAR(rotated.min[0], 0, 1e-3f);
    EXPECT_NEAR(rotated.max[0], 16, 1e-3f);
}

TEST(Rasteriser, BoundsOfAnEmptyModelAreInvalid)
{
    EXPECT_FALSE(computeBounds(GlbModel{}, plain()).valid);
}

namespace
{
    // 2x2 RGBA PNG: red, green / blue, fully transparent white.
    const std::vector<uint8_t> kTestPng = {
        0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a, 0x00, 0x00, 0x00, 0x0d, 0x49, 0x48, 0x44, 0x52, 0x00, 0x00,
        0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x08, 0x06, 0x00, 0x00, 0x00, 0x72, 0xb6, 0x0d, 0x24, 0x00, 0x00, 0x00,
        0x13, 0x49, 0x44, 0x41, 0x54, 0x78, 0x9c, 0x63, 0xf8, 0xcf, 0xc0, 0xf0, 0x1f, 0x0c, 0x81, 0x34, 0x08, 0x30,
        0x00, 0x00, 0x48, 0xc9, 0x08, 0xf8, 0x71, 0xc5, 0x31, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x49, 0x45, 0x4e, 0x44,
        0xae, 0x42, 0x60, 0x82
    };

    GlbModel texturedSquare()
    {
        auto model = squareModel(0, 1, { 1, 1, 1, 1 });
        model.materials[0].textureImage = 0;
        Image png;
        png.mimeType = "image/png";
        png.bytes = kTestPng;
        model.images.push_back(png);
        // uv follows the horizontal position, so the 4 texels land in the 4 quarters of the square.
        for (auto& v : model.primitives[0].vertices)
            v.uv = { v.position[0], v.position[2] };
        return model;
    }

    size_t countColour(const RenderedImage& image, bool (*match)(const std::array<uint8_t, 4>&))
    {
        return static_cast<size_t>(std::count_if(image.rgba.begin(), image.rgba.end(), [&](const auto& p) {
            return p[3] != 0 && match(p);
        }));
    }
} // namespace

TEST(Rasteriser, DecodesTheTexturesOfAModel)
{
    const auto set = TextureSet::decode(texturedSquare());
    ASSERT_EQ(set.textures.size(), 1u);
    const auto& texture = set.textures[0];
    EXPECT_EQ(texture.width, 2);
    EXPECT_EQ(texture.height, 2);
    EXPECT_EQ(texture.sample(0.1f, 0.1f), (std::array<float, 4>{ 1, 0, 0, 1 }));
    EXPECT_EQ(texture.sample(0.9f, 0.1f), (std::array<float, 4>{ 0, 1, 0, 1 }));
    EXPECT_EQ(texture.sample(0.1f, 0.9f), (std::array<float, 4>{ 0, 0, 1, 1 }));
    EXPECT_FLOAT_EQ(texture.sample(0.9f, 0.9f)[3], 0);
    // Repeat wrapping, also for negative coordinates.
    EXPECT_EQ(texture.sample(1.1f, 1.1f), texture.sample(0.1f, 0.1f));
    EXPECT_EQ(texture.sample(-0.9f, 0.1f), texture.sample(0.1f, 0.1f));
}

TEST(Rasteriser, UndecodableImagesLeaveAnEmptyTexture)
{
    auto model = texturedSquare();
    model.images[0].bytes = { 1, 2, 3 };
    const auto set = TextureSet::decode(model);
    ASSERT_EQ(set.textures.size(), 1u);
    EXPECT_TRUE(set.textures[0].rgba.empty());
    EXPECT_EQ(set.textures[0].sample(0.5f, 0.5f), (std::array<float, 4>{ 1, 1, 1, 1 })); // neutral
}

TEST(Rasteriser, SamplesTexturesWhenGivenAndCutsOutTransparentTexels)
{
    const auto model = texturedSquare();
    const auto set = TextureSet::decode(model);

    auto settings = plain();
    settings.ambient = 1.0f; // no shading, so texel colours come through unchanged
    const auto flatColour = render(model, settings);

    settings.textures = &set;
    const auto textured = render(model, settings);
    ASSERT_FALSE(textured.empty());
    const auto red = countColour(textured, [](const auto& p) { return p[0] > 200 && p[1] < 50 && p[2] < 50; });
    const auto green = countColour(textured, [](const auto& p) { return p[1] > 200 && p[0] < 50 && p[2] < 50; });
    const auto blue = countColour(textured, [](const auto& p) { return p[2] > 200 && p[0] < 50 && p[1] < 50; });
    EXPECT_GT(red, 40u);
    EXPECT_GT(green, 40u);
    EXPECT_GT(blue, 40u);
    // The transparent quarter is cut out, leaving about three quarters of the plain silhouette.
    EXPECT_LT(textured.coveredPixels(), flatColour.coveredPixels() * 85 / 100);
    EXPECT_GT(textured.coveredPixels(), flatColour.coveredPixels() * 60 / 100);
}

TEST(Rasteriser, CutOutTexelsKeepWhatIsBehind)
{
    // A textured square raised in front of a plain blue one: its hole shows the blue square behind.
    auto textured = texturedSquare();
    auto both = squareModel(-1, 1, { 0, 0, 1, 1 }, true, textured);
    for (auto& v : both.primitives[0].vertices)
        v.position[1] += 2;
    auto settings = plain();
    settings.ambient = 1.0f;
    const auto set = TextureSet::decode(both);
    settings.textures = &set;
    auto isBlue = [](const auto& p) { return p[2] > 200 && p[0] < 50 && p[1] < 50; };
    const auto withBack = countColour(render(both, settings), isBlue);

    for (auto& v : textured.primitives[0].vertices)
        v.position[1] += 2;
    const auto aloneSet = TextureSet::decode(textured);
    settings.textures = &aloneSet;
    const auto alone = countColour(render(textured, settings), isBlue);
    EXPECT_GT(withBack, alone + 20);
}

TEST(Rasteriser, RemapColoursReplaceTheTexture)
{
    const auto model = texturedSquare();
    const auto set = TextureSet::decode(model);
    auto settings = plain();
    settings.ambient = 1.0f;
    settings.textures = &set;
    settings.materialColours = { std::array<float, 4>{ 0, 1, 0, 1 } };
    const auto image = render(model, settings);
    const auto notGreen = countColour(image, [](const auto& p) { return p[0] > 50 || p[2] > 50; });
    EXPECT_EQ(notGreen, 0u);
    EXPECT_GT(image.coveredPixels(), 100u); // and no cut-out hole either
}
