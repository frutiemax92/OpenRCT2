/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

// Base object pack: the objects the map and the scenario editor need by default are the game's own
// (data/object/golf, scripts/placeholders/objects.py), not RCT or OpenRCT2 objects.

#include <gtest/gtest.h>
#include <openrct2/Context.h>
#include <openrct2/GameState.h>
#include <openrct2/OpenRCT2.h>
#include <openrct2/config/Config.h>
#include <openrct2/drawing/RenderTarget.h>
#include <openrct2/drawing/X8DrawingEngine.h>
#include <openrct2/interface/Viewport.h>
#include <openrct2/object/DefaultObjects.h>
#include <openrct2/object/Object.h>
#include <openrct2/object/ObjectManager.h>
#include <openrct2/object/ObjectRepository.h>
#include <openrct2/object/TerrainEdgeObject.h>
#include <openrct2/object/TerrainSurfaceObject.h>
#include <openrct2/ui/UiContext.h>
#include <openrct2/world/Map.h>
#include <string>
#include <vector>

using namespace OpenRCT2;

class BaseObjects : public testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        gOpenRCT2Headless = true;
        gOpenRCT2NoGraphics = false; // load the images, to check the image tables
        _context = CreateContext();
        ASSERT_TRUE(_context->Initialise());
    }

    static void TearDownTestSuite()
    {
        _context = nullptr;
    }

    static Object* Load(std::string_view identifier)
    {
        auto& repository = GetContext()->GetObjectRepository();
        const auto* item = repository.FindObject(identifier);
        if (item == nullptr)
            return nullptr;
        return GetContext()->GetObjectManager().LoadObject(identifier);
    }

    static std::unique_ptr<IContext> _context;
};

std::unique_ptr<IContext> BaseObjects::_context;

TEST_F(BaseObjects, DefaultObjectsAreOurOwn)
{
    for (auto identifier : kCommonScenarioAndTrackDesignerObjects)
    {
        EXPECT_TRUE(identifier.starts_with("golf.")) << identifier;
        auto* object = Load(identifier);
        ASSERT_NE(nullptr, object) << identifier;
        EXPECT_EQ(ObjectSourceGame::custom, object->GetSourceGames().at(0)) << identifier;
    }
}

TEST_F(BaseObjects, MinimumRequiredObjectsLoad)
{
    for (auto identifier : kMinimumRequiredObjects)
    {
        ASSERT_NE(nullptr, Load(identifier)) << identifier;
    }
    for (auto identifier : kMinimumRequiredObjectsDesigner)
    {
        ASSERT_NE(nullptr, Load(identifier)) << identifier;
    }
}

TEST_F(BaseObjects, TerrainSurfacesHaveThreeEntries)
{
    for (auto identifier : kCommonScenarioAndTrackDesignerObjects)
    {
        if (!identifier.starts_with("golf.terrain_surface."))
            continue;
        auto* surface = static_cast<TerrainSurfaceObject*>(Load(identifier));
        ASSERT_NE(nullptr, surface) << identifier;
        ASSERT_EQ(ObjectType::terrainSurface, surface->GetObjectType());
        // Icon, then normal, grid and underground entries of 19 slope images.
        EXPECT_EQ(1u + 3u * 19u, surface->GetImageTable().GetCount()) << identifier;
        EXPECT_EQ(3u, surface->NumEntries) << identifier;
        EXPECT_EQ(surface->IconImageId + 1, surface->EntryBaseImageId) << identifier;
    }
}

TEST_F(BaseObjects, TerrainEdgeHasAnIconAndAllEdgeImages)
{
    auto* edge = static_cast<TerrainEdgeObject*>(Load("golf.terrain_edge.rock_cliff"));
    ASSERT_NE(nullptr, edge);
    ASSERT_EQ(ObjectType::terrainEdge, edge->GetObjectType());
    // Icon, 44 edge images and 4 extra tunnel images.
    EXPECT_EQ(49u, edge->GetImageTable().GetCount());
}

TEST_F(BaseObjects, ClimateLoads)
{
    auto* climate = Load("golf.climate.temperate");
    ASSERT_NE(nullptr, climate);
    EXPECT_EQ(ObjectType::climate, climate->GetObjectType());
}

TEST_F(BaseObjects, TerrainIsDrawn)
{
    // An empty map with the default terrain renders terrain pixels where the viewport looks at the map.
    auto& objectManager = GetContext()->GetObjectManager();
    objectManager.UnloadAll();
    for (auto identifier : kMinimumRequiredObjects)
        ASSERT_NE(nullptr, objectManager.LoadObject(identifier)) << identifier;
    gameStateInitAll(getGameState(), { 16, 16 });
    Config::Get().general.multiThreading = false;

    constexpr int32_t kWidth = 128;
    constexpr int32_t kHeight = 64;
    constexpr auto kGarbage = static_cast<Drawing::PaletteIndex>(0xAB);
    Drawing::X8DrawingEngine engine(GetContext()->GetUiContext());
    std::vector<Drawing::PaletteIndex> pixels(kWidth * kHeight, kGarbage);
    Drawing::RenderTarget rt;
    rt.DrawingEngine = &engine;
    rt.bits = pixels.data();
    rt.width = kWidth;
    rt.height = kHeight;

    const CoordsXYZ centre{ 8 * kCoordsXYStep, 8 * kCoordsXYStep, TileElementHeight({ 8 * kCoordsXYStep, 8 * kCoordsXYStep }) };
    const auto screen = Translate3DTo2DWithZ(0, centre);
    Viewport viewport;
    viewport.width = kWidth;
    viewport.height = kHeight;
    viewport.viewPos = { screen.x - kWidth / 2, screen.y - kHeight / 2 };
    viewport.isVisible = true;

    engine.BeginDraw();
    ViewportRender(rt, &viewport);
    engine.EndDraw();

    size_t terrain = 0;
    for (auto pixel : pixels)
    {
        ASSERT_NE(kGarbage, pixel);
        if (pixel != kViewportVoidColour)
            terrain++;
    }
    EXPECT_GT(terrain, pixels.size() / 2);
}
