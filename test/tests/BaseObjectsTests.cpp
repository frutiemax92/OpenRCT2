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
#include <openrct2/park/Legacy.h>
#include <openrct2/actions/GameActionRunner.h>
#include <openrct2/actions/terraform/LandLowerAction.h>
#include <openrct2/actions/terraform/LandRaiseAction.h>
#include <openrct2/actions/terraform/LandSetHeightAction.h>
#include <openrct2/actions/terraform/SurfaceSetStyleAction.h>
#include <openrct2/world/MapSelection.h>
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
#include <openrct2/object/BuildingObject.h>
#include <openrct2/object/PathAdditionEntry.h>
#include <openrct2/object/PathAdditionObject.h>
#include <openrct2/object/StreamObject.h>
#include <openrct2/object/TerrainEdgeObject.h>
#include <openrct2/object/TerrainSurfaceObject.h>
#include <openrct2/actions/scenery/LargeSceneryPlaceAction.h>
#include <openrct2/actions/scenery/SmallSceneryPlaceAction.h>
#include <openrct2/golf/MapBallWorld.h>
#include <openrct2/object/LargeSceneryObject.h>
#include <openrct2/object/SmallSceneryEntry.h>
#include <openrct2/object/SmallSceneryObject.h>
#include <openrct2/object/WallObject.h>
#include <openrct2/world/TileElementsView.h>
#include <openrct2/world/tile_element/SmallSceneryElement.h>
#include <openrct2/world/tile_element/LargeSceneryElement.h>
#include <openrct2/drawing/TerrainSpriteGenerator.h>
#include <openrct2/drawing/TerrainSpriteStore.h>
#include <openrct2/ui/UiContext.h>
#include <openrct2/world/Map.h>
#include <openrct2/world/tile_element/SurfaceElement.h>
#include <openrct2/world/tile_element/Slope.h>
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
        // Icon, then normal, grid and underground entries of 19 slope images, then the square texture.
        EXPECT_EQ(1u + 3u * 19u + 1u, surface->GetImageTable().GetCount()) << identifier;
        EXPECT_EQ(3u, surface->NumEntries) << identifier;
        EXPECT_EQ(surface->IconImageId + 1, surface->EntryBaseImageId) << identifier;
    }
}

TEST_F(BaseObjects, TerrainSurfacesHaveASquareTileableTexture)
{
    for (auto identifier : kCommonScenarioAndTrackDesignerObjects)
    {
        if (!identifier.starts_with("golf.terrain_surface."))
            continue;
        auto* surface = static_cast<TerrainSurfaceObject*>(Load(identifier));
        ASSERT_NE(nullptr, surface) << identifier;
        EXPECT_TRUE(surface->HasTexture) << identifier;
        const auto* texture = surface->getTexture();
        ASSERT_NE(nullptr, texture) << identifier;
        EXPECT_EQ(64, texture->size) << identifier;
        EXPECT_GT(texture->smoothingPriority, 0) << identifier;
        EXPECT_EQ(surface->SmoothingPriority, texture->smoothingPriority) << identifier;
        EXPECT_EQ(static_cast<size_t>(Drawing::kTerrainZoomLevels), texture->mips.size()) << identifier;
        for (auto pixel : texture->mips[0])
            ASSERT_NE(Drawing::kTerrainTransparent, pixel) << identifier;
    }
}

TEST_F(BaseObjects, TerrainSurfacesHaveTheDefaultGolfProperties)
{
    struct Expected
    {
        const char* name;
        float roughness, bounciness, distance, accuracy;
        bool puttable, hazard;
    };
    const Expected table[] = {
        { "green", 0.05f, 0.30f, 1.00f, 1.00f, true, false },     { "tee", 0.10f, 0.40f, 1.00f, 1.00f, false, false },
        { "fairway", 0.15f, 0.45f, 1.00f, 1.00f, false, false },  { "grass", 0.35f, 0.35f, 0.90f, 0.90f, false, false },
        { "rough", 0.45f, 0.30f, 0.85f, 0.85f, false, false },    { "deep_rough", 0.75f, 0.20f, 0.60f, 0.65f, false, false },
        { "sand", 0.90f, 0.05f, 0.55f, 0.75f, false, true },      { "woods", 0.80f, 0.25f, 0.50f, 0.50f, false, false },
        { "rocks", 0.30f, 0.75f, 0.70f, 0.60f, false, false },
    };
    for (const auto& row : table)
    {
        const auto identifier = std::string("golf.terrain_surface.") + row.name;
        auto* surface = static_cast<TerrainSurfaceObject*>(Load(identifier));
        ASSERT_NE(nullptr, surface) << identifier;
        const auto& golf = surface->Golf;
        EXPECT_FLOAT_EQ(row.roughness, golf.roughness) << identifier;
        EXPECT_FLOAT_EQ(row.bounciness, golf.bounciness) << identifier;
        EXPECT_FLOAT_EQ(row.distance, golf.distanceFactor) << identifier;
        EXPECT_FLOAT_EQ(row.accuracy, golf.accuracyFactor) << identifier;
        EXPECT_EQ(row.puttable, golf.isPuttable) << identifier;
        EXPECT_EQ(row.hazard, golf.isHazard) << identifier;
        EXPECT_GT(golf.buildCost, 0) << identifier;
        EXPECT_GT(golf.removeCost, 0) << identifier;
    }
}

TEST_F(BaseObjects, TerrainEdgeHasAnIconAndAllEdgeImages)
{
    auto* edge = static_cast<TerrainEdgeObject*>(Load("golf.terrain_edge.rock_cliff"));
    ASSERT_NE(nullptr, edge);
    ASSERT_EQ(ObjectType::terrainEdge, edge->GetObjectType());
    // Icon, 44 edge images, 4 extra tunnel images and the square texture.
    EXPECT_EQ(50u, edge->GetImageTable().GetCount());
    ASSERT_TRUE(edge->HasTexture);
    const auto* texture = edge->getTexture();
    ASSERT_NE(nullptr, texture);
    EXPECT_EQ(64, texture->size);
    EXPECT_EQ(static_cast<size_t>(Drawing::kTerrainZoomLevels), texture->mips.size());
}

TEST_F(BaseObjects, ClimateLoads)
{
    auto* climate = Load("golf.climate.temperate");
    ASSERT_NE(nullptr, climate);
    EXPECT_EQ(ObjectType::climate, climate->GetObjectType());
}

TEST_F(BaseObjects, StreamLoadsItsProperties)
{
    auto* stream = static_cast<StreamObject*>(Load("golf.stream.creek"));
    ASSERT_NE(nullptr, stream);
    EXPECT_EQ(ObjectType::stream, stream->GetObjectType());
    EXPECT_EQ(12, stream->Width);
    EXPECT_GT(stream->BuildCost, 0);
    EXPECT_GT(stream->RemoveCost, 0);
    EXPECT_TRUE(stream->IsHazard);
    EXPECT_EQ(4, stream->FrameCount);
    EXPECT_EQ(8, stream->FrameDurationTicks);
    // Icon followed by one image per frame.
    EXPECT_EQ(5u, stream->GetImageTable().GetCount());
}

TEST_F(BaseObjects, StreamFramesAreSquareTextures)
{
    auto* stream = static_cast<StreamObject*>(Load("golf.stream.creek"));
    ASSERT_NE(nullptr, stream);
    ASSERT_EQ(4u, stream->Frames.size());
    for (const auto& frame : stream->Frames)
        EXPECT_EQ(64, frame.size);
    // Frame lookups wrap around.
    EXPECT_EQ(stream->getFrame(0), stream->getFrame(4));
    EXPECT_NE(stream->getFrame(0), stream->getFrame(1));
}

TEST_F(BaseObjects, BuildingLoadsItsProperties)
{
    auto* building = static_cast<BuildingObject*>(Load("golf.building.club_house"));
    ASSERT_NE(nullptr, building);
    EXPECT_EQ(ObjectType::building, building->GetObjectType());
    EXPECT_EQ(BuildingKind::golfClub, building->Kind);
    EXPECT_EQ(3, building->FootprintX);
    EXPECT_EQ(2, building->FootprintY);
    EXPECT_EQ(2, building->getFootprintX(1));
    EXPECT_EQ(3, building->getFootprintY(1));
    EXPECT_TRUE(building->HasEntrance);
    EXPECT_EQ(3, building->EntranceEdge);
    EXPECT_GT(building->BuildCost, 0);
    EXPECT_TRUE(building->Services.has(BuildingService::food));
    EXPECT_TRUE(building->Services.has(BuildingService::drink));
    EXPECT_TRUE(building->Services.has(BuildingService::toilet));
    EXPECT_TRUE(building->Services.has(BuildingService::proShop));
    // Icon followed by one view per camera rotation.
    EXPECT_EQ(5u, building->GetImageTable().GetCount());
    EXPECT_EQ(building->getViewImageId(0), building->getViewImageId(4));
}

TEST_F(BaseObjects, AllBuildingKindsLoadWithTheirProperties)
{
    struct BuildingExpectation
    {
        const char* id;
        BuildingKind kind;
        int32_t footprintX;
        int32_t footprintY;
        BuildingServices services;
    };

    BuildingServices clubServices{};
    clubServices.set(BuildingService::food);
    clubServices.set(BuildingService::drink);
    clubServices.set(BuildingService::toilet);
    clubServices.set(BuildingService::proShop);

    BuildingServices foodTruckServices{};
    foodTruckServices.set(BuildingService::food);
    foodTruckServices.set(BuildingService::drink);

    BuildingServices cottageServices{};
    cottageServices.set(BuildingService::rest);

    BuildingServices kioskServices{};
    kioskServices.set(BuildingService::proShop);

    BuildingServices restroomServices{};
    restroomServices.set(BuildingService::toilet);

    const BuildingExpectation expectations[] = {
        { "golf.building.club_house", BuildingKind::golfClub, 3, 2, clubServices },
        { "golf.building.food_truck", BuildingKind::foodTruck, 1, 1, foodTruckServices },
        { "golf.building.cottage", BuildingKind::cottage, 2, 2, cottageServices },
        { "golf.building.driving_range_kiosk", BuildingKind::drivingRangeKiosk, 1, 1, kioskServices },
        { "golf.building.cart_rental", BuildingKind::cartRental, 2, 1, BuildingServices{} },
        { "golf.building.restroom", BuildingKind::restroom, 1, 1, restroomServices },
    };

    for (const auto& exp : expectations)
    {
        auto* obj = Load(exp.id);
        ASSERT_NE(nullptr, obj) << "Failed to load " << exp.id;
        auto* bld = static_cast<BuildingObject*>(obj);
        EXPECT_EQ(bld->Kind, exp.kind) << exp.id;
        EXPECT_EQ(bld->FootprintX, exp.footprintX) << exp.id;
        EXPECT_EQ(bld->FootprintY, exp.footprintY) << exp.id;
        EXPECT_EQ(bld->Services, exp.services) << exp.id;
        EXPECT_TRUE(bld->HasEntrance) << exp.id;
    }
}

TEST_F(BaseObjects, PathAdditionsLoadWithTheirProperties)
{
    struct PathAdditionExpectation
    {
        const char* id;
        PathAdditionDrawType drawType;
        PathAdditionFlag flag;
    };

    const PathAdditionExpectation expectations[] = {
        { "golf.path_addition.bench", PathAdditionDrawType::bench, PathAdditionFlag::isBench },
        { "golf.path_addition.bin", PathAdditionDrawType::bin, PathAdditionFlag::isBin },
        { "golf.path_addition.lamp", PathAdditionDrawType::light, PathAdditionFlag::lamp },
    };

    for (const auto& exp : expectations)
    {
        auto* obj = Load(exp.id);
        ASSERT_NE(nullptr, obj) << "Failed to load " << exp.id;
        EXPECT_EQ(ObjectType::pathAdditions, obj->GetObjectType()) << exp.id;
        auto* entry = static_cast<PathAdditionEntry*>(obj->GetLegacyData());
        ASSERT_NE(nullptr, entry) << exp.id;
        EXPECT_EQ(exp.drawType, entry->draw_type) << exp.id;
        EXPECT_TRUE(entry->flags.has(exp.flag)) << exp.id;
        EXPECT_GT(entry->price, 0) << exp.id;
    }
}

TEST_F(BaseObjects, BuildingCollisionDefaultsToTheFootprintBox)
{
    auto* building = static_cast<BuildingObject*>(Load("golf.building.club_house"));
    ASSERT_NE(nullptr, building);
    EXPECT_EQ(CollisionShape::box, building->Collision.shape);
    EXPECT_EQ(building->Height, building->Collision.height);
    EXPECT_EQ(3 * 32, building->Collision.footprintX);
    EXPECT_EQ(2 * 32, building->Collision.footprintY);
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

TEST_F(BaseObjects, QuarterStepTerrainIsDrawnProcedurally)
{
    auto& objectManager = GetContext()->GetObjectManager();
    objectManager.UnloadAll();
    for (auto identifier : kMinimumRequiredObjects)
        ASSERT_NE(nullptr, objectManager.LoadObject(identifier)) << identifier;
    gameStateInitAll(getGameState(), { 16, 16 });
    Config::Get().general.multiThreading = false;

    // A quarter-step slope on the tile under the view centre.
    auto* surface = MapGetSurfaceElementAt(TileCoordsXY{ 8, 8 });
    ASSERT_NE(nullptr, surface);
    surface->setTerrainShape({ 1, 3, 0, 2 });

    constexpr int32_t kWidth = 128;
    constexpr int32_t kHeight = 64;
    Drawing::X8DrawingEngine engine(GetContext()->GetUiContext());
    std::vector<Drawing::PaletteIndex> pixels(kWidth * kHeight);
    Drawing::RenderTarget rt;
    rt.DrawingEngine = &engine;
    rt.bits = pixels.data();
    rt.width = kWidth;
    rt.height = kHeight;

    const CoordsXYZ centre{ 8 * kCoordsXYStep + 16, 8 * kCoordsXYStep + 16, TileElementHeight({ 8 * kCoordsXYStep, 8 * kCoordsXYStep }) };
    const auto screen = Translate3DTo2DWithZ(0, centre);
    Viewport viewport;
    viewport.width = kWidth;
    viewport.height = kHeight;
    viewport.viewPos = { screen.x - kWidth / 2, screen.y - kHeight / 2 };
    viewport.isVisible = true;

    auto& store = Drawing::TerrainSpriteStore::Instance();
    store.Clear();
    store.BeginFrame();
    engine.BeginDraw();
    ViewportRender(rt, &viewport);
    engine.EndDraw();
    // The sloped tile and its flat neighbours were generated and registered as images.
    EXPECT_GE(store.GetSpriteCount(), 2u);

    // Rendering the same view again reuses the cached sprites.
    const auto count = store.GetSpriteCount();
    store.BeginFrame();
    engine.BeginDraw();
    ViewportRender(rt, &viewport);
    engine.EndDraw();
    EXPECT_EQ(count, store.GetSpriteCount());

    // Unloading the terrain objects drops them.
    objectManager.UnloadAll();
    EXPECT_EQ(0u, store.GetSpriteCount());
}

TEST_F(BaseObjects, LandSetHeightStoresQuarterStepShapes)
{
    auto& objectManager = GetContext()->GetObjectManager();
    objectManager.UnloadAll();
    for (auto identifier : kMinimumRequiredObjects)
        ASSERT_NE(nullptr, objectManager.LoadObject(identifier)) << identifier;
    gameStateInitAll(getGameState(), { 16, 16 });
    getGameState().cheats.sandboxMode = true;

    const CoordsXY pos{ 5 * kCoordsXYStep, 6 * kCoordsXYStep };
    GameActions::LandSetHeightAction action(pos, 20, Terrain::TerrainShape{ 1, 3, 0, 4 });
    const auto result = GameActions::ExecuteNested(&action, getGameState());
    ASSERT_EQ(GameActions::Status::ok, result.error);
    auto* surface = MapGetSurfaceElementAt(pos);
    ASSERT_NE(nullptr, surface);
    EXPECT_EQ(20, surface->baseHeight);
    EXPECT_EQ((Terrain::TerrainShape{ 1, 3, 0, 4 }), surface->getTerrainShape());

    // Legacy slope flags still work, and an invalid shape is refused.
    GameActions::LandSetHeightAction legacy(pos, 22, static_cast<uint8_t>(kTileSlopeNCornerUp));
    ASSERT_EQ(GameActions::Status::ok, GameActions::ExecuteNested(&legacy, getGameState()).error);
    EXPECT_EQ((Terrain::TerrainShape{ 2, 0, 0, 0 }), surface->getTerrainShape());
    GameActions::LandSetHeightAction invalid(pos, 22, Terrain::TerrainShape{ 1, 1, 1, 1 });
    EXPECT_NE(GameActions::Status::ok, GameActions::Query(&invalid, getGameState()).error);
}

class LandTools : public BaseObjects
{
protected:
    static void SetUpMap()
    {
        auto& objectManager = GetContext()->GetObjectManager();
        objectManager.UnloadAll();
        for (auto identifier : kMinimumRequiredObjects)
            ASSERT_NE(nullptr, objectManager.LoadObject(identifier)) << identifier;
        gameStateInitAll(getGameState(), { 32, 32 });
        getGameState().cheats.sandboxMode = true;
    }

    static MapRange Tile(int x, int y)
    {
        return { x * kCoordsXYStep, y * kCoordsXYStep, x * kCoordsXYStep, y * kCoordsXYStep };
    }

    static Terrain::TerrainShape ShapeAt(int x, int y)
    {
        return MapGetSurfaceElementAt(TileCoordsXY{ x, y })->getTerrainShape();
    }
};

TEST_F(LandTools, RaisingACornerByAQuarterMakesAQuarterSlope)
{
    SetUpMap();
    const auto base = MapGetSurfaceElementAt(TileCoordsXY{ 10, 10 })->baseHeight;
    GameActions::LandRaiseAction raise({ 10 * 32, 10 * 32 }, Tile(10, 10), MapSelectType::corner0, 1); // N corner
    ASSERT_EQ(GameActions::Status::ok, GameActions::ExecuteNested(&raise, getGameState()).error);
    EXPECT_EQ((Terrain::TerrainShape{ 1, 0, 0, 0 }), ShapeAt(10, 10));
    EXPECT_EQ(base, MapGetSurfaceElementAt(TileCoordsXY{ 10, 10 })->baseHeight);
    // The N corner of (10, 10) is shared: the S corner of (11, 11), the E of (10, 11), the W of (11, 10).
    EXPECT_EQ((Terrain::TerrainShape{ 0, 0, 1, 0 }), ShapeAt(11, 11));
    EXPECT_EQ((Terrain::TerrainShape{ 0, 1, 0, 0 }), ShapeAt(10, 11));
    EXPECT_EQ((Terrain::TerrainShape{ 0, 0, 0, 1 }), ShapeAt(11, 10));
    EXPECT_EQ((Terrain::TerrainShape{ 0, 0, 0, 0 }), ShapeAt(12, 12));
}

TEST_F(LandTools, DefaultStepIsAHalfStepAndLoweringUndoesIt)
{
    SetUpMap();
    GameActions::LandRaiseAction raise({ 10 * 32, 10 * 32 }, Tile(10, 10), MapSelectType::corner1);
    ASSERT_EQ(GameActions::Status::ok, GameActions::ExecuteNested(&raise, getGameState()).error);
    EXPECT_EQ((Terrain::TerrainShape{ 0, 2, 0, 0 }), ShapeAt(10, 10)); // same as the legacy slope step
    GameActions::LandLowerAction lower({ 10 * 32, 10 * 32 }, Tile(10, 10), MapSelectType::corner1);
    ASSERT_EQ(GameActions::Status::ok, GameActions::ExecuteNested(&lower, getGameState()).error);
    EXPECT_EQ((Terrain::TerrainShape{ 0, 0, 0, 0 }), ShapeAt(10, 10));
    EXPECT_EQ((Terrain::TerrainShape{ 0, 0, 0, 0 }), ShapeAt(11, 9));
}

TEST_F(LandTools, RaisingAWholeTileRaisesItsBase)
{
    SetUpMap();
    const auto base = MapGetSurfaceElementAt(TileCoordsXY{ 10, 10 })->baseHeight;
    GameActions::LandRaiseAction raise({ 10 * 32, 10 * 32 }, Tile(10, 10), MapSelectType::full, 3);
    ASSERT_EQ(GameActions::Status::ok, GameActions::ExecuteNested(&raise, getGameState()).error);
    EXPECT_EQ(base + 3, MapGetSurfaceElementAt(TileCoordsXY{ 10, 10 })->baseHeight);
    EXPECT_EQ((Terrain::TerrainShape{ 0, 0, 0, 0 }), ShapeAt(10, 10));
    // Its neighbours slope up to it by 3 Q on the shared edge.
    EXPECT_EQ((Terrain::TerrainShape{ 3, 3, 0, 0 }), ShapeAt(9, 10));
}

TEST_F(LandTools, SteepRaisesSmoothTheSurroundings)
{
    SetUpMap();
    for (int i = 0; i < 3; i++)
    {
        GameActions::LandRaiseAction raise({ 10 * 32, 10 * 32 }, Tile(10, 10), MapSelectType::corner2, 4);
        ASSERT_EQ(GameActions::Status::ok, GameActions::ExecuteNested(&raise, getGameState()).error);
    }
    // Every tile of the map still fits the shape model.
    for (int y = 0; y < 32; y++)
        for (int x = 0; x < 32; x++)
            ASSERT_TRUE(Terrain::IsValidShape(ShapeAt(x, y))) << x << "," << y;
    EXPECT_TRUE(ShapeAt(9, 9).north > 0 || ShapeAt(9, 9).east > 0 || ShapeAt(9, 9).south > 0 || ShapeAt(9, 9).west > 0);
}

TEST_F(LandTools, InvalidStepsAndLimitsAreRefused)
{
    SetUpMap();
    GameActions::LandRaiseAction zero({ 10 * 32, 10 * 32 }, Tile(10, 10), MapSelectType::full, 0);
    EXPECT_NE(GameActions::Status::ok, GameActions::Query(&zero, getGameState()).error);
    GameActions::LandRaiseAction big({ 10 * 32, 10 * 32 }, Tile(10, 10), MapSelectType::full, 5);
    EXPECT_NE(GameActions::Status::ok, GameActions::Query(&big, getGameState()).error);
    GameActions::LandLowerAction tooLow({ 10 * 32, 10 * 32 }, Tile(10, 10), MapSelectType::full, 4);
    // Default land is at 14 Q: lowering by 4 repeatedly hits the minimum (2 Q) and is refused without changes.
    for (int i = 0; i < 3; i++)
        GameActions::ExecuteNested(&tooLow, getGameState());
    EXPECT_GE(MapGetSurfaceElementAt(TileCoordsXY{ 10, 10 })->baseHeight, kMinimumLandHeight);
}

TEST_F(LandTools, PaintingQuadrantsAndWholeTiles)
{
    SetUpMap();
    auto* surface = MapGetSurfaceElementAt(TileCoordsXY{ 4, 4 });
    ASSERT_NE(nullptr, surface);
    const auto original = surface->getSurfaceObjectIndex();
    for (auto identifier : kCommonScenarioAndTrackDesignerObjects)
        GetContext()->GetObjectManager().LoadObject(identifier);
    ObjectEntryIndex other = original;
    for (ObjectEntryIndex i = 0; i < 20 && other == original; i++)
    {
        if (i != original && TerrainSurfaceObject::GetById(i) != nullptr)
            other = i;
    }
    ASSERT_NE(original, other);

    // Only the south-east quadrant.
    GameActions::SurfaceSetStyleAction half(
        Tile(4, 4), other, kObjectEntryIndexNull, Drawing::Colour::black, Drawing::Colour::black,
        1 << static_cast<int>(Terrain::Quadrant::southEast));
    ASSERT_EQ(GameActions::Status::ok, GameActions::ExecuteNested(&half, getGameState()).error);
    EXPECT_EQ(other, surface->getQuadrantSurfaceIndex(Terrain::Quadrant::southEast));
    EXPECT_EQ(original, surface->getQuadrantSurfaceIndex(Terrain::Quadrant::northEast));
    EXPECT_EQ(original, surface->getQuadrantSurfaceIndex(Terrain::Quadrant::southWest));
    EXPECT_EQ(original, surface->getQuadrantSurfaceIndex(Terrain::Quadrant::northWest));

    // Two quadrants on one side of a diagonal (half-tile brush).
    GameActions::SurfaceSetStyleAction side(
        Tile(4, 4), other, kObjectEntryIndexNull, Drawing::Colour::black, Drawing::Colour::black,
        (1 << static_cast<int>(Terrain::Quadrant::northEast)) | (1 << static_cast<int>(Terrain::Quadrant::northWest)));
    ASSERT_EQ(GameActions::Status::ok, GameActions::ExecuteNested(&side, getGameState()).error);
    EXPECT_EQ(other, surface->getQuadrantSurfaceIndex(Terrain::Quadrant::northWest));
    EXPECT_EQ(original, surface->getQuadrantSurfaceIndex(Terrain::Quadrant::southWest));

    // The default paints the whole tile.
    GameActions::SurfaceSetStyleAction whole(Tile(4, 4), original, kObjectEntryIndexNull, Drawing::Colour::black, Drawing::Colour::black);
    ASSERT_EQ(GameActions::Status::ok, GameActions::ExecuteNested(&whole, getGameState()).error);
    for (uint8_t q = 0; q < 4; q++)
        EXPECT_EQ(original, surface->getQuadrantSurfaceIndex(static_cast<Terrain::Quadrant>(q)));

    // Painting what is already there costs nothing.
    GameActions::SurfaceSetStyleAction same(Tile(4, 4), original, kObjectEntryIndexNull, Drawing::Colour::black, Drawing::Colour::black);
    EXPECT_EQ(0, GameActions::Query(&same, getGameState()).cost);
}

TEST_F(LandTools, TileElementHeightFollowsTheTriangleSplit)
{
    SetUpMap();
    auto* surface = MapGetSurfaceElementAt(TileCoordsXY{ 6, 7 });
    const Terrain::TerrainShape shape{ 1, 3, 0, 4 };
    surface->setTerrainShape(shape);
    const int32_t baseZ = surface->getBaseZ();
    for (int y = 0; y < 32; y += 5)
        for (int x = 0; x < 32; x += 3)
        {
            const CoordsXY pos{ 6 * 32 + x, 7 * 32 + y };
            EXPECT_EQ(Terrain::GetSurfaceZAt(baseZ, shape, x, y), TileElementHeight(pos)) << x << "," << y;
        }
    // A flat tile is flat everywhere, and off the map is the minimum land height.
    EXPECT_EQ(MapGetSurfaceElementAt(TileCoordsXY{ 2, 2 })->getBaseZ(), TileElementHeight(CoordsXY{ 2 * 32 + 17, 2 * 32 + 3 }));
    EXPECT_EQ(kMinimumLandZ, TileElementHeight(CoordsXY{ -5, -5 }));
}

#include <openrct2/world/map_generator/MapGen.h>

TEST_F(LandTools, GeneratedTerrainIsContinuousQuarterStepLand)
{
    SetUpMap();
    World::MapGenerator::Settings settings;
    settings.algorithm = World::MapGenerator::Algorithm::simplexNoise;
    settings.mapSize = { 48, 48 };
    settings.trees = false;
    settings.beaches = false;
    settings.smoothTileEdges = true;
    World::MapGenerator::generate(&settings);

    bool anyQuarter = false;
    for (int y = 1; y < 47; y++)
    {
        for (int x = 1; x < 47; x++)
        {
            const auto* s = MapGetSurfaceElementAt(TileCoordsXY{ x, y });
            ASSERT_TRUE(Terrain::IsValidShape(s->getTerrainShape())) << x << "," << y;
            const auto sh = s->getTerrainShape();
            anyQuarter |= (sh.north | sh.east | sh.south | sh.west) & 1;

            // No cliffs towards the next tile in +x: the shared edge has the same heights.
            if (x + 1 >= 47)
                continue;
            const auto* r = MapGetSurfaceElementAt(TileCoordsXY{ x + 1, y });
            const auto rs = r->getTerrainShape();
            EXPECT_EQ(s->baseHeight + sh.east, r->baseHeight + rs.south) << x << "," << y; // corner (x+1, y)
            EXPECT_EQ(s->baseHeight + sh.north, r->baseHeight + rs.west) << x << "," << y; // corner (x+1, y+1)
        }
    }
    EXPECT_TRUE(anyQuarter);
}

TEST_F(LandTools, GeneratedTerracesHaveFlatTilesWhenEdgesAreNotSmoothed)
{
    SetUpMap();
    World::MapGenerator::Settings settings;
    settings.algorithm = World::MapGenerator::Algorithm::simplexNoise;
    settings.mapSize = { 40, 40 };
    settings.trees = false;
    settings.beaches = false;
    settings.smoothTileEdges = false;
    World::MapGenerator::generate(&settings);
    for (int y = 1; y < 39; y++)
        for (int x = 1; x < 39; x++)
            ASSERT_EQ((Terrain::TerrainShape{ 0, 0, 0, 0 }), ShapeAt(x, y)) << x << "," << y;
}

TEST_F(LandTools, WaterHeightIsStoredInQuarterSteps)
{
    SetUpMap();
    auto* surface = MapGetSurfaceElementAt(TileCoordsXY{ 3, 3 });
    for (int q = 1; q <= 9; q++)
    {
        surface->setWaterHeight(q * kCoordsZStep);
        EXPECT_EQ(q * kCoordsZStep, surface->getWaterHeight()) << q; // odd quarter steps survive
    }
    surface->setWaterHeight(0);
    EXPECT_EQ(0, surface->getWaterHeight());
    EXPECT_EQ(kCoordsZStep, kWaterHeightStep);
}

TEST(LegacySurfaceLayout, OldParkSurfacesConvertToQuarterSteps)
{
    // Old layout bytes: type, flags, base, clearance, owner, slope, water, grass, ownership, style, edge, colour, edge
    // colour, 3 padding.
    TileElement el{};
    auto* raw = reinterpret_cast<uint8_t*>(&el);
    raw[0] = static_cast<uint8_t>(TileElementType::surface) << 2;
    raw[2] = 20;
    raw[3] = 22;
    raw[5] = kTileSlopeNCornerUp | kTileSlopeECornerUp; // north-east side up
    raw[6] = 10;                                        // water: 10 half steps
    raw[9] = 7;                                         // surface style
    convertLegacySurfaceLayout(el);

    const auto* surface = el.asSurface();
    EXPECT_EQ((Terrain::TerrainShape{ 2, 2, 0, 0 }), surface->getTerrainShape());
    EXPECT_EQ(20, el.baseHeight);
    EXPECT_EQ(10 * 2 * kCoordsZStep, surface->getWaterHeight()); // the same height in z units as before
    for (uint8_t q = 0; q < 4; q++)
        EXPECT_EQ(7, surface->getQuadrantSurfaceIndex(static_cast<Terrain::Quadrant>(q)));

    // All corners raised: a higher base, flat.
    TileElement raised{};
    auto* rawRaised = reinterpret_cast<uint8_t*>(&raised);
    rawRaised[0] = static_cast<uint8_t>(TileElementType::surface) << 2;
    rawRaised[2] = 20;
    rawRaised[3] = 20;
    rawRaised[5] = kTileSlopeRaisedCornersMask;
    convertLegacySurfaceLayout(raised);
    EXPECT_EQ(22, raised.baseHeight);
    EXPECT_EQ((Terrain::TerrainShape{ 0, 0, 0, 0 }), raised.asSurface()->getTerrainShape());
}

TEST_F(LandTools, GridLinesAndSelectionsAreDrawnProceduralOverlays)
{
    SetUpMap();
    MapGetSurfaceElementAt(TileCoordsXY{ 8, 8 })->setTerrainShape({ 1, 2, 0, 3 });

    constexpr int32_t kWidth = 128, kHeight = 64;
    Drawing::X8DrawingEngine engine(GetContext()->GetUiContext());
    std::vector<Drawing::PaletteIndex> pixels(kWidth * kHeight);
    Drawing::RenderTarget rt;
    rt.DrawingEngine = &engine;
    rt.bits = pixels.data();
    rt.width = kWidth;
    rt.height = kHeight;
    const CoordsXYZ centre{ 8 * kCoordsXYStep + 16, 8 * kCoordsXYStep + 16, MapGetSurfaceElementAt(TileCoordsXY{ 8, 8 })->getBaseZ() };
    const auto screen = Translate3DTo2DWithZ(0, centre);
    Viewport viewport;
    viewport.width = kWidth;
    viewport.height = kHeight;
    viewport.viewPos = { screen.x - kWidth / 2, screen.y - kHeight / 2 };
    viewport.isVisible = true;

    auto& store = Drawing::TerrainSpriteStore::Instance();
    auto render = [&] {
        store.BeginFrame();
        engine.BeginDraw();
        ViewportRender(rt, &viewport);
        engine.EndDraw();
    };
    store.Clear();
    render();
    const auto plain = store.GetSpriteCount();

    viewport.flags.set(ViewportFlag::gridlines);
    render();
    const auto withGrid = store.GetSpriteCount();
    EXPECT_GT(withGrid, plain); // grid outlines were generated for the visible tiles

    gMapSelectFlags.set(MapSelectFlag::enable);
    gMapSelectPositionA = { 8 * kCoordsXYStep, 8 * kCoordsXYStep };
    gMapSelectPositionB = gMapSelectPositionA;
    gMapSelectType = MapSelectType::corner1;
    render();
    EXPECT_GT(store.GetSpriteCount(), withGrid); // and the corner highlight
    gMapSelectFlags.clearAll();
}

TEST_F(BaseObjects, MiniPuttObstaclesLoadWithTheirProperties)
{
    auto* bumper = Load("golf.scenery_small.bumper");
    ASSERT_NE(nullptr, bumper);
    auto* smallBumper = dynamic_cast<SmallSceneryObject*>(bumper);
    ASSERT_NE(nullptr, smallBumper);
    EXPECT_EQ(CollisionShape::cylinder, smallBumper->Collision.shape);
    EXPECT_EQ(6, smallBumper->Collision.radius);
    EXPECT_FLOAT_EQ(0.9f, smallBumper->Collision.restitution);

    auto* ramp = Load("golf.scenery_small.ramp");
    ASSERT_NE(nullptr, ramp);
    auto* smallRamp = dynamic_cast<SmallSceneryObject*>(ramp);
    ASSERT_NE(nullptr, smallRamp);
    EXPECT_EQ(CollisionShape::box, smallRamp->Collision.shape);
    EXPECT_EQ(32, smallRamp->Collision.footprintX);
    EXPECT_FLOAT_EQ(0.5f, smallRamp->Collision.restitution);

    auto* wallObj = Load("golf.wall.low_wall");
    ASSERT_NE(nullptr, wallObj);
    auto* lowWall = dynamic_cast<WallObject*>(wallObj);
    ASSERT_NE(nullptr, lowWall);
    EXPECT_EQ(CollisionShape::box, lowWall->Collision.shape);
    EXPECT_FLOAT_EQ(0.85f, lowWall->Collision.restitution);
}

TEST_F(LandTools, WoodsSurfaceSpawnsTreeObstacles)
{
    SetUpMap();
    auto& objManager = GetContext()->GetObjectManager();
    auto* woodsObj = Load("golf.terrain_surface.woods");
    ASSERT_NE(nullptr, woodsObj);
    auto* bumper = Load("golf.scenery_small.bumper");
    ASSERT_NE(nullptr, bumper);

    auto* smallScenery = dynamic_cast<SmallSceneryObject*>(bumper);
    ASSERT_NE(nullptr, smallScenery);
    auto* legacyEntry = reinterpret_cast<SmallSceneryEntry*>(smallScenery->GetLegacyData());
    legacyEntry->flags.set(SmallSceneryFlag::isTree);

    const auto woodsIdx = objManager.GetLoadedObjectEntryIndex("golf.terrain_surface.woods");
    ASSERT_NE(kObjectEntryIndexNull, woodsIdx);

    // Paint without spawnTrees: no tree placed
    GameActions::SurfaceSetStyleAction paintNoTrees(
        Tile(5, 5), woodsIdx, kObjectEntryIndexNull, Drawing::Colour::black, Drawing::Colour::black, 0xF, false);
    ASSERT_EQ(GameActions::Status::ok, GameActions::ExecuteNested(&paintNoTrees, getGameState()).error);
    int sceneryCount = 0;
    for (auto* el : TileElementsView<TileElement>(TileCoordsXY{ 5, 5 }.toCoordsXY()))
        if (el->getType() == TileElementType::smallScenery) sceneryCount++;
    EXPECT_EQ(0, sceneryCount);

    // Paint with spawnTrees: tree placed
    GameActions::SurfaceSetStyleAction paintTrees(
        Tile(6, 6), woodsIdx, kObjectEntryIndexNull, Drawing::Colour::black, Drawing::Colour::black, 0xF, true);
    const auto queryRes = GameActions::Query(&paintTrees, getGameState());
    EXPECT_EQ(GameActions::Status::ok, queryRes.error);
    EXPECT_GT(queryRes.cost, 0); // includes tree cost

    const auto execRes = GameActions::ExecuteNested(&paintTrees, getGameState());
    ASSERT_EQ(GameActions::Status::ok, execRes.error);
    sceneryCount = 0;
    for (auto* el : TileElementsView<TileElement>(TileCoordsXY{ 6, 6 }.toCoordsXY()))
        if (el->getType() == TileElementType::smallScenery) sceneryCount++;
    EXPECT_EQ(1, sceneryCount);
}

TEST_F(LandTools, GrassGrowthIncreasesRoughness)
{
    SetUpMap();
    auto* surface = MapGetSurfaceElementAt(TileCoordsXY{ 4, 4 });
    ASSERT_NE(nullptr, surface);
    Golf::MapBallWorld world;

    const Fixed x = Fixed::fromInt(4 * 32 + 16);
    const Fixed y = Fixed::fromInt(4 * 32 + 16);

    surface->setGrassLength(GRASS_LENGTH_MOWED);
    const auto baseSample = world.surfaceAt(x, y);

    surface->setGrassLength(GRASS_LENGTH_CLEAR_0); // stage 1: +0.05
    const auto mownSample = world.surfaceAt(x, y);
    EXPECT_NEAR(mownSample.roughness.toDouble(), baseSample.roughness.toDouble() + 0.05, 0.01);

    surface->setGrassLength(GRASS_LENGTH_CLUMPS_2); // stage 6: +0.30
    const auto clumpsSample = world.surfaceAt(x, y);
    EXPECT_NEAR(clumpsSample.roughness.toDouble(), baseSample.roughness.toDouble() + 0.30, 0.01);

    // Mowing returns it to base
    surface->setGrassLength(GRASS_LENGTH_MOWED);
    const auto clearedSample = world.surfaceAt(x, y);
    EXPECT_FLOAT_EQ(baseSample.roughness.toDouble(), clearedSample.roughness.toDouble());
}

TEST_F(LandTools, LargeSceneryPlacementSlopeConstraints)
{
    SetUpMap();
    auto* surface = MapGetSurfaceElementAt(TileCoordsXY{ 8, 8 });
    ASSERT_NE(nullptr, surface);

    // 1 Q slope on tile (within 1 Q of flat): allowed
    surface->setTerrainShape({ 1, 0, 0, 0 });
    const auto corners1 = Terrain::GetCornerHeights(surface->getBaseZ(), surface->getTerrainShape());
    const int32_t diff1 = corners1.north - corners1.east;
    EXPECT_EQ(Terrain::kQuarterStepZ, diff1);

    // 2 Q slope on tile (> 1 Q): exceeds 1 Q limit
    surface->setTerrainShape({ 2, 0, 0, 0 });
    const auto corners2 = Terrain::GetCornerHeights(surface->getBaseZ(), surface->getTerrainShape());
    const int32_t diff2 = corners2.north - corners2.east;
    EXPECT_EQ(Terrain::kQuarterStepZ * 2, diff2);
}

