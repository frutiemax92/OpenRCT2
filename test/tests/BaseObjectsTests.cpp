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
#include <openrct2/object/TerrainEdgeObject.h>
#include <openrct2/object/TerrainSurfaceObject.h>
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
