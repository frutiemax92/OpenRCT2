/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "TestData.h"

#include <gtest/gtest.h>
#include <memory>
#include <optional>
#include <openrct2/Context.h>
#include <openrct2/Game.h>
#include <openrct2/GameState.h>
#include <openrct2/OpenRCT2.h>
#include <openrct2/ParkImporter.h>
#include <openrct2/actions/GameActionRunner.h>
#include <openrct2/actions/footpath/FootpathPlaceAction.h>
#include <openrct2/actions/scenery/BuildingPlaceAction.h>
#include <openrct2/actions/scenery/BuildingRemoveAction.h>
#include <openrct2/drawing/Drawing.h>
#include <openrct2/drawing/Palette.h>
#include <openrct2/entity/EntityRegistry.h>
#include <openrct2/entity/EntityTweener.h>
#include <openrct2/object/ObjectManager.h>
#include <openrct2/object/BuildingObject.h>
#include <openrct2/world/Map.h>
#include <openrct2/world/MapAnimation.h>
#include <openrct2/world/Building.h>
#include <openrct2/world/Footpath.h>
#include <openrct2/actions/golf/BuildingEditAction.h>
#include <openrct2/actions/golf/CartEditAction.h>
#include <openrct2/actions/golf/RangeEditAction.h>
#include <openrct2/actions/golf/TournamentEditAction.h>
#include <openrct2/actions/golf/ZoneEditAction.h>
#include <openrct2/world/MapOwnership.h>
#include <openrct2/actions/scenery/SmallSceneryPlaceAction.h>
#include <openrct2/golf/DrivingRange.h>
#include <openrct2/golf/MapObstacles.h>
#include <openrct2/actions/scenery/ParkingPlaceAction.h>
#include <openrct2/world/Parking.h>
#include <openrct2/world/Street.h>
#include <openrct2/actions/scenery/ParkingRemoveAction.h>
#include <openrct2/actions/scenery/StreetPlaceAction.h>
#include <openrct2/actions/scenery/StreetRoadEntranceAction.h>
#include <openrct2/entity/AnimalEntity.h>
#include <openrct2/entity/GolfBall.h>
#include <openrct2/entity/GolfVehicleEntity.h>
#include <openrct2/entity/GolferEntity.h>
#include <openrct2/object/AnimalObject.h>
#include <openrct2/object/VehicleModelObject.h>
#include <openrct2/model/Rasteriser.h>
#include <openrct2/golf/GolfRuntime.h>
#include <openrct2/drawing/ModelSpriteStore.h>
#include <openrct2/management/Finance.h>
#include <openrct2/management/NewsItem.h>
#include <openrct2/object/PathAdditionObject.h>
#include <openrct2/golf/Hole.h>
#include <openrct2/object/GolferModelObject.h>
#include <openrct2/paint/entity/Paint.Animal.h>
#include <openrct2/paint/entity/Paint.Golfer.h>
#include <openrct2/world/TileElementsView.h>
#include <openrct2/world/tile_element/PathElement.h>
#include <openrct2/world/tile_element/BuildingElement.h>
#include <openrct2/world/tile_element/SurfaceElement.h>

using namespace OpenRCT2;

namespace
{
    std::unique_ptr<IContext> startPark()
    {
        gOpenRCT2Headless = true;
        gOpenRCT2NoGraphics = true;

        auto context = CreateContext();
        if (!context->Initialise())
            return {};

        auto importer = ParkImporter::CreateParkFile(context->GetObjectRepository());
        auto loadResult = importer->LoadSavedGame(TestData::GetParkPath("small_park_with_ferris_wheel.park").c_str(), false);
        context->GetObjectManager().LoadObjects(loadResult.RequiredObjects);

        MapAnimations::ClearAll();
        auto& gameState = getGameState();
        importer->Import(gameState);
        gameState.entities.resetEntitySpatialIndices();
        ResetAllSpriteQuadrantPlacements();
        Drawing::LoadPalette();
        EntityTweener::get().reset();
        gGameSpeed = 1;
        return context;
    }

    bool isBareFlat(const CoordsXY& tile, int32_t z)
    {
        if (!MapIsLocationValid(tile))
            return false;
        auto* surface = MapGetSurfaceElementAt(tile);
        if (surface == nullptr || surface->getWaterHeight() > 0 || surface->getSlope() != 0 || surface->getBaseZ() != z)
            return false;
        int count = 0;
        for (auto* element : TileElementsView<TileElement>(tile))
        {
            (void)element;
            count++;
        }
        return count == 1;
    }

    // The corner of a bare, flat w x h block of tiles with a bare ring around it.
    std::optional<CoordsXY> findFlatBlock(int32_t w, int32_t h)
    {
        for (int32_t y = 4; y < 60; y++)
        {
            for (int32_t x = 4; x < 60; x++)
            {
                const CoordsXY corner{ x * 32, y * 32 };
                if (!MapIsLocationValid(corner))
                    continue;
                auto* first = MapGetSurfaceElementAt(corner);
                if (first == nullptr)
                    continue;
                const auto z = first->getBaseZ();
                bool ok = true;
                for (int32_t dy = -1; dy <= h && ok; dy++)
                    for (int32_t dx = -1; dx <= w && ok; dx++)
                        ok = isBareFlat({ (x + dx) * 32, (y + dy) * 32 }, z);
                if (ok)
                    return corner;
            }
        }
        return std::nullopt;
    }

    void insertPath(const CoordsXY& tile)
    {
        const auto baseZ = MapGetSurfaceElementAt(tile)->getBaseZ();
        auto* path = TileElementInsert<PathElement>({ tile.x, tile.y, baseZ }, 0b1111);
        path->setClearanceZ(baseZ + 16);
    }

    BuildingObject* loadClub(IContext& context, ObjectEntryIndex& index)
    {
        auto& objectManager = context.GetObjectManager();
        auto* object = objectManager.LoadObject("golf.building.club_house");
        if (object == nullptr)
            return nullptr;
        index = objectManager.GetLoadedObjectEntryIndex(object);
        return BuildingObject::GetById(index);
    }
} // namespace


TEST(GolfRuntimeTests, FindsTheClubAndTicksOnlyWhenACourseExists)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    gameState.cheats.sandboxMode = true;
    ObjectEntryIndex index{};
    auto* club = loadClub(*context, index);
    if (club == nullptr)
        GTEST_SKIP() << "the club house object is not installed";

    // No course and no runtime: updating the golf does nothing.
    Golf::updateGolf(gameState);
    EXPECT_EQ(gameState.golfRuntime, nullptr);

    const auto found = findFlatBlock(3, 2);
    ASSERT_TRUE(found.has_value());
    insertPath(Building::GetEntrance(*club, *found, 0).outsideTile);
    GameActions::BuildingPlaceAction place(*found, index, 0);
    ASSERT_EQ(GameActions::ExecuteNested(&place, gameState).error, GameActions::Status::ok);

    const auto scan = Golf::scanBuildings();
    ASSERT_TRUE(scan.hasClub);
    // The door is on the entrance edge of the club, outside its footprint.
    const auto entrance = Building::GetEntrance(*club, *found, 0);
    EXPECT_EQ((scan.club.x.raw() >> 16) / 32, entrance.outsideTile.x / 32);
    EXPECT_EQ((scan.club.y.raw() >> 16) / 32, entrance.outsideTile.y / 32);

    gameState.golf.createCourse("Links");
    Golf::updateGolf(gameState);
    ASSERT_NE(gameState.golfRuntime, nullptr);
    EXPECT_TRUE(gameState.golfRuntime->active());

    // A new map forgets the runtime.
    Golf::resetRuntime(gameState);
    EXPECT_EQ(gameState.golfRuntime, nullptr);
}

TEST(GolfRuntimeTests, EveryGolferHasAnEntityThatFollowsItAndIsRemovedWhenItLeaves)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    gameState.cheats.sandboxMode = true;
    ObjectEntryIndex index{};
    auto* club = loadClub(*context, index);
    if (club == nullptr)
        GTEST_SKIP() << "the club house object is not installed";

    const auto found = findFlatBlock(3, 2);
    ASSERT_TRUE(found.has_value());
    insertPath(Building::GetEntrance(*club, *found, 0).outsideTile);
    GameActions::BuildingPlaceAction place(*found, index, 0);
    ASSERT_EQ(GameActions::ExecuteNested(&place, gameState).error, GameActions::Status::ok);
    gameState.golf.createCourse("Links");

    // A stale entity from before belongs to no golfer and is removed when the runtime starts.
    auto* stale = GolferEntity::create(999, { found->x, found->y, 0 });
    ASSERT_NE(stale, nullptr);

    auto& runtime = Golf::runtimeOf(gameState);
    EXPECT_EQ(gameState.entities.getEntityListCount(EntityType::golfer), 0u);

    Golf::GolfSimulation::Spawn spawn;
    spawn.name = "Test Golfer";
    spawn.position = runtime.buildings().club;
    const uint32_t id = runtime.director().simulation().spawn(spawn).id;

    runtime.update(gameState.currentTicks);
    auto* entity = runtime.entityOf(id);
    ASSERT_NE(entity, nullptr);
    EXPECT_EQ(entity->golfer, id);
    EXPECT_EQ(entity->x, runtime.buildings().club.x.floor());
    EXPECT_EQ(gameState.entities.getEntityListCount(EntityType::golfer), 1u);

    // The entity moves with the golfer.
    auto* golfer = runtime.director().simulation().find(id);
    ASSERT_NE(golfer, nullptr);
    golfer->position.x = golfer->position.x + Fixed::fromInt(64);
    runtime.update(gameState.currentTicks);
    EXPECT_EQ(runtime.entityOf(id)->x, golfer->position.x.floor());

    // When the golfer is gone its entity goes too.
    runtime.director().simulation().golfers().clear();
    runtime.update(gameState.currentTicks);
    EXPECT_EQ(runtime.entityOf(id), nullptr);
    EXPECT_EQ(gameState.entities.getEntityListCount(EntityType::golfer), 0u);
}

TEST(GolfRuntimeTests, EveryStateMapsToARequiredAnimationClip)
{
    for (int i = 0; i < static_cast<int>(Golf::GolferState::count); i++)
    {
        const std::string name = golferClipName(static_cast<Golf::GolferState>(i));
        bool found = false;
        for (auto required : kRequiredGolferClips)
            found = found || required == name;
        EXPECT_TRUE(found) << "state " << i << " plays '" << name << "', which golfer models are not required to have";
    }
}

TEST(GolfRuntimeTests, TheStandardGolferModelLoadsWithAllClipsAndRemapMaterials)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& objectManager = context->GetObjectManager();
    auto* object = objectManager.LoadObject("golf.golfer_model.standard");
    if (object == nullptr)
        GTEST_SKIP() << "the golfer model object is not installed";
    auto* golferModel = static_cast<GolferModelObject*>(object);
    ASSERT_NE(golferModel->ModelId, 0u);
    EXPECT_EQ(GolferModelObject::ActiveModelId, golferModel->ModelId);

    const auto* model = Drawing::ModelSpriteStore::Instance().GetModel(golferModel->ModelId);
    ASSERT_NE(model, nullptr);
    EXPECT_LE(model->triangleCount(), static_cast<size_t>(golferModel->TriangleBudget));
    for (auto required : kRequiredGolferClips)
        EXPECT_NE(model->findClip(std::string(required)), nullptr) << "missing clip " << required;
    for (const char* name : Model::kRemapMaterialNames)
    {
        bool found = false;
        for (const auto& material : model->materials)
            found = found || material.name == name;
        EXPECT_TRUE(found) << "missing material " << name;
    }
    // Every state of a golfer plays a clip the model has.
    for (int i = 0; i < static_cast<int>(Golf::GolferState::count); i++)
        EXPECT_NE(model->findClip(golferClipName(static_cast<Golf::GolferState>(i))), nullptr);
}

// A golfer who walks along +x must show the stride along +x: the yaw the entity takes from its travel direction has to put
// the model's forward axis on that direction, or the golfer walks sideways.
TEST(GolfRuntimeTests, AWalkingGolferModelStridesAlongTheDirectionItIsTurnedTo)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto* object = context->GetObjectManager().LoadObject("golf.golfer_model.standard");
    if (object == nullptr)
        GTEST_SKIP() << "the golfer model object is not installed";
    const auto modelId = static_cast<GolferModelObject*>(object)->ModelId;
    const auto* model = Drawing::ModelSpriteStore::Instance().GetModel(modelId);
    ASSERT_NE(model, nullptr);
    const auto* walk = model->findClip("walk");
    ASSERT_NE(walk, nullptr);

    // Yaw is in turns of 256 and 0 faces -x, so +x is 128, +y is 192, -x is 0 and -y is 64.
    struct Case
    {
        int yaw;
        bool alongX;
    };
    for (const auto c : { Case{ 0, true }, Case{ 128, true }, Case{ 64, false }, Case{ 192, false } })
    {
        Model::RenderSettings settings;
        settings.clipIndex = static_cast<int32_t>(walk - model->clips.data());
        settings.time = 0; // the legs are at the widest stride
        settings.orientation.yaw = static_cast<float>(c.yaw) * (2.0f * 3.14159265f / 256.0f);
        const auto bounds = Model::computeBounds(*model, settings);
        ASSERT_TRUE(bounds.valid);
        const float extentX = bounds.max[0] - bounds.min[0];
        const float extentY = bounds.max[1] - bounds.min[1];
        if (c.alongX)
            EXPECT_GT(extentX, extentY) << "yaw " << c.yaw;
        else
            EXPECT_GT(extentY, extentX) << "yaw " << c.yaw;
    }
}

TEST(GolfRuntimeTests, BenchesAndBinsOnThePathAreFoundAndKeepTheirState)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    auto& objectManager = context->GetObjectManager();
    auto* benchObject = objectManager.LoadObject("golf.path_addition.bench");
    auto* binObject = objectManager.LoadObject("golf.path_addition.bin");
    if (benchObject == nullptr || binObject == nullptr)
        GTEST_SKIP() << "the path additions are not installed";

    const auto found = findFlatBlock(3, 2);
    ASSERT_TRUE(found.has_value());
    auto place = [&](const CoordsXY& tile, Object* object) {
        const auto baseZ = MapGetSurfaceElementAt(tile)->getBaseZ();
        auto* path = TileElementInsert<PathElement>({ tile.x, tile.y, baseZ }, 0b1111);
        path->setClearanceZ(baseZ + 16);
        path->setAdditionEntryIndex(objectManager.GetLoadedObjectEntryIndex(object));
    };
    place({ found->x, found->y }, benchObject);
    place({ found->x + 32, found->y }, binObject);

    // The park may have benches of its own: look for the two that were just placed.
    auto bench = [&](const Golf::BuildingScan& scan) -> const Golf::Bench* {
        for (const auto& b : scan.benches)
            if (b.position.x.floor() == found->x + 16 && b.position.y.floor() == found->y + 16)
                return &b;
        return nullptr;
    };
    auto bin = [&](const Golf::BuildingScan& scan) -> const Golf::LitterBin* {
        for (const auto& b : scan.bins)
            if (b.position.x.floor() == found->x + 32 + 16 && b.position.y.floor() == found->y + 16)
                return &b;
        return nullptr;
    };
    const auto scan = Golf::scanBuildings();
    ASSERT_NE(bench(scan), nullptr);
    ASSERT_NE(bin(scan), nullptr);
    EXPECT_NE(bench(scan)->id, bin(scan)->id);

    // A rescan keeps who sits on the bench and what is in the bin.
    gameState.golf.createCourse("Links");
    auto& runtime = Golf::runtimeOf(gameState);
    auto& env = runtime.director().simulation().environment();
    for (auto& b : env.benches)
        if (b.id == bench(scan)->id)
            b.occupants = 2;
    for (auto& b : env.bins)
        if (b.id == bin(scan)->id)
            b.trashCount = 7;
    runtime.refreshBuildings();
    int occupants = -1;
    uint32_t trash = 0;
    for (const auto& b : env.benches)
        occupants = b.id == bench(scan)->id ? b.occupants : occupants;
    for (const auto& b : env.bins)
        trash = b.id == bin(scan)->id ? b.trashCount : trash;
    EXPECT_EQ(occupants, 2);
    EXPECT_EQ(trash, 7u);
}

TEST(GolfRuntimeTests, GreenFeesReachTheCashOfThePark)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    gameState.cheats.sandboxMode = true;
    ObjectEntryIndex index{};
    auto* club = loadClub(*context, index);
    if (club == nullptr)
        GTEST_SKIP() << "the club house object is not installed";

    const auto found = findFlatBlock(3, 2);
    ASSERT_TRUE(found.has_value());
    insertPath(Building::GetEntrance(*club, *found, 0).outsideTile);
    GameActions::BuildingPlaceAction place(*found, index, 0);
    ASSERT_EQ(GameActions::ExecuteNested(&place, gameState).error, GameActions::Status::ok);

    auto* course = gameState.golf.createCourse("Links");
    course->open = true;
    course->greenFee = 1000;
    const auto courseId = course->id;
    auto* hole = gameState.golf.createHole(courseId);
    hole->tees[0] = Golf::HolePoint{ found->x + 64, found->y + 200 };
    hole->cup = Golf::HolePoint{ found->x + 64, found->y + 800 };

    auto& runtime = Golf::runtimeOf(gameState);
    Golf::GolfSimulation::Spawn spawn;
    spawn.name = "Payer";
    spawn.courseId = courseId;
    spawn.money = 100000;
    spawn.position = runtime.buildings().club;
    runtime.director().simulation().spawn(spawn);

    const auto cashBefore = gameState.park.cash;
    // Five game minutes of 06:00 are enough for the group to check in and pay.
    uint32_t tick = 6 * 60 * Golf::kTicksPerGameMinute;
    for (uint32_t i = 0; i < 20 * Golf::kTicksPerGameMinute; i++)
        runtime.update(tick + i);
    EXPECT_GT(gameState.park.cash, cashBefore);
    EXPECT_EQ(gameState.park.cash - cashBefore, runtime.director().total().greenFees + runtime.director().today().greenFees);
}

TEST(GolfRuntimeTests, BuildingsCostTheirUpkeepEachGameDay)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    gameState.cheats.sandboxMode = true;
    ObjectEntryIndex index{};
    auto* club = loadClub(*context, index);
    if (club == nullptr)
        GTEST_SKIP() << "the club house object is not installed";
    ASSERT_GT(club->UpkeepPerMonth, 0);

    const auto found = findFlatBlock(3, 2);
    ASSERT_TRUE(found.has_value());
    insertPath(Building::GetEntrance(*club, *found, 0).outsideTile);
    GameActions::BuildingPlaceAction place(*found, index, 0);
    ASSERT_EQ(GameActions::ExecuteNested(&place, gameState).error, GameActions::Status::ok);
    gameState.golf.createCourse("Links");

    auto& runtime = Golf::runtimeOf(gameState);
    EXPECT_EQ(runtime.buildings().upkeepPerMonth, club->UpkeepPerMonth);

    // The first update only notes the day; each new day costs a thirtieth of the month.
    runtime.update(Golf::kTicksPerGameDay);
    const auto cash = gameState.park.cash;
    runtime.update(2 * Golf::kTicksPerGameDay);
    EXPECT_EQ(cash - gameState.park.cash, club->UpkeepPerMonth / 30);
    runtime.update(2 * Golf::kTicksPerGameDay + Golf::kTicksPerGameMinute);
    EXPECT_EQ(cash - gameState.park.cash, club->UpkeepPerMonth / 30); // the same day costs nothing more
}

TEST(GolfRuntimeTests, ARageQuitIsReportedInTheNews)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    gameState.cheats.sandboxMode = true;
    ObjectEntryIndex index{};
    auto* club = loadClub(*context, index);
    if (club == nullptr)
        GTEST_SKIP() << "the club house object is not installed";
    const auto found = findFlatBlock(3, 2);
    ASSERT_TRUE(found.has_value());
    insertPath(Building::GetEntrance(*club, *found, 0).outsideTile);
    GameActions::BuildingPlaceAction place(*found, index, 0);
    ASSERT_EQ(GameActions::ExecuteNested(&place, gameState).error, GameActions::Status::ok);
    gameState.golf.createCourse("Links");
    auto& runtime = Golf::runtimeOf(gameState);

    Golf::GolfSimulation::Spawn spawn;
    spawn.name = "Grumpy Gary";
    spawn.position = { Fixed::fromInt(96), Fixed::fromInt(160), Fixed::fromInt(16) };
    const uint32_t id = runtime.director().simulation().spawn(spawn).id;
    runtime.update(0);
    runtime.update(1);

    Golf::SimEvent shot;
    shot.type = Golf::SimEvent::Type::shotHit;
    shot.golferId = id;
    runtime.announce(shot);
    EXPECT_TRUE(gameState.park.newsItems.isEmpty()); // only rage quits are news

    Golf::SimEvent event;
    event.type = Golf::SimEvent::Type::rageQuit;
    event.golferId = id;
    event.value = 7;
    event.insult = 2;
    runtime.announce(event);
    ASSERT_FALSE(gameState.park.newsItems.isEmpty());
    const auto& item = gameState.park.newsItems.current();
    EXPECT_EQ(item.type, News::ItemType::blank);
    // The news points at where the golfer was (it walked a few units in the two ticks).
    EXPECT_NEAR(static_cast<int>(item.assoc & 0xFFFF), 96, 8);
    EXPECT_NEAR(static_cast<int>(item.assoc >> 16), 160, 8);
    EXPECT_NE(item.text.find("Grumpy Gary"), std::string::npos);
    EXPECT_NE(item.text.find("hole 7"), std::string::npos);
}

TEST(GolfRuntimeTests, TheCarAndCartModelsLoadWithTheRemapMaterials)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& objectManager = context->GetObjectManager();
    auto* car = static_cast<VehicleModelObject*>(objectManager.LoadObject("golf.vehicle_model.car"));
    auto* cart = static_cast<VehicleModelObject*>(objectManager.LoadObject("golf.vehicle_model.golf_cart"));
    if (car == nullptr || cart == nullptr)
        GTEST_SKIP() << "the vehicle model objects are not installed";
    ASSERT_NE(car->ModelId, 0u);
    ASSERT_NE(cart->ModelId, 0u);
    EXPECT_NE(car->ModelId, cart->ModelId);
    EXPECT_EQ(VehicleModelObject::ActiveCarModelId, car->ModelId);
    EXPECT_EQ(VehicleModelObject::ActiveCartModelId, cart->ModelId);

    for (const auto* vehicle : { car, cart })
    {
        const auto* model = Drawing::ModelSpriteStore::Instance().GetModel(vehicle->ModelId);
        ASSERT_NE(model, nullptr);
        EXPECT_GT(model->triangleCount(), 0u);
        for (const char* name : { "remap_primary", "remap_secondary" })
        {
            bool found = false;
            for (const auto& material : model->materials)
                found = found || material.name == name;
            EXPECT_TRUE(found) << name;
        }
        // The model is as long as its contact points say (the car 2.4 m, the cart 2 m, in tenths of a unit).
        Model::RenderSettings settings;
        settings.unitsPerMetre = Drawing::ModelSpriteStore::Instance().GetUnitsPerMetre(vehicle->ModelId);
        const auto bounds = Model::computeBounds(*model, settings);
        ASSERT_TRUE(bounds.valid);
        const float length = std::max(bounds.max[0] - bounds.min[0], bounds.max[1] - bounds.min[1]);
        EXPECT_NEAR(length, vehicle->ContactPoints.halfLength * 2.0f, 5.0f);
    }
}

TEST(GolfRuntimeTests, GolfVehicleEntitiesTurnAndMove)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    auto* entity = GolfVehicleEntity::create(GolfVehicleEntity::Kind::cart, 12, { 100, 100, 8 });
    ASSERT_NE(entity, nullptr);
    EXPECT_EQ(entity->vehicle, 12u);
    EXPECT_EQ(gameState.entities.getEntityListCount(EntityType::golfVehicle), 1u);

    // A heading is a map direction: 0 = -x, 1 = +y, 2 = +x, 3 = -y; the yaw turns counter-clockwise from -x.
    entity->follow({ 100, 100, 8 }, 0);
    EXPECT_EQ(entity->yaw, 0);
    entity->follow({ 100, 100, 8 }, 1);
    EXPECT_EQ(entity->yaw, 192);
    entity->follow({ 100, 100, 8 }, 2);
    EXPECT_EQ(entity->yaw, 128);
    entity->follow({ 100, 100, 8 }, 3);
    EXPECT_EQ(entity->yaw, 64);
    // Without a heading it turns to the way it moves, and that agrees with the headings.
    entity->follow({ 140, 100, 8 }, -1);
    EXPECT_EQ(entity->yaw, 128); // towards +x
    EXPECT_EQ(entity->x, 140);
    entity->follow({ 140, 140, 8 }, -1);
    EXPECT_EQ(entity->yaw, 192); // towards +y
    entity->follow({ 100, 140, 8 }, -1);
    EXPECT_EQ(entity->yaw, 0); // towards -x
    entity->follow({ 100, 100, 8 }, -1);
    EXPECT_EQ(entity->yaw, 64); // towards -y
    entity->remove();
    EXPECT_EQ(gameState.entities.getEntityListCount(EntityType::golfVehicle), 0u);
}

namespace
{
    // A club, a street from the edge of the map with a road entrance, and a parking lot of two tiles with a path to the
    // club; returns false when the test park has ground in the way.
    bool buildLotAndStreet(IContext& context, GameState_t& gameState)
    {
        gameState.cheats.sandboxMode = true;
        // The test park is a small map inside bigger storage; the traffic scans the map by its size.
        gameState.mapSize = { 60, 60 };
        ObjectEntryIndex clubIndex{};
        auto* club = loadClub(context, clubIndex);
        auto& objectManager = context.GetObjectManager();
        auto* road = objectManager.LoadObject("golf.street.road");
        if (club == nullptr || road == nullptr)
            return false;
        const auto roadIndex = objectManager.GetLoadedObjectEntryIndex(road);

        // A row of bare ground from the edge ring (tile 1) to tile 8, with bare ground below tiles 4 to 6 for the lot.
        auto bare = [](int x, int y) {
            const CoordsXY tile{ x * 32, y * 32 };
            auto* surface = MapGetSurfaceElementAt(tile);
            return surface != nullptr && isBareFlat(tile, surface->getBaseZ());
        };
        int row = -1;
        for (int y = 3; y < 20 && row < 0; y++)
        {
            bool free = true;
            for (int x = 1; x <= 8; x++)
                free = free && bare(x, y);
            for (int x = 4; x <= 6; x++)
                free = free && bare(x, y + 1);
            if (free)
                row = y;
        }
        if (row < 0)
            return false;

        auto ok = [&](const GameActions::GameAction& action) {
            return GameActions::ExecuteNested(&action, gameState).error == GameActions::Status::ok;
        };
        for (int x = 1; x <= 8; x++)
        {
            GameActions::StreetPlaceAction street({ x * 32, row * 32 }, roadIndex);
            if (!ok(street))
                return false;
        }
        GameActions::StreetRoadEntranceAction entrance({ 1 * 32, row * 32 }, true);
        if (!ok(entrance))
            return false;
        // Two parking tiles below the street facing it (-y), and a path next to them.
        for (int x : { 5, 6 })
        {
            GameActions::ParkingPlaceAction parking({ x * 32, (row + 1) * 32 }, roadIndex, 3);
            if (!ok(parking))
                return false;
        }
        insertPath({ 4 * 32, (row + 1) * 32 });

        // The club goes where there is room left.
        const auto found = findFlatBlock(3, 2);
        if (!found.has_value())
            return false;
        insertPath(Building::GetEntrance(*club, *found, 0).outsideTile);
        GameActions::BuildingPlaceAction place(*found, clubIndex, 0);
        return ok(place);
    }
} // namespace

TEST(GolfRuntimeTests, GroupsArriveByCarAndParkInTheLot)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    if (!buildLotAndStreet(*context, gameState))
        GTEST_SKIP() << "the test park has no room for a street and a lot, or the objects are missing";

    auto* course = gameState.golf.createCourse("Links");
    course->open = true;
    course->greenFee = 500;
    auto* hole = gameState.golf.createHole(course->id);
    hole->tees[0] = Golf::HolePoint{ 10 * 32, 8 * 32 };
    hole->cup = Golf::HolePoint{ 10 * 32 + 200, 8 * 32 };

    auto& runtime = Golf::runtimeOf(gameState);
    runtime.director().setPopularity(1000);
    runtime.update(0); // learns the club, the streets and the lot
    ASSERT_TRUE(runtime.hasCarTraffic());
    EXPECT_EQ(runtime.traffic().capacity(), 4); // two tiles, two spaces each

    // Run a morning: cars come in at the entrance, park, and the golfers get out of them.
    const uint32_t start = Golf::kTicksPerGameDay + 6 * 60 * Golf::kTicksPerGameMinute;
    bool sawCar = false, sawParked = false, sawGolferOut = false;
    for (uint32_t t = start; t < start + 4 * 60 * Golf::kTicksPerGameMinute; t++)
    {
        runtime.update(t);
        for (const auto& car : runtime.traffic().cars())
        {
            sawCar = true;
            sawParked = sawParked || car.state == Golf::CarState::parked;
        }
        // STE-011: never more cars on the way to or in a space than there are spaces.
        ASSERT_GE(runtime.traffic().freeSpaces(), 0);
        ASSERT_LE(runtime.traffic().freeSpaces(), 4);
        for (const auto& golfer : runtime.director().simulation().golfers())
            sawGolferOut = sawGolferOut || (golfer.state != Golf::GolferState::inCar && golfer.hasExit);
    }
    EXPECT_TRUE(sawCar);
    EXPECT_TRUE(sawParked);
    EXPECT_TRUE(sawGolferOut);
    // The cars have entities that show them.
    EXPECT_EQ(gameState.entities.getEntityListCount(EntityType::golfVehicle), runtime.traffic().cars().size());
}

TEST(GolfRuntimeTests, WithoutStreetsGroupsStillTurnUpAtTheClub)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    gameState.cheats.sandboxMode = true;
    ObjectEntryIndex clubIndex{};
    auto* club = loadClub(*context, clubIndex);
    if (club == nullptr)
        GTEST_SKIP() << "the club house object is not installed";
    const auto found = findFlatBlock(3, 2);
    ASSERT_TRUE(found.has_value());
    insertPath(Building::GetEntrance(*club, *found, 0).outsideTile);
    GameActions::BuildingPlaceAction place(*found, clubIndex, 0);
    ASSERT_EQ(GameActions::ExecuteNested(&place, gameState).error, GameActions::Status::ok);

    auto* course = gameState.golf.createCourse("Links");
    course->open = true;
    auto* hole = gameState.golf.createHole(course->id);
    hole->tees[0] = Golf::HolePoint{ found->x + 64, found->y + 200 };
    hole->cup = Golf::HolePoint{ found->x + 64, found->y + 800 };

    auto& runtime = Golf::runtimeOf(gameState);
    runtime.director().setPopularity(1000);
    EXPECT_FALSE(runtime.hasCarTraffic());
    const uint32_t start = Golf::kTicksPerGameDay + 6 * 60 * Golf::kTicksPerGameMinute;
    bool sawGolfer = false;
    for (uint32_t t = start; t < start + 60 * Golf::kTicksPerGameMinute; t++)
    {
        runtime.update(t);
        for (const auto& golfer : runtime.director().simulation().golfers())
        {
            EXPECT_NE(golfer.state, Golf::GolferState::inCar);
            sawGolfer = true;
        }
    }
    EXPECT_TRUE(sawGolfer);
    EXPECT_TRUE(runtime.traffic().cars().empty());
}

TEST(GolfRuntimeTests, EveryCarDrivesAwayOnceItsGroupHasLeft)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    if (!buildLotAndStreet(*context, gameState))
        GTEST_SKIP() << "the test park has no room for a street and a lot, or the objects are missing";

    auto* course = gameState.golf.createCourse("Links");
    course->open = true;
    course->greenFee = 500;
    auto* hole = gameState.golf.createHole(course->id);
    hole->tees[0] = Golf::HolePoint{ 10 * 32, 8 * 32 };
    hole->cup = Golf::HolePoint{ 10 * 32 + 120, 8 * 32 };

    auto& runtime = Golf::runtimeOf(gameState);
    runtime.director().setPopularity(1000);
    size_t mostCars = 0;
    // From 06:00 to the small hours of the next morning: arrivals stop before closing, everyone is off the course by 20:00.
    const uint32_t start = Golf::kTicksPerGameDay + 6 * 60 * Golf::kTicksPerGameMinute;
    for (uint32_t t = start; t < start + 16 * 60 * Golf::kTicksPerGameMinute; t++)
    {
        runtime.update(t);
        mostCars = std::max(mostCars, runtime.traffic().cars().size());
        ASSERT_LE(runtime.traffic().cars().size(), 4u); // a car holds a space from the moment it is on its way
    }
    EXPECT_GT(mostCars, 0u);
    EXPECT_TRUE(runtime.director().simulation().golfers().empty());
    // Every car has left by the road entrance, and so has its entity and its place in the way of the balls.
    for (const auto& car : runtime.traffic().cars())
        EXPECT_EQ(car.state, Golf::CarState::gone);
    EXPECT_EQ(gameState.entities.getEntityListCount(EntityType::golfVehicle), 0u);
    EXPECT_EQ(runtime.traffic().freeSpaces(), runtime.traffic().capacity());
}

TEST(GolfRuntimeTests, CartsAreBoughtIntoTheBaysOfTheMapAndDrawn)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    gameState.cheats.sandboxMode = true;
    gameState.mapSize = { 60, 60 };
    auto& objectManager = context->GetObjectManager();
    auto* road = objectManager.LoadObject("golf.street.road");
    if (road == nullptr)
        GTEST_SKIP() << "the street object is not installed";
    const auto roadIndex = objectManager.GetLoadedObjectEntryIndex(road);
    const auto found = findFlatBlock(3, 2);
    ASSERT_TRUE(found.has_value());
    const CoordsXY bay1{ found->x, found->y }, bay2{ found->x + 32, found->y };
    for (const auto& bay : { bay1, bay2 })
    {
        GameActions::ParkingPlaceAction place(bay, roadIndex, 3, true);
        ASSERT_EQ(GameActions::ExecuteNested(&place, gameState).error, GameActions::Status::ok);
    }

    auto& runtime = Golf::runtimeOf(gameState);
    runtime.update(0); // learns the bays
    auto& fleet = runtime.director().fleet;
    ASSERT_EQ(fleet.spaceCount(), 2);
    EXPECT_EQ(fleet.cartCount(), 0);
    EXPECT_EQ(fleet.spaces()[0].position.x.floor(), bay1.x + 16);

    // Buying five carts buys two: there are two bays. The price is paid.
    const auto cash = gameState.park.cash;
    GameActions::CartEditAction buy(GameActions::CartEditType::buy, 5);
    const auto bought = GameActions::ExecuteNested(&buy, gameState);
    ASSERT_EQ(bought.error, GameActions::Status::ok);
    EXPECT_EQ(fleet.cartCount(), 2);
    EXPECT_EQ(bought.cost, 2 * fleet.cartPrice);
    // Another one has nowhere to stand.
    GameActions::CartEditAction more(GameActions::CartEditType::buy, 1);
    EXPECT_EQ(GameActions::Query(&more, gameState).error, GameActions::Status::disallowed);
    GameActions::CartEditAction none(GameActions::CartEditType::buy, 0);
    EXPECT_EQ(GameActions::Query(&none, gameState).error, GameActions::Status::invalidParameters);
    (void)cash;

    // CRT-002: parked carts are drawn, one entity each, and the bays do not go away.
    runtime.update(1);
    EXPECT_EQ(gameState.entities.getEntityListCount(EntityType::golfVehicle), 2u);

    GameActions::CartEditAction fee(GameActions::CartEditType::setFee, 750);
    ASSERT_EQ(GameActions::ExecuteNested(&fee, gameState).error, GameActions::Status::ok);
    EXPECT_EQ(fleet.cartFee, 750);
    GameActions::CartEditAction badFee(GameActions::CartEditType::setFee, -1);
    EXPECT_EQ(GameActions::Query(&badFee, gameState).error, GameActions::Status::invalidParameters);

    GameActions::CartEditAction sell(GameActions::CartEditType::sell, 1);
    ASSERT_EQ(GameActions::ExecuteNested(&sell, gameState).error, GameActions::Status::ok);
    EXPECT_EQ(fleet.cartCount(), 1);
    runtime.update(2);
    EXPECT_EQ(gameState.entities.getEntityListCount(EntityType::golfVehicle), 1u);

    // Taking a bay away takes the cart that stood in it.
    GameActions::ParkingRemoveAction removeBay1(bay1);
    GameActions::ParkingRemoveAction removeBay2(bay2);
    ASSERT_EQ(GameActions::ExecuteNested(&removeBay1, gameState).error, GameActions::Status::ok);
    ASSERT_EQ(GameActions::ExecuteNested(&removeBay2, gameState).error, GameActions::Status::ok);
    runtime.update(Golf::kTicksPerGameMinute); // the bays are looked at once a game minute
    EXPECT_EQ(fleet.spaceCount(), 0);
    EXPECT_EQ(fleet.cartCount(), 0);
    runtime.update(Golf::kTicksPerGameMinute + 1);
    EXPECT_EQ(gameState.entities.getEntityListCount(EntityType::golfVehicle), 0u);
}

TEST(GolfRuntimeTests, ARangeFindsItsKioskAndDistanceSignsAndAimsAtTheMiddle)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    gameState.cheats.sandboxMode = true;
    gameState.mapSize = { 60, 60 };
    auto& objectManager = context->GetObjectManager();
    auto* kioskObject = objectManager.LoadObject("golf.building.driving_range_kiosk");
    auto* signObject = objectManager.LoadObject("golf.scenery_small.distance_sign");
    if (kioskObject == nullptr || signObject == nullptr)
        GTEST_SKIP() << "the range objects are not installed";
    const auto kioskIndex = objectManager.GetLoadedObjectEntryIndex(kioskObject);
    const auto signIndex = objectManager.GetLoadedObjectEntryIndex(signObject);

    // A strip of bare ground: a 1 x 1 kiosk at the west end and signs along it.
    const auto found = findFlatBlock(3, 2);
    ASSERT_TRUE(found.has_value());
    const auto* kiosk = BuildingObject::GetById(kioskIndex);
    ASSERT_NE(kiosk, nullptr);
    insertPath(Building::GetEntrance(*kiosk, *found, 0).outsideTile);
    GameActions::BuildingPlaceAction placeKiosk(*found, kioskIndex, 0);
    if (GameActions::ExecuteNested(&placeKiosk, gameState).error != GameActions::Status::ok)
        GTEST_SKIP() << "no room for the kiosk";
    const CoordsXY signTile{ found->x + 64, found->y + 32 };
    const CoordsXYZD signAt{ signTile.x + 16, signTile.y + 16, MapGetSurfaceElementAt(signTile)->getBaseZ(), 0 };
    GameActions::SmallSceneryPlaceAction placeSign(
        signAt, 0, signIndex, Drawing::Colour::black, Drawing::Colour::black, Drawing::Colour::black);
    if (GameActions::ExecuteNested(&placeSign, gameState).error != GameActions::Status::ok)
        GTEST_SKIP() << "no room for the sign";

    // A range around both, and one that has nothing in it.
    GameActions::RangeEditAction around(
        GameActions::RangeEditType::create, 0, { found->x - 32, found->y - 32 }, { found->x + 128, found->y + 96 });
    const auto id = GameActions::ExecuteNested(&around, gameState).getData<GameActions::RangeEditResult>().rangeId;
    GameActions::RangeEditAction empty(GameActions::RangeEditType::create, 0, { 0, 0 }, { 0, 0 });
    const auto emptyId = GameActions::ExecuteNested(&empty, gameState).getData<GameActions::RangeEditResult>().rangeId;
    GameActions::RangeEditAction tile(GameActions::RangeEditType::addHittingTile, id, { found->x + 96, found->y });
    ASSERT_EQ(GameActions::ExecuteNested(&tile, gameState).error, GameActions::Status::ok);

    auto& runtime = Golf::runtimeOf(gameState);
    runtime.update(0);
    const auto* range = gameState.golf.findRange(id);
    ASSERT_TRUE(range->kiosk.has_value());
    EXPECT_EQ(range->kiosk->x, found->x + 16);
    ASSERT_EQ(range->signs.size(), 1u);
    EXPECT_EQ(range->signs[0].position.x, signTile.x + 16);
    // 4 tiles from the kiosk is 20 yards.
    EXPECT_EQ(range->signs[0].distance, Golf::signDistance(*range->kiosk, range->signs[0].position));
    EXPECT_GT(range->signs[0].distance, 0);
    // The hitting tile is east of the middle of the area, so the golfers shoot back west (about 180 degrees).
    EXPECT_NEAR(range->shootingDirectionDegrees, 180, 90);
    // The other range found nothing.
    const auto* other = gameState.golf.findRange(emptyId);
    EXPECT_FALSE(other->kiosk.has_value());
    EXPECT_TRUE(other->signs.empty());

    // Once the player sets a direction it stays.
    GameActions::RangeEditAction aim(GameActions::RangeEditType::setDirection, id, {}, {}, 45);
    ASSERT_EQ(GameActions::ExecuteNested(&aim, gameState).error, GameActions::Status::ok);
    runtime.update(Golf::kTicksPerGameMinute);
    EXPECT_EQ(gameState.golf.findRange(id)->shootingDirectionDegrees, 45);
    GameActions::RangeEditAction automatic(GameActions::RangeEditType::autoDirection, id);
    ASSERT_EQ(GameActions::ExecuteNested(&automatic, gameState).error, GameActions::Status::ok);
    runtime.update(2 * Golf::kTicksPerGameMinute);
    EXPECT_NE(gameState.golf.findRange(id)->shootingDirectionDegrees, 45);
}

TEST(GolfRuntimeTests, MirroredBallsFollowTheSimulationAndDoNotMoveOnTheirOwn)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    Golf::Ball ball;
    ball.position = { Fixed::fromInt(200), Fixed::fromInt(300), Fixed::fromInt(40) };
    ball.state = Golf::BallState::InFlight;
    ball.velocity = { Fixed::fromInt(5), Fixed{}, Fixed{} };
    auto* entity = GolfBall::create(ball.position, ball.state, 7);
    ASSERT_NE(entity, nullptr);
    entity->mirror(ball);
    EXPECT_TRUE(entity->mirrored);
    EXPECT_EQ(entity->x, 200);
    EXPECT_EQ(entity->y, 300);

    // The entity does not step the ball by itself: only the simulation moves it.
    entity->update();
    entity->update();
    EXPECT_EQ(entity->x, 200);
    EXPECT_EQ(entity->ball.velocity.x, Fixed::fromInt(5));

    // Following the simulation leaves a trail while the ball flies, and none once it rests.
    ball.position.x = Fixed::fromInt(240);
    entity->mirror(ball);
    EXPECT_EQ(entity->x, 240);
    EXPECT_GE(entity->trailCount, 1);
    ball.state = Golf::BallState::AtRest;
    entity->mirror(ball);
    EXPECT_EQ(entity->trailCount, 0);
    (void)gameState;
}

TEST(GolfRuntimeTests, TheBallOfAGolferInPlayIsDrawnAndGoneWhenHoled)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    gameState.cheats.sandboxMode = true;
    ObjectEntryIndex index{};
    auto* club = loadClub(*context, index);
    if (club == nullptr)
        GTEST_SKIP() << "the club house object is not installed";
    const auto found = findFlatBlock(3, 2);
    ASSERT_TRUE(found.has_value());
    insertPath(Building::GetEntrance(*club, *found, 0).outsideTile);
    GameActions::BuildingPlaceAction place(*found, index, 0);
    ASSERT_EQ(GameActions::ExecuteNested(&place, gameState).error, GameActions::Status::ok);

    auto* course = gameState.golf.createCourse("Links");
    course->open = true;
    auto* hole = gameState.golf.createHole(course->id);
    hole->tees[0] = Golf::HolePoint{ found->x + 64, found->y + 200 };
    hole->cup = Golf::HolePoint{ found->x + 64, found->y + 700 };
    auto& runtime = Golf::runtimeOf(gameState);
    runtime.director().setPopularity(0);

    Golf::GolfSimulation::Spawn spawn;
    spawn.name = "Player";
    spawn.courseId = course->id;
    spawn.money = 100000;
    spawn.position = runtime.buildings().club;
    const uint32_t id = runtime.director().simulation().spawn(spawn).id;

    bool sawBall = false, sawBallFly = false;
    uint32_t t = Golf::kTicksPerGameDay + 7 * 60 * Golf::kTicksPerGameMinute;
    for (uint32_t end = t + 90 * Golf::kTicksPerGameMinute; t < end; t++)
    {
        runtime.update(t);
        const auto* golfer = runtime.director().simulation().find(id);
        if (golfer == nullptr)
            break;
        const auto balls = gameState.entities.getEntityListCount(EntityType::golfBall);
        if (golfer->ballInPlay && golfer->ball.state != Golf::BallState::Holed)
        {
            sawBall = sawBall || balls == 1;
            sawBallFly = sawBallFly || golfer->ball.state == Golf::BallState::InFlight;
            // One ball per golfer in play, and it is where the simulation has it.
            EXPECT_EQ(balls, 1u);
        }
    }
    EXPECT_TRUE(sawBall);
    EXPECT_TRUE(sawBallFly);
    // The round is over: no ball is left on the map.
    EXPECT_EQ(gameState.entities.getEntityListCount(EntityType::golfBall), 0u);
}

TEST(GolfRuntimeTests, SwingsPuttsAndTheCupMakeTheirSounds)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    gameState.cheats.sandboxMode = true;
    ObjectEntryIndex index{};
    auto* club = loadClub(*context, index);
    if (club == nullptr)
        GTEST_SKIP() << "the club house object is not installed";
    const auto found = findFlatBlock(3, 2);
    ASSERT_TRUE(found.has_value());
    insertPath(Building::GetEntrance(*club, *found, 0).outsideTile);
    GameActions::BuildingPlaceAction place(*found, index, 0);
    ASSERT_EQ(GameActions::ExecuteNested(&place, gameState).error, GameActions::Status::ok);
    gameState.golf.createCourse("Links");
    auto& runtime = Golf::runtimeOf(gameState);

    Golf::GolfSimulation::Spawn spawn;
    spawn.name = "Noisy";
    spawn.position = { Fixed::fromInt(96), Fixed::fromInt(160), Fixed::fromInt(16) };
    const uint32_t id = runtime.director().simulation().spawn(spawn).id;
    runtime.update(0); // the runtime learns where the golfer is

    std::vector<std::pair<Audio::SoundId, CoordsXYZ>> heard;
    runtime.setSoundSink([&](Audio::SoundId sound, const CoordsXYZ& where) { heard.emplace_back(sound, where); });
    auto event = [&](Golf::SimEvent::Type type, int32_t value) {
        Golf::SimEvent e;
        e.type = type;
        e.golferId = id;
        e.value = value;
        runtime.announce(e);
    };
    event(Golf::SimEvent::Type::shotHit, static_cast<int32_t>(Golf::ClubId::driver));
    event(Golf::SimEvent::Type::shotHit, static_cast<int32_t>(Golf::ClubId::wood3));
    event(Golf::SimEvent::Type::shotHit, static_cast<int32_t>(Golf::ClubId::iron7));
    event(Golf::SimEvent::Type::shotHit, static_cast<int32_t>(Golf::ClubId::pitchingWedge));
    event(Golf::SimEvent::Type::shotHit, static_cast<int32_t>(Golf::ClubId::putter));
    event(Golf::SimEvent::Type::shotHit, static_cast<int32_t>(Golf::ClubId::miniPutter));
    event(Golf::SimEvent::Type::holeFinished, 0);
    event(Golf::SimEvent::Type::holeFinished, -2);
    event(Golf::SimEvent::Type::rageQuit, 4);
    // Other events are silent.
    event(Golf::SimEvent::Type::ballSettled, 0);
    event(Golf::SimEvent::Type::paidFee, 500);
    event(Golf::SimEvent::Type::thought, 0);

    using Audio::SoundId;
    const std::vector<SoundId> expected = { SoundId::driverSwing, SoundId::driverSwing, SoundId::ironSwing, SoundId::ironSwing,
                                            SoundId::putt,        SoundId::putt,        SoundId::cupDrop,   SoundId::crowdClap,
                                            SoundId::golferInsultGrumble };
    ASSERT_EQ(heard.size(), expected.size());
    for (size_t i = 0; i < expected.size(); i++)
        EXPECT_EQ(heard[i].first, expected[i]) << "sound " << i;
    // The sounds come from where the golfer is.
    EXPECT_NEAR(heard[0].second.x, 96, 12);
    EXPECT_NEAR(heard[0].second.y, 160, 12);

    // An unknown golfer makes no sound.
    Golf::SimEvent stranger;
    stranger.type = Golf::SimEvent::Type::shotHit;
    stranger.golferId = 12345;
    runtime.announce(stranger);
    EXPECT_EQ(heard.size(), expected.size());
}

TEST(GolfRuntimeTests, BallImpactsAndGolferReactionsMakeSounds)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    gameState.cheats.sandboxMode = true;
    ObjectEntryIndex index{};
    auto* club = loadClub(*context, index);
    if (club == nullptr)
        GTEST_SKIP() << "the club house object is not installed";
    const auto found = findFlatBlock(3, 2);
    ASSERT_TRUE(found.has_value());
    insertPath(Building::GetEntrance(*club, *found, 0).outsideTile);
    GameActions::BuildingPlaceAction place(*found, index, 0);
    ASSERT_EQ(GameActions::ExecuteNested(&place, gameState).error, GameActions::Status::ok);
    gameState.mapSize = { 60, 60 };
    gameState.golf.createCourse("Links");
    auto& runtime = Golf::runtimeOf(gameState);

    Golf::GolfSimulation::Spawn spawn;
    spawn.name = "Audible";
    spawn.position = { Fixed::fromInt(100), Fixed::fromInt(200), Fixed::fromInt(16) };
    const uint32_t id = runtime.director().simulation().spawn(spawn).id;
    runtime.update(0);

    std::vector<std::pair<Audio::SoundId, CoordsXYZ>> heard;
    runtime.setSoundSink([&](Audio::SoundId sound, const CoordsXYZ& where) { heard.emplace_back(sound, where); });

    auto event = [&](Golf::SimEvent::Type type, int32_t value, int32_t x = 0, int32_t y = 0, int32_t z = 0) {
        Golf::SimEvent e;
        e.type = type;
        e.golferId = id;
        e.value = value;
        e.x = x;
        e.y = y;
        e.z = z;
        runtime.announce(e);
    };

    using Audio::SoundId;
    using Golf::BallImpactType;

    event(Golf::SimEvent::Type::cheer, 0);
    event(Golf::SimEvent::Type::groan, 0);
    event(Golf::SimEvent::Type::hitByBall, 0);

    event(Golf::SimEvent::Type::ballImpact, static_cast<int32_t>(BallImpactType::grass), 120, 220, 10);
    event(Golf::SimEvent::Type::ballImpact, static_cast<int32_t>(BallImpactType::sand), 130, 230, 10);
    event(Golf::SimEvent::Type::ballImpact, static_cast<int32_t>(BallImpactType::hard), 140, 240, 10);
    event(Golf::SimEvent::Type::ballImpact, static_cast<int32_t>(BallImpactType::water), 150, 250, 10);
    event(Golf::SimEvent::Type::ballImpact, static_cast<int32_t>(BallImpactType::tree), 160, 260, 10);
    event(Golf::SimEvent::Type::ballImpact, static_cast<int32_t>(BallImpactType::building), 170, 270, 10);

    const std::vector<SoundId> expected = {
        SoundId::golferCheer,
        SoundId::golferGroan,
        SoundId::golferGroan,
        SoundId::ballImpactGrass,
        SoundId::ballImpactSand,
        SoundId::ballImpactHard,
        SoundId::ballImpactWater,
        SoundId::ballImpactTree,
        SoundId::ballImpactBuilding,
    };

    ASSERT_EQ(heard.size(), expected.size());
    for (size_t i = 0; i < expected.size(); i++)
        EXPECT_EQ(heard[i].first, expected[i]) << "sound " << i;

    // Golfer reactions with 0,0 coords fallback to golfer's position
    EXPECT_NEAR(heard[0].second.x, 100, 12);
    EXPECT_NEAR(heard[0].second.y, 200, 12);

    // Ball impacts use the impact coordinates
    EXPECT_EQ(heard[3].second.x, 120);
    EXPECT_EQ(heard[3].second.y, 220);
    EXPECT_EQ(heard[3].second.z, 10);
}

TEST(GolfRuntimeTests, TheDuckAndTheDeerLoadWithTheirClipsWithinBudget)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& objectManager = context->GetObjectManager();
    for (const char* id : { "golf.animal.duck", "golf.animal.deer" })
    {
        auto* animal = static_cast<AnimalObject*>(objectManager.LoadObject(id));
        if (animal == nullptr)
            GTEST_SKIP() << id << " is not installed";
        ASSERT_NE(animal->ModelId, 0u) << id;
        const auto* model = Drawing::ModelSpriteStore::Instance().GetModel(animal->ModelId);
        ASSERT_NE(model, nullptr);
        // ANM-001, R3D-003.
        EXPECT_LE(model->triangleCount(), static_cast<size_t>(animal->TriangleBudget)) << id;
        for (auto clip : kRequiredAnimalClips)
            EXPECT_NE(model->findClip(std::string(clip)), nullptr) << id << " has no clip " << clip;
        EXPECT_TRUE(animal->ValidateRequiredClips());
    }
}

TEST(GolfRuntimeTests, AnimalEntitiesTurnTheWayTheyMoveAndRestartTheirAnimation)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    auto* entity = AnimalEntity::create(5, 3, { 100, 100, 4 });
    ASSERT_NE(entity, nullptr);
    EXPECT_EQ(entity->animal, 5u);
    EXPECT_EQ(entity->object, 3);
    EXPECT_EQ(gameState.entities.getEntityListCount(EntityType::animal), 1u);

    entity->follow({ 100, 100, 4 }, Golf::AnimalState::idle);
    entity->follow({ 100, 100, 4 }, Golf::AnimalState::idle);
    EXPECT_EQ(entity->animationTick, 2u);
    // A change of state starts the new clip from the beginning.
    entity->follow({ 100, 100, 4 }, Golf::AnimalState::walk);
    EXPECT_EQ(entity->animationTick, 0u);
    // Moving towards +x faces +x; towards +y faces +y (the same turns as the golfers).
    entity->follow({ 120, 100, 4 }, Golf::AnimalState::walk);
    EXPECT_EQ(entity->yaw, 128);
    entity->follow({ 120, 140, 4 }, Golf::AnimalState::walk);
    EXPECT_EQ(entity->yaw, 192);
    entity->remove();
    EXPECT_EQ(gameState.entities.getEntityListCount(EntityType::animal), 0u);
}

TEST(GolfRuntimeTests, EveryAnimalStateMapsToAClipTheDuckHas)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto* duck = static_cast<AnimalObject*>(context->GetObjectManager().LoadObject("golf.animal.duck"));
    if (duck == nullptr)
        GTEST_SKIP() << "the duck is not installed";
    const auto* model = Drawing::ModelSpriteStore::Instance().GetModel(duck->ModelId);
    ASSERT_NE(model, nullptr);
    for (auto state : { Golf::AnimalState::idle, Golf::AnimalState::walk, Golf::AnimalState::run, Golf::AnimalState::flee,
                        Golf::AnimalState::swim })
        EXPECT_NE(model->findClip(animalClipName(state)), nullptr) << animalClipName(state);
    // The duck has no flying clip in this model; the painter falls back to walking, so the name alone is not an error.
    EXPECT_STREQ(animalClipName(Golf::AnimalState::fly), "fly");
}

TEST(GolfRuntimeTests, TheSpeciesAreTheLoadedAnimalObjects)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    gameState.golf.createCourse("Links");
    auto& objectManager = context->GetObjectManager();
    if (objectManager.LoadObject("golf.animal.duck") == nullptr || objectManager.LoadObject("golf.animal.deer") == nullptr)
        GTEST_SKIP() << "the animals are not installed";
    auto& runtime = Golf::runtimeOf(gameState);
    runtime.update(0); // the species are looked at once a game minute
    const auto& species = runtime.director().animals->species();
    ASSERT_EQ(species.size(), 2u);
    bool duck = false, deer = false;
    for (const auto& s : species)
    {
        duck = duck || (s.id == "golf.animal.duck" && (s.habitat & Golf::kHabitatWater) != 0 && s.canSwim);
        deer = deer || (s.id == "golf.animal.deer" && (s.habitat & Golf::kHabitatWoods) != 0 && !s.canSwim);
    }
    EXPECT_TRUE(duck);
    EXPECT_TRUE(deer);
    // The group sizes and the spawn rate come from the objects.
    for (const auto& s : species)
    {
        EXPECT_GE(s.minGroup, 1);
        EXPECT_GE(s.maxGroup, s.minGroup);
        EXPECT_GT(s.spawnPerThousandPerDay, 0);
    }
}

TEST(GolfRuntimeTests, AnimalsAreDrawnAndAreSmallObstaclesForTheBalls)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    gameState.mapSize = { 60, 60 };
    gameState.golf.createCourse("Links");
    auto& runtime = Golf::runtimeOf(gameState);
    // A species that lives anywhere on dry ground, added by hand (the minute boundaries would replace it with the objects').
    Golf::AnimalSpecies everywhere;
    everywhere.id = "golf.animal.test";
    everywhere.habitat = Golf::kHabitatFairway | Golf::kHabitatRough | Golf::kHabitatWoods;
    everywhere.minGroup = everywhere.maxGroup = 2;
    runtime.director().animals->addSpecies(everywhere);
    const FixedVec3 centre{ Fixed::fromInt(10 * 32), Fixed::fromInt(10 * 32), Fixed{} };
    if (runtime.director().animals->spawnGroup(0, centre, 3 * 32) == 0)
        GTEST_SKIP() << "no dry ground at the centre of the test park";

    runtime.update(1);
    EXPECT_EQ(gameState.entities.getEntityListCount(EntityType::animal), 2u);
    // ANM-003: a ball meets the animal as an obstacle.
    const auto& animal = runtime.director().animals->animals()[0];
    Golf::ObstacleContact contact;
    const FixedVec3 at{ animal.position.x, animal.position.y, animal.position.z + Fixed::fromInt(2) };
    EXPECT_TRUE(Golf::GetMapObstacles().query(at, Fixed::ratio(1, 2), contact));
    EXPECT_TRUE(Golf::isAnimalObstacle(contact.id));
    EXPECT_EQ(Golf::animalOfObstacle(contact.id), animal.id);

    // Animals that are gone lose their entity and their obstacle.
    runtime.director().animals->setSpecies({});
    runtime.update(2);
    EXPECT_EQ(gameState.entities.getEntityListCount(EntityType::animal), 0u);
    Golf::ObstacleContact none;
    EXPECT_FALSE(Golf::GetMapObstacles().query(at, Fixed::ratio(1, 2), none));
}

TEST(GolfRuntimeTests, GolfersOnTheCourseAreObstaclesForBallsButNotForTheirOwn)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    gameState.cheats.sandboxMode = true;
    ObjectEntryIndex index{};
    auto* club = loadClub(*context, index);
    if (club == nullptr)
        GTEST_SKIP() << "the club house object is not installed";
    const auto found = findFlatBlock(3, 2);
    ASSERT_TRUE(found.has_value());
    insertPath(Building::GetEntrance(*club, *found, 0).outsideTile);
    GameActions::BuildingPlaceAction place(*found, index, 0);
    ASSERT_EQ(GameActions::ExecuteNested(&place, gameState).error, GameActions::Status::ok);
    auto* course = gameState.golf.createCourse("Links");
    course->open = true;
    auto* hole = gameState.golf.createHole(course->id);
    hole->tees[0] = Golf::HolePoint{ found->x + 64, found->y + 200 };
    hole->cup = Golf::HolePoint{ found->x + 64, found->y + 700 };
    auto& runtime = Golf::runtimeOf(gameState);
    runtime.director().setPopularity(0);

    Golf::GolfSimulation::Spawn spawn;
    spawn.courseId = course->id;
    spawn.money = 100000;
    spawn.position = runtime.buildings().club;
    const uint32_t id = runtime.director().simulation().spawn(spawn).id;
    const uint32_t start = Golf::kTicksPerGameDay + 7 * 60 * Golf::kTicksPerGameMinute;
    bool sawObstacle = false;
    for (uint32_t t = start; t < start + 40 * Golf::kTicksPerGameMinute; t++)
    {
        runtime.update(t);
        const auto* golfer = runtime.director().simulation().find(id);
        if (golfer == nullptr || golfer->state != Golf::GolferState::preparingShot)
            continue;
        Golf::ObstacleContact contact;
        const FixedVec3 at{ golfer->position.x, golfer->position.y, golfer->position.z + Fixed::fromInt(4) };
        sawObstacle = Golf::GetMapObstacles().query(at, Fixed::ratio(1, 2), contact);
        if (sawObstacle)
        {
            EXPECT_TRUE(contact.isGolfer);
            EXPECT_EQ(contact.golferId, id);
            EXPECT_EQ(contact.id, Golf::golferObstacleId(id));
            // The ball on the tee is theirs, and ignores them once it is hit.
            break;
        }
    }
    EXPECT_TRUE(sawObstacle);
}

TEST(GolfRuntimeTests, ADuckThatIsHitQuacksAndADeerChirps)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    gameState.mapSize = { 60, 60 };
    gameState.golf.createCourse("Links");
    auto& objectManager = context->GetObjectManager();
    if (objectManager.LoadObject("golf.animal.duck") == nullptr || objectManager.LoadObject("golf.animal.deer") == nullptr)
        GTEST_SKIP() << "the animals are not installed";
    auto& runtime = Golf::runtimeOf(gameState);
    runtime.update(0);
    std::vector<Audio::SoundId> heard;
    runtime.setSoundSink([&](Audio::SoundId sound, const CoordsXYZ&) { heard.push_back(sound); });

    // Put one of each on dry ground (the test park has no pond for the duck) and knock them.
    auto& animals = *runtime.director().animals;
    auto species = animals.species();
    ASSERT_EQ(species.size(), 2u);
    for (auto& s : species)
    {
        s.habitat = Golf::kHabitatFairway | Golf::kHabitatRough | Golf::kHabitatWoods;
        s.minGroup = s.maxGroup = 1;
    }
    animals.setSpecies(species);
    for (uint32_t index = 0; index < species.size(); index++)
    {
        if (animals.spawnGroup(index, { Fixed::fromInt(10 * 32), Fixed::fromInt(10 * 32), Fixed{} }, 6 * 32) == 0)
            GTEST_SKIP() << "the test park has no dry ground";
    }
    for (const auto& animal : animals.animals())
        animals.hitByBall(animal.id);
    runtime.update(1);
    // One sound each: the duck's own and the chirp of the deer.
    EXPECT_EQ(heard.size(), 2u);
    EXPECT_NE(std::find(heard.begin(), heard.end(), Audio::SoundId::duck), heard.end());
    EXPECT_NE(std::find(heard.begin(), heard.end(), Audio::SoundId::birdChirp), heard.end());
}

TEST(GolfRuntimeTests, AVipBuysACottageLotWhenTheCourseIsPopular)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    gameState.cheats.sandboxMode = true;
    gameState.mapSize = { 60, 60 };
    auto& objectManager = context->GetObjectManager();
    ObjectEntryIndex clubIndex{};
    auto* club = loadClub(*context, clubIndex);
    if (club == nullptr || objectManager.LoadObject("golf.building.cottage") == nullptr)
        GTEST_SKIP() << "the club house or the cottage is not installed";

    // The lot: a block of bare ground with a path all round it.
    const auto block = findFlatBlock(4, 3);
    ASSERT_TRUE(block.has_value());
    for (int32_t y = -1; y <= 3; y++)
        for (int32_t x = -1; x <= 4; x++)
            if (x == -1 || x == 4 || y == -1 || y == 3)
                insertPath({ block->x + x * 32, block->y + y * 32 });
    const CoordsXY lotMax{ block->x + 3 * 32, block->y + 2 * 32 };
    GameActions::ZoneEditAction zone(GameActions::ZoneEditType::createLot, 0, *block, lotMax);
    const auto created = GameActions::ExecuteNested(&zone, gameState);
    ASSERT_EQ(created.error, GameActions::Status::ok);
    const auto lotId = created.getData<GameActions::ZoneEditResult>().zoneId;

    // The club, somewhere else.
    const auto clubAt = findFlatBlock(3, 2);
    ASSERT_TRUE(clubAt.has_value());
    insertPath(Building::GetEntrance(*club, *clubAt, 0).outsideTile);
    GameActions::BuildingPlaceAction placeClub(*clubAt, clubIndex, 0);
    ASSERT_EQ(GameActions::ExecuteNested(&placeClub, gameState).error, GameActions::Status::ok);
    gameState.golf.createCourse("Links");

    auto& runtime = Golf::runtimeOf(gameState);
    runtime.director().setPopularity(1000);
    const auto cash = gameState.park.cash;
    // Days go by until the lot is bought (about one in five a day at this popularity).
    for (uint32_t day = 1; day < 120 && !gameState.golf.findLot(lotId)->sold; day++)
        runtime.update(day * Golf::kTicksPerGameDay);
    const auto* lot = gameState.golf.findLot(lotId);
    ASSERT_TRUE(lot->sold) << "the lot was never sold";

    // The park was paid for the land, and the cottage stands on it.
    EXPECT_GT(lot->pricePaid, 0);
    // (less the upkeep of the cottage for the days since it was built)
    EXPECT_LE(gameState.park.cash - cash, lot->pricePaid);
    EXPECT_GT(gameState.park.cash - cash, lot->pricePaid - 2000);
    bool cottage = false;
    for (int32_t y = lot->min.y; y <= lot->max.y; y++)
        for (int32_t x = lot->min.x; x <= lot->max.x; x++)
            cottage = cottage || Building::FindAt({ x * 32, y * 32 }) != nullptr;
    EXPECT_TRUE(cottage);
    // ECO-033: the land is no longer the park's, so nothing can be built on it.
    for (int32_t y = lot->min.y; y <= lot->max.y; y++)
        for (int32_t x = lot->min.x; x <= lot->max.x; x++)
            EXPECT_FALSE(MapGetSurfaceElementAt(CoordsXY{ x * 32, y * 32 })->hasOwnership(OwnershipFlag::landOwned));
    // The buyer is a VIP in the registry, who cannot be forgotten.
    const auto* owner = runtime.director().registry().find(lot->ownerGolferId);
    ASSERT_NE(owner, nullptr);
    EXPECT_TRUE(owner->vip);
    EXPECT_EQ(owner->lotId, lotId);
    // And the lot is not for sale a second time.
    GameActions::ZoneEditAction again(GameActions::ZoneEditType::removeLot, lotId);
    EXPECT_EQ(GameActions::Query(&again, gameState).error, GameActions::Status::invalidParameters);
}

TEST(GolfRuntimeTests, NoLotSellsBelowThePopularityItNeeds)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    gameState.cheats.sandboxMode = true;
    gameState.mapSize = { 60, 60 };
    ObjectEntryIndex clubIndex{};
    auto* club = loadClub(*context, clubIndex);
    if (club == nullptr || context->GetObjectManager().LoadObject("golf.building.cottage") == nullptr)
        GTEST_SKIP() << "the club house or the cottage is not installed";
    const auto block = findFlatBlock(4, 3);
    ASSERT_TRUE(block.has_value());
    for (int32_t y = -1; y <= 3; y++)
        for (int32_t x = -1; x <= 4; x++)
            if (x == -1 || x == 4 || y == -1 || y == 3)
                insertPath({ block->x + x * 32, block->y + y * 32 });
    GameActions::ZoneEditAction zone(GameActions::ZoneEditType::createLot, 0, *block, CoordsXY{ block->x + 96, block->y + 64 });
    const auto lotId = GameActions::ExecuteNested(&zone, gameState).getData<GameActions::ZoneEditResult>().zoneId;
    const auto clubAt = findFlatBlock(3, 2);
    ASSERT_TRUE(clubAt.has_value());
    insertPath(Building::GetEntrance(*club, *clubAt, 0).outsideTile);
    GameActions::BuildingPlaceAction placeClub(*clubAt, clubIndex, 0);
    ASSERT_EQ(GameActions::ExecuteNested(&placeClub, gameState).error, GameActions::Status::ok);
    gameState.golf.createCourse("Links");

    auto& runtime = Golf::runtimeOf(gameState);
    runtime.director().setPopularity(100);
    for (uint32_t day = 1; day < 200; day++)
    {
        runtime.director().setPopularity(100); // the day's popularity update would raise it
        runtime.update(day * Golf::kTicksPerGameDay);
    }
    EXPECT_FALSE(gameState.golf.findLot(lotId)->sold);
}

TEST(GolfRuntimeTests, ACottageOwnerPlaysFromTheirOwnDoor)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    gameState.cheats.sandboxMode = true;
    gameState.mapSize = { 60, 60 };
    auto& objectManager = context->GetObjectManager();
    ObjectEntryIndex clubIndex{};
    auto* club = loadClub(*context, clubIndex);
    if (club == nullptr || objectManager.LoadObject("golf.building.cottage") == nullptr)
        GTEST_SKIP() << "the club house or the cottage is not installed";

    const auto block = findFlatBlock(4, 3);
    ASSERT_TRUE(block.has_value());
    for (int32_t y = -1; y <= 3; y++)
        for (int32_t x = -1; x <= 4; x++)
            if (x == -1 || x == 4 || y == -1 || y == 3)
                insertPath({ block->x + x * 32, block->y + y * 32 });
    GameActions::ZoneEditAction zone(GameActions::ZoneEditType::createLot, 0, *block, CoordsXY{ block->x + 96, block->y + 64 });
    const auto lotId = GameActions::ExecuteNested(&zone, gameState).getData<GameActions::ZoneEditResult>().zoneId;
    const auto clubAt = findFlatBlock(3, 2);
    ASSERT_TRUE(clubAt.has_value());
    insertPath(Building::GetEntrance(*club, *clubAt, 0).outsideTile);
    GameActions::BuildingPlaceAction placeClub(*clubAt, clubIndex, 0);
    ASSERT_EQ(GameActions::ExecuteNested(&placeClub, gameState).error, GameActions::Status::ok);

    auto* course = gameState.golf.createCourse("Links");
    course->open = true;
    auto* hole = gameState.golf.createHole(course->id);
    hole->tees[0] = Golf::HolePoint{ clubAt->x + 64, clubAt->y + 200 };
    hole->cup = Golf::HolePoint{ clubAt->x + 64, clubAt->y + 700 };

    auto& runtime = Golf::runtimeOf(gameState);
    runtime.director().setPopularity(1000);
    for (uint32_t day = 1; day < 120 && !gameState.golf.findLot(lotId)->sold; day++)
        runtime.update(day * Golf::kTicksPerGameDay);
    const auto* lot = gameState.golf.findLot(lotId);
    ASSERT_TRUE(lot->sold) << "the lot was never sold";
    const auto owner = lot->ownerGolferId;

    // Over the next days the owner turns up on foot beside their cottage, within the opening hours, and only ever one at a
    // time.
    // The owner is only admitted on a game minute, so the days are stepped a minute at a time until they turn up; then
    // the next hour runs tick by tick to count how many there are.
    bool seen = false;
    int32_t most = 0;
    uint32_t seenAt = 0;
    const auto countVips = [&] {
        int32_t vips = 0;
        for (const auto& golfer : runtime.director().simulation().golfers())
            if (golfer.vip)
                vips++;
        most = std::max(most, vips);
    };
    for (uint32_t day = 130; day < 200 && !seen; day++)
    {
        // Nobody else comes: golfers do not walk while the minutes are skipped, so they would only fill the course.
        runtime.director().setPopularity(0);
        for (uint32_t t = day * Golf::kTicksPerGameDay + 7 * 60 * Golf::kTicksPerGameMinute;
             t < day * Golf::kTicksPerGameDay + 18 * 60 * Golf::kTicksPerGameMinute && !seen; t += Golf::kTicksPerGameMinute)
        {
            runtime.update(t);
            for (const auto& golfer : runtime.director().simulation().golfers())
            {
                if (!golfer.vip)
                    continue;
                seen = true;
                seenAt = t;
                // They came out of the cottage on the lot, which is a few tiles from where they are now at most.
                EXPECT_LT(std::abs(static_cast<int32_t>(golfer.position.x.floor()) - (lot->min.x * 32 + 64)), 6 * 32);
                EXPECT_LT(std::abs(static_cast<int32_t>(golfer.position.y.floor()) - (lot->min.y * 32 + 48)), 6 * 32);
                break;
            }
        }
    }
    countVips();
    for (uint32_t t = seenAt + 1; seen && t < seenAt + 60 * Golf::kTicksPerGameMinute; t++)
    {
        runtime.update(t);
        countVips();
    }
    EXPECT_TRUE(seen);
    EXPECT_EQ(most, 1);
    (void)owner;
}

TEST(GolfRuntimeTests, TheTournamentChecklistComesFromTheMapAndABookingNeedsItMet)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    gameState.cheats.sandboxMode = true;
    gameState.mapSize = { 60, 60 };
    auto& objectManager = context->GetObjectManager();
    ObjectEntryIndex clubIndex{};
    auto* club = loadClub(*context, clubIndex);
    auto* road = objectManager.LoadObject("golf.street.road");
    if (club == nullptr || road == nullptr)
        GTEST_SKIP() << "the club house or the street is not installed";
    const auto roadIndex = objectManager.GetLoadedObjectEntryIndex(road);

    // 18 holes of 60 tiles (300 yards): 5400 yards, par 72.
    auto* course = gameState.golf.createCourse("Championship");
    course->open = true;
    for (int i = 0; i < 18; i++)
    {
        auto* hole = gameState.golf.createHole(course->id);
        hole->tees[0] = Golf::HolePoint{ 100, (4 + i) * 32 };
        hole->cup = Golf::HolePoint{ 100 + 60 * 32, (4 + i) * 32 };
    }
    ASSERT_EQ(course->totalPar(), 72);

    // A street with ten tiles of parking along it and a path: 20 spaces. And the club.
    const auto lotBlock = findFlatBlock(12, 3);
    if (!lotBlock.has_value())
        GTEST_SKIP() << "the test park has no room for a car park";
    for (int x = 0; x < 10; x++)
    {
        GameActions::StreetPlaceAction street({ lotBlock->x + x * 32, lotBlock->y }, roadIndex);
        ASSERT_EQ(GameActions::ExecuteNested(&street, gameState).error, GameActions::Status::ok);
        GameActions::ParkingPlaceAction parking({ lotBlock->x + x * 32, lotBlock->y + 32 }, roadIndex, 3);
        ASSERT_EQ(GameActions::ExecuteNested(&parking, gameState).error, GameActions::Status::ok);
        insertPath({ lotBlock->x + x * 32, lotBlock->y + 64 });
    }
    const auto clubAt = findFlatBlock(3, 2);
    ASSERT_TRUE(clubAt.has_value());
    insertPath(Building::GetEntrance(*club, *clubAt, 0).outsideTile);
    GameActions::BuildingPlaceAction placeClub(*clubAt, clubIndex, 0);
    ASSERT_EQ(GameActions::ExecuteNested(&placeClub, gameState).error, GameActions::Status::ok);

    auto& runtime = Golf::runtimeOf(gameState);
    runtime.director().setPopularity(500);
    runtime.update(0);
    auto inputs = runtime.tournamentInputs();
    EXPECT_EQ(inputs.holes, 18);
    EXPECT_EQ(inputs.totalYards, 18 * 300);
    EXPECT_EQ(inputs.totalPar, 72);
    EXPECT_EQ(inputs.popularity, 500);
    EXPECT_EQ(inputs.golfClubs, 1);
    EXPECT_EQ(inputs.parkingCapacity, 20);
    EXPECT_EQ(inputs.spectatorTiles, 0);

    // The zones are not drawn yet: the booking is refused.
    GameActions::TournamentEditAction book(GameActions::TournamentEditType::book, 0, 5, 1500);
    EXPECT_EQ(GameActions::Query(&book, gameState).error, GameActions::Status::invalidParameters);

    auto zone = [&](Golf::TournamentZoneKind kind, int32_t x, int32_t y, int32_t w, int32_t h) {
        GameActions::ZoneEditAction action(
            GameActions::ZoneEditType::createTournamentZone, 0, CoordsXY{ x * 32, y * 32 }, CoordsXY{ (x + w - 1) * 32, (y + h - 1) * 32 },
            static_cast<int32_t>(kind));
        EXPECT_EQ(GameActions::ExecuteNested(&action, gameState).error, GameActions::Status::ok);
    };
    zone(Golf::TournamentZoneKind::spectator, 2, 30, 5, 4);       // 20 tiles
    zone(Golf::TournamentZoneKind::media, 8, 30, 2, 2);           // 4
    zone(Golf::TournamentZoneKind::extraParking, 12, 30, 5, 4);   // 20
    zone(Golf::TournamentZoneKind::grandstand, 61 - 10, 4 + 17, 3, 2); // 6, by the last green (tile 61, 21)
    inputs = runtime.tournamentInputs();
    EXPECT_EQ(inputs.spectatorTiles, 20);
    EXPECT_EQ(inputs.mediaTiles, 4);
    EXPECT_EQ(inputs.extraParkingTiles, 20);
    EXPECT_EQ(inputs.grandstandTiles, 6);
    EXPECT_LE(inputs.grandstandDistanceToLastGreen, 12);
    EXPECT_TRUE(Golf::canHost(Golf::TournamentTier::local, inputs));
    EXPECT_FALSE(Golf::canHost(Golf::TournamentTier::national, inputs));

    // Now it can be booked: for a day to come, with a ticket price.
    GameActions::TournamentEditAction past(GameActions::TournamentEditType::book, 0, 0, 1500);
    EXPECT_EQ(GameActions::Query(&past, gameState).error, GameActions::Status::invalidParameters);
    GameActions::TournamentEditAction national(GameActions::TournamentEditType::book, 2, 5, 1500);
    EXPECT_EQ(GameActions::Query(&national, gameState).error, GameActions::Status::invalidParameters);
    GameActions::TournamentEditAction cheap(GameActions::TournamentEditType::book, 0, 5, -1);
    EXPECT_EQ(GameActions::Query(&cheap, gameState).error, GameActions::Status::invalidParameters);
    ASSERT_EQ(GameActions::ExecuteNested(&book, gameState).error, GameActions::Status::ok);
    const auto& tournament = runtime.director().tournament;
    EXPECT_EQ(tournament.phase, Golf::TournamentPhase::scheduled);
    EXPECT_EQ(tournament.tier, Golf::TournamentTier::local);
    EXPECT_EQ(tournament.day, runtime.director().currentDay() + 5);
    EXPECT_EQ(tournament.ticketPrice, 1500);

    // One at a time; cancelling frees the calendar.
    GameActions::TournamentEditAction second(GameActions::TournamentEditType::book, 0, 9, 1500);
    EXPECT_EQ(GameActions::Query(&second, gameState).error, GameActions::Status::invalidParameters);
    GameActions::TournamentEditAction cancel(GameActions::TournamentEditType::cancel);
    ASSERT_EQ(GameActions::ExecuteNested(&cancel, gameState).error, GameActions::Status::ok);
    EXPECT_EQ(runtime.director().tournament.phase, Golf::TournamentPhase::none);
    EXPECT_EQ(GameActions::Query(&cancel, gameState).error, GameActions::Status::invalidParameters);
}

TEST(GolfRuntimeTests, TheClubGetsItsPriceAndCanBeClosedAndKeepsItsBooks)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    gameState.cheats.sandboxMode = true;
    ObjectEntryIndex index{};
    auto* club = loadClub(*context, index);
    if (club == nullptr)
        GTEST_SKIP() << "the club house object is not installed";
    const auto found = findFlatBlock(3, 2);
    ASSERT_TRUE(found.has_value());
    insertPath(Building::GetEntrance(*club, *found, 0).outsideTile);
    GameActions::BuildingPlaceAction place(*found, index, 0);
    ASSERT_EQ(GameActions::ExecuteNested(&place, gameState).error, GameActions::Status::ok);
    auto* course = gameState.golf.createCourse("Links");
    course->open = true;
    auto* hole = gameState.golf.createHole(course->id);
    hole->tees[0] = Golf::HolePoint{ found->x + 64, found->y + 200 };
    hole->cup = Golf::HolePoint{ found->x + 64, found->y + 500 };

    auto& runtime = Golf::runtimeOf(gameState);
    runtime.director().setPopularity(0);
    runtime.update(0);
    auto& services = runtime.director().simulation().environment().services;
    ASSERT_EQ(services.size(), 1u);
    EXPECT_EQ(services[0].price, Golf::defaultServicePrice(BuildingKind::golfClub));
    const auto key = services[0].id;
    // The id is the tile of the building, not the order it was found in.
    EXPECT_EQ(key, Golf::buildingKey(found->x / 32, found->y / 32));

    // Any tile of the building will do to name it.
    const CoordsXY otherPart{ found->x + 32, found->y + 32 };
    ASSERT_NE(Building::FindAt(otherPart), nullptr);
    GameActions::BuildingEditAction price(otherPart, GameActions::BuildingEditType::setPrice, 725);
    ASSERT_EQ(GameActions::ExecuteNested(&price, gameState).error, GameActions::Status::ok);
    runtime.update(Golf::kTicksPerGameMinute);
    EXPECT_EQ(runtime.director().simulation().environment().services[0].price, 725);

    // -1 goes back to the usual price.
    GameActions::BuildingEditAction usual(*found, GameActions::BuildingEditType::setPrice, -1);
    ASSERT_EQ(GameActions::ExecuteNested(&usual, gameState).error, GameActions::Status::ok);
    runtime.update(2 * Golf::kTicksPerGameMinute);
    EXPECT_EQ(runtime.director().simulation().environment().services[0].price, Golf::defaultServicePrice(BuildingKind::golfClub));

    // A closed building serves no one (the golfers go elsewhere), and opens again.
    GameActions::BuildingEditAction close(*found, GameActions::BuildingEditType::setClosed, 1);
    ASSERT_EQ(GameActions::ExecuteNested(&close, gameState).error, GameActions::Status::ok);
    runtime.update(3 * Golf::kTicksPerGameMinute);
    EXPECT_TRUE(runtime.director().simulation().environment().services.empty());
    GameActions::BuildingEditAction open(*found, GameActions::BuildingEditType::setClosed, 0);
    ASSERT_EQ(GameActions::ExecuteNested(&open, gameState).error, GameActions::Status::ok);
    runtime.update(4 * Golf::kTicksPerGameMinute);
    EXPECT_EQ(runtime.director().simulation().environment().services.size(), 1u);

    // Errors: nothing there, a bad value, a building with nothing to set.
    GameActions::BuildingEditAction nothing({ 50 * 32, 50 * 32 }, GameActions::BuildingEditType::setPrice, 100);
    EXPECT_EQ(GameActions::Query(&nothing, gameState).error, GameActions::Status::invalidParameters);
    GameActions::BuildingEditAction negative(*found, GameActions::BuildingEditType::setPrice, -5);
    EXPECT_EQ(GameActions::Query(&negative, gameState).error, GameActions::Status::invalidParameters);
    GameActions::BuildingEditAction twoState(*found, GameActions::BuildingEditType::setClosed, 2);
    EXPECT_EQ(GameActions::Query(&twoState, gameState).error, GameActions::Status::invalidParameters);

    // A golfer who is hungry eats at the club: the visit and its price go into the books of the building.
    runtime.director().buildings[key].price = 500;
    Golf::GolfSimulation::Spawn spawn;
    spawn.courseId = course->id;
    spawn.money = 100000;
    spawn.position = runtime.buildings().club;
    const uint32_t id = runtime.director().simulation().spawn(spawn).id;
    bool starved = false;
    const uint32_t start = Golf::kTicksPerGameDay + 7 * 60 * Golf::kTicksPerGameMinute;
    for (uint32_t t = start; t < start + 60 * Golf::kTicksPerGameMinute; t++)
    {
        runtime.update(t);
        if (auto* golfer = runtime.director().simulation().find(id); golfer != nullptr && !starved
            && golfer->state == Golf::GolferState::preparingShot)
        {
            golfer->needs.hunger = 8 * 16; // critical
            starved = true;
        }
    }
    ASSERT_TRUE(starved);
    const auto& book = runtime.director().buildings[key];
    EXPECT_GE(book.visitorsThisMonth, 1);
    EXPECT_EQ(book.revenueThisMonth, book.visitorsThisMonth * 500);
}
