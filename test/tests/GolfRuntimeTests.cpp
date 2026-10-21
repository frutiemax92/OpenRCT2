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
#include <openrct2/entity/GolferEntity.h>
#include <openrct2/golf/GolfRuntime.h>
#include <openrct2/golf/Hole.h>
#include <openrct2/object/GolferModelObject.h>
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
