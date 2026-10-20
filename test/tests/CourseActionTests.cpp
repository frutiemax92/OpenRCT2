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
#include <cstring>
#include <memory>
#include <optional>
#include <openrct2/Context.h>
#include <openrct2/Game.h>
#include <openrct2/GameState.h>
#include <openrct2/OpenRCT2.h>
#include <openrct2/ParkImporter.h>
#include <openrct2/actions/GameActionRunner.h>
#include <openrct2/actions/golf/CourseEditAction.h>
#include <openrct2/actions/golf/HoleEditAction.h>
#include <openrct2/core/DataSerialiser.h>
#include <openrct2/core/MemoryStream.h>
#include <openrct2/drawing/Drawing.h>
#include <openrct2/drawing/Palette.h>
#include <openrct2/entity/EntityRegistry.h>
#include <openrct2/entity/EntityTweener.h>
#include <openrct2/object/ObjectManager.h>
#include <openrct2/world/Map.h>
#include <openrct2/world/MapAnimation.h>
#include <openrct2/world/tile_element/SurfaceElement.h>

using namespace OpenRCT2;
using namespace OpenRCT2::GameActions;

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
        gameState.cheats.sandboxMode = true;
        return context;
    }

    template<typename TAction>
    Result run(TAction& action)
    {
        auto query = GameActions::Query(&action, getGameState());
        if (query.error != Status::ok)
            return query;
        return GameActions::ExecuteNested(&action, getGameState());
    }

    CourseEditResult createCourse(const std::string& name = "Links")
    {
        CourseEditAction action(CourseEditType::create, 0, name);
        auto res = run(action);
        EXPECT_EQ(res.error, Status::ok);
        return res.getData<CourseEditResult>();
    }

    // A dry flat tile with only its surface.
    CoordsXY bareTile(int skip = 0)
    {
        for (int32_t y = 3; y < 60; y++)
            for (int32_t x = 3; x < 60; x++)
            {
                const CoordsXY tile{ x * 32, y * 32 };
                auto* surface = MapGetSurfaceElementAt(tile);
                if (surface != nullptr && surface->getWaterHeight() == 0 && skip-- == 0)
                    return tile;
            }
        return { 96, 96 };
    }
} // namespace

TEST(CourseActions, CreateRenameAndConfigureACourse)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    const auto created = createCourse("Old Course");
    EXPECT_NE(created.courseId, 0u);
    auto* course = getGameState().golf.findCourse(created.courseId);
    ASSERT_NE(course, nullptr);
    EXPECT_EQ(course->name, "Old Course");
    EXPECT_FALSE(course->open);

    CourseEditAction rename(CourseEditType::rename, created.courseId, "New Course");
    EXPECT_EQ(run(rename).error, Status::ok);
    CourseEditAction fee(CourseEditType::setFee, created.courseId, {}, 4500);
    EXPECT_EQ(run(fee).error, Status::ok);
    CourseEditAction open(CourseEditType::setOpen, created.courseId, {}, 1);
    EXPECT_EQ(run(open).error, Status::ok);
    course = getGameState().golf.findCourse(created.courseId);
    EXPECT_EQ(course->name, "New Course");
    EXPECT_EQ(course->greenFee, 4500);
    EXPECT_TRUE(course->open);

    CourseEditAction badFee(CourseEditType::setFee, created.courseId, {}, -1);
    EXPECT_EQ(run(badFee).error, Status::invalidParameters);
    CourseEditAction badOpen(CourseEditType::setOpen, created.courseId, {}, 2);
    EXPECT_EQ(run(badOpen).error, Status::invalidParameters);
    CourseEditAction missing(CourseEditType::rename, 999, "x");
    EXPECT_EQ(run(missing).error, Status::invalidParameters);
    CourseEditAction longName(CourseEditType::rename, created.courseId, std::string(200, 'x'));
    EXPECT_EQ(run(longName).error, Status::invalidParameters);

    CourseEditAction remove(CourseEditType::remove, created.courseId);
    EXPECT_EQ(run(remove).error, Status::ok);
    EXPECT_EQ(getGameState().golf.findCourse(created.courseId), nullptr);
}

TEST(CourseActions, TheNumberOfCoursesIsLimited)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    for (size_t i = 0; i < Golf::CourseManager::kMaxCourses; i++)
        createCourse();
    CourseEditAction one(CourseEditType::create, 0, "too many");
    EXPECT_EQ(run(one).error, Status::disallowed);
}

TEST(CourseActions, PlacingATeeStartsANewHoleAndBuildsItUp)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    const auto course = createCourse();
    const auto tile = bareTile();

    HoleEditAction tee(course.courseId, 0, HoleEditType::placeTee, 0, { tile.x + 5, tile.y + 9 });
    auto res = run(tee);
    ASSERT_EQ(res.error, Status::ok);
    const auto holeId = res.getData<CourseEditResult>().holeId;
    EXPECT_NE(holeId, 0u);

    auto* hole = getGameState().golf.findCourse(course.courseId)->findHole(holeId);
    ASSERT_NE(hole, nullptr);
    // The tee box is the whole tile: its position is the centre.
    EXPECT_EQ(hole->tees[0], (Golf::HolePoint{ tile.x + 16, tile.y + 16 }));
    EXPECT_FALSE(hole->isComplete());

    // More tees and aim points on the same hole.
    const auto second = bareTile(4);
    HoleEditAction womens(course.courseId, holeId, HoleEditType::placeTee, 1, second);
    EXPECT_EQ(run(womens).error, Status::ok);
    HoleEditAction aim(course.courseId, holeId, HoleEditType::placeAim, 0, { 600, 700 });
    EXPECT_EQ(run(aim).error, Status::ok);
    HoleEditAction par(course.courseId, holeId, HoleEditType::setPar, 5);
    EXPECT_EQ(run(par).error, Status::ok);
    hole = getGameState().golf.findCourse(course.courseId)->findHole(holeId);
    EXPECT_TRUE(hole->tees[1].has_value());
    EXPECT_EQ(hole->aimPoints[0], (Golf::HolePoint{ 600, 700 }));
    EXPECT_EQ(hole->parOverride, 5);

    HoleEditAction removeWomens(course.courseId, holeId, HoleEditType::removeTee, 1);
    EXPECT_EQ(run(removeWomens).error, Status::ok);
    HoleEditAction removeAim(course.courseId, holeId, HoleEditType::removeAim, 0);
    EXPECT_EQ(run(removeAim).error, Status::ok);
    hole = getGameState().golf.findCourse(course.courseId)->findHole(holeId);
    EXPECT_FALSE(hole->tees[1].has_value());
    EXPECT_FALSE(hole->aimPoints[0].has_value());
}

TEST(CourseActions, ATeeBoxTurnsItsTileIntoTheTeeSurface)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& objectManager = context->GetObjectManager();
    auto* object = objectManager.LoadObject("golf.terrain_surface.tee");
    if (object == nullptr)
        GTEST_SKIP() << "the tee surface is not installed";
    const auto teeIndex = objectManager.GetLoadedObjectEntryIndex(object);

    const auto course = createCourse();
    const auto tile = bareTile(7);
    HoleEditAction tee(course.courseId, 0, HoleEditType::placeTee, 0, tile);
    ASSERT_EQ(run(tee).error, Status::ok);
    auto* surface = MapGetSurfaceElementAt(tile);
    for (auto q : { Terrain::Quadrant::northEast, Terrain::Quadrant::southEast, Terrain::Quadrant::southWest,
                    Terrain::Quadrant::northWest })
        EXPECT_EQ(surface->getQuadrantSurfaceIndex(q), teeIndex);
}

TEST(CourseActions, TheCupNeedsAGreen)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& objectManager = context->GetObjectManager();
    auto* green = objectManager.LoadObject("golf.terrain_surface.green");
    auto* fairway = objectManager.LoadObject("golf.terrain_surface.fairway");
    if (green == nullptr || fairway == nullptr)
        GTEST_SKIP() << "the golf surfaces are not installed";
    const auto greenIndex = objectManager.GetLoadedObjectEntryIndex(green);
    const auto fairwayIndex = objectManager.GetLoadedObjectEntryIndex(fairway);

    const auto course = createCourse();
    const auto onFairway = bareTile(10);
    const auto onGreen = bareTile(12);
    for (auto q : { Terrain::Quadrant::northEast, Terrain::Quadrant::southEast, Terrain::Quadrant::southWest,
                    Terrain::Quadrant::northWest })
    {
        MapGetSurfaceElementAt(onFairway)->setQuadrantSurfaceIndex(q, fairwayIndex);
        MapGetSurfaceElementAt(onGreen)->setQuadrantSurfaceIndex(q, greenIndex);
    }

    HoleEditAction bad(course.courseId, 0, HoleEditType::placeCup, 0, { onFairway.x + 8, onFairway.y + 8 });
    EXPECT_EQ(run(bad).error, Status::disallowed);
    EXPECT_TRUE(getGameState().golf.findCourse(course.courseId)->holes.empty()); // nothing was created

    HoleEditAction good(course.courseId, 0, HoleEditType::placeCup, 0, { onGreen.x + 8, onGreen.y + 8 });
    auto res = run(good);
    ASSERT_EQ(res.error, Status::ok);
    auto* hole = getGameState().golf.findCourse(course.courseId)->findHole(res.getData<CourseEditResult>().holeId);
    EXPECT_EQ(hole->cup, (Golf::HolePoint{ onGreen.x + 8, onGreen.y + 8 }));

    HoleEditAction remove(course.courseId, hole->id, HoleEditType::removeCup, 0);
    EXPECT_EQ(run(remove).error, Status::ok);
    EXPECT_FALSE(getGameState().golf.findCourse(course.courseId)->findHole(hole->id)->cup.has_value());
}

TEST(CourseActions, PointsMustBeOnDryLandOnTheMap)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    const auto course = createCourse();
    HoleEditAction offMap(course.courseId, 0, HoleEditType::placeTee, 0, { -64, 100 });
    EXPECT_EQ(run(offMap).error, Status::invalidParameters);
    const auto tile = bareTile(3);
    auto* surface = MapGetSurfaceElementAt(tile);
    surface->setWaterHeight(surface->getBaseZ() + 16);
    HoleEditAction wet(course.courseId, 0, HoleEditType::placeAim, 0, { tile.x + 8, tile.y + 8 });
    EXPECT_EQ(run(wet).error, Status::disallowed);
    EXPECT_TRUE(getGameState().golf.findCourse(course.courseId)->holes.empty());
}

TEST(CourseActions, BadParametersAreRejected)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    const auto course = createCourse();
    const auto tile = bareTile();
    HoleEditAction badCategory(course.courseId, 0, HoleEditType::placeTee, 3, tile);
    EXPECT_EQ(run(badCategory).error, Status::invalidParameters);
    HoleEditAction badAim(course.courseId, 0, HoleEditType::placeAim, 2, tile);
    EXPECT_EQ(run(badAim).error, Status::invalidParameters);
    HoleEditAction noCourse(999, 0, HoleEditType::placeTee, 0, tile);
    EXPECT_EQ(run(noCourse).error, Status::invalidParameters);
    HoleEditAction noHole(course.courseId, 77, HoleEditType::placeTee, 0, tile);
    EXPECT_EQ(run(noHole).error, Status::invalidParameters);
    // Only placing something creates a hole.
    HoleEditAction removeFromNothing(course.courseId, 0, HoleEditType::removeCup, 0);
    EXPECT_EQ(run(removeFromNothing).error, Status::invalidParameters);
    HoleEditAction badPar(course.courseId, 0, HoleEditType::setPar, 3);
    EXPECT_EQ(run(badPar).error, Status::invalidParameters);
}

TEST(CourseActions, ACourseHoldsEighteenHolesAndTheyCanBeReorderedAndRemoved)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    const auto course = createCourse();
    std::vector<uint32_t> ids;
    for (int i = 0; i < 18; i++)
    {
        HoleEditAction tee(course.courseId, 0, HoleEditType::placeTee, 0, bareTile(i));
        auto res = run(tee);
        ASSERT_EQ(res.error, Status::ok) << i;
        ids.push_back(res.getData<CourseEditResult>().holeId);
    }
    HoleEditAction nineteenth(course.courseId, 0, HoleEditType::placeTee, 0, bareTile(20));
    EXPECT_EQ(run(nineteenth).error, Status::disallowed);

    // Hole numbers follow creation order until the player moves a hole.
    auto* c = getGameState().golf.findCourse(course.courseId);
    for (size_t i = 0; i < ids.size(); i++)
        EXPECT_EQ(c->holeNumber(ids[i]), i + 1);
    CourseEditAction move(CourseEditType::moveHole, course.courseId, {}, 17, 0);
    EXPECT_EQ(run(move).error, Status::ok);
    c = getGameState().golf.findCourse(course.courseId);
    EXPECT_EQ(c->holeNumber(ids[17]), 1u);
    EXPECT_EQ(c->holeNumber(ids[0]), 2u);
    CourseEditAction badMove(CourseEditType::moveHole, course.courseId, {}, 18, 0);
    EXPECT_EQ(run(badMove).error, Status::invalidParameters);

    CourseEditAction remove(CourseEditType::removeHole, course.courseId, {}, ids[5]);
    EXPECT_EQ(run(remove).error, Status::ok);
    EXPECT_EQ(getGameState().golf.findCourse(course.courseId)->holes.size(), 17u);
    CourseEditAction again(CourseEditType::removeHole, course.courseId, {}, ids[5]);
    EXPECT_EQ(run(again).error, Status::invalidParameters);
}

TEST(CourseActions, TheActionsSerialiseForNetworkPlayAndReplays)
{
    HoleEditAction hole(3, 4, HoleEditType::placeAim, 1, { 640, 960 });
    MemoryStream stream;
    DataSerialiser writer(true, stream);
    hole.Serialise(writer);
    stream.SetPosition(0);
    HoleEditAction copy;
    DataSerialiser reader(false, stream);
    copy.Serialise(reader);
    MemoryStream again;
    DataSerialiser writer2(true, again);
    copy.Serialise(writer2);
    ASSERT_EQ(stream.GetLength(), again.GetLength());
    EXPECT_EQ(std::memcmp(stream.GetData(), again.GetData(), stream.GetLength()), 0);

    CourseEditAction course(CourseEditType::rename, 7, "Fairways", 12, 3);
    MemoryStream s1;
    DataSerialiser w1(true, s1);
    course.Serialise(w1);
    s1.SetPosition(0);
    CourseEditAction c2;
    DataSerialiser r1(false, s1);
    c2.Serialise(r1);
    MemoryStream s2;
    DataSerialiser w2(true, s2);
    c2.Serialise(w2);
    ASSERT_EQ(s1.GetLength(), s2.GetLength());
    EXPECT_EQ(std::memcmp(s1.GetData(), s2.GetData(), s1.GetLength()), 0);
}
