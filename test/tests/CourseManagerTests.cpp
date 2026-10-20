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
#include <openrct2/Context.h>
#include <openrct2/Game.h>
#include <openrct2/GameState.h>
#include <openrct2/OpenRCT2.h>
#include <openrct2/ParkImporter.h>
#include <openrct2/core/MemoryStream.h>
#include <openrct2/drawing/Drawing.h>
#include <openrct2/drawing/Palette.h>
#include <openrct2/entity/EntityRegistry.h>
#include <openrct2/entity/EntityTweener.h>
#include <openrct2/golf/CourseManager.h>
#include <openrct2/object/ObjectManager.h>
#include <openrct2/park/ParkFile.h>
#include <openrct2/world/MapAnimation.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Golf;

TEST(CourseManager, CreatesCoursesWithUniqueIds)
{
    CourseManager m;
    auto* a = m.createCourse("Front nine");
    ASSERT_NE(a, nullptr);
    const auto idA = a->id;
    auto* b = m.createCourse("Back nine");
    ASSERT_NE(b, nullptr);
    const auto idB = b->id; // the pointers die when the vector changes
    EXPECT_NE(idA, idB);
    EXPECT_EQ(m.courses().size(), 2u);
    a = m.findCourse(idA);
    EXPECT_EQ(a->name, "Front nine");
    EXPECT_EQ(m.findCourse(404), nullptr);
    EXPECT_FALSE(m.findCourse(idA)->open); // courses start closed

    EXPECT_TRUE(m.removeCourse(idA));
    EXPECT_FALSE(m.removeCourse(idA));
    EXPECT_EQ(m.findCourse(idA), nullptr);
    // Ids are not reused.
    EXPECT_GT(m.createCourse("Third")->id, idB);
}

TEST(CourseManager, ACapOnTheNumberOfCourses)
{
    CourseManager m;
    for (size_t i = 0; i < CourseManager::kMaxCourses; i++)
        EXPECT_NE(m.createCourse("c"), nullptr);
    EXPECT_EQ(m.createCourse("one too many"), nullptr);
}

TEST(CourseManager, HolesAreAppendedInCreationOrderUpToEighteen)
{
    CourseManager m;
    const auto courseId = m.createCourse("Links")->id;
    uint32_t last = 0;
    for (size_t i = 0; i < kMaxHolesPerCourse; i++)
    {
        auto* hole = m.createHole(courseId);
        ASSERT_NE(hole, nullptr);
        EXPECT_GT(hole->id, last);
        last = hole->id;
    }
    EXPECT_EQ(m.createHole(courseId), nullptr);
    EXPECT_EQ(m.createHole(999), nullptr);
    const auto* course = m.findCourse(courseId);
    EXPECT_EQ(course->holes.size(), 18u);
    EXPECT_EQ(course->holeNumber(course->holes[0].id), 1u);
    EXPECT_EQ(course->holeNumber(course->holes[17].id), 18u);
}

TEST(CourseManager, HolesCanBeRemovedAndFoundAcrossCourses)
{
    CourseManager m;
    const auto a = m.createCourse("A")->id;
    const auto b = m.createCourse("B")->id;
    const auto holeA = m.createHole(a)->id;
    const auto holeB = m.createHole(b)->id;
    const auto ref = m.findHole(holeB);
    ASSERT_NE(ref.hole, nullptr);
    EXPECT_EQ(ref.course->id, b);
    EXPECT_EQ(m.findHole(12345).hole, nullptr);

    EXPECT_FALSE(m.removeHole(a, holeB)); // it is not in course A
    EXPECT_TRUE(m.removeHole(a, holeA));
    EXPECT_EQ(m.findHole(holeA).hole, nullptr);
    EXPECT_FALSE(m.removeHole(a, holeA));
    EXPECT_FALSE(m.removeHole(77, holeB));
}

TEST(CourseBoundary, IsUndefinedUntilARegionIsMarked)
{
    CourseBoundary boundary;
    EXPECT_FALSE(boundary.defined());
    EXPECT_TRUE(boundary.isInPlay({ 5, 5 })); // the caller applies the owned-land default
    const TileCoordsXY size{ 20, 10 };
    boundary.setRegion({ 2, 2 }, { 5, 4 }, true, size, false);
    EXPECT_TRUE(boundary.defined());
    EXPECT_TRUE(boundary.isInPlay({ 2, 2 }));
    EXPECT_TRUE(boundary.isInPlay({ 5, 4 }));
    EXPECT_FALSE(boundary.isInPlay({ 6, 4 }));
    EXPECT_FALSE(boundary.isInPlay({ 0, 0 }));
    EXPECT_FALSE(boundary.isInPlay({ -1, 3 }));
    EXPECT_FALSE(boundary.isInPlay({ 20, 3 }));
    EXPECT_FALSE(boundary.isInPlay({ 3, 10 }));
}

TEST(CourseBoundary, StartsFromTheDefaultStateAndRegionsCanBeRemoved)
{
    CourseBoundary boundary;
    const TileCoordsXY size{ 8, 8 };
    // Starting from "everything in play" and cutting a hole out of it.
    boundary.setRegion({ 3, 3 }, { 4, 4 }, false, size, true);
    EXPECT_TRUE(boundary.isInPlay({ 0, 0 }));
    EXPECT_FALSE(boundary.isInPlay({ 3, 3 }));
    EXPECT_TRUE(boundary.isInPlay({ 5, 5 }));
    // Corners may be given in any order and are clamped to the map.
    boundary.setRegion({ 20, 20 }, { 6, 6 }, false, size, true);
    EXPECT_FALSE(boundary.isInPlay({ 7, 7 }));
    EXPECT_TRUE(boundary.isInPlay({ 5, 6 }));
    boundary.clear();
    EXPECT_FALSE(boundary.defined());
    EXPECT_TRUE(boundary.bits().empty());
}

TEST(CourseBoundary, RestoreRejectsInconsistentData)
{
    CourseBoundary boundary;
    boundary.restore({ 4, 4 }, std::vector<uint8_t>(16, 1));
    EXPECT_TRUE(boundary.defined());
    EXPECT_TRUE(boundary.isInPlay({ 3, 3 }));
    boundary.restore({ 4, 4 }, std::vector<uint8_t>(15, 1));
    EXPECT_FALSE(boundary.defined());
    boundary.restore({ 0, 4 }, {});
    EXPECT_FALSE(boundary.defined());
}

TEST(CourseManager, CoursesAndTheBoundarySurviveASaveAndLoadOfThePark)
{
    gOpenRCT2Headless = true;
    gOpenRCT2NoGraphics = true;
    MemoryStream saved;
    {
        auto context = CreateContext();
        ASSERT_TRUE(context->Initialise());
        auto importer = ParkImporter::CreateParkFile(context->GetObjectRepository());
        auto loadResult = importer->LoadSavedGame(TestData::GetParkPath("small_park_with_ferris_wheel.park").c_str(), false);
        context->GetObjectManager().LoadObjects(loadResult.RequiredObjects);
        auto& gameState = getGameState();
        importer->Import(gameState);

        auto& golf = gameState.golf;
        golf.clear();
        auto* course = golf.createCourse("Test links");
        course->greenFee = 12340;
        course->open = true;
        const auto courseId = course->id;
        auto* hole = golf.createHole(courseId);
        hole->tees[0] = HolePoint{ 320, 640 };
        hole->tees[2] = HolePoint{ 352, 640 };
        hole->cup = HolePoint{ 3200, 640 };
        hole->aimPoints[0] = HolePoint{ 1600, 900 };
        hole->parOverride = 5;
        golf.createHole(courseId); // an empty second hole
        golf.createCourse("Second");
        golf.boundary().setRegion({ 3, 3 }, { 9, 12 }, true, gameState.mapSize, false);

        auto exporter = std::make_unique<ParkFileExporter>();
        exporter->ExportObjectsList = context->GetObjectManager().GetPackableObjects();
        exporter->Export(gameState, saved, kParkFileSaveCompressionLevel);
    }
    {
        auto context = CreateContext();
        ASSERT_TRUE(context->Initialise());
        saved.SetPosition(0);
        auto importer = ParkImporter::CreateParkFile(context->GetObjectRepository());
        auto loadResult = importer->LoadFromStream(&saved, false);
        context->GetObjectManager().LoadObjects(loadResult.RequiredObjects);
        auto& gameState = getGameState();
        gameState.golf.createCourse("stale"); // loading replaces whatever was there
        importer->Import(gameState);

        const auto& golf = gameState.golf;
        ASSERT_EQ(golf.courses().size(), 2u);
        const auto& course = golf.courses()[0];
        EXPECT_EQ(course.name, "Test links");
        EXPECT_EQ(course.greenFee, 12340);
        EXPECT_TRUE(course.open);
        ASSERT_EQ(course.holes.size(), 2u);
        const auto& hole = course.holes[0];
        EXPECT_EQ(hole.tees[0], (HolePoint{ 320, 640 }));
        EXPECT_FALSE(hole.tees[1].has_value());
        EXPECT_EQ(hole.tees[2], (HolePoint{ 352, 640 }));
        EXPECT_EQ(hole.cup, (HolePoint{ 3200, 640 }));
        EXPECT_EQ(hole.aimPoints[0], (HolePoint{ 1600, 900 }));
        EXPECT_FALSE(hole.aimPoints[1].has_value());
        EXPECT_EQ(hole.parOverride, 5);
        EXPECT_FALSE(course.holes[1].isComplete());
        EXPECT_EQ(golf.courses()[1].name, "Second");
        // New ids carry on after the loaded ones.
        EXPECT_GT(golf.nextCourseId(), golf.courses()[1].id);
        EXPECT_GT(golf.nextHoleId(), course.holes[1].id);

        EXPECT_TRUE(golf.boundary().defined());
        EXPECT_TRUE(golf.boundary().isInPlay({ 5, 5 }));
        EXPECT_FALSE(golf.boundary().isInPlay({ 2, 5 }));
        EXPECT_FALSE(golf.boundary().isInPlay({ 5, 13 }));
    }
}

TEST(CourseManager, AParkWithoutCoursesLoadsAsEmpty)
{
    gOpenRCT2Headless = true;
    gOpenRCT2NoGraphics = true;
    auto context = CreateContext();
    ASSERT_TRUE(context->Initialise());
    getGameState().golf.createCourse("left over from another park");
    auto importer = ParkImporter::CreateParkFile(context->GetObjectRepository());
    auto loadResult = importer->LoadSavedGame(TestData::GetParkPath("small_park_with_ferris_wheel.park").c_str(), false);
    context->GetObjectManager().LoadObjects(loadResult.RequiredObjects);
    importer->Import(getGameState());
    EXPECT_TRUE(getGameState().golf.courses().empty());
    EXPECT_FALSE(getGameState().golf.boundary().defined());
}
