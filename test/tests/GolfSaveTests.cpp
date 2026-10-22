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
#include <openrct2/golf/GolfRuntime.h>
#include <openrct2/object/ObjectManager.h>
#include <openrct2/park/ParkFile.h>
#include <openrct2/world/MapAnimation.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Golf;

TEST(GolfSave, GolfersSurviveASaveAndLoadOfThePark)
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

        gameState.golf.clear();
        auto* course = gameState.golf.createCourse("Links");
        course->open = true;

        auto& runtime = runtimeOf(gameState);
        GolfSimulation::Spawn spawn;
        spawn.name = "Saved Sam";
        spawn.courseId = course->id;
        spawn.skills = GolferSkills{};
        auto& golfer = runtime.director().simulation().spawn(spawn);
        golfer.scorecard = { 1, -1, 0 };
        golfer.thoughts.push_back(Thought{ ThoughtType::enjoyingTheRound, 77 });
        // The spots where a golfer stands aside while a group mate plays: a golfer that loses them walks to the origin.
        golfer.hasTurnSpot = true;
        golfer.turnSpot = { Fixed::fromInt(1500), Fixed::fromInt(2100), Fixed::fromInt(8) };
        golfer.queueSpot = { Fixed::fromInt(1700), Fixed::fromInt(2300), Fixed::fromInt(8) };
        runtime.director().setPopularity(812);

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
        importer->Import(gameState);

        // The golfers wait in the game state until the runtime is built.
        EXPECT_EQ(gameState.golfRuntime, nullptr);
        EXPECT_FALSE(gameState.golfSimulationData.empty());
        auto& runtime = runtimeOf(gameState);
        EXPECT_TRUE(gameState.golfSimulationData.empty());
        ASSERT_EQ(runtime.director().simulation().golfers().size(), 1u);
        const auto& golfer = runtime.director().simulation().golfers()[0];
        EXPECT_EQ(golfer.name, "Saved Sam");
        EXPECT_EQ(golfer.scorecard, (std::vector<int8_t>{ 1, -1, 0 }));
        ASSERT_EQ(golfer.thoughts.size(), 1u);
        EXPECT_EQ(golfer.thoughts[0].tick, 77u);
        EXPECT_TRUE(golfer.hasTurnSpot);
        EXPECT_EQ(golfer.turnSpot.x, Fixed::fromInt(1500));
        EXPECT_EQ(golfer.turnSpot.y, Fixed::fromInt(2100));
        EXPECT_EQ(golfer.queueSpot.x, Fixed::fromInt(1700));
        EXPECT_EQ(golfer.queueSpot.y, Fixed::fromInt(2300));
        EXPECT_EQ(runtime.director().popularity(), 812);
    }
}
