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
#include <openrct2/Context.h>
#include <openrct2/Diagnostic.h>
#include <openrct2/Game.h>
#include <openrct2/GameState.h>
#include <openrct2/GameStateSnapshots.h>
#include <openrct2/OpenRCT2.h>
#include <openrct2/ParkImporter.h>
#include <openrct2/audio/AudioContext.h>
#include <openrct2/core/MemoryStream.h>
#include <openrct2/core/String.hpp>
#include <openrct2/drawing/Drawing.h>
#include <openrct2/drawing/Palette.h>
#include <openrct2/entity/EntityRegistry.h>
#include <openrct2/entity/EntityTweener.h>
#include <openrct2/object/ObjectManager.h>
#include <openrct2/park/ParkFile.h>
#include <openrct2/ride/Ride.h>
#include <openrct2/scenario/Scenario.h>
#include <openrct2/world/MapAnimation.h>
#include <string>

using namespace OpenRCT2;

static bool LoadFileToBuffer(MemoryStream& stream, const std::string& filePath)
{
    FILE* fp = fopen(filePath.c_str(), "rb");
    EXPECT_NE(fp, nullptr);
    if (fp == nullptr)
        return false;

    uint8_t buf[1024];
    size_t bytesRead = fread(buf, 1, sizeof(buf), fp);
    while (bytesRead > 0)
    {
        stream.Write(buf, bytesRead);
        bytesRead = fread(buf, 1, sizeof(buf), fp);
    }
    fclose(fp);

    return true;
}

static void GameInit(bool retainSpatialIndices)
{
    auto& gameState = getGameState();
    if (!retainSpatialIndices)
        gameState.entities.resetEntitySpatialIndices();

    ResetAllSpriteQuadrantPlacements();
    Drawing::LoadPalette();
    EntityTweener::get().reset();
    MapAnimations::MarkAllTiles();
    FixInvalidVehicleSpriteSizes();

    gGameSpeed = 1;
}

static bool ImportPark(MemoryStream& stream, std::unique_ptr<IContext>& context, bool retainSpatialIndices)
{
    stream.SetPosition(0);

    auto& objManager = context->GetObjectManager();

    auto importer = ParkImporter::CreateParkFile(context->GetObjectRepository());
    auto loadResult = importer->LoadFromStream(&stream, false);
    objManager.LoadObjects(loadResult.RequiredObjects);

    // TODO: Have a separate GameState and exchange once loaded.
    auto& gameState = getGameState();
    importer->Import(gameState);

    GameInit(retainSpatialIndices);

    return true;
}

static bool ExportSave(MemoryStream& stream, std::unique_ptr<IContext>& context)
{
    auto& objManager = context->GetObjectManager();

    auto exporter = std::make_unique<ParkFileExporter>();
    exporter->ExportObjectsList = objManager.GetPackableObjects();

    auto& gameState = getGameState();
    exporter->Export(gameState, stream, kParkFileSaveCompressionLevel);

    return true;
}

static void RecordGameStateSnapshot(std::unique_ptr<IContext>& context, MemoryStream& snapshotStream)
{
    auto* snapshots = context->GetGameStateSnapshots();

    auto& snapshot = snapshots->CreateSnapshot();
    snapshots->Capture(snapshot);
    snapshots->LinkSnapshot(snapshot, getGameState().currentTicks, ScenarioRandState().s0);
    DataSerialiser snapShotDs(true, snapshotStream);
    snapshots->SerialiseSnapshot(snapshot, snapShotDs);
}

static void AdvanceGameTicks(uint32_t ticks, std::unique_ptr<IContext>& context)
{
    for (uint32_t i = 0; i < ticks; i++)
    {
        gameStateUpdateLogic();
    }
}

static void CompareStates(MemoryStream& importBuffer, MemoryStream& exportBuffer, MemoryStream& snapshotStream)
{
    if (importBuffer.GetLength() != exportBuffer.GetLength())
    {
        LOG_WARNING(
            "Inconsistent export size! Import Size: %llu bytes, Export Size: %llu bytes",
            static_cast<unsigned long long>(importBuffer.GetLength()),
            static_cast<unsigned long long>(exportBuffer.GetLength()));
    }

    std::unique_ptr<IContext> context = CreateContext();
    EXPECT_NE(context, nullptr);
    bool initialised = context->Initialise();
    ASSERT_TRUE(initialised);

    DataSerialiser ds(false, snapshotStream);
    IGameStateSnapshots* snapshots = GetContext()->GetGameStateSnapshots();

    GameStateSnapshot_t& importSnapshot = snapshots->CreateSnapshot();
    snapshots->SerialiseSnapshot(importSnapshot, ds);

    GameStateSnapshot_t& exportSnapshot = snapshots->CreateSnapshot();
    snapshots->SerialiseSnapshot(exportSnapshot, ds);

    try
    {
        GameStateCompareData cmpData = snapshots->Compare(importSnapshot, exportSnapshot);

        // Find out if there are any differences between the two states
        auto res = std::find_if(
            cmpData.spriteChanges.begin(), cmpData.spriteChanges.end(),
            [](const GameStateSpriteChange& diff) { return diff.changeType != GameStateSpriteChange::EQUAL; });

        if (res != cmpData.spriteChanges.end())
        {
            LOG_WARNING("Snapshot data differences. %s", snapshots->GetCompareDataText(cmpData).c_str());
            FAIL();
        }
    }
    catch (const std::runtime_error& err)
    {
        LOG_WARNING("Snapshot data failed to be read. Snapshot not compared. %s", err.what());
        FAIL();
    }
}

TEST(ParkImportExportBasic, all)
{
    gOpenRCT2Headless = true;
    gOpenRCT2NoGraphics = true;

    MemoryStream importBuffer;
    MemoryStream exportBuffer;
    MemoryStream snapshotStream;

    // Load initial park data.
    {
        std::unique_ptr<IContext> context = CreateContext();
        EXPECT_NE(context, nullptr);

        bool initialised = context->Initialise();
        ASSERT_TRUE(initialised);

        std::string testParkPath = TestData::GetParkPath("BigMapTest.park");
        ASSERT_TRUE(LoadFileToBuffer(importBuffer, testParkPath));
        ASSERT_TRUE(ImportPark(importBuffer, context, false));
        RecordGameStateSnapshot(context, snapshotStream);

        ASSERT_TRUE(ExportSave(exportBuffer, context));
    }

    // Import the exported version.
    {
        std::unique_ptr<IContext> context = CreateContext();
        EXPECT_NE(context, nullptr);

        bool initialised = context->Initialise();
        ASSERT_TRUE(initialised);

        ASSERT_TRUE(ImportPark(exportBuffer, context, true));

        RecordGameStateSnapshot(context, snapshotStream);
    }

    snapshotStream.SetPosition(0);
    CompareStates(importBuffer, exportBuffer, snapshotStream);

    SUCCEED();
}

TEST(ParkImportExportAdvanceTicks, all)
{
    gOpenRCT2Headless = true;
    gOpenRCT2NoGraphics = true;

    MemoryStream importBuffer;
    MemoryStream exportBuffer;
    MemoryStream snapshotStream;

    // Load initial park data.
    {
        std::unique_ptr<IContext> context = CreateContext();
        EXPECT_NE(context, nullptr);

        bool initialised = context->Initialise();
        ASSERT_TRUE(initialised);

        std::string testParkPath = TestData::GetParkPath("BigMapTest.park");
        ASSERT_TRUE(LoadFileToBuffer(importBuffer, testParkPath));
        ASSERT_TRUE(ImportPark(importBuffer, context, false));
        AdvanceGameTicks(1000, context);
        ASSERT_TRUE(ExportSave(exportBuffer, context));

        RecordGameStateSnapshot(context, snapshotStream);
    }

    // Import the exported version.
    {
        std::unique_ptr<IContext> context = CreateContext();
        EXPECT_NE(context, nullptr);

        bool initialised = context->Initialise();
        ASSERT_TRUE(initialised);

        ASSERT_TRUE(ImportPark(exportBuffer, context, true));

        RecordGameStateSnapshot(context, snapshotStream);
    }

    snapshotStream.SetPosition(0);
    CompareStates(importBuffer, exportBuffer, snapshotStream);

    SUCCEED();
}
