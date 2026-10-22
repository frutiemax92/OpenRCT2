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
#include <openrct2/Game.h>
#include <openrct2/GameState.h>
#include <openrct2/OpenRCT2.h>
#include <openrct2/ParkImporter.h>
#include <openrct2/drawing/Drawing.h>
#include <openrct2/drawing/Palette.h>
#include <openrct2/entity/AnimalEntity.h>
#include <openrct2/entity/EntityRegistry.h>
#include <openrct2/entity/EntityTweener.h>
#include <openrct2/entity/GolfVehicleEntity.h>
#include <openrct2/entity/GolferEntity.h>
#include <openrct2/interface/Viewport.h>
#include <openrct2/object/ObjectManager.h>
#include <openrct2/paint/Paint.h>
#include <openrct2/world/Building.h>
#include <openrct2/world/MapAnimation.h>
#include <openrct2/world/TileElementsView.h>
#include <openrct2/world/tile_element/BuildingElement.h>

using namespace OpenRCT2;

class GolfPickingTests : public testing::Test
{
protected:
    void SetUp() override
    {
        gOpenRCT2Headless = true;
        gOpenRCT2NoGraphics = true;
        _context = CreateContext();
        ASSERT_TRUE(_context->Initialise());

        auto importer = ParkImporter::CreateParkFile(_context->GetObjectRepository());
        auto loadResult = importer->LoadSavedGame(TestData::GetParkPath("small_park_with_ferris_wheel.park").c_str(), false);
        _context->GetObjectManager().LoadObjects(loadResult.RequiredObjects);

        MapAnimations::ClearAll();
        auto& gameState = getGameState();
        importer->Import(gameState);
        gameState.entities.resetEntitySpatialIndices();
        ResetAllSpriteQuadrantPlacements();
        Drawing::LoadPalette();
        EntityTweener::get().reset();
        gGameSpeed = 1;
    }

    void TearDown() override
    {
        _context = nullptr;
    }

    std::unique_ptr<IContext> _context;
};

// VIEW-002: GolferEntity bounding box extents for screen culling & mouse picking
TEST_F(GolfPickingTests, GolferEntityExtents)
{
    auto* golfer = GolferEntity::create(1, { 640, 640, 16 });
    ASSERT_NE(golfer, nullptr);
    EXPECT_EQ(golfer->spriteData.width, 16);
    EXPECT_EQ(golfer->spriteData.heightMin, 28);
    EXPECT_EQ(golfer->spriteData.heightMax, 8);

    auto screenCoords = Translate3DTo2DWithZ(0, golfer->getLocation());
    auto spriteRect = ScreenRect(
        screenCoords - ScreenCoordsXY{ golfer->spriteData.width, golfer->spriteData.heightMin },
        screenCoords + ScreenCoordsXY{ golfer->spriteData.width, golfer->spriteData.heightMax });

    // The screen rect must cover the golfer's body above their feet (screenCoords.y)
    EXPECT_LT(spriteRect.getTop(), screenCoords.y);
    EXPECT_GT(spriteRect.getBottom(), screenCoords.y);
    EXPECT_EQ(spriteRect.getWidth(), 32);
    EXPECT_EQ(spriteRect.getHeight(), 36);

    golfer->remove();
}

// VIEW-002: GolfVehicleEntity bounding box extents for screen culling & mouse picking
TEST_F(GolfPickingTests, GolfVehicleEntityExtents)
{
    auto* vehicle = GolfVehicleEntity::create(GolfVehicleEntity::Kind::cart, 1, { 640, 640, 16 });
    ASSERT_NE(vehicle, nullptr);
    EXPECT_EQ(vehicle->spriteData.width, 24);
    EXPECT_EQ(vehicle->spriteData.heightMin, 20);
    EXPECT_EQ(vehicle->spriteData.heightMax, 12);

    auto screenCoords = Translate3DTo2DWithZ(0, vehicle->getLocation());
    auto spriteRect = ScreenRect(
        screenCoords - ScreenCoordsXY{ vehicle->spriteData.width, vehicle->spriteData.heightMin },
        screenCoords + ScreenCoordsXY{ vehicle->spriteData.width, vehicle->spriteData.heightMax });

    EXPECT_LT(spriteRect.getTop(), screenCoords.y);
    EXPECT_GT(spriteRect.getBottom(), screenCoords.y);
    EXPECT_EQ(spriteRect.getWidth(), 48);
    EXPECT_EQ(spriteRect.getHeight(), 32);

    vehicle->remove();
}

// VIEW-002: AnimalEntity bounding box extents for screen culling & mouse picking
TEST_F(GolfPickingTests, AnimalEntityExtents)
{
    auto* animal = AnimalEntity::create(1, 0, { 640, 640, 16 });
    ASSERT_NE(animal, nullptr);
    EXPECT_EQ(animal->spriteData.width, 14);
    EXPECT_EQ(animal->spriteData.heightMin, 16);
    EXPECT_EQ(animal->spriteData.heightMax, 6);

    auto screenCoords = Translate3DTo2DWithZ(0, animal->getLocation());
    auto spriteRect = ScreenRect(
        screenCoords - ScreenCoordsXY{ animal->spriteData.width, animal->spriteData.heightMin },
        screenCoords + ScreenCoordsXY{ animal->spriteData.width, animal->spriteData.heightMax });

    EXPECT_LT(spriteRect.getTop(), screenCoords.y);
    EXPECT_GT(spriteRect.getBottom(), screenCoords.y);
    EXPECT_EQ(spriteRect.getWidth(), 28);
    EXPECT_EQ(spriteRect.getHeight(), 22);

    animal->remove();
}

// VIEW-001: ViewportInteractionItem enum includes building
TEST_F(GolfPickingTests, ViewportInteractionBuildingEnumSupported)
{
    constexpr auto buildingItem = ViewportInteractionItem::building;
    EXPECT_EQ(buildingItem, ViewportInteractionItem::building);

    // Verify interaction items can include building in flags
    ViewportInteractionItems flags{ ViewportInteractionItem::entity, ViewportInteractionItem::building };
    EXPECT_TRUE(flags.has(ViewportInteractionItem::building));
    EXPECT_TRUE(flags.has(ViewportInteractionItem::entity));
}

// VIEW-001: Building::FindAt finds building elements on tiles for viewport interaction
TEST_F(GolfPickingTests, BuildingFindAtTileElement)
{
    const CoordsXY tile{ 640, 640 };
    EXPECT_EQ(Building::FindAt(tile), nullptr);

    auto* buildingElement = TileElementInsert<BuildingElement>({ tile.x, tile.y, 16 }, 0);
    ASSERT_NE(buildingElement, nullptr);
    buildingElement->setObjectIndex(0);

    const auto* found = Building::FindAt(tile);
    EXPECT_EQ(found, buildingElement);
    EXPECT_EQ(found->getType(), TileElementType::building);
}
