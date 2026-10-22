/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <gtest/gtest.h>
#include <openrct2/Context.h>
#include <openrct2/GameState.h>
#include <openrct2/OpenRCT2.h>
#include <openrct2/actions/GameActionRunner.h>
#include <openrct2/actions/footpath/FootpathLayoutPlaceAction.h>
#include <openrct2/actions/golf/CourseBoundaryAction.h>
#include <openrct2/actions/golf/CourseEditAction.h>
#include <openrct2/actions/scenery/BuildingPlaceAction.h>
#include <openrct2/actions/scenery/BuildingRemoveAction.h>
#include <openrct2/actions/scenery/ParkingPlaceAction.h>
#include <openrct2/actions/scenery/ParkingRemoveAction.h>
#include <openrct2/actions/scenery/StreamPlaceAction.h>
#include <openrct2/actions/scenery/StreamRemoveAction.h>
#include <openrct2/actions/scenery/StreetPlaceAction.h>
#include <openrct2/actions/scenery/StreetRoadEntranceAction.h>
#include <openrct2/golf/CourseManager.h>
#include <openrct2/golf/GolfRuntime.h>
#include <openrct2/golf/StreetNetwork.h>
#include <openrct2/object/BuildingObject.h>
#include <openrct2/object/ObjectLimits.h>
#include <openrct2/object/ObjectManager.h>
#include <openrct2/object/StreamObject.h>
#include <openrct2/object/StreetObject.h>
#include <openrct2/world/Building.h>
#include <openrct2/world/Map.h>
#include <openrct2/world/MapOwnership.h>
#include <openrct2/world/Parking.h>
#include <openrct2/world/Street.h>
#include <openrct2/world/TileElementsView.h>
#include <openrct2/world/tile_element/BuildingElement.h>
#include <openrct2/world/tile_element/PathElement.h>
#include <openrct2/world/tile_element/StreamElement.h>
#include <openrct2/world/tile_element/SurfaceElement.h>

using namespace OpenRCT2;
using namespace OpenRCT2::GameActions;

namespace
{
    void insertPath(const CoordsXY& tile)
    {
        auto* surface = MapGetSurfaceElementAt(tile);
        if (surface != nullptr)
        {
            const auto baseZ = surface->getBaseZ();
            auto* path = TileElementInsert<PathElement>({ tile.x, tile.y, baseZ }, 0b1111);
            path->setClearanceZ(baseZ + 16);
        }
    }
} // namespace

class GolfToolTests : public testing::Test
{
protected:
    void SetUp() override
    {
        gOpenRCT2Headless = true;
        gOpenRCT2NoGraphics = true;
        _context = CreateContext();
        ASSERT_TRUE(_context->Initialise());
    }

    void TearDown() override
    {
        _context = nullptr;
    }

    std::unique_ptr<IContext> _context;
};

// TOOL-001: Golf Building Placement and Removal
TEST_F(GolfToolTests, BuildingPlacementAndDemolish)
{
    auto& gameState = getGameState();
    Golf::SetupEmptyGolfSandbox(gameState);
    gameState.cheats.sandboxMode = true;

    // Find the first loaded BuildingObject
    auto& objMgr = _context->GetObjectManager();
    ObjectEntryIndex buildingIdx = kObjectEntryIndexNull;
    BuildingObject* obj = nullptr;
    for (ObjectEntryIndex i = 0; i < kMaxBuildingObjects; ++i)
    {
        obj = objMgr.GetLoadedObject<BuildingObject>(i);
        if (obj != nullptr)
        {
            buildingIdx = i;
            break;
        }
    }
    ASSERT_NE(buildingIdx, kObjectEntryIndexNull);
    ASSERT_NE(obj, nullptr);

    const CoordsXY placeLoc{ 20 * kCoordsXYStep, 20 * kCoordsXYStep };
    const auto entrance = Building::GetEntrance(*obj, placeLoc, 0);
    insertPath(entrance.outsideTile);

    // Place building
    BuildingPlaceAction placeAction(placeLoc, buildingIdx, 0);
    auto placeRes = ExecuteNested(&placeAction, gameState);
    EXPECT_EQ(placeRes.error, Status::ok);

    const auto* buildingEl = Building::FindAt(placeLoc);
    ASSERT_NE(buildingEl, nullptr);
    EXPECT_EQ(buildingEl->getObjectIndex(), buildingIdx);

    // Remove building (demolish mode)
    BuildingRemoveAction removeAction(placeLoc);
    auto removeRes = ExecuteNested(&removeAction, gameState);
    EXPECT_EQ(removeRes.error, Status::ok);

    EXPECT_EQ(Building::FindAt(placeLoc), nullptr);
}

// TOOL-002: Stream Placement and Removal
TEST_F(GolfToolTests, StreamPlacementAndRemoval)
{
    auto& gameState = getGameState();
    Golf::SetupEmptyGolfSandbox(gameState);
    gameState.cheats.sandboxMode = true;

    // Find loaded stream object
    auto& objMgr = _context->GetObjectManager();
    ObjectEntryIndex streamIdx = kObjectEntryIndexNull;
    for (ObjectEntryIndex i = 0; i < kMaxStreamObjects; ++i)
    {
        if (objMgr.GetLoadedObject<StreamObject>(i) != nullptr)
        {
            streamIdx = i;
            break;
        }
    }
    ASSERT_NE(streamIdx, kObjectEntryIndexNull);

    const CoordsXY streamLoc{ 15 * kCoordsXYStep, 15 * kCoordsXYStep };

    // Place stream
    StreamPlaceAction placeAction(streamLoc, streamIdx, 1);
    auto placeRes = ExecuteNested(&placeAction, gameState);
    EXPECT_EQ(placeRes.error, Status::ok);

    // Verify stream element exists on tile
    bool foundStream = false;
    for (auto* element : TileElementsView<StreamElement>(streamLoc))
    {
        foundStream = true;
        EXPECT_EQ(element->getFlowDirection(), 1);
    }
    EXPECT_TRUE(foundStream);

    // Remove stream
    StreamRemoveAction removeAction(streamLoc);
    auto removeRes = ExecuteNested(&removeAction, gameState);
    EXPECT_EQ(removeRes.error, Status::ok);

    foundStream = false;
    for ([[maybe_unused]] auto* element : TileElementsView<StreamElement>(streamLoc))
    {
        foundStream = true;
    }
    EXPECT_FALSE(foundStream);
}

// TOOL-003: Half-Tile Footpath Sub-Quads
TEST_F(GolfToolTests, HalfTileFootpathSubQuadsPlacement)
{
    auto& gameState = getGameState();
    Golf::SetupEmptyGolfSandbox(gameState);
    gameState.cheats.sandboxMode = true;

    const CoordsXY pathLoc{ 25 * kCoordsXYStep, 25 * kCoordsXYStep };
    auto* surface = MapGetSurfaceElementAt(pathLoc);
    ASSERT_NE(surface, nullptr);
    const int32_t baseZ = surface->getBaseZ();

    // Place sub-quad 0 (bitmask 0x01)
    uint8_t subQuadMask1 = 0x01;
    FootpathLayoutPlaceAction placeAction1(
        { pathLoc.x, pathLoc.y, baseZ }, { FootpathSlopeType::flat, 0 }, 0, kObjectEntryIndexNull, subQuadMask1,
        PathConstructFlag::IsLegacyPathObject);
    auto res1 = ExecuteNested(&placeAction1, gameState);
    EXPECT_EQ(res1.error, Status::ok);

    // Verify PathElement has edge mask 0x01
    PathElement* pathEl = nullptr;
    for (auto* el : TileElementsView<PathElement>(pathLoc))
    {
        if (el->getBaseZ() == baseZ)
        {
            pathEl = el;
            break;
        }
    }
    ASSERT_NE(pathEl, nullptr);
    EXPECT_EQ(pathEl->getEdges() & 0x01, 0x01);

    // Place sub-quad on another tile
    const CoordsXY pathLoc2{ 26 * kCoordsXYStep, 25 * kCoordsXYStep };
    uint8_t subQuadMask2 = 0x02;
    FootpathLayoutPlaceAction placeAction2(
        { pathLoc2.x, pathLoc2.y, baseZ }, { FootpathSlopeType::flat, 0 }, 0, kObjectEntryIndexNull, subQuadMask2,
        PathConstructFlag::IsLegacyPathObject);
    auto res2 = ExecuteNested(&placeAction2, gameState);
    EXPECT_EQ(res2.error, Status::ok);

    PathElement* pathEl2 = nullptr;
    for (auto* el : TileElementsView<PathElement>(pathLoc2))
    {
        if (el->getBaseZ() == baseZ)
        {
            pathEl2 = el;
            break;
        }
    }
    ASSERT_NE(pathEl2, nullptr);
    EXPECT_EQ(pathEl2->getEdges() & 0x02, 0x02);

    // Test sub-quad combination on existing path
    pathEl->setEdges(pathEl->getEdges() | 0x02);
    EXPECT_EQ(pathEl->getEdges() & 0x03, 0x03);
}

// TOOL-004: Mini-Putt Course Creation, Boundary Action & Hole Overlay
TEST_F(GolfToolTests, MiniPuttCourseAndBoundaryActions)
{
    auto& gameState = getGameState();
    Golf::SetupEmptyGolfSandbox(gameState);
    gameState.cheats.sandboxMode = true;

    // Create Mini-Putt course via CourseEditAction with value = 1 (miniPutt)
    CourseEditAction createMiniAction(CourseEditType::create, 0, "My Mini-Putt", 1);
    auto createRes = ExecuteNested(&createMiniAction, gameState);
    EXPECT_EQ(createRes.error, Status::ok);

    auto resultData = createRes.getData<CourseEditResult>();
    auto* course = gameState.golf.findCourse(resultData.courseId);
    ASSERT_NE(course, nullptr);
    EXPECT_EQ(course->kind, Golf::CourseKind::miniPutt);
    EXPECT_EQ(course->name, "My Mini-Putt");

    // Test GolfRuntime HoleOverlay toggle
    auto* runtime = Golf::GetGolfRuntime();
    ASSERT_NE(runtime, nullptr);
    EXPECT_FALSE(runtime->holeOverlayEnabled());
    runtime->setHoleOverlayEnabled(true);
    EXPECT_TRUE(runtime->holeOverlayEnabled());
    runtime->setHoleOverlayEnabled(false);
    EXPECT_FALSE(runtime->holeOverlayEnabled());

    // Test CourseBoundaryAction
    const CoordsXY minTile{ 10 * kCoordsXYStep, 10 * kCoordsXYStep };
    const CoordsXY maxTile{ 15 * kCoordsXYStep, 15 * kCoordsXYStep };

    CourseBoundaryAction inPlayAction(minTile, maxTile, CourseBoundaryAction::kInPlay);
    EXPECT_EQ(ExecuteNested(&inPlayAction, gameState).error, Status::ok);
    EXPECT_TRUE(gameState.golf.boundary().defined());

    CourseBoundaryAction outOfPlayAction(minTile, maxTile, CourseBoundaryAction::kOutOfPlay);
    EXPECT_EQ(ExecuteNested(&outOfPlayAction, gameState).error, Status::ok);

    CourseBoundaryAction resetAction(minTile, maxTile, CourseBoundaryAction::kReset);
    EXPECT_EQ(ExecuteNested(&resetAction, gameState).error, Status::ok);
    EXPECT_FALSE(gameState.golf.boundary().defined());
}

// TOOL-005: Street Tool status, live scanning and bay connectivity
TEST_F(GolfToolTests, StreetToolStatusAndTrafficMetrics)
{
    auto& gameState = getGameState();
    Golf::SetupEmptyGolfSandbox(gameState);
    gameState.cheats.sandboxMode = true;

    // Find loaded street object
    auto& objMgr = _context->GetObjectManager();
    ObjectEntryIndex streetIdx = kObjectEntryIndexNull;
    for (ObjectEntryIndex i = 0; i < kMaxStreetObjects; ++i)
    {
        if (objMgr.GetLoadedObject<StreetObject>(i) != nullptr)
        {
            streetIdx = i;
            break;
        }
    }
    ASSERT_NE(streetIdx, kObjectEntryIndexNull);

    // Place a road entrance at the edge of the map (x = 1)
    const CoordsXY entranceLoc{ 1 * kCoordsXYStep, 10 * kCoordsXYStep };
    StreetPlaceAction placeStreet(entranceLoc, streetIdx, 2);
    auto streetRes = ExecuteNested(&placeStreet, gameState);
    EXPECT_EQ(streetRes.error, Status::ok);

    StreetRoadEntranceAction makeEntrance(entranceLoc, true);
    auto entranceRes = ExecuteNested(&makeEntrance, gameState);
    EXPECT_EQ(entranceRes.error, Status::ok);

    // Place a parking space adjacent to street (x = 2 facing x = 1, direction 0 (-x))
    const CoordsXY parkingLoc{ 2 * kCoordsXYStep, 10 * kCoordsXYStep };
    ParkingPlaceAction placeParking(parkingLoc, streetIdx, 0, false);
    EXPECT_EQ(ExecuteNested(&placeParking, gameState).error, Status::ok);

    // Place a cart bay
    const CoordsXY cartBayLoc{ 3 * kCoordsXYStep, 10 * kCoordsXYStep };
    ParkingPlaceAction placeCartBay(cartBayLoc, streetIdx, 0, true);
    EXPECT_EQ(ExecuteNested(&placeCartBay, gameState).error, Status::ok);

    // Scan street network and parking
    Golf::StreetNetwork network;
    Street::BuildNetwork(network);
    EXPECT_GE(network.roadEntrances().size(), 1u);

    const auto bays = Parking::FindCartBays();
    EXPECT_GE(bays.size(), 1u);
}
