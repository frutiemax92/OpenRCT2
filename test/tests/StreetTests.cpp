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
#include <openrct2/actions/scenery/StreetPlaceAction.h>
#include <openrct2/actions/scenery/StreetRemoveAction.h>
#include <openrct2/actions/scenery/StreetRoadEntranceAction.h>
#include <openrct2/object/StreetObject.h>
#include <openrct2/drawing/Drawing.h>
#include <openrct2/drawing/Palette.h>
#include <openrct2/entity/EntityRegistry.h>
#include <openrct2/entity/EntityTweener.h>
#include <openrct2/object/ObjectManager.h>
#include <openrct2/object/BuildingObject.h>
#include <openrct2/world/Map.h>
#include <openrct2/world/MapAnimation.h>
#include <openrct2/world/Footpath.h>
#include <openrct2/golf/StreetNetwork.h>
#include <openrct2/world/Street.h>
#include <openrct2/world/tile_element/StreetElement.h>
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
} // namespace

TEST(Street, ElementFitsATileElementSlot)
{
    EXPECT_EQ(sizeof(StreetElement), sizeof(TileElement));
    EXPECT_EQ(static_cast<uint8_t>(TileElementType::street), 10);
}

TEST(Street, ElementStoresConnectionsFlowEntranceAndSlope)
{
    TileElement slot{};
    slot.clearAs(TileElementType::street);
    auto* street = slot.asStreet();
    ASSERT_NE(street, nullptr);
    EXPECT_EQ(slot.asStream(), nullptr);
    EXPECT_EQ(street->getConnections(), 0);
    EXPECT_FALSE(street->isRoadEntrance());
    EXPECT_EQ(street->getRise(), 0);

    street->setObjectIndex(300);
    street->setConnections(0xFF);
    EXPECT_EQ(street->getConnections(), 0b1111);
    street->setConnection(1, false);
    EXPECT_FALSE(street->hasConnection(1));
    EXPECT_TRUE(street->hasConnection(0));
    street->setConnection(1, true);
    EXPECT_EQ(street->getConnections(), 0b1111);

    // Flow, entrance and slope do not disturb each other.
    street->setFlowDirection(6);
    EXPECT_EQ(street->getFlowDirection(), 2);
    street->setRoadEntrance(true);
    EXPECT_TRUE(street->isRoadEntrance());
    EXPECT_EQ(street->getFlowDirection(), 2);
    street->setFlowDirection(3);
    EXPECT_TRUE(street->isRoadEntrance());
    street->setRoadEntrance(false);
    EXPECT_EQ(street->getFlowDirection(), 3);

    street->setSlope(3, 2);
    EXPECT_EQ(street->getRise(), 3);
    EXPECT_EQ(street->getRiseDirection(), 2);
    street->setSlope(9, 1); // clamped to the steepest level
    EXPECT_EQ(street->getRise(), StreetElement::kMaxRise);
    EXPECT_EQ(street->getObjectIndex(), 300);
    EXPECT_EQ(slot.getType(), TileElementType::street);
}

namespace
{
    // Lays a street like the place action will: connected to the streets next to it, which connect back. A ghost is
    // a preview and touches nobody.
    StreetElement* lay(const CoordsXY& tile, bool ghost = false)
    {
        const auto baseZ = MapGetSurfaceElementAt(tile)->getBaseZ();
        auto* street = TileElementInsert<StreetElement>({ tile.x, tile.y, baseZ }, 0b1111);
        street->setClearanceZ(baseZ);
        street->setGhost(ghost);
        if (!ghost)
        {
            street->setConnections(Street::AutoConnect(tile));
            Street::UpdateNeighbours(tile, true);
        }
        return street;
    }
} // namespace

TEST(Street, TilesConnectToTheirNeighboursBothWays)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    const auto found = findFlatBlock(3, 2);
    ASSERT_TRUE(found.has_value());
    const CoordsXY a = *found;
    const CoordsXY b{ a.x + 32, a.y };
    const CoordsXY c{ a.x + 64, a.y };

    EXPECT_EQ(Street::FindAt(a), nullptr);
    lay(a);
    ASSERT_NE(Street::FindAt(a), nullptr);
    EXPECT_EQ(Street::FindAt(a)->getConnections(), 0);

    lay(b);
    // a is at -x of b, so b connects through its -x edge (0), a through its +x edge (2).
    EXPECT_TRUE(Street::FindAt(b)->hasConnection(0));
    EXPECT_TRUE(Street::FindAt(a)->hasConnection(2));
    EXPECT_FALSE(Street::FindAt(b)->hasConnection(2));

    lay(c);
    EXPECT_TRUE(Street::FindAt(b)->hasConnection(2));
    EXPECT_EQ(Street::FindAt(c)->getConnections(), 0b0001);
    EXPECT_EQ(Street::FindAt(b)->getConnections(), 0b0101); // straight along x

    // Taking the middle tile away lets go of both ends.
    Street::UpdateNeighbours(b, false);
    EXPECT_FALSE(Street::FindAt(a)->hasConnection(2));
    EXPECT_FALSE(Street::FindAt(c)->hasConnection(0));
}

TEST(Street, TheNetworkIsBuiltFromTheMap)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    const auto found = findFlatBlock(3, 2);
    ASSERT_TRUE(found.has_value());
    const CoordsXY a = *found;
    const CoordsXY b{ a.x + 32, a.y };
    const CoordsXY c{ a.x + 64, a.y };
    lay(a);
    lay(b);
    lay(c)->setRoadEntrance(true);
    // A ghost is not part of the network.
    const CoordsXY d{ a.x, a.y + 32 };
    lay(d, true);

    Golf::StreetNetwork network;
    Street::BuildNetwork(network);
    const TileCoordsXY ta{ a.x / 32, a.y / 32 }, tb{ b.x / 32, b.y / 32 }, tc{ c.x / 32, c.y / 32 };
    const auto* tileA = network.tileAt(ta);
    ASSERT_NE(tileA, nullptr);
    EXPECT_EQ(tileA->connections, 0b0100);
    EXPECT_FALSE(tileA->oneWay);
    EXPECT_GT(tileA->speedLimitKph, 0);
    ASSERT_NE(network.tileAt(tc), nullptr);
    EXPECT_TRUE(network.tileAt(tc)->roadEntrance);
    EXPECT_FALSE(network.tileAt(tb)->roadEntrance);
    EXPECT_EQ(network.tileAt({ a.x / 32, a.y / 32 + 1 }), nullptr);
    // Nothing connects to a ghost: the ghost at +y of a does not show in a's neighbours.
    EXPECT_EQ(Street::GetNeighbourMask(a) & 0b0010, 0);
    EXPECT_NE(Street::FindAt(d), nullptr);

    // Cars can drive from one end to the other.
    EXPECT_TRUE(network.canDrive(ta, 2));
    EXPECT_FALSE(network.canDrive(ta, 0));
    const auto route = network.findRoute(ta, tc);
    EXPECT_EQ(route.size(), 3u);
}

namespace
{
    ObjectEntryIndex loadStreet(IContext& context, const char* id, const StreetObject** object = nullptr)
    {
        auto& objectManager = context.GetObjectManager();
        auto* loaded = objectManager.LoadObject(id);
        if (loaded == nullptr)
            return kObjectEntryIndexNull;
        if (object != nullptr)
            *object = static_cast<const StreetObject*>(loaded);
        return objectManager.GetLoadedObjectEntryIndex(loaded);
    }
} // namespace

TEST(Street, SlopeOfARampIsItsRiseAndDirection)
{
    using Terrain::TerrainShape;
    using Street::RoadSlope;
    // Shapes are {north, east, south, west}.
    EXPECT_EQ(Street::SlopeOf(TerrainShape{ 0, 0, 0, 0 }), (RoadSlope{ 0, 0 }));
    // The north and east corners are the +x edge: raised, the road rises towards +x.
    EXPECT_EQ(Street::SlopeOf(TerrainShape{ 2, 2, 0, 0 }), (RoadSlope{ 2, 2 }));
    // Raised west and north corners are the +y edge.
    EXPECT_EQ(Street::SlopeOf(TerrainShape{ 4, 0, 0, 4 }), (RoadSlope{ 4, 1 }));
    // South and west: the -x edge.
    EXPECT_EQ(Street::SlopeOf(TerrainShape{ 0, 0, 1, 1 }), (RoadSlope{ 1, 0 }));
    // South and east: the -y edge.
    EXPECT_EQ(Street::SlopeOf(TerrainShape{ 0, 3, 3, 0 }), (RoadSlope{ 3, 3 }));
    // A single raised corner, a saddle and a twisted tile are no road.
    EXPECT_FALSE(Street::SlopeOf(TerrainShape{ 1, 0, 0, 0 }).has_value());
    EXPECT_FALSE(Street::SlopeOf(TerrainShape{ 2, 0, 2, 0 }).has_value());
    EXPECT_FALSE(Street::SlopeOf(TerrainShape{ 2, 1, 0, 0 }).has_value());
}

TEST(StreetActions, PlacingAndRemovingAStreetConnectsAndDisconnectsTheNeighbours)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    gameState.cheats.sandboxMode = true;
    const StreetObject* road = nullptr;
    const auto index = loadStreet(*context, "golf.street.road", &road);
    if (road == nullptr)
        GTEST_SKIP() << "the street object is not installed";

    const auto found = findFlatBlock(3, 2);
    ASSERT_TRUE(found.has_value());
    const CoordsXY a = *found, b{ a.x + 32, a.y };

    GameActions::StreetPlaceAction placeA(a, index);
    const auto resA = GameActions::ExecuteNested(&placeA, gameState);
    ASSERT_EQ(resA.error, GameActions::Status::ok);
    EXPECT_EQ(resA.cost, road->BuildCost);
    ASSERT_NE(Street::FindAt(a), nullptr);
    EXPECT_EQ(Street::FindAt(a)->getObjectIndex(), index);
    EXPECT_EQ(Street::FindAt(a)->getRise(), 0);
    EXPECT_EQ(Street::FindAt(a)->getConnections(), 0);

    GameActions::StreetPlaceAction placeB(b, index);
    ASSERT_EQ(GameActions::ExecuteNested(&placeB, gameState).error, GameActions::Status::ok);
    EXPECT_TRUE(Street::FindAt(a)->hasConnection(2));
    EXPECT_TRUE(Street::FindAt(b)->hasConnection(0));

    // A tile holds one street.
    GameActions::StreetPlaceAction again(a, index);
    EXPECT_EQ(GameActions::Query(&again, gameState).error, GameActions::Status::itemAlreadyPlaced);

    GameActions::StreetRemoveAction remove(b);
    const auto removed = GameActions::ExecuteNested(&remove, gameState);
    ASSERT_EQ(removed.error, GameActions::Status::ok);
    EXPECT_EQ(removed.cost, road->RemoveCost);
    EXPECT_EQ(Street::FindAt(b), nullptr);
    EXPECT_FALSE(Street::FindAt(a)->hasConnection(2));

    GameActions::StreetRemoveAction missing(b);
    EXPECT_EQ(GameActions::Query(&missing, gameState).error, GameActions::Status::invalidParameters);
}

TEST(StreetActions, RejectsBadObjectsOffMapTilesAndBlockedGround)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    gameState.cheats.sandboxMode = true;
    const StreetObject* road = nullptr;
    const auto index = loadStreet(*context, "golf.street.road", &road);
    if (road == nullptr)
        GTEST_SKIP() << "the street object is not installed";
    const auto found = findFlatBlock(3, 2);
    ASSERT_TRUE(found.has_value());

    GameActions::StreetPlaceAction badObject(*found, 200);
    EXPECT_EQ(GameActions::Query(&badObject, gameState).error, GameActions::Status::invalidParameters);
    GameActions::StreetPlaceAction offMap({ -32, 0 }, index);
    EXPECT_EQ(GameActions::Query(&offMap, gameState).error, GameActions::Status::invalidParameters);

    // A path in the way blocks the road.
    {
        const auto baseZ = MapGetSurfaceElementAt(*found)->getBaseZ();
        auto* path = TileElementInsert<PathElement>({ found->x, found->y, baseZ }, 0b1111);
        path->setClearanceZ(baseZ + 16);
    }
    GameActions::StreetPlaceAction blocked(*found, index);
    EXPECT_EQ(GameActions::Query(&blocked, gameState).error, GameActions::Status::noClearance);
}

TEST(StreetActions, AGhostPreviewConnectsNothingAndLeavesTheNeighboursAlone)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    gameState.cheats.sandboxMode = true;
    const StreetObject* road = nullptr;
    const auto index = loadStreet(*context, "golf.street.road", &road);
    if (road == nullptr)
        GTEST_SKIP() << "the street object is not installed";
    const auto found = findFlatBlock(3, 2);
    ASSERT_TRUE(found.has_value());
    const CoordsXY a = *found, b{ a.x + 32, a.y };

    GameActions::StreetPlaceAction placeA(a, index);
    ASSERT_EQ(GameActions::ExecuteNested(&placeA, gameState).error, GameActions::Status::ok);

    GameActions::StreetPlaceAction ghost(b, index);
    ghost.SetFlags(GameActions::CommandFlag::ghost);
    ASSERT_EQ(GameActions::ExecuteNested(&ghost, gameState).error, GameActions::Status::ok);
    ASSERT_NE(Street::FindAt(b), nullptr);
    EXPECT_TRUE(Street::FindAt(b)->isGhost());
    EXPECT_EQ(Street::FindAt(b)->getConnections(), 0);
    EXPECT_FALSE(Street::FindAt(a)->hasConnection(2));

    // Removing the ghost leaves the street next to it as it was.
    GameActions::StreetRemoveAction remove(b);
    remove.SetFlags(GameActions::CommandFlag::ghost);
    ASSERT_EQ(GameActions::ExecuteNested(&remove, gameState).error, GameActions::Status::ok);
    EXPECT_EQ(Street::FindAt(b), nullptr);
    EXPECT_EQ(Street::FindAt(a)->getConnections(), 0);
}

TEST(StreetActions, RoadEntrancesAreOnStreetsAtTheEdgeOfTheMap)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& gameState = getGameState();
    gameState.cheats.sandboxMode = true;
    const StreetObject* road = nullptr;
    const auto index = loadStreet(*context, "golf.street.road", &road);
    if (road == nullptr)
        GTEST_SKIP() << "the street object is not installed";

    // A street at the edge ring of the map (tile 1) and one further in.
    const CoordsXY edge{ 1 * 32, 5 * 32 };
    const CoordsXY inner{ 5 * 32, 5 * 32 };
    ASSERT_NE(MapGetSurfaceElementAt(edge), nullptr);
    for (const auto& tile : { edge, inner })
    {
        const auto z = MapGetSurfaceElementAt(tile)->getBaseZ();
        // Flatten nothing: the tile must be bare ground that takes a road.
        GameActions::StreetPlaceAction place(tile, index);
        const auto result = GameActions::ExecuteNested(&place, gameState);
        if (result.error != GameActions::Status::ok)
            GTEST_SKIP() << "the test park has ground at " << tile.x << "," << tile.y << " that takes no road (z " << z << ")";
    }

    // Nothing to enter on a tile without a street.
    GameActions::StreetRoadEntranceAction nothing({ 3 * 32, 9 * 32 }, true);
    EXPECT_EQ(GameActions::Query(&nothing, gameState).error, GameActions::Status::invalidParameters);

    // Not at the edge.
    GameActions::StreetRoadEntranceAction tooFarIn(inner, true);
    EXPECT_EQ(GameActions::Query(&tooFarIn, gameState).error, GameActions::Status::disallowed);

    GameActions::StreetRoadEntranceAction atEdge(edge, true);
    ASSERT_EQ(GameActions::ExecuteNested(&atEdge, gameState).error, GameActions::Status::ok);
    EXPECT_TRUE(Street::FindAt(edge)->isRoadEntrance());

    // The network knows it.
    Golf::StreetNetwork network;
    Street::BuildNetwork(network);
    ASSERT_EQ(network.roadEntrances().size(), 1u);
    EXPECT_EQ(network.roadEntrances()[0], (TileCoordsXY{ 1, 5 }));

    GameActions::StreetRoadEntranceAction remove(edge, false);
    ASSERT_EQ(GameActions::ExecuteNested(&remove, gameState).error, GameActions::Status::ok);
    EXPECT_FALSE(Street::FindAt(edge)->isRoadEntrance());
    // Taking an entrance away works anywhere.
    GameActions::StreetRoadEntranceAction removeInner(inner, false);
    EXPECT_EQ(GameActions::Query(&removeInner, gameState).error, GameActions::Status::ok);
}
