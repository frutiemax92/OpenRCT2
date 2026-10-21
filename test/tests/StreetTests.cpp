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
