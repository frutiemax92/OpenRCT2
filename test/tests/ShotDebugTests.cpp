/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "GolfTestWorld.h"
#include "TestData.h"

#include <gtest/gtest.h>
#include <memory>
#include <optional>
#include <openrct2/Context.h>
#include <openrct2/Game.h>
#include <openrct2/GameState.h>
#include <openrct2/OpenRCT2.h>
#include <openrct2/ParkImporter.h>
#include <openrct2/drawing/Drawing.h>
#include <openrct2/drawing/Palette.h>
#include <openrct2/entity/EntityList.h>
#include <openrct2/entity/EntityRegistry.h>
#include <openrct2/entity/EntityTweener.h>
#include <openrct2/entity/GolfBall.h>
#include <openrct2/golf/ShotDebug.h>
#include <openrct2/object/ObjectManager.h>
#include <openrct2/world/Map.h>
#include <openrct2/world/MapAnimation.h>
#include <openrct2/world/TileElementsView.h>
#include <openrct2/world/tile_element/SurfaceElement.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Golf;
using namespace OpenRCT2::Terrain;

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
        GetShotDebug().clear();
        return context;
    }

    // Makes a clear, flat, dry strip of `length` tiles along +x, three tiles wide, and returns its start.
    CoordsXY makeClearStrip(int32_t length)
    {
        const int32_t y0 = 10;
        for (int32_t ty = y0 - 1; ty <= y0 + 1; ty++)
            for (int32_t tx = 4; tx < 4 + length; tx++)
            {
                const CoordsXY tile{ tx * 32, ty * 32 };
                // Remove everything on top of the surface and make the ground flat, dry and at one height.
                for (;;)
                {
                    TileElement* extra = nullptr;
                    for (auto* element : TileElementsView<TileElement>(tile))
                        if (element->getType() != TileElementType::surface)
                        {
                            extra = element;
                            break;
                        }
                    if (extra == nullptr)
                        break;
                    TileElementRemove(extra);
                }
                auto* surface = MapGetSurfaceElementAt(tile);
                surface->setBaseZ(16);
                surface->setClearanceZ(16);
                surface->setTerrainShape(TerrainShape{ 0, 0, 0, 0 });
                surface->setWaterHeight(0);
            }
        return CoordsXY{ 4 * 32, y0 * 32 };
    }
} // namespace

TEST(ShotDebug, AimVectorsFollowTheMapAxes)
{
    const auto east = ShotDebug::aimVector(0);
    EXPECT_NEAR(east.x.toDouble(), 1.0, 1e-3);
    EXPECT_NEAR(east.y.toDouble(), 0.0, 1e-3);
    const auto south = ShotDebug::aimVector(90);
    EXPECT_NEAR(south.x.toDouble(), 0.0, 1e-3);
    EXPECT_NEAR(south.y.toDouble(), 1.0, 1e-3);
    const auto diag = ShotDebug::aimVector(45);
    EXPECT_NEAR(diag.x.toDouble(), 0.7071, 2e-3);
    EXPECT_NEAR(diag.y.toDouble(), 0.7071, 2e-3);
    EXPECT_NEAR(ShotDebug::aimVector(-90).y.toDouble(), -1.0, 1e-3);
}

TEST(ShotDebug, TheTeeMustBeOnLandOnTheMap)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& debug = GetShotDebug();
    EXPECT_FALSE(debug.fire()); // no tee yet
    EXPECT_FALSE(debug.setTee({ -64, 100 }));
    EXPECT_FALSE(debug.setTee({ 1000000, 100 }));
    EXPECT_FALSE(debug.tee().has_value());

    const auto strip = makeClearStrip(30);
    EXPECT_TRUE(debug.setTee({ strip.x + 16, strip.y + 16 }));
    ASSERT_TRUE(debug.tee().has_value());
    EXPECT_EQ(debug.tee()->z, Fixed::fromInt(16));

    // Water is not a tee.
    MapGetSurfaceElementAt(CoordsXY{ strip.x + 64, strip.y })->setWaterHeight(40);
    EXPECT_FALSE(debug.setTee({ strip.x + 64 + 16, strip.y + 16 }));
    EXPECT_TRUE(debug.tee().has_value()); // the old tee is kept
}

TEST(ShotDebug, APerfectShotCarriesTheClubDistanceAndIsRecorded)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& debug = GetShotDebug();
    const auto strip = makeClearStrip(30);
    ASSERT_TRUE(debug.setTee({ strip.x + 16, strip.y + 16 }));
    debug.settings = {};
    debug.settings.club = ClubId::iron7;
    debug.settings.directionDegrees = 0;
    ASSERT_TRUE(debug.fire());
    EXPECT_TRUE(debug.hasShot());
    EXPECT_FALSE(debug.finished());

    int count = 0;
    GolfBall* ball = nullptr;
    for (auto* b : EntityList<GolfBall>())
    {
        ball = b;
        count++;
    }
    ASSERT_EQ(count, 1);
    for (int t = 0; t < 4000 && !debug.finished(); t++)
        ball->update();
    ASSERT_TRUE(debug.finished());

    ShotRequest same;
    same.club = ClubId::iron7;
    const double wanted = shotDistance(same).toDouble();
    EXPECT_NEAR(debug.carry().toDouble() / wanted, 1.0, 0.03);
    EXPECT_GT(debug.distance(), debug.carry());          // it rolled on after landing
    EXPECT_GT(debug.maxHeight(), Fixed::fromInt(30));
    EXPECT_FALSE(debug.impacts().empty());
    EXPECT_GT(debug.impacts().front().normalSpeed, Fixed::fromInt(1));
    EXPECT_GT(debug.samples().size(), 50u);
    EXPECT_EQ(debug.samples().front().state, BallState::Teed);
    EXPECT_EQ(debug.state(), BallState::AtRest);
    EXPECT_EQ(debug.ticks(), debug.samples().size() - 1);
    // A perfect shot goes straight along the aim.
    EXPECT_NEAR(debug.samples().back().position.y.toDouble(), debug.tee()->y.toDouble(), 1.0);
    // The overlay data: lie of the resting ball.
    EXPECT_FALSE(debug.lieIsHazard());
    EXPECT_FALSE(debug.lieOnPath());
}

TEST(ShotDebug, FiringAgainReplacesTheTestBall)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& debug = GetShotDebug();
    const auto strip = makeClearStrip(30);
    ASSERT_TRUE(debug.setTee({ strip.x + 16, strip.y + 16 }));
    ASSERT_TRUE(debug.fire());
    ASSERT_TRUE(debug.fire());
    int count = 0;
    for ([[maybe_unused]] auto* b : EntityList<GolfBall>())
        count++;
    EXPECT_EQ(count, 1);
    debug.clear();
    count = 0;
    for ([[maybe_unused]] auto* b : EntityList<GolfBall>())
        count++;
    EXPECT_EQ(count, 0);
    EXPECT_FALSE(debug.hasShot());
    EXPECT_TRUE(debug.samples().empty());
}

TEST(ShotDebug, DirectionAndPowerChangeTheShot)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& debug = GetShotDebug();
    const auto strip = makeClearStrip(30);
    ASSERT_TRUE(debug.setTee({ strip.x + 16, strip.y + 16 }));

    auto run = [&](int degrees, Fixed power) {
        debug.settings = {};
        debug.settings.club = ClubId::pitchingWedge;
        debug.settings.directionDegrees = degrees;
        debug.settings.power = power;
        EXPECT_TRUE(debug.fire());
        GolfBall* ball = nullptr;
        for (auto* b : EntityList<GolfBall>())
            ball = b;
        for (int t = 0; t < 4000 && !debug.finished(); t++)
            ball->update();
        return debug.samples().back().position;
    };
    const auto full = run(0, Fixed::fromInt(1));
    const auto half = run(0, Fixed::ratio(1, 2));
    const auto angled = run(10, Fixed::fromInt(1));
    const double tx = debug.tee()->x.toDouble();
    EXPECT_GT(full.x.toDouble() - tx, 1.5 * (half.x.toDouble() - tx));
    EXPECT_GT(angled.y, full.y + Fixed::fromInt(20)); // turned towards +y
}

TEST(ShotDebug, ARandomShotMissesTheLineButAPerfectOneDoesNot)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto& debug = GetShotDebug();
    const auto strip = makeClearStrip(30);
    ASSERT_TRUE(debug.setTee({ strip.x + 16, strip.y + 16 }));
    debug.settings = {};
    debug.settings.club = ClubId::driver;
    debug.settings.skills.accuracy = 0;
    debug.settings.perfect = false;
    double worst = 0;
    for (int i = 0; i < 6; i++)
    {
        ASSERT_TRUE(debug.fire());
        GolfBall* ball = nullptr;
        for (auto* b : EntityList<GolfBall>())
            ball = b;
        for (int t = 0; t < 4000 && !debug.finished(); t++)
            ball->update();
        worst = std::max(worst, std::fabs(debug.samples().back().position.y.toDouble() - debug.tee()->y.toDouble()));
    }
    // Accuracy 0 with a driver strays by several tiles at least once in six shots.
    EXPECT_GT(worst, 8.0);
}
