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
#include <openrct2/Context.h>
#include <openrct2/Game.h>
#include <openrct2/GameState.h>
#include <openrct2/GameStateSnapshots.h>
#include <openrct2/OpenRCT2.h>
#include <openrct2/ParkImporter.h>
#include <openrct2/core/DataSerialiser.h>
#include <openrct2/core/MemoryStream.h>
#include <openrct2/drawing/Drawing.h>
#include <openrct2/drawing/ImageImporter.h>
#include <openrct2/drawing/GolfBallSprites.h>
#include <openrct2/drawing/Palette.h>
#include <openrct2/entity/EntityList.h>
#include <openrct2/entity/EntityRegistry.h>
#include <openrct2/entity/EntityTweener.h>
#include <openrct2/entity/GolfBall.h>
#include <openrct2/object/ObjectManager.h>
#include <openrct2/park/ParkFile.h>
#include <openrct2/world/Map.h>
#include <openrct2/world/MapAnimation.h>
#include <openrct2/world/tile_element/SurfaceElement.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Golf;
using namespace GolfTest;

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
        gameState.mapSize = { 60, 60 }; // the test park is only 15 tiles wide
        return context;
    }

    FixedVec3 groundAt(int x, int y)
    {
        const auto* surface = MapGetSurfaceElementAt(CoordsXY{ x, y });
        return vec(x, y, surface->getBaseZ());
    }
} // namespace

TEST(GolfBallEntity, IsCreatedAtRestOnTheGround)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    const auto at = groundAt(200, 200);
    auto* ball = GolfBall::create(at, BallState::Teed, 7);
    ASSERT_NE(ball, nullptr);
    EXPECT_EQ(ball->type, EntityType::golfBall);
    EXPECT_TRUE(ball->is<GolfBall>());
    EXPECT_EQ(ball->x, 200);
    EXPECT_EQ(ball->y, 200);
    EXPECT_EQ(ball->z, at.z.floor());
    EXPECT_EQ(ball->ball.state, BallState::Teed);
    EXPECT_EQ(ball->ball.owner, 7u);
    EXPECT_FALSE(ball->isMoving());

    int count = 0;
    for ([[maybe_unused]] auto* b : EntityList<GolfBall>())
        count++;
    EXPECT_EQ(count, 1);
    // Balls are misc entities, like the ducks and balloons.
    EXPECT_GE(getGameState().entities.getMiscEntityCount(), 1);
}

TEST(GolfBallEntity, ATeedBallDoesNotMoveWhenUpdated)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto* ball = GolfBall::create(groundAt(200, 200), BallState::Teed, 1);
    ASSERT_NE(ball, nullptr);
    for (int i = 0; i < 20; i++)
        ball->update();
    EXPECT_EQ(ball->x, 200);
    EXPECT_EQ(ball->ball.state, BallState::Teed);
}

TEST(GolfBallEntity, AHitBallFliesLandsAndComesToRestOnTheMap)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto* ball = GolfBall::create(groundAt(100, 400), BallState::Teed, 1);
    ASSERT_NE(ball, nullptr);

    class FixedRandom final : public IBallRandom
    {
    public:
        uint32_t next() override
        {
            return 0x80000000u;
        }
    } rng;
    ShotRequest request;
    request.club = ClubId::iron7;
    request.skills.accuracy = 100;
    ball->hit(computeShot(request, rng));
    EXPECT_EQ(ball->ball.state, BallState::InFlight);
    EXPECT_TRUE(ball->isMoving());

    bool wasAirborne = false;
    int ticks = 0;
    for (; ticks < 6000 && !isSettled(ball->ball.state); ticks++)
    {
        ball->update();
        wasAirborne |= ball->ball.state == BallState::InFlight;
    }
    EXPECT_TRUE(wasAirborne);
    EXPECT_TRUE(isSettled(ball->ball.state)) << "ticks " << ticks;
    EXPECT_NE(ball->ball.state, BallState::Teed);
    // The entity position follows the simulation.
    EXPECT_EQ(ball->x, ball->ball.position.x.floor());
    EXPECT_EQ(ball->y, ball->ball.position.y.floor());
    EXPECT_EQ(ball->z, ball->ball.position.z.floor());
}

TEST(GolfBallEntity, ATrailFollowsABallInFlightAndClearsWhenItRolls)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto* ball = GolfBall::create(groundAt(100, 400), BallState::InFlight, 1);
    ASSERT_NE(ball, nullptr);
    ball->ball.position.z += Fixed::fromInt(3);
    ball->ball.velocity = vec(6, 0, 4);
    for (int i = 0; i < 4; i++)
        ball->update();
    EXPECT_EQ(ball->trailCount, 4);
    EXPECT_NE(ball->trail[0].x, ball->x);
    for (int i = 0; i < 30; i++)
        ball->update();
    EXPECT_EQ(ball->trailCount, GolfBall::kTrailLength);

    // Put the ball back on the ground rolling: the trail is only for balls in the air.
    ball->ball.position.z = Fixed::fromInt(MapGetSurfaceElementAt(CoordsXY{ ball->x, ball->y })->getBaseZ());
    ball->ball.state = BallState::Rolling;
    ball->ball.velocity = vec(1, 0, 0);
    ball->update();
    EXPECT_EQ(ball->ball.state, BallState::Rolling);
    EXPECT_EQ(ball->trailCount, 0);
}

TEST(GolfBallEntity, AnAbandonedBallDisappearsButAnOwnedOneStays)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto* abandoned = GolfBall::create(groundAt(100, 400), BallState::AtRest, 0);
    auto* owned = GolfBall::create(groundAt(100, 440), BallState::AtRest, 3);
    ASSERT_NE(abandoned, nullptr);
    ASSERT_NE(owned, nullptr);
    const auto abandonedId = abandoned->id;
    for (uint32_t i = 0; i <= GolfBall::kAbandonedBallTicks + 1; i++)
    {
        if (getGameState().entities.getEntity(abandonedId) != nullptr)
            abandoned->update();
        owned->update();
    }
    EXPECT_EQ(getGameState().entities.getEntity<GolfBall>(abandonedId), nullptr);
    EXPECT_NE(getGameState().entities.getEntity<GolfBall>(owned->id), nullptr);
}

TEST(GolfBallEntity, SerialisesEveryFieldOfTheSimulation)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto* ball = GolfBall::create(groundAt(120, 130), BallState::InFlight, 9);
    ASSERT_NE(ball, nullptr);
    ball->ball.velocity = { Fixed::fromRaw(123457), Fixed::fromRaw(-9876), Fixed::fromRaw(555) };
    ball->ball.sideSpin = Fixed::fromRaw(-777);
    ball->ball.backSpin = Fixed::fromRaw(888);
    ball->ball.hazardEntry = vec(1, 2, 3);
    ball->ball.shotOrigin = vec(4, 5, 6);
    ball->ball.passingObstacle = 44;
    ball->ball.lippedOut = true;
    ball->ball.bounces = 5;
    ball->ball.lieSurface = 3;
    ball->restingTicks = 17;

    MemoryStream stream;
    DataSerialiser writer(true, stream);
    ball->serialise(writer);

    stream.SetPosition(0);
    GolfBall copy{};
    DataSerialiser reader(false, stream);
    copy.serialise(reader);
    EXPECT_EQ(copy.ball.position, ball->ball.position);
    EXPECT_EQ(copy.ball.velocity, ball->ball.velocity);
    EXPECT_EQ(copy.ball.sideSpin, ball->ball.sideSpin);
    EXPECT_EQ(copy.ball.backSpin, ball->ball.backSpin);
    EXPECT_EQ(copy.ball.state, BallState::InFlight);
    EXPECT_EQ(copy.ball.owner, 9u);
    EXPECT_EQ(copy.ball.hazardEntry, ball->ball.hazardEntry);
    EXPECT_EQ(copy.ball.shotOrigin, ball->ball.shotOrigin);
    EXPECT_EQ(copy.ball.passingObstacle, 44u);
    EXPECT_TRUE(copy.ball.lippedOut);
    EXPECT_EQ(copy.ball.bounces, 5);
    EXPECT_EQ(copy.ball.lieSurface, 3);
    EXPECT_EQ(copy.restingTicks, 17u);
}

TEST(GolfBallEntity, SurvivesASaveAndLoadOfThePark)
{
    MemoryStream saved;
    uint16_t id = 0;
    {
        auto context = startPark();
        ASSERT_NE(context.get(), nullptr);
        auto* ball = GolfBall::create(groundAt(150, 160), BallState::Rolling, 5);
        ASSERT_NE(ball, nullptr);
        ball->ball.velocity = { Fixed::fromRaw(3333), Fixed::fromRaw(-1111), Fixed{} };
        ball->ball.bounces = 2;
        id = ball->id.ToUnderlying();
        auto& wind = getGameState().wind;
        wind.direction = 123;
        wind.strength = 456;
        wind.targetDirection = 200;
        wind.targetStrength = 789;
        wind.randomState = 0xABCDEF01u;

        auto exporter = std::make_unique<ParkFileExporter>();
        exporter->ExportObjectsList = context->GetObjectManager().GetPackableObjects();
        exporter->Export(getGameState(), saved, kParkFileSaveCompressionLevel);
    }
    {
        auto context = CreateContext();
        ASSERT_TRUE(context->Initialise());
        saved.SetPosition(0);
        auto importer = ParkImporter::CreateParkFile(context->GetObjectRepository());
        auto loadResult = importer->LoadFromStream(&saved, false);
        context->GetObjectManager().LoadObjects(loadResult.RequiredObjects);
        importer->Import(getGameState());
        getGameState().entities.resetEntitySpatialIndices();

        auto* loaded = getGameState().entities.getEntity<GolfBall>(EntityId::FromUnderlying(id));
        ASSERT_NE(loaded, nullptr);
        EXPECT_EQ(loaded->ball.owner, 5u);
        EXPECT_EQ(loaded->ball.state, BallState::Rolling);
        EXPECT_EQ(loaded->ball.velocity.x.raw(), 3333);
        EXPECT_EQ(loaded->ball.velocity.y.raw(), -1111);
        EXPECT_EQ(loaded->ball.bounces, 2);
        EXPECT_EQ(loaded->x, 150);
        EXPECT_EQ(loaded->trailCount, 0);
        // The wind is saved with the climate (BAL-031).
        const auto& wind = getGameState().wind;
        EXPECT_EQ(wind.direction, 123);
        EXPECT_EQ(wind.strength, 456);
        EXPECT_EQ(wind.targetDirection, 200);
        EXPECT_EQ(wind.targetStrength, 789);
        EXPECT_EQ(wind.randomState, 0xABCDEF01u);
    }
}

TEST(GolfBallEntity, ParksFromBeforeGolfBallsStillLoad)
{
    auto context = startPark(); // the test park predates the golf ball chunk
    ASSERT_NE(context.get(), nullptr);
    int count = 0;
    for ([[maybe_unused]] auto* b : EntityList<GolfBall>())
        count++;
    EXPECT_EQ(count, 0);
    EXPECT_GE(kParkFileCurrentVersion, kGolfBallEntityVersion);
}

TEST(GolfBallSprites, TheBallIsTwoByTwoPixelsOfTheWhitestColour)
{
    const auto white = Drawing::FindWhiteIndex();
    EXPECT_NE(white, 0);
    EXPECT_GE(StandardPalette[white].red, 200);
    EXPECT_GE(StandardPalette[white].green, 200);
    EXPECT_GE(StandardPalette[white].blue, 200);
    const auto pixels = Drawing::MakeGolfBallPixels(white);
    EXPECT_EQ(pixels.size(), 4u);
    for (auto p : pixels)
        EXPECT_EQ(p, white);
}

TEST(GolfBallSprites, TheShadowIsAFlatEllipseMask)
{
    const auto shadow = Drawing::MakeGolfBallShadowPixels();
    ASSERT_EQ(shadow.size(), static_cast<size_t>(Drawing::GolfBallSprites::kShadowWidth * Drawing::GolfBallSprites::kShadowHeight));
    size_t covered = 0;
    for (auto p : shadow)
        covered += p != 0 ? 1 : 0;
    EXPECT_GT(covered, 10u);
    EXPECT_LT(covered, shadow.size());
    EXPECT_EQ(shadow[0], 0); // the corners are empty
    EXPECT_EQ(shadow[Drawing::GolfBallSprites::kShadowWidth], 1);
}

TEST(GolfBallSprites, TheSpritesAreRegisteredAsImages)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    // Images are not allocated without graphics, which the other tests ask for.
    gOpenRCT2NoGraphics = false;
    Drawing::GolfBallSprites::Release();
    const auto& sprites = Drawing::GolfBallSprites::Get();
    EXPECT_NE(sprites.ball, kImageIndexUndefined);
    EXPECT_NE(sprites.shadow, kImageIndexUndefined);
    EXPECT_NE(sprites.trail, kImageIndexUndefined);
    EXPECT_NE(sprites.ball, sprites.shadow);
    // Asking again returns the same images.
    EXPECT_EQ(Drawing::GolfBallSprites::Get().ball, sprites.ball);
    Drawing::GolfBallSprites::Release();
    gOpenRCT2NoGraphics = true;
}

TEST(GolfBallEntity, EntityTypeMatchesGolfBall)
{
    auto context = startPark();
    ASSERT_NE(context.get(), nullptr);
    auto* ball = GolfBall::create({ Fixed::fromInt(100), Fixed::fromInt(100), Fixed::fromInt(16) }, BallState::Teed, 1);
    ASSERT_NE(ball, nullptr);
    EXPECT_EQ(ball->type, EntityType::golfBall);
}
