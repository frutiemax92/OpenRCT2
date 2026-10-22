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
#include <openrct2/actions/GameActionRunner.h>
#include <openrct2/actions/peep/StaffHireNewAction.h>
#include <openrct2/drawing/Drawing.h>
#include <openrct2/drawing/Palette.h>
#include <openrct2/entity/EntityRegistry.h>
#include <openrct2/entity/EntityTweener.h>
#include <openrct2/entity/Staff.h>
#include <openrct2/golf/Clubs.h>
#include <openrct2/golf/GolfDirector.h>
#include <openrct2/golf/GolfRuntime.h>
#include <openrct2/golf/Golfer.h>
#include <openrct2/object/ObjectManager.h>
#include <openrct2/world/MapAnimation.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Golf;
using namespace OpenRCT2::GameActions;

class GolfMonitoringTests : public testing::Test
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

// MON-001: Golfer skill tiers, state names, to-par scores, and satisfaction
TEST_F(GolfMonitoringTests, GolferDataFieldsAndSkillTiers)
{
    Golfer golfer;
    golfer.id = 101;
    golfer.name = "Tiger Woods";
    golfer.state = GolferState::swinging;
    EXPECT_STREQ(golferStateName(golfer.state), "Swinging");

    // VIP tier
    golfer.vip = true;
    EXPECT_TRUE(golfer.vip);

    // Score vs par
    golfer.scorecard = { 0, -1, 1, 0 }; // Even, Birdie, Bogey, Even -> Even
    EXPECT_EQ(golfer.toParSoFar(), 0);

    // Satisfaction calculation from happinessByte
    golfer.needs.happiness = 200 * 16;
    uint8_t satisfactionPct = static_cast<uint8_t>((golfer.needs.happinessByte() * 100) / 255);
    EXPECT_EQ(satisfactionPct, 78);
}

// MON-002: Golfer coordinates, active club, and needs access
TEST_F(GolfMonitoringTests, GolferPositionAndActiveClub)
{
    Golfer golfer;
    golfer.position = { Fixed::fromInt(128), Fixed::fromInt(256), Fixed::fromInt(32) };
    EXPECT_EQ(golfer.position.x.floor(), 128);
    EXPECT_EQ(golfer.position.y.floor(), 256);
    EXPECT_EQ(golfer.position.z.floor(), 32);

    golfer.decision.club = ClubId::driver;
    EXPECT_STREQ(clubInfo(golfer.decision.club).name, "Driver");

    golfer.decision.club = ClubId::putter;
    EXPECT_STREQ(clubInfo(golfer.decision.club).name, "Putter");
}

// MON-003: Greenskeeper / Staff hiring auto-positions without park entrances
TEST_F(GolfMonitoringTests, StaffHiringAutoPositionFallbackWithoutParkEntrance)
{
    auto& gameState = getGameState();
    // Clear any park entrances so park.entrances is empty
    gameState.park.entrances.clear();
    ASSERT_TRUE(gameState.park.entrances.empty());

    // Hire a handyman / greenskeeper
    auto hireAction = GameActions::StaffHireNewAction(true, StaffType::handyman, 0, STAFF_ORDERS_MOWING);
    auto res = GameActions::ExecuteNested(&hireAction, gameState);
    ASSERT_EQ(res.error, GameActions::Status::ok);

    auto result = res.getData<GameActions::StaffHireNewActionResult>();
    auto* staff = gameState.entities.getEntity<Staff>(result.StaffEntityId);
    ASSERT_NE(staff, nullptr);

    // Staff must be placed in the world, not stuck in picked state with null location
    EXPECT_NE(staff->x, kLocationNull);
    EXPECT_NE(staff->y, kLocationNull);
    EXPECT_EQ(staff->state, PeepState::falling);
}

// MON-003: Turf condition overlay toggle
TEST_F(GolfMonitoringTests, TurfOverlayToggle)
{
    auto* runtime = Golf::GetGolfRuntime();
    ASSERT_NE(runtime, nullptr);

    EXPECT_FALSE(runtime->turfOverlayEnabled());
    runtime->setTurfOverlayEnabled(true);
    EXPECT_TRUE(runtime->turfOverlayEnabled());
    runtime->setTurfOverlayEnabled(false);
    EXPECT_FALSE(runtime->turfOverlayEnabled());
}
