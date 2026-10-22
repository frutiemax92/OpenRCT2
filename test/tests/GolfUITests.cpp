/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <algorithm>
#include <gtest/gtest.h>
#include <openrct2-ui/input/ShortcutIds.h>
#include <openrct2/Context.h>
#include <openrct2/GameState.h>
#include <openrct2/OpenRCT2.h>
#include <openrct2/entity/GolferEntity.h>
#include <openrct2/golf/GolfDirector.h>
#include <openrct2/golf/GolfRuntime.h>
#include <openrct2/interface/WindowClasses.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Ui;

class GolfUITests : public testing::Test
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

// UI-001: Verification of Golf WindowClasses for toolbar & window management
TEST_F(GolfUITests, GolfWindowClassesAreDistinctAndValid)
{
    EXPECT_EQ(EnumValue(WindowClass::course), 144);
    EXPECT_EQ(EnumValue(WindowClass::golfer), 145);
    EXPECT_EQ(EnumValue(WindowClass::holeTool), 146);
    EXPECT_EQ(EnumValue(WindowClass::streetTool), 147);
    EXPECT_EQ(EnumValue(WindowClass::carts), 148);
    EXPECT_EQ(EnumValue(WindowClass::building), 149);
    EXPECT_EQ(EnumValue(WindowClass::zones), 150);
    EXPECT_EQ(EnumValue(WindowClass::tournament), 151);
    EXPECT_EQ(EnumValue(WindowClass::buildingTool), 152);

    // Ensure all window classes have unique values
    std::set<uint8_t> golfClasses = {
        EnumValue(WindowClass::course),
        EnumValue(WindowClass::golfer),
        EnumValue(WindowClass::holeTool),
        EnumValue(WindowClass::streetTool),
        EnumValue(WindowClass::carts),
        EnumValue(WindowClass::building),
        EnumValue(WindowClass::zones),
        EnumValue(WindowClass::tournament),
        EnumValue(WindowClass::buildingTool),
    };
    EXPECT_EQ(golfClasses.size(), 9u);
}

// UI-002: Bottom-left HUD Live Golfer Count and Course Popularity Metrics
TEST_F(GolfUITests, LiveGolferCountAndPopularityHUDMetrics)
{
    auto& gameState = getGameState();

    // 1. Without GolfRuntime, HUD metrics fallback to entities and park rating
    gameState.golfRuntime = nullptr;
    gameState.park.rating = 750;

    auto getGolferCount = [](const GameState_t& gs) -> uint32_t {
        if (gs.golfRuntime != nullptr)
        {
            return static_cast<uint32_t>(gs.golfRuntime->director().simulation().golfers().size());
        }
        return gs.entities.getEntityListCount(EntityType::golfer);
    };

    auto getPopularity = [](const GameState_t& gs) -> uint16_t {
        if (gs.golfRuntime != nullptr)
        {
            return static_cast<uint16_t>(
                std::clamp<int32_t>(gs.golfRuntime->director().popularity() * 10, 0, 1000));
        }
        return gs.park.rating;
    };

    EXPECT_EQ(getGolferCount(gameState), 0u);
    EXPECT_EQ(getPopularity(gameState), 750);

    // 2. Initialize Golf sandbox
    Golf::SetupEmptyGolfSandbox(gameState);
    gameState.golfRuntime = std::make_shared<Golf::GolfRuntime>(gameState);
    ASSERT_NE(gameState.golfRuntime, nullptr);

    // Empty sandbox has 0 active golfers
    EXPECT_EQ(getGolferCount(gameState), 0u);

    // Director popularity is scaled by 10 (0..1000)
    int32_t rawPopularity = gameState.golfRuntime->director().popularity();
    EXPECT_EQ(getPopularity(gameState), static_cast<uint16_t>(std::clamp(rawPopularity * 10, 0, 1000)));

    // Ensure factor calculation for 114px rating bar works correctly
    uint16_t popularity = getPopularity(gameState);
    uint8_t factor = static_cast<uint8_t>(std::max(10, ((popularity / 4) * 263) / 256));
    int16_t barWidth = (factor * 114) / 255;
    EXPECT_GE(barWidth, 0);
    EXPECT_LE(barWidth, 114);

    // With a 75% rating (750 / 1000), bar is filled substantially
    uint8_t factor750 = static_cast<uint8_t>(std::max(10, ((750 / 4) * 263) / 256));
    int16_t barWidth750 = (factor750 * 114) / 255;
    EXPECT_GT(barWidth750, 50);
    EXPECT_LE(barWidth750, 114);
}

// UI-003: Keyboard Shortcuts for Golf Windows
TEST_F(GolfUITests, KeyboardShortcutIdsAreConfigured)
{
    // New golf shortcuts (UI-003)
    EXPECT_EQ(ShortcutId::kInterfaceOpenStreets, "interface.open.streets");
    EXPECT_EQ(ShortcutId::kInterfaceOpenZones, "interface.open.zones");
    EXPECT_EQ(ShortcutId::kInterfaceOpenTournaments, "interface.open.tournaments");
    EXPECT_EQ(ShortcutId::kInterfaceOpenCarts, "interface.open.carts");
    EXPECT_EQ(ShortcutId::kInterfaceOpenShotTool, "interface.open.shot_tool");

    // Standard hotkey mappings for golf equivalents
    EXPECT_EQ(ShortcutId::kInterfaceOpenRides, "interface.open.rides");
    EXPECT_EQ(ShortcutId::kInterfaceOpenNewRide, "interface.open.new_ride");
    EXPECT_EQ(ShortcutId::kInterfaceOpenGuests, "interface.open.guests");
}
