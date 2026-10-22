/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <gtest/gtest.h>
#include <openrct2-ui/interface/ViewportInteraction.h>
#include <openrct2/Context.h>
#include <openrct2/GameState.h>
#include <openrct2/OpenRCT2.h>
#include <openrct2/drawing/Drawing.h>
#include <openrct2/drawing/Palette.h>
#include <openrct2/entity/EntityList.h>
#include <openrct2/entity/GolferEntity.h>
#include <openrct2/golf/GolfRuntime.h>
#include <openrct2/interface/Viewport.h>
#include <openrct2/world/Map.h>
#include <openrct2/world/MapAnimation.h>
#include <openrct2/world/tile_element/PathElement.h>
#include <openrct2/world/tile_element/SurfaceElement.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Ui;

class GolfGameplayFlowTests : public testing::Test
{
protected:
    void SetUp() override
    {
        gOpenRCT2Headless = true;
        gOpenRCT2NoGraphics = true;
        _context = CreateContext();
        ASSERT_TRUE(_context->Initialise());
        Drawing::LoadPalette();
        MapAnimations::ClearAll();
    }

    void TearDown() override
    {
        _context = nullptr;
    }

    std::unique_ptr<IContext> _context;
};

// TEST-001 & TEST-002: Demo course setup creates all required infrastructure
TEST_F(GolfGameplayFlowTests, DemoCourseBuilderCreatesValidTilesAndInfrastructure)
{
    auto& gameState = getGameState();
    Golf::SetupDemoGolfCourse(gameState);
    Golf::updateGolf(gameState);
    auto& runtime = Golf::runtimeOf(gameState);

    EXPECT_TRUE(runtime.buildings().hasClub);
    EXPECT_GE(gameState.golf.courses().size(), 2u);
    EXPECT_GT(runtime.traffic().capacity(), 0);
    EXPECT_FALSE(runtime.traffic().lots().empty());
    EXPECT_TRUE(runtime.hasCarTraffic());
    EXPECT_GE(gameState.golf.lots().size(), 1u);
}

// TEST-001 & TEST-002: Ticking simulation updates golfers and entities
TEST_F(GolfGameplayFlowTests, GolfRuntimeTicksProgressGolfersAndEntities)
{
    auto& gameState = getGameState();
    Golf::SetupDemoGolfCourse(gameState);
    auto& runtime = Golf::runtimeOf(gameState);

    runtime.spawnGolferGroup(0);
    EXPECT_GE(runtime.director().simulation().golfers().size(), 3u);

    for (int32_t i = 0; i < 50; i++)
    {
        gameState.currentTicks++;
        Golf::updateGolf(gameState);
    }

    size_t golferEntitiesCount = 0;
    for (auto* entity : EntityList<GolferEntity>())
    {
        (void)entity;
        golferEntitiesCount++;
    }
    EXPECT_GT(golferEntitiesCount, 0u);
}

// TEST-001 & TEST-002: Interactive sandbox actions trigger correctly
TEST_F(GolfGameplayFlowTests, TestingSandboxActionsExecuteSuccessfully)
{
    auto& gameState = getGameState();
    Golf::SetupDemoGolfCourse(gameState);
    auto& runtime = Golf::runtimeOf(gameState);

    // 1. Driving range customer
    runtime.spawnDrivingRangeCustomer();
    bool foundRangeCustomer = false;
    for (const auto& g : runtime.director().simulation().golfers())
    {
        if (g.name == "Range Customer" || g.state == Golf::GolferState::walkingToRange)
        {
            foundRangeCustomer = true;
            break;
        }
    }
    EXPECT_TRUE(foundRangeCustomer);

    // 2. VIP cottage evaluation
    EXPECT_FALSE(gameState.golf.lots().empty());
    runtime.evaluateVipCottages();
    EXPECT_TRUE(gameState.golf.lots().front().sold);

    // 3. Tournament start & finish
    runtime.startOrAdvanceTournament(0);
    EXPECT_EQ(runtime.director().tournament.phase, Golf::TournamentPhase::running);
    runtime.startOrAdvanceTournament(0);
    EXPECT_EQ(runtime.director().tournament.phase, Golf::TournamentPhase::finished);
    EXPECT_GT(runtime.director().tournament.popularityGained, 0);

    // 4. Weather & wind override
    runtime.setWeatherAndWind(Weather::Type::rain, 90, 500);
    EXPECT_EQ(gameState.weatherCurrent.weatherType, Weather::Type::rain);
    EXPECT_EQ(gameState.wind.direction, 90);
    EXPECT_EQ(gameState.wind.strength, 500);

    // 5. Overlays toggle
    EXPECT_FALSE(runtime.holeOverlayEnabled());
    runtime.toggleHoleOverlay();
    EXPECT_TRUE(runtime.holeOverlayEnabled());
    runtime.toggleHoleOverlay();
    EXPECT_FALSE(runtime.holeOverlayEnabled());

    EXPECT_FALSE(runtime.turfOverlayEnabled());
    runtime.toggleTurfOverlay();
    EXPECT_TRUE(runtime.turfOverlayEnabled());
    runtime.toggleTurfOverlay();
    EXPECT_FALSE(runtime.turfOverlayEnabled());

    // 6. Wildlife spawning
    runtime.spawnWildlife(0);
    EXPECT_FALSE(runtime.director().animals->animals().empty());
}

// TEST-002 & VIEW-001: Viewport interaction preserves building and golfer items
TEST_F(GolfGameplayFlowTests, ViewportInteractionPreservesBuildingAndGolferItems)
{
    // The paint picking drops every item the filter helper rejects: a building used to be rejected because it sits after
    // the banner in the enum, so clicking one never reached the building window.
    EXPECT_TRUE(IsPickableInteractionItem(ViewportInteractionItem::building));
    EXPECT_TRUE(IsPickableInteractionItem(ViewportInteractionItem::entity));
    EXPECT_TRUE(IsPickableInteractionItem(ViewportInteractionItem::terrain));
    EXPECT_TRUE(IsPickableInteractionItem(ViewportInteractionItem::banner));
    EXPECT_FALSE(IsPickableInteractionItem(ViewportInteractionItem::none));
    EXPECT_FALSE(IsPickableInteractionItem(ViewportInteractionItem::label));

    auto& gameState = getGameState();
    Golf::SetupDemoGolfCourse(gameState);
    auto& runtime = Golf::runtimeOf(gameState);
    runtime.spawnGolferGroup(0);

    size_t count = 0;
    for (auto* entity : EntityList<GolferEntity>())
    {
        EXPECT_EQ(entity->type, EntityType::golfer);
        count++;
    }
    EXPECT_GT(count, 0u);
}

// Golfers must arrive on their own soon after a new demo game starts, without the sandbox spawn buttons.
TEST_F(GolfGameplayFlowTests, DemoCourseAdmitsGolfersSoonAfterStartWithoutManualSpawning)
{
    auto& gameState = getGameState();
    Golf::SetupDemoGolfCourse(gameState);
    auto& runtime = Golf::runtimeOf(gameState);

    // Three game hours from the start of a new game.
    size_t mostGolfers = 0;
    const uint32_t start = gameState.currentTicks;
    for (uint32_t tick = start; tick < start + static_cast<uint32_t>(Golf::hhmm(3, 0) * Golf::kTicksPerGameMinute); tick++)
    {
        gameState.currentTicks = tick;
        Golf::updateGolf(gameState);
        mostGolfers = std::max(mostGolfers, runtime.director().simulation().golfers().size());
    }

    EXPECT_GT(mostGolfers, 0u);
}

// The footpaths of the demo course join up, so that a golfer can follow them from one hole to the next.
TEST_F(GolfGameplayFlowTests, DemoCourseFootpathsAreJoinedAlongTheHoles)
{
    auto& gameState = getGameState();
    Golf::SetupDemoGolfCourse(gameState);

    auto edgesAt = [](int32_t x, int32_t y) -> int {
        auto* surface = MapGetSurfaceElementAt(TileCoordsXY{ x, y });
        auto* path = surface == nullptr ? nullptr : MapGetPathElementAt(TileCoordsXYZ{ x, y, surface->getBaseZ() / kCoordsZStep });
        return path == nullptr ? -1 : path->getEdges();
    };
    // Straight stretch beside the first fairway: joined east and west only.
    EXPECT_EQ(edgesAt(50, 35), 0b0101);
    EXPECT_NE(edgesAt(114, 60), -1);
    EXPECT_NE(edgesAt(60, 106), -1);
}
