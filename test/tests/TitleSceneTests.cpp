/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

// ARC-002: the title screen no longer plays title sequences (RCT2 parks); it shows an empty map.

#include <gtest/gtest.h>
#include <openrct2/Context.h>
#include <openrct2/Game.h>
#include <openrct2/GameState.h>
#include <openrct2/OpenRCT2.h>
#include <openrct2/entity/EntityRegistry.h>
#include <openrct2/ride/RideManager.hpp>
#include <openrct2/scenes/SceneManager.h>
#include <openrct2/world/Map.h>

using namespace OpenRCT2;

class TitleScene : public testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        gOpenRCT2Headless = true;
        gOpenRCT2NoGraphics = true;
        _context = CreateContext();
        ASSERT_TRUE(_context->Initialise());
    }

    static void TearDownTestSuite()
    {
        _context = nullptr;
    }

    static std::unique_ptr<IContext> _context;
};

std::unique_ptr<IContext> TitleScene::_context;

TEST_F(TitleScene, LoadsAnEmptyMap)
{
    auto* sceneManager = GetContext()->GetSceneManager();
    auto* titleScene = sceneManager->getTitleScene();
    ASSERT_NE(nullptr, titleScene);

    sceneManager->setActiveScene(titleScene);

    EXPECT_EQ(LegacyScene::titleSequence, gLegacyScene);
    auto& gameState = getGameState();
    EXPECT_EQ(kDefaultMapSize, gameState.mapSize);
    EXPECT_TRUE(gCurrentLoadedPath.empty());
    EXPECT_EQ(0u, RideManager(gameState).size());
    EXPECT_EQ(0u, getGameState().entities.getEntityListCount(EntityType::guest));
}
