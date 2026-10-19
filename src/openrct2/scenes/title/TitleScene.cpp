/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "TitleScene.h"

#include "../../Context.h"
#include "../../Diagnostic.h"
#include "../../Game.h"
#include "../../GameState.h"
#include "../../Input.h"
#include "../../OpenRCT2.h"
#include "../../audio/Audio.h"
#include "../../config/Config.h"
#include "../../drawing/Drawing.Screen.h"
#include "../../drawing/Palette.h"
#include "../../interface/Screenshot.h"
#include "../../interface/Window.h"
#include "../../network/NetworkBase.h"
#include "../../object/DefaultObjects.h"
#include "../../object/ObjectManager.h"
#include "../../scenes/SceneManager.h"
#include "../../ui/UiContext.h"
#include "../../ui/WindowManager.h"
#include "../../world/Map.h"

using namespace OpenRCT2;

void TitleScene::Load()
{
    LOG_VERBOSE("TitleScene::Load()");

    if (GameIsPaused())
    {
        PauseToggle();
    }

    gLegacyScene = LegacyScene::titleSequence;
    gScreenAge = 0;
    gCurrentLoadedPath.clear();

#ifndef DISABLE_NETWORK
    GetContext().GetNetwork().Close();
#endif
    // The title screen shows an empty map behind the menu, made of the default terrain.
    auto& objectManager = GetContext().GetObjectManager();
    objectManager.UnloadAll();
    for (const auto& identifier : kMinimumRequiredObjects)
    {
        objectManager.LoadObject(identifier);
    }
    gameStateInitAll(getGameState(), kDefaultMapSize);
    ContextResetSubsystems();
    ContextOpenWindow(WindowClass::mainWindow);
    CentreView();

    Audio::PlayTitleMusic();

    CreateWindows();

    if (gOpenRCT2ShowChangelog)
    {
        gOpenRCT2ShowChangelog = false;
        ContextOpenWindow(WindowClass::changelog);
    }

    LOG_VERBOSE("TitleScene::Load() finished");
}

void TitleScene::CentreView()
{
    WindowBase* mainWindow = WindowGetMain();
    if (mainWindow == nullptr)
    {
        return;
    }
    const auto centre = TileCoordsXY{ kDefaultMapSize.x / 2, kDefaultMapSize.y / 2 }.toCoordsXY().toTileCentre();
    WindowScrollToLocation(*mainWindow, CoordsXYZ{ centre, TileElementHeight(centre) });
}

void TitleScene::Tick()
{
    gInUpdateCode = true;

    ScreenshotCheck();
    TitleHandleKeyboardInput();

    if (!gOpenRCT2Headless && GameIsNotPaused())
    {
        int32_t numUpdates = 1;
        if (gGameSpeed > 1)
        {
            numUpdates = 1 << (gGameSpeed - 1);
        }
        for (int32_t i = 0; i < numUpdates; i++)
        {
            gameStateUpdateLogic();
        }
        Drawing::UpdatePaletteEffects();
        // update_weather_animation();
    }

    gInputFlags.unset(InputFlag::viewportScrolling);

    ContextHandleInput();

    gInUpdateCode = false;
}

void TitleScene::Stop()
{
    Audio::StopAll();
}

/**
 * Creates the windows shown on the title screen; New game, load game,
 * tutorial, toolbox and exit.
 */
void TitleScene::CreateWindows()
{
    ContextOpenWindow(WindowClass::titleMenu);
    ContextOpenWindow(WindowClass::titleExit);
    ContextOpenWindow(WindowClass::titleOptions);
    ContextOpenWindow(WindowClass::titleLogo);
    ContextOpenWindow(WindowClass::titleVersion);
    WindowResizeGui(ContextGetWidth(), ContextGetHeight());
}

void TitleCreateWindows()
{
    auto* sceneMgr = GetContext()->GetSceneManager();
    auto* titleScene = static_cast<TitleScene*>(sceneMgr->getTitleScene());
    if (titleScene != nullptr)
    {
        titleScene->CreateWindows();
    }
}
