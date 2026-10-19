/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

// Viewport areas where nothing is painted are cleared every frame instead of keeping the previous frame.

#include <gtest/gtest.h>
#include <openrct2/Context.h>
#include <openrct2/GameState.h>
#include <openrct2/OpenRCT2.h>
#include <openrct2/config/Config.h>
#include <openrct2/drawing/RenderTarget.h>
#include <openrct2/drawing/X8DrawingEngine.h>
#include <openrct2/interface/Viewport.h>
#include <openrct2/ui/UiContext.h>

#include <vector>

using namespace OpenRCT2;
using namespace OpenRCT2::Drawing;

class ViewportClear : public testing::Test
{
protected:
    static constexpr int32_t kWidth = 128;
    static constexpr int32_t kHeight = 64;
    static constexpr auto kGarbage = static_cast<PaletteIndex>(0xAB);

    static void SetUpTestSuite()
    {
        gOpenRCT2Headless = true;
        gOpenRCT2NoGraphics = false;
        _context = CreateContext();
        ASSERT_TRUE(_context->Initialise());
        Config::Get().general.multiThreading = false;
        gameStateInitAll(getGameState(), { 16, 16 });
    }

    static void TearDownTestSuite()
    {
        _context = nullptr;
    }

    // Renders a viewport looking far away from the map, so that nothing is painted.
    std::vector<PaletteIndex> RenderVoid(ViewportFlags flags)
    {
        X8DrawingEngine engine(GetContext()->GetUiContext());
        std::vector<PaletteIndex> pixels(kWidth * kHeight, kGarbage);

        RenderTarget rt;
        rt.DrawingEngine = &engine;
        rt.bits = pixels.data();
        rt.width = kWidth;
        rt.height = kHeight;

        Viewport viewport;
        viewport.width = kWidth;
        viewport.height = kHeight;
        viewport.viewPos = { -100000, -100000 };
        viewport.flags = flags;
        viewport.isVisible = true;

        engine.BeginDraw();
        ViewportRender(rt, &viewport);
        engine.EndDraw();
        return pixels;
    }

    static std::unique_ptr<IContext> _context;
};

std::unique_ptr<IContext> ViewportClear::_context;

TEST_F(ViewportClear, VoidIsClearedToVoidColour)
{
    for (auto pixel : RenderVoid({}))
    {
        ASSERT_EQ(kViewportVoidColour, pixel);
    }
}

TEST_F(ViewportClear, TransparentBackgroundIsKept)
{
    for (auto pixel : RenderVoid({ ViewportFlag::transparentBackground }))
    {
        ASSERT_EQ(kGarbage, pixel);
    }
}
