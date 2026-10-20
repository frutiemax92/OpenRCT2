/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <algorithm>
#include <openrct2-ui/interface/Widget.h>
#include <openrct2-ui/interface/Window.h>
#include <openrct2-ui/windows/Windows.h>
#include <openrct2/Context.h>
#include <openrct2/drawing/Drawing.Screen.h>
#include <openrct2/drawing/Drawing.h>
#include <openrct2/drawing/ModelSpriteStore.h>
#include <openrct2/drawing/Text.h>
#include <openrct2/localisation/StringIds.h>
#include <openrct2/model/ModelViewer.h>
#include <openrct2/ui/WindowManager.h>

namespace OpenRCT2::Ui::Windows
{
    // Debug window (R3D-008): the loaded model with all its animation clips (rows) in all four view rotations (columns).
    enum WindowModelViewerWidgetIdx : WidgetIndex
    {
        WIDX_BACKGROUND,
        WIDX_TITLE,
        WIDX_CLOSE,
        WIDX_PAUSE,
    };

    static constexpr int32_t kLabelWidth = 96;
    static constexpr int32_t kCellWidth = 72;
    static constexpr int32_t kCellHeight = 60;
    static constexpr int32_t kHeaderHeight = 54;
    static constexpr int32_t kMaxVisibleRows = 8;
    static constexpr int32_t kWindowWidth = kLabelWidth + Model::ModelViewer::kRotations * kCellWidth + 16;
    // One game tick lasts 40 ms.
    static constexpr float kSecondsPerTick = 0.025f;

    static ScreenSize WindowSizeForRows(int32_t rows)
    {
        return { kWindowWidth, kHeaderHeight + std::clamp(rows, 1, kMaxVisibleRows) * kCellHeight + 8 };
    }

    // clang-format off
    static constexpr auto kModelViewerWidgets = makeWidgets(
        makeWindowShim(STR_MODEL_VIEWER_TITLE, { kWindowWidth, 64 }),
        makeWidget({ 8, 36 }, { 160, 12 }, WidgetType::checkbox, WindowColour::secondary, STR_MODEL_VIEWER_PAUSE)
    );
    // clang-format on

    class ModelViewerWindow final : public Window
    {
    public:
        void onOpen() override
        {
            setWidgets(kModelViewerWidgets);
            initScrollWidgets();
            const auto size = WindowSizeForRows(Model::GetModelViewer().rowCount());
            width = minWidth = maxWidth = size.width;
            height = minHeight = maxHeight = size.height;
            widgets[WIDX_BACKGROUND].right = width - 1;
            widgets[WIDX_BACKGROUND].bottom = height - 1;
            widgets[WIDX_TITLE].right = width - 2;
            widgets[WIDX_CLOSE].left = width - 13;
            widgets[WIDX_CLOSE].right = width - 3;
        }

        void onMouseUp(WidgetIndex widgetIndex) override
        {
            switch (widgetIndex)
            {
                case WIDX_CLOSE:
                    close();
                    break;
                case WIDX_PAUSE:
                {
                    auto& viewer = Model::GetModelViewer();
                    viewer.setPaused(!viewer.paused());
                    invalidate();
                    break;
                }
            }
        }

        void onUpdate() override
        {
            Model::GetModelViewer().advance(kSecondsPerTick);
            invalidate();
        }

        void onPrepareDraw() override
        {
            setCheckboxValue(WIDX_PAUSE, Model::GetModelViewer().paused());
        }

        void onDraw(Drawing::RenderTarget& rt) override
        {
            WindowDrawWidgets(*this, rt);

            auto& viewer = Model::GetModelViewer();
            drawText(rt, windowPos + ScreenCoordsXY{ 8, 20 }, viewer.summary(), { colours[1] });

            auto& store = Drawing::ModelSpriteStore::Instance();
            const int32_t rows = std::min(viewer.rowCount(), kMaxVisibleRows);
            for (int32_t row = 0; row < rows; row++)
            {
                const int32_t top = kHeaderHeight + row * kCellHeight;
                drawText(rt, windowPos + ScreenCoordsXY{ 8, top + kCellHeight / 2 - 5 }, viewer.rowName(row), { colours[1] });
                for (int32_t rotation = 0; rotation < Model::ModelViewer::kRotations; rotation++)
                {
                    // The model origin sits at the bottom centre of its cell.
                    const ScreenCoordsXY origin = windowPos
                        + ScreenCoordsXY{ 8 + kLabelWidth + rotation * kCellWidth + kCellWidth / 2, top + kCellHeight - 14 };
                    const auto image = store.GetImage(viewer.keyFor(row, rotation));
                    if (image != kImageIndexUndefined)
                        GfxDrawSprite(rt, ImageId(image), origin);
                }
            }
        }
    };

    WindowBase* ModelViewerOpen()
    {
        auto* windowMgr = GetWindowManager();
        auto* window = windowMgr->BringToFrontByClass(WindowClass::modelViewer);
        if (window != nullptr)
            windowMgr->Close(*window); // reopened so the window is sized for the model that was just loaded
        return windowMgr->Create<ModelViewerWindow>(
            WindowClass::modelViewer, ScreenCoordsXY(32, 32), WindowSizeForRows(Model::GetModelViewer().rowCount()));
    }
} // namespace OpenRCT2::Ui::Windows
