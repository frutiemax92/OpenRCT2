/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
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
#include <openrct2/object/AnimalObject.h>
#include <openrct2/object/GolferModelObject.h>
#include <openrct2/object/ObjectLimits.h>
#include <openrct2/object/ObjectManager.h>
#include <openrct2/object/VehicleModelObject.h>
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
        WIDX_PREV,
        WIDX_NEXT,
        WIDX_YAW_LEFT,
        WIDX_YAW_RIGHT,
    };

    static constexpr int32_t kLabelWidth = 96;
    static constexpr int32_t kCellWidth = 72;
    static constexpr int32_t kCellHeight = 46;
    static constexpr int32_t kHeaderHeight = 54;
    static constexpr int32_t kMaxVisibleRows = 13;
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
        makeWidget({ 8, 36 }, { 80, 12 }, WidgetType::checkbox, WindowColour::secondary, STR_MODEL_VIEWER_PAUSE),
        makeWidget({ 96, 34 }, { 22, 16 }, WidgetType::button, WindowColour::secondary, kStringIdNone),
        makeWidget({ 120, 34 }, { 22, 16 }, WidgetType::button, WindowColour::secondary, kStringIdNone),
        makeWidget({ 160, 34 }, { 40, 16 }, WidgetType::button, WindowColour::secondary, kStringIdNone),
        makeWidget({ 204, 34 }, { 40, 16 }, WidgetType::button, WindowColour::secondary, kStringIdNone)
    );
    // clang-format on

    struct ListedModel
    {
        std::string name;
        uint32_t modelId;
    };

    // The models the loaded objects registered: the golfer, the cars and carts, the animals.
    static std::vector<ListedModel> ListModels()
    {
        std::vector<ListedModel> models;
        auto& objects = GetContext()->GetObjectManager();
        for (ObjectEntryIndex i = 0; i < kMaxGolferModelObjects; i++)
            if (const auto* o = objects.GetLoadedObject<GolferModelObject>(i); o != nullptr && o->ModelId != 0)
                models.push_back({ std::string(o->GetIdentifier()), o->ModelId });
        for (ObjectEntryIndex i = 0; i < kMaxVehicleModelObjects; i++)
            if (const auto* o = objects.GetLoadedObject<VehicleModelObject>(i); o != nullptr && o->ModelId != 0)
                models.push_back({ std::string(o->GetIdentifier()), o->ModelId });
        for (ObjectEntryIndex i = 0; i < kMaxAnimalObjects; i++)
            if (const auto* o = objects.GetLoadedObject<AnimalObject>(i); o != nullptr && o->ModelId != 0)
                models.push_back({ std::string(o->GetIdentifier()), o->ModelId });
        return models;
    }

    static void ShowModelAtOffset(int32_t offset)
    {
        auto& viewer = Model::GetModelViewer();
        const auto models = ListModels();
        if (models.empty())
            return;
        int32_t current = -1;
        for (size_t i = 0; i < models.size(); i++)
            if (models[i].modelId == viewer.modelId())
                current = static_cast<int32_t>(i);
        const int32_t count = static_cast<int32_t>(models.size());
        const int32_t next = current < 0 ? 0 : ((current + offset) % count + count) % count;
        viewer.showRegistered(models[next].modelId, models[next].name);
    }

    class ModelViewerWindow final : public Window
    {
    public:
        void onOpen() override
        {
            setWidgets(kModelViewerWidgets);
            initScrollWidgets();
            const auto size = WindowSizeForRows(kMaxVisibleRows);
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
                case WIDX_PREV:
                    ShowModelAtOffset(-1);
                    invalidate();
                    break;
                case WIDX_NEXT:
                    ShowModelAtOffset(1);
                    invalidate();
                    break;
                case WIDX_YAW_LEFT:
                case WIDX_YAW_RIGHT:
                {
                    // Steps of 32 in the 256 turns of the entity yaw (45 degrees): 0 faces -x, 128 +x.
                    auto& v = Model::GetModelViewer();
                    const float step = 3.14159265f / 4.0f * (widgetIndex == WIDX_YAW_RIGHT ? 1.0f : -1.0f);
                    v.setYaw(std::fmod(v.yaw() + step + 2.0f * 3.14159265f, 2.0f * 3.14159265f));
                    invalidate();
                    break;
                }
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
            const auto label = [&](WidgetIndex w, const char* text) {
                drawText(rt, windowPos + ScreenCoordsXY{ widgets[w].left + 4, widgets[w].top + 3 }, std::string(text), { colours[1] });
            };
            label(WIDX_PREV, "<");
            label(WIDX_NEXT, ">");
            label(WIDX_YAW_LEFT, "Yaw-");
            label(WIDX_YAW_RIGHT, "Yaw+");
            // The game's own yaw unit (turns of 256, 0 faces -x), with the map direction it points to.
            const int32_t entityYaw = static_cast<int32_t>(std::lround(viewer.yaw() * 256.0f / (2.0f * 3.14159265f))) & 255;
            static constexpr const char* kFacing[8] = { "-x", "-x+y", "+y", "+x+y", "+x", "+x-y", "-y", "-x-y" };
            drawText(
                rt, windowPos + ScreenCoordsXY{ 252, 37 },
                "yaw " + std::to_string(entityYaw) + " faces " + kFacing[((entityYaw + 16) / 32) & 7], { colours[1] });

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
        // With nothing loaded from a file, show the first model the game has: the golfer.
        if (!Model::GetModelViewer().isLoaded())
            ShowModelAtOffset(0);
        return windowMgr->Create<ModelViewerWindow>(
            WindowClass::modelViewer, ScreenCoordsXY(32, 32), WindowSizeForRows(kMaxVisibleRows));
    }
} // namespace OpenRCT2::Ui::Windows
