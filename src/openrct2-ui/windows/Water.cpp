/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <openrct2-ui/input/MouseInput.h>
#include <openrct2-ui/interface/LandTool.h>
#include <openrct2-ui/interface/Widget.h>
#include <openrct2-ui/interface/Window.h>
#include <openrct2-ui/windows/Windows.h>
#include <openrct2/Context.h>
#include <openrct2/GameState.h>
#include <openrct2/Input.h>
#include <openrct2/SpriteIds.h>
#include <openrct2/actions/GameActionRunner.h>
#include <openrct2/actions/scenery/StreamPlaceAction.h>
#include <openrct2/actions/scenery/StreamRemoveAction.h>
#include <openrct2/actions/terraform/WaterLowerAction.h>
#include <openrct2/actions/terraform/WaterRaiseAction.h>
#include <openrct2/drawing/Text.h>
#include <openrct2/interface/Viewport.h>
#include <openrct2/localisation/Formatter.h>
#include <openrct2/object/ObjectLimits.h>
#include <openrct2/object/ObjectManager.h>
#include <openrct2/object/StreamObject.h>
#include <openrct2/ui/WindowManager.h>
#include <openrct2/world/MapSelection.h>
#include <string>

namespace OpenRCT2::Ui::Windows
{
    using namespace GameActions;

    static constexpr StringId kWindowTitle = STR_WATER;
    static constexpr ScreenSize kWindowSize = { 144, 115 };
    static constexpr const char* kFlowDirections[4] = { "North (-y)", "East (+x)", "South (+y)", "West (-x)" };

    enum class WaterToolMode : uint8_t
    {
        water,
        streamPlace,
        streamRemove,
    };

    enum WindowWaterWidgetIdx : WidgetIndex
    {
        WIDX_BACKGROUND,
        WIDX_TITLE,
        WIDX_CLOSE,
        WIDX_MODE_WATER,
        WIDX_MODE_STREAM_PLACE,
        WIDX_MODE_STREAM_REMOVE,
        WIDX_STREAM_FLOW,
        WIDX_PREVIEW,
        WIDX_DECREMENT,
        WIDX_INCREMENT,
    };

    // clang-format off
    static constexpr auto _waterWidgets = makeWidgets(
        makeWindowShim(kWindowTitle, kWindowSize),
        makeWidget     ({ 6, 19}, {42, 20}, WidgetType::button, WindowColour::secondary, kStringIdNone),
        makeWidget     ({50, 19}, {44, 20}, WidgetType::button, WindowColour::secondary, kStringIdNone),
        makeWidget     ({96, 19}, {42, 20}, WidgetType::button, WindowColour::secondary, kStringIdNone),
        makeWidget     ({10, 48}, {124, 20}, WidgetType::button, WindowColour::secondary, kStringIdNone),
        makeWidget     ({50, 44}, {44, 32}, WidgetType::imgBtn, WindowColour::primary , ImageId(SPR_LAND_TOOL_SIZE_0),   kStringIdNone),            // preview box
        makeRemapWidget({51, 45}, {16, 16}, WidgetType::trnBtn, WindowColour::tertiary, SPR_LAND_TOOL_DECREASE, STR_ADJUST_SMALLER_WATER_TIP), // decrement size
        makeRemapWidget({77, 59}, {16, 16}, WidgetType::trnBtn, WindowColour::tertiary, SPR_LAND_TOOL_INCREASE, STR_ADJUST_LARGER_WATER_TIP) // increment size
    );
    // clang-format on

    class WaterWindow final : public Window
    {
    private:
        money64 _waterToolRaiseCost = kMoney64Undefined;
        money64 _waterToolLowerCost = kMoney64Undefined;
        WaterToolMode _mode = WaterToolMode::water;
        Direction _flowDirection = 0;
        std::string _status;

        static ObjectEntryIndex getFirstLoadedStreamObject()
        {
            auto& objMgr = GetContext()->GetObjectManager();
            for (ObjectEntryIndex i = 0; i < kMaxStreamObjects; ++i)
            {
                if (objMgr.GetLoadedObject<StreamObject>(i) != nullptr)
                    return i;
            }
            return kObjectEntryIndexNull;
        }

        void applyStreamPlace(const ScreenCoordsXY& screenCoords)
        {
            const auto position = ScreenGetMapXY(screenCoords, nullptr);
            if (!position.has_value())
                return;
            const auto tile = position->toTileStart();
            const auto streamType = getFirstLoadedStreamObject();
            if (streamType == kObjectEntryIndexNull)
            {
                _status = "No stream loaded.";
                invalidate();
                return;
            }
            StreamPlaceAction action(tile, streamType, _flowDirection);
            auto res = Execute(&action, getGameState());
            if (res.error != Status::ok)
                _status = res.getErrorMessage();
            else
                _status.clear();
            invalidate();
        }

        void applyStreamRemove(const ScreenCoordsXY& screenCoords)
        {
            const auto position = ScreenGetMapXY(screenCoords, nullptr);
            if (!position.has_value())
                return;
            const auto tile = position->toTileStart();
            StreamRemoveAction action(tile);
            auto res = Execute(&action, getGameState());
            if (res.error != Status::ok)
                _status = res.getErrorMessage();
            else
                _status.clear();
            invalidate();
        }

    public:
        void onOpen() override
        {
            setWidgets(_waterWidgets);

            widgetsSetHoldable(*this, { WIDX_INCREMENT, WIDX_DECREMENT });
            WindowInitScrollWidgets(*this);
            WindowPushOthersBelow(*this);

            gLandToolSize = 1;
        }

        void onClose() override
        {
            // If the tool wasn't changed, turn tool off
            if (isToolActive(WindowClass::water, WIDX_BACKGROUND))
            {
                ToolCancel();
            }
        }

        void onMouseUp(WidgetIndex widgetIndex) override
        {
            switch (widgetIndex)
            {
                case WIDX_CLOSE:
                    close();
                    break;
                case WIDX_MODE_WATER:
                    _mode = WaterToolMode::water;
                    ToolSet(*this, WIDX_BACKGROUND, Tool::waterDown);
                    _status.clear();
                    invalidate();
                    break;
                case WIDX_MODE_STREAM_PLACE:
                    _mode = WaterToolMode::streamPlace;
                    ToolSet(*this, WIDX_BACKGROUND, Tool::crosshair);
                    _status.clear();
                    invalidate();
                    break;
                case WIDX_MODE_STREAM_REMOVE:
                    _mode = WaterToolMode::streamRemove;
                    ToolSet(*this, WIDX_BACKGROUND, Tool::crosshair);
                    _status.clear();
                    invalidate();
                    break;
                case WIDX_STREAM_FLOW:
                    _flowDirection = (_flowDirection + 1) & 3;
                    invalidate();
                    break;
                case WIDX_PREVIEW:
                    if (_mode == WaterToolMode::water)
                        InputSize();
                    break;
            }
        }

        void onMouseDown(WidgetIndex widgetIndex) override
        {
            if (_mode != WaterToolMode::water)
                return;

            switch (widgetIndex)
            {
                case WIDX_DECREMENT:
                    // Decrement land tool size
                    gLandToolSize = std::max<uint16_t>(kLandToolMinimumSize, gLandToolSize - 1);

                    // Invalidate the window
                    invalidate();
                    break;
                case WIDX_INCREMENT:
                    // Increment land tool size
                    gLandToolSize = std::min<uint16_t>(kLandToolMaximumSize, gLandToolSize + 1);

                    // Invalidate the window
                    invalidate();
                    break;
            }
        }

        void onUpdate() override
        {
            // Close window if another tool is open
            if (!isToolActive(WindowClass::water, WIDX_BACKGROUND))
            {
                close();
            }
        }

        void onTextInput(WidgetIndex widgetIndex, std::string_view text) override
        {
            int32_t size;
            char* end;

            if (widgetIndex != WIDX_PREVIEW || _mode != WaterToolMode::water)
            {
                return;
            }

            std::string textStr = std::string(text);
            size = strtol(textStr.c_str(), &end, 10);
            if (*end == '\0')
            {
                size = std::max<uint16_t>(kLandToolMinimumSize, size);
                size = std::min<uint16_t>(kLandToolMaximumSize, size);
                gLandToolSize = size;

                invalidate();
            }
        }

        void onPrepareDraw() override
        {
            setWidgetPressed(WIDX_MODE_WATER, _mode == WaterToolMode::water);
            setWidgetPressed(WIDX_MODE_STREAM_PLACE, _mode == WaterToolMode::streamPlace);
            setWidgetPressed(WIDX_MODE_STREAM_REMOVE, _mode == WaterToolMode::streamRemove);

            widgets[WIDX_PREVIEW].setVisible(_mode == WaterToolMode::water);
            widgets[WIDX_DECREMENT].setVisible(_mode == WaterToolMode::water);
            widgets[WIDX_INCREMENT].setVisible(_mode == WaterToolMode::water);
            widgets[WIDX_STREAM_FLOW].setVisible(_mode == WaterToolMode::streamPlace);

            if (_mode == WaterToolMode::water)
            {
                // Set the preview image button to be pressed down
                setWidgetPressed(WIDX_PREVIEW, true);

                // Update the preview image
                widgets[WIDX_PREVIEW].image = ImageId(LandTool::SizeToSpriteIndex(gLandToolSize));
            }
        }

        void onDraw(Drawing::RenderTarget& rt) override
        {
            drawWidgets(rt);

            auto text = [&](int32_t x, int32_t y, const std::string& s, Drawing::Colour colour = Drawing::Colour::black) {
                drawText(rt, windowPos + ScreenCoordsXY{ x, y }, s, { colour });
            };

            text(widgets[WIDX_MODE_WATER].left + 5, widgets[WIDX_MODE_WATER].top + 4, "Water");
            text(widgets[WIDX_MODE_STREAM_PLACE].left + 4, widgets[WIDX_MODE_STREAM_PLACE].top + 4, "Stream");
            text(widgets[WIDX_MODE_STREAM_REMOVE].left + 3, widgets[WIDX_MODE_STREAM_REMOVE].top + 4, "Remove");

            if (_mode == WaterToolMode::water)
            {
                auto screenCoords = ScreenCoordsXY{ windowPos.x + widgets[WIDX_PREVIEW].midX(),
                                                    windowPos.y + widgets[WIDX_PREVIEW].midY() };
                // Draw number for tool sizes bigger than 7
                if (gLandToolSize > kLandToolMaximumSizeWithSprite)
                {
                    auto ft = Formatter();
                    ft.Add<uint16_t>(gLandToolSize);
                    drawText(rt, screenCoords - ScreenCoordsXY{ 0, 2 }, STR_LAND_TOOL_SIZE_VALUE, ft, { TextAlignment::centre });
                }

                if (!getGameState().park.flags.has(ParkFlag::noMoney))
                {
                    // Draw raise cost amount
                    screenCoords = { widgets[WIDX_PREVIEW].midX() + windowPos.x, widgets[WIDX_PREVIEW].bottom + windowPos.y + 5 };
                    if (_waterToolRaiseCost != kMoney64Undefined && _waterToolRaiseCost != 0)
                    {
                        auto ft = Formatter();
                        ft.Add<money64>(_waterToolRaiseCost);
                        drawText(rt, screenCoords, STR_RAISE_COST_AMOUNT, ft, { TextAlignment::centre });
                    }
                    screenCoords.y += 10;

                    // Draw lower cost amount
                    if (_waterToolLowerCost != kMoney64Undefined && _waterToolLowerCost != 0)
                    {
                        auto ft = Formatter();
                        ft.Add<money64>(_waterToolLowerCost);
                        drawText(rt, screenCoords, STR_LOWER_COST_AMOUNT, ft, { TextAlignment::centre });
                    }
                }
            }
            else if (_mode == WaterToolMode::streamPlace)
            {
                text(widgets[WIDX_STREAM_FLOW].left + 6, widgets[WIDX_STREAM_FLOW].top + 4, std::string("Flow: ") + kFlowDirections[_flowDirection & 3]);
                const auto streamType = getFirstLoadedStreamObject();
                if (streamType == kObjectEntryIndexNull)
                    text(10, 74, "No stream object loaded", Drawing::Colour::grey);
                else
                    text(10, 74, "Click / drag to place stream", Drawing::Colour::darkGreen);

                if (!_status.empty())
                    text(10, 92, _status, Drawing::Colour::brightRed);
            }
            else if (_mode == WaterToolMode::streamRemove)
            {
                text(10, 52, "Remove Stream Mode", Drawing::Colour::black);
                text(10, 72, "Click stream to remove", Drawing::Colour::brightRed);
                if (!_status.empty())
                    text(10, 92, _status, Drawing::Colour::brightRed);
            }
        }

        void onToolUpdate(WidgetIndex widgetIndex, const ScreenCoordsXY& screenCoords) override
        {
            if (_mode == WaterToolMode::water)
            {
                switch (widgetIndex)
                {
                    case WIDX_BACKGROUND:
                        ToolUpdateWater(screenCoords);
                        break;
                }
            }
            else
            {
                const auto position = ScreenGetMapXY(screenCoords, nullptr);
                if (position.has_value())
                {
                    const auto tile = position->toTileStart();
                    gMapSelectFlags.set(MapSelectFlag::enable);
                    gMapSelectType = MapSelectType::full;
                    gMapSelectPositionA = tile;
                    gMapSelectPositionB = tile;
                }
                else
                {
                    gMapSelectFlags.unset(MapSelectFlag::enable);
                }
            }
        }

        void onToolDown(WidgetIndex widgetIndex, const ScreenCoordsXY& screenCoords) override
        {
            if (_mode == WaterToolMode::water)
            {
                switch (widgetIndex)
                {
                    case WIDX_BACKGROUND:
                        if (gMapSelectFlags.has(MapSelectFlag::enable))
                        {
                            gCurrentToolId = Tool::upDownArrow;
                        }
                        break;
                }
            }
            else if (_mode == WaterToolMode::streamPlace)
            {
                applyStreamPlace(screenCoords);
            }
            else if (_mode == WaterToolMode::streamRemove)
            {
                applyStreamRemove(screenCoords);
            }
        }

        void onToolDrag(WidgetIndex widgetIndex, const ScreenCoordsXY& screenCoords) override
        {
            if (_mode == WaterToolMode::water)
            {
                switch (widgetIndex)
                {
                    case WIDX_BACKGROUND:
                        WaterToolDrag(screenCoords);
                        break;
                }
            }
            else if (_mode == WaterToolMode::streamPlace)
            {
                applyStreamPlace(screenCoords);
            }
            else if (_mode == WaterToolMode::streamRemove)
            {
                applyStreamRemove(screenCoords);
            }
        }

        void onToolUp(WidgetIndex widgetIndex, const ScreenCoordsXY& screenCoords) override
        {
            if (_mode == WaterToolMode::water)
            {
                switch (widgetIndex)
                {
                    case WIDX_BACKGROUND:
                        gMapSelectFlags.unset(MapSelectFlag::enable);
                        gCurrentToolId = Tool::waterDown;
                        break;
                }
            }
        }

        void onToolAbort(WidgetIndex widgetIndex) override
        {
            switch (widgetIndex)
            {
                case WIDX_BACKGROUND:
                    HideGridlines();
                    break;
            }
        }

    private:
        /**
         *  part of window_top_toolbar_tool_drag(0x0066CB4E)
         *  rct2: 0x006E6D4B
         */
        void WaterToolDrag(const ScreenCoordsXY& screenPos)
        {
            auto* windowMgr = GetWindowManager();
            auto* window = windowMgr->FindFromPoint(screenPos);
            if (window == nullptr || window->viewport == nullptr)
                return;

            int16_t dx = window->viewport->zoom.ApplyInversedTo(-16);

            auto offsetPos = screenPos - ScreenCoordsXY{ 0, gInputDragLast.y };

            auto& gameState = getGameState();

            if (offsetPos.y <= dx)
            {
                gInputDragLast.y += dx;

                auto waterRaiseAction = GameActions::WaterRaiseAction(
                    { gMapSelectPositionA.x, gMapSelectPositionA.y, gMapSelectPositionB.x, gMapSelectPositionB.y });
                GameActions::Execute(&waterRaiseAction, gameState);

                _waterToolRaiseCost = kMoney64Undefined;
                _waterToolLowerCost = kMoney64Undefined;

                return;
            }

            dx = -dx;

            if (offsetPos.y >= dx)
            {
                gInputDragLast.y += dx;

                auto waterLowerAction = GameActions::WaterLowerAction(
                    { gMapSelectPositionA.x, gMapSelectPositionA.y, gMapSelectPositionB.x, gMapSelectPositionB.y });
                GameActions::Execute(&waterLowerAction, gameState);
                _waterToolRaiseCost = kMoney64Undefined;
                _waterToolLowerCost = kMoney64Undefined;

                return;
            }
        }

        /**
         *
         *  rct2: 0x006E6BDC
         */
        void ToolUpdateWater(const ScreenCoordsXY& screenPos)
        {
            auto* windowMgr = GetWindowManager();
            auto& gameState = getGameState();

            if (gCurrentToolId == Tool::upDownArrow)
            {
                if (!gMapSelectFlags.has(MapSelectFlag::enable))
                    return;

                auto waterLowerAction = GameActions::WaterLowerAction(
                    { gMapSelectPositionA.x, gMapSelectPositionA.y, gMapSelectPositionB.x, gMapSelectPositionB.y });
                waterLowerAction.SetFlags({ GameActions::CommandFlag::allowDuringPaused });
                auto waterRaiseAction = GameActions::WaterRaiseAction(
                    { gMapSelectPositionA.x, gMapSelectPositionA.y, gMapSelectPositionB.x, gMapSelectPositionB.y });
                waterRaiseAction.SetFlags({ GameActions::CommandFlag::allowDuringPaused });

                auto res = GameActions::Query(&waterLowerAction, gameState);
                money64 lowerCost = res.error == GameActions::Status::ok ? res.cost : kMoney64Undefined;

                res = GameActions::Query(&waterRaiseAction, gameState);
                money64 raiseCost = res.error == GameActions::Status::ok ? res.cost : kMoney64Undefined;

                if (_waterToolRaiseCost != raiseCost || _waterToolLowerCost != lowerCost)
                {
                    _waterToolRaiseCost = raiseCost;
                    _waterToolLowerCost = lowerCost;
                    windowMgr->InvalidateByClass(WindowClass::water);
                }
                return;
            }

            gMapSelectFlags.unset(MapSelectFlag::enable);

            auto info = GetMapCoordinatesFromPos(
                screenPos, { ViewportInteractionItem::terrain, ViewportInteractionItem::water });

            if (info.interactionType == ViewportInteractionItem::none)
            {
                if (_waterToolRaiseCost != kMoney64Undefined || _waterToolLowerCost != kMoney64Undefined)
                {
                    _waterToolRaiseCost = kMoney64Undefined;
                    _waterToolLowerCost = kMoney64Undefined;
                    windowMgr->InvalidateByClass(WindowClass::water);
                }
                return;
            }

            auto mapTile = info.Loc;

            uint8_t state_changed = 0;

            if (!gMapSelectFlags.has(MapSelectFlag::enable))
            {
                gMapSelectFlags.set(MapSelectFlag::enable);
                state_changed++;
            }

            if (gMapSelectType != MapSelectType::fullWater)
            {
                gMapSelectType = MapSelectType::fullWater;
                state_changed++;
            }

            uint16_t tool_size = std::max<uint16_t>(1, gLandToolSize);
            uint16_t tool_length = (tool_size - 1) * kCoordsXYStep;

            // Move to tool bottom left
            mapTile.x -= tool_length / 2;
            mapTile.y -= tool_length / 2;
            mapTile = mapTile.toTileStart();

            if (gMapSelectPositionA.x != mapTile.x)
            {
                gMapSelectPositionA.x = mapTile.x;
                state_changed++;
            }

            if (gMapSelectPositionA.y != mapTile.y)
            {
                gMapSelectPositionA.y = mapTile.y;
                state_changed++;
            }

            mapTile.x += tool_length;
            mapTile.y += tool_length;

            if (gMapSelectPositionB.x != mapTile.x)
            {
                gMapSelectPositionB.x = mapTile.x;
                state_changed++;
            }

            if (gMapSelectPositionB.y != mapTile.y)
            {
                gMapSelectPositionB.y = mapTile.y;
                state_changed++;
            }

            if (!state_changed)
                return;

            auto waterLowerAction = GameActions::WaterLowerAction(
                { gMapSelectPositionA.x, gMapSelectPositionA.y, gMapSelectPositionB.x, gMapSelectPositionB.y });
            waterLowerAction.SetFlags({ GameActions::CommandFlag::allowDuringPaused });
            auto waterRaiseAction = GameActions::WaterRaiseAction(
                { gMapSelectPositionA.x, gMapSelectPositionA.y, gMapSelectPositionB.x, gMapSelectPositionB.y });
            waterRaiseAction.SetFlags({ GameActions::CommandFlag::allowDuringPaused });

            auto res = GameActions::Query(&waterLowerAction, gameState);
            money64 lowerCost = res.error == GameActions::Status::ok ? res.cost : kMoney64Undefined;

            res = GameActions::Query(&waterRaiseAction, gameState);
            money64 raiseCost = res.error == GameActions::Status::ok ? res.cost : kMoney64Undefined;

            if (_waterToolRaiseCost != raiseCost || _waterToolLowerCost != lowerCost)
            {
                _waterToolRaiseCost = raiseCost;
                _waterToolLowerCost = lowerCost;
                windowMgr->InvalidateByClass(WindowClass::water);
            }
        }

        void InputSize()
        {
            Formatter ft;
            ft.Add<uint16_t>(kLandToolMinimumSize);
            ft.Add<uint16_t>(kLandToolMaximumSize);
            WindowTextInputOpen(
                this, WIDX_PREVIEW, STR_SELECTION_SIZE, STR_ENTER_SELECTION_SIZE, ft, kStringIdNone, kStringIdNone, 3);
        }
    };

    WindowBase* WaterOpen()
    {
        auto* windowMgr = GetWindowManager();
        return windowMgr->FocusOrCreate<WaterWindow>(
            WindowClass::water, ScreenCoordsXY(ContextGetWidth() - kWindowSize.width, 29), kWindowSize, {});
    }

    /**
     *
     *  rct2: 0x0066CD9C
     */
    void ToggleWaterWindow()
    {
        if (isToolActive(WindowClass::water, WIDX_BACKGROUND))
        {
            ToolCancel();
        }
        else
        {
            ShowGridlines();
            auto* toolWindow = ContextOpenWindow(WindowClass::water);
            ToolSet(*toolWindow, WIDX_BACKGROUND, Tool::waterDown);
            gInputFlags.set(InputFlag::allowRightMouseRemoval);
        }
    }
} // namespace OpenRCT2::Ui::Windows
