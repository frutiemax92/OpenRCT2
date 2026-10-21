/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Windows.h"

#include <algorithm>
#include <openrct2-ui/interface/Widget.h>
#include <openrct2-ui/interface/Window.h>
#include <openrct2/Context.h>
#include <openrct2/GameState.h>
#include <openrct2/actions/GameActionRunner.h>
#include <openrct2/actions/scenery/ParkingPlaceAction.h>
#include <openrct2/actions/scenery/ParkingRemoveAction.h>
#include <openrct2/actions/scenery/StreetPlaceAction.h>
#include <openrct2/actions/scenery/StreetRemoveAction.h>
#include <openrct2/actions/scenery/StreetRoadEntranceAction.h>
#include <openrct2/drawing/Colour.h>
#include <openrct2/drawing/Drawing.Screen.h>
#include <openrct2/drawing/Drawing.h>
#include <openrct2/drawing/RenderTarget.h>
#include <openrct2/drawing/Text.h>
#include <openrct2/interface/Viewport.h>
#include <openrct2/localisation/StringIds.h>
#include <openrct2/object/ObjectLimits.h>
#include <openrct2/object/ObjectManager.h>
#include <openrct2/object/StreetObject.h>
#include <openrct2/ui/WindowManager.h>
#include <openrct2/world/Street.h>
#include <openrct2/world/tile_element/StreetElement.h>
#include <string>

namespace OpenRCT2::Ui::Windows
{
    using namespace GameActions;
    using namespace OpenRCT2::Drawing;

    // The street tool (section 12.1): lay streets tile by tile (drag to paint a run of them), take them away, and mark road
    // entrances at the edge of the map.
    enum WindowStreetToolWidgetIdx : WidgetIndex
    {
        WIDX_BACKGROUND,
        WIDX_TITLE,
        WIDX_CLOSE,
        WIDX_TYPE_PREV,
        WIDX_TYPE_NEXT,
        WIDX_BUILD,
        WIDX_REMOVE,
        WIDX_ENTRANCE,
        WIDX_FLOW,
        WIDX_PARKING,
        WIDX_REMOVE_PARKING,
    };

    static constexpr ScreenSize kWindowSize = { 260, 160 };
    static constexpr const char* kFlowNames[4] = { "-x", "+y", "+x", "-y" };

    // clang-format off
    static constexpr auto kStreetToolWidgets = makeWidgets(
        makeWindowShim(STR_STREET_TOOL_TITLE, kWindowSize),
        makeWidget({  10, 20 }, {  20, 14 }, WidgetType::button, WindowColour::secondary, kStringIdNone),
        makeWidget({  34, 20 }, {  20, 14 }, WidgetType::button, WindowColour::secondary, kStringIdNone),
        makeWidget({  10, 44 }, {  75, 18 }, WidgetType::button, WindowColour::secondary, STR_STREET_TOOL_BUILD),
        makeWidget({  90, 44 }, {  75, 18 }, WidgetType::button, WindowColour::secondary, STR_STREET_TOOL_REMOVE),
        makeWidget({ 170, 44 }, {  80, 18 }, WidgetType::button, WindowColour::secondary, STR_STREET_TOOL_ENTRANCE, STR_STREET_TOOL_ENTRANCE_TIP),
        makeWidget({  10, 70 }, { 110, 14 }, WidgetType::button, WindowColour::secondary, STR_STREET_TOOL_FLOW),
        makeWidget({  10, 90 }, { 110, 18 }, WidgetType::button, WindowColour::secondary, STR_STREET_TOOL_PARKING),
        makeWidget({ 130, 90 }, { 120, 18 }, WidgetType::button, WindowColour::secondary, STR_STREET_TOOL_REMOVE_PARKING)
    );
    // clang-format on

    class StreetToolWindow final : public Window
    {
    private:
        ObjectEntryIndex _type = kObjectEntryIndexNull;
        Direction _flow = 2;
        std::string _status;

        static StreetObject* streetAt(ObjectEntryIndex index)
        {
            return GetContext()->GetObjectManager().GetLoadedObject<StreetObject>(index);
        }

        // Steps to the next loaded street object in a direction, wrapping around; kObjectEntryIndexNull when none is loaded.
        ObjectEntryIndex stepType(int32_t step) const
        {
            const int32_t count = kMaxStreetObjects;
            int32_t index = _type == kObjectEntryIndexNull ? (step > 0 ? -1 : 0) : _type;
            for (int32_t tried = 0; tried < count; tried++)
            {
                index = (index + step + count) % count;
                if (streetAt(static_cast<ObjectEntryIndex>(index)) != nullptr)
                    return static_cast<ObjectEntryIndex>(index);
            }
            return kObjectEntryIndexNull;
        }

        WidgetIndex activeTool() const
        {
            for (auto widget : { WIDX_BUILD, WIDX_REMOVE, WIDX_ENTRANCE, WIDX_PARKING, WIDX_REMOVE_PARKING })
                if (isToolActive(*this, widget))
                    return widget;
            return kWidgetIndexNull;
        }

        void apply(WidgetIndex tool, const ScreenCoordsXY& screenCoords)
        {
            const auto position = ScreenGetMapXY(screenCoords, nullptr);
            if (!position.has_value())
                return;
            const CoordsXY tile{ position->x & ~(kCoordsXYStep - 1), position->y & ~(kCoordsXYStep - 1) };
            Result result;
            auto& gameState = getGameState();
            switch (tool)
            {
                case WIDX_BUILD:
                {
                    if (_type == kObjectEntryIndexNull)
                        return;
                    StreetPlaceAction action(tile, _type, _flow);
                    result = Execute(&action, gameState);
                    break;
                }
                case WIDX_REMOVE:
                {
                    StreetRemoveAction action(tile);
                    result = Execute(&action, gameState);
                    break;
                }
                case WIDX_PARKING:
                {
                    if (_type == kObjectEntryIndexNull)
                        return;
                    // The cars drive in through the edge the "flow" arrow points at.
                    ParkingPlaceAction action(tile, _type, _flow);
                    result = Execute(&action, gameState);
                    break;
                }
                case WIDX_REMOVE_PARKING:
                {
                    ParkingRemoveAction action(tile);
                    result = Execute(&action, gameState);
                    break;
                }
                default:
                {
                    const auto* street = Street::FindAt(tile);
                    StreetRoadEntranceAction action(tile, street == nullptr || !street->isRoadEntrance());
                    result = Execute(&action, gameState);
                    break;
                }
            }
            _status = result.error == Status::ok ? std::string() : result.getErrorMessage();
        }

    public:
        void onOpen() override
        {
            setWidgets(kStreetToolWidgets);
            _type = stepType(1);
        }

        void onClose() override
        {
            if (activeTool() != kWidgetIndexNull)
                ToolCancel();
        }

        void onMouseUp(WidgetIndex widgetIndex) override
        {
            switch (widgetIndex)
            {
                case WIDX_CLOSE:
                    close();
                    return;
                case WIDX_TYPE_PREV:
                    _type = stepType(-1);
                    break;
                case WIDX_TYPE_NEXT:
                    _type = stepType(1);
                    break;
                case WIDX_FLOW:
                    _flow = DirectionNext(_flow);
                    break;
                case WIDX_BUILD:
                case WIDX_REMOVE:
                case WIDX_ENTRANCE:
                case WIDX_PARKING:
                case WIDX_REMOVE_PARKING:
                    if (isToolActive(*this, widgetIndex))
                        ToolCancel();
                    else
                        ToolSet(*this, widgetIndex, Tool::crosshair);
                    break;
            }
            invalidate();
        }

        void onToolDown(WidgetIndex widgetIndex, const ScreenCoordsXY& screenCoords) override
        {
            apply(widgetIndex, screenCoords);
            invalidate();
        }

        void onToolDrag(WidgetIndex widgetIndex, const ScreenCoordsXY& screenCoords) override
        {
            // Dragging lays or takes away a run of tiles; the entrance tool toggles only on the click.
            if (widgetIndex != WIDX_ENTRANCE)
                apply(widgetIndex, screenCoords);
            invalidate();
        }

        void onToolAbort(WidgetIndex widgetIndex) override
        {
            invalidate();
        }

        void onPrepareDraw() override
        {
            for (auto widget : { WIDX_BUILD, WIDX_REMOVE, WIDX_ENTRANCE, WIDX_PARKING, WIDX_REMOVE_PARKING })
                setWidgetPressed(widget, isToolActive(*this, widget));
            const auto* object = streetAt(_type);
            // The arrow gives the flow of a one-way street, and the edge a parking tile faces.
            setWidgetDisabled(WIDX_FLOW, object == nullptr);
        }

        void onDraw(RenderTarget& rt) override
        {
            drawWidgets(rt);
            auto text = [&](int32_t x, int32_t y, const std::string& s, Colour colour = Colour::black) {
                drawText(rt, windowPos + ScreenCoordsXY{ x, y }, s, { colour });
            };
            text(widgets[WIDX_TYPE_PREV].left + 6, widgets[WIDX_TYPE_PREV].top + 1, "<");
            text(widgets[WIDX_TYPE_NEXT].left + 6, widgets[WIDX_TYPE_NEXT].top + 1, ">");
            const auto* object = streetAt(_type);
            if (object == nullptr)
            {
                text(60, 23, "No street object loaded", Colour::grey);
                return;
            }
            text(60, 23, std::string(object->GetName()) + (object->OneWay ? " (one-way)" : ""));
            text(12, 114, "Speed limit: " + std::to_string(object->SpeedLimitKph) + " km/h", Colour::grey);
            text(130, 73, std::string(object->OneWay ? "Flow and parking face " : "Parking faces ") + kFlowNames[_flow & 3]);
            if (!_status.empty())
                text(12, 130, _status, Colour::brightRed);
        }
    };

    WindowBase* StreetToolOpen()
    {
        auto* windowMgr = GetWindowManager();
        return windowMgr->FocusOrCreate<StreetToolWindow>(WindowClass::streetTool, ScreenCoordsXY(32, 48), kWindowSize);
    }
} // namespace OpenRCT2::Ui::Windows
