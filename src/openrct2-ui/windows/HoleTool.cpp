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
#include <openrct2/actions/golf/CourseEditAction.h>
#include <openrct2/actions/golf/HoleEditAction.h>
#include <openrct2/drawing/Colour.h>
#include <openrct2/drawing/Drawing.Screen.h>
#include <openrct2/drawing/Drawing.h>
#include <openrct2/drawing/RenderTarget.h>
#include <openrct2/drawing/Text.h>
#include <openrct2/golf/CourseManager.h>
#include <openrct2/golf/HoleMarkers.h>
#include <openrct2/interface/Viewport.h>
#include <openrct2/localisation/StringIds.h>
#include <openrct2/ui/WindowManager.h>
#include <string>

namespace OpenRCT2::Ui::Windows
{
    using namespace Golf;
    using namespace GameActions;
    using namespace OpenRCT2::Drawing;

    // The hole construction flow (HOL-001): pick a course and a hole, then place the men's tee, the optional women's and
    // child's tees, the cup, and up to two aim points by clicking the map.
    enum WindowHoleToolWidgetIdx : WidgetIndex
    {
        WIDX_BACKGROUND,
        WIDX_TITLE,
        WIDX_CLOSE,
        WIDX_COURSE_PREV,
        WIDX_COURSE_NEXT,
        WIDX_NEW_COURSE,
        WIDX_HOLE_PREV,
        WIDX_HOLE_NEXT,
        WIDX_NEW_HOLE,
        WIDX_MEN_TEE,
        WIDX_WOMEN_TEE,
        WIDX_CHILD_TEE,
        WIDX_CUP,
        WIDX_AIM_1,
        WIDX_AIM_2,
        WIDX_REMOVE,
        WIDX_OPEN_COURSE,
        WIDX_OPEN_ZONES,
    };

    static constexpr ScreenSize kWindowSize = { 280, 224 };
    static constexpr int32_t kFirstMarkerWidget = WIDX_MEN_TEE;
    static constexpr int32_t kMarkerCount = 6;

    // clang-format off
    static constexpr auto kHoleToolWidgets = makeWidgets(
        makeWindowShim(STR_HOLE_TOOL_TITLE, kWindowSize),
        makeWidget({  10, 20 }, {  20, 14 }, WidgetType::button, WindowColour::secondary, kStringIdNone),
        makeWidget({  34, 20 }, {  20, 14 }, WidgetType::button, WindowColour::secondary, kStringIdNone),
        makeWidget({ 180, 20 }, {  90, 14 }, WidgetType::button, WindowColour::secondary, STR_HOLE_TOOL_NEW_COURSE),
        makeWidget({  10, 40 }, {  20, 14 }, WidgetType::button, WindowColour::secondary, kStringIdNone),
        makeWidget({  34, 40 }, {  20, 14 }, WidgetType::button, WindowColour::secondary, kStringIdNone),
        makeWidget({ 180, 40 }, {  90, 14 }, WidgetType::button, WindowColour::secondary, STR_HOLE_TOOL_NEW_HOLE),
        makeWidget({  10, 64 }, {  84, 18 }, WidgetType::button, WindowColour::secondary, STR_HOLE_TOOL_MEN_TEE),
        makeWidget({  98, 64 }, {  84, 18 }, WidgetType::button, WindowColour::secondary, STR_HOLE_TOOL_WOMEN_TEE),
        makeWidget({ 186, 64 }, {  84, 18 }, WidgetType::button, WindowColour::secondary, STR_HOLE_TOOL_CHILD_TEE),
        makeWidget({  10, 86 }, {  84, 18 }, WidgetType::button, WindowColour::secondary, STR_HOLE_TOOL_CUP),
        makeWidget({  98, 86 }, {  84, 18 }, WidgetType::button, WindowColour::secondary, STR_HOLE_TOOL_AIM_1),
        makeWidget({ 186, 86 }, {  84, 18 }, WidgetType::button, WindowColour::secondary, STR_HOLE_TOOL_AIM_2),
        makeWidget({  10, 110 }, { 120, 12 }, WidgetType::checkbox, WindowColour::secondary, STR_HOLE_TOOL_REMOVE, STR_HOLE_TOOL_REMOVE_TIP),
        makeWidget({  10, 196 }, { 125, 20 }, WidgetType::button, WindowColour::secondary, STR_COURSE_WINDOW_TITLE),
        makeWidget({ 145, 196 }, { 125, 20 }, WidgetType::button, WindowColour::secondary, STR_ZONES_TITLE)
    );
    // clang-format on

    class HoleToolWindow final : public Window
    {
    private:
        size_t _courseIndex = 0;
        size_t _holeIndex = 0;
        bool _remove = false;
        std::string _status;

        Course* course()
        {
            auto& courses = getGameState().golf.courses();
            if (courses.empty())
                return nullptr;
            _courseIndex = std::min(_courseIndex, courses.size() - 1);
            return &courses[_courseIndex];
        }

        Hole* hole()
        {
            auto* c = course();
            if (c == nullptr || c->holes.empty())
                return nullptr;
            _holeIndex = std::min(_holeIndex, c->holes.size() - 1);
            return &c->holes[_holeIndex];
        }

        // Which marker a button places: the tee category, the cup or the aim point.
        struct Marker
        {
            HoleEditType place;
            HoleEditType remove;
            int32_t index;
        };
        static Marker markerFor(WidgetIndex widgetIndex)
        {
            switch (widgetIndex)
            {
                case WIDX_MEN_TEE:
                    return { HoleEditType::placeTee, HoleEditType::removeTee, 0 };
                case WIDX_WOMEN_TEE:
                    return { HoleEditType::placeTee, HoleEditType::removeTee, 1 };
                case WIDX_CHILD_TEE:
                    return { HoleEditType::placeTee, HoleEditType::removeTee, 2 };
                case WIDX_CUP:
                    return { HoleEditType::placeCup, HoleEditType::removeCup, 0 };
                case WIDX_AIM_1:
                    return { HoleEditType::placeAim, HoleEditType::removeAim, 0 };
                default:
                    return { HoleEditType::placeAim, HoleEditType::removeAim, 1 };
            }
        }

        WidgetIndex activeMarkerWidget() const
        {
            for (int32_t i = 0; i < kMarkerCount; i++)
                if (isToolActive(*this, kFirstMarkerWidget + i))
                    return kFirstMarkerWidget + i;
            return kWidgetIndexNull;
        }

    public:
        void onOpen() override
        {
            setWidgets(kHoleToolWidgets);
            // The aim points are only drawn while holes are being built.
            GetHoleMarkers().setShowAimPoints(true);
        }

        void onClose() override
        {
            GetHoleMarkers().setShowAimPoints(false);
            if (activeMarkerWidget() != kWidgetIndexNull)
                ToolCancel();
        }

        void selectCourse(size_t index)
        {
            _courseIndex = index;
            _holeIndex = 0;
            _status.clear();
        }

        void onMouseUp(WidgetIndex widgetIndex) override
        {
            auto& gameState = getGameState();
            switch (widgetIndex)
            {
                case WIDX_CLOSE:
                    close();
                    return;
                case WIDX_COURSE_PREV:
                case WIDX_COURSE_NEXT:
                {
                    const auto count = gameState.golf.courses().size();
                    if (count > 0)
                    {
                        const auto step = widgetIndex == WIDX_COURSE_NEXT ? 1 : count - 1;
                        selectCourse((_courseIndex + step) % count);
                    }
                    break;
                }
                case WIDX_NEW_COURSE:
                {
                    CourseEditAction action(
                        CourseEditType::create, 0, "Course " + std::to_string(gameState.golf.courses().size() + 1));
                    const auto result = Execute(&action, gameState);
                    if (result.error == Status::ok)
                        selectCourse(gameState.golf.courses().size() - 1);
                    else
                        _status = result.getErrorMessage();
                    break;
                }
                case WIDX_HOLE_PREV:
                case WIDX_HOLE_NEXT:
                {
                    if (auto* c = course(); c != nullptr && !c->holes.empty())
                    {
                        const auto step = widgetIndex == WIDX_HOLE_NEXT ? 1 : c->holes.size() - 1;
                        _holeIndex = (_holeIndex + step) % c->holes.size();
                    }
                    break;
                }
                case WIDX_NEW_HOLE:
                {
                    if (auto* c = course())
                    {
                        CourseEditAction action(CourseEditType::createHole, c->id);
                        const auto result = Execute(&action, gameState);
                        if (result.error == Status::ok)
                            _holeIndex = course()->holes.size() - 1;
                        else
                            _status = result.getErrorMessage();
                    }
                    break;
                }
                case WIDX_REMOVE:
                    _remove = !_remove;
                    break;
                case WIDX_OPEN_COURSE:
                    ContextOpenWindow(WindowClass::course);
                    break;
                case WIDX_OPEN_ZONES:
                    ContextOpenWindow(WindowClass::zones);
                    break;
                case WIDX_MEN_TEE:
                case WIDX_WOMEN_TEE:
                case WIDX_CHILD_TEE:
                case WIDX_CUP:
                case WIDX_AIM_1:
                case WIDX_AIM_2:
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
            if (widgetIndex < kFirstMarkerWidget || widgetIndex >= kFirstMarkerWidget + kMarkerCount)
                return;
            auto* c = course();
            auto* h = hole();
            if (c == nullptr || h == nullptr)
            {
                _status = "Create a course and a hole first.";
                invalidate();
                return;
            }
            const auto position = ScreenGetMapXY(screenCoords, nullptr);
            if (!position.has_value())
                return;
            const auto marker = markerFor(widgetIndex);
            HoleEditAction action(c->id, h->id, _remove ? marker.remove : marker.place, marker.index, *position);
            const auto result = Execute(&action, getGameState());
            _status = result.error == Status::ok ? std::string() : result.getErrorMessage();
            invalidate();
        }

        void onToolAbort(WidgetIndex widgetIndex) override
        {
            invalidate();
        }

        void onPrepareDraw() override
        {
            for (int32_t i = 0; i < kMarkerCount; i++)
                setWidgetPressed(kFirstMarkerWidget + i, isToolActive(*this, kFirstMarkerWidget + i));
            setCheckboxValue(WIDX_REMOVE, _remove);
        }

        void onDraw(RenderTarget& rt) override
        {
            drawWidgets(rt);
            auto text = [&](int32_t x, int32_t y, const std::string& s, Colour colour = Colour::black) {
                drawText(rt, windowPos + ScreenCoordsXY{ x, y }, s, { colour });
            };
            text(widgets[WIDX_COURSE_PREV].left + 6, widgets[WIDX_COURSE_PREV].top + 1, "<");
            text(widgets[WIDX_COURSE_NEXT].left + 6, widgets[WIDX_COURSE_NEXT].top + 1, ">");
            text(widgets[WIDX_HOLE_PREV].left + 6, widgets[WIDX_HOLE_PREV].top + 1, "<");
            text(widgets[WIDX_HOLE_NEXT].left + 6, widgets[WIDX_HOLE_NEXT].top + 1, ">");

            auto* c = course();
            if (c == nullptr)
            {
                text(60, 23, "No course yet", Colour::grey);
                return;
            }
            text(60, 23, c->name);
            auto* h = hole();
            if (h == nullptr)
            {
                text(60, 43, "No holes yet", Colour::grey);
            }
            else
            {
                text(60, 43, "Hole " + std::to_string(_holeIndex + 1) + " of " + std::to_string(c->holes.size()));

                // What the hole has, and what it measures (HOL-005, HOL-006).
                int32_t y = 128;
                auto have = [&](const char* name, bool present) {
                    text(12, y, std::string(name) + (present ? ": placed" : ": missing"), present ? Colour::darkGreen : Colour::brightRed);
                    y += 12;
                };
                have("Men's tee", h->hasTee(TeeCategory::men));
                have("Cup", h->cup.has_value());
                if (h->isComplete())
                {
                    text(
                        12, y,
                        "Length " + std::to_string(h->lengthYards(h->firstTee().value())) + " yd, par " + std::to_string(h->par()));
                }
            }
            if (!_status.empty())
                text(12, 178, _status, Colour::brightRed);
        }
    };

    WindowBase* HoleToolOpen()
    {
        auto* windowMgr = GetWindowManager();
        return windowMgr->FocusOrCreate<HoleToolWindow>(WindowClass::holeTool, ScreenCoordsXY(32, 48), kWindowSize);
    }
} // namespace OpenRCT2::Ui::Windows
