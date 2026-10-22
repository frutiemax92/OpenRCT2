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
#include <openrct2/Context.h>
#include <openrct2-ui/interface/Widget.h>
#include <openrct2-ui/interface/Window.h>
#include <openrct2/GameState.h>
#include <openrct2/SpriteIds.h>
#include <openrct2/actions/GameActionRunner.h>
#include <openrct2/actions/golf/CourseBoundaryAction.h>
#include <openrct2/actions/golf/CourseEditAction.h>
#include <openrct2/actions/golf/HoleEditAction.h>
#include <openrct2/world/MapSelection.h>
#include <openrct2/drawing/Colour.h>
#include <openrct2/drawing/Drawing.Screen.h>
#include <openrct2/drawing/Drawing.h>
#include <openrct2/drawing/Line.h>
#include <openrct2/drawing/Rectangle.h>
#include <openrct2/drawing/RenderTarget.h>
#include <openrct2/drawing/Text.h>
#include <openrct2/golf/CourseManager.h>
#include <openrct2/golf/HoleDifficulty.h>
#include <openrct2/golf/HoleSchedule.h>
#include <openrct2/golf/MapBallWorld.h>
#include <openrct2/interface/Viewport.h>
#include <openrct2/localisation/Formatter.h>
#include <openrct2/localisation/StringIds.h>
#include <openrct2/ui/WindowManager.h>
#include <string>

namespace OpenRCT2::Ui::Windows
{
    using namespace Golf;
    using namespace GameActions;
    using namespace OpenRCT2::Drawing;

    static constexpr ScreenSize kWindowSize = { 460, 320 };
    static constexpr int32_t kScrollableRowHeight = 16;

    enum WindowCoursePage
    {
        WINDOW_COURSE_PAGE_HOLES,
        WINDOW_COURSE_PAGE_SCHEDULING,
        WINDOW_COURSE_PAGE_STATISTICS,
        WINDOW_COURSE_PAGE_COUNT,
    };

    enum WindowCourseWidgetIdx : WidgetIndex
    {
        WIDX_BACKGROUND,
        WIDX_TITLE,
        WIDX_CLOSE,
        WIDX_PAGE_BACKGROUND,
        WIDX_TAB_1,
        WIDX_TAB_2,
        WIDX_TAB_3,

        // Common to pages
        WIDX_NEW_MINI_PUTT,
        WIDX_BOUNDARY_TOOL,
        WIDX_COURSE_PREV,
        WIDX_COURSE_NEXT,

        // Holes page widgets
        WIDX_HOLE_LIST = 11,
        WIDX_TEST_HOLE,
        WIDX_MOVE_UP,
        WIDX_MOVE_DOWN,
        WIDX_PAR_LABEL,
        WIDX_PAR_SPINNER,
        WIDX_PAR_DOWN,
        WIDX_PAR_UP,

        // Scheduling page widgets
        WIDX_SCHED_LIST = 11,
        WIDX_MODE_TOGGLE,
        WIDX_SLOT_LABEL,
        WIDX_SLOT_SPINNER,
        WIDX_SLOT_DOWN,
        WIDX_SLOT_UP,

        // Statistics page widgets
        WIDX_OPEN_CLOSE = 11,
        WIDX_CARTS_OFF_PATH,
        WIDX_FEE_LABEL,
        WIDX_FEE_SPINNER,
        WIDX_FEE_DOWN,
        WIDX_FEE_UP,
        WIDX_STAT_TOURNAMENT,
        WIDX_STAT_ZONES,
        WIDX_STAT_CARTS,
    };

    // clang-format off
    static constexpr auto makeCourseWidgets = [](int16_t width) {
        return makeWidgets(
            makeWindowShim(STR_COURSE_WINDOW_TITLE, { width, kWindowSize.height }),
            makeWidget({   0, 43 }, { width, kWindowSize.height - 43 }, WidgetType::resize, WindowColour::secondary),
            makeTab   ({   3, 17 }, STR_COURSE_TAB_HOLES                                                            ),
            makeTab   ({  34, 17 }, STR_COURSE_TAB_SCHEDULING                                                       ),
            makeTab   ({  65, 17 }, STR_COURSE_TAB_STATISTICS                                                       ),
            makeWidget({ width - 180, 47 }, { 55, 14 }, WidgetType::button, WindowColour::secondary, kStringIdNone     ),
            makeWidget({ width - 120, 47 }, { 55, 14 }, WidgetType::button, WindowColour::secondary, kStringIdNone     ),
            makeWidget({ width - 55, 47 }, { 24, 14 }, WidgetType::button, WindowColour::secondary, kStringIdNone     ),
            makeWidget({ width - 28, 47 }, { 24, 14 }, WidgetType::button, WindowColour::secondary, kStringIdNone     )
        );
    };

    static constexpr auto _holesWidgets = makeWidgets(
        makeCourseWidgets(460),
        makeWidget({  10,  78 }, { 440, 160 }, WidgetType::scroll,  WindowColour::secondary, SCROLL_VERTICAL       ),
        makeWidget({  10, 244 }, {  80,  18 }, WidgetType::button,  WindowColour::secondary, STR_COURSE_TEST_HOLE, STR_COURSE_TEST_HOLE_TIP),
        makeWidget({  95, 244 }, {  55,  18 }, WidgetType::button,  WindowColour::secondary, STR_COURSE_MOVE_UP   ),
        makeWidget({ 155, 244 }, {  55,  18 }, WidgetType::button,  WindowColour::secondary, STR_COURSE_MOVE_DOWN ),
        makeWidget({ 230, 246 }, {  30,  14 }, WidgetType::label,   WindowColour::secondary, STR_COURSE_PAR       ),
        makeSpinnerWidgets({ 265, 244 }, { 60, 18 }, WidgetType::spinner, WindowColour::secondary                 )
    );

    static constexpr auto _schedulingWidgets = makeWidgets(
        makeCourseWidgets(460),
        makeWidget({  10,  78 }, { 440, 160 }, WidgetType::scroll,  WindowColour::secondary, SCROLL_VERTICAL       ),
        makeWidget({  10, 244 }, { 100,  18 }, WidgetType::button,  WindowColour::secondary, STR_COURSE_SLOT_MODE ),
        makeWidget({ 130, 246 }, {  60,  14 }, WidgetType::label,   WindowColour::secondary, STR_COURSE_SLOT_TIME ),
        makeSpinnerWidgets({ 195, 244 }, { 70, 18 }, WidgetType::spinner, WindowColour::secondary                 )
    );

    static constexpr auto _statisticsWidgets = makeWidgets(
        makeCourseWidgets(460),
        makeWidget({  10,  76 }, { 100,  18 }, WidgetType::button,   WindowColour::secondary, STR_COURSE_OPEN     ),
        makeWidget({ 120,  76 }, { 140,  18 }, WidgetType::checkbox, WindowColour::secondary, STR_COURSE_CARTS_OFF_PATH, STR_COURSE_CARTS_OFF_PATH_TIP),
        makeWidget({ 280,  78 }, {  70,  14 }, WidgetType::label,    WindowColour::secondary, STR_COURSE_GREEN_FEE),
        makeSpinnerWidgets({ 355, 76 }, { 85, 18 }, WidgetType::spinner, WindowColour::secondary                   ),
        makeWidget({  20, 260 }, { 120,  22 }, WidgetType::button,   WindowColour::secondary, STR_TOURNAMENT_TITLE ),
        makeWidget({ 160, 260 }, { 120,  22 }, WidgetType::button,   WindowColour::secondary, STR_ZONES_TITLE      ),
        makeWidget({ 300, 260 }, { 120,  22 }, WidgetType::button,   WindowColour::secondary, STR_CARTS_TITLE      )
    );
    // clang-format on

    static std::span<const Widget> _pagedWidgets[] = {
        _holesWidgets,
        _schedulingWidgets,
        _statisticsWidgets,
    };

    class CourseWindow final : public Window
    {
    private:
        size_t _courseIndex = 0;
        size_t _selectedHoleIndex = 0;
        std::string _statusMessage;

    public:
        void onOpen() override
        {
            setPage(WINDOW_COURSE_PAGE_HOLES);
        }

        void onClose() override
        {
            if (isToolActive(*this, WIDX_BOUNDARY_TOOL))
                ToolCancel();
        }

        void onPrepareDraw() override
        {
            setWidgetPressed(WIDX_TAB_1, page == WINDOW_COURSE_PAGE_HOLES);
            setWidgetPressed(WIDX_TAB_2, page == WINDOW_COURSE_PAGE_SCHEDULING);
            setWidgetPressed(WIDX_TAB_3, page == WINDOW_COURSE_PAGE_STATISTICS);
            setWidgetPressed(WIDX_BOUNDARY_TOOL, isToolActive(*this, WIDX_BOUNDARY_TOOL));
        }

        void selectCourse(uint32_t courseId)
        {
            auto& courses = getGameState().golf.courses();
            for (size_t i = 0; i < courses.size(); i++)
            {
                if (courses[i].id == courseId)
                {
                    _courseIndex = i;
                    _selectedHoleIndex = 0;
                    invalidate();
                    initScrollWidgets();
                    break;
                }
            }
        }

        Course* getSelectedCourse()
        {
            auto& courses = getGameState().golf.courses();
            if (courses.empty())
                return nullptr;
            if (_courseIndex >= courses.size())
                _courseIndex = 0;
            return &courses[_courseIndex];
        }

        void setPage(int32_t newPage)
        {
            page = newPage;
            setWidgets(_pagedWidgets[newPage]);
            setWidgetPressed(WIDX_TAB_1, page == WINDOW_COURSE_PAGE_HOLES);
            setWidgetPressed(WIDX_TAB_2, page == WINDOW_COURSE_PAGE_SCHEDULING);
            setWidgetPressed(WIDX_TAB_3, page == WINDOW_COURSE_PAGE_STATISTICS);
            invalidate();
            initScrollWidgets();
        }

        void initScrollWidgets()
        {
            auto* course = getSelectedCourse();
            size_t count = course != nullptr ? course->holes.size() : 0;
            scrolls[0].contentHeight = static_cast<int32_t>(count * kScrollableRowHeight);
            scrolls[0].contentWidth = 440;
        }

        void DrawTabImages(RenderTarget& rt)
        {
            GfxDrawSprite(rt, ImageId(SPR_TAB_PARK), windowPos + ScreenCoordsXY{ widgets[WIDX_TAB_1].left + 3, widgets[WIDX_TAB_1].top });
            GfxDrawSprite(rt, ImageId(SPR_TAB_GEARS_0), windowPos + ScreenCoordsXY{ widgets[WIDX_TAB_2].left + 3, widgets[WIDX_TAB_2].top });
            GfxDrawSprite(rt, ImageId(SPR_TAB_STATS), windowPos + ScreenCoordsXY{ widgets[WIDX_TAB_3].left + 3, widgets[WIDX_TAB_3].top });
        }

        void onDraw(RenderTarget& rt) override
        {
            drawWidgets(rt);
            DrawTabImages(rt);

            auto* course = getSelectedCourse();
            auto text = [&](int32_t x, int32_t y, const std::string& s, Colour col = Colour::black) {
                drawText(rt, windowPos + ScreenCoordsXY{ x, y }, s, { col });
            };

            // Course buttons
            text(widgets[WIDX_NEW_MINI_PUTT].left + 6, widgets[WIDX_NEW_MINI_PUTT].top + 1, "+Mini");
            text(widgets[WIDX_BOUNDARY_TOOL].left + 6, widgets[WIDX_BOUNDARY_TOOL].top + 1, "Bounds");
            text(widgets[WIDX_COURSE_PREV].left + 8, widgets[WIDX_COURSE_PREV].top + 1, "<");
            text(widgets[WIDX_COURSE_NEXT].left + 8, widgets[WIDX_COURSE_NEXT].top + 1, ">");

            // Course title bar / switcher
            if (course == nullptr)
            {
                text(14, 48, "No golf courses have been created yet.");
                return;
            }

            auto& courses = getGameState().golf.courses();
            std::string courseHeader = "Course " + std::to_string(_courseIndex + 1) + "/" +
                                       std::to_string(courses.size()) + ": " + course->name;
            if (course->kind == CourseKind::miniPutt)
                courseHeader += " (Mini-Putt)";
            text(14, 48, courseHeader, Colour::black);

            switch (page)
            {
                case WINDOW_COURSE_PAGE_HOLES:
                    drawHolesPage(rt, *course);
                    break;
                case WINDOW_COURSE_PAGE_SCHEDULING:
                    drawSchedulingPage(rt, *course);
                    break;
                case WINDOW_COURSE_PAGE_STATISTICS:
                    drawStatisticsPage(rt, *course);
                    break;
            }
        }

        void drawHolesPage(RenderTarget& rt, Course& course)
        {
            auto text = [&](int32_t x, int32_t y, const std::string& s, Colour col = Colour::black) {
                drawText(rt, windowPos + ScreenCoordsXY{ x, y }, s, { col });
            };

            // Column headers for holes list
            text(14, 65, "#", Colour::grey);
            text(38, 65, "Par", Colour::grey);
            text(70, 65, "Length (Yards)", Colour::grey);
            text(240, 65, "Difficulty", Colour::grey);
            text(315, 65, "Avg Score", Colour::grey);
            text(385, 65, "Avg Wait", Colour::grey);

            // Display current selected hole par in spinner
            if (_selectedHoleIndex < course.holes.size())
            {
                const auto& hole = course.holes[_selectedHoleIndex];
                text(275, 247, std::to_string(hole.par()));
            }

            // Status / Test result message
            if (!_statusMessage.empty())
            {
                text(12, 275, _statusMessage, Colour::darkGreen);
            }
            else if (_selectedHoleIndex < course.holes.size())
            {
                const auto& hole = course.holes[_selectedHoleIndex];
                std::string summary = "Hole " + std::to_string(_selectedHoleIndex + 1) + ": ";
                if (!hole.isComplete())
                    summary += "Incomplete (place tee and cup on map to complete)";
                else
                    summary += "Ready for play (Par " + std::to_string(hole.par()) + ")";
                text(12, 275, summary, Colour::grey);
            }
        }

        void drawSchedulingPage(RenderTarget& rt, Course& course)
        {
            auto text = [&](int32_t x, int32_t y, const std::string& s, Colour col = Colour::black) {
                drawText(rt, windowPos + ScreenCoordsXY{ x, y }, s, { col });
            };

            // Column headers for scheduling list
            text(14, 65, "#", Colour::grey);
            text(40, 65, "Par / Length", Colour::grey);
            text(150, 65, "Mode", Colour::grey);
            text(220, 65, "Slot Time", Colour::grey);
            text(300, 65, "Measured Play", Colour::grey);
            text(395, 65, "Status", Colour::grey);

            if (_selectedHoleIndex < course.holes.size())
            {
                const auto& hole = course.holes[_selectedHoleIndex];
                std::string modeStr = hole.schedule.manual ? "Manual" : "Auto";
                text(25, 247, "Mode: " + modeStr);
                text(205, 247, std::to_string(hole.schedule.manualSlotMinutes) + " mins");
            }
        }

        void drawStatisticsPage(RenderTarget& rt, Course& course)
        {
            auto text = [&](int32_t x, int32_t y, const std::string& s, Colour col = Colour::black) {
                drawText(rt, windowPos + ScreenCoordsXY{ x, y }, s, { col });
            };

            // Open/Closed button text
            text(40, 79, course.open ? "Open" : "Closed", course.open ? Colour::darkGreen : Colour::brightRed);

            // Carts off-path checkbox state
            setCheckboxValue(WIDX_CARTS_OFF_PATH, course.cartsAllowedOffPath);

            // Green fee spinner
            auto ft = Formatter();
            ft.Add<money64>(course.greenFee);
            StringId feeStr = (course.greenFee == 0) ? StringId(STR_FREE) : StringId(STR_CURRENCY2DP);
            drawText(rt, windowPos + ScreenCoordsXY{ 365, 79 }, feeStr, ft, { colours[1] });

            // Course Overview
            int32_t y = 110;
            text(20, y, "Course Overview", Colour::black);
            y += 20;

            const size_t playable = course.playableHoles();
            std::string holesSummary = std::to_string(playable) + " of " + std::to_string(course.holes.size()) +
                                       " holes complete (Par " + std::to_string(course.totalPar()) + ", " +
                                       std::to_string(course.totalYards(TeeCategory::men)) + " yards)";
            text(30, y, holesSummary);
            y += 20;

            // Rounds played
            text(30, y, "Rounds played: " + std::to_string(course.stats.roundsPlayed));
            y += 20;

            // Revenue
            ft = Formatter();
            ft.Add<money64>(course.stats.revenue);
            drawText(rt, windowPos + ScreenCoordsXY{ 30, y }, STR_COURSE_TOTAL_REVENUE, { Colour::black });
            drawText(rt, windowPos + ScreenCoordsXY{ 130, y }, STR_CURRENCY2DP, ft, { Colour::black });
            y += 20;

            // Satisfaction
            std::string satStr = "Golfer satisfaction: ";
            if (course.stats.satisfactionSamples == 0)
                satStr += "N/A";
            else
                satStr += std::to_string(course.stats.averageSatisfaction() * 100 / 255) + "%";
            text(30, y, satStr);
            y += 20;

            // Average wait across course
            int32_t totalWait = 0, holesWithData = 0;
            for (const auto& h : course.holes)
            {
                if (h.schedule.samples > 0)
                {
                    totalWait += h.schedule.measuredWaitTenths;
                    holesWithData++;
                }
            }
            std::string avgWaitStr = "Average wait per hole: ";
            if (holesWithData == 0)
                avgWaitStr += "No wait data yet";
            else
            {
                int32_t avgWait = totalWait / holesWithData;
                avgWaitStr += std::to_string(avgWait / 10) + "." + std::to_string(avgWait % 10) + " minutes";
            }
            text(30, y, avgWaitStr);
        }

        void onScrollDraw(int32_t scrollIndex, RenderTarget& rt) override
        {
            auto* course = getSelectedCourse();
            if (course == nullptr)
                return;

            MapBallWorld world;

            for (size_t i = 0; i < course->holes.size(); i++)
            {
                int32_t y = static_cast<int32_t>(i) * kScrollableRowHeight;
                if (y + kScrollableRowHeight < rt.y || y >= rt.y + rt.height)
                    continue;

                const auto& hole = course->holes[i];

                if (i == _selectedHoleIndex)
                {
                    Rectangle::filter(rt, { 0, y, rt.width, y + kScrollableRowHeight - 1 }, FilterPaletteID::paletteDarken1);
                }

                auto text = [&](int32_t x, const std::string& s, Colour col = Colour::black) {
                    drawText(rt, { x, y + 2 }, s, { col });
                };

                if (page == WINDOW_COURSE_PAGE_HOLES)
                {
                    // Hole number
                    text(4, std::to_string(i + 1));
                    // Par
                    text(28, std::to_string(hole.par()));

                    // Lengths
                    std::string lenStr;
                    if (hole.hasTee(TeeCategory::men))
                        lenStr += "M:" + std::to_string(hole.lengthYards(TeeCategory::men)) + " ";
                    if (hole.hasTee(TeeCategory::women))
                        lenStr += "W:" + std::to_string(hole.lengthYards(TeeCategory::women)) + " ";
                    if (hole.hasTee(TeeCategory::child))
                        lenStr += "C:" + std::to_string(hole.lengthYards(TeeCategory::child));
                    if (lenStr.empty())
                        lenStr = "-";
                    text(60, lenStr);

                    // Difficulty
                    std::string diffStr;
                    if (hole.estimatedDifficulty >= 0)
                    {
                        diffStr = std::to_string(hole.estimatedDifficulty / 10) + "." +
                                  std::to_string(hole.estimatedDifficulty % 10) + "*";
                    }
                    else if (hole.isComplete())
                    {
                        auto d = rateHole(hole, world);
                        diffStr = std::to_string(d.total / 10) + "." + std::to_string(d.total % 10);
                    }
                    else
                    {
                        diffStr = "-";
                    }
                    text(230, diffStr);

                    // Average score
                    std::string scoreStr = "-";
                    if (hole.stats.timesPlayed > 0)
                    {
                        uint32_t tenths = (hole.stats.totalStrokes * 10) / hole.stats.timesPlayed;
                        scoreStr = std::to_string(tenths / 10) + "." + std::to_string(tenths % 10);
                    }
                    text(305, scoreStr);

                    // Average wait
                    std::string waitStr = "-";
                    if (hole.schedule.samples > 0)
                    {
                        waitStr = std::to_string(hole.schedule.measuredWaitTenths / 10) + "." +
                                  std::to_string(hole.schedule.measuredWaitTenths % 10) + "m";
                    }
                    text(375, waitStr);
                }
                else if (page == WINDOW_COURSE_PAGE_SCHEDULING)
                {
                    int32_t diff = hole.isComplete() ? rateHole(hole, world).total : 0;
                    bool bottleneck = isBottleneck(hole, diff);
                    Colour col = bottleneck ? Colour::brightRed : Colour::black;

                    // Hole number
                    text(4, std::to_string(i + 1), col);

                    // Par & yards
                    std::string parYards = "Par " + std::to_string(hole.par()) + " (" +
                                           std::to_string(hole.lengthYards(TeeCategory::men)) + "y)";
                    text(30, parYards, col);

                    // Mode
                    text(140, hole.schedule.manual ? "Manual" : "Auto", col);

                    // Slot Time
                    text(210, std::to_string(slotMinutes(hole, diff)) + " mins", col);

                    // Measured Play
                    std::string playStr = "-";
                    if (hole.schedule.samples > 0)
                    {
                        playStr = std::to_string(hole.schedule.measuredPlayTenths / 10) + "." +
                                  std::to_string(hole.schedule.measuredPlayTenths % 10) + " mins";
                    }
                    text(290, playStr, col);

                    // Status
                    text(385, bottleneck ? "Bottleneck" : "Normal", col);
                }
            }
        }

        void onScrollMouseDown(int32_t scrollIndex, const ScreenCoordsXY& screenCoords) override
        {
            size_t index = screenCoords.y / kScrollableRowHeight;
            auto* course = getSelectedCourse();
            if (course != nullptr && index < course->holes.size())
            {
                _selectedHoleIndex = index;
                invalidate();
            }
        }

        void onMouseUp(WidgetIndex widgetIndex) override
        {
            switch (widgetIndex)
            {
                case WIDX_CLOSE:
                    close();
                    return;
                case WIDX_TAB_1:
                case WIDX_TAB_2:
                case WIDX_TAB_3:
                    setPage(widgetIndex - WIDX_TAB_1);
                    return;
                case WIDX_COURSE_PREV:
                {
                    auto& courses = getGameState().golf.courses();
                    if (!courses.empty())
                    {
                        _courseIndex = (_courseIndex + courses.size() - 1) % courses.size();
                        _selectedHoleIndex = 0;
                        _statusMessage.clear();
                        invalidate();
                        initScrollWidgets();
                    }
                    return;
                }
                case WIDX_COURSE_NEXT:
                {
                    auto& courses = getGameState().golf.courses();
                    if (!courses.empty())
                    {
                        _courseIndex = (_courseIndex + 1) % courses.size();
                        _selectedHoleIndex = 0;
                        _statusMessage.clear();
                        invalidate();
                        initScrollWidgets();
                    }
                    return;
                }
                case WIDX_NEW_MINI_PUTT:
                {
                    auto& gameState = getGameState();
                    CourseEditAction action(
                        CourseEditType::create, 0, "Mini-Putt " + std::to_string(gameState.golf.courses().size() + 1), 1);
                    const auto result = Execute(&action, gameState);
                    if (result.error == Status::ok)
                        selectCourse(result.getData<CourseEditResult>().courseId);
                    else
                        _statusMessage = result.getErrorMessage();
                    invalidate();
                    return;
                }
                case WIDX_BOUNDARY_TOOL:
                    if (isToolActive(*this, WIDX_BOUNDARY_TOOL))
                        ToolCancel();
                    else
                        ToolSet(*this, WIDX_BOUNDARY_TOOL, Tool::crosshair);
                    invalidate();
                    return;
            }

            auto* course = getSelectedCourse();
            if (course == nullptr)
                return;

            switch (page)
            {
                case WINDOW_COURSE_PAGE_HOLES:
                    onMouseUpHoles(widgetIndex, *course);
                    break;
                case WINDOW_COURSE_PAGE_SCHEDULING:
                    onMouseUpScheduling(widgetIndex, *course);
                    break;
                case WINDOW_COURSE_PAGE_STATISTICS:
                    onMouseUpStatistics(widgetIndex, *course);
                    break;
            }
        }

        void onMouseUpHoles(WidgetIndex widgetIndex, Course& course)
        {
            switch (widgetIndex)
            {
                case WIDX_TEST_HOLE:
                {
                    if (_selectedHoleIndex < course.holes.size())
                    {
                        auto& hole = course.holes[_selectedHoleIndex];
                        MapBallWorld world;
                        auto res = testHole(hole, world);
                        if (res.reachedCup || res.strokes > 0)
                        {
                            hole.estimatedDifficulty = res.estimatedDifficulty;
                            _statusMessage = "Test Hole " + std::to_string(_selectedHoleIndex + 1) + ": " +
                                             std::to_string(res.strokes) + " strokes (Par " + std::to_string(res.par) +
                                             ") - Difficulty " + std::to_string(res.estimatedDifficulty / 10) + "." +
                                             std::to_string(res.estimatedDifficulty % 10);
                        }
                        else
                        {
                            _statusMessage = "Hole " + std::to_string(_selectedHoleIndex + 1) + " incomplete (needs tee and cup)";
                        }
                        invalidate();
                    }
                    break;
                }
                case WIDX_MOVE_UP:
                {
                    if (_selectedHoleIndex > 0 && _selectedHoleIndex < course.holes.size())
                    {
                        CourseEditAction action(CourseEditType::moveHole, course.id, {}, _selectedHoleIndex, _selectedHoleIndex - 1);
                        GameActions::Execute(&action, getGameState());
                        _selectedHoleIndex--;
                        invalidate();
                        initScrollWidgets();
                    }
                    break;
                }
                case WIDX_MOVE_DOWN:
                {
                    if (_selectedHoleIndex + 1 < course.holes.size())
                    {
                        CourseEditAction action(CourseEditType::moveHole, course.id, {}, _selectedHoleIndex, _selectedHoleIndex + 1);
                        GameActions::Execute(&action, getGameState());
                        _selectedHoleIndex++;
                        invalidate();
                        initScrollWidgets();
                    }
                    break;
                }
                case WIDX_PAR_DOWN:
                {
                    if (_selectedHoleIndex < course.holes.size())
                    {
                        auto& hole = course.holes[_selectedHoleIndex];
                        int32_t currentPar = hole.par();
                        if (currentPar > 3)
                        {
                            HoleEditAction action(course.id, hole.id, HoleEditType::setPar, currentPar - 1);
                            GameActions::Execute(&action, getGameState());
                            invalidate();
                        }
                    }
                    break;
                }
                case WIDX_PAR_UP:
                {
                    if (_selectedHoleIndex < course.holes.size())
                    {
                        auto& hole = course.holes[_selectedHoleIndex];
                        int32_t currentPar = hole.par();
                        if (currentPar < 6)
                        {
                            HoleEditAction action(course.id, hole.id, HoleEditType::setPar, currentPar + 1);
                            GameActions::Execute(&action, getGameState());
                            invalidate();
                        }
                    }
                    break;
                }
            }
        }

        void onMouseUpScheduling(WidgetIndex widgetIndex, Course& course)
        {
            switch (widgetIndex)
            {
                case WIDX_MODE_TOGGLE:
                {
                    if (_selectedHoleIndex < course.holes.size())
                    {
                        auto& hole = course.holes[_selectedHoleIndex];
                        if (hole.schedule.manual)
                        {
                            HoleEditAction action(course.id, hole.id, HoleEditType::setSlot, 0);
                            GameActions::Execute(&action, getGameState());
                        }
                        else
                        {
                            int32_t slot = hole.schedule.manualSlotMinutes > 0 ? hole.schedule.manualSlotMinutes : kDefaultManualSlotMinutes;
                            HoleEditAction action(course.id, hole.id, HoleEditType::setSlot, slot);
                            GameActions::Execute(&action, getGameState());
                        }
                        invalidate();
                    }
                    break;
                }
                case WIDX_SLOT_DOWN:
                {
                    if (_selectedHoleIndex < course.holes.size())
                    {
                        auto& hole = course.holes[_selectedHoleIndex];
                        int32_t currentSlot = hole.schedule.manualSlotMinutes;
                        if (currentSlot > kMinSlotMinutes)
                        {
                            HoleEditAction action(course.id, hole.id, HoleEditType::setSlot, currentSlot - 1);
                            GameActions::Execute(&action, getGameState());
                            invalidate();
                        }
                    }
                    break;
                }
                case WIDX_SLOT_UP:
                {
                    if (_selectedHoleIndex < course.holes.size())
                    {
                        auto& hole = course.holes[_selectedHoleIndex];
                        int32_t currentSlot = hole.schedule.manualSlotMinutes;
                        if (currentSlot < kMaxSlotMinutes)
                        {
                            HoleEditAction action(course.id, hole.id, HoleEditType::setSlot, currentSlot + 1);
                            GameActions::Execute(&action, getGameState());
                            invalidate();
                        }
                    }
                    break;
                }
            }
        }

        void onMouseUpStatistics(WidgetIndex widgetIndex, Course& course)
        {
            switch (widgetIndex)
            {
                case WIDX_OPEN_CLOSE:
                {
                    CourseEditAction action(CourseEditType::setOpen, course.id, {}, course.open ? 0 : 1);
                    GameActions::Execute(&action, getGameState());
                    invalidate();
                    break;
                }
                case WIDX_CARTS_OFF_PATH:
                {
                    CourseEditAction action(CourseEditType::setCartsAllowedOffPath, course.id, {}, course.cartsAllowedOffPath ? 0 : 1);
                    GameActions::Execute(&action, getGameState());
                    invalidate();
                    break;
                }
                case WIDX_FEE_DOWN:
                {
                    if (course.greenFee >= 500)
                    {
                        CourseEditAction action(CourseEditType::setFee, course.id, {}, course.greenFee - 500);
                        GameActions::Execute(&action, getGameState());
                        invalidate();
                    }
                    break;
                }
                case WIDX_FEE_UP:
                {
                    CourseEditAction action(CourseEditType::setFee, course.id, {}, course.greenFee + 500);
                    GameActions::Execute(&action, getGameState());
                    invalidate();
                    break;
                }
                case WIDX_STAT_TOURNAMENT:
                    ContextOpenWindow(WindowClass::tournament);
                    break;
                case WIDX_STAT_ZONES:
                    ContextOpenWindow(WindowClass::zones);
                    break;
                case WIDX_STAT_CARTS:
                    ContextOpenWindow(WindowClass::carts);
                    break;
            }
        }

        void onToolDown(WidgetIndex widgetIndex, const ScreenCoordsXY& screenCoords) override
        {
            if (widgetIndex == WIDX_BOUNDARY_TOOL)
            {
                const auto position = ScreenGetMapXY(screenCoords, nullptr);
                if (position.has_value())
                {
                    const auto tile = position->toTileStart();
                    gMapSelectPositionA = tile;
                    gMapSelectPositionB = tile;
                    gMapSelectFlags.set(MapSelectFlag::enable);
                    gMapSelectType = MapSelectType::full;
                }
            }
        }

        void onToolDrag(WidgetIndex widgetIndex, const ScreenCoordsXY& screenCoords) override
        {
            if (widgetIndex == WIDX_BOUNDARY_TOOL)
            {
                const auto position = ScreenGetMapXY(screenCoords, nullptr);
                if (position.has_value())
                {
                    const auto tile = position->toTileStart();
                    gMapSelectPositionB = tile;
                    gMapSelectFlags.set(MapSelectFlag::enable);
                    gMapSelectType = MapSelectType::full;
                }
            }
        }

        void onToolUp(WidgetIndex widgetIndex, const ScreenCoordsXY& screenCoords) override
        {
            if (widgetIndex == WIDX_BOUNDARY_TOOL)
            {
                const auto minX = std::min(gMapSelectPositionA.x, gMapSelectPositionB.x);
                const auto minY = std::min(gMapSelectPositionA.y, gMapSelectPositionB.y);
                const auto maxX = std::max(gMapSelectPositionA.x, gMapSelectPositionB.x);
                const auto maxY = std::max(gMapSelectPositionA.y, gMapSelectPositionB.y);

                CourseBoundaryAction action({ minX, minY }, { maxX, maxY }, CourseBoundaryAction::kInPlay);
                Execute(&action, getGameState());
                gMapSelectFlags.unset(MapSelectFlag::enable);
                invalidate();
            }
        }

        void onToolAbort(WidgetIndex widgetIndex) override
        {
            if (widgetIndex == WIDX_BOUNDARY_TOOL)
            {
                gMapSelectFlags.unset(MapSelectFlag::enable);
                invalidate();
            }
        }
    };

    WindowBase* CourseOpen(uint32_t courseId)
    {
        auto* windowMgr = Ui::GetWindowManager();
        auto* window = windowMgr->FocusOrCreate<CourseWindow>(WindowClass::course, ScreenCoordsXY(64, 48), kWindowSize);
        if (window != nullptr && courseId != 0)
        {
            window->selectCourse(courseId);
        }
        return window;
    }
} // namespace OpenRCT2::Ui::Windows

