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
#include <openrct2/GameState.h>
#include <openrct2/SpriteIds.h>
#include <openrct2/drawing/Colour.h>
#include <openrct2/drawing/Drawing.Screen.h>
#include <openrct2/drawing/Drawing.h>
#include <openrct2/drawing/Rectangle.h>
#include <openrct2/drawing/RenderTarget.h>
#include <openrct2/drawing/Text.h>
#include <openrct2/entity/EntityRegistry.h>
#include <openrct2/entity/GolferEntity.h>
#include <openrct2/golf/CourseManager.h>
#include <openrct2/golf/GolfRuntime.h>
#include <openrct2/localisation/Formatter.h>
#include <openrct2/localisation/StringIds.h>
#include <openrct2/ui/WindowManager.h>
#include <string>

namespace OpenRCT2::Ui::Windows
{
    using namespace Golf;
    using namespace OpenRCT2::Drawing;

    static constexpr ScreenSize kWindowSize = { 240, 200 };

    // Section 13.7: the pages of the golfer window, as in the guest window.
    enum WindowGolferPage
    {
        WINDOW_GOLFER_PAGE_OVERVIEW,
        WINDOW_GOLFER_PAGE_NEEDS,
        WINDOW_GOLFER_PAGE_SKILLS,
        WINDOW_GOLFER_PAGE_SCORECARD,
        WINDOW_GOLFER_PAGE_THOUGHTS,
        WINDOW_GOLFER_PAGE_MONEY,
        WINDOW_GOLFER_PAGE_COUNT,
    };

    enum WindowGolferWidgetIdx : WidgetIndex
    {
        WIDX_BACKGROUND,
        WIDX_TITLE,
        WIDX_CLOSE,
        WIDX_PAGE_BACKGROUND,
        WIDX_TAB_1,
        WIDX_TAB_2,
        WIDX_TAB_3,
        WIDX_TAB_4,
        WIDX_TAB_5,
        WIDX_TAB_6,
    };

    // clang-format off
    static constexpr auto _golferWidgets = makeWidgets(
        makeWindowShim(STR_GOLFER_WINDOW_TITLE, kWindowSize),
        makeWidget({   0, 43 }, { kWindowSize.width, kWindowSize.height - 43 }, WidgetType::resize, WindowColour::secondary),
        makeTab   ({   3, 17 }, STR_GOLFER_TAB_OVERVIEW  ),
        makeTab   ({  34, 17 }, STR_GOLFER_TAB_NEEDS     ),
        makeTab   ({  65, 17 }, STR_GOLFER_TAB_SKILLS    ),
        makeTab   ({  96, 17 }, STR_GOLFER_TAB_SCORECARD ),
        makeTab   ({ 127, 17 }, STR_GOLFER_TAB_THOUGHTS  ),
        makeTab   ({ 158, 17 }, STR_GOLFER_TAB_MONEY     )
    );
    // clang-format on

    static constexpr int32_t kBarLeft = 80;
    static constexpr int32_t kBarWidth = 140;
    static constexpr int32_t kRowHeight = 12;

    class GolferWindow final : public Window
    {
    public:
        void onOpen() override
        {
            setWidgets(_golferWidgets);
            setPage(WINDOW_GOLFER_PAGE_OVERVIEW);
        }

        void setPage(int32_t newPage)
        {
            page = newPage;
            for (int32_t i = 0; i < WINDOW_GOLFER_PAGE_COUNT; i++)
                setWidgetPressed(WIDX_TAB_1 + i, page == i);
            invalidate();
        }

        void onMouseUp(WidgetIndex widgetIndex) override
        {
            switch (widgetIndex)
            {
                case WIDX_CLOSE:
                    close();
                    break;
                case WIDX_TAB_1:
                case WIDX_TAB_2:
                case WIDX_TAB_3:
                case WIDX_TAB_4:
                case WIDX_TAB_5:
                case WIDX_TAB_6:
                    setPage(widgetIndex - WIDX_TAB_1);
                    break;
            }
        }

        void onUpdate() override
        {
            // The golfer moves and plays on: redraw every few ticks.
            invalidate();
            if (findGolfer() == nullptr && findEntity() == nullptr)
                close();
        }

        void onDraw(RenderTarget& rt) override
        {
            drawWidgets(rt);
            drawTabImages(rt);

            const auto* golfer = findGolfer();
            if (golfer == nullptr)
            {
                text(rt, 12, 52, "This golfer has left the course.", Colour::grey);
                return;
            }

            text(rt, 12, 48, golfer->name + (golfer->vip ? " (VIP)" : ""), Colour::black);
            text(rt, 12, 60, golferStateName(golfer->state), Colour::grey);

            switch (page)
            {
                case WINDOW_GOLFER_PAGE_OVERVIEW:
                    drawOverview(rt, *golfer);
                    break;
                case WINDOW_GOLFER_PAGE_NEEDS:
                    drawNeeds(rt, *golfer);
                    break;
                case WINDOW_GOLFER_PAGE_SKILLS:
                    drawSkills(rt, *golfer);
                    break;
                case WINDOW_GOLFER_PAGE_SCORECARD:
                    drawScorecard(rt, *golfer);
                    break;
                case WINDOW_GOLFER_PAGE_THOUGHTS:
                    drawThoughts(rt, *golfer);
                    break;
                case WINDOW_GOLFER_PAGE_MONEY:
                    drawMoney(rt, *golfer);
                    break;
            }
        }

    private:
        const GolferEntity* findEntity() const
        {
            return getGameState().entities.getEntity<GolferEntity>(EntityId::FromUnderlying(number));
        }

        const Golfer* findGolfer() const
        {
            const auto* entity = findEntity();
            const auto& runtime = getGameState().golfRuntime;
            if (entity == nullptr || runtime == nullptr)
                return nullptr;
            return const_cast<GolfSimulation&>(runtime->director().simulation()).find(entity->golfer);
        }

        void text(RenderTarget& rt, int32_t x, int32_t y, const std::string& s, Colour colour = Colour::black)
        {
            drawText(rt, windowPos + ScreenCoordsXY{ x, y }, s, { colour });
        }

        void bar(RenderTarget& rt, int32_t y, const std::string& label, int32_t value, int32_t maximum, Colour colour)
        {
            text(rt, 12, y, label);
            const int32_t filled = std::clamp(value, 0, maximum) * kBarWidth / std::max(maximum, 1);
            const auto left = windowPos.x + kBarLeft;
            const auto top = windowPos.y + y;
            Rectangle::fillInset(
                rt, { { left, top }, { left + kBarWidth - 1, top + 8 } }, colours[1], Rectangle::BorderStyle::inset);
            if (filled > 0)
                Rectangle::fillInset(rt, { { left, top }, { left + filled - 1, top + 8 } }, ColourWithFlags{ colour }, Rectangle::BorderStyle::outset);
        }

        void drawOverview(RenderTarget& rt, const Golfer& golfer)
        {
            int32_t y = 80;
            const auto& courses = getGameState().golf.courses();
            const Course* course = nullptr;
            for (const auto& c : courses)
                course = c.id == golfer.courseId ? &c : course;
            if (course != nullptr)
            {
                text(rt, 12, y, "Course: " + course->name);
                y += kRowHeight;
                text(
                    rt, 12, y,
                    "Hole " + std::to_string(golfer.holeIndex + 1) + " of " + std::to_string(course->holes.size()));
                y += kRowHeight;
            }
            text(rt, 12, y, "Strokes on this hole: " + std::to_string(golfer.score.strokes()) + " (par " + std::to_string(golfer.score.par) + ")");
            y += kRowHeight;
            const int32_t toPar = golfer.toParSoFar();
            text(rt, 12, y, "Round so far: " + std::string(toPar > 0 ? "+" : "") + std::to_string(toPar));
            y += kRowHeight;
            text(rt, 12, y, "Group: " + std::to_string(golfer.groupId));
            y += kRowHeight;
            text(rt, 12, y, "Equipment level: " + std::to_string(golfer.equipmentLevel));
        }

        void drawNeeds(RenderTarget& rt, const Golfer& golfer)
        {
            // GLF-010: the needs are bytes. A full bar is good for happiness and energy and bad for the others.
            int32_t y = 80;
            bar(rt, y, "Happiness", golfer.needs.happinessByte(), 255, Colour::brightGreen);
            y += kRowHeight + 4;
            bar(rt, y, "Energy", golfer.needs.energyByte(), 255, Colour::brightGreen);
            y += kRowHeight + 4;
            bar(rt, y, "Hunger", 255 - golfer.needs.hungerByte(), 255, Colour::brightRed);
            y += kRowHeight + 4;
            bar(rt, y, "Thirst", 255 - golfer.needs.thirstByte(), 255, Colour::brightRed);
            y += kRowHeight + 4;
            bar(rt, y, "Bladder", golfer.needs.bladderByte(), 255, Colour::brightRed);
        }

        void drawSkills(RenderTarget& rt, const Golfer& golfer)
        {
            // GLF-020: the four skills, 0..100.
            const auto skills = golfer.currentSkills();
            int32_t y = 80;
            bar(rt, y, "Long play", skills.longPlay, 100, Colour::yellow);
            y += kRowHeight + 4;
            bar(rt, y, "Accuracy", skills.accuracy, 100, Colour::yellow);
            y += kRowHeight + 4;
            bar(rt, y, "Foolproof", skills.foolproof, 100, Colour::yellow);
            y += kRowHeight + 4;
            bar(rt, y, "Physique", skills.physique, 100, Colour::yellow);
        }

        void drawScorecard(RenderTarget& rt, const Golfer& golfer)
        {
            // Strokes against par for each finished hole; the hole in play shows its strokes so far.
            int32_t y = 76;
            text(rt, 12, y, "Hole", Colour::grey);
            text(rt, 56, y, "Par", Colour::grey);
            text(rt, 96, y, "Score", Colour::grey);
            y += kRowHeight;
            const size_t visible = 8;
            const size_t first = golfer.scorecard.size() > visible ? golfer.scorecard.size() - visible : 0;
            for (size_t i = first; i < golfer.scorecard.size(); i++)
            {
                const int32_t toPar = golfer.scorecard[i];
                const int32_t par = i < golfer.parCard.size() ? golfer.parCard[i] : 0;
                text(rt, 12, y, std::to_string(i + 1));
                text(rt, 56, y, std::to_string(par));
                if (toPar == 127)
                    text(rt, 96, y, "-", Colour::grey);
                else
                    text(rt, 96, y, std::to_string(par + toPar) + " (" + (toPar > 0 ? "+" : "") + std::to_string(toPar) + ")");
                y += kRowHeight;
            }
            text(rt, 12, 182, "Total strokes: " + std::to_string(golfer.strokesTotal), Colour::grey);
        }

        void drawThoughts(RenderTarget& rt, const Golfer& golfer)
        {
            int32_t y = 76;
            if (golfer.thoughts.empty())
                text(rt, 12, y, "Nothing in particular.", Colour::grey);
            const size_t visible = 9;
            const size_t first = golfer.thoughts.size() > visible ? golfer.thoughts.size() - visible : 0;
            for (size_t i = golfer.thoughts.size(); i > first; i--)
            {
                text(rt, 12, y, thoughtName(golfer.thoughts[i - 1].type));
                y += kRowHeight;
            }
        }

        void drawMoney(RenderTarget& rt, const Golfer& golfer)
        {
            int32_t y = 80;
            {
                auto ft = Formatter();
                ft.Add<money64>(golfer.money);
                drawText(rt, windowPos + ScreenCoordsXY{ 12, y }, STR_GUEST_STAT_CASH_IN_POCKET, ft, { colours[1] });
            }
            y += kRowHeight;
            text(rt, 12, y, "Arrived on day " + std::to_string(golfer.arrivalDay));
            y += kRowHeight;
            text(rt, 12, y, "Holes finished: " + std::to_string(golfer.scorecard.size()));
        }

        void drawTabImages(RenderTarget& rt)
        {
            static constexpr uint32_t kImages[WINDOW_GOLFER_PAGE_COUNT] = {
                SPR_TAB_PARK, SPR_TAB_GEARS_0, SPR_TAB_STATS, SPR_TAB_AWARDS, SPR_TAB_QUESTION, SPR_TAB_GUEST_INVENTORY,
            };
            for (int32_t i = 0; i < WINDOW_GOLFER_PAGE_COUNT; i++)
            {
                const auto& widget = widgets[WIDX_TAB_1 + i];
                GfxDrawSprite(rt, ImageId(kImages[i]), windowPos + ScreenCoordsXY{ widget.left + 3, widget.top });
            }
        }
    };

    WindowBase* GolferOpen(EntityId golferEntity)
    {
        auto* windowMgr = Ui::GetWindowManager();
        auto* window = windowMgr->BringToFrontByNumber(WindowClass::golfer, golferEntity.ToUnderlying());
        if (window == nullptr)
        {
            window = windowMgr->Create<GolferWindow>(WindowClass::golfer, kWindowSize, WindowFlags{});
            if (window == nullptr)
                return nullptr;
            window->number = golferEntity.ToUnderlying();
        }
        return window;
    }
} // namespace OpenRCT2::Ui::Windows
