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
#include <openrct2/actions/GameActionRunner.h>
#include <openrct2/actions/golf/TournamentEditAction.h>
#include <openrct2/drawing/Colour.h>
#include <openrct2/drawing/RenderTarget.h>
#include <openrct2/drawing/Text.h>
#include <openrct2/golf/GolfDirector.h>
#include <openrct2/golf/GolfRuntime.h>
#include <openrct2/golf/Tournament.h>
#include <openrct2/localisation/StringIds.h>
#include <openrct2/ui/WindowManager.h>
#include <string>

namespace OpenRCT2::Ui::Windows
{
    using namespace GameActions;
    using namespace OpenRCT2::Drawing;

    // The tournament window (section 22.5): the checklist of a tier with what the course has and what it needs, and the
    // booking of the day and the ticket price.
    enum WindowTournamentWidgetIdx : WidgetIndex
    {
        WIDX_BACKGROUND,
        WIDX_TITLE,
        WIDX_CLOSE,
        WIDX_TIER_PREV,
        WIDX_TIER_NEXT,
        WIDX_DAYS_DOWN,
        WIDX_DAYS_UP,
        WIDX_PRICE_DOWN,
        WIDX_PRICE_UP,
        WIDX_BOOK,
        WIDX_CANCEL,
    };

    static constexpr ScreenSize kWindowSize = { 320, 300 };

    // clang-format off
    static constexpr auto kTournamentWidgets = makeWidgets(
        makeWindowShim(STR_TOURNAMENT_TITLE, kWindowSize),
        makeWidget({  10, 22 }, { 20, 14 }, WidgetType::button, WindowColour::secondary, kStringIdNone),
        makeWidget({ 130, 22 }, { 20, 14 }, WidgetType::button, WindowColour::secondary, kStringIdNone),
        makeWidget({ 250, 190 }, { 20, 14 }, WidgetType::button, WindowColour::secondary, kStringIdNone),
        makeWidget({ 274, 190 }, { 20, 14 }, WidgetType::button, WindowColour::secondary, kStringIdNone),
        makeWidget({ 250, 208 }, { 20, 14 }, WidgetType::button, WindowColour::secondary, kStringIdNone),
        makeWidget({ 274, 208 }, { 20, 14 }, WidgetType::button, WindowColour::secondary, kStringIdNone),
        makeWidget({  10, 240 }, { 140, 18 }, WidgetType::button, WindowColour::secondary, STR_TOURNAMENT_BOOK),
        makeWidget({ 160, 240 }, { 140, 18 }, WidgetType::button, WindowColour::secondary, STR_TOURNAMENT_CANCEL)
    );
    // clang-format on

    class TournamentWindow final : public Window
    {
    private:
        int32_t _tier = 0;
        int32_t _daysAhead = 14;
        int64_t _price = 2000;

        Golf::GolfDirector* director() const
        {
            auto& runtime = getGameState().golfRuntime;
            return runtime != nullptr ? &runtime->director() : nullptr;
        }
        Golf::TournamentInputs inputs() const
        {
            auto& runtime = getGameState().golfRuntime;
            return runtime != nullptr ? runtime->tournamentInputs() : Golf::TournamentInputs{};
        }

    public:
        void onOpen() override
        {
            setWidgets(kTournamentWidgets);
        }

        void onMouseUp(WidgetIndex widgetIndex) override
        {
            const auto tiers = static_cast<int32_t>(Golf::TournamentTier::count);
            auto& gameState = getGameState();
            switch (widgetIndex)
            {
                case WIDX_CLOSE:
                    close();
                    return;
                case WIDX_TIER_PREV:
                    _tier = (_tier + tiers - 1) % tiers;
                    break;
                case WIDX_TIER_NEXT:
                    _tier = (_tier + 1) % tiers;
                    break;
                case WIDX_DAYS_DOWN:
                    _daysAhead = std::max(1, _daysAhead - 1);
                    break;
                case WIDX_DAYS_UP:
                    _daysAhead++;
                    break;
                case WIDX_PRICE_DOWN:
                    _price = std::max<int64_t>(0, _price - 100);
                    break;
                case WIDX_PRICE_UP:
                    _price += 100;
                    break;
                case WIDX_BOOK:
                {
                    TournamentEditAction action(TournamentEditType::book, static_cast<uint8_t>(_tier), _daysAhead, _price);
                    Execute(&action, gameState);
                    break;
                }
                case WIDX_CANCEL:
                {
                    TournamentEditAction action(TournamentEditType::cancel);
                    Execute(&action, gameState);
                    break;
                }
                default:
                    break;
            }
            invalidate();
        }

        void onPrepareDraw() override
        {
            const auto* d = director();
            const auto tier = static_cast<Golf::TournamentTier>(_tier);
            const bool booked = d != nullptr && d->tournament.phase == Golf::TournamentPhase::scheduled;
            setWidgetDisabled(WIDX_BOOK, d == nullptr || booked || !Golf::canHost(tier, inputs()));
            setWidgetDisabled(WIDX_CANCEL, !booked);
        }

        void onDraw(RenderTarget& rt) override
        {
            drawWidgets(rt);
            auto text = [&](int32_t x, int32_t y, const std::string& s, Colour colour = Colour::black) {
                drawText(rt, windowPos + ScreenCoordsXY{ x, y }, s, { colour });
            };
            auto label = [&](WidgetIndex widget, const char* s) {
                text(widgets[widget].left + 6, widgets[widget].top + 1, s);
            };
            label(WIDX_TIER_PREV, "<");
            label(WIDX_TIER_NEXT, ">");
            label(WIDX_DAYS_DOWN, "-");
            label(WIDX_DAYS_UP, "+");
            label(WIDX_PRICE_DOWN, "-");
            label(WIDX_PRICE_UP, "+");

            const auto tier = static_cast<Golf::TournamentTier>(_tier);
            text(36, 24, std::string("Tier: ") + Golf::tierName(tier));
            int32_t y = 44;
            for (const auto& line : Golf::checklist(tier, inputs()))
            {
                text(
                    12, y, std::string(line.met ? "[x] " : "[ ] ") + line.label + ": " + std::to_string(line.current) + " / " + std::to_string(line.required),
                    line.met ? Colour::darkGreen : Colour::brightRed);
                y += 12;
            }
            text(12, 192, "Days ahead: " + std::to_string(_daysAhead));
            text(12, 210, "Ticket price: " + std::to_string(_price));

            if (const auto* d = director())
            {
                const auto& t = d->tournament;
                switch (t.phase)
                {
                    case Golf::TournamentPhase::scheduled:
                        text(12, 266, std::string("Booked: ") + Golf::tierName(t.tier) + ", day " + std::to_string(t.day));
                        break;
                    case Golf::TournamentPhase::running:
                        text(12, 266, "A tournament is being played; the course is closed.");
                        break;
                    case Golf::TournamentPhase::finished:
                        text(12, 266, "Last result: " + std::to_string(t.result) + "/100, revenue " + std::to_string(t.revenue) + ", popularity +" + std::to_string(t.popularityGained));
                        break;
                    default:
                        text(12, 266, "Nothing booked.", Colour::grey);
                        break;
                }
            }
        }
    };

    WindowBase* TournamentOpen()
    {
        auto* windowMgr = GetWindowManager();
        return windowMgr->FocusOrCreate<TournamentWindow>(WindowClass::tournament, ScreenCoordsXY(32, 48), kWindowSize);
    }
} // namespace OpenRCT2::Ui::Windows
