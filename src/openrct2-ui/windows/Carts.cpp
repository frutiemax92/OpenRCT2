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
#include <openrct2/actions/golf/CartEditAction.h>
#include <openrct2/drawing/Colour.h>
#include <openrct2/drawing/Drawing.h>
#include <openrct2/drawing/RenderTarget.h>
#include <openrct2/drawing/Text.h>
#include <openrct2/golf/GolfRuntime.h>
#include <openrct2/localisation/Formatter.h>
#include <openrct2/localisation/StringIds.h>
#include <openrct2/ui/WindowManager.h>
#include <string>

namespace OpenRCT2::Ui::Windows
{
    using namespace GameActions;
    using namespace OpenRCT2::Drawing;

    // The golf carts (section 14): how many there are, how many are free to rent, and the buttons to buy and sell them and
    // to set the cart fee riders pay.
    enum WindowCartsWidgetIdx : WidgetIndex
    {
        WIDX_BACKGROUND,
        WIDX_TITLE,
        WIDX_CLOSE,
        WIDX_BUY,
        WIDX_SELL,
        WIDX_FEE_LABEL,
        WIDX_FEE,
        WIDX_FEE_DOWN,
        WIDX_FEE_UP,
    };

    static constexpr ScreenSize kWindowSize = { 250, 150 };
    static constexpr int64_t kFeeStep = 50;

    // clang-format off
    static constexpr auto kCartsWidgets = makeWidgets(
        makeWindowShim(STR_CARTS_TITLE, kWindowSize),
        makeWidget({  10, 76 }, { 110, 18 }, WidgetType::button, WindowColour::secondary, STR_CARTS_BUY, STR_CARTS_BUY_TIP),
        makeWidget({ 130, 76 }, { 110, 18 }, WidgetType::button, WindowColour::secondary, STR_CARTS_SELL),
        makeWidget({  10, 104 }, { 90, 14 }, WidgetType::label,  WindowColour::secondary, kStringIdNone),
        makeSpinnerWidgets({ 105, 102 }, { 100, 18 }, WidgetType::spinner, WindowColour::secondary)
    );
    // clang-format on

    class CartsWindow final : public Window
    {
    private:
        std::string _status;

        Golf::CartFleet* fleet() const
        {
            auto& runtime = getGameState().golfRuntime;
            return runtime != nullptr ? &runtime->director().fleet : nullptr;
        }

        void run(CartEditType type, int64_t value)
        {
            CartEditAction action(type, value);
            const auto result = Execute(&action, getGameState());
            _status = result.error == Status::ok ? std::string() : result.getErrorMessage();
        }

    public:
        void onOpen() override
        {
            setWidgets(kCartsWidgets);
        }

        void onMouseUp(WidgetIndex widgetIndex) override
        {
            switch (widgetIndex)
            {
                case WIDX_CLOSE:
                    close();
                    return;
                case WIDX_BUY:
                    run(CartEditType::buy, 1);
                    break;
                case WIDX_SELL:
                    run(CartEditType::sell, 1);
                    break;
                case WIDX_FEE_UP:
                    if (const auto* f = fleet())
                        run(CartEditType::setFee, f->cartFee + kFeeStep);
                    break;
                case WIDX_FEE_DOWN:
                    if (const auto* f = fleet())
                        run(CartEditType::setFee, std::max<int64_t>(0, f->cartFee - kFeeStep));
                    break;
            }
            invalidate();
        }

        void onUpdate() override
        {
            invalidate();
        }

        void onDraw(RenderTarget& rt) override
        {
            drawWidgets(rt);
            auto text = [&](int32_t x, int32_t y, const std::string& s, Colour colour = Colour::black) {
                drawText(rt, windowPos + ScreenCoordsXY{ x, y }, s, { colour });
            };
            const auto* f = fleet();
            if (f == nullptr)
            {
                text(12, 24, "No golf course yet.", Colour::grey);
                return;
            }
            text(12, 24, "Carts: " + std::to_string(f->cartCount()) + "   free to rent: " + std::to_string(f->freeCarts()));
            text(12, 38, "Cart bays: " + std::to_string(f->spaceCount()), f->spaceCount() == 0 ? Colour::brightRed : Colour::black);
            {
                auto ft = Formatter();
                ft.Add<money64>(f->cartPrice);
                drawText(rt, windowPos + ScreenCoordsXY{ 12, 54 }, STR_CARTS_PRICE_EACH, ft, { colours[1] });
            }
            text(12, 106, "Cart fee");
            {
                auto ft = Formatter();
                ft.Add<money64>(f->cartFee);
                drawText(
                    rt, windowPos + ScreenCoordsXY{ widgets[WIDX_FEE].left + 4, widgets[WIDX_FEE].top + 3 },
                    f->cartFee == 0 ? StringId(STR_FREE) : StringId(STR_CURRENCY2DP), ft, { colours[1] });
            }
            if (!_status.empty())
                text(12, 128, _status, Colour::brightRed);
        }
    };

    WindowBase* CartsOpen()
    {
        auto* windowMgr = GetWindowManager();
        return windowMgr->FocusOrCreate<CartsWindow>(WindowClass::carts, ScreenCoordsXY(32, 48), kWindowSize);
    }
} // namespace OpenRCT2::Ui::Windows
