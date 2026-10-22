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
#include <openrct2/actions/golf/BuildingEditAction.h>
#include <openrct2/actions/scenery/BuildingRemoveAction.h>
#include <openrct2/drawing/Colour.h>
#include <openrct2/drawing/Drawing.h>
#include <openrct2/drawing/RenderTarget.h>
#include <openrct2/drawing/Text.h>
#include <openrct2/golf/GolfDirector.h>
#include <openrct2/golf/GolfRuntime.h>
#include <openrct2/localisation/Formatter.h>
#include <openrct2/localisation/StringIds.h>
#include <openrct2/object/BuildingObject.h>
#include <openrct2/ui/WindowManager.h>
#include <openrct2/world/Building.h>
#include <openrct2/world/tile_element/BuildingElement.h>
#include <string>

namespace OpenRCT2::Ui::Windows
{
    using namespace GameActions;
    using namespace OpenRCT2::Drawing;

    // Section 9.3: the name, kind, services and prices of a building, what it earned this month and last, its visitors,
    // whether it is open, and a button to take it down.
    enum WindowBuildingWidgetIdx : WidgetIndex
    {
        WIDX_BACKGROUND,
        WIDX_TITLE,
        WIDX_CLOSE,
        WIDX_PRICE_LABEL,
        WIDX_PRICE,
        WIDX_PRICE_DOWN,
        WIDX_PRICE_UP,
        WIDX_TOGGLE_OPEN,
        WIDX_DEMOLISH,
    };

    static constexpr ScreenSize kWindowSize = { 270, 190 };
    static constexpr int64_t kPriceStep = 50;

    // clang-format off
    static constexpr auto kBuildingWidgets = makeWidgets(
        makeWindowShim(STR_BUILDING_TITLE, kWindowSize),
        makeWidget({  10, 78 }, {  80, 14 }, WidgetType::label,  WindowColour::secondary, kStringIdNone),
        makeSpinnerWidgets({ 95, 76 }, { 100, 18 }, WidgetType::spinner, WindowColour::secondary),
        makeWidget({  10, 100 }, { 120, 18 }, WidgetType::button, WindowColour::secondary, STR_BUILDING_CLOSE),
        makeWidget({ 140, 100 }, { 120, 18 }, WidgetType::button, WindowColour::secondary, STR_BUILDING_DEMOLISH)
    );
    // clang-format on

    static const char* kindName(BuildingKind kind)
    {
        switch (kind)
        {
            case BuildingKind::golfClub:
                return "Golf club";
            case BuildingKind::cottage:
                return "Golf cottage";
            case BuildingKind::foodTruck:
                return "Food truck";
            case BuildingKind::drivingRangeKiosk:
                return "Driving range kiosk";
            case BuildingKind::cartRental:
                return "Cart rental";
            case BuildingKind::restroom:
                return "Restroom";
            default:
                return "Building";
        }
    }

    class BuildingWindow final : public Window
    {
    private:
        CoordsXY _tile{};

        const BuildingElement* element() const
        {
            return Building::FindAt(_tile);
        }
        Golf::GolfDirector* director() const
        {
            auto& runtime = getGameState().golfRuntime;
            return runtime != nullptr ? &runtime->director() : nullptr;
        }
        bool hasServices(const BuildingObject& object) const
        {
            return object.Services.hasAny(
                BuildingService::food, BuildingService::drink, BuildingService::toilet, BuildingService::proShop,
                BuildingService::rest);
        }
        uint32_t key() const
        {
            const auto* e = element();
            const auto origin = Building::GetOrigin(_tile, *e);
            return Golf::buildingKey(origin.x / 32, origin.y / 32);
        }
        void run(BuildingEditType type, int64_t value)
        {
            BuildingEditAction action(_tile, type, value);
            Execute(&action, getGameState());
        }

    public:
        void onOpen() override
        {
            setWidgets(kBuildingWidgets);
        }

        void setTile(const CoordsXY& tile)
        {
            _tile = tile;
            invalidate();
        }

        void onMouseUp(WidgetIndex widgetIndex) override
        {
            auto* d = director();
            if (element() == nullptr)
            {
                close();
                return;
            }
            const auto* object = element()->getObject();
            if (object == nullptr)
            {
                close();
                return;
            }
            const Golf::BuildingBook defaults;
            const auto book = d != nullptr && d->buildings.count(key()) ? d->buildings.at(key()) : defaults;
            const int64_t usual = Golf::defaultServicePrice(object->Kind);
            const int64_t price = book.price >= 0 ? book.price : usual;
            switch (widgetIndex)
            {
                case WIDX_CLOSE:
                    close();
                    return;
                case WIDX_TOGGLE_OPEN:
                    run(BuildingEditType::setClosed, book.closed ? 0 : 1);
                    break;
                case WIDX_PRICE_UP:
                    run(BuildingEditType::setPrice, price + kPriceStep);
                    break;
                case WIDX_PRICE_DOWN:
                    run(BuildingEditType::setPrice, std::max<int64_t>(0, price - kPriceStep));
                    break;
                case WIDX_DEMOLISH:
                {
                    BuildingRemoveAction action(_tile);
                    Execute(&action, getGameState());
                    close();
                    return;
                }
            }
            invalidate();
        }

        void onUpdate() override
        {
            if (element() == nullptr)
                close();
            else
                invalidate();
        }

        void onPrepareDraw() override
        {
            const auto* e = element();
            const auto* object = e != nullptr ? e->getObject() : nullptr;
            const bool services = object != nullptr && hasServices(*object);
            setWidgetDisabled(WIDX_PRICE_DOWN, !services);
            setWidgetDisabled(WIDX_PRICE_UP, !services);
            setWidgetDisabled(WIDX_TOGGLE_OPEN, !services);
            // A cottage is not the player's to take down (BLD-012).
            setWidgetDisabled(WIDX_DEMOLISH, object == nullptr || object->Kind == BuildingKind::cottage);
            if (auto* d = director(); d != nullptr && e != nullptr)
            {
                const auto it = d->buildings.find(key());
                const bool closed = it != d->buildings.end() && it->second.closed;
                widgets[WIDX_TOGGLE_OPEN].text = closed ? STR_BUILDING_OPEN : STR_BUILDING_CLOSE;
            }
        }

        void onDraw(RenderTarget& rt) override
        {
            drawWidgets(rt);
            auto text = [&](int32_t x, int32_t y, const std::string& s, Colour colour = Colour::black) {
                drawText(rt, windowPos + ScreenCoordsXY{ x, y }, s, { colour });
            };
            const auto* e = element();
            const auto* object = e != nullptr ? e->getObject() : nullptr;
            if (object == nullptr)
                return;
            text(10, 22, std::string(object->GetName()));
            text(10, 34, kindName(object->Kind), Colour::grey);

            std::string offered;
            auto add = [&](BuildingService service, const char* name) {
                if (object->Services.has(service))
                    offered += (offered.empty() ? "" : ", ") + std::string(name);
            };
            add(BuildingService::food, "food");
            add(BuildingService::drink, "drinks");
            add(BuildingService::toilet, "toilets");
            add(BuildingService::proShop, "pro shop");
            add(BuildingService::rest, "rest");
            text(10, 48, offered.empty() ? "No services for golfers" : "Services: " + offered);

            auto money = [&](int32_t x, int32_t y, const std::string& label, int64_t amount) {
                auto ft = Formatter();
                ft.Add<money64>(amount);
                text(x, y, label);
                drawText(rt, windowPos + ScreenCoordsXY{ x + 110, y }, STR_CURRENCY2DP, ft, { colours[1] });
            };
            text(10, 80, hasServices(*object) ? "Price of a visit" : "");
            {
                const auto* d = director();
                const Golf::BuildingBook empty;
                const auto& book = d != nullptr && d->buildings.count(key()) ? d->buildings.at(key()) : empty;
                const int64_t price = book.price >= 0 ? book.price : Golf::defaultServicePrice(object->Kind);
                if (hasServices(*object))
                {
                    auto ft = Formatter();
                    ft.Add<money64>(price);
                    drawText(
                        rt, windowPos + ScreenCoordsXY{ widgets[WIDX_PRICE].left + 4, widgets[WIDX_PRICE].top + 3 },
                        price == 0 ? StringId(STR_FREE) : StringId(STR_CURRENCY2DP), ft, { colours[1] });
                    text(10, 124, "This month: " + std::to_string(book.visitorsThisMonth) + " visitors");
                    money(10, 136, "  revenue", book.revenueThisMonth);
                    text(10, 150, "Last month: " + std::to_string(book.visitorsLastMonth) + " visitors");
                    money(10, 162, "  revenue", book.revenueLastMonth);
                    if (book.closed)
                        text(10, 176, "Closed to golfers", Colour::brightRed);
                }
                money(150, 150, "Upkeep a month", object->UpkeepPerMonth);
            }
        }
    };

    WindowBase* BuildingOpen(const CoordsXY& position)
    {
        auto* windowMgr = GetWindowManager();
        auto* window = windowMgr->FocusOrCreate<BuildingWindow>(WindowClass::building, ScreenCoordsXY(32, 48), kWindowSize);
        if (window != nullptr)
            window->setTile(position);
        return window;
    }
} // namespace OpenRCT2::Ui::Windows
