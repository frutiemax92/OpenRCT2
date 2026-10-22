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
#include <openrct2/Input.h>
#include <openrct2/actions/GameActionRunner.h>
#include <openrct2/actions/golf/BuildingEditAction.h>
#include <openrct2/actions/scenery/BuildingPlaceAction.h>
#include <openrct2/actions/scenery/BuildingRemoveAction.h>
#include <openrct2/drawing/Colour.h>
#include <openrct2/drawing/Drawing.h>
#include <openrct2/drawing/RenderTarget.h>
#include <openrct2/drawing/Text.h>
#include <openrct2/golf/GolfDirector.h>
#include <openrct2/golf/GolfRuntime.h>
#include <openrct2/interface/Viewport.h>
#include <openrct2/localisation/Formatter.h>
#include <openrct2/localisation/StringIds.h>
#include <openrct2/object/BuildingObject.h>
#include <openrct2/object/ObjectLimits.h>
#include <openrct2/object/ObjectManager.h>
#include <openrct2/ui/WindowManager.h>
#include <openrct2/world/Building.h>
#include <openrct2/world/MapSelection.h>
#include <openrct2/world/tile_element/BuildingElement.h>
#include <string>
#include <vector>

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

    enum WindowBuildingToolWidgetIdx : WidgetIndex
    {
        WIDX_TOOL_BACKGROUND,
        WIDX_TOOL_TITLE,
        WIDX_TOOL_CLOSE,
        WIDX_TOOL_PREV,
        WIDX_TOOL_NEXT,
        WIDX_TOOL_ROTATE,
        WIDX_TOOL_DEMOLISH,
    };

    static constexpr ScreenSize kToolWindowSize = { 260, 160 };

    // clang-format off
    static constexpr auto kBuildingToolWidgets = makeWidgets(
        makeWindowShim(STR_BUILDING_TITLE, kToolWindowSize),
        makeWidget({  10, 24 }, {  30, 20 }, WidgetType::button, WindowColour::secondary, kStringIdNone),
        makeWidget({ 220, 24 }, {  30, 20 }, WidgetType::button, WindowColour::secondary, kStringIdNone),
        makeWidget({  10, 105 }, { 115, 20 }, WidgetType::button, WindowColour::secondary, kStringIdNone),
        makeWidget({ 135, 105 }, { 115, 20 }, WidgetType::button, WindowColour::secondary, STR_BUILDING_DEMOLISH)
    );
    // clang-format on

    class BuildingToolWindow final : public Window
    {
    private:
        std::vector<ObjectEntryIndex> _buildings;
        size_t _selectedIndex{ 0 };
        Direction _rotation{ 0 };
        bool _demolish{ false };
        std::string _status;

        void refreshBuildings()
        {
            _buildings.clear();
            auto& objects = GetContext()->GetObjectManager();
            for (ObjectEntryIndex i = 0; i < kMaxBuildingObjects; i++)
            {
                const auto* object = objects.GetLoadedObject<BuildingObject>(i);
                if (object != nullptr)
                {
                    _buildings.push_back(i);
                }
            }
            if (_selectedIndex >= _buildings.size())
                _selectedIndex = 0;
        }

    public:
        BuildingToolWindow()
        {
            setWidgets(kBuildingToolWidgets);
            refreshBuildings();
            ToolSet(*this, WIDX_TOOL_BACKGROUND, Tool::crosshair);
        }

        void onClose() override
        {
            if (isToolActive(*this, WIDX_TOOL_BACKGROUND))
                ToolCancel();
        }

        void onMouseUp(WidgetIndex widgetIndex) override
        {
            switch (widgetIndex)
            {
                case WIDX_TOOL_CLOSE:
                    close();
                    break;
                case WIDX_TOOL_PREV:
                    if (!_buildings.empty())
                    {
                        if (_selectedIndex == 0)
                            _selectedIndex = _buildings.size() - 1;
                        else
                            _selectedIndex--;
                        invalidate();
                    }
                    break;
                case WIDX_TOOL_NEXT:
                    if (!_buildings.empty())
                    {
                        _selectedIndex = (_selectedIndex + 1) % _buildings.size();
                        invalidate();
                    }
                    break;
                case WIDX_TOOL_ROTATE:
                    _rotation = (_rotation + 1) & 3;
                    invalidate();
                    break;
                case WIDX_TOOL_DEMOLISH:
                    _demolish = !_demolish;
                    invalidate();
                    break;
            }
        }

        void onToolUpdate(WidgetIndex widgetIndex, const ScreenCoordsXY& screenCoords) override
        {
            const auto position = ScreenGetMapXY(screenCoords, nullptr);
            if (!position.has_value())
                return;

            const auto tile = position->toTileStart();
            gMapSelectFlags.set(MapSelectFlag::enable);
            gMapSelectType = MapSelectType::full;

            if (_demolish)
            {
                gMapSelectPositionA = tile;
                gMapSelectPositionB = tile;
            }
            else if (!_buildings.empty() && _selectedIndex < _buildings.size())
            {
                const auto* obj = BuildingObject::GetById(_buildings[_selectedIndex]);
                if (obj != nullptr)
                {
                    const int32_t fx = obj->getFootprintX(_rotation);
                    const int32_t fy = obj->getFootprintY(_rotation);
                    gMapSelectPositionA = tile;
                    gMapSelectPositionB = { tile.x + (fx - 1) * 32, tile.y + (fy - 1) * 32 };
                }
            }
        }

        void onToolDown(WidgetIndex widgetIndex, const ScreenCoordsXY& screenCoords) override
        {
            const auto position = ScreenGetMapXY(screenCoords, nullptr);
            if (!position.has_value())
                return;

            const auto tile = position->toTileStart();
            auto& gameState = getGameState();

            if (_demolish)
            {
                BuildingRemoveAction remove(tile);
                auto res = Execute(&remove, gameState);
                if (res.error != Status::ok)
                    _status = res.getErrorMessage();
                else
                    _status = "Building removed.";
            }
            else if (!_buildings.empty() && _selectedIndex < _buildings.size())
            {
                BuildingPlaceAction place(tile, _buildings[_selectedIndex], _rotation);
                auto res = Execute(&place, gameState);
                if (res.error != Status::ok)
                    _status = res.getErrorMessage();
                else
                    _status.clear();
            }
            invalidate();
        }

        void onPrepareDraw() override
        {
            setWidgetPressed(WIDX_TOOL_DEMOLISH, _demolish);
        }

        void onDraw(RenderTarget& rt) override
        {
            drawWidgets(rt);
            auto text = [&](int32_t x, int32_t y, const std::string& s, Colour colour = Colour::black) {
                drawText(rt, windowPos + ScreenCoordsXY{ x, y }, s, { colour });
            };

            text(widgets[WIDX_TOOL_PREV].left + 10, widgets[WIDX_TOOL_PREV].top + 3, "<");
            text(widgets[WIDX_TOOL_NEXT].left + 10, widgets[WIDX_TOOL_NEXT].top + 3, ">");
            text(widgets[WIDX_TOOL_ROTATE].left + 35, widgets[WIDX_TOOL_ROTATE].top + 3, "Rotate");

            if (_buildings.empty())
            {
                refreshBuildings();
            }

            if (_buildings.empty())
            {
                text(45, 26, "No buildings loaded", Colour::grey);
                return;
            }

            const auto* obj = BuildingObject::GetById(_buildings[_selectedIndex]);
            if (obj == nullptr)
                return;

            text(45, 26, kindName(obj->Kind));
            text(12, 50, "Kind: " + std::string(kindName(obj->Kind)));
            text(12, 64, "Footprint: " + std::to_string(obj->getFootprintX(_rotation)) + " x " + std::to_string(obj->getFootprintY(_rotation)) + " tiles");
            text(12, 78, "Build cost: $" + std::to_string(obj->BuildCost / 10));
            text(12, 92, "Rotation: " + std::to_string(_rotation * 90) + " deg");

            if (_demolish)
            {
                text(12, 132, "Demolish mode: click building", Colour::brightRed);
            }
            else if (!_status.empty())
            {
                text(12, 132, _status, Colour::brightRed);
            }
        }
    };

    WindowBase* BuildingToolOpen()
    {
        auto* windowMgr = GetWindowManager();
        return windowMgr->FocusOrCreate<BuildingToolWindow>(
            WindowClass::buildingTool, ScreenCoordsXY(32, 48), kToolWindowSize);
    }
} // namespace OpenRCT2::Ui::Windows
