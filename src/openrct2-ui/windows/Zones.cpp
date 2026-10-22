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
#include <openrct2/actions/golf/RangeEditAction.h>
#include <openrct2/actions/golf/ZoneEditAction.h>
#include <openrct2/drawing/Colour.h>
#include <openrct2/drawing/Drawing.h>
#include <openrct2/drawing/RenderTarget.h>
#include <openrct2/drawing/Text.h>
#include <openrct2/golf/DrivingRange.h>
#include <openrct2/interface/Viewport.h>
#include <openrct2/localisation/StringIds.h>
#include <openrct2/ui/WindowManager.h>
#include <string>

namespace OpenRCT2::Ui::Windows
{
    using namespace GameActions;
    using namespace OpenRCT2::Drawing;

    // The zone tool: draw the areas of sections 17 and 22 by clicking two corners, mark hitting tiles, take zones away, and
    // set up the driving range (price of a bucket, balls, direction, and the warning when the kiosk may be hit).
    enum WindowZonesWidgetIdx : WidgetIndex
    {
        WIDX_BACKGROUND,
        WIDX_TITLE,
        WIDX_CLOSE,
        WIDX_LOT,
        WIDX_SPECTATORS,
        WIDX_MEDIA,
        WIDX_EXTRA_PARKING,
        WIDX_GRANDSTAND,
        WIDX_RANGE,
        WIDX_HITTING_TILE,
        WIDX_REMOVE,
        WIDX_RANGE_PREV,
        WIDX_RANGE_NEXT,
        WIDX_PRICE_DOWN,
        WIDX_PRICE_UP,
        WIDX_BALLS_DOWN,
        WIDX_BALLS_UP,
        WIDX_AIM_LEFT,
        WIDX_AIM_RIGHT,
        WIDX_AIM_AUTO,
    };

    static constexpr ScreenSize kWindowSize = { 290, 252 };
    static constexpr int32_t kToolCount = 8;
    static constexpr int32_t kFirstTool = WIDX_LOT;

    // clang-format off
    static constexpr auto kZonesWidgets = makeWidgets(
        makeWindowShim(STR_ZONES_TITLE, kWindowSize),
        makeWidget({  10, 20 }, { 135, 18 }, WidgetType::button, WindowColour::secondary, STR_ZONES_LOT),
        makeWidget({ 145, 20 }, { 135, 18 }, WidgetType::button, WindowColour::secondary, STR_ZONES_SPECTATORS),
        makeWidget({  10, 40 }, { 135, 18 }, WidgetType::button, WindowColour::secondary, STR_ZONES_MEDIA),
        makeWidget({ 145, 40 }, { 135, 18 }, WidgetType::button, WindowColour::secondary, STR_ZONES_EXTRA_PARKING),
        makeWidget({  10, 60 }, { 135, 18 }, WidgetType::button, WindowColour::secondary, STR_ZONES_GRANDSTAND),
        makeWidget({ 145, 60 }, { 135, 18 }, WidgetType::button, WindowColour::secondary, STR_ZONES_RANGE),
        makeWidget({  10, 80 }, { 135, 18 }, WidgetType::button, WindowColour::secondary, STR_ZONES_HITTING_TILE),
        makeWidget({ 145, 80 }, { 135, 18 }, WidgetType::button, WindowColour::secondary, STR_ZONES_REMOVE, STR_ZONES_REMOVE_TIP),
        makeWidget({  10, 120 }, { 20, 14 }, WidgetType::button, WindowColour::secondary, kStringIdNone),
        makeWidget({  34, 120 }, { 20, 14 }, WidgetType::button, WindowColour::secondary, kStringIdNone),
        makeWidget({ 190, 144 }, { 20, 14 }, WidgetType::button, WindowColour::secondary, kStringIdNone),
        makeWidget({ 214, 144 }, { 20, 14 }, WidgetType::button, WindowColour::secondary, kStringIdNone),
        makeWidget({ 190, 162 }, { 20, 14 }, WidgetType::button, WindowColour::secondary, kStringIdNone),
        makeWidget({ 214, 162 }, { 20, 14 }, WidgetType::button, WindowColour::secondary, kStringIdNone),
        makeWidget({ 190, 180 }, { 20, 14 }, WidgetType::button, WindowColour::secondary, kStringIdNone),
        makeWidget({ 214, 180 }, { 20, 14 }, WidgetType::button, WindowColour::secondary, kStringIdNone),
        makeWidget({ 238, 180 }, { 40, 14 }, WidgetType::button, WindowColour::secondary, kStringIdNone)
    );
    // clang-format on

    class ZonesWindow final : public Window
    {
    private:
        size_t _range = 0;
        std::optional<CoordsXY> _first; // the first corner of a rectangle being drawn
        std::string _status;

        Golf::DrivingRange* range()
        {
            auto& ranges = getGameState().golf.ranges();
            if (ranges.empty())
                return nullptr;
            _range = std::min(_range, ranges.size() - 1);
            return &ranges[_range];
        }

        WidgetIndex activeTool() const
        {
            for (int32_t i = 0; i < kToolCount; i++)
                if (isToolActive(*this, kFirstTool + i))
                    return kFirstTool + i;
            return kWidgetIndexNull;
        }

        void report(const Result& result)
        {
            _status = result.error == Status::ok ? std::string() : result.getErrorMessage();
        }

        void removeAt(const CoordsXY& position)
        {
            auto& gameState = getGameState();
            const TileCoordsXY tile{ position.x >> 5, position.y >> 5 };
            for (const auto& lot : gameState.golf.lots())
            {
                if (lot.contains(tile))
                {
                    ZoneEditAction action(ZoneEditType::removeLot, lot.id);
                    report(Execute(&action, gameState));
                    return;
                }
            }
            for (const auto& zone : gameState.golf.tournamentZones())
            {
                if (zone.contains(tile))
                {
                    ZoneEditAction action(ZoneEditType::removeTournamentZone, zone.id);
                    report(Execute(&action, gameState));
                    return;
                }
            }
            for (const auto& r : gameState.golf.ranges())
            {
                if (r.containsTile(tile))
                {
                    RangeEditAction action(RangeEditType::remove, r.id);
                    report(Execute(&action, gameState));
                    return;
                }
            }
            _status = "Nothing to remove there.";
        }

    public:
        void onOpen() override
        {
            setWidgets(kZonesWidgets);
        }

        void onClose() override
        {
            if (activeTool() != kWidgetIndexNull)
                ToolCancel();
        }

        void onMouseUp(WidgetIndex widgetIndex) override
        {
            auto& gameState = getGameState();
            switch (widgetIndex)
            {
                case WIDX_CLOSE:
                    close();
                    return;
                case WIDX_RANGE_PREV:
                case WIDX_RANGE_NEXT:
                {
                    const auto count = gameState.golf.ranges().size();
                    if (count > 0)
                        _range = (_range + (widgetIndex == WIDX_RANGE_NEXT ? 1 : count - 1)) % count;
                    break;
                }
                case WIDX_PRICE_UP:
                case WIDX_PRICE_DOWN:
                    if (auto* r = range())
                    {
                        RangeEditAction action(
                            RangeEditType::setPrice, r->id, {}, {}, std::max<int64_t>(0, r->bucketPrice + (widgetIndex == WIDX_PRICE_UP ? 50 : -50)));
                        report(Execute(&action, gameState));
                    }
                    break;
                case WIDX_BALLS_UP:
                case WIDX_BALLS_DOWN:
                    if (auto* r = range())
                    {
                        RangeEditAction action(
                            RangeEditType::setBalls, r->id, {}, {}, std::clamp(r->ballsPerBucket + (widgetIndex == WIDX_BALLS_UP ? 5 : -5), 1, 100));
                        report(Execute(&action, gameState));
                    }
                    break;
                case WIDX_AIM_LEFT:
                case WIDX_AIM_RIGHT:
                    if (auto* r = range())
                    {
                        RangeEditAction action(
                            RangeEditType::setDirection, r->id, {}, {},
                            (r->shootingDirectionDegrees + (widgetIndex == WIDX_AIM_RIGHT ? 15 : 345)) % 360);
                        report(Execute(&action, gameState));
                    }
                    break;
                case WIDX_AIM_AUTO:
                    if (auto* r = range())
                    {
                        RangeEditAction action(RangeEditType::autoDirection, r->id);
                        report(Execute(&action, gameState));
                    }
                    break;
                default:
                    if (widgetIndex >= kFirstTool && widgetIndex < kFirstTool + kToolCount)
                    {
                        _first.reset();
                        if (isToolActive(*this, widgetIndex))
                            ToolCancel();
                        else
                            ToolSet(*this, widgetIndex, Tool::crosshair);
                    }
                    break;
            }
            invalidate();
        }

        void onToolDown(WidgetIndex widgetIndex, const ScreenCoordsXY& screenCoords) override
        {
            if (widgetIndex < kFirstTool || widgetIndex >= kFirstTool + kToolCount)
                return;
            const auto position = ScreenGetMapXY(screenCoords, nullptr);
            if (!position.has_value())
                return;
            auto& gameState = getGameState();
            if (widgetIndex == WIDX_REMOVE)
            {
                removeAt(*position);
            }
            else if (widgetIndex == WIDX_HITTING_TILE)
            {
                // A click on a hitting tile takes it away, a click elsewhere in a range makes one.
                const Golf::DrivingRange* inside = nullptr;
                for (const auto& r : gameState.golf.ranges())
                    if (r.containsTile({ position->x >> 5, position->y >> 5 }))
                        inside = &r;
                if (inside == nullptr)
                {
                    _status = "Click inside a driving range.";
                }
                else
                {
                    const Golf::HolePoint centre{ (position->x & ~31) + 16, (position->y & ~31) + 16 };
                    const bool known = std::any_of(inside->hittingTiles.begin(), inside->hittingTiles.end(), [&](const Golf::HolePoint& p) {
                        return p.x == centre.x && p.y == centre.y;
                    });
                    RangeEditAction action(
                        known ? RangeEditType::removeHittingTile : RangeEditType::addHittingTile, inside->id, *position);
                    report(Execute(&action, gameState));
                }
            }
            else if (!_first.has_value())
            {
                _first = *position;
                _status.clear();
            }
            else
            {
                const auto a = *_first;
                _first.reset();
                if (widgetIndex == WIDX_LOT)
                {
                    ZoneEditAction action(ZoneEditType::createLot, 0, a, *position);
                    report(Execute(&action, gameState));
                }
                else if (widgetIndex == WIDX_RANGE)
                {
                    RangeEditAction action(RangeEditType::create, 0, a, *position);
                    report(Execute(&action, gameState));
                }
                else
                {
                    static constexpr Golf::TournamentZoneKind kinds[] = { Golf::TournamentZoneKind::spectator, Golf::TournamentZoneKind::media,
                                                                          Golf::TournamentZoneKind::extraParking,
                                                                          Golf::TournamentZoneKind::grandstand };
                    ZoneEditAction action(
                        ZoneEditType::createTournamentZone, 0, a, *position,
                        static_cast<int32_t>(kinds[widgetIndex - WIDX_SPECTATORS]));
                    report(Execute(&action, gameState));
                }
            }
            invalidate();
        }

        void onToolAbort(WidgetIndex widgetIndex) override
        {
            _first.reset();
            invalidate();
        }

        void onPrepareDraw() override
        {
            for (int32_t i = 0; i < kToolCount; i++)
                setWidgetPressed(kFirstTool + i, isToolActive(*this, kFirstTool + i));
            const bool hasRange = range() != nullptr;
            for (auto widget : { WIDX_PRICE_DOWN, WIDX_PRICE_UP, WIDX_BALLS_DOWN, WIDX_BALLS_UP, WIDX_AIM_LEFT, WIDX_AIM_RIGHT,
                                 WIDX_AIM_AUTO, WIDX_RANGE_PREV, WIDX_RANGE_NEXT })
                setWidgetDisabled(widget, !hasRange);
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
            label(WIDX_RANGE_PREV, "<");
            label(WIDX_RANGE_NEXT, ">");
            label(WIDX_PRICE_DOWN, "-");
            label(WIDX_PRICE_UP, "+");
            label(WIDX_BALLS_DOWN, "-");
            label(WIDX_BALLS_UP, "+");
            label(WIDX_AIM_LEFT, "<");
            label(WIDX_AIM_RIGHT, ">");
            text(widgets[WIDX_AIM_AUTO].left + 6, widgets[WIDX_AIM_AUTO].top + 1, "auto");
            text(12, 104, _first.has_value() ? "Click the opposite corner." : "Pick a zone, then click two corners.", Colour::grey);

            const auto& golf = getGameState().golf;
            text(60, 122, "Driving range " + std::to_string(golf.ranges().empty() ? 0 : _range + 1) + " of " + std::to_string(golf.ranges().size()));
            if (const auto* r = range())
            {
                text(12, 146, "Bucket price: " + std::to_string(r->bucketPrice));
                text(12, 164, "Balls in a bucket: " + std::to_string(r->ballsPerBucket));
                text(12, 182, "Direction: " + std::to_string(r->shootingDirectionDegrees) + (r->autoDirection ? " (auto)" : ""));
                text(12, 200, r->kiosk.has_value() ? "Kiosk: found" : "Kiosk: none inside the area", r->kiosk.has_value() ? Colour::darkGreen : Colour::brightRed);
                text(12, 212, "Hitting tiles: " + std::to_string(r->hittingTiles.size()) + ", distance signs: " + std::to_string(r->signs.size()));
                // DRV-002: a warning that does not stop anything.
                if (Golf::kioskMayBeHit(*r))
                    text(12, 224, "Warning: the kiosk may be hit by balls.", Colour::brightRed);
            }
            if (!_status.empty())
                text(12, 238, _status, Colour::brightRed);
        }
    };

    WindowBase* ZonesOpen()
    {
        auto* windowMgr = GetWindowManager();
        return windowMgr->FocusOrCreate<ZonesWindow>(WindowClass::zones, ScreenCoordsXY(32, 48), kWindowSize);
    }
} // namespace OpenRCT2::Ui::Windows
