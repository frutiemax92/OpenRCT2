/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <algorithm>
#include <openrct2-ui/interface/Widget.h>
#include <openrct2-ui/interface/Window.h>
#include <openrct2-ui/windows/Windows.h>
#include <openrct2/Context.h>
#include <openrct2/GameState.h>
#include <openrct2/world/Weather.h>
#include <openrct2/drawing/Drawing.Screen.h>
#include <openrct2/drawing/Drawing.h>
#include <openrct2/drawing/Line.h>
#include <openrct2/drawing/Text.h>
#include <openrct2/golf/Clubs.h>
#include <openrct2/golf/ShotDebug.h>
#include <openrct2/interface/Viewport.h>
#include <openrct2/localisation/StringIds.h>
#include <openrct2/ui/WindowManager.h>
#include <string>

namespace OpenRCT2::Ui::Windows
{
    using namespace Golf;

    // Debug shot tool (BAL-060): pick a tee on the map, choose club, power and direction, fire a ball and read its
    // trajectory, bounces, lie, speed and state.
    enum WindowShotToolWidgetIdx : WidgetIndex
    {
        WIDX_BACKGROUND,
        WIDX_TITLE,
        WIDX_CLOSE,
        WIDX_PICK_TEE,
        WIDX_CLUB,
        WIDX_CLUB_UP,
        WIDX_CLUB_DOWN,
        WIDX_POWER,
        WIDX_POWER_UP,
        WIDX_POWER_DOWN,
        WIDX_DIRECTION,
        WIDX_DIRECTION_UP,
        WIDX_DIRECTION_DOWN,
        WIDX_PERFECT,
        WIDX_WIND,
        WIDX_FIRE,
        WIDX_CLEAR,
    };

    static constexpr ScreenSize kWindowSize = { 300, 292 };
    static constexpr int32_t kChartLeft = 10;
    static constexpr int32_t kChartTop = 148;
    static constexpr int32_t kChartWidth = 280;
    static constexpr int32_t kChartHeight = 80;
    static constexpr int32_t kPowerStep = 5;
    static constexpr int32_t kDirectionStep = 5;

    // clang-format off
    static constexpr auto kShotToolWidgets = makeWidgets(
        makeWindowShim(STR_SHOT_TOOL_TITLE, kWindowSize),
        makeWidget({ 10, 20 }, { 280, 14 }, WidgetType::button, WindowColour::secondary, STR_SHOT_TOOL_PICK_TEE),
        makeSpinnerWidgets({ 120, 40 }, { 170, 12 }, WidgetType::spinner, WindowColour::secondary),
        makeSpinnerWidgets({ 120, 56 }, { 170, 12 }, WidgetType::spinner, WindowColour::secondary),
        makeSpinnerWidgets({ 120, 72 }, { 170, 12 }, WidgetType::spinner, WindowColour::secondary),
        makeWidget({ 10, 90 }, { 135, 12 }, WidgetType::checkbox, WindowColour::secondary, STR_SHOT_TOOL_PERFECT),
        makeWidget({ 155, 90 }, { 135, 12 }, WidgetType::checkbox, WindowColour::secondary, STR_SHOT_TOOL_WIND),
        makeWidget({ 10, 108 }, { 135, 14 }, WidgetType::button, WindowColour::secondary, STR_SHOT_TOOL_FIRE),
        makeWidget({ 155, 108 }, { 135, 14 }, WidgetType::button, WindowColour::secondary, STR_SHOT_TOOL_CLEAR)
    );
    // clang-format on

    static const char* StateName(BallState state)
    {
        switch (state)
        {
            case BallState::Teed:
                return "teed";
            case BallState::InFlight:
                return "in flight";
            case BallState::Bouncing:
                return "bouncing";
            case BallState::Rolling:
                return "rolling";
            case BallState::AtRest:
                return "at rest";
            case BallState::Holed:
                return "holed";
            case BallState::Lost:
                return "lost in water";
            case BallState::OutOfBounds:
                return "out of bounds";
        }
        return "?";
    }

    static std::string Yards(Fixed units)
    {
        return std::to_string(unitsToYards(units).round()) + " yd";
    }

    class ShotToolWindow final : public Window
    {
    public:
        void onOpen() override
        {
            setWidgets(kShotToolWidgets);
            initScrollWidgets();
            width = minWidth = maxWidth = kWindowSize.width;
            height = minHeight = maxHeight = kWindowSize.height;
            widgets[WIDX_BACKGROUND].right = width - 1;
            widgets[WIDX_BACKGROUND].bottom = height - 1;
        }

        void onClose() override
        {
            if (isToolActive(*this, WIDX_PICK_TEE))
                ToolCancel();
        }

        void onMouseUp(WidgetIndex widgetIndex) override
        {
            auto& debug = GetShotDebug();
            switch (widgetIndex)
            {
                case WIDX_CLOSE:
                    close();
                    break;
                case WIDX_PICK_TEE:
                    if (isToolActive(*this, WIDX_PICK_TEE))
                        ToolCancel();
                    else
                        ToolSet(*this, WIDX_PICK_TEE, Tool::crosshair);
                    break;
                case WIDX_PERFECT:
                    debug.settings.perfect = !debug.settings.perfect;
                    break;
                case WIDX_WIND:
                    debug.settings.wind = !debug.settings.wind;
                    break;
                case WIDX_FIRE:
                    debug.fire();
                    break;
                case WIDX_CLEAR:
                    debug.clear();
                    break;
                case WIDX_CLUB_UP:
                    ChangeClub(1);
                    break;
                case WIDX_CLUB_DOWN:
                    ChangeClub(-1);
                    break;
                case WIDX_POWER_UP:
                    debug.settings.power = clamp(debug.settings.power + Fixed::ratio(kPowerStep, 100), Fixed{}, Fixed::fromInt(1));
                    break;
                case WIDX_POWER_DOWN:
                    debug.settings.power = clamp(debug.settings.power - Fixed::ratio(kPowerStep, 100), Fixed{}, Fixed::fromInt(1));
                    break;
                case WIDX_DIRECTION_UP:
                    debug.settings.directionDegrees = (debug.settings.directionDegrees + kDirectionStep) % 360;
                    break;
                case WIDX_DIRECTION_DOWN:
                    debug.settings.directionDegrees = (debug.settings.directionDegrees - kDirectionStep + 360) % 360;
                    break;
            }
            invalidate();
        }

        void onToolDown(WidgetIndex widgetIndex, const ScreenCoordsXY& screenCoords) override
        {
            if (widgetIndex != WIDX_PICK_TEE)
                return;
            const auto position = ScreenGetMapXY(screenCoords, nullptr);
            if (position.has_value())
                GetShotDebug().setTee(*position);
            invalidate();
        }

        void onToolAbort(WidgetIndex widgetIndex) override
        {
            invalidate();
        }

        void onUpdate() override
        {
            // Redraw while a ball is flying so the numbers and the chart follow it.
            if (GetShotDebug().hasShot() && !GetShotDebug().finished())
                invalidate();
        }

        void onPrepareDraw() override
        {
            setCheckboxValue(WIDX_PERFECT, GetShotDebug().settings.perfect);
            setCheckboxValue(WIDX_WIND, GetShotDebug().settings.wind);
            setWidgetPressed(WIDX_PICK_TEE, isToolActive(*this, WIDX_PICK_TEE));
            setWidgetDisabled(WIDX_FIRE, !GetShotDebug().tee().has_value());
        }

        void onDraw(Drawing::RenderTarget& rt) override
        {
            WindowDrawWidgets(*this, rt);
            auto& debug = GetShotDebug();

            auto text = [&](int32_t x, int32_t y, const std::string& s) { drawText(rt, windowPos + ScreenCoordsXY{ x, y }, s, { colours[1] }); };
            text(10, 41, "Club");
            text(10, 57, "Power");
            text(10, 73, "Direction");
            text(123, 41, clubInfo(debug.settings.club).name);
            text(123, 57, std::to_string((debug.settings.power * 100).round()) + "%");
            text(123, 73, std::to_string(debug.settings.directionDegrees) + " deg (0 = +x, 90 = +y)");

            if (const auto& tee = debug.tee(); tee.has_value())
                text(10, 128, "Tee " + std::to_string(tee->x.floor() / 32) + ", " + std::to_string(tee->y.floor() / 32));
            else
                text(10, 128, "No tee chosen");
            {
                const auto& wind = getGameState().wind;
                text(150, 128, "Wind " + std::to_string(wind.strength / 10) + "% to " + Weather::windCompassName(wind.direction));
            }

            DrawChart(rt);

            if (debug.hasShot())
            {
                text(10, 234, std::string("State: ") + StateName(debug.state()) + ",  speed " + std::to_string(debug.currentSpeed().toDouble()).substr(0, 5) + " u/tick");
                text(10, 247, "Carry " + Yards(debug.carry()) + ",  distance " + Yards(debug.distance()) + ",  apex " + std::to_string(debug.maxHeight().floor()) + " z");
                std::string lie = "Lie: surface " + std::to_string(debug.lieSurface());
                if (debug.lieOnPath())
                    lie += " (path, hardpan)";
                if (debug.lieIsHazard())
                    lie += " (hazard)";
                if (debug.liePuttable())
                    lie += " (green)";
                text(10, 260, lie + ",  bounces " + std::to_string(debug.impacts().size()));
            }
        }

    private:
        void ChangeClub(int32_t delta)
        {
            auto& debug = GetShotDebug();
            const int32_t count = static_cast<int32_t>(kClubCount);
            const int32_t next = (static_cast<int32_t>(debug.settings.club) + delta + count) % count;
            debug.settings.club = static_cast<ClubId>(next);
        }

        // Side view of the flight: distance from the tee along the horizontal axis, height up. Impacts are marked.
        void DrawChart(Drawing::RenderTarget& rt)
        {
            const auto origin = windowPos + ScreenCoordsXY{ kChartLeft, kChartTop };
            const auto bottomRight = origin + ScreenCoordsXY{ kChartWidth, kChartHeight };
            GfxDrawLine(rt, { origin, { bottomRight.x, origin.y } }, Drawing::PaletteIndex::pi10);
            GfxDrawLine(rt, { { origin.x, bottomRight.y }, bottomRight }, Drawing::PaletteIndex::pi10);
            GfxDrawLine(rt, { origin, { origin.x, bottomRight.y } }, Drawing::PaletteIndex::pi10);
            GfxDrawLine(rt, { { bottomRight.x, origin.y }, bottomRight }, Drawing::PaletteIndex::pi10);

            auto& debug = GetShotDebug();
            if (!debug.hasShot() || !debug.tee().has_value() || debug.samples().size() < 2)
                return;

            // One scale for both axes so the arc keeps its shape.
            const double reach = std::max(160.0, debug.distance().toDouble() * 1.05 + 8.0);
            const double scale = std::min((kChartWidth - 8) / reach, (kChartHeight - 8) / std::max(40.0, debug.maxHeight().toDouble() * 1.1));
            const auto tee = *debug.tee();
            auto project = [&](const FixedVec3& p) {
                const double along = length(FixedVec3{ p.x - tee.x, p.y - tee.y, Fixed{} }).toDouble();
                const double up = (p.z - tee.z).toDouble();
                return ScreenCoordsXY{ origin.x + 4 + static_cast<int32_t>(along * scale),
                                       bottomRight.y - 4 - static_cast<int32_t>(up * scale) };
            };
            const auto& samples = debug.samples();
            for (size_t i = 1; i < samples.size(); i++)
            {
                const auto a = project(samples[i - 1].position);
                const auto b = project(samples[i].position);
                const bool onGround = samples[i].state == BallState::Rolling || samples[i].state == BallState::AtRest;
                GfxDrawLine(rt, { a, b }, onGround ? Drawing::PaletteIndex::pi61 : Drawing::PaletteIndex::pi20);
            }
            for (const auto& impact : debug.impacts())
            {
                const auto p = project(impact.position);
                GfxDrawLine(rt, { { p.x - 2, p.y }, { p.x + 2, p.y } }, Drawing::PaletteIndex::pi172);
                GfxDrawLine(rt, { { p.x, p.y - 2 }, { p.x, p.y + 2 } }, Drawing::PaletteIndex::pi172);
            }
        }
    };

    WindowBase* ShotToolOpen()
    {
        auto* windowMgr = GetWindowManager();
        return windowMgr->FocusOrCreate<ShotToolWindow>(WindowClass::shotTool, ScreenCoordsXY(32, 48), kWindowSize);
    }
} // namespace OpenRCT2::Ui::Windows
