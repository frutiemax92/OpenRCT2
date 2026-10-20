/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../core/EnumUtils.hpp"

#include <array>
#include <cstdint>

namespace OpenRCT2::Drawing
{
    enum class FilterPaletteID : int32_t;
}

namespace OpenRCT2::Weather
{
    enum class Type : uint8_t
    {
        sunny,
        partiallyCloudy,
        cloudy,
        rain,
        heavyRain,
        thunder,
        snow,
        heavySnow,
        blizzard,
        count
    };

    static constexpr auto kNumWeatherTypes = EnumValue(Type::count);
    static constexpr auto kNumClimateMonths = 8;
    static constexpr auto kWeatherDistSize = 23;

    struct Pattern
    {
        int8_t baseTemperature;
        int8_t randomBias;
        Type distribution[kWeatherDistSize];
    };

    struct TemperatureThresholds
    {
        int8_t cold;
        int8_t warm;
    };

    struct Climate
    {
        std::array<Pattern, kNumClimateMonths> patterns;
        TemperatureThresholds itemThresholds;
    };

    enum class EffectType : uint8_t
    {
        none,
        rain,
        storm,
        snow,
        blizzard,
    };

    enum class Level
    {
        none,
        light,
        heavy,
    };

    struct State
    {
        Type weatherType;
        int8_t temperature;
        EffectType weatherEffect;
        uint8_t weatherGloom;
        Level level;
    };

    // The wind (BAL-031): a direction and a strength that drift slowly towards random targets chosen from the current
    // weather. Part of the game state and saved with the climate.
    struct WindState
    {
        int16_t direction = 0;       // degrees the wind blows towards, 0 = +x, 90 = +y, 0..359
        uint16_t strength = 0;       // 0..1000 per mille of the strongest wind
        int16_t targetDirection = 0;
        uint16_t targetStrength = 0;
        uint32_t randomState = 0x9E3779B9u; // private generator, so the wind never disturbs the scenario random numbers
    };

    constexpr int32_t kWindStrengthMax = 1000;
    // Wind speed in world units per tick at full strength (1.5 u/tick is about 8.5 m/s).
    constexpr int32_t kWindMaxSpeedHundredths = 150;
    constexpr uint32_t kWindUpdateInterval = 128; // ticks between steps
    constexpr int32_t kWindDirectionStep = 3;     // degrees per step
    constexpr int32_t kWindStrengthStep = 15;     // per mille per step

    struct WindRange
    {
        uint16_t min;
        uint16_t max;
    };
    // Strength range the weather asks for.
    WindRange windRangeFor(Type weather);
    // One step of the wind: moves towards the targets and picks new ones when it arrives.
    void stepWind(WindState& wind, Type weather);
    // The wind steps every kWindUpdateInterval ticks.
    void updateWind(WindState& wind, Type weather, uint32_t ticks);
    // Compass name of a direction, like "NE" (north is -y on the map).
    const char* windCompassName(int32_t directionDegrees);

    extern uint16_t gLightningFlash;

    int32_t celsiusToFahrenheit(int32_t celsius);
    void reset();
    void update();
    void updateSound();
    void stopWeatherSound();
    void forceWeather(Type weather);

    bool isDry();
    bool isRaining();
    bool isTransitioningToSnow();
    bool isSnowing();
    bool isSnowingHeavily();
    bool isPrecipitating();
    bool hasWeatherEffect();
    Drawing::FilterPaletteID getWeatherGloomPaletteId(const State& state);
    uint32_t getWeatherSpriteId(Type weatherType);
} // namespace OpenRCT2::Weather
