/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include <cstdint>

namespace OpenRCT2::Golf
{
    // Golfer needs, happiness and thoughts (GLF-010 to GLF-014). Everything is integer arithmetic (ARC-020).

    // GLF-010: each need runs 0..255 on the scale of OpenRCT2 peeps: hunger, thirst and energy are full at 255 and
    // drain; happiness is high at 255; the bladder fills towards 255.
    struct Needs
    {
        // Values with 4 fractional bits (0..4080) so slow drain does not get lost in rounding; the byte value is >> 4.
        int32_t hunger = 255 * 16;
        int32_t thirst = 255 * 16;
        int32_t energy = 255 * 16;
        int32_t happiness = 128 * 16;
        int32_t bladder = 0;

        uint8_t hungerByte() const
        {
            return static_cast<uint8_t>(hunger >> 4);
        }
        uint8_t thirstByte() const
        {
            return static_cast<uint8_t>(thirst >> 4);
        }
        uint8_t energyByte() const
        {
            return static_cast<uint8_t>(energy >> 4);
        }
        uint8_t happinessByte() const
        {
            return static_cast<uint8_t>(happiness >> 4);
        }
        uint8_t bladderByte() const
        {
            return static_cast<uint8_t>(bladder >> 4);
        }
    };

    // GLF-011: drain per game minute, in sixteenths of a point, at average Physique (50). Walking and swinging also cost
    // energy. Physique scales every drain between 150% (skill 0) and 50% (skill 100).
    constexpr int32_t kHungerDrainPerMinute = 16;   // 1.0 per minute
    constexpr int32_t kThirstDrainPerMinute = 24;   // 1.5 per minute
    constexpr int32_t kBladderRisePerMinute = 12;   // 0.75 per minute
    constexpr int32_t kEnergyDrainIdlePerMinute = 4;     // 0.25 per minute
    constexpr int32_t kEnergyDrainWalkingPerMinute = 14; // ~0.9 per minute while walking
    constexpr int32_t kEnergyDrainPerSwing = 10;         // 0.6 per swing

    // Percentage by which Physique scales the drain.
    constexpr int32_t physiqueDrainPercent(int32_t physique)
    {
        const int32_t p = physique < 0 ? 0 : (physique > 100 ? 100 : physique);
        return 150 - p;
    }

    void drainMinutes(Needs& needs, int32_t minutes, int32_t physique, bool walking);
    void drainSwing(Needs& needs, int32_t physique);

    // GLF-012: needs below these (bladder above) send the golfer to a building between holes; the critical levels send
    // them at once, even in the middle of a hole.
    enum class NeedKind : uint8_t
    {
        none,
        hunger,
        thirst,
        energy,
        bladder,
    };
    constexpr int32_t kSeekHungerBelow = 90;
    constexpr int32_t kSeekThirstBelow = 90;
    constexpr int32_t kSeekEnergyBelow = 60;
    constexpr int32_t kSeekBladderAbove = 150;
    constexpr int32_t kCriticalHungerBelow = 30;
    constexpr int32_t kCriticalThirstBelow = 30;
    constexpr int32_t kCriticalEnergyBelow = 20;
    constexpr int32_t kCriticalBladderAbove = 220;

    // The most pressing need that calls for a building, or none. Critical needs are ranked above ordinary ones.
    NeedKind mostUrgentNeed(const Needs& needs, bool* critical = nullptr);

    // GLF-013: events that change happiness, with their deltas in one table.
    enum class HappinessEvent : uint8_t
    {
        goodShot,
        parOrBetter,
        birdie,
        eagle,
        holeInOne,
        distanceSign,
        rangeDistanceSign,  // a ball at the driving range landed near a distance sign (DRV-005)
        sceneryNearHole,
        cleanPaths,
        shortWait,
        waitedAMinute,      // per minute waited at a tee
        bogey,
        doubleBogey,
        worseThanDoubleBogey,
        lostBall,
        ballInHazard,
        holeTooHard,
        hungry,             // per game minute while a need is high
        thirsty,
        bladderFull,
        litter,
        hitByBall,
        crowdedPath,
        highPrices,
        noCart,             // wanted a cart and had to walk (CRT-003)
        sawAnimal,          // saw an animal near the hole (ANM-005)
        washedBall,         // cleaned golf ball at ball washer (PTH-023)
        count,
    };
    // Happiness delta on the 0..255 scale for each event.
    int32_t happinessDelta(HappinessEvent event);
    void applyHappiness(Needs& needs, HappinessEvent event, int32_t times = 1);
    // Needs that are bad enough hurt happiness every game minute.
    void applyNeedPenalties(Needs& needs, int32_t minutes);

    // GLF-014: thoughts shown to the player.
    enum class ThoughtType : uint8_t
    {
        holeTooHard,
        waitedTooLong,
        greatDrive,
        lostBall,
        hungry,
        thirsty,
        needToilet,
        tired,
        pricesTooHigh,
        needCart,
        hitByBall,
        enjoyingTheRound,
        count,
    };
    ThoughtType thoughtForNeed(NeedKind need);
    const char* thoughtName(ThoughtType thought);
} // namespace OpenRCT2::Golf
