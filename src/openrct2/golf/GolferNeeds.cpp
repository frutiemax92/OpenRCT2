/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "GolferNeeds.h"

#include <algorithm>
#include <iterator>

namespace OpenRCT2::Golf
{
    namespace
    {
        constexpr int32_t kMax = 255 * 16;

        int32_t clampNeed(int32_t value)
        {
            return std::clamp(value, 0, kMax);
        }

        int32_t scaled(int32_t perMinute, int32_t minutes, int32_t physique)
        {
            return perMinute * minutes * physiqueDrainPercent(physique) / 100;
        }

        // GLF-013: one table for every happiness modifier.
        constexpr int32_t kHappinessTable[] = {
            +8,   // goodShot
            +15,  // parOrBetter
            +30,  // birdie
            +45,  // eagle
            +80,  // holeInOne
            +10,  // distanceSign
            +2,   // rangeDistanceSign
            +10,  // sceneryNearHole
            +3,   // cleanPaths
            +5,   // shortWait
            -2,   // waitedAMinute
            -10,  // bogey
            -18,  // doubleBogey
            -25,  // worseThanDoubleBogey
            -20,  // lostBall
            -12,  // ballInHazard
            -20,  // holeTooHard
            -1,   // hungry
            -1,   // thirsty
            -2,   // bladderFull
            -3,   // litter
            -25,  // hitByBall
            -2,   // crowdedPath
            -6,   // highPrices
            -8,   // noCart
            +3,   // sawAnimal
            +8,   // washedBall (PTH-023)
        };
        static_assert(std::size(kHappinessTable) == static_cast<size_t>(HappinessEvent::count));
    } // namespace

    void drainMinutes(Needs& needs, int32_t minutes, int32_t physique, bool walking)
    {
        if (minutes <= 0)
            return;
        needs.hunger = clampNeed(needs.hunger - scaled(kHungerDrainPerMinute, minutes, physique));
        needs.thirst = clampNeed(needs.thirst - scaled(kThirstDrainPerMinute, minutes, physique));
        needs.bladder = clampNeed(needs.bladder + scaled(kBladderRisePerMinute, minutes, physique));
        const int32_t energy = walking ? kEnergyDrainWalkingPerMinute : kEnergyDrainIdlePerMinute;
        needs.energy = clampNeed(needs.energy - scaled(energy, minutes, physique));
    }

    void drainSwing(Needs& needs, int32_t physique)
    {
        needs.energy = clampNeed(needs.energy - scaled(kEnergyDrainPerSwing, 1, physique));
    }

    NeedKind mostUrgentNeed(const Needs& needs, bool* critical)
    {
        const int32_t hunger = needs.hungerByte(), thirst = needs.thirstByte(), energy = needs.energyByte(),
                      bladder = needs.bladderByte();
        if (critical != nullptr)
            *critical = false;

        // Critical needs first, the most severe of them.
        struct Entry
        {
            NeedKind kind;
            int32_t severity; // how far past the critical level, in byte units
        };
        Entry critical_[4];
        int32_t n = 0;
        if (hunger < kCriticalHungerBelow)
            critical_[n++] = { NeedKind::hunger, kCriticalHungerBelow - hunger };
        if (thirst < kCriticalThirstBelow)
            critical_[n++] = { NeedKind::thirst, kCriticalThirstBelow - thirst };
        if (energy < kCriticalEnergyBelow)
            critical_[n++] = { NeedKind::energy, kCriticalEnergyBelow - energy };
        if (bladder > kCriticalBladderAbove)
            critical_[n++] = { NeedKind::bladder, bladder - kCriticalBladderAbove };
        if (n > 0)
        {
            if (critical != nullptr)
                *critical = true;
            return std::max_element(critical_, critical_ + n, [](const Entry& a, const Entry& b) { return a.severity < b.severity; })
                ->kind;
        }

        Entry ordinary[4];
        n = 0;
        if (hunger < kSeekHungerBelow)
            ordinary[n++] = { NeedKind::hunger, kSeekHungerBelow - hunger };
        if (thirst < kSeekThirstBelow)
            ordinary[n++] = { NeedKind::thirst, kSeekThirstBelow - thirst };
        if (energy < kSeekEnergyBelow)
            ordinary[n++] = { NeedKind::energy, kSeekEnergyBelow - energy };
        if (bladder > kSeekBladderAbove)
            ordinary[n++] = { NeedKind::bladder, bladder - kSeekBladderAbove };
        if (n == 0)
            return NeedKind::none;
        return std::max_element(ordinary, ordinary + n, [](const Entry& a, const Entry& b) { return a.severity < b.severity; })->kind;
    }

    int32_t happinessDelta(HappinessEvent event)
    {
        const auto index = static_cast<size_t>(event);
        return index < std::size(kHappinessTable) ? kHappinessTable[index] : 0;
    }

    void applyHappiness(Needs& needs, HappinessEvent event, int32_t times)
    {
        needs.happiness = clampNeed(needs.happiness + happinessDelta(event) * 16 * std::max(0, times));
    }

    void applyNeedPenalties(Needs& needs, int32_t minutes)
    {
        if (minutes <= 0)
            return;
        if (needs.hungerByte() < kSeekHungerBelow)
            applyHappiness(needs, HappinessEvent::hungry, minutes);
        if (needs.thirstByte() < kSeekThirstBelow)
            applyHappiness(needs, HappinessEvent::thirsty, minutes);
        if (needs.bladderByte() > kSeekBladderAbove)
            applyHappiness(needs, HappinessEvent::bladderFull, minutes);
    }

    ThoughtType thoughtForNeed(NeedKind need)
    {
        switch (need)
        {
            case NeedKind::hunger:
                return ThoughtType::hungry;
            case NeedKind::thirst:
                return ThoughtType::thirsty;
            case NeedKind::energy:
                return ThoughtType::tired;
            case NeedKind::bladder:
                return ThoughtType::needToilet;
            default:
                return ThoughtType::enjoyingTheRound;
        }
    }

    const char* thoughtName(ThoughtType thought)
    {
        switch (thought)
        {
            case ThoughtType::holeTooHard:
                return "This hole is too hard";
            case ThoughtType::waitedTooLong:
                return "I waited too long";
            case ThoughtType::greatDrive:
                return "Great drive!";
            case ThoughtType::lostBall:
                return "I lost my ball";
            case ThoughtType::hungry:
                return "I'm hungry";
            case ThoughtType::thirsty:
                return "I'm thirsty";
            case ThoughtType::needToilet:
                return "I need the toilet";
            case ThoughtType::tired:
                return "I'm tired";
            case ThoughtType::pricesTooHigh:
                return "The prices are too high";
            case ThoughtType::needCart:
                return "I need a cart";
            case ThoughtType::hitByBall:
                return "Ouch! I was hit by a ball";
            case ThoughtType::enjoyingTheRound:
                return "I'm enjoying the round";
            default:
                return "";
        }
    }
} // namespace OpenRCT2::Golf
