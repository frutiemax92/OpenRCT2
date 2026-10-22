/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "GolfTestWorld.h"

#include <gtest/gtest.h>
#include <map>
#include <openrct2/golf/Arrivals.h>
#include <openrct2/golf/GolferNames.h>
#include <set>

using namespace OpenRCT2;
using namespace OpenRCT2::Golf;
using GolfTest::SeqRandom;

TEST(Arrivals, OpeningHoursAreSixToTwentyByDefault)
{
    OpeningHours hours;
    EXPECT_FALSE(isOpenAt(hours, hhmm(5, 59)));
    EXPECT_TRUE(isOpenAt(hours, hhmm(6, 0)));
    EXPECT_TRUE(isOpenAt(hours, hhmm(19, 59)));
    EXPECT_FALSE(isOpenAt(hours, hhmm(20, 0)));
}

TEST(Arrivals, GolfersStopArrivingAnHourBeforeClosing)
{
    OpeningHours hours;
    EXPECT_TRUE(acceptsArrivalsAt(hours, hhmm(18, 59)));
    EXPECT_FALSE(acceptsArrivalsAt(hours, hhmm(19, 0)));
    EXPECT_FALSE(acceptsArrivalsAt(hours, hhmm(5, 0)));
    EXPECT_TRUE(acceptsArrivalsAt(hours, hhmm(6, 0)));
}

TEST(Arrivals, EveryoneMustHaveLeftByClosingTime)
{
    OpeningHours hours;
    EXPECT_FALSE(mustHaveLeft(hours, hhmm(19, 59)));
    EXPECT_TRUE(mustHaveLeft(hours, hhmm(20, 0)));
    EXPECT_TRUE(mustHaveLeft(hours, hhmm(23, 0)));
    EXPECT_FALSE(mustHaveLeft(hours, hhmm(3, 0))); // before opening: nobody is expected, nobody is chased away
}

TEST(Arrivals, FairPricesAreFairAndCheapOnesAreBargains)
{
    EXPECT_EQ(priceFairnessPercent(250 * 9, 9), 100);
    EXPECT_GT(priceFairnessPercent(100 * 9, 9), 100);
    EXPECT_LE(priceFairnessPercent(0, 9), 150);
    EXPECT_EQ(priceFairnessPercent(0, 9), 150);
    EXPECT_LT(priceFairnessPercent(500 * 9, 9), 100);
    EXPECT_EQ(priceFairnessPercent(750 * 9, 9), 0);
    EXPECT_EQ(priceFairnessPercent(100000, 9), 0);
    // More holes make the same fee cheaper.
    EXPECT_GT(priceFairnessPercent(3000, 18), priceFairnessPercent(3000, 9));
}

TEST(Arrivals, TheDayHasABusyMiddleAndQuietEdges)
{
    EXPECT_EQ(timeOfDayPercent(hhmm(6, 0)), 40);
    EXPECT_EQ(timeOfDayPercent(hhmm(10, 0)), 100);
    EXPECT_EQ(timeOfDayPercent(hhmm(14, 0)), 100);
    EXPECT_LT(timeOfDayPercent(hhmm(18, 0)), 70);
    EXPECT_GE(timeOfDayPercent(hhmm(19, 59)), 40);
    EXPECT_GT(timeOfDayPercent(hhmm(7, 0)), timeOfDayPercent(hhmm(6, 0)));
}

TEST(Arrivals, BadWeatherKeepsGolfersAway)
{
    EXPECT_EQ(weatherPercent(Weather::Type::sunny), 100);
    EXPECT_LT(weatherPercent(Weather::Type::cloudy), 100);
    EXPECT_LT(weatherPercent(Weather::Type::rain), weatherPercent(Weather::Type::cloudy));
    EXPECT_LT(weatherPercent(Weather::Type::thunder), weatherPercent(Weather::Type::rain));
    EXPECT_LE(weatherPercent(Weather::Type::blizzard), weatherPercent(Weather::Type::heavyRain));
}

TEST(Arrivals, PopularityDrivesTheArrivalRate)
{
    ArrivalContext c;
    c.greenFee = 250 * 9;
    c.popularity = 1000;
    const int32_t full = arrivalChancePerMillion(c);
    c.popularity = 500;
    const int32_t half = arrivalChancePerMillion(c);
    c.popularity = 0;
    EXPECT_EQ(arrivalChancePerMillion(c), 0);
    EXPECT_GT(full, 0);
    EXPECT_NEAR(static_cast<double>(half) / full, 0.5, 0.01);
    c.popularity = 5000; // out of range is clamped
    EXPECT_EQ(arrivalChancePerMillion(c), full);
}

TEST(Arrivals, PriceTimeAndWeatherScaleTheRate)
{
    ArrivalContext c;
    c.popularity = 800;
    c.greenFee = 250 * 9;
    const int32_t base = arrivalChancePerMillion(c);
    c.greenFee = 600 * 9;
    EXPECT_LT(arrivalChancePerMillion(c), base);
    c.greenFee = 250 * 9;
    c.weather = Weather::Type::heavyRain;
    EXPECT_LT(arrivalChancePerMillion(c), base / 2);
    c.weather = Weather::Type::sunny;
    c.minuteOfDay = hhmm(6, 30);
    EXPECT_LT(arrivalChancePerMillion(c), base);
}

TEST(Arrivals, NoArrivalsWhenClosedFullOrOutsideTheHours)
{
    ArrivalContext c;
    c.popularity = 900;
    EXPECT_GT(arrivalChancePerMillion(c), 0);
    auto closed = c;
    closed.courseOpen = false;
    EXPECT_EQ(arrivalChancePerMillion(closed), 0);
    auto full = c;
    full.golfersOnCourse = full.maxGolfers;
    EXPECT_EQ(arrivalChancePerMillion(full), 0);
    auto night = c;
    night.minuteOfDay = hhmm(23, 0);
    EXPECT_EQ(arrivalChancePerMillion(night), 0);
    auto late = c;
    late.minuteOfDay = hhmm(19, 30);
    EXPECT_EQ(arrivalChancePerMillion(late), 0);
    auto noHoles = c;
    noHoles.holes = 0;
    EXPECT_EQ(arrivalChancePerMillion(noHoles), 0);
}

TEST(Arrivals, TheDefaultMaximumIsFiveHundredGolfers)
{
    EXPECT_EQ(kDefaultMaxGolfers, 500);
    ArrivalContext c;
    EXPECT_EQ(c.maxGolfers, 500);
}

TEST(Arrivals, GroupsArriveAtTheRollRate)
{
    ArrivalContext c;
    c.popularity = 1000;
    c.greenFee = 250 * 9;
    c.minuteOfDay = hhmm(10, 0);
    const int32_t chance = arrivalChancePerMillion(c);
    SeqRandom rng(31);
    int arrivals = 0;
    constexpr int kMinutes = 200000;
    for (int i = 0; i < kMinutes; i++)
        arrivals += groupArrives(c, rng) ? 1 : 0;
    EXPECT_NEAR(static_cast<double>(arrivals) / kMinutes, chance / 1e6, 0.0025);
    // Over a day of 840 opening minutes a busy course sees some twenty groups.
    EXPECT_GT(chance * 840LL / 1000000, 10);
    c.courseOpen = false;
    EXPECT_FALSE(groupArrives(c, rng));
}

TEST(Arrivals, GroupSizesAreOneToFourAndRespectTheSpaceLeft)
{
    SeqRandom rng(8);
    std::map<int, int> sizes;
    for (int i = 0; i < 4000; i++)
        sizes[rollGroupSize(rng)]++;
    for (int size = 1; size <= 4; size++)
        EXPECT_GT(sizes[size], 200) << size;
    EXPECT_EQ(sizes.size(), 4u);
    EXPECT_GT(sizes[4], sizes[1]);
    for (int i = 0; i < 100; i++)
    {
        EXPECT_LE(rollGroupSize(rng, 2), 2);
        EXPECT_EQ(rollGroupSize(rng, 1), 1);
        EXPECT_GE(rollGroupSize(rng, 0), 1);
    }
}

TEST(Arrivals, NewGolfersHaveSkillsMoneyAndEquipment)
{
    SeqRandom rng(2);
    int64_t poorTotal = 0, richTotal = 0, vipTotal = 0;
    double skillSum = 0;
    int maxEquipment = 0;
    constexpr int kN = 500;
    for (int i = 0; i < kN; i++)
    {
        const auto poor = rollNewGolfer(rng, 100, false);
        const auto rich = rollNewGolfer(rng, 1000, false);
        const auto vip = rollNewGolfer(rng, 1000, true);
        poorTotal += poor.money;
        richTotal += rich.money;
        vipTotal += vip.money;
        skillSum += poor.skills.longPlay + poor.skills.accuracy + poor.skills.foolproof + poor.skills.physique;
        EXPECT_LE(poor.skills.longPlay, 100);
        EXPECT_GE(poor.skills.longPlay, 25);
        EXPECT_FALSE(poor.vip);
        EXPECT_TRUE(vip.vip);
        EXPECT_GE(vip.equipmentLevel, 2);
        maxEquipment = std::max<int>(maxEquipment, vip.equipmentLevel);
        EXPECT_LE(poor.equipmentLevel, 3);
        EXPECT_GE(poor.riskTolerance, Fixed{});
        EXPECT_LE(poor.riskTolerance, Fixed::fromInt(1));
    }
    EXPECT_GT(richTotal, poorTotal);
    EXPECT_GT(vipTotal, richTotal * 3);
    EXPECT_NEAR(skillSum / (kN * 4), 50.0, 3.0);
    EXPECT_EQ(maxEquipment, 3);
}

TEST(Arrivals, NamesComeFromTheListsOrFallBack)
{
    SeqRandom rng(1);
    EXPECT_EQ(rollName(rng, {}, {}, 12), "Golfer 12");
    EXPECT_EQ(rollName(rng, { "Ann" }, { "Birdie" }, 1), "Ann Birdie");
    EXPECT_EQ(rollName(rng, { "Ann" }, {}, 1), "Ann");
    EXPECT_EQ(rollName(rng, {}, { "Birdie" }, 1), "Birdie");
    std::map<std::string, int> seen;
    for (int i = 0; i < 200; i++)
        seen[rollName(rng, { "A", "B", "C" }, { "X", "Y" }, 0)]++;
    EXPECT_EQ(seen.size(), 6u);
}

TEST(Arrivals, UniqueNamesNeverRepeatEvenWhenCrowded)
{
    SeqRandom rng(3);
    std::set<std::string> taken;
    const std::vector<std::string> firsts{ "A", "B", "C" };
    const std::vector<std::string> lasts{ "X", "Y" };
    // More golfers than combinations: the first six use every combination, the rest are numbered.
    for (int i = 0; i < 10; i++)
    {
        auto name = rollUniqueName(rng, firsts, lasts, 0, [&](const std::string& n) { return taken.count(n) != 0; });
        EXPECT_TRUE(taken.insert(name).second) << name;
    }
    EXPECT_EQ(taken.size(), 10u);
}

TEST(Arrivals, DefaultNameListsAreLargeAndWithoutDuplicates)
{
    const auto& firsts = defaultFirstNames();
    const auto& lasts = defaultFamilyNames();
    EXPECT_GE(firsts.size(), 100u);
    EXPECT_GE(lasts.size(), 100u);
    EXPECT_EQ(std::set<std::string>(firsts.begin(), firsts.end()).size(), firsts.size());
    EXPECT_EQ(std::set<std::string>(lasts.begin(), lasts.end()).size(), lasts.size());
}
