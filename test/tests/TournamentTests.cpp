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
#include <string>
#include <vector>
#include <openrct2/golf/GolfObjectives.h>
#include <openrct2/golf/Tournament.h>

using namespace OpenRCT2::Golf;
using GolfTest::SeqRandom;

namespace
{
    // A course that meets the regional tier with room to spare.
    TournamentInputs goodCourse()
    {
        TournamentInputs in;
        in.holes = 18;
        in.totalYards = 6400;
        in.totalPar = 72;
        in.popularity = 700;
        in.golfClubs = 1;
        in.parkingCapacity = 60;
        in.spectatorTiles = 50;
        in.mediaTiles = 10;
        in.extraParkingTiles = 50;
        in.grandstandTiles = 14;
        in.grandstandDistanceToLastGreen = 6;
        return in;
    }

    std::vector<Requirement> unmet(TournamentTier tier, const TournamentInputs& in)
    {
        std::vector<Requirement> out;
        for (const auto& r : checklist(tier, in))
            if (!r.met)
                out.push_back(r);
        return out;
    }
} // namespace

TEST(Tournament, TheRegionalTierAsksForEighteenHolesAndSixThousandYards)
{
    const auto& r = requirementsFor(TournamentTier::regional);
    EXPECT_EQ(r.holes, 18);
    EXPECT_EQ(r.minYards, 6000);
    EXPECT_EQ(r.minPar, 70);
    EXPECT_EQ(r.maxPar, 72);
}

TEST(Tournament, TiersGetHarderAndMoreRewarding)
{
    const auto& local = requirementsFor(TournamentTier::local);
    const auto& regional = requirementsFor(TournamentTier::regional);
    const auto& national = requirementsFor(TournamentTier::national);
    EXPECT_LT(local.minYards, regional.minYards);
    EXPECT_LT(regional.minYards, national.minYards);
    EXPECT_LT(local.minPopularity, regional.minPopularity);
    EXPECT_LT(regional.minPopularity, national.minPopularity);
    EXPECT_LT(local.minParking, regional.minParking);
    EXPECT_LT(regional.minSpectatorTiles, national.minSpectatorTiles);
    EXPECT_LT(local.sponsorMoney, regional.sponsorMoney);
    EXPECT_LT(regional.sponsorMoney, national.sponsorMoney);
    EXPECT_LT(local.popularityBonus, national.popularityBonus);
    EXPECT_STREQ(tierName(TournamentTier::national), "National");
}

TEST(Tournament, ACourseThatMeetsEverythingCanHost)
{
    const auto in = goodCourse();
    EXPECT_TRUE(canHost(TournamentTier::regional, in));
    EXPECT_TRUE(canHost(TournamentTier::local, in));
    EXPECT_FALSE(canHost(TournamentTier::national, in)); // 700 popularity is short of 800
    const auto list = checklist(TournamentTier::regional, in);
    EXPECT_EQ(list.size(), static_cast<size_t>(RequirementId::count));
    for (const auto& r : list)
    {
        EXPECT_TRUE(r.met) << r.label;
        EXPECT_NE(std::string(r.label), "");
    }
}

TEST(Tournament, EachRequirementCanFailOnItsOwn)
{
    struct Case
    {
        RequirementId id;
        void (*spoil)(TournamentInputs&);
    };
    const Case cases[] = {
        { RequirementId::holes, [](TournamentInputs& i) { i.holes = 9; } },
        { RequirementId::length, [](TournamentInputs& i) { i.totalYards = 5900; } },
        { RequirementId::par, [](TournamentInputs& i) { i.totalPar = 73; } },
        { RequirementId::popularity, [](TournamentInputs& i) { i.popularity = 599; } },
        { RequirementId::golfClub, [](TournamentInputs& i) { i.golfClubs = 0; } },
        { RequirementId::parking, [](TournamentInputs& i) { i.parkingCapacity = 39; } },
        { RequirementId::spectatorZone, [](TournamentInputs& i) { i.spectatorTiles = 10; } },
        { RequirementId::mediaZone, [](TournamentInputs& i) { i.mediaTiles = 1; } },
        { RequirementId::extraParkingZone, [](TournamentInputs& i) { i.extraParkingTiles = 5; } },
        { RequirementId::grandstandZone, [](TournamentInputs& i) { i.grandstandTiles = 4; } },
        { RequirementId::grandstandNearGreen, [](TournamentInputs& i) { i.grandstandDistanceToLastGreen = 30; } },
    };
    for (const auto& c : cases)
    {
        auto in = goodCourse();
        c.spoil(in);
        const auto failed = unmet(TournamentTier::regional, in);
        ASSERT_GE(failed.size(), 1u) << static_cast<int>(c.id);
        bool found = false;
        for (const auto& r : failed)
            found |= r.id == c.id;
        EXPECT_TRUE(found) << static_cast<int>(c.id);
        EXPECT_FALSE(canHost(TournamentTier::regional, in));
    }
}

TEST(Tournament, TheParMustBeInsideTheWindow)
{
    auto in = goodCourse();
    for (int par : { 70, 71, 72 })
    {
        in.totalPar = par;
        EXPECT_TRUE(canHost(TournamentTier::regional, in)) << par;
    }
    in.totalPar = 69;
    EXPECT_FALSE(canHost(TournamentTier::regional, in));
    in.totalPar = 73;
    EXPECT_FALSE(canHost(TournamentTier::regional, in));
}

TEST(Tournament, AGrandstandMustExistToBeNearTheGreen)
{
    auto in = goodCourse();
    in.grandstandTiles = 0;
    in.grandstandDistanceToLastGreen = 0;
    EXPECT_FALSE(canHost(TournamentTier::local, in));
}

TEST(Tournament, BookingNeedsAFutureDayAndMetRequirements)
{
    Tournament t;
    EXPECT_EQ(book(t, TournamentTier::regional, goodCourse(), 10, 10, 500), BookResult::dayInThePast);
    EXPECT_EQ(book(t, TournamentTier::regional, goodCourse(), 10, 5, 500), BookResult::dayInThePast);
    EXPECT_EQ(book(t, TournamentTier::regional, goodCourse(), 10, 20, -1), BookResult::negativePrice);
    auto weak = goodCourse();
    weak.golfClubs = 0;
    EXPECT_EQ(book(t, TournamentTier::regional, weak, 10, 20, 500), BookResult::requirementsNotMet);
    EXPECT_EQ(t.phase, TournamentPhase::none);

    ASSERT_EQ(book(t, TournamentTier::regional, goodCourse(), 10, 20, 500), BookResult::ok);
    EXPECT_EQ(t.phase, TournamentPhase::scheduled);
    EXPECT_EQ(t.day, 20u);
    EXPECT_EQ(t.professionals, kProfessionalFieldSize);
    EXPECT_GT(t.spectators, 0);
    EXPECT_EQ(book(t, TournamentTier::local, goodCourse(), 10, 30, 500), BookResult::alreadyBooked);
}

TEST(Tournament, ABookedTournamentCanBeCancelled)
{
    Tournament t;
    ASSERT_EQ(book(t, TournamentTier::local, goodCourse(), 1, 5, 100), BookResult::ok);
    cancel(t);
    EXPECT_EQ(t.phase, TournamentPhase::none);
    ASSERT_EQ(book(t, TournamentTier::local, goodCourse(), 1, 5, 100), BookResult::ok);
    advanceDay(t, 5);
    cancel(t); // too late once it is running
    EXPECT_EQ(t.phase, TournamentPhase::running);
}

TEST(Tournament, TheCourseClosesToTheGeneralPublicOnTheDay)
{
    Tournament t;
    ASSERT_EQ(book(t, TournamentTier::regional, goodCourse(), 10, 20, 500), BookResult::ok);
    EXPECT_FALSE(courseClosedToPublic(t));
    EXPECT_FALSE(advanceDay(t, 19));
    EXPECT_FALSE(courseClosedToPublic(t));
    EXPECT_TRUE(advanceDay(t, 20));
    EXPECT_TRUE(courseClosedToPublic(t));
    EXPECT_FALSE(advanceDay(t, 21)); // already running
}

TEST(Tournament, ProfessionalsAreExcellentPlayers)
{
    SeqRandom rng(4);
    for (auto tier : { TournamentTier::local, TournamentTier::regional, TournamentTier::national })
    {
        for (int i = 0; i < 100; i++)
        {
            const auto s = professionalSkills(tier, rng);
            const int floorValue = 85 + static_cast<int>(tier) * 4;
            for (auto v : { s.longPlay, s.accuracy, s.foolproof, s.physique })
            {
                EXPECT_GE(v, floorValue);
                EXPECT_LE(v, 100);
            }
        }
    }
}

TEST(Tournament, BigEventsDrawMoreSpectators)
{
    EXPECT_GT(spectatorCount(TournamentTier::national, 800), spectatorCount(TournamentTier::regional, 800));
    EXPECT_GT(spectatorCount(TournamentTier::regional, 900), spectatorCount(TournamentTier::regional, 500));
    EXPECT_EQ(spectatorCount(TournamentTier::local, 0), 0);
    EXPECT_EQ(spectatorCount(TournamentTier::local, -50), 0);
}

TEST(Tournament, TheResultFollowsHappinessBottlenecksAndRageQuits)
{
    EXPECT_EQ(tournamentResult(255, 0, 0, 18), 100);
    EXPECT_EQ(tournamentResult(0, 0, 0, 18), 0);
    EXPECT_NEAR(tournamentResult(128, 0, 0, 18), 50, 1);
    EXPECT_LT(tournamentResult(200, 3, 0, 18), tournamentResult(200, 0, 0, 18));
    EXPECT_LT(tournamentResult(200, 0, 2, 18), tournamentResult(200, 0, 0, 18));
    EXPECT_EQ(tournamentResult(200, 18, 9, 18), 0); // floored at zero
    EXPECT_GE(tournamentResult(255, 0, 0, 0), 0);
}

TEST(Tournament, RevenueIsTicketsPlusSponsors)
{
    Tournament t;
    ASSERT_EQ(book(t, TournamentTier::regional, goodCourse(), 1, 10, 800), BookResult::ok);
    EXPECT_EQ(tournamentRevenue(t), static_cast<int64_t>(t.spectators) * 800 + requirementsFor(TournamentTier::regional).sponsorMoney);
}

TEST(Tournament, FinishingPaysOutAndRatesTheEvent)
{
    Tournament t;
    ASSERT_EQ(book(t, TournamentTier::regional, goodCourse(), 1, 10, 800), BookResult::ok);
    finish(t, 255, 0, 0, 18, 700); // not running yet: nothing happens
    EXPECT_EQ(t.phase, TournamentPhase::scheduled);
    advanceDay(t, 10);
    finish(t, 230, 1, 0, 18, 700);
    EXPECT_EQ(t.phase, TournamentPhase::finished);
    EXPECT_GT(t.result, 70);
    EXPECT_EQ(t.revenue, tournamentRevenue(t));
    EXPECT_GT(t.popularityGained, 0);
    EXPECT_LE(t.popularityGained, requirementsFor(TournamentTier::regional).popularityBonus);
    EXPECT_FALSE(courseClosedToPublic(t));

    // A fiasco brings little popularity.
    Tournament bad;
    ASSERT_EQ(book(bad, TournamentTier::regional, goodCourse(), 1, 10, 800), BookResult::ok);
    advanceDay(bad, 10);
    finish(bad, 60, 8, 3, 18, 700);
    EXPECT_LT(bad.popularityGained, t.popularityGained);
    // A finished tournament can be replaced by a new booking.
    EXPECT_EQ(book(bad, TournamentTier::local, goodCourse(), 11, 20, 100), BookResult::ok);
}

TEST(GolfObjectives, ProfitByADate)
{
    GolfObjective o;
    o.type = GolfObjectiveType::profitBy;
    o.target = 50000;
    o.deadlineDay = 100;
    GolfObjectiveProgress p;
    p.today = 50;
    p.totalProfit = 20000;
    EXPECT_EQ(evaluateObjective(o, p), ObjectiveStatus::pending);
    EXPECT_EQ(objectiveProgressPercent(o, p), 40);
    p.totalProfit = 50000;
    EXPECT_EQ(evaluateObjective(o, p), ObjectiveStatus::achieved);
    p.totalProfit = 49000;
    p.today = 101;
    EXPECT_EQ(evaluateObjective(o, p), ObjectiveStatus::failed);
    p.totalProfit = 60000; // reached late still counts: it was reached
    EXPECT_EQ(evaluateObjective(o, p), ObjectiveStatus::achieved);
}

TEST(GolfObjectives, PopularityAndGolfersPerDay)
{
    GolfObjective pop;
    pop.type = GolfObjectiveType::popularityBy;
    pop.target = 700;
    GolfObjectiveProgress p;
    p.popularity = 699;
    EXPECT_EQ(evaluateObjective(pop, p), ObjectiveStatus::pending); // no deadline: never fails
    p.today = 100000;
    EXPECT_EQ(evaluateObjective(pop, p), ObjectiveStatus::pending);
    p.popularity = 700;
    EXPECT_EQ(evaluateObjective(pop, p), ObjectiveStatus::achieved);

    GolfObjective perDay;
    perDay.type = GolfObjectiveType::golfersPerDay;
    perDay.target = 120;
    p = {};
    p.golfersToday = 80;
    EXPECT_EQ(evaluateObjective(perDay, p), ObjectiveStatus::pending);
    EXPECT_EQ(objectiveProgressPercent(perDay, p), 66);
    p.bestGolfersInADay = 125;
    p.golfersToday = 3;
    EXPECT_EQ(evaluateObjective(perDay, p), ObjectiveStatus::achieved);
}

TEST(GolfObjectives, HostingATournamentOfAtLeastTheTier)
{
    GolfObjective o;
    o.type = GolfObjectiveType::hostTournament;
    o.tier = TournamentTier::regional;
    GolfObjectiveProgress p;
    p.tournamentTiersHosted = 1u << static_cast<int>(TournamentTier::local);
    EXPECT_EQ(evaluateObjective(o, p), ObjectiveStatus::pending);
    EXPECT_EQ(objectiveProgressPercent(o, p), 0);
    p.tournamentTiersHosted = 1u << static_cast<int>(TournamentTier::national); // a bigger one counts
    EXPECT_EQ(evaluateObjective(o, p), ObjectiveStatus::achieved);
    EXPECT_EQ(objectiveProgressPercent(o, p), 100);
}

TEST(GolfObjectives, NoObjectiveNeverEnds)
{
    GolfObjective none;
    GolfObjectiveProgress p;
    p.today = 99999;
    p.totalProfit = 1000000;
    EXPECT_EQ(evaluateObjective(none, p), ObjectiveStatus::pending);
    EXPECT_EQ(objectiveProgressPercent(none, p), 0);
    EXPECT_STREQ(objectiveDescription(GolfObjectiveType::none), "");
    EXPECT_NE(std::string(objectiveDescription(GolfObjectiveType::golfersPerDay)), "");
    GolfObjective zero;
    zero.type = GolfObjectiveType::profitBy;
    zero.target = 0;
    EXPECT_EQ(objectiveProgressPercent(zero, p), 100);
}
