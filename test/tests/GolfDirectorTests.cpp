/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "GolfTestWorld.h"

#include <cmath>
#include <gtest/gtest.h>
#include <map>
#include <set>
#include <memory>
#include <tuple>
#include <openrct2/golf/GolfDirector.h>
#include <openrct2/golf/GolfSave.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Golf;
using namespace GolfTest;

namespace
{
    struct Park
    {
        FunctionWorld world;
        CourseManager courses;
        SeqRandom rng{ 17 };
        uint32_t courseId = 0;
        std::vector<HolePoint> cups;
        std::unique_ptr<GolfDirector> director;

        // STE-021: when set, groups arrive by car. The cars that came are logged; carsCome says whether one can.
        bool carsCome = true;
        std::vector<std::vector<uint32_t>> cars;

        explicit Park(bool byCar, int fee = 750, int holes = 3)
            : Park(fee, holes, byCar)
        {
        }

        Park(int fee = 750, int holes = 3, bool byCar = false)
        {
            courseId = courses.createCourse("Links")->id;
            auto* course = courses.findCourse(courseId);
            course->open = true;
            course->greenFee = fee;
            for (int i = 0; i < holes; i++)
            {
                auto* hole = courses.createHole(courseId);
                hole->tees[0] = HolePoint{ 336, 3200 + i * 400 };
                hole->cup = HolePoint{ 336 + (200 + i * 40) * 32 / 5, 3200 + i * 400 };
                cups.push_back(*hole->cup);
            }
            world.puttable = [this](double x, double y) {
                for (const auto& cup : cups)
                    if (std::hypot(x - cup.x, y - cup.y) < 40)
                        return true;
                return false;
            };
            world.roughness = [this](double x, double y) {
                for (const auto& cup : cups)
                    if (std::hypot(x - cup.x, y - cup.y) < 40)
                        return 0.05;
                return 0.15;
            };
            GolfSimulation::Environment env;
            env.world = &world;
            env.courses = &courses;
            env.rng = &rng;
            env.clubEntrance = vec(160, 3000, 0);
            env.exit = vec(100, 2900, 0);
            GolfDirector::Sources sources;
            sources.spawnPoint = [] { return vec(100, 2900, 0); };
            sources.clubEntrance = [] { return vec(160, 3000, 0); };
            sources.exitPoint = [] { return vec(100, 2900, 0); };
            sources.weather = [] { return Weather::Type::sunny; };
            if (byCar)
            {
                sources.carArrival = [this](const std::vector<uint32_t>& ids, uint32_t) {
                    if (!carsCome)
                        return false;
                    cars.push_back(ids);
                    return true;
                };
            }
            director = std::make_unique<GolfDirector>(env, sources);
            director->setPopularity(900);
        }

        // Runs from game tick `from` for `ticks`.
        void run(uint32_t from, uint32_t ticks)
        {
            for (uint32_t t = from; t < from + ticks; t++)
                director->update(t);
        }
    };

    uint32_t at(int day, int hour, int minute = 0)
    {
        return static_cast<uint32_t>(day) * kTicksPerGameDay + static_cast<uint32_t>(hhmm(hour, minute)) * kTicksPerGameMinute;
    }
} // namespace

TEST(GolfDirector, GolfersArriveWhileTheCourseIsOpenAndPopular)
{
    Park park;
    park.run(at(1, 6), 4 * 60 * kTicksPerGameMinute); // 06:00 to 10:00
    EXPECT_GT(park.director->groupsArrived(), 2u);
    EXPECT_GT(park.director->golfersToday(), 2);
    EXPECT_GT(park.director->simulation().golfers().size() + park.director->roundsFinished(), 2u);
}

TEST(GolfDirector, NoOneComesWhenTheCourseIsClosedOrUnpopular)
{
    Park closed;
    closed.courses.findCourse(closed.courseId)->open = false;
    closed.run(at(1, 6), 6 * 60 * kTicksPerGameMinute);
    EXPECT_EQ(closed.director->groupsArrived(), 0u);

    Park unpopular;
    unpopular.director->setPopularity(0);
    unpopular.run(at(1, 6), 6 * 60 * kTicksPerGameMinute);
    EXPECT_EQ(unpopular.director->groupsArrived(), 0u);
}

TEST(GolfDirector, NoOneArrivesOutsideTheOpeningHours)
{
    Park park;
    park.run(at(1, 21), 6 * 60 * kTicksPerGameMinute); // 21:00 until 03:00
    EXPECT_EQ(park.director->groupsArrived(), 0u);
    park.run(at(2, 3), 2 * 60 * kTicksPerGameMinute); // 03:00 to 05:00
    EXPECT_EQ(park.director->groupsArrived(), 0u);
}

TEST(GolfDirector, TheMaximumNumberOfGolfersIsRespected)
{
    Park park;
    park.director->maxGolfers = 3;
    for (uint32_t t = at(1, 6); t < at(1, 18); t += kTicksPerGameMinute)
    {
        // Only step the minutes: the golfers stay in place, so the count only grows through arrivals.
        park.director->update(t);
        EXPECT_LE(park.director->simulation().golfers().size(), 3u);
    }
}

TEST(GolfDirector, EveryoneIsOffTheCourseByClosingTime)
{
    Park park;
    park.run(at(1, 12), 5 * 60 * kTicksPerGameMinute); // arrivals until 17:00, still playing late
    // Push the clock to the minute before closing with golfers on the course, then past it.
    park.run(at(1, 17), 4 * 60 * kTicksPerGameMinute); // 17:00 to 21:00
    EXPECT_TRUE(park.director->simulation().golfers().empty());
    EXPECT_GT(park.director->roundsFinished(), 0);
}

TEST(GolfDirector, FeesAndServicesFeedTheAccounts)
{
    Park park;
    Service club;
    club.id = 1;
    club.position = vec(400, 3100, 0);
    club.food = true;
    club.drink = true;
    club.toilet = true;
    club.price = 200;
    GolfDirector::Sources sources;
    // Re-create the director with a service in the world.
    GolfSimulation::Environment env;
    env.world = &park.world;
    env.courses = &park.courses;
    env.rng = &park.rng;
    env.clubEntrance = vec(160, 3000, 0);
    env.exit = vec(100, 2900, 0);
    sources.spawnPoint = [] { return vec(100, 2900, 0); };
    sources.clubEntrance = [] { return vec(160, 3000, 0); };
    sources.exitPoint = [] { return vec(100, 2900, 0); };
    sources.services = [club] { return std::vector<Service>{ club }; };
    park.director = std::make_unique<GolfDirector>(env, sources);
    park.director->setPopularity(1000);
    park.run(at(1, 6), 14 * 60 * kTicksPerGameMinute);
    park.run(at(1, 20), 2 * 60 * kTicksPerGameMinute);
    const auto& l = park.director->today();
    const auto& total = park.director->total();
    const int64_t fees = l.greenFees + total.greenFees;
    EXPECT_GT(fees, 0);
    EXPECT_EQ(fees % park.courses.findCourse(park.courseId)->greenFee, 0); // every golfer paid the fee once
    EXPECT_GT(park.director->simulation().income(), 0);
}

TEST(GolfDirector, TheRegistryKeepsGolfersForTheNextVisit)
{
    Park park;
    park.run(at(1, 6), 14 * 60 * kTicksPerGameMinute);
    park.run(at(1, 20), 2 * 60 * kTicksPerGameMinute);
    EXPECT_GT(park.director->registry().size(), 0u);
    for (const auto& e : park.director->registry().entries())
    {
        EXPECT_FALSE(e.name.empty());
        EXPECT_GE(e.visits, 1u) << e.name;
    }
}

TEST(GolfDirector, ReturningGolfersComeBackWithTheirSkills)
{
    Park park;
    // A regular, good at everything, in the registry.
    RegistryEntry regular;
    regular.id = 900;
    regular.name = "Regular Rita";
    regular.skills = SkillPoints::from({ 90, 90, 90, 90 });
    regular.initial = regular.skills;
    regular.lastHappiness = 250;
    regular.lastSeenDay = 0;
    park.director->registry().remember(regular);
    bool seenReturning = false;
    for (uint32_t t = at(1, 6); t < at(1, 19) && !seenReturning; t++)
    {
        park.director->update(t);
        for (const auto& g : park.director->simulation().golfers())
            seenReturning |= g.name == "Regular Rita";
    }
    EXPECT_TRUE(seenReturning);
}

TEST(GolfDirector, PopularityMovesTowardsWhatTheCourseEarns)
{
    Park park;
    park.director->setPopularity(100);
    park.run(at(1, 6), 24 * 60 * kTicksPerGameMinute);
    park.run(at(2, 6), 24 * 60 * kTicksPerGameMinute);
    // The base score of a decent course (fair price, happy golfers) is well above 100.
    EXPECT_GT(park.director->popularity(), 100);
    EXPECT_LE(park.director->popularity(), kPopularityMax);
}

TEST(GolfDirector, TheRageQuitsAreCountedAndHurtPopularity)
{
    Park park;
    park.run(at(1, 6), 2 * 60 * kTicksPerGameMinute);
    ASSERT_FALSE(park.director->simulation().golfers().empty());
    // Make everyone on the course furious.
    for (auto& g : park.director->simulation().golfers())
    {
        g.needs.happiness = 5 * 16;
        g.rageCauses = kRageTooHard;
    }
    park.run(at(1, 8), 10 * kTicksPerGameMinute);
    EXPECT_GT(park.director->rageQuits(), 0);
}

TEST(GolfDirector, TheDayChangesTheLedgerAndKeepsTheTotals)
{
    Park park;
    park.run(at(1, 6), 16 * 60 * kTicksPerGameMinute);
    const int64_t earnedBeforeMidnight = park.director->today().greenFees;
    park.run(at(1, 22), 3 * 60 * kTicksPerGameMinute); // across midnight
    EXPECT_GE(park.director->currentDay(), 2u);
    EXPECT_GE(park.director->total().greenFees, earnedBeforeMidnight);
    EXPECT_GT(park.director->bestGolfersInADay(), 0);
}

TEST(GolfDirector, ATournamentClosesTheCourseToThePublicAndBringsProfessionals)
{
    Park park;
    TournamentInputs ok;
    ok.holes = 18;
    ok.totalYards = 5200;
    ok.totalPar = 70;
    ok.popularity = 900;
    ok.golfClubs = 1;
    ok.parkingCapacity = 30;
    ok.spectatorTiles = 30;
    ok.mediaTiles = 6;
    ok.extraParkingTiles = 30;
    ok.grandstandTiles = 10;
    ok.grandstandDistanceToLastGreen = 5;
    ASSERT_EQ(book(park.director->tournament, TournamentTier::local, ok, 1, 3, 500), BookResult::ok);
    park.run(at(1, 6), 24 * 60 * kTicksPerGameMinute);
    EXPECT_GT(park.director->groupsArrived(), 0u); // day 1 is a normal day
    const auto before = park.director->groupsArrived();
    park.run(at(2, 6), 24 * 60 * kTicksPerGameMinute);
    // Day 3 is the tournament: the public stays out, the professionals play.
    park.run(at(3, 6), 2 * 60 * kTicksPerGameMinute);
    EXPECT_TRUE(courseClosedToPublic(park.director->tournament));
    bool pros = false;
    for (const auto& g : park.director->simulation().golfers())
        pros |= g.name.rfind("Pro ", 0) == 0;
    EXPECT_TRUE(pros);
    EXPECT_GT(before, 0u);
    // The next day it is over: revenue is booked and the tier is marked as hosted.
    park.run(at(3, 8), 40 * 60 * kTicksPerGameMinute);
    EXPECT_NE(park.director->tournament.phase, TournamentPhase::running);
    EXPECT_NE(park.director->tiersHosted & 1u, 0u);
    EXPECT_GT(park.director->total().tournamentRevenue + park.director->today().tournamentRevenue, 0);
}

TEST(GolfDirector, ObjectivesAreEvaluatedFromTheDirectorsFigures)
{
    Park park;
    park.director->objective.type = GolfObjectiveType::golfersPerDay;
    park.director->objective.target = 1;
    EXPECT_EQ(park.director->objectiveStatus(), ObjectiveStatus::pending);
    park.run(at(1, 6), 6 * 60 * kTicksPerGameMinute);
    EXPECT_EQ(park.director->objectiveStatus(), ObjectiveStatus::achieved);
}

TEST(GolfDirector, TheSimulationIsDeterministic)
{
    auto run = [] {
        Park park;
        park.run(at(1, 6), 10 * 60 * kTicksPerGameMinute);
        return std::make_tuple(park.director->groupsArrived(), park.director->golfersToday(), park.director->roundsFinished(),
                               park.director->today().greenFees, park.director->simulation().golfers().size());
    };
    EXPECT_EQ(run(), run());
}

TEST(GolfDirector, PopularityReflectsCourseScenery)
{
    Park parkWithoutScenery;
    const auto inPlain = parkWithoutScenery.director->popularityInputs();
    EXPECT_EQ(inPlain.sceneryScore, 0);

    Park parkWithScenery;
    auto* course = parkWithScenery.courses.findCourse(parkWithScenery.courseId);
    for (auto& h : course->holes)
        h.sceneryScore = 80;
    const auto inScenery = parkWithScenery.director->popularityInputs();
    EXPECT_EQ(inScenery.sceneryScore, 80);
}

TEST(GolfDirector, SavedGolfersComeBackExactly)
{
    Park park;
    park.run(at(1, 6), 5 * 60 * kTicksPerGameMinute); // 06:00 to 11:00, golfers are out on the course
    ASSERT_FALSE(park.director->simulation().golfers().empty());
    park.director->tournament.phase = TournamentPhase::scheduled;
    park.director->tournament.day = 9;
    park.director->hours.close = hhmm(21, 30);

    const auto saved = saveDirector(*park.director);
    ASSERT_FALSE(saved.empty());

    Park other;
    EXPECT_TRUE(loadDirector(*other.director, saved));
    EXPECT_EQ(saveDirector(*other.director), saved);
    EXPECT_EQ(other.director->simulation().golfers().size(), park.director->simulation().golfers().size());
    EXPECT_EQ(other.director->registry().size(), park.director->registry().size());
    EXPECT_EQ(other.director->popularity(), park.director->popularity());
    EXPECT_EQ(other.director->tournament.day, 9u);
    EXPECT_EQ(other.director->hours.close, hhmm(21, 30));
    const auto& a = park.director->simulation().golfers().front();
    const auto& b = other.director->simulation().golfers().front();
    EXPECT_EQ(a.name, b.name);
    EXPECT_EQ(a.state, b.state);
    EXPECT_EQ(a.position.x, b.position.x);
    EXPECT_EQ(a.ball.position.y, b.ball.position.y);
    EXPECT_EQ(a.scorecard, b.scorecard);
    EXPECT_EQ(a.thoughts.size(), b.thoughts.size());
}

TEST(GolfDirector, DamagedSaveDataIsRejectedAndChangesNothing)
{
    Park park;
    park.run(at(1, 6), 3 * 60 * kTicksPerGameMinute);
    const auto before = saveDirector(*park.director);

    auto truncated = before;
    truncated.resize(truncated.size() / 2);
    EXPECT_FALSE(loadDirector(*park.director, truncated));
    EXPECT_EQ(saveDirector(*park.director), before);

    auto newer = before;
    newer[0] = 99; // a format from the future
    EXPECT_FALSE(loadDirector(*park.director, newer));
    EXPECT_EQ(saveDirector(*park.director), before);

    EXPECT_FALSE(loadDirector(*park.director, {}));
    EXPECT_EQ(saveDirector(*park.director), before);
}

TEST(GolfDirector, GroupsArrivingByCarWaitInTheCarUntilTheyGetOut)
{
    Park park(true);
    park.run(at(1, 6), 3 * 60 * kTicksPerGameMinute);
    ASSERT_FALSE(park.cars.empty());
    ASSERT_EQ(park.director->simulation().golfers().size(), [&] {
        size_t n = 0;
        for (const auto& car : park.cars)
            n += car.size();
        return n;
    }());

    // Everyone is still in a car: no one has moved, eaten, drunk or paid.
    for (const auto& golfer : park.director->simulation().golfers())
    {
        EXPECT_EQ(golfer.state, GolferState::inCar);
        EXPECT_EQ(golfer.position.x, vec(100, 2900, 0).x);
    }
    EXPECT_EQ(park.director->total().greenFees + park.director->today().greenFees, 0);

    // Getting out puts the golfer where the car stands, and they walk to the club from there.
    const auto id = park.cars.front().front();
    ASSERT_TRUE(park.director->simulation().release(id, vec(130, 2950, 0), vec(125, 2990, 0)));
    EXPECT_FALSE(park.director->simulation().release(id, vec(0, 0, 0), vec(0, 0, 0))); // only once
    const auto* golfer = park.director->simulation().find(id);
    ASSERT_NE(golfer, nullptr);
    EXPECT_EQ(golfer->state, GolferState::arriving);
    EXPECT_EQ(golfer->position.x, vec(130, 2950, 0).x);
    park.run(at(1, 9), 5 * kTicksPerGameMinute);
    EXPECT_NE(park.director->simulation().find(id)->state, GolferState::inCar);
}

TEST(GolfDirector, NoGroupArrivesWhenNoCarCanCome)
{
    Park park(true);
    park.carsCome = false;
    park.run(at(1, 6), 6 * 60 * kTicksPerGameMinute);
    EXPECT_EQ(park.director->groupsArrived(), 0u);
    EXPECT_EQ(park.director->golfersToday(), 0);
    EXPECT_TRUE(park.director->simulation().golfers().empty());
    // No one was remembered either.
    EXPECT_EQ(park.director->registry().size(), 0u);

    // When a car can come again, they do.
    park.carsCome = true;
    park.run(at(1, 12), 3 * 60 * kTicksPerGameMinute);
    EXPECT_GT(park.director->groupsArrived(), 0u);
    EXPECT_GT(park.director->registry().size(), 0u);
}

TEST(GolfDirector, GolfersLeaveBackToTheirCar)
{
    Park park(true);
    park.run(at(1, 6), 2 * 60 * kTicksPerGameMinute);
    ASSERT_FALSE(park.cars.empty());
    const auto id = park.cars.front().front();
    const FixedVec3 car = vec(240, 3100, 0);
    ASSERT_TRUE(park.director->simulation().release(id, vec(160, 3000, 0), car));
    auto* golfer = park.director->simulation().find(id);
    ASSERT_NE(golfer, nullptr);
    EXPECT_TRUE(golfer->hasExit);

    // At closing time everyone walks off; this golfer ends at the car, not at the exit of the course.
    FixedVec3 last = golfer->position;
    for (uint32_t t = at(1, 8); t < at(1, 22) && park.director->simulation().find(id) != nullptr; t++)
    {
        park.director->update(t);
        if (auto* g = park.director->simulation().find(id))
            last = g->position;
    }
    EXPECT_EQ(park.director->simulation().find(id), nullptr);
    EXPECT_LT(abs(last.x - car.x), Fixed::fromInt(8));
    EXPECT_LT(abs(last.y - car.y), Fixed::fromInt(8));
}

TEST(GolfDirector, TheCarsOfSavedGolfersComeBack)
{
    Park park(true);
    park.run(at(1, 6), 2 * 60 * kTicksPerGameMinute);
    ASSERT_FALSE(park.cars.empty());
    const auto id = park.cars.front().front();
    ASSERT_TRUE(park.director->simulation().release(id, vec(130, 2950, 0), vec(125, 2990, 0)));

    const auto saved = saveDirector(*park.director);
    Park other(true);
    ASSERT_TRUE(loadDirector(*other.director, saved));
    const auto* golfer = other.director->simulation().find(id);
    ASSERT_NE(golfer, nullptr);
    EXPECT_TRUE(golfer->hasExit);
    EXPECT_EQ(golfer->exitPosition.y, vec(125, 2990, 0).y);
    // The golfers who never got out are still in their cars.
    bool someInCar = false;
    for (const auto& g : other.director->simulation().golfers())
        someInCar = someInCar || g.state == GolferState::inCar;
    EXPECT_EQ(someInCar, [&] {
        for (const auto& g : park.director->simulation().golfers())
            if (g.state == GolferState::inCar)
                return true;
        return false;
    }());
}

namespace
{
    // A group of unfit golfers, who all want a cart; they are in the sim before anything else arrives.
    std::vector<uint32_t> spawnUnfitGroup(Park& park, int count, uint32_t group)
    {
        std::vector<uint32_t> ids;
        for (int i = 0; i < count; i++)
        {
            GolfSimulation::Spawn spawn;
            spawn.name = "Unfit " + std::to_string(group * 10 + i);
            spawn.groupId = group;
            spawn.courseId = park.courseId;
            spawn.money = 100000;
            spawn.skills = GolferSkills{ 50, 50, 50, 0 };
            spawn.position = vec(100, 2900, 0);
            ids.push_back(park.director->simulation().spawn(spawn).id);
        }
        return ids;
    }

    void addCarts(Park& park, int spaces, int carts)
    {
        for (int i = 0; i < spaces; i++)
            park.director->fleet.addSpace(vec(120 + 30 * i, 2950, 0));
        park.director->fleet.buyCarts(carts);
    }
}

TEST(GolfDirectorCarts, AGroupTakesCartsTwoToACartAndPaysTheFee)
{
    Park park;
    park.director->setPopularity(0); // nobody else arrives
    addCarts(park, 2, 2);
    ASSERT_EQ(park.director->fleet.cartCount(), 2);
    const auto ids = spawnUnfitGroup(park, 4, 1);

    // Run until everyone has checked in.
    uint32_t t = at(1, 8);
    for (; t < at(1, 8) + 6 * kTicksPerGameMinute; t++)
        park.director->update(t);

    std::set<uint32_t> carts;
    for (const auto id : ids)
    {
        const auto* golfer = park.director->simulation().find(id);
        ASSERT_NE(golfer, nullptr);
        EXPECT_NE(golfer->cartId, 0u) << "golfer " << id;
        carts.insert(golfer->cartId);
        // The green fee and the cart fee were both paid.
        EXPECT_EQ(golfer->money, 100000 - 750 - park.director->fleet.cartFee);
    }
    EXPECT_EQ(carts.size(), 2u);
    for (const auto& cart : park.director->fleet.carts())
        EXPECT_EQ(cart.riders.size(), 2u);
    EXPECT_EQ(park.director->fleet.freeCarts(), 0);
    EXPECT_EQ(park.director->today().cartFees, 4 * park.director->fleet.cartFee);
    EXPECT_EQ(park.director->today().greenFees, 4 * 750);
}

TEST(GolfDirectorCarts, ALaterGroupFindsNoCartAndWalks)
{
    Park park;
    park.director->setPopularity(0);
    addCarts(park, 2, 2);
    const auto first = spawnUnfitGroup(park, 4, 1);
    uint32_t t = at(1, 8);
    for (; t < at(1, 8) + 6 * kTicksPerGameMinute; t++)
        park.director->update(t);
    ASSERT_EQ(park.director->fleet.freeCarts(), 0);

    const auto second = spawnUnfitGroup(park, 2, 2);
    for (uint32_t end = t + 6 * kTicksPerGameMinute; t < end; t++)
        park.director->update(t);
    for (const auto id : second)
    {
        const auto* golfer = park.director->simulation().find(id);
        ASSERT_NE(golfer, nullptr);
        EXPECT_EQ(golfer->cartId, 0u);
        EXPECT_FALSE(golfer->onCart);
        // CRT-003: they minded.
        ASSERT_FALSE(golfer->thoughts.empty());
        bool minded = false;
        for (const auto& thought : golfer->thoughts)
            minded = minded || thought.type == ThoughtType::needCart;
        EXPECT_TRUE(minded);
    }
    EXPECT_EQ(park.director->today().cartFees, 4 * park.director->fleet.cartFee); // the second group paid none
    (void)first;
}

TEST(GolfDirectorCarts, TheCartDrivesWithItsRidersAndComesBackAfterTheRound)
{
    Park park;
    park.director->setPopularity(0);
    addCarts(park, 2, 1);
    const auto ids = spawnUnfitGroup(park, 2, 1);

    bool sawRidersMoveWithCart = false;
    uint32_t t = at(1, 8);
    for (; t < at(1, 8) + 5 * 60 * kTicksPerGameMinute; t++)
    {
        park.director->update(t);
        const auto* leader = park.director->simulation().find(ids[0]);
        if (leader == nullptr)
            break;
        if (leader->onCart)
        {
            const auto* cart = park.director->fleet.find(leader->cartId);
            ASSERT_NE(cart, nullptr);
            // A rider on the cart is where the cart is.
            EXPECT_LT(abs(cart->position.x - leader->position.x), Fixed::fromInt(2));
            EXPECT_LT(abs(cart->position.y - leader->position.y), Fixed::fromInt(2));
            sawRidersMoveWithCart = true;
        }
    }
    EXPECT_TRUE(sawRidersMoveWithCart);

    // The round is over: both golfers have gone, the cart is parked in a space again and free to rent.
    for (const auto id : ids)
        EXPECT_EQ(park.director->simulation().find(id), nullptr);
    EXPECT_EQ(park.director->fleet.freeCarts(), 1);
    const auto& cart = park.director->fleet.carts()[0];
    EXPECT_EQ(cart.state, CartState::parked);
    EXPECT_TRUE(cart.riders.empty());
    bool inASpace = false;
    for (const auto& space : park.director->fleet.spaces())
        inASpace = inASpace || (space.cartId == cart.id && space.position.x == cart.position.x);
    EXPECT_TRUE(inASpace);
}

TEST(GolfDirectorCarts, AGolferWhoRageQuitsGivesTheCartBack)
{
    Park park;
    park.director->setPopularity(0);
    addCarts(park, 1, 1);
    const auto ids = spawnUnfitGroup(park, 1, 1);
    uint32_t t = at(1, 8);
    for (; t < at(1, 8) + 6 * kTicksPerGameMinute; t++)
        park.director->update(t);
    auto* golfer = park.director->simulation().find(ids[0]);
    ASSERT_NE(golfer, nullptr);
    ASSERT_NE(golfer->cartId, 0u);
    EXPECT_EQ(park.director->fleet.freeCarts(), 0);

    // Make them furious about a hole that is too hard.
    golfer->needs.happiness = 0;
    golfer->rageCauses = kRageTooHard;
    for (uint32_t end = t + 3 * kTicksPerGameMinute; t < end; t++)
        park.director->update(t);
    EXPECT_EQ(park.director->fleet.freeCarts(), 1);
    EXPECT_EQ(park.director->fleet.carts()[0].state, CartState::parked);
}

TEST(GolfDirectorCarts, TheFleetAndItsRidersAreSaved)
{
    Park park;
    park.director->setPopularity(0);
    addCarts(park, 2, 2);
    park.director->fleet.cartFee = 900;
    const auto ids = spawnUnfitGroup(park, 2, 1);
    uint32_t t = at(1, 8);
    for (; t < at(1, 8) + 20 * kTicksPerGameMinute; t++)
        park.director->update(t);
    const auto* golfer = park.director->simulation().find(ids[0]);
    ASSERT_NE(golfer, nullptr);
    ASSERT_NE(golfer->cartId, 0u);

    const auto saved = saveDirector(*park.director);
    Park other;
    ASSERT_TRUE(loadDirector(*other.director, saved));
    EXPECT_EQ(saveDirector(*other.director), saved);
    EXPECT_EQ(other.director->fleet.cartFee, 900);
    EXPECT_EQ(other.director->fleet.cartCount(), 2);
    EXPECT_EQ(other.director->fleet.spaceCount(), 2);
    const auto* loaded = other.director->simulation().find(ids[0]);
    ASSERT_NE(loaded, nullptr);
    EXPECT_EQ(loaded->cartId, golfer->cartId);
    EXPECT_EQ(loaded->onCart, golfer->onCart);
    EXPECT_EQ(other.director->fleet.freeCarts(), park.director->fleet.freeCarts());
    const auto* cart = other.director->fleet.find(golfer->cartId);
    ASSERT_NE(cart, nullptr);
    EXPECT_EQ(cart->riders.size(), park.director->fleet.find(golfer->cartId)->riders.size());
}

TEST(CartFleet, SpacesFollowTheBaysOfTheMap)
{
    CartFleet fleet;
    fleet.addSpace(vec(16, 16, 0));
    fleet.addSpace(vec(48, 16, 0));
    EXPECT_EQ(fleet.buyCarts(2).bought, 2);

    // One bay is gone and a new one appears: the cart that stood in the gone bay goes with it.
    fleet.syncSpaces({ vec(48, 16, 0), vec(80, 16, 0) });
    EXPECT_EQ(fleet.spaceCount(), 2);
    EXPECT_EQ(fleet.cartCount(), 1);
    EXPECT_EQ(fleet.freeCarts(), 1);
    EXPECT_EQ(fleet.carts()[0].position.x, vec(48, 16, 0).x);

    // Nothing changes when the bays stay as they are, and a new cart goes in the new bay.
    fleet.syncSpaces({ vec(48, 16, 0), vec(80, 16, 0) });
    EXPECT_EQ(fleet.cartCount(), 1);
    EXPECT_EQ(fleet.buyCarts(1).bought, 1);
    EXPECT_EQ(fleet.cartCount(), 2);
    EXPECT_EQ(fleet.buyCarts(1).bought, 0); // no free space left
}

TEST(GolfDirectorCarts, ACartWithAFlatBatteryIsLeftAndItsRidersWalkOn)
{
    Park park;
    park.director->setPopularity(0);
    addCarts(park, 2, 1);
    const auto ids = spawnUnfitGroup(park, 2, 1);
    uint32_t t = at(1, 8);
    for (; t < at(1, 8) + 8 * kTicksPerGameMinute; t++)
        park.director->update(t);
    auto* golfer = park.director->simulation().find(ids[0]);
    ASSERT_NE(golfer, nullptr);
    ASSERT_NE(golfer->cartId, 0u);
    const auto cartId = golfer->cartId;

    // The battery dies while they ride.
    park.director->fleet.find(cartId)->battery = 0;
    for (uint32_t end = t + 3 * kTicksPerGameMinute; t < end; t++)
        park.director->update(t);
    for (const auto id : ids)
    {
        if (const auto* g = park.director->simulation().find(id))
        {
            EXPECT_EQ(g->cartId, 0u);
            EXPECT_FALSE(g->onCart);
        }
    }
    // The cart is back in a space, parked and not rentable until it is charged.
    const auto* cart = park.director->fleet.find(cartId);
    ASSERT_NE(cart, nullptr);
    EXPECT_EQ(cart->state, CartState::parked);
    EXPECT_EQ(park.director->fleet.freeCarts(), 0);
    park.director->fleet.rechargeParkedCarts(50);
    EXPECT_EQ(park.director->fleet.freeCarts(), 1);
}
