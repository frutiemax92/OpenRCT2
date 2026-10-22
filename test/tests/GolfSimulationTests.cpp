/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "GolfTestWorld.h"

#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <map>
#include <openrct2/golf/GolfSimulation.h>
#include <openrct2/golf/ObstacleGrid.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Golf;
using namespace GolfTest;

namespace
{
    // A flat course: greens around every cup, nothing else on the map.
    struct Course3
    {
        FunctionWorld world;
        CourseManager courses;
        SeqRandom rng{ 5 };
        uint32_t courseId = 0;
        std::vector<HolePoint> cups;

        Course3(std::vector<int> yards = { 150, 300, 380 })
        {
            courseId = courses.createCourse("Test links")->id;
            auto* course = courses.findCourse(courseId);
            course->open = true;
            course->greenFee = 2500;
            int y = 3200;
            for (int len : yards)
            {
                auto* hole = courses.createHole(courseId);
                hole->tees[0] = HolePoint{ 336, y };
                hole->cup = HolePoint{ 336 + len * 32 / 5, y };
                cups.push_back(*hole->cup);
                y += 400;
            }
            world.puttable = [this](double x, double yy) {
                for (const auto& cup : cups)
                    if (std::hypot(x - cup.x, yy - cup.y) < 40)
                        return true;
                return false;
            };
            world.roughness = [this](double x, double yy) {
                for (const auto& cup : cups)
                    if (std::hypot(x - cup.x, yy - cup.y) < 40)
                        return 0.05;
                return 0.15;
            };
        }

        GolfSimulation::Environment environment()
        {
            GolfSimulation::Environment env;
            env.world = &world;
            env.courses = &courses;
            env.rng = &rng;
            env.clubEntrance = vec(160, 3000, 0);
            env.exit = vec(100, 2900, 0);
            return env;
        }
    };

    GolfSimulation::Spawn aGolfer(const Course3& c, uint32_t group = 0, std::string name = "")
    {
        GolfSimulation::Spawn s;
        s.name = name;
        s.groupId = group;
        s.skills = { 60, 60, 60, 60 };
        s.courseId = c.courseId;
        s.position = vec(100, 2900, 0);
        return s;
    }

    struct Tally
    {
        std::map<SimEvent::Type, int> count;
        std::vector<SimEvent> all;
        void collect(GolfSimulation& sim)
        {
            for (const auto& e : sim.takeEvents())
            {
                count[e.type]++;
                all.push_back(e);
            }
        }
    };

    // Runs until every golfer has left or the tick budget ends; returns the ticks used.
    uint32_t runUntilEmpty(GolfSimulation& sim, Tally& tally, uint32_t maxTicks = 600000, uint32_t start = 1000)
    {
        uint32_t t = start;
        for (; t < start + maxTicks && !sim.golfers().empty(); t++)
        {
            sim.tick(t);
            if (t % 50 == 0)
                tally.collect(sim);
        }
        tally.collect(sim);
        return t - start;
    }
} // namespace

TEST(GolfSimulation, ASoloGolferPlaysTheWholeCourseAndLeaves)
{
    Course3 c;
    GolfSimulation sim(c.environment());
    auto& g = sim.spawn(aGolfer(c));
    EXPECT_EQ(g.state, GolferState::arriving);
    const uint32_t id = g.id;
    Tally tally;
    runUntilEmpty(sim, tally);
    ASSERT_TRUE(sim.golfers().empty()) << "the golfer is still playing";

    auto left = sim.takeLeft();
    ASSERT_EQ(left.size(), 1u);
    const auto& done = left[0];
    EXPECT_EQ(done.id, id);
    EXPECT_FALSE(done.rageQuit);
    EXPECT_EQ(done.scorecard.size(), 3u);
    for (size_t i = 0; i < 3; i++)
    {
        EXPECT_NE(done.scorecard[i], 127) << "hole " << i;
        EXPECT_GT(done.parCard[i], 0);
    }
    EXPECT_EQ(done.parCard[0], 3);
    EXPECT_EQ(done.parCard[1], 4);
    EXPECT_EQ(done.parCard[2], 4);
    EXPECT_GE(done.strokesTotal, 3 + 3 + 3);
    EXPECT_LE(done.strokesTotal, 3 * (4 + 4 + 4)) << "a hole is capped at par + 4";
    EXPECT_EQ(tally.count[SimEvent::Type::teedOff], 3);
    EXPECT_EQ(tally.count[SimEvent::Type::holeFinished], 3);
    EXPECT_EQ(tally.count[SimEvent::Type::roundFinished], 1);
    EXPECT_EQ(tally.count[SimEvent::Type::paidFee], 1);
    EXPECT_GE(tally.count[SimEvent::Type::shotHit], 9);
    EXPECT_EQ(sim.income(), 2500);
    EXPECT_EQ(done.money, 10000 - 2500);
}

TEST(GolfSimulation, SkillsGrowWithEveryHolePlayed)
{
    Course3 c;
    GolfSimulation sim(c.environment());
    sim.spawn(aGolfer(c));
    Tally tally;
    runUntilEmpty(sim, tally);
    const auto left = sim.takeLeft();
    ASSERT_EQ(left.size(), 1u);
    EXPECT_GT(left[0].skills.longPlay, left[0].initialSkills.longPlay);
    EXPECT_GT(left[0].skills.accuracy, left[0].initialSkills.accuracy);
    EXPECT_GT(left[0].skills.foolproof, left[0].initialSkills.foolproof);
    EXPECT_GT(left[0].skills.physique, left[0].initialSkills.physique);
}

TEST(GolfSimulation, AGroupOfFourPlaysTogetherWithoutTeeingOffAtOnce)
{
    Course3 c;
    GolfSimulation sim(c.environment());
    for (int i = 0; i < 4; i++)
        sim.spawn(aGolfer(c, 77));
    Tally tally;
    runUntilEmpty(sim, tally);
    ASSERT_TRUE(sim.golfers().empty());
    EXPECT_EQ(sim.takeLeft().size(), 4u);
    EXPECT_EQ(tally.count[SimEvent::Type::paidFee], 4);
    EXPECT_EQ(tally.count[SimEvent::Type::holeFinished], 12);
    EXPECT_EQ(sim.income(), 4 * 2500);
}

TEST(GolfSimulation, MembersOfAGroupNeverSwingAtTheSameTime)
{
    Course3 c;
    GolfSimulation sim(c.environment());
    for (int i = 0; i < 3; i++)
        sim.spawn(aGolfer(c, 5));
    bool overlap = false;
    for (uint32_t t = 1000; t < 1000 + 400000 && !sim.golfers().empty(); t++)
    {
        sim.tick(t);
        int busy = 0;
        for (const auto& g : sim.golfers())
            if (g.state == GolferState::swinging || g.state == GolferState::putting || g.state == GolferState::watchingBall)
                busy++;
        overlap |= busy > 1;
    }
    EXPECT_FALSE(overlap);
}

TEST(GolfSimulation, AGolferWaitsAsideForTheGroupAndTheFarthestBallPlaysNext)
{
    Course3 c;
    GolfSimulation sim(c.environment());
    for (int i = 0; i < 3; i++)
        sim.spawn(aGolfer(c, 5));
    auto dist = [](const FixedVec3& a, const FixedVec3& b) {
        return std::hypot(a.x.toDouble() - b.x.toDouble(), a.y.toDouble() - b.y.toDouble());
    };
    std::map<uint32_t, GolferState> before;
    int waited = 0;
    int turnsChecked = 0;
    bool twoOnTee = false;
    bool groupNotBeside = false;
    bool sawFollowing = false;
    bool waitingAwayFromItsSpot = false;
    bool mateOnTheBall = false;
    int shotsWithGroupWaiting = 0;
    uint32_t longestPreparing = 0;
    bool walkedWhileMateHit = false;
    bool notFarthest = false;
    for (uint32_t t = 1000; t < 1000 + 400000 && !sim.golfers().empty(); t++)
    {
        sim.tick(t);
        for (const auto& g : sim.golfers())
        {
            const auto was = before.count(g.id) ? before[g.id] : g.state;
            if (g.state == GolferState::waitingForTurn)
                waited++;
            if (g.state == GolferState::walkingToBall && was != GolferState::walkingToBall)
            {
                const auto cup = c.cups[g.holeIndex];
                auto distance = [&](const Golfer& o) {
                    return std::hypot(o.ball.position.x.toDouble() - cup.x, o.ball.position.y.toDouble() - cup.y);
                };
                for (const auto& other : sim.golfers())
                {
                    if (other.id == g.id || other.holeIndex != g.holeIndex)
                        continue;
                    if (other.state == GolferState::swinging || other.state == GolferState::putting
                        || other.state == GolferState::watchingBall)
                        walkedWhileMateHit = true;
                    if (was == GolferState::waitingForTurn && other.state == GolferState::waitingForTurn
                        && distance(other) > distance(g))
                        notFarthest = true;
                }
                turnsChecked++;
            }
        }
        int preparing = 0;
        for (const auto& g : sim.golfers())
        {
            before[g.id] = g.state;
            if (g.state == GolferState::preparingShot)
            {
                preparing++;
                longestPreparing = std::max(longestPreparing, g.stateTicks);
            }
        }
        twoOnTee |= preparing > 1;
        // Golfers who walk to a waiting spot show walking, and the ones standing on it show waiting.
        for (const auto& g : sim.golfers())
        {
            if (g.state == GolferState::followingGroup)
                sawFollowing = true;
            if (g.state == GolferState::waitingForTurn && g.hasTurnSpot
                && dist(g.position, g.turnSpot) > 64)
                waitingAwayFromItsSpot = true;
            // Nobody stands on the ball of the golfer who is about to swing.
            if (g.state == GolferState::swinging && g.stateTicks == 1)
                for (const auto& other : sim.golfers())
                    if (other.id != g.id && other.holeIndex == g.holeIndex && other.state != GolferState::holeFinished
                        && dist(other.position, g.ball.position) < 16)
                        mateOnTheBall = true;
        }
        // The group waits beside the ball of the golfer who plays.
        for (const auto& g : sim.golfers())
        {
            if (g.state != GolferState::swinging || g.stateTicks != 1 || g.score.shots == 0)
                continue;
            for (const auto& other : sim.golfers())
                if (other.id != g.id && other.holeIndex == g.holeIndex && other.state == GolferState::waitingForTurn)
                {
                    shotsWithGroupWaiting++;
                    if (std::hypot(other.position.x.toDouble() - g.ball.position.x.toDouble(),
                            other.position.y.toDouble() - g.ball.position.y.toDouble()) > 120)
                        groupNotBeside = true;
                }
        }
    }
    EXPECT_TRUE(sawFollowing);
    EXPECT_FALSE(waitingAwayFromItsSpot);
    EXPECT_FALSE(mateOnTheBall);
    EXPECT_FALSE(groupNotBeside);
    EXPECT_GT(shotsWithGroupWaiting, 3);
    EXPECT_FALSE(twoOnTee); // one golfer of the group at a time gets ready on the tee
    EXPECT_LT(longestPreparing, 3000u); // and nobody stands there stuck
    EXPECT_GT(waited, 0); // after the tee shot the golfer stands aside instead of walking to the ball
    EXPECT_GT(turnsChecked, 3);
    EXPECT_FALSE(walkedWhileMateHit);
    EXPECT_FALSE(notFarthest);
}

TEST(GolfSimulation, SeveralGroupsOfThreeAllGetToPlayTheFirstHole)
{
    Course3 c;
    GolfSimulation sim(c.environment());
    for (uint32_t group = 1; group <= 3; group++)
        for (int i = 0; i < 3; i++)
            sim.spawn(aGolfer(c, group));
    std::map<uint32_t, int> shots;
    for (uint32_t t = 1000; t < 1000 + 120000 && !sim.golfers().empty(); t++)
    {
        sim.tick(t);
        if (t % 50 == 0)
            for (const auto& e : sim.takeEvents())
                if (e.type == SimEvent::Type::shotHit)
                    shots[e.golferId]++;
    }
    // A group that has begun its tee shots does not get shut out by a group that arrives at the tee in between.
    EXPECT_EQ(shots.size(), 9u);
}

TEST(GolfSimulation, AGroupMovesToTheNextHoleTogetherAfterTheLastGolferHolesOut)
{
    Course3 c;
    GolfSimulation sim(c.environment());
    for (int i = 0; i < 3; i++)
        sim.spawn(aGolfer(c, 7));
    bool wentAhead = false;
    bool moved = false;
    for (uint32_t t = 1000; t < 1000 + 200000 && !sim.golfers().empty(); t++)
    {
        sim.tick(t);
        uint32_t lowest = 99;
        uint32_t highest = 0;
        for (const auto& g : sim.golfers())
        {
            lowest = std::min(lowest, g.holeIndex);
            highest = std::max(highest, g.holeIndex);
        }
        wentAhead |= highest > lowest; // one golfer on the next hole while a mate is still on the last
        moved |= highest > 0;
    }
    EXPECT_TRUE(moved);
    EXPECT_FALSE(wentAhead);
}

TEST(GolfSimulation, TwoGroupsTeeOffWithoutAFixedSlotBetweenThem)
{
    Course3 c;
    GolfSimulation sim(c.environment());
    sim.spawn(aGolfer(c, 1));
    sim.spawn(aGolfer(c, 2));
    std::map<uint32_t, std::vector<uint32_t>> teeMinutes; // golfer id -> minutes of tee offs
    Tally tally;
    for (uint32_t t = 1000; t < 1000 + 700000 && !sim.golfers().empty(); t++)
    {
        sim.tick(t);
        for (const auto& e : sim.takeEvents())
        {
            if (e.type == SimEvent::Type::teedOff)
                teeMinutes[e.golferId].push_back(gameMinuteAt(t));
        }
    }
    ASSERT_EQ(teeMinutes.size(), 2u);
    ASSERT_GE(teeMinutes[1].size(), 3u);
    for (size_t h = 0; h < 3; h++)
    {
        const auto a = teeMinutes[1][h], b = teeMinutes[2][h];
        // No fixed slot between groups: the second one shoots as soon as the first is out of the way, so it may share the minute.
        EXPECT_LE(std::max(a, b) - std::min(a, b), 8u) << "hole " << h;
    }
}

TEST(GolfSimulation, AGroupDoesNotWaitForTheSlotWhenNobodyIsInTheWay)
{
    Course3 c;
    // A very long manual slot that used to hold the second group on the tee.
    c.courses.findCourse(c.courseId)->holes[0].schedule.manual = true;
    c.courses.findCourse(c.courseId)->holes[0].schedule.manualSlotMinutes = 25;
    GolfSimulation sim(c.environment());
    sim.spawn(aGolfer(c, 1));
    sim.spawn(aGolfer(c, 2));
    std::map<uint32_t, uint32_t> firstTeeOff; // golfer id -> minute of the first tee off
    for (uint32_t t = 1000; t < 1000 + 120000; t++)
    {
        sim.tick(t);
        for (const auto& e : sim.takeEvents())
            if (e.type == SimEvent::Type::teedOff && firstTeeOff.find(e.golferId) == firstTeeOff.end())
                firstTeeOff[e.golferId] = gameMinuteAt(t);
    }
    ASSERT_EQ(firstTeeOff.size(), 2u);
    EXPECT_LT(firstTeeOff[2] - firstTeeOff[1], 10u);
}

TEST(GolfSimulation, AHungryGolferEatsAtTheClubBetweenHoles)
{
    Course3 c;
    auto env = c.environment();
    Service club;
    club.id = 1;
    club.position = vec(400, 3100, 0);
    club.food = true;
    club.drink = true;
    club.toilet = true;
    club.minutes = 2;
    club.price = 300;
    env.services.push_back(club);
    GolfSimulation sim(env);
    auto& g = sim.spawn(aGolfer(c));
    g.needs.hunger = 70 * 16; // below the threshold from the start
    Tally tally;
    runUntilEmpty(sim, tally);
    const auto left = sim.takeLeft();
    ASSERT_EQ(left.size(), 1u);
    EXPECT_EQ(tally.count[SimEvent::Type::usedFacility], 1);
    EXPECT_EQ(left[0].money, 10000 - 2500 - 300);
    EXPECT_EQ(sim.income(), 2500 + 300);
    EXPECT_GT(left[0].needs.hungerByte(), 150); // fed
    EXPECT_EQ(tally.count[SimEvent::Type::holeFinished], 3);
}

TEST(GolfSimulation, ProShopUpgradesGolferEquipment)
{
    Course3 c;
    auto env = c.environment();
    Service club;
    club.id = 1;
    club.position = vec(400, 3100, 0);
    club.food = true;
    club.proShop = true;
    club.price = 300;
    env.services.push_back(club);
    GolfSimulation sim(env);
    auto spawn = aGolfer(c);
    spawn.equipmentLevel = 1;
    spawn.money = 10000;
    auto& g = sim.spawn(spawn);
    g.needs.hunger = 70 * 16;
    Tally tally;
    runUntilEmpty(sim, tally);
    const auto left = sim.takeLeft();
    ASSERT_EQ(left.size(), 1u);
    EXPECT_EQ(left[0].equipmentLevel, 2);
    EXPECT_EQ(tally.count[SimEvent::Type::usedFacility], 1);
    EXPECT_EQ(left[0].money, 10000 - 2500 - 300 - 2000);
    EXPECT_EQ(sim.income(), 2500 + 300 + 2000);
}

TEST(GolfSimulation, ACriticalNeedInTheMiddleOfAHoleSendsTheGolferAway)
{
    Course3 c;
    auto env = c.environment();
    Service restroom;
    restroom.id = 2;
    restroom.position = vec(700, 3300, 0);
    restroom.toilet = true;
    restroom.minutes = 1;
    env.services.push_back(restroom);
    GolfSimulation sim(env);
    auto& g = sim.spawn(aGolfer(c));
    g.needs.bladder = 240 * 16;
    Tally tally;
    runUntilEmpty(sim, tally);
    EXPECT_GE(tally.count[SimEvent::Type::usedFacility], 1);
    const auto left = sim.takeLeft();
    ASSERT_EQ(left.size(), 1u);
    EXPECT_EQ(tally.count[SimEvent::Type::holeFinished], 3); // and still finished the round
}

TEST(GolfSimulation, WithoutABuildingTheNeedJustWaits)
{
    Course3 c;
    GolfSimulation sim(c.environment());
    auto& g = sim.spawn(aGolfer(c));
    g.needs.hunger = 10 * 16;
    Tally tally;
    runUntilEmpty(sim, tally);
    EXPECT_EQ(tally.count[SimEvent::Type::usedFacility], 0);
    EXPECT_EQ(tally.count[SimEvent::Type::holeFinished], 3);
}

TEST(GolfSimulation, AnUnhappyGolferWithACourseProblemRageQuits)
{
    Course3 c;
    GolfSimulation sim(c.environment());
    auto& g = sim.spawn(aGolfer(c));
    g.needs.happiness = 20 * 16;
    g.rageCauses = kRageLongWait;
    Tally tally;
    runUntilEmpty(sim, tally);
    const auto left = sim.takeLeft();
    ASSERT_EQ(left.size(), 1u);
    EXPECT_TRUE(left[0].rageQuit);
    EXPECT_EQ(tally.count[SimEvent::Type::rageQuit], 1);
    EXPECT_LT(tally.count[SimEvent::Type::holeFinished], 3);
    const auto& event = *std::find_if(tally.all.begin(), tally.all.end(), [](const SimEvent& e) { return e.type == SimEvent::Type::rageQuit; });
    EXPECT_GE(event.insult, 0);
    EXPECT_LT(event.insult, kInsultCount);
    EXPECT_GE(event.value, 1);
}

TEST(GolfSimulation, UnhappinessWithoutACourseProblemDoesNotMakeAGolferLeave)
{
    Course3 c;
    GolfSimulation sim(c.environment());
    auto& g = sim.spawn(aGolfer(c));
    g.needs.happiness = 5 * 16;
    g.rageCauses = 0;
    Tally tally;
    runUntilEmpty(sim, tally);
    EXPECT_EQ(tally.count[SimEvent::Type::rageQuit], 0);
    EXPECT_EQ(tally.count[SimEvent::Type::holeFinished], 3);
}

TEST(GolfSimulation, ARageQuitInsultsGolfersWithinThreeTiles)
{
    Course3 c;
    GolfSimulation sim(c.environment());
    const uint32_t angryId = sim.spawn(aGolfer(c)).id;
    const uint32_t nearId = sim.spawn(aGolfer(c)).id;
    const uint32_t farId = sim.spawn(aGolfer(c)).id;
    // (Spawning can move the golfers in memory, so they are looked up by id.)
    sim.find(nearId)->position = vec(110, 2900, 0);
    sim.find(farId)->position = vec(100 + 10 * 32, 2900, 0);
    sim.find(angryId)->needs.happiness = 10 * 16;
    sim.find(angryId)->rageCauses = kRageTooHard;
    EXPECT_EQ(sim.find(nearId)->needs.happinessByte(), 128);
    sim.tick(kTicksPerGameMinute); // a game minute boundary: the angry golfer storms off
    // The exit is right where the golfers stand, so the angry one is gone at once.
    const auto left = sim.takeLeft();
    ASSERT_EQ(left.size(), 1u);
    EXPECT_EQ(left[0].id, angryId);
    EXPECT_TRUE(left[0].rageQuit);
    EXPECT_EQ(sim.find(nearId)->needs.happinessByte(), 128 - kInsultHappinessLoss);
    EXPECT_EQ(sim.find(farId)->needs.happinessByte(), 128);
}

TEST(GolfSimulation, AClosedCourseSendsGolfersHome)
{
    Course3 c;
    c.courses.findCourse(c.courseId)->open = false;
    GolfSimulation sim(c.environment());
    sim.spawn(aGolfer(c));
    Tally tally;
    runUntilEmpty(sim, tally);
    EXPECT_EQ(tally.count[SimEvent::Type::teedOff], 0);
    EXPECT_EQ(tally.count[SimEvent::Type::paidFee], 0);
    EXPECT_EQ(sim.takeLeft().size(), 1u);
    EXPECT_EQ(sim.income(), 0);
}

TEST(GolfSimulation, AGolferWhoCannotAffordTheFeeLeavesUnhappy)
{
    Course3 c;
    GolfSimulation sim(c.environment());
    auto spawn = aGolfer(c);
    spawn.money = 1000;
    sim.spawn(spawn);
    Tally tally;
    runUntilEmpty(sim, tally);
    const auto left = sim.takeLeft();
    ASSERT_EQ(left.size(), 1u);
    EXPECT_EQ(tally.count[SimEvent::Type::paidFee], 0);
    EXPECT_LT(left[0].needs.happinessByte(), 128);
    EXPECT_EQ(sim.income(), 0);
    EXPECT_FALSE(left[0].thoughts.empty());
    EXPECT_EQ(left[0].thoughts.back().type, ThoughtType::pricesTooHigh);
}

TEST(GolfSimulation, VipsPlayForFree)
{
    Course3 c;
    GolfSimulation sim(c.environment());
    auto spawn = aGolfer(c);
    spawn.vip = true;
    spawn.money = 0;
    sim.spawn(spawn);
    Tally tally;
    runUntilEmpty(sim, tally);
    EXPECT_EQ(tally.count[SimEvent::Type::holeFinished], 3);
    EXPECT_EQ(sim.income(), 0);
}

TEST(GolfSimulation, HolesLearnHowLongGroupsTakeToPlayThem)
{
    Course3 c;
    GolfSimulation sim(c.environment());
    for (uint32_t g = 1; g <= 3; g++)
        sim.spawn(aGolfer(c, g));
    Tally tally;
    runUntilEmpty(sim, tally);
    const auto* course = c.courses.findCourse(c.courseId);
    for (const auto& hole : course->holes)
    {
        EXPECT_EQ(hole.schedule.samples, 3);
        EXPECT_GT(hole.schedule.measuredPlayTenths, 0);
    }
}

TEST(GolfSimulation, ATooHardHoleLowersHappinessAndCanCauseARageQuit)
{
    Course3 c({ 150, 300 });
    // Water everywhere beside the first fairway makes it hard.
    c.world.hazard = [](double, double y) { return std::fabs(y - 3200) > 60 && std::fabs(y - 3200) < 250; };
    GolfSimulation sim(c.environment());
    auto spawn = aGolfer(c);
    spawn.skills = { 5, 5, 5, 5 };
    auto& g = sim.spawn(spawn);
    g.needs.happiness = 60 * 16;
    Tally tally;
    runUntilEmpty(sim, tally);
    EXPECT_GE(tally.count[SimEvent::Type::tooHard], 1);
    const auto left = sim.takeLeft();
    ASSERT_EQ(left.size(), 1u);
    bool thoughtHard = false;
    for (const auto& t : left[0].thoughts)
        thoughtHard |= t.type == ThoughtType::holeTooHard;
    EXPECT_TRUE(thoughtHard);
}

TEST(GolfSimulation, TheWholeRoundIsDeterministic)
{
    auto run = [] {
        Course3 c;
        GolfSimulation sim(c.environment());
        for (int i = 0; i < 3; i++)
            sim.spawn(aGolfer(c, 9));
        Tally tally;
        runUntilEmpty(sim, tally);
        uint64_t hash = 1469598103934665603ull;
        for (auto& g : sim.takeLeft())
        {
            for (auto s : g.scorecard)
                hash = (hash ^ static_cast<uint8_t>(s)) * 1099511628211ull;
            hash = (hash ^ static_cast<uint64_t>(g.strokesTotal)) * 1099511628211ull;
            hash = (hash ^ static_cast<uint64_t>(g.needs.happiness)) * 1099511628211ull;
        }
        return hash;
    };
    EXPECT_EQ(run(), run());
}

TEST(GolfSimulation, GolfersGetHungryOverTheRoundAndTheClockRunsInGameMinutes)
{
    Course3 c;
    GolfSimulation sim(c.environment());
    const uint32_t id = sim.spawn(aGolfer(c)).id;
    const int start = sim.find(id)->needs.hungerByte();
    for (uint32_t t = 1000; t < 1000 + 8 * kTicksPerGameMinute; t++)
        sim.tick(t);
    const auto* after = sim.find(id);
    ASSERT_NE(after, nullptr);
    // About a point a minute at average physique; eight minutes pass between the first and the last tick.
    EXPECT_LE(after->needs.hungerByte(), start - 6);
    EXPECT_GE(after->needs.hungerByte(), start - 10);
}

TEST(GolfSimulation, OnAMiniPuttCourseOnlyTheMiniPuttClubIsUsedAndOnlyAccuracyGrows)
{
    Course3 c({ 12, 18, 25 }); // short holes, in yards
    c.courses.findCourse(c.courseId)->kind = CourseKind::miniPutt;
    // The whole course is carpet.
    c.world.puttable = [](double, double) { return true; };
    c.world.roughness = [](double, double) { return 0.03; };
    GolfSimulation sim(c.environment());
    const uint32_t id = sim.spawn(aGolfer(c)).id;
    std::map<ClubId, int> clubs;
    Tally tally;
    for (uint32_t t = 1000; t < 1000 + 400000 && !sim.golfers().empty(); t++)
    {
        sim.tick(t);
        if (auto* g = sim.find(id); g != nullptr && g->state == GolferState::swinging)
            clubs[g->decision.club]++;
        if (auto* g = sim.find(id); g != nullptr && g->state == GolferState::putting)
            clubs[g->decision.club]++;
        if (t % 50 == 0)
            tally.collect(sim);
    }
    tally.collect(sim);
    ASSERT_TRUE(sim.golfers().empty());
    EXPECT_EQ(clubs.size(), 1u);
    EXPECT_GT(clubs[ClubId::miniPutter], 0);
    const auto left = sim.takeLeft();
    ASSERT_EQ(left.size(), 1u);
    EXPECT_GT(left[0].skills.accuracy, left[0].initialSkills.accuracy);
    EXPECT_EQ(left[0].skills.longPlay, left[0].initialSkills.longPlay);
    EXPECT_EQ(left[0].skills.foolproof, left[0].initialSkills.foolproof);
    EXPECT_EQ(left[0].skills.physique, left[0].initialSkills.physique);
    EXPECT_EQ(tally.count[SimEvent::Type::holeFinished], 3);
}

TEST(GolfSimulation, AMiniPuttCourseWithWallsAroundItIsPlayable)
{
    // MPT acceptance: a mini putt with walls is playable, balls bounce off the walls and Accuracy increases.
    class WalledWorld final : public IBallWorld
    {
    public:
        SurfaceSample surfaceAt(Fixed, Fixed) const override
        {
            SurfaceSample s;
            s.puttable = true;
            s.roughness = Fixed::ratio(3, 100);
            s.bounciness = Fixed::ratio(6, 10);
            return s;
        }
        bool obstacleAt(const FixedVec3& p, Fixed radius, ObstacleContact& contact) const override
        {
            return grid.query(p, radius, contact);
        }
        ObstacleGrid grid;
    } world;
    // A wall around the playing field: x 100..900, y 3100..3500.
    auto wall = [&](uint32_t id, int cx, int cy, int hx, int hy) {
        Obstacle o;
        o.id = id;
        o.shape = ObstacleShape::box;
        o.centre = vec(cx, cy, 0);
        o.halfX = Fixed::fromInt(hx);
        o.halfY = Fixed::fromInt(hy);
        o.height = Fixed::fromInt(8);
        o.restitution = Fixed::ratio(7, 10);
        o.friction = Fixed::ratio(1, 10);
        world.grid.add(o);
    };
    wall(1, 500, 3090, 420, 6);
    wall(2, 500, 3510, 420, 6);
    wall(3, 92, 3300, 6, 220);
    wall(4, 908, 3300, 6, 220);

    CourseManager courses;
    const uint32_t courseId = courses.createCourse("Mini")->id;
    auto* course = courses.findCourse(courseId);
    course->kind = CourseKind::miniPutt;
    course->open = true;
    course->greenFee = 500;
    for (int i = 0; i < 3; i++)
    {
        auto* hole = courses.createHole(courseId);
        hole->tees[0] = HolePoint{ 160, 3200 + i * 100 };
        hole->cup = HolePoint{ 800, 3200 + i * 100 };
    }
    SeqRandom rng(21);
    GolfSimulation::Environment env;
    env.world = &world;
    env.courses = &courses;
    env.rng = &rng;
    env.clubEntrance = vec(160, 3150, 0);
    env.exit = vec(120, 3150, 0);
    GolfSimulation sim(env);
    GolfSimulation::Spawn spawn;
    spawn.skills = { 50, 50, 50, 50 };
    spawn.courseId = courseId;
    spawn.position = vec(130, 3150, 0);
    sim.spawn(spawn);
    Tally tally;
    runUntilEmpty(sim, tally);
    EXPECT_EQ(tally.count[SimEvent::Type::holeFinished], 3);
    const auto left = sim.takeLeft();
    ASSERT_EQ(left.size(), 1u);
    EXPECT_GT(left[0].skills.accuracy, left[0].initialSkills.accuracy);
}

TEST(GolfSimulation, GolferSitsOnBenchWhenLowOnEnergy)
{
    Course3 c;
    auto env = c.environment();
    Bench bench;
    bench.id = 1;
    bench.position = vec(400, 3100, 0);
    bench.occupants = 0;
    env.benches.push_back(bench);
    GolfSimulation sim(env);
    auto spawn = aGolfer(c);
    auto& g = sim.spawn(spawn);
    g.needs.energy = 50 * 16;
    Tally tally;
    runUntilEmpty(sim, tally);
    EXPECT_GE(tally.count[SimEvent::Type::restedOnBench], 1);
    const auto left = sim.takeLeft();
    ASSERT_EQ(left.size(), 1u);
    EXPECT_GT(left[0].needs.energy, 50 * 16);
}

TEST(GolfSimulation, BenchCapacityIsLimitedToTwoOccupants)
{
    Course3 c;
    auto env = c.environment();
    Bench bench;
    bench.id = 1;
    bench.position = vec(400, 3100, 0);
    bench.occupants = 2; // already full
    env.benches.push_back(bench);
    GolfSimulation sim(env);
    auto spawn = aGolfer(c);
    auto& g = sim.spawn(spawn);
    g.needs.energy = 50 * 16;
    Tally tally;
    runUntilEmpty(sim, tally);
    EXPECT_EQ(tally.count[SimEvent::Type::restedOnBench], 0);
}

TEST(GolfSimulation, OnlyOneGolferOfAGroupStandsOnTheTee)
{
    Course3 c;
    auto env = c.environment();
    GolfSimulation sim(env);
    for (int i = 0; i < 3; i++)
    {
        auto s = aGolfer(c);
        s.groupId = 2;
        sim.spawn(s);
    }

    int32_t mostOnTee = 0;
    for (uint32_t t = 1000; t < 200000 && !sim.golfers().empty(); t++)
    {
        sim.tick(t);
        int32_t onTee = 0;
        for (const auto& g : sim.golfers())
            if (!g.finished && g.holeIndex == 0 && g.score.shots == 0 && g.state == GolferState::waitingForTeeTime)
                onTee++;
        mostOnTee = std::max(mostOnTee, onTee);
    }
    EXPECT_LE(mostOnTee, 1);
}

TEST(GolfSimulation, GolferDisposesTrashInNearbyBin)
{
    Course3 c;
    auto env = c.environment();
    Service foodTruck;
    foodTruck.id = 1;
    foodTruck.position = vec(400, 3100, 0);
    foodTruck.food = true;
    foodTruck.price = 200;
    env.services.push_back(foodTruck);

    LitterBin bin;
    bin.id = 1;
    bin.position = vec(410, 3110, 0);
    bin.trashCount = 0;
    env.bins.push_back(bin);

    GolfSimulation sim(env);
    auto spawn = aGolfer(c);
    auto& g = sim.spawn(spawn);
    g.needs.hunger = 70 * 16;
    Tally tally;
    runUntilEmpty(sim, tally);
    EXPECT_EQ(tally.count[SimEvent::Type::littered], 0);
    EXPECT_GT(sim.environment().bins[0].trashCount, 0u);
}

TEST(GolfSimulation, GolferLittersWhenNoBinNearby)
{
    Course3 c;
    auto env = c.environment();
    Service foodTruck;
    foodTruck.id = 1;
    foodTruck.position = vec(400, 3100, 0);
    foodTruck.food = true;
    foodTruck.price = 200;
    env.services.push_back(foodTruck);

    GolfSimulation sim(env);
    auto spawn = aGolfer(c);
    auto& g = sim.spawn(spawn);
    g.needs.hunger = 70 * 16;
    Tally tally;
    runUntilEmpty(sim, tally);
    EXPECT_GE(tally.count[SimEvent::Type::littered], 1);
}

TEST(GolfSimulation, SceneryNearHoleRaisesHappiness)
{
    Course3 c;
    auto* course = c.courses.findCourse(c.courseId);
    course->holes[0].sceneryScore = 50;

    auto env = c.environment();
    GolfSimulation sim(env);
    auto spawn = aGolfer(c);
    sim.spawn(spawn);
    Tally tally;
    runUntilEmpty(sim, tally);
    const auto left = sim.takeLeft();
    ASSERT_EQ(left.size(), 1u);
    EXPECT_TRUE(std::any_of(left[0].thoughts.begin(), left[0].thoughts.end(), [](const Thought& t) {
        return t.type == ThoughtType::enjoyingTheRound;
    }));
}

// A golfer walks 1.96 yards per second on a path, 1.5 on normal ground and slower on rough going. A game second is 200 / 60
// ticks and a yard is 6.4 world units.
TEST(GolfSimulationWalking, SpeedDependsOnTheGroundUnderTheGolfer)
{
    const auto yardsPerSecond = [](int32_t hundredthsPerTick) {
        return hundredthsPerTick / 100.0 * 40.0 / 6.4;
    };
    EXPECT_NEAR(yardsPerSecond(Golf::kWalkSpeedPathHundredths), 1.96, 0.05);
    EXPECT_NEAR(yardsPerSecond(Golf::kWalkSpeedHundredths), 1.5, 0.01);
    EXPECT_LT(yardsPerSecond(Golf::kWalkSpeedHarshHundredths), 1.22);
    // A 5 yard tile (32 units) takes 3.33 seconds on normal ground.
    EXPECT_NEAR(32.0 / (Golf::kWalkSpeedHundredths / 100.0) / 40.0, 3.33, 0.01);

    SurfaceSample fairway;
    fairway.roughness = Fixed::ratio(15, 100);
    EXPECT_EQ(Golf::walkSpeedHundredthsOn(fairway), Golf::kWalkSpeedHundredths);

    SurfaceSample path = fairway;
    path.path = true;
    EXPECT_EQ(Golf::walkSpeedHundredthsOn(path), Golf::kWalkSpeedPathHundredths);

    SurfaceSample sand = fairway;
    sand.hazard = true;
    sand.roughness = Fixed::ratio(90, 100);
    EXPECT_EQ(Golf::walkSpeedHundredthsOn(sand), Golf::kWalkSpeedHarshHundredths);

    SurfaceSample deepRough = fairway;
    deepRough.roughness = Fixed::ratio(75, 100);
    EXPECT_EQ(Golf::walkSpeedHundredthsOn(deepRough), Golf::kWalkSpeedHarshHundredths);

    SurfaceSample slope = fairway;
    slope.normal = { Fixed::ratio(60, 100), Fixed{}, Fixed::ratio(80, 100) };
    EXPECT_EQ(Golf::walkSpeedHundredthsOn(slope), Golf::kWalkSpeedHarshHundredths);

    // A path wins over a slope: paths are laid to be walked.
    slope.path = true;
    EXPECT_EQ(Golf::walkSpeedHundredthsOn(slope), Golf::kWalkSpeedPathHundredths);
}

namespace
{
    struct WalkTrial
    {
        uint32_t ticks = 0;
        uint32_t ticksOnHarshGround = 0;
        bool arrived = false;
    };

    // Walks a golfer from `start` to the club entrance of the environment and reports how long it took and for how many ticks it
    // stood on harsh ground on the way.
    WalkTrial walkToTheClub(Course3& c, GolfSimulation::Environment env, FixedVec3 start)
    {
        GolfSimulation sim(env);
        auto spawn = aGolfer(c);
        spawn.position = start;
        sim.spawn(spawn);
        WalkTrial trial;
        for (uint32_t t = 1000; t < 6000; t++)
        {
            sim.tick(t);
            const auto& g = sim.golfers().front();
            if (g.state != GolferState::walkingToCheckIn && g.state != GolferState::arriving)
            {
                trial.arrived = true;
                break;
            }
            trial.ticks++;
            if (walkSpeedHundredthsOn(c.world.surfaceAt(g.position.x, g.position.y)) == kWalkSpeedHarshHundredths)
                trial.ticksOnHarshGround++;
        }
        return trial;
    }
} // namespace

TEST(GolfSimulationWalking, AGolferWalksRoundHarshGroundWhenThatIsQuicker)
{
    // A strip of sand, 100 units long and 12 wide, lies on the straight way from (100, 2960) to the club at (300, 2960).
    const auto sand = [](double x, double y) { return x > 120 && x < 220 && y > 2954 && y < 2966; };
    Course3 c;
    c.world.hazard = sand;
    c.world.roughness = [sand](double x, double y) { return sand(x, y) ? 0.9 : 0.15; };
    auto env = c.environment();
    env.clubEntrance = vec(300, 2960, 0);
    const auto trial = walkToTheClub(c, env, vec(100, 2960, 0));
    EXPECT_TRUE(trial.arrived);
    // Only the edge of the strip is grazed on the corner.
    EXPECT_LT(trial.ticksOnHarshGround, 10u);
    // Straight on, 100 units of sand at 0.14 and 100 of grass at 0.24 take 1131 ticks; the way round takes less than 1000.
    EXPECT_LT(trial.ticks, 1000u);
}

TEST(GolfSimulationWalking, AGolferCrossesHarshGroundWhenTheWayRoundIsLonger)
{
    // A narrow strip of sand reaching far to both sides is cheaper to cross than to walk round.
    const auto sand = [](double x, double y) { return x > 120 && x < 124 && y > 2000 && y < 4000; };
    Course3 c;
    c.world.hazard = sand;
    c.world.roughness = [sand](double x, double y) { return sand(x, y) ? 0.9 : 0.15; };
    auto env = c.environment();
    env.clubEntrance = vec(160, 2960, 0);
    const auto trial = walkToTheClub(c, env, vec(100, 2960, 0));
    EXPECT_TRUE(trial.arrived);
    EXPECT_GT(trial.ticksOnHarshGround, 0u);
}

TEST(GolfSimulationWalking, AGolferPrefersAPathOverFlatGround)
{
    // A path runs east along y 2880..2920 and the club lies 40 units north of its end: following the path before cutting
    // across the grass is faster than the straight line (1135 ticks against 1267).
    Course3 withPath;
    withPath.world.path = [](double x, double y) { return y > 2880 && y < 2920 && x > 80 && x < 420; };
    auto env = withPath.environment();
    env.clubEntrance = vec(400, 2960, 0);
    const auto onPath = walkToTheClub(withPath, env, vec(100, 2900, 0));

    Course3 plain;
    auto plainEnv = plain.environment();
    plainEnv.clubEntrance = vec(400, 2960, 0);
    const auto onGrass = walkToTheClub(plain, plainEnv, vec(100, 2900, 0));
    EXPECT_TRUE(onPath.arrived);
    EXPECT_TRUE(onGrass.arrived);
    EXPECT_LT(onPath.ticks + 50, onGrass.ticks);
}
