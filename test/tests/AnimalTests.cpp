/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *
 *****************************************************************************/

#include "GolfTestWorld.h"

#include <cmath>
#include <gtest/gtest.h>
#include <openrct2/golf/Animals.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Golf;
using namespace GolfTest;

namespace
{
    SurfaceSample ground(double roughness, bool water = false, bool path = false)
    {
        SurfaceSample s;
        s.roughness = Fixed::fromRaw(static_cast<int64_t>(roughness * 65536));
        s.water = water;
        s.path = path;
        return s;
    }

    AnimalSpecies deer()
    {
        AnimalSpecies s;
        s.id = "golf.animal.deer";
        s.habitat = kHabitatWoods | kHabitatRough;
        s.minGroup = 2;
        s.maxGroup = 4;
        return s;
    }

    AnimalSpecies duck()
    {
        AnimalSpecies s;
        s.id = "golf.animal.duck";
        s.habitat = kHabitatWater | kHabitatGreenBank;
        s.canSwim = true;
        s.minGroup = 2;
        s.maxGroup = 3;
        return s;
    }

    AnimalSpecies squirrel()
    {
        AnimalSpecies s;
        s.id = "golf.animal.squirrel";
        s.habitat = kHabitatWoods;
        s.thief = true;
        s.minGroup = 1;
        s.maxGroup = 1;
        return s;
    }

    // Woods in the north half (y < 3000), fairway in the south, a pond in the middle.
    FunctionWorld forestWorld()
    {
        FunctionWorld w;
        w.roughness = [](double, double y) { return y < 3000 ? 0.80 : 0.15; };
        w.water = [](double x, double y) { return std::hypot(x - 5000, y - 5000) < 200; };
        return w;
    }
} // namespace

TEST(AnimalHabitat, DucksLiveOnWaterAndItsBanks)
{
    EXPECT_TRUE(suitsHabitat(duck(), ground(0.15, true), false));
    EXPECT_TRUE(suitsHabitat(duck(), ground(0.15), true));
    EXPECT_FALSE(suitsHabitat(duck(), ground(0.15), false));
    EXPECT_FALSE(suitsHabitat(duck(), ground(0.8), false));
}

TEST(AnimalHabitat, DeerLiveInTheWoodsAndTheRough)
{
    EXPECT_TRUE(suitsHabitat(deer(), ground(0.80), false)); // woods
    EXPECT_TRUE(suitsHabitat(deer(), ground(0.45), false)); // rough
    EXPECT_FALSE(suitsHabitat(deer(), ground(0.15), false)); // fairway
    EXPECT_FALSE(suitsHabitat(deer(), ground(0.80, true), false)); // water
    EXPECT_FALSE(suitsHabitat(squirrel(), ground(0.45), false)); // squirrels need real woods
    EXPECT_TRUE(suitsHabitat(squirrel(), ground(0.80), false));
}

TEST(AnimalHabitat, StreetsAndTheEdgeOfTheMapAreNeverHabitat)
{
    EXPECT_FALSE(suitsHabitat(deer(), ground(0.80, false, true), false));
    SurfaceSample outside = ground(0.80);
    outside.outOfBounds = true;
    EXPECT_FALSE(suitsHabitat(deer(), outside, false));
    AnimalSpecies fairwayWalker;
    fairwayWalker.habitat = kHabitatFairway;
    EXPECT_TRUE(suitsHabitat(fairwayWalker, ground(0.15), false));
    SurfaceSample green = ground(0.05);
    green.puttable = true;
    EXPECT_FALSE(suitsHabitat(fairwayWalker, green, false)); // not on the greens
}

TEST(AnimalSimulation, GroupsSpawnInSuitableHabitat)
{
    auto world = forestWorld();
    SeqRandom rng(5);
    AnimalSimulation sim(&world, &rng);
    const auto d = sim.addSpecies(deer());
    const int32_t spawned = sim.spawnGroup(d, vec(2000, 2000, 0));
    EXPECT_GE(spawned, 2);
    EXPECT_LE(spawned, 4);
    ASSERT_EQ(sim.animals().size(), static_cast<size_t>(spawned));
    const uint32_t group = sim.animals()[0].groupId;
    for (const auto& a : sim.animals())
    {
        EXPECT_LT(a.position.y, Fixed::fromInt(3000)); // in the woods
        EXPECT_EQ(a.groupId, group);
        EXPECT_NEAR(a.position.x.toDouble(), sim.animals()[0].position.x.toDouble(), 40);
    }
    EXPECT_EQ(sim.takeEvents().size(), static_cast<size_t>(spawned));
}

TEST(AnimalSimulation, NoHabitatNoSpawn)
{
    auto world = forestWorld();
    SeqRandom rng(5);
    AnimalSimulation sim(&world, &rng);
    const auto d = sim.addSpecies(deer());
    EXPECT_EQ(sim.spawnGroup(d, vec(2000, 8000, 0)), 0); // all fairway there
    EXPECT_EQ(sim.spawnGroup(77, vec(2000, 2000, 0)), 0); // unknown species
    EXPECT_TRUE(sim.animals().empty());
}

TEST(AnimalSimulation, DucksSpawnInThePond)
{
    auto world = forestWorld();
    SeqRandom rng(11);
    AnimalSimulation sim(&world, &rng);
    const auto d = sim.addSpecies(duck());
    EXPECT_GE(sim.spawnGroup(d, vec(5000, 5000, 0), 100), 2);
    for (const auto& a : sim.animals())
        EXPECT_LT(std::hypot(a.position.x.toDouble() - 5000, a.position.y.toDouble() - 5000), 230);
}

TEST(AnimalSimulation, SpawnRatesDecideHowOftenAGroupAppears)
{
    auto world = forestWorld();
    SeqRandom rng(3);
    AnimalSimulation sim(&world, &rng);
    auto species = deer();
    species.spawnPerThousandPerDay = 1000;
    sim.addSpecies(species);
    auto never = deer();
    never.spawnPerThousandPerDay = 0;
    never.id = "never";
    sim.addSpecies(never);
    EXPECT_GT(sim.dailySpawns({ vec(2000, 2000, 0) }), 0);
    for (const auto& a : sim.animals())
        EXPECT_EQ(a.species, 0u); // only the always-spawning species
    EXPECT_EQ(sim.dailySpawns({}), 0);
}

TEST(AnimalSimulation, AnimalsWanderButStayInTheirHabitat)
{
    auto world = forestWorld();
    SeqRandom rng(8);
    AnimalSimulation sim(&world, &rng);
    const auto d = sim.addSpecies(deer());
    sim.spawnGroup(d, vec(2000, 2000, 0));
    ASSERT_FALSE(sim.animals().empty());
    const auto start = sim.animals()[0].position;
    double moved = 0;
    for (int t = 0; t < 6000; t++)
    {
        sim.tick({});
        for (const auto& a : sim.animals())
            EXPECT_LE(a.position.y, Fixed::fromInt(3000) + Fixed::fromInt(4)) << t;
        moved = std::max(moved, std::hypot(sim.animals()[0].position.x.toDouble() - start.x.toDouble(),
                                           sim.animals()[0].position.y.toDouble() - start.y.toDouble()));
    }
    EXPECT_GT(moved, 20.0);
}

TEST(AnimalSimulation, GolfersAndCartsMakeThemFlee)
{
    auto world = forestWorld();
    SeqRandom rng(2);
    AnimalSimulation sim(&world, &rng);
    const auto d = sim.addSpecies(deer());
    sim.spawnGroup(d, vec(2000, 2000, 0));
    sim.takeEvents();
    const auto before = sim.animals()[0].position;
    // A golfer walks up from the south.
    const Threat golfer{ before + vec(0, 100, 0), false };
    sim.tick({ golfer });
    int fled = 0;
    for (const auto& e : sim.takeEvents())
        fled += e.type == AnimalEvent::Type::fled;
    EXPECT_EQ(static_cast<size_t>(fled), sim.animals().size());
    for (const auto& a : sim.animals())
        EXPECT_EQ(a.state, AnimalState::flee);
    for (int t = 0; t < 300; t++)
        sim.tick({ golfer });
    // They ran away from the threat (north, since it came from the south).
    EXPECT_LT(sim.animals()[0].position.y, before.y);
}

TEST(AnimalSimulation, FarAwayThreatsAreIgnored)
{
    auto world = forestWorld();
    SeqRandom rng(2);
    AnimalSimulation sim(&world, &rng);
    const auto d = sim.addSpecies(deer());
    sim.spawnGroup(d, vec(2000, 2000, 0));
    sim.takeEvents();
    sim.tick({ Threat{ sim.animals()[0].position + vec(1500, 0, 0), true } });
    for (const auto& e : sim.takeEvents())
        EXPECT_NE(e.type, AnimalEvent::Type::fled);
}

TEST(AnimalSimulation, BirdsFlyOffRatherThanRun)
{
    auto world = forestWorld();
    SeqRandom rng(2);
    AnimalSimulation sim(&world, &rng);
    auto bird = squirrel();
    bird.canFly = true;
    bird.thief = false;
    const auto b = sim.addSpecies(bird);
    sim.spawnGroup(b, vec(2000, 2000, 0));
    sim.tick({ Threat{ sim.animals()[0].position + vec(40, 0, 0), false } });
    EXPECT_EQ(sim.animals()[0].state, AnimalState::fly);
    for (int t = 0; t < 10; t++)
        sim.tick({ Threat{ sim.animals()[0].position + vec(40, 0, 0), false } });
    EXPECT_GT(sim.animals()[0].position.z, Fixed::fromInt(10)); // up in the air
}

TEST(AnimalSimulation, AnimalsAvoidStreets)
{
    FunctionWorld world = forestWorld();
    // A street across the woods at x = 2400.
    class StreetWorld final : public IBallWorld
    {
    public:
        explicit StreetWorld(FunctionWorld& inner)
            : _inner(inner)
        {
        }
        SurfaceSample surfaceAt(Fixed x, Fixed y) const override
        {
            auto s = _inner.surfaceAt(x, y);
            s.path = x.toDouble() > 2400 && x.toDouble() < 2440;
            return s;
        }

    private:
        FunctionWorld& _inner;
    } streets(world);
    SeqRandom rng(6);
    AnimalSimulation sim(&streets, &rng);
    auto species = deer();
    species.wanderRadiusUnits = 600;
    const auto d = sim.addSpecies(species);
    sim.spawnGroup(d, vec(2200, 2000, 0), 50);
    ASSERT_FALSE(sim.animals().empty());
    for (int t = 0; t < 20000; t++)
    {
        sim.tick({});
        for (const auto& a : sim.animals())
            ASSERT_FALSE(a.position.x > Fixed::fromInt(2400) && a.position.x < Fixed::fromInt(2440)) << t;
    }
}

TEST(AnimalSimulation, AnimalsAreSmallCylindersForBallsAndFleeWhenHit)
{
    auto world = forestWorld();
    SeqRandom rng(2);
    AnimalSimulation sim(&world, &rng);
    const auto d = sim.addSpecies(deer());
    sim.spawnGroup(d, vec(2000, 2000, 0));
    sim.takeEvents();
    const auto& animal = sim.animals()[0];
    const auto o = sim.obstacleOf(animal);
    EXPECT_EQ(o.shape, ObstacleShape::cylinder);
    EXPECT_EQ(o.halfX, Fixed::fromInt(3));
    EXPECT_EQ(o.centre, animal.position);
    EXPECT_NE(o.id, 0u);
    const uint32_t id = animal.id;
    sim.hitByBall(id);
    sim.tick({});
    bool hitEvent = false, fled = false;
    for (const auto& e : sim.takeEvents())
    {
        hitEvent |= e.type == AnimalEvent::Type::hitByBall && e.animalId == id;
        fled |= e.type == AnimalEvent::Type::fled && e.animalId == id;
    }
    EXPECT_TRUE(hitEvent);
    EXPECT_TRUE(fled);
    sim.hitByBall(9999); // unknown animals are ignored
}

TEST(AnimalSimulation, ThievesSometimesTakeABallOnTheRough)
{
    auto world = forestWorld();
    SeqRandom rng(12);
    AnimalSimulation sim(&world, &rng);
    const auto s = sim.addSpecies(squirrel());
    sim.spawnGroup(s, vec(2000, 2000, 0));
    const auto pos = sim.animals()[0].position;
    int steals = 0;
    constexpr int kMinutes = 200000;
    for (int i = 0; i < kMinutes; i++)
        steals += sim.tryStealBall(pos + vec(10, 0, 0), 7) ? 1 : 0;
    // "Rare": about 0.15% of the minutes.
    EXPECT_GT(steals, kMinutes * kStealChancePerMillionPerMinute / 1000000 / 2);
    EXPECT_LT(steals, kMinutes * kStealChancePerMillionPerMinute / 1000000 * 2);
    bool event = false;
    for (const auto& e : sim.takeEvents())
        event |= e.type == AnimalEvent::Type::stoleBall && e.value == 7;
    EXPECT_TRUE(event);
}

TEST(AnimalSimulation, NobodyStealsFromTheFairwayTheGreenOrWhenTooFar)
{
    auto world = forestWorld();
    world.roughness = [](double, double y) { return y < 3000 ? 0.80 : 0.15; };
    SeqRandom rng(12);
    AnimalSimulation sim(&world, &rng);
    const auto s = sim.addSpecies(squirrel());
    sim.spawnGroup(s, vec(2000, 2000, 0));
    // Make the thief stand right at the boundary, then check a ball on the fairway far from it and one at a distance.
    for (int i = 0; i < 100000; i++)
    {
        EXPECT_FALSE(sim.tryStealBall(vec(2000, 6000, 0), 1)); // fairway, far away
        EXPECT_FALSE(sim.tryStealBall(sim.animals()[0].position + vec(1000, 0, 0), 1)); // too far
    }
    // Non-thieves never steal.
    AnimalSimulation deerOnly(&world, &rng);
    const auto d = deerOnly.addSpecies(deer());
    deerOnly.spawnGroup(d, vec(2000, 2000, 0));
    for (int i = 0; i < 100000; i++)
        EXPECT_FALSE(deerOnly.tryStealBall(deerOnly.animals()[0].position, 1));
}

TEST(AnimalSimulation, GolfersSeeAnimalsWithinFiveTiles)
{
    auto world = forestWorld();
    SeqRandom rng(2);
    AnimalSimulation sim(&world, &rng);
    const auto d = sim.addSpecies(deer());
    sim.spawnGroup(d, vec(2000, 2000, 0));
    const auto p = sim.animals()[0].position;
    EXPECT_GE(sim.animalsInSight(p + vec(100, 0, 0)), 1);
    EXPECT_EQ(sim.animalsInSight(p + vec(5 * 32 + 200, 0, 0)), 0);
}

TEST(AnimalSightings, AreCappedEachHole)
{
    AnimalSightings s;
    EXPECT_TRUE(s.take());
    EXPECT_TRUE(s.take());
    EXPECT_FALSE(s.take());
    EXPECT_EQ(s.thisHole, kAnimalHappinessPerHoleCap);
    s.newHole();
    EXPECT_TRUE(s.take());
}
