/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "GolfTestWorld.h"

#include <chrono>
#include <gtest/gtest.h>
#include <openrct2/core/Random.hpp>
#include <openrct2/drawing/TerrainSpriteGenerator.h>
#include <openrct2/golf/Animals.h>
#include <openrct2/golf/Carts.h>
#include <openrct2/golf/CourseManager.h>
#include <openrct2/golf/GolfSimulation.h>
#include <openrct2/golf/Traffic.h>
#include <openrct2/world/TerrainShape.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Golf;
using namespace GolfTest;

// TER-002: Performance target: a 256x256 map with an 18-hole course, 300 golfers,
// 60 vehicles and 50 animals runs at >= 30 fps, and simulation tick stays < 10 ms.
TEST(PerformanceBenchmark, Ter002SimulationTickUnderTenMilliseconds)
{
    FunctionWorld world;
    CourseManager courses;
    SeqRandom rng{ 42 };

    // 1. Create an 18-hole course on a 256x256 map (TER-001 / TER-002)
    auto* course = courses.createCourse("Championship 18");
    course->open = true;
    course->greenFee = 2000;

    for (int i = 0; i < 18; i++)
    {
        auto* hole = courses.createHole(course->id);
        hole->tees[0] = HolePoint{ 500 + i * 200, 1000 };
        hole->cup = HolePoint{ 500 + i * 200, 2000 };
    }

    GolfSimulation::Environment env;
    env.world = &world;
    env.courses = &courses;
    env.rng = &rng;
    env.clubEntrance = vec(400, 1000, 0);
    env.exit = vec(350, 950, 0);

    GolfSimulation golfSim(env);

    // 2. Spawn 300 golfers
    for (int i = 0; i < 300; i++)
    {
        GolfSimulation::Spawn sp;
        sp.name = "Golfer " + std::to_string(i + 1);
        sp.groupId = (i / 4) + 1;
        sp.skills.longPlay = 50;
        sp.skills.physique = 50;
        sp.money = 10000;
        sp.courseId = course->id;
        sp.position = vec(400 + (i % 20) * 10, 1000 + (i / 20) * 10, 0);
        golfSim.spawn(sp);
    }
    ASSERT_EQ(golfSim.golfers().size(), 300u);

    // 3. Set up 60 vehicles (40 cars in TrafficSimulation + 20 carts in CartFleet)
    StreetNetwork network;
    for (int x = 1; x <= 60; x++)
        network.placeTile({ x, 10 });
    network.setRoadEntrance({ 1, 10 }, true, { 256, 256 });
    TrafficSimulation trafficSim{ &network };
    auto& lot = trafficSim.addLot({ 30, 10 }, { 30, 12 });
    uint32_t nextSpaceId = 1;
    for (int i = 0; i < 20; i++)
        lot.addTile({ 20 + i, 11 }, 3, nextSpaceId);

    // Spawn 40 cars
    for (int i = 0; i < 40; i++)
        trafficSim.spawn({ static_cast<uint32_t>(100 + i) }, static_cast<uint32_t>(i + 1));

    CartFleet cartFleet;
    for (int i = 0; i < 20; i++)
        cartFleet.addSpace(vec(450 + i * 20, 1050, 0));
    cartFleet.buyCarts(20);
    ASSERT_EQ(cartFleet.cartCount(), 20);

    // 4. Set up 50 animals
    AnimalSpecies deer;
    deer.id = "golf.animal.deer";
    deer.habitat = kHabitatFairway | kHabitatRough | kHabitatWoods;
    deer.minGroup = 2;
    deer.maxGroup = 2;

    AnimalSimulation animalSim(&world, &rng);
    const uint32_t deerIdx = animalSim.addSpecies(deer);

    for (int i = 0; i < 25; i++)
    {
        animalSim.spawnGroup(deerIdx, vec(600 + i * 30, 1200, 0), 32);
    }
    ASSERT_EQ(animalSim.animals().size(), 50u);

    // 5. Warm up 5 ticks
    for (uint32_t t = 1; t <= 5; t++)
    {
        golfSim.tick(t);
        trafficSim.tick();
        cartFleet.rechargeParkedCarts();
        animalSim.tick({});
    }

    // 6. Benchmark 50 simulation ticks
    constexpr int kBenchmarkTicks = 50;
    const auto startTime = std::chrono::high_resolution_clock::now();

    for (uint32_t t = 6; t < 6 + kBenchmarkTicks; t++)
    {
        golfSim.tick(t);
        trafficSim.tick();
        cartFleet.rechargeParkedCarts();
        animalSim.tick({});
    }

    const auto endTime = std::chrono::high_resolution_clock::now();
    const auto elapsedMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();
    const double avgTickMs = elapsedMs / kBenchmarkTicks;

    // TER-002 exit criterion: simulation tick stays < 10 ms
    EXPECT_LT(avgTickMs, 10.0);
}

// ARC-021: Simulation determinism: all simulation state advances identically
// across multiple runs with identical random sequence seeds and initial state.
TEST(SimulationDeterminism, Arc021ReplayMatchesIdenticalSeeds)
{
    auto runSim = [](uint32_t seed) {
        FunctionWorld world;
        CourseManager courses;
        SeqRandom rng{ seed };

        auto* course = courses.createCourse("Deterministic Links");
        course->open = true;
        auto* hole = courses.createHole(course->id);
        hole->tees[0] = HolePoint{ 300, 500 };
        hole->cup = HolePoint{ 300, 1200 };

        GolfSimulation::Environment env;
        env.world = &world;
        env.courses = &courses;
        env.rng = &rng;
        env.clubEntrance = vec(300, 500, 0);
        env.exit = vec(280, 480, 0);

        GolfSimulation sim(env);
        for (int i = 0; i < 10; i++)
        {
            GolfSimulation::Spawn sp;
            sp.name = "Golfer " + std::to_string(i + 1);
            sp.groupId = i + 1;
            sp.skills.longPlay = 50;
            sp.skills.physique = 50;
            sp.money = 10000;
            sp.courseId = course->id;
            sp.position = vec(300, 500, 0);
            sim.spawn(sp);
        }

        for (uint32_t t = 1; t <= 100; t++)
        {
            sim.tick(t);
        }

        std::vector<int64_t> positions;
        for (const auto& g : sim.golfers())
        {
            positions.push_back(g.position.x.raw());
            positions.push_back(g.position.y.raw());
            positions.push_back(g.position.z.raw());
        }
        return positions;
    };

    const auto runA = runSim(12345);
    const auto runB = runSim(12345);
    EXPECT_EQ(runA, runB);
}

// WAT-004: Shoreline clipping of water against partially submerged sloped tiles
TEST(WaterShoreline, Wat004TrianglesClippedCorrectlyAgainstTerrainSlopes)
{
    // A sloped tile rising 16 z units across east and north corners.
    // Water plane at -16 z relative to base height submerges the lower half,
    // leaving the upper half dry.
    const Terrain::TerrainShape slope{ 4, 4, 0, 0 };
    auto waterSprite = Drawing::GenerateTerrainWaterSprite(slope, -16, Drawing::WaterPattern::mask, 0, 0);

    size_t coveredPixels = 0;
    for (uint8_t pixel : waterSprite.pixels)
    {
        if (pixel != 0)
            coveredPixels++;
    }

    // A full tile diamond is approximately 64 * 32 / 2 = 1024 pixels.
    // The half-submerged slope must have water covering roughly half (~512 pixels), clipped cleanly.
    EXPECT_GT(coveredPixels, 200u);
    EXPECT_LT(coveredPixels, 800u);
}
