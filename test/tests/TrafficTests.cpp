/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <map>
#include <openrct2/golf/Traffic.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Golf;

namespace
{
    const TileCoordsXY kMapSize{ 60, 60 };

    // A street along y = 10 from the entrance at x = 1 to x = 30, with a lot of five parking tiles (ten spaces) at x = 15.
    struct Layout
    {
        StreetNetwork network;
        TrafficSimulation sim{ &network };

        Layout(int tilesInLot = 5)
        {
            for (int x = 1; x <= 30; x++)
                network.placeTile({ x, 10 });
            network.setRoadEntrance({ 1, 10 }, true, kMapSize);
            auto& lot = sim.addLot({ 15, 10 }, { 15, 12 });
            uint32_t nextId = 1;
            for (int i = 0; i < tilesInLot; i++)
                lot.addTile({ 14 + i, 11 }, 3, nextId);
        }
    };

    double distanceBetween(const Car& a, const Car& b)
    {
        return std::hypot((a.position.x - b.position.x).toDouble(), (a.position.y - b.position.y).toDouble());
    }

    int stateCount(const TrafficSimulation& sim, CarState state)
    {
        return static_cast<int>(std::count_if(sim.cars().begin(), sim.cars().end(), [&](const Car& c) { return c.state == state; }));
    }
} // namespace

TEST(ParkingLot, EveryTileHoldsTwoSpaces)
{
    ParkingLot lot;
    uint32_t next = 1;
    lot.addTile({ 5, 5 }, 1, next);
    lot.addTile({ 6, 5 }, 1, next);
    EXPECT_EQ(lot.capacity(), 4);
    EXPECT_EQ(lot.freeSpaces(), 4);
    EXPECT_EQ(lot.spaces[0].id, 1u);
    EXPECT_EQ(lot.spaces[3].id, 4u);
    EXPECT_EQ(lot.spaces[0].slot, 0);
    EXPECT_EQ(lot.spaces[1].slot, 1);
    EXPECT_EQ(next, 5u);
}

TEST(ParkingLot, ReservedSpacesCountAsTaken)
{
    ParkingLot lot;
    uint32_t next = 1;
    lot.addTile({ 5, 5 }, 1, next);
    ASSERT_NE(lot.reserve(7), nullptr);
    EXPECT_EQ(lot.freeSpaces(), 1);
    ASSERT_NE(lot.reserve(8), nullptr);
    EXPECT_EQ(lot.freeSpaces(), 0);
    EXPECT_EQ(lot.reserve(9), nullptr);
    ASSERT_NE(lot.spaceOf(7), nullptr);
    EXPECT_EQ(lot.spaceOf(7)->reservedBy, 7u);
    lot.release(7);
    EXPECT_EQ(lot.freeSpaces(), 1);
    EXPECT_EQ(lot.spaceOf(7), nullptr);
    lot.release(404); // unknown cars are ignored
    EXPECT_EQ(lot.freeSpaces(), 1);
}

TEST(ParkingLot, TheTwoSpacesOfATileAreSideBySide)
{
    ParkingLot lot;
    uint32_t next = 1;
    lot.addTile({ 5, 5 }, 3, next); // facing -y: the street is above the tile on the map
    const auto a = parkingSpaceCentre(lot.spaces[0]);
    const auto b = parkingSpaceCentre(lot.spaces[1]);
    EXPECT_NE(a, b);
    EXPECT_EQ(a.y, b.y);
    EXPECT_EQ((b.x - a.x).raw(), Fixed::fromInt(16).raw());
    EXPECT_EQ(((a.x + b.x) / 2).raw(), Fixed::fromInt(5 * 32 + 16).raw()); // centred on the tile
    // Facing +x puts them one above the other.
    ParkingLot east;
    east.addTile({ 5, 5 }, 2, next);
    EXPECT_EQ(parkingSpaceCentre(east.spaces[0]).x, parkingSpaceCentre(east.spaces[1]).x);
    EXPECT_NE(parkingSpaceCentre(east.spaces[0]).y, parkingSpaceCentre(east.spaces[1]).y);
}

TEST(Traffic, ACarSpawnsAtTheEntranceAndDrivesToTheLot)
{
    Layout l;
    const uint32_t id = l.sim.spawn({ 1, 2, 3, 4 }, 9);
    ASSERT_NE(id, 0u);
    const Car* car = l.sim.find(id);
    ASSERT_NE(car, nullptr);
    EXPECT_EQ(car->route.front(), (TileCoordsXY{ 1, 10 }));
    EXPECT_EQ(car->route.back(), (TileCoordsXY{ 14, 10 })); // the street tile in front of its own space
    EXPECT_EQ(car->golferIds.size(), 4u);
    EXPECT_EQ(car->groupId, 9u);
    EXPECT_EQ(car->state, CarState::driving);
    EXPECT_EQ(l.sim.freeSpaces(), 9); // the space is reserved already

    int ticks = 0;
    while (l.sim.find(id)->state != CarState::parked && ticks < 5000)
    {
        l.sim.tick();
        ticks++;
    }
    EXPECT_EQ(l.sim.find(id)->state, CarState::parked);
    EXPECT_LT(ticks, 1500);
    const auto events = l.sim.takeEvents();
    bool parked = false;
    for (const auto& e : events)
        parked |= e.type == TrafficEvent::Type::parked && e.carId == id;
    EXPECT_TRUE(parked);
    // It stands in a space of the lot now.
    EXPECT_EQ(l.sim.lots()[0].spaceOf(id)->carId, id);
}

TEST(Traffic, ExactlyTenCarsParkInALotOfTenSpacesAndTheEleventhNeverSpawns)
{
    // Acceptance criterion of section 12: with a parking lot of 10 spaces, exactly 10 cars park, the 11th never spawns
    // until one leaves.
    Layout l;
    ASSERT_EQ(l.sim.capacity(), 10);
    std::vector<uint32_t> ids;
    int refused = 0;
    for (int attempt = 0; attempt < 11; attempt++)
    {
        const uint32_t id = l.sim.spawn({ static_cast<uint32_t>(attempt + 1) });
        if (id != 0)
            ids.push_back(id);
        else
            refused++;
        for (int t = 0; t < 70; t++)
            l.sim.tick();
    }
    EXPECT_EQ(ids.size(), 10u);
    EXPECT_EQ(refused, 1);
    EXPECT_EQ(l.sim.freeSpaces(), 0);
    // Keep trying while they drive in: still refused.
    for (int t = 0; t < 4000; t++)
    {
        l.sim.tick();
        if (t % 200 == 0)
        {
            EXPECT_EQ(l.sim.spawn({ 99 }), 0u);
        }
    }
    EXPECT_EQ(stateCount(l.sim, CarState::parked), 10);
    EXPECT_EQ(l.sim.carCount(), 10u);

    // One car leaves: it drives out, frees its space, and a new car can come.
    ASSERT_TRUE(l.sim.depart(ids[3]));
    EXPECT_EQ(l.sim.spawn({ 100 }), 0u); // still in its space until it pulls out
    for (int t = 0; t < 3000; t++)
        l.sim.tick();
    EXPECT_EQ(l.sim.find(ids[3]), nullptr); // gone off the map
    EXPECT_EQ(l.sim.freeSpaces(), 1);
    EXPECT_NE(l.sim.spawn({ 100 }), 0u);
    EXPECT_EQ(l.sim.freeSpaces(), 0);
}

TEST(Traffic, CarsKeepTheirDistanceOnTheRoad)
{
    Layout l;
    for (int i = 0; i < 6; i++)
    {
        EXPECT_NE(l.sim.spawn({ static_cast<uint32_t>(i + 1) }), 0u);
        for (int t = 0; t < 25; t++) // close together: the later cars have to hold back
            l.sim.tick();
    }
    double closest = 1e9;
    for (int t = 0; t < 3000; t++)
    {
        l.sim.tick();
        const auto& cars = l.sim.cars();
        for (size_t a = 0; a < cars.size(); a++)
            for (size_t b = a + 1; b < cars.size(); b++)
                if (cars[a].state == CarState::driving && cars[b].state == CarState::driving)
                    closest = std::min(closest, distanceBetween(cars[a], cars[b]));
    }
    EXPECT_GT(closest, kCarLength * 0.8);
    EXPECT_EQ(stateCount(l.sim, CarState::parked), 6);
}

TEST(Traffic, CarsStayOnTheirSideOfTheRoad)
{
    Layout l;
    const uint32_t id = l.sim.spawn({ 1 });
    for (int t = 0; t < 40; t++)
        l.sim.tick();
    const Car* car = l.sim.find(id);
    ASSERT_NE(car, nullptr);
    // Driving east along y = 10 tiles: the lane is to the right, the +y side on the map for this heading... one lane
    // offset from the centre line.
    const double centre = 10 * 32 + 16;
    EXPECT_NEAR(std::fabs(car->position.y.toDouble() - centre), kLaneOffset, 0.01);
    EXPECT_EQ(car->heading, 2);
}

TEST(Traffic, GroupsOfOneToFourOnly)
{
    Layout l;
    EXPECT_EQ(l.sim.spawn({}), 0u);
    EXPECT_EQ(l.sim.spawn({ 1, 2, 3, 4, 5 }), 0u);
    EXPECT_NE(l.sim.spawn({ 1 }), 0u);
}

TEST(Traffic, NoEntranceMeansNoCars)
{
    StreetNetwork network;
    for (int x = 1; x <= 20; x++)
        network.placeTile({ x, 10 });
    TrafficSimulation sim(&network);
    auto& lot = sim.addLot({ 10, 10 }, { 10, 12 });
    uint32_t next = 1;
    lot.addTile({ 10, 11 }, 3, next);
    EXPECT_EQ(sim.spawn({ 1 }), 0u);
}

TEST(Traffic, NoWayToTheLotMeansNoCars)
{
    StreetNetwork network;
    for (int x = 1; x <= 8; x++)
        network.placeTile({ x, 10 });
    for (int x = 12; x <= 20; x++)
        network.placeTile({ x, 10 }); // a gap between the entrance and the lot
    network.setRoadEntrance({ 1, 10 }, true, kMapSize);
    TrafficSimulation sim(&network);
    auto& lot = sim.addLot({ 15, 10 }, { 15, 12 });
    uint32_t next = 1;
    lot.addTile({ 15, 11 }, 3, next);
    EXPECT_EQ(sim.spawn({ 1 }), 0u);
    EXPECT_EQ(sim.freeSpaces(), 2);
}

TEST(Traffic, OneWayStreetsAreRespected)
{
    // Entrance at x = 1 on a one-way eastward street to x = 12, then a loop back round to the lot's street.
    StreetNetwork network;
    for (int x = 1; x <= 12; x++)
        network.placeTile({ x, 10 }, x >= 3 && x <= 11, 2);
    for (int y = 11; y <= 14; y++)
        network.placeTile({ 12, y });
    for (int x = 3; x <= 11; x++)
        network.placeTile({ x, 14 });
    network.placeTile({ 2, 14 });
    network.placeTile({ 2, 13 });
    network.placeTile({ 2, 12 });
    network.placeTile({ 2, 11 });
    network.placeTile({ 2, 10 });
    network.setRoadEntrance({ 1, 10 }, true, kMapSize);
    TrafficSimulation sim(&network);
    auto& lot = sim.addLot({ 7, 14 }, { 7, 16 });
    uint32_t next = 1;
    lot.addTile({ 7, 15 }, 3, next);
    const uint32_t id = sim.spawn({ 1 });
    ASSERT_NE(id, 0u);
    std::vector<TileCoordsXY> visited;
    for (int t = 0; t < 4000 && sim.find(id) != nullptr && sim.find(id)->state != CarState::parked; t++)
    {
        sim.tick();
        const auto* car = sim.find(id);
        if (car != nullptr && !car->route.empty() && (visited.empty() || !(visited.back() == car->route[car->index])))
            visited.push_back(car->route[car->index]);
    }
    ASSERT_EQ(sim.find(id)->state, CarState::parked);
    ASSERT_GE(visited.size(), 2u);
    for (size_t i = 1; i < visited.size(); i++)
    {
        int dir = -1;
        for (int d = 0; d < 4; d++)
        {
            const auto s = directionStep(d);
            if (visited[i - 1].x + s.x == visited[i].x && visited[i - 1].y + s.y == visited[i].y)
                dir = d;
        }
        ASSERT_GE(dir, 0);
        EXPECT_TRUE(network.canDrive(visited[i - 1], dir)) << "drove against the flow at step " << i;
    }
}

TEST(Traffic, JunctionsAreSharedOneCarAtATime)
{
    // A crossing at (10, 10): one car comes from the west, another from the north; both want to go east of it.
    StreetNetwork network;
    for (int x = 1; x <= 20; x++)
        network.placeTile({ x, 10 });
    for (int y = 1; y <= 9; y++)
        network.placeTile({ 10, y });
    network.setRoadEntrance({ 1, 10 }, true, kMapSize);
    network.setRoadEntrance({ 10, 1 }, true, kMapSize);
    TrafficSimulation sim(&network);
    auto& lot = sim.addLot({ 18, 10 }, { 18, 12 });
    uint32_t next = 1;
    for (int i = 0; i < 4; i++)
        lot.addTile({ 17 + i, 11 }, 3, next);
    const uint32_t a = sim.spawn({ 1 });
    const uint32_t b = sim.spawn({ 2 });
    ASSERT_NE(a, 0u);
    ASSERT_NE(b, 0u);
    double closestInJunction = 1e9;
    int bothNear = 0;
    for (int t = 0; t < 3000; t++)
    {
        sim.tick();
        const Car* ca = sim.find(a);
        const Car* cb = sim.find(b);
        if (ca == nullptr || cb == nullptr)
            continue;
        const double cx = 10 * 32 + 16, cy = 10 * 32 + 16;
        auto near = [&](const Car* c) {
            return std::fabs(c->position.x.toDouble() - cx) < 14 && std::fabs(c->position.y.toDouble() - cy) < 14;
        };
        if (near(ca) && near(cb))
        {
            bothNear++;
            closestInJunction = std::min(closestInJunction, distanceBetween(*ca, *cb));
        }
    }
    EXPECT_EQ(bothNear, 0) << "two cars were inside the junction together";
    EXPECT_EQ(stateCount(sim, CarState::parked), 2);
}

TEST(Traffic, CarsAreObstaclesForBalls)
{
    Layout l;
    const uint32_t id = l.sim.spawn({ 1 });
    for (int t = 0; t < 100; t++)
        l.sim.tick();
    const Car* car = l.sim.find(id);
    ASSERT_NE(car, nullptr);
    const auto o = l.sim.obstacleOf(*car);
    EXPECT_EQ(o.shape, ObstacleShape::box);
    EXPECT_EQ(o.centre, car->position);
    EXPECT_GT(o.height, Fixed{});
    EXPECT_NE(o.id, 0u);
    ObstacleGrid grid;
    EXPECT_TRUE(grid.add(o));
    ObstacleContact c;
    EXPECT_TRUE(grid.query(car->position + FixedVec3{ Fixed{}, Fixed{}, Fixed::fromInt(5) }, Fixed::ratio(1, 2), c));
}

TEST(Traffic, DepartingCarsPullOutAndDriveToTheNearestEntrance)
{
    Layout l;
    // A second entrance at the far end: the lot is nearer the first.
    l.network.setRoadEntrance({ 30, 10 }, true, kMapSize);
    const uint32_t id = l.sim.spawn({ 1, 2 });
    for (int t = 0; t < 2000 && l.sim.find(id)->state != CarState::parked; t++)
        l.sim.tick();
    ASSERT_EQ(l.sim.find(id)->state, CarState::parked);
    EXPECT_FALSE(l.sim.depart(999));
    ASSERT_TRUE(l.sim.depart(id));
    EXPECT_FALSE(l.sim.depart(id)); // no longer parked
    EXPECT_EQ(l.sim.find(id)->state, CarState::departing);
    for (int t = 0; t < 200 && l.sim.find(id)->state == CarState::departing; t++)
        l.sim.tick();
    ASSERT_NE(l.sim.find(id), nullptr);
    EXPECT_EQ(l.sim.find(id)->state, CarState::leaving);
    EXPECT_EQ(l.sim.find(id)->route.back(), (TileCoordsXY{ 1, 10 })); // 14 tiles to the west against 15 to the east
    bool departed = false, despawned = false;
    for (int t = 0; t < 3000 && l.sim.find(id) != nullptr; t++)
        l.sim.tick();
    for (const auto& e : l.sim.takeEvents())
    {
        departed |= e.type == TrafficEvent::Type::departed;
        despawned |= e.type == TrafficEvent::Type::despawned;
    }
    EXPECT_TRUE(departed);
    EXPECT_TRUE(despawned);
    EXPECT_EQ(l.sim.carCount(), 0u);
}

TEST(Traffic, CarContactPointsMatchModelConstants)
{
    const auto cp = carContactPoints();
    EXPECT_FLOAT_EQ(12.0f, cp.halfLength);
    EXPECT_FLOAT_EQ(6.0f, cp.halfWidth);
}

TEST(Traffic, ACarTurnsIntoItsSpaceAtARightAngleInsteadOfCuttingAcrossTheLot)
{
    Layout l;
    // Fill the first spaces so the next car must go further along the lot.
    for (int i = 0; i < 6; i++)
        l.sim.spawn({ static_cast<uint32_t>(i + 1) });
    const uint32_t id = l.sim.spawn({ 99 });
    ASSERT_NE(id, 0u);
    const auto* space = l.sim.lots()[0].spaceOf(id);
    ASSERT_NE(space, nullptr);
    // It drives to the street tile in front of that space.
    EXPECT_EQ(l.sim.find(id)->route.back(), parkingSpaceStreetTile(*space));
    const auto target = parkingSpaceCentre(*space);
    int ticks = 0;
    while (l.sim.find(id)->state != CarState::parking && ticks++ < 8000)
        l.sim.tick();
    ASSERT_EQ(l.sim.find(id)->state, CarState::parking);
    // While pulling in the car is either level with the space along the street or level with it across the street: it
    // never moves diagonally.
    for (int t = 0; t < kParkingTicks; t++)
    {
        l.sim.tick();
        const auto& c = l.sim.find(id)->position;
        const bool onLegOne = c.y == l.sim.find(id)->parkFrom.y; // the street runs along x
        const bool onLegTwo = c.x == target.x;
        EXPECT_TRUE(onLegOne || onLegTwo) << "diagonal move at tick " << t;
    }
}
