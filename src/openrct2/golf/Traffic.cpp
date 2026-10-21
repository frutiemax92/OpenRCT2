/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Traffic.h"
#include "../object/VehicleModelObject.h"

#include <algorithm>

namespace OpenRCT2::Golf
{
    namespace
    {
        const Fixed kTile = Fixed::fromInt(32);
        const Fixed kHalfTile = Fixed::fromInt(16);

        int64_t tileKey(const TileCoordsXY& p)
        {
            return (static_cast<int64_t>(p.x) << 32) ^ static_cast<int64_t>(static_cast<uint32_t>(p.y));
        }

        int32_t directionBetween(const TileCoordsXY& from, const TileCoordsXY& to)
        {
            for (int32_t d = 0; d < kStreetDirections; d++)
            {
                const auto step = directionStep(d);
                if (from.x + step.x == to.x && from.y + step.y == to.y)
                    return d;
            }
            return 0;
        }
    } // namespace

    int32_t ParkingLot::freeSpaces() const
    {
        return static_cast<int32_t>(std::count_if(spaces.begin(), spaces.end(), [](const ParkingSpace& s) {
            return s.carId == 0 && s.reservedBy == 0;
        }));
    }

    ParkingSpace* ParkingLot::reserve(uint32_t carId)
    {
        for (auto& space : spaces)
            if (space.carId == 0 && space.reservedBy == 0)
            {
                space.reservedBy = carId;
                return &space;
            }
        return nullptr;
    }

    void ParkingLot::release(uint32_t carId)
    {
        for (auto& space : spaces)
        {
            if (space.carId == carId)
                space.carId = 0;
            if (space.reservedBy == carId)
                space.reservedBy = 0;
        }
    }

    ParkingSpace* ParkingLot::spaceOf(uint32_t carId)
    {
        for (auto& space : spaces)
            if (space.carId == carId || space.reservedBy == carId)
                return &space;
        return nullptr;
    }

    void ParkingLot::addTile(const TileCoordsXY& tile, uint8_t facing, uint32_t& nextSpaceId)
    {
        for (uint8_t slot = 0; slot < 2; slot++)
        {
            ParkingSpace s;
            s.id = nextSpaceId++;
            s.tile = tile;
            s.slot = slot;
            s.facing = facing;
            spaces.push_back(s);
        }
    }

    FixedVec3 parkingSpaceCentre(const ParkingSpace& space)
    {
        const Fixed cx = Fixed::fromInt(space.tile.x * 32) + kHalfTile;
        const Fixed cy = Fixed::fromInt(space.tile.y * 32) + kHalfTile;
        // The two slots sit a quarter of a tile either side of the centre, across the line from the street.
        const auto step = directionStep(space.facing);
        const Fixed across = space.slot == 0 ? Fixed::fromInt(-8) : Fixed::fromInt(8);
        // The perpendicular of the facing direction.
        return { cx + Fixed::fromInt(-step.y) * across, cy + Fixed::fromInt(step.x) * across, Fixed{} };
    }

    Model::ContactPoints carContactPoints()
    {
        auto* carObj = VehicleModelObject::FindFirstByKind(VehicleModelKind::car);
        if (carObj != nullptr)
        {
            return carObj->ContactPoints;
        }
        return { 12.0f, 6.0f };
    }

    ParkingLot& TrafficSimulation::addLot(const TileCoordsXY& streetTile, const TileCoordsXY& pathTile)
    {
        ParkingLot lot;
        lot.id = _nextLotId++;
        lot.streetTile = streetTile;
        lot.pathTile = pathTile;
        _lots.push_back(lot);
        return _lots.back();
    }

    void TrafficSimulation::reset()
    {
        _lots.clear();
        _cars.clear();
        _events.clear();
        _junctionOwner.clear();
    }

    const ParkingSpace* TrafficSimulation::findSpace(uint32_t spaceId) const
    {
        for (const auto& lot : _lots)
            for (const auto& space : lot.spaces)
                if (space.id == spaceId)
                    return &space;
        return nullptr;
    }

    int32_t TrafficSimulation::freeSpaces() const
    {
        int32_t total = 0;
        for (const auto& lot : _lots)
            total += lot.freeSpaces();
        return total;
    }

    int32_t TrafficSimulation::capacity() const
    {
        int32_t total = 0;
        for (const auto& lot : _lots)
            total += lot.capacity();
        return total;
    }

    Car* TrafficSimulation::find(uint32_t carId)
    {
        for (auto& car : _cars)
            if (car.id == carId)
                return &car;
        return nullptr;
    }

    std::vector<TrafficEvent> TrafficSimulation::takeEvents()
    {
        std::vector<TrafficEvent> events;
        events.swap(_events);
        return events;
    }

    Fixed TrafficSimulation::unitsPerTick(int32_t kph)
    {
        // 1 km/h = 0.2778 m/s = 1.944 units/s = 0.0486 units/tick.
        return Fixed::ratio(kph * 486, 10000);
    }

    uint32_t TrafficSimulation::spawn(const std::vector<uint32_t>& golferIds, uint32_t groupId)
    {
        int32_t maxSeats = 4;
        auto* carObj = VehicleModelObject::FindFirstByKind(VehicleModelKind::car);
        if (carObj != nullptr)
        {
            maxSeats = carObj->Seats;
        }
        if (golferIds.empty() || static_cast<int32_t>(golferIds.size()) > maxSeats)
            return 0;
        const auto entrances = _network->roadEntrances();
        if (entrances.empty())
            return 0;

        // The nearest lot with a free space that a car can reach from an entrance.
        const uint32_t carId = _nextCarId;
        ParkingLot* bestLot = nullptr;
        std::vector<TileCoordsXY> bestRoute;
        for (const auto& entrance : entrances)
        {
            for (auto& lot : _lots)
            {
                if (lot.freeSpaces() == 0)
                    continue;
                auto route = _network->findRoute(entrance, lot.streetTile);
                if (route.empty())
                    continue;
                if (bestLot == nullptr || route.size() < bestRoute.size())
                {
                    bestLot = &lot;
                    bestRoute = std::move(route);
                }
            }
        }
        if (bestLot == nullptr)
            return 0;
        // STE-011: the space is taken from the moment the car is on its way.
        auto* space = bestLot->reserve(carId);
        if (space == nullptr)
            return 0;

        Car car;
        car.id = _nextCarId++;
        car.groupId = groupId;
        car.golferIds = golferIds;
        car.route = std::move(bestRoute);
        car.lotId = bestLot->id;
        car.spaceId = space->id;
        car.state = CarState::driving;
        updatePosition(car);
        _cars.push_back(std::move(car));
        _events.push_back({ TrafficEvent::Type::spawned, _cars.back().id, 0 });
        return _cars.back().id;
    }

    bool TrafficSimulation::depart(uint32_t carId)
    {
        auto* car = find(carId);
        if (car == nullptr || car->state != CarState::parked)
            return false;
        car->state = CarState::departing;
        car->stateTicks = 0;
        return true;
    }

    Obstacle TrafficSimulation::obstacleOf(const Car& car) const
    {
        auto* carObj = VehicleModelObject::FindFirstByKind(VehicleModelKind::car);
        if (carObj != nullptr && carObj->Collision.shape != CollisionShape::none)
        {
            auto o = makeObstacle(0x40000000u + car.id, carObj->Collision, car.position, car.heading);
            o.id = 0x40000000u + car.id;
            return o;
        }
        Obstacle o;
        o.id = 0x40000000u + car.id;
        o.shape = ObstacleShape::box;
        o.centre = car.position;
        o.halfX = Fixed::fromInt(kCarLength / 2);
        o.halfY = Fixed::fromInt(5);
        o.height = Fixed::fromInt(10);
        o.restitution = Fixed::ratio(3, 10);
        o.friction = Fixed::ratio(3, 10);
        return o;
    }

    void TrafficSimulation::updatePosition(Car& car)
    {
        if (car.route.empty())
            return;
        const size_t i = std::min(car.index, car.route.size() - 1);
        const auto& a = car.route[i];
        const auto b = i + 1 < car.route.size() ? car.route[i + 1] : a;
        const int32_t dir = i + 1 < car.route.size() ? directionBetween(a, b) : car.heading;
        car.heading = dir;
        const auto step = directionStep(dir);
        const Fixed ax = Fixed::fromInt(a.x * 32) + kHalfTile;
        const Fixed ay = Fixed::fromInt(a.y * 32) + kHalfTile;
        // The lane is to the right of the direction of travel: the perpendicular rotated clockwise on the map.
        const Fixed rx = Fixed::fromInt(-step.y * kLaneOffset);
        const Fixed ry = Fixed::fromInt(step.x * kLaneOffset);
        car.position = { ax + Fixed::fromInt(step.x) * car.progress + rx, ay + Fixed::fromInt(step.y) * car.progress + ry, Fixed{} };
        if (const auto* tile = _network->tileAt(a))
            car.position.z = Fixed::fromInt(tile->height);
    }

    Fixed TrafficSimulation::gapAhead(const Car& car) const
    {
        // Another car on the same segment ahead of this one, or on the segment after it.
        Fixed best = Fixed::fromInt(1000);
        for (const auto& other : _cars)
        {
            if (other.id == car.id || other.route.empty() || other.state == CarState::parked || other.state == CarState::parking
                || other.state == CarState::gone)
                continue;
            if (car.index >= car.route.size() || other.index >= other.route.size())
                continue;
            if (other.route[other.index] == car.route[car.index] && other.index + 1 < other.route.size() && car.index + 1 < car.route.size()
                && other.route[other.index + 1] == car.route[car.index + 1])
            {
                if (other.progress > car.progress)
                    best = min(best, other.progress - car.progress - Fixed::fromInt(kCarLength));
            }
            else if (car.index + 1 < car.route.size() && other.route[other.index] == car.route[car.index + 1])
            {
                // On the next segment (or about to be): distance to it is the rest of this one plus its progress.
                if (other.index + 1 < other.route.size())
                {
                    const Fixed dist = kTile - car.progress + other.progress - Fixed::fromInt(kCarLength);
                    best = min(best, dist);
                }
            }
        }
        return best;
    }

    Fixed TrafficSimulation::targetSpeed(const Car& car) const
    {
        if (car.index + 1 >= car.route.size())
            return Fixed{};
        const auto* tile = _network->tileAt(car.route[car.index]);
        Fixed speed = unitsPerTick(tile != nullptr ? tile->speedLimitKph : 40);
        // Slow for a turn at the end of this segment and for slopes.
        if (car.index + 2 < car.route.size())
        {
            const int32_t now = directionBetween(car.route[car.index], car.route[car.index + 1]);
            const int32_t next = directionBetween(car.route[car.index + 1], car.route[car.index + 2]);
            if (now != next && car.progress > Fixed::fromInt(12))
                speed = speed * Fixed::ratio(6, 10);
        }
        if (tile != nullptr)
        {
            const auto* next = _network->tileAt(car.route[car.index + 1]);
            if (next != nullptr && std::abs(next->height - tile->height) >= 8)
                speed = speed * Fixed::ratio(7, 10);
        }
        // Keep the gap to the car ahead: stop when closer than the minimum, slow down on approach.
        const Fixed gap = gapAhead(car);
        if (gap < Fixed::fromInt(kMinGap))
            return Fixed{};
        if (gap < Fixed::fromInt(kMinGap * 3))
            speed = min(speed, (gap - Fixed::fromInt(kMinGap)) / 8 + Fixed::ratio(1, 10));
        return speed;
    }

    bool TrafficSimulation::tryEnterJunction(Car& car)
    {
        // STE-020: first come, first served at junctions. A car asks for the junction tile when it is about to reach it.
        if (car.index + 1 >= car.route.size())
            return true;
        const auto& next = car.route[car.index + 1];
        const auto* tile = _network->tileAt(next);
        if (tile == nullptr || !isJunction(tile->connections))
            return true;
        const int64_t key = tileKey(next);
        auto it = _junctionOwner.find(key);
        if (it == _junctionOwner.end() || it->second == car.id)
        {
            _junctionOwner[key] = car.id;
            return true;
        }
        // Someone else holds it; drop the claim if that car is gone.
        if (find(it->second) == nullptr)
        {
            _junctionOwner[key] = car.id;
            return true;
        }
        car.waitingFor = it->second;
        return false;
    }

    void TrafficSimulation::releaseJunctions(Car& car)
    {
        for (auto it = _junctionOwner.begin(); it != _junctionOwner.end();)
        {
            if (it->second != car.id)
            {
                ++it;
                continue;
            }
            // Held while the car is on the approach to it or inside it; released once it has cleared the junction.
            bool still = false;
            if (car.index + 1 < car.route.size() && tileKey(car.route[car.index + 1]) == it->first)
                still = true;
            if (car.index < car.route.size() && tileKey(car.route[car.index]) == it->first && car.progress < Fixed::fromInt(kJunctionClear))
                still = true;
            if (still)
                ++it;
            else
                it = _junctionOwner.erase(it);
        }
    }

    void TrafficSimulation::driveCar(Car& car)
    {
        const bool atApproach = car.progress >= kTile - Fixed::fromInt(kJunctionApproach);
        bool blocked = false;
        car.waitingFor = 0;
        if (car.index + 1 < car.route.size() && !tryEnterJunction(car))
            blocked = true;
        // Stop short of a held junction: the car waits where it asked.
        Fixed target = targetSpeed(car);
        if (blocked && atApproach)
            target = Fixed{};
        else if (blocked)
            target = min(target, Fixed::ratio(5, 10));

        // Accelerate and brake smoothly.
        Fixed accel = Fixed::ratio(6, 100);
        auto* carObj = VehicleModelObject::FindFirstByKind(VehicleModelKind::car);
        if (carObj != nullptr)
        {
            accel = carObj->Acceleration;
            target = min(target, carObj->MaxSpeed);
        }
        if (car.speed < target)
            car.speed = min(target, car.speed + accel);
        else
            car.speed = max(target, car.speed - Fixed::ratio(25, 100));

        if (blocked && atApproach && car.progress + car.speed > kTile - Fixed::fromInt(kJunctionApproach))
        {
            car.progress = kTile - Fixed::fromInt(kJunctionApproach);
            car.speed = Fixed{};
        }
        else
        {
            car.progress += car.speed;
        }
        while (car.progress >= kTile && car.index + 1 < car.route.size())
        {
            car.progress -= kTile;
            car.index++;
        }
        releaseJunctions(car);
        if (car.index + 1 >= car.route.size())
            car.progress = Fixed{};
        updatePosition(car);
    }

    void TrafficSimulation::tick()
    {
        for (size_t i = 0; i < _cars.size(); i++)
        {
            auto& car = _cars[i];
            car.stateTicks++;
            switch (car.state)
            {
                case CarState::driving:
                    driveCar(car);
                    if (car.index + 1 >= car.route.size())
                    {
                        car.state = CarState::parking;
                        car.stateTicks = 0;
                        car.speed = Fixed{};
                    }
                    break;
                case CarState::parking:
                {
                    auto* lot = [&]() -> ParkingLot* {
                        for (auto& l : _lots)
                            if (l.id == car.lotId)
                                return &l;
                        return nullptr;
                    }();
                    if (car.stateTicks >= kParkingTicks)
                    {
                        if (lot != nullptr)
                            if (auto* space = lot->spaceOf(car.id))
                            {
                                space->carId = car.id;
                                space->reservedBy = 0;
                                car.position = parkingSpaceCentre(*space);
                            }
                        car.state = CarState::parked;
                        car.stateTicks = 0;
                        _events.push_back({ TrafficEvent::Type::parked, car.id, car.spaceId });
                    }
                    break;
                }
                case CarState::parked:
                    break;
                case CarState::departing:
                    if (car.stateTicks >= kDepartingTicks)
                    {
                        // STE-022: pull out, free the space, and drive to the nearest road entrance.
                        for (auto& l : _lots)
                            if (l.id == car.lotId)
                            {
                                l.release(car.id);
                                car.route = _network->routeToNearestEntrance(l.streetTile);
                            }
                        car.index = 0;
                        car.progress = Fixed{};
                        car.speed = Fixed{};
                        car.state = CarState::leaving;
                        car.stateTicks = 0;
                        if (car.route.empty())
                        {
                            // No way out: the car vanishes rather than blocking the street forever.
                            car.state = CarState::gone;
                            _events.push_back({ TrafficEvent::Type::despawned, car.id, 0 });
                        }
                        else
                        {
                            updatePosition(car);
                            _events.push_back({ TrafficEvent::Type::departed, car.id, car.spaceId });
                        }
                    }
                    break;
                case CarState::leaving:
                    driveCar(car);
                    if (car.index + 1 >= car.route.size())
                    {
                        car.state = CarState::gone;
                        _events.push_back({ TrafficEvent::Type::despawned, car.id, 0 });
                    }
                    break;
                case CarState::gone:
                    break;
            }
        }
        // Cars that have despawned are removed after the pass so the indices above stay valid.
        for (size_t i = 0; i < _cars.size();)
        {
            if (_cars[i].state == CarState::gone)
            {
                for (auto it = _junctionOwner.begin(); it != _junctionOwner.end();)
                    it = it->second == _cars[i].id ? _junctionOwner.erase(it) : std::next(it);
                _cars.erase(_cars.begin() + static_cast<std::ptrdiff_t>(i));
            }
            else
            {
                i++;
            }
        }
    }
} // namespace OpenRCT2::Golf
