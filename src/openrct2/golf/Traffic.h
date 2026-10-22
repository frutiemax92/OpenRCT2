/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../core/FixedVec.hpp"
#include "../model/TerrainAlignment.h"
#include "ObstacleGrid.h"
#include "StreetNetwork.h"

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace OpenRCT2::Golf
{
    // Parking lots (section 12.2) and cars (section 12.3).

    // STE-010: a parking tile holds two spaces, oriented towards the street.
    struct ParkingSpace
    {
        uint32_t id = 0;
        TileCoordsXY tile;
        uint8_t slot = 0;          // 0 or 1 within the tile
        uint8_t facing = 0;        // direction of the street side
        uint32_t carId = 0;        // the car standing here
        uint32_t reservedBy = 0;   // a car on its way (counted as taken, STE-011)
    };

    struct ParkingLot
    {
        uint32_t id = 0;
        TileCoordsXY streetTile;   // the street tile the lot connects to
        TileCoordsXY pathTile;     // the path tile golfers walk to (STE-012)
        std::vector<ParkingSpace> spaces;

        int32_t capacity() const
        {
            return static_cast<int32_t>(spaces.size());
        }
        // STE-011: spaces that are neither taken nor reserved.
        int32_t freeSpaces() const;
        ParkingSpace* reserve(uint32_t carId);
        // The space that reserve() would take next; nullptr when the lot is full.
        const ParkingSpace* firstFree() const;
        void release(uint32_t carId);
        ParkingSpace* spaceOf(uint32_t carId);
        // Adds the two spaces of a tile.
        void addTile(const TileCoordsXY& tile, uint8_t facing, uint32_t& nextSpaceId);
    };

    // Centre of a space in world units: two spaces share a tile, one each side of its centre across the facing direction.
    FixedVec3 parkingSpaceCentre(const ParkingSpace& space);
    // The street tile in front of a space: the one a car drives to, and leaves from, so it never crosses the other spaces.
    TileCoordsXY parkingSpaceStreetTile(const ParkingSpace& space);

    enum class CarState : uint8_t
    {
        driving,    // on the way to a parking space
        parking,    // pulling into the space
        parked,     // golfers are out playing
        departing,  // pulling out
        leaving,    // driving to a road entrance
        gone,       // despawned at the entrance
    };

    struct Car
    {
        uint32_t id = 0;
        uint32_t groupId = 0;
        std::vector<uint32_t> golferIds;   // STE-021: 1 to 4
        CarState state = CarState::driving;
        std::vector<TileCoordsXY> route;   // tiles to drive through
        size_t index = 0;                  // the tile the car is leaving (route[index] to route[index + 1])
        Fixed progress{};                  // 0 at the centre of route[index], 32 at the centre of the next tile
        Fixed speed{};
        uint32_t lotId = 0;
        uint32_t spaceId = 0;
        int32_t stateTicks = 0;
        FixedVec3 position;                // world position with the lane offset
        FixedVec3 parkFrom;                // where the car stood on the street when it began to pull into its space
        FixedVec3 parkVia;                 // the corner of the 90 degree turn between the street and the space
        int32_t heading = 0;               // direction 0-3 of travel, for drawing
        uint32_t waitingFor = 0;           // car or junction blocking it, for the tests
    };

    // Constants of the lane-following model (STE-020).
    constexpr int32_t kCarLength = 12;           // units
    constexpr int32_t kMinGap = 14;              // bumper to bumper, units
    Model::ContactPoints carContactPoints();
    constexpr int32_t kLaneOffset = 6;           // units to the right of the centre line
    constexpr int32_t kParkingTicks = 50;
    constexpr int32_t kDepartingTicks = 50;
    constexpr int32_t kJunctionApproach = 14;    // units before the centre of a junction where a car asks to enter
    constexpr int32_t kJunctionClear = 14;       // units into the next tile before a car has cleared the junction

    struct TrafficEvent
    {
        enum class Type : uint8_t
        {
            spawned,
            parked,      // the golfers may get out: value = space id
            departed,    // pulled out of the space and drove off
            despawned,   // reached a road entrance
        };
        Type type = Type::spawned;
        uint32_t carId = 0;
        uint32_t value = 0;
    };

    // What a save keeps of the traffic (the lots themselves come back from the map): the cars, who stands in which space and the
    // numbers the ids go on from. `layout` and the first ids say which streets and lots the ids belong to.
    struct TrafficSnapshot
    {
        struct Taken
        {
            uint32_t spaceId = 0;
            uint32_t carId = 0;
            uint32_t reservedBy = 0;
        };
        struct Junction
        {
            int64_t tile = 0;
            uint32_t carId = 0;
        };
        bool valid = false;
        uint64_t layout = 0;
        uint32_t layoutLotId = 1;    // the first lot id of the layout
        uint32_t layoutSpaceId = 1;  // the first space id of the layout
        uint32_t nextSpaceId = 1;
        uint32_t nextCarId = 1;
        uint32_t nextLotId = 1;
        std::vector<Car> cars;
        std::vector<Taken> taken;
        std::vector<Junction> junctions;
    };

    class TrafficSimulation
    {
    public:
        explicit TrafficSimulation(const StreetNetwork* network)
            : _network(network)
        {
        }

        ParkingLot& addLot(const TileCoordsXY& streetTile, const TileCoordsXY& pathTile);
        // Forgets the lots, the cars and who was in the way of whom: the streets or the parking changed under them. Ids go on.
        void reset();
        uint32_t nextLotId() const
        {
            return _nextLotId;
        }
        void setNextLotId(uint32_t id)
        {
            _nextLotId = id;
        }
        // Writes the cars and the spaces they hold into `out` (the layout fields are the caller's).
        void exportState(TrafficSnapshot& out) const;
        // Puts the cars back into lots rebuilt with the same ids; cars of lots or spaces that are gone are dropped.
        void importState(const TrafficSnapshot& in);
        // The space with an id, in whichever lot it is; nullptr when there is none.
        const ParkingSpace* findSpace(uint32_t spaceId) const;
        std::vector<ParkingLot>& lots()
        {
            return _lots;
        }
        const std::vector<ParkingLot>& lots() const
        {
            return _lots;
        }
        // Spaces in all lots that are neither taken nor reserved.
        int32_t freeSpaces() const;
        int32_t capacity() const;

        // STE-021: a car with a group of golfers appears at a road entrance and heads for a lot with a free space. Returns
        // 0 and spawns nothing when there is no road entrance, no free space (STE-011) or no way to get there.
        uint32_t spawn(const std::vector<uint32_t>& golferIds, uint32_t groupId = 0);
        // STE-022: the group is back: the car pulls out and drives to the nearest road entrance. False when it is not parked.
        bool depart(uint32_t carId);

        void tick();

        Car* find(uint32_t carId);
        const std::vector<Car>& cars() const
        {
            return _cars;
        }
        size_t carCount() const
        {
            return _cars.size();
        }
        std::vector<TrafficEvent> takeEvents();

        // STE-024: cars are obstacles for balls.
        Obstacle obstacleOf(const Car& car) const;

    private:
        void driveCar(Car& car);
        TileCoordsXY findLotStreet(uint32_t lotId) const;
        // Pulling into a space and out of it: along the street to the space, then straight in (one 90 degree turn).
        void moveAlongTurn(Car& car, const FixedVec3& from, const FixedVec3& to, Fixed fraction) const;
        Fixed targetSpeed(const Car& car) const;
        Fixed gapAhead(const Car& car) const;
        bool tryEnterJunction(Car& car);
        void releaseJunctions(Car& car);
        void updatePosition(Car& car);
        static Fixed unitsPerTick(int32_t kph);

        const StreetNetwork* _network;
        std::vector<ParkingLot> _lots;
        std::vector<Car> _cars;
        std::vector<TrafficEvent> _events;
        std::unordered_map<int64_t, uint32_t> _junctionOwner; // tile key -> car id
        uint32_t _nextCarId = 1;
        uint32_t _nextLotId = 1;
        uint32_t _nextSpaceId = 1;
    };
} // namespace OpenRCT2::Golf
