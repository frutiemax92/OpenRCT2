/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "EntityBase.h"

namespace OpenRCT2
{
    // A car or a golf cart as the map sees it (STE-020, CRT-002): drawn as the vehicle model of its kind, found by the
    // spatial index and an obstacle for the balls. The traffic simulation owns the vehicle; the runtime keeps one of these
    // entities per vehicle and moves it, like the golfer entities (GolferEntity). It is not saved.
    struct GolfVehicleEntity : EntityBase
    {
        static constexpr auto kEntityType = EntityType::golfVehicle;

        enum class Kind : uint8_t
        {
            car,
            cart,
        };

        Kind kind = Kind::car;
        uint32_t vehicle = 0; // id of the vehicle in the simulation (Car::id, or the cart)
        uint8_t yaw = 0;      // 0..255 for a full turn counter-clockwise from above: 0 faces -x, 64 faces -y, 128 +x, 192 +y

        // Creates an entity for a vehicle at a position. Returns nullptr when the entity limit is hit.
        static GolfVehicleEntity* create(Kind kind, uint32_t vehicleId, const CoordsXYZ& position);
        // Moves to a position and turns to a direction (0-3, the direction numbers of the map: 0 = -x, 1 = +y, 2 = +x,
        // 3 = -y) or to the direction of travel when `heading` is negative.
        void follow(const CoordsXYZ& position, int32_t heading);
        void remove();
    };
} // namespace OpenRCT2
