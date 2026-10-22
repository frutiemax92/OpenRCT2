/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../golf/Golfer.h"
#include "../world/Location.hpp"
#include "EntityBase.h"

namespace OpenRCT2
{
    // The golfer as the map sees it (GLF-001): the thing that is drawn, found by the spatial index and picked with the
    // mouse. The golfer's behaviour and data live in the golf simulation (Golf::GolfSimulation); the runtime keeps one of
    // these entities per simulated golfer, moves it, and removes it when the golfer leaves. It is not saved: loading a park
    // starts with no golfers on the course, and the runtime removes entities that no golfer owns.
    struct GolferEntity : EntityBase
    {
        static constexpr auto kEntityType = EntityType::golfer;

        uint32_t golfer = 0; // id of the Golf::Golfer this entity shows
        Golf::GolferState state = Golf::GolferState::arriving;
        uint8_t yaw = 0;     // 0..255 for a full turn counter-clockwise from above: 0 faces -x, 64 faces -y, 128 +x, 192 +y
        CoordsXY facingFrom{}; // where the heading was last taken from (see UpdateFacingFromTravel)
        uint32_t animationTick = 0;

        // Creates an entity for a golfer at a position. Returns nullptr when the entity limit is hit.
        static GolferEntity* create(uint32_t golferId, const CoordsXYZ& position);
        // Moves to a position and turns towards the direction of travel; sets the state that picks the animation.
        void follow(const CoordsXYZ& position, Golf::GolferState newState);
        void remove();
    };
} // namespace OpenRCT2
