/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../golf/Animals.h"
#include "../world/Location.hpp"
#include "EntityBase.h"

namespace OpenRCT2
{
    // An animal as the map sees it (ANM-001): drawn as the model of its species in the clip of what it is doing, found by
    // the spatial index and picked with the mouse. The animal simulation owns the animal; the runtime keeps one of these
    // entities per animal and moves it, like the golfer entities. It is not saved.
    struct AnimalEntity : EntityBase
    {
        static constexpr auto kEntityType = EntityType::animal;

        uint32_t animal = 0;                       // id of the animal in the simulation
        uint16_t object = 0;                       // entry index of the animal object that gives the model
        Golf::AnimalState state = Golf::AnimalState::idle;
        uint8_t yaw = 0;                           // counter-clockwise from above: 0 faces -x, 64 -y, 128 +x, 192 +y
        CoordsXY facingFrom{}; // where the heading was last taken from (see UpdateFacingFromTravel)
        uint32_t animationTick = 0;

        static AnimalEntity* create(uint32_t animalId, uint16_t objectIndex, const CoordsXYZ& position);
        // Moves to a position and turns the way it moves; a change of state restarts the animation.
        void follow(const CoordsXYZ& position, Golf::AnimalState newState);
        void remove();
    };
} // namespace OpenRCT2
