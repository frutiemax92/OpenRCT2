/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "GolferEntity.h"
#include "Facing.h"

#include "../GameState.h"
#include "EntityRegistry.h"

#include <cmath>

namespace OpenRCT2
{
    template<>
    bool EntityBase::is<GolferEntity>() const
    {
        return type == EntityType::golfer;
    }

    GolferEntity* GolferEntity::create(uint32_t golferId, const CoordsXYZ& position)
    {
        auto* entity = getGameState().entities.createEntity<GolferEntity>();
        if (entity == nullptr)
            return nullptr;
        entity->golfer = golferId;
        entity->state = Golf::GolferState::arriving;
        entity->yaw = 0;
        entity->animationTick = 0;
        // About 6 feet tall: a golfer is drawn as a 3D model, this is the box used for picking and sorting.
        entity->spriteData.width = 16;
        entity->spriteData.heightMin = 28;
        entity->spriteData.heightMax = 8;
        entity->facingFrom = { position.x, position.y };
        entity->moveToAndUpdateSpatialIndex(position);
        return entity;
    }

    void GolferEntity::follow(const CoordsXYZ& position, Golf::GolferState newState)
    {
        const auto here = getLocation();
        UpdateFacingFromTravel(yaw, facingFrom, { position.x, position.y });
        if (newState != state)
            animationTick = 0;
        else
            animationTick++;
        state = newState;
        if (position != here)
        {
            invalidate();
            moveToAndUpdateSpatialIndex(position);
        }
    }

    void GolferEntity::remove()
    {
        invalidate();
        getGameState().entities.entityRemove(this);
    }
} // namespace OpenRCT2
