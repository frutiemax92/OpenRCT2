/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "AnimalEntity.h"

#include "../GameState.h"
#include "EntityRegistry.h"

#include <cmath>

namespace OpenRCT2
{
    template<>
    bool EntityBase::is<AnimalEntity>() const
    {
        return type == EntityType::animal;
    }

    AnimalEntity* AnimalEntity::create(uint32_t animalId, uint16_t objectIndex, const CoordsXYZ& position)
    {
        auto* entity = getGameState().entities.createEntity<AnimalEntity>();
        if (entity == nullptr)
            return nullptr;
        entity->animal = animalId;
        entity->object = objectIndex;
        entity->state = Golf::AnimalState::idle;
        entity->yaw = 0;
        entity->animationTick = 0;
        entity->spriteData.width = 4;
        entity->spriteData.heightMin = 0;
        entity->spriteData.heightMax = 8;
        entity->moveToAndUpdateSpatialIndex(position);
        return entity;
    }

    void AnimalEntity::follow(const CoordsXYZ& position, Golf::AnimalState newState)
    {
        const auto here = getLocation();
        if (position.x != here.x || position.y != here.y)
        {
            const double angle = std::atan2(static_cast<double>(position.y - here.y), static_cast<double>(position.x - here.x));
            double turns = angle / (2.0 * M_PI) + 0.5;
            turns -= std::floor(turns);
            yaw = static_cast<uint8_t>(static_cast<int32_t>(turns * 256.0) & 0xFF);
        }
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

    void AnimalEntity::remove()
    {
        invalidate();
        getGameState().entities.entityRemove(this);
    }
} // namespace OpenRCT2
