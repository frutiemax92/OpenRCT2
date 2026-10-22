/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "GolfVehicleEntity.h"

#include "../GameState.h"
#include "EntityRegistry.h"

#include <cmath>

namespace OpenRCT2
{
    template<>
    bool EntityBase::is<GolfVehicleEntity>() const
    {
        return type == EntityType::golfVehicle;
    }

    GolfVehicleEntity* GolfVehicleEntity::create(Kind kind, uint32_t vehicleId, const CoordsXYZ& position)
    {
        auto* entity = getGameState().entities.createEntity<GolfVehicleEntity>();
        if (entity == nullptr)
            return nullptr;
        entity->kind = kind;
        entity->vehicle = vehicleId;
        entity->yaw = 0;
        // About the size of a car (24 x 12 units) and 16 tall: the box used for picking and sorting.
        entity->spriteData.width = 24;
        entity->spriteData.heightMin = 20;
        entity->spriteData.heightMax = 12;
        entity->moveToAndUpdateSpatialIndex(position);
        return entity;
    }

    void GolfVehicleEntity::follow(const CoordsXYZ& position, int32_t heading)
    {
        const auto here = getLocation();
        if (heading >= 0)
        {
            // The rasteriser turns counter-clockwise seen from above, and the models face -x at yaw 0, so the directions
            // 0 = -x, 1 = +y, 2 = +x, 3 = -y are the yaws 0, 192, 128 and 64.
            yaw = static_cast<uint8_t>(((4 - (heading & 3)) & 3) * 64);
        }
        else if (position.x != here.x || position.y != here.y)
        {
            const double angle = std::atan2(static_cast<double>(position.y - here.y), static_cast<double>(position.x - here.x));
            double turns = angle / (2.0 * M_PI) + 0.5;
            turns -= std::floor(turns);
            yaw = static_cast<uint8_t>(static_cast<int32_t>(turns * 256.0) & 0xFF);
        }
        if (position != here)
        {
            invalidate();
            moveToAndUpdateSpatialIndex(position);
        }
    }

    void GolfVehicleEntity::remove()
    {
        invalidate();
        getGameState().entities.entityRemove(this);
    }
} // namespace OpenRCT2
