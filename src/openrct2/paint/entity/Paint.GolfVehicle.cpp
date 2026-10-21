/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Paint.GolfVehicle.h"

#include "../../drawing/ModelSpriteStore.h"
#include "../../entity/GolfVehicleEntity.h"
#include "../../object/VehicleModelObject.h"
#include "../../profiling/Profiling.h"
#include "../Paint.h"
#include "Paint.Model.h"

#include <cmath>

namespace OpenRCT2
{
    void PaintGolfVehicle(PaintSession& session, const GolfVehicleEntity& vehicle, int32_t imageDirection)
    {
        PROFILED_FUNCTION();

        if (session.rt.zoom_level > ZoomLevel{ 1 })
            return;
        const auto modelId = vehicle.kind == GolfVehicleEntity::Kind::car ? VehicleModelObject::ActiveCarModelId
                                                                          : VehicleModelObject::ActiveCartModelId;
        if (modelId == 0)
            return;

        ModelPose pose;
        pose.modelId = modelId;
        pose.orientation.yaw = static_cast<float>(vehicle.yaw) * (2.0f * static_cast<float>(M_PI) / 256.0f);
        PaintModelEntity(session, pose, vehicle.z);
    }
} // namespace OpenRCT2
