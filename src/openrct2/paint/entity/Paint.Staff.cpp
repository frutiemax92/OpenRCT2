/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Paint.Staff.h"

#include "../../GameState.h"
#include "../../drawing/LightFX.h"
#include "../../drawing/ModelSpriteStore.h"
#include "../../entity/Staff.h"
#include "../../object/GolferModelObject.h"
#include "../../profiling/Profiling.h"
#include "../Paint.h"
#include "Paint.Model.h"
#include "Paint.Peep.h"

using namespace OpenRCT2;

namespace LightFx = OpenRCT2::Drawing::LightFx;

void PaintStaff(PaintSession& session, const Staff& staff, int32_t orientation)
{
    PROFILED_FUNCTION();

    if (session.rt.zoom_level > ZoomLevel{ 2 })
    {
        return;
    }

    PaintStaffLightingEffects(staff);

    // Staff are 3D models like the golfers: the golfer model, walking while they move and idle otherwise.
    const auto modelId = GolferModelObject::ActiveModelId;
    if (modelId == 0)
        return;
    const auto* model = Drawing::ModelSpriteStore::Instance().GetModel(modelId);
    if (model == nullptr)
        return;

    ModelPose pose;
    pose.modelId = modelId;
    const char* clipName = staff.state == PeepState::walking ? "walk" : "idle";
    for (size_t i = 0; i < model->clips.size(); i++)
    {
        if (model->clips[i].name == clipName)
        {
            pose.clip = static_cast<int32_t>(i);
            pose.time = static_cast<float>(getGameState().currentTicks) / 40.0f;
            break;
        }
    }
    // The staff orientation runs 0..31 (8 per direction); the model yaw is in radians.
    pose.orientation.yaw = static_cast<float>(staff.orientation) * (2.0f * 3.14159265f / 32.0f);
    PaintModelEntity(session, pose, staff.z);
}

void PaintStaffLightingEffects(const Staff& staff)
{
    if (!LightFx::IsAvailable())
        return;

    auto loc = staff.getLocation();
    switch (staff.orientation)
    {
        case 0:
            loc.x -= 10;
            break;
        case 8:
            loc.y += 10;
            break;
        case 16:
            loc.x += 10;
            break;
        case 24:
            loc.y -= 10;
            break;
        default:
            return;
    }

    LightFx::Add3DLight(staff, 0, loc, LightFx::LightType::spot1);
}
