/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Paint.Golfer.h"

#include "../../drawing/ModelSpriteStore.h"
#include "../../entity/GolferEntity.h"
#include "../../object/GolferModelObject.h"
#include "../../profiling/Profiling.h"
#include "../Paint.h"
#include "Paint.Model.h"

#include <cmath>

namespace OpenRCT2
{
    using Golf::GolferState;

    const char* golferClipName(GolferState state, uint32_t animationTick)
    {
        switch (state)
        {
            case GolferState::walkingToCheckIn:
            case GolferState::goingToTee:
            case GolferState::goingToCart:
            case GolferState::walkingToBall:
            case GolferState::arriving:
            case GolferState::leaving:
            case GolferState::returningCart:
            case GolferState::followingGroup:
                return "walk";
            case GolferState::swinging:
                return "swing_full";
            case GolferState::putting:
                return "swing_putt";
            case GolferState::resting:
                return "sit_bench";
            case GolferState::talking:
                return "talk";
            case GolferState::holeFinished:
                // Celebrates for a moment, then stands waiting for the rest of the group (40 game ticks a second).
                return animationTick < 120 ? "celebrate" : "idle";
            case GolferState::rageQuitting:
                return "rage_quit";
            case GolferState::searchingBall:
                return "groan";
            default:
                return "idle";
        }
    }

    void PaintGolfer(PaintSession& session, const GolferEntity& golfer, int32_t imageDirection)
    {
        PROFILED_FUNCTION();

        // A golfer is a few pixels tall: too small to bother with when zoomed far out.
        if (session.rt.zoom_level > ZoomLevel{ 1 })
            return;
        const auto modelId = GolferModelObject::ActiveModelId;
        if (modelId == 0)
            return;
        const auto* model = Drawing::ModelSpriteStore::Instance().GetModel(modelId);
        if (model == nullptr)
            return;

        ModelPose pose;
        pose.modelId = modelId;
        const auto* clipName = golferClipName(golfer.state, golfer.animationTick);
        for (size_t i = 0; i < model->clips.size(); i++)
        {
            if (model->clips[i].name == clipName)
            {
                pose.clip = static_cast<int32_t>(i);
                pose.time = static_cast<float>(golfer.animationTick) / 40.0f; // 40 game ticks a second
                break;
            }
        }
        pose.orientation.yaw = static_cast<float>(golfer.yaw) * (2.0f * static_cast<float>(M_PI) / 256.0f);
        PaintModelEntity(session, pose, golfer.z);
    }
} // namespace OpenRCT2
