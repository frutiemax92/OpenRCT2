/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Paint.Animal.h"

#include "../../drawing/ModelSpriteStore.h"
#include "../../entity/AnimalEntity.h"
#include "../../object/AnimalObject.h"
#include "../../profiling/Profiling.h"
#include "../Paint.h"
#include "Paint.Model.h"

#include <cmath>

namespace OpenRCT2
{
    using Golf::AnimalState;

    const char* animalClipName(AnimalState state)
    {
        switch (state)
        {
            case AnimalState::walk:
                return "walk";
            case AnimalState::run:
                return "run";
            case AnimalState::flee:
                return "flee";
            case AnimalState::fly:
                return "fly";
            case AnimalState::swim:
                return "swim";
            default:
                return "idle";
        }
    }

    void PaintAnimal(PaintSession& session, const AnimalEntity& animal, int32_t imageDirection)
    {
        PROFILED_FUNCTION();

        // Animals are small: not worth drawing zoomed far out.
        if (session.rt.zoom_level > ZoomLevel{ 1 })
            return;
        const auto* object = AnimalObject::GetById(animal.object);
        if (object == nullptr || object->ModelId == 0)
            return;
        const auto* model = Drawing::ModelSpriteStore::Instance().GetModel(object->ModelId);
        if (model == nullptr)
            return;

        ModelPose pose;
        pose.modelId = object->ModelId;
        // A species without the clip (a swimming duck has "swim", a deer does not) plays the closest it has.
        const char* wanted = animalClipName(animal.state);
        const Model::Clip* clip = model->findClip(wanted);
        if (clip == nullptr && (animal.state == AnimalState::swim || animal.state == AnimalState::fly))
            clip = model->findClip("walk");
        if (clip == nullptr)
            clip = model->findClip("idle");
        if (clip != nullptr)
        {
            pose.clip = static_cast<int32_t>(clip - model->clips.data());
            pose.time = static_cast<float>(animal.animationTick) / 40.0f; // 40 game ticks a second
        }
        pose.orientation.yaw = static_cast<float>(animal.yaw) * (2.0f * static_cast<float>(M_PI) / 256.0f);
        PaintModelEntity(session, pose, animal.z);
    }
} // namespace OpenRCT2
