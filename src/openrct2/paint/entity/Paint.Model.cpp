/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Paint.Model.h"

#include "../../drawing/ModelSpriteStore.h"
#include "../Paint.h"

#include <algorithm>
#include <cmath>

namespace OpenRCT2
{
    BoundBoxXYZ ComputeModelBoundBox(const Model::GlbModel& model, const Model::RenderSettings& settings)
    {
        const auto bounds = Model::computeBounds(model, settings);
        if (!bounds.valid)
            return {};
        auto lo = [&](size_t k) { return static_cast<int32_t>(std::floor(bounds.min[k])); };
        auto size = [&](size_t k) { return std::max<int32_t>(1, static_cast<int32_t>(std::ceil(bounds.max[k] - bounds.min[k]))); };
        return { { lo(0), lo(1), lo(2) }, { size(0), size(1), size(2) } };
    }

    void PaintModelEntity(PaintSession& session, const ModelPose& pose, int32_t z)
    {
        auto& store = Drawing::ModelSpriteStore::Instance();
        const auto* model = store.GetModel(pose.modelId);
        if (model == nullptr)
            return;

        const auto key = Model::makeImposterKey(
            pose.modelId, *model, pose.clip, pose.time, pose.orientation, static_cast<uint8_t>(session.CurrentRotation), 0,
            pose.remap);
        const auto image = store.GetImage(key);
        if (image == kImageIndexUndefined)
            return;

        // The box is that of the pose that was rasterised (the bucket centres), in the unrotated frame of the entity.
        Model::RenderSettings settings;
        settings.unitsPerMetre = store.GetUnitsPerMetre(pose.modelId);
        settings.orientation.yaw = Model::bucketToYaw(key.yawBucket);
        settings.orientation.pitch = Model::bucketToTilt(key.pitchBucket);
        settings.orientation.roll = Model::bucketToTilt(key.rollBucket);
        settings.clipIndex = key.clip;
        settings.time = pose.time;
        auto box = ComputeModelBoundBox(*model, settings);
        box.offset.z += z;
        PaintAddImageAsParent(session, ImageId(image), { 0, 0, z }, box);
    }
} // namespace OpenRCT2
