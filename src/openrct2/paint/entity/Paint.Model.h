/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../../model/ImposterCache.h"
#include "../../world/Location.hpp"
#include "../Boundbox.h"

struct PaintSession;

namespace OpenRCT2
{
    struct ModelPose
    {
        uint32_t modelId = 0;
        int32_t clip = -1;
        float time = 0;
        Model::Orientation orientation;
        std::array<uint8_t, Model::kRemapSlots> remap{};
    };

    // Bounding box of a posed model relative to the entity position, for paint sorting (R3D-004). The box covers the
    // axis aligned extent of the model, never smaller than 1 unit so thin models still sort.
    BoundBoxXYZ ComputeModelBoundBox(const Model::GlbModel& model, const Model::RenderSettings& settings);

    // Paints a 3D entity as a normal sprite of the paint session at its position (R3D-004).
    void PaintModelEntity(PaintSession& session, const ModelPose& pose, int32_t z);
} // namespace OpenRCT2
