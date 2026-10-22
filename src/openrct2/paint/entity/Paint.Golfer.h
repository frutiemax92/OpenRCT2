/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../../golf/Golfer.h"

#include <cstdint>

struct PaintSession;

namespace OpenRCT2
{
    struct GolferEntity;

    // The animation clip a golfer plays in a state (GLF-030, GLF-032).
    // `animationTick` counts the ticks in the state: a golfer who has holed out celebrates for a moment, then stands.
    const char* golferClipName(Golf::GolferState state, uint32_t animationTick = 0);

    // Draws a golfer as the golfer model, in the clip of the state it is in, turned towards where it walks (R3D-004).
    void PaintGolfer(PaintSession& session, const GolferEntity& golfer, int32_t imageDirection);
} // namespace OpenRCT2
