/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include <cstdint>

struct PaintSession;

namespace OpenRCT2
{
    struct GolfBall;

    // Draws the ball, its shadow on the surface below and, while it flies, a short trail (BAL-002).
    void PaintGolfBall(PaintSession& session, const GolfBall& ball, int32_t imageDirection);
} // namespace OpenRCT2
