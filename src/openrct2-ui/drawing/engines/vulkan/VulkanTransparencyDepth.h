/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "VulkanDrawCommands.h"

namespace OpenRCT2::Ui
{
    /*
     * Determines an approximation of the number of depth peeling iterations needed
     * to render the command batch. It will never underestimate the number of
     * iterations, but it can overestimate, usually by no more than +2.
     *
     * A straight port of the OpenGL renderer's TransparencyDepth.h/cpp (see that file for the
     * full algorithm description) - pure CPU-side bookkeeping over bounds/clip rectangles, no
     * Vulkan/GPU calls, so the algorithm itself is byte-for-byte identical, just operating on
     * VulkanDrawRectCommand/RectCommandBatch instead of the OpenGL renderer's own types.
     */
    int32_t MaxTransparencyDepth(const RectCommandBatch& transparent);
} // namespace OpenRCT2::Ui
