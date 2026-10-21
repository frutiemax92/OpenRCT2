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
    struct GolfVehicleEntity;

    // Draws a car or a golf cart as the vehicle model of its kind (R3D-004).
    void PaintGolfVehicle(PaintSession& session, const GolfVehicleEntity& vehicle, int32_t imageDirection);
} // namespace OpenRCT2
