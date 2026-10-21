/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "GolfDirector.h"

#include <cstdint>
#include <span>
#include <vector>

namespace OpenRCT2::Golf
{
    // Saving the golfers (SAV-001): everything the director and the simulation keep between two ticks, the golfers on the
    // course, the registry of returning golfers, the tee sheets, the day's accounts, the popularity and the tournament.
    // The result is one block of bytes that the park file stores in its golf simulation chunk.
    // 2: golfers remember the car they came in (STE-012).
    // 3: the golf carts and who rides them (CRT-002 to CRT-005).
    constexpr uint32_t kGolfSaveFormat = 3;

    std::vector<uint8_t> saveDirector(const GolfDirector& director);
    // Restores the state saved by saveDirector into a director built on the same courses. Returns false (and leaves the
    // director as it was) when the data is damaged or from a newer format.
    bool loadDirector(GolfDirector& director, std::span<const uint8_t> data);
} // namespace OpenRCT2::Golf
