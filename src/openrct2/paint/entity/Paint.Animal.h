/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../../golf/Animals.h"

#include <cstdint>

struct PaintSession;

namespace OpenRCT2
{
    struct AnimalEntity;

    // The clip an animal plays in a state (ANM-001): idle, walk, run, flee, and fly or swim when it has them.
    const char* animalClipName(Golf::AnimalState state);

    // Draws an animal as the model of its species (R3D-004).
    void PaintAnimal(PaintSession& session, const AnimalEntity& animal, int32_t imageDirection);
} // namespace OpenRCT2
