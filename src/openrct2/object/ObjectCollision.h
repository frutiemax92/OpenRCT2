/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../core/Json.hpp"

#include <cstdint>

namespace OpenRCT2
{
    enum class CollisionShape : uint8_t
    {
        box,
        cylinder,
        none,
    };

    // The physics block any object a ball can hit accepts (spec §5.3).
    struct ObjectCollision
    {
        CollisionShape shape = CollisionShape::box;
        int32_t height = 0;     // z units, from base
        int32_t footprintX = 32; // xy units; for a cylinder the diameter
        int32_t footprintY = 32;
        int32_t radius = 16;     // cylinder only
        float restitution = 0.4f;
        float friction = 0.3f;
        float passThroughChance = 0.0f;
    };

    /**
     * Reads the optional "collision" block of an object's properties. Missing fields (or a missing block) take the
     * defaults of §5.3, derived from the object's clearance height and footprint (in xy units).
     */
    ObjectCollision readObjectCollision(json_t& properties, int32_t defaultHeight, int32_t defaultFootprintX = 32, int32_t defaultFootprintY = 32);
} // namespace OpenRCT2
