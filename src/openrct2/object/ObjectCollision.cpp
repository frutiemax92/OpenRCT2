/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "ObjectCollision.h"

#include <algorithm>

namespace OpenRCT2
{
    static float readUnit(json_t& block, const char* key, float defaultValue)
    {
        if (!block.contains(key))
            return defaultValue;
        return std::clamp(Json::GetNumber<float>(block[key], defaultValue), 0.0f, 1.0f);
    }

    ObjectCollision readObjectCollision(json_t& properties, int32_t defaultHeight, int32_t defaultFootprintX, int32_t defaultFootprintY)
    {
        ObjectCollision result;
        result.height = defaultHeight;
        result.footprintX = defaultFootprintX;
        result.footprintY = defaultFootprintY;
        result.radius = std::max(defaultFootprintX, defaultFootprintY) / 2;

        if (!properties.is_object() || !properties.contains("collision") || !properties["collision"].is_object())
            return result;
        auto& block = properties["collision"];

        if (block.contains("shape"))
        {
            const auto shape = Json::GetString(block["shape"]);
            if (shape == "cylinder")
                result.shape = CollisionShape::cylinder;
            else if (shape == "none")
                result.shape = CollisionShape::none;
            else
                result.shape = CollisionShape::box;
        }
        if (block.contains("height"))
            result.height = std::max(0, Json::GetNumber<int32_t>(block["height"], defaultHeight));
        if (block.contains("footprint") && block["footprint"].is_array() && block["footprint"].size() >= 2)
        {
            result.footprintX = std::max(1, block["footprint"][0].get<int32_t>());
            result.footprintY = std::max(1, block["footprint"][1].get<int32_t>());
            result.radius = std::max(result.footprintX, result.footprintY) / 2;
        }
        if (block.contains("radius"))
            result.radius = std::max(1, Json::GetNumber<int32_t>(block["radius"], result.radius));
        result.restitution = readUnit(block, "restitution", result.restitution);
        result.friction = readUnit(block, "friction", result.friction);
        result.passThroughChance = readUnit(block, "passThroughChance", result.passThroughChance);
        return result;
    }
} // namespace OpenRCT2
