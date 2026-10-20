/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "BuildingElement.h"

#include "../../object/BuildingObject.h"

namespace OpenRCT2
{
    uint16_t BuildingElement::getObjectIndex() const
    {
        return objectIndex;
    }

    void BuildingElement::setObjectIndex(uint16_t newIndex)
    {
        objectIndex = newIndex;
    }

    const BuildingObject* BuildingElement::getObject() const
    {
        return BuildingObject::GetById(objectIndex);
    }

    Direction BuildingElement::getRotation() const
    {
        return rotation & 3;
    }

    void BuildingElement::setRotation(Direction newRotation)
    {
        rotation = newRotation & 3;
    }

    uint8_t BuildingElement::getPartX() const
    {
        return part & 0x0F;
    }

    uint8_t BuildingElement::getPartY() const
    {
        return part >> 4;
    }

    void BuildingElement::setPart(uint8_t x, uint8_t y)
    {
        part = static_cast<uint8_t>((x & 0x0F) | (y << 4));
    }
} // namespace OpenRCT2
