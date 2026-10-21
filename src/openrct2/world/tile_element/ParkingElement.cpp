/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "ParkingElement.h"

#include "../../object/StreetObject.h"

namespace OpenRCT2
{
    uint16_t ParkingElement::getObjectIndex() const
    {
        return objectIndex;
    }

    void ParkingElement::setObjectIndex(uint16_t newIndex)
    {
        objectIndex = newIndex;
    }

    const StreetObject* ParkingElement::getObject() const
    {
        return StreetObject::GetById(objectIndex);
    }

    bool ParkingElement::isCartBay() const
    {
        return kind == 1;
    }

    void ParkingElement::setCartBay(bool on)
    {
        kind = on ? 1 : 0;
    }

    Direction ParkingElement::getFacing() const
    {
        return facing & 3;
    }

    void ParkingElement::setFacing(Direction direction)
    {
        facing = direction & 3;
    }
} // namespace OpenRCT2
