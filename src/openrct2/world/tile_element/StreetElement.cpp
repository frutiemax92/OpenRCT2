/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "StreetElement.h"

#include "../../object/StreetObject.h"

#include <algorithm>

namespace OpenRCT2
{
    uint16_t StreetElement::getObjectIndex() const
    {
        return objectIndex;
    }

    void StreetElement::setObjectIndex(uint16_t newIndex)
    {
        objectIndex = newIndex;
    }

    const StreetObject* StreetElement::getObject() const
    {
        return StreetObject::GetById(objectIndex);
    }

    uint8_t StreetElement::getConnections() const
    {
        return connections & kConnectionsMask;
    }

    void StreetElement::setConnections(uint8_t newConnections)
    {
        connections = newConnections & kConnectionsMask;
    }

    bool StreetElement::hasConnection(Direction direction) const
    {
        return (connections & (1 << (direction & 3))) != 0;
    }

    void StreetElement::setConnection(Direction direction, bool on)
    {
        const auto bit = static_cast<uint8_t>(1 << (direction & 3));
        connections = on ? (connections | bit) : (connections & ~bit);
        connections &= kConnectionsMask;
    }

    Direction StreetElement::getFlowDirection() const
    {
        return flags & kFlowMask;
    }

    void StreetElement::setFlowDirection(Direction direction)
    {
        flags = static_cast<uint8_t>((flags & ~kFlowMask) | (direction & kFlowMask));
    }

    bool StreetElement::isRoadEntrance() const
    {
        return (flags & kRoadEntranceFlag) != 0;
    }

    void StreetElement::setRoadEntrance(bool on)
    {
        flags = on ? (flags | kRoadEntranceFlag) : (flags & ~kRoadEntranceFlag);
    }

    uint8_t StreetElement::getRise() const
    {
        return slope & 0b111;
    }

    Direction StreetElement::getRiseDirection() const
    {
        return (slope >> 3) & 3;
    }

    void StreetElement::setSlope(uint8_t rise, Direction riseDirection)
    {
        slope = static_cast<uint8_t>((std::min<uint8_t>(rise, kMaxRise) & 0b111) | ((riseDirection & 3) << 3));
    }
} // namespace OpenRCT2
