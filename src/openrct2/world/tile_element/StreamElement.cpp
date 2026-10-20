/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "StreamElement.h"

#include "../../object/StreamObject.h"

namespace OpenRCT2
{
    uint16_t StreamElement::getObjectIndex() const
    {
        return objectIndex;
    }

    void StreamElement::setObjectIndex(uint16_t newIndex)
    {
        objectIndex = newIndex;
    }

    const StreamObject* StreamElement::getObject() const
    {
        return StreamObject::GetById(objectIndex);
    }

    uint8_t StreamElement::getEdges() const
    {
        return edges & kEdgesMask;
    }

    void StreamElement::setEdges(uint8_t newEdges)
    {
        edges = newEdges & kEdgesMask;
    }

    bool StreamElement::hasEdge(Direction direction) const
    {
        return (edges & (1 << (direction & 3))) != 0;
    }

    void StreamElement::setEdge(Direction direction, bool on)
    {
        const auto bit = static_cast<uint8_t>(1 << (direction & 3));
        edges = on ? (edges | bit) : (edges & ~bit);
        edges &= kEdgesMask;
    }

    Direction StreamElement::getFlowDirection() const
    {
        return flowDirection & 3;
    }

    void StreamElement::setFlowDirection(Direction direction)
    {
        flowDirection = direction & 3;
    }
} // namespace OpenRCT2
