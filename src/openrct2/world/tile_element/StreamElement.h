/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../../Identifiers.h"
#include "TileElementBase.h"

namespace OpenRCT2
{
    class StreamObject;

#pragma pack(push, 1)
    // A water stream lying on the surface of its tile (STR-001). It has no slope of its own: the stream texture is
    // projected onto the surface triangles, so it follows any terrain shape (STR-003).
    struct StreamElement : TileElementBase
    {
        static constexpr TileElementType kElementType = TileElementType::stream;

        // Bits of the connection set, one per tile edge, indexed by Direction (0 = -x, 1 = +y, 2 = +x, 3 = -y).
        static constexpr uint8_t kEdgesMask = 0b00001111;

    private:
        uint16_t objectIndex;   // 5
        uint8_t edges;          // 7 (connection set in the lower 4 bits)
        uint8_t flowDirection;  // 8 (Direction the water leaves the tile through, used on flat tiles)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-private-field"
        uint8_t pad09[7];
#pragma clang diagnostic pop
    public:
        uint16_t getObjectIndex() const;
        void setObjectIndex(uint16_t newIndex);
        const StreamObject* getObject() const;

        uint8_t getEdges() const;
        void setEdges(uint8_t newEdges);
        bool hasEdge(Direction direction) const;
        void setEdge(Direction direction, bool on);

        Direction getFlowDirection() const;
        void setFlowDirection(Direction direction);
    };
    static_assert(sizeof(StreamElement) == kTileElementSize);
#pragma pack(pop)
} // namespace OpenRCT2
