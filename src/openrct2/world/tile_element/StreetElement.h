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
    class StreetObject;

#pragma pack(push, 1)
    // One tile of street (STE-001): a two-way or one-way road on the full-tile grid, connected to its neighbours like a
    // path. A road on a slope stores how many quarter steps it rises and the direction it rises towards (STE-003).
    struct StreetElement : TileElementBase
    {
        static constexpr TileElementType kElementType = TileElementType::street;

        // Bits of the connection set, one per tile edge, indexed by Direction (0 = -x, 1 = +y, 2 = +x, 3 = -y).
        static constexpr uint8_t kConnectionsMask = 0b00001111;
        static constexpr uint8_t kFlowMask = 0b00000011;
        static constexpr uint8_t kRoadEntranceFlag = 0b00000100;
        static constexpr uint8_t kMaxRise = 4; // in quarter steps, the steepest slope level of section 6.2

    private:
        uint16_t objectIndex;   // 5
        uint8_t connections;    // 7 (the connection set in the lower 4 bits)
        uint8_t flags;          // 8 (flow direction in bits 0-1, road entrance in bit 2)
        uint8_t slope;          // 9 (rise in quarter steps in bits 0-2, the direction it rises towards in bits 3-4)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-private-field"
        uint8_t pad0A[6];
#pragma clang diagnostic pop
    public:
        uint16_t getObjectIndex() const;
        void setObjectIndex(uint16_t newIndex);
        const StreetObject* getObject() const;

        uint8_t getConnections() const;
        void setConnections(uint8_t newConnections);
        bool hasConnection(Direction direction) const;
        void setConnection(Direction direction, bool on);

        // For a one-way road, the direction traffic leaves the tile through (STE-002).
        Direction getFlowDirection() const;
        void setFlowDirection(Direction direction);

        // A car spawns and despawns here (STE-005).
        bool isRoadEntrance() const;
        void setRoadEntrance(bool on);

        uint8_t getRise() const;
        Direction getRiseDirection() const;
        void setSlope(uint8_t rise, Direction riseDirection);
    };
    static_assert(sizeof(StreetElement) == kTileElementSize);
#pragma pack(pop)
} // namespace OpenRCT2
