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
    // A tile of parking (STE-010): two parking spaces side by side, facing the street on one edge. The surface is laid with
    // the road texture of a street object. A parking lot is made of these tiles next to each other, connected to a street
    // on the edge they face and to a path on another edge. A cart bay (CRT-002) is the same tile for a golf cart: one space
    // in the middle, where a cart waits to be rented.
    struct ParkingElement : TileElementBase
    {
        static constexpr TileElementType kElementType = TileElementType::parking;

    private:
        uint16_t objectIndex; // 5 (the street object that gives the surface)
        uint8_t facing;       // 7 (the Direction of the edge the cars drive in through; low 2 bits)
        uint8_t kind;         // 8 (0 car parking, 1 cart bay)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-private-field"
        uint8_t pad09[7];
#pragma clang diagnostic pop
    public:
        uint16_t getObjectIndex() const;
        void setObjectIndex(uint16_t newIndex);
        const StreetObject* getObject() const;

        Direction getFacing() const;
        void setFacing(Direction direction);

        // A cart bay holds one golf cart (CRT-002); otherwise the tile is for cars.
        bool isCartBay() const;
        void setCartBay(bool on);
    };
    static_assert(sizeof(ParkingElement) == kTileElementSize);
#pragma pack(pop)
} // namespace OpenRCT2
