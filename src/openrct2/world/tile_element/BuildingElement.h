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
    class BuildingObject;

#pragma pack(push, 1)
    // One tile of a building (BLD-001). A building of N tiles is N elements sharing the object and the rotation; each one
    // remembers which tile of the footprint it is, in the building's own frame, so any of them can find the others.
    struct BuildingElement : TileElementBase
    {
        static constexpr TileElementType kElementType = TileElementType::building;

    private:
        uint16_t objectIndex; // 5
        uint8_t rotation;     // 7 (quarter turns, lower 2 bits)
        uint8_t part;         // 8 (x in the lower nibble, y in the upper one, in the building's own frame)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-private-field"
        uint8_t pad09[7];
#pragma clang diagnostic pop
    public:
        uint16_t getObjectIndex() const;
        void setObjectIndex(uint16_t newIndex);
        const BuildingObject* getObject() const;

        Direction getRotation() const;
        void setRotation(Direction newRotation);

        uint8_t getPartX() const;
        uint8_t getPartY() const;
        void setPart(uint8_t x, uint8_t y);
    };
    static_assert(sizeof(BuildingElement) == kTileElementSize);
#pragma pack(pop)
} // namespace OpenRCT2
