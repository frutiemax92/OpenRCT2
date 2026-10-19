/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../../object/ObjectTypes.h"
#include "../MapOwnership.h"
#include "../TerrainShape.h"
#include "TileElementBase.h"

namespace OpenRCT2
{
    namespace Drawing
    {
        enum class Colour : uint8_t;
    }

    enum
    {
        SURFACE_ELEMENT_HAS_TRACK_THAT_NEEDS_WATER = (1 << 6),
    };

    enum GrassLength : uint8_t
    {
        GRASS_LENGTH_MOWED,
        GRASS_LENGTH_CLEAR_0,
        GRASS_LENGTH_CLEAR_1,
        GRASS_LENGTH_CLEAR_2,
        GRASS_LENGTH_CLUMPS_0,
        GRASS_LENGTH_CLUMPS_1,
        GRASS_LENGTH_CLUMPS_2,
    };

    constexpr uint8_t kTileElementSurfaceOwnershipMask = 0xF0;
    constexpr uint8_t kTileElementSurfaceParkFenceMask = 0x0F;

    class TerrainEdgeObject;
    class TerrainSurfaceObject;

#pragma pack(push, 1)
    struct SurfaceElement : TileElementBase
    {
        static constexpr TileElementType kElementType = TileElementType::surface;

        // TER-024 layout. The base height is in the element base header.
        // Offsets from the base: shape low 8 bits, water, grass, ownership, quadrant 0 surface, edge, colour,
        // shape high 4 bits, then the surfaces of quadrants 1 to 3.
    private:
        uint8_t shapeLow;
        uint8_t waterHeight;
        uint8_t grassLength;
        uint8_t ownership;
        uint8_t surfaceStyle; // surface of quadrant 0
        uint8_t edgeObjectIndex;
        Drawing::Colour surfaceColour1;
        uint8_t shapeHigh;
        uint8_t quadrantSurface[3]; // surfaces of quadrants 1 to 3

    public:
        // Quarter-step terrain shape: 4 corner offsets of 0..4 Q above the base height (TER-010).
        Terrain::TerrainShape getTerrainShape() const;
        void setTerrainShape(const Terrain::TerrainShape& shape);

        bool isFlat() const;
        // Height of the highest corner above the base height, in Q.
        uint8_t getHighestCornerOffset() const;
        // Absolute z of the highest corner, rounded up to a multiple of roundTo.
        int32_t getHighestCornerZ(int32_t roundTo = 1) const;

        // Legacy slope bit flags, converted from and to the terrain shape. Remove once all users are migrated (TER-016).
        uint8_t getSlope() const;
        void setSlope(uint8_t newSlope);

        // Surface object of each of the 4 quadrants (TER-021), indexed by Terrain::Quadrant.
        ObjectEntryIndex getQuadrantSurfaceIndex(Terrain::Quadrant quadrant) const;
        void setQuadrantSurfaceIndex(Terrain::Quadrant quadrant, ObjectEntryIndex index);

        // Surface of quadrant 0; setting it paints the whole tile.
        ObjectEntryIndex getSurfaceObjectIndex() const;
        TerrainSurfaceObject* getSurfaceObject() const;
        void setSurfaceObjectIndex(ObjectEntryIndex newStyle);

        ObjectEntryIndex getEdgeObjectIndex() const;
        TerrainEdgeObject* getEdgeObject() const;
        void setEdgeObjectIndex(ObjectEntryIndex newIndex);

        bool canGrassGrow() const;
        uint8_t getGrassLength() const;
        void setGrassLength(uint8_t newLength);
        void setGrassLengthAndInvalidate(uint8_t newLength, const CoordsXY& coords);
        void updateGrassLength(const CoordsXY& coords);

        OwnershipFlags getOwnership() const;
        void setOwnership(OwnershipFlags newOwnership);
        bool hasOwnership(OwnershipFlag flag) const;

        int32_t getWaterHeight() const;
        void setWaterHeight(int32_t newWaterHeight);

        uint8_t getParkFences() const;
        void setParkFences(uint8_t newParkFences);

        bool hasTrackThatNeedsWater() const;
        void setHasTrackThatNeedsWater(bool on);

        Drawing::Colour getPrimarySurfaceColour() const;
        void setPrimarySurfaceColour(Drawing::Colour newColour);

        // Edge colours are gone with the quarter-step layout: edges are textured. Kept as stubs until the callers go.
        Drawing::Colour getPrimaryEdgeColour() const;
        void setPrimaryEdgeColour(Drawing::Colour newColour);
    };
    static_assert(sizeof(SurfaceElement) == kTileElementSize);
#pragma pack(pop)
} // namespace OpenRCT2
