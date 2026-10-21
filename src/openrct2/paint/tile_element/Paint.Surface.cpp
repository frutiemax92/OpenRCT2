/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Paint.Surface.h"

#include "../../Cheats.h"
#include "../../GameState.h"
#include "../../OpenRCT2.h"
#include "../../SpriteIds.h"
#include "../../config/Config.h"
#include "../../core/Numerics.hpp"
#include "../../entity/EntityRegistry.h"
#include "../../entity/PatrolArea.h"
#include "../../entity/Staff.h"
#include "../../interface/Viewport.h"
#include "../../drawing/ColourMap.h"
#include "../../drawing/TerrainSpriteStore.h"
#include "../../object/TerrainEdgeObject.h"
#include "../../object/TerrainSurfaceObject.h"
#include "../../paint/Paint.h"
#include "../../profiling/Profiling.h"
#include "../../world/Map.h"
#include "../../world/MapSelection.h"
#include "../../world/Stream.h"
#include "../../world/Parking.h"
#include "../../world/Street.h"
#include "../../world/tile_element/Slope.h"
#include "../../world/tile_element/SurfaceElement.h"
#include "../../world/tile_element/TileElement.h"
#include "../Boundbox.h"
#include "../Paint.SessionFlags.h"
#include "Paint.HoleMarkers.h"
#include "Paint.TileElement.h"
#include "Segment.h"

#include <cassert>
#include <cstring>
#include <iterator>

using namespace OpenRCT2;
using namespace OpenRCT2::Drawing;

// Needed to make the sign appear above footpaths.
static constexpr int16_t ForSaleSignZOffset = 3;

static constexpr uint8_t Byte97B444[] = {
    0, 2, 1, 3, 8, 10, 9, 11, 4, 6, 5, 7, 12, 14, 13, 15, 0, 0, 0, 0, 0, 0, 0, 17, 0, 0, 0, 16, 0, 18, 15, 0,
};

// rct2: 0x97B464, 0x97B474, 0x97B484, 0x97B494
static constexpr CoordsXY kNeighbouringTileCoordOffsets[4][kNumOrthogonalDirections] = {
    {
        { 32, 0 },
        { 0, 32 },
        { -32, 0 },
        { 0, -32 },
    },
    {
        { 0, 32 },
        { -32, 0 },
        { 0, -32 },
        { 32, 0 },
    },
    {
        { 0, -32 },
        { 32, 0 },
        { 0, 32 },
        { -32, 0 },
    },
    {
        { -32, 0 },
        { 0, -32 },
        { 32, 0 },
        { 0, 32 },
    },
};

// clang-format off

struct TunnelDescriptor
{
    uint8_t height;
    uint8_t boundBoxLength;
    int16_t boundBoxZOffset;
    int8_t lowerEdgeBoundingBoxZ;
    TunnelType lowClearanceAlternative;
    uint8_t imageOffset;
};
static constexpr TunnelDescriptor kTunnels[] = {
    { 2, 2, 0,   15, TunnelType::standardFlat,                    36 }, // TunnelType::StandardFlat
    { 3, 3, 0,   15, TunnelType::standardFlat,                    40 }, // TunnelType::StandardSlopeStart
    { 3, 5, -32,  4, TunnelType::standardFlat,                    44 }, // TunnelType::StandardSlopeEnd
    { 3, 3, 0,   15, TunnelType::invertedFlat,                    48 }, // TunnelType::InvertedFlat
    { 4, 4, 0,   15, TunnelType::invertedFlat,                    52 }, // TunnelType::InvertedSlopeStart
    { 4, 7, -48,  4, TunnelType::invertedFlat,                    56 }, // TunnelType::InvertedSlopeEnd
    { 2, 2, 0,   15, TunnelType::squareFlat,                      60 }, // TunnelType::SquareFlat
    { 3, 3, 0,   15, TunnelType::squareFlat,                      64 }, // TunnelType::SquareSlopeStart
    { 3, 5, -32,  4, TunnelType::squareFlat,                      68 }, // TunnelType::SquareSlopeEnd
    { 3, 3, 0,   15, TunnelType::squareFlat,                      72 }, // TunnelType::InvertedSquare
    { 2, 3, -16, 15, TunnelType::pathAndMiniGolf,                 76 }, // TunnelType::PathAndMiniGolf
    { 2, 3, -16, 15, TunnelType::path11,                          80 }, // TunnelType::Path11
    { 2, 3, -16,  4, TunnelType::standardFlatTo25Deg,             36 }, // TunnelType::StandardFlatTo25Deg
    { 3, 4, -16,  4, TunnelType::invertedFlatTo25Deg,             48 }, // TunnelType::InvertedFlatTo25Deg
    { 2, 3, -16,  4, TunnelType::squareFlatTo25Deg,               60 }, // TunnelType::SquareFlatTo25Deg
    { 3, 4, -16,  4, TunnelType::squareFlatTo25Deg,               72 }, // TunnelType::InvertedSquareFlatTo25Deg
    { 2, 2, 0,   15, TunnelType::doorClosed,                      84 }, // TunnelType::doorClosed
    { 2, 2, 0,   15, TunnelType::doorOpeningOutward,              88 }, // TunnelType::doorOpeningOutward
    { 2, 2, 0,   15, TunnelType::doorOpenOutward,                 92 }, // TunnelType::doorOpenOutward
    { 2, 2, 0,   15, TunnelType::doorOpeningInward,               96 }, // TunnelType::doorOpeningInward
    { 2, 2, 0,   15, TunnelType::doorOpenInward,                 100 }, // TunnelType::doorOpenInward
    { 2, 3, -16,  4, TunnelType::doorClosedFlatToDown25,          84 }, // TunnelType::doorClosedFlatToDown25
    { 2, 3, -16,  4, TunnelType::doorOpeningOutwardFlatToDown25,  88 }, // TunnelType::doorOpeningOutwardFlatToDown25
    { 2, 3, -16,  4, TunnelType::doorOpenOutwardFlatToDown25,     92 }, // TunnelType::doorOpenOutwardFlatToDown25
    { 2, 3, -16,  4, TunnelType::doorOpeningInwardFlatToDown25,   96 }, // TunnelType::doorOpeningInwardFlatToDown25
    { 2, 3, -16,  4, TunnelType::doorOpenInwardFlatToDown25,     100 }, // TunnelType::doorOpenInwardFlatToDown25
};
static_assert(std::size(kTunnels) == kTunnelTypeCount);

// clang-format on
// tunnel offset

struct TileDescriptor
{
    TileCoordsXY tile_coords;
    const TileElement* tile_element;
    const TerrainSurfaceObject* surfaceObject;
    uint8_t slope;
    SlopeRelativeCornerHeights corner_heights;
};

struct TileSurfaceBoundaryData
{
    int32_t bit_1;
    int32_t bit_8;
    int32_t bit_4;
    int32_t bit_2;
    uint32_t image[5];
    CoordsXY offset;
    BoundBoxXY Boundbox;
};

static constexpr TileSurfaceBoundaryData _tileSurfaceBoundaries[4] = {
    {
        // Bottom right
        1,
        8,
        4,
        2,
        {
            SPR_TERRAIN_BOUNDARY_FENCES_1,
            SPR_TERRAIN_BOUNDARY_FENCES_5,
            SPR_TERRAIN_BOUNDARY_FENCES_3,
            SPR_TERRAIN_BOUNDARY_FENCES_3,
            SPR_TERRAIN_BOUNDARY_FENCES_5,
        },
        { 1, 31 },
        { { 1, 31 }, { 30, 1 } },
    },
    {
        // Bottom left
        1,
        2,
        4,
        8,
        {
            SPR_TERRAIN_BOUNDARY_FENCES_2,
            SPR_TERRAIN_BOUNDARY_FENCES_6,
            SPR_TERRAIN_BOUNDARY_FENCES_4,
            SPR_TERRAIN_BOUNDARY_FENCES_4,
            SPR_TERRAIN_BOUNDARY_FENCES_6,
        },
        { 31, 0 },
        { { 31, 1 }, { 1, 30 } },
    },
    {
        // Top left
        4,
        2,
        8,
        1,
        {
            SPR_TERRAIN_BOUNDARY_FENCES_1,
            SPR_TERRAIN_BOUNDARY_FENCES_3,
            SPR_TERRAIN_BOUNDARY_FENCES_5,
            SPR_TERRAIN_BOUNDARY_FENCES_3,
            SPR_TERRAIN_BOUNDARY_FENCES_5,
        },
        { 1, 0 },
        { { 1, 1 }, { 30, 1 } },
    },
    {
        // Top right
        4,
        8,
        2,
        1,
        {
            SPR_TERRAIN_BOUNDARY_FENCES_2,
            SPR_TERRAIN_BOUNDARY_FENCES_4,
            SPR_TERRAIN_BOUNDARY_FENCES_6,
            SPR_TERRAIN_BOUNDARY_FENCES_4,
            SPR_TERRAIN_BOUNDARY_FENCES_6,
        },
        { 1, 1 },
        { { 1, 1 }, { 1, 30 } },
    },
};

static ImageId GetEdgeImageWithOffset(const TerrainEdgeObject* edgeObject, uint32_t offset, Colour selectedColour1)
{
    ImageId result;
    if (edgeObject != nullptr)
    {
        result = ImageId(edgeObject->BaseImageId + offset, selectedColour1);
    }
    return result;
}

static ImageId GetEdgeImage(const TerrainEdgeObject* edgeObject, uint8_t type, Colour selectedColour1)
{
    static constexpr uint32_t offsets[] = {
        0,
        20,
        10,
        30,
    };

    ImageId result;
    if (type < std::size(offsets))
    {
        result = GetEdgeImageWithOffset(edgeObject, offsets[type], selectedColour1);
    }
    return result;
}

static ImageId GetTunnelImage(const TerrainEdgeObject* edgeObject, TunnelType type, edge_t edge, Colour selectedColour1)
{
    bool hasDoors = false;
    if (edgeObject != nullptr)
    {
        hasDoors = edgeObject->flags.has(TerrainEdgeFlag::hasDoors) && !edgeObject->UsesFallbackImages();
    }

    if (!hasDoors && EnumValue(type) >= kRegularTunnelTypeCount)
        type = TunnelType::standardFlat;

    ImageId result = GetEdgeImageWithOffset(edgeObject, kTunnels[EnumValue(type)].imageOffset, selectedColour1)
                         .WithIndexOffset(edge == EDGE_BOTTOMRIGHT ? 2 : 0);

    return result;
}

static uint8_t ViewportSurfacePaintSetupGetRelativeSlope(const SurfaceElement& surfaceElement, int32_t rotation)
{
    const uint8_t slope = surfaceElement.getSlope();
    const uint8_t slopeHeight = slope & kTileSlopeDiagonalFlag;
    uint16_t slopeCorners = (slope & kTileSlopeRaisedCornersMask) << rotation;
    slopeCorners = ((slopeCorners >> 4) | slopeCorners) & 0x0F;
    return slopeHeight | slopeCorners;
}

/**
 *  rct2: 0x0065E890, 0x0065E946, 0x0065E9FC, 0x0065EAB2
 */
static bool TileIsInsideClipView(const TileDescriptor& tile)
{
    if (tile.tile_element == nullptr)
        return false;

    if (tile.tile_element->getBaseZ() > gClipHeight * kCoordsZStep)
        return false;

    auto coords = tile.tile_coords.toCoordsXY();
    if (coords.x < gClipSelectionA.x || coords.x > gClipSelectionB.x)
        return false;
    if (coords.y < gClipSelectionA.y || coords.y > gClipSelectionB.y)
        return false;

    return true;
}

static void ViewportSurfaceDrawTileSideBottom(
    PaintSession& session, enum edge_t edge, uint16_t height, const TerrainEdgeObject* edgeObject, const TileDescriptor& self,
    const TileDescriptor& neighbour, bool isWater, Colour selectedColour1)
{
    PROFILED_FUNCTION();

    // From big Z to tiny Z
    height /= kCoordsZPerTinyZ;
    int16_t cornerHeight1, neighbourCornerHeight1, cornerHeight2, neighbourCornerHeight2;

    CoordsXY offset = { 0, 0 };
    CoordsXY bounds = { 0, 0 };
    CoordsXY tunnelBounds = { 1, 1 };
    CoordsXY tunnelTopBoundBoxOffset = { 0, 0 };

    switch (edge)
    {
        case EDGE_BOTTOMLEFT:
            cornerHeight1 = self.corner_heights.left;
            cornerHeight2 = self.corner_heights.bottom;

            neighbourCornerHeight1 = neighbour.corner_heights.top;
            neighbourCornerHeight2 = neighbour.corner_heights.right;

            offset.x = 30;
            bounds.y = 30;
            tunnelBounds.x = 32;
            tunnelTopBoundBoxOffset.y = 31;
            break;

        case EDGE_BOTTOMRIGHT:
            cornerHeight1 = self.corner_heights.right;
            cornerHeight2 = self.corner_heights.bottom;

            neighbourCornerHeight1 = neighbour.corner_heights.top;
            neighbourCornerHeight2 = neighbour.corner_heights.left;

            offset.y = 30;
            bounds.x = 30;
            tunnelBounds.y = 32;
            tunnelTopBoundBoxOffset.x = 31;
            break;

        default:
            return;
    }

    bool neighbourIsClippedAway = session.ViewFlags.has(ViewportFlag::clipView) && !TileIsInsideClipView(neighbour);

    if (neighbour.tile_element == nullptr || neighbourIsClippedAway)
    {
        // The neighbour tile doesn't exist or isn't drawn - assume minimum height to draw full edges
        neighbourCornerHeight2 = kMinimumLandHeight / 2;
        neighbourCornerHeight1 = kMinimumLandHeight / 2;
    }

    if (isWater && neighbour.tile_element != nullptr)
    {
        auto waterHeight = neighbour.tile_element->asSurface()->getWaterHeight() / (kCoordsZStep * 2);
        if (waterHeight == height && !neighbourIsClippedAway)
        {
            // Don't draw the edge when the neighbour's water level is the same
            return;
        }

        const int16_t minimumNeighbourCornerHeight = std::min(neighbourCornerHeight1, neighbourCornerHeight2);
        neighbourCornerHeight1 = std::max(cornerHeight1, minimumNeighbourCornerHeight);
        neighbourCornerHeight2 = std::max(cornerHeight2, minimumNeighbourCornerHeight);

        cornerHeight1 = height;
        cornerHeight2 = height;
    }

    if (cornerHeight1 <= neighbourCornerHeight1 && cornerHeight2 <= neighbourCornerHeight2)
    {
        // The edge is not visible behind the neighbour's slope
        return;
    }

    auto baseImageId = GetEdgeImage(edgeObject, 0, selectedColour1);
    if (session.ViewFlags.has(ViewportFlag::undergroundInside))
    {
        baseImageId = GetEdgeImage(edgeObject, 1, selectedColour1);
    }

    if (edge == EDGE_BOTTOMRIGHT)
    {
        baseImageId = baseImageId.WithIndexOffset(5);
    }

    uint8_t curHeight = std::min(neighbourCornerHeight1, neighbourCornerHeight2);
    if (neighbourCornerHeight2 != neighbourCornerHeight1)
    {
        // If bottom part of edge isn't straight, add a filler
        uint32_t image_offset = 3;

        if (neighbourCornerHeight2 >= neighbourCornerHeight1)
        {
            image_offset = 4;
        }

        if (curHeight != cornerHeight1 && curHeight != cornerHeight2)
        {
            auto imageId = baseImageId.WithIndexOffset(image_offset);
            PaintAddImageAsParent(session, imageId, { offset, curHeight * kCoordsZPerTinyZ }, { bounds, 15 });
            curHeight++;
        }
    }

    neighbourCornerHeight1 = cornerHeight2;

    const auto lowestCornerHeight = std::min(cornerHeight1, cornerHeight2);

    const auto& tunnels = edge == EDGE_BOTTOMLEFT ? session.LeftTunnels : session.RightTunnels;
    for (auto& tunnel : tunnels)
    {
        if (curHeight > tunnel.height || tunnel.height >= lowestCornerHeight)
        {
            continue;
        }

        const auto tdOriginal = kTunnels[EnumValue(tunnel.type)];

        // Draw land edges up to the bottom of the tunnel
        while (curHeight < tunnel.height)
        {
            const auto boundBoxZ = curHeight == tunnel.height - 1 ? tdOriginal.lowerEdgeBoundingBoxZ : kCoordsZPerTinyZ - 1;
            PaintAddImageAsParent(session, baseImageId, { offset, curHeight * kCoordsZPerTinyZ }, { bounds, boundBoxZ });
            curHeight++;
        }

        auto tunnelType = tunnel.type;
        auto td = tdOriginal;
        uint8_t tunnelHeight = td.height;
        int16_t zOffset = curHeight;

        // When dealing with flat land but a sloped track, we fall back to the non-sloped variant.
        if ((zOffset + tunnelHeight) > neighbourCornerHeight1 || (zOffset + tunnelHeight) > cornerHeight1)
        {
            tunnelType = td.lowClearanceAlternative;
            td = kTunnels[EnumValue(tunnelType)];
        }

        zOffset *= 16;

        const int8_t boundBoxLengthBase = (tdOriginal.boundBoxLength + (td.height - tdOriginal.height)) * 16;

        int16_t boundBoxOffsetZ = zOffset + tdOriginal.boundBoxZOffset;
        int8_t boundBoxLength = boundBoxLengthBase;
        if (boundBoxOffsetZ < 16)
        {
            boundBoxOffsetZ += 16;
            boundBoxLength -= 16;
        }

        auto imageId = GetTunnelImage(edgeObject, tunnelType, edge, selectedColour1);
        PaintAddImageAsParent(
            session, imageId, { offset, zOffset }, { { 0, 0, boundBoxOffsetZ }, { tunnelBounds, boundBoxLength - 1 } });

        boundBoxOffsetZ = curHeight * kCoordsZPerTinyZ;
        boundBoxLength = boundBoxLengthBase;
        boundBoxOffsetZ += tdOriginal.boundBoxZOffset;
        if (boundBoxOffsetZ == 0)
        {
            boundBoxOffsetZ += 16;
            boundBoxLength -= 16;
        }

        imageId = GetTunnelImage(edgeObject, tunnelType, edge, selectedColour1).WithIndexOffset(1);
        PaintAddImageAsParent(
            session, imageId, { offset, curHeight * kCoordsZPerTinyZ },
            { { tunnelTopBoundBoxOffset, boundBoxOffsetZ }, { tunnelBounds, boundBoxLength - 1 } });

        curHeight += td.height;
    }

    // Draw land edges up to the lowest corner
    while (curHeight < lowestCornerHeight)
    {
        PaintAddImageAsParent(session, baseImageId, { offset, curHeight * kCoordsZPerTinyZ }, { bounds, kCoordsZPerTinyZ - 1 });
        curHeight++;
    }

    // Draw filler edges if the top edge is not straight
    if (curHeight < cornerHeight1 || curHeight < cornerHeight2)
    {
        const uint32_t imageOffset = curHeight >= cornerHeight1 ? 2 : 1;
        const auto imageId = baseImageId.WithIndexOffset(imageOffset);
        PaintAddImageAsParent(session, imageId, { offset, curHeight * kCoordsZPerTinyZ }, { bounds, kCoordsZPerTinyZ - 1 });
    }
}

static void ViewportSurfaceDrawTileSideTop(
    PaintSession& session, enum edge_t edge, uint16_t height, const TerrainEdgeObject* edgeObject, const TileDescriptor& self,
    const TileDescriptor& neighbour, bool isWater, Drawing::Colour selectedColour1)
{
    PROFILED_FUNCTION();

    // From big Z to tiny Z
    height /= kCoordsZPerTinyZ;

    int16_t cornerHeight1, neighbourCornerHeight1, cornerHeight2, neighbourCornerHeight2;

    CoordsXY offset = { 0, 0 };
    CoordsXY bounds = { 0, 0 };

    switch (edge)
    {
        case EDGE_TOPLEFT:
            cornerHeight1 = self.corner_heights.top;
            cornerHeight2 = self.corner_heights.left;

            neighbourCornerHeight1 = neighbour.corner_heights.right;
            neighbourCornerHeight2 = neighbour.corner_heights.bottom;

            offset.y = -2;
            bounds.x = 30;
            break;

        case EDGE_TOPRIGHT:
            cornerHeight1 = self.corner_heights.top;
            cornerHeight2 = self.corner_heights.right;

            neighbourCornerHeight1 = neighbour.corner_heights.left;
            neighbourCornerHeight2 = neighbour.corner_heights.bottom;

            offset.x = -2;
            bounds.y = 30;
            break;

        default:
            return;
    }

    // save ecx
    if (neighbour.tile_element == nullptr)
    {
        neighbourCornerHeight1 = 1;
        neighbourCornerHeight2 = 1;
    }
    else
    {
        if (isWater)
        {
            auto waterHeight = neighbour.tile_element->asSurface()->getWaterHeight() / (kCoordsZStep * 2);
            if (height == waterHeight)
            {
                return;
            }

            neighbourCornerHeight1 = std::max(cornerHeight1, neighbourCornerHeight1);
            neighbourCornerHeight2 = std::max(cornerHeight2, neighbourCornerHeight2);

            cornerHeight1 = height;
            cornerHeight2 = height;
        }
    }

    if (cornerHeight1 <= neighbourCornerHeight1 && cornerHeight2 <= neighbourCornerHeight2)
    {
        return;
    }

    ImageId baseImageId;
    if (isWater)
    {
        baseImageId = GetEdgeImage(edgeObject, 2, selectedColour1); // var_08
        if (session.ViewFlags.has(ViewportFlag::undergroundInside))
        {
            baseImageId = GetEdgeImage(edgeObject, 1, selectedColour1); // var_04
        }
        baseImageId = baseImageId.WithIndexOffset(edge == EDGE_TOPLEFT ? 5 : 0);

        offset.x = 0;
        offset.y = 0;
    }
    else
    {
        if (!(session.ViewFlags.has(ViewportFlag::undergroundInside)))
        {
            const uint8_t incline = (cornerHeight2 - cornerHeight1) + 1;
            const auto imageId = GetEdgeImage(edgeObject, 3, selectedColour1)
                                     .WithIndexOffset((edge == EDGE_TOPLEFT ? 3 : 0) + incline);
            const int16_t y = (height - cornerHeight1) * kCoordsZPerTinyZ;
            PaintAttachToPreviousPS(session, imageId, 0, y);
            return;
        }
        baseImageId = GetEdgeImage(edgeObject, 1, selectedColour1).WithIndexOffset(edge == EDGE_TOPLEFT ? 5 : 0);
    }

    uint8_t cur_height = std::min(neighbourCornerHeight2, neighbourCornerHeight1);
    if (neighbourCornerHeight2 != neighbourCornerHeight1)
    {
        // neighbour tile corners aren't level
        uint32_t image_offset = 3;
        if (neighbourCornerHeight2 > neighbourCornerHeight1)
        {
            image_offset = 4;
        }

        if (cur_height != cornerHeight1 && cur_height != cornerHeight2)
        {
            auto imageId = baseImageId.WithIndexOffset(image_offset);
            PaintAddImageAsParent(
                session, imageId, { offset.x, offset.y, cur_height * kCoordsZPerTinyZ }, { bounds.x, bounds.y, 15 });
            cur_height++;
        }
    }

    neighbourCornerHeight1 = cornerHeight2;

    while (cur_height < cornerHeight1 && cur_height < neighbourCornerHeight1)
    {
        PaintAddImageAsParent(session, baseImageId, { offset, cur_height * kCoordsZPerTinyZ }, { bounds, 15 });
        cur_height++;
    }

    uint32_t image_offset = 1;
    if (cur_height >= cornerHeight1)
    {
        image_offset = 2;

        if (cur_height >= neighbourCornerHeight1)
        {
            return;
        }
    }

    auto imageId = baseImageId.WithIndexOffset(image_offset);
    PaintAddImageAsParent(session, imageId, { offset, cur_height * kCoordsZPerTinyZ }, { bounds, 15 });
}

std::optional<OpenRCT2::Drawing::Colour> GetPatrolAreaTileColour(const CoordsXY& pos)
{
    bool selected = gMapSelectFlags.has(MapSelectFlag::enable)
        && (gMapSelectType == MapSelectType::full || gMapSelectType == MapSelectType::fullTerrainAndWater)
        && pos.x >= gMapSelectPositionA.x && pos.x <= gMapSelectPositionB.x && pos.y >= gMapSelectPositionA.y
        && pos.y <= gMapSelectPositionB.y;

    auto patrolAreaToRender = GetPatrolAreaToRender();
    if (const auto* staffType = std::get_if<StaffType>(&patrolAreaToRender))
    {
        if (IsPatrolAreaSetForStaffType(*staffType, pos))
        {
            return selected ? OpenRCT2::Drawing::Colour::white : OpenRCT2::Drawing::Colour::grey;
        }
    }
    else
    {
        auto& staffId = std::get<EntityId>(patrolAreaToRender);
        auto* staff = getGameState().entities.getEntity<Staff>(staffId);
        if (staff != nullptr)
        {
            if (staff->isPatrolAreaSet(pos))
            {
                return selected ? OpenRCT2::Drawing::Colour::icyBlue : OpenRCT2::Drawing::Colour::lightBlue;
            }
            else if (IsPatrolAreaSetForStaffType(staff->assignedStaffType, pos))
            {
                return selected ? OpenRCT2::Drawing::Colour::white : OpenRCT2::Drawing::Colour::grey;
            }
        }
    }
    return {};
}

// Overlays on the terrain (TER-034): drawn procedurally on the exact surface, and also flat on the water when the tile
// is under water.
static uint8_t OverlayColour(Drawing::Colour colour, uint8_t shade = 8)
{
    return static_cast<uint8_t>(Drawing::getColourMapShade(colour, shade));
}

static void PaintTerrainOverlay(
    PaintSession& session, const SurfaceElement& tileElement, uint16_t height, const Drawing::TerrainOverlay& overlay,
    bool attachToSurface)
{
    auto& store = Drawing::TerrainSpriteStore::Instance();
    const int32_t rotation = session.CurrentRotation;

    const auto image = store.GetOverlayImage(tileElement.getTerrainShape(), overlay, rotation);
    if (image != kImageIndexUndefined)
    {
        if (attachToSurface)
        {
            PaintAttachToPreviousPS(session, ImageId(image), 0, 0);
        }
        else
        {
            PaintStruct* backup = session.LastPS;
            PaintAddImageAsParent(session, ImageId(image), { 0, 0, height }, { 32, 32, 1 });
            session.LastPS = backup;
        }
    }

    const int32_t waterHeight = tileElement.getWaterHeight();
    if (waterHeight > height)
    {
        const auto waterImage = store.GetOverlayImage({ 0, 0, 0, 0 }, overlay, rotation);
        if (waterImage != kImageIndexUndefined)
        {
            PaintStruct* backup = session.LastPS;
            PaintAddImageAsParent(session, ImageId(waterImage), { 0, 0, waterHeight }, { 32, 32, 1 });
            session.LastPS = backup;
        }
    }
}

static void PaintPatrolArea(PaintSession& session, const SurfaceElement& element, int32_t height)
{
    auto colour = GetPatrolAreaTileColour(session.MapPosition);
    if (colour)
    {
        PaintTerrainOverlay(
            session, element, height, { Drawing::TerrainOverlayKind::fill, 0, OverlayColour(*colour) }, false);
    }
}

static void PaintSurfaceLandOwnership(PaintSession& session, const SurfaceElement& tileElement, uint16_t height)
{
    if (tileElement.hasOwnership(OwnershipFlag::landOwned))
    {
        // Outline on top of the surface (and on the water when under water)
        PaintTerrainOverlay(
            session, tileElement, height,
            { Drawing::TerrainOverlayKind::outline, 0, OverlayColour(Drawing::Colour::brightYellow) }, true);
    }
    else if (tileElement.hasOwnership(OwnershipFlag::landForSale))
    {
        const int32_t baseZ = std::max<int32_t>(height, tileElement.getWaterHeight());
        const auto height2 = Terrain::GetSurfaceZAt(height, tileElement.getTerrainShape(), kCoordsXYStep / 2, kCoordsXYStep / 2);
        PaintStruct* backup = session.LastPS;
        PaintAddImageAsParent(
            session, ImageId(SPR_LAND_OWNERSHIP_AVAILABLE), { 16, 16, std::max<int32_t>(baseZ, height2) + ForSaleSignZOffset },
            { 1, 1, 0 });
        session.LastPS = backup;
    }
}

static void PaintSurfaceConstructionRights(PaintSession& session, const SurfaceElement& tileElement, uint16_t height)
{
    if (tileElement.hasOwnership(OwnershipFlag::constructionRightsOwned))
    {
        PaintTerrainOverlay(
            session, tileElement, height,
            { Drawing::TerrainOverlayKind::dotted, 0, OverlayColour(Drawing::Colour::brightYellow) }, true);
    }
    else if (tileElement.hasOwnership(OwnershipFlag::constructionRightsForSale))
    {
        const int32_t baseZ = std::max<int32_t>(height, tileElement.getWaterHeight());
        const auto height2 = Terrain::GetSurfaceZAt(height, tileElement.getTerrainShape(), kCoordsXYStep / 2, kCoordsXYStep / 2);
        PaintStruct* backup = session.LastPS;
        PaintAddImageAsParent(
            session, ImageId(SPR_LAND_CONSTRUCTION_RIGHTS_AVAILABLE),
            { 16, 16, std::max<int32_t>(baseZ, height2) + ForSaleSignZOffset }, { 1, 1, 0 });
        session.LastPS = backup;
    }
}

/**
 * Procedural edges (TER-060): vertical faces on the edges facing the viewer, hanging from this tile down to the
 * lower neighbour. For water, the faces hang from the water level to the lower water or land next to it. Returns false
 * when the edge object has no texture, so the caller falls back to the legacy edges.
 */
static bool PaintProceduralEdges(PaintSession& session, const SurfaceElement& tileElement, uint16_t height, bool water)
{
    using Terrain::Quadrant;
    auto& store = Drawing::TerrainSpriteStore::Instance();
    const auto edgeIndex = tileElement.getEdgeObjectIndex();
    const auto* edgeObject = TerrainEdgeObject::GetById(edgeIndex);
    if (edgeObject == nullptr || edgeObject->getTexture() == nullptr)
        return false;

    const int32_t rotation = session.CurrentRotation;
    const auto shape = tileElement.getTerrainShape();
    const int32_t baseZ = tileElement.getBaseZ();
    constexpr int32_t kQ = Terrain::kQuarterStepZ;

    struct EdgeDef
    {
        Quadrant quadrant;
        CoordsXY neighbourOffset;
        int32_t normalX, normalY;
        uint8_t selfA, selfB;           // offsets of this tile's corners at the edge ends
        uint8_t neighbourA, neighbourB; // offsets of the neighbour's corners at the same ends
    };
    // Corners are N, E, S, W = indices 0..3 into the neighbour shape below.
    const EdgeDef defs[4] = {
        { Quadrant::northEast, { kCoordsXYStep, 0 }, 1, 0, shape.east, shape.north, 2, 3 },
        { Quadrant::southEast, { 0, -kCoordsXYStep }, 0, -1, shape.south, shape.east, 3, 0 },
        { Quadrant::southWest, { -kCoordsXYStep, 0 }, -1, 0, shape.south, shape.west, 1, 0 },
        { Quadrant::northWest, { 0, kCoordsXYStep }, 0, 1, shape.west, shape.north, 2, 1 },
    };

    for (const auto& def : defs)
    {
        if (!Drawing::IsEdgeFacingViewer(def.quadrant, rotation))
            continue;

        const auto neighbourPos = session.MapPosition + def.neighbourOffset;
        const SurfaceElement* neighbour = MapIsLocationValid(neighbourPos) ? MapGetSurfaceElementAt(neighbourPos) : nullptr;

        const int32_t waterZ = tileElement.getWaterHeight();
        int32_t topA = def.selfA * kQ, topB = def.selfB * kQ;
        int32_t bottomA, bottomB;
        if (water)
        {
            // From the water level down to what is next to it: the lower water level, or the land, never below the
            // land of this tile.
            topA = topB = waterZ - baseZ;
            if (neighbour != nullptr && neighbour->getWaterHeight() == waterZ)
                continue;
            const auto ns = neighbour != nullptr ? neighbour->getTerrainShape() : Terrain::TerrainShape{};
            const uint8_t offsets[4] = { ns.north, ns.east, ns.south, ns.west };
            const int32_t nBase = neighbour != nullptr ? neighbour->getBaseZ() : 0;
            const int32_t nWater = neighbour != nullptr ? neighbour->getWaterHeight() : 0;
            auto level = [&](uint8_t corner) {
                if (neighbour == nullptr)
                    return int32_t{ 0 };
                return nWater > 0 ? nWater - baseZ : nBase + offsets[corner] * kQ - baseZ;
            };
            bottomA = std::max<int32_t>(def.selfA * kQ, level(def.neighbourA));
            bottomB = std::max<int32_t>(def.selfB * kQ, level(def.neighbourB));
        }
        else if (neighbour == nullptr)
        {
            // Nothing beyond: hang down to the lowest land.
            bottomA = bottomB = kMinimumLandHeight * kQ - baseZ;
        }
        else
        {
            const auto ns = neighbour->getTerrainShape();
            const uint8_t offsets[4] = { ns.north, ns.east, ns.south, ns.west };
            const int32_t nBase = neighbour->getBaseZ();
            bottomA = nBase + offsets[def.neighbourA] * kQ - baseZ;
            bottomB = nBase + offsets[def.neighbourB] * kQ - baseZ;
        }
        bottomA = std::min(bottomA, topA);
        bottomB = std::min(bottomB, topB);
        if (bottomA == topA && bottomB == topB)
            continue;

        const auto image = store.GetEdgeImage(
            edgeIndex, { def.quadrant, topA, topB, bottomA, bottomB, baseZ }, rotation);
        if (image == kImageIndexUndefined)
            continue;

        int32_t nx = def.normalX, ny = def.normalY;
        {
            // Rotated frame of the paint code: the bottom-left edge has the normal (1, 0), the bottom-right edge (0, 1).
            const int32_t ox = nx, oy = ny;
            switch (rotation & 3)
            {
                case 1:
                    nx = oy;
                    ny = -ox;
                    break;
                case 2:
                    nx = -ox;
                    ny = -oy;
                    break;
                case 3:
                    nx = -oy;
                    ny = ox;
                    break;
                default:
                    break;
            }
        }
        const int32_t zLow = baseZ + std::min(bottomA, bottomB);
        const int32_t span = std::max(1, std::max(topA, topB) - std::min(bottomA, bottomB));
        const CoordsXYZ boxOffset = nx == 1 ? CoordsXYZ{ 30, 0, zLow } : CoordsXYZ{ 0, 30, zLow };
        const CoordsXYZ boxSize = nx == 1 ? CoordsXYZ{ 0, 30, span } : CoordsXYZ{ 30, 0, span };
        auto imageId = ImageId(image);
        if (session.ViewFlags.hasAny(ViewportFlag::undergroundInside, ViewportFlag::hideBase))
            imageId = imageId.WithTransparency(FilterPaletteID::paletteDarken1);
        PaintAddImageAsParent(session, imageId, { 0, 0, baseZ }, { boxOffset, boxSize });
    }
    return true;
}

/**
 *  rct2: 0x0066062C
 */
void PaintSurface(PaintSession& session, uint8_t direction, uint16_t height, const SurfaceElement& tileElement)
{
    PROFILED_FUNCTION();

    session.InteractionType = ViewportInteractionItem::terrain;
    session.Flags |= PaintSessionFlags::PassedSurface;
    session.Surface = &tileElement;

    const auto zoomLevel = session.rt.zoom_level;
    const uint8_t rotation = session.CurrentRotation;
    const uint8_t surfaceShape = ViewportSurfacePaintSetupGetRelativeSlope(tileElement, rotation);
    const CoordsXY& base = session.MapPosition;
    const auto cornerHeights = GetSlopeRelativeCornerHeights(surfaceShape);
    const TileElement* elementPtr = &reinterpret_cast<const TileElement&>(tileElement);

    const auto* surfaceObject = tileElement.getSurfaceObject();
    const auto* edgeObject = tileElement.getEdgeObject();

    const auto selfDescriptor = TileDescriptor{
        TileCoordsXY(base),
        elementPtr,
        surfaceObject,
        surfaceShape,
        {
            static_cast<uint8_t>(height / 16 + cornerHeights.top),
            static_cast<uint8_t>(height / 16 + cornerHeights.right),
            static_cast<uint8_t>(height / 16 + cornerHeights.bottom),
            static_cast<uint8_t>(height / 16 + cornerHeights.left),
        },
    };

    TileDescriptor tileDescriptors[4];

    for (std::size_t i = 0; i < std::size(kNeighbouringTileCoordOffsets); i++)
    {
        const CoordsXY& offset = kNeighbouringTileCoordOffsets[i][rotation];
        const CoordsXY position = base + offset;

        TileDescriptor& descriptor = tileDescriptors[i];

        descriptor.tile_element = nullptr;
        if (!MapIsLocationValid(position))
        {
            continue;
        }

        auto surfaceElement = MapGetSurfaceElementAt(position);
        if (surfaceElement == nullptr)
        {
            continue;
        }

        const uint32_t surfaceSlope = ViewportSurfacePaintSetupGetRelativeSlope(*surfaceElement, rotation);
        const uint8_t baseHeight = surfaceElement->getBaseZ() / 16;
        const auto ch = GetSlopeRelativeCornerHeights(surfaceSlope);

        descriptor.tile_coords = TileCoordsXY{ position };
        descriptor.tile_element = reinterpret_cast<TileElement*>(surfaceElement);
        descriptor.surfaceObject = surfaceElement->getSurfaceObject();
        descriptor.slope = surfaceSlope;
        descriptor.corner_heights.top = baseHeight + ch.top;
        descriptor.corner_heights.right = baseHeight + ch.right;
        descriptor.corner_heights.bottom = baseHeight + ch.bottom;
        descriptor.corner_heights.left = baseHeight + ch.left;
    }

    if (PaintShouldShowHeightMarkers(session, ViewportFlag::landHeights))
    {
        const int16_t x = session.MapPosition.x;
        const int16_t y = session.MapPosition.y;

        int32_t surfaceHeight = TileElementHeight({ x + 16, y + 16 });
        int32_t dx = surfaceHeight + 3;

        int32_t image_id = (SPR_HEIGHT_MARKER_BASE + dx / 16);
        image_id += GetHeightMarkerOffset();
        image_id -= kMapBaseZ;

        PaintAddImageAsParent(
            session, ImageId(image_id, OpenRCT2::Drawing::Colour::oliveGreen), { 16, 16, surfaceHeight }, { 1, 1, 0 });
    }

    if (session.VerticalTunnelHeight * kCoordsZPerTinyZ == height)
    {
        // Vertical tunnels
        PaintAddImageAsParent(session, ImageId(1575), { 0, 0, height }, { { -2, 1, height - 40 }, { 1, 30, 39 } });
        PaintAddImageAsParent(session, ImageId(1576), { 0, 0, height }, { { 1, 31, height }, { 30, 1, 0 } });
        PaintAddImageAsParent(session, ImageId(1577), { 0, 0, height }, { { 31, 1, height }, { 1, 30, 0 } });
        PaintAddImageAsParent(session, ImageId(1578), { 0, 0, height }, { { 1, -2, height - 40 }, { 30, 1, 39 } });
    }
    else
    {
        const bool showGridlines = session.ViewFlags.has(ViewportFlag::gridlines);

        assert(surfaceShape < std::size(Byte97B444));
        const uint8_t image_offset = Byte97B444[surfaceShape];

        ImageId imageId;
        if (surfaceObject != nullptr)
        {
            uint8_t grassLength = TerrainSurfaceObject::kNoValue;
            if (zoomLevel <= ZoomLevel{ 0 })
            {
                if (!session.ViewFlags.hasAny(ViewportFlag::hideBase, ViewportFlag::undergroundInside))
                {
                    grassLength = tileElement.getGrassLength() & 0x7;
                }
            }
            imageId = surfaceObject->GetImageId(
                session.MapPosition, grassLength, rotation, image_offset, showGridlines, false,
                tileElement.getPrimarySurfaceColour());
        }
        if (session.ViewFlags.hasAny(ViewportFlag::undergroundInside, ViewportFlag::hideBase))
        {
            imageId = imageId.WithTransparency(FilterPaletteID::paletteDarken1);
        }

        if (session.SelectedElement == elementPtr)
        {
            imageId = imageId.WithRemap(FilterPaletteID::paletteGhost);
        }

        // Procedural terrain (TER-031). Sprites not generated yet fall back to the slope images.
        {
            const auto proceduralImage = Drawing::TerrainSpriteStore::Instance().GetImage(tileElement, session.MapPosition, rotation);
            if (proceduralImage != kImageIndexUndefined)
            {
                auto procedural = ImageId(proceduralImage);
                if (session.ViewFlags.hasAny(ViewportFlag::undergroundInside, ViewportFlag::hideBase))
                    procedural = procedural.WithTransparency(FilterPaletteID::paletteDarken1);
                if (session.SelectedElement == elementPtr)
                    procedural = procedural.WithRemap(FilterPaletteID::paletteGhost);
                imageId = procedural;
            }
        }

        PaintAddImageAsParent(session, imageId, { 0, 0, height }, { 32, 32, -1 });

        // Stream surface projected on terrain (STR-005)
        const auto* stream = Stream::FindAt(session.MapPosition);
        if (stream != nullptr)
        {
            const auto streamImage = Drawing::TerrainSpriteStore::Instance().GetStreamImage(tileElement, *stream, rotation);
            if (streamImage != kImageIndexUndefined)
            {
                PaintAttachToPreviousPS(session, ImageId(streamImage), 0, 0);
            }
        }

        // Street surface projected on terrain (STE-003)
        const auto* street = Street::FindAt(session.MapPosition);
        if (street != nullptr)
        {
            const auto streetImage = Drawing::TerrainSpriteStore::Instance().GetStreetImage(tileElement, *street, rotation);
            if (streetImage != kImageIndexUndefined)
            {
                PaintAttachToPreviousPS(session, ImageId(streetImage), 0, 0);
            }
        }

        // Parking spaces likewise (STE-010)
        const auto* parking = Parking::FindAt(session.MapPosition);
        if (parking != nullptr)
        {
            const auto parkingImage = Drawing::TerrainSpriteStore::Instance().GetParkingImage(tileElement, *parking, rotation);
            if (parkingImage != kImageIndexUndefined)
            {
                PaintAttachToPreviousPS(session, ImageId(parkingImage), 0, 0);
            }
        }

        if (showGridlines)
        {
            const auto grid = Drawing::TerrainSpriteStore::Instance().GetOverlayImage(
                tileElement.getTerrainShape(),
                { Drawing::TerrainOverlayKind::outline, 0, OverlayColour(Drawing::Colour::grey, 4) }, rotation);
            if (grid != kImageIndexUndefined)
                PaintAttachToPreviousPS(session, ImageId(grid), 0, 0);
        }
    }

    PaintPatrolArea(session, tileElement, height);

    auto& gameState = getGameState();
    // Draw Peep Spawns
    if ((gLegacyScene == LegacyScene::scenarioEditor || gameState.cheats.sandboxMode)
        && session.ViewFlags.has(ViewportFlag::landOwnership))
    {
        const CoordsXY& pos = session.MapPosition;
        for (auto& spawn : gameState.peepSpawns)
        {
            if ((spawn.x & 0xFFE0) == pos.x && (spawn.y & 0xFFE0) == pos.y)
            {
                {
                    const auto spawnOutline = Drawing::TerrainSpriteStore::Instance().GetOverlayImage(
                        { 0, 0, 0, 0 }, { Drawing::TerrainOverlayKind::outline, 0, OverlayColour(Drawing::Colour::white) },
                        rotation);
                    if (spawnOutline != kImageIndexUndefined)
                        PaintAddImageAsParent(session, ImageId(spawnOutline), { 0, 0, spawn.z }, { 32, 32, 16 });
                }

                const int32_t offset = (DirectionReverse(spawn.direction) + rotation) & 3;
                const auto image_id = ImageId(PEEP_SPAWN_ARROW_0 + offset, OpenRCT2::Drawing::Colour::lightBlue);
                PaintAddImageAsParent(session, image_id, { 0, 0, spawn.z }, { 32, 32, 19 });
            }
        }
    }

    if (session.ViewFlags.has(ViewportFlag::landOwnership))
    {
        PaintSurfaceLandOwnership(session, tileElement, height);
    }

    if (session.ViewFlags.has(ViewportFlag::constructionRights) && !(tileElement.hasOwnership(OwnershipFlag::landOwned)))
    {
        PaintSurfaceConstructionRights(session, tileElement, height);
    }

    // ebx[0] = esi;
    // ebp[4] = ebp;
    // ebp[8] = ebx

    if (gMapSelectFlags.has(MapSelectFlag::enable))
    {
        const CoordsXY& pos = session.MapPosition;
        if (pos.x >= gMapSelectPositionA.x && pos.x <= gMapSelectPositionB.x && pos.y >= gMapSelectPositionA.y
            && pos.y <= gMapSelectPositionB.y)
        {
            using Drawing::TerrainOverlay;
            using Drawing::TerrainOverlayKind;
            const auto mapSelectionType = gMapSelectType;
            if (mapSelectionType >= MapSelectType::edge0)
            {
                // A band along one edge. edge0 is the edge between the south and west corners (x = 0), then clockwise
                // through the quadrants: south-west, north-west, north-east, south-east.
                const auto edge = static_cast<uint8_t>((EnumValue(mapSelectionType) - EnumValue(MapSelectType::edge0) + 2) & 3);
                PaintTerrainOverlay(
                    session, tileElement, height,
                    { TerrainOverlayKind::edge, edge, OverlayColour(Drawing::Colour::brightYellow, 9) }, true);
            }
            else if (mapSelectionType >= MapSelectType::quarter0)
            {
                // Selection split into four quarter segments
                const auto quarter = static_cast<uint8_t>(EnumValue(mapSelectionType) - EnumValue(MapSelectType::quarter0));
                PaintTerrainOverlay(
                    session, tileElement, height,
                    { TerrainOverlayKind::quarter, quarter, OverlayColour(Drawing::Colour::lightOrange, 9) }, true);
            }
            else if (mapSelectionType <= MapSelectType::full)
            {
                // A corner of the tile, or all of it
                if (mapSelectionType == MapSelectType::full)
                    PaintTerrainOverlay(
                        session, tileElement, height,
                        { TerrainOverlayKind::fill, 0, OverlayColour(Drawing::Colour::brightYellow, 9) }, true);
                else
                    PaintTerrainOverlay(
                        session, tileElement, height,
                        { TerrainOverlayKind::corner, static_cast<uint8_t>(EnumValue(mapSelectionType)),
                          OverlayColour(Drawing::Colour::brightYellow, 9) },
                        true);
            }
            else if (mapSelectionType == MapSelectType::fullTerrainAndWater)
            {
                PaintTerrainOverlay(
                    session, tileElement, height, { TerrainOverlayKind::fill, 0, OverlayColour(Drawing::Colour::lightOrange) },
                    true);
            }
            else
            {
                // The water tool draws on the water, rather than on the surface under water.
                const int32_t waterHeight = tileElement.getWaterHeight();
                const bool onWater = waterHeight > height;
                const auto image = Drawing::TerrainSpriteStore::Instance().GetOverlayImage(
                    onWater ? Terrain::TerrainShape{ 0, 0, 0, 0 } : tileElement.getTerrainShape(),
                    { TerrainOverlayKind::fill, 0, OverlayColour(Drawing::Colour::icyBlue) }, rotation);
                if (image != kImageIndexUndefined)
                {
                    PaintStruct* backup = session.LastPS;
                    PaintAddImageAsParent(session, ImageId(image), { 0, 0, onWater ? waterHeight : height }, { 32, 32, 1 });
                    session.LastPS = backup;
                }
            }
        }
    }

    if (gMapSelectFlags.has(MapSelectFlag::enableConstruct))
    {
        const CoordsXY& pos = session.MapPosition;

        for (const auto& tile : MapSelection::getSelectedTiles())
        {
            if (tile.x != pos.x || tile.y != pos.y)
            {
                continue;
            }

            const auto marker = gMapSelectFlags.has(MapSelectFlag::green) ? Drawing::Colour::brightGreen
                                                                          : Drawing::Colour::lightOrange;
            PaintTerrainOverlay(
                session, tileElement, height, { Drawing::TerrainOverlayKind::fill, 0, OverlayColour(marker) }, true);
            break;
        }
    }

    if (session.ViewFlags.has(ViewportFlag::undergroundInside) && !session.ViewFlags.has(ViewportFlag::hideBase))
    {
        const uint8_t image_offset = Byte97B444[surfaceShape];
        ImageId imageId;
        if (surfaceObject != nullptr)
            imageId = surfaceObject->GetImageId(
                session.MapPosition, 1, rotation, image_offset, false, true, tileElement.getPrimarySurfaceColour());
        PaintAttachToPreviousPS(session, imageId, 0, 0);
    }

    if (!session.ViewFlags.has(ViewportFlag::hideVertical))
    {
        if (!PaintProceduralEdges(session, tileElement, height, false))
        {
            auto edgeColour1 = tileElement.getPrimaryEdgeColour();
            ViewportSurfaceDrawTileSideTop(
                session, EDGE_TOPLEFT, height, edgeObject, selfDescriptor, tileDescriptors[2], false, edgeColour1);
            ViewportSurfaceDrawTileSideTop(
                session, EDGE_TOPRIGHT, height, edgeObject, selfDescriptor, tileDescriptors[3], false, edgeColour1);
            ViewportSurfaceDrawTileSideBottom(
                session, EDGE_BOTTOMLEFT, height, edgeObject, selfDescriptor, tileDescriptors[0], false, edgeColour1);
            ViewportSurfaceDrawTileSideBottom(
                session, EDGE_BOTTOMRIGHT, height, edgeObject, selfDescriptor, tileDescriptors[1], false, edgeColour1);
        }
    }

    const uint16_t waterHeight = tileElement.getWaterHeight();
    const bool waterGetsClipped = session.ViewFlags.has(ViewportFlag::clipView) && (waterHeight > gClipHeight * kCoordsZStep);
    const bool waterIsTransparent = Config::Get().general.transparentWater
        || session.ViewFlags.has(ViewportFlag::undergroundInside);

    if (waterHeight > 0 && !waterGetsClipped)
    {
        // Loc6615A9: (water height)
        session.InteractionType = ViewportInteractionItem::water;

        session.WaterHeight = waterHeight;

        // Procedural water (WAT-001): the water plane where the terrain is under it. The mask tints what is below; the
        // overlay adds the waves, fully opaque unless the water is transparent.
        auto& store = Drawing::TerrainSpriteStore::Instance();
        const auto maskImage = store.GetWaterImage(tileElement, Drawing::WaterPattern::mask, rotation);
        if (maskImage != kImageIndexUndefined)
        {
            PaintAddImageAsParent(
                session, ImageId(maskImage, FilterPaletteID::paletteWater).WithBlended(true), { 0, 0, waterHeight },
                { 32, 32, -1 });
            const auto overlayImage = store.GetWaterImage(
                tileElement, waterIsTransparent ? Drawing::WaterPattern::sparse : Drawing::WaterPattern::opaque, rotation);
            if (overlayImage != kImageIndexUndefined)
                PaintAttachToPreviousPS(session, ImageId(overlayImage), 0, 0);
        }

        if (!session.ViewFlags.has(ViewportFlag::hideVertical))
            PaintProceduralEdges(session, tileElement, waterHeight, true);
    }

    if ((tileElement.getParkFences()))
    {
        // Owned land boundary fences
        session.InteractionType = ViewportInteractionItem::parkEntrance;

        uint8_t rotatedFences = Numerics::rol4(tileElement.getParkFences(), rotation);

        for (const auto& fenceData : _tileSurfaceBoundaries)
        {
            const int32_t edgeHasFence = rotatedFences & 1;
            rotatedFences >>= 1;

            if (edgeHasFence == 0)
                continue;

            int32_t fenceHeight = height;
            int32_t image_id = 0;

            if (!(surfaceShape & fenceData.bit_1))
            { // first
                if (surfaceShape & fenceData.bit_8)
                { // second
                    image_id = fenceData.image[2];
                }
                else
                {
                    image_id = fenceData.image[0];
                }
            }
            else if (!(surfaceShape & fenceData.bit_8))
            { // Loc6619A2:
                image_id = fenceData.image[1];
            }
            else
            {
                fenceHeight += 16;

                if (!(surfaceShape & 0x10))
                { // Loc6619B5 (first)
                    image_id = fenceData.image[0];
                }
                else if (surfaceShape & fenceData.bit_4)
                { // Loc6619B5 (second)
                    image_id = fenceData.image[3];
                }
                else if (surfaceShape & fenceData.bit_2)
                { // Loc6619B5 (third)
                    image_id = fenceData.image[4];
                }
                else
                {
                    image_id = fenceData.image[0];
                }
            }

            if (!waterIsTransparent && fenceHeight < waterHeight)
            {
                continue;
            }

            PaintAddImageAsParent(
                session, ImageId(image_id), { fenceData.offset, fenceHeight },
                { { fenceData.Boundbox.offset, fenceHeight + 1 }, { fenceData.Boundbox.length, 9 } });
        }
    }

    session.InteractionType = ViewportInteractionItem::terrain;

    switch (surfaceShape)
    {
        default:
            // Loc661C2C
            //     00
            //   00  00
            // 00  00  00
            //   00  00
            //     00
            PaintUtilSetSegmentSupportHeight(session, kSegmentsAll, height, 0);
            PaintUtilForceSetGeneralSupportHeight(session, height, 0);
            break;

        case 1:
            // Loc661CB9
            //     00
            //   00  00
            // 01  01  01
            //   1B  1B
            //     1B
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::top, PaintSegment::topLeft, PaintSegment::topRight }, height, 0);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::left, PaintSegment::centre, PaintSegment::right }, height, 1);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::bottomLeft, PaintSegment::bottomRight }, height + 6, 0x1B);
            PaintUtilSetSegmentSupportHeight(session, PaintSegment::bottom, height + 6 + 6, 0x1B);
            PaintUtilForceSetGeneralSupportHeight(session, height, 1);
            break;

        case 2:
            // Loc661D4E
            //     02
            //   17  00
            // 17  02  00
            //   17  00
            //     02
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::right, PaintSegment::topRight, PaintSegment::bottomRight }, height, 0);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::top, PaintSegment::centre, PaintSegment::bottom }, height, 2);
            PaintUtilSetSegmentSupportHeight(session, { PaintSegment::topLeft, PaintSegment::bottomLeft }, height + 6, 0x17);
            PaintUtilSetSegmentSupportHeight(session, PaintSegment::left, height + 6 + 6, 0x17);
            PaintUtilForceSetGeneralSupportHeight(session, height, 2);
            break;

        case 3:
            // Loc661DE3
            //     03
            //   03  03
            // 03  03  03
            //   03  03
            //     03
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::top, PaintSegment::topRight, PaintSegment::right }, height + 2, 3);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::topLeft, PaintSegment::centre, PaintSegment::bottomRight }, height + 2 + 6, 3);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::left, PaintSegment::bottomLeft, PaintSegment::bottom }, height + 2 + 6 + 6, 3);
            PaintUtilForceSetGeneralSupportHeight(session, height, 3);
            break;

        case 4:
            // Loc661E7C
            //     1E
            //   1E  1E
            // 04  04  04
            //   00  00
            //     00
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::bottom, PaintSegment::bottomLeft, PaintSegment::bottomRight }, height, 0);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::left, PaintSegment::centre, PaintSegment::right }, height, 4);
            PaintUtilSetSegmentSupportHeight(session, { PaintSegment::topLeft, PaintSegment::topRight }, height + 6, 0x1E);
            PaintUtilSetSegmentSupportHeight(session, PaintSegment::top, height + 6 + 6, 0x1E);
            PaintUtilForceSetGeneralSupportHeight(session, height, 4);
            break;

        case 5:
            // Loc661F11
            //     1E          ▓▓
            //   1E  1E      ▒▒  ▒▒
            // 05  05  05  ░░  ░░  ░░
            //   1B  1B      ▒▒  ▒▒
            //     1B          ▓▓
            PaintUtilSetSegmentSupportHeight(session, PaintSegment::top, height + 6 + 6, 0x1E);
            PaintUtilSetSegmentSupportHeight(session, { PaintSegment::topLeft, PaintSegment::topRight }, height + 6, 0x1E);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::left, PaintSegment::centre, PaintSegment::right }, height, 5);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::bottomLeft, PaintSegment::bottomRight }, height + 6, 0x1B);
            PaintUtilSetSegmentSupportHeight(session, PaintSegment::bottom, height + 6 + 6, 0x1B);
            PaintUtilForceSetGeneralSupportHeight(session, height, 5);
            break;

        case 6:
            // Loc661FA6
            //     06          ▓▓
            //   06  06      ▓▓  ▒▒
            // 06  06  06  ▓▓  ▒▒  ░░
            //   06  06      ▒▒  ░░
            //     06          ░░
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::right, PaintSegment::bottomRight, PaintSegment::bottom }, height + 2, 6);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::bottomLeft, PaintSegment::centre, PaintSegment::topRight }, height + 2 + 6, 6);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::left, PaintSegment::topLeft, PaintSegment::top }, height + 2 + 6 + 6, 6);
            PaintUtilForceSetGeneralSupportHeight(session, height, 6);
            break;

        case 7:
            // Loc66203F
            //     07          ▓▓
            //   00  17      ▓▓  ▒▒
            // 00  07  17  ▓▓  ▓▓  ░░
            //   00  17      ▓▓  ▒▒
            //     07          ▓▓
            PaintUtilSetSegmentSupportHeight(session, PaintSegment::right, height + 4, 0x17);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::topRight, PaintSegment::bottomRight }, height + 4 + 6, 0x17);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::top, PaintSegment::centre, PaintSegment::bottom }, height + 4 + 6 + 6, 7);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::topLeft, PaintSegment::bottomLeft, PaintSegment::left }, height + 4 + 6 + 6, 0);
            PaintUtilForceSetGeneralSupportHeight(session, height, 7);
            break;

        case 8:
            // Loc6620D8
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::left, PaintSegment::topLeft, PaintSegment::bottomLeft }, height, 0);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::top, PaintSegment::centre, PaintSegment::bottom }, height, 8);
            PaintUtilSetSegmentSupportHeight(session, { PaintSegment::topRight, PaintSegment::bottomRight }, height + 6, 0x1D);
            PaintUtilSetSegmentSupportHeight(session, PaintSegment::right, height + 6 + 6, 0x1D);
            PaintUtilForceSetGeneralSupportHeight(session, height, 8);
            break;

        case 9:
            // Loc66216D
            PaintUtilForceSetGeneralSupportHeight(session, height, 9);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::top, PaintSegment::topLeft, PaintSegment::left }, height + 2, 9);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::bottomLeft, PaintSegment::centre, PaintSegment::topRight }, height + 2 + 6, 9);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::bottom, PaintSegment::bottomRight, PaintSegment::right }, height + 2 + 6 + 6, 9);
            break;

        case 10:
            // Loc662206
            PaintUtilForceSetGeneralSupportHeight(session, height, 0xA);
            PaintUtilSetSegmentSupportHeight(session, PaintSegment::left, height + 6 + 6, 0x17);
            PaintUtilSetSegmentSupportHeight(session, { PaintSegment::topLeft, PaintSegment::bottomLeft }, height + 6, 0x17);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::top, PaintSegment::centre, PaintSegment::bottom }, height, 0xA);
            PaintUtilSetSegmentSupportHeight(session, { PaintSegment::topRight, PaintSegment::bottomRight }, height + 6, 0x1D);
            PaintUtilSetSegmentSupportHeight(session, PaintSegment::right, height + 6 + 6, 0x1D);
            break;

        case 11:
            // Loc66229B
            PaintUtilForceSetGeneralSupportHeight(session, height, 0xB);
            PaintUtilSetSegmentSupportHeight(session, PaintSegment::top, height + 4, 0x1B);
            PaintUtilSetSegmentSupportHeight(session, { PaintSegment::topLeft, PaintSegment::topRight }, height + 4 + 6, 0x1B);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::left, PaintSegment::centre, PaintSegment::right }, height + 4 + 6 + 6, 0xB);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::bottomLeft, PaintSegment::bottomRight, PaintSegment::bottom }, height + 4 + 6 + 6, 0);
            break;

        case 12:
            // Loc662334
            PaintUtilForceSetGeneralSupportHeight(session, height, 0xC);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::left, PaintSegment::bottomLeft, PaintSegment::bottom }, height + 2, 0xC);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::topLeft, PaintSegment::centre, PaintSegment::bottomRight }, height + 2 + 6, 0xC);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::top, PaintSegment::topRight, PaintSegment::right }, height + 2 + 6 + 6, 0xC);
            break;

        case 13:
            // Loc6623CD
            PaintUtilForceSetGeneralSupportHeight(session, height, 0xD);
            PaintUtilSetSegmentSupportHeight(session, PaintSegment::left, height + 4, 0x1D);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::topLeft, PaintSegment::bottomLeft }, height + 4 + 6, 0x1D);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::top, PaintSegment::centre, PaintSegment::bottom }, height + 4 + 6 + 6, 0xD);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::topRight, PaintSegment::bottomRight, PaintSegment::right }, height + 4 + 6 + 6, 0);
            break;

        case 14:
            // Loc662466
            PaintUtilForceSetGeneralSupportHeight(session, height, 0xE);
            PaintUtilSetSegmentSupportHeight(session, PaintSegment::bottom, height + 4, 0x1E);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::bottomLeft, PaintSegment::bottomRight }, height + 4 + 6, 0x1E);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::left, PaintSegment::centre, PaintSegment::right }, height + 4 + 6 + 6, 0xE);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::topLeft, PaintSegment::topRight, PaintSegment::top }, height + 4 + 6 + 6, 0);
            break;

        case 23:
            // Loc6624FF
            PaintUtilForceSetGeneralSupportHeight(session, height, 0x17);
            PaintUtilSetSegmentSupportHeight(session, PaintSegment::right, height + 4, 0x17);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::topRight, PaintSegment::bottomRight }, height + 4 + 6, 0x17);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::top, PaintSegment::centre, PaintSegment::bottom }, height + 4 + 6 + 6, 0x17);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::topLeft, PaintSegment::bottomLeft }, height + 4 + 6 + 6 + 6, 0x17);
            PaintUtilSetSegmentSupportHeight(session, PaintSegment::left, height + 4 + 6 + 6 + 6 + 6, 0x17);
            break;

        case 27:
            // Loc6625A0
            PaintUtilForceSetGeneralSupportHeight(session, height, 0x1B);
            PaintUtilSetSegmentSupportHeight(session, PaintSegment::top, height + 4, 0x1B);
            PaintUtilSetSegmentSupportHeight(session, { PaintSegment::topLeft, PaintSegment::topRight }, height + 4 + 6, 0x1B);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::left, PaintSegment::centre, PaintSegment::right }, height + 4 + 6 + 6, 0x1B);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::bottomLeft, PaintSegment::bottomRight }, height + 4 + 6 + 6 + 6, 0x1B);
            PaintUtilSetSegmentSupportHeight(session, PaintSegment::bottom, height + 4 + 6 + 6 + 6 + 6, 0x1B);
            break;

        case 29:
            // Loc662641
            PaintUtilForceSetGeneralSupportHeight(session, height, 0x1D);
            PaintUtilSetSegmentSupportHeight(session, PaintSegment::left, height + 4, 0x1D);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::topLeft, PaintSegment::bottomLeft }, height + 4 + 6, 0x1D);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::top, PaintSegment::centre, PaintSegment::bottom }, height + 4 + 6 + 6, 0x1D);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::topRight, PaintSegment::bottomRight }, height + 4 + 6 + 6 + 6, 0x1D);
            PaintUtilSetSegmentSupportHeight(session, PaintSegment::right, height + 4 + 6 + 6 + 6 + 6, 0x1D);
            break;

        case 30:
            // Loc6626E2
            PaintUtilForceSetGeneralSupportHeight(session, height, 0x1E);
            PaintUtilSetSegmentSupportHeight(session, PaintSegment::bottom, height + 4, 0x1E);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::bottomLeft, PaintSegment::bottomRight }, height + 4 + 6, 0x1E);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::left, PaintSegment::centre, PaintSegment::right }, height + 4 + 6 + 6, 0x1E);
            PaintUtilSetSegmentSupportHeight(
                session, { PaintSegment::topLeft, PaintSegment::topRight }, height + 4 + 6 + 6 + 6, 0x1E);
            PaintUtilSetSegmentSupportHeight(session, PaintSegment::top, height + 4 + 6 + 6 + 6 + 6, 0x1E);
            break;
    }

    PaintHoleMarkers(session);
}
