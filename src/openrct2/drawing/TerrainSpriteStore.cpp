/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "TerrainSpriteStore.h"

#include <algorithm>

#include "../GameState.h"
#include "../config/Config.h"
#include "HoleMarkerSprites.h"
#include "../object/StreamObject.h"
#include "../object/StreetObject.h"
#include "../object/TerrainEdgeObject.h"
#include "../object/TerrainSurfaceObject.h"
#include "../world/Map.h"
#include "../world/tile_element/StreamElement.h"
#include "../world/tile_element/ParkingElement.h"
#include "../world/tile_element/StreetElement.h"
#include "../world/tile_element/SurfaceElement.h"
#include "G1Element.h"
#include "Image.h"
#include "ImageImporter.h"

namespace OpenRCT2::Drawing
{
    namespace
    {
        // The tile diamond is 32 pixels tall and the generator is relative to its centre; images are relative to the
        // top corner.
        constexpr int32_t kCentreToTopCorner = 16;

        TerrainShadeTable BuildShades()
        {
            std::array<std::array<uint8_t, 3>, 256> rgb{};
            for (size_t i = 0; i < rgb.size(); i++)
                rgb[i] = { StandardPalette[i].red, StandardPalette[i].green, StandardPalette[i].blue };
            return TerrainShadeTable::FromPalette(rgb);
        }
    } // namespace

    TerrainSpriteStore& TerrainSpriteStore::Instance()
    {
        static TerrainSpriteStore store;
        return store;
    }

    TerrainSpriteStore::TerrainSpriteStore()
    {
        _cache.SetEvictCallback([this](const TerrainSpriteKey& key, TerrainSprite&) {
            auto it = _images.find(key);
            if (it != _images.end())
            {
                GfxObjectFreeImages(it->second, 1);
                _images.erase(it);
            }
        });
    }

    void TerrainSpriteStore::BeginFrame()
    {
        std::lock_guard lock(_mutex);
        _cache.BeginFrame();
    }

    void TerrainSpriteStore::Clear()
    {
        std::lock_guard lock(_mutex);
        _cache.Clear();
    }

    size_t TerrainSpriteStore::GetSpriteCount() const
    {
        std::lock_guard lock(_mutex);
        return _cache.GetCount();
    }

    ImageIndex TerrainSpriteStore::RegisterImage(const TerrainSpriteKey& key, const TerrainSprite& sprite)
    {
        auto it = _images.find(key);
        if (it != _images.end())
            return it->second;

        G1Element g1{};
        g1.offset = const_cast<uint8_t*>(sprite.pixels.data());
        g1.width = static_cast<int16_t>(sprite.width);
        g1.height = static_cast<int16_t>(sprite.height);
        g1.xOffset = static_cast<int16_t>(sprite.xOffset);
        g1.yOffset = static_cast<int16_t>(sprite.yOffset + kCentreToTopCorner);
        g1.flags = { G1Flag::hasTransparency };
        const auto image = GfxObjectAllocateImages(&g1, 1);
        if (image == kImageIndexUndefined)
            return kImageIndexUndefined;
        _images[key] = image;
        return image;
    }

    ImageIndex TerrainSpriteStore::GetEdgeImage(ObjectEntryIndex edgeObject, const TerrainEdgeFace& face, int32_t rotation)
    {
        const auto* object = TerrainEdgeObject::GetById(edgeObject);
        const auto* texture = object != nullptr ? object->getTexture() : nullptr;
        if (texture == nullptr)
            return kImageIndexUndefined;

        TerrainSpriteKey key{ kEdgeFaceShapeIndex, {}, static_cast<uint8_t>(rotation & 3), 0 };
        key.quadrantSurfaces = { static_cast<uint8_t>(edgeObject), static_cast<uint8_t>(face.edge), 0, 0 };
        key.edgeData = { static_cast<int16_t>(face.topA), static_cast<int16_t>(face.topB), static_cast<int16_t>(face.bottomA),
                         static_cast<int16_t>(face.bottomB), static_cast<int16_t>(face.baseZ & 63) };

        std::lock_guard lock(_mutex);
        if (!_shadesBuilt)
        {
            _shades = BuildShades();
            _shadesBuilt = true;
        }
        const auto* sprite = _cache.Get(key, [&] { return GenerateTerrainEdgeSprite(*texture, face, _shades, rotation, 0); });
        if (sprite == nullptr || sprite->pixels.empty())
            return kImageIndexUndefined;
        return RegisterImage(key, *sprite);
    }

    ImageIndex TerrainSpriteStore::GetWaterImage(const SurfaceElement& surface, WaterPattern pattern, int32_t rotation)
    {
        const int32_t waterZ = surface.getWaterHeight();
        const auto shape = surface.getTerrainShape();
        const auto shapeIndex = Terrain::GetShapeIndex(shape);
        if (waterZ <= 0 || shapeIndex == 0xFFFF)
            return kImageIndexUndefined;
        const int32_t rel = surface.getBaseZ() - waterZ;
        const int32_t highest = rel + std::max({ shape.north, shape.east, shape.south, shape.west }) * Terrain::kQuarterStepZ;
        if (rel >= 0)
            return kImageIndexUndefined; // the whole tile is dry
        (void)highest;

        TerrainSpriteKey key{ static_cast<uint16_t>(kWaterShapeIndexBase + shapeIndex), {}, static_cast<uint8_t>(rotation & 3), 0 };
        key.quadrantSurfaces[0] = static_cast<uint8_t>(pattern);
        key.edgeData[0] = static_cast<int16_t>(std::clamp(rel, -4000, 0));

        std::lock_guard lock(_mutex);
        const auto* sprite = _cache.Get(
            key, [&] { return GenerateTerrainWaterSprite(shape, key.edgeData[0], pattern, rotation, 0); });
        if (sprite == nullptr || sprite->pixels.empty())
            return kImageIndexUndefined;
        return RegisterImage(key, *sprite);
    }

    ImageIndex TerrainSpriteStore::GetOverlayImage(const Terrain::TerrainShape& shape, const TerrainOverlay& overlay, int32_t rotation)
    {
        const auto shapeIndex = Terrain::GetShapeIndex(shape);
        if (shapeIndex == 0xFFFF)
            return kImageIndexUndefined;
        TerrainSpriteKey key{ static_cast<uint16_t>(kOverlayShapeIndexBase + shapeIndex), {}, static_cast<uint8_t>(rotation & 3), 0 };
        key.quadrantSurfaces = { static_cast<uint8_t>(overlay.kind), overlay.index, overlay.colour, 0 };

        std::lock_guard lock(_mutex);
        const auto* sprite = _cache.Get(key, [&] { return GenerateTerrainOverlaySprite(shape, overlay, rotation, 0); });
        if (sprite == nullptr || sprite->pixels.empty())
            return kImageIndexUndefined;
        return RegisterImage(key, *sprite);
    }

    ImageIndex TerrainSpriteStore::GetStreamImage(const SurfaceElement& surface, const StreamElement& stream, int32_t rotation)
    {
        const auto shape = surface.getTerrainShape();
        const auto shapeIndex = Terrain::GetShapeIndex(shape);
        if (shapeIndex == 0xFFFF)
            return kImageIndexUndefined;

        const auto* streamObject = stream.getObject();
        if (streamObject == nullptr || streamObject->Frames.empty())
            return kImageIndexUndefined;

        const int32_t frameDuration = std::max(1, streamObject->FrameDurationTicks);
        const int32_t frameCount = std::max(1, streamObject->FrameCount);
        const int32_t frameIdx = (getGameState().currentTicks / frameDuration) % frameCount;
        const auto* texture = streamObject->getFrame(frameIdx);
        if (texture == nullptr)
            return kImageIndexUndefined;

        TerrainSpriteKey key{ static_cast<uint16_t>(kStreamShapeIndexBase + shapeIndex), {}, static_cast<uint8_t>(rotation & 3), 0 };
        key.quadrantSurfaces[0] = static_cast<uint8_t>(stream.getObjectIndex());
        key.quadrantSurfaces[1] = stream.getEdges();
        key.quadrantSurfaces[2] = static_cast<uint8_t>(frameIdx);
        key.quadrantSurfaces[3] = static_cast<uint8_t>(std::clamp(streamObject->Width, 1, 32));

        std::lock_guard lock(_mutex);
        if (!_shadesBuilt)
        {
            _shades = BuildShades();
            _shadesBuilt = true;
        }
        const auto* sprite = _cache.Get(key, [&] {
            return GenerateTerrainStreamSprite(shape, *texture, stream.getEdges(), streamObject->Width, _shades, rotation, 0);
        });
        if (sprite == nullptr || sprite->pixels.empty())
            return kImageIndexUndefined;
        return RegisterImage(key, *sprite);
    }

    ImageIndex TerrainSpriteStore::GetStreetImage(const SurfaceElement& surface, const StreetElement& street, int32_t rotation)
    {
        const auto shape = surface.getTerrainShape();
        const auto shapeIndex = Terrain::GetShapeIndex(shape);
        if (shapeIndex == 0xFFFF)
            return kImageIndexUndefined;

        const auto* streetObject = street.getObject();
        if (streetObject == nullptr || streetObject->Texture.size == 0)
            return kImageIndexUndefined;

        // The flow only changes the picture of a one-way road.
        const int32_t flow = streetObject->OneWay ? street.getFlowDirection() : 0;
        TerrainSpriteKey key{ static_cast<uint16_t>(kStreetShapeIndexBase + shapeIndex), {}, static_cast<uint8_t>(rotation & 3), 0 };
        key.quadrantSurfaces[0] = static_cast<uint8_t>(street.getObjectIndex());
        key.quadrantSurfaces[1] = street.getConnections();
        key.quadrantSurfaces[2] = static_cast<uint8_t>(flow);
        key.quadrantSurfaces[3] = static_cast<uint8_t>(kStreetRoadWidth);

        std::lock_guard lock(_mutex);
        if (!_shadesBuilt)
        {
            _shades = BuildShades();
            _shadesBuilt = true;
        }
        const auto* sprite = _cache.Get(key, [&] {
            return GenerateTerrainStreetSprite(
                shape, streetObject->Texture, streetObject->Markings, streetObject->markingTable(), street.getConnections(),
                streetObject->OneWay, flow, kStreetRoadWidth, _shades, rotation, 0);
        });
        if (sprite == nullptr || sprite->pixels.empty())
            return kImageIndexUndefined;
        return RegisterImage(key, *sprite);
    }

    ImageIndex TerrainSpriteStore::GetParkingImage(const SurfaceElement& surface, const ParkingElement& parking, int32_t rotation)
    {
        const auto shape = surface.getTerrainShape();
        const auto shapeIndex = Terrain::GetShapeIndex(shape);
        if (shapeIndex == 0xFFFF)
            return kImageIndexUndefined;

        const auto* streetObject = parking.getObject();
        if (streetObject == nullptr || streetObject->Texture.size == 0)
            return kImageIndexUndefined;

        // The bay lines are white, as the centre line of the street.
        const uint8_t lineColour = FindPaletteIndex(255, 255, 255);
        TerrainSpriteKey key{ static_cast<uint16_t>(kParkingShapeIndexBase + shapeIndex), {}, static_cast<uint8_t>(rotation & 3), 0 };
        key.quadrantSurfaces[0] = static_cast<uint8_t>(parking.getObjectIndex());
        key.quadrantSurfaces[1] = static_cast<uint8_t>(parking.getFacing());
        key.quadrantSurfaces[2] = lineColour;
        key.quadrantSurfaces[3] = parking.isCartBay() ? 1 : 0;

        std::lock_guard lock(_mutex);
        if (!_shadesBuilt)
        {
            _shades = BuildShades();
            _shadesBuilt = true;
        }
        const auto* sprite = _cache.Get(key, [&] {
            return GenerateTerrainParkingSprite(
                shape, streetObject->Texture, lineColour, parking.getFacing(), _shades, rotation, 0, parking.isCartBay());
        });
        if (sprite == nullptr || sprite->pixels.empty())
            return kImageIndexUndefined;
        return RegisterImage(key, *sprite);
    }

    ImageIndex TerrainSpriteStore::GetImage(const SurfaceElement& surface, const CoordsXY& tilePos, int32_t rotation)
    {
        const auto shape = surface.getTerrainShape();
        const auto shapeIndex = Terrain::GetShapeIndex(shape);
        if (shapeIndex == 0xFFFF)
            return kImageIndexUndefined;

        std::array<const TerrainTexture*, 4> textures{};
        TerrainSpriteKey key{ shapeIndex, {}, static_cast<uint8_t>(rotation & 3), 0 };
        for (uint8_t q = 0; q < 4; q++)
        {
            const auto surfaceIndex = surface.getQuadrantSurfaceIndex(static_cast<Terrain::Quadrant>(q));
            const auto* object = TerrainSurfaceObject::GetById(surfaceIndex);
            textures[q] = object != nullptr ? object->getTexture() : nullptr;
            if (textures[q] == nullptr)
                return kImageIndexUndefined;
            key.quadrantSurfaces[q] = static_cast<uint8_t>(surfaceIndex);
        }

        // The quadrant across each tile edge: north-east touches x + 1, south-east y - 1, south-west x - 1, north-west y + 1.
        TerrainSmoothing smoothing;
        smoothing.enabled = Config::Get().general.landscapeSmoothing;
        key.smoothing = smoothing.enabled;
        if (smoothing.enabled)
        {
            struct Across
            {
                CoordsXY offset;
                Terrain::Quadrant touching;
            };
            const Across across[4] = {
                { { kCoordsXYStep, 0 }, Terrain::Quadrant::southWest },
                { { 0, -kCoordsXYStep }, Terrain::Quadrant::northWest },
                { { -kCoordsXYStep, 0 }, Terrain::Quadrant::northEast },
                { { 0, kCoordsXYStep }, Terrain::Quadrant::southEast },
            };
            for (size_t q = 0; q < 4; q++)
            {
                key.neighbourSurfaces[q] = key.quadrantSurfaces[q];
                const auto neighbourPos = tilePos + across[q].offset;
                if (!MapIsLocationValid(neighbourPos))
                    continue;
                const auto* neighbour = MapGetSurfaceElementAt(neighbourPos);
                if (neighbour == nullptr)
                    continue;
                const auto neighbourSurface = neighbour->getQuadrantSurfaceIndex(across[q].touching);
                const auto* object = TerrainSurfaceObject::GetById(neighbourSurface);
                if (object == nullptr || object->getTexture() == nullptr)
                    continue;
                key.neighbourSurfaces[q] = static_cast<uint8_t>(neighbourSurface);
                smoothing.edgeNeighbours[q] = object->getTexture();
            }
        }

        std::lock_guard lock(_mutex);
        if (!_shadesBuilt)
        {
            _shades = BuildShades();
            _shadesBuilt = true;
        }
        const auto* sprite = _cache.Get(key, [&] { return GenerateTerrainSprite(shape, textures, _shades, rotation, 0, smoothing); });
        if (sprite == nullptr || sprite->pixels.empty())
            return kImageIndexUndefined;

        return RegisterImage(key, *sprite);
    }
} // namespace OpenRCT2::Drawing
