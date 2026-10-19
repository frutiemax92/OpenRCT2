/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "TerrainSpriteStore.h"

#include "../config/Config.h"
#include "../object/TerrainEdgeObject.h"
#include "../object/TerrainSurfaceObject.h"
#include "../world/Map.h"
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
