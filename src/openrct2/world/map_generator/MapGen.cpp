/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "MapGen.h"

#include "../../Context.h"
#include "../../GameState.h"
#include "../../object/ObjectManager.h"
#include "../../object/TerrainEdgeObject.h"
#include "../../object/TerrainSurfaceObject.h"
#include "../Map.h"
#include "../MapLimits.h"
#include "../TerrainHeightField.h"
#include "../tile_element/Slope.h"
#include "../tile_element/SurfaceElement.h"
#include "HeightMap.hpp"
#include "PngTerrainGenerator.h"
#include "SimplexNoise.h"
#include "SurfaceSelection.h"
#include "TreePlacement.h"

namespace OpenRCT2::World::MapGenerator
{
    static void generateBlankMap(Settings* settings);

    static void addBeaches(Settings* settings);

    void generate(Settings* settings)
    {
        // First, generate the height map
        switch (settings->algorithm)
        {
            case Algorithm::blank:
                generateBlankMap(settings);
                break;

            case Algorithm::simplexNoise:
                generateSimplexMap(settings);
                break;

            case Algorithm::heightmapImage:
                GenerateFromHeightmapImage(settings);
                break;
        }

        // Add beaches?
        if (settings->beaches)
            addBeaches(settings);

        // Place trees?
        if (settings->trees)
            placeTrees(settings);
    }

    void resetSurfaces(Settings* settings)
    {
        MapClearAllElements();
        MapInit(settings->mapSize, Drawing::Colour::black);

        const auto surfaceTextureId = generateSurfaceTextureId(settings);
        const auto edgeTextureId = generateEdgeTextureId(settings, surfaceTextureId);

        auto& objManager = GetContext()->GetObjectManager();

        auto surfaceColour1 = Drawing::Colour::black;
        const auto* surfaceObject = objManager.GetLoadedObject<TerrainSurfaceObject>(surfaceTextureId);
        if (surfaceObject != nullptr && surfaceObject->Flags.has(TerrainSurfaceFlag::hasPrimaryColour))
        {
            surfaceColour1 = settings->surfaceColour1;
        }

        auto edgeColour1 = Drawing::Colour::black;
        const auto* edgeObject = objManager.GetLoadedObject<TerrainEdgeObject>(edgeTextureId);
        if (edgeObject != nullptr && edgeObject->flags.has(TerrainEdgeFlag::hasPrimaryColour))
        {
            edgeColour1 = settings->edgeColour1;
        }

        for (auto y = 1; y < settings->mapSize.y - 1; y++)
        {
            for (auto x = 1; x < settings->mapSize.x - 1; x++)
            {
                auto surfaceElement = MapGetSurfaceElementAt(TileCoordsXY{ x, y });
                if (surfaceElement != nullptr)
                {
                    surfaceElement->setSurfaceObjectIndex(surfaceTextureId);
                    surfaceElement->setEdgeObjectIndex(edgeTextureId);
                    surfaceElement->setPrimarySurfaceColour(surfaceColour1);
                    surfaceElement->setPrimaryEdgeColour(edgeColour1);
                    surfaceElement->baseHeight = settings->heightmapLow;
                    surfaceElement->clearanceHeight = settings->heightmapLow;
                }
            }
        }
    }

    static void generateBlankMap(Settings* settings)
    {
        resetSurfaces(settings);
        setWaterLevel(settings->waterLevel);
    }

    static void addBeaches(Settings* settings)
    {
        auto beachTextureId = generateBeachTextureId();
        if (beachTextureId == kObjectEntryIndexNull)
            return;

        // Add sandy beaches
        auto& gameState = getGameState();
        for (auto y = 1; y < gameState.mapSize.y - 1; y++)
        {
            for (auto x = 1; x < gameState.mapSize.x - 1; x++)
            {
                auto surfaceElement = MapGetSurfaceElementAt(TileCoordsXY{ x, y });

                if (surfaceElement != nullptr && surfaceElement->baseHeight < settings->waterLevel + 6)
                    surfaceElement->setSurfaceObjectIndex(beachTextureId);
            }
        }
    }

    /**
     * Sets each tile's water level to the specified water level if underneath that water level.
     */
    void setWaterLevel(int32_t waterLevel)
    {
        auto& gameState = getGameState();
        for (int32_t y = 1; y < gameState.mapSize.y - 1; y++)
        {
            for (int32_t x = 1; x < gameState.mapSize.x - 1; x++)
            {
                auto surfaceElement = MapGetSurfaceElementAt(TileCoordsXY{ x, y });
                if (surfaceElement != nullptr && surfaceElement->baseHeight < waterLevel)
                    surfaceElement->setWaterHeight(waterLevel * kCoordsZStep);
            }
        }
    }

    /**
     * Sets the height of the actual game map tiles to the height map.
     */
    void setMapHeight(Settings* settings, const HeightMap& heightMap)
    {
        const int32_t tilesX = heightMap.width / heightMap.density, tilesY = heightMap.height / heightMap.density;

        // Height of a grid vertex in quarter steps, like the height map.
        auto vertexHeight = [&](int32_t vx, int32_t vy) {
            const auto sx = std::min<int32_t>(vx * heightMap.density, heightMap.width - 1);
            const auto sy = std::min<int32_t>(vy * heightMap.density, heightMap.height - 1);
            int32_t height = std::max<int32_t>(2, heightMap[{ sx, sy }]);

            // If it is below water level, lower it to create more natural shorelines
            if (height >= 4 && height <= settings->waterLevel)
                height -= 2;
            return height;
        };

        // Interior tiles are tiles 1 .. size - 2.
        const int32_t innerX = tilesX - 2, innerY = tilesY - 2;
        if (innerX <= 0 || innerY <= 0)
            return;

        if (!settings->smoothTileEdges)
        {
            // Flat tiles at the average height of their corners: terraces with cliffs between them.
            for (int32_t y = 1; y < tilesY - 1; y++)
            {
                for (int32_t x = 1; x < tilesX - 1; x++)
                {
                    auto* surfaceElement = MapGetSurfaceElementAt(TileCoordsXY{ x, y });
                    if (surfaceElement == nullptr)
                        continue;
                    const int32_t sum = vertexHeight(x, y) + vertexHeight(x + 1, y) + vertexHeight(x, y + 1)
                        + vertexHeight(x + 1, y + 1);
                    surfaceElement->baseHeight = static_cast<uint8_t>(std::max(2, (sum + 2) / 4));
                    surfaceElement->clearanceHeight = surfaceElement->baseHeight;
                    surfaceElement->setTerrainShape({ 0, 0, 0, 0 });
                }
            }
            return;
        }

        // Continuous terrain: tiles share their corner heights, so there are no cliffs.
        Terrain::HeightField field(innerX, innerY, kMaximumLandHeight, kMinimumLandHeight);
        for (int32_t vy = 0; vy <= innerY; vy++)
            for (int32_t vx = 0; vx <= innerX; vx++)
                field.Set(vx, vy, vertexHeight(vx + 1, vy + 1));
        field.Smooth();

        for (int32_t y = 0; y < innerY; y++)
        {
            for (int32_t x = 0; x < innerX; x++)
            {
                auto* surfaceElement = MapGetSurfaceElementAt(TileCoordsXY{ x + 1, y + 1 });
                if (surfaceElement == nullptr)
                    continue;
                const auto [base, shape] = field.GetTile(x, y);
                surfaceElement->baseHeight = static_cast<uint8_t>(base);
                surfaceElement->clearanceHeight = static_cast<uint8_t>(
                    base + std::max({ shape.north, shape.east, shape.south, shape.west }));
                surfaceElement->setTerrainShape(shape);
            }
        }
    }
} // namespace OpenRCT2::World::MapGenerator
