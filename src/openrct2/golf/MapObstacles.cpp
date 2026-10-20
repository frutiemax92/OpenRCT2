/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "MapObstacles.h"

#include "../Context.h"
#include "../object/BuildingObject.h"
#include "../object/LargeSceneryObject.h"
#include "../object/ObjectManager.h"
#include "../object/SmallSceneryObject.h"
#include "../object/WallObject.h"
#include "../world/Map.h"
#include "../world/Scenery.h"
#include "../world/TileElementsView.h"
#include "../world/tile_element/BuildingElement.h"
#include "../world/tile_element/LargeSceneryElement.h"
#include "../world/tile_element/SmallSceneryElement.h"
#include "../world/tile_element/WallElement.h"

#include <cstring>

namespace OpenRCT2::Golf
{
    namespace
    {
        constexpr int32_t kTile = 32;
        constexpr int32_t kMaxElementsPerTile = 16;

        bool isObstacleType(TileElementType type)
        {
            return type == TileElementType::smallScenery || type == TileElementType::wall
                || type == TileElementType::largeScenery || type == TileElementType::building;
        }

        uint64_t signatureOf(const TileCoordsXY& tile)
        {
            uint64_t h = 1469598103934665603ull;
            auto mix = [&](uint64_t v) {
                h ^= v;
                h *= 1099511628211ull;
            };
            for (auto* element : TileElementsView<TileElement>(tile.toCoordsXY()))
            {
                if (!isObstacleType(element->getType()))
                    continue;
                // The whole 16 byte element: type, heights, object, rotation, quadrant. Anything that matters changes it.
                uint8_t raw[16];
                std::memcpy(raw, element, sizeof(raw));
                for (auto byte : raw)
                    mix(byte);
            }
            return h;
        }

        int64_t tileKey(const TileCoordsXY& tile)
        {
            return (static_cast<int64_t>(tile.x) << 32) ^ static_cast<int64_t>(static_cast<uint32_t>(tile.y));
        }

        uint32_t elementId(const TileCoordsXY& tile, int32_t index)
        {
            return static_cast<uint32_t>((tile.y * 1024 + tile.x) * kMaxElementsPerTile + index) + 1;
        }

        Obstacle fullTileBox(uint32_t id, const CoordsXY& tile, int32_t baseZ, const ObjectCollision& c)
        {
            ObjectCollision tileBox = c;
            tileBox.shape = c.shape == CollisionShape::cylinder ? CollisionShape::cylinder : CollisionShape::box;
            tileBox.footprintX = kTile;
            tileBox.footprintY = kTile;
            return makeObstacle(id, tileBox, { Fixed::fromInt(tile.x + kTile / 2), Fixed::fromInt(tile.y + kTile / 2), Fixed::fromInt(baseZ) });
        }

        void addObstacles(
            ObstacleGrid& grid, std::vector<uint32_t>& ids, const TileCoordsXY& tile, TileElement* element, int32_t index)
        {
            auto& objectManager = GetContext()->GetObjectManager();
            const auto id = elementId(tile, index);
            const CoordsXY origin = tile.toCoordsXY();
            Obstacle obstacle;

            if (auto* small = element->as<SmallSceneryElement>())
            {
                auto* object = objectManager.GetLoadedObject<SmallSceneryObject>(small->getEntryIndex());
                auto* entry = small->getEntry();
                if (object == nullptr || entry == nullptr)
                    return;
                CoordsXY centre{ origin.x + kTile / 2, origin.y + kTile / 2 };
                if (!entry->flags.has(SmallSceneryFlag::occupiesFullTile))
                {
                    const auto& offset = SceneryQuadrantOffsets[small->getSceneryQuadrant() & 3];
                    centre = { origin.x + offset.x, origin.y + offset.y };
                }
                obstacle = makeObstacle(
                    id, object->Collision,
                    { Fixed::fromInt(centre.x), Fixed::fromInt(centre.y), Fixed::fromInt(small->getBaseZ()) });
            }
            else if (auto* large = element->as<LargeSceneryElement>())
            {
                auto* object = objectManager.GetLoadedObject<LargeSceneryObject>(large->getEntryIndex());
                if (object == nullptr)
                    return;
                obstacle = fullTileBox(id, origin, large->getBaseZ(), object->Collision);
            }
            else if (auto* wall = element->as<WallElement>())
            {
                auto* object = objectManager.GetLoadedObject<WallObject>(wall->getEntryIndex());
                if (object == nullptr || object->Collision.shape == CollisionShape::none)
                    return;
                // A thin box along the tile edge the wall stands on (0 = -x, 1 = +y, 2 = +x, 3 = -y).
                const int32_t half = MapObstacles::kWallThickness / 2;
                Fixed cx = Fixed::fromInt(origin.x + kTile / 2), cy = Fixed::fromInt(origin.y + kTile / 2);
                Fixed hx = Fixed::fromInt(kTile / 2), hy = Fixed::fromInt(kTile / 2);
                switch (wall->getDirection() & 3)
                {
                    case 0:
                        cx = Fixed::fromInt(origin.x + half);
                        hx = Fixed::fromInt(half);
                        break;
                    case 1:
                        cy = Fixed::fromInt(origin.y + kTile - half);
                        hy = Fixed::fromInt(half);
                        break;
                    case 2:
                        cx = Fixed::fromInt(origin.x + kTile - half);
                        hx = Fixed::fromInt(half);
                        break;
                    default:
                        cy = Fixed::fromInt(origin.y + half);
                        hy = Fixed::fromInt(half);
                        break;
                }
                obstacle = makeObstacle(id, object->Collision, { cx, cy, Fixed::fromInt(wall->getBaseZ()) });
                obstacle.shape = ObstacleShape::box;
                obstacle.halfX = hx;
                obstacle.halfY = hy;
            }
            else if (auto* building = element->as<BuildingElement>())
            {
                auto* object = building->getObject();
                if (object == nullptr)
                    return;
                obstacle = fullTileBox(id, origin, building->getBaseZ(), object->Collision);
            }
            else
            {
                return;
            }

            if (grid.add(obstacle))
                ids.push_back(id);
        }
    } // namespace

    void MapObstacles::refreshTile(const TileCoordsXY& tile)
    {
        const auto signature = signatureOf(tile);
        const auto key = tileKey(tile);
        auto seen = _tiles.find(key);
        if (seen != _tiles.end() && seen->second.signature == signature)
            return;

        auto& state = _tiles[key];
        for (auto id : state.ids)
            _grid.remove(id);
        state.ids.clear();
        state.signature = signature;
        _rebuilds++;

        int32_t index = 0;
        for (auto* element : TileElementsView<TileElement>(tile.toCoordsXY()))
        {
            if (!isObstacleType(element->getType()))
                continue;
            if (index >= kMaxElementsPerTile)
                break;
            addObstacles(_grid, state.ids, tile, element, index);
            index++;
        }
    }

    bool MapObstacles::query(const FixedVec3& position, Fixed radius, ObstacleContact& contact)
    {
        // Obstacles never extend beyond their own tile, so the tiles the ball touches are enough.
        const int32_t x0 = static_cast<int32_t>((position.x - radius).floor() >> 5);
        const int32_t x1 = static_cast<int32_t>((position.x + radius).floor() >> 5);
        const int32_t y0 = static_cast<int32_t>((position.y - radius).floor() >> 5);
        const int32_t y1 = static_cast<int32_t>((position.y + radius).floor() >> 5);
        for (int32_t ty = y0; ty <= y1; ty++)
            for (int32_t tx = x0; tx <= x1; tx++)
            {
                const TileCoordsXY tile{ tx, ty };
                if (tx < 0 || ty < 0 || !MapIsLocationValid(tile.toCoordsXY()))
                    continue;
                refreshTile(tile);
            }
        return _grid.query(position, radius, contact);
    }

    void MapObstacles::setDynamic(uint32_t entityId, Obstacle obstacle)
    {
        obstacle.id = kDynamicBase + entityId;
        _grid.add(obstacle);
    }

    void MapObstacles::removeDynamic(uint32_t entityId)
    {
        _grid.remove(kDynamicBase + entityId);
    }

    void MapObstacles::reset()
    {
        _grid.clear();
        _tiles.clear();
        _rebuilds = 0;
    }

    MapObstacles& GetMapObstacles()
    {
        static MapObstacles obstacles;
        return obstacles;
    }
} // namespace OpenRCT2::Golf
