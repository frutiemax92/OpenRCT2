/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "MapBallWorld.h"

#include "../GameState.h"
#include "../object/StreamObject.h"
#include "../object/TerrainSurfaceObject.h"
#include "../world/Map.h"
#include "../world/MapOwnership.h"
#include "../world/TileElementsView.h"
#include "../world/tile_element/PathElement.h"
#include "../world/tile_element/StreamElement.h"
#include "../world/tile_element/SurfaceElement.h"

#include <cmath>

namespace OpenRCT2::Golf
{
    namespace
    {
        constexpr int32_t kTile = 32;

        Fixed fixedOf(float v)
        {
            return Fixed::fromRaw(static_cast<int64_t>(std::lround(static_cast<double>(v) * Fixed::kOne)));
        }

        struct Heights
        {
            Fixed s, e, n, w;
        };

        Heights cornerHeights(int32_t baseZ, const Terrain::TerrainShape& shape)
        {
            const auto c = Terrain::GetCornerHeights(baseZ, shape);
            return { Fixed::fromInt(c.south), Fixed::fromInt(c.east), Fixed::fromInt(c.north), Fixed::fromInt(c.west) };
        }

        bool inFirstTriangle(const Terrain::TerrainShape& shape, Fixed u, Fixed v)
        {
            if (Terrain::GetSplitDiagonal(shape) == Terrain::SplitDiagonal::northSouth)
                return u >= v;
            return u + v <= Fixed::fromInt(kTile);
        }
    } // namespace

    Fixed terrainHeightAt(int32_t baseZ, const Terrain::TerrainShape& shape, Fixed u, Fixed v)
    {
        const auto h = cornerHeights(baseZ, shape);
        const Fixed size = Fixed::fromInt(kTile);
        const bool northSouth = Terrain::GetSplitDiagonal(shape) == Terrain::SplitDiagonal::northSouth;
        if (northSouth)
        {
            if (u >= v)
                return h.s + (h.e - h.s) * u / size + (h.n - h.e) * v / size;
            return h.s + (h.n - h.w) * u / size + (h.w - h.s) * v / size;
        }
        if (u + v <= size)
            return h.s + (h.e - h.s) * u / size + (h.w - h.s) * v / size;
        return h.n + (h.w - h.n) * (size - u) / size + (h.e - h.n) * (size - v) / size;
    }

    FixedVec3 terrainNormalAt(const Terrain::TerrainShape& shape, Fixed u, Fixed v)
    {
        const auto h = cornerHeights(0, shape);
        const bool northSouth = Terrain::GetSplitDiagonal(shape) == Terrain::SplitDiagonal::northSouth;
        const bool first = inFirstTriangle(shape, u, v);
        // Gradients in z per tile width along x and y.
        Fixed gx, gy;
        if (northSouth)
        {
            if (first)
            {
                gx = h.e - h.s;
                gy = h.n - h.e;
            }
            else
            {
                gx = h.n - h.w;
                gy = h.w - h.s;
            }
        }
        else
        {
            if (first)
            {
                gx = h.e - h.s;
                gy = h.w - h.s;
            }
            else
            {
                gx = h.n - h.w;
                gy = h.n - h.e;
            }
        }
        const Fixed size = Fixed::fromInt(kTile);
        return normalised(FixedVec3{ -gx / size, -gy / size, Fixed::fromInt(1) });
    }

    Fixed distanceToStream(uint8_t edges, Fixed u, Fixed v)
    {
        const Fixed half = Fixed::fromInt(kTile / 2);
        const Fixed full = Fixed::fromInt(kTile);
        auto distanceToAxisSegment = [&](Fixed x0, Fixed y0, Fixed x1, Fixed y1) {
            const Fixed cx = clamp(u, min(x0, x1), max(x0, x1));
            const Fixed cy = clamp(v, min(y0, y1), max(y0, y1));
            return length(FixedVec3{ u - cx, v - cy, Fixed{} });
        };
        if ((edges & StreamElement::kEdgesMask) == 0)
            return length(FixedVec3{ u - half, v - half, Fixed{} });
        Fixed best = Fixed::fromInt(1000);
        // Direction 0 = -x, 1 = +y, 2 = +x, 3 = -y.
        if (edges & 1)
            best = min(best, distanceToAxisSegment(Fixed{}, half, half, half));
        if (edges & 2)
            best = min(best, distanceToAxisSegment(half, half, half, full));
        if (edges & 4)
            best = min(best, distanceToAxisSegment(half, half, full, half));
        if (edges & 8)
            best = min(best, distanceToAxisSegment(half, Fixed{}, half, half));
        return best;
    }

    FixedVec3 windVelocity(const Weather::WindState& wind)
    {
        const Fixed radians = Fixed::fromInt(wind.direction) * kFixedPi / 180;
        const Fixed speed = Fixed::ratio(Weather::kWindMaxSpeedHundredths, 100) * Fixed::ratio(wind.strength, Weather::kWindStrengthMax);
        return { cos(radians) * speed, sin(radians) * speed, Fixed{} };
    }

    bool isTileOwned(const TileCoordsXY& tile)
    {
        auto* surface = MapGetSurfaceElementAt(tile);
        return surface != nullptr && surface->hasOwnership(OwnershipFlag::landOwned);
    }

    bool isOutOfCourseBounds(const CoordsXY& position)
    {
        const auto& golf = getGameState().golf;
        if (golf.courses().empty())
            return false;
        const TileCoordsXY tile{ position.x >> 5, position.y >> 5 };
        if (golf.boundary().defined())
            return !golf.boundary().isInPlay(tile);
        return !isTileOwned(tile);
    }

    bool MapBallWorld::obstacleAt(const FixedVec3& position, Fixed radius, ObstacleContact& contact) const
    {
        bool found = _obstacles != nullptr && _obstacles->query(position, radius, contact);
        if (!_useCourses)
            return found;
        // The flag pin is a thin pole standing on the cup (HOL-003).
        for (const auto& course : getGameState().golf.courses())
        {
            for (const auto& hole : course.holes)
            {
                if (!hole.cup.has_value())
                    continue;
                if (abs(position.x - Fixed::fromInt(hole.cup->x)) > Fixed::fromInt(4)
                    || abs(position.y - Fixed::fromInt(hole.cup->y)) > Fixed::fromInt(4))
                    continue;
                Obstacle pin;
                pin.id = 0xFFFFFF00u + (hole.id & 0xFF);
                pin.shape = ObstacleShape::cylinder;
                const auto ground = surfaceAt(Fixed::fromInt(hole.cup->x), Fixed::fromInt(hole.cup->y));
                pin.centre = { Fixed::fromInt(hole.cup->x), Fixed::fromInt(hole.cup->y), ground.height };
                pin.halfX = Fixed::fromInt(kPinRadius);
                pin.halfY = pin.halfX;
                pin.height = Fixed::fromInt(kPinHeight);
                pin.restitution = Fixed::ratio(3, 10);
                pin.friction = Fixed::ratio(5, 10);
                ObstacleContact pinContact;
                if (ObstacleGrid::contactWith(pin, position, radius, pinContact) && (!found || pinContact.depth > contact.depth))
                {
                    contact = pinContact;
                    found = true;
                }
            }
        }
        return found;
    }

    bool MapBallWorld::cupNear(Fixed x, Fixed y, CupInfo& cup) const
    {
        if (!_useCourses)
            return false;
        for (const auto& course : getGameState().golf.courses())
        {
            for (const auto& hole : course.holes)
            {
                if (!hole.cup.has_value())
                    continue;
                const Fixed dx = x - Fixed::fromInt(hole.cup->x);
                const Fixed dy = y - Fixed::fromInt(hole.cup->y);
                if (abs(dx) > Fixed::fromInt(8) || abs(dy) > Fixed::fromInt(8))
                    continue;
                const auto ground = surfaceAt(Fixed::fromInt(hole.cup->x), Fixed::fromInt(hole.cup->y));
                cup.position = { Fixed::fromInt(hole.cup->x), Fixed::fromInt(hole.cup->y), ground.height };
                cup.radius = Fixed::fromInt(3);
                return true;
            }
        }
        return false;
    }

    void MapBallWorld::useGameWind()
    {
        _wind = windVelocity(getGameState().wind);
    }

    SurfaceSample MapBallWorld::surfaceAt(Fixed x, Fixed y) const
    {
        SurfaceSample s;
        const CoordsXY tile{ static_cast<int32_t>(x.floor()) & ~(kTile - 1), static_cast<int32_t>(y.floor()) & ~(kTile - 1) };
        // The outermost ring of tiles is the invisible border of the map: beyond the playable area (BAL-051).
        const auto& mapSize = getGameState().mapSize;
        if (x < Fixed::fromInt(kTile) || y < Fixed::fromInt(kTile) || tile.x >= (mapSize.x - 1) * kTile
            || tile.y >= (mapSize.y - 1) * kTile || !MapIsLocationValid(tile))
        {
            s.outOfBounds = true;
            return s;
        }
        auto* surface = MapGetSurfaceElementAt(tile);
        if (surface == nullptr)
        {
            s.outOfBounds = true;
            return s;
        }

        const Fixed u = x - Fixed::fromInt(tile.x);
        const Fixed v = y - Fixed::fromInt(tile.y);
        const auto shape = surface->getTerrainShape();
        const int32_t baseZ = surface->getBaseZ();
        s.height = terrainHeightAt(baseZ, shape, u, v);
        s.normal = terrainNormalAt(shape, u, v);
        if (_outOfBounds && _outOfBounds(CoordsXY{ static_cast<int32_t>(x.floor()), static_cast<int32_t>(y.floor()) }))
            s.outOfBounds = true;

        const int32_t xl = static_cast<int32_t>(u.floor());
        const int32_t yl = static_cast<int32_t>(v.floor());
        const auto quadrant = Terrain::GetQuadrantAt(xl, yl);
        s.surface = static_cast<uint8_t>(surface->getQuadrantSurfaceIndex(quadrant));
        if (const auto* object = TerrainSurfaceObject::GetById(surface->getQuadrantSurfaceIndex(quadrant)))
        {
            const auto& golf = object->Golf;
            s.roughness = fixedOf(golf.roughness);
            s.bounciness = fixedOf(golf.bounciness);
            s.distanceFactor = fixedOf(golf.distanceFactor);
            s.accuracyFactor = fixedOf(golf.accuracyFactor);
            s.hazard = golf.isHazard;
            s.puttable = golf.isPuttable;
        }

        const int32_t waterZ = surface->getWaterHeight();
        if (waterZ > 0 && s.height < Fixed::fromInt(waterZ))
        {
            s.water = true;
            s.height = Fixed::fromInt(waterZ);
            s.normal = { Fixed{}, Fixed{}, Fixed::fromInt(1) };
            return s;
        }

        for (auto* element : TileElementsView<StreamElement>(tile))
        {
            const auto* object = element->getObject();
            if (object == nullptr || !object->IsHazard)
                continue;
            if (abs(Fixed::fromInt(element->getBaseZ()) - s.height) > Fixed::fromInt(MapBallWorld::kPathHeightTolerance))
                continue;
            if (distanceToStream(element->getEdges(), u, v) <= Fixed::ratio(object->Width, 2))
            {
                s.water = true;
                s.stream = true;
                return s;
            }
        }

        for (auto* element : TileElementsView<PathElement>(tile))
        {
            if (abs(Fixed::fromInt(element->getBaseZ()) - s.height) > Fixed::fromInt(kPathHeightTolerance))
                continue;
            s.path = true;
            s.roughness = Fixed::ratio(kPathRoughnessPercent, 100);
            s.bounciness = Fixed::ratio(kPathBouncinessPercent, 100);
            s.hazard = false;
            s.puttable = false;
            break;
        }
        return s;
    }
} // namespace OpenRCT2::Golf
