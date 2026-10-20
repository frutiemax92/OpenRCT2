/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "ObstacleGrid.h"

#include <algorithm>
#include <cmath>

namespace OpenRCT2::Golf
{
    namespace
    {
        int32_t cellOf(Fixed v)
        {
            return static_cast<int32_t>(v.floor() >> 5); // 32 units per tile, floor division
        }

        struct Extent
        {
            int32_t x0, y0, x1, y1;
        };

        Extent extentOf(const Obstacle& o)
        {
            return { cellOf(o.centre.x - o.halfX), cellOf(o.centre.y - o.halfY), cellOf(o.centre.x + o.halfX),
                     cellOf(o.centre.y + o.halfY) };
        }
    } // namespace

    Obstacle makeObstacle(uint32_t id, const ObjectCollision& collision, const FixedVec3& base, int32_t rotation)
    {
        Obstacle o;
        if (collision.shape == CollisionShape::none)
            return o;
        // Object data is read as floats once at load time; converting to fixed-point here keeps the simulation integer.
        auto fixedOf = [](float v) { return Fixed::fromRaw(static_cast<int64_t>(std::lround(static_cast<double>(v) * Fixed::kOne))); };
        o.id = id;
        o.centre = base;
        o.height = Fixed::fromInt(collision.height);
        o.restitution = clamp(fixedOf(collision.restitution), Fixed{}, Fixed::fromInt(1));
        o.friction = clamp(fixedOf(collision.friction), Fixed{}, Fixed::fromInt(1));
        o.passThroughChance = clamp(fixedOf(collision.passThroughChance), Fixed{}, Fixed::fromInt(1));
        if (collision.shape == CollisionShape::cylinder)
        {
            o.shape = ObstacleShape::cylinder;
            o.halfX = Fixed::fromInt(collision.radius);
            o.halfY = o.halfX;
        }
        else
        {
            o.shape = ObstacleShape::box;
            const bool swap = (rotation & 1) != 0;
            o.halfX = Fixed::ratio(swap ? collision.footprintY : collision.footprintX, 2);
            o.halfY = Fixed::ratio(swap ? collision.footprintX : collision.footprintY, 2);
        }
        return o;
    }

    int64_t ObstacleGrid::cellKey(int32_t cellX, int32_t cellY)
    {
        return (static_cast<int64_t>(cellX) << 32) ^ static_cast<int64_t>(static_cast<uint32_t>(cellY));
    }

    void ObstacleGrid::insertIntoCells(const Obstacle& o)
    {
        const auto e = extentOf(o);
        for (int32_t y = e.y0; y <= e.y1; y++)
            for (int32_t x = e.x0; x <= e.x1; x++)
                _cells[cellKey(x, y)].push_back(o.id);
    }

    void ObstacleGrid::removeFromCells(const Obstacle& o)
    {
        const auto e = extentOf(o);
        for (int32_t y = e.y0; y <= e.y1; y++)
            for (int32_t x = e.x0; x <= e.x1; x++)
            {
                auto it = _cells.find(cellKey(x, y));
                if (it == _cells.end())
                    continue;
                auto& ids = it->second;
                ids.erase(std::remove(ids.begin(), ids.end(), o.id), ids.end());
                if (ids.empty())
                    _cells.erase(it);
            }
    }

    bool ObstacleGrid::add(const Obstacle& obstacle)
    {
        if (obstacle.id == 0 || obstacle.halfX <= Fixed{} || obstacle.height <= Fixed{}
            || (obstacle.shape == ObstacleShape::box && obstacle.halfY <= Fixed{}))
            return false;
        remove(obstacle.id);
        Obstacle stored = obstacle;
        if (stored.shape == ObstacleShape::cylinder)
            stored.halfY = stored.halfX; // the radius, so the cell extent is a square
        _obstacles[stored.id] = stored;
        insertIntoCells(stored);
        return true;
    }

    bool ObstacleGrid::remove(uint32_t id)
    {
        auto it = _obstacles.find(id);
        if (it == _obstacles.end())
            return false;
        removeFromCells(it->second);
        _obstacles.erase(it);
        return true;
    }

    bool ObstacleGrid::move(uint32_t id, const FixedVec3& newCentre)
    {
        auto it = _obstacles.find(id);
        if (it == _obstacles.end())
            return false;
        const auto before = extentOf(it->second);
        Obstacle moved = it->second;
        moved.centre = newCentre;
        const auto after = extentOf(moved);
        if (before.x0 == after.x0 && before.y0 == after.y0 && before.x1 == after.x1 && before.y1 == after.y1)
        {
            it->second = moved; // same cells: nothing to update in the grid
            return true;
        }
        removeFromCells(it->second);
        it->second = moved;
        insertIntoCells(moved);
        return true;
    }

    void ObstacleGrid::clear()
    {
        _obstacles.clear();
        _cells.clear();
    }

    const Obstacle* ObstacleGrid::find(uint32_t id) const
    {
        auto it = _obstacles.find(id);
        return it != _obstacles.end() ? &it->second : nullptr;
    }

    bool ObstacleGrid::contactWith(const Obstacle& o, const FixedVec3& p, Fixed radius, ObstacleContact& out)
    {
        const Fixed zMin = o.centre.z;
        const Fixed zMax = o.centre.z + o.height;
        FixedVec3 closest;
        FixedVec3 normal;
        Fixed depth;

        if (o.shape == ObstacleShape::box)
        {
            const FixedVec3 lo{ o.centre.x - o.halfX, o.centre.y - o.halfY, zMin };
            const FixedVec3 hi{ o.centre.x + o.halfX, o.centre.y + o.halfY, zMax };
            closest = { clamp(p.x, lo.x, hi.x), clamp(p.y, lo.y, hi.y), clamp(p.z, lo.z, hi.z) };
            const FixedVec3 d = p - closest;
            const Fixed dist = length(d);
            if (dist > radius)
                return false;
            if (dist > Fixed{})
            {
                normal = { d.x / dist, d.y / dist, d.z / dist };
                depth = radius - dist;
            }
            else
            {
                // The centre is inside the box: leave through the nearest face.
                const Fixed pen[6] = { p.x - lo.x, hi.x - p.x, p.y - lo.y, hi.y - p.y, p.z - lo.z, hi.z - p.z };
                const FixedVec3 faces[6] = { { -Fixed::fromInt(1), {}, {} }, { Fixed::fromInt(1), {}, {} },
                                             { {}, -Fixed::fromInt(1), {} }, { {}, Fixed::fromInt(1), {} },
                                             { {}, {}, -Fixed::fromInt(1) }, { {}, {}, Fixed::fromInt(1) } };
                int best = 0;
                for (int i = 1; i < 6; i++)
                    if (pen[i] < pen[best])
                        best = i;
                normal = faces[best];
                depth = pen[best] + radius;
            }
        }
        else
        {
            const Fixed dx = p.x - o.centre.x;
            const Fixed dy = p.y - o.centre.y;
            const Fixed horizontal = length(FixedVec3{ dx, dy, Fixed{} });
            const Fixed qz = clamp(p.z, zMin, zMax);
            Fixed qx = p.x, qy = p.y;
            if (horizontal > o.halfX)
            {
                qx = o.centre.x + dx * o.halfX / horizontal;
                qy = o.centre.y + dy * o.halfX / horizontal;
            }
            const FixedVec3 d{ p.x - qx, p.y - qy, p.z - qz };
            const Fixed dist = length(d);
            if (dist > radius)
                return false;
            if (dist > Fixed{})
            {
                normal = { d.x / dist, d.y / dist, d.z / dist };
                depth = radius - dist;
            }
            else
            {
                const Fixed side = o.halfX - horizontal;
                const Fixed bottom = p.z - zMin;
                const Fixed top = zMax - p.z;
                if (top <= side && top <= bottom)
                {
                    normal = { {}, {}, Fixed::fromInt(1) };
                    depth = top + radius;
                }
                else if (bottom <= side)
                {
                    normal = { {}, {}, -Fixed::fromInt(1) };
                    depth = bottom + radius;
                }
                else
                {
                    normal = horizontal > Fixed{} ? FixedVec3{ dx / horizontal, dy / horizontal, Fixed{} }
                                                  : FixedVec3{ Fixed::fromInt(1), Fixed{}, Fixed{} };
                    depth = side + radius;
                }
            }
        }

        out.normal = normal;
        out.depth = depth;
        out.restitution = o.restitution;
        out.friction = o.friction;
        out.passThroughChance = o.passThroughChance;
        out.id = o.id;
        out.isGolfer = o.isGolfer;
        out.golferId = o.golferId;
        return true;
    }

    bool ObstacleGrid::query(const FixedVec3& position, Fixed radius, ObstacleContact& contact) const
    {
        _lastExamined = 0;
        bool found = false;
        const int32_t x0 = cellOf(position.x - radius), x1 = cellOf(position.x + radius);
        const int32_t y0 = cellOf(position.y - radius), y1 = cellOf(position.y + radius);
        for (int32_t y = y0; y <= y1; y++)
            for (int32_t x = x0; x <= x1; x++)
            {
                auto cell = _cells.find(cellKey(x, y));
                if (cell == _cells.end())
                    continue;
                for (auto id : cell->second)
                {
                    _lastExamined++;
                    const auto& o = _obstacles.at(id);
                    ObstacleContact candidate;
                    if (!contactWith(o, position, radius, candidate))
                        continue;
                    // Deepest overlap wins; ties go to the lower id so the result never depends on cell order.
                    if (!found || candidate.depth > contact.depth || (candidate.depth == contact.depth && candidate.id < contact.id))
                    {
                        contact = candidate;
                        found = true;
                    }
                }
            }
        return found;
    }
} // namespace OpenRCT2::Golf
