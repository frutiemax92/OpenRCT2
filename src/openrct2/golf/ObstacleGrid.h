/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../object/ObjectCollision.h"
#include "BallPhysics.h"

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace OpenRCT2::Golf
{
    // Broad phase of the obstacles a ball can hit (BAL-035): a grid with one cell per tile, kept up to date
    // incrementally as obstacles are placed and removed. Queries only look at the cells the ball touches.

    enum class ObstacleShape : uint8_t
    {
        box,
        cylinder,
    };

    struct Obstacle
    {
        uint32_t id = 0;                   // unique and non-zero
        ObstacleShape shape = ObstacleShape::box;
        FixedVec3 centre{};                // centre of the base (box and cylinder), in world units
        Fixed halfX{};                     // box half extents; the cylinder uses halfX as its radius
        Fixed halfY{};
        Fixed height{};                    // above the base
        Fixed restitution = Fixed::ratio(4, 10);
        Fixed friction = Fixed::ratio(3, 10);
        Fixed passThroughChance{};
        bool isGolfer = false;
        uint32_t golferId = 0;
    };

    // The obstacle for an object with a collision block (spec 5.3) placed with the centre of its base at `base`.
    // `rotation` (0-3) swaps the footprint axes for quarter turns. A shape of "none" gives an obstacle that is not
    // valid (id 0), which the grid ignores.
    Obstacle makeObstacle(uint32_t id, const ObjectCollision& collision, const FixedVec3& base, int32_t rotation = 0);

    class ObstacleGrid
    {
    public:
        static constexpr int32_t kTileSize = 32;

        // Adds (or replaces) an obstacle. An obstacle with id 0 or no size is ignored and false is returned.
        bool add(const Obstacle& obstacle);
        bool remove(uint32_t id);
        // Moves an obstacle, e.g. a golfer who walks. Returns false for unknown ids.
        bool move(uint32_t id, const FixedVec3& newCentre);
        void clear();

        const Obstacle* find(uint32_t id) const;
        size_t size() const
        {
            return _obstacles.size();
        }

        // Contact of a sphere with a single obstacle, without a grid (for obstacles that are computed on the fly).
        static bool contactWith(const Obstacle& o, const FixedVec3& p, Fixed radius, ObstacleContact& out);

        // Deepest overlap of a ball (a sphere) with any obstacle.
        bool query(const FixedVec3& position, Fixed radius, ObstacleContact& contact) const;
        // Number of obstacles examined by the last query, to show that queries stay local.
        size_t lastQueryExamined() const
        {
            return _lastExamined;
        }

    private:
        static int64_t cellKey(int32_t cellX, int32_t cellY);
        void insertIntoCells(const Obstacle& obstacle);
        void removeFromCells(const Obstacle& obstacle);

        std::unordered_map<uint32_t, Obstacle> _obstacles;
        std::unordered_map<int64_t, std::vector<uint32_t>> _cells;
        mutable size_t _lastExamined = 0;
    };
} // namespace OpenRCT2::Golf
