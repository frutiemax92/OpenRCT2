/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../world/Location.hpp"
#include "../world/TerrainShape.h"
#include "../world/Weather.h"
#include "BallPhysics.h"
#include "MapObstacles.h"

#include <functional>

namespace OpenRCT2::Golf
{
    // The ball physics world backed by the map (BAL-032, BAL-040): the terrain triangles of section 6.2 give the
    // height and the normal under the ball, the surface objects its roughness, bounciness and lie.

    // Height (z units) of the terrain surface at a position inside a tile, from the corner heights and the split
    // diagonal (TER-013). u and v are the position inside the tile in world units, 0 up to 32. Exact in fixed point and
    // continuous across tile edges, unlike the integer GetSurfaceZAt.
    Fixed terrainHeightAt(int32_t baseZ, const Terrain::TerrainShape& shape, Fixed u, Fixed v);
    // Unit normal of the triangle under the position.
    FixedVec3 terrainNormalAt(const Terrain::TerrainShape& shape, Fixed u, Fixed v);

    // Distance from a point to a stream (STR-001) lying in a tile: the segments from the tile centre to the midpoint of
    // each connected edge, or a disc at the centre when no edge is connected. u and v are the position in the tile.
    Fixed distanceToStream(uint8_t edges, Fixed u, Fixed v);

    // The wind as the velocity of the air in world units per tick (BAL-031).
    FixedVec3 windVelocity(const Weather::WindState& wind);

    // True when the tile is owned land, the default extent of the course (HOL-007).
    bool isTileOwned(const TileCoordsXY& tile);
    // Whether a world position is beyond the boundary of the courses; false when the park has no course.
    bool isOutOfCourseBounds(const CoordsXY& position);

    // Radius of the flag pin at the cup in world units, and its height above the green (HOL-003).
    constexpr int32_t kPinRadius = 1;
    constexpr int32_t kPinHeight = 24;

    class MapBallWorld final : public IBallWorld
    {
    public:
        // True for positions that are out of bounds beyond the edge of the map (the course boundary of section 16 is set
        // through this predicate).
        using BoundsFn = std::function<bool(const CoordsXY&)>;

        // The surface the ball interacts with: roughness 0.10 and bounciness 0.55 on paths and streets.
        static constexpr int32_t kPathRoughnessPercent = 10;
        static constexpr int32_t kPathBouncinessPercent = 55;
        // A path counts as the ground under the ball when it lies within this many z units of the terrain.
        static constexpr int32_t kPathHeightTolerance = 16;

        SurfaceSample surfaceAt(Fixed x, Fixed y) const override;
        bool obstacleAt(const FixedVec3& position, Fixed radius, ObstacleContact& contact) const override;
        bool cupNear(Fixed x, Fixed y, CupInfo& cup) const override;
        FixedVec3 wind() const override
        {
            return _wind;
        }

        void setOutOfBounds(BoundsFn fn)
        {
            _outOfBounds = std::move(fn);
        }
        // The obstacles the ball can hit; none when unset (BAL-034).
        void setObstacles(MapObstacles* obstacles)
        {
            _obstacles = obstacles;
        }
        void setWind(const FixedVec3& wind)
        {
            _wind = wind;
        }
        // Uses the wind of the game state, which follows the weather.
        void useGameWind();
        // Uses the course boundary of the game state for out of bounds, and the cups of its holes.
        void useCourses()
        {
            _useCourses = true;
            _outOfBounds = isOutOfCourseBounds;
        }

    private:
        BoundsFn _outOfBounds;
        MapObstacles* _obstacles = nullptr;
        bool _useCourses = false;
        FixedVec3 _wind{};
    };
} // namespace OpenRCT2::Golf
