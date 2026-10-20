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
#include "ObstacleGrid.h"

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace OpenRCT2::Golf
{
    // The obstacles of the map (BAL-034, BAL-035): small and large scenery, walls and buildings, with the shapes of their
    // collision blocks (spec 5.3), plus dynamic obstacles such as golfers and vehicles. They live in an ObstacleGrid with
    // one cell per tile. A tile is rebuilt, and only that tile, when the elements on it changed since it was last looked
    // at: every query first compares a signature of the elements on the tiles it touches, so placement, removal, undo,
    // loading and scripting are all noticed without a hook in each of them.
    class MapObstacles
    {
    public:
        // Obstacle ids: map elements use 1 to kDynamicBase - 1, dynamic obstacles kDynamicBase and above.
        static constexpr uint32_t kDynamicBase = 0x80000000u;

        // Thickness in world units of the box a wall presents along its tile edge.
        static constexpr int32_t kWallThickness = 2;

        // Contact of a ball with the closest obstacle among the tiles it touches.
        bool query(const FixedVec3& position, Fixed radius, ObstacleContact& contact);

        // Dynamic obstacles (golfers, vehicles): their id is kDynamicBase + the entity id.
        void setDynamic(uint32_t entityId, Obstacle obstacle);
        void removeDynamic(uint32_t entityId);

        // Forgets everything; the next query rebuilds the tiles it needs.
        void reset();

        size_t obstacleCount() const
        {
            return _grid.size();
        }
        // Tiles rebuilt so far, to show that only changed tiles are rebuilt.
        size_t rebuildCount() const
        {
            return _rebuilds;
        }
        size_t lastQueryExamined() const
        {
            return _grid.lastQueryExamined();
        }

    private:
        struct TileState
        {
            uint64_t signature = 0;
            std::vector<uint32_t> ids;
        };

        void refreshTile(const TileCoordsXY& tile);

        ObstacleGrid _grid;
        std::unordered_map<int64_t, TileState> _tiles;
        size_t _rebuilds = 0;
    };

    // The singleton for the game map; reset when a park is loaded.
    MapObstacles& GetMapObstacles();
} // namespace OpenRCT2::Golf
