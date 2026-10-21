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
#include "CourseManager.h"

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace OpenRCT2::Golf
{
    // What is drawn on the map for the holes of the courses (HOL-002, HOL-003): the tee markers in a colour per category,
    // the flag pin at the cup and, while holes are being built, the aim points. The markers are found by tile, so the
    // painter of a tile asks for the few that lie on it.
    struct HoleMarker
    {
        enum class Kind : uint8_t
        {
            tee,
            cup,
            aim,
        };
        Kind kind = Kind::tee;
        uint8_t index = 0;   // tee category or aim point number
        int32_t x = 0;       // map position in world units
        int32_t y = 0;
        uint32_t holeId = 0;
    };

    class HoleMarkerIndex
    {
    public:
        // Rebuilds the index from the courses when they changed since the last call. Call it once per frame on the main
        // thread, before painting starts; painting only reads.
        void update(const CourseManager& courses);

        // The markers on a tile, nullptr when there are none.
        const std::vector<HoleMarker>* at(const TileCoordsXY& tile) const;

        // Aim points are only shown while holes are being built.
        void setShowAimPoints(bool show)
        {
            _showAimPoints = show;
        }
        bool showAimPoints() const
        {
            return _showAimPoints;
        }
        size_t markerCount() const
        {
            return _count;
        }
        // The tile a position lies on.
        static TileCoordsXY tileOf(int32_t x, int32_t y)
        {
            return TileCoordsXY{ x >> 5, y >> 5 };
        }

    private:
        static uint64_t key(const TileCoordsXY& tile)
        {
            return (static_cast<uint64_t>(static_cast<uint32_t>(tile.y)) << 32) | static_cast<uint32_t>(tile.x);
        }

        std::unordered_map<uint64_t, std::vector<HoleMarker>> _byTile;
        uint64_t _fingerprint = 0;
        bool _built = false;
        bool _showAimPoints = false;
        size_t _count = 0;
    };

    HoleMarkerIndex& GetHoleMarkers();
} // namespace OpenRCT2::Golf
