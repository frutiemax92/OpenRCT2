/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "HoleMarkers.h"

namespace OpenRCT2::Golf
{
    namespace
    {
        uint64_t mix(uint64_t hash, uint64_t value)
        {
            // FNV-1a over the 8 bytes of the value.
            for (int i = 0; i < 8; i++)
            {
                hash ^= (value >> (8 * i)) & 0xFF;
                hash *= 1099511628211ull;
            }
            return hash;
        }

        template<typename Fn>
        void forEachMarker(const CourseManager& courses, Fn&& fn)
        {
            for (const auto& course : courses.courses())
            {
                for (const auto& hole : course.holes)
                {
                    for (size_t i = 0; i < hole.tees.size(); i++)
                    {
                        if (hole.tees[i])
                            fn(HoleMarker{ HoleMarker::Kind::tee, static_cast<uint8_t>(i), hole.tees[i]->x, hole.tees[i]->y, hole.id });
                    }
                    if (hole.cup)
                        fn(HoleMarker{ HoleMarker::Kind::cup, 0, hole.cup->x, hole.cup->y, hole.id });
                    for (size_t i = 0; i < hole.aimPoints.size(); i++)
                    {
                        if (hole.aimPoints[i])
                        {
                            fn(HoleMarker{ HoleMarker::Kind::aim, static_cast<uint8_t>(i), hole.aimPoints[i]->x,
                                           hole.aimPoints[i]->y, hole.id });
                        }
                    }
                }
            }
            for (const auto& range : courses.ranges())
            {
                for (const auto& tile : range.hittingTiles)
                    fn(HoleMarker{ HoleMarker::Kind::hittingTile, 0, tile.x, tile.y, range.id });
                for (const auto& sign : range.signs)
                    fn(HoleMarker{ HoleMarker::Kind::distanceSign, 0, sign.position.x, sign.position.y, range.id, sign.distance });
            }
        }
    } // namespace

    void HoleMarkerIndex::update(const CourseManager& courses)
    {
        uint64_t fingerprint = 1469598103934665603ull;
        forEachMarker(courses, [&](const HoleMarker& m) {
            fingerprint = mix(fingerprint, (static_cast<uint64_t>(m.kind) << 8) | m.index | (static_cast<uint64_t>(static_cast<uint32_t>(m.value)) << 16));
            fingerprint = mix(fingerprint, (static_cast<uint64_t>(static_cast<uint32_t>(m.x)) << 32) | static_cast<uint32_t>(m.y));
        });
        if (_built && fingerprint == _fingerprint)
            return;

        _byTile.clear();
        _count = 0;
        forEachMarker(courses, [&](const HoleMarker& m) {
            _byTile[key(tileOf(m.x, m.y))].push_back(m);
            _count++;
        });
        _fingerprint = fingerprint;
        _built = true;
    }

    std::vector<int32_t> HoleMarkerIndex::signDistances() const
    {
        std::vector<int32_t> distances;
        for (const auto& [key, markers] : _byTile)
            for (const auto& marker : markers)
                if (marker.kind == HoleMarker::Kind::distanceSign)
                    distances.push_back(marker.value);
        return distances;
    }

    const std::vector<HoleMarker>* HoleMarkerIndex::at(const TileCoordsXY& tile) const
    {
        const auto it = _byTile.find(key(tile));
        return it == _byTile.end() ? nullptr : &it->second;
    }

    HoleMarkerIndex& GetHoleMarkers()
    {
        static HoleMarkerIndex index;
        return index;
    }
} // namespace OpenRCT2::Golf
