/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "Hole.h"

#include <cstdint>
#include <string>
#include <vector>

namespace OpenRCT2::Golf
{
    // The area in which the ball is in play (HOL-007). Until the player draws a boundary it is all owned land; once drawn
    // it is exactly the tiles the player marked.
    class CourseBoundary
    {
    public:
        bool defined() const
        {
            return _defined;
        }
        // Marks a rectangle of tiles (inclusive, in tile coordinates) in or out of play. The first call defines the
        // boundary, starting from `initiallyInPlay` for every tile outside the rectangle.
        void setRegion(const TileCoordsXY& min, const TileCoordsXY& max, bool inPlay, const TileCoordsXY& mapSize, bool initiallyInPlay);
        // Defines the boundary from a rule for each tile, e.g. "owned land": used the first time the player edits it, so
        // the edit starts from what the default was.
        template<typename TPredicate>
        void defineFrom(const TileCoordsXY& mapSize, TPredicate inPlay)
        {
            if (mapSize.x <= 0 || mapSize.y <= 0)
                return;
            _size = mapSize;
            _bits.assign(static_cast<size_t>(mapSize.x) * static_cast<size_t>(mapSize.y), 0);
            for (int32_t y = 0; y < mapSize.y; y++)
                for (int32_t x = 0; x < mapSize.x; x++)
                    _bits[static_cast<size_t>(y) * static_cast<size_t>(mapSize.x) + static_cast<size_t>(x)] = inPlay(TileCoordsXY{ x, y }) ? 1 : 0;
            _defined = true;
        }
        bool isInPlay(const TileCoordsXY& tile) const;
        // Back to the default (all owned land).
        void clear();

        const std::vector<uint8_t>& bits() const
        {
            return _bits;
        }
        const TileCoordsXY& size() const
        {
            return _size;
        }
        // Rebuilds the boundary from saved data.
        void restore(const TileCoordsXY& size, std::vector<uint8_t> bits);

    private:
        bool _defined = false;
        TileCoordsXY _size{};
        std::vector<uint8_t> _bits; // one byte per tile, row by row, 1 for in play
    };

    // All the courses of the park and the boundary they share (HOL-010: a map can host several courses).
    class CourseManager
    {
    public:
        static constexpr size_t kMaxCourses = 16;

        // Returns nullptr when the limit is reached.
        Course* createCourse(const std::string& name);
        bool removeCourse(uint32_t courseId);
        Course* findCourse(uint32_t courseId);
        const Course* findCourse(uint32_t courseId) const;
        std::vector<Course>& courses()
        {
            return _courses;
        }
        const std::vector<Course>& courses() const
        {
            return _courses;
        }

        // Adds an empty hole at the end of a course (HOL-004: numbered in creation order). Returns nullptr when the course
        // does not exist or already has 18 holes.
        Hole* createHole(uint32_t courseId);
        bool removeHole(uint32_t courseId, uint32_t holeId);
        // The course and hole holding a hole id, wherever it is.
        struct HoleRef
        {
            Course* course = nullptr;
            Hole* hole = nullptr;
        };
        HoleRef findHole(uint32_t holeId);

        CourseBoundary& boundary()
        {
            return _boundary;
        }
        const CourseBoundary& boundary() const
        {
            return _boundary;
        }

        void clear();

        uint32_t nextCourseId() const
        {
            return _nextCourseId;
        }
        uint32_t nextHoleId() const
        {
            return _nextHoleId;
        }
        // For loading saved data.
        void restoreIds(uint32_t nextCourse, uint32_t nextHole)
        {
            _nextCourseId = nextCourse;
            _nextHoleId = nextHole;
        }

    private:
        std::vector<Course> _courses;
        CourseBoundary _boundary;
        uint32_t _nextCourseId = 1;
        uint32_t _nextHoleId = 1;
    };
} // namespace OpenRCT2::Golf
