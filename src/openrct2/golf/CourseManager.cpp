/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "CourseManager.h"

#include <algorithm>

namespace OpenRCT2::Golf
{
    void CourseBoundary::setRegion(
        const TileCoordsXY& min, const TileCoordsXY& max, bool inPlay, const TileCoordsXY& mapSize, bool initiallyInPlay)
    {
        if (mapSize.x <= 0 || mapSize.y <= 0)
            return;
        if (!_defined || _size.x != mapSize.x || _size.y != mapSize.y)
        {
            _size = mapSize;
            _bits.assign(static_cast<size_t>(mapSize.x) * static_cast<size_t>(mapSize.y), initiallyInPlay ? 1 : 0);
            _defined = true;
        }
        const int32_t x0 = std::clamp(std::min(min.x, max.x), 0, mapSize.x - 1);
        const int32_t x1 = std::clamp(std::max(min.x, max.x), 0, mapSize.x - 1);
        const int32_t y0 = std::clamp(std::min(min.y, max.y), 0, mapSize.y - 1);
        const int32_t y1 = std::clamp(std::max(min.y, max.y), 0, mapSize.y - 1);
        for (int32_t y = y0; y <= y1; y++)
            for (int32_t x = x0; x <= x1; x++)
                _bits[static_cast<size_t>(y) * static_cast<size_t>(mapSize.x) + static_cast<size_t>(x)] = inPlay ? 1 : 0;
    }

    bool CourseBoundary::isInPlay(const TileCoordsXY& tile) const
    {
        if (!_defined)
            return true; // the caller applies the default of owned land
        if (tile.x < 0 || tile.y < 0 || tile.x >= _size.x || tile.y >= _size.y)
            return false;
        return _bits[static_cast<size_t>(tile.y) * static_cast<size_t>(_size.x) + static_cast<size_t>(tile.x)] != 0;
    }

    void CourseBoundary::clear()
    {
        _defined = false;
        _size = {};
        _bits.clear();
    }

    void CourseBoundary::restore(const TileCoordsXY& size, std::vector<uint8_t> bits)
    {
        if (size.x <= 0 || size.y <= 0 || bits.size() != static_cast<size_t>(size.x) * static_cast<size_t>(size.y))
        {
            clear();
            return;
        }
        _defined = true;
        _size = size;
        _bits = std::move(bits);
    }

    Course* CourseManager::createCourse(const std::string& name)
    {
        if (_courses.size() >= kMaxCourses)
            return nullptr;
        Course course;
        course.id = _nextCourseId++;
        course.name = name;
        _courses.push_back(std::move(course));
        return &_courses.back();
    }

    bool CourseManager::removeCourse(uint32_t courseId)
    {
        auto it = std::find_if(_courses.begin(), _courses.end(), [&](const Course& c) { return c.id == courseId; });
        if (it == _courses.end())
            return false;
        _courses.erase(it);
        return true;
    }

    Course* CourseManager::findCourse(uint32_t courseId)
    {
        for (auto& course : _courses)
            if (course.id == courseId)
                return &course;
        return nullptr;
    }

    const Course* CourseManager::findCourse(uint32_t courseId) const
    {
        for (const auto& course : _courses)
            if (course.id == courseId)
                return &course;
        return nullptr;
    }

    Hole* CourseManager::createHole(uint32_t courseId)
    {
        auto* course = findCourse(courseId);
        if (course == nullptr || course->isFull())
            return nullptr;
        Hole hole;
        hole.id = _nextHoleId++;
        course->holes.push_back(hole);
        return &course->holes.back();
    }

    bool CourseManager::removeHole(uint32_t courseId, uint32_t holeId)
    {
        auto* course = findCourse(courseId);
        if (course == nullptr)
            return false;
        auto it = std::find_if(course->holes.begin(), course->holes.end(), [&](const Hole& h) { return h.id == holeId; });
        if (it == course->holes.end())
            return false;
        course->holes.erase(it);
        return true;
    }

    CourseManager::HoleRef CourseManager::findHole(uint32_t holeId)
    {
        for (auto& course : _courses)
            if (auto* hole = course.findHole(holeId))
                return { &course, hole };
        return {};
    }

    DrivingRange* CourseManager::createRange(const TileCoordsXY& cornerA, const TileCoordsXY& cornerB)
    {
        if (_ranges.size() >= kMaxRanges)
            return nullptr;
        DrivingRange range;
        range.id = _nextRangeId++;
        range.areaMin = { std::min(cornerA.x, cornerB.x), std::min(cornerA.y, cornerB.y) };
        range.areaMax = { std::max(cornerA.x, cornerB.x), std::max(cornerA.y, cornerB.y) };
        _ranges.push_back(std::move(range));
        return &_ranges.back();
    }

    bool CourseManager::removeRange(uint32_t rangeId)
    {
        const auto it = std::find_if(_ranges.begin(), _ranges.end(), [rangeId](const DrivingRange& r) { return r.id == rangeId; });
        if (it == _ranges.end())
            return false;
        _ranges.erase(it);
        return true;
    }

    DrivingRange* CourseManager::findRange(uint32_t rangeId)
    {
        for (auto& range : _ranges)
            if (range.id == rangeId)
                return &range;
        return nullptr;
    }

    const DrivingRange* CourseManager::findRange(uint32_t rangeId) const
    {
        return const_cast<CourseManager*>(this)->findRange(rangeId);
    }

    CottageLot* CourseManager::createLot(const TileCoordsXY& cornerA, const TileCoordsXY& cornerB)
    {
        if (_lots.size() >= kMaxLots)
            return nullptr;
        CottageLot lot;
        lot.id = _nextZoneId++;
        lot.min = { std::min(cornerA.x, cornerB.x), std::min(cornerA.y, cornerB.y) };
        lot.max = { std::max(cornerA.x, cornerB.x), std::max(cornerA.y, cornerB.y) };
        _lots.push_back(lot);
        return &_lots.back();
    }

    bool CourseManager::removeLot(uint32_t lotId)
    {
        const auto it = std::find_if(_lots.begin(), _lots.end(), [lotId](const CottageLot& l) { return l.id == lotId; });
        if (it == _lots.end())
            return false;
        _lots.erase(it);
        return true;
    }

    CottageLot* CourseManager::findLot(uint32_t lotId)
    {
        for (auto& lot : _lots)
            if (lot.id == lotId)
                return &lot;
        return nullptr;
    }

    const CottageLot* CourseManager::findLot(uint32_t lotId) const
    {
        return const_cast<CourseManager*>(this)->findLot(lotId);
    }

    TournamentZone* CourseManager::createTournamentZone(
        TournamentZoneKind kind, const TileCoordsXY& cornerA, const TileCoordsXY& cornerB)
    {
        if (_tournamentZones.size() >= kMaxTournamentZones)
            return nullptr;
        TournamentZone zone;
        zone.id = _nextZoneId++;
        zone.kind = kind;
        zone.min = { std::min(cornerA.x, cornerB.x), std::min(cornerA.y, cornerB.y) };
        zone.max = { std::max(cornerA.x, cornerB.x), std::max(cornerA.y, cornerB.y) };
        _tournamentZones.push_back(zone);
        return &_tournamentZones.back();
    }

    bool CourseManager::removeTournamentZone(uint32_t zoneId)
    {
        const auto it = std::find_if(
            _tournamentZones.begin(), _tournamentZones.end(), [zoneId](const TournamentZone& z) { return z.id == zoneId; });
        if (it == _tournamentZones.end())
            return false;
        _tournamentZones.erase(it);
        return true;
    }

    TournamentZone* CourseManager::findTournamentZone(uint32_t zoneId)
    {
        for (auto& zone : _tournamentZones)
            if (zone.id == zoneId)
                return &zone;
        return nullptr;
    }

    int32_t CourseManager::tournamentZoneTiles(TournamentZoneKind kind) const
    {
        int32_t total = 0;
        for (const auto& zone : _tournamentZones)
            if (zone.kind == kind)
                total += zone.tiles();
        return total;
    }

    void CourseManager::clear()
    {
        _lots.clear();
        _tournamentZones.clear();
        _nextZoneId = 1;
        _ranges.clear();
        _nextRangeId = 1;
        _courses.clear();
        _boundary.clear();
        _nextCourseId = 1;
        _nextHoleId = 1;
    }
} // namespace OpenRCT2::Golf
