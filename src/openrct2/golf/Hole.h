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
#include "HoleScheduleData.h"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace OpenRCT2::Golf
{
    // Golf holes and courses (section 16.1, 16.2). A hole has up to three tee boxes, a cup, and up to two aim points
    // that bend the playing line around doglegs.

    enum class TeeCategory : uint8_t
    {
        men,
        women,
        child,
    };
    constexpr size_t kTeeCategoryCount = 3;
    constexpr size_t kMaxAimPoints = 2;
    constexpr size_t kMaxHolesPerCourse = 18;

    // HOL-006: par from the length in yards, one table per tee category in one place. A hole of up to `par3Max` yards is
    // par 3, up to `par4Max` par 4 and anything longer par 5.
    struct ParTable
    {
        int32_t par3Max;
        int32_t par4Max;
    };
    constexpr std::array<ParTable, kTeeCategoryCount> kParTables = { {
        { 250, 470 }, // men
        { 210, 400 }, // women
        { 150, 280 }, // children
    } };

    int32_t parForLength(int32_t yards, TeeCategory category = TeeCategory::men);

    // Position of something on the map, in world units; z is looked up from the terrain when it is needed.
    using HolePoint = CoordsXY;

    struct Hole
    {
        uint32_t id = 0; // unique across the park, stable when the holes are reordered
        std::array<std::optional<HolePoint>, kTeeCategoryCount> tees;
        std::optional<HolePoint> cup;
        std::array<std::optional<HolePoint>, kMaxAimPoints> aimPoints; // used in order: aim point 0 first
        int32_t parOverride = 0; // 0 for the computed par (men's tee)
        HoleSchedule schedule;   // the time slot and what groups really take (HOL-020)
        int32_t sceneryScore = 0; // 0..100: scenery near the hole (SCN-003)

        bool hasTee(TeeCategory category) const
        {
            return tees[static_cast<size_t>(category)].has_value();
        }
        // The first tee that exists, the men's tee first: where the construction flow starts.
        std::optional<TeeCategory> firstTee() const;
        // A hole needs at least one tee and one cup to be playable (HOL-001).
        bool isComplete() const
        {
            return cup.has_value() && firstTee().has_value();
        }

        // The playing line from a tee through the aim points to the cup, in order.
        std::vector<HolePoint> playingLine(TeeCategory category) const;
        // HOL-005: length of the playing line in yards (1 tile = 5 yards). Zero when the hole is incomplete.
        int32_t lengthYards(TeeCategory category) const;
        // HOL-006: par from the men's tee length, or from the first tee that exists; the override wins.
        int32_t par() const;
        // Par ignoring the override, for the UI to show what "auto" would give.
        int32_t computedPar() const;
    };

    // Straight distance in yards between two map positions, rounded to the nearest yard (1 tile = 5 yards).
    int32_t distanceYards(const HolePoint& a, const HolePoint& b);

    // MPT-001: a mini putt course is built like a regular one, but only the mini-putt club is used on it.
    enum class CourseKind : uint8_t
    {
        regular,
        miniPutt,
    };

    struct Course
    {
        uint32_t id = 0;
        CourseKind kind = CourseKind::regular;
        std::string name;
        int64_t greenFee = 0; // money64 in currency units
        bool open = false;
        bool cartsAllowedOffPath = false; // PTH-005: course setting (default off)
        std::vector<Hole> holes; // in playing order: hole 1 first (HOL-004, HOL-011)

        bool isFull() const
        {
            return holes.size() >= kMaxHolesPerCourse;
        }
        // Number of holes that can be played (complete holes only).
        size_t playableHoles() const;
        int32_t totalPar() const;
        int32_t totalYards(TeeCategory category) const;
        // Moves a hole to another position in the list; false for an invalid index.
        bool moveHole(size_t from, size_t to);
        const Hole* findHole(uint32_t holeId) const;
        Hole* findHole(uint32_t holeId);
        // 1-based number of a hole in this course, 0 when it is not in it.
        size_t holeNumber(uint32_t holeId) const;
    };
} // namespace OpenRCT2::Golf
