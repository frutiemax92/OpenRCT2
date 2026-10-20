/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Hole.h"

#include "../core/Fixed.hpp"

#include <algorithm>

namespace OpenRCT2::Golf
{
    int32_t parForLength(int32_t yards, TeeCategory category)
    {
        const auto& table = kParTables[std::min(static_cast<size_t>(category), kTeeCategoryCount - 1)];
        if (yards <= table.par3Max)
            return 3;
        if (yards <= table.par4Max)
            return 4;
        return 5;
    }

    int32_t distanceYards(const HolePoint& a, const HolePoint& b)
    {
        const int64_t dx = a.x - b.x;
        const int64_t dy = a.y - b.y;
        const auto units = static_cast<int64_t>(integerSqrt(static_cast<uint64_t>(dx * dx + dy * dy)));
        // 32 units per tile, 5 yards per tile.
        return static_cast<int32_t>((units * 5 + 16) / 32);
    }

    std::optional<TeeCategory> Hole::firstTee() const
    {
        for (size_t i = 0; i < kTeeCategoryCount; i++)
            if (tees[i].has_value())
                return static_cast<TeeCategory>(i);
        return std::nullopt;
    }

    std::vector<HolePoint> Hole::playingLine(TeeCategory category) const
    {
        std::vector<HolePoint> line;
        const auto& tee = tees[static_cast<size_t>(category)];
        if (!tee.has_value() || !cup.has_value())
            return line;
        line.push_back(*tee);
        for (const auto& aim : aimPoints)
            if (aim.has_value())
                line.push_back(*aim);
        line.push_back(*cup);
        return line;
    }

    int32_t Hole::lengthYards(TeeCategory category) const
    {
        const auto line = playingLine(category);
        if (line.size() < 2)
            return 0;
        // Sum the segments in units first and convert once, so rounding does not add up over the segments.
        int64_t units = 0;
        for (size_t i = 1; i < line.size(); i++)
        {
            const int64_t dx = line[i].x - line[i - 1].x;
            const int64_t dy = line[i].y - line[i - 1].y;
            units += static_cast<int64_t>(integerSqrt(static_cast<uint64_t>(dx * dx + dy * dy)));
        }
        return static_cast<int32_t>((units * 5 + 16) / 32);
    }

    int32_t Hole::computedPar() const
    {
        const auto tee = firstTee();
        if (!tee.has_value() || !cup.has_value())
            return 0;
        return parForLength(lengthYards(*tee), *tee);
    }

    int32_t Hole::par() const
    {
        return parOverride > 0 ? parOverride : computedPar();
    }

    size_t Course::playableHoles() const
    {
        return static_cast<size_t>(std::count_if(holes.begin(), holes.end(), [](const Hole& h) { return h.isComplete(); }));
    }

    int32_t Course::totalPar() const
    {
        int32_t total = 0;
        for (const auto& hole : holes)
            total += hole.par();
        return total;
    }

    int32_t Course::totalYards(TeeCategory category) const
    {
        int32_t total = 0;
        for (const auto& hole : holes)
            total += hole.lengthYards(category);
        return total;
    }

    bool Course::moveHole(size_t from, size_t to)
    {
        if (from >= holes.size() || to >= holes.size())
            return false;
        if (from == to)
            return true;
        auto hole = holes[from];
        holes.erase(holes.begin() + static_cast<std::ptrdiff_t>(from));
        holes.insert(holes.begin() + static_cast<std::ptrdiff_t>(to), std::move(hole));
        return true;
    }

    const Hole* Course::findHole(uint32_t holeId) const
    {
        for (const auto& hole : holes)
            if (hole.id == holeId)
                return &hole;
        return nullptr;
    }

    Hole* Course::findHole(uint32_t holeId)
    {
        for (auto& hole : holes)
            if (hole.id == holeId)
                return &hole;
        return nullptr;
    }

    size_t Course::holeNumber(uint32_t holeId) const
    {
        for (size_t i = 0; i < holes.size(); i++)
            if (holes[i].id == holeId)
                return i + 1;
        return 0;
    }
} // namespace OpenRCT2::Golf
