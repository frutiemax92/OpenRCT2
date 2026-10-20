/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "GolferSkills.h"

#include <algorithm>

namespace OpenRCT2::Golf
{
    namespace
    {
        int32_t clampPoints(int32_t v)
        {
            return std::clamp(v, 0, kSkillMax256);
        }

        // Gain with diminishing returns: the nearer to 100, the smaller the step.
        int32_t gain(int32_t current, int32_t amount256, int32_t units)
        {
            if (units <= 0)
                return current;
            int64_t value = current;
            for (int32_t i = 0; i < units; i++)
            {
                const int64_t room = kSkillMax256 - value;
                if (room <= 0)
                    break;
                // The step scales with the room left, with a rounded result so slow gains still move.
                value += (static_cast<int64_t>(amount256) * room + kSkillMax256 / 2) / kSkillMax256;
            }
            return clampPoints(static_cast<int32_t>(value));
        }

        int32_t decayOne(int32_t current, int32_t initial, int32_t days)
        {
            const int32_t floorValue = std::max(0, initial - kDecayFloor256);
            if (current <= floorValue || days <= 0)
                return current;
            const int64_t decayed = static_cast<int64_t>(current) - static_cast<int64_t>(kDecayPerDay256) * days;
            return std::max(floorValue, static_cast<int32_t>(std::max<int64_t>(decayed, floorValue)));
        }
    } // namespace

    SkillPoints SkillPoints::from(const GolferSkills& skills)
    {
        return { skills.longPlay * 256, skills.accuracy * 256, skills.foolproof * 256, skills.physique * 256 };
    }

    GolferSkills SkillPoints::toSkills() const
    {
        auto toByte = [](int32_t points) { return static_cast<uint8_t>(std::clamp(points >> 8, 0, 100)); };
        return { toByte(longPlay), toByte(accuracy), toByte(foolproof), toByte(physique) };
    }

    void gainSkills(SkillPoints& skills, Activity activity, int32_t units)
    {
        switch (activity)
        {
            case Activity::drivingRangeBall:
                skills.longPlay = gain(skills.longPlay, kRangeLongPlayGain256, units);
                break;
            case Activity::miniPuttHole:
                skills.accuracy = gain(skills.accuracy, kMiniPuttAccuracyGain256, units);
                break;
            case Activity::courseHole:
                skills.longPlay = gain(skills.longPlay, kCourseLongPlayGain256, units);
                skills.accuracy = gain(skills.accuracy, kCourseAccuracyGain256, units);
                skills.foolproof = gain(skills.foolproof, kCourseFoolproofGain256, units);
                skills.physique = gain(skills.physique, kCoursePhysiqueGain256, units);
                break;
        }
    }

    void decaySkills(SkillPoints& skills, const SkillPoints& initial, int32_t days)
    {
        skills.longPlay = decayOne(skills.longPlay, initial.longPlay, days);
        skills.accuracy = decayOne(skills.accuracy, initial.accuracy, days);
        skills.foolproof = decayOne(skills.foolproof, initial.foolproof, days);
        skills.physique = decayOne(skills.physique, initial.physique, days);
    }

    void GolferRegistry::reindex()
    {
        _index.clear();
        for (size_t i = 0; i < _entries.size(); i++)
            _index[_entries[i].id] = i;
    }

    void GolferRegistry::evictIfNeeded()
    {
        while (_entries.size() > _capacity && !_entries.empty())
        {
            auto oldest = std::min_element(
                _entries.begin(), _entries.end(), [](const RegistryEntry& a, const RegistryEntry& b) { return a.touch < b.touch; });
            _entries.erase(oldest);
            reindex();
        }
    }

    RegistryEntry& GolferRegistry::remember(const RegistryEntry& entry)
    {
        auto it = _index.find(entry.id);
        if (it != _index.end())
        {
            auto& existing = _entries[it->second];
            existing = entry;
            existing.touch = ++_clock;
            return existing;
        }
        _entries.push_back(entry);
        _entries.back().touch = ++_clock;
        const uint32_t id = entry.id;
        reindex();
        evictIfNeeded();
        // The new entry is the most recently touched, so it is never the one evicted (unless the capacity is zero).
        auto found = _index.find(id);
        static RegistryEntry dummy;
        return found != _index.end() ? _entries[found->second] : dummy;
    }

    RegistryEntry* GolferRegistry::find(uint32_t id)
    {
        auto it = _index.find(id);
        return it != _index.end() ? &_entries[it->second] : nullptr;
    }

    const RegistryEntry* GolferRegistry::find(uint32_t id) const
    {
        auto it = _index.find(id);
        return it != _index.end() ? &_entries[it->second] : nullptr;
    }

    bool GolferRegistry::forget(uint32_t id)
    {
        auto it = _index.find(id);
        if (it == _index.end())
            return false;
        _entries.erase(_entries.begin() + static_cast<std::ptrdiff_t>(it->second));
        reindex();
        return true;
    }

    bool GolferRegistry::recordVisit(uint32_t id, uint32_t day, uint8_t happiness)
    {
        auto* entry = find(id);
        if (entry == nullptr)
            return false;
        entry->lastSeenDay = day;
        entry->lastHappiness = happiness;
        entry->visits++;
        entry->touch = ++_clock;
        return true;
    }

    void GolferRegistry::decayTo(uint32_t today)
    {
        for (auto& entry : _entries)
        {
            if (today <= entry.lastSeenDay)
                continue;
            decaySkills(entry.skills, entry.initial, static_cast<int32_t>(std::min<uint32_t>(today - entry.lastSeenDay, 100000)));
            entry.lastSeenDay = today; // the decay of the days up to today is applied once
        }
    }

    const RegistryEntry* GolferRegistry::pickReturning(IBallRandom& rng, const std::vector<uint32_t>& excluded) const
    {
        int64_t total = 0;
        auto allowed = [&](const RegistryEntry& e) { return std::find(excluded.begin(), excluded.end(), e.id) == excluded.end(); };
        for (const auto& e : _entries)
            if (allowed(e))
                total += e.lastHappiness + kHappinessBias;
        if (total <= 0)
            return nullptr;
        int64_t roll = static_cast<int64_t>(rng.next() % static_cast<uint64_t>(total));
        for (const auto& e : _entries)
        {
            if (!allowed(e))
                continue;
            roll -= e.lastHappiness + kHappinessBias;
            if (roll < 0)
                return &e;
        }
        return nullptr;
    }

    void GolferRegistry::setCapacity(size_t capacity)
    {
        _capacity = capacity;
        evictIfNeeded();
    }

    void GolferRegistry::clear()
    {
        _entries.clear();
        _index.clear();
    }
} // namespace OpenRCT2::Golf
