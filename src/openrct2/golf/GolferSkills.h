/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "BallPhysics.h"
#include "Shot.h"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace OpenRCT2::Golf
{
    // Skill gain and decay (GLF-021, GLF-022) and the registry of returning golfers (GLF-023).

    // Skills with 8 fractional bits, so slow gain and decay add up: the skill is points >> 8.
    struct SkillPoints
    {
        int32_t longPlay = 50 * 256;
        int32_t accuracy = 50 * 256;
        int32_t foolproof = 50 * 256;
        int32_t physique = 50 * 256;

        static SkillPoints from(const GolferSkills& skills);
        GolferSkills toSkills() const;
        bool operator==(const SkillPoints&) const = default;
    };

    enum class Activity : uint8_t
    {
        drivingRangeBall, // GLF-021: Long play
        miniPuttHole,     // Accuracy
        courseHole,       // all four skills
    };

    // Gain per unit of activity at skill 0, in skill points (not fractions); it shrinks towards 100 (GLF-021).
    constexpr int32_t kRangeLongPlayGain256 = 154;    // 0.6 per bucket ball
    constexpr int32_t kMiniPuttAccuracyGain256 = 384; // 1.5 per hole
    constexpr int32_t kCourseLongPlayGain256 = 64;    // 0.25 per hole
    constexpr int32_t kCourseAccuracyGain256 = 64;
    constexpr int32_t kCourseFoolproofGain256 = 51;   // 0.2
    constexpr int32_t kCoursePhysiqueGain256 = 26;    // 0.1

    constexpr int32_t kSkillMax256 = 100 * 256;

    void gainSkills(SkillPoints& skills, Activity activity, int32_t units = 1);

    // GLF-022: each game day without playing every skill decays by this much, never below the golfer's initial value
    // minus kDecayFloor.
    constexpr int32_t kDecayPerDay256 = 51; // 0.2
    constexpr int32_t kDecayFloor256 = 10 * 256;
    void decaySkills(SkillPoints& skills, const SkillPoints& initial, int32_t days);

    // GLF-023: the bounded registry of golfers that come back.
    struct RegistryEntry
    {
        uint32_t id = 0;
        std::string name;
        SkillPoints skills;
        SkillPoints initial;
        uint8_t equipmentLevel = 0;
        uint32_t lastSeenDay = 0;
        uint8_t lastHappiness = 128;
        uint32_t visits = 0;
        uint64_t touch = 0; // increases with every use, to evict the least recently seen
    };

    class GolferRegistry
    {
    public:
        static constexpr size_t kDefaultCapacity = 2000;

        explicit GolferRegistry(size_t capacity = kDefaultCapacity)
            : _capacity(capacity)
        {
        }

        // Adds a golfer or refreshes an existing one. Evicts the least recently seen entry when full.
        RegistryEntry& remember(const RegistryEntry& entry);
        RegistryEntry* find(uint32_t id);
        const RegistryEntry* find(uint32_t id) const;
        bool forget(uint32_t id);
        // Marks a golfer as seen on a day with the happiness they left with.
        bool recordVisit(uint32_t id, uint32_t day, uint8_t happiness);
        // Applies the decay of the days since each golfer was last seen up to `today` (GLF-022).
        void decayTo(uint32_t today);

        // A returning golfer with probability tied to their last happiness (GLF-023): the chance of each entry is
        // proportional to happiness + kHappinessBias. `excluded` golfers are on the course already. Returns nullptr when
        // no one qualifies.
        static constexpr int32_t kHappinessBias = 16;
        const RegistryEntry* pickReturning(IBallRandom& rng, const std::vector<uint32_t>& excluded = {}) const;

        size_t size() const
        {
            return _entries.size();
        }
        size_t capacity() const
        {
            return _capacity;
        }
        void setCapacity(size_t capacity);
        const std::vector<RegistryEntry>& entries() const
        {
            return _entries;
        }
        void clear();

    private:
        void evictIfNeeded();
        void reindex();

        size_t _capacity;
        uint64_t _clock = 0;
        std::vector<RegistryEntry> _entries;
        std::unordered_map<uint32_t, size_t> _index;
    };
} // namespace OpenRCT2::Golf
