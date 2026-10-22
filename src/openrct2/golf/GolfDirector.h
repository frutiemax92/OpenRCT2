/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "Animals.h"
#include "Arrivals.h"
#include "Carts.h"
#include "GolfObjectives.h"
#include "GolfSimulation.h"
#include "Popularity.h"
#include "Tournament.h"

#include <functional>
#include <memory>

namespace OpenRCT2::Golf
{
    // The director runs the course (sections 13.1, 22): it lets golfers in at the rate the course earns, sends them home at
    // closing time, keeps the registry of returning golfers, the popularity, the accounts and the tournament calendar, and
    // reports what happened. The game feeds it the world; it knows nothing of the map.
    class GolfDirector
    {
    public:
        // What the director needs from the game.
        struct Sources
        {
            std::function<std::vector<Service>()> services;       // buildings that serve needs
            std::function<std::vector<Bench>()> benches;          // path benches (PTH-021)
            std::function<std::vector<LitterBin>()> bins;         // litter bins (PTH-022)
            std::function<int32_t()> sceneryScore;                // scenery score (0..100, SCN-003)
            std::function<FixedVec3()> spawnPoint;                // where arriving golfers appear
            std::function<FixedVec3()> clubEntrance;              // the check-in desk
            std::function<FixedVec3()> exitPoint;
            std::function<Weather::Type()> weather;
            std::function<TournamentInputs()> tournamentInputs;   // the checklist figures
            // STE-021: a group arrives by car. The golfers are in the car (GolferState::inCar) when this is called; return
            // false when no car can come (no road entrance, no free parking space) and the group stays away. The game gets
            // them out with GolfSimulation::release once the car has parked.
            std::function<bool(const std::vector<uint32_t>& golferIds, uint32_t groupId)> carArrival;
        };

        GolfDirector(GolfSimulation::Environment environment, Sources sources);

        GolfSimulation& simulation()
        {
            return _sim;
        }
        GolferRegistry& registry()
        {
            return _registry;
        }

        // CRT-002: the golf carts, owned here so they are saved with the golfers who ride them.
        CartFleet fleet;
        // Animals (section 20), owned here so they are saved with the rest of the course.
        std::unique_ptr<AnimalSimulation> animals;

        OpeningHours hours;
        int32_t maxGolfers = kDefaultMaxGolfers;
        int32_t vipFeePercent = 0;      // ECO-032
        std::vector<std::string> firstNames;
        std::vector<std::string> lastNames;

        // Advances one tick; `ticks` is the game tick count.
        void update(uint32_t ticks);

        int32_t popularity() const
        {
            return _popularity;
        }
        void setPopularity(int32_t value)
        {
            _popularity = value;
        }
        const Ledger& today() const
        {
            return _today;
        }
        const Ledger& total() const
        {
            return _total;
        }
        int32_t golfersToday() const
        {
            return _arrivedToday;
        }
        int32_t bestGolfersInADay() const
        {
            return _bestDay;
        }
        int32_t roundsFinished() const
        {
            return _roundsFinished;
        }
        int32_t rageQuits() const
        {
            return _rageQuitsTotal;
        }
        // The average happiness of golfers who finished a round recently, 0..255.
        int32_t averageHappiness() const
        {
            return _averageHappiness;
        }

        // The tournament calendar (ECO-040 to ECO-042).
        Tournament tournament;
        uint8_t tiersHosted = 0;
        GolfObjective objective;
        ObjectiveStatus objectiveStatus() const;

        // Events since the last call, for the game (thoughts, news, sounds).
        std::vector<SimEvent> takeEvents();

        uint32_t groupsArrived() const
        {
            return _groupsArrived;
        }
        // For the day boundary processing in the tests and the game.
        uint32_t currentDay() const
        {
            return _day;
        }

        PopularityInputs popularityInputs() const;

    private:
        friend struct GolfSaveAccess;
        void everyMinute(uint32_t ticks);
        void everyDay();
        void admitGroup(uint32_t courseId, bool tournamentField);
        void sendEveryoneHome();
        void processEvents();
        uint32_t openCourse() const;

        GolfSimulation _sim;
        Sources _sources;
        GolferRegistry _registry;
        Ledger _today;
        Ledger _total;
        std::vector<SimEvent> _events;
        int32_t _popularity = 300;
        int32_t _arrivedToday = 0;
        int32_t _bestDay = 0;
        int32_t _roundsFinished = 0;
        int32_t _roundsRecent = 0;
        int32_t _rageQuitsRecent = 0;
        int32_t _rageQuitsTotal = 0;
        int32_t _averageHappiness = 128;
        uint32_t _day = 0;
        uint32_t _groupsArrived = 0;
        uint32_t _nextGroup = 1;
        uint32_t _nextRegistryId = 1;
        int64_t _profitSoFar = 0;
        bool _started = false;
        std::unordered_map<uint32_t, uint32_t> _registryOf; // golfer id -> registry id
    };
} // namespace OpenRCT2::Golf
