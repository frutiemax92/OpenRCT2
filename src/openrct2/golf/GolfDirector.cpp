/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "GolfDirector.h"

#include "Cottages.h"

#include <algorithm>

namespace OpenRCT2::Golf
{
    GolfDirector::GolfDirector(GolfSimulation::Environment environment, Sources sources)
        : _sim(std::move(environment))
        , _sources(std::move(sources))
    {
        _sim.environment().carts = &fleet;
        animals = std::make_unique<AnimalSimulation>(_sim.environment().world, _sim.environment().rng);
        _sim.environment().animals = animals.get();
    }

    uint32_t GolfDirector::openCourse() const
    {
        // The first open course with something to play; arrivals choose it.
        if (_sim.environment().courses == nullptr)
            return 0;
        for (const auto& course : _sim.environment().courses->courses())
            if (course.open && course.playableHoles() > 0)
                return course.id;
        return 0;
    }

    std::vector<SimEvent> GolfDirector::takeEvents()
    {
        std::vector<SimEvent> events;
        events.swap(_events);
        return events;
    }

    ObjectiveStatus GolfDirector::objectiveStatus() const
    {
        GolfObjectiveProgress p;
        p.totalProfit = _profitSoFar + _today.profit();
        p.popularity = _popularity;
        p.tournamentTiersHosted = tiersHosted;
        p.golfersToday = _arrivedToday;
        p.bestGolfersInADay = _bestDay;
        p.today = _day;
        return evaluateObjective(objective, p);
    }

    void GolfDirector::sendEveryoneHome()
    {
        for (auto& g : _sim.golfers())
        {
            if (g.state == GolferState::leaving || g.state == GolferState::rageQuitting || g.finished)
                continue;
            g.ballInPlay = false;
            g.destination = _sim.environment().exit;
            g.state = GolferState::leaving;
            g.stateTicks = 0;
        }
    }

    PopularityInputs GolfDirector::popularityInputs() const
    {
        PopularityInputs in;
        in.averageHappiness = _averageHappiness;
        in.priceFairness = 100;
        in.difficultyBalance = 80;
        in.recentRageQuits = _rageQuitsRecent;
        in.recentRounds = _roundsRecent;
        in.tournamentsHosted = 0;
        for (int t = 0; t < static_cast<int>(TournamentTier::count); t++)
            in.tournamentsHosted += (tiersHosted >> t) & 1;
        if (const auto* course = _sim.environment().courses != nullptr && !_sim.environment().courses->courses().empty()
                ? &_sim.environment().courses->courses().front()
                : nullptr)
            in.priceFairness = std::min(100, priceFairnessPercent(course->greenFee, static_cast<int32_t>(course->playableHoles())));

        if (_sources.sceneryScore)
        {
            in.sceneryScore = _sources.sceneryScore();
        }
        else if (_sim.environment().courses != nullptr)
        {
            int32_t totalScenery = 0;
            int32_t holeCount = 0;
            for (const auto& c : _sim.environment().courses->courses())
            {
                for (const auto& h : c.holes)
                {
                    if (h.isComplete())
                    {
                        totalScenery += h.sceneryScore;
                        holeCount++;
                    }
                }
            }
            if (holeCount > 0)
                in.sceneryScore = std::clamp(totalScenery / holeCount, 0, 100);
        }
        return in;
    }

    void GolfDirector::admitGroup(uint32_t courseId, bool tournamentField)
    {
        auto* rng = _sim.environment().rng;
        const auto* course = _sim.environment().courses->findCourse(courseId);
        if (course == nullptr)
            return;
        const int32_t room = maxGolfers - static_cast<int32_t>(_sim.golfers().size());
        if (room <= 0)
            return;
        const uint32_t group = _nextGroup++;
        std::vector<uint32_t> arrived;          // for the car
        std::vector<uint32_t> newRegistryIds;   // to take back if the car cannot come
        const int32_t size = tournamentField ? std::min(room, kProfessionalFieldSize) : rollGroupSize(*rng, room);
        // GLF-023: a share of the arriving golfers come back from the registry.
        for (int32_t i = 0; i < size; i++)
        {
            GolfSimulation::Spawn spawn;
            spawn.groupId = group;
            spawn.courseId = courseId;
            spawn.position = _sources.spawnPoint ? _sources.spawnPoint() : FixedVec3{};
            const RegistryEntry* returning = nullptr;
            if (!tournamentField && !_registry.entries().empty() && static_cast<int32_t>(rng->next() % 100) < kReturningGolferPercent)
            {
                std::vector<uint32_t> onCourse;
                for (const auto& [golferId, registryId] : _registryOf)
                    onCourse.push_back(registryId);
                returning = _registry.pickReturning(*rng, onCourse);
            }
            if (returning != nullptr)
            {
                spawn.name = returning->name;
                spawn.skills = returning->skills.toSkills();
                spawn.equipmentLevel = returning->equipmentLevel;
                NewGolferTemplate extra = rollNewGolfer(*rng, _popularity, false);
                spawn.money = extra.money;
                spawn.riskTolerance = extra.riskTolerance;
            }
            else if (tournamentField)
            {
                spawn.skills = professionalSkills(tournament.tier, *rng);
                spawn.money = 0;
                spawn.vip = true; // professionals do not pay
                spawn.equipmentLevel = 3;
                spawn.name = "Pro " + std::to_string(_nextRegistryId);
            }
            else
            {
                const bool vip = false;
                NewGolferTemplate t = rollNewGolfer(*rng, _popularity, vip);
                spawn.skills = t.skills;
                spawn.equipmentLevel = t.equipmentLevel;
                spawn.money = t.money;
                spawn.riskTolerance = t.riskTolerance;
                spawn.name = rollName(*rng, firstNames, lastNames, _sim.nextGolferId());
            }
            auto& golfer = _sim.spawn(spawn);
            arrived.push_back(golfer.id);
            if (_sources.carArrival)
                _sim.putInCar(golfer.id);
            // Registry: every golfer gets an entry on arrival so skills persist between visits (Q5).
            uint32_t registryId = 0;
            if (returning != nullptr)
            {
                registryId = returning->id;
            }
            else if (!tournamentField)
            {
                RegistryEntry entry;
                entry.id = _nextRegistryId++;
                entry.name = golfer.name;
                entry.skills = golfer.skills;
                entry.initial = golfer.initialSkills;
                entry.equipmentLevel = golfer.equipmentLevel;
                entry.lastSeenDay = _day;
                registryId = _registry.remember(entry).id;
                newRegistryIds.push_back(registryId);
            }
            if (registryId != 0)
                _registryOf[golfer.id] = registryId;
            _arrivedToday++;
        }
        if (_sources.carArrival && !_sources.carArrival(arrived, group))
        {
            // No car can come: the group stays away, as if it had never decided to play.
            for (const auto id : arrived)
            {
                _sim.remove(id);
                _registryOf.erase(id);
                _arrivedToday--;
            }
            for (const auto registryId : newRegistryIds)
                _registry.forget(registryId);
            return;
        }
        _groupsArrived++;
    }

    void GolfDirector::everyMinute(uint32_t ticks)
    {
        const int32_t minuteOfDay = minuteOfDayAt(ticks);
        // GLF-005: everyone is off the course by closing time.
        if (mustHaveLeft(hours, minuteOfDay))
            sendEveryoneHome();

        const uint32_t courseId = openCourse();
        const bool closedForTournament = courseClosedToPublic(tournament);
        if (courseId != 0 && !closedForTournament)
        {
            const auto* course = _sim.environment().courses->findCourse(courseId);
            ArrivalContext c;
            c.popularity = _popularity;
            c.greenFee = course->greenFee;
            c.holes = static_cast<int32_t>(course->playableHoles());
            c.weather = _sources.weather ? _sources.weather() : Weather::Type::sunny;
            c.minuteOfDay = minuteOfDay;
            c.golfersOnCourse = static_cast<int32_t>(_sim.golfers().size());
            c.maxGolfers = maxGolfers;
            c.hours = hours;
            c.courseOpen = true;
            if (groupArrives(c, *_sim.environment().rng))
                admitGroup(courseId, false);
        }
        else if (courseId != 0 && closedForTournament && isOpenAt(hours, minuteOfDay) && _sim.golfers().empty()
                 && tournament.professionals > 0)
        {
            // ECO-042: the professionals play instead; the field arrives once.
            admitGroup(courseId, true);
            tournament.professionals = 0;
        }
    }

    void GolfDirector::everyDay()
    {
        _day++;
        // A new day on the golf calendar.
        _bestDay = std::max(_bestDay, _arrivedToday);
        _arrivedToday = 0;
        _profitSoFar += _today.profit();
        _total.add(_today);
        _today = Ledger{};
        _registry.decayTo(_day);
        // Rage quits and rounds are remembered over about ten days.
        _rageQuitsRecent = _rageQuitsRecent * 9 / 10;
        _roundsRecent = _roundsRecent * 9 / 10;
        _popularity = movePopularity(_popularity, computePopularity(popularityInputs()));

        // The tournament moves on: it starts on its day and is over the next one.
        if (tournament.phase == TournamentPhase::running)
        {
            finish(tournament, _averageHappiness, 0, 0, 18, _popularity);
            _today.tournamentRevenue += tournament.revenue;
            _popularity = std::min(kPopularityMax, _popularity + tournament.popularityGained);
            tiersHosted |= static_cast<uint8_t>(1u << static_cast<int>(tournament.tier));
        }
        advanceDay(tournament, _day);
    }

    void GolfDirector::processEvents()
    {
        for (const auto& e : _sim.takeEvents())
        {
            switch (e.type)
            {
                case SimEvent::Type::paidFee:
                    _today.greenFees += e.value;
                    break;
                case SimEvent::Type::paidRangeFee:
                    _today.rangeBuckets += e.value;
                    break;
                case SimEvent::Type::paidCartFee:
                    _today.cartFees += e.value;
                    break;
                case SimEvent::Type::usedFacility:
                    _today.buildingServices += e.value;
                    break;
                case SimEvent::Type::rageQuit:
                    _rageQuitsRecent++;
                    _rageQuitsTotal++;
                    break;
                case SimEvent::Type::roundFinished:
                    _roundsFinished++;
                    _roundsRecent++;
                    break;
                default:
                    break;
            }
            _events.push_back(e);
        }
        for (auto& g : _sim.takeLeft())
        {
            // The average happiness moves a little with every golfer who leaves (the mood of the last ~20).
            _averageHappiness += (g.needs.happinessByte() - _averageHappiness) / 20;
            auto it = _registryOf.find(g.id);
            if (it != _registryOf.end())
            {
                if (auto* entry = _registry.find(it->second))
                {
                    entry->skills = g.skills;
                    _registry.recordVisit(it->second, _day, g.needs.happinessByte());
                }
                _registryOf.erase(it);
            }
        }
    }

    void GolfDirector::update(uint32_t ticks)
    {
        if (!_started)
        {
            _started = true;
            _day = gameDayAt(ticks);
        }
        if (ticks % static_cast<uint32_t>(kTicksPerGameMinute) == 0)
        {
            // The buildings and the desk can change at any time; look again once a game minute.
            auto& env = _sim.environment();
            if (_sources.services)
                env.services = _sources.services();
            if (_sources.benches)
                env.benches = _sources.benches();
            if (_sources.bins)
                env.bins = _sources.bins();
            if (_sources.clubEntrance)
                env.clubEntrance = _sources.clubEntrance();
            if (_sources.exitPoint)
                env.exit = _sources.exitPoint();
            everyMinute(ticks);
            if (minuteOfDayAt(ticks) == 0 && gameDayAt(ticks) != _day)
                everyDay();
        }
        _sim.tick(ticks);
        processEvents();
    }
} // namespace OpenRCT2::Golf
