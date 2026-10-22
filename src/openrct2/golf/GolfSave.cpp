/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "GolfSave.h"

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <type_traits>

namespace OpenRCT2::Golf
{
    namespace
    {
        constexpr size_t kMaxCount = 1u << 20; // sanity bound for any saved list

        // One routine describes a structure for both directions: the archive writes the values when saving and fills them
        // in when loading.
        class Archive
        {
        public:
            // Saving.
            Archive() = default;
            // Loading.
            explicit Archive(std::span<const uint8_t> data)
                : _reading(true)
                , _data(data)
            {
            }

            bool reading() const
            {
                return _reading;
            }
            // The format of the data being read or written; the structures read what the format has.
            uint32_t format = kGolfSaveFormat;
            std::vector<uint8_t> take()
            {
                return std::move(_out);
            }

            template<typename T>
            void io(T& value)
            {
                if constexpr (std::is_enum_v<T>)
                {
                    auto raw = static_cast<std::underlying_type_t<T>>(value);
                    io(raw);
                    value = static_cast<T>(raw);
                }
                else if constexpr (std::is_same_v<T, bool>)
                {
                    uint8_t raw = value ? 1 : 0;
                    io(raw);
                    value = raw != 0;
                }
                else
                {
                    static_assert(std::is_arithmetic_v<T>);
                    uint8_t bytes[sizeof(T)];
                    if (_reading)
                    {
                        take(bytes, sizeof(T));
                        T result = 0;
                        for (size_t i = 0; i < sizeof(T); i++)
                            result = static_cast<T>(result | (static_cast<std::make_unsigned_t<T>>(bytes[i]) << (8 * i)));
                        value = result;
                    }
                    else
                    {
                        auto raw = static_cast<std::make_unsigned_t<T>>(value);
                        for (size_t i = 0; i < sizeof(T); i++)
                            _out.push_back(static_cast<uint8_t>(raw >> (8 * i)));
                    }
                }
            }

            void io(Fixed& value)
            {
                int64_t raw = value.raw();
                io(raw);
                value = Fixed::fromRaw(raw);
            }
            void io(FixedVec3& v)
            {
                io(v.x);
                io(v.y);
                io(v.z);
            }
            void io(std::string& value)
            {
                uint32_t size = static_cast<uint32_t>(value.size());
                io(size);
                if (_reading)
                {
                    check(size);
                    value.assign(reinterpret_cast<const char*>(_data.data() + _pos), size);
                    _pos += size;
                }
                else
                    _out.insert(_out.end(), value.begin(), value.end());
            }

            // A list of structures, each described by `each`.
            template<typename T, typename F>
            void list(std::vector<T>& values, F each)
            {
                uint32_t size = static_cast<uint32_t>(values.size());
                io(size);
                if (_reading)
                {
                    if (size > kMaxCount)
                        throw std::runtime_error("golf save: list too long");
                    values.assign(size, T{});
                }
                for (auto& value : values)
                    each(value);
            }

        private:
            void check(size_t bytes) const
            {
                if (_pos + bytes > _data.size())
                    throw std::runtime_error("golf save: unexpected end");
            }
            void take(uint8_t* into, size_t bytes)
            {
                check(bytes);
                std::memcpy(into, _data.data() + _pos, bytes);
                _pos += bytes;
            }

            bool _reading = false;
            std::span<const uint8_t> _data;
            size_t _pos = 0;
            std::vector<uint8_t> _out;
        };

        void io(Archive& a, SkillPoints& s)
        {
            a.io(s.longPlay);
            a.io(s.accuracy);
            a.io(s.foolproof);
            a.io(s.physique);
        }

        void io(Archive& a, Needs& n)
        {
            a.io(n.hunger);
            a.io(n.thirst);
            a.io(n.energy);
            a.io(n.happiness);
            a.io(n.bladder);
        }

        void io(Archive& a, HoleScore& s)
        {
            a.io(s.par);
            a.io(s.shots);
            a.io(s.penalties);
            a.io(s.holed);
            a.io(s.pickedUp);
        }

        void io(Archive& a, Ball& b)
        {
            a.io(b.position);
            a.io(b.velocity);
            a.io(b.sideSpin);
            a.io(b.backSpin);
            a.io(b.state);
            a.io(b.owner);
            a.io(b.lastSafePosition);
            a.io(b.hazardEntry);
            a.io(b.shotOrigin);
            a.io(b.passingObstacle);
            a.io(b.lippedOut);
            a.io(b.bounces);
            a.io(b.lieSurface);
        }

        void io(Archive& a, ShotDecision& d)
        {
            a.io(d.club);
            a.io(d.power);
            a.io(d.target);
            a.io(d.aimX);
            a.io(d.aimY);
            a.io(d.putt);
            a.io(d.expectedDistance);
        }

        void io(Archive& a, Golfer& g)
        {
            a.io(g.id);
            a.io(g.name);
            a.io(g.groupId);
            io(a, g.skills);
            io(a, g.initialSkills);
            a.io(g.equipmentLevel);
            a.io(g.money);
            a.io(g.vip);
            a.io(g.riskTolerance);
            io(a, g.needs);
            a.io(g.state);
            a.io(g.stateTicks);
            a.io(g.rageCauses);
            a.io(g.courseId);
            a.io(g.holeIndex);
            a.io(g.tee);
            io(a, g.score);
            a.list(g.scorecard, [&](int8_t& v) { a.io(v); });
            a.list(g.parCard, [&](uint8_t& v) { a.io(v); });
            a.io(g.strokesTotal);
            a.io(g.position);
            a.io(g.destination);
            io(a, g.ball);
            io(a, g.decision);
            a.io(g.shotTargetDistance);
            a.io(g.ballInPlay);
            a.io(g.teeTimeMinute);
            a.io(g.waitedMinutes);
            a.io(g.lastWaitMinute);
            a.io(g.holeStartMinute);
            a.io(g.facilityTicksLeft);
            a.io(g.facilityService);
            a.io(g.facilityResumeBall);
            a.io(g.restingBenchId);
            a.io(g.searchedBall);
            a.io(g.facilityNeed);
            a.io(g.wantsCart);
            a.io(g.rageQuit);
            a.io(g.finished);
            a.io(g.arrivalDay);
            if (a.format >= 2)
            {
                a.io(g.hasExit);
                a.io(g.exitPosition);
            }
            if (a.format >= 4)
            {
                a.io(g.rangeId);
                a.io(g.rangeVisited);
            }
            if (a.format >= 3)
            {
                a.io(g.cartId);
                a.io(g.cartDecided);
                a.io(g.onCart);
                a.io(g.cartStop);
            }
            a.list(g.thoughts, [&](Thought& t) {
                a.io(t.type);
                a.io(t.tick);
            });
        }

        void io(Archive& a, RegistryEntry& e)
        {
            a.io(e.id);
            a.io(e.name);
            io(a, e.skills);
            io(a, e.initial);
            a.io(e.equipmentLevel);
            a.io(e.lastSeenDay);
            a.io(e.lastHappiness);
            a.io(e.visits);
            a.io(e.touch);
            if (a.format >= 6)
            {
                a.io(e.vip);
                a.io(e.lotId);
            }
        }

        void io(Archive& a, Ledger& l)
        {
            a.io(l.greenFees);
            a.io(l.cartFees);
            a.io(l.rangeBuckets);
            a.io(l.miniPuttFees);
            a.io(l.buildingServices);
            a.io(l.cottageSales);
            a.io(l.tournamentRevenue);
            a.io(l.construction);
            a.io(l.buildingUpkeep);
            a.io(l.staffWages);
            a.io(l.cartPurchases);
            a.io(l.loanInterest);
            a.io(l.marketing);
        }

        void io(Archive& a, Tournament& t)
        {
            a.io(t.phase);
            a.io(t.tier);
            a.io(t.day);
            a.io(t.ticketPrice);
            a.io(t.professionals);
            a.io(t.spectators);
            a.io(t.revenue);
            a.io(t.result);
            a.io(t.popularityGained);
        }

        void io(Archive& a, GolfObjective& o)
        {
            a.io(o.type);
            a.io(o.target);
            a.io(o.tier);
            a.io(o.deadlineDay);
        }
    } // namespace

    // The keys of a map in order, so the same state always saves to the same bytes.
    template<typename Map>
    static std::vector<typename Map::key_type> sortedKeys(const Map& map)
    {
        std::vector<typename Map::key_type> keys;
        keys.reserve(map.size());
        for (const auto& entry : map)
            keys.push_back(entry.first);
        std::sort(keys.begin(), keys.end());
        return keys;
    }

    struct GolfSaveAccess
    {
        static void describe(Archive& a, GolfDirector& d)
        {
            uint32_t format = kGolfSaveFormat;
            a.io(format);
            if (format > kGolfSaveFormat)
                throw std::runtime_error("golf save: newer format");
            a.format = format;

            // The simulation.
            auto& sim = d._sim;
            a.io(sim._now);
            a.io(sim._nextId);
            a.io(sim._income);
            a.list(sim._golfers, [&](Golfer& g) { io(a, g); });

            uint32_t sheets = static_cast<uint32_t>(sim._sheets.size());
            a.io(sheets);
            if (a.reading())
            {
                if (sheets > kMaxCount)
                    throw std::runtime_error("golf save: too many sheets");
                sim._sheets.clear();
                for (uint32_t i = 0; i < sheets; i++)
                {
                    uint32_t course = 0;
                    a.io(course);
                    TeeSheet sheet;
                    describeSheet(a, sheet);
                    sim._sheets.emplace(course, std::move(sheet));
                }
            }
            else
            {
                for (auto course : sortedKeys(sim._sheets))
                {
                    a.io(course);
                    describeSheet(a, sim._sheets.at(course));
                }
            }

            uint32_t bookings = static_cast<uint32_t>(sim._groupTeeTime.size());
            a.io(bookings);
            if (a.reading())
            {
                if (bookings > kMaxCount)
                    throw std::runtime_error("golf save: too many bookings");
                sim._groupTeeTime.clear();
                for (uint32_t i = 0; i < bookings; i++)
                {
                    uint64_t key = 0;
                    uint32_t minute = 0;
                    a.io(key);
                    a.io(minute);
                    sim._groupTeeTime[key] = minute;
                }
            }
            else
            {
                for (auto key : sortedKeys(sim._groupTeeTime))
                {
                    uint32_t minute = sim._groupTeeTime.at(key);
                    a.io(key);
                    a.io(minute);
                }
            }
            if (a.reading())
            {
                sim._difficulty.clear();
                sim._left.clear();
                sim._events.clear();
            }

            // The golf carts.
            if (a.format >= 3)
            {
                auto& fleet = d.fleet;
                a.io(fleet.cartFee);
                a.io(fleet.cartPrice);
                a.io(fleet._nextSpaceId);
                a.io(fleet._nextCartId);
                a.list(fleet._spaces, [&](CartSpace& s) {
                    a.io(s.id);
                    a.io(s.position);
                    a.io(s.cartId);
                    a.io(s.reservedBy);
                });
                a.list(fleet._carts, [&](Cart& c) {
                    a.io(c.id);
                    a.io(c.state);
                    a.io(c.position);
                    a.io(c.destination);
                    a.list(c.riders, [&](uint32_t& rider) { a.io(rider); });
                    a.io(c.spaceId);
                    a.io(c.groupId);
                    a.io(c.speed);
                    a.io(c.battery);
                    a.io(c.wearTicks);
                    a.io(c.condition);
                });
            }

            // The animals: their species by id, since the species table comes from the animal objects.
            if (a.format >= 5)
            {
                auto& animals = *d.animals;
                a.io(animals._nextId);
                a.io(animals._nextGroup);
                struct SavedAnimal
                {
                    std::string species;
                    Animal animal;
                };
                std::vector<SavedAnimal> saved;
                if (!a.reading())
                {
                    for (const auto& animal : animals._animals)
                        saved.push_back({ animals._species[animal.species].id, animal });
                }
                a.list(saved, [&](SavedAnimal& s) {
                    a.io(s.species);
                    a.io(s.animal.id);
                    a.io(s.animal.groupId);
                    a.io(s.animal.position);
                    a.io(s.animal.target);
                    a.io(s.animal.state);
                    a.io(s.animal.ticksLeft);
                    a.io(s.animal.hit);
                });
                if (a.reading())
                {
                    animals._animals.clear();
                    for (auto& s : saved)
                    {
                        // An animal of a species that is not loaded any more does not come back.
                        const auto it = std::find_if(
                            animals._species.begin(), animals._species.end(), [&](const AnimalSpecies& species) { return species.id == s.species; });
                        if (it == animals._species.end())
                            continue;
                        s.animal.species = static_cast<uint32_t>(it - animals._species.begin());
                        animals._animals.push_back(s.animal);
                    }
                }
            }

            // The registry.
            auto& registry = d._registry;
            uint64_t capacity = registry._capacity;
            a.io(capacity);
            a.io(registry._clock);
            a.list(registry._entries, [&](RegistryEntry& e) { io(a, e); });
            if (a.reading())
            {
                registry._capacity = static_cast<size_t>(capacity);
                registry.reindex();
            }

            // The director.
            a.io(d.hours.open);
            a.io(d.hours.close);
            a.io(d.hours.lastArrivalBeforeClose);
            a.io(d.maxGolfers);
            a.io(d.vipFeePercent);
            io(a, d.tournament);
            a.io(d.tiersHosted);
            io(a, d.objective);
            io(a, d._today);
            io(a, d._total);
            a.io(d._popularity);
            a.io(d._arrivedToday);
            a.io(d._bestDay);
            a.io(d._roundsFinished);
            a.io(d._roundsRecent);
            a.io(d._rageQuitsRecent);
            a.io(d._rageQuitsTotal);
            a.io(d._averageHappiness);
            a.io(d._day);
            a.io(d._groupsArrived);
            a.io(d._nextGroup);
            a.io(d._nextRegistryId);
            a.io(d._profitSoFar);
            a.io(d._started);

            uint32_t links = static_cast<uint32_t>(d._registryOf.size());
            a.io(links);
            if (a.reading())
            {
                if (links > kMaxCount)
                    throw std::runtime_error("golf save: too many links");
                d._registryOf.clear();
                for (uint32_t i = 0; i < links; i++)
                {
                    uint32_t golfer = 0, entry = 0;
                    a.io(golfer);
                    a.io(entry);
                    d._registryOf[golfer] = entry;
                }
            }
            else
            {
                for (auto golfer : sortedKeys(d._registryOf))
                {
                    uint32_t entry = d._registryOf.at(golfer);
                    a.io(golfer);
                    a.io(entry);
                }
            }
        }

        static void describeSheet(Archive& a, TeeSheet& sheet)
        {
            a.list(sheet._lastTeeOff, [&](uint32_t& v) { a.io(v); });
            std::vector<uint8_t> played(sheet._played.size());
            for (size_t i = 0; i < played.size(); i++)
                played[i] = sheet._played[i] ? 1 : 0;
            a.list(played, [&](uint8_t& v) { a.io(v); });
            if (a.reading())
            {
                sheet._played.assign(played.size(), false);
                for (size_t i = 0; i < played.size(); i++)
                    sheet._played[i] = played[i] != 0;
            }
        }
    };

    std::vector<uint8_t> saveDirector(const GolfDirector& director)
    {
        Archive archive;
        // Describing only reads the director when saving; the const is cast away for the shared routine.
        GolfSaveAccess::describe(archive, const_cast<GolfDirector&>(director));
        return archive.take();
    }

    bool loadDirector(GolfDirector& director, std::span<const uint8_t> data)
    {
        // Load into a copy of nothing: a failure part way would leave the director half written, so it is kept aside by
        // saving its current state first and putting that back on failure.
        const auto before = saveDirector(director);
        try
        {
            Archive archive(data);
            GolfSaveAccess::describe(archive, director);
            return true;
        }
        catch (const std::exception&)
        {
            Archive restore{ std::span<const uint8_t>(before) };
            GolfSaveAccess::describe(restore, director);
            return false;
        }
    }
} // namespace OpenRCT2::Golf
