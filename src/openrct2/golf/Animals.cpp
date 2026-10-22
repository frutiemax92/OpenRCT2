/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Animals.h"

#include <algorithm>

namespace OpenRCT2::Golf
{
    namespace
    {
        constexpr int32_t kWoodsRoughnessPercent = 70;
        constexpr int32_t kRoughRoughnessPercent = 35;

        Fixed distanceXY(const FixedVec3& a, const FixedVec3& b)
        {
            return length(FixedVec3{ a.x - b.x, a.y - b.y, Fixed{} });
        }
    } // namespace

    bool suitsHabitat(const AnimalSpecies& species, const SurfaceSample& here, bool nextToWater)
    {
        if (here.outOfBounds || here.path)
            return false;
        if (here.water)
            return (species.habitat & kHabitatWater) != 0 || species.canSwim;
        const int32_t roughness = static_cast<int32_t>((here.roughness * 100).round());
        if ((species.habitat & kHabitatGreenBank) && nextToWater)
            return true;
        if ((species.habitat & kHabitatWoods) && roughness >= kWoodsRoughnessPercent && !here.hazard)
            return true;
        if ((species.habitat & kHabitatRough) && roughness >= kRoughRoughnessPercent && roughness < kWoodsRoughnessPercent)
            return true;
        if ((species.habitat & kHabitatFairway) && roughness < kRoughRoughnessPercent && !here.puttable)
            return true;
        return false;
    }

    uint32_t AnimalSimulation::addSpecies(const AnimalSpecies& species)
    {
        _species.push_back(species);
        return static_cast<uint32_t>(_species.size() - 1);
    }

    void AnimalSimulation::setSpecies(std::vector<AnimalSpecies> species)
    {
        std::vector<Animal> kept;
        for (auto animal : _animals)
        {
            const auto& old = _species[animal.species];
            const auto it = std::find_if(species.begin(), species.end(), [&](const AnimalSpecies& s) { return s.id == old.id; });
            if (it == species.end())
                continue;
            animal.species = static_cast<uint32_t>(it - species.begin());
            kept.push_back(animal);
        }
        _animals = std::move(kept);
        _species = std::move(species);
    }

    Animal* AnimalSimulation::find(uint32_t id)
    {
        for (auto& a : _animals)
            if (a.id == id)
                return &a;
        return nullptr;
    }

    std::vector<AnimalEvent> AnimalSimulation::takeEvents()
    {
        std::vector<AnimalEvent> events;
        events.swap(_events);
        return events;
    }

    bool AnimalSimulation::suitable(const AnimalSpecies& species, const FixedVec3& position) const
    {
        const auto here = _world->surfaceAt(position.x, position.y);
        bool nextToWater = false;
        if (species.habitat & kHabitatGreenBank)
        {
            const Fixed reach = Fixed::fromInt(32);
            for (const auto& offset : { FixedVec3{ reach, Fixed{}, Fixed{} }, FixedVec3{ -reach, Fixed{}, Fixed{} },
                                        FixedVec3{ Fixed{}, reach, Fixed{} }, FixedVec3{ Fixed{}, -reach, Fixed{} } })
                nextToWater |= _world->surfaceAt(position.x + offset.x, position.y + offset.y).water;
        }
        return suitsHabitat(species, here, nextToWater);
    }

    int32_t AnimalSimulation::spawnGroup(uint32_t speciesIndex, const FixedVec3& centre, int32_t searchRadiusUnits)
    {
        if (speciesIndex >= _species.size())
            return 0;
        const auto& species = _species[speciesIndex];
        // Try a number of random places around the centre for suitable ground.
        FixedVec3 spot{};
        bool found = false;
        for (int attempt = 0; attempt < 24 && !found; attempt++)
        {
            const int32_t dx = static_cast<int32_t>(_rng->next() % static_cast<uint32_t>(searchRadiusUnits * 2 + 1)) - searchRadiusUnits;
            const int32_t dy = static_cast<int32_t>(_rng->next() % static_cast<uint32_t>(searchRadiusUnits * 2 + 1)) - searchRadiusUnits;
            FixedVec3 p{ centre.x + Fixed::fromInt(dx), centre.y + Fixed::fromInt(dy), Fixed{} };
            if (suitable(species, p))
            {
                spot = p;
                found = true;
            }
        }
        if (!found)
            return 0;
        const int32_t span = std::max(1, species.maxGroup - species.minGroup + 1);
        const int32_t count = species.minGroup + static_cast<int32_t>(_rng->next() % static_cast<uint32_t>(span));
        const uint32_t group = _nextGroup++;
        int32_t spawned = 0;
        for (int32_t i = 0; i < count; i++)
        {
            Animal a;
            a.id = _nextId++;
            a.species = speciesIndex;
            a.groupId = group;
            // Members of a group stand near each other, on suitable ground.
            FixedVec3 p = spot;
            for (int tries = 0; tries < 6; tries++)
            {
                FixedVec3 q{ spot.x + Fixed::fromInt(static_cast<int32_t>(_rng->next() % 33) - 16),
                             spot.y + Fixed::fromInt(static_cast<int32_t>(_rng->next() % 33) - 16), Fixed{} };
                if (suitable(species, q))
                {
                    p = q;
                    break;
                }
            }
            p.z = _world->surfaceAt(p.x, p.y).height;
            a.position = p;
            a.target = p;
            a.state = AnimalState::idle;
            a.ticksLeft = 40 + static_cast<int32_t>(_rng->next() % 120);
            _animals.push_back(a);
            _events.push_back({ AnimalEvent::Type::spawned, a.id, group });
            spawned++;
        }
        return spawned;
    }

    int32_t AnimalSimulation::dailySpawns(const std::vector<FixedVec3>& candidates)
    {
        if (candidates.empty())
            return 0;
        int32_t total = 0;
        for (uint32_t s = 0; s < _species.size(); s++)
        {
            if (static_cast<int32_t>(_rng->next() % 1000) >= _species[s].spawnPerThousandPerDay)
                continue;
            const auto& centre = candidates[_rng->next() % candidates.size()];
            total += spawnGroup(s, centre);
        }
        return total;
    }

    void AnimalSimulation::chooseWander(Animal& a, const AnimalSpecies& species)
    {
        const int32_t r = species.wanderRadiusUnits;
        for (int attempt = 0; attempt < 6; attempt++)
        {
            FixedVec3 p{ a.position.x + Fixed::fromInt(static_cast<int32_t>(_rng->next() % static_cast<uint32_t>(r * 2 + 1)) - r),
                         a.position.y + Fixed::fromInt(static_cast<int32_t>(_rng->next() % static_cast<uint32_t>(r * 2 + 1)) - r),
                         Fixed{} };
            if (suitable(species, p))
            {
                a.target = p;
                a.state = AnimalState::walk;
                a.ticksLeft = 400;
                return;
            }
        }
        a.state = AnimalState::idle;
        a.ticksLeft = 60 + static_cast<int32_t>(_rng->next() % 120);
    }

    void AnimalSimulation::tick(const std::vector<Threat>& threats)
    {
        for (auto& a : _animals)
        {
            const auto& species = _species[a.species];
            // ANM-002: golfers and carts within the flee radius send the animal off, away from the nearest of them. A
            // knocked animal does the same at once.
            const Threat* scare = nullptr;
            Fixed scareDistance{};
            for (const auto& t : threats)
            {
                const Fixed d = distanceXY(t.position, a.position);
                if (d <= Fixed::fromInt(species.fleeRadiusUnits) && (scare == nullptr || d < scareDistance))
                {
                    scare = &t;
                    scareDistance = d;
                }
            }
            if ((scare != nullptr || a.hit) && a.state != AnimalState::flee)
            {
                const FixedVec3 from = scare != nullptr ? scare->position : a.position;
                const FixedVec3 away = normalised(FixedVec3{ a.position.x - from.x, a.position.y - from.y, Fixed{} });
                const FixedVec3 dir = away == FixedVec3{} ? FixedVec3{ Fixed::fromInt(1), Fixed{}, Fixed{} } : away;
                // Run to ground that suits the animal, away from the scare.
                FixedVec3 goal = a.position;
                for (Fixed reach = Fixed::fromInt(kFleeDistanceUnits); reach >= Fixed::fromInt(32); reach = reach / 2)
                {
                    FixedVec3 p{ a.position.x + dir.x * reach, a.position.y + dir.y * reach, Fixed{} };
                    if (suitable(species, p))
                    {
                        goal = p;
                        break;
                    }
                }
                a.target = goal;
                a.state = species.canFly ? AnimalState::fly : AnimalState::flee;
                a.ticksLeft = 600;
                a.hit = false;
                _events.push_back({ AnimalEvent::Type::fled, a.id, 0 });
            }

            switch (a.state)
            {
                case AnimalState::idle:
                    if (--a.ticksLeft <= 0)
                        chooseWander(a, species);
                    break;
                case AnimalState::walk:
                case AnimalState::run:
                case AnimalState::flee:
                case AnimalState::fly:
                case AnimalState::swim:
                {
                    const bool fast = a.state == AnimalState::flee || a.state == AnimalState::fly || a.state == AnimalState::run;
                    const Fixed speed = Fixed::ratio(fast ? species.runSpeedHundredths : species.walkSpeedHundredths, 100);
                    const FixedVec3 delta{ a.target.x - a.position.x, a.target.y - a.position.y, Fixed{} };
                    const Fixed distance = length(delta);
                    if (distance <= speed || --a.ticksLeft <= 0)
                    {
                        a.position.x = a.target.x;
                        a.position.y = a.target.y;
                        a.position.z = _world->surfaceAt(a.position.x, a.position.y).height;
                        a.state = AnimalState::idle;
                        a.ticksLeft = 60 + static_cast<int32_t>(_rng->next() % 200);
                        break;
                    }
                    const FixedVec3 next{ a.position.x + delta.x * speed / distance, a.position.y + delta.y * speed / distance, Fixed{} };
                    // ANM-002: never step onto a street (or off the map), and stay on ground the species lives on.
                    const auto ahead = _world->surfaceAt(next.x, next.y);
                    if ((species.avoidsStreets && ahead.path) || ahead.outOfBounds || (ahead.water && !species.canSwim && !species.canFly))
                    {
                        a.state = AnimalState::idle;
                        a.ticksLeft = 30;
                        break;
                    }
                    a.position.x = next.x;
                    a.position.y = next.y;
                    a.position.z = species.canFly && a.state == AnimalState::fly ? ahead.height + Fixed::fromInt(24) : ahead.height;
                    break;
                }
            }
        }
    }

    Obstacle AnimalSimulation::obstacleOf(const Animal& animal) const
    {
        const auto& species = _species[animal.species];
        Obstacle o;
        o.id = 0x60000000u + animal.id;
        o.shape = ObstacleShape::cylinder;
        o.centre = animal.position;
        o.halfX = Fixed::fromInt(species.bodyRadiusUnits);
        o.halfY = o.halfX;
        o.height = Fixed::fromInt(species.bodyHeightUnits);
        o.restitution = Fixed::ratio(2, 10);
        o.friction = Fixed::ratio(5, 10);
        return o;
    }

    void AnimalSimulation::hitByBall(uint32_t animalId)
    {
        if (auto* a = find(animalId))
        {
            a->hit = true;
            _events.push_back({ AnimalEvent::Type::hitByBall, animalId, 0 });
        }
    }

    bool AnimalSimulation::tryStealBall(const FixedVec3& ballPosition, uint32_t golferId)
    {
        // Only a ball at rest on rough ground is of interest (ANM-004).
        const auto here = _world->surfaceAt(ballPosition.x, ballPosition.y);
        const int32_t roughness = static_cast<int32_t>((here.roughness * 100).round());
        if (roughness < kRoughRoughnessPercent || here.water || here.path || here.puttable)
            return false;
        for (const auto& a : _animals)
        {
            if (!_species[a.species].thief)
                continue;
            if (distanceXY(a.position, ballPosition) > Fixed::fromInt(kStealRadiusUnits))
                continue;
            if (static_cast<int32_t>(_rng->next() % 1000000) < kStealChancePerMillionPerMinute)
            {
                _events.push_back({ AnimalEvent::Type::stoleBall, a.id, golferId });
                return true;
            }
        }
        return false;
    }

    int32_t AnimalSimulation::animalsInSight(const FixedVec3& position) const
    {
        int32_t count = 0;
        for (const auto& a : _animals)
            if (distanceXY(a.position, position) <= Fixed::fromInt(kAnimalSightUnits))
                count++;
        return count;
    }
} // namespace OpenRCT2::Golf
