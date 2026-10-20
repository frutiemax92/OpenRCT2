/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../core/FixedVec.hpp"
#include "BallPhysics.h"
#include "ObstacleGrid.h"

#include <cstdint>
#include <string>
#include <vector>

namespace OpenRCT2::Golf
{
    // Animals (section 20): species with a habitat, group size and spawn rate; they wander, flee golfers and carts, are hit
    // by balls, now and then steal one, and cheer up the golfers who see them.

    // ANM-001: where a species lives. A sample of the ground qualifies when any of the flags it asks for matches.
    enum AnimalHabitat : uint8_t
    {
        kHabitatWater = 1 << 0,     // ducks swim
        kHabitatGreenBank = 1 << 1, // dry ground next to water
        kHabitatWoods = 1 << 2,     // deer, squirrels
        kHabitatRough = 1 << 3,
        kHabitatFairway = 1 << 4,
    };

    struct AnimalSpecies
    {
        std::string id;
        uint8_t habitat = kHabitatWoods;
        // Roughness windows that identify woods and rough on the surface samples (TER-042: woods 0.80, rough 0.45).
        int32_t minGroup = 1;
        int32_t maxGroup = 3;
        int32_t spawnPerThousandPerDay = 40;   // chance per game day of spawning a group, per mille
        int32_t fleeRadiusUnits = 5 * 32;
        int32_t walkSpeedHundredths = 40;
        int32_t runSpeedHundredths = 120;
        int32_t wanderRadiusUnits = 4 * 32;
        bool canFly = false;                   // birds can take a ball and fly off
        bool canSwim = false;
        bool avoidsStreets = true;             // ANM-002
        int32_t bodyRadiusUnits = 3;           // ANM-003: a small cylinder
        int32_t bodyHeightUnits = 8;
        bool thief = false;                    // ANM-004: may steal a ball
    };

    // Habitat test on the ground under a position.
    bool suitsHabitat(const AnimalSpecies& species, const SurfaceSample& here, bool nextToWater);

    enum class AnimalState : uint8_t
    {
        idle,
        walk,
        run,
        flee,
        fly,
        swim,
    };

    struct Animal
    {
        uint32_t id = 0;
        uint32_t species = 0;       // index into the species table
        uint32_t groupId = 0;
        FixedVec3 position;
        FixedVec3 target;
        AnimalState state = AnimalState::idle;
        int32_t ticksLeft = 0;      // in the current state
        bool hit = false;           // knocked by a ball: flees at once
    };

    // Things the animals run away from.
    struct Threat
    {
        FixedVec3 position;
        bool isCart = false;
    };

    struct AnimalEvent
    {
        enum class Type : uint8_t
        {
            spawned,
            fled,
            hitByBall,      // a sound (ANM-003)
            stoleBall,      // ANM-004: value = golfer id
            flewAway,
        };
        Type type = Type::spawned;
        uint32_t animalId = 0;
        uint32_t value = 0;
    };

    constexpr int32_t kStealChancePerMillionPerMinute = 1500; // rare (ANM-004)
    constexpr int32_t kStealRadiusUnits = 3 * 32;
    constexpr int32_t kAnimalSightUnits = 5 * 32;             // ANM-005
    constexpr int32_t kAnimalHappinessPerHoleCap = 2;         // sightings that count each hole
    constexpr int32_t kFleeDistanceUnits = 6 * 32;            // how far an animal runs from a scare

    class AnimalSimulation
    {
    public:
        AnimalSimulation(const IBallWorld* world, IBallRandom* rng)
            : _world(world)
            , _rng(rng)
        {
        }

        uint32_t addSpecies(const AnimalSpecies& species);
        const std::vector<AnimalSpecies>& species() const
        {
            return _species;
        }

        // ANM-002: a group appears in suitable habitat near `centre`. Returns the number of animals spawned (0 when no
        // suitable ground was found).
        int32_t spawnGroup(uint32_t speciesIndex, const FixedVec3& centre, int32_t searchRadiusUnits = 8 * 32);
        // The daily spawn roll: tries each species with its spawn rate around the given candidate positions.
        int32_t dailySpawns(const std::vector<FixedVec3>& candidates);

        void tick(const std::vector<Threat>& threats);

        const std::vector<Animal>& animals() const
        {
            return _animals;
        }
        std::vector<AnimalEvent> takeEvents();

        // ANM-003: animals are small cylinders for the balls; a hit animal flees and plays a sound.
        Obstacle obstacleOf(const Animal& animal) const;
        void hitByBall(uint32_t animalId);

        // ANM-004: a thief near a ball at rest on rough may take it. Returns true if one did (+1 stroke for the golfer).
        bool tryStealBall(const FixedVec3& ballPosition, uint32_t golferId);

        // ANM-005: how many animals are in sight of a position.
        int32_t animalsInSight(const FixedVec3& position) const;

    private:
        Animal* find(uint32_t id);
        bool suitable(const AnimalSpecies& species, const FixedVec3& position) const;
        void chooseWander(Animal& a, const AnimalSpecies& species);

        const IBallWorld* _world;
        IBallRandom* _rng;
        std::vector<AnimalSpecies> _species;
        std::vector<Animal> _animals;
        std::vector<AnimalEvent> _events;
        uint32_t _nextId = 1;
        uint32_t _nextGroup = 1;
    };

    // ANM-005: a golfer's sightings in the current hole, capped so a zoo does not make them euphoric.
    struct AnimalSightings
    {
        int32_t thisHole = 0;
        // True (and counted) while below the cap.
        bool take()
        {
            if (thisHole >= kAnimalHappinessPerHoleCap)
                return false;
            thisHole++;
            return true;
        }
        void newHole()
        {
            thisHole = 0;
        }
    };
} // namespace OpenRCT2::Golf
