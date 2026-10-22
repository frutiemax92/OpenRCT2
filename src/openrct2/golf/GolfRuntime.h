/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../core/Money.hpp"
#include "../audio/Audio.h"
#include "../object/BuildingObject.h"
#include "GolfDirector.h"
#include "MapBallWorld.h"
#include "MapObstacles.h"
#include "StreetNetwork.h"
#include "Traffic.h"

#include <functional>
#include <memory>
#include <unordered_map>

namespace OpenRCT2
{
    struct GameState_t;
    struct GolferEntity;
}

namespace OpenRCT2::Golf
{
    // What the buildings of the map offer to the golfers, found by scanning the building elements (BLD-005, GLF-012).
    struct BuildingScan
    {
        bool hasClub = false;
        FixedVec3 club;                 // the entrance of the first golf club
        std::vector<Service> services;  // food trucks, restrooms, the pro shop of the club
        int32_t clubCount = 0;               // golf club buildings (ECO-040)
        std::vector<HolePoint> kiosks;       // driving range kiosks (DRV-001)
        std::vector<HolePoint> distanceSigns; // driving range distance signs (DRV-004)
        std::vector<FixedVec3> pathPoints; // the middle of every path tile, where carts may stop (CRT-004)
        money64 upkeepPerMonth = 0;     // upkeep of every building (ECO-011)
        std::vector<Bench> benches;     // path additions that are benches (PTH-021)
        std::vector<LitterBin> bins;    // path additions that are litter bins (PTH-022)
    };
    BuildingScan scanBuildings();
    int64_t defaultServicePrice(BuildingKind kind);

    // Ties the golf director to the game: the ball world of the map, the buildings that serve needs, the weather. It is
    // created on first use and ticked with the game logic. Nothing runs until the map has a golf club and an open course.
    class GolfRuntime
    {
    public:
        explicit GolfRuntime(GameState_t& gameState);
        ~GolfRuntime();

        // Advances the golf by one game tick.
        void update(uint32_t ticks);
        // Rescans the buildings; done once per game minute by update().
        void refreshBuildings();

        GolfDirector& director()
        {
            return *_director;
        }
        bool active() const
        {
            return _scan.hasClub;
        }
        // Where the sounds of the course go (section 21). By default they are played in the world with the spatial audio
        // model; a test replaces it. `where` is a map position.
        using SoundSink = std::function<void(Audio::SoundId, const CoordsXYZ& where)>;
        void setSoundSink(SoundSink sink)
        {
            _sound = std::move(sink);
        }

        // Tells the player about what the golfers did (GLF-042): a rage quit becomes a news item.
        void announce(const SimEvent& event);
        // The sound an event makes, if any (AUD-001, AUD-006).
        void sound(const SimEvent& event);

        // True when the map has streets with a road entrance and a parking lot, so groups arrive by car (STE-021).
        bool hasCarTraffic() const
        {
            return !_network.roadEntrances().empty() && !_traffic.lots().empty();
        }
        const TrafficSimulation& traffic() const
        {
            return _traffic;
        }

        // ECO-040: the figures the tournament checklist is made of, from the map. The course is the regular course with the
        // most holes.
        TournamentInputs tournamentInputs() const;

        // The entity that shows a golfer, nullptr when it has none.
        GolferEntity* entityOf(uint32_t golferId) const;
        const BuildingScan& buildings() const
        {
            return _scan;
        }

    private:
        class GameRandom;
        void syncEntities();
        void syncCarts();
        void syncBalls();
        // Animals (section 20) and the golfers as obstacles for the balls.
        void refreshAnimals();
        // Cottage lots (section 22.4): a VIP buys a lot when the course is popular.
        void sellCottages();
        bool sellLot(CottageLot& lot, int32_t attractiveness);
        void visitFromCottages(uint32_t ticks);
        void driveAnimals();
        void syncObstacles();
        void refreshRanges();
        // The streets and parking lots of the map, and the cars on them (STE-020 to STE-022).
        void refreshTraffic();
        void driveTraffic();
        bool receiveCar(const std::vector<uint32_t>& golferIds, uint32_t groupId);
        void getOut(const Car& car);
        void releaseStrandedGolfers();
        void settleAccounts();
        void payUpkeep();
        Ledger currentLedger() const;
        std::vector<Bench> mergeBenches() const;
        // The services as the player has set them: closed buildings serve no one, and prices can be changed (section 9.3).
        std::vector<Service> servicesWithSettings() const;
        std::vector<LitterBin> mergeBins() const;
        MapBallWorld _world;
        std::unique_ptr<GameRandom> _rng;
        BuildingScan _scan;
        std::unique_ptr<GolfDirector> _director;
        std::unordered_map<uint32_t, uint16_t> _entityOf; // golfer id -> entity id
        uint32_t _upkeepDay = 0xFFFFFFFFu; // the game day the upkeep was last paid
        struct Known
        {
            std::string name;
            int32_t x = 0, y = 0;
        };
        std::unordered_map<uint32_t, Known> _known; // golfer id -> name and place, for events about golfers that have left
        StreetNetwork _network;
        TrafficSimulation _traffic{ &_network };
        uint64_t _layout = 0;                // fingerprint of the streets and lots the traffic was built for
        bool _layoutKnown = false;
        uint32_t _nextSpaceId = 1;
        std::unordered_map<uint32_t, uint16_t> _carEntities;              // car id -> entity id
        std::unordered_map<uint32_t, std::vector<uint32_t>> _carGolfers;  // car id -> the golfers it brought
        std::unordered_map<uint32_t, uint16_t> _cartEntities;             // cart id -> entity id
        std::unordered_map<uint32_t, uint16_t> _animalEntities;           // animal id -> entity id
        std::vector<uint16_t> _animalObjects;                             // species index -> animal object entry index
        std::vector<Audio::SoundId> _animalSounds;                        // species index -> the sound it makes when hit
        std::vector<uint32_t> _golferObstacles;                           // golfer ids that are in the way of balls
        uint32_t _animalDay = 0xFFFFFFFFu;
        uint32_t _saleDay = 0xFFFFFFFFu;                                  // the game day cottage lots were last offered                                // the game day animals last tried to spawn
        std::unordered_map<uint64_t, uint16_t> _ballEntities;             // golfer id (range balls: | 1 << 32) -> entity id
        SoundSink _sound;
        Ledger _paid; // what of the director's income has reached the cash of the park
    };

    // The runtime of a game state, created on first call.
    GolfRuntime& runtimeOf(GameState_t& gameState);
    // Forgets the runtime (a new map or park was loaded).
    void resetRuntime(GameState_t& gameState);
    // Called from the game logic update: ticks the runtime when the park is in play.
    void updateGolf(GameState_t& gameState);
} // namespace OpenRCT2::Golf
