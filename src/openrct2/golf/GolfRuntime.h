/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "GolfDirector.h"
#include "MapBallWorld.h"
#include "MapObstacles.h"

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
        std::vector<Bench> benches;     // path additions that are benches (PTH-021)
        std::vector<LitterBin> bins;    // path additions that are litter bins (PTH-022)
    };
    BuildingScan scanBuildings();

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
        // The entity that shows a golfer, nullptr when it has none.
        GolferEntity* entityOf(uint32_t golferId) const;
        const BuildingScan& buildings() const
        {
            return _scan;
        }

    private:
        class GameRandom;
        void syncEntities();
        void settleAccounts();
        Ledger currentLedger() const;
        std::vector<Bench> mergeBenches() const;
        std::vector<LitterBin> mergeBins() const;
        MapObstacles _obstacles;
        MapBallWorld _world;
        std::unique_ptr<GameRandom> _rng;
        BuildingScan _scan;
        std::unique_ptr<GolfDirector> _director;
        std::unordered_map<uint32_t, uint16_t> _entityOf; // golfer id -> entity id
        Ledger _paid; // what of the director's income has reached the cash of the park
    };

    // The runtime of a game state, created on first call.
    GolfRuntime& runtimeOf(GameState_t& gameState);
    // Forgets the runtime (a new map or park was loaded).
    void resetRuntime(GameState_t& gameState);
    // Called from the game logic update: ticks the runtime when the park is in play.
    void updateGolf(GameState_t& gameState);
} // namespace OpenRCT2::Golf
