/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "GolfRuntime.h"

#include "../GameState.h"
#include "../object/BuildingObject.h"
#include "../scenario/Scenario.h"
#include "../world/Map.h"
#include "../world/tile_element/BuildingElement.h"
#include "../world/tile_element/TileElement.h"

namespace OpenRCT2::Golf
{
    namespace
    {
        constexpr int32_t kTileSize = 32;

        // World position (tile units of 32) of the door of a building part: the middle of the entrance edge.
        FixedVec3 doorPosition(const BuildingElement& element, const BuildingObject& object, const TileCoordsXY& tile)
        {
            const int32_t cx = tile.x * kTileSize + kTileSize / 2;
            const int32_t cy = tile.y * kTileSize + kTileSize / 2;
            const int32_t edge = (object.EntranceEdge - element.getRotation()) & 3;
            static constexpr int32_t kDx[4] = { -1, 0, 1, 0 };
            static constexpr int32_t kDy[4] = { 0, 1, 0, -1 };
            return { Fixed::fromInt(cx + kDx[edge] * (kTileSize / 2 + 1)), Fixed::fromInt(cy + kDy[edge] * (kTileSize / 2 + 1)),
                     Fixed::fromInt(element.getBaseZ()) };
        }
    } // namespace

    BuildingScan scanBuildings()
    {
        BuildingScan scan;
        uint32_t nextId = 1;
        TileElementIterator it;
        TileElementIteratorBegin(&it);
        do
        {
            if (it.element == nullptr || it.element->getType() != TileElementType::building)
                continue;
            const auto* element = it.element->asBuilding();
            const auto* object = element->getObject();
            if (object == nullptr || !object->HasEntrance)
                continue;
            // Only the part that holds the entrance is the door.
            if (element->getPartX() != object->EntranceTileX || element->getPartY() != object->EntranceTileY)
                continue;
            const auto door = doorPosition(*element, *object, TileCoordsXY{ it.x, it.y });

            Service service;
            service.id = nextId++;
            service.position = door;
            service.food = object->Services.has(BuildingService::food);
            service.drink = object->Services.has(BuildingService::drink);
            service.toilet = object->Services.has(BuildingService::toilet);
            service.rest = object->Services.has(BuildingService::rest);
            service.proShop = object->Services.has(BuildingService::proShop);

            if (object->Kind == BuildingKind::golfClub && !scan.hasClub)
            {
                scan.hasClub = true;
                scan.club = door;
            }
            if (service.food || service.drink || service.toilet || service.rest || service.proShop)
                scan.services.push_back(service);
        } while (TileElementIteratorNext(&it));
        return scan;
    }

    class GolfRuntime::GameRandom final : public IBallRandom
    {
    public:
        uint32_t next() override
        {
            return ScenarioRand();
        }
    };

    GolfRuntime::GolfRuntime(GameState_t& gameState)
        : _rng(std::make_unique<GameRandom>())
    {
        _world.useCourses();
        _world.useGameWind();
        _world.setObstacles(&_obstacles);

        GolfSimulation::Environment env;
        env.world = &_world;
        env.courses = &gameState.golf;
        env.rng = _rng.get();

        GolfDirector::Sources sources;
        sources.services = [this] { return _scan.services; };
        sources.spawnPoint = [this] { return _scan.club; };
        sources.clubEntrance = [this] { return _scan.club; };
        sources.exitPoint = [this] { return _scan.club; };
        sources.weather = [&gameState] { return gameState.weatherCurrent.weatherType; };
        _director = std::make_unique<GolfDirector>(env, sources);
        refreshBuildings();
    }

    GolfRuntime::~GolfRuntime() = default;

    void GolfRuntime::refreshBuildings()
    {
        _scan = scanBuildings();
        auto& env = _director->simulation().environment();
        env.clubEntrance = _scan.club;
        env.exit = _scan.club;
        env.services = _scan.services;
    }

    void GolfRuntime::update(uint32_t ticks)
    {
        if (ticks % kTicksPerGameMinute == 0)
        {
            _world.useGameWind();
            refreshBuildings();
        }
        if (!_scan.hasClub)
            return;
        _director->update(ticks);
    }

    GolfRuntime& runtimeOf(GameState_t& gameState)
    {
        if (gameState.golfRuntime == nullptr)
            gameState.golfRuntime = std::make_shared<GolfRuntime>(gameState);
        return *gameState.golfRuntime;
    }

    void resetRuntime(GameState_t& gameState)
    {
        gameState.golfRuntime.reset();
    }

    void updateGolf(GameState_t& gameState)
    {
        // Nothing to run before a course exists.
        if (gameState.golfRuntime == nullptr && gameState.golf.courses().empty())
            return;
        runtimeOf(gameState).update(gameState.currentTicks);
    }
} // namespace OpenRCT2::Golf
