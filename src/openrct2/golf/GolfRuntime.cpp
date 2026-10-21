/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "GolfRuntime.h"

#include "../Diagnostic.h"
#include "../GameState.h"
#include "../entity/EntityList.h"
#include "../localisation/Formatter.h"
#include "../localisation/StringIds.h"
#include "../management/Finance.h"
#include "../management/NewsItem.h"
#include "../entity/EntityRegistry.h"
#include "../entity/GolferEntity.h"
#include "../object/BuildingObject.h"
#include "../object/PathAdditionEntry.h"
#include "../world/tile_element/PathElement.h"
#include "../scenario/Scenario.h"
#include "../world/Map.h"
#include "../world/tile_element/BuildingElement.h"
#include "../world/tile_element/TileElement.h"
#include "GolfSave.h"
#include "../entity/GolfVehicleEntity.h"
#include "../world/Parking.h"
#include "../world/Street.h"

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
            if (it.element != nullptr && it.element->getType() == TileElementType::path)
            {
                const auto* path = it.element->asPath();
                const FixedVec3 centre{ Fixed::fromInt(it.x * kTileSize + kTileSize / 2),
                                        Fixed::fromInt(it.y * kTileSize + kTileSize / 2),
                                        Fixed::fromInt(path->getBaseZ()) };
                if (!path->isGhost())
                    scan.pathPoints.push_back(centre);
                const auto* addition = path->hasAddition() && !path->additionIsGhost() ? path->getAdditionEntry() : nullptr;
                if (addition == nullptr)
                    continue;
                // The id is the tile, so a bench keeps its occupants and a bin its rubbish from one scan to the next.
                const uint32_t id = static_cast<uint32_t>(it.y) * 1024u + static_cast<uint32_t>(it.x) + 1u;
                if (addition->flags.has(PathAdditionFlag::isBench))
                {
                    Bench bench;
                    bench.id = id;
                    bench.position = centre;
                    scan.benches.push_back(bench);
                }
                else if (addition->flags.has(PathAdditionFlag::isBin))
                {
                    LitterBin bin;
                    bin.id = id;
                    bin.position = centre;
                    scan.bins.push_back(bin);
                }
                continue;
            }
            if (it.element == nullptr || it.element->getType() != TileElementType::building)
                continue;
            const auto* element = it.element->asBuilding();
            const auto* object = element->getObject();
            if (object == nullptr)
                continue;
            if (element->getPartX() == 0 && element->getPartY() == 0)
                scan.upkeepPerMonth += object->UpkeepPerMonth;
            if (!object->HasEntrance)
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
        _world.setObstacles(&GetMapObstacles());

        GolfSimulation::Environment env;
        env.world = &_world;
        env.courses = &gameState.golf;
        env.rng = _rng.get();

        GolfDirector::Sources sources;
        sources.services = [this] { return _scan.services; };
        sources.benches = [this] { return mergeBenches(); };
        sources.bins = [this] { return mergeBins(); };
        sources.spawnPoint = [this] { return _scan.club; };
        sources.clubEntrance = [this] { return _scan.club; };
        sources.exitPoint = [this] { return _scan.club; };
        sources.carArrival = [this](const std::vector<uint32_t>& ids, uint32_t group) { return receiveCar(ids, group); };
        sources.weather = [&gameState] { return gameState.weatherCurrent.weatherType; };
        env.pathPoints = [this] { return _scan.pathPoints; };
        _director = std::make_unique<GolfDirector>(env, sources);
        refreshBuildings();
        if (!gameState.golfSimulationData.empty())
        {
            if (!loadDirector(*_director, gameState.golfSimulationData))
                LOG_ERROR("The saved golfers are damaged and were dropped");
            gameState.golfSimulationData.clear();
        }

        _paid = currentLedger();

        // Golfer entities from before (a map that was reset or swapped) belong to no golfer.
        std::vector<GolferEntity*> orphans;
        for (auto* entity : EntityList<GolferEntity>())
            orphans.push_back(entity);
        for (auto* entity : orphans)
            entity->remove();
        std::vector<GolfVehicleEntity*> orphanVehicles;
        for (auto* entity : EntityList<GolfVehicleEntity>())
            orphanVehicles.push_back(entity);
        for (auto* entity : orphanVehicles)
            entity->remove();
    }

    GolferEntity* GolfRuntime::entityOf(uint32_t golferId) const
    {
        const auto it = _entityOf.find(golferId);
        if (it == _entityOf.end())
            return nullptr;
        return getGameState().entities.getEntity<GolferEntity>(EntityId::FromUnderlying(it->second));
    }

    void GolfRuntime::announce(const SimEvent& event)
    {
        if (event.type != SimEvent::Type::rageQuit)
            return;
        const auto known = _known.find(event.golferId);
        auto ft = Formatter();
        const std::string who = known != _known.end() ? known->second.name : std::string("A golfer");
        ft.Add<const char*>(who.c_str());
        ft.Add<int16_t>(static_cast<int16_t>(event.value));
        const int32_t insult = std::clamp(event.insult, 0, kInsultCount - 1);
        ft.Add<StringId>(STR_GOLFER_INSULT_0 + insult);
        const int32_t x = known != _known.end() ? known->second.x : 0;
        const int32_t y = known != _known.end() ? known->second.y : 0;
        const uint32_t packedXY = (static_cast<uint32_t>(y) << 16) | static_cast<uint32_t>(x);
        News::AddItemToQueue(getGameState().park.newsItems, News::ItemType::blank, STR_GOLFER_RAGE_QUIT_NEWS, packedXY, ft);
    }

    void GolfRuntime::syncEntities()
    {
        auto& sim = _director->simulation();
        for (const auto& g : sim.golfers())
            _known[g.id] = Known{ g.name, static_cast<int32_t>(g.position.x.floor()), static_cast<int32_t>(g.position.y.floor()) };
        std::unordered_map<uint32_t, bool> alive;
        for (const auto& g : sim.golfers())
        {
            alive[g.id] = true;
            const CoordsXYZ where{ static_cast<int32_t>(g.position.x.floor()), static_cast<int32_t>(g.position.y.floor()),
                                   static_cast<int32_t>(g.position.z.floor()) };
            auto* entity = entityOf(g.id);
            if (entity == nullptr)
            {
                entity = GolferEntity::create(g.id, where);
                if (entity == nullptr)
                    continue; // the entity limit is hit: this golfer plays unseen
                _entityOf[g.id] = entity->id.ToUnderlying();
            }
            entity->follow(where, g.state);
        }
        for (auto it = _entityOf.begin(); it != _entityOf.end();)
        {
            if (alive.find(it->first) == alive.end())
            {
                if (auto* entity = entityOf(it->first))
                    entity->remove();
                it = _entityOf.erase(it);
            }
            else
                ++it;
        }
    }

    GolfRuntime::~GolfRuntime() = default;

    Ledger GolfRuntime::currentLedger() const
    {
        Ledger sum = _director->total();
        sum.add(_director->today());
        return sum;
    }

    // ECO-010: the income of the director reaches the cash of the park, under the categories of the finance window.
    void GolfRuntime::settleAccounts()
    {
        const auto now = currentLedger();
        auto pay = [](int64_t& paid, int64_t current, ExpenditureType type) {
            if (current != paid)
            {
                // A payment is a cost, so income is a negative payment.
                FinancePayment(-static_cast<money64>(current - paid), type);
                paid = current;
            }
        };
        pay(_paid.greenFees, now.greenFees, ExpenditureType::parkEntranceTickets);
        pay(_paid.tournamentRevenue, now.tournamentRevenue, ExpenditureType::parkEntranceTickets);
        pay(_paid.cartFees, now.cartFees, ExpenditureType::parkRideTickets);
        pay(_paid.rangeBuckets, now.rangeBuckets, ExpenditureType::parkRideTickets);
        pay(_paid.miniPuttFees, now.miniPuttFees, ExpenditureType::parkRideTickets);
        pay(_paid.buildingServices, now.buildingServices, ExpenditureType::foodDrinkSales);
        pay(_paid.cottageSales, now.cottageSales, ExpenditureType::landPurchase);
    }

    // ECO-011: the buildings cost their upkeep, a thirtieth of the month each game day.
    void GolfRuntime::payUpkeep()
    {
        const auto day = _director->currentDay();
        if (day == _upkeepDay)
            return;
        const bool first = _upkeepDay == 0xFFFFFFFFu;
        _upkeepDay = day;
        if (first || _scan.upkeepPerMonth == 0)
            return;
        FinancePayment(_scan.upkeepPerMonth / 30, ExpenditureType::rideRunningCosts);
    }

    void GolfRuntime::refreshTraffic()
    {
        StreetNetwork network;
        Street::BuildNetwork(network);
        const auto lots = Parking::FindLots();

        // A fingerprint of everything the traffic depends on, so it is only rebuilt when the map changed.
        uint64_t layout = 1469598103934665603ull;
        auto mix = [&layout](uint64_t value) {
            for (int i = 0; i < 8; i++)
            {
                layout ^= (value >> (8 * i)) & 0xFF;
                layout *= 1099511628211ull;
            }
        };
        for (const auto& tile : network.tiles())
        {
            const auto* t = network.tileAt(tile);
            mix((static_cast<uint64_t>(static_cast<uint32_t>(tile.x)) << 32) | static_cast<uint32_t>(tile.y));
            mix(t->connections | (t->oneWay ? 0x100u : 0u) | (t->roadEntrance ? 0x200u : 0u) | (static_cast<uint64_t>(t->flow) << 12)
                | (static_cast<uint64_t>(t->speedLimitKph) << 16));
        }
        for (const auto& lot : lots)
        {
            mix((static_cast<uint64_t>(static_cast<uint32_t>(lot.streetTile.x)) << 32) | static_cast<uint32_t>(lot.streetTile.y));
            mix((static_cast<uint64_t>(static_cast<uint32_t>(lot.pathTile.x)) << 32) | static_cast<uint32_t>(lot.pathTile.y));
            for (const auto& tile : lot.tiles)
                mix((static_cast<uint64_t>(static_cast<uint32_t>(tile.tile.x)) << 32) | (static_cast<uint32_t>(tile.tile.y) << 4) | tile.facing);
        }
        if (_layoutKnown && layout == _layout)
            return;
        _layout = layout;
        _layoutKnown = true;

        // The cars on the old streets are gone, and their golfers get out at the club.
        for (const auto& [carId, ids] : _carGolfers)
        {
            for (const auto id : ids)
                _director->simulation().release(id, _scan.club, _scan.club);
            if (auto* entity = getGameState().entities.getEntity<GolfVehicleEntity>(EntityId::FromUnderlying(_carEntities[carId])))
                entity->remove();
            GetMapObstacles().removeDynamic(0x40000000u + carId);
        }
        _carGolfers.clear();
        _carEntities.clear();
        _traffic.reset();

        _network = std::move(network);
        for (const auto& lot : lots)
        {
            auto& added = _traffic.addLot(lot.streetTile, lot.pathTile);
            for (const auto& tile : lot.tiles)
                added.addTile(tile.tile, tile.facing, _nextSpaceId);
        }
    }

    bool GolfRuntime::receiveCar(const std::vector<uint32_t>& golferIds, uint32_t groupId)
    {
        auto& sim = _director->simulation();
        if (!hasCarTraffic())
        {
            // A map with no streets yet: the golfers simply turn up at the club, as before there were cars.
            for (const auto id : golferIds)
                sim.release(id, _scan.club, _scan.club);
            return true;
        }
        const auto carId = _traffic.spawn(golferIds, groupId);
        if (carId == 0)
            return false; // STE-011: no free space, no road entrance, no way there
        _carGolfers[carId] = golferIds;
        return true;
    }

    // STE-012: the golfers get out of a parked car and walk off from its space.
    void GolfRuntime::getOut(const Car& car)
    {
        const auto* space = _traffic.findSpace(car.spaceId);
        const auto where = space != nullptr ? parkingSpaceCentre(*space) : _scan.club;
        for (const auto id : car.golferIds)
            _director->simulation().release(id, where, where);
    }

    // The golfers of a car that was lost (a saved game, a rebuilt street) get out at the club.
    void GolfRuntime::releaseStrandedGolfers()
    {
        auto& sim = _director->simulation();
        for (auto& golfer : sim.golfers())
        {
            if (golfer.state != GolferState::inCar)
                continue;
            bool known = false;
            for (const auto& [carId, ids] : _carGolfers)
                known = known || std::find(ids.begin(), ids.end(), golfer.id) != ids.end();
            if (!known)
                sim.release(golfer.id, _scan.club, _scan.club);
        }
    }

    void GolfRuntime::driveTraffic()
    {
        _traffic.tick();
        for (const auto& event : _traffic.takeEvents())
        {
            if (event.type == TrafficEvent::Type::parked)
            {
                if (const auto* car = _traffic.find(event.carId))
                    getOut(*car);
            }
            else if (event.type == TrafficEvent::Type::despawned)
            {
                if (auto it = _carEntities.find(event.carId); it != _carEntities.end())
                {
                    if (auto* entity = getGameState().entities.getEntity<GolfVehicleEntity>(EntityId::FromUnderlying(it->second)))
                        entity->remove();
                    _carEntities.erase(it);
                }
                GetMapObstacles().removeDynamic(0x40000000u + event.carId);
                _carGolfers.erase(event.carId);
            }
        }

        // STE-022: when the whole group is off the course the car pulls out and drives to the nearest road entrance.
        auto& sim = _director->simulation();
        for (const auto& car : _traffic.cars())
        {
            if (car.state != CarState::parked)
                continue;
            bool anyoneLeft = false;
            for (const auto id : car.golferIds)
                anyoneLeft = anyoneLeft || sim.find(id) != nullptr;
            if (!anyoneLeft)
                _traffic.depart(car.id);
        }

        // The entities follow the cars.
        for (const auto& car : _traffic.cars())
        {
            if (car.state == CarState::gone)
                continue;
            const CoordsXYZ where{ static_cast<int32_t>(car.position.x.floor()), static_cast<int32_t>(car.position.y.floor()),
                                   static_cast<int32_t>(car.position.z.floor()) };
            GolfVehicleEntity* entity = nullptr;
            if (auto it = _carEntities.find(car.id); it != _carEntities.end())
                entity = getGameState().entities.getEntity<GolfVehicleEntity>(EntityId::FromUnderlying(it->second));
            if (entity == nullptr)
            {
                entity = GolfVehicleEntity::create(GolfVehicleEntity::Kind::car, car.id, where);
                if (entity == nullptr)
                    continue;
                _carEntities[car.id] = entity->id.ToUnderlying();
            }
            entity->follow(where, car.heading);
            // STE-024: a car is an obstacle for the balls.
            GetMapObstacles().setDynamic(0x40000000u + car.id, _traffic.obstacleOf(car));
        }
    }

    // CRT-002: the carts stand in the cart bays of the map; they are always drawn, parked or moving, and they are obstacles
    // for the balls (CRT-006).
    void GolfRuntime::syncCarts()
    {
        auto& fleet = _director->fleet;
        auto& entities = getGameState().entities;
        for (const auto& cart : fleet.carts())
        {
            const CoordsXYZ where{ static_cast<int32_t>(cart.position.x.floor()), static_cast<int32_t>(cart.position.y.floor()),
                                   static_cast<int32_t>(cart.position.z.floor()) };
            GolfVehicleEntity* entity = nullptr;
            if (auto it = _cartEntities.find(cart.id); it != _cartEntities.end())
                entity = entities.getEntity<GolfVehicleEntity>(EntityId::FromUnderlying(it->second));
            if (entity == nullptr)
            {
                entity = GolfVehicleEntity::create(GolfVehicleEntity::Kind::cart, cart.id, where);
                if (entity == nullptr)
                    continue;
                _cartEntities[cart.id] = entity->id.ToUnderlying();
                // A parked cart faces out of its bay; the bay's facing is not kept, so it starts facing the way it moves.
                entity->follow(where, 0);
            }
            else
            {
                // Face the way it drives; a cart standing still keeps its heading.
                entity->follow(where, -1);
            }
            GetMapObstacles().setDynamic(0x50000000u + cart.id, CartFleet::obstacleOf(cart));
        }
        // Carts that are gone (a bay was taken away) lose their entity.
        for (auto it = _cartEntities.begin(); it != _cartEntities.end();)
        {
            if (fleet.find(it->first) != nullptr)
            {
                ++it;
                continue;
            }
            if (auto* entity = entities.getEntity<GolfVehicleEntity>(EntityId::FromUnderlying(it->second)))
                entity->remove();
            GetMapObstacles().removeDynamic(0x50000000u + it->first);
            it = _cartEntities.erase(it);
        }
    }

    void GolfRuntime::refreshBuildings()
    {
        _scan = scanBuildings();
        // The cart bays of the map are the spaces of the fleet, and parked carts charge.
        {
            std::vector<FixedVec3> positions;
            for (const auto& bay : Parking::FindCartBays())
                positions.push_back({ Fixed::fromInt(bay.x), Fixed::fromInt(bay.y), Fixed::fromInt(bay.z) });
            _director->fleet.syncSpaces(positions);
            _director->fleet.rechargeParkedCarts();
        }
        auto& env = _director->simulation().environment();
        env.clubEntrance = _scan.club;
        env.exit = _scan.club;
        env.services = _scan.services;
        env.benches = mergeBenches();
        env.bins = mergeBins();
    }

    std::vector<Bench> GolfRuntime::mergeBenches() const
    {
        // A bench that is still there keeps who sits on it.
        auto benches = _scan.benches;
        for (auto& bench : benches)
            for (const auto& old : _director->simulation().environment().benches)
                if (old.id == bench.id)
                    bench.occupants = old.occupants;
        return benches;
    }

    std::vector<LitterBin> GolfRuntime::mergeBins() const
    {
        auto bins = _scan.bins;
        for (auto& bin : bins)
            for (const auto& old : _director->simulation().environment().bins)
                if (old.id == bin.id)
                    bin.trashCount = old.trashCount;
        return bins;
    }

    void GolfRuntime::update(uint32_t ticks)
    {
        if (ticks % kTicksPerGameMinute == 0)
        {
            _world.useGameWind();
            refreshBuildings();
        }
        // Carts are drawn wherever they stand, club or no club.
        syncCarts();
        if (!_scan.hasClub)
            return;
        if (ticks % kTicksPerGameMinute == 0)
        {
            refreshTraffic();
            releaseStrandedGolfers();
        }
        _director->update(ticks);
        driveTraffic();
        for (const auto& event : _director->takeEvents())
            announce(event);
        syncEntities();
        settleAccounts();
        payUpkeep();
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
