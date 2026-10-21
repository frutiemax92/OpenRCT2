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
                const auto* addition = path->hasAddition() && !path->additionIsGhost() ? path->getAdditionEntry() : nullptr;
                if (addition == nullptr)
                    continue;
                // The id is the tile, so a bench keeps its occupants and a bin its rubbish from one scan to the next.
                const uint32_t id = static_cast<uint32_t>(it.y) * 1024u + static_cast<uint32_t>(it.x) + 1u;
                const FixedVec3 centre{ Fixed::fromInt(it.x * kTileSize + kTileSize / 2),
                                        Fixed::fromInt(it.y * kTileSize + kTileSize / 2),
                                        Fixed::fromInt(path->getBaseZ()) };
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
        _world.setObstacles(&_obstacles);

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
        sources.weather = [&gameState] { return gameState.weatherCurrent.weatherType; };
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

    void GolfRuntime::refreshBuildings()
    {
        _scan = scanBuildings();
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
        if (!_scan.hasClub)
            return;
        _director->update(ticks);
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
