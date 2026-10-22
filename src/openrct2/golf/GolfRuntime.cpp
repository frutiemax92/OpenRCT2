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
#include "Arrivals.h"
#include "Clubs.h"
#include "../entity/EntityRegistry.h"
#include "../entity/GolferEntity.h"
#include "../object/BuildingObject.h"
#include "../object/PathAdditionEntry.h"
#include "../object/SmallSceneryEntry.h"
#include "../world/tile_element/SmallSceneryElement.h"
#include "DrivingRange.h"
#include "../world/tile_element/PathElement.h"
#include "../scenario/Scenario.h"
#include "../world/Map.h"
#include "../world/tile_element/BuildingElement.h"
#include "../world/tile_element/TileElement.h"
#include "GolfSave.h"
#include "../Context.h"
#include "../entity/AnimalEntity.h"
#include "../entity/GolfBall.h"
#include "../object/AnimalObject.h"
#include "../actions/GameActionRunner.h"
#include "../actions/scenery/BuildingPlaceAction.h"
#include "../world/Building.h"
#include "../world/MapOwnership.h"
#include "../world/TileElementsView.h"
#include "../world/Park.h"
#include "../world/tile_element/SurfaceElement.h"
#include "Cottages.h"
#include "../object/ObjectLimits.h"
#include "../object/ObjectManager.h"
#include "../entity/GolfVehicleEntity.h"
#include "../world/Parking.h"
#include "../world/Street.h"
#include "../actions/scenery/ParkingPlaceAction.h"
#include "../actions/scenery/StreetPlaceAction.h"
#include "../actions/scenery/StreetRoadEntranceAction.h"
#include "../object/DefaultObjects.h"
#include "../GameState.h"

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

    // What a visit costs at a building of a kind until the player says otherwise.
    int64_t defaultServicePrice(BuildingKind kind)
    {
        switch (kind)
        {
            case BuildingKind::golfClub:
                return 400;
            case BuildingKind::foodTruck:
                return 350;
            default:
                return 0; // restrooms and the like are free
        }
    }

    BuildingScan scanBuildings()
    {
        BuildingScan scan;
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
                else if (addition->flags.has(PathAdditionFlag::isBallWasher))
                {
                    BallWasher washer;
                    washer.id = id;
                    washer.position = centre;
                    scan.ballWashers.push_back(washer);
                }
                else if (addition->flags.has(PathAdditionFlag::isSign))
                {
                    PathSign sign;
                    sign.id = id;
                    sign.position = centre;
                    scan.pathSigns.push_back(sign);
                }
                continue;
            }
            if (it.element != nullptr && it.element->getType() == TileElementType::smallScenery)
            {
                // DRV-004: distance signs are small scenery with the flag.
                const auto* entry = it.element->asSmallScenery()->getEntry();
                if (entry != nullptr && !it.element->isGhost() && entry->flags.has(SmallSceneryFlag::isDistanceSign))
                    scan.distanceSigns.push_back({ it.x * kTileSize + kTileSize / 2, it.y * kTileSize + kTileSize / 2 });
                continue;
            }
            if (it.element == nullptr || it.element->getType() != TileElementType::building)
                continue;
            const auto* element = it.element->asBuilding();
            const auto* object = element->getObject();
            if (object == nullptr)
                continue;
            if (element->getPartX() == 0 && element->getPartY() == 0)
            {
                scan.upkeepPerMonth += object->UpkeepPerMonth;
                if (object->Kind == BuildingKind::drivingRangeKiosk && !element->isGhost())
                    scan.kiosks.push_back({ it.x * kTileSize + kTileSize / 2, it.y * kTileSize + kTileSize / 2 });
            }
            if (!object->HasEntrance)
                continue;
            // Only the part that holds the entrance is the door.
            if (element->getPartX() != object->EntranceTileX || element->getPartY() != object->EntranceTileY)
                continue;
            const auto door = doorPosition(*element, *object, TileCoordsXY{ it.x, it.y });

            Service service;
            // The id of a service is the tile of its building, so it stays when others are built or taken away.
            const auto origin = Building::GetOrigin(CoordsXY{ it.x * kTileSize, it.y * kTileSize }, *element);
            service.id = buildingKey(origin.x / kTileSize, origin.y / kTileSize);
            service.position = door;
            service.food = object->Services.has(BuildingService::food);
            service.drink = object->Services.has(BuildingService::drink);
            service.toilet = object->Services.has(BuildingService::toilet);
            service.rest = object->Services.has(BuildingService::rest);
            service.proShop = object->Services.has(BuildingService::proShop);

            if (object->Kind == BuildingKind::golfClub)
                scan.clubCount++;
            if (object->Kind == BuildingKind::golfClub && !scan.hasClub)
            {
                scan.hasClub = true;
                scan.club = door;
            }
            if (service.food || service.drink || service.toilet || service.rest || service.proShop)
            {
                service.price = defaultServicePrice(object->Kind);
                scan.services.push_back(service);
            }
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
        sources.services = [this] { return servicesWithSettings(); };
        sources.benches = [this] { return mergeBenches(); };
        sources.bins = [this] { return mergeBins(); };
        sources.spawnPoint = [this] { return _scan.club; };
        sources.clubEntrance = [this] { return _scan.club; };
        sources.exitPoint = [this] { return _scan.club; };
        sources.tournamentInputs = [this] { return tournamentInputs(); };
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
        // The balls of golfers that were shown before: the simulation shows them again.
        std::vector<GolfBall*> orphanBalls;
        for (auto* entity : EntityList<GolfBall>())
            if (entity->ball.owner != 0)
                orphanBalls.push_back(entity);
        for (auto* entity : orphanBalls)
        {
            entity->invalidate();
            getGameState().entities.entityRemove(entity);
        }
        std::vector<AnimalEntity*> orphanAnimals;
        for (auto* entity : EntityList<AnimalEntity>())
            orphanAnimals.push_back(entity);
        for (auto* entity : orphanAnimals)
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

    // AUD-006: swings, putts, ball impacts, crowds, and golfer vocal reactions.
    void GolfRuntime::sound(const SimEvent& event)
    {
        Audio::SoundId id = Audio::SoundId::null;
        switch (event.type)
        {
            case SimEvent::Type::shotHit:
            {
                const auto club = static_cast<ClubId>(event.value);
                if (club == ClubId::putter || club == ClubId::miniPutter)
                    id = Audio::SoundId::putt;
                else if (club == ClubId::driver || club == ClubId::wood3 || club == ClubId::wood5)
                    id = Audio::SoundId::driverSwing;
                else
                    id = Audio::SoundId::ironSwing;
                break;
            }
            case SimEvent::Type::holeFinished:
                // The ball drops into the cup; a birdie or better gets a hand.
                id = event.value <= -1 ? Audio::SoundId::crowdClap : Audio::SoundId::cupDrop;
                break;
            case SimEvent::Type::cheer:
                id = Audio::SoundId::golferCheer;
                break;
            case SimEvent::Type::groan:
            case SimEvent::Type::hitByBall:
                id = Audio::SoundId::golferGroan;
                break;
            case SimEvent::Type::ballImpact:
            {
                const auto impact = static_cast<BallImpactType>(event.value);
                switch (impact)
                {
                    case BallImpactType::water:
                        id = Audio::SoundId::ballImpactWater;
                        break;
                    case BallImpactType::sand:
                        id = Audio::SoundId::ballImpactSand;
                        break;
                    case BallImpactType::hard:
                        id = Audio::SoundId::ballImpactHard;
                        break;
                    case BallImpactType::tree:
                        id = Audio::SoundId::ballImpactTree;
                        break;
                    case BallImpactType::building:
                        id = Audio::SoundId::ballImpactBuilding;
                        break;
                    case BallImpactType::grass:
                    default:
                        id = Audio::SoundId::ballImpactGrass;
                        break;
                }
                break;
            }
            case SimEvent::Type::rageQuit:
                id = Audio::SoundId::golferInsultGrumble;
                break;
            default:
                return;
        }

        CoordsXYZ where{ event.x, event.y, event.z };
        if (where.x == 0 && where.y == 0)
        {
            const auto known = _known.find(event.golferId);
            if (known == _known.end())
                return;
            where = { known->second.x, known->second.y, 0 };
        }

        if (_sound)
            _sound(id, where);
        else
            Audio::PlaySpatial(id, where);
    }

    void GolfRuntime::announce(const SimEvent& event)
    {
        sound(event);
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

    // BAL-001: the balls the golfers play, and those hit at the driving ranges, are golf ball entities that follow the
    // simulation, so they fly, bounce and roll in front of the player.
    void GolfRuntime::syncBalls()
    {
        auto& entities = getGameState().entities;
        std::unordered_map<uint64_t, Ball> wanted;
        for (const auto& golfer : _director->simulation().golfers())
        {
            if (golfer.ballInPlay && golfer.ball.state != BallState::Holed)
                wanted[golfer.id] = golfer.ball;
        }
        for (const auto& view : _director->simulation().rangeBalls())
            wanted[(uint64_t{ 1 } << 32) | view.golferId] = view.ball;

        for (const auto& [key, ball] : wanted)
        {
            GolfBall* entity = nullptr;
            if (auto it = _ballEntities.find(key); it != _ballEntities.end())
                entity = entities.getEntity<GolfBall>(EntityId::FromUnderlying(it->second));
            if (entity == nullptr)
            {
                entity = GolfBall::create(ball.position, ball.state, static_cast<uint32_t>(key & 0xFFFFFFFFu));
                if (entity == nullptr)
                    continue;
                _ballEntities[key] = entity->id.ToUnderlying();
            }
            entity->mirror(ball);
        }
        for (auto it = _ballEntities.begin(); it != _ballEntities.end();)
        {
            if (wanted.find(it->first) != wanted.end())
            {
                ++it;
                continue;
            }
            if (auto* entity = entities.getEntity<GolfBall>(EntityId::FromUnderlying(it->second)))
            {
                entity->invalidate();
                entities.entityRemove(entity);
            }
            it = _ballEntities.erase(it);
        }
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
            // Over the ball the golfer looks along the shot; the aim is a direction of any length, kept in a fixed point.
            const bool playingShot = g.state == GolferState::swinging || g.state == GolferState::putting
                || g.state == GolferState::watchingBall;
            if (playingShot)
            {
                const CoordsXY aim{ static_cast<int32_t>((g.decision.aimX * Fixed::fromInt(1000)).floor()),
                                    static_cast<int32_t>((g.decision.aimY * Fixed::fromInt(1000)).floor()) };
                entity->follow(where, g.state, &aim);
            }
            else
            {
                entity->follow(where, g.state);
            }
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

    TournamentInputs GolfRuntime::tournamentInputs() const
    {
        TournamentInputs in;
        const auto& golf = getGameState().golf;
        const Course* best = nullptr;
        for (const auto& course : golf.courses())
            if (course.kind == CourseKind::regular && (best == nullptr || course.playableHoles() > best->playableHoles()))
                best = &course;
        if (best != nullptr)
        {
            in.holes = static_cast<int32_t>(best->playableHoles());
            in.totalYards = best->totalYards(TeeCategory::men);
            in.totalPar = best->totalPar();
        }
        in.popularity = _director->popularity();
        in.golfClubs = _scan.clubCount;
        in.parkingCapacity = _traffic.capacity();
        in.spectatorTiles = golf.tournamentZoneTiles(TournamentZoneKind::spectator);
        in.mediaTiles = golf.tournamentZoneTiles(TournamentZoneKind::media);
        in.extraParkingTiles = golf.tournamentZoneTiles(TournamentZoneKind::extraParking);
        in.grandstandTiles = golf.tournamentZoneTiles(TournamentZoneKind::grandstand);
        // ECO-040: the grandstand is by the last green: the distance in tiles from its zone to the cup of the last hole.
        in.grandstandDistanceToLastGreen = 1000;
        if (best != nullptr)
        {
            for (auto it = best->holes.rbegin(); it != best->holes.rend(); ++it)
            {
                if (!it->isComplete())
                    continue;
                for (const auto& zone : golf.tournamentZones())
                {
                    if (zone.kind != TournamentZoneKind::grandstand)
                        continue;
                    const int32_t cx = it->cup->x >> 5, cy = it->cup->y >> 5;
                    const int32_t dx = std::max({ zone.min.x - cx, cx - zone.max.x, 0 });
                    const int32_t dy = std::max({ zone.min.y - cy, cy - zone.max.y, 0 });
                    in.grandstandDistanceToLastGreen = std::min(in.grandstandDistanceToLastGreen, std::max(dx, dy));
                }
                break;
            }
        }
        return in;
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
                {
                    const CoordsXYZ where{ static_cast<int32_t>(car->position.x.floor()), static_cast<int32_t>(car->position.y.floor()), static_cast<int32_t>(car->position.z.floor()) };
                    if (_sound)
                        _sound(Audio::SoundId::carDoor, where);
                    else
                        Audio::PlaySpatial(Audio::SoundId::carDoor, where);
                    getOut(*car);
                }
            }
            else if (event.type == TrafficEvent::Type::departed)
            {
                if (const auto* car = _traffic.find(event.carId))
                {
                    const CoordsXYZ where{ static_cast<int32_t>(car->position.x.floor()), static_cast<int32_t>(car->position.y.floor()), static_cast<int32_t>(car->position.z.floor()) };
                    if (_sound)
                        _sound(Audio::SoundId::carDoor, where);
                    else
                        Audio::PlaySpatial(Audio::SoundId::carDoor, where);
                }
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

    // DRV-001, DRV-004: the kiosk and the distance signs of a range are the ones on the map inside its area; the signs show
    // the distance from the kiosk. The direction follows the hitting tiles until the player sets one.
    void GolfRuntime::refreshRanges()
    {
        for (auto& range : getGameState().golf.ranges())
        {
            std::optional<HolePoint> kiosk;
            for (const auto& candidate : _scan.kiosks)
            {
                if (range.containsArea(candidate))
                {
                    kiosk = candidate;
                    break;
                }
            }
            range.kiosk = kiosk;
            range.signs.clear();
            for (const auto& position : _scan.distanceSigns)
            {
                if (!range.containsArea(position))
                    continue;
                DistanceSign sign;
                sign.id = static_cast<uint32_t>((position.y >> 5) * 1024 + (position.x >> 5) + 1);
                sign.position = position;
                sign.distance = kiosk.has_value() ? signDistance(*kiosk, position) : 0;
                range.signs.push_back(sign);
            }
            if (range.autoDirection)
                range.shootingDirectionDegrees = defaultShootingDirection(range);
        }
    }

    namespace
    {
        constexpr size_t kMaxAnimals = 48;

        AnimalSpecies speciesOf(const AnimalObject& object)
        {
            AnimalSpecies species;
            species.id = std::string(object.GetIdentifier());
            species.habitat = 0;
            for (const auto& surface : object.AllowedSurfaces)
            {
                if (surface == "water")
                    species.habitat |= kHabitatWater;
                else if (surface == "grass")
                    species.habitat |= kHabitatGreenBank | kHabitatFairway;
                else if (surface == "rough")
                    species.habitat |= kHabitatRough;
                else if (surface == "woods")
                    species.habitat |= kHabitatWoods;
            }
            if (species.habitat == 0)
                species.habitat = kHabitatRough | kHabitatWoods;
            species.canSwim = (species.habitat & kHabitatWater) != 0;
            species.canFly = object.HasClip("fly");
            species.minGroup = std::max(1, object.MinGroupSize);
            species.maxGroup = std::max(species.minGroup, object.MaxGroupSize);
            species.spawnPerThousandPerDay = std::clamp(object.SpawnRate * 4, 0, 1000);
            species.bodyRadiusUnits = std::max(1, object.Collision.radius);
            species.bodyHeightUnits = std::max(2, object.Collision.height);
            return species;
        }
    } // namespace

    // ANM-001: the species are the animal objects that are loaded.
    void GolfRuntime::refreshAnimals()
    {
        auto& objects = GetContext()->GetObjectManager();
        std::vector<AnimalSpecies> species;
        std::vector<uint16_t> entries;
        for (uint16_t i = 0; i < kMaxAnimalObjects; i++)
        {
            if (const auto* object = objects.GetLoadedObject<AnimalObject>(i))
            {
                species.push_back(speciesOf(*object));
                entries.push_back(i);
            }
        }
        const auto& known = _director->animals->species();
        bool same = known.size() == species.size();
        for (size_t i = 0; same && i < known.size(); i++)
            same = known[i].id == species[i].id;
        if (same && entries == _animalObjects)
            return;
        _animalObjects = entries;
        _animalSounds.clear();
        for (const auto entry : entries)
        {
            const auto* object = objects.GetLoadedObject<AnimalObject>(entry);
            _animalSounds.push_back(object != nullptr && object->SoundEffect == "duck" ? Audio::SoundId::duck : Audio::SoundId::birdChirp);
        }
        _director->animals->setSpecies(std::move(species));
    }

    void GolfRuntime::driveAnimals()
    {
        auto& animals = *_director->animals;
        auto& sim = _director->simulation();

        // ANM-002: animals keep away from golfers and moving carts.
        std::vector<Threat> threats;
        for (const auto& golfer : sim.golfers())
            if (golfer.state != GolferState::inCar)
                threats.push_back({ golfer.position, false });
        for (const auto& cart : _director->fleet.carts())
            if (cart.state != CartState::parked)
                threats.push_back({ cart.position, true });
        animals.tick(threats);

        // The daily roll for new groups, around the holes and the ranges and in a few places on the map.
        const auto day = _director->currentDay();
        if (day != _animalDay)
        {
            const bool first = _animalDay == 0xFFFFFFFFu;
            _animalDay = day;
            if (!first && animals.animals().size() < kMaxAnimals)
            {
                std::vector<FixedVec3> candidates;
                for (const auto& course : getGameState().golf.courses())
                    for (const auto& hole : course.holes)
                        if (hole.cup)
                            candidates.push_back({ Fixed::fromInt(hole.cup->x), Fixed::fromInt(hole.cup->y), Fixed{} });
                for (const auto& range : getGameState().golf.ranges())
                    candidates.push_back(
                        { Fixed::fromInt((range.areaMin.x + range.areaMax.x + 1) * 16),
                          Fixed::fromInt((range.areaMin.y + range.areaMax.y + 1) * 16), Fixed{} });
                const auto size = getGameState().mapSize;
                if (size.x > 4 && size.y > 4)
                {
                    for (int i = 0; i < 6; i++)
                    {
                        const auto roll = _rng->next();
                        candidates.push_back({ Fixed::fromInt((2 + static_cast<int32_t>(roll % static_cast<uint32_t>(size.x - 4))) * 32),
                                               Fixed::fromInt((2 + static_cast<int32_t>((roll >> 12) % static_cast<uint32_t>(size.y - 4))) * 32),
                                               Fixed{} });
                    }
                }
                animals.dailySpawns(candidates);
            }
        }

        // ANM-003: what the balls hit makes a sound; a species with a sound plays its own.
        for (const auto& event : animals.takeEvents())
        {
            if (event.type != AnimalEvent::Type::hitByBall)
                continue;
            for (const auto& animal : animals.animals())
            {
                if (animal.id != event.animalId)
                    continue;
                const CoordsXYZ where{ static_cast<int32_t>(animal.position.x.floor()), static_cast<int32_t>(animal.position.y.floor()), 0 };
                const auto sound = animal.species < _animalSounds.size() ? _animalSounds[animal.species] : Audio::SoundId::birdChirp;
                if (_sound)
                    _sound(sound, where);
                else
                    Audio::PlaySpatial(sound, where);
            }
        }

        // The entities follow the animals.
        auto& entities = getGameState().entities;
        std::unordered_map<uint32_t, bool> alive;
        for (const auto& animal : animals.animals())
        {
            alive[animal.id] = true;
            const CoordsXYZ where{ static_cast<int32_t>(animal.position.x.floor()), static_cast<int32_t>(animal.position.y.floor()),
                                   static_cast<int32_t>(animal.position.z.floor()) };
            AnimalEntity* entity = nullptr;
            if (auto it = _animalEntities.find(animal.id); it != _animalEntities.end())
                entity = entities.getEntity<AnimalEntity>(EntityId::FromUnderlying(it->second));
            if (entity == nullptr)
            {
                const uint16_t object = animal.species < _animalObjects.size() ? _animalObjects[animal.species] : 0;
                entity = AnimalEntity::create(animal.id, object, where);
                if (entity == nullptr)
                    continue;
                _animalEntities[animal.id] = entity->id.ToUnderlying();
            }
            entity->follow(where, animal.state);
            // ANM-003: a small cylinder in the way of the balls.
            GetMapObstacles().setDynamic(kAnimalObstacleBase + animal.id, animals.obstacleOf(animal));
        }
        for (auto it = _animalEntities.begin(); it != _animalEntities.end();)
        {
            if (alive.find(it->first) != alive.end())
            {
                ++it;
                continue;
            }
            if (auto* entity = entities.getEntity<AnimalEntity>(EntityId::FromUnderlying(it->second)))
                entity->remove();
            GetMapObstacles().removeDynamic(kAnimalObstacleBase + it->first);
            it = _animalEntities.erase(it);
        }
    }

    namespace
    {
        // ECO-031: what a lot looks out on, counted around its middle.
        LotFeatures featuresOf(const CottageLot& lot, const FixedVec3& club)
        {
            LotFeatures features;
            const TileCoordsXY centre{ (lot.min.x + lot.max.x) / 2, (lot.min.y + lot.max.y) / 2 };
            const auto& golf = getGameState().golf;
            auto near = [&](const HolePoint& p, int32_t tiles) {
                return std::abs((p.x >> 5) - centre.x) <= tiles && std::abs((p.y >> 5) - centre.y) <= tiles;
            };
            for (const auto& course : golf.courses())
            {
                for (const auto& hole : course.holes)
                {
                    for (const auto& tee : hole.tees)
                        if (tee && near(*tee, 10))
                            features.holesInView++;
                    if (hole.cup && near(*hole.cup, 10))
                        features.holesInView++;
                }
            }
            constexpr int32_t kLook = 6;
            for (int32_t y = centre.y - kLook; y <= centre.y + kLook; y++)
            {
                for (int32_t x = centre.x - kLook; x <= centre.x + kLook; x++)
                {
                    const CoordsXY at{ x * kCoordsXYStep, y * kCoordsXYStep };
                    if (!MapIsLocationValid(at))
                        continue;
                    if (const auto* surface = MapGetSurfaceElementAt(at); surface != nullptr && surface->getWaterHeight() > 0)
                        features.waterTiles++;
                    for (const auto* scenery : TileElementsView<SmallSceneryElement>(at))
                        if (!scenery->isGhost())
                            features.sceneryItems++;
                }
            }
            features.distanceToClubTiles = std::max(
                std::abs(static_cast<int32_t>(club.x.floor()) / kTileSize - centre.x),
                std::abs(static_cast<int32_t>(club.y.floor()) / kTileSize - centre.y));
            return features;
        }
    } // namespace

    // ECO-031: a cottage goes up on the lot (the buyer pays for it), the park is paid for the land, and the land is the VIP's.
    bool GolfRuntime::sellLot(CottageLot& lot, int32_t attractiveness)
    {
        auto& objects = GetContext()->GetObjectManager();
        std::vector<ObjectEntryIndex> cottages;
        for (ObjectEntryIndex i = 0; i < kMaxBuildingObjects; i++)
        {
            const auto* object = objects.GetLoadedObject<BuildingObject>(i);
            if (object != nullptr && object->Kind == BuildingKind::cottage)
                cottages.push_back(i);
        }
        if (cottages.empty())
            return false;
        // A random cottage of the loaded set; the first that fits the lot with its door on a path.
        const auto first = static_cast<size_t>(_rng->next() % cottages.size());
        for (size_t n = 0; n < cottages.size(); n++)
        {
            const auto index = cottages[(first + n) % cottages.size()];
            const auto* object = BuildingObject::GetById(index);
            for (Direction rotation = 0; rotation < 4; rotation++)
            {
                const int32_t fx = object->getFootprintX(rotation), fy = object->getFootprintY(rotation);
                for (int32_t y = lot.min.y; y + fy - 1 <= lot.max.y; y++)
                {
                    for (int32_t x = lot.min.x; x + fx - 1 <= lot.max.x; x++)
                    {
                        GameActions::BuildingPlaceAction place({ x * kCoordsXYStep, y * kCoordsXYStep }, index, rotation);
                        if (GameActions::QueryNested(&place, getGameState()).error != GameActions::Status::ok)
                            continue;
                        if (GameActions::ExecuteNested(&place, getGameState()).error != GameActions::Status::ok)
                            continue;

                        const int32_t tiles = lot.width() * lot.depth();
                        const int64_t price = lotPrice(_director->popularity(), attractiveness, tiles);
                        FinancePayment(-static_cast<money64>(price), ExpenditureType::landPurchase);
                        lot.sold = true;
                        lot.buildingObject = index;
                        lot.pricePaid = price;
                        lot.ownerGolferId = _director->addVip(lot.id);
                        // The land is no longer the park's: it cannot be changed (ECO-033), and balls do not play over it.
                        for (int32_t ty = lot.min.y; ty <= lot.max.y; ty++)
                        {
                            for (int32_t tx = lot.min.x; tx <= lot.max.x; tx++)
                            {
                                const CoordsXY at{ tx * kCoordsXYStep, ty * kCoordsXYStep };
                                if (auto* surface = MapGetSurfaceElementAt(at))
                                {
                                    surface->setOwnership(kUnowned);
                                    Park::UpdateFencesAroundTile(at);
                                    MapInvalidateTileFull(at);
                                }
                            }
                        }
                        return true;
                    }
                }
            }
        }
        return false;
    }

    // GLF-002, ECO-032: the owners of cottages come out of their doors to play, a few times a week, within the opening hours.
    void GolfRuntime::visitFromCottages(uint32_t ticks)
    {
        if (ticks % kTicksPerGameMinute != 0 || !acceptsArrivalsAt(_director->hours, minuteOfDayAt(ticks)))
            return;
        // kVipPlayChancePerMille a day, spread over the minutes of the opening hours.
        const int32_t openMinutes = std::max(1, _director->hours.close - _director->hours.open - _director->hours.lastArrivalBeforeClose);
        const int32_t perMillion = kVipPlayChancePerMille * 1000 / openMinutes;
        for (const auto& lot : getGameState().golf.lots())
        {
            if (!lot.sold || lot.ownerGolferId == 0)
                continue;
            if (static_cast<int32_t>(_rng->next() % 1000000u) >= perMillion)
                continue;
            // The door of the cottage on the lot.
            for (int32_t y = lot.min.y; y <= lot.max.y; y++)
            {
                for (int32_t x = lot.min.x; x <= lot.max.x; x++)
                {
                    const CoordsXY at{ x * kCoordsXYStep, y * kCoordsXYStep };
                    for (const auto* element : TileElementsView<BuildingElement>(at))
                    {
                        const auto* object = element->getObject();
                        if (object == nullptr || object->Kind != BuildingKind::cottage || !object->HasEntrance)
                            continue;
                        if (element->getPartX() != object->EntranceTileX || element->getPartY() != object->EntranceTileY)
                            continue;
                        _director->admitVip(lot.ownerGolferId, doorPosition(*element, *object, TileCoordsXY{ x, y }));
                        y = lot.max.y + 1;
                        x = lot.max.x + 1;
                        break;
                    }
                }
            }
        }
    }

    // Once a game day each lot that is not sold has a chance of being bought, from the popularity and how good the lot is.
    void GolfRuntime::sellCottages()
    {
        const auto day = _director->currentDay();
        if (day == _saleDay)
            return;
        const bool first = _saleDay == 0xFFFFFFFFu;
        _saleDay = day;
        if (first)
            return;
        for (auto& lot : getGameState().golf.lots())
        {
            if (lot.sold)
                continue;
            const auto attractiveness = lotAttractiveness(featuresOf(lot, _scan.club));
            const auto chance = salePerMillionPerDay(_director->popularity(), attractiveness);
            if (chance == 0 || static_cast<int32_t>(_rng->next() % 1000000u) >= chance)
                continue;
            sellLot(lot, attractiveness);
        }
    }

    // BAL-034, BAL-035: a golfer standing out on the course is a cylinder a ball can hit.
    void GolfRuntime::syncObstacles()
    {
        auto& obstacles = GetMapObstacles();
        std::vector<uint32_t> present;
        for (const auto& golfer : _director->simulation().golfers())
        {
            if (golfer.state == GolferState::inCar || golfer.state == GolferState::arriving)
                continue;
            Obstacle o;
            o.shape = ObstacleShape::cylinder;
            o.centre = golfer.position;
            o.halfX = Fixed::fromInt(3);
            o.halfY = o.halfX;
            o.height = Fixed::fromInt(16);
            o.restitution = Fixed::ratio(2, 10);
            o.friction = Fixed::ratio(5, 10);
            o.isGolfer = true;
            o.golferId = golfer.id;
            obstacles.setDynamic(golferObstacleId(golfer.id) - 0x80000000u, o);
            present.push_back(golfer.id);
        }
        for (const auto id : _golferObstacles)
        {
            if (std::find(present.begin(), present.end(), id) == present.end())
                obstacles.removeDynamic(golferObstacleId(id) - 0x80000000u);
        }
        _golferObstacles = std::move(present);
    }

    void GolfRuntime::refreshBuildings()
    {
        _scan = scanBuildings();
        refreshRanges();
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
        env.services = servicesWithSettings();
        env.benches = mergeBenches();
        env.bins = mergeBins();
        env.ballWashers = _scan.ballWashers;
        env.pathSigns = _scan.pathSigns;
    }

    std::vector<Service> GolfRuntime::servicesWithSettings() const
    {
        std::vector<Service> services;
        for (auto service : _scan.services)
        {
            const auto it = _director->buildings.find(service.id);
            if (it != _director->buildings.end())
            {
                if (it->second.closed)
                    continue;
                if (it->second.price >= 0)
                    service.price = it->second.price;
            }
            services.push_back(service);
        }
        return services;
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
        if (ticks % kTicksPerGameMinute == 0)
            refreshAnimals();
        driveAnimals();
        if (!_scan.hasClub)
            return;
        if (ticks % kTicksPerGameMinute == 0)
        {
            refreshTraffic();
            releaseStrandedGolfers();
        }
        _director->update(ticks);
        sellCottages();
        visitFromCottages(ticks);
        driveTraffic();
        for (const auto& event : _director->takeEvents())
            announce(event);
        syncEntities();
        syncBalls();
        syncObstacles();
        settleAccounts();
        payUpkeep();
    }

    GolfRuntime& runtimeOf(GameState_t& gameState)
    {
        if (gameState.golfRuntime == nullptr)
            gameState.golfRuntime = std::make_shared<GolfRuntime>(gameState);
        return *gameState.golfRuntime;
    }

    void GolfRuntime::spawnGolferGroup(int32_t tier)
    {
        auto& gameState = getGameState();
        uint32_t courseId = 1;
        if (auto* course = gameState.golf.findCourse(1))
            courseId = course->id;
        else if (!gameState.golf.courses().empty())
            courseId = gameState.golf.courses().front().id;

        FixedVec3 spawnPos{};
        if (_scan.hasClub)
        {
            spawnPos = _scan.club;
        }
        else
        {
            spawnPos = FixedVec3(Fixed::fromInt(50 * 32), Fixed::fromInt(50 * 32), Fixed{});
        }

        uint32_t group = _rng ? (_rng->next() % 10000 + 1) : 1;
        int32_t count = 3;
        for (int32_t i = 0; i < count; i++)
        {
            GolfSimulation::Spawn spawn;
            spawn.groupId = group;
            spawn.courseId = courseId;
            spawn.position = spawnPos;
            if (tier == 1) // Pro
            {
                if (_rng)
                    spawn.skills = professionalSkills(_director ? _director->tournament.tier : TournamentTier::local, *_rng);
                spawn.money = 100000;
                spawn.equipmentLevel = 3;
                spawn.professional = true;
                spawn.name = "Pro " + std::to_string(i + 1);
            }
            else if (tier == 2) // VIP
            {
                if (_rng)
                    spawn.skills = rollNewGolfer(*_rng, _director ? _director->popularity() : 300, true).skills;
                spawn.money = 500000;
                spawn.equipmentLevel = 2;
                spawn.vip = true;
                spawn.name = "VIP " + std::to_string(i + 1);
            }
            else // Regular
            {
                if (_rng)
                    spawn.skills = rollNewGolfer(*_rng, _director ? _director->popularity() : 300, false).skills;
                spawn.money = 20000;
                spawn.equipmentLevel = 1;
                spawn.name = "Golfer " + std::to_string(i + 1);
            }

            auto& g = _director->simulation().spawn(spawn);
            g.destination = spawnPos;
            g.state = GolferState::checkingIn;
        }
        syncEntities();
    }

    void GolfRuntime::spawnDrivingRangeCustomer()
    {
        refreshRanges();
        auto& ranges = getGameState().golf.ranges();
        if (ranges.empty())
            return;
        auto& range = ranges.front();
        HolePoint kioskPos{ range.areaMin.x * 32 + 16, range.areaMin.y * 32 + 16 };
        if (range.kiosk.has_value())
        {
            kioskPos = *range.kiosk;
        }
        else if (!_scan.kiosks.empty())
        {
            kioskPos = _scan.kiosks.front();
            range.kiosk = kioskPos;
        }

        GolfSimulation::Spawn spawn;
        spawn.groupId = _rng ? (_rng->next() % 10000 + 1) : 2;
        spawn.position = FixedVec3(Fixed::fromInt(kioskPos.x), Fixed::fromInt(kioskPos.y), Fixed{});
        if (_rng)
            spawn.skills = rollNewGolfer(*_rng, _director ? _director->popularity() : 300, false).skills;
        spawn.money = 15000;
        spawn.equipmentLevel = 1;
        spawn.name = "Range Customer";

        auto& g = _director->simulation().spawn(spawn);
        g.rangeId = range.id;
        g.rangeVisited = true;
        g.destination = spawn.position;
        g.state = GolferState::walkingToRange;
        syncEntities();
    }

    void GolfRuntime::evaluateVipCottages()
    {
        auto& gameState = getGameState();
        for (auto& lot : gameState.golf.lots())
        {
            if (lot.sold)
                continue;
            const auto attractiveness = lotAttractiveness(featuresOf(lot, _scan.club));
            if (sellLot(lot, attractiveness))
                break;
            lot.sold = true;
            lot.pricePaid = lotPrice(_director ? _director->popularity() : 300, attractiveness, lot.width() * lot.depth());
            if (_director)
                lot.ownerGolferId = _director->addVip(lot.id);
            break;
        }
    }

    void GolfRuntime::startOrAdvanceTournament(int32_t tier)
    {
        if (_director == nullptr)
            return;

        if (_director->tournament.phase == TournamentPhase::none)
        {
            _director->tournament.tier = static_cast<TournamentTier>(tier % static_cast<int32_t>(TournamentTier::count));
            _director->tournament.phase = TournamentPhase::running;
            _director->tournament.professionals = kProfessionalFieldSize;
            _director->tournament.day = _director->currentDay();

            uint32_t courseId = 1;
            if (!getGameState().golf.courses().empty())
                courseId = getGameState().golf.courses().front().id;

            FixedVec3 spawnPos{};
            if (_scan.hasClub)
                spawnPos = _scan.club;
            else
                spawnPos = FixedVec3(Fixed::fromInt(50 * 32), Fixed::fromInt(50 * 32), Fixed{});

            for (int32_t i = 0; i < kProfessionalFieldSize; i++)
            {
                GolfSimulation::Spawn spawn;
                spawn.groupId = 2000 + i / 4;
                spawn.courseId = courseId;
                if (_rng)
                    spawn.skills = professionalSkills(_director->tournament.tier, *_rng);
                spawn.professional = true;
                spawn.equipmentLevel = 3;
                spawn.name = "Tour Pro #" + std::to_string(i + 1);
                spawn.position = spawnPos;
                auto& g = _director->simulation().spawn(spawn);
                g.destination = spawnPos;
                g.state = GolferState::checkingIn;
            }
            syncEntities();
        }
        else if (_director->tournament.phase == TournamentPhase::running)
        {
            _director->tournament.phase = TournamentPhase::finished;
            _director->tournament.result = 85;
            _director->tournament.popularityGained = 50;
            _director->setPopularity(_director->popularity() + 50);
        }
        else
        {
            _director->tournament.phase = TournamentPhase::none;
        }
    }

    void GolfRuntime::setWeatherAndWind(Weather::Type weather, int16_t directionDeg, uint16_t strength)
    {
        auto& gameState = getGameState();
        gameState.weatherCurrent.weatherType = weather;
        gameState.wind.direction = directionDeg;
        gameState.wind.targetDirection = directionDeg;
        gameState.wind.strength = strength;
        gameState.wind.targetStrength = strength;
        _world.setWind(windVelocity(gameState.wind));
    }

    void GolfRuntime::spawnWildlife(int32_t speciesIndex, int32_t nearX, int32_t nearY)
    {
        if (_director == nullptr || _director->animals == nullptr)
            return;

        refreshAnimals();
        if (_director->animals->species().empty())
        {
            AnimalSpecies defaultDuck;
            defaultDuck.id = "golf.animal.duck";
            defaultDuck.habitat = kHabitatGreenBank | kHabitatRough | kHabitatFairway | kHabitatWoods | kHabitatWater;
            defaultDuck.minGroup = 1;
            defaultDuck.maxGroup = 3;
            defaultDuck.canFly = true;
            defaultDuck.canSwim = true;
            defaultDuck.avoidsStreets = false;
            _director->animals->addSpecies(defaultDuck);
        }

        FixedVec3 centre{};
        if (nearX >= 0 && nearY >= 0)
        {
            centre = FixedVec3(Fixed::fromInt(nearX), Fixed::fromInt(nearY), Fixed{});
        }
        else if (_scan.hasClub)
        {
            centre = _scan.club;
        }
        else
        {
            centre = FixedVec3(Fixed::fromInt(50 * 32), Fixed::fromInt(50 * 32), Fixed{});
        }

        const auto& species = _director->animals->species();
        if (species.empty())
            return;

        uint32_t sp = (speciesIndex >= 0 && speciesIndex < static_cast<int32_t>(species.size()))
            ? static_cast<uint32_t>(speciesIndex)
            : (_rng ? (_rng->next() % species.size()) : 0);

        int32_t spawned = _director->animals->spawnGroup(sp, centre, 32 * 32);
        if (spawned == 0)
        {
            for (const auto& course : getGameState().golf.courses())
            {
                for (const auto& hole : course.holes)
                {
                    if (hole.tees[0].has_value())
                    {
                        spawned = _director->animals->spawnGroup(
                            sp, FixedVec3(Fixed::fromInt(hole.tees[0]->x), Fixed::fromInt(hole.tees[0]->y), Fixed{}), 32 * 32);
                        if (spawned > 0)
                            break;
                    }
                    if (hole.cup.has_value())
                    {
                        spawned = _director->animals->spawnGroup(
                            sp, FixedVec3(Fixed::fromInt(hole.cup->x), Fixed::fromInt(hole.cup->y), Fixed{}), 32 * 32);
                        if (spawned > 0)
                            break;
                    }
                }
                if (spawned > 0)
                    break;
            }
        }
        driveAnimals();
        syncEntities();
    }

    GolfRuntime* GetGolfRuntime()
    {
        return &runtimeOf(getGameState());
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

    void SetupEmptyGolfSandbox(GameState_t& gameState)
    {
        static constexpr TileCoordsXY kDefaultMapSize{ 150, 150 };
        gameStateInitAll(gameState, kDefaultMapSize);

        auto* context = GetContext();
        auto& objectManager = context->GetObjectManager();
        for (const auto& entry : kCommonScenarioAndTrackDesignerObjects)
        {
            objectManager.LoadObject(entry);
        }

        gameState.park.cash = 100000.00_GBP;
        gameState.park.flags.set(ParkFlag::parkOpen);
        gameState.park.flags.set(ParkFlag::freeEntry);
        gameState.park.name = "Golf Sandbox";

        for (int32_t y = 1; y < kDefaultMapSize.y - 1; y++)
        {
            for (int32_t x = 1; x < kDefaultMapSize.x - 1; x++)
            {
                auto* surface = MapGetSurfaceElementAt(TileCoordsXY{ x, y });
                if (surface != nullptr)
                {
                    surface->setOwnership(OwnershipFlag::landOwned);
                }
            }
        }
        Park::UpdateSize(gameState.park);

        gameState.savedView = ScreenCoordsXY{ 75 * 32, 75 * 32 };
        gameState.savedViewZoom = ZoomLevel{ 0 };
        gameState.savedViewRotation = 0;
    }

    void SetupDemoGolfCourse(GameState_t& gameState)
    {
        SetupEmptyGolfSandbox(gameState);
        gameState.park.cash = 50000.00_GBP;
        gameState.park.name = "Pinewood Golf Club";

        auto* context = GetContext();
        auto& objectManager = context->GetObjectManager();

        auto getObjIndex = [&](std::string_view id) -> ObjectEntryIndex {
            auto* obj = objectManager.LoadObject(id);
            return obj != nullptr ? objectManager.GetLoadedObjectEntryIndex(obj) : kObjectEntryIndexNull;
        };

        auto roadIndex = getObjIndex("golf.street.road");
        auto clubIndex = getObjIndex("golf.building.club_house");
        auto cartIndex = getObjIndex("golf.building.cart_rental");
        auto kioskIndex = getObjIndex("golf.building.driving_range_kiosk");
        auto fairwayIndex = getObjIndex("golf.terrain_surface.fairway");
        auto greenIndex = getObjIndex("golf.terrain_surface.green");
        auto teeSurfIndex = getObjIndex("golf.terrain_surface.tee");
        auto sandIndex = getObjIndex("golf.terrain_surface.sand");
        auto carpetIndex = getObjIndex("golf.terrain_surface.mini_putt_carpet");

        auto paintRect = [&](int32_t minX, int32_t minY, int32_t maxX, int32_t maxY, ObjectEntryIndex surf) {
            if (surf == kObjectEntryIndexNull)
                return;
            for (int32_t y = minY; y <= maxY; y++)
            {
                for (int32_t x = minX; x <= maxX; x++)
                {
                    auto* surface = MapGetSurfaceElementAt(TileCoordsXY{ x, y });
                    if (surface != nullptr)
                    {
                        surface->setSurfaceObjectIndex(surf);
                    }
                }
            }
        };

        auto addPath = [](int32_t x, int32_t y) {
            auto* surface = MapGetSurfaceElementAt(TileCoordsXY{ x, y });
            if (surface != nullptr)
            {
                const auto baseZ = surface->getBaseZ();
                auto* path = TileElementInsert<PathElement>(CoordsXYZ{ x * 32, y * 32, baseZ }, 0b1111);
                if (path != nullptr)
                {
                    path->setClearanceZ(baseZ + 16);
                }
            }
        };

        // 1. Road from map edge (x=1) to parking lot
        const int32_t roadRow = 30;
        if (roadIndex != kObjectEntryIndexNull)
        {
            for (int32_t x = 1; x <= 16; x++)
            {
                GameActions::StreetPlaceAction street(CoordsXY{ x * 32, roadRow * 32 }, roadIndex);
                GameActions::ExecuteNested(&street, gameState);
            }
            GameActions::StreetRoadEntranceAction entrance(CoordsXY{ 1 * 32, roadRow * 32 }, true);
            GameActions::ExecuteNested(&entrance, gameState);

            // Parking spaces (x = 8..15, y = 31..32)
            for (int32_t x = 8; x <= 15; x++)
            {
                GameActions::ParkingPlaceAction parking(CoordsXY{ x * 32, (roadRow + 1) * 32 }, roadIndex, 3);
                GameActions::ExecuteNested(&parking, gameState);
            }
        }

        // Footpath connecting parking to buildings, entrance doors and cottage lots
        for (int32_t y = roadRow; y <= 45; y++)
        {
            addPath(7, y);
        }
        for (int32_t x = 7; x <= 22; x++)
        {
            addPath(x, 35);
        }
        for (int32_t x = 7; x <= 8; x++)
        {
            addPath(x, 42);
        }

        // 2. Buildings
        // Golf Club (footprint 3x2, origin at { 10 * 32, 36 * 32 }, entrance at y=35)
        if (clubIndex != kObjectEntryIndexNull)
        {
            GameActions::BuildingPlaceAction placeClub(CoordsXY{ 10 * 32, 36 * 32 }, clubIndex, 0);
            GameActions::ExecuteNested(&placeClub, gameState);
        }
        // Cart Rental (footprint 2x1, origin at { 14 * 32, 36 * 32 }, entrance at y=35)
        if (cartIndex != kObjectEntryIndexNull)
        {
            GameActions::BuildingPlaceAction placeCart(CoordsXY{ 14 * 32, 36 * 32 }, cartIndex, 0);
            GameActions::ExecuteNested(&placeCart, gameState);
        }
        // Driving Range Kiosk (footprint 1x1, origin at { 18 * 32, 36 * 32 }, entrance at y=35)
        if (kioskIndex != kObjectEntryIndexNull)
        {
            GameActions::BuildingPlaceAction placeKiosk(CoordsXY{ 18 * 32, 36 * 32 }, kioskIndex, 0);
            GameActions::ExecuteNested(&placeKiosk, gameState);
        }

        // Driving range zone
        gameState.golf.createRange({ 18, 20 }, { 28, 36 });

        // Cottage lots
        gameState.golf.createLot({ 9, 41 }, { 13, 45 });

        // 3. Courses & Holes
        auto* regular = gameState.golf.createCourse("Championship Course");
        if (regular != nullptr)
        {
            regular->open = true;
            regular->greenFee = 2500;
            regular->kind = CourseKind::regular;

            // Hole 1: Par 4
            auto* hole1 = gameState.golf.createHole(regular->id);
            if (hole1 != nullptr)
            {
                hole1->tees[0] = HolePoint{ 30 * 32 + 16, 38 * 32 + 16 };
                hole1->aimPoints[0] = HolePoint{ 45 * 32 + 16, 38 * 32 + 16 };
                hole1->cup = HolePoint{ 60 * 32 + 16, 38 * 32 + 16 };
                hole1->parOverride = 4;
                paintRect(29, 37, 31, 39, teeSurfIndex);
                paintRect(32, 37, 57, 39, fairwayIndex);
                paintRect(45, 40, 47, 41, sandIndex);
                paintRect(58, 36, 62, 40, greenIndex);
            }

            // Hole 2: Par 3
            auto* hole2 = gameState.golf.createHole(regular->id);
            if (hole2 != nullptr)
            {
                hole2->tees[0] = HolePoint{ 62 * 32 + 16, 43 * 32 + 16 };
                hole2->cup = HolePoint{ 78 * 32 + 16, 43 * 32 + 16 };
                hole2->parOverride = 3;
                paintRect(61, 42, 63, 44, teeSurfIndex);
                paintRect(64, 42, 75, 44, fairwayIndex);
                paintRect(72, 45, 74, 46, sandIndex);
                paintRect(76, 41, 80, 45, greenIndex);
            }

            // Hole 3: Par 5
            auto* hole3 = gameState.golf.createHole(regular->id);
            if (hole3 != nullptr)
            {
                hole3->tees[0] = HolePoint{ 78 * 32 + 16, 48 * 32 + 16 };
                hole3->aimPoints[0] = HolePoint{ 58 * 32 + 16, 55 * 32 + 16 };
                hole3->cup = HolePoint{ 38 * 32 + 16, 55 * 32 + 16 };
                hole3->parOverride = 5;
                paintRect(77, 47, 79, 49, teeSurfIndex);
                paintRect(55, 50, 76, 53, fairwayIndex);
                paintRect(39, 54, 57, 56, fairwayIndex);
                paintRect(35, 53, 38, 57, greenIndex);
            }
        }

        // Mini-Putt Course
        auto* mini = gameState.golf.createCourse("Mini-Putt Course");
        if (mini != nullptr)
        {
            mini->open = true;
            mini->greenFee = 800;
            mini->kind = CourseKind::miniPutt;

            auto* miniHole = gameState.golf.createHole(mini->id);
            if (miniHole != nullptr)
            {
                miniHole->tees[0] = HolePoint{ 12 * 32 + 16, 48 * 32 + 16 };
                miniHole->cup = HolePoint{ 18 * 32 + 16, 48 * 32 + 16 };
                miniHole->parOverride = 2;
                paintRect(12, 48, 18, 48, carpetIndex);
            }
        }

        gameState.savedView = ScreenCoordsXY{ 12 * 32, 37 * 32 };
        gameState.savedViewZoom = ZoomLevel{ 0 };
        gameState.savedViewRotation = 0;
    }
} // namespace OpenRCT2::Golf
