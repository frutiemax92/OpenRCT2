/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "GolfSimulation.h"
#include "../world/Stream.h"

#include <algorithm>

namespace OpenRCT2::Golf
{
    namespace
    {
        const Fixed kOne = Fixed::fromInt(1);

        bool isWalkingState(GolferState s)
        {
            switch (s)
            {
                case GolferState::walkingToCheckIn:
                case GolferState::goingToCart:
                case GolferState::walkingToRange:
                case GolferState::goingToTee:
                case GolferState::walkingToBall:
                case GolferState::returningCart:
                case GolferState::leaving:
                case GolferState::rageQuitting:
                    return true;
                default:
                    return false;
            }
        }

        uint64_t groupKey(uint32_t group, uint32_t holeIndex)
        {
            return (static_cast<uint64_t>(group) << 32) | holeIndex;
        }

        Fixed horizontal(const FixedVec3& a, const FixedVec3& b)
        {
            return length(FixedVec3{ a.x - b.x, a.y - b.y, Fixed{} });
        }

        // The tee a golfer plays a hole from: their category when the hole has it, otherwise the first one it has.
        std::optional<HolePoint> teeFor(const Hole& hole, TeeCategory category)
        {
            if (hole.tees[static_cast<size_t>(category)].has_value())
                return hole.tees[static_cast<size_t>(category)];
            if (const auto first = hole.firstTee())
                return hole.tees[static_cast<size_t>(*first)];
            return std::nullopt;
        }
    } // namespace

    namespace
    {
        // The ball world of the golfers: the world they were given, with the cups of the holes (HOL-003) added to it.
        class CupWorld final : public IBallWorld
        {
        public:
            CupWorld(const IBallWorld* inner, const CourseManager* courses)
                : _inner(inner)
                , _courses(courses)
            {
            }
            SurfaceSample surfaceAt(Fixed x, Fixed y) const override
            {
                return _inner->surfaceAt(x, y);
            }
            bool obstacleAt(const FixedVec3& position, Fixed radius, ObstacleContact& contact) const override
            {
                return _inner->obstacleAt(position, radius, contact);
            }
            FixedVec3 wind() const override
            {
                return _inner->wind();
            }
            bool cupNear(Fixed x, Fixed y, CupInfo& cup) const override
            {
                if (_inner->cupNear(x, y, cup))
                    return true;
                if (_courses == nullptr)
                    return false;
                for (const auto& course : _courses->courses())
                    for (const auto& hole : course.holes)
                    {
                        if (!hole.cup.has_value())
                            continue;
                        const Fixed cx = Fixed::fromInt(hole.cup->x), cy = Fixed::fromInt(hole.cup->y);
                        if (abs(x - cx) > Fixed::fromInt(8) || abs(y - cy) > Fixed::fromInt(8))
                            continue;
                        cup.position = { cx, cy, _inner->surfaceAt(cx, cy).height };
                        cup.radius = Fixed::fromInt(3);
                        return true;
                    }
                return false;
            }

        private:
            const IBallWorld* _inner;
            const CourseManager* _courses;
        };
    } // namespace

    GolfSimulation::GolfSimulation(Environment environment)
        : _env(std::move(environment))
    {
        _rawWorld = _env.world;
        _cupWorld = std::make_shared<CupWorld>(_rawWorld, _env.courses);
        _env.world = _cupWorld.get();
    }

    Fixed GolfSimulation::groundZ(Fixed x, Fixed y) const
    {
        return _env.world != nullptr ? _env.world->surfaceAt(x, y).height : Fixed{};
    }

    Golfer* GolfSimulation::find(uint32_t id)
    {
        for (auto& g : _golfers)
            if (g.id == id)
                return &g;
        return nullptr;
    }

    std::vector<SimEvent> GolfSimulation::takeEvents()
    {
        std::vector<SimEvent> events;
        events.swap(_events);
        return events;
    }

    std::vector<Golfer> GolfSimulation::takeLeft()
    {
        std::vector<Golfer> left;
        left.swap(_left);
        return left;
    }

    TeeSheet& GolfSimulation::teeSheet(uint32_t courseId)
    {
        auto it = _sheets.find(courseId);
        if (it == _sheets.end())
        {
            const auto* course = _env.courses != nullptr ? _env.courses->findCourse(courseId) : nullptr;
            it = _sheets.emplace(courseId, TeeSheet(course != nullptr ? course->holes.size() : 0)).first;
        }
        return it->second;
    }

    const HoleDifficulty& GolfSimulation::difficultyOf(uint32_t courseId, uint32_t holeId)
    {
        const uint64_t key = groupKey(courseId, holeId);
        auto it = _difficulty.find(key);
        if (it == _difficulty.end())
        {
            HoleDifficulty d;
            const auto* course = _env.courses != nullptr ? _env.courses->findCourse(courseId) : nullptr;
            if (course != nullptr && _env.world != nullptr)
                if (const auto* hole = course->findHole(holeId))
                    d = rateHole(*hole, *_env.world);
            it = _difficulty.emplace(key, d).first;
        }
        return it->second;
    }

    const Course* GolfSimulation::courseOf(const Golfer& g) const
    {
        return _env.courses != nullptr ? _env.courses->findCourse(g.courseId) : nullptr;
    }

    const Hole* GolfSimulation::holeOf(const Golfer& g) const
    {
        const auto* course = courseOf(g);
        return course != nullptr && g.holeIndex < course->holes.size() ? &course->holes[g.holeIndex] : nullptr;
    }

    void GolfSimulation::emit(SimEvent::Type type, const Golfer& g, int32_t value)
    {
        SimEvent e;
        e.type = type;
        e.golferId = g.id;
        e.value = value;
        _events.push_back(e);
    }

    void GolfSimulation::think(Golfer& g, ThoughtType thought)
    {
        g.think(thought, _now);
        SimEvent e;
        e.type = SimEvent::Type::thought;
        e.golferId = g.id;
        e.thought = thought;
        _events.push_back(e);
    }

    void GolfSimulation::setState(Golfer& g, GolferState state)
    {
        g.state = state;
        g.stateTicks = 0;
    }

    Golfer& GolfSimulation::spawn(const Spawn& s)
    {
        Golfer g;
        g.id = _nextId++;
        g.name = s.name.empty() ? "Golfer " + std::to_string(g.id) : s.name;
        g.groupId = s.groupId != 0 ? s.groupId : g.id;
        g.skills = SkillPoints::from(s.skills);
        g.initialSkills = g.skills;
        g.equipmentLevel = s.equipmentLevel;
        g.money = s.money;
        g.vip = s.vip;
        g.riskTolerance = s.riskTolerance;
        g.courseId = s.courseId;
        g.tee = s.tee;
        g.position = s.position;
        g.destination = _env.clubEntrance;
        g.arrivalDay = gameDayAt(_now);
        if (_env.rng != nullptr)
        {
            // GLF-003: needs at the start of the round.
            g.needs.hunger = static_cast<int32_t>(200 + _env.rng->next() % 56) * 16;
            g.needs.thirst = static_cast<int32_t>(200 + _env.rng->next() % 56) * 16;
            g.needs.bladder = static_cast<int32_t>(_env.rng->next() % 60) * 16;
            g.needs.energy = 255 * 16;
        }
        if (const auto* course = courseOf(g))
        {
            g.scorecard.assign(course->holes.size(), 127);
            g.parCard.assign(course->holes.size(), 0);
        }
        _golfers.push_back(std::move(g));
        return _golfers.back();
    }

    bool GolfSimulation::putInCar(uint32_t id)
    {
        auto* g = find(id);
        if (g == nullptr)
            return false;
        setState(*g, GolferState::inCar);
        return true;
    }

    bool GolfSimulation::release(uint32_t id, const FixedVec3& position, const FixedVec3& carPosition)
    {
        auto* g = find(id);
        if (g == nullptr || g->state != GolferState::inCar)
            return false;
        g->position = position;
        g->hasExit = true;
        g->exitPosition = carPosition;
        setState(*g, GolferState::arriving);
        return true;
    }

    bool GolfSimulation::remove(uint32_t id)
    {
        for (size_t i = 0; i < _golfers.size(); i++)
        {
            if (_golfers[i].id == id)
            {
                _golfers.erase(_golfers.begin() + static_cast<std::ptrdiff_t>(i));
                return true;
            }
        }
        return false;
    }

    // DRV-005: a driving range with a kiosk and a tile to stand on, for a golfer who can pay for a bucket.
    void GolfSimulation::syncRanges()
    {
        if (_env.courses == nullptr)
            return;
        for (const auto& range : _env.courses->ranges())
        {
            auto it = _ranges.find(range.id);
            if (it == _ranges.end())
            {
                _ranges.emplace(range.id, std::make_unique<RangeSimulation>(_env.world, _env.rng, range));
                continue;
            }
            it->second->updateRange(range);
        }
        for (auto it = _ranges.begin(); it != _ranges.end();)
        {
            if (_env.courses->findRange(it->first) == nullptr)
                it = _ranges.erase(it);
            else
                ++it;
        }
    }

    RangeSimulation* GolfSimulation::rangeOf(const Golfer& g)
    {
        if (g.rangeId == 0)
            return nullptr;
        const auto it = _ranges.find(g.rangeId);
        return it == _ranges.end() ? nullptr : it->second.get();
    }

    // Golfers who are not yet good with the long clubs like to warm up before a round more than those who are.
    bool GolfSimulation::wantsRange(const Golfer& g)
    {
        if (_env.rng == nullptr || g.rangeVisited)
            return false;
        const int32_t longPlay = g.skills.toSkills().longPlay;
        const int32_t percent = 10 + (100 - std::clamp(longPlay, 0, 100)) / 3;
        return static_cast<int32_t>(_env.rng->next() % 100) < percent;
    }

    void GolfSimulation::visitRange(Golfer& g, const DrivingRange& range)
    {
        g.rangeId = range.id;
        g.rangeVisited = true;
        g.destination = { Fixed::fromInt(range.kiosk->x), Fixed::fromInt(range.kiosk->y), Fixed{} };
        setState(g, GolferState::walkingToRange);
    }

    // The visit is over, or never happened: the round begins with the cart, then the first tee.
    void GolfSimulation::endRangeVisit(Golfer& g)
    {
        if (auto* range = rangeOf(g))
            range->leave(g.id);
        g.rangeId = 0;
        const auto* course = courseOf(g);
        if (course == nullptr)
        {
            leave(g);
            return;
        }
        beginRound(g, *course);
    }

    // The cart is chosen (CRT-003) and fetched, then the golfer goes to the first tee.
    void GolfSimulation::beginRound(Golfer& g, const Course& course)
    {
        decideCarts(g, course);
        if (auto* cart = cartOf(g))
        {
            g.destination = cart->position;
            setState(g, GolferState::goingToCart);
            return;
        }
        startHole(g);
    }

    Cart* GolfSimulation::cartOf(const Golfer& g) const
    {
        return g.cartId != 0 && _env.carts != nullptr ? _env.carts->find(g.cartId) : nullptr;
    }

    // CRT-003: the first of a group to check in decides for all of it who takes a cart; riders pay the cart fee, go two to
    // a cart, and the others walk.
    void GolfSimulation::decideCarts(Golfer& first, const Course& course)
    {
        // Without carts nobody expects one, so no one minds walking.
        if (_env.carts == nullptr || _env.carts->cartCount() == 0 || first.cartDecided)
            return;
        std::vector<Golfer*> members;
        for (auto& other : _golfers)
        {
            if (other.groupId == first.groupId && !other.finished && other.state != GolferState::leaving
                && other.state != GolferState::rageQuitting && other.state != GolferState::inCar)
            {
                other.cartDecided = true;
                members.push_back(&other);
            }
        }
        const int32_t yards = course.totalYards(first.tee);
        std::vector<CartFleet::Rider> riders;
        std::vector<Golfer*> wanting;
        for (auto* member : members)
        {
            const int32_t jitter = _env.rng != nullptr ? static_cast<int32_t>(_env.rng->next() % 100) : 0;
            if (wantsCart(
                    member->needs.energyByte(), member->skills.toSkills().physique, yards, member->money, _env.carts->cartFee, jitter))
            {
                riders.push_back({ member->id, member->money });
                wanting.push_back(member);
            }
        }
        if (riders.empty())
            return;
        const auto rental = _env.carts->rent(riders, first.groupId, _env.clubEntrance);
        for (const auto cartId : rental.cartIds)
        {
            const auto* cart = _env.carts->find(cartId);
            for (const auto rider : cart->riders)
            {
                if (auto* g = find(rider))
                {
                    g->cartId = cartId;
                    g->money -= _env.carts->cartFee;
                    _income += _env.carts->cartFee;
                    emit(SimEvent::Type::paidCartFee, *g, static_cast<int32_t>(_env.carts->cartFee));
                }
            }
        }
        // CRT-003: those who wanted a cart and found none walk, and mind it.
        for (const auto id : rental.walkers)
        {
            if (auto* g = find(id))
            {
                applyHappiness(g->needs, HappinessEvent::noCart);
                think(*g, ThoughtType::needCart);
            }
        }
    }

    // The golfer takes the wheel (or sits beside whoever does): the cart is where its first rider stands.
    void GolfSimulation::board(Golfer& g)
    {
        auto* cart = cartOf(g);
        if (cart == nullptr)
        {
            g.onCart = false;
            return;
        }
        g.onCart = true;
        if (!cart->riders.empty() && cart->riders.front() == g.id)
        {
            // The cart leaves its space (if it is in one) and picks the driver up where they stand.
            _env.carts->depart(cart->id);
            cart->position = g.position;
            cart->speed = Fixed{};
            if (cart->state != CartState::returning)
                cart->state = CartState::driving;
        }
    }

    void GolfSimulation::dismount(Golfer& g)
    {
        g.onCart = false;
        if (auto* cart = cartOf(g))
        {
            if (!cart->riders.empty() && cart->riders.front() == g.id)
            {
                cart->speed = Fixed{};
                cart->state = CartState::waitingForGolfers;
            }
        }
    }

    // CRT-004: the cart drives, and its riders go with it. The first rider drives; the other keeps the seat beside them.
    bool GolfSimulation::ride(Golfer& g, const FixedVec3& to)
    {
        auto* cart = cartOf(g);
        if (cart == nullptr)
        {
            g.onCart = false;
            return walk(g, to);
        }
        // CRT-007: a flat battery strands the cart; its riders walk on and the cart goes back to be charged.
        if (cart->battery == 0)
        {
            const auto riders = cart->riders;
            for (const auto id : riders)
                if (auto* rider = find(id))
                    abandonCart(*rider);
            g.onCart = false;
            g.cartId = 0;
            return walk(g, to);
        }
        bool arrived = false;
        if (!cart->riders.empty() && cart->riders.front() == g.id)
        {
            cart->destination = to;
            arrived = _env.carts->drive(*cart, to);
        }
        else
        {
            arrived = horizontal(cart->position, cart->destination) <= Fixed::fromInt(kArriveDistanceUnits);
        }
        g.position = { cart->position.x, cart->position.y, groundZ(cart->position.x, cart->position.y) };
        return arrived;
    }

    // CRT-005: after the last hole the cart goes back to the nearest free space; with none free it is simply left.
    void GolfSimulation::returnCartAndLeave(Golfer& g)
    {
        auto* cart = cartOf(g);
        if (cart == nullptr)
        {
            g.cartId = 0;
            g.onCart = false;
            leave(g);
            return;
        }
        const bool leader = !cart->riders.empty() && cart->riders.front() == g.id;
        if (leader && cart->state != CartState::returning && !_env.carts->returnCart(cart->id))
        {
            // Every space is taken: the cart stays where it is, and so does its rider's trouble.
            abandonCart(g);
            leave(g);
            return;
        }
        board(g);
        g.destination = cart->destination;
        setState(g, GolferState::returningCart);
    }

    // A golfer who leaves for good (rage quit, no space to return the cart to) gives up the cart: it goes back by itself.
    void GolfSimulation::abandonCart(Golfer& g)
    {
        auto* cart = cartOf(g);
        g.onCart = false;
        if (cart == nullptr)
        {
            g.cartId = 0;
            return;
        }
        cart->riders.erase(std::remove(cart->riders.begin(), cart->riders.end(), g.id), cart->riders.end());
        g.cartId = 0;
        if (cart->riders.empty() && cart->state != CartState::parked)
        {
            if (cart->state != CartState::returning)
            {
                if (!_env.carts->returnCart(cart->id))
                    return; // every space is taken: it stays where it stands
            }
            _env.carts->park(cart->id);
        }
    }

    bool GolfSimulation::walk(Golfer& g, const FixedVec3& to)
    {
        if (g.onCart)
            return ride(g, to);
        const Fixed speed = Fixed::ratio(kWalkSpeedHundredths, 100);
        const FixedVec3 delta{ to.x - g.position.x, to.y - g.position.y, Fixed{} };
        const Fixed distance = length(delta);
        if (distance <= Fixed::fromInt(kArriveDistanceUnits) || distance <= speed)
        {
            const CoordsXY currTile{ static_cast<int32_t>(g.position.x.floor()), static_cast<int32_t>(g.position.y.floor()) };
            const CoordsXY destTile{ static_cast<int32_t>(to.x.floor()), static_cast<int32_t>(to.y.floor()) };
            if (!Stream::CanMove(currTile, destTile, false))
            {
                return false;
            }
            g.position = { to.x, to.y, groundZ(to.x, to.y) };
            return true;
        }
        const Fixed nextX = g.position.x + delta.x * speed / distance;
        const Fixed nextY = g.position.y + delta.y * speed / distance;
        const CoordsXY currTile{ static_cast<int32_t>(g.position.x.floor()), static_cast<int32_t>(g.position.y.floor()) };
        const CoordsXY nextTile{ static_cast<int32_t>(nextX.floor()), static_cast<int32_t>(nextY.floor()) };
        if (!Stream::CanMove(currTile, nextTile, false))
        {
            return false;
        }
        g.position.x = nextX;
        g.position.y = nextY;
        g.position.z = groundZ(g.position.x, g.position.y);
        return false;
    }

    bool GolfSimulation::groupBusy(const Golfer& g) const
    {
        for (const auto& other : _golfers)
        {
            if (other.id == g.id || other.groupId != g.groupId)
                continue;
            if (other.state == GolferState::swinging || other.state == GolferState::putting
                || other.state == GolferState::watchingBall)
                return true;
        }
        return false;
    }

    void GolfSimulation::startHole(Golfer& g)
    {
        const auto* hole = holeOf(g);
        if (hole == nullptr)
        {
            leave(g);
            return;
        }
        g.score = HoleScore{};
        g.score.par = hole->par();
        g.waitedMinutes = 0;
        g.ballInPlay = false;
        g.searchedBall = false;
        const auto tee = teeFor(*hole, g.tee);
        g.destination = tee.has_value() ? FixedVec3{ Fixed::fromInt(tee->x), Fixed::fromInt(tee->y), Fixed{} } : g.position;
        setState(g, GolferState::goingToTee);
    }

    void GolfSimulation::teeOff(Golfer& g)
    {
        const auto* hole = holeOf(g);
        const auto tee = teeFor(*hole, g.tee);
        const FixedVec3 at = tee.has_value() ? FixedVec3{ Fixed::fromInt(tee->x), Fixed::fromInt(tee->y), Fixed{} } : g.position;
        placeBall(g.ball, *_env.world, at.x, at.y, BallState::Teed);
        g.ballInPlay = true;
        g.holeStartMinute = minuteNow();
        g.position = { at.x, at.y, groundZ(at.x, at.y) };
        emit(SimEvent::Type::teedOff, g);
        setState(g, GolferState::preparingShot);
    }

    bool GolfSimulation::seekFacility(Golfer& g, NeedKind need)
    {
        const Service* best = nullptr;
        Fixed bestDistance = Fixed::fromInt(1000000);
        for (const auto& service : _env.services)
        {
            const bool serves = (need == NeedKind::hunger && service.food) || (need == NeedKind::thirst && service.drink)
                || (need == NeedKind::bladder && service.toilet) || (need == NeedKind::energy && service.rest);
            if (!serves)
                continue;
            const Fixed d = horizontal(service.position, g.position);
            if (d < bestDistance)
            {
                best = &service;
                bestDistance = d;
            }
        }

        // PTH-021: a golfer with low energy prefers or uses a path bench if available.
        if (need == NeedKind::energy && !_env.benches.empty())
        {
            Bench* bestBench = nullptr;
            Fixed bestBenchDist = Fixed::fromInt(1000000);
            for (auto& b : _env.benches)
            {
                if (b.occupants >= Bench::kMaxOccupants)
                    continue;
                const Fixed d = horizontal(b.position, g.position);
                if (d < bestBenchDist)
                {
                    bestBench = &b;
                    bestBenchDist = d;
                }
            }
            if (bestBench != nullptr && (best == nullptr || bestBenchDist < bestDistance))
            {
                bestBench->occupants++;
                g.restingBenchId = bestBench->id;
                g.destination = bestBench->position;
                g.facilityTicksLeft = kWalkingToFacility;
                setState(g, GolferState::resting);
                return true;
            }
        }

        if (best == nullptr)
            return false;
        g.facilityNeed = need;
        g.facilityService = best->id;
        g.destination = best->position;
        g.facilityTicksLeft = kWalkingToFacility;
        setState(g, GolferState::usingFacility);
        return true;
    }

    void GolfSimulation::startRageQuit(Golfer& g)
    {
        if (g.state == GolferState::rageQuitting || g.state == GolferState::leaving || g.finished)
            return;
        if (g.restingBenchId != 0)
        {
            for (auto& b : _env.benches)
                if (b.id == g.restingBenchId && b.occupants > 0)
                    b.occupants--;
            g.restingBenchId = 0;
        }
        g.rageQuit = true;
        g.ballInPlay = false;
        if (auto* range = rangeOf(g))
            range->leave(g.id);
        g.rangeId = 0;
        if (g.cartId != 0)
            abandonCart(g);
        g.destination = _env.exit;
        // GLF-041: an insult for everyone within three tiles, who each lose a little happiness.
        SimEvent e;
        e.type = SimEvent::Type::rageQuit;
        e.golferId = g.id;
        e.value = static_cast<int32_t>(g.holeIndex) + 1;
        e.insult = _env.rng != nullptr ? static_cast<int32_t>(_env.rng->next() % kInsultCount) : 0;
        _events.push_back(e);
        for (auto& other : _golfers)
        {
            if (other.id == g.id || other.finished)
                continue;
            if (horizontal(other.position, g.position) <= Fixed::fromInt(kRageRadiusUnits))
                other.needs.happiness = std::max(0, other.needs.happiness - kInsultHappinessLoss * 16);
        }
        setState(g, GolferState::rageQuitting);
    }

    void GolfSimulation::leave(Golfer& g)
    {
        if (g.restingBenchId != 0)
        {
            for (auto& b : _env.benches)
                if (b.id == g.restingBenchId && b.occupants > 0)
                    b.occupants--;
            g.restingBenchId = 0;
        }
        g.ballInPlay = false;
        if (auto* range = rangeOf(g))
            range->leave(g.id);
        g.rangeId = 0;
        if (g.cartId != 0)
            abandonCart(g);
        g.destination = g.hasExit ? g.exitPosition : _env.exit;
        setState(g, GolferState::leaving);
    }

    void GolfSimulation::judgeBall(Golfer& g)
    {
        const auto outcome = judgeShot(g.ball, *_env.world);
        emit(SimEvent::Type::ballSettled, g);
        if (outcome.kind == ShotOutcome::Kind::holed)
        {
            g.score.apply(outcome);
            setState(g, GolferState::holeFinished);
            return;
        }
        if (outcome.kind == ShotOutcome::Kind::penalty)
        {
            const bool pickedUp = g.score.apply(outcome);
            if (outcome.penalty == Penalty::water)
            {
                applyHappiness(g.needs, HappinessEvent::ballInHazard);
                think(g, ThoughtType::lostBall);
            }
            else
            {
                applyHappiness(g.needs, HappinessEvent::lostBall);
                g.rageCauses |= kRageLostBall;
                think(g, ThoughtType::lostBall);
                emit(SimEvent::Type::lostBall, g);
            }
            placeBall(g.ball, *_env.world, outcome.dropPosition.x, outcome.dropPosition.y, BallState::AtRest);
            g.searchedBall = true;
            if (pickedUp)
            {
                setState(g, GolferState::holeFinished);
                return;
            }
            g.destination = g.ball.position;
            setState(g, GolferState::walkingToBall);
            return;
        }

        // A good shot lands close to where it was aimed.
        const Fixed meant = g.shotTargetDistance;
        if (meant > Fixed::fromInt(32))
        {
            const Fixed travelled = horizontal(g.ball.position, g.ball.shotOrigin);
            if (abs(travelled - meant) * 100 <= meant * kGoodShotPercent && g.score.shots > 0)
            {
                applyHappiness(g.needs, HappinessEvent::goodShot);
                if (g.score.shots == 1 && meant > Fixed::fromInt(160 * 6))
                    think(g, ThoughtType::greatDrive);
            }
        }
        g.searchedBall = false;
        g.destination = g.ball.position;
        setState(g, GolferState::walkingToBall);
    }

    void GolfSimulation::finishHole(Golfer& g)
    {
        const auto* course = courseOf(g);
        const auto* hole = holeOf(g);
        const int32_t strokes = g.score.strokes();
        const int32_t par = g.score.par;
        if (g.holeIndex < g.scorecard.size())
        {
            g.scorecard[g.holeIndex] = static_cast<int8_t>(std::clamp(g.score.toPar(), -20, 20));
            g.parCard[g.holeIndex] = static_cast<uint8_t>(par);
        }
        g.strokesTotal += strokes;
        g.ballInPlay = false;

        // GLF-013: the mood after the hole.
        HappinessEvent mood = HappinessEvent::parOrBetter;
        if (g.score.pickedUp && !g.score.holed)
            mood = HappinessEvent::worseThanDoubleBogey;
        else
            switch (scoreName(strokes, par))
            {
                case ScoreName::holeInOne:
                    mood = HappinessEvent::holeInOne;
                    break;
                case ScoreName::albatross:
                case ScoreName::eagle:
                    mood = HappinessEvent::eagle;
                    break;
                case ScoreName::birdie:
                    mood = HappinessEvent::birdie;
                    break;
                case ScoreName::par:
                    mood = HappinessEvent::parOrBetter;
                    break;
                case ScoreName::bogey:
                    mood = HappinessEvent::bogey;
                    break;
                case ScoreName::doubleBogey:
                    mood = HappinessEvent::doubleBogey;
                    break;
                default:
                    mood = HappinessEvent::worseThanDoubleBogey;
                    break;
            }
        applyHappiness(g.needs, mood);

        if (course != nullptr && hole != nullptr)
        {
            // SCN-003: nice scenery near the hole contributes to golfer happiness.
            if (hole->sceneryScore > 0)
            {
                applyHappiness(g.needs, HappinessEvent::sceneryNearHole);
                think(g, ThoughtType::enjoyingTheRound);
            }
            // HOL-041: a hole too hard for the golfer costs happiness and is remembered as a cause of anger.
            const auto& difficulty = difficultyOf(g.courseId, hole->id);
            if (isTooHard(difficulty, g.currentSkills()))
            {
                applyHappiness(g.needs, HappinessEvent::holeTooHard);
                g.rageCauses |= kRageTooHard;
                think(g, ThoughtType::holeTooHard);
                emit(SimEvent::Type::tooHard, g);
            }
            if (g.waitedMinutes >= static_cast<uint32_t>(kLongWaitMinutes))
            {
                g.rageCauses |= kRageLongWait;
                think(g, ThoughtType::waitedTooLong);
            }
            // HOL-021, HOL-023: the hole learns how long groups take and wait.
            auto* mutableHole = _env.courses->findCourse(g.courseId)->findHole(hole->id);
            const int32_t played = static_cast<int32_t>(minuteNow() - g.holeStartMinute);
            recordPlay(mutableHole->schedule, played, static_cast<int32_t>(g.waitedMinutes));
            mutableHole->stats.totalStrokes += strokes;
            mutableHole->stats.timesPlayed++;
        }
        // GLF-021, MPT-003: a mini putt hole trains Accuracy only; a regular hole trains everything.
        gainSkills(g.skills, course != nullptr && course->kind == CourseKind::miniPutt ? Activity::miniPuttHole : Activity::courseHole, 1);
        emit(SimEvent::Type::holeFinished, g, g.score.toPar());

        if (shouldRageQuit(g.needs.happinessByte(), g.rageCauses))
        {
            startRageQuit(g);
            return;
        }
        goToNextStep(g);
    }

    void GolfSimulation::goToNextStep(Golfer& g)
    {
        const auto* course = courseOf(g);
        // The next hole that can be played.
        uint32_t next = g.holeIndex + 1;
        while (course != nullptr && next < course->holes.size() && !course->holes[next].isComplete())
            next++;
        if (course == nullptr || next >= course->holes.size())
        {
            if (course != nullptr)
            {
                auto* mutableCourse = _env.courses->findCourse(g.courseId);
                if (mutableCourse != nullptr)
                {
                    mutableCourse->stats.roundsPlayed++;
                    mutableCourse->stats.totalSatisfaction += g.needs.happinessByte();
                    mutableCourse->stats.satisfactionSamples++;
                }
            }
            // CRT-005: the cart goes back before the golfer goes home.
            if (g.cartId != 0)
                returnCartAndLeave(g);
            else
                leave(g);
            return;
        }
        g.holeIndex = next;
        bool critical = false;
        const auto need = mostUrgentNeed(g.needs, &critical);
        if (need != NeedKind::none)
        {
            g.facilityResumeBall = false;
            if (seekFacility(g, need))
                return;
        }
        startHole(g);
    }

    void GolfSimulation::onMinute(Golfer& g)
    {
        drainMinutes(g.needs, 1, g.skills.toSkills().physique, isWalkingState(g.state));
        applyNeedPenalties(g.needs, 1);
        if (g.state == GolferState::waitingForTeeTime)
        {
            g.waitedMinutes++;
            if (g.waitedMinutes > static_cast<uint32_t>(kFreeWaitMinutes))
                applyHappiness(g.needs, HappinessEvent::waitedAMinute);
        }
        // GLF-040: anger builds up until the golfer storms off.
        if (g.state != GolferState::leaving && g.state != GolferState::rageQuitting
            && shouldRageQuit(g.needs.happinessByte(), g.rageCauses))
            startRageQuit(g);
    }

    void GolfSimulation::updateGolfer(Golfer& g)
    {
        // Nothing happens to a golfer who has not got out of the car yet.
        if (g.state == GolferState::inCar)
            return;
        g.stateTicks++;
        if (_now % static_cast<uint32_t>(kTicksPerGameMinute) == 0)
            onMinute(g);

        switch (g.state)
        {
            case GolferState::arriving:
                g.destination = _env.clubEntrance;
                setState(g, GolferState::walkingToCheckIn);
                break;
            case GolferState::walkingToCheckIn:
                if (walk(g, _env.clubEntrance))
                    setState(g, GolferState::checkingIn);
                break;
            case GolferState::checkingIn:
            {
                if (g.stateTicks < static_cast<uint32_t>(kCheckInTicks))
                    break;
                const auto* course = courseOf(g);
                if (course == nullptr || course->playableHoles() == 0 || !course->open)
                {
                    leave(g);
                    break;
                }
                // GLF-032: the green fee is paid at check in; VIPs play free.
                const int64_t fee = g.vip ? 0 : course->greenFee;
                if (g.money < fee)
                {
                    applyHappiness(g.needs, HappinessEvent::highPrices);
                    think(g, ThoughtType::pricesTooHigh);
                    leave(g);
                    break;
                }
                g.money -= fee;
                _income += fee;
                auto* mutableCourse = _env.courses->findCourse(g.courseId);
                if (mutableCourse != nullptr)
                {
                    mutableCourse->stats.revenue += fee;
                }
                emit(SimEvent::Type::paidFee, g, static_cast<int32_t>(fee));
                // HOL-011: always from hole 1, the first hole that can be played.
                g.holeIndex = 0;
                while (g.holeIndex < course->holes.size() && !course->holes[g.holeIndex].isComplete())
                    g.holeIndex++;
                // HOL-022: the tee time is booked at check in for the whole group.
                const auto key = groupKey(g.groupId, g.holeIndex);
                if (_groupTeeTime.find(key) == _groupTeeTime.end())
                {
                    const auto booking = teeSheet(g.courseId).bookFirstTee(minuteNow(), slotMinutes(course->holes[g.holeIndex]));
                    _groupTeeTime[key] = booking.minute;
                }
                // DRV-005: some warm up at the driving range first, when there is one to use.
                if (_env.courses != nullptr && !_env.courses->ranges().empty() && wantsRange(g))
                {
                    syncRanges();
                    for (const auto& range : _env.courses->ranges())
                    {
                        if (range.kiosk.has_value() && !range.hittingTiles.empty() && g.money >= range.bucketPrice + 1)
                        {
                            visitRange(g, range);
                            break;
                        }
                    }
                    if (g.state == GolferState::walkingToRange)
                        break;
                }
                beginRound(g, *course);
                break;
            }
            case GolferState::walkingToRange:
            {
                auto* range = rangeOf(g);
                if (range == nullptr)
                {
                    endRangeVisit(g);
                    break;
                }
                if (!walk(g, g.destination))
                    break;
                // At the kiosk: buy a bucket, and take a tile if there is one (it waits a few minutes if not).
                const auto result = range->enter(g);
                if (result == RangeSimulation::Enter::ok)
                    break; // enter() made the state atDrivingRange
                if (result == RangeSimulation::Enter::noFreeTile
                    && g.stateTicks < static_cast<uint32_t>(3 * kTicksPerGameMinute))
                    break;
                endRangeVisit(g);
                break;
            }
            case GolferState::atDrivingRange:
            {
                auto* range = rangeOf(g);
                if (range == nullptr || range->tick(g))
                    endRangeVisit(g);
                break;
            }
            case GolferState::goingToCart:
            {
                auto* cart = cartOf(g);
                if (cart == nullptr)
                {
                    g.cartId = 0;
                    startHole(g);
                    break;
                }
                // The cart may have driven off with the other rider already: go where it is.
                if (!walk(g, cart->position))
                    break;
                board(g);
                startHole(g);
                break;
            }
            case GolferState::returningCart:
            {
                auto* cart = cartOf(g);
                if (cart == nullptr)
                {
                    g.cartId = 0;
                    g.onCart = false;
                    leave(g);
                    break;
                }
                if (!walk(g, g.destination))
                    break;
                g.onCart = false;
                if (cart->state == CartState::returning)
                    _env.carts->park(cart->id);
                g.cartId = 0;
                leave(g);
                break;
            }
            case GolferState::goingToTee:
            {
                if (g.cartId != 0 && g.stateTicks == 1)
                    board(g);
                if (!walk(g, g.destination))
                    break;
                if (g.onCart)
                    dismount(g);
                const auto* course = courseOf(g);
                const auto key = groupKey(g.groupId, g.holeIndex);
                auto it = _groupTeeTime.find(key);
                if (it == _groupTeeTime.end())
                {
                    // The first of the group to reach this tee books the slot for everyone (HOL-022).
                    const auto booking = teeSheet(g.courseId).bookTeeOff(g.holeIndex, minuteNow(), slotMinutes(course->holes[g.holeIndex]));
                    it = _groupTeeTime.emplace(key, booking.minute).first;
                }
                g.teeTimeMinute = it->second;
                g.waitedMinutes = 0;
                setState(g, GolferState::waitingForTeeTime);
                break;
            }
            case GolferState::waitingForTeeTime:
                if (minuteNow() >= g.teeTimeMinute && !groupBusy(g))
                {
                    teeOff(g);
                    break;
                }
                // GLF-031: golfers in the same group waiting at the tee chat with each other.
                if (g.stateTicks > 10 && g.stateTicks % 20 == 0)
                {
                    for (auto& other : _golfers)
                    {
                        if (other.id != g.id && other.groupId == g.groupId && other.state == GolferState::waitingForTeeTime
                            && other.holeIndex == g.holeIndex)
                        {
                            setState(g, GolferState::talking);
                            setState(other, GolferState::talking);
                            applyHappiness(g.needs, HappinessEvent::shortWait);
                            applyHappiness(other.needs, HappinessEvent::shortWait);
                            emit(SimEvent::Type::talkedWithGolfer, g);
                            emit(SimEvent::Type::talkedWithGolfer, other);
                            break;
                        }
                    }
                }
                break;
            case GolferState::talking:
                if (minuteNow() >= g.teeTimeMinute || g.stateTicks >= static_cast<uint32_t>(kTalkMinutes * kTicksPerGameMinute))
                {
                    setState(g, GolferState::waitingForTeeTime);
                }
                break;
            case GolferState::preparingShot:
            {
                if (g.stateTicks == 1)
                {
                    const auto* hole = holeOf(g);
                    // A need that cannot wait sends the golfer away in the middle of the hole (GLF-012).
                    bool critical = false;
                    const auto need = mostUrgentNeed(g.needs, &critical);
                    if (critical && g.stateTicks == 1)
                    {
                        g.facilityResumeBall = true;
                        if (seekFacility(g, need))
                            break;
                    }
                    PlayContext c;
                    c.world = _env.world;
                    c.skills = g.currentSkills();
                    c.energy = g.needs.energyByte();
                    c.equipmentLevel = g.equipmentLevel;
                    c.riskTolerance = g.riskTolerance;
                    c.ball = g.ball.position;
                    c.line = hole->playingLine(teeFor(*hole, g.tee).has_value() ? g.tee : *hole->firstTee());
                    if (c.line.empty())
                        c.line = hole->playingLine(*hole->firstTee());
                    const auto ground = _env.world->surfaceAt(Fixed::fromInt(hole->cup->x), Fixed::fromInt(hole->cup->y));
                    c.cup = { Fixed::fromInt(hole->cup->x), Fixed::fromInt(hole->cup->y), ground.height };
                    c.teeShot = g.score.shots == 0;
                    c.miniPutt = courseOf(g)->kind == CourseKind::miniPutt;
                    g.decision = decideShot(c);
                    g.shotTargetDistance = g.decision.expectedDistance;
                }
                if (g.stateTicks >= static_cast<uint32_t>(kPrepareShotTicks) && !groupBusy(g))
                    setState(g, g.decision.putt ? GolferState::putting : GolferState::swinging);
                break;
            }
            case GolferState::swinging:
            case GolferState::putting:
            {
                if (g.stateTicks < static_cast<uint32_t>(kSwingTicks))
                    break;
                ShotRequest r;
                r.club = g.decision.club;
                r.power = g.decision.power;
                r.aimX = g.decision.aimX;
                r.aimY = g.decision.aimY;
                r.energy = g.needs.energyByte();
                r.skills = g.currentSkills();
                r.equipmentLevel = g.equipmentLevel;
                const auto here = _env.world->surfaceAt(g.ball.position.x, g.ball.position.y);
                r.lie = lieOf(here);
                r.surfaceRoughness = here.roughness;
                const auto shot = computeShot(r, *_env.rng);
                applyShot(g.ball, shot);
                g.score.addShot();
                drainSwing(g.needs, r.skills.physique);
                emit(SimEvent::Type::shotHit, g);
                setState(g, GolferState::watchingBall);
                break;
            }
            case GolferState::watchingBall:
            {
                stepBall(g.ball, *_env.world, *_env.rng);
                if (isSettled(g.ball.state))
                    judgeBall(g);
                break;
            }
            case GolferState::walkingToBall:
            {
                // CRT-004: the cart takes the golfer to the path nearest the ball (or near the ball when carts may leave the
                // paths); the last stretch is on foot.
                if (g.cartId != 0 && g.stateTicks == 1)
                {
                    board(g);
                    const auto* course = courseOf(g);
                    const std::vector<FixedVec3> points = _env.pathPoints ? _env.pathPoints() : std::vector<FixedVec3>{};
                    g.cartStop = CartFleet::stopPoint(
                        g.ball.position, points, course != nullptr && course->cartsAllowedOffPath);
                }
                if (g.onCart)
                {
                    if (!walk(g, g.cartStop))
                        break;
                    dismount(g);
                    break;
                }
                if (!walk(g, g.ball.position))
                    break;
                if (!g.searchedBall)
                {
                    const auto here = _env.world->surfaceAt(g.ball.position.x, g.ball.position.y);
                    if (lostBallChancePercent(here) > 0)
                    {
                        setState(g, GolferState::searchingBall);
                        break;
                    }
                }
                setState(g, GolferState::preparingShot);
                break;
            }
            case GolferState::searchingBall:
            {
                if (g.stateTicks < static_cast<uint32_t>(kSearchTicks))
                    break;
                const auto here = _env.world->surfaceAt(g.ball.position.x, g.ball.position.y);
                g.searchedBall = true;
                if (searchFindsBall(here, *_env.rng))
                {
                    setState(g, GolferState::preparingShot);
                    break;
                }
                const auto outcome = lostBallOutcome(g.ball);
                const bool pickedUp = g.score.apply(outcome);
                applyHappiness(g.needs, HappinessEvent::lostBall);
                g.rageCauses |= kRageLostBall;
                think(g, ThoughtType::lostBall);
                emit(SimEvent::Type::lostBall, g);
                placeBall(g.ball, *_env.world, outcome.dropPosition.x, outcome.dropPosition.y, BallState::AtRest);
                if (pickedUp)
                {
                    setState(g, GolferState::holeFinished);
                    break;
                }
                g.destination = g.ball.position;
                setState(g, GolferState::walkingToBall);
                break;
            }
            case GolferState::holeFinished:
                finishHole(g);
                break;
            case GolferState::usingFacility:
            {
                const Service* service = nullptr;
                for (const auto& s : _env.services)
                    if (s.id == g.facilityService)
                        service = &s;
                if (service == nullptr)
                {
                    // The building was removed on the way.
                    if (g.facilityResumeBall)
                        setState(g, GolferState::walkingToBall);
                    else
                        startHole(g);
                    break;
                }
                if (g.facilityTicksLeft == kWalkingToFacility)
                {
                    if (walk(g, service->position))
                        g.facilityTicksLeft = static_cast<uint32_t>(service->minutes * kTicksPerGameMinute);
                    break;
                }
                if (g.facilityTicksLeft > 0)
                {
                    g.facilityTicksLeft--;
                    break;
                }
                // GLF-012, GLF-050: the need is met; the building takes its price.
                if (service->food)
                    g.needs.hunger = 255 * 16;
                if (service->drink)
                    g.needs.thirst = 255 * 16;
                if (service->toilet)
                    g.needs.bladder = 0;
                if (service->rest)
                    g.needs.energy = std::min(255 * 16, g.needs.energy + kRestEnergyPerMinute * 16 * service->minutes);
                if (service->proShop && g.equipmentLevel < 3)
                {
                    constexpr int64_t kEquipmentUpgradePrice = 2000;
                    if (g.money >= kEquipmentUpgradePrice + service->price)
                    {
                        g.equipmentLevel++;
                        g.money -= kEquipmentUpgradePrice;
                        _income += kEquipmentUpgradePrice;
                    }
                }
                g.money -= std::min<int64_t>(g.money, service->price);
                _income += service->price;
                emit(SimEvent::Type::usedFacility, g, static_cast<int32_t>(service->price));

                // PTH-022: food and drink generate litter. If a bin is within reach, use it; otherwise litter.
                if (service->food || service->drink)
                {
                    LitterBin* nearestBin = nullptr;
                    Fixed nearestBinDist = Fixed::fromInt(1000000);
                    constexpr Fixed kMaxBinDistance = Fixed::fromInt(10 * 32); // 10 tiles
                    for (auto& bin : _env.bins)
                    {
                        const Fixed d = horizontal(bin.position, g.position);
                        if (d <= kMaxBinDistance && d < nearestBinDist)
                        {
                            nearestBin = &bin;
                            nearestBinDist = d;
                        }
                    }
                    if (nearestBin != nullptr && nearestBin->trashCount < LitterBin::kCapacity)
                    {
                        nearestBin->trashCount++;
                        applyHappiness(g.needs, HappinessEvent::cleanPaths);
                    }
                    else
                    {
                        emit(SimEvent::Type::littered, g);
                        applyHappiness(g.needs, HappinessEvent::litter);
                        for (auto& other : _golfers)
                        {
                            if (other.id != g.id && !other.finished
                                && horizontal(other.position, g.position) <= Fixed::fromInt(3 * 32))
                            {
                                applyHappiness(other.needs, HappinessEvent::litter);
                            }
                        }
                    }
                }

                if (g.facilityResumeBall)
                {
                    g.facilityResumeBall = false;
                    g.destination = g.ball.position;
                    setState(g, GolferState::walkingToBall);
                }
                else
                {
                    startHole(g);
                }
                break;
            }
            case GolferState::resting:
            {
                Bench* bench = nullptr;
                for (auto& b : _env.benches)
                {
                    if (b.id == g.restingBenchId)
                    {
                        bench = &b;
                        break;
                    }
                }
                if (bench == nullptr)
                {
                    if (g.facilityResumeBall)
                    {
                        g.facilityResumeBall = false;
                        g.destination = g.ball.position;
                        setState(g, GolferState::walkingToBall);
                    }
                    else
                    {
                        startHole(g);
                    }
                    break;
                }
                if (g.facilityTicksLeft == kWalkingToFacility)
                {
                    if (walk(g, bench->position))
                        g.facilityTicksLeft = static_cast<uint32_t>(2 * kTicksPerGameMinute);
                    break;
                }
                if (g.facilityTicksLeft > 0)
                {
                    g.facilityTicksLeft--;
                    break;
                }
                // Rest is complete: restore 2 minutes worth of energy
                g.needs.energy = std::min(255 * 16, g.needs.energy + kRestEnergyPerMinute * 16 * 2);
                if (bench->occupants > 0)
                    bench->occupants--;
                g.restingBenchId = 0;
                emit(SimEvent::Type::restedOnBench, g);
                if (g.facilityResumeBall)
                {
                    g.facilityResumeBall = false;
                    g.destination = g.ball.position;
                    setState(g, GolferState::walkingToBall);
                }
                else
                {
                    startHole(g);
                }
                break;
            }
            case GolferState::leaving:
            case GolferState::rageQuitting:
                if (walk(g, g.hasExit ? g.exitPosition : _env.exit))
                {
                    g.finished = true;
                    emit(SimEvent::Type::roundFinished, g, g.toParSoFar());
                }
                break;
            default:
                break;
        }
    }

    void GolfSimulation::tick(uint32_t nowTicks)
    {
        _now = nowTicks;
        if (_now % static_cast<uint32_t>(kTicksPerGameMinute) == 0)
            syncRanges();
        for (size_t i = 0; i < _golfers.size(); i++)
        {
            if (!_golfers[i].finished)
                updateGolfer(_golfers[i]);
        }
        // What happened at the ranges: the buckets are income, the shots make a sound.
        for (auto& [id, range] : _ranges)
        {
            for (const auto& event : range->takeEvents())
            {
                const auto* golfer = find(event.golferId);
                if (golfer == nullptr)
                    continue;
                if (event.type == RangeEvent::Type::boughtBucket)
                {
                    _income += event.value;
                    emit(SimEvent::Type::paidRangeFee, *golfer, event.value);
                }
                else if (event.type == RangeEvent::Type::ballHit)
                {
                    emit(SimEvent::Type::shotHit, *golfer);
                }
            }
        }
        for (size_t i = 0; i < _golfers.size();)
        {
            if (_golfers[i].finished)
            {
                _left.push_back(std::move(_golfers[i]));
                _golfers.erase(_golfers.begin() + static_cast<std::ptrdiff_t>(i));
            }
            else
            {
                i++;
            }
        }
    }
} // namespace OpenRCT2::Golf
