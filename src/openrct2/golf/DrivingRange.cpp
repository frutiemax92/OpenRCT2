/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "DrivingRange.h"

#include "GameClock.h"
#include "Shot.h"

#include <algorithm>

namespace OpenRCT2::Golf
{
    namespace
    {
        constexpr int32_t kPrepareTicks = 30;
        constexpr int32_t kSwingTicks = 30;
        constexpr int32_t kMaxFlightTicks = 2000;
        constexpr int32_t kWalkSpeedHundredths = 24;

        Fixed distanceXY(const FixedVec3& a, const FixedVec3& b)
        {
            const FixedVec3 d{ a.x - b.x, a.y - b.y, Fixed{} };
            return length(d);
        }
    } // namespace

    bool DrivingRange::containsArea(const HolePoint& p) const
    {
        const int32_t tx = p.x >> 5, ty = p.y >> 5;
        return tx >= areaMin.x && tx <= areaMax.x && ty >= areaMin.y && ty <= areaMax.y;
    }

    int32_t signDistance(const HolePoint& kiosk, const HolePoint& sign, DistanceUnit unit)
    {
        const int32_t yards = distanceYards(kiosk, sign);
        if (unit == DistanceUnit::yards)
            return roundToFive(yards);
        // 1 yard is 0.9144 m: work in 1/10000 to stay exact.
        const int64_t metres = (static_cast<int64_t>(yards) * 9144 + 5000) / 10000;
        return roundToFive(static_cast<int32_t>(metres));
    }

    int32_t defaultShootingDirection(const DrivingRange& range)
    {
        if (range.hittingTiles.empty())
            return range.shootingDirectionDegrees;
        int64_t sx = 0, sy = 0;
        for (const auto& t : range.hittingTiles)
        {
            sx += t.x;
            sy += t.y;
        }
        const int64_t cx = sx / static_cast<int64_t>(range.hittingTiles.size());
        const int64_t cy = sy / static_cast<int64_t>(range.hittingTiles.size());
        const int64_t ax = (static_cast<int64_t>(range.areaMin.x + range.areaMax.x + 1) * 32) / 2;
        const int64_t ay = (static_cast<int64_t>(range.areaMin.y + range.areaMax.y + 1) * 32) / 2;
        const int64_t dx = ax - cx, dy = ay - cy;
        if (dx == 0 && dy == 0)
            return range.shootingDirectionDegrees;
        // atan2 in whole degrees through the fixed-point sin/cos: pick the degree whose direction is closest.
        int32_t best = 0;
        int64_t bestDot = INT64_MIN;
        for (int32_t degrees = 0; degrees < 360; degrees++)
        {
            const Fixed radians = Fixed::fromInt(degrees) * kFixedPi / 180;
            const int64_t d = static_cast<int64_t>(cos(radians).raw()) * dx + static_cast<int64_t>(sin(radians).raw()) * dy;
            if (d > bestDot)
            {
                bestDot = d;
                best = degrees;
            }
        }
        return best;
    }

    bool kioskMayBeHit(const DrivingRange& range)
    {
        if (!range.kiosk.has_value() || range.hittingTiles.empty())
            return false;
        const Fixed radians = Fixed::fromInt(range.shootingDirectionDegrees) * kFixedPi / 180;
        const Fixed dx = cos(radians), dy = sin(radians);
        // The kiosk is safe when it lies behind every hitting tile: against the direction of fire.
        for (const auto& tile : range.hittingTiles)
        {
            const Fixed along = Fixed::fromInt(range.kiosk->x - tile.x) * dx + Fixed::fromInt(range.kiosk->y - tile.y) * dy;
            if (along >= Fixed::fromInt(-1))
                return true; // level with or in front of this tile
        }
        return false;
    }

    RangeSimulation::RangeSimulation(const IBallWorld* world, IBallRandom* rng, DrivingRange range)
        : _world(world)
        , _rng(rng)
        , _range(std::move(range))
    {
        _tileTaken.assign(_range.hittingTiles.size(), false);
    }

    void RangeSimulation::updateRange(const DrivingRange& range)
    {
        // A tile is the same tile when it is at the same place; the sessions stay on theirs.
        std::vector<bool> taken(range.hittingTiles.size(), false);
        for (auto it = _sessions.begin(); it != _sessions.end();)
        {
            const auto& old = _range.hittingTiles[it->second.tile];
            size_t found = range.hittingTiles.size();
            for (size_t i = 0; i < range.hittingTiles.size(); i++)
                if (range.hittingTiles[i].x == old.x && range.hittingTiles[i].y == old.y)
                    found = i;
            if (found == range.hittingTiles.size())
            {
                it = _sessions.erase(it);
                continue;
            }
            it->second.tile = found;
            taken[found] = true;
            ++it;
        }
        _range = range;
        _tileTaken = std::move(taken);
    }

    std::vector<RangeSimulation::BallView> RangeSimulation::balls() const
    {
        std::vector<BallView> views;
        for (const auto& [golferId, session] : _sessions)
        {
            if (session.phase == Session::Phase::flying)
                views.push_back({ golferId, session.ball });
        }
        std::sort(views.begin(), views.end(), [](const BallView& a, const BallView& b) { return a.golferId < b.golferId; });
        return views;
    }

    int32_t RangeSimulation::freeTiles() const
    {
        return static_cast<int32_t>(std::count(_tileTaken.begin(), _tileTaken.end(), false));
    }

    bool RangeSimulation::isOnRange(uint32_t golferId) const
    {
        return _sessions.find(golferId) != _sessions.end();
    }

    std::vector<RangeEvent> RangeSimulation::takeEvents()
    {
        std::vector<RangeEvent> events;
        events.swap(_events);
        return events;
    }

    RangeSimulation::Enter RangeSimulation::enter(Golfer& g)
    {
        if (!_range.kiosk.has_value())
            return Enter::noKiosk;
        if (_range.hittingTiles.empty())
            return Enter::noTiles;
        if (g.money < _range.bucketPrice)
            return Enter::cannotPay;
        size_t tile = _tileTaken.size();
        for (size_t i = 0; i < _tileTaken.size(); i++)
            if (!_tileTaken[i])
            {
                tile = i;
                break;
            }
        if (tile == _tileTaken.size())
            return Enter::noFreeTile;

        g.money -= _range.bucketPrice;
        _income += _range.bucketPrice;
        _tileTaken[tile] = true;
        Session s;
        s.tile = tile;
        s.ballsLeft = _range.ballsPerBucket;
        _sessions[g.id] = s;
        g.state = GolferState::atDrivingRange;
        g.stateTicks = 0;
        g.destination = { Fixed::fromInt(_range.hittingTiles[tile].x), Fixed::fromInt(_range.hittingTiles[tile].y), Fixed{} };
        _events.push_back({ RangeEvent::Type::boughtBucket, g.id, static_cast<int32_t>(_range.bucketPrice) });
        return Enter::ok;
    }

    void RangeSimulation::leave(uint32_t golferId)
    {
        auto it = _sessions.find(golferId);
        if (it == _sessions.end())
            return;
        _tileTaken[it->second.tile] = false;
        _sessions.erase(it);
    }

    void RangeSimulation::startBall(Golfer& g, Session& s)
    {
        const auto& tile = _range.hittingTiles[s.tile];
        placeBall(s.ball, *_world, Fixed::fromInt(tile.x), Fixed::fromInt(tile.y), BallState::Teed);
        // A mix of clubs: mostly the driver and a long iron, now and then a wedge.
        const uint32_t roll = _rng->next() % 10;
        s.club = roll < 4 ? ClubId::driver : (roll < 6 ? ClubId::wood3 : (roll < 8 ? ClubId::iron7 : ClubId::pitchingWedge));
        s.phase = Session::Phase::preparing;
        s.phaseTicks = 0;
        s.landed = false;
        (void)g;
    }

    bool RangeSimulation::tick(Golfer& g)
    {
        auto it = _sessions.find(g.id);
        if (it == _sessions.end())
            return true;
        auto& s = it->second;
        const auto& tile = _range.hittingTiles[s.tile];
        const FixedVec3 tilePos{ Fixed::fromInt(tile.x), Fixed::fromInt(tile.y), Fixed{} };
        s.phaseTicks++;

        switch (s.phase)
        {
            case Session::Phase::walking:
            {
                const Fixed speed = Fixed::ratio(kWalkSpeedHundredths, 100);
                const FixedVec3 delta{ tilePos.x - g.position.x, tilePos.y - g.position.y, Fixed{} };
                const Fixed distance = length(delta);
                if (distance <= Fixed::fromInt(2) || distance <= speed)
                {
                    g.position = { tilePos.x, tilePos.y, _world->surfaceAt(tilePos.x, tilePos.y).height };
                    startBall(g, s);
                }
                else
                {
                    g.position.x += delta.x * speed / distance;
                    g.position.y += delta.y * speed / distance;
                }
                return false;
            }
            case Session::Phase::preparing:
                if (s.phaseTicks >= kPrepareTicks)
                {
                    s.phase = Session::Phase::swinging;
                    s.phaseTicks = 0;
                }
                return false;
            case Session::Phase::swinging:
            {
                if (s.phaseTicks < kSwingTicks)
                    return false;
                ShotRequest r;
                r.club = s.club;
                r.skills = g.currentSkills();
                r.energy = g.needs.energyByte();
                r.equipmentLevel = g.equipmentLevel;
                const Fixed radians = Fixed::fromInt(_range.shootingDirectionDegrees) * kFixedPi / 180;
                r.aimX = cos(radians);
                r.aimY = sin(radians);
                r.lie = lieOf(_world->surfaceAt(s.ball.position.x, s.ball.position.y));
                applyShot(s.ball, computeShot(r, *_rng));
                s.ball.passingObstacle = golferObstacleId(g.id);
                drainSwing(g.needs, r.skills.physique);
                s.phase = Session::Phase::flying;
                s.phaseTicks = 0;
                _events.push_back({ RangeEvent::Type::ballHit, g.id, static_cast<int32_t>(s.club) });
                return false;
            }
            case Session::Phase::flying:
            {
                const auto events = stepBall(s.ball, *_world, *_rng);
                if (events.hitGolfer)
                {
                    // The victim is told by the caller through the event stream of the ball; the shooter just carries on.
                }
                if (events.impacts > 0 && !s.landed)
                {
                    s.landed = true;
                    const HolePoint landing{ static_cast<int32_t>(s.ball.position.x.floor()), static_cast<int32_t>(s.ball.position.y.floor()) };
                    const int32_t yards = distanceYards(tile, landing);
                    _events.push_back({ RangeEvent::Type::ballLanded, g.id, yards });
                    if (!_range.containsArea(landing))
                        _events.push_back({ RangeEvent::Type::ballLeftArea, g.id, yards });
                    // DRV-005: near a distance sign the golfer learns their distance.
                    for (const auto& sign : _range.signs)
                        if (distanceYards(sign.position, landing) <= kSignRadiusYards)
                        {
                            applyHappiness(g.needs, HappinessEvent::rangeDistanceSign);
                            _events.push_back({ RangeEvent::Type::nearSign, g.id, static_cast<int32_t>(sign.id) });
                            break;
                        }
                    // DRV-006: every ball improves Long play.
                    SkillPoints skills = g.skills;
                    gainSkills(skills, Activity::drivingRangeBall, 1);
                    g.skills = skills;
                }
                if (isSettled(s.ball.state) || s.phaseTicks > kMaxFlightTicks)
                {
                    if (s.landed)
                    {
                        _landedBalls.push_back(s.ball.position);
                    }
                    s.ballsLeft--;
                    if (s.ballsLeft <= 0)
                    {
                        _events.push_back({ RangeEvent::Type::finished, g.id, 0 });
                        leave(g.id);
                        return true;
                    }
                    startBall(g, s);
                }
                return false;
            }
        }
        return false;
    }

    void RangeSimulation::deployPicker()
    {
        _picker.active = true;
        if (_range.kiosk.has_value())
        {
            _picker.position = { Fixed::fromInt(_range.kiosk->x), Fixed::fromInt(_range.kiosk->y), Fixed{} };
        }
        else if (!_range.hittingTiles.empty())
        {
            _picker.position = { Fixed::fromInt(_range.hittingTiles[0].x), Fixed::fromInt(_range.hittingTiles[0].y), Fixed{} };
        }
    }

    void RangeSimulation::recallPicker()
    {
        _picker.active = false;
        if (_range.kiosk.has_value())
        {
            _picker.target = { Fixed::fromInt(_range.kiosk->x), Fixed::fromInt(_range.kiosk->y), Fixed{} };
        }
    }

    void RangeSimulation::tickPicker()
    {
        if (!_picker.active)
            return;

        const Fixed speed = Fixed::ratio(150, 100);

        if (_landedBalls.empty())
        {
            if (_range.kiosk.has_value())
            {
                _picker.target = { Fixed::fromInt(_range.kiosk->x), Fixed::fromInt(_range.kiosk->y), Fixed{} };
                const FixedVec3 delta{ _picker.target.x - _picker.position.x, _picker.target.y - _picker.position.y, Fixed{} };
                const Fixed dist = length(delta);
                if (dist > speed)
                {
                    _picker.position.x += delta.x * speed / dist;
                    _picker.position.y += delta.y * speed / dist;
                }
                else
                {
                    _picker.position = _picker.target;
                }
            }
            return;
        }

        size_t bestIdx = 0;
        Fixed bestDist{};
        for (size_t i = 0; i < _landedBalls.size(); i++)
        {
            const Fixed d = distanceXY(_picker.position, _landedBalls[i]);
            if (i == 0 || d < bestDist)
            {
                bestDist = d;
                bestIdx = i;
            }
        }

        _picker.target = _landedBalls[bestIdx];
        const FixedVec3 delta{ _picker.target.x - _picker.position.x, _picker.target.y - _picker.position.y, Fixed{} };
        const Fixed dist = length(delta);

        if (dist <= Fixed::fromInt(4) || dist <= speed)
        {
            _landedBalls.erase(_landedBalls.begin() + bestIdx);
            _picker.collectedBalls++;
            _picker.position = _picker.target;
        }
        else
        {
            _picker.position.x += delta.x * speed / dist;
            _picker.position.y += delta.y * speed / dist;
        }
    }
} // namespace OpenRCT2::Golf
