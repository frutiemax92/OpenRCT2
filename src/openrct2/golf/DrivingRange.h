/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "BallPhysics.h"
#include "Golfer.h"
#include "Hole.h"

#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

namespace OpenRCT2::Golf
{
    // The driving range (section 17).

    struct DistanceSign
    {
        uint32_t id = 0;
        HolePoint position;
    };

    struct DrivingRange
    {
        uint32_t id = 0;
        TileCoordsXY areaMin;                  // DRV-001: the area drawn with the zone tool, inclusive
        TileCoordsXY areaMax;
        std::vector<HolePoint> hittingTiles;   // where golfers stand; world positions of the tile centres
        std::optional<HolePoint> kiosk;        // the one driving range kiosk building
        std::vector<DistanceSign> signs;
        int64_t bucketPrice = 300;
        int32_t ballsPerBucket = 30;           // DRV-005
        int32_t shootingDirectionDegrees = 0;  // 0 = +x, 90 = +y: where the golfers aim

        bool containsArea(const HolePoint& p) const;
    };

    // DRV-004: the distance a sign shows: from the kiosk to the sign, an integer rounded to 5, in yards or metres.
    enum class DistanceUnit : uint8_t
    {
        yards,
        metres,
    };
    int32_t signDistance(const HolePoint& kiosk, const HolePoint& sign, DistanceUnit unit = DistanceUnit::yards);
    // Roundings to the nearest multiple of five.
    constexpr int32_t roundToFive(int32_t value)
    {
        return ((value + 2) / 5) * 5;
    }

    // DRV-002: the kiosk may be hit by balls when it is not behind the hitting line seen from the shooting direction.
    // False (no warning) when there are no hitting tiles or no kiosk.
    bool kioskMayBeHit(const DrivingRange& range);

    // The direction golfers shoot in, from the hitting tiles towards the middle of the area, in degrees 0..359.
    int32_t defaultShootingDirection(const DrivingRange& range);

    // DRV-005: a ball counts for a sign when it lands this close to it, in yards.
    constexpr int32_t kSignRadiusYards = 10;

    struct RangeEvent
    {
        enum class Type : uint8_t
        {
            boughtBucket,
            ballHit,
            ballLanded,    // value = distance from the hitting tile in yards
            nearSign,      // the golfer knows their distance now
            ballLeftArea,  // landed outside the area (DRV-006)
            finished,
        };
        Type type = Type::ballHit;
        uint32_t golferId = 0;
        int32_t value = 0;
    };

    class RangeSimulation
    {
    public:
        RangeSimulation(const IBallWorld* world, IBallRandom* rng, DrivingRange range);

        DrivingRange& range()
        {
            return _range;
        }

        // A golfer comes to the range: buys a bucket and gets a hitting tile if there is one free. Returns false when the
        // golfer cannot pay or every tile is taken (the golfer waits and asks again).
        enum class Enter : uint8_t
        {
            ok,
            cannotPay,
            noFreeTile,
            noKiosk,
            noTiles,
        };
        Enter enter(Golfer& golfer);
        bool isOnRange(uint32_t golferId) const;
        // Advances the golfer's session by one tick; returns true when the bucket is empty and the golfer is done.
        bool tick(Golfer& golfer);
        // The golfer leaves early (rage quit, closing time): the tile is free again.
        void leave(uint32_t golferId);

        int32_t freeTiles() const;
        std::vector<RangeEvent> takeEvents();
        int64_t income() const
        {
            return _income;
        }

    private:
        struct Session
        {
            size_t tile = 0;
            int32_t ballsLeft = 0;
            int32_t phaseTicks = 0;
            enum class Phase : uint8_t
            {
                walking,
                preparing,
                swinging,
                flying,
            } phase = Phase::walking;
            Ball ball;
            bool landed = false;
            ClubId club = ClubId::driver;
        };

        void startBall(Golfer& g, Session& s);

        const IBallWorld* _world;
        IBallRandom* _rng;
        DrivingRange _range;
        std::unordered_map<uint32_t, Session> _sessions;
        std::vector<bool> _tileTaken;
        std::vector<RangeEvent> _events;
        int64_t _income = 0;
    };
} // namespace OpenRCT2::Golf
