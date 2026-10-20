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
#include "Shot.h"

#include <cstdint>
#include <optional>
#include <vector>

namespace OpenRCT2::Golf
{
    // Golf carts (section 14): the fleet, the parking spaces, who takes a cart, where carts go and how they come back.

    constexpr int32_t kCartSeats = 2;                // CRT-001
    constexpr int32_t kDefaultCartPrice = 4000;      // cost to buy one cart (ECO-011)
    constexpr int32_t kDefaultCartFee = 600;         // CRT-003
    constexpr int32_t kCartTopSpeedHundredths = 220; // world units per tick: a bit faster than walking
    constexpr int32_t kCartStopDistanceUnits = 10 * 32 / 5; // CRT-004: stop within ten yards of the ball when off the path
    // Wheel contact points for the terrain tilt (R3D-006, CRT-006), in world units from the centre.
    constexpr int32_t kCartHalfLength = 10;
    constexpr int32_t kCartHalfWidth = 6;

    // CRT-002: a cart parking space, a tile or a spot at the side of a path.
    struct CartSpace
    {
        uint32_t id = 0;
        FixedVec3 position;
        uint32_t cartId = 0;  // the cart standing here
        uint32_t reservedBy = 0; // a cart on its way back
    };

    enum class CartState : uint8_t
    {
        parked,
        waitingForGolfers,   // taken out of its space, golfers are walking to it
        driving,             // carrying golfers on the course
        returning,           // heading back to a parking space
    };

    struct Cart
    {
        uint32_t id = 0;
        CartState state = CartState::parked;
        FixedVec3 position;
        FixedVec3 destination;
        std::vector<uint32_t> riders;   // CRT-004: one or two golfers of the same group
        uint32_t spaceId = 0;           // the space it is parked in or heading for
        uint32_t groupId = 0;
        Fixed speed{};
    };

    // CRT-003: whether a golfer wants a cart, from their energy, their physique, the length of the course and their
    // money. A deterministic score so the same golfer always decides alike; `jitter` (0..99) makes some differ.
    constexpr int32_t kWantCartThreshold = 40;
    int32_t cartDesire(uint8_t energy, int32_t physique, int32_t courseYards);
    bool wantsCart(uint8_t energy, int32_t physique, int32_t courseYards, int64_t money, int64_t cartFee, int32_t jitter = 0);

    struct CartRental
    {
        std::vector<uint32_t> cartIds;      // carts used by the group
        std::vector<uint32_t> walkers;      // golfers who found no cart and walk (a happiness penalty, CRT-003)
        int64_t feeEach = 0;
    };

    class CartFleet
    {
    public:
        int64_t cartFee = kDefaultCartFee;
        int64_t cartPrice = kDefaultCartPrice;

        CartSpace& addSpace(const FixedVec3& position);
        // CRT-002: the player buys carts; each is placed at a free parking space. Returns how many were placed (it can be
        // fewer than asked when the spaces run out) and the price paid.
        struct Purchase
        {
            int32_t bought = 0;
            int64_t cost = 0;
        };
        Purchase buyCarts(int32_t count);

        int32_t cartCount() const
        {
            return static_cast<int32_t>(_carts.size());
        }
        int32_t freeCarts() const;
        int32_t spaceCount() const
        {
            return static_cast<int32_t>(_spaces.size());
        }
        const std::vector<Cart>& carts() const
        {
            return _carts;
        }
        const std::vector<CartSpace>& spaces() const
        {
            return _spaces;
        }
        Cart* find(uint32_t cartId);

        // CRT-003/004: a group takes carts for the golfers who want one, two to a cart, nearest cart first from `from`. The
        // fee is charged to each golfer by the caller (`money` is checked here, per golfer id). Golfers who cannot get a cart
        // walk.
        struct Rider
        {
            uint32_t golferId = 0;
            int64_t money = 0;
        };
        CartRental rent(const std::vector<Rider>& riders, uint32_t groupId, const FixedVec3& from);
        // The cart leaves its space and drives; the riders are on board.
        void depart(uint32_t cartId);

        // CRT-004: where the cart stops for a shot: on the path nearest to the ball, or within ten yards of it when carts may
        // leave the paths (PTH-005, off by default). `pathPoints` are positions along paths (world units).
        static FixedVec3 stopPoint(const FixedVec3& ball, const std::vector<FixedVec3>& pathPoints, bool allowOffPath);

        // Moves a cart towards its destination for one tick; returns true on arrival.
        bool drive(Cart& cart, const FixedVec3& destination);

        // CRT-005: after the last hole the cart returns to any free space, the nearest. Returns false when every space is
        // taken.
        bool returnCart(uint32_t cartId);
        // The cart has arrived: it stands in its space, the riders are out.
        void park(uint32_t cartId);

        // CRT-006: carts are obstacles; balls and other vehicles avoid them.
        static Obstacle obstacleOf(const Cart& cart);

    private:
        CartSpace* nearestFreeSpace(const FixedVec3& from);

        std::vector<CartSpace> _spaces;
        std::vector<Cart> _carts;
        uint32_t _nextSpaceId = 1;
        uint32_t _nextCartId = 1;
    };
} // namespace OpenRCT2::Golf
