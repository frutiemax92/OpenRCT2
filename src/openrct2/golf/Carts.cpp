/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Carts.h"
#include "../object/VehicleModelObject.h"

#include <algorithm>

namespace OpenRCT2::Golf
{
    namespace
    {
        Fixed distanceXY(const FixedVec3& a, const FixedVec3& b)
        {
            return length(FixedVec3{ a.x - b.x, a.y - b.y, Fixed{} });
        }
    } // namespace

    int32_t cartDesire(uint8_t energy, int32_t physique, int32_t courseYards)
    {
        const int32_t p = std::clamp(physique, 0, 100);
        // A tired, unfit golfer on a long course wants a cart most: up to 50 for low physique, 32 for no energy and 30 for
        // a 6000 yard course.
        return (100 - p) / 2 + (255 - energy) / 8 + std::clamp(courseYards, 0, 12000) / 200;
    }

    Model::ContactPoints cartContactPoints()
    {
        auto* cartObj = VehicleModelObject::FindFirstByKind(VehicleModelKind::golfCart);
        if (cartObj != nullptr)
        {
            return cartObj->ContactPoints;
        }
        return { static_cast<float>(kCartHalfLength), static_cast<float>(kCartHalfWidth) };
    }

    bool wantsCart(uint8_t energy, int32_t physique, int32_t courseYards, int64_t money, int64_t cartFee, int32_t jitter)
    {
        if (money < cartFee)
            return false;
        return cartDesire(energy, physique, courseYards) + std::clamp(jitter, 0, 99) / 5 >= kWantCartThreshold;
    }

    CartSpace& CartFleet::addSpace(const FixedVec3& position)
    {
        CartSpace space;
        space.id = _nextSpaceId++;
        space.position = position;
        _spaces.push_back(space);
        return _spaces.back();
    }

    int32_t CartFleet::freeCarts() const
    {
        return static_cast<int32_t>(std::count_if(_carts.begin(), _carts.end(), [](const Cart& c) { return c.state == CartState::parked; }));
    }

    Cart* CartFleet::find(uint32_t cartId)
    {
        for (auto& cart : _carts)
            if (cart.id == cartId)
                return &cart;
        return nullptr;
    }

    CartFleet::Purchase CartFleet::buyCarts(int32_t count)
    {
        Purchase result;
        for (int32_t i = 0; i < count; i++)
        {
            CartSpace* space = nullptr;
            for (auto& s : _spaces)
                if (s.cartId == 0 && s.reservedBy == 0)
                {
                    space = &s;
                    break;
                }
            if (space == nullptr)
                break;
            Cart cart;
            cart.id = _nextCartId++;
            cart.position = space->position;
            cart.spaceId = space->id;
            space->cartId = cart.id;
            _carts.push_back(cart);
            result.bought++;
            result.cost += cartPrice;
        }
        return result;
    }

    CartSpace* CartFleet::nearestFreeSpace(const FixedVec3& from)
    {
        CartSpace* best = nullptr;
        Fixed bestDistance{};
        for (auto& s : _spaces)
        {
            if (s.cartId != 0 || s.reservedBy != 0)
                continue;
            const Fixed d = distanceXY(s.position, from);
            if (best == nullptr || d < bestDistance)
            {
                best = &s;
                bestDistance = d;
            }
        }
        return best;
    }

    CartRental CartFleet::rent(const std::vector<Rider>& riders, uint32_t groupId, const FixedVec3& from)
    {
        CartRental rental;
        rental.feeEach = cartFee;
        size_t next = 0;
        int32_t seats = kCartSeats;
        auto* cartObj = VehicleModelObject::FindFirstByKind(VehicleModelKind::golfCart);
        if (cartObj != nullptr)
        {
            seats = cartObj->Seats;
        }
        while (next < riders.size())
        {
            // The nearest parked cart to the group.
            Cart* best = nullptr;
            Fixed bestDistance{};
            for (auto& c : _carts)
            {
                if (c.state != CartState::parked)
                    continue;
                const Fixed d = distanceXY(c.position, from);
                if (best == nullptr || d < bestDistance)
                {
                    best = &c;
                    bestDistance = d;
                }
            }
            // Golfers who cannot pay walk; so do those left over when the carts run out.
            std::vector<uint32_t> seated;
            while (next < riders.size() && static_cast<int32_t>(seated.size()) < seats)
            {
                const auto& r = riders[next++];
                if (best != nullptr && r.money >= cartFee)
                    seated.push_back(r.golferId);
                else
                    rental.walkers.push_back(r.golferId);
            }
            if (best == nullptr || seated.empty())
                continue;
            best->state = CartState::waitingForGolfers;
            best->riders = seated;
            best->groupId = groupId;
            rental.cartIds.push_back(best->id);
        }
        return rental;
    }

    void CartFleet::depart(uint32_t cartId)
    {
        auto* cart = find(cartId);
        if (cart == nullptr || cart->state != CartState::waitingForGolfers)
            return;
        for (auto& s : _spaces)
            if (s.cartId == cartId)
                s.cartId = 0;
        cart->state = CartState::driving;
    }

    FixedVec3 CartFleet::stopPoint(const FixedVec3& ball, const std::vector<FixedVec3>& pathPoints, bool allowOffPath)
    {
        const FixedVec3* nearest = nullptr;
        Fixed nearestDistance{};
        for (const auto& p : pathPoints)
        {
            const Fixed d = distanceXY(p, ball);
            if (nearest == nullptr || d < nearestDistance)
            {
                nearest = &p;
                nearestDistance = d;
            }
        }
        if (!allowOffPath || nearest == nullptr)
            return nearest != nullptr ? *nearest : ball;
        // Off the path (PTH-005): drive straight at the ball and stop short of it, but never further than the path.
        if (nearestDistance <= Fixed::fromInt(kCartStopDistanceUnits))
            return *nearest;
        const FixedVec3 toBall = normalised(FixedVec3{ ball.x - nearest->x, ball.y - nearest->y, Fixed{} });
        const Fixed travel = nearestDistance - Fixed::fromInt(kCartStopDistanceUnits);
        return { nearest->x + toBall.x * travel, nearest->y + toBall.y * travel, ball.z };
    }

    bool CartFleet::drive(Cart& cart, const FixedVec3& destination)
    {
        Fixed top = Fixed::ratio(kCartTopSpeedHundredths, 100);
        Fixed accel = Fixed::ratio(8, 100);
        auto* cartObj = VehicleModelObject::FindFirstByKind(VehicleModelKind::golfCart);
        if (cartObj != nullptr)
        {
            top = cartObj->MaxSpeed;
            accel = cartObj->Acceleration;
        }
        const FixedVec3 delta{ destination.x - cart.position.x, destination.y - cart.position.y, Fixed{} };
        const Fixed distance = length(delta);
        // Brake for the last stretch.
        const Fixed wanted = min(top, max(Fixed::ratio(30, 100), distance / 12));
        cart.speed = cart.speed < wanted ? min(wanted, cart.speed + accel) : wanted;
        if (distance <= cart.speed || distance <= Fixed::fromInt(1))
        {
            cart.position.x = destination.x;
            cart.position.y = destination.y;
            cart.speed = Fixed{};
            return true;
        }
        cart.position.x += delta.x * cart.speed / distance;
        cart.position.y += delta.y * cart.speed / distance;
        return false;
    }

    bool CartFleet::returnCart(uint32_t cartId)
    {
        auto* cart = find(cartId);
        if (cart == nullptr)
            return false;
        auto* space = nearestFreeSpace(cart->position);
        if (space == nullptr)
            return false;
        space->reservedBy = cartId;
        cart->spaceId = space->id;
        cart->destination = space->position;
        cart->state = CartState::returning;
        return true;
    }

    void CartFleet::park(uint32_t cartId)
    {
        auto* cart = find(cartId);
        if (cart == nullptr || cart->state != CartState::returning)
            return;
        for (auto& s : _spaces)
            if (s.id == cart->spaceId)
            {
                s.cartId = cartId;
                s.reservedBy = 0;
                cart->position = s.position;
            }
        cart->riders.clear();
        cart->groupId = 0;
        cart->state = CartState::parked;
    }

    Obstacle CartFleet::obstacleOf(const Cart& cart)
    {
        auto* cartObj = VehicleModelObject::FindFirstByKind(VehicleModelKind::golfCart);
        if (cartObj != nullptr && cartObj->Collision.shape != CollisionShape::none)
        {
            auto o = makeObstacle(0x50000000u + cart.id, cartObj->Collision, cart.position, 0);
            o.id = 0x50000000u + cart.id;
            return o;
        }
        Obstacle o;
        o.id = 0x50000000u + cart.id;
        o.shape = ObstacleShape::box;
        o.centre = cart.position;
        o.halfX = Fixed::fromInt(kCartHalfLength);
        o.halfY = Fixed::fromInt(kCartHalfWidth);
        o.height = Fixed::fromInt(14);
        o.restitution = Fixed::ratio(3, 10);
        o.friction = Fixed::ratio(3, 10);
        return o;
    }
} // namespace OpenRCT2::Golf
