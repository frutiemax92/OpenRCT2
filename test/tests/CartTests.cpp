/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "GolfTestWorld.h"

#include <gtest/gtest.h>
#include <openrct2/golf/Carts.h>
#include <openrct2/model/TerrainAlignment.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Golf;
using GolfTest::vec;

namespace
{
    CartFleet fleetWith(int spaces, int carts)
    {
        CartFleet fleet;
        for (int i = 0; i < spaces; i++)
            fleet.addSpace(vec(100 + i * 20, 100, 0));
        fleet.buyCarts(carts);
        return fleet;
    }

    std::vector<CartFleet::Rider> group(int n, int64_t money = 5000)
    {
        std::vector<CartFleet::Rider> riders;
        for (int i = 0; i < n; i++)
            riders.push_back({ static_cast<uint32_t>(i + 1), money });
        return riders;
    }
} // namespace

TEST(CartDesire, TiredUnfitGolfersOnLongCoursesWantACart)
{
    // Fresh, fit golfer on a short course: no.
    EXPECT_FALSE(wantsCart(255, 90, 2800, 5000, 600));
    // Unfit, tired, on a long course: yes.
    EXPECT_TRUE(wantsCart(60, 10, 6400, 5000, 600));
    EXPECT_GT(cartDesire(60, 10, 6400), cartDesire(255, 90, 2800));
    EXPECT_GT(cartDesire(100, 50, 5000), cartDesire(200, 50, 5000)); // tiredness
    EXPECT_GT(cartDesire(200, 20, 5000), cartDesire(200, 80, 5000)); // physique
    EXPECT_GT(cartDesire(200, 50, 7000), cartDesire(200, 50, 3000)); // length
}

TEST(CartDesire, PoorGolfersCannotAffordOne)
{
    EXPECT_FALSE(wantsCart(10, 0, 9000, 599, 600));
    EXPECT_TRUE(wantsCart(10, 0, 9000, 600, 600));
}

TEST(CartDesire, JitterTipsBorderlineGolfers)
{
    // A fit golfer on a short course gets tired step by step until they are a few points under the threshold.
    int energy = 255;
    while (energy > 5 && cartDesire(static_cast<uint8_t>(energy), 80, 3000) < kWantCartThreshold - 5)
        energy -= 1;
    // Now the desire is within five points under the threshold.
    const int32_t desire = cartDesire(static_cast<uint8_t>(energy), 80, 3000);
    ASSERT_LT(desire, kWantCartThreshold);
    ASSERT_GE(desire, kWantCartThreshold - 5);
    EXPECT_FALSE(wantsCart(static_cast<uint8_t>(energy), 80, 3000, 5000, 600, 0));
    EXPECT_TRUE(wantsCart(static_cast<uint8_t>(energy), 80, 3000, 5000, 600, 99));
    EXPECT_EQ(wantsCart(10, 0, 9000, 5000, 600, 500), wantsCart(10, 0, 9000, 5000, 600, 99)); // jitter is clamped
}

TEST(CartFleet, CartsAreBoughtIntoFreeSpaces)
{
    CartFleet fleet = fleetWith(3, 0);
    fleet.cartPrice = 4000;
    const auto purchase = fleet.buyCarts(2);
    EXPECT_EQ(purchase.bought, 2);
    EXPECT_EQ(purchase.cost, 8000);
    EXPECT_EQ(fleet.cartCount(), 2);
    EXPECT_EQ(fleet.freeCarts(), 2);
    // Only one space is left: asking for four buys one.
    const auto more = fleet.buyCarts(4);
    EXPECT_EQ(more.bought, 1);
    EXPECT_EQ(more.cost, 4000);
    EXPECT_EQ(fleet.buyCarts(1).bought, 0);
    EXPECT_EQ(fleet.cartCount(), 3);
    // Each cart stands in its own space.
    for (const auto& cart : fleet.carts())
        EXPECT_EQ(fleet.spaces()[cart.spaceId - 1].cartId, cart.id);
}

TEST(CartFleet, WithoutSpacesNothingCanBeBought)
{
    CartFleet fleet;
    EXPECT_EQ(fleet.buyCarts(5).bought, 0);
    EXPECT_EQ(fleet.cartCount(), 0);
    EXPECT_EQ(fleet.spaceCount(), 0);
}

TEST(CartFleet, ACartCarriesTwoGolfersOfAGroup)
{
    auto fleet = fleetWith(4, 2);
    const auto rental = fleet.rent(group(2), 7, vec(0, 100, 0));
    ASSERT_EQ(rental.cartIds.size(), 1u);
    EXPECT_TRUE(rental.walkers.empty());
    const auto* cart = fleet.find(rental.cartIds[0]);
    ASSERT_NE(cart, nullptr);
    EXPECT_EQ(cart->riders.size(), 2u);
    EXPECT_EQ(cart->groupId, 7u);
    EXPECT_EQ(cart->state, CartState::waitingForGolfers);
    EXPECT_EQ(fleet.freeCarts(), 1);
    EXPECT_EQ(rental.feeEach, 600);
}

TEST(CartFleet, WithTwoCartsAGroupOfFourUsesBothAndALaterGroupWalks)
{
    // Acceptance criterion of section 14.
    auto fleet = fleetWith(4, 2);
    const auto first = fleet.rent(group(4), 1, vec(0, 100, 0));
    EXPECT_EQ(first.cartIds.size(), 2u);
    EXPECT_TRUE(first.walkers.empty());
    EXPECT_EQ(fleet.freeCarts(), 0);
    const auto later = fleet.rent(group(3), 2, vec(0, 100, 0));
    EXPECT_TRUE(later.cartIds.empty());
    EXPECT_EQ(later.walkers.size(), 3u);
}

TEST(CartFleet, OddGroupsLeaveOneSeatEmpty)
{
    auto fleet = fleetWith(4, 3);
    const auto rental = fleet.rent(group(3), 1, vec(0, 100, 0));
    ASSERT_EQ(rental.cartIds.size(), 2u);
    EXPECT_EQ(fleet.find(rental.cartIds[0])->riders.size(), 2u);
    EXPECT_EQ(fleet.find(rental.cartIds[1])->riders.size(), 1u);
}

TEST(CartFleet, RunningOutOfCartsMidGroupLeavesTheRestWalking)
{
    auto fleet = fleetWith(4, 1);
    const auto rental = fleet.rent(group(4), 1, vec(0, 100, 0));
    EXPECT_EQ(rental.cartIds.size(), 1u);
    EXPECT_EQ(rental.walkers.size(), 2u);
    EXPECT_EQ(rental.walkers[0], 3u);
    EXPECT_EQ(rental.walkers[1], 4u);
}

TEST(CartFleet, GolfersWhoCannotPayWalk)
{
    auto fleet = fleetWith(4, 2);
    std::vector<CartFleet::Rider> riders = { { 1, 5000 }, { 2, 100 }, { 3, 5000 } };
    const auto rental = fleet.rent(riders, 1, vec(0, 100, 0));
    ASSERT_EQ(rental.walkers.size(), 1u);
    EXPECT_EQ(rental.walkers[0], 2u);
    EXPECT_EQ(rental.cartIds.size(), 1u); // golfers 1 and 3 share one
    EXPECT_EQ(fleet.find(rental.cartIds[0])->riders, (std::vector<uint32_t>{ 1, 3 }));
}

TEST(CartFleet, TheNearestCartIsTakenFirst)
{
    auto fleet = fleetWith(4, 4);
    const auto rental = fleet.rent(group(2), 1, vec(10000, 100, 0));
    ASSERT_EQ(rental.cartIds.size(), 1u);
    EXPECT_EQ(fleet.find(rental.cartIds[0])->position.x, Fixed::fromInt(100 + 3 * 20)); // the last space is closest
}

TEST(CartFleet, DepartingFreesTheSpace)
{
    auto fleet = fleetWith(2, 2);
    const auto rental = fleet.rent(group(2), 1, vec(0, 100, 0));
    const uint32_t id = rental.cartIds[0];
    fleet.depart(id);
    EXPECT_EQ(fleet.find(id)->state, CartState::driving);
    int free = 0;
    for (const auto& s : fleet.spaces())
        free += s.cartId == 0 ? 1 : 0;
    EXPECT_EQ(free, 1);
    fleet.depart(id); // already gone: nothing happens
    fleet.depart(404);
}

TEST(CartFleet, CartsDriveToTheirDestinationAndBrakeForIt)
{
    auto fleet = fleetWith(1, 1);
    Cart& cart = fleet.carts().empty() ? *fleet.find(0) : *fleet.find(fleet.carts()[0].id);
    const FixedVec3 target = vec(900, 100, 0);
    int ticks = 0;
    Fixed fastest{};
    while (!fleet.drive(cart, target) && ticks < 5000)
    {
        fastest = max(fastest, cart.speed);
        ticks++;
    }
    EXPECT_EQ(cart.position.x, target.x);
    EXPECT_EQ(cart.position.y, target.y);
    EXPECT_LE(fastest, Fixed::ratio(kCartTopSpeedHundredths, 100));
    EXPECT_GT(fastest, Fixed::ratio(1, 1));
    // 800 units at about 2.2 per tick.
    EXPECT_GT(ticks, 300);
    EXPECT_LT(ticks, 500);
    EXPECT_EQ(cart.speed, Fixed{});
}

TEST(CartFleet, ACartStopsAtThePathNearestTheBall)
{
    const std::vector<FixedVec3> path = { vec(0, 0, 0), vec(100, 0, 0), vec(200, 0, 0), vec(300, 0, 0) };
    const auto stop = CartFleet::stopPoint(vec(210, 150, 0), path, false);
    EXPECT_EQ(stop, vec(200, 0, 0));
    // With no path at all the cart stays with the ball.
    EXPECT_EQ(CartFleet::stopPoint(vec(5, 5, 0), {}, false), vec(5, 5, 0));
    EXPECT_EQ(CartFleet::stopPoint(vec(5, 5, 0), {}, true), vec(5, 5, 0));
}

TEST(CartFleet, WhereCartsMayLeaveThePathTheyStopTenYardsShortOfTheBall)
{
    const std::vector<FixedVec3> path = { vec(0, 0, 0), vec(200, 0, 0) };
    const auto ball = vec(200, 300, 0);
    const auto stop = CartFleet::stopPoint(ball, path, true);
    EXPECT_NEAR(stop.x.toDouble(), 200.0, 0.1);
    EXPECT_NEAR(stop.y.toDouble(), 300.0 - kCartStopDistanceUnits, 0.5);
    // A ball closer than that is reached from the path.
    EXPECT_EQ(CartFleet::stopPoint(vec(200, 30, 0), path, true), vec(200, 0, 0));
}

TEST(CartFleet, CartsReturnToAnyFreeSpaceTheNearest)
{
    auto fleet = fleetWith(4, 1);
    const auto rental = fleet.rent(group(1), 1, vec(0, 100, 0));
    const uint32_t id = rental.cartIds[0];
    fleet.depart(id);
    // Out on the course near the far end of the row of spaces.
    fleet.find(id)->position = vec(300, 100, 0);
    ASSERT_TRUE(fleet.returnCart(id));
    EXPECT_EQ(fleet.find(id)->state, CartState::returning);
    EXPECT_EQ(fleet.find(id)->destination, vec(100 + 3 * 20, 100, 0));
    // The reserved space is not given to another cart.
    EXPECT_EQ(fleet.buyCarts(5).bought, 3); // four spaces, one of them reserved for the returning cart
    fleet.park(id);
    EXPECT_EQ(fleet.find(id)->state, CartState::parked);
    EXPECT_TRUE(fleet.find(id)->riders.empty());
    EXPECT_EQ(fleet.find(id)->position, vec(160, 100, 0));
    EXPECT_EQ(fleet.freeCarts(), 4);
}

TEST(CartFleet, ACartWithNoFreeSpaceCannotReturn)
{
    auto fleet = fleetWith(1, 1);
    const auto rental = fleet.rent(group(1), 1, vec(0, 100, 0));
    fleet.depart(rental.cartIds[0]);
    fleet.buyCarts(1); // takes the space the cart left
    EXPECT_FALSE(fleet.returnCart(rental.cartIds[0]));
    EXPECT_FALSE(fleet.returnCart(404));
    fleet.park(rental.cartIds[0]); // not returning: ignored
    EXPECT_EQ(fleet.find(rental.cartIds[0])->state, CartState::driving);
}

TEST(CartFleet, CartsAreObstaclesForBallsAndTiltOnSlopes)
{
    auto fleet = fleetWith(1, 1);
    const auto& cart = fleet.carts()[0];
    const auto o = CartFleet::obstacleOf(cart);
    EXPECT_EQ(o.shape, ObstacleShape::box);
    EXPECT_EQ(o.halfX, Fixed::fromInt(kCartHalfLength));
    EXPECT_EQ(o.centre, cart.position);
    EXPECT_NE(o.id, 0u);
    ObstacleGrid grid;
    EXPECT_TRUE(grid.add(o));
    ObstacleContact c;
    EXPECT_TRUE(grid.query(cart.position + vec(0, 0, 6), Fixed::ratio(1, 2), c));

    // A cart driving across a slope tilts visibly (acceptance criterion of section 11): the wheel contacts give a pitch.
    Model::ContactPoints wheels = cartContactPoints();
    const auto slope = [](float x, float) { return x * 0.3f; };
    const auto a = Model::computeAlignment(slope, 100, 100, 0, wheels);
    EXPECT_LT(a.pitch, -0.2f);
}

TEST(CartFleet, CartContactPointsMatchModelConstants)
{
    const auto cp = cartContactPoints();
    EXPECT_FLOAT_EQ(static_cast<float>(kCartHalfLength), cp.halfLength);
    EXPECT_FLOAT_EQ(static_cast<float>(kCartHalfWidth), cp.halfWidth);
}
