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
#include <openrct2/golf/ObstacleGrid.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Golf;
using namespace GolfTest;

namespace
{
    Obstacle box(uint32_t id, int cx, int cy, int half, int height)
    {
        Obstacle o;
        o.id = id;
        o.shape = ObstacleShape::box;
        o.centre = vec(cx, cy, 0);
        o.halfX = Fixed::fromInt(half);
        o.halfY = Fixed::fromInt(half);
        o.height = Fixed::fromInt(height);
        return o;
    }

    Obstacle cylinder(uint32_t id, int cx, int cy, int radius, int height)
    {
        Obstacle o = box(id, cx, cy, radius, height);
        o.shape = ObstacleShape::cylinder;
        return o;
    }

    const Fixed kBall = Fixed::ratio(1, 2);
} // namespace

TEST(ObstacleGrid, RejectsObstaclesWithoutIdOrSize)
{
    ObstacleGrid grid;
    EXPECT_FALSE(grid.add(box(0, 0, 0, 10, 10)));
    EXPECT_FALSE(grid.add(box(1, 0, 0, 0, 10)));
    EXPECT_FALSE(grid.add(box(1, 0, 0, 10, 0)));
    EXPECT_EQ(grid.size(), 0u);
    EXPECT_TRUE(grid.add(box(1, 0, 0, 10, 10)));
    EXPECT_EQ(grid.size(), 1u);
}

TEST(ObstacleGrid, BoxContactPushesOutAlongTheNearestFace)
{
    ObstacleGrid grid;
    grid.add(box(1, 100, 100, 10, 40));
    ObstacleContact c;
    // Touching the +x face from outside.
    ASSERT_TRUE(grid.query(vec(110, 100, 20) + FixedVec3{ Fixed::ratio(1, 4), {}, {} }, kBall, c));
    EXPECT_EQ(c.id, 1u);
    EXPECT_NEAR(c.normal.x.toDouble(), 1.0, 1e-3);
    EXPECT_NEAR(c.depth.toDouble(), 0.25, 1e-3);
    // From above, the normal points up.
    ASSERT_TRUE(grid.query(FixedVec3{ Fixed::fromInt(100), Fixed::fromInt(100), Fixed::fromInt(40) + Fixed::ratio(1, 4) }, kBall, c));
    EXPECT_NEAR(c.normal.z.toDouble(), 1.0, 1e-3);
    // Out of reach.
    EXPECT_FALSE(grid.query(vec(115, 100, 20), kBall, c));
    EXPECT_FALSE(grid.query(vec(100, 100, 45), kBall, c));
}

TEST(ObstacleGrid, ACentreInsideTheBoxLeavesThroughTheNearestFace)
{
    ObstacleGrid grid;
    grid.add(box(1, 100, 100, 10, 40));
    ObstacleContact c;
    ASSERT_TRUE(grid.query(FixedVec3{ Fixed::fromInt(108), Fixed::fromInt(100), Fixed::fromInt(20) }, kBall, c));
    EXPECT_NEAR(c.normal.x.toDouble(), 1.0, 1e-3);
    EXPECT_NEAR(c.depth.toDouble(), 2.5, 1e-3); // 2 inside plus the radius
}

TEST(ObstacleGrid, CylinderContactUsesTheRadialDirection)
{
    ObstacleGrid grid;
    grid.add(cylinder(2, 200, 200, 8, 30));
    ObstacleContact c;
    ASSERT_TRUE(grid.query(FixedVec3{ Fixed::fromInt(200), Fixed::fromInt(208) + Fixed::ratio(1, 4), Fixed::fromInt(10) }, kBall, c));
    EXPECT_NEAR(c.normal.y.toDouble(), 1.0, 1e-3);
    EXPECT_NEAR(c.normal.x.toDouble(), 0.0, 1e-3);
    // Diagonal: the corner of a box would be hit, but a cylinder is round.
    const double d = 8.0 * 0.7071 + 0.2;
    ASSERT_TRUE(grid.query(FixedVec3{ Fixed::fromInt(200) + Fixed::fromRaw(static_cast<int64_t>(d * 65536)),
                                      Fixed::fromInt(200) + Fixed::fromRaw(static_cast<int64_t>(d * 65536)), Fixed::fromInt(10) },
                           kBall, c));
    EXPECT_NEAR(c.normal.x.toDouble(), 0.7071, 0.02);
    EXPECT_NEAR(c.normal.y.toDouble(), 0.7071, 0.02);
    // The top cap.
    ASSERT_TRUE(grid.query(FixedVec3{ Fixed::fromInt(202), Fixed::fromInt(200), Fixed::fromInt(30) + Fixed::ratio(1, 4) }, kBall, c));
    EXPECT_NEAR(c.normal.z.toDouble(), 1.0, 1e-3);
    // Above the cylinder's top and past its radius: no contact.
    EXPECT_FALSE(grid.query(vec(209, 200, 31), kBall, c));
}

TEST(ObstacleGrid, TheDeepestOverlapWinsAndTiesGoToTheLowerId)
{
    ObstacleGrid grid;
    grid.add(box(7, 100, 100, 10, 40)); // face at x = 110
    grid.add(box(3, 120, 100, 10, 40)); // face at x = 110 from the other side
    ObstacleContact c;
    // Exactly between them (overlapping both by the same amount): the lower id decides.
    ASSERT_TRUE(grid.query(FixedVec3{ Fixed::fromInt(110), Fixed::fromInt(100), Fixed::fromInt(20) }, kBall, c));
    EXPECT_EQ(c.id, 3u);
    // Nearer to the first one: it overlaps deeper... the closer face has the greater depth.
    ASSERT_TRUE(grid.query(FixedVec3{ Fixed::ratio(1095, 10), Fixed::fromInt(100), Fixed::fromInt(20) }, kBall, c));
    EXPECT_EQ(c.id, 7u);
}

TEST(ObstacleGrid, ObstaclesCanBeRemovedAndReplaced)
{
    ObstacleGrid grid;
    grid.add(box(1, 100, 100, 10, 40));
    ObstacleContact c;
    EXPECT_TRUE(grid.query(vec(100, 100, 10), kBall, c));
    EXPECT_TRUE(grid.remove(1));
    EXPECT_FALSE(grid.remove(1));
    EXPECT_FALSE(grid.query(vec(100, 100, 10), kBall, c));
    EXPECT_EQ(grid.find(1), nullptr);

    grid.add(box(2, 100, 100, 10, 40));
    grid.add(box(2, 300, 300, 10, 40)); // same id: replaces, the old cells are freed
    EXPECT_EQ(grid.size(), 1u);
    EXPECT_FALSE(grid.query(vec(100, 100, 10), kBall, c));
    EXPECT_TRUE(grid.query(vec(300, 300, 10), kBall, c));
    grid.clear();
    EXPECT_EQ(grid.size(), 0u);
    EXPECT_FALSE(grid.query(vec(300, 300, 10), kBall, c));
}

TEST(ObstacleGrid, MovingAnObstacleUpdatesItsCells)
{
    ObstacleGrid grid;
    Obstacle golfer = cylinder(9, 100, 100, 4, 50);
    golfer.isGolfer = true;
    golfer.golferId = 33;
    grid.add(golfer);
    ObstacleContact c;
    EXPECT_TRUE(grid.query(vec(100, 100, 10), kBall, c));
    EXPECT_TRUE(c.isGolfer);
    EXPECT_EQ(c.golferId, 33u);

    EXPECT_TRUE(grid.move(9, vec(500, 500, 0)));
    EXPECT_FALSE(grid.query(vec(100, 100, 10), kBall, c));
    EXPECT_TRUE(grid.query(vec(500, 500, 10), kBall, c));
    EXPECT_TRUE(grid.move(9, vec(502, 500, 0))); // within the same cells
    EXPECT_TRUE(grid.query(vec(502, 500, 10), kBall, c));
    EXPECT_FALSE(grid.move(404, vec(0, 0, 0)));
}

TEST(ObstacleGrid, ObstaclesSpanningSeveralTilesAreFoundFromEachOfThem)
{
    ObstacleGrid grid;
    grid.add(box(1, 100, 100, 50, 30)); // 100 units wide: 4 tiles in each direction
    ObstacleContact c;
    for (int x : { 55, 70, 100, 130, 145 })
        EXPECT_TRUE(grid.query(vec(x, 100, 10), kBall, c)) << x;
    EXPECT_FALSE(grid.query(vec(40, 100, 10), kBall, c));
}

TEST(ObstacleGrid, NegativeCoordinatesWork)
{
    ObstacleGrid grid;
    grid.add(box(1, -50, -50, 10, 30));
    ObstacleContact c;
    EXPECT_TRUE(grid.query(vec(-50, -45, 10), kBall, c));
    EXPECT_FALSE(grid.query(vec(50, 50, 10), kBall, c));
}

TEST(ObstacleGrid, QueriesStayLocalWithManyObstacles)
{
    ObstacleGrid grid;
    // 10 000 trees on a 100 x 100 tile course.
    uint32_t id = 1;
    for (int y = 0; y < 100; y++)
        for (int x = 0; x < 100; x++)
            grid.add(cylinder(id++, x * 32 + 16, y * 32 + 16, 3, 40));
    EXPECT_EQ(grid.size(), 10000u);
    ObstacleContact c;
    EXPECT_FALSE(grid.query(vec(5 * 32 + 3, 5 * 32 + 3, 10), kBall, c));
    EXPECT_LE(grid.lastQueryExamined(), 9u);
    EXPECT_TRUE(grid.query(vec(5 * 32 + 16, 5 * 32 + 16 + 3, 10), kBall, c));
    EXPECT_EQ(c.id, 5u * 100 + 5 + 1);
}

TEST(ObstacleGrid, WorksAsTheWorldOfTheBallSimulation)
{
    // The grid plugs into the physics: a ball fired at a wall bounces back (acceptance criterion of section 15).
    class GridWorld final : public IBallWorld
    {
    public:
        SurfaceSample surfaceAt(Fixed, Fixed) const override
        {
            return {};
        }
        bool obstacleAt(const FixedVec3& p, Fixed radius, ObstacleContact& contact) const override
        {
            return grid.query(p, radius, contact);
        }
        ObstacleGrid grid;
    } world;
    Obstacle wall = box(1, 300, 100, 10, 200);
    wall.halfY = Fixed::fromInt(500);
    wall.restitution = Fixed::ratio(5, 10);
    world.grid.add(wall);

    SeqRandom rng;
    Ball ball;
    placeBall(ball, world, Fixed::fromInt(100), Fixed::fromInt(100));
    ball.position.z = Fixed::fromInt(30);
    ball.velocity = vec(6, 0, 1);
    ball.state = BallState::InFlight;
    int hits = 0;
    for (int t = 0; t < 3000 && !isSettled(ball.state); t++)
        hits += stepBall(ball, world, rng).obstacleHits;
    EXPECT_GT(hits, 0);
    EXPECT_LT(ball.position.x, Fixed::fromInt(290));
}

TEST(ObstacleGrid, ObstaclesAreBuiltFromTheCollisionBlockOfAnObject)
{
    ObjectCollision tree;
    tree.shape = CollisionShape::cylinder;
    tree.radius = 6;
    tree.height = 64;
    tree.restitution = 0.25f;
    tree.friction = 0.5f;
    tree.passThroughChance = 0.5f;
    const auto o = makeObstacle(4, tree, vec(40, 50, 8));
    EXPECT_EQ(o.id, 4u);
    EXPECT_EQ(o.shape, ObstacleShape::cylinder);
    EXPECT_EQ(o.halfX, Fixed::fromInt(6));
    EXPECT_EQ(o.height, Fixed::fromInt(64));
    EXPECT_EQ(o.restitution, Fixed::ratio(1, 4));
    EXPECT_EQ(o.friction, Fixed::ratio(1, 2));
    EXPECT_EQ(o.passThroughChance, Fixed::ratio(1, 2));
    EXPECT_EQ(o.centre.z, Fixed::fromInt(8));

    ObjectCollision wall;
    wall.shape = CollisionShape::box;
    wall.footprintX = 32;
    wall.footprintY = 8;
    wall.height = 20;
    wall.restitution = 7.0f; // out of range values are clamped
    const auto straight = makeObstacle(5, wall, vec(0, 0, 0), 0);
    EXPECT_EQ(straight.halfX, Fixed::fromInt(16));
    EXPECT_EQ(straight.halfY, Fixed::fromInt(4));
    EXPECT_EQ(straight.restitution, Fixed::fromInt(1));
    const auto turned = makeObstacle(5, wall, vec(0, 0, 0), 1);
    EXPECT_EQ(turned.halfX, Fixed::fromInt(4));
    EXPECT_EQ(turned.halfY, Fixed::fromInt(16));

    ObjectCollision none;
    none.shape = CollisionShape::none;
    ObstacleGrid grid;
    EXPECT_FALSE(grid.add(makeObstacle(6, none, vec(0, 0, 0))));
    EXPECT_TRUE(grid.add(straight));
}
