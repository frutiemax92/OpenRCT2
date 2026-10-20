/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <gtest/gtest.h>
#include <openrct2/object/ObjectCollision.h>

using namespace OpenRCT2;

static json_t parse(const char* text)
{
    return Json::FromString(text);
}

TEST(ObjectCollision, AbsentBlockDerivesFromClearanceAndFootprint)
{
    auto props = parse(R"({"height": 24})");
    const auto c = readObjectCollision(props, 24, 32, 32);
    EXPECT_EQ(c.shape, CollisionShape::box);
    EXPECT_EQ(c.height, 24);
    EXPECT_EQ(c.footprintX, 32);
    EXPECT_EQ(c.footprintY, 32);
    EXPECT_FLOAT_EQ(c.restitution, 0.4f);
    EXPECT_FLOAT_EQ(c.friction, 0.3f);
    EXPECT_FLOAT_EQ(c.passThroughChance, 0.0f);
}

TEST(ObjectCollision, ReadsEveryField)
{
    auto props = parse(R"({"collision": {"shape": "box", "height": 48, "footprint": [16, 8],
        "restitution": 0.5, "friction": 0.6, "passThroughChance": 0.25}})");
    const auto c = readObjectCollision(props, 10);
    EXPECT_EQ(c.shape, CollisionShape::box);
    EXPECT_EQ(c.height, 48);
    EXPECT_EQ(c.footprintX, 16);
    EXPECT_EQ(c.footprintY, 8);
    EXPECT_FLOAT_EQ(c.restitution, 0.5f);
    EXPECT_FLOAT_EQ(c.friction, 0.6f);
    EXPECT_FLOAT_EQ(c.passThroughChance, 0.25f);
}

TEST(ObjectCollision, CylinderUsesRadiusAndPartialBlocksKeepDefaults)
{
    auto props = parse(R"({"collision": {"shape": "cylinder", "radius": 6}})");
    const auto c = readObjectCollision(props, 64);
    EXPECT_EQ(c.shape, CollisionShape::cylinder);
    EXPECT_EQ(c.radius, 6);
    EXPECT_EQ(c.height, 64);
    EXPECT_FLOAT_EQ(c.restitution, 0.4f);
}

TEST(ObjectCollision, NoneAndUnknownShapes)
{
    auto none = parse(R"({"collision": {"shape": "none"}})");
    EXPECT_EQ(readObjectCollision(none, 8).shape, CollisionShape::none);
    auto bogus = parse(R"({"collision": {"shape": "sphere"}})");
    EXPECT_EQ(readObjectCollision(bogus, 8).shape, CollisionShape::box);
}

TEST(ObjectCollision, UnitValuesAreClampedAndBadTypesIgnored)
{
    auto props = parse(R"({"collision": {"restitution": 3, "friction": -1, "passThroughChance": 0.5}})");
    const auto c = readObjectCollision(props, 8);
    EXPECT_FLOAT_EQ(c.restitution, 1.0f);
    EXPECT_FLOAT_EQ(c.friction, 0.0f);
    EXPECT_FLOAT_EQ(c.passThroughChance, 0.5f);

    auto notAnObject = parse(R"({"collision": 5})");
    EXPECT_EQ(readObjectCollision(notAnObject, 8).height, 8);
}
