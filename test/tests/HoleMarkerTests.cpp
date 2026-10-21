/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <algorithm>
#include <gtest/gtest.h>
#include <openrct2/drawing/GolfBallSprites.h>
#include <openrct2/drawing/HoleMarkerSprites.h>
#include <openrct2/golf/HoleMarkers.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Golf;
using namespace OpenRCT2::Drawing;

namespace
{
    // A course with one hole: tees at tile (2,3), a cup at tile (12,3).
    CourseManager makeCourse(uint32_t* holeId = nullptr)
    {
        CourseManager courses;
        auto* course = courses.createCourse("Links");
        auto* hole = courses.createHole(course->id);
        hole->tees[0] = HolePoint{ 2 * 32 + 10, 3 * 32 + 12 };
        hole->tees[1] = HolePoint{ 2 * 32 + 20, 3 * 32 + 12 };
        hole->cup = HolePoint{ 12 * 32 + 16, 3 * 32 + 16 };
        hole->aimPoints[0] = HolePoint{ 7 * 32 + 5, 3 * 32 + 5 };
        if (holeId != nullptr)
            *holeId = hole->id;
        return courses;
    }
} // namespace

TEST(HoleMarkers, FindsTheTeesTheCupAndTheAimPointsByTile)
{
    uint32_t holeId = 0;
    auto courses = makeCourse(&holeId);
    HoleMarkerIndex index;
    index.update(courses);
    EXPECT_EQ(index.markerCount(), 4u);

    const auto* tees = index.at({ 2, 3 });
    ASSERT_NE(tees, nullptr);
    ASSERT_EQ(tees->size(), 2u);
    EXPECT_EQ((*tees)[0].kind, HoleMarker::Kind::tee);
    EXPECT_EQ((*tees)[0].index, 0);
    EXPECT_EQ((*tees)[1].index, 1);
    EXPECT_EQ((*tees)[0].holeId, holeId);

    const auto* cup = index.at({ 12, 3 });
    ASSERT_NE(cup, nullptr);
    ASSERT_EQ(cup->size(), 1u);
    EXPECT_EQ((*cup)[0].kind, HoleMarker::Kind::cup);
    EXPECT_EQ((*cup)[0].x, 12 * 32 + 16);

    const auto* aim = index.at({ 7, 3 });
    ASSERT_NE(aim, nullptr);
    EXPECT_EQ((*aim)[0].kind, HoleMarker::Kind::aim);

    EXPECT_EQ(index.at({ 5, 5 }), nullptr);
}

TEST(HoleMarkers, FollowsChangesToTheCourses)
{
    auto courses = makeCourse();
    HoleMarkerIndex index;
    index.update(courses);
    ASSERT_NE(index.at({ 12, 3 }), nullptr);

    // Moving the cup to another tile moves its marker.
    courses.courses()[0].holes[0].cup = HolePoint{ 20 * 32 + 3, 9 * 32 + 3 };
    index.update(courses);
    EXPECT_EQ(index.at({ 12, 3 }), nullptr);
    ASSERT_NE(index.at({ 20, 9 }), nullptr);

    // Removing a tee removes it.
    courses.courses()[0].holes[0].tees[1].reset();
    index.update(courses);
    EXPECT_EQ(index.at({ 2, 3 })->size(), 1u);

    // No courses, no markers.
    courses.clear();
    index.update(courses);
    EXPECT_EQ(index.markerCount(), 0u);
    EXPECT_EQ(index.at({ 2, 3 }), nullptr);
}

TEST(HoleMarkers, ThingsOnTheSameTileShareIt)
{
    CourseManager courses;
    auto* course = courses.createCourse("Links");
    auto* a = courses.createHole(course->id);
    a->tees[0] = HolePoint{ 40, 40 };
    auto* b = courses.createHole(course->id);
    b->cup = HolePoint{ 50, 50 };
    HoleMarkerIndex index;
    index.update(courses);
    ASSERT_NE(index.at({ 1, 1 }), nullptr);
    EXPECT_EQ(index.at({ 1, 1 })->size(), 2u);
}

TEST(HoleMarkerSprites, TeeMarkersHaveOneColourPerCategory)
{
    std::vector<std::vector<uint8_t>> seen;
    for (size_t category = 0; category < HoleMarkerSprites::kTeeCategories; category++)
    {
        const auto marker = MakeTeeMarkerPixels(category);
        EXPECT_EQ(marker.pixels.size(), static_cast<size_t>(marker.width * marker.height));
        EXPECT_GT(std::count_if(marker.pixels.begin(), marker.pixels.end(), [](uint8_t p) { return p != 0; }), 0);
        for (const auto& other : seen)
            EXPECT_NE(other, marker.pixels) << "category " << category << " looks like another one";
        seen.push_back(marker.pixels);
        // The foot is the bottom of the sprite.
        EXPECT_EQ(marker.yOffset, -marker.height);
    }
    EXPECT_EQ(seen.size(), HoleMarkerSprites::kTeeCategories);
}

TEST(HoleMarkerSprites, ThePinIsATallPoleWithAFlagAndTheCupIsAFlatEllipse)
{
    const auto pin = MakePinPixels();
    EXPECT_EQ(pin.height, 20);
    // The pole runs the full height in the first column; the flag is only at the top.
    for (int32_t y = 0; y < pin.height; y++)
        EXPECT_NE(pin.pixels[static_cast<size_t>(y * pin.width)], 0) << "row " << y;
    EXPECT_NE(pin.pixels[static_cast<size_t>(1)], 0);
    EXPECT_EQ(pin.pixels[static_cast<size_t>(10 * pin.width + 1)], 0);

    const auto cup = MakeCupPixels();
    EXPECT_GT(cup.width, cup.height);
    EXPECT_EQ(cup.pixels[0], 0); // round corners

    const auto aim = MakeAimPixels();
    EXPECT_NE(aim.pixels[static_cast<size_t>(3 * aim.width + 3)], 0);
    EXPECT_EQ(aim.pixels[0], 0);
}

TEST(HoleMarkerSprites, PaletteSearchFindsTheClosestColour)
{
    // The same colour twice gives the same index, and black and white are not the same.
    EXPECT_EQ(FindPaletteIndex(255, 255, 255), FindWhiteIndex());
    EXPECT_NE(FindPaletteIndex(255, 255, 255), FindPaletteIndex(10, 10, 10));
}
