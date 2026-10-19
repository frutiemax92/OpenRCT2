/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

// AST-001, AST-002: g1 and g2 images come from the project's base graphics pack, not from RCT files.

#include "TestData.h"

#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <openrct2/OpenRCT2.h>
#include <openrct2/PlatformEnvironment.h>
#include <openrct2/SpriteIds.h>
#include <openrct2/command_line/sprite/SpriteCommands.h>
#include <openrct2/core/Path.hpp>
#include <openrct2/drawing/Drawing.Sprite.h>
#include <openrct2/drawing/ImageImporter.h>

using namespace OpenRCT2;

class BaseGraphicsTests : public testing::Test
{
protected:
    std::filesystem::path _dir;
    std::unique_ptr<IPlatformEnvironment> _env;
    bool _headless{};
    bool _noGraphics{};

    void SetUp() override
    {
        _headless = gOpenRCT2Headless;
        _noGraphics = gOpenRCT2NoGraphics;
        gOpenRCT2Headless = true; // no message box on failure
        gOpenRCT2NoGraphics = false;

        _dir = std::filesystem::temp_directory_path()
            / ("openrct2-base-graphics-" + std::string(testing::UnitTest::GetInstance()->current_test_info()->name()));
        std::filesystem::remove_all(_dir);
        std::filesystem::create_directories(_dir);
        std::filesystem::copy_file(
            std::filesystem::u8path(Path::Combine(TestData::GetBasePath(), u8"images", u8"logo.png")), _dir / "logo.png");

        DirBaseValues basePaths;
        basePaths[EnumValue(DirBase::openrct2)] = _dir.u8string();
        _env = CreatePlatformEnvironment(basePaths);
    }

    void TearDown() override
    {
        GfxUnloadBaseGraphics();
        gOpenRCT2Headless = _headless;
        gOpenRCT2NoGraphics = _noGraphics;
        std::filesystem::remove_all(_dir);
    }

    void BuildPack(const std::string& manifest)
    {
        auto manifestPath = (_dir / "sprites.json").string();
        std::ofstream(manifestPath) << manifest;
        auto output = (_dir / kBaseGraphicsFileName).string();
        const char* cmd[4] = { "build", output.c_str(), manifestPath.c_str(), "silent" };
        ASSERT_EQ(CommandLine::ExitCode::ok, CommandLine::Sprite::build(cmd, 4, Drawing::ImportMode::standard));
    }
};

TEST_F(BaseGraphicsTests, LoadsSlotsIntoTheG1AndG2Ranges)
{
    BuildPack(
        R"([ { "slot": )" + std::to_string(SPR_RESIZE) + R"(, "path": "logo.png", "x": 3, "y": -4 },
             { "slot": )" + std::to_string(SPR_G2_LOGO) + R"(, "path": "logo.png" } ])");
    ASSERT_TRUE(GfxLoadBaseGraphics(*_env));

    const auto* resize = GfxGetG1Element(SPR_RESIZE);
    ASSERT_NE(nullptr, resize);
    ASSERT_GT(resize->width, 0);
    ASSERT_GT(resize->height, 0);
    ASSERT_EQ(3, resize->xOffset);
    ASSERT_EQ(-4, resize->yOffset);

    const auto* logo = GfxGetG1Element(SPR_G2_LOGO);
    ASSERT_NE(nullptr, logo);
    ASSERT_EQ(resize->width, logo->width);
}

TEST_F(BaseGraphicsTests, SlotsNotInThePackAreEmpty)
{
    BuildPack(R"([ { "slot": )" + std::to_string(SPR_RESIZE) + R"(, "path": "logo.png" } ])");
    ASSERT_TRUE(GfxLoadBaseGraphics(*_env));

    for (ImageIndex index : { ImageIndex{ 0 }, ImageIndex{ SPR_RESIZE - 1 }, ImageIndex{ SPR_RESIZE + 1 },
                              ImageIndex{ SPR_G1_END - 1 }, ImageIndex{ SPR_G2_BEGIN }, ImageIndex{ SPR_G2_END - 1 } })
    {
        const auto* element = GfxGetG1Element(index);
        ASSERT_NE(nullptr, element) << index;
        ASSERT_EQ(0, element->width) << index;
        ASSERT_EQ(0, element->height) << index;
    }
}

TEST_F(BaseGraphicsTests, MissingPackFailsToLoad)
{
    ASSERT_FALSE(GfxLoadBaseGraphics(*_env));
    ASSERT_EQ(nullptr, GfxGetG1Element(SPR_RESIZE));
}

TEST_F(BaseGraphicsTests, PackLargerThanTheRangeFailsToLoad)
{
    BuildPack(R"([ { "slot": )" + std::to_string(SPR_G2_END) + R"(, "path": "logo.png" } ])");
    ASSERT_FALSE(GfxLoadBaseGraphics(*_env));
    ASSERT_EQ(nullptr, GfxGetG1Element(SPR_RESIZE));
}

TEST_F(BaseGraphicsTests, NoRct2PathIsNeeded)
{
    // The environment has no RCT1/RCT2 base paths at all.
    ASSERT_TRUE(_env->GetDirectoryPath(DirBase::rct2).empty());
    BuildPack(R"([ { "slot": 0, "path": "logo.png" } ])");
    ASSERT_TRUE(GfxLoadBaseGraphics(*_env));
}
