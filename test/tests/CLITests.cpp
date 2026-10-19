
#include "TestData.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <iterator>
#include <openrct2/command_line/sprite/SpriteCommands.h>
#include <openrct2/command_line/sprite/SpriteFile.h>
#include <openrct2/core/Path.hpp>

using namespace OpenRCT2;
using namespace OpenRCT2::CommandLine;
using namespace OpenRCT2::CommandLine::Sprite;

class CommandLineTests : public testing::Test
{
public:
    static std::string SpriteTestDataPath()
    {
        return Path::Combine(TestData::GetBasePath(), u8"sprites");
    }

    static std::string ManifestFilePath()
    {
        return Path::Combine(SpriteTestDataPath(), u8"manifest.json");
    }

    static std::string BadManifestFilePath()
    {
        return Path::Combine(SpriteTestDataPath(), u8"badManifest.json");
    }

    static std::string ExampleSpriteFilePath()
    {
        return Path::Combine(SpriteTestDataPath(), u8"example.dat");
    }

    static std::string BuildOutputfilePath()
    {
        return Path::Combine(SpriteTestDataPath(), u8"result.dat");
    }

    static bool CompareSpriteFiles(std::string original, std::string generated)
    {
        std::ifstream originalFile(original, std::ios::binary | std::ifstream::in);
        std::ifstream generatedFile(generated, std::ios::binary | std::ifstream::in);
        if (!(originalFile.is_open() && generatedFile.is_open()))
        {
            return false;
        }
        if (originalFile.tellg() != generatedFile.tellg())
        {
            return false;
        }
        return std::equal(
            std::istreambuf_iterator<char>(originalFile.rdbuf()), std::istreambuf_iterator<char>(),
            std::istreambuf_iterator<char>(generatedFile.rdbuf()));
    }
};

TEST_F(CommandLineTests, command_line_for_sprite_details)
{
    std::string exampleFilePath = ExampleSpriteFilePath();
    const char* detailsCmd[3] = { "details", exampleFilePath.c_str() };

    auto result = details(detailsCmd, 2);
    // need to come up with some way to extract stdout/stderr stream if we want to
    // fully test this module
    ASSERT_EQ(result, ExitCode::ok);
}

TEST_F(CommandLineTests, command_line_for_sprite_build)
{
    std::string manifestFilePath = ManifestFilePath();
    std::string outputfilePath = BuildOutputfilePath();
    const char* buildCmd[3] = { "build", outputfilePath.c_str(), manifestFilePath.c_str() };

    auto result = build(buildCmd, 3, ImportMode::standard);
    ASSERT_EQ(result, ExitCode::ok);
    // compare the resulting output file and assert its identical to expected
    ASSERT_TRUE(CompareSpriteFiles(ExampleSpriteFilePath(), outputfilePath));
}

TEST_F(CommandLineTests, command_line_for_sprite_failed_build)
{
    // run on correct manifest file
    std::string manifestFilePath = ManifestFilePath();
    std::string outputfilePath = BuildOutputfilePath();
    const char* buildCmd[3] = { "build", outputfilePath.c_str(), manifestFilePath.c_str() };
    auto result = build(buildCmd, 3, ImportMode::standard);
    ASSERT_EQ(result, ExitCode::ok);
    ASSERT_TRUE(CompareSpriteFiles(ExampleSpriteFilePath(), outputfilePath));

    // now use bad manifest and make sure output file is not edited
    std::string badManifestFilePath = BadManifestFilePath();
    buildCmd[2] = badManifestFilePath.c_str();
    result = build(buildCmd, 3, ImportMode::standard);
    // check the command failed
    ASSERT_EQ(result, ExitCode::fail);
    // validate the target file was unchanged
    ASSERT_TRUE(CompareSpriteFiles(ExampleSpriteFilePath(), outputfilePath));
}

// AST-002: sparse sprite files keep stable indices through "slot".
class SpriteBuildSlotTests : public testing::Test
{
protected:
    std::filesystem::path _dir;

    void SetUp() override
    {
        _dir = std::filesystem::temp_directory_path()
            / ("openrct2-slot-test-" + std::string(testing::UnitTest::GetInstance()->current_test_info()->name()));
        std::filesystem::remove_all(_dir);
        std::filesystem::create_directories(_dir);
        std::filesystem::copy_file(
            std::filesystem::u8path(Path::Combine(TestData::GetBasePath(), u8"images", u8"logo.png")), _dir / "logo.png");
    }

    void TearDown() override
    {
        std::filesystem::remove_all(_dir);
    }

    ExitCode Build(const std::string& manifest)
    {
        auto manifestPath = (_dir / "manifest.json").string();
        std::ofstream(manifestPath) << manifest;
        _output = (_dir / "out.gx").string();
        const char* cmd[4] = { "build", _output.c_str(), manifestPath.c_str(), "silent" };
        return build(cmd, 4, ImportMode::standard);
    }

    std::string _output;
};

TEST_F(SpriteBuildSlotTests, SlotsPadWithEmptyEntries)
{
    ASSERT_EQ(ExitCode::ok, Build(R"([ { "path": "logo.png" }, { "slot": 3, "path": "logo.png", "x": -2, "y": 5 } ])"));
    auto file = SpriteFile::Open(_output.c_str());
    ASSERT_TRUE(file.has_value());
    ASSERT_EQ(4u, file->Header.numEntries);
    ASSERT_GT(file->Entries[0].width, 0);
    for (size_t i : { 1, 2 })
    {
        ASSERT_EQ(0, file->Entries[i].width) << i;
        ASSERT_EQ(0, file->Entries[i].height) << i;
    }
    ASSERT_EQ(file->Entries[0].width, file->Entries[3].width);
    ASSERT_EQ(-2, file->Entries[3].xOffset);
    ASSERT_EQ(5, file->Entries[3].yOffset);
}

TEST_F(SpriteBuildSlotTests, SlotEqualToNextIndexAddsNoPadding)
{
    ASSERT_EQ(ExitCode::ok, Build(R"([ { "slot": 0, "path": "logo.png" }, { "slot": 1, "path": "logo.png" } ])"));
    auto file = SpriteFile::Open(_output.c_str());
    ASSERT_TRUE(file.has_value());
    ASSERT_EQ(2u, file->Header.numEntries);
    ASSERT_GT(file->Entries[1].width, 0);
}

TEST_F(SpriteBuildSlotTests, SlotWorksForPalettes)
{
    ASSERT_EQ(ExitCode::ok, Build(R"([ { "slot": 2, "index": 10, "colours": [ "#102030", "#405060" ] } ])"));
    auto file = SpriteFile::Open(_output.c_str());
    ASSERT_TRUE(file.has_value());
    ASSERT_EQ(3u, file->Header.numEntries);
    ASSERT_TRUE(file->Entries[2].flags.has(G1Flag::isPalette));
}

TEST_F(SpriteBuildSlotTests, DecreasingSlotFails)
{
    ASSERT_EQ(ExitCode::fail, Build(R"([ { "slot": 2, "path": "logo.png" }, { "slot": 1, "path": "logo.png" } ])"));
}

TEST_F(SpriteBuildSlotTests, ReusedSlotFails)
{
    ASSERT_EQ(ExitCode::fail, Build(R"([ { "path": "logo.png" }, { "slot": 0, "path": "logo.png" } ])"));
}

TEST_F(SpriteBuildSlotTests, InvalidSlotFails)
{
    ASSERT_EQ(ExitCode::fail, Build(R"([ { "slot": -1, "path": "logo.png" } ])"));
    ASSERT_EQ(ExitCode::fail, Build(R"([ { "slot": "3", "path": "logo.png" } ])"));
    ASSERT_EQ(ExitCode::fail, Build(R"([ { "slot": 1.5, "path": "logo.png" } ])"));
}
