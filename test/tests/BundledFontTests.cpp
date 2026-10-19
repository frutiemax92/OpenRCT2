/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

// AST-003: text is drawn with TrueType fonts shipped in data/fonts, under an open licence.

#ifndef DISABLE_TTF

    #include "TestData.h"

    #include <filesystem>
    #include <fstream>
    #include <gtest/gtest.h>
    #include <openrct2/core/EnumUtils.hpp>
    #include <openrct2/drawing/TTF.h>
    #include <openrct2/interface/Fonts.h>

using namespace OpenRCT2;

static std::filesystem::path RepositoryDataDirectory()
{
    // build/testdata links to test/tests/testdata
    return std::filesystem::canonical(std::filesystem::u8path(TestData::GetBasePath())) / ".." / ".." / ".." / "data";
}

TEST(BundledFonts, FindsFontsInTheDataFontsDirectory)
{
    auto dir = std::filesystem::temp_directory_path() / "openrct2-bundled-font-test";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir / "fonts");
    std::ofstream(dir / "fonts" / "Test.ttf") << "x";

    TTFFontDescriptor present{ "Test.ttf", "Test", 12, 0, 0, 12, 0, nullptr };
    TTFFontDescriptor missing{ "Missing.ttf", "Missing", 12, 0, 0, 12, 0, nullptr };
    TTFFontDescriptor noFile{ nullptr, "None", 12, 0, 0, 12, 0, nullptr };

    auto found = TTFFindBundledFont(dir.u8string(), present);
    ASSERT_FALSE(found.empty());
    ASSERT_TRUE(std::filesystem::equivalent(std::filesystem::u8path(found), dir / "fonts" / "Test.ttf"));
    ASSERT_TRUE(TTFFindBundledFont(dir.u8string(), missing).empty());
    ASSERT_TRUE(TTFFindBundledFont(dir.u8string(), noFile).empty());
    ASSERT_TRUE(TTFFindBundledFont("", present).empty());

    std::filesystem::remove_all(dir);
}

TEST(BundledFonts, GameFontSetIsShippedWithLicences)
{
    auto data = RepositoryDataDirectory();
    for (const auto& desc : TTFFontGame.size)
    {
        auto path = TTFFindBundledFont(data.u8string(), desc);
        ASSERT_FALSE(path.empty()) << desc.filename;

        auto stem = std::filesystem::u8path(desc.filename).stem().u8string();
        auto family = stem.substr(0, stem.find('-'));
        ASSERT_TRUE(std::filesystem::exists(data / "fonts" / (family + "-OFL.txt"))) << family;
    }
}

TEST(BundledFonts, GameFontsOpenAndCoverLatinText)
{
    auto data = RepositoryDataDirectory();
    ASSERT_EQ(0, TTF_Init());
    for (const auto& desc : TTFFontGame.size)
    {
        // Pixel fonts must be rendered without anti-aliasing to stay crisp.
        ASSERT_EQ(0, desc.hinting_threshold) << desc.filename;
        ASSERT_GT(desc.line_height, 0) << desc.filename;

        auto path = TTFFindBundledFont(data.u8string(), desc);
        auto* font = TTF_OpenFont(path.c_str(), desc.ptSize);
        ASSERT_NE(nullptr, font) << path;

        for (codepoint_t c = 0x20; c < 0x7F; c++)
        {
            ASSERT_TRUE(TTF_GlyphIsProvided(font, c)) << desc.filename << " U+" << std::hex << c;
        }
        // Accented letters of the Latin-script languages that used the sprite font.
        for (codepoint_t c : { 0xE9u, 0xE8u, 0xE7u, 0xFCu, 0xF6u, 0xE4u, 0xDFu, 0xF1u, 0x0142u, 0x0119u, 0x0161u })
        {
            ASSERT_TRUE(TTF_GlyphIsProvided(font, c)) << desc.filename << " U+" << std::hex << c;
        }

        int w = 0, h = 0;
        ASSERT_EQ(0, TTF_SizeUTF8(font, "Hole 7", &w, &h));
        ASSERT_GT(w, 0);
        ASSERT_GT(h, 0);
        TTF_CloseFont(font);
    }
    TTF_Quit();
}

#endif // DISABLE_TTF
