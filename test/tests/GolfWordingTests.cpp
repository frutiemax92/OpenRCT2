/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

// REN-052: no English text the player can see may contain the word "park". Words such as "parking" or "parked" are
// golf features and don't match.

#include "TestData.h"

#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <regex>
#include <string>

static std::filesystem::path RepositoryLanguageDirectory()
{
    // build/testdata links to test/tests/testdata
    return std::filesystem::canonical(std::filesystem::u8path(TestData::GetBasePath())) / ".." / ".." / ".." / "data"
        / "language";
}

static void ExpectNoParkWording(const char* fileName)
{
    std::ifstream file(RepositoryLanguageDirectory() / fileName);
    ASSERT_TRUE(file.is_open()) << fileName;

    const std::regex word(R"(\bpark(s|'s|’s)?\b)", std::regex::icase);
    const std::regex entry(R"(^(STR_\d+)\s*:(.*)$)");
    std::string line;
    int offenders = 0;
    while (std::getline(file, line))
    {
        std::smatch match;
        if (std::regex_match(line, match, entry) && std::regex_search(match[2].str(), word))
        {
            ADD_FAILURE() << fileName << " " << match[1] << ": " << match[2];
            offenders++;
        }
    }
    EXPECT_EQ(offenders, 0);
}

TEST(GolfWording, BritishEnglishHasNoParkWording)
{
    ExpectNoParkWording("en-GB.txt");
}

TEST(GolfWording, AmericanEnglishHasNoParkWording)
{
    ExpectNoParkWording("en-US.txt");
}
