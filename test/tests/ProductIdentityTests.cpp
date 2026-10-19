/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

// ARC-001c: the game has its own user-visible identity and never shares user folders with OpenRCT2.

#include <gtest/gtest.h>
#include <openrct2/PlatformEnvironment.h>
#include <openrct2/Version.h>
#include <openrct2/core/Path.hpp>
#include <openrct2/core/String.hpp>
#include <string>

using namespace OpenRCT2;

TEST(ProductIdentity, NameIsTheGame)
{
    ASSERT_STREQ("Open Golf Simulator", OPENRCT2_NAME);
}

TEST(ProductIdentity, FullVersionNamesTheGameAndTheEngine)
{
    std::string full = gVersionInfoFull;
    ASSERT_TRUE(String::startsWith(full, OPENRCT2_NAME ", v")) << full;
    ASSERT_NE(full.find("(OpenRCT2 engine v" kOpenRCT2Version ")"), std::string::npos) << full;
}

TEST(ProductIdentity, VersionTagIsNotAnOpenRCT2Tag)
{
    std::string tag = gVersionInfoTag;
    ASSERT_FALSE(tag.empty());
    ASSERT_EQ('v', tag[0]) << tag;
    // Without a golf-v* git tag, the tag falls back to the game version.
    ASSERT_FALSE(String::startsWith(tag, "golf-")) << tag;
    ASSERT_FALSE(String::startsWith(tag, "v" kOpenRCT2Version)) << tag;
}

TEST(ProductIdentity, UserDirectoryNameDiffersFromOpenRCT2)
{
    auto name = GetUserDirectoryName();
#if defined(__ANDROID__)
    ASSERT_EQ("open-golf-simulator-user", name);
#else
    ASSERT_EQ("open-golf-simulator", name);
#endif
    ASSERT_FALSE(String::iequals(name, "OpenRCT2"));
    ASSERT_FALSE(String::iequals(name, "openrct2-user"));
}

TEST(ProductIdentity, DefaultUserFoldersUseTheGameDirectory)
{
    auto env = CreatePlatformEnvironment();
    auto name = GetUserDirectoryName();
    for (auto base : { DirBase::user, DirBase::config, DirBase::cache })
    {
        auto path = env->GetDirectoryPath(base);
        ASSERT_EQ(name, Path::GetFileName(path)) << path;
    }
}
