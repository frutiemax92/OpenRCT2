/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

// AST-005: sound effects come from the game's own audio objects, not from RCT or OpenRCT2.

#include "TestData.h"

#include <filesystem>
#include <gtest/gtest.h>
#include <openrct2/Context.h>
#include <openrct2/OpenRCT2.h>
#include <openrct2/audio/Audio.h>
#include <openrct2/object/Object.h>
#include <openrct2/object/ObjectFactory.h>
#include <openrct2/object/ObjectManager.h>
#include <openrct2/object/ObjectRepository.h>

using namespace OpenRCT2;

static std::filesystem::path GameAudioDirectory()
{
    // build/testdata links to test/tests/testdata
    return std::filesystem::canonical(std::filesystem::u8path(TestData::GetBasePath())) / ".." / ".." / ".." / "data"
        / "object" / "golf" / "audio";
}

class GameAudio : public testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        gOpenRCT2Headless = true;
        gOpenRCT2NoGraphics = true;
        _context = CreateContext();
        ASSERT_TRUE(_context->Initialise());
    }

    static void TearDownTestSuite()
    {
        _context = nullptr;
    }

    static std::unique_ptr<IContext> _context;
};

std::unique_ptr<IContext> GameAudio::_context;

static void ExpectAudioObject(std::string_view identifier)
{
    auto path = GameAudioDirectory() / (std::string(identifier) + ".parkobj");
    ASSERT_TRUE(std::filesystem::exists(path)) << path;

    auto object = ObjectFactory::CreateObjectFromZipFile(path.string(), false);
    ASSERT_NE(nullptr, object) << path;
    ASSERT_EQ(ObjectType::audio, object->GetObjectType());
    ASSERT_EQ(identifier, object->GetIdentifier());

    // Installed with the game's data and found by the object repository like any other object.
    auto& repository = GetContext()->GetObjectRepository();
    const auto* item = repository.FindObject(identifier);
    ASSERT_NE(nullptr, item) << identifier;
    ASSERT_EQ(ObjectType::audio, item->Type);

    auto& manager = GetContext()->GetObjectManager();
    auto* loaded = manager.LoadObject(identifier);
    ASSERT_NE(nullptr, loaded) << identifier;
    manager.UnloadObjects({ loaded->GetDescriptor() });
}

TEST_F(GameAudio, BaseObjectIsShipped)
{
    ExpectAudioObject(Audio::AudioObjectIdentifiers::kGameBase);
}

TEST_F(GameAudio, AdditionalObjectIsShipped)
{
    ExpectAudioObject(Audio::AudioObjectIdentifiers::kGameAdditional);
}

TEST_F(GameAudio, IdentifiersAreNotRctOrOpenRCT2)
{
    for (auto id : { Audio::AudioObjectIdentifiers::kGameBase, Audio::AudioObjectIdentifiers::kGameAdditional })
    {
        ASSERT_EQ(0u, id.find("golf.")) << id;
    }
}
