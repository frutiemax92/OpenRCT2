/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

// ARC-002: the RCT1 importers are removed; RCT1 saves, scenarios and track designs are rejected.

#include <gtest/gtest.h>
#include <openrct2/FileClassifier.h>
#include <openrct2/ParkImporter.h>
#include <openrct2/TrackImporter.h>
#include <stdexcept>

using namespace OpenRCT2;

TEST(UnsupportedFormats, RCT1ParksAreRejected)
{
    EXPECT_THROW(ParkImporter::Create("park.sv4"), std::runtime_error);
    EXPECT_THROW(ParkImporter::Create("scenario.SC4"), std::runtime_error);
    EXPECT_FALSE(ParkImporter::ExtensionIsScenario(".sc4"));
    EXPECT_TRUE(ParkImporter::ExtensionIsScenario(".sc6"));
}

TEST(UnsupportedFormats, RCT1TrackDesignsAreRejected)
{
    EXPECT_THROW(TrackImporter::Create("design.td4"), std::runtime_error);
}

TEST(UnsupportedFormats, RCT1ExtensionsAreUnknown)
{
    EXPECT_EQ(FileExtension::unknown, GetFileExtensionType("park.sv4"));
    EXPECT_EQ(FileExtension::unknown, GetFileExtensionType("scenario.sc4"));
    EXPECT_EQ(FileExtension::unknown, GetFileExtensionType("design.td4"));
    EXPECT_EQ(FileExtension::park, GetFileExtensionType("park.park"));
}
