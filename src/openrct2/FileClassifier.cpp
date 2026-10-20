/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "FileClassifier.h"

#include "Diagnostic.h"
#include "core/Console.hpp"
#include "core/FileStream.h"
#include "core/Path.hpp"
#include "core/String.hpp"
#include "park/ParkFile.h"
#include "rct2/RCT2.h"
#include "sawyer_coding/SawyerChunkReader.h"
#include "sawyer_coding/SawyerCoding.h"

using namespace OpenRCT2;
using namespace OpenRCT2::SawyerCoding;

static bool TryClassifyAsPark(IStream* stream, ClassifiedFileInfo* result);

bool TryClassifyFile(const std::string& path, ClassifiedFileInfo* result)
{
    try
    {
        auto fs = FileStream(path, FileMode::open);
        return TryClassifyFile(&fs, result);
    }
    catch (const std::exception&)
    {
        return false;
    }
}

bool TryClassifyFile(IStream* stream, ClassifiedFileInfo* result)
{
    // Park detection
    if (TryClassifyAsPark(stream, result))
    {
        return true;
    }

    return false;
}

static bool TryClassifyAsPark(IStream* stream, ClassifiedFileInfo* result)
{
    bool success = false;
    uint64_t originalPosition = stream->GetPosition();
    try
    {
        auto magic = stream->ReadValue<uint32_t>();
        if (magic == kParkFileMagic || magic == kGolfFileMagic)
        {
            result->Type = FileType::park;
            result->Version = 0;
            success = true;
        }
    }
    catch (const std::exception& e)
    {
        success = false;
        LOG_VERBOSE(e.what());
    }
    stream->SetPosition(originalPosition);
    return success;
}

FileExtension GetFileExtensionType(u8string_view path)
{
    auto extension = Path::GetExtension(path);
    if (String::iequals(extension, ".dat") || String::iequals(extension, ".pob"))
        return FileExtension::dat;
    if (String::iequals(extension, ".park") || String::iequals(extension, ".golf"))
        return FileExtension::park;
    return FileExtension::unknown;
}
