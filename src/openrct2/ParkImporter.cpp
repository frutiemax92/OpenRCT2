/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "ParkImporter.h"

#include "Context.h"
#include "core/Path.hpp"
#include "core/String.hpp"

#include <memory>

namespace OpenRCT2::ParkImporter
{
    std::unique_ptr<IParkImporter> Create(const std::string& hintPath)
    {
        std::string extension = Path::GetExtension(hintPath);
        if (!ExtensionIsOpenRCT2ParkFile(extension))
        {
            // RCT1 and RCT2 saves and scenarios are not supported (ARC-002).
            throw std::runtime_error("Only .park files are supported.");
        }
        return CreateParkFile(GetContext()->GetObjectRepository());
    }

    bool ExtensionIsOpenRCT2ParkFile(std::string_view extension)
    {
        return String::iequals(extension, ".park");
    }
} // namespace OpenRCT2::ParkImporter
