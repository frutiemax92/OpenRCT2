/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

namespace OpenRCT2::Golf
{
    // The database of names golfers are given when they are created (GLF-003): a golfer rolls one first name and one family
    // name. A peep_names object can replace the lists (GolfDirector::firstNames and lastNames).
    const std::vector<std::string>& defaultFirstNames();
    const std::vector<std::string>& defaultFamilyNames();
} // namespace OpenRCT2::Golf
