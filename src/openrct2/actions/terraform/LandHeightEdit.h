/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../../world/Location.hpp"
#include "../GameAction.hpp"

enum class MapSelectType : uint8_t;

namespace OpenRCT2::GameActions
{
    // Raises or lowers the corners selected by `selectionType` in `range` by `stepQ` quarter steps, smoothing the
    // neighbouring tiles so none exceeds 4 Q (TER-015). Applies the change through LandSetHeightAction, nested.
    Result ChangeLandHeight(
        GameState_t& gameState, const MapRange& range, MapSelectType selectionType, int32_t stepQ, bool raise,
        bool isExecuting, CommandFlags flags, StringId errorTitle);
} // namespace OpenRCT2::GameActions
