/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../world/Location.hpp"

struct PaintSession;

namespace OpenRCT2
{
    struct TrackElement;
}

// Paints the parts of a track element that remain: height markers, kiosk/shop lights and the interaction target.
void PaintTrack(PaintSession& session, Direction direction, int32_t height, const OpenRCT2::TrackElement& trackElement);
