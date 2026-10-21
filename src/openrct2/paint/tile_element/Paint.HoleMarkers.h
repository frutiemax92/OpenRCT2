/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

struct PaintSession;

namespace OpenRCT2
{
    // Draws the tee markers, the flag pin with its cup and the aim points that lie on the tile being painted
    // (HOL-002, HOL-003). Called after the surface of the tile.
    void PaintHoleMarkers(PaintSession& session);
} // namespace OpenRCT2
