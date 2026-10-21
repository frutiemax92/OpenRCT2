/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Paint.HoleMarkers.h"

#include "../../drawing/HoleMarkerSprites.h"
#include "../../golf/HoleMarkers.h"
#include "../../golf/MapBallWorld.h"
#include "../../profiling/Profiling.h"
#include "../Paint.h"

namespace OpenRCT2
{
    void PaintHoleMarkers(PaintSession& session)
    {
        PROFILED_FUNCTION();

        // Markers are a few pixels: not worth drawing zoomed far out.
        if (session.rt.zoom_level > ZoomLevel{ 1 })
            return;
        const auto& index = Golf::GetHoleMarkers();
        const auto* markers = index.at(TileCoordsXY{ session.MapPosition });
        if (markers == nullptr)
            return;

        const auto& sprites = Drawing::HoleMarkerSprites::Get();
        Golf::MapBallWorld world;
        auto draw = [&](ImageIndex image, int32_t x, int32_t y, int32_t z) {
            if (image == kImageIndexUndefined)
                return;
            const CoordsXYZ offset{ x - session.MapPosition.x, y - session.MapPosition.y, z };
            PaintAddImageAsParent(session, ImageId(image), offset, { offset, { 1, 1, 0 } });
        };

        for (const auto& marker : *markers)
        {
            const auto ground = world.surfaceAt(Fixed::fromInt(marker.x), Fixed::fromInt(marker.y));
            if (ground.water || ground.outOfBounds)
                continue;
            const int32_t z = static_cast<int32_t>(ground.height.floor());
            switch (marker.kind)
            {
                case Golf::HoleMarker::Kind::tee:
                    // The tee box is marked by a pair of posts to either side of the point.
                    draw(sprites.tee[std::min<size_t>(marker.index, sprites.tee.size() - 1)], marker.x - 5, marker.y + 5, z);
                    draw(sprites.tee[std::min<size_t>(marker.index, sprites.tee.size() - 1)], marker.x + 5, marker.y - 5, z);
                    break;
                case Golf::HoleMarker::Kind::cup:
                    draw(sprites.cup, marker.x, marker.y, z);
                    draw(sprites.pin, marker.x, marker.y, z);
                    break;
                case Golf::HoleMarker::Kind::aim:
                    if (index.showAimPoints())
                        draw(sprites.aim, marker.x, marker.y, z);
                    break;
            }
        }
    }
} // namespace OpenRCT2
