/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Paint.GolfBall.h"

#include "../../config/Config.h"
#include "../../drawing/FilterPaletteIds.h"
#include "../../drawing/GolfBallSprites.h"
#include "../../entity/GolfBall.h"
#include "../../golf/MapBallWorld.h"
#include "../../profiling/Profiling.h"
#include "../Paint.h"

namespace OpenRCT2
{
    void PaintGolfBall(PaintSession& session, const GolfBall& ball, int32_t imageDirection)
    {
        PROFILED_FUNCTION();

        if (session.rt.zoom_level > ZoomLevel{ 1 })
            return;
        const auto& sprites = Drawing::GolfBallSprites::Get();
        if (sprites.ball == kImageIndexUndefined)
            return;

        // The shadow lies on the surface below the ball; the further the ball is above it the lighter it should look,
        // but a single darkening level is all the filter offers.
        Golf::MapBallWorld world;
        const auto ground = world.surfaceAt(ball.ball.position.x, ball.ball.position.y);
        const int32_t groundZ = static_cast<int32_t>(ground.height.floor());
        if (!ground.water && !ground.outOfBounds && sprites.shadow != kImageIndexUndefined)
        {
            PaintAddImageAsParent(
                session, ImageId(sprites.shadow).WithTransparency(Drawing::FilterPaletteID::paletteDarken1), { 0, 0, groundZ },
                { 1, 1, 0 });
        }

        if (Config::Get().general.golfBallTrail && sprites.trail != kImageIndexUndefined)
        {
            for (int32_t i = 0; i < ball.trailCount; i++)
            {
                const CoordsXYZ offset{ ball.trail[i].x - ball.x, ball.trail[i].y - ball.y, ball.trail[i].z };
                PaintAddImageAsParent(session, ImageId(sprites.trail), offset, { { offset.x, offset.y, offset.z }, { 1, 1, 0 } });
            }
        }

        PaintAddImageAsParent(session, ImageId(sprites.ball), { 0, 0, ball.z }, { 1, 1, 0 });
    }
} // namespace OpenRCT2
