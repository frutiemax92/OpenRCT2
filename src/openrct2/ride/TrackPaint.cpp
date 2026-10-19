/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "TrackPaint.h"

#include "../Diagnostic.h"
#include "../SpriteIds.h"
#include "../drawing/LightFX.h"
#include "../interface/Viewport.h"
#include "../paint/Paint.h"
#include "../paint/tile_element/Paint.TileElement.h"
#include "../world/tile_element/TrackElement.h"
#include "Ride.h"
#include "RideData.h"
#include "TrackData.h"
#include "ted/TrackElementDescriptor.h"

using namespace OpenRCT2;
using namespace OpenRCT2::Drawing;
using namespace OpenRCT2::TrackMetadata;

void PaintTrack(PaintSession& session, Direction direction, int32_t height, const TrackElement& trackElement)
{
    RideId rideIndex = trackElement.getRideIndex();
    auto ride = GetRide(rideIndex);
    if (ride == nullptr)
    {
        LOG_ERROR("Attempted to paint invalid ride: %d", rideIndex);
        return;
    }

    if (!session.ViewFlags.has(ViewportFlag::highlightPathIssues))
    {
        auto trackType = trackElement.getTrackType();
        int32_t trackSequence = trackElement.getSequenceIndex();

        if (PaintShouldShowHeightMarkers(session, ViewportFlag::trackHeights))
        {
            session.InteractionType = ViewportInteractionItem::none;
            const auto& ted = GetTrackElementDescriptor(trackType);
            if ((trackType == TrackElemType::maze)
                || (trackSequence < ted.sequenceData.numSequences
                    && ted.sequenceData.sequences[trackSequence].flags.has(SequenceFlag::hasHeightMarker)))
            {
                uint16_t ax = ride->getRideTypeDescriptor().Heights.VehicleZOffset;
                // 0x1689 represents 0 height there are -127 to 128 heights above and below it
                // There are 3 arrays of 256 heights (units, m, ft) chosen with the GetHeightMarkerOffset()
                auto heightNum = (height + 8) / 16 - kMapBaseZ;
                auto imageId = ImageId(
                    SPR_HEIGHT_MARKER_BASE + GetHeightMarkerOffset() + heightNum, OpenRCT2::Drawing::Colour::lightBlue);

                PaintAddImageAsParent(session, imageId, { 16, 16, height + ax + 3 }, { { 1000, 1000, 2047 }, { 1, 1, 0 } });
            }
        }

        if (LightFx::IsAvailable())
        {
            uint8_t zOffset = 16;
            const auto& rtd = ride->getRideTypeDescriptor();
            if (rtd.specialType == RtdSpecialType::toilet || rtd.specialType == RtdSpecialType::firstAid
                || rtd.specialType == RtdSpecialType::cashMachine)
                zOffset = 23;

            const auto* originElement = ride->getOriginElement(StationIndex::FromUnderlying(0));
            if (originElement != nullptr && originElement->getTrackType() == TrackElemType::flatTrack1x1B)
                LightFx::AddKioskLights(session.MapPosition, height, zOffset);
            else if (kRideTypeDescriptors[ride->type].flags.has(RtdFlag::isShopOrFacility))
                LightFx::AddShopLights(session.MapPosition, trackElement.getDirection(), height, zOffset);
        }

        // The ride track pieces themselves are no longer drawn (ARC-002: track paint removed).
        session.InteractionType = trackElement.isGhost() ? ViewportInteractionItem::none : ViewportInteractionItem::ride;
    }
}
