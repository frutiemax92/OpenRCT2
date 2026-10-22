/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Paint.Building.h"

#include "../../SpriteIds.h"
#include "../../drawing/Drawing.h"
#include "../../interface/Viewport.h"
#include "../../object/BuildingObject.h"
#include "../../world/Building.h"
#include "../../world/tile_element/BuildingElement.h"
#include "../Paint.h"

using namespace OpenRCT2;
using namespace OpenRCT2::Drawing;

// A building is one image covering the whole footprint, anchored on the centre of its ground. The tile nearest to the
// camera paints it, so everything behind the building is painted first.
void PaintBuilding(PaintSession& session, int32_t height, const BuildingElement& element)
{
    const auto* object = element.getObject();
    if (object == nullptr)
        return;

    auto tile = session.MapPosition;
    auto origin = Building::GetOrigin(tile, element);
    auto rotation = element.getRotation();
    if (Building::GetPaintTile(*object, origin, rotation, session.CurrentRotation) != tile)
        return;

    auto sizeX = object->getFootprintX(rotation) * kCoordsXYStep;
    auto sizeY = object->getFootprintY(rotation) * kCoordsXYStep;
    CoordsXYZ offset{ origin.x + sizeX / 2 - tile.x, origin.y + sizeY / 2 - tile.y, height };
    BoundBoxXYZ boundBox{ { origin.x - tile.x + 1, origin.y - tile.y + 1, height }, { sizeX - 2, sizeY - 2, object->Height } };

    session.InteractionType = ViewportInteractionItem::building;

    auto view = (rotation + session.CurrentRotation) & 3;
    ImageId image(object->getViewImageId(view));
    if (element.isGhost())
        image = image.WithRemap(FilterPaletteID::paletteGhost);
    PaintAddImageAsParent(session, image, offset, boundBox);
}
