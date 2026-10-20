/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "StreamObject.h"

#include "../Context.h"
#include "../Diagnostic.h"
#include "../OpenRCT2.h"
#include "../core/Guard.hpp"
#include "../core/Json.hpp"
#include "../drawing/Drawing.Sprite.h"
#include "../drawing/Drawing.h"
#include "../drawing/G1Element.h"
#include "../interface/ScreenCoords.hpp"
#include "ObjectManager.h"

#include <algorithm>

namespace OpenRCT2
{
    void StreamObject::ReadJson(IReadObjectContext* context, json_t& root)
    {
        Guard::Assert(root.is_object(), "StreamObject::ReadJson expects parameter root to be object");

        auto& properties = root["properties"];
        if (properties.is_object())
        {
            BuildCost = Json::GetNumber<money64>(properties["buildCost"]);
            RemoveCost = Json::GetNumber<money64>(properties["removeCost"]);
            Width = std::clamp(Json::GetNumber<int32_t>(properties["width"], kStreamDefaultWidth), 1, kStreamMaxWidth);
            IsHazard = properties.contains("isHazard") ? Json::GetBoolean(properties["isHazard"]) : true;
            Sound = Json::GetString(properties["sound"]);

            auto& image = properties["image"];
            if (image.is_object())
            {
                FrameCount = std::clamp(Json::GetNumber<int32_t>(image["frames"], 1), 1, kStreamMaxFrames);
                FrameDurationTicks = std::max(
                    Json::GetNumber<int32_t>(image["frameDurationTicks"], kStreamDefaultFrameDurationTicks), 1);
            }
        }

        PopulateTablesFromJson(context, root);
    }

    void StreamObject::Load()
    {
        GetStringTable().Sort();
        NameStringId = LanguageAllocateObjectString(GetName());
        IconImageId = LoadImages();
        FirstFrameImageId = IconImageId + 1;

        Frames.clear();
        if (gOpenRCT2NoGraphics)
            return;

        // The frames are the images after the icon; a frame that is not a square power of two is skipped.
        const auto imageCount = static_cast<int32_t>(GetImageTable().GetCount());
        for (int32_t i = 0; i < std::min(FrameCount, imageCount - 1); i++)
        {
            const auto* g1 = GfxGetG1Element(FirstFrameImageId + static_cast<uint32_t>(i));
            if (g1 == nullptr || g1->offset == nullptr || g1->flags.has(G1Flag::hasRLECompression) || g1->width != g1->height
                || g1->width <= 0 || (g1->width & (g1->width - 1)) != 0)
            {
                LOG_WARNING("Stream '%s' frame %d is not a valid square texture", GetIdentifier().data(), i);
                continue;
            }
            const auto size = static_cast<size_t>(g1->width);
            Frames.push_back(
                Drawing::TerrainTexture::Create(g1->width, std::vector<uint8_t>(g1->offset, g1->offset + size * size)));
        }
    }

    void StreamObject::Unload()
    {
        Frames.clear();
        LanguageFreeObjectString(NameStringId);
        UnloadImages();

        NameStringId = 0;
        IconImageId = 0;
        FirstFrameImageId = 0;
    }

    void StreamObject::DrawPreview(Drawing::RenderTarget& rt, int32_t width, int32_t height) const
    {
        GfxDrawSprite(rt, ImageId(IconImageId), ScreenCoordsXY{ width / 2 - 16, height / 2 - 16 });
    }

    StreamObject* StreamObject::GetById(ObjectEntryIndex entryIndex)
    {
        auto& objMgr = GetContext()->GetObjectManager();
        return objMgr.GetLoadedObject<StreamObject>(entryIndex);
    }
} // namespace OpenRCT2
