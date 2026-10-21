/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "StreetObject.h"

#include "../Context.h"
#include "../Diagnostic.h"
#include "../OpenRCT2.h"
#include "../core/Guard.hpp"
#include "../core/Json.hpp"
#include "../drawing/Drawing.Sprite.h"
#include "../drawing/Drawing.h"
#include "../drawing/G1Element.h"
#include "../drawing/ImageImporter.h"
#include "../interface/ScreenCoords.hpp"
#include "ObjectManager.h"

#include <algorithm>

namespace OpenRCT2
{
    namespace
    {
        // The texture and the mask are raw images: palette indices, square, a power of two. Returns false for anything
        // else (RLE images, other shapes).
        bool readSquareTexture(uint32_t imageId, Drawing::TerrainTexture& into)
        {
            const auto* g1 = GfxGetG1Element(imageId);
            if (g1 == nullptr || g1->offset == nullptr || g1->flags.has(G1Flag::hasRLECompression) || g1->width != g1->height
                || g1->width <= 0 || (g1->width & (g1->width - 1)) != 0)
                return false;
            const auto size = static_cast<size_t>(g1->width);
            into = Drawing::TerrainTexture::Create(g1->width, std::vector<uint8_t>(g1->offset, g1->offset + size * size));
            return true;
        }
    } // namespace

    StreetMarking classifyStreetMarking(uint8_t red, uint8_t green, uint8_t blue)
    {
        constexpr int32_t kMaxDistanceSquared = 60 * 60;
        int32_t best = kMaxDistanceSquared;
        StreetMarking result = StreetMarking::none;
        for (int32_t i = 0; i < 3; i++)
        {
            const int32_t dr = red - kStreetMarkingColours[i][0];
            const int32_t dg = green - kStreetMarkingColours[i][1];
            const int32_t db = blue - kStreetMarkingColours[i][2];
            const int32_t distance = dr * dr + dg * dg + db * db;
            if (distance < best)
            {
                best = distance;
                result = static_cast<StreetMarking>(i + 1);
            }
        }
        return result;
    }

    StreetMarking StreetObject::markingAt(int32_t u, int32_t v) const
    {
        if (Markings.size == 0)
            return StreetMarking::none;
        return _markingOfIndex[Markings.Sample(0, u, v)];
    }

    void StreetObject::ReadJson(IReadObjectContext* context, json_t& root)
    {
        Guard::Assert(root.is_object(), "StreetObject::ReadJson expects parameter root to be object");

        auto& properties = root["properties"];
        if (properties.is_object())
        {
            BuildCost = Json::GetNumber<money64>(properties["buildCost"]);
            RemoveCost = Json::GetNumber<money64>(properties["removeCost"]);
            OneWay = Json::GetBoolean(properties["oneWay"], false);
            SpeedLimitKph = std::clamp(
                Json::GetNumber<int32_t>(properties["speedLimit"], kStreetDefaultSpeedLimitKph), 5, kStreetMaxSpeedLimitKph);
            Sound = Json::GetString(properties["sound"]);
        }

        PopulateTablesFromJson(context, root);
    }

    void StreetObject::Load()
    {
        GetStringTable().Sort();
        NameStringId = LanguageAllocateObjectString(GetName());
        IconImageId = LoadImages();
        TextureImageId = IconImageId + 1;
        MarkingsImageId = IconImageId + 2;

        Texture = {};
        Markings = {};
        if (gOpenRCT2NoGraphics)
            return;

        if (!readSquareTexture(TextureImageId, Texture))
            LOG_WARNING("Street '%s' has no valid road texture (the second image)", GetIdentifier().data());
        if (!readSquareTexture(MarkingsImageId, Markings))
            LOG_WARNING("Street '%s' has no valid markings mask (the third image)", GetIdentifier().data());
        // Index 0 is transparent and so no marking.
        for (int32_t i = 1; i < 256; i++)
            _markingOfIndex[i] = classifyStreetMarking(StandardPalette[i].red, StandardPalette[i].green, StandardPalette[i].blue);
    }

    void StreetObject::Unload()
    {
        Texture = {};
        Markings = {};
        LanguageFreeObjectString(NameStringId);
        UnloadImages();

        NameStringId = 0;
        IconImageId = 0;
        TextureImageId = 0;
        MarkingsImageId = 0;
    }

    void StreetObject::DrawPreview(Drawing::RenderTarget& rt, int32_t width, int32_t height) const
    {
        GfxDrawSprite(rt, ImageId(IconImageId), ScreenCoordsXY{ width / 2 - 16, height / 2 - 16 });
    }

    StreetObject* StreetObject::GetById(ObjectEntryIndex entryIndex)
    {
        auto& objMgr = GetContext()->GetObjectManager();
        return objMgr.GetLoadedObject<StreetObject>(entryIndex);
    }
} // namespace OpenRCT2
