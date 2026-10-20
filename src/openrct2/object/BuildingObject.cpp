/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "BuildingObject.h"

#include "../Context.h"
#include "../Diagnostic.h"
#include "../core/Guard.hpp"
#include "../core/Json.hpp"
#include "../drawing/Drawing.h"
#include "../interface/ScreenCoords.hpp"
#include "../world/Location.hpp"
#include "ObjectManager.h"

#include <algorithm>

namespace OpenRCT2
{
    static BuildingKind parseKind(std::string_view kind)
    {
        if (kind == "golf_club")
            return BuildingKind::golfClub;
        if (kind == "cottage")
            return BuildingKind::cottage;
        if (kind == "food_truck")
            return BuildingKind::foodTruck;
        if (kind == "driving_range_kiosk")
            return BuildingKind::drivingRangeKiosk;
        if (kind == "cart_rental")
            return BuildingKind::cartRental;
        if (kind == "restroom")
            return BuildingKind::restroom;
        return BuildingKind::generic;
    }

    static BuildingServices parseServices(const json_t& services)
    {
        BuildingServices result{};
        if (!services.is_array())
            return result;
        for (const auto& entry : services)
        {
            if (!entry.is_string())
                continue;
            const auto name = entry.get<std::string>();
            if (name == "food")
                result.set(BuildingService::food);
            else if (name == "drink")
                result.set(BuildingService::drink);
            else if (name == "toilet")
                result.set(BuildingService::toilet);
            else if (name == "pro_shop")
                result.set(BuildingService::proShop);
            else if (name == "rest")
                result.set(BuildingService::rest);
        }
        return result;
    }

    void BuildingObject::ReadJson(IReadObjectContext* context, json_t& root)
    {
        Guard::Assert(root.is_object(), "BuildingObject::ReadJson expects parameter root to be object");

        auto& properties = root["properties"];
        if (properties.is_object())
        {
            Kind = parseKind(Json::GetString(properties["kind"]));
            auto& footprint = properties["footprint"];
            if (footprint.is_array() && footprint.size() >= 2)
            {
                FootprintX = std::clamp(Json::GetNumber<int32_t>(footprint[0], 1), 1, kBuildingMaxFootprint);
                FootprintY = std::clamp(Json::GetNumber<int32_t>(footprint[1], 1), 1, kBuildingMaxFootprint);
            }
            Height = std::max(Json::GetNumber<int32_t>(properties["height"], 32), 1);
            AllowSloped = Json::GetBoolean(properties["allowSloped"], false);

            auto& entrance = properties["entrance"];
            if (entrance.is_object())
            {
                auto& tile = entrance["tile"];
                if (tile.is_array() && tile.size() >= 2)
                {
                    EntranceTileX = std::clamp(Json::GetNumber<int32_t>(tile[0], 0), 0, FootprintX - 1);
                    EntranceTileY = std::clamp(Json::GetNumber<int32_t>(tile[1], 0), 0, FootprintY - 1);
                }
                EntranceEdge = static_cast<uint8_t>(Json::GetNumber<int32_t>(entrance["edge"], 0) & 3);
                HasEntrance = true;
            }

            BuildCost = Json::GetNumber<money64>(properties["buildCost"]);
            RemoveCost = Json::GetNumber<money64>(properties["removeCost"]);
            UpkeepPerMonth = Json::GetNumber<money64>(properties["upkeepPerMonth"]);
            Services = parseServices(properties["services"]);
            Capacity = std::max(Json::GetNumber<int32_t>(properties["capacity"], 0), 0);
            if (properties["sounds"].is_object())
                AmbientSound = Json::GetString(properties["sounds"]["ambient"]);

            // The default collision box is the footprint times the height (BLD-002).
            Collision = readObjectCollision(
                properties, Height, FootprintX * kCoordsXYStep, FootprintY * kCoordsXYStep);
        }

        PopulateTablesFromJson(context, root);
    }

    void BuildingObject::Load()
    {
        GetStringTable().Sort();
        NameStringId = LanguageAllocateObjectString(GetName());
        IconImageId = LoadImages();
        FirstViewImageId = IconImageId + 1;

        // A building needs its four views; otherwise it would draw other objects' sprites.
        if (GetImageTable().GetCount() < static_cast<uint32_t>(1 + kBuildingViewCount))
        {
            LOG_WARNING("Building '%s' has fewer than %d view images", GetIdentifier().data(), kBuildingViewCount);
        }
    }

    void BuildingObject::Unload()
    {
        LanguageFreeObjectString(NameStringId);
        UnloadImages();

        NameStringId = 0;
        IconImageId = 0;
        FirstViewImageId = 0;
    }

    void BuildingObject::DrawPreview(Drawing::RenderTarget& rt, int32_t width, int32_t height) const
    {
        GfxDrawSprite(rt, ImageId(IconImageId), ScreenCoordsXY{ width / 2 - 16, height / 2 - 16 });
    }

    BuildingObject* BuildingObject::GetById(ObjectEntryIndex entryIndex)
    {
        auto& objMgr = GetContext()->GetObjectManager();
        return objMgr.GetLoadedObject<BuildingObject>(entryIndex);
    }
} // namespace OpenRCT2
