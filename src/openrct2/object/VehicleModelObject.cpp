/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "VehicleModelObject.h"

#include "../Context.h"
#include "../Diagnostic.h"
#include "../core/Guard.hpp"
#include "../core/Json.hpp"
#include "../drawing/Drawing.h"
#include "../drawing/ModelSpriteStore.h"
#include "../model/GlbModel.h"
#include "../interface/ScreenCoords.hpp"
#include "../localisation/LanguagePack.h"
#include "ObjectLimits.h"
#include "ObjectList.h"
#include "ObjectManager.h"

#include <algorithm>

namespace OpenRCT2
{
    static Fixed fixedFromDouble(double v)
    {
        return Fixed::fromRaw(static_cast<int64_t>(std::lround(v * Fixed::kOne)));
    }

    static VehicleModelKind parseKind(std::string_view kind)
    {
        if (kind == "car")
            return VehicleModelKind::car;
        return VehicleModelKind::golfCart;
    }

    void VehicleModelObject::ReadJson(IReadObjectContext* context, json_t& root)
    {
        Guard::Assert(root.is_object(), "VehicleModelObject::ReadJson expects parameter root to be object");

        auto& properties = root["properties"];
        if (properties.is_object())
        {
            Kind = parseKind(Json::GetString(properties["kind"]));
            ModelPath = Json::GetString(properties["model"]);
            ModelData.clear();
            if (!ModelPath.empty())
                ModelData = context->GetData(ModelPath);

            int32_t defaultHeight = 14;
            int32_t defaultFootprintX = 20;
            int32_t defaultFootprintY = 12;

            if (Kind == VehicleModelKind::golfCart)
            {
                Seats = 2;
                MaxSpeed = Fixed::ratio(220, 100);
                Acceleration = Fixed::ratio(8, 100);
                ContactPoints = { 10.0f, 6.0f };
                EngineSound = "golf.cart.engine";
                HornSound = "golf.cart.horn";
                defaultHeight = 14;
                defaultFootprintX = 20;
                defaultFootprintY = 12;
            }
            else // VehicleModelKind::car
            {
                Seats = 4;
                MaxSpeed = Fixed::ratio(200, 100);
                Acceleration = Fixed::ratio(6, 100);
                ContactPoints = { 12.0f, 6.0f };
                EngineSound = "golf.car.engine";
                HornSound = "golf.car.horn";
                defaultHeight = 16;
                defaultFootprintX = 24;
                defaultFootprintY = 12;
            }

            Seats = std::max(Json::GetNumber<int32_t>(properties["seats"], Seats), 1);

            if (properties.contains("maxSpeed"))
            {
                MaxSpeed = fixedFromDouble(Json::GetNumber<double>(properties["maxSpeed"]));
            }
            else if (properties.contains("speed"))
            {
                MaxSpeed = fixedFromDouble(Json::GetNumber<double>(properties["speed"]));
            }

            if (properties.contains("acceleration"))
            {
                Acceleration = fixedFromDouble(Json::GetNumber<double>(properties["acceleration"]));
            }

            // Contact points: either an object with halfLength/halfWidth (or length/width) or an array [halfLength, halfWidth]
            const auto readContacts = [&](const json_t& cp) {
                if (cp.is_object())
                {
                    if (cp.contains("halfLength"))
                        ContactPoints.halfLength = Json::GetNumber<float>(cp["halfLength"], ContactPoints.halfLength);
                    else if (cp.contains("length"))
                        ContactPoints.halfLength = Json::GetNumber<float>(cp["length"], ContactPoints.halfLength * 2.0f) * 0.5f;

                    if (cp.contains("halfWidth"))
                        ContactPoints.halfWidth = Json::GetNumber<float>(cp["halfWidth"], ContactPoints.halfWidth);
                    else if (cp.contains("width"))
                        ContactPoints.halfWidth = Json::GetNumber<float>(cp["width"], ContactPoints.halfWidth * 2.0f) * 0.5f;
                }
                else if (cp.is_array() && cp.size() >= 2)
                {
                    ContactPoints.halfLength = Json::GetNumber<float>(cp[0], ContactPoints.halfLength);
                    ContactPoints.halfWidth = Json::GetNumber<float>(cp[1], ContactPoints.halfWidth);
                }
            };

            if (properties.contains("contactPoints"))
                readContacts(properties["contactPoints"]);
            else if (properties.contains("wheelContactPoints"))
                readContacts(properties["wheelContactPoints"]);
            else if (properties.contains("contact_points"))
                readContacts(properties["contact_points"]);

            // Sounds: engine and horn
            if (properties["sounds"].is_object())
            {
                EngineSound = Json::GetString(properties["sounds"]["engine"], EngineSound);
                HornSound = Json::GetString(properties["sounds"]["horn"], HornSound);
            }
            if (properties.contains("engineSound"))
                EngineSound = Json::GetString(properties["engineSound"]);
            if (properties.contains("hornSound"))
                HornSound = Json::GetString(properties["hornSound"]);

            Collision = readObjectCollision(properties, defaultHeight, defaultFootprintX, defaultFootprintY);
        }

        PopulateTablesFromJson(context, root);
    }

    void VehicleModelObject::Load()
    {
        GetStringTable().Sort();
        NameStringId = LanguageAllocateObjectString(GetName());
        if (GetImageTable().GetCount() > 0)
        {
            IconImageId = LoadImages();
        }
        if (ModelData.empty())
            return;
        auto result = Model::loadGlb(ModelData, 0, 128);
        if (!result.ok)
        {
            LOG_ERROR("Vehicle model '%s' rejected: %s", std::string(GetIdentifier()).c_str(), result.error.c_str());
            return;
        }
        ModelId = Drawing::ModelSpriteStore::Instance().RegisterModel(std::make_shared<Model::GlbModel>(std::move(result.model)));
        (Kind == VehicleModelKind::car ? ActiveCarModelId : ActiveCartModelId) = ModelId;
    }

    void VehicleModelObject::Unload()
    {
        if (ModelId != 0)
        {
            Drawing::ModelSpriteStore::Instance().UnregisterModel(ModelId);
            auto& active = Kind == VehicleModelKind::car ? ActiveCarModelId : ActiveCartModelId;
            if (active == ModelId)
                active = 0;
            ModelId = 0;
        }
        LanguageFreeObjectString(NameStringId);
        if (IconImageId != 0)
        {
            UnloadImages();
        }

        NameStringId = 0;
        IconImageId = 0;
    }

    void VehicleModelObject::DrawPreview(Drawing::RenderTarget& rt, int32_t width, int32_t height) const
    {
        if (IconImageId != 0)
        {
            GfxDrawSprite(rt, ImageId(IconImageId), ScreenCoordsXY{ width / 2 - 16, height / 2 - 16 });
        }
    }

    VehicleModelObject* VehicleModelObject::GetById(ObjectEntryIndex entryIndex)
    {
        auto& objMgr = GetContext()->GetObjectManager();
        return objMgr.GetLoadedObject<VehicleModelObject>(entryIndex);
    }

    VehicleModelObject* VehicleModelObject::FindFirstByKind(VehicleModelKind kind)
    {
        auto* context = GetContext();
        if (context == nullptr)
            return nullptr;
        auto& objMgr = context->GetObjectManager();
        size_t count = getObjectEntryGroupCount(ObjectType::vehicleModel);
        for (size_t i = 0; i < count; ++i)
        {
            auto* obj = objMgr.GetLoadedObject<VehicleModelObject>(i);
            if (obj != nullptr && obj->Kind == kind)
            {
                return obj;
            }
        }
        return nullptr;
    }
} // namespace OpenRCT2
