/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "AnimalObject.h"

#include "../Context.h"
#include "../core/Guard.hpp"
#include "../core/Json.hpp"
#include "../Diagnostic.h"
#include "../Diagnostic.h"
#include "../drawing/Drawing.h"
#include "../drawing/ModelSpriteStore.h"
#include "../model/GlbModel.h"
#include "../drawing/ModelSpriteStore.h"
#include "../model/GlbModel.h"
#include "ObjectCollision.h"
#include "ObjectManager.h"

#include <algorithm>

namespace OpenRCT2
{
    void AnimalObject::ReadJson(IReadObjectContext* context, json_t& root)
    {
        Guard::Assert(root.is_object(), "AnimalObject::ReadJson expects parameter root to be object");

        auto& properties = root["properties"];
        if (properties.is_object())
        {
            ModelPath = Json::GetString(properties["model"]);
            ModelData.clear();
            if (!ModelPath.empty())
                ModelData = context->GetData(ModelPath);
            TriangleBudget = Json::GetNumber<int32_t>(properties["triangleBudget"], 600);
            MinGroupSize = Json::GetNumber<int32_t>(properties["minGroupSize"], 2);
            MaxGroupSize = Json::GetNumber<int32_t>(properties["maxGroupSize"], 6);
            SpawnRate = Json::GetNumber<int32_t>(properties["spawnRate"], 10);
            SoundEffect = Json::GetString(properties["sound"]);

            Clips.clear();
            if (properties.contains("clips") && properties["clips"].is_array())
            {
                for (const auto& item : properties["clips"])
                {
                    if (item.is_string())
                    {
                        Clips.push_back(item.get<std::string>());
                    }
                }
            }
            else
            {
                for (auto clip : kRequiredAnimalClips)
                {
                    Clips.emplace_back(clip);
                }
            }

            AllowedSurfaces.clear();
            if (properties.contains("habitat") && properties["habitat"].is_array())
            {
                for (const auto& item : properties["habitat"])
                {
                    if (item.is_string())
                    {
                        AllowedSurfaces.push_back(item.get<std::string>());
                    }
                }
            }

            // Ball obstacle collision (small cylinder)
            Collision = readObjectCollision(properties, 12, 12, 12);
            if (!properties.contains("collision") || !properties["collision"].is_object() || !properties["collision"].contains("shape"))
            {
                Collision.shape = CollisionShape::cylinder;
                Collision.radius = 6;
            }
        }
    }

    void AnimalObject::Load()
    {
        if (ModelData.empty())
            return;
        // R3D-003: animals stay within their triangle budget (twice over is rejected).
        auto result = Model::loadGlb(ModelData, static_cast<size_t>(TriangleBudget), 128);
        if (!result.ok)
        {
            LOG_ERROR("Animal model '%s' rejected: %s", std::string(GetIdentifier()).c_str(), result.error.c_str());
            return;
        }
        ModelId = Drawing::ModelSpriteStore::Instance().RegisterModel(std::make_shared<Model::GlbModel>(std::move(result.model)));
    }

    void AnimalObject::Unload()
    {
        if (ModelId != 0)
        {
            Drawing::ModelSpriteStore::Instance().UnregisterModel(ModelId);
            ModelId = 0;
        }
    }

    void AnimalObject::DrawPreview(Drawing::RenderTarget& rt, int32_t width, int32_t height) const
    {
    }

    bool AnimalObject::HasClip(std::string_view name) const
    {
        return std::find(Clips.begin(), Clips.end(), name) != Clips.end();
    }

    bool AnimalObject::ValidateRequiredClips(std::vector<std::string>* missing) const
    {
        bool allFound = true;
        for (auto required : kRequiredAnimalClips)
        {
            if (!HasClip(required))
            {
                allFound = false;
                if (missing != nullptr)
                {
                    missing->emplace_back(required);
                }
            }
        }
        return allFound;
    }

    AnimalObject* AnimalObject::GetById(ObjectEntryIndex entryIndex)
    {
        auto& objMgr = GetContext()->GetObjectManager();
        return objMgr.GetLoadedObject<AnimalObject>(entryIndex);
    }
} // namespace OpenRCT2

