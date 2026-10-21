/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "GolferModelObject.h"

#include "../Context.h"
#include "../core/Guard.hpp"
#include "../core/Json.hpp"
#include "../drawing/Drawing.h"
#include "ObjectCollision.h"
#include "ObjectManager.h"

#include <algorithm>

namespace OpenRCT2
{
    void GolferModelObject::ReadJson(IReadObjectContext* context, json_t& root)
    {
        Guard::Assert(root.is_object(), "GolferModelObject::ReadJson expects parameter root to be object");

        auto& properties = root["properties"];
        if (properties.is_object())
        {
            ModelPath = Json::GetString(properties["model"]);
            TriangleBudget = Json::GetNumber<int32_t>(properties["triangleBudget"], 800);
            MaterialSlots = Json::GetNumber<int32_t>(properties["materialSlots"], 3);

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
                // Default to standard required clips
                for (auto clip : kRequiredGolferClips)
                {
                    Clips.emplace_back(clip);
                }
            }

            // Cylinder collision for golfer body
            Collision = readObjectCollision(properties, 32, 12, 12);
            if (!properties.contains("collision") || !properties["collision"].is_object() || !properties["collision"].contains("shape"))
            {
                Collision.shape = CollisionShape::cylinder;
                Collision.radius = 6;
            }
        }
    }

    void GolferModelObject::Load()
    {
    }

    void GolferModelObject::Unload()
    {
    }

    void GolferModelObject::DrawPreview(Drawing::RenderTarget& rt, int32_t width, int32_t height) const
    {
    }

    bool GolferModelObject::HasClip(std::string_view name) const
    {
        return std::find(Clips.begin(), Clips.end(), name) != Clips.end();
    }

    bool GolferModelObject::ValidateRequiredClips(std::vector<std::string>* missing) const
    {
        bool allFound = true;
        for (auto required : kRequiredGolferClips)
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

    GolferModelObject* GolferModelObject::GetById(ObjectEntryIndex entryIndex)
    {
        auto& objMgr = GetContext()->GetObjectManager();
        return objMgr.GetLoadedObject<GolferModelObject>(entryIndex);
    }
} // namespace OpenRCT2

