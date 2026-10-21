/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "Object.h"
#include "ObjectCollision.h"

#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace OpenRCT2
{
    // GLF-030: required animation clips for golfer_model
    inline constexpr std::array<std::string_view, 10> kRequiredGolferClips = {
        "idle",
        "walk",
        "sit_bench",
        "talk",
        "swing_full",
        "swing_putt",
        "cheer",
        "groan",
        "celebrate",
        "rage_quit"
    };

    // Golfer model object (GLF-030, R3D-001, R3D-003, R3D-007)
    class GolferModelObject final : public Object
    {
    public:
        static constexpr ObjectType kObjectType = ObjectType::golferModel;

        StringId NameStringId{};
        uint32_t IconImageId{};

        std::string ModelPath;
        int32_t TriangleBudget = 800; // R3D-003: golfer <= 800
        int32_t MaterialSlots = 3;    // R3D-007: up to 3 remappable material slots
        std::vector<std::string> Clips;
        ObjectCollision Collision{};

        void ReadJson(IReadObjectContext* context, json_t& root) override;
        void Load() override;
        void Unload() override;

        void DrawPreview(Drawing::RenderTarget& rt, int32_t width, int32_t height) const override;

        bool HasClip(std::string_view name) const;
        bool ValidateRequiredClips(std::vector<std::string>* missing = nullptr) const;

        static GolferModelObject* GetById(ObjectEntryIndex entryIndex);
    };
} // namespace OpenRCT2

