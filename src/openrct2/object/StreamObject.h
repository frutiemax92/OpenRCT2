/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../core/Money.hpp"
#include "../drawing/TerrainSpriteGenerator.h"
#include "Object.h"

#include <string>
#include <vector>

namespace OpenRCT2
{
    constexpr int32_t kStreamDefaultWidth = 12;
    constexpr int32_t kStreamMaxWidth = 32;
    constexpr int32_t kStreamMaxFrames = 64;
    constexpr int32_t kStreamDefaultFrameDurationTicks = 8;

    // A water stream laid on top of a surface (STR-004). The images are the icon followed by the square, tileable
    // frames of the flow animation.
    class StreamObject final : public Object
    {
    public:
        static constexpr ObjectType kObjectType = ObjectType::stream;

        StringId NameStringId{};
        uint32_t IconImageId{};
        uint32_t FirstFrameImageId{};

        money64 BuildCost{};
        money64 RemoveCost{};
        int32_t Width = kStreamDefaultWidth; // in xy units
        int32_t FrameCount = 1;
        int32_t FrameDurationTicks = kStreamDefaultFrameDurationTicks;
        bool IsHazard = true;
        std::string Sound; // identifier of the ambient loop (§21), empty for none

        std::vector<Drawing::TerrainTexture> Frames;
        const Drawing::TerrainTexture* getFrame(int32_t index) const
        {
            if (Frames.empty())
                return nullptr;
            return &Frames[static_cast<size_t>(index) % Frames.size()];
        }

        void ReadJson(IReadObjectContext* context, json_t& root) override;
        void Load() override;
        void Unload() override;

        void DrawPreview(Drawing::RenderTarget& rt, int32_t width, int32_t height) const override;

        static StreamObject* GetById(ObjectEntryIndex entryIndex);
    };
} // namespace OpenRCT2
