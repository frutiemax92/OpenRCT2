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

#include <array>
#include <string>

namespace OpenRCT2
{
    constexpr int32_t kStreetDefaultSpeedLimitKph = 40;
    constexpr int32_t kStreetMaxSpeedLimitKph = 120;

    // What the markings mask paints over the road texture at a pixel (STE-003, STE-004). The mask is an image like any
    // other, so a marking is told by its colour: white for the centre line, yellow for the edge line, light blue for the
    // arrow of a one-way road; a transparent pixel is no marking.
    enum class StreetMarking : uint8_t
    {
        none = 0,
        centreLine, // the dashed line between the two lanes
        edgeLine,   // the solid line along the verge
        arrow,      // the arrow of a one-way road
    };
    // The colours of the markings in the mask, as RGB.
    constexpr uint8_t kStreetMarkingColours[3][3] = { { 255, 255, 255 }, { 255, 220, 0 }, { 120, 200, 255 } };
    // Which marking a colour stands for: the closest of the three, or none when none is close.
    StreetMarking classifyStreetMarking(uint8_t red, uint8_t green, uint8_t blue);

    // A kind of street (STE-004): a two-way road, or a one-way road. The images are the icon, the top-down road texture
    // and the markings mask. Both textures show a straight road along the x axis; the renderer turns and cuts them for the
    // connections and the slope of each tile (STE-003).
    class StreetObject final : public Object
    {
    public:
        static constexpr ObjectType kObjectType = ObjectType::street;

        StringId NameStringId{};
        uint32_t IconImageId{};
        uint32_t TextureImageId{};
        uint32_t MarkingsImageId{};

        money64 BuildCost{};
        money64 RemoveCost{};
        bool OneWay = false;
        int32_t SpeedLimitKph = kStreetDefaultSpeedLimitKph;
        std::string Sound; // identifier of the traffic ambience (§21), empty for none

        Drawing::TerrainTexture Texture;  // the road surface
        Drawing::TerrainTexture Markings; // values of StreetMarking

        // The marking at a pixel of the (wrapping) mask.
        StreetMarking markingAt(int32_t u, int32_t v) const;

    private:
        std::array<StreetMarking, 256> _markingOfIndex{};

    public:

        void ReadJson(IReadObjectContext* context, json_t& root) override;
        void Load() override;
        void Unload() override;

        void DrawPreview(Drawing::RenderTarget& rt, int32_t width, int32_t height) const override;

        static StreetObject* GetById(ObjectEntryIndex entryIndex);
    };
} // namespace OpenRCT2
