/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "GlbModel.h"

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace OpenRCT2::Model
{
    // Software rasteriser for 3D models (R3D-002). It draws a model with the game's isometric camera into an
    // off-screen RGBA + depth image, and quantises that to palette indices to match the 8-bit sprites.
    //
    // Model space is glTF (+Y up, +X right, +Z towards the viewer). It maps to map space as
    // (x, y, z) = (mx, -mz, my), then the 2:1 dimetric projection of the map is used:
    //   screenX = (x - y) * scale,  screenY = ((x + y) / 2 - z) * scale
    // The viewer is at +x +y +z, so a larger (x + y + z) is closer to the camera.

    struct Orientation
    {
        float yaw = 0;   // radians about the vertical axis (map z)
        float pitch = 0; // radians about map y, for terrain alignment (R3D-006)
        float roll = 0;  // radians about map x
    };

    struct RenderSettings
    {
        Orientation orientation;
        float unitsPerMetre = 10.0f;  // map units per model metre (a tile is 32 units)
        float pixelsPerUnit = 1.0f;   // zoom: 1 at the closest zoom level, 0.5 one level out, ...
        Vec3 lightDirection{ -0.4f, 0.3f, 0.85f }; // towards the light, in map space
        float ambient = 0.45f;
        // Per-material base colour override (R3D-007 remap slots); entries beyond the material count are ignored.
        std::vector<std::optional<std::array<float, 4>>> materialColours;
        int32_t clipIndex = -1; // animation clip, -1 for the bind pose
        float time = 0;
    };

    struct RenderedImage
    {
        int32_t width = 0;
        int32_t height = 0;
        int32_t originX = 0; // position of the model origin inside the image
        int32_t originY = 0;
        std::vector<std::array<uint8_t, 4>> rgba; // alpha 0 where nothing was drawn
        std::vector<float> depth;                 // larger is closer; -infinity where nothing was drawn

        bool empty() const
        {
            return width == 0 || height == 0;
        }
        size_t coveredPixels() const;
    };

    RenderedImage render(const GlbModel& model, const RenderSettings& settings);

    // Projects a map-space point to image-relative screen coordinates (relative to the model origin).
    std::array<float, 2> project(const Vec3& mapPos, float pixelsPerUnit);
    Vec3 modelToMap(const Vec3& modelPos);

    struct PaletteColour
    {
        uint8_t r, g, b;
    };

    struct QuantiseSettings
    {
        // Candidate palette entries, as (index, colour); index 0 is reserved for transparency by the caller.
        std::span<const std::pair<uint8_t, PaletteColour>> palette;
        std::optional<uint8_t> outlineIndex; // draws a 1 pixel outline in this palette index
    };

    struct PaletteImage
    {
        int32_t width = 0;
        int32_t height = 0;
        int32_t originX = 0;
        int32_t originY = 0;
        std::vector<uint8_t> pixels; // 0 = transparent
    };

    PaletteImage quantise(const RenderedImage& image, const QuantiseSettings& settings);
} // namespace OpenRCT2::Model
