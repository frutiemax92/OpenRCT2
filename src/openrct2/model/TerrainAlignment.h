/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "Rasteriser.h"

#include <functional>

namespace OpenRCT2::Model
{
    // Terrain alignment of 3D entities (R3D-006). Cars, carts and animals tilt to the surface under their
    // wheels or feet; golfers stay upright with their feet on the surface.

    // Surface height in map z units at a world position (map units, 32 per tile).
    using HeightSampler = std::function<float(float x, float y)>;

    // Distances of the contact points from the model origin, in map units. The model faces map +x before yaw.
    struct ContactPoints
    {
        float halfLength = 12.0f; // front and back contacts, along the heading
        float halfWidth = 6.0f;   // left and right contacts
    };

    struct Alignment
    {
        float pitch = 0; // radians, positive tips the nose down (the rasteriser's convention)
        float roll = 0;  // radians, positive lifts the left side (+y before yaw)
        float height = 0; // origin height: the mean of the four contact heights
    };

    // Samples the surface at the four contacts of an entity at (x, y) heading `yaw`.
    Alignment computeAlignment(const HeightSampler& surface, float x, float y, float yaw, const ContactPoints& contacts);

    // Height for an upright entity: the highest of its feet so neither sinks into the ground on a slope.
    float feetHeight(const HeightSampler& surface, float x, float y, float yaw, float footSpacing);

    // Exponential smoothing of pitch and roll over a few ticks so a vehicle does not jerk at tile boundaries.
    class AlignmentSmoother
    {
    public:
        static constexpr float kDefaultFactor = 0.35f; // fraction of the remaining difference applied per tick
        static constexpr float kMaxTilt = 0.6f;        // about 34 degrees; steeper surfaces are clamped

        explicit AlignmentSmoother(float factor = kDefaultFactor);

        // Feeds the raw alignment of one tick and returns the smoothed one. The first call snaps to the target.
        Alignment update(const Alignment& target);
        void reset();
        const Alignment& current() const
        {
            return _current;
        }

    private:
        float _factor;
        bool _initialised = false;
        Alignment _current;
    };
} // namespace OpenRCT2::Model
