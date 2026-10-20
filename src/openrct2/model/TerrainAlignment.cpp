/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "TerrainAlignment.h"

#include <algorithm>
#include <cmath>

namespace OpenRCT2::Model
{
    Alignment computeAlignment(const HeightSampler& surface, float x, float y, float yaw, const ContactPoints& contacts)
    {
        const auto c = std::cos(yaw);
        const auto s = std::sin(yaw);
        // Local (lx, ly) to world: forward is (c, s), left is (-s, c).
        auto sample = [&](float lx, float ly) { return surface(x + lx * c - ly * s, y + lx * s + ly * c); };

        const auto front = sample(contacts.halfLength, 0);
        const auto back = sample(-contacts.halfLength, 0);
        const auto left = sample(0, contacts.halfWidth);
        const auto right = sample(0, -contacts.halfWidth);

        Alignment result;
        if (contacts.halfLength > 0)
            result.pitch = -std::atan((front - back) / (2 * contacts.halfLength));
        if (contacts.halfWidth > 0)
            result.roll = std::atan((left - right) / (2 * contacts.halfWidth));
        result.height = (front + back + left + right) / 4.0f;
        return result;
    }

    float feetHeight(const HeightSampler& surface, float x, float y, float yaw, float footSpacing)
    {
        const auto half = footSpacing / 2;
        const auto c = std::cos(yaw);
        const auto s = std::sin(yaw);
        float height = surface(x, y);
        height = std::max(height, surface(x + half * c, y + half * s));
        height = std::max(height, surface(x - half * c, y - half * s));
        height = std::max(height, surface(x - half * s, y + half * c));
        height = std::max(height, surface(x + half * s, y - half * c));
        return height;
    }

    AlignmentSmoother::AlignmentSmoother(float factor)
        : _factor(std::clamp(factor, 0.0f, 1.0f))
    {
    }

    Alignment AlignmentSmoother::update(const Alignment& target)
    {
        Alignment clamped = target;
        clamped.pitch = std::clamp(clamped.pitch, -kMaxTilt, kMaxTilt);
        clamped.roll = std::clamp(clamped.roll, -kMaxTilt, kMaxTilt);
        if (!_initialised)
        {
            _current = clamped;
            _initialised = true;
        }
        else
        {
            _current.pitch += (clamped.pitch - _current.pitch) * _factor;
            _current.roll += (clamped.roll - _current.roll) * _factor;
            _current.height = clamped.height; // the height is never lagged, or the entity would float or sink
        }
        return _current;
    }

    void AlignmentSmoother::reset()
    {
        _initialised = false;
        _current = {};
    }
} // namespace OpenRCT2::Model
