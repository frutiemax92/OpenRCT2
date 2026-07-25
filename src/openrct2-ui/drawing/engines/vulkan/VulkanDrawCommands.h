/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include <cstdint>
#include <vector>

namespace OpenRCT2::Ui
{
    struct IVec3
    {
        int32_t x, y, z;
    };

    struct IVec4
    {
        int32_t x, y, z, w;
    };

    struct FVec4
    {
        float x, y, z, w;
    };

    // Per-instance data for lines. Field layout/order must exactly match the vertex input
    // attributes declared in data/shaders/line_vk.vert (locations 0-2).
    struct VulkanDrawLineCommand
    {
        IVec4 bounds;
        uint32_t colour;
        int32_t depth;
    };

    // Per-instance data for rects/sprites/glyphs/text. Field layout/order must exactly match
    // the vertex input attributes declared in data/shaders/rect_vk.vert (locations 0-10). This
    // is a straight port of the OpenGL renderer's DrawRectCommand (see
    // src/openrct2-ui/drawing/engines/opengl/DrawCommands.h) - kept field-for-field identical
    // so the shader logic ported from drawrect.frag stays correct without reinterpretation.
    struct VulkanDrawRectCommand
    {
        IVec4 clip;
        int32_t texColourAtlas;
        FVec4 texColourBounds;
        int32_t texMaskAtlas;
        FVec4 texMaskBounds;
        IVec3 palettes;
        int32_t flags;
        uint32_t colour;
        IVec4 bounds;
        int32_t depth;
        float zoom;

        enum
        {
            FLAG_NO_TEXTURE = (1u << 2u),
            FLAG_MASK = (1u << 3u),
            FLAG_CROSS_HATCH = (1u << 4u),
            FLAG_TTF_TEXT = (1u << 5u),
        };
    };

    // Simple growable per-frame command batch, mirroring OpenGL renderer's CommandBatch<T>.
    template<typename T>
    class CommandBatch
    {
    private:
        std::vector<T> _instances;
        size_t _numInstances = 0;

    public:
        [[nodiscard]] bool empty() const
        {
            return _numInstances == 0;
        }
        void clear()
        {
            _numInstances = 0;
        }
        T& allocate()
        {
            if (_numInstances + 1 > _instances.size())
            {
                _instances.resize((_numInstances + 1) << 1);
            }
            return _instances[_numInstances++];
        }
        [[nodiscard]] size_t size() const
        {
            return _numInstances;
        }
        const T* data() const
        {
            return _instances.data();
        }

        typename std::vector<T>::iterator begin() // NOLINT(readability-identifier-naming)
        {
            return _instances.begin();
        }
        typename std::vector<T>::const_iterator begin() const // NOLINT(readability-identifier-naming)
        {
            return _instances.cbegin();
        }
        typename std::vector<T>::iterator end() // NOLINT(readability-identifier-naming)
        {
            return _instances.begin() + _numInstances;
        }
        typename std::vector<T>::const_iterator end() const // NOLINT(readability-identifier-naming)
        {
            return _instances.cbegin() + _numInstances;
        }
    };

    using LineCommandBatch = CommandBatch<VulkanDrawLineCommand>;
    using RectCommandBatch = CommandBatch<VulkanDrawRectCommand>;
} // namespace OpenRCT2::Ui
