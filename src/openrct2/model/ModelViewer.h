/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "ImposterCache.h"

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace OpenRCT2::Model
{
    // State of the model viewer debug window (R3D-008): one loaded model shown with all its animation clips in all
    // four view rotations. A row is a clip (the first row is the bind pose), a column a view rotation.
    class ModelViewer
    {
    public:
        static constexpr int32_t kRotations = 4;

        ~ModelViewer();

        bool loadFile(const std::string& path, std::string& error);
        bool loadBytes(std::span<const uint8_t> data, const std::string& name, std::string& error);
        // Shows a model another object registered in the sprite store (the golfer, a car, a deer): it stays the object's.
        void showRegistered(uint32_t modelId, const std::string& name);
        void unload();
        bool isLoaded() const
        {
            return _modelId != 0;
        }

        const std::string& name() const
        {
            return _name;
        }
        uint32_t modelId() const
        {
            return _modelId;
        }
        const GlbModel* model() const;

        // Rows: bind pose first, then one row per clip.
        int32_t rowCount() const;
        std::string rowName(int32_t row) const;
        int32_t clipOfRow(int32_t row) const; // -1 for the bind pose

        // The pose drawn in a cell. The yaw is the heading the viewer shows (turned by `yaw`), the rotation the view
        // rotation of the column.
        ImposterKey keyFor(int32_t row, int32_t rotation) const;

        // Animation time shared by all rows; paused viewers keep their time.
        void advance(float seconds);
        void setPaused(bool paused);
        bool paused() const
        {
            return _paused;
        }
        float time() const
        {
            return _time;
        }
        void setYaw(float radians);
        float yaw() const
        {
            return _yaw;
        }
        // One line summary: triangle count, clips and whether the model is within the budget of its kind.
        std::string summary() const;

    private:
        uint32_t _modelId = 0;
        std::string _name;
        float _time = 0;
        float _yaw = 0;
        bool _paused = false;
        bool _owned = true; // registered by the viewer itself, so it unregisters the model
    };

    ModelViewer& GetModelViewer();
} // namespace OpenRCT2::Model
