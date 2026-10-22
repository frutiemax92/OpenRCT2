/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "ModelViewer.h"

#include "../core/File.h"
#include "../drawing/ModelSpriteStore.h"

#include <algorithm>
#include <cmath>

namespace OpenRCT2::Model
{
    ModelViewer::~ModelViewer()
    {
        unload();
    }

    bool ModelViewer::loadFile(const std::string& path, std::string& error)
    {
        std::vector<uint8_t> bytes;
        try
        {
            bytes = File::ReadAllBytes(path);
        }
        catch (const std::exception& e)
        {
            error = e.what();
            return false;
        }
        const auto slash = path.find_last_of("/\\");
        return loadBytes(bytes, slash == std::string::npos ? path : path.substr(slash + 1), error);
    }

    bool ModelViewer::loadBytes(std::span<const uint8_t> data, const std::string& name, std::string& error)
    {
        auto result = loadGlb(data);
        if (!result.ok)
        {
            error = result.error;
            return false;
        }
        unload();
        _modelId = Drawing::ModelSpriteStore::Instance().RegisterModel(
            std::make_shared<GlbModel>(std::move(result.model)));
        _owned = true;
        _name = name;
        _time = 0;
        _yaw = 0;
        return true;
    }

    void ModelViewer::showRegistered(uint32_t modelId, const std::string& name)
    {
        unload();
        _modelId = modelId;
        _owned = false;
        _name = name;
        _time = 0;
        _yaw = 0;
    }

    void ModelViewer::unload()
    {
        if (_modelId != 0 && _owned)
        {
            Drawing::ModelSpriteStore::Instance().UnregisterModel(_modelId);
            Drawing::ModelSpriteStore::Instance().Clear();
        }
        _modelId = 0;
        _owned = true;
        _name.clear();
    }

    const GlbModel* ModelViewer::model() const
    {
        return _modelId != 0 ? Drawing::ModelSpriteStore::Instance().GetModel(_modelId) : nullptr;
    }

    int32_t ModelViewer::rowCount() const
    {
        const auto* m = model();
        return m == nullptr ? 0 : static_cast<int32_t>(m->clips.size()) + 1;
    }

    std::string ModelViewer::rowName(int32_t row) const
    {
        const auto* m = model();
        if (m == nullptr || row < 0 || row >= rowCount())
            return {};
        if (row == 0)
            return "(bind pose)";
        const auto& name = m->clips[row - 1].name;
        return name.empty() ? "clip " + std::to_string(row - 1) : name;
    }

    int32_t ModelViewer::clipOfRow(int32_t row) const
    {
        return std::clamp(row, 0, std::max(0, rowCount() - 1)) - 1;
    }

    ImposterKey ModelViewer::keyFor(int32_t row, int32_t rotation) const
    {
        const auto* m = model();
        if (m == nullptr)
            return {};
        Orientation orientation;
        orientation.yaw = _yaw;
        return makeImposterKey(_modelId, *m, clipOfRow(row), _time, orientation, static_cast<uint8_t>(rotation & 3), 0, {});
    }

    void ModelViewer::advance(float seconds)
    {
        if (!_paused)
            _time += seconds;
    }

    void ModelViewer::setPaused(bool paused)
    {
        _paused = paused;
    }

    void ModelViewer::setYaw(float radians)
    {
        _yaw = radians;
    }

    std::string ModelViewer::summary() const
    {
        const auto* m = model();
        if (m == nullptr)
            return "No model loaded";
        return _name + ": " + std::to_string(m->triangleCount()) + " triangles, " + std::to_string(m->clips.size())
            + " clips, " + std::to_string(m->images.size()) + " textures";
    }

    ModelViewer& GetModelViewer()
    {
        static ModelViewer viewer;
        return viewer;
    }
} // namespace OpenRCT2::Model
