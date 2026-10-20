/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../model/ImposterCache.h"
#include "../model/ImposterWorker.h"
#include "ImageIndexType.h"

#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace OpenRCT2::Drawing
{
    // Glue between the 3D models and the image list (R3D-004): the pose of an entity is rasterised through the imposter
    // cache and registered as an image so the paint code can draw it like any other sprite.
    class ModelSpriteStore
    {
    public:
        static ModelSpriteStore& Instance();

        // Registers a model and returns its id, used in imposter keys. unitsPerMetre is the scale of the model.
        uint32_t RegisterModel(std::shared_ptr<const Model::GlbModel> model, float unitsPerMetre = 10.0f);
        const Model::GlbModel* GetModel(uint32_t id) const;
        float GetUnitsPerMetre(uint32_t id) const;
        void UnregisterModel(uint32_t id);

        // Main thread, once per frame before painting: evicts old sprites.
        void BeginFrame();
        void Clear();
        void SetBudget(size_t bytes);

        // The image of a pose, or kImageIndexUndefined if the model is unknown or draws nothing. The image stays valid
        // until the next BeginFrame. Safe to call from paint threads.
        ImageIndex GetImage(const Model::ImposterKey& key);

        // Rasterises missing poses on worker threads (R3D-009): GetImage then returns kImageIndexUndefined for a pose until
        // its image is ready, which BeginFrame collects. 0 threads goes back to rasterising on the calling thread.
        void SetWorkerThreads(size_t threads);
        // Moves finished worker images into the cache; BeginFrame does this too. Returns how many arrived.
        size_t CollectFinished();
        // Blocks until all requested poses are rasterised and collected (tests, screenshots).
        void WaitForWorkers();

        size_t GetSpriteCount() const;

    private:
        ModelSpriteStore();

        struct Entry
        {
            std::shared_ptr<const Model::GlbModel> model;
            float unitsPerMetre;
            std::shared_ptr<const Model::TextureSet> textures;
        };

        mutable std::mutex _mutex;
        Model::ImposterCache _cache;
        std::unordered_map<Model::ImposterKey, ImageIndex, Model::ImposterKeyHash> _images;
        std::unordered_map<uint32_t, Entry> _models;
        std::vector<std::pair<uint8_t, Model::PaletteColour>> _palette;
        std::shared_ptr<const std::vector<std::pair<uint8_t, Model::PaletteColour>>> _sharedPalette;
        std::unique_ptr<Model::ImposterWorker> _worker;
        uint32_t _nextId = 1;
    };
} // namespace OpenRCT2::Drawing
