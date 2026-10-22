/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "ModelSpriteStore.h"

#include "Drawing.h"
#include "ImageImporter.h"
#include "G1Element.h"
#include "Image.h"

namespace OpenRCT2::Drawing
{
    namespace
    {
        // The colour ramps of the standard palette; 0 is transparent and the entries above are water, remap and
        // special colours that a shaded model must not use.
        constexpr uint8_t kFirstShadeIndex = 10;
        constexpr uint8_t kLastShadeIndex = 201;
    } // namespace

    ModelSpriteStore& ModelSpriteStore::Instance()
    {
        static ModelSpriteStore store;
        return store;
    }

    ModelSpriteStore::ModelSpriteStore()
        : _cache(Model::kDefaultImposterBudgetBytes, false)
    {
        for (int32_t i = kFirstShadeIndex; i <= kLastShadeIndex; i++)
            _palette.push_back(
                { static_cast<uint8_t>(i), { StandardPalette[i].red, StandardPalette[i].green, StandardPalette[i].blue } });
        _sharedPalette = std::make_shared<const std::vector<std::pair<uint8_t, Model::PaletteColour>>>(_palette);
        _cache.setEvictCallback([this](const Model::ImposterKey& key) {
            auto it = _images.find(key);
            if (it != _images.end())
            {
                GfxObjectFreeImages(it->second, 1);
                _images.erase(it);
            }
        });
    }

    uint32_t ModelSpriteStore::RegisterModel(std::shared_ptr<const Model::GlbModel> model, float unitsPerMetre)
    {
        std::lock_guard lock(_mutex);
        const auto id = _nextId++;
        auto textures = std::make_shared<const Model::TextureSet>(Model::TextureSet::decode(*model));
        _models[id] = Entry{ std::move(model), unitsPerMetre, std::move(textures) };
        return id;
    }

    const Model::GlbModel* ModelSpriteStore::GetModel(uint32_t id) const
    {
        std::lock_guard lock(_mutex);
        auto it = _models.find(id);
        return it != _models.end() ? it->second.model.get() : nullptr;
    }

    float ModelSpriteStore::GetUnitsPerMetre(uint32_t id) const
    {
        std::lock_guard lock(_mutex);
        auto it = _models.find(id);
        return it != _models.end() ? it->second.unitsPerMetre : 10.0f;
    }

    void ModelSpriteStore::UnregisterModel(uint32_t id)
    {
        std::lock_guard lock(_mutex);
        _models.erase(id);
    }

    size_t ModelSpriteStore::BeginFrame()
    {
        std::lock_guard lock(_mutex);
        size_t numFinished = 0;
        if (_worker != nullptr)
        {
            for (auto& finished : _worker->takeFinished())
            {
                _cache.insert(finished.key, std::move(finished.image));
                numFinished++;
            }
        }
        _cache.trim();
        return numFinished;
    }

    void ModelSpriteStore::SetWorkerThreads(size_t threads)
    {
        std::unique_ptr<Model::ImposterWorker> old;
        {
            std::lock_guard lock(_mutex);
            old = std::move(_worker);
        }
        old.reset(); // joins outside the lock: running renders do not touch the store
        std::lock_guard lock(_mutex);
        if (threads > 0)
            _worker = std::make_unique<Model::ImposterWorker>(threads);
    }

    size_t ModelSpriteStore::CollectFinished()
    {
        std::lock_guard lock(_mutex);
        if (_worker == nullptr)
            return 0;
        auto finished = _worker->takeFinished();
        for (auto& f : finished)
            _cache.insert(f.key, std::move(f.image));
        return finished.size();
    }

    void ModelSpriteStore::WaitForWorkers()
    {
        Model::ImposterWorker* worker;
        {
            std::lock_guard lock(_mutex);
            worker = _worker.get();
        }
        if (worker != nullptr)
            worker->waitIdle();
        CollectFinished();
    }

    void ModelSpriteStore::Clear()
    {
        std::lock_guard lock(_mutex);
        _cache.clear();
    }

    void ModelSpriteStore::SetBudget(size_t bytes)
    {
        std::lock_guard lock(_mutex);
        _cache.setBudget(bytes);
    }

    size_t ModelSpriteStore::GetSpriteCount() const
    {
        std::lock_guard lock(_mutex);
        return _cache.count();
    }

    ImageIndex ModelSpriteStore::GetImage(const Model::ImposterKey& key)
    {
        std::lock_guard lock(_mutex);
        auto model = _models.find(key.modelId);
        if (model == _models.end())
            return kImageIndexUndefined;

        auto known = _images.find(key);
        if (known != _images.end())
        {
            _cache.find(key); // keeps it recently used
            return known->second;
        }

        std::shared_ptr<const Model::PaletteImage> image;
        if (_worker != nullptr)
        {
            image = _cache.find(key);
            if (image == nullptr)
            {
                _worker->request(
                    key, [entry = model->second, palette = _sharedPalette, key] {
                        Model::ImposterRenderSettings settings;
                        settings.unitsPerMetre = entry.unitsPerMetre;
                        settings.quantise.palette = *palette;
                        settings.textures = entry.textures.get();
                        return Model::renderImposter(*entry.model, key, settings);
                    });
                return kImageIndexUndefined;
            }
        }
        else
        {
            Model::ImposterRenderSettings settings;
            settings.unitsPerMetre = model->second.unitsPerMetre;
            settings.quantise.palette = _palette;
            settings.textures = model->second.textures.get();
            image = _cache.getOrCreate(key, [&] { return Model::renderImposter(*model->second.model, key, settings); });
        }
        if (image == nullptr || image->pixels.empty())
            return kImageIndexUndefined;

        G1Element g1{};
        g1.offset = const_cast<uint8_t*>(image->pixels.data());
        g1.width = static_cast<int16_t>(image->width);
        g1.height = static_cast<int16_t>(image->height);
        g1.xOffset = static_cast<int16_t>(-image->originX);
        g1.yOffset = static_cast<int16_t>(-image->originY);
        g1.flags = { G1Flag::hasTransparency };
        const auto index = GfxObjectAllocateImages(&g1, 1);
        if (index == kImageIndexUndefined)
            return kImageIndexUndefined;
        _images[key] = index;
        return index;
    }
} // namespace OpenRCT2::Drawing

namespace OpenRCT2::Drawing
{
    ImageIndex ModelSpriteStore::GetImageOrNearest(const Model::ImposterKey& key, Model::ImposterKey& shown)
    {
        shown = key;
        const auto exact = GetImage(key); // also queues the rasterisation of a missing pose
        if (exact != kImageIndexUndefined)
            return exact;

        std::lock_guard lock(_mutex);
        constexpr int32_t kMaxFrameDistance = 8;
        constexpr int32_t kMaxYawDistance = 2;
        for (int32_t yawStep = 0; yawStep <= kMaxYawDistance; yawStep++)
        {
            for (int32_t yawSign = (yawStep == 0 ? 1 : -1); yawSign <= 1; yawSign += 2)
            {
                auto candidate = key;
                candidate.yawBucket = static_cast<uint8_t>(
                    (key.yawBucket + yawSign * yawStep + Model::kYawBuckets) % Model::kYawBuckets);
                for (int32_t frameStep = 0; frameStep <= kMaxFrameDistance; frameStep++)
                {
                    // Prefer the frame just before: it is the one that was on screen a moment ago.
                    for (int32_t frameSign = (frameStep == 0 ? 1 : -1); frameSign <= 1; frameSign += 2)
                    {
                        const auto frame = static_cast<int32_t>(key.frame) + frameSign * frameStep;
                        if (frame < 0)
                            continue;
                        candidate.frame = static_cast<uint16_t>(frame);
                        auto it = _images.find(candidate);
                        if (it != _images.end())
                        {
                            shown = candidate;
                            return it->second;
                        }
                    }
                }
            }
        }
        return kImageIndexUndefined;
    }
} // namespace OpenRCT2::Drawing
