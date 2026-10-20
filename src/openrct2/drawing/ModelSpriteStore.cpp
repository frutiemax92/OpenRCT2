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
        _models[id] = Entry{ std::move(model), unitsPerMetre };
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

    void ModelSpriteStore::BeginFrame()
    {
        std::lock_guard lock(_mutex);
        _cache.trim();
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

        Model::ImposterRenderSettings settings;
        settings.unitsPerMetre = model->second.unitsPerMetre;
        settings.quantise.palette = _palette;
        settings.quantise.outlineIndex = std::nullopt;
        auto image = _cache.getOrCreate(key, [&] { return Model::renderImposter(*model->second.model, key, settings); });
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
