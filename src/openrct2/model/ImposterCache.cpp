/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "ImposterCache.h"

#include <algorithm>
#include <cmath>

namespace OpenRCT2::Model
{
    static constexpr float kTwoPi = 6.28318530718f;

    size_t ImposterKeyHash::operator()(const ImposterKey& key) const
    {
        uint64_t h = 1469598103934665603ull;
        auto mix = [&](uint64_t v) {
            h ^= v;
            h *= 1099511628211ull;
        };
        mix(key.modelId);
        mix(static_cast<uint16_t>(key.clip));
        mix(key.frame);
        mix(key.yawBucket);
        mix(key.pitchBucket);
        mix(key.rollBucket);
        mix(key.rotation);
        mix(key.zoom);
        for (auto r : key.remap)
            mix(r);
        return static_cast<size_t>(h);
    }

    uint8_t yawToBucket(float yawRadians)
    {
        const auto turns = yawRadians / kTwoPi;
        const auto bucket = static_cast<int32_t>(std::lround(turns * kYawBuckets));
        return static_cast<uint8_t>(((bucket % kYawBuckets) + kYawBuckets) % kYawBuckets);
    }

    float bucketToYaw(uint8_t bucket)
    {
        return static_cast<float>(bucket % kYawBuckets) * kTwoPi / kYawBuckets;
    }

    // Bucket b stands for (b - 4) * 5 degrees, so an upright model is rendered exactly upright.
    uint8_t tiltToBucket(float radians)
    {
        const auto bucket = static_cast<int32_t>(std::lround(radians / kTiltBucketSize)) + kTiltBuckets / 2;
        return static_cast<uint8_t>(std::clamp(bucket, 0, kTiltBuckets - 1));
    }

    float bucketToTilt(uint8_t bucket)
    {
        return static_cast<float>(static_cast<int32_t>(bucket) - kTiltBuckets / 2) * kTiltBucketSize;
    }

    uint16_t animationFrame(const GlbModel& model, int32_t clip, float time, float fps)
    {
        if (clip < 0 || clip >= static_cast<int32_t>(model.clips.size()) || fps <= 0)
            return 0;
        const auto duration = model.clips[clip].duration;
        const auto frames = std::max<int32_t>(1, static_cast<int32_t>(std::lround(duration * fps)));
        auto frame = static_cast<int32_t>(std::floor(time * fps)) % frames;
        if (frame < 0)
            frame += frames;
        return static_cast<uint16_t>(frame);
    }

    ImposterKey makeImposterKey(
        uint32_t modelId, const GlbModel& model, int32_t clip, float time, const Orientation& orientation, uint8_t rotation,
        uint8_t zoom, const std::array<uint8_t, kRemapSlots>& remap, float fps)
    {
        ImposterKey key;
        key.modelId = modelId;
        const bool validClip = clip >= 0 && clip < static_cast<int32_t>(model.clips.size());
        key.clip = validClip ? static_cast<int16_t>(clip) : static_cast<int16_t>(-1);
        key.frame = validClip ? animationFrame(model, clip, time, fps) : 0;
        key.yawBucket = yawToBucket(orientation.yaw);
        key.pitchBucket = tiltToBucket(orientation.pitch);
        key.rollBucket = tiltToBucket(orientation.roll);
        key.rotation = rotation & 3;
        key.zoom = zoom;
        key.remap = remap;
        return key;
    }

    PaletteImage renderImposter(const GlbModel& model, const ImposterKey& key, const ImposterRenderSettings& settings)
    {
        RenderSettings render_settings;
        render_settings.orientation.yaw = bucketToYaw(key.yawBucket) + static_cast<float>(key.rotation) * (kTwoPi / 4);
        render_settings.orientation.pitch = bucketToTilt(key.pitchBucket);
        render_settings.orientation.roll = bucketToTilt(key.rollBucket);
        render_settings.unitsPerMetre = settings.unitsPerMetre;
        render_settings.pixelsPerUnit = 1.0f / static_cast<float>(1 << key.zoom);
        render_settings.clipIndex = key.clip;
        if (key.clip >= 0 && key.clip < static_cast<int32_t>(model.clips.size()))
            render_settings.time = static_cast<float>(key.frame) / settings.fps;

        render_settings.materialColours.resize(model.materials.size());
        for (size_t material = 0; material < model.materials.size(); material++)
        {
            for (int32_t slot = 0; slot < kRemapSlots; slot++)
            {
                if (model.materials[material].name != kRemapMaterialNames[slot])
                    continue;
                for (const auto& [index, colour] : settings.quantise.palette)
                {
                    if (index != key.remap[slot])
                        continue;
                    const auto alpha = model.materials[material].baseColour[3];
                    render_settings.materialColours[material] = std::array<float, 4>{ colour.r / 255.0f, colour.g / 255.0f,
                                                                                     colour.b / 255.0f, alpha };
                    break;
                }
            }
        }
        return quantise(render(model, render_settings), settings.quantise);
    }

    ImposterCache::ImposterCache(size_t budgetBytes, bool trimOnInsert)
        : _budget(budgetBytes)
        , _trimOnInsert(trimOnInsert)
    {
    }

    void ImposterCache::setEvictCallback(EvictCallback callback)
    {
        _onEvict = std::move(callback);
    }

    ImposterCache::Image ImposterCache::find(const ImposterKey& key)
    {
        auto it = _entries.find(key);
        if (it == _entries.end())
        {
            _misses++;
            return nullptr;
        }
        _hits++;
        _lru.splice(_lru.begin(), _lru, it->second.order);
        return it->second.image;
    }

    ImposterCache::Image ImposterCache::insert(const ImposterKey& key, PaletteImage image)
    {
        auto existing = _entries.find(key);
        if (existing != _entries.end())
        {
            if (_onEvict)
                _onEvict(key);
            _bytes -= existing->second.image->pixels.size();
            _lru.erase(existing->second.order);
            _entries.erase(existing);
        }
        auto shared = std::make_shared<const PaletteImage>(std::move(image));
        _bytes += shared->pixels.size();
        _lru.push_front(key);
        _entries[key] = Entry{ shared, _lru.begin() };
        if (_trimOnInsert)
            trim();
        return shared;
    }

    ImposterCache::Image ImposterCache::getOrCreate(const ImposterKey& key, const Factory& factory)
    {
        if (auto found = find(key))
            return found;
        return insert(key, factory());
    }

    void ImposterCache::trim()
    {
        // The newest entry is always kept, even when it alone exceeds the budget.
        while (_bytes > _budget && _lru.size() > 1)
        {
            auto it = _entries.find(_lru.back());
            if (_onEvict)
                _onEvict(it->first);
            _bytes -= it->second.image->pixels.size();
            _entries.erase(it);
            _lru.pop_back();
        }
    }

    void ImposterCache::clear()
    {
        if (_onEvict)
            for (const auto& key : _lru)
                _onEvict(key);
        _entries.clear();
        _lru.clear();
        _bytes = 0;
    }

    void ImposterCache::setBudget(size_t bytes)
    {
        _budget = bytes;
        trim();
    }
} // namespace OpenRCT2::Model
