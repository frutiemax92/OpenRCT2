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

#include <array>
#include <cstdint>
#include <functional>
#include <list>
#include <memory>
#include <unordered_map>

namespace OpenRCT2::Model
{
    // Imposter cache (R3D-005): rasterised model poses, keyed by the quantised pose so that identical poses are
    // never rasterised twice.
    constexpr int32_t kYawBuckets = 32;
    constexpr int32_t kTiltBuckets = 8;         // pitch and roll
    constexpr float kTiltBucketSize = 0.0872665f; // 5 degrees, in radians
    constexpr float kDefaultAnimationFps = 12.0f;
    constexpr size_t kDefaultImposterBudgetBytes = 64u * 1024u * 1024u;
    constexpr int32_t kRemapSlots = 3;

    struct ImposterKey
    {
        uint32_t modelId = 0;
        int16_t clip = -1; // -1 for the bind pose
        uint16_t frame = 0;
        uint8_t yawBucket = 0;
        uint8_t pitchBucket = kTiltBuckets / 2; // the bucket that contains 0 radians is the upright one
        uint8_t rollBucket = kTiltBuckets / 2;
        uint8_t rotation = 0; // view rotation 0-3
        uint8_t zoom = 0;
        std::array<uint8_t, kRemapSlots> remap{}; // palette indices of the 3 remappable material slots

        bool operator==(const ImposterKey&) const = default;
    };

    struct ImposterKeyHash
    {
        size_t operator()(const ImposterKey& key) const;
    };

    // Quantisation of the continuous pose values into key buckets, and back to the value that is rendered.
    uint8_t yawToBucket(float yawRadians);
    float bucketToYaw(uint8_t bucket);
    uint8_t tiltToBucket(float radians); // clamps to the outermost buckets
    float bucketToTilt(uint8_t bucket);
    // Animation frame at `time` seconds, looping over the clip. Returns 0 for the bind pose.
    uint16_t animationFrame(const GlbModel& model, int32_t clip, float time, float fps = kDefaultAnimationFps);

    ImposterKey makeImposterKey(
        uint32_t modelId, const GlbModel& model, int32_t clip, float time, const Orientation& orientation, uint8_t rotation,
        uint8_t zoom, const std::array<uint8_t, kRemapSlots>& remap, float fps = kDefaultAnimationFps);

    // Materials named like this are recoloured by the remap slots of the key (R3D-007).
    constexpr const char* kRemapMaterialNames[kRemapSlots] = { "remap_primary", "remap_secondary", "remap_tertiary" };

    struct ImposterRenderSettings
    {
        float unitsPerMetre = 10.0f;
        float fps = kDefaultAnimationFps;
        QuantiseSettings quantise;
    };

    // Rasterises the pose a key stands for (the bucket centres, not the original continuous values), so the cached
    // result is the same whichever caller filled the slot.
    PaletteImage renderImposter(const GlbModel& model, const ImposterKey& key, const ImposterRenderSettings& settings);

    class ImposterCache
    {
    public:
        explicit ImposterCache(size_t budgetBytes = kDefaultImposterBudgetBytes, bool trimOnInsert = true);

        // Called just before an image is dropped (eviction or clear).
        using EvictCallback = std::function<void(const ImposterKey&)>;
        void setEvictCallback(EvictCallback callback);

        using Image = std::shared_ptr<const PaletteImage>;
        using Factory = std::function<PaletteImage()>;

        Image find(const ImposterKey& key);
        Image insert(const ImposterKey& key, PaletteImage image);
        // The cached image, or the result of `factory`, which is then cached.
        Image getOrCreate(const ImposterKey& key, const Factory& factory);

        // Drops the least recently used images until the cache fits its budget.
        void trim();
        void clear();
        void setBudget(size_t bytes);

        size_t sizeBytes() const
        {
            return _bytes;
        }
        size_t count() const
        {
            return _entries.size();
        }
        uint64_t hits() const
        {
            return _hits;
        }
        uint64_t misses() const
        {
            return _misses;
        }

    private:
        struct Entry
        {
            Image image;
            std::list<ImposterKey>::iterator order;
        };

        size_t _budget;
        bool _trimOnInsert;
        EvictCallback _onEvict;
        size_t _bytes = 0;
        uint64_t _hits = 0;
        uint64_t _misses = 0;
        std::list<ImposterKey> _lru; // most recently used first
        std::unordered_map<ImposterKey, Entry, ImposterKeyHash> _entries;
    };
} // namespace OpenRCT2::Model
