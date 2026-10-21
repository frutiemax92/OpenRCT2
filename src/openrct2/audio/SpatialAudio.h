/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "AudioMixer.h"

#include <algorithm>
#include <cstdint>
#include <vector>

namespace OpenRCT2::Audio
{
    // 3D spatial audio (AUD-001 to AUD-005): the mixing decisions for sounds that come from places on the map. Rendering
    // and audio may use floating point (ARC-020 applies to the simulation only).

    // AUD-001: how an emitter sounds, from the `sounds` block of its object.
    struct SoundParams
    {
        float volume = 1.0f;        // 0..1 at the minimum distance
        bool loop = false;
        float minDistance = 64.0f;  // world units; full volume inside
        float maxDistance = 640.0f; // silent beyond
        float rolloff = 1.0f;       // exponent of the inverse distance law
        bool detail = false;        // swings, footsteps: lost when zoomed out (AUD "lowers detail sounds")
    };

    // AUD-005: the volume groups a sound belongs to.
    enum class SoundCategory : uint8_t
    {
        ambience, // nature, streams, animals
        vehicles,
        golfers,
        ui,
        music,
        count,
    };

    struct CategoryVolumes
    {
        // Percent 0..100.
        int32_t master = 100;
        int32_t category[static_cast<size_t>(SoundCategory::count)] = { 100, 100, 100, 100, 100 };

        float gain(SoundCategory c) const;
    };

    // AUD-002: where the listener is. The viewport centre projected onto the terrain; the distance to the ground grows
    // with the zoom level.
    struct Listener
    {
        float x = 0;
        float y = 0;
        float z = 0;
        int32_t rotation = 0;     // view rotation 0-3
        int32_t zoomLevel = 0;    // 0 closest
        float viewWidth = 1280;   // viewport width in pixels
        float pixelsPerUnit = 1;  // screen pixels per world unit at zoom level 0
    };

    struct SpatialResult
    {
        float volume = 0;   // 0..1 after distance, zoom and camera attenuation
        float pan = 0;      // -1 (left) .. +1 (right)
        float distance = 0; // world units
        bool audible = false;
    };

    // Emitters this far in front of the listener plane (towards the camera) are "behind the camera" and attenuated.
    constexpr float kBehindCameraMargin = 160.0f;
    constexpr float kBehindCameraGain = 0.5f;
    // Detail sounds fade out with the zoom level: 1.0, 0.6, 0.25 and none from the fourth level (AUD "zooming out").
    float detailZoomGain(int32_t zoomLevel);
    // How far apart the ear is from the ground: distance added to every emitter, growing with zoom.
    float listenerHeight(int32_t zoomLevel);

    // Distance attenuation: inverse distance clamped between the minimum and maximum, silent beyond the maximum.
    float distanceGain(float distance, const SoundParams& params);

    SpatialResult spatialise(const Listener& listener, float x, float y, float z, const SoundParams& params);

    // AUD-004: of all the sounds that want to play, the loudest ones get a voice, up to the limit.
    constexpr int32_t kDefaultVoiceLimit = 32;
    struct VoiceRequest
    {
        uint32_t emitterId = 0;
        float volume = 0;       // after attenuation and the volume of its group
        bool loop = false;
    };
    // Returns the indices into `requests`, loudest first, at most `limit` of them; silent requests never get a voice.
    std::vector<size_t> pickVoices(const std::vector<VoiceRequest>& requests, int32_t limit = kDefaultVoiceLimit);

    // AUD-004: looping sounds fade in and out when entering and leaving range. Moves `current` towards `target` by at
    // most `perTick` and returns the new value.
    float fadeTowards(float current, float target, float perTick);
    constexpr float kLoopFadePerTick = 0.04f; // about 0.6 s from silence to full at 40 ticks per second

    // AUD-003: existing mixer pan is 0.0 (left) to 1.0 (right), 0.5 centre.
    // SpatialResult pan is -1.0 (left) to +1.0 (right), 0.0 centre.
    inline float spatialPanToMixer(float pan)
    {
        return std::clamp((pan + 1.0f) * 0.5f, 0.0f, 1.0f);
    }

    // Converts spatial volume (0.0 to 1.0) to mixer volume (0 to kMixerVolumeMax).
    inline int32_t spatialVolumeToMixer(float volume)
    {
        return static_cast<int32_t>(std::clamp(volume, 0.0f, 1.0f) * kMixerVolumeMax);
    }
} // namespace OpenRCT2::Audio
