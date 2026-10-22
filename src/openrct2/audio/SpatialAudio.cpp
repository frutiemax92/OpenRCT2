/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "SpatialAudio.h"

#include "../config/Config.h"
#include "Audio.h"

#include <algorithm>
#include <cmath>

namespace OpenRCT2::Audio
{
    float CategoryVolumes::gain(SoundCategory c) const
    {
        const auto index = static_cast<size_t>(c);
        const int32_t group = index < static_cast<size_t>(SoundCategory::count) ? category[index] : 100;
        return std::clamp(master, 0, 100) / 100.0f * std::clamp(group, 0, 100) / 100.0f;
    }

    SoundCategory soundCategoryFor(SoundId id)
    {
        switch (id)
        {
            case SoundId::birdChirp:
            case SoundId::duck:
            case SoundId::streamLoop:
            case SoundId::rain:
            case SoundId::layingOutWater:
            case SoundId::water1:
            case SoundId::water2:
            case SoundId::waterSplash:
                return SoundCategory::ambience;

            case SoundId::cartMotorLoop:
            case SoundId::carEngineLoop:
            case SoundId::carDoor:
            case SoundId::goKartEngine:
            case SoundId::trainWhistle:
            case SoundId::trainDeparting:
            case SoundId::tram:
                return SoundCategory::vehicles;

            case SoundId::driverSwing:
            case SoundId::ironSwing:
            case SoundId::putt:
            case SoundId::ballImpactGrass:
            case SoundId::ballImpactSand:
            case SoundId::ballImpactHard:
            case SoundId::ballImpactWater:
            case SoundId::ballImpactTree:
            case SoundId::ballImpactBuilding:
            case SoundId::cupDrop:
            case SoundId::crowdClap:
            case SoundId::golferCheer:
            case SoundId::golferGroan:
            case SoundId::golferInsultGrumble:
            case SoundId::crowdAmbience:
                return SoundCategory::golfers;

            default:
                return SoundCategory::ui;
        }
    }

    CategoryVolumes getCurrentCategoryVolumes()
    {
        CategoryVolumes v;
        const auto& s = Config::Get().sound;
        v.master = s.masterSoundEnabled ? s.masterVolume : 0;
        v.category[static_cast<size_t>(SoundCategory::ambience)] = s.soundEnabled ? s.ambienceVolume : 0;
        v.category[static_cast<size_t>(SoundCategory::vehicles)] = s.soundEnabled ? s.vehicleVolume : 0;
        v.category[static_cast<size_t>(SoundCategory::golfers)] = s.soundEnabled ? s.golferVolume : 0;
        v.category[static_cast<size_t>(SoundCategory::ui)] = s.soundEnabled ? s.uiVolume : 0;
        v.category[static_cast<size_t>(SoundCategory::music)] = s.rideMusicEnabled ? s.rideMusicVolume : 0;
        return v;
    }

    float detailZoomGain(int32_t zoomLevel)
    {
        switch (std::max(0, zoomLevel))
        {
            case 0:
                return 1.0f;
            case 1:
                return 0.6f;
            case 2:
                return 0.25f;
            default:
                return 0.0f;
        }
    }

    float listenerHeight(int32_t zoomLevel)
    {
        // 48 units above the ground at the closest zoom, doubling with each zoom level.
        return 48.0f * static_cast<float>(1 << std::clamp(zoomLevel, 0, 5));
    }

    float distanceGain(float distance, const SoundParams& p)
    {
        if (distance >= p.maxDistance)
            return 0.0f;
        const float clamped = std::clamp(distance, p.minDistance, p.maxDistance);
        if (clamped <= 0.0f)
            return p.volume;
        const float ratio = p.minDistance / clamped;
        // The law is inverse distance (rolloff 1); a larger rolloff falls off faster. Fades to zero over the last tenth so
        // a sound does not cut off at the maximum distance.
        float gain = std::pow(ratio, p.rolloff);
        const float fadeStart = p.maxDistance * 0.9f;
        if (distance > fadeStart)
            gain *= (p.maxDistance - distance) / (p.maxDistance - fadeStart);
        return std::clamp(gain, 0.0f, 1.0f) * p.volume;
    }

    SpatialResult spatialise(const Listener& l, float x, float y, float z, const SoundParams& p)
    {
        SpatialResult r;
        const float dx = x - l.x, dy = y - l.y, dz = z - l.z;
        // The listener's ears are above the ground, so even an emitter right below is some way off.
        const float height = listenerHeight(l.zoomLevel);
        const float horizontal = std::sqrt(dx * dx + dy * dy);
        r.distance = std::sqrt(horizontal * horizontal + (dz + height) * (dz + height));
        if (r.distance >= p.maxDistance)
            return r;
        float volume = distanceGain(r.distance, p);

        // The projection rotates with the view: screen x is the difference of the rotated axes.
        float rx = dx, ry = dy;
        for (int32_t i = 0; i < (l.rotation & 3); i++)
        {
            const float nx = ry;
            const float ny = -rx;
            rx = nx;
            ry = ny;
        }
        const float screenX = (rx - ry) * l.pixelsPerUnit / static_cast<float>(1 << std::clamp(l.zoomLevel, 0, 5));
        r.pan = std::clamp(screenX / std::max(1.0f, l.viewWidth * 0.5f), -1.0f, 1.0f);

        // Depth along the line of sight (the camera looks towards -x -y after the rotation): emitters between the camera
        // and the listener are behind the camera plane.
        const float depth = (rx + ry) * 0.70710678f;
        if (depth > kBehindCameraMargin)
            volume *= kBehindCameraGain;

        if (p.detail)
            volume *= detailZoomGain(l.zoomLevel);
        r.volume = std::clamp(volume, 0.0f, 1.0f);
        r.audible = r.volume > 0.001f;
        return r;
    }

    std::vector<size_t> pickVoices(const std::vector<VoiceRequest>& requests, int32_t limit)
    {
        std::vector<size_t> order;
        for (size_t i = 0; i < requests.size(); i++)
            if (requests[i].volume > 0.001f)
                order.push_back(i);
        // Loudest first; looping sounds win ties so engines and streams do not flicker; then the emitter id for stability.
        std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
            if (requests[a].volume != requests[b].volume)
                return requests[a].volume > requests[b].volume;
            if (requests[a].loop != requests[b].loop)
                return requests[a].loop;
            return requests[a].emitterId < requests[b].emitterId;
        });
        if (limit >= 0 && order.size() > static_cast<size_t>(limit))
            order.resize(static_cast<size_t>(limit));
        return order;
    }

    float fadeTowards(float current, float target, float perTick)
    {
        if (current < target)
            return std::min(target, current + perTick);
        return std::max(target, current - perTick);
    }
} // namespace OpenRCT2::Audio
