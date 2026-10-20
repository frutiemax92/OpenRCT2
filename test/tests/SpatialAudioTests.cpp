/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <cmath>
#include <gtest/gtest.h>
#include <vector>
#include <openrct2/audio/SpatialAudio.h>

using namespace OpenRCT2::Audio;

namespace
{
    Listener listenerAt(float x, float y, int rotation = 0, int zoom = 0)
    {
        Listener l;
        l.x = x;
        l.y = y;
        l.rotation = rotation;
        l.zoomLevel = zoom;
        l.viewWidth = 1000;
        l.pixelsPerUnit = 1.0f;
        return l;
    }
} // namespace

TEST(CategoryVolumes, MasterScalesEveryGroup)
{
    CategoryVolumes v;
    EXPECT_FLOAT_EQ(v.gain(SoundCategory::golfers), 1.0f);
    v.master = 50;
    v.category[static_cast<size_t>(SoundCategory::golfers)] = 40;
    EXPECT_FLOAT_EQ(v.gain(SoundCategory::golfers), 0.2f);
    EXPECT_FLOAT_EQ(v.gain(SoundCategory::vehicles), 0.5f);
    v.master = 0;
    EXPECT_FLOAT_EQ(v.gain(SoundCategory::ambience), 0.0f);
    v.master = 500; // clamped
    v.category[0] = -5;
    EXPECT_FLOAT_EQ(v.gain(SoundCategory::ambience), 0.0f);
    EXPECT_FLOAT_EQ(v.gain(SoundCategory::ui), 1.0f);
}

TEST(DistanceGain, IsFullInsideTheMinimumAndInverseBeyondIt)
{
    SoundParams p;
    p.minDistance = 100;
    p.maxDistance = 1000;
    EXPECT_FLOAT_EQ(distanceGain(0, p), 1.0f);
    EXPECT_FLOAT_EQ(distanceGain(100, p), 1.0f);
    EXPECT_NEAR(distanceGain(200, p), 0.5f, 1e-4f);
    EXPECT_NEAR(distanceGain(400, p), 0.25f, 1e-4f);
    EXPECT_LT(distanceGain(800, p), distanceGain(400, p));
}

TEST(DistanceGain, IsSilentBeyondTheMaximumAndFadesToIt)
{
    SoundParams p;
    p.minDistance = 100;
    p.maxDistance = 1000;
    EXPECT_FLOAT_EQ(distanceGain(1000, p), 0.0f);
    EXPECT_FLOAT_EQ(distanceGain(5000, p), 0.0f);
    EXPECT_GT(distanceGain(990, p), 0.0f);
    EXPECT_LT(distanceGain(990, p), distanceGain(900, p) * 0.2f); // faded out over the last tenth
}

TEST(DistanceGain, RolloffAndVolumeShapeTheCurve)
{
    SoundParams soft;
    soft.minDistance = 100;
    soft.maxDistance = 2000;
    SoundParams steep = soft;
    steep.rolloff = 2.0f;
    EXPECT_LT(distanceGain(400, steep), distanceGain(400, soft));
    EXPECT_NEAR(distanceGain(400, steep), 0.0625f, 1e-4f);
    SoundParams quiet = soft;
    quiet.volume = 0.5f;
    EXPECT_NEAR(distanceGain(400, quiet), distanceGain(400, soft) * 0.5f, 1e-5f);
}

TEST(Spatialise, ASoundOnTheRightPansRight)
{
    const auto l = listenerAt(1000, 1000);
    SoundParams p;
    // With the default view rotation, screen right is +x and -y.
    const auto right = spatialise(l, 1100, 1000, 0, p);
    const auto left = spatialise(l, 900, 1000, 0, p);
    const auto centre = spatialise(l, 1000, 1000, 0, p);
    EXPECT_GT(right.pan, 0.05f);
    EXPECT_LT(left.pan, -0.05f);
    EXPECT_NEAR(centre.pan, 0.0f, 1e-4f);
    EXPECT_NEAR(right.pan, -left.pan, 1e-4f);
    // Along the other diagonal the pan is the opposite sign: +y moves left.
    EXPECT_LT(spatialise(l, 1000, 1100, 0, p).pan, -0.05f);
}

TEST(Spatialise, TheViewRotationTurnsThePan)
{
    SoundParams p;
    const float x = 1100, y = 1050;
    const float p0 = spatialise(listenerAt(1000, 1000, 0), x, y, 0, p).pan;
    const float p1 = spatialise(listenerAt(1000, 1000, 1), x, y, 0, p).pan;
    const float p2 = spatialise(listenerAt(1000, 1000, 2), x, y, 0, p).pan;
    EXPECT_GT(p0, 0.0f);
    EXPECT_LT(p2, 0.0f); // half a turn flips it
    EXPECT_NEAR(p0, -p2, 1e-4f);
    EXPECT_NE(p1, p0);
}

TEST(Spatialise, ScrollingPastAStreamPansLeftToRightAndFades)
{
    // Acceptance criterion of section 21: a stream passes the view; its sound moves across and fades with distance.
    SoundParams stream;
    stream.loop = true;
    stream.minDistance = 100;
    stream.maxDistance = 1200;
    const float sx = 2000, sy = 2000;
    float lastPan = -2.0f;
    float peakVolume = 0;
    int pans = 0;
    // The listener moves from -x to +x past the stream along the screen axis (+x/-y diagonal).
    std::vector<float> volumes;
    for (int step = -10; step <= 10; step++)
    {
        const auto l = listenerAt(sx - step * 80.0f, sy + step * 80.0f * 0.0f);
        // The emitter is the stream; as the listener moves towards -x the stream moves from the left to the right.
        const auto r = spatialise(l, sx, sy, 0, stream);
        volumes.push_back(r.volume);
        if (r.audible)
        {
            EXPECT_GE(r.pan, lastPan - 1e-3f); // moving across the screen in one direction
            lastPan = r.pan;
            pans++;
        }
        peakVolume = std::max(peakVolume, r.volume);
    }
    EXPECT_GT(pans, 8);
    // It is loudest when the stream is level with the listener and quieter at both ends.
    EXPECT_GT(volumes[10], volumes[0]);
    EXPECT_GT(volumes[10], volumes[20]);
    EXPECT_FLOAT_EQ(peakVolume, volumes[10]);
}

TEST(Spatialise, DistanceAttenuatesAndSilencesFarEmitters)
{
    SoundParams p;
    p.minDistance = 64;
    p.maxDistance = 640;
    const auto l = listenerAt(0, 0);
    const auto near = spatialise(l, 40, 0, 0, p);
    const auto far = spatialise(l, 400, 0, 0, p);
    const auto gone = spatialise(l, 5000, 0, 0, p);
    EXPECT_GT(near.volume, far.volume);
    EXPECT_TRUE(near.audible);
    EXPECT_TRUE(far.audible);
    EXPECT_FALSE(gone.audible);
    EXPECT_FLOAT_EQ(gone.volume, 0.0f);
    EXPECT_GT(far.distance, 400.0f);
}

TEST(Spatialise, EmittersBehindTheCameraPlaneAreQuieter)
{
    SoundParams p;
    p.minDistance = 10;
    p.maxDistance = 5000;
    p.rolloff = 0.0f; // no distance effect, so only the camera plane matters
    const auto l = listenerAt(1000, 1000);
    // The camera sits at +x +y: positive depth along (1, 1) is towards the camera.
    const auto towardsCamera = spatialise(l, 1400, 1400, 0, p);
    const auto awayFromCamera = spatialise(l, 600, 600, 0, p);
    EXPECT_NEAR(awayFromCamera.volume, 1.0f, 1e-4f);
    EXPECT_NEAR(towardsCamera.volume, kBehindCameraGain, 1e-4f);
}

TEST(Spatialise, ZoomingOutLowersDetailSoundsOnly)
{
    SoundParams swing;
    swing.detail = true;
    swing.minDistance = 100;
    swing.maxDistance = 4000;
    SoundParams ambience = swing;
    ambience.detail = false;
    float lastDetail = 2, lastAmbience = 0;
    for (int zoom = 0; zoom <= 3; zoom++)
    {
        const auto l = listenerAt(0, 0, 0, zoom);
        const auto d = spatialise(l, 50, 0, 0, swing);
        const auto a = spatialise(l, 50, 0, 0, ambience);
        EXPECT_LE(d.volume, lastDetail + 1e-6f) << zoom;
        lastDetail = d.volume;
        // Ambience is not cut by the zoom, only softened by the height of the listener above the ground.
        EXPECT_TRUE(a.audible) << zoom;
        if (zoom > 0)
        {
            EXPECT_LE(a.volume, lastAmbience + 1e-6f);
        }
        lastAmbience = a.volume;
        if (zoom == 3)
        {
            EXPECT_FALSE(d.audible);
        }
    }
    EXPECT_FLOAT_EQ(detailZoomGain(0), 1.0f);
    EXPECT_FLOAT_EQ(detailZoomGain(3), 0.0f);
    EXPECT_FLOAT_EQ(detailZoomGain(-2), 1.0f);
    EXPECT_GT(listenerHeight(3), listenerHeight(0));
}

TEST(Voices, TheLoudestSoundsGetTheVoices)
{
    std::vector<VoiceRequest> requests;
    for (uint32_t i = 0; i < 50; i++)
        requests.push_back({ i, 0.01f + 0.01f * i, false });
    requests.push_back({ 100, 0.0f, false }); // silent
    const auto chosen = pickVoices(requests);
    ASSERT_EQ(chosen.size(), 32u);
    EXPECT_EQ(requests[chosen[0]].emitterId, 49u); // loudest first
    EXPECT_EQ(requests[chosen[31]].emitterId, 18u);
    for (auto index : chosen)
        EXPECT_NE(requests[index].emitterId, 100u);
    // Fewer sounds than voices: everything audible plays.
    std::vector<VoiceRequest> few = { { 1, 0.5f, false }, { 2, 0.0f, false }, { 3, 0.2f, true } };
    const auto picked = pickVoices(few);
    ASSERT_EQ(picked.size(), 2u);
    EXPECT_EQ(few[picked[0]].emitterId, 1u);
    EXPECT_EQ(pickVoices(few, 1).size(), 1u);
    EXPECT_TRUE(pickVoices({}, 32).empty());
}

TEST(Voices, EqualLoudnessPrefersLoopsThenTheLowerId)
{
    std::vector<VoiceRequest> requests = { { 9, 0.5f, false }, { 4, 0.5f, false }, { 7, 0.5f, true } };
    const auto chosen = pickVoices(requests);
    ASSERT_EQ(chosen.size(), 3u);
    EXPECT_EQ(requests[chosen[0]].emitterId, 7u);
    EXPECT_EQ(requests[chosen[1]].emitterId, 4u);
    EXPECT_EQ(requests[chosen[2]].emitterId, 9u);
}

TEST(Voices, LoopsFadeInAndOutAtASteadyRate)
{
    float v = 0;
    int ticks = 0;
    while (v < 1.0f && ticks < 1000)
    {
        v = fadeTowards(v, 1.0f, kLoopFadePerTick);
        ticks++;
    }
    EXPECT_EQ(ticks, 25);
    EXPECT_FLOAT_EQ(v, 1.0f);
    v = fadeTowards(v, 0.0f, 0.5f);
    EXPECT_FLOAT_EQ(v, 0.5f);
    v = fadeTowards(v, 0.0f, 0.5f);
    v = fadeTowards(v, 0.0f, 0.5f);
    EXPECT_FLOAT_EQ(v, 0.0f);
    EXPECT_FLOAT_EQ(fadeTowards(0.3f, 0.3f, 0.1f), 0.3f);
}
