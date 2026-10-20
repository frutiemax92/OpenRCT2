/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Shot.h"

namespace OpenRCT2::Golf
{
    namespace
    {
        const Fixed kOne = Fixed::fromInt(1);

        // Documented constants of the shot model.
        constexpr int32_t kDirectionSigmaAccuracy0 = 600;  // 6.00 degrees of direction error at Accuracy 0
        constexpr int32_t kDirectionSigmaAccuracy100 = 60; // 0.60 degrees at Accuracy 100
        constexpr int32_t kDistanceSigmaAccuracy0 = 12;    // 12% distance error at Accuracy 0
        constexpr int32_t kDistanceSigmaAccuracy100 = 2;   // 2% at Accuracy 100
        constexpr int32_t kFatigueDistanceLoss = 20;       // percent of distance lost at full fatigue
        constexpr int32_t kFatigueSpreadGain = 2;          // spread grows by this multiple of the fatigue
        constexpr int32_t kEquipmentSkillBonus = 3;        // skill points per equipment level
        constexpr int32_t kSideSpinPerRadian = 3;
        constexpr int32_t kPuttDirectionShare = 25;        // putts use this percent of the direction spread

        // A bell shaped random number with standard deviation 1: the sum of four uniform numbers in [-1, 1).
        Fixed bell(IBallRandom& rng)
        {
            int64_t sum = 0;
            for (int i = 0; i < 4; i++)
                sum += static_cast<int64_t>((rng.next() >> 8) & 0xFFFF) * 2 - 65536;
            // The sum has a deviation of sqrt(4/3); multiply by sqrt(3/4) = 0.866 to get 1.
            return Fixed::fromRaw(sum * 56756 / 65536);
        }

        int32_t clampSkill(int32_t value)
        {
            return value < 0 ? 0 : (value > 100 ? 100 : value);
        }

        int32_t effectiveLongPlay(const ShotRequest& r)
        {
            return clampSkill(r.skills.longPlay + r.equipmentLevel * kEquipmentSkillBonus);
        }
        int32_t effectiveAccuracy(const ShotRequest& r)
        {
            return clampSkill(r.skills.accuracy + r.equipmentLevel * kEquipmentSkillBonus);
        }

        Fixed degreesToRadians(Fixed degrees)
        {
            return degrees * kFixedPi / 180;
        }
    } // namespace

    Fixed longPlayFactor(int32_t longPlay)
    {
        return Fixed::ratio(60, 100) + Fixed::ratio(70 * clampSkill(longPlay), 10000);
    }

    Fixed fatigue(uint8_t energy, int32_t foolproof)
    {
        const Fixed tired = Fixed::ratio(255 - energy, 255);
        return tired * Fixed::ratio(100 - clampSkill(foolproof), 100);
    }

    Fixed shotDistance(const ShotRequest& request)
    {
        const auto& club = clubInfo(request.club);
        const Fixed tired = fatigue(request.energy, request.skills.foolproof);
        Fixed distance = yardsToUnits(club.carryYards) * clamp(request.power, Fixed{}, kOne);
        distance = distance * longPlayFactor(effectiveLongPlay(request)) * request.lie.distanceFactor;
        return distance * (kOne - tired * Fixed::ratio(kFatigueDistanceLoss, 100));
    }

    Fixed directionSpread(const ShotRequest& request)
    {
        const auto& club = clubInfo(request.club);
        const Fixed accuracy = Fixed::ratio(effectiveAccuracy(request), 100);
        Fixed degrees = Fixed::ratio(kDirectionSigmaAccuracy0, 100)
            + (Fixed::ratio(kDirectionSigmaAccuracy100, 100) - Fixed::ratio(kDirectionSigmaAccuracy0, 100)) * accuracy;
        degrees = degrees * club.accuracyModifier;
        if (request.lie.accuracyFactor > Fixed{})
            degrees = degrees / request.lie.accuracyFactor;
        degrees = degrees * (kOne + fatigue(request.energy, request.skills.foolproof) * kFatigueSpreadGain);
        if (club.isPutter)
            degrees = degrees * Fixed::ratio(kPuttDirectionShare, 100);
        return degreesToRadians(degrees);
    }

    Fixed distanceSpread(const ShotRequest& request)
    {
        const auto& club = clubInfo(request.club);
        const Fixed accuracy = Fixed::ratio(effectiveAccuracy(request), 100);
        Fixed fraction = Fixed::ratio(kDistanceSigmaAccuracy0, 100)
            + (Fixed::ratio(kDistanceSigmaAccuracy100, 100) - Fixed::ratio(kDistanceSigmaAccuracy0, 100)) * accuracy;
        fraction = fraction * club.accuracyModifier;
        if (request.lie.accuracyFactor > Fixed{})
            fraction = fraction / request.lie.accuracyFactor;
        return fraction * (kOne + fatigue(request.energy, request.skills.foolproof) * kFatigueSpreadGain);
    }

    ShotResult computeShot(const ShotRequest& request, IBallRandom& rng, const BallParams& params)
    {
        const auto& club = clubInfo(request.club);
        ShotResult result;
        result.isPutt = club.isPutter;
        result.intendedDistance = shotDistance(request);

        const Fixed distanceError = bell(rng) * distanceSpread(request);
        result.requestedDistance = max(Fixed{}, result.intendedDistance * (kOne + distanceError));
        result.directionError = bell(rng) * directionSpread(request);

        // Horizontal direction: the aim turned by the error.
        FixedVec3 aim{ request.aimX, request.aimY, Fixed{} };
        aim = normalised(aim);
        if (aim == FixedVec3{})
            aim = { kOne, Fixed{}, Fixed{} };
        const Fixed cs = cos(result.directionError);
        const Fixed sn = sin(result.directionError);
        const FixedVec3 direction{ aim.x * cs - aim.y * sn, aim.x * sn + aim.y * cs, Fixed{} };

        if (club.isPutter)
        {
            // Constant rolling deceleration a: v^2 = 2 a d.
            const Fixed deceleration = params.rollResistance * request.surfaceRoughness;
            const Fixed speed = sqrt(deceleration * result.requestedDistance * 2);
            result.velocity = direction * speed;
            return result;
        }

        // Drag-free ballistic launch: range = v^2 sin(2 loft) / g.
        const Fixed loft = degreesToRadians(Fixed::fromInt(club.loftDegrees));
        const Fixed sin2 = sin(loft * 2);
        const Fixed speed = sqrt(result.requestedDistance * params.gravity / sin2) * club.launchScale;
        result.velocity = { direction.x * speed * cos(loft), direction.y * speed * cos(loft), speed * sin(loft) };
        result.backSpin = club.backSpin;
        result.sideSpin = bell(rng) * directionSpread(request) * kSideSpinPerRadian;
        return result;
    }

    void applyShot(Ball& ball, const ShotResult& shot)
    {
        ball.velocity = shot.velocity;
        ball.sideSpin = shot.sideSpin;
        ball.backSpin = shot.backSpin;
        ball.state = shot.isPutt ? BallState::Rolling : BallState::InFlight;
        ball.shotOrigin = ball.position;
        ball.lastSafePosition = ball.position;
        ball.passingObstacle = 0;
        ball.lippedOut = false;
        ball.bounces = 0;
    }
} // namespace OpenRCT2::Golf
