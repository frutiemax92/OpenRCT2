/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "BallPhysics.h"
#include "GolferNeeds.h"
#include "GolferSkills.h"
#include "Hole.h"
#include "PlayAI.h"
#include "Rules.h"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace OpenRCT2::Golf
{
    // The golfer (section 13): the data of a person playing, and the states of the state machine that drives them
    // (GLF-032). The behaviour is in GolfSimulation; this is the data and the rules that need no world.

    // GLF-032: what a golfer is doing. The order follows the life of a round.
    enum class GolferState : uint8_t
    {
        arriving,           // just spawned, coming from the car or the cottage
        walkingToCheckIn,
        checkingIn,         // at the golf club desk: pays the green fee and gets a tee time
        goingToCart,
        goingToTee,
        waitingForTeeTime,  // at the tee, until the group's tee time
        preparingShot,      // choosing club and aim
        swinging,           // the swing animation; the ball is hit at the end
        watchingBall,       // the ball is in the air or rolling
        walkingToBall,
        drivingToBall,
        searchingBall,      // looking for a ball lost in the rough (BAL-052)
        putting,
        holeFinished,
        usingFacility,      // eating, drinking, the toilet
        resting,            // sitting on a bench to get energy back
        talking,            // chatting with the group
        atDrivingRange,
        atMiniPutt,
        returningCart,
        leaving,            // walking back to the exit
        rageQuitting,       // storming off (GLF-041)
        inCar,              // still in the car that is driving to a parking space (STE-021)
        count,
    };
    const char* golferStateName(GolferState state);
    // Whether the golfer is in the middle of a hole, with a ball in play.
    bool isInPlay(GolferState state);

    // GLF-040: the causes of a golfer's anger that count as course problems.
    enum RageCause : uint8_t
    {
        kRageTooHard = 1 << 0,
        kRageLongWait = 1 << 1,
        kRageLostBall = 1 << 2,
        kRageHitByBall = 1 << 3,
    };
    constexpr uint32_t kWalkingToFacility = 0xFFFFFFFFu;
    constexpr int32_t kRageHappinessBelow = 30; // of 255
    constexpr int32_t kRageRadiusUnits = 3 * 32; // golfers within 3 tiles hear the insult (GLF-041)
    constexpr int32_t kInsultHappinessLoss = 6;  // of 255, for each golfer who hears it
    // The insults are strings of the language file, indexed 0..kInsultCount - 1.
    constexpr int32_t kInsultCount = 8;

    // A rage quit needs happiness under the threshold and at least one course problem behind it.
    bool shouldRageQuit(int32_t happinessByte, uint8_t causes);

    struct Thought
    {
        ThoughtType type = ThoughtType::enjoyingTheRound;
        uint32_t tick = 0;
    };

    struct Golfer
    {
        uint32_t id = 0;
        std::string name;
        uint32_t groupId = 0;

        SkillPoints skills;
        SkillPoints initialSkills;
        uint8_t equipmentLevel = 0;      // 0..3 (GLF-050)
        int64_t money = 0;               // currency units
        bool vip = false;
        Fixed riskTolerance = Fixed::ratio(5, 10);

        Needs needs;
        GolferState state = GolferState::arriving;
        uint32_t stateTicks = 0;         // ticks spent in the state
        uint8_t rageCauses = 0;

        uint32_t courseId = 0;
        uint32_t holeIndex = 0;          // position in the course, from 0
        TeeCategory tee = TeeCategory::men;
        HoleScore score;                 // the hole being played
        std::vector<int8_t> scorecard;   // strokes against par for each finished hole, 127 for a hole not finished
        std::vector<uint8_t> parCard;    // par of each finished hole
        int32_t strokesTotal = 0;

        FixedVec3 position;              // where the golfer stands
        FixedVec3 destination;           // where they are walking to
        Ball ball;                       // their ball, when in play
        ShotDecision decision;           // the shot they are about to play
        Fixed shotTargetDistance{};      // how far the shot was meant to go, to judge it afterwards
        bool ballInPlay = false;
        uint32_t teeTimeMinute = 0;      // when the group may tee off on the current hole
        uint32_t waitedMinutes = 0;      // waiting at the current tee
        uint32_t lastWaitMinute = 0;
        uint32_t holeStartMinute = 0;
        uint32_t facilityTicksLeft = 0;  // kWalkingToFacility while on the way there
        uint32_t facilityService = 0;    // id of the Service being visited
        bool facilityResumeBall = false; // after the visit, go back to the ball instead of the next hole
        uint32_t restingBenchId = 0;     // id of the bench being used (PTH-021)
        bool searchedBall = false;       // the lost ball check was done for the current lie
        NeedKind facilityNeed = NeedKind::none;
        bool wantsCart = false;
        bool rageQuit = false;
        bool finished = false;           // left the course
        uint32_t arrivalDay = 0;
        bool hasExit = false;            // leaves by their own way out: the car they came in (STE-012, STE-022)
        FixedVec3 exitPosition;

        std::vector<Thought> thoughts;

        GolferSkills currentSkills() const
        {
            return skills.toSkills();
        }
        // Stroke rate as a number against par over the finished holes.
        int32_t toParSoFar() const;
        void think(ThoughtType type, uint32_t tick);
    };
} // namespace OpenRCT2::Golf
