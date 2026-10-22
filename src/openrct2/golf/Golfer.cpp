/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Golfer.h"

namespace OpenRCT2::Golf
{
    const char* golferStateName(GolferState state)
    {
        switch (state)
        {
            case GolferState::arriving:
                return "Arriving";
            case GolferState::walkingToCheckIn:
                return "Walking to check in";
            case GolferState::checkingIn:
                return "Checking in";
            case GolferState::goingToCart:
                return "Going to the cart";
            case GolferState::goingToTee:
                return "Going to the tee";
            case GolferState::waitingForTeeTime:
                return "Waiting for the tee time";
            case GolferState::preparingShot:
                return "Preparing a shot";
            case GolferState::swinging:
                return "Swinging";
            case GolferState::watchingBall:
                return "Watching the ball";
            case GolferState::walkingToBall:
                return "Walking to the ball";
            case GolferState::drivingToBall:
                return "Driving to the ball";
            case GolferState::searchingBall:
                return "Searching for the ball";
            case GolferState::putting:
                return "Putting";
            case GolferState::holeFinished:
                return "Hole finished";
            case GolferState::usingFacility:
                return "Using a facility";
            case GolferState::resting:
                return "Resting";
            case GolferState::talking:
                return "Talking";
            case GolferState::atDrivingRange:
                return "At the driving range";
            case GolferState::atMiniPutt:
                return "At the mini putt";
            case GolferState::returningCart:
                return "Returning the cart";
            case GolferState::leaving:
                return "Leaving";
            case GolferState::inCar:
                return "In car";
            case GolferState::walkingToRange:
                return "Going to the driving range";
            case GolferState::waitingForTurn:
                return "Waiting for a turn";
            case GolferState::rageQuitting:
                return "Rage quitting";
            default:
                return "";
        }
    }

    bool isInPlay(GolferState state)
    {
        switch (state)
        {
            case GolferState::preparingShot:
            case GolferState::swinging:
            case GolferState::watchingBall:
            case GolferState::walkingToBall:
            case GolferState::drivingToBall:
            case GolferState::searchingBall:
            case GolferState::putting:
            case GolferState::waitingForTurn:
                return true;
            default:
                return false;
        }
    }

    bool shouldRageQuit(int32_t happinessByte, uint8_t causes)
    {
        return happinessByte < kRageHappinessBelow && (causes & (kRageTooHard | kRageLongWait | kRageLostBall | kRageHitByBall)) != 0;
    }

    int32_t Golfer::toParSoFar() const
    {
        int32_t total = 0;
        for (size_t i = 0; i < scorecard.size(); i++)
            if (scorecard[i] != 127)
                total += scorecard[i];
        return total;
    }

    void Golfer::think(ThoughtType type, uint32_t tick)
    {
        // The same thought is not repeated while it is still the latest one.
        if (!thoughts.empty() && thoughts.back().type == type)
        {
            thoughts.back().tick = tick;
            return;
        }
        thoughts.push_back({ type, tick });
        constexpr size_t kMaxThoughts = 16;
        if (thoughts.size() > kMaxThoughts)
            thoughts.erase(thoughts.begin());
    }
} // namespace OpenRCT2::Golf
