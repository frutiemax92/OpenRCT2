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
#include "Shot.h"

#include <cstdint>
#include <vector>

namespace OpenRCT2::Golf
{
    // Tournaments (ECO-040 to ECO-043): the checklist a course must pass, scheduling, professional players, spectators,
    // revenue and the popularity bonus from the result.

    enum class TournamentTier : uint8_t
    {
        local,
        regional,
        national,
        count,
    };
    const char* tierName(TournamentTier tier);

    // ECO-040, ECO-043: what each tier asks for. One table.
    struct TierRequirements
    {
        int32_t holes;
        int32_t minYards;
        int32_t minPar;
        int32_t maxPar;
        int32_t minPopularity;
        int32_t minClubs;
        int32_t minParking;
        // ECO-041: the reserved infrastructure zones the player designates, in tiles.
        int32_t minSpectatorTiles;
        int32_t minMediaTiles;
        int32_t minExtraParkingTiles;
        int32_t minGrandstandTiles;
        int32_t maxGrandstandDistanceToLastGreen; // tiles
        // Rewards: ticket and sponsor money and the pull on spectators.
        int64_t sponsorMoney;
        int32_t spectatorsPerPopularityPoint; // per mille spectators per point of popularity
        int32_t popularityBonus;
    };
    const TierRequirements& requirementsFor(TournamentTier tier);

    struct TournamentInputs
    {
        int32_t holes = 0;           // playable holes of the course
        int32_t totalYards = 0;      // from the men's tees
        int32_t totalPar = 0;
        int32_t popularity = 0;
        int32_t golfClubs = 0;       // golf club buildings
        int32_t parkingCapacity = 0; // spaces of the regular parking lots
        int32_t spectatorTiles = 0;
        int32_t mediaTiles = 0;
        int32_t extraParkingTiles = 0;
        int32_t grandstandTiles = 0;
        int32_t grandstandDistanceToLastGreen = 1000;
    };

    enum class RequirementId : uint8_t
    {
        holes,
        length,
        par,
        popularity,
        golfClub,
        parking,
        spectatorZone,
        mediaZone,
        extraParkingZone,
        grandstandZone,
        grandstandNearGreen,
        count,
    };

    // One line of the checklist shown in the Tournament window.
    struct Requirement
    {
        RequirementId id;
        const char* label;
        int32_t current;
        int32_t required;   // for the par and the distance, the bound that matters (see the label)
        bool met;
    };
    std::vector<Requirement> checklist(TournamentTier tier, const TournamentInputs& inputs);
    bool canHost(TournamentTier tier, const TournamentInputs& inputs);

    // ECO-042: the state of a tournament from the booking to the result.
    enum class TournamentPhase : uint8_t
    {
        none,
        scheduled,
        running,
        finished,
    };

    struct Tournament
    {
        TournamentPhase phase = TournamentPhase::none;
        TournamentTier tier = TournamentTier::local;
        uint32_t day = 0;            // game day it takes place on
        int64_t ticketPrice = 0;
        int32_t professionals = 0;
        int32_t spectators = 0;
        int64_t revenue = 0;
        int32_t result = 0;          // 0..100 once finished
        int32_t popularityGained = 0;
    };

    // Books a tournament for a future day. Fails when it is not allowed to be hosted, the day is not in the future, or one
    // is already booked.
    enum class BookResult : uint8_t
    {
        ok,
        requirementsNotMet,
        dayInThePast,
        alreadyBooked,
        negativePrice,
    };
    BookResult book(Tournament& t, TournamentTier tier, const TournamentInputs& inputs, uint32_t today, uint32_t day, int64_t ticketPrice);
    void cancel(Tournament& t);
    // The day changes: a booked tournament starts on its day and ends the day after. Returns true when the phase changed.
    bool advanceDay(Tournament& t, uint32_t today);
    // ECO-042: on the day of the tournament the course is closed to regular golfers.
    bool courseClosedToPublic(const Tournament& t);

    // Professional golfers (ECO-042): skills near the top.
    constexpr int32_t kProfessionalFieldSize = 12;
    GolferSkills professionalSkills(TournamentTier tier, IBallRandom& rng);
    int32_t spectatorCount(TournamentTier tier, int32_t popularity);

    // The result of a tournament, 0..100, from how happy the players were, how many holes bottlenecked and how many rage
    // quit; and its revenue (tickets and sponsors).
    int32_t tournamentResult(int32_t averagePlayerHappiness, int32_t bottleneckHoles, int32_t rageQuits, int32_t holes);
    int64_t tournamentRevenue(const Tournament& t);
    // Applies the finished result: revenue, rating and popularity bonus (scaled by the result).
    void finish(Tournament& t, int32_t averagePlayerHappiness, int32_t bottleneckHoles, int32_t rageQuits, int32_t holes, int32_t popularity);
} // namespace OpenRCT2::Golf
